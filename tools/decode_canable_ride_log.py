#!/usr/bin/env python3
"""Decode FW-145 Level-4 telemetry from the raw text log written by EVistDrive CANable.

Input example (the existing CANable logger format):
  [08:52:10]\t[INFO]\t539158290483\tID:83106302\tDLC:5\tData:A7 00 60 4A 00

FW-145 telemetry is an extended-ID block 0x10400..0x10407. Seven data frames share the
same 16-bit 4 kHz control tick; META (0x10407) carries schema + a full 32-bit tick anchor.
The decoder writes:
  <prefix>.decoded.csv    rich internal-observation rows (partial snapshots retained)
  <prefix>.canonical.csv  replay input rows (only snapshots with CORE + STATE)
  <prefix>.metadata.json  loss/coverage/schema information

This tool never guesses missing CAN frames and never compresses time around them.

SIGN DOMAINS - READ BEFORE COMPARING iq_ref WITH iq_actual.
Schema 1 carries two conventions, and they are NOT interchangeable:

  iq_requested, iq_allowed, iq_ref   demand domain, POSITIVE when assisting
  iq_actual, id_actual               Park domain (MS.i_q / MS.i_d), where a drive configured
                                     MP.reverse = -1 makes forward drive NEGATIVE

Comparing them by sign therefore shows a near-total "inversion" that is only the convention -
it has already cost one wrong diagnosis. Compare MAGNITUDES, exactly as the firmware's own
src/rolling_no_assist_diag.c does. The controller's loop is consistent internally: it applies
the same MP.reverse factor to the PI setpoint (src/main.c, PI_iq.setpoint).

MP.reverse is not transmitted, so this tool cannot normalise the sign for you. It is left raw
on purpose: changing the meaning of a field without changing RIDE_TELEMETRY_SCHEMA_VERSION
would make old and new captures indistinguishable.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

BASE = 0x00010400
# Frame INDEX, which is also the identifier offset for 0..7. Schema 2 added two data frames
# ABOVE META rather than renumbering anything, so ASSIST and RIDER are 8 and 9 - the whole
# point being that every schema-1 frame keeps the identifier its decoder already knows.
CORE, DEMAND, MOTOR, BATT, LIMITS, STATE, ROTOR, META, ASSIST, RIDER = range(10)

# Data frames per schema version. Frame 7 is META in both and is never a data frame.
DATA_FRAMES_V1 = (CORE, DEMAND, MOTOR, BATT, LIMITS, STATE, ROTOR)
DATA_FRAMES_V2 = DATA_FRAMES_V1 + (ASSIST, RIDER)
SUPPORTED_SCHEMAS = (1, 2)
CONTROL_HZ = 4000.0

# Which stage of the one limiter chain was binding. Schema 2 only: the low eight bits travel in
# the STATE frame's spare byte, which schema 1 left zero.
LIMIT_FLAG_NAMES = [
    (0, "lim_power"), (1, "lim_battery"), (2, "lim_phase"), (3, "lim_voltage"),
    (4, "lim_thermal"), (5, "lim_speed"), (6, "lim_zeroed"), (7, "lim_start"),
]

LINE_RE = re.compile(
    r"^\[(?P<wall>[^\]]+)\]\s+\[[^\]]+\]\s+"
    r"(?P<mono>\d+)\s+ID:(?P<id>[0-9A-Fa-f]+)\s+"
    r"DLC:(?P<dlc>\d+)\s+Data:(?P<data>.*)$"
)

FLAG_NAMES = [
    (0, "battery_limit"), (1, "brake"), (2, "walk"), (3, "torque_fault"),
    (4, "overtemp_cut"), (5, "foc_saturated"), (6, "torque_cal"),
    (7, "offroad"), (8, "pwm_on_flag"), (9, "start_phase_flag"),
    (10, "walk_can_req"), (11, "direction_inhibit_flag"), (12, "backpedal_flag"),
]

DECODED_FIELDS = [
    "time_s", "capture_time_s", "control_tick", "tick16", "frame_mask", "complete",
    "schema_version",
    "load_centikg",
    # SCHEMA 1 ONLY - the removed native torque filters. Left as distinct column names rather
    # than reused, so a mixed archive never silently compares a native ADC delta against a
    # permille demand.
    "torque_fast_native", "torque_run_native",
    # SCHEMA 2 - the demand model, in permille of the rider-effort full scale.
    "torque_normalized_permille", "rider_demand_permille",
    "assist_base_permille", "assist_dynamic_permille", "assist_response_permille",
    "rider_aggression_permille", "load_state_permille", "auto_factor_permille",
    "limit_flags_hex", *[name for _, name in LIMIT_FLAG_NAMES],
    "cadence_raw_rpm", "cadence_control_rpm",
    "iq_requested", "iq_allowed", "iq_ref", "iq_actual", "id_actual", "motor_erps",
    "battery_voltage_v", "battery_current_a", "soc_display_pct",
    "wheel_speed_kph", "u_abs", "flags_hex",
    *[name for _, name in FLAG_NAMES], "bridge_lifecycle_flags",
    "permission_bits_hex", "debug_flags_hex", "assist_level", "session_state", "qzero_state",
    "theta_q15", "hall_age_ticks", "hall_state", "rotor_trusted", "bridge_lifecycle_rotor", "pwm_on",
    "pas_ab_snapshot", "pas_direction_state", "pas_backpedal", "direction_inhibit", "start_phase",
]

CANONICAL_FIELDS = [
    "time_s", "cadence_rpm", "torque_raw_native", "torque_ckg",
    "wheel_speed_kph", "battery_voltage_v", "battery_current_a",
    "assist_level", "brake", "walk", "motor_erps", "iq_actual",
    "pas_ab", "pas_direction", "recorded_iq_request", "recorded_iq_ref",
]


def u16(d: bytes, off: int) -> int:
    return (d[off] << 8) | d[off + 1]


def i16(d: bytes, off: int) -> int:
    v = u16(d, off)
    return v - 0x10000 if v & 0x8000 else v


def u32(d: bytes, off: int) -> int:
    return (d[off] << 24) | (d[off + 1] << 16) | (d[off + 2] << 8) | d[off + 3]


def fmt(v, digits: int = 6) -> str:
    if v is None:
        return "nan"
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        return format(v, f".{digits}f") if math.isfinite(v) else "nan"
    return str(v)


@dataclass
class Frame:
    capture_tick: int
    can_id: int
    dlc: int
    data: bytes


@dataclass
class Snapshot:
    tick: int
    tick16: int
    capture_first: int
    capture_last: int
    frames: dict[int, bytes] = field(default_factory=dict)
    # The schema this snapshot is read under. A capture that starts before its first META frame
    # has no version yet; see resolve_schema() for why that is inferred rather than assumed.
    schema: int = 1

    @property
    def expected_frames(self) -> tuple:
        return DATA_FRAMES_V2 if self.schema >= 2 else DATA_FRAMES_V1

    @property
    def mask(self) -> int:
        m = 0
        for idx in self.frames:
            if idx != META and 0 <= idx < RIDER + 1:
                m |= 1 << idx
        return m

    @property
    def complete(self) -> bool:
        want = 0
        for idx in self.expected_frames:
            want |= 1 << idx
        return (self.mask & want) == want

    @property
    def replayable(self) -> bool:
        # Cadence/assist policy lives in STATE; physical load lives in CORE.
        return CORE in self.frames and STATE in self.frames


def iter_canable(path: Path) -> Iterable[Frame]:
    with path.open("r", encoding="utf-8-sig", errors="replace") as f:
        for raw in f:
            m = LINE_RE.match(raw.strip())
            if not m:
                continue
            raw_id = int(m.group("id"), 16)
            # CANable logs the driver can_id verbatim, so an extended frame keeps
            # CAN_EFF_FLAG and RTR/ERR frames set 0x60000000. Mask exactly like the rest
            # of the project does before comparing against the telemetry ID block.
            if raw_id & 0x60000000:
                continue
            can_id = raw_id & 0x1FFFFFFF
            dlc = int(m.group("dlc"))
            tokens = [x for x in m.group("data").strip().split() if x]
            try:
                data = bytes(int(x, 16) for x in tokens)
            except ValueError:
                continue
            if len(data) < dlc:
                continue
            yield Frame(int(m.group("mono")), can_id, dlc, data[:dlc])


class TickUnwrapper:
    def __init__(self, capture_hz: float) -> None:
        self.last: int | None = None
        self.last_capture: int | None = None
        self.capture_hz = capture_hz

    def candidate(self, low: int, capture_tick: int) -> int:
        if self.last is None or self.last_capture is None:
            return low
        dt = max(0.0, (capture_tick - self.last_capture) / self.capture_hz)
        predicted = self.last + int(round(dt * CONTROL_HZ))
        value = (predicted & ~0xFFFF) | low
        while value - predicted > 0x8000:
            value -= 0x10000
        while predicted - value > 0x8000:
            value += 0x10000
        return value

    def unwrap(self, low: int, capture_tick: int) -> int:
        value = self.candidate(low, capture_tick)
        # CANable's monotonic timestamp disambiguates long (>8 s) telemetry gaps. A tiny negative
        # result can only be bus reordering; keep one monotonic snapshot timeline for replay.
        if self.last is not None and value < self.last:
            while value < self.last:
                value += 0x10000
        self.last = value
        self.last_capture = capture_tick
        return value


def decode_row(s: Snapshot, t0: int, capture0: int, capture_hz: float) -> dict[str, object]:
    row: dict[str, object] = {k: None for k in DECODED_FIELDS}
    row.update({
        "time_s": (s.tick - t0) / CONTROL_HZ,
        "capture_time_s": (s.capture_first - capture0) / capture_hz,
        "control_tick": s.tick,
        "tick16": s.tick16,
        "frame_mask": f"0x{s.mask:02X}",
        "complete": s.complete,
    })
    row["schema_version"] = s.schema
    if CORE in s.frames:
        d=s.frames[CORE]
        row.update(load_centikg=u16(d,2))
        if s.schema >= 2:
            # SCHEMA 2 redefined these two words. Decoding them under their schema-1 names would
            # report a permille demand as a native ADC delta - the exact confusion the version
            # byte exists to prevent.
            row.update(torque_normalized_permille=u16(d,4), rider_demand_permille=u16(d,6))
        else:
            row.update(torque_fast_native=u16(d,4), torque_run_native=u16(d,6))
    if DEMAND in s.frames:
        d=s.frames[DEMAND]; row.update(iq_requested=i16(d,2), iq_allowed=i16(d,4), iq_ref=i16(d,6))
    if MOTOR in s.frames:
        d=s.frames[MOTOR]; row.update(iq_actual=i16(d,2), id_actual=i16(d,4), motor_erps=u16(d,6))
    if BATT in s.frames:
        d=s.frames[BATT]; row.update(battery_voltage_v=u16(d,2)/100.0, battery_current_a=i16(d,4)/100.0, soc_display_pct=u16(d,6)/10.0)
    if LIMITS in s.frames:
        d=s.frames[LIMITS]; flags=u16(d,6)
        row.update(wheel_speed_kph=u16(d,2)/100.0, u_abs=u16(d,4), flags_hex=f"0x{flags:04X}", bridge_lifecycle_flags=(flags>>13)&7)
        for bit,name in FLAG_NAMES: row[name]=bool(flags & (1<<bit))
    if STATE in s.frames:
        d=s.frames[STATE]; packed=d[4]
        row.update(permission_bits_hex=f"0x{d[2]:02X}", debug_flags_hex=f"0x{d[3]:02X}",
                   assist_level=packed&0x0F, session_state=(packed>>4)&3, qzero_state=(packed>>6)&3,
                   cadence_raw_rpm=d[5], cadence_control_rpm=d[6])
        if s.schema >= 2:
            lim = d[7]
            row["limit_flags_hex"] = f"0x{lim:02X}"
            for bit, name in LIMIT_FLAG_NAMES:
                row[name] = bool(lim & (1 << bit))
    if ASSIST in s.frames:
        d=s.frames[ASSIST]
        row.update(assist_base_permille=u16(d,2), assist_dynamic_permille=u16(d,4),
                   assist_response_permille=u16(d,6))
    if RIDER in s.frames:
        d=s.frames[RIDER]
        row.update(rider_aggression_permille=u16(d,2), load_state_permille=u16(d,4),
                   auto_factor_permille=u16(d,6))
    if ROTOR in s.frames:
        d=s.frames[ROTOR]; rr=d[6]; pp=d[7]
        row.update(theta_q15=i16(d,2), hall_age_ticks=u16(d,4), hall_state=rr&7,
                   rotor_trusted=bool((rr>>3)&1), bridge_lifecycle_rotor=(rr>>4)&7, pwm_on=bool((rr>>7)&1),
                   pas_ab_snapshot=pp&3, pas_direction_state=(pp>>2)&3, pas_backpedal=bool((pp>>4)&1),
                   direction_inhibit=bool((pp>>5)&1), start_phase=bool((pp>>6)&1))
    return row


def canonical_row(row: dict[str, object]) -> dict[str, str]:
    cad = row.get("cadence_control_rpm")
    back = bool(row.get("pas_backpedal"))
    inhibit = bool(row.get("direction_inhibit"))
    # A 47 Hz PAS A/B snapshot is NOT a raw quadrature event stream, so pas_ab stays nan. Direction
    # is retained only as the coarse physical/safety fact needed by the current replay harness.
    if back:
        direction = -1
    elif inhibit:
        direction = 0
    elif isinstance(cad, int) and cad > 0:
        direction = 1
    else:
        direction = 0
    return {
        "time_s": fmt(row.get("time_s"),9),
        "cadence_rpm": fmt(cad),
        "torque_raw_native": "nan",
        "torque_ckg": fmt(row.get("load_centikg")),
        "wheel_speed_kph": fmt(row.get("wheel_speed_kph")),
        "battery_voltage_v": fmt(row.get("battery_voltage_v")),
        "battery_current_a": fmt(row.get("battery_current_a")),
        "assist_level": fmt(row.get("assist_level")),
        "brake": fmt(row.get("brake")),
        "walk": fmt(row.get("walk")),
        "motor_erps": fmt(row.get("motor_erps")),
        "iq_actual": fmt(row.get("iq_actual")),
        "pas_ab": "nan",
        "pas_direction": str(direction),
        "recorded_iq_request": fmt(row.get("iq_requested")),
        "recorded_iq_ref": fmt(row.get("iq_ref")),
    }


def main() -> int:
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input", type=Path)
    ap.add_argument("--output-prefix", type=Path, help="path without suffix; default: input stem")
    ap.add_argument("--capture-hz", type=float, default=1_000_000.0,
                    help="frequency of CANable monotonic timestamp field (default 1 MHz)")
    args=ap.parse_args()
    if args.capture_hz <= 0:
        raise SystemExit("--capture-hz must be >0")
    prefix=args.output_prefix or args.input.with_suffix("")
    decoded=Path(str(prefix)+".decoded.csv")
    canonical=Path(str(prefix)+".canonical.csv")
    metadata=Path(str(prefix)+".metadata.json")
    for p in (decoded,canonical,metadata): p.parent.mkdir(parents=True,exist_ok=True)

    total_frames=0; telem_frames=0; malformed_telem=0; meta_records=[]
    unwrap=TickUnwrapper(args.capture_hz); snaps: dict[int,Snapshot]={}
    meta_epoch_offsets=[]
    capture_first_all=None; capture_first_telem=None
    for fr in iter_canable(args.input):
        total_frames += 1
        if capture_first_all is None: capture_first_all=fr.capture_tick
        if not (BASE <= fr.can_id <= BASE+RIDER):
            continue
        telem_frames += 1
        if capture_first_telem is None: capture_first_telem=fr.capture_tick
        idx=fr.can_id-BASE
        if fr.dlc != 8 or len(fr.data) != 8:
            malformed_telem += 1; continue
        if idx == META:
            schema=fr.data[0]; bank=fr.data[1]; full=u32(fr.data,2)
            pseudo_meta=unwrap.candidate(full & 0xFFFF, fr.capture_tick)
            meta_epoch_offsets.append(full - pseudo_meta)
            meta_records.append({"capture_tick":fr.capture_tick,"schema_version":schema,"active_profile_bank":bank,
                                 "full_control_tick":full,"failed_frames":fr.data[6],"data_frame_count":fr.data[7]})
            continue
        low=u16(fr.data,0); full=unwrap.unwrap(low, fr.capture_tick)
        s=snaps.get(full)
        if s is None:
            s=Snapshot(full,low,fr.capture_tick,fr.capture_tick); snaps[full]=s
        s.capture_last=max(s.capture_last,fr.capture_tick); s.capture_first=min(s.capture_first,fr.capture_tick)
        # Duplicate same fragment for a snapshot is retained as a countable anomaly, but the latest
        # bytes win so a logger duplicate cannot create a second virtual-time row.
        s.frames[idx]=fr.data

    ordered=sorted(snaps.values(),key=lambda x:x.tick)

    # ---- schema resolution -----------------------------------------------------------------
    #
    # META carries the version and is sent at most once a second, so a capture that starts
    # mid-stream has data frames before its first META. Guessing a version for those would be
    # exactly the failure this whole mechanism exists to prevent, so they are not guessed at:
    # every snapshot is read under the version of the NEAREST PRECEDING META, and snapshots
    # before the first META take the version of the first one in the capture. That is an
    # inference, and it is recorded as one - `schema_before_first_meta` in the metadata says how
    # many rows were read that way, so a reader can discount them if the capture is a mixture.
    meta_schemas = [m["schema_version"] for m in meta_records]
    first_schema = meta_schemas[0] if meta_schemas else 1
    meta_by_capture = sorted(
        ((m["capture_tick"], m["schema_version"]) for m in meta_records), key=lambda x: x[0])
    rows_before_first_meta = 0
    for snap in ordered:
        chosen = first_schema
        seen_meta = False
        for cap, ver in meta_by_capture:
            if cap <= snap.capture_first:
                chosen = ver
                seen_meta = True
            else:
                break
        if not seen_meta:
            rows_before_first_meta += 1
        snap.schema = chosen

    epoch_offset = 0
    if meta_epoch_offsets:
        # Every valid offset is an integer number of 16-bit wraps. Pick the most common anchor;
        # disagreements remain visible in metadata instead of silently changing replay time.
        counts={}
        for off in meta_epoch_offsets: counts[off]=counts.get(off,0)+1
        epoch_offset=max(counts, key=counts.get)
        for snap in ordered: snap.tick += epoch_offset
    if not ordered:
        meta={"input":str(args.input),"source_sha256":hashlib.sha256(args.input.read_bytes()).hexdigest(),
              "total_can_frames":total_frames,"telemetry_frames":telem_frames,"snapshots":0,
              "replayable_snapshots":0,"complete_snapshots":0,"malformed_telemetry_frames":malformed_telem,
              "schema_versions":sorted({m['schema_version'] for m in meta_records}),"meta_records":meta_records,
              "coverage":[],"meta_epoch_offsets":meta_epoch_offsets,"status":"NO_FW145_TELEMETRY"}
        metadata.write_text(json.dumps(meta,indent=2)+"\n")
        print(f"DECODE NO TELEMETRY frames={total_frames} telemetry={telem_frames} metadata={metadata}")
        return 3

    t0=ordered[0].tick; c0=capture_first_telem if capture_first_telem is not None else ordered[0].capture_first
    decoded_rows=[decode_row(s,t0,c0,args.capture_hz) for s in ordered]
    replay_rows=[canonical_row(r) for s,r in zip(ordered,decoded_rows) if s.replayable]

    with decoded.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=DECODED_FIELDS); w.writeheader()
        for r in decoded_rows: w.writerow({k:fmt(r.get(k)) for k in DECODED_FIELDS})
    with canonical.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=CANONICAL_FIELDS); w.writeheader(); w.writerows(replay_rows)

    complete=sum(s.complete for s in ordered); replayable=sum(s.replayable for s in ordered)
    missing_counts={str(i):sum(i not in s.expected_frames or i not in s.frames for s in ordered)
                    for i in range(RIDER + 1) if i != META}
    schemas=sorted({m["schema_version"] for m in meta_records})
    bad_schema=[v for v in schemas if v not in SUPPORTED_SCHEMAS]
    # Coverage is deliberately conservative: PAS_AB here is only a sparse snapshot, not PAS_RAW.
    coverage=["CORE","INTERNAL"] if replayable else []
    if any(BATT in s.frames and LIMITS in s.frames for s in ordered): coverage.append("LIMITS")
    if any(MOTOR in s.frames for s in ordered): coverage.append("MOTOR")
    if any(ROTOR in s.frames for s in ordered): coverage.append("ROTOR_PAS_STATE")
    gaps=[]
    prev=None
    for s in ordered:
        if prev is not None:
            dt=(s.tick-prev.tick)/CONTROL_HZ
            if dt > 0.100: gaps.append({"after_control_tick":prev.tick,"gap_s":dt})
        prev=s
    meta={
        "input":str(args.input),"decoded":str(decoded),"canonical":str(canonical),
        "source_sha256":hashlib.sha256(args.input.read_bytes()).hexdigest(),
        "total_can_frames":total_frames,"telemetry_frames":telem_frames,"malformed_telemetry_frames":malformed_telem,
        "snapshots":len(ordered),"complete_snapshots":complete,"replayable_snapshots":replayable,
        "snapshot_complete_ratio":complete/len(ordered),"missing_frame_counts":missing_counts,
        "schema_versions":schemas,"supported_schema_versions":list(SUPPORTED_SCHEMAS),
        "unsupported_schema_versions":bad_schema,
        "schema_before_first_meta":rows_before_first_meta,"meta_records":meta_records,
        "meta_epoch_offsets":meta_epoch_offsets,"selected_epoch_offset":epoch_offset,
        "coverage":coverage,"time_source":"firmware_control_tick_4khz","capture_timestamp_hz":args.capture_hz,
        "long_gaps_over_100ms":gaps,
        "pas_note":"0x10406 contains a sparse PAS state snapshot, not a raw transition stream; canonical pas_ab is intentionally nan",
        "status":"PASS" if replayable and not bad_schema else ("UNSUPPORTED_SCHEMA" if bad_schema else "INSUFFICIENT_CORE_STATE"),
    }
    metadata.write_text(json.dumps(meta,indent=2)+"\n")
    print(f"DECODE {meta['status']} can={total_frames} telem={telem_frames} snapshots={len(ordered)} "
          f"complete={complete} replayable={replayable} coverage={'+'.join(coverage)}")
    print(f"decoded={decoded}\ncanonical={canonical}\nmetadata={metadata}")
    return 0 if meta["status"] == "PASS" else 4

if __name__ == "__main__":
    raise SystemExit(main())
