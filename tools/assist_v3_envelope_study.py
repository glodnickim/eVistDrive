#!/usr/bin/env python3
"""Assist Behavior V3 - design-time torque/power envelope and mode-character study.

Static (steady-state) model only. Transient behaviour (Response, Start, Carry) is the SIL matrix's job.

Anchor: the legacy G5300 surface dumped from PRODUCTION code by tests/host/tools/assist_v3_legacy_surface.c
(g53_static_target + kL/env_equiv of the intent module). This file re-implements the static map arithmetic in
Python (`legacy_iq`), proves the replica against that production CSV, and then generalises it into the V3 mode
character (MODE_CHARACTER.md):

  c4      = env * cad_eff * 35/10000 * 637/1000             (rider mechanical input, G5300 units)
  ratio   = clamp(base + slope * c4, rmin, rmax)            (G5300: fixed level ratio, or S+ AUTO r7 + (maxr-r7)*c4/den)
  conv    = c4 * ratio/100 * 1000 / LUT(cad_eff)            (cadence normalisation, unchanged)
  Iq      = min(conv,1000) * 0.455                          (0.65*P at P=700)
  cad_eff = max(cad, c_floor)  -> flat, torque-proportional support below the torque/power crossover c_floor
  then envelopes: Max Torque (Iq cap, % of 455) and Max Power (battery power via the existing g1 battery owner).

Physical conversions are ASSUMPTIONS, labelled in the report: crank torque 75 Nm at Iq 455 (M820 datasheet figure,
mapping to Iq unverified), drive efficiency 0.80, battery 48 V nominal, battery current owner limit 15 A
(BATTERYCURRENT_MAX).

Usage: python tools/assist_v3_envelope_study.py [--legacy CSV] [--out DIR]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LEGACY = ROOT / ".build/envelope_study/legacy_surface.csv"
OUT = ROOT / ".build/envelope_study"

IQ_MAX = 455.0                   # 0.65 * P, P = 700 (ARCHITECTURE_V3 5)
NM_AT_IQ_MAX = 75.0              # ASSUMPTION
ETA = 0.80                       # ASSUMPTION
V_BATT = 48.0                    # ASSUMPTION (nominal)
I_BATT_MAX = 15.0                # BATTERYCURRENT_MAX 15000 mA (inc/config.h)
P_BATT_MAX = V_BATT * I_BATT_MAX
CADENCES = (20, 25, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130)
LEVEL_RATIO = {1: 95, 2: 215, 3: 310, 4: 525, 5: 525}
HW_MAX_W = P_BATT_MAX                                    # "hardware maximum" profile default (D-034)   # HMI level -> slot 2/4/6/8/9 (g53_port_chain.c:2029)
AUTO = dict(r7=1, den=200, maxr=525)                     # S+ AUTO: D+64=1, scale 2 -> den 200 (chain.c:612-626)


def lut(x: float) -> float:
    xs = (10, 30, 40, 50, 60, 70); ys = (200, 400, 520, 620, 710, 740)
    if x <= xs[0]: return ys[0]
    if x >= xs[-1]: return ys[-1]
    for i in range(5):
        if xs[i] <= x < xs[i + 1]:
            return ys[i] + int((x - xs[i]) * (ys[i + 1] - ys[i]) / (xs[i + 1] - xs[i]))
    return 0.0


def legacy_iq(level: int, env: int, cad: int) -> float:
    """Integer-faithful replica of g53_static_target() steady state (ready, no taper, floor inert)."""
    x = env * 700 if cad <= 20 else env * cad * 35
    c2 = x // 10000
    c4 = (c2 * 637) // 1000
    if level == 4:
        maxr = LEVEL_RATIO[4]; r7 = AUTO["r7"]; den = AUTO["den"]
        ratio = maxr if maxr <= r7 else r7 + ((maxr - r7) * c4) // den
        ratio = min(ratio, maxr)
    else:
        ratio = LEVEL_RATIO[level]
    d4 = (c4 * ratio) // 100
    d6 = lut(cad)
    conv = (d4 * 1000) // int(d6)
    if (conv >> 3) > 124: conv = 1000
    q = ((conv << 12) * 274877907) >> 38
    target = 40960 if q & 61440 else (q * 5) << 1
    return target * IQ_MAX / 40960.0


# ----------------------------------------------------------------------------------------------- V3 model
@dataclass(frozen=True)
class Curve:
    """Internal parameter as a function of a BASIC macro m in 0..100: geometric between lo (m=0) and hi (m=100)."""
    lo: float
    hi: float

    def at(self, m: float) -> float:
        if self.lo <= 0 or self.hi <= 0:
            return self.lo + (self.hi - self.lo) * m / 100.0
        return self.lo * (self.hi / self.lo) ** (m / 100.0)


@dataclass(frozen=True)
class Profile:
    name: str
    ref_level: int
    assist_default: float
    base: Curve            # ratio at zero mechanical input
    slope: Curve           # ratio growth per c4 unit (progression)
    rmax: Curve            # ratio ceiling (assist range max)
    rmin: float            # ratio floor once engaged (assist range min)
    c_floor: float         # torque/power crossover cadence
    max_torque_pct: float  # default Max Torque
    max_power_w: float     # default Max Power (battery W)
    hc_bias: float = 0.0   # high-cadence support bias, -0.5..+0.5 (above 100 rpm)


def cad_eff(p: Profile, cad: float, hc_bias: float | None = None) -> float:
    c = max(cad, p.c_floor)
    b = p.hc_bias if hc_bias is None else hc_bias
    if c > 100:
        c = 100 + (c - 100) * (1.0 + b)
    return c


def v3_raw_iq(p: Profile, assist: float, env: float, cad: float, hc_bias: float | None = None,
              overrides: dict | None = None) -> float:
    """Desired support before envelopes, in the same integer order as the G5300 static map (the production V3
    support stage reuses g53_static_target with an injected ratio law). overrides: ADVANCED values that win over the
    macro derivation."""
    o = overrides or {}
    ce = int(round(cad_eff(p, cad, hc_bias)))
    env = int(env)
    x = env * 700 if ce <= 20 else env * ce * 35
    c2 = x // 10000
    c4 = (c2 * 637) // 1000
    base = o.get("base", p.base.at(assist))
    slope = o.get("slope", p.slope.at(assist))
    rmax = o.get("rmax", p.rmax.at(assist))
    rmin = o.get("rmin", p.rmin)
    ratio = int(min(max(base + slope * c4, rmin), rmax, 1000.0))
    d4 = (c4 * ratio) // 100
    conv = (d4 * 1000) // int(lut(ce))
    if (conv >> 3) > 124: conv = 1000
    q = ((conv << 12) * 274877907) >> 38
    target = 40960 if q & 61440 else (q * 5) << 1
    return target * IQ_MAX / 40960.0


def iq_power_cap(max_power_w: float, cad: float) -> float:
    """Battery power -> Iq ceiling at this cadence (mid-drive: motor speed follows the crank). ASSUMPTIONS above."""
    p = min(max_power_w, P_BATT_MAX) * ETA
    omega = max(cad, 1.0) * 2 * math.pi / 60
    return min(IQ_MAX, p / omega / NM_AT_IQ_MAX * IQ_MAX)


def v3_iq(p: Profile, assist: float, env: float, cad: float, max_torque=None, max_power=None, hc_bias=None,
          overrides=None) -> tuple[float, float, str]:
    raw = v3_raw_iq(p, assist, env, cad, hc_bias, overrides)
    tq = (p.max_torque_pct if max_torque is None else max_torque) * IQ_MAX / 100
    pw = iq_power_cap(p.max_power_w if max_power is None else max_power, cad)
    val = min(raw, tq, pw)
    binder = "none" if val >= raw - 1e-9 else ("torque" if tq <= pw else "power")
    return raw, val, binder


def profiles() -> dict[str, Profile]:
    """Candidate hidden profiles. Default Assist 50 reproduces the reference legacy level above c_floor."""
    def fixed(level, lo, hi):    # geometric around the legacy ratio: lo*hi == 1 keeps Assist 50 at the legacy ratio
        r = LEVEL_RATIO[level]
        return Curve(r * lo, r * hi)
    zero = Curve(0.0, 0.0)
    auto_slope = (AUTO["maxr"] - AUTO["r7"]) / AUTO["den"]
    return {
        # ECO: Assist acts on base support; calm; lower power; torque still useful; legacy L1 shape.
        "ECO": Profile("ECO", 1, 50, fixed(1, 0.5, 2.0), zero, Curve(1000, 1000), 0, 20, 80, 350),
        # TRAIL: flat torque support below 40 rpm (deliberate deviation from L2), strong range use.
        "TRAIL": Profile("TRAIL", 2, 50, fixed(2, 1 / 1.8, 1.8), zero, Curve(1000, 1000), 0, 40, 100, HW_MAX_W),
        # SPORT: legacy L3 above 30 rpm; reaches the envelope quickly.
        "SPORT": Profile("SPORT", 3, 50, fixed(3, 1 / 1.65, 1.65), zero, Curve(1000, 1000), 0, 30, 100, HW_MAX_W),
        # SPORT+: legacy S+ AUTO law; Assist moves progression (slope) and the ratio ceiling.
        "SPORT+": Profile("SPORT+", 4, 50, Curve(1, 1), Curve(auto_slope / 2.0, auto_slope * 2.0),
                          Curve(AUTO["maxr"] / 1.333, AUTO["maxr"] * 1.333), 0, 30, 100, HW_MAX_W),
        # BOOST: legacy L5 fixed 525 % (owner decision D-032).
        "BOOST": Profile("BOOST", 5, 50, fixed(5, 1 / 1.8, 1.8), zero, Curve(1000, 1000), 0, 30, 100, HW_MAX_W),
        # AUTO (Milestone F): static default point inside a TRAIL..SPORT+ range chosen by terrain state.
        "AUTO": Profile("AUTO", 2, 50, fixed(2, 1 / 1.8, 1.8), zero, Curve(1000, 1000), 0, 35, 100, HW_MAX_W),
    }


# ----------------------------------------------------------------------------------------------- checks
def load_surface(path: Path) -> list[dict]:
    return [{k: (float(v) if "." in v else int(v)) for k, v in r.items()} for r in csv.DictReader(open(path))]


def check_replica(rows, model=legacy_iq) -> dict:
    worst = 0.0
    for r in rows:
        worst = max(worst, abs(model(r["level"], r["env_equiv"], r["cadence_rpm"]) - r["iq_p700"]))
    return {"points": len(rows), "max_abs_iq_error": round(worst, 6), "pass": worst < 1e-6}


def check_default_parity(rows, profs, tol=0.03) -> dict:
    """At default Assist and before envelopes, V3 == legacy wherever cad >= c_floor (both below the 455 cap)."""
    out = {}
    for name, p in profs.items():
        if name == "AUTO":
            continue
        worst = 0.0; pts = 0
        for r in rows:
            if r["level"] != p.ref_level or r["cadence_rpm"] < p.c_floor or r["iq_p700"] < 2:
                continue
            raw = v3_raw_iq(p, p.assist_default, r["env_equiv"], r["cadence_rpm"])
            if r["iq_p700"] >= IQ_MAX - 0.5 and raw >= IQ_MAX - 0.5:
                continue
            worst = max(worst, abs(raw - r["iq_p700"])); pts += 1
        out[name] = {"points": pts, "max_abs_iq_error": round(worst, 3), "pass": pts > 0 and worst <= 1e-6}
    return out


def check_monotone_assist(rows, profs, model=v3_iq) -> dict:
    fails = []; pts = 0
    envs = sorted({r["env_equiv"] for r in rows if r["env_equiv"] > 0})
    for name, p in profs.items():
        for cad in (25, 60, 90, 120):
            for env in envs:
                prev_raw = prev_val = -1.0
                for a in range(0, 101, 5):
                    raw, val, _ = model(p, a, env, cad)
                    pts += 1
                    if raw < prev_raw - 1e-9: fails.append((name, cad, env, a, "raw"))
                    if val < prev_val - 1e-9: fails.append((name, cad, env, a, "enveloped"))
                    prev_raw, prev_val = raw, val
    return {"points": pts, "failures": len(fails), "first": fails[:3], "pass": not fails}


def crossover(p: Profile) -> float:
    """Cadence above which the default power cap is below the default torque cap."""
    tq = p.max_torque_pct * IQ_MAX / 100
    for c10 in range(100, 2001):
        if iq_power_cap(p.max_power_w, c10 / 10) < tq:
            return c10 / 10
    return 200.0


def check_envelope_regions(rows, profs, cap_fn=iq_power_cap) -> dict:
    """G2-ORTHO (static): lowering Max Power changes only points where the new power cap binds, lowering Max Torque
    only points where the new torque cap binds; reports the cadence band where each binds."""
    envs = sorted({r["env_equiv"] for r in rows if r["env_equiv"] > 0})
    res = {}
    for name, p in profs.items():
        leak_p = leak_t = 0; pw_b = []; tq_b = []
        for cad in CADENCES:
            for env in envs:
                raw, v0, _ = v3_iq(p, p.assist_default, env, cad)
                _, vp, _ = v3_iq(p, p.assist_default, env, cad, max_power=p.max_power_w * 0.6)
                _, vt, _ = v3_iq(p, p.assist_default, env, cad, max_torque=p.max_torque_pct * 0.6)
                cap_p = cap_fn(p.max_power_w * 0.6, cad)
                cap_t = p.max_torque_pct * 0.6 * IQ_MAX / 100
                if abs(vp - v0) > 1e-9 and v0 <= cap_p + 1e-9: leak_p += 1
                if abs(vt - v0) > 1e-9 and v0 <= cap_t + 1e-9: leak_t += 1
                if vp < v0 - 1e-9: pw_b.append(cad)
                if vt < v0 - 1e-9: tq_b.append(cad)
        res[name] = {"power_leaks": leak_p, "torque_leaks": leak_t,
                     "power_binds_rpm": [min(pw_b), max(pw_b)] if pw_b else None,
                     "torque_binds_rpm": [min(tq_b), max(tq_b)] if tq_b else None,
                     "crossover_rpm_at_defaults": crossover(p),
                     "pass": leak_p == 0 and leak_t == 0}
    return res


def effect_set(p, rows, **kw) -> set:
    envs = sorted({r["env_equiv"] for r in rows if r["env_equiv"] > 0})
    s = set(); i = 0
    for cad in CADENCES:
        for env in envs:
            _, v0, _ = v3_iq(p, p.assist_default, env, cad)
            _, v1, _ = v3_iq(p, kw.get("assist", p.assist_default), env, cad, kw.get("max_torque"),
                             kw.get("max_power"), kw.get("hc_bias"))
            if abs(v1 - v0) > 1e-6: s.add(i)
            i += 1
    return s


def check_basic_independence(rows, profs) -> dict:
    """Assist, Max Torque, Max Power must act on recognisably different regions of (effort x cadence). Jaccard
    overlap of the affected point sets; Max Torque vs Max Power must not be duplicates (< 0.9)."""
    out = {}
    n = len(CADENCES) * len({r["env_equiv"] for r in rows if r["env_equiv"] > 0})
    for name, p in profs.items():
        sets = {"assist": effect_set(p, rows, assist=80),
                "max_torque": effect_set(p, rows, max_torque=p.max_torque_pct * 0.6),
                "max_power": effect_set(p, rows, max_power=p.max_power_w * 0.6),
                "hc_bias": effect_set(p, rows, hc_bias=0.3)}
        jac = {}
        keys = sorted(sets)
        for i, a in enumerate(keys):
            for b in keys[i + 1:]:
                u = len(sets[a] | sets[b]) or 1
                jac[f"{a}~{b}"] = round(len(sets[a] & sets[b]) / u, 3)
        out[name] = {"affected_share": {k: round(len(s) / n, 3) for k, s in sets.items()}, "jaccard": jac,
                     "pass": jac["max_power~max_torque"] < 0.9}
    return out


def low_cadence_support(profs) -> dict:
    out = {}
    env = 600
    for name, p in profs.items():
        r60 = v3_raw_iq(p, p.assist_default, env, 60)
        out[name] = {f"{c}/60": round(v3_raw_iq(p, p.assist_default, env, c) / r60, 2) for c in (20, 25, 30, 40)}
    for lvl in (1, 2, 3):
        out[f"legacy_L{lvl}"] = {f"{c}/60": round(legacy_iq(lvl, env, c) / legacy_iq(lvl, env, 60), 2)
                                 for c in (20, 25, 30, 40)}
    return out


def mode_table(profs) -> list[dict]:
    rows = []
    for name, p in profs.items():
        for a in (0, 20, 50, 80, 100):
            row = {"mode": name, "assist": a}
            for env, cad in ((150, 25), (300, 25), (300, 60), (600, 60), (600, 90), (1000, 120)):
                row[f"env{env}@{cad}rpm"] = round(v3_iq(p, a, env, cad)[1])
            rows.append(row)
    return rows


def run(legacy_path: Path, out_dir: Path) -> dict:
    rows = load_surface(legacy_path)
    profs = profiles()
    res = {
        "replica_vs_production": check_replica(rows),
        "default_parity": check_default_parity(rows, profs),
        "monotone_assist": check_monotone_assist(rows, profs),
        "envelope_regions": check_envelope_regions(rows, profs),
        "basic_independence": check_basic_independence(rows, profs),
        "low_cadence_support": low_cadence_support(profs),
        "assumptions": {"nm_at_iq_max": NM_AT_IQ_MAX, "eta": ETA, "v_batt": V_BATT, "i_batt_max": I_BATT_MAX},
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "study.json").write_text(json.dumps(res, indent=2) + "\n")
    t = mode_table(profs)
    with open(out_dir / "mode_character.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(t[0].keys())); w.writeheader(); w.writerows(t)
    return res


def verdict(res: dict) -> bool:
    return (res["replica_vs_production"]["pass"] and res["monotone_assist"]["pass"]
            and all(x["pass"] for x in res["default_parity"].values())
            and all(x["pass"] for x in res["envelope_regions"].values())
            and all(x["pass"] for x in res["basic_independence"].values()))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--legacy", type=Path, default=LEGACY)
    ap.add_argument("--out", type=Path, default=OUT)
    a = ap.parse_args()
    res = run(a.legacy, a.out)
    print(json.dumps(res, indent=2))
    print("VERDICT:", "PASS" if verdict(res) else "FAIL")
    raise SystemExit(0 if verdict(res) else 1)


if __name__ == "__main__":
    main()
