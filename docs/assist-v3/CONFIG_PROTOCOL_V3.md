# Assist Behavior V3 — Configuration Protocol (draft v2 rev 2)

```text
STATUS:  DRAFT v2 rev 2 — REVIEW-T issues resolved; independent re-check before the v2 rework is coded.
         v1 (schema_id 1, implemented in 0ea5f56) is SUPERSEDED. Until the v2 rework lands, the firmware keeps v1 and
         its per-level "response" field is the Milestone C release source (v1 defined it as "how quickly assist drops
         when you ease off"). Clients must NOT implement v1.
SOURCE:  audit/D_CONFIG_CAN.md §5 (transport); MODE_CHARACTER.md (model, precedence §6, applicability §5)
CLIENTS: CANable (first), mobile app via HMI (later, cap bit 7), any HMI — one contract
```

## 1. Parameter model

Per mode (ECO, TRAIL, SPORT, SPORT+, BOOST, AUTO) one **mode profile object** stores configured values:
`0xFFFF` = DEFAULT, any other value = override. Resolution, legacy P0/P1 inputs and precedence: MODE_CHARACTER §6
(the only copy of that rule). Applicability per mode: MODE_CHARACTER §5.

| ID | Param | Tier | Wire unit / range | Consumer |
|---|---|---|---|---|
| 0 | assist | BASIC | 0..100 | mode character |
| 1 | max_torque | BASIC | 10..100 % of the V3 demand full scale 0.65·P | mode character |
| 2 | max_power | BASIC | W, 50..65000; a value >= max_power_hw means "hardware maximum" (stored as written, effective = min(value, max_power_hw)) | mode character |
| 3 | response | BASIC | 0..100 | mode character (attack + release); masked in C |
| 4 | start | BASIC | 0..100 | mode character |
| 5 | carry | BASIC | 0..100, 0 = off | D |
| 6..7 | reserved | | `0xFFFF` | |
| 8 | assist_base | ADVANCED | ratio %, 0..1000 | mode character |
| 9 | assist_range_min | ADVANCED | ratio %, 0..1000 (AUTO only) | F |
| 10 | assist_range_max | ADVANCED | ratio %, 0..1000 (when progression > 0) | mode character |
| 11 | assist_progression | ADVANCED | 0..100 -> slope = p/100 · slope_max(mode) | mode character |
| 12 | attack | ADVANCED | 0..100 | mode character |
| 13 | release | ADVANCED | 0..100 | C |
| 14 | max_acceleration | ADVANCED candidate | 0..100 | pending SIL/L4 |
| 15 | phase_compensation | ADVANCED candidate | 0..100 | pending SIL |
| 16 | carry_strength | ADVANCED | 0..100 | D |
| 17 | carry_time_limit | ADVANCED | ms, 0..hard max | D |
| 18 | carry_distance_limit | ADVANCED | dm, 0..hard max | D |
| 19 | terrain_adaptation | ADVANCED | 0..100 (AUTO) | F |
| 20 | high_cadence_bias | ADVANCED | −50..+50, wire = value + 0x8000 | E |
| 21..23 | reserved | | `0xFFFF` | |

Sentinels: `0xFFFF` DEFAULT, `0xFFFE` KEEP (write only: leave the configured value unchanged; never stored).
Every profile value at Assist 0..100 lies inside its wire range.

Global object: engine (0 G5300, 1 V3), level -> mode map (5 bytes, default ECO, TRAIL, SPORT, SPORT+, BOOST), reserved.

Rules: internal algorithm constants are never on the wire; inert legacy bank fields with the same meaning
(`release_ms`, `max_iq_pct`, `support_ratio_pct`, `max_motor_power_w`, `iq_rise_fast_ms`, `smooth_start`) are
DEPRECATED and never feed V3; a parameter without a consumer has its `param_mask` bit 0 (reads `0xFFFF`, writes other
than DEFAULT/KEEP rejected, reason 4); an inapplicable parameter of a mode is reported with source 5 and writes other
than DEFAULT/KEEP to it are rejected (reason 4).

## 2. Transport (IDs pending allocation)

| ID | Op | Purpose |
|---|---|---|
| `0x6035` | READ | CAPS + STATUS (30 B, format 2) |
| `0x6036` | READ `[object, index, view, level]` | object 2 mode profile (index = mode 1..6), object 3 global; view 0 saved, 1 effective (needs `level` 1..5), 2 defaults, 3 configured |
| `0x6036` | WRITE multiframe | apply one object to RAM; exactly one result frame |
| `0x6037` | WRITE short `[op, arg, gen_lo, gen_hi]` | 1 persist (deferred to standstill; optional expected generation, 0xFFFF = none), 2 revert RAM to saved, 3 restore all V3 defaults, 4 restore one mode (arg = mode) |

- Result frame: NORMAL_ACK with the new `config_generation` (2 B), or ERROR_ACK `[reason, index]`: 0 schema/version,
  1 length/stride, 2 CRC, 3 range, 4 capability/reserved/not applicable, 5 busy/foreign owner, 6 stale generation,
  7 reserved bits.
- Transfer: own staging buffer, bounded frame index and declared length, owner = declaring source, foreign
  declaration rejected while a transfer is live, timeout, source 5 only (source 3 behind cap bit 7).
- IDs: registry and BESST catalog show no use (B-CFG); the stock M560/G5300 dictionary scan is still [UNKNOWN];
  record the allocation in COMMUNICATION_REGISTRY before any client ships.

## 3. Layouts (little-endian)

CAPS/STATUS, format 2 (30 B):

| Bytes | Field |
|---|---|
| 0..1 | magic `'B','V'` |
| 2 | format = 2 |
| 3 | protocol_version = 2 |
| 4 | schema_min (mode profile) |
| 5 | schema_max (mode profile) |
| 6 | mode_count = 6 |
| 7 | object mask (bit 2 mode profile, bit 3 global) |
| 8..9 | max_object_len |
| 10 | param_count = 24 |
| 11 | legacy_shadowed (bit n = level n+1) |
| 12..15 | caps u32 (bit 0 behaviour v2, 1 effective readback, 2 saved readback, 3 defaults view, 4 deferred persist, 5 revert, 6 restore mode, 7 accepts source 3) |
| 16..19 | param_mask u32 (bit = param ID) |
| 20..21 | config_generation (skips 0xFFFE/0xFFFF) |
| 22 | persist_state (0 clean, 1 RAM dirty, 2 pending standstill, 3 failed (flash), 4 rejected: stale generation) |
| 23 | flash_record_state (0 valid, 1 absent, 2 newer ignored, 3 CRC bad, 4 v1 record ignored) |
| 24 | engine_active (0 G5300, 1 V3) — truthful: what publishes Iq now |
| 25 | engine_requested |
| 26..27 | max_power_hw_w: hardware maximum battery power at the present pack voltage (15 A × V), for UI ranges |
| 28..29 | CRC16-CCITT (0x1021, init 0xFFFF) over 0..27 |

Format 1 (26 B, v1) is described in git history (0ea5f56); shared test vectors cover both formats.

MODE PROFILE object (schema_id 2), write and views 0/2/3 (66 B): header 16 B (`'B','V'`, schema_id 2, schema_version,
total_len u16, index (mode), param_count, stride 2, view/flags, caps u32 (write: required subset), generation u16
(write: base, 0xFFFF = no check)), 24 × u16 values, CRC16 over 0..63.
View 1 effective (90 B, read only): header with `level` in the flags byte, 24 × u16 effective values, 24 × u8 source
codes (MODE_CHARACTER §7), CRC16.

GLOBAL object (schema_id 3, 34 B): header, engine u16, level -> mode map (5 B), reserved `0xFF`, CRC16.

## 4. Semantics

- **Write:** validate magic, schema_id, schema_version within [schema_min, schema_max], total_len, index,
  param_count/stride, reserved values, applicability, required caps, generation, CRC, every range — then one commit in
  main-loop context between chain steps. Any failure = ERROR_ACK and zero mutation. Reject, never clamp.
- **Partial update:** `0xFFFE` keeps a value; a shorter `param_count` means "missing IDs = keep". A client that knows
  fewer parameters never wipes the others.
- **Engine:** a write sets `engine_requested`; it becomes active only under the latch rule (ARCHITECTURE_V3 §2.2).
  A firmware built without V3 rejects `engine = V3` (reason 4) and clears caps bit 0.
- **Readback:** view 3 configured, view 1 effective + source for one level, view 0 persisted (read from the CONFIG_A
  log, no RAM copy), view 2 profile defaults. The effective Max Power of a hardware-maximum default is the W at the
  present pack voltage (source 0); it moves with the pack voltage, like the legacy limit.
- **Applicability** is validated on the object as it will be after the write (e.g. range_max with progression > 0
  in the same write is accepted).
- **Restore:** op 3 all overrides -> DEFAULT; op 4 one mode. RAM only until persisted.
- **Persist:** never implicit. `0x6037 op 1` sets a flag; at the next standstill the RAM state **at that moment** is
  written (documented; the optional expected generation makes the persist fail if another client wrote in between).
  The immediate ACK only confirms queueing; the outcome is reported in `persist_state` (0 done, 3 flash failure,
  4 rejected: stale generation) and in the view 0 generation, which clients poll. STATUS reports the persisted generation via view 0. Storage: append-only log in CONFIG_A
  `0x0803E800`, slots holding the whole configuration (5+1 mode objects' configured values + global ≈ 300 B payload,
  ≥ 6 slots per page), newest valid CRC wins, erase only when full; erase window -> "absent" -> defaults (tested).
  `MotorParams_t` untouched (sizeof 728).
- **v1 records** (schema 1) found in CONFIG_A are treated as absent (state 4), flash untouched until the next explicit
  persist.
- **CONFIG_A vs bootloader:** whether a BL820 update erases CONFIG_A is [UNKNOWN] — hardware marker check
  (RIDE_TEST_PLAN §0).
- **Discovery:** clients read `0x6035` first; no reply = "V3 unsupported".

## 5. Profile defaults

Firmware-owned per mode (MODE_CHARACTER §3), chosen by the static study and the autonomous-default SIL matrix.

## 6. Verification (G1-CFG2, G2-AUTONOMOUS, G2-OVERRIDE, G2-ORTHO, G2-MACRO)

Shared byte vectors (CAPS format 1 and 2, objects); every reject reason with no-mutation assertion; KEEP sentinel and
short param_count; generation in ACK, stale generation, persist with expected generation; restore mode / all;
configured vs effective vs source per level, including legacy input, shadowed and not-applicable cases; effective
value of every parameter at Assist 0/50/100 inside its wire range; P0 write does not clear overrides; v1 record
ignored; persist -> restart -> readback; multiple objects then one persist; erase window; legacy P0/P1/P2, bank and
tuning byte streams unchanged; CANable decoder ignores the new IDs.

## 7. Known merge point

HMI-SRC3 (`e2868f6`) changes the same `processCAN_Rx` hunks; merge SRC3 first, V3 handlers on top of its owner gate.
