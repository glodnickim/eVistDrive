# Assist Behavior V3 — Configuration Protocol (draft v2)

```text
STATUS:  DRAFT v2 — mode profile objects (owner override 2026-10-07); IDs pending registry allocation.
         v1 (schema_id 1, one 114 B block of 5 level records) is SUPERSEDED before any client used it; the B-CFG
         implementation (0ea5f56) carries v1 and is reworked to v2 (transport, staging, flash log and tests stay).
SOURCE:  audit/D_CONFIG_CAN.md §5 (transport, header, semantics); MODE_CHARACTER.md (parameter model)
CLIENTS: CANable (first), mobile app via HMI (later, cap bit 7), any HMI — all use this one contract
```

## 1. Parameter model (see MODE_CHARACTER.md)

Per mode (ECO, TRAIL, SPORT, SPORT+, AUTO) one **mode profile object** holds the configured value of every public
parameter: `0xFFFF` = DEFAULT (firmware profile default), any other value = user override. Storage keeps overrides
only. The firmware resolves the effective value (precedence: ADVANCED override > BASIC macro derivation > legacy P0/P1
input > profile default) and reports it with its source.

| ID | Param | Tier | Unit / range | Consumer milestone |
|---|---|---|---|---|
| 0 | assist | BASIC | 0..100 | mode character |
| 1 | max_torque | BASIC | 10..100 % rated | mode character (E) |
| 2 | max_power | BASIC | W, 100..max_power_hw | mode character (E) — legacy P1 % is the input while DEFAULT |
| 3 | response | BASIC | 0..100 | C (release); attack in mode character |
| 4 | start | BASIC | 0..100 | mode character |
| 5 | carry | BASIC | 0..100, 0 = off | D |
| 6..7 | reserved | — | `0xFFFF` | — |
| 8 | assist_base | ADVANCED | 0..400 % support | mode character |
| 9 | assist_range_min | ADVANCED | 0..400 % | mode character |
| 10 | assist_range_max | ADVANCED | 0..600 % | mode character |
| 11 | assist_progression | ADVANCED | 0..100 | mode character |
| 12 | attack | ADVANCED | 0..100 | mode character |
| 13 | release | ADVANCED | 0..100 | C |
| 14 | max_acceleration | ADVANCED (candidate) | 0..100 | pending matrix |
| 15 | phase_compensation | ADVANCED (candidate) | 0..100 | pending matrix |
| 16 | carry_strength | ADVANCED | 0..100 | D |
| 17 | carry_time_limit | ADVANCED | ms, 0..hard max | D |
| 18 | carry_distance_limit | ADVANCED | dm, 0..hard max | D |
| 19 | terrain_adaptation | ADVANCED | 0..100 | F |
| 20 | high_cadence_bias | ADVANCED (candidate) | −50..+50 (signed, offset 0x8000) | E |
| 21..23 | reserved | — | `0xFFFF` | — |

Global object: engine (0 G5300, 1 V3), level -> mode map (5 bytes, default L1 ECO, L2 TRAIL, L3 SPORT, L4 SPORT+,
L5 AUTO), reserved.

Rules:

- Assist, acceleration and power keep the stock M560 P0/P1 per-level values as **legacy inputs** (stock HMI app
  keeps working): when the V3 parameter of the same meaning is DEFAULT, the profile default is adjusted by the legacy
  value relative to its factory value; a V3 override wins. One effective value, documented precedence, source in
  readback (refines D-011).
- Internal algorithm constants are never on the wire.
- Inert legacy bank fields with the same meaning (`release_ms`, `max_iq_pct`, `support_ratio_pct`,
  `max_motor_power_w`, `iq_rise_fast_ms`, `smooth_start` in 0x6020/0x6021) are DEPRECATED; clients hide them.
  Max Torque reuses the existing `level_iq_limit` path in ap2_limits (one ceiling owner).
- A parameter without a consumer in this firmware has its `param_mask` bit 0: reads return `0xFFFF`, a write other than
  `0xFFFF` is rejected (reason 4). No parameter is ever accepted and silently ignored.

## 2. Transport (IDs pending allocation)

| ID | Op | Purpose |
|---|---|---|
| `0x6035` | READ | CAPS + STATUS (26 B multiframe) |
| `0x6036` | READ `[object, index, view]` | object 2 = mode profile (index = mode 1..5), object 3 = global; view 0 saved, 1 effective, 2 defaults, 3 configured (RAM) |
| `0x6036` | WRITE multiframe | apply one object to RAM; exactly one result frame |
| `0x6037` | WRITE short `[op, arg]` | 1 persist (deferred to standstill), 2 revert RAM to saved, 3 restore all V3 defaults (RAM), 4 restore one mode's defaults (arg = mode) |

- Result frame: NORMAL_ACK, or ERROR_ACK with `[reason, index]`: 0 schema/version, 1 length/stride, 2 CRC, 3 range,
  4 capability/reserved param, 5 busy/foreign owner, 6 stale generation, 7 reserved bits.
- V3 transfer: own staging buffer, hard bounds on frame index and declared length, owner = declaring source, foreign
  declaration rejected while a transfer is live, transfer timeout, source 5 only (source 3 behind cap bit 7).
- Before allocation: scan the stock M560/G5300 object dictionary and the BESST catalog for `0x6035..0x6037`
  (registry and BESST catalog: no use found by B-CFG; stock dictionary scan still [UNKNOWN]); record in
  COMMUNICATION_REGISTRY.

## 3. Layouts (little-endian)

CAPS/STATUS (26 B): [0..21] as audit D §5.2 (magic `BV`, format, protocol_version=2, schema_min/max, mode_count 5,
object mask, max_object_len, param_count 24, caps u32, param_mask u32 is carried in [16..19] — format 2 widens it
from u16, config_generation u16 (skips 0xFFFF), persist_state, flash_record_state), [22] engine_active,
[23] engine_requested, [24..25] CRC16-CCITT (0x1021, init 0xFFFF) over [0..23]. Exact offsets are fixed in the
implementation header and shared test vectors; readers use the declared length.

MODE PROFILE object (schema_id 2), write and views 0/2/3 (66 B):

```text
[0..15]   header: 'B','V', schema_id=2, schema_version=1, total_len u16=66, index=mode 1..5, param_count=24,
          stride=2, view/flags, caps u32 (write: required subset), generation u16 (write: base, 0xFFFF = no check)
[16..63]  24 x u16 configured values (0xFFFF = DEFAULT)
[64..65]  CRC16-CCITT over [0..63]
```

View 1 (effective, read only, 90 B): header, 24 x u16 effective values, 24 x u8 source codes
(0 profile default, 1 macro-derived, 2 legacy P0/P1 input, 3 user override, 4 limited by firmware, 0xFF reserved),
CRC16.

GLOBAL object (schema_id 3, 34 B): header, [16..17] engine u16, [18..22] level -> mode map, [23..31] reserved
(`0xFF`), [32..33] CRC16.

## 4. Semantics

- **Engine:** a write sets `engine_requested`; it becomes active only under the latch rule of ARCHITECTURE_V3 §2.2.
- **Write:** validate magic, schema_id, exact schema_version, total_len, index, param_count/stride, reserved values,
  required caps, generation, CRC, every range — then one commit in main-loop context between chain steps. Any failure
  = ERROR_ACK and zero mutation (staging discarded). Reject, never clamp.
- **Readback:** view 3 configured (RAM, DEFAULT shown as 0xFFFF), view 1 effective + source, view 0 persisted,
  view 2 profile defaults.
- **Restore:** op 3 sets every V3 override to DEFAULT; op 4 one mode. Both RAM only until persisted.
- **Persist:** never implicit. `0x6037 op 1` sets a flag consumed at standstill. Storage is an **append-only log** in
  flash page CONFIG_A `0x0803E800`: fixed-size slots holding the whole configuration (5 mode objects' configured values
  + global, ~260 B payload; slot size chosen by the implementation, ≥ 7 slots per page), newest valid CRC wins, erase
  only when full. Erase window: a power loss between page erase and the first program leaves "absent" -> defaults,
  never a corrupt configuration (tested). `MotorParams_t` untouched (sizeof 728).
- **CONFIG_A vs bootloader:** whether a BL820 update erases CONFIG_A is [UNKNOWN] — hardware marker check before
  relying on persist (RIDE_TEST_PLAN §0).
- **Restart:** valid record -> RAM; absent/corrupt -> defaults; newer schema -> defaults, flash untouched until the
  next explicit persist.
- **Migration:** a record with fewer params is back-filled with `0xFFFF`.
- **Discovery:** clients read `0x6035` first; no reply in the timeout = "V3 unsupported" (never "factory").

## 5. Profile defaults

Firmware-owned tables per mode (MODE_CHARACTER §3), chosen by the envelope simulation and the autonomous-default
matrix (SIMULATION_REPORT). No default is stored in flash or in a client.

## 6. Verification (TEST_MATRIX G1-CFG, G2-AUTONOMOUS, G2-OVERRIDE, G2-ORTHO, G2-MACRO)

Shared byte vectors; every reject reason with a no-mutation assertion; reserved-param rejection; generation
conflict; restore mode / restore all; configured vs effective vs source; persist -> restart -> readback;
corrupt/absent/newer record; erase window; P0/P1/P2, bank and tuning byte streams unchanged; CANable decoder ignores
the new IDs.

## 7. Known merge point

HMI-SRC3 (`e2868f6`) changes the same `processCAN_Rx` hunks and adds single-owner transfers. Merge order when it is
accepted: SRC3 first, V3 handlers on top of its owner gate.
