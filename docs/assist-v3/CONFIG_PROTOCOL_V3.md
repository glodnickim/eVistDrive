# Assist Behavior V3 — Configuration Protocol (draft v1)

```text
STATUS:  DRAFT — frozen with ARCHITECTURE_V3 at REVIEW 1; IDs pending registry allocation
SOURCE:  audit/D_CONFIG_CAN.md §5 (transport, header, semantics), this file fixes the parameter model
CLIENTS: CANable (first), mobile app via HMI (later, cap bit 7), any HMI — all use this one contract
```

## 1. One user model, one owner per parameter

| User parameter | Meaning for the rider | Owner / carrier | Unit, range | Consumer in C | Milestone |
|---|---|---|---|---|---|
| Assist | how much the motor adds to your effort | existing M560 P0 assist ratio per level (0x6010) | %, as today | G53 static map (V3-4) | existing |
| Acceleration | how fast assist builds up | existing M560 P0 accel per level (0x6010) | 1..8, as today | V3-6 rise rate | existing |
| Max Power | power ceiling | existing M560 P1 power % per level (0x6011) | %, as today | g1 limit | existing |
| Response | how quickly assist drops when you ease off | **V3 block** level record [0] | 0..100 % | V3-3 / V3-6 release rate | C |
| Start Response | how strongly assist starts from rest | **V3 block** level record [1] | 0..100 % | none yet (mask bit off) | later |
| Carry strength | how much push continues when you stop pedalling under load | **V3 block** level record [2] | 0..100 % (0 = off) | V3-5 carry | D |
| Carry extent | how long/far that push may last | **V3 block** level record [3] | 0..100 % of the hard bounds | V3-5 carry caps | D |
| Max Torque | torque ceiling | **V3 block** level record [4] | 10..100 % rated | envelope | E |
| Assistance Range | dynamic assist window for AUTO | **V3 block** level record [5] | 0..100 % | terrain/AUTO | F |
| Engine | G5300 behaviour or V3 behaviour | **V3 block** global record [0] | 0 = G5300, 1 = V3 | pipeline selector | C |

Rules:

- The three existing classes keep their M560 owner (stock-compatible, also written by the HMI app). The V3 block does
  **not** mirror them — a mirror would be a second owner. Clients present one model and write each parameter to its
  owner.
- Internal algorithm constants (template bins, α, windows, thresholds, hard bounds) are not on the wire.
- Inert legacy bank fields with the same meaning (`release_ms`, `max_iq_pct`, `support_ratio_pct`,
  `max_motor_power_w`, `iq_rise_fast_ms`, `smooth_start` in 0x6020/0x6021) are DEPRECATED / inert in both engines;
  clients hide them. Milestone E Max Torque reuses the existing `level_iq_limit` path in ap2_limits (one ceiling
  owner), not a new ceiling.
- A parameter exists on the wire before its consumer only as "reserved": its `param_mask` bit is 0, reads return
  `0xFFFF`, and a write of any value other than `0xFFFF` is rejected (reason 4). The bit flips to 1 in the firmware that
  adds the consumer. No parameter is ever accepted and silently ignored.

## 2. Transport (IDs pending allocation)

| ID | Op | Purpose |
|---|---|---|
| `0x6035` | READ | CAPS + STATUS (26 B multiframe) |
| `0x6036` | READ `[view, set]` | BEHAVIOR block: view 0 saved, 1 effective, 2 firmware defaults |
| `0x6036` | WRITE multiframe | apply BEHAVIOR block to RAM; exactly one result frame |
| `0x6037` | WRITE short `[op]` | 1 persist (deferred to standstill), 2 revert RAM to saved, 3 load defaults to RAM |

- Result frame: NORMAL_ACK, or ERROR_ACK with `[reason, index]`: 0 schema/version, 1 length/stride, 2 CRC, 3 range,
  4 capability/reserved param, 5 busy/foreign owner, 6 stale generation, 7 reserved bits.
- V3 transfer: own staging buffer, hard bounds on frame index and declared length, owner = declaring source, foreign
  declaration rejected while a transfer is live, transfer timeout, source 5 only (source 3 behind cap bit 7).
- Before allocation: scan the stock M560/G5300 object dictionary and the BESST catalog
  (`integration/evidence/besst-pro-can-catalog-20261006`) for `0x6035..0x6037`; record in COMMUNICATION_REGISTRY.

## 3. Layouts (little-endian)

CAPS/STATUS (24 B) — as audit D §5.2: magic `BV`, format, protocol_version, schema_min/max, level_count 5, set_count 1,
max_block_len, record_stride 16, param_count, caps u32, param_mask u16, config_generation u16 (skips 0xFFFF on
increment), persist_state, flash_record_state, engine_active u8, engine_requested u8, CRC16-CCITT (0x1021,
init 0xFFFF). Exact offsets: [0..21] as audit D §5.2, [22] engine_active, [23] engine_requested, [24..25] CRC16 over [0..23];
total 26 B (format 1). Readers use the declared length.

`param_mask` bits: 0 response, 1 start_response, 2 carry_strength, 3 carry_extent, 4 max_torque, 5 assist_range,
15 engine. Milestone C firmware: bits 0 and 15.

BEHAVIOR block (114 B):

```text
[0..15]   header: 'B','V', schema_id=1, schema_version=1, total_len u16=114, set_index=0, level_count=5,
          record_stride=16, flags=0, caps u32 (write: required subset), generation u16 (write: base, 0xFFFF = no check)
[16..31]  global record: [0] engine u16, [1..7] reserved = 0xFFFF
[32..111] level records 1..5 (HMI level numbers, not M560 slots): 8 x u16
          [0] response  [1] start_response  [2] carry_strength  [3] carry_extent
          [4] max_torque  [5] assist_range  [6..7] reserved = 0xFFFF
[112..113] CRC16-CCITT over [0..111]
```

`0xFFFF` in any value = "use the firmware default for this firmware version".

## 4. Semantics

- **Engine:** a write of `engine` sets `engine_requested`; it becomes active only under the latch rule of
  ARCHITECTURE_V3 §2.2 (never immediately while riding).
- **Write:** validate magic, schema_id, exact schema_version, total_len, stride, level_count, reserved values,
  required caps, generation, CRC, every range — then one commit in main-loop context between chain steps. Any failure
  = ERROR_ACK and zero mutation (staging discarded). Reject, never clamp.
- **Readback:** view 1 = values in force after firmware limits; view 0 = last persisted; view 2 = compiled defaults.
- **Persist:** never implicit. `0x6037 op 1` sets a flag consumed at standstill. Storage is an **append-only log**
  in flash page CONFIG_A `0x0803E800` (reserved in the linker script, unused by code): fixed-size slots (record =
  magic, schema_id, schema_version, stride, payload, generation, crc32 written last), the newest slot with a valid CRC
  wins, the page is erased only when full and only after the new record is ready to be written first in the fresh
  page. Between page erase and the first program of the fresh page there is a short window (one 2 KB erase plus one
  record write) in which a power loss leaves no valid record; the result is "absent" -> defaults, never a corrupt
  configuration. Only one free page exists (CONFIG_B holds MotorParams_t, SOC has its own page), so a two-page scheme
  is not available; erases happen once per ~15 persists and only at standstill. The erase window is tested
  (REVIEW 1 #11, re-check N6).
  `MotorParams_t` is not touched (sizeof stays 728).
- **CONFIG_A vs bootloader:** the linked image cannot overlap CONFIG_A (linker ASSERT + build_firmware check), but
  the BL820 container header carries only size mod 64 KiB, so whether an update erases CONFIG_A is [UNKNOWN].
  Hardware check before relying on persist: write a marker record, perform a BL820 update, read back. If it is wiped,
  either move the log after the CONFIG_B footer (audit D §5.3) or accept "defaults after update" as an owner decision.
- **Restart:** valid record -> RAM; absent/corrupt -> defaults (`flash_record_state` 1/3); newer schema than firmware ->
  defaults, flash left untouched until the next explicit persist (state 2). If a bootloader update erases CONFIG_A
  ([UNKNOWN]), the result is "absent" -> defaults.
- **Migration:** a record with a shorter stride is back-filled with `0xFFFF`.
- **Discovery:** clients read `0x6035` first; no reply in the timeout = "V3 unsupported" (never "factory").

## 5. Firmware defaults (candidates — final values from SIMULATION_REPORT)

| Level | Character | Response | Carry strength | Carry extent | Max Torque |
|---|---|---|---|---|---|
| 1 | ECO | 40 | 20 | 30 | 100 |
| 2 | TRAIL | 60 | 70 | 60 | 100 |
| 3 | SPORT | 80 | 50 | 50 | 100 |
| 4 | SPORT+ | 90 | 80 | 80 | 100 |
| 5 | BOOST | 80 | 50 | 50 | 100 |

Engine default in the V3 candidate build: 1 (V3).

## 6. Verification (TEST_MATRIX G1-CFG / G3-CFG)

Shared byte vectors; negative tests for each reject reason with a "no mutation" assertion; reserved-param rejection;
generation conflict; persist -> restart -> readback; corrupt/absent/newer record; P0/P1/P2, bank and tuning byte
streams unchanged; CANable decoder ignores the new IDs.

## 7. Known merge point

HMI-SRC3 (`e2868f6`) changes the same `processCAN_Rx` hunks and adds single-owner transfers. Merge order when it is
accepted: SRC3 first, V3 handlers on top of its owner gate.
