# Audit D - Configuration / CAN protocol (Assist Behavior V3)

AUDIT: D (read-only) | AT: 2026-10-07T12:21:50+02:00 | BASELINE: `25df554` (`feature/assist-behavior-v3`)
SCOPE: how behavior parameters can be added with a versioned contract usable by CANable now and a mobile app later.
Evidence labels per AGENTS.md section 4. Line numbers refer to the baseline worktree unless stated.
CANable: `C:\Projekty\eVistDrive\canable-web` HEAD `62aef1f` + ~37 uncommitted WIP files (read only). Mobile app not inspected.

## 1. Current map

### 1.1 Legacy blocks P0/P1/P2 (`0x6010/0x6011/0x6012`) [CONFIRMED_CODE]

All three are 64 B RAM images `Para0/1/2` (`src/CAN_Display.c:80-82`), rebuilt from `MotorParams_t` by `parse_MOparams()` (`src/parser.c:267`) and parsed back by `parse_DPparams()` (`src/parser.c:195`).

| Block | Byte | Meaning in this firmware | Persisted | Stock M560 descriptor (`p0_p1_descriptor_table.txt`) |
|---|---|---|---|---|
| P0 | [0] | untouched | no | defined (min 12, max 255, def 50) |
| P0 | [1..9] | accel per slot 1..9; only slots 2,4,6,8,9 stored; 1,3,5,7 read factory | yes (`assist_settings[k][2]`) | `StartAcceleration` |
| P0 | [10..27] | ratio u16 LE per slot (`TQO_threshold[]`) | yes | `AssistRatio` |
| P0 | [28..29] | read constant 1000, write ignored | no | defined (def 1000) |
| P0 | [30..50], [55..62] | no consumer; echoed from RAM only (not persisted, zero after reboot) | no | all defined (e.g. [41..50] feed D+0x88, [55] u16 def 500, [57],[59],[61] u16 def 2400) |
| P0 | [51..54] | G5300 AUTO scale/enable/step, READ = effective, WRITE ignored (`parser.c:326-342`, `g53_port_chain.c:2030-2050`) | no (volatile) | defined |
| P0 | [63] | forced 0 (`update_checksum`, `CAN_Display.c:1369`) | no | flag 0..1 |
| P1 | [0..4],[7..14],[36..39],[60..61] etc. | system limits, battery, walk (see `parser.c:195-262`) | yes | defined [0..59] |
| P1 | [40..48] | power % per slot (`assist_settings[k][0]`) | yes | M560 power |
| P1 | [49..57] | speed limit % per slot | yes | defined |
| P1 | [63] | sum8 of [0..62] (not verified on write) | no | n/a |
| P2 | [0..29],[31..41] | orphan `assist_profile`, `ext_boost_*` (round-trip only) | yes | `P2[54]` = legacy global accel |
| P2 | [63] | sum8 | no | n/a |

Transport (`src/CAN_Display.c`, `src/can_multiframe.c`):
- READ P0 (src 5 only; other sources get the factory 4 B `0x6003` mini block, line 1087-1105), READ P1/P2 any source (lines 1106, 1281); reply is a `send_multiframe` of 64 B. P2 reply carries the factory trailer.
- WRITE: declaration = op0, DLC1, `data[0]` = total length (>8), accepted only from source 5 (line 375); then op4 START, op5 frames, op6 END (lines 532-626). **The length is one byte, so every multiframe transfer is <= 255 B**; `send_multiframe` also takes `uint8_t` (`CANMF_MAX_PAYLOAD 255`).
- ACK: the declaration is ACKed by the generic `sendAcknoledge()` (op2, always to target 5, line 497/815). P0/P1/P2 get **no completion/result frame** (comment "to do send acknoledge OK", line ~597). Only `0x6021` and `0x6024` answer with a real result via `sendWriteResult()` (op2 ok / op3 error, DLC0, line 786).
- One transfer slot only: `Rx_MF_active` + `rx_data_length` (lines 123, 154), no timeout, a new declaration overwrites the old one.

### 1.2 eVD blobs [CONFIRMED_CODE]

| Cmd | Schema | Size | Version / validation | Commit |
|---|---|---|---|---|
| `0x6020` R / `0x6021` W | assist bank, magic `EB`, header 13 B + 5x48 B + CRC16 | 255 B (hard ceiling, `assist_modes.h:139-145`) | version byte at [2], now v10 (accepts v1..v10, stride in [5]); CRC, mode ids and stride validated **before** commit (`assist_modes.c:713-780`) | RAM apply only; persist by `0x6022` |
| `0x6023` R / `0x6024` W | tuning, magic `TU`, v8 | 32 B (one spare u16 at [28]) | v2..v8 accepted, unknown version rejected; CRC then clamp then commit (`tuning_config.c:171-230`) | RAM apply; persist by `0x6022` |
| `0x6022` W | persist trigger | DLC any | source 5 | sets `bank_save_request`; written at standstill (`main.c:3344-3360`) |
| `0x6028` R | SY v2 | 8 B | CRC low byte | n/a |

All bank and tuning bytes are used. The bank record has no spare byte (13 + 5*48 + 2 = 255).

### 1.3 Versioning / capability today [CONFIRMED_CODE]

There is **no capability field for the configuration plane**. Version bytes exist per blob (bank v10, tuning v8, SY v2, DG v9). CANable infers "eVD" from a successful `0x6020` parse (COMM-GAP-09, COMMUNICATION_REGISTRY). `0x6001` carries a build string only. P0/P1/P2 have no version.

### 1.4 Persistence [CONFIRMED_CODE]

| Item | Fact |
|---|---|
| Page | one 2 KB page `0x0803F000` (`main.c:93-95`); SOC page `0x0803F800` separate (wear-levelled) |
| Layout | 7 x int32 hall (28 B) + `MotorParams_t` (728 B, pinned by `_Static_assert`, `main.c:334`) + 16 B footer at +756 (`FMC_OFFSET_FOOTER`, `main.c:121`) = 772 B used, **1276 B free in the page** |
| Footer | magic `0xEB1C5001`, version 1, length, crc32 over payload, programmed LAST (`main.c:6040-6065`) |
| Validity | `param_record_valid()` checks magic+version+length+crc32; also accepts a legacy 724 B record (footer at +752) (`main.c:5995-6006`) |
| Write | whole-page erase then program, skipped if content identical (`main.c:6044-6052`). P0/P1/P2 completion calls `write_virtual_eeprom()` **inline** (`CAN_Display.c:~619`), i.e. flash stall during riding is possible; bank/tuning/SOC/torque-cal persist is deferred to standstill (`main.c:3344`) |
| Corrupt / old | invalid record -> `InitEEPROM()` defaults + new record (`main.c:1225-1228`); hall angles revert to compiled defaults. Old-length record only survives where an explicit legacy path exists (724). Any `sizeof(MotorParams_t)` change wipes every rider setting (FW-095 note, `main.c:318-334`) |
| Migration | marker field in `MotorParams_t` (`assist_levels_magic = 0xA560`, `parser.c:12,268`); other blobs migrate by their own version byte |
| Defaults | `InitEEPROM()` (`parser.c:348`), `factory_*` tables (`parser.c:13-16`), `tuning_config.c` statics, `assist_modes` defaults |
| Restart | `read_virtual_eeprom` -> `parse_MOparams` -> `apply_assist_levels` (`main.c:1225-1232`); banks/tuning restored from `bank_store[]`/`tuning_store[]` if magic matches (`main.c:1239-1250`) |
| Readback | P0/P1/P2 READ returns the RAM image (so it shows what was parsed and re-serialised); bank/tuning serialise the live RAM state |
| Dedicated spare page | `ldscripts/gd32f30x_flash.ld:5-9,162-164`: `CONFIG_A` 2 KB at `0x0803E800` is reserved and unused by any source; `tools/build_firmware.py:25,245` checks the app image ends before it [UNKNOWN: bootloader/BESST update flow ownership] |

### 1.5 Existing semantic owners that a V3 parameter list may overlap [CONFIRMED_CODE]

Per-level behavior is already spread over three places: M560 P0/P1 (accel, ratio, power %, via G53 chain `g53_port_set_levels`); bank record fields honoured by the V2 pipeline (`support_ratio_pct`, `max_motor_power_w`, `max_iq_pct`, `iq_rise_fast_ms`, `release_ms`, `smooth_start.duration_ms`, `assist_modes.h:258-282`); global tuning. V3 must name a single owner per class (architecture decision, outside this audit) or it creates a second owner.

## 2. Free / reserved space verdict

| Candidate | Verdict | Why |
|---|---|---|
| P0 [30..50],[55..62] | REJECT | every byte has stock M560 meaning (descriptor table); CANable writes `unknown_bytes[30..62]` back verbatim (`bafang-serializer.js` P0 writer), BESST writes P0; not persisted today |
| P1 [60..62] | REJECT | [60..61] already walk speed; [62] single byte; CANable fills unspecified P1 bytes with `0xFF` (`bafang-serializer.js` P1 writer) so any new byte would be clobbered on every old-CANable save |
| P2 [0..29],[31..41] orphans | REJECT | reclaiming = reinterpreting stored data and old tool round trips (`main.h:344-349`); stock `P2[54]` meaning |
| Bank blob (`0x6021`) | REJECT | exactly 255 B, no free byte, per-bank not per-level-global; needs different packing (v11) and breaks the one-byte transport length |
| Tuning blob | REJECT | 1 spare u16 only, global |
| `0x6028` SY | REJECT | 8 B, positional parser in CANable |
| Grow `MotorParams_t` | REJECT as primary | changes 728 -> legacy-footer style migration for every bike; couples V3 to the core settings record |
| **New command IDs + own block** | **ACCEPT** | needs allocation (section 5) |

IDs: `0x6035..0x6037` are not listed in COMMUNICATION_REGISTRY (inventory says "empty is not free"), not in the BESST catalog (`0x6000-6003,6007,6008,600F,6010-6014,6017,60E0,60F0,60F5,60FF,6101,6200,62D9,62DC,63xx,64xx,6500`), not in the stock-compat evidence. `0x602A`, `0x602E` have no handler but are registry "UNKNOWN, not FREE". `0x60E0/0x60E1` are stock 256 B objects (avoid). Stock M560 object dictionary scan for `0x603x` has NOT been done: [UNKNOWN].

## 3. Atomicity today

| Path | Validation before commit | Partial mutation? |
|---|---|---|
| `0x6021` bank | magic, version, stride, length, CRC16, all mode ids; then writes all fields | No for CRC/length/mode failure. Field values are clamped after CRC, not rejected |
| `0x6024` tuning | length, magic, version (unknown rejected), CRC16, then clamp | No |
| `0x3203` | whole frame validated first (comment at `CAN_Display.c:~432`) | No |
| `0x6010/6011/6012` | **none before data lands**: frames are `memcpy`'d straight into `Para0/1/2` while arriving (`append_multiframe`, line 1363); `parse_DPparams` clamps field by field via `repair_motor_params` | **Yes**. (a) An interrupted/short transfer leaves a half-overwritten `Para*` image: `rx_data_length` mismatch only clears state (lines 629-634), the image stays, READ then returns it, and the **next** completed write of any of the three blocks parses all three images and commits the garbage. (b) `parse_DPparams` always parses P0+P1+P2, so a P0 write also re-commits P1/P2 images. (c) No checksum check of P1/P2 sum8. (d) Inline flash write on completion |
| Defect candidate | No bound on the frame index for P0/P1/P2 (`LONG_TRANG/END` cases at lines 552-590; bank/tuning cases have `command < 31/3`). A declared length up to 255 with >8 frames writes past the 64 B `Para*` arrays [CONFIRMED_CODE, not exercised]. Log as separate BUG, do not copy the pattern |
| Declaration ACK | `sendAcknoledge()` runs for **every** WRITE command that is not special-cased, including unknown IDs (lines 481, 497). An ACK on a declaration is therefore not proof that the command is supported |

## 4. CANable compatibility (read-only look)

| Frame | CANable decodes/writes | Effect of a firmware V3 extension on new IDs |
|---|---|---|
| `0x6010` | `parameter0()` reads accel[1..9], ratio[10..27], limit[28..29], `unknown_bytes[30..62]`; writer sends all of them back, byte 63 = 0 (HEAD `62aef1f`) | none while P0 bytes are untouched |
| `0x6011` | field-mapped P1; unspecified bytes = `0xFF` | none |
| `0x6020/6021` | parser accepts v1..v9 in WIP (HEAD: v1..v8), writer emits <= v9 (`canbus.js:900-905`); **firmware already emits v10** | pre-existing break: v10 read -> parse error -> eVD detection fails -> UI falls back to `factory_bafang` (COMM-GAP-09). A V3 design must not rely on the bank parser for detection |
| `0x6023/6024` | tuning parser accepts <= v8 = firmware v8 | ok |
| unknown IDs | frames from node 2 with unknown command are parsed as `unknown` and ignored (`canbus.js` dispatch ~675-690) | new `0x6035-0x6037` replies are ignored by current CANable; no crash |
| Detection | `READ_BANK:0` + bank parser, 1800 ms timeout then "factory" | a new CAPS read is independent; **timeout must mean "V3 unsupported", never "factory"** |

Net: adding new IDs and keeping P0/P1/P2/bank/tuning byte-identical breaks nothing in CANable beyond the existing v10 gap. Reusing spare bytes in P0/P1 would break (clobber by write-back).

## 5. Recommendation: CONFIG_PROTOCOL_V3

Principles: new IDs, never reinterpret old bytes; one RAM owner module (pattern of `tuning_config.c`); validate whole block then commit once; reject, do not clamp, at the wire; separate RAM-apply from persist; capability discovery before any write.

### 5.1 Transport (proposed, needs registry allocation and stock-dictionary collision check)

| ID | Op | Source | Purpose |
|---|---|---|---|
| `0x6035` | R (DLC0) | 5 (later 3 via cap bit) | CAPS + STATUS block, 24 B multiframe |
| `0x6036` | R (payload: `view`,`set`) | 5 | BEHAVIOR block read, view 0 saved, 1 effective, 2 firmware defaults |
| `0x6036` | W multiframe (declaration len <= 255) | 5 | BEHAVIOR block apply to RAM; ends with result frame |
| `0x6037` | W short (DLC<=8) `op` byte: 1 persist (deferred to standstill, as `0x6022`), 2 revert RAM to saved, 3 load firmware defaults to RAM | 5 | control; result frame |

Result frame: NORMAL_ACK (op2) ok, ERROR_ACK (op3) with **DLC>=2**: `[reason, index]` (0 unknown schema/version, 1 length/stride, 2 CRC, 3 range, 4 capability missing, 5 busy/foreign owner, 6 stale generation, 7 reserved bits). Requires a small `sendWriteResultEx`; stock tools ignore the payload.
Transfer rules for V3 only: staging buffer separate from `Para*`/`BankBlob`, hard bound on frame index and declared length, owner = source of the declaration, reject a new declaration while another owner's transfer is live (add a transfer timeout), exactly one result frame, accept only source 5 (mobile-via-HMI gets its own cap bit when the HMI path is defined).

### 5.2 Block layout (all little-endian)

CAPS/STATUS (`0x6035`, 24 B):
`[0..1]` magic `BV` | `[2]` format 1 | `[3]` protocol_version 1 | `[4]` schema_min | `[5]` schema_max | `[6]` level_count (5) | `[7]` set_count (1; 2 if per-bank) | `[8..9]` max_block_len | `[10]` record_stride | `[11]` param_count | `[12..15]` caps u32 | `[16..17]` param_mask u16 (which classes this firmware really implements) | `[18..19]` config_generation u16 (++ per accepted write) | `[20]` persist_state (0 clean, 1 RAM dirty, 2 pending standstill, 3 failed) | `[21]` flash_record_state (0 valid, 1 absent, 2 newer-version ignored, 3 crc bad) | `[22..23]` CRC16-CCITT (same poly 0x1021/init 0xFFFF as bank/tuning).
Caps bits: 0 behavior_v1, 1 effective_readback, 2 saved_readback, 3 defaults_view, 4 deferred_persist, 5 revert, 6 per_bank_sets, 7 accepts_src3, 8..31 reserved. Zero bits must be ignored by readers.

BEHAVIOR block (`0x6036`): header 16 B + `level_count` x `record_stride` + CRC16.
Header: `[0..1]` `BV` | `[2]` schema_id (1 = assist behavior) | `[3]` schema_version (1) | `[4..5]` total_len | `[6]` set_index | `[7]` level_count | `[8]` record_stride | `[9]` view/flags (write: 0) | `[10..13]` caps (read: controller caps; write: caps the sender requires, must be a subset) | `[14..15]` generation (read: current; write: base generation, `0xFFFF` = no check, otherwise stale write rejected).
Record i = HMI level i+1 (explicitly not M560 slot numbers, avoiding COMM-GAP-07). v1 stride 16 = 8 x u16: assist_range, max_torque, max_power, response, start_response, acceleration, carry_strength, carry_extent (names/units/ranges to be fixed by the program contract; `0xFFFF` = "use firmware default"). Size with 5 levels = 98 B, 157 B spare under the 255 B transport limit.

### 5.3 Semantics

| Topic | Rule |
|---|---|
| Write | validate magic, schema_id, schema_version == supported exactly, total_len, stride == known stride, level_count, reserved bits 0, required caps subset, generation, CRC16, every value range **before** touching live state; then one commit in the main loop; result frame always sent |
| Unknown version | ERROR_ACK reason 0, zero mutation (staging buffer discarded). Read side may tolerate larger stride (skip trailing bytes) and newer schema_version only if schema_min/max says so |
| Readback | `view 1 effective` = the values actually in force after firmware limits (limp, ceilings); `view 0 saved` = last persisted; the client compares requested vs effective to detect limiting |
| Defaults | compile-time table in the owner module; stored `0xFFFF` follows firmware defaults, explicit value is pinned per schema_version (FW-151 lesson: stored value must keep its meaning) |
| Persist | not on write. `0x6037 op1` sets a flag consumed at standstill. Storage: own record with magic/schema/version/len/crc32 footer written last |
| Where | preferred: reserved flash page `CONFIG_A` `0x0803E800` (independent erase, V3 cannot wipe core settings, own migration). Fallback: area after the existing footer in page `0x0803F000` (+772, 1276 B free) programmed before the main footer in the same `write_virtual_eeprom` cycle. Do **not** grow `MotorParams_t` |
| Migration | record carries its own schema_version and stride; loader backfills missing newer params with `0xFFFF` (bank v6+ stride pattern); record newer than firmware = ignored, defaults used, flash untouched until the next explicit persist, reported in `flash_record_state` |
| Corrupt record | defaults, `flash_record_state = 3`, no effect on `MotorParams_t` record |
| Apply to control | owner module snapshot taken in main-loop context between chain steps (same rule as `apply_assist_levels`, LEVELS task), never from ISR |
| Capability | client MUST read `0x6035` first; no reply within timeout = V3 unsupported; do not infer from declaration ACK (section 3) |

### 5.4 Alternatives considered

1. Spare bytes in P0/P1/P2: rejected (stock-defined, clobbered by CANable/BESST write-back, not persisted, no version).
2. Bank blob v11: rejected (255 B ceiling, one-byte length, per-bank only, CANable bank parser already lags at v9).
3. Tuning blob spare u16: rejected (2 bytes, global).
4. Grow `MotorParams_t`: rejected as primary (wipes every bike without a hand-written legacy migration; couples V3 to core record; FW-095 assert).
5. Sub-index multiplexed inside `0x6010/0x6011`: rejected (stock IDs; BESST and HMI write them; overloading).
6. Six IDs like the draft `protocol/evistdrive_config_schema.yaml` command plan (caps, read_saved, read_runtime, apply_ram, save_flash, revert_ram): equivalent semantics, more IDs than needed. This proposal keeps the same six functions but folds them into 3 IDs via `view`/`op`.
7. Raw-ID diagnostics namespace (`0x000102xx`): rejected (diagnostics only, outside normal addressing).
8. Reuse `0x6022` for persist: rejected (changes the effect of an existing command for old tools); keep it untouched.

## 6. Conflicts and risks

HMI-SRC3 `e2868f6` / WHEEL-BCAST `0c04f24` (not in this worktree, branches `feat/M560-hmi-src3`, `feat/M560-wheel-bcast`):
- Same hunks in `processCAN_Rx`: WRITE declaration condition (line 375), ACK block (497), LONG_START/TRANG/END (532/552/575), completion/incomplete (596-634). They add `Rx_MF_source` (single-owner transfer) and `inc/can_display_src3.h`. V3 case labels will conflict textually; merge SRC3 first, then add V3 using `can_display_long_from_owner`.
- Ownership semantics: SRC3 lets node 3 open a 64 B `0x6011` transfer and ACKs it to node 3; the single transfer slot stays shared. A stock-app declaration (seen 4x at ~1 s intervals in the HMI log) can preempt a CANable V3 transfer. V3 must add the "reject new declaration while foreign transfer live" rule and a timeout; changing existing P0/P1/P2 behavior for that is a separate decision.
- HMI-originated P1 writes pass through `parse_DPparams` + inline `write_virtual_eeprom`: keep V3 parameters out of P1 (done by design).
- WHEEL-BCAST adds periodic slot 8 (`0x02F83203`, 450 ms): extra bus/queue load during V3 multiframe tests (tx queue is 16 frames).
- `0xF203` is HMI-only; BESST (node 5) must stay silent (COMM-GAP-11): V3 must not answer `0x7F04/F203/7F01`.

Blockers / open questions:
1. Allocate `0x6035-0x6037` in COMMUNICATION_REGISTRY after scanning the G5300/M560 object dictionary and BESST for collisions [UNKNOWN today].
2. Confirm `CONFIG_A` (`0x0803E800`) is not used by the bootloader (BL820) or update tools; otherwise use the after-footer fallback.
3. Decide per-bank vs global sets (`set_count`) and the single owner per class vs bank fields / M560 P0/P1 (section 1.5).
4. Separate BUG candidates found (not fixed here): unbounded P0/P1/P2 frame index and declared length vs 64 B arrays; partial-write residue in `Para*`; inline flash write while riding; no transfer timeout.
5. CANable: bank parser/writer stops at v9 while firmware emits v10 (detection break, COMM-GAP-09). First V3 client should detect via `0x6035`.
6. Mobile path (BLE to HMI to CAN): source node and framing for `0x6035-0x6037` undefined; `accepts_src3` cap reserves the option.
7. Verification per docs/28: shared byte vectors, negative CRC/length/version/range tests with a mutation-free assertion, restart/persist round trip, old-tool regression (P0/P1 byte-identical), HW scope for flash page.

Files cited: `src/CAN_Display.c`, `src/parser.c`, `src/main.c`, `src/can_multiframe.c`, `src/tuning_config.c`, `src/assist_modes.c`, `inc/assist_modes.h`, `inc/main.h`, `src/g53_port_chain.c`, `ldscripts/gd32f30x_flash.ld`, `tools/build_firmware.py`, `protocol/evistdrive_config_schema.yaml`; governance: `integration/contracts/COMMUNICATION_REGISTRY.md`, `M560_ASSIST_LEVEL_CAN_CONTRACT.md`, `integration/evidence/m560-assist-can-map-20261006/p0_p1_descriptor_table.txt`, tasks `TASK-EVD-M560-LEVELS-001/AUTO-SPLUS-001/HMI-SRC3-001/WHEEL-BCAST-001`; CANable `bafang-parser.js`, `bafang-serializer.js`, `canbus.js`.
