# TQ-06 Phase-9 EB74 startup/zero-lifecycle conformance audit

TIMESTAMP: 2026-09-26T08:24:53+02:00
TASK: TARGETED EB74 STARTUP / ZERO-LIFECYCLE CONFORMANCE AUDIT
FIRMWARE: `feat/TQ-06-g53-port` @ `62785bd105a0e5d2681b1db71b40f62180af6c53`
GOVERNANCE SSoT: `1b57ceda9326b1c2bf08487c89497be1a9baf5fb` (read-only)
PRODUCTION / GOVERNANCE / L4 stimulus changes in this audit: NONE

## Accepted sources and identity

- Current contract: `C:/Projekty/eVistDrive-tq06-phase9-g5/contracts/M820_TORQUE_INTENT_CONTRACT.md`, SHA-256 `9d8d8ed3d1b9e29e95c8915cd385b8e7c8a64edd1de03ccce0ade3c08c8c8ef4`; EB74 §5 and A-D7EC §4.3.
- Current accepted formalization: `.../evidence/evd-tq/TQ-02B/REV-EB74_FORMALIZATION.md`, SHA-256 `6783353330cc6a4becd8be1b9e43156614e0438ba1e37aff0155f18dd215b688`.
- Accepted semantic transcription `eb74_model.py`, SHA-256 `b1164ea5d0e67d7fab9421ed39b0e6aa8e4fe7dbb4130f70e6268eda714d0264`.
- Accepted native startup result `REV-EB74/results/startup_boundary.json`, SHA-256 `3a1a559b32d1849d2c392c4b7e0b1a5d46645964089da36c84de998f8b3cf6b8`.
- Accepted Level-4 comparison fixture `host/boundaries/eb74-reference.csv`, SHA-256 `1ec982a8d01b5e166b5c581fbbf62d4e5d2f0c78dc3404fe6d3743b4e828d47d`; 29,516 rows. Its provenance JSON records the fixture-producing governance SHA as `6f8ab5ec...`; this audit separately verified the requested current SSoT and file hashes above. No governance files were edited.

The original-BIN Unicorn runner was not re-executed because `G5300_BIN` is unset in this environment. The accepted native startup result already exists; the two required lifecycle schedules were executed with the accepted semantic transcription and compared invocation-by-invocation against the current C port. No reverse project or governance change was created.

## Frozen 960/check-window behavior

The frozen adapter maps `CLU` to synthetic pre-EB74 input as `min(3200, 750 + floor(CLU*2450/6000))`. The 960 anchor is reachable at CLU 515; CLU 0 maps to 750 and CLU 12000 maps to saturated 3200. This is the owner-adapter mapping frozen by contract, not a physical sensor-equivalence claim.

In `g53_ad7ec_step()`, the corresponding port code is `EB74_INITIAL_THRESHOLD=960`, initialized as `out.threshold` by reset. While enabled and `check_count < 100`, the port increments `check_count`; if `source > out.threshold` (strict `>`; equality 960 does not take this branch), it rewinds the counter to `100-10=90`. This is the power-on plausibility/check window and its B+0x51 check-fault path, not adaptive-zero qualification, normal thresholding, or M+0x29E qualification. The frozen transcription does the same counter increment and strict B+0x38 comparison, sets B+0x51 and rewinds B+0x3C to 90 when source is above 960. A high input therefore needs at least ten uninterrupted invocations at or below 960 to advance 90→100; the following invocation enters normal processing. Starting low from reset gives 30 startup calls plus 100 window calls, with normal processing on invocation 131.

One real port mismatch was found in the same check window. For `source <= 960`, frozen EB74 does not write B+0x58 during the check window, so a nonzero entry value is retained. The port unconditionally assigns `rider_input_native=0` and returns for every check-window call. Accepted native startup test A3 explicitly seeds B+0x58 to `0x1234` at invocation 31 with M+0x90=960 and confirms it is retained through invocation 130. The port clears it on invocation 31 and every remaining check-window call. The model/port sentinel trace reproduces this mismatch for 100 rows. For a normal reset whose output is zero, both paths appear zero during the window; that is why the persisted zero-initialized lifecycle fixture did not expose this state-retention distinction.

Other branch comparisons:

| Element | Port vs frozen evidence | Detail |
|---|---|---|
| CLU→pre-EB74 mapping / saturation | MATCH | Formula and anchors match the frozen owner adapter. |
| Startup counter | MATCH | 0→10…300 in 30 calls; saturated at 300; output zero during this reset gate. |
| B+0x38=960 check comparator / counter rewind | MATCH | Strict `source > 960`; increment then rewind to 90; high path forces zero through the check fault. |
| Check-window output retention | MISMATCH | Frozen holds an existing B+0x58 value when the check-fault comparator is false; port writes zero unconditionally. |
| Adaptive zero and gates | MATCH in tested stock paths | Initial zero 750; cadence/speed/M+0x298 gate resets tracker; auto-zero arm/window/sample flow and 750..995 clamp match the accepted model vectors. |
| D+0xEE threshold selection | MATCH | Zero selects zero+245; nonzero selects fixed 820. |
| Input saturation / filter / below-threshold reset | MATCH | Saturate before subtraction; coefficient 6176/shift 12; equality produces zero filter input/reset. The accepted 29,516-row fixture has zero port/reference mismatches. |
| M+0x29E | MATCH for accepted stock default | P+0x52 is 0 in the frozen stock fixture, so M+0x29E remains 0. Contract marks the P+0x52/cadence latch conditional on the M+0x29E-sensitive E1E8 path; no broader claim is made for a path this adapter does not input. |

## Reused tests and bounded lifecycle schedules

Standalone existing test command:

```text
gcc -std=c11 -Wall -Wextra -Werror -Iinc -o .build/eb74-boundaries-targeted.exe tests/host/g53_port_boundaries_host.c src/g53_port_boundaries.c
.build/eb74-boundaries-targeted.exe
```

Result: `EB74 independent reference: 29516 vectors, 0 mismatches PASS`; boundary host test PASS. The persisted fixture's episode 0 has CLU=0 for invocations 1–130 then CLU=6000 and positive output at 131. Episode 1 has CLU=6000 for 1–200, CLU=0 for 201–340, then high load from 341 with positive output; it does not test sustained high from reset. The accepted `eb74_diff.py` T01/T02 scenarios likewise include low qualification or a subsequent return below the check threshold. The host 64/64 result is not used as evidence that Level-4 has its prerequisite history.

The current port was compared against the accepted executable semantic transcription for these schedules. Every trace row records both model and port values for CLU, pre-EB74, startup/check counters, zero, threshold, filtered value and accumulator, auto-zero counters/state, D+0xEE, M+0x298, rider output and M+0x29E.

| Schedule | Inputs | Frozen/model outcome | Current port comparison |
|---|---|---|---|
| A — unloaded boot then Level-4 load | Reset; CLU=0 for 130 calls; CLU=12000 for 20; feedback zero | Calls 1–130 output 0; call 131 pre=3200, threshold=995, filtered/output=831 | All 150 calls, all recorded fields match |
| B — high from reset | Reset; CLU=12000 for 400 calls; feedback zero | Output remains zero; after call 31 check_count remains 90 and normal path is never entered | All 400 calls, all recorded fields match. This recurrence is invariant for constant high input until a call at/below 960 permits progress. |
| S — accepted B+0x58 hold sentinel | Reset; CLU=515 → pre=960; D+0xEE=1; seed B+0x58=`0x1234` just before call 31 | Frozen output retains 4660 through calls 31–130; call 131 enters normal path and output becomes 52 | Mismatch on calls 31–130 (100 rows): port output is 0; call 131 agrees at 52 |

Traces:

- [`schedule-A.csv`](./EXEC-TQ06-PHASE9-EB74-CONFORMANCE-20260926-schedule-A.csv), SHA-256 `28a0f913f072ce3ef72c935cf927b61503992fe0ba2f446705d967da9a1806ad`.
- [`schedule-B.csv`](./EXEC-TQ06-PHASE9-EB74-CONFORMANCE-20260926-schedule-B.csv), SHA-256 `ab2d7f59a094a93abbf742856531511799a4ab1d9f161b862b8260e44b24547a`.
- [`schedule-S.csv`](./EXEC-TQ06-PHASE9-EB74-CONFORMANCE-20260926-schedule-S.csv), SHA-256 `684142fa332f7f480fb39402cf93557dc4b4f9dde516449f7db05f8396d6db9a`.
- [`port-vectors.c`](./EXEC-TQ06-PHASE9-EB74-CONFORMANCE-20260926-port-vectors.c), SHA-256 `3be2f9021953f3eb28b57faa7a889eb306bc016c09b75535ef2fe37a2d3ff1e9`.

## Level-4 tick-zero audit

The existing Level-4 fixed matrix was observed with a temporary test-only projection; the source is [`level4-startup-observer.c`](./EXEC-TQ06-PHASE9-EB74-CONFORMANCE-20260926-level4-startup-observer.c), SHA-256 `4f652ad069eef84cc8c5531f3ec27de9264dada8d144b19be698fc96886dc730`. It does not feed observation values into control. [`level4-startup.csv`](./EXEC-TQ06-PHASE9-EB74-CONFORMANCE-20260926-level4-startup.csv) records all nine fixed cases.

All nine start at control tick 1 with `load_ctrl=12000`, mapping to pre-EB74 3200; native PAS/cadence are still zero at that point. The first logical EB74 call is at control tick 4, with startup counter 0 and already-high load. There are zero EB74 invocations before the first CLU>600. No case has any control tick with pre-EB74 <=960 during the 32,000-tick/8-second scenario; minimum CLU ranges from 883 (`flat_soc90`, pre=1110) to 11,490 (`hill15_soc20`, pre=3200). Thus the high-from-reset check branch continually rewinds its check count and cannot qualify in these fixed scenarios. This condition is common to all nine. `evd_rider_init()` sets virtual `pedaling=true`; the Level-4 rider produces strong launch preload at zero crank cadence. The harness models a fresh controller boot with immediate rider torque/preload, not an unloaded boot sequence. This is a test-model/startup assumption, not evidence of real M820 boot history.

## Classification and disposition

- High-load-from-reset behavior itself is consistent with the accepted EB74 transcription: constant source above 960 holds output at zero until an unloaded/check-qualified window occurs.
- The Level-4 9/9 result is explained by its missing unloaded prehistory; this is the separate harness-startup condition.
- The conformance audit nevertheless found the concrete B+0x58 check-window retention mismatch in the port. Per the requested precedence, the single blocker classification is `IMPLEMENTATION_DEFECT`.
- No fix was applied. Production, EB74 thresholds/algorithm, Level-4 stimulus/prehistory, governance and frozen contracts were not changed. Phase 9 remains blocked; DISC-007 was not reached; no commit/push/merge/flash; Phase 10 not started.
