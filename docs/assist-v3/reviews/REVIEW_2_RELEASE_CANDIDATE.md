# REVIEW 2 — Assist Behavior V3 release candidate

| Field | Value |
|---|---|
| TIMESTAMP | 2026-10-08T08:53:37+02:00 (system clock) |
| Reviewer | Independent Claude subagent, fresh context. Did not write any of the reviewed code |
| Scope | `25df554` (tag `baseline/rideable-25df554`) .. `43a1d44` (branch `feature/assist-behavior-v3`). The review was started on `b034a11`. `43a1d44` changes only `tools/test_m820_stack_gate.py` (the window for the known-bad O0/2K image) and adds no firmware change |
| Inputs | src/, inc/, ldscripts/, scripts/sources-m820.txt, tools/build_firmware.py, the V3 host tests, tools/assist_v3_acceptance.py, tools/run_assist_v3_matrix.py, the existing .build/ artefacts (matrix.csv, acceptance.json, carry_l4_full.log, build manifests), docs/assist-v3/* |
| Not done | No new target build and no new full gate run. The background `verify_all --require-target` was left alone. Evidence quoted from .build/ is earlier output, labelled [SIM]/[LOG]. Code analysis is [CONFIRMED_CODE]. Inferences are labelled |
| **VERDICT** | **CHANGES_REQUIRED.** No safety BLOCKER was found in the code: the safety ownership in V3 mode is sound. The owner's own gate is not met yet: the acceptance tool exits REJECT, D-039 CPU is not measured, the build manifest misdescribes the image, and the simulation report is stale. The fixes are small and bounded. Most of them are process or traceability items |

## Answers to the review questions

### Q1 Safety ownership in V3 mode — PASS (one MINOR, #4)

Trace through `assist_pipeline_update()` (src/assist_pipeline.c:377-578) with `v3_mode`:

- The V3 request is `(y*g1)>>12`, with g1 clamped to 0..1.0 (l.408-415). It enters `ap2_limits_apply` exactly where the G53 request did (l.419-437). The protection ceiling (l.440-444) is applied unchanged, after its own rate limit. The V3 code writes no limit, ceiling, slew mode or zero policy.
- Assist off forces 0 (l.448). The backstop is a min only (l.453-454). `native_cut` comes after the backstop, and in both engines it gives 0 + SAFETY + 3200 ticks (l.465-468). Brake reaches this point through `safety_cut_non_direction` (main.c:3410, ride_control.c `pipe_in.safety_cut`). `in->brake` is only an observation.
- R1 bumpless release compares against `lim.final_iq`, the post-limit request of the active engine (l.447, 478-484, 512). The fast slew guard is unchanged (l.506-507). In V3 mode the standstill predicate replaces the stock hard zero, still with FORCE_ZERO (l.496-504).
- Owner arbitration is unchanged. Walk, calibration and walk-release reset the pipeline, which resets V3 (ride_control.c:230-284).
- **Engine latch** (l.516-530): it switches only when the published request is 0, `!assist_v3_engaged()` (y==0), the G53 pre-limit request is 0, and no veto is active (cut, assist off, backstop not OPEN, standstill). So it cannot switch with non-zero demand. On the switch tick it sets `pulled_down`. Readback is truthful (`engine_active` is set only here). At boot `engine_active=false` (assist_v3_config.c:353). With no CONFIG_A record the engine is requested V3. The switch then happens at the first pedal onset, before G53 or V3 build any demand, because standstill is a veto. This is safe, but the test plan must expect `engine_active=0` until the first stroke (#10).
- **Backstop after rework 9dc1a0a**: REVERSE = 2 consecutive counted reverse steps, or the native backpedal latch (`pas_direction_backpedal_confirmed`, 3 consecutive reverse steps, pas_direction.c:96-101). An INVALID transition gives `dsteps==0` in crank_phase. That does not reset `rev_run`, which only a counted forward step resets (l.295-298). So R, INVALID, R still closes. **A single INVALID or jitter step cannot keep demand alive in a real reverse.** A real reverse produces net consecutive reverse steps. V3 does not depend on the backstop here: it zeroes y on the first reverse step with the load released (`reverse_released`, assist_v3.c:332-334), on 2 consecutive reverse steps, and on negative G53 cadence. `rev_hold` blocks any rise until a forward step. A R-F-R-F pattern reopens the backstop at <= 3.5 Iq/ms. With zero net motion and an unloaded pedal this is not a hazard. Recorded as an observation only.
- HOLD/DECAY timing is correct when HOLD ends by time (1500 ms, then linear 300 ms). It is defective when HOLD ends by **distance** (#4). The defect errs on the safe side.
- The host tests force V3 to its maximum through a link-time wrap (tests/host/assist_v3_pipeline_host.c SAFE0-10, BS1-9, SEL1-11, VETO). The published bounds therefore come from code outside assist_v3*.c. This is the right evidence shape.

No path was found that publishes V3 demand while brake, fault (torque or PAS invalid, critical overtemperature through safety_cut), reverse or standstill should zero it.

### Q2 Carry — PASS for safety, MINOR for efficacy and evidence (#5, #6)

- **Bounded:** the caps are first-wins. Time is `min(cfg, 1200 ms)` and distance is `min(cfg, 1500 mm)` (assist_v3.c:241-244). `_Static_assert`s tie them to the backstop's 1500 ms / 2000 mm (inc/assist_v3.h:10-16). The carry target is clamped to <= y (l.247, 362) and falls linearly to half. `move_toward` is the only rate, so carry can never raise demand.
- **Cancels:** brake, native_cut and level 0 give reason 1. Direction inhibit, any reverse step and negative cadence give reason 2. Acceleration (`rel_accel > 300 permille/s`) gives reason 3. The time cap gives 4 and the distance cap 5 (l.256-261). Reverse also wins by branch order (l.351). The pipeline zeroes the request on native_cut whatever carry does.
- **Independence:** the backstop is pipeline-owned and does not read carry (assist_pipeline.c:286-350). In HOLD it never exceeds the published value. It ends at G53 true-stop/real_stop + 1500 ms + 300 ms, or earlier by distance. BS5/BS9 cover this with V3 forced to max.
- **Plausibility on the real bike:** admission needs effort >= 4500 CLU, `load_ctrl >= 1500`, measured Iq >= 16 % P, y >= 7.8 % P, cadence >= 20, motion quality >= 1 and speed 1.5..10 km/h. The frozen score must also be >= 3000, which needs effort >= ~6100, or Iq >= ~35 % P, or a combination. Carry is not over-eager. The evidence suggests it may be **mostly dead** at the speeds where it matters ([INFERRED]):
  - The wheel sensor gives no fresh interval below ~3 km/h (10600 ticks = 2.65 s, one pulse per 2218 mm), so quality 1 is unavailable there.
  - Quality 2 needs a ratio learned at >= 5 km/h with steady cadence (motion_est.c:27-33). That ratio may come from another gear: after a downshift, speed is overestimated, so admission is denied above 10 km/h and the distance cap is reached early. Both effects make carry more conservative.
  - In the SIL matrix carry never activates (0 activations in every row, matrix.csv [SIM]). L4 has one positive case: `climb_pedal_stop_obstacle`, active 0.200 s, 0.495 m, cancelled by distance ([SIM], carry_l4_full.log).
- **Motion quality 0:** carry is not admitted, and the cancels still work.
- **Quality 1:** `rel_accel` is 0 (it is computed only at quality 2, motion_est.c:58), so the acceleration cancel cannot fire. A downhill roll during carry is then bounded only by the time and distance caps (#5).

### Q3 G5300 byte-identity on the bike — functional PASS, timing OPEN (#1)

- The publishing path in G5300 mode is the baseline path (`pre_limits` = G53 request, `stock_hard_zero`, R1 against the G53 request). The backstop and standstill predicate are computed but not applied. G-EQ: SIL matrix 150/150, L4 on/off with 17 CSVs + fuzz byte-identical ([SIM], carry_l4_full.log).
- parser.c: weak observer only (`assist_v3_config_set_legacy`). It changes no G53 input.
- CAN 0x6035-0x6037: tool source only, and the generic ACK is suppressed for them. The legacy multiframe declaration path is unchanged in effect, because `assist_v3_config_other_declaration` returns false only for a non-owner source while a transfer is live, and the owner is always source 5.
- ride_telemetry: V3 frames exist only when `CAN_RIDE_TELEMETRY_ENABLE` is set (DIAG). NORMAL frames are unchanged.
- main.c: `crank_phase_on_event` in the PAS drain and `set_pack_voltage_mv` are observation only. CONFIG_A is written only on an explicit persist command, at standstill.
- **RAM/stack:** the stack gate reports margin 2080 B of 5120 B (fg 1888 + ISR 1152) for the normal image at 4a215b4+dirty (manifest [LOG]). The ~304 B above the initial SP is unused RAM above the stack top, not a collision risk. No malloc is used in src/, so the 1 KB heap is unused. The gate is static `.su` analysis and does not follow the flash-HAL calls made through function pointers. `persist_now` puts ~620 B on the stack, which is still inside the margin. PASS. Recommendation: one stack-paint high-water check on target in the DIAG ride image.
- **Timing:** the V3 stage, `motion_est` and the backstop run every foreground tick **in G5300 mode too**. Byte-identity on the bike therefore also depends on the target's dropped G53 logical ticks, which are not measured (#1).

### Q4 IMU seam — PASS

- `assist_motion_sanitize` reads `valid` as a byte, rejects stale samples and zeroes padding (src/assist_motion.c).
- tests/host/assist_v3_trajectory_host.c T12 drives random garbage with valid=0 and with stale valid=1. It asserts that the V3 output **and** the telemetry are bit-identical to an all-zero seam over 24 000 ticks.
- V3 consumes no motion field, only `imu_valid` in telemetry. main.c passes `.motion = {.valid=false}`.

### Q5 Config protocol v2 — PASS with MINOR (#7)

- Atomicity: the write is decoded into a private candidate and committed in one assignment only after the full validation, the CRC and the expected-generation check (assist_v3_config.c:132-171, 451-454). A rejected write mutates nothing.
- KEEP and short param_count preserve values. The generation skips 0xFFFE/0xFFFF. Persist runs only on an explicit op with the expected generation, only at standstill (main.c:3388) and never during a transfer.
- Records: the CRC is programmed last and the record is verified by read-back. Records are scanned newest first. v1 records are ignored, including v1 records offset at 128 B. Newer and corrupt records fall back to defaults.
- **Weakness:** when the page fills (6 slots), the whole page is erased before the new record is programmed (l.342). A power loss in that window loses every record. Defaults then apply, so the engine becomes V3 even though the owner had persisted G5300.
- CONFIG_A vs BL820 erase is still [UNKNOWN]. CONFIG_A was unused at baseline, and the existing virtual EEPROM lives next to it at 0x0803F000. That it survives updates is only [INFERRED].
- D-020 is respected: the default engine with no record is V3, but V3 becomes active only through the zero-demand latch.

### Q6 Evidence adequacy — ISSUE (#3, #5, #9)

- tools/assist_v3_acceptance.py currently returns **REJECT 483/484**. The failing case is dead spot 20 rpm, ripple 0.257 vs 0.25 (baseline 0.971). For the rider this is a large improvement, not a risk. Under the owner's rule ("full final gate PASS") it still needs either a fix or a written, owner-approved waiver.
- The tool is weaker than SIMULATION_REPORT §2 in several ways:
  - TRUE_RELEASE <= 250 ms, where §2 requires <= 45° + R(Response).
  - The ripple criterion is relaxed to `max(base+0.01, 0.10)`.
  - G1-STAND (load-held stop within ±20 %, two-sided, no early cut) is not checked.
  - "Class transitions in steady riding = 0" is not checked.
  - There is no positive carry criterion, so a carry that never fires passes.
  - A missing (profile, cadence, metric) row adds no criterion and fails silently.
- The matrix covers level 3 and one effort level only.
- The host tests do exercise the active V3 path in both engines (pipeline host, with real config transfer and real fast_iq_slew).
- **Doc claims without current support:** SIMULATION_REPORT.md still reports the 15df8e9 run (G1-LEVEL FAIL, restart FAIL) and contains no "SIMULATION_REPORT 2" section. PROGRAM_STATE.md describes HEAD 15df8e9. D-041 still says y is zeroed on `g53_reverse`, which rework 9dc1a0a removed (assist_v3.c:324-329).

### Q7 Telemetry — PASS (MINOR #11)

The DIAG V3 group carries enough to explain "why this Iq now": intent, env, demand, base target, target, rate_mode, release class, carry state, score, cancel reason and remaining caps, motion speed, quality and acceleration, final Iq, backstop Iq and state, engine active/requested, standstill, pulled_down, switched, CPU max/last and dropped ticks. NORMAL frames are unchanged.

Gaps:
- The g1 / `v3_request_iq` value after g1 is not sent.
- The data is a snapshot every ~50 ms, so one-tick events are visible only through latched fields.
- `rate_mode` is packed into 4 bits (ride_telemetry.c case 11). This holds only while the rate-mode enum stays below 16.

## Numbered issues

1. **MAJOR — D-039 foreground CPU and dropped G53 ticks are not measured on target, and the V3 stage also runs in G5300 mode.**
   - Location: src/assist_pipeline.c:405-407 (`motion_est_update` + `v3_stage` called unconditionally).
   - Each tick runs `g53_static_target()` 5 times (assist_v3.c:282, 295-299), with int64 divisions on a Cortex-M4. It also runs `assist_v3_effective()` twice (assist_pipeline.c:212-214) and the intent update. The cost is unknown: [INFERRED] tens of µs.
   - Why it matters: brake is sampled and applied in the foreground (main.c:1454, 3410). A foreground overrun therefore lengthens brake-to-zero latency in **both** engines, and breaks G5300 byte-identity on the bike.
   - TEST_MATRIX G1-CPU itself says "before any V3 ride".
   - **Fix:**
     - (a) Bench-measure the DIAG image on a stand: V3 on vs `--assist-v3 off`, both engines, max cycles and dropped logical ticks against the D-039 budget.
     - (b) Preferred: skip `v3_stage`/`motion_est` while `!v3_mode && !requested && !ASSIST_V3_SHADOW_TELEMETRY`, and reset V3 when the engine becomes requested. The latch only needs y==0, which holds when V3 is not computed. This restores G5300 timing identity.
2. **MAJOR — The build manifest misdescribes the image.**
   - Location: tools/build_firmware.py:306 hard-codes `"stage": "shadow (Milestone B)"`.
   - Every candidate image is labelled shadow (see the .build/*/manifest.json files), yet with no CONFIG_A record it publishes V3 after the first pedal stroke. This breaks RULE 26.
   - **Fix:** describe the stage as "active V3 (Milestones C+D), default engine V3 without a CONFIG_A record". Rebuild the release images.
3. **MAJOR — The acceptance and report state do not support "PASS".**
   - The acceptance tool exits REJECT 483/484. SIMULATION_REPORT §3 still shows FAILs from 15df8e9. The relaxed criteria are not recorded as a decision, and missing rows pass silently.
   - **Fix:**
     - Add §5 to SIMULATION_REPORT with the b034a11/43a1d44 run and the exact command.
     - Record either a fix or an owner waiver (with timestamp, as a DECISIONS entry) for the dead-spot 20 rpm ripple.
     - Record the deviations from §2 as a decision.
     - Make `assist_v3_acceptance.py` fail when an expected (profile, cadence, metric) row is missing.
4. **MINOR — Backstop HOLD → DECAY on distance underflows.**
   - Location: src/assist_pipeline.c:330 `ticks=ctx.bs.hold_ticks-V3_BS_T_STOP_HARD_TICKS;`.
   - When the distance bound (2000 mm) ends HOLD before 1500 ms, `hold_ticks < 6000` and the uint32 wraps. The product at l.335 wraps too, so the ceiling usually steps to 0 instead of decaying over 300 ms.
   - This errs on the safe side (published Iq then falls at the 6.84 Iq/ms fast slew), but it is non-deterministic and contradicts the documented bound.
   - It is likely on the bike: coasting at >= 5 km/h while standing on a loaded pedal.
   - BS9 cannot detect it, because it accepts any time to zero.
   - **Fix:** `ticks = ctx.bs.hold_ticks > V3_BS_T_STOP_HARD_TICKS ? ctx.bs.hold_ticks - V3_BS_T_STOP_HARD_TICKS : 0u;` Extend BS9 to assert a linear decay of no more than `decay_from/300` per ms.
5. **MINOR — Carry gating at motion quality 1.**
   - The acceleration cancel needs `rel_accel`, which exists only at quality 2 (motion_est.c:58).
   - At quality 1, carry is admitted with no acceleration information and can run on a downhill until the time or distance cap.
   - **Fix:** admit carry only at quality 2, or apply a reduced cap at quality 1.
   - Separately, positive carry evidence is one L4 case of 0.2 s. Treat carry on the bike as observational (DIAG `carry_*` fields), not as an acceptance item.
6. **MINOR — `motion_est` holds a stale wheel speed.**
   - At quality 1 it holds the last interval's speed for up to 2.65 s, with no silence-implied decay like main.c:1589-1593.
   - The learned ratio survives gear changes until it is refreshed at >= 5 km/h.
   - Effect: the V3 standstill predicate (assist_pipeline.c:359) and the carry speed window use an optimistic speed. Safety is not degraded, because the backstop and the V3 stop rules act earlier.
   - **Fix:** cap speed by the silence-implied value, and mark the ratio stale after N wheel pulses with a mismatched cadence or erps relationship.
7. **MINOR — The config log is not power-safe across the page erase, and BL820 erase is [UNKNOWN].**
   - Losing records means defaults, which means engine V3.
   - **Fix (now):** in the ride test procedure, read CAPS (`engine_active`/`engine_requested`, flash_state) after every flash and every power cycle.
   - **Later:** ping-pong between two pages, or write before erase.
8. **MINOR — Documentation drift.**
   - PROGRAM_STATE.md (HEAD 15df8e9, Milestone B table) is stale.
   - D-041 contradicts the code after 9dc1a0a and needs a superseding entry.
   - ARCHITECTURE_V3 §7.3 should state the 2-step / native-latch reverse definition and the distance bound.
9. **MINOR — Matrix coverage is level 3 and one effort level only.**
   - Levels 1 and 5 (ratio 95 %…525 %) and the high- and low-effort cases are not in the acceptance run. The static map is exact per level (G1-STATIC), but the envelope and intent interaction with the level ratio is not tested.
   - **Fix:** run the matrix at levels 1 and 5. Meanwhile, start the bike test at L3.
10. **MINOR — Engine readback at boot.**
    - `engine_active` reads 0 (G5300) after every boot until the first pedal onset, even with V3 requested, because standstill is a latch veto.
    - This is correct and truthful, but it must be written in RIDE_TEST_PLAN, so an operator does not abort the test or misread a G5300-only first stroke.
11. **MINOR — Telemetry.**
    - Add `v3_request_iq` (the value after g1) or g1 Q12 to the V3 group.
    - Add a static assert that the rate-mode enum is < 16.
12. **Observation (bike test case, not a defect).** While y > 0, which includes the decay after a stop, a forward crank step bypasses the start-readiness gate (assist_v3.c:398). This is restart continuity by design. Test it explicitly: at standstill with a loaded pedal and no brake, nudge the crank one step and confirm the response is <= G5300.

## Must fix before flashing the controlled-test image

1. Fix #2: the manifest stage string. Rebuild, and record the SHA-256 of the NORMAL and DIAG images that will be flashed.
2. Fix #4: one line plus the test. It is cheap and removes a non-deterministic transition the ride would actually hit.
3. Close #3: SIMULATION_REPORT §5 with the current run, a fix or an owner waiver for the 20 rpm dead-spot ripple, the deviations from §2 recorded, and missing-row detection.
4. Before the first **ride** (bench flash allowed for this): #1. Measure DWT cycles and dropped G53 ticks for the DIAG image on a stand, in both engines and with V3 off. Do not ride if the D-039 worst case is exceeded or if dropped ticks rise vs ASSIST_V3 off. Preferably apply #1(b) first.
5. Test procedure items: #7 (CAPS readback after every flash and power cycle), #10 (expected boot readback), #12 (nudge test), start at L3 (#9), treat carry as observational (#5).

Items #5, #6, #8, #9, #11 may follow the controlled test, provided they are tracked as TECH/BUG candidates.
