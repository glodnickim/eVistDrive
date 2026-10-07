# B - Test-system audit for Assist Behavior V3

Auditor: read-only test audit. Baseline: `25df554` (branch `feature/assist-behavior-v3`), worktree `C:\Projekty\eVistDrive-assist-v3`.
Toolchain on this machine: w64devkit gcc 14.2.0 (host), Arm GNU 13.2.1 (target), Python 3.12, node (present).
Evidence labels follow AGENTS.md section 4. Everything marked [CONFIRMED_LOG] was run by this audit on the baseline; nothing in the tree was modified (only `.build/`, which is git-ignored, and this report).

## 0. Headline findings

1. **The active PEDAL demand path is small and fully identified** [CONFIRMED_CODE]. `main.c -> ride_control_update -> assist_pipeline_update -> g53_port_update (g53_port_boundaries A-x/EB74, g53_port_pas, g53_port_chain D7EC/E1E8/BDE8) -> ap2_limits_apply -> fast_iq_slew` (16 kHz owner). Rider intent, release detection, stop/reverse ramp and level/ratio logic all live in `g53_port_*`. `torque_input_load_ctrl()` (control-domain native delta, not the FAST/RUN filters) is the only torque signal that enters it; `cadence_rpm` from `cadence_filter` is used on the PEDAL path only for telemetry (`rider_power_w`), the chain derives its own cadence from `pas_ab`.
2. **`ap2_pas_state.c`, `ap2_rider_demand.c`, `ap2_estimators.c` are dead at baseline** [CONFIRMED_CODE]: no call to any of their functions exists in `src/*.c` outside the three files. They are still in `scripts/sources-m820.txt` (so they are built into the firmware) and are linked into SIL, electrical SIL, L4 and replay. Only `ap2_timebase_host` (H60) and part of `torque_control_domain_host` (H40, check D8) execute them. `assist_modes.c`/`ap2_profiles.c` are alive (level/bank/config), `ap2_limits.c` is alive (limiter chain).
3. **Many "old" suites are NOT dead.** `ap2_pipeline_scenarios_host`, `reverse_ramp_host`, `fast_slew_i2_host`, `m560_*`, `m820_uncontrolled_iq_safety_host` all drive the real G53 path through `assist_pipeline_update`. Names are misleading; classification below is by linkage.
4. **FAST/RUN torque filters (FW-033/085/112.4/141) are reachable but do not drive the motor.** They publish `assist_delta_*` to diagnostics and telemetry; `snapshot.load_ctrl` is computed from the unfiltered corrected delta (`src/torque_input.c:788-925`). H21, H37 and the torque benches therefore protect a signal that V3 may *reuse* (hence ADAPT, not remove) but that the baseline behaviour does not depend on.
5. **Ride-feel metrics are observation-only at baseline.** `run_regression.py` asserts only a byte-identical RUN_100 re-run; `analyze_assist_ripple.py` fails only on unusable evidence (its own docstring: attenuation "is not a TQ-06 gate"; worst measured 1.296 on CRUISE_20_SPORTPLUS). Replay manifests gate only "assist present / direction of response / pause releases >=95 % at zero / restart recovers"; `OUTPUT_PINNED 0/6`. So **there is currently no gate that would detect a V3 regression in rider feel**, only in safety/ownership. This is the main gap (section 5).
6. **Broken / stale test code exists and is not run** [CONFIRMED_LOG]: 10 of 12 `tests/*.js` exit 1 (they read deleted files: `power_curve.c`, `cadence_comp.c`, `assist_dynamics.c`, `assist_extended_boost.c`, ...), `power_revolution_bench_host.c` and `rider_effort_ab_host.c` do not compile, `qs_transition_diag_host.c` (unregistered orphan) FAILS (T6), `run_high_cadence.ps1` TEST B cannot build.
7. **Fast loop is cheap except one suite** [CONFIRMED_LOG]: 74 of 75 host suites run in <=1.0 s each (compile dominated, 0.1-0.9 s; run <=0.3 s). H75 `m820_uncontrolled_iq_safety_host` runs 61 s because the runner compiles without `-O`; the same file at `-O2` runs in 14 s. Whole registry is 108 s serial.
8. **ASan/UBSan cannot run on this Windows toolchain** [CONFIRMED_LOG]: `verify_all.sanitizers_available()` returns False with w64devkit gcc 14.2, so `verify_all` silently prints "ASan/UBSan SKIP". The baseline commit message's PASS therefore very likely did not include sanitizers. A V3 release candidate needs a Linux/WSL (or UCRT/clang) run to close that.
9. **Hidden coupling to V3's likely edit points.** Three SIL builds (`run_sil.py`, `run_electrical_sil.py`, `run_level4.py`) link with `-Wl,--wrap=g53_ad7ec_step` plus `sim/l4/eb74_invocation_observer.c`, and require exactly 131 EB74 invocations of "prehistory" per cold start (L4-PRE-1..8). `run_level4.py::verify_fuzz_contract()` pins literal source text of `sim/l4/virtual_bike_l4.c`. If V3 renames/removes `g53_ad7ec_step` or changes the EB74 readiness model, all three sims fail to link or fail PRE checks before any behavior is judged.
10. **Run-from-repo-root dependency**: H66-H68, H70 open `integration/evidence/evd-tq/TQ-06/host/{boundaries,pas,chain}/*.csv` by cwd-relative path (tracked in the worktree, 65 files). Replay smoke reads the committed static `tests/host/out/RUN_60_ride.csv` (24000 rows) - an old-pipeline trace used only as replay stimulus.

## 1. verify_all.py - steps, flags, measured runtime

Flags: `--quick` (1000 supervisory fuzz instead of 10000, 250 electrical instead of 1000, 25 L4 instead of 100, **no sanitizers**), `--target` (try exact ARM build), `--require-target` (fail if toolchain missing), `--target-variant normal|diagnostic`. No step selector exists.

| # | Step (in order) | Command | Class | Runtime [CONFIRMED_LOG unless "est."] |
|---|---|---|---|---|
| 1 | source manifest completeness | in-process `manifest_gate` (every `src/*.c` must be in `scripts/sources-m820.txt`) | KEEP-CRITICAL (build/package) | <0.1 s |
| 2 | `git diff --check` | in-process | KEEP-CRITICAL (hygiene) | <0.5 s est. |
| 3 | target-tree + BL820 packager self-check | `tools/build_firmware.py --check-only` | KEEP-CRITICAL (build/package) | 0.13 s |
| 4 | independent BL820 container regression | `tests/tools_prepare_m820_bl820.py` | KEEP-CRITICAL (package) | 0.09 s |
| 5 | 75 host suites | `tools/run_host_tests.py` | mixed, see section 2 | 108 s (75/75 PASS, no selector) |
| 6 | Walk/CAN/calibration safety from production text | `tests/test_m820_walk_can_safety.py` (extracts blocks of `main.c` + `CAN_Display.c`, links ride_control + G53 + limits + PAS + battery_trip + pa4) | KEEP-CRITICAL | 2.0 s |
| 7 | whole-pipeline deterministic regression | `tools/run_regression.py` (4 harnesses x 6 RUN scenarios x 3 layers + 14 CRUISE assist-only + burst + RUN_100 determinism) | KEEP-BEHAVIOR (determinism only; metrics are written, not asserted) | 19.4 s |
| 8 | assist ripple attenuation | `tools/analyze_assist_ripple.py` (needs step 7 output) | ADAPT (observation-only, AP2 target retired) | 3.1 s |
| 9 | ripple analyzer refuses bad evidence | `tests/test_ripple_analyzer_rejects.py` | LEGACY-EVIDENCE (tests the analyzer tool, not firmware) | 38.3 s (slow, pure tool self-test) |
| 10 | closed-loop supervisory SIL + fuzz (+ASan/UBSan if available) | `tools/run_sil.py --fuzz 1000\|10000 [--sanitize --sanitize-fuzz 1000]` | KEEP-CRITICAL + KEEP-BEHAVIOR | 4.3 s at fuzz 50; 8.8 s at fuzz 1000; 10000 est. 40-70 s; sanitizer build+run est. +30 s (not runnable here) |
| 11 | real FOC/PMSM/Hall electrical SIL | `tools/run_electrical_sil.py --full-fuzz 250\|1000 [--sanitize]` (standalone FOC + full chain + Walk matrix 126 + Hall sweep 24x2 + stop/reverse axis) | KEEP-CRITICAL | 14.9 s at 250 (4 shards); 1000 est. 40-50 s |
| 12 | Level-4 rider+bike+battery/SOC+FOC | `tools/run_level4.py --fuzz 25\|100 [--sanitize]` | KEEP-CRITICAL + KEEP-BEHAVIOR | 5.4 s quick; 100 est. 15-20 s |
| 13 | recorded-ride replay + stated behaviour | `tools/run_replay_regression.py` (smoke + 6 w1 cases; recompiles `replay_fw` for every case) | KEEP-BEHAVIOR | 26.6 s (3.3 s of it per compile, x8) |
| 14 | every behaviour criterion can reject | `tests/test_replay_behavior.py` | KEEP-BEHAVIOR (infra) | 0.12 s |
| 15 | CANable FW145 decode -> canonical -> replay | `tests/test_canable_ride_decode.py` | KEEP-CRITICAL (diagnostic toolchain; V3 telemetry fields will extend it) | 7.0 s |
| 16 | (with `--target`) M820 stack-gate self-test + exact ARM build | `tools/test_m820_stack_gate.py` + `tools/build_firmware.py --variant <v> --mode developer` | KEEP-CRITICAL (build/stack/RAM) | 20 s + 15 s [CONFIRMED_LOG, normal variant built OK: raw BIN 134068 B] |

Sum of measured steps without target (`--quick`): about 235 s (about 4 min). Full (`verify_all.py`) is est. 6-9 min where sanitizers exist. Diagnostic target variant not built by this audit.

Other runners (not in verify_all): `tests/host/run-host-tests.ps1` (PowerShell twin and *source of truth* of the registry; `run_host_tests.py` regex-parses it and aborts if parsed != declared), `tests/host/run_regression.ps1` (twin of #7), `tests/host/run_high_cadence.ps1` (TEST A builds, TEST B does not), `tests/host/build_version_allocator_host.ps1` (version allocator, tooling governance, PowerShell only), `sim/controller_lab/*` (WWW live analyzer built from the L4 `PROD` module list; dev tool, not a gate), `tools/run_replay.py` (single replay), `tools/analyze_l4_trace.py`.

## 2. Host-suite registry (75) - functional classification

Legend. Linked src: `PIPE*` = g53_port + g53_port_boundaries + g53_port_pas + g53_port_chain + g53_g1_limiter + ap2_limits + assist_pipeline + assist_modes + torque_input + tuning_config + battery_iq_cap + fast_iq_slew; `G53*` = the five g53 files. `text:X` = the suite reads production file X as text (wiring/ownership guard, no execution of X). "(none)" = self-contained model, replica or source-text only. V3-break = will intentional V3 behavior change break it. Runtime measured individually [CONFIRMED_LOG]: all `fast` (<1.0 s: compile 0.1-0.9 s, run <=0.3 s) except H75.
Hxx numbers are registry order in `run-host-tests.ps1`.

| ID | Harness (tests/host/) | Linked src | Contract asserted | Class | V3-break | Runtime |
|---|---|---|---|---|---|---|
| H01 | `stop_click_regression_host.c` | quiet_zero rotor_angle | Hall confidence + stop/restart rotor angle, quiet-zero hand-off | KEEP-CRITICAL | no | fast |
| H02 | `stop_trace_host.c` | stop_trace | passive stop-trace recorder (diagnostic) | LEGACY-EVIDENCE | no | fast |
| H03 | `walk_assist_run_min_host.c` | walk_assist_motor walk_speed_controller | Walk RUN minimum Iq | KEEP-CRITICAL | no | fast |
| H04 | `walk_assist_diag_host.c` | walk_assist_motor walk_speed_controller | Walk no hold timeout + reason bits | KEEP-CRITICAL | no | fast |
| H05 | `fw130_walk_governor_host.c` | walk_assist_motor walk_speed_controller | Walk governor ramp, gear band, wheel fuse | KEEP-CRITICAL | no | fast |
| H06 | `fw131_rotor_angle_host.c` | rotor_angle | rotor angle continuity (FOC) | KEEP-CRITICAL | no | fast |
| H07 | `fw137_erps_edge_age_host.c` | text:main.c | erps edge-age speed ceiling (estimator REPLICA + main.c text guard) | KEEP-CRITICAL | no | fast |
| H08 | `fw1360_click_measurement_host.c` | text:CAN_Display.c text:main.c | click-zone measurement wiring (diagnostic measurement) | LEGACY-EVIDENCE | no | fast |
| H09 | `fw135_update_power_hold_host.c` | text:CAN_Display.c text:main.c | update-session power hold (text guard) | KEEP-CRITICAL | no | fast |
| H10 | `fw101_episode_host.c` | ride_episode | FW-101 episode recorder (diagnostic) | LEGACY-EVIDENCE | no | fast |
| H11 | `fw102_pas_trace_host.c` | pas_trace | FW-102 PAS trace (diagnostic) | LEGACY-EVIDENCE | no | fast |
| H12 | `fw106_recorder_host.c` | pas_raw pas_trace ride_episode | FW-106 recorders (diagnostic) | LEGACY-EVIDENCE | no | fast |
| H13 | `fw106_session_host.c` | diag_session | FW-106 diag session/dump | LEGACY-EVIDENCE | no | fast |
| H14 | `fw117_trace_host.c` | fw117_trace | FW-117 bridge lifecycle trace, START trigger | LEGACY-EVIDENCE | no | fast |
| H15 | `fw117_trace_host.c` | fw117_trace | FW-117 bridge lifecycle trace, STOP trigger | LEGACY-EVIDENCE | no | fast |
| H16 | `fw117_trace_bad_selector_host.c` | (none: self-contained / replica) | build-refusal of invalid selector (must fail to build) | LEGACY-EVIDENCE | no | fast |
| H17 | `fw106_integration_host.c` | diag_session pas_raw pas_trace ride_episode | FW-106 recorders integration | LEGACY-EVIDENCE | no | fast |
| H18 | `pas_quadrature_host.c` | pas_quadrature | PAS quadrature decoder, all 16 pairs | KEEP-CRITICAL | no | fast |
| H19 | `pas_sampler_glitch_filter_host.c` | pas_direction pas_quadrature pas_sampler | PAS glitch filter + direction safety | KEEP-CRITICAL | no | fast |
| H20 | `pas_direction_host.c` | pas_direction | PAS direction automaton (reverse detection), exhaustive | KEEP-CRITICAL | no | fast |
| H21 | `torque/torque_run_asym_host.c` | crank_model torque_input | ARUN asymmetric filter vs S1-S8: filter output is NOT on the active control path (see section 3) | ADAPT | no | fast |
| H22 | `can_tx_queue_host.c` | can_tx_queue | CAN TX queue non-blocking | KEEP-CRITICAL | no | fast |
| H23 | `can_rx_queue_host.c` | can_rx_queue text:CAN_Display.c text:gd32f30x_it.c text:main.c | CAN RX FIFO burst retention + HMI liveness | KEEP-CRITICAL | no | fast |
| H24 | `can_multiframe_host.c` | can_multiframe can_tx_queue | CAN multiframe producer (V3 config transfer may extend) | KEEP-CRITICAL | maybe | fast |
| H25 | `can_reply_effects_host.c` | can_multiframe can_reply_effects can_tx_queue | deferred 0x6029 peak reset reply effects | KEEP-CRITICAL | maybe | fast |
| H26 | `main_startup_wiring_host.c` | text:main.c | pas_direction_init startup order (text guard) | KEEP-CRITICAL | no | fast |
| H27 | `main_standstill_wiring_host.c` | text:main.c | standstill rotor seed wiring (text guard) | KEEP-CRITICAL | no | fast |
| H28 | `fw110_can_blocking_guard_host.c` | text:CAN_Display.c text:main.c | CAN blocking-wait guard (text guard) | KEEP-CRITICAL | no | fast |
| H29 | `walk3202_bit0_guard_host.c` | text:CAN_Display.c | Walk 0x3202 bit0 gate (text guard) | KEEP-CRITICAL | no | fast |
| H30 | `step2a_neutral_dwell_wiring_host.c` | text:main.c | neutral-dwell bridge lifecycle (text guard) | KEEP-CRITICAL | no | fast |
| H31 | `armed_zero_lifecycle_host.c` | text:FOC.c text:main.c | persistent ARMED_ZERO lifecycle (model + text guards) | KEEP-CRITICAL | no | fast |
| H32 | `focaw1_tracking_aw_host.c` | text:FOC.c text:main.c | FOC D/Q tracking anti-windup | KEEP-CRITICAL | no | fast |
| H33 | `foc_current_loop_parity_host.c` | foc_current_loop | FOC current-loop extraction parity (randomized) | KEEP-CRITICAL | no | fast |
| H34 | `stopclick_c1_pi_integral_host.c` | text:FOC.c text:main.c | PI integrator continuity at stop-click | KEEP-CRITICAL | no | fast |
| H35 | `fw139_start_trajectory_host.c` | text:ride_control.c | negative text guard: one startup trajectory owner, no preload in ride_control | KEEP-BEHAVIOR | maybe | fast |
| H36 | `fw140_control_cadence_host.c` | cadence_filter text:main.c | conditioned cadence filter + main.c wiring; cadence feeds telemetry only on the G53 path | ADAPT | maybe | fast |
| H37 | `fw141_torque_elapsed_time_host.c` | torque_input | torque FAST/RUN filters use elapsed hardware time (filters not on active control path) | ADAPT | no | fast |
| H38 | `torque/torque_cal_migration_host.c` | torque_input | torque calibration persistence migration | KEEP-CRITICAL | no | fast |
| H39 | `torque/torque_gain_decoupling_host.c` | torque_input | calibration gain decoupled from kg curve | KEEP-CRITICAL | no | fast |
| H40 | `torque/torque_control_domain_host.c` | ap2_profiles ap2_rider_demand assist_modes torque_input tuning_config | torque control-domain contract D1-D8: D1-D7 live (load_ctrl), D8 effort normalisation exercises DEAD ap2_rider_demand | ADAPT | maybe | fast |
| H41 | `torque/torque_threshold_migration_host.c` | ap2_profiles assist_modes torque_input tuning_config | start-load threshold migration, bank v10 (breaks only if V3 bumps the bank schema) | KEEP-CRITICAL | maybe | fast |
| H42 | `qzero_quiet_zero_host.c` | fast_iq_slew quiet_zero text:FOC.c text:assist_pipeline.c text:main.c text:ride_control.c | Quiet Zero PI fade at Iq_ref=0 (+text guards) | KEEP-CRITICAL | no | fast |
| H43 | `qs3d_16khz_slew_host.c` | fast_iq_slew map_adapter text:FOC.c text:assist_pipeline.c text:main.c text:ride_control.c | 16 kHz final-Iq slew parity vs 4 kHz owner | KEEP-CRITICAL | no | fast |
| H44 | `qs3c_battery_cap_host.c` | battery_iq_cap text:ap2_limits.c text:main.c text:ride_control.c | battery-current limiter ownership upstream (text guard on ride_control/ap2_limits) | KEEP-CRITICAL | maybe | fast |
| H45 | `fw127a_pwm_geometry_host.c` | pwm_geometry | applied PWM geometry clamp | KEEP-CRITICAL | no | fast |
| H46 | `fw127b_sample_ctx_host.c` | current_sample_ctx | atomic current sample context | KEEP-CRITICAL | no | fast |
| H47 | `fw127c_sample_window_host.c` | pwm_geometry sample_window | sampling window from applied geometry | KEEP-CRITICAL | no | fast |
| H48 | `fw127c_wiring_host.c` | text:main.c | acquisition wiring (text guard) | KEEP-CRITICAL | no | fast |
| H49 | `fw127d_current_feedback_host.c` | current_feedback | current feedback validity | KEEP-CRITICAL | no | fast |
| H50 | `fw128a_iq_chain_host.c` | iq_chain text:ap2_limits.c text:main.c text:ride_control.c | canonical q-current owner iq_chain (+ text guard on ride_control/ap2_limits) | KEEP-CRITICAL | maybe | fast |
| H51 | `pre128_pas_timebase_host.c` | pas_cadence pas_quadrature pas_sampler | PAS timebase (sampler + cadence + quadrature) | KEEP-CRITICAL | no | fast |
| H52 | `fw128b1_battery_timebase_host.c` | battery_current text:main.c | battery current timebase | KEEP-CRITICAL | no | fast |
| H53 | `battery_trip_host.c` | battery_trip text:main.c text:ride_control.c | hard battery-overcurrent trip (+ wiring guards) | KEEP-CRITICAL | no | fast |
| H54 | `pa4_buttons_host.c` | pa4_buttons text:main.c | PA4 buttons: power-off, Walk | KEEP-CRITICAL | no | fast |
| H55 | `g53_g1_closed_loop_host.c` | battery_current battery_trip g53_g1_limiter | closed loop limiter + battery tap + hard trip, 36 cases | KEEP-CRITICAL | no | fast |
| H56 | `fw128b0_battery_scale_host.c` | text:ap2_limits.c text:main.c text:ride_control.c | battery current scale (model + text guard) | KEEP-CRITICAL | no | fast |
| H57 | `fw128c0_current_scale_host.c` | text:FOC.c text:main.c | physical current scale (model + text guard) | KEEP-CRITICAL | no | fast |
| H58 | `fw1267_current_cal_host.c` | current_cal | production current calibration | KEEP-CRITICAL | no | fast |
| H59 | `fw1267_cal_wiring_host.c` | text:main.c | calibration wiring (text guard) | KEEP-CRITICAL | no | fast |
| H60 | `ap2_timebase_host.c` | ap2_estimators ap2_rider_demand | elapsed-time invariance of ap2_rider_demand/ap2_estimators: both are linked in firmware but NEVER called from main/ISR at baseline | REMOVE/RETIRE | n/a | fast |
| H61 | `ap2_pipeline_scenarios_host.c` | PIPE* | production pipeline: zero policy, native-cut ownership, limiter chain, 16 kHz owner | KEEP-CRITICAL | maybe | fast |
| H62 | `reverse_ramp_host.c` | PIPE* | G5300-only stop/reverse ramp a-g (release/stop timing of the G53 chain) | KEEP-BEHAVIOR | YES (partly) | fast |
| H63 | `fast_slew_i2_host.c` | PIPE* | fast current-reference slew on PEDAL path; engage/release unchanged check depends on BDE8 rates | KEEP-CRITICAL | maybe | fast |
| H64 | `assist_bank_contract_host.c` | ap2_profiles assist_modes torque_input tuning_config | stored bank round trip / restart / meaning of zero (breaks only on bank-schema change) | KEEP-CRITICAL | maybe | fast |
| H65 | `fw144_soc_core_parity_host.c` | soc_core | SOC core exact parity + randomized | KEEP-CRITICAL | no | fast |
| H66 | `g53_port_boundaries_host.c` | (none: self-contained / replica) | G53 boundaries A-x / EB74 (AD7EC rider-input conditioning) vs oracle vectors | KEEP-BEHAVIOR | maybe | fast |
| H67 | `g53_port_pas_host.c` | g53_port_pas | G53 PAS direction/cadence/timeout vs oracle (96-transition geometry) | KEEP-BEHAVIOR | YES if g53_port_pas changes | fast |
| H68 | `g53_port_chain_host.c` | g53_port_chain g53_port_pas | G53 chain (D7EC/E1E8/BDE8) differential vs oracle chain-reference.csv (128 columns) | KEEP-BEHAVIOR | YES if g53_port_chain changes | fast |
| H69 | `g53_g1_limiter_host.c` | g53_g1_limiter | G53 PI#1 battery limiter bit-exact vs G5300, 300k vectors | KEEP-CRITICAL | no | fast |
| H70 | `g53_port_integration_host.c` | G53* pas_quadrature | G53 integration: PA6/PAS mapping, exhaustive pairs, text checks | KEEP-BEHAVIOR | maybe | fast |
| H71 | `m560_levels_host.c` | G53* | M560 level rise, ratio, power, reset on G53 chain | KEEP-BEHAVIOR | YES (likely) | fast |
| H72 | `m560_auto_splus_host.c` | PIPE* | Sport+ AUTO interpolation, ratio slew, 16 kHz Iq timing | KEEP-BEHAVIOR | YES (likely) | fast |
| H73 | `m560_parser_host.c` | parser | M560 P0/P1 parser round trip, clipping, migration | KEEP-CRITICAL | maybe | fast |
| H74 | `ride_telemetry_host.c` | ride_telemetry | ride telemetry pacing/priority/wire schema (extension point for V3 diagnostics) | ADAPT | maybe | fast |
| H75 | `m820_uncontrolled_iq_safety_host.c` | PIPE* iq_chain map_adapter motor_core pas_direction pas_liveness pas_quadrature pas_sampler ride_control rider_input text:assist_pipeline.c | uncontrolled-Iq safety T1-T7: PA6 throttle, no-rider matrix, hard inhibits, release preserved | KEEP-CRITICAL | YES (partly: T3, T6, T7) | SLOW (61 s at default flags, 14 s at -O2) |

Registry totals (75): KEEP-CRITICAL 51, KEEP-BEHAVIOR 8, ADAPT 5, LEGACY-EVIDENCE 10, REMOVE/RETIRE 1.
Notes on the less obvious calls:
- H21/H37 (ADAPT): real `torque_input.c`, but the RUN/FAST outputs they assert are diagnostics-only at baseline. Keep as a reusable asset if V3 builds its effort estimate on those filters; otherwise they become LEGACY-EVIDENCE. They only break if V3 edits `torque_input.c`.
- H40 (ADAPT): D1-D7 guard the control-domain torque scale (live, KEEP-CRITICAL material); D8 runs `ap2_rider_demand_update` (dead). Split D8 out when AP2 demand code is retired.
- H60 (REMOVE/RETIRE): links only `ap2_rider_demand.c` + `ap2_estimators.c`; I verified by grep that nothing in `src/` calls them. It is safe to retire, but the *method* (same physical interval, dense vs sparse stepping, must agree) is worth porting to the V3 intent module as an elapsed-time invariance test.
- H66-H68, H70 (KEEP-BEHAVIOR, oracle): these are the pinned stock-G5300 differential (EB74, PAS, chain CSV references). They are the contract "baseline == stock G5300". They break only if V3 edits the corresponding `g53_port_*` file. See recommendation R1.
- H61/H63/H75 are KEEP-CRITICAL because the *ownership* contracts (zero policy, native-cut = exact zero in the same update, comm inhibit, ceiling binds immediately, single final-Iq owner, PA6 cannot create demand) must survive V3. Their V3-break flag marks only the scenarios that encode current G53 release/engage timing (H63 `check_no_double_slowing`/`engage_unchanged`, H75 T3/T6/T7, H61 engage helper).
- Source-text guards (`text:main.c`, `text:ride_control.c`, ...) anchor on literal strings; they break on a cosmetic rewrite of those lines even if behavior is kept. Flagged "maybe" only where V3 is likely to touch the text (H35, H44, H50).
- Orphans: `qs_hmi_version_static_host.c` (passes, unregistered), `qs_transition_diag_host.c` (links active `qs_transition_diag.c`; FAILS "T6 power-on auto-arms generation one" at baseline, cause UNKNOWN, unregistered, so verify_all never sees it). Neither involves V3.

## 3. Everything that is not in the 75-suite registry

| ID | Item | What it links / asserts | Class | V3-break | Runtime |
|---|---|---|---|---|---|
| X01 | `tools/run_regression.py` + `pipeline/assist_pipeline_host.c`, `pipeline/ride_control_pipeline_host.c`, `torque/torque_trace_host.c`, `scenarios/missed_tick_burst_host.c` | assist layer = PIPE* + rider_input + ap2_* (dead ones linked) + ride_control/iq_chain/motor_core; drives G53 via `g53_port_update` with a crank model. Asserts only RUN_100 byte-identical re-run; writes metrics_summary.csv | KEEP-BEHAVIOR (baseline trace generator; the CSVs are the A/B material V3 needs) | traces change by design; determinism check stays valid | 19 s |
| X02 | `tools/analyze_assist_ripple.py` | reads X01 CRUISE_* traces; fails on missing/clipped/limiter-shaped/zero evidence; attenuation not gated | ADAPT (metric from the retired AP2 target; usable once V3 sets a gating attenuation target) | no (observation) | 3 s |
| X03 | `tests/test_ripple_analyzer_rejects.py` | tool self-test | LEGACY-EVIDENCE | no | 38 s |
| X04 | `tools/run_sil.py` -> `sim/evist_sil.c` (fast current-loop stand-in) | PIPE*, ride_control, pas_*, cadence_filter, + dead ap2_*; `--wrap=g53_ad7ec_step` + EB74 observer. Scenarios: clean/loaded start, pas_bounce, steady_ripple, cadence_raw/filtered, pedal20..80, stop_restart, stop/reverse AXIS timing, STOP_RESTART cold qualification, DISC-007; fuzz asserts: first permission/Iq/Hall exist, Hall<=250 ms after permission, no false reverse, Iq within [0,PH_CURRENT_MAX], angle error <=31 deg | KEEP-CRITICAL (invariants) + KEEP-BEHAVIOR (start timing) | maybe (link: wrap symbol; PRE-131 EB74 calls; start-timing numbers are printed not asserted) | 4-9 s |
| X05 | `tools/run_electrical_sil.py` -> `sim/foc_electrical_sil.c`, `sim/evist_sil.c` with `EVD_SIL_REAL_FOC` | real FOC.c, foc_current_loop, pwm_geometry, rotor_angle, quiet_zero, Walk + PMSM/Hall plant: SVPWM geometry, 6-sector sweep, Walk matrix 126, Hall start sweep 24x2, stop_restart with QZERO entries==1, fuzz shards | KEEP-CRITICAL | maybe (same link coupling as X04) | 15 s (250) |
| X06 | `tools/run_level4.py` -> `sim/l4/*` | production chain + FOC + SOC + virtual rider/bike/battery; 9 fixed routes (flat/hill/low SOC/sag/cad120) and 8 SOC endurance runs, randomized physics; asserts per-route PASS (speed, Vmin, IbatMax within limit, SOC error bounds) | KEEP-CRITICAL + KEEP-BEHAVIOR | maybe (EB74 observer, source-text pins in `verify_fuzz_contract`; rider model feeds G53) | 5 s quick |
| X07 | `tools/run_replay_regression.py` + `sim/replay/replay_fw.c` + `sim/replay/cases/w1-*` (6 real-bike fragments: stable cad49, cad85, load-rise, load-fall, pause 43 s, pause 10 s) | PIPE* + ride_control + walk; criteria: produces_assist, responds_to_load rising/falling, pause_releases >=95 % zero, restart_recovers; OUTPUT_PINNED 0/6 | KEEP-BEHAVIOR | maybe: f07/f08 pause-release timing is exactly the release-detection layer V3 changes | 27 s |
| X08 | `tests/test_replay_behavior.py`, `tools/replay_behavior.py` | criteria evaluators can reject | KEEP-BEHAVIOR (infra) | no | 0.1 s |
| X09 | `tests/test_canable_ride_decode.py`, `tools/decode_canable_ride_log.py`, `tests/host/ride_telemetry_fixture.c` | production telemetry serializer -> CANable raw log -> canonical -> replay | KEEP-CRITICAL (diag toolchain) | maybe (new V3 telemetry fields) | 7 s |
| X10 | `tests/test_m820_walk_can_safety.py` + `tests/host/m820_walk_can_safety_harness.c` | production main.c/CAN_Display.c blocks (extracted by anchor) + ride_control + G53 chain + limits + pa4 + battery_trip: Walk gates, comms watchdog, 0x6300 level decoder, position calibration | KEEP-CRITICAL | no (maybe if V3 edits those main.c blocks; a moved anchor is a FAIL by design) | 2 s |
| X11 | `tools/build_firmware.py` (+`--check-only`), `tools/m820_stack_gate.py`, `tools/test_m820_stack_gate.py`, `tests/tools_prepare_m820_bl820.py`, `VERIFY_AND_BUILD*.bat` | manifest, memory map, RWE, stack budget (fail-closed incl. known-bad O0/2K firmware), BL820 container | KEEP-CRITICAL | no (V3 adds code: stack/size headroom is the thing to watch) | 0.1-35 s |
| X12 | `tests/fw085_run_window.js`, `tests/fw090_run_attack.js` | text/replica checks of `torque_input.c` RUN window and attack (exit 0 today) | LEGACY-EVIDENCE | no | <1 s |
| X13 | other 10 `tests/*.js` (fw056, fw057, fw058, fw060, fw077, fw078, fw087, fw088, fw092, fw100) | read deleted sources (`power_curve.c`, `cadence_comp.c`, `assist_dynamics.c`, `assist_extended_boost.c`, ...) or missing `#define`s; **exit 1 at baseline**; not referenced by any runner | REMOVE/RETIRE (proven: assert deleted code) | n/a | n/a |
| X14 | `tests/fw016_ride_core_model.ps1` | pure PowerShell arithmetic model of an old ride core, no production link | REMOVE/RETIRE | n/a | n/a |
| X15 | `tests/host/pipeline/power_revolution_bench_host.c`, `tests/host/redesign/rider_effort_ab_host.c`, `run_high_cadence.ps1` TEST B | include `assist_dynamics.h`, `assist_mode_output_t` etc. which no longer exist: **do not compile** | REMOVE/RETIRE (the high-cadence *question* is a gap, section 5) | n/a | n/a |
| X16 | `tests/host/torque/torque_revolution_bench_host.c` (+`run_high_cadence.ps1` TEST A, `tests/host/tools/HighCadenceTools.ps1`) | builds (needs `signal_stats.c`); torque_input filters only | LEGACY-EVIDENCE | no | n/a |
| X17 | `tests/host/emtb_full.py`, `emtb_pu_forensic_sweep.py`, `gen_emtb.py`, `emtb_results.txt`, `torque_asym_s1_s8.csv`, `golden/candidates/metrics_summary.csv` | offline model/forensic scripts of FW-112 eMTB redesign; not wired | LEGACY-EVIDENCE | no | n/a |
| X18 | `tests/stop_trace_decoder.ps1`, `tests/host/build_version_allocator_host.ps1`, `tests/host/emit_bank_fixture.c` | decoders / build-version tooling / bank fixture emitter | LEGACY-EVIDENCE (tooling; allocator test is KEEP-CRITICAL for release numbering, outside the V3 loop) | no | n/a |
| X19 | `sim/controller_lab/*` | live WWW analyzer built from L4 `PROD` list; forced crank/cadence/speed/level inputs | ADAPT (best interactive stimulus for V3 tuning; not a gate) | maybe (module list, `--wrap`) | interactive |

Counts of section 3 (one class per item; X04-X06 are dual KEEP-CRITICAL + KEEP-BEHAVIOR and counted once as KEEP-CRITICAL): KEEP-CRITICAL 6 (X04, X05, X06, X09, X10, X11), KEEP-BEHAVIOR 3 (X01, X07, X08), ADAPT 2 (X02, X19), LEGACY-EVIDENCE 7 (X03, X12, X16, X17, X18 and the two orphan qs_* harnesses), REMOVE/RETIRE 3 groups (X13 = 10 files, X14 = 1, X15 = 3).
Combined with the registry: KEEP-CRITICAL 57, KEEP-BEHAVIOR 11, ADAPT 7, LEGACY-EVIDENCE 17, REMOVE/RETIRE 4 groups (H60 + X13 + X14 + X15 = 15 files). Nothing is classified REMOVE/RETIRE by name or age; each of the four groups was proven by a missing include/source or by the absence of any caller (grep), as stated in its row.

## 4. What V3 will intentionally break (or may)

Will break if V3 modifies the named module (all fast, all localized):
- H62 `reverse_ramp_host` (G53-only stop/reverse ramp) - **yes**, rewrite/adapt; keep its safety half (brake, reverse never positive, zero at standstill).
- H67 `g53_port_pas_host` (oracle PAS), H68 `g53_port_chain_host` (oracle chain), H66 `g53_port_boundaries_host`, H70 `g53_port_integration_host` - break **only** if V3 edits the matching `g53_port_*.c`. If V3 adds a new module above/beside G53 and keeps G53 selectable these stay green as baseline oracle.
- H71 `m560_levels_host`, H72 `m560_auto_splus_host` - likely (level rise/ratio/AUTO slew are chain behavior V3 may re-time).
- H63 `fast_slew_i2_host` (partial), H61 `ap2_pipeline_scenarios_host` (partial), H75 `m820_uncontrolled_iq_safety_host` T3/T6/T7 (partial; T1/T2/T4/T5 are pure safety and must stay unchanged).
- H41/H64/H73 only if V3 bumps the bank schema or adds config fields.
- X07 replay f07/f08 `pause_releases` thresholds (release timing), X01 trace shapes (not asserted), X04-X06 only through link/PRE coupling (`--wrap=g53_ad7ec_step`, 131-call EB74 prehistory, `verify_fuzz_contract` text pins) and through the printed (not asserted) start-timing numbers.

Should NOT break (if one does, V3 has a safety/ownership regression): H1-H6, H18-H20, H22-H34, H38-H39, H42-H59, H65, H69, X05, X10, X11, and H75 T1/T2/T4/T5.

## 5. Gaps - coverage V3 needs and does not have

1. **No gate on rider feel.** run_regression/analyzer/replay are observation or "response in the right direction". V3 needs asserted metrics on a controlled stimulus (time-to-assist after load, release-to-zero time, ripple attenuation with a stated limit, carry hold time) and a baseline-vs-candidate differential on identical inputs (freeze rule). The only A/B harness (`redesign/rider_effort_ab_host.c`) is broken. Smallest fix: a script that runs X01 + X07 on two worktrees (`25df554` vs candidate) and diffs `metrics_summary.csv` and replayed CSVs with stated tolerances.
2. **Release detection**: covered only by H62 (stop/reverse), SIL stop/reverse AXIS timing, and two replay pauses (43 s, 10 s). Missing: short releases (0.2-2 s) while rolling at speed, release at each cadence/speed/level, partial release (load drop without PAS stop), back-pedal then resume, release-then-obstacle.
3. **Obstacle carry**: zero coverage. No stimulus has a load spike at ~0 cadence or wheel-speed dip (curb, step, stall), nor a recorded obstacle ride (the only real rides are six w1 fragments; no CANable ride case is registered).
4. **Phase-aware intent**: `tests/host/common/crank_model.c` + `scenario_profiles.h` give crank-angle-resolved torque and PAS at any cadence and are reusable, but nothing asserts phase behavior. Existing attenuation is non-gating.
5. **Cadence matrix**: CRUISE_20/40/60 for four profiles and 80/100 only for SPORT, RUN_60..120, SIL pedal20..80. No gating matrix over cadence x speed x level x load, nothing below 20 or above 120 rpm, and the high-cadence bench (TEST B) is broken.
6. **Safety ownership at the new layer**: H75 T2/T3 (no rider load => Iq exactly 0 across PA6 x speed x level x PAS pattern; `!forward_valid` never raises Iq) are written against `assist_pipeline_update`. They must be ported/re-run against the V3 module API before V3 can claim ownership parity.
7. **Config/CAN for V3 parameters**: H64, H73, H41 and H24/H25 need new round-trip, clipping and migration cases; merge point with local commits `e2868f6` (0x6011 from node 3) and `0c04f24` (0x02F83203 broadcast) noted in BASELINE_FREEZE.md. Telemetry (H74, X09) needs V3 intent-state fields.
8. **Sanitizers** cannot run on this machine's toolchain (see 0.8).
9. **Timing/CPU budget**: no host or target measurement of 4 kHz loop cost; stack gate and map checks exist, execution time does not. Hardware gate only.
10. **Diagnostic target variant** is only built by a second `verify_all --require-target --target-variant diagnostic` invocation (repeats all PC steps) or by `build_firmware.py --variant diagnostic` directly.
11. **Dead AP2 demand modules** (H60 subject) still ship in firmware and in all four sim builds. Capture as a TECH candidate (RULE 42), do not chase inside V3.

## 6. Gate proposal

Durations are measured pieces from this audit (serial, this PC). `$TC` = `"C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\13.2 Rel1\bin"`. Run everything from the repo root with `CC=gcc` (w64devkit on PATH).

### Selector gap and smallest change

`tools/run_host_tests.py` accepts **no arguments**: no subset, no list, no parallelism, no `-O`. verify_all, run_sil, run_electrical_sil, run_level4 already have their own size knobs (`--fuzz`, `--full-fuzz`, `--quick`, `--jobs`); run_regression, run_replay_regression, analyze_assist_ripple have none.

Smallest change (not implemented; about 30 lines, default behavior unchanged, parsed==declared guard kept and applied before filtering):
- `--list` print `ID | name | harness` and exit.
- `--only REGEX` (repeatable, matched case-insensitively against suite name and harness path) and `--exclude REGEX`.
- `--group NAME` as sugar over `--only`, defined in a dict at the top of the file, suggested:
  - `g53` = `g53_|m560_|reverse_ramp|fast_slew|ap2_pipeline_scenarios|m820_uncontrolled`
  - `safety` = `m820_uncontrolled|battery_trip|g53_g1|qs3c|fw128a|armed_zero|stopclick|qzero|pas_direction|pas_quadrature|pas_sampler`
  - `config` = `assist_bank|torque_cal|torque_threshold|torque_gain|m560_parser|torque_control_domain`
  - `can` = `can_|fw110|fw135|fw1360|walk3202`
- `-j N` run suites in a `ThreadPoolExecutor` (exe names are already unique per index; keep result order by index).
- `--opt O2` append `-O2` to the compile line (H75: 61 s -> 14 s; whole registry would drop from 108 s to roughly 25-30 s with `-j 4 --opt O2`, estimate).
Optional, same spirit: `run_replay_regression.py` recompiles `replay_fw` 8 times (about 3.3 s each); compile once and pass the exe to `run_replay.py` saves about 20 s.

### GATE 0 - edit loop (target 10-20 s)
```
# 1. compile check of the touched module (existing, per file, ~0.3 s)
gcc -std=c11 -Wall -Wextra -Werror -Wno-type-limits -Iinc -Itests/host/common/host_stubs -Itests/host/common -fsyntax-only src/<module>.c
# 2. targeted host suites  [NEEDS the selector; ~6-10 s with -j 4 --opt O2, H75 excluded]
python tools/run_host_tests.py --group g53 --exclude m820_uncontrolled --opt O2 -j 4
# 3. supervisory SIL, small fuzz (existing flag, ~4-5 s; also proves link + EB74 coupling)
python tools/run_sil.py --fuzz 100
```
Without the selector, step 2 has no supported form; the only fallback is a hand-written `gcc` line copied from the registry (e.g. H62: `gcc -std=c11 -Wall -Wextra -Werror -Wno-type-limits -Iinc -o .build/h62.exe tests/host/reverse_ramp_host.c src/g53_port.c src/g53_port_boundaries.c src/g53_port_pas.c src/g53_port_chain.c src/g53_g1_limiter.c src/ap2_limits.c src/assist_pipeline.c src/assist_modes.c src/torque_input.c src/tuning_config.c src/battery_iq_cap.c src/fast_iq_slew.c -lm`).
Optional Gate 0+ (15 s, existing): `python tools/build_firmware.py --variant normal --mode developer --toolchain $TC --output-dir .build/dev` (ARM compile + stack gate + BL820 container).

### GATE 1 - feature gate (about 1.5-2 min, no full verify_all)
Rider intent, cadence matrix, release, carry, G53/pipeline, safety ownership.
```
python tools/run_host_tests.py --group g53  --opt O2 -j 4          # incl. H75 (14 s at -O2); selector needed
python tools/run_host_tests.py --group safety --group config --opt O2 -j 4   # selector needed
python tests/test_m820_walk_can_safety.py                          # 2 s
python tools/run_regression.py                                     # 19 s, traces for A/B diff
python tools/analyze_assist_ripple.py                              # 3 s (observation until V3 sets a limit)
python tools/run_replay_regression.py                              # 27 s (release/pause cases)
python tools/run_sil.py --fuzz 300                                 # ~6 s
python tools/run_level4.py --quick                                 # 5 s
```
New V3 scenario packs (to be written; section 5) belong here: release matrix, cadence x speed x level matrix, obstacle-carry stimulus, no-rider-no-Iq property at the V3 API.
Without the selector, run the full registry (`python tools/run_host_tests.py`, 108 s) and the Gate becomes about 3.5 min.

### GATE 2 - milestone integration (about 4-5 min)
Existing quick mode; nothing to add to the code:
```
python tools/verify_all.py --quick --target
```
Measured sum of its steps: about 235 s PC-side (ripple-rejects 38 s and host registry 108 s dominate) + 35 s target (stack-gate self-test 20 s + ARM build 15 s). Quick mode skips sanitizers and uses 1000/250/25 fuzz. For a faster milestone: Gate 1 + `python tools/run_host_tests.py --opt O2 -j 4` (selector) + `python tools/run_electrical_sil.py --full-fuzz 250` + `python tests/test_canable_ride_decode.py`.

### GATE 3 - release candidate
```
python tools/verify_all.py --require-target                                   # NORMAL, 10k/1000/100 fuzz
python tools/verify_all.py --require-target --target-variant diagnostic       # DIAGNOSTIC (repeats PC steps; cheaper: build_firmware.py --variant diagnostic --mode developer --toolchain $TC)
CC=<compiler with libasan/libubsan> python tools/verify_all.py                # sanitizers; NOT possible with w64devkit - run on Linux/WSL/clang and record the result
python tests/test_canable_ride_decode.py                                      # CANable decode (also inside verify_all)
python tests/tools_prepare_m820_bl820.py                                      # package
```
Config round trip = H38-H41, H64, H73 (+ new V3 cases) inside the registry run. Plus the new V3 scenario packs, the baseline-vs-candidate A/B report, and the hardware gate required by the freeze rules (no ride-feel claim without bike evidence; pinning replay outputs `accepted_output_sha256` only after bike validation). Estimated 6-9 min PC-side where sanitizers exist (not measured end to end by this audit).

## 7. Recommendations (not executed - this audit is read-only)

- R1 (decision for owner): keep `g53_port_*` untouched as a selectable *baseline mode* and implement V3 as a new module + switch. Then H66-H72 remain the stock-G5300 oracle with no edits, V3 gets new suites, and "baseline vs V3 on identical inputs" is a one-flag A/B. If V3 edits `g53_port_*` in place, H62/H67/H68/H71/H72 must be rewritten in the same PR and the oracle moves to `LEGACY-EVIDENCE`.
- R2: retire (move to `archive/`, do not delete - RULE 53) H60, the 10 stale `tests/*.js`, `fw016_ride_core_model.ps1`, `power_revolution_bench_host.c`, `rider_effort_ab_host.c`, `run_high_cadence.ps1` TEST B; register or fix the failing orphan `qs_transition_diag_host.c` (separate TECH item).
- R3: add the selector (section 6) first; it is the single change that makes the dev loop fast, and it is independent of V3 design.
- R4: before V3 touches `g53_ad7ec_step` or EB74 readiness, plan the three SIL harnesses (`--wrap`, observer, 131-call prehistory, `verify_fuzz_contract` pins) as part of the same change.
- R5: obtain a sanitizer-capable run environment for Gate 3 now; the baseline's own claim cannot be reproduced on this machine for that step.

## 8. What this audit ran (all on baseline 25df554, all PASS)

`run_host_tests.py` 75/75 (108 s) and per-suite timing; `run_regression.py`; `analyze_assist_ripple.py`; `run_replay_regression.py` 6/6 accepted; `run_sil.py --fuzz 1000` and `--fuzz 50`; `run_electrical_sil.py --full-fuzz 250`; `run_level4.py --quick`; `build_firmware.py --check-only`; `tests/tools_prepare_m820_bl820.py`; `tests/test_m820_walk_can_safety.py`; `tests/test_ripple_analyzer_rejects.py`; `tests/test_replay_behavior.py`; `tests/test_canable_ride_decode.py`; `build_firmware.py --variant normal --mode developer` (to `.build/audit-target`); `test_m820_stack_gate.py`. Also ran (not part of any gate): 12 `tests/*.js` (10 fail), two orphan host harnesses (1 fails), compile probes of two broken benches, an `-O2` rebuild of H75. **Not run:** `verify_all.py` itself, DIAGNOSTIC target, sanitizers (unavailable), PowerShell runners.
`git status` after the audit shows only `docs/assist-v3/` untracked; `.build/` is ignored.
