# Assist Behavior V3 — Audit C: Simulation / Bench Capability

| Field | Value |
|---|---|
| AUDIT | C_SIMULATION (read-only algorithm/simulation audit) |
| WORKTREE / BRANCH / HEAD | `C:\Projekty\eVistDrive-assist-v3` / `feature/assist-behavior-v3` / `25df554` (frozen baseline) |
| TIMESTAMP | 2026-10-07T12:25:51+02:00 |
| STATUS | READY_FOR_REVIEW (findings, not decisions) |
| Files modified | this report only. No commit. `tools/verify_all.py` not run. |

All claims are `[CONFIRMED_CODE]` (read in this session) unless labelled otherwise. Runtimes are
`[CONFIRMED_LOG]` from builds/runs executed in this session (scratch dir, worktree untouched).

---

## 0. Executive summary

* **Primary harness for the V3 cadence × profile matrix: `sim/evist_sil.c` (supervisory SIL).**
  It is the only open-loop harness that runs the *production PAS front end* (`pas_sampler` edge
  events with 4 kHz ISR timestamps, `pas_direction`, `pas_liveness`/real-stop, `pas_cadence`,
  `cadence_filter`) **and** the full `ride_control_update()` → `assist_pipeline` → G53 → `ap2_limits`
  → `fast_iq_slew` chain, with reverse, PAS-bounce glitch, stop/restart and stage-timed safety axes
  already implemented, deterministic, ~140× real time.
* **Secondary harness for closed-loop bike dynamics (carry / crest / technical climb / energy):
  `sim/l4/virtual_bike_l4.c` (Level-4)** — after fixing one modelling defect: the crank is
  kinematically locked to the wheel whenever the motor pushes, so "pedal stop while motor carries"
  is physically impossible in L4 today (§2.3, finding F-L4-1).
* **Keep `tests/host/pipeline/assist_pipeline_host.c` + `tools/analyze_assist_ripple.py` as an
  unchanged regression gate** (Iq-ripple attenuation, already in `verify_all`), not as the V3 host.
* Several benches named in the brief are **stale and do not build** against the current tree
  (`run_high_cadence.ps1` TEST B, `power_revolution_bench_host.c`, `rider_effort_ab_host.c`,
  `sim/controller_lab/controller_lab.c`, `tests/host/run_regression.ps1`, `emtb_*.py`). Their
  *scenario ideas* (torque step, stop/restart, PAS edge drop/jitter, foreground stall) should be
  ported into the SIL, not revived.
* No second simulator is needed. Extension estimate: **~2.0–3.0 kLOC across ~8–10 files**.

---

## 1. Capability matrix

### 1.1 Live (builds on 25df554, verified this session)

| Harness | Production code linked | Time base | Per-edge PAS timing | Crank-angle torque | Rider model | Bike dynamics | Motor / electrical | Battery | Brake / reverse / glitch | Determinism | Runtime (this session) | Output | Metrics already computed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **assist_pipeline_host** `tests/host/pipeline/assist_pipeline_host.c` | torque_input, rider_input, assist_modes, tuning_config, ap2_*, g53_port*, g53_g1_limiter, ap2_limits, battery_iq_cap, fast_iq_slew, assist_pipeline (`tools/run_regression.py:82-85`) — **no** ride_control, **no** pas_sampler/direction/liveness | 4 kHz, `elapsed_ticks=1` always (`:289`) | AB level only, from `crank_model` step count (`:257`); no edge events, no `real_stop` (forward_valid forced true `:274`) | yes, `crank_model` half-sine/leg, ripple, **asymmetry**, **dead-spot depth/width**, phase shift (`tests/host/common/crank_model.c:116-160`) | open loop; constant cadence or linear ramp (`:111-133`) | none; speed fixed 15 km/h (`:75`) | none (u_abs fixed 1024) | none (fixed 42 V) | none | yes; byte-identical rerun asserted (`tools/run_regression.py:111-116`) | build 2.5 s; 8 s scenario 0.15 s incl. 5.5 MB CSV | CSV every 4 kHz tick, ~60 cols incl. G53 trace | per-column mean/p5/p95/ptp/ripple (`run_regression.py:60-73`); **Iq ripple attenuation** gate (`tools/analyze_assist_ripple.py`) |
| **ride_control_pipeline_host** `tests/host/pipeline/ride_control_pipeline_host.c` | assist chain + ride_control, iq_chain, motor_core | 4 kHz, elapsed=1 | AB level from crank_model | crank_model fixed shape | open loop (RUN_60..120, ramp) | none | none | none | none | yes | <0.2 s/scenario (est., same class) | CSV; 3 of its columns are frozen DEPRECATE_ZERO fields | regression summary only |
| **evist_sil** `sim/evist_sil.c` (`tools/run_sil.py`) | full supervisory chain: torque_input … assist_pipeline, ride_control, fast_iq_slew, battery_iq_cap, iq_chain, motor_core, **pas_quadrature/direction/liveness/sampler/cadence**, cadence_filter (`run_sil.py:9-19`); optional real FOC with `-DEVD_SIL_REAL_FOC` (`tools/run_electrical_sil.py:44`) | 4 kHz ctrl + 4×16 kHz inner; elapsed=1 (`:742`) | **yes**: physical crank rev → 96 quadrature transitions/rev, edges at exact crossing ticks (`:494-531`), through production `pas_sampler_isr_tick/pop` (`:588-621`) | 2/rev sine only `mean + ripple·sin(4π·rev)` (`:546`); **no asymmetry, no dead-spot shaping** | constant rpm + **intra-rev cadence ripple** (dead-spot slow-down, `:514`), torque ramp, `active`(stop), `direction`(reverse) | none; wheel speed a constant per scenario (`:946`) | rotor plant (Iq → ERPS, Hall age); PMSM dq + real FOC under flag | none (battery current 0, `:713-716`) | reverse: yes (`:1127-1211`); PAS bounce every 7th edge (`:527`); brake: **no** (`safety_cut_non_direction` never set) | yes; fixed fuzz seed `0xE7157A39` | build 3.3 s; all fixed scenarios + stop/restart + axis **0.54 s** | CSV at 1 kHz (`:774`), ~57 cols incl. G53 internals; stdout summary | start latency, steady Iq mean/std/pp, stop→Iq0, restart permission→Iq, max rise/fall step, **stage-timed stop/reverse axis** (edge→detect→permission→target→ref0→current0, peak drive after detect) (`:1214-1266`) |
| **virtual_bike_l4** `sim/l4/virtual_bike_l4.c` (`tools/run_level4.py`) | everything in SIL + FOC, foc_current_loop, pwm_geometry, rotor_angle, quiet_zero, soc_core, walk (`run_level4.py:17-27`) | 4 kHz ctrl + 16 kHz FOC, 8 plant sub-steps; elapsed=1 (`:439`) | **yes**, `crank_rev × 96` from bike state (`:358-368`) through production pas_sampler | `mean·(1+ripple·sin2φ+asym·sinφ)`, floor 0 = natural dead spot (`sim/l4/bike_rider.c:91-114`); asymmetry yes, dead-spot depth/width no | P-controller on cadence error around a target, base torque (`bike_rider.c:98-100`); scenario is **static** (no events) | **yes**: mass, Crr, CdA, grade, gear, drivetrain eff. (`bike_rider.c:116-146`); grade constant per scenario; no obstacle/crest | real FOC + PMSM dq; motor crank torque = Iq·0.075 Nm (`:56`) | **yes**: OCV/SOC/R0/dynamic sag, two chemistries, fw soc_core closed loop (`sim/l4/battery_pack.c`) | brake field exists (`:439`) but never set; no reverse; PAS bounce every 11th transition (`:364`) | yes; fuzz seed `0x144B1CE5`, golden vector signatures | build 4.3 s; 9×8 s fixed + SOC endurance **0.67 s** | CSV decimated to 200 Hz (`:480`), 20 cols; stage trace for flat_soc90 | max speed/Ibat/Iq, Vmin, SOC error, battery/speed-limit ticks, Hall angle error, start classification |
| **replay_fw** `sim/replay/replay_fw.c` (`tools/run_replay*.py`) | assist chain + ride_control (`replay_fw.c:8-18`) | recorded row dt (W1 ≈ 32 Hz), catch-up via elapsed (`:150-219`) | **synthetic** uniform PAS from cadence (`:156-176`); W1 cases have no `pas_ab` | recorded torque (32 Hz) — real ripple, aliased at high cadence | recorded | recorded wheel speed only | none (u_abs knob `REPLAY_U_ABS`) | recorded V/I | brake column supported (`safety_cut_non_direction`) | yes | fast (240-row cases) | CSV with new vs recorded deltas | `tools/replay_behavior.py` direction/timescale criteria |
| **foc_electrical_sil** `sim/foc_electrical_sil.c` | FOC, foc_current_loop, pwm_geometry only | 16 kHz | n/a | n/a | n/a | n/a | PMSM | n/a | n/a | yes | — | stdout | FOC tracking/clamp — **irrelevant to V3** |
| **torque_revolution_bench_host** (TEST A) | torque_input only | 4 kHz | crank_model steps | crank_model full shape | open loop | none | none | none | none | yes | builds (checked) | per-rev + **96-bin phase-binned** CSV | per-revolution stats, phase-binned FAST/RUN — reusable *methodology* for template evaluation |
| **torque_run_asym_host** (registered suite) | torque_input only | 4 kHz | crank_model | crank_model | S1..S8 trajectories (const, sharp rise, ramps, sine, sharp/gradual release, dead-band osc) × 20/40/60/80 rpm | — | — | — | — | yes | — | `torque_asym_s1_s8.csv` | first-positive, rise10/50/90, decay, ripple — reusable metric code |

### 1.2 Stale — do not build at 25df554 (verified)

| Item | Failure | Note |
|---|---|---|
| `tests/host/run_high_cadence.ps1` | references removed `cadence_comp.c, power_curve.c, assist_start.c, assist_extended_boost.c, assist_dynamics.c, assist_limits.c` (`:79-86`) | TEST A alone still builds |
| `tests/host/pipeline/power_revolution_bench_host.c` | `assist_mode_output_t` no longer exists (`:201`) | legacy assist_modes math |
| `tests/host/redesign/rider_effort_ab_host.c` | `assist_dynamics.h` missing | design idea (BASE + capped PEAK effort) still interesting |
| `sim/controller_lab/controller_lab.c` | `power_curve.h`, `ride_session.h` missing (`:27-29`) | has the richest **event generator** (torque step, stop/restart, PAS edge drop/jitter, foreground stall/missed ticks; `sim/controller_lab/README.md` "Transient/Disturbance events") — port the generator, not the harness |
| `tests/host/run_regression.ps1` | same removed modules (`:117-119`) | superseded by `tools/run_regression.py` |
| `tests/host/emtb_*.py`, `gen_emtb.py` | pure-Python replicas of pre-G53 eMTB math | not production-linked; discard for V3 |

---

## 2. Production context that constrains the harness

### 2.1 Assist task rate and CPU context

| Fact | Evidence |
|---|---|
| MCU: GD32F30x HD, Cortex-M4F (fpv4-sp-d16, hard-float), **120 MHz** | `Firmware/CMSIS/GD/GD32F30x/Source/system_gd32f30x.c:57`; `tools/build_firmware.py:197-199` |
| RAM 48 KB, Flash 230 KB (app) | `ldscripts/gd32f30x_flash.ld:5,9` |
| TIMER1 ISR at 4 kHz increments `control_time_ticks`, sets `reg_ADC_flag`, **samples PAS A/B in the ISR** → `pas_sampler_isr_tick()` | `src/main.c:2362-2392` |
| **Assist task = `ride_control_update()` called from `reg_ADC_processing()` in the foreground `while(1)`**, once per processed 4 kHz tick, `elapsed_ticks = control_delta` (≥1, catch-up) | `src/main.c:3383-3427`, `:2700-2735` |
| G53 chain runs at **1 kHz logical** (4 control ticks per step), catch-up capped at 64 steps; on catch-up the held `pas_ab` is reused — "lost edge history is never reconstructed" | `src/g53_port.c:7,111-130` |
| Final Iq slew at 16 kHz in the FOC ISR (`ADC0_1_IRQHandler`), centre-aligned `_T=3750` @120 MHz | `src/main.c:3845,4152`; `inc/config.h:19,550` |
| Foreground budget: 250 µs = **30 000 cycles per 4 kHz tick**, minus 16 kHz FOC ISR load and other ISRs | derived |
| Foreground already overruns: comment records **12 543 missed-tick events in one 88 s session** | `src/main.c:296-300` — `[DOCUMENTED]` in code comment, not re-verified here |
| **Firmware compiles at `-O0`** except the G53 set (`g53_port*.c`, `g53_g1_limiter.c` at `-O2`); `-fstack-usage` feeds the stack gate | `tools/build_firmware.py:27-30,203,209`; `tools/m820_stack_gate.py` |

Implications for V3 / harness:
1. V3 must be **elapsed-time correct** (consume `elapsed_ticks`, never count calls) and should do
   phase work **per PAS edge event** (≤ 208 edges/s at 130 rpm, ≤ 1 edge per 4 kHz tick per
   `inc/pas_sampler.h:42-45`) or at the 1 kHz logical tick — not heavy work per 4 kHz call.
2. A new V3 source must either join `G53_OPTIMIZED_SOURCES` or be costed at `-O0`.
3. Phase source should be the **ISR-timestamped `pas_step_event_t {tick, gap, step}`**
   (`inc/pas_sampler.h:57-62`), which survives foreground stalls; the G53 1 kHz `pas_ab` sample does
   not. Today `assist_pipeline_input_t` carries only `pas_ab` level (`inc/assist_pipeline.h`), so an
   edge/phase input must be plumbed — and the harness must deliver the same events.
4. Harness gap: **every harness hardcodes `elapsed_ticks=1`** (pipeline host `:289`, SIL `:742`,
   L4 `:439`); only replay and the stale controller_lab exercise catch-up. Foreground-stall
   injection must be added to the primary harness.

### 2.2 Plumbing divergences harness ↔ production (matter for carry score inputs)

| Signal | Production `main.c` | SIL | L4 | Impact |
|---|---|---|---|---|
| `ride_control_input_t.current_iq` ("motor Iq") | **`MS.i_q_setpoint` (reference)** `src/main.c:3405` | plant measured `iq_actual` (`sim/evist_sil.c:724-730`) | measured `ms.i_q` (`virtual_bike_l4.c:433`) | a V3 carry score using "motor Iq" sees reference on the bike but measurement in sims → must be decided and made identical |
| wheel speed | `Speed_processing()` in `main.c`: **1 pulse/wheel rev**, period-based, max-accel rejection, decay cap after 0.1 s silence, hard zero after 2.65 s (`src/main.c:1578-1586, 2637-2675`; `inc/config.h:159,187`) | constant | **exact physics speed every tick** (`:404`) | wheel dv/dt in L4 is unrealistically clean; at 10 km/h one wheel pulse every ~0.8 s. A wheel-sensor model mirroring `Speed_processing()` is required (it is not a linkable unit — see TECH candidate in §7) |
| brake | folded into `safety_cut_non_direction` with overtemp/torque fault/calibration (`src/main.c:3381-3382`) | not exercised | field present, never set | V3 cannot see "brake" separately; carry cancel must key off safety cut (already a hard cut) |
| PAS phase | ISR edge events available in main | same events via `process_pas` | same | pipeline host has no edge events |

### 2.3 Finding F-L4-1 — L4 cannot represent "pedal stop while motor carries" or reverse

`evd_bike_step()` (`sim/l4/bike_rider.c:126-145`): when `drivetrain_engaged` the crank rpm is
*derived from wheel speed*; `l4_tick` sets `engaged = rider.pedaling || motor_torque>0.1`
(`virtual_bike_l4.c:464`). So whenever the motor pushes, the cranks turn and PAS edges continue —
a mid-drive with crank freewheel (M560) does not do that. Consequences: PEDAL_STOP during carry is
unobservable, carry termination on "pedal stop" can never trigger, reverse pedalling is impossible
(`crank_rev` only increases). **Fix (plant only):** separate crank state from drivetrain — motor
torque always reaches the wheel; rider torque reaches the wheel only while the crank is engaged;
crank speed follows wheel-derived speed only while the rider pedals, otherwise free (decay to 0 or
scripted reverse). ~60–100 LOC in `bike_rider.c/.h` + `virtual_bike_l4.c`.

### 2.4 Other facts relevant to V3 design

* 96 transitions/rev is `[INFERRED]` from arithmetic on field-correct cadence (`inc/config.h:687-717`,
  FW-086), not a bench measurement. 12/24/32 bins all divide 96 (8/4/3 edges per bin).
* PAS stop timeout adapts 2×last gap within [800, 2000] ticks = 200–500 ms (`inc/config.h:728-729`,
  `sim/evist_sil.c:596-600` mirrors main). At 20 rpm one edge every ~31 ms. This bounds PEDAL_STOP
  latency in the baseline; V3 TRUE_RELEASE must beat it via torque, not PAS.
* Baseline stop latency measured in SIL this session: **edge→detect 197.75 ms, permission→target
  834 ms, event→current0 1033.5 ms** at 60 rpm, Iq 585; reverse total 174.75 ms
  (`[CONFIRMED_LOG]`, SIL AXIS lines). These are the baseline numbers V3 TRUE_RELEASE / PEDAL_STOP
  should be compared against.
* Float determinism risk: host x86-64 vs arm-none-eabi may differ if the compiler contracts to FMA
  at `-O2`; V3 should use integer/Q-format or `-ffp-contract=off` so host results transfer.

---

## 3. V3 scenario matrix → host and minimal extension

Cadences: 20, 25, 30, 40, 60, 80, 100, 110, 120, 130 rpm. Current coverage: crank_model sets
stop at 120 (`tests/host/common/scenario_profiles.h:91`); SIL fixed set 20–80; L4 up to 120. 25/30/110/130 are new
points only (no code limits found — `crank_state_advance_tick` handles >1 step/tick).

| Profile | Best host | What exists | Minimal extension |
|---|---|---|---|
| steady | SIL (matrix) | 2/rev sine rider | swap torque shape to `crank_torque_raw_mv()` shape (link `tests/host/common/crank_model.c`); cadence list |
| strong dead spot | SIL | intra-rev cadence ripple (`:514`) | crank_model `dead_spot_depth/width` (exists) + couple cadence slow-down to same angle |
| L/R asymmetry | SIL | — in SIL; crank_model has `asymmetry_pct` | same as above |
| gradual real release | SIL | torque ramp up only (`:533-547`) | scripted mean-torque segment (ramp down over N revs); ground-truth label `release_start` |
| sudden TRUE_RELEASE | SIL | — | scripted step to ~0 torque while crank keeps turning (`torque_active` exists; `active` stays true) |
| strong attack | SIL | — (controller_lab had `torque_step_*`) | scripted step-up + optional cadence rise |
| attack → PEDAL_STOP | SIL (classification) + L4 (physics) | `active=false` stop path, axis timing | script segment sequence; L4 needs F-L4-1 fix |
| attack → PEDAL_STOP → restart | SIL | `run_stop_restart_scenario` (`:900-1085`) | generalise to scripted restart at any cadence/phase; restart-continuity metric |
| technical climb (high effort, high motor load, ~0 accel) | **L4** | grade, battery, real FOC | time-varying grade(t); keep cadence low (gear); label |
| crest (effort high→falling load + rising speed) | **L4** | — | grade(t) sign change; rider effort script tied to segment |
| obstacle carry (step/impulse) | **L4** | — | road-force impulse / short grade spike at distance d; rider stops pedalling before it (needs F-L4-1); wheel-pulse sensor model |
| coast | SIL + L4 | `active=false` with wheel speed held | scripted wheel speed (SIL) / L4 natural decel |
| reverse | SIL | `direction=-1` axis (`:1127-1211`) | allow reverse at scripted time in matrix mode |
| brake | SIL (+L4) | not exercised; plumbing exists | set `in.safety_cut_non_direction` from script; add `brake` axis to `report_axis` |
| PAS glitches | SIL | bounce every 7th edge (`:527`) | add edge drop, edge timing jitter (±k ticks), illegal 2-bit jump, burst; port controller_lab's definitions |
| foreground stall / missed ticks (cross-cutting) | SIL | none (elapsed=1) | `fg_stall` script event: keep ISR-side PAS sampling, call `ride_control_update` with `elapsed=N+1` |
| torque sensor noise (cross-cutting) | SIL | none | seeded additive noise on raw mV (xorshift already in file) |
| IMU `valid=false` invariance (cross-cutting) | SIL | — | feed garbage IMU fields with `valid=false`; require byte-identical output vs zeroed IMU |

**Recommended shape of the extension (one generator, no new simulator):**

1. `tests/host/common/rider_script.[ch]` (new, ~300–400 LOC): time/angle-segmented script
   (`cadence(t)`, `torque mean(t)` + `crank_torque_shape_t`, `wheel_speed(t)` or "bike" coupling,
   events: stop, restart, reverse, brake, glitch {bounce, drop, jitter, illegal}, fg_stall,
   noise seed, IMU garbage) and **ground-truth labels** emitted per tick
   (`gt_class ∈ {NORMAL, DIP, TRUE_RELEASE, PEDAL_STOP, …}`, `gt_event_id`, crank angle, cumulative
   angle, true phase offset). Text/CSV script files under `tests/host/scenarios/assist_v3/`. Shared by
   SIL and L4 so both harnesses read identical scenario definitions.
2. `sim/evist_sil.c`: new `--script <file>` / `--v3-matrix` mode; rider plant uses the script and
   `crank_model` shape; brake/stall/glitch hooks; extra CSV columns (ground truth + V3 telemetry)
   **only in the new mode** so existing scenario CSVs stay byte-identical. ~400–600 LOC.
3. `sim/l4/*`: F-L4-1 drivetrain decoupling; grade(t)/impulse route; scripted rider events; wheel
   pulse sensor model; `discharged_wh` and V3 telemetry in CSV. ~300–500 LOC.
4. `tools/run_assist_v3_matrix.py` (new runner, ~300–500 LOC) and `tools/assist_v3_metrics.py`
   (metric functions, ~300–500 LOC) + `tests/test_assist_v3_metrics.py` proving each metric on
   synthetic known signals and that each can reject (pattern: `documentation/assist-pipeline-work/AP-01/test_metrics.py`,
   `tests/test_ripple_analyzer_rejects.py`). Reuse/port `AP-01/metrics.py` primitives
   (rise10/90, stop_points with confirmation, ripple) rather than re-deriving.

---

## 4. Metric availability

| Metric | Exists? | Where | What to add / where |
|---|---|---|---|
| true-release detection latency | **No** (analogue: SIL axis edge→detect for PAS stop) | `sim/evist_sil.c:1214-1266` | `t(first V3 class==TRUE_RELEASE) − t(gt release event)`, also in crank degrees; `assist_v3_metrics.py` |
| phase-dip false-positive rate | **No** | — | count of TRUE_RELEASE/PEDAL_STOP decisions per 100 revs in scenarios whose ground truth has no release (steady/dead spot/asym/noise/glitch) |
| Iq ripple | **Yes** | `tools/analyze_assist_ripple.py` (pipeline host CRUISE set, in verify_all); SIL steady Iq std/pp (`:1338-1356`); `run_regression.py` ripple column | add **per-revolution** ripple (port TEST A per-rev windowing, `torque_revolution_bench_host.c`) so it is comparable across cadences |
| time / angle to new lower demand | **No** | — | from gt release: first time `iq_final ≤ new_steady + tol` sustained (use AP-01 `stop_points` confirmation logic); report ms and crank deg |
| attack response | Partial | rise10/50/90 in `torque_run_asym_host.c` (torque only); AP-01 `metrics.py` `rise_time_10_90_detail` (Python, in `documentation/`, not `tools/`) | apply to `iq_final` after gt attack; move primitives into `tools/` |
| carry activation correctness / FP / duration / distance | **No** | L4 has `bike.distance_m` | gt obstacle/no-obstacle labels; carry flag + remaining time/distance from V3 telemetry; FP = activations in no-obstacle stops; duration/distance vs caps; cancel latency on brake/reverse/fault/acceleration |
| restart continuity | Partial | SIL stop_restart: restart permission, permission→Iq, max rise/fall step (`:1040-1080`) | generalise; add "no dip below X / no step > Y" after restart during or after carry |
| cadence invariance | **No** (only implicit in AP-01 cadence sweep) | — | aggregate each metric across the 10 cadences; report spread in ms **and** degrees; pass if angle-normalised spread within bound |
| energy use | Partial | L4 battery model has `discharged_wh` (`battery_pack.h:178`) but not in CSV; pipeline telemetry `motor_power_w` (`src/assist_pipeline.c:250`) | emit Wh per scenario (L4); `Σ motor_power_w·dt` proxy (SIL) |
| safety-zero timing | **Yes** for stop/reverse | SIL axis (edge→detect→permission→target→ref0→current0, peak drive) | add **brake** axis and "carry active at event" variants; assert carry never delays safety zero |

---

## 5. Baseline vs candidate on identical inputs

Production modules keep state in file-scope statics, so two variants cannot share one process
without symbol renaming. Recommended mechanism — **two builds, same harness, same script files**:

1. **Compile switch** `ASSIST_V3_ENABLE` (0/1) consumed only by the V3 module/its call site;
   harness emits V3 telemetry columns only `#if ASSIST_V3_ENABLE` (or writes a fixed `NA` so CSV
   schemas match).
2. **Pristine baseline build:** compile the *candidate tree's harness + rider_script* against
   **`src/`+`inc/` exported from 25df554** (`git archive 25df554 src inc | tar -x -C <scratch>`), so
   "baseline" is literally the frozen code, not a flag-off approximation. This works as long as the
   harness uses only APIs present at 25df554 (true for SIL/L4 today).
3. **Equivalence gate:** candidate built with `ASSIST_V3_ENABLE=0` must produce **byte-identical**
   CSVs to the pristine baseline for (a) all new matrix scripts, (b) existing SIL fixed scenarios,
   (c) `run_regression.py` traces, (d) L4 fixed CSVs. `run_regression.py` already does byte-compare
   determinism (`:111-116`) — same technique.
4. `tools/run_assist_v3_matrix.py` builds `{baseline, candidate}` × `{sil, l4}`, runs every script on
   both, computes metrics, writes a side-by-side table (`metric, baseline, candidate, delta, verdict`)
   and fails on missing/unusable evidence (follow `analyze_assist_ripple.py` "refuse" discipline).

---

## 6. Template bin count (12 / 24 / 32)

| Dimension | How the extended harness evaluates it | Status |
|---|---|---|
| Quality | generator knows exact crank angle → ground-truth torque per bin; compare learned template (exported per bin in telemetry or a debug dump at scenario end) vs truth: normalised RMS error, peak-bin alignment, revolutions-to-confidence | needs V3 telemetry + metric fn |
| Phase-dip robustness | FP rate (§4) per bin count across dead-spot/asym/20–130 rpm | needs gt labels |
| Noise robustness | sweep torque noise σ, PAS jitter/drop/bounce, cadence ripple, fg stalls, random initial phase (`phase_shift_deg` exists in `crank_torque_shape_t`) → template error + FP/latency vs bin count | needs script events |
| No-absolute-TDC proof | randomise `phase_shift_deg` and start angle per run; metrics must be statistically invariant | trivial once scripted |
| Memory | static: 32 bins × (int32 acc + u16 count/conf) ≈ 192–256 B vs 48 KB RAM; confirm with `tools/build_firmware.py` report `ram.data_bytes/bss_bytes` (`:294`) | target build |
| CPU | host sim **cannot** measure Cortex-M4 cycles. Options: (a) cross-compile V3 unit with arm-none-eabi-gcc at `-O0` *and* `-O2`, inspect size/disassembly for per-edge and per-tick paths; (b) HW measurement with DWT->CYCCNT around the call (pattern exists: `stop_trace_timing`, `src/main.c:4440`, `src/stop_trace.c:154`) | HW gate for final number |
| Budget hint | ≤ 1 edge per 4 kHz tick; 30 000 cycles/tick total and already overrunning; aim for V3 per-tick path ≪ 1 000 cycles at `-O0` and O(1) per-edge update (one bin) — no per-tick loop over all bins | `[INFERRED]` |

---

## 7. Effort estimate and candidates

| Item | Files | LOC (order) |
|---|---|---|
| rider_script generator + scenario files | `tests/host/common/rider_script.[ch]`, `tests/host/scenarios/assist_v3/*.csv|txt` | 300–400 + data |
| SIL matrix mode, shape/brake/glitch/stall/noise/IMU hooks, CSV | `sim/evist_sil.c` | 400–600 |
| L4: drivetrain decoupling (F-L4-1), route grade(t)/impulse, rider events, wheel-pulse sensor model, Wh + V3 columns | `sim/l4/bike_rider.[ch]`, `sim/l4/virtual_bike_l4.c` | 300–500 |
| Runner (baseline/candidate builds, matrix, compare) | `tools/run_assist_v3_matrix.py` (+ `run_sil.py`/`run_level4.py` module lists when V3 source is added) | 300–500 |
| Metric library + rejecting self-tests | `tools/assist_v3_metrics.py`, `tests/test_assist_v3_metrics.py` | 500–800 |
| **Total** | ~8–10 files | **~2.0–3.0 kLOC** |

Discovery candidates (captured, not chased — §22):
* `TECH-CANDIDATE`: extract `Speed_processing()` + decay logic from `main.c` into a linkable
  `wheel_speed.c` so harnesses link the production wheel sensor instead of mirroring it.
* `TECH-CANDIDATE`: retire or repair stale benches listed in §1.2 (they mislead tool inventory).
* `BUG/DECISION-CANDIDATE`: `current_iq` is the reference on target but measured Iq in SIL/L4 (§2.2).

## 8. Blockers / risks

1. **F-L4-1** must be fixed before any closed-loop carry/pedal-stop/reverse result from L4 is
   meaningful.
2. V3 input plumbing (PAS edge events, IMU struct, telemetry getter) does not exist yet; harness
   columns depend on it.
3. Wheel speed realism: without the 1-pulse/rev sensor model, carry cancel on "clear bike
   acceleration" will look far better in sim than on the bike.
4. CPU cost is not measurable on host; final acceptance needs a target build (and HW DWT
   measurement) — firmware is `-O0` by default.
5. 96 transitions/rev is inferred, not bench-measured; a 48/rev reality would halve bins per edge.
6. No sim can prove ride feel; matrix results are behavioural evidence within model scope
   (`TEST PASS != PROOF OUTSIDE TEST SCOPE`).

## 9. Commands executed (this session, scratch dir `…\scratchpad\simaudit`)

```text
gcc -std=c11 -O2 -Wall -Wextra -Werror -Wno-type-limits -Itests/host/common -Iinc \
  -o assist_pipeline_host.exe tests/host/pipeline/assist_pipeline_host.c tests/host/common/crank_model.c \
  <assist_mod list from tools/run_regression.py:82-85> -lm                       # 2.5 s
assist_pipeline_host.exe CRUISE_20_SPORT c20.csv                                  # 0.145 s (8 s sim, 32 000 rows)
assist_pipeline_host.exe RUN_120 r120.csv                                         # 0.105 s
gcc ... -O2 -Isim/l4 -Iinc -Itests/host/common/host_stubs -Itests/host/common -o evist_sil.exe \
  sim/evist_sil.c sim/l4/eb74_invocation_observer.c <mods from tools/run_sil.py:9-19> \
  -Wl,--wrap=g53_ad7ec_step -lm                                                   # 3.3 s
evist_sil.exe   (cwd with .build/sil pre-created; cmd `mkdir` path fails under Git Bash)  # 0.54 s, all PASS
gcc ... -O2 -Isim/full_host_stubs -Isim/l4 -Iinc -Itests/host/common -o virtual_bike_l4.exe \
  sim/l4/virtual_bike_l4.c sim/l4/eb74_invocation_observer.c sim/l4/battery_pack.c sim/l4/bike_rider.c \
  <PROD from tools/run_level4.py:17-27> -Wl,--wrap=g53_ad7ec_step -lm             # 4.3 s
virtual_bike_l4.exe                                                               # 0.67 s, 9/9 PASS + SOC endurance
gcc ... torque_revolution_bench_host.c crank_model.c signal_stats.c torque_input.c   # builds
gcc -c power_revolution_bench_host.c        -> error assist_mode_output_t (stale)
gcc -c sim/controller_lab/controller_lab.c  -> power_curve.h missing (stale)
```

Matrix cost estimate: 10 cadences × ~14 profiles × ~12 s ≈ 1 700 s simulated per variant ≈
15–30 s wall in SIL; ×2 variants ×5 noise seeds still < 5 min. `[INFERRED]` from measured rates.

## 10. NEXT EXACT ACTION

Master/Architect: confirm SIL-primary + L4-secondary split and the two-build baseline mechanism;
then open (a) a TASK for `rider_script` + SIL matrix mode with ground-truth labels, (b) a TASK for
L4 drivetrain decoupling (F-L4-1) + wheel-pulse sensor model, (c) a decision on the `current_iq`
reference-vs-measured input for the V3 carry score.
