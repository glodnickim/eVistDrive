# Assist Behavior V3 — Test Matrix and Gates

Source of classification: TEST_AUDIT.md (summary) and audit/B_TEST_AUDIT.md (full table). Run everything from the
repo root with `CC=gcc` (w64devkit on PATH). `$TC` = Arm GNU 13.2 Rel1 bin directory.

## Gate levels — when each is required

| Gate | When | Budget | Commands |
|---|---|---|---|
| **G0 edit loop** | after each small change | 10–30 s | `gcc ... -fsyntax-only src/<module>.c`; `python tools/run_host_tests.py --group v3 --opt O2 -j 4`; affected group (`g53`/`config`); `python tools/run_sil.py --fuzz 100` when pipeline/PAS plumbing changed |
| **G1 feature gate** | a logical piece is complete (intent, classifier, trajectory, carry, config) | ~2–3 min | `run_host_tests.py --group v3 --group g53 --group safety --group config --opt O2 -j 4`; `tests/test_m820_walk_can_safety.py`; `tools/run_regression.py`; `tools/analyze_assist_ripple.py`; `tools/run_replay_regression.py`; `run_sil.py --fuzz 300`; `run_level4.py --quick`; **V3 matrix** `tools/run_assist_v3_matrix.py --quick` |
| **G2 milestone integration** | end of milestone B, C, D, E, F | ~5 min | `python tools/verify_all.py --quick --target` + V3 matrix full + A/B report |
| **G3 release candidate** | before RELEASE_REPORT / PASS | ~10 min | `verify_all.py --require-target`; DIAG target `build_firmware.py --variant diagnostic --mode developer --toolchain $TC`; config round trip suites; `tests/test_canable_ride_decode.py`; `tests/tools_prepare_m820_bl820.py`; V3 matrix full + A/B; sanitizers only where a capable compiler exists (not on this PC — recorded as NOT_RUN) |

Fail rule: a G2/G3 failure is reproduced on the failing component, fixed, and passed at G0/G1 for that component.
The full gate is re-run once at the end, not after every attempt.

## Required V3 test groups (new; `--group v3`)

| ID | What it proves | Gate |
|---|---|---|
| G-EQ | candidate with `ASSIST_V3` off, and with engine = G5300, reproduces baseline CSVs byte-for-byte (SIL, regression, replay) | G1 |
| G1-LEVEL | V3 steady-state mean Iq vs baseline: ±5 % with converged template, ±15 % and no step > 10 % with the prior; L1-L5 + S+ AUTO, 20..130 rpm, dead-spot depth 0.1/0.3/0.6, three load levels; plus light spin then attack within one revolution with no overshoot above the baseline characteristic | G1 |
| G1-TRACK | pedal-pressure tracking: mean-effort ramps up/down at 20/60/120 rpm; demand monotonic, tracking error vs baseline characteristic, no dip at dead spots | G1 |
| G1-FALLBACK | first 3 revolutions after start, after reverse, after a glitch, at 20 rpm with deep dead spots: zero TRUE_RELEASE classifications | G1 |
| G1-STYLE | sit -> stand at equal mean; asymmetric legs at 20 and 130 rpm: no assist dip, fallback within one revolution of mismatch, template re-converges | G1 |
| G1-VETO | brake release while pedalling, assist off -> on, both engines: climb back at the R1 rate | G1 |
| G1-STAND | start from rest with speed 0 for the first 4.4 m gets assist; at speed 0: reverse and crank-stop-load-released zero time <= baseline; crank-stop-load-held follows the legacy ramp within ±20 % of baseline (~1.0 s from 455 Iq) — two-sided, no early cut | G1 |
| G1-STOP | legacy stop timing: V3 stop/reverse zero time <= baseline for load held/released at 25/60/120 rpm | G1 |
| G1-STATIC-T | level change while riding and S+ AUTO attack, in time, against the transcription | G1 |
| G1-PAS | sampler ring overflow (>32 events in a stall) and INVALID jumps: phase unaligned, no false class, re-alignment | G1 |
| G1-CLIMB | gear-shift unload (0.2-0.5 s partial release) and ratcheting (quarter strokes back/forward) on a technical climb | G1 |
| G1-LONG | >= 1 h simulated: template renormalisation drift, counter/tick wrap | G2 |
| G1-OSC | class transitions in steady riding = 0 | G1 |
| G1-START | pedalling unloaded at speed, then loading: both engines, difference explained (REVIEW 1 discovery) | G1 |
| G1-IMU | garbage in every motion field with valid=false (and valid but stale) -> bit-identical V3 output + telemetry over the matrix | G1 |
| G1-STATIC | `g53_static_target()` equals the transcription's target for the same env/cadence/level/AUTO (sweep) | G1 |
| G1-PHASE | phase tracker: mod-96 continuity through stop, back-pedal, restart; glitch drops confidence | G0/G1 |
| G1-TPL | template learning converges on steady, dead-spot, asymmetric profiles at 20..130 rpm; renormalises; fallback when confidence/cadence invalid | G1 |
| G1-CLS | classifier: PHASE_DIP never lowers intent; TRUE_RELEASE detected within the angle budget at every cadence; ATTACK; PEDAL_STOP | G1 |
| G1-TRAJ | one trajectory: rates per table; never exceeds the 6.84 Iq/ms guard in normal riding; restart continuity | G1 |
| G1-SAFE | V3 cannot bypass native_cut, owner arbitration, limits, g1, ceiling, standstill zero, backstop; no rider load -> zero Iq (ported property) | G1 |
| G1-BACKSTOP | V3 forced to output max (also at speed 0): reverse decay rate, stop hold bound, decay to 0, re-open only after forward steps at <= 3.5 Iq/ms (reverse -> forward, stop past T_STOP_HARD -> restart), standstill FORCE_ZERO | G1 |
| G1-CARRY | activation on climb stop, no activation on coast/crest/low score, caps (first wins), cancels (brake, reverse, fault, acceleration), restart handover | G1 (Milestone D) |
| G1-CFG | CONFIG_PROTOCOL_V3 vectors, every reject reason with no-mutation assertion, reserved-param rejection, generation (skips 0xFFFF, wrap), foreign transfer live, transfer timeout, engine write while riding latched not immediate, append-only persist with power loss between erase and program, restart/corrupt/newer record, legacy streams byte-identical | G1 |
| G1-SEL | switch latches only with published and both engine demands 0 and no veto; during release, during brake, at standstill with demand: no step; R1 governs the climb; pipeline reset resets V3 | G1 |

## Mode character groups (owner override 2026-10-07; G2 = run at each milestone that activates a consumer)

| ID | What it proves | Gate |
|---|---|---|
| G2-AUTONOMOUS | every mode with NO user override passes the whole V3 behaviour matrix (cadence x profile) and the acceptance thresholds of SIMULATION_REPORT §2 | G2 |
| G2-OVERRIDE | one parameter changed at a time (Assist, Max Torque, Max Power, Response, Start, Carry low/high, per mode): the effect matches the parameter's documented meaning | G2 |
| G2-ORTHO | changing one parameter leaves unrelated metrics within tolerance: Max Power -> no change in classification, TRUE_RELEASE latency, start threshold; Carry -> no change in normal RUN; Response -> no change in power/torque ceilings; Assist -> no change in safety/battery/thermal/legal limits. Couplings found are documented or the API changes | G2 |
| G2-MACRO | Assist 20/40/60/80/100 x 25/60/90/120 rpm x every mode: intent, desired support, torque equivalent, motor power, response time, release; demand monotone in Assist (equality only where an envelope binds); mode keeps its character across the range | G2 |
| G1-CFG2 | config v2: mode profile + global objects, configured vs effective vs source, restore one mode / all, legacy P0/P1 input precedence, reserved params, persist/restart, erase window | G1 |

## G-EQ rules (REVIEW 1 #20)

1. Harnesses set `engine = G5300` explicitly (the candidate default is V3; the flash stub has no record).
2. V3 telemetry goes to a separate CSV (or separate columns excluded from the comparison).
3. No V3 DIAG frame is emitted in G5300 mode in any simulated CAN trace.

## V3 scenario matrix (tools/run_assist_v3_matrix.py)

Cadences 20, 25, 30, 40, 60, 80, 100, 110, 120, 130 rpm × profiles: steady, strong dead spot, L/R asymmetry, gradual
release, sudden TRUE_RELEASE, strong attack, attack→PEDAL_STOP, attack→PEDAL_STOP→restart, technical climb, crest,
coast, reverse, brake, PAS glitches; plus foreground stalls and torque noise. Host: SIL (open loop, production PAS
front end); Level-4 for climb/crest/carry/energy once the freewheel drivetrain lands.

Metrics, baseline vs candidate, with a reject self-test per metric: true-release latency (ms and crank deg),
phase-dip false-positive rate, per-revolution Iq ripple, time/angle to the new lower demand, attack response, carry
activation correctness / false positives / duration / distance, restart continuity, cadence invariance, energy (Wh),
safety-zero timing.

## Existing suites V3 will intentionally change

Only if V3 edits them (D-003 says it does not): H62, H67, H68. Expected touches: H71/H72 (levels/AUTO consumption),
H61/H63 partially, H75 T3/T6/T7, replay f07/f08 pause cases (V3 mode only). SIL/L4 `--wrap=g53_ad7ec_step` and the
EB74 prehistory requirement stay valid because the chain still runs. Any change to an existing expectation is listed in
RELEASE_REPORT with the reason.
