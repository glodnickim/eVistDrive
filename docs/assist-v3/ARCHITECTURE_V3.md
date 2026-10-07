# Assist Behavior V3 — Architecture

```text
STATUS:    FROZEN CANDIDATE — awaiting REVIEW 1 (independent, before active implementation)
BASELINE:  25df554 (tag baseline/rideable-25df554)
AUTHOR:    Lead/Master (Claude Code, claude-opus-5-5), 2026-10-07
INPUTS:    audit/A_CONTROL_PATH.md, audit/B_TEST_AUDIT.md, audit/C_SIMULATION.md, audit/D_CONFIG_CAN.md
UNITS:     P = phase_current_max (700 on M820). "Iq" = firmware Iq units (P = full phase current).
           control tick = 4 kHz foreground period; logical tick = 1 ms. step = one PAS quadrature
           transition = 3.75 deg of crank (96 per revolution).
```

Numbers marked *candidate* are starting points for the simulation matrix (SIMULATION_REPORT.md). None of them is a
production value until the matrix and a ride select it.

## 1. Problem statement (from the audits)

At baseline the whole PEDAL demand is the transcribed G53 chain (EB74 -> D7EC -> BDE8, × g1). The temporal behaviour
is shaped in six stacked places (audit A §5). The rider-visible defects come from one of them:

- **The D7EC envelope is the "phase filter" and the release limiter at the same time.** It holds the stroke peak and
  decays with τ = (8·cadence + 1)·10 ms: 4.8 s at 60 rpm, 7.2 s at 90 rpm. Dropping the load to zero while pedalling
  takes 4.9–7.6 s to reach zero assist (probe). Assist even *rises* after a release if cadence rises (C2 ∝ env·cadence).
- It is time-domain. The same hold means a different fraction of a revolution at 25 rpm and at 120 rpm.
- Nothing separates a pedal-phase dip from a real release, or a release from a pedal stop.
- There is no carry concept. "Crank stopped, load held" decays at a fixed 0.455 Iq/ms; "crank stopped, load released"
  falls at the BDE8 rate. Neither knows whether the bike is climbing or rolling away.

V3 replaces exactly that layer: rider interpretation and demand trajectory. FOC, safety, limiters, CAN transport,
G53 static assist characteristic, battery limiter and the simulators stay.

## 2. Layer map and ownership

```text
 SENSORS (existing owners, unchanged)
   torque_input: load_ctrl (CLU, unfiltered) + valid       pas_sampler/pas_direction: step events, direction
   wheel: speed_x100 (+ pulse tick, NEW)                    motor: erps, Iq measured (NEW plumb), live Iq ref
   battery V/I, g1 (G53 PI #1)                               motion/IMU: motion_input_t (NEW seam, valid=false today)
        |
        v
 [V3-1] CRANK PHASE TRACKER        assist_v3_intent.c   relative phase mod 96 from signed step count; per-step torque
 [V3-2] RIDER INTENT               assist_v3_intent.c   learned phase template + expected-effort-normalised windows
 [V3-3] RELEASE CLASSIFIER         assist_v3_intent.c   NORMAL / PHASE_DIP / ATTACK / TRUE_RELEASE / PEDAL_STOP
        |
        v
 [V3-4] BASE ASSIST                g53 static map       the G5300 characteristic, as a pure function (legacy ratio/AUTO/LUT)
        |                          (+ Milestone E: torque/power blend; Milestone F: dynamic assist range)
        v
 [V3-5] TRANSIENT MANAGER          assist_v3.c          start, attack, release, carry (Milestone D) -> target
        |
        v
 [V3-6] THE TRAJECTORY             assist_v3.c          ONE rate-limited state y: the only temporal shaper of demand
        |                                               output: iq_demand >= 0, pre-limits
        v
 PIPELINE (assist_pipeline.c, existing owner, extended)
   × g1 (battery envelope, as BDE8 does)  ->  V3 stop/reverse backstop (NEW, pipeline-owned)
   -> ap2_limits (power/phase/UV/thermal/speed)  ->  iq_ceiling  ->  zero/slew policy
        |
        v
 fast_iq_slew (16 kHz)  — electrical anti-step guard only          ->  FOC
```

### 2.1 Single trajectory owner (hard requirement)

`assist_v3.c` holds the one demand state `y`. In V3 mode:

| Existing shaper (audit A id) | V3 mode |
|---|---|
| G4 D7EC envelope, G5 history, G6 readiness/C2 retention | **not consumed** — replaced by V3-2/V3-3 |
| G7 ratio rise limiter (+10/10 ms) | V3 keeps its own ratio state inside the static map call (same rule) |
| G9 D7EC accel trajectory (rise D28, fall D3E), G10 start hold-off | **not consumed** — rise rate *value* reused by V3-6 |
| G12 BDE8 S5 ±50/ms, state 7->1 decay | **not consumed** — its rate is a cap inside V3-6 rise |
| M2 pipeline R1 bumpless veto release | **bypassed** in V3 mode (V3 re-engages from the published value) |
| M3 `stock_hard_zero` from shadow BDE8 | **re-derived** by the pipeline: V3 demand == 0 and speed_native <= 0 |
| M1 iq_ceiling slew | kept — protection envelope, not a demand shaper |
| F1 fast_iq_slew 6.84 Iq/ms | kept — V3 rates are below it by construction (test asserts it never binds in normal riding) |
| g1 multiply | kept — envelope |

The transcribed G53 chain keeps running every tick in both modes (shadow). It still supplies g1, PAS cadence, the
EB74 zero, AUTO configuration and telemetry, and it is the warm fallback. No line of the transcription is edited;
V3 adds read-only accessors and one pure static-map function (§5).

### 2.2 Engine selection

- `engine ∈ {G5300, V3}`, from the V3 config block (CONFIG_PROTOCOL_V3.md). Firmware default for the V3 candidate
  build: **V3**. G5300 stays selectable without reflashing.
- A requested switch is latched only while the published request is 0 (or at standstill). The pipeline reset on an
  owner change resets V3 too.
- G5300 mode is bit-identical to baseline: gate `ASSIST_V3` compiled out *and* `engine = G5300` must both reproduce
  baseline CSVs byte-for-byte (TEST_MATRIX G-EQ).

## 3. Data contracts

### 3.1 V3 input (new struct, filled by the pipeline from existing + 4 new signals)

```c
typedef struct {
    uint32_t elapsed_ticks;            /* 4 kHz periods since last call (>= 1)            */
    uint16_t load_ctrl;                /* CLU, unfiltered (torque_input)                  */
    bool     torque_valid;
    int32_t  crank_steps;              /* NEW: signed running quadrature step count       */
    uint32_t crank_step_tick;          /* NEW: 4 kHz timestamp of the last step           */
    bool     pas_glitch;               /* NEW: illegal transition seen since last call    */
    int16_t  cadence_rpm;              /* G53 PAS signed cadence (shadow trace)           */
    bool     direction_inhibit, inhibit_is_reverse, real_stop;   /* native observations   */
    uint32_t speed_x100;  bool wheel_valid;
    uint32_t wheel_pulse_tick;         /* NEW: tick of the last wheel pulse               */
    uint16_t motor_erps;
    int32_t  iq_measured;              /* NEW: MS.i_q (A-audit open point: measured, not ref) */
    int32_t  last_published_iq;        /* pipeline ctx.last_final_iq                       */
    int32_t  phase_current_max;
    uint16_t g1_q12;                   /* observation; the pipeline applies it             */
    uint8_t  level;                    /* HMI 0..5                                          */
    bool     brake;                    /* observation only — the cut is native_cut          */
    motion_input_t motion;             /* sanitised copy (§3.3)                             */
    const g53_static_ctx_t *g53;       /* read-only accessors into the shadow chain (§5)    */
} assist_v3_input_t;

int32_t assist_v3_update(const assist_v3_input_t *in);   /* the single output: iq_demand >= 0 */
void    assist_v3_reset(void);
const assist_v3_telemetry_t *assist_v3_telemetry(void);  /* never a control input */
```

V3 never writes slew mode, ceiling, zero policy or any safety state.

### 3.2 Crank step events

The native PAS event drain (main.c:2772) already consumes timestamped `pas_step_event_t` from the 4 kHz sampler ISR.
It gains a signed accumulator and last-step tick, carried through `rider_input_t` -> `ride_control_input_t` ->
`assist_pipeline_input_t`. This avoids audit finding A-D7 (the G53 PAS replays a held `pas_ab` through foreground
catch-up and loses edges): V3 sees every step the ISR saw.

### 3.3 Optional motion / IMU seam

```c
typedef struct {
    bool     valid;
    uint16_t age_ms;            /* time since the sample was taken                 */
    int16_t  pitch_cdeg;        /* +nose up, 0.01 deg                              */
    int16_t  roll_cdeg;
    int16_t  pitch_rate_cdps;   /* 0.01 deg/s                                      */
    int16_t  accel_long_mg;     /* longitudinal, bike frame, gravity removed, mg   */
    int16_t  accel_vert_mg;
} motion_input_t;

void assist_motion_sanitize(const motion_input_t *raw, motion_input_t *out);
```

- `assist_motion_sanitize` copies only when `valid && age_ms <= MOTION_MAX_AGE_MS`; otherwise it writes an all-zero
  struct with `valid = false`. V3 and the future terrain estimator read only the sanitised copy.
- Current hardware: main.c passes `{ .valid = false }`. No driver is implemented.
- Hard test (TEST_MATRIX G1-IMU): random bytes in every field with `valid = false` (and with `valid = true` but stale)
  produce a bit-identical V3 output and telemetry over the whole scenario matrix.

## 4. Rider Intent V1 (Milestone B shadow, Milestone C active)

### 4.1 Crank phase tracker

- `phase = crank_steps mod 96` — relative, no TDC needed. Reverse steps move it backwards, so the phase survives
  back-pedalling and stops. It is lost only when steps are lost; a PAS glitch drops template confidence.
- Per step: `obs = mean(load_ctrl)` over the control ticks since the previous step (if one call carries n steps, the
  mean is attributed to all n). One O(1) update per step; at most ~208 steps/s at 130 rpm.
- A 96-entry ring of per-step `obs` (u16, 192 B) holds exactly one revolution.

### 4.2 Phase template

- `NB` bins per revolution, `bin = phase·NB/96`. *Candidates* NB = 12 / 24 / 32 (all divide 96); chosen by the matrix
  on quality, noise robustness and memory (SIMULATION_REPORT §template).
- `s[bin]` (Q12) = learned expected effort at that bin divided by the revolution mean (mean of s = 1.0).
  It captures dead spots and left/right asymmetry in one 360° profile.
- Learning (only in NORMAL_PRESSURE, intent above a floor, cadence in the trusted band, no glitch):
  `s[b] += α·(obs/I − s[b])`, renormalised to mean 1 once per revolution. *Candidate* α = 1/8 per visit.
- Confidence `c` (0..1): rises per clean revolution (low residual), decays slowly while stopped, drops to 0 on a PAS
  glitch, reverse, torque invalid or power-on. Below `c_min`, or outside the trusted cadence band, the template is
  replaced by uniform `s ≡ 1` (fallback).

### 4.3 Intent estimator — expected-effort-normalised windows

For any window W of recent steps:

```text
E_W = Σ_W obs / Σ_W s[bin]          (phase-compensated effort estimate)
```

- With W = one full revolution, `Σ s = 96` and `E_W` is the plain revolution mean — exact phase cancellation even with
  a poor template. This is the robust **long estimate** `E_long`.
- With a short window the template weights each step by how much effort that crank angle normally carries. Dead-spot
  steps have small `s`, so they barely move the estimate. That is what lets a short window see a real change without
  being fooled by a dip. The short window is the smallest W >= `A_min` (*candidate* 30°, 8 steps) whose `Σ s` reaches
  `S_min` (*candidate* the expected effort of 8 average steps), capped at 180° (48 steps). If the last steps lie in a
  dead spot, the window grows until it contains real expected effort.
- Normal intent `I = E_long` while the short estimate agrees with it (`ρ = E_short / I` inside a band).
  With high template confidence the long window may shrink to 180° (less lag); with low confidence it stays at 360°.

Every threshold is in crank angle or in expected effort, not milliseconds. The same rule decides in 30–45° at 25 rpm
and at 130 rpm. Time enters only for PEDAL_STOP and for fallback at very low cadence.

### 4.4 Release classification

| Class | Detection (per step, angle domain) | Intent | Trajectory consequence |
|---|---|---|---|
| NORMAL_PRESSURE | ρ in band | `E_long` | follow at normal rates |
| PHASE_DIP | raw `obs` < 0.5·I, but ρ in band (dip is expected at this phase) | unchanged | **no dip in demand** |
| ATTACK | ρ > `R_att` (*candidate* 1.4) over the short window | `E_short` (override) | rise at the level's legacy accel rate |
| TRUE_RELEASE | ρ < `R_rel` (*candidate* 0.5) over the short window | `E_short` (override, can reach 0) | fall at the Response release rate |
| PEDAL_STOP | no step for `T_stop` = clamp(k·expected step period, T_min, T_max) (*candidate* k = 4 steps = 15°, 60–400 ms), or native real_stop | held (last E) | handed to the stop/carry logic (§6) |

- An override lasts until `E_long` (refilled with post-change steps) agrees with `E_short` again, so a real change is
  followed immediately and a full revolution later the robust estimate takes over.
- Restart after a stop uses only post-restart steps (no stale pre-stop revolution).
- Gradual release stays inside the band and is followed by `E_long` (lag ≈ half the window).

## 5. Base assist — the G5300 characteristic as a pure function

Milestone C keeps the legacy assist ratio, AUTO and cadence normalisation. They are the part of D7EC after the
envelope (g53_port_chain.c:585-727): `c2 = env·cad·35/10000` (or `env·700/10000` at cad <= 20) -> `c4` -> ratio
(fixed per level, or AUTO interpolation on S+) with the +10/10 ms ratio limiter -> `d4` -> `rider_lut(cad)` -> `conv`
-> floor/clamp -> target (E2 domain, 0..40960).

- New exported pure function in the port, e.g. `g53_static_target(env_equiv, cad, level, &ratio_state)`, reading the
  same configuration fields the transcription reads (level ratios, AUTO enable/scale/step, LUT, floor). It writes
  nothing in the chain image. Parity test: for steady inputs it returns exactly the target the transcription computes
  from the same `env` (G1-STATIC).
- V3's intent is in CLU. It is converted to the EB74 `cur` domain with the EB74 affine transfer and live zero
  (`env_equiv = max(0, 750 + I·2450/6000 − zero)`), so the characteristic sees the same units as in G5300 mode.
- Target in Iq: `target_iq = target_E2 · 0.65·P / 40960` (E2 -> EE Q12 -> BDE8 Q5A cap 6500 -> m2aa -> Iq). Max is
  0.65·P, as at baseline.
- Engage threshold: V3 uses the EB74 engage/release thresholds (995 / 820 counts with the zero at 750, audit A-D8) as
  its intent deadband, so start sensitivity does not change in Milestone C.

Milestone E adds the torque <-> rider-power blend and Max Torque / Max Power envelopes here (§8). Milestone F adds the
dynamic assist range. Both change only this function's inputs/outputs, not the trajectory owner.

## 6. Transient manager and the trajectory

### 6.1 The trajectory

One state `y` (Iq, Q8). Each call: `y` moves toward `target` limited by:

| Direction / case | Rate | Source |
|---|---|---|
| rise (normal, ATTACK, start) | min(level legacy accel rate from D7EC `rise` (D+232), BDE8 50 m2aa/ms) | legacy attack kept |
| fall, NORMAL / gradual | release rate R(Response) | user Response |
| fall, TRUE_RELEASE | release rate R(Response) — target itself has dropped | user Response |
| PEDAL_STOP, Milestone C | load released: R(Response); load held: legacy 0.455 Iq/ms | baseline-equivalent stop |
| PEDAL_STOP, Milestone D | carry state machine (§7) | user Carry |

*Candidate* R(Response): full scale (0.65·P) in 600 ms at Response 0 % down to 150 ms at 100 %; all rates stay below
the 6.84 Iq/ms electrical guard.

### 6.2 Start

From rest: rise at the level's legacy accel rate once intent exceeds the engage threshold and forward PAS evidence
meets the existing readiness (`evid >= d26` or `env >= d24`, read from the chain configuration). The G10 hold-off
(EE forced 0 for up to 320 ms, semantics of its trigger G04 [UNKNOWN]) is not reproduced. Start Response
(CONFIG reserved slot) gets its consumer only after the matrix shows a need.

## 7. Obstacle carry / intelligent overrun (Milestone D)

The goal is not to detect an obstacle. It is to detect "the rider clearly wanted to keep driving, the bike is
loaded, and the rider briefly stopped or unloaded the pedals".

### 7.1 Carry score (bounded 0..1, computed continuously, frozen at the PEDAL_STOP transition)

Inputs: recent intent (last 1–2 revolutions), recent peak effort, attack (dI/dangle), cadence before the stop, wheel
speed, relative acceleration, motor load (Iq measured vs P), recent assist. Relative acceleration uses
`Δerps/erps` (gear-independent while the motor turns, fine-grained) with the wheel pulse speed as the absolute
reference. High score: low/medium speed, high intent, high motor load, acceleration <= ~0, sudden stop after a strong
stroke.

### 7.2 State machine

```text
IDLE --(PEDAL_STOP, score >= on-threshold, level carry_strength > 0, no cancel)--> CARRY
CARRY: demand = carry_level · decay(t), carry_level = f(score, strength) · min(previous assist, intent-based cap)
CARRY --(steps resume forward)--> NORMAL (trajectory continues from y — restart continuity)
CARRY --(time cap OR distance cap reached, first wins)--> RELEASE (fall at R(Response)) --> IDLE
CARRY --(cancel)--> RELEASE
cancel = brake | reverse step | fault/native_cut | assist off | bike clearly accelerating (relative accel > threshold)
```

- Time cap and distance cap come from the user "extent" parameter, each bounded by a firmware hard maximum.
- Slow release is never used as a substitute for carry.

### 7.3 Pipeline backstop (V3 mode only; outside V3)

At baseline, stop and reverse are G53 behaviour (OWNER-DEC-2026-10-06-G5300-ONLY). In V3 mode the G53 stop logic is
not consumed, so the pipeline gets its own bound, independent of V3 code (DECISIONS D-008):

- reverse step / direction inhibit: allowed demand decays from the published value at >= the BDE8 rate
  (3.5 Iq/ms, full scale in 130 ms) — same as G5300 reverse.
- crank stopped (native real_stop or G53 cadence 0): allowed demand holds for at most `T_STOP_HARD` (*candidate*
  1500 ms) or `D_STOP_HARD` (*candidate* 2.0 m), first wins, then decays to 0 within 300 ms.
- standstill: FORCE_ZERO as at baseline.
- The backstop is a ceiling (`min`), never a demand. Brake/fault keep the native SAFETY 200 ms path upstream.

## 8. Envelopes and dynamic range (Milestone E/F — contracts only now)

- **Max Torque** (E): ceiling on motor torque demand, per level. Expressed as % of the motor's rated torque until a
  physical Nm calibration exists (torque constant is [UNKNOWN] on M820).
- **Max Power**: stays the existing P1 power % per level via g1 (one owner). No second power owner.
- **Torque <-> rider-power blend** (E): for > 110 rpm and < 30 rpm, blend the characteristic input between rider
  torque and rider power. Not mixed with the Milestone C release work.
- **Terrain/Load State** (F): `terrain_state_t { load_class, slope_est, confidence, source }`, source = IMU when the
  sanitised motion input is valid, otherwise wheel speed + dv/dt + motor load + effort + cadence. Runs in shadow first.
- **Assistance Range** (F): dynamic min/max ratio window driven by terrain state (AUTO).

## 9. Safety contract (unchanged ownership)

The behaviour layer can request more or less NORMAL assist inside its range. It cannot:

- bypass `native_cut` (brake, overtemp >= 2, torque fault/calibration, PAS not seeded) -> SAFETY 200 ms;
- bypass owner arbitration (comms loss, battery trip, calibration, Walk) -> FORCE_ZERO / own owner;
- bypass ap2_limits, iq_ceiling, g1, the fast slew ceiling, FOC limits, the standstill hard zero, or the V3 backstop;
- write any zero policy or slew mode.

Each item has a test in TEST_MATRIX (G1-SAFE).

## 10. Mode character

Levels keep their HMI mapping (1 ECO, 2 TOUR/TRAIL, 3 SPORT, 4 SPORT+ with AUTO ratio, 5 BOOST). A level is a
strategy, expressed as firmware default sets for the V3 parameters (CONFIG_PROTOCOL_V3 §defaults):

| Character | Response (release) | Carry strength / extent | Notes |
|---|---|---|---|
| ECO | gentle | small / short | energy first |
| TRAIL | medium | large / medium | technical climbing, natural effort |
| SPORT | fast | medium / medium | quick attack (legacy accel) and fast TRUE_RELEASE |
| SPORT+ | fastest | large / long | strong attack primes carry via the score |
| BOOST | fast | medium / medium | as SPORT, higher legacy ratio |
| AUTO | — | — | Milestone F dynamic range; today S+ keeps the G5300 AUTO ratio |

Values are chosen from the simulation matrix, not by feel.

## 11. Telemetry (DIAG build only)

One extra DIAG frame group, only the fields needed to answer "why this Iq now": `intent`, `E_short`, `phase`,
`template_conf`, `expected_effort`, `release_class`, `carry_score`, `carry_state`, `carry_remaining_ms`,
`carry_remaining_cm`, `base_target_iq`, `v3_demand_iq`, `backstop_iq`, `final_iq` (existing), `cadence`, `rel_accel`,
`terrain_est` (F), `imu_valid`, `engine`. NORMAL build: unchanged frames.

## 12. Resources

- RAM: ring 192 B + template ≤ 64 B + state ≈ 100 B ≈ 0.4 KB of 48 KB.
- CPU: O(1) per step plus ≤ 48-step window walk on each step (≈ 10 k simple ops/s at 130 rpm), O(1) per control tick.
  Integer only. The firmware builds at -O0 except G53 sources at -O2; `assist_v3*.c` joins the -O2 list. Target cost
  is measured with the DWT cycle counter on the DIAG build before the ride (not provable on host).

## 13. Milestones

| Milestone | Content | Ride behaviour |
|---|---|---|
| A Foundation | baseline freeze, audits, this architecture, test matrix, config contract draft, IMU seam, sim inputs | unchanged |
| B Shadow | V3-1..V3-3 + V3-4/6 computed beside G53, telemetry, matrix A/B on identical inputs | unchanged |
| C Active release | engine V3: intent, classifier, static map, trajectory, backstop; legacy ratio/attack/power/battery/safety | release changed |
| D Carry | carry score + state machine, L4 freewheel drivetrain | carry added |
| E Envelopes | torque/power blend, Max Torque, wide cadence | characteristic changed |
| F Terrain/Auto | terrain state shadow -> dynamic range | AUTO changed |

## 14. Open items carried into REVIEW 1

- D-008 backstop bounds (`T_STOP_HARD`, `D_STOP_HARD`) and the EN 15194 run-on reference are [EXTERNAL_REFERENCE,
  not verified in this repo]; owner confirmation before the ride.
- BDE8 G04 / D25 semantics [UNKNOWN] — only matters if the matrix shows a start-behaviour regression.
- On-bike size of the release hold [UNKNOWN] until a V3 DIAG ride; the probe used a load step.
- CONFIG_A page vs bootloader update erase [UNKNOWN]; a wiped record falls back to defaults safely.
