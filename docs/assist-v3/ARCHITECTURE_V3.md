# Assist Behavior V3 — Architecture

```text
STATUS:    FROZEN rev 3 — REVIEW 1 closed PASS_WITH_ISSUES (2026-10-07T13:05:09+02:00); residuals R-a..R-d tracked
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
| G7 ratio rise limiter (+step per 10 ms D7EC call) | V3 keeps its own ratio state inside `g53_static_target()`, advanced per **elapsed 10 ms** (accumulator), never per call. Accepted legacy-characteristic shaper in C/D (it is what makes S+ AUTO attack and level changes legacy); moved into V3-6 in Milestone E so single ownership becomes literal (R1-#8) |
| G9 D7EC accel trajectory (rise D28, fall D3E), G10 start hold-off | **not consumed** — rise rate *value* and the D3E rate are reused by V3-6 |
| G12 BDE8 S5 ±50/ms, state 7->1 decay | **not consumed** — its rate is a cap inside V3-6 rise and the legacy stop rate |
| M2 pipeline R1 bumpless veto release | **kept in V3 mode** — it is a veto-release limiter, not a demand shaper; it binds only after native_cut, assist-off, the standstill zero, the backstop, or an engine switch (`pulled_down` computed against the post-limit V3 request) (R1-#3) |
| M3 `stock_hard_zero` from shadow BDE8 | **replaced** by one pipeline predicate: `standstill_zero = speed_native <= 0 ∧ (direction_inhibit ∨ (crank stopped ∧ V3 stop target == 0))` -> FORCE_ZERO, where *crank stopped* = G53 PAS true-stop ∨ native real_stop (R1-#7, re-check N2, closure R-b). The V3 term depends on V3 internals; if V3 misbehaves, the bound at standstill is the independent backstop (§7.3, <= 1.8 s), tested with V3 forced to max at speed 0 (closure R-a). It reproduces baseline at speed 0: reverse -> ~2 ms, crank stopped with load removed -> ~32 ms, crank stopped with load **held** -> the legacy D3E ramp (~1.0 s from 455 Iq), not a cut. The V3 term can only make zeroing earlier, never hold demand. A start from rest (speed 0 for the first ~4.4 m, crank turning) is not zeroed |
| M1 iq_ceiling slew | kept — protection envelope, not a demand shaper |
| F1 fast_iq_slew 6.84 Iq/ms | kept — V3 rates are below it by construction (test asserts it never binds in normal riding) |
| g1 multiply | kept — envelope |

The transcribed G53 chain keeps running every tick in both modes (shadow). It still supplies g1, PAS cadence, the
EB74 zero, AUTO configuration and telemetry, and it is the warm fallback. No line of the transcription is edited;
V3 adds read-only accessors and one pure static-map function (§5).

### 2.2 Engine selection

- `engine ∈ {G5300, V3}`, from the V3 config block (CONFIG_PROTOCOL_V3.md). Default when no record exists, in the V3
  candidate build: **V3** (the BIN exists to test V3; CANable has no V3 UI yet, so the fallback is reflashing the
  baseline BIN 0.638 — DECISIONS D-020). G5300 stays selectable over the protocol without reflashing.
- `engine_requested` is latched into `engine_active` only when the published request **and both engines' demands**
  (V3 `y`, shadow G53 `iq_request_pre_limits`) are 0 and no veto (native_cut, assist-off, backstop, standstill) is
  active. On the switch tick the pipeline sets `pulled_down = true`, so R1 governs any climb. No standstill clause
  (R1-#10). Both values are reported in CAPS/STATUS.
- The pipeline reset on an owner change resets V3 (template kept — it describes the rider, not the ride).
- G5300 mode is bit-identical to baseline: `ASSIST_V3` compiled out *and* `engine = G5300` must both reproduce
  baseline CSVs byte-for-byte under the G-EQ rules in TEST_MATRIX.
- V3 engagement is gated on the EB74 "armed" state (startup window done and the pedal seen unloaded once after a
  reset), read through an accessor, so restart semantics stay baseline (R1-#14).

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
A new production module `crank_phase.c` owns the signed step accumulator, the last-step tick and the glitch flag
(INVALID two-bit jumps **and** sampler ring overflow, `pas_sampler_take_overflow`). main.c and every harness (SIL,
L4) feed it the same events, so the matrix tests production code (R1-#19). Its outputs are carried through
`rider_input_t` -> `ride_control_input_t` -> `assist_pipeline_input_t`. This avoids audit finding A-D7 (the G53 PAS
replays a held `pas_ab` through foreground catch-up and loses edges): V3 sees every step the ISR saw. Counters are
wrap-safe (differences only).

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

- `NB` bins per revolution, `bin = phase·NB/96`. *Candidates* NB = 12 / 24 (32 only if a latency offset is learned:
  torque-sensor latency is constant in time, so it shifts the template by more steps at 130 rpm; R1-#15). Chosen by
  the matrix on quality, noise robustness and memory.
- `s[bin]` (Q12) = learned expected effort at that bin divided by the revolution mean (mean of s = 1.0). It captures
  dead spots and left/right asymmetry in one 360° profile.
- **Prior:** a fixed population-typical stroke table (peak/mean ≈ 1.4–1.6, chosen from the matrix and later from V3
  ride logs), never `s ≡ 1`. The prior is the template after power-on and the reference for alignment (R1-#1.3).
- **Learning gate = revolution stability, not the per-step class** (R1-#6): when a revolution completes and its mean
  is within ±15 % of the previous revolution's mean (intent unchanged, only the shape may differ), the revolution is
  learned retroactively from the 96-entry ring: `s[b] += α·(obs_b/I_rev − s[b])`, then renormalised to mean 1.
  *Candidate* α = 1/8 per revolution. A rider who changes style (sit → stand) adapts within a few revolutions
  without any class gate locking learning out.
- Confidence `c` (0..1): residual-based. It rises per learned revolution with low residual, falls with high residual,
  decays slowly while stopped. A revolution whose residual exceeds the mismatch threshold (style change, e.g.
  sit -> stand) drops the classifier to fallback mode within that one revolution (re-check N5). A glitch (INVALID jump or ring overflow) marks the phase **unaligned**: after the next
  full revolution the phase is re-aligned by circular cross-correlation of that revolution against the template
  (96 × NB operations once per revolution) and confidence is restored if the correlation peak is clear (R1-#15).
  Power-on, torque invalid and reverse-then-unknown keep the prior until confidence is rebuilt.
- Trusted band: template mode only for 15 rpm <= cadence <= 150 rpm *(candidate)*, `c >= c_min`, phase aligned.

### 4.3 Two quantities: physical intent `I` and envelope-equivalent `env_equiv`

These are kept separate on purpose (R1-#1):

- **`I`** = physical rider effort, CLU, never rescaled. Drives the classifier, the carry score, the Milestone E
  torque/power blend and the Milestone F terrain state.
- **`env_equiv`** = the value the G5300 D7EC envelope would hold for this rider's stroke at this cadence. Used
  **only** as the input of the G5300 static map (§5) in Milestones C/D, so steady-state assist equals baseline.

**Estimating `I` — expected-effort-normalised windows.** For a window W of recent steps,
`E_W = Σ_W obs / Σ_W s[bin]`. With W = one revolution `Σ s = 96` and `E_W` is the plain revolution mean: exact phase
cancellation even with a poor template (`E_long`). In template mode a short window (smallest W >= `A_min`,
*candidate* 30° / 8 steps, whose `Σ s` reaches `S_min`, capped at 180°) gives `E_short`: dead-spot steps have small
`s`, so they barely move it. Normal intent `I = E_long` (360°, or 180° when confidence is high).

**Computing `env_equiv`.** Once per revolution, and when cadence changes by more than 10 %, run the exact D7EC
recurrence (`k = 8·cad`, 10 ms step, integer `k|1` form, attack `env = x`) over the template-reconstructed stroke
`x(θ) = EB74_active(I_rev · s(θ))`, where `EB74_active(L) = max(0, 750 + L·2450/6000 − thr)` with the **active**
threshold `thr` chosen by **V3's own engaged flag** (820 while V3 demand is engaged, 995 to engage); only the EB74
zero (750 today) is read from the shadow chain through an accessor (R1-#2, re-check N4). The steady-state mean
`env_ss` is converted back to the **load domain** (re-check N1):

```text
L_eq = (env_ss + thr - 750) * 6000 / 2450      (the constant load EB74 maps to env_ss)
kL   = clamp(L_eq / I_rev, 1.0, 2.5);  I_rev < ~300 CLU -> kL of the prior template at this cadence
env_equiv = EB74_active(kL * I)
```

Defining the factor before the EB74 deadband keeps it bounded: light spinning followed by a hard push in the same
revolution cannot multiply the new load by a near-deadband ratio. Between recomputations `env_equiv` follows `I` at
once when the classifier moves it (release, attack) — no envelope state carries over; all dynamics stay in V3-6.
Cost: ≤ 300 recurrence steps per revolution at 20 rpm, ≤ 50 at 120 rpm.

Acceptance (TEST_MATRIX G1-LEVEL): V3 steady-state mean Iq within ±5 % of baseline for L1–L5 and S+ AUTO,
20..130 rpm, dead-spot depth 0.1/0.3/0.6, three load levels, with a converged template; with the prior template
within ±15 % and no step above 10 % while the template converges (re-check N3).

### 4.4 Release classification

Template mode (aligned, confident, trusted band):

| Class | Detection (per step, angle domain) | Intent | Trajectory consequence |
|---|---|---|---|
| NORMAL_PRESSURE | `ρ = E_short / I` in band | `E_long` | follow at normal rates |
| PHASE_DIP | raw `obs` < 0.5·I but ρ in band | unchanged | **no dip in demand** |
| ATTACK | enter ρ > 1.4, exit \|ρ − 1\| < 0.2 *(candidates)* | `E_short` (override) | rise at the level's legacy accel rate |
| TRUE_RELEASE | enter ρ < 0.5, exit \|ρ − 1\| < 0.2 *(candidates)* | `E_short` (override, can reach 0) | fall at R(Response) |
| PEDAL_STOP | no step for `T_stop` = clamp(4 expected step periods, 60, 400 ms) *(candidate)*, or G53 true-stop, or native real_stop | held (last E) | stop logic (§6.1 C, §7 D) |

Fallback mode (prior or low confidence, unaligned, or outside the trusted band) uses rules that need no template
(R1-#5); every 180° of crank contains a power stroke:

| Class | Fallback detection |
|---|---|
| TRUE_RELEASE | **maximum** per-step `obs` over the last 180° < `R_rel · I` |
| ATTACK | mean over the last 180° > `R_att · I` |
| PHASE_DIP / NORMAL | otherwise (a dip inside 180° can never trigger a release) |

- Detection budget ≤ 180° at any cadence in fallback, ≈ 30–45° in template mode.
- Hysteresis: separate enter/exit thresholds above, and a minimum dwell of 90° in ATTACK / TRUE_RELEASE before exit
  (R1-#16). Steady riding must show zero class transitions (oscillation metric).
- An override ends when `E_long` (refilled with post-change steps) agrees with `E_short` within the exit band.
- Restart after a stop uses only post-restart steps.
- Gradual release stays inside the band and is followed by `E_long` (lag ≈ half the window).

## 5. Base assist — the G5300 characteristic as a pure function

Milestone C keeps the legacy assist ratio, AUTO and cadence normalisation. They are the part of D7EC after the
envelope (g53_port_chain.c:585-727): `c2 = env·cad·35/10000` (or `env·700/10000` at cad <= 20) -> `c4` -> ratio
(fixed per level, or AUTO interpolation on S+) with the ratio rise limiter -> `d4` -> `rider_lut(cad)` -> `conv`
-> floor/clamp -> target (E2 domain, 0..40960).

- New exported pure function in the port, `g53_static_target(env_equiv, cad, level, &ratio_state, elapsed_10ms)`,
  reading the same configuration fields the transcription reads (level ratios, AUTO enable/scale/step, LUT, floor).
  It writes nothing in the chain image. The ratio limiter advances per elapsed 10 ms (§2.1).
- Parity tests: G1-STATIC — equal target for the same `env` in steady state; G1-STATIC-T — level change and S+ AUTO
  attack in time against the transcription.
- Input is `env_equiv` (§4.3), never `I`.
- Target in Iq: `target_iq = target_E2 · 0.65·P / 40960` (E2 -> EE Q12 -> BDE8 Q5A cap 6500 -> m2aa -> Iq). Max 0.65·P.
- Engage: `env_equiv > 0` requires the active EB74 threshold to be exceeded, so start sensitivity is unchanged, and
  engagement also requires EB74 armed (§2.2).

Fixed APIs for every later milestone (REVIEW-T #14), so no milestone re-plumbs the pipeline:

```c
typedef struct { uint16_t value[24]; uint8_t source[24]; } assist_v3_effective_t;   /* resolved, one level */
const assist_v3_effective_t *assist_v3_effective(uint8_t hmi_level);   /* cached, recomputed on generation/level */

typedef struct { int32_t base_x100, slope_q16, range_min_x100, range_max_x100; uint8_t c_floor_rpm;
                 int8_t hc_bias_pct; } g53_ratio_law_t;
/* g53_static_input_t gains `const g53_ratio_law_t *law;` - NULL = legacy slot ratio/AUTO law (bit-identical). */
```

The G7 ratio-limiter step and the rise rate are taken from the **mode** (its attack), not from the HMI slot, once the
level -> mode map is active (REVIEW-T #15).

Milestone E replaces (not stacks on) the `c2`/LUT cadence term with the torque <-> rider-power blend, and adds Max
Torque via the existing `level_iq_limit` path in ap2_limits (one ceiling owner, R1-#17). Milestone F adds the dynamic
assist range. Both change only this stage, not the trajectory owner.

## 6. Transient manager and the trajectory

### 6.1 The trajectory

One state `y` (Iq, Q8, pre-g1, pre-limits). Each call: `y` moves toward `target` limited by:

| Direction / case | Rate | Source |
|---|---|---|
| rise (normal, ATTACK, start) | min(level legacy accel rate from D7EC `rise` (D+232), BDE8 50 m2aa/ms) | legacy attack kept |
| fall while pedalling (NORMAL / gradual / TRUE_RELEASE) | R(Response) | user Response |
| PEDAL_STOP, Milestone C, load released | legacy **3.5 Iq/ms** (BDE8 −50/ms after the D7EC zero-reset) | legacy stop (R1-#9) |
| PEDAL_STOP, Milestone C, load held | legacy **0.455 Iq/ms** (D3E) after the stop is confirmed | legacy stop (R1-#9) |
| PEDAL_STOP, Milestone D | carry target profile (§7); V3-6 stays the only rate limiter | user Carry |

- *Candidate* R(Response): full scale (0.65·P) in 600 ms at Response 0 % down to 150 ms at 100 %. All rates stay
  below the 6.84 Iq/ms electrical guard.
- Response never slows a stop: in Milestone C every stop/reverse timing must be <= baseline (matrix metric).
- `y` is never clamped to the published value by limiters (speed taper, thermal, UV, g1 do not re-seed it), so legal
  speed behaviour stays as at baseline. Veto release is R1's job in the pipeline (§2.1).

### 6.2 Start

From rest: rise at the level's legacy accel rate once `env_equiv` exceeds the active EB74 engage threshold, EB74 is
armed (§2.2), and forward PAS evidence meets the existing readiness (`evid >= d26` or `env >= d24`, read from the
chain configuration). The G10 hold-off (EE forced 0 for up to 320 ms, trigger G04 [UNKNOWN]) is not reproduced; the
matrix compares start timing with baseline, including "pedalling unloaded at speed, then loading" (R1-#12
discovery). Start Response (reserved) gets a consumer only after the matrix shows a need.

## 7. Obstacle carry / intelligent overrun (Milestone D)

The goal is not to detect an obstacle. It is to detect "the rider clearly wanted to keep driving, the bike is
loaded, and the rider briefly stopped or unloaded the pedals".

### 7.1 Carry score (bounded 0..1, computed continuously, frozen at the PEDAL_STOP transition)

Inputs: recent `I` (last 1–2 revolutions), recent peak effort, attack (dI/dangle), cadence before the stop, motion
estimate (§7.4) speed and relative acceleration, motor load (Iq measured vs P), recent assist. High score:
low/medium speed, high intent, high motor load, acceleration <= ~0, sudden stop after a strong stroke.

### 7.2 State machine

```text
IDLE --(PEDAL_STOP, score >= on-threshold, level carry_strength > 0, no cancel)--> CARRY
CARRY: target = carry_level · profile(t); carry_level = f(score, strength) · min(previous assist, intent-based cap)
       profile(t) falls slower than R(Response) so V3-6 does not distort it
CARRY --(steps resume forward)--> NORMAL (trajectory continues from y — restart continuity)
CARRY --(time cap OR distance cap reached, first wins)--> RELEASE (fall at R(Response)) --> IDLE
CARRY --(cancel)--> RELEASE
cancel = brake | reverse step | fault/native_cut | assist off | bike clearly accelerating (relative accel > threshold)
```

- Time cap and distance cap come from the user "extent" parameter, each bounded by a firmware hard maximum, with
  `static_assert(CARRY_HARD_MAX_* <= BACKSTOP_*)`.
- Slow release is never used as a substitute for carry.
- In Milestone D the standstill predicate (§2.1) uses the motion estimate instead of `speed_native`, so carry is not
  killed at walking pace where the wheel sensor reads 0; this change is proven against baseline stop timing.

### 7.3 Pipeline backstop (V3 mode only; outside V3)

At baseline, stop and reverse are G53 behaviour (OWNER-DEC-2026-10-06-G5300-ONLY). In V3 mode the G53 stop logic is
not consumed, so the pipeline gets its own bound, independent of V3 code (DECISIONS D-008):

- reverse step / direction inhibit: the allowed ceiling decays from the published value at >= the BDE8 rate
  (3.5 Iq/ms, full scale in 130 ms) — same as G5300 reverse.
- crank stopped (G53 true-stop or native real_stop): the ceiling holds for at most `T_STOP_HARD` (*candidate*
  1500 ms), then decays to 0 within 300 ms. The distance bound joins in Milestone D once §7.4 exists, expressed in
  estimator distance with a stated resolution — never as a bare 2.0 m from wheel pulses (one pulse = 2.218 m).
- **Re-open** (R1-#4): only after forward steps resume, at no more than the BDE8 rise rate (3.5 Iq/ms) from the
  published value.
- standstill: the §2.1 predicate -> FORCE_ZERO.
- The backstop is a ceiling (`min`), never a demand. Brake/fault keep the native SAFETY 200 ms path upstream.
- Verified with V3 forced to output its maximum (G1-BACKSTOP).

### 7.4 Motion estimate (contract now, implementation in Milestone D)

The wheel sensor gives one pulse per 2.218 m, reads 0 below ~3 km/h and needs two pulses after a stop
(R1-#13). A pipeline-owned estimator provides `speed_est`, `distance_est` and `rel_accel` with a quality flag:

- wheel pulses with interpolation between pulses;
- while the motor drives: `erps × ratio`, where `ratio = wheel speed / erps` is learned over the last pedalled
  seconds (the gear cannot change while the crank is stopped); `rel_accel = Δerps/erps` is gear-independent;
- later, an IMU longitudinal acceleration when the sanitised motion input is valid.

Carry caps, the carry acceleration cancel, the Milestone D standstill predicate and the backstop distance bound read
only this estimator.

## 8. Envelopes and dynamic range (Milestone E/F — contracts only now)

- **Max Torque** (E): ceiling on motor torque demand, per level. Expressed as % of the motor's rated torque until a
  physical Nm calibration exists (torque constant is [UNKNOWN] on M820).
- **Max Power**: closed-loop battery-current limit through g1, per level, from W (D-034, supersedes the earlier
  "P1 % only" line). One power owner.
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

Superseded by MODE_CHARACTER.md (owner override 2026-10-07): five separated layers (intent / mode character /
physical envelope / transient / safety), autonomous modes ECO, TRAIL, SPORT, SPORT+, AUTO with complete firmware
defaults, BASIC macros (Assist, Max Torque, Max Power, Response, Start, Carry) and ADVANCED parameters, DEFAULT +
override storage, configured vs effective readback. Milestone C keeps the legacy G5300 characteristic; the Assist
macro and envelopes become active in the mode-character milestone.

## 11. Telemetry (DIAG build only)

One extra DIAG frame group, only the fields needed to answer "why this Iq now": `intent` (I), `env_equiv`, `kappa`,
`E_short`, `phase`, `phase_aligned`, `template_mode`,
`template_conf`, `expected_effort`, `release_class`, `carry_score`, `carry_state`, `carry_remaining_ms`,
`carry_remaining_cm`, `base_target_iq`, `v3_demand_iq`, `backstop_iq`, `final_iq` (existing), `cadence`, `rel_accel`,
`terrain_est` (F), `imu_valid`, `engine_active`, `engine_requested`. No V3 frame is emitted in G5300 mode. NORMAL build: unchanged frames.

## 12. Resources

- RAM: ring 192 B + template ≤ 64 B + state ≈ 100 B ≈ 0.4 KB of 48 KB. Measured after Milestone B (NORMAL, 5 KB
  stack): 840 B free. Budget: config v2 ≈ +0.1 KB net (one RAM image, saved view read from flash, active-level cache),
  carry + motion estimate (D) ≈ 0.2 KB, terrain (F) ≈ 0.1 KB; next lever per D-024.
- CPU: O(1) per step plus ≤ 48-step window walk on each step (≈ 10 k simple ops/s at 130 rpm), O(1) per control tick.
  Integer only. The firmware builds at -O0 except G53 sources at -O2; `assist_v3*.c` joins the -O2 list. Per-call
  budget and the spreading of per-revolution work: D-039, measured with DWT on the DIAG build before any V3 image rides.

## 13. Milestones

| Milestone | Content | Ride behaviour |
|---|---|---|
| A Foundation | baseline freeze, audits, this architecture, test matrix, config contract draft, IMU seam, sim inputs | unchanged |
| B Shadow | V3-1..V3-3 + V3-4/6 computed beside G53, telemetry, matrix A/B on identical inputs | unchanged |
| C Active release | engine V3: intent, classifier, static map, trajectory, backstop; legacy ratio/attack/power/battery/safety | release changed |
| D Carry | carry score + state machine, L4 freewheel drivetrain | carry added |
| E Envelopes | torque/power blend, Max Torque, wide cadence | characteristic changed |
| F Terrain/Auto | terrain state shadow -> dynamic range | AUTO changed |

## 14. Open items

- REVIEW 1 (2026-10-07T12:52:54+02:00, CHANGES_REQUIRED) issues #1-#20 are resolved in this revision; see
  reviews/REVIEW_1_ARCHITECTURE.md and DECISIONS D-015..D-022. Re-check requested before Milestone B code.
- D-008 backstop time bound and the EN 15194 run-on reference are [EXTERNAL_REFERENCE, not verified in this repo];
  owner confirmation before the ride.
- BDE8 G04 / D25 semantics [UNKNOWN] — matters only if the matrix shows a start-behaviour difference.
- On-bike size of the release hold [UNKNOWN] until a V3 DIAG ride; probes used model strokes.
- CONFIG_A survival across a BL820 update [UNKNOWN]: hardware check before relying on persist (RIDE_TEST_PLAN
  pre-step); a wiped record falls back to defaults safely.
