# REVIEW 1 — Assist Behavior V3 architecture (before active implementation)

```text
TIMESTAMP:   2026-10-07T12:52:54+02:00 (system clock)
REVIEWER:    independent Claude subagent, fresh context (read-only; did not author the architecture)
SCOPE:       docs/assist-v3/ARCHITECTURE_V3.md, DECISIONS.md, CONFIG_PROTOCOL_V3.md, TEST_MATRIX.md,
             audit/A_CONTROL_PATH.md, audit/D_CONFIG_CAN.md, checked against src/ at the worktree
WORKTREE:    C:\Projekty\eVistDrive-assist-v3, feature/assist-behavior-v3. Review started at a3fdf6f; HEAD moved
             to 5141300 during the review (runner selector + PROGRAM_STATE only, architecture docs unchanged).
             Uncommitted sim/ changes from another session were present and were not touched.
METHOD:      source reading + two host probes in the session scratchpad (not in the repo, RULE 70: observations,
             not qualified test tools):
             P1 env_vs_mean.py  - pure model of EB74 + D7EC envelope recurrence vs revolution mean
             P2 meanpeak.c      - the REAL production pipeline (g53_port*, g53_g1_limiter, ap2_limits,
                                  assist_pipeline, assist_modes, fast_iq_slew), same build line as
                                  tests/host/reverse_ramp_host.c: steady pedalling with a rippled load vs a
                                  constant load equal to its mean
VERDICT:     CHANGES_REQUIRED  (1 BLOCKER, 9 MAJOR, 10 MINOR)
```

The architecture is a good skeleton: the Boundary-B seam, "do not edit the transcription", one state `y`, the
angle-domain intent, the IMU sanitiser and the new-ID config block are all the right calls and I would keep them.
The blocker is quantitative: as written, Milestone C would ride with noticeably less assist than baseline, which
breaks requirement 9 and would make the A/B ride uninterpretable. Most MAJOR items are underspecified edges
(veto release, standstill predicate, engine switch, fallback template) where the current text would produce a
behaviour change nobody asked for.

## Answers to the review questions

| Q | Result | Issues |
|---|---|---|
| A one trajectory owner | ISSUE (MAJOR) | #3 R1 bypass / re-seed undefined, #4 ceiling release of backstop, #8 AUTO ratio limiter rate/time base |
| B static map with V3 intent | **ISSUE (BLOCKER)** | #1 mean vs peak-hold: 15-35 % less assist (L3), 30-60 % less (S+ AUTO); #2 wrong EB74 offset |
| C intent estimator | ISSUE (MAJOR) | #5 uniform fallback fires TRUE_RELEASE on every dead spot, #6 template lock-out on style change, #15 phase recovery, #16 hysteresis |
| D safety paths in V3 mode | ISSUE (MAJOR) | #7 standstill predicate, #9 PEDAL_STOP release rate not legacy, #14 EB74 re-arm interlock lost |
| E engine switch | ISSUE (MAJOR) | #10 switch at "published 0" can surge into the G53 shadow's held demand |
| F config protocol | PASS_WITH_ISSUES | #11 CONFIG_A erase by BL820 still unknown + single-erase persist, #17 inert bank fields as second owners, #18 generation/engine status |
| G test matrix | ISSUE (MAJOR) | #12 missing level-parity and several scenario tests; #19 SIL duplicates main.c drain; #20 G-EQ CSV columns |
| H rebuild risk for D/E/F | ISSUE (MAJOR) | #13 wheel sensor (1 pulse / 2.218 m, 0 below 3 km/h) cannot carry the distance cap or the carry/standstill logic; #1 fix must keep physical intent separate |

## Issues

### #1 — BLOCKER — Revolution-mean intent through the G5300 static map delivers much less assist than baseline

Evidence:
- `src/g53_port_chain.c:508-521` envelope: `env = cur` on attack, else `env = env*k/(k|1)` per 10 ms, `k = 8*cad`.
  Between two power strokes (half a revolution) it decays by about `exp(-375/cad²)`: 0.90 at 60 rpm, 0.97 at
  120 rpm, 0.39 at 20 rpm. So at baseline `env` is close to the stroke **peak** at normal cadence.
- `src/g53_port_chain.c:590-602, 680-720`: `c2 = env*cad*35/10000`, `c4 = c2*637/1000`, `d4 = c4*ratio/100`,
  `conv = d4*1000/lut(cad)`, `target = conv*40.96`. For fixed-ratio levels the whole map is linear in `env`;
  on S+ AUTO (`chain.c:610-626`, `ratio = r7 + (maxr-r7)*c4/den`) the ratio is itself proportional to `c4`, so
  assist is ~quadratic in `env` below the ratio cap.
- ARCHITECTURE_V3 §4.3/§5 feed the map with `env_equiv` derived from `I = E_long` = **revolution mean**.

Probe P2 (real pipeline, P = 700, 15 km/h, steady state, load `Lpk*(d + (1-d)|sin θ|)`; "ratio" = mean Iq with a
constant load equal to the stroke mean / mean Iq with the real rippled load; the constant-mean load goes through
the same EB74, so this isolates mean-vs-peak with the correct EB74 offset):

| Level | profile d (dead-spot depth) | 20 rpm | 45 rpm | 60 rpm | 90 rpm | 120 rpm |
|---|---|---|---|---|---|---|
| 3 SPORT, Lpk 1500 | 0.1 (deep) | 0.79 | 0.69 | 0.67 | 0.64 | 0.64 |
| 3 SPORT, Lpk 1500 | 0.3 | 0.91 | 0.77 | 0.76 | 0.73 | 0.72 |
| 3 SPORT, Lpk 1500 | 0.6 (flat) | 0.97 | 0.90 | 0.88 | 0.85 | 0.85 |
| 4 SPORT+ AUTO, Lpk 1500 | 0.1 | 0.56 | 0.46 | 0.44 | 0.41 | 0.41 |
| 4 SPORT+ AUTO, Lpk 1500 | 0.3 | 0.76 | 0.58 | 0.58 | 0.53 | 0.52 |

(At Lpk 3000 the numbers are similar until the 0.65·P cap at 455 Iq masks the difference.) P1, the pure model,
agrees with P2 within a few percent. [PROBE, observation only; rider profile shape is a model, not a ride log.]

So a rider with a normal stroke would get roughly a quarter to a third less assist on SPORT and up to half on S+,
and the shortfall depends on cadence and pedalling style, so no single scale factor fixes it.

Fix (required before Milestone B code):
1. Keep two quantities, explicitly named:
   - `I` = physical rider effort (revolution / angle-window mean, CLU). Used by the classifier, carry score,
     Milestone E torque/power blend, Milestone F terrain. Never rescaled.
   - `env_equiv` = the **envelope-equivalent** of the current stroke, used **only** as the input of the G5300
     static map in Milestones C/D.
2. Compute `env_equiv` as the steady-state mean of the exact D7EC recurrence (same `k = 8*cad`, same 10 ms step,
   same integer `k|1` form) applied to the template-reconstructed stroke
   `x(θ) = EB74_active(I · s(θ))`, i.e. `max(0, 750 + I·s(θ)·2450/6000 − 820)` (see #2), evaluated once per
   revolution (≤ 300 recurrence steps at 20 rpm, ≤ 50 at 120 rpm: cheap), or tabulated offline as
   `κ(cad, peak/mean of s)` from the same pure recurrence. All dynamics (release, attack) stay in V3-6: when the
   classifier overrides `I`, `env_equiv` follows the new `I` immediately — no envelope state carries over.
3. The fallback template must be a population-typical prior (fixed table, peak/mean ≈ 1.4-1.6, chosen from logs or
   the matrix), **not** `s ≡ 1`. With `s ≡ 1`, `κ = 1` and assist would visibly creep up by 20-50 % while the
   template converges (8+ revolutions; 25+ s at 20 rpm), after every start, glitch and reverse.
4. Add test G1-LEVEL (see #12): V3 steady-state mean Iq within ±5 % of baseline across L1-L5, S+ AUTO,
   20..130 rpm, d ∈ {0.1, 0.3, 0.6}, three load levels, with converged and with prior template.

### #2 — MAJOR — `env_equiv` uses the EB74 zero, but the chain subtracts the active threshold

Evidence: `src/g53_port_boundaries.c:113-131`: `cur = IIR(source − threshold)`, `threshold = 820` while
`d7ec_rider != 0` (assisting) and `zero + 245 = 995` otherwise; inputs at or below the threshold hard-reset the
filter. ARCHITECTURE_V3 §5 writes `env_equiv = max(0, 750 + I·2450/6000 − zero)` with zero = 750: a constant
+70-count (≈ 171 CLU) bias. It partially hides #1 at low effort (P1: +5..+14 % at 20 rpm with a flat stroke) and
adds a load-dependent error everywhere else. G1-STATIC compares the map at equal `env`, so it cannot catch this.
Fix: apply the EB74 transfer with the active threshold (820 when the V3 demand is engaged, 995 to engage) inside
the reconstructed stroke of #1; read zero/threshold through a read-only accessor so a future auto-zero arm stays
consistent.

### #3 — MAJOR — R1 bypass leaves veto release and re-engagement undefined (brake, assist off, limits, g1)

Evidence: `src/assist_pipeline.c:174-193, 215`: R1 limits the climb back after a native veto
(`pulled_down = final < post-limit G53 request`) to the BDE8 rate. ARCHITECTURE_V3 §2.1 bypasses R1 and says
"V3 re-engages from the published value", but V3 is never told about the veto (`brake` is observation only) and
`y` is in the pre-g1, pre-limit domain while `last_published_iq` is post-g1, post-limit.
- If V3 does nothing: after a brake release (SAFETY 200 ms to 0) the published value steps back to `y·g1`, limited
  only by the 6.84 Iq/ms electrical guard (0.65·P in ~66 ms). Requirement 9 keeps legacy brake behaviour,
  which includes the gentle R1 re-engagement.
- If V3 clamps `y` to the published value every tick: every speed-taper, thermal or UV limit pulls `y` down, and the
  trajectory changes legal-speed behaviour (baseline G53 never sees the limiters).
Fix: either keep R1 in V3 mode (it is a veto-release limiter, not a demand shaper; it only binds after
`native_cut`, assist-off or the standstill zero), or define one explicit re-seed rule: on the tick a pipeline veto
ends, `y := min(y, last_published·4096/g1)`; never re-seed from limiter clamps. Add a brake-release-while-pedalling
test to G1-SAFE / G1-TRAJ in both engines.

### #4 — MAJOR — Backstop (D-008) defines the fall but not the release of its ceiling

Evidence: ARCHITECTURE_V3 §7.3. The backstop is a decaying ceiling keyed on reverse/stop. Nothing says what happens
when forward steps resume: if V3's `y` stayed high (that is exactly the V3 bug the backstop guards against, or a
long carry), the ceiling lifts and the published value jumps to `y` at the electrical-guard rate.
Fix: the backstop ceiling re-opens at no more than the BDE8 rise rate (3.5 Iq/ms) from the published value, and
only after forward steps resume; add the "reverse → forward with V3 forced to max" and "stop past T_STOP_HARD →
restart" cases to G1-BACKSTOP.

### #5 — MAJOR — With the uniform fallback template the classifier calls TRUE_RELEASE on every dead spot

Evidence: ARCHITECTURE_V3 §4.2-4.4. Below `c_min` or outside the trusted cadence band, `s ≡ 1`. Then `Σs` over
8 steps is 8 ≥ `S_min`, so the short window stays at 30°. A dead spot of a normal stroke lasts ±30-45° around
TDC/BDC with effort ~0.1-0.3 of the mean, so `ρ = E_short/I ≈ 0.1-0.3 < R_rel = 0.5`: TRUE_RELEASE twice per
revolution, override intent `E_short`, demand falls at the Response rate. That is the exact failure requirement 1
forbids. It happens after every start, glitch and reverse (confidence 0) and, by the text, always outside the
"trusted cadence band", i.e. probably in the < 30 rpm regime requirement 2 targets. Learning at α = 1/8 per visit
takes ≥ 8 revolutions (≥ 24 s at 20 rpm), so this is not a corner case.
Fix: at low confidence do not use a short window at all. Use the angle-domain rule that needs no template:
TRUE_RELEASE when the **maximum** per-step `obs` over the last 180° (every 180° contains a power stroke) is below
`R_rel · I`; ATTACK when the 180° mean exceeds `R_att · I`. Detection ≤ 180° at any cadence. Also define the
"trusted cadence band" numerically and prove 20 and 130 rpm in the matrix with the template active (#12).

### #6 — MAJOR — Template learning is gated on NORMAL_PRESSURE, which locks out adaptation to a new pedalling style

Evidence: ARCHITECTURE_V3 §4.2 ("Learning only in NORMAL_PRESSURE"). Sit → stand on a climb, a different gear or
fatigue changes the stroke shape. The old template then misjudges the new dead spots (ρ < 0.5 in a deeper dip →
false TRUE_RELEASE, or ρ > 1.4 on a sharper peak → false ATTACK), learning is suspended because the class is not
NORMAL, and the misclassification persists.
Fix: gate learning on long-window stability, not on the per-step class: after a revolution completes, if `E_long`
of that revolution is within ±15 % of the previous one (intent did not change; only the shape did), learn from
that revolution retroactively from the 96-entry ring, and raise the residual-based confidence drop. Add a
sit→stand profile switch at constant mean to G1-TPL / G1-CLS (no assist dip allowed).

### #7 — MAJOR — Standstill FORCE_ZERO predicate is either a no-op or too broad

Evidence: `src/assist_pipeline.c:205-207` (baseline `stock_hard_zero = !normal_permission && m2aa == 0`);
BDE8 state 1 with `speed ≤ 0` zeroes Q5C at once (`src/g53_port_chain.c:1516-1552`, labels BF4A -> C1B6).
`tests/host/reverse_ramp_host.c:156-158` pins "standstill reaches zero within 2 ms with FORCE_ZERO".
ARCHITECTURE_V3 §2.1 re-derives it as "V3 demand == 0 and speed_native ≤ 0": that only fires when the demand is
already 0, so a reverse step or a stop at standstill now falls at the V3/backstop rate instead of ≤ 2 ms. §7.3
then says "standstill: FORCE_ZERO as at baseline" with no predicate: if it means "speed == 0", it kills every start
from rest, because `MS.Speedx100` stays 0 until the **second** wheel pulse (`src/main.c:2644-2650`; 1 pulse per
2.218 m wheel, `inc/config.h:159,175`), i.e. the first ~4.4 m of an uphill start get no assist.
Fix: one written predicate, independent of V3 internals: `standstill_zero = speed_native ≤ 0 ∧ (G53 PAS true-stop
∨ native real_stop ∨ direction_inhibit/reverse)`. Prove it differentially against baseline: V3-mode zero time at
standstill ≤ baseline for stop/reverse × load held/released, and "start from rest, speed 0, crank turning, load
high → assist flows".

### #8 — MAJOR — The ratio rise limiter retained in the static map is a hidden rise shaper with a call-rate-dependent rate

Evidence: `src/g53_port_chain.c:673-683`: `ratio` may rise by `step = D+122` (= `auto_rise_step` = 10,
`chain.c:2032`) **per D7EC call** (10 ms). ARCHITECTURE_V3 §2.1 keeps "its own ratio state inside the static map
call (same rule)", but V3 runs at the pipeline rate (4 kHz with `elapsed_ticks`). Called per pipeline tick, the
limiter becomes 40x faster; called per 10 ms, it is a second rise shaper that binds on S+ AUTO attack
(1 → 525 takes 0.52 s, slower than the slot-8 trajectory 0.26 s) and on every level change.
Fix: state explicitly that `g53_static_target()` advances the ratio state per **elapsed 10 ms** (accumulator), and
record it as an accepted legacy-characteristic shaper for C/D (it keeps legacy S+ attack). In E/F move the ratio
slew into V3-6 so the single-owner rule becomes literal. Add a level-change and S+ attack case to G1-STATIC
comparing against the transcription in time, not only at steady state.

### #9 — MAJOR — PEDAL_STOP with load released uses R(Response), not the legacy stop rate (requirement 9)

Evidence: ARCHITECTURE_V3 §6.1 table: "PEDAL_STOP, Milestone C: load released: R(Response)". Baseline: crank
stops, load released → zero-reset → BDE8 −50/ms = 3.5 Iq/ms, 116 ms from 300 Iq (audit A §6 table). Candidate
R(Response) is full scale in 150-600 ms (0.76-3.0 Iq/ms); ECO at Response 40 ≈ 420 ms. Milestone C would make
stopping slower than today on most levels. Requirement 9 says STOP stays legacy in C.
Fix: Milestone C PEDAL_STOP uses the legacy rates exactly (released: 3.5 Iq/ms; held: 0.455 Iq/ms after the G53
true-stop timeout), Response applies only to TRUE_RELEASE / gradual fall while pedalling. Add a baseline-vs-V3
stop-timing metric to the matrix with a ≤ baseline acceptance.

### #10 — MAJOR — Engine switch at "published request 0" can surge into the G53 shadow's held demand

Evidence: ARCHITECTURE_V3 §2.2 latches the switch "while the published request is 0 (or at standstill)". The G53
chain keeps running in shadow, so when V3 has dropped to 0 after a fast TRUE_RELEASE, the shadow G53 still holds
its envelope demand for 5-7 s (audit A §6). A switch V3 → G5300 at that moment publishes the held m2aa, limited
only by the fast slew (and R1 would not engage, because `pulled_down` was computed against the V3 request). The
same happens if the switch latches during a brake (published 0, demands high). "Or at standstill" allows a switch
with a non-zero request (uphill start), giving a step either way. V3 → reset also starts with a cold template
(#1/#5).
Fix: latch only when the published request **and both engines' demands** are 0 and no veto is active; on the
switch tick set `pulled_down = true` so R1 governs any climb; drop the standstill clause. Expose
`engine_requested` vs `engine_active` in CAPS/STATUS. Add these cases to G1-SEL.

### #11 — MAJOR — CONFIG_A survival across a BL820 update is still unknown, and persist is erase-then-write in one page

Evidence (verified): `ldscripts/gd32f30x_flash.ld:5-9,165-172`: FLASH `0x08005000 + 230K = 0x0803E800` ends exactly
at CONFIG_A, with an ASSERT that the image ends before it; `tools/build_firmware.py:25,246,573` repeats the check.
So the **linked image** cannot overlap CONFIG_A. But `tools/prepare_m820_bl820.py:75-83` writes only
`len(payload) & 0xFFFF` (size modulo 64 KiB) into the container header, so the BL820 cannot know the image size
from the header and probably erases a fixed region whose end is [UNKNOWN]. If it erases the stock app span up to
0x0803F000, CONFIG_A is wiped while CONFIG_B (MotorParams) survives — plausible, not proven.
Separately, CONFIG_PROTOCOL_V3 §4 persists by page erase + program: a power loss in between loses the record
(defaults, safe but surprising).
Fix: HW check before Milestone C persist work: write a marker record to CONFIG_A, perform a BL820 update, read back
(EV record). Use a two-slot append-only log in the 2 KB page (≈ 130 B records, 15 slots, erase only when full, newest
valid CRC wins) so a persist never destroys the last good record. If CONFIG_A is wiped by updates, fall back to the
after-footer area of CONFIG_B (audit D §5.3) or accept "defaults after update" as a documented owner decision.

### #12 — MAJOR — Test matrix misses tests that are essential for requirements 1-4, 8 and 9

Missing (add to TEST_MATRIX):
- G1-LEVEL steady-state assist parity (#1, #2), with converged and prior template.
- Pedal-pressure tracking (req. 1): ramps of mean effort up and down at 20/60/120 rpm; metric = tracking error of
  demand vs baseline characteristic, monotonicity, no dip at dead spots (not only "gradual release").
- Fallback/start: first 3 revolutions after start, after a reverse and after a PAS glitch, at 20 rpm with deep dead
  spots: zero TRUE_RELEASE classifications (#5).
- Style switch sit → stand at equal mean (#6); asymmetric legs at 20 and 130 rpm.
- Brake release while pedalling and assist-off → on, both engines (#3); backstop release (#4).
- Standstill: start from rest with speed 0 for the first 4.4 m; stop/reverse at speed 0 with load held/released (#7).
- Legacy stop timing (#9); level change during riding and S+ attack timing (#8).
- Engine switch during release, during brake, at standstill with demand (#10).
- PAS ring overflow (> 32 events in a foreground stall, `src/pas_sampler.c:96-110`) and INVALID two-bit jumps:
  phase marked invalid, no false class.
- Gear-shift unload (0.2-0.5 s partial release) and ratcheting (back/forward quarter strokes) on a technical climb.
- Long run (≥ 1 h simulated) for template renormalisation drift and counter wrap (`crank_steps`, tick wrap).
- Class-transition count in steady riding = 0 (oscillation check, #16).
- Requirement 7: write while a foreign transfer is live, transfer timeout, generation wrap, persist with power loss
  between erase and program (#11), engine field write while riding (latched, not immediate).
- Discovery from P2, to be checked not chased: on the host pipeline, pedalling **unloaded** at 15 km/h for 0.5 s and
  then loading produced **no G53 assist at all** (BDE8 state 1, D7EC hold-off window re-armed every call via
  `D+24`, `src/g53_port_chain.c:325-345`), while starting PAS together with the load engages normally. This may be
  a harness artefact (constant speed without wheel pulses), but V3 bypasses that path, so the matrix must contain
  "unloaded pedalling at speed → load" in both engines and explain any difference. [PROBE, low confidence]

### #13 — MAJOR — Wheel speed sensing cannot support the distance caps, the carry cancel or a low-speed standstill rule (Milestone D risk, backstop now)

Evidence: `inc/config.h:159,175,187`: one pulse per 2.218 m wheel revolution; `MS.Speedx100` becomes 0 after
2.65 s of silence, i.e. below ~3.0 km/h; at 5 km/h a pulse arrives every 1.6 s; the first pulse after a stop only
seeds the reference (`src/main.c:2644-2650`). Consequences:
- D-008 `D_STOP_HARD = 2.0 m` is smaller than one pulse: the distance cap is either "first pulse" (2.2 m) or
  unmeasurable. The EN 15194 run-on argument [EXTERNAL_REFERENCE] cannot rest on it.
- Carry happens exactly at walking-pace obstacles, where speed reads 0 and dv/dt from pulses is one sample per
  1.6-2.6 s. Any standstill rule keyed on `speed == 0` would kill carry there (#7).
Fix (design now, implement in D): a distance/speed estimator owned by the pipeline: wheel pulse count plus
interpolation, and while the motor drives, `erps × learned (wheel speed / erps)` ratio captured during the last
pedalled seconds (the gear cannot change while the crank is stopped). Specify the cap in pulses + estimator with a
stated resolution, keep the time cap as the hard bound, and add `static_assert(carry_hard_max ≤ backstop bounds)`.

### #14 — MINOR — The EB74 "pedal must be unloaded once" re-arm interlock loses its owner in V3 mode

Evidence: `src/g53_port_boundaries.c:97-111`: after every `g53_port_reset()` (power-on and every pipeline reset on
an owner change: comms loss, battery trip, Walk, calibration) EB74 outputs 0 for 30 ticks and then until the source
stays at or below the threshold for 10 consecutive ticks. P2 reproduced it: a load applied from t = 0 gives no
assist ever. V3 reads `load_ctrl` directly and would assist immediately after such a reset with a foot on the pedal.
Fix: gate V3 engagement on a read-only "EB74 armed" accessor (startup and check window complete), so the restart
semantics stay baseline in C.

### #15 — MINOR — Phase alignment is relearned from zero after every glitch

Evidence: ARCHITECTURE_V3 §4.1-4.2; INVALID events are two-step jumps (±7.5°), overflow loses order
(`src/pas_sampler.c:96-110`, `pas_sampler_take_overflow`). Dropping confidence to 0 forces ≥ 8 revolutions in
fallback. Fix: include overflow in `pas_glitch`; after a glitch, re-align by circular cross-correlation of the last
revolution against the template (96 × NB ops once per revolution) and restore confidence if the correlation peak is
clear. Also note torque-sensor latency is constant in time, so it shifts the template by more steps at 130 rpm than
at 60 rpm; with NB = 32 that is visible. Learn a latency offset or keep NB ≤ 24.

### #16 — MINOR — Override exit band and hysteresis are not specified

Evidence: ARCHITECTURE_V3 §4.4 ("until `E_long` agrees with `E_short` again"). Without a numeric agreement band,
hysteresis and a minimum dwell in angle, ATTACK/TRUE_RELEASE can chatter near R_att/R_rel. Fix: define the band
(e.g. |ρ − 1| < 0.2 to exit), enter/exit hysteresis, min dwell 90°; add the transition-count metric (#12).

### #17 — MINOR — Inert bank fields are still on the wire with the same meaning as new V3 parameters

Evidence: `0x6020/0x6021` bank records carry `release_ms`, `max_iq_pct`, `support_ratio_pct`,
`max_motor_power_w`, `iq_rise_fast_ms`, `smooth_start` (`inc/assist_modes.h:258-282`, audit D §1.5). None is
consumed on PEDAL (`max_iq_pct` is computed and then forced to `AP2_LIMITS_NO_LEVEL_CEILING`,
`src/assist_pipeline.c:139`). V3 adds Response and Max Torque in a new block. D-011 covers the P0/P1 owners only.
Fix: CONFIG_PROTOCOL_V3 §1 lists these bank fields as DEPRECATED / inert in both engines, CANable hides them, and
Milestone E Max Torque reuses the existing `level_iq_limit` path in ap2_limits (one ceiling owner).

### #18 — MINOR — Generation and engine status details

`config_generation` is u16 and `0xFFFF` means "no check" on write: the counter must skip 0xFFFF. CAPS/STATUS should
expose the active engine and a pending engine switch (#10). Defaults for an absent record use engine = V3 in the
candidate build; recommend G5300 as the absent-record default until Milestone C passes its ride (owner decision).

### #19 — MINOR — The SIL/L4 harnesses duplicate the main.c PAS drain

Evidence: `sim/evist_sil.c:589-617` re-implements the drain loop; L4 and controller_lab do the same. The new signed
accumulator, last-step tick and glitch/overflow flag would be written twice and could diverge, and the matrix would
then not test the production code. Fix: put the accumulator in a production module (e.g. `crank_phase.c` fed by
`pas_step_event_t`) called by main.c and by every harness.

### #20 — MINOR — G-EQ byte identity needs explicit rules

G-EQ is achievable (the plumbing adds unused fields; host runs are deterministic), but only if (a) the harness sets
`engine = G5300` explicitly (the candidate default is V3 and the flash stub has no record), (b) V3 telemetry columns
go to a separate CSV or are excluded from the comparison, (c) DIAG V3 frames are not emitted in G5300 mode in any
simulated CAN trace. Write these three rules into TEST_MATRIX G-EQ.

## Other observations (no action required for REVIEW 1)

- IMU seam (§3.3): PASS. Sanitise-then-read is sound and the G1-IMU test is the right proof. For F, define who owns
  mounting-offset calibration and the bike-frame convention (driver side, not V3).
- CPU/RAM (§12): PASS on paper; the per-step window walk and divisions are cheap at 208 steps/s. Measure on target
  as planned.
- Envelopes and terrain (§8): PASS_WITH_NOTE. The G5300 map is already cadence-scaled (`c2 ∝ env·cad`, LUT
  saturating at 70 rpm), so the Milestone E torque/power blend must **replace** the `c2`/LUT cadence term, not be
  stacked on top of it.
- Carry decay (§7.2): express carry as a target profile and keep V3-6 the only rate limiter; `decay(t)` must be
  slower than R(Response) or the trajectory will distort it.
- Doc drift: audit A cites `g53_port_boundaries.c:236-320`; the file has 143 lines (EB74 step at :91-134,
  Boundary B at :136-143). PROGRAM_STATE was stale at a3fdf6f (updated in 5141300).

## Required before active implementation (gate for REVIEW 1 re-check)

1. #1/#2: written normalisation (`I` vs `env_equiv`), prior template, G1-LEVEL in TEST_MATRIX.
2. #3/#4/#7/#9/#10: one written rule each (veto release, backstop release, standstill predicate, legacy stop rates in
   C, switch latch).
3. #5/#6: fallback classifier without short window; learning gate on long-window stability.
4. #8: ratio limiter time base.
5. #11: HW check plan for CONFIG_A; append-only record format.
6. #12/#13: matrix additions; distance/speed estimator contract (implementation may wait for D).

MINOR items can be closed during Milestone B.

## Re-check (rev 2)

```text
TIMESTAMP:   2026-10-07T13:01:13+02:00 (system clock)
REVIEWER:    independent Claude subagent (same reviewer as REVIEW 1), read-only
TARGET:      commit 8ebdae5 (ARCHITECTURE_V3 rev 2, DECISIONS D-015..D-022, TEST_MATRIX, CONFIG_PROTOCOL_V3)
SCOPE:       only the fixes for issues #1-#20, plus new defects introduced by those fixes
METHOD:      diff 5141300..8ebdae5 + one more host probe P3 (copy of tests/host/reverse_ramp_host.c, two extra
             cases at speed 0, run on the production pipeline in the scratchpad; RULE 70 observation)
VERDICT:     CHANGES_REQUIRED (narrow: 2 new MAJOR defects in the fixes for #1 and #7; everything else resolved)
```

### Status per issue

| # | Status | Note |
|---|---|---|
| 1 | PARTIAL | The structure is right: `I` and `env_equiv` are separate, there is a prior template and G1-LEVEL exists. But the scale formula is defective, see N1; the G1-LEVEL tolerance with the prior template is not achievable, see N3 |
| 2 | RESOLVED | Active threshold used. Small ambiguity about whose "engaged" state selects 820/995, see N4 |
| 3 | RESOLVED | R1 kept; `pulled_down` computed against the post-limit V3 request; `y` never re-seeded by limiters |
| 4 | RESOLVED | Backstop re-opens only after forward steps resume, at <= 3.5 Iq/ms; covered by G1-BACKSTOP |
| 5 | RESOLVED | The fallback classifier (180 deg max / 180 deg mean) needs no template; the trusted band is defined (15..150 rpm) |
| 6 | RESOLVED | Learning is now gated on revolution stability (learned from the ring after the fact). G1-STYLE must show that confidence drops to fallback within one revolution of a style change (N5) |
| 7 | PARTIAL | The predicate is now written down, but it is **more aggressive than baseline** for a held load at speed 0, see N2 |
| 8 | RESOLVED | The ratio limiter advances per elapsed 10 ms; it is accepted as a legacy shaper in C/D and moves into V3-6 in E; G1-STATIC-T added |
| 9 | RESOLVED | Legacy stop rates in C; Response never slows a stop; G1-STOP added |
| 10 | RESOLVED | Switch latches only with both demands at 0 and no veto; `pulled_down = true` on the switch tick; standstill clause removed; both engine values in CAPS |
| 11 | PARTIAL | The hardware check plan is good. The append-only claim "never the last good one" is false with a single erase page, see N6 |
| 12 | PARTIAL | All requested rows were added. G1-STAND's one-sided "<= baseline" acceptance hides N2, and G1-LEVEL needs a split tolerance (N3) |
| 13 | RESOLVED | Motion estimator contract section 7.4 / D-022; distance bound removed from the C backstop; static_assert on carry vs backstop |
| 14 | RESOLVED | Engagement gated on EB74 "armed" |
| 15 | RESOLVED | Overflow counts as a glitch; re-alignment by cross-correlation; NB <= 24 unless a latency offset is learned |
| 16 | RESOLVED | Enter/exit thresholds, 90 deg minimum dwell, G1-OSC |
| 17 | RESOLVED | Bank fields DEPRECATED/inert; Max Torque goes through `level_iq_limit` |
| 18 | RESOLVED (default: accepted with conditions) | Generation skips 0xFFFF; engine_active/requested bytes added. D-020's reason is acceptable, see below |
| 19 | RESOLVED | Production `crank_phase.c` shared by main.c and all harnesses |
| 20 | RESOLVED | Three G-EQ rules written into TEST_MATRIX |

### D-020 judgement (#18, absent-record default V3)

**Accepted, with conditions.** The candidate BIN exists to ride V3. CANable has no V3 UI yet, so a G5300 default
would make the test build behave like the baseline and the A/B ride would be empty. Reflashing 0.638 is a real
fallback. The safety layers (native_cut, backstop, R1, limits) do not depend on the engine. Conditions:

- (a) The default applies to the V3 **candidate/test build only**. The default for a released build is decided again
  at RELEASE_REPORT.
- (b) RIDE_TEST_PLAN records the on-trail fallback, since there is no UI switch: assist level 0 or power-off, then
  reflash 0.638 (or a raw CANable write of engine = 0). The owner acknowledges this before the first ride.
- (c) Once a record has been persisted, the default no longer applies. The plan must say so, so that a later
  "defaults" test is not confused by an old record.

### New defects introduced by the fixes

#### N1 - MAJOR - kappa is defined in the post-threshold domain: it is unbounded near the deadband and makes assist surge on attack

Evidence: ARCHITECTURE_V3 rev 2 section 4.3: `kappa = env_equiv_ss / EB74_active(I_rev)` and between recomputations
`env_equiv = kappa * EB74_active(I)`, with `EB74_active(L) = max(0, 750 + L*2450/6000 - 820)`.
`EB74_active` subtracts ~70 counts, so near the deadband the denominator goes to 0 while the stroke peaks are still
above threshold, which is exactly the case where baseline assists:

- I_rev = 300 CLU, stroke depth 0.1: EB74_active(300) ~ 52, the steady envelope ~ 114, so kappa ~ 2.2
  (the true peak/mean in the load domain is ~ 1.48).
- I_rev = 200 CLU: kappa ~ 4.3.
- I_rev <= 171 CLU: kappa = x/0, undefined.

If the rider then attacks (spin lightly, then push into a climb), kappa is not recomputed until the revolution
completes (up to 3 s at 20 rpm). At I = 1500 CLU, env_equiv ~ 4.3 x 542 ~ 2330 instead of ~ 836: the target
saturates (conv clamp, 0.65*P) and V3-6 climbs toward full assist at the legacy accel rate for up to a revolution.
That is an assist surge nobody asked for. A light-load start can also divide by zero.

Fix:
- Define the scale in the **load domain**: choose `kappa_L` so that `EB74_active(kappa_L * I_rev) = env_equiv_ss`,
  i.e. `kappa_L = (env_equiv_ss + thr - 750) * 6000 / (2450 * I_rev)`.
- Clamp `kappa_L` to [1.0, kappa_max] (kappa_max ~ 2.5 from the matrix).
- Use the prior's `kappa_L` when I_rev is below a floor (e.g. 300 CLU) or the revolution is not stable.
- Then `env_equiv = EB74_active(kappa_L * I)`. This is bounded and well defined at every load, keeps engagement on
  the envelope-equivalent load (as baseline engages on stroke peaks), and follows I at once.
- Add a G1-LEVEL/G1-TRACK case: light spin (I ~ 200-400 CLU), then a step to 1500-3000 CLU within one revolution
  at 20 and 60 rpm. The V3 target must never exceed the baseline target by more than 10 %.

#### N2 - MAJOR - The standstill predicate zeroes a held load at once; baseline ramps it down over ~1 s

Evidence: the rev 2 predicate `standstill_zero = speed_native <= 0 AND (G53 true-stop OR real_stop OR
direction_inhibit)` (section 2.1, D-019). Probe P3 ran on the production pipeline at speed 0, starting from 455 Iq:

| Case | Baseline time to zero |
|---|---|
| crank stops, load held | 1022 ms (t50 522 ms, D3E ramp, no hard zero) |
| crank stops, load removed | 32 ms |
| reverse step | 2 ms |

Under the rev 2 predicate, the load-held case becomes FORCE_ZERO at the G53 true-stop (~21 ms here). The wheel
sensor reads 0 below ~3 km/h, so this hits every crank pause while crawling up a technical climb, and every pause
during a steep start: a hard cut to zero, then a rise from 0. That is a legacy-STOP change (requirement 9) in the
TRAIL use case. G1-STAND only demands "<= baseline", so it passes and hides the regression.

Fix:
- Narrow the predicate to what baseline does:
  `speed_native <= 0 AND (direction_inhibit OR (crank stopped AND V3 stop target == 0))`.
  The second branch is the D7EC zero-reset equivalent: load released, or the legacy 0.455 Iq/ms held-load ramp has
  reached 0. A held load at standstill then follows the legacy ramp, still bounded by the backstop.
- Make G1-STAND two-sided for the load-held case: |V3 - baseline| zero time <= 50 ms, and <= baseline for the
  released and reverse cases.

#### N3 - MINOR - G1-LEVEL +-5 % with the prior template cannot pass

One fixed prior has a single peak/mean, but real strokes span ~1.17 (flat stroke, depth 0.6) to ~1.48 (deep dead
spot, depth 0.1) in the REVIEW 1 model. A +-5 % parity across depths 0.1/0.3/0.6 with the prior is impossible by
construction. Fix: +-5 % with a converged template; with the prior, +-15 % plus no step larger than 10 % when the
template converges.

#### N4 - MINOR - Whose "engaged" state selects 820 or 995 is ambiguous

"Read through an accessor" would give the shadow G53's threshold. That depends on the shadow D7EC state, which
differs from V3's during a V3 fast release and at start. Fix: V3 selects 820/995 from its **own** engaged flag
using the same constants, and reads only the zero through the accessor.

#### N5 - MINOR - Confidence drop rate on a style change is not specified

With the old template still "confident", false TRUE_RELEASE can occur during the first revolutions after sit -> stand,
before the revolution-stability learning catches up. Fix: state that one revolution with residual above X drops c
below c_min (fallback mode). G1-STYLE asserts no assist dip from the first revolution on.

#### N6 - MINOR - A single-page append-only log cannot guarantee "never the last good record"

On the GD32F303 the 2 KB page is the erase unit, and CONFIG_A is one page. When the log is full, erasing it destroys
the last good record before the new one is written, so a power loss in that window (once per ~15 persists, at
standstill) loses it. Fix: either use two pages (CONFIG_A + the after-footer area of CONFIG_B, ping-pong), or correct
the text to "at most once per page-full cycle, a power loss during the erase/write window falls back to defaults"
and test exactly that.

### Residual items for the author (before Milestone B code)

1. N1: load-domain kappa_L, clamped, with a prior below the floor, plus the light-spin -> attack test.
2. N2: narrowed standstill predicate plus two-sided G1-STAND.
3. N3-N6: text and test-spec corrections (can be closed during Milestone B).

Once N1 and N2 are fixed as described, this review can close as PASS_WITH_ISSUES without another full pass;
N3-N6 can be checked at the Milestone B gate.

## Closure (rev 3)

```text
TIMESTAMP:   2026-10-07T13:05:09+02:00 (system clock)
REVIEWER:    independent Claude subagent (same reviewer), read-only
TARGET:      commit 0214a13 (diff 8ebdae5..0214a13: ARCHITECTURE_V3 2.1/4.2/4.3, DECISIONS D-019/D-020/D-023,
             TEST_MATRIX G1-LEVEL/G1-STYLE/G1-STAND, CONFIG_PROTOCOL_V3 section 4)
VERDICT:     PASS_WITH_ISSUES - REVIEW 1 may close; the minor residuals below go to the Milestone B gate
```

| Item | Status | Note |
|---|---|---|
| N1 | RESOLVED | The factor is now computed in the load domain: `kL = clamp(L_eq / I_rev, 1.0, 2.5)`, with the prior template's `kL` below ~300 CLU and `env_equiv = EB74_active(kL * I)`. It stays bounded near the deadband, so the light spin -> attack surge is gone. The 750 in `L_eq` is the fixed CLU->count offset, not the zero, so it is correct. G1-LEVEL covers light spin then attack, with no overshoot allowed. D-023 records the decision |
| N2 | RESOLVED | The predicate is now `speed_native <= 0 AND (direction_inhibit OR (crank stopped AND V3 stop target == 0))`. A held load at speed 0 now follows the legacy D3E ramp. G1-STAND is two-sided (within +-20 % of baseline for load held; <= baseline for released and reverse). D-019 is correctly marked SUPERSEDED, with the old text kept |
| N3 | RESOLVED | Tolerance split: +-5 % with a converged template; +-15 % and no step > 10 % with the prior |
| N4 | RESOLVED | 820/995 is chosen by V3's own engaged flag; only the zero comes from the shadow chain |
| N5 | RESOLVED (value open) | A revolution above the mismatch threshold drops to fallback within that revolution; G1-STYLE asserts it. The threshold value is set by the matrix |
| N6 | RESOLVED | The text is corrected: the erase window leaves an "absent" record -> defaults, never a corrupt configuration. The single free page is justified and the window is tested |
| D-020 conditions | RESOLVED | Conditions (a)-(c) recorded in D-020 |

Residuals (MINOR, close at the Milestone B gate):

- R-a: The standstill predicate now depends on one V3 internal (`V3 stop target == 0`). If V3 has a bug, the bound
  at standstill is the independent backstop (`T_STOP_HARD` + 300 ms, i.e. <= 1.8 s), not the hard zero. State this
  in section 2.1, and include "speed 0, crank stopped, V3 forced to max" in G1-BACKSTOP.
- R-b: Spell out "crank stopped" in the predicate (G53 true-stop OR native real_stop), as in the backstop text.
- R-c: The N5 mismatch threshold, `kappa_max` = 2.5 and the 300 CLU floor are candidates; their final values come from
  SIMULATION_REPORT.
- R-d (carried over, not a review defect): CONFIG_A survival across a BL820 update and the EN 15194 run-on
  reference are still [UNKNOWN] / [EXTERNAL_REFERENCE]. Both are owner/hardware items before the ride, as already
  listed in ARCHITECTURE_V3 section 14.

REVIEW 1 status: **CLOSED - PASS_WITH_ISSUES** at 0214a13. Active implementation (Milestone B) may start.
