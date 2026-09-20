# FINAL AUDIT — TORQUE DOMAIN SEPARATION / PIPELINE 2 (FW-151)

**Date:** 2026-09-20
**Scope:** the torque path from raw ADC to the rider-demand input, the persistence of every
threshold on that path, and the correctness of two claims made by previous audits.
**Out of scope, deliberately:** Pipeline 2 itself (`ap2_pas_state`, base/dynamic, profiles, AUTO,
limits, trajectory, PI/FOC) is not rebuilt. One change inside it is a revert, argued in §L.

---

## A. GIT IDENTITY

```text
REPO          glodnickim/eVistDrive  (motor-controller-firmware)
BRANCH        feature/assist-pipeline-v2
BASELINE      1d6c6ba3b1a3a494446135550d03f49817bfaf11   last state verified on the bike
BAD PORT      2091c2a9f67df1d26082a95aea788343d18f3974   ap2_torque_chain (reverted, deleted)
FIRST FIX     e4f2b8872e10128a54c228dae01cbd7a568b41dc   Pipeline 2 semantics restored
START_HEAD    3ccfb9d2f8f148c15e8cff5ec8b352d67c412315   state audited here
FINAL_HEAD    see the Task Execution Report / git log (this commit)
```

History was not rewritten and the branch was not reset.

---

## B. OLD ARCHITECTURE (at START_HEAD)

```text
RAW ADC (mV)
  -> auto zero (offset_correction)                                   torque_input.c
  -> delta above zero, clamped [0, TORQUE_SPAN_MAX_NATIVE]
  -> user GAIN (span_native / TORQUE_DEFAULT_SPAN_NATIVE)
  -> FW-150 MEASURED kg table  ------------------------> load_centikg  (0.01 kg)
                                                              |
                          +-----------------------------------+
                          |                    |                        |
                          v                    v                        v
              ap2_pas_state.c        ap2_rider_demand.c        telemetry / HMI /
              permission:            effort = map(ckg - 55,    calibration / recorders
              ckg >= engage_ckg      0, full_scale_ckg - 55,
                                     0, 1000)
```

One scale did two jobs. `load_centikg` was simultaneously:

- the number shown to the rider, defined by a MEASUREMENT of the sensor, and
- the number every control threshold was compared against.

Everything downstream of `load_centikg` was therefore a function of how well the sensor's
kilogram scale happened to be measured.

### What actually reaches the control path

An incidental finding of the mapping exercise, recorded because it changes how the filter map
below should be read: the entire native filter cascade in `torque_input.c` — the 35 ms FAST
filter, the FW-085 crank-angle RUN window, and the FW-112 rolling-rearm recovery automaton — has
**no consumer in the Pipeline 2 control path**. `ride_control.c` passes only the load value into
the pipeline. `torque_input_begin_rolling_rearm()` and `torque_input_cancel_rolling_rearm()` have
no caller anywhere in `src/`, so the recovery automaton can never open. `rider_input_t`'s
`torque_assist_filtered`, `torque_run_filtered` and `torque_assist_now_native` are written by
`main.c` and read by nobody.

This is **not changed** by this work — it is a separate, larger decision (either the estimator is
needed and must be reconnected, or it is dead weight and should go). It is recorded in §N.

---

## C. WHAT THE CENTIKG CONTROL DOMAIN ACTUALLY COST

The FW-150 reference-weight measurement replaced a disproved two-point kg curve with a measured
three-point one. Because control read that curve, correcting it retuned the bike. The same stored
number moved to a different pedal force:

| what is stored | pre-FW-150 sensor signal | post-FW-150 sensor signal | change |
|---|---|---|---|
| standing gate 70 (0.70 kg) | **17 mV** | **5 mV** | 3.4x more sensitive |
| rolling gate 30 (0.30 kg) | **8 mV** | **2 mV** | 4x more sensitive |
| effort deadband 15 (0.15 kg) | 3.65 mV | 1 mV | 3.7x more sensitive |
| effort full scale 6000 (60 kg) | 1139 mV | 3047 mV | 2.7x less sensitive |

The branch responded to this inconsistently, and that inconsistency is the defect:

| commit | deadband | standing | rolling | standing in mV | verdict |
|---|---|---|---|---|---|
| `1d6c6ba` baseline | 15 ckg | 70 ckg | 30 ckg | **17 mV** | verified on the bike |
| `e4f2b88` first fix | 55 ckg | 250 ckg | 110 ckg | **17 mV** | sensor-signal preserving |
| `3ccfb9d` START_HEAD | 55 ckg | 70 ckg | 30 ckg | **5 mV** | **behaviour changed** |

The deadband was migrated by preserving the SENSOR SIGNAL (15 → 55 ckg keeps 3.65 mV). The two
start thresholds were "restored to baseline values - verified working on bike" by preserving the
NUMBER. Those two migrations contradict each other, and the comment in `inc/assist_modes.h` that
asserted the second one had restored the verified behaviour was false.

Two consequences at START_HEAD, both measured in `tests/host/torque/torque_control_domain_host.c`:

1. **The standing gate sat inside the sensor's own rest noise.** `TQ_RECAL_STABLE_MV` is 10 mV —
   the spread a coast may show before the firmware distrusts it as a zero reference. A permission
   gate at 5 mV can be opened by noise alone.
2. **The rolling gate sat below the effort deadband** (2 mV vs 4 mV), so permission could be
   granted at a load that produces exactly zero rider effort — an incoherent state.

And the effort axis itself changed shape in both directions:

| native | baseline effort | START_HEAD effort |
|---|---|---|
| 100 | 66 / 1000 | 94 / 1000 |
| 500 | 419 / 1000 | 244 / 1000 |
| 1139 | 1000 / 1000 | 423 / 1000 |

---

## D. NEW DOMAIN ARCHITECTURE

```text
RAW ADC (mV)
    |
    v  auto zero (offset_correction)                          SENSOR
  corrected_native
    |
    v  delta above zero, clamped
  delta_native
    |
    v  user GAIN: canonical_native() = delta * GAIN_REFERENCE / span_native
  ============ CANONICAL INTERNAL TORQUE =============        the one internal unit
    |                                       |
    | FROZEN control characteristic         | MEASURED FW-150 kg table
    v                                       v
  CONTROL LOAD (CLU)                     load_centikg  (0.01 kg)
    |                                       |
    |                                       +--> HMI, CAN telemetry, calibration display,
    |                                            ride_episode / pas_trace kg fields, logs
    v
  ap2_pas_state.c       permission      thresholds in CLU
  ap2_rider_demand.c    effort 0..1000  deadband + full scale in CLU   <- ONE normalization
  ap2_estimators.c      aggression / terrain load
  ap2_profiles.c        profile / AUTO
  ap2_limits.c          the one limiter chain
  assist_pipeline.c     the ONE path to Iq
  fast_iq_slew.c        Iq trajectory (16 kHz)
  foc_current_loop.c    PI + vector saturation
```

The two characteristics are siblings reading the same canonical value, never a chain. They are
implemented as separate functions on purpose — sharing an implementation would invite a later
"unification" that recreates the coupling.

---

## E. THE CANONICAL INTERNAL TORQUE UNIT

**Definition.** The native ADC delta above the automatic zero that the FACTORY sensor would have
produced for this pedal force. Integer millivolt counts.

```text
canonical_native(delta) = delta                                  when calibration is DEFAULT
canonical_native(delta) = delta * TORQUE_GAIN_REFERENCE_NATIVE / span_native
                                                                 when a user gain is active
range: 0 .. TORQUE_SPAN_MAX_NATIVE (4200)
```

It satisfies the requirements asked of it: integer / fixed point, no float on the realtime path,
monotonic, zero-referenced, gain-aware, independent of the kg table and of the HMI, explicitly
bounded, and reached by exactly one conversion from the measurement.

**`span_native` was redefined in meaning, not in value.** It used to be documented as "the native
delta this sensor produces at 60.00 kg", which tied every calibrated rider's gain to the kg table.
It is now a pure gain against the frozen `TORQUE_GAIN_REFERENCE_NATIVE` (3047 — numerically the
FW-150 value, so no stored record changes meaning). This is why calibration persist v3 stays
valid; see §I.

---

## F. NORMALIZED EFFORT (the control domain)

**Unit: CLU, "control load units".** Not physical, never displayed. A two-segment piecewise
characteristic of the canonical internal torque, plus one linear projection onto the effort axis.

```text
CLU:        0 .. TORQUE_CTRL_BREAK_NATIVE  (146)  ->     0 ..  600   linear
            146 .. TORQUE_CTRL_HIGH_NATIVE (1580) ->   600 .. 8400   linear, extrapolates above
            ceiling TORQUE_CTRL_MAX_CLU (12000)

effort:     above     = max(0, median3(CLU) - AP2_EFFORT_DEADBAND_CTRL)
            effort    = above * 1000 / (full_scale_ctrl - AP2_EFFORT_DEADBAND_CTRL)
            deadband  = 15 CLU          full scale = 6000 CLU (default, rider-settable)
```

**The values are seeded from `1d6c6ba`** — the characteristic the bike was last verified on. That
is deliberate and is why this refactor does not change the ride: every threshold and the whole
effort axis keep their exact native trip points. Measured sweep (default calibration):

| native | CLU | effort ‰ | kg shown | standing gate | rolling gate |
|---:|---:|---:|---:|:---:|:---:|
| 0 | 0 | 0 | 0.00 | – | – |
| 2 | 8 | 0 | 0.30 | – | – |
| 4 | 16 | 0 | 0.60 | – | – |
| 5 | 21 | 1 | 0.75 | – | – |
| 8 | 33 | 3 | 1.20 | – | **Y** |
| 15 | 62 | 7 | 2.25 | – | Y |
| **17** | **70** | 9 | 2.55 | **Y** | Y |
| 20 | 82 | 11 | 3.00 | Y | Y |
| 50 | 205 | 31 | 4.18 | Y | Y |
| 100 | 411 | 66 | 6.15 | Y | Y |
| 146 | 600 | 97 | 7.96 | Y | Y |
| 185 | 812 | 133 | 9.50 | Y | Y |
| 300 | 1438 | 237 | 11.53 | Y | Y |
| 500 | 2526 | 419 | 15.06 | Y | Y |
| 780 | 4049 | 674 | 20.00 | Y | Y |
| 1139 | 6001 | 1000 | 26.34 | Y | Y |
| 3047 | 12000 | 1000 | 60.01 | Y | Y |

The trip points (standing 17 mV, rolling 8 mV, deadband 4 mV) and the effort values at native 100
/ 500 / 1139 (66 / 419 / 1000 ‰) are exactly the bike-verified baseline's.

**Direct native → effort was chosen over native → kg → effort**, as the task asked to consider.
The intermediate CLU value is not a second normalization: it is the single sensor→control
characteristic, and the linear projection onto 0..1000 is its second half. Thresholds are compared
in CLU, before and independently of the effort projection, so there is exactly one place where the
sensor becomes control.

### Why the kg table is NOT reused for control, even though it is the better measurement

FW-150 is a better measurement of KILOGRAMS. It is not evidence about how assist should respond to
a pedal force — that is a tuning decision with its own ride evidence. Re-measuring a display scale
must not retune the bike; deliberately retuning the bike means editing the frozen characteristic,
which invalidates stored thresholds and therefore requires a bank version bump. Both statements
are enforced by tests, not by convention.

---

## G. PHYSICAL kg REPRESENTATION (human domain)

Unchanged FW-150 three-point measured table, `TORQUE_CURVE_P1..P3`: 3.00 kg at delta 20, 9.50 kg
at 185, 20.00 kg at 780, extrapolating on the last slope. Strongly progressive (6.7 → 25.4 → 56.7
mV/kg). Used for, and only for: HMI, CAN telemetry (`0x6025`), calibration display and the
reference-weight capture, diagnostics, logs, the kg fields of the recorders, the rider-power
estimate, and the configuration boundary where a rider enters kilograms.

It may be re-measured freely. Doing so changes displayed kilograms and nothing else.

---

## H. PERSISTENT MIGRATION — BANK v1..v10

The blob is at its hard 255 B ceiling (13 B header + 5 x 48 B record + 2 B CRC), so **v10 grows
nothing**. Header, record and length are byte for byte the v8 layout; what v10 states is that
bytes `[19..20]` (u16) and `[35]` (u8) carry CLU rather than 0.01/0.1 kg. A reader cannot tell the
two apart from the bytes, which is exactly what a version byte is for — the same reason v4 and v9
exist.

| stored version | what the bytes held | migration to v10 | goes through kg? |
|---|---|---|---|
| v1..v6 | calibrated sensor delta in **mV** | read on the frozen control characteristic: native → CLU | **no** |
| v7, v8, v9 | 0.01 / 0.1 **kg** on the PRE-FW-150 curve | **identity** — that curve is the one the control domain is frozen to, so the stored number already IS the CLU value | **no** |
| v10 | **CLU** | taken as stored | n/a |

The v7..v9 identity is not a coincidence. Seeding the control domain from the pre-FW-150
characteristic is what makes every rider's stored threshold keep the sensor trip point they
configured, with no conversion and no drift.

### Worked examples (measured, `torque_threshold_migration_host.c`)

| old stored | old interpretation | old native | new v10 CLU | kg displayed now | control error |
|---:|---|---:|---:|---:|---|
| 70 (v9) | 0.70 kg, pre-FW-150 | 17 mV | 70 | 2.55 kg | **none** — trips at 17 mV |
| 30 (v9, 3 steps) | 0.30 kg, pre-FW-150 | 8 mV | 30 | 1.20 kg | **none** — trips at 8 mV |
| 250 (v9) | 2.50 kg, pre-FW-150 | 61 mV | 250 | 4.66 kg | none |
| 18 mV (v6) | sensor delta | 18 mV | 70 | 2.55 kg | 1 mV, wire quantization |
| 8 mV (v6 derived) | minimum − reduction | 8 mV | 30 | 1.20 kg | 1 mV, wire quantization |
| 1500 (v9) | 15.00 kg, pre-FW-150 | 321 mV | 1500 | 12.0 kg | none |

The only error is the wire format's own quantization: the stored field has a 10 CLU step and the
curve runs at 4.1 CLU/mV there, so half a step is ~1.2 mV. That rounding existed in every version
of this format. What the control domain removes is the 3.4x shift — a different order of magnitude.

**The historical pre-FW-150 kg conversion is NOT retained as a migration helper**, because the
migration does not need it: v7..v9 is the identity and v1..v6 goes native → CLU directly. There is
therefore no `pre_fw150_centikg_to_native()` in the tree to be accidentally reached from runtime.

**The global tuning blob** carries the effort full scale. Its numbers are unchanged (6000 CLU is
the point 6000 centikg used to be on the frozen curve), so no stored tuning blob needs migrating
and no tuning version bump is required.

**Known ambiguity, accepted and recorded.** A v9 blob written by the unreleased `e4f2b88` or
`3ccfb9d` builds holds kg on the NEW table, and is migrated as if it were pre-FW-150. Neither
build was released and `3ccfb9d` is the build this audit finds defective, so treating v9 as
pre-FW-150 is correct for every bank that exists in the field. A rider who saved a bank from one
of those two developer builds should re-check their start thresholds.

---

## I. CALIBRATION PERSIST — VERSIONING

| version | meaning | on restore |
|---|---|---|
| v1 | pre-FW-129: `span = delta_ref * 6000 / reference` | **reject**, set the legacy-dropped flag |
| v2 | pre-FW-150: gain form, but against the disproved curve | **reject**, set the legacy-dropped flag |
| v3 | gain against `TORQUE_GAIN_REFERENCE_NATIVE` | **accept** |

**v3 is NOT bumped, and the reason is load-bearing.** `span_native` has always been applied as a
gain ratio `span_native / 3047`, and `TORQUE_GAIN_REFERENCE_NATIVE` freezes that 3047 at the value
FW-150 gave it. The arithmetic applied to a stored v3 record is byte-for-byte what it was, so the
record's numeric meaning is unchanged and a bump would invalidate valid calibrations for nothing.

What DID change is the documented meaning of the constant: it is a frozen gain reference, no
longer "the delta at 60.00 kg". Had it stayed defined in kilograms, a future kg re-measurement
would have silently rescaled the gain of every calibrated sensor in the field. **If the gain
reference is ever changed numerically, v3 must be bumped** — that is the condition to watch, and
it is stated at the constant.

Test T1 was also strengthened: the v2 rejection case now uses a span a real v2 writer could have
produced (1139, the pre-FW-150 default), which sits inside the accepted range. The previous case
used the current default span, so it would have passed even with the version check removed.

---

## J. SPAN OVERFLOW — P0, CLOSED

`torque_input_cal_tick()` computed `span` in 32 bits and then validated the **narrowed** value:

```c
uint32_t span = (delta_reference * TORQUE_DEFAULT_SPAN_NATIVE + ...) / reference_delta;
if (!span_in_range((uint16_t)span)) {   /* BUG: cast before range check */
```

Measured on the real calibration state machine (not on the helper):

```text
reference weight            5.00 kg
default sensor delta        71 native
measured delta              1546 native      (a miswired / wrong-scale sensor)
true span                   1546 * 3047 / 71 = 66347
(uint16_t)66347             = 811
accepted range              [800, 4200]      -> 811 IS INSIDE
```

So the pre-fix firmware **accepted 811 as a calibration** and let it be committed — a gain wrong
by a factor of ~80, silently. Fix: compare in 32 bits before narrowing.

```c
if (span < TORQUE_SPAN_MIN_NATIVE || span > TORQUE_SPAN_MAX_NATIVE) {
        cal_fail(TORQUE_CAL_ERR_SPAN_RANGE);
        return;
}
cal_preview_span = (uint16_t)span;   /* now provably lossless */
```

| | old behaviour | new behaviour |
|---|---|---|
| state | `PREVIEW` | `FAILED` |
| error | `TORQUE_CAL_ERR_NONE` | `TORQUE_CAL_ERR_SPAN_RANGE` |
| preview span | 811 | 0 |
| `cal_commit()` | **succeeds** | refused |
| calibration source | USER (garbage gain) | stays DEFAULT |

Test: `torque_cal_migration_host.c` TC10, driving `cal_start` → zero capture → reference →
load capture → span computation. Verified to FAIL at START_HEAD (4 checks) and pass after.

---

## K. PIPELINE DIFF vs BASELINE `1d6c6ba`

Every functional difference in the assist path, after this work:

| # | difference | status |
|---|---|---|
| 1 | control reads CLU instead of `load_centikg` | **new** — this work. Same native trip points, so no behaviour change. |
| 2 | start / rolling thresholds stored in CLU, bank v10 | **new** — this work. |
| 3 | effort deadband and full scale in CLU | **new** — this work. Same 3.65 mV / 1139 mV as baseline. |
| 4 | stuck-high fault gate in CLU (`TQ_STUCK_CTRL`) | **new** — this work. Same sensor level. |
| 5 | span range check before the narrowing cast | **new** — P0 fix, §J. |
| 6 | `iq_ceiling` tracking the reference on `block_positive` | **REVERTED** to baseline, §L. |
| 7 | FW-150 kg table (display) replaces the two-point curve | kept — a measurement. Now display-only. |
| 8 | calibration persist v2 rejected | kept. |
| 9 | `ap2_torque_chain` and its 6-state gate | **absent** — deleted in `e4f2b88`, stays deleted. |

`ap2_pas_state`, `ap2_rider_demand`'s model, `ap2_estimators`, `ap2_profiles`, `ap2_limits`,
`fast_iq_slew`, `foc_current_loop` and every profile number are otherwise untouched. Profiles were
not retuned.

---

## L. S17 / CEILING — **REVERT**

**Decision: REVERT. The change was a regression artifact, not an independent fix.**

The change made `ctx.ceiling` slew to zero whenever `pas.block_positive` was set, justified as
"otherwise it can fall below the reference during the release ramp".

**That justification is impossible.** `iq_ceiling` is defined in `inc/ap2_limits.h` as the limiter
chain applied to a request of FULL SCALE — the largest current the protections permit right now,
*independent of what the rider asked for*. During `block_positive` the reference is forced to zero
(`lim.final_iq = 0`), so the ceiling is necessarily at or above it. There was no case to fix.

What the change did do:

1. **Conflated two independent concepts** — a protection limit became a function of lifecycle
   state, giving one value two meanings.
2. **Took the release away from its owner.** The ceiling clamp in `src/fast_iq_slew.c` zeroes the
   trajectory rate when it binds (`if (fis.rate > 0) fis.rate = 0;`), so a descending ceiling
   overrides the safety release mid-ramp instead of letting `FIS_MODE_SAFETY` run its 200 ms.
3. **Added a 400 ms assist clamp after every block.** Once the ceiling has fallen to ~0 it must
   climb back over `AP2_CEILING_RISE_MS` = 400 ms, clamping legitimate assist after every level-0
   toggle, direction confirm, brake release or fault clear. The bike-verified baseline has no such
   delay.
4. **Reintroduced the command churn `dc23faf` had to defend against.** `fis_command_equal()`
   compares `iq_ceiling`, so a ceiling changing every tick of a release is precisely the pattern
   that made release-rate re-derivation Zeno-like (a 190 ms stop taking 831 ms). `dc23faf` fixed
   that by deriving the rate at the release edge; this change re-created its trigger.

Reaching zero on a block is already handled, twice: `lim.final_iq = 0` as an absolute final gate,
and `trajectory()` returning `FIS_MODE_SAFETY` over `AP2_SAFETY_RELEASE_MS`. Scenario S17 in
`ap2_pipeline_scenarios_host.c` ("a release takes release_ms whatever else is moving in the
command") passes with the revert in place.

To reintroduce it, a measurement showing `iq_ceiling` binding *below* the reference is required.

---

## M. TESTS

### New and strengthened

| test | what it pins | fails at START_HEAD? |
|---|---|---|
| `torque_cal_migration_host.c` TC9 | a realistic v2 span (1139) is rejected on its VERSION, not incidentally on range | no (new coverage) |
| `torque_cal_migration_host.c` TC10 | span overflow, through the real calibration state machine | **yes, 4 checks** |
| `torque_control_domain_host.c` D1/D2 | both projections monotonic over 0..4200, explicit ceiling | n/a (new API) |
| — D3 | CLU round trips through native | n/a |
| — D4 | the control domain is not derived from the kg table (shape divergence + control constants are control-domain) | n/a |
| — D5 | the shipped gates keep their bike-verified trip points (17 / 8 / 4 mV) | **yes** — START_HEAD gives 5 / 2 / 4 mV |
| — D6 | the standing gate is outside sensor rest noise; the rolling gate is at or above the deadband | **yes** — both violated at START_HEAD |
| — D7 | the user gain moves both projections together and is exactly reversible | n/a |
| — D8 | effort monotonic and bounded over the sweep, endpoints exact, one normalization | n/a |
| — D9 | **base/dynamic is alive**: sustained base, separate fast excess, base outlives the dead spot | see below |
| `torque_threshold_migration_host.c` M1 | v10 serialize → deserialize → serialize is byte identical | n/a |
| — M2 | v7/v8/v9 migrate to the same sensor trip point | n/a |
| — M3 | arbitrary rider values migrate, not just the defaults | n/a |
| — M4 | v1..v6 mV thresholds migrate straight onto the control characteristic | n/a |
| — M5 | v6 vs v9 equivalence for the same requested pressure | n/a |
| — M6 | migration is one-shot and survives a restart; written back as v10 | n/a |

**D9 exists because of a gap this work found.** A mutation that zeroes the dynamic term passed
**every** host suite in the repository. base/dynamic is the mechanism Assist Pipeline V2 exists
for, and nothing was holding it. D9 now does.

### Executed

| gate | result |
|---|---|
| Real-module host suites (59 suites) | **PASS** |
| Whole-pipeline deterministic regression | **PASS** |
| Assist ripple attenuation (14 scenarios) | **PASS** |
| Ripple analyzer can reject unusable evidence | **PASS** |
| Closed-loop SIL + deterministic fuzz (1000) | **PASS** |
| Real FOC/PMSM/Hall electrical SIL | **PASS** |
| Level-4 virtual rider + bike + battery/SOC | **PASS** (9 fixed + 25 fuzz) |
| Every replay behaviour criterion can reject | **PASS** |
| CANable raw-log decode → canonical → replay | **PASS** |
| ARM target build NORMAL | **PASS** — 116368 B, RAM 80.76 % |
| ARM target build DIAGNOSTIC | **PASS** — 144504 B, RAM 61.95 % |
| Recorded-ride replay behaviour | **FAIL 2/6 — pre-existing, see below** |
| ASan / UBSan | **NOT_RUN** — no sanitizer runtime linkable with this gcc; the gate itself reports SKIP, not pass |

### Recorded-ride replay: measured before and after

This gate was **already red at START_HEAD**. It is not made red by this work, and it is not
claimed as a pass.

| case | START_HEAD | FW-151 | limit | change |
|---|---:|---:|---:|---|
| w1-f01-stable-cad49 | 0.592 PASS | **0.618 FAIL** | 0.6 | regressed, marginal |
| w1-f03-stable-cad85 | PASS | PASS | | — |
| w1-f05-load-rise | PASS | PASS | | — |
| w1-f06-load-fall | PASS | PASS | | — |
| w1-f07-pause-43s | 0.890 FAIL | **0.785 FAIL** | 0.6 | improved, still fails |
| w1-f08-pause-10s | 0.697 FAIL | **0.588 PASS** | 0.6 | **fixed** |
| **accepted** | **4/6** | **4/6** | | |

The direction is consistent with restoring the baseline effort curve: less effort at low load
(pause/restart cases f07, f08 improve) and more at high load (steady cad49 worsens). No criterion
was weakened to accommodate this.

### Mutation tests (all reverted; none committed)

| mutation | expected | observed |
|---|---|---|
| **A** restore the narrowing cast before span validation | overflow test FAILS | **FAILS**, 5 checks |
| **B** disable the v1..v6 native → CLU migration | migration test FAILS | **FAILS**, M4 (17 mV → 5 mV) |
| **C** feed the measured kg table back in as the control domain | domain invariance FAILS | **FAILS**, D1 + D3 |
| **D** zero the dynamic term | pipeline regression FAILS | **FAILS**, D9 — and passed everything *before* D9 was added |

---

## N. OPEN BIKE RISKS

1. **Nothing here is validated on the physical bike.** The claim is that this refactor reproduces
   the trip points and effort values of `1d6c6ba` exactly, proven in host tests. `1d6c6ba` is the
   state the owner reports as working. That is an argument, not a ride.
2. **START_HEAD is more sensitive than the bike was ever ridden with.** If anyone flashed
   `3ccfb9d`, its standing gate was 5 mV, inside sensor rest noise. This commit moves it back to
   17 mV, so assist will feel like it needs MORE pressure than `3ccfb9d` did — that is the
   intended correction, not a new regression.
3. **w1-f01-stable-cad49 ripple attenuation 0.618 vs a 0.6 limit.** Marginal, in a gate that was
   already failing two other cases. Needs a decision on the bike: either the criterion or the
   recording is wrong, or steady low-cadence ripple genuinely needs attention.
4. **The kg table above 20 kg is extrapolated.** No reference weight above 20 kg exists. Displayed
   kilograms above that are the weakest number in the system. It is now display-only, so it cannot
   affect control — but calibration reference weights are capped at 30.00 kg and read on it.
5. **The FAST/RUN/recovery cascade is dead in the control path** (§B). `begin_rolling_rearm()` has
   no caller; the crank-angle RUN window feeds only telemetry. Either the estimator is needed and
   must be reconnected, or it is dead weight — both are real work with ride consequences, and
   neither belongs in this commit. **Recommend a dedicated card.**
6. **`TQ_RECAL_BAND_MV` (30) and `TQ_REACQUIRE_MAX_MV` (40) are unchanged and now known to be
   wider in real force than intended.** On the measured curve they are ~3.4 kg and ~6.0 kg, not the
   ~1.1 kg and ~1.5 kg their comments once claimed. They are auto-zero acceptance windows: a
   static pedal load below ~3.4 kg can still be absorbed as "zero". Not changed here — changing an
   auto-zero window without bike data is exactly how a zero starts walking. **Needs measurement.**
7. **Extended Boost is configured but not wired.** `extended_boost.trigger_load_centikg` is stored,
   round-tripped and validated, and no Pipeline 2 block reads it. Left in kilograms deliberately
   since it is not a control path; if it is ever connected, it must move to CLU first.
8. **Two stale JS guards, broken before this work.** `tests/fw091_limit_source.js` reads
   `src/assist_limits.c`, a module Pipeline 2 replaced (crashes on ENOENT).
   `tests/fw089_start_pressure_gate.js` checks 5 and 6 match `main.c` shapes that no longer exist.
   Neither is in `tools/verify_all.py`, so neither was noticed. fw089's domain-dependent checks
   (1–4, 7) were converted to CLU here and pass; its null-dereference crash was guarded so the
   later checks can run at all. **The staleness itself needs its own card.**
9. **A bank saved from `e4f2b88` or `3ccfb9d`** is migrated as pre-FW-150 kg (§H). Neither build
   was released; a rider who used one should re-check their start thresholds.

---

## VERDICT

```text
centikg in realtime control     NO
centikg in HMI / telemetry      YES
centikg in calibration UI       YES
ap2_torque_chain present        NO
ap2_rider_demand is the owner   YES
base preserved                  YES
dynamic preserved               YES  (and now protected by D9)
profiles retuned                NO
one sensor -> control conversion YES
second hidden normalization     NO
kg table change moves the bike  NO   (enforced by D4/D5/D6)
span overflow                   CLOSED
S17                             REVERTED
```

Executor does not approve their own work: **READY_FOR_REVIEW**.
