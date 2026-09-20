# Assist Pipeline V2

The whole pedal-assist path, from the torque sensor to one `final_iq_request`.

This replaces the previous assist path entirely. It is not a set of fixes to it: the modes, the
ride latch, the current floor, the hold grace, Extended Boost, the smooth start, the rearm
machinery and the per-caller limiters are gone, and what is here was designed as one chain.

---

## 1. The whole path, in one picture

```text
     TORQUE SENSOR        PAS A/B + DIRECTION        CADENCE        SPEED      MOTOR STATE
          |                        |                    |             |             |
          +------------------------+--------------------+-------------+-------------+
                                   |
                                   v
INPUT VALIDATION                      ap2_torque_chain.c
                                       |
                                       v
                         TORQUE ZERO / NORMALIZATION           ap2_torque_chain.c
                                       |
                                       v
                         RIDER DEMAND                          ap2_torque_chain.c
                                       |
                                       v
                         PEDAL CYCLE: BASE + DYNAMIC           ap2_torque_chain.c
                                   |
              +--------------------+--------------------+
              |                                         |
              v                                         v
   RIDER AGGRESSION (fast)                    LOAD / TERRAIN (slow)
   ap2_estimators.c                           ap2_estimators.c
              |                                         |
              +--------------------+--------------------+
                                   |
     PAS / DIRECTION LIFECYCLE     |          ap2_pas_state.c
     (STOPPED / STARTING / FORWARD / STOPPING / REVERSE / INVALID)
                                   |
                                   v
                        PROFILE / AUTO                        ap2_profiles.c
                        ASSIST CHARACTERISTIC                 ap2_profiles.c
                                   |
                                   v
                        BASE + DYNAMIC COMPONENT              assist_pipeline.c
                                   |
                                   v
                        START / ATTACK / RELEASE / STOP       assist_pipeline.c
                                   |
                                   v
                        LIMITS                                ap2_limits.c
                        power -> battery -> phase -> voltage -> thermal -> speed
                                   |
                                   v
                        final_iq_request
                                   |
                                   v
                   ONE 16 kHz final Iq trajectory owner       fast_iq_slew.c
                                   |
                                   v
                        FOC / PWM / MOTOR
```

**One path.** `assist_pipeline_update()` is the only place in this firmware where pedal assist
becomes a current request. No mode, boost, floor, latch or adaptive feature writes Iq beside it
or after it. Walk Assist and the position calibration are separate *owners of the same single
mailbox*, selected in `ride_control.c` before the pipeline is called — they are not a second
assist path, and they pass the same limiter chain.

---

## 2. Execution rates

| Domain | Rate | What runs there |
|---|---|---|
| Foreground control tick | 4 kHz | The whole pipeline: every block in the diagram above |
| FOC ISR | 16 kHz | `fast_iq_slew_tick()` — the single owner of the Iq reference |

Every block takes `elapsed_ticks`: the number of 4 kHz periods this invocation represents. The
foreground loop can coalesce periods, so a filter written against the *call count* silently
changes its time constant whenever the loop is late. Nothing in this pipeline counts calls.

---

## 3. Units

| Quantity | Unit | Where it comes from |
|---|---|---|
| pedal force | 0.01 kgf (centikg) | `torque_input.c`, after zero and calibration |
| effort / demand / base / dynamic / response | permille, 0..1000 | normalized against the ride-feel full scale |
| aggression, load state, auto factor | permille, 0..1000 | confidences, no physical dimension |
| cadence | rpm | `cadence_filter.c` (the one control cadence) |
| speed | 0.01 km/h | wheel sensor |
| Iq | `PH_CURRENT_MAX` domain (700 = full scale) | — |
| power | W | measured duty conversion, see §8 |
| times | ms for a FULL-SCALE move | so a number means the same thing to the rider everywhere |

The rider-effort full scale is `tuning_config_assist_torque_full_scale_centikg()`. It is a
**ride-feel** setting: it decides how much pedal force counts as "everything you have". It is
not the sensor calibration, the sensor maximum or the ADC maximum, and changing it moves no
kilogram reading anywhere.

---

## 4. Two kinds of dynamics, and only two

`inc/ap2_math.h` provides exactly two primitives, and every block uses one of them:

- **`ap2_lpf_step()` — estimator lag.** A first-order response with an explicit time constant,
  used where the job is *estimate a slowly true quantity from a noisy or pulsating one*. Each
  use site must be able to say what it estimates.
- **`ap2_slew_step()` — explicit dynamics.** A constant-rate move toward a target, parameterised
  by how long a full-scale move takes. Used where the job is *decide how fast the machine is
  allowed to change*, which is a rider-facing behaviour, not a measurement.

There is **one filter on the measurement path** (§5). Pedal ripple is not filtered away; it is
*modelled*. If a future change needs a third mechanism in the signal path, that is a signal that
the block it sits in is modelled wrongly.

---

## 5. Input validation, normalization, rider demand

`ap2_torque_chain.c`, 4 kHz.

| In | Out |
|---|---|
| `load_centikg`, `torque_valid`, `cadence_rpm`, `pedaling`, `full_scale_centikg`, `base_hold_ms` | `effort`, `demand`, `base`, `dynamic`, `stroke_peak`, `stroke_period_ms` |

1. **Validation.** A faulted or in-calibration torque sensor produces no effort at all, and
   every state that could outlive the fault is dropped with it. There is deliberately no "last
   good value": a stuck reading is what a torque fault looks like, and holding it would keep
   the motor pulling.
2. **Spike rejection.** Median of the last three samples (0.75 ms). This removes a single
   corrupted ADC sample without adding the lag a filter long enough to do the same job would.
3. **Deadband and normalization.** `AP2_EFFORT_DEADBAND_CENTIKG` (0.15 kgf) in *physical force*,
   so it means the same thing after a sensor recalibration; then a linear projection onto
   0..1000 against the ride-feel full scale.
4. **The one noise filter.** `AP2_EFFORT_LPF_MS` = 20 ms. Its job is sensor noise. It is far too
   short to touch pedal ripple — a stroke at 90 rpm lasts 333 ms — which is the point.

### The pedal-cycle model

Pedal torque pulsates, one leg push per half revolution with a dead spot between:

```text
          /\              /\
      ___/  \____________/  \____
```

Copying that into the motor gives a motor that surges and drops twice per revolution. Low-passing
it away buys smoothness by destroying responsiveness. So the signal is **modelled** instead: one
stroke carries two separate facts.

| Signal | Meaning | Rise | Fall |
|---|---|---|---|
| `base` | the sustained level the rider is working at | `AP2_BASE_RISE_MS` = 120 ms | `max(profile base_hold_ms, 1.5 stroke periods)` |
| `dynamic` | `max(0, demand - base)`: pushed *harder* than that | `AP2_DYNAMIC_RISE_MS` = 25 ms | `AP2_DYNAMIC_FALL_MS` = 150 ms |

The base decay floor is tied to the **stroke period**, so at low cadence — where the dead spot is
longest and a fixed constant fails worst — the base outlives the gap by construction rather than
by a tuned number.

At engagement the base is **seeded** to what the rider is pressing right now, so the first stroke
of a ride is answered at its real magnitude instead of being walked up from zero. It is a seed,
not a floor: it cannot produce assist the rider is not asking for.

---

## 6. Two estimators, deliberately different speeds

`ap2_estimators.c`, 4 kHz.

### `rider_aggression` — fast

"How sharply is this rider riding right now?" Answers within a stroke or two, forgets within a
couple of seconds. Three independent pieces of evidence, taken as a **maximum** rather than a
mean — each alone is a complete argument, and averaging lets two quiet signals silence the one
that is speaking:

- rate of rise of `demand`, over a 50 ms window, in permille per 100 ms
- stroke peak above the sustained base, relative to the base
- cadence being wound up, over a 250 ms window, in rpm per second

Envelope: 120 ms rise, 1500 ms fall.

**What it is allowed to change:** attack, release and the dynamic assist term. It is **never** a
global multiplier on the request — a rider who pushes sharply once should get a sharper *answer*,
not a permanently stronger motor.

### `load_state` — slow

"Is the bike working hard — a climb, a headwind, soft ground?" The signature of a climb is not
high torque; it is high *sustained* torque that is not turning into cadence and not turning into
speed, held long enough that a single hard stroke cannot produce it.

```text
load = effort_evidence x (0.5 + cadence_evidence/2)      cadence SCALES effort, never adds to it
     + 250 if speed has been flat for 1.5 s while effort is high
```

Envelope: 2500 ms rise, 6000 ms fall — deliberately much slower than aggression.

**What it is allowed to change:** the sustained base term, and in AUTO the power envelope.

Mixing these two into one "effort" number is what makes an assist impossible to reason about: a
sprint on the flat and a slow climb ask for opposite dynamics from similar torque readings.

---

## 7. The PAS / direction lifecycle

`ap2_pas_state.c`, 4 kHz. **One** owner of "is pedal assist allowed to flow right now".

```text
     STOPPED ---forward crank--->  STARTING_FORWARD
        ^                                 |
        |                        steps + pedal load met
        |                                 v
        +---grace expired / true stop--  FORWARD
        |                                 |
        |                        forward pedalling lost
        |                                 v
        +------------------------------ STOPPING --resumed--> FORWARD

     any state --reverse crank step--> REVERSE   (assist blocked immediately)
     any state --illegal transition--> INVALID   (assist blocked immediately)
     REVERSE / INVALID --inhibit cleared--> STOPPED
```

It is **not** a sensor owner. Direction plausibility stays in `pas_direction.c`, electrical PAS
sampling in `pas_sampler.c`, conditioned cadence in `cadence_filter.c`.

- **Terminal causes are evaluated first**, independently of the current state: direction inhibit,
  safety cut, assist level 0, lost sensor. None of them is a consequence of what the automaton
  previously decided, so none can be missed by getting a transition wrong. This is the
  default-deny backstop.
- **Engagement** requires forward pedalling, the anti-jiggle step count, and pedal load over the
  threshold. The step count is eased by one while the bike is already rolling: turning the cranks
  on a stationary bike proves nothing, on a moving one the rider has already shown intent.
- **STOPPING** is a grace window of `AP2_PAS_STOP_GRACE_MS` = 400 ms, for exactly one rider-visible
  case: easing off for one leg and pushing again. Resuming inside it re-engages without the cold
  start gate — the ride never ended — but the load threshold still applies, so a freewheeling
  crank cannot re-engage on its own.
- **REVERSE is absolute.** `block_positive` is set and the pipeline contract is that nothing
  downstream may present a positive request on that tick: not a held base, not a decaying
  estimator, not a running slew. The current already in the motor is retired by the
  firmware-owned safety release — a *current* trajectory, bounded, never re-raised.

---

## 8. Profiles, the characteristic, and AUTO

`ap2_profiles.c`, 4 kHz.

**A mode is not a percentage.** Each profile defines a whole behaviour:

| Profile | gain | curve | base share | dynamic gain | attack | release | start | max power | base hold | aggr | load |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ECO | 80 | SOFT | 90 | 60 | 500 | 600 | 450 | 250 W | 300 | 20 | 40 |
| TRAIL | 130 | LINEAR | 100 | 100 | 350 | 450 | 350 | 450 W | 350 | 40 | 60 |
| SPORT | 190 | LINEAR | 105 | 140 | 240 | 380 | 300 | 650 W | 400 | 60 | 70 |
| SPORT+ | 250 | EAGER | 115 | 200 | 140 | 320 | 250 | 900 W | 500 | 100 | 90 |
| AUTO | ECO ← → SPORT, continuously |
| AUTO SPORT+ | TRAIL ← → SPORT+, continuously |

Times are milliseconds for a full-scale move.

### The assist characteristic

Rider effort → assist response, before any gain. Three shapes, each a five-point piecewise-linear
curve over 0..1000, so it costs no `pow()` and can be read off a table by whoever is tuning:

| curve | 0 | 250 | 500 | 750 | 1000 |
|---|---|---|---|---|---|
| SOFT | 0 | 120 | 300 | 580 | 1000 |
| LINEAR | 0 | 250 | 500 | 750 | 1000 |
| EAGER | 0 | 340 | 620 | 840 | 1000 |

SOFT keeps output below input at low effort — that is what makes ECO feel like a bicycle. EAGER
leads the rider through the middle of the range — the motor is already there when you lean on it.

### SPORT+ is not "instantly maximum Iq"

What makes it different from SPORT: an EAGER characteristic, a much larger **dynamic** term (it
answers a harder *push*, not just pulls harder overall), the shortest attack, the largest power
envelope, and the longest base hold — so under a big load it **sustains** instead of surging and
sagging. The attack is still an explicit, bounded ramp, which is what keeps it smooth.

### AUTO is not a second pipeline

AUTO and AUTO SPORT+ run the exact same blocks as every other profile. They move the parameters
of that one pipeline continuously between a **calm** endpoint and a **strong** endpoint, both of
which are ordinary profiles — so AUTO can never reach a behaviour a fixed profile could not.

The decision takes the maximum of three independent reasons, for the same reason the aggression
estimator does:

```text
want = max(rider demand, rider aggression, terrain load)
     + 200 if a real load is present at a cadence below 45 rpm
```

That last term is the one situation none of the three states loudly enough on its own, and it is
precisely where a calm profile feels like it gave up.

The result moves with an asymmetric lag — 800 ms to give, 4000 ms to take back — so the character
never flickers mid-stroke. The characteristic switches at the midpoint rather than being
interpolated into a curve neither endpoint defines; gain and the dynamic terms already move
continuously, so the handover is not a step in behaviour.

---

## 9. Composing the request

`assist_pipeline.c`, 4 kHz.

```text
base_shaped    = characteristic(base)
assist_base    = base_shaped x gain x base_share   x (1 + load       x load_influence)
assist_dynamic = dynamic      x gain x dynamic_gain x (1 + aggression x aggression_influence)

assist_response = clamp(assist_base + assist_dynamic, 0, 1000)
iq_request      = level_iq_limit x assist_response / 1000
```

Only the **sustained** term goes through the characteristic. The dynamic term is already an
excess above that sustained level; putting it through the same curve again would apply the
profile shape twice to the same pedal force.

**ASSIST is a torque-domain characteristic**, so it converts to current directly and is finite at
a standstill — where a power-domain request would divide by a duty that has gone to zero. POWER
is a separate ceiling, applied in the limiter chain.

### Three things that are not the same

| | What it is | Set by |
|---|---|---|
| **ASSIST** | the characteristic: rider effort → motor demand | profile + gain trim |
| **POWER** | the ceiling: how much the profile may spend | `max_power_w` |
| **ACCELERATION** | the dynamics: how fast the demand may move | attack / release |

They are never merged into one percentage.

---

## 10. Start, attack, release, stop

### START

There is **no delay** before assist: the request exists from the tick the lifecycle engages. What
the start segment changes is the *rate* for the first `start_ms`, so the first appearance of
torque is gentle and then hands over to the normal attack. A resumed ride gets a third of it —
the rider is already moving and has already had a gentle first appearance on this ride.

### ATTACK / RELEASE

One dynamics definition, realized by the single 16 kHz owner. The 4 kHz producer computes the Q8
per-tick step that walks a full-scale move across the configured time, so `attack_ms` means the
same thing in the code as it does to the rider.

Rider aggression shortens both, bounded to half, so an aggressive style can never turn an explicit
ramp into a step.

### STOP

```text
forward pedalling lost
        v
request = 0 in the SAME tick          (no hold, no floor, no estimator may carry it)
        v
FIS_MODE_RELEASE, release_ms          (the CURRENT trajectory, bounded)
        v
zero-current state, Quiet Zero armed
        v
PI lifecycle / neutral PWM lifecycle  (unchanged, main.c)
```

Time to zero **equals `release_ms`**, whatever the current was releasing from: the 16 kHz owner
derives the release rate once, at the **edge into the release**, from its live accumulator.

Re-deriving it later is the failure mode, and it has bitten twice from two directions. Each
derivation is individually correct; the *sequence* of them is Zeno, and the reference approaches
zero instead of arriving at it. Both halves of the rule are needed:

> **1. The release time is latched by the producer when the release begins.** Aggression shortens
> the release, and aggression decays *during* the release — so without the latch the published
> command changed on almost every tick. Measured before the fix: 1331 ms to zero against a 380 ms
> setting. After: 386 ms. It is also the right behaviour on its own — how fast the motor lets go
> is decided when the rider stops, not renegotiated while it is happening.
>
> **2. The consumer re-derives only on the release's own edge, not on any command change.** The
> command also carries the rate-limited protection ceiling, which moves every tick by design, so
> "the command changed" is not the same question as "this is a new release". Measured before the
> fix, with the ceiling in the command: **831 ms** to zero against a 190 ms setting, in the
> electrical SIL. After: **187 ms**. The rate is still re-derived when the release duration itself
> changes, and when the ceiling clamp has cancelled the rate — a protection cutting in mid-release
> must take effect at once.

`S17` pins both: it runs one stop with the ceiling deliberately moving and one with it still, and
requires the *same* time to zero. It also asserts that the ceiling really moved, because the first
version of that scenario drifted a signal outside the derate band and so passed against the defect
it was written for.

A zero commanded below `RIDE_COAST_RELEASE_ERPS` finishes immediately instead of trickling current
through the commutation-angle handover, which is the click heard at a standstill.

Reverse, brake, overtemperature and torque fault use `FIS_MODE_SAFETY` with the firmware-owned
`AP2_SAFETY_RELEASE_MS` = 200 ms, which no rider setting can lengthen.

---

## 11. The limiter chain

`ap2_limits.c`. Every demand that can reach the motor passes through it, in this order. Pedal
assist, throttle and Walk Assist all use it; there is no second path and no stage a mode can skip.

| # | Stage | Kind | What it is |
|---|---|---|---|
| 1 | POWER | cap | the profile ceiling in watts, converted at the measured duty |
| 2 | BATTERY | cap | the configured battery-current ceiling (`battery_iq_cap.c`) |
| 3 | PHASE / Iq | cap | the level ceiling and the hardware phase-current ceiling |
| 4 | VOLTAGE | derate | undervoltage taper |
| 5 | THERMAL | derate | controller temperature, 75..90 °C |
| 6 | SPEED | derate | legal / configured taper — **last**, so nothing can soften it |

**Why this order.** The first three are absolute caps — each answers "how much current may flow
at all" for a different physical reason, and a `min()` of caps is order independent. The last
three are multiplicative derates that scale whatever survived; a derate has to come after the
caps, or it would scale a number the caps were about to discard and the resulting limit would
depend on which stage happened to bind.

### One physical conversion, used from both ends

```text
battery_current_mA = Iq x CAL_I x u_abs / 2048
P_W                = V_mV x I_mA / 1e6
=> Iq_for_power    = P_max_W x 2048 x 1e6 / (V_mV x CAL_I x u_abs)
```

This needs no motor constant: the duty is *measured*, so the conversion cannot be wrong by a
fixed factor the way a load-to-current map can. Its one blind spot is a duty near zero, where the
battery power is negligible anyway — handled by the `AP2_POWER_MIN_U_ABS` guard rather than by an
invented reference operating point.

### Bumpless by construction

No stage holds an integrator, so none can wind up. A cap that starts binding changes the
**target**, and the single final Iq owner moves the current to that target over the profile
release time — so entering and leaving a limit is a bounded ramp, never a torque step. That is the
whole reason limiting happens here, upstream of the one trajectory owner, rather than at the
current regulator.

---

## 12. Walk Assist

Unchanged in behaviour and still a separate demand owner: its own motor-speed controller owns its
complete Iq trajectory, because a second dynamic element behind that speed loop would only make it
less stable. Target range remains **10..60 chainring rpm**, default 30.

What **did** change: Walk now passes the same `ap2_limits` chain as everything else — battery
current, phase current, undervoltage and controller temperature — instead of its own copy of two
of them. Its own wheel-speed cut-off stays its own, which is why it enters the chain as
`AP2_LIMIT_SOURCE_WALK` and the legal pedal-assist taper does not apply to it.

---

## 13. Tuning parameters

The complete rider-facing surface, and what each one *feels* like when it goes up.

| Group | Parameter | Raising it means |
|---|---|---|
| PROFILE | profile | a different behaviour, not a different number — see §8 |
| ASSIST | assist trim (`support_ratio_pct`, 100 = as designed) | full assist arrives at a **lower** pedal force |
| POWER | `max_motor_power_w` | a **ceiling only**: it can tighten the profile envelope, never widen it |
| ACCELERATION | attack (`iq_rise_fast_ms`) | *lowering* it makes the motor answer a change sooner; too low feels twitchy |
| | release (`release_ms`) | *lowering* it makes the motor let go sooner; too low feels like a cut-out |
| START | start (`smooth_start.duration_ms`) | the first torque of a ride is softer |
| START | `minimum_pedal_load_centikg` | more force needed to start from a standstill |
| | `riding_start_load_centikg` | more force needed to re-engage while rolling |
| | `tuning_config_start_steps()` | more crank movement needed before assist may start |
| BASE ASSIST | `full_scale_centikg` (global) | the same pedal force counts as **less** effort — everything gets gentler |
| LIMITS | battery current, phase current %, speed limit | the corresponding ceiling |

Attack, release and start are honoured for **fixed** profiles only. Choosing AUTO *is* choosing to
let the pipeline pick the dynamics; pinning them by hand would leave a mode that adapts its
strength but not its character, which is worse than either choice made deliberately.

The assist trim is a **trim**, not a replacement, so it composes with an AUTO blend instead of
freezing the gain AUTO exists to move.

Everything else in the stored bank record is **wire ballast**: still stored, still round-tripped,
read by no control path. See `inc/assist_bank_wire.h`. Do not wire a new feature onto one of those
fields — a reader would have no way to tell a value set for the old meaning from one set for the
new one.

### The stored-configuration contract

A bank blob crosses two codebases, so its meaning has to be written down once and obeyed at both
ends. These three rules were each broken in a different way before they were stated here, and
every one of them is now pinned by `tests/host/assist_bank_contract_host.c`.

**Supported mode numbers.** The stored `mode_type` byte is the rider's profile choice. This
firmware interprets:

| Stored value | Meaning |
|---|---|
| 0 | reserved — the level produces no assist |
| 1..6 | a legacy mode. Accepted and **migrated** to the closest V2 profile on read; the blob keeps the original number, so a downgrade still finds what it wrote |
| 7..12 | the six V2 profiles: ECO, TRAIL, SPORT, SPORT+, AUTO, AUTO SPORT+ |
| ≥ 13 | unknown. The **whole bank** is refused, before any part of it has been applied |

An app must not silently rewrite a stored number it does not recognise: keep it, show it as
unsupported, and let the rider decide. A validator that accepts less than this list rejects the
firmware's own default bank — which is exactly what happened, on the boot restore path, where it
turned a correctly saved configuration back into compiled defaults at every power-up.

**Zero in `max_iq_pct` means the level is switched off.** Not "no extra limit". 100 is the
compiled default and always has been, so a stored 0 cannot have come from a default — it came
from somebody who set it, in an app that has always described it as switching assist off at that
level. 1..99 is an ordinary ceiling, that percentage of the phase-current maximum. A request with
no assist level behind it — Walk — says so with `AP2_LIMITS_NO_LEVEL_CEILING`, not with 0.

**Zero in a dynamics field means "the profile decides".** `iq_rise_fast_ms` (attack),
`release_ms` and `smooth_start.duration_ms` are per-level **overrides**: 0 leaves the profile's
own value in force. Neither end may clamp a stored 0 up to the 20 ms ramp floor — every shipped
level stores 0, so a single clamp turns a plain read → save with no edit at all into a real change
of behaviour on all five levels. A non-zero value is a rider's number and is still held to the
floor.

---

## 14. Telemetry

Nothing below is read to make a decision. It exists so the bike can be tuned from a log.

`assist_pipeline_telemetry()` publishes every stage: `torque_load_centikg`,
`torque_normalized_permille`, `cadence_rpm`, `pas_state`, `rider_demand_permille`,
`assist_base_permille`, `assist_dynamic_permille`, `stroke_period_ms`,
`rider_aggression_permille`, `load_state_permille`, `profile_id`, `auto_factor_permille`,
`assist_gain_pct`, `attack_ms`, `release_ms`, `max_power_w`, `assist_response_permille`,
`iq_request_before_limits`, `final_iq_request`, the six limiter states, the lifecycle flags,
`rider_power_w`, `motor_power_w`, `applied_support_ratio_pct`.

Two compact bytes summarise it:

- `assist_pipeline_state_byte()` — low nibble `ap2_pas_state_t`, high nibble `ap2_profile_id_t`.
- `assist_pipeline_reason_bits()` — `AP2_WHY_*`: not permitted, blocked, no demand, limited,
  zeroed by a limit, start, release, auto. "The current is zero" on its own says nothing about
  who zeroed it, which is what turns a regression hunt into guesswork.

### Where it goes

| Channel | Change |
|---|---|
| Live ride telemetry `0x10400..0x10409` | **schema 2**. Frame 0 carries the demand model; two frames were added at `0x10408`/`0x10409`, **above** META so no existing frame changed identifier. |
| Ride diagnostics `0x6029` | **ver 7**. Same length, same CRC; the slots whose concepts went away carry the pipeline stages. A ver6-only decoder sees 7 and stops rather than misreading. |
| Replay / regression CSV | `tests/host/pipeline/assist_pipeline_host.c` emits every stage per tick. |

---

## 15. How it is verified

| Layer | What it proves |
|---|---|
| `tests/host/ap2_pipeline_scenarios_host.c` | 17 rider-describable scenarios against the shipped chain: calm riding, a harder push, an aggressive burst, a climb, stop, reverse, restart, the profile ordering, SPORT+ still being ramped, AUTO moving continuously, AUTO SPORT+ reaching further, every limiter binding **and releasing**, safety cut, level 0, no-torque-no-assist, the limiter latch surviving an owner change, the per-level Iq ceiling and its migration, the ceiling binding the reference inside the ISR, and a release keeping its timing while the ceiling moves under it. |
| `tools/analyze_assist_ripple.py` | The one property the pipeline exists for, measured across **14 scenarios** — 20/40/60/80/100 rpm against ECO/TRAIL/SPORT/SPORT+: Iq does not reproduce pedal ripple. |
| `tests/test_ripple_analyzer_rejects.py` | That the analyzer above **refuses** seven kinds of unusable evidence rather than passing on them. |
| `tools/run_regression.py` | Deterministic whole-chain traces over the RUN/CADENCE_RAMP/CRUISE scenarios, byte-identical on rerun. |
| `tools/run_sil.py` | Closed-loop supervisory SIL plus deterministic fuzz. |
| `tools/run_electrical_sil.py` | The real FOC/PMSM/Hall path behind the pipeline. |
| `tools/run_level4.py` | Virtual rider + bicycle + battery/SOC around the production controller. |
| `sim/replay/cases/w1-*` | Six fragments of a **real recorded ride** through the production chain, each with stated behaviour criteria. |
| `tests/test_replay_behavior.py` | That every one of those criteria can reject, not only accept. |
| `AXIS stop` / `AXIS reverse` in both SIL backends | The **whole stop and reverse axis**, timed end to end — see below. |

### The stop and reverse axis, measured end to end

Every other check looks at one link. Audit correction K1 asked for the whole chain on one axis,
because a fast reference means nothing if the crank event took 400 ms to be noticed, and a fast
detection means nothing if current keeps flowing. `run_stop_reverse_axis` in `sim/evist_sil.c`
times all of it, in the backend where every stage is the production article: real
`pas_sampler`/`pas_quadrature`/`pas_direction`/`pas_liveness` consuming physical quadrature edges
from a crank the harness turns, the real pipeline, the real final-Iq owner, and — under
`EVD_SIL_REAL_FOC` — production `FOC.c` driving a PMSM plant whose dq currents are integrated. The
current it reports is the modelled machine's, not a copy of the command.

| Stage | stop | reverse |
|---|---|---|
| physical crank event → detection | 197.75 ms | 0.00 ms |
| detection → permission withdrawn | 0.00 ms | 0.00 ms |
| permission → request zero | 0.00 ms | 0.00 ms |
| permission → reference zero | 187.00 ms (`release_ms` = 190) | 0.00 ms |
| reference zero → drive current gone | −178.50 ms | 0.50 ms |
| **physical event → no drive current** | **206.25 ms** | **0.50 ms** |

The stop's negative last row is not a missing measurement: the drive current is gone *before* the
reference finishes, because the last few counts of reference are too small to move the machine.

The bounds are derived, never fitted to the result:

- stop detection is allowed the production true-stop window, `PAS_STOP_TICKS`..`PAS_STOP_TICKS_MAX`
  = 200..500 ms. That window is a deliberate setting — it is what stops a slow stroke being read
  as a stop — and K1 is explicit that a smaller constant from another manufacturer's controller is
  not a reason to change it;
- reverse detection is bounded by the quadrature itself, not a timer;
- reference-to-zero is bounded by the profile's own `release_ms`, read from telemetry;
- current-to-zero is the electrical decay of the modelled machine, and the bound is one-sided.

**Drive and braking are counted separately.** Quiet Zero deliberately drives *negative* current to
bring the rotor down; a magnitude would score that as the motor overrunning. What §19 forbids is
the motor continuing to **pull**, so the criterion is that the positive current never exceeds what
the rider was already getting — on reverse, 34.8 counts against 173 at the event.

**What this still does not prove:** that there is no audible click, and that the real M820 behaves
like the modelled machine. Those need the bike. The numbers above are printed on every gate run so
a bench session has something to compare against.

No scenario check pins an exact Iq count. The numbers here are ride-feel settings expected to move
during tuning; a test that froze them would turn every tuning change into a test failure. What is
asserted are invariants and orderings, which stay true across tuning and stop being true the moment
the architecture regresses.

### Running a replay and behaving correctly on it are two different claims

`tools/run_replay_regression.py` prints three verdicts per case, because they answer three
questions and merging them into one `CASE PASS` promised more than had been checked:

| Verdict | What it means |
|---|---|
| `REPLAY_EXECUTED` | The production chain consumed a real recorded sensor history and produced a trace. Infrastructure only — it catches a crash, a hang, a NaN, a lost column, and says nothing about the assist. |
| `BEHAVIOR_ACCEPTED` | The trace satisfies criteria stated in that case's `manifest.json`. A case with no criteria reports `BEHAVIOR_NOT_ASSESSED` and is never called a pass. |
| `OUTPUT_PINNED` | The bytes match a hash a human accepted after validating that ride. |

The criteria are quantitative and each is carried only by the fragments that can support it:

| Criterion | Asks | Carried by |
|---|---|---|
| `produces_assist` | Did any current come out at all? | all six |
| `responds_to_load` | When the pedal force moved one way, did the assist follow? | f03, f05, f06 |
| `pause_releases` | When the rider stopped, did the current actually reach and hold zero? | f07, f08 |
| `restart_recovers` | And did it come back when the rider did? | f07, f08 |
| `max_attenuation` | Did the current swing proportionally less than the pedal? | f01, f07, f08 |

**Why not all six carry `max_attenuation`.** In f05 and f06 the request sits on its ceiling for
52 % and 44 % of the fragment, and in f03 for 13 %. A clipped signal is smooth because it is
clipped, so an attenuation number measured there would pass for entirely the wrong reason. The
criterion refuses such a fragment rather than reporting the flattering value, and those fragments
carry `responds_to_load` instead — which is what they can actually settle.

The limit is `0.60`, the same number and the same meaning as `tools/analyze_assist_ripple.py`:
one standard, not two. Measured today: f01 `0.318`, f07 `0.372`, f08 `0.302`.

### The real-ride outputs are deliberately not pinned

`accepted_output_sha256` is `null` for every `w1-*` case, so `OUTPUT_PINNED` is `0/6`. Pinning
would freeze an untuned, never-ridden output as the definition of correct. That is a separate
question from behaviour: the criteria above hold the ride to stated properties while leaving the
exact numbers free to move during tuning. Pin the bytes only after a ride on the bike says the
behaviour is right.

---

## 16. What is still open

- **Every ride-feel number in the profile table is a starting point, not a measurement.** The
  pipeline has never driven the physical motor.
- **The torque sensor scale is an unresolved question** for this bike
  (`CLAIM-EVD-TORQUE-SCALE-004`: three different scales in the project's history). It sets what
  "full effort" means, so it dominates every gain in the table. Measure it with a known weight
  before tuning anything else.
- `ASSIST_LEVEL` → profile mapping for banks saved before V2 is a migration by *character*
  (§`assist_modes_profile_for_level`). A rider who had tuned a legacy mode will need to re-pick.

## Review and external reference (2026-09-14)

Before further tuning, read the [V2 audit including correction K1](AUDIT_ASSIST_PIPELINE_V2_2026-09-14_PL.md) and the [G5300 reference intake](reference/g5300/README.md). The latter preserves a user-supplied reverse summary for a different controller, with provenance, internal corrections and an explicit mapping to M820. It is not a specification for copying stock constants or a request to replace V2 again.

The [G5300 closure qualification and audit K2](reference/g5300/CLOSURE_REVIEW_2026-09-14_PL.md) supersedes the initial intake's open alpha question and records the newly documented downstream current-command trajectory. It also tracks remaining gaps, including current scaling, limiter coverage and the 72/144 event-rate discrepancy. The original audit targets `9dc0b0a`; subsequent implementation fixes require their own review.
