# Early assist experiment - analysis and roadmap

> **ARCHIWALNY** (2026-07-16): pierwszy eksperyment TSDZ2. Aktualne zrodlo: RIDE_CORE_MASTER_CHECKLIST_PL.md + RIDE_CORE_STATUS_CANABLE.md. Mapa dokumentacji: documentation/README.md.

This document collects the design notes, ride-test expectations, and possible
next implementation stages for the early EBICS pedal assist experiment.
It is intended for analysis after the first test firmware ride.

No firmware code is changed by this document.

## 1. Original problem

Observed behaviour before the experiment:

- Assist can sometimes start when the rider only rotates the cranks lightly,
  with little or no real pedal pressure.
- After assist disappears, a small forward crank movement can sometimes start
  assist again, as if some previous state was not fully reset.
- The longer assist is inactive, the unwanted restart effect seems smaller.
- At slow, light pedalling the motor power can appear and disappear repeatedly.
- When rider pressure is reduced, EBICS can reach assist cut-off too quickly
  instead of falling smoothly and proportionally.

The suspected cause is not one single variable. The old behaviour was likely
the result of several mechanisms interacting:

- torque filter accumulation,
- torque counter / cadence gate,
- torque override,
- extended boost / overrun,
- decay logic,
- PAS timeout,
- current output history.

## 2. Desired behaviour

The requested target feel:

- Every new assist start must feel identical after assist has disappeared.
- Nothing should accumulate from the previous assist event and influence the
  next start.
- Light crank rotation without real pressure should not start the motor.
- Assist should appear proportionally to pedal pressure.
- Strong pressure should give a strong, immediate increase, but still limited
  and controlled.
- After the clean start phase, the normal/developer assist algorithm may run.
- The path from zero to power can be different from the path while power is
  already active.
- A start kick / boost is wanted, but it must be predictable.
- During slow light pedalling, assist should not blink on/off every crank dead
  spot.
- When pressure is reduced, power should fall smoothly, not cut suddenly.

## 3. What is implemented in the first experiment

Branch:

`experiment/tsdz-experiment`

Main firmware path:

- `src/main.c`
- `inc/config.h`

Documentation:

- `archive/EXPERIMENT_CAN_CANDIDATES.md` (ten katalog)
- `archive/ASSIST_ANALYSIS_AND_ROADMAP.md` (ten plik)

The first experiment adds a separate assist state machine:

`OFF -> ENGAGE -> RUNNING -> RELEASE`

### OFF

Assist is inactive. Runtime memory is cleared:

- torque accumulation,
- filtered torque,
- human power estimate,
- torque counter,
- overrun strength,
- overrun counter,
- overrun flag,
- engage counter,
- sustain counter,
- last assist current,
- current fall counter.

The goal is to make the next start independent from old assist history.

### ENGAGE

Clean start phase.

Assist may start only when:

- assist level is above zero,
- scaled phase current limit is above zero,
- torque sensor is not in fault state,
- PAS direction is forward,
- enough fresh forward PAS steps were seen,
- current torque is above the start deadband.

Start current is calculated directly from current raw/normalized torque above
the start threshold. It does not use old torque filter memory, old overrun, or
old extended boost state.

### RUNNING

Normal ride phase.

The old developer algorithm still runs here:

- assist level current limit,
- speed limit per assist level,
- torque filter setting,
- assist profile,
- TS coefficient,
- cadence exponent,
- torque override,
- extended boost,
- PAS timeout,
- decay base,
- voltage limit,
- temperature limit,
- legal speed limit.

The experiment adds an extra running shaper:

- raw torque floor while already running,
- short sustain window through crank dead spots,
- slower current fall when requested power drops.

### RELEASE

Assist is commanded down. When output reaches zero, runtime memory is cleared
and the system returns to OFF.

## 4. New experimental constants

Current compile-time constants in `inc/config.h`:

| Define | Current value | Meaning |
| --- | ---: | --- |
| `ASSIST_TORQUE_ZERO_MV` | 740 mV | Normalized no-load torque point after offset correction. |
| `ASSIST_TORQUE_FULL_SCALE_MV` | 500 mV | Useful pressure span above zero mapped to full scaled assist during clean start and raw running floor. |
| `ASSIST_START_DEADBAND_MV` | 35 mV | Pressure above zero required to start from OFF. |
| `ASSIST_START_LATCH_TICKS` | 1200 ticks | About 300 ms sync window between fresh pressure and fresh PAS movement. |
| `ASSIST_RUN_DEADBAND_MV` | 12 mV | Lower pressure threshold while already RUNNING. |
| `ASSIST_START_MIN_FWD_STEPS` | 2 steps | Fresh forward PAS steps required before start. |
| `ASSIST_ENGAGE_TICKS` | 800 ticks | About 200 ms start envelope at 4 kHz. |
| `ASSIST_ENGAGE_DROPOUT_TICKS` | 600 ticks | About 150 ms torque dropout allowed during ENGAGE before aborting start. |
| `ASSIST_START_BOOST_GAIN_PCT` | 30 % | Extra start kick proportional to current pressure. |
| `ASSIST_START_BOOST_MAX_PCT` | 35 % | Maximum start kick as percent of scaled phase current limit. |
| `ASSIST_RUN_RAW_FLOOR_PCT` | 40 % | Raw torque floor blended in while RUNNING. |
| `ASSIST_SUSTAIN_TICKS` | 1600 ticks | About 400 ms sustain through dead spots at 4 kHz. |
| `ASSIST_SUSTAIN_MIN_PCT` | 5 % | Small floor while sustain is active. |
| `ASSIST_CURRENT_FALL_TICKS` | 16 ticks | Current falls by one step every N ticks. |

## 5. Existing developer parameters still active

The first experiment does not remove the old developer parameters. Most still
have real influence, but not all are allowed to trigger assist from zero.

### Still active at start and while riding

`assist_settings[level][0]`

Current limit per assist level. It still defines `phase_current_max_scaled`.
The new start is also capped by this value.

`assist_settings[level][1]`

Speed limit per assist level. Still defines `speedlimitx100_scaled`.

`phase_current_max`, `battery_current_max`, `voltage_min`

Still apply as global power/current/voltage limits.

Temperature and legal speed limiting still apply.

Brake cut-off still applies.

Throttle still exists as a separate override path.

### Active mainly after clean start

`assist_settings[level][2]` / `TQfilter`

Still shapes filtered torque while riding. The important change is that filter
memory is cleared for a new OFF -> ENGAGE start.

`assist_profile[][]`

Still shapes normal assist in RUNNING.

`TS_coeff`

Still controls torque/power assist calculation in RUNNING.

`Cadence_exponent`

Still controls cadence sensitivity in RUNNING.

`TQO_threshold[]`

Still controls Torque Override in RUNNING. It should no longer be able to cause
an unclean start from OFF.

`ext_boost_duration[]`, `ext_boost_strength[]`, `Override_Duration`

Still control extended boost/overrun in RUNNING. They should no longer provide
the first start decision from OFF.

`PAS_timeout`, `decay_base`

Still affect the old decay/current reduction path in RUNNING. The new
sustain/fall layer is above this, so the ride feel may be smoother than before.

### Parameter to re-check

`ramp_end`

It is still parsed from configuration, but in the currently inspected assist
path it does not appear to have a strong direct effect on the new start or
running assist. Re-check before relying on it as a tuning knob.

## 6. What to test in the first firmware ride

### Start from standstill

Test:

- level 1/2 light pressure start,
- medium pressure start,
- strong pressure start.

Expected:

- no assist from crank movement alone,
- start point should be repeatable,
- strong pressure should give stronger assist,
- no old-state kick after previous assist stopped.

Feedback needed:

- starts too early,
- starts too late,
- start kick too weak,
- start kick too aggressive,
- start repeatability OK/not OK.

### Stop and restart

Test:

- ride until assist is active,
- stop pedalling until assist disappears,
- lightly rotate forward,
- then apply real pressure.

Expected:

- light rotation alone should not restart assist,
- real pressure should restart in the same way each time.

Feedback needed:

- still feels like memory remains,
- clean and repeatable,
- delay too long after restart.

### Slow light pedalling

Test:

- low cadence,
- very light pressure,
- flat ground and slight climb if possible.

Expected:

- less on/off pulsing,
- less sudden cut near dead spots,
- assist should not push strongly with no pressure.

Feedback needed:

- still pulses,
- sustain too weak,
- sustain too long,
- rower "sam niesie" too much.

### Reducing pressure while riding

Test:

- ride with medium assist,
- gradually reduce pedal pressure.

Expected:

- power falls smoothly,
- no hard cut when torque briefly drops,
- full release still cuts assist safely.

Feedback needed:

- still cuts too quickly,
- falls too slowly,
- feels natural,
- feels delayed.

### Reverse crank movement

Expected:

- assist should not start,
- active assist should be released.

## 7. First tuning after ride test

### First test result: no assist start

The first ride result was that the motor did not start on any assist level, at
any pedal pressure.

Most likely reasons in the first implementation:

- clean-start torque was mapped from start threshold to 3300 mV, which made
  practical start current too small if the real sensor pressure span is much
  lower;
- ENGAGE required torque to stay above the run threshold for the full start
  envelope, so a short crank dead-spot/dropout could abort the start repeatedly.

Fix applied:

- added `ASSIST_TORQUE_FULL_SCALE_MV = 500` and use it for clean start and
  running raw floor mapping;
- added `ASSIST_ENGAGE_DROPOUT_TICKS = 160` so ENGAGE survives a short torque
  dropout;
- build artifact after this fix: `.build/0.0100_M820_BL820.bin`.

### Second test result: start improved but still irregular

The second ride result was that the motor starts much better, but engage is
still not predictable enough.

Most likely remaining reason:

- the clean start still required current torque and fresh PAS movement to line
  up closely in time;
- in real pedalling, pedal pressure can appear just before the PAS forward step,
  or the PAS step can appear while torque is already falling through a crank
  dead spot.

Fix applied:

- added `ASSIST_START_LATCH_TICKS = 1200`, a short 300 ms pressure/PAS sync
  latch;
- the latch stores only a fresh start pressure peak and still requires fresh
  forward PAS movement before assist can start;
- increased `ASSIST_ENGAGE_DROPOUT_TICKS` from 160 to 600 ticks, about 150 ms,
  so ENGAGE does not abort on a normal crank dead spot;
- build artifact after this fix: `.build/0.0102_M820_BL820.bin`.

### Third test result: sustain/fade improved, start still not predictable

The third ride result was positive for sustain and power fade:

- assist hold while slow/light pedalling is much better;
- power fade when pressure decreases is also better.

The remaining issue is start predictability.

Likely cause found in code:

- the start pressure latch was cleared whenever PAS was idle;
- this means pressure applied before the first crank movement could be discarded;
- the start still depended on torque and PAS forward steps lining up closely in
  time.

Fix applied:

- start pressure latch is no longer cleared only because PAS is idle;
- it is still cleared on reverse/backward movement;
- assist still cannot start from pressure alone, because fresh forward PAS
  movement is still required before OFF can enter ENGAGE.

Use this order. Change only one or two values per test round.

### If assist starts too easily

Increase:

- `ASSIST_START_DEADBAND_MV`, for example 35 -> 45 mV.

Optionally increase:

- `ASSIST_START_MIN_FWD_STEPS`, for example 2 -> 3.

### If assist needs too much pressure to start

Decrease:

- `ASSIST_START_DEADBAND_MV`, for example 35 -> 25 mV.

### If start kick is too weak

Increase:

- `ASSIST_START_BOOST_GAIN_PCT`, for example 30 -> 40 %.

Possibly increase:

- `ASSIST_START_BOOST_MAX_PCT`, for example 35 -> 45 %.

### If start kick is too harsh

Decrease:

- `ASSIST_START_BOOST_GAIN_PCT`, for example 30 -> 20 %.

Decrease:

- `ASSIST_START_BOOST_MAX_PCT`, for example 35 -> 25 %.

Possibly increase:

- `ASSIST_ENGAGE_TICKS`, for example 800 -> 1000/1200.

### If slow light pedalling still pulses

Decrease:

- `ASSIST_RUN_DEADBAND_MV`, for example 12 -> 8 mV.

Increase:

- `ASSIST_SUSTAIN_TICKS`, for example 1600 -> 2000.

Increase:

- `ASSIST_RUN_RAW_FLOOR_PCT`, for example 40 -> 50 %.

### If assist keeps pushing too long after pressure is released

Decrease:

- `ASSIST_SUSTAIN_TICKS`, for example 1600 -> 1000/1200.

Decrease:

- `ASSIST_SUSTAIN_MIN_PCT`, for example 5 -> 3 %.

Decrease:

- `ASSIST_CURRENT_FALL_TICKS`, for example 16 -> 8/12.

### If power falls too suddenly when pressure decreases

Increase:

- `ASSIST_CURRENT_FALL_TICKS`, for example 16 -> 24/32.

Possibly increase:

- `ASSIST_RUN_RAW_FLOOR_PCT`, for example 40 -> 45/50 %.

## 8. TSDZ2 ideas worth borrowing

The TSDZ2 860C firmware exposes concepts that are useful for EBICS:

- torque sensor calibration,
- torque ADC offset,
- torque ADC max,
- torque ADC threshold,
- motor acceleration,
- startup boost torque factor,
- startup boost cadence step,
- multiple assist modes:
  - POWER,
  - TORQUE,
  - CADENCE,
  - EMTB,
  - HYBRID.

The most relevant ideas for EBICS are:

1. normalize torque sensor input,
2. separate rider assist request from motor output shaping,
3. make startup boost cadence-dependent,
4. add hybrid torque/power behaviour,
5. add progressive eMTB-like torque curve,
6. make sustain dependent on crank phase/PAS steps, not only time,
7. add a proper motor acceleration/deceleration limiter.

Reference:

https://github.com/emmebrusa/TSDZ2-Smart-EBike-860C/wiki

## 9. Auto-calibration and torque drift

Manual torque calibration can drift because the sensor zero changes with:

- temperature,
- mechanical stress,
- sensor ageing,
- crank position,
- whether the pedal was pressed at power-on.

The recommended model is:

`torque_zero_base + torque_zero_trim = torque_zero_effective`

Where:

- `torque_zero_base` comes from manual/factory/user calibration,
- `torque_zero_trim` is a small automatic correction during confirmed no-load
  coast conditions.

Do not let auto-calibration freely move the whole zero point. That risks
treating a light foot resting on the pedal as the new zero.

Auto-zero should only run when:

- PAS is idle for long enough,
- cadence is zero,
- motor current/output is zero or near zero,
- throttle is inactive,
- torque signal is stable,
- torque raw value is inside a plausible rest window.

Auto-zero should use:

- small correction steps,
- deadband around zero,
- maximum total trim limit,
- several repeated coast confirmations before accepting a larger correction.

Torque max should probably remain a manual/service calibration value. Normal
riding does not tell firmware when the rider applied true maximum pedal force.

## 10. Proposed implementation stages after first test

### Stage 0 - Current first ride test

Status:

- already implemented in the experiment branch.

Goal:

- verify clean repeatable start,
- verify no false start from crank movement alone,
- verify less pulsing at slow light pedalling,
- verify smoother power fall.

Do not add CAN configuration yet. Tune compile-time constants first.

### Stage 1 - Stabilize the experiment values

Implement only if first ride shows the direction is correct.

Work:

- tune start deadband,
- tune run deadband,
- tune start boost,
- tune sustain,
- tune current fall.

Output:

- one known-good baseline set of constants.

No CAN yet unless values are clearly bike/rider dependent.

### Stage 2 - Torque normalization layer

Add:

- `torque_zero_base_mv`,
- `torque_zero_trim_mv`,
- `torque_zero_effective_mv`,
- `torque_max_mv`,
- `torque_norm` in a fixed range, for example 0..1000.

Expected benefit:

- better repeatability,
- easier tuning,
- less dependence on raw mV thresholds,
- easier future CAN diagnostics.

Estimated ride-feel impact:

- 30-40 % of the total improvement in repeatable start feel.

### Stage 3 - Safe auto-zero trim

Add:

- coast-based zero trim,
- rate limit,
- plausibility window,
- maximum trim range,
- CAN/debug visibility of trim and raw torque.

Expected benefit:

- compensation for sensor drift,
- less need for manual re-calibration,
- better start consistency after temperature changes.

Estimated ride-feel impact:

- 10-20 %, but very important for long-term consistency.

### Stage 4 - Separate assist request from motor output shaping

Refactor logic into two conceptual stages:

`rider input -> assist request`

and:

`assist request -> motor current output`

Expected benefit:

- old developer algorithm can remain,
- start/running/release shaping becomes easier to reason about,
- fewer accidental interactions between boost, decay, and filters.

Estimated ride-feel impact:

- 20-30 % improvement in smoothness and tunability.

### Stage 5 - Cadence-dependent startup boost

Borrow from TSDZ2 startup boost:

- high boost at cadence 0,
- boost fades as cadence increases,
- configurable fade step.

Possible fields:

- `start_boost_gain_pct`,
- `start_boost_max_pct`,
- `start_boost_cadence_step`.

Expected benefit:

- strong start from standstill,
- less nervous behaviour while already pedalling.

Estimated ride-feel impact:

- 15-25 % improvement in start feel.

### Stage 6 - Crank-phase/PAS-step sustain

Replace or supplement time-only sustain with PAS-step/crank-fraction sustain.

Expected benefit:

- better support through crank dead spots,
- sustain adapts naturally to cadence,
- less pulsing at slow pedalling without too much push at higher cadence.

Estimated ride-feel impact:

- 15-25 % reduction in low-cadence pulsing.

### Stage 7 - Hybrid torque/power assist mode

Add a mode or blend:

- low cadence: more torque-based,
- medium cadence: torque + human power blend,
- high cadence: more power/cadence extension.

Expected benefit:

- good climbing/slow cadence response,
- less dead feel when pedalling slowly,
- more natural assistance across cadence range.

Estimated ride-feel impact:

- 25-35 % improvement in natural ride feel.

### Stage 8 - Progressive eMTB-like curve

Add progressive torque curve:

- low pressure gives gentle support,
- medium pressure is linear/natural,
- high pressure reaches power quickly.

Expected benefit:

- less twitchy at light pressure,
- stronger when rider really pushes,
- modern eMTB-like feel.

Estimated ride-feel impact:

- 20-30 % in sport/natural feel.

### Stage 9 - CAN configuration

Only after values prove useful in real rides.

High-priority fields:

- `assist_start_deadband_mv`,
- `assist_run_deadband_mv`,
- `assist_engage_ms`,
- `assist_start_boost_gain_pct`,
- `assist_start_boost_max_pct`,
- `assist_run_raw_floor_pct`,
- `assist_sustain_ms`,
- `assist_fall_ms`.

Possible later fields:

- `assist_sustain_min_pct`,
- `assist_start_min_fwd_steps`,
- `torque_zero_base_mv`,
- `torque_max_mv`,
- `start_boost_cadence_step`,
- `motor_accel`,
- `motor_decel`,
- `assist_curve_mode`.

Do not expose too many knobs at once. First expose only fields that clearly
change ride feel and need user/rider adjustment.

### Stage 10 - Diagnostics for test riding

Useful CAN/debug fields:

- assist state: OFF/ENGAGE/RUNNING/RELEASE,
- raw torque mV,
- corrected torque mV,
- torque normalized value,
- torque zero trim,
- start threshold,
- run threshold,
- current request before shaping,
- current output after shaping,
- sustain counter,
- start boost amount,
- PAS forward steps,
- cadence,
- reason for release/cut.

Expected benefit:

- much faster tuning,
- easier to separate sensor/calibration problems from assist algorithm problems.

## 11. Priority summary

Suggested order:

1. Test current firmware.
2. Tune current constants only if needed.
3. Add torque normalization.
4. Add safe auto-zero trim.
5. Split assist request from motor output shaping.
6. Add cadence-dependent start boost.
7. Add crank-phase sustain.
8. Add hybrid assist blend.
9. Add progressive eMTB curve.
10. Expose proven parameters over CAN.
11. Add ride diagnostics.

Estimated contribution to final ride feel:

- torque normalization and thresholds: about 30-40 %,
- hybrid/sustain at low cadence: about 25-35 %,
- cadence-dependent startup boost: about 15-25 %,
- output shaping / acceleration / deceleration: about 15-25 %,
- diagnostics and CAN configuration: not directly ride feel, but high value for
  tuning speed and correctness.

These percentages are estimates for prioritization, not measured values.

## 12. Feedback template after first ride

Use short observations like:

- start too early / too late,
- start kick too weak / too strong,
- start repeatability OK / not OK,
- light crank movement still starts assist / does not start assist,
- low-cadence light pedalling pulses / smooth,
- power falls too quickly / too slowly / naturally,
- assist pushes after release / stops correctly,
- level 1 good but level 4 too strong,
- flat ground OK but climb not OK,
- cold start different from warm start.

The most important question:

After assist fully disappears, does the next start feel identical every time?
