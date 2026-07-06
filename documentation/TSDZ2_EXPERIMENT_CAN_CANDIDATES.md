# TSDZ2 experiment - CAN configuration candidates

This file tracks the temporary constants added for the TSDZ2-style assist
experiment. Nothing here is exposed over CAN yet. If the ride tests confirm the
behaviour, these are the candidates to move from compile-time defines to runtime
configuration.

Reference points from TSDZ2 860C user configuration:

- Torque sensor: Torque ADC threshold, Assist w/o pedal rotation, calibration,
  Torque ADC offset, Torque ADC max.
- Startup boost: feature enable, Startup boost torque factor, Startup boost
  cadence step.
- Motor: Motor acceleration.
- Assist level: per-mode/per-level assist factors.

Source:
https://github.com/emmebrusa/TSDZ2-Smart-EBike-860C/wiki

## High-priority candidates

| EBICS define | Current value | TSDZ2 user setting closest in meaning | Proposed CAN field | Why expose later |
| --- | ---: | --- | --- | --- |
| `ASSIST_TORQUE_FULL_SCALE_MV` | 500 mV | Torque ADC max / torque sensor scaling | `assist_torque_full_scale_mv` | Defines the useful pressure span used by clean start and running raw floor. Added after first test because mapping to 3300 mV made start current too small. |
| `ASSIST_START_DEADBAND_MV` | 35 mV | Torque ADC threshold / Torque ADC offset sensitivity | `assist_start_deadband_mv` | Main start sensitivity. Raise if motor starts from too little pressure; lower if start needs too much pressure. |
| `ASSIST_START_LATCH_TICKS` | 1200 ticks / 300 ms | No exact direct item; related to startup boost timing | `assist_start_latch_ms` | Short sync window between a fresh pressure sample and fresh PAS movement. Improves repeatability when torque and PAS pulses do not happen on the same tick. |
| `ASSIST_RUN_DEADBAND_MV` | 12 mV | No exact direct item; related to torque threshold hysteresis | `assist_run_deadband_mv` | Keeps running threshold lower than start threshold, so assist does not blink off at light low-cadence pedalling. |
| `ASSIST_ENGAGE_TICKS` | 800 ticks / 200 ms | Motor acceleration / startup shaping | `assist_engage_ms` | Defines identical 0-to-power start envelope every time. |
| `ASSIST_ENGAGE_DROPOUT_TICKS` | 600 ticks / 150 ms | No exact direct item; related to startup robustness | `assist_engage_dropout_ms` | Allows a short torque dip during ENGAGE so the motor does not abort the start on crank dead spots. |
| `ASSIST_START_BOOST_GAIN_PCT` | 30 % | Startup boost torque factor | `assist_start_boost_gain_pct` | Controls the extra kick on fresh start, proportional to actual pressure. |
| `ASSIST_START_BOOST_MAX_PCT` | 35 % | Startup boost safety limit, no exact direct TSDZ2 item | `assist_start_boost_max_pct` | Caps the start kick so high assist levels do not become too harsh. |
| `ASSIST_RUN_RAW_FLOOR_PCT` | 40 % | No exact direct item; related to torque assist/eMTB sensitivity | `assist_run_raw_floor_pct` | Adds a raw torque floor while already running; useful against power disappearing too early. |
| `ASSIST_SUSTAIN_TICKS` | 1600 ticks / 400 ms | No exact direct item | `assist_sustain_ms` | Holds through crank dead spots and slow light pedalling, but never starts from zero by itself. |
| `ASSIST_CURRENT_FALL_TICKS` | 16 ticks / about 4 ms per current step | Motor acceleration / ramp-down feel | `assist_fall_ticks` or `assist_fall_ms` | Controls how softly power falls when rider pressure decreases. |
| `IQ_RAMP_UP_SLOW_TICKS` | 9200 ticks / 2300 ms | Motor acceleration ramp-up default | `iq_ramp_up_slow_ms` | Low-speed/start ramp-up time. This is the main "no kick from standstill" comfort knob. |
| `IQ_RAMP_UP_FAST_TICKS` | 1200 ticks / 300 ms | Motor acceleration ramp-up minimum | `iq_ramp_up_fast_ms` | In-ride ramp-up time when speed or cadence already confirms normal riding. |
| `IQ_RAMP_DOWN_SLOW_TICKS` | 4000 ticks / 1000 ms | Motor acceleration ramp-down default | `iq_ramp_down_slow_ms` | Soft power fade at low speed; does not override brake/reverse/thermal immediate cuts. |
| `IQ_RAMP_DOWN_FAST_TICKS` | 560 ticks / 140 ms | Motor acceleration ramp-down minimum | `iq_ramp_down_fast_ms` | Fast power release during normal riding without making pedal-off feel like a hard cut. |

## Medium-priority candidates

| EBICS define | Current value | TSDZ2 user setting closest in meaning | Proposed CAN field | Why maybe expose |
| --- | ---: | --- | --- | --- |
| `ASSIST_SUSTAIN_MIN_PCT` | 5 % | No exact direct item | `assist_sustain_min_pct` | Minimum small current while sustain window is active. Useful if sustain feels either empty or too pushy. |
| `ASSIST_START_MIN_FWD_STEPS` | 2 steps | Assist w/o pedal rotation is related but different | `assist_start_min_fwd_steps` | Anti-false-start confidence gate. Usually better kept hidden unless testing shows starts from tiny accidental movement. |
| `START_CADENCE_SEED_RPM` | 10 rpm | Startup boost cadence step is related | `start_cadence_seed_rpm` | Temporary first cadence value after valid forward pressure start. Expose only if testers need to tune first-assist latency. |

## Probably keep internal

| EBICS define | Current value | Reason |
| --- | ---: | --- |
| `ASSIST_TORQUE_ZERO_MV` | 740 mV | This is the normalized zero point after offset correction. TSDZ2 exposes torque calibration (`Torque ADC offset`, `Torque ADC max`) instead of a simple start algorithm constant. If exposed later, it should be part of torque sensor calibration, not a user comfort slider. |

## Already related EBICS CAN/EEPROM settings

These are not new in the experiment, but they overlap with the TSDZ2 ideas and
should be considered before adding more fields:

| Existing EBICS field | Source in code | Related TSDZ2 idea |
| --- | --- | --- |
| `MP.ext_boost_duration[]` | `Para2[31..35]` | Startup/extended boost duration. |
| `MP.ext_boost_strength[]` | `Para2[37..41]` | Startup/extended boost strength. |
| `MP.PAS_timeout` | `Para1[38]` | Current loading / assist timeout behaviour. |
| `MP.ramp_end` | `Para1[39]` | Current shedding / ramp-down behaviour. |
| `MP.decay_base` | `Para1[21]` | Current decay curve after torque fades. |
| `MP.TQO_threshold[]` | `Para0[12..27]` | Torque override threshold per level. |
| `MP.assist_profile[][]` | `Para2[0..29]` | Per-level assist shaping. |

## Suggested order if this goes to CAN

1. Add only the high-priority comfort knobs first:
   `assist_start_deadband_mv`, `assist_run_deadband_mv`, `assist_engage_ms`,
   `assist_start_boost_gain_pct`, `assist_start_boost_max_pct`,
   `assist_run_raw_floor_pct`, `assist_sustain_ms`, `assist_fall_ms`.
2. Keep the medium-priority knobs compiled in until ride tests prove they are
   needed by users.
3. Keep `ASSIST_TORQUE_ZERO_MV` inside torque calibration, not inside the assist
   feel menu.
4. Reuse existing Para/CAN slots only after checking the app layout, because some
   Bafang fields are already repurposed in this firmware.
