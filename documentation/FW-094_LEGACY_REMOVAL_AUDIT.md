# FW-094 — Legacy removal audit

**Status: implemented in code, NOT built, NOT ridden.**
Behaviour is intended to be identical. Walk Assist and position calibration are touched, so
both need a bike test before this card counts as confirmed.

## What was actually still alive

Engine selection had been gone since FW-030. What had not gone was the assist monolith itself:
`legacy_assist_calculate_monolith()` was still called on every control tick from two places —
Walk Assist and phase 2 of position calibration.

On both paths it ran its **entire** pre-ride-core assist body first (cadence map, pressure
floor, throttle override, smooth-start envelope, overrun) and then discarded the result:

- **Walk Assist** — the walk branch overwrote it with `walk_motor_update()`.
- **Calibration** — the phase-2 block overwrote it unconditionally with the fixed probe current.

So the old assist math executed continuously and could not influence the motor. The old
Extended Boost ("overrun") was dead twice over: behind `EXTENDED_BOOST_ENABLE 0` *and* inside a
branch nothing reached.

## Classification

| Element | File | Role | Class | Decision |
|---|---|---|---|---|
| `legacy_assist_calculate_monolith()` | main.c | 2 reachable branches, rest dead | mixed | split into two named functions |
| `legacy_assist.c/.h` | – | empty bridge | dead | removed |
| `ride_engine_t`, `ride_control_get_engine()` | ride_control | returned a constant | dead | removed |
| `Overrun_strength/counter/flag` | main.c | old power drag-on | Legacy-only | removed |
| `MS.ext_boost_duration/strength` | main.h | cache for the overrun block | Legacy-only | removed |
| `MS.i_q_setpoint_temp`, `i_d_setpoint_temp` | main.h | monolith scratch registers | dead | removed |
| `assist_limits_apply_legacy()` | assist_limits | MS/MP wrapper | Legacy-only | removed |
| `map_rezi()`, `interpolate_assistfactor()`, `helper` | main.c | old assist math | dead | removed |
| `ASSIST_TORQUE_MODE`, `ASSIST_CURVE_EXPO_L*`, `SMOOTH_START_ENABLE`, `START_RAMP_TICKS`, `EXTENDED_BOOST_ENABLE`, `RIDE_ENGINE_DEFAULT`, `TQ_GATE_RELEASE`, `START_MIN_STEPS` | config.h | build switches of the old path | Legacy-only | removed |
| `ASSIST_MODE_LEGACY = 0` | assist_modes.h | wire value | protocol field | renamed `ASSIST_MODE_RESERVED_0`, value kept |
| byte 3 of 0x6028 / 0x6029 | CAN_Display.c | engine id | protocol field | constant `DIAG_ENGINE_ID_RIDE_CORE`, marked deprecated |
| `Override_Duration`, `decay_base`, `TS_coeff`, `ext_boost_*[6]`, `assist_profile`, `TQO_threshold`, `ramp_end`, `MagicNumber` | main.h (EEPROM) | not read by anything | **orphan, frozen** | kept, documented |
| `PAS_timeout`, `assist_settings` | main.h | LIVE | shared | kept |
| FW-084 Extended Boost | assist_extended_boost.c | native ride core | TSDZ | kept — see "Open question" |

## A. Removed

The monolith and all of its arithmetic; `legacy_assist.c/.h`; the engine type and its getter;
the overrun state; the monolith's scratch fields in `MotorState_t`; the legacy limiter wrapper;
`map_rezi()`; `interpolate_assistfactor()`; `helper`; `mapped_torque` / `mapped_throttle`;
`assist_curve_exponent`; and the config switches listed above.

No `if (1)`, no `#define USE_TSDZ`, no fallback. `src/legacy_assist.c` is also out of
`scripts/sources-m820.txt`.

## B. Kept

The two reachable branches became purpose-built motor-layer functions, declared in the new
`inc/motor_service.h` and defined in main.c next to the hardware state they need:

- `walk_assist_iq_request()` — Walk Assist plus the shared undervoltage and controller
  temperature limits. The legal speed taper stays suppressed, exactly as the removed wrapper
  did it: Walk Assist carries its own wheel-speed ceiling.
- `hall_calibration_iq_request()` — phase 2 of position calibration.

Both perform the same operations in the same order as before. **One intentional difference:**
the Walk result is clamped to 0…65535 instead of relying on implicit truncation. In the
reachable range this cannot change a value; it removes a latent hazard.

Useful side effect: `assist_limits.h` no longer includes `main.h`. The removed wrapper was the
only thing binding the shared limiter to this controller's globals, so the limiter is now
genuinely motor-agnostic — integers in, limited current out.

## C. Protocol / UI migration

| Field | Situation | Action taken |
|---|---|---|
| byte 3 of 0x6028 / 0x6029 | app parses both blocks positionally | kept as a constant, documented as deprecated |
| byte 4 of 0x6028 (pending engine) | same | kept at 0xFF, documented |
| `mode_type: legacy` → `reserved_0` | schema + firmware | renamed; **wire value 0 unchanged** |
| "Legacy decay base", "Cadence exponent", "Legacy Extended Boost duration", "Legacy ramp-end" | UI claimed "compiled in but not reached" — no longer true | help text corrected to "dead setting, the code was removed" |
| "PAS timeout" | **LIVE**, but looked Legacy by association | help text now states plainly that it does affect riding |
| blue "Ext. Boost" labels on the factory Bafang tabs | collided with the FW-084 name | relabelled "Old overrun (unused since FW-094)" |

**Orphan parameters** — written by the app, echoed by the firmware, read by nobody:
`Override_Duration`, `decay_base`, `TS_coeff`, `ext_boost_duration[6]`, `ext_boost_strength[6]`,
`assist_profile[5][6]`, `TQO_threshold[6]` (now only a parser sanity value), `ramp_end`,
`MagicNumber`.

Removing them is a protocol change, not a cleanup: it needs a Para-block version bump and a
matching app release. Deliberately out of scope here.

## D. Open question — needs an owner decision

FW-084 Extended Boost is native to the ride core, so it is out of scope for a Legacy removal
and was not touched. But it does the one thing this card's brief warned about: it **holds motor
torque for up to 1000 ms after the cranks stop**, on a bike with no brake sensor.

Facts, so the decision can be made on them:

- It is **off by default** (duration 0) and, per the project notes, has never been confirmed on
  the bike.
- It has its own cancels: safety cut, reverse, sensor invalid, motion lost, pedalling resumed,
  arm timeout, walk, calibration, config change.
- It is classified NON_PEDAL, so in legal mode it tapers from 5 km/h and gives nothing from 7.

Options: leave it off, remove it, or reshape it into "extend the boost while correct pedalling
is still detected". Not decided here.

## E. Call flow after the refactor

```
ride_control_update
  ├─ walk_active        → walk_assist_iq_request       (motor_service)
  ├─ calibration_active → hall_calibration_iq_request  (motor_service)
  └─ riding:
       rider_input → assist_modes → [ride latch] → assist_extended_boost
                   → safety_cut → assist_limits (pedal and throttle limited separately)
                   → assist_start → gear preload → coast release
                   → assist_dynamics → motor_core
```

One pipeline. Release dynamics (`release_ms`) untouched; Extended Boost is not mixed into it.

## F. Persistent config

**`MotorParams_t` layout is byte-for-byte unchanged. No migration, no EEPROM version bump,
stored settings survive the update.**

This was deliberate. Any change to `sizeof(MotorParams_t)` fails the FW-023 stored-record
length check and silently reverts every setting on the bike to defaults at the next boot. Only
`MotorState_t` fields were removed, and that struct is never written to flash.

## G. Build

```
LINK EXIT: 0     text 95236   data 264   bss 7080
```

13 warnings, **all pre-existing** in untouched code (12× pointer signedness in `CAN_Display.c`,
1× unused `fw_ver` under `#ifdef`). None introduced. No unreachable branches or unused
variables left behind by the removal.

All 16 host tests pass. Two needed fixing because they anchored on deleted lines:

- **FW-078** matched `if(MS.hall_angle_detect_flag>1)` and `return MS.i_q_setpoint_temp`.
  Re-anchored on the new function; the contract it checks is unchanged.
- **FW-084** asserted the bare token `NON_PEDAL` appeared in `assist_limits.c` — but that token
  only ever existed in the removed wrapper, so the check passed for the wrong reason and broke
  on a pure deletion. It now asserts the mechanism: the limiter branches on the request source,
  and the non-pedal arm is the 5..7 km/h taper.

## Test plan before this card is confirmed

1. Flash and read back 0x6028 / 0x6029 in the app — engine byte still parses, no field shift.
2. Confirm stored settings survived the update (banks, tuning, torque calibration, wheel code).
3. **Walk Assist**: standstill start, hold, brake pause, speed cut-off, release. Same feel.
4. **Position calibration**: run it to completion, confirm the angle is stored and the bridge
   goes off cleanly.
5. Normal ride A/B against the previous build: start, steady, easing off, stopping.
6. Level 0 still gives nothing; backpedalling still gives nothing.
