# FW-095 — Extended Boost may not outlive real pedalling

**Status: implemented in code, NOT built for release, NOT ridden.**

Follows FW-094 (Legacy removal). Where FW-094 deleted the dead old engine, this card fixes the
one live safety problem the audit had flagged and left open.

## The problem

FW-084 Extended Boost was, in plain terms, motor overrun after the cranks stop.

- It was **armed** by a firm push while pedalling.
- It **started on the edge of pedalling stopping** — `previous_pedaling_active && !pedaling_active`.
- It then held motor current for up to 1000 ms with the cranks stationary.
- While it ran it raised the profile's "pedalling" flag (`profile_hold_active` →
  `profile_pedaling_active = true`) to suppress the release fade, i.e. it reported pedalling
  that was not happening.
- It cancelled if pedalling **resumed** — the opposite of what safety wants.

The M820 has no brake-sensor input we can depend on, so "keep pulling after the rider stops
pedalling" has no independent way to be stopped. The feature was off by default and had never
been confirmed on the bike, so it was changed rather than tuned.

## The rule now

> Extended Boost may only produce current while real forward pedalling is happening, and it may
> never claim that pedalling is happening when it is not.

Implemented as an unconditional cancel at the top of the module's cancel chain:

```c
} else if (!input->pedaling_active) {
        cancel = ASSIST_EXT_BOOST_CANCEL_PEDALING_STOPPED;
} else if (!input->pedal_assist_latched) {
        cancel = ASSIST_EXT_BOOST_CANCEL_PEDALING_STOPPED;
}
```

It is deliberately not "unless the timer is nearly done" and not "unless the load is still
high". If such a post-PAS overrun is ever wanted it must be its own explicitly reasoned safety
function, not a ride-feel setting. **This card does not implement one.**

## G. Extended Boost — full specification

| | |
|---|---|
| **Arming condition** | Merged into the start condition. There is no waiting state any more. |
| **Start condition** | Pedal load ≥ trigger, held continuously for `EXT_BOOST_CONFIRM_MS` (30 ms), while `pedaling_active` **and** the ride latch are true, the bike is moving (≥1.0 km/h and ≥10 motor ERPS), assist level ≠ 0, duration ≠ 0 and strength ≠ 0. Confirmation starts it in that tick. |
| **Active condition** | Every start condition is re-checked each 4 kHz tick. Current is `((iq_limit × (peak − trigger)) / (full_scale − trigger)) × strength%`, clamped to the level's own ceiling (max motor current and max motor power) and then to every shared limit. |
| **Stop condition** | Whichever comes first: the timer expires (`COMPLETED`), `pedaling_active` goes false (`PEDALING_STOPPED`), the ride latch drops (`PEDALING_STOPPED`), hard cut (`SAFETY_CUT`), reverse, sensor invalid, walk, calibration, level 0, level/bank change, bank rewrite, or motion lost. All act in the same control tick. |
| **Duration unit** | `uint16_t` real milliseconds, 0–1000, step 25 in the app. 0 = off, and off is the default. |
| **Strength unit** | Percent, 0–255, applied to the load-derived current. Never raises the result above the level's own current limit. |
| **Interaction with PAS STOP** | Ends it, unconditionally, in the same tick. This is the defining behaviour of the card. |
| **Interaction with hard cut** | Cancelled by it, and the boost is applied *before* the hard-cut block and both limiter calls, so brake/reverse/fault and every speed, power, voltage and temperature limit still have the last word. |
| **Re-arming** | One push gives one boost. A held push cannot chain into the next: after a boost ends the load must fall a further `EXT_BOOST_RELEASE_HYST_CENTIKG` (0.5 kg) below the trigger before another may start. |

### Duration range decision (option 1)

The ceiling **stays at 0–1000 ms**. The semantics of the feature just changed; widening the
range in the same step would make the first bike test ambiguous — a bad result could be the new
semantics or the longer time. It is a one-constant change once the behaviour is confirmed.

### Speed-limit classification changed — deliberately

FW-084 forced `ASSIST_LIMIT_SOURCE_NON_PEDAL` while a boost ran, and its stated reason was
correct for what it did: the cranks were stopped, so nothing was confirming pedalling. That
reason no longer holds for any tick a boost can run in. The override is removed and the boost
is classified from the ride latch, exactly like ordinary pedal assist.

**Consequence to be aware of:** in legal mode the boost is no longer capped by the 5–7 km/h
non-pedal taper. The normal legal speed limit still applies. Keeping the old override would
have applied a no-pedalling limit to a rider who is demonstrably pedalling.

## J. Safety — what sets Iq in each case

| Case | Detected as | Iq target | Fade to zero |
|---|---|---|---|
| Normal release | rider eases off / stops; mode result falls | mode's own result | **level's `release_ms`** (ride feel, up to 3000 ms) |
| PAS stop | `pedaling_active` false | boost → 0 immediately; assist follows the mode | level's `release_ms` |
| Brake | `hard_cut` | **forced 0**, latch dropped, throttle path skipped | `RIDE_HARD_CUT_RAMP_MS` (200 ms, firmware-owned) |
| Backward pedalling | `hard_cut` | forced 0 | `RIDE_HARD_CUT_RAMP_MS` |
| Torque-sensor fault (Error 25) | `hard_cut` | forced 0 | `RIDE_HARD_CUT_RAMP_MS` |
| Critical overtemperature | `hard_cut` | forced 0 | `RIDE_HARD_CUT_RAMP_MS` |
| Load calibration running | `hard_cut` | forced 0 | `RIDE_HARD_CUT_RAMP_MS` |
| Overcurrent | motor fault | bridge disabled outright in `FOC.c` | none — immediate |

### Why the hard cut is a 200 ms ramp and not a single-tick snap

It already was separate from the comfortable release before this card; FW-095 makes that
structural rather than incidental. `RIDE_HARD_CUT_RAMP_MS` is now a named constant with

```c
_Static_assert(RIDE_HARD_CUT_RAMP_MS <= 250,
        "the hard-cut ramp is a safety bound, not a comfort setting");
```

so it can never be sourced from the rider-configurable `level->release_ms`.

It is a ramp because the FOC current loop runs at 16 kHz behind it, and stepping its reference
from full current to zero produced a torque step the drivetrain took up as an audible clunk
(FW-037). It is safe because FW-093 means a zero Iq target releases the half bridges within a
few ms of the measured current reaching zero, so the ramp ends in a real Hi-Z coast; and during
the ramp the commanded current is monotonically falling and can never rise.

`hard_cut` is now a single named `const bool` at the top of `ride_control_update()`, and every
site that used to read `input->safety_cut` reads it instead.

## H. Walk Assist — call flow

```
main.c reg_ADC_processing
  → ride_control_update(input.walk_active = true)
      → assist_extended_boost_reset(CANCEL_WALK)
      → walk_assist_iq_request()            [motor_service.h, defined in main.c]
            brake / torque fault / load calibration → 0
            else walk_motor_update()        [walk_assist_motor.c + walk_speed_controller.c]
            → assist_limits_apply()         voltage + controller temperature only
      → assist_dynamics_apply()             returns the target unchanged while walk_active
      → motor_core_set_command()
```

No Legacy assist anywhere on this path.

## I. Hall calibration — call flow

```
main.c reg_ADC_processing
  → ride_control_update(input.position_calibration_active = true)
      → assist_extended_boost_reset(CANCEL_CALIBRATION)
      → hall_calibration_iq_request()       [motor_service.h, defined in main.c]
            probe current 100; trims MP.angle_correction
            on convergence: Iq 0, PI integrals cleared, PWM disabled,
                            pwm_stage = COAST, write_virtual_eeprom()
      → motor_core_set_command()            ride-feel ramp bypassed on purpose
```

No Legacy assist anywhere on this path.

## E. EEPROM

`sizeof(MotorParams_t)` is **unchanged at 724 bytes**, and now pinned:

```c
_Static_assert(sizeof(MotorParams_t) == 724,
        "persistent record size changed: every stored setting on every bike would reset");
```

The FW-023 validity check includes the record length, so the size is part of the persistent
format. This is why the orphaned Legacy parameters stay in the struct — removing them would
reset both profile banks, the ride-feel tuning, the torque calibration, the wheel code and the
full-charge voltage on every bike at the next boot. The assertion turns that from a comment
into a compile error.

No migration is required by this card.

## F. Protocol compatibility

| Field | Decision |
|---|---|
| 0x6029 byte 45 bits 0x02 / 0x08 | Kept in place, never set. They reported the removed waiting state and its stale-arming flag. |
| Cancel reasons 9 and 10 | Reserved, never reported. The app's table keeps them as `(reserved)` so every other index keeps its position. |
| Cancel reason 13 | New: `PEDALING_STOPPED`. |
| `assist_extended_boost_diag_t.arm_expired` | Kept in the struct (byte 53), always false. |
| 0x6028/0x6029 byte 3 (engine id) | Unchanged from FW-094: constant, documented as deprecated. |
| `Override_Duration`, `decay_base`, `TS_coeff`, `ext_boost_*[6]`, `assist_profile`, `TQO_threshold`, `ramp_end`, `MagicNumber` | Orphans. Still round-tripped through Para0/1/2 for the app; read by nothing. Removing them is a protocol version bump, out of scope. |

## K / L. Tests and build

See the session report. In short: firmware compiles and links clean (13 warnings, all
pre-existing in untouched code); all JS design tests pass, including the new
`tests/fw095_extended_boost_safety.js`; the C host harness
(`tests/host/fw095_extended_boost_host.c`) cross-compiles and links under
`-Wall -Wextra -Werror` but **could not be executed** — no host C compiler on this machine, so
the runner reports SKIPPED, not PASS.

## Test plan before this card is confirmed

1. Bench, wheel off the ground: confirm the boost fires on a hard push and that **letting the
   cranks stop kills it instantly** — watch `ext_boost_cancel_reason` read `stopped pedalling`.
2. Confirm the boost cannot be chained by leaning on the pedal.
3. Brake mid-boost: current must fall to zero within ~200 ms and the bridge go Hi-Z.
4. Backpedal mid-boost: same.
5. Ride: check the boost is useful on a step/root and does not feel like a lurch.
6. Confirm legal-mode speed behaviour matches the new classification.
7. Regression: Walk Assist, position calibration, normal start/stop unchanged.
