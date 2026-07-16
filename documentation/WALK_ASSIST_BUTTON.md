# Walk Assist — physical button requirement

> **ARCHIWALNY** (2026-07-16): notatka historyczna galezi M820-walk-button. Aktualny opis dzialania: WALK_ASSIST_DZIALANIE.md.

This document describes the Walk Assist (push assist) behaviour on the `M820-walk-button`
branch. The branch is based on the accepted PR #1 (`mdi-9` → `M820`, walk-assist params by
Arkadiusz Goleń) and adds a single safety change on top: **Walk Assist only runs while a
physical button is held**.

## Update on `experiment/tsdz-experiment`

Walk Assist no longer uses wheel speed as a hard `pushassist_flag` gate. The
previous speed gate made the motor accelerate to the target, cut off, slow down,
and re-engage. On this branch, Walk Assist remains active while the CAN request
and physical button are held; `update_setpoint()` regulates motor current toward
`MP.walk_assist_speed`.

The current PI loop uses:

- `WA_RAMP_TICKS` for a softer start current ceiling,
- `WA_KI_PERIOD_TICKS` so the integrator updates at 100 Hz instead of 4 kHz,
- `WA_OVERSPEED_MARGIN` to force zero current and clear the integrator above the
  target speed plus margin.

The motor control stays **open-loop** (the standard EBiCS walk assist). The experimental FOC
walk-assist work lives on `feat/walk-assist-foc` and is intentionally **not** part of this branch.

## Behaviour

Walk Assist activates only when **all** of these are true at the same time:

1. **CAN request** — the display sends the push-assist command (`receive_message.rx_data[1] == 6`),
   debounced over 3 samples. This sets `MS.walk_can_request`.
2. **Physical button held** — the "down" button on `PA4` (`adc_value[5]`) is pressed.
   Raw ADC is ~4095 when released and ~3300 when "down" is held, so the detection window is
   `WA_BUTTON_THRESHOLD_LOW (3000)` … `WA_BUTTON_THRESHOLD_HIGH (3700)`, debounced over
   `WA_BUTTON_DEBOUNCE (20)` control-loop ticks.
3. Wheel speed below the limit (`MS.Speedx100 < 700`, i.e. < 7 km/h).
4. Brake not active.
5. No error state.
6. Not blocked by the safety timeout (see below).

`pushassist_flag` is derived from these conditions **exclusively in `main.c`**
(`reg_ADC_processing`). `CAN_Display.c` no longer sets `pushassist_flag` directly — it only
sets `walk_can_request`. Releasing the button (or losing the CAN request) stops Walk Assist
immediately.

## Configurable target speed and current

Inherited from the accepted PR, configurable from the display (no recompile needed):

| Parameter             | Source field        | Default (fallback) |
|-----------------------|---------------------|--------------------|
| `walk_assist_speed`   | `Para1[60]/[61]`    | `600` = 6.0 km/h   |
| `walk_assist_current` | `Para1[36]`         | `30` %             |

Setpoint (open-loop), in `update_setpoint()`:

```c
MS.i_q_setpoint_temp = map(MS.Speedx100,
                           (int32_t)MP.walk_assist_speed - 200, MP.walk_assist_speed,
                           MP.phase_current_max * MP.walk_assist_current / 100, 0);
```

So the current tapers to 0 as wheel speed approaches `walk_assist_speed` — Walk Assist holds
roughly the configured target speed (6 km/h by default).

## Safety timeout

If Walk Assist stays active continuously for `WA_TIMEOUT_MS (10000 ms)` it is blocked
(`ui8_WA_blocked`). The block is released only after the button is released **and** the CAN
request is gone. This guards against a stuck button / stuck CAN command.

## Relevant files

- `inc/config.h` — `WA_BUTTON_THRESHOLD_LOW/HIGH`, `WA_BUTTON_DEBOUNCE`, `WA_TIMEOUT_MS/TICKS`.
- `inc/main.h` — `MotorState_t.walk_can_request`.
- `src/main.c` — button debounce + gating block in `reg_ADC_processing`; derives `pushassist_flag`.
- `src/CAN_Display.c` — CAN command sets `walk_can_request` (debounced), not `pushassist_flag`.

## Note (before production)

`SYSTEM_VOLTAGE` is set to `40` (11S) for development/testing on this branch. Restore the
production value (52 V, or the correct value for the target pack) before release.
