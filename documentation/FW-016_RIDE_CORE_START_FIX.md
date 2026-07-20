# FW-016 — Ride Core start and torque-path fix

## Implemented

- Ride Core consumes its own 35 ms, 4 kHz torque filter. Legacy still consumes
  `MS.torque_filtered` and its per-level `TQfilter`.
- The filter runs continuously and is not cleared on a PAS stop. Ride Core is
  armed only after `START_MIN_STEPS` forward transitions.
- The temporary cadence seed arms assist, but startup boost and human-power
  calculations treat it as 0 rpm until a real cadence sample arrives.
- Power, eMTB and Torque modes produce phase `Iq` from rider load/torque.
  `P/U` remains a battery-current calculation and becomes a phase-current
  ceiling through measured `MS.u_abs` duty (0..2048).
- Ride Core uses its profile `max_iq_pct` and the global phase/limp limit. It no
  longer inherits the Legacy per-level current percentage or `TQfilter`.
- eMTB and Torque calculations retain the 0..160 target in Q8 until conversion
  to native `Iq`, preventing small valid inputs from truncating to zero.
- Human power uses calibrated load, 165 mm crank length and measured cadence.
- Default load conversion is usable without calibration and passes through the
  measured points 740 mV = 0 kg, 886 mV = 6 kg and 2320 mV = 84 kg. Explicit
  user load calibration remains optional.

## Safety invariants

- Legacy remains the compiled default (`RIDE_ENGINE_DEFAULT=0`).
- FOC code is unchanged.
- Brake, reverse PAS, torque Error 25, active torque calibration and thermal
  cutoff force the Ride Core command immediately to zero.
- Existing phase-current, battery-current, voltage, legal-speed and thermal
  limits remain in the command path.
- Start torque has no fixed current floor. It is proportional to measured load
  and is passed through the existing adaptive `Iq` ramp.
- Position-sensor calibration has priority over both ride engines and bypasses
  assist dynamics. A persisted TSDZ selection cannot suppress its second phase,
  and the completion command `Iq=0` cannot be replaced by a stale ramp value.

## Verification

- `tests/fw016_ride_core_model.ps1` checks calibration points, monotonicity,
  filter response and non-zero bounded eMTB start/running requests.
- Hardware acceptance must begin with the drive wheel unloaded/off the ground.
  Ground testing follows only after brake and reverse-PAS cuts are confirmed.
