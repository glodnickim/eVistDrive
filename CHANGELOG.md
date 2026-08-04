# Changelog

All notable changes to eVistDrive firmware, summarized per release. This is a concise,
English summary for the public repository — day-to-day development detail is kept in
local (untracked) notes.

## [Unreleased]

### Walk Assist controller — FW-077 through FW-082
- Start condition thresholds are now expressed directly in kg (per-level standstill and
  rolling minimums), with the old mV-based pressure-rise detector removed.
- Hall sensor autocalibration for Walk Assist.
- The Walk Assist speed controller now regulates to the configured target chainring
  RPM instead of drifting to whatever the drivetrain settles at.
- Walk Assist recovers from coasting to zero without needing the button released and
  re-pressed.
- Added a Hall keepalive current floor and capped the initial start current, fixing a
  slip/reacquire/stall cycle seen on light drivetrains.
- Faster RUN response and a wider normal operating current range, based on real-world
  testing.

### Pedal-assist start sensitivity
- The ride-latch start gate compared a deadbanded and 35 ms filtered torque signal
  against the rider's configured kg threshold, which silently raised every threshold
  by roughly 0.4 kg. It now compares the already-computed raw kg reading directly.
- The required crank-rotation step count is eased by one step specifically while the
  bike is already rolling (never below zero) — resuming pedalling after any pause was
  otherwise treated exactly like a fresh standstill start.

### Housekeeping
- Renamed `protocol/ebics_config_schema.yaml` to `evistdrive_config_schema.yaml`,
  completing the eVistDrive rebrand (HMI and README already used the new name).
- Added `LICENSE` (GPL-3.0) and `QUICKSTART.md`.

All host-side tests pass. Bank blob and CAN protocol are unchanged for the Walk Assist
work; the start-sensitivity fix touches only runtime logic in `ride_control.c`, no wire
format changes.
