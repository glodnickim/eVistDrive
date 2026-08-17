# Changelog

All notable changes to eVistDrive firmware, summarized per release. This is a concise,
English summary for the public repository — day-to-day development detail is kept in
local (untracked) notes.

## [Unreleased]

### Calmer high assist levels: per-level dynamics defaults
- All five levels previously shared almost the same assist dynamics. A higher assist ratio
  multiplies the same rider torque change into a much larger motor-torque request, which made
  L4/L5 noticeably more nervous than the lower levels for the same riding input.
- The compiled-in factory defaults now deliberately slow the rise/fall dynamics as assist
  rises, so the reaction stays calm at high power and high cadence. Only the *default values* of
  the existing per-level parameters changed — the `assist_dynamics` algorithm is untouched.

  | Level | Assist | Power rise | Power fall | Iq rise slow | Iq rise fast | Iq fall slow | Iq fall fast | Release |
  |---|---|---|---|---|---|---|---|---|
  | L1 | 100% | 150 ms | 375 ms | 600 ms | 300 ms | 1000 ms | 180 ms | 650 ms |
  | L2 | 200% | 160 ms | 400 ms | 600 ms | 330 ms | 1000 ms | 210 ms | 650 ms |
  | L3 | 320% | 190 ms | 450 ms | 650 ms | 380 ms | 1050 ms | 250 ms | 650 ms |
  | L4 | 420% | 220 ms | 500 ms | 700 ms | 450 ms | 1100 ms | 300 ms | 650 ms |
  | L5 | 520% | 250 ms | 550 ms | 750 ms | 500 ms | 1200 ms | 350 ms | 650 ms |

- `release_ms` stays 650 ms on every level. The change is factory/default only: stored user
  banks are never overwritten — the bank blob stays v8 with the same record layout, no EEPROM or
  CAN change — and the new values apply to factory reset, creating a new default bank, or
  restoring defaults. The Canable UI placeholder/defaults were synced to the same values.
- Not touched: the `assist_dynamics` algorithm, torque filtering, cadence compensation, safety/
  hard cut, release behaviour.

### Adaptive Iq ramp cadence range for the M820
- The SLOW→FAST interpolation of the adaptive Iq ramp used 20–70 rpm. On the M820 that range
  sat too low: from 70 rpm the whole normal riding cadence already ran fully on the FAST
  characteristic.
- The compile-time thresholds are now 50–110 rpm, so FAST builds up across the cadence band the
  M820 actually uses: 50 rpm = 0% FAST (full SLOW), ~17% at 60, ~33% at 70, ~50% at 80, ~67% at
  90, ~83% at 100, 100% at 110 rpm. Only the input bounds changed — the interpolation math, the
  per-level Iq rise/fall values and every filter are untouched.
- The protocol schema metadata (`ramp_cadence_low_rpm` / `ramp_cadence_high_rpm`) was synced to
  the new defaults (50/110); allowed ranges, types and layout are unchanged.
- The 50/110 bounds are tuned for the M820. TODO: this cadence ramp range belongs in the
  motor-specific profile — future EVistDrive motors may have a different usable cadence band.
- Not touched: Iq rise/fall values, power filters, torque filter, `release_ms`, S-curve,
  PAS/pre-stop, safety/hard cut, CAN and EEPROM.

### Build after these changes
- Firmware builds successfully as `0.0332_M820_BL820.bin`. The existing warnings
  (`-Wpointer-sign` in `CAN_Display.c`, unused variable `fw_ver` in `main.c`) predate these
  changes.

### Extended Boost may no longer outlive real pedalling — FW-095
- Extended Boost used to be motor overrun after the cranks stop: a firm push armed it, the
  EDGE of pedalling stopping started it, and it then held motor current for up to a second
  with the cranks stationary — while raising the profile's "pedalling" flag so the release
  fade would not run. On a bike with no dependable brake-sensor input there was no independent
  way to stop that. It was off by default and had never been confirmed on the bike, so it has
  been changed rather than tuned.
- It now does the opposite. A hard push, held for 30 ms while the rider is genuinely still
  pedalling forward and the ride latch is armed, starts the boost immediately; it runs for the
  configured time and ends on whichever comes first — the timer, or real forward pedalling
  stopping. The pedalling-stopped cancel is unconditional and acts in the same control tick.
  Nothing in the firmware may claim pedalling that is not happening any more: the flag the
  module used to raise for this is gone from the code.
- One push gives one boost. Leaning on the pedal cannot chain one boost into the next — the
  load has to fall back below the trigger before another may start.
- Duration stays 0–1000 ms in real milliseconds, and stays off by default. The range was
  deliberately not widened in the same step as the semantics change, so the first bike test
  cannot be ambiguous about which one caused what.
- Consequence to be aware of: while a boost runs the request is no longer forced to the
  no-pedalling speed classification. That override existed because the cranks were stopped,
  which is no longer true of any tick a boost can run in, so in legal mode the boost now
  follows the normal pedalling speed limit instead of the 5–7 km/h taper.
- Separately, the hard cut is now structural rather than incidental. Brake, backward
  pedalling, critical overtemperature, a torque-sensor fault and a running load calibration
  force the demand to zero and fade it over a fixed, firmware-owned 200 ms bound that an
  assertion keeps short — never over the rider-configurable per-level release time, which
  continues to serve the normal end of assist. Only overcurrent still kills the bridge outright.
- The persistent record size is now pinned by an assertion, so the layout that keeps every
  stored setting alive cannot be changed by accident.

### One ride engine: the pre-ride-core assist path is gone — FW-094
- Engine selection had been removed in FW-030, but the old assist monolith itself was still
  running: Walk Assist and phase 2 of position calibration called it on every control tick. It
  computed its entire pre-ride-core assist body — cadence map, pressure floor, throttle
  override, smooth-start envelope, overrun — and then discarded the result, because both of
  those paths overwrote it. The old arithmetic executed continuously and could not reach the
  motor. It has been deleted.
- The two paths that did reach the motor now have their own named functions in a new motor
  layer (`motor_service.h`): Walk Assist, and position calibration. Both do exactly what the
  reachable branches did, in the same order. The one intentional difference is that the Walk
  result is clamped rather than implicitly truncated, which cannot change a value in the
  reachable range.
- Also removed: the ride-engine type and its getter, the overrun state
  (`Overrun_strength`/`_counter`/`_flag`) and its per-level duration/strength cache, the legacy
  limiter wrapper, `map_rezi()`, `interpolate_assistfactor()`, and the build switches
  `ASSIST_TORQUE_MODE`, `ASSIST_CURVE_EXPO_L*`, `SMOOTH_START_ENABLE`, `START_RAMP_TICKS`,
  `EXTENDED_BOOST_ENABLE`, `RIDE_ENGINE_DEFAULT`, `TQ_GATE_RELEASE`, `START_MIN_STEPS`. No
  compatibility layer and no fallback were left in their place.
- **Stored settings are unaffected.** The persistent parameter layout is byte-for-byte
  unchanged, so no EEPROM migration is needed and nothing on the bike reverts to defaults. The
  parameters that the removed code used to read are now orphaned — still written by the app and
  echoed back by the firmware, but read by nobody. The Canable help text for each of them now
  says so plainly instead of claiming they are "compiled in but not reached".
- The engine byte in the 0x6028 and 0x6029 blocks stays on the wire as a documented constant:
  the shipped app parses those blocks positionally, so dropping it would shift every field
  after it. Assist mode 0 is likewise reserved rather than reused.
- Side effect worth having: the shared limiter no longer depends on this controller's globals,
  so it is genuinely motor-agnostic.
- Not addressed here: FW-084 Extended Boost still holds torque after the cranks stop on a bike
  without a brake sensor. It is native to the ride core, off by default, and unconfirmed on the
  bike — see `documentation/FW-094_LEGACY_REMOVAL_AUDIT.md`.

### Extended Boost: a deliberate drive hold for steps and rocks — FW-084
- New per-level setting group. The rider arms it with a firm push on the pedal; once the
  cranks are recognized as stopped the motor keeps pulling for a configured time at a current
  derived from the peak load of that push, and the existing single release ramp takes over
  afterwards. It exists for lifting over steps, rocks and short breaks in pedalling on a
  technical climb.
- Three settings per level, per bank: trigger pedal load (1.0–60.0 kg in 0.5 kg steps,
  default 20.0), boost strength (0–255 %, default 100) and boost duration (0–1000 ms, default
  **0 = off**). Every new and every migrated profile has it switched off, and with duration 0
  the current trajectory is identical to firmware without this card.
- The level's own ceiling still applies. Because the boost REPLACES the mode's result, both
  Maximum motor current and Maximum motor power are re-applied to it afterwards; without that
  a level limited to 20 % could have been handed the full global limit by one hard push.
- While the boost runs the request is classified as non-pedal — the cranks are stopped — so in
  legal mode it tapers from 5 km/h and gives nothing from 7 km/h. That is a deliberate policy
  written out in the code, not a side effect, and the Canable card says so.
- Writing a profile bank cancels any arming made under the previous settings.
- Deliberately narrow triggering: the load must be *held* for about 30 ms, so an ADC glitch,
  a chain slap or a pothole cannot arm it; a rate of rise alone never arms it; the arming
  goes stale after 1.5 s; and the latest confirmed push replaces an earlier one even when it
  is weaker. Boost starts only on the edge of pedalling stopping, and nothing — no PAS, Hall
  or speed pulse, no limit trimming the current — extends the timer once it runs.
- Brake, backward cranks, a sensor fault, assist level 0, a level or bank change, Walk Assist,
  position calibration, losing motion and resuming pedalling all cancel it in the same control
  tick. Throttle can neither arm it nor feed it: the module acts on the pedal-only target,
  before the throttle floor, and every shared limit still runs after it.
- Bank blob v8: the record grows 46 → 48 B and the blob to 255 B, which is exactly the ceiling
  of the transport (the length travels in a single byte). Older blobs are still accepted and
  migrate with the function off. The next per-level field will need an existing byte reused or
  a new transport version.
- Ride diagnostics 0x6029 extended to v5 with the boost state, latest peak load, computed
  current, remaining time and the reason the last one was cancelled.

### Releasing assist now really lets the motor coast — FW-093
- Zero torque was not a coast. Once the assist target reached zero the half bridges stayed
  enabled and the FOC went on regulating the *measured* current to zero, which on a turning
  rotor behaves as electrical damping — and pulls the gearbox into its last position at the
  very end. The bridge was released only after about three seconds without rotor movement,
  and the Hall interrupt resets that timer on every half rotation, so for as long as the
  motor turned the bridge never let go. Releasing Walk Assist and then turning the motor
  backwards by hand was where it was easiest to feel.
- The power stage now has an explicit DRIVE / COAST state shared by every torque source —
  Torque, Walk Assist, Power and Power Curve, throttle and Extended Boost — because they all
  end in the same current target. When that target reaches zero the controller waits only for
  the real current to decay (about 6 ms, with a 50 ms ceiling) and then switches the bridge
  off for a true high-impedance coast. No module switches the MOSFETs by itself.
- "No torque requested", "the bridge is released" and "the rotor has stopped" are three
  separate states in the code now. The rotor-stopped timer keeps its own job — cutting a
  bridge that is driving into a motor that will not turn — but no longer decides when zero
  torque may become a coast.
- Re-engaging while the motor is still spinning is handled explicitly. Switching the bridge
  on with zero applied voltage would put the full back-EMF across a shorted winding for
  several milliseconds of hard regenerative braking, so the current regulator is pre-loaded
  with the back-EMF measured as the coast began, scaled to the speed assist resumes at, and
  the outputs are enabled only once the first real switching pattern has been computed.
  Falling back to the previous zero start is still what happens from a standstill.
- A second cause of the same symptom was removed with it: the current regulator's integral
  term was being wiped on every control tick while the target was zero. The regulator runs
  four times faster than that, and with the shipped gains the integral could only ever reach
  about 3 % of the proportional term before being cleared — so at zero target the current
  loop was effectively proportional-only. Holding zero current on a turning rotor requires
  the loop to produce the back-EMF at zero error, which a proportional-only loop cannot do,
  so a real braking current kept flowing. The integral is now left to work while the bridge
  still drives; "a zero request makes no torque" is guaranteed by releasing the bridge
  instead, which is stronger, and both regulators are cleared as part of that release.
- Whether the bridge may drive is decided on the whole current command, not on the torque
  axis alone, so a future d-axis use cannot request current from a released bridge.
- The current release ramp, the bumpless bridge-on, Hall angle tracking and every safety
  shutdown are unchanged. Overcurrent, self power-off and position calibration keep their own
  immediate, unconditional cuts: this covers ordinary release only.
- Diagnostics frame 0x00010207 reports each power-stage transition — sent on the change, not
  in the control loop.

### Smooth Start no longer mistakes coasting for a standstill — FW-092
- The standstill test looked only at cadence and motor speed. On a mid-drive the freewheel
  lets the motor stand still while the bike rolls, so ordinary coasting satisfied it and every
  mid-ride re-engage armed the launch envelope: with a 300 ms setting the first current
  arrived about 22 ms late at the default latch floor, and later still at smaller targets.
- The test now also requires the wheel to be stopped. An armed-but-unspent envelope is also
  cancelled once the bike rolls without any demand: arming used to be cleared only when an
  envelope completed, so standing still and then rolling away without pedalling — a push
  off, a downhill start — left the arming lying in wait for the first pedal stroke at speed.
  An envelope already running under real demand still finishes normally.
- Launching from a real standstill is eased in exactly as before; only coasting stops
  counting as a launch. Smooth Start is off by default, so this affected configured bikes
  rather than factory settings.

### Re-engaging assist while riding no longer waits on a cadence filter — FW-091
- The limiter decided whether a request was "the rider pedalling" from filtered cadence, and
  anything at or below 15 rpm was treated as non-pedal and clamped to 5–7 km/h — a hard zero
  at riding speed. That filter is zeroed on every pedal stop and rebuilds exponentially, so
  after a lull assist was blocked for 45° of crank at 60 rpm, 180° at 20 rpm, and **never**
  below 16 rpm, where the filter converges below the threshold. It explains both the delay
  and why it was never the same twice.
- The limiter now classifies the *source* of a request. Confirmed pedalling — the ride latch,
  which already requires forward crank direction, the configured PAS step count and pedal load
  over the configured kg threshold — gets the normal speed limit. Throttle and without-rotation
  launches keep the low non-pedal limit.
- Pedal and throttle currents are limited separately and combined only afterwards. Sharing one
  limiter would have let a latched rider hand the throttle the full pedal speed limit; separate
  calls make that impossible regardless of latch state. Walk Assist is unchanged.
- The latch current floor now rounds up, so a small percentage of a small limit cannot vanish
  in integer division. Minimum Iq = 0% still means no floor.
- The FW-090 fast-attack experiment ships disabled: it addressed a secondary effect, and the
  behaviour should be judged without it first.

### RUN torque estimator follows a genuine rise immediately — FW-090
- Averaging pedal effort over half a crank turn (FW-085) removed the per-leg pulsing, but it
  also delayed a genuine *increase* in effort by the same half turn. Re-catching assist after
  the power faded mid-ride became a lottery: with recent samples still in the window a touch
  was enough, but after coasting the window held near-zero samples and the rider had to push
  through roughly 180° before the motor responded. It was more noticeable at low assist
  levels, because the shortfall is multiplied by the support ratio.
- A sustained rise now re-seeds the window, the same way arming the ride latch already did.
  Both trigger conditions are set so ordinary pedalling can never satisfy them — a leg push
  peaks at about 1.57× its own mean, well under the 2× margin, and no single peak lasts the
  required eight crank steps. Falling effort is untouched, so dips between leg pushes are
  still ridden out.

### The configured start load is now the only pressure condition — FW-089
- Declaring a start also required a raw-ADC pressure threshold sitting 29 counts above the
  sensor zero — roughly 1.19 kg on the default characteristic and 1.53 kg after a user
  calibration, and it moved whenever the sensor was recalibrated. The rider's own
  configured start load is 0.70 kg standing and 0.30 kg rolling, so a push anywhere in
  between cleared the threshold the rider had set and was still refused by a constant they
  could neither see nor change. Assist then waited for the first cadence measurement, one
  crank step later than it should have engaged.
- The start phase now depends on forward crank movement alone. Pressure is not lost as a
  condition — it lives where it belongs, in the configurable kg threshold that still has to
  be met before the ride latch arms and before any current flows. Crank rocking remains
  blocked by the consecutive-forward-step counter, which any reverse step resets.

### Standing starts no longer read as "no effort" on Progressive and Curve — FW-088
- Rider power is pedal load times crank speed, so pulling away from a stop is close to zero
  watts however hard the pedal is pushed. The support curve took that power as its input,
  mapped it to the bottom of the curve, and returned the minimum support ratio — the least
  help exactly where a standing start needs the most. Power Linear was unaffected (its
  ratio is a constant), which is why only some levels felt weak pulling away.
- During the launch phase the curve input alone is now evaluated at a nominal cadence, so a
  given pedal load earns the same support ratio starting as it does riding. Harder pushes
  still earn more than light ones, and the support window still caps the result. Reported
  rider power, motor power and every limit continue to use the real (near-zero) figure, so
  nothing on the display or in the power ceiling is inflated.

### The launch state is an explicit flag, not a fake cadence — FW-087
- Once pedalling clearly started but no cadence had been measured yet, the firmware wrote a
  placeholder 1 rpm into the cadence and set a companion flag. No assist calculation ever
  read that 1 — every consumer substitutes zero while the flag is up — so it existed only
  to get past two gates while presenting itself as a measurement. It gave the cadence two
  meanings, put a fake value on the display and in CAN telemetry, and made the whole launch
  protection collapse whenever anything cleared the flag.
- The launch state is now a plain boolean, and the cadence only ever holds real
  measurements or a clean zero. The two gates it used to slip through — the assist gate and
  forward-pedalling detection — ask the flag directly, so a bad measurement can no longer
  defeat them. Assist behaviour is unchanged; during launch the display now shows 0 rpm
  rather than a fabricated 1.

### Assist no longer stalls part-way into a start — FW-086
- The interval counter behind the cadence measurement was reset only when a cadence pulse
  fired; the pedal-stop branch never touched it, so it kept counting through the whole
  standstill and saturated. The first pulse after pulling away then computed 0 rpm from
  that stale interval and cleared the launch flag with it — which both cut assist (a zero
  cadence is rejected downstream) and prematurely enabled the power-derived current
  ceiling that is deliberately bypassed at launch. Assist only recovered on the second
  pulse, i.e. after roughly twice the crank movement the start setting asks for.
- The first forward step after a stop now starts a fresh interval, and is treated as that
  interval's origin rather than its first count, so the next pulse spans a full interval
  and publishes a true cadence.

### RUN torque smoothing now measured in crank angle — FW-085
- The RUN effort estimator averaged pedal load over a window set in milliseconds, while
  its purpose — as its own source comment stated — was to average over a fraction of a
  crank turn. Those are only the same thing at one cadence: 300 ms covered 45% of a
  revolution at 90 rpm but 25% at 50 rpm, so assist visibly pulsed once per leg on steep
  climbs, and no single millisecond value could be right across the cadence range.
- The estimator is now a moving average over a window of **crank degrees**, advanced by the
  quadrature decoder rather than by the control loop. Ripple rejection is therefore the same
  at every cadence, and response gets quicker as cadence rises instead of staying fixed.
  A window equal to a whole number of leg periods cancels the per-leg ripple outright.
- Tuning blob v7. Layout is identical to v6; only the unit of the field at offset 20
  changes (milliseconds → crank degrees), so v6 blobs are still accepted. The stored
  millisecond value is deliberately **not** converted — what it was worth depended on the
  cadence it was tuned at — so anything other than "off" migrates to the 180° default.
- Canable: the Dynamics field is now 0–360° in 15° steps (default 180°, one leg), and the
  writer negotiates down to v6/v5 for older controllers.

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
