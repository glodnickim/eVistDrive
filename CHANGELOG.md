# Changelog

All notable changes to eVistDrive firmware, summarized per release. This is a concise,
English summary for the public repository — day-to-day development detail is kept in
local (untracked) notes.

## [Unreleased]

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
