# EVistDrive — current architecture map

> The pedal-assist path is **Assist Pipeline V2**. Its blocks, units, state machines, parameters,
> limiters and telemetry are documented in [ASSIST_PIPELINE_V2.md](ASSIST_PIPELINE_V2.md); this
> file keeps the whole-firmware view around it.

## Control flow

```text
RAW SENSORS
  torque ADC / PAS A-B / Hall / wheel / battery / temperature
        |
        v
SENSOR TRUTH
  PAS plausibility -> direction
  conditioned cadence
  torque calibration + filtering
  rotor motion / angle confidence
        |
        v
RIDE PERMISSION / SESSION
        |
        v
RIDE PERMISSION / SELECTION
  ride_control.c: calibration | Walk Assist | pedal assist - one owner per tick
        |
        v
ASSIST PIPELINE V2
  rider demand -> base + dynamic -> aggression / load -> profile / AUTO
  -> characteristic -> start / attack / release
        |
        v
LIMITS (one chain, ap2_limits.c)
  power -> battery current -> phase current -> voltage -> thermal -> speed
        |
        v
FINAL Iq TRAJECTORY @ 16 kHz
  `fast_iq_slew.c`
        |
        v
CURRENT LOOP
  `FOC.c` + `foc_current_loop.c`
  Clarke/Park -> PI Id/Iq -> common vector saturation -> inverse Park -> SVPWM
        |
        v
PWM / INVERTER / MOTOR
```

## START lifecycle

Current intended normal start:

```text
valid forward rider intent
 -> permission
 -> final Iq state starts from zero
 -> continuous bounded rise
 -> motor moves
 -> Hall timing becomes confident
 -> normal ACTIVE
```

There is no normal Hall-gated Gear Preload current cap. Hall is feedback/evidence, not a condition that suddenly unlocks a second torque owner.

## ZERO / RESTART lifecycle

```text
ACTIVE
 -> demand goes to zero
 -> final Iq trajectory reaches zero
 -> PI/ADC/theta remain coherent
 -> ordinary zero torque / ARMED_ZERO semantics
 -> new valid demand
 -> warm restart without unnecessary cold bridge cycle
```

Fault shutdown is separate from ordinary zero torque.

## Hall/angle

3-Hall hardware cannot know exact electrical position at standstill. The safe model is:

- insufficient timing: sector anchor (bounded initial error);
- fresh timing: interpolation only with confidence;
- stale timing: fall back instead of extrapolating forever;
- angle confidence and speed confidence are separate facts;
- one final Park angle owner.

Electrical SIL explicitly sweeps all Hall sectors/start angles.

## Rider-input stability

### PAS
Impossible 1-3 tick reverse bounce is rejected before direction safety. True reverse remains a hard permission loss.

### Cadence
Control uses the conditioned cadence. Raw cadence is retained for telemetry/diagnostics.

### Torque
Filtering is elapsed-time based, so missed/coalesced 4 kHz foreground calls do not stretch a
nominal filter by call count. The assist path applies exactly ONE filter to the measurement
(20 ms, sensor noise only); pedal ripple is modelled by the base/dynamic split, not filtered away.

## Simulation layers

### Fast supervisory SIL
Cheap plant for 10k+ fuzz cases. Exercises real supervisory production path and final Iq logic.

### Electrical FOC SIL
Uses real production:

```text
final Iq slew
 -> FOC current measurement/Park
 -> PI Id/Iq + vector saturation
 -> inverse Park
 -> SVPWM
 -> virtual PMSM
 -> physical currents + rotor
 -> Hall edges
 -> production rotor angle/motion
```

QZERO also executes in this backend.

The PMSM parameters are test parameters, not claimed measured M820 constants. Do not tune production FOC/QZERO to the virtual plant unless hardware evidence confirms the parameter domain.

## Walk Assist

Walk has a separate demand owner but shares the downstream electrical FOC path. Its normal target domain is:

```text
10..60 chainring/output rpm
default 30 rpm
```

The Walk governor may float slightly around a low target under very small load; exact rpm tracking is not a
safety invariant. The verification hard-gates no-runaway (<80 rpm in the current electrical SIL contract),
correct current ceilings, safe stall, Hall/lifecycle behavior and true-zero safety exits. Out-of-range target
values such as 70/80 rpm are rejected/fallback inputs, never valid normal targets.

## User-facing functionality rule

Keep rich TSDZ-style configurability, but do not let each parameter create a new hidden controller/state machine.

Examples:

- Profile -> a whole behaviour (characteristic, base/dynamic mix, dynamics, power envelope)
- Assist trim -> how much effort is needed to reach full assist
- Power -> ceiling only; it may tighten the profile envelope, never widen it
- Attack -> how fast the demand may rise
- Release -> how fast it lets go, and the time to zero on a stop
- Start -> the rate in force for the first start_ms of a ride

Every user setting should have a single documented owner.

## Current known architectural debt

These are not authorization to refactor blindly:

1. Legacy/inactive configuration fields are still stored and round-tripped (`inc/assist_bank_wire.h`).
   They should eventually be reclaimed by a versioned transport change, not silently repurposed.
2. QZERO and the low-speed coast policy require hardware-backed A/B before removal or retuning.
3. `main.c` remains a large hardware/glue node; extract only when ownership is clear and parity is tested.
4. Every ride-feel number in the Assist Pipeline V2 profile table is a starting point, not a
   measurement - the pipeline has never driven the physical motor. The torque sensor scale
   (`CLAIM-EVD-TORQUE-SCALE-004`) dominates all of them and must be measured first.

## Definition of a flash candidate

A flash candidate requires all of:

```text
host modules PASS
+ whole-pipeline traces PASS
+ supervisory SIL PASS
+ electrical FOC/Hall SIL PASS
+ deterministic fuzz PASS
+ sanitizers PASS
+ exact Arm GCC 13.2.1 target build PASS
+ BL820 packaging/CRC/identity PASS
```


## Level 4 virtual bicycle and battery

The next layer closes physical rider, road and battery feedback around the same production controller:

```text
rider -> PAS/torque -> production controller -> real FOC/PMSM -> drivetrain/bike/grade -> sensors -> controller
battery truth/SOC/sag -> Vbus/current -> production limits + soc_core -> controller/display SOC
```

`sim/l4/` contains plant assumptions only. `src/soc_core.c` is production SOC math and is shared by target and
simulator. Real captures are normalized and replayed by `sim/replay/`; accepted hardware regressions become
persistent cases under `sim/replay/cases/`.

## FW145 diagnostic observation path

The Level-4 live logger is deliberately outside the control ownership graph:

```text
existing owners @ 4 kHz / 16 kHz
        |
        +--> ISR-only qzero diagnostic mirror
        |
        +--> coherent read-only snapshot ~48 Hz
                  |
                  v
           ride_telemetry.c
          best-effort CAN pacing
                  |
            0x10400..10407
                  |
             CANable raw log
                  |
        offline decoder / replay
```

No arrow from telemetry returns into rider demand, permission, Iq, FOC, theta, QZERO or SOC.
