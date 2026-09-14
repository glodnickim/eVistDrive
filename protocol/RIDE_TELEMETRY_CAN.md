# EVistDrive continuous Level-4 ride telemetry over CAN

## Schema versions

| Schema | Data frames | Introduced with | What changed |
|---|---|---|---|
| 1 | 7 (`0x10400..0x10406`) | FW145 | the original block |
| 2 | 9 (`0x10400..0x10406`, `0x10408`, `0x10409`) | Assist Pipeline V2 | CORE bytes 4..7 changed meaning; the STATE spare byte carries limiter flags; two frames added |

**Schema 2 added its frames ABOVE META, not in place of it.** `0x10407` stays META and every
schema-1 frame keeps the identifier its decoder already knows, so a decoder that understands only
schema 1 sees a version byte of 2 and can stop cleanly instead of misreading. `tools/decode_canable_ride_log.py`
decodes both and records which one each capture used.

The one field that changed meaning without changing position is CORE bytes 4..7. Under schema 1
they are the removed native torque filters; under schema 2 they are the demand model in permille.
The decoder keeps them in SEPARATE output columns for exactly that reason - a mixed archive must
never compare an ADC delta against a permille demand.

## Purpose

This is a **developer/diagnostic observation stream** for recording a real ride with an ordinary
CANable raw sniffer and replaying that ride in the FW144+ Level-4 environment. It is not a control
protocol and no receiver is allowed to influence motor control through these frames.

Default enable rule:

```text
normal build      CAN_DIAGNOSTICS_ENABLE=0  -> stream OFF
diagnostic build  CAN_DIAGNOSTICS_ENABLE=1  -> stream ON
```

`CAN_RIDE_TELEMETRY_ENABLE` can explicitly disable the stream inside a diagnostic build, but it
cannot enable it when diagnostics are globally disabled.

## Scheduling / ownership

- FOC ISR remains 16 kHz and **does not transmit CAN**.
- QZERO publishes only a one-byte ISR-owned diagnostic mirror; foreground never reaches into the
  QZERO state-machine object.
- The foreground builds one coherent snapshot about every 84 control ticks (~21 ms / ~47.6 Hz).
- The main loop serializes at most one telemetry CAN frame every 12 control ticks (~3 ms).
- A snapshot is seven data frames under schema 1 and nine under schema 2; every data frame in
  it carries the same `tick16`, which is what identifies the snapshot.
- META is inserted at most once per second between complete snapshots.
- Critical CAN queue, HMI multiframe traffic and existing diagnostic dumps have priority.
- If no CAN mailbox is free, the telemetry step returns immediately. There is no busy-wait.
- A mailbox failure is counted and the stream advances; a failed old sample is not retry-flooded.

Nominal continuous stream load is about 333 data-frame starts/s plus <=1 META frame/s while a ride
session is active. The actual rate may be lower when critical CAN traffic has priority.

## Extended CAN ID block

The range is compile-time checked against all other EVistDrive diagnostic blocks by
`inc/diag_efid_map.h`.

```text
0x00010400 CORE
0x00010401 DEMAND
0x00010402 MOTOR
0x00010403 BATT
0x00010404 LIMITS
0x00010405 STATE
0x00010406 ROTOR/PAS
0x00010407 META
0x00010408 ASSIST     schema 2
0x00010409 RIDER      schema 2
```

`0x10300..0x10307` remains owned by STOP_TRACE and must not be reused.

All 16/32-bit numeric fields below are **big-endian** on the telemetry wire.

## Common data-frame prefix

Every data frame:

```text
byte 0..1  control_tick low 16 bits (4 kHz timebase)
```

The same value on all data frames of a snapshot identifies it.

### 0x10400 CORE

```text
0..1 tick16
2..3 load_centikg              calibrated pedal force, 0.01 kgf   (both schemas)

schema 1:
4..5 torque FAST native delta  removed assist path, native ADC units
6..7 torque RUN native delta   removed assist path, native ADC units

schema 2:
4..5 torque_normalized         permille of the rider-effort full scale
6..7 rider_demand              permille - the rider's intent after conditioning
```

### 0x10401 DEMAND

```text
0..1 tick16
2..3 Iq_requested  signed
4..5 Iq_allowed    signed
6..7 Iq_ref        signed
```

### 0x10402 MOTOR

```text
0..1 tick16
2..3 Iq_actual     signed
4..5 Id_actual     signed
6..7 motor_erps
```

### 0x10403 BATT

```text
0..1 tick16
2..3 battery voltage in 10 mV
4..5 battery current in 10 mA, signed
6..7 production SOC display in 0.1 %
```

### 0x10404 LIMITS

```text
0..1 tick16
2..3 wheel speed x100 km/h
4..5 u_abs
6..7 flags16
```

`flags16`:

```text
bit  0 battery current limiter active
bit  1 brake
bit  2 Walk
bit  3 torque fault
bit  4 overtemperature cut stage
bit  5 FOC/vector saturation observed
bit  6 torque-sensor calibration active
bit  7 offroad
bit  8 PWM on
bit  9 start phase
bit 10 Walk CAN request
bit 11 PAS direction inhibit
bit 12 confirmed backpedal
bits 13..15 bridge lifecycle (0..7)
```

### 0x10405 STATE

```text
0..1 tick16
2    ride permission bits
3    ride-control debug flags
4    bits 0..3 assist level
     bits 4..5 ride session state
     bits 6..7 QZERO state
5    raw cadence rpm
6    conditioned control cadence rpm
7    schema 1: reserved = 0
     schema 2: limiter flags, low 8 bits (see below)
```

`limit_flags` (schema 2, STATE byte 7) - which stage of the one limiter chain was binding:

```text
bit 0 power ceiling
bit 1 battery current
bit 2 phase / level Iq ceiling
bit 3 undervoltage derate
bit 4 thermal derate
bit 5 speed / legal taper
bit 6 a non-zero request was taken all the way to zero by a limit
bit 7 the start segment is in force
```

Byte 2 (`permission_bits`) and byte 3 (`debug_flags`) keep their positions but, under schema 2,
carry the assist chain's own answers: byte 2 is the lifecycle and profile
(low nibble `ap2_pas_state_t`, high nibble `ap2_profile_id_t`) and byte 3 is the
`AP2_WHY_*` reason bitfield.

### 0x10406 ROTOR/PAS

```text
0..1 tick16
2..3 theta_final high16 / q15 signed view
4..5 Hall age ticks
6    bits 0..2 Hall state
     bit  3 rotor-angle trusted
     bits 4..6 bridge lifecycle
     bit  7 PWM on
7    bits 0..1 PAS A/B snapshot
     bits 2..3 PAS direction-state enum
     bit  4 confirmed backpedal
     bit  5 direction inhibit
     bit  6 start phase
     bit  7 reserved
```

Important: the PAS A/B value in this ~48 Hz stream is a **state snapshot, not a raw quadrature
transition recorder**. It must not be presented as `PAS_RAW` evidence. Short high-resolution PAS /
START/STOP investigations still use the existing PAS/QS/STOP recorders.

### 0x10408 ASSIST (schema 2)

The request, split into the two halves the demand model produces. All permille.

```text
0..1 tick16
2..3 assist_base       the sustained term - what survives the pedal dead spot
4..5 assist_dynamic    the reactive term - the excess above that sustained level
6..7 assist_response   the combined response, before it is scaled to Iq
```

### 0x10409 RIDER (schema 2)

The two estimators and the adaptive decision. All permille, all confidences with no physical
dimension.

```text
0..1 tick16
2..3 rider_aggression  fast: how sharply the bike is being ridden
4..5 load_state        slow: how hard the bike is working (climb, headwind, soft ground)
6..7 auto_factor       where an adaptive profile currently sits between calm and strong
```

## 0x10407 META

At most once per second:

```text
byte 0     telemetry schema version (1 or 2)
byte 1     active assist-profile bank
byte 2..5  full 32-bit 4 kHz control tick
byte 6     failed telemetry-frame counter, saturated to 255
byte 7     number of data frames per snapshot (7 for schema 1, 9 for schema 2)
```

### Captures that begin before the first META

META is sent at most once a second, so a capture joined mid-ride has data frames before any
version is known. The decoder does not guess: it reads every snapshot under the version of the
nearest PRECEDING META, and snapshots before the first META take the version of the first one in
the capture. That is an inference and it is recorded as one - `schema_before_first_meta` in the
metadata says how many rows were read that way, so a reader can discount them if the capture
spans a firmware change.

The full tick is an epoch anchor. The raw CANable monotonic timestamp is retained independently by
the decoder, so missed telemetry frames remain real time gaps rather than compressed time.

## Existing CANable raw-log format

`tools/decode_canable_ride_log.py` directly accepts the format already produced by the current
CANable logger, for example:

```text
[08:52:10] [INFO] 539158290483 ID:83106302 DLC:5 Data:A7 00 60 4A 00
```

Unknown/non-FW145 CAN IDs remain untouched and are ignored by this decoder.

## Practical workflow

1. Build/flash the **diagnostic** firmware:

```text
VERIFY_AND_BUILD_DIAGNOSTIC_WINDOWS.bat
```

2. In CANable select `All Traffic`, enable raw logging, and record the ride.
3. Save e.g. `ride_problem_001.log`.
4. Decode:

```bash
python tools/decode_canable_ride_log.py ride_problem_001.log --output-prefix ride_problem_001
```

This writes:

```text
ride_problem_001.decoded.csv    all available internal observations
ride_problem_001.canonical.csv  Level-4 replay input
ride_problem_001.metadata.json  coverage/loss/schema report
```

5. Replay current code:

```bash
python tools/run_replay.py ride_problem_001.canonical.csv
```

6. After the hardware fault is reproduced, fixed and the new behavior is reviewed, register the raw
capture permanently:

```bash
python tools/register_canable_ride_case.py ride_problem_001.log BUG_001_NAME --accept-current
```

Never use `--accept-current` on a capture whose current replay still represents the known-bad
behavior merely to turn the regression green.
