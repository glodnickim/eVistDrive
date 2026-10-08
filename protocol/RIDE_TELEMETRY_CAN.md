# EVistDrive continuous Level-4 ride telemetry over CAN

## Schema versions

| Schema | Data frames | Introduced with | What changed |
|---|---|---|---|
| 1 | 7 (`0x10400..0x10406`) | FW145 | the original block |
| 2 | 9 (`0x10400..0x10406`, `0x10408`, `0x10409`) | Assist Pipeline V2 | CORE bytes 4..7 changed meaning; the STATE spare byte carries limiter flags; two frames added |

Assist V3 has an optional eight-frame extension (`0x1040A..0x10411`). Its schema 3
adds carry and motion observations and gives `rate_mode` four bits. It does not
change the nine base schema-2 frames; a base-only decoder may ignore the V3 IDs.

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
- A snapshot is seven data frames under schema 1, nine under schema 2, or seventeen with the
  optional V3 schema-3 extension; every data frame in
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
0x0001040A V3A        optional Assist V3 extension
0x0001040B V3B        optional Assist V3 extension
0x0001040C V3C        optional Assist V3 extension
0x0001040D V3D        final request / backstop / CPU max
0x0001040E V3E        dropped ticks / CPU last / backstop state
0x0001040F V3F        carry score / state / motion quality
0x00010410 V3G        remaining caps / estimated speed
0x00010411 V3H        relative acceleration
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

**Battery-current bits since TASK-EVD-TQ-06-G1 / ADR-013** (`flags16` bit 0, `limit_flags` bit 1,
the FW-028 diag `0x00010204` `flags2` bit 6, and DG byte 34 bit 0 / byte 44 bit 1). The wire layout
is unchanged. The bit means "a battery-current limiter is acting":
- on the PEDAL path: the ported G53 PI #1, `g1 < 4096`. G53 also trims by up to 6% when the current is
  within 5 A below the limit (P offset 500). Expect the bit to be set during the ~0.2 s soft start after
  power-on, and permanently when the configured limit is below the 5 A knee.
- on Walk: the M820 ap2 battery stage, as before.

The hard battery-overcurrent trip is reported separately (DG v9 byte 34 bit 1, bytes 73..74).

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

### 0x1040A..0x1040C Assist V3 extension (historical schema 1)

This optional group is appended after RIDER in a diagnostic build with `ASSIST_V3=1` and
`ASSIST_V3_SHADOW_TELEMETRY=1`. Phase 1 computes V3 in shadow; G53 still drives the motor. A
normal build emits none of these frames. All three carry the snapshot's `tick16` and use
big-endian signed or unsigned 16-bit fields as indicated.

```text
0x1040A V3A: 0..1 tick16 | 2..3 intent u16 (CLU) | 4..5 env_equiv u16 | 6..7 V3 demand Iq i16
0x1040B V3B: 0..1 tick16 | 2..3 base target Iq i16 | 4..5 trajectory target Iq i16 |
              6..7 short-window expected effort u16 (CLU)
0x1040C V3C: 0..1 tick16 | 2..3 load-domain kappa Q12 u16 |
              4 template confidence Q12 >> 4, saturated to u8 | 5 relative crank phase 0..95 |
              6 bits 0..2 release class, 3..5 trajectory rate mode, 6..7 V3 schema (=1) |
              7 flags (bit 0 phase aligned, 1 template mode, 2 engaged, 3 crank stopped,
                       4 stop target zero, 5 IMU valid, 6 engine active, 7 engine requested)
```

The legacy schema-2 decoder ignores the extension, retaining the same replay columns and
completeness criteria. The separate V3 schema allows its payload to evolve independently.

## 0x10407 META

### Assist V3 extension schema 3 (Milestone D)

The optional V3 group now spans `0x1040A..0x10411` (eight data frames). The
first five frames retain their schema-2 fields, except V3C byte 6: bits 0..2
are release class, bits 3..6 are the full four-bit `rate_mode`, and bit 7 is a
schema-3 marker. The full schema number is V3F byte 2. V3 rate modes 8 and 9
are carry and carry release. Existing CORE through RIDER bytes and META ID are
unchanged. A capture without V3 data still has nine data frames.

```text
0x1040F V3F: 0..1 tick16 | 2 schema (=3) | 3 carry_state (0 idle, 1 carry,
              2 release) | 4..5 carry_score Q12 | 6 cancel reason
              (0 none, 1 native cut/assist off, 2 reverse, 3 acceleration,
               4 time cap, 5 distance cap, 6 restart) | 7 motion quality
              (0 unknown, 1 wheel, 2 learned motor ratio)
0x10410 V3G: 0..1 tick16 | 2..3 carry remaining ms | 4..5 carry remaining cm |
              6..7 estimated speed in 0.01 km/h
0x10411 V3H: 0..1 tick16 | 2..3 relative acceleration, signed permille/s |
              4..7 reserved zero
```

The CANable decoder adds these V3 observations to decoded CSV when the V3F
schema byte is 3. Replay input remains the same sensor history.

At most once per second:

```text
byte 0     telemetry schema version (1 or 2)
byte 1     active assist-profile bank
byte 2..5  full 32-bit 4 kHz control tick
byte 6     failed telemetry-frame counter, saturated to 255
byte 7     number of data frames per snapshot (7 for schema 1, 9 for schema 2,
           12 for V3 schema 1, 14 for V3 schema 2, 17 for V3 schema 3)
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
