# EVistDrive v3 — testable motor-control project

Shared CAN/BLE communication authority: [eVistDrive communication registry](../integration/contracts/COMMUNICATION_REGISTRY.md). Every configuration change must define its motor, HMI, CANable and mobile mapping, units, capability, result and persistence. Preserve factory frame semantics; consult the registry and its detailed contract before changing handlers.

This tree is based on `EvistDrive06092026v3.zip` and contains the production GD32/M820 sources,
portable host/regression tests, closed-loop supervisory/electrical SIL backends, a Level-4 rider/bicycle/battery digital twin,
and a cross-platform M820_BL820 developer build path.

## One-command PC verification

Quick gate:

```bash
python tools/verify_all.py --quick
```

Full PC gate (10k supervisory fuzz + sanitizers + 1k real-FOC/Hall fuzz):

```bash
python tools/verify_all.py
```

The gate covers:

- source-manifest completeness;
- BL820 packager self-check and an independent CRC/container regression;
- real-module host suites;
- deterministic whole-pipeline traces;
- fast closed-loop rider/PAS/torque/assist/final-Iq SIL;
- real `FOC.c` + `PI_control()` + vector limiter + `svpwm()` + PMSM + physical Hall feedback;
- all six Hall sectors / 24 electrical start angles;
- QZERO stop/restart lifecycle;
- deterministic fuzz;
- Level-4 rider/bike/road/battery/SOC routes + randomized physics;
- recorded-ride import/replay determinism and registered hardware regressions;
- AddressSanitizer + UndefinedBehaviorSanitizer.

## Exact M820 target build

Required compiler: **Arm GNU Toolchain arm-none-eabi 13.2.1**.

Cross-platform developer build:

```bash
python tools/build_firmware.py --toolchain "<path-to-arm-none-eabi-bin>"
```

Windows example:

```powershell
python tools\build_firmware.py --toolchain "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\13.2 Rel1\bin"
```

The builder compiles the explicit `scripts/sources-m820.txt` manifest, startup assembly and the
production linker script, checks target memory-map symbols and RWE segments, creates raw BIN/HEX,
wraps the raw application in the BL820 update container, checks the embedded build identity, and
writes a JSON build manifest with SHA-256 hashes.

Run every PC test and require the exact target build in one command:

```bash
python tools/verify_all.py --require-target
```

`--require-target` fails instead of silently skipping when GCC 13.2.1 is unavailable.

## SIL layers

### Fast supervisory SIL

`sim/evist_sil.c` without `EVD_SIL_REAL_FOC` uses the real production supervisory path and a small
current-loop stand-in. It is intentionally cheap enough for 10,000+ deterministic randomized runs.

### Electrical FOC SIL

The same `sim/evist_sil.c` compiled with `EVD_SIL_REAL_FOC` executes the real production:

```text
PAS / torque / cadence
 -> ride_control
 -> final 16 kHz Iq slew
 -> real FOC.c current measurement / Park
 -> real PI Id/Iq + vector saturation
 -> real inverse Park / SVPWM
 -> virtual PMSM
 -> physical current + rotor motion
 -> physical Hall edges
 -> production rotor_angle + rotor_motion confidence
 -> back to FOC and ride_control
```

QZERO is also executed in this full backend. The motor parameters in the virtual PMSM are test
parameters, not claimed M820 measurements; tests therefore assert architecture/invariants and
safe state transitions rather than tuning production constants to the plant.

### Walk Assist full electrical SIL

Walk Assist is also driven through the same real FOC/PMSM/Hall backend. The deterministic matrix
uses 10/15/20/30/40/50/60 chainring rpm, three virtual loads and all six Hall-sector starts
(126 cases). 70/80/>80 rpm are range/limiter tests, not normal Walk targets. Exact low-load rpm
tracking is reported as a quality metric; small speed float is not a FAIL. Safety/lifecycle
invariants (start, no runaway, current ceiling, safe stall, release/brake/fault/wheel-cut behavior)
are the hard gate. See `docs/FW143_WALK_ASSIST_10_60_TEST_CONTRACT.md`.

## Current motor-control fixes in this branch

- FW139: reject physically impossible PAS reverse bounce before direction safety.
- FW139: remove Hall-gated Gear Preload as a second normal-start current owner.
- FW140: use one conditioned cadence for assist/dynamics while retaining raw cadence for telemetry.
- FW141: make FAST/RUN torque filters depend on elapsed 4 kHz time, not foreground call count.
- FW142: extract the existing PI/vector-saturation math into one production module used unchanged
  by both target firmware and electrical SIL; no algorithm change is intended by the extraction.
- FW143: make Walk target range 10..60 chainring rpm from one shared source of truth and add a
  126-case real-FOC/PMSM/Hall Walk matrix; speed float is evidence, not a virtual-motor tuning gate.
- FW144: share the existing production SOC math through `soc_core.c`, add the Level-4 rider/bike/road/battery/SOC
  digital twin, and add canonical import/replay/registration of real-ride logs. The virtual battery/vehicle parameters
  are plant assumptions; the production control/SOC code is shared, not reimplemented.
- FW145: add an observation-only diagnostic CAN stream `0x10400..0x10407` (~47.6 coherent snapshots/s),
  keep CAN TX outside the 16 kHz FOC ISR and below critical HMI/multiframe traffic, and add direct conversion of
  the existing CANable `All Traffic` text log into Level-4 canonical replay input.

See `VERIFICATION_STATUS_2026-09-07_PL.md` for current evidence and remaining hardware gate.


## Level 4 and real-ride replay

Run Level 4 directly:

```bash
python tools/run_level4.py --quick
python tools/run_level4.py --fuzz 100 --sanitize
```

Normalize and replay a captured ride:

```bash
python tools/import_ride_log.py raw.csv canonical.csv
python tools/run_replay.py canonical.csv
```

Register a reviewed fixed hardware case:

```bash
python tools/register_ride_case.py raw.csv BUG_NAME --accept-current
```

Every registered case is subsequently executed by `tools/verify_all.py`.

The heavy electrical fuzz supports deterministic non-overlapping sharding through `tools/run_electrical_sil.py`;
this reduces wall time without reducing the requested case count or reusing random cases.

## FW145 live CAN telemetry -> Level-4 replay

FW145 adds a diagnostic-only, observation-only CAN stream `0x10400..0x10407`. It publishes about
47.6 coherent snapshots/s without transmitting from the 16 kHz FOC ISR and yields to critical HMI /
multiframe traffic. Exact wire schema: `protocol/RIDE_TELEMETRY_CAN.md`.

Build the logging firmware on Windows with:

```text
VERIFY_AND_BUILD_DIAGNOSTIC_WINDOWS.bat
```

The ordinary `VERIFY_AND_BUILD_WINDOWS.bat` still builds the normal, silent variant.

With the diagnostic firmware flashed, the existing CANable `All Traffic` raw logger can record the
new IDs without understanding them. Convert a `.log` directly:

```bash
python tools/decode_canable_ride_log.py ride.log --output-prefix ride
python tools/run_replay.py ride.canonical.csv
```

Register a reviewed real-bike case in one step:

```bash
python tools/register_canable_ride_case.py ride.log BUG_NAME --accept-current
```

The global `verify_all.py` gate includes a synthetic raw-CANable -> FW145 decoder -> canonical CSV
-> native C replay test so the wire decoder cannot silently drift away from the firmware schema.
