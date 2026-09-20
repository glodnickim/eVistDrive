# EVistDrive — agent entry point

This repository is the current **FW145 Level-4 + live CAN telemetry/replay testable baseline**. Start here before changing production code.

## 1. Required reading order

1. `AGENTS.md` (this file)
2. `README_TESTING.md`
3. `VERIFICATION_STATUS_2026-09-07_PL.md`
4. `docs/ARCHITECTURE_CURRENT.md`
5. `protocol/RIDE_TELEMETRY_CAN.md`
6. `docs/FW144_LEVEL4_VIRTUAL_BIKE.md`
7. `protocol/EVISTDRIVE_LIVE_RIDE_LOG_CONTRACT.md`
8. `verification_evidence/FW145_FINAL_GATE_SUMMARY.md`

Do not start from old FW/QS ticket notes and do not recreate removed workarounds without new evidence.

## 2. Baseline identity

Current documented history:

- `8e602e7` baseline from `EvistDrive06092026v3`
- `7033c71` FW139 PAS bounce rejection
- `0be2557` FW139 removal of Hall-gated Gear Preload
- `e6150d9` FW140 one conditioned cadence for control
- `18d81e0` FW141 elapsed-time-invariant torque filters
- `2599bc7` FW142 shared production PI/vector limiter + electrical FOC/Hall SIL
- `7ef4779` one-command exact target gate on Windows
- FW143 Walk 10..60 rpm + full Walk FOC matrix
- FW144 production SOC-core parity + Level-4 rider/bike/battery + recorded-ride replay
- FW145 observation-only live CAN telemetry `0x10400..0x10407` + direct CANable raw-log decode/replay

Use `git log --oneline` to confirm the current checkout before work.

## 3. One-owner architecture

Production ownership rules:

- PAS electrical plausibility: `pas_sampler.c`
- PAS direction safety: `pas_direction.c`
- cadence used by control: `cadence_filter.c`
- rider torque conditioning (zero, calibration, physical kg): `torque_input.c`
- pedal-assist permission and lifecycle: `ap2_pas_state.c` - the ONE owner
- rider demand and the pedal-cycle base/dynamic split: `ap2_torque_chain.c`
- rider aggression and terrain load: `ap2_estimators.c`
- profiles, the assist characteristic and AUTO: `ap2_profiles.c`
- the one limiter chain: `ap2_limits.c`
- assist demand, start/stop dynamics, and the ONE path to Iq: `assist_pipeline.c`
- selection between calibration / Walk / pedal assist, and the single publish: `ride_control.c`
- assist configuration storage and the bank wire format: `assist_modes.c` (no control math)
- final Iq trajectory: `fast_iq_slew.c` (16 kHz owner)
- current-loop PI + common voltage-vector saturation: `foc_current_loop.c`
- transforms / current measurement / SVPWM path: `FOC.c`
- final rotor angle policy: `rotor_angle.c` + `rotor_motion.c`
- ordinary zero-torque lifecycle: persistent FOC / ARMED_ZERO semantics
- low-speed stop helper: `quiet_zero.c` (QZERO; still hardware-sensitive)

Never add a second independent writer for final Iq, PI setpoint, or final Park angle.

### Assist Pipeline V2 ownership rules

Read [docs/ASSIST_PIPELINE_V2.md](docs/ASSIST_PIPELINE_V2.md) before changing anything in the
assist path. The rules that are not negotiable:

- `assist_pipeline_update()` is the ONLY place pedal assist becomes a current request. Do not add
  a second writer, a floor, a boost or an override beside or after it.
- A reverse crank step, a safety cut or assist level 0 means the request is exactly zero in the
  SAME tick. No hold, estimator or ramp may carry a positive value past that point.
- There is ONE filter on the measurement path and two dynamics primitives (`inc/ap2_math.h`).
  If a change seems to need a third mechanism in the signal path, the block it sits in is
  modelled wrongly - fix the model, do not add a filter.
- Every block takes elapsed 4 kHz ticks. Never reintroduce call-count-based filtering.
- Ride-feel numbers live in the profile table and in the stored bank. Do not scatter them.
- Wire values and diagnostic source indices are protocol. Extend them; never renumber or
  silently reinterpret them.

## 4. Important fixes that must not regress

### FW139 PAS
Very short physically impossible reverse bounces are rejected before direction safety. A real reverse must still inhibit assist immediately.

### FW139 START
The old Hall-gated ~1 A Gear Preload state was removed. Do not restore a state that waits for Hall before releasing normal start torque. Normal start must begin from the final Iq trajectory at zero and rise continuously.

### FW140 cadence
Raw cadence remains useful for telemetry/diagnostics, but control uses the conditioned cadence. Do not feed short-window raw cadence back into assist/dynamics without an explicit experiment and regression.

### FW141 torque timebase
Torque filters use elapsed 4 kHz time. Do not replace elapsed-time catch-up with call-count-based filtering.

### FW142 FOC testability
`foc_current_loop.c` is production code used by both target firmware and electrical SIL. Do not fork/copy this math into a simulator-only implementation.

### FW143 Walk range and verification
Walk target is **10..60 chainring/output rpm** with 30 rpm as the safe default. Do not raise the normal
target above 60 rpm. 70/80/>80 are negative/range tests only. Small low-load speed float is acceptable;
do not tune production Walk to force exact tracking in the virtual PMSM. Hard failures are safety/lifecycle
violations: no start, runaway, current-ceiling violation, unsafe stall, or broken release/brake/fault/wheel cut.
Read `docs/FW143_WALK_ASSIST_10_60_TEST_CONTRACT.md` before changing Walk.


### FW144 Level 4 and SOC/replay
`soc_core.c` is the production SOC math shared with Level 4. Do not fork the SOC algorithm into the simulator.
`sim/l4/` owns only virtual physics/rider/battery assumptions. `sim/replay/` replays recorded sensor history
through production C. A real-bike bug should become a registered replay case before its fix is considered closed.
Read `docs/FW144_LEVEL4_VIRTUAL_BIKE.md` and `protocol/EVISTDRIVE_LIVE_RIDE_LOG_CONTRACT.md`.

## 5. Build i numeracja wersji

### Tryby build

| Tryb | Polecenie | Wersja | Zastosowanie |
|------|-----------|--------|--------------|
| `developer` | `python tools/build_firmware.py --mode developer` | `DEV-NONCANONICAL` | Szybka weryfikacja, bramka, debug — **bez numeru kanonicznego** |
| `auto` | `python tools/build_firmware.py --mode auto` | `0.601`, `0.602`... | **Wydanie kanoniczne** — rezerwuje numer z globalnego allocatora |
| `repro` | `python tools/build_firmware.py --mode repro --version 0.601` | dokładnie `0.601` | Odtworzenie konkretnego历史的 builda |

Na Windowsie kanoniczny entry point to `VERIFY_AND_BUILD_WINDOWS.bat` (wykonuje pełną bramkę + build z `--mode developer`).

### Allocator wersji

Globalny licznik żyje w `.ebics-version-state/M820_BL820.json` (poziom wyżej niż repo motor-controller-firmware). Format: `X.YYY` (trzy miejsca po przecinku). Obecny HWM = 601 (następny: 0.602).

```powershell
# Inicjalizacja (jeśli stan nie istnieje)
python tools/build_firmware.py --init-state 600

# Rezerwacja następnego numeru
python tools/build_firmware.py --toolchain "..." --mode auto --variant normal

# Sprawdzanie aktualnego HWM
type ..\..\..\.ebics-version-state\M820_BL820.json
```

**Zasada:** każde kanoniczne wydanie (FLASHowane na rower) musi rezerwować numer przez `--mode auto`. Buildy developerskie (bramka, testy) używają `--mode developer`.

Po zbudowaniu trybem `auto` finalny BIN jest automatycznie kopiowany do `releases/<version>_M820_BL820.bin`:

```powershell
releases\0.601_M820_BL820.bin
releases\0.602_M820_BL820.bin
```

Tylko plik `.bin` — żadnych dodatkowych artifactów (manifest, ELF, MAP pozostają w `.build\`).

**ZASADAKANONICZNA:**
1. Każdy kanoniczny build musi używać `--mode auto` (rezerwuje numer z globalnego allocatora).
2. Wynikowy BIN leży w `releases/<version>_M820_BL820.bin` — **bez podkatalogów**.
3. Build developerski (`--mode developer`) NIGDY nie kopiuje do `releases/`.
4. Przed flashowaniem sprawdź SHA256 z konsoli — musi być identyczne z tym w terminalu.
5. Żaden agent nie tworzy nowych folderów w `releases/` (np. `releases/latest`, `releases/final`) — zawsze `<wersja>_M820_BL820.bin`.

### FW-xxx vs BUILD version

```text
FW145     = etykieta funkcjonalna w dokumentacji (baseline, owner rules)
0.601     = kanoniczny numer builda (z allocatora, monotonny, traceable)
DEV-NONCANONICAL = build developerski (nie wydany, bez numeru)
```

Nie mieszaj tych systemów — `docs/19_BUILD_VERSION_AND_TOOLING_GOVERNANCE.md` §Build ID != Feature ID.

## 6. Verification gate

Before any production-code change:

```bash
python tools/verify_all.py --quick
```

After the change:

```bash
python tools/verify_all.py
```

Before producing a flash candidate, require exact Arm GCC 13.2.1 target build:

```bash
python tools/verify_all.py --require-target
```

On Windows use:

```text
VERIFY_AND_BUILD_WINDOWS.bat
```

A change is not accepted because a local unit test passes. The full gate must stay green.

## 6. Test philosophy

> Uwaga: sekcja "Verification gate" została przeniesiona pod §5 (Build i numeracja wersji).

Prefer invariants and end-to-end scenarios over isolated expected constants. At minimum consider:

- cold start from every Hall sector;
- loaded start;
- PAS bounce and true reverse;
- steady 20/40/60/80 rpm riding;
- Walk 10/15/20/30/40/50/60 rpm across load/Hall-start matrix;
- stop -> ARMED_ZERO -> restart;
- current/power/speed/thermal limiter transitions;
- missed foreground ticks;
- deterministic fuzz;
- sanitizer runs.

If a new bug is found on hardware, first add a deterministic reproduction to SIL/host tests, then change production code.

## 7. Reference hierarchy

Use references for different purposes, not as code to copy:

- G532: START/STOP lifecycle, bounded Q/Iq behavior, bicycle feel
- Fake Taxi: robust 3-Hall timing/angle behavior
- TSDZ2: rider-facing functionality and parameter semantics
- VESC: FOC/current-control/Hall-confidence cross-check

Do not blindly copy raw constants across different current/angle/time domains.

## 8. Known open / hardware-sensitive areas

Do not claim these are closed solely from SIL:

- exact real M820 PMSM R/L/flux/inertia/backlash parameters;
- final QZERO tuning/handback behavior on the physical motor;
- final ride-feel tuning of Start Take-up / Accel / Decel / Release;
- exact target `.bin` reproducibility until built with Arm GNU 13.2.1 on the supported target toolchain.

The simulator must test invariants and safety here, not force production constants to match an assumed virtual motor.

## 9. Change discipline

For every non-trivial production change:

1. state the symptom/root cause;
2. identify the current owner;
3. add/reuse a reproducing test;
4. make the smallest owner-local change;
5. run quick gate;
6. run full gate;
7. update architecture/evidence only if semantics changed;
8. commit one coherent change.

Do not mix rider-feel tuning, FOC changes, configuration migration, and protocol changes in one patch.

## 10. FW145 live telemetry ownership rule

`src/ride_telemetry.c` owns only serialization/pacing of **observations**. It owns no rider demand,
permission, Iq, FOC, QZERO, Hall or SOC state.

- IDs `0x10400..0x10407` are reserved to FW145 live ride telemetry.
- Do not reuse `0x10300..0x10307` (STOP_TRACE).
- Do not transmit telemetry from the 16 kHz FOC ISR.
- Do not let 4 kHz foreground read the `quiet_zero_state` object; it may only read the ISR-published
  `qzero_diag_state_isr` mirror.
- Keep snapshot construction rate-limited (~48 Hz) and CAN sending non-blocking/best-effort.
- Critical queue/multiframe/dumps always have priority.
- `0x10406` PAS A/B is a sparse state snapshot, not a raw quadrature event recorder.
- Any wire schema change requires a schema/version update, decoder test update and
  `protocol/RIDE_TELEMETRY_CAN.md` update in the same commit.

Hardware capture workflow is raw CANable `.log` -> `tools/decode_canable_ride_log.py` -> canonical
replay -> reviewed permanent case via `tools/register_canable_ride_case.py`.
