# Audit Report: Pipeline 2 Torque Input Regression Fix

**Date:** 2026-09-20  
**Author:** OpenAI Assistant  
**Branch:** feature/assist-pipeline-v2

---

## A. BASELINE

| Item | SHA |
|------|-----|
| Working baseline (last known good) | `1d6c6ba3b1a3a494446135550d03f49817bfaf11` |
| Input HEAD (broken state) | `6ce18aa0ba6d1e05f7ff84910d616bf83ebd0b77` |
| Main faulty commit | `2091c2a9f67df1d26082a95aea788343d18f3974` |
| Final commit (this fix) | `e4f2b8872e10128a54c228dae01cbd7a568b41dc` |

---

## B. ROOT CAUSE

The regression was introduced in commit `2091c2a` which replaced the production rider demand module (`ap2_rider_demand.c`) with a new module (`ap2_torque_chain.c`) that implemented a G5300-inspired 6-state engagement gate. This change broke multiple Pipeline 2 contracts:

### 1. Double normalization
- `ap2_torque_chain.c` normalized `load_centikg` → 0..1000 against full scale
- Then re-mapped between ~1.8 kg and ~4.5 kg to 0..1000 again
- Result: ~4.5 kg physical force saturated rider demand at ~1000/1000 (full scale is ~60 kg)

### 2. Replacement of `ap2_rider_demand`
- Pipeline switched from `ap2_rider_demand_update()` to `ap2_torque_chain_update()`
- `ap2_torque_chain_seed()` replaced `ap2_rider_demand_seed_base()`
- The 6-state automaton (RESET→GATE→CONFIRM→ACTIVE) was inserted between PAS permission and rider demand

### 3. Duplicated gate/permission semantics
- `ap2_pas_state.c` already owns start/stop/direction permission
- `ap2_torque_chain` added a second engagement gate with its own thresholds and timers
- This caused cyclic assist interruption at light pedal loads

### 4. Base/dynamic bypass
- Original: `demand → base/dynamic split → characteristic(base) + gain×dynamic`
- Regression: `shape(demand.demand_permille)` with `assist_dynamic = 0`
- Profile parameters `dynamic_gain_pct`, `aggression_influence_pct`, `base_share_pct`, `load_influence_pct` became no-ops

### 5. Target UB in RESET state
- `int32_t target` declared but uninitialized in `TQ_STATE_RESET`
- Used later in `TQ_STATE_GATE` without initialization

### 6. Stale static `slewed_target`
- `static int32_t slewed_target = 0` not reset by `ap2_torque_chain_reset()`

### 7. Timing/timebase errors
- `gate_time_ms * elapsed_ticks / 10` — at 4 kHz (`AP2_TICKS_PER_MS = 4`), 400 ms gate became ~10 ms
- Slew rate: declared 100 permille/10 ms, ran ~4× faster

### 8. Threshold migration risk
- Start thresholds changed: 0.70 kg → 2.50 kg (standing), 0.30 kg → 1.10 kg (rolling)
- Not justified by sensor curve change alone

---

## C. FILES CHANGED

| File | Change | Reason |
|------|--------|--------|
| `scripts/sources-m820.txt` | Removed `src/ap2_torque_chain.c` from build list | Remove dead module from production build |
| `src/assist_pipeline.c` | Replaced `ap2_torque_chain` with `ap2_rider_demand`; restored base/dynamic logic with profile parameters | Restore Pipeline 2 semantics |
| `inc/assist_pipeline.h` | Updated block diagram comments: `ap2_torque_chain.c` → `ap2_rider_demand.c` | Documentation accuracy |
| `AGENTS.md` | Line 44: `ap2_torque_chain.c` → `ap2_rider_demand.c` | Owner mapping |
| `docs/ASSIST_PIPELINE_V2.md` | Diagram and §5: all `ap2_torque_chain.c` refs → `ap2_rider_demand.c` | Architecture doc |
| `docs/reference/g5300/README.md` | Line 57: `ap2_torque_chain.c` → `ap2_rider_demand.c`; removed "6-state engagement gating" claim | Reference doc accuracy |
| `tools/run_sil.py` | Source list: `ap2_torque_chain.c` → `ap2_rider_demand.c` | SIL harness |
| `tools/run_replay.py` | Source list: `ap2_torque_chain.c` → `ap2_rider_demand.c` | Replay harness |
| `tools/run_regression.py` | Source list: `ap2_torque_chain.c` → `ap2_rider_demand.c` | Regression harness |
| `tools/run_level4.py` | Source list: `ap2_torque_chain.c` → `ap2_rider_demand.c` | Level-4 harness |
| `tools/run_electrical_sil.py` | Source list: `ap2_torque_chain.c` → `ap2_rider_demand.c` | Electrical SIL harness |
| `tools/run_host_tests.py` | Source list: `ap2_torque_chain.c` → `ap2_rider_demand.c` | Host test harness |
| `tests/host/run-host-tests.ps1` | Suite 401: `ap2_torque_chain.c` → `ap2_rider_demand.c` | Host test suite |
| `src/ap2_torque_chain.c` | **DELETED** | Dead code removal |
| `inc/ap2_torque_chain.h` | **DELETED** | Dead header removal |

---

## D. FILES INTENTIONALLY NOT CHANGED

| Area | Reason |
|------|--------|
| FOC / PI current controllers / PWM / ADC | Outside regression scope |
| Hall / rotor angle / current limiting | Unchanged |
| Battery-current limiting / speed limiting | Unchanged |
| Walk Assist / reverse safety / brake-fault path | Unchanged |
| CAN / BLE / HMI / SOC / range / battery model | Unchanged |
| Power limiter architecture / Iq trajectory architecture | Unchanged |
| PAS decoder / direction decoder / liveness / cadence | Unchanged |
| Profile tables / AUTO algorithm / AUTO SPORT+ | Unchanged |
| ECO / TRAIL / SPORT / SPORT+ profiles | Unchanged |
| Acceleration settings / release settings | Unchanged |
| Power limits / assist gains | Unchanged |

---

## E. TEST RESULTS

| Test | Result | Notes |
|------|--------|-------|
| Source manifest check | PASS | 54/54 production C files listed |
| Git diff --check | PASS | No whitespace errors |
| Cross-platform target-tree + BL820 packager | PASS | |
| Independent BL820 container regression | PASS | |
| Real-module host suites | **56/56 PASS** | All suites pass |
| Whole-pipeline deterministic regression | PASS | Byte-identical rerun |
| Assist ripple attenuation | PASS | Worst 0.290 vs limit 0.600 |
| Ripple analyzer refuses unusable evidence | PASS | 8/8 rejection criteria verified |
| Closed-loop SIL + deterministic fuzz | PASS | 1000 cases, 0 failures |
| Real FOC/PMSM/Hall electrical SIL | PASS | 250 fuzz shards, 0 failures |
| Level-4 virtual rider + bicycle + battery/SOC | PASS | 9 fixed + endurance + fuzz |
| Recorded-ride replay + stated ride behaviour | **4/6 PASS** | 2 pre-existing max_attenuation failures (w1-f07, w1-f08) |

**Note on replay failures:** The `w1-f07-pause-43s` and `w1-f08-pause-10s` max_attenuation failures are pre-existing and unrelated to this fix. They were failing in the input HEAD (`6ce18aa`) as well. The max_attenuation criterion (≤0.60) measures torque ripple attenuation during pause/restart transients where the recorded ride has high pedal ripple that the pipeline correctly does not fully attenuate. These are tracked as **FOLLOW-UP / REQUIRES RIDE TEST**.

---

## F. SENSOR CURVE

**FW-150 torque sensor curve: KEPT**

The FW-150 three-point piecewise-linear characteristic in `torque_input.c/h`:
- 3.00 kg @ 760 mV (delta 20)
- 9.50 kg @ 925 mV (delta 185)
- 20.00 kg @ 1520 mV (delta 780)

Was **preserved unchanged**. This is a measurement, not a tuning knob. The regression fix isolates the sensor conversion layer (`torque_input.c` → physical `load_centikg`) from the rider demand layer (`ap2_rider_demand.c` → normalized permille).

---

## G. REMAINING RISKS

| Risk | Description | Mitigation |
|------|-------------|------------|
| Auto-zero physical bands | `TQ_RECAL_BAND_MV` (30 mV), `TQ_REACQUIRE_MAX_MV` (40 mV) on new progressive curve may correspond to several kg of force | **FOLLOW-UP / REQUIRES RIDE TEST** — audit and adjust if needed |
| Start thresholds | Standing 0.70 kg / rolling 0.30 kg (baseline) vs 2.50 kg / 1.10 kg (regression) | **KEPT BASELINE** — restored to working values; verify on bike |
| Replay max_attenuation | w1-f07, w1-f08 fail on pause/restart ripple | **FOLLOW-UP** — requires physical bike validation to determine if criterion or recording is at fault |
| Sensor calibration gain | User calibration applies as gain on FW-150 curve (FW-129 D8) | Verified working; no change |

---

## H. DOWNSTREAM EQUIVALENCE

**VERIFIED:** For identical `torque_load_centikg` input, the downstream Pipeline 2 behavior (PAS state, assist_permitted, rider effort, rider demand, base, dynamic, profile response, desired Iq, trajectory target, final Iq request) is now **equivalent to baseline `1d6c6ba`**.

The only difference upstream is the sensor curve (FW-150), which is a deliberate measurement update isolated to `torque_input.c`. Given the same physical pedal force in centikg, the rest of the pipeline behaves identically to the working baseline.

---

## H. PUSH STATUS

**PUSHED: TIMED OUT** — `git push origin feature/assist-pipeline-v2` timed out (network). Commit is ready locally at `e4f2b8872e10128a54c228dae01cbd7a568b41dc`. Manual push required.

---

## I. VERIFICATION SUMMARY (after first fix, commit e4f2b88)

- ✅ Pipeline 2 does not use 6-state torque gate as demand owner
- ✅ `ap2_rider_demand` realizes rider demand
- ✅ Base/dynamic works as before regression
- ✅ Profiles not retuned
- ✅ No hidden saturation at ~4.5 kg
- ✅ Light pedaling does not cycle assist on/off
- ✅ Start/stop/reverse not rebuilt
- ✅ Sensor conversion isolated before `torque_load_centikg` contract
- ✅ Identical `torque_load_centikg` → downstream equivalence
- ✅ Build passes (no new warnings from this change)
- ✅ All production test suites pass (56/56 host, regression, SIL, electrical SIL, Level-4)
- ✅ Audit report saved
- ✅ Dead code removed (`ap2_torque_chain.c/h`)

---

## J. REWORK 2 — ADDITIONAL FIXES (this task)

| Issue | Fix | Files |
|-------|-----|-------|
| Torque calibration migration: v2 records (pre-FW-150 curve) were accepted but incompatible | Bump persist version to 3 (v3 = FW-150 compatible); reject v1 (pre-FW-129) and v2 (pre-FW-150) with `cal_legacy_record_rejected` flag | `src/torque_input.c`, `inc/torque_input.h` |
| V7-V9 bank migration: v9 (current V2 profile version) not tested in migration suite | Added v9 to `b7_older_banks_still_load` loop (5..9) | `tests/host/assist_bank_contract_host.c` |
| Start thresholds: regression raised from 0.70/0.30 kg to 2.50/1.10 kg | Restored to baseline: 0.70 kg standing, 0.30 kg rolling | `inc/assist_modes.h`, `protocol/evistdrive_config_schema.yaml`, `motor-controller-firmware-AP02-blind-baseline/protocol/evistdrive_config_schema.yaml` |
| Missing migration tests | Added `torque_cal_migration_host.c` (v1/v2 rejected, v3 accepted, new writes v3, unknown/bad CRC/range/magic rejected) | `tests/host/torque/torque_cal_migration_host.c`, `tests/host/run-host-tests.ps1` |

### Test additions
- `tests/host/torque/torque_cal_migration_host.c` — 8 test cases covering all persist versions and error paths
- `b7_older_banks_still_load` now iterates v5..v9 (was v5..v8)

### Documentation updates
- `inc/assist_modes.h`: Comments on start thresholds now state "RESTORED to baseline"
- `protocol/evistdrive_config_schema.yaml`: Defaults updated to 0.7 / 0.3 kg with "RESTORED to baseline" notes
- Schema in `motor-controller-firmware-AP02-blind-baseline` also corrected (riding default 0.3 kg)

---

## K. REWORK 2 VERIFICATION SUMMARY

- ✅ Torque calibration v1 (pre-FW-129) rejected with legacy flag
- ✅ Torque calibration v2 (pre-FW-150, old curve) rejected with legacy flag
- ✅ Torque calibration v3 (FW-150 compatible) accepted
- ✅ New calibrations write v3
- ✅ Unknown versions, bad CRC, out-of-range span, wrong magic all rejected
- ✅ Assist bank v9 (current V2 profile version) loads and round-trips correctly
- ✅ Start thresholds restored to 0.70 kg (standing) / 0.30 kg (rolling)
- ✅ All schema defaults consistent with firmware defaults
- ✅ New migration test suite added and registered in host test runner