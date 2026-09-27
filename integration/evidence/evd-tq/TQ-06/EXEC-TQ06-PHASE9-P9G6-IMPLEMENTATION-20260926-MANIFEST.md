# TQ-06 Phase-9 P9-G6 implementation and downstream STOP

TIMESTAMP: 2026-09-26T09:56:48+02:00
GOVERNANCE: glodnickim/eVistDrive-development-system, origin/master = 640bc7984a88712e3c18bb698ec85c55e1e401c0. The active P9-G6 section was read directly from `git show origin/master:contracts/M820_TORQUE_INTENT_CONTRACT.md`. Governance files/worktree were not changed.
FIRMWARE BASE: `feat/TQ-06-g53-port` @ `62785bd105a0e5d2681b1db71b40f62180af6c53`; pre-existing dirty Phase-9 WIP preserved.

## Implementation

- Added `sim/l4/eb74_invocation_observer.c/.h`, linked only into Level-4 with `-Wl,--wrap=g53_ad7ec_step`. The test observer calls the real production function and records its actual invocation count/output; it does not write production state.
- In `sim/l4/virtual_bike_l4.c`, each of the nine fixed cases performs the real `l4_init()` cold reset, then runs 131 public `ride_control_update()` calls with neutral native PAS, valid non-fault sensors, zero torque, and `load_ctrl=0`. Each uses `elapsed_ticks=4`; the observer confirms exactly one real EB74 invocation per iteration.
- No Level-4 plant/control tick runs during prehistory. Scenario tick, bike, motor, battery, energy/speed/distance and maxima remain at reset values. Scenario time starts with its original first scored control update.
- Invocation 131 is recorded as `PREHISTORY` / unscored, with startup_count=300 and check_count=100. First scored `flat_soc90` input follows it and retains old t=0 values: load_ctrl=12000, mapped pre-EB74=3200, native PAS=0, PA6=0, speed/cadence=0, assist level=3, elapsed_ticks=1, valid sensors and no veto.
- Separate cold high-load negative guard uses the real port for 400 invocations with load_ctrl=12000 / source=3200; EB74 remains zero/unqualified and no G53 permission/Iq request appears.
- No production source, EB74 constant/algorithm, PAS normalization, scenario stimulus or governance file was changed by P9-G6.

OLD POWER_ON_UNDER_LOAD BLOCKER (nine fixed Level-4 cases): CLOSED by the P9-G6 decision and verified 9/9 result.\nREQUIRES NEW GOVERNANCE DECISION: NO. No new contract gap was found; the later STOP_RESTART result is recorded as a separate failed gate, with its internal first-zero cause not yet localized.\n\n## L4-PRE-1..8

PASS. PRE-1 cold reset; PRE-2 all 131 actual prehistory calls load=0/preEB74=750 and valid/non-fault; PRE-3 no scored update before readiness; PRE-3A invocation 131 unscored; PRE-3B first scenario update strictly afterward; PRE-4 no prehistory metrics/time/plant advance; PRE-5 original first scored values retained; PRE-6 cold high-load negative remains zero/unqualified; PRE-7 EB74 state remains private in its separately compiled production translation unit, with no seeding API used; PRE-8 each iteration increments the observed real EB74 count exactly once.

## Level-4 evidence

`python tools/run_level4.py --quick` was actually rerun after the harness change. All nine fixed ride scenarios pass (9/9) and SOC checks pass. The complete command exits 1 in its separate 25-case random fuzz phase (10 EXPECTED_START cases report no Iq/Hall). Those generated cold-start cases were not given P9-G6 prehistory or modified; no Criterion-C/cold-start behavior was changed.

Representative `flat_soc90` stage trace:

- PREHISTORY invocations 1..131: load=0/pre=750, all unscored; call 130 is check_count=100; call 131 completes normal EB74 processing while unscored.
- First scored update: scenario tick 1, load_ctrl=12000, mapped pre=3200, actual EB74 count remains 131.
- First loaded EB74 call: scenario tick 4 / invocation 132; rider_input_native=831.
- First D7EC rider >0: tick 3080 / invocation 901.
- First normal_permission, M2AA, Boundary-B request and final Iq >0: tick 3088 / invocation 903; final Iq setpoint=3.
- First E1E8 output >0: tick 3132 / invocation 914; output=67. The first all-positive row has M2AA=399 and final Iq=27.

## Next mandatory gate and stop

`python tools/run_sil.py --fuzz 0` was executed and stops at `STOP_RESTART FAIL: did not establish ACTIVE ride`; first_permission_tick and Iq remain zero. Resume-after-real_stop and DISC-007 were not reached. The preserved clean_start.csv shows native PAS activity/forward qualification/cadence (AB reaches 2 at tick 64; later fwd_run=250/cadence=40/load_ckg=1801) while iq_request remains zero. No speculative fix was applied.

`DISC-007: NOT_REACHED`. `RESUME_AFTER_REAL_STOP: NOT_REACHED`. Supervisory/electrical SIL, remaining S-E2E, replay/W1/CRUISE/host/full-regression/final-scope gates were not run after this stop. No commit/push/merge/flash; Phase 10 not started.

## Evidence files and SHA-256

- `EXEC-TQ06-PHASE9-P9G6-20260926-level4-quick.txt` — E6C754D8F7209144EEDF8E5F4AC0FC639C871806E9C2B5BE1A312AA7AC7081B4
- `EXEC-TQ06-PHASE9-P9G6-20260926-flat_soc90-prehistory.csv` — D1D62E5F51870E29B9DC39810B3CF811DFF33F108E5CBA48B221F69FCE84DF6F
- `EXEC-TQ06-PHASE9-P9G6-20260926-flat_soc90-stages.csv` — 80693F6D484769145C0F7D2218550404633D120B32B92767C867ACD449EF40A8
- `EXEC-TQ06-PHASE9-P9G6-20260926-stop-restart.txt` — CE3B94AB85286C678437857C0962E6836C95B35093FCBE39DCE96D8368842242
- `EXEC-TQ06-PHASE9-P9G6-20260926-sil-clean-start.csv` — A179AC59D65BE014232E9F668408A384F791F29616DF525728307738A830AC22
- `EXEC-TQ06-PHASE9-P9G6-20260926-harness-diff.patch` — EE7778D498F1D81DDCB6D756480F714BE49C9CDF7B444C0B2E897D50C9ECE59C
- `EXEC-TQ06-PHASE9-P9G6-20260926-eb74-invocation-observer.c` — 764EE225FB93BACC65D4F013B49CD69F3FAC56AE489AFEEE5642B36B30DD70C9
- `EXEC-TQ06-PHASE9-P9G6-20260926-eb74-invocation-observer.h` — 08C5B8B6538CBDB57ABF416D259B8CB1AFF96E4E4A6F9DBAF105AE7E6D20F1DE