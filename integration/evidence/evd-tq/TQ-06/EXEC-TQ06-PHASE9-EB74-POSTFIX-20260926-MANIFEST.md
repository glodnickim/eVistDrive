# TQ-06 Phase-9 EB74 B+0x58 preservation fix

TIMESTAMP: 2026-09-26T08:42:44+02:00
TASK: TARGETED EB74 IMPLEMENTATION FIX AND REQUIRED GATES
FIRMWARE: feat/TQ-06-g53-port @ 62785bd105a0e5d2681b1db71b40f62180af6c53
GOVERNANCE: unchanged; no governance checkout/file was written
PHASE 10: NOT STARTED
COMMIT/PUSH/MERGE/FLASH: NONE

## Exact implementation change

In src/g53_port_boundaries.c::g53_ad7ec_step(), moved rider_input_native = 0 inside the
source > threshold fault branch under check_count < EB74_CHECK_LENGTH. Thus source <= 960
preserves the previous B+0x58 equivalent; source > 960 still clears output and rewinds the
counter. Fresh reset still initializes output to zero. No threshold, comparator, counter,
lifecycle, auto-zero, filtering, feedback, PAS, or safety behavior was changed.

The test-only host harness includes the real boundary source in its translation unit so it can
seed the file-private output state exactly as the accepted native sentinel fixture does. No
production test hook/API was added. The boundary suite runner no longer separately links that
same module; the pre-existing P9-G5 integration-suite source addition (pas_quadrature.c) is
retained.

## Regression and conformance results

- Targeted GCC command: gcc -std=c11 -Wall -Wextra -Werror -Iinc -o .build/eb74-boundaries-postfix.exe tests/host/g53_port_boundaries_host.c, then the executable; exit 0.
- Persisted independent frozen reference: 29,516 vectors, zero mismatches.
- T1 sentinel: 0x1234 is retained through invocations 31–130; invocation 131 yields 52 on the accepted feedback/source schedule.
- T2 reset-zero: remains zero throughout the check window through preservation.
- T3 high-load-from-reset: source 961 rewinds/holds check_count at 90 and output remains zero through 400 check-path invocations.
- T4 unloaded boot then load: 130 qualifying calls, then invocation 131 yields positive output 831.
- T5 boundary: source 960 advances check_count to 1 and preserves output; source 961 rewinds it to 90 and clears a seeded sentinel, proving strict > and the frozen fault-output gate.
- Accepted schedules re-run against the port: A has 150 rows and invocation 131 output 831; B has 400 high-from-reset rows and output remains zero; S has 100 sentinel-preservation rows (31–130) and invocation 131 output 52.
- Adaptive-zero, D+0xEE, filter/feedback, and remaining accepted fixture fields continue to match in the 29,516-row frozen reference.

Evidence:
- Pre-fix sentinel (preserved unchanged): EXEC-TQ06-PHASE9-EB74-CONFORMANCE-20260926-schedule-S.csv, SHA-256 684142fa332f7f480fb39402cf93557dc4b4f9dde516449f7db05f8396d6db9a.
- Post-fix schedules A/B/S: EXEC-TQ06-PHASE9-EB74-POSTFIX-20260926-port-vectors.csv, SHA-256 157D1936CB61A344D5BADA21174CA3BBB38FCE43E08285BF3CA408F3B78403A4.
- Targeted host output: EXEC-TQ06-PHASE9-EB74-POSTFIX-20260926-targeted-host.txt, SHA-256 4CA2E79B2F3B47A2BB659E784062B8B61B020D0202762488BED964A8E066A4FB.
- Full host output: EXEC-TQ06-PHASE9-EB74-POSTFIX-20260926-host-gate.txt, SHA-256 F6A64F1514146DA6DB8BB5D65A0CE2A17D07AE2A503232681C236ECC1C97A7F4.
- Level-4 output: EXEC-TQ06-PHASE9-EB74-POSTFIX-20260926-level4-quick.txt, SHA-256 AA0B2A35AA4CAEAEB3D9FE028F3D2C4173F1523085FD34371E22A4403CC2F847.
- Exact final source/harness/runner diff: EXEC-TQ06-PHASE9-EB74-POSTFIX-20260926-final-source-diff.patch, SHA-256 48BD9D39DD2046B2991B1E30DA1A455FD1920A6516482FE6B17158AEFFF7AD3D. The earlier source-diff.patch records an intermediate implementation and is superseded by this final patch.

## Gates

The complete powershell.exe -NoProfile -ExecutionPolicy Bypass -File tests/host/run-host-tests.ps1
host gate exits 0 with All host suites: PASS.

After host PASS, the unchanged python tools/run_level4.py --quick was run. It exits 1:
all nine fixed ride scenarios fail with IqMax=0; all SOC endurance checks pass. No Level-4
source, startup sequence, prehistory, load stimulus, expectation, or scenario definition was changed.

## Remaining Level-4 startup classification

The source explicitly initializes firmware state in l4_init() (memset, ride_control_init(),
pas_sampler_init()) for every scenario. evd_rider_init() sets pedaling=true; at zero cadence
the rider model supplies its launch torque immediately. The fixed matrix starts its first control
tick with load_ctrl=12000 (pre-EB74=3200); the first logical EB74 call is tick 4. The accepted
startup audit found no input at/below 960 in any of the nine cases. Thus the encoded startup
semantics are POWER_ON_UNDER_LOAD, not already-riding with qualification history or an unloaded
numerical warm start.

The Level-4 README says a hard failure includes failure to produce assist when the virtual rider
has a clear (>15%) static launch-torque margin. That expectation conflicts with the frozen EB74
high-load-from-reset result (zero until check qualification). Classification: FROZEN_BEHAVIOR_EXPECTED.
This task is stopped without a second fix. Resolving the incompatible acceptance/lifecycle
expectation requires a reviewed governance/contract decision before any later Level-4 change.

## Scope and verification

- Production thresholds: unchanged.
- Level-4 stimulus/prehistory: unchanged.
- P9-G5: unchanged.
- Governance: unchanged.
- git diff --check: PASS after the final report/evidence append.
- Existing Phase-9 working-tree changes are preserved; no commit, push, merge, or Phase 10 work.

NEXT EXACT ACTION: obtain a reviewed decision for the Level-4 cold-start-under-load expectation versus
the frozen EB74 qualification contract; then resume Phase 9 at that blocker only.
