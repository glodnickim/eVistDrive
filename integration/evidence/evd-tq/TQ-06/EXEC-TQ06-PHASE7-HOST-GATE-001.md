# TASK EXECUTION REPORT — TASK-EVD-TQ-06 — Phase 7 host-gate rework

```text
REPORT_TIMESTAMP: 2026-09-25T07:52:54+02:00
```

## Tożsamość wykonania

```text
PROJECT_ID:             EVD
REPOSITORY:             motor-controller-firmware
COMPONENT:              firmware assist pipeline / host verification
TARGET:                 M820 host gate
FEATURE_ID:             TQ-06
TASK_ID:                TASK-EVD-TQ-06
TASK_TITLE:             TQ-06 firmware implementation — Phase 7 host-gate rework
EXECUTION_ID:           EXEC-EVD-TQ-06-PHASE7-HOST-GATE-REWORK-001
STARTED_AT:             UNKNOWN
ENDED_AT:               2026-09-25T07:52:54+02:00
EXECUTION_SCOPE:        REPOSITORY_LOCAL
PRIMARY_OWNER:          motor-controller-firmware
AFFECTED_REPOSITORIES:  motor-controller-firmware
FINAL_STATUS:           BLOCKED
```

## Dla użytkownika

**Czego dotyczyło zadanie:** dokończenie kontroli hostowej po Phase 4 w ramach aktywnej Phase 7.

**Co zrobiono:** zaktualizowano kontrole Quiet Zero i QS-3D do właścicieli G53 oraz podłączono suite 57 do produkcyjnego zestawu modułów G53, limiterów i final-Iq ownera. Dodano inicjalizację banku poziomów w scenariuszu hostowym.

**Wynik:** pełny host gate: 63/64 PASS, suite 57 FAIL. Phase 4 pozostała zamknięta; nie rozpoczęto kolejnej fazy.

**Stan:** `BLOCKED`

**Czy coś pozostało:** tak; scenariusz suite 57 musi uzyskać dodatnie M2AA przez aktywną ścieżkę G53, aby sprawdzić P6/P7/P8.

**Najważniejszy problem:** scenariusz aktywnego wspomagania kończy się `m2aa_native=0`, `normal_permission=false`, więc nie ustanawia dodatniego żądania.

**Dlaczego wystąpił:** przyczyna zachowania BDE8/G53 w tym scenariuszu pozostaje `UNKNOWN`; dokładna obserwacja jest zapisana poniżej.

**Co dalej:** uzyskać zaakceptowany, deterministyczny wektor wejściowy end-to-end G53 dla M820 albo osobny zakres diagnozy granicy wejście→BDE8. Nie zmieniać zamkniętych modułów Phase 3/5 ani kontraktu w ramach tego reworku.

## Wykonanie

**SCOPE EXECUTED:** Phase 7 host verification files only, together with the pre-existing Phase 7 production work in the working tree.

**WHAT WAS ACTUALLY DONE:**

- Replaced the seven stale QZERO T14 checks that expected the retired AP2 trajectory with guards for the single G53 pipeline zero-policy assignment, native direction/safety owners, service exclusion, limiter ordering, and absence of an AP2 fallback.
- Replaced the three stale QS-3D static checks for AP2 trajectory/Q10 mode dispatch with checks for the G53 final request, the safety modes, and BYPASS ownership.
- Corrected suite 57's manifest in `tests/host/run-host-tests.ps1`: the suite now links `g53_port.c`, `g53_port_boundaries.c`, `g53_port_pas.c`, `g53_port_chain.c`, the active pipeline, limits, and 16 kHz owner. `tools/run_host_tests.py` already parses this canonical runner and needed no change.
- Reworked the authorized `tests/host/ap2_pipeline_scenarios_host.c` harness to exercise the production pipeline and its zero-policy scenarios. It calls `assist_modes_init()` before using configured levels.
- Ran the complete host gate: 64 suites parsed, 63 PASS, one failing suite (57), zero post-step skips. Suites 41 and 42 pass.

**WHAT WAS NOT DONE:**

- Did not weaken or delete P6, P7, or P8 checks.
- Did not modify completed Phase 4/5 commits, start a later phase, commit, push, merge, flash, or alter the frozen contract.
- Did not establish the accepted end-to-end G53 input vector that enters active BDE8 output through the M820 port adapter.

**FILES / COMPONENTS TOUCHED:**

- Pre-existing Phase 7 working tree: `inc/assist_pipeline.h`, `inc/g53_port_chain.h`, `inc/ride_control.h`, `src/assist_pipeline.c`, `src/g53_port.c`, `src/g53_port_chain.c`, `src/main.c`, `src/ride_control.c`.
- This rework: `tests/host/ap2_pipeline_scenarios_host.c`, `tests/host/qs3d_16khz_slew_host.c`, `tests/host/qzero_quiet_zero_host.c`, `tests/host/run-host-tests.ps1`.
- Report: `integration/evidence/evd-tq/TQ-06/EXEC-TQ06-PHASE7-HOST-GATE-001.md`.

**IMPORTANT DECISIONS:**

- Phase 4 remains closed at commit `20650bb`.
- Phase 7 does not pass its host gate and cannot advance.
- The remaining observed behavior is not promoted to a confirmed production defect: the harness reached a G53 PAS direction/cadence state and D7EC stage values, but BDE8 stayed at state 1 and emitted zero M2AA. Root cause remains `UNKNOWN` pending a frozen, accepted integration vector or separately authorized diagnosis.

## Problemy

```text
PROBLEM_ID:      TQ06-PHASE7-HOST-57
PROBLEM:         Three behavior checks in suite 57 fail because the positive-demand fixture never produces positive M2AA.
CAUSE:           UNKNOWN. In the observed run, final command/ref were zero, G53 m2aa_native=0, normal_permission=false, BDE8 mode=0, q50=1; PAS direction=1, filtered cadence=102, transition_count=6, evidence=794; D7EC rider=500 and requested=475.
IMPACT:          P8 cannot establish positive normal G53 demand; P7 cannot test a battery-limiter-created zero from positive demand; P6 cannot test release/decay from an established positive G53 request. Full host gate is 63/64.
ACTION TAKEN:    Fixed suite source set and bank initialization; retained all three behavior assertions and reran the full gate.
STATUS:          BLOCKING
```

## Jeżeli STOPPED lub BLOCKED

```text
WORK COMPLETED BEFORE STOP:  Suites 41/42 repaired and green; suite 57 now links the active modules and executes; full gate reproduced at 63/64.
STOP / BLOCK CONDITION:      Suite 57 remains red on P6/P7/P8 because the active G53 scenario outputs no M2AA demand.
WHY CONTINUING WAS NOT ALLOWED: Advancing would violate the explicit requirement for a green 64/64 host gate. Changing Phase 3/5 G53 behavior or the frozen contract is outside this rework and would reopen completed phases.
PARTIAL / UNCOMMITTED STATE: Phase 7 production and host-test changes remain uncommitted in the working tree.
SAFE STATE:                  Phase 4 commit `20650bb` and later committed phases remain untouched; no push/merge/flash performed.
WHAT IS REQUIRED TO RESUME:  An accepted deterministic end-to-end G53/M820 scenario vector, or separately authorized scope to diagnose the input-to-BDE8 transition if the frozen chain cannot produce the required output.
```

## Build / Test / Review

```text
BUILD:   PASS (suite 57 compiled and executed)
TEST:    FAIL (full host gate 63/64; suite 57 has three failed behavior checks)
REVIEW:  NOT_RUN
```

```text
VERIFICATION EXECUTED:
  method:          Canonical full host-gate runner
  tool:            python tools/run_host_tests.py
  input:           64 suites registered in tests/host/run-host-tests.ps1
  expected:        64/64 PASS, zero post-step skips
  actual:          63 PASS, suite 57 FAIL, zero post-step skips
  RESULT:          FAIL
  evidence_location: .build/host-linux/REPORT.txt

NOT EXECUTED:
  method:          Later-phase verification and target/HW gates
  reason:           Phase 7 host gate is not green
  impact:            Work cannot advance to a later phase

REGRESSION ASSET CREATED:   YES
REGRESSION_ASSET_LOCATION:  tests/host/ap2_pipeline_scenarios_host.c
HARDWARE TEST:              BLOCKED
VERIFICATION EXECUTION STATE:   FAILED
VERIFICATION REVIEW STATE:     NOT_REVIEWED
```

```text
DISCOVERIES CAPTURED:   NONE
DID ANY DISCOVERY CHANGE CURRENT TASK?   NO
```

**REMAINING WORK:** provide or establish an accepted end-to-end input vector that produces positive G53 M2AA through the actual port facade, then rerun the full host gate.

## NEXT EXACT ACTION

> Obtain an accepted M820 G53 port integration vector that reaches positive M2AA; use it in suite 57 and rerun `python tools/run_host_tests.py`. Do not touch Phase 3/5 source or start Phase 8 until the full gate reports 64/64 PASS.

## Korekty (append-only)

```text
CORRECTED / SUPERSEDED BY: This append-only correction
REASON: Final diagnostic rerun and cleanup completed after the initial report timestamp; no net production/test source change resulted from that probe.
NEW EVIDENCE: 2026-09-25T07:55:04+02:00 — ENDED_AT is 2026-09-25T07:55:04+02:00. The full gate remains 63/64; suite 57 fails only P6/P7/P8. Phase 4 and later committed phases remain unchanged.
```

```text
CORRECTED / SUPERSEDED BY: Phase 7 completion evidence, append-only
TIMESTAMP: 2026-09-25T09:35:45+02:00
REASON: The accepted bounded-reachability vector was integrated into the authorized suite 57 harness.
CHANGE: P6/P7/P8 replay PAS input from the accepted reset block beginning at chain-reference.csv row 13202 through logical tick 1220, with load_ctrl 0 for ticks 0-129 and 6000 from tick 130; raw_pa6_adc=0, assist_level=1, speed_x100=5000, elapsed_ticks=4, phase_current_max=900, and the specified validity/veto inputs.
POSITIVE-STATE ASSERTIONS: M2AA>0, normal_permission=true, iq_request_pre_limits>0, final_iq_request>0, BYPASS/NONE. P7 verifies a battery-limiter-created final zero while native demand stays positive; P6 begins positive and tests release after forward_valid=false.
VERIFICATION: suite 57 PASS; canonical `python tools/run_host_tests.py` PASS (64/64, zero skipped post-steps); `git diff --check` PASS.
SCOPE: This execution changed only the authorized `tests/host/ap2_pipeline_scenarios_host.c`. No Phase 3/4/5/6 or READ_ONLY files were changed in this execution. Phase 4 commit 20650bb, Phase 5 commit 40828eb, and Phase 6 commit e196555 remain unchanged.
PHASE STATUS: Phase 7 host gate is green and ready for local phase commit; independent review remains NOT_RUN. Phase 8 has not yet been verified.
```