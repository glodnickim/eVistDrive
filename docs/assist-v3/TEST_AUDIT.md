# Assist Behavior V3 — Test Audit (summary)

Full table: audit/B_TEST_AUDIT.md (all 75 registry suites + harnesses, linked sources, V3-break flag, runtime class).

Baseline 25df554 re-checked 2026-10-07 by the audit, piece by piece: host registry 75/75 (108 s), regression, ripple
analyzer, replay 6/6, SIL, electrical SIL, Level-4 quick, Python tests, NORMAL ARM build, stack-gate self-test — all
PASS. Not run: `verify_all.py` itself, DIAG target, sanitizers (unavailable with w64devkit gcc 14.2, so the baseline's
own PASS skipped them).

| Class | Count (registry suites) | Meaning for V3 |
|---|---|---|
| KEEP-CRITICAL | 57 (51) | must pass at every G1+ gate |
| KEEP-BEHAVIOR | 11 (8) | G5300 oracle; must pass in engine = G5300 |
| ADAPT | 7 (5) | infrastructure reused; scenarios/metrics extended for V3 |
| LEGACY-EVIDENCE | 17 (10) | kept; not part of the V3 fast loop |
| REMOVE/RETIRE | 4 groups, 15 files | proven dead (missing source/include, no caller). Not deleted in this program; candidates for a separate TECH task (move to `archive/`) |

Key facts:

- `ap2_pas_state.c`, `ap2_rider_demand.c`, `ap2_estimators.c`, `ap2_profiles.c` have no production caller.
- `ap2_pipeline_scenarios`, `reverse_ramp`, `fast_slew_i2`, `m560_*`, `m820_uncontrolled` do drive the real G53 path
  despite their names.
- No suite asserts rider-feel metrics today; regression/ripple/replay assert determinism or direction only.
- `qs_transition_diag_host.c` (unregistered) fails at baseline — cause UNKNOWN, separate item.
- `run_host_tests.py` has no selector — added first (D-014).
