# Assist Behavior V3 — Program State

UPDATED: 2026-10-08T09:25+02:00

| Field | Value |
|---|---|
| Baseline | `25df554` (tag `baseline/rideable-25df554`), builds 0.638 / 0.639 — see BASELINE_FREEZE.md |
| Branch / HEAD | `feature/assist-behavior-v3` @ release candidate commit (tag `assist-v3-rc1`) |
| Phase | **RC1 — SOFTWARE RELEASE CANDIDATE — PASS (owner waiver W-001); READY FOR CONTROLLED BIKE TEST after the bench CPU gate** |
| Last passed gate | `verify_all --require-target` PASS on firmware c7f489b; acceptance 487/488 + W-001; REVIEW 2 follow-up PASS_WITH_ISSUES |
| Candidate BINs | 0.644 NORMAL `b5da40d1…7d6e`, 0.645 DIAG `ff7f3d12…2183` (RELEASE_REPORT.md); 0.646/0.647 VOID |

## Frozen decisions

- ARCHITECTURE_V3 rev 3, MODE_CHARACTER rev 2, CONFIG_PROTOCOL_V3 v2 rev 2; D-001..D-046.

## Milestones

| Milestone | State |
|---|---|
| A Foundation (baseline freeze, audits, architecture, REVIEW 1) | DONE |
| B Shadow engine (+ SIL matrix, runner selector) | DONE |
| Mode character design + envelope study + REVIEW-T | DONE (design only) |
| C Active phase-aware release (+ rework) | DONE — in RC1 |
| Config protocol v2 | DONE — in RC1 (no client yet; CANable UI later) |
| D Obstacle carry + motion estimate (+ L4 freewheel plant) | DONE — in RC1, observational |
| E Mode character activation, torque/power envelopes | NOT STARTED — needs ride data (D-046) |
| F Terrain / AUTO / optional IMU use | NOT STARTED |

## Open items

- Bench CPU/dropped-tick gate and CONFIG_A-vs-BL820 check (RIDE_TEST_PLAN §0) — owner, before riding.
- W-001 dead spot 20 rpm ripple (waived for the ride, verify in R2).
- Carry calibration from DIAG logs (D-043); CLU→kg scale (CLAIM-004/TQ-02C).
- TECH: DIAG rate_mode width; glossary entries; cadence-0 map edge; prior-phase step 11.5 % at 20 rpm asymmetric.
- Repo-local git author "OpenAI Assistant" in motor-controller-firmware/.git/config — owner decision.
- Merge points: HMI-SRC3 (`e2868f6`) / WHEEL-BCAST (`0c04f24`) when accepted (CONFIG_PROTOCOL_V3 §7).

## Next exact action

Owner: push the branch and tags, bench steps 1–7 with 0.645, then the controlled ride R1–R15 with CANable logging.
