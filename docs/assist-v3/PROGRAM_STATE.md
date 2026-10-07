# Assist Behavior V3 — Program State

UPDATED: 2026-10-07T12:15:18+02:00

| Field | Value |
|---|---|
| Baseline | `25df554` (tag `baseline/rideable-25df554`) — see BASELINE_FREEZE.md |
| Branch / HEAD | `feature/assist-behavior-v3` @ `a3fdf6f` (milestone A docs) |
| Phase | MILESTONE A — audits A-D done; ARCHITECTURE_V3 frozen candidate; REVIEW 1 running; tooling (test selector, SIL V3 matrix) in progress |
| Last passed gate | baseline re-check by audit B (75/75 host, SIL, L4, replay, NORMAL build) |

## Frozen decisions

- D-001..D-014 in DECISIONS.md (frozen candidates until REVIEW 1).

## Open blockers

- Steady-state assist level: V3 intent is a revolution mean, G5300 envelope is near-peak → needs an
  envelope-equivalent normalisation before Milestone C (pending REVIEW 1).

## Next

- REVIEW 1 verdict → fix architecture → Milestone B (shadow engine).
