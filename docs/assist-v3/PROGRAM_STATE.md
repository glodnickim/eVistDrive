# Assist Behavior V3 — Program State

UPDATED: 2026-10-07 (after milestone B and the mode character study)

| Field | Value |
|---|---|
| Baseline | `25df554` (tag `baseline/rideable-25df554`) — see BASELINE_FREEZE.md |
| Branch / HEAD | `feature/assist-behavior-v3` @ `f38dc68` |
| Phase | MILESTONE B DONE (shadow); mode character design + study DONE; targeted review next |
| Last passed gate | REVIEW 1 closed PASS_WITH_ISSUES (0214a13); G-EQ 150/150 on the SIL matrix harness (7d35f4d) |

## Frozen decisions

- ARCHITECTURE_V3 rev 3 + MODE_CHARACTER; D-001..D-031 in DECISIONS.md.

## Work packages (Milestone B)

| WP | Scope | State |
|---|---|---|
| B-SEL | host runner selector | DONE (5141300) |
| B-SIL | SIL --script, rider_script, metrics, matrix runner | DONE (7d35f4d) |
| B-INT | rider intent | DONE (a00b4a7) |
| B-MAP | static map + accessors | DONE (ac768c4) |
| B-CFG | config v1 owner module | DONE (0ea5f56); rework to v2 (mode profile objects) pending |
| B-PIPE | crank_phase.c, assist_v3.c, pipeline shadow, DIAG telemetry, SIL hook | phase 1 DONE (4ce1be4; G-EQ 150/150, NORMAL/DIAG builds, stack 5K) |
| MC-DESIGN | mode character + config v2 design (owner override) | DONE (60a9bdf) |
| MC-SIM | torque/power envelope + mode character study | DONE (f38dc68; CODEX ran out of credits, Lead completed) |
| REVIEW-T | targeted architecture review (mode character, config v2, study, shadow integration) | running |

Order (owner override, D-031): B-PIPE shadow + G-EQ -> RAM -> mode character design -> envelope simulation ->
advanced config contract -> targeted architecture review -> active release -> carry -> active torque/power/mode
character -> dynamic range/AUTO -> terrain/IMU.

## Open items

- REVIEW 1 residuals R-c (candidate constants → SIMULATION_REPORT) and R-d (CONFIG_A vs BL820 erase, EN 15194
  run-on reference → owner/HW before ride).
- Discoveries captured (not chased): EB74 auto-zero never arms on M820 (A-D8); per-level Iq ceiling inert (A-D1);
  SIL production state survives sim_init (scenario-order dependence); G53 no-assist after unloaded pedalling at speed
  (REVIEW 1 probe, low confidence); 4 P0/P1/P2 multiframe BUG candidates (audit D); `qs_transition_diag_host.c` fails
  at baseline (audit B).
