# Assist Behavior V3 — Release Report (Release Candidate 1)

```text
DATE:        2026-10-08T09:20+02:00 (build), report written after the final gate
STATUS:      SOFTWARE RELEASE CANDIDATE — PASS (with owner waiver W-001)
             READY FOR CONTROLLED BIKE TEST once the bench CPU gate (RIDE_TEST_PLAN §0 step 5) passes
             NOT production validated: ride-feel acceptance needs the physical M820 ride.
BASELINE:    25df554 (tag baseline/rideable-25df554), builds 0.638 / 0.639
CANDIDATE:   feature/assist-behavior-v3 @ 75b3e94b4de7fa59e6aa391d69320b0dcee0c181
             (firmware sources identical to c7f489b; later commits are docs/tools only)
SCOPE:       Milestones B (shadow), C (active phase-aware release), D (obstacle carry + motion estimate),
             configuration protocol v2. Not in scope: E (mode character activation), F (AUTO/terrain, IMU use).
```

## 1. Firmware candidate

| Build | File (motor-controller-firmware/releases) | SHA-256 |
|---|---|---|
| NORMAL | `0.644_M820_BL820.bin` | `b5da40d1a97f0c4653b9fa692f564710aa144df9bdca4ce3e52f2dee8fa17d6e` |
| DIAG | `0.645_M820_BL820_DIAG.bin` | `ff7f3d12703b99dd4885d21c2d88e35fb693451f4e361fccce5116ddd9402183` |

Both built with `tools/build_firmware.py --mode auto` from a clean tree (manifest `worktree_dirty: false`,
`git_commit 75b3e94`, `assist_v3.compiled_in: true`). Manifests are next to the BINs. A duplicate invocation
allocated **0.646 / 0.647** from the same commit; those numbers are VOID (files moved to `.build/void-builds/`, not
distributed). Ride the DIAG image (0.645) for the controlled test; 0.644 is for after the logs are reviewed.

## 2. What changed for the rider (plain language)

- When you ease off the pedals while still pedalling, assist now drops within about 1/10 to 1/4 of a crank turn plus
  a short smooth fall (simulation: about 0.08–0.16 s to half), instead of lingering for 1–3 s.
- At slow cadence (20–30 rpm) the assist no longer pulses with each pedal stroke; the dead spot of the stroke is
  recognised as normal.
- Steady assist level, acceleration (attack), braking, stopping and back-pedalling behave as on 0.638.
- New: a short, bounded push may continue when you stop pedalling hard on a slow, loaded climb (obstacle carry). It is
  limited to 1.2 s and 1.5 m, never after braking, back-pedalling or when the bike speeds up. In this candidate it is
  calibrated only in simulation, so treat it as something to observe, not to rely on.
- The old behaviour remains selectable over the new configuration protocol (engine = G5300) without reflashing.

## 3. Technical changes

Layers (ARCHITECTURE_V3): crank phase tracker and rider intent with a learned 24-bin phase template, expected-effort
normalised windows and a release classifier (NORMAL / PHASE_DIP / ATTACK / TRUE_RELEASE / PEDAL_STOP); the G5300
static characteristic reused as a pure function (`g53_static_target`) fed by an envelope-equivalent of the measured
stroke; one trajectory owner (`assist_v3.c`) with legacy attack, Response release and legacy stop rates; pipeline-owned
stop/reverse backstop, standstill predicate, engine latch and R1 veto release; obstacle carry state machine with a
pipeline motion estimate; optional IMU seam (inert with valid = false); configuration protocol v2 (0x6035..0x6037,
mode-profile objects, per-level resolver, CONFIG_A append-only log); DIAG telemetry for V3 (schema bump); host runner
selector, SIL scenario matrix, Level-4 freewheel drivetrain and wheel sensor model, relative behaviour acceptance.
Decisions D-001..D-046, reviews REVIEW 1, REVIEW-T, REVIEW 2 (all closed PASS_WITH_ISSUES).

## 4. Evidence

| Gate / check | Result |
|---|---|
| `tools/verify_all.py --require-target` (final, firmware = c7f489b) | **PASS** — manifest, diff check, BL820 packager/container, 85 host suites, Walk/CAN safety, whole-pipeline regression, ripple, SIL + fuzz, electrical SIL, Level-4, replay, CANable decode, stack-gate self-test, exact ARM NORMAL build |
| Stack gate NORMAL / DIAG | PASS — total 3040 B / margin 2080 B (NORMAL), margin ~1910 B (DIAG); stack 5 KB (D-024) |
| RAM NORMAL | ~300 B free above the stack (tight; next lever per D-024) |
| G5300 mode = baseline (G-EQ) | PASS — 150/150 SIL scenarios, SIL fuzz, L4, regression traces byte-identical with V3 compiled in/out |
| Behaviour acceptance (SIMULATION_REPORT §5, `tools/assist_v3_acceptance.py`) | 487/488 — release, phase-dip, level parity ±5 %, attack, stop/reverse/brake, restart, carry correctness PASS; **1 deviation, waived (W-001)** |
| Config roundtrip | PASS — host suite G1-CFG2 (vectors, rejects without mutation, KEEP, generation, persist/restart, erase window, v1 records) |
| Release / restart / carry invariants | PASS — host G1-SAFE/BACKSTOP/SEL/STAND/STOP/VETO/TRAJ, BS9b (mutation-checked), L4 carry scenarios |
| One trajectory owner | PASS — REVIEW 1 / REVIEW 2; G53 shaping not consumed in V3 mode |
| IMU valid = false | PASS — trajectory test T12 bit-identical output and telemetry with garbage |
| Sanitizers | NOT RUN — no ASan/UBSan runtime with w64devkit gcc on this PC |
| DIAG target via verify_all | built by `--mode auto` (0.645) with stack gate PASS; verify_all ran the NORMAL target |

## 5. Owner waiver

**W-001 — 2026-10-08T09:07:12+02:00 — accepted by the owner (Mariusz Głodnicki) via the program question.**
Criterion "per-revolution ripple < 0.25 at <= 30 rpm" fails in one case: dead spot profile at 20 rpm, 0.257
(baseline 0.97; phase-dip false positives 0). Two fixes tested and rejected (SIMULATION_REPORT §5). Waived for the
controlled ride; verified on the bike in manoeuvre R2; further tuning on ride data.

## 6. Limitations and unproven items

- No physical ride yet: ride feel, real stroke shapes, real CPU timing on target (bench gate before riding).
- Carry thresholds are candidates (D-043): CLU→kg scale unverified (CLAIM-004 / TQ-02C); carry fires in simulation only
  on the closed-loop obstacle scenario; it may be inactive on the bike.
- Behaviour matrix covers level 3 only; other levels rely on the shared code and host tests.
- CONFIG_A survival across a BL820 update is [UNKNOWN] (RIDE_TEST_PLAN §0 step 2). If erased, defaults apply: engine V3.
- EN 15194 run-on reference for the backstop bounds is [EXTERNAL_REFERENCE, not verified].
- Absent-record default engine = V3 applies to this candidate only (D-020 conditions); the release default is decided
  after the ride.
- Firmware commits in this repository carry the author "OpenAI Assistant" from a repo-local git setting (baseline
  included); program commits from 4a215b4 onward use the owner's identity explicitly.
- Open TECH items (not blocking the ride): DIAG rate_mode field width; PROJECT_GLOSSARY entries "SPORT+ ratio law" vs
  "AUTO mode"; cadence-0 map edge noted in the Milestone C rework; prior-phase level step 11.5 % at 20 rpm asymmetric.

## 7. Next exact action

Owner: bench steps 1–7 of RIDE_TEST_PLAN §0 with 0.645 (DIAG), including the CPU/dropped-tick gate; if PASS, the
controlled ride R1–R15 with CANable logging. Logs come back to the program for: carry calibration, the 20 rpm ripple
waiver check, and the data needed for Milestone E (mode character) and F (AUTO/terrain).
