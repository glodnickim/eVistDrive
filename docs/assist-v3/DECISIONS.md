# Assist Behavior V3 — Decisions

Each decision: timestamp, decision, alternatives rejected, why. Superseded decisions stay, marked SUPERSEDED.

### D-001 — 2026-10-07T12:15:18+02:00 — Baseline and branch
Baseline `25df554` (tag `baseline/rideable-25df554`); V3 on `feature/assist-behavior-v3` from it.
Rejected: branching from `0c04f24` (WHEEL-BCAST) — it carries HW-pending CAN changes unrelated to assist.

### D-002 — 2026-10-07 — One trajectory owner at the Boundary-B seam
`assist_v3_update()` inside `assist_pipeline_update()` replaces `iq_request_pre_limits` in V3 mode; the G53 chain runs
on as shadow and fallback.
Rejected: (a) writing V3 into D7EC output — BDE8 ±50/ms stays a second shaper, breaks port bit-exactness;
(b) shaping `load_ctrl` before G53 — the D7EC envelope still holds; (c) inside fast_iq_slew — ISR, no sensors, mixes
behaviour with the electrical guard. Source: audit A §8.

### D-003 — 2026-10-07 — Transcribed G53 code is not edited
V3 adds read-only accessors and one pure static-map function. G53 tests (H66-H72) stay as the G5300 oracle.
Rejected: refactoring D7EC into stages — loses TQ-02B/TQ-04 parity evidence for no behavioural gain.

### D-004 — 2026-10-07 — Keep the G5300 static characteristic in Milestone C
Legacy ratio, AUTO, LUT(cadence) and accel rise rates are reused; only interpretation and trajectory change.
Rejected: a new torque-proportional characteristic now — mixes two changes and makes the A/B ride uninterpretable.
Revisited in Milestone E.

### D-005 — 2026-10-07 — Intent = expected-effort-normalised windows over a learned phase template
`E_W = Σobs / Σs[bin]`; full revolution = robust mean; short angle window = fast change detection.
Rejected: (a) single ms timeout — cadence-dependent, the user explicitly excluded it; (b) obs/s[bin] per step —
divides by ~0 in dead spots; (c) plain revolution mean only — 2.4 s lag at 25 rpm; (d) Kalman/harmonic fit — more
CPU and tuning for no shown benefit; can be revisited if the matrix shows template limits.

### D-006 — 2026-10-07 — Phase from the native PAS event queue, not from the G53 PAS replay
Signed step accumulator + last-step tick from the main.c PAS event drain.
Rejected: G53 PAS evidence counters — lose edges under foreground catch-up (audit A-D7).

### D-007 — 2026-10-07 — Intent observation = `load_ctrl` (linear CLU), mapped to EB74 units for the static map
Rejected: EB74 `cur` as observation — its hard-reset deadband distorts the low-effort template.

### D-008 — 2026-10-07 — Pipeline-owned stop/reverse backstop in V3 mode
Reverse: decay at >= BDE8 rate. Crank stopped: hold at most `T_STOP_HARD` / `D_STOP_HARD` (candidates 1500 ms / 2.0 m)
then decay <= 300 ms. Standstill: FORCE_ZERO. Only in V3 mode; G5300 mode unchanged (OWNER-DEC-2026-10-06-G5300-ONLY
still applies there).
Why: in V3 mode the G53 stop logic is not consumed, so nothing outside V3 would bound a stuck demand (audit A §10).
It is a decaying ceiling, not a cut, so it does not bring back the abrupt on/off the owner decision removed.
Rejected: trusting V3's own carry caps alone — a V3 bug would be unbounded.
Open: bounds vs EN 15194 run-on [EXTERNAL_REFERENCE, not verified]; owner confirms before the ride.

### D-009 — 2026-10-07 — Telemetry only in DIAG builds
Rejected: adding V3 fields to NORMAL frames — bus load, CANable decode churn.

### D-010 — 2026-10-07 — New versioned config IDs 0x6035..0x6037 with own flash record in CONFIG_A
Rejected: spare P0/P1/P2 bytes (clobbered by CANable write-back), bank v11 (255 B limit, parser lag), growing
`MotorParams_t` (wipes all settings). Source: audit D §5.4.

### D-011 — 2026-10-07 — Assist ratio, Acceleration, Max Power keep their M560 P0/P1 owner
The V3 block carries only new parameters. Rejected: mirroring them in the V3 block — second owner.

### D-012 — 2026-10-07 — Reserved parameters are rejected unless 0xFFFF
Rejected: accepting and ignoring — a client would believe a setting works.

### D-013 — 2026-10-07 — Simulation harness: extend SIL (primary) + Level-4 (closed loop); no new simulator
Baseline vs candidate = same harness on `git archive 25df554` sources vs V3 tree, identical scripts; candidate with
`engine = G5300` must be byte-identical to baseline. Source: audit C.

### D-014 — 2026-10-07 — Test runner selector before V3 code
`run_host_tests.py` gets `--list/--only/--exclude/--group/-j/--opt` (defaults unchanged). Needed for Gate 0/1.
Stale suites (audit B REMOVE/RETIRE) are not deleted in this program; they are listed as LEGACY in TEST_AUDIT.md.

### D-015 — 2026-10-07 — Physical intent `I` and envelope-equivalent `env_equiv` are separate (REVIEW 1 #1, #2)
`env_equiv` = steady-state mean of the exact D7EC recurrence over the template-reconstructed stroke, with the active
EB74 threshold; feeds only the G5300 static map. `I` is never rescaled. Acceptance G1-LEVEL ±5 % of baseline.
Rejected: (a) revolution mean straight into the map — 21-59 % less assist (REVIEW 1 probe P2); (b) one fixed scale
factor — shortfall depends on cadence and stroke shape; (c) learning a gain from the shadow envelope — couples V3 to
the shadow's state and contaminates during transients.

### D-016 — 2026-10-07 — Prior template instead of uniform fallback; fallback classifier without a short window (#5)
Rejected: `s ≡ 1` fallback — TRUE_RELEASE twice per revolution after every start/glitch/reverse and below 30 rpm.

### D-017 — 2026-10-07 — Template learning gated on revolution stability (#6), re-alignment by cross-correlation (#15)
Rejected: learning only in NORMAL_PRESSURE — locks out adaptation to a changed pedalling style.

### D-018 — 2026-10-07 — R1 veto-release limiter kept in V3 mode (#3); backstop re-opens at the BDE8 rise rate (#4)
Rejected: re-seeding `y` from the published value — limiters would change legal-speed behaviour.

### D-019 — 2026-10-07 — Milestone C stop rates are legacy (#9); standstill predicate written explicitly (#7)
SUPERSEDED 2026-10-07 (re-check N2: cut load-held stops at speed 0 in ~21 ms vs baseline 1022 ms):
`standstill_zero = speed_native <= 0 ∧ (G53 true-stop ∨ real_stop ∨ direction_inhibit)`.
Now: `standstill_zero = speed_native <= 0 ∧ (direction_inhibit ∨ (crank stopped ∧ V3 stop target == 0))`.
Milestone D swaps `speed_native` for the motion estimate.

### D-020 — 2026-10-07 — Engine switch latch and absent-record default (#10, #18)
Latch only when published request and both engine demands are 0 and no veto; `pulled_down = true` on the switch tick.
Absent-record default in the candidate build = V3. REVIEW 1 recommended G5300; rejected because CANable has no V3 UI
yet, so a G5300 default would make the candidate BIN ride baseline behaviour; the fallback is reflashing 0.638.
Owner may override. Re-check conditions: (a) candidate/test build only, the release default is decided in
RELEASE_REPORT; (b) RIDE_TEST_PLAN states the on-trail fallback (level 0 / power off, then reflash 0.638 or a raw
`engine = 0` write); (c) once a record is persisted the absent-record default no longer applies.

### D-021 — 2026-10-07 — Production `crank_phase.c` shared by main.c and all harnesses (#19)
Rejected: harness copies of the PAS drain — the matrix would not test production code.

### D-022 — 2026-10-07 — Motion estimator (wheel pulses + erps × learned ratio) owns distance/accel for carry (#13)
Rejected: distance caps from wheel pulses (1 pulse = 2.218 m, 0 below 3 km/h). Implementation in Milestone D.

### D-023 — 2026-10-07 — Envelope factor defined in the load domain and clamped (re-check N1)
`kL = clamp(L_eq / I_rev, 1.0, 2.5)`, prior value below ~300 CLU, `env_equiv = EB74_active(kL * I)`.
Rejected: factor after the EB74 deadband (rev 2) — unbounded near the threshold, ~2.8x surge on light spin then attack.

### D-024 — 2026-10-07 — RAM budget for V3: reduce the reserved stack 6 KB -> 5 KB when B-PIPE lands
Measured on the B-CFG target build: static RAM ends at 0x2000A2D8, heap 1 KB, stack 6 KB; only 296 B free. The
stack gate (tools/m820_stack_gate.py) proves worst case foreground + ISR = 2920 B and requires margin >= 1024 B.
At 5 KB the margin is ~2200 B (> 2x the required minimum) and 1 KB is freed for V3 (~0.4-0.6 KB). The gate is re-run
on every target build, so a V3 stack increase is caught.
Rejected: shrinking STOP_TRACE (27 KB diagnostic recorder in NORMAL builds, owner tooling depends on it); removing
the 1 KB heap (newlib malloc is linked; runtime use not proven absent); squeezing V3 buffers below the spec.
