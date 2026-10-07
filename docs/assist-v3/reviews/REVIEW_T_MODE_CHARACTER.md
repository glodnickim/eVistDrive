# REVIEW-T — Mode character, config v2, envelope study, Milestone B shadow integration

```text
TIMESTAMP:   2026-10-07T22:49:29+02:00 (system clock, review start)
REVIEWER:    independent Claude subagent, fresh context (read-only; did not author any reviewed artifact)
ROLE:        targeted architecture review (REVIEW-T), challenger
WORKTREE:    C:\Projekty\eVistDrive-assist-v3, feature/assist-behavior-v3 @ 6a284c3 (606cae8 + PROGRAM_STATE cleanup);
             baseline 25df554. No tracked file modified; nothing committed.
SCOPE:       MODE_CHARACTER.md, CONFIG_PROTOCOL_V3.md (draft v2), ENVELOPE_STUDY.md + tools/assist_v3_envelope_study.py
             + tests/test_assist_v3_envelope_study.py, DECISIONS D-001..D-031, ARCHITECTURE_V3 rev 3,
             commit 4ce1be4 (src/assist_v3.c, src/crank_phase.c, src/assist_pipeline.c, src/main.c, src/ride_control.c,
             ride telemetry, linker, build scripts), src/assist_v3_config.c (v1, 0ea5f56), src/g53_port.c,
             src/g53_g1_limiter.c, src/ap2_limits.c, src/assist_modes.c, src/assist_v3_intent.c (cost paths only)
METHOD:      source reading; reproduced:
             - NORMAL build (tools/build_firmware.py --variant normal --mode developer, output in session scratchpad):
               RAM 48312 B / 48 KB (98.29 %), _ebss 0x2000A4B4, stack gate fg 1832 + isr 1152 = 2984 B of 5120 B,
               margin 2136 B (min 1024) -> PASS; free RAM after heap+stack = 0xC000 - 0xBCB8 = 840 B
             - tools/run_sil.py --fuzz 50 (default --assist-v3 both): G-EQ ASSIST_V3 on/off PASS (11 CSVs + fuzz)
             - tools/assist_v3_envelope_study.py: VERDICT PASS (exit 0)
VERDICT:     CHANGES_REQUIRED  (0 BLOCKER, 14 MAJOR, 11 MINOR)
             Shadow integration (section D) is sound for a host/G-EQ gate. The CHANGES are to the mode-character /
             config-v2 contract and to the evidence claims of the envelope study; they must land before the config v2
             rework and before the mode-character milestone. Milestone C coding may start once issues 10, 11 and the
             section E list are in its scope.
```

Severity: BLOCKER = unsafe or prevents the next milestone; MAJOR = would force a rebuild/re-contract later or an
owner requirement is not met; MINOR = fix when touching the area.

## Summary by question

| Q | Result |
|---|---|
| A. MODE_CHARACTER + CONFIG v2 vs owner requirements 1-7 | ISSUES — precedence/legacy rule not well defined (1, 2), ADVANCED set partly duplicate/inert per mode (3, 4), units/ranges inconsistent with the study (5), L5 regression undocumented (6) |
| B. Config v2 wire/storage | ISSUES — CAPS layout collision (7), no partial-update/forward-compat rule (8), RAM headroom (9), engine readback (20), v1 migration (21) |
| C. Envelope study | ISSUES — support model sound and implementable; ORTHO/monotone checks pass by construction (12); g1 is not an exact power owner (10); Max Torque basis/path (11) |
| D. Shadow integration 4ce1be4 | PASS on host (G-EQ reproduced). Target-only timing risk (13); define handling (22) |
| E. Readiness for Milestone C | list in section E |
| F. Rebuild risks for D / mode character / F | 14-19, 23, 24 |

## Issues

### A. Mode character and parameter model

**1. MAJOR — Precedence is stated twice and differently; with either wording the legacy P0/P1 input is unreachable.**
MODE_CHARACTER.md:106-112 says *ADVANCED override > value derived from the BASIC macro (macro DEFAULT or override) >
profile default*, with legacy P0/P1 "adjusting the profile default". CONFIG_PROTOCOL_V3.md:15-16 says *ADVANCED >
BASIC macro derivation > legacy P0/P1 > profile default*. Because a BASIC macro always has a value (its own DEFAULT is
the profile's `default macro`, MODE_CHARACTER §3), "macro derivation" always produces the internal value, so a rule
ranked below it (CONFIG) never fires, and a rule that "adjusts the profile default" of an internal parameter (MODE
CHARACTER) is overwritten by the derivation. Result: the stock app's P0 ratio edit is silently ignored in V3 — exactly
what D-029 promises not to do. "Relative to the factory value" is also undefined (bank default compiled in firmware?
the value CANable wrote back?), and for the SPORT+ slot the P0 "ratio" is the AUTO-law ceiling `maxr`, not a ratio.
*Fix:* define legacy inputs as acting on the **macro**, never on internal parameters, and only while that macro is
DEFAULT. Example (monotone by construction because each internal curve is monotone in the macro):
`Assist_eff = DEFAULT ? clamp(a_def + g_mode(P0_level / P0_factory_fw), 0, 100) : Assist_override`, with `g_mode` the
inverse of the mode's `base(a)` (fixed modes) or `range_max(a)` (SPORT+) curve, `P0_factory_fw` = the compiled bank
default; P1 power % -> `MaxPower_eff = DEFAULT ? profile_W * P1/100 : override`; P1 acceleration -> attack only.
Write the single ordering once (CONFIG_PROTOCOL_V3 §1 references MODE_CHARACTER §6, not a second copy), report
`source = 2` when the legacy value differs from factory, and add G1-CFG2 vectors for each case.

**2. MAJOR — Legacy inputs are per HMI level, V3 parameters are per mode; the configurable level -> mode map makes the
mapping ambiguous.** MODE_CHARACTER.md:36, 109-112; D-029, D-030. With a non-identity map (two levels on one mode,
or L5 -> AUTO while P0 slot 5 is a fixed 525 % ratio) "the profile default adjusted by the legacy value" has two
candidate legacy values or none. Also answer the question the owner will ask: *stock app writes P0 while a V3 Assist
override exists* -> the V3 override wins and the P0 edit has no effect, which the stock app cannot show. *Fix:* (a)
legacy adjustment is evaluated **per level** at resolution time: `effective(level) = resolve(mode(level), legacy(level))`
— the mode object stays per mode, the legacy delta stays per level; the effective view gets an optional `level`
selector; (b) document "a V3 override shadows the stock field" and set a CAPS/STATUS flag `legacy_shadowed` (bit per
level) so CANable can warn; do **not** clear overrides on a P0 write (CANable writes all P0 values back, D-010, which
would wipe every override).

**3. MAJOR — `assist_range_min/max` and `assist_progression` are inert or duplicates of `assist_base` in the fixed
modes.** ENVELOPE_STUDY.md:47-53 and tools/assist_v3_envelope_study.py:164-173: ECO/TRAIL/SPORT/AUTO have `slope = 0`,
`range_max = 1000`, `range_min = 0`, so `ratio = base`. There, progression does nothing, `range_max` does nothing unless
set below `base` (then it is just a second way to lower `base`), `range_min` likewise. This fails the owner's own test
"predictable? independent? not a duplicate?" (requirement 2) for 3 of 5 modes. *Fix (choose one, before the wire is
frozen):* give every mode a non-zero default progression (ratio law with real range, as SPORT+ has), or make
applicability per mode explicit: a per-mode `param_mask` (24 bits × 5 modes in CAPS or in the effective view) and a
source code `5 = not applicable in this mode`; the UI hides inapplicable fields. Do not ship globally-masked fields
that silently do nothing in some modes.

**4. MAJOR — ADVANCED "Start response" duplicates BASIC "Start" and is missing from the wire table.**
MODE_CHARACTER.md:53 (Start drives only "start response") and :91 (ADVANCED Start response = "yes"); CONFIG_PROTOCOL_V3.md
IDs 8-20 have no start parameter. Either Start drives ≥ 2 internals (e.g. start boost level + start rise rate) and
ADVANCED splits it (like Response -> attack/release, Carry -> strength/time/distance), or ADVANCED has no start field
and MODE_CHARACTER §5 says so. Also state explicitly that Start never moves the engage threshold (EB74 820/995,
D-015/§6.2) — today only the table cell "never drives: normal riding" implies it.

**5. MAJOR — Wire units and ranges do not match the study's own internal values.** CONFIG_PROTOCOL_V3.md:27-31 vs
ENVELOPE_STUDY.md:47-53: `assist_base` 0..400 % but SPORT at Assist 100 gives base 512 % (310 × 1.65); `range_max`
0..600 % but internal default is 1000 and SPORT+ A100 is 700; `assist_progression` 0..100 but the internal quantity is
a slope 1.31..5.24 ratio-%/c4-unit with no mapping defined. The effective view (view 1) would have to return values
outside the declared write range, and an override of `range_max = 600` in SPORT+ would *lower* the A100 behaviour.
*Fix:* define every ADVANCED unit as the internal unit or with an explicit, monotone mapping (and its inverse for the
effective view); ranges must contain every profile value at Assist 0..100; add a G1-CFG2 vector "effective value of
every parameter at Assist 0/50/100 is inside its write range".

**6. MAJOR — L5 loses ~60 % of its support at default and this is not listed as a deliberate deviation.** Legacy L5
is a fixed 525 % ratio (tools/assist_v3_envelope_study.py:44, "BOOST" in src/assist_v3_config.c:36). D-030 maps
L5 -> AUTO, and the AUTO profile is the TRAIL point (base 215 %, ENVELOPE_STUDY.md:53). ENVELOPE_STUDY §5 lists three
deliberate deviations but not this one, and nothing in MODE_CHARACTER keeps a strong fixed level. *Fix:* owner decision
recorded in DECISIONS (supersede D-030 or add the deviation), and either give AUTO a default point near the legacy L5
characteristic or keep L5 on SPORT+/a BOOST profile until F.

### B. Config v2 wire, storage, RAM

**7. MAJOR — CAPS/STATUS layout collides when `param_mask` is widened.** CONFIG_PROTOCOL_V3.md:77-81 keeps "[0..21] as
audit D §5.2" but widens `param_mask` to u32 at [16..19]. In audit D §5.2 (audit/D_CONFIG_CAN.md:129) [18..19] was
`config_generation`, [20] `persist_state`, [21] `flash_record_state`. Widening pushes them to [20..21], [22], [23],
which are also assigned to `engine_active` [22] and `engine_requested` [23]; CRC is over [0..23]. The total must
become 28 B. "Exact offsets are fixed in the implementation header" makes the code the contract — the reverse of the
governance (contract precedes implementation, RULE 14). *Fix:* write the full v2 byte table in CONFIG_PROTOCOL_V3 §3
(28 B, format 2), including what [7] (object mask) and [10] hold now; shared test vector for format 1 and format 2.

**8. MAJOR — No rule for partial updates or for clients that know fewer parameters; "exact schema_version" contradicts
schema_min/max.** CONFIG_PROTOCOL_V3.md:83-89, 102-104, 117. A write replaces all 24 values. A BASIC-only phone app, or
any client built against a 24-parameter firmware writing to a 32-parameter one, must echo values it does not
understand; if it writes `0xFFFF` it wipes ADVANCED overrides. "Back-fill with 0xFFFF" is defined for storage only.
*Fix:* reserve a per-parameter sentinel `0xFFFE = keep current value` (accepted for any ID, never stored), accept
`schema_version` in `[schema_min, schema_max]`, and define that a shorter `param_count` means "missing = keep". Return
the new `config_generation` in the NORMAL_ACK (2 bytes) so a multi-object sequence does not need a CAPS read between
writes. Optional: persist op carries the expected generation, so a persist issued by client A cannot silently store
client B's interleaved writes.

**9. MAJOR — RAM headroom for v2 is ~0.5 KB and the plan does not budget it.** Measured: 840 B free (NORMAL,
5 KB stack). v1 holds `ram_values` 96 B + `saved_values` 96 B + `xfer` 128 B (nm). v2 per-mode objects need
5 × 24 × 2 + global ≈ 256 B per image, i.e. +~320 B for two images, leaving ~520 B for carry state (D), motion
estimator (D), terrain state (F) and any effective-value cache. *Fix:* drop `saved_values` — view 0 and the DIRTY test
read the newest valid slot directly from memory-mapped CONFIG_A (saves ~256 B); resolve effective values only for the
active mode (48 B cache, recomputed on generation or level change); record the budget per milestone in
ARCHITECTURE §12 and keep D-024's "rejected" list (STOP_TRACE 27 KB `R`) as the next lever.

### C. Envelope study and envelopes

**10. MAJOR — g1 is not an exact battery-power owner, and W -> g1 is not "limit = W / V_batt" in the existing code.**
ENVELOPE_STUDY.md:34, 119-120; MODE_CHARACTER.md:75-76. src/g53_port.c:97-103, 160-165 and
src/g53_g1_limiter.c:123-174: the g1 limit is `battery_limit_centiamp (global 15 A) × level_pct / 100`, `level_pct` is
a u8 per HMI slot clamped to 100, updated every 10 logical ms; g1 itself is a PI on measured battery current with a
ramped setpoint (soft start ~190 ms, audit A G11) — it overshoots on an attack and settles; it is not a static ceiling.
Implementing W needs a new per-mode, voltage-dependent `level_pct` input (or a reconfigured base) in g53_port.c (port
glue, allowed by D-003) and creates a V-sag feedback (lower V -> higher current allowance). Also: with W semantics
SPORT+ default 720 W is *below* 15 A at a full pack (54.6 V -> 819 W), so the default changes legacy P1 100 %
behaviour; and a cap below the SOC knee (50 % of 15 A ≈ 7.5 A, e.g. ECO 350 W ≈ 7.3 A) makes the SOC derate inert for
that mode (r7 < 0 branch, g53_g1_limiter.c:144-149). ARCHITECTURE_V3.md:372 still says "Max Power stays P1 % via g1".
*Fix:* rewrite the claim as "closed-loop battery-current limit via g1, steady-state accurate, transient overshoot
bounded by the PI (measure)"; add a `0xFFFE`-style or "HW MAX" default so a mode can mean "no extra cap" (keeps legacy
parity); state the SOC-knee interaction; update ARCHITECTURE §8 with a decision (D-0xx) superseding the "P1 % only"
line; add G2-ORTHO transient cases (Max Power must not change classification/start even while g1 is acting) on the
SIL closed loop, not the static model. Consider `ap2_limits` `max_power_w` (src/ap2_limits.c:57-80, a static
motor-power cap already in the chain, currently passed 0) as the explicit alternative in that decision.

**11. MAJOR — Max Torque via `level_iq_limit`: path exists but its basis and side effects are wrong for V3.**
src/assist_pipeline.c:218 forces `AP2_LIMITS_NO_LEVEL_CEILING` (audit A-D1). The value the pipeline would otherwise
use (src/ride_control.c:330-332, src/assist_modes.c:421-441) is `min(ride_core_iq_limit, max_iq_pct% × phase_current_max)`:
(a) basis is % of P = 700, while the study uses % of 0.65·P = 455 (tools/assist_v3_envelope_study.py:148) — with the
legacy basis every Max Torque ≥ 65 % does nothing; (b) `ride_core_iq_limit` carries the low-SOC `limp_factor`
(src/main.c:1467), which is inert on PEDAL today (SOC derate is g1's job, ADR-013) — wiring `level_iq_limit` would
re-activate a second SOC derate in V3 mode only; (c) the deprecated bank field `max_iq_pct` (hidden by clients,
CONFIG_PROTOCOL_V3.md:52-54) would silently cap V3. *Fix:* in V3 mode the pipeline computes
`level_iq_limit = MaxTorque_eff% × 0.65 · P / 100` from the V3 resolver only; never from `pipe_in.level_iq_limit`;
define "% rated" in CONFIG_PROTOCOL_V3 as "% of the V3 demand full scale 0.65·P"; G-EQ in G5300 mode unchanged.

**12. MAJOR (evidence) — The static ORTHO and monotonicity checks pass by construction; they are not evidence of
orthogonality.** tools/assist_v3_envelope_study.py:145-152, 232-255: `v3_iq = min(raw, torque_cap, power_cap)` with
caps that do not depend on Assist, and the leak test asks whether lowering one cap changes a point the new cap does
not bind — impossible for a `min()`. The self-test (tests/test_assist_v3_envelope_study.py:65-72) proves the checker
detects a different, non-`min` model, not that the design is orthogonal. Monotonicity likewise follows from monotone
geometric curves and a monotone integer map. The Jaccard gate only requires `max_power~max_torque < 0.9`. The
substantive questions (g1 PI dynamics vs release/start, ceiling slew vs R1, ratio limiter G7 under Assist changes,
integer order in firmware) are untested. ENVELOPE_STUDY.md:73-76 and :108 present these as PASS of G2-MACRO/G2-ORTHO.
*Fix:* relabel as `[SIM-STATIC, BY CONSTRUCTION]` "design consistency" checks (RULE 67/70); G2-ORTHO and G2-MACRO stay
OPEN until run on SIL/L4 with the firmware resolver and g1; keep the replica-vs-production and default-parity results
(those are real evidence, reproduced: VERDICT PASS). The support model itself (generalised ratio law + `cad_eff`
crossover through `g53_static_target`) is sound: `g53_static_target` (src/g53_port_chain.c:2529-2619) takes
`cadence` and `lut_cadence` separately, so `cad_eff` can be injected; the ratio law needs a new optional input (e.g.
`const v3_ratio_law_t *law`, NULL = legacy slot) — specify it now (see 15). Note `cad_eff = max(cad, c_floor)` also
applies at measured cadence 0..20 during a start from rest (SPORT+ ≈ 3× legacy start support): document it as part of
deviation §5.1 and include "start from rest" in G2-ORTHO for Start vs c_floor.

### D. Shadow integration (4ce1be4)

**13. MAJOR — Foreground CPU burst on revolution completion is unmeasured; "V3 cannot affect the published command"
holds on host only.** src/assist_v3_intent.c:285-299 (`try_align`: 96 offsets × 24 bins × `bin_sum` of 4 ring reads
with `% 96`, plus 2304 u64 MACs), called every revolution while unaligned (:323), plus learning and up to two kL
recurrences (≤ 400 iterations each, :229-235) in the same call. Estimate [INFERRED]: ~150k cycles ≈ 1.2-1.4 ms at
120 MHz, i.e. ~5 periods of the 4 kHz foreground. On target this shifts G53 catch-up (and A-D7 edge loss in the G53
PAS replay) — a published-path change the virtual-time G-EQ cannot see. *Fix (before any ride of a V3-compiled image,
shadow included):* DWT max-cycles per `assist_v3_update` in the DIAG build plus `dropped_logical_ticks` A/B vs
`--assist-v3 off`; spread `try_align` over several calls (e.g. 12 offsets per call) or correlate coarse-to-fine
(24 bin offsets, then ±2 steps); compute bin sums once per revolution. State a per-call cycle budget in ARCHITECTURE §12.

Verified PASS (no issue): `ctx.v3_iq_demand` is written only (src/assist_pipeline.c:174, 211); the V3 stage reads
chain state via accessors after `g53_port_update` and writes only V3 state; G-EQ ASSIST_V3 on/off reproduced in SIL.
Trajectory units in src/assist_v3.c:70-111 are correct (Q8·2^16 per ms × ticks / 4; BDE8 P/200 Iq/ms, D3E/D+232 E2 per
10 ms -> step·P·13/32000 Q8/ms, Response 0.65·P/T). crank_phase: wrap-safe u32 count, INVALID jumps and ring overflow
both set the glitch flag, snapshot and pipeline run in the same pass (src/main.c:2787, 3026-3032, 3165-3175). DIAG frames
are compiled only with `ASSIST_V3 && ASSIST_V3_SHADOW_TELEMETRY && CAN_RIDE_TELEMETRY_ENABLE` (inc/config.h); NORMAL
frames unchanged. Stack: reproduced 2984 B worst, 2136 B margin at 5 KB; the gate is binary-level and conservative on
indirect calls (tools/m820_stack_gate.py), so the CONFIG_A function-pointer HAL is covered. CAN config handlers run in
main-loop context (src/main.c:1378), so the v1 commit is atomic with respect to the pipeline.

## E. What Milestone C (active phase-aware release) must contain — and fix first

Not implemented yet and required before V3 publishes (all V3-mode only; G5300 mode stays byte-identical):

1. Engine latch (ARCH §2.2): latch `engine_requested -> engine_active` only when the published request, V3 `y` and the
   shadow `iq_request_pre_limits` are 0 and no veto; `pulled_down = true` on the switch tick. Today
   `assist_v3_config_init` sets `engine_active = requested` at boot (src/assist_v3_config.c:443) and nothing latches.
2. Publication: `req = (y · g1) >> 12` with this tick's `ctx.g53.trace.g1`, then ap2_limits; `ctx.pulled_down` and R1
   computed against the post-limit **V3** request (ARCH §2.1 M2).
3. Pipeline-owned backstop (§7.3, D-008) with its own state, independent of assist_v3.c: reverse decay ≥ 3.5 Iq/ms from
   the published value, crank-stopped hold ≤ `T_STOP_HARD` then ≤ 300 ms decay, re-open only after forward steps at
   ≤ 3.5 Iq/ms; G1-BACKSTOP with V3 forced to max (also at speed 0).
4. Standstill predicate replacing `stock_hard_zero` (src/assist_pipeline.c:284) in V3 mode:
   `speed_native <= 0 ∧ (direction_inhibit ∨ (crank stopped ∧ assist_v3_stop_target_zero()))` -> FORCE_ZERO.
5. Response source: per **mode** through a resolver API (`assist_v3_effective(level, PARAM_RESPONSE)`), not the v1
   per-level accessor (src/assist_pipeline.c:166). Decide whether C ships on config v1 or v2; if v1, C's BIN carries a
   wire that is superseded before any client — then do not let CANable implement v1.
6. Response in C drives release only (MODE_CHARACTER.md:142). Either expose ADVANCED `release` (ID 13) as the C consumer
   and keep `response` (ID 3) masked until attack exists, or document that the meaning of a stored Response value
   widens later (param_mask rule, CONFIG_PROTOCOL_V3.md:55-56).
7. Telemetry: emission condition `engine_active == V3`; add `backstop_iq` and published `final_iq` (ARCH §11 lists them,
   the V3 frame group does not carry them).
8. Issue 13 measured and within budget; issue 20 fixed (truthful engine readback).
9. Host tests: pipeline suites (tests/host/pipeline/*) built **with** `-DASSIST_V3=1` in both engines, plus the existing
   G-EQ without it; G1-SAFE, G1-BACKSTOP, G1-LEVEL, stop/reverse timing ≤ baseline, R1 after native_cut/assist-off/
   engine switch.
10. Assist off: `v3.level = 0` makes `y` fall at the Response rate while the published value is 0 (BYPASS); on
    re-enable R1 governs — add a test that re-enabling never publishes the stale `y` as a step.

## F. Rebuild risks for carry (D), mode character, AUTO/terrain (F)

**14. MAJOR — Specify the resolver and the `g53_static_target` extension now.** Every later milestone needs
"effective parameter of the active mode for this level" and an injected ratio law + `cad_eff`. Without a fixed API
each milestone re-plumbs `assist_pipeline.c` (today: `assist_v3_config_response_pct(level)`), and the ratio law
risks a second characteristic beside `g53_static_target`. Define `assist_v3_effective_t` (all 24 values + sources for
the active level) and `g53_static_input_t.law` (NULL = legacy) in ARCHITECTURE §5 before config v2 is coded.

**15. MINOR — G7 ratio-limiter step and D7EC rise rate are taken from the HMI slot** (src/assist_v3.c:151, 259). Once the
level -> mode map is active they must come from the mode (or the mode's attack parameter), else re-mapping a level
mixes one mode's characteristic with another slot's attack.

**16. MINOR — DIAG `rate_mode` field is full.** 8 rate modes in 3 bits (inc/assist_v3.h:71-78,
src/ride_telemetry.c V3C d6). Carry adds at least CARRY and CARRY_RELEASE; reserve the schema bump now (V3 schema 2:
rate_mode 4 bits, take one bit from the schema field or move it to a new byte).

**17. MINOR — "AUTO" names two different concepts.** Legacy G5300 "S+ AUTO" = the progressive ratio law (now SPORT+'s
character); V3 AUTO = terrain-chosen operating point. Add both to PROJECT_GLOSSARY (RULE 48) and use "SPORT+ ratio law"
for the former in V3 documents.

**18. MINOR — "Mode keeps its character across the Assist range" (TEST_MATRIX G2-MACRO) has no metric.** The study
shows heavy overlap (ENVELOPE_STUDY.md:59-62: ECO A100 at env600@60 = 98 Iq ≈ SPORT A0 = 97; SPORT+ below SPORT at
light effort). Define the invariants per mode (e.g. ordering of attack/release rates, power ceiling, low-cadence ratio
shape, progressive vs flat) so the gate can fail.

**19. MINOR — AUTO before F is a static TRAIL-like point** (ENVELOPE_STUDY.md:53). Requirement 7 (algorithm chooses the
operating point; no IMU must not disable AUTO) is only met in F. State in MODE_CHARACTER §9 what AUTO does between the
mode-character milestone and F, and that the F terrain state must have a non-IMU source from its first version
(ARCH §8 already says so — keep it as an acceptance item).

### Other MINOR findings

**20. MINOR — Engine readback is untruthful in the shadow build.** `engine_active_v3 = engine_requested()` at init
(src/assist_v3_config.c:443) and the absent-record default is V3 (D-020), so CAPS [22] and the DIAG flag
`ENGINE_ACTIVE` report V3 while G5300 publishes every Iq. A shadow ride log would be misread. Report 0 until Milestone C
activation exists; in an image built with `--assist-v3 off` (config module still compiled, sources-m820.txt) reject
`engine = V3` with reason 4 and clear the capability bit (D-012: never accept and ignore).

**21. MINOR — v1 -> v2 migration undefined.** A bench image of 0ea5f56..6a284c3 may already hold a v1 record ('A3CF',
schema 1, 128 B slots) in CONFIG_A. "Back-fill with 0xFFFF" does not apply (different object model). Define: schema 1 =
absent (defaults), flash untouched until the next explicit persist; test vector.

**22. MINOR — `#ifdef ASSIST_V3` accepts `-DASSIST_V3=0` as "on".** inc/config.h already guards
`ASSIST_V3_SHADOW_TELEMETRY` with `#if` + `#error`; do the same for `ASSIST_V3` (default in config.h, `#if ASSIST_V3`),
so a mis-typed define cannot build a NORMAL image whose V3 state differs from the build_info record. Two build entry
points set the define (tools/build_firmware.py, scripts/build-firmware.ps1) — RULE 22: mark the .ps1 as non-canonical
or derive its define list from the Python tool.

**23. MINOR — Persist semantics.** `0x6037 op 1` persists whatever RAM holds at the next standstill
(src/assist_v3_config.c:604-607), not the state at the op. Document it, expose the persisted generation in STATUS, and
add the "multiple objects then one persist" sequence to G1-CFG2.

**24. MINOR — Source codes need "not applicable / shadowed".** CONFIG_PROTOCOL_V3.md:92-94: add `5 = not applicable in
this mode` (issue 3) and `6 = shadowed by an ADVANCED override` (e.g. Assist in ECO when `assist_base` is overridden —
the Assist slider then does nothing and the UI must say so); define that "effective" is the resolved configuration,
never the momentary g1/thermal-limited value (that is telemetry).

**25. MINOR (governance) — D-011 is refined in substance by D-028/D-029 but not marked.** DECISIONS.md:54-55 must carry
`SUPERSEDED (partially) by D-029` (RULE 18); ARCHITECTURE_V3 §8 Max Power line likewise (issue 10).

## Requirement trace (owner override)

| Req | Status |
|---|---|
| 1 autonomous modes | design OK; G2-AUTONOMOUS defined; not yet evidenced (Response/Start/Carry transient-only) |
| 2 BASIC/ADVANCED tests | gaps: 3, 4, 5 |
| 3 Assist macro | design OK and monotone by construction; legacy interplay (1, 2) and shadowing (24) open |
| 4 DEFAULT + override | design OK; wire gaps 7, 8, 21, 23; RAM 9 |
| 5 tests | defined in TEST_MATRIX G2-*; static study is not G2 evidence (12); metric missing (18) |
| 6 layer separation | OK on paper; real couplings to test: g1 PI vs transient (10), limp via level ceiling (11), c_floor vs Start (12) |
| 7 AUTO | only in F (19); no-IMU path specified in ARCH §8 |

## NEXT EXACT ACTION

Lead: revise MODE_CHARACTER §5-§6 and CONFIG_PROTOCOL_V3 §1-§4 for issues 1-8, 10, 11, 24 (one precedence text, per-level
legacy resolution, per-mode applicability, units/ranges, 28 B CAPS table, keep-sentinel, generation in ACK), add
DECISIONS entries for Max Power owner (10), Max Torque basis (11) and L5 (6, owner), relabel ENVELOPE_STUDY §4 checks
(12); in parallel start Milestone C with the section E list and the DWT measurement of issue 13. Re-check of 1-12 by
an independent reviewer before the config v2 rework is coded.
