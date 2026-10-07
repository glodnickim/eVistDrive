# CODEX brief — Torque/power envelope and mode-character study (design-time simulation)

You are an analysis worker (CODEX). The Lead (Claude) reviews and commits. **Do not commit, stage, stash, checkout
or restore anything.** Another CODEX process is concurrently editing firmware sources in the same worktree
(B-PIPE: src/, inc/, sim/, tools/run_*.py, tools/build_firmware.py, tests/host/run-host-tests.ps1). **Do not edit any
existing file.** You may only ADD: `tools/assist_v3_envelope_study.py`, `tests/host/tools/assist_v3_legacy_surface.c`
(or a similar new host dump program), `tests/test_assist_v3_envelope_study.py`, `docs/assist-v3/ENVELOPE_STUDY.md`,
and outputs under `.build/envelope_study/`.

Repository: `C:\Projekty\eVistDrive-assist-v3`. Windows, gcc (w64devkit) as `gcc`, Python 3.12 (check which
packages exist before using numpy/matplotlib; stdlib-only is fine). Use `TEMP`/`TMP` =
`C:\Projekty\eVistDrive-assist-v3\.build\codex-tmp`.

## Read first

- `docs/assist-v3/MODE_CHARACTER.md` (the design you are evaluating — whole file)
- `docs/assist-v3/CONFIG_PROTOCOL_V3.md` §1 (parameter list and ranges)
- `docs/assist-v3/ARCHITECTURE_V3.md` §4.3 (env_equiv), §5 (G5300 static map, Iq conversion), §8
- `docs/assist-v3/SIMULATION_REPORT.md` (baseline numbers)
- `inc/g53_port_chain.h` / `src/g53_port_chain.c` `g53_static_target()` (production pure function of the legacy
  characteristic), `inc/assist_v3_intent.h` (`assist_v3_intent_compute_kl`, `assist_v3_eb74_active`)

## Questions to answer

1. **Legacy reference surface.** Build a small host C program linking the production modules
   (`g53_static_target`, `assist_v3_intent_compute_kl`, `assist_v3_eb74_active`) that dumps a CSV of the legacy
   steady-state demand: for each HMI level 1..5 (S+ AUTO on 4), cadence 20..130 rpm (step 10 + 25), rider mean effort
   I = 200..4000 CLU (≥ 12 values), with the prior stroke template: env_equiv, target_E2, Iq (P = 700), and a rider
   power proxy (I x cadence; also in W using the crank-length/force conversion of `rider_power_w()` in
   src/assist_pipeline.c, labelled [ASSUMPTION: CLU->kg scale unverified, CLAIM-004/TQ-02C]). Motor power proxy:
   Iq x V with V = 48 V nominal, labelled as a proxy. This is the anchor: DEFAULT profiles must be explainable relative
   to it.
2. **Support function family.** Implement in Python the design of MODE_CHARACTER §4.1: desired support S(I, cad;
   base, range_min, range_max, progression) applied to a rider mechanical input that blends torque and power
   (blend weight as a function of cadence, with a high-cadence bias parameter), then Max Torque (% of 0.65·P Iq ceiling,
   i.e. legacy max) and Max Power (W, via the proxy) envelopes. Propose concrete formulas (document them), integer-
   friendly (they will be ported to C).
3. **Profiles.** For each mode (ECO, TRAIL, SPORT, SPORT+, AUTO) propose a hidden profile: per internal parameter
   (value at Assist 0, value at Assist 100, default Assist), plus Response -> (attack, release) and Carry ->
   (strength, time, distance) tendency tables. Fit DEFAULT so that, at the default Assist, steady-state demand
   matches the legacy level within ±10 % over 40..100 rpm and I in the normal range (ECO≈L1, TRAIL≈L2 but with more
   support at 20..40 rpm, SPORT≈L3, SPORT+≈L4 incl. its AUTO-ratio shape, AUTO = TRAIL..SPORT+ range); document every
   deliberate deviation (e.g. more low-cadence torque in TRAIL) and why.
4. **Checks (automated, in the test file):**
   - G2-MACRO monotonicity: Assist 0..100 step 5 x cadence {25, 60, 90, 120} x I grid x every mode: demand
     non-decreasing in Assist before envelopes; after envelopes equal only where an envelope binds.
   - Mode character preserved: the ordering ECO < TRAIL < SPORT < SPORT+ of mean support at default Assist; and each
     mode's curve shape (where it rises) stays recognisable across Assist (define a shape metric).
   - G2-ORTHO on the static model: Max Power changes only points where the power envelope binds (high cadence /
     high effort); Max Torque only where the torque envelope binds (low cadence / high effort); Assist never changes the
     envelope ceilings; Response/Carry have no static effect.
   - Independence of BASIC macros: for each pair, the effect patterns (difference maps over the grid) are not
     duplicates (e.g. correlation < 0.9); report the matrix.
   - Each check must have a self-test proving it can FAIL (inject a non-monotone curve, a duplicate parameter, an
     envelope that leaks into low effort).
5. **ADVANCED candidates.** For max_acceleration, phase_compensation, high_cadence_bias: say from the model whether
   each gives a predictable independent effect or duplicates another parameter. Recommend keep / drop / needs SIL.
6. **Resource sketch.** Profile tables size per mode in bytes (flash const), per-call integer cost estimate.

## Output

- `docs/assist-v3/ENVELOPE_STUDY.md`: method, formulas, legacy surface summary table, proposed profiles (tables),
  check results with numbers, ADVANCED recommendations, open questions, all physical conversions labelled.
- Print a final report (<= 40 lines) to stdout: files added, how to run (exact commands + runtime), key results,
  recommended BASIC set (keep/drop each of the six), open issues. Status READY_FOR_REVIEW.
