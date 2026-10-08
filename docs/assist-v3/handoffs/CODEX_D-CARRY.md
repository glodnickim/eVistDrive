# CODEX brief — Milestone D: obstacle carry / intelligent overrun + motion estimate

You are a firmware worker (CODEX). The Lead (Claude) reviews and commits; an independent review follows at the
release candidate. **Do not commit, stage, stash, checkout or restore anything.**

Repository: `C:\Projekty\eVistDrive-assist-v3` (branch `feature/assist-behavior-v3`). Windows, gcc as `gcc`, Python 3.12,
ARM toolchain `C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\13.2 Rel1\bin`. `TEMP`/`TMP` =
`C:\Projekty\eVistDrive-assist-v3\.build\codex-tmp`. Start by reading `git log --oneline -15` and
`docs/assist-v3/PROGRAM_STATE.md`.

## Normative

- `docs/assist-v3/ARCHITECTURE_V3.md` §7 (carry score §7.1, state machine §7.2, backstop §7.3, motion estimate §7.4),
  §2.1 (standstill predicate; Milestone D swaps `speed_native` for the motion estimate), §9 safety, §11 telemetry.
- `docs/assist-v3/MODE_CHARACTER.md` §4 (Carry macro -> strength, time cap, distance cap), §2 invariants (carry
  strength TRAIL ≥ SPORT, SPORT+ highest, ECO small), §10.
- `docs/assist-v3/CONFIG_PROTOCOL_V3.md` IDs 5, 16, 17, 18 and the resolver `assist_v3_effective()`.
- `docs/assist-v3/DECISIONS.md` D-008, D-019, D-022, D-039, D-041.
- `docs/assist-v3/TEST_MATRIX.md` G1-CARRY, G1-BACKSTOP, G1-STAND; REVIEW-T #16 (DIAG rate_mode is full: bump the V3
  telemetry schema to give rate_mode 4 bits).

## Scope

1. `src/motion_est.c` + `inc/motion_est.h`, pipeline-owned: `speed_est` (0.01 km/h), `distance_est` (mm, wrap-safe),
   `rel_accel` (per-mille per second), quality flag. Sources: wheel pulses (tick of last pulse, interpolation between
   pulses), and while the motor drives, `erps × ratio` where `ratio = wheel speed / erps` is learned while pedalling at
   speed ≥ 5 km/h with steady cadence (gear cannot change while the crank is stopped); `rel_accel = Δerps/erps`.
   IMU input reserved (sanitised motion seam, unused while valid = false — prove bit-identical with garbage).
2. Carry score (bounded Q12) per §7.1 from recent intent/peak, attack, cadence before stop, motion estimate, motor load
   (measured Iq / P), recent assist; frozen at the PEDAL_STOP transition.
3. Carry state machine per §7.2 inside `assist_v3.c` (the trajectory stays the only rate limiter; carry is a target
   profile that falls slower than R(Response)); caps from the resolver: Carry macro → strength / time / distance with
   firmware hard maxima `CARRY_HARD_MAX_MS ≤ T_STOP_HARD (1500 ms)` and `CARRY_HARD_MAX_MM ≤ backstop distance bound`
   (`static_assert`). Cancels: brake, reverse step, native_cut, assist off, `rel_accel` above threshold (bike clearly
   accelerating). Restart → NORMAL from the current `y`.
4. Backstop distance bound (§7.3) from `distance_est`; standstill predicate in V3 mode uses `speed_est` (quality
   gated) instead of `speed_native`. Prove: V3 stop/reverse timing still ≤ baseline (G1-STOP/G1-STAND) and carry is
   not killed at walking pace.
5. Per-mode defaults (candidates): carry macro ECO 20, TRAIL 70, SPORT 50, SPORT+ 80, BOOST 50, AUTO 60; strength
   0..100 % of the pre-stop assist, time cap 0..1200 ms, distance cap 0..1.5 m scaled by the macro. They must respect
   the MODE_CHARACTER §2 invariants.
6. Telemetry: carry_score, carry_state, carry_remaining_ms, carry_remaining_cm, speed_est, rel_accel, motion quality;
   V3 telemetry schema bump (rate_mode 4 bits); decoder tests updated.
7. Tests: host G1-CARRY (activation on the climb/obstacle stop; no activation on coast, crest, light effort, low
   score; time cap and distance cap — first wins; each cancel; restart handover with no hole; carry false-positive
   rate over the SIL matrix = 0 on coast/crest/steady profiles); motion estimate unit tests (pulse interpolation,
   ratio learning, rel_accel, quality, IMU garbage bit-identical); L4 closed-loop scenarios from `sim/l4`
   (`climb_pedal_stop_obstacle`, `crest_pedal_stop`, `coast_stop`, `reverse_while_motor`): carry activates only in
   the first, ends within its caps, crest cancels on acceleration. Add carry metrics (activation correctness, false
   positives, duration, distance) to `tools/assist_v3_metrics.py` with reject self-tests.
8. Budget: RAM (NORMAL build: 400 B free before this task; must stay ≥ 200 B free), stack gate margin ≥ 1024 B, per-call cycles
   per D-039 (op-count estimate).

## Gate

```
python tools/run_host_tests.py --opt O2 -j 4          # all PASS
python tools/run_regression.py
python tools/run_sil.py --fuzz 300
python tools/run_level4.py                            # full, incl. the four scripts
python tools/run_assist_v3_matrix.py --equivalence    # engine G5300 vs baseline 150/150
python tools/run_assist_v3_matrix.py --engine v3      # report carry metrics + the existing V3 metrics unchanged
python tools/build_firmware.py --variant normal --mode developer --toolchain "<ARM bin>" --output-dir .build/carry
python tools/build_firmware.py --variant diagnostic --mode developer --toolchain "<ARM bin>" --output-dir .build/carry-diag
```

## Output

Print a final report (<= 50 lines): files, design notes per scope item, test results, L4 scenario table (carry active
yes/no, duration, distance, cancel reason) baseline vs V3, matrix deltas, RAM/stack/flash, open issues.
Status READY_FOR_REVIEW. Do not write the report to a file.
