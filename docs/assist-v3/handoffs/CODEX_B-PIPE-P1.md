# CODEX brief — B-PIPE phase 1 completion (Assist Behavior V3 shadow integration)

You are a firmware worker (CODEX). The Lead (Claude) reviews and commits. **Do not commit, stage, stash, checkout or
restore anything.** Leave all changes in the working tree.

Repository: `C:\Projekty\eVistDrive-assist-v3` (branch `feature/assist-behavior-v3`, HEAD `a00b4a7`). Windows,
Git Bash/PowerShell, gcc (w64devkit) as `gcc`, Python 3.12. ARM toolchain:
`C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\13.2 Rel1\bin`.
Use `TEMP`/`TMP` = `C:\Projekty\eVistDrive-assist-v3\.build\codex-tmp` (create it) — the user temp dir is blocked in
your sandbox.

## Situation

A previous worker (Claude) implemented most of phase 1 and was cut off by a rate limit while writing the
crank_phase host suite. Its work is **uncommitted in the working tree**: `git status` / `git diff` show it
(~24 modified files + new `inc/assist_v3.h`, `inc/crank_phase.h`, `src/assist_v3.c`, `src/crank_phase.c`,
`tests/host/assist_v3_crank_phase_host.c`, `tests/host/assist_v3_telemetry_frames_host.c`,
`tests/host/assist_v3_trajectory_host.c`, `tests/host/common/assist_v3_harness.h`). Start by reading that diff and
those files completely; then finish, fix and verify phase 1. Do not throw the work away; correct it where it is wrong.
`docs/assist-v3/RIDE_TEST_PLAN.md` is the Lead's draft — do not edit it.

## Normative documents (read)

- `docs/assist-v3/ARCHITECTURE_V3.md` (rev 3): §2.1 ownership (R1 kept, standstill predicate), §2.2 engine latch
  and EB74 armed gate, §3.1/3.2 inputs and `crank_phase.c`, §3.3 motion seam, §4.3 env_equiv, §5 static map,
  §6 trajectory (Milestone C rates incl. legacy stop rates), §6.2 start, §7.3 backstop, §9 safety, §11 telemetry,
  §12 resources.
- `docs/assist-v3/DECISIONS.md` (D-002, D-008, D-018..D-021, D-024 RAM).
- `docs/assist-v3/TEST_MATRIX.md` (G-EQ rules 1-3).
- APIs: `inc/assist_v3_intent.h`, `inc/assist_motion.h`, `inc/assist_v3_config.h`, `inc/g53_port.h`,
  `inc/g53_port_chain.h`.

## Phase 1 scope (shadow only — the motor is still driven by G53)

1. `crank_phase.c/.h`: production owner of signed crank step count, last-step tick, glitch flag (INVALID jumps and
   sampler overflow). Fed by the main.c PAS event drain; SIL and L4 feed the same module (no harness copies of the
   new logic).
2. New fields carried rider_input_t -> ride_control_input_t -> assist_pipeline_input_t: crank_steps,
   crank_step_tick, pas_glitch, wheel_pulse_tick, iq_measured, brake (observation).
3. `assist_v3.c/.h`: trajectory owner + Milestone C transient rules (no carry), intent call, env_equiv -> 
   `g53_static_target()` -> Iq (`target_E2*0.65*P/40960`), rise min(D7EC level rise, BDE8 50 m2aa/ms), fall
   R(Response) (0 % -> 600 ms full scale, 100 % -> 150 ms, linear), legacy stop rates (released 3.5 Iq/ms, held
   0.455 Iq/ms after stop confirmed), engage per §6.2. Single output iq_demand >= 0 + `v3_stop_target_zero`
   observation + telemetry. Integer only.
4. Pipeline: G53 always runs; V3 computed every tick in SHADOW; published request = G53 in every engine in phase 1.
5. DIAG telemetry: one V3 frame group, DIAG builds only, never in NORMAL builds; decoders must at least ignore it.
6. SIL/L4/matrix hooks: V3 columns in a separate CSV (G-EQ rule 2); matrix runner supports `--engine v3` later.
7. Host suites registered in `tests/host/run-host-tests.ps1`, names matching regex `assist_v3|assist_motion`
   (case-insensitive) so `--group v3` selects them. Check that the rider-intent and motion suites
   (`assist_v3_intent_host.c`, `assist_motion_host.c`) are registered with such names (modules:
   `src/assist_v3_intent.c` + `tests/host/common/rider_script.c`; `src/assist_motion.c`).
8. New sources in `scripts/sources-m820.txt`; `assist_v3.c` and `assist_v3_intent.c` on the -O2 list in
   `tools/build_firmware.py` (and `scripts/build-firmware.ps1` if it mirrors it).

Never let V3 write slew mode, ceiling or zero policy. Safety paths untouched. No float in production code.

## Phase 1 gate (run all, report results)

```
python tools/run_host_tests.py --opt O2 -j 4                 # all PASS
python tools/run_regression.py                               # PASS
python tools/run_sil.py --fuzz 300                           # PASS
python tools/run_level4.py --quick                           # PASS
python tools/run_assist_v3_matrix.py --equivalence           # 150/150 byte-identical vs 25df554
python tools/build_firmware.py --variant normal --mode developer --toolchain "<ARM bin>" --output-dir .build/pipe
python tools/build_firmware.py --variant diagnostic --mode developer --toolchain "<ARM bin>" --output-dir .build/pipe-diag
```

Target builds must pass including the stack gate. Report static RAM end (ebss), stack gate worst case and margin,
flash use. If static RAM does not fit: apply D-024 (`__stack_size` 6K -> 5K in `ldscripts/gd32f30x_flash.ld`) and
show the stack gate passes with margin >= 1024 B; if it still does not fit, STOP and report.

## Output

Print a final report (<= 50 lines) to stdout: files changed/added with one line each, what you fixed in the previous
worker's code, gate results with numbers, RAM/stack/flash, open issues. Status READY_FOR_REVIEW. Do not write the
report to a file.
