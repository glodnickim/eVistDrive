# CODEX brief — B-CFG rework to config protocol v2 rev 2

You are a firmware worker (CODEX). The Lead (Claude) reviews and commits. **Do not commit, stage, stash, checkout or
restore anything.** Leave all changes in the working tree.

Repository: `C:\Projekty\eVistDrive-assist-v3` (branch `feature/assist-behavior-v3`). Windows, gcc (w64devkit) as
`gcc`, Python 3.12. ARM toolchain: `C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\13.2 Rel1\bin`.
Use `TEMP`/`TMP` = `C:\Projekty\eVistDrive-assist-v3\.build\codex-tmp`.

## Read first (normative)

- `docs/assist-v3/CONFIG_PROTOCOL_V3.md` (v2 rev 2 — the wire contract, CAPS format 2 = 30 B table, objects, sentinels,
  semantics, verification list §6)
- `docs/assist-v3/MODE_CHARACTER.md` (§5 applicability and units, §6 the one precedence rule incl. legacy inputs per
  level and the attack exception, §7 source codes, §9 RAM budget)
- `docs/assist-v3/DECISIONS.md` D-032..D-040
- `docs/assist-v3/reviews/REVIEW_T_MODE_CHARACTER.md` (why each rule exists, incl. the re-check section)
- Current v1 implementation: `src/assist_v3_config.c`, `inc/assist_v3_config.h`, `tests/host/assist_v3_config_host.c`,
  CAN wiring in `src/CAN_Display.c` / `inc/CAN_Display.h`, persist service in `src/main.c`.

## Task

Rework the v1 owner module to v2 rev 2, keeping what is good (transfer handling, staging, append-only CONFIG_A log,
flash HAL injection, fault-injection tests):

1. Objects: MODE PROFILE (schema 2, index = mode 1..6: ECO, TRAIL, SPORT, SPORT+, BOOST, AUTO; 24 × u16 configured
   values) and GLOBAL (schema 3: engine, level -> mode map default ECO/TRAIL/SPORT/SPORT+/BOOST). Views 0 saved (read
   from the CONFIG_A log, no RAM copy), 1 effective + source for one level, 2 profile defaults, 3 configured.
2. Resolver: `const assist_v3_effective_t *assist_v3_effective(uint8_t hmi_level)` implementing MODE_CHARACTER §6
   exactly (ADVANCED override > macro derivation; macro DEFAULT adjusted by that level's legacy P0/P1 input; attack
   exception; Max Power percentage semantics; per-mode applicability with source 5; shadowed source 6; limited source 4
   only where a firmware limit changes the resolved configuration). Cached for the active level, recomputed on
   generation or level change. Profile tables: use the candidate values of ENVELOPE_STUDY §3 (put them in one const
   table with a comment that they are candidates). The legacy P0/P1 per-level values are read through the existing
   accessors (find how `apply_assist_levels` / `g53_port_set_levels` get them; do not change their behaviour).
   Keep `assist_v3_effective_release_pct(level)` (added in Milestone C) working on top of the resolver (release = ID 13
   effective value).
3. Wire: CAPS format 2 (30 B, exact table), 0x6036 READ [object, index, view, level], 0x6036 WRITE multiframe per
   object, 0x6037 [op, arg, gen_lo, gen_hi] with ops 1..4; NORMAL_ACK carries the new generation; KEEP 0xFFFE and short
   param_count = keep; schema_version within [min, max]; post-write applicability validation; reserved/not-applicable
   rejects (reason 4); generation skips 0xFFFE/0xFFFF; persist with optional expected generation and outcome in
   persist_state (4 = stale); v1 records ignored (flash_record_state 4); truthful engine_active (set by the pipeline
   setter); firmware without ASSIST_V3 rejects engine = V3 and clears caps bit 0.
4. Storage: the whole configuration in one log slot (6 mode objects + global), newest valid CRC wins, erase only when
   full, persist at standstill only.
5. RAM: one configured image (~304 B) + 72 B effective cache + staging; report .bss before/after. NORMAL build must
   still pass the stack gate (margin >= 1024 B); report the free RAM.
6. Tests: rewrite `tests/host/assist_v3_config_host.c` to cover CONFIG_PROTOCOL_V3 §6 completely (G1-CFG2), keeping the
   fault-injection and no-mutation patterns. Keep the CAN_Display text guard (legacy P0/P1/P2, bank, tuning untouched).

## Gate (run all, report numbers)

```
python tools/run_host_tests.py --opt O2 -j 4                      # all PASS
python tools/run_regression.py
python tools/run_sil.py --fuzz 300
python tools/run_assist_v3_matrix.py --equivalence                # engine G5300 vs baseline 150/150
python tools/build_firmware.py --variant normal --mode developer --toolchain "<ARM bin>" --output-dir .build/cfg2
python tools/build_firmware.py --variant diagnostic --mode developer --toolchain "<ARM bin>" --output-dir .build/cfg2-diag
```

## Output

Print a final report (<= 45 lines) to stdout: files changed, design notes (slot size, resolver caching, RAM), test
list and results, gate numbers, open issues. Status READY_FOR_REVIEW. Do not write the report to a file.
