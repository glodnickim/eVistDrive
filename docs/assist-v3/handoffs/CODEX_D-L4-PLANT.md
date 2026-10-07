# CODEX brief — Milestone D prerequisite: Level-4 freewheel drivetrain + wheel sensor model

You are a simulation worker (CODEX). The Lead (Claude) reviews and commits. **Do not commit, stage, stash, checkout or
restore anything.** Edit only files under `sim/l4/` and `tools/run_level4.py` / `tools/analyze_l4_trace.py`, plus
new files you add under `sim/l4/` and `tests/`. Production `src/`/`inc/` are off-limits.

Repository: `C:\Projekty\eVistDrive-assist-v3`. Windows, gcc as `gcc`, Python 3.12. `TEMP`/`TMP` =
`C:\Projekty\eVistDrive-assist-v3\.build\codex-tmp`.

## Why

Obstacle carry (ARCHITECTURE_V3 §7) must be simulated closed loop. Audit C (`docs/assist-v3/audit/C_SIMULATION.md`)
found two plant defects in Level-4: (1) whenever the motor pushes, crank speed is derived from wheel speed
(`sim/l4/bike_rider.c` ~126-145, `sim/l4/virtual_bike_l4.c` ~464), so PAS edges keep coming during motor-only drive and
a pedal stop while the motor carries the bike — or reverse pedalling — cannot be represented; (2) wheel speed reaches
the firmware as exact physics every tick, while production sees one pulse per wheel revolution (2.218 m,
`inc/config.h` ~159-187; `MS.Speedx100` = 0 after 2.65 s of silence; the first pulse after a stop only seeds,
`src/main.c` ~2637-2675).

## Task

1. Drivetrain: model the mid-drive freewheel — the crank (rider) and the chainring/motor are coupled only when the
   crank drives forward at least as fast as the chainring; otherwise the crank can stop or turn backwards while the
   motor drives the chain. The crank angle (and therefore PAS edges into the production sampler) follows the rider's
   own crank dynamics. Keep the existing scenarios' results unchanged where the rider pedals normally (prove: the L4
   G-EQ / existing CSVs either byte-identical, or list every changed scenario with the reason).
2. Wheel sensor model: generate wheel pulses from wheel angle (circumference from config), feed the firmware's speed
   path exactly as production does (pulse timing → `MS.Speedx100` logic replicated in a harness helper if
   `Speed_processing()` cannot be linked; document), including the 2.65 s timeout and the first-pulse seeding.
3. Terrain events: grade over time, an impulse obstacle (step / root: short high resistance torque at the wheel),
   a crest (grade falling to downhill), so carry/crest/climb scenarios can be scripted. Reuse the existing scenario
   mechanism of `tools/run_level4.py`.
4. New scenarios: `climb_pedal_stop_obstacle` (low speed, high effort, pedal stop at an obstacle), `crest_pedal_stop`
   (pedal stop while the bike accelerates over a crest), `coast_stop` (flat, light effort, stop pedalling),
   `reverse_while_motor` (back-pedal one quarter turn while moving). Output CSV columns: time, crank angle, wheel speed
   (physics and sensor), distance, PAS state, published Iq, battery current/energy.
5. Self-tests: a test that fails if PAS edges are generated while the scripted rider crank is stopped and the motor
   drives; a test that the sensor speed reads 0 below ~3 km/h and lags as specified.

## Gate

`python tools/run_level4.py --quick` and the full L4 run PASS; L4 G-EQ (ASSIST_V3 on/off) still PASS; new
scenarios run deterministically (two runs byte-identical).

## Output

Print a final report (<= 40 lines): files changed, model description, changed existing results (if any) with reasons,
new scenarios and what they show for baseline 25df554 behaviour (stop timing, distance travelled after pedal stop),
how to run, open issues. Status READY_FOR_REVIEW.
