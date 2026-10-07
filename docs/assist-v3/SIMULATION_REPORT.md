# Assist Behavior V3 — Simulation Report

Harness: `tools/run_assist_v3_matrix.py` (SIL `--script` mode, production PAS front end and pipeline, open loop:
wheel speed scripted). Metrics: `tools/assist_v3_metrics.py`, each with a reject self-test
(`tests/test_assist_v3_metrics.py`, 32 tests). Labels: [SIM] = host simulation observation, not bike evidence.

## 1. Baseline 25df554 (2026-10-07, quick set, level 3, base effort 1000 ckg, demand = iq_ref) [SIM]

| Metric | 25 rpm | 60 rpm | 120 rpm |
|---|---|---|---|
| TRUE_RELEASE latency to 50 % (ms) | 860 | 1950 | 2500 |
| same, crank degrees | 130 | 700 | 1794 |
| TRUE_RELEASE latency to 10 % (ms) | 2451 | 4720 | 4670 |
| Phase-dip false-positive rate (steady / dead spot / asymmetry) | 1.0 / 1.0 / 1.0 | 0 / 0 / 0 | 0 / 0 / 0 |
| Per-revolution ripple, median (steady) | 0.55 | 0.15 | 0.12 |
| Per-revolution ripple, median (dead spot) | 0.81 | 0.21 | 0.22 |
| Gradual-release lag (ms) | 972 | 2811 | 3931 |
| Attack t63 (ms) | 350 | 490 | 571 |
| Restart dip | 0.00 | 0.00 | 0.00 |
| Safety ref -> 0 (ms): stop (load removed) / reverse / brake | 116 / 36 / 194 | 93 / 35 / 196 | 97 / 60 / 198 |

Across all ten cadences the phase-dip false-positive rate is 1.0 at 20–30 rpm and 0 from 60 rpm up: at low cadence
the G53 envelope decays between strokes and the assist pulses with the pedal phase. Release latency grows with
cadence (the envelope decay constant is proportional to cadence). Both are the defects V3 targets.

G-EQ: candidate tree with no V3 code vs `git archive 25df554` — 150/150 scenario traces byte-identical; fixed SIL
modes, fuzz and electrical SIL unchanged.

Limitations: open loop (climb/crest/carry approximate until the Level-4 freewheel drivetrain); stop segments here
unload the pedal (the existing SIL stop axis keeps 60 kg, 1033 ms); energy is a ∫Iq·dt proxy; metric pass thresholds
in `CHECKS` are placeholders until set below.

## 2. Acceptance thresholds for V3 (set by the Lead before candidate runs)

| Metric | V3 acceptance |
|---|---|
| TRUE_RELEASE latency to 50 % | <= 45° of crank + R(Response) fall time at every cadence 20..130 rpm |
| Phase-dip false-positive rate | 0 at every cadence 20..130 rpm (converged template); 0 in fallback mode for the first 3 revolutions |
| Steady-state level vs baseline | G1-LEVEL (±5 % converged, ±15 % prior) |
| Per-revolution ripple | <= baseline at 60–130 rpm; < 0.25 at 20–30 rpm |
| Attack t63 | within ±20 % of baseline (legacy attack kept) |
| Stop / reverse / brake safety timing | <= baseline (brake path unchanged) |
| Restart dip | <= baseline + 0.05 |
| Class transitions in steady riding | 0 |

## 3. Milestone C candidate (15df8e9) vs baseline — full matrix, level 3 [SIM]

Cadence columns: 20 | 25 | 30 | 40 | 60 | 80 | 100 | 110 | 120 | 130 rpm. Each cell baseline / V3.

| Metric | Values |
|---|---|
| TRUE_RELEASE to 50 % (ms) | 820/156 · 860/132 · 900/117 · 1190/97 · 1950/78 · 3040/149 · 3150/92 · 2780/137 · 2500/88 · 2431/105 |
| same, crank degrees | 102/20 · 130/21 · 161/22 · 288/25 · 700/30 · 1453/69 · 1881/54 · 1827/89 · 1794/67 · 1887/79 |
| Phase-dip FP (steady/dead/asym) | baseline 1.0 at 20–30 rpm (asym 0.5 at 30–40); V3 0 everywhere |
| Ripple, steady | .77/.04 · .55/.00 · .43/.03 · .27/.01 · .15/.04 · .17/.11 · .12/.10 · .12/.10 · .12/.10 · .12/.11 |
| Ripple, dead spot | .97/.20 · .81/.18 · .67/.14 · .32/.13 · .21/.11 · .28/.21 · .23/.20 · .24/.20 · .22/.19 · .23/.21 |
| Attack t63 (ms) | 321/360 · 350/350 · 381/381 · 421/418 · 490/480 · 591/601 · 571/515 · 591/517 · 571/511 · 571/499 |
| Stop, crank unloaded, ref -> 0 (ms) | 122/108 · 116/114 · 100/97 · 88/81 · 93/84 · 87/85 · 97/91 · 97/91 · 97/87 · 97/89 |
| Reverse ref -> 0 (ms) | 37/32 · 36/33 · 37/35 · 40/33 · 35/32 · 54/46 · 66/56 · 59/52 · 60/54 · 68/59 |
| Brake ref -> 0 (ms) | 194–199 in both (native SAFETY 200 ms unchanged) |
| Coast | V3 <= baseline at every cadence |
| Restart dip | .01/.01 · .00/.04 · .01/.10 · .00/.02 · .00/.01 · 0/0 from 80 rpm |
| Level parity V3/baseline | steady 0.944–0.964, climb 0.956–1.0, asym 0.888–0.980, dead spot 0.848–0.919 |

Verdict against §2: release, phase dip, ripple, attack, stop/reverse/brake PASS. **G1-LEVEL FAIL** (−4…−15 %),
**restart FAIL at 30 rpm** (dip 0.10) and first-Iq delay at 25/40 rpm — rework in progress. pas_glitch scenario:
phase-dip FP 0.05–0.30 at 25–100 rpm — rework in progress.

## 4. Candidates

(filled at Milestone B/C: template NB 12 vs 24, α, windows, R_rel/R_att, Response mapping)
