# Assist Behavior V3 — Torque/Power Envelope and Mode Character Study

```text
STATUS:   rev 2 after REVIEW-T — design-time static model; checks relabelled per D-038
TOOL:     tools/assist_v3_envelope_study.py (VERDICT PASS), self-tests tests/test_assist_v3_envelope_study.py (6/6)
ANCHOR:   .build/envelope_study/legacy_surface.csv, dumped by tests/host/tools/assist_v3_legacy_surface.c from the
          PRODUCTION g53_static_target() + intent kL/env_equiv (910 points: 5 levels x 13 cadences x 14 efforts)
LABEL:    [SIM-STATIC] steady-state model; transient parameters (Response, Start, Carry) are not evaluated here
AUTHOR:   Lead (Claude). A first CODEX attempt stopped at a stub when its workspace ran out of credits; its files
          were set aside (.build/envelope_study/codex_stub_*) and this study replaces them.
```

## 1. What the legacy characteristic is (from production code)

`Iq ≈ 0.455 · conv`, `conv = c4 · ratio/100 · 1000 / LUT(cad)`, `c4 ∝ env · max(cad,20)`. LUT rises to 740 at
70 rpm and stays flat, so:

- above ~70 rpm support grows with cadence (rider-power proportional);
- below, it shrinks: at 20 rpm a fixed level gives 0.75× of its 60 rpm support for the same pedal effort;
- S+ AUTO (L4) is progressive: ratio `1 + 524·c4/200` up to 525, so light effort gets little support and low
  cadence very little (0.14–0.18× of 60 rpm).

The Python replica of the map matches the production surface exactly (910/910 points, max error 0.000 Iq).

## 2. V3 support model = the G5300 map with a generalised ratio law and a torque/power crossover

```text
cad_eff = max(cad, c_floor)              torque/power crossover: flat, torque-proportional support below c_floor
          above 100 rpm: 100 + (cad-100)·(1 + high_cadence_bias)
c4      = env · cad_eff · 35/10000 · 637/1000        (same integer order as G5300)
ratio   = clamp(base + slope·c4, range_min, range_max)  fixed level: slope 0; S+ AUTO: base 1, slope 524/200, max 525
conv    = c4 · ratio/100 · 1000 / LUT(cad_eff);  Iq = min(conv,1000)·0.455
then    Max Torque: Iq ≤ max_torque% · 455        (existing level_iq_limit path in ap2_limits)
        Max Power : battery power ≤ W              (existing g1 battery-current owner: limit = W / V_batt, ≤ 15 A)
```

Why this shape: it keeps the tuned G5300 cadence normalisation, reproduces every legacy level exactly at the
default Assist, and can be implemented by passing a ratio law and `cad_eff` into the existing pure
`g53_static_target()` — no second characteristic in the firmware.

**Assist macro** (per mode, geometric so equal steps feel equal): fixed-ratio modes scale `base` from `lo` to `hi`
around the legacy ratio at Assist 50; SPORT+ scales the progression `slope` (×0.5..×2) and the ceiling `range_max`
(×0.75..×1.33). All curves are non-decreasing in Assist.

## 3. Candidate profiles (defaults at Assist 50)

| Mode | Reference | base (A0 → A100) | slope | range_max | c_floor | Max Torque | Max Power |
|---|---|---|---|---|---|---|---|
| ECO | L1 | 47 → 190 | 0 | 1000 | 20 rpm (legacy) | 80 % | 350 W |
| TRAIL | L2 | 119 → 387 | 0 | 1000 | 40 rpm | 100 % | HW max (720 W at 48 V) |
| SPORT | L3 | 188 → 512 | 0 | 1000 | 30 rpm | 100 % | HW max |
| SPORT+ | L4 (SPORT+ ratio law) | 1 | 1.31 → 5.24 | 394 → 700 | 30 rpm | 100 % | HW max |
| BOOST | L5 | 292 → 945 | 0 | 1000 | 30 rpm | 100 % | HW max |
| AUTO | L2 point, TRAIL..SPORT+ range in F | 119 → 387 | 0 | 1000 | 35 rpm | 100 % | HW max |

Max Power defaults follow D-034 (hardware maximum except ECO = legacy parity). L5 = BOOST per D-032. The
assist_progression wire value maps to `slope = p/100 · slope_max(mode)`; slope_max(SPORT+) = 5.24.

Steady-state Iq (P = 700) by Assist, from `.build/envelope_study/mode_character.csv` (env = env_equiv units):

| Mode | Assist | env 150@25 | env 300@25 | env 300@60 | env 600@60 | env 600@90 | env 1000@120 |
|---|---|---|---|---|---|---|---|
| ECO | 0 / 50 / 100 | 4 / 9 / 19 | 9 / 19 / 39 | 11 / 24 / 49 | 24 / 49 / 97 | 34 / 70 / 140 | 76 / 135 / 135 |
| TRAIL | 0 / 50 / 100 | 13 / 23 / 44 | 26 / 48 / 87 | 30 / 55 / 98 | 60 / 110 / 198 | 87 / 158 / 285 | 195 / 278 / 278 |
| SPORT | 0 / 50 / 100 | 18 / 30 / 51 | 40 / 66 / 110 | 47 / 79 / 131 | 95 / 159 / 261 | 137 / 228 / 371 | 278 / 278 / 278 |
| SPORT+ | 0 / 50 / 100 | 1 / 2 / 4 | 4 / 10 / 21 | 13 / 27 / 54 | 54 / 107 / 215 | 116 / 232 / 371 | 278 / 278 / 278 |
| BOOST | 0 / 50 / 100 | 30 / 53 / 96 | 62 / 112 / 203 | 74 / 134 / 242 | 148 / 269 / 455 | 214 / 371 / 371 | 278 / 278 / 278 |
| AUTO | 0 / 50 / 100 | 13 / 23 / 41 | 25 / 46 / 84 | 30 / 55 / 98 | 60 / 110 / 198 | 87 / 158 / 285 | 195 / 278 / 278 |

Reading: Assist changes the feel inside the envelope; at high effort and cadence the Max Power ceiling binds and
Assist no longer adds (intended: Assist never moves the physical maximum).

## 4. Check results

| Check | Result |
|---|---|
| Replica vs production surface | PASS — 910/910, max error 0.000 Iq |
| Default parity (Assist 50, cad ≥ c_floor, before envelopes) | PASS — ECO 176, TRAIL 118, SPORT 111, SPORT+ 80 points, error 0.000 Iq |
| Monotonicity in Assist [SIM-STATIC, BY CONSTRUCTION] (Assist 0..100 step 5 × 25/60/90/120 rpm × all efforts × 6 modes) | consistent — 0 violations (follows from monotone curves and a monotone integer map; not G2-MACRO evidence) |
| Envelope regions [SIM-STATIC, BY CONSTRUCTION] | consistent — 0 leaks (a `min()` of caps that ignore Assist; not G2-ORTHO evidence; g1 PI dynamics untested) |
| BASIC independence (overlap of affected regions, Jaccard) | PASS — Max Power vs Max Torque 0.00 / 0.27 / 0.50 / 0.64 / 0.42 (ECO/TRAIL/SPORT/S+/AUTO); Assist vs either ≤ 0.15 |
| Self-tests (each check fails on an injected defect) | 6/6 (they prove the checker, not the design) |

Evidence status (D-038): replica-vs-production and default parity are evidence about the model. G2-MACRO and
G2-ORTHO remain OPEN until run in SIL/L4 on firmware code with the resolver and g1.

Where the envelopes bind at default settings (grid 20..130 rpm, all efforts): Max Torque at 20..90–120 rpm (high
effort), Max Power at 40..60–130 rpm. Torque/power crossover (default caps equal): ECO 45, TRAIL 56, SPORT 66,
SPORT+ 73, AUTO 66 rpm.

Low cadence, support at 20 / 25 / 30 rpm relative to 60 rpm for the same effort:

| | ECO | TRAIL | SPORT | SPORT+ | AUTO | legacy L1–L3 | legacy L4 |
|---|---|---|---|---|---|---|---|
| 20 rpm | 0.75 | 0.90 | 0.89 | 0.45 | 0.88 | 0.75 | ≈ 0.14–0.18 |
| 25 rpm | 0.82 | 0.90 | 0.89 | 0.45 | 0.88 | 0.82 | |
| 30 rpm | 0.89 | 0.90 | 0.89 | 0.45 | 0.88 | 0.89 | |

## 5. Deliberate deviations from legacy at default (to confirm in the SIL matrix and on the bike)

1. TRAIL, SPORT, SPORT+, BOOST, AUTO below their crossover: more support at low cadence (TRAIL +20 % at 20 rpm,
   SPORT+ about 3×). This is the < 30 rpm requirement. It also applies during a start from rest (measured cadence
   0..20 rpm), so Start vs c_floor is part of G2-ORTHO.
2. ECO Max Power 350 W: at high effort above ~45 rpm ECO is capped where legacy L1 was not (e.g. 135 vs 274 Iq at
   120 rpm, full effort). This is the ECO character requested by the owner; the value is a candidate.
3. SPORT+ gives less support than SPORT at light effort (progressive law, kept from legacy S+ on purpose: strong
   attack is rewarded).

## 6. ADVANCED candidates

| Candidate | Static evidence | Recommendation |
|---|---|---|
| high_cadence_bias | acts only above 100 rpm (8–12 % of the grid), overlap with Max Power ≤ 0.11 | keep (E), verify in SIL at 110–130 rpm |
| max_acceleration | no static effect; needs closed-loop bike dynamics | needs SIL/L4 before exposure |
| phase_compensation | no static effect; interacts with the intent layer | needs SIL; default hidden |
| range_min | no effect at defaults (0) | keep for AUTO (F) |

BASIC set: **Assist, Max Torque, Max Power** pass the static independence test. **Response, Start, Carry** are
transient-only and go to the SIL override/orthogonality matrix (G2-OVERRIDE, G2-ORTHO) before the UI shows them.

## 7. Resources (implementation sketch)

Per mode: about 8 curve endpoints + 4 scalars as u16 ≈ 40 B const flash; × 5 modes ≈ 200 B. Per call: the existing
static map plus one multiply-add and a clamp for the ratio law and one `max()` for `cad_eff` — negligible.

## 8. Assumptions and open questions

- [ASSUMPTION] 75 Nm crank torque at Iq 455 (M820 datasheet value; mapping to Iq unverified), drive efficiency 0.80,
  48 V nominal battery. They only place the power-cap curve; the firmware will enforce Max Power on battery current
  through g1, which is exact.
- [UNKNOWN] Physical CLU → kg scale of the torque sensor (CLAIM-004 / TQ-02C), so "rider W" is a proxy.
- Profile numbers are candidates. Final values come from G2-AUTONOMOUS (no overrides, full SIL matrix) and the ride.
- AUTO dynamic range (terrain-driven position inside TRAIL..SPORT+) is Milestone F; here only its static default point.
