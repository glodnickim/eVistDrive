# Assist Behavior V3 — Controlled Ride Test Plan (DRAFT)

```text
STATUS:    RC version — BIN names, SHA256 and HEAD are in RELEASE_REPORT.md
BUILD:     <candidate NORMAL BIN> + <candidate DIAG BIN>  (DIAG for the logged rides)
BASELINE:  0.638 (NORMAL) / 0.639 (DIAG) from 25df554 — the fallback image
SCOPE:     Milestone C (phase-aware intent, TRUE_RELEASE, legacy start/attack/ratio/power/stop) and
           Milestone D (obstacle carry, OBSERVATIONAL in this ride: thresholds are candidates, D-043)
```

## 0. Before the first ride (bench, bike on a stand)

1. Flash the DIAG candidate. Confirm version on the HMI/CANable.
2. CONFIG_A survival check (REVIEW 1 R-d): with CANable raw frames, persist a V3 block (0x6036 write, then 0x6037 op 1
   at standstill), power-cycle, read 0x6035/0x6036 view 0. Then perform a BL820 update with the same BIN and read
   back again. Record whether the record survived. If not, persist is "defaults after update" (owner decision).
3. Engine check: 0x6035 reports `engine_active = 1` (V3). Write `engine = 0`, release the pedals, confirm it switches
   only at zero demand; write `engine = 1` back.
4. Stand test, rear wheel free: light pedalling, strong pedalling, brake lever while pedalling (assist drops in
   ~200 ms), back-pedal (assist ramps out ≤ ~130 ms), stop pedalling with and without foot pressure.

5. CPU budget (REVIEW 2 #1, D-039) — **no riding if over budget**: on the DIAG build, ride the stand test of step 4
   for 2 minutes and read the V3 frame CPU fields (max cycles of the V3 stage per call) and the dropped G53 logical
   tick counter. Budget: worst case <= 60 µs per call at 120 MHz (7200 cycles) and no more dropped ticks than the
   same stand test on 0.639. Over budget -> stop, report to the program.
6. Readback after every flash and every power cycle: read 0x6035. Expected `engine_requested = 1`; `engine_active`
   reads 0 (G5300) right after boot and becomes 1 at the first moment the latch allows it (all demands 0, no veto —
   normally before the first pedal stroke at standstill). If it never becomes 1, V3 is not riding: stop.
7. Standstill, loaded crank: with the bike held, put weight on a pedal without turning it, then nudge the crank a few
   degrees forward: assist must stay low and controlled and end when the pedal is unloaded or the crank stops.

## 1. On-trail fallback (read before riding)

- Anything unexpected: brake (always authoritative), or assist level 0, or power off.
- Return to known behaviour: reflash 0.638, or send a raw `engine = 0` write (0x6036) from CANable.
- The absent-record default of the candidate is V3; once a record is persisted, the stored engine wins.

## 2. Manoeuvres (DIAG BIN, CANable logging on)

| # | Manoeuvre | What to do | Expected V3 behaviour | Compare with baseline |
|---|---|---|---|---|
| R1 | Flat, 60–80 rpm, steady | 1 min steady pressure, mid level | same assist level as 0.638 (±5 %), smoother (no pulsing) | same loop on 0.639 |
| R2 | Low cadence climb | 20–30 rpm, heavy gear, steady climb | no pulsing at dead spots; assist steady through the stroke | 0.639 pulses (phase-dip FP 1.0 in SIM) |
| R3 | High cadence | 110–130 rpm, light gear | steady assist, no hunting | |
| R4 | Deliberate release while pedalling | keep spinning, take pressure off quickly | assist drops within ~1/8 revolution + Response fall (≈150–600 ms) — baseline takes 2–5 s | 0.639: assist lingers |
| R5 | Gradual release | slowly reduce pressure over 2–3 s | assist follows smoothly, no steps | |
| R6 | Strong attack | from easy pedalling, stand up and push | assist rises as fast as on 0.638 (legacy acceleration) | |
| R7 | Stop pedalling, foot off | coast | assist ends as fast as on 0.638 (~0.1 s) | |
| R8 | Stop pedalling, foot loaded | trackstand / pause on a climb | assist ramps out as on 0.638 (~0.7–1.0 s), no cut | |
| R9 | Restart after pause | resume pedalling after R8 | no jerk, no hole | |
| R10 | Brake while pedalling | squeeze brake | cut in ~200 ms as on 0.638 | |
| R11 | Back-pedal while moving | one quarter turn back | assist ramps out ≤ ~130 ms | |
| R12 | Gear shift under load | ease off for the shift, resume | short dip only, no surge on resume | |
| R13 | Level changes while riding | step through levels 1–5 | smooth, S+ AUTO behaves as on 0.638 | |

| R14 | Obstacle carry (observational) | on a steep technical climb at walking pace, push hard, then stop pedalling for a root/step | at most a short push (<= 1.2 s, <= 1.5 m) or none; never after braking, back-pedalling or when the bike speeds up | 0.639 has no carry |
| R15 | Pedal pause on flat / crest | stop pedalling on flat at speed, and over a crest | no carry, assist ends as on 0.638 | |

Start at level 3 (SPORT, the level the simulation matrix covers); then levels 1, 2, 4, 5.

Each manoeuvre: 3 repetitions, note level, approximate speed and cadence, and a one-line feel rating
(better / same / worse than 0.638).

## 3. Signals to log (DIAG)

Existing: final Iq, cadence, speed, load_ctrl, battery V/I, level, brake.
V3 frame group: intent `I`, `env_equiv`, `kappa`, `E_short`, `phase`, `phase_aligned`, `template_mode`,
`confidence`, `release_class`, `v3_demand_iq`, `backstop_iq`, `final_iq`, `engine_active`, `carry_score`,
`carry_state`, `carry_remaining_ms/cm`, `speed_est`, `rel_accel`, motion quality, V3 CPU max/last, dropped G53 ticks.

## 4. What closes ride-feel acceptance

- R1 level parity and R2/R3 smoothness judged by the owner, supported by the logs.
- R4 release clearly faster than baseline with no pulsing in R1–R3.
- R7–R11 not worse than baseline (safety timing).
- Any "worse" rating is a FAIL for that manoeuvre and goes back to the program as an issue.
