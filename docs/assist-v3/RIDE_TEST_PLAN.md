# Assist Behavior V3 — Controlled Ride Test Plan (DRAFT)

```text
STATUS:    DRAFT — BIN / SHA256 / HEAD filled in at the release candidate (RELEASE_REPORT.md)
BUILD:     <candidate NORMAL BIN> + <candidate DIAG BIN>  (DIAG for the logged rides)
BASELINE:  0.638 (NORMAL) / 0.639 (DIAG) from 25df554 — the fallback image
SCOPE:     Milestone C behaviour (phase-aware intent, TRUE_RELEASE, legacy start/attack/ratio/power/stop)
           Carry (Milestone D) is NOT in this BIN unless RELEASE_REPORT says so.
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

Each manoeuvre: 3 repetitions, note level, approximate speed and cadence, and a one-line feel rating
(better / same / worse than 0.638).

## 3. Signals to log (DIAG)

Existing: final Iq, cadence, speed, load_ctrl, battery V/I, level, brake.
V3 frame group: intent `I`, `env_equiv`, `kappa`, `E_short`, `phase`, `phase_aligned`, `template_mode`,
`confidence`, `release_class`, `v3_demand_iq`, `backstop_iq`, `engine_active`.

## 4. What closes ride-feel acceptance

- R1 level parity and R2/R3 smoothness judged by the owner, supported by the logs.
- R4 release clearly faster than baseline with no pulsing in R1–R3.
- R7–R11 not worse than baseline (safety timing).
- Any "worse" rating is a FAIL for that manoeuvre and goes back to the program as an issue.
