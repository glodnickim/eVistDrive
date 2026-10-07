# Audit A: active PEDAL assist control path and V3 trajectory owner

```text
DOCUMENT:   docs/assist-v3/audit/A_CONTROL_PATH.md
PROGRAM:    Assist Behavior V3, Milestone A (read-only audits A-D)
BASELINE:   25df554 (feature/assist-behavior-v3 == tag baseline/rideable-25df554)
TIMESTAMP:  2026-10-07T12:25:51+02:00
AUTHOR:     read-only audit subagent (Claude Code, claude-opus-5-5)
STATUS:     READY_FOR_REVIEW (no code changed, nothing committed)
METHOD:     source reading of the worktree + one host probe built from the production modules
            (g53_port*, g53_g1_limiter, ap2_limits, assist_pipeline, assist_modes, fast_iq_slew),
            same harness shape as tests/host/reverse_ramp_host.c. verify_all.py was not run.
LABELS:     [CONFIRMED_CODE] read in source; [PROBE] measured on production modules on the host
            (an observation, not HW evidence); [PRIOR] taken from accepted earlier evidence
            (TQ-02B block-table, TQ-06-G2 audit, STOP-RAMP / AUTO-SPLUS reports);
            [INFERRED]; [UNKNOWN]
UNITS:      P = phase_current_max (700 in every number below). m2aa: 10000 = P.
            Iq = m2aa*P/10000. "logical tick" = 1 ms = 4 control ticks of 4 kHz.
```

## 0. Summary

- The PEDAL demand has exactly one producer at baseline: the transcribed G53 chain (EB74 -> D7EC -> BDE8, scaled by
  g1). Everything after it is a limit or a guard: ap2_limits (stateless on PEDAL), a slewed protection ceiling, a
  pipeline zero/slew policy, and fast_iq_slew at 16 kHz. [CONFIRMED_CODE]
- The temporal trajectory (rise, hold, fall) is shaped in **six places**, four of them inside D7EC/BDE8 (§5).
- **TRUE_RELEASE while still pedalling is slow because of the D7EC envelope peak-hold.** Its decay is
  `env *= k/(k+1)` every 10 ms with `k = 8*cadence_rpm`, so τ = 4.8 s at 60 rpm and 7.2 s at 90 rpm. On the probe,
  dropping the load to zero at 60 rpm took **7.6 s to reach zero** (t50 2.5 s). The fixed fall constant
  **D3E = -409 / 10 ms** on the D7EC accel trajectory (1.0 s full scale, 0.455 Iq/ms) is a second, faster floor that
  only binds when the D7EC target collapses without a hard clear (crank stopped with load held: 0.69 s). [PROBE] [CONFIRMED_CODE]
- Pedal phase enters only as PAS **counts** (cadence, evidence, timeouts). No absolute crank angle exists. A
  crank-angle torque window (FW-085, 180°) exists in torque_input but is telemetry only. [CONFIRMED_CODE]
- **Recommended single V3 trajectory owner:** a new `assist_v3` stage inside `assist_pipeline_update()`, placed at the
  Boundary-B seam. It replaces `ctx.g53.iq_request_pre_limits` (assist_pipeline.c:136) as the request entering
  `ap2_limits_apply`. The G53 chain keeps running as a shadow, providing g1, PAS and telemetry, and stays the G5300
  fallback (§8).

## 1. Execution contexts

| Context | Rate | What runs | Ref |
|---|---|---|---|
| TIMER1 ISR | 4 kHz | PAS line sampling `pas_sampler_isr_tick`; battery-current sample and IIR; `battery_trip_sample` (bridge MOE off on trip) | main.c:2362-2420 |
| Foreground `reg_ADC_processing` | about 4 kHz. Flag-driven, so it can coalesce periods; `control_delta` = elapsed periods | torque ADC (main.c:2759), torque_input (offset, fault, load_ctrl), PAS event drain (main.c:2772), pas_liveness/real_stop (3043-3049), rider_input publish (3155), `ride_control_update` (3427) -> `assist_pipeline_update` -> `g53_port_update` -> `ap2_limits` -> mailbox publish | main.c:2689-3430 |
| G53 logical tick inside the foreground | 1 ms (4 control ticks, catch-up ≤64, excess counted as dropped) | per tick: EB74, G53 PAS, g1 PI #1, BDE8. Every 10 ticks, by phase: D7EC (phase 0), E1E8 (phase 3), g1 limit update (phase 5), FSM (phase 7, throttle input held 0) | g53_port.c:113-163, g53_port_chain.c:2295-2335 |
| ADC0_1 ISR (FOC) | 16 kHz | `fast_iq_slew_tick` (sole writer of `MS.i_q_setpoint`), `foc_current_loop_step` (PI, `_U_MAX` vector limit). QZERO compiled out | main.c:3845-3966, config.h:421 |
| Main loop | each pass | brake GPIO PC13 -> `MS.brake_active_flag`, no debounce | main.c:1446 |

[CONFIRMED_CODE] for all rows. When the foreground coalesces, the G53 port replays the same held `pas_ab` for every
missed logical tick, so lost PAS edges are not reconstructed (g53_port.c:128-129). [CONFIRMED_CODE]

## 2. Active path, end to end (G5300 mode = baseline)

```text
torque ADC (adc_value[2], 4 kHz DMA) -> torque_input_correct (offset, coast re-zero)
  -> load_ctrl = frozen CLU characteristic of the raw delta, UNFILTERED            torque_input.c:929
PAS A/B (TIMER1 4 kHz) -> pas_sampler_state() raw 2-bit state                      main.c:3385
wheel -> MS.Speedx100 (0.01 km/h; reads 0 after 2.65 s pulse silence)              main.c:1581
        |
ride_control_update: owner arbitration (comm_inhibit > battery_trip > calibration > walk > walk_release > PEDAL)
        |                                                                          ride_control.c:205-349
assist_pipeline_update                                                             assist_pipeline.c:111
  g53_port_update (1 ms logical ticks)
    EB74  source=750+load*2450/6000; threshold zero+245 (fb=0) / 820 (fb!=0); IIR    g53_port_boundaries.c:268-311
    G53 PAS 96 tr/rev: cadence (adaptive span, event IIR), evidence, true-stop       g53_port_pas.c:417-497
    D7EC (10 ms): envelope/history/readiness -> C4 -> ratio/AUTO -> conv/LUT(cad)
                  -> accel trajectory E2 (rise D28[slot], fall D3E) -> EE (Q12)      g53_port_chain.c:365-757
    g1 PI #1 battery limiter (1 ms), limit update (10 ms, level power %)             g53_port.c:149-162
    BDE8 (1 ms): Q5A = min(6500,6500)*EE>>12; S5 slew +-50/ms -> Q5C;
                 m2aa = Q5C*g1>>12 (g2=1.0)                                          g53_port_chain.c:1977-2010
    E1E8 (10 ms): computed, OUTPUT UNCONSUMED on M820 (taper input fixed 1.0)        g53_port.c:149
  Boundary B: iq_request_pre_limits = m2aa*P/10000                                   g53_port_boundaries.c:313-320
  ap2_limits (PEDAL): power(off) -> battery(skipped, owned by g1) -> phase -> UV -> thermal -> speed taper
  iq_ceiling = ap2_slew(400 ms rise / 120 ms fall full scale)                        assist_pipeline.c:155-161
  zero/slew policy: native_cut->SAFETY 200 ms; R1 bumpless rise; stock_hard_zero->FORCE_ZERO;
                    else RISE/FALL step 0.625*P per 4 kHz tick                       assist_pipeline.c:174-214
ride_publish_final_iq -> seqlock mailbox                                             ride_control.c:356
fast_iq_slew_tick (16 kHz) -> MS.i_q_setpoint -> PI_iq -> foc_current_loop_step     main.c:3845,3960
```

### ap2_* status [CONFIRMED_CODE]

| Module | In firmware manifest | Production caller | Verdict |
|---|---|---|---|
| ap2_limits.c | yes | assist_pipeline.c:154 (PEDAL), ride_control.c:178 (Walk) | ACTIVE. Stateless on PEDAL because `battery_stage_owned_upstream=true`; Walk keeps `battery_iq_cap` |
| ap2_math.h | header | slew/clamp/map helpers | ACTIVE |
| ap2_rider_demand.c, ap2_estimators.c, ap2_pas_state.c, ap2_profiles.c | yes (scripts/sources-m820.txt) | none in src/ (only host tools) | DEAD on the product path. Only the enum types (`ap2_pas_state_t`, `ap2_profile_id_t`) are still used. The linker should discard the code [INFERRED] |
| docs/ASSIST_PIPELINE_V2.md §1-§10 | — | — | STALE: describes the pre-G53 ap2 chain |

## 3. Stage table: every stage with memory, ramp, filter or hold

Rise/fall values are per call unless stated otherwise. "G5300" = faithful transcription (TQ-02B CB / N4 [PRIOR]);
"M820" = addition specific to this firmware.

| # | file:function | Owner | Domain / units | Rate, context | Rise | Fall | Hold | Reset | Stop behavior | Origin | Label |
|---|---|---|---|---|---|---|---|---|---|---|---|
| T1 | torque_input.c:733 `torque_input_correct` + :738 coast re-zero | torque_input | ADC mV, offset | 4 kHz fg; re-zero on coast episodes | — | — | offset is persistent | startup zero, coast evaluation | — | M820 | CONFIRMED_CODE |
| T2 | torque_input.c:785 `torque_input_update_elapsed` -> `load_ctrl` | torque_input | CLU 0..6000+ | 4 kHz fg | none (raw) | none | none | — | — | M820 | CONFIRMED_CODE |
| T3 | torque_input.c:317 fast filter (35 ms) and :494 RUN crank window (180° = 48 PAS steps) | torque_input | native delta | 4 kHz / per forward step | 35 ms / window | same | window holds between steps | rearm automaton | — | M820 | CONFIRMED_CODE: **not on the control path** (only into `rider_input_t` fields nobody reads for control) |
| T4 | main.c:3100-3112 torque fault debounce | main | mV range, stuck CLU | 4 kHz fg | — | — | fault hold about 5 s | cause cleared plus hold | `torque_fault` -> native_cut | M820 | CONFIRMED_CODE |
| P1 | pas_direction / pas_liveness / cadence_filter | native PAS | steps, ticks, rpm | per PAS event / fg | — | — | liveness timeout 200..500 ms | stop | `real_stop`, `direction_inhibit`: **observation only** on PEDAL; `cadence_rpm` is telemetry only | M820 | CONFIRMED_CODE (assist_pipeline.c:166-173, 235-247) |
| G1 | g53_port_boundaries.c:268 `g53_ad7ec_step` (EB74) | G53 port | pre-EB74 counts 750..3200 | 1 ms | IIR α = 6176/16384 ≈ 0.38/ms (τ ≈ 2 ms) | **hard reset to 0** when source ≤ threshold | startup 30 ticks; check window 100 ticks | `g53_port_reset` | — | G5300 (owner CLU adapter) | CONFIRMED_CODE |
| G2 | g53_port_boundaries.c:236 EB74 auto-zero | G53 port | counts 750..995 | 1 ms | averages 128 samples after a 301-tick window | — | zero persists | disarmed unless cadence==0 ∧ speed_native>1000 ∧ m298==0. With the X3 adapter unit of 0.1 km/h that means >100 km/h, so it **effectively never arms on M820** (zero stays 750) | — | G5300 | CONFIRMED_CODE (consequence INFERRED) |
| G3 | g53_port_pas.c:417 `g53_pas_step` | G53 port | 96 tr/rev; rpm (signed) | 1 ms | event IIR α ≈ 0.1/measurement | — | — | true-stop: no transition for 25..311 ticks (`180000/(96*mag+1)`) | cadence, evidence, filter cleared | G5300 logic, M820 geometry 96 | CONFIRMED_CODE |
| G4 | g53_port_chain.c:508-521 D7EC envelope | D7EC | rider units | 10 ms | attack: `env=cur` | `env*k/(k+1)`, k=8·cad (40 at 0 rpm), floor ≥ 1 unit/call; τ=(k+1)·10 ms: **4.8 s @60 rpm, 7.2 s @90 rpm, 0.41 s @0 rpm** | **peak hold over pedal strokes** | reverse; zero-reset | — | G5300 | CONFIRMED_CODE / PROBE |
| G5 | :524-581 D7EC history (cur==0) | D7EC | rider units | 10 ms | — | `candA = AE*(1-B2/1000)`, B2 = PAS evidence since release: linear to 0 over 1000 transitions = **10.4 crank revs** (crank-angle domain); candB disabled (D84=0) | snapshot AE frozen at release | move==0 clears | — | G5300 | CONFIRMED_CODE |
| G6 | :585-600 readiness / C2 | D7EC | power-like | 10 ms | ready = evidence ≥ 4 or env ≥ 2500 | not ready and move==0 -> C2=0 | **C2 retained** when not ready but moving | — | — | G5300 | CONFIRMED_CODE |
| G7 | :607-683 ratio / AUTO + rise limiter D7A | D7EC | ratio pts | 10 ms | +10 / call (all levels; binds on level change, and on AUTO tracking) | unlimited | D0 state | — | — | G5300 (AUTO on S+ = M820 config) | CONFIRMED_CODE |
| G8 | :688-721 conv / LUT(cad) / DE floor | D7EC | permille | 10 ms | — | — | DE floor **inert** (D58 table = 0) | — | — | G5300 | CONFIRMED_CODE |
| G9 | :722-731 **accel trajectory E2 -> EE** | D7EC | E2 0..40960, EE Q12 | 10 ms | `D28[slot]` = 40960/(250-32(n-1)): n=4/5/6/8 -> 265/335/455/1575 per call (full scale 1.55/1.22/0.90/0.26 s) | **D3E = -409 per call** (0xFE67, chain.c:2148) -> **full scale 1.0 s** | — | zero-reset clears; reverse clears EE but **keeps live E2** | — | G5300 (M820 level table) | CONFIRMED_CODE |
| G10 | :320-356 tail G05 hold-off window | D7EC | ticks | 10 ms | EE forced 0 until window ≥ d1c (311): 32 calls; shortened to 2 calls when cadence≠0 | — | — | G04 / D25 event | — | G5300 | CONFIRMED_CODE; G04 semantics UNKNOWN |
| G11 | g53_port.c:149, g53_g1_limiter.c | G53 port | g1 Q12 | 1 ms PI; 10 ms limit | PI ref slews +10/ms; soft start ≈ 190 ms after power-on | PI-limited | survives pipeline resets | power-on only (`g53_port_init` is **never called**; relies on static zero) | — | G5300 bit-exact; M820 SOC knee | CONFIRMED_CODE / PRIOR |
| G12 | g53_port_chain.c:1977-1994 BDE8 S5 | BDE8 | m2aa 0..6500 | 1 ms | +50/ms (state 5: +10) = **3.5 Iq/ms** | -50/ms = **3.5 Iq/ms; 130 ms from 6500** | state 1 while speed>0: decays at -50/ms | speed_native ≤ 0 in state 1 -> Q5C=0, mode 0 (chain.c:1525,1545) | **standstill = immediate zero** | G5300 | CONFIRMED_CODE |
| G13 | BDE8 tail `m2aa = Q5C*g1>>12` | BDE8 | m2aa | 1 ms | g1 applied **after** S5, so a g1 change is not slewed by BDE8 | same | — | — | — | G5300 | CONFIRMED_CODE |
| G14 | chain_e1e8_model | E1E8 | Q12 | 10 ms | +256 | -60 | latch | — | — | G5300 | CONFIRMED_CODE: **output not consumed on M820** |
| M1 | assist_pipeline.c:155-161 `iq_ceiling` | pipeline | Iq | 4 kHz fg | full scale in 400 ms (1.75 Iq/ms) | full scale in 120 ms (5.8 Iq/ms) | — | pipeline reset | — | M820 | CONFIRMED_CODE |
| M2 | assist_pipeline.c:187-193 R1 bumpless veto release | pipeline | Iq | 4 kHz fg | +50 m2aa/ms = 3.5 Iq/ms, only while `pulled_down` | never delayed | — | — | — | M820 (G5300-rate) | CONFIRMED_CODE |
| M3 | assist_pipeline.c:205-211 slew policy | pipeline | mode | 4 kHz fg | — | — | — | — | `stock_hard_zero` (`!normal_permission && m2aa==0`) -> FORCE_ZERO | M820 mapping of a G5300 event | CONFIRMED_CODE |
| F1 | fast_iq_slew.c `fast_iq_slew_tick` | fast_iq_slew | Iq Q10 | 16 kHz ISR | RISE: 0.625·P Q8 per 4 kHz tick = **6.84 Iq/ms** | FALL: same; SAFETY: linear from live in 200 ms | HOLD mode (unused on PEDAL) | FORCE_ZERO | ceiling clamp is immediate | port of G5300 0x0801A26C | CONFIRMED_CODE |
| F2 | foc_current_loop.c PI + `_U_MAX` | FOC | u_q/u_d | 16 kHz ISR | plant / current loop | — | integrator | bridge-off resets | QZERO compiled out (config.h:421) | M820 | CONFIRMED_CODE |

## 4. Zero, inhibit and decay map (who forces zero, and how)

| Event | Detector | Path at baseline | Effect on published Iq | Label |
|---|---|---|---|---|
| Comms loss | `comm_inhibit` | ride_control.c:205 owner switch | FORCE_ZERO, ceiling 0, pipeline (G53) reset | CONFIRMED_CODE |
| Battery hard trip | TIMER1 ISR `battery_trip_sample` | MOE off in the ISR; ride_control.c:223 | FORCE_ZERO until re-armed at standstill | CONFIRMED_CODE |
| Brake, overtemp stage ≥ 2, torque fault, torque calibration | main.c:3381 -> `safety_cut` | assist_pipeline.c:174 `native_cut` | request 0, **SAFETY linear 200 ms** from the live value. The chain also sees a fatal diag (0x10) and zeroes BDE8 | CONFIRMED_CODE |
| PAS sensor not seeded | `pas_sampler_seeded()==0` | `native_cut` | SAFETY 200 ms | CONFIRMED_CODE |
| Assist level 0 / level disabled | assist_pipeline.c:118 | exact zero, BYPASS | immediate 0 | CONFIRMED_CODE |
| Load released, crank still turning (TRUE_RELEASE) | EB74 threshold (load_ctrl ≲ 171 CLU) -> cur=0 | D7EC envelope/history decay (G4/G5) -> target -> E2 (≤ D3E) -> BDE8 | **seconds** (§6) | CONFIRMED_CODE / PROBE |
| Crank stops, load released | G53 PAS true-stop -> cadence 0 -> D7EC zero-reset (cur==0 ∧ cad==0) | EE=0 -> BDE8 state 1 -50/ms | 116 ms from 300 Iq | PROBE |
| Crank stops, load held | true-stop -> readiness lost -> C2=0 -> target 0 | E2 falls at D3E -409/10 ms | 691 ms from 300 Iq (1022 ms from a higher start [PRIOR]) | PROBE |
| Reverse step while moving | G53 PAS direction -1 -> D7EC `cad ≤ -1` hard clear (chain.c:498) | EE=0 -> BDE8 -50/ms | 130 ms from 6500 m2aa [PRIOR] | CONFIRMED_CODE |
| Standstill (speed_x100 < 10, i.e. 0 after 2.65 s pulse silence) | BDE8 state 1 with speed_native ≤ 0 (contract X3: speed_native = speed_x100/10) | `stock_hard_zero` -> FORCE_ZERO | ≤ 2 ms | CONFIRMED_CODE |
| `direction_inhibit`, `real_stop`, `forward_valid` (native) | pas_direction / pas_liveness | telemetry and `zero_policy` only (OWNER-DEC-2026-10-06-G5300-ONLY) | none | CONFIRMED_CODE |
| Walk / calibration | ride_control owner | own owner, BYPASS; pipeline reset on entry | — | CONFIRMED_CODE |
| Quiet Zero | — | `QUIET_ZERO_ENABLE 0` | none | CONFIRMED_CODE |

## 5. Q1: temporal shapers and duplicates

The stages that shape rise, hold or fall in ordinary riding (G5300 mode):

| Job | Shapers stacked today | Usually binding |
|---|---|---|
| Rise | G9 E2 rise D28 · G7 ratio +10/10 ms · G12 BDE8 +50/ms · G11 g1 soft start (boot only) · M2 R1 +50/ms (after native_cut) · M1 ceiling 400 ms · F1 6.84 Iq/ms | G9 (0.26-1.55 s full scale per level). BDE8 binds only at accel 8 |
| Hold | **G4 envelope peak-hold** · G5 history snapshot · G6 C2 retention · G10 hold-off (EE=0 hold on start) | G4 |
| Fall | **G4 envelope decay** (cadence-scaled) · G5 linear over 10.4 revs · G9 D3E -409/10 ms · G12 BDE8 -50/ms · F1 · SAFETY 200 ms | G4 when pedalling; G9 when the target collapses; G12 after hard clears |
| Pedal-ripple smoothing ("phase filter") | G4 envelope (time domain, cadence-scaled) · T3 crank-angle window (unused) | G4 |

Duplicates doing the same job:

- **Fall:** G4, G5, G9 and G12 are four stacked fall limiters. G4 is in the time domain, G5 in the crank-angle domain,
  G9 and G12 are fixed rates.
- **Rise:** G9, G12, M2 and M1 are four stacked rise limiters, plus F1 as the electrical guard.
- **Hold:** G4 and G6 overlap.
- **Battery:** g1 (G11) multiplies after the BDE8 slew. Only F1 smooths a g1 step.
- **Ceiling:** M1 is a protection envelope with its own rate. It shapes the recovery after a speed, thermal or
  undervoltage limit releases (1.75 Iq/ms). [INFERRED: rarely binding]

## 6. Q2: where the slow fall lives, with numbers

Host probe on the production modules (P=700, HMI level 3 = slot 6, 20 km/h, 60 rpm before the event,
`t10/t50/t90` = time to lose 10/50/90 % of the request) [PROBE]:

| Scenario | Iq before | t10 | t50 | t90 | zero | Max fall / ms | Binding mechanism |
|---|---:|---:|---:|---:|---:|---:|---|
| TRUE_RELEASE: load 3000 -> 0, keep 60 rpm | 300 | 371 | 2481 | 6421 | **7611 ms** | 3 | G4 envelope (E2 == target, BDE8 tracks) |
| same, keep 90 rpm | 300 -> **rises to 400** | 2111 | 4511 | 6431 | 6911 ms | 3 | G4. C2 ∝ env·cad, so the request grows when cadence rises after release |
| load 1500 -> 0, 60 rpm | 139 | 261 | 2311 | 4371 | 4921 ms | 3 | G4 |
| level 1, 3000 -> 0, 60 rpm | 91 | 401 | 2481 | 6371 | 7361 ms | 1 | G4 |
| S+ AUTO, 3000 -> 0, 60 rpm | 378 | 191 | 1271 | 3961 | 6821 ms | 4 | G4 (+ AUTO ratio falls with C4) |
| partial 3000 -> 1000, 60 rpm | 300 | 371 | 2481 | settles at 87 Iq after ≈ 4.5 s | — | 3 | G4 decays to the new `cur` |
| crank stops, load released | 300 | 38 | 73 | 107 | 116 ms | 4 | true-stop -> zero-reset -> BDE8 -50/ms |
| crank stops, load held | 300 | 101 | 361 | 622 | 691 ms | 4 | readiness lost -> D3E |

Fixed constants [CONFIRMED_CODE]:

- **D3E (D7EC accel fall)** = -409 per 10 ms on E2 (full scale 40960) -> EE -40.9 Q12 / 10 ms -> m2aa -6.5/ms ->
  **0.455 Iq/ms**. 455 Iq -> 0 takes 1.0 s; 300 Iq -> 0 takes about 0.66 s.
- **Envelope (D7EC)**: τ = (8·cad+1)·10 ms, with an integer floor of ≥ 1 unit per 10 ms. The release time grows
  with cadence: about 4.9-7.6 s to zero at 60-90 rpm in the probe.
- **History candA**: linear to zero over 1000 PAS transitions (10.4 crank revolutions at 96 per revolution).
- **BDE8 S5**: ±50 m2aa/ms = 3.5 Iq/ms; 6500 -> 0 in 130 ms.
- **Fast slew**: 6.84 Iq/ms; never binding on ordinary releases.

## 7. Q3: where pedal phase and cadence enter

| Entry | Domain | Used for demand? | Label |
|---|---|---|---|
| Raw `pas_ab` -> G53 PAS (96 transitions/rev) | transition counts, ticks | yes: cadence (signed, filtered), evidence count, movement, true-stop timeout -> D7EC | CONFIRMED_CODE |
| D7EC envelope decay `k = 8·cad` | time, cadence-scaled | yes: hold/fall | CONFIRMED_CODE |
| D7EC C2 = env·cad·35/10000 (cad > 20), LUT(cad) normalization | cadence | yes: gain | CONFIRMED_CODE |
| D7EC history candA over PAS evidence | **crank-angle domain** (count of 3.75° steps) | yes: fall after release | CONFIRMED_CODE |
| torque_input RUN window, 180° = 48 forward steps | **crank-angle domain** | **no**, telemetry only | CONFIRMED_CODE |
| M820 `cadence_filter` (IIR 1/8 per pulse) | rpm | no (rider-power telemetry, Walk) | CONFIRMED_CODE |
| Absolute crank angle / pedal phase index | — | **does not exist**. There is no index sensor; `pas_fwd_accum` (main.c:549) is a free-running forward count, diagnostics only | CONFIRMED_CODE |

A phase reference would have to be inferred, for example from torque-ripple minima against a step counter
mod 96. [HYPOTHESIS]

## 8. Q4: recommended single trajectory owner

### Recommendation: `assist_v3` stage at the Boundary-B seam inside `assist_pipeline_update()`

```text
native_cut / assist_off evaluation (unchanged, upstream-authoritative)
g53_port_update(...)                 // ALWAYS runs: shadow G53, PAS, EB74 zero, g1, telemetry, G5300 fallback
if (behavior == V3)  req = assist_v3_update(&v3_in) * g1 >> 12;   // the ONE trajectory
else                 req = ctx.g53.iq_request_pre_limits;          // baseline, bit-identical
ap2_limits_apply(req)                // envelope only (power/phase/UV/thermal/speed)
zero/slew policy                     // native_cut SAFETY, assist_off, standstill FORCE_ZERO, else RISE/FALL guard
fast_iq_slew (16 kHz)                // electrical anti-step only
```

Why this location:

- It is the only point where the demand is already in the Iq domain, with every safety gate and every limit envelope
  downstream or alongside, but no G53 shaper is consumed.
- The transcribed chain stays untouched, so TQ-02B/TQ-04 parity and oracle tests remain valid.
- G5300 stays available as a one-line fallback.
- Rate is 4 kHz with `elapsed_ticks`, the same timebase as the rest of the pipeline.

**Neutralise when V3 is active** (by not consuming them; never by editing the transcription):

1. D7EC: G4 envelope, G5 history, G6 readiness, G7 ratio limiter, G9 accel rise/fall (incl. **D3E**), G10 hold-off.
2. BDE8: G12 S5 ±50/ms and the state 7->1 decay. The shadow `normal_permission`/`m2aa` must not drive
   `stock_hard_zero` in V3 mode. Re-derive the standstill hard zero in the pipeline from V3 demand == 0 and
   `speed_native ≤ 0`, the same physical condition as BDE8 C1B6.
3. EB74 as demand source. V3 reads `load_ctrl` itself; it may read the EB74 auto-zero as an observation if exposed.
4. Pipeline R1 bumpless veto release (assist_pipeline.c:187-193). V3 owns re-engagement from `last_published_iq`;
   otherwise R1 is a second rise limiter.

**Keep as is:**

- `native_cut` -> SAFETY 200 ms; assist_off; comm/trip FORCE_ZERO.
- `ap2_limits`, and the `iq_ceiling` slew (document it as a protection envelope).
- **g1 as a multiplicative battery envelope**, the same `·g1>>12` BDE8 applies. Its limit update ignores the BDE8
  state except state 6 (throttle, unreachable) [INFERRED], so running it beside V3 is safe.
- Fast slew RISE/FALL 0.625·P. Add a V3 test asserting the guard never binds in normal riding, i.e. the V3 trajectory
  stays below 6.84 Iq/ms.

**Fallback and selection:**

- Default = G5300.
- The selector is latched only while the published request is 0, or at standstill.
- The G53 chain steps every tick in both modes, so a switch back is warm. A V3 internal fault forces a fallback at
  the next zero, or a bounded decay.
- Pipeline reset on an owner change also resets V3.

### Alternatives (worse)

| Alternative | Why worse |
|---|---|
| **A1: inside the G53 chain at the D7EC output** (V3 writes EE / Q+0xA, keeps BDE8 and g1 natively) | BDE8 ±50/ms stays a second shaper: V3 cannot release faster than 130 ms full scale or attack faster than 3.5 Iq/ms. Output is quantised to 10 ms and Q12, capped at 6500. It writes into the transcribed register image, which breaks the port's bit-exact and oracle claims. Fallback switching then sits inside the transcription |
| A2: shape `load_ctrl` before G53 | The D7EC envelope still holds and decays on top, so TRUE_RELEASE stays slow. Pure duplication |
| A3: inside `fast_iq_slew` (16 kHz ISR) | ISR context with no sensor access. It mixes behavior with the electrical guard and the SAFETY/FORCE_ZERO modes, against the rule that the final slew is only an anti-step guard |

## 9. Q5: minimal seams for a V3 behavior module

```c
typedef struct {
    uint32_t elapsed_ticks;                 /* 4 kHz periods (existing) */
    /* rider */
    uint16_t torque_load_ctrl;              /* existing: unfiltered CLU */
    bool     torque_sensor_valid;           /* existing */
    /* crank */
    uint8_t  pas_ab;                        /* existing raw state */
    int32_t  crank_steps;                   /* NEW: signed accumulated quadrature steps (96/rev) */
    uint32_t crank_last_edge_tick;          /* NEW: 4 kHz tick of last accepted PAS edge */
    int16_t  cadence_rpm_signed;            /* from ctx.g53.trace.cadence (G53 PAS) */
    uint16_t pas_no_transition_ticks, pas_stop_timeout_ticks;  /* from g53 trace */
    bool     direction_inhibit, pas_backward, real_stop;       /* existing native observations */
    /* vehicle */
    uint32_t speed_x100;  bool wheel_valid; /* existing */
    uint32_t speed_sample_tick;             /* NEW: tick of last wheel edge, for a real dv/dt */
    /* motor / electrical (read-only) */
    int32_t  live_iq_ref;                   /* existing (MS.i_q_setpoint) */
    int32_t  iq_measured;                   /* NEW: MS.i_q */
    uint16_t motor_erps;                    /* existing */
    int32_t  last_published_iq;             /* pipeline ctx.last_final_iq */
    int32_t  phase_current_max;             /* existing */
    uint32_t battery_voltage_mv; int32_t battery_current_ma;   /* existing */
    uint16_t g1_q12;                        /* from ctx.g53.trace.g1: observation, applied by pipeline */
    /* mode */
    uint8_t  assist_level_index;            /* existing; level params via assist_modes / 0x6010-0x6011 */
    bool     brake_active;                  /* NEW: observation only, the cut stays in native_cut */
} assist_v3_input_t;

int32_t assist_v3_update(const assist_v3_input_t *in);  /* SINGLE output: Iq demand, pre-limits, >= 0 */
void    assist_v3_reset(void);                         /* called from assist_pipeline_reset() */
const assist_v3_telemetry_t *assist_v3_telemetry(void);/* read-only diagnostics, not a control output */
```

New plumbing needed:

- `crank_steps` and `crank_last_edge_tick`: derived in the main.c:2772 PAS event drain, carried through
  `rider_input_t` -> `assist_pipeline_input_t`.
- `speed_sample_tick`: from Speed_processing.
- `iq_measured` and `brake_active`: added to `ride_control_input_t`.
- A behavior selector, plus V3 config, through the config protocol (audit C/D scope).

V3 must not write slew mode, ceiling or zero policy. The pipeline derives all three.

## 10. Q6: safety-authoritative, not owned by the behavior layer

- Owner arbitration and its zeros: comm_inhibit, battery hard trip (ISR MOE off), calibration, Walk, the
  walk-release cut (ride_control.c).
- `native_cut`: brake, overtemp stage ≥ 2, torque range/stuck/calibration fault, torque calibration, PAS not seeded.
  These map to SAFETY 200 ms. Also assist_off.
- Limit envelopes: ap2_limits (phase/level, undervoltage, thermal 75-90 °C, legal speed taper), `iq_ceiling` and its
  slew, the fast_iq_slew ceiling clamp, g1 PI #1 (ADR-013 battery owner).
- Standstill hard zero (FORCE_ZERO): derived by the pipeline, not by V3.
- FOC: `_U_MAX` vector limit, phase over-current trip, bridge lifecycle, current PI.
- **Open policy point:** at baseline, reverse and stop are G53 *behavior* (OWNER-DEC-2026-10-06-G5300-ONLY), not
  safety. In V3 mode nothing outside V3 would bound a stuck positive demand while the crank is stopped or
  reversed. Recommend a pipeline backstop as a bounded decay, not a step, keyed on G53/native `real_stop` and
  `direction_inhibit`, armed only in V3 mode. That needs an owner decision (EN 15194 stop distance; R-1 in
  EXEC-EVD-STOP-RAMP-001). [INFERRED requirement]

## 11. Discoveries (captured, not chased)

| ID | Finding | Label |
|---|---|---|
| A-D1 | `level_iq_limit` (per-level max_iq_pct) is computed in ride_control.c:330 but the pipeline forces `AP2_LIMITS_NO_LEVEL_CEILING` (assist_pipeline.c:139). The per-level Iq ceiling is inert on PEDAL | CONFIRMED_CODE |
| A-D2 | `g53_port_init()` is never called. g1 and `g1_configured_limit` rely on static zero (`0`, not `-1`) | CONFIRMED_CODE ([PRIOR] judged equivalent) |
| A-D3 | E1E8 runs every 10 ms but its outputs (M+0x2A2/2A4) feed only telemetry (taper input fixed 1.0) | CONFIRMED_CODE |
| A-D4 | With the load released, rising cadence **increases** assist (C2 ∝ env·cad): 300 -> 400 Iq at 90 rpm in the probe | PROBE |
| A-D5 | D7EC reverse clears EE but keeps the live E2. Resuming forward restarts the trajectory from the retained E2 (BDE8-slewed) | CONFIRMED_CODE (G5300-faithful) |
| A-D6 | ap2_rider_demand / estimators / pas_state / profiles are compiled but dead; docs/ASSIST_PIPELINE_V2.md §1-10 are stale | CONFIRMED_CODE |
| A-D7 | Foreground catch-up replays a held `pas_ab`, so edges inside a coalesced burst are lost to the G53 PAS | CONFIRMED_CODE |
| A-D8 | EB74 auto-zero needs `speed_native > 1000`. With the X3 adapter unit of 0.1 km/h that is 100 km/h, so the auto-zero never arms and the EB74 zero stays at 750. The base threshold is then fixed at 995 counts (load_ctrl ≈ 600 CLU) for engage, and 820 (≈ 171 CLU) for release. The D7EC speed taper (D22) would hit the same unit question, but it is disabled | CONFIRMED_CODE / INFERRED |

## 12. Unknowns and blockers

- [UNKNOWN] Semantics of BDE8 G04 (M+0x299) and of D25, which open the D7EC hold-off window.
- [UNKNOWN] On-bike magnitude of the TRUE_RELEASE hold. The probe uses a constant load step; real pedal ripple
  re-arms the envelope attack every stroke. Needs a ride log (audit B/D / HW).
- [INFERRED] Wheel speed reads 0 below about 3 km/h ([PRIOR] TQ-06-G2); this sets where `stock_hard_zero` takes over.
- No blocker for the owner choice. The backstop policy in §10 needs an owner decision before V3 rides.

## 13. Reproduction

The probe source is in the session scratchpad, not in the repo, and is not a qualified test tool (RULE 70:
observation only). It is the reverse_ramp_host harness with these changes: load stepped at t=4 s, cadence kept or
stopped, D7EC/BDE8 trace printed every 250 ms. Build:
`gcc -std=c11 -Itests/host/common/host_stubs -Itests/host/common -Iinc probe.c src/g53_port.c src/g53_port_boundaries.c
src/g53_port_pas.c src/g53_port_chain.c src/g53_g1_limiter.c src/ap2_limits.c src/assist_pipeline.c src/assist_modes.c
src/torque_input.c src/tuning_config.c src/battery_iq_cap.c src/fast_iq_slew.c`.
