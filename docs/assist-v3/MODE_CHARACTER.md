# Assist Behavior V3 — Mode Character and User Parameter Model

```text
STATUS:   DESIGN rev 2 — REVIEW-T (2026-10-07T22:49:29+02:00, CHANGES_REQUIRED) issues resolved; re-check pending
SCOPE:    semantics; consumers land per milestone (§10). Numbers are candidates (ENVELOPE_STUDY, SIL matrix).
RELATES:  ARCHITECTURE_V3.md (layers, §5 resolver/ratio-law API), CONFIG_PROTOCOL_V3.md (wire), DECISIONS D-025..D-039
```

## 1. Five separated layers

```text
RIDER INTENT        what the rider is doing          assist_v3_intent   (never configured by the user)
MODE CHARACTER      how this mode interprets it      hidden profile + user macros/overrides -> internal params
PHYSICAL ENVELOPE   what the motor may deliver       Max Torque, Max Power, battery g1, thermal, legal
TRANSIENT BEHAVIOUR how fast demand moves            attack, release, start, carry (V3-5/V3-6)
SAFETY              what always wins                 native_cut, owners, limits, backstop, standstill zero
```

A user setting reaches only its own layer: character never touches the intent detector or safety; an envelope
never changes classification, release timing or the engage threshold.

## 2. Autonomous modes

Each mode is a complete strategy with firmware defaults for every parameter. With **no user override** the firmware
must pass the whole behaviour matrix in every mode (G2-AUTONOMOUS). Configuration personalises; it never repairs.

| Mode | Character (hidden profile intent) | Invariants (G2-MACRO must keep them at every Assist) |
|---|---|---|
| ECO | Assist acts on base support; calm response; lower power ceiling; usable torque for slow climbs; small carry | flat ratio law; power ceiling below the other modes; slowest attack/release rates of all modes |
| TRAIL | natural response; flat torque support below 40 rpm; medium/strong carry | flat ratio law; c_floor 40 rpm (highest low-cadence ratio); carry strength ≥ SPORT |
| SPORT | reaches the envelope quickly; fast attack; fast clean release | flat ratio law; release faster than TRAIL and ECO |
| SPORT+ | progressive "SPORT+ ratio law" (legacy S+ AUTO law); very fast attack; most active carry | progressive law (slope > 0); attack and release the fastest; carry strength the highest |
| BOOST | strongest fixed support (legacy L5, ratio 525 %) | flat law; highest base ratio at default |
| AUTO | user sets range, envelopes, response tendency, carry, adaptation; the algorithm picks the operating point (terrain state; IMU optional, never required) | operating point stays inside [range_min, range_max] |

Default level -> mode map (global object, configurable): **L1 ECO, L2 TRAIL, L3 SPORT, L4 SPORT+, L5 BOOST**
(owner decision 2026-10-07, D-032). AUTO is assignable to any level. Until the mode-character milestone the levels keep
their legacy G5300 behaviour (Milestone C keeps the legacy characteristic).

Naming: "SPORT+ ratio law" = the legacy G5300 S+ AUTO interpolation (progressive support). "AUTO mode" = the V3
terrain-chosen operating point (Milestone F). They are different concepts (glossary item).

AUTO between the mode-character milestone and F: a static point inside its range (TRAIL default), labelled in
readback as source "profile default". From its first version the F terrain state must have a non-IMU source.

## 3. Character profile (hidden, firmware-owned)

Each mode carries tendencies (support, torque, power, attack, release, start, carry, adaptation). They define how the
BASIC macros map onto internal parameters: for each internal parameter the profile stores `(value at macro 0, value at
macro 100)` and the macro's default. "Assist 70" means a different curve in ECO and in SPORT (intended); inside a mode
every mapping is monotone and deterministic.

## 4. BASIC parameters (macros)

| Param | Rider meaning | Drives (unless overridden in ADVANCED) | Never drives |
|---|---|---|---|
| Assist 0..100 | overall support feel | assist base (flat modes), progression + range max (SPORT+ law) | envelopes, Response, Carry, engage threshold, any safety/battery/thermal/legal limit |
| Max Torque 10..100 % | torque ceiling of the mode, % of the V3 demand full scale 0.65·P | Iq ceiling (pipeline `level_iq_limit` from the V3 resolver only) | support curve, release |
| Max Power W | sustained battery power ceiling of the mode | g1 battery-current limit (closed loop; §4.2) | support curve, release, classification |
| Response 0..100 | how quickly assist follows you up and down | attack, release | envelopes, start, classification |
| Start 0..100 | how strongly assist starts from rest | start boost and start rise (one internal start profile) | engage threshold (EB74 820/995 stays), normal riding |
| Carry 0..100 (0 = off) | push continuing when you briefly stop pedalling under load | carry strength, time cap, distance cap (within hard bounds) | normal riding, release while pedalling |

Assist, Max Torque and Max Power have a static effect (ENVELOPE_STUDY); Response, Start and Carry are transient and
are exposed only after the SIL override/orthogonality matrix shows an independent, predictable effect.

### 4.1 Support function (mode-character milestone)

The V3 support stage is the G5300 static map with two injected inputs (ARCHITECTURE §5): a ratio law
`ratio = clamp(base + slope·c4, range_min, range_max)` and a torque/power crossover `cad_eff = max(cad, c_floor)`
(plus the high-cadence bias above 100 rpm). Flat modes: `slope = 0` by default; SPORT+: legacy law `base 1,
slope 524/200, max 525`. At the default Assist every profile reproduces its legacy level exactly above `c_floor`.
Below `c_floor` support is torque-proportional (also during a start from rest at measured cadence 0..20 rpm — a
deliberate deviation, ENVELOPE_STUDY §5).

### 4.2 Physical envelopes

- **Max Torque**: `level_iq_limit = MaxTorque_eff % × 0.65·P / 100`, computed by the pipeline from the V3 resolver
  only, never from the bank field `max_iq_pct` and never from `ride_core_iq_limit` (which carries the legacy limp
  factor). G5300 mode unchanged.
- **Max Power**: a closed-loop battery-current limit through the existing g1 PI owner: per-level percentage
  `pct = round(100 · W / (V_batt · 15 A))`, updated with the g1 limit every 10 ms. Steady state accurate; transient
  overshoot bounded by the PI (to be measured in SIL). The profile default of TRAIL, SPORT, SPORT+, BOOST and AUTO is
  **hardware maximum** (pct 100 = legacy parity at any pack voltage); ECO defaults to a lower W. The SOC knee derate
  must stay active below a mode cap: the implementation multiplies the mode percentage with the SOC derate instead of
  replacing the base (requirement for the mode-character milestone; today a cap below the knee would make the derate
  inert, REVIEW-T #10).

## 5. ADVANCED parameters and applicability

| Param | Unit / mapping | Applicable in | Notes |
|---|---|---|---|
| assist_base | ratio %, 0..1000 (internal unit) | all | in flat modes the Assist macro moves it |
| assist_progression | 0..100 -> `slope = progression/100 · slope_max(mode)` (inverse used for the effective view) | all | 0 in flat modes by default = linear; > 0 makes support progressive |
| assist_range_max | ratio %, 0..1000 | modes with effective progression > 0 | ceiling of the progressive law; otherwise "not applicable" |
| assist_range_min | ratio %, 0..1000 | AUTO only | floor of the dynamic range |
| attack | 0..100 | all | Response macro splits into attack + release |
| release | 0..100 | all | the Milestone C consumer |
| max_acceleration | 0..100 | candidate | exposed only if SIL/L4 shows it is not a duplicate of attack |
| phase_compensation | 0..100 | candidate | exposed only if SIL shows riders can predict it |
| carry_strength / carry_time_limit (ms) / carry_distance_limit (dm) | as named, ≤ firmware hard maxima | all | Carry macro splits into these |
| terrain_adaptation | 0..100 | AUTO (F) | |
| high_cadence_bias | −50..+50 | all (E) | acts only above 100 rpm |

There is no ADVANCED start field: Start drives one internal start profile. Ranges contain every profile value at
Assist 0..100 (G1-CFG2 vector). Internal algorithm constants (template α, classifier thresholds/hysteresis, bins,
confidence decay, filters, epsilons) are never user parameters; diagnostics see them only in DIAG telemetry.

## 6. DEFAULT, override and legacy inputs — the one precedence rule

Resolution is done **per HMI level** at the moment it is needed: `effective(level) = resolve(mode(level), legacy(level))`.

1. An internal parameter with an **ADVANCED override** takes that value (source "user override").
2. Otherwise it is derived from its **BASIC macro** through the mode curve (source "macro-derived").
3. A BASIC macro takes its **override** if set; otherwise its DEFAULT, which is the profile default **adjusted by the
   legacy input of that level** (source "legacy input" when the legacy value differs from the compiled factory value,
   else "profile default"):
   - Assist: `a = clamp(a_default + inverse_curve_mode(P0_ratio(level) / P0_factory(level)), 0, 100)`, where the
     inverse curve is that of `base(a)` (flat modes) or `range_max(a)` (SPORT+ law) and `P0_factory` is the bank
     default compiled into the firmware;
   - Max Power: `profile_W(level) × P1_power_pct(level) / 100`;
   - Response (attack part): from the P0 per-level acceleration (1..8 -> attack curve), release unaffected.
4. Storage keeps overrides only; restore one mode / restore all sets overrides to DEFAULT. Defaults live in firmware,
   so firmware updates can improve them without touching user choices.

Consequences, stated for the owner and the clients:

- A V3 override **shadows** the stock field of the same meaning: the stock HMI app's P0/P1 edit then has no effect on
  that level. CAPS/STATUS carries `legacy_shadowed` (one bit per level) so CANable can warn.
- A P0/P1 write never clears V3 overrides (CANable writes all P0 values back).
- Every step is monotone because each curve and its inverse are monotone.

## 7. Configured vs effective readback

For every parameter of a level: configured (DEFAULT or value), effective (the resolved configuration, never the
momentary limited value — that is telemetry) and source: 0 profile default, 1 macro-derived, 2 legacy input,
3 user override, 4 limited by firmware, 5 not applicable in this mode, 6 shadowed by an ADVANCED override (the macro
then does nothing for that internal parameter and the UI must say so).

## 8. CANable model (later; firmware is canonical)

```text
SPORT                              ADVANCED >
[ Assist        75 ]               [ Assist base     AUTO ]   AUTO = DEFAULT (firmware/profile)
[ Max Torque    85 ]               [ Progression     AUTO ]   fields marked "not applicable" are hidden
[ Max Power   700 W]               [ Range max       n/a  ]
[ Response      70 ]               [ Attack / Release AUTO ]
[ Start         65 ]               [ Carry time / distance AUTO ]
[ Carry         70 ]               [ High-cadence bias AUTO ]
```

CANable reads capabilities, defaults, effective values, sources and configured overrides; it never stores defaults.
A phone app uses the same contract.

## 9. RAM budget

Configured values for 5 modes + global ≈ 256 B (one RAM image; the saved view is read from the memory-mapped CONFIG_A
log, no second copy); effective cache for the active level only (≈ 48 B, recomputed on generation or level change).
NORMAL build headroom after Milestone B: 840 B (REVIEW-T measurement), so config v2 leaves ≈ 0.5 KB for carry (D),
motion estimate (D) and terrain (F). Next lever if needed: D-024 rejected list.

## 10. When each part becomes active

| Milestone | Active consumers |
|---|---|
| C active release | release (ADVANCED ID 13) and engine; Response stays masked until attack exists; legacy G5300 characteristic and P0/P1 |
| D carry | Carry macro + carry strength/time/distance |
| Mode character (after D, with E) | Assist macro, base/progression/range_max, Max Torque, Max Power (W), Start, attack (Response unmasked) |
| F terrain/AUTO | AUTO mode, range_min, terrain adaptation, high-cadence bias |

Every parameter is on the wire from the first v2 contract; its applicability and `param_mask` bit turn on with its
consumer.
