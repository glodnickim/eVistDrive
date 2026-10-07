# Assist Behavior V3 — Mode Character and User Parameter Model

```text
STATUS:   DESIGN DRAFT (owner override 2026-10-07: autonomous modes + BASIC/ADVANCED configuration)
SCOPE:    semantics only; consumers land per milestone (§9). Numbers are candidates for the envelope simulation.
RELATES:  ARCHITECTURE_V3.md (layers), CONFIG_PROTOCOL_V3.md (wire), DECISIONS D-025..D-031
```

## 1. Five separated layers

```text
RIDER INTENT        what the rider is doing          assist_v3_intent   (never configured by the user)
MODE CHARACTER      how this mode interprets it      hidden profile + user macros/overrides -> internal params
PHYSICAL ENVELOPE   what the motor may deliver       Max Torque, Max Power, battery g1, thermal, legal
TRANSIENT BEHAVIOUR how fast demand moves            attack, release, start, carry (V3-5/V3-6)
SAFETY              what always wins                 native_cut, owners, limits, backstop, standstill zero
```

A user setting can only reach the layer it belongs to. Changing a mode's character never touches the intent detector
or safety; changing an envelope never changes classification or release timing.

## 2. Autonomous modes

Each mode is a complete strategy with firmware defaults for every parameter. With **no user override** the firmware
must pass the whole behaviour matrix in every mode (TEST_MATRIX G2-AUTONOMOUS). Configuration personalises; it never
repairs defaults.

| Mode | Character (hidden profile intent) |
|---|---|
| ECO | Assist acts mainly on base support; calm response; lower power ceiling; torque still useful for slow climbs; small carry |
| TRAIL | Assist uses the dynamic range strongly; natural response; strong low-speed torque; medium/strong carry |
| SPORT | Assist reaches the torque/power envelope quickly; fast attack; fast clean release (no rubbery hold); high power |
| SPORT+ | Large use of the envelope; very fast attack; aggressive transients; most active carry when intent qualifies |
| AUTO | User sets range, envelopes, response tendency, carry, adaptation; the algorithm picks the operating point (terrain state; IMU optional, never required) |

Default level -> mode map (configurable, global object): L1 ECO, L2 TRAIL, L3 SPORT, L4 SPORT+, L5 AUTO. Until the
mode-character milestone is active the levels keep their legacy G5300 behaviour (Milestone C keeps legacy ratio).

## 3. Character profile (hidden, firmware-owned)

Each mode carries a table of tendencies: support, torque, power, attack, release, start, carry, adaptation. A tendency
is not a user field. It defines **how the user macros are interpreted** for that mode: for every internal parameter
the profile stores `(value at macro 0, value at macro 100, default macro)`. So "Assist 70" means a different curve in
ECO and in SPORT (intended), but inside a mode every macro is monotone and deterministic.

## 4. BASIC parameters (macros)

| Param | Rider meaning | Drives (internal, unless overridden in ADVANCED) | Never drives |
|---|---|---|---|
| Assist 0..100 | overall support feel: how easily and how strongly the mode uses the motor's capability in response to intent | assist base, assist range min/max, progression | Max Torque, Max Power, Response, Carry, any safety/battery/thermal/legal limit |
| Max Torque 10..100 % | hard torque ceiling of the mode | torque envelope | support curve, release |
| Max Power W | hard power ceiling of the mode | power envelope (via the existing power owner) | support curve, release |
| Response 0..100 | how quickly assist follows you up and down | attack, release (per character tendency) | envelopes, start threshold, classification |
| Start 0..100 | how strongly assist starts from rest | start response | normal riding |
| Carry 0..100 (0 = off) | how much push continues when you briefly stop pedalling under load | carry strength, time cap, distance cap (within hard bounds) | normal riding, release while pedalling |

Six macros are the candidate BASIC set. Each must show an independent, predictable effect in the override matrix
(TEST_MATRIX G2-OVERRIDE / G2-ORTHO) before the UI shows it.

### 4.1 Assist as a macro — the support function (mode-character milestone)

```text
desired_support = S(I, cadence, terrain; base, range_min, range_max, progression)
  S rises from `base` at low intent towards `range_max` as intent grows; `progression` sets how much intent is
  needed to reach the upper part of the range; `range_min` is the floor once engaged.
Assist a -> base(a), range_max(a), progression(a): each non-decreasing in a, from the mode profile.
motor demand = desired_support × rider mechanical input (torque/power blend, Milestone E)
               -> clipped by Max Torque / Max Power envelopes -> transient layer -> safety.
```

Concrete model (ENVELOPE_STUDY.md §2, verified on the production surface): the V3 support stage is the G5300 static
map with an injected ratio law `ratio = clamp(base + slope·c4, range_min, range_max)` and a torque/power crossover
`cad_eff = max(cad, c_floor)` (plus a high-cadence bias above 100 rpm). Fixed levels are `slope = 0`; S+ AUTO is
`base 1, slope 524/200, max 525`. At the default Assist every profile reproduces its legacy level exactly above
`c_floor`. Max Torque uses the existing `level_iq_limit` path; Max Power is the battery power limit enforced by the
existing g1 owner (limit = W / V_batt, at most 15 A).

Monotonicity rule: for every mode, intent, cadence and terrain, `a2 > a1 ⇒ demand(a2) >= demand(a1)` before
envelopes; after envelopes equality is allowed only where an envelope binds (documented). Tested on Assist 20/40/60/80/100
× 25/60/90/120 rpm × every mode (G2-MACRO).

## 5. ADVANCED parameters (candidates)

| Param | Candidate test (predictable? independent? not a duplicate?) |
|---|---|
| Assist base | yes: support at light effort |
| Assist range min / max | yes: floor / ceiling of support ratio |
| Assist progression | yes: effort needed to reach upper range |
| Attack | yes: rise speed (Response splits into attack + release) |
| Release | yes: fall speed while pedalling |
| Start response | yes |
| Max acceleration | candidate: caps demand when the bike accelerates hard; must show it is not a duplicate of Attack in the matrix |
| Phase compensation | candidate: 0 = follow the stroke (pulsing), 100 = full compensation; exposed only if the matrix shows riders can predict it |
| Carry strength / time limit / distance limit | yes: split of the Carry macro; limits bounded by firmware hard maxima |
| Terrain adaptation strength | yes (AUTO / F): how much terrain state moves the operating point |
| High-cadence support bias | candidate (E): biases the torque/power blend above ~100 rpm |

Not user parameters, ever: template α, classifier thresholds/hysteresis, bin counts, confidence decay, filter
coefficients, epsilons. They are implementation (ARCHITECTURE §4); diagnostics may read them via DIAG telemetry only.

## 6. DEFAULT + OVERRIDE

- Each public parameter of each mode is either **DEFAULT** (`0xFFFF` on the wire and in storage) or a **user override**.
- Storage holds overrides only: profile defaults live in firmware, so a firmware update can improve defaults without
  touching conscious user choices.
- Precedence for an internal parameter: explicit ADVANCED override > value derived from the BASIC macro (macro itself
  DEFAULT or override) > profile default. A BASIC macro influences only components that are not overridden in
  ADVANCED.
- Legacy owners (D-011 refined): the stock M560 P0/P1 per-level values (assist ratio, acceleration, power %) are kept
  as **legacy inputs**: when the V3 parameter of the same meaning is DEFAULT, the profile default is adjusted by the
  legacy value relative to the factory value (so the stock HMI app still works); a V3 override wins over the legacy
  input. One effective value, one documented precedence, source reported in readback.
- Restore: one mode -> all its overrides DEFAULT; all -> every V3 override DEFAULT. No manual entry of factory values.

## 7. Configured vs effective readback

For every parameter of a mode the protocol returns: configured (DEFAULT or value), effective value, and source
(profile default / macro-derived / legacy P0-P1 input / user override / limited by firmware). AUTO and the future
terrain state report the effective operating point the same way.

## 8. CANable model (later; firmware is canonical)

```text
SPORT                              ADVANCED >
[ Assist        75 ]               [ Assist min     AUTO ]   AUTO = DEFAULT (firmware/profile)
[ Max Torque    85 ]               [ Assist max     AUTO ]
[ Max Power   700 W]               [ Progression    AUTO ]
[ Response      70 ]               [ Attack / Release AUTO ]
[ Start         65 ]               [ Max accel      AUTO ]
[ Carry         70 ]               [ Phase comp     AUTO ]
                                   [ Carry time / distance AUTO ]
                                   [ Terrain adapt  AUTO ]
```

CANable reads capabilities, defaults, effective values and configured overrides; it never stores defaults. A phone
app uses the same contract.

## 9. When each part becomes active

| Milestone | Active consumers |
|---|---|
| C active release | Response (-> release; attack stays legacy), engine; legacy G5300 characteristic and P0/P1 |
| D carry | Carry macro + carry strength/time/distance |
| Mode character (after D, with E) | Assist macro + base/range/progression, Max Torque, Max Power (W), Start, Attack, Max accel (if kept) |
| F terrain/AUTO | AUTO mode, terrain adaptation, high-cadence bias |

Every parameter is on the wire from the first contract version; its `param_mask` bit turns on with its consumer.
