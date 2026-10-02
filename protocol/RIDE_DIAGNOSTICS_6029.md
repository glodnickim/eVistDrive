# Ride diagnostics — 0x6029

The controller's live diagnostics snapshot, read by `canable-web` over the multiframe transport.
`src/CAN_Display.c` builds it; `canable-web/bafang-parser.js` decodes it. This file is the
contract between them, and it exists because there was none: the firmware comment pointed here
and the document did not exist, so the only description of the v7 field meanings was the packing
code itself.

**The version byte is the whole safety mechanism.** Byte 2 says which meanings the bytes carry.
Several slots kept their position and changed what they hold, so a decoder that accepts a version
it does not understand does not fail — it silently reports one quantity under another one's name.
A decoder must refuse a version it has no map for.

| Version | Body | Total | Added |
|---|---|---|---|
| 1 | 22 | 24 | peak-hold only |
| 2 | 30 | 32 | flags byte, live PAS idle / pressure / Iq |
| 3 | 35 | 37 | RUN pressure, measured `i_q`, battery-limit flag |
| 4 | 45 | 47 | cadence compensation, `u_abs`, pack voltage |
| 5 | 53 | 55 | Extended Boost state |
| 6 | 69 | 71 | the unit-domain block: pedal load, conversion anchors, crossfade |
| 7 | 69 | 71 | **Assist Pipeline V2.** Same length and CRC, new meanings — see below |
| 8 | 69 | 71 | **TQ-06 G53.** Same length and CRC, native G53 and limiter observations — see below |

CRC-16/CCITT-FALSE (init 0xFFFF, poly 0x1021) over the body, little-endian in the two bytes that
follow it. All multi-byte values are little-endian.

## v7 — the Assist Pipeline V2 map

v7 is the same 71 bytes as v6. What changed is that the mechanisms behind several v6 fields no
longer exist — Extended Boost, the launch/measured-duty crossfade, cadence compensation — and the
pipeline that replaced them has its own questions to answer. The slots were reused rather than
appended because the packet is at its length; the version byte is what makes that a versioned
change instead of a silent reinterpretation.

| Offset | Type | v7 meaning | v6 meaning at the same offset |
|---|---|---|---|
| 0..1 | `'D','G'` | magic | same |
| 2 | u8 | version = 7 | 6 |
| 3 | u8 | deprecated engine id | same |
| 4 | u8 | cadence, **peak** [rpm] | same |
| 5 | u8 | flags, see below | same |
| 6..7 | u16 | **rider demand, peak [permille]** | torque to assist, peak [mV] |
| 8..9 | u16 | rider power, peak [W] | same |
| 10..11 | u16 | applied support ratio, peak [%] | same |
| 12..13 | u16 | motor power, peak [W] | same |
| 14..15 | u16 | requested battery current, peak [mA] | same |
| 16..17 | u16 | Iq request, peak | same |
| 18..19 | u16 | Iq setpoint, peak | same |
| 20..21 | u16 | speed [0.01 km/h] | same |
| 22..23 | u16 | PAS idle, live [ms] | same |
| 24..25 | u16 | **rider demand, live [permille]** | fast pressure, live |
| 26..27 | u16 | Iq request before the final ramp, live | same |
| 28..29 | u16 | Iq setpoint, live | same |
| 30..31 | u16 | **assist base — the sustained term, live [permille]** | RUN pressure, live |
| 32..33 | i16 | measured `i_q`, live (signed) | same |
| 34 | u8 | bit0 = battery-current limiter active | same |
| 35..36 | u16 | **assist dynamic — the reactive term, live [permille]** | cadence compensation [permille] |
| 37..38 | u16 | **rider aggression, live [permille]** | pre-compensation motor power [W] |
| 39..40 | u16 | `u_abs`, peak | same |
| 41..42 | u16 | pack voltage [mV] | same |
| 43 | u8 | cadence, live [rpm] | same |
| 44 | u8 | **limiter and lifecycle bits**, see below | cadence-compensation enabled |
| 45 | u8 | **PAS lifecycle state** (`ap2_pas_state_t`) | Extended Boost state bits |
| 46..47 | u16 | **terrain load, live [permille]** | Extended Boost peak load [centikg] |
| 48..49 | u16 | **AUTO factor, live [permille]** — 0 calm, 1000 strong | Extended Boost Iq |
| 50..51 | u16 | **assist response, live [permille]** | Extended Boost time left [ms] |
| 52 | u8 | **profile in force** (`ap2_profile_id_t`) | Extended Boost cancel reason |
| 53..54 | u16 | calibrated pedal load, live [centikg] | same |
| 55..56 | u16 | **normalized effort, live [permille]** | assist torque ×160 |
| 57..58 | u16 | **attack time in force [ms]** | Iq launch anchor |
| 59..60 | u16 | **release time in force [ms]** | Iq measured-duty anchor |
| 61..62 | u16 | **power ceiling in force [W]** | launch blend [permille] |
| 63..64 | u16 | Iq request before the limiter chain, live | Iq before the level ceiling |
| 65..66 | u16 | requested motor power, live [W] | same |
| 67..68 | u16 | `u_abs`, live | same |
| 69..70 | u16 | CRC-16/CCITT-FALSE over 0..68 | same |

### Byte 5 — flags (unchanged from v2)

| Bit | Meaning |
|---|---|
| 0 | assist permitted |
| 1 | pedalling active |
| 2 | brake active |
| 3 | torque sensor fault |
| 4 | backward crank confirmed |
| 5 | torque calibration active |
| 6 | HMI communication lost |
| 7 | PWM on |

### Byte 44 — which stage of the one limiter chain was binding (v7)

| Bit | Meaning |
|---|---|
| 0 | profile power ceiling |
| 1 | battery current |
| 2 | phase / level Iq ceiling |
| 3 | undervoltage derate |
| 4 | thermal derate |
| 5 | speed / legal taper |
| 6 | START segment active |
| 7 | RELEASE active |

### Byte 52 — profile in force

`0` ECO, `1` TRAIL, `2` SPORT, `3` SPORT+, `4` AUTO, `5` AUTO SPORT+.

A level stored under a legacy mode number reports the profile it was **migrated** to, not the
stored number — this field says what the pipeline was actually running.

### What v7 deliberately does not carry

Extended Boost state, the launch/measured-duty anchors and their crossfade, and cadence
compensation. Those mechanisms were removed with the legacy pipeline. A decoder must report them
as *unavailable* on v7, never as zero: zero is a measurement, and "this controller does not have
that mechanism" is not one.

## v8 — TQ-06 G53 map

v1–v7 remain historical wire contracts. v8 retains the 71-byte envelope and CRC while replacing
retired AP2 estimator/profile fields with read-only G53/native state. All integer fields are
little-endian. Saturation happens before packing as specified below. No v8 field feeds control.

| Offset | v7 disposition | v8 meaning | Type / unit | Source and packing |
|---|---|---|---|---|
| 0..1 | retain | magic `D`,`G` | u8 × 2 | exact constants |
| 2 | replace | version = 8 | u8 | exact constant |
| 3 | retain | deprecated engine id | u8 | `DIAG_ENGINE_ID_RIDE_CORE` |
| 4 | retain | peak cadence | u8 rpm | native cadence peak, saturate u8 |
| 5 | replace | native permission/safety facts | u8 bits | see byte 5 below |
| 6..7 | replace | peak G53 D7EC rider output | u16 G53 native | `g53_port_output_t.trace.d7ec_rider`, peak, saturate u16 |
| 8..9 | retain | peak rider power | u16 W | native observation, saturate u16 |
| 10..11 | retain | peak applied support ratio | u16 % | native observation, saturate u16 |
| 12..13 | retain | peak motor power | u16 W | native electrical observation, saturate u16 |
| 14..15 | retain | peak requested battery current | u16 mA | existing motor-power/pack-voltage derivation, saturate u16 |
| 16..17 | retain | peak pre-limit Iq request | u16 Iq | telemetry `iq_request_before_limits`, peak, clamp 0..32767 |
| 18..19 | retain | peak `MS.i_q_setpoint` | u16 Iq | native final-Iq observation, peak, clamp 0..32767 |
| 20..21 | retain | speed | u16 0.01 km/h | `MS.Speedx100`, saturate u16 |
| 22..23 | retain | PAS idle | u16 ms | `pas_idle_ticks / 4`, saturate u16 |
| 24..25 | replace | live G53 D7EC rider output | u16 G53 native | `trace.d7ec_rider`, saturate u16 |
| 26..27 | replace | final allowed Iq before final slew | i16 Iq | telemetry `final_iq_request`, saturate signed 16-bit |
| 28..29 | retain | live `MS.i_q_setpoint` | i16 Iq | native final-Iq writer, saturate signed 16-bit |
| 30..31 | replace | live D7EC envelope | u16 G53 native | `trace.d7ec_envelope`, saturate u16 |
| 32..33 | retain | measured `MS.i_q` | i16 Iq | native measurement, saturate signed 16-bit |
| 34 | retain | battery-current limiter | u8 bits | bit0 active; bits1..7 zero |
| 35..36 | replace | live D7EC acceleration output | u16 G53 native | `trace.d7ec_accel`, saturate u16 |
| 37..38 | replace | live E1E8 output | u16 G53 native | `trace.e1e8_output`, saturate u16 |
| 39..40 | retain | peak `u_abs` | u16 native | diagnostic peak, saturate u16 |
| 41..42 | retain | pack voltage | u16 mV | `MS.Voltage`, saturate u16 |
| 43 | retain | live cadence | u8 rpm | `rider_input_t.cadence_rpm`, saturate u8 |
| 44 | replace | limiter, zero, and G53 permission facts | u8 bits | see byte 44 below |
| 45 | replace | G53 PAS direction | i8 G53 native | `trace.pas_direction`, low 8 bits (two's complement) |
| 46..47 | replace | Boundary A conditioned x | u16 G53 native | `trace.x`, saturate u16 |
| 48..49 | replace | Boundary A-D7EC rider input | u16 G53 native | `trace.rider_input_native`, saturate u16 |
| 50..51 | replace | M2AA | u16 G53 native | `g53_port_output_t.m2aa_native`, exact u16 |
| 52 | replace | final slew mode and zero policy | packed u8 enum | see byte 52 below |
| 53..54 | retain | calibrated pedal load | u16 centikg | native torque observation, saturate u16 |
| 55..56 | replace | raw PA6 ADC | u16 ADC counts | G53 input trace, 0..4095 |
| 57..58 | replace | final-Iq release duration | u16 ticks at 16 kHz | `fast_iq_slew_current_release_ticks_16k()`, saturate u16; maximum contract 48000 |
| 59..60 | replace | active final-Iq ceiling | i16 Iq | `fast_iq_slew_current_ceiling()`, saturate signed 16-bit |
| 61 | replace | BDE8 q50 | u8 G53 native | `trace.bde8_q50`, low 8 bits |
| 62 | replace | E1E8 state | u8 G53 native | `trace.e1e8_state`, low 8 bits |
| 63..64 | retain | Iq request before limiter chain | i16 Iq | G53 compatibility seam, saturate signed 16-bit |
| 65..66 | retain | requested motor power | u16 W | native observation, saturate u16 |
| 67..68 | retain | live `u_abs` | u16 native | `MS.u_abs`, saturate u16 |
| 69..70 | retain | CRC-16/CCITT-FALSE | u16 | over bytes 0..68, little-endian |

Byte 5 is live: bit0 final assist-permission observation; bit1 native `forward_valid`;
bit2 native safety cut; bit3 torque sensor fault; bit4 direction inhibit; bit5 real stop;
bit6 service cut; bit7 PWM on.

Byte 44 is live: bit0 power limiter; bit1 battery-current limiter; bit2 phase limiter;
bit3 undervoltage limiter; bit4 thermal limiter; bit5 speed/legal limiter; bit6 limiter zeroed;
bit7 G53 `normal_permission`.

Byte 52 is live: bits0..3 `fis_mode_t` (0..6); bit4 is set for
`FIS_ZERO_POLICY_QUIET` and clear for `FIS_ZERO_POLICY_NONE`; bits5..7 are zero/reserved.

On v8, the replaced slots do not describe AUTO factor, a SPORT profile, attack/release times,
rider aggression, terrain estimator, or AP2 PAS lifecycle. A downstream decoder must recognize
version 8 explicitly and continue to reject unknown versions.

## v9 — G53 battery-current limiter and hard trip (TASK-EVD-TQ-06-G1, ADR-013)

v9 keeps bytes 0..68 of v8 with the same type and packing. Two bits change their meaning,
and three fields are appended. The CRC moves to the end. The envelope grows from 71 to 77 bytes.
A decoder must recognize version 9 explicitly and keep rejecting unknown versions.

| Offset | v8 disposition | v9 meaning | Type / unit | Source and packing |
|---|---|---|---|---|
| 2 | replace | version = 9 | u8 | exact constant |
| 34 | extend | battery-current facts | u8 bits | bit0: battery-current limiter active, meaning the G53 PI #1 `g1 < 4096` on the PEDAL path or the ap2 battery stage on Walk (`BC_limit_flag`). bit1: hard battery-overcurrent trip latched. bits2..7 zero |
| 44 | extend | limiter facts | u8 bits | as v8; bit1 now also set while the G53 PI #1 limits (`g1 < 4096`) |
| 69..70 | new | `g1` | u16 Q12 | `g53_port_output_t.trace.g1`, clamp 0..65535; 4096 = no limiting |
| 71..72 | new | limiter feedback | i16 0.01 A | `g53_port_g1_state()->pi.fb`, the exact u16 the limiter read (two's complement) |
| 73..74 | new | hard-trip count | u16 | `battery_trip_count()` since power-on, saturating |
| 75..76 | move | CRC-16/CCITT-FALSE | u16 | over bytes 0..74, little-endian |

`g1` is the multiplier the original G5300 applies to the whole BDE8 command (N4 §2.6). The feedback
is the fast-tap battery current (1/8 pole at 4 kHz). Neither field feeds control.
