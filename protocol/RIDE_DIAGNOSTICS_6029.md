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
