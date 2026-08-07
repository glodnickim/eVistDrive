/*
 * config.h
 *
 *  Automatically created by Lishui Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "stdint.h"

// System constants, don't touch!
#define DISPLAY_TYPE_BAFANG (1<<1)							// For ASCII-Output in Debug mode);
#define DISPLAY_TYPE_DEBUG (1<<0)							// For ASCII-Output in Debug mode);
#define EXTERNAL 1
#define INTERNAL 0
//----------------------------------------------------------------------
// advanced setting, don't touch, if you don't know what you are doing!
#define _T 3750//5625
#define TRIGGER_OFFSET_ADC 10
#define TRIGGER_DEFAULT _T-TRIGGER_OFFSET_ADC
#define DYNAMIC_ADC_THRESHOLD 3750 //to be tested
#define CAL_BAT_V 17
#define CAL_BAT_I 37.0
#define CAL_BAT_I_OFFSET 2035
#define CAL_V 15LL<<8
#define CAL_I 95 //Zurückgerechnet aus Batteriestrom = Tastverhältnis * Motorstrom
#ifndef BOOTLOADER
#define BOOTLOADER 820
#endif
// BionX IGH3
//#define INDUCTANCE 12LL
//#define RESISTANCE 220LL
//#define FLUX_LINKAGE 2400LL
//#define GAMMA 13LL

// Hoverboard Motor
#define INDUCTANCE 11LL
#define RESISTANCE 120LL
#define FLUX_LINKAGE 2000LL
#define GAMMA 13LL


//#define FAST_LOOP_LOG
//#define DISABLE_DYNAMIC_ADC
//#define INDIVIDUAL_MODES
//#define SPEEDTHROTTLE
//#define PRINTDEBUG_UART
// --- Optional CAN diagnostics (one compile-time master switch) ---
// 0 = normal riding build (default): no unsolicited developer frames and no
//     on-demand diagnostic blocks 0x6017, 0x6025 or 0x6029.
// 1 = developer build: enables the 0x81F83100 stream, debug frames
//     0x00010203..0x00010206 and the three Canable diagnostic blocks above.
// Essential HMI traffic and Canable configuration (including 0x6020/0x6023
// and system/config status 0x6028) are never disabled by this switch.
// scripts/build-firmware.ps1 sets this from -Variant; the fallback keeps IDE
// and other direct compiler builds quiet.
#ifndef CAN_DIAGNOSTICS_ENABLE
#define CAN_DIAGNOSTICS_ENABLE 0
#endif
#if (CAN_DIAGNOSTICS_ENABLE != 0) && (CAN_DIAGNOSTICS_ENABLE != 1)
#error "CAN_DIAGNOSTICS_ENABLE must be 0 or 1"
#endif
#define R_TEMP_PULLUP 3500
#define SIXSTEPTHRESHOLD 10000
#define SPEED_PLL 0 //1 for using PLL, 0 for angle extrapolation
#define P_FACTOR_PLL 10
#define I_FACTOR_PLL 10

//----------------------------------------------------------------------
//Battery bar settings for Kunteng and Bafang Display
#define BATTERY_LEVEL_1 323000
#define BATTERY_LEVEL_2 329000
#define BATTERY_LEVEL_3 344000
#define BATTERY_LEVEL_4 368000
#define BATTERY_LEVEL_5 380000

//----------------------------------------------------------------------
//PI-control factor settings
#define P_FACTOR_I_Q 1.5
#define I_FACTOR_I_Q 0.01
#define P_FACTOR_I_D 1.5
#define I_FACTOR_I_D 0.01
#define P_FACTOR_SPEED 1
#define I_FACTOR_SPEED 10

//----------------------------------------------------------------------
//PAS mode settings
//#define DIRDET
#define FRAC_HIGH 30
#define FRAC_LOW 15
#define PAS_TIMEOUT 400
#define RAMP_END 1200

//---------------------------------------------------------------------
//Throttle settings
#define THROTTLE_OFFSET 920   //only default value, throttle offset is set at startup automatically
#define THROTTLE_MAX 2850
#define THROTTLE_OVERRIDE

//--------------------------------------------------------------------
//Speed settings
#define WHEEL_CIRCUMFERENCE 2218 //mm; 27.5" rim (584) + 2.4" tire (2x61) -> dia 706 * pi. HMI 0x3203 write overrides at runtime.
// FW-076: Bafang wheel-diameter code carried in 0x3203 bytes 2-3. It is METADATA for the
// tools and the display — speed and distance are computed from WHEEL_CIRCUMFERENCE alone —
// but it has to survive a power cycle, or the app shows a different wheel after every boot.
// B5 01 = 27.5", D0 01 = 29". Two raw bytes, never interpreted by the controller.
#define WHEEL_DIAMETER_CODE_0 0xB5
#define WHEEL_DIAMETER_CODE_1 0x01
#define WHEEL_DIAMETER_MAGIC  0x5744
// Accepted ranges for a 0x3203 write. A frame outside them is rejected WHOLE — a partially
// applied frame would leave the bike with one new value and one old, and nothing would say so.
#define SPEEDLIMIT_X100_MIN 1
#define SPEEDLIMIT_X100_MAX 6000
#define WHEEL_CIRCUMFERENCE_MIN 400
#define WHEEL_CIRCUMFERENCE_MAX 4000
#define GEAR_RATIO 80 //11 for BionX IGH3
#define SPEEDLIMIT 2500
#define PULSES_PER_REVOLUTION 1 //wheel revolution, Para1[20]
// Speed display stop detection + decay (was: frozen last value for 5 s after stopping).
#define SPEED_STOP_TICKS 10600      // ticks @4kHz = 2.65 s without a wheel pulse -> speed = 0. Min detectable speed = circ*1440/ticks ~= 3.0 km/h @2218mm.
#define SPEED_DECAY_MARGIN_PCT 25   // between pulses show at most the speed implied by the silence so far, but only once a pulse is >25% overdue -> steady riding never touched, braking display falls smoothly instead of freezing
#define SPEEDSOURCE EXTERNAL
#define SPEEDFILTER 1
#define SPDSHFT 0
#define LEGALFLAG 1

//---------------------------------------------------------------------
//power settings
// FW-030/dev: phase current ceiling DECOUPLED from the battery limit and fixed at 700 (dev value).
// Mid-drive: phase current > battery current gives launch torque; the battery-current limiter
// (main.c runPIcontrol) still caps actual battery current at BATTERYCURRENT_MAX. More motor heat.
#define PH_CURRENT_MAX 700
#define BATTERYCURRENT_MAX 15000
#define REVERSE -1 //1 for normal direction, -1 for reverse //use field Motor Type (Para1[18]) 1 = 1, 0 = -1
#define VOLTAGE_MIN 1320 //33V
#define SYSTEM_VOLTAGE 40// in V
#define MAX_VOLTAGE 59// in V

// Ride Core developer selector. Keep 0 for normal/Legacy firmware.
// 0 = frozen Legacy path, 1 = ride-core Power Linear path.
#define RIDE_ENGINE_DEFAULT 0

//---------------------------------------------------------------------
//Battery SOC & Range settings (coulomb counting + voltage correction)
#define BATTERY_CAPACITY_MAH 14000   // default expected capacity (mAh), overwritten by Canable "Expected Battery Capacity" (Para1[7..8])
#define R_BATT_MOHM 80               // pack internal resistance for IR compensation [mOhm]
#define I_REST_MA 500                // |battery current| below this counts as "at rest" for OCV correction [mA]
#define REST_TIME_S 30               // seconds at rest before slow OCV correction is applied
#define OCV_CORR_GAIN 0.02f          // gain pulling SOC_real towards SOC_voltage when at rest
#define SOC_DISP_GAIN 0.05f          // SOC_display low-pass gain per slow update towards SOC_real
#define SOC_DISP_MAX_STEP 1.0f       // max SOC_display change per minute [%] (anti-jump)
#define SOC_SAVE_DELTA 3             // save state to flash when |SOC change since last save| >= this [%]
#define SAVE_MIN_INTERVAL_S 600      // ...but not more often than this [s]
#define WH_PER_KM_DEFAULT 12         // fallback consumption seed [Wh/km] before real data is available
#define RANGE_LEARN_MIN_M 300        // start blending real consumption after this distance [m]
#define RANGE_EMA_ALPHA 0.05f        // EMA gain for avg_wh_per_km
#define RECHARGE_MARGIN_PCT 5        // min OCV-vs-stored SOC rise to treat restart as a recharge [%]
//FW-018: configurable full-charge PACK-voltage threshold -> anchor SOC to 100% at boot (set from Canable)
#define SOC_FULL_MAGIC       0x5F01  // MP.soc_full_magic value marking soc_full_pack_10mv valid
#define SOC_FULL_BOOT_SETTLE_S  10   // seconds of stable pack voltage after boot before the 100% anchor
#define SOC_FULL_BOOT_STABLE_MV 200  // max pack-voltage wobble allowed inside the settle window [mV]
#define SOC_FULL_PACK_MIN_MV 20000   // hard safety range for the configured threshold [mV]
#define SOC_FULL_PACK_MAX_MV 90000
#define SOC_FULL_RELEASE_FRAC 0.010f // release the 100% anchor after using 1.0% of estimated capacity
//Limp mode (motor power reduction at low SoC), Canable Para1[10] / Para1[11], 0xFF = disabled
#define LIMP_FLOOR_PCT 30            // motor power floor at 0% SoC [%]
#define LIMP_STAGE2_PCT 15           // motor power at/below stage-2 SoC threshold [%]
#define LIMP_DISABLED 0xFF

//---------------------------------------------------------------------
//Walk Assist closed-loop speed PI (holds MP.walk_assist_speed). Integer fixed-point gains; tune on bike.
#define WA_RAMP_TICKS 720   // ticks @4kHz = 180 ms: kickstart slew (caps output rise so the kick is firm, not a jerk)
#define WA_KP_NUM    3      // P gain numerator -> out_p = (err * WA_KP_NUM) >> WA_KP_SHIFT
#define WA_KP_SHIFT  4      //   3/16 ~= 0.19 i_q per 0.01km/h error: full error (~600) saturates ceiling => kick from standstill
#define WA_KI_SHIFT  11     // I gain: integral term = wa_integral >> WA_KI_SHIFT (larger = slower trim @4kHz). TUNE.
#define WA_KICK_SPEED 50    // Speedx100 < 0.5 km/h at engage = standstill -> apply kick; above -> resume without kick
#define WALK_ASSIST_CURRENT_DEFAULT 30 // % of phase_current_max stored in Para1[36]
#define WALK_ASSIST_RPM_DEFAULT     20 // raw chainring RPM stored in Para1[60..61]
#define WALK_ASSIST_RPM_MIN         20
#define WALK_ASSIST_RPM_MAX         60
// Start boost: raised current ceiling at low speed so the initial shove actually moves the bike.
// Ride test 0.0133: launch too weak at the very first moment, then runaway until the overspeed cut.
// Requested: launch x2, hold power /2. Launch is now an ABSOLUTE % of phase current (independent of the
// stored walk_assist_current, same philosophy as the level-independent pedal startup boost); hold is the
// stored walk_assist_current scaled by WA_HOLD_PCT in firmware (survives whatever HMI/Canable has saved).
#define WA_START_PCT        100 // % of phase_current_max commanded at 0 km/h (was min(200%*wa_max,60%)=60%; x2=120% clamps at 100 = full phase)
#define WA_START_FULL_SPEED 300 // 0.01 km/h: launch shove fully faded down to the hold ceiling at 3 km/h
#define WA_HOLD_PCT         50  // % of (phase_current_max*walk_assist_current) used as the PI hold ceiling: 50 = half the user setting
// Approach control: power is limited EARLIER, before the target speed is reached (no overshoot).
#define WA_FADE_BAND 250    // 0.01 km/h: power ceiling fades linearly over the last 2.5 km/h before walk_assist_speed
#define WA_NEAR_HOLD_PCT 15 // % of wa_hold still allowed AT the target (keeps the bike walking; 0 would stall+pump below target)
#define WA_OVERSPEED_MARGIN 50 // 0.01 km/h: at target+0.5 km/h output -> 0 and integrator flushed (hard anti-overshoot)
#define WA_DEADBAND 20      // 0.01 km/h: within +-0.2 km/h of target the integrator is frozen (no current pumping at the target)

//---------------------------------------------------------------------
//Minimal engage/disengage slew on commanded current (i_q) to soften the mechanical "click" (gear lash). @4kHz tick.
#define IQ_SLEW_UP    5     // max i_q rise per tick (~35 ms 0..700): gentle torque build-up on engage
#define IQ_SLEW_DOWN  10    // max i_q fall per tick (~17 ms): prompt but soft release on disengage

// LEGACY OVERRUN (inactive in Ride Core). Holds motor current for a while AFTER the rider stops
// pushing on the pedal — the old "power drag-on" behaviour. Lives in the frozen Legacy monolith
// (main.c: Overrun_strength / Overrun_counter / Overrun_flag) and is reached only from the Legacy
// assist path, which the ride core no longer uses. 0 = OFF, the shipped state; leave it there.
//
// NOT the same thing as FW-084 Extended Boost, despite this macro's name. FW-084 is a separate,
// per-level Ride Core feature with its own module (assist_extended_boost.c): it is armed by a
// confirmed pedal push, starts on the PAS-STOP edge, and re-applies the level's current ceiling.
// This block has none of that — different state sources, the counter starts at the wrong moment
// and it bypasses part of the ride core. Do not enable it to "get" Extended Boost.
#define EXTENDED_BOOST_ENABLE 0   /* legacy overrun; FW-084 Extended Boost is a different feature */

// --- Adaptive i_q ramp (#1): how fast motor current rises/falls, scaled by wheel speed + cadence ---
// 1 = adaptive (gentle at low speed, snappy at speed -> smooth transitions & start).
// 0 = fixed ramp (slow tick constants in time mode, IQ_SLEW_* in legacy mode).
#define IQ_RAMP_ADAPTIVE   1

// 1 = time-based ramp using fractional internal steps. This can reproduce multi-second
// ramps at 4kHz. 0 = legacy integer step ramp below (IQ_SLEW_*).
#define IQ_RAMP_TIME_MODE  1
#define IQ_RAMP_Q_SHIFT    8    // fractional bits for internal ramp accumulator; keep >=1

// Time-based ramp targets in 4kHz control ticks. Slow is used near standstill/low cadence, fast at speed/cadence.
// Reference figures: ramp-up about 2.3s slow / 0.3s fast, ramp-down about 1.0s slow / 0.14s fast.
// UP_SLOW 9200 (2.3s) smeared the STARTUP_BOOST kick into a 2-second crawl -> 2400 (0.6s) lets the
// pull-away kick actually be felt while still protecting the drivetrain; revert to 9200 if start feels harsh.
#define IQ_RAMP_UP_SLOW_TICKS    2400
#define IQ_RAMP_UP_FAST_TICKS    1200
#define IQ_RAMP_DOWN_SLOW_TICKS  4000
#define IQ_RAMP_DOWN_FAST_TICKS  560

// Legacy integer step ramp. Used only when IQ_RAMP_TIME_MODE=0.
#define IQ_SLEW_UP_SLOW    6    // i_q rise/tick at standstill/low cadence (was 3 - too slow to build up)
#define IQ_SLEW_UP_FAST    12   // i_q rise/tick at speed/high cadence (was 7 - snappier response)
#define IQ_SLEW_DOWN_SLOW  2    // i_q fall/tick at low speed (lower = SLOWER power fade when easing off / stopping)
#define IQ_SLEW_DOWN_FAST  5    // i_q fall/tick at speed

#define IQ_RAMP_SPEED_LO   400  // Speedx100 = 4.0 km/h (below -> SLOW)
#define IQ_RAMP_SPEED_HI   2000 // 20.0 km/h (above -> FAST)
#define IQ_RAMP_CAD_LO     20   // rpm
#define IQ_RAMP_CAD_HI     70   // rpm

// --- Smooth start (#2): attenuate assist 0->100% while pulling away from standstill (@4kHz tick) ---
// 0 = OFF (default; adaptive ramp already softens start). 1 = ON if start still feels harsh.
#define SMOOTH_START_ENABLE 0
#define START_RAMP_TICKS   1200 // ~300 ms envelope

// FW-0xx: the legacy monolith's own STARTUP_BOOST_* constants and powf()-based boost were
// removed (single source of truth). Startup boost for real riding lives in tuning_config.c /
// assist_start.c (Canable-configurable, 120-entry integer table) - see assist_start.h.

// --- Soft cut-off stopnia mocy (usuwa klik przy koncowym DISABLE po zatrzymaniu) ---
// 1 = przed wylaczeniem mostka zjedz napiecia faz do wektora neutralnego (_T/2)
// 0 = stara sciezka: natychmiastowy zapis _T/2 + DISABLE (klik)
#define SOFT_CUTOFF_ENABLE  1
// liczba cykli petli sterowania (~4 kHz) na wygaszenie do neutral; 40 ~= 10 ms
#define SOFT_CUTOFF_TICKS   40
// Test diagnostyczny kliku: bazowo 4000 ~= 1 s, 12000 ~= 3 s.
// Jesli klik przesunie sie o 2 s, jego zrodlem jest koncowe wylaczenie mostka.
// FW-093: this is the ROTOR-STOPPED fallback only. It no longer decides when zero torque
// may become Hi-Z — see POWER_STAGE_COAST_* below.
#define POWER_STAGE_STOP_TICKS 12000

// --- FW-093: DRIVE -> COAST (true Hi-Z) after the torque request disappears -------------
//
// ZERO TORQUE IS NOT COAST. With MS.i_q_setpoint == 0 the FOC current controller is still
// running: it keeps regulating the MEASURED i_q/i_d to zero, and on a turning rotor that
// regulation behaves like electrical damping (and pulls the gearbox into its last position
// at the very end -> the "klik"). Only clearing TIMER0 MOE really releases the half bridges;
// a switchtime of _T/2 on all three phases is the ZERO VOLTAGE VECTOR, i.e. a brake, not Hi-Z.
//
// UNITS. MS.i_q / MS.i_d are the Park-transformed phase currents in the SAME native EBICS
// counts as MS.i_q_setpoint — runPIcontrol() compares them directly. 1 count = CAL_I = 95 mA
// of phase current (battery mA = i_q * CAL_I * u_abs >> 11, and u_abs = 2048 is duty 1.0
// where battery current equals phase current), so PH_CURRENT_MAX = 700 counts = 66.5 A.
// FOC.c filters both with an EMA (>>3, ~0.5 ms at the 16 kHz FOC rate) before publishing them.
//
// Threshold: the binding noise source is not the ADC resolution but the hardcoded
// inserted-channel zero offsets (2012/2020/2028 in adc_config), which can be off by ~10
// counts. 20 counts = 1.9 A = 2.9 % of the ceiling: safely above that residual, and far
// below anything the rider can feel. Raising it releases the bridge sooner but leaves a
// slightly larger current to decay through the body diodes; lowering it below ~12 risks
// never satisfying the window, in which case the max-wait below takes over.
#define POWER_STAGE_COAST_CURRENT 20
// Both currents must stay inside the window this long before the bridge is released.
// 24 ticks @4 kHz = 6 ms — long enough that a single noisy sample cannot trigger it,
// short enough that the rider cannot feel the delay.
#define POWER_STAGE_COAST_STABLE_TICKS 24
// Hard ceiling on the wait. The closed-loop current decay after the setpoint reaches zero
// takes at most ~8 ms (PI max_step 15 per 16 kHz cycle over the full _U_MAX span), so this
// only ever fires if the current reading is unusable. 200 ticks @4 kHz = 50 ms: an
// unreadable current must never leave the bridge actively damping a coasting rotor.
#define POWER_STAGE_COAST_MAX_WAIT_TICKS 200
// "The rotor is turning" for the bridge-on path: age of the last Hall edge, @4 kHz.
// 1000 ticks = 250 ms, i.e. below ~0.7 erps the angle is treated as unknown and re-anchored
// from the raw Hall state. This is a statement about the MOTOR only — it never gates coast.
#define ROTOR_MOVING_HALL_AGE_TICKS 1000

// --- Torque->power upper span (#4): map(torque_on_crank, TQO_threshold, TQ_FULL_SCALE_MV, 0, current). ---
// 3300 = old (hard pressure barely reaches full assist). 2000 = pressure-linear / "naciskowe" (Bosch-like):
// firm press reaches full assist. Also = IMMEDIATE start power (mapped_torque is a cadence-free pressure FLOOR,
// the only assist before cadence builds -> no "must crank first"). Rolling: cadence term usually wins, feel kept.
#define TQ_FULL_SCALE_MV 2000

// --- Torque gate (#D): min pedal pressure [mV above the ~750 mV rest] before cadence-based assist fires. ---
// Gates on RAW torque_on_crank (level-independent) so it works the same on every assist level (S+/Boost too).
// Without it, wiggling the cranks back/forth with almost no pressure excites the motor. Pure-pressure path
// (mapped_torque) is unaffected. Higher = firmer press to engage; too high = light pedalling won't assist.
// (Was gating on torque_filtered which is TQfilter/EMA-dependent per level -> broke high levels; fixed.)
#define TQ_GATE_MIN 18
#define TQ_PRESSURE_FLOOR_START_MV (750 + TQ_GATE_MIN)

// --- Consistent engagement (#engage): forward crank steps required to arm assist (jiggle-proof). ---
// Assist engages only with REAL pressure (TQ_GATE_MIN) AND >=START_MIN_STEPS consecutive forward quadrature
// steps. Any reverse step resets the counter -> back/forth wiggle on descents/dead-spots can't engage.
// ~96 steps/rev, so 4 steps ≈ 15° of crank -> fast & repeatable. Higher = firmer/longer push to start.
// FW-068: for the RIDE-CORE path this is now only the DEFAULT. The live value is configurable from
// Canable (Dynamics -> "Crank movement to start", TUNING_START_STEPS_*), and main.c reads it through
// tuning_config_start_steps(). The constant below still applies verbatim inside the frozen Legacy
// monolith (Walk Assist + position calibration), which is why it stays here.
#define START_MIN_STEPS 4

// --- Start phase: pedalling has clearly begun, but no cadence has been MEASURED yet ---
// This does not engage assist by itself. It only stops the control path treating "no cadence
// reading yet" as "not pedalling", while the normal START_MIN_STEPS + TQ_GATE_MIN latch still
// decides whether motor power may start.
//
// FW-087: this used to be expressed by writing a fake 1 rpm into MS.cadence (via a seed-rpm
// constant, earlier 18 then 10). That value was never read by any assist calculation - every consumer
// substituted 0 for it - so it existed purely to get past two gates while pretending to be a
// measurement. It made MS.cadence mean two different things, put a fake 1 on the HMI, and made the
// whole launch protection collapse the moment anything cleared the companion flag (exactly the
// FW-086 defect). It is now an explicit boolean, and MS.cadence only ever holds real measurements.
#define START_PHASE_ENABLE 1
#define START_PHASE_STEPS 2

// FW-088: cadence the SUPPORT CURVE is evaluated at while the start phase is up (Power
// Progressive / Power Curve only). Power = torque x crank speed, so a standing start has
// ~0 W however hard the pedal is pushed, and feeding that 0 to the curve returned
// support_min_pct - the least help exactly when pulling away needs the most. The curve
// input alone uses this nominal cadence; reported rider power stays the true ~0.
// 60 rpm matches PREVIEW_CADENCE_RPM in the Canable preview, so the chart and the bike agree.
#define START_PHASE_CURVE_RPM 60

// --- Engagement HYSTERESIS (#hold): once engaged, stay engaged until pressure drops to TQ_GATE_RELEASE mV
// above rest (must be < TQ_GATE_MIN). Without it the assist unloads the pedal -> pressure falls below the
// engage threshold -> cut -> pressure rises -> re-engage... = shudder. With it, assist holds and its
// magnitude just scales down with pressure. Release near rest so soft-pedalling still relaxes it.
#define TQ_GATE_RELEASE 5

// --- Assist character (KROK 2): torque/pressure mode vs cadence mode ---
// 0 = OFF (default): legacy cadence-based assist (TS_coeff * cadence^helper * torque_filtered). Cadence-driven.
// 1 = ON: Bosch-like PRESSURE mode - assist proportional to pedal pressure (mapped_torque), only while pedalling
//     forward with real load; cadence NOT used. Fixes twitchy/irregular engage + "wiggle without pressure".
//     IMPORTANT: in this mode LOWER TQ_FULL_SCALE_MV (~1800-2200), otherwise assist is weak.
// 2 = PRESSURE mode with PER-LEVEL EXPO CURVE (VESC-style): y = x^(1+e) on the normalized pressure span.
//     Same gates/latch/ramps as mode 1; only the pressure->power SHAPE changes, separately per assist level.
//     Simulation of the curve: https://claude.ai/code/artifact/2fd06015-0b0a-40d6-bf53-2dfb3e6df175
#define ASSIST_TORQUE_MODE 0

// Per-level curve bend for ASSIST_TORQUE_MODE=2, in percent -100..+100 (0 = straight line = mode 1):
//   +% = progressive: soft on light pedalling, power comes when you PUSH (sporty; +50 ~ eMTB feel)
//   -% = degressive: strong from the first touch, flattens on hard press (city/comfort)
// Exponent mapping: E>=0 -> 1+E/33.3 (up to 4.0); E<0 -> 1/(1-E/33.3) (down to 0.25). Endpoints 0/100% fixed,
// output always rises with pressure (monotonic by construction). Applied once per level change, powf per tick.
#define ASSIST_CURVE_EXPO_L1 0   // level array 1 (Eco)
#define ASSIST_CURVE_EXPO_L2 0   // level array 2 (Tour)
#define ASSIST_CURVE_EXPO_L3 0   // level array 3 (Sport)
#define ASSIST_CURVE_EXPO_L4 0   // level array 4 (Sport+)
#define ASSIST_CURVE_EXPO_L5 0   // level array 5 (Boost)

//---------------------------------------------------------------------
//Torque sensor: fault detection (Bafang Error 25) + cyclic offset re-zero on coast (thermal drift)
#define ERR_TORQUE          25    // Bafang error 25 = torque sensor signal failure
#define TQ_FAULT_LOW_MV     300   // torque_on_crank < this = disconnect/short-to-gnd -> Error 25 (rest ~740 mV)
#define TQ_FAULT_HIGH_MV    4300  // factory value; NOTE: EBiCS scale caps ~3300 mV so high never fires here (kept for parity)
#define TQ_FAULT_TICKS      400   // ~100 ms @4kHz out-of-range before raising fault (debounce)
// FW-058: re-zero was firing on essentially every in-ride coast. One coast can move the zero by
// TQ_RECAL_MAX_STEP (20 mV ~ 0.74 kg), which is MORE than the whole assist engage threshold
// (Minimum pedal load, 18 mV ~ 0.67 kg) -> the force needed to pick assist up while rolling kept
// changing. Longer idle window + a minimum period between corrections while moving. Standstill is
// left alone: with a foot on the ground the sensor sees a genuinely unloaded rest, so that
// re-zero is the trustworthy one.
#define TQ_RECAL_IDLE_TICKS 20000 // ~5 s @4kHz of no pedalling -> coast/idle -> eligible for re-zero (was 6000 = 1.5 s)
#define TQ_RECAL_MIN_PERIOD_TICKS 240000U // ~60 s @4kHz minimum between applied corrections WHILE MOVING (standstill unrestricted)
#define TQ_RECAL_MOVING_X100 100  // >= 1.0 km/h counts as moving (MS.Speedx100 scale) -> the lockout above applies
#define TQ_RECAL_SETTLE_TICKS 2000// coast must last this long (~0.5 s) before its averaged rest is trusted
#define TQ_RECAL_BAND_MV    30    // re-zero immediately if rest within 740±this (~1.1 kg @27mV/kg) - static pedal load must stay outside
#define TQ_RECAL_MAX_STEP   5     // max offset correction per coast (mV). FW-059: was 20, which alone exceeded the
                                  // 18 mV assist engage threshold - one bad coast could redefine how hard you must
                                  // press. Thermal drift is slow, so 5 mV/correction still tracks it.
#define TQ_RECAL_STABLE_MV  10    // FW-059: max spread of the corrected signal across the sampling window; a coast
                                  // noisier than this (rough road, chain slap, foot shifting) yields no calibration
#define TQ_REACQUIRE_COASTS 3     // out-of-band rest must REPEAT consistently over this many coasts -> real drift -> re-acquire (anti-stuck)
#define TQ_REACQUIRE_TOL_MV 30    // consecutive coasts must agree within this to count as "consistent" (not a random load)
#define TQ_REACQUIRE_MAX_MV 40    // reacquire accepts only rest within this of the zero: drift (~1.5 kg) yes, a static 2+ kg load never
#define TQ_REST_RAW_MIN     300   // absolute plausible UNLOADED raw baseline window (mV, pre-normalization): re-zero only within...
#define TQ_REST_RAW_MAX     1500  // ...this window (anti-infinite-drift); outside => pedal pressed/sensor fault -> Error 25, no re-zero
#define TQ_STUCK_CENTIKG    5600  // load held at/above ~56 kg counts toward stuck-high detection (domain: 0.01 kg, scale-independent)
#define TQ_STUCK_TICKS      80000U// ~20 s @4kHz continuously above TQ_STUCK_CENTIKG -> sensor fault (real pedaling always dips between legs)
#define TQ_FAULT_HOLD_TICKS 20000U// ~5 s minimum hold of torque_fault after the cause clears (no Error 25 flicker / assist chatter)

//---------------------------------------------------------------------
//Quadrature PAS decoder (PC12=A, PD2=B), polled @4kHz. Confirmed by CAN log: forward = negative raw step.
#define PAS_DIR_SIGN -1       // sign applied to raw quadrature step so that FORWARD pedalling => +1 (from test)
#define PAS_STEPS_PER_PULSE 4 // cadence pulse every 4 quadrature transitions -> 24 pulses/rev.
/*
 * FW-086: RESOLVED - 96 quadrature transitions per crank revolution (3.75 deg each).
 * The old note here left this open, citing a reverse-engineering claim of 48 pulses/rev
 * and asking for a bench measurement before trusting it. No measurement is needed: the
 * arithmetic settles it, given that the reported cadence is correct in the field.
 *
 * Let N = transitions per revolution. Pulses per rev = N/4, and at C rpm one revolution
 * is 240000 ticks @4kHz, so ticks per pulse = 960000/(C*N). main.c publishes
 * MS.cadence = 10000/ticks = C * N/96. That equals the true C only when N = 96, and the
 * reading IS true - so N = 96 and the constant 10000 is exactly that assumption baked in.
 * (Independent check on the tick rate: SPEED_STOP_TICKS 10600 = 2.65 s -> 4000 Hz.)
 *
 * Both figures were right, counting different things: 24 magnet pole-pairs give 4*24 = 96
 * QUADRATURE TRANSITIONS, while edges on a SINGLE channel give 2*24 = 48 "pulses/rev".
 * Do NOT change this to 2 - that would halve the reported cadence.
 *
 * This also fixes the crank-angle scale used by FW-085 (1 step = 3.75 deg, 96 steps/rev),
 * so its RUN smoothing window really is the fraction of a turn its label claims.
 */
// FW-025 set this to a fixed 200 ms and ride-confirmed it as OK (the "runs on for seconds"
// symptom that prompted looking at this window turned out to be the unrelated PI windup bug,
// not this timeout). FW-0xx revisits it: at low/uneven cadence the average inter-transition gap
// (~625/rpm ms at ~96 transitions/rev) can exceed 200 ms well before a real stop, misreading a
// slow pedal stroke as "stopped" and re-arming the startup boost/seed on every recovery. Kept as
// the CANable-configurable floor (`pas_stop_ms`, evistdrive_config_schema.yaml) - same meaning as
// before at normal/fast cadence - and stretched adaptively above it only when the crank is
// genuinely turning slowly; see PAS_STOP_TICKS_MAX and pas_last_period_ticks in main.c.
#define PAS_STOP_TICKS 800    // FW-025: ticks @4kHz = 200 ms floor with no quadrature transition -> pedalling stopped
#define PAS_STOP_TICKS_MAX 2000 // FW-0xx: ticks @4kHz = 500 ms ceiling for the adaptive stretch at low cadence
                              // (was 2000 = 500 ms). Under load the cadence is HELD until this window, so assist
                              // lingered ~500 ms after you stop pedalling. Measured on 0.0194:
                              // quadrature transitions arrive every ~10-60 ms while pedalling, so 200 ms keeps a
                              // 3-6x margin against a false stop even at low cadence.
                              // Shared by both engines; also gates cadence-zero, forward_pedaling and the FW-024b
                              // reverse-flag clear (all consistent).
// FW-024: one reverse step LATCHES Backwards_counter to this value instead of netting +1. Forward steps bleed
// it down by 1 each, so a clean forward run (this many steps) is needed to clear -> reliable backward detection
// despite crank jitter during backpedalling (the old net +1 vs -1 never reached the >=4 cut threshold). Chosen
// so the backward hold clears at roughly the same forward-step count as the fwd_run re-engage (START_MIN_STEPS).
#define BACKWARD_LATCH_COUNT 8

//---------------------------------------------------------------------
// Auto-off (self power-off after inactivity) + comms watchdog (CAN loss from HMI).
// Slow loop runs every 40 ms, so all *_TICKS below are counted in 40 ms units.
#define AUTO_OFF_MINUTES 10   // default inactivity timeout [min] before self power-off (0 = disabled). Overwritten at runtime by HMI 0x6303 if HMI sends its own auto-off time.
#define COMM_CUT_TICKS 75     // 75*40ms = 3.0 s with no HMI frame -> assist forced to 0 (fail-safe: broken cable / dead HMI, motor stops pulling)
#define COMM_OFF_TICKS 250    // 250*40ms = 10 s with no HMI frame AND standstill -> self power-off (never powers off while still moving)

//---------------------------------------------------------------------
//Thermal protection (controller NTC) + Error 10 (overtemperature) signalling
#define TEMP_WARN 75       // degC: start of power derating + stage 1 (pulsed Error 10)
#define TEMP_CUTOFF 90     // degC: power -> 0 + stage 2 (solid Error 10)
#define TEMP_CLEAR 68      // degC: clear thermal state (hysteresis)
#define ERR_OVERTEMP 10    // Bafang error code 10 = motor/overtemperature
#define ERR_PULSE_ON_S 2   // stage 1: seconds the error code is reported (HMI shows it)
#define ERR_PULSE_OFF_S 6  // stage 1: seconds the error code is cleared (so HMI blinks, not too often)
#define TEMP_OFFSET_C 11   // global calibration offset added to int_Temperature at source (affects CAN, thermal, HMI). ~+11 to match original FW (26C) from raw ~15C; exact only near this temp (NTC nonlinear) - for range accuracy fix T_NTC params instead

//---------------------------------------------------------------------
//torquesensor settings
#define TS_COEF 4
#define TS_MODE
#define TQONAD1
#define TQFILTER 6

//---------------------------------------------------------------------
//Display settings
#define DISPLAY_TYPE DISPLAY_TYPE_BAFANG

//---------------------------------------------------------------------
//Regen settings

#define REGEN_CURRENT 800
#define REGEN_CURRENT_MAX 10000
//#define ADC_BRAKE

//---------------------------------------------------------------------
//Walk Assist safety settings
#define WA_BUTTON_THRESHOLD_LOW  3000
#define WA_BUTTON_THRESHOLD_HIGH 3700
#define WA_BUTTON_DEBOUNCE       20
#define WA_BUTTON_RELEASE        20    // probki poza zakresem [LOW,HIGH] by wylaczyc przycisk (anty-chatter)
#define WA_SPEED_RESUME_HYST_X100 50   // restart 0.5 km/h below the per-bank wheel cut-off

//---------------------------------------------------------------------
#define AUTODETECT 0

#endif /* CONFIG_H_ */
