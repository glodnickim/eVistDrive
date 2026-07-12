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
#define BOOTLOADER 820
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
// --- Developer motor-tuning telemetry on CAN ---
// Controls the two EXTRA frames used only for developer tuning:
//   0x81F83100 (torque/cadence, every 10 ms) and 0x80010203 (FOC debug: Ibat, u_q, i_q).
// 0 = OFF (default). Factory M820 does NOT send these; EBICS flooding them at startup can stop the
//     HMI from completing its firmware/info readout. Keep 0 for normal riding.
// 1 = ON (developer only).
#define SEND_DEV_TELEMETRY 0
#if SEND_DEV_TELEMETRY
#define PRINTDEBUG_CAN
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
#define GEAR_RATIO 80 //11 for BionX IGH3
#define SPEEDLIMIT 2500
#define PULSES_PER_REVOLUTION 1 //wheel revolution, Para1[20]
// Speed display stop detection + decay (was: frozen last value for 5 s after stopping).
#define SPEED_STOP_TICKS 10600      // ticks @4kHz = 2.65 s without a wheel pulse -> speed = 0. Min detectable speed = circ*1440/ticks ~= 3.0 km/h @2218mm (TSDZ2 uses ~2.1 s / ~3.7 km/h).
#define SPEED_DECAY_MARGIN_PCT 25   // between pulses show at most the speed implied by the silence so far, but only once a pulse is >25% overdue -> steady riding never touched, braking display falls smoothly instead of freezing
#define SPEEDSOURCE EXTERNAL
#define SPEEDFILTER 1
#define SPDSHFT 0
#define LEGALFLAG 0

//---------------------------------------------------------------------
//power settings
#define PH_CURRENT_MAX (BATTERYCURRENT_MAX / CAL_I)  //ties phase ceiling to battery limit; Para1[9]
#define BATTERYCURRENT_MAX 12000
#define REVERSE -1 //1 for normal direction, -1 for reverse //use field Motor Type (Para1[18]) 1 = 1, 0 = -1
#define VOLTAGE_MIN 1320 //33V
#define SYSTEM_VOLTAGE 52// in V
#define MAX_VOLTAGE 59// in V

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
// Start boost: raised current ceiling at low speed so the initial shove actually moves the bike.
// Ride test 0.0133: launch too weak at the very first moment, then runaway until the overspeed cut.
// Requested: launch x2, hold power /2. Launch is now an ABSOLUTE % of phase current (independent of the
// stored walk_assist_current, same philosophy as the level-independent pedal startup boost); hold is the
// stored walk_assist_current scaled by WA_HOLD_PCT in firmware (survives whatever HMI/Canable has saved).
#define WA_START_PCT        100 // % of phase_current_max commanded at 0 km/h (was min(200%*wa_max,60%)=60%; x2=120% clamps at 100 = full phase)
#define WA_START_FULL_SPEED 300 // 0.01 km/h: launch shove fully faded down to the hold ceiling at 3 km/h
#define WA_HOLD_PCT         50  // % of (phase_current_max*walk_assist_current) used as the PI hold ceiling: 50 = half the user setting
// TSDZ2-style approach control: power is limited EARLIER, before the target speed is reached (no overshoot).
#define WA_FADE_BAND 250    // 0.01 km/h: power ceiling fades linearly over the last 2.5 km/h before walk_assist_speed
#define WA_NEAR_HOLD_PCT 15 // % of wa_hold still allowed AT the target (keeps the bike walking; 0 would stall+pump below target)
#define WA_OVERSPEED_MARGIN 50 // 0.01 km/h: at target+0.5 km/h output -> 0 and integrator flushed (hard anti-overshoot)
#define WA_DEADBAND 20      // 0.01 km/h: within +-0.2 km/h of target the integrator is frozen (no current pumping at the target)

//---------------------------------------------------------------------
//Minimal engage/disengage slew on commanded current (i_q) to soften the mechanical "click" (gear lash). @4kHz tick.
#define IQ_SLEW_UP    5     // max i_q rise per tick (~35 ms 0..700): gentle torque build-up on engage
#define IQ_SLEW_DOWN  10    // max i_q fall per tick (~17 ms): prompt but soft release on disengage

// Extended Boost ("Override"): holds motor current for a while AFTER the rider stops pushing on the pedal.
// This is what causes the motor to "drag on" / power not dropping smoothly when you stop pedalling.
// 0 = OFF (default): power follows the pedal directly (Bosch-like, smooth power-down). 1 = ON (legacy carry).
#define EXTENDED_BOOST_ENABLE 0

// --- Adaptive i_q ramp (#1): how fast motor current rises/falls, scaled by wheel speed + cadence (TSDZ2-style) ---
// 1 = adaptive (gentle at low speed, snappy at speed -> smooth transitions & start).
// 0 = fixed ramp (slow tick constants in time mode, IQ_SLEW_* in legacy mode).
#define IQ_RAMP_ADAPTIVE   1

// 1 = time-based ramp using fractional internal steps. This can reproduce TSDZ2-like multi-second
// ramps at 4kHz. 0 = legacy integer step ramp below (IQ_SLEW_*).
#define IQ_RAMP_TIME_MODE  1
#define IQ_RAMP_Q_SHIFT    8    // fractional bits for internal ramp accumulator; keep >=1

// Time-based ramp targets in 4kHz control ticks. Slow is used near standstill/low cadence, fast at speed/cadence.
// TSDZ2 reference: ramp-up about 2.3s slow / 0.3s fast, ramp-down about 1.0s slow / 0.14s fast.
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

// --- TSDZ2-style STARTUP BOOST: cadence-decaying MULTIPLIER on pedal pressure (ported from OSF TSDZ2
// apply_startup_boost()). The ONLY pull-away boost mechanism (STARTUP_FLOOR was removed so effects can't
// stack -> clean, attributable ride feedback). It SCALES the pressure signal (mapped_torque) by a factor
// that is maximal at cadence 0 and decays geometrically as cadence builds:
//   factor(cad)% = STARTUP_BOOST_FACTOR * (1 - CADENCE_STEP/256)^cad   (same law as TSDZ2, which precomputes
// it into a 120-entry table; here powf() computes it directly). Boost is proportional to how hard you press
// -> a strong press gives a strong-but-controlled kick that fades on its own with cadence.
// Boosted pressure is capped to full MP.phase_current_max (level-independent kick).
#define STARTUP_BOOST_ENABLE        1
#define STARTUP_BOOST_FACTOR        200  // factor[0] in %: extra pressure at cadence 0 (TSDZ "start boost torque factor"). 200 = up to +200%
#define STARTUP_BOOST_CADENCE_STEP  25   // geometric decay per RPM (step/256). Higher = boost fades faster with cadence (TSDZ "cadence step").
                                         // 25 = TSDZ-typical: ~36% left at 10 rpm, ~13% at 20 rpm, gone ~40 rpm. 50 faded so fast the kick barely outlived the first crank degrees
#define STARTUP_BOOST_MODE          0    // 0=CADENCE (always on, fades w/ cadence); 1=SPEED (only from standstill, drop >45 rpm); 2=AUTO (off when little press while rolling)
#define STARTUP_BOOST_AUTO_TQ       20   // AUTO mode only: pressure [mV above ~750 rest] below which boost drops once already moving

// --- Soft cut-off stopnia mocy (usuwa klik przy koncowym DISABLE po zatrzymaniu) ---
// 1 = przed wylaczeniem mostka zjedz napiecia faz do wektora neutralnego (_T/2)
// 0 = stara sciezka: natychmiastowy zapis _T/2 + DISABLE (klik)
#define SOFT_CUTOFF_ENABLE  1
// liczba cykli petli sterowania (~4 kHz) na wygaszenie do neutral; 40 ~= 10 ms
#define SOFT_CUTOFF_TICKS   40

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

// --- Consistent engagement (#engage): forward crank steps required to arm assist (jiggle-proof). ---
// Assist engages only with REAL pressure (TQ_GATE_MIN) AND >=START_MIN_STEPS consecutive forward quadrature
// steps. Any reverse step resets the counter -> back/forth wiggle on descents/dead-spots can't engage.
// ~96 steps/rev, so 4 steps ≈ 15° of crank -> fast & repeatable. Higher = firmer/longer push to start.
#define START_MIN_STEPS 4

// --- TSDZ2-style cadence seed: after a fresh valid forward start, publish a small temporary cadence ---
// This does not engage assist by itself. It only avoids a dead first cadence calculation while the normal
// START_MIN_STEPS + TQ_GATE_MIN latch still decides whether motor power may start.
#define START_CADENCE_SEED_ENABLE 1
#define START_CADENCE_SEED_STEPS 2
#define START_CADENCE_SEED_RPM 18   // was 10 - higher seed = more power right at start (cadence^helper term bigger from the first move); still gated by START_MIN_STEPS + pressure

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
//   +% = progressive: soft on light pedalling, power comes when you PUSH (sporty; +50 ~ TSDZ2 eMTB feel)
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
#define TQ_RECAL_IDLE_TICKS 6000  // ~1.5 s @4kHz of no pedalling -> coast/idle -> eligible for re-zero (catches in-ride coasting)
#define TQ_RECAL_SETTLE_TICKS 2000// coast must last this long (~0.5 s) before its averaged rest is trusted
#define TQ_RECAL_BAND_MV    100   // re-zero immediately if rest within 740±this (~3 Nm @32mV/Nm) - guard vs static pedal load
#define TQ_RECAL_MAX_STEP   20    // max offset correction per coast (mV) - rate limit so one coast can't jump the zero
#define TQ_REACQUIRE_COASTS 3     // out-of-band rest must REPEAT consistently over this many coasts -> real drift -> re-acquire (anti-stuck)
#define TQ_REACQUIRE_TOL_MV 30    // consecutive coasts must agree within this to count as "consistent" (not a random load)
#define TQ_REST_RAW_MIN     300   // absolute plausible UNLOADED raw baseline window (mV, pre-normalization): re-zero only within...
#define TQ_REST_RAW_MAX     1500  // ...this window (anti-infinite-drift); outside => pedal pressed/sensor fault -> Error 25, no re-zero

//---------------------------------------------------------------------
//Quadrature PAS decoder (PC12=A, PD2=B), polled @4kHz. Confirmed by CAN log: forward = negative raw step.
#define PAS_DIR_SIGN -1       // sign applied to raw quadrature step so that FORWARD pedalling => +1 (from test)
#define PAS_STEPS_PER_PULSE 4 // cadence pulse every 4 quadrature transitions. Tested OK on HMI (=true RPM) -> implies ~96 transitions/rev (24 magnets). Reverser says "48 pulses/rev"; if that means 48 transitions, this would read half - VERIFY by measuring before changing to 2.
#define PAS_STOP_TICKS 2000   // ticks @4kHz = 500 ms with no quadrature transition -> pedalling stopped (cadence=0)

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
#define WA_TIMEOUT_MS            10000
#define WA_TIMEOUT_TICKS         (WA_TIMEOUT_MS * 4)

//---------------------------------------------------------------------
#define AUTODETECT 0

#endif /* CONFIG_H_ */
