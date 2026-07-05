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
#define WHEEL_CIRCUMFERENCE 2200
#define GEAR_RATIO 80 //11 for BionX IGH3
#define SPEEDLIMIT 2500
#define PULSES_PER_REVOLUTION 1 //wheel revolution, Para1[20]
#define SPEEDSOURCE EXTERNAL
#define SPEEDFILTER 1
#define SPDSHFT 0
#define LEGALFLAG 0

//---------------------------------------------------------------------
//power settings
#define PH_CURRENT_MAX (BATTERYCURRENT_MAX / CAL_I)  //ties phase ceiling to battery limit; Para1[9]
#define BATTERYCURRENT_MAX 12000
#define REVERSE -1 //1 for normal direction, -1 for reverse //use field Motor Type (Para1[18]) 1 = 1, 0 = -1
#define PUSHASSIST_CURRENT 300
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

//---------------------------------------------------------------------
//Minimal engage/disengage slew on commanded current (i_q) to soften the mechanical "click" (gear lash). @4kHz tick.
#define IQ_SLEW_UP    5     // max i_q rise per tick (~35 ms 0..700): gentle torque build-up on engage
#define IQ_SLEW_DOWN  10    // max i_q fall per tick (~17 ms): prompt but soft release on disengage

// Extended Boost ("Override"): holds motor current for a while AFTER the rider stops pushing on the pedal.
// This is what causes the motor to "drag on" / power not dropping smoothly when you stop pedalling.
// 0 = OFF (default): power follows the pedal directly (Bosch-like, smooth power-down). 1 = ON (legacy carry).
#define EXTENDED_BOOST_ENABLE 0

// --- Adaptive i_q ramp (#1): how fast motor current rises/falls, scaled by wheel speed + cadence (TSDZ2-style) ---
// 1 = adaptive (gentle at low speed, snappy at speed -> smooth transitions & start). 0 = fixed IQ_SLEW_UP/DOWN.
#define IQ_RAMP_ADAPTIVE   1
#define IQ_SLEW_UP_SLOW    6    // i_q rise/tick at standstill/low cadence (was 3 - too slow to build up)
#define IQ_SLEW_UP_FAST    12   // i_q rise/tick at speed/high cadence (was 7 - snappier response)
#define IQ_SLEW_DOWN_SLOW  4    // i_q fall/tick at low speed (softer -> power FADES, not cuts, when easing off)
#define IQ_SLEW_DOWN_FAST  8    // i_q fall/tick at speed
#define IQ_RAMP_SPEED_LO   400  // Speedx100 = 4.0 km/h (below -> SLOW)
#define IQ_RAMP_SPEED_HI   2000 // 20.0 km/h (above -> FAST)
#define IQ_RAMP_CAD_LO     20   // rpm
#define IQ_RAMP_CAD_HI     70   // rpm

// --- Smooth start (#2): attenuate assist 0->100% while pulling away from standstill (@4kHz tick) ---
// 0 = OFF (default; adaptive ramp already softens start). 1 = ON if start still feels harsh.
#define SMOOTH_START_ENABLE 0
#define START_RAMP_TICKS   1200 // ~300 ms envelope

// --- Torque->power upper span (#4): map(torque_on_crank, TQO_threshold, TQ_FULL_SCALE_MV, 0, current). ---
// 3300 = current behaviour (hard pedal pressure barely reaches full assist). Lower (~1800-2200) = more
// pressure-linear / "naciskowe" (Bosch-like): a firm press reaches full assist. Tune to taste.
#define TQ_FULL_SCALE_MV 3300

// --- Torque gate (#D): min pedal pressure [mV above the ~750 mV rest] before cadence-based assist fires. ---
// Gates on RAW torque_on_crank (level-independent) so it works the same on every assist level (S+/Boost too).
// Without it, wiggling the cranks back/forth with almost no pressure excites the motor. Pure-pressure path
// (mapped_torque) is unaffected. Higher = firmer press to engage; too high = light pedalling won't assist.
// (Was gating on torque_filtered which is TQfilter/EMA-dependent per level -> broke high levels; fixed.)
#define TQ_GATE_MIN 25

// --- Consistent engagement (#engage): forward crank steps required to arm assist (jiggle-proof). ---
// Assist engages only with REAL pressure (TQ_GATE_MIN) AND >=START_MIN_STEPS consecutive forward quadrature
// steps. Any reverse step resets the counter -> back/forth wiggle on descents/dead-spots can't engage.
// ~96 steps/rev, so 4 steps ≈ 15° of crank -> fast & repeatable. Higher = firmer/longer push to start.
#define START_MIN_STEPS 4

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
#define ASSIST_TORQUE_MODE 0

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
