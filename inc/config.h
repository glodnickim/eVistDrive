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
// FW-127C: DYNAMIC_ADC_THRESHOLD was REMOVED. It sat at _T, so the branch it guarded was
// reachable only where the geometry had already exceeded ARR - and there it computed a CH3
// that could never match. The sampling window is now derived from timing (see
// inc/sample_window.h) instead of gated by a magic duty threshold.
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
// .\build_firmware.ps1 (repo root) sets this via -CanDiagnostics; the fallback keeps IDE
// and other direct compiler builds quiet.
#ifndef CAN_DIAGNOSTICS_ENABLE
#define CAN_DIAGNOSTICS_ENABLE 0
#endif
#if (CAN_DIAGNOSTICS_ENABLE != 0) && (CAN_DIAGNOSTICS_ENABLE != 1)
#error "CAN_DIAGNOSTICS_ENABLE must be 0 or 1"
#endif

// --- Continuous Level-4 live-ride telemetry (0x10400..0x10407) ---
// Default: follow CAN_DIAGNOSTICS_ENABLE. Normal riding builds stay completely silent; a
// developer/diagnostic build publishes a best-effort ~48 Hz coherent snapshot for CANable.
// It may be overridden independently at compile time (for example, diagnostics on but live
// telemetry off while doing a one-shot dump). The stream never uses the critical CAN FIFO.
#ifndef CAN_RIDE_TELEMETRY_ENABLE
#define CAN_RIDE_TELEMETRY_ENABLE CAN_DIAGNOSTICS_ENABLE
#endif
#if (CAN_RIDE_TELEMETRY_ENABLE != 0) && (CAN_RIDE_TELEMETRY_ENABLE != 1)
#error "CAN_RIDE_TELEMETRY_ENABLE must be 0 or 1"
#endif
#if (CAN_RIDE_TELEMETRY_ENABLE != 0) && (CAN_DIAGNOSTICS_ENABLE == 0)
#error "CAN_RIDE_TELEMETRY_ENABLE requires CAN_DIAGNOSTICS_ENABLE=1"
#endif
// --- Optional standalone torque-sensor CAN emulation stream (0x81F83100) ---
// FW-110: this used to be silently tied to CAN_DIAGNOSTICS_ENABLE even though nothing in this
// firmware reads the frame back - it exists only for an external bus logger/tool that wants to
// see torque-sensor-shaped traffic. Measured at ~92 frames/s on a busy bus, which is real load on
// top of everything else. Independent flag, default OFF in every build including
// CAN_DIAGNOSTICS_ENABLE=1 (the PAS/diagnostics-recorder variant does not want this flood by
// default). If ever turned on, the call site is its OWN separate, best-effort, single-attempt
// path - deliberately NOT can_tx_queue: that queue's 16 slots are reserved for frames that must
// be delivered, and this stream would compete with them for the same slots purely by existing.
// Gated so an attempt is not even made unless can_tx_queue is empty and the multiframe producer
// is idle (both draw from the same physical CAN mailboxes this stream also uses), and always
// serviced AFTER them in the main loop - never earlier. See src/CAN_Display.c's sendCAN_3100().
#ifndef CAN_TORQUE_STREAM_ENABLE
#define CAN_TORQUE_STREAM_ENABLE 0
#endif
#if (CAN_TORQUE_STREAM_ENABLE != 0) && (CAN_TORQUE_STREAM_ENABLE != 1)
#error "CAN_TORQUE_STREAM_ENABLE must be 0 or 1"
#endif
// FW-110 v4: never combine the optional 0x3100 torque-sensor emulation stream with the
// diagnostic build. Both are best-effort/diagnostic-only paths whose coexistence in one
// firmware would need a bus-priority argument this card does not make; the 0x6029 peak-reset
// path this card validates must never share a build with the ~92 frames/s 0x3100 flood.
#if (CAN_DIAGNOSTICS_ENABLE != 0) && (CAN_TORQUE_STREAM_ENABLE != 0)
#error "CAN_DIAGNOSTICS_ENABLE=1 and CAN_TORQUE_STREAM_ENABLE=1 together are forbidden (FW-110 v4): the 0x3100 stream must not coexist with the diagnostics path it would displace."
#endif
#define R_TEMP_PULLUP 3500
#define SIXSTEPTHRESHOLD 10000
// FW-131: one canonical rotor angle (see inc/rotor_angle.h for the defect and the mechanism).
// 1 = canonical base + bumpless handover + bounded extrapolation; 0 = the legacy two-formula
// branch in main.c, byte for byte. This is commutation, so the A/B is two .bin files.
#ifndef CANONICAL_ANGLE_ENABLE
#define CANONICAL_ANGLE_ENABLE 1
#endif
#if (CANONICAL_ANGLE_ENABLE != 0) && (CANONICAL_ANGLE_ENABLE != 1)
#error "CANONICAL_ANGLE_ENABLE must be 0 (legacy two-formula angle) or 1 (FW-131 canonical angle)"
#endif
// FW-131: how long without a Hall edge counts as "the rotor has stopped, no edge is coming".
// Expressed in multiples of the filtered sector period, so it scales with speed instead of being
// a fixed time that means different things at 5 and at 50 electrical rev/s.
#define CANONICAL_ANGLE_STALL_PERIODS 4U
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
//FW-103/104: the ONE place the control-loop tick rate is named. TIMER1 drives control_time_ticks
//at this rate (see main.c) - speed, PAS diagnostics and ride_episode all derive their "ticks per
//second" from here, so a future prescaler change cannot leave a stale rate baked into one formula
//and not another. SPEED_TIMEBASE_HZ is kept as an alias so FW-103's speed code did not need a
//rename too.
#define CONTROL_TIMEBASE_HZ 4000U
#define SPEED_TIMEBASE_HZ CONTROL_TIMEBASE_HZ
// Speed display stop detection + decay (was: frozen last value for 5 s after stopping).
//FW-103: derived from SPEED_TIMEBASE_HZ, not a bare tick count - this means 2.65 s at
//whatever the timebase actually is, not "10600" regardless of it. *265U/100U == *2.65,
//done in integer math; still exactly 10600 at 4000 Hz. Min detectable speed = circ*1440/ticks ~= 3.0 km/h @2218mm.
#define SPEED_STOP_TICKS ((SPEED_TIMEBASE_HZ*265U)/100U)
#define SPEED_DECAY_MARGIN_PCT 25   // between pulses show at most the speed implied by the silence so far, but only once a pulse is >25% overdue -> steady riding never touched, braking display falls smoothly instead of freezing
//FW-103: also derived - 0.1 s of silence before the decay clamp engages, same reasoning as above.
#define SPEED_DECAY_GUARD_TICKS (SPEED_TIMEBASE_HZ/10U)
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
// FW-130: 30 -> 15 -> 25. The byte finally reaches the motor (it had no reader since FW-060), so
// the number now means something. Owner decision 2026-09-03, after the first walk test under load
// reported "no force": 25 % of PH_CURRENT_MAX is 175 Iq, which the module clamps to its absolute
// WA ceiling WA_MOTOR_IQ_ABS_MAX = 157 Iq (~15 A). So the default now sits AT that hard ceiling -
// deliberately: everything from ~23 % up resolves to the same 157, and the setting only has room
// to go DOWN from here. Raising the force further means raising WA_MOTOR_IQ_ABS_MAX, which is a
// separate safety decision, not a slider.
// A controller with an older value stored in Para1[36] keeps it until it is changed in Canable.
#define WALK_ASSIST_CURRENT_DEFAULT 25 // % of phase_current_max stored in Para1[36]
// FW-130 zamknięcie 2026-09-03: domyślny target 30 rpm został potwierdzony jazdą pod obciążeniem.
// FW-143: konfigurowalny zakres rozszerzony w dół do 10 rpm; 30 rpm pozostaje domyślne, bo jest
// sprawdzonym punktem pracy. 10..60 rpm to zakres targetu zębatki/wyjścia przekładni. 70/80 rpm
// nie są normalnymi targetami Walk i służą wyłącznie jako test przekroczenia zakresu. Przy bardzo
// małym obciążeniu governor może lekko pływać wokół niskiego targetu - nie jest to błąd bezpieczeństwa.
#define WALK_ASSIST_RPM_DEFAULT     30 // raw chainring RPM stored in Para1[60..61]
#define WALK_ASSIST_RPM_MIN         10
#define WALK_ASSIST_RPM_MAX         60

//---------------------------------------------------------------------
// FW-130: Walk Assist in the G532 character - one ramped demand limited by two soft ceilings.
//
// The FW-060..082 law was a speed PI on the motor current. Three measured consequences:
// a fixed 40 Iq ceiling (5.7 % of PH_CURRENT_MAX) that no setting could raise, a ramp that rose
// in 427 ms but fell in 1280 ms (stock G532 falls 5x FASTER than it rises, we fell 3x slower),
// and a wheel-speed limit that was an on/off switch which zeroed the current and restarted the
// session from scratch. FW-130 replaces the law: no integrator, a demand ramped at a constant
// rate, and speed held by continuously lowering the current ceiling.
//
// 1 = FW-130 governor (B), 0 = the FW-060..082 speed PI (A). Both laws are compiled from
// src/walk_speed_controller.c so an A/B ride is two .bin files, not two branches.
// Overridable from the command line so the host suite can run BOTH laws in one pass.
#ifndef WALK_GOVERNOR_ENABLE
#define WALK_GOVERNOR_ENABLE 1
#endif
#if (WALK_GOVERNOR_ENABLE != 0) && (WALK_GOVERNOR_ENABLE != 1)
#error "WALK_GOVERNOR_ENABLE must be 0 (A: FW-060..082 speed PI) or 1 (B: FW-130 governor)"
#endif

// Ramp character taken from the G532 reverse, normalised to the walk ceiling (= full scale) and
// expressed in 4 kHz control ticks. Deliberately NOT a fixed time to full current: at a lower
// ceiling the same rate reaches the target proportionally sooner, exactly as the stock does.
#define WA_RISE_FULL_SCALE_TICKS  2200  // 550 ms full-scale rise
// FW-130.1: 440 (110 ms) -> 1000 (250 ms). The stock figure is 110 ms, but the stock has a
// separate downstream slew after it and we do not: Walk Assist bypasses the 16 kHz final slew
// entirely, so this IS the motor current's own fall. At 110 ms any momentary interruption became
// a complete collapse to zero, the rotor stopped behind the freewheel, and the recovery had to
// rebuild over a 550 ms rise - felt on the bike as on/off pulsing. 250 ms is still five times
// faster than the FW-060..082 law's 1280 ms, so the anti-overshoot property is kept.
#define WA_FALL_FULL_SCALE_TICKS  1000  // 250 ms full-scale fall

// Gear-RPM governor. The band is CENTRED on the bank's own target (owner requirement
// 2026-09-03: "sterowanie ma oscylowac wokol ustawionej predkosci rpm"), and proportional so the
// feel is consistent across the supported 10..60 chainring rpm range. Full current up to target-band, zero at
// target+band, linear between. Start wide (FW-130 card SS34/43): a narrow band without an
// integrator is high gain and invites hunting. Tighten only after a ride log.
#define WA_GOV_BAND_PCT             15  // % of the target ERPS
#define WA_GOV_BAND_MIN_ERPS         4  // low-speed stability floor; narrow bands at 10 rpm are high-gain and invite hunting

// Wheel speed is the fuse, never the controlled value: it tapers the ceiling below the per-bank
// cut-off, forces an immediate zero AT the cut-off (a safety limit may clamp without a ramp),
// and only above cut-off + margin is the whole session torn down and the start re-armed.
#define WA_WHEEL_TAPER_X100        150  // 1.50 km/h of taper below the cut-off
#define WA_WHEEL_HARD_MARGIN_X100  100  // 1.00 km/h above it before the session is dropped
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

// FW-094: the old "overrun" / power-drag-on block and its EXTENDED_BOOST_ENABLE switch are
// gone with the rest of the pre-ride-core assist path. Extended Boost is FW-084
// (assist_extended_boost.c): a per-level ride-core feature, armed by a confirmed pedal push,
// starting on the PAS-STOP edge and re-applying the level's current ceiling. It shares nothing
// with the removed mechanism and needs no build switch — duration 0 turns it off per level.

// --- Adaptive i_q ramp (#1): how fast motor current rises/falls, scaled by wheel speed + cadence ---
// 1 = adaptive (gentle at low speed, snappy at speed -> smooth transitions & start).
// 0 = fixed ramp (slow tick constants in time mode, IQ_SLEW_* in step mode).
#define IQ_RAMP_ADAPTIVE   1

// 1 = time-based ramp using fractional internal steps. This can reproduce multi-second
// ramps at 4kHz. 0 = integer step ramp below (IQ_SLEW_*).
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

// Integer step ramp. Used only when IQ_RAMP_TIME_MODE=0.
#define IQ_SLEW_UP_SLOW    6    // i_q rise/tick at standstill/low cadence (was 3 - too slow to build up)
#define IQ_SLEW_UP_FAST    12   // i_q rise/tick at speed/high cadence (was 7 - snappier response)
#define IQ_SLEW_DOWN_SLOW  2    // i_q fall/tick at low speed (lower = SLOWER power fade when easing off / stopping)
#define IQ_SLEW_DOWN_FAST  5    // i_q fall/tick at speed

#define IQ_RAMP_SPEED_LO   400  // Speedx100 = 4.0 km/h (below -> SLOW)
#define IQ_RAMP_SPEED_HI   2000 // 20.0 km/h (above -> FAST)
// M820 cadence ramp range: 50 rpm = 0% FAST (full SLOW), 110 rpm = 100% FAST, linear
// interpolation in between (50..60..70..80..90..100..110 rpm -> ~0/17/33/50/67/83/100% FAST).
// Plain compile-time constants for now.
// TODO: cadence ramp range (LO/HI) belongs in the motor-specific profile — future
// EVistDrive motors may have a different usable cadence band.
#define IQ_RAMP_CAD_LO     50   // rpm
#define IQ_RAMP_CAD_HI     110  // rpm

// FW-094: the pre-ride-core smooth-start envelope (SMOOTH_START_ENABLE / START_RAMP_TICKS) and
// that path's own STARTUP_BOOST_* powf() boost are gone. Both live in the ride core now, per
// level and configurable from Canable: smooth start in assist_start.c, startup boost in
// tuning_config.c / assist_start.c (120-entry integer table) - see assist_start.h.

// --- Soft cut-off stopnia mocy (usuwa klik przy koncowym DISABLE po zatrzymaniu) ---
// 1 = przed wylaczeniem mostka zjedz napiecia faz do wektora neutralnego (_T/2)
// 0 = stara sciezka: natychmiastowy zapis _T/2 + DISABLE (klik)
#define SOFT_CUTOFF_ENABLE  1
// liczba cykli petli sterowania (~4 kHz) na wygaszenie do neutral; 40 ~= 10 ms
#define SOFT_CUTOFF_TICKS   40
// --- Opoznienie koncowego wylaczenia mostka po zatrzymaniu roweru ---
// uint16_half_rotation_counter tyka w petli sterowania (~4 kHz), wiec 4000 ~= 1 s. 4000 to
// WARTOSC PRODUKCYJNA i jedyna, ktora wolno wozic na rowerze.
//
// FW117_BRIDGE_TIMING_TEST (domyslnie 0) wydluza to opoznienie do 12000 ~= 3 s. To WYLACZNIE
// test przesuniecia momentu kliku: jesli klik przesunie sie o te same ~2 s, jego zrodlem jest
// koncowe wylaczenie mostka, a nie zanik wspomagania. Ustawienie testowe NIE jest przeznaczone
// do normalnej jazdy - trzyma stopien mocy zalaczony 3 s po zatrzymaniu, wiec zostawia mostek
// pod napieciem duzo dluzej niz potrzeba i zmienia zachowanie na postoju. Wlacza sie je
// swiadomie, przez -DFW117_BRIDGE_TIMING_TEST=1 na linii kompilacji - nigdy domyslnie.
#ifndef FW117_BRIDGE_TIMING_TEST
#define FW117_BRIDGE_TIMING_TEST 0
#endif
#if (FW117_BRIDGE_TIMING_TEST != 0) && (FW117_BRIDGE_TIMING_TEST != 1)
#error "FW117_BRIDGE_TIMING_TEST must be 0 or 1"
#endif
#if FW117_BRIDGE_TIMING_TEST
#define POWER_STAGE_STOP_TICKS 12000  // FW-117 bench test only - NOT for riding
#else
#define POWER_STAGE_STOP_TICKS 4000   // production: ~1 s
#endif
/*
 * QZERO (Quiet Zero) - A/B switch and safety bound. Mechanism and rationale: inc/quiet_zero.h.
 *
 * QUIET_ZERO_ENABLE is the whole A/B axis of this card, and it gates ONE thing: whether the
 * 16 kHz FOC ISR applies the Quiet Zero action. The zero-policy word is published either way, so
 * the two images differ only in the consumer.
 *
 *   0 = A, baseline: STOP-CLICK-C1 as it ships today. After Iq_ref reaches 0 the integrators
 *       keep regulating measured current to zero, PI_iq parks at u_q ~= BEMF and the rotor
 *       free-wheels for a long time.
 *   1 = B, Quiet Zero: on a normal rider release or a reverse/safety release the integrators are
 *       faded to exact zero over QZERO_BLEND_TICKS and held there while the reference is zero.
 *
 * QZERO_ABORT_CURRENT bounds the P-only hold. During the hold nothing but the proportional term
 * limits winding current, so measured |Iq|/|Id| at or above this ends the hold and returns the
 * full zero-current PI (see quiet_zero_tick()). Half of PH_CURRENT_MAX: comfortably inside the
 * current the drive is allowed to COMMAND on any ordinary pull, an eighth of FOC.c's own
 * hard-fault trip (PH_CURRENT_MAX<<2), and far above what a decoupled rotor spinning down
 * through the freewheel should ever draw. If a ride log shows the abort counter climbing, this
 * constant - not the fade - is the first thing to look at.
 */
#ifndef QUIET_ZERO_ENABLE
#define QUIET_ZERO_ENABLE 1
#endif
#if (QUIET_ZERO_ENABLE != 0) && (QUIET_ZERO_ENABLE != 1)
#error "QUIET_ZERO_ENABLE must be 0 (A: baseline) or 1 (B: Quiet Zero)"
#endif
#define QZERO_ABORT_CURRENT (PH_CURRENT_MAX >> 1)
/*
 * QZERO-3 note: the seeded low-speed handback has NO compile switch on purpose. src/quiet_zero.c
 * is deliberately free of config.h - every input arrives by value so the state machine runs on a
 * host - and a macro it cannot see would have been a switch that silently did nothing. The A/B is
 * build against build: 0.498 is the same firmware WITHOUT the seed.
 */

/*
 * Rotor speed below which the drive must not produce ANY q-axis current, in erps.
 *
 * Moved here from ride_control.c so the two mechanisms that depend on it cannot drift apart.
 * FW-048 owns the reason: below ~5.56 erps main.c switches the commutation angle from the
 * interpolated formula to the fixed six-step one (SIXSTEPTHRESHOLD, TIMER2 at 500 kHz, six Hall
 * events per electrical revolution; hysteresis puts the up-switch at 8.33 erps). The two formulas
 * do not agree, so the angle JUMPS - and any current still flowing jumps with it. That step, not
 * the assist fade, is the clunk historically heard exactly at standstill, which is also why
 * stretching the release ramp never removed it.
 *
 * 10 erps = 7.5 chainring rpm (erps = chainring rpm x 4/3), i.e. a deliberate margin above the
 * 5.56 switch. Two consumers, one number:
 *   - ride_control.c: FW-048 coast release forces the REFERENCE to exact zero below it;
 *   - quiet_zero.c:   QZERO ends the P-only hold below it. This one is not optional - QZERO
 *     produces current from the ABSENCE of the back-EMF-matching voltage, so a zero reference
 *     alone would no longer keep the angle-switch zone current-free. Nothing is lost: energy
 *     scales with the square of speed, so at 4-7 chainring rpm the drivetrain holds well under
 *     1 % of its cruise energy and electromagnetic braking there buys no measurable time.
 */
#define RIDE_COAST_RELEASE_ERPS 10

// FW-117.1: zakaz laczenia dwoch oddzielnych eksperymentow FW-117 w jednym obrazie. Obraz
// diagnostyczny (CAN_DIAGNOSTICS_ENABLE=1) zbiera lifecycle trace mostka wokol jednej,
// wybranej krawedzi (FW117_TRACE_TRIGGER_EVENT w inc/fw117_trace.h); FW117_BRIDGE_TIMING_TEST
// przesuwa moment koncowego wylaczenia mostka o ~2 s. Polaczone w jednym flashu daja log, w
// ktorym nie da sie odroznic, czy przesuniecie kliku pochodzi z timeoutu, czy trace po prostu
// zlapal inny moment na dluzszej osi czasu - dokladnie ten sam problem mieszania eksperymentow,
// ktory karta FW-117.1 ma usunac. Kazdy eksperyment jeździ osobno.
#if (CAN_DIAGNOSTICS_ENABLE != 0) && (FW117_BRIDGE_TIMING_TEST != 0)
#error "FW-117.1: CAN_DIAGNOSTICS_ENABLE=1 and FW117_BRIDGE_TIMING_TEST=1 together are forbidden - the bridge trace image and the 12000-tick timing test must not be combined in one build."
#endif
// FW-117: bridge start is gated solely by positive Iq + current calibration safety.
// The old BRIDGE_START_IQ_DEADZONE (=10) created a deadzone where a small legitimate positive
// current request could never start the bridge. Removed: ride_control owns the Iq decision and
// the power stage responds to any positive demand.

// FW-126 FOC START TRACE reuses the small FW-117 recorder infrastructure, but has its own
// deliberately bounded 250 Hz / 280 ms capture and is compiled only in a diagnostic image.
// NORMAL stays at 0 with no trace RAM or CAN traffic.
#ifndef FW117_TRACE_ENABLE
#define FW117_TRACE_ENABLE CAN_DIAGNOSTICS_ENABLE
#endif
#if (FW117_TRACE_ENABLE != 0) && (FW117_TRACE_ENABLE != 1)
#error "FW117_TRACE_ENABLE must be 0 or 1"
#endif

// Rolling no-assist diagnostic: enabled when diagnostics is on.
#ifndef ROLLING_NO_ASSIST_DIAG_ENABLE
#if CAN_DIAGNOSTICS_ENABLE
#define ROLLING_NO_ASSIST_DIAG_ENABLE 1
#else
#define ROLLING_NO_ASSIST_DIAG_ENABLE 0
#endif
#endif

// --- STEP 2A: START NEUTRAL DWELL lifecycle ---
// After MOE ON, hold neutral PWM for this many real PWM/ISR cycles before releasing FOC.
// 4 cycles @16kHz PWM = 250 us. Conservative first value; adjust after bench measurement.
#define START_NEUTRAL_DWELL_CYCLES  4
// Failsafe: if dwell does not complete within this many main-loop iterations, force bridge off.
#define START_DWELL_TIMEOUT_CYCLES  100
// FW-126.7: the SAME failsafe, but for the one start that is also calibrating. A calibration
// attempt is bounded by CURRENT_CAL_MAX_CYCLES dwell ISR cycles (16 kHz); the main loop runs
// faster than that ISR, so the budget here must be larger in its own currency or the failsafe
// would end the start before any attempt could ever finish. Measured on the bike (FW-126.5,
// log 2026-08-26 15:21): 100 main-loop iterations elapsed in 31 dwell ISR cycles, i.e. ~3.2
// iterations per cycle. 256 cycles therefore needs ~825; 2000 keeps ~2.4x margin over that and
// still bounds the whole thing at roughly 40 ms of neutral, zero-torque bridge.
#define START_CAL_DWELL_TIMEOUT_CYCLES  2000

// --- FW-118: Independent Phase Current Zero Calibration ---
// Runtime calibration of per-phase ADC offsets (PA2=A, PA3=B, PA5=C).
// Replaces hardcoded HW offsets with calibrated SW offsets so Clarke/Park see ~0 at zero current.
// PA0 battery current is on a separate path (bat_current_offset) — NOT affected.
//
// ADC mapping (confirmed):
//   Phase A (PA2) → ADC2 ins ch0, HW offset 2020
//   Phase B (PA3) → ADC1 ins ch0, HW offset 2028
//   Phase C (PA5) → ADC0 ins ch0, HW offset 2012
//   Battery (PA0) → ADC0 regular ch0, separate calibration (bat_current_offset)
//
// FW-125: the calibration source is adc_inserted_data_read() on each phase's OWN ADC instance —
// the exact same call the FOC ISR makes at runtime (see i16_ph1/2/3_current in
// ADC0_1_IRQHandler). Earlier (FW-118/119) it read the ADC0 REGULAR scan instead
// (adc_value[7]/[8]/[4]), which is a different silicon ADC for phases A and B (ADC2/ADC1 vs
// ADC0) with its own independent zero-current offset — measuring one and applying the result to
// the other is a domain mismatch, confirmed as the root cause of FW-122 CASE C (sector-dependent
// Iq error at low Iq). See documentation/FW-125_PHASE_CURRENT_SAME_PATH_CALIBRATION_PL.md.
//
// Because adc_inserted_data_read() already returns (raw - IOFFx) courtesy of
// adc_inserted_channel_offset_config() below, the calibration domain is now a small SIGNED
// residual centred on 0, not a ~2048 raw ADC12 code. CURRENT_HW_OFFSET_A/B/C remain in use only
// to program those IOFFx registers in adc_config() — current_cal.c no longer subtracts them a
// second time.
#define CURRENT_HW_OFFSET_A           2020
#define CURRENT_HW_OFFSET_B           2028
#define CURRENT_HW_OFFSET_C           2012

// Validation range: |residual JDR| at zero current. FW-126.6 CONFIRMED the semantics this
// number always claimed to have but never actually tested: the settled neutral-bridge reading
// is JDR -16 / -5 / +8, i.e. a residual of a few tens of LSB on top of the IOFFx trim above.
// +-300 keeps its provisional generosity and is NOT widened - it is the gate that rejects both
// the saturated dark reading (JDR ~ +1850) and the bridge-enable transient (JDR ~ +2050).
#define CURRENT_CAL_RESIDUAL_MAX_DEVIATION   300    // signed residual domain
#define CURRENT_CAL_RESIDUAL_MIN             (-CURRENT_CAL_RESIDUAL_MAX_DEVIATION)
#define CURRENT_CAL_RESIDUAL_MAX             (CURRENT_CAL_RESIDUAL_MAX_DEVIATION)

// Peak-to-peak limit across the ACCEPTED sample set. Still provisional in magnitude, but no
// longer load-bearing on its own: FW-126.6 proved a saturated amplifier is very quiet (dark
// spread 10 LSB, dump P2P 17-19), so a small P2P can never be the proof of a good calibration.
// It is the last check, after the window and the stability gate.
#define CURRENT_CAL_MAX_P2P           200

// --- FW-126.7 settling / eligibility gate --------------------------------------------------
// All three are counted in DWELL ISR CYCLES: one injected conversion per PWM period, and
// TIMER0 is centre-aligned with _T = 3750 at 120 MHz, so 120e6 / (2 * 3750) = 16 kHz, i.e.
// 62.5 us per cycle. Every number below is derived from the FW-126.5 hardware trace
// (log 2026-08-26 15:21, DIAG 0.0440), not copied from the "7 cycles" observation.
//
// Measured there, per dwell cycle index:
//     cycle 0  : JDR +2058 / +2049 / +2067   (amplifier still at its rail)
//     cycle 7  : JDR   -16 /   -5  /   +9    (settled)
//     cycle 23 : JDR   -16 /   -5  /   +8    (identical to cycle 7 within 1 LSB)
//
// STABLE_BAND - the largest cycle-to-cycle change still called steady. Settled noise measured
// ~10 LSB spread over 16 samples, so ~+-5 LSB peak; 16 gives 3x margin over that. The
// rail->midpoint ramp covers ~2000 LSB in at most 7 cycles, i.e. >=280 LSB per cycle, so this
// band rejects the ramp by more than an order of magnitude. Nothing in between was observed.
#define CURRENT_CAL_STABLE_BAND       16
// STABLE_CYCLES - consecutive in-window, steady cycles required before ANY sample is eligible.
// 8 cycles = 0.5 ms, deliberately longer than the entire measured settling (7 cycles), so a
// collection cannot begin until the transient is provably over.
#define CURRENT_CAL_STABLE_CYCLES     8
// COLLECT_SAMPLES - eligible samples averaged into the offset. With sigma ~2 LSB the standard
// error of the mean at 32 samples is ~0.35 LSB, well under the 1 LSB the offset is stored in;
// 128 (the old count) would buy nothing measurable and cost 6 ms more of dwell.
#define CURRENT_CAL_COLLECT_SAMPLES   32
// MAX_CYCLES - bounded budget for one attempt, from first dwell cycle to verdict. Nominal cost
// is 7 (settling) + 8 (stability) + 32 (collection) = 47 cycles = 2.9 ms; 256 cycles = 16 ms
// leaves 5x headroom for restarts and still cannot hang the start.
#define CURRENT_CAL_MAX_CYCLES        256

// How many neutral-dwell starts may be spent on calibration in one power cycle before the
// controller stops trying. Each attempt is itself bounded by CURRENT_CAL_MAX_CYCLES, and a new
// attempt only ever begins on a fresh legitimate start request - there is no retry loop inside
// a single start.
#define CURRENT_CAL_MAX_ATTEMPTS      3

#if (CURRENT_CAL_MAX_ATTEMPTS < 1) || (CURRENT_CAL_MAX_ATTEMPTS > 10)
#error "CURRENT_CAL_MAX_ATTEMPTS must be between 1 and 10."
#endif
#if (CURRENT_CAL_STABLE_CYCLES + CURRENT_CAL_COLLECT_SAMPLES) >= CURRENT_CAL_MAX_CYCLES
#error "CURRENT_CAL_MAX_CYCLES must exceed the nominal stability + collection cost, or no attempt can ever finish."
#endif
#if CURRENT_CAL_STABLE_BAND >= CURRENT_CAL_RESIDUAL_MAX_DEVIATION
#error "CURRENT_CAL_STABLE_BAND must be far below the residual window, or stability adds nothing."
#endif


// --- Stored torque-threshold sanity range (parser.c). ---
// Upper end of the pedal-pressure span a stored TQO_threshold may sit in. It no longer shapes
// assist — FW-094 removed the pressure map that used it — but parser.c still needs a bound to
// recognise a stale/implausible stored value and repair it.
#define TQ_FULL_SCALE_MV 2000

// --- Pedal-pressure rest offset [mV above the ~750 mV unloaded reading]. ---
// The default and repair value for a stored TQO_threshold (parser.c). The rider-facing engage
// threshold is NOT this: the ride core uses the per-level "Minimum pedal load" in kg
// (assist_modes / ride_control).
#define TQ_GATE_MIN 18
#define TQ_PRESSURE_FLOOR_START_MV (750 + TQ_GATE_MIN)

// --- Start phase: pedalling has clearly begun, but no cadence has been MEASURED yet ---
// This does not engage assist by itself. It only stops the control path treating "no cadence
// reading yet" as "not pedalling", while the normal ride latch (forward crank steps from
// tuning_config_start_steps() + the per-level kg threshold) still decides whether motor power
// may start.
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

// FW-094: TQ_GATE_RELEASE (the removed path's engage hysteresis) is gone. The ride core solves
// the same shudder with the ride latch and its hold time — see ride_control.c (FW-031/FW-032)
// and the Dynamics settings in the app.

// FW-094: ASSIST_TORQUE_MODE and its ASSIST_CURVE_EXPO_L* table are gone. They were the three
// build-time shapes of the pre-ride-core assist calculation (cadence-driven, pressure-linear,
// pressure with an expo curve). The ride core replaces all three with per-level modes the rider
// picks in the app and stores in a profile bank - see assist_mode_type_t in assist_modes.h.

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
#define TQ_RECAL_BAND_MV    30    // re-zero immediately if rest within 740±this - static pedal load must stay outside.
                                  // FW-150: on the MEASURED curve 30 mV is ~3.4 kg, not the ~1.1 kg this comment used to
                                  // claim from the disproved 27 mV/kg figure. A static pedal load below ~3.4 kg can still
                                  // be absorbed as "zero" - value left unchanged deliberately, but it is now known to be
                                  // wider in real force than intended and is a candidate for its own measurement.
#define TQ_RECAL_MAX_STEP   5     // max offset correction per coast (mV). FW-059: was 20, which alone exceeded the
                                  // 18 mV assist engage threshold - one bad coast could redefine how hard you must
                                  // press. Thermal drift is slow, so 5 mV/correction still tracks it.
#define TQ_RECAL_STABLE_MV  10    // FW-059: max spread of the corrected signal across the sampling window; a coast
                                  // noisier than this (rough road, chain slap, foot shifting) yields no calibration
#define TQ_REACQUIRE_COASTS 3     // out-of-band rest must REPEAT consistently over this many coasts -> real drift -> re-acquire (anti-stuck)
#define TQ_REACQUIRE_TOL_MV 30    // consecutive coasts must agree within this to count as "consistent" (not a random load)
#define TQ_REACQUIRE_MAX_MV 40    // reacquire accepts only rest within this of the zero. FW-150: on the measured curve
                                  // that is ~6.0 kg, not the ~1.5 kg this comment used to claim - see TQ_RECAL_BAND_MV.
#define TQ_REST_RAW_MIN     300   // absolute plausible UNLOADED raw baseline window (mV, pre-normalization): re-zero only within...
#define TQ_REST_RAW_MAX     1500  // ...this window (anti-infinite-drift); outside => pedal pressed/sensor fault -> Error 25, no re-zero
#define TQ_STUCK_CENTIKG    5600  // HUMAN/diagnostic label only: the stuck-high level as the kg table reads it
                                  // FW-151: the GATE itself is TQ_STUCK_CTRL below. This constant is retained for
                                  // diagnostics and documentation; no control decision reads it.
#define TQ_STUCK_CTRL       5600  // FW-151: stuck-high sensor fault, in the frozen CONTROL domain (CLU). Numerically
                                  // the bike-verified value, so the fault trips at the same sensor signal as before.
                                  // A re-measured kg table must never move a fault threshold - see inc/torque_input.h.
#define TQ_STUCK_TICKS      80000U// ~20 s @4kHz continuously above TQ_STUCK_CENTIKG -> sensor fault (real pedaling always dips between legs)
#define TQ_FAULT_HOLD_TICKS 20000U// ~5 s minimum hold of torque_fault after the cause clears (no Error 25 flicker / assist chatter)

//---------------------------------------------------------------------
//Quadrature PAS decoder (PC12=A, PD2=B), polled @4kHz. Confirmed by CAN log: forward = negative raw step.
#define PAS_DIR_SIGN -1       // sign applied to raw quadrature step so that FORWARD pedalling => +1 (from test)
#define PAS_STEPS_PER_PULSE 4 // cadence pulse every 4 quadrature transitions -> 24 pulses/rev.
// PRE-FW128: quadrature state transitions per crank revolution. The repo has always assumed 96
// (= 24 magnets x 4 states, i.e. 3.75 deg per transition - the figure the torque EMA comment in
// main.c quotes). It is NOT independently verified: the PAS resolution note records an open
// ambiguity between 48 and 96 transitions/rev, which is a factor of TWO on the cadence SCALE.
// That is a scale question and cannot produce jitter, so this card does not change it - but it
// is written down here rather than left implicit inside a magic number.
#define PAS_TRANSITIONS_PER_REV 96U
// rpm = steps_per_pulse * 60 * tick_rate / (transitions_per_rev * period_ticks)
//     = 4 * 60 * 4000 / 96 / ticks = 10000 / ticks   <- exactly the constant used before.
#define PAS_CADENCE_RPM_NUMERATOR \
    (((uint32_t)PAS_STEPS_PER_PULSE * 60UL * (uint32_t)CONTROL_TIMEBASE_HZ) / PAS_TRANSITIONS_PER_REV)
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
 * (Independent check on the tick rate: SPEED_TIMEBASE_HZ = 4000 Hz, the same TIMER1 this
 * quadrature decoder runs from. FW-103: SPEED_STOP_TICKS is now DERIVED from that constant,
 * so it stopped being independent corroboration the moment it stopped being its own literal.)
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
// FW-099: 8 -> 5. ONE constant, nothing else, so the ride that follows measures exactly one
// thing.
//
// THE ARITHMETIC, because it is easy to get wrong and we did. After a confirmed reversal
// bc = BACKWARD_LATCH_COUNT and fwd_run = 0. Every forward step decrements one and increments
// the other IN PARALLEL, so after N forward steps: bc = LATCH - N, fwd_run = N. Assist needs
// BOTH bc < 4 (safety_cut clears) and fwd_run >= tuning_config_start_steps() (default 4):
//
//   LATCH 8 -> N > 4 and N >= 4  ->  binding N = 5
//   LATCH 5 -> N > 1 and N >= 4  ->  binding N = 4   <- fwd_run takes over as the constraint
//
// So this saves ONE PAS step, not three. At the 24 rpm median measured at these events, and
// 96 steps per crank revolution, one step is ~26 ms. HONEST EXPECTATION: the median return
// gap moves from ~220 ms to ~190-200 ms. Anything much larger would mean Backwards_counter
// also influences the engagement path somewhere we have not traced, which is worth knowing.
//
// Below 5 there is no point until fwd_run is addressed: fwd_run >= 4 already binds here, so
// lowering further buys nothing while giving up hysteresis for free.
//
// Unchanged on purpose: the first cut (every reverse step still clears fwd_run, so torque
// goes immediately), and BACKWARD_CONFIRM_STEPS. During sustained backpedalling each
// confirmed reverse step re-sets the counter to 5, which is still >= 4, so the cut holds for
// as long as the rider keeps pedalling backwards.
#define BACKWARD_LATCH_COUNT 5

// FW-098: how many CONSECUTIVE reverse quadrature steps must be seen before the long latch
// above is applied. One step is not enough on its own.
//
// Measured on the bike (0.0304, log 14:59): the sensor is not lying. Reverse steps arrive with
// gaps of 35-285 control ticks — real movement, not contact bounce — and they form a complete
// reverse quadrature cycle. What they are is the crank rocking back a fraction of a degree in
// the dead spot at low cadence. Physiological, not a decision to backpedal.
//
// The old code could not tell the two apart: one step latched the counter to 8, which needs
// five forward steps (18.75 deg of crank) to bleed below the cut threshold, and at 16-52 rpm
// the next micro-reversal arrived first. Result: the latch stood at 8 for 12.8 % of frames and
// the cut was active for 18 of 39 seconds of genuine forward pedalling.
//
// Run lengths in that log: 46x one step, 22x two, 6x three, 6x four, 1x six. Deliberate
// backpedalling produces an unbroken run, so a threshold of 3 keeps it — it fires 7.5 deg of
// crank later than before (2 extra steps at 3.75 deg each) — while the isolated one- and
// two-step rocking no longer triggers the long penalty.
//
// SAFETY, and the reason this is not simply "ignore the first two steps": every reverse step
// still clears fwd_run in the decoder, which drops the ride latch and removes assist
// immediately. The motor cannot help during real backward movement. This constant only governs
// the LONG penalty, never whether assist is cut.
#define BACKWARD_CONFIRM_STEPS 3

//---------------------------------------------------------------------
// Auto-off (self power-off after inactivity) + comms watchdog (CAN loss from HMI).
// Slow loop runs every 40 ms, so all *_TICKS below are counted in 40 ms units.
#define AUTO_OFF_MINUTES 10   // default inactivity timeout [min] before self power-off (0 = disabled). Overwritten at runtime by HMI 0x6303 if HMI sends its own auto-off time.
#define COMM_CUT_TICKS 75     // 75*40ms = 3.0 s with no HMI frame -> assist forced to 0 (fail-safe: broken cable / dead HMI, motor stops pulling)
#define COMM_OFF_TICKS 250    // 250*40ms = 10 s with a SILENT BUS AND standstill -> self power-off (never powers off while still moving)
/*
 * FW-135: the two thresholds above answer two DIFFERENT questions and must not share one
 * counter. Assist needs a live DISPLAY (source == 3). Staying powered only needs a live BUS,
 * because a display firmware update fills the bus with frames addressed to node 3 while the
 * display itself sits in its bootloader and says nothing - and powering off there cuts the
 * display's own supply in the middle of a flash write.
 *
 * The hold below closes the remaining gap: a flash erase can be quieter than COMM_OFF_TICKS.
 * Any 0x3005, addressed or broadcast, arms it, and it then simply EXPIRES. Releasing it on the
 * first frame from the display would be a hole: the display usually sends a frame or two after
 * the announcement before it enters its bootloader. It suppresses ONLY the silence power-off -
 * the on/off button and the inactivity auto-off are untouched, so the bike still turns itself
 * off and can never be left stuck on.
 */
#define UPDATE_HOLD_TICKS 7500 // 7500*40ms = 5 min after any 0x3005 -> silence power-off suspended

//---------------------------------------------------------------------
//Thermal protection (controller NTC) + Error 10 (overtemperature) signalling
#define TEMP_WARN 75       // degC: start of power derating + stage 1 (pulsed Error 10)
#define TEMP_CUTOFF 90     // degC: power -> 0 + stage 2 (solid Error 10)
#define TEMP_CLEAR 68      // degC: clear thermal state (hysteresis)
#define ERR_OVERTEMP 10    // Bafang error code 10 = motor/overtemperature
#define ERR_PULSE_ON_S 2   // stage 1: seconds the error code is reported (HMI shows it)
#define ERR_PULSE_OFF_S 6  // stage 1: seconds the error code is cleared (so HMI blinks, not too often)
#define TEMP_OFFSET_C 0    // global calibration offset added to int_Temperature at source (affects CAN, thermal, HMI). Was +11 as a hack for the estimated Beta curve; zeroed when T_NTC switched to the exact stock M820 LUT (FW-115) - keep 0 unless hardware calibration proves otherwise

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
