/*!
    \file    main.h
    \brief   the header file of main 

   \version 2024-12-20, V3.0.1, firmware for GD32F30x
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#ifndef MAIN_H
#define MAIN_H


#include "gd32f30x.h"
#include <arm_math.h>
#include "systick.h"
#include "gd32f307c_eval.h"
#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/*
 * Power-stage lifecycle. `ui_8_PWM_ON_Flag` mirrors TIMER0 MOE/POEN: it is
 * set after MOE is enabled and cleared whenever a production path disables
 * MOE. It is not a torque-demand or FOC-write permission flag; ARMED_ZERO
 * keeps both MOE and the FOC ISR alive.
 *
 * PREPARE is the NEUTRAL_COMMIT -> FOC_RELEASE sequence. ARMED_ZERO is the
 * persistent normal no-torque state between ACTIVE runs. FAULT is terminal.
 */
#define BRIDGE_LIFECYCLE_DISABLED        0U
#define BRIDGE_LIFECYCLE_IDLE            BRIDGE_LIFECYCLE_DISABLED /* legacy name */
#define BRIDGE_LIFECYCLE_NEUTRAL_COMMIT  1U
#define BRIDGE_LIFECYCLE_MOE_ON          2U
#define BRIDGE_LIFECYCLE_NEUTRAL_DWELL   3U
#define BRIDGE_LIFECYCLE_FOC_RELEASE     4U
#define BRIDGE_LIFECYCLE_RUN             5U /* ACTIVE */
#define BRIDGE_LIFECYCLE_ARMED_ZERO      6U
#define BRIDGE_LIFECYCLE_FAULT           7U

extern uint8_t ui_8_PWM_ON_Flag;
extern uint8_t bridge_lifecycle;


/* led spark function */
void led_spark(void);
void TIMER2_IRQHandler(void);
void runPIcontrol(void);
void write_virtual_eeprom(void);
void read_virtual_eeprom(void);
/*
 * FW-110 v5: autodetect() drives the motor open-loop for >5 s. It is reachable again from
 * CAN command 0x6200, but ONLY through hall_calibration_request() below, which refuses unless
 * the standstill gate has held continuously for AUTODETECT_STANDSTILL_MS. The request is not
 * executed inside the CAN parser: it sets a flag that main()'s while(1) services, so the CAN
 * receive path never blocks for the length of the procedure. autodetect()'s body remains
 * deliberately untouched, including its own third standstill check.
 *
 * Why this was re-enabled: FW-110 v4 removed the trigger and the FW-110 v3 supervisor
 * (src/hall_calibration.c) together, which left every controller running the compiled-in
 * HALL_DEF_* table for good. Measured on a bike, the six stored angles were bit-identical to
 * those defaults, and the resulting field orientation drove Iq_actual opposite to Iq_ref in
 * about two thirds of moving samples while the battery still supplied current. A calibration
 * that cannot be run is not a safety feature.
 *
 * HONEST LIMITATION (not mitigated by this card): once this function starts, IT is the main
 * loop for the whole of its own >5 s run - nothing else in main()'s while(1) executes until it
 * returns, including the PAS/speed decode this same standstill check depends on. The guard at
 * the top of this function's own body is therefore a POINT-IN-TIME check, true at the instant
 * the motor starts turning, not a continuous one held throughout the run. Physical safety during
 * those >5 s (the rider not touching the pedals/throttle) is not verified in software here - it
 * would require rewriting this procedure itself into a non-blocking automaton, which is real
 * motor-control work outside this card's CAN-queueing scope, not something a comment can supply.
 */
void autodetect(void);
/*
 * FW-110 v5: the ONE entry point for a CAN-originated calibration request. Returns true when
 * the standstill gate accepted it (the caller answers NORMAL_ACK), false otherwise (ERROR_ACK).
 * Accepting only arms the request; main()'s while(1) runs it.
 */
bool hall_calibration_request(void);
extern uint16_t slow_loop_counter;
/* FW-127A: the REQUESTED SVPWM geometry, signed. svpwm()'s arithmetic can legitimately go
 * negative and can exceed ARR (see inc/pwm_geometry.h for the proof); storing it unsigned made
 * both cases invisible. The APPLIED geometry - the only thing the timer and every sampling
 * decision may use - is pwm_applied[] below. */
extern int32_t switchtime[3];
/* FW-127A: the APPLIED geometry. This is what was actually written to TIMER0 CH0/CH1/CH2, and
 * therefore the only geometry any sampling/reconstruction decision is allowed to describe. */
extern uint16_t pwm_applied[3];
extern uint32_t timeout;
extern uint8_t transmit_mailbox;
extern can_trasnmit_message_struct transmit_message;
extern can_receive_message_struct receive_message;
extern FlagStatus receive_flag;
extern int32_t Z_position;
extern int32_t PWM_offset;

enum state {Stop, SixStep, Regen, Running, BatteryCurrentLimit, Interpolation, PLL, IdleRun, Sensorless, OpenLoop};
enum com_mode {Hallsensor, Sensorless_openloop, Sensorless_startkick, Hallsensor_Sensorless};
enum operation {WRITE_CMD,READ_CMD,NORMAL_ACK,ERROR_ACK, LONG_START_CMD, LONG_TRANG_CMD ,LONG_END_CMD,LONG_WARNING_CMD};

/*
 * ============================================================================================
 * FOC-AW1: tracking (back-calculation) anti-windup for the D/Q current regulators.
 * ============================================================================================
 *
 * THE PROBLEM THIS SOLVES. runPIcontrol() runs two independent PI controllers and then applies
 * a shared CIRCLE limiter to their combined output vector:
 *
 *     Vq_requested = PI_control(&PI_iq)            (PI_iq.out)
 *     Vd_requested = -PI_control(&PI_id)           (-PI_id.out; note the sign)
 *     u_abs        = |(Vd_requested, Vq_requested)|
 *     if (u_abs > _U_MAX)  Vx_applied = Vx_requested * _U_MAX / u_abs   (both axes scaled)
 *     else                 Vx_applied = Vx_requested
 *
 * Each PI already has a LOCAL integral clamp (limit_i) and output clamp (limit_output), and
 * those keep working unchanged. But neither of them knows about the circle limiter. When the
 * vector is scaled down, an axis can be receiving far less voltage than its own regulator
 * asked for while its own clamps are nowhere near their limits - because the budget went to
 * the OTHER axis. The integrator then keeps charging against a demand the hardware is not
 * delivering, and the wind-up is released only when the vector comes back inside the circle.
 *
 * THE FIX. Standard back-calculation: feed the residual the limiter threw away back into the
 * integrator, so the integrator stops charging exactly at the point the output stops moving.
 *
 *     sat_error_q = Vq_requested - Vq_applied
 *     sat_error_d = Vd_requested - Vd_applied
 *     integral_error = current_error - Kaw * sat_error          (next cycle)
 *
 * WHY Kaw = 1/Kp, AND WHY THAT IS A DERIVATION AND NOT A TUNING KNOB. sat_error is a VOLTAGE;
 * current_error is a CURRENT. Kaw must convert one into the other, so it is not free. Write
 * the classic form with a tracking time constant Tt:
 *
 *     I += Ki*Ts*e - (Ts/Tt) * sat_error
 *
 * and match it against this codebase's per-tick form, where gain_i already IS Ki*Ts:
 *
 *     I += gain_i * (e - Kaw*sat_error)   =>   gain_i * Kaw = Ts/Tt
 *
 * Choosing the textbook default Tt = Ti = Kp/Ki gives Ts/Tt = Ki*Ts/Kp = gain_i/gain_p, hence
 *
 *     Kaw = (gain_i/gain_p) / gain_i = 1 / gain_p
 *
 * The result has an exact physical meaning independent of that derivation: 1/Kp maps the
 * voltage the limiter refused back to the current error that would have asked for it, so the
 * correction removes precisely the demand the hardware could not honour - no more, no less.
 * Its steady state is the property that matters: during sustained saturation dI/dt = 0 gives
 * gain_p*e = sat_error, and since Vrequested = gain_p*e + I, the integrator settles at exactly
 * V_APPLIED. It can never charge past what the bridge is actually delivering. Nothing here is
 * copied from any other firmware; both the value and its scaling come from this project's own
 * gain_p, gain_i and _U_MAX.
 *
 * FIXED POINT. gain_p lives in the PI struct as a float, so 1/gain_p is computed ONCE at init
 * (foreground, pi_aw_init()) into Q15 - aw_inv_kp_q15 = round(2^15 / gain_p). With the shipped
 * P_FACTOR_I_Q = P_FACTOR_I_D = 1.5 that is 21845, i.e. Kaw = 0.66665649 against an ideal
 * 0.66666667 (relative error 1.5e-5). The ISR then does one integer multiply and one
 * arithmetic shift - no division, no float division, no loop:
 *
 *     aw_delta = (aw_sat_error * aw_inv_kp_q15) >> 15        [current-error units]
 *
 * OVERFLOW. aw_sat_error is clamped to +/-FOC_AW_SAT_ERROR_MAX (2^12) when it is published and
 * aw_inv_kp_q15 to FOC_AW_INV_KP_Q15_MAX (2^18) when it is derived, so the product is bounded
 * by 2^30 and cannot overflow int32. The >> on a negative product is an arithmetic shift (GCC
 * on ARM; the same assumption every other fixed-point path in this firmware already makes),
 * which biases the correction by at most 1 LSB of a quantity whose full scale is _U_MAX.
 *
 * NO-SATURATION PARITY. When aw_sat_error is 0 the correction is exactly 0.0f and the
 * integrator update degenerates to (Delta - 0.0f)*gain_i, which is bit-identical to the
 * pre-FOC-AW1 Delta*gain_i for every representable Delta. Unsaturated behaviour is therefore
 * unchanged by construction, not by measurement - and the term is applied unconditionally, so
 * the ISR keeps constant timing either way.
 *
 * WHAT THIS DOES NOT TOUCH. Kp/Ki, the local integral/output clamps, max_step, the final Iq
 * slew, Ramp Up/Down, RUN smoothing, ARMED_ZERO, MOE, ADC/current reconstruction, dead-time
 * compensation, feed-forward, field weakening. In particular it adds no periodic or one-shot
 * PI reset at Iq_ref == 0: STOP-CLICK-C1 removed those deliberately and this card keeps them
 * gone. Tracking state is reset ONLY where the whole regulator is already reset and FOC is
 * inactive or going inactive - see foc_aw_tracking_reset().
 */

/* Kaw ceiling, as Q15. 2^18 == Kaw <= 8.0, far above anything a sane gain_p produces
 * (gain_p = 1.5 -> 21845), and chosen so aw_sat_error * aw_inv_kp_q15 <= 2^12 * 2^18 = 2^30. */
#define FOC_AW_INV_KP_Q15_MAX   262144L

/* Residual ceiling, in voltage-vector units where _U_MAX = 1920 is full scale. Both the
 * requested and the applied voltage are individually bounded by limit_output = _U_MAX, so a
 * physically reachable residual cannot exceed 2*_U_MAX = 3840; 4096 = 2^12 is the next power
 * of two above that and keeps the overflow bound above exact. */
#define FOC_AW_SAT_ERROR_MAX    4096L

/*
 * FOC-AW1 observability. The existing FW-117 trace already carries PI_iq/PI_id.integral_part,
 * the APPLIED MS.u_q/MS.u_d and MS.u_abs; what it could not show is what was REQUESTED, which
 * is the whole subject of this card. MotorState_t gains the requested vector and the residual
 * (below), and these two globals carry the saturation state itself. No new logger is added.
 *
 * foc_aw_saturated   1 while the circle limiter is actively cutting the vector, else 0.
 * foc_aw_sat_ticks   16 kHz cycles spent saturated since the last foc_aw_tracking_reset(),
 *                    i.e. per bridge run. Free-running (wraps); it is a counter, not a limit.
 */
extern volatile uint8_t  foc_aw_saturated;
extern volatile uint32_t foc_aw_sat_ticks;

/*
 * FOC-AW1: drop all tracking state and the saturation observability with it.
 *
 * CALL ONLY where the D/Q regulators themselves are already being reset AND the FOC ISR is
 * inactive or being made inactive in the same breath - i.e. exactly the sites STOP-CLICK-C1
 * enumerated and preserved: cold PREPARE before a fresh bridge-on, the dwell-timeout failsafe
 * on the way to IDLE, and the hall-calibration service path as it disables the bridge.
 *
 * MUST NOT be called on ordinary zero torque, on RUN <-> ARMED_ZERO transitions, or on any
 * periodic tick. A stale residual is self-correcting - it is overwritten by the next cycle of
 * the limiter that produced it - whereas a foreground writer racing the 16 kHz owner is the
 * exact defect STOP-CLICK-C1 removed.
 */
void foc_aw_tracking_reset(void);

typedef struct
{

	int32_t       	Voltage;
	uint32_t       	Speedx100;
	int32_t         i_d;
	int32_t         i_q;
	int32_t 		i_q_setpoint;
	int32_t 		i_d_setpoint;
	int32_t 		i_setpoint_abs;
	/* FW-094: i_q_setpoint_temp / i_d_setpoint_temp removed. They were the removed assist
	 * monolith's scratch registers; the ride core returns its request by value. MotorState_t is
	 * runtime only (never persisted), so dropping them changes no stored layout. */
	int32_t         u_d;
	int32_t         u_q;
	int32_t         u_abs;

	/*
	 * FOC-AW1 observability: the vector as the D/Q regulators ASKED for it, kept beside the
	 * applied one above rather than replacing it. u_d/u_q/u_abs keep their exact existing
	 * meaning - the APPLIED, post-circle-limiter vector - because assist_modes' voltage
	 * utilisation and battery_iq_cap both read u_abs and must keep seeing what the bridge
	 * really produced. Nothing in the control path reads the four fields below; they exist so
	 * a trace or a debugger can see the residual the anti-windup is acting on.
	 *
	 *   u_d_req/u_q_req    requested Vd/Vq, before the circle limiter
	 *   u_abs_req          |(u_d_req, u_q_req)|, i.e. u_abs before it was clamped to _U_MAX
	 *   u_d_sat_err/       sat_error_d / sat_error_q = requested - applied, in the PHYSICAL
	 *   u_q_sat_err        Vd/Vq domain (PI_id's own sign-flipped copy lives in its struct)
	 */
	int32_t         u_d_req;
	int32_t         u_q_req;
	int32_t         u_abs_req;
	int32_t         u_d_sat_err;
	int32_t         u_q_sat_err;
	int32_t         Battery_Current;
	int32_t			teta_obs;
	float       	distance_since_startup;
	int32_t       	sin_delay_filter;
	int32_t       	cos_delay_filter;
	int16_t 		torque_on_crank;
	uint16_t 		torque_filtered;
	uint16_t 		p_human;
	uint16_t        calories;
	uint16_t        range;
	int16_t         int_Temperature;
	int16_t 		KV_detect_flag;
	uint8_t 		hall_angle_detect_flag;
	uint8_t 		assist_level;
	uint8_t 		SOC;
	int8_t         	system_state;
	int8_t         	level_counter_global;
	FlagStatus      offroadflag;
	uint8_t         offroadtics;
	uint8_t         bank_splash_kmh; //FW-005: brief speed-field override after bank switch (10/20 km/h), 0 = off
	int8_t         	error_state;
	int8_t 			angle_est;
	uint8_t 		cadence;
	int8_t 			Obs_flag;
	int8_t 			TQfilter;
	/* FW-094: ext_boost_duration / ext_boost_strength removed. They cached the per-level
	 * Para2 bytes for the removed overrun block. FW-084 Extended Boost is per-level config
	 * inside the profile bank (assist_extended_boost_config_t) and never used these. */
	FlagStatus 		pushassist_flag;
	FlagStatus 		walk_can_request;
	FlagStatus 		light_flag;
	FlagStatus 		button_up_flag;
	FlagStatus 		button_down_flag;
	FlagStatus 		brake_active_flag;

	//--- SOC / Range (runtime, not persisted in EEPROM) ---
	float       	remaining_mah;   // coulomb counter: charge left in pack [mAh]
	float       	used_wh;         // energy drawn this trip [Wh] (signed; regen subtracts)
	float       	avg_wh_per_km;   // EMA consumption used for range [Wh/km]
	float       	soc_real;        // SOC from coulomb counting [%]
	float       	soc_display;     // SOC shown to user (filtered) [%]
	int8_t        	soc_voltage;     // SOC from IR-compensated OCV lookup [%]

}MotorState_t;

/*
 * FW-094 — ORPHANED BUT FROZEN FIELDS.
 *
 * These are still parsed from and echoed back to the Bafang Para blocks, but after the removal
 * of the pre-ride-core assist path most fields no longer make a riding decision. M560 now
 * reads TQO_threshold as assist ratio. The retained fields stay at their original offsets:
 *
 *   1. sizeof(MotorParams_t) is part of the FW-023 stored-record length check. A size change
 *      requires old-footer validation and migration before rewriting the stored record.
 *   2. The shipped app reads and writes the Para bytes they map to. Dropping the round-trip
 *      would make those fields read back as garbage.
 *
 *   decay_base        Para1[21]  - was the cadence-assist decay curve
 *   Cadence_exponent  Para1[12]  - was the cadence exponent of that curve
 *   Override_Duration Para1[37]  - was the overrun/"power drag-on" duration
 *   MagicNumber       Para1[24..25] - unread since FW-050 (offroad gesture is a fixed sequence)
 *   TS_coeff          0x62D9     - was the cadence-assist gain
 *   ramp_end          Para1[39]  - unread; no consumer has existed for several releases
 *   assist_profile    Para2[0..29]  - was the per-level speed/assist interpolation table
 *   ext_boost_*       Para2[31..41] - was the per-level overrun duration/strength
 *   TQO_threshold     Para0      - M560 assist ratio per HMI level
 *
 * Removing the other retained fields is a protocol change, not a cleanup. See section C of
 * the FW-094 audit. M560 reuses Para0 storage with a migration marker below.
 */
typedef struct
{

	uint16_t       	wheel_cirumference;
	uint16_t       	decay_base;        //orphan, see the note above
	uint16_t       	Cadence_exponent;  //orphan
	uint16_t       	Override_Duration; //orphan
	uint16_t       	MagicNumber;       //orphan
	uint16_t       	TS_coeff;          //orphan
	uint16_t       	PAS_timeout;
	uint16_t       	ramp_end;          //orphan
	uint16_t       	throttle_offset;
	uint16_t       	throttle_max;
	uint16_t       	active_profile_bank; //was torque_offset (unused); FW-005: 0 = Power bank, 1 = eMTB bank
	uint16_t       	torque_full_scale_native; //was torque_max (unused); 0 = not calibrated
	uint16_t       	gear_ratio;
	uint16_t       	phase_current_max;
	uint16_t		battery_current_max;
	int16_t       	voltage_min;
	uint16_t       	speedLimitx100;
	uint16_t       	walk_assist_speed; // raw front-chainring RPM; legacy field name kept for EEPROM/CAN
	uint8_t        	walk_assist_current; // 0-100 %, maps to Para1[36] (speed_limit_enabled in JS)
	uint16_t       	TQO_threshold[6];   //M560 assist ratio for five HMI levels
	uint8_t       	com_mode;
	int8_t       	system_voltage;
	int8_t       	max_voltage;
	int8_t       	reverse; //use field Motor Type (Para1[18]) 1 = 1, 0 = -1
	int8_t       	legalflag; //use field Coaster Brake Support

	uint8_t       	pulses_per_revolution;
	uint8_t 		assist_profile[5][6];  //orphan: five assist levels with 6 assist factors each
	uint8_t 		assist_settings[6][3]; //LIVE: 0 power, 1 speed limit, 2 M560 acceleration
	uint8_t 		ext_boost_duration[6]; //orphan
	uint8_t 		ext_boost_strength[6]; //orphan
	q31_t 			angle_correction;

	//--- Battery SOC / Range params (appended at end to keep EEPROM offsets stable) ---
	uint16_t       	battery_capacity_mah;           // expected capacity, Canable Para1[7..8]
	uint16_t       	battery_capacity_estimated_mah; // learned capacity (slow adaptation)
	uint16_t       	r_batt_mohm;                    // pack internal resistance for IR comp [mOhm]
	uint8_t       	limp_soc_limit;                 // Canable Para1[10], 0xFF = disabled
	uint8_t       	limp_soc_limit_stage2;          // Canable Para1[11], 0xFF = disabled

	//--- FW-006: profile bank storage (appended at end to keep EEPROM offsets stable) ---
	// FW-068/069: 320 B per bank (wire format v6 uses 295) so the next per-level field costs a
	// version bump, not another change of MotorParams_t — every size change here invalidates the
	// whole stored record (FW-023 length check) and resets ALL settings to defaults.
	uint16_t       	bank_store_magic;               // 0xB16B = bank_store holds valid serialized banks
	uint8_t       	bank_store[2][256];             // serialized bank blobs (wire format v8, 255 B used — full)

	//--- FW-010: global ride-feel tuning storage (appended at end) ---
	uint16_t       	tuning_store_magic;             // 0x7501 = tuning_store holds valid values
	uint8_t       	tuning_store[64];               // serialized tuning blob (FW-068: wire format v6, 32 B used)

	//--- FW-013: user torque calibration (span only; zero is always automatic) ---
	uint16_t       	torque_cal_magic;               // 0x7C41 = fields below hold a valid user calibration
	uint8_t        	torque_cal_version;             // 1
	uint8_t        	torque_cal_pad;
	uint16_t       	torque_cal_span_native;         // native span for 60.00 kg
	uint16_t       	torque_cal_crc;                 // CRC16-CCITT over version+span

	//--- FW-076: persisted Bafang wheel-diameter code (0x3203 bytes 2-3) ---
	// Reuses the FW-014 ride-engine slot, which went dead when engine selection was removed
	// in FW-030: uint16 + uint8 + uint8 in, uint16 + uint8[2] out. Same size, same order,
	// same alignment — deliberately, because ANY change to sizeof(MotorParams_t) invalidates
	// the stored record (FW-023 length check) and resets every setting the rider has.
	uint16_t       	wheel_diameter_magic;           // 0x5744 = wheel_diameter_code is valid
	uint8_t        	wheel_diameter_code[2];         // raw Bafang code, e.g. B5 01 = 27.5"

	//--- FW-018: configurable full-charge PACK voltage (100% anchor at boot) ---
	uint16_t       	soc_full_magic;                 // 0x5F01 = soc_full_pack_10mv holds a valid threshold
	uint16_t       	soc_full_pack_10mv;             // full-charge pack voltage in units of 10 mV (4587 = 45.87 V); 0 = not set
	uint16_t       	assist_levels_magic;            // 0xA560 = M560 acceleration/ratio migrated

}MotorParams_t;

typedef struct
{
	float       	gain_p;
	float       	gain_i;
	int16_t       	limit_i;
	int16_t       	limit_output;
	int16_t       	recent_value;
	int32_t       	setpoint;
	float       	integral_part;
	int16_t       	max_step;
	int32_t       	out;
	int8_t       	shift;

	/*
	 * FOC-AW1: tracking (back-calculation) anti-windup state, expressed in THIS controller's
	 * own OUTPUT domain (`out`), not in the physical Vd/Vq domain. For PI_iq the two are the
	 * same thing; for PI_id they differ by a sign, because runPIcontrol() applies
	 * Vd = -PI_id.out. That sign conversion happens once, at the single point that publishes
	 * the residual (foc_aw_publish_residual() in main.c), so the regulator itself never has to
	 * know which axis it is driving.
	 *
	 * aw_sat_error   residual left by the VECTOR limiter in cycle N, in `out` units:
	 *                out_requested - out_applied. PI_control() consumes it in cycle N+1. It
	 *                deliberately does NOT include this controller's own max_step slew or
	 *                limit_output clamp - those are the local clamps the card keeps unchanged.
	 * aw_inv_kp_q15  Q15 fixed-point 1/gain_p, derived once at init by pi_aw_init(). This IS
	 *                Kaw: the residual is a VOLTAGE and the integrator input is a CURRENT
	 *                error, and 1/Kp is exactly the conversion between them (derivation in the
	 *                FOC-AW1 block above). 0 disables tracking and restores the pre-FOC-AW1
	 *                regulator bit-for-bit.
	 */
	int32_t       	aw_sat_error;
	int32_t       	aw_inv_kp_q15;

}PI_control_t;


#endif /* MAIN_H */
