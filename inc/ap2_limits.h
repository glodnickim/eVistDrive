#ifndef AP2_LIMITS_H_
#define AP2_LIMITS_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * ASSIST PIPELINE V2 - THE ONE LIMITER CHAIN.
 *
 * Every demand that can reach the motor passes through this chain, in this order, and comes
 * out as final_iq_request. Pedal assist, throttle and Walk Assist all use it; there is no
 * second path and no stage that can be skipped by a mode.
 *
 *     assist request
 *        |
 *        v  POWER          the profile ceiling, in watts, converted at the measured duty
 *        v  BATTERY        the configured battery-current ceiling (battery_iq_cap.c)
 *        v  PHASE / Iq     the level ceiling and the hardware phase-current ceiling
 *        v  VOLTAGE        undervoltage derate
 *        v  THERMAL        controller-temperature derate
 *        v  SPEED          legal / configured speed taper
 *        |
 *        v
 *     final_iq_request
 *
 * WHY THIS ORDER. The first three are ABSOLUTE CAPS - they answer "how much current may flow
 * at all", each for a different physical reason, and a min() of caps is order independent.
 * The last three are DERATES - multiplicative factors that scale whatever survived. A derate
 * has to come after the caps, or it would scale a number the caps were about to discard and
 * the resulting limit would depend on which stage happened to bind. Within the derates the
 * legal speed taper is deliberately LAST, so nothing downstream can soften it.
 *
 * BUMPLESS BY CONSTRUCTION. No stage here holds an integrator, so none of them can wind up.
 * A cap that starts binding changes the TARGET, and the single final Iq owner moves the
 * current to that target over the profile release time - so entering and leaving a limit is a
 * bounded ramp, never a torque step. That is the whole reason limiting happens here, upstream
 * of the one trajectory owner, rather than at the current regulator.
 */

typedef enum {
	/* Legally confirmed pedalling: the PAS lifecycle is in FORWARD. */
	AP2_LIMIT_SOURCE_PEDAL = 0,
	/* Throttle, and anything else not backed by confirmed forward pedalling. */
	AP2_LIMIT_SOURCE_NON_PEDAL = 1,
	/* Walk Assist: its own speed rule applies, set by the walk module, not by this one. */
	AP2_LIMIT_SOURCE_WALK = 2
} ap2_limit_source_t;

/*
 * Below this duty the power conversion divides by a number that carries no information, and
 * the battery power at that duty is negligible anyway - so no power cap is applied. The phase
 * current ceiling still holds, which is the limit that actually matters at a standstill.
 */
#define AP2_POWER_MIN_U_ABS 64

/* The voltage-space-vector full scale, shared with FOC.h and battery_iq_cap.c. */
#define AP2_U_ABS_FULL_SCALE 2048

/* Undervoltage taper width, in raw ADC counts above voltage_min_raw. Unchanged from the
 * limiter this chain replaces: the value is a property of the pack and the divider. */
#define AP2_UNDERVOLTAGE_SPAN_RAW 176

/* Controller temperature derate band, degrees C. */
#define AP2_THERMAL_DERATE_START_C 75
#define AP2_THERMAL_DERATE_END_C   90

/* Speed taper width above the configured limit, in 0.01 km/h. */
#define AP2_SPEED_TAPER_SPAN_X100 200

/* The no-pedalling taper: 5.0 to 7.0 km/h. */
#define AP2_NON_PEDAL_SPEED_LO_X100 500
#define AP2_NON_PEDAL_SPEED_HI_X100 700

typedef struct {
	int32_t iq_request;           /* demand before any limit, Iq domain */
	ap2_limit_source_t source;

	/* POWER */
	uint16_t max_power_w;         /* 0 = no profile ceiling */
	uint32_t battery_voltage_mv;
	int32_t u_abs;
	int32_t cal_i;

	/* BATTERY */
	int32_t battery_current_ma;
	int32_t battery_current_max;

	/* PHASE / Iq */
	int32_t level_iq_limit;       /* the assist level ceiling, Iq domain */
	int32_t phase_current_max;    /* hardware ceiling, Iq domain */

	/* VOLTAGE */
	uint16_t voltage_raw;
	int16_t voltage_min_raw;

	/* THERMAL */
	int16_t controller_temperature_c;

	/* SPEED */
	uint32_t speed_x100;
	uint16_t speed_limit_x100;
	bool legal_enabled;
	bool offroad;
} ap2_limits_input_t;

typedef struct {
	int32_t final_iq;
	/* Which stages were binding this tick, and what each of them allowed. Observation only:
	 * nothing reads these to make a decision, they exist so a ride log can name the limit
	 * instead of leaving a tuner to guess which one took the current away. */
	bool power_limited;
	bool battery_limited;
	bool phase_limited;
	bool voltage_limited;
	bool thermal_limited;
	bool speed_limited;
	int32_t power_cap;
	int32_t battery_cap;
	int32_t phase_cap;
	int32_t after_voltage;
	int32_t after_thermal;
} ap2_limits_output_t;

void ap2_limits_reset(void);
void ap2_limits_apply(const ap2_limits_input_t *in, ap2_limits_output_t *out);

/* True while the battery-current limiter latch is held. Drives the legacy CAN diagnostic bit
 * from the same single source of truth the cap itself comes from. */
bool ap2_limits_battery_active(void);

/*
 * The shared physical conversion, exposed because both the power ceiling and the battery
 * ceiling are the same equation seen from two ends:
 *
 *     battery_current_mA = Iq * cal_i * u_abs / 2048
 *
 * Returns the Iq that would draw exactly `battery_current_ma` at this duty, or
 * `phase_current_max` when the duty carries no information.
 */
int32_t ap2_limits_iq_for_battery_current(int32_t battery_current_ma, int32_t u_abs,
	int32_t cal_i, int32_t phase_current_max);

#endif /* AP2_LIMITS_H_ */
