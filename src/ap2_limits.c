#include "ap2_limits.h"

#include "ap2_math.h"
#include "battery_iq_cap.h"

/* See inc/ap2_limits.h for the stage order and why it is that order. */

static battery_iq_cap_output_t battery_cap_state;

void ap2_limits_reset(void)
{
	battery_iq_cap_reset(&battery_cap_state);
}

bool ap2_limits_battery_active(void)
{
	return battery_cap_state.bc_active;
}

int32_t ap2_limits_iq_for_battery_current(int32_t battery_current_ma, int32_t u_abs,
	int32_t cal_i, int32_t phase_current_max)
{
	int64_t numerator;
	int64_t denominator;
	int64_t iq;

	if (phase_current_max < 1) {
		phase_current_max = 1;
	}
	if (cal_i <= 0 || u_abs < AP2_POWER_MIN_U_ABS || battery_current_ma <= 0) {
		return phase_current_max;
	}
	numerator = (int64_t)battery_current_ma * (int64_t)AP2_U_ABS_FULL_SCALE;
	denominator = (int64_t)cal_i * (int64_t)u_abs;
	iq = numerator / denominator;
	if (iq > phase_current_max) {
		iq = phase_current_max;
	}
	if (iq < 0) {
		iq = 0;
	}
	return (int32_t)iq;
}

/*
 * POWER -> Iq at the measured duty.
 *
 *     P_W  = V_mV * I_mA / 1e6
 *     I_mA = Iq * cal_i * u_abs / 2048
 *  => Iq   = P_W * 2048 * 1e6 / (V_mV * cal_i * u_abs)
 *
 * This needs no motor constant: the duty is measured, so the conversion cannot be wrong by a
 * fixed factor the way a load-to-current map can. Its one blind spot is a duty near zero,
 * which is exactly where the battery power is negligible - handled by the AP2_POWER_MIN_U_ABS
 * guard rather than by an invented reference operating point.
 */
static int32_t power_cap_iq(uint16_t max_power_w, uint32_t battery_voltage_mv,
	int32_t u_abs, int32_t cal_i, int32_t phase_current_max)
{
	int64_t numerator;
	int64_t denominator;
	int64_t iq;

	if (phase_current_max < 1) {
		phase_current_max = 1;
	}
	if (max_power_w == 0U || battery_voltage_mv == 0U || cal_i <= 0 ||
		u_abs < AP2_POWER_MIN_U_ABS) {
		return phase_current_max;
	}
	numerator = (int64_t)max_power_w * (int64_t)AP2_U_ABS_FULL_SCALE * 1000000;
	denominator = (int64_t)battery_voltage_mv * (int64_t)cal_i * (int64_t)u_abs;
	iq = numerator / denominator;
	if (iq > phase_current_max) {
		iq = phase_current_max;
	}
	if (iq < 0) {
		iq = 0;
	}
	return (int32_t)iq;
}

void ap2_limits_apply(const ap2_limits_input_t *in, ap2_limits_output_t *out)
{
	int32_t iq;
	int32_t phase_max;
	int32_t cap;

	if (out == 0) {
		return;
	}
	if (in == 0) {
		out->final_iq = 0;
		out->power_limited = false;
		out->battery_limited = false;
		out->phase_limited = false;
		out->voltage_limited = false;
		out->thermal_limited = false;
		out->speed_limited = false;
		out->power_cap = 0;
		out->battery_cap = 0;
		out->phase_cap = 0;
		out->after_voltage = 0;
		out->after_thermal = 0;
		return;
	}

	phase_max = (in->phase_current_max > 0) ? in->phase_current_max : 1;
	iq = in->iq_request;
	if (iq < 0) {
		iq = 0;
	}

	out->power_limited = false;
	out->battery_limited = false;
	out->phase_limited = false;
	out->voltage_limited = false;
	out->thermal_limited = false;
	out->speed_limited = false;

	/* ---- 1. POWER ------------------------------------------------------------------- */
	cap = power_cap_iq(in->max_power_w, in->battery_voltage_mv, in->u_abs, in->cal_i, phase_max);
	out->power_cap = cap;
	if (cap < iq) {
		iq = cap;
		out->power_limited = true;
	}

	/* ---- 2. BATTERY CURRENT ---------------------------------------------------------- */
	battery_iq_cap_update(in->battery_current_ma, in->battery_current_max, phase_max,
		iq, in->u_abs, in->cal_i, &battery_cap_state);
	out->battery_cap = battery_cap_state.iq_battery_cap;
	if (battery_cap_state.iq_battery_cap < iq) {
		iq = battery_cap_state.iq_battery_cap;
		out->battery_limited = true;
	}

	/* ---- 3. PHASE / Iq CEILING -------------------------------------------------------
	 * The level ceiling (what the rider configured for this assist level) and the hardware
	 * ceiling are the same kind of fact, so they are one stage.
	 */
	cap = phase_max;
	if (in->level_iq_limit > 0 && in->level_iq_limit < cap) {
		cap = in->level_iq_limit;
	}
	out->phase_cap = cap;
	if (cap < iq) {
		iq = cap;
		out->phase_limited = true;
	}

	/* ---- 4. UNDERVOLTAGE DERATE ------------------------------------------------------ */
	{
		int32_t derated = ap2_map((int32_t)in->voltage_raw,
			(int32_t)in->voltage_min_raw,
			(int32_t)in->voltage_min_raw + AP2_UNDERVOLTAGE_SPAN_RAW,
			0, iq);
		if (derated < iq) {
			out->voltage_limited = true;
		}
		iq = derated;
	}
	out->after_voltage = iq;

	/* ---- 5. THERMAL DERATE ----------------------------------------------------------- */
	{
		int32_t derated = ap2_map((int32_t)in->controller_temperature_c,
			AP2_THERMAL_DERATE_START_C, AP2_THERMAL_DERATE_END_C, iq, 0);
		if (derated < iq) {
			out->thermal_limited = true;
		}
		iq = derated;
	}
	out->after_thermal = iq;

	/* ---- 6. SPEED / LEGAL TAPER ------------------------------------------------------
	 * LAST on purpose: nothing after it may raise what it took away. Walk Assist is exempt
	 * here because its own module owns a wheel-speed cut-off that is stricter and is part of
	 * the walk contract, not of the legal pedal-assist rule.
	 */
	if (in->legal_enabled && !in->offroad && in->source != AP2_LIMIT_SOURCE_WALK) {
		int32_t derated;
		if (in->source == AP2_LIMIT_SOURCE_PEDAL) {
			derated = ap2_map((int32_t)in->speed_x100,
				(int32_t)in->speed_limit_x100,
				(int32_t)in->speed_limit_x100 + AP2_SPEED_TAPER_SPAN_X100,
				iq, 0);
		} else {
			derated = ap2_map((int32_t)in->speed_x100,
				AP2_NON_PEDAL_SPEED_LO_X100, AP2_NON_PEDAL_SPEED_HI_X100, iq, 0);
		}
		if (derated < iq) {
			out->speed_limited = true;
		}
		iq = derated;
	}

	if (iq < 0) {
		iq = 0;
	}
	out->final_iq = iq;
}
