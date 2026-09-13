#ifndef BATTERY_IQ_CAP_H_
#define BATTERY_IQ_CAP_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * QS-3C — BATTERY CURRENT LIMITER AS AN UPSTREAM IQ CAP.
 *
 * The legacy battery-current limiter swapped PI_iq's domain: it made PI_iq a battery-
 * current regulator (feedback = battery current, setpoint = battery_current_max) instead
 * of a motor q-axis current regulator. QS-3C removes that domain switch and moves the
 * limiter UPSTREAM: it becomes an Iq-domain CAP on the allowed motor current, applied
 * before the single final Iq slew, so PI_iq always remains an Iq-domain current regulator.
 *
 * PHYSICAL MODEL (the same relationship the legacy exit condition already relied on):
 *
 *     battery_current ~ Iq * CAL_I * u_abs / 2048
 *
 * where CAL_I = 95 is the phase-current-per-unit scale (config.h) and u_abs is the
 * voltage-space-vector magnitude scaled to 2048 = 2^11 (FOC.h _U_MAX domain). Solving for
 * the Iq that would draw exactly battery_current_max:
 *
 *     Iq_max = battery_current_max * 2048 / (CAL_I * u_abs)
 *
 * This is a straightforward, physically justified conversion: it equates the DC battery
 * power drawn by a commanded Iq at a given duty/modulation with the battery-current cap.
 * It is the SAME arithmetic the legacy code used in its exit test, so no new calibration
 * is invented.
 *
 * BEHAVIOUR
 * --------
 *   - Entry: measured battery current exceeds battery_current_max
 *   - Exit:  measured battery current falls below 90% of battery_current_max (hysteresis)
 *   - While active, the module emits the Iq cap Iq_max computed above.
 *   - When inactive it emits no constraint (clamped to PH_CURRENT_MAX == no-op for the min).
 *   - The cap is a slow, continuous limiting function, NOT a hard-off. Catastrophic paths
 *     (phase overcurrent in FOC.c, 200 ms hard-cut ramp in ride_control.c) are untouched
 *     and remain independent.
 *
 * OWNERSHIP
 * ---------
 * This module ONLY produces an Iq-domain cap. It never writes PI_iq.setpoint or
 * PI_iq.recent_value. ride_control.c min-arbitrates the cap into the Iq_allowed demand
 * BEFORE the single final Iq slew (fast_iq_slew.c), so the normal runtime rule is
 * PI_iq.setpoint = MS.i_q_setpoint. The inner PI_iq controller keeps:
 *     setpoint   = MS.i_q_setpoint (Iq domain, already capped upstream)
 *     recent_value = measured motor Iq
 */

typedef struct {
	bool    bc_active;          /* limiter latched active */
	int32_t iq_battery_cap;     /* allowed Iq cap in PH_CURRENT_MAX domain */
} battery_iq_cap_output_t;

/*
 * Compute the battery-current Iq cap from one control iteration's inputs.
 *
 *   battery_current_mA  measured battery current from MS.Battery_Current (milliamperes)
 *   battery_current_max the configured maximum battery current (milliamperes)
 *   phase_current_max   the phase-current full scale (PH_CURRENT_MAX, Iq domain top)
 *   iq_ref              the pre-cap motor Iq reference (Iq_allowed demand, Iq domain)
 *   u_abs               voltage-space-vector magnitude, scaled to 2048 = 2^11
 *   cal_i               phase-current scale (CAL_I, e.g. 95)
 *
 * The caller (ride_control.c) min-arbitrates the cap into the Iq_allowed demand BEFORE the
 * single final Iq slew, e.g.:
 *
 *     if (out->iq_battery_cap < iq_ref) iq_ref = out->iq_battery_cap;
 *
 * It must NOT be applied after the final slew / at PI_iq: that would make the battery limiter
 * a second downstream command owner and break the single-final-owner rule.
 *
 * The latch (bc_active) is internal state so re-entry/exit is hysteresis-controlled and
 * bumpless.
 */
void battery_iq_cap_update(
	int32_t battery_current_mA,
	int32_t battery_current_max,
	int32_t phase_current_max,
	int32_t iq_ref,
	int32_t u_abs,
	int32_t cal_i,
	battery_iq_cap_output_t *out);

void battery_iq_cap_reset(battery_iq_cap_output_t *out);

#endif /* BATTERY_IQ_CAP_H_ */
