#include "crank_model.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void crank_state_init(crank_state_t *state)
{
	state->crank_angle_deg = 0.0;
	state->cumulative_deg = 0.0;
	state->cadence_rpm = 0.0;
	state->step_count = 0U;
}

uint32_t crank_state_advance_tick(crank_state_t *state, double cadence_rpm)
{
	/* deg/tick = rpm * (360 deg/rev) / (60 s/min) / tick_hz = rpm * 6 / tick_hz */
	double deg_per_tick = cadence_rpm * 6.0 / CRANK_MODEL_TICK_HZ;
	state->cumulative_deg += deg_per_tick;
	state->cadence_rpm = cadence_rpm;

	double wrapped = fmod(state->cumulative_deg, 360.0);
	if (wrapped < 0.0) {
		wrapped += 360.0;
	}
	state->crank_angle_deg = wrapped;

	uint32_t new_total_steps = (uint32_t)(state->cumulative_deg / CRANK_MODEL_STEP_DEG);
	uint32_t steps_this_tick = new_total_steps - state->step_count;
	state->step_count = new_total_steps;
	return steps_this_tick;
}

uint16_t crank_torque_raw_mv(const crank_state_t *state, const crank_torque_shape_t *shape)
{
	double angle = state->crank_angle_deg - shape->phase_shift_deg;
	angle = fmod(angle, 360.0);
	if (angle < 0.0) {
		angle += 360.0;
	}

	/* Rectified half-sine per leg (period 180 deg): 0 at each leg boundary (the
	 * physical dead spot), 1 at mid-stroke. */
	double leg_progress = fmod(angle, 180.0) / 180.0; /* 0..1 within this leg */
	double envelope = sin(M_PI * leg_progress);

	/* Optional EXTRA suppression near the boundary, independent of the natural
	 * rectified-sine zero-crossing — widens/deepens the near-zero region rather
	 * than merely relying on the single point where sin() is already 0. */
	double dist_from_boundary_deg = (leg_progress <= 0.5) ?
		leg_progress * 180.0 : 180.0 - leg_progress * 180.0;
	if (shape->dead_spot_width_deg > 0.0 && dist_from_boundary_deg < shape->dead_spot_width_deg) {
		double notch = 0.5 * (1.0 + cos(M_PI * dist_from_boundary_deg / shape->dead_spot_width_deg));
		envelope *= (1.0 - (shape->dead_spot_depth_pct / 100.0) * notch);
	}

	int leg_b = (angle >= 180.0);
	double asym_scale = leg_b ?
		(1.0 - shape->asymmetry_pct / 200.0) : (1.0 + shape->asymmetry_pct / 200.0);
	double amplitude = shape->mean_native_delta * (shape->ripple_pct / 100.0) * asym_scale;

	/* envelope in [0,1] -> centred roughly on [-1,1] so the ripple swings both
	 * sides of the mean rather than only adding to it. */
	double centered = (envelope - 0.5) * 2.0;
	double delta = shape->mean_native_delta + amplitude * centered;
	if (delta < 0.0) {
		delta = 0.0;
	}

	double raw = CRANK_MODEL_TORQUE_ZERO_NATIVE + delta;
	if (raw < 0.0) {
		raw = 0.0;
	}
	if (raw > 4095.0) {
		raw = 4095.0;
	}
	return (uint16_t)(raw + 0.5);
}

uint8_t crank_pas_state(const crank_state_t *state)
{
	return (uint8_t)(state->step_count % 4U);
}

double crank_cadence_ramp(double rpm_start, double rpm_end, double duration_s, double t_s)
{
	if (duration_s <= 0.0) {
		return rpm_end;
	}
	if (t_s <= 0.0) {
		return rpm_start;
	}
	if (t_s >= duration_s) {
		return rpm_end;
	}
	double frac = t_s / duration_s;
	return rpm_start + (rpm_end - rpm_start) * frac;
}
