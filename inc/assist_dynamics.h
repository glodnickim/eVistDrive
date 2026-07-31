#ifndef ASSIST_DYNAMICS_H_
#define ASSIST_DYNAMICS_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	uint32_t speed_x100;
	uint8_t cadence_rpm;
	int32_t iq_scale;
	int32_t phase_current_max;
	bool walk_active;
	bool immediate_cut;
	bool safety_cut;
	bool profile_pedaling_active;
	uint16_t profile_release_ms;
	/*
	 * FW-069: Iq ramps, supplied per level by the caller (they live in the bank blob now,
	 * not in the global tuning blob). Same path profile_release_ms already took. 0 is not a
	 * legal ramp length, so the module falls back to its compiled defaults if it ever sees
	 * one — a missed assignment must not turn into an instant, un-ramped current step.
	 */
	uint16_t ramp_up_slow_ms;
	uint16_t ramp_up_fast_ms;
	uint16_t ramp_down_slow_ms;
	uint16_t ramp_down_fast_ms;
	/*
	 * FW-048: the motor has slowed into the range where the commutation angle switches
	 * formula (six-step), which makes the angle jump. Finish the fade NOW and let the motor
	 * coast out on its own inertia, so no current flows while the angle is unreliable.
	 * Set only while releasing — never during a start, which happens in that same range.
	 */
	bool coast_release;
} assist_dynamics_input_t;

int32_t assist_dynamics_apply(
	int32_t iq_target,
	int32_t iq_reference,
	const assist_dynamics_input_t *input);

#endif /* ASSIST_DYNAMICS_H_ */
