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
	bool safety_cut;
	bool profile_pedaling_active;
	uint16_t profile_release_ms;
} assist_dynamics_input_t;

int32_t assist_dynamics_apply(
	int32_t iq_target,
	int32_t iq_reference,
	const assist_dynamics_input_t *input);

#endif /* ASSIST_DYNAMICS_H_ */
