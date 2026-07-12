#include "assist_limits.h"

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

int32_t assist_limits_apply_legacy(
	int32_t iq_request,
	uint16_t voltage_raw,
	uint16_t cadence_filtered,
	uint16_t speed_limit_x100,
	const MotorState_t *motor_state,
	const MotorParams_t *motor_params)
{
	int32_t limited = map(voltage_raw,
		motor_params->voltage_min,
		motor_params->voltage_min + 176,
		0,
		iq_request);

	limited = map(motor_state->int_Temperature, 75, 90, limited, 0);

	if (motor_params->legalflag &&
		!motor_state->offroadflag &&
		!motor_state->pushassist_flag) {
		if ((cadence_filtered >> 3) > 15) {
			limited = map(motor_state->Speedx100,
				speed_limit_x100,
				speed_limit_x100 + 200,
				limited,
				0);
		} else {
			limited = map(motor_state->Speedx100, 500, 700, limited, 0);
		}
	}

	return limited;
}
