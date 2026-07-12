#ifndef ASSIST_LIMITS_H_
#define ASSIST_LIMITS_H_

#include <stdint.h>

#include "main.h"

int32_t assist_limits_apply_legacy(
	int32_t iq_request,
	uint16_t voltage_raw,
	uint16_t cadence_filtered,
	uint16_t speed_limit_x100,
	const MotorState_t *motor_state,
	const MotorParams_t *motor_params);

#endif /* ASSIST_LIMITS_H_ */
