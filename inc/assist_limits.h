#ifndef ASSIST_LIMITS_H_
#define ASSIST_LIMITS_H_

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

typedef struct {
	uint16_t voltage_raw;
	int16_t voltage_min_raw;
	int16_t controller_temperature_c;
	uint16_t cadence_filtered_x8;
	uint32_t speed_x100;
	uint16_t speed_limit_x100;
	bool legal_enabled;
	bool offroad;
	bool walk_active;
} assist_limits_input_t;

int32_t assist_limits_apply(
	int32_t iq_request,
	const assist_limits_input_t *input);

int32_t assist_limits_apply_legacy(
	int32_t iq_request,
	uint16_t voltage_raw,
	uint16_t cadence_filtered,
	uint16_t speed_limit_x100,
	const MotorState_t *motor_state,
	const MotorParams_t *motor_params);

#endif /* ASSIST_LIMITS_H_ */
