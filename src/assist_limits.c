#include "assist_limits.h"

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

int32_t assist_limits_apply(
	int32_t iq_request,
	const assist_limits_input_t *input)
{
	if (input == 0) {
		return 0;
	}

	int32_t limited = map(input->voltage_raw,
		input->voltage_min_raw,
		input->voltage_min_raw + 176,
		0,
		iq_request);

	limited = map(input->controller_temperature_c, 75, 90, limited, 0);

	if (input->legal_enabled && !input->offroad && !input->walk_active) {
		if ((input->cadence_filtered_x8 >> 3) > 15) {
			limited = map(input->speed_x100,
				input->speed_limit_x100,
				input->speed_limit_x100 + 200,
				limited,
				0);
		} else {
			limited = map(input->speed_x100, 500, 700, limited, 0);
		}
	}

	return limited;
}

int32_t assist_limits_apply_legacy(
	int32_t iq_request,
	uint16_t voltage_raw,
	uint16_t cadence_filtered,
	uint16_t speed_limit_x100,
	const MotorState_t *motor_state,
	const MotorParams_t *motor_params)
{
	assist_limits_input_t input = {
		.voltage_raw = voltage_raw,
		.voltage_min_raw = motor_params->voltage_min,
		.controller_temperature_c = motor_state->int_Temperature,
		.cadence_filtered_x8 = cadence_filtered,
		.speed_x100 = motor_state->Speedx100,
		.speed_limit_x100 = speed_limit_x100,
		.legal_enabled = motor_params->legalflag != 0,
		.offroad = motor_state->offroadflag != RESET,
		.walk_active = motor_state->pushassist_flag != RESET
	};
	return assist_limits_apply(iq_request, &input);
}
