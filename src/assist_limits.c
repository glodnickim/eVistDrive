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
		if (input->source == ASSIST_LIMIT_SOURCE_PEDAL_CONFIRMED) {
			limited = map(input->speed_x100,
				input->speed_limit_x100,
				input->speed_limit_x100 + 200,
				limited,
				0);
		} else {
			/* Non-pedal demand (throttle, without-rotation launch): 5..7 km/h. */
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
	/*
	 * FW-091: the frozen Legacy monolith keeps its original behaviour, cadence guess and
	 * all — it is not part of the ride core and must not change. The cadence value is
	 * mapped to the new source enum here so the shared limiter has one code path.
	 */
	assist_limits_input_t input = {
		.voltage_raw = voltage_raw,
		.voltage_min_raw = motor_params->voltage_min,
		.controller_temperature_c = motor_state->int_Temperature,
		.source = ((cadence_filtered >> 3) > 15) ?
			ASSIST_LIMIT_SOURCE_PEDAL_CONFIRMED :
			ASSIST_LIMIT_SOURCE_NON_PEDAL,
		.speed_x100 = motor_state->Speedx100,
		.speed_limit_x100 = speed_limit_x100,
		.legal_enabled = motor_params->legalflag != 0,
		.offroad = motor_state->offroadflag != RESET,
		.walk_active = motor_state->pushassist_flag != RESET
	};
	return assist_limits_apply(iq_request, &input);
}
