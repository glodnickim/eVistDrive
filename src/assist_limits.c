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
