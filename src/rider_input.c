#include "rider_input.h"

static rider_input_t current_input;

void rider_input_update(const rider_input_t *sample)
{
	if (sample != 0) {
		current_input = *sample;
	}
}

const rider_input_t *rider_input_get(void)
{
	return &current_input;
}
