#include "torque_input.h"

static uint16_t full_scale_native = TORQUE_INPUT_FULL_SCALE_DEFAULT_NATIVE;
static bool full_scale_stored_valid;
static uint16_t load_centikg;

static bool full_scale_in_range(uint16_t value)
{
	return value >= TORQUE_INPUT_FULL_SCALE_MIN_NATIVE &&
		value <= TORQUE_INPUT_FULL_SCALE_MAX_NATIVE;
}

void torque_input_init(uint16_t stored_full_scale_native)
{
	full_scale_stored_valid = full_scale_in_range(stored_full_scale_native);
	full_scale_native = full_scale_stored_valid ?
		stored_full_scale_native : TORQUE_INPUT_FULL_SCALE_DEFAULT_NATIVE;
	load_centikg = 0;
}

void torque_input_update(int16_t torque_corrected_native)
{
	int32_t delta = (int32_t)torque_corrected_native -
		(int32_t)TORQUE_INPUT_ZERO_NATIVE;
	uint16_t span = torque_input_span_native();

	if (delta < 0) {
		delta = 0;
	}
	if (delta > (int32_t)span) {
		delta = span;
	}
	load_centikg = (uint16_t)(((uint32_t)delta *
		TORQUE_INPUT_FULL_SCALE_CENTIKG + span / 2U) / span);
}

uint16_t torque_input_load_centikg(void)
{
	return load_centikg;
}

uint16_t torque_input_zero_native(void)
{
	return TORQUE_INPUT_ZERO_NATIVE;
}

uint16_t torque_input_full_scale_native(void)
{
	return full_scale_native;
}

uint16_t torque_input_span_native(void)
{
	return full_scale_native - TORQUE_INPUT_ZERO_NATIVE;
}

bool torque_input_full_scale_stored_valid(void)
{
	return full_scale_stored_valid;
}

bool torque_input_set_full_scale_native(uint16_t value)
{
	if (!full_scale_in_range(value)) {
		return false;
	}
	full_scale_native = value;
	return true;
}

uint16_t torque_input_centikg_to_native_delta(uint16_t centikg)
{
	if (centikg > TORQUE_INPUT_FULL_SCALE_CENTIKG) {
		centikg = TORQUE_INPUT_FULL_SCALE_CENTIKG;
	}
	return (uint16_t)(((uint32_t)centikg * torque_input_span_native() +
		TORQUE_INPUT_FULL_SCALE_CENTIKG / 2U) /
		TORQUE_INPUT_FULL_SCALE_CENTIKG);
}

uint16_t torque_input_native_delta_to_centikg(uint16_t delta_native)
{
	uint16_t span = torque_input_span_native();

	if (delta_native > span) {
		delta_native = span;
	}
	return (uint16_t)(((uint32_t)delta_native *
		TORQUE_INPUT_FULL_SCALE_CENTIKG + span / 2U) / span);
}
