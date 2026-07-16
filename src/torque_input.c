#include "torque_input.h"

#include "config.h"

#define REST_TARGET_NATIVE 740

static uint16_t full_scale_native = TORQUE_INPUT_FULL_SCALE_DEFAULT_NATIVE;
static bool full_scale_stored_valid;
static uint16_t load_centikg;

static int32_t offset_correction;
static bool cal_fault;
static int32_t coast_rest_acc;
static bool coast_active;
static uint16_t coast_ticks;
static int32_t last_coast_rest;
static uint8_t reacquire_count;

static bool full_scale_in_range(uint16_t value)
{
	return value >= TORQUE_INPUT_FULL_SCALE_MIN_NATIVE &&
		value <= TORQUE_INPUT_FULL_SCALE_MAX_NATIVE;
}

static bool rest_raw_plausible(int32_t raw_rest)
{
	return raw_rest >= TQ_REST_RAW_MIN && raw_rest <= TQ_REST_RAW_MAX;
}

static void coast_accumulate(int16_t torque_corrected_native)
{
	if (!coast_active) {
		coast_active = true;
		coast_ticks = 0;
		coast_rest_acc = (int32_t)torque_corrected_native << 6;
		return;
	}
	coast_rest_acc -= coast_rest_acc >> 6;
	coast_rest_acc += torque_corrected_native;
	if (coast_ticks < 64000U) {
		coast_ticks++;
	}
}

static bool drift_confirmed(int32_t rest)
{
	int32_t spread = rest - last_coast_rest;

	if (spread < 0) {
		spread = -spread;
	}
	if (reacquire_count > 0 && spread <= TQ_REACQUIRE_TOL_MV) {
		reacquire_count++;
	} else {
		reacquire_count = 1;
	}
	last_coast_rest = rest;
	if (reacquire_count >= TQ_REACQUIRE_COASTS) {
		reacquire_count = 0;
		return true;
	}
	return false;
}

static void apply_offset_step(int32_t diff)
{
	if (diff > TQ_RECAL_MAX_STEP) {
		diff = TQ_RECAL_MAX_STEP;
	} else if (diff < -TQ_RECAL_MAX_STEP) {
		diff = -TQ_RECAL_MAX_STEP;
	}
	offset_correction += diff;
}

static void coast_evaluate(void)
{
	int32_t rest = coast_rest_acc >> 6;
	int32_t raw_rest = rest - offset_correction;
	int32_t diff;
	int32_t distance;

	if (!rest_raw_plausible(raw_rest)) {
		cal_fault = true;
		return;
	}
	cal_fault = false;

	diff = REST_TARGET_NATIVE - rest;
	distance = diff < 0 ? -diff : diff;
	if (distance <= TQ_RECAL_BAND_MV) {
		reacquire_count = 0;
		apply_offset_step(diff);
	} else if (drift_confirmed(rest)) {
		apply_offset_step(diff);
	}
}

void torque_input_startup_zero(int32_t rest_raw_native)
{
	offset_correction = REST_TARGET_NATIVE - rest_raw_native;
	if (!rest_raw_plausible(rest_raw_native)) {
		offset_correction = 0;
		cal_fault = true;
	}
}

int16_t torque_input_correct(uint16_t raw_native)
{
	return (int16_t)(raw_native + offset_correction);
}

void torque_input_coast_update(int16_t torque_corrected_native, bool coast_eligible)
{
	if (coast_eligible) {
		coast_accumulate(torque_corrected_native);
		return;
	}
	if (coast_active && coast_ticks > TQ_RECAL_SETTLE_TICKS) {
		coast_evaluate();
	}
	coast_active = false;
}

bool torque_input_cal_fault(void)
{
	return cal_fault;
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
