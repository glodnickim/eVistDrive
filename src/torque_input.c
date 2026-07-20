#include "torque_input.h"

#include "config.h"

#define REST_TARGET_NATIVE ((int32_t)TORQUE_ZERO_TARGET_NATIVE)

static uint16_t span_native = TORQUE_DEFAULT_SPAN_NATIVE;
static uint8_t calibration_source = TORQUE_CAL_SOURCE_DEFAULT;
static torque_snapshot_t snapshot;
static int32_t assist_filter_q;

static int32_t offset_correction;
static bool cal_fault;
static int32_t coast_rest_acc;
static bool coast_active;
static uint16_t coast_ticks;
static int32_t last_coast_rest;
static uint8_t reacquire_count;
static uint32_t stuck_ticks;
static bool stuck_fault;

static bool span_in_range(uint16_t value)
{
	return value >= TORQUE_SPAN_MIN_NATIVE &&
		value <= TORQUE_SPAN_MAX_NATIVE;
}

static uint16_t default_native_delta_to_centikg(uint16_t delta_native)
{
	uint32_t load;
	if (delta_native <= TORQUE_DEFAULT_LOW_NATIVE) {
		load = ((uint32_t)delta_native * TORQUE_DEFAULT_LOW_CENTIKG +
			TORQUE_DEFAULT_LOW_NATIVE / 2U) / TORQUE_DEFAULT_LOW_NATIVE;
	} else {
		load = TORQUE_DEFAULT_LOW_CENTIKG +
			((uint32_t)(delta_native - TORQUE_DEFAULT_LOW_NATIVE) *
			(TORQUE_DEFAULT_HIGH_CENTIKG - TORQUE_DEFAULT_LOW_CENTIKG) +
			(TORQUE_DEFAULT_HIGH_NATIVE - TORQUE_DEFAULT_LOW_NATIVE) / 2U) /
			(TORQUE_DEFAULT_HIGH_NATIVE - TORQUE_DEFAULT_LOW_NATIVE);
	}
	return (load > TORQUE_INPUT_MAX_CENTIKG) ?
		TORQUE_INPUT_MAX_CENTIKG : (uint16_t)load;
}

static uint16_t default_centikg_to_native_delta(uint16_t centikg)
{
	uint32_t delta;
	if (centikg <= TORQUE_DEFAULT_LOW_CENTIKG) {
		delta = ((uint32_t)centikg * TORQUE_DEFAULT_LOW_NATIVE +
			TORQUE_DEFAULT_LOW_CENTIKG / 2U) / TORQUE_DEFAULT_LOW_CENTIKG;
	} else {
		delta = TORQUE_DEFAULT_LOW_NATIVE +
			((uint32_t)(centikg - TORQUE_DEFAULT_LOW_CENTIKG) *
			(TORQUE_DEFAULT_HIGH_NATIVE - TORQUE_DEFAULT_LOW_NATIVE) +
			(TORQUE_DEFAULT_HIGH_CENTIKG - TORQUE_DEFAULT_LOW_CENTIKG) / 2U) /
			(TORQUE_DEFAULT_HIGH_CENTIKG - TORQUE_DEFAULT_LOW_CENTIKG);
	}
	return (delta > TORQUE_SPAN_MAX_NATIVE) ?
		TORQUE_SPAN_MAX_NATIVE : (uint16_t)delta;
}

static uint16_t native_delta_to_centikg(uint16_t delta_native)
{
	if (calibration_source == TORQUE_CAL_SOURCE_DEFAULT) {
		return default_native_delta_to_centikg(delta_native);
	}
	uint32_t load = ((uint32_t)delta_native *
		TORQUE_PUBLIC_FULL_SCALE_CENTIKG + span_native / 2U) / span_native;
	return (load > TORQUE_INPUT_MAX_CENTIKG) ?
		TORQUE_INPUT_MAX_CENTIKG : (uint16_t)load;
}

static uint16_t centikg_to_native_delta(uint16_t centikg)
{
	if (centikg > TORQUE_INPUT_MAX_CENTIKG) {
		centikg = TORQUE_INPUT_MAX_CENTIKG;
	}
	if (calibration_source == TORQUE_CAL_SOURCE_DEFAULT) {
		return default_centikg_to_native_delta(centikg);
	}
	uint32_t delta = ((uint32_t)centikg * span_native +
		TORQUE_PUBLIC_FULL_SCALE_CENTIKG / 2U) /
		TORQUE_PUBLIC_FULL_SCALE_CENTIKG;
	return (delta > TORQUE_SPAN_MAX_NATIVE) ?
		TORQUE_SPAN_MAX_NATIVE : (uint16_t)delta;
}

static uint16_t update_assist_filter(uint16_t target_native)
{
	const int32_t filter_ticks =
		(int32_t)TORQUE_ASSIST_FILTER_MS * TORQUE_INPUT_TICKS_PER_MS;
	int32_t target_q = (int32_t)target_native << TORQUE_ASSIST_FILTER_Q_SHIFT;
	int32_t error_q = target_q - assist_filter_q;
	if (error_q != 0) {
		int32_t step_q = error_q / filter_ticks;
		if (step_q == 0) {
			step_q = (error_q > 0) ? 1 : -1;
		}
		assist_filter_q += step_q;
	}
	return (uint16_t)((assist_filter_q +
		(1L << (TORQUE_ASSIST_FILTER_Q_SHIFT - 1U))) >>
		TORQUE_ASSIST_FILTER_Q_SHIFT);
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
	} else if (distance <= TQ_REACQUIRE_MAX_MV) {
		if (drift_confirmed(rest)) {
			apply_offset_step(diff);
		}
	} else {
		reacquire_count = 0;
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
	if (torque_input_calibration_active()) {
		coast_active = false;
		return;
	}
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
	return cal_fault || stuck_fault;
}

void torque_input_init(void)
{
	span_native = TORQUE_DEFAULT_SPAN_NATIVE;
	calibration_source = TORQUE_CAL_SOURCE_DEFAULT;
	snapshot = (torque_snapshot_t){0};
	assist_filter_q = 0;
	snapshot.zero_effective_native = TORQUE_ZERO_TARGET_NATIVE;
	snapshot.span_native = span_native;
}

void torque_input_update(uint16_t raw_native, int16_t torque_corrected_native,
	bool sensor_valid)
{
	int32_t delta = (int32_t)torque_corrected_native - REST_TARGET_NATIVE;
	uint16_t assist_delta;

	if (delta < 0) {
		delta = 0;
	}
	if (delta > (int32_t)TORQUE_SPAN_MAX_NATIVE) {
		delta = TORQUE_SPAN_MAX_NATIVE;
	}

	snapshot.raw_native = raw_native;
	snapshot.zero_effective_native =
		(uint16_t)(REST_TARGET_NATIVE - offset_correction);
	snapshot.corrected_native = torque_corrected_native;
	snapshot.delta_native = (uint16_t)delta;
	assist_delta = (delta > (int32_t)TORQUE_ASSIST_DEADBAND_NATIVE) ?
		(uint16_t)(delta - TORQUE_ASSIST_DEADBAND_NATIVE) : 0U;
	if (assist_delta > span_native) {
		assist_delta = span_native;
	}
	snapshot.assist_delta_native = assist_delta;
	snapshot.assist_delta_filtered_native = update_assist_filter(
		sensor_valid ? assist_delta : 0U);
	snapshot.load_centikg = native_delta_to_centikg((uint16_t)delta);
	snapshot.span_native = span_native;
	snapshot.calibration_source = calibration_source;
	snapshot.sensor_valid = sensor_valid;

	if (snapshot.load_centikg >= TQ_STUCK_CENTIKG) {
		if (stuck_ticks < TQ_STUCK_TICKS) {
			stuck_ticks++;
		} else {
			stuck_fault = true;
		}
	} else {
		stuck_ticks = 0;
		stuck_fault = false;
	}
}

const torque_snapshot_t *torque_input_get_snapshot(void)
{
	return &snapshot;
}

uint16_t torque_input_load_centikg(void)
{
	return snapshot.load_centikg;
}

uint16_t torque_input_zero_native(void)
{
	return TORQUE_ZERO_TARGET_NATIVE;
}

uint16_t torque_input_span_native(void)
{
	return span_native;
}

uint16_t torque_input_full_scale_native(void)
{
	return (uint16_t)(TORQUE_ZERO_TARGET_NATIVE + span_native);
}

uint8_t torque_input_calibration_source(void)
{
	return calibration_source;
}

bool torque_input_set_user_span(uint16_t value)
{
	if (!span_in_range(value)) {
		return false;
	}
	span_native = value;
	calibration_source = TORQUE_CAL_SOURCE_USER;
	return true;
}

void torque_input_restore_default_span(void)
{
	span_native = TORQUE_DEFAULT_SPAN_NATIVE;
	calibration_source = TORQUE_CAL_SOURCE_DEFAULT;
}

#define CAL_AVG_SHIFT 6U
#define CAL_AVG_TICKS 256U
#define CAL_STABILITY_MAX_SPREAD 8
#define CAL_DELTA_MIN_NATIVE 100
#define CAL_SATURATION_NATIVE 3200
#define CAL_WAIT_TIMEOUT_TICKS 120000UL

static uint8_t cal_state = TORQUE_CAL_STATE_IDLE;
static uint8_t cal_error = TORQUE_CAL_ERR_NONE;
static uint16_t cal_reference_centikg;
static uint16_t cal_preview_span;
static int32_t cal_zero_avg;
static int32_t cal_acc;
static int16_t cal_window_min;
static int16_t cal_window_max;
static uint16_t cal_window_ticks;
static uint32_t cal_wait_ticks;
static bool cal_persist_request;

static void cal_window_reset(int16_t corrected)
{
	cal_acc = 0;
	cal_window_min = corrected;
	cal_window_max = corrected;
	cal_window_ticks = 0;
}

static void cal_fail(uint8_t error)
{
	cal_state = TORQUE_CAL_STATE_FAILED;
	cal_error = error;
}

bool torque_input_calibration_active(void)
{
	return cal_state >= TORQUE_CAL_STATE_CAPTURE_ZERO &&
		cal_state <= TORQUE_CAL_STATE_PREVIEW;
}

uint8_t torque_input_cal_state(void)
{
	return cal_state;
}

uint8_t torque_input_cal_error(void)
{
	return cal_error;
}

uint16_t torque_input_cal_reference_centikg(void)
{
	return cal_reference_centikg;
}

uint16_t torque_input_cal_preview_span(void)
{
	return cal_preview_span;
}

void torque_input_cal_start(void)
{
	cal_error = TORQUE_CAL_ERR_NONE;
	cal_preview_span = 0;
	cal_reference_centikg = 0;
	cal_wait_ticks = 0;
	cal_window_reset(snapshot.corrected_native);
	cal_state = TORQUE_CAL_STATE_CAPTURE_ZERO;
}

void torque_input_cal_capture_load(uint16_t reference_centikg)
{
	if (cal_state != TORQUE_CAL_STATE_WAIT_REFERENCE) {
		return;
	}
	if (reference_centikg < TORQUE_CAL_REFERENCE_MIN_CENTIKG ||
		reference_centikg > TORQUE_CAL_REFERENCE_MAX_CENTIKG) {
		cal_fail(TORQUE_CAL_ERR_REFERENCE_RANGE);
		return;
	}
	cal_reference_centikg = reference_centikg;
	cal_window_reset(snapshot.corrected_native);
	cal_state = TORQUE_CAL_STATE_CAPTURE_LOAD;
}

bool torque_input_cal_commit(void)
{
	if (cal_state != TORQUE_CAL_STATE_PREVIEW ||
		!torque_input_set_user_span(cal_preview_span)) {
		return false;
	}
	cal_state = TORQUE_CAL_STATE_SUCCESS;
	cal_persist_request = true;
	return true;
}

void torque_input_cal_cancel(void)
{
	if (torque_input_calibration_active()) {
		cal_state = TORQUE_CAL_STATE_CANCELLED;
	}
}

void torque_input_cal_restore_default(void)
{
	torque_input_restore_default_span();
	cal_state = TORQUE_CAL_STATE_IDLE;
	cal_error = TORQUE_CAL_ERR_NONE;
	cal_persist_request = true;
}

bool torque_input_cal_take_persist_request(void)
{
	bool pending = cal_persist_request;
	cal_persist_request = false;
	return pending;
}

void torque_input_cal_tick(int16_t torque_corrected_native, bool stationary)
{
	if (!torque_input_calibration_active()) {
		return;
	}
	if (cal_fault || stuck_fault) {
		cal_fail(TORQUE_CAL_ERR_SENSOR_FAULT);
		return;
	}
	if (!stationary) {
		cal_fail(TORQUE_CAL_ERR_NOT_STATIONARY);
		return;
	}

	if (cal_state == TORQUE_CAL_STATE_WAIT_REFERENCE) {
		if (++cal_wait_ticks > CAL_WAIT_TIMEOUT_TICKS) {
			cal_fail(TORQUE_CAL_ERR_TIMEOUT);
		}
		return;
	}
	if (cal_state == TORQUE_CAL_STATE_PREVIEW) {
		if (++cal_wait_ticks > CAL_WAIT_TIMEOUT_TICKS) {
			cal_fail(TORQUE_CAL_ERR_TIMEOUT);
		}
		return;
	}

	cal_acc += torque_corrected_native;
	if (torque_corrected_native < cal_window_min) {
		cal_window_min = torque_corrected_native;
	}
	if (torque_corrected_native > cal_window_max) {
		cal_window_max = torque_corrected_native;
	}
	if (++cal_window_ticks < CAL_AVG_TICKS) {
		return;
	}

	if ((cal_window_max - cal_window_min) > CAL_STABILITY_MAX_SPREAD) {
		cal_fail(TORQUE_CAL_ERR_UNSTABLE);
		return;
	}
	int32_t average = cal_acc >> (CAL_AVG_SHIFT + 2U);

	if (cal_state == TORQUE_CAL_STATE_CAPTURE_ZERO) {
		cal_zero_avg = average;
		cal_wait_ticks = 0;
		cal_state = TORQUE_CAL_STATE_WAIT_REFERENCE;
		return;
	}

	if (average >= CAL_SATURATION_NATIVE) {
		cal_fail(TORQUE_CAL_ERR_SATURATED);
		return;
	}
	int32_t delta_reference = average - cal_zero_avg;
	if (delta_reference < CAL_DELTA_MIN_NATIVE) {
		cal_fail(TORQUE_CAL_ERR_DELTA_TOO_SMALL);
		return;
	}
	uint32_t span = ((uint32_t)delta_reference *
		TORQUE_PUBLIC_FULL_SCALE_CENTIKG) / cal_reference_centikg;
	if (!span_in_range((uint16_t)span)) {
		cal_fail(TORQUE_CAL_ERR_SPAN_RANGE);
		return;
	}
	cal_preview_span = (uint16_t)span;
	cal_wait_ticks = 0;
	cal_state = TORQUE_CAL_STATE_PREVIEW;
}

#define TORQUE_CAL_PERSIST_MAGIC 0x7C41U
#define TORQUE_CAL_PERSIST_VERSION 1U

static uint16_t torque_cal_crc16(uint8_t version, uint16_t span)
{
	uint8_t buffer[3] = { version, (uint8_t)(span & 0xFFU),
		(uint8_t)(span >> 8) };
	uint16_t crc = 0xFFFFU;
	for (uint8_t i = 0; i < sizeof(buffer); i++) {
		crc ^= (uint16_t)buffer[i] << 8;
		for (uint8_t bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000U) ?
				(uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

bool torque_input_restore_persist(uint16_t magic, uint8_t version,
	uint16_t span_stored, uint16_t crc)
{
	if (magic != TORQUE_CAL_PERSIST_MAGIC ||
		version != TORQUE_CAL_PERSIST_VERSION ||
		crc != torque_cal_crc16(version, span_stored) ||
		!span_in_range(span_stored)) {
		return false;
	}
	span_native = span_stored;
	calibration_source = TORQUE_CAL_SOURCE_USER;
	return true;
}

void torque_input_build_persist(uint16_t *magic, uint8_t *version,
	uint16_t *span_out, uint16_t *crc)
{
	if (calibration_source == TORQUE_CAL_SOURCE_USER) {
		*magic = TORQUE_CAL_PERSIST_MAGIC;
		*version = TORQUE_CAL_PERSIST_VERSION;
		*span_out = span_native;
		*crc = torque_cal_crc16(TORQUE_CAL_PERSIST_VERSION, span_native);
	} else {
		*magic = 0;
		*version = 0;
		*span_out = 0;
		*crc = 0;
	}
}

static void put16(uint8_t *b, uint16_t v)
{
	b[0] = (uint8_t)(v & 0xFFU);
	b[1] = (uint8_t)(v >> 8);
}

uint16_t torque_input_serialize_telemetry(uint8_t *buffer)
{
	buffer[0] = 0x54;  /* 'T' */
	buffer[1] = 0x43;  /* 'C' */
	buffer[2] = 1;     /* version */
	buffer[3] = TORQUE_CAP_LOAD_TELEMETRY_V1 | TORQUE_CAP_CALIBRATION_V1;
	put16(&buffer[4], snapshot.load_centikg);
	put16(&buffer[6], snapshot.zero_effective_native);
	put16(&buffer[8], snapshot.delta_native);
	put16(&buffer[10], span_native);
	buffer[12] = calibration_source;
	buffer[13] = cal_state;
	buffer[14] = cal_error;
	buffer[15] = snapshot.sensor_valid ? 1U : 0U;
	put16(&buffer[16], cal_reference_centikg);
	put16(&buffer[18], cal_preview_span);
	put16(&buffer[20], torque_input_full_scale_native());
	uint16_t crc = 0xFFFFU;
	for (uint8_t i = 0; i < 22; i++) {
		crc ^= (uint16_t)buffer[i] << 8;
		for (uint8_t bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000U) ?
				(uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
		}
	}
	put16(&buffer[22], crc);
	return TORQUE_TELEMETRY_BLOB_LEN;
}

uint16_t torque_input_centikg_to_native_delta(uint16_t centikg)
{
	return centikg_to_native_delta(centikg);
}

uint16_t torque_input_native_delta_to_centikg(uint16_t delta_native)
{
	if (delta_native > TORQUE_SPAN_MAX_NATIVE) {
		delta_native = TORQUE_SPAN_MAX_NATIVE;
	}
	return native_delta_to_centikg(delta_native);
}
