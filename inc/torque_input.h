#ifndef TORQUE_INPUT_H_
#define TORQUE_INPUT_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Single owner of the torque sensor chain: raw ADC millivolts -> automatic
 * zero -> corrected signal -> delta above zero -> public kilogram-force
 * scale in 0.01 kg units. Default characteristic measured with reference
 * weights on this bike (165 mm crank): zero 740 mV, 6 kg at 886 mV and
 * 84 kg at 2320 mV. The default conversion is piecewise-linear through
 * those measured points, so the firmware is usable without load calibration.
 * A user load calibration may replace it with a linear 60 kg span; the zero point
 * is always automatic and never writable. The assist deadband is a separate
 * relative offset so the kg scale starts at the true zero. Integer math
 * only. torque_input_cal_fault() reports overall signal plausibility:
 * implausible zero (startup/coast) or a stuck-high load that never dips.
 */

#define TORQUE_ZERO_TARGET_NATIVE        740U
#define TORQUE_DEFAULT_LOW_NATIVE        146U
#define TORQUE_DEFAULT_LOW_CENTIKG       600U
#define TORQUE_DEFAULT_HIGH_NATIVE       1580U
#define TORQUE_DEFAULT_HIGH_CENTIKG      8400U
/* Interpolated native delta corresponding to 60.00 kg on the default curve. */
#define TORQUE_DEFAULT_SPAN_NATIVE       1139U
#define TORQUE_SPAN_MIN_NATIVE           800U
#define TORQUE_SPAN_MAX_NATIVE           2600U
#define TORQUE_PUBLIC_FULL_SCALE_CENTIKG 6000U
#define TORQUE_INPUT_MAX_CENTIKG         12000U
#define TORQUE_ASSIST_DEADBAND_NATIVE    10U
#define TORQUE_ASSIST_FILTER_MS          35U
#define TORQUE_INPUT_TICKS_PER_MS        4U
#define TORQUE_ASSIST_FILTER_Q_SHIFT     8U
/* FW-033: slow "RUN" effort estimator (second filter of the fast signal). */
#define TORQUE_RUN_FILTER_MS_DEFAULT     300U
#define TORQUE_RUN_FILTER_MS_MAX         1000U

typedef enum {
	TORQUE_CAL_SOURCE_DEFAULT = 0,
	TORQUE_CAL_SOURCE_USER = 1
} torque_cal_source_t;

typedef enum {
	TORQUE_CAL_STATE_IDLE = 0,
	TORQUE_CAL_STATE_CAPTURE_ZERO = 1,
	TORQUE_CAL_STATE_WAIT_REFERENCE = 2,
	TORQUE_CAL_STATE_CAPTURE_LOAD = 3,
	TORQUE_CAL_STATE_PREVIEW = 4,
	TORQUE_CAL_STATE_SUCCESS = 5,
	TORQUE_CAL_STATE_FAILED = 6,
	TORQUE_CAL_STATE_CANCELLED = 7
} torque_cal_state_t;

typedef enum {
	TORQUE_CAL_ERR_NONE = 0,
	TORQUE_CAL_ERR_NOT_STATIONARY = 1,
	TORQUE_CAL_ERR_UNSTABLE = 2,
	TORQUE_CAL_ERR_REFERENCE_RANGE = 3,
	TORQUE_CAL_ERR_DELTA_TOO_SMALL = 4,
	TORQUE_CAL_ERR_SATURATED = 5,
	TORQUE_CAL_ERR_SPAN_RANGE = 6,
	TORQUE_CAL_ERR_SENSOR_FAULT = 7,
	TORQUE_CAL_ERR_TIMEOUT = 8
} torque_cal_error_t;

#define TORQUE_CAL_REFERENCE_MIN_CENTIKG 500U
#define TORQUE_CAL_REFERENCE_MAX_CENTIKG 3000U

typedef struct {
	uint16_t raw_native;
	uint16_t zero_effective_native;
	int16_t corrected_native;
	uint16_t delta_native;
	uint16_t assist_delta_native;
	uint16_t assist_delta_filtered_native;
	uint16_t assist_delta_run_native;   /* FW-033: slow RUN estimator of the fast signal */
	uint16_t load_centikg;
	uint16_t span_native;
	uint8_t calibration_source;
	bool sensor_valid;
} torque_snapshot_t;

void torque_input_init(void);
void torque_input_startup_zero(int32_t rest_raw_native);
int16_t torque_input_correct(uint16_t raw_native);
/* FW-058: bike_moving gates the minimum period between in-ride re-zeros. Pass
 * false when the wheel is stopped — standstill re-zero stays unrestricted. */
void torque_input_coast_update(int16_t torque_corrected_native, bool coast_eligible,
	bool bike_moving);
bool torque_input_cal_fault(void);
void torque_input_update(uint16_t raw_native, int16_t torque_corrected_native,
	bool sensor_valid);

const torque_snapshot_t *torque_input_get_snapshot(void);

/* FW-033: RUN effort estimator. set_run_filter_ms configures the slow filter
 * (0 = disabled, run tracks the fast signal = old behaviour). seed_run pre-loads
 * the filter (e.g. to the fast value at ride start) for a crisp launch. */
void torque_input_set_run_filter_ms(uint16_t ms);
void torque_input_seed_run(uint16_t value_native);

uint16_t torque_input_load_centikg(void);
uint16_t torque_input_zero_native(void);
uint16_t torque_input_span_native(void);
uint16_t torque_input_full_scale_native(void);
uint8_t torque_input_calibration_source(void);

bool torque_input_set_user_span(uint16_t span_native);
void torque_input_restore_default_span(void);

bool torque_input_calibration_active(void);
uint8_t torque_input_cal_state(void);
uint8_t torque_input_cal_error(void);
uint16_t torque_input_cal_reference_centikg(void);
uint16_t torque_input_cal_preview_span(void);
void torque_input_cal_start(void);
void torque_input_cal_capture_load(uint16_t reference_centikg);
bool torque_input_cal_commit(void);
void torque_input_cal_cancel(void);
void torque_input_cal_restore_default(void);
bool torque_input_cal_take_persist_request(void);
void torque_input_cal_tick(int16_t torque_corrected_native, bool stationary);

/* Persist span with magic/version/CRC. restore returns true when a valid user
 * calibration was loaded; a bad or absent record keeps the default span. */
bool torque_input_restore_persist(uint16_t magic, uint8_t version,
	uint16_t span_native, uint16_t crc);
void torque_input_build_persist(uint16_t *magic, uint8_t *version,
	uint16_t *span_native, uint16_t *crc);

/* Versioned CAN telemetry blob (0x6025): capabilities + load + calibration
 * status + CRC16. Returns byte length written.
 * FW-061 v2 appends the coast re-zero diagnostics: without them a zero that walks
 * cannot be told apart from a zero that is simply never being corrected. */
#define TORQUE_TELEMETRY_BLOB_LEN 56U
#define TORQUE_TELEMETRY_BLOB_LEN_V1 24U
#define TORQUE_CAP_LOAD_TELEMETRY_V1 0x01U
#define TORQUE_CAP_CALIBRATION_V1    0x02U
#define TORQUE_CAP_COAST_DIAG_V2     0x04U
uint16_t torque_input_serialize_telemetry(uint8_t *buffer);

/* FW-061: outcome of the most recent coast evaluation. Kept deliberately
 * separate from the counters so a single reading answers "what happened last
 * time" as well as "how often". */
typedef enum {
	TORQUE_COAST_NONE = 0,
	TORQUE_COAST_APPLIED = 1,
	TORQUE_COAST_NO_CHANGE = 2,
	TORQUE_COAST_TOO_SHORT = 3,
	TORQUE_COAST_UNSTABLE = 4,
	TORQUE_COAST_LOCKOUT = 5,
	TORQUE_COAST_OUT_OF_REACQUIRE_RANGE = 6,
	TORQUE_COAST_IMPLAUSIBLE_RAW = 7
} torque_coast_result_t;

uint16_t torque_input_centikg_to_native_delta(uint16_t centikg);
uint16_t torque_input_native_delta_to_centikg(uint16_t delta_native);

#endif /* TORQUE_INPUT_H_ */
