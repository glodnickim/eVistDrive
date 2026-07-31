#ifndef ASSIST_MODES_H_
#define ASSIST_MODES_H_

#include <stdbool.h>
#include <stdint.h>

#include "assist_start.h"
#include "rider_input.h"

typedef enum {
	ASSIST_MODE_LEGACY = 0,
	ASSIST_MODE_POWER_LINEAR = 1,
	ASSIST_MODE_POWER_PROGRESSIVE = 2,
	ASSIST_MODE_EMTB = 3,
	ASSIST_MODE_EMTB_CUSTOM = 4,
	ASSIST_MODE_TORQUE = 5,
	ASSIST_MODE_POWER_CURVE = 6 //FW-056
} assist_mode_type_t;

/*
 * WIRE VALUES — byte 0 of every level record in the bank blob, and the mode_type enum in
 * protocol/ebics_config_schema.yaml. A stored bank carries the NUMBER, not the name, so
 * renumbering any of these silently reinterprets every profile the rider already saved:
 * a bank written as eMTB would come back as something else after a firmware update, with
 * no error anywhere. Renaming a member is free; changing its value is not.
 *
 * FW-073 renamed the eMTB member. These assertions exist so the next rename cannot quietly
 * take the value with it.
 */
_Static_assert(ASSIST_MODE_LEGACY == 0, "wire value must stay 0");
_Static_assert(ASSIST_MODE_POWER_LINEAR == 1, "wire value must stay 1");
_Static_assert(ASSIST_MODE_POWER_PROGRESSIVE == 2, "wire value must stay 2");
_Static_assert(ASSIST_MODE_EMTB == 3, "eMTB wire value must stay 3");
_Static_assert(ASSIST_MODE_EMTB_CUSTOM == 4, "wire value must stay 4");
_Static_assert(ASSIST_MODE_TORQUE == 5, "wire value must stay 5");
_Static_assert(ASSIST_MODE_POWER_CURVE == 6, "wire value must stay 6");

typedef struct {
	assist_mode_type_t mode_type;
	uint16_t support_ratio_pct;
	uint16_t support_min_pct;
	uint16_t support_max_pct;
	uint16_t reference_power_w;
	uint8_t progression_pct;
	/* FW-056: gamma x10 (3..25) for ASSIST_MODE_POWER_CURVE, one per half of the
	 * support window: _x10 shapes support_min -> window middle, _high_x10 shapes
	 * window middle -> support_max. On the wire they ride in bytes that belong to
	 * other modes and are dead in POWER_CURVE (byte 9 = progression_pct, byte 1 =
	 * low half of support_ratio_pct), so the record stays 35 B and the blob 189 B. */
	uint8_t curve_exponent_x10;
	uint8_t curve_exponent_high_x10;
	uint8_t emtb_parameter;
	bool emtb_based_on_power;
	uint16_t emtb_reference_voltage_mv;
	uint8_t torque_assist_factor;
	uint16_t max_motor_power_w;
	uint8_t max_iq_pct;
	bool assist_without_rotation;
	uint16_t without_rotation_threshold_mv;
	assist_startup_boost_config_t startup_boost;
	assist_smooth_start_config_t smooth_start;
	uint16_t release_ms;
	uint16_t power_rise_filter_ms;
	uint16_t power_fall_filter_ms;
	/* FW-068: start condition, per level. All zero = exactly the pre-FW-068 behaviour.
	 * start_load_reduction_mv lowers without_rotation_threshold_mv while the cranks are
	 * actually turning, so re-catching assist in motion no longer costs as much as a
	 * standstill launch. start_rise_* is the alternative engage path: a *rise* of pedal
	 * load after the crank moved, which is immune to zero drift (see FW-058/FW-059). */
	uint16_t start_load_reduction_mv;
	uint16_t start_rise_mv;
	uint16_t start_rise_window_ms;
	/* FW-069: Iq ramps, moved here from the global tuning blob. They decide the character
	 * of how power builds, so they belong next to release_ms/power_*_filter_ms, which were
	 * already per level. Per level in a per-bank store also gives per-bank for free. */
	uint16_t iq_rise_slow_ms;
	uint16_t iq_rise_fast_ms;
	uint16_t iq_fall_slow_ms;
	uint16_t iq_fall_fast_ms;
} assist_level_config_t;

typedef struct {
	uint16_t human_power_w;
	uint16_t assist_basis_power_w;
	uint16_t raw_motor_power_w;
	uint16_t motor_power_w;
	uint16_t applied_support_ratio_pct;
	uint32_t requested_battery_current_ma;
	int32_t iq_request;
	uint8_t cadence_for_assist_rpm;
	bool assist_without_rotation_active;
	uint16_t torque_for_assist_mv;
	uint16_t startup_boost_extra_pct;
	bool startup_boost_active;
	uint16_t emtb_denominator;
	uint16_t emtb_target_x160;
	/* FW-056: bench diagnostics only, not serialized to CAN. */
	uint16_t curve_input_permille;
	uint16_t curve_output_permille;
	/* FW-057: cadence compensation, reported over 0x6029. */
	uint16_t cadence_comp_permille;
	uint16_t precomp_motor_power_w;
} assist_mode_output_t;

#define ASSIST_BANK_COUNT 2U
/*
 * FW-068/069: v6, 13 B header + 5x46 B + CRC = 245 B.
 *
 * HARD CEILING: the multiframe write protocol carries the total length in ONE byte
 * (CAN_Display.c rx_data_length, taken from rx_data[0]), and send_multiframe() takes a
 * uint8_t length. A blob above 255 B cannot be transferred at all - the write would fail
 * on CRC with no useful error. That is why the three FW-068 fields are u8 on the wire
 * (0..100 mV fits; the rise window travels in 10 ms units) even though they are u16 here.
 *
 * Only 10 B of headroom are left (2 B per level). The next per-level field beyond that
 * needs a 16-bit length on both sides of the protocol first.
 */
#define ASSIST_BANK_BLOB_LEN 245U
/* Wire resolution of start_rise_window_ms: stored in ms, transferred in 10 ms units. */
#define ASSIST_START_RISE_WINDOW_WIRE_STEP_MS 10U

/* FW-068 per-level start condition limits. */
#define ASSIST_START_LOAD_REDUCTION_MAX_MV 100U
#define ASSIST_START_RISE_MAX_MV 100U
#define ASSIST_START_RISE_WINDOW_MAX_MS 2000U
#define ASSIST_START_RISE_WINDOW_DEFAULT_MS 400U
/* FW-069 per-level Iq ramp limits (same range the global tuning blob used). */
#define ASSIST_RAMP_MS_MIN 20U
#define ASSIST_RAMP_MS_MAX 5000U

const assist_level_config_t *assist_modes_get_default_level(uint8_t level_index);

void assist_modes_init(void);
void assist_modes_set_active_bank(uint8_t bank_index);
uint8_t assist_modes_get_active_bank(void);

/* FW-043: Walk Assist cut-off wheel speed of the active bank, in 0.01 km/h units
 * (same scale as MS.Speedx100). Configurable per bank from Canable. */
uint16_t assist_modes_get_wa_max_wheel_x100(void);
/* FW-060 compatibility field: still serialized for older Canable, ignored by WA control. */
uint8_t assist_modes_get_wa_current_pct(void);
uint8_t assist_modes_get_wa_target_rpm(void);
bool assist_modes_get_wa_latch_after_release(void);
uint8_t assist_modes_get_wa_latch_timeout_s(void);
void assist_modes_seed_wa_defaults(uint8_t current_pct, uint16_t target_rpm);

/* FW-057: cadence compensation on/off for the active bank. */
bool assist_modes_get_cadence_comp_enabled(void);

uint16_t assist_modes_serialize_bank(uint8_t bank_index, uint8_t *buffer);
bool assist_modes_apply_bank_blob(const uint8_t *buffer, uint16_t length);

void assist_modes_reset(void);

bool assist_modes_calculate(
	const rider_input_t *input,
	const assist_level_config_t *config,
	uint32_t battery_voltage_mv,
	int32_t iq_limit,
	assist_mode_output_t *output);

const assist_mode_output_t *assist_modes_get_last_output(void);

#endif /* ASSIST_MODES_H_ */
