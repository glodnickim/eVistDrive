#include "assist_modes.h"

#include "cadence_comp.h"
#include "config.h"
#include "power_curve.h"
#include "torque_input.h"

#define ASSIST_LEVEL_COUNT 5
#define ASSIST_MOTOR_POWER_HARD_MAX_W 1500U
#define ASSIST_SUPPORT_RATIO_MAX_PCT 1000U
#define ASSIST_WITHOUT_ROTATION_THRESHOLD_MAX_MV 300U
#define HUMAN_POWER_CENTIKG_RPM_NUMERATOR 1694U
#define HUMAN_POWER_CENTIKG_RPM_DENOMINATOR 1000U
#define RIDE_CORE_FULL_IQ_SUPPORT_PCT 500U
#define RIDE_CORE_DEMAND_PERMILLE_MAX 1000U
#define MOTOR_VOLTAGE_UTILIZATION_SCALE 2048U
#define CONTROL_TICKS_PER_MS 4U
#define POWER_FILTER_MAX_MS 3000U
#define POWER_FILTER_Q_SHIFT 8U
#define PROGRESSIVE_REFERENCE_POWER_MIN_W 50U
#define PROGRESSIVE_REFERENCE_POWER_MAX_W 500U
#define PROGRESSION_MAX_PCT 100U
#define EMTB_TORQUE_RANGE 160U
#define EMTB_PARAMETER_MAX 250U
#define EMTB_DENOMINATOR_BASE 510U
#define EMTB_DENOMINATOR_MIN 10U
#define EMTB_REFERENCE_VOLTAGE_MIN_MV 24000U
#define EMTB_REFERENCE_VOLTAGE_MAX_MV 60000U
#define EMTB_UNIT_CURRENT_MA 160U
#define TORQUE_ASSIST_FACTOR_DENOMINATOR 120U
#define EMTB_FIXED_Q_SHIFT 8U
#define EMTB_FIXED_Q_ONE (1U << EMTB_FIXED_Q_SHIFT)

/*
 * Power ratios follow the reference factors 50/100/160/260,
 * where factor 50 means 1.0x rider power. SPORT+ is the midpoint between
 * SPORT and BOOST. These are provisional developer defaults; Legacy remains
 * the boot default until the versioned configuration is enabled.
 */
#define DEFAULT_POWER_LEVEL(mode, ratio, emtb_level, torque_factor) { \
	.mode_type = (mode), \
	.support_ratio_pct = (ratio), \
	.support_min_pct = (ratio), \
	.support_max_pct = (ratio), \
	.reference_power_w = 200, \
	.progression_pct = 0, \
	.curve_exponent_x10 = POWER_CURVE_EXP_DEFAULT_X10, \
	.curve_exponent_high_x10 = POWER_CURVE_EXP_DEFAULT_X10, \
	.emtb_parameter = (emtb_level), \
	.emtb_based_on_power = true, \
	.emtb_reference_voltage_mv = 36000, \
	.torque_assist_factor = (torque_factor), \
	.max_motor_power_w = 0, \
	.max_iq_pct = 100, \
	.assist_without_rotation = false, \
	.without_rotation_threshold_mv = 18, \
	.startup_boost = {true, ASSIST_STARTUP_BOOST_CADENCE, 100, 27}, \
	.smooth_start = {false, 300}, \
	.release_ms = 650, \
	.power_rise_filter_ms = 150, \
	.power_fall_filter_ms = 375, \
	.start_load_reduction_mv = 0, \
	.start_rise_mv = 0, \
	.start_rise_window_ms = ASSIST_START_RISE_WINDOW_DEFAULT_MS, \
	.iq_rise_slow_ms = 600, \
	.iq_rise_fast_ms = 300, \
	.iq_fall_slow_ms = 1000, \
	.iq_fall_fast_ms = 140 \
}

#define DEFAULT_IDLE_LEVEL { \
	.mode_type = ASSIST_MODE_POWER_LINEAR, \
	.reference_power_w = 200, \
	.curve_exponent_x10 = POWER_CURVE_EXP_DEFAULT_X10, \
	.curve_exponent_high_x10 = POWER_CURVE_EXP_DEFAULT_X10, \
	.emtb_based_on_power = true, \
	.emtb_reference_voltage_mv = 36000, \
	.without_rotation_threshold_mv = 18, \
	.startup_boost = {false, ASSIST_STARTUP_BOOST_CADENCE, 0, 45}, \
	.smooth_start = {false, 300}, \
	.start_rise_window_ms = ASSIST_START_RISE_WINDOW_DEFAULT_MS, \
	/* FW-069: level 0 never assists, but the shared Iq ramp still runs through it while \
	 * the current fades out after a level change to 0. Zero here would mean "no ramp". */ \
	.iq_rise_slow_ms = 600, \
	.iq_rise_fast_ms = 300, \
	.iq_fall_slow_ms = 1000, \
	.iq_fall_fast_ms = 140 \
}

static const assist_level_config_t default_levels[ASSIST_LEVEL_COUNT + 1] = {
	DEFAULT_IDLE_LEVEL,
	DEFAULT_POWER_LEVEL(ASSIST_MODE_POWER_LINEAR, 100, 60, 50),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_POWER_LINEAR, 200, 100, 80),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_POWER_LINEAR, 320, 140, 120),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_POWER_LINEAR, 420, 160, 160),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_POWER_LINEAR, 520, 180, 200)
};

static const assist_level_config_t emtb_levels[ASSIST_LEVEL_COUNT + 1] = {
	DEFAULT_IDLE_LEVEL,
	DEFAULT_POWER_LEVEL(ASSIST_MODE_EMTB, 100, 60, 50),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_EMTB, 200, 100, 80),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_EMTB, 320, 140, 120),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_EMTB, 420, 160, 160),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_EMTB, 520, 180, 200)
};

#undef DEFAULT_POWER_LEVEL
#undef DEFAULT_IDLE_LEVEL

static const assist_level_config_t *const bank_defaults[ASSIST_BANK_COUNT] = {
	default_levels,
	emtb_levels
};

static assist_level_config_t bank_config[ASSIST_BANK_COUNT][ASSIST_LEVEL_COUNT + 1];
static uint8_t active_bank;

/*
 * FW-043: per-bank Walk Assist cut-off wheel speed, in units of 0.1 km/h (70 = 7.0 km/h).
 * Rides in the ONE spare byte of the bank blob header (buffer[7], previously always 0), so the
 * blob length, CRC position and EEPROM layout are all unchanged. A stored 0 means "old blob,
 * not set" and maps to the default — old saved banks keep working without a reset.
 */
/* FW-051 supersedes the wire detail above: v2 uses a 10 B header and 187 B. */
#define BANK_WA_MAX_WHEEL_X10_DEFAULT 70U
#define BANK_WA_MAX_WHEEL_X10_MIN     10U   /* 1.0 km/h */
#define BANK_WA_MAX_WHEEL_X10_MAX     255U  /* 25.5 km/h */
#define BANK_WA_CURRENT_DEFAULT       30U
#define BANK_WA_CURRENT_MIN           1U
#define BANK_WA_CURRENT_MAX           100U
#define BANK_WA_TARGET_RPM_DEFAULT    50U
#define BANK_WA_TARGET_RPM_MIN        20U
#define BANK_WA_TARGET_RPM_MAX        60U
#define BANK_WA_LATCH_DEFAULT         0U
#define BANK_WA_LATCH_TIMEOUT_DEFAULT 30U
#define BANK_WA_LATCH_TIMEOUT_MIN     1U
#define BANK_WA_LATCH_TIMEOUT_MAX     120U
static uint8_t bank_wa_max_wheel_x10[ASSIST_BANK_COUNT];
static uint8_t bank_wa_current_pct[ASSIST_BANK_COUNT];
static uint8_t bank_wa_target_rpm[ASSIST_BANK_COUNT];
static uint8_t bank_wa_latch_after_release[ASSIST_BANK_COUNT];
static uint8_t bank_wa_latch_timeout_s[ASSIST_BANK_COUNT];
/* FW-057: cadence compensation on/off, one setting per bank. */
static uint8_t bank_cadence_comp_enabled[ASSIST_BANK_COUNT];
#define BANK_CADENCE_COMP_DEFAULT 0U

#define BANK_BLOB_MAGIC0 0x45U
#define BANK_BLOB_MAGIC1 0x42U
#define BANK_BLOB_VERSION_V1 1U
#define BANK_BLOB_VERSION_V2 2U
#define BANK_BLOB_VERSION_V3 3U
/* FW-056: v4 has the exact same layout and length as v3. The version byte is
 * purely a capability marker so Canable knows this firmware understands
 * ASSIST_MODE_POWER_CURVE and may offer it; old firmware never sees a v4 blob
 * because Canable only sends v4 to a controller that reported v4. */
#define BANK_BLOB_VERSION_V4 4U
/* FW-057: v5 adds header byte 12 = cadence compensation on/off for this bank.
 * 190 B still fits bank_store[2][192], BankBlob[192] and the 24-frame limit. */
#define BANK_BLOB_VERSION_V5 5U
/* FW-068/069: v6 is the first version to GROW THE RECORD (35 -> 56 B). Everything up to v5
 * assumed one compile-time record length, which is why buffer[5] used to be compared against
 * it instead of being used. From here on buffer[5] is the actual stride, so a shorter (older)
 * record is read field by field and the tail is backfilled - growing the record no longer
 * silently discards the user's whole profile configuration. */
#define BANK_BLOB_VERSION_V6 6U
#define BANK_BLOB_VERSION BANK_BLOB_VERSION_V6
#define BANK_BLOB_HEADER_LEN_V1 8U
#define BANK_BLOB_HEADER_LEN_V2 10U
#define BANK_BLOB_HEADER_LEN_V3 12U
#define BANK_BLOB_HEADER_LEN 13U
#define BANK_RECORD_LEN_V5 35U
/* 46 = 35 + 3 (FW-068 start condition, u8 each) + 8 (FW-069 four u16 ramps).
 * Kept deliberately tight: see the 255 B protocol ceiling in assist_modes.h. */
#define BANK_RECORD_LEN 46U

static uint16_t bank_blob_crc16(const uint8_t *buffer, uint16_t length)
{
	uint16_t crc = 0xFFFFU;
	for (uint16_t i = 0; i < length; i++) {
		crc ^= (uint16_t)buffer[i] << 8;
		for (uint8_t bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000U) ?
				(uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

static bool bank_mode_valid(uint8_t mode)
{
	return mode == ASSIST_MODE_POWER_LINEAR ||
		mode == ASSIST_MODE_POWER_PROGRESSIVE ||
		mode == ASSIST_MODE_EMTB ||
		mode == ASSIST_MODE_TORQUE ||
		mode == ASSIST_MODE_POWER_CURVE; //FW-056
}

static uint16_t clamp_u16(uint16_t value, uint16_t min, uint16_t max)
{
	if (value < min) {
		return min;
	}
	return (value > max) ? max : value;
}

static uint8_t valid_curve_exponent_x10(uint8_t value) //FW-056
{
	return (value < POWER_CURVE_EXP_MIN_X10 || value > POWER_CURVE_EXP_MAX_X10) ?
		POWER_CURVE_EXP_DEFAULT_X10 : value;
}

static uint8_t valid_wa_current_pct(uint8_t value)
{
	return (value >= BANK_WA_CURRENT_MIN && value <= BANK_WA_CURRENT_MAX) ?
		value : BANK_WA_CURRENT_DEFAULT;
}

static uint8_t valid_wa_target_rpm(uint16_t value)
{
	return (value >= BANK_WA_TARGET_RPM_MIN && value <= BANK_WA_TARGET_RPM_MAX) ?
		(uint8_t)value : BANK_WA_TARGET_RPM_DEFAULT;
}

static uint8_t valid_wa_max_wheel_x10(uint8_t value)
{
	return (value >= BANK_WA_MAX_WHEEL_X10_MIN &&
		value <= BANK_WA_MAX_WHEEL_X10_MAX) ?
		value : BANK_WA_MAX_WHEEL_X10_DEFAULT;
}

static uint8_t valid_wa_latch_timeout_s(uint8_t value)
{
	return (value >= BANK_WA_LATCH_TIMEOUT_MIN &&
		value <= BANK_WA_LATCH_TIMEOUT_MAX) ?
		value : BANK_WA_LATCH_TIMEOUT_DEFAULT;
}

static assist_mode_output_t last_output;

typedef struct {
	const assist_level_config_t *config_address;
	uint16_t rise_ms;
	uint16_t fall_ms;
	uint32_t filtered_power_q;
} power_filter_state_t;

typedef struct {
	uint8_t cadence_for_assist_rpm;
	uint16_t human_load_centikg;
	uint16_t assist_load_centikg;
	uint16_t torque_for_assist_mv;
	bool without_rotation_active;
	bool cadence_seeded;
} prepared_assist_input_t;

static power_filter_state_t power_filter_state;

static void clear_output(assist_mode_output_t *output)
{
	output->human_power_w = 0;
	output->assist_basis_power_w = 0;
	output->raw_motor_power_w = 0;
	output->motor_power_w = 0;
	output->applied_support_ratio_pct = 0;
	output->requested_battery_current_ma = 0;
	output->iq_request = 0;
	output->cadence_for_assist_rpm = 0;
	output->assist_without_rotation_active = false;
	output->torque_for_assist_mv = 0;
	output->startup_boost_extra_pct = 0;
	output->startup_boost_active = false;
	output->emtb_denominator = 0;
	output->emtb_target_x160 = 0;
	output->curve_input_permille = 0;
	output->curve_output_permille = 0;
	output->cadence_comp_permille = CADENCE_COMP_UNITY_PERMILLE;
	output->precomp_motor_power_w = 0;
}

static uint16_t clamp_power_filter_ms(uint16_t filter_ms)
{
	return (filter_ms > POWER_FILTER_MAX_MS) ?
		POWER_FILTER_MAX_MS : filter_ms;
}

static void stop_power_filter(const assist_level_config_t *config)
{
	power_filter_state.config_address = config;
	power_filter_state.rise_ms = clamp_power_filter_ms(
		config->power_rise_filter_ms);
	power_filter_state.fall_ms = clamp_power_filter_ms(
		config->power_fall_filter_ms);
	power_filter_state.filtered_power_q = 0;
}

static uint32_t filter_motor_power(
	uint32_t raw_motor_power_mw,
	const assist_level_config_t *config)
{
	uint16_t rise_ms = clamp_power_filter_ms(config->power_rise_filter_ms);
	uint16_t fall_ms = clamp_power_filter_ms(config->power_fall_filter_ms);
	uint32_t raw_power_q = raw_motor_power_mw << POWER_FILTER_Q_SHIFT;

	if (power_filter_state.config_address != config ||
		power_filter_state.rise_ms != rise_ms ||
		power_filter_state.fall_ms != fall_ms) {
		power_filter_state.config_address = config;
		power_filter_state.rise_ms = rise_ms;
		power_filter_state.fall_ms = fall_ms;
		power_filter_state.filtered_power_q = raw_power_q;
		return raw_motor_power_mw;
	}

	uint16_t filter_ms = (raw_power_q > power_filter_state.filtered_power_q) ?
		rise_ms : fall_ms;
	if (filter_ms == 0 || raw_power_q == power_filter_state.filtered_power_q) {
		power_filter_state.filtered_power_q = raw_power_q;
		return raw_motor_power_mw;
	}

	uint32_t filter_ticks = (uint32_t)filter_ms * CONTROL_TICKS_PER_MS;
	if (raw_power_q > power_filter_state.filtered_power_q) {
		uint32_t delta = raw_power_q - power_filter_state.filtered_power_q;
		uint32_t step = delta / filter_ticks;
		if (step == 0) step = 1;
		power_filter_state.filtered_power_q += step;
	} else {
		uint32_t delta = power_filter_state.filtered_power_q - raw_power_q;
		uint32_t step = delta / filter_ticks;
		if (step == 0) step = 1;
		power_filter_state.filtered_power_q -= step;
	}

	return (power_filter_state.filtered_power_q +
		(1U << (POWER_FILTER_Q_SHIFT - 1U))) >> POWER_FILTER_Q_SHIFT;
}

/*
 * Shared by POWER_PROGRESSIVE and POWER_CURVE: clamp the support window and the
 * reference power, then express the rider's power as 0..1000 permille of that
 * reference. Both modes must validate identically, so this lives in one place.
 */
static uint16_t normalize_power_support_bounds(
	uint32_t assist_basis_power_mw,
	const assist_level_config_t *config,
	uint16_t *support_min_pct,
	uint16_t *support_max_pct)
{
	uint16_t min_pct = config->support_min_pct;
	uint16_t max_pct = config->support_max_pct;
	if (min_pct > ASSIST_SUPPORT_RATIO_MAX_PCT) {
		min_pct = ASSIST_SUPPORT_RATIO_MAX_PCT;
	}
	if (max_pct > ASSIST_SUPPORT_RATIO_MAX_PCT) {
		max_pct = ASSIST_SUPPORT_RATIO_MAX_PCT;
	}
	if (max_pct < min_pct) {
		max_pct = min_pct;
	}
	*support_min_pct = min_pct;
	*support_max_pct = max_pct;

	uint16_t reference_power_w = config->reference_power_w;
	if (reference_power_w < PROGRESSIVE_REFERENCE_POWER_MIN_W) {
		reference_power_w = PROGRESSIVE_REFERENCE_POWER_MIN_W;
	} else if (reference_power_w > PROGRESSIVE_REFERENCE_POWER_MAX_W) {
		reference_power_w = PROGRESSIVE_REFERENCE_POWER_MAX_W;
	}
	uint32_t input_permille = assist_basis_power_mw / reference_power_w;
	return (input_permille > 1000U) ? 1000U : (uint16_t)input_permille;
}

static uint16_t apply_support_window(
	uint16_t support_min_pct,
	uint16_t support_max_pct,
	uint16_t curve_permille)
{
	return (uint16_t)(support_min_pct +
		((uint32_t)(support_max_pct - support_min_pct) * curve_permille) /
		1000U);
}

static uint16_t calculate_power_linear_support_pct(
	const assist_level_config_t *config)
{
	uint16_t ratio = config->support_ratio_pct;
	return (ratio > ASSIST_SUPPORT_RATIO_MAX_PCT) ?
		ASSIST_SUPPORT_RATIO_MAX_PCT : ratio;
}

static uint16_t calculate_power_progressive_support_pct(
	uint32_t assist_basis_power_mw,
	const assist_level_config_t *config)
{
	uint16_t support_min_pct;
	uint16_t support_max_pct;
	uint16_t input_permille = normalize_power_support_bounds(
		assist_basis_power_mw, config, &support_min_pct, &support_max_pct);

	uint8_t progression_pct = config->progression_pct;
	if (progression_pct > PROGRESSION_MAX_PCT) {
		progression_pct = PROGRESSION_MAX_PCT;
	}
	uint32_t curve_permille =
		((uint32_t)(100U - progression_pct) * input_permille +
		 ((uint32_t)progression_pct * input_permille * input_permille) / 1000U) /
		100U;

	return apply_support_window(
		support_min_pct, support_max_pct, (uint16_t)curve_permille);
}

/*
 * FW-056: support = support_min + (support_max - support_min) * curve(x), where
 * x is the rider's power relative to reference_power_w. The support window is
 * split in half and each half gets its own exponent:
 *
 *   lower half  x 0 .. 1/2 -> support_min .. window middle, shaped by gamma low
 *   upper half  x 1/2 .. 1 -> window middle .. support_max, shaped by gamma high
 *
 * Both halves are evaluated on the full-scale table, so each one keeps the whole
 * curve resolution. Equal exponents on both halves give a symmetric S shape, and
 * 1.0 on both gives exactly the straight line. The curve only shapes the support
 * ratio; every downstream limit, filter and ramp stays untouched.
 */
static uint16_t power_curve_shape_permille(
	uint16_t input_permille,
	uint8_t exponent_low_x10,
	uint8_t exponent_high_x10)
{
	if (input_permille <= 500U) {
		uint16_t half = power_curve_eval_permille(
			(uint16_t)(input_permille * 2U), exponent_low_x10);
		return (uint16_t)((half + 1U) / 2U);
	}
	uint16_t half = power_curve_eval_permille(
		(uint16_t)((input_permille - 500U) * 2U), exponent_high_x10);
	return (uint16_t)(500U + (half + 1U) / 2U);
}

static uint16_t calculate_power_curve_support_pct(
	uint32_t assist_basis_power_mw,
	const assist_level_config_t *config,
	assist_mode_output_t *output)
{
	uint16_t support_min_pct;
	uint16_t support_max_pct;
	uint16_t input_permille = normalize_power_support_bounds(
		assist_basis_power_mw, config, &support_min_pct, &support_max_pct);

	uint16_t curve_permille = power_curve_shape_permille(
		input_permille,
		config->curve_exponent_x10,
		config->curve_exponent_high_x10);

	if (output != 0) {
		output->curve_input_permille = input_permille;
		output->curve_output_permille = curve_permille;
	}

	return apply_support_window(
		support_min_pct, support_max_pct, curve_permille);
}

/*
 * Explicit switch on purpose: with three Power modes an "if not Linear then
 * Progressive" shortcut would silently run the wrong algorithm.
 */
static uint16_t calculate_support_ratio_pct(
	uint32_t assist_basis_power_mw,
	const assist_level_config_t *config,
	assist_mode_output_t *output)
{
	switch (config->mode_type) {
	case ASSIST_MODE_POWER_LINEAR:
		return calculate_power_linear_support_pct(config);
	case ASSIST_MODE_POWER_PROGRESSIVE:
		return calculate_power_progressive_support_pct(
			assist_basis_power_mw, config);
	case ASSIST_MODE_POWER_CURVE:
		return calculate_power_curve_support_pct(
			assist_basis_power_mw, config, output);
	default:
		return 0;
	}
}

static uint32_t calculate_human_power_mw(uint16_t load_centikg, uint8_t cadence_rpm)
{
	/*
	 * P = m * g * crank_length * 2*pi*rpm/60. For a 165 mm crank
	 * this is 1.694 mW per (0.01 kg * rpm). Split 1.694 into 1 + 0.694
	 * so every intermediate stays in uint32_t at 120 kg and 255 rpm.
	 */
	uint32_t product = (uint32_t)load_centikg * cadence_rpm;
	return product + (product *
		(HUMAN_POWER_CENTIKG_RPM_NUMERATOR -
		 HUMAN_POWER_CENTIKG_RPM_DENOMINATOR) +
		HUMAN_POWER_CENTIKG_RPM_DENOMINATOR / 2U) /
		HUMAN_POWER_CENTIKG_RPM_DENOMINATOR;
}

static int32_t calculate_load_iq_request(
	uint16_t load_centikg,
	uint16_t support_ratio_pct,
	int32_t iq_limit)
{
	if (iq_limit <= 0 || load_centikg == 0 || support_ratio_pct == 0) {
		return 0;
	}
	if (load_centikg > TORQUE_PUBLIC_FULL_SCALE_CENTIKG) {
		load_centikg = TORQUE_PUBLIC_FULL_SCALE_CENTIKG;
	}
	if (support_ratio_pct > ASSIST_SUPPORT_RATIO_MAX_PCT) {
		support_ratio_pct = ASSIST_SUPPORT_RATIO_MAX_PCT;
	}
	/* 60 kg at 500% support is the conservative full-Iq reference. */
	uint32_t demand_permille =
		((uint32_t)load_centikg * support_ratio_pct + 1500U) / 3000U;
	if (demand_permille > RIDE_CORE_DEMAND_PERMILLE_MAX) {
		demand_permille = RIDE_CORE_DEMAND_PERMILLE_MAX;
	}
	if (demand_permille == 0) {
		return 0;
	}
	return (int32_t)(((uint32_t)iq_limit * demand_permille +
		RIDE_CORE_DEMAND_PERMILLE_MAX - 1U) /
		RIDE_CORE_DEMAND_PERMILLE_MAX);
}

static int32_t calculate_target_x160_iq_request(
	uint32_t target_x160_q,
	int32_t iq_limit)
{
	if (iq_limit <= 0 || target_x160_q == 0) {
		return 0;
	}
	uint32_t full_scale_q = EMTB_TORQUE_RANGE * EMTB_FIXED_Q_ONE;
	if (target_x160_q > full_scale_q) {
		target_x160_q = full_scale_q;
	}
	return (int32_t)((target_x160_q * (uint32_t)iq_limit +
		full_scale_q - 1U) / full_scale_q);
}

static bool prepare_assist_input(
	const rider_input_t *input,
	const assist_level_config_t *config,
	prepared_assist_input_t *prepared,
	assist_mode_output_t *output)
{
	uint8_t cadence_for_assist = input->cadence_rpm;
	// FW-033: RUN power/eMTB/torque and the startup boost use the slow RUN estimator
	// so they no longer amplify each individual leg peak. The without-rotation start
	// branch below overrides this back to the fast signal (standstill launch).
	uint16_t torque_for_assist = input->torque_run_filtered;
	bool without_rotation_active = false;

	if (config->assist_without_rotation &&
		cadence_for_assist == 0 &&
		input->torque_sensor_valid &&
		input->pas_sensor_valid) {
		uint16_t threshold_mv = config->without_rotation_threshold_mv;
		if (threshold_mv > ASSIST_WITHOUT_ROTATION_THRESHOLD_MAX_MV) {
			threshold_mv = ASSIST_WITHOUT_ROTATION_THRESHOLD_MAX_MV;
		}
		uint16_t corrected_delta_mv = input->torque_assist_filtered;
		if (corrected_delta_mv > threshold_mv) {
			/* Keep synthetic cadence local; never modify MS or rider_input. */
			cadence_for_assist = 1;
			torque_for_assist = corrected_delta_mv;
			without_rotation_active = true;
		}
	}
	{
		uint16_t torque_range = torque_input_span_native();
		if (torque_for_assist > torque_range) {
			torque_for_assist = torque_range;
		}
	}

	output->cadence_for_assist_rpm = cadence_for_assist;
	output->assist_without_rotation_active = without_rotation_active;
	if (!input->torque_sensor_valid ||
		!input->pas_sensor_valid ||
		(!input->pedaling_active && !without_rotation_active) ||
		cadence_for_assist == 0) {
		return false;
	}

	prepared->cadence_for_assist_rpm = cadence_for_assist;
	prepared->human_load_centikg = input->torque_load_centikg;
	prepared->without_rotation_active = without_rotation_active;
	prepared->cadence_seeded = input->cadence_seeded;

	assist_startup_boost_input_t boost_input = {
		.torque_input_mv = torque_for_assist,
		/* The temporary 18 rpm seed arms the mode but is not real cadence. */
		.cadence_for_assist_rpm = input->cadence_seeded ? 0U : cadence_for_assist,
		.wheel_speed_x100 = input->wheel_speed_x100,
		.torque_sensor_valid = input->torque_sensor_valid
	};
	assist_startup_boost_output_t boost_output;
	assist_start_apply_boost(
		&boost_input,
		&config->startup_boost,
		&boost_output);
	prepared->torque_for_assist_mv = boost_output.torque_output_mv;
	prepared->assist_load_centikg = torque_input_native_delta_to_centikg(
		boost_output.torque_output_mv);
	output->torque_for_assist_mv = boost_output.torque_output_mv;
	output->startup_boost_extra_pct = boost_output.extra_pct;
	output->startup_boost_active = boost_output.active;
	return true;
}

static bool finish_power_request(
	const rider_input_t *input,
	const assist_level_config_t *config,
	uint32_t battery_voltage_mv,
	int32_t iq_limit,
	uint32_t human_power_mw,
	uint32_t assist_basis_power_mw,
	uint32_t support_ratio_pct,
	uint32_t motor_power_mw,
	int32_t phase_iq_request,
	assist_mode_output_t *output)
{
	/*
	 * FW-057: cadence compensation. Applied here because all three pedalling
	 * modes (Power, eMTB, Torque) funnel through this function, and here is
	 * after the base assist is known but before the motor-power ceiling, the Iq
	 * ceiling, the P/U voltage ceiling, the power filter and the final ramp.
	 *
	 * Not applied to the throttle floor or Walk Assist: both are added after
	 * assist_modes_calculate() in ride_control, so they never reach this code.
	 * Braking zeroes the target there too. The artificial start-up cadence seed
	 * is excluded explicitly — it is not a cadence anyone is pedalling at.
	 */
	uint16_t cadence_comp_permille = CADENCE_COMP_UNITY_PERMILLE;
	if (bank_cadence_comp_enabled[active_bank] && !input->cadence_seeded) {
		cadence_comp_permille =
			cadence_comp_multiplier_permille(input->cadence_rpm);
	}
	uint32_t precomp_motor_power_mw = motor_power_mw;
	if (cadence_comp_permille != CADENCE_COMP_UNITY_PERMILLE) {
		motor_power_mw = (motor_power_mw * cadence_comp_permille) /
			CADENCE_COMP_UNITY_PERMILLE;
		if (phase_iq_request > 0) {
			phase_iq_request = (int32_t)
				(((uint32_t)phase_iq_request * cadence_comp_permille) /
				CADENCE_COMP_UNITY_PERMILLE);
		}
	}
	output->cadence_comp_permille = cadence_comp_permille;
	uint32_t precomp_motor_power_w = precomp_motor_power_mw / 1000U;
	output->precomp_motor_power_w = (precomp_motor_power_w > UINT16_MAX) ?
		UINT16_MAX : (uint16_t)precomp_motor_power_w;

	uint32_t power_limit_w = config->max_motor_power_w;
	if (power_limit_w == 0 || power_limit_w > ASSIST_MOTOR_POWER_HARD_MAX_W) {
		power_limit_w = ASSIST_MOTOR_POWER_HARD_MAX_W;
	}
	uint32_t power_limit_mw = power_limit_w * 1000U;
	if (motor_power_mw > power_limit_mw) {
		motor_power_mw = power_limit_mw;
	}
	uint32_t raw_motor_power_mw = motor_power_mw;
	motor_power_mw = filter_motor_power(raw_motor_power_mw, config);

	/* P/U is battery current. Convert it to a phase-current ceiling using
	 * EBICS' measured voltage utilization (duty, scale 0..2048). At launch
	 * duty is near zero, so the torque-derived Iq request remains authoritative;
	 * as the motor accelerates the power ceiling becomes effective. */
	uint32_t requested_current_ma =
		(motor_power_mw * 1000U) / battery_voltage_mv;
	int32_t profile_iq_limit =
		(iq_limit * (int32_t)config->max_iq_pct) / 100;
	if (profile_iq_limit < 0) {
		profile_iq_limit = 0;
	}
	if (phase_iq_request < 0) {
		phase_iq_request = 0;
	}
	if (phase_iq_request > profile_iq_limit) {
		phase_iq_request = profile_iq_limit;
	}
	if (!input->cadence_seeded && requested_current_ma > 0U &&
		input->motor_voltage_utilization > 0U) {
		uint32_t power_iq_limit =
			(requested_current_ma * MOTOR_VOLTAGE_UTILIZATION_SCALE) /
			((uint32_t)input->motor_voltage_utilization * CAL_I);
		if ((uint32_t)phase_iq_request > power_iq_limit) {
			phase_iq_request = (int32_t)power_iq_limit;
		}
	}

	uint32_t human_power_w = human_power_mw / 1000U;
	uint32_t assist_basis_power_w = assist_basis_power_mw / 1000U;
	uint32_t raw_motor_power_w = raw_motor_power_mw / 1000U;
	uint32_t motor_power_w = motor_power_mw / 1000U;
	output->human_power_w = (human_power_w > UINT16_MAX) ?
		UINT16_MAX : (uint16_t)human_power_w;
	output->assist_basis_power_w = (assist_basis_power_w > UINT16_MAX) ?
		UINT16_MAX : (uint16_t)assist_basis_power_w;
	output->raw_motor_power_w = (uint16_t)raw_motor_power_w;
	output->motor_power_w = (uint16_t)motor_power_w;
	output->applied_support_ratio_pct = (support_ratio_pct > UINT16_MAX) ?
		UINT16_MAX : (uint16_t)support_ratio_pct;
	output->requested_battery_current_ma = requested_current_ma;
	output->iq_request = phase_iq_request;
	return true;
}

static bool calculate_power(
	const rider_input_t *input,
	const assist_level_config_t *config,
	uint32_t battery_voltage_mv,
	int32_t iq_limit,
	assist_mode_output_t *output)
{
	bool support_disabled;
	switch (config->mode_type) { //FW-056: explicit per-mode "no assist" test
	case ASSIST_MODE_POWER_LINEAR:
		support_disabled = config->support_ratio_pct == 0;
		break;
	case ASSIST_MODE_POWER_PROGRESSIVE:
	case ASSIST_MODE_POWER_CURVE:
		support_disabled = config->support_max_pct == 0;
		break;
	default:
		support_disabled = true;
		break;
	}

	prepared_assist_input_t prepared;
	if (!prepare_assist_input(input, config, &prepared, output) ||
		battery_voltage_mv == 0 ||
		iq_limit <= 0 ||
		support_disabled ||
		config->max_iq_pct == 0) {
		stop_power_filter(config);
		return true;
	}

	uint8_t power_cadence = prepared.cadence_seeded ?
		0U : prepared.cadence_for_assist_rpm;
	uint32_t human_power_mw = calculate_human_power_mw(
		prepared.human_load_centikg, power_cadence);
	uint32_t assist_basis_power_mw = calculate_human_power_mw(
		prepared.assist_load_centikg, power_cadence);

	uint32_t support_ratio_pct = calculate_support_ratio_pct(
		assist_basis_power_mw,
		config,
		output);
	uint32_t motor_power_mw =
		(assist_basis_power_mw * support_ratio_pct) / 100U;
	int32_t phase_iq_request = calculate_load_iq_request(
		prepared.assist_load_centikg,
		(uint16_t)support_ratio_pct,
		iq_limit);
	return finish_power_request(
		input,
		config,
		battery_voltage_mv,
		iq_limit,
		human_power_mw,
		assist_basis_power_mw,
		support_ratio_pct,
		motor_power_mw,
		phase_iq_request,
		output);
}

/*
 * eMTB assist: assist target derived from the SQUARE of the pedal-load delta, so the
 * response is gentle at light effort and steepens as you push.
 * denominator = 510 - 2*parameter, reduced by the
 * cadence when the mode is power based, floored at +10; progressive target
 * = delta^2 / denominator over the 0..160 torque range. The curve stays
 * in Q8 until it is converted directly to EBICS phase Iq, avoiding the old
 * integer dead zone and the invalid battery-current-to-Iq assignment.
 */
static bool calculate_emtb(
	const rider_input_t *input,
	const assist_level_config_t *config,
	uint32_t battery_voltage_mv,
	int32_t iq_limit,
	assist_mode_output_t *output)
{
	prepared_assist_input_t prepared;
	if (!prepare_assist_input(input, config, &prepared, output) ||
		battery_voltage_mv == 0 ||
		iq_limit <= 0 ||
		config->emtb_parameter == 0 ||
		config->max_iq_pct == 0) {
		stop_power_filter(config);
		return true;
	}

	uint32_t parameter = config->emtb_parameter;
	if (parameter > EMTB_PARAMETER_MAX) {
		parameter = EMTB_PARAMETER_MAX;
	}
	uint32_t reference_voltage_mv = config->emtb_reference_voltage_mv;
	if (reference_voltage_mv < EMTB_REFERENCE_VOLTAGE_MIN_MV) {
		reference_voltage_mv = EMTB_REFERENCE_VOLTAGE_MIN_MV;
	}
	if (reference_voltage_mv > EMTB_REFERENCE_VOLTAGE_MAX_MV) {
		reference_voltage_mv = EMTB_REFERENCE_VOLTAGE_MAX_MV;
	}

	uint32_t torque_range = torque_input_span_native();
	uint32_t delta_x160_q =
		((uint32_t)prepared.torque_for_assist_mv * EMTB_TORQUE_RANGE *
		EMTB_FIXED_Q_ONE +
		torque_range / 2U) / torque_range;

	uint32_t denominator = EMTB_DENOMINATOR_BASE - 2U * parameter;
	if (config->emtb_based_on_power) {
		uint32_t cadence = prepared.cadence_seeded ?
			0U : prepared.cadence_for_assist_rpm;
		denominator = (denominator > cadence) ? denominator - cadence : 0U;
	}
	denominator += EMTB_DENOMINATOR_MIN;

	uint32_t target_x160_q =
		(delta_x160_q * delta_x160_q) /
		(denominator * EMTB_FIXED_Q_ONE);

	uint8_t power_cadence = prepared.cadence_seeded ?
		0U : prepared.cadence_for_assist_rpm;
	uint32_t human_power_mw = calculate_human_power_mw(
		prepared.human_load_centikg, power_cadence);
	uint32_t assist_basis_power_mw = calculate_human_power_mw(
		prepared.assist_load_centikg, power_cadence);

	uint32_t target_for_power_q = target_x160_q;
	uint32_t target_full_scale_q =
		EMTB_TORQUE_RANGE * EMTB_FIXED_Q_ONE;
	if (target_for_power_q > target_full_scale_q) {
		target_for_power_q = target_full_scale_q;
	}
	uint32_t target_current_ma =
		(target_for_power_q * EMTB_UNIT_CURRENT_MA +
		EMTB_FIXED_Q_ONE / 2U) / EMTB_FIXED_Q_ONE;
	uint32_t motor_power_mw =
		(target_current_ma * reference_voltage_mv) / 1000U;
	uint32_t support_ratio_pct = (assist_basis_power_mw > 0U) ?
		(uint32_t)(((uint64_t)motor_power_mw * 100U) /
		assist_basis_power_mw) : 0U;
	int32_t phase_iq_request = calculate_target_x160_iq_request(
		target_x160_q, iq_limit);

	output->emtb_denominator = (uint16_t)denominator;
	uint32_t target_x160_display =
		(target_x160_q + EMTB_FIXED_Q_ONE - 1U) / EMTB_FIXED_Q_ONE;
	output->emtb_target_x160 = (target_x160_display > UINT16_MAX) ?
		UINT16_MAX : (uint16_t)target_x160_display;

	return finish_power_request(
		input,
		config,
		battery_voltage_mv,
		iq_limit,
		human_power_mw,
		assist_basis_power_mw,
		support_ratio_pct,
		motor_power_mw,
		phase_iq_request,
		output);
}

/*
 * Torque assist: assist target proportional to the pedal-load delta.
 * Target current = torque delta * factor / 120 in
 * the 0..160 range. Q8 preserves small requests before conversion to
 * EBICS phase Iq. Cadence gates the assist but does not scale it.
 */
static bool calculate_torque_assist(
	const rider_input_t *input,
	const assist_level_config_t *config,
	uint32_t battery_voltage_mv,
	int32_t iq_limit,
	assist_mode_output_t *output)
{
	prepared_assist_input_t prepared;
	if (!prepare_assist_input(input, config, &prepared, output) ||
		battery_voltage_mv == 0 ||
		iq_limit <= 0 ||
		config->torque_assist_factor == 0 ||
		config->max_iq_pct == 0) {
		stop_power_filter(config);
		return true;
	}

	uint32_t reference_voltage_mv = config->emtb_reference_voltage_mv;
	if (reference_voltage_mv < EMTB_REFERENCE_VOLTAGE_MIN_MV) {
		reference_voltage_mv = EMTB_REFERENCE_VOLTAGE_MIN_MV;
	}
	if (reference_voltage_mv > EMTB_REFERENCE_VOLTAGE_MAX_MV) {
		reference_voltage_mv = EMTB_REFERENCE_VOLTAGE_MAX_MV;
	}

	uint32_t torque_range = torque_input_span_native();
	uint32_t delta_x160_q =
		((uint32_t)prepared.torque_for_assist_mv * EMTB_TORQUE_RANGE *
		EMTB_FIXED_Q_ONE +
		torque_range / 2U) / torque_range;
	uint32_t target_x160_q =
		(delta_x160_q * config->torque_assist_factor) /
		TORQUE_ASSIST_FACTOR_DENOMINATOR;

	uint8_t power_cadence = prepared.cadence_seeded ?
		0U : prepared.cadence_for_assist_rpm;
	uint32_t human_power_mw = calculate_human_power_mw(
		prepared.human_load_centikg, power_cadence);
	uint32_t assist_basis_power_mw = calculate_human_power_mw(
		prepared.assist_load_centikg, power_cadence);

	uint32_t target_for_power_q = target_x160_q;
	uint32_t target_full_scale_q =
		EMTB_TORQUE_RANGE * EMTB_FIXED_Q_ONE;
	if (target_for_power_q > target_full_scale_q) {
		target_for_power_q = target_full_scale_q;
	}
	uint32_t target_current_ma =
		(target_for_power_q * EMTB_UNIT_CURRENT_MA +
		EMTB_FIXED_Q_ONE / 2U) / EMTB_FIXED_Q_ONE;
	uint32_t motor_power_mw =
		(target_current_ma * reference_voltage_mv) / 1000U;
	uint32_t support_ratio_pct = (assist_basis_power_mw > 0U) ?
		(uint32_t)(((uint64_t)motor_power_mw * 100U) /
		assist_basis_power_mw) : 0U;
	int32_t phase_iq_request = calculate_target_x160_iq_request(
		target_x160_q, iq_limit);

	uint32_t target_x160_display =
		(target_x160_q + EMTB_FIXED_Q_ONE - 1U) / EMTB_FIXED_Q_ONE;
	output->emtb_target_x160 = (target_x160_display > UINT16_MAX) ?
		UINT16_MAX : (uint16_t)target_x160_display;

	return finish_power_request(
		input,
		config,
		battery_voltage_mv,
		iq_limit,
		human_power_mw,
		assist_basis_power_mw,
		support_ratio_pct,
		motor_power_mw,
		phase_iq_request,
		output);
}

const assist_level_config_t *assist_modes_get_default_level(uint8_t level_index)
{
	if (level_index > ASSIST_LEVEL_COUNT) {
		level_index = 0;
	}
	return &bank_config[active_bank][level_index];
}

void assist_modes_init(void)
{
	for (uint8_t bank = 0; bank < ASSIST_BANK_COUNT; bank++) {
		bank_wa_max_wheel_x10[bank] = BANK_WA_MAX_WHEEL_X10_DEFAULT; //FW-043
		bank_wa_current_pct[bank] = BANK_WA_CURRENT_DEFAULT;
		bank_wa_target_rpm[bank] = BANK_WA_TARGET_RPM_DEFAULT;
		bank_wa_latch_after_release[bank] = BANK_WA_LATCH_DEFAULT;
		bank_wa_latch_timeout_s[bank] = BANK_WA_LATCH_TIMEOUT_DEFAULT;
		bank_cadence_comp_enabled[bank] = BANK_CADENCE_COMP_DEFAULT; //FW-057
		for (uint8_t level = 0; level <= ASSIST_LEVEL_COUNT; level++) {
			bank_config[bank][level] = bank_defaults[bank][level];
		}
	}
}

void assist_modes_seed_wa_defaults(uint8_t current_pct, uint16_t target_rpm)
{
	uint8_t current = valid_wa_current_pct(current_pct);
	uint8_t rpm = valid_wa_target_rpm(target_rpm);
	for (uint8_t bank = 0; bank < ASSIST_BANK_COUNT; bank++) {
		bank_wa_current_pct[bank] = current;
		bank_wa_target_rpm[bank] = rpm;
	}
}

//FW-043: Walk Assist cut-off wheel speed of the ACTIVE bank, in 0.01 km/h (matches MS.Speedx100).
//Single source of truth: both the pushassist_flag gate in main.c and the walk module read this,
//so the threshold can no longer drift apart between the two places it used to be hard-coded in.
uint16_t assist_modes_get_wa_max_wheel_x100(void)
{
	uint8_t value = valid_wa_max_wheel_x10(bank_wa_max_wheel_x10[active_bank]);
	return (uint16_t)value * 10U;
}

uint8_t assist_modes_get_wa_current_pct(void)
{
	/* FW-060: compatibility/API only; the constant-RPM controller does not read it. */
	return valid_wa_current_pct(bank_wa_current_pct[active_bank]);
}

uint8_t assist_modes_get_wa_target_rpm(void)
{
	return valid_wa_target_rpm(bank_wa_target_rpm[active_bank]);
}

bool assist_modes_get_wa_latch_after_release(void)
{
	return bank_wa_latch_after_release[active_bank] != 0U;
}

uint8_t assist_modes_get_wa_latch_timeout_s(void)
{
	return valid_wa_latch_timeout_s(bank_wa_latch_timeout_s[active_bank]);
}

bool assist_modes_get_cadence_comp_enabled(void) //FW-057
{
	return bank_cadence_comp_enabled[active_bank] != 0U;
}

void assist_modes_set_active_bank(uint8_t bank_index)
{
	if (bank_index >= ASSIST_BANK_COUNT) {
		bank_index = 0;
	}
	if (bank_index != active_bank) {
		active_bank = bank_index;
		assist_modes_reset();
	}
}

uint8_t assist_modes_get_active_bank(void)
{
	return active_bank;
}

static void put_u16(uint8_t *buffer, uint16_t value)
{
	buffer[0] = (uint8_t)(value & 0xFFU);
	buffer[1] = (uint8_t)(value >> 8);
}

static uint16_t get_u16(const uint8_t *buffer)
{
	return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
}

uint16_t assist_modes_serialize_bank(uint8_t bank_index, uint8_t *buffer)
{
	if (bank_index >= ASSIST_BANK_COUNT || buffer == 0) {
		return 0;
	}
	buffer[0] = BANK_BLOB_MAGIC0;
	buffer[1] = BANK_BLOB_MAGIC1;
	buffer[2] = BANK_BLOB_VERSION;
	buffer[3] = bank_index;
	buffer[4] = ASSIST_LEVEL_COUNT;
	buffer[5] = BANK_RECORD_LEN;
	buffer[6] = active_bank;
	buffer[7] = valid_wa_max_wheel_x10(bank_wa_max_wheel_x10[bank_index]);
	/* FW-060: retain byte 8 for backward compatibility; new WA ignores it. */
	buffer[8] = valid_wa_current_pct(bank_wa_current_pct[bank_index]);
	buffer[9] = valid_wa_target_rpm(bank_wa_target_rpm[bank_index]);
	buffer[10] = bank_wa_latch_after_release[bank_index] ? 1U : 0U;
	buffer[11] = valid_wa_latch_timeout_s(bank_wa_latch_timeout_s[bank_index]);
	buffer[12] = bank_cadence_comp_enabled[bank_index] ? 1U : 0U; //FW-057
	uint8_t *record = &buffer[BANK_BLOB_HEADER_LEN];
	for (uint8_t level = 1; level <= ASSIST_LEVEL_COUNT; level++) {
		const assist_level_config_t *cfg = &bank_config[bank_index][level];
		record[0] = (uint8_t)cfg->mode_type;
		if (cfg->mode_type == ASSIST_MODE_POWER_CURVE) { //FW-056
			/* support_ratio_pct belongs to POWER_LINEAR only, so its two bytes
			 * carry the upper-half exponent here. Byte 2 stays reserved. */
			record[1] = cfg->curve_exponent_high_x10;
			record[2] = 0;
		} else {
			put_u16(&record[1], cfg->support_ratio_pct);
		}
		put_u16(&record[3], cfg->support_min_pct);
		put_u16(&record[5], cfg->support_max_pct);
		put_u16(&record[7], cfg->reference_power_w);
		/* FW-056: byte 9 carries gamma for POWER_CURVE, progression otherwise.
		 * The two shapes belong to different modes and never coexist, so the
		 * record stays 35 B and the blob stays 189 B. */
		record[9] = (cfg->mode_type == ASSIST_MODE_POWER_CURVE) ?
			cfg->curve_exponent_x10 : cfg->progression_pct;
		record[10] = cfg->emtb_parameter;
		record[11] = cfg->emtb_based_on_power ? 1U : 0U;
		put_u16(&record[12], cfg->emtb_reference_voltage_mv);
		record[14] = cfg->torque_assist_factor;
		put_u16(&record[15], cfg->max_motor_power_w);
		record[17] = cfg->max_iq_pct;
		record[18] = cfg->assist_without_rotation ? 1U : 0U;
		put_u16(&record[19], cfg->without_rotation_threshold_mv);
		record[21] = cfg->startup_boost.enabled ? 1U : 0U;
		record[22] = (uint8_t)cfg->startup_boost.mode;
		put_u16(&record[23], cfg->startup_boost.strength_pct);
		record[25] = cfg->startup_boost.end_rpm;
		record[26] = cfg->smooth_start.enabled ? 1U : 0U;
		put_u16(&record[27], cfg->smooth_start.duration_ms);
		put_u16(&record[29], cfg->release_ms);
		put_u16(&record[31], cfg->power_rise_filter_ms);
		put_u16(&record[33], cfg->power_fall_filter_ms);
		//FW-068: u8 on the wire (0..100 mV), window in 10 ms units - the blob has to
		//stay under the 255 B multiframe ceiling, see assist_modes.h.
		record[35] = (uint8_t)cfg->start_load_reduction_mv;
		record[36] = (uint8_t)cfg->start_rise_mv;
		record[37] = (uint8_t)(cfg->start_rise_window_ms /
			ASSIST_START_RISE_WINDOW_WIRE_STEP_MS);
		put_u16(&record[38], cfg->iq_rise_slow_ms);          //FW-069
		put_u16(&record[40], cfg->iq_rise_fast_ms);
		put_u16(&record[42], cfg->iq_fall_slow_ms);
		put_u16(&record[44], cfg->iq_fall_fast_ms);
		record += BANK_RECORD_LEN;
	}
	uint16_t crc_at = BANK_BLOB_HEADER_LEN +
		(uint16_t)ASSIST_LEVEL_COUNT * BANK_RECORD_LEN;
	put_u16(&buffer[crc_at], bank_blob_crc16(buffer, crc_at));
	return ASSIST_BANK_BLOB_LEN;
}

bool assist_modes_apply_bank_blob(const uint8_t *buffer, uint16_t length)
{
	if (buffer == 0 ||
		length < BANK_BLOB_HEADER_LEN_V1 ||
		buffer[0] != BANK_BLOB_MAGIC0 ||
		buffer[1] != BANK_BLOB_MAGIC1 ||
		buffer[3] >= ASSIST_BANK_COUNT ||
		buffer[4] != ASSIST_LEVEL_COUNT) {
		return false;
	}
	/*
	 * FW-068/069: buffer[5] is the record STRIDE, not a constant to compare against. The old
	 * equality check meant that the first time the record ever grew, every previously stored
	 * bank was rejected and the whole profile configuration silently fell back to defaults.
	 * A shorter record is now read up to its own length and the tail is backfilled below.
	 */
	uint8_t record_len = buffer[5];
	if (record_len < BANK_RECORD_LEN_V5 || record_len > BANK_RECORD_LEN) {
		return false;
	}

	uint8_t version = buffer[2];
	uint16_t header_len;
	uint16_t expected_len;
	if (version == BANK_BLOB_VERSION_V1) {
		header_len = BANK_BLOB_HEADER_LEN_V1;
	} else if (version == BANK_BLOB_VERSION_V2) {
		header_len = BANK_BLOB_HEADER_LEN_V2;
	} else if (version == BANK_BLOB_VERSION_V3 ||
		version == BANK_BLOB_VERSION_V4) { //FW-056: v4 == v3 layout
		header_len = BANK_BLOB_HEADER_LEN_V3;
	} else if (version == BANK_BLOB_VERSION_V5 ||
		version == BANK_BLOB_VERSION_V6) { //FW-057 header; FW-068/069 longer record
		header_len = BANK_BLOB_HEADER_LEN;
	} else {
		return false;
	}

	expected_len = header_len + (uint16_t)ASSIST_LEVEL_COUNT * record_len + 2U;
	if (length < expected_len) {
		return false;
	}
	uint16_t crc_at = header_len + (uint16_t)ASSIST_LEVEL_COUNT * record_len;
	if (get_u16(&buffer[crc_at]) != bank_blob_crc16(buffer, crc_at)) {
		return false;
	}
	const uint8_t *record = &buffer[header_len];
	for (uint8_t level = 1; level <= ASSIST_LEVEL_COUNT; level++) {
		if (!bank_mode_valid(record[0])) {
			return false;
		}
		record += record_len;
	}
	uint8_t bank_index = buffer[3];
	bank_wa_max_wheel_x10[bank_index] = valid_wa_max_wheel_x10(buffer[7]);
	if (version >= BANK_BLOB_VERSION_V2) {
		bank_wa_current_pct[bank_index] = valid_wa_current_pct(buffer[8]);
		bank_wa_target_rpm[bank_index] = valid_wa_target_rpm(buffer[9]);
	}
	bank_wa_latch_after_release[bank_index] = BANK_WA_LATCH_DEFAULT;
	bank_wa_latch_timeout_s[bank_index] = BANK_WA_LATCH_TIMEOUT_DEFAULT;
	if (version >= BANK_BLOB_VERSION_V3) {
		bank_wa_latch_after_release[bank_index] = buffer[10] ? 1U : 0U;
		bank_wa_latch_timeout_s[bank_index] =
			valid_wa_latch_timeout_s(buffer[11]);
	}
	/* FW-057: older blobs predate the setting, so it stays off after migration. */
	bank_cadence_comp_enabled[bank_index] = (version >= BANK_BLOB_VERSION_V5) ?
		(buffer[12] ? 1U : 0U) : BANK_CADENCE_COMP_DEFAULT;
	record = &buffer[header_len];
	for (uint8_t level = 1; level <= ASSIST_LEVEL_COUNT; level++) {
		assist_level_config_t *cfg = &bank_config[bank_index][level];
		cfg->mode_type = (assist_mode_type_t)record[0];
		if (cfg->mode_type == ASSIST_MODE_POWER_CURVE) { //FW-056
			cfg->support_ratio_pct = 0;
			cfg->curve_exponent_high_x10 = valid_curve_exponent_x10(record[1]);
		} else {
			cfg->support_ratio_pct =
				clamp_u16(get_u16(&record[1]), 0, ASSIST_SUPPORT_RATIO_MAX_PCT);
			cfg->curve_exponent_high_x10 = POWER_CURVE_EXP_DEFAULT_X10;
		}
		cfg->support_min_pct =
			clamp_u16(get_u16(&record[3]), 0, ASSIST_SUPPORT_RATIO_MAX_PCT);
		cfg->support_max_pct =
			clamp_u16(get_u16(&record[5]), 0, ASSIST_SUPPORT_RATIO_MAX_PCT);
		cfg->reference_power_w = clamp_u16(get_u16(&record[7]),
			PROGRESSIVE_REFERENCE_POWER_MIN_W,
			PROGRESSIVE_REFERENCE_POWER_MAX_W);
		if (cfg->mode_type == ASSIST_MODE_POWER_CURVE) { //FW-056
			cfg->progression_pct = 0;
			cfg->curve_exponent_x10 = valid_curve_exponent_x10(record[9]);
		} else {
			cfg->progression_pct = (record[9] > PROGRESSION_MAX_PCT) ?
				PROGRESSION_MAX_PCT : record[9];
			cfg->curve_exponent_x10 = POWER_CURVE_EXP_DEFAULT_X10;
		}
		cfg->emtb_parameter = (record[10] > EMTB_PARAMETER_MAX) ?
			EMTB_PARAMETER_MAX : record[10];
		cfg->emtb_based_on_power = record[11] != 0;
		cfg->emtb_reference_voltage_mv = clamp_u16(get_u16(&record[12]),
			EMTB_REFERENCE_VOLTAGE_MIN_MV, EMTB_REFERENCE_VOLTAGE_MAX_MV);
		cfg->torque_assist_factor = record[14];
		cfg->max_motor_power_w = clamp_u16(get_u16(&record[15]),
			0, ASSIST_MOTOR_POWER_HARD_MAX_W);
		cfg->max_iq_pct = (record[17] > 100U) ? 100U : record[17];
		cfg->assist_without_rotation = record[18] != 0;
		cfg->without_rotation_threshold_mv = clamp_u16(get_u16(&record[19]),
			0, ASSIST_WITHOUT_ROTATION_THRESHOLD_MAX_MV);
		cfg->startup_boost.enabled = record[21] != 0;
		cfg->startup_boost.mode = (record[22] > ASSIST_STARTUP_BOOST_AUTO) ?
			ASSIST_STARTUP_BOOST_CADENCE :
			(assist_startup_boost_mode_t)record[22];
		cfg->startup_boost.strength_pct =
			clamp_u16(get_u16(&record[23]), 0, 300U);
		cfg->startup_boost.end_rpm = (record[25] > 120U) ? 120U : record[25];
		cfg->smooth_start.enabled = record[26] != 0;
		cfg->smooth_start.duration_ms =
			clamp_u16(get_u16(&record[27]), 0, 5000U);
		cfg->release_ms = clamp_u16(get_u16(&record[29]), 0, 3000U);
		cfg->power_rise_filter_ms =
			clamp_u16(get_u16(&record[31]), 0, 3000U);
		cfg->power_fall_filter_ms =
			clamp_u16(get_u16(&record[33]), 0, 3000U);
		/*
		 * FW-068/069: fields past the v5 record. A shorter record means an older writer
		 * that never had them, so they take the compiled default of this level instead of
		 * whatever happens to sit past the end of the record.
		 */
		if (record_len >= BANK_RECORD_LEN) {
			cfg->start_load_reduction_mv = clamp_u16(record[35],
				0, ASSIST_START_LOAD_REDUCTION_MAX_MV);
			cfg->start_rise_mv = clamp_u16(record[36],
				0, ASSIST_START_RISE_MAX_MV);
			cfg->start_rise_window_ms = clamp_u16(
				(uint16_t)record[37] * ASSIST_START_RISE_WINDOW_WIRE_STEP_MS,
				0, ASSIST_START_RISE_WINDOW_MAX_MS);
			cfg->iq_rise_slow_ms = clamp_u16(get_u16(&record[38]),
				ASSIST_RAMP_MS_MIN, ASSIST_RAMP_MS_MAX);
			cfg->iq_rise_fast_ms = clamp_u16(get_u16(&record[40]),
				ASSIST_RAMP_MS_MIN, ASSIST_RAMP_MS_MAX);
			cfg->iq_fall_slow_ms = clamp_u16(get_u16(&record[42]),
				ASSIST_RAMP_MS_MIN, ASSIST_RAMP_MS_MAX);
			cfg->iq_fall_fast_ms = clamp_u16(get_u16(&record[44]),
				ASSIST_RAMP_MS_MIN, ASSIST_RAMP_MS_MAX);
		} else {
			const assist_level_config_t *fallback =
				&bank_defaults[bank_index][level];
			cfg->start_load_reduction_mv = fallback->start_load_reduction_mv;
			cfg->start_rise_mv = fallback->start_rise_mv;
			cfg->start_rise_window_ms = fallback->start_rise_window_ms;
			cfg->iq_rise_slow_ms = fallback->iq_rise_slow_ms;
			cfg->iq_rise_fast_ms = fallback->iq_rise_fast_ms;
			cfg->iq_fall_slow_ms = fallback->iq_fall_slow_ms;
			cfg->iq_fall_fast_ms = fallback->iq_fall_fast_ms;
		}
		record += record_len;
	}
	assist_modes_reset();
	return true;
}

void assist_modes_reset(void)
{
	assist_start_reset();
	power_filter_state = (power_filter_state_t){0};
	clear_output(&last_output);
}

bool assist_modes_calculate(
	const rider_input_t *input,
	const assist_level_config_t *config,
	uint32_t battery_voltage_mv,
	int32_t iq_limit,
	assist_mode_output_t *output)
{
	assist_mode_output_t local_output;
	if (output == 0) {
		output = &local_output;
	}
	clear_output(output);

	if (input == 0 || config == 0) {
		last_output = *output;
		return false;
	}

	bool supported = false;
	switch (config->mode_type) {
	case ASSIST_MODE_POWER_LINEAR:
	case ASSIST_MODE_POWER_PROGRESSIVE:
	case ASSIST_MODE_POWER_CURVE: //FW-056
		supported = calculate_power(
			input,
			config,
			battery_voltage_mv,
			iq_limit,
			output);
		break;
	case ASSIST_MODE_EMTB:
		supported = calculate_emtb(
			input,
			config,
			battery_voltage_mv,
			iq_limit,
			output);
		break;
	case ASSIST_MODE_TORQUE:
		supported = calculate_torque_assist(
			input,
			config,
			battery_voltage_mv,
			iq_limit,
			output);
		break;
	case ASSIST_MODE_LEGACY:
	case ASSIST_MODE_EMTB_CUSTOM:
	default:
		break;
	}

	last_output = *output;
	return supported;
}

const assist_mode_output_t *assist_modes_get_last_output(void)
{
	return &last_output;
}
