#include "assist_modes.h"

#include "config.h"

#define ASSIST_LEVEL_COUNT 5
#define ASSIST_MOTOR_POWER_HARD_MAX_W 1500U
#define ASSIST_SUPPORT_RATIO_MAX_PCT 1000U
#define ASSIST_WITHOUT_ROTATION_THRESHOLD_MAX_MV 300U
#define ASSIST_TORQUE_ZERO_MV 750
#define ASSIST_TORQUE_DELTA_MAX_MV 2550U
#define HUMAN_POWER_NUMERATOR_SCALE 342U
#define HUMAN_POWER_MW_DENOMINATOR 100U
#define CONTROL_TICKS_PER_MS 4U
#define POWER_FILTER_MAX_MS 3000U
#define POWER_FILTER_Q_SHIFT 8U
#define PROGRESSIVE_REFERENCE_POWER_MIN_W 50U
#define PROGRESSIVE_REFERENCE_POWER_MAX_W 500U
#define PROGRESSION_MAX_PCT 100U
#define EMTB_TSDZ_TORQUE_RANGE 160U
#define EMTB_TSDZ_PARAMETER_MAX 250U
#define EMTB_TSDZ_DENOMINATOR_BASE 510U
#define EMTB_TSDZ_DENOMINATOR_MIN 10U
#define EMTB_REFERENCE_VOLTAGE_MIN_MV 24000U
#define EMTB_REFERENCE_VOLTAGE_MAX_MV 60000U
#define EMTB_UNIT_CURRENT_MA 160U

/*
 * Power ratios are based on the TSDZ2 reference factors 50/100/160/260,
 * where factor 50 means 1.0x rider power. SPORT+ is the midpoint between
 * SPORT and BOOST. These are provisional developer defaults; Legacy remains
 * the boot default until the versioned configuration is enabled.
 */
#define DEFAULT_POWER_LEVEL(mode, ratio, emtb_level) { \
	.mode_type = (mode), \
	.support_ratio_pct = (ratio), \
	.support_min_pct = (ratio), \
	.support_max_pct = (ratio), \
	.reference_power_w = 200, \
	.progression_pct = 0, \
	.emtb_parameter = (emtb_level), \
	.emtb_based_on_power = true, \
	.emtb_reference_voltage_mv = 36000, \
	.max_motor_power_w = 0, \
	.max_iq_pct = 100, \
	.assist_without_rotation = false, \
	.without_rotation_threshold_mv = 18, \
	.startup_boost = {true, ASSIST_STARTUP_BOOST_CADENCE, 200, 45}, \
	.smooth_start = {false, 300}, \
	.release_ms = 0, \
	.power_rise_filter_ms = 0, \
	.power_fall_filter_ms = 0 \
}

#define DEFAULT_IDLE_LEVEL { \
	.mode_type = ASSIST_MODE_POWER_LINEAR, \
	.reference_power_w = 200, \
	.emtb_based_on_power = true, \
	.emtb_reference_voltage_mv = 36000, \
	.without_rotation_threshold_mv = 18, \
	.startup_boost = {false, ASSIST_STARTUP_BOOST_CADENCE, 0, 45}, \
	.smooth_start = {false, 300} \
}

static const assist_level_config_t default_levels[ASSIST_LEVEL_COUNT + 1] = {
	DEFAULT_IDLE_LEVEL,
	DEFAULT_POWER_LEVEL(ASSIST_MODE_POWER_LINEAR, 100, 60),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_POWER_LINEAR, 200, 100),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_POWER_LINEAR, 320, 140),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_POWER_LINEAR, 420, 160),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_POWER_LINEAR, 520, 180)
};

static const assist_level_config_t emtb_levels[ASSIST_LEVEL_COUNT + 1] = {
	DEFAULT_IDLE_LEVEL,
	DEFAULT_POWER_LEVEL(ASSIST_MODE_EMTB_TSDZ, 100, 60),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_EMTB_TSDZ, 200, 100),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_EMTB_TSDZ, 320, 140),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_EMTB_TSDZ, 420, 160),
	DEFAULT_POWER_LEVEL(ASSIST_MODE_EMTB_TSDZ, 520, 180)
};

#undef DEFAULT_POWER_LEVEL
#undef DEFAULT_IDLE_LEVEL

static const assist_level_config_t *const bank_levels[ASSIST_BANK_COUNT] = {
	default_levels,
	emtb_levels
};

static uint8_t active_bank;

static assist_mode_output_t last_output;

typedef struct {
	const assist_level_config_t *config_address;
	uint16_t rise_ms;
	uint16_t fall_ms;
	uint32_t filtered_power_q;
} power_filter_state_t;

typedef struct {
	uint8_t cadence_for_assist_rpm;
	uint16_t human_torque_mv;
	uint16_t torque_for_assist_mv;
	bool without_rotation_active;
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

static uint16_t calculate_support_ratio_pct(
	uint32_t assist_basis_power_mw,
	const assist_level_config_t *config)
{
	if (config->mode_type == ASSIST_MODE_POWER_LINEAR) {
		uint16_t ratio = config->support_ratio_pct;
		return (ratio > ASSIST_SUPPORT_RATIO_MAX_PCT) ?
			ASSIST_SUPPORT_RATIO_MAX_PCT : ratio;
	}

	uint16_t support_min_pct = config->support_min_pct;
	uint16_t support_max_pct = config->support_max_pct;
	if (support_min_pct > ASSIST_SUPPORT_RATIO_MAX_PCT) {
		support_min_pct = ASSIST_SUPPORT_RATIO_MAX_PCT;
	}
	if (support_max_pct > ASSIST_SUPPORT_RATIO_MAX_PCT) {
		support_max_pct = ASSIST_SUPPORT_RATIO_MAX_PCT;
	}
	if (support_max_pct < support_min_pct) {
		support_max_pct = support_min_pct;
	}

	uint16_t reference_power_w = config->reference_power_w;
	if (reference_power_w < PROGRESSIVE_REFERENCE_POWER_MIN_W) {
		reference_power_w = PROGRESSIVE_REFERENCE_POWER_MIN_W;
	} else if (reference_power_w > PROGRESSIVE_REFERENCE_POWER_MAX_W) {
		reference_power_w = PROGRESSIVE_REFERENCE_POWER_MAX_W;
	}
	uint32_t input_permille = assist_basis_power_mw / reference_power_w;
	if (input_permille > 1000U) input_permille = 1000U;

	uint8_t progression_pct = config->progression_pct;
	if (progression_pct > PROGRESSION_MAX_PCT) {
		progression_pct = PROGRESSION_MAX_PCT;
	}
	uint32_t curve_permille =
		((uint32_t)(100U - progression_pct) * input_permille +
		 ((uint32_t)progression_pct * input_permille * input_permille) / 1000U) /
		100U;

	return (uint16_t)(support_min_pct +
		((uint32_t)(support_max_pct - support_min_pct) * curve_permille) /
		1000U);
}

static uint16_t torque_delta_from_corrected(const rider_input_t *input)
{
	if (input->torque_corrected_mv <= ASSIST_TORQUE_ZERO_MV) {
		return 0;
	}
	return (uint16_t)(input->torque_corrected_mv - ASSIST_TORQUE_ZERO_MV);
}

static bool prepare_assist_input(
	const rider_input_t *input,
	const assist_level_config_t *config,
	prepared_assist_input_t *prepared,
	assist_mode_output_t *output)
{
	uint8_t cadence_for_assist = input->cadence_rpm;
	uint16_t torque_for_assist = input->torque_filtered;
	bool without_rotation_active = false;

	if (config->assist_without_rotation &&
		cadence_for_assist == 0 &&
		input->torque_sensor_valid &&
		input->pas_sensor_valid) {
		uint16_t threshold_mv = config->without_rotation_threshold_mv;
		if (threshold_mv > ASSIST_WITHOUT_ROTATION_THRESHOLD_MAX_MV) {
			threshold_mv = ASSIST_WITHOUT_ROTATION_THRESHOLD_MAX_MV;
		}
		uint16_t corrected_delta_mv = torque_delta_from_corrected(input);
		if (corrected_delta_mv > threshold_mv) {
			/* Keep synthetic cadence local; never modify MS or rider_input. */
			cadence_for_assist = 1;
			torque_for_assist = corrected_delta_mv;
			without_rotation_active = true;
		}
	}
	if (torque_for_assist > ASSIST_TORQUE_DELTA_MAX_MV) {
		torque_for_assist = ASSIST_TORQUE_DELTA_MAX_MV;
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
	prepared->human_torque_mv = torque_for_assist;
	prepared->without_rotation_active = without_rotation_active;

	assist_startup_boost_input_t boost_input = {
		.torque_input_mv = torque_for_assist,
		.cadence_for_assist_rpm = cadence_for_assist,
		.wheel_speed_x100 = input->wheel_speed_x100,
		.torque_sensor_valid = input->torque_sensor_valid
	};
	assist_startup_boost_output_t boost_output;
	assist_start_apply_boost(
		&boost_input,
		&config->startup_boost,
		&boost_output);
	prepared->torque_for_assist_mv = boost_output.torque_output_mv;
	output->torque_for_assist_mv = boost_output.torque_output_mv;
	output->startup_boost_extra_pct = boost_output.extra_pct;
	output->startup_boost_active = boost_output.active;
	return true;
}

static bool finish_power_request(
	const assist_level_config_t *config,
	uint32_t battery_voltage_mv,
	int32_t iq_limit,
	uint32_t human_power_mw,
	uint32_t assist_basis_power_mw,
	uint32_t support_ratio_pct,
	uint32_t motor_power_mw,
	assist_mode_output_t *output)
{
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

	/*
	 * The TSDZ2 modes convert requested motor power to battery current
	 * using P/U. EBICS controls phase Iq, so the current request is converted
	 * to native Iq units with CAL_I and then clamped. At low PWM duty this is
	 * deliberately conservative: no duty-cycle compensation is injected into
	 * the rider mode.
	 */
	uint32_t requested_current_ma =
		(motor_power_mw * 1000U) / battery_voltage_mv;
	int32_t iq_request = (int32_t)(requested_current_ma / CAL_I);
	int32_t profile_iq_limit =
		(iq_limit * (int32_t)config->max_iq_pct) / 100;
	if (profile_iq_limit < 0) {
		profile_iq_limit = 0;
	}
	if (iq_request > profile_iq_limit) {
		iq_request = profile_iq_limit;
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
	output->iq_request = iq_request;
	return true;
}

static bool calculate_power(
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
		(config->mode_type == ASSIST_MODE_POWER_LINEAR ?
			config->support_ratio_pct == 0 : config->support_max_pct == 0) ||
		config->max_iq_pct == 0) {
		stop_power_filter(config);
		return true;
	}

	uint32_t human_power_numerator =
		(uint32_t)prepared.human_torque_mv *
		(uint32_t)prepared.cadence_for_assist_rpm *
		HUMAN_POWER_NUMERATOR_SCALE;
	uint32_t human_power_mw =
		human_power_numerator / HUMAN_POWER_MW_DENOMINATOR;
	uint32_t assist_basis_power_numerator =
		(uint32_t)prepared.torque_for_assist_mv *
		(uint32_t)prepared.cadence_for_assist_rpm *
		HUMAN_POWER_NUMERATOR_SCALE;
	uint32_t assist_basis_power_mw =
		assist_basis_power_numerator / HUMAN_POWER_MW_DENOMINATOR;

	uint32_t support_ratio_pct = calculate_support_ratio_pct(
		assist_basis_power_mw,
		config);
	uint32_t motor_power_mw =
		(assist_basis_power_mw * support_ratio_pct) / 100U;
	return finish_power_request(
		config,
		battery_voltage_mv,
		iq_limit,
		human_power_mw,
		assist_basis_power_mw,
		support_ratio_pct,
		motor_power_mw,
		output);
}

/*
 * Faithful port of emmebrusa TSDZ2-Smart-EBike-1 apply_emtb_assist()
 * (src/ebike_app.c:950): denominator = 510 - 2*parameter, reduced by the
 * cadence when the mode is power based, floored at +10; progressive target
 * = delta^2 / denominator in the TSDZ 0..160 torque range; one TSDZ current
 * unit is 0.16 A and the request is normalized to the reference voltage so
 * the ride character does not follow battery sag.
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
	if (parameter > EMTB_TSDZ_PARAMETER_MAX) {
		parameter = EMTB_TSDZ_PARAMETER_MAX;
	}
	uint32_t reference_voltage_mv = config->emtb_reference_voltage_mv;
	if (reference_voltage_mv < EMTB_REFERENCE_VOLTAGE_MIN_MV) {
		reference_voltage_mv = EMTB_REFERENCE_VOLTAGE_MIN_MV;
	}
	if (reference_voltage_mv > EMTB_REFERENCE_VOLTAGE_MAX_MV) {
		reference_voltage_mv = EMTB_REFERENCE_VOLTAGE_MAX_MV;
	}

	uint32_t delta_x160 =
		((uint32_t)prepared.torque_for_assist_mv * EMTB_TSDZ_TORQUE_RANGE +
		ASSIST_TORQUE_DELTA_MAX_MV / 2U) / ASSIST_TORQUE_DELTA_MAX_MV;

	uint32_t denominator = EMTB_TSDZ_DENOMINATOR_BASE - 2U * parameter;
	if (config->emtb_based_on_power) {
		uint32_t cadence = prepared.cadence_for_assist_rpm;
		denominator = (denominator > cadence) ? denominator - cadence : 0U;
	}
	denominator += EMTB_TSDZ_DENOMINATOR_MIN;

	uint32_t target_x160 = (delta_x160 * delta_x160) / denominator;

	uint32_t human_power_numerator =
		(uint32_t)prepared.human_torque_mv *
		(uint32_t)prepared.cadence_for_assist_rpm *
		HUMAN_POWER_NUMERATOR_SCALE;
	uint32_t human_power_mw =
		human_power_numerator / HUMAN_POWER_MW_DENOMINATOR;
	uint32_t assist_basis_power_numerator =
		(uint32_t)prepared.torque_for_assist_mv *
		(uint32_t)prepared.cadence_for_assist_rpm *
		HUMAN_POWER_NUMERATOR_SCALE;
	uint32_t assist_basis_power_mw =
		assist_basis_power_numerator / HUMAN_POWER_MW_DENOMINATOR;

	uint32_t motor_power_mw =
		((target_x160 * reference_voltage_mv) / 1000U) *
		EMTB_UNIT_CURRENT_MA;
	uint32_t support_ratio_pct = (assist_basis_power_mw > 0U) ?
		(motor_power_mw * 100U) / assist_basis_power_mw : 0U;

	output->emtb_denominator = (uint16_t)denominator;
	output->emtb_target_x160 = (target_x160 > UINT16_MAX) ?
		UINT16_MAX : (uint16_t)target_x160;

	return finish_power_request(
		config,
		battery_voltage_mv,
		iq_limit,
		human_power_mw,
		assist_basis_power_mw,
		support_ratio_pct,
		motor_power_mw,
		output);
}

const assist_level_config_t *assist_modes_get_default_level(uint8_t level_index)
{
	if (level_index > ASSIST_LEVEL_COUNT) {
		level_index = 0;
	}
	return &bank_levels[active_bank][level_index];
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
		supported = calculate_power(
			input,
			config,
			battery_voltage_mv,
			iq_limit,
			output);
		break;
	case ASSIST_MODE_EMTB_TSDZ:
		supported = calculate_emtb(
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
