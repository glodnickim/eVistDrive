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

/*
 * Power ratios are based on the TSDZ2 reference factors 50/100/160/260,
 * where factor 50 means 1.0x rider power. SPORT+ is the midpoint between
 * SPORT and BOOST. These are provisional developer defaults; Legacy remains
 * the boot default until the versioned configuration is enabled.
 */
static const assist_level_config_t default_levels[ASSIST_LEVEL_COUNT + 1] = {
	{ASSIST_MODE_POWER_LINEAR, 0, 0, 0, false, 18,
		{false, ASSIST_STARTUP_BOOST_CADENCE, 0, 45}, {false, 300}, 0},
	{ASSIST_MODE_POWER_LINEAR, 100, 0, 100, false, 18,
		{true, ASSIST_STARTUP_BOOST_CADENCE, 200, 45}, {false, 300}, 0},
	{ASSIST_MODE_POWER_LINEAR, 200, 0, 100, false, 18,
		{true, ASSIST_STARTUP_BOOST_CADENCE, 200, 45}, {false, 300}, 0},
	{ASSIST_MODE_POWER_LINEAR, 320, 0, 100, false, 18,
		{true, ASSIST_STARTUP_BOOST_CADENCE, 200, 45}, {false, 300}, 0},
	{ASSIST_MODE_POWER_LINEAR, 420, 0, 100, false, 18,
		{true, ASSIST_STARTUP_BOOST_CADENCE, 200, 45}, {false, 300}, 0},
	{ASSIST_MODE_POWER_LINEAR, 520, 0, 100, false, 18,
		{true, ASSIST_STARTUP_BOOST_CADENCE, 200, 45}, {false, 300}, 0}
};

static assist_mode_output_t last_output;

static void clear_output(assist_mode_output_t *output)
{
	output->human_power_w = 0;
	output->assist_basis_power_w = 0;
	output->motor_power_w = 0;
	output->requested_battery_current_ma = 0;
	output->iq_request = 0;
	output->cadence_for_assist_rpm = 0;
	output->assist_without_rotation_active = false;
	output->torque_for_assist_mv = 0;
	output->startup_boost_extra_pct = 0;
	output->startup_boost_active = false;
}

static uint16_t torque_delta_from_corrected(const rider_input_t *input)
{
	if (input->torque_corrected_mv <= ASSIST_TORQUE_ZERO_MV) {
		return 0;
	}
	return (uint16_t)(input->torque_corrected_mv - ASSIST_TORQUE_ZERO_MV);
}

static bool calculate_power_linear(
	const rider_input_t *input,
	const assist_level_config_t *config,
	uint32_t battery_voltage_mv,
	int32_t iq_limit,
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
			/*
			 * Keep the synthetic cadence local to the selected assist mode.
			 * The shared rider snapshot and Legacy cadence stay unchanged.
			 */
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
		cadence_for_assist == 0 ||
		torque_for_assist == 0 ||
		battery_voltage_mv == 0 ||
		iq_limit <= 0 ||
		config->support_ratio_pct == 0 ||
		config->max_iq_pct == 0) {
		return true;
	}

	uint16_t human_torque_mv = torque_for_assist;
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
	torque_for_assist = boost_output.torque_output_mv;
	output->torque_for_assist_mv = torque_for_assist;
	output->startup_boost_extra_pct = boost_output.extra_pct;
	output->startup_boost_active = boost_output.active;

	uint32_t human_power_numerator =
		(uint32_t)human_torque_mv *
		(uint32_t)cadence_for_assist *
		HUMAN_POWER_NUMERATOR_SCALE;
	uint32_t human_power_mw =
		human_power_numerator / HUMAN_POWER_MW_DENOMINATOR;
	uint32_t assist_basis_power_numerator =
		(uint32_t)torque_for_assist *
		(uint32_t)cadence_for_assist *
		HUMAN_POWER_NUMERATOR_SCALE;
	uint32_t assist_basis_power_mw =
		assist_basis_power_numerator / HUMAN_POWER_MW_DENOMINATOR;

	uint32_t support_ratio_pct = config->support_ratio_pct;
	if (support_ratio_pct > ASSIST_SUPPORT_RATIO_MAX_PCT) {
		support_ratio_pct = ASSIST_SUPPORT_RATIO_MAX_PCT;
	}
	uint32_t motor_power_mw =
		(assist_basis_power_mw * support_ratio_pct) / 100U;
	uint32_t power_limit_w = config->max_motor_power_w;
	if (power_limit_w == 0 || power_limit_w > ASSIST_MOTOR_POWER_HARD_MAX_W) {
		power_limit_w = ASSIST_MOTOR_POWER_HARD_MAX_W;
	}
	uint32_t power_limit_mw = power_limit_w * 1000U;
	if (motor_power_mw > power_limit_mw) {
		motor_power_mw = power_limit_mw;
	}

	/*
	 * The TSDZ2 Power mode converts requested motor power to battery current
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
	uint32_t motor_power_w = motor_power_mw / 1000U;
	output->human_power_w = (human_power_w > UINT16_MAX) ?
		UINT16_MAX : (uint16_t)human_power_w;
	output->assist_basis_power_w = (assist_basis_power_w > UINT16_MAX) ?
		UINT16_MAX : (uint16_t)assist_basis_power_w;
	output->motor_power_w = (uint16_t)motor_power_w;
	output->requested_battery_current_ma = requested_current_ma;
	output->iq_request = iq_request;
	return true;
}

const assist_level_config_t *assist_modes_get_default_level(uint8_t level_index)
{
	if (level_index > ASSIST_LEVEL_COUNT) {
		level_index = 0;
	}
	return &default_levels[level_index];
}

void assist_modes_reset(void)
{
	assist_start_reset();
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
		supported = calculate_power_linear(
			input,
			config,
			battery_voltage_mv,
			iq_limit,
			output);
		break;
	case ASSIST_MODE_LEGACY:
	case ASSIST_MODE_POWER_PROGRESSIVE:
	case ASSIST_MODE_EMTB_TSDZ:
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
