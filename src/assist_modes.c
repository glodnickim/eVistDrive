#include "assist_modes.h"

#include "config.h"

#define ASSIST_LEVEL_COUNT 5
#define ASSIST_MOTOR_POWER_HARD_MAX_W 1500U
#define HUMAN_POWER_NUMERATOR_SCALE 342U
#define HUMAN_POWER_DENOMINATOR 100000U

/*
 * Power ratios are based on the TSDZ2 reference factors 50/100/160/260,
 * where factor 50 means 1.0x rider power. SPORT+ is the midpoint between
 * SPORT and BOOST. These are provisional developer defaults; Legacy remains
 * the boot default until the versioned configuration is enabled.
 */
static const assist_level_config_t default_levels[ASSIST_LEVEL_COUNT + 1] = {
	{ASSIST_MODE_POWER_LINEAR, 0, 0, 0},
	{ASSIST_MODE_POWER_LINEAR, 100, 0, 100},
	{ASSIST_MODE_POWER_LINEAR, 200, 0, 100},
	{ASSIST_MODE_POWER_LINEAR, 320, 0, 100},
	{ASSIST_MODE_POWER_LINEAR, 420, 0, 100},
	{ASSIST_MODE_POWER_LINEAR, 520, 0, 100}
};

static assist_mode_output_t last_output;

static void clear_output(assist_mode_output_t *output)
{
	output->human_power_w = 0;
	output->motor_power_w = 0;
	output->requested_battery_current_ma = 0;
	output->iq_request = 0;
}

static bool calculate_power_linear(
	const rider_input_t *input,
	const assist_level_config_t *config,
	uint32_t battery_voltage_mv,
	int32_t iq_limit,
	assist_mode_output_t *output)
{
	if (!input->torque_sensor_valid ||
		!input->pas_sensor_valid ||
		!input->pedaling_active ||
		input->cadence_rpm == 0 ||
		input->torque_filtered == 0 ||
		battery_voltage_mv == 0 ||
		iq_limit <= 0 ||
		config->support_ratio_pct == 0 ||
		config->max_iq_pct == 0) {
		return true;
	}

	uint32_t human_power_numerator =
		(uint32_t)input->torque_filtered *
		(uint32_t)input->cadence_rpm *
		HUMAN_POWER_NUMERATOR_SCALE;
	uint32_t human_power_w =
		human_power_numerator / HUMAN_POWER_DENOMINATOR;

	uint32_t motor_power_w =
		(human_power_w * (uint32_t)config->support_ratio_pct) / 100U;
	uint32_t power_limit_w = config->max_motor_power_w;
	if (power_limit_w == 0 || power_limit_w > ASSIST_MOTOR_POWER_HARD_MAX_W) {
		power_limit_w = ASSIST_MOTOR_POWER_HARD_MAX_W;
	}
	if (motor_power_w > power_limit_w) {
		motor_power_w = power_limit_w;
	}

	/*
	 * The TSDZ2 Power mode converts requested motor power to battery current
	 * using P/U. EBICS controls phase Iq, so the current request is converted
	 * to native Iq units with CAL_I and then clamped. At low PWM duty this is
	 * deliberately conservative: no duty-cycle compensation is injected into
	 * the rider mode.
	 */
	uint32_t requested_current_ma =
		(motor_power_w * 1000000U) / battery_voltage_mv;
	int32_t iq_request = (int32_t)(requested_current_ma / CAL_I);
	int32_t profile_iq_limit =
		(iq_limit * (int32_t)config->max_iq_pct) / 100;
	if (profile_iq_limit < 0) {
		profile_iq_limit = 0;
	}
	if (iq_request > profile_iq_limit) {
		iq_request = profile_iq_limit;
	}

	output->human_power_w = (human_power_w > UINT16_MAX) ?
		UINT16_MAX : (uint16_t)human_power_w;
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
