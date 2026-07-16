#include "assist_start.h"

#include <limits.h>

#define STARTUP_BOOST_CURVE_SIZE 120U
#define STARTUP_BOOST_CADENCE_STEP 20U
#define STARTUP_BOOST_STRENGTH_MAX_PCT 300U
#define STARTUP_BOOST_AUTO_TORQUE_MV 20U
#define ASSIST_TORQUE_DELTA_MAX_MV 2550U

static uint16_t startup_boost_curve[STARTUP_BOOST_CURVE_SIZE];
static uint16_t cached_strength_pct = UINT16_MAX;
static bool speed_boost_latched;
static assist_startup_boost_mode_t previous_mode = ASSIST_STARTUP_BOOST_CADENCE;

static void rebuild_startup_boost_curve(uint16_t strength_pct)
{
	if (strength_pct > STARTUP_BOOST_STRENGTH_MAX_PCT) {
		strength_pct = STARTUP_BOOST_STRENGTH_MAX_PCT;
	}
	if (strength_pct == cached_strength_pct) {
		return;
	}

	startup_boost_curve[0] = strength_pct;
	for (uint8_t cadence = 1; cadence < STARTUP_BOOST_CURVE_SIZE; cadence++) {
		uint32_t previous = startup_boost_curve[cadence - 1];
		uint32_t next =
			(previous << 8) - (previous * STARTUP_BOOST_CADENCE_STEP);
		startup_boost_curve[cadence] = (uint16_t)(next >> 8);
	}
	cached_strength_pct = strength_pct;
}

void assist_start_reset(void)
{
	speed_boost_latched = false;
	previous_mode = ASSIST_STARTUP_BOOST_CADENCE;
}

void assist_start_apply_boost(
	const assist_startup_boost_input_t *input,
	const assist_startup_boost_config_t *config,
	assist_startup_boost_output_t *output)
{
	if (output == 0) {
		return;
	}
	output->torque_output_mv = 0;
	output->extra_pct = 0;
	output->active = false;
	if (input == 0 || config == 0) {
		return;
	}

	uint16_t torque_input_mv = input->torque_input_mv;
	if (torque_input_mv > ASSIST_TORQUE_DELTA_MAX_MV) {
		torque_input_mv = ASSIST_TORQUE_DELTA_MAX_MV;
	}
	output->torque_output_mv = torque_input_mv;

	if (!config->enabled ||
		!input->torque_sensor_valid ||
		config->strength_pct == 0) {
		speed_boost_latched = false;
		return;
	}

	if (config->mode != previous_mode) {
		speed_boost_latched = false;
		previous_mode = config->mode;
	}

	bool boost_enabled = false;
	switch (config->mode) {
	case ASSIST_STARTUP_BOOST_CADENCE:
		boost_enabled = true;
		break;
	case ASSIST_STARTUP_BOOST_SPEED:
		if (input->wheel_speed_x100 == 0) {
			speed_boost_latched = true;
		} else if (input->cadence_for_assist_rpm > config->end_rpm) {
			speed_boost_latched = false;
		}
		boost_enabled = speed_boost_latched;
		break;
	case ASSIST_STARTUP_BOOST_AUTO:
		boost_enabled = !(
			torque_input_mv < STARTUP_BOOST_AUTO_TORQUE_MV &&
			input->wheel_speed_x100 > 0);
		break;
	default:
		speed_boost_latched = false;
		return;
	}

	if (!boost_enabled ||
		input->cadence_for_assist_rpm > config->end_rpm) {
		return;
	}

	rebuild_startup_boost_curve(config->strength_pct);
	uint8_t curve_index = input->cadence_for_assist_rpm;
	if (curve_index >= STARTUP_BOOST_CURVE_SIZE) {
		curve_index = STARTUP_BOOST_CURVE_SIZE - 1;
	}
	uint16_t extra_pct = startup_boost_curve[curve_index];
	if (extra_pct == 0) {
		return;
	}

	uint32_t boosted_torque = torque_input_mv +
		((uint32_t)torque_input_mv * extra_pct) / 100U;
	if (boosted_torque > ASSIST_TORQUE_DELTA_MAX_MV) {
		boosted_torque = ASSIST_TORQUE_DELTA_MAX_MV;
	}

	output->torque_output_mv = (uint16_t)boosted_torque;
	output->extra_pct = extra_pct;
	output->active = boosted_torque > torque_input_mv;
}
