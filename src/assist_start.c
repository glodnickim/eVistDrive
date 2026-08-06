#include "assist_start.h"

#include <limits.h>

#include "torque_input.h"
#include "tuning_config.h"

#define STARTUP_BOOST_CURVE_SIZE 120U
#define STARTUP_BOOST_STRENGTH_MAX_PCT 300U
#define STARTUP_BOOST_AUTO_TORQUE_MV 20U
#define CONTROL_TICKS_PER_MS 4U
#define SMOOTH_START_DURATION_MAX_MS 5000U
#define SMOOTH_START_ENVELOPE_MAX_PERMILLE 1000U

static uint16_t startup_boost_curve[STARTUP_BOOST_CURVE_SIZE];
static uint16_t cached_strength_pct = UINT16_MAX;
static uint16_t cached_cadence_step = UINT16_MAX;
static bool speed_boost_latched;
static assist_startup_boost_mode_t previous_mode = ASSIST_STARTUP_BOOST_CADENCE;

typedef struct {
	const assist_smooth_start_config_t *config_address;
	uint16_t duration_ms;
	bool enabled;
	bool stopped_seen;
	bool armed;
	uint32_t elapsed_ticks;
} smooth_start_state_t;

static smooth_start_state_t smooth_start_state;
static assist_smooth_start_output_t last_smooth_output;

static void rebuild_startup_boost_curve(uint16_t strength_pct)
{
	uint16_t cadence_step = tuning_config_cadence_step();

	if (strength_pct > STARTUP_BOOST_STRENGTH_MAX_PCT) {
		strength_pct = STARTUP_BOOST_STRENGTH_MAX_PCT;
	}
	if (strength_pct == cached_strength_pct &&
		cadence_step == cached_cadence_step) {
		return;
	}

	startup_boost_curve[0] = strength_pct;
	for (uint8_t cadence = 1; cadence < STARTUP_BOOST_CURVE_SIZE; cadence++) {
		uint32_t previous = startup_boost_curve[cadence - 1];
		uint32_t next =
			(previous << 8) - (previous * cadence_step);
		startup_boost_curve[cadence] = (uint16_t)(next >> 8);
	}
	cached_strength_pct = strength_pct;
	cached_cadence_step = cadence_step;
}

void assist_start_reset(void)
{
	speed_boost_latched = false;
	previous_mode = ASSIST_STARTUP_BOOST_CADENCE;
	smooth_start_state = (smooth_start_state_t){0};
	last_smooth_output = (assist_smooth_start_output_t){0};
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

	uint16_t torque_range = torque_input_span_native();
	uint16_t torque_input_mv = input->torque_input_mv;
	if (torque_input_mv > torque_range) {
		torque_input_mv = torque_range;
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

	if (!boost_enabled) {
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

	/*
	 * Deliberately not clamped to torque_range here: this is the "virtual"
	 * boosted torque, allowed to exceed the raw sensor span. Downstream,
	 * torque_input_native_delta_to_centikg() clamps to the wider
	 * TORQUE_SPAN_MAX_NATIVE, and the profile/power/Iq/phase-current/battery/
	 * thermal limits still apply after that — those are the intended ceiling,
	 * not the sensor's physical range.
	 */
	uint32_t boosted_torque = torque_input_mv +
		((uint32_t)torque_input_mv * extra_pct) / 100U;

	output->torque_output_mv = (uint16_t)boosted_torque;
	output->extra_pct = extra_pct;
	output->active = boosted_torque > torque_input_mv;
}

int32_t assist_start_apply_smooth(
	const assist_smooth_start_input_t *input,
	const assist_smooth_start_config_t *config,
	assist_smooth_start_output_t *output)
{
	assist_smooth_start_output_t local_output = {0};
	if (output == 0) {
		output = &local_output;
	}
	*output = (assist_smooth_start_output_t){0};
	if (input == 0 || config == 0) {
		last_smooth_output = *output;
		return 0;
	}

	uint16_t duration_ms = config->duration_ms;
	if (duration_ms > SMOOTH_START_DURATION_MAX_MS) {
		duration_ms = SMOOTH_START_DURATION_MAX_MS;
	}
	if (smooth_start_state.config_address != config ||
		smooth_start_state.enabled != config->enabled ||
		smooth_start_state.duration_ms != duration_ms) {
		smooth_start_state = (smooth_start_state_t){
			.config_address = config,
			.duration_ms = duration_ms,
			.enabled = config->enabled
		};
	}

	/*
	 * FW-092: a rolling bike is not stopped, however still the cranks and the motor are.
	 * Smooth start exists to soften a launch from a standstill; arming it while the rider
	 * is merely coasting turns every mid-ride re-engage into a fresh launch ramp.
	 */
	bool stopped =
		input->measured_cadence_rpm == 0 && input->motor_erps == 0 &&
		!input->bike_rolling;
	if (stopped && !smooth_start_state.stopped_seen) {
		smooth_start_state.armed = true;
		smooth_start_state.elapsed_ticks = 0;
	}
	smooth_start_state.stopped_seen = stopped;

	/*
	 * An armed envelope that was never spent must not lie in wait. Standing still arms it;
	 * if the rider then rolls away without pedalling — a push off, a downhill start — the
	 * arming survived, because nothing cleared it until an envelope actually completed. The
	 * first pedal stroke at 15 km/h then got a standstill launch ramp it had not earned.
	 *
	 * Only cancelled while rolling with NO demand. An envelope already running under real
	 * demand is legitimate and must be allowed to finish, which is why iq_target is checked.
	 */
	if (input->bike_rolling && input->iq_target <= 0) {
		smooth_start_state.armed = false;
		smooth_start_state.elapsed_ticks = 0;
	}

	if (input->safety_cut || input->iq_target <= 0) {
		last_smooth_output = *output;
		return 0;
	}

	if (!config->enabled || duration_ms == 0 || !smooth_start_state.armed) {
		output->iq_target = input->iq_target;
		output->envelope_permille = SMOOTH_START_ENVELOPE_MAX_PERMILLE;
		last_smooth_output = *output;
		return output->iq_target;
	}

	uint32_t duration_ticks = (uint32_t)duration_ms * CONTROL_TICKS_PER_MS;
	if (smooth_start_state.elapsed_ticks < duration_ticks) {
		smooth_start_state.elapsed_ticks++;
	}
	uint32_t envelope_permille =
		(smooth_start_state.elapsed_ticks * SMOOTH_START_ENVELOPE_MAX_PERMILLE) /
		duration_ticks;
	if (envelope_permille >= SMOOTH_START_ENVELOPE_MAX_PERMILLE) {
		envelope_permille = SMOOTH_START_ENVELOPE_MAX_PERMILLE;
		smooth_start_state.armed = false;
	}

	output->iq_target = (int32_t)(
		((uint32_t)input->iq_target * envelope_permille) /
		SMOOTH_START_ENVELOPE_MAX_PERMILLE);
	output->envelope_permille = (uint16_t)envelope_permille;
	output->active = smooth_start_state.armed;
	last_smooth_output = *output;
	return output->iq_target;
}

const assist_smooth_start_output_t *assist_start_get_last_smooth_output(void)
{
	return &last_smooth_output;
}
