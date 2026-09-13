#include "ride_control.h"

#include "ap2_limits.h"
#include "assist_pipeline.h"
#include "config.h"
#include "fast_iq_slew.h"
#include "iq_chain.h"
#include "motor_core.h"
#include "motor_service.h"
#include "rider_input.h"
#include "tuning_config.h"

/*
 * RIDE CONTROL - the arbitration between the THREE things that may own the motor, and nothing
 * else.
 *
 *     position calibration   a workshop service mode; owns Iq outright
 *     Walk Assist            its own speed controller; owns its own trajectory
 *     pedal assist           assist_pipeline.c, the one rider-facing path
 *
 * Exactly one of them is selected per tick, and all three publish through the SAME single
 * final-Iq mailbox, which the 16 kHz FOC ISR consumes. There is no second writer of the motor
 * command anywhere in this firmware.
 *
 * This file used to contain the assist algorithm itself - the ride latch, the current floor,
 * the hold grace, the mode call, Extended Boost, the start smoothing, the separate limiter
 * calls and the rearm machinery. All of that moved into the pipeline, where it is one ordered
 * chain instead of a sequence of blocks that could each raise or lower the demand for reasons
 * of their own. What is left here is selection and publication.
 */

/*
 * The 4 kHz -> 16 kHz final-Iq mailbox (seqlocked). Owned here on the producer side; the
 * 16 kHz FOC ISR consumes it via fast_iq_slew_tick(). There is exactly ONE dynamic Iq state
 * in the firmware and it lives at 16 kHz.
 */
static fast_iq_slew_mailbox_t final_iq_slew_mailbox;
static volatile int32_t final_iq_requested;

/* Walk Assist was active on the previous tick. Leaving Walk must not let a stale walk current
 * fade out through the assist release: it is cut in the same tick. */
static bool walk_was_active;

bool ride_control_battery_limit_active(void)
{
	return assist_pipeline_battery_limited();
}

fast_iq_slew_mailbox_t *ride_control_final_iq_slew_mailbox(void)
{
	return &final_iq_slew_mailbox;
}

int32_t ride_control_final_iq_requested(void)
{
	return final_iq_requested;
}

static void ride_publish_final_iq(
	int32_t target,
	fis_mode_t mode,
	uint16_t step_mag_8,
	uint32_t release_ticks_16k,
	fis_zero_policy_t zero_policy)
{
	fast_iq_slew_publish(
		&final_iq_slew_mailbox,
		target,
		mode,
		step_mag_8,
		release_ticks_16k,
		zero_policy);
	final_iq_requested = target;
}

void ride_control_force_final_iq_zero(void)
{
	/* A forced zero is never a rider release - NONE keeps the ordinary zero-current PI. */
	ride_publish_final_iq(0, FIS_MODE_FORCE_ZERO, 0U, 0U, FIS_ZERO_POLICY_NONE);
}

void ride_control_request_service_iq(int32_t iq_target)
{
	if (iq_target < 0) {
		iq_target = 0;
	}
	/* Service and Walk own their own trajectory; they never arm Quiet Zero. */
	ride_publish_final_iq(iq_target, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE);
}

uint8_t ride_control_get_session_state(void)
{
	return ap2_pas_state_legacy_session_code();
}

uint8_t ride_control_get_pas_state(void)
{
	return (uint8_t)assist_pipeline_pas_state();
}

void ride_control_init(void)
{
	walk_was_active = false;
	assist_pipeline_init();
	iq_chain_reset();
	fast_iq_slew_reset(&final_iq_slew_mailbox);
	final_iq_requested = 0;
}

/*
 * WALK ASSIST THROUGH THE SHARED CEILINGS.
 *
 * Walk owns its own speed controller and its own trajectory - that part is deliberately not
 * touched. What it did NOT own before, and does now, is the common protection chain: the same
 * battery-current, phase-current and thermal ceilings every other request passes. Its speed
 * rule stays its own (a wheel-speed cut-off stricter than the legal pedal taper), which is why
 * it enters the chain as its own source.
 */
static int32_t walk_iq_through_shared_limits(const ride_control_input_t *input, int32_t walk_iq)
{
	ap2_limits_input_t lim_in;
	ap2_limits_output_t lim;

	lim_in.iq_request = walk_iq;
	lim_in.source = AP2_LIMIT_SOURCE_WALK;
	lim_in.max_power_w = 0U;   /* Walk has no profile power envelope of its own */
	lim_in.battery_voltage_mv = input->battery_voltage_mv;
	lim_in.u_abs = input->u_abs;
	lim_in.cal_i = input->cal_i;
	lim_in.battery_current_ma = input->battery_current_mA;
	lim_in.battery_current_max = input->battery_current_max;
	lim_in.level_iq_limit = 0;   /* the assist level ceiling is not a Walk concept */
	lim_in.phase_current_max = input->phase_current_max;
	lim_in.voltage_raw = input->voltage_raw;
	lim_in.voltage_min_raw = input->voltage_min_raw;
	lim_in.controller_temperature_c = input->controller_temperature_c;
	lim_in.speed_x100 = input->speed_x100;
	lim_in.speed_limit_x100 = input->speed_limit_x100;
	lim_in.legal_enabled = input->legal_enabled;
	lim_in.offroad = input->offroad;
	ap2_limits_apply(&lim_in, &lim);
	return lim.final_iq;
}

void ride_control_update(const ride_control_input_t *input)
{
	const rider_input_t *rider;
	assist_pipeline_input_t pipe_in;
	assist_pipeline_command_t cmd;
	bool walk_release_cut;

	if (input == 0) {
		ride_control_force_final_iq_zero();
		motor_core_set_id_target(0);
		return;
	}

	walk_release_cut = walk_was_active && !input->walk_active;
	walk_was_active = input->walk_active;

	/*
	 * SERVICE MODE. Position-sensor calibration owns Iq outright and bypasses the ride-feel
	 * trajectory: on the completion tick the calibration code sets Iq to 0, disables PWM and
	 * stores the angle, and no stale ramp value may re-enable the bridge. The pipeline is
	 * reset on the way in, so a ride cannot survive a detour through a service mode and fire
	 * on the way out.
	 */
	if (input->position_calibration_active) {
		assist_pipeline_reset();
		ride_control_request_service_iq(hall_calibration_iq_request());
		motor_core_set_id_target(input->current_id);
		return;
	}

	/*
	 * WALK ASSIST. A separate demand owner, not an assist mode with different numbers: it has
	 * its own motor-speed controller and a second dynamic element behind it would only make
	 * that loop less stable. The pipeline is reset for the same reason as above.
	 */
	if (input->walk_active) {
		int32_t walk_iq = walk_iq_through_shared_limits(input,
			(int32_t)walk_assist_iq_request());
		assist_pipeline_reset();
		iq_chain_note_requested(walk_iq);
		iq_chain_note_allowed(walk_iq);
		ride_publish_final_iq(walk_iq, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE);
		motor_core_set_id_target(input->current_id);
		return;
	}

	/*
	 * Leaving Walk: the walk current must not be handed to the assist release as if the rider
	 * had just stopped pedalling. It is zeroed in the same tick, and the pipeline starts the
	 * next ride from zero.
	 */
	if (walk_release_cut) {
		assist_pipeline_reset();
		ride_publish_final_iq(0, FIS_MODE_FORCE_ZERO, 0U, 0U, FIS_ZERO_POLICY_NONE);
		motor_core_set_id_target(input->current_id);
		return;
	}

	/* ---- PEDAL ASSIST: the one rider-facing path -------------------------------------- */
	rider = rider_input_get();

	pipe_in.torque_load_centikg = rider->torque_load_centikg;
	pipe_in.torque_sensor_valid = rider->torque_sensor_valid;
	pipe_in.cadence_rpm = input->cadence_rpm;
	pipe_in.speed_x100 = input->speed_x100;
	pipe_in.motor_erps = rider->motor_erps;

	pipe_in.forward_valid = rider->crank_direction_ok;
	pipe_in.direction_inhibit = rider->direction_inhibit_active;
	pipe_in.inhibit_is_reverse = rider->pas_backward;
	pipe_in.real_stop = rider->real_stop;
	pipe_in.wheel_valid = rider->wheel_valid;
	pipe_in.pas_sensor_valid = rider->pas_sensor_valid;
	pipe_in.forward_steps = rider->crank_forward_steps;
	/* The anti-jiggle guard is a property of the bike and its sensor, not of an assist
	 * level, so it stays a global tuning value rather than travelling through the input. */
	pipe_in.required_steps = tuning_config_start_steps();

	pipe_in.assist_level_index = input->assist_level_index;

	pipe_in.safety_cut = input->safety_cut_non_direction;
	pipe_in.service_cut = input->service_cut_active;

	pipe_in.battery_voltage_mv = input->battery_voltage_mv;
	pipe_in.battery_current_ma = input->battery_current_mA;
	pipe_in.battery_current_max = input->battery_current_max;
	pipe_in.u_abs = input->u_abs;
	pipe_in.cal_i = input->cal_i;
	pipe_in.level_iq_limit = input->ride_core_iq_limit;
	pipe_in.phase_current_max = input->phase_current_max;
	pipe_in.voltage_raw = input->voltage_raw;
	pipe_in.voltage_min_raw = input->voltage_min_raw;
	pipe_in.controller_temperature_c = input->controller_temperature_c;

	pipe_in.speed_limit_x100 = input->speed_limit_x100;
	pipe_in.legal_enabled = input->legal_enabled;
	pipe_in.offroad = input->offroad;

	pipe_in.throttle_iq = input->throttle_iq;
	pipe_in.elapsed_ticks = input->elapsed_ticks;

	assist_pipeline_update(&pipe_in, &cmd);

	/* The two named stages of the demand, for diagnostics. The pipeline is the producer of
	 * both; recording a value cannot change it. */
	iq_chain_note_requested(assist_pipeline_telemetry()->iq_request_before_limits);
	iq_chain_note_allowed(cmd.final_iq_request);

	ride_publish_final_iq(cmd.final_iq_request, cmd.slew_mode, cmd.step_mag_8,
		cmd.release_ticks_16k, cmd.zero_policy);

	/* Iq is owned exclusively by fast_iq_slew_tick(); motor_core owns only Id here. */
	motor_core_set_id_target(input->current_id);
}
