#include "ride_control.h"

#include "assist_dynamics.h"
#include "legacy_assist.h"
#include "motor_core.h"

static ride_engine_t active_engine;

void ride_control_init(void)
{
	active_engine = RIDE_ENGINE_LEGACY;
}

int32_t ride_control_update_request(void)
{
	switch (active_engine) {
	case RIDE_ENGINE_LEGACY:
		return legacy_assist_calculate();
	case RIDE_ENGINE_TSDZ:
	default:
		/* TSDZ is intentionally unavailable until the Legacy refactor is verified. */
		return 0;
	}
}

void ride_control_update(const ride_control_input_t *input)
{
	if (input == 0) {
		motor_command_t stop_command = {
			.iq_target = 0,
			.id_target = 0,
			.enable = false,
			.emergency_stop = true
		};
		motor_core_set_command(&stop_command);
		return;
	}

	int32_t iq_target = ride_control_update_request();
	assist_dynamics_input_t dynamics_input = {
		.speed_x100 = input->speed_x100,
		.cadence_rpm = input->cadence_rpm,
		.iq_scale = input->iq_scale,
		.phase_current_max = input->phase_current_max,
		.walk_active = input->walk_active,
		.safety_cut = input->safety_cut
	};
	int32_t iq_reference = assist_dynamics_apply_legacy(
		iq_target,
		input->current_iq,
		&dynamics_input);
	motor_command_t command = {
		.iq_target = iq_reference,
		.id_target = input->current_id,
		.enable = true,
		.emergency_stop = false
	};
	motor_core_set_command(&command);
}

ride_engine_t ride_control_get_engine(void)
{
	return active_engine;
}
