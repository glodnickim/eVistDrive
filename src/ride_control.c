#include "ride_control.h"

#include "assist_dynamics.h"
#include "assist_limits.h"
#include "assist_modes.h"
#include "config.h"
#include "legacy_assist.h"
#include "motor_core.h"

static ride_engine_t active_engine;

void ride_control_init(void)
{
	if (RIDE_ENGINE_DEFAULT == RIDE_ENGINE_TSDZ) {
		active_engine = RIDE_ENGINE_TSDZ;
	} else {
		active_engine = RIDE_ENGINE_LEGACY;
	}
}

int32_t ride_control_update_request(void)
{
	switch (active_engine) {
	case RIDE_ENGINE_LEGACY:
		return legacy_assist_calculate();
	case RIDE_ENGINE_TSDZ:
	default:
		/* TSDZ requests need the full ride_control_input_t path below. */
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

	int32_t iq_target;
	if (input->walk_active) {
		/*
		 * The existing Walk controller remains the exclusive source until the
		 * dedicated ERPS-based module replaces it. It must not disappear when
		 * the developer selects the TSDZ riding engine.
		 */
		iq_target = legacy_assist_calculate();
	} else if (active_engine == RIDE_ENGINE_TSDZ) {
		const rider_input_t *rider = rider_input_get();
		const assist_level_config_t *level =
			assist_modes_get_default_level(input->assist_level_index);
		assist_mode_output_t mode_output;
		bool supported = assist_modes_calculate(
			rider,
			level,
			input->battery_voltage_mv,
			input->iq_scale,
			&mode_output);
		iq_target = supported ? mode_output.iq_request : 0;

		assist_limits_input_t limits_input = {
			.voltage_raw = input->voltage_raw,
			.voltage_min_raw = input->voltage_min_raw,
			.controller_temperature_c = input->controller_temperature_c,
			.cadence_filtered_x8 = input->cadence_filtered_x8,
			.speed_x100 = input->speed_x100,
			.speed_limit_x100 = input->speed_limit_x100,
			.legal_enabled = input->legal_enabled,
			.offroad = input->offroad,
			.walk_active = input->walk_active
		};
		iq_target = assist_limits_apply(iq_target, &limits_input);
	} else {
		iq_target = ride_control_update_request();
	}
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

bool ride_control_set_engine(ride_engine_t engine)
{
	if (engine != RIDE_ENGINE_LEGACY && engine != RIDE_ENGINE_TSDZ) {
		return false;
	}
	active_engine = engine;
	return true;
}

ride_engine_t ride_control_get_engine(void)
{
	return active_engine;
}
