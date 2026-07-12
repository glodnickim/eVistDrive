#include "motor_core.h"

static MotorState_t *state;

void motor_core_init(MotorState_t *motor_state)
{
	state = motor_state;
}

void motor_core_set_command(const motor_command_t *command)
{
	if (state == 0 || command == 0) {
		return;
	}

	if (command->emergency_stop || !command->enable) {
		state->i_q_setpoint = 0;
		state->i_d_setpoint = 0;
		return;
	}

	state->i_q_setpoint = command->iq_target;
	state->i_d_setpoint = command->id_target;
}

void motor_core_legacy_set_iq_target(int32_t iq_target)
{
	if (state != 0) {
		state->i_q_setpoint = iq_target;
	}
}

void motor_core_legacy_set_id_target(int32_t id_target)
{
	if (state != 0) {
		state->i_d_setpoint = id_target;
	}
}
