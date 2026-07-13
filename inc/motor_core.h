#ifndef MOTOR_CORE_H_
#define MOTOR_CORE_H_

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

/*
 * Ride Control produces this command. Motor Core is the only layer that may
 * eventually copy the final targets into the state consumed by the FOC loop.
 * Values use the existing EBICS native current units during the behaviour-
 * preserving refactor; unit conversion is intentionally out of scope here.
 */
typedef struct {
	int32_t iq_target;
	int32_t id_target;
	bool enable;
	bool emergency_stop;
} motor_command_t;

/* Bind the legacy motor state without changing its initial values. */
void motor_core_init(MotorState_t *motor_state);

/* Command entry point to be adopted one legacy writer at a time. */
void motor_core_set_command(const motor_command_t *command);

#endif /* MOTOR_CORE_H_ */
