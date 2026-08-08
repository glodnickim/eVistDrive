#ifndef MOTOR_SERVICE_H_
#define MOTOR_SERVICE_H_

#include <stdint.h>

/*
 * Motor-layer service paths: the two Iq producers that are NOT the assist pipeline.
 *
 * Both are defined in main.c because they read state that belongs to the motor layer — the
 * Hall capture counters, the TIMER0 PWM registers, the virtual-EEPROM writer. ride_control.c
 * reaches them through this header, so the shared ride logic never touches the motor layer
 * directly and can be built for a different controller by providing these two functions.
 *
 * FW-094: these replace legacy_assist_calculate(). The pre-ride-core assist monolith used to
 * serve both of them, running its entire cadence-assist body first and then throwing the
 * result away — only the two branches below ever reached the motor. Removing the monolith
 * removed the calculation, not the behaviour: both functions do exactly what the reachable
 * branches did, in the same order.
 */

/*
 * Walk Assist. walk_assist_motor / walk_speed_controller own the complete Iq trajectory
 * (FW-060/FW-067); this wrapper hands them the motor state and applies the shared
 * undervoltage and controller-temperature limits.
 *
 * The legal speed taper is deliberately NOT applied: Walk Assist carries its own wheel-speed
 * ceiling inside walk_motor_update(), and the pre-FW-094 path reached the same limiter with
 * the same walk flag set.
 */
uint16_t walk_assist_iq_request(void);

/*
 * Phase 2 of position-sensor (Hall angle) calibration: hold a fixed probe current while the
 * stored angle correction is trimmed, then switch the bridge off and persist the result. A
 * controller service mode, never an assist mode — it owns Iq outright and bypasses the
 * ride-feel ramp (see ride_control_update).
 */
uint16_t hall_calibration_iq_request(void);

#endif /* MOTOR_SERVICE_H_ */
