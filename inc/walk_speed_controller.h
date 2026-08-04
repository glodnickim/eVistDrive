#ifndef WALK_SPEED_CONTROLLER_H_
#define WALK_SPEED_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Pure Walk Assist speed controller.
 *
 * The caller owns Hall estimation, hard safety gates and the LIMIT/STALL state
 * machine. This module owns the complete Iq trajectory while WA is active:
 * start preload, breakaway floor, PI, anti-windup and output slew.
 */
typedef struct {
	int32_t integral_q;
	int32_t iq_command_q;
	int32_t desired_iq;
	uint32_t session_ticks;
	uint8_t control_divider;
	bool startup_complete;
	bool reacquire_active;
} walk_speed_controller_t;

typedef struct {
	uint16_t target_erps;
	uint16_t measured_erps;
	bool hall_valid;
	bool reacquire;                 /* bounded recovery after Hall loss */
	int32_t iq_ceiling;
	int32_t run_iq_min;             /* normal RUN keepalive; 0 disables */
	int32_t run_iq_max;             /* soft RUN ceiling; unlike safety ceiling it uses the fall slew */
	int32_t downstream_iq;
} walk_speed_controller_input_t;

typedef struct {
	int32_t iq_target;
	int16_t error_erps;
	int16_t controlled_error_erps;
	int16_t p_iq;
	int16_t integral_iq;
	int16_t startup_iq;
	bool startup_active;
	bool above_target;
	bool saturated;
} walk_speed_controller_output_t;

void walk_speed_controller_reset(walk_speed_controller_t *controller);

int32_t walk_speed_controller_update(
	walk_speed_controller_t *controller,
	const walk_speed_controller_input_t *input,
	walk_speed_controller_output_t *output);

#endif /* WALK_SPEED_CONTROLLER_H_ */
