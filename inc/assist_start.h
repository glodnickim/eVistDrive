#ifndef ASSIST_START_H_
#define ASSIST_START_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	ASSIST_STARTUP_BOOST_CADENCE = 0,
	ASSIST_STARTUP_BOOST_SPEED = 1,
	ASSIST_STARTUP_BOOST_AUTO = 2
} assist_startup_boost_mode_t;

typedef struct {
	bool enabled;
	assist_startup_boost_mode_t mode;
	uint16_t strength_pct;
	uint8_t end_rpm;
} assist_startup_boost_config_t;

typedef struct {
	uint16_t torque_input_mv;
	uint8_t cadence_for_assist_rpm;
	uint32_t wheel_speed_x100;
	bool torque_sensor_valid;
} assist_startup_boost_input_t;

typedef struct {
	uint16_t torque_output_mv;
	uint16_t extra_pct;
	bool active;
} assist_startup_boost_output_t;

typedef struct {
	bool enabled;
	uint16_t duration_ms;
} assist_smooth_start_config_t;

typedef struct {
	int32_t iq_target;
	uint8_t measured_cadence_rpm;
	uint16_t motor_erps;
	/*
	 * FW-092: the bike may be rolling with BOTH the cranks and the motor stationary — that
	 * is simply what coasting looks like on a mid-drive, where the freewheel lets the motor
	 * stand still at speed. Judging "stopped" from cadence and motor alone therefore armed
	 * the standstill launch envelope during an ordinary mid-ride re-engage, so the first
	 * current arrived a whole smooth-start duration late instead of on the third PAS step.
	 *
	 * Passed as a ready flag rather than a speed: ride_control already owns the definition
	 * of "the bike is rolling" for the eased start-step count, and a second speed threshold
	 * here would be a copy that only LOOKS shared until someone changes one of them.
	 */
	bool bike_rolling;
	bool safety_cut;
} assist_smooth_start_input_t;

typedef struct {
	int32_t iq_target;
	uint16_t envelope_permille;
	bool active;
} assist_smooth_start_output_t;

void assist_start_reset(void);

void assist_start_apply_boost(
	const assist_startup_boost_input_t *input,
	const assist_startup_boost_config_t *config,
	assist_startup_boost_output_t *output);

int32_t assist_start_apply_smooth(
	const assist_smooth_start_input_t *input,
	const assist_smooth_start_config_t *config,
	assist_smooth_start_output_t *output);

const assist_smooth_start_output_t *assist_start_get_last_smooth_output(void);

#endif /* ASSIST_START_H_ */
