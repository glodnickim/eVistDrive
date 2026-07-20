#ifndef RIDER_INPUT_H_
#define RIDER_INPUT_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Normalized view of rider-related sensors in the existing EBICS native
 * units. Fixed-point values keep the 4 kHz Legacy path deterministic.
 */
typedef struct {
	uint16_t torque_raw_mv;
	int16_t torque_corrected_mv;
	uint16_t torque_filtered;
	uint16_t torque_assist_filtered;
	uint16_t torque_load_centikg;

	uint8_t cadence_rpm;
	uint32_t wheel_speed_x100;
	uint16_t motor_erps;
	uint16_t motor_voltage_utilization;

	bool pas_forward;
	bool pas_backward;
	bool pedaling_active;
	bool cadence_seeded;
	bool torque_sensor_valid;
	bool pas_sensor_valid;
} rider_input_t;

void rider_input_update(const rider_input_t *sample);
const rider_input_t *rider_input_get(void);

#endif /* RIDER_INPUT_H_ */
