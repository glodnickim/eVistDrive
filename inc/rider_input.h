#ifndef RIDER_INPUT_H_
#define RIDER_INPUT_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Normalized view of rider-related sensors in the existing EBICS native
 * units. Fixed-point throughout: the assist pipeline runs in the 4 kHz control
 * tick and must be deterministic, so no value here is floating point.
 */
typedef struct {
	uint16_t torque_raw_mv;
	int16_t torque_corrected_mv;
	uint16_t torque_filtered;
	uint16_t torque_assist_filtered;   /* fast 35 ms: start, safety, start-gate */
	uint16_t torque_run_filtered;      /* FW-033: slow RUN estimator: power/eMTB/torque, boost */
	uint16_t torque_load_centikg;

	uint8_t cadence_rpm;
	uint32_t wheel_speed_x100;
	uint16_t motor_erps;
	uint16_t motor_voltage_utilization;

	bool pas_forward;
	bool pas_backward;
	bool pedaling_active;
	/* FW-083: raw consecutive-forward-step count and direction (the two
	 * components pedaling_active above is built from — cadence>0, not
	 * reversed, not idle-timed-out), exposed separately so ride_control can
	 * apply a lower step requirement specifically while the bike is already
	 * rolling, without re-deriving the direction check from scratch. */
	uint8_t crank_forward_steps;
	bool crank_direction_ok;
	bool start_phase;
	bool torque_sensor_valid;
	bool pas_sensor_valid;
} rider_input_t;

void rider_input_update(const rider_input_t *sample);
const rider_input_t *rider_input_get(void);

#endif /* RIDER_INPUT_H_ */
