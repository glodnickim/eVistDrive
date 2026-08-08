#ifndef RIDE_CONTROL_H_
#define RIDE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * FW-094: there is no engine type any more. The ride core is the only assist pipeline, so
 * nothing selects, reports or branches on one. The single remaining trace is a constant byte
 * in the 0x6028/0x6029 telemetry blocks that the shipped app still parses — it lives in
 * CAN_Display.c as a protocol constant, not as a runtime state.
 */

typedef struct {
	uint32_t speed_x100;
	uint8_t cadence_rpm;
	uint8_t assist_level_index;
	uint32_t battery_voltage_mv;
	int32_t iq_scale;
	int32_t ride_core_iq_limit;
	int32_t phase_current_max;
	int32_t current_iq;
	int32_t current_id;
	uint16_t voltage_raw;
	int16_t voltage_min_raw;
	int16_t controller_temperature_c;
	uint16_t cadence_filtered_x8;
	uint16_t speed_limit_x100;
	bool legal_enabled;
	bool offroad;
	bool walk_active;
	bool position_calibration_active;
	bool safety_cut;
	int32_t throttle_iq;   // FW-030: throttle current (mapped from ADC in main.c); floor on ride-core output
} ride_control_input_t;

/*
 * FW-096 diagnostics: WHY is the current zero?
 *
 * Pure observation — nothing here changes a decision, it only records which gate the pipeline
 * went through on the last tick. Added because "MS.i_q_setpoint is 0" on its own says nothing
 * about who zeroed it, which turned a regression hunt into guesswork.
 */
#define RIDE_DBG_WALK             0x01  /* Walk Assist owns Iq this tick */
#define RIDE_DBG_CALIBRATION      0x02  /* position calibration owns Iq this tick */
#define RIDE_DBG_HARD_CUT         0x04  /* brake / reverse / overtemp / torque fault / cal */
#define RIDE_DBG_LEVEL_ZERO       0x08  /* assist level 0 */
#define RIDE_DBG_NOT_LATCHED      0x10  /* ride latch not armed -> assist target forced 0 */
#define RIDE_DBG_MODE_UNSUPPORTED 0x20  /* assist_modes_calculate() reported unsupported */
#define RIDE_DBG_LIMITER_ZEROED   0x40  /* a non-zero demand was taken to 0 by assist_limits */
#define RIDE_DBG_COAST_RELEASE    0x80  /* FW-048 coast-out cut the tail */

uint8_t ride_control_get_debug_flags(void);

void ride_control_init(void);
void ride_control_update(const ride_control_input_t *input);

#endif /* RIDE_CONTROL_H_ */
