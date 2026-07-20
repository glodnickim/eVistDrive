#ifndef RIDE_CONTROL_H_
#define RIDE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	RIDE_ENGINE_LEGACY = 0,
	RIDE_ENGINE_TSDZ
} ride_engine_t;

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
} ride_control_input_t;

void ride_control_init(void);
int32_t ride_control_update_request(void);
void ride_control_update(const ride_control_input_t *input);
bool ride_control_set_engine(ride_engine_t engine);
ride_engine_t ride_control_get_engine(void);

#endif /* RIDE_CONTROL_H_ */
