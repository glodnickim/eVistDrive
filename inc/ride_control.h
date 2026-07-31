#ifndef RIDE_CONTROL_H_
#define RIDE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * These are WIRE values: the engine byte goes out in the 0x6029 diagnostics block and
 * Canable reads it as 0 = Legacy, 1 = ride core. Written out explicitly rather than left
 * to enum ordering, so inserting a member above cannot silently renumber the telemetry.
 */
typedef enum {
	RIDE_ENGINE_LEGACY = 0,
	RIDE_ENGINE_CORE = 1
} ride_engine_t;

_Static_assert(RIDE_ENGINE_CORE == 1,
	"ride engine telemetry value must stay 1 (0x6029 diagnostics)");

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

void ride_control_init(void);
void ride_control_update(const ride_control_input_t *input);
// FW-030: engine selection removed (ride core only). get_engine kept as a stub that always
// reports the ride core, for telemetry; set_engine / update_request removed.
ride_engine_t ride_control_get_engine(void);

#endif /* RIDE_CONTROL_H_ */
