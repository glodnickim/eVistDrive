#ifndef ASSIST_MODES_H_
#define ASSIST_MODES_H_

#include <stdbool.h>
#include <stdint.h>

#include "assist_start.h"
#include "rider_input.h"

typedef enum {
	ASSIST_MODE_LEGACY = 0,
	ASSIST_MODE_POWER_LINEAR = 1,
	ASSIST_MODE_POWER_PROGRESSIVE = 2,
	ASSIST_MODE_EMTB_TSDZ = 3,
	ASSIST_MODE_EMTB_CUSTOM = 4
} assist_mode_type_t;

typedef struct {
	assist_mode_type_t mode_type;
	uint16_t support_ratio_pct;
	uint16_t support_min_pct;
	uint16_t support_max_pct;
	uint16_t reference_power_w;
	uint8_t progression_pct;
	uint16_t max_motor_power_w;
	uint8_t max_iq_pct;
	bool assist_without_rotation;
	uint16_t without_rotation_threshold_mv;
	assist_startup_boost_config_t startup_boost;
	assist_smooth_start_config_t smooth_start;
	uint16_t release_ms;
	uint16_t power_rise_filter_ms;
	uint16_t power_fall_filter_ms;
} assist_level_config_t;

typedef struct {
	uint16_t human_power_w;
	uint16_t assist_basis_power_w;
	uint16_t raw_motor_power_w;
	uint16_t motor_power_w;
	uint16_t applied_support_ratio_pct;
	uint32_t requested_battery_current_ma;
	int32_t iq_request;
	uint8_t cadence_for_assist_rpm;
	bool assist_without_rotation_active;
	uint16_t torque_for_assist_mv;
	uint16_t startup_boost_extra_pct;
	bool startup_boost_active;
} assist_mode_output_t;

const assist_level_config_t *assist_modes_get_default_level(uint8_t level_index);

void assist_modes_reset(void);

bool assist_modes_calculate(
	const rider_input_t *input,
	const assist_level_config_t *config,
	uint32_t battery_voltage_mv,
	int32_t iq_limit,
	assist_mode_output_t *output);

const assist_mode_output_t *assist_modes_get_last_output(void);

#endif /* ASSIST_MODES_H_ */
