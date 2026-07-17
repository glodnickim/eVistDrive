#ifndef TUNING_CONFIG_H_
#define TUNING_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Single owner of the global ride-feel tuning values that were previously
 * compile-time constants: the shared Iq acceleration/deceleration ramps
 * (assist_dynamics, used by both engines) and the ride-core startup boost
 * cadence decay (assist_start). Legacy's own separate STARTUP_BOOST_CADENCE_STEP
 * in config.h is untouched. Values are stored in RAM; MotorParams persistence
 * is handled by the caller (main.c), same pattern as profile banks.
 */

#define TUNING_RAMP_MS_MIN 20U
#define TUNING_RAMP_MS_MAX 5000U
#define TUNING_CADENCE_STEP_MIN 1U
#define TUNING_CADENCE_STEP_MAX 100U
#define TUNING_BLOB_LEN 16U

int32_t tuning_config_ramp_up_slow_ticks(void);
int32_t tuning_config_ramp_up_fast_ticks(void);
int32_t tuning_config_ramp_down_slow_ticks(void);
int32_t tuning_config_ramp_down_fast_ticks(void);
uint16_t tuning_config_cadence_step(void);

uint16_t tuning_config_serialize(uint8_t *buffer);
bool tuning_config_apply_blob(const uint8_t *buffer, uint16_t length);

#endif /* TUNING_CONFIG_H_ */
