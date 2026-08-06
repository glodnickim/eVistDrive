#ifndef TUNING_CONFIG_H_
#define TUNING_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Single owner of the GLOBAL ride-feel tuning values that were previously
 * compile-time constants: the ride-core startup boost cadence decay
 * (assist_start), the ride latch, the RUN torque estimator and the start-condition
 * crank movement. FW-0xx removed the legacy monolith's separate STARTUP_BOOST_*
 * constants and boost application - this module is now the only source. Values
 * are stored in RAM; MotorParams persistence is handled by the caller (main.c),
 * same pattern as profile banks.
 *
 * FW-069: the four Iq ramps LEFT this module and are now per level, in the bank blob
 * (assist_level_config_t). Their four u16 slots stay in the wire format so an older
 * Canable still writes a blob this firmware accepts - but the values are ignored.
 */

#define TUNING_RAMP_MS_MIN 20U
#define TUNING_RAMP_MS_MAX 5000U
#define TUNING_CADENCE_STEP_MIN 1U
#define TUNING_CADENCE_STEP_MAX 100U

/* FW-068: forward crank steps required before assist may start (global: this is the
 * anti-jiggle guard, a property of the bike and its sensor, not of an assist level). */
#define TUNING_START_STEPS_MIN 1U
#define TUNING_START_STEPS_MAX 20U
#define TUNING_START_STEPS_DEFAULT 4U

/* FW-032: ride latch (run deadband / hold / current floor), exposed to Canable. */
#define TUNING_RUN_DEADBAND_MV_MAX 100U
#define TUNING_HOLD_MS_MAX 3000U
#define TUNING_MIN_IQ_PCT_MAX 25U

/* FW-085: RUN torque estimator averaging window, in CRANK DEGREES (was ms up to v6). */
#define TUNING_TORQUE_RUN_WINDOW_DEG_MAX 360U
#define TUNING_TORQUE_RUN_WINDOW_DEG_DEFAULT 180U

/* Blob: v2 = 22 B (+3 latch u16), v3/v4/v5 = 24 B (+1 torque-run u16),
 * v6 = 32 B (+1 start-steps u16 + 3 reserved u16), v7 = 32 B (same layout as v6;
 * the field at offset 20 changes unit from ms to crank degrees). apply_blob accepts
 * all of them; the per-version body/length table lives in tuning_config.c. */
#define TUNING_BLOB_LEN 32U
#define TUNING_BLOB_LEN_V3 24U
#define TUNING_BLOB_LEN_V2 22U

uint16_t tuning_config_cadence_step(void);

/* FW-068: forward crank steps required before assist may start. */
uint8_t tuning_config_start_steps(void);

/* FW-032: ride latch getters (see ride_control.c). */
uint16_t tuning_config_run_deadband_mv(void);
int32_t tuning_config_assist_hold_ticks(void);
uint16_t tuning_config_min_iq_pct(void);

/* FW-085: RUN torque estimator window (crank degrees; 0 = disabled). */
uint16_t tuning_config_assist_torque_run_window_deg(void);

uint16_t tuning_config_serialize(uint8_t *buffer);
bool tuning_config_apply_blob(const uint8_t *buffer, uint16_t length);

#endif /* TUNING_CONFIG_H_ */
