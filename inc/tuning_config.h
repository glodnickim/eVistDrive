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

/*
 * FW-129 §6: ASSIST TORQUE FULL SCALE. The pedal load that maps to 100 % of the normalized
 * 0..160 torque axis eMTB and Torque work on.
 *
 * This is a RIDE-FEEL setting and has nothing to do with the sensor. It is NOT the torque
 * calibration span, NOT the sensor's native maximum and NOT the ADC maximum: changing it does
 * not move the kilogram reading, the zero, the calibration or the 0x6025 telemetry by one
 * unit. It only decides how early those two modes reach the upper part of their own curve.
 * Global rather than per level because the bank record has no room left (255 B, see
 * inc/assist_modes.h) and because it describes the rider's pedal, not one assist level.
 */
/*
 * FW-151: stored and applied in the CONTROL domain (CLU), not in kilograms. It is the top of
 * the rider-effort axis, so a re-measured kg table must not move it - that would rescale the
 * whole effort curve of every stored profile. The rider still sets it in kilograms; the wire
 * value is converted once, where the setting is accepted. The numbers are unchanged, which is
 * why no stored tuning blob needs migrating: on the characteristic the control domain is frozen
 * to, 6000 CLU is the same point 6000 centikg used to be.
 */
#define TUNING_ASSIST_TORQUE_FULL_SCALE_CTRL_DEFAULT 6000U
#define TUNING_ASSIST_TORQUE_FULL_SCALE_CTRL_MIN     2000U
#define TUNING_ASSIST_TORQUE_FULL_SCALE_CTRL_MAX    12000U

/*
 * FW-129 §10: crank length. load_centikg is a physical force on the pedal, so rider power is
 * force x crank length x crank speed - the arm length is part of the equation, not a constant.
 * Range 150..190 mm: 140 mm does not exist on a standard e-bike crank (150 mm is the shortest
 * in series) and every millimetre below that only costs fixed-point resolution.
 */
#define TUNING_CRANK_LENGTH_MM_DEFAULT 165U
#define TUNING_CRANK_LENGTH_MM_MIN     150U
#define TUNING_CRANK_LENGTH_MM_MAX     190U

/* Blob: v2 = 22 B (+3 latch u16), v3/v4/v5 = 24 B (+1 torque-run u16),
 * v6 = 32 B (+1 start-steps u16 + 3 reserved u16), v7 = 32 B (same layout as v6;
 * the field at offset 20 changes unit from ms to crank degrees), v8 = 32 B (FW-129: spends
 * two of v6's three reserved u16 on assist torque full scale and crank length; the third,
 * at offset 28, stays reserved). apply_blob accepts all of them; the per-version body/length
 * table lives in tuning_config.c.
 *
 * The LENGTH does not change and must not: 0x6023/0x6024 carry a fixed number of frames. */
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

/* FW-129/151: ride-feel torque axis (control domain) and the crank arm of the rider-power
 * equation (physical - rider power genuinely is a force times a length). */
uint16_t tuning_config_assist_torque_full_scale_ctrl(void);
uint16_t tuning_config_crank_length_mm(void);

uint16_t tuning_config_serialize(uint8_t *buffer);
bool tuning_config_apply_blob(const uint8_t *buffer, uint16_t length);

#endif /* TUNING_CONFIG_H_ */
