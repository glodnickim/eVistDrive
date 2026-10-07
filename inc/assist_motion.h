/*
 * Assist Behavior V3 - optional motion / IMU seam (ARCHITECTURE_V3.md section 3.3).
 *
 * Current hardware has no IMU driver: main.c passes { .valid = false }. V3 and the future terrain
 * estimator read ONLY the sanitised copy produced by assist_motion_sanitize(), so garbage in an
 * invalid or stale sample can never reach a control decision (TEST_MATRIX G1-IMU).
 */
#ifndef ASSIST_MOTION_H
#define ASSIST_MOTION_H

#include <stdbool.h>
#include <stdint.h>

/* Oldest sample still accepted, ms (candidate; no driver exists yet, the terrain estimator of
 * Milestone F sets the final value). Overridable at compile time. */
#ifndef MOTION_MAX_AGE_MS
#define MOTION_MAX_AGE_MS 100u
#endif

typedef struct {
    bool     valid;
    uint16_t age_ms;            /* time since the sample was taken                 */
    int16_t  pitch_cdeg;        /* +nose up, 0.01 deg                              */
    int16_t  roll_cdeg;
    int16_t  pitch_rate_cdps;   /* 0.01 deg/s                                      */
    int16_t  accel_long_mg;     /* longitudinal, bike frame, gravity removed, mg   */
    int16_t  accel_vert_mg;
} motion_input_t;

/* Copies raw to out only when raw->valid && raw->age_ms <= MOTION_MAX_AGE_MS; otherwise writes an
 * all-zero struct with valid = false. raw == NULL is treated as invalid. raw and out may alias.
 * The validity flag is read as a byte (any non-zero byte = true) so a corrupted bool cannot
 * produce undefined behaviour. */
void assist_motion_sanitize(const motion_input_t *raw, motion_input_t *out);

#endif /* ASSIST_MOTION_H */
