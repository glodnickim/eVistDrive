#ifndef MOTION_EST_H_
#define MOTION_EST_H_

#include <stdbool.h>
#include <stdint.h>

/* Pipeline-owned wheel/motor observation. All distances are monotonic modulo 2^32 mm. */
typedef struct {
    uint32_t speed_x100;
    uint32_t distance_mm;
    int16_t rel_accel_permille_s;
    uint8_t quality; /* 0 unknown, 1 wheel interpolation, 2 learned motor ratio */
} motion_est_output_t;

typedef struct {
    uint32_t last_pulse_tick, pulse_mm, distance_mm, distance_frac;
    uint32_t ratio_q12, previous_tick;
    uint16_t wheel_speed_x100, previous_erps;
    int16_t pulse_cadence_rpm;
    uint8_t pulse_seen, ratio_valid;
    motion_est_output_t out;
} motion_est_t;

void motion_est_reset(motion_est_t *s);
const motion_est_output_t *motion_est_update(motion_est_t *s, uint32_t now_tick,
    uint32_t elapsed_ticks, bool wheel_valid, uint32_t wheel_pulse_tick,
    uint16_t motor_erps, int16_t cadence_rpm, int32_t iq_measured);

#endif
