/* Harness mirror of main.c Speed_processing() and its foreground silence decay.
 * Speed_processing cannot be linked without GD32 main and ISR globals. Constants
 * and integer order deliberately match the target path. One edge per wheel turn. */
#include "wheel_sensor.h"
#include "config.h"
#include <stdbool.h>

#define L4_MAX_INSTANT_X100 7000U
#define L4_MAX_ACCEL_X100_PER_S 2500U

void evd_wheel_sensor_tick(evd_wheel_sensor_t *s, uint32_t tick, float distance_m)
{
    uint32_t physical_pulses = distance_m > 0.0f ?
        (uint32_t)(distance_m * 1000.0f * PULSES_PER_REVOLUTION / WHEEL_CIRCUMFERENCE) : 0U;
    if (physical_pulses > s->pulse_count) {
        s->pulse_count = physical_pulses;
        if (!s->have_reference) {
            s->have_reference = true;
            s->last_tick = tick;
        } else {
            uint32_t period = tick - s->last_tick;
            if (period) {
                uint32_t instant = WHEEL_CIRCUMFERENCE * (SPEED_TIMEBASE_HZ / 1000U) * 360U /
                                   (PULSES_PER_REVOLUTION * period);
                uint32_t allowed = s->last_valid_x100 +
                    L4_MAX_ACCEL_X100_PER_S * period / SPEED_TIMEBASE_HZ;
                if (instant <= L4_MAX_INSTANT_X100 && instant <= allowed) {
                    s->cumulative_x100 -= s->cumulative_x100 / PULSES_PER_REVOLUTION;
                    s->cumulative_x100 += instant;
                    s->speed_x100 = s->cumulative_x100 / PULSES_PER_REVOLUTION;
                    s->last_valid_x100 = s->speed_x100;
                    s->last_tick = tick;
                }
            }
        }
    }
    uint32_t silence = tick - s->last_tick;
    if (silence > SPEED_STOP_TICKS) {
        s->speed_x100 = 0U;
        s->last_valid_x100 = 0U;
    } else if (s->speed_x100 && silence > SPEED_DECAY_GUARD_TICKS) {
        uint32_t implied = WHEEL_CIRCUMFERENCE * (SPEED_TIMEBASE_HZ / 1000U) * 360U /
                           (PULSES_PER_REVOLUTION * silence);
        if (s->speed_x100 * 100U > implied * (100U + SPEED_DECAY_MARGIN_PCT)) {
            s->speed_x100 = implied;
            s->last_valid_x100 = implied;
        }
    }
}
