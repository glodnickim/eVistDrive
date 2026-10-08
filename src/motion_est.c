#include "motion_est.h"
#include <string.h>

#define WHEEL_MM 2218u
#define TICKS_PER_SECOND 4000u
#define SPEED_DEN 1440u /* speed_x100 * ticks / 1440 = mm */
#define WHEEL_FRESH_TICKS 10600u

void motion_est_reset(motion_est_t *s) { if (s) memset(s, 0, sizeof(*s)); }

const motion_est_output_t *motion_est_update(motion_est_t *s, uint32_t now_tick,
    uint32_t elapsed_ticks, bool wheel_valid, uint32_t wheel_pulse_tick,
    uint16_t motor_erps, int16_t cadence_rpm, int32_t iq_measured)
{
    if (!s) return 0;
    uint32_t dt = elapsed_ticks ? elapsed_ticks : 1u;
    if (dt > 256u) dt = 256u;
    const bool pulse = wheel_valid && wheel_pulse_tick != 0u &&
        (!s->pulse_seen || wheel_pulse_tick != s->last_pulse_tick);
    if (pulse) {
        if (s->pulse_seen) {
            const uint32_t interval = wheel_pulse_tick - s->last_pulse_tick;
            if (interval >= 100u && interval <= WHEEL_FRESH_TICKS) {
                const uint32_t v = WHEEL_MM * SPEED_DEN / interval;
                s->wheel_speed_x100 = (uint16_t)(v > 10000u ? 10000u : v);
                const int32_t cadence_delta = (int32_t)cadence_rpm - s->pulse_cadence_rpm;
                if (cadence_rpm >= 20 && cadence_delta >= -5 && cadence_delta <= 5 &&
                    motor_erps >= 5u && iq_measured > 20 && v >= 500u) {
                    /* A ratio is accepted only while the rider is pedalling; a stopped crank
                     * cannot change gear. Refresh from a measured wheel interval. */
                    s->ratio_q12 = (v * 4096u) / motor_erps;
                    s->ratio_valid = 1u;
                }
            }
        }
        s->pulse_seen = 1u;
        s->last_pulse_tick = wheel_pulse_tick;
        s->pulse_cadence_rpm = cadence_rpm;
        s->pulse_mm += WHEEL_MM;
    }
    const bool wheel_fresh = s->pulse_seen && wheel_valid &&
        now_tick - s->last_pulse_tick <= WHEEL_FRESH_TICKS;
    uint32_t speed = wheel_fresh ? s->wheel_speed_x100 : 0u;
    uint8_t quality = wheel_fresh && speed ? 1u : 0u;
    if (s->ratio_valid && motor_erps >= 3u && iq_measured > 20) {
        speed = (uint32_t)(((uint64_t)motor_erps * s->ratio_q12 + 2048u) / 4096u);
        if (speed > 10000u) speed = 10000u;
        quality = 2u;
    }
    /* Between sparse wheel pulses, integrate speed with a fractional millimetre. A pulse
     * anchors the estimate without allowing a negative distance jump. */
    const uint64_t accum = (uint64_t)s->distance_frac + (uint64_t)speed * dt;
    s->distance_frac = (uint32_t)(accum % SPEED_DEN);
    s->distance_mm += (uint32_t)(accum / SPEED_DEN);
    if (pulse && !s->ratio_valid && (int32_t)(s->pulse_mm - s->distance_mm) > 0)
        s->distance_mm = s->pulse_mm;
    int64_t accel = 0;
    if (quality == 2u && s->previous_erps >= 3u && s->previous_tick != 0u) {
        const uint32_t period = now_tick - s->previous_tick;
        if (period > 0u && period <= WHEEL_FRESH_TICKS)
            accel = ((int64_t)((int32_t)motor_erps - (int32_t)s->previous_erps) *
                4000000) / ((int32_t)s->previous_erps * (int32_t)period);
        if (accel > 2000) accel = 2000;
        if (accel < -2000) accel = -2000;
    }
    if (now_tick - s->previous_tick >= 400u || s->previous_tick == 0u) {
        s->previous_tick = now_tick;
        s->previous_erps = motor_erps;
    }
    s->out.speed_x100 = speed;
    s->out.distance_mm = s->distance_mm;
    s->out.rel_accel_permille_s = (int16_t)accel;
    s->out.quality = quality;
    return &s->out;
}
