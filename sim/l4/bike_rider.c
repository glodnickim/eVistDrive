#include "bike_rider.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float clampf_local(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void evd_bike_init(evd_bike_t *b)
{
    memset(b, 0, sizeof(*b));
    b->mass_total_kg = 112.0f;      /* rider + bike test default, scenario may override */
    b->wheel_radius_m = 0.35f;
    b->chain_gear_ratio = 2.10f;    /* wheel rpm / crank rpm */
    b->drivetrain_efficiency = 0.94f;
    b->crr = 0.008f;
    b->cda_m2 = 0.58f;
    b->air_density = 1.225f;
}

void evd_rider_init(evd_rider_t *r)
{
    memset(r, 0, sizeof(*r));
    r->pedaling = true;
    r->target_cadence_rpm = 60.0f;
    r->base_torque_nm = 18.0f;
    /* Virtual rider envelope: about 84 kgf at a 170 mm crank (~140 Nm),
     * matching the top calibrated torque-sensor region rather than artificially
     * limiting steep-hill starts to 75 Nm. Scenario base torque remains much lower. */
    r->max_torque_nm = 140.0f;
    /* Cadence-hold effort of the virtual rider. At near-zero cadence the rider
     * deliberately produces a strong launch push, then backs off as target cadence is reached. */
    r->cadence_kp_nm_per_rpm = 1.00f;
    r->leg_ripple = 0.40f;
    r->left_right_asymmetry = 0.08f;
    r->crank_length_m = 0.17f;
}

void evd_rider_step(evd_rider_t *r, const evd_bike_t *b)
{
    if (!r->pedaling) {
        r->torque_nm = 0.0f;
        r->torque_ckg = 0.0f;
        return;
    }
    float cadence_error = r->target_cadence_rpm - b->crank_rpm;
    float mean = r->base_torque_nm + r->cadence_kp_nm_per_rpm * cadence_error;
    mean = clampf_local(mean, 0.0f, r->max_torque_nm);

    /* Two leg pushes per revolution plus one-revolution L/R imbalance. The floor at zero
     * naturally creates dead spots under aggressive ripple/asymmetry. */
    float phase = (float)(2.0 * M_PI) * b->crank_rev;
    float shape = 1.0f + r->leg_ripple * sinf(2.0f * phase) +
                  r->left_right_asymmetry * sinf(phase);
    if (shape < 0.0f) shape = 0.0f;
    r->torque_nm = clampf_local(mean * shape, 0.0f, r->max_torque_nm);
    if (r->crank_length_m > 0.01f) {
        r->torque_ckg = r->torque_nm / (9.80665f * r->crank_length_m) * 100.0f;
    } else {
        r->torque_ckg = 0.0f;
    }
}

void evd_bike_step(evd_bike_t *b, float rider_torque_nm, float motor_crank_torque_nm,
                   float rider_target_rpm, float dt_s)
{
    const float g = 9.80665f;
    float v = b->speed_mps;
    float rolling = b->mass_total_kg * g * b->crr;
    float aero = 0.5f * b->air_density * b->cda_m2 * v * v;
    float grade = b->mass_total_kg * g * b->grade;
    b->road_force_n = rolling + aero + grade + b->obstacle_force_n;

    /* The motor drives the chainring independently. The crank pawl engages only
     * while the rider drives forward at chainring speed or faster. */
    b->crank_freewheel_engaged = rider_target_rpm > 0.0f && rider_torque_nm > 0.0f &&
                                 b->crank_rpm + 0.01f >= b->chainring_rpm;
    float crank_torque = (b->crank_freewheel_engaged ? rider_torque_nm : 0.0f) + motor_crank_torque_nm;
    if (crank_torque < 0.0f) crank_torque = 0.0f;
    float wheel_torque = b->chain_gear_ratio > 0.01f ?
        crank_torque / b->chain_gear_ratio * b->drivetrain_efficiency : 0.0f;
    b->drive_force_n = wheel_torque / b->wheel_radius_m;

    float accel = (b->drive_force_n - b->road_force_n) / b->mass_total_kg;
    b->speed_mps += accel * dt_s;
    if (b->speed_mps < 0.0f) b->speed_mps = 0.0f;
    b->distance_m += b->speed_mps * dt_s;

    if (b->wheel_radius_m > 0.01f && b->chain_gear_ratio > 0.01f) {
        float wheel_rps = b->speed_mps / ((float)(2.0 * M_PI) * b->wheel_radius_m);
        b->chainring_rpm = wheel_rps * 60.0f / b->chain_gear_ratio;
    }
    if (rider_target_rpm > 0.0f && b->crank_freewheel_engaged)
        b->crank_rpm = b->chainring_rpm;
    else {
        /* A released crank stops promptly; a scripted back pedal can rotate
         * independently of a forward moving chainring. */
        float target = rider_target_rpm;
        if (target == 0.0f) b->crank_rpm = 0.0f;
        else {
            float decay = dt_s / (0.08f + dt_s);
            b->crank_rpm += decay * (target - b->crank_rpm);
            if (target > 0.0f && b->crank_rpm >= b->chainring_rpm &&
                rider_torque_nm > 0.0f) b->crank_rpm = b->chainring_rpm;
        }
    }
    b->crank_rev += b->crank_rpm / 60.0f * dt_s;
}

uint32_t evd_bike_speed_x100(const evd_bike_t *b)
{
    float x = b->speed_mps * 3.6f * 100.0f;
    if (x < 0.0f) x = 0.0f;
    if (x > 4294967295.0f) x = 4294967295.0f;
    return (uint32_t)lroundf(x);
}
