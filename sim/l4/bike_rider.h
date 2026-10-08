#ifndef EVD_L4_BIKE_RIDER_H_
#define EVD_L4_BIKE_RIDER_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float mass_total_kg;
    float wheel_radius_m;
    float chain_gear_ratio;
    float drivetrain_efficiency;
    float crr;
    float cda_m2;
    float air_density;
    float grade;
    float speed_mps;
    float distance_m;
    float crank_rpm;
    float crank_rev;
    float road_force_n;
    float drive_force_n;
    float obstacle_force_n;
    float chainring_rpm;
    bool crank_freewheel_engaged;
} evd_bike_t;

typedef struct {
    bool pedaling;
    float target_cadence_rpm;
    float base_torque_nm;
    float max_torque_nm;
    float cadence_kp_nm_per_rpm;
    float leg_ripple;
    float left_right_asymmetry;
    float crank_length_m;
    float torque_nm;
    float torque_ckg;
} evd_rider_t;

void evd_bike_init(evd_bike_t *b);
void evd_rider_init(evd_rider_t *r);
void evd_rider_step(evd_rider_t *r, const evd_bike_t *b);
void evd_bike_step(evd_bike_t *b, float rider_torque_nm, float motor_crank_torque_nm,
                   float rider_target_rpm, float dt_s);
uint32_t evd_bike_speed_x100(const evd_bike_t *b);

#endif
