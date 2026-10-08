#ifndef EVD_L4_WHEEL_SENSOR_H_
#define EVD_L4_WHEEL_SENSOR_H_
#include <stdint.h>

typedef struct {
    uint32_t last_tick;
    uint32_t last_valid_x100;
    uint32_t cumulative_x100;
    uint32_t speed_x100;
    uint32_t pulse_count;
    uint8_t have_reference;
} evd_wheel_sensor_t;

void evd_wheel_sensor_tick(evd_wheel_sensor_t *s, uint32_t tick, float distance_m);
#endif
