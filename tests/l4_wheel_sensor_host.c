#include <assert.h>
#include <stdio.h>
#include "wheel_sensor.h"
#include "ride_wheel.h"
#include "config.h"

int main(void)
{
    evd_wheel_sensor_t s={0};
    const float circumference_m=(float)WHEEL_CIRCUMFERENCE/1000.0f;
    /* A wheel pulse at 0.8 s seeds timing. The next pulse establishes speed. */
    for(uint32_t tick=1;tick<=6400U;tick++){
        float distance=tick>=6400U?2.01f*circumference_m:
                       tick>=3200U?1.01f*circumference_m:0.0f;
        evd_wheel_sensor_tick(&s,tick,distance);
        if(tick==3200U)assert(s.speed_x100==0U);
    }
    assert(s.speed_x100>=990U && s.speed_x100<=1005U);
    assert(ride_wheel_valid(6400U,s.last_tick));
    for(uint32_t tick=6401U;tick<=6400U+SPEED_STOP_TICKS+1U;tick++)
        evd_wheel_sensor_tick(&s,tick,2.01f*circumference_m);
    assert(s.speed_x100==0U);
    assert(!ride_wheel_valid(6400U+SPEED_STOP_TICKS+1U,s.last_tick));

    /* At 2.5 km/h, successive wheel turns take >2.65 s. The firmware speed
     * therefore reads zero before the next pulse, despite physical motion. */
    evd_wheel_sensor_t slow={0};
    uint32_t period=(uint32_t)(circumference_m/(2.5f/3.6f)*SPEED_TIMEBASE_HZ);
    evd_wheel_sensor_tick(&slow,period,1.01f*circumference_m);
    assert(slow.speed_x100==0U);
    evd_wheel_sensor_tick(&slow,period+SPEED_STOP_TICKS+1U,1.01f*circumference_m);
    assert(slow.speed_x100==0U);
    puts("L4 wheel sensor first pulse, lag, timeout, sub-3 km/h: PASS");
    return 0;
}
