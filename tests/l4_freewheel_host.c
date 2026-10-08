#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "bike_rider.h"

int main(void)
{
    evd_bike_t bike;
    evd_bike_init(&bike);
    for(unsigned i=0;i<4000U;i++)
        evd_bike_step(&bike,45.0f,30.0f,60.0f,0.00025f);
    const float crank_at_stop=bike.crank_rev;
    const float distance_at_stop=bike.distance_m;
    for(unsigned i=0;i<4000U;i++)
        evd_bike_step(&bike,0.0f,30.0f,0.0f,0.00025f);
    assert(fabsf(bike.crank_rev-crank_at_stop)<0.000001f);
    assert(bike.distance_m>distance_at_stop);
    assert(!bike.crank_freewheel_engaged);
    for(unsigned i=0;i<3000U;i++)
        evd_bike_step(&bike,0.0f,30.0f,-30.0f,0.00025f);
    assert(bike.crank_rev<crank_at_stop-0.25f);
    puts("L4 freewheel motor-only stop and reverse: PASS");
    return 0;
}
