#include "motion_est.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    motion_est_t s;
    motion_est_reset(&s);
    const motion_est_output_t *o;
    o=motion_est_update(&s,4000u,1u,true,4000u,30u,60,100);
    assert(o->quality==0u && o->distance_mm==2218u);
    o=motion_est_update(&s,8000u,1u,true,8000u,30u,60,100);
    assert(o->quality==2u && o->speed_x100>700u && o->speed_x100<850u);
    const uint32_t d=o->distance_mm;
    o=motion_est_update(&s,8400u,400u,true,8000u,30u,0,100);
    assert(o->quality==2u && o->distance_mm>d);
    o=motion_est_update(&s,8800u,400u,true,8000u,36u,0,100);
    assert(o->rel_accel_permille_s>0);
    o=motion_est_update(&s,9200u,400u,true,8000u,32u,0,100);
    assert(o->rel_accel_permille_s<0);
    o=motion_est_update(&s,24000u,400u,false,8000u,0u,0,0);
    assert(o->quality==0u && o->speed_x100==0u);
    motion_est_reset(&s);
    (void)motion_est_update(&s,4000u,1u,true,4000u,30u,60,0);
    o=motion_est_update(&s,8000u,1u,true,8000u,30u,60,0);
    assert(o->quality==1u && o->speed_x100>700u);
    const uint32_t wheel_d=o->distance_mm;
    o=motion_est_update(&s,8400u,400u,true,8000u,30u,0,0);
    assert(o->quality==1u && o->distance_mm>wheel_d);
    puts("motion_est wheel/motor/interpolation/accel/quality PASS");
    return 0;
}
