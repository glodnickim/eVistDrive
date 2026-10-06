#include "g53_port.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    const uint16_t expected[9]={163,163,187,220,265,335,455,706,1575};
    const uint8_t accel[10]={1,8,1,7,4,6,8,3,8,8};
    const uint16_t ratio[10]={1,45,1000,155,800,260,120,370,0,525};
    const uint8_t power[10]={0,15,85,30,60,60,40,70,10,99};
    uint8_t a[10]={1,1,2,3,4,5,6,7,8,8};
    uint16_t r[10]={1,45,95,155,215,260,310,370,525,525};
    g53_port_init();
    assert(g53_chain_rise_step(0)==163);
    assert(g53_chain_rise_step(4)==335);
    assert(g53_chain_rise_step(8)==1575);
    assert(g53_chain_rise_step(9)==1575);
    for(unsigned n=1;n<=8;n++) {
        a[2]=(uint8_t)n;
        g53_chain_set_levels(a,r);
        assert(g53_chain_rise_step(2)==expected[n]);
    }
    g53_port_set_levels(accel,ratio,power);
    assert(g53_chain_rise_step(2)==163);
    assert(g53_chain_ratio(2)==1000);
    assert(g53_chain_rise_step(4)==265);
    assert(g53_chain_ratio(4)==800);
    assert(g53_chain_ratio(8)==0);
    assert(g53_chain_rise_step(9)==1575);
    assert(g53_chain_ratio(9)==525);
    g53_chain_reset();
    assert(g53_chain_rise_step(2)==163 && g53_chain_ratio(2)==1000);
    assert(g53_chain_rise_step(4)==265 && g53_chain_ratio(8)==0);
    for(unsigned hmi=1;hmi<=5;hmi++) {
        static const uint8_t slots[5]={2,4,6,8,9};
        g53_port_input_t in={0};
        g53_port_output_t out;
        in.assist_level=(uint8_t)hmi;
        in.elapsed_ticks=40;
        in.battery_limit_centiamp=1500;
        in.torque_sensor_valid=true;
        g53_port_update(&in,&out);
        assert(g53_port_g1_state()->level_pct==power[slots[hmi-1]]);
    }
    puts("M560 G53 level tables and power: PASS");
    return 0;
}
