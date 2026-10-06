/* TASK-EVD-M560-AUTO-SPLUS-001: G5300 D7EC AUTO and production Iq path. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "g53_port_chain.h"
#include "assist_pipeline.h"
#include "assist_modes.h"
#include "fast_iq_slew.h"
#include "g53_port.h"

static const uint8_t accel[10]={1,4,4,5,5,6,6,7,8,8};
static const uint16_t ratio[10]={1,45,95,155,215,260,310,370,525,525};
static const uint8_t pas_cycle[4]={0,2,3,1};

static g53_chain_output_t chain_run(uint8_t hmi,uint16_t rider,unsigned ticks)
{
    g53_chain_input_t in={0};
    g53_chain_output_t out={0};
    in.level=g53_chain_level(hmi);
    in.pas.cadence=60;
    in.pas.evidence=10;
    in.pas.movement=1;
    in.pas.base_timeout_ticks=200;
    in.pas.max_timeout_ticks=500;
    in.rider_input_native=rider;
    in.g1_q12=4096;
    in.g2_q12=4096;
    in.speed_native=200;
    while(ticks--) g53_chain_step(&in,&out);
    return out;
}

static void chain_scenarios(void)
{
    g53_chain_output_t o;
    uint8_t enable,step;
    uint16_t scale;
    static const unsigned levels[5]={1,2,3,4,5};
    static const unsigned slots[5]={2,4,6,8,9};
    unsigned samples=0;
    for(unsigned i=0;i<5;i++) {
        g53_chain_reset();
        o=chain_run((uint8_t)levels[i],700,1500);
        if(i==3) {
            assert(o.trace.d7ec_assist_ratio < ratio[8]);
            assert(o.trace.d7ec_assist_ratio > 1);
        } else assert(o.trace.d7ec_assist_ratio == ratio[slots[i]]);
        printf("a HMI%u slot%u C4=%ld desired=%ld\n",levels[i],slots[i],
            (long)o.trace.d7ec_rider,(long)o.trace.d7ec_assist_ratio);
    }
    static const uint16_t loads[5]={0,200,750,1500,3000};
    for(unsigned i=0;i<5;i++) {
        g53_chain_reset();
        o=chain_run(4,loads[i],1500);
        const unsigned c4=(unsigned)o.trace.d7ec_rider;
        unsigned expected=1+(524u*c4)/200u;
        if(expected>525) expected=525;
        assert((unsigned)o.trace.d7ec_assist_ratio==expected);
        printf("b load=%u C4=%u desired=%u\n",loads[i],c4,expected);
        samples++;
    }
    assert(samples==5);
    {
        uint16_t edited[10];
        memcpy(edited,ratio,sizeof(edited)); edited[8]=400;
        g53_chain_set_levels(accel,edited);
        g53_chain_reset(); o=chain_run(4,3000,1500);
        assert(o.trace.d7ec_assist_ratio==400);
        printf("b edited slot8 max=%ld\n",(long)o.trace.d7ec_assist_ratio);
        g53_chain_set_levels(accel,ratio);
    }
    g53_chain_reset();
    o=chain_run(4,0,100);
    int32_t last=o.trace.d7ec_ratio;
    for(unsigned i=0;i<700;i++) {
        o=chain_run(4,3000,1);
        if(o.trace.phases_executed&8) {
            assert(o.trace.d7ec_ratio-last<=10);
            last=o.trace.d7ec_ratio;
        }
    }
    /* Move the desired ratio far below the live value. G5300 limits only rises. */
    g53_chain_set_auto(1,655,10);
    o=chain_run(4,3000,10);
    assert(last-o.trace.d7ec_ratio>10);
    assert(o.trace.d7ec_ratio==o.trace.d7ec_assist_ratio);
    printf("c rise cap=10/10ms, fall %ld->%ld\n",(long)last,(long)o.trace.d7ec_ratio);
    g53_chain_set_auto(1,3,7);
    g53_chain_reset(); g53_chain_get_auto(&enable,&scale,&step);
    assert(enable==1 && scale==3 && step==7);
    g53_port_init(); g53_chain_get_auto(&enable,&scale,&step);
    assert(enable==1 && scale==3 && step==7);
    g53_chain_set_auto(1,0,10);
    g53_chain_get_auto(&enable,&scale,&step);
    assert(scale==1);
    g53_chain_set_auto(1,2,10);
    puts("d chain reset and port init retain AUTO 1/3/7; scale zero clamps to one");
}

static assist_pipeline_input_t base_input(uint16_t load)
{
    assist_pipeline_input_t in={0};
    in.torque_load_ctrl=load;
    in.torque_sensor_valid=true;
    in.pas_sensor_valid=true;
    in.forward_valid=true;
    in.wheel_valid=true;
    in.assist_level_index=4;
    in.cadence_rpm=60;
    in.speed_x100=2000;
    in.phase_current_max=900;
    in.battery_voltage_mv=42000;
    in.battery_current_max=15000;
    in.u_abs=1024;
    in.cal_i=95;
    in.voltage_raw=4000;
    in.voltage_min_raw=2800;
    in.controller_temperature_c=30;
    in.speed_limit_x100=6000;
    in.elapsed_ticks=4;
    return in;
}

static unsigned crossing_up(const int32_t *v,unsigned from,unsigned to,int32_t threshold)
{
    for(unsigned t=from;t<to;t++) if(v[t]>=threshold) return t-from;
    return to-from;
}
static int crossing_down(const int32_t *v,unsigned from,unsigned to,int32_t threshold)
{
    for(unsigned t=from;t<to;t++) if(v[t]<=threshold) return (int)(t-from);
    return -1;
}

static void timing_case(uint8_t auto_on,uint16_t load)
{
    enum { END=6500, ENGAGE=700, PEDAL_RELEASE=2500, PEDAL_RESUME=3300, CRANK_STOP=4400 };
    static int32_t iq[END];
    fast_iq_slew_mailbox_t mb={0};
    int32_t ref=0,peak=0,stop_ref;
    assist_pipeline_input_t in=base_input(load);
    assist_pipeline_command_t cmd;
    g53_chain_set_auto(auto_on,2,10);
    assist_modes_init(); g53_port_init(); assist_pipeline_init(); fast_iq_slew_reset(&mb);
    for(unsigned t=0;t<END;t++) {
        in.speed_x100=t<1000 ? 0 : 2000;
        in.torque_load_ctrl=(t<ENGAGE || (t>=PEDAL_RELEASE && t<PEDAL_RESUME)) ? 0 : load;
        in.pas_ab=t<ENGAGE ? 0 :
            pas_cycle[(((t>=CRANK_STOP ? CRANK_STOP-1u : t)-ENGAGE)*12u/125u)%4u];
        assist_pipeline_update(&in,&cmd);
        fast_iq_slew_publish(&mb,cmd.final_iq_request,cmd.slew_mode,cmd.step_mag_8,
            cmd.release_ticks_16k,cmd.zero_policy,cmd.iq_ceiling);
        for(unsigned n=0;n<16;n++) fast_iq_slew_tick(&mb,&ref);
        iq[t]=ref;
        if(t>=ENGAGE && t<PEDAL_RELEASE && ref>peak) peak=ref;
    }
    if(peak==0) { printf("e %s load=%u peak=0 (no engagement)\n",auto_on ? "AUTO" : "0.636",load); return; }
    stop_ref=iq[CRANK_STOP];
    const unsigned e10=crossing_up(iq,ENGAGE,PEDAL_RELEASE,(peak+9)/10);
    const unsigned e50=crossing_up(iq,ENGAGE,PEDAL_RELEASE,(peak+1)/2);
    const unsigned e90=crossing_up(iq,ENGAGE,PEDAL_RELEASE,(9*peak+9)/10);
    const int32_t release_ref=iq[PEDAL_RELEASE];
    const int r90=crossing_down(iq,PEDAL_RELEASE,PEDAL_RESUME,(9*release_ref)/10);
    const int r50=crossing_down(iq,PEDAL_RELEASE,PEDAL_RESUME,release_ref/2);
    const int r10=crossing_down(iq,PEDAL_RELEASE,PEDAL_RESUME,release_ref/10);
    const int rzero=crossing_down(iq,PEDAL_RELEASE,PEDAL_RESUME,0);
    const int s90=crossing_down(iq,CRANK_STOP,END,(9*stop_ref)/10);
    const int s50=crossing_down(iq,CRANK_STOP,END,stop_ref/2);
    const int s10=crossing_down(iq,CRANK_STOP,END,stop_ref/10);
    const int szero=crossing_down(iq,CRANK_STOP,END,0);
    assert(e10<=e50 && e50<=e90 && e90<PEDAL_RELEASE-ENGAGE);
    assert(stop_ref>0 && s90>=0 && s90<=s50 && s50<=s10 && s10<=szero && szero<END-CRANK_STOP);
    printf("e %s load=%u peak=%ld engage=%u/%u/%u pedal_release=%ld->%ld t90/50/10/0=%d/%d/%d/%d stop_ref=%ld stop=%d/%d/%d/%d ms\n",
        auto_on ? "AUTO" : "0.636",load,(long)peak,e10,e50,e90,
        (long)release_ref,(long)iq[PEDAL_RESUME-1],r90,r50,r10,rzero,
        (long)stop_ref,s90,s50,s10,szero);
}

int main(void)
{
    chain_scenarios();
    for(unsigned i=0;i<3;i++) {
        static const uint16_t loads[3]={1500,3000,6000};
        timing_case(0,loads[i]);
        timing_case(1,loads[i]);
    }
    g53_chain_set_auto(1,2,10);
    puts("M560 AUTO S+ scenarios a-e: PASS");
    return 0;
}
