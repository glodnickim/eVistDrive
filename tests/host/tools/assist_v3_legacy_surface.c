/* Design-time reference only. Link the shipped production G53 and intent modules. */
#include "g53_port_chain.h"
#include "assist_v3_intent.h"
#include <stdint.h>
#include <stdio.h>

static const uint16_t efforts[] = {200,300,450,600,800,1000,1200,1500,1800,2200,2600,3000,3500,4000};
static const uint16_t cadences[] = {20,25,30,40,50,60,70,80,90,100,110,120,130};

int main(void)
{
    const uint16_t *prior;
    g53_chain_reset();
    assist_v3_intent_power_on();
    prior = assist_v3_intent_prior();
    puts("level,cadence_rpm,intent_clu,kl_q12,env_equiv,target_e2,iq_p700,rider_proxy_clu_rpm,rider_w_assumed_165mm,motor_proxy_iq_v48,auto_active");
    for (unsigned level=1; level<=5; ++level)
    for (unsigned c=0; c<sizeof(cadences)/sizeof(cadences[0]); ++c)
    for (unsigned i=0; i<sizeof(efforts)/sizeof(efforts[0]); ++i) {
        uint16_t effort=efforts[i], cad=cadences[c], env_ss=0;
        uint16_t kl=assist_v3_intent_compute_kl(prior,effort,cad,820,&env_ss,0);
        g53_static_ratio_state_t state;
        g53_static_diag_t diag;
        g53_static_input_t in={0};
        if (!kl) kl=4096;
        in.env=assist_v3_eb74_active(((uint32_t)effort*kl)/4096,820);
        in.cadence=(int16_t)cad;
        in.lut_cadence=cad;
        in.level=g53_chain_level((uint8_t)level);
        g53_static_ratio_init(&state);
        uint16_t e2=g53_static_target(&in,&state,100000,&diag);
        double iq=(double)e2*455.0/40960.0;
        /* Same integer order as rider_power_w(), at assumed 165 mm crank and CLU=centikg. */
        uint64_t mw=((uint64_t)effort*cad*1694u)/1000u;
        uint32_t rider_w=(uint32_t)(mw/1000u);
        printf("%u,%u,%u,%u,%u,%u,%.6f,%u,%u,%.6f,%u\n",level,cad,effort,kl,in.env,e2,iq,
               (unsigned)(effort*cad),rider_w,iq*48.0,diag.auto_active);
    }
    return 0;
}
