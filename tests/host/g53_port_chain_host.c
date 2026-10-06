#include "g53_port_chain.h"
#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    /* The pinned G5300 transcription vector predates configurable M560 levels. */
    const uint8_t accel[10]={1,8,8,8,8,8,8,8,8,8};
    const uint16_t ratio[10]={1,45,95,155,215,260,310,370,525,525};
    g53_chain_set_levels(accel,ratio);
    const char *path="integration/evidence/evd-tq/TQ-06/host/chain/chain-reference.csv";
    FILE *f=fopen(path,"r");
    if(!f) { perror(path); return 1; }
    char line[4096];
    if(!fgets(line,sizeof(line),f)) return 1;
    unsigned rows=0;
    g53_pas_ctx_t pas;
    while(fgets(line,sizeof(line),f)) {
        long long v[128]; char *p=line;
        for(unsigned i=0;i<128;++i) {
            char *end; v[i]=strtoll(p,&end,10);
            if(end==p || (i<127 && *end!=',')) return 1;
            p=end+1;
        }
        if(v[0]) { g53_chain_reset(); g53_pas_reset(&pas,true,0); }
        g53_chain_input_t in={0};
        in.pas=g53_pas_step(&pas,(uint8_t)v[1],(uint8_t)v[2]);
        in.level=(uint8_t)v[2]; in.x=(uint16_t)v[3]; in.rider_input_native=(uint16_t)v[4];
        in.speed_native=(int16_t)v[5]; in.diag_word=(uint32_t)v[6]; in.external_inhibit=(uint8_t)v[7];
        in.g1_q12=4096; in.g2_q12=4096;
        g53_chain_output_t out;
        g53_chain_step(&in,&out);
        const int64_t actual[]={
            out.trace.logical_tick,
            out.trace.fast_phase,
            out.trace.supervisor_phase,
            out.trace.phases_executed,
            out.trace.x,
            out.trace.fsm_state,
            out.trace.fsm_output,
            out.trace.fsm_t34,
            out.trace.fsm_t36,
            out.trace.fsm_t38,
            out.trace.fsm_t3a,
            out.trace.fsm_t3c,
            out.trace.fsm_t40,
            out.trace.fsm_t44,
            out.trace.fsm_t46,
            out.trace.fsm_t48,
            out.trace.fsm_t4a,
            out.trace.fsm_t4b,
            out.trace.fsm_t4c,
            out.trace.fsm_t4d,
            out.trace.fsm_g1fe,
            out.trace.fsm_g1ff,
            out.trace.fsm_g200,
            out.trace.fsm_g203,
            out.trace.pas_direction,
            out.trace.pas_direction_candidate,
            out.trace.cadence,
            out.trace.evidence,
            out.trace.movement,
            out.trace.base_timeout,
            out.trace.full_timeout,
            out.trace.max_timeout,
            out.trace.no_transition,
            out.trace.pas_code,
            out.trace.pas_current_delta,
            out.trace.pas_previous_delta,
            out.trace.pas_transition_count,
            out.trace.pas_elapsed,
            out.trace.pas_span,
            out.trace.pas_magnitude,
            out.trace.pas_raw_cadence,
            out.trace.pas_filter_accumulator,
            out.trace.pas_filtered_cadence,
            out.trace.pas_plausibility_budget,
            out.trace.pas_plausibility_base,
            out.trace.pas_plausibility_latch,
            out.trace.pas_plausibility_previous_latch,
            out.trace.pas_plausibility_flag,
            out.trace.pas_anti_rock,
            out.trace.d7ec_mode,
            out.trace.d7ec_selected_current,
            out.trace.d7ec_speed,
            out.trace.d7ec_m50,
            out.trace.d7ec_m52,
            out.trace.d7ec_m29f,
            out.trace.d7ec_base_timeout,
            out.trace.d7ec_no_transition,
            out.trace.d7ec_max_timeout,
            out.trace.d7ec_envelope,
            out.trace.d7ec_readiness,
            out.trace.d7ec_rider_intermediate,
            out.trace.d7ec_rider,
            out.trace.d7ec_assist_ratio,
            out.trace.d7ec_ratio,
            out.trace.d7ec_taper,
            out.trace.d7ec_requested,
            out.trace.d7ec_lut,
            out.trace.d7ec_converted,
            out.trace.d7ec_hold,
            out.trace.d7ec_accel_target,
            out.trace.d7ec_accel_live,
            out.trace.d7ec_accel_rise,
            out.trace.d7ec_accel,
            out.trace.d7ec_accel_window,
            out.trace.d7ec_accel_counter,
            out.trace.d7ec_history_delta,
            out.trace.d7ec_history_a,
            out.trace.d7ec_history_b,
            out.trace.d7ec_history_selected,
            out.trace.d7ec_external_guard,
            out.trace.d7ec_d19,
            out.trace.e1e8_state,
            out.trace.e1e8_factor,
            out.trace.e1e8_taper,
            out.trace.e1e8_pi,
            out.trace.e1e8_target,
            out.trace.e1e8_output,
            out.trace.e1e8_output_factor,
            out.trace.e1e8_taper_factor,
            out.trace.e1e8_latch,
            out.trace.bde8_q50,
            out.trace.bde8_q5a,
            out.trace.bde8_q5c,
            out.trace.bde8_mode,
            out.trace.bde8_demand,
            out.trace.bde8_angle,
            out.trace.bde8_g04,
            out.trace.g1,
            out.trace.g2,
            out.trace.m298,
            out.trace.m299,
            out.trace.m28,
            out.trace.m2a,
            out.trace.m2c,
            out.trace.m34,
            out.trace.m40,
            out.trace.m50,
            out.trace.m52,
            out.trace.m29f,
            out.trace.m2a2,
            out.trace.m2a4,
            out.trace.m2a8,
            out.trace.m2aa,
            out.trace.m2ac,
            out.trace.m2ae,
            out.trace.m2b0,
            out.trace.diag,
            out.trace.external_inhibit,
            out.trace.fatal_fault,
            out.trace.nonfatal_fault
        };
        const char *names[]={"logical_tick","fast_phase","supervisor_phase","phases_executed","x","fsm_state","fsm_output","fsm_t34","fsm_t36","fsm_t38","fsm_t3a","fsm_t3c","fsm_t40","fsm_t44","fsm_t46","fsm_t48","fsm_t4a","fsm_t4b","fsm_t4c","fsm_t4d","fsm_g1fe","fsm_g1ff","fsm_g200","fsm_g203","pas_direction","pas_direction_candidate","cadence","evidence","movement","base_timeout","full_timeout","max_timeout","no_transition","pas_code","pas_current_delta","pas_previous_delta","pas_transition_count","pas_elapsed","pas_span","pas_magnitude","pas_raw_cadence","pas_filter_accumulator","pas_filtered_cadence","pas_plausibility_budget","pas_plausibility_base","pas_plausibility_latch","pas_plausibility_previous_latch","pas_plausibility_flag","pas_anti_rock","d7ec_mode","d7ec_selected_current","d7ec_speed","d7ec_m50","d7ec_m52","d7ec_m29f","d7ec_base_timeout","d7ec_no_transition","d7ec_max_timeout","d7ec_envelope","d7ec_readiness","d7ec_rider_intermediate","d7ec_rider","d7ec_assist_ratio","d7ec_ratio","d7ec_taper","d7ec_requested","d7ec_lut","d7ec_converted","d7ec_hold","d7ec_accel_target","d7ec_accel_live","d7ec_accel_rise","d7ec_accel","d7ec_accel_window","d7ec_accel_counter","d7ec_history_delta","d7ec_history_a","d7ec_history_b","d7ec_history_selected","d7ec_external_guard","d7ec_d19","e1e8_state","e1e8_factor","e1e8_taper","e1e8_pi","e1e8_target","e1e8_output","e1e8_output_factor","e1e8_taper_factor","e1e8_latch","bde8_q50","bde8_q5a","bde8_q5c","bde8_mode","bde8_demand","bde8_angle","bde8_g04","g1","g2","m298","m299","m28","m2a","m2c","m34","m40","m50","m52","m29f","m2a2","m2a4","m2a8","m2aa","m2ac","m2ae","m2b0","diag","external_inhibit","fatal_fault","nonfatal_fault"};
        for(unsigned i=0;i<120;++i) if(actual[i]!=v[i+8]) {
            fprintf(stderr,"Chain row %u %s: actual=%lld expected=%lld\n",rows+1,names[i],(long long)actual[i],v[i+8]); return 1;
        }
        if(out.m2aa_native!=(uint16_t)out.trace.m2aa || out.normal_permission!=(out.trace.m298==2)) return 1;
        ++rows;
    }
    if(ferror(f) || rows!=39600) return 1;
    fclose(f);
    printf("G53 chain: %u rows x 120 fields, zero mismatches PASS\n",rows);
    return 0;
}
