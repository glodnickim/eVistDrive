#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "assist_modes.h"
#include "assist_pipeline.h"
#include "cadence_filter.h"
#include "config.h"
#include "fast_iq_slew.h"
#include "motor_core.h"
#include "ride_control.h"
#include "rider_input.h"
#include "torque_input.h"
#include "tuning_config.h"

typedef struct {
    double time_s, cadence_rpm, torque_raw_native, torque_ckg;
    double wheel_speed_kph, battery_voltage_v, battery_current_a;
    double assist_level, brake, walk, motor_erps, iq_actual, pas_ab, pas_direction;
    double recorded_iq_request, recorded_iq_ref;
} replay_row_t;

static double field(const char *s) { return (s && *s) ? strtod(s, NULL) : NAN; }

static int split_csv(char *line, char **v, int cap)
{
    int n=0; char *p=line;
    while (n<cap) {
        v[n++]=p;
        char *c=strchr(p, ',');
        if (!c) break;
        *c='\0'; p=c+1;
    }
    for (int i=0;i<n;i++) {
        char *e=v[i]+strlen(v[i]);
        while(e>v[i]&&(e[-1]=='\r'||e[-1]=='\n'))*--e='\0';
    }
    return n;
}

static bool read_row(char *line, replay_row_t *r)
{
    char *v[16]={0}; int n=split_csv(line,v,16);
    if(n<16) return false;
    r->time_s=field(v[0]); r->cadence_rpm=field(v[1]); r->torque_raw_native=field(v[2]);
    r->torque_ckg=field(v[3]); r->wheel_speed_kph=field(v[4]); r->battery_voltage_v=field(v[5]);
    r->battery_current_a=field(v[6]); r->assist_level=field(v[7]); r->brake=field(v[8]); r->walk=field(v[9]);
    r->motor_erps=field(v[10]); r->iq_actual=field(v[11]); r->pas_ab=field(v[12]); r->pas_direction=field(v[13]);
    r->recorded_iq_request=field(v[14]); r->recorded_iq_ref=field(v[15]);
    return isfinite(r->time_s);
}

static uint16_t raw_from_row(const replay_row_t *r)
{
    if (isfinite(r->torque_raw_native)) {
        double x=r->torque_raw_native; if(x<0)x=0; if(x>65535)x=65535; return (uint16_t)llround(x);
    }
    if (isfinite(r->torque_ckg)) {
        double c=r->torque_ckg; if(c<0)c=0; if(c>65535)c=65535;
        uint16_t d=torque_input_centikg_to_native_delta((uint16_t)llround(c));
        uint32_t raw=(uint32_t)TORQUE_ZERO_TARGET_NATIVE+d; return raw>65535U?65535U:(uint16_t)raw;
    }
    return TORQUE_ZERO_TARGET_NATIVE;
}

int main(int argc,char **argv)
{
    if(argc<3){fprintf(stderr,"usage: %s canonical.csv replay_out.csv [max_iq_ref_delta]\n",argv[0]);return 2;}
    double tolerance=argc>=4?strtod(argv[3],NULL):-1.0;
    FILE *in=fopen(argv[1],"rb"); if(!in){perror(argv[1]);return 2;}
    FILE *out=fopen(argv[2],"wb"); if(!out){perror(argv[2]);fclose(in);return 2;}
    char line[4096]; if(!fgets(line,sizeof(line),in)){fprintf(stderr,"empty replay\n");return 2;}
    /*
     * AP-0a: motor voltage utilisation is a CLOSED-LOOP quantity (|Vd,Vq| out of the current
     * regulators, FOC.h _U_MAX domain, 2048 = full scale). This harness has no motor model, so
     * it cannot derive one honestly - and the recorded W1 rides carry neither motor_erps nor
     * iq_actual to reconstruct it from.
     *
     * It used to be hard-wired to 0, which is NOT a neutral choice: at u_abs = 0
     * launch_blend_permille() (src/assist_modes.c) returns 0, so finish_power_request() emits
     * the pure LAUNCH ANCHOR term computed against the fixed ASSIST_LAUNCH_REFERENCE_U_ABS,
     * and the measured-duty branch (power_to_phase_iq with the real u_abs) NEVER EXECUTES.
     * Every number produced that way describes the standstill anchor stretched across a whole
     * ride, not the branch the bike actually runs.
     *
     * So it is now an explicit knob instead of a hidden constant. DEFAULT IS UNCHANGED (0) on
     * purpose: sim/replay/cases/<x>/manifest.json pins accepted_output_sha256, and silently
     * moving the default would invalidate every registered regression without a decision.
     * Set REPLAY_U_ABS to sweep it; the value used is reported on the summary line. Since we
     * do not know the true value, the honest use is a SWEEP that shows how sensitive a
     * conclusion is to it - not a single invented number presented as the operating point.
     */
    int32_t cfg_u_abs=0;
    {
        const char *e=getenv("REPLAY_U_ABS");
        if(e&&*e){ long v=strtol(e,NULL,10); if(v<0)v=0; if(v>2048)v=2048; cfg_u_abs=(int32_t)v; }
    }
    fprintf(out,"time_s,cadence_rpm,torque_ckg,wheel_speed_kph,battery_v,battery_a,iq_request_new,iq_ref_new,iq_request_recorded,iq_ref_recorded,delta_request,delta_ref,debug_flags\n");

    torque_input_init(); torque_input_startup_zero(TORQUE_ZERO_TARGET_NATIVE);
    torque_input_set_run_window_deg(tuning_config_assist_torque_run_window_deg());
    assist_modes_init(); assist_modes_set_active_bank(0U); cadence_filter_reset();
    static MotorState_t ms; motor_core_init(&ms); ride_control_init();
    fast_iq_slew_mailbox_t *mb=ride_control_final_iq_slew_mailbox();

    double prev_t=NAN, pas_frac=0.0, max_req_delta=0.0, max_ref_delta=0.0, sum_ref_delta=0.0;
    uint64_t rows=0, compared=0, forward_steps=0; uint32_t defaults=0;
    while(fgets(line,sizeof(line),in)){
        replay_row_t r; if(!read_row(line,&r))continue;
        double dt=isfinite(prev_t)?r.time_s-prev_t:0.00025; prev_t=r.time_s;
        if(!(dt>0.0)||dt>1.0)dt=0.00025;
        uint32_t elapsed=(uint32_t)llround(dt*4000.0); if(elapsed<1U)elapsed=1U; if(elapsed>4000U)elapsed=4000U;
        double cad=isfinite(r.cadence_rpm)&&r.cadence_rpm>0?r.cadence_rpm:0.0;
        int direction=isfinite(r.pas_direction)?(r.pas_direction>0?1:(r.pas_direction<0?-1:0)):(cad>0?1:0);
        pas_frac += cad * (double)PAS_TRANSITIONS_PER_REV / 60.0 * dt;
        unsigned steps=(unsigned)floor(pas_frac); pas_frac-=steps;
        if(direction>0){for(unsigned i=0;i<steps;i++)torque_input_run_filter_step(); forward_steps+=steps;}
        else if(direction<0)forward_steps=0;

        uint16_t raw=raw_from_row(&r); int16_t corr=torque_input_correct(raw);
        torque_input_update_elapsed(raw,corr,true,elapsed);
        const torque_snapshot_t *ts=torque_input_get_snapshot();
        uint8_t cadence=(uint8_t)(cad>255?255:llround(cad));
        uint32_t speed_x100=isfinite(r.wheel_speed_kph)&&r.wheel_speed_kph>=0?(uint32_t)llround(r.wheel_speed_kph*100.0):1500U;
        double vb=isfinite(r.battery_voltage_v)?r.battery_voltage_v:42.0; if(!isfinite(r.battery_voltage_v))defaults++;
        double ia=isfinite(r.battery_current_a)?r.battery_current_a:0.0;
        int32_t iq_actual=isfinite(r.iq_actual)?(int32_t)llround(r.iq_actual):ms.i_q_setpoint;
        uint16_t erps=isfinite(r.motor_erps)&&r.motor_erps>0?(uint16_t)fmin(r.motor_erps,65535.0):0U;
        bool pedaling=direction>0&&cadence>0;
        bool reverse=direction<0;
        rider_input_t ri={0};
        ri.torque_raw_mv=raw; ri.torque_corrected_mv=corr; ri.torque_filtered=ts->delta_native;
        ri.torque_assist_now_native=ts->assist_delta_native; ri.torque_assist_filtered=ts->assist_delta_filtered_native;
        ri.torque_run_filtered=ts->assist_delta_run_native; ri.torque_load_ctrl=ts->load_ctrl; ri.torque_load_centikg=ts->load_centikg;
        ri.cadence_rpm=cadence; ri.wheel_speed_x100=speed_x100; ri.motor_erps=erps;
        /* AP-0a: THIS is the field finish_power_request() actually reads (assist_modes.c uses
         * rider_input_t.motor_voltage_utilization, not ride_control_input_t.u_abs - the latter
         * only feeds the battery-current cap). The harness never set it at all, so it was a
         * zero-initialised 0 and the launch/measured crossfade could never leave the anchor.
         * Both are driven from the same knob so they cannot silently disagree. */
        ri.motor_voltage_utilization=(uint16_t)cfg_u_abs;
        ri.pas_forward=pedaling; ri.pas_backward=reverse; ri.pedaling_active=pedaling;
        ri.crank_forward_steps=(uint8_t)(forward_steps>250?250:forward_steps); ri.crank_direction_ok=pedaling;
        ri.real_stop=!pedaling&&!reverse; ri.wheel_valid=speed_x100>0; ri.direction_inhibit_active=reverse;
        ri.forward_confirmed_this_tick=false; ri.sample_tick=(uint32_t)rows; ri.start_phase=pedaling&&cadence==0;
        ri.torque_sensor_valid=true; ri.pas_sensor_valid=true; rider_input_update(&ri);

        ride_control_input_t ci={0};
        ci.speed_x100=speed_x100; ci.cadence_rpm=cadence;
        ci.assist_level_index=isfinite(r.assist_level)?(uint8_t)fmax(0,fmin(8,llround(r.assist_level))):3U;
        ci.battery_voltage_mv=(uint32_t)llround(vb*1000.0); ci.iq_scale=PH_CURRENT_MAX;
        ci.ride_core_iq_limit=PH_CURRENT_MAX; ci.phase_current_max=PH_CURRENT_MAX;
        ci.battery_current_mA=(int32_t)llround(ia*1000.0); ci.battery_current_max=BATTERYCURRENT_MAX;
        ci.u_abs=cfg_u_abs; ci.cal_i=CAL_I; ci.current_iq=iq_actual; ci.current_id=0;
        ci.voltage_raw=(uint16_t)llround(vb*1000.0/(double)CAL_BAT_V); ci.voltage_min_raw=VOLTAGE_MIN;
        ci.controller_temperature_c=25; ci.cadence_filtered_x8=(uint16_t)cadence*8U; ci.speed_limit_x100=SPEEDLIMIT;
        ci.legal_enabled=true; ci.offroad=false; ci.walk_active=isfinite(r.walk)&&r.walk!=0.0;
        ci.safety_cut_non_direction=isfinite(r.brake)&&r.brake!=0.0; ci.service_cut_active=false; ci.elapsed_ticks=elapsed;
        ride_control_update(&ci);
        for(uint32_t k=0;k<elapsed*4U;k++)fast_iq_slew_tick(mb,&ms.i_q_setpoint);
        const assist_pipeline_telemetry_t *mo=assist_pipeline_telemetry();
        double dr=isfinite(r.recorded_iq_request)?(double)mo->iq_request_before_limits-r.recorded_iq_request:NAN;
        double df=isfinite(r.recorded_iq_ref)?(double)ms.i_q_setpoint-r.recorded_iq_ref:NAN;
        if(isfinite(dr)&&fabs(dr)>max_req_delta)max_req_delta=fabs(dr);
        if(isfinite(df)){if(fabs(df)>max_ref_delta)max_ref_delta=fabs(df);sum_ref_delta+=fabs(df);compared++;}
        fprintf(out,"%.9f,%.3f,%u,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f,0x%02X\n",
                r.time_s,(double)cadence,(unsigned)ts->load_centikg,(double)speed_x100/100.0,vb,ia,
                mo->iq_request_before_limits,ms.i_q_setpoint,r.recorded_iq_request,r.recorded_iq_ref,dr,df,
                (unsigned)assist_pipeline_reason_bits());
        rows++;
    }
    fclose(in); fclose(out);
    double mae=compared?sum_ref_delta/(double)compared:0.0;
    printf("REPLAY PASS rows=%llu compared=%llu maxReqDelta=%.1f maxRefDelta=%.1f meanAbsRefDelta=%.3f defaults=%u u_abs=%ld%s output=%s\n",
           (unsigned long long)rows,(unsigned long long)compared,max_req_delta,max_ref_delta,mae,defaults,
           (long)cfg_u_abs,cfg_u_abs==0?" (LAUNCH-ANCHOR ONLY - measured-duty branch not exercised)":"",argv[2]);
    if(tolerance>=0.0&&compared&&max_ref_delta>tolerance){
        fprintf(stderr,"REPLAY COMPARE FAIL maxRefDelta %.1f > tolerance %.1f\n",max_ref_delta,tolerance);return 1;
    }
    return rows?0:2;
}
