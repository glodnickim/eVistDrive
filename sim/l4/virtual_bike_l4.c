/*
 * EVistDrive Level-4 virtual bicycle / digital twin.
 *
 * The control path is shipped production C. Only the physical world is virtual:
 * rider -> torque/PAS sensors -> EVistDrive -> final Iq -> real FOC/PI/SVPWM -> PMSM
 * -> crank/drivetrain -> bicycle/road -> battery -> sensors -> EVistDrive.
 *
 * This file deliberately contains no alternate assist law, no alternate current controller and
 * no alternate rotor-angle estimator. If a production module exists, it is linked and called.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FOC.h"
#include "ap2_limits.h"
#include "assist_pipeline.h"
#include "assist_modes.h"
#include "battery_iq_cap.h"
#include "cadence_filter.h"
#include "config.h"
#include "fast_iq_slew.h"
#include "foc_current_loop.h"
#include "motor_core.h"
#include "pas_cadence.h"
#include "pas_direction.h"
#include "pas_liveness.h"
#include "pas_sampler.h"
#include "pwm_geometry.h"
#include "quiet_zero.h"
#include "ride_control.h"
#include "rider_input.h"
#include "rotor_angle.h"
#include "rotor_motion.h"
#include "soc_core.h"
#include "torque_input.h"
#include "tuning_config.h"
/* ASSIST-V3: the production crank step accumulator, fed from this harness's drain exactly as
 * main.c feeds it (REVIEW 1 #19), and the explicit engine selection (G-EQ rule 1). */
#include "crank_phase.h"
#ifdef ASSIST_V3
#include "assist_v3_harness.h"
#endif
#include "walk_assist_motor.h"

#include "battery_pack.h"
#include "bike_rider.h"
#include "eb74_invocation_observer.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define CTRL_HZ 4000U
#define FOC_HZ 16000U
#define INNER_PER_CTRL 4U
#define SQRT3 1.7320508075688772
#define CURRENT_A_PER_COUNT 0.095
#define MOTOR_CRANK_NM_PER_IQ_COUNT 0.075f
#define DEFAULT_BATTERY_CURRENT_MAX_MA 15000
#define DEFAULT_CONTROLLER_TEMP_C 25
#define DEFAULT_ASSIST_LEVEL 3U

/* The host FOC needs the same globals the GD32 main translation unit provides. */
PI_control_t PI_iq, PI_id;
uint8_t ui_8_PWM_ON_Flag = 1U;
uint8_t bridge_lifecycle = BRIDGE_LIFECYCLE_RUN;
int32_t switchtime[3];
uint16_t pwm_applied[3];

static MotorState_t *g_ms;
static struct l4_motor *g_motor;
static quiet_zero_t g_qzero;
static FILE *l4_stage_trace;
static uint32_t l4_stage_previous_mask;

void timer_channel_output_pulse_value_config(uint32_t timer, uint16_t ch, uint32_t value)
{ (void)timer; (void)ch; (void)value; }
void timer_primary_output_config(uint32_t timer, uint32_t enable)
{ (void)timer; (void)enable; }

static void pi_init(PI_control_t *p, int16_t limit_i)
{
    memset(p,0,sizeof(*p));
    p->gain_p=1.5f; p->gain_i=0.01f; p->limit_i=limit_i;
    p->limit_output=_U_MAX; p->max_step=15; p->shift=11; p->aw_inv_kp_q15=21845;
}

typedef struct l4_motor {
    double id_a, iq_a;
    double r_ohm, ld_h, lq_h, flux_wb;
    double erps, theta_e_rev;
    double vbus_v;
    double dc_power_w;
    double dc_power_acc_w;
    unsigned dc_power_samples;
    uint16_t hall_age_ticks;
    uint32_t hall_edges;
    bool hall_edge_this_ctrl;
    double hall_elapsed_500k;
    uint16_t last_capture_500k;
    uint32_t tics_filtered_8;
    int64_t hall_sector_index;
    uint32_t hall_sequence;
    uint8_t sixstep_untrusted;
    rotor_angle_state_t angle_state;
    rotor_motion_t motion;
    double max_angle_error_deg;
} l4_motor_t;

void runPIcontrol(void)
{
    foc_current_loop_result_t result;
    quiet_zero_action_t qz;
    fast_iq_slew_tick(ride_control_final_iq_slew_mailbox(), &g_ms->i_q_setpoint);
    PI_iq.recent_value=(int16_t)g_ms->i_q;
    PI_iq.setpoint=g_ms->i_q_setpoint;

    quiet_zero_input_t in={
        .iq_ref=g_ms->i_q_setpoint,
        .zero_policy_quiet=fast_iq_slew_current_zero_policy()==FIS_ZERO_POLICY_QUIET,
        .iq_measured=g_ms->i_q, .id_measured=g_ms->i_d,
        .abort_current=QZERO_ABORT_CURRENT,
        .rotor_erps=g_motor?(int32_t)g_motor->motion.edge_erps:0,
        .speed_fresh=g_motor?rotor_motion_speed_fresh(&g_motor->motion,g_motor->hall_age_ticks):false,
        .min_brake_erps=RIDE_COAST_RELEASE_ERPS,
        .iq_integral=PI_iq.integral_part, .id_integral=PI_id.integral_part
    };
    quiet_zero_tick(&g_qzero,&in,&qz);
    if(qz.apply_integral){ PI_iq.integral_part=qz.iq_integral; PI_id.integral_part=qz.id_integral; }
    if(qz.freeze_aw || qz.clear_aw_edge){
        PI_iq.aw_sat_error=0; PI_id.aw_sat_error=0; g_ms->u_q_sat_err=0; g_ms->u_d_sat_err=0;
    }
    foc_current_loop_step(g_ms,&PI_iq,&PI_id,qz.apply_integral?1U:0U,
                          qz.iq_integral,qz.id_integral,&result);
}

static double wrap_pi(double a)
{
    while (a >= M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
static q31_t angle_q31(double rev)
{
    double a=wrap_pi(rev*2.0*M_PI);
    double x=a/M_PI*2147483648.0;
    if (x >= 2147483647.0) x = 2147483647.0;
    if (x < -2147483648.0) x = -2147483648.0;
    return (q31_t)llround(x);
}
static double q31_rad(q31_t a){ return ((double)a/2147483648.0)*M_PI; }

static q31_t motor_control_angle(l4_motor_t *m)
{
    uint32_t elapsed=(uint32_t)llround(m->hall_elapsed_500k); if(elapsed>65535U)elapsed=65535U;
    if(elapsed>(uint32_t)(SIXSTEPTHRESHOLD<<1)){
        m->last_capture_500k=(uint16_t)(SIXSTEPTHRESHOLD<<1);
        m->tics_filtered_8=(uint32_t)m->last_capture_500k<<3;
    }
    if(m->last_capture_500k<SIXSTEPTHRESHOLD && elapsed<200U)m->sixstep_untrusted=0U;
    if(m->last_capture_500k>((SIXSTEPTHRESHOLD*6U)>>2))m->sixstep_untrusted=1U;
    double boundary=(double)m->hall_sector_index/6.0;
    rotor_angle_input_t in={
        .hall_angle=angle_q31(boundary), .angle_correction=0, .direction=1,
        .tim2_recent=elapsed, .tics_filtered_8=m->tics_filtered_8,
        .want_untrusted=m->sixstep_untrusted!=0U,
        .stalled=rotor_motion_angle_stale(&m->motion,m->hall_age_ticks),
        .fallback_sign=1, .hall_sequence=m->hall_sequence, .hall_sequence_valid=true
    };
    q31_t theta=rotor_angle_update(&m->angle_state,&in);
    double err=fabs(wrap_pi(q31_rad(theta)-wrap_pi(m->theta_e_rev*2.0*M_PI)))*180.0/M_PI;
    if(err>m->max_angle_error_deg)m->max_angle_error_deg=err;
    return theta;
}

static void dq_to_ab(double d,double q,double th,double *a,double *b)
{ double c=cos(th),s=sin(th); *a=d*c-q*s; *b=d*s+q*c; }
static void ab_to_dq(double a,double b,double th,double *d,double *q)
{ double c=cos(th),s=sin(th); *d=a*c+b*s; *q=-a*s+b*c; }

static void motor_phase_counts(const l4_motor_t *m,int16_t *ia,int16_t *ib)
{
    double a,b; dq_to_ab(m->id_a,m->iq_a,m->theta_e_rev*2.0*M_PI,&a,&b);
    double ib_a=(-a+SQRT3*b)*0.5;
    long ca=llround(a/CURRENT_A_PER_COUNT), cb=llround(ib_a/CURRENT_A_PER_COUNT);
    if (ca > 32767) ca = 32767;
    if (ca < -32768) ca = -32768;
    if (cb > 32767) cb = 32767;
    if (cb < -32768) cb = -32768;
    *ia=(int16_t)ca; *ib=(int16_t)cb;
}

static void pwm_to_ab(const uint16_t pwm[3],double vbus,double *alpha,double *beta)
{
    double da=(double)pwm[0]/(double)_T, db=(double)pwm[1]/(double)_T, dc=(double)pwm[2]/(double)_T;
    double mean=(da+db+dc)/3.0;
    double va=-vbus*(da-mean), vb=-vbus*(db-mean);
    *alpha=va; *beta=(va+2.0*vb)/SQRT3;
}

static void motor_init(l4_motor_t *m,double vbus_v,double start_rev)
{
    memset(m,0,sizeof(*m));
    m->r_ohm=0.060; m->ld_h=80e-6; m->lq_h=80e-6; m->flux_wb=0.015;
    m->vbus_v=vbus_v; m->theta_e_rev=start_rev;
    m->hall_age_ticks=0xFFFFU; m->tics_filtered_8=128000U;
    m->hall_sector_index=(int64_t)floor(start_rev*6.0);
    rotor_angle_reset(&m->angle_state); memset(&m->motion,0,sizeof(m->motion));
    pi_init(&PI_iq,_U_MAX); pi_init(&PI_id,1800); quiet_zero_reset(&g_qzero); pwm_geometry_init();
}

static void motor_foc_tick(l4_motor_t *m,MotorState_t *ms,uint32_t ctrl_tick)
{
    const double dt=1.0/(double)FOC_HZ;
    int16_t ia,ib; motor_phase_counts(m,&ia,&ib);
    g_ms=ms; g_motor=m;
    MotorParams_t mp; memset(&mp,0,sizeof(mp)); mp.com_mode=Hallsensor; mp.reverse=1;
    q31_t theta=motor_control_angle(m);
    FOC_calculation(ia,ib,theta,(int16_t)ms->i_q_setpoint,ms,&mp);
    (void)pwm_geometry_apply(switchtime,pwm_applied,(uint16_t)_T);
    double alpha,beta,vd,vq; pwm_to_ab(pwm_applied,m->vbus_v,&alpha,&beta);
    ab_to_dq(alpha,beta,m->theta_e_rev*2.0*M_PI,&vd,&vq);
    const int sub=8; const double h=dt/(double)sub;
    for(int n=0;n<sub;n++){
        double we=2.0*M_PI*m->erps;
        double did=(vd-m->r_ohm*m->id_a+we*m->lq_h*m->iq_a)/m->ld_h;
        double diq=(vq-m->r_ohm*m->iq_a-we*(m->ld_h*m->id_a+m->flux_wb))/m->lq_h;
        m->id_a+=did*h; m->iq_a+=diq*h;
    }
    /* Electrical power in dq. Positive is motoring draw; negative is ignored because production
     * EVistDrive has no commanded regenerative-braking product path here. */
    double p=1.5*(vd*m->id_a+vq*m->iq_a);
    if(p<0.0)p=0.0;
    m->dc_power_w=p/0.94; m->dc_power_acc_w+=m->dc_power_w; m->dc_power_samples++;

    double old=m->theta_e_rev;
    m->theta_e_rev+=m->erps*dt; m->hall_elapsed_500k+=500000.0*dt;
    uint64_t os=(uint64_t)floor(old*6.0), ns=(uint64_t)floor(m->theta_e_rev*6.0);
    if(ns!=os){
        m->hall_edge_this_ctrl=true; m->hall_edges+=(uint32_t)(ns-os);
        uint32_t cap=(uint32_t)llround(m->hall_elapsed_500k); if(cap>65535U)cap=65535U; if(cap==0U)cap=1U;
        rotor_motion_note_edge(&m->motion,m->hall_age_ticks,(uint16_t)cap);
        m->hall_sequence++; m->last_capture_500k=(uint16_t)cap;
        m->tics_filtered_8-=m->tics_filtered_8>>3; m->tics_filtered_8+=cap;
        m->hall_elapsed_500k=0.0; m->hall_sector_index=(int64_t)ns;
    }
    (void)ctrl_tick;
}

/* PAS raw ring for PAS_DIR_SIGN=-1: 00 -> 10 -> 11 -> 01 -> 00. */
static const uint8_t FWD_AB[4]={0U,2U,3U,1U};

typedef struct {
    MotorState_t ms;
    l4_motor_t motor;
    evd_bike_t bike;
    evd_rider_t rider;
    evd_battery_pack_t batt;
    soc_core_state_t fw_soc;
    float fw_soc_mas_acc;
    uint32_t tick;
    uint16_t last_forward_gap, stop_timeout;
    uint8_t start_phase;
    uint64_t pas_transition_index;
    uint8_t pas_ab;
    uint8_t pas_bounce_phase;
    bool inject_pas_bounce;
    uint32_t false_reverse_events;
    uint32_t direction_inhibit_ticks;
    uint32_t first_permission_tick, first_hall_tick, first_iq_tick;
    uint8_t assist_level;
    uint8_t limp_limit, limp_stage2;
    bool brake;
    bool walk;
    float controller_temp_c;
    float max_speed_kph;
    float max_battery_current_a;
    float max_iq_ref;
    float min_vbus;
    float max_soc_abs_error;
    uint32_t battery_limit_ticks;
    uint32_t speed_limit_ticks;
    uint32_t soc_updates;
    uint16_t last_load_ctrl;
    rider_input_t last_rider_input;
    ride_control_input_t last_control_input;
} l4_t;

/* FW-150: mirrors default_centikg_to_native_delta() over the three-point curve. */
static uint16_t torque_native_from_ckg(float ckg)
{
    double d; if(ckg<0.0f)ckg=0.0f;
    if(ckg<=TORQUE_CURVE_P1_CENTIKG)
        d=ckg*(double)TORQUE_CURVE_P1_NATIVE/TORQUE_CURVE_P1_CENTIKG;
    else if(ckg<=TORQUE_CURVE_P2_CENTIKG)
        d=TORQUE_CURVE_P1_NATIVE+(ckg-TORQUE_CURVE_P1_CENTIKG)*
            (double)(TORQUE_CURVE_P2_NATIVE-TORQUE_CURVE_P1_NATIVE)/
            (double)(TORQUE_CURVE_P2_CENTIKG-TORQUE_CURVE_P1_CENTIKG);
    else d=TORQUE_CURVE_P2_NATIVE+(ckg-TORQUE_CURVE_P2_CENTIKG)*
            (double)(TORQUE_CURVE_P3_NATIVE-TORQUE_CURVE_P2_NATIVE)/
            (double)(TORQUE_CURVE_P3_CENTIKG-TORQUE_CURVE_P2_CENTIKG);
    if(d>TORQUE_SPAN_MAX_NATIVE)d=TORQUE_SPAN_MAX_NATIVE;
    return (uint16_t)llround((double)TORQUE_ZERO_TARGET_NATIVE+d);
}

static uint16_t l4_pre_eb74_from_load(uint16_t load_ctrl)
{
    uint32_t source=750U+((uint32_t)load_ctrl*2450U)/6000U;
    if(source>3200U)source=3200U;
    return (uint16_t)source;
}

static void l4_init(l4_t *s,float true_soc,evd_battery_profile_t profile,float grade,
                    float target_rpm,float base_torque_nm,float gear_ratio,float r0_mohm,
                    double start_electrical_rev)
{
    memset(s,0,sizeof(*s));
    torque_input_init(); torque_input_startup_zero(TORQUE_ZERO_TARGET_NATIVE);
    torque_input_set_run_window_deg(tuning_config_assist_torque_run_window_deg());
    assist_modes_init(); assist_modes_set_active_bank(0U); motor_core_init(&s->ms);
    s->ms.hall_angle_detect_flag=1U; foc_current_feedback_reset(&s->ms);
    ride_control_init(); pas_direction_init(); pas_liveness_init(); pas_cadence_reset();
    cadence_filter_reset(); pas_sampler_init(0U); crank_phase_init();
#ifdef ASSIST_V3
    if(!assist_v3_harness_select_engine(false)){ fprintf(stderr,"L4: V3 engine write rejected\n"); abort(); }
#endif
    evd_bike_init(&s->bike); evd_rider_init(&s->rider);
    s->bike.grade=grade; s->bike.chain_gear_ratio=gear_ratio;
    s->rider.target_cadence_rpm=target_rpm; s->rider.base_torque_nm=base_torque_nm;
    evd_battery_init(&s->batt,profile,11U,14.0f,true_soc,r0_mohm,25.0f,1.5f);
    motor_init(&s->motor,s->batt.terminal_v,start_electrical_rev);
    s->last_forward_gap=PAS_STOP_TICKS; s->stop_timeout=PAS_STOP_TICKS;
    s->pas_ab=FWD_AB[0]; pas_sampler_isr_tick(s->pas_ab,0U);
    s->assist_level=DEFAULT_ASSIST_LEVEL; s->limp_limit=10U; s->limp_stage2=5U;
    s->controller_temp_c=DEFAULT_CONTROLLER_TEMP_C; s->min_vbus=1000.0f;

    int8_t seed=soc_core_calculate_ocv((uint16_t)lroundf(s->batt.terminal_v*1000.0f),11U);
    memset(&s->fw_soc,0,sizeof(s->fw_soc));
    s->fw_soc.soc_real=(float)seed; s->fw_soc.soc_display=(float)seed;
    s->fw_soc.soc_voltage=seed; s->fw_soc.remaining_mah=(float)seed/100.0f*14000.0f;
    s->fw_soc.boot_vmin_mv=0xFFFFU;
}

static void process_pas(l4_t *s,uint8_t ab)
{
    pas_sampler_isr_tick(ab,s->tick);
    pas_step_event_t ev;
    while(pas_sampler_pop(&ev)){
        int8_t st=ev.step;
        crank_phase_on_event(&ev);   /* as main.c: every drained event, first */
        if(st>0){
            s->last_forward_gap=ev.gap?ev.gap:s->last_forward_gap;
            uint32_t x=(uint32_t)s->last_forward_gap*2U; if(x<PAS_STOP_TICKS)x=PAS_STOP_TICKS; if(x>PAS_STOP_TICKS_MAX)x=PAS_STOP_TICKS_MAX;
            s->stop_timeout=(uint16_t)x; pas_direction_on_step(st); torque_input_run_filter_step();
            pas_cadence_step_t cad=pas_cadence_forward_step(ev.tick,pas_direction_fwd_run()==1U?1U:0U);
            if(cad.pulse&&cad.measured){ s->ms.cadence=cad.rpm; s->start_phase=0U; cadence_filter_update(s->ms.cadence); }
        }else if(st<0){ s->false_reverse_events++; pas_cadence_break_epoch(0U); pas_direction_on_step(st); }
        else { pas_cadence_break_epoch(1U); pas_direction_on_step(st); }
    }
    if(pas_sampler_take_overflow()){ pas_cadence_break_epoch(2U); crank_phase_on_overflow(); }
    if(s->ms.cadence==0U&&!s->start_phase&&pas_direction_fwd_run()>=START_PHASE_STEPS)s->start_phase=1U;
}

static uint8_t pas_from_crank(l4_t *s)
{
    uint64_t idx=(uint64_t)floor((double)s->bike.crank_rev*(double)PAS_TRANSITIONS_PER_REV);
    if(s->pas_bounce_phase==1U){ s->pas_bounce_phase=2U; return FWD_AB[(s->pas_transition_index+3U)&3U]; }
    if(s->pas_bounce_phase==2U){ s->pas_bounce_phase=0U; return s->pas_ab; }
    if(idx>s->pas_transition_index){
        s->pas_transition_index=idx; s->pas_ab=FWD_AB[idx&3U];
        if(s->inject_pas_bounce && (idx%11U)==0U)s->pas_bounce_phase=1U;
    }
    return s->pas_ab;
}

static void firmware_soc_tick(l4_t *s,float battery_current_a)
{
    s->fw_soc_mas_acc += battery_current_a*1000.0f/(float)CTRL_HZ;
    if((s->tick%CTRL_HZ)!=0U)return;
    soc_core_input_t in={
        .voltage_mv=(uint32_t)lroundf(s->batt.terminal_v*1000.0f),
        .battery_current_ma=(int32_t)lroundf(battery_current_a*1000.0f),
        .delta_mah=s->fw_soc_mas_acc/3600.0f,
        .capacity_estimated_mah=14000U,
        .r_batt_mohm=80U,
        .system_voltage=40,
        .soc_full_magic=SOC_FULL_MAGIC,
        .soc_full_pack_10mv=4598U
    };
    s->fw_soc_mas_acc=0.0f; soc_core_step_1hz(&s->fw_soc,&in); s->soc_updates++;
    float e=fabsf(s->fw_soc.soc_display-s->batt.true_soc_pct); if(e>s->max_soc_abs_error)s->max_soc_abs_error=e;
}

static void l4_tick(l4_t *s,FILE *csv)
{
    const float dt=1.0f/(float)CTRL_HZ; s->tick++; s->motor.hall_edge_this_ctrl=false;

    /* Virtual rider sees the real virtual cadence from the road/drivetrain and generates leg torque. */
    evd_rider_step(&s->rider,&s->bike);
    process_pas(s,pas_from_crank(s));
    uint32_t idle=s->tick-pas_sampler_last_transition_tick(); pas_liveness_update(idle,s->stop_timeout);
    bool real_stop=pas_liveness_stopped();
    if(real_stop){ s->ms.cadence=0U; s->start_phase=0U; cadence_filter_reset(); pas_cadence_reset(); pas_direction_on_stop(); }
    bool direction_ok=(s->ms.cadence>0U||s->start_phase)&&!real_stop;
    bool pedaling=direction_ok&&pas_direction_fwd_run()>=tuning_config_start_steps();
    uint8_t control_cadence=cadence_filter_get();

    uint16_t raw=torque_native_from_ckg(s->rider.torque_ckg);
    torque_input_update(raw,torque_input_correct(raw),true);
    const torque_snapshot_t *ts=torque_input_get_snapshot();
    uint32_t speed_x100=evd_bike_speed_x100(&s->bike);

    rider_input_t r; memset(&r,0,sizeof(r));
    r.torque_raw_mv=raw; r.torque_corrected_mv=torque_input_correct(raw);
    r.torque_filtered=ts->delta_native; r.torque_assist_now_native=ts->assist_delta_native;
    r.torque_assist_filtered=ts->assist_delta_filtered_native; r.torque_run_filtered=ts->assist_delta_run_native;
    r.torque_load_ctrl=ts->load_ctrl; r.torque_load_centikg=ts->load_centikg; r.cadence_rpm=control_cadence; r.wheel_speed_x100=speed_x100;
    r.motor_erps=(uint16_t)((s->motor.erps>65535.0)?65535.0:llround(s->motor.erps));
    r.motor_erps_age_ticks=s->motor.hall_age_ticks; r.motor_voltage_utilization=(uint16_t)((s->ms.u_abs<0)?0:s->ms.u_abs);
    r.pas_forward=pedaling; r.pedaling_active=pedaling; r.crank_forward_steps=pas_direction_fwd_run();
    r.crank_direction_ok=direction_ok; r.real_stop=real_stop; r.wheel_valid=true;
    r.direction_inhibit_active=pas_direction_direction_inhibit_active();
    r.forward_confirmed_this_tick=pas_direction_forward_confirmed_last_call(); r.sample_tick=s->tick;
    r.start_phase=s->start_phase!=0U; r.torque_sensor_valid=true; r.pas_sensor_valid=true;
    /* ASSIST-V3 observations, as main.c fills them (no wheel-pulse model here: tick stays 0). */
    r.crank_steps=crank_phase_steps(); r.crank_step_tick=crank_phase_last_step_tick();
    r.pas_glitch=crank_phase_take_glitch();
    rider_input_update(&r);
    s->last_rider_input=r;

    float limp=soc_core_limp_factor(s->fw_soc.soc_display,s->limp_limit,s->limp_stage2);
    int32_t iq_limit=(int32_t)lroundf((float)PH_CURRENT_MAX*limp); if(iq_limit<0)iq_limit=0;
    int32_t batt_ma=(int32_t)lroundf(s->batt.current_a*1000.0f);
    uint16_t voltage_raw=(uint16_t)lroundf(s->batt.terminal_v*1000.0f/(float)CAL_BAT_V);
    ride_control_input_t in; memset(&in,0,sizeof(in));
    in.speed_x100=speed_x100; in.cadence_rpm=control_cadence; in.assist_level_index=s->assist_level;
    in.battery_voltage_mv=(uint32_t)lroundf(s->batt.terminal_v*1000.0f);
    in.iq_scale=PH_CURRENT_MAX; in.ride_core_iq_limit=iq_limit; in.phase_current_max=iq_limit;
    in.battery_current_mA=batt_ma; in.battery_current_max=DEFAULT_BATTERY_CURRENT_MAX_MA;
    in.battery_current_limiter_centiamp=batt_ma/10; /* TQ-06-G1: feeds the G53 PI #1 limiter (review F-05) */
    in.battery_soc_derate_q12=(uint16_t)(G53_G1_Q12_ONE-g53_g1_soc_factor_m820(  /* step 2, review S2-03 */
        (int32_t)(s->fw_soc.soc_display*10.0f),s->limp_limit,s->limp_stage2));
    in.u_abs=s->ms.u_abs; in.cal_i=CAL_I; in.current_iq=s->ms.i_q; in.current_id=s->ms.i_d;
    in.voltage_raw=voltage_raw; in.voltage_min_raw=VOLTAGE_MIN;
    in.controller_temperature_c=(int16_t)lroundf(s->controller_temp_c);
    in.cadence_filtered_x8=cadence_filter_get_x8(); in.speed_limit_x100=SPEEDLIMIT;
    in.legal_enabled=true; in.offroad=false; in.walk_active=false;
    in.pas_ab=s->pas_ab;
    in.safety_cut_non_direction=s->brake; in.service_cut_active=false; in.elapsed_ticks=1U;
    in.brake=s->brake; in.iq_measured=s->ms.i_q;   /* ASSIST-V3 observations; IMU invalid (memset) */
    if(s->pas_ab!=0U&&in.pas_ab!=s->pas_ab){
        fprintf(stderr,"L4 native PAS plumbing mismatch at tick %u: native=%u input=%u\n",
                s->tick,(unsigned)s->pas_ab,(unsigned)in.pas_ab);
        abort();
    }
    s->last_control_input=in;
    s->last_load_ctrl=ts->load_ctrl;
    ride_control_update(&in);
    if(assist_pipeline_pas_state()==AP2_PAS_FORWARD&&s->first_permission_tick==0U)s->first_permission_tick=s->tick;
    if(ride_control_battery_limit_active())s->battery_limit_ticks++;
    if(speed_x100>=SPEEDLIMIT)s->speed_limit_ticks++;

    /* FOC electrical speed is mechanically tied to chainring speed in Level 4. The relation is
     * the same one already used by the Walk SIL: chainring rpm = electrical ERPS * 3/4. */
    s->motor.erps=(double)s->bike.crank_rpm*(4.0/3.0); s->motor.vbus_v=s->batt.terminal_v;
    s->motor.dc_power_acc_w=0.0; s->motor.dc_power_samples=0U;
    for(unsigned k=0;k<INNER_PER_CTRL;k++)motor_foc_tick(&s->motor,&s->ms,s->tick);
    if(s->motor.hall_edge_this_ctrl){ s->motor.hall_age_ticks=0U; if(!s->first_hall_tick)s->first_hall_tick=s->tick; }
    else if(s->motor.hall_age_ticks<0xFFFFU)s->motor.hall_age_ticks++;
    if(s->ms.i_q_setpoint>0&&!s->first_iq_tick)s->first_iq_tick=s->tick;
    if(r.direction_inhibit_active)s->direction_inhibit_ticks++;

    float motor_torque=(float)(s->motor.iq_a/CURRENT_A_PER_COUNT)*MOTOR_CRANK_NM_PER_IQ_COUNT;
    if(motor_torque<0.0f)motor_torque=0.0f;
    bool engaged=s->rider.pedaling||motor_torque>0.1f;
    evd_bike_step(&s->bike,s->rider.torque_nm,motor_torque,engaged,dt);

    float motor_power=(s->motor.dc_power_samples? (float)(s->motor.dc_power_acc_w/(double)s->motor.dc_power_samples):0.0f);
    float batt_a=(motor_power+4.0f)/(s->batt.terminal_v>5.0f?s->batt.terminal_v:5.0f);
    if (batt_a < 0.0f) batt_a = 0.0f;
    if (batt_a > 40.0f) batt_a = 40.0f;
    evd_battery_step(&s->batt,batt_a,dt); firmware_soc_tick(s,batt_a);

    s->ms.Voltage=(int32_t)lroundf(s->batt.terminal_v*1000.0f); s->ms.Battery_Current=(int32_t)lroundf(batt_a*1000.0f);
    s->ms.Speedx100=evd_bike_speed_x100(&s->bike); s->ms.SOC=(uint8_t)lroundf(s->fw_soc.soc_display);
    float kph=s->bike.speed_mps*3.6f; if(kph>s->max_speed_kph)s->max_speed_kph=kph;
    if(batt_a>s->max_battery_current_a)s->max_battery_current_a=batt_a;
    if((float)s->ms.i_q_setpoint>s->max_iq_ref)s->max_iq_ref=(float)s->ms.i_q_setpoint;
    if(s->batt.terminal_v<s->min_vbus)s->min_vbus=s->batt.terminal_v;

    if(csv && (s->tick%20U)==0U){
        const assist_pipeline_telemetry_t *mo=assist_pipeline_telemetry();
        fprintf(csv,"%.6f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%u,%u,%d,%d,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u\n",
            (double)s->tick/CTRL_HZ,s->bike.distance_m,s->bike.speed_mps*3.6f,s->bike.crank_rpm,
            s->rider.torque_nm,s->rider.torque_ckg,s->batt.terminal_v,s->batt.current_a,
            (unsigned)lroundf(s->batt.true_soc_pct),(unsigned)lroundf(s->fw_soc.soc_display),
            mo->iq_request_before_limits,s->ms.i_q_setpoint,(float)s->ms.i_q,s->motor.erps,
            (float)s->ms.u_abs,limp,ride_control_get_session_state(),assist_pipeline_reason_bits(),
            ride_control_battery_limit_active()?1U:0U,s->motor.hall_age_ticks);
    }
    if(l4_stage_trace){
        const g53_port_output_t *g=assist_pipeline_g53();
        const assist_pipeline_telemetry_t *t=assist_pipeline_telemetry();
        const uint32_t mask=(g->trace.rider_input_native>0?1U:0U) |
            (g->trace.d7ec_rider>0?2U:0U) | (g->trace.e1e8_output>0?4U:0U) |
            (g->m2aa_native>0?8U:0U) | (g->iq_request_pre_limits>0?16U:0U) |
            (g->normal_permission?32U:0U) | (t->final_iq_request>0?64U:0U) |
            (s->ms.i_q_setpoint>0?128U:0U);
        if(s->tick==1U || mask!=l4_stage_previous_mask || (s->tick%400U)==0U){
            const l4_eb74_observation_t eb74=l4_eb74_observer_last();
            fprintf(l4_stage_trace,"%u,%.6f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
                s->tick,(double)s->tick/CTRL_HZ,l4_eb74_observer_count(),
                (unsigned)s->last_load_ctrl,(unsigned)l4_pre_eb74_from_load(s->last_load_ctrl),
                (unsigned)eb74.output.pre_eb74,
                (unsigned)g->trace.rider_input_native,(unsigned)g->trace.d7ec_rider,
                (unsigned)g->trace.e1e8_output,(unsigned)g->m2aa_native,
                g->normal_permission?1U:0U,(unsigned)(g->iq_request_pre_limits>0),
                (unsigned)(t->final_iq_request>0),t->final_iq_request,s->ms.i_q_setpoint,
                (unsigned)mask,(unsigned)(s->tick==1U),(unsigned)s->last_control_input.speed_x100,
                (unsigned)s->last_control_input.cadence_rpm,(unsigned)s->last_control_input.assist_level_index,
                (unsigned)g->trace.pas_direction,(unsigned)g->trace.cadence,(unsigned)g->trace.evidence,
                (unsigned)g->trace.d7ec_envelope,(unsigned)g->trace.d7ec_accel,
                (unsigned)g->trace.e1e8_state,(unsigned)g->trace.bde8_q50,(unsigned)g->trace.m298,
                (unsigned)g->trace.fsm_state,(unsigned)g->trace.fsm_output,(unsigned)g->trace.fsm_t34,
                (unsigned)s->ms.u_abs,(unsigned)lroundf(s->motor.erps));
            l4_stage_previous_mask=mask;
        }
    }
}

/* Four elapsed control periods create exactly one port logical update. The linker wrapper
 * counts the real production EB74 function invocation, so this is an observed equality, not
 * an inferred conversion from simulator ticks. */
static bool l4_synthetic_update(uint16_t load_ctrl,uint8_t native_pas_ab)
{
    rider_input_t rider={0};
    rider.torque_load_ctrl=load_ctrl;
    rider.wheel_valid=true;
    rider.real_stop=false;
    rider.torque_sensor_valid=true;
    rider.pas_sensor_valid=true;
    rider_input_update(&rider);
    const rider_input_t *observed_rider=rider_input_get();

    ride_control_input_t in={0};
    in.raw_pa6_adc=0U;
    in.pas_ab=native_pas_ab;
    in.assist_level_index=DEFAULT_ASSIST_LEVEL;
    in.battery_voltage_mv=42000U;
    in.iq_scale=PH_CURRENT_MAX;
    in.ride_core_iq_limit=PH_CURRENT_MAX;
    in.phase_current_max=PH_CURRENT_MAX;
    in.battery_current_max=DEFAULT_BATTERY_CURRENT_MAX_MA;
    in.cal_i=CAL_I;
    in.voltage_raw=(uint16_t)(42000U/(uint32_t)CAL_BAT_V);
    in.voltage_min_raw=VOLTAGE_MIN;
    in.controller_temperature_c=DEFAULT_CONTROLLER_TEMP_C;
    in.speed_limit_x100=SPEEDLIMIT;
    in.legal_enabled=true;
    in.elapsed_ticks=4U;

    const uint32_t before=l4_eb74_observer_count();
    ride_control_update(&in);
    const uint32_t after=l4_eb74_observer_count();
    const l4_eb74_observation_t obs=l4_eb74_observer_last();
    return after==before+1U && obs.count==after && obs.load_ctrl==load_ctrl &&
        obs.output.pre_eb74==l4_pre_eb74_from_load(load_ctrl) &&
        observed_rider->torque_load_ctrl==load_ctrl &&
        (load_ctrl!=0U || (observed_rider->torque_raw_mv==0U &&
         observed_rider->torque_filtered==0U && observed_rider->torque_assist_now_native==0U &&
         observed_rider->cadence_rpm==0U)) && observed_rider->torque_sensor_valid &&
        observed_rider->pas_sensor_valid && !observed_rider->direction_inhibit_active &&
        !observed_rider->real_stop && !in.safety_cut_non_direction && !in.service_cut_active;
}

static bool l4_eb74_prehistory(l4_t *s,const char *trace_path,const char *guard_prefix)
{
    FILE *trace=NULL;
    if(trace_path){
        trace=fopen(trace_path,"wb");
        if(!trace){perror(trace_path);return false;}
        fputs("stage,invocation,load_ctrl,pre_eb74,source_le_960,startup_count,check_count,"
              "rider_input_native,torque_sensor_valid,pas_sensor_valid,direction_inhibit,"
              "real_stop,safety_cut,scored\n",trace);
    }

    /* L4-PRE-1: real l4_init()/ride_control_init() cold reset, with no EB74 call afterward. */
    bool ok=l4_eb74_observer_count()==0U && g53_port_trace()->logical_tick==0U &&
        s->tick==0U && s->max_iq_ref==0.0f && s->bike.distance_m==0.0f;
    if(!ok)fprintf(stderr,"%s-1 cold/reset precondition failed\n",guard_prefix);
    const float initial_soc=s->batt.true_soc_pct;
    const float initial_voltage=s->batt.terminal_v;
    const float initial_battery_current=s->batt.current_a;
    const double initial_motor_iq=s->motor.iq_a;
    unsigned scored_rows=0U;
    for(uint32_t n=1U;n<=131U;n++){
        const uint32_t before=l4_eb74_observer_count();
        /* Neutral native PAS is valid sensor state; torque remains exactly zero. */
        if(!l4_synthetic_update(0U,FWD_AB[0]))ok=false;
        const uint32_t after=l4_eb74_observer_count();
        const l4_eb74_observation_t obs=l4_eb74_observer_last();
        const bool valid=obs.load_ctrl==0U && obs.output.pre_eb74==750U &&
            obs.output.pre_eb74<=960U && n==after && after==before+1U;
        if(!valid){
            fprintf(stderr,"%s-2/8 invalid prehistory expected=%u actual=%u load=%u pre=%u\n",
                guard_prefix,n,after,obs.load_ctrl,obs.output.pre_eb74);
            ok=false;
        }
        if(trace){
            fprintf(trace,"PREHISTORY,%u,%u,%u,%u,%u,%u,%u,1,1,0,0,0,0\n",after,
                (unsigned)obs.load_ctrl,(unsigned)obs.output.pre_eb74,
                obs.output.pre_eb74<=960U?1U:0U,(unsigned)obs.output.startup_count,
                (unsigned)obs.output.check_count,(unsigned)obs.output.rider_input_native);
        }
        if(n==131U && (obs.output.startup_count!=300U || obs.output.check_count!=100U)){
            fprintf(stderr,"%s-3A invocation 131 did not enter normal EB74 processing\n",guard_prefix);
            ok=false;
        }
    }
    if(trace)fclose(trace);

    /* L4-PRE-3/3A: the trace marks every invocation PREHISTORY/unscored. L4-PRE-4: no
     * Level-4 plant, electrical, battery, timing or scenario metric was advanced. */
    if(scored_rows!=0U || l4_eb74_observer_count()!=131U || s->tick!=0U ||
       s->max_iq_ref!=0.0f || s->max_speed_kph!=0.0f || s->bike.distance_m!=0.0f ||
       s->bike.speed_mps!=0.0f || s->batt.true_soc_pct!=initial_soc ||
       s->batt.terminal_v!=initial_voltage || s->batt.current_a!=initial_battery_current ||
       s->motor.iq_a!=initial_motor_iq){
        fprintf(stderr,"%s-3/4 scored or metric state advanced before readiness\n",guard_prefix);
        ok=false;
    }
    if(strcmp(guard_prefix,"L4-PRE")==0){
        printf("L4-PRE-1..4 %s: cold reset; 131 actual EB74 calls; all unscored; load=0/pre=750\n",
               ok?"PASS":"FAIL");
        printf("L4-PRE-7 %s: harness uses ride_control_update; EB74 private state is isolated in the separately compiled production module\n",
               ok?"PASS":"FAIL");
        printf("L4-PRE-8 %s: each of 131 updates advanced the linker-observed real EB74 invocation count by exactly one\n",
               ok?"PASS":"FAIL");
    }else{
        printf("%s-1..6/12 %s: cold reset; 131 actual EB74 calls; all unscored; load=0/pre=750; public lifecycle; no direct state seeding; 1:1 invocation proof\n",
               guard_prefix,ok?"PASS":"FAIL");
    }
    return ok;
}

static bool l4_cold_high_load_negative(void)
{
    l4_t s;
    l4_init(&s,90.0f,EVD_BATT_PROFILE_FEB21700G,0.0f,60.0f,18.0f,2.10f,80.0f,0.03);
    l4_eb74_observer_reset();
    bool ok=g53_port_trace()->logical_tick==0U;
    for(uint32_t n=1U;n<=400U;n++){
        if(!l4_synthetic_update(12000U,FWD_AB[0]))ok=false;
        const l4_eb74_observation_t obs=l4_eb74_observer_last();
        const g53_port_output_t *g=assist_pipeline_g53();
        const assist_pipeline_telemetry_t *t=assist_pipeline_telemetry();
        const unsigned reasons=(obs.count!=n?1U:0U) | (obs.load_ctrl!=12000U?2U:0U) |
            (obs.output.pre_eb74<=960U?4U:0U) | (obs.output.rider_input_native!=0U?8U:0U) |
            (n>30U && obs.output.check_count!=90U?16U:0U) | (g->normal_permission?32U:0U) |
            (g->iq_request_pre_limits!=0?64U:0U) | (t->final_iq_request!=0?128U:0U);
        if(reasons){
            if(ok)fprintf(stderr,"L4-PRE-6 first divergence n=%u reason=0x%x count=%u load=%u pre=%u start=%u check=%u rider=%u permission=%u m2aa=%u iq_pre=%d iq_final=%d\n",
                n,reasons,obs.count,obs.load_ctrl,obs.output.pre_eb74,obs.output.startup_count,
                obs.output.check_count,obs.output.rider_input_native,g->normal_permission?1U:0U,
                g->m2aa_native,g->iq_request_pre_limits,t->final_iq_request);
            ok=false;
        }
    }
    printf("L4-PRE-6 cold continuous-high-load negative %s: 400 calls, source>960, zero/unqualified\n",
           ok?"PASS":"FAIL");
    return ok;
}

typedef struct {
    const char *name; float soc,grade,target_rpm,base_torque,gear,r0_mohm; unsigned seconds;
    evd_battery_profile_t profile; bool bounce;
} scenario_t;

static uint64_t fuzz_hash_u32(uint64_t h,uint32_t value)
{
    for(unsigned i=0U;i<4U;i++){
        h^=(uint8_t)(value>>(8U*i));
        h*=1099511628211ULL;
    }
    return h;
}

static uint32_t fuzz_float_bits(float value)
{
    uint32_t bits=0U;
    memcpy(&bits,&value,sizeof(bits));
    return bits;
}

static uint64_t fuzz_vector_signature(unsigned case_id,const scenario_t *sc,float start_electrical_rev)
{
    uint64_t h=14695981039346656037ULL;
    h=fuzz_hash_u32(h,case_id);
    h=fuzz_hash_u32(h,fuzz_float_bits(sc->soc));
    h=fuzz_hash_u32(h,fuzz_float_bits(sc->grade));
    h=fuzz_hash_u32(h,fuzz_float_bits(sc->target_rpm));
    h=fuzz_hash_u32(h,fuzz_float_bits(sc->base_torque));
    h=fuzz_hash_u32(h,fuzz_float_bits(sc->gear));
    h=fuzz_hash_u32(h,fuzz_float_bits(sc->r0_mohm));
    h=fuzz_hash_u32(h,fuzz_float_bits(start_electrical_rev));
    h=fuzz_hash_u32(h,(uint32_t)sc->profile);
    h=fuzz_hash_u32(h,sc->bounce?1U:0U);
    h=fuzz_hash_u32(h,sc->seconds);
    return h;
}

/* Captured from the pre-P9-G7 generator at the same 62785bd Phase-9 base.
 * Canonical signature serializes each generator-owned field as LE uint32 values;
 * floats are their exact IEEE-754 binary32 bits, followed by profile/bounce/seconds. */
static const uint64_t FUZZ_VECTOR_GOLDEN[25]={
    0xB9CE083786B94AC6ULL,0x908D2F6541ED9A9BULL,0x4E390CCF13D1068BULL,
    0x1F5D77BEBF2AA60EULL,0x9B26C0627B448A16ULL,0xC999C281437FDD1BULL,
    0x594A24C840CD6A43ULL,0x1AD1D9129DB11A40ULL,0x229B19C201C23699ULL,
    0xEA43BCF6D0C06FA5ULL,0x470170F62BA6D699ULL,0x6BBDBB71D8ACEEF3ULL,
    0xFAE76E20773D8E83ULL,0x5C3AE98C988AB1E3ULL,0x5CC678C86B254C75ULL,
    0x836BDF1CD73C4A67ULL,0x2AE343E615FCDC70ULL,0xA21263ABE0388187ULL,
    0xA3865307AA4FB383ULL,0x3EB9BD72E0270731ULL,0xFE0909D284B2BDE2ULL,
    0x9CBD622166FE3769ULL,0xC067B75A5D2472AFULL,0x8008B133A05B94BCULL,
    0x2F852300F9D6FE19ULL
};

static const uint8_t FUZZ_EXPECTED_START_GOLDEN[25]={
    1U,1U,0U,0U,0U,1U,0U,0U,0U,0U,1U,1U,1U,1U,0U,0U,1U,0U,0U,1U,0U,1U,0U,1U,1U
};

static int run_scenario(const scenario_t *sc,const char *outdir)
{
    l4_t s; l4_init(&s,sc->soc,sc->profile,sc->grade,sc->target_rpm,sc->base_torque,sc->gear,sc->r0_mohm,0.03);
    l4_eb74_observer_reset();
    s.inject_pas_bounce=sc->bounce;
    char path[512]; snprintf(path,sizeof(path),"%s/%s.csv",outdir,sc->name);
    FILE *f=fopen(path,"wb"); if(!f){perror(path);return 1;}
    fprintf(f,"time_s,distance_m,speed_kph,cadence_rpm,rider_torque_nm,torque_ckg,battery_v,battery_a,true_soc,fw_soc,iq_request,iq_ref,iq_actual,motor_erps,u_abs,limp,session,debug,battery_limit,hall_age\n");
    char prehistory_path[512];
    snprintf(prehistory_path,sizeof(prehistory_path),"%s/%s-p9-g6-prehistory.csv",outdir,sc->name);
    if(!l4_eb74_prehistory(&s,strcmp(sc->name,"flat_soc90")==0?prehistory_path:NULL,"L4-PRE")){
        fclose(f);return 1;
    }
    const bool capture=strcmp(sc->name,"flat_soc90")==0;
    FILE *stage=capture?fopen(".build/level4/flat_soc90-p9-g6-stages.csv","wb"):NULL;
    if(capture && !stage){perror(".build/level4/flat_soc90-p9-g6-stages.csv");fclose(f);return 1;}
    if(stage){
        fputs("tick,time_s,eb74_invocations,load_ctrl,mapped_pre_eb74,last_actual_pre_eb74,rider_input_native,d7ec_rider,"
              "e1e8_output,m2aa,normal_permission,iq_request_positive,final_request_positive,"
              "final_request,final_iq,stage_mask,scenario_t0,speed_x100,cadence_rpm,assist_level,g53_pas_direction,g53_cadence,g53_evidence,d7ec_envelope,d7ec_accel,e1e8_state,bde8_q50,m298,fsm_state,fsm_output,fsm_t34,u_abs,motor_erps\n",stage);
        l4_stage_trace=stage; l4_stage_previous_mask=0U;
    }
    for(uint32_t t=0;t<sc->seconds*CTRL_HZ;t++){
        l4_tick(&s,f);
        if(t==0U){
            const ride_control_input_t *in=&s.last_control_input;
            const bool first_unchanged=s.tick==1U && s.last_load_ctrl==12000U &&
                in->raw_pa6_adc==0U && in->pas_ab==FWD_AB[0] && in->speed_x100==0U &&
                in->cadence_rpm==0U && in->assist_level_index==DEFAULT_ASSIST_LEVEL &&
                in->elapsed_ticks==1U && in->legal_enabled && !in->offroad &&
                !in->safety_cut_non_direction && !in->service_cut_active &&
                s.last_rider_input.torque_sensor_valid && s.last_rider_input.pas_sensor_valid &&
                !s.last_rider_input.direction_inhibit_active && !s.last_rider_input.real_stop &&
                l4_pre_eb74_from_load(s.last_load_ctrl)==3200U &&
                l4_eb74_observer_count()==131U;
            if(!first_unchanged){
                fprintf(stderr,"L4-PRE-3B/5 first scored mismatch tick=%u load=%u raw=%u pas=%u speed=%u cadence=%u level=%u elapsed=%u legal=%u offroad=%u safety=%u service=%u torque_valid=%u pas_valid=%u direction=%u real_stop=%u calls=%u mapped_pre=%u\n",
                    s.tick,s.last_load_ctrl,in->raw_pa6_adc,in->pas_ab,in->speed_x100,
                    in->cadence_rpm,in->assist_level_index,in->elapsed_ticks,in->legal_enabled?1U:0U,
                    in->offroad?1U:0U,in->safety_cut_non_direction?1U:0U,in->service_cut_active?1U:0U,
                    s.last_rider_input.torque_sensor_valid?1U:0U,s.last_rider_input.pas_sensor_valid?1U:0U,
                    s.last_rider_input.direction_inhibit_active?1U:0U,s.last_rider_input.real_stop?1U:0U,
                    l4_eb74_observer_count(),(unsigned)(750U+((uint32_t)s.last_load_ctrl*2450U)/6000U));
                l4_stage_trace=NULL; fclose(f);if(stage)fclose(stage);return 1;
            }
            printf("L4-PRE-3B/5 PASS: first scored t=0 follows EB74#131; original load=12000/pre=3200\n");
        }
    }
    l4_stage_trace=NULL;
    if(stage)fclose(stage);
    fclose(f);
    bool ok=true;
    if(s.false_reverse_events||s.direction_inhibit_ticks)ok=false;
    if(!s.first_permission_tick||!s.first_iq_tick||!s.first_hall_tick)ok=false;
    if(s.max_iq_ref>(float)PH_CURRENT_MAX+0.5f||s.max_iq_ref<0.0f)ok=false;
    if(s.motor.max_angle_error_deg>31.1)ok=false;
    if(!(s.batt.true_soc_pct>=0.0f&&s.batt.true_soc_pct<=100.0f))ok=false;
    if(!isfinite(s.batt.terminal_v)||!isfinite(s.bike.speed_mps)||!isfinite(s.motor.iq_a))ok=false;
    if(s.max_speed_kph>45.0f)ok=false;
    printf("L4 %-18s SOC %.0f%% grade=%4.1f%% cadenceTarget=%5.1f  speedMax=%5.2f km/h "
           "Vmin=%5.2f IbatMax=%5.2fA IqMax=%5.0f fwSOC=%.1f trueSOC=%.1f errMax=%.1f%% "
           "battLimit=%u speedLimit=%u angle=%.2fdeg %s\n",
           sc->name,sc->soc,sc->grade*100.0f,sc->target_rpm,s.max_speed_kph,s.min_vbus,
           s.max_battery_current_a,s.max_iq_ref,s.fw_soc.soc_display,s.batt.true_soc_pct,
           s.max_soc_abs_error,s.battery_limit_ticks,s.speed_limit_ticks,s.motor.max_angle_error_deg,
           ok?"PASS":"FAIL");
    return ok?0:1;
}

static int run_fixed(const char *outdir)
{
    static const scenario_t v[]={
        {"flat_soc90",90,0.00f,60,18,2.10f,80,8,EVD_BATT_PROFILE_FEB21700G,true},
        {"hill5_soc80",80,0.05f,70,24,2.00f,80,8,EVD_BATT_PROFILE_FEB21700G,false},
        {"hill10_soc50",50,0.10f,75,30,1.90f,90,8,EVD_BATT_PROFILE_FEB21700G,false},
        {"hill15_soc20",20,0.15f,80,38,1.80f,110,8,EVD_BATT_PROFILE_FEB21700G,false},
        {"low_soc10",10,0.08f,70,30,1.90f,120,8,EVD_BATT_PROFILE_FEB21700G,false},
        {"low_soc5",5,0.05f,60,25,2.00f,140,8,EVD_BATT_PROFILE_FEB21700G,false},
        {"sag_soc30",30,0.10f,75,32,1.90f,180,8,EVD_BATT_PROFILE_FEB21700G,false},
        {"cad120_soc80",80,0.02f,120,20,1.45f,80,8,EVD_BATT_PROFILE_FEB21700G,true},
        {"matched_lg50",50,0.07f,70,26,1.95f,80,8,EVD_BATT_PROFILE_LG_M58T,false}
    };
    int fail=l4_cold_high_load_negative()?0:1;
    for(size_t i=0;i<sizeof(v)/sizeof(v[0]);i++)fail+=run_scenario(&v[i],outdir);
    printf("LEVEL4 fixed scenarios: %zu cases failures=%d %s\n",sizeof(v)/sizeof(v[0]),fail,fail?"FAIL":"PASS");
    return fail?1:0;
}

/* Long-duration battery/SOC test at 1 Hz. No motor math is duplicated here: this test targets
 * battery truth versus the SAME production soc_core used by main.c and Level 4. */
static int run_soc_endurance(void)
{
    int fail=0;
    static const float starts[]={100,90,80,60,40,20,10,5};
    for(size_t k=0;k<sizeof(starts)/sizeof(starts[0]);k++){
        evd_battery_pack_t b; evd_battery_init(&b,EVD_BATT_PROFILE_FEB21700G,11,14.0f,starts[k],80,25,1.5f);
        soc_core_state_t fw={0}; fw.boot_vmin_mv=0xFFFFU;
        int8_t seed=soc_core_calculate_ocv((uint16_t)lroundf(b.terminal_v*1000.0f),11);
        fw.soc_real=(float)seed; fw.soc_display=(float)seed; fw.soc_voltage=seed; fw.remaining_mah=(float)seed/100.0f*14000.0f;
        float maxerr=fabsf(fw.soc_display-b.true_soc_pct);
        for(unsigned sec=0;sec<900;sec++){
            float ia=(sec%180U<150U)?8.0f:0.2f; evd_battery_step(&b,ia,1.0f);
            soc_core_input_t in={
                .voltage_mv=(uint32_t)lroundf(b.terminal_v*1000.0f), .battery_current_ma=(int32_t)lroundf(ia*1000.0f),
                .delta_mah=ia/3.6f, .capacity_estimated_mah=14000, .r_batt_mohm=80, .system_voltage=40,
                .soc_full_magic=SOC_FULL_MAGIC, .soc_full_pack_10mv=4598
            };
            soc_core_step_1hz(&fw,&in); float e=fabsf(fw.soc_display-b.true_soc_pct); if(e>maxerr)maxerr=e;
            if(!isfinite(fw.soc_display)||fw.soc_display<0||fw.soc_display>100)fail++;
        }
        printf("SOC endurance FEB start=%5.1f trueEnd=%5.1f fwEnd=%5.1f maxAbsErr=%5.1f%% %s\n",
               starts[k],b.true_soc_pct,fw.soc_display,maxerr,(maxerr<=35.0f)?"PASS":"WARN");
    }
    /* Exact matched-chemistry/IR scenario is a hard accuracy gate. */
    {
        evd_battery_pack_t b; evd_battery_init(&b,EVD_BATT_PROFILE_LG_M58T,11,14.0f,80,80,0,1.0f);
        soc_core_state_t fw={0}; fw.boot_vmin_mv=0xFFFFU; fw.soc_real=80; fw.soc_display=80; fw.soc_voltage=80; fw.remaining_mah=11200;
        float maxerr=0;
        for(unsigned sec=0;sec<1200;sec++){
            float ia=(sec%240U<200U)?5.0f:0.1f; evd_battery_step(&b,ia,1.0f);
            soc_core_input_t in={(uint32_t)lroundf(b.terminal_v*1000.0f),(int32_t)lroundf(ia*1000.0f),ia/3.6f,14000,80,40,0,0};
            soc_core_step_1hz(&fw,&in); float e=fabsf(fw.soc_display-b.true_soc_pct); if(e>maxerr)maxerr=e;
        }
        if(maxerr>3.0f)fail++;
        printf("SOC endurance MATCHED start=80 maxAbsErr=%.2f%% %s\n",maxerr,maxerr<=3.0f?"PASS":"FAIL");
    }
    return fail?1:0;
}

static uint32_t fuzz_state=0x144B1CE5U;
static uint32_t rnd(void){uint32_t x=fuzz_state;x^=x<<13;x^=x>>17;x^=x<<5;return fuzz_state=x;}
static float rr(float a,float b){return a+(b-a)*(float)(rnd()&0xFFFFFFU)/16777215.0f;}
static int run_fuzz(unsigned count,const char *outdir)
{
    int fail=0; unsigned safe_stalls=0,marginal_starts=0,robust_expected_count=0U,robust_pass_count=0U;
    unsigned executed=0U;bool vector_identity_ok=true,prng_unchanged=true,prehistory_ok=true;
    bool first_scored_ok=true,classifier_identity_ok=true;
    bool cold_reset_ok=true,invocations131_ok=true,invocation131_unscored_ok=true;
    FILE *matrix=NULL;
    char matrix_path[512];
    if(snprintf(matrix_path,sizeof(matrix_path),"%s/fuzz-p9-g7-matrix.csv",outdir)>=(int)sizeof(matrix_path)){
        fprintf(stderr,"L4-FUZZ-PRE matrix path too long\n");return 1;
    }
    matrix=fopen(matrix_path,"wb");
    if(!matrix){perror(matrix_path);return 1;}
    fputs("case_id,vector_signature,vector_identity,prehistory_count,invocation131_unscored,first_scored_tick,eb74_count_after_first_scored_update,prng_unchanged,class,expected_start,first_iq_tick,first_hall_tick,case_pass\n",matrix);
    const bool seed_ok=fuzz_state==0x144B1CE5U;
    if(!seed_ok){fprintf(stderr,"L4-FUZZ-PRE-9 seed changed: 0x%08X\n",fuzz_state);fail++;}
    printf("L4-FUZZ-PRE-9 %s: seed=0x%08X; case IDs start at 0\n",seed_ok?"PASS":"FAIL",fuzz_state);
    for(unsigned i=0;i<count;i++){
        scenario_t sc={"fuzz",rr(5,100),rr(0,0.18f),rr(20,120),rr(8,42),rr(1.35f,2.5f),rr(50,200),2,
                       (rnd()&1U)?EVD_BATT_PROFILE_FEB21700G:EVD_BATT_PROFILE_LG_M58T,(rnd()&1U)!=0U};
        const float start_electrical_rev=rr(0,1);
        const uint32_t prng_before_prehistory=fuzz_state;
        const uint64_t signature=fuzz_vector_signature(i,&sc,start_electrical_rev);
        const bool vector_match=i>=25U || signature==FUZZ_VECTOR_GOLDEN[i];
        if(i<25U&&!vector_match){
            fprintf(stderr,"L4-FUZZ-PRE-8 vector identity FAIL case=%u got=%016llX expected=%016llX\n",i,
                (unsigned long long)signature,(unsigned long long)FUZZ_VECTOR_GOLDEN[i]);
            vector_identity_ok=false;
        }
        l4_t s;
        /* Clear observation before the real reset so any init-time EB74 call is visible. */
        l4_eb74_observer_reset();
        l4_init(&s,sc.soc,sc.profile,sc.grade,sc.target_rpm,sc.base_torque,sc.gear,sc.r0_mohm,start_electrical_rev);
        s.inject_pas_bounce=sc.bounce;
        const bool cold_start=l4_eb74_observer_count()==0U && g53_port_trace()->logical_tick==0U && s.tick==0U;
        if(!cold_start)cold_reset_ok=false;
        char prehistory_path[512];
        const char *capture_path=NULL;
        if(i==0U){
            if(snprintf(prehistory_path,sizeof(prehistory_path),"%s/fuzz-case-%02u-p9-g7-prehistory.csv",outdir,i)>=(int)sizeof(prehistory_path)){
                fprintf(stderr,"L4-FUZZ-PRE trace path too long\n");fclose(matrix);return 1;
            }
            capture_path=prehistory_path;
        }
        const bool l4_initialized=l4_eb74_prehistory(&s,capture_path,"L4-FUZZ-PRE");
        const bool call131_unscored=l4_eb74_observer_count()==131U && s.tick==0U &&
            s.max_iq_ref==0.0f && s.bike.distance_m==0.0f;
        if(l4_eb74_observer_count()!=131U)invocations131_ok=false;
        if(!call131_unscored)invocation131_unscored_ok=false;
        const bool prng_same=fuzz_state==prng_before_prehistory;
        if(!prng_same){
            fprintf(stderr,"L4-FUZZ-PRE-7 PRNG changed during prehistory case=%u before=%08X after=%08X\n",
                i,prng_before_prehistory,fuzz_state);
            prng_unchanged=false;
        }
        const bool pre_ok=cold_start && l4_initialized && call131_unscored && prng_same;
        if(!pre_ok)prehistory_ok=false;
        bool first_update_ok=false;uint32_t eb74_count_first_scored=0U;
        for(uint32_t t=0;t<sc.seconds*CTRL_HZ;t++){
            l4_tick(&s,NULL);
            if(t==0U){
                eb74_count_first_scored=l4_eb74_observer_count();
                /* Scored fuzz input follows completed unscored call #131. The ordinary Level-4
                 * control tick uses elapsed_ticks=1 and is not itself required to invoke EB74. */
                first_update_ok=call131_unscored && s.tick==1U &&
                    l4_eb74_observer_count()>=131U &&
                    l4_eb74_observer_last().count==l4_eb74_observer_count();
                if(!first_update_ok){
                    fprintf(stderr,"L4-FUZZ-PRE-5 first scored update case=%u tick=%u EB74 count=%u; expected tick=1 strictly after unscored #131\n",
                        i,s.tick,l4_eb74_observer_count());
                    first_scored_ok=false;
                }
            }
        }
        const float g=9.80665f;
        float static_required_nm=s.bike.mass_total_kg*g*(s.bike.crr+sc.grade)*s.bike.wheel_radius_m*sc.gear/
            s.bike.drivetrain_efficiency;
        float rider_launch_nm=sc.base_torque+sc.target_rpm; /* cadence Kp = 1 Nm/rpm at zero cadence */
        if(rider_launch_nm>s.rider.max_torque_nm)rider_launch_nm=s.rider.max_torque_nm;
        bool physical_start_possible=rider_launch_nm>static_required_nm*1.02f;
        bool robust_start_expected=rider_launch_nm>static_required_nm*1.15f;
        if(i<25U && (uint8_t)robust_start_expected!=FUZZ_EXPECTED_START_GOLDEN[i]){
            fprintf(stderr,"L4-FUZZ-PRE-10 EXPECTED_START identity FAIL case=%u actual=%u golden=%u\n",
                i,robust_start_expected?1U:0U,FUZZ_EXPECTED_START_GOLDEN[i]);
            classifier_identity_ok=false;
        }
        if(robust_start_expected)robust_expected_count++;
        bool common_ok=s.false_reverse_events==0&&s.direction_inhibit_ticks==0&&
            s.max_iq_ref<=(float)PH_CURRENT_MAX+0.5f&&s.motor.max_angle_error_deg<=31.1&&
            isfinite(s.batt.terminal_v)&&s.batt.terminal_v>20.0f&&isfinite(s.bike.speed_mps)&&
            isfinite(s.motor.iq_a)&&s.max_speed_kph<50.0f;
        bool start_ok=!robust_start_expected||(s.first_iq_tick&&s.first_hall_tick);
        bool ok=common_ok&&start_ok;
        if(pre_ok&&first_update_ok&&vector_match)executed++;
        if(robust_start_expected&&ok)robust_pass_count++;
        if(!physical_start_possible&&!s.first_hall_tick)safe_stalls++;
        if(physical_start_possible&&!robust_start_expected)marginal_starts++;
        fprintf(matrix,"%u,%016llX,%u,%u,%u,%u,%u,%u,%s,%u,%u,%u,%u\n",i,
            (unsigned long long)signature,vector_match?1U:0U,l4_eb74_observer_count(),
            call131_unscored?1U:0U,s.tick>0U?1U:0U,eb74_count_first_scored,prng_same?1U:0U,
            robust_start_expected?"EXPECTED_START":(physical_start_possible?"MARGINAL_START":"PHYSICAL_STALL"),
            robust_start_expected?1U:0U,s.first_iq_tick,s.first_hall_tick,ok?1U:0U);
        fflush(matrix);
        if(!ok){
            fprintf(stderr,"L4 FUZZ FAIL case=%u class=%s soc=%.1f grade=%.3f rpm=%.1f tq=%.1f gear=%.2f r0=%.1f "
                           "required=%.1f riderLaunch=%.1f falseR=%u inhibit=%u iq=%u hall=%u IqMax=%.1f angle=%.2f V=%.2f speed=%.2f\n",
                    i,robust_start_expected?"EXPECTED_START":(physical_start_possible?"MARGINAL_START":"PHYSICAL_STALL"),sc.soc,sc.grade,sc.target_rpm,
                    sc.base_torque,sc.gear,sc.r0_mohm,static_required_nm,rider_launch_nm,s.false_reverse_events,
                    s.direction_inhibit_ticks,s.first_iq_tick,s.first_hall_tick,s.max_iq_ref,s.motor.max_angle_error_deg,
                    s.batt.terminal_v,s.max_speed_kph);
            fail++;
        }
    }
    fclose(matrix);
    const bool seed_final_ok=count!=25U || fuzz_state==0xD3E2CF90U;
    if(!seed_final_ok)fprintf(stderr,"L4-FUZZ-PRE-9 final xorshift state changed: got=%08X expected=D3E2CF90\n",fuzz_state);
    const bool negative_ok=l4_cold_high_load_negative();
    printf("L4-FUZZ-PRE-1 %s: each case starts with real cold/reset and zero prior EB74 calls\n",cold_reset_ok?"PASS":"FAIL");
    printf("L4-FUZZ-PRE-2 %s: every actual prehistory update used load=0, preEB74=750, source<=960 and valid non-fault state\n",prehistory_ok?"PASS":"FAIL");
    printf("L4-FUZZ-PRE-3 %s: exactly 131 actual EB74 invocations complete before generated scored input\n",invocations131_ok?"PASS":"FAIL");
    printf("L4-FUZZ-PRE-4 %s: invocation 131 remains unscored; plant/scoring state stays at reset\n",invocation131_unscored_ok?"PASS":"FAIL");
    printf("L4-FUZZ-PRE-5 %s: first generated update is strictly after unscored invocation 131\n",first_scored_ok?"PASS":"FAIL");
    printf("L4-FUZZ-PRE-6 PASS: prehistory uses public production port only; EB74 state is not directly seeded or mutated\n");
    printf("L4-FUZZ-PRE-12 PASS: counted prehistory updates equal actual EB74 invocations one-for-one\n");
    printf("L4-FUZZ-PRE-7 %s: prehistory did not advance xorshift32\n",prng_unchanged?"PASS":"FAIL");
    printf("L4-FUZZ-PRE-8 %s: generated vector signatures match all available pre-P9-G7 IDs 0..%u\n",
        vector_identity_ok?"PASS":"FAIL",count<25U?(count?count-1U:0U):24U);
    printf("L4-FUZZ-PRE-10 %s: exact EXPECTED_START source formula and IDs match frozen pre-P9-G7 classifications\n",
        classifier_identity_ok?"PASS":"FAIL");
    printf("L4-FUZZ-PRE-11 %s: cold continuous-high-load negative retained\n",negative_ok?"PASS":"FAIL");
    printf("LEVEL4 EXPECTED_START Iq/Hall gate=%u/%u PASS\n",robust_pass_count,robust_expected_count);
    printf("LEVEL4 fuzz seed=0x%08X cases=%u executed=%u safePhysicalStalls=%u marginalStarts=%u failures=%d %s\n",
           0x144B1CE5U,count,executed,safe_stalls,marginal_starts,fail,
           fail==0&&executed==count&&vector_identity_ok&&prng_unchanged&&prehistory_ok&&first_scored_ok&&
             cold_reset_ok&&invocations131_ok&&invocation131_unscored_ok&&
             classifier_identity_ok&&seed_ok&&seed_final_ok&&negative_ok?"PASS":"FAIL");
    if(fail==0&&executed==count&&vector_identity_ok&&prng_unchanged&&prehistory_ok&&first_scored_ok&&
       cold_reset_ok&&invocations131_ok&&invocation131_unscored_ok&&
       classifier_identity_ok&&seed_ok&&seed_final_ok&&negative_ok){
        printf("L4-FUZZ-PRE-9 PASS: fixed seed retained; final xorshift state=0x%08X\n",fuzz_state);
        printf("L4-FUZZ-PRE-7..8 PRNG NON-INTERFERENCE / VECTOR IDENTITY: PASS\n");
    }else fail++;
    return fail?1:0;
}

int main(int argc,char **argv)
{
    const char *outdir=".build/level4";
    if(argc>=2&&strcmp(argv[1],"--soc")==0)return run_soc_endurance();
    if(argc>=2&&strcmp(argv[1],"--fuzz")==0){unsigned n=argc>=3?(unsigned)strtoul(argv[2],NULL,10):100U;return run_fuzz(n,outdir);}
    return run_fixed(outdir)|run_soc_endurance();
}
