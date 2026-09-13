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
#include "walk_assist_motor.h"

#include "battery_pack.h"
#include "bike_rider.h"

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
} l4_t;

static uint16_t torque_native_from_ckg(float ckg)
{
    double d; if(ckg<0.0f)ckg=0.0f;
    if(ckg<=TORQUE_DEFAULT_LOW_CENTIKG)
        d=ckg*(double)TORQUE_DEFAULT_LOW_NATIVE/TORQUE_DEFAULT_LOW_CENTIKG;
    else d=TORQUE_DEFAULT_LOW_NATIVE+(ckg-TORQUE_DEFAULT_LOW_CENTIKG)*
        (double)(TORQUE_DEFAULT_HIGH_NATIVE-TORQUE_DEFAULT_LOW_NATIVE)/
        (double)(TORQUE_DEFAULT_HIGH_CENTIKG-TORQUE_DEFAULT_LOW_CENTIKG);
    if(d>TORQUE_SPAN_MAX_NATIVE)d=TORQUE_SPAN_MAX_NATIVE;
    return (uint16_t)llround((double)TORQUE_ZERO_TARGET_NATIVE+d);
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
    cadence_filter_reset(); pas_sampler_init(0U);
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
        if(st>0){
            s->last_forward_gap=ev.gap?ev.gap:s->last_forward_gap;
            uint32_t x=(uint32_t)s->last_forward_gap*2U; if(x<PAS_STOP_TICKS)x=PAS_STOP_TICKS; if(x>PAS_STOP_TICKS_MAX)x=PAS_STOP_TICKS_MAX;
            s->stop_timeout=(uint16_t)x; pas_direction_on_step(st); torque_input_run_filter_step();
            pas_cadence_step_t cad=pas_cadence_forward_step(ev.tick,pas_direction_fwd_run()==1U?1U:0U);
            if(cad.pulse&&cad.measured){ s->ms.cadence=cad.rpm; s->start_phase=0U; cadence_filter_update(s->ms.cadence); }
        }else if(st<0){ s->false_reverse_events++; pas_cadence_break_epoch(0U); pas_direction_on_step(st); }
        else { pas_cadence_break_epoch(1U); pas_direction_on_step(st); }
    }
    if(pas_sampler_take_overflow())pas_cadence_break_epoch(2U);
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
    r.torque_load_centikg=ts->load_centikg; r.cadence_rpm=control_cadence; r.wheel_speed_x100=speed_x100;
    r.motor_erps=(uint16_t)((s->motor.erps>65535.0)?65535.0:llround(s->motor.erps));
    r.motor_erps_age_ticks=s->motor.hall_age_ticks; r.motor_voltage_utilization=(uint16_t)((s->ms.u_abs<0)?0:s->ms.u_abs);
    r.pas_forward=pedaling; r.pedaling_active=pedaling; r.crank_forward_steps=pas_direction_fwd_run();
    r.crank_direction_ok=direction_ok; r.real_stop=real_stop; r.wheel_valid=true;
    r.direction_inhibit_active=pas_direction_direction_inhibit_active();
    r.forward_confirmed_this_tick=pas_direction_forward_confirmed_last_call(); r.sample_tick=s->tick;
    r.start_phase=s->start_phase!=0U; r.torque_sensor_valid=true; r.pas_sensor_valid=true;
    rider_input_update(&r);

    float limp=soc_core_limp_factor(s->fw_soc.soc_display,s->limp_limit,s->limp_stage2);
    int32_t iq_limit=(int32_t)lroundf((float)PH_CURRENT_MAX*limp); if(iq_limit<0)iq_limit=0;
    int32_t batt_ma=(int32_t)lroundf(s->batt.current_a*1000.0f);
    uint16_t voltage_raw=(uint16_t)lroundf(s->batt.terminal_v*1000.0f/(float)CAL_BAT_V);
    ride_control_input_t in; memset(&in,0,sizeof(in));
    in.speed_x100=speed_x100; in.cadence_rpm=control_cadence; in.assist_level_index=s->assist_level;
    in.battery_voltage_mv=(uint32_t)lroundf(s->batt.terminal_v*1000.0f);
    in.iq_scale=PH_CURRENT_MAX; in.ride_core_iq_limit=iq_limit; in.phase_current_max=iq_limit;
    in.battery_current_mA=batt_ma; in.battery_current_max=DEFAULT_BATTERY_CURRENT_MAX_MA;
    in.u_abs=s->ms.u_abs; in.cal_i=CAL_I; in.current_iq=s->ms.i_q; in.current_id=s->ms.i_d;
    in.voltage_raw=voltage_raw; in.voltage_min_raw=VOLTAGE_MIN;
    in.controller_temperature_c=(int16_t)lroundf(s->controller_temp_c);
    in.cadence_filtered_x8=cadence_filter_get_x8(); in.speed_limit_x100=SPEEDLIMIT;
    in.legal_enabled=true; in.offroad=false; in.walk_active=false;
    in.safety_cut_non_direction=s->brake; in.service_cut_active=false; in.elapsed_ticks=1U;
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
}

typedef struct {
    const char *name; float soc,grade,target_rpm,base_torque,gear,r0_mohm; unsigned seconds;
    evd_battery_profile_t profile; bool bounce;
} scenario_t;

static int run_scenario(const scenario_t *sc,const char *outdir)
{
    l4_t s; l4_init(&s,sc->soc,sc->profile,sc->grade,sc->target_rpm,sc->base_torque,sc->gear,sc->r0_mohm,0.03);
    s.inject_pas_bounce=sc->bounce;
    char path[512]; snprintf(path,sizeof(path),"%s/%s.csv",outdir,sc->name);
    FILE *f=fopen(path,"wb"); if(!f){perror(path);return 1;}
    fprintf(f,"time_s,distance_m,speed_kph,cadence_rpm,rider_torque_nm,torque_ckg,battery_v,battery_a,true_soc,fw_soc,iq_request,iq_ref,iq_actual,motor_erps,u_abs,limp,session,debug,battery_limit,hall_age\n");
    for(uint32_t t=0;t<sc->seconds*CTRL_HZ;t++)l4_tick(&s,f);
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
    int fail=0; for(size_t i=0;i<sizeof(v)/sizeof(v[0]);i++)fail+=run_scenario(&v[i],outdir);
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
    int fail=0; unsigned safe_stalls=0, marginal_starts=0; (void)outdir;
    for(unsigned i=0;i<count;i++){
        scenario_t sc={"fuzz",rr(5,100),rr(0,0.18f),rr(20,120),rr(8,42),rr(1.35f,2.5f),rr(50,200),2,
                       (rnd()&1U)?EVD_BATT_PROFILE_FEB21700G:EVD_BATT_PROFILE_LG_M58T,(rnd()&1U)!=0U};
        l4_t s; l4_init(&s,sc.soc,sc.profile,sc.grade,sc.target_rpm,sc.base_torque,sc.gear,sc.r0_mohm,rr(0,1));
        s.inject_pas_bounce=sc.bounce;
        for(uint32_t t=0;t<sc.seconds*CTRL_HZ;t++)l4_tick(&s,NULL);
        const float g=9.80665f;
        float static_required_nm=s.bike.mass_total_kg*g*(s.bike.crr+sc.grade)*s.bike.wheel_radius_m*sc.gear/
            s.bike.drivetrain_efficiency;
        float rider_launch_nm=sc.base_torque+sc.target_rpm; /* cadence Kp = 1 Nm/rpm at zero cadence */
        if(rider_launch_nm>s.rider.max_torque_nm)rider_launch_nm=s.rider.max_torque_nm;
        bool physical_start_possible=rider_launch_nm>static_required_nm*1.02f;
        bool robust_start_expected=rider_launch_nm>static_required_nm*1.15f;
        bool common_ok=s.false_reverse_events==0&&s.direction_inhibit_ticks==0&&
            s.max_iq_ref<=(float)PH_CURRENT_MAX+0.5f&&s.motor.max_angle_error_deg<=31.1&&
            isfinite(s.batt.terminal_v)&&s.batt.terminal_v>20.0f&&isfinite(s.bike.speed_mps)&&
            isfinite(s.motor.iq_a)&&s.max_speed_kph<50.0f;
        bool start_ok=!robust_start_expected||(s.first_iq_tick&&s.first_hall_tick);
        bool ok=common_ok&&start_ok;
        if(!physical_start_possible&&!s.first_hall_tick)safe_stalls++;
        if(physical_start_possible&&!robust_start_expected)marginal_starts++;
        if(!ok){
            fprintf(stderr,"L4 FUZZ FAIL case=%u class=%s soc=%.1f grade=%.3f rpm=%.1f tq=%.1f gear=%.2f r0=%.1f "
                           "required=%.1f riderLaunch=%.1f falseR=%u inhibit=%u iq=%u hall=%u IqMax=%.1f angle=%.2f V=%.2f speed=%.2f\n",
                    i,robust_start_expected?"EXPECTED_START":(physical_start_possible?"MARGINAL_START":"PHYSICAL_STALL"),sc.soc,sc.grade,sc.target_rpm,
                    sc.base_torque,sc.gear,sc.r0_mohm,static_required_nm,rider_launch_nm,s.false_reverse_events,
                    s.direction_inhibit_ticks,s.first_iq_tick,s.first_hall_tick,s.max_iq_ref,s.motor.max_angle_error_deg,
                    s.batt.terminal_v,s.max_speed_kph);
            if(++fail>=10)break;
        }
    }
    printf("LEVEL4 fuzz seed=0x%08X cases=%u safePhysicalStalls=%u marginalStarts=%u failures=%d %s\n",
           0x144B1CE5U,count,safe_stalls,marginal_starts,fail,fail?"FAIL":"PASS");
    return fail?1:0;
}

int main(int argc,char **argv)
{
    const char *outdir=".build/level4";
    if(argc>=2&&strcmp(argv[1],"--soc")==0)return run_soc_endurance();
    if(argc>=2&&strcmp(argv[1],"--fuzz")==0){unsigned n=argc>=3?(unsigned)strtoul(argv[2],NULL,10):100U;return run_fuzz(n,outdir);}
    return run_fixed(outdir)|run_soc_endurance();
}
