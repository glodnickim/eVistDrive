/*
 * EVistDrive closed-loop supervisory SIL.
 * Runs shipped production C for PAS -> torque -> rider_input -> ride_control -> final 16 kHz Iq.
 * The plant is deliberately small but CLOSED LOOP: final Iq accelerates a rotor, rotor motion
 * generates Hall edges/age/speed, and those facts return to ride_control (notably START preload).
 * The rider plant generates physical crank angle, quadrature PAS and a two-leg torque waveform.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "assist_modes.h"
#include "ap2_limits.h"
#include "assist_pipeline.h"
#include "cadence_filter.h"
#include "config.h"
#include "motor_core.h"
#include "pas_cadence.h"
#include "pas_direction.h"
#include "pas_liveness.h"
#include "pas_sampler.h"
#include "rider_input.h"
#include "ride_control.h"
#include "torque_input.h"
#include "tuning_config.h"

#ifdef EVD_SIL_REAL_FOC
#include "FOC.h"
#include "foc_current_loop.h"
#include "pwm_geometry.h"
#include "quiet_zero.h"
#include "rotor_angle.h"
#include "rotor_motion.h"
#include "walk_assist_motor.h"
#endif

#define CTRL_HZ 4000U
#define INNER_PER_CTRL 4U
#define PI 3.14159265358979323846
#define TEST_BATTERY_MV 42000U
#define TEST_VOLTAGE_RAW 2000
#define TEST_TEMP_C 25

typedef struct {
    double iq_actual;
    double erps;
    double theta_e_rev;
    double breakaway_iq;
    double accel_erps_s_per_iq;
    double drag_per_s;
    uint16_t hall_age_ticks;
    uint32_t hall_edges;
    uint32_t last_hall_tick;
    uint32_t prev_hall_tick;
    bool hall_edge_this_ctrl;
#ifdef EVD_SIL_REAL_FOC
    double id_a;
    double iq_a;
    double r_ohm;
    double ld_h;
    double lq_h;
    double flux_wb;
    rotor_angle_state_t angle_state;
    rotor_motion_t rotor_motion;
    uint32_t hall_sequence;
    uint32_t tics_filtered_8;
    uint16_t last_capture_500k;
    double hall_elapsed_500k;
    int64_t hall_sector_index;
    uint8_t sixstep_untrusted;
    double max_angle_error_deg;
#endif
} plant_t;

#ifdef EVD_SIL_REAL_FOC
#define SIL_CURRENT_A_PER_COUNT 0.095
#define SIL_VBUS 40.0
#define SIL_SQRT3 1.7320508075688772
PI_control_t PI_iq, PI_id;
uint8_t ui_8_PWM_ON_Flag = 1U;
uint8_t bridge_lifecycle = BRIDGE_LIFECYCLE_RUN;
int32_t switchtime[3];
uint16_t pwm_applied[3];
static MotorState_t *g_foc_ms;
static plant_t *g_foc_plant;
static MotorParams_t g_foc_mp;
static quiet_zero_t g_quiet_zero;
static int32_t g_qz_entry_erps;
static int32_t g_qz_abort_erps;
static int32_t g_qz_peak_abs_iq;

void timer_channel_output_pulse_value_config(uint32_t timer, uint16_t ch, uint32_t value)
{ (void)timer; (void)ch; (void)value; }
void timer_primary_output_config(uint32_t timer, uint32_t enable)
{ (void)timer; (void)enable; }

static void sil_pi_init(PI_control_t *p, int16_t limit_i)
{
    memset(p, 0, sizeof(*p));
    p->gain_p = 1.5f;
    p->gain_i = 0.01f;
    p->limit_i = limit_i;
    p->limit_output = _U_MAX;
    p->max_step = 15;
    p->shift = 11;
    p->aw_inv_kp_q15 = 21845;
}

void runPIcontrol(void)
{
    foc_current_loop_result_t r;
    quiet_zero_action_t qz;

    fast_iq_slew_tick(ride_control_final_iq_slew_mailbox(), &g_foc_ms->i_q_setpoint);
    PI_iq.recent_value = (int16_t)g_foc_ms->i_q;
    PI_iq.setpoint = g_foc_ms->i_q_setpoint;

    /* Keep the electrical SIL on the production zero-current lifecycle too. This mirrors the
     * QZERO block in main.c: the same state machine decides whether the current regulators'
     * integrals are faded/held, while the real PI and vector limiter still execute below. */
    {
        quiet_zero_input_t in = {
            .iq_ref = g_foc_ms->i_q_setpoint,
            .zero_policy_quiet =
                fast_iq_slew_current_zero_policy() == FIS_ZERO_POLICY_QUIET,
            .iq_measured = g_foc_ms->i_q,
            .id_measured = g_foc_ms->i_d,
            .abort_current = QZERO_ABORT_CURRENT,
            .rotor_erps = g_foc_plant ? (int32_t)g_foc_plant->rotor_motion.edge_erps : 0,
            .speed_fresh = g_foc_plant ?
                rotor_motion_speed_fresh(&g_foc_plant->rotor_motion,
                                         g_foc_plant->hall_age_ticks) : false,
            .min_brake_erps = RIDE_COAST_RELEASE_ERPS,
            .iq_integral = PI_iq.integral_part,
            .id_integral = PI_id.integral_part
        };
        quiet_zero_tick(&g_quiet_zero, &in, &qz);
        if (qz.entered) g_qz_entry_erps = in.rotor_erps;
        if (g_quiet_zero.state != (uint32_t)QZERO_INACTIVE) {
            int32_t a = in.iq_measured < 0 ? -in.iq_measured : in.iq_measured;
            if (a > g_qz_peak_abs_iq) g_qz_peak_abs_iq = a;
        }
        if (qz.aborted) g_qz_abort_erps = in.rotor_erps;
    }

    if (qz.apply_integral) {
        PI_iq.integral_part = qz.iq_integral;
        PI_id.integral_part = qz.id_integral;
    }
    if (qz.freeze_aw || qz.clear_aw_edge) {
        PI_iq.aw_sat_error = 0;
        PI_id.aw_sat_error = 0;
        g_foc_ms->u_q_sat_err = 0;
        g_foc_ms->u_d_sat_err = 0;
    }

    foc_current_loop_step(g_foc_ms, &PI_iq, &PI_id,
                          qz.apply_integral ? 1U : 0U,
                          qz.iq_integral, qz.id_integral, &r);
}

static double sil_wrap_pi(double a)
{
    while (a >= PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}

static q31_t sil_angle_q31(double rev)
{
    double a = sil_wrap_pi(rev * 2.0 * PI);
    double x = a / PI * 2147483648.0;
    if (x >= 2147483647.0) x = 2147483647.0;
    if (x < -2147483648.0) x = -2147483648.0;
    return (q31_t)llround(x);
}

static double sil_q31_to_rad(q31_t a)
{
    return ((double)a / 2147483648.0) * PI;
}

static q31_t sil_estimated_hall_angle(plant_t *p)
{
    uint32_t elapsed = (uint32_t)llround(p->hall_elapsed_500k);
    if (elapsed > 65535U) elapsed = 65535U;

    if (elapsed > (uint32_t)(SIXSTEPTHRESHOLD << 1)) {
        p->last_capture_500k = (uint16_t)(SIXSTEPTHRESHOLD << 1);
        p->tics_filtered_8 = (uint32_t)p->last_capture_500k << 3;
    }
    if (p->last_capture_500k < SIXSTEPTHRESHOLD && elapsed < 200U)
        p->sixstep_untrusted = 0U;
    if (p->last_capture_500k > ((SIXSTEPTHRESHOLD * 6U) >> 2))
        p->sixstep_untrusted = 1U;

    double boundary_rev = (double)p->hall_sector_index / 6.0;
    rotor_angle_input_t in = {
        .hall_angle = sil_angle_q31(boundary_rev),
        .angle_correction = 0,
        .direction = 1,
        .tim2_recent = elapsed,
        .tics_filtered_8 = p->tics_filtered_8,
        .want_untrusted = p->sixstep_untrusted != 0U,
        .stalled = rotor_motion_angle_stale(&p->rotor_motion, p->hall_age_ticks),
        .fallback_sign = 1,
        .hall_sequence = p->hall_sequence,
        .hall_sequence_valid = true
    };
    q31_t theta = rotor_angle_update(&p->angle_state, &in);
    double err = sil_wrap_pi(sil_q31_to_rad(theta) -
                             sil_wrap_pi(p->theta_e_rev * 2.0 * PI));
    double deg = fabs(err) * 180.0 / PI;
    if (deg > p->max_angle_error_deg) p->max_angle_error_deg = deg;
    return theta;
}

static void sil_dq_to_ab(double d, double q, double th, double *a, double *b)
{
    double c = cos(th), s = sin(th);
    *a = d * c - q * s;
    *b = d * s + q * c;
}

static void sil_ab_to_dq(double a, double b, double th, double *d, double *q)
{
    double c = cos(th), s = sin(th);
    *d = a * c + b * s;
    *q = -a * s + b * c;
}

static void sil_current_to_phase_counts(const plant_t *p, int16_t *ia, int16_t *ib)
{
    double a, b;
    double th = p->theta_e_rev * 2.0 * PI;
    sil_dq_to_ab(p->id_a, p->iq_a, th, &a, &b);
    double ib_a = (-a + SIL_SQRT3 * b) * 0.5;
    long ca = llround(a / SIL_CURRENT_A_PER_COUNT);
    long cb = llround(ib_a / SIL_CURRENT_A_PER_COUNT);
    if (ca > 32767) ca = 32767;
    if (ca < -32768) ca = -32768;
    if (cb > 32767) cb = 32767;
    if (cb < -32768) cb = -32768;
    *ia = (int16_t)ca;
    *ib = (int16_t)cb;
}

static void sil_pwm_to_ab(const uint16_t pwm[3], double *alpha, double *beta)
{
    double da = (double)pwm[0] / (double)_T;
    double db = (double)pwm[1] / (double)_T;
    double dc = (double)pwm[2] / (double)_T;
    double mean = (da + db + dc) / 3.0;
    /* TIMER/PWM polarity established by the standalone real-FOC electrical SIL. */
    double va = -SIL_VBUS * (da - mean);
    double vb = -SIL_VBUS * (db - mean);
    *alpha = va;
    *beta = (va + 2.0 * vb) / SIL_SQRT3;
}
#endif

typedef struct {
    double rpm;
    double crank_rev;
    double cadence_ripple_fraction;
    double next_pas_edge_rev;
    uint8_t state_index;
    uint8_t ab;
    uint8_t bounce_ticks;
    bool inject_bounce;
    uint32_t forward_edges;
    bool active;
    double torque_mean_ckg;
    double torque_ripple_ckg;
} rider_plant_t;

typedef struct {
    MotorState_t MS;
    plant_t plant;
    rider_plant_t rider;
    uint32_t tick;
    uint16_t last_forward_gap;
    uint16_t stop_timeout;
    uint8_t start_phase;
    uint32_t direction_inhibit_ticks;
    uint32_t false_reverse_events;
    uint32_t first_permission_tick;
    uint32_t first_iq_tick;
    uint32_t first_hall_tick;
    int32_t iq_min_run;
    int32_t iq_max_run;
    double iq_sum_run;
    double iq_sq_sum_run;
    uint32_t iq_samples_run;
    bool use_filtered_control_cadence;
    int32_t iq_min_all;
    int32_t iq_max_all;
} sim_t;

/* Raw forward ring for PAS_DIR_SIGN=-1: 00 -> 10 -> 11 -> 01 -> 00. */
static const uint8_t FWD_AB[4] = {0U, 2U, 3U, 1U};

static uint16_t sensor_native_from_ckg(double ckg)
{
    double d;
    if (ckg < 0.0) ckg = 0.0;
    if (ckg <= TORQUE_DEFAULT_LOW_CENTIKG) {
        d = ckg * (double)TORQUE_DEFAULT_LOW_NATIVE / TORQUE_DEFAULT_LOW_CENTIKG;
    } else {
        d = TORQUE_DEFAULT_LOW_NATIVE +
            (ckg - TORQUE_DEFAULT_LOW_CENTIKG) *
            (double)(TORQUE_DEFAULT_HIGH_NATIVE - TORQUE_DEFAULT_LOW_NATIVE) /
            (double)(TORQUE_DEFAULT_HIGH_CENTIKG - TORQUE_DEFAULT_LOW_CENTIKG);
    }
    if (d > TORQUE_SPAN_MAX_NATIVE) d = TORQUE_SPAN_MAX_NATIVE;
    return (uint16_t)llround((double)TORQUE_ZERO_TARGET_NATIVE + d);
}

static void plant_init(plant_t *p, double breakaway_iq)
{
    memset(p, 0, sizeof(*p));
    p->breakaway_iq = breakaway_iq;
    p->accel_erps_s_per_iq = 20.0; /* plant parameter, not firmware truth */
    p->drag_per_s = 5.0;
    p->hall_age_ticks = 0xFFFFU;
#ifdef EVD_SIL_REAL_FOC
    p->r_ohm = 0.060;
    p->ld_h = 80e-6;
    p->lq_h = 80e-6;
    p->flux_wb = 0.015;
    rotor_angle_reset(&p->angle_state);
    memset(&p->rotor_motion, 0, sizeof(p->rotor_motion));
    p->tics_filtered_8 = 128000U;
    p->last_capture_500k = 0U;
    p->hall_elapsed_500k = 0.0;
    p->hall_sector_index = (int64_t)floor(p->theta_e_rev * 6.0);
    p->sixstep_untrusted = 0U;
    sil_pi_init(&PI_iq, _U_MAX);
    sil_pi_init(&PI_id, 1800);
    quiet_zero_reset(&g_quiet_zero);
    g_qz_entry_erps = 0;
    g_qz_abort_erps = 0;
    g_qz_peak_abs_iq = 0;
    pwm_geometry_init();
#endif
}

#ifdef EVD_SIL_REAL_FOC
static void plant_set_start_angle(plant_t *p, double electrical_rev)
{
    p->theta_e_rev = electrical_rev;
    p->hall_sector_index = (int64_t)floor(p->theta_e_rev * 6.0);
    p->hall_elapsed_500k = 0.0;
    p->hall_sequence = 0U;
    p->tics_filtered_8 = 128000U;
    p->last_capture_500k = 0U;
    p->sixstep_untrusted = 0U;
    p->max_angle_error_deg = 0.0;
    memset(&p->rotor_motion, 0, sizeof(p->rotor_motion));
    rotor_angle_reset(&p->angle_state);
}
#endif

static void plant_inner_tick(plant_t *p, MotorState_t *ms, int32_t iq_cmd, uint32_t ctrl_tick)
{
    const double dt = 1.0 / 16000.0;
#ifdef EVD_SIL_REAL_FOC
    int16_t ia, ib;
    sil_current_to_phase_counts(p, &ia, &ib);
    g_foc_ms = ms;
    g_foc_plant = p;
    g_foc_mp.com_mode = Hallsensor;
    g_foc_mp.reverse = 1;
    q31_t theta_control = sil_estimated_hall_angle(p);
    FOC_calculation(ia, ib, theta_control,
                    (int16_t)ms->i_q_setpoint, ms, &g_foc_mp);
    (void)pwm_geometry_apply(switchtime, pwm_applied, (uint16_t)_T);

    double alpha, beta, vd, vq;
    sil_pwm_to_ab(pwm_applied, &alpha, &beta);
    sil_ab_to_dq(alpha, beta, p->theta_e_rev * 2.0 * PI, &vd, &vq);

    /* Controller stays at 16 kHz. Only the virtual motor is sub-stepped. */
    const int substeps = 8;
    const double h = dt / (double)substeps;
    for (int n = 0; n < substeps; n++) {
        double we = 2.0 * PI * p->erps;
        double did = (vd - p->r_ohm * p->id_a + we * p->lq_h * p->iq_a) / p->ld_h;
        double diq = (vq - p->r_ohm * p->iq_a - we * (p->ld_h * p->id_a + p->flux_wb)) / p->lq_h;
        p->id_a += did * h;
        p->iq_a += diq * h;
    }
    p->iq_actual = p->iq_a / SIL_CURRENT_A_PER_COUNT;
    /* A positive static/load torque opposes forward motion. Once moving, negative electrical Iq
     * is allowed to brake the virtual rotor; clamping negative drive to zero would make QZERO
     * electrically visible but mechanically ineffective and would hide exactly the STOP path
     * this backend is meant to validate. */
    double load_iq = (p->erps > 0.01 || p->iq_actual > p->breakaway_iq) ?
        p->breakaway_iq : 0.0;
    double drive = p->iq_actual - load_iq;
    double accel = drive * p->accel_erps_s_per_iq - p->drag_per_s * p->erps;
    p->erps += accel * dt;
    if (p->erps < 0.0) p->erps = 0.0;
#else
    (void)ms;
    /* Current loop stand-in: ~2 ms current tracking. The production PI is independently host-
     * tested; here we need its actuator effect so supervisory states see real/no Hall motion. */
    const double alpha = dt / (0.002 + dt);
    p->iq_actual += ((double)iq_cmd - p->iq_actual) * alpha;

    double drive = p->iq_actual - p->breakaway_iq;
    if (drive < 0.0) drive = 0.0;
    double accel = drive * p->accel_erps_s_per_iq - p->drag_per_s * p->erps;
    p->erps += accel * dt;
    if (p->erps < 0.0) p->erps = 0.0;
#endif

    double old = p->theta_e_rev;
    p->theta_e_rev += p->erps * dt;
#ifdef EVD_SIL_REAL_FOC
    p->hall_elapsed_500k += 500000.0 * dt;
#endif
    /* Hall edge each 1/6 electrical revolution. */
    uint64_t old_sector = (uint64_t)floor(old * 6.0);
    uint64_t new_sector = (uint64_t)floor(p->theta_e_rev * 6.0);
    if (new_sector != old_sector) {
        p->hall_edge_this_ctrl = true;
        p->hall_edges += (uint32_t)(new_sector - old_sector);
        p->prev_hall_tick = p->last_hall_tick;
        p->last_hall_tick = ctrl_tick;
#ifdef EVD_SIL_REAL_FOC
        uint32_t capture = (uint32_t)llround(p->hall_elapsed_500k);
        if (capture > 65535U) capture = 65535U;
        if (capture == 0U) capture = 1U;
        rotor_motion_note_edge(&p->rotor_motion, p->hall_age_ticks, (uint16_t)capture);
        p->hall_sequence++;
        p->last_capture_500k = (uint16_t)capture;
        p->tics_filtered_8 -= p->tics_filtered_8 >> 3;
        p->tics_filtered_8 += capture;
        p->hall_elapsed_500k = 0.0;
        p->hall_sector_index = (int64_t)new_sector;
#endif
    }
}

static void rider_init(rider_plant_t *r, double rpm, double cadence_ripple_fraction,
                       double mean_ckg, double ripple_ckg, bool bounce)
{
    memset(r, 0, sizeof(*r));
    r->rpm = rpm;
    r->cadence_ripple_fraction = cadence_ripple_fraction;
    r->torque_mean_ckg = mean_ckg;
    r->torque_ripple_ckg = ripple_ckg;
    r->state_index = 0U;
    r->ab = FWD_AB[0];
    r->next_pas_edge_rev = 1.0 / (double)PAS_TRANSITIONS_PER_REV;
    r->inject_bounce = bounce;
    r->active = true;
}

static uint8_t rider_pas_tick(rider_plant_t *r)
{
    if (!r->active) {
        r->bounce_ticks = 0U;
        return r->ab;
    }
    if (r->bounce_ticks == 1U) {
        r->bounce_ticks = 2U;
        /* return briefly to the previous raw state: one-tick reverse bounce */
        return FWD_AB[(r->state_index + 3U) & 3U];
    }
    if (r->bounce_ticks == 2U) {
        r->bounce_ticks = 0U;
        return r->ab;
    }
    /* Human cadence is not uniform inside a crank revolution. Two leg pushes speed the crank
     * up and the two dead spots slow it down. This is deliberately a physical-angle ripple so
     * the PAS period estimator sees the same local-speed modulation it sees on a bicycle. */
    double inst_rpm = r->rpm *
        (1.0 + r->cadence_ripple_fraction * sin(4.0 * PI * r->crank_rev));
    if (inst_rpm < 1.0) inst_rpm = 1.0;
    r->crank_rev += inst_rpm / (60.0 * CTRL_HZ);
    if (r->crank_rev >= r->next_pas_edge_rev) {
        r->next_pas_edge_rev += 1.0 / (double)PAS_TRANSITIONS_PER_REV;
        r->state_index = (uint8_t)((r->state_index + 1U) & 3U);
        r->ab = FWD_AB[r->state_index];
        r->forward_edges++;
        /* deterministic bounce on every 7th physical edge */
        if (r->inject_bounce && (r->forward_edges % 7U) == 0U) r->bounce_ticks = 1U;
    }
    return r->ab;
}

static double rider_torque_ckg(const rider_plant_t *r, uint32_t tick)
{
    (void)tick;
    if (!r->active) return 0.0;
    /* two leg pushes per crank revolution; mean plus bounded sinusoidal ripple */
    double v = r->torque_mean_ckg + r->torque_ripple_ckg * sin(4.0 * PI * r->crank_rev);
    return v < 0.0 ? 0.0 : v;
}

static void sim_init(sim_t *s, double rpm, double cadence_ripple_fraction,
                     double mean_ckg, double ripple_ckg, bool bounce,
                     double breakaway_iq, bool use_filtered_control_cadence)
{
    memset(s, 0, sizeof(*s));
    torque_input_init();
    torque_input_startup_zero(TORQUE_ZERO_TARGET_NATIVE);
    torque_input_set_run_window_deg(tuning_config_assist_torque_run_window_deg());
    assist_modes_init();
    assist_modes_set_active_bank(0U);
    motor_core_init(&s->MS);
#ifdef EVD_SIL_REAL_FOC
    s->MS.hall_angle_detect_flag = 1U;
    foc_current_feedback_reset(&s->MS);
#endif
    ride_control_init();
    pas_direction_init();
    pas_liveness_init();
    pas_cadence_reset();
    cadence_filter_reset();
    pas_sampler_init(0U);
    rider_init(&s->rider, rpm, cadence_ripple_fraction, mean_ckg, ripple_ckg, bounce);
    plant_init(&s->plant, breakaway_iq);
    s->last_forward_gap = PAS_STOP_TICKS;
    s->stop_timeout = PAS_STOP_TICKS;
    s->iq_min_run = 0x7fffffff;
    s->iq_max_run = -0x7fffffff;
    s->iq_min_all = 0x7fffffff;
    s->iq_max_all = -0x7fffffff;
    s->use_filtered_control_cadence = use_filtered_control_cadence;
    /* seed physical PAS state */
    pas_sampler_isr_tick(s->rider.ab, 0U);
}

static void process_pas(sim_t *s, uint8_t ab)
{
    pas_sampler_isr_tick(ab, s->tick);
    pas_step_event_t ev;
    while (pas_sampler_pop(&ev)) {
        int8_t st = ev.step;
        if (st > 0) {
            s->last_forward_gap = ev.gap ? ev.gap : s->last_forward_gap;
            uint32_t x = (uint32_t)s->last_forward_gap * 2U;
            if (x < PAS_STOP_TICKS) x = PAS_STOP_TICKS;
            if (x > PAS_STOP_TICKS_MAX) x = PAS_STOP_TICKS_MAX;
            s->stop_timeout = (uint16_t)x;
            pas_direction_on_step(st);
            torque_input_run_filter_step();
            pas_cadence_step_t cad = pas_cadence_forward_step(ev.tick,
                pas_direction_fwd_run() == 1U ? 1U : 0U);
            if (cad.pulse && cad.measured) {
                s->MS.cadence = cad.rpm;
                s->start_phase = 0U;
                cadence_filter_update(s->MS.cadence);
            }
        } else if (st < 0) {
            s->false_reverse_events++;
            pas_cadence_break_epoch(0U);
            pas_direction_on_step(st);
        } else {
            pas_cadence_break_epoch(1U);
            pas_direction_on_step(st);
        }
    }
    if (pas_sampler_take_overflow()) pas_cadence_break_epoch(2U);
    if (s->MS.cadence == 0U && !s->start_phase &&
        pas_direction_fwd_run() >= START_PHASE_STEPS) s->start_phase = 1U;
}

static void sim_ctrl_tick(sim_t *s, FILE *csv)
{
    s->tick++;
    s->plant.hall_edge_this_ctrl = false;

    uint8_t ab = rider_pas_tick(&s->rider);
    process_pas(s, ab);

    uint32_t idle = s->tick - pas_sampler_last_transition_tick();
    pas_liveness_update(idle, s->stop_timeout);
    bool real_stop = pas_liveness_stopped();
    if (real_stop) {
        s->MS.cadence = 0U;
        s->start_phase = 0U;
        cadence_filter_reset();
        pas_cadence_reset();
        pas_direction_on_stop();
    }
    bool crank_direction_ok = (s->MS.cadence > 0U || s->start_phase) && !real_stop;
    bool pedaling = crank_direction_ok &&
        pas_direction_fwd_run() >= tuning_config_start_steps();
    uint8_t control_cadence = s->MS.cadence;
    if (s->use_filtered_control_cadence) control_cadence = cadence_filter_get();

    double load_ckg = rider_torque_ckg(&s->rider, s->tick);
    uint16_t raw = sensor_native_from_ckg(load_ckg);
    torque_input_update(raw, torque_input_correct(raw), true);
    const torque_snapshot_t *ts = torque_input_get_snapshot();

    rider_input_t r;
    memset(&r, 0, sizeof(r));
    r.torque_raw_mv = raw;
    r.torque_corrected_mv = torque_input_correct(raw);
    r.torque_filtered = ts->delta_native;
    r.torque_assist_now_native = ts->assist_delta_native;
    r.torque_assist_filtered = ts->assist_delta_filtered_native;
    r.torque_run_filtered = ts->assist_delta_run_native;
    r.torque_load_centikg = ts->load_centikg;
    r.cadence_rpm = control_cadence;
    r.wheel_speed_x100 = 0U;
    r.motor_erps = (uint16_t)(s->plant.erps > 65535.0 ? 65535.0 : llround(s->plant.erps));
    r.motor_erps_age_ticks = s->plant.hall_age_ticks;
    /* Approximate increasing electrical voltage utilization with motor speed. At standstill the
     * production demand path deliberately leans on its launch anchor; once the virtual motor is
     * spinning this moves the calculation into the measured-duty branch, which is required to
     * exercise cadence-dependent Power/eMTB demand in closed loop. This is a SIL plant law, not
     * a claimed M820 calibration. */
    {
#ifdef EVD_SIL_REAL_FOC
        double u = (double)s->MS.u_abs;
#else
        double u = 600.0 + 2.0 * s->plant.erps;
        if (u > 1800.0) u = 1800.0;
#endif
        r.motor_voltage_utilization = (uint16_t)llround(u);
    }
    r.pas_forward = pedaling;
    r.pedaling_active = pedaling;
    r.crank_forward_steps = pas_direction_fwd_run();
    r.crank_direction_ok = crank_direction_ok;
    r.real_stop = real_stop;
    r.wheel_valid = false;
    r.direction_inhibit_active = pas_direction_direction_inhibit_active();
    r.forward_confirmed_this_tick = pas_direction_forward_confirmed_last_call();
    r.sample_tick = s->tick;
    r.start_phase = s->start_phase != 0U;
    r.torque_sensor_valid = true;
    r.pas_sensor_valid = true;
    rider_input_update(&r);

    ride_control_input_t in;
    memset(&in, 0, sizeof(in));
    in.speed_x100 = 0U;
    in.cadence_rpm = control_cadence;
    in.assist_level_index = 3U; /* default Power level 3 */
    in.battery_voltage_mv = TEST_BATTERY_MV;
    in.iq_scale = PH_CURRENT_MAX;
    in.ride_core_iq_limit = PH_CURRENT_MAX;
    in.phase_current_max = PH_CURRENT_MAX;
    in.battery_current_mA = 0;
    in.battery_current_max = 15000;
    in.u_abs =
#ifdef EVD_SIL_REAL_FOC
        s->MS.u_abs;
#else
        600;
#endif
    in.cal_i = 95;
    in.current_iq =
#ifdef EVD_SIL_REAL_FOC
        s->MS.i_q;
#else
        (int32_t)llround(s->plant.iq_actual);
#endif
    in.current_id =
#ifdef EVD_SIL_REAL_FOC
        s->MS.i_d;
#else
        0;
#endif
    in.voltage_raw = TEST_VOLTAGE_RAW;
    in.voltage_min_raw = VOLTAGE_MIN;
    in.controller_temperature_c = TEST_TEMP_C;
    in.cadence_filtered_x8 = cadence_filter_get_x8();
    in.speed_limit_x100 = SPEEDLIMIT;
    in.legal_enabled = true;
    in.elapsed_ticks = 1U;
    ride_control_update(&in);

    if (assist_pipeline_pas_state() == AP2_PAS_FORWARD && s->first_permission_tick == 0U)
        s->first_permission_tick = s->tick;

    for (unsigned k = 0; k < INNER_PER_CTRL; k++) {
#ifndef EVD_SIL_REAL_FOC
        fast_iq_slew_tick(ride_control_final_iq_slew_mailbox(), &s->MS.i_q_setpoint);
#endif
        plant_inner_tick(&s->plant, &s->MS, s->MS.i_q_setpoint, s->tick);
    }
    if (s->plant.hall_edge_this_ctrl) {
        s->plant.hall_age_ticks = 0U;
        if (s->first_hall_tick == 0U) s->first_hall_tick = s->tick;
    } else if (s->plant.hall_age_ticks < 0xFFFFU) {
        s->plant.hall_age_ticks++;
    }
    if (s->MS.i_q_setpoint > 0 && s->first_iq_tick == 0U) s->first_iq_tick = s->tick;
    if (s->MS.i_q_setpoint < s->iq_min_all) s->iq_min_all = s->MS.i_q_setpoint;
    if (s->MS.i_q_setpoint > s->iq_max_all) s->iq_max_all = s->MS.i_q_setpoint;
    if (r.direction_inhibit_active) s->direction_inhibit_ticks++;

    if (s->tick > 2U * CTRL_HZ) {
        int32_t iq = s->MS.i_q_setpoint;
        if (iq < s->iq_min_run) s->iq_min_run = iq;
        if (iq > s->iq_max_run) s->iq_max_run = iq;
        s->iq_sum_run += iq;
        s->iq_sq_sum_run += (double)iq * iq;
        s->iq_samples_run++;
    }

    if (csv && (s->tick % 4U) == 0U) {
        const assist_pipeline_telemetry_t *mo = assist_pipeline_telemetry();
        fprintf(csv, "%u,%u,%u,%u,%u,%d,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%d\n",
            s->tick, ab, pas_direction_fwd_run(), s->MS.cadence,
            cadence_filter_get(), s->MS.i_q_setpoint,
            s->plant.iq_actual, s->plant.erps, s->plant.hall_age_ticks,
            ride_control_get_session_state(), assist_pipeline_reason_bits(),
            ts->load_centikg, ts->assist_delta_filtered_native,
            ts->assist_delta_run_native, mo->iq_request_before_limits);
    }
}

static uint32_t fuzz_state = 0xE7157A39U;

static uint32_t fuzz_u32(void)
{
    /* Deterministic xorshift32. A failing seed can be replayed exactly. */
    uint32_t x = fuzz_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    fuzz_state = x;
    return x;
}

static double fuzz_range(double lo, double hi)
{
    double u = (double)(fuzz_u32() & 0x00FFFFFFU) / 16777215.0;
    return lo + (hi - lo) * u;
}

static int run_fuzz(unsigned count)
{
    unsigned failures = 0U;
    uint32_t initial_seed = fuzz_state;
    for (unsigned i = 0U; i < count; i++) {
        uint32_t case_seed = fuzz_state;
        double rpm = fuzz_range(20.0, 120.0);
        double cadence_ripple = fuzz_range(0.0, 0.40);
        double mean_ckg = fuzz_range(900.0, 3500.0);
        double torque_ripple = mean_ckg * fuzz_range(0.0, 0.55);
        double breakaway_iq = fuzz_range(0.0, 20.0);
        bool bounce = (fuzz_u32() & 1U) != 0U;

        sim_t s;
        sim_init(&s, rpm, cadence_ripple, mean_ckg, torque_ripple, bounce,
            breakaway_iq, true);
#ifdef EVD_SIL_REAL_FOC
        plant_set_start_angle(&s.plant, fuzz_range(0.0, 1.0));
#endif
        uint32_t n = 2U * CTRL_HZ;
        for (uint32_t t = 0U; t < n; t++) sim_ctrl_tick(&s, NULL);

        bool ok = true;
        /* A sustained forward rider with well-above-threshold load must not be stuck behind a
         * hidden Hall gate. At 20 rpm four configured start transitions take ~125 ms, then the
         * virtual drivetrain still gets 250 ms to break away. This is intentionally generous:
         * the deterministic fixed scenarios above are much tighter. */
        if (!s.first_permission_tick || !s.first_iq_tick || !s.first_hall_tick) ok = false;
        if (s.first_permission_tick && s.first_hall_tick &&
            (s.first_hall_tick - s.first_permission_tick) > CTRL_HZ / 4U) ok = false;
        if (s.false_reverse_events != 0U || s.direction_inhibit_ticks != 0U) ok = false;
        if (s.iq_min_all < 0 || s.iq_max_all > PH_CURRENT_MAX) ok = false;
#ifdef EVD_SIL_REAL_FOC
        if (s.plant.max_angle_error_deg > 31.0) ok = false;
#endif

        if (!ok) {
            failures++;
            fprintf(stderr,
                "FUZZ FAIL case=%u seed=0x%08X rpm=%.2f cadRipple=%.3f meanCkg=%.1f "
                "torqueRipple=%.1f breakawayIq=%.2f bounce=%u perm=%u iq=%u hall=%u "
                "falseR=%u inhibit=%u iqRange=[%d,%d]"
#ifdef EVD_SIL_REAL_FOC
                " angleErrMax=%.2fdeg"
#endif
                "\n",
                i, case_seed, rpm, cadence_ripple, mean_ckg, torque_ripple,
                breakaway_iq, bounce ? 1U : 0U, s.first_permission_tick, s.first_iq_tick,
                s.first_hall_tick, s.false_reverse_events, s.direction_inhibit_ticks,
                s.iq_min_all, s.iq_max_all
#ifdef EVD_SIL_REAL_FOC
                , s.plant.max_angle_error_deg
#endif
                );
            if (failures >= 10U) break;
        }
    }
    printf("FUZZ deterministic seed=0x%08X cases=%u failures=%u\n",
        initial_seed, count, failures);
    return failures ? 1 : 0;
}


static int run_stop_restart_scenario(void)
{
    sim_t s;
    sim_init(&s, 60.0, 0.20, 1800.0, 400.0, false, 8.0, true);

    /* Establish a normal ACTIVE ride first. */
    for (uint32_t i = 0U; i < 2U * CTRL_HZ; i++) sim_ctrl_tick(&s, NULL);
    if (s.MS.i_q_setpoint <= 0 || s.first_permission_tick == 0U) {
        fprintf(stderr, "STOP_RESTART FAIL: did not establish ACTIVE ride\n");
        return 1;
    }

    const uint32_t stop_tick = s.tick;
    const int32_t iq_at_release = s.MS.i_q_setpoint;
    s.rider.active = false;
    uint32_t zero_tick = 0U;
    int32_t prev_iq = s.MS.i_q_setpoint;
    int32_t max_fall_step = 0;
    for (uint32_t i = 0U; i < 2U * CTRL_HZ; i++) {
        sim_ctrl_tick(&s, NULL);
        int32_t d = prev_iq - s.MS.i_q_setpoint;
        if (d > max_fall_step) max_fall_step = d;
        prev_iq = s.MS.i_q_setpoint;
        if (!zero_tick && s.MS.i_q_setpoint == 0) zero_tick = s.tick;
    }
    if (!zero_tick || s.MS.i_q_setpoint != 0) {
        fprintf(stderr, "STOP_RESTART FAIL: normal stop never reached Iq=0\n");
        return 1;
    }

    /* Restart from a true stopped/PAS-reset state. No stale Iq may survive. */
    const uint32_t restart_tick = s.tick;
    s.rider.active = true;
    uint32_t restart_permission = 0U;
    uint32_t restart_first_iq = 0U;
    int32_t first_positive_iq = 0;
    int32_t max_rise_step = 0;
    prev_iq = s.MS.i_q_setpoint;
    for (uint32_t i = 0U; i < 2U * CTRL_HZ; i++) {
        sim_ctrl_tick(&s, NULL);
        int32_t d = s.MS.i_q_setpoint - prev_iq;
        if (d > max_rise_step) max_rise_step = d;
        prev_iq = s.MS.i_q_setpoint;
        if (!restart_permission && assist_pipeline_pas_state() == AP2_PAS_FORWARD)
            restart_permission = s.tick;
        if (!restart_first_iq && s.MS.i_q_setpoint > 0) {
            restart_first_iq = s.tick;
            first_positive_iq = s.MS.i_q_setpoint;
        }
    }

    bool ok = restart_permission != 0U && restart_first_iq != 0U &&
              restart_first_iq >= restart_permission &&
              first_positive_iq > 0 && first_positive_iq < PH_CURRENT_MAX &&
              s.false_reverse_events == 0U && s.direction_inhibit_ticks == 0U;
#ifdef EVD_SIL_REAL_FOC
    /* A real electrical STOP test is only meaningful if the production QZERO lifecycle actually
     * armed. Otherwise this would silently regress to the ordinary zero-current PI while still
     * printing a green stop/restart result. */
    ok = ok && g_quiet_zero.entries == 1U &&
         g_quiet_zero.aborts <= 1U &&
         g_quiet_zero.state == (uint32_t)QZERO_INACTIVE;
#endif
    double stop_to_zero_ms = 1000.0 * (double)(zero_tick - stop_tick) / CTRL_HZ;
    double restart_permission_ms = restart_permission ?
        1000.0 * (double)(restart_permission - restart_tick) / CTRL_HZ : -1.0;
    double permission_to_iq_ms = (restart_permission && restart_first_iq) ?
        1000.0 * (double)(restart_first_iq - restart_permission) / CTRL_HZ : -1.0;
    printf("SCENARIO stop_restart       releaseIq=%d stop->Iq0=%.2fms maxFallStep=%d "
           "restartPermission=%.2fms permission->Iq=%.2fms firstRestartIq=%d maxRiseStep=%d"
#ifdef EVD_SIL_REAL_FOC
           " qzeroEntries=%u qzeroAborts=%u qzeroHandbacks=%u qzEntryErps=%d qzAbortErps=%d qzPeakIq=%d"
#endif
           " %s\n",
           iq_at_release, stop_to_zero_ms, max_fall_step, restart_permission_ms,
           permission_to_iq_ms, first_positive_iq, max_rise_step,
#ifdef EVD_SIL_REAL_FOC
           g_quiet_zero.entries, g_quiet_zero.aborts, g_quiet_zero.low_speed_exits,
           g_qz_entry_erps, g_qz_abort_erps, g_qz_peak_abs_iq,
#endif
           ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

static void run_scenario(const char *name, double rpm, double cadence_ripple_fraction,
                         double mean_ckg, double ripple_ckg, bool bounce,
                         double breakaway_iq, bool filtered_cadence, double seconds)
{
    sim_t s;
    sim_init(&s, rpm, cadence_ripple_fraction, mean_ckg, ripple_ckg, bounce,
        breakaway_iq, filtered_cadence);
    char path[256];
    snprintf(path, sizeof(path), ".build/sil/%s.csv", name);
    FILE *f = fopen(path, "w");
    if (!f) { perror(path); exit(2); }
    fprintf(f, "tick,ab,fwd_run,cadence_raw,cadence_filtered,iq_ref,iq_actual,erps,hall_age,session,debug,load_ckg,torque_fast,torque_run,iq_request\n");
    uint32_t n = (uint32_t)(seconds * CTRL_HZ);
    for (uint32_t i = 0; i < n; i++) sim_ctrl_tick(&s, f);
    fclose(f);

    double mean = s.iq_samples_run ? s.iq_sum_run / s.iq_samples_run : 0.0;
    double var = s.iq_samples_run ? s.iq_sq_sum_run / s.iq_samples_run - mean * mean : 0.0;
    if (var < 0.0) var = 0.0;
    double std = sqrt(var);
    double permission_ms = s.first_permission_tick ? 1000.0 * s.first_permission_tick / CTRL_HZ : -1.0;
    double iq_ms = s.first_iq_tick ? 1000.0 * s.first_iq_tick / CTRL_HZ : -1.0;
    double hall_ms = s.first_hall_tick ? 1000.0 * s.first_hall_tick / CTRL_HZ : -1.0;
    double perm_to_hall = (s.first_permission_tick && s.first_hall_tick) ?
        1000.0 * (s.first_hall_tick - s.first_permission_tick) / CTRL_HZ : -1.0;
    printf("SCENARIO %-18s permission=%7.2fms firstIq=%7.2fms firstHall=%7.2fms perm->Hall=%7.2fms falseR=%u inhibitTicks=%u steadyIqMean=%.1f std=%.1f pp=%d"
#ifdef EVD_SIL_REAL_FOC
           " angleErrMax=%.2fdeg"
#endif
           "\n",
        name, permission_ms, iq_ms, hall_ms, perm_to_hall,
        s.false_reverse_events, s.direction_inhibit_ticks, mean, std,
        s.iq_samples_run ? (s.iq_max_run - s.iq_min_run) : 0
#ifdef EVD_SIL_REAL_FOC
        , s.plant.max_angle_error_deg
#endif
        );
}

#ifdef EVD_SIL_REAL_FOC
typedef struct {
    uint32_t tick;
    uint32_t first_hall_tick;
    uint32_t limit_ticks;
    uint32_t stall_ticks;
    uint32_t samples;
    double rpm_sum;
    double rpm_sq_sum;
    double rpm_min;
    double rpm_max;
    int32_t iq_peak;
    uint16_t measured_erps_last;
    uint16_t target_erps_last;
    uint8_t final_state;
    uint16_t final_reason;
    int32_t final_iq;
} walk_sil_metrics_t;

static void walk_sil_init(MotorState_t *ms, plant_t *p, double breakaway_iq,
                          double start_angle_rev)
{
    motor_core_init(ms);
    ms->hall_angle_detect_flag = 1U;
    foc_current_feedback_reset(ms);
    ride_control_init(); /* owns/resets the one final Iq mailbox used by Walk BYPASS too */
    walk_motor_release();
    plant_init(p, breakaway_iq);
    plant_set_start_angle(p, start_angle_rev);
}

static void walk_sil_ctrl_tick(MotorState_t *ms, plant_t *p, uint32_t ctrl_tick,
                               uint16_t target_rpm, int32_t walk_iq_max,
                               uint16_t wheel_speed_x100, walk_sil_metrics_t *m)
{
    p->hall_edge_this_ctrl = false;
    walk_motor_input_t in = {
        .active = true,
        .brake = false,
        .fault = false,
        .wheel_speed_x100 = wheel_speed_x100,
        .max_wheel_speed_x100 = 700U,
        .motor_hall_ticks = p->last_capture_500k,
        .motor_erps_age_ticks = p->hall_age_ticks,
        .motor_iq_actual = ms->i_q,
        .motor_iq_reference = ms->i_q_setpoint,
        .target_chainring_rpm = target_rpm,
        .walk_iq_max = walk_iq_max
    };
    walk_motor_output_t out;
    int32_t iq = walk_motor_update(&in, &out);

    /* The same ONE limiter chain ride_control runs Walk through in production. */
    ap2_limits_input_t lim;
    ap2_limits_output_t lim_out;
    memset(&lim, 0, sizeof(lim));
    lim.iq_request = iq;
    lim.source = AP2_LIMIT_SOURCE_WALK;
    lim.battery_voltage_mv = 42000U;
    lim.u_abs = 1024;
    lim.cal_i = 95;
    lim.battery_current_max = 15000;
    lim.phase_current_max = (int32_t)PH_CURRENT_MAX;
    lim.voltage_raw = TEST_VOLTAGE_RAW;
    lim.voltage_min_raw = VOLTAGE_MIN;
    lim.controller_temperature_c = TEST_TEMP_C;
    lim.speed_x100 = wheel_speed_x100;
    lim.speed_limit_x100 = SPEEDLIMIT;
    lim.legal_enabled = true;
    ap2_limits_apply(&lim, &lim_out);
    iq = lim_out.final_iq;
    if (iq < 0) iq = 0;
    if (iq > 65535) iq = 65535;

    fast_iq_slew_publish(ride_control_final_iq_slew_mailbox(), iq,
                         FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE, iq);
    for (unsigned k = 0; k < INNER_PER_CTRL; k++) {
        plant_inner_tick(p, ms, iq, ctrl_tick);
    }
    if (p->hall_edge_this_ctrl) {
        p->hall_age_ticks = 0U;
        if (m && m->first_hall_tick == 0U) m->first_hall_tick = ctrl_tick;
    } else if (p->hall_age_ticks < 0xFFFFU) {
        p->hall_age_ticks++;
    }

    if (m) {
        if (out.state == (uint8_t)WA_STATE_LIMIT) m->limit_ticks++;
        if (out.state == (uint8_t)WA_STATE_STALL) m->stall_ticks++;
        if (ms->i_q_setpoint > m->iq_peak) m->iq_peak = ms->i_q_setpoint;
        m->measured_erps_last = out.measured_erps;
        m->target_erps_last = out.target_erps;
        m->final_state = out.state;
        m->final_reason = out.reason;
        m->final_iq = ms->i_q_setpoint;
        if (ctrl_tick > 3U * CTRL_HZ) {
            double rpm = p->erps * 0.75; /* M820: chainring rpm = electrical ERPS * 3/4 */
            if (m->samples == 0U) { m->rpm_min = rpm; m->rpm_max = rpm; }
            if (rpm < m->rpm_min) m->rpm_min = rpm;
            if (rpm > m->rpm_max) m->rpm_max = rpm;
            m->rpm_sum += rpm;
            m->rpm_sq_sum += rpm * rpm;
            m->samples++;
        }
    }
}

static int run_walk_foc_matrix(void)
{
    static const uint16_t rpms[] = {10U, 15U, 20U, 30U, 40U, 50U, 60U};
    static const double loads[] = {3.0, 10.0, 20.0};
    static const double starts[] = {0.00, 1.0/6.0, 2.0/6.0, 3.0/6.0, 4.0/6.0, 5.0/6.0};
    int failures = 0;
    int tracking_warnings = 0;
    int safe_stall_cases = 0;
    double worst_first_hall_ms = 0.0;
    double worst_mean_error_pct = 0.0;
    double worst_pp = 0.0;

    for (uint32_t ri = 0; ri < sizeof(rpms)/sizeof(rpms[0]); ri++) {
        for (uint32_t li = 0; li < sizeof(loads)/sizeof(loads[0]); li++) {
            for (uint32_t si = 0; si < sizeof(starts)/sizeof(starts[0]); si++) {
                MotorState_t ms;
                plant_t p;
                walk_sil_metrics_t m = {0};
                walk_sil_init(&ms, &p, loads[li], starts[si]);
                const uint32_t duration = 6U * CTRL_HZ;
                for (uint32_t t = 1U; t <= duration; t++) {
                    walk_sil_ctrl_tick(&ms, &p, t, rpms[ri], 105, 0U, &m);
                }
                double first_hall_ms = m.first_hall_tick ?
                    1000.0 * (double)m.first_hall_tick / CTRL_HZ : 1e9;
                double mean = m.samples ? m.rpm_sum / (double)m.samples : 0.0;
                double err_pct = rpms[ri] ? fabs(mean - (double)rpms[ri]) * 100.0 / rpms[ri] : 0.0;
                double pp = m.samples ? (m.rpm_max - m.rpm_min) : 0.0;
                if (first_hall_ms > worst_first_hall_ms && first_hall_ms < 1e8) worst_first_hall_ms = first_hall_ms;
                if (err_pct > worst_mean_error_pct) worst_mean_error_pct = err_pct;
                if (pp > worst_pp) worst_pp = pp;

                /* The virtual PMSM R/L/flux/inertia/load are test parameters, not measured M820
                 * values. Hard-gate only architecture and safety facts that must hold independently
                 * of that tuning. Exact rpm tracking remains an evidence metric below. A deliberately
                 * heavy virtual load may either keep moving or reach the production LIMIT/STALL
                 * safety latch; if it stalls, final Iq must be zero. */
                uint16_t expected_erps = (uint16_t)(((uint32_t)rpms[ri] * 4U + 1U) / 3U);
                bool stall_safe = m.stall_ticks == 0U ||
                    (m.final_state == (uint8_t)WA_STATE_STALL && m.final_iq == 0);
                bool ok = m.target_erps_last == expected_erps &&
                          m.first_hall_tick != 0U && first_hall_ms < 1500.0 &&
                          m.iq_peak <= 157 && p.max_angle_error_deg <= 61.0 &&
                          m.rpm_max <= 80.0 && stall_safe;
                if (m.stall_ticks != 0U && stall_safe) safe_stall_cases++;
                if (err_pct > 25.0 || pp > ((double)rpms[ri] * 1.5 + 5.0)) {
                    tracking_warnings++;
                }
                if (!ok) {
                    failures++;
                    fprintf(stderr,
                        "WALK FOC FAIL rpm=%u load=%.1f start=%.0fdeg hall=%.1fms "
                        "mean=%.2f pp=%.2f err=%.1f%% limit=%u stall=%u finalState=%u finalIq=%d "
                        "iqPeak=%d angle=%.2fdeg targetErps=%u measuredErps=%u reason=0x%04X\n",
                        rpms[ri], loads[li], starts[si]*360.0, first_hall_ms, mean, pp, err_pct,
                        m.limit_ticks, m.stall_ticks, m.final_state, m.final_iq, m.iq_peak,
                        p.max_angle_error_deg, m.target_erps_last, m.measured_erps_last,
                        m.final_reason);
                }
            }
        }
    }

    /* Values above the supported target range are never allowed to become hidden 70/80-rpm
     * commands. Production policy for corrupt/out-of-range input is the safe 30-rpm default. */
    {
        static const uint16_t invalid[] = {0U, 9U, 61U, 70U, 80U, 100U};
        for (uint32_t i = 0; i < sizeof(invalid)/sizeof(invalid[0]); i++) {
            MotorState_t ms;
            plant_t p;
            walk_sil_metrics_t m = {0};
            walk_sil_init(&ms, &p, 3.0, 0.0);
            walk_sil_ctrl_tick(&ms, &p, 1U, invalid[i], 105, 0U, &m);
            if (m.target_erps_last != 40U) {
                failures++;
                fprintf(stderr, "WALK RANGE FAIL input=%u targetErps=%u expected default 40\n",
                        invalid[i], m.target_erps_last);
            }
        }
    }

    printf("WALK FOC MATRIX 7 targets x 3 loads x 6 Hall starts = 126: failures=%d "
           "safeStalls=%d trackingWarnings=%d worstFirstHall=%.2fms worstMeanErr=%.1f%% "
           "worstSteadyPP=%.2frpm %s\n",
           failures, safe_stall_cases, tracking_warnings, worst_first_hall_ms,
           worst_mean_error_pct, worst_pp, failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}

static int run_hall_start_angle_sweep(void)
{
    int failures = 0;
    double worst_ms = 0.0;
    double worst_err = 0.0;
    for (int deg = 0; deg < 360; deg += 15) {
        for (int load = 0; load < 2; load++) {
            sim_t s;
            sim_init(&s, 40.0, 0.20, 1800.0, 400.0, false,
                     load ? 15.0 : 6.0, true);
            plant_set_start_angle(&s.plant, (double)deg / 360.0);
            for (uint32_t i = 0; i < CTRL_HZ; i++) sim_ctrl_tick(&s, NULL);
            double ms = (s.first_permission_tick && s.first_hall_tick) ?
                1000.0 * (double)(s.first_hall_tick - s.first_permission_tick) / CTRL_HZ : 1e9;
            if (ms > worst_ms && ms < 1e8) worst_ms = ms;
            if (s.plant.max_angle_error_deg > worst_err) worst_err = s.plant.max_angle_error_deg;
            if (!s.first_permission_tick || !s.first_hall_tick || ms > 100.0 ||
                s.plant.max_angle_error_deg > 31.0 || s.false_reverse_events != 0U)
                failures++;
        }
    }
    printf("HALL START SWEEP 24 angles x 2 loads: failures=%d worstPerm->Hall=%.2fms worstAngleErr=%.2fdeg %s\n",
           failures, worst_ms, worst_err, failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}
#endif

int main(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "--fuzz") == 0) {
        unsigned count = (argc >= 3) ? (unsigned)strtoul(argv[2], NULL, 10) : 1000U;
        if (argc >= 4) fuzz_state = (uint32_t)strtoul(argv[3], NULL, 0);
        return run_fuzz(count);
    }
    /* glibc marks system() warn_unused_result; the SIL build is -Werror, and every
       scenario below writes into this directory, so a failure must stop the run.
       Windows cmd.exe has no `mkdir -p`; use the equivalent cmd spell. */
#if defined(_WIN32)
    if (system("if not exist .build/sil mkdir .build/sil") != 0) {
#else
    if (system("mkdir -p .build/sil") != 0) {
#endif
        fprintf(stderr, "evist_sil: cannot create .build/sil\n");
        return 1;
    }
    run_scenario("clean_start", 40.0, 0.0, 1800.0, 0.0, false, 6.0, false, 4.0);
    run_scenario("loaded_start", 40.0, 0.0, 1800.0, 0.0, false, 15.0, false, 4.0);
    run_scenario("pas_bounce", 60.0, 0.0, 1800.0, 300.0, true, 6.0, false, 4.0);
    run_scenario("steady_ripple", 60.0, 0.0, 1800.0, 500.0, false, 6.0, false, 6.0);
    /* Same physical non-uniform crank, identical torque. Only the cadence signal consumed by
     * assist/dynamics changes. This isolates the current raw-cadence wiring from the proposed
     * filtered-control wiring. */
    run_scenario("cadence_raw", 60.0, 0.35, 1800.0, 0.0, false, 6.0, false, 8.0);
    run_scenario("cadence_filtered", 60.0, 0.35, 1800.0, 0.0, false, 6.0, true, 8.0);
    run_scenario("pedal20", 20.0, 0.30, 1800.0, 700.0, false, 6.0, true, 10.0);
    run_scenario("pedal40", 40.0, 0.30, 1800.0, 700.0, false, 6.0, true, 8.0);
    run_scenario("pedal60", 60.0, 0.30, 1800.0, 700.0, false, 6.0, true, 8.0);
    run_scenario("pedal80", 80.0, 0.30, 1800.0, 700.0, false, 6.0, true, 8.0);
#ifdef EVD_SIL_REAL_FOC
    if (run_hall_start_angle_sweep() != 0) return 1;
    if (run_walk_foc_matrix() != 0) return 1;
#endif
    if (run_stop_restart_scenario() != 0) return 1;
    return 0;
}
