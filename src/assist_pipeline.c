#include "assist_pipeline.h"
#include "ap2_math.h"
#include "assist_modes.h"
#include "config.h"
#include "tuning_config.h"
#include <string.h>
#if ASSIST_V3
#include "assist_v3.h"
#include "assist_v3_config.h"
#endif
#if ASSIST_V3_CPU_PROBE
#include "gd32f30x.h"   /* DWT cycle counter (target DIAG build only, inc/config.h) */
#endif

/*
 * ASSIST_V3 (compile-time, inc/config.h). 1: the V3 behaviour stage (src/assist_v3.c) is computed
 * every tick beside the G53 chain; 0: no V3 code is referenced and this file is the baseline
 * pipeline.
 *
 * MILESTONE C - ACTIVE ENGINE (ARCHITECTURE_V3 2.1, 2.2, 7.3). Both demands are computed every
 * tick in both engines; `engine_active` (assist_v3_config, truthful readback) selects the one that
 * is published:
 *   G5300  the published request is the G53 one, exactly as at baseline: the V3 demand, the
 *          backstop and the standstill predicate are computed but read by nothing that publishes
 *          (TEST_MATRIX G-EQ: byte-identical with ASSIST_V3 compiled out and in).
 *   V3     req = (y * g1) >> 12 with this tick's g1 -> ap2_limits -> iq_ceiling (unchanged) ->
 *          pipeline-owned stop/reverse BACKSTOP ceiling (min, never a demand) -> native_cut /
 *          assist-off / R1 bumpless veto release (against the post-limit V3 request) -> the
 *          STANDSTILL predicate (FORCE_ZERO) in place of the stock BDE8 hard zero -> zero/slew
 *          policy (unchanged). V3 writes no slew mode, ceiling, zero policy or safety state.
 * ENGINE LATCH: engine_requested becomes engine_active only on a tick where the published request,
 * V3 y and the G53 pre-limit request are all 0 and no veto (native_cut, assist off, backstop,
 * standstill) is active; the switch tick sets pulled_down so R1 governs any climb (R1-#10, D-020).
 * The backstop is applied after ap2_limits (a min commutes with the limiter clamps), so the limiter
 * state sees the true V3 request and the backstop counts as a veto for R1, like native_cut.
 */

/* One normal demand owner: G53 boundaries/PAS/FSM/D7EC/E1E8/BDE8.
 * M820 keeps hardware limits, unconditional vetoes and the sole final-Iq owner. */
#define AP2_SAFETY_RELEASE_MS 200U
#define AP2_FOC_TICKS_PER_MS 16U
#define AP2_CEILING_FALL_MS 120U
#define AP2_CEILING_RISE_MS 400U
#define AP2_HUMAN_POWER_NUM 1694U
#define AP2_HUMAN_POWER_DEN 1000U
#define AP2_HUMAN_POWER_REF_CRANK_MM 165U

typedef struct {
    int32_t ceiling;
    bool ceiling_valid;
    /* Previous published request for bumpless release from the native safety veto. */
    int32_t last_final_iq;
    /* Bumpless veto release: true while the published request was held below the G53 request by
     * an M820 veto/gate on the previous tick, and the remainder of the rise-rate budget. */
    bool pulled_down;
    uint32_t rise_acc;
    assist_pipeline_telemetry_t tlm;
    g53_port_output_t g53;
#if ASSIST_V3
    /* The V3 demand (Iq, pre-g1, pre-limits) of this tick and its g1-scaled request. */
    int32_t v3_iq_demand;
    int32_t v3_request;
    bool v3_skipped;        /* V3 stage not computed last tick (G5300 publishing, V3 not requested) */
    motion_est_t motion_est;
    /* Pipeline-owned stop/reverse backstop (ARCHITECTURE_V3 7.3, D-008): own state, independent
     * of assist_v3.c, a ceiling on the published request only. */
    struct {
        uint8_t state;            /* assist_pipeline_backstop_t */
        bool fwd_seen;            /* a forward crank step since the ceiling closed */
        bool steps_primed;
        uint8_t rev_run;          /* consecutive reverse crank steps */
        int32_t last_steps;
        int32_t ceiling;          /* Iq, meaningful while state != OPEN */
        uint32_t hold_ticks;      /* control ticks in HOLD */
        uint32_t start_distance_mm;
        uint32_t rate_acc;        /* BDE8-rate remainder, Iq * 40000 */
        int32_t decay_from;       /* ceiling when the post-hold decay began */
        uint32_t decay_acc;       /* post-hold decay remainder, Iq * (300 ms * 4 ticks) */
    } bs;
    assist_pipeline_v3_status_t v3s;
#endif
} ap2_pipeline_ctx_t;
static ap2_pipeline_ctx_t ctx;

void assist_pipeline_reset(void)
{
    const uint16_t seq=ctx.tlm.engage_seq;
#if ASSIST_V3
    /* The D-039 CPU statistics describe the image since power-on, not one owner period. */
    const uint32_t cpu_max=ctx.v3s.cpu_max_cycles, cpu_over=ctx.v3s.cpu_over_budget;
#endif
    memset(&ctx,0,sizeof(ctx));
    ctx.tlm.engage_seq=seq;
#if ASSIST_V3
    ctx.v3s.cpu_max_cycles=cpu_max;
    ctx.v3s.cpu_over_budget=cpu_over;
    ctx.v3s.engine_v3_active=assist_v3_config_engine_active();
#endif
    g53_port_reset();
#if ASSIST_V3
    /* An owner change resets V3 with the chain (ARCHITECTURE_V3 2.2); the learned template is
     * kept, it describes the rider, not the ride. */
    assist_v3_reset();
#endif
    /* Shared battery protections survive rider/Walk/service owner changes. */
}
void assist_pipeline_init(void)
{
    assist_pipeline_reset();
    ap2_limits_reset();
#if ASSIST_V3
    assist_v3_power_on();   /* prior template, trajectory at 0 */
#endif
}
const assist_pipeline_telemetry_t *assist_pipeline_telemetry(void) { return &ctx.tlm; }
const g53_port_output_t *assist_pipeline_g53(void) { return &ctx.g53; }
ap2_pas_state_t assist_pipeline_pas_state(void) { return (ap2_pas_state_t)ctx.tlm.pas_state; }
/* ADR-013: on the PEDAL path the battery current is limited by G53 PI #1 (g1 < 1.0), the Walk
 * path still by the ap2 battery stage; either one is "battery limited". */
bool assist_pipeline_battery_limited(void) { return ap2_limits_battery_active() || ctx.tlm.battery_limited; }
uint8_t assist_pipeline_state_byte(void) { return ctx.tlm.pas_state & 0x0fu; }
uint8_t assist_pipeline_reason_bits(void)
{
    const assist_pipeline_telemetry_t *t=&ctx.tlm;
    uint8_t why=0;
    if(!t->assist_permitted) why|=AP2_WHY_NOT_PERMITTED;
    if(t->block_positive) why|=AP2_WHY_BLOCKED;
    if(t->assist_permitted && t->iq_request_before_limits==0) why|=AP2_WHY_NO_DEMAND;
    if(t->power_limited || t->battery_limited || t->phase_limited ||
       t->voltage_limited || t->thermal_limited || t->speed_limited) why|=AP2_WHY_LIMITED;
    if(t->limiter_zeroed) why|=AP2_WHY_ZEROED_BY_LIMIT;
    if(t->release_active) why|=AP2_WHY_RELEASE;
    return why; /* START and AUTO are deprecated zero; never control inputs. */
}

/* Motor input power at the measured duty - the inverse of the power ceiling conversion in
 * ap2_limits.c, so the number reported and the number limited are the same physics. */
static uint16_t motor_power_w(int32_t iq, uint32_t battery_voltage_mv, int32_t u_abs,
	int32_t cal_i)
{
	int64_t p;

	if (iq <= 0 || u_abs <= 0 || cal_i <= 0 || battery_voltage_mv == 0U) {
		return 0U;
	}
	p = ((int64_t)battery_voltage_mv * (int64_t)iq * (int64_t)cal_i * (int64_t)u_abs) /
		((int64_t)AP2_U_ABS_FULL_SCALE * 1000000);
	if (p < 0) {
		p = 0;
	}
	if (p > 65535) {
		p = 65535;
	}
	return (uint16_t)p;
}

static uint16_t rider_power_w(uint16_t load_centikg, uint8_t cadence_rpm)
{
	uint32_t crank_mm = tuning_config_crank_length_mm();
	uint64_t mw;

	if (load_centikg == 0U || cadence_rpm == 0U) {
		return 0U;
	}
	if (crank_mm == 0U) {
		crank_mm = AP2_HUMAN_POWER_REF_CRANK_MM;
	}
	mw = ((uint64_t)load_centikg * (uint64_t)cadence_rpm * AP2_HUMAN_POWER_NUM) /
		AP2_HUMAN_POWER_DEN;
	mw = (mw * crank_mm) / AP2_HUMAN_POWER_REF_CRANK_MM;
	mw /= 1000U;
	return (mw > 65535U) ? 65535U : (uint16_t)mw;
}


#if ASSIST_V3
/*
 * ASSIST-V3 STAGE (ARCHITECTURE_V3 2, 3.1). Runs after g53_port_update() so every accessor reads
 * this tick's chain state, in both engines. It reads the chain only through the read-only V3
 * accessors and the configuration getters and writes nothing but V3's own state; its result is
 * kept in ctx.v3_iq_demand, which the pipeline publishes only while V3 is the active engine.
 */
static void v3_stage(const assist_pipeline_input_t *in, uint32_t used_ticks, bool assist_off)
{
    assist_v3_input_t v3;
    memset(&v3,0,sizeof(v3));
    v3.elapsed_ticks=used_ticks;
    v3.now_tick=in->control_tick;
    v3.load_ctrl=in->torque_load_ctrl;
    v3.torque_valid=in->torque_sensor_valid;
    v3.crank_steps=in->crank_steps;
    v3.crank_step_tick=in->crank_step_tick;
    v3.pas_glitch=in->pas_glitch;
    v3.cadence_rpm=g53_port_pas_cadence();
    v3.lut_cadence=(uint16_t)ctx.g53.trace.d7ec_m50;
    v3.speed_native=(uint16_t)ctx.g53.trace.d7ec_speed;
    v3.g53_true_stop=g53_port_pas_true_stop();
    v3.g53_reverse=ctx.g53.trace.pas_direction<0;
    v3.direction_inhibit=in->direction_inhibit;
    v3.inhibit_is_reverse=in->inhibit_is_reverse;
    v3.real_stop=in->real_stop;
    v3.speed_x100=in->speed_x100;
    v3.wheel_valid=in->wheel_valid;
    v3.wheel_pulse_tick=in->wheel_pulse_tick;
    v3.motor_erps=in->motor_erps;
    v3.iq_measured=in->iq_measured;
    v3.last_published_iq=ctx.last_final_iq;
    v3.phase_current_max=in->phase_current_max;
    v3.g1_q12=(uint16_t)(ctx.g53.trace.g1<0 ? 0 : ctx.g53.trace.g1);
    /* Same level the chain gets: assist off is level 0 (ratio 0). */
    v3.level=assist_off ? 0u : in->assist_level_index;
    /* Milestone C release source through the resolver (REVIEW-T #5, #14). */
    v3.response_pct=assist_v3_effective_release_pct(in->assist_level_index);
    {
        const assist_v3_effective_t *eff=assist_v3_effective(in->assist_level_index);
        if(eff){
            v3.carry_strength_pct=(uint8_t)eff->value[16];
            v3.carry_time_ms=eff->value[17];
            v3.carry_distance_dm=eff->value[18];
        }
    }
    v3.speed_est_x100=ctx.motion_est.out.speed_x100;
    v3.distance_est_mm=ctx.motion_est.out.distance_mm;
    v3.rel_accel_permille_s=ctx.motion_est.out.rel_accel_permille_s;
    v3.motion_quality=ctx.motion_est.out.quality;
    v3.native_cut=in->safety_cut || !in->torque_sensor_valid || !in->pas_sensor_valid;
    v3.brake=in->brake;
    v3.eb74_zero=g53_port_eb74_zero();
    v3.eb74_armed=g53_port_eb74_armed();
    v3.engine_active=assist_v3_config_engine_active();
    v3.engine_requested=assist_v3_config_engine_requested();
    /* V3 reads only the sanitised motion copy (3.3): an invalid or stale sample is all-zero. */
    assist_motion_sanitize(&in->motion,&v3.motion);
#if ASSIST_V3_CPU_PROBE
    /* D-039: cycles of the whole V3 stage (intent + static map + trajectory) per call. */
    const uint32_t c0=DWT->CYCCNT;
#endif
    ctx.v3_iq_demand=assist_v3_update(&v3);
#if ASSIST_V3_CPU_PROBE
    {
        const uint32_t dc=DWT->CYCCNT-c0;
        ctx.v3s.cpu_last_cycles=dc;
        if(dc>ctx.v3s.cpu_max_cycles) ctx.v3s.cpu_max_cycles=dc;
        if(dc>ASSIST_V3_CPU_BUDGET_CYCLES && ctx.v3s.cpu_over_budget<UINT32_MAX) ++ctx.v3s.cpu_over_budget;
    }
#endif
}

/* BDE8 rate (+-50 M2AA per 1 ms logical tick on the 6500 full scale = P*50/10000 Iq/ms, 3.5 Iq/ms
 * at P = 700) over `ticks` control ticks, as whole Iq with the remainder carried (R1's rule). */
static int32_t v3_bde8_step(uint32_t *acc, int32_t p, uint32_t ticks)
{
    *acc+=(uint32_t)p*50u*ticks;
    const uint32_t step=*acc/(10000u*AP2_TICKS_PER_MS);
    *acc-=step*(10000u*AP2_TICKS_PER_MS);
    return (int32_t)step;
}

/*
 * PIPELINE BACKSTOP (ARCHITECTURE_V3 7.3, D-008, R1-#4). A CEILING on the published request in V3
 * mode, independent of every V3 internal (it reads native PAS facts, the G53 PAS true-stop
 * accessor, the crank step count and the published value only):
 *   reverse inhibit               the ceiling decays from the published value at the BDE8 rate
 *                                 (3.5 Iq/ms, full scale in 130 ms), as G5300 reverse does;
 *   crank stopped (G53 true-stop  the ceiling holds (never above the published value) for at most
 *   or native real_stop)          T_STOP_HARD = 1500 ms, then decays to 0 within 300 ms;
 *   re-open                       only after forward crank steps resume with no reverse and no
 *                                 stop flag, at no more than the BDE8 rate from the published
 *                                 value, until the ceiling no longer binds;
 *   standstill                    the separate predicate below (FORCE_ZERO).
 * It runs in both engines (its state is a latch veto) and binds only in V3 mode.
 */
#define V3_BS_T_STOP_HARD_TICKS (ASSIST_V3_BACKSTOP_HARD_MAX_MS*AP2_TICKS_PER_MS)
#define V3_BS_D_STOP_HARD_MM ASSIST_V3_BACKSTOP_HARD_MAX_MM
#define V3_BS_STOP_DECAY_TICKS  (300u*AP2_TICKS_PER_MS)
static void v3_backstop_close(uint8_t state)
{
    if(ctx.bs.state==ASSIST_PIPELINE_BS_OPEN || ctx.bs.ceiling>ctx.last_final_iq)
        ctx.bs.ceiling=ctx.last_final_iq;    /* from the published value, never above it */
    ctx.bs.state=state;
    ctx.bs.fwd_seen=false;
    ctx.bs.hold_ticks=0;
    ctx.bs.start_distance_mm=ctx.motion_est.out.distance_mm;
    ctx.bs.rate_acc=0;
    ctx.bs.decay_acc=0;
}
static void v3_backstop_step(const assist_pipeline_input_t *in, uint32_t ticks, int32_t request)
{
    const int32_t p=in->phase_current_max>0 ? in->phase_current_max : 0;
    /* REVERSE = a counted reverse crank step or the native reverse inhibit. An INVALID PAS
     * transition also raises direction_inhibit (and flips the G53 PAS direction for a few ms);
     * closing on it turned every PAS glitch into a 3.5 Iq/ms assist dip that baseline G53 never
     * shows (pas_glitch matrix, rework 2026-10-07). */
    int32_t dsteps=0;
    if(ctx.bs.steps_primed) dsteps=(int32_t)((uint32_t)in->crank_steps-(uint32_t)ctx.bs.last_steps);
    if(dsteps>0) ctx.bs.rev_run=0;
    else if(dsteps<0) ctx.bs.rev_run=(uint8_t)(ctx.bs.rev_run-dsteps>255 ? 255 : ctx.bs.rev_run-dsteps);
    /* two or more reverse steps in a row: one reverse step between forward steps is PAS jitter */
    const bool reverse=(dsteps<0 && ctx.bs.rev_run>=2u) || (in->direction_inhibit && in->inhibit_is_reverse);
    const bool stop=g53_port_pas_true_stop() || in->real_stop;
    const bool fwd_step=dsteps>0;
    ctx.bs.steps_primed=true;
    ctx.bs.last_steps=in->crank_steps;
    if(ctx.bs.state==ASSIST_PIPELINE_BS_OPEN || ctx.bs.state==ASSIST_PIPELINE_BS_REOPEN) {
        if(reverse) v3_backstop_close(ASSIST_PIPELINE_BS_REVERSE);
        else if(stop) v3_backstop_close(ASSIST_PIPELINE_BS_HOLD);
    } else if(reverse && ctx.bs.state!=ASSIST_PIPELINE_BS_REVERSE) {
        v3_backstop_close(ASSIST_PIPELINE_BS_REVERSE);
    }
    if(ctx.bs.state!=ASSIST_PIPELINE_BS_OPEN && ctx.bs.state!=ASSIST_PIPELINE_BS_REOPEN) {
        if(fwd_step && !reverse) ctx.bs.fwd_seen=true;
        if(ctx.bs.fwd_seen && !reverse && !stop) {
            ctx.bs.state=ASSIST_PIPELINE_BS_REOPEN;
            ctx.bs.rate_acc=0;
        }
    }
    switch(ctx.bs.state) {
    case ASSIST_PIPELINE_BS_REVERSE: {
        int32_t c=ctx.bs.ceiling<ctx.last_final_iq ? ctx.bs.ceiling : ctx.last_final_iq;
        c-=v3_bde8_step(&ctx.bs.rate_acc,p,ticks);
        ctx.bs.ceiling=c>0 ? c : 0;
        break; }
    case ASSIST_PIPELINE_BS_HOLD:
        if(ctx.bs.ceiling>ctx.last_final_iq) ctx.bs.ceiling=ctx.last_final_iq;
        ctx.bs.hold_ticks+=ticks;
        if(ctx.bs.hold_ticks<V3_BS_T_STOP_HARD_TICKS &&
           (uint32_t)(ctx.motion_est.out.distance_mm-ctx.bs.start_distance_mm)<V3_BS_D_STOP_HARD_MM) break;
        ctx.bs.state=ASSIST_PIPELINE_BS_DECAY;
        ctx.bs.decay_from=ctx.bs.ceiling;
        ctx.bs.decay_acc=0;
        /* the part of this call past the hold; a hold ended by the DISTANCE bound starts its 300 ms
         * decay now (REVIEW 2 #4: the subtraction wrapped and dropped the ceiling to 0 at once) */
        ticks=ctx.bs.hold_ticks>V3_BS_T_STOP_HARD_TICKS ? ctx.bs.hold_ticks-V3_BS_T_STOP_HARD_TICKS : 0u;
        /* fall through */
    case ASSIST_PIPELINE_BS_DECAY: {
        /* linear: decay_from in exactly 300 ms (whole Iq per tick, remainder carried) */
        int32_t c=ctx.bs.ceiling<ctx.last_final_iq ? ctx.bs.ceiling : ctx.last_final_iq;
        ctx.bs.decay_acc+=(uint32_t)ctx.bs.decay_from*ticks;
        const uint32_t dec=ctx.bs.decay_acc/V3_BS_STOP_DECAY_TICKS;
        ctx.bs.decay_acc-=dec*V3_BS_STOP_DECAY_TICKS;
        c-=(int32_t)(dec>(uint32_t)INT32_MAX ? (uint32_t)INT32_MAX : dec);
        ctx.bs.ceiling=c>0 ? c : 0;
        break; }
    case ASSIST_PIPELINE_BS_REOPEN: {
        /* from the PUBLISHED value at no more than the BDE8 rise rate (R1-#4) */
        const int32_t c=ctx.last_final_iq+v3_bde8_step(&ctx.bs.rate_acc,p,ticks);
        ctx.bs.ceiling=c;
        if(c>=request) ctx.bs.state=ASSIST_PIPELINE_BS_OPEN;   /* no longer binds */
        break; }
    default:
        break;
    }
}

/* STANDSTILL predicate (ARCHITECTURE_V3 2.1, D-019 rev, R1-#7/N2), replacing the stock BDE8 hard
 * zero in V3 mode: speed_native <= 0 AND (direction inhibit OR (crank stopped AND V3 stop target
 * == 0)), crank stopped = G53 PAS true-stop OR native real_stop. speed_native is the value the
 * chain gets (speed_x100 / 10, 0.1 km/h). The V3 term can only make zeroing earlier; if V3
 * misbehaves the bound at standstill is the independent backstop (<= 1.8 s). */
static bool v3_standstill_zero(const assist_pipeline_input_t *in)
{
    const bool speed_zero=ctx.motion_est.out.quality ? ctx.motion_est.out.speed_x100<10u :
        (in->speed_x100/10u)==0u;
    const bool crank_stopped=g53_port_pas_true_stop() || in->real_stop;
    return speed_zero && (in->direction_inhibit || (crank_stopped && assist_v3_stop_target_zero()));
}

const assist_pipeline_v3_status_t *assist_pipeline_v3_status(void) { return &ctx.v3s; }
#endif

/* G5300 fast slew step: 0.625*P Q8 per 4 kHz tick, half up, never 0 for P>0 (TQ-06-G2 I2). */
static uint16_t fast_slew_step(int32_t phase_current_max)
{
    uint32_t step=((uint32_t)phase_current_max*5u+4u)/8u;
    if(step==0u) step=1u;
    if(step>UINT16_MAX) step=UINT16_MAX;
    return (uint16_t)step;
}

void assist_pipeline_update(const assist_pipeline_input_t *in,assist_pipeline_command_t *cmd)
{
    if(!cmd) return;
    memset(cmd,0,sizeof(*cmd));
    if(!in) { assist_pipeline_reset(); cmd->slew_mode=FIS_MODE_FORCE_ZERO; return; }
    const uint32_t used_ticks=in->elapsed_ticks ? in->elapsed_ticks : 1u;
    const assist_level_config_t *level=assist_modes_get_default_level(in->assist_level_index);
    const bool assist_off=in->assist_level_index==0 || assist_modes_level_disables_assist(level);
    const bool native_cut=in->safety_cut || !in->torque_sensor_valid || !in->pas_sensor_valid;
    const bool observed_veto=in->direction_inhibit || native_cut || in->real_stop;
    const g53_port_input_t port_in={
        .raw_pa6_adc=in->raw_pa6_adc, .load_ctrl=in->torque_load_ctrl,
        .pas_ab=in->pas_ab, .assist_level=assist_off ? 0 : in->assist_level_index,
        .speed_x100=in->speed_x100, .elapsed_ticks=used_ticks,
        .phase_current_max=in->phase_current_max,
        .battery_feedback_centiamp=in->battery_current_limiter_centiamp,
        .battery_limit_centiamp=in->battery_current_max/10,
        .battery_soc_derate_q12=in->battery_soc_derate_q12,
        .torque_sensor_valid=in->torque_sensor_valid,
        .direction_inhibit=in->direction_inhibit, .real_stop=in->real_stop,
        .safety_cut=in->safety_cut
    };
    g53_port_update(&port_in,&ctx.g53);
    ap2_limits_input_t lim_in={0};
    ap2_limits_output_t lim;
#if ASSIST_V3
    /* The engine that publishes THIS tick (truthful readback, changed only by the latch below). */
    const bool v3_mode=assist_v3_config_engine_active();
    (void)motion_est_update(&ctx.motion_est,in->control_tick,used_ticks,in->wheel_valid,
        in->wheel_pulse_tick,in->motor_erps,in->cadence_rpm,in->iq_measured);
    /* REVIEW 2 #1: the V3 stage costs foreground time (static map x5, resolver). When G5300 publishes
     * and V3 is not requested, it is skipped, so a G5300 image behaves (and times) like the baseline
     * on the bike; brake latency in the foreground is not lengthened. Shadow-telemetry builds keep
     * computing it. When it starts again V3 is reset (stale intent ring), and y is 0 by the latch rule. */
    {
        const bool v3_needed=v3_mode || assist_v3_config_engine_requested() || ASSIST_V3_SHADOW_TELEMETRY;
        if(v3_needed) {
            if(ctx.v3_skipped) { assist_v3_reset(); ctx.v3_skipped=false; }
            v3_stage(in,used_ticks,assist_off);   /* shadow in G5300 (requested V3), published in V3 */
        } else { ctx.v3_skipped=true; ctx.v3_iq_demand=0; }
    }
    {
        /* x g1 (battery envelope, as BDE8 applies it), this tick's g1, Q12, clamped to 0..1.0 */
        int32_t g1=ctx.g53.trace.g1;
        if(g1<0) g1=0;
        if(g1>(int32_t)G53_G1_Q12_ONE) g1=(int32_t)G53_G1_Q12_ONE;
        ctx.v3_request=(int32_t)(((int64_t)ctx.v3_iq_demand*g1)>>12);
    }
    const int32_t pre_limits=v3_mode ? ctx.v3_request : ctx.g53.iq_request_pre_limits;
#else
    const int32_t pre_limits=ctx.g53.iq_request_pre_limits;
#endif
    lim_in.iq_request=pre_limits;
    lim_in.source=AP2_LIMIT_SOURCE_PEDAL;
    lim_in.max_power_w=0;
    lim_in.level_iq_limit=AP2_LIMITS_NO_LEVEL_CEILING;
    lim_in.phase_current_max=in->phase_current_max;
    lim_in.battery_voltage_mv=in->battery_voltage_mv;
    lim_in.u_abs=in->u_abs;
    lim_in.cal_i=in->cal_i;
    lim_in.battery_current_ma=in->battery_current_ma;
    lim_in.battery_current_max=in->battery_current_max;
    lim_in.battery_stage_owned_upstream=true; /* G53 PI #1 owns it (ADR-013) */
    lim_in.voltage_raw=in->voltage_raw;
    lim_in.voltage_min_raw=in->voltage_min_raw;
    lim_in.controller_temperature_c=in->controller_temperature_c;
    lim_in.speed_x100=in->speed_x100;
    lim_in.speed_limit_x100=in->speed_limit_x100;
    lim_in.legal_enabled=in->legal_enabled;
    lim_in.offroad=in->offroad;
    ap2_limits_apply(&lim_in,&lim);
    /* Preserve the independent rate-limited HARD protection ceiling. It never
     * follows demand or a native veto, so safety retirement retains its owner. */
    if(!ctx.ceiling_valid) { ctx.ceiling=lim.iq_ceiling; ctx.ceiling_valid=true; }
    else ctx.ceiling=ap2_slew_step(ctx.ceiling,lim.iq_ceiling,
        in->phase_current_max>0 ? in->phase_current_max : 1,
        AP2_CEILING_RISE_MS,AP2_CEILING_FALL_MS,used_ticks);
    cmd->iq_ceiling=ctx.ceiling;
    cmd->final_iq_request=lim.final_iq;
    /* The post-limit request of the ACTIVE engine: the R1 reference (ARCHITECTURE_V3 2.1 M2). */
    const int32_t g53_request=lim.final_iq;
    if(assist_off) cmd->final_iq_request=0;
#if ASSIST_V3
    /* Backstop and standstill are computed in both engines (latch vetoes), applied in V3 only. */
    v3_backstop_step(in,used_ticks,lim.final_iq);
    const bool standstill_zero=v3_standstill_zero(in);
    if(v3_mode && ctx.bs.state!=ASSIST_PIPELINE_BS_OPEN && cmd->final_iq_request>ctx.bs.ceiling)
        cmd->final_iq_request=ctx.bs.ceiling;   /* a ceiling (min), never a demand */
#endif
    bool quiet=false;
    /*
     * OWNER-DEC-2026-10-06-G5300-ONLY: the PEDAL request is the G53 chain output after
     * ap2_limits. A reverse step clears D7EC's drive permission, and BDE8 ramps its current
     * request down at 50 native units/ms while moving. The G53 PAS true-stop timer handles
     * a stationary crank; BDE8 stock_hard_zero handles standstill. Native brake/fault/invalid
     * sensor safety remains the M820 SAFETY release. No PAS direction, liveness or forward
     * observation adds a second veto to the published request.
     */
    if(native_cut) {
        cmd->final_iq_request=0; cmd->slew_mode=FIS_MODE_SAFETY;
        cmd->release_ticks_16k=AP2_SAFETY_RELEASE_MS*AP2_FOC_TICKS_PER_MS;
        quiet=true;
    } else {
        /*
         * BUMPLESS VETO RELEASE (TASK-EVD-TQ-06-G2 R1). The M820 vetoes above zero only the
         * PUBLISHED request; the G53 chain keeps its own demand while the rider keeps loading
         * the pedal. When the veto clears, the request may climb back toward the chain only at
         * the G5300 BDE8 rate (+50 M2AA per 1 ms logical tick = phase_current_max*50/10000 Iq per
         * ms), as G5300's Q5C would from the level actually being produced - never as a step.
         * Decreases are never delayed, and a request that was not pulled down follows G53 as is.
         */
        if(ctx.pulled_down && cmd->final_iq_request>ctx.last_final_iq && in->phase_current_max>0) {
            ctx.rise_acc+=(uint32_t)in->phase_current_max*50u*used_ticks;
            const uint32_t step=ctx.rise_acc/(10000u*AP2_TICKS_PER_MS);
            ctx.rise_acc-=step*(10000u*AP2_TICKS_PER_MS);
            if((int64_t)cmd->final_iq_request>(int64_t)ctx.last_final_iq+(int64_t)step)
                cmd->final_iq_request=(int32_t)(ctx.last_final_iq+(int32_t)step);
        } else ctx.rise_acc=0;
        /*
         * G5300 FAST CURRENT-REFERENCE SLEW (TASK-EVD-TQ-06-G2 I2, stock 0x0801A26C): the final
         * reference moves by at most +-10 Q14 per call, 16 calls per 1 ms = +-160 Q14/ms, where Q14
         * 16384 is phase_current_max. That is 160*P/16384 Iq/ms = 0.625*P per 4 kHz tick in Q8
         * (438 at P=700), rise and fall alike. The G53 BDE8 ramps (R1 3.5 Iq/ms engage, release
         * ~3 Iq/ms) are slower, so ordinary riding is untouched; only a faster step of the request
         * (limiter drop, g1) is now slewed instead of stepped. fast_iq_slew stays the single owner.
         * The stock hard-zero events (BDE8 drive permission falls with m2aa 0: standstill release)
         * reset the slew state in the stock (mode 0/3) and so stay FORCE_ZERO here. Assist off
         * keeps its same-update exact zero. The protection ceiling still binds immediately.
         */
        const bool stock_hard_zero=!ctx.g53.normal_permission && ctx.g53.m2aa_native==0;
#if ASSIST_V3
        /* V3 mode: the standstill predicate replaces the stock BDE8 hard zero (2.1 M3). */
        const bool hard_zero=v3_mode ? standstill_zero : stock_hard_zero;
#else
        const bool hard_zero=stock_hard_zero;
#endif
        if(assist_off || in->phase_current_max<=0) cmd->slew_mode=FIS_MODE_BYPASS;
        else if(hard_zero) { cmd->final_iq_request=0; cmd->slew_mode=FIS_MODE_FORCE_ZERO; }
        else {
            cmd->step_mag_8=fast_slew_step(in->phase_current_max);
            cmd->slew_mode=cmd->final_iq_request>ctx.last_final_iq ? FIS_MODE_RISE : FIS_MODE_FALL;
        }
        /* Zero policy still observes forward_valid; QZERO is compiled out in variant B. */
        quiet=cmd->final_iq_request==0 && (assist_off || !in->forward_valid);
    }
    ctx.pulled_down=cmd->final_iq_request<g53_request;
    cmd->zero_policy=quiet && !in->service_cut ? FIS_ZERO_POLICY_QUIET : FIS_ZERO_POLICY_NONE;
    ctx.last_final_iq=cmd->final_iq_request;
#if ASSIST_V3
    {
        /* ENGINE LATCH (ARCHITECTURE_V3 2.2, R1-#10, D-020): published request, V3 y and the G53
         * pre-limit request all 0, no veto. The new engine publishes from the next tick, with
         * pulled_down set so R1 governs any climb. No standstill CLAUSE: standstill is a veto. */
        const bool requested=assist_v3_config_engine_requested();
        const bool vetoed=native_cut || assist_off || ctx.bs.state!=ASSIST_PIPELINE_BS_OPEN ||
                          standstill_zero;
        ctx.v3s.switched=false;
        if(requested!=v3_mode && cmd->final_iq_request==0 && !assist_v3_engaged() &&
           ctx.g53.iq_request_pre_limits==0 && !vetoed) {
            assist_v3_config_set_engine_active(requested);
            ctx.pulled_down=true;
            ctx.rise_acc=0;
            ctx.v3s.switched=true;
        }
        ctx.v3s.engine_v3_active=assist_v3_config_engine_active();
        ctx.v3s.engine_v3_requested=requested;
        ctx.v3s.backstop_state=ctx.bs.state;
        ctx.v3s.backstop_iq=ctx.bs.state==ASSIST_PIPELINE_BS_OPEN ? -1 : ctx.bs.ceiling;
        ctx.v3s.v3_demand_iq=ctx.v3_iq_demand;
        ctx.v3s.v3_request_iq=ctx.v3_request;
        ctx.v3s.final_iq=cmd->final_iq_request;
        ctx.v3s.standstill_zero=standstill_zero;
        ctx.v3s.pulled_down=ctx.pulled_down;
        ctx.v3s.dropped_logical_ticks=ctx.g53.trace.dropped_logical_ticks;
    }
#endif
    const bool was_permitted=ctx.tlm.assist_permitted;
    const uint16_t seq=ctx.tlm.engage_seq;
    memset(&ctx.tlm,0,sizeof(ctx.tlm)); /* Frozen DEPRECATE_ZERO fields. */
    ctx.tlm.engage_seq=seq;
    ctx.tlm.torque_load_ctrl=in->torque_load_ctrl;
    ctx.tlm.torque_load_centikg=in->torque_load_centikg;
    ctx.tlm.cadence_rpm=in->cadence_rpm;
    ctx.tlm.iq_request_before_limits=pre_limits;   /* the active engine's request */
    ctx.tlm.final_iq_request=cmd->final_iq_request;
    ctx.tlm.iq_ceiling=ctx.ceiling;
    ctx.tlm.power_limited=lim.power_limited;
    ctx.tlm.battery_limited=lim.battery_limited ||
        (ctx.g53.trace.g1>=0 && ctx.g53.trace.g1<(int32_t)G53_G1_Q12_ONE); /* G53 PI #1 acting */
    ctx.tlm.phase_limited=lim.phase_limited;
    ctx.tlm.voltage_limited=lim.voltage_limited;
    ctx.tlm.thermal_limited=lim.thermal_limited;
    ctx.tlm.speed_limited=lim.speed_limited;
    /* Diagnostic permission preserves the native direction/liveness observation; it is not
     * consulted by the G53 PEDAL request decision above. */
    ctx.tlm.assist_permitted=ctx.g53.normal_permission && !observed_veto && !assist_off;
    ctx.tlm.block_positive=observed_veto || assist_off;
    ctx.tlm.direction_block=in->direction_inhibit;
    ctx.tlm.release_active=cmd->slew_mode==FIS_MODE_RELEASE || cmd->slew_mode==FIS_MODE_SAFETY;
    ctx.tlm.limiter_zeroed=pre_limits>0 && cmd->final_iq_request==0;
    if(!was_permitted && ctx.tlm.assist_permitted) ++ctx.tlm.engage_seq;
    if(in->direction_inhibit) ctx.tlm.pas_state=in->inhibit_is_reverse ? AP2_PAS_REVERSE : AP2_PAS_INVALID;
    else if(in->real_stop) ctx.tlm.pas_state=AP2_PAS_STOPPED;
    else if(ctx.g53.normal_permission) ctx.tlm.pas_state=AP2_PAS_FORWARD;
    else if(in->forward_valid) ctx.tlm.pas_state=AP2_PAS_STARTING_FORWARD;
    else ctx.tlm.pas_state=AP2_PAS_STOPPED;
    ctx.tlm.bike_rolling=in->speed_x100>100;
    ctx.tlm.rider_power_w=rider_power_w(in->torque_load_centikg,in->cadence_rpm);
    ctx.tlm.motor_power_w=motor_power_w(cmd->final_iq_request,in->battery_voltage_mv,in->u_abs,in->cal_i);
    ctx.tlm.applied_support_ratio_pct=ctx.tlm.rider_power_w>0 ?
        (uint16_t)ap2_clamp(((int32_t)ctx.tlm.motor_power_w*100)/ctx.tlm.rider_power_w,0,65535) : 0;
}
