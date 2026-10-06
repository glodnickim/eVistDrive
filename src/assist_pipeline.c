#include "assist_pipeline.h"
#include "ap2_math.h"
#include "assist_modes.h"
#include "config.h"
#include "tuning_config.h"
#include <string.h>

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
    /* The request published on the previous tick: the reference a non-forward tick may not
     * rise above. 0 after every reset and after every veto, so nothing waits behind one. */
    int32_t last_final_iq;
    /* 4 kHz ticks the direction inhibit has been continuously active (saturating). It bounds
     * the moving-bike decay below; 0 whenever the inhibit is not active. */
    uint32_t inhibit_ticks;
    /* Bumpless veto release: true while the published request was held below the G53 request by
     * an M820 veto/gate on the previous tick, and the remainder of the rise-rate budget. */
    bool pulled_down;
    uint32_t rise_acc;
    assist_pipeline_telemetry_t tlm;
    g53_port_output_t g53;
} ap2_pipeline_ctx_t;
static ap2_pipeline_ctx_t ctx;

void assist_pipeline_reset(void)
{
    const uint16_t seq=ctx.tlm.engage_seq;
    memset(&ctx,0,sizeof(ctx));
    ctx.tlm.engage_seq=seq;
    g53_port_reset();
    /* Shared battery protections survive rider/Walk/service owner changes. */
}
void assist_pipeline_init(void) { assist_pipeline_reset(); ap2_limits_reset(); }
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
    const bool veto=in->direction_inhibit || native_cut || in->real_stop;
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
    lim_in.iq_request=ctx.g53.iq_request_pre_limits;
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
    const int32_t g53_request=lim.final_iq;
    /*
     * RIDER GATES on the G53 request. normal_permission (m298==2) is a BDE8 drive mode, not a
     * pedal permission, so it gates nothing here.
     *   assist off     -> exactly zero in the same tick.
     *   not forward    -> never a new or a rising request: an already valid pedal demand may
     *                     only hold or decay (G53's own release), and 0 stays 0.
     */
    /* Never above where the live reference really is, when it is known: the published target
     * is compared with the PREVIOUS REQUEST, but the 16 kHz owner may still be below it (a
     * binding fast rise), and a hold/decay must not lift it (review ISSUE 1). */
    int32_t hold_cap=ctx.last_final_iq;
    if(in->live_iq_valid && hold_cap>(in->live_iq_ref>0 ? in->live_iq_ref : 0))
        hold_cap=in->live_iq_ref>0 ? in->live_iq_ref : 0;
    if(assist_off) cmd->final_iq_request=0;
    else if(!in->forward_valid && cmd->final_iq_request>hold_cap)
        cmd->final_iq_request=hold_cap;
    bool quiet=false;
    if(in->direction_inhibit) {
        if(ctx.inhibit_ticks<UINT32_MAX-used_ticks) ctx.inhibit_ticks+=used_ticks;
        else ctx.inhibit_ticks=UINT32_MAX;
    } else ctx.inhibit_ticks=0;
    /*
     * P2, before P3/P4. TASK-EVD-TQ-06-G2 / OWNER-DEC-2026-10-05-TQ06G2-A: a reverse or invalid
     * crank step on a MOVING bike (the speed G53 itself sees, speed_x100/10 > 0) must not be a
     * step to zero: stock G5300 ramps the demand down while the wheel turns and cuts at once only
     * at standstill. The direction authority therefore becomes "never rise, bounded decay":
     *   - the published request is the G53 chain's own output (it already decays like G5300),
     *     clamped so it never exceeds the previous tick's published value;
     *   - it is exactly zero no later than AP2_SAFETY_RELEASE_MS after the inhibit began, whatever
     *     the chain does (hard bound, FORCE_ZERO from then on).
     * Everything else keeps the same-tick exact zero: standstill, assist off, and any native cut
     * (brake / fault / calibration / invalid sensor) or real stop that coincides with the inhibit.
     */
    const bool moving=in->speed_x100>=10u;
    if(in->direction_inhibit) {
        const bool bounded_decay=moving && !assist_off && !native_cut && !in->real_stop &&
            !in->service_cut && ctx.inhibit_ticks<AP2_SAFETY_RELEASE_MS*AP2_TICKS_PER_MS;
        if(bounded_decay) {
            if(cmd->final_iq_request>hold_cap) cmd->final_iq_request=hold_cap;
            /* FALL (G5300 step), never BYPASS: BYPASS copies the target into the live
             * accumulator and would lift a still-rising reference to the previous request. */
            if(in->phase_current_max>0) {
                cmd->step_mag_8=fast_slew_step(in->phase_current_max);
                cmd->slew_mode=FIS_MODE_FALL;
            } else cmd->slew_mode=FIS_MODE_BYPASS;
            quiet=cmd->final_iq_request==0;
        } else {
            cmd->final_iq_request=0; cmd->slew_mode=FIS_MODE_FORCE_ZERO; quiet=true;
        }
    } else if(native_cut || in->real_stop) {
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
        if(assist_off || in->phase_current_max<=0) cmd->slew_mode=FIS_MODE_BYPASS;
        else if(stock_hard_zero) { cmd->final_iq_request=0; cmd->slew_mode=FIS_MODE_FORCE_ZERO; }
        else {
            cmd->step_mag_8=fast_slew_step(in->phase_current_max);
            cmd->slew_mode=cmd->final_iq_request>ctx.last_final_iq ? FIS_MODE_RISE : FIS_MODE_FALL;
        }
        quiet=cmd->final_iq_request==0 && (assist_off || !in->forward_valid);
    }
    ctx.pulled_down=cmd->final_iq_request<g53_request;
    cmd->zero_policy=quiet && !in->service_cut ? FIS_ZERO_POLICY_QUIET : FIS_ZERO_POLICY_NONE;
    ctx.last_final_iq=cmd->final_iq_request;
    const bool was_permitted=ctx.tlm.assist_permitted;
    const uint16_t seq=ctx.tlm.engage_seq;
    memset(&ctx.tlm,0,sizeof(ctx.tlm)); /* Frozen DEPRECATE_ZERO fields. */
    ctx.tlm.engage_seq=seq;
    ctx.tlm.torque_load_ctrl=in->torque_load_ctrl;
    ctx.tlm.torque_load_centikg=in->torque_load_centikg;
    ctx.tlm.cadence_rpm=in->cadence_rpm;
    ctx.tlm.iq_request_before_limits=ctx.g53.iq_request_pre_limits;
    ctx.tlm.final_iq_request=cmd->final_iq_request;
    ctx.tlm.iq_ceiling=ctx.ceiling;
    ctx.tlm.power_limited=lim.power_limited;
    ctx.tlm.battery_limited=lim.battery_limited ||
        (ctx.g53.trace.g1>=0 && ctx.g53.trace.g1<(int32_t)G53_G1_Q12_ONE); /* G53 PI #1 acting */
    ctx.tlm.phase_limited=lim.phase_limited;
    ctx.tlm.voltage_limited=lim.voltage_limited;
    ctx.tlm.thermal_limited=lim.thermal_limited;
    ctx.tlm.speed_limited=lim.speed_limited;
    ctx.tlm.assist_permitted=ctx.g53.normal_permission && !veto && !assist_off;
    ctx.tlm.block_positive=veto || assist_off;
    ctx.tlm.direction_block=in->direction_inhibit;
    ctx.tlm.release_active=cmd->slew_mode==FIS_MODE_RELEASE || cmd->slew_mode==FIS_MODE_SAFETY;
    ctx.tlm.limiter_zeroed=ctx.g53.iq_request_pre_limits>0 && cmd->final_iq_request==0;
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
