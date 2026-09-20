#include "ap2_torque_chain.h"
#include "ap2_math.h"

typedef enum {
    TQ_STATE_RESET = 0,
    TQ_STATE_GATE = 1,
    TQ_STATE_CONFIRM = 2,
    TQ_STATE_ACTIVE = 3
} tq_state_t;

typedef struct {
    tq_state_t state;
    uint32_t gate_ticks;
    uint32_t confirm_ticks;
    int32_t envelope_q16;
    int32_t base_q16;
    int32_t dynamic_q16;
    int32_t peak_q16;
} tq_ctx_t;

static tq_ctx_t ctx;

static uint16_t gate_threshold_ckg = AP2_TORQUE_CHAIN_GATE_THRESHOLD_CENTIKG;
static uint16_t confirm_threshold_ckg = AP2_TORQUE_CHAIN_CONFIRM_THRESHOLD_CENTIKG;
static uint16_t active_threshold_ckg = AP2_TORQUE_CHAIN_ACTIVE_THRESHOLD_CENTIKG;
static uint16_t saturation_ckg = AP2_TORQUE_CHAIN_SATURATION_CENTIKG;
static uint16_t gate_time_ms = AP2_TORQUE_CHAIN_GATE_TIME_MS;
static uint16_t confirm_time_ms = AP2_TORQUE_CHAIN_CONFIRM_TIME_MS;
static uint16_t slew_rise_permille_per_10ms = AP2_TORQUE_CHAIN_SLEW_RISE_PERMILLE_PER_10MS;

void ap2_torque_chain_reset(void)
{
    ctx.state = TQ_STATE_RESET;
    ctx.gate_ticks = 0U;
    ctx.confirm_ticks = 0U;
    ctx.envelope_q16 = 0;
    ctx.base_q16 = 0;
    ctx.dynamic_q16 = 0;
    ctx.peak_q16 = 0;
}

void ap2_torque_chain_seed(int32_t permille)
{
    int32_t v = ap2_clamp(permille, 0, AP2_TORQUE_CHAIN_PERMILLE_MAX);
    ctx.state = TQ_STATE_ACTIVE;
    ctx.gate_ticks = 0U;
    ctx.confirm_ticks = 0U;
    ctx.envelope_q16 = ap2_q16_from(v);
    ctx.base_q16 = ap2_q16_from(v);
    ctx.dynamic_q16 = 0;
    ctx.peak_q16 = ap2_q16_from(v);
}

static uint16_t stroke_period_ms(uint8_t cadence_rpm)
{
    uint32_t ms;
    if (cadence_rpm == 0U) {
        return 1500U;
    }
    ms = 30000U / (uint32_t)cadence_rpm;
    if (ms < 150U) ms = 150U;
    if (ms > 1500U) ms = 1500U;
    return (uint16_t)ms;
}

void ap2_torque_chain_update(const ap2_demand_input_t *in, ap2_demand_output_t *out)
{
    uint16_t period;
    int32_t effort;
    int32_t demand;

    if (out == 0) {
        return;
    }
    if (in == 0) {
        ap2_torque_chain_reset();
        out->effort_permille = 0;
        out->demand_permille = 0;
        out->base_permille = 0;
        out->dynamic_permille = 0;
        out->stroke_peak_permille = 0;
        out->stroke_period_ms = stroke_period_ms(0);
        out->load_centikg = 0U;
        return;
    }

    if (!in->torque_valid) {
        ap2_torque_chain_reset();
        out->effort_permille = 0;
        out->demand_permille = 0;
        out->base_permille = 0;
        out->dynamic_permille = 0;
        out->stroke_peak_permille = 0;
        out->stroke_period_ms = stroke_period_ms(in->cadence_rpm);
        out->load_centikg = in->load_centikg;
        return;
    }

    period = stroke_period_ms(in->cadence_rpm);

    if (in->load_centikg <= AP2_TORQUE_CHAIN_DEADBAND_CENTIKG) {
        effort = 0;
    } else {
        uint16_t full_scale = in->full_scale_centikg ? in->full_scale_centikg : 6000U;
        if (full_scale <= AP2_TORQUE_CHAIN_DEADBAND_CENTIKG) {
            full_scale = AP2_TORQUE_CHAIN_DEADBAND_CENTIKG + 1U;
        }
        effort = ap2_map((int32_t)in->load_centikg - (int32_t)AP2_TORQUE_CHAIN_DEADBAND_CENTIKG,
                         0, (int32_t)full_scale - (int32_t)AP2_TORQUE_CHAIN_DEADBAND_CENTIKG,
                         0, AP2_TORQUE_CHAIN_PERMILLE_MAX);
    }

    ctx.envelope_q16 = ap2_lpf_step(ctx.envelope_q16, effort,
                                    20U, in->elapsed_ticks);
    demand = ap2_q16_value(ctx.envelope_q16);

    /* Map demand to target with engagement gating and saturation.
     * The PAS state machine owns the stop/start decision; here we only gate
     * the initial engagement. Once engaged, ACTIVE exits only on !pedaling,
     * not on within-stroke demand variation (fixes S1 base term collapse). */
    uint32_t gate_ticks_needed = ((uint32_t)gate_time_ms * in->elapsed_ticks) / 10U;
    uint32_t confirm_ticks_needed = ((uint32_t)confirm_time_ms * in->elapsed_ticks) / 10U;
    int32_t target;

    switch (ctx.state) {
        case TQ_STATE_RESET:
            ctx.state = TQ_STATE_GATE;
            ctx.gate_ticks = 0U;
            break;

        case TQ_STATE_GATE:
            if (demand >= gate_threshold_ckg * AP2_TORQUE_CHAIN_PERMILLE_MAX / 6000) {
                if (ctx.gate_ticks < gate_ticks_needed) {
                    ctx.gate_ticks += in->elapsed_ticks;
                }
                if (ctx.gate_ticks >= gate_ticks_needed) {
                    ctx.state = TQ_STATE_CONFIRM;
                    ctx.confirm_ticks = 0U;
                }
            } else {
                if (ctx.gate_ticks >= in->elapsed_ticks) ctx.gate_ticks -= in->elapsed_ticks;
                else ctx.gate_ticks = 0U;
            }
            target = 0;
            break;

        case TQ_STATE_CONFIRM:
            if (demand >= confirm_threshold_ckg * AP2_TORQUE_CHAIN_PERMILLE_MAX / 6000) {
                if (ctx.confirm_ticks < confirm_ticks_needed) {
                    ctx.confirm_ticks += in->elapsed_ticks;
                }
                if (ctx.confirm_ticks >= confirm_ticks_needed) {
                    ctx.state = TQ_STATE_ACTIVE;
                }
            } else {
                ctx.state = TQ_STATE_GATE;
                ctx.gate_ticks = 0U;
            }
            target = 0;
            break;

        case TQ_STATE_ACTIVE:
            /* Exit ACTIVE only on true pedal stop (PAS state machine decision).
             * Within-stroke demand variation (dead spot between legs) must NOT exit ACTIVE,
             * otherwise base term collapses and assist pulses (S1 failure). */
            if (!in->pedaling) {
                ctx.state = TQ_STATE_RESET;
                target = 0;
                break;
            }
            if (demand >= saturation_ckg * AP2_TORQUE_CHAIN_PERMILLE_MAX / 6000) {
                target = AP2_TORQUE_CHAIN_PERMILLE_MAX;
            } else if (demand > active_threshold_ckg * AP2_TORQUE_CHAIN_PERMILLE_MAX / 6000) {
                target = ap2_map(demand,
                                 active_threshold_ckg * AP2_TORQUE_CHAIN_PERMILLE_MAX / 6000,
                                 saturation_ckg * AP2_TORQUE_CHAIN_PERMILLE_MAX / 6000,
                                 0, AP2_TORQUE_CHAIN_PERMILLE_MAX);
            } else {
                /* Between active_threshold(180) and zero: track demand directly.
                 * Base/dynamic split handles the dead-spot hold. */
                target = demand;
            }
            break;

        default:
            ctx.state = TQ_STATE_RESET;
            target = 0;
            break;
    }

    /* Slew limit the target (rise limited, fall immediate). */
    static int32_t slewed_target = 0;
    if (target > slewed_target) {
        int32_t max_rise = slew_rise_permille_per_10ms * in->elapsed_ticks / 10U;
        int32_t diff = target - slewed_target;
        slewed_target += (diff > max_rise) ? max_rise : diff;
    } else {
        slewed_target = target;
    }

    /* ---- BASE / DYNAMIC SPLIT (original ap2_rider_demand.c logic) -----------
     * Split the demand into a sustained base (slow) and a fast dynamic excess.
     * Base rises quickly toward higher effort and falls over at least one and a half
     * pedal strokes, so the dead spot between leg pushes cannot empty it. When the
     * cranks are not driving, the base is released at the stroke rate so it cannot
     * survive into the next ride - but the STOP decision itself belongs to the PAS
     * state machine, not here.
     */
    period = stroke_period_ms(in->cadence_rpm);
    int32_t base_fall_ms = ((uint32_t)period * 3U) / 2U;  /* 1.5 strokes */
    if (in->base_hold_ms > (uint16_t)base_fall_ms) {
        base_fall_ms = in->base_hold_ms;
    }
    if (!in->pedaling) {
        base_fall_ms = period;  /* One stroke period when not pedaling */
    }

    if (slewed_target >= ap2_q16_value(ctx.base_q16)) {
        ctx.base_q16 = ap2_lpf_step(ctx.base_q16, slewed_target, 120U, in->elapsed_ticks);
    } else {
        ctx.base_q16 = ap2_lpf_step(ctx.base_q16, slewed_target, base_fall_ms, in->elapsed_ticks);
    }
    int32_t base = ap2_clamp(ap2_q16_value(ctx.base_q16), 0, AP2_TORQUE_CHAIN_PERMILLE_MAX);

    /* ---- DYNAMIC COMPONENT ----------------------------------------------------------- */
    int32_t excess = slewed_target - base;
    if (excess < 0) excess = 0;
    if (excess >= ap2_q16_value(ctx.dynamic_q16)) {
        ctx.dynamic_q16 = ap2_lpf_step(ctx.dynamic_q16, excess, 25U, in->elapsed_ticks);
    } else {
        ctx.dynamic_q16 = ap2_lpf_step(ctx.dynamic_q16, excess, 150U, in->elapsed_ticks);
    }
    int32_t dynamic = ap2_clamp(ap2_q16_value(ctx.dynamic_q16), 0, AP2_TORQUE_CHAIN_PERMILLE_MAX);

    /* ---- STROKE PEAK (description only) ---------------------------------------------- */
    if (slewed_target >= ap2_q16_value(ctx.peak_q16)) {
        ctx.peak_q16 = ap2_q16_from(slewed_target);
    } else {
        ctx.peak_q16 = ap2_lpf_step(ctx.peak_q16, slewed_target,
            (uint32_t)period * 2U, in->elapsed_ticks);
    }

    out->effort_permille = effort;
    out->demand_permille = slewed_target;
    out->base_permille = base;
    out->dynamic_permille = dynamic;
    out->stroke_peak_permille = ap2_clamp(ap2_q16_value(ctx.peak_q16), 0, AP2_TORQUE_CHAIN_PERMILLE_MAX);
    out->stroke_period_ms = period;
    out->load_centikg = in->load_centikg;
}