#include "g53_port_pas.h"
#include <string.h>

/* Contract D4 B1..B12, X2: M820 geometry is 96, never the G5300 fixture's 144.
 * Units here are logical ticks. Native pas_direction/pas_liveness are separate
 * unconditional safety owners; this module never reads or writes their state. */
enum { PAS_TRANSITIONS = 96, PAS_TIMEOUT_MAX = 311, PAS_TIMEOUT_MIN = 25 };
static const uint8_t rank_by_code[4] = {1, 2, 4, 3};

static int32_t pas_s32(uint32_t value)
{
    return value <= INT32_MAX ? (int32_t)value
        : (int32_t)((int64_t)value - INT64_C(4294967296));
}

static int16_t pas_s16(uint16_t value)
{
    return value <= INT16_MAX ? (int16_t)value : (int16_t)((int32_t)value - 65536);
}

static int32_t pas_asr(int32_t value, unsigned shift)
{
    return value >= 0 ? value / (int32_t)(1u << shift)
        : (int32_t)(-((-(int64_t)value + ((1u << shift)-1u)) >> shift));
}

void g53_pas_reset(g53_pas_ctx_t *ctx, bool boot, uint8_t pas_ab)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->first_boot_call = boot;
    ctx->skip_first_boot_transition = boot;
    ctx->previous_code = boot ? 0 : (pas_ab & 3u);
    ctx->previous_rank = rank_by_code[ctx->previous_code];
    ctx->output.span = 3;
    ctx->output.base_timeout_ticks = PAS_TIMEOUT_MAX;
    ctx->output.full_timeout_ticks = PAS_TIMEOUT_MAX;
    ctx->output.max_timeout_ticks = PAS_TIMEOUT_MAX;
}

static void pas_filter_reset(g53_pas_output_t *out)
{
    out->filter_accumulator = 0;
    out->filtered_cadence = 0;
}

static void pas_true_stop(g53_pas_output_t *out)
{
    out->no_transition_ticks = out->full_timeout_ticks;
    out->span = 3;
    out->plausibility_budget = 32;
    out->direction = 0;
    out->evidence = 0;
    out->movement = 0;
    out->elapsed_ticks = 0;
    out->transition_count = 0;
    out->raw_cadence = 0;
    out->magnitude = 0;
    pas_filter_reset(out);
}

static void pas_plausibility(g53_pas_output_t *out)
{
    uint16_t latch = out->plausibility_latch;
    const bool cancel = out->current_delta + out->previous_delta == 0;
    if (!cancel && out->direction_candidate != 0) {
        latch = 0;
        if (out->plausibility_budget == 0) {
            out->plausibility_base = 0;
            out->plausibility_flag = 0;
        } else {
            out->plausibility_budget = (uint16_t)(out->plausibility_budget - 1u);
        }
    } else if ((uint16_t)(out->filtered_cadence + 15) > 30 || out->plausibility_latch == 1) {
        const uint16_t candidate = (uint16_t)(out->plausibility_budget + 2u);
        const uint16_t threshold = (uint16_t)(out->plausibility_base + PAS_TRANSITIONS/2);
        latch = 1;
        out->plausibility_budget = candidate;
        if (threshold < candidate) {
            out->plausibility_budget = threshold;
            out->plausibility_flag = 1;
        }
    }
    if ((uint16_t)(latch - out->plausibility_previous_latch) == 1) {
        out->plausibility_base = (uint16_t)(out->plausibility_budget - 2u);
    }
    out->plausibility_latch = latch;
}

static void pas_publish(g53_pas_output_t *out, uint8_t code)
{
    out->code = code;
    /* D4 B10: publication_scale=100, so (filtered*100)/100 is exact. */
    out->cadence = out->filtered_cadence;
    out->anti_rock = (uint8_t)(1u & out->plausibility_flag);
}

g53_pas_output_t g53_pas_step(g53_pas_ctx_t *ctx, uint8_t pas_ab, uint8_t g53_level)
{
    (void)g53_level; /* D4 B12: every per-level extra stop timeout is zero. */
    const uint8_t code = pas_ab & 3u;
    g53_pas_output_t *out = &ctx->output;
    out->no_transition_ticks = (uint16_t)(out->no_transition_ticks + 1u);
    out->elapsed_ticks = (uint16_t)(out->elapsed_ticks + 1u);
    if (out->no_transition_ticks >= out->full_timeout_ticks) pas_true_stop(out);
    const bool changed = code != ctx->previous_code;
    if (ctx->first_boot_call) {
        ctx->first_boot_call = false;
        if (!changed) {
            out->no_transition_ticks = 0;
            out->elapsed_ticks = 0;
            pas_publish(out, code);
            return *out;
        }
    }
    if (changed) {
        const uint8_t rank = rank_by_code[code];
        out->no_transition_ticks = 0;
        out->movement = 1;
        if (ctx->skip_first_boot_transition) {
            ctx->skip_first_boot_transition = false;
            out->transition_count = (uint16_t)(out->transition_count + 1u);
            ctx->previous_rank = rank;
            ctx->previous_code = code;
            pas_publish(out, code);
            return *out;
        }
        const int16_t delta = (int16_t)((int)rank - ctx->previous_rank);
        out->current_delta = delta;
        int16_t direction = out->direction;
        if (delta == 1 || delta == -3) {
            direction = 1;
            out->evidence = (uint16_t)(out->evidence + 1u);
        } else if (delta == 3 || delta == -1) {
            direction = -1;
            out->evidence = 0;
        }
        out->direction_candidate = direction;
        if (direction != out->direction || out->plausibility_flag == 1) {
            out->transition_count = 0;
            out->elapsed_ticks = 0;
            out->raw_cadence = 0;
            out->magnitude = 0;
            pas_filter_reset(out);
            out->direction = direction;
        } else if (direction != 0) {
            out->transition_count = (uint16_t)(out->transition_count + 1u);
            if (out->elapsed_ticks != 0 && out->transition_count >= out->span) {
                const uint32_t numerator = 5u * ((36000u / PAS_TRANSITIONS) * out->span);
                const uint32_t magnitude = (numerator / out->elapsed_ticks) / 3u;
                out->magnitude = (uint16_t)magnitude;
                out->raw_cadence = pas_s16((uint16_t)(direction * (int32_t)magnitude));
                out->transition_count = 0;
                out->elapsed_ticks = 0;
            }
        }
        const int threshold = 16 * out->span - 8;
        const uint16_t candidate1 = (uint16_t)(1u + out->magnitude / 16u);
        const uint16_t candidate2 = (uint16_t)(1u + (out->magnitude + 8u) / 16u);
        out->span = (uint8_t)(out->magnitude < threshold ? candidate2 : candidate1);
        if (out->magnitude < 40) out->span = 3;
        const int32_t correction = pas_asr(1638 * ((int32_t)out->raw_cadence - out->filtered_cadence), 12);
        out->filter_accumulator = pas_s32((uint32_t)out->filter_accumulator + (uint32_t)correction);
        out->filtered_cadence = pas_s16((uint16_t)pas_asr(out->filter_accumulator, 2));
        uint32_t timeout = 180000u / ((uint32_t)PAS_TRANSITIONS * out->magnitude + 1u);
        if (timeout < PAS_TIMEOUT_MIN) timeout = PAS_TIMEOUT_MIN;
        if (timeout > PAS_TIMEOUT_MAX) timeout = PAS_TIMEOUT_MAX;
        out->base_timeout_ticks = (uint16_t)timeout;
        out->full_timeout_ticks = (uint16_t)timeout;
        pas_plausibility(out);
        ctx->previous_rank = rank;
        ctx->previous_code = code;
    }
    out->previous_delta = out->current_delta;
    out->plausibility_previous_latch = out->plausibility_latch;
    pas_publish(out, code);
    return *out;
}
