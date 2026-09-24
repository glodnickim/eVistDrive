#ifndef G53_PORT_PAS_H
#define G53_PORT_PAS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t direction;
    int16_t direction_candidate;
    int16_t cadence;
    uint16_t evidence;
    uint8_t movement;
    uint16_t base_timeout_ticks;
    uint16_t full_timeout_ticks;
    uint16_t max_timeout_ticks;
    uint16_t no_transition_ticks;
    uint8_t code;
    uint8_t anti_rock;
    uint8_t span;
    uint16_t magnitude;
    int16_t raw_cadence;
    int16_t current_delta;
    int16_t previous_delta;
    uint16_t transition_count;
    uint16_t elapsed_ticks;
    int32_t filter_accumulator;
    int16_t filtered_cadence;
    uint16_t plausibility_budget;
    uint16_t plausibility_base;
    uint16_t plausibility_latch;
    uint16_t plausibility_previous_latch;
    uint16_t plausibility_flag;
} g53_pas_output_t;

typedef struct {
    g53_pas_output_t output;
    uint8_t previous_code;
    uint8_t previous_rank;
    bool first_boot_call;
    bool skip_first_boot_transition;
} g53_pas_ctx_t;

void g53_pas_reset(g53_pas_ctx_t *ctx, bool boot, uint8_t pas_ab);
g53_pas_output_t g53_pas_step(g53_pas_ctx_t *ctx, uint8_t pas_ab, uint8_t g53_level);

#endif
