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
} g53_pas_output_t;

typedef struct {
    g53_pas_output_t output;
} g53_pas_ctx_t;

void g53_pas_reset(g53_pas_ctx_t *ctx, bool boot, uint8_t pas_ab);
g53_pas_output_t g53_pas_step(g53_pas_ctx_t *ctx, uint8_t pas_ab, uint8_t g53_level);

#endif
