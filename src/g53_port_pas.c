#include "g53_port_pas.h"
#include <string.h>

void g53_pas_reset(g53_pas_ctx_t *ctx, bool boot, uint8_t pas_ab)
{
    (void)boot;
    (void)pas_ab;
    memset(ctx, 0, sizeof(*ctx));
}

g53_pas_output_t g53_pas_step(g53_pas_ctx_t *ctx, uint8_t pas_ab, uint8_t g53_level)
{
    (void)pas_ab;
    (void)g53_level;
    return ctx->output;
}
