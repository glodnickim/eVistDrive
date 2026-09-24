#include "g53_port_pas.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    g53_pas_ctx_t ctx;
    memset(&ctx, 0xa5, sizeof(ctx));
    g53_pas_reset(&ctx, true, 0);
    g53_pas_output_t out = g53_pas_step(&ctx, 0, 0);
    assert(out.direction == 0 && out.cadence == 0 && out.evidence == 0);
    assert(out.movement == 0);
    puts("G53 PAS: phase-1 reset/zero smoke PASS");
    return 0;
}
