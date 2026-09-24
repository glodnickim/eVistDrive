#include "g53_port_chain.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    const g53_chain_input_t in = {.g1_q12 = 0x1000, .g2_q12 = 0x1000};
    g53_chain_output_t out;
    memset(&out, 0xa5, sizeof(out));
    g53_chain_reset();
    g53_chain_step(&in, &out);
    assert(out.m2aa_native == 0 && !out.normal_permission);
    assert(out.trace.m2aa == 0);
    puts("G53 chain: phase-1 reset/zero smoke PASS");
    return 0;
}
