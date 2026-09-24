#include "g53_port.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    const g53_port_input_t in = {.phase_current_max = 900, .elapsed_ticks = 1};
    g53_port_output_t out;
    memset(&out, 0xa5, sizeof(out));
    g53_port_init();
    g53_port_update(&in, &out);
    assert(out.iq_request_pre_limits == 0 && out.m2aa_native == 0);
    assert(!out.normal_permission && g53_port_trace()->m2aa == 0);
    g53_port_reset();
    assert(g53_port_trace()->m2aa == 0);
    puts("G53 integration: phase-1 facade reset/zero smoke PASS");
    return 0;
}
