#include "g53_port_boundaries.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    const g53_ad7ec_feedback_t feedback = {0};
    g53_ax_reset();
    g53_ad7ec_reset();
    assert(g53_ax_step(0) == 0);
    assert(g53_ad7ec_step(0, &feedback).rider_input_native == 0);
    assert(g53_boundary_b_iq_request(0, 900) == 0);
    puts("G53 boundaries: phase-1 reset/zero smoke PASS");
    return 0;
}
