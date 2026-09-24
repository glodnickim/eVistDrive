#include "g53_port_boundaries.h"
#include <string.h>

/* Phase-1 safe stubs. No production caller until the phase-7 cutover. */
void g53_ax_reset(void) {}
uint16_t g53_ax_step(uint16_t raw_pa6_adc)
{
    (void)raw_pa6_adc;
    return 0;
}

void g53_ad7ec_reset(void) {}
g53_ad7ec_output_t g53_ad7ec_step(uint16_t load_ctrl,
                                 const g53_ad7ec_feedback_t *feedback)
{
    g53_ad7ec_output_t out;
    (void)load_ctrl;
    (void)feedback;
    memset(&out, 0, sizeof(out));
    return out;
}

int32_t g53_boundary_b_iq_request(uint16_t m2aa_native, int32_t phase_current_max)
{
    (void)m2aa_native;
    (void)phase_current_max;
    return 0;
}
