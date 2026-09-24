#include "g53_port_boundaries.h"
#include <string.h>

/* Boundary A-x: contract M-x / G53 0x080164B0. Raw PA6, not torque or mapped Iq.
 * State and updates are in G53 logical invocations, with one integer truncation
 * at each of the two shifts. The facade supplies the logical tick schedule. */
static uint16_t ax;
void g53_ax_reset(void) { ax = 0; }
uint16_t g53_ax_step(uint16_t raw_pa6_adc)
{
    const uint16_t u = (uint16_t)(((uint32_t)raw_pa6_adc * 498u) >> 12);
    ax = (uint16_t)((3u * (uint32_t)ax + u) >> 2);
    return ax;
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
