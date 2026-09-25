#include "g53_port.h"
#include "g53_port_boundaries.h"
#include <string.h>

/* M820 adapter: 4 control periods (4 kHz) per logical tick. This is an explicit
 * product timebase, not a claim about the original G53 oscillator. */
enum { CONTROL_TICKS_PER_LOGICAL=4, MAX_CATCHUP_LOGICAL_TICKS=64 };
static g53_port_trace_t port_trace;
static g53_pas_ctx_t pas;
static uint8_t control_remainder;
static bool normal_permission;
static uint32_t dropped_ticks;

void g53_port_reset(void)
{
    g53_ax_reset();
    g53_ad7ec_reset();
    g53_pas_reset(&pas,true,0);
    g53_chain_reset();
    memset(&port_trace,0,sizeof(port_trace));
    control_remainder=0;
    normal_permission=false;
    dropped_ticks=0;
}
void g53_port_init(void) { g53_port_reset(); }

void g53_port_update(const g53_port_input_t *in,g53_port_output_t *out)
{
    const uint64_t elapsed=(uint64_t)(in->elapsed_ticks ? in->elapsed_ticks : 1u)+control_remainder;
    uint32_t steps=(uint32_t)(elapsed/CONTROL_TICKS_PER_LOGICAL);
    control_remainder=(uint8_t)(elapsed%CONTROL_TICKS_PER_LOGICAL);
    if(steps>MAX_CATCHUP_LOGICAL_TICKS) {
        const uint32_t lost=steps-MAX_CATCHUP_LOGICAL_TICKS;
        dropped_ticks=UINT32_MAX-dropped_ticks<lost ? UINT32_MAX : dropped_ticks+lost;
        steps=MAX_CATCHUP_LOGICAL_TICKS;
    }
    const uint8_t level=g53_chain_level(in->assist_level);
    const uint32_t speed=in->speed_x100/10u;
    const uint32_t diag_word=(in->safety_cut || !in->torque_sensor_valid) ? 0x10u : 0u;
    /* Missed ticks use this update's held input. Lost edge history is never
     * reconstructed. Catch-up is bounded; any excess is explicitly counted. */
    while(steps--) {
        g53_chain_input_t ci={0};
        g53_chain_output_t co;
        ci.x=g53_ax_step(in->raw_pa6_adc);
        ci.pas=g53_pas_step(&pas,in->pas_ab,level);
        const g53_ad7ec_feedback_t feedback={
            .cadence=ci.pas.cadence,
            .speed_native=(uint16_t)(speed>32767u ? 32767u : speed),
            .d7ec_rider=(uint16_t)port_trace.d7ec_accel,
            .m298=(uint8_t)port_trace.m298
        };
        ci.rider_input_native=g53_ad7ec_step(in->load_ctrl,&feedback).rider_input_native;
        ci.level=level;
        ci.speed_native=(int16_t)feedback.speed_native;
        ci.diag_word=diag_word;
        ci.external_inhibit=0;
        ci.g1_q12=0x1000; ci.g2_q12=0x1000;
        g53_chain_step(&ci,&co);
        port_trace=co.trace;
        normal_permission=co.normal_permission;
    }
    port_trace.raw_pa6_adc=in->raw_pa6_adc;
    port_trace.diag=diag_word;
    port_trace.dropped_logical_ticks=dropped_ticks;
    out->trace=port_trace;
    out->m2aa_native=(uint16_t)port_trace.m2aa;
    out->normal_permission=normal_permission;
    out->iq_request_pre_limits =
        g53_boundary_b_iq_request(out->m2aa_native,in->phase_current_max);
    /* direction_inhibit and real_stop stay native pipeline vetoes, never G53
     * external_inhibit, diag aliases or extra behavioral demand gates. */
}
const g53_port_trace_t *g53_port_trace(void) { return &port_trace; }
