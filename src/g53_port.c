#include "g53_port.h"
#include "g53_port_boundaries.h"
#include <string.h>

static g53_port_trace_t port_trace;
static g53_pas_ctx_t pas;

void g53_port_reset(void)
{
    g53_ax_reset();
    g53_ad7ec_reset();
    g53_pas_reset(&pas, true, 0);
    g53_chain_reset();
    memset(&port_trace, 0, sizeof(port_trace));
}

void g53_port_init(void) { g53_port_reset(); }

void g53_port_update(const g53_port_input_t *in, g53_port_output_t *out)
{
    /* Phase 1: safe-zero facade, not wired into production. */
    memset(out, 0, sizeof(*out));
    out->iq_request_pre_limits =
        g53_boundary_b_iq_request(out->m2aa_native, in->phase_current_max);
    port_trace = out->trace;
}

const g53_port_trace_t *g53_port_trace(void) { return &port_trace; }
