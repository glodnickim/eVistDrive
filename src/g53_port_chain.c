#include "g53_port_chain.h"
#include <string.h>

void g53_chain_reset(void) {}
void g53_chain_step(const g53_chain_input_t *in, g53_chain_output_t *out)
{
    (void)in;
    memset(out, 0, sizeof(*out));
}
