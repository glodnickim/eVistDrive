#ifndef L4_EB74_INVOCATION_OBSERVER_H
#define L4_EB74_INVOCATION_OBSERVER_H

#include <stdint.h>
#include "g53_port_boundaries.h"

typedef struct {
    uint32_t count;
    uint16_t load_ctrl;
    g53_ad7ec_output_t output;
} l4_eb74_observation_t;

void l4_eb74_observer_reset(void);
uint32_t l4_eb74_observer_count(void);
l4_eb74_observation_t l4_eb74_observer_last(void);

#endif
