#include "eb74_invocation_observer.h"

static l4_eb74_observation_t observation;

g53_ad7ec_output_t __real_g53_ad7ec_step(uint16_t load_ctrl,
                                        const g53_ad7ec_feedback_t *feedback);

g53_ad7ec_output_t __wrap_g53_ad7ec_step(uint16_t load_ctrl,
                                        const g53_ad7ec_feedback_t *feedback)
{
    const g53_ad7ec_output_t output = __real_g53_ad7ec_step(load_ctrl, feedback);
    ++observation.count;
    observation.load_ctrl = load_ctrl;
    observation.output = output;
    return output;
}

void l4_eb74_observer_reset(void)
{
    observation = (l4_eb74_observation_t){0};
}

uint32_t l4_eb74_observer_count(void)
{
    return observation.count;
}

l4_eb74_observation_t l4_eb74_observer_last(void)
{
    return observation;
}
