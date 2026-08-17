#include "pas_liveness.h"

/* FW-112.1: the dedicated any-PAS-edge liveness timer and its REAL_STOP verdict. See the header
 * for why this is a separate counter rather than pas_idle_ticks (which invalid transitions never
 * refresh). */
static uint32_t idle_ticks;
static bool     stopped;

void pas_liveness_init(void)
{
	idle_ticks = 0U;
	stopped = false;
}

void pas_liveness_transition(void)
{
	idle_ticks = 0U;
}

void pas_liveness_tick(uint16_t stop_timeout_ticks)
{
	if (idle_ticks < 0xFFFFFFFFU) idle_ticks++;
	stopped = (idle_ticks > (uint32_t)stop_timeout_ticks);
}

bool pas_liveness_stopped(void)
{
	return stopped;
}

uint32_t pas_liveness_idle_ticks(void)
{
	return idle_ticks;
}