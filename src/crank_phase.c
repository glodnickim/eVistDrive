#include "crank_phase.h"

#include <string.h>

/* See inc/crank_phase.h. All state in one struct so the RAM cost is one sizeof(). */
static struct {
	uint32_t steps;            /* unsigned so the wrap is defined; read back as int32_t */
	uint32_t last_step_tick;
	bool glitch;
	crank_phase_stats_t stats;
} C;

void crank_phase_init(void)
{
	memset(&C, 0, sizeof(C));
}

void crank_phase_on_event(const pas_step_event_t *ev)
{
	if (ev == 0) return;
	if (ev->step > 0) {
		C.steps += 1u;
		C.last_step_tick = ev->tick;
		C.stats.forward_steps++;
	} else if (ev->step < 0) {
		C.steps -= 1u;
		C.last_step_tick = ev->tick;
		C.stats.reverse_steps++;
	} else {
		/* A two-bit jump: two steps happened but the direction is unknown, so nothing is
		 * counted and the phase is reported as lost (ARCHITECTURE_V3 3.2, R1-#15). */
		C.glitch = true;
		C.stats.invalid_jumps++;
	}
}

void crank_phase_on_overflow(void)
{
	/* The ring dropped events: the count is short by an unknown number of steps. */
	C.glitch = true;
	C.stats.overflows++;
}

int32_t crank_phase_steps(void) { return (int32_t)C.steps; }
uint32_t crank_phase_last_step_tick(void) { return C.last_step_tick; }

bool crank_phase_take_glitch(void)
{
	const bool g = C.glitch;
	C.glitch = false;
	return g;
}

const crank_phase_stats_t *crank_phase_stats(void) { return &C.stats; }
