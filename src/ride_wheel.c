#include "ride_wheel.h"
#include "config.h"

bool ride_wheel_valid(uint32_t now_ticks, uint32_t last_accepted_pulse_tick)
{
	/* Exactly main.c's existing freshness fact (SPEED_STOP_TICKS = 2.65 s @ 4 kHz),
	 * extracted so firmware and host harness link the SAME decision. The unsigned
	 * subtraction wraps correctly across the 32-bit tick counter. */
	return (now_ticks - last_accepted_pulse_tick) < SPEED_STOP_TICKS;
}
