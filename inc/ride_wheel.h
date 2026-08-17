#ifndef RIDE_WHEEL_H_
#define RIDE_WHEEL_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * FW-112.2: the ONE production definition of "the wheel has demonstrably turned
 * recently enough to be rolling" - the helper that lets the REAL firmware and the
 * host harness link and test the exact same decision (no hand-copied mirror).
 *
 * It is the existing speed-freshness semantics reused unchanged:
 *   (now_ticks - last_accepted_pulse_tick) < SPEED_STOP_TICKS
 * i.e. an ACCEPTED wheel edge within the last 2.65 s (config.h's SPEED_STOP_TICKS,
 * derived from the 4 kHz control timebase - the same fact main.c already uses for
 * the speed display hard-zero, diag_moving, the auto-off and the comm watchdog).
 *
 * "ACCEPTED" is deliberate: main.c advances speed_last_tick only on an edge that
 * survived Speed_processing()'s glitch / impossible-jump rejection, so a rejected
 * (junk) edge must not re-validate a stopped wheel. 2.65 s is the conservative
 * production timeout: below ~3 km/h (1 pulse/rev, ~2218 mm circumference) the
 * firmware already declares the bike stopped, so this helper never claims
 * "rolling" where production would show Speedx100 == 0.
 *
 * Both tick arguments are the free-running control_time_ticks domain; the unsigned
 * subtraction stays correct across the 32-bit wrap (same discipline as main.c).
 */

bool ride_wheel_valid(uint32_t now_ticks, uint32_t last_accepted_pulse_tick);

#endif /* RIDE_WHEEL_H_ */
