#ifndef AP2_PAS_STATE_H_
#define AP2_PAS_STATE_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * ASSIST PIPELINE V2 - PAS / DIRECTION STATE MACHINE.
 *
 * ONE owner of the question "is pedal assist allowed to flow right now, and what part of the
 * pedalling lifecycle are we in". It replaces the three overlapping permission concepts the
 * legacy path carried (the ride session automaton, the pedal-assist gate and the ride latch's
 * own hold/floor bookkeeping), which between them could each answer "yes" for different
 * reasons and had to be reconciled at every call site.
 *
 * IT IS NOT A SENSOR OWNER. Direction plausibility stays in src/pas_direction.c, electrical
 * PAS sampling in src/pas_sampler.c and conditioned cadence in src/cadence_filter.c. This
 * module consumes their published facts and owns only the lifecycle built on top of them.
 *
 *      STOPPED ---forward crank--->  STARTING_FORWARD
 *         ^                                 |
 *         |                        steps + pedal load met
 *         |                                 v
 *         +---grace expired / true stop--  FORWARD
 *         |                                 |
 *         |                        forward pedalling lost
 *         |                                 v
 *         +------------------------------ STOPPING --resumed--> FORWARD
 *
 *      any state --reverse crank step--> REVERSE   (assist blocked immediately)
 *      any state --illegal transition--> INVALID   (assist blocked immediately)
 *      REVERSE / INVALID --inhibit cleared--> STOPPED
 *
 * REVERSE IS ABSOLUTE. In REVERSE (and INVALID) `block_positive` is set, and the pipeline
 * contract is that NOTHING downstream may present a positive request on such a tick: not a
 * held assist_base, not an estimator still decaying, not a slew still running. The current
 * already in the motor is retired by the firmware-owned safety release in the single final Iq
 * owner - that is a CURRENT trajectory, not a demand, and it is bounded and never re-raised.
 */

typedef enum {
	AP2_PAS_STOPPED = 0,
	AP2_PAS_STARTING_FORWARD = 1,
	AP2_PAS_FORWARD = 2,
	AP2_PAS_STOPPING = 3,
	AP2_PAS_REVERSE = 4,
	AP2_PAS_INVALID = 5
} ap2_pas_state_t;

/*
 * How long a lost forward-pedalling signal is treated as STOPPING (a gap the rider may
 * close) rather than as a finished ride.
 *
 * The upstream `forward_valid` fact already carries an adaptive PAS timeout (2x the last
 * forward-transition gap, clamped to 200..500 ms). This grace sits on top of it and exists
 * for exactly one rider-visible case: easing off for one leg and pushing again. Longer than
 * this and a genuine stop would keep the start gate disarmed; shorter and every dead spot at
 * low cadence would demand a fresh start threshold.
 */
#define AP2_PAS_STOP_GRACE_MS 400U

typedef struct {
	/* Forward pedalling is live: cadence present or start phase, and not idle-timed-out.
	 * Published by main.c from the PAS decoder; it carries no direction-safety term. */
	bool forward_valid;
	/* src/pas_direction.c's direction-safety automaton, this tick. A LEVEL, not an edge. */
	bool direction_inhibit;
	/* Diagnostic split of the line above: true = a reverse crank step, false = an illegal
	 * PAS transition. Nothing downstream decides on this; it only names the state. */
	bool inhibit_is_reverse;
	/* Genuine PAS stop (adaptive idle timeout), independent of direction. */
	bool real_stop;
	/* The wheel is demonstrably still turning, so a PAS timeout is a coast, not a stop. */
	bool wheel_valid;
	bool pas_sensor_valid;
	bool torque_sensor_valid;
	/* Brake / critical overtemperature / torque-sensor fault / pedal-load calibration. */
	bool safety_cut;
	/* Assist level 0, or any other reason the rider asked for no assist at all. */
	bool assist_off;

	uint8_t forward_steps;      /* consecutive forward PAS steps (pas_direction) */
	uint8_t required_steps;     /* anti-jiggle guard, from tuning config */
	uint16_t load_centikg;      /* calibrated pedal force, raw per-tick */
	uint16_t engage_load_centikg; /* the threshold in force THIS tick (standstill vs rolling) */

	uint32_t elapsed_ticks;
} ap2_pas_input_t;

typedef struct {
	ap2_pas_state_t state;
	/* The single permission fact. True only in FORWARD. */
	bool assist_permitted;
	/* This tick FORWARD was entered. Consumed by the start envelope. */
	bool engaged_edge;
	/* The engagement that produced `engaged_edge` resumed an interrupted ride (came from
	 * STOPPING) rather than starting a cold one. The start envelope is shorter for it. */
	bool resumed;
	/* Assist must be exactly zero this tick and no downstream state may hold a positive
	 * request: reverse, illegal PAS transition, safety cut or assist off. */
	bool block_positive;
	/* A controlled stop is in progress (the release window). */
	bool stopping;
	/* Remaining STOPPING grace, in 4 kHz ticks. Observation only. */
	uint32_t stop_grace_ticks;
} ap2_pas_output_t;

void ap2_pas_state_reset(void);
void ap2_pas_state_update(const ap2_pas_input_t *in, ap2_pas_output_t *out);
ap2_pas_state_t ap2_pas_state_get(void);

/*
 * Legacy diagnostic mapping. The 0x6028/FW-109 diagnostic byte and several decoders still
 * report a "ride session state" with values 0=COLD, 1=ACTIVE, 2=SUSPENDED_BY_DIRECTION. The
 * new lifecycle is a superset, so it is projected onto those three rather than changing a
 * wire value that shipped decoders parse.
 */
uint8_t ap2_pas_state_legacy_session_code(void);

#endif /* AP2_PAS_STATE_H_ */
