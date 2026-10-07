#ifndef CRANK_PHASE_H_
#define CRANK_PHASE_H_

#include <stdbool.h>
#include <stdint.h>

#include "pas_sampler.h"

/*
 * CRANK PHASE - the production owner of the signed crank step count (ARCHITECTURE_V3.md 3.2,
 * DECISIONS D-006, D-021).
 *
 * WHY A MODULE. Assist V3 measures the pedal stroke in crank angle, so it needs every step the
 * 4 kHz sampler ISR saw, in order. The G53 PAS front end cannot provide that: it replays one held
 * pas_ab per logical tick and loses edges inside a coalesced foreground burst (audit A-D7). The
 * timestamped pas_step_event_t queue drained in main.c already carries every step, so this module
 * accumulates exactly those events. main.c and every harness (SIL, Level-4) call it from their
 * drain loop with the same events, so the simulation matrix exercises this code and not a copy of
 * it (REVIEW 1 #19).
 *
 * WHAT IT OWNS:
 *   steps       signed running count of quadrature steps, +1 forward, -1 reverse (96 per crank
 *               revolution). It wraps; consumers use differences only.
 *   step tick   the 4 kHz timestamp (pas_step_event_t.tick, the ISR clock) of the last counted
 *               step. An INVALID jump is not counted (its direction is unknown), so it does not
 *               move this tick either: a period measured between two ticks always spans the
 *               steps that were counted between them.
 *   glitch      an INVALID two-bit jump or a sampler ring overflow happened since the flag was
 *               last taken. Either one means steps were lost, so the relative crank phase can no
 *               longer be trusted (V3 marks it unaligned, ARCHITECTURE_V3 4.2).
 *
 * It makes no decision and feeds nothing back into the PAS front end, the cadence or the G53
 * chain. Observation only: removing every consumer leaves the firmware behaviour unchanged.
 *
 * Context: main loop / foreground only (the same context that drains the sampler ring).
 */

typedef struct {
	uint32_t forward_steps;   /* counted +1 events since init (wraps)          */
	uint32_t reverse_steps;   /* counted -1 events since init (wraps)          */
	uint32_t invalid_jumps;   /* INVALID two-bit jumps seen since init (wraps) */
	uint32_t overflows;       /* sampler ring overflows reported since init    */
} crank_phase_stats_t;

void crank_phase_init(void);

/* One drained sampler event, in queue order. Call for EVERY event the drain pops. */
void crank_phase_on_event(const pas_step_event_t *ev);

/* The sampler reported a ring overflow (pas_sampler_take_overflow() returned non-zero): ordering
 * was lost, so the step count is short by an unknown number of steps. */
void crank_phase_on_overflow(void);

int32_t crank_phase_steps(void);
uint32_t crank_phase_last_step_tick(void);

/* Returns true once per glitch burst and clears it. Called where the rider snapshot is built, so
 * the flag means "since the previous snapshot". */
bool crank_phase_take_glitch(void);

const crank_phase_stats_t *crank_phase_stats(void);

#endif /* CRANK_PHASE_H_ */
