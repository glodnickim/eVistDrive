#ifndef PAS_LIVENESS_H_
#define PAS_LIVENESS_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * FW-112.1: PAS liveness / real-stop separation.
 *
 * One question, asked once: has the crank genuinely stopped producing ANY physical PAS
 * transition? DIRECTION PERMISSION is a different question (pas_direction.c) and must never
 * leak into it.
 *
 * Before FW-112.1, main.c's pas_idle_ticks served as the real-stop timer, and it is refreshed
 * by a forward step OR a reverse step but NOT by an illegal two-bit (INVALID) transition, so a
 * session held in SUSPENDED_BY_DIRECTION by reverse/invalid activity could still be collapsed
 * to COLD by real_stop even though the PAS lines were physically toggling. This module owns a
 * dedicated liveness counter refreshed by ANY qualified physical edge - forward, reverse and
 * invalid alike - and answers the stop question from that counter alone. Reverse/invalid is
 * therefore still never treated as permission or cadence; it is only ever evidence that the
 * crank is still physically moving, so "direction suspended" stays distinct from "real stop".
 */

void pas_liveness_init(void);

/* Call once per control tick, AFTER this tick's stop threshold is known. The module counts
 * idle ticks internally and records this tick's REAL_STOP verdict. */
void pas_liveness_tick(uint16_t stop_timeout_ticks);

/* Call once on every physical PAS transition - forward, reverse and INVALID alike. The crank's
 * encoder lines demonstrably moved; only the direction decoder could not call it safe. */
void pas_liveness_transition(void);

/* This tick's verdict: true when no PAS transition has occurred for longer than the stop
 * threshold - the crank has genuinely stopped. */
bool pas_liveness_stopped(void);

/* Diagnostic readback: ticks since the last physical PAS transition. */
uint32_t pas_liveness_idle_ticks(void);

#endif /* PAS_LIVENESS_H_ */