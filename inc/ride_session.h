#ifndef RIDE_SESSION_H_
#define RIDE_SESSION_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * FW-112 v2: the ride SESSION automaton - the single, authoritative owner of whether assist
 * current is PERMITTED to flow right now, and of how a ride gets back into that state after a
 * direction inhibit.
 *
 * Deliberately free of MS/MP and of every other project type (rider_input_t, ride_control_
 * input_t, assist_mode_output_t): every input it needs arrives through ride_session_input_t,
 * built by the caller (src/ride_control.c) from whatever sources those facts actually live in.
 * This is what lets a host test link and drive the REAL module directly, with no risk of a
 * hand-copied mirror silently drifting from production.
 *
 * FW-112 v2: PERMISSION, NOT DEMAND.
 *
 * v1 (and the FW-109 two-phase commit that preceded it) made this module try to GUARANTEE that a
 * resumed ride only flowed under "fresh pressure AND a real, positive Iq demand": a
 * WAIT_REARM_LOAD state held ACTIVE hostage until such a demand was observed, and a separate
 * ride_session_commit_rearm() call closed the loop. The price was the exact coupling the FW-112
 * brief rejected: the automaton was making a demand-availability decision (needs an Iq value, a
 * pressure sample, filtered thresholds) that properly belongs to the per-tick current
 * calculation, and it carried a latching "rearm entitlement" that had to be re-validated against
 * conditions this module could not even see.
 *
 * v2 splits the two concepts cleanly:
 *   - PERMISSION is owned HERE: may the pipeline even ASK for current. It is a pure direction
 *     fact. A suspension (reverse or invalid step) drops it; the exact tick the PAS direction
 *     automaton re-confirms forward - and only then - restores it, with NO pressure/torque
 *     condition and NO waiting for an Iq demand. The caller reads out->latched /
 *     out->fast_rearm_this_tick and, for exactly that one tick, grants "treat this rider as
 *     pedalling" to its assist-mode calculation (see src/ride_control.c).
 *   - DEMAND is owned by the caller (ride_control.c -> assist_modes_calculate), evaluated FRESH
 *     EVERY TICK regardless of latch state, exactly as it always has been. Because there is no
 *     stored pre-reverse demand to restore and no cross-tick entitlement, the invariant "no
 *     current unless the current calculation itself asks for it" holds by construction on every
 *     tick: a deadband, a ceiling, zero load or any other decline simply yields 0 that tick, and
 *     the caller's own final gate zeroes anything unlatched anyway.
 *
 * There is therefore NO WAIT_REARM_LOAD in production: SUSPENDED_BY_DIRECTION goes straight back
 * to ACTIVE on the confirm tick. RIDE_SESSION_WAIT_REARM_LOAD survives in the enum ONLY as a
 * RESERVED legacy value (the diagnostic schema still encodes the old numeric state 3) - no
 * production transition ever sets it, and ride_session_update() defensively normalizes it (and
 * any other unrecognized value) to COLD so a corrupted state can never grant a latch.
 *
 * See the transition table in ride_session_update()'s own comments for the exact rule fired by
 * each input in each state - it is the literal implementation of the owner's specification, not
 * a paraphrase of it.
 */

typedef enum {
	RIDE_SESSION_COLD                   = 0,
	RIDE_SESSION_ACTIVE                 = 1,
	RIDE_SESSION_SUSPENDED_BY_DIRECTION = 2,
	/* RESERVED: the FW-112 v1 / FW-109 "waiting for rearm load" state. Never entered by
	 * production code in v2 - kept only so older diagnostic decoders still recognise the
	 * numeric value. ride_session_update() normalizes it (and anything else unrecognized)
	 * to COLD. */
	RIDE_SESSION_WAIT_REARM_LOAD        = 3
} ride_session_state_t;

typedef struct {
	/*
	 * --- direction safety (src/pas_direction.c's automaton, read by the caller) -------------
	 * direction_inhibit_active is a LEVEL, not an edge: true for every tick a reverse OR
	 * invalid-step hold lasts, however long. This is deliberate - it is what lets ONE rule
	 * ("while this is true, a session may not be ACTIVE") cover 1, 2, 4, 7, 40 or unbounded
	 * reverse steps, any number of illegal transitions, and any interleaving of the two,
	 * identically, with no separate "how many / which kind so far" case anywhere in this module.
	 */
	bool direction_inhibit_active;
	/* True only on the tick the direction automaton's confirmation just completed (edge, not
	 * level) - src/pas_direction.c's pas_direction_forward_confirmed_last_call(). This edge is
	 * the ONLY thing that restores permission after a direction suspension - FW-112 v2 grants
	 * the rearm on direction alone, with no pressure/torque condition and no waiting for an Iq
	 * demand (see the file header). */
	bool forward_confirmed_this_tick;

	/*
	 * --- FW-112.2: rolling-coast retention (REAL_STOP vs ROLLING COAST) -------------------
	 * rolling_valid: is the wheel demonstrably rolling right now - ride_wheel_valid(), the
	 * production SPEED_STOP_TICKS freshness of the accepted wheel edge (see inc/ride_wheel.h).
	 * A PAS timeout while rolling_valid holds is a COAST, not a true stop: the session is
	 * retained (SUSPENDED, never latched) so a genuine stop's cold-start steps are not
	 * demanded mid-roll, and the resume below does not need them.
	 *
	 * forward_pedaling: genuine forward crank evidence this tick (the caller's validated
	 * crank_direction_ok: forward and not reversed and not idle-timed-out) - the SECOND, equal
	 * legitimate path back to ACTIVE after a rolling coast, alongside forward_confirmed_this_tick.
	 * During a forward coast the direction automaton stays FORWARD_SAFE, so NO confirm edge ever
	 * fires; forward_pedaling re-arms the instant the rider genuinely resumes. It can never re-arm
	 * inside a direction hold, because the caller only sets it when the crank is moving forward.
	 */
	bool rolling_valid;
	bool forward_pedaling;

	/*
	 * --- session-ending (terminal) inputs, jointly and ONLY authoritative for ending a
	 * session outright - see the module comment for the exact production sources of each. ----
	 */
	bool non_direction_safety_cut; /* brake / overtemp cutoff / torque-sensor fault / calibration */
	bool assist_off;               /* assist level 0 */
	/* FW-112.2: QUALIFIED by the wheel. real_stop is only terminal when the wheel is no longer
	 * rolling (real_stop && !rolling_valid); while the wheel is still fresh, real_stop retains
	 * the session in SUSPENDED_BY_DIRECTION (a coast), so a true stop - PAS stopped AND wheel
	 * invalid after the conservative timeout - still ends the ride outright. */
	bool real_stop;

	/*
	 * --- COLD -> ACTIVE, the ordinary cold start. UNCHANGED conditions, resolved by the
	 * caller exactly as before this card - this module only asks "were they met this tick". ---
	 */
	bool cold_start_ready;
} ride_session_input_t;

typedef struct {
	ride_session_state_t state;
	bool latched;              /* THE answer: may assist current flow this tick */
	bool fast_rearm_this_tick; /* true only on the exact tick ACTIVE is re-entered from
	                             * SUSPENDED_BY_DIRECTION - drives the caller's assist_modes
	                             * resumption grant bookkeeping and the diagnostic fast_rearm
	                             * flag, nothing else. */
	bool cold_arm_this_tick;   /* true only on the exact tick ACTIVE is entered via COLD - the
	                             * ordinary, unchanged start. */
} ride_session_output_t;

void ride_session_init(void);
void ride_session_update(const ride_session_input_t *in, ride_session_output_t *out);
ride_session_state_t ride_session_get_state(void);

/*
 * Force COLD immediately - for the caller's own service-mode bypass paths (Walk Assist, position
 * calibration) that skip the normal ride-latch block entirely. A suspended or active session
 * must not survive a detour through one of those and fire on the way out.
 */
void ride_session_force_cold(void);

#endif /* RIDE_SESSION_H_ */
