#ifndef PAS_DIRECTION_H_
#define PAS_DIRECTION_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * FW-109 v2: this module is the ONE place in the firmware that interprets a raw quadrature step
 * as forward, reverse or illegal. Nothing else - not main.c, not ride_control.c, not Extended
 * Boost - may re-derive that meaning from the raw decoder independently. This module owns:
 *
 *   (1) fwd_run              consecutive-forward-step counter, the cold-start gate's own input.
 *   (2) the DIRECTION SAFETY AUTOMATON - the one authoritative answer to "is it currently safe to
 *       ask the motor for any current, direction-wise".
 *   (3) the legacy Backwards_counter/pas_rev_run confirmed-backpedal counters (FW-024/FW-098),
 *       DERIVED from the same classified step stream, not read from a second independent
 *       decoder - see part 3 below.
 *
 * WHY A SEPARATE AUTOMATON, NOT MORE CONDITIONS BOLTED ONTO A COUNTER. FW-107/FW-108 each added
 * another special case to ride_control.c's rearm_state to fix one more reverse scenario a real
 * ride log had exposed. Both fixes were correct for the scenario that motivated them and both
 * left the underlying problem in place: "is it safe to drive right now, direction-wise" was never
 * ONE fact with ONE owner, it was reconstructed ad-hoc at every call site from whichever counters
 * happened to be lying around. FW-109 v1 fixed that for REVERSE. FW-109 v2 fixes it for the
 * second gap the same review found: an ILLEGAL quadrature transition (decoded_dir==0, a two-bit
 * jump impossible for real motion) left fwd_run and dir_state completely untouched - so a single
 * decode glitch immediately after a reverse silently preserved FORWARD_CONFIRMING's progress
 * (or, from FORWARD_SAFE, was invisible to this automaton entirely) instead of being treated as
 * "direction is not currently trustworthy", exactly the failure mode a corrupted/ungrounded PAS
 * signal produces.
 *
 * PRINCIPLE CHECKED AGAINST A REFERENCE DESIGN (TSDZ2-Smart-EBike-860C, src/motor.c's cadence
 * decoder, GPL-3.0 - reviewed for principle only, no code taken). That decoder tracks one known-
 * good forward ring of raw states; ANY transition that is not the next state in that ring -
 * including a genuine backward step - resets its reference to "unsynchronised" and demands a
 * fresh run of valid forward transitions before it trusts cadence again. It does not maintain two
 * different reactions for "backward" versus "desynchronised"; both collapse to one fail class.
 * This module adopts that principle for its SAFETY reaction (REVERSE and INVALID both
 * unconditionally zero fwd_run and (re)enter DIRECTION_INHIBIT - see pas_direction_on_step()) while
 * still keeping a separate, orthogonal DIAGNOSTIC cause (pas_direction_last_inhibit_reason()) for
 * the two, because this controller - unlike that reference decoder - has other consumers
 * (Extended Boost, the WSTECZ diagnostic) that care specifically whether the crank is being
 * pedalled backward, not merely "direction is not currently trusted".
 *
 * STATES. Iq must be 0 in BOTH DIRECTION_INHIBIT and FORWARD_CONFIRMING - direction is not proven
 * safe again until confirmation actually completes - enforced by ride_control's own final gate
 * (pas_direction_direction_inhibit_active(), true in either state), not by this module, which only
 * ever REPORTS the fact.
 *   FORWARD_SAFE        no reverse/invalid in progress; direction is trusted.
 *   DIRECTION_INHIBIT   a reverse step OR an illegal transition happened and has not yet been
 *                       cleared. Entered on the FIRST such event, no confirmation needed - this
 *                       is a safety inhibit, not a measurement. (Renamed from FW-109 v1's
 *                       REVERSE_INHIBIT: it is now entered by two distinct causes, not one - see
 *                       pas_direction_last_inhibit_reason() for which.)
 *   FORWARD_CONFIRMING  direction is being re-proven. PAS_REVERSE_RECOVERY_CONFIRM_STEPS
 *                       consecutive forward steps clear it back to FORWARD_SAFE; any REVERSE or
 *                       INVALID step during confirmation drops straight back to
 *                       DIRECTION_INHIBIT, discarding all confirmation progress - no path exists
 *                       where forward steps from before that event survive to be counted after
 *                       it, because fwd_run itself (the thing being counted) is zeroed by the
 *                       same event.
 *
 * NOT THE SAME MECHANISM AS the legacy Backwards_counter/BACKWARD_CONFIRM_STEPS this module now
 * DERIVES (see pas_direction_backpedal_confirmed() etc. below). That pair exists to tell
 * deliberate backpedalling apart from crank dead-spot jitter for OTHER consumers (Extended
 * Boost's crank_reverse input, the WSTECZ diagnostic flags, the ratio metrics) and still does
 * exactly that, unchanged in every threshold and every tick of timing - only WHERE the counting
 * happens moved (from main.c, reading the raw decoder a second time, to here, reading this
 * module's own classification of the same step). It requires 3 CONSECUTIVE reverse steps before
 * it latches, is deliberately slower/stickier than the safety automaton above, and - preserving
 * its exact FW-024/FW-098 behaviour - is NOT fed by INVALID steps: an illegal transition is a
 * decode glitch, not evidence of backpedalling, and must not be reported as one.
 */

/* PAS_STEP_NONE/FORWARD/REVERSE/INVALID - what pas_direction_on_step() just classified this
 * call as. INVALID: an illegal two-bit quadrature jump, fed to this module (rather than silently
 * dropped by the caller) so it can be counted diagnostically and so "an illegal transition is not
 * a valid forward step" lives as ONE rule inside this module. */
#define PAS_STEP_NONE    0U
#define PAS_STEP_FORWARD 1U
#define PAS_STEP_REVERSE 2U
#define PAS_STEP_INVALID 3U

typedef enum {
	PAS_DIR_FORWARD_SAFE       = 0,
	PAS_DIR_DIRECTION_INHIBIT  = 1,
	PAS_DIR_FORWARD_CONFIRMING = 2
} pas_direction_state_t;

/* Diagnostic-only: WHY direction is not currently trusted (or was last untrusted). Never read by
 * any control-flow decision in this module, ride_session.c or ride_control.c - the safety
 * automaton's own state (pas_direction_direction_inhibit_active()) is the only fact those act on,
 * and it reacts identically regardless of cause. This exists solely so a log/CAN diagnostic can
 * still tell "the rider backpedalled" apart from "the decoder saw a glitch" - see the header's
 * TSDZ2 comparison for why the two are unified everywhere EXCEPT here. Invariant: this reads NONE
 * if and only if pas_direction_get_state() == PAS_DIR_FORWARD_SAFE. */
typedef enum {
	PAS_DIR_INHIBIT_REASON_NONE    = 0,
	PAS_DIR_INHIBIT_REASON_REVERSE = 1,
	PAS_DIR_INHIBIT_REASON_INVALID = 2
} pas_direction_inhibit_reason_t;

/*
 * How many consecutive CONFIRMED forward steps clear DIRECTION_INHIBIT once entered.
 *
 * Deliberately its OWN named constant, not tuning_config_start_steps() (the rider-configurable
 * cold-start step count) and not BACKWARD_CONFIRM_STEPS (this module's own, slower, 3-step
 * deliberate-backpedal confirm - see above). Using either of those here would make direction
 * safety's recovery time an accidental side effect of a knob owned for a different purpose -
 * exactly what this card exists to stop.
 *
 * Value: 2, unchanged from FW-107/109 v1 - cheaper than a full cold start (which has no prior
 * context and must prove itself from nothing), because a ride already under way a moment ago
 * does not need to reprove itself from scratch. Two clean forward steps is also enough to rule
 * out the single/double-step dead-spot rocking a ride log showed accounts for most reverse events
 * at 16-52 rpm (FW-098's own measurement: 46 single-step runs, 22 two-step, only 12 runs of three
 * or more) - a genuine recovery clears it in two steps; residual dead-spot jitter cannot, because
 * any further reverse OR invalid step during confirmation resets the count to zero.
 */
#define PAS_REVERSE_RECOVERY_CONFIRM_STEPS 2U

void pas_direction_init(void);

/*
 * Feed the SIGNED direction from pas_quadrature_step() for EVERY detected transition
 * (new_state != previous_state) - including illegal ones (decoded_dir==0). Do not call this for
 * a tick with no transition at all; use pas_direction_on_stop() for "crank genuinely stopped".
 *
 * Drives: (1) fwd_run, the consecutive-forward-step counter the cold-start gate reads - increments
 * on forward (capped at 250), zeroed on EITHER a reverse OR an invalid step (FW-109 v2: INVALID no
 * longer spared - see the header); (2) the PAS direction safety automaton; (3) the legacy
 * Backwards_counter/pas_rev_run derivation, REVERSE-only, unchanged timing. Returns which event
 * this call represents.
 */
uint8_t pas_direction_on_step(int8_t decoded_dir);

/*
 * Call on the EXISTING crank-stop condition (pas_idle_ticks > pas_stop_timeout in main.c).
 * Resets fwd_run and the direction automaton to FORWARD_SAFE: with the crank genuinely
 * stationary there is no ongoing reverse/invalid motion left to inhibit against - the ride
 * SESSION automaton (src/ride_session.c) is what decides whether a genuine stop ends the ride,
 * independently of this reset (see real_stop there - never inferred from this call).
 *
 * Deliberately does NOT touch the legacy backpedal-confirm latch: that reset has its own,
 * differently-gated production trigger (main.c, unchanged since FW-024b) - see
 * pas_direction_clear_backpedal_latch() below.
 */
void pas_direction_on_stop(void);

uint8_t pas_direction_fwd_run(void);

/* --- PAS direction safety automaton: observation only, see the state table above ------------ */
pas_direction_state_t pas_direction_get_state(void);
/*
 * True whenever direction has NOT been proven safe again - dir_state is DIRECTION_INHIBIT OR
 * FORWARD_CONFIRMING, i.e. dir_state != FORWARD_SAFE. Deliberately covers the WHOLE recovery
 * window, not just the initial inhibit: this is the one fact ride_control's final safety gate
 * checks (src/ride_control.c), and that gate's whole point is to hold even if the ride session
 * automaton's own logic has a bug - it must not depend on FORWARD_CONFIRMING being separately
 * kept safe by that other module. (Renamed from FW-109 v1's pas_direction_reverse_inhibit_active:
 * the fact it reports is now general - "direction untrusted, for either reason" - not
 * reverse-specific.)
 */
bool pas_direction_direction_inhibit_active(void);
/* Diagnostic cause of the CURRENT (or, while FORWARD_SAFE, the most recently cleared) inhibit -
 * see the type's own comment for the invariant tying it to dir_state. */
pas_direction_inhibit_reason_t pas_direction_last_inhibit_reason(void);
/* True only on the exact pas_direction_on_step() call whose forward step was the Nth confirming
 * one (FORWARD_CONFIRMING -> FORWARD_SAFE fired on THIS call). Never latched/historical - a
 * caller that does not consume it the same tick has lost it. */
bool pas_direction_forward_confirmed_last_call(void);
/* Lifetime count of illegal (two-bit-jump) quadrature transitions - diagnostic only, never read
 * by any decision. Saturates rather than wrapping. */
uint32_t pas_direction_invalid_count(void);

/*
 * --- legacy Backwards_counter/pas_rev_run derivation (FW-024/FW-098/FW-109 v2) ----------------
 * Same names, same arithmetic, same consumers as the main.c globals this replaces - only the
 * ownership moved. See BACKWARD_CONFIRM_STEPS/BACKWARD_LATCH_COUNT in config.h for the constants.
 */
/* True once BACKWARD_LATCH_COUNT has latched and has not yet decayed below 4 - the exact
 * Backwards_counter>=4 test every consumer (Extended Boost's crank_reverse, the WSTECZ diagnostic
 * flags) used to perform on the raw counter. */
bool pas_direction_backpedal_confirmed(void);
/* The raw 0..BACKWARD_LATCH_COUNT counter value, for the CAN diagnostic byte that used to
 * transmit Backwards_counter directly. */
uint8_t pas_direction_backpedal_count(void);
/* Current unbroken run of consecutive REVERSE steps (mirrors the old pas_rev_run). */
uint8_t pas_direction_rev_run(void);
/* Longest such run seen this session (mirrors the old pas_rev_run_max). */
uint8_t pas_direction_rev_run_max(void);
/* Count of runs that crossed BACKWARD_CONFIRM_STEPS (mirrors the old pas_rev_latches). */
uint16_t pas_direction_backpedal_latches(void);
/*
 * Clear the backpedal-confirm latch (only - fwd_run/dir_state/rev_run untouched). Call from
 * main.c's own, differently-gated real-stop-and-no-torque cleanup (unchanged since FW-024b:
 * torque_counter>4000 && PAS_counter>MP.PAS_timeout && pas_idle_ticks>pas_stop_timeout) - this is
 * a stricter, later condition than pas_direction_on_stop()'s, and always was; the two resets are
 * deliberately not merged; see main.c for the exact call site this replaces.
 */
void pas_direction_clear_backpedal_latch(void);

#endif /* PAS_DIRECTION_H_ */
