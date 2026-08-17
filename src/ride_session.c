#include "ride_session.h"

static ride_session_state_t state;

void ride_session_init(void)
{
	state = RIDE_SESSION_COLD;
}

ride_session_state_t ride_session_get_state(void)
{
	return state;
}

void ride_session_force_cold(void)
{
	state = RIDE_SESSION_COLD;
}

/*
 * FW-112 v2: one function, one decision per tick. The literal transition table (state x event
 * -> next state), FW-112.2-qualified:
 *
 *   COLD:
 *     cold_start_ready (and no terminal event)               -> ACTIVE, cold_arm_this_tick
 *     (direction_inhibit_active has NO effect here - fwd_run already keeps cold_start_ready
 *      from being met while a reverse/invalid is recent, so nothing extra is needed)
 *
 *   ACTIVE:
 *     terminal (non_direction_safety_cut || assist_off ||
 *              (real_stop && !rolling_valid))                -> COLD
 *     else direction_inhibit_active                          -> SUSPENDED_BY_DIRECTION
 *     else real_stop && rolling_valid                        -> SUSPENDED_BY_DIRECTION
 *         (FW-112.2 ROLLING COAST: the rider stopped pedalling but the wheel is still fresh.
 *          The session is RETAINED in the existing SUSPENDED state - never latched, so the
 *          coast can never generate assist - and does not demand a cold start on resume.)
 *
 *   SUSPENDED_BY_DIRECTION:
 *     terminal (as above, qualified)                          -> COLD
 *     else !direction_inhibit_active &&
 *          (forward_confirmed_this_tick || forward_pedaling)  -> ACTIVE, fast_rearm_this_tick.
 *         Permission is a pure direction/forward fact - FW-112 v2 restores it on the confirm
 *         edge, and FW-112.2 adds the equal forward_pedaling path for a coast resume (a
 *         forward coast leaves the direction automaton FORWARD_SAFE, so no confirm edge ever
 *         fires there). NO pressure/torque condition and NO waiting for an Iq demand; see the
 *         file header for why demand needs no gate here in v2 (it is evaluated fresh every tick
 *         by the caller, so a decline just yields 0 that tick).
 *     else                                                      stays SUSPENDED
 *
 * Terminal events are checked FIRST in every non-COLD state and unconditionally win over a
 * simultaneous direction confirm or inhibit - "jeżeli reverse/invalid i brake wystąpią razem,
 * wygrywa non_direction_safety_cut".
 *
 * RESERVED / corrupted states: RIDE_SESSION_WAIT_REARM_LOAD (=3) is never entered by production
 * (kept only for legacy diag decoders); any state outside the three real ones - the reserved
 * value or memory corruption - is defensively normalized to COLD at the top, so it can never
 * grant a latch.
 */
void ride_session_update(const ride_session_input_t *in, ride_session_output_t *out)
{
	out->fast_rearm_this_tick = false;
	out->cold_arm_this_tick = false;

	if (state != RIDE_SESSION_COLD && state != RIDE_SESSION_ACTIVE &&
	    state != RIDE_SESSION_SUSPENDED_BY_DIRECTION) {
		state = RIDE_SESSION_COLD;
	}

	/* FW-112.2: a PAS timeout is only terminal when the wheel has also lost its conservative
	 * freshness - real_stop alone (while rolling_valid holds) is a COAST and retains the
	 * session. non_direction_safety_cut / assist_off remain unconditionally terminal. */
	bool terminal = in->non_direction_safety_cut || in->assist_off ||
	                (in->real_stop && !in->rolling_valid);

	switch (state) {
	case RIDE_SESSION_COLD:
		if (!terminal && in->cold_start_ready) {
			state = RIDE_SESSION_ACTIVE;
			out->cold_arm_this_tick = true;
		}
		break;

	case RIDE_SESSION_ACTIVE:
		if (terminal) {
			state = RIDE_SESSION_COLD;
		} else if (in->direction_inhibit_active) {
			state = RIDE_SESSION_SUSPENDED_BY_DIRECTION;
		} else if (in->real_stop && in->rolling_valid) {
			state = RIDE_SESSION_SUSPENDED_BY_DIRECTION;
		}
		break;

	case RIDE_SESSION_SUSPENDED_BY_DIRECTION:
		if (terminal) {
			state = RIDE_SESSION_COLD;
		} else if (!in->direction_inhibit_active &&
		           (in->forward_confirmed_this_tick || in->forward_pedaling)) {
			state = RIDE_SESSION_ACTIVE;
			out->fast_rearm_this_tick = true;
		}
		/* else: direction_inhibit_active still true, or the rider is not genuinely pedalling
		 * forward again - stays SUSPENDED_BY_DIRECTION. */
		break;

	default:
		/* unreachable after the normalization above; kept so an unrecognized enum value can
		 * never fall through to grant a latch */
		state = RIDE_SESSION_COLD;
		break;
	}

	out->state = state;
	out->latched = (state == RIDE_SESSION_ACTIVE);
}