#include "pas_direction.h"

#include "config.h"   /* BACKWARD_CONFIRM_STEPS / BACKWARD_LATCH_COUNT - see header. */

/* FW-107/109: fwd_run - the consecutive-forward-step counter the cold-start gate reads. Same
 * type, same starting value via static zero-init, same 250 cap. FW-109 v2: now also zeroed by an
 * INVALID step, not just a reverse one - see pas_direction_on_step(). */
static uint8_t fwd_run;

/* FW-109: the PAS direction safety automaton's own state, plus the two per-call outputs a
 * caller needs THIS call only (never latched/historical - see the header). */
static pas_direction_state_t dir_state;
static pas_direction_inhibit_reason_t last_inhibit_reason;
static bool forward_confirmed_last_call;
static uint32_t invalid_count;

/* FW-109 v2: the legacy Backwards_counter/pas_rev_run/pas_rev_run_max/pas_rev_latches
 * derivation - REVERSE-only, unchanged arithmetic, see the header. */
static uint8_t rev_run;
static uint8_t rev_run_max;
static uint8_t backpedal_latch;
static uint16_t backpedal_latches;

void pas_direction_init(void)
{
	fwd_run = 0U;
	dir_state = PAS_DIR_FORWARD_SAFE;
	last_inhibit_reason = PAS_DIR_INHIBIT_REASON_NONE;
	forward_confirmed_last_call = false;
	invalid_count = 0U;
	rev_run = 0U;
	rev_run_max = 0U;
	backpedal_latch = 0U;
	backpedal_latches = 0U;
}

uint8_t pas_direction_on_step(int8_t decoded_dir)
{
	forward_confirmed_last_call = false;

	if (decoded_dir > 0) {
		if (fwd_run < 250U) fwd_run++;
		rev_run = 0U;                              //FW-098: a forward step breaks the reverse run
		if (backpedal_latch) backpedal_latch--;

		switch (dir_state) {
		case PAS_DIR_FORWARD_SAFE:
			/* Already safe - a forward step here proves nothing new. */
			break;
		case PAS_DIR_DIRECTION_INHIBIT:
			/* First forward step after a reverse/invalid begins confirmation. fwd_run was
			 * just zeroed by the event that entered DIRECTION_INHIBIT and has now taken
			 * exactly one forward step, so fwd_run==1 here is the confirm count. */
			dir_state = PAS_DIR_FORWARD_CONFIRMING;
			if (fwd_run >= PAS_REVERSE_RECOVERY_CONFIRM_STEPS) {
				/* PAS_REVERSE_RECOVERY_CONFIRM_STEPS <= 1: a single step already clears it. */
				dir_state = PAS_DIR_FORWARD_SAFE;
				forward_confirmed_last_call = true;
				last_inhibit_reason = PAS_DIR_INHIBIT_REASON_NONE;
			}
			break;
		case PAS_DIR_FORWARD_CONFIRMING:
			if (fwd_run >= PAS_REVERSE_RECOVERY_CONFIRM_STEPS) {
				dir_state = PAS_DIR_FORWARD_SAFE;
				forward_confirmed_last_call = true;
				last_inhibit_reason = PAS_DIR_INHIBIT_REASON_NONE;
			}
			break;
		}
		return PAS_STEP_FORWARD;
	}

	/*
	 * decoded_dir <= 0: REVERSE or INVALID. FW-109 v2: both are "not a valid forward
	 * continuation" and both fail safe IDENTICALLY - zero forward-confirmation progress and
	 * (re)enter DIRECTION_INHIBIT, from ANY prior state including FORWARD_SAFE. This one block
	 * is what makes 1/2/4/7/40/unbounded reverse steps, a reverse arriving mid-confirmation,
	 * AND an illegal transition arriving in any of those places all behave identically - none
	 * of them is a special case, and none of them can leave stale confirmation progress behind
	 * for a later forward step to (wrongly) complete. See the header's TSDZ2 comparison for the
	 * principle this follows.
	 */
	fwd_run = 0U;
	dir_state = PAS_DIR_DIRECTION_INHIBIT;

	if (decoded_dir < 0) {
		last_inhibit_reason = PAS_DIR_INHIBIT_REASON_REVERSE;
		/*
		 * Legacy Backwards_counter/pas_rev_run derivation - REVERSE only, unchanged threshold
		 * arithmetic (FW-098/FW-101). INVALID deliberately does not reach this: it is not
		 * evidence of backpedalling, only a decode glitch, and must not be reported as one to
		 * Extended Boost's crank_reverse or the WSTECZ diagnostic.
		 */
		if (rev_run < 255U) rev_run++;
		if (rev_run > rev_run_max) rev_run_max = rev_run;
		if (rev_run >= BACKWARD_CONFIRM_STEPS) {
			backpedal_latch = BACKWARD_LATCH_COUNT;
			/* FW-101: count SERIES, not steps - only the step that crosses the threshold
			 * counts, so a run of six reports one latch, not four. */
			if (rev_run == BACKWARD_CONFIRM_STEPS && backpedal_latches < 0xFFFFU) backpedal_latches++;
		}
		return PAS_STEP_REVERSE;
	}

	if (invalid_count < 0xFFFFFFFFU) invalid_count++;
	last_inhibit_reason = PAS_DIR_INHIBIT_REASON_INVALID;
	return PAS_STEP_INVALID;
}

void pas_direction_on_stop(void)
{
	fwd_run = 0U;
	/* The crank is genuinely stationary: no reverse/invalid motion is ongoing to inhibit
	 * against. Whether a genuine stop ENDS THE RIDE is the session automaton's own decision
	 * (src/ride_session.c's real_stop input) - independent of this reset. */
	dir_state = PAS_DIR_FORWARD_SAFE;
	last_inhibit_reason = PAS_DIR_INHIBIT_REASON_NONE;
	forward_confirmed_last_call = false;
	/* rev_run/backpedal_latch deliberately untouched here - see pas_direction_clear_backpedal_
	 * latch()'s own, differently-gated production trigger and the header comment. */
}

uint8_t pas_direction_fwd_run(void)
{
	return fwd_run;
}

pas_direction_state_t pas_direction_get_state(void)
{
	return dir_state;
}

bool pas_direction_direction_inhibit_active(void)
{
	/* True in EITHER DIRECTION_INHIBIT or FORWARD_CONFIRMING - direction is not yet PROVEN safe
	 * again until confirmation actually completes (dir_state returns to FORWARD_SAFE). This is
	 * deliberate: ride_control's own final safety gate reads this as its independent backstop
	 * for "Iq must be 0", and that backstop must not depend on ride_session.c's own state
	 * machine correctly keeping the session suspended through the WHOLE confirmation window -
	 * it must hold even if that logic had a bug. An exhaustive host test caught exactly this
	 * gap in an earlier (FW-109 v1) draft that only checked dir_state == the inhibit state
	 * literally: for PAS_REVERSE_RECOVERY_CONFIRM_STEPS > 1 that left the FIRST (N-1)
	 * confirming forward steps reporting "safe" one entire automaton transition before
	 * direction was actually reproven. */
	return dir_state != PAS_DIR_FORWARD_SAFE;
}

pas_direction_inhibit_reason_t pas_direction_last_inhibit_reason(void)
{
	return last_inhibit_reason;
}

bool pas_direction_forward_confirmed_last_call(void)
{
	return forward_confirmed_last_call;
}

uint32_t pas_direction_invalid_count(void)
{
	return invalid_count;
}

bool pas_direction_backpedal_confirmed(void)
{
	return backpedal_latch >= 4U;
}

uint8_t pas_direction_backpedal_count(void)
{
	return backpedal_latch;
}

uint8_t pas_direction_rev_run(void)
{
	return rev_run;
}

uint8_t pas_direction_rev_run_max(void)
{
	return rev_run_max;
}

uint16_t pas_direction_backpedal_latches(void)
{
	return backpedal_latches;
}

void pas_direction_clear_backpedal_latch(void)
{
	backpedal_latch = 0U;
}
