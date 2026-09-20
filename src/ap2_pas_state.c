#include "ap2_pas_state.h"

#include "ap2_math.h"

/*
 * See inc/ap2_pas_state.h for the state diagram and the ownership boundary. This file holds
 * the automaton and nothing else: no torque conditioning, no demand, no current.
 */

typedef struct {
	ap2_pas_state_t state;
	uint32_t stop_grace_ticks;
	/* The engagement that is about to happen resumes an interrupted ride. Latched when
	 * FORWARD is left for STOPPING and consumed by the next engagement. */
	bool resume_pending;
} ap2_pas_ctx_t;

static ap2_pas_ctx_t ctx;

void ap2_pas_state_reset(void)
{
	ctx.state = AP2_PAS_STOPPED;
	ctx.stop_grace_ticks = 0U;
	ctx.resume_pending = false;
}

ap2_pas_state_t ap2_pas_state_get(void)
{
	return ctx.state;
}

uint8_t ap2_pas_state_legacy_session_code(void)
{
	switch (ctx.state) {
	case AP2_PAS_FORWARD:
	case AP2_PAS_STOPPING:
		return 1U;   /* ACTIVE */
	case AP2_PAS_REVERSE:
	case AP2_PAS_INVALID:
		return 2U;   /* SUSPENDED_BY_DIRECTION */
	default:
		return 0U;   /* COLD */
	}
}

void ap2_pas_state_update(const ap2_pas_input_t *in, ap2_pas_output_t *out)
{
	bool engaged_edge = false;
	bool terminal;
	bool forward_ok;
	bool load_met;
	bool steps_met;

	if (out == 0) {
		return;
	}
	if (in == 0) {
		ap2_pas_state_reset();
		out->state = AP2_PAS_STOPPED;
		out->assist_permitted = false;
		out->engaged_edge = false;
		out->resumed = false;
		out->block_positive = true;
		out->direction_block = false;
		out->stopping = false;
		out->stop_grace_ticks = 0U;
		return;
	}

	/*
	 * TERMINAL CAUSES, evaluated first and independently of the current state. Each of them
	 * is a fact the rider or the hardware asserted; none of them is a consequence of what the
	 * automaton previously decided, so none of them can be missed by getting a transition
	 * wrong. This is the pipeline's default-deny backstop.
	 */
	terminal = in->direction_inhibit || in->safety_cut || in->assist_off ||
		!in->pas_sensor_valid || !in->torque_sensor_valid;

	if (in->direction_inhibit) {
		ctx.state = in->inhibit_is_reverse ? AP2_PAS_REVERSE : AP2_PAS_INVALID;
		ctx.stop_grace_ticks = 0U;
		ctx.resume_pending = false;
	} else if (terminal) {
		/* A safety cut, level 0 or a lost sensor ends the lifecycle outright: the next ride
		 * starts from the full start gate, never from a state that survived the cut. */
		ctx.state = AP2_PAS_STOPPED;
		ctx.stop_grace_ticks = 0U;
		ctx.resume_pending = false;
	} else {
		forward_ok = in->forward_valid;
		steps_met = in->forward_steps >= in->required_steps;
		load_met = (in->engage_load_ctrl == 0U) ||
			(in->load_ctrl >= in->engage_load_ctrl);

		switch (ctx.state) {
		case AP2_PAS_REVERSE:
		case AP2_PAS_INVALID:
			/* The inhibit is gone (checked above), so the hold is over. A fresh ride must
			 * pass the whole start gate again - permission is never restored by the mere
			 * absence of the thing that removed it. */
			ctx.state = AP2_PAS_STOPPED;
			ctx.resume_pending = false;
			break;

		case AP2_PAS_STOPPED:
			if (forward_ok) {
				ctx.state = AP2_PAS_STARTING_FORWARD;
			}
			break;

		case AP2_PAS_STARTING_FORWARD:
			if (!forward_ok) {
				ctx.state = AP2_PAS_STOPPED;
			} else if (steps_met && load_met) {
				ctx.state = AP2_PAS_FORWARD;
				engaged_edge = true;
			}
			break;

		case AP2_PAS_FORWARD:
			if (!forward_ok) {
				ctx.state = AP2_PAS_STOPPING;
				ctx.stop_grace_ticks = AP2_PAS_STOP_GRACE_MS * AP2_TICKS_PER_MS;
				ctx.resume_pending = true;
			}
			break;

		case AP2_PAS_STOPPING:
		default:
			if (in->real_stop && !in->wheel_valid) {
				/* A genuine stop: the cranks timed out AND the bike is not rolling. */
				ctx.state = AP2_PAS_STOPPED;
				ctx.stop_grace_ticks = 0U;
				ctx.resume_pending = false;
			} else if (forward_ok && load_met) {
				/*
				 * The rider closed the gap. Re-engage without the cold start gate: the
				 * step count proved forward intent at the start of THIS ride and the ride
				 * never ended. The load threshold still applies, so a freewheeling crank
				 * cannot re-engage on its own.
				 */
				ctx.state = AP2_PAS_FORWARD;
				engaged_edge = true;
			} else {
				uint32_t used = (in->elapsed_ticks == 0U) ? 1U : in->elapsed_ticks;
				if (used >= ctx.stop_grace_ticks) {
					ctx.stop_grace_ticks = 0U;
					ctx.state = AP2_PAS_STOPPED;
					ctx.resume_pending = false;
				} else {
					ctx.stop_grace_ticks -= used;
				}
			}
			break;
		}
	}

	out->state = ctx.state;
	out->assist_permitted = (ctx.state == AP2_PAS_FORWARD);
	out->engaged_edge = engaged_edge;
	out->resumed = engaged_edge && ctx.resume_pending;
	out->stopping = (ctx.state == AP2_PAS_STOPPING);
	out->stop_grace_ticks = ctx.stop_grace_ticks;
	/*
	 * block_positive is the absolute zero contract. It is true for every cause that must not
	 * be softened by any downstream hold, estimator or ramp - in particular a reverse crank
	 * step, which §PAS of the pipeline contract requires to remove the request in the SAME
	 * tick it is detected.
	 */
	out->block_positive = terminal;
	out->direction_block = in->direction_inhibit;
	if (engaged_edge) {
		ctx.resume_pending = false;
	}
}
