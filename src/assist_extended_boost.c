#include "assist_extended_boost.h"

#include "torque_input.h"

/*
 * FW-084: see the header for what this is and what it deliberately is not.
 *
 * The module owns NO global state of its own beyond the state machine below, takes its
 * whole world through assist_extended_boost_input_t and never touches MS/MP — so the same
 * code can be exercised at 4 kHz on a PC (tests/fw084_extended_boost.js).
 */

#define EXT_BOOST_CONFIRM_TICKS \
	(EXT_BOOST_CONFIRM_MS * EXT_BOOST_CONTROL_TICKS_PER_MS)

static assist_extended_boost_state_t state;
/* Peak of the window currently being qualified, and the peak the running boost was computed
 * from. Kept separate so a later spike cannot silently change the current of a boost that is
 * already paying out. */
static uint16_t candidate_peak_centikg;
static uint16_t fired_peak_centikg;
static uint16_t confirm_ticks;
static uint32_t active_ticks_left;
static bool window_open;
/*
 * Set when a boost ends while the rider is STILL leaning on the pedal. Without it, a push held
 * above the trigger would satisfy the confirm test again on the very next tick and restart the
 * boost for ever — the duration would bound nothing. Cleared only when the load falls back
 * below the trigger by the hysteresis, i.e. when the push genuinely ends.
 */
static bool rearm_blocked;
static int32_t boost_iq;
static uint8_t last_cancel_reason;
static uint8_t last_bank_index;
static uint8_t last_level_index;
static bool have_context;

static void clear_state(void)
{
	state = ASSIST_EXT_BOOST_IDLE;
	candidate_peak_centikg = 0;
	fired_peak_centikg = 0;
	confirm_ticks = 0;
	active_ticks_left = 0;
	window_open = false;
	boost_iq = 0;
}

void assist_extended_boost_init(void)
{
	clear_state();
	rearm_blocked = false;
	last_cancel_reason = ASSIST_EXT_BOOST_CANCEL_NONE;
	last_bank_index = 0;
	last_level_index = 0;
	have_context = false;
}

void assist_extended_boost_reset(uint8_t reason)
{
	clear_state();
	if (reason != ASSIST_EXT_BOOST_CANCEL_NONE) {
		last_cancel_reason = reason;
	}
}

static uint16_t valid_trigger_centikg(uint16_t configured)
{
	if (configured < ASSIST_EXT_BOOST_TRIGGER_MIN_CENTIKG) {
		return ASSIST_EXT_BOOST_TRIGGER_MIN_CENTIKG;
	}
	if (configured > ASSIST_EXT_BOOST_TRIGGER_MAX_CENTIKG) {
		return ASSIST_EXT_BOOST_TRIGGER_MAX_CENTIKG;
	}
	return configured;
}

static uint16_t valid_duration_ms(uint16_t configured)
{
	return (configured > ASSIST_EXT_BOOST_DURATION_MAX_MS) ?
		ASSIST_EXT_BOOST_DURATION_MAX_MS : configured;
}

/*
 * Linear map of the part of the pedal load that sits ABOVE the trigger, onto the current
 * limit of the active level. Touching the threshold arms the function but yields almost no
 * current; a harder push yields more. strength_pct may raise the result above the load map,
 * never above the level's own limit — and every shared limit still runs after this.
 *
 * The trigger may be set to full scale, which makes span zero. The early return below
 * covers it: peak is clamped to full scale first, so peak <= trigger holds and the function
 * is out before the division. Do not reorder those two.
 */
static int32_t compute_boost_iq(
	uint16_t peak_centikg,
	uint16_t trigger_centikg,
	uint8_t strength_pct,
	int32_t iq_limit)
{
	if (iq_limit <= 0 || strength_pct == 0) {
		return 0;
	}
	uint32_t peak = (peak_centikg > TORQUE_PUBLIC_FULL_SCALE_CENTIKG) ?
		TORQUE_PUBLIC_FULL_SCALE_CENTIKG : peak_centikg;
	if (peak <= trigger_centikg) {
		/* Also the zero-span guard: a trigger at full scale can never be exceeded. */
		return 0;
	}
	uint32_t span = (uint32_t)TORQUE_PUBLIC_FULL_SCALE_CENTIKG - trigger_centikg;
	uint32_t above = peak - trigger_centikg;
	/* 32-bit intermediates throughout: iq_limit <= 32767 and above <= 6000, so the
	 * product stays two orders of magnitude below the signed 32-bit ceiling. */
	uint32_t base_iq = ((uint32_t)iq_limit * above + span / 2U) / span;
	uint32_t scaled = (base_iq * strength_pct + 50U) / 100U;
	if (scaled > (uint32_t)iq_limit) {
		scaled = (uint32_t)iq_limit;
	}
	return (int32_t)scaled;
}

/*
 * A qualifying window is one CONTINUOUS push above the threshold. It opens on the threshold,
 * keeps its own peak, and closes half a kilogram below — the hysteresis stabilizes the END of
 * a push and never moves the threshold the rider set.
 *
 * Returns true on the tick the window becomes confirmed, i.e. the tick a boost may start.
 * FW-095: confirmation IS the start. There is no waiting state in between, because the thing
 * it used to wait for was the rider stopping pedalling.
 */
static bool qualify_push(
	const assist_extended_boost_input_t *input,
	uint16_t trigger_centikg)
{
	bool may_qualify = input->pedaling_active && input->pedal_assist_latched;
	uint16_t load = input->pedal_load_centikg;

	if (may_qualify && load >= trigger_centikg) {
		if (!window_open) {
			window_open = true;
			confirm_ticks = 0;
			candidate_peak_centikg = 0;
			if (state == ASSIST_EXT_BOOST_IDLE) {
				state = ASSIST_EXT_BOOST_QUALIFY;
			}
		}
		if (load > candidate_peak_centikg) {
			candidate_peak_centikg = load;
		}
		if (rearm_blocked) {
			/* This push already paid out. Track its peak so the diagnostics stay
			 * honest, but do not let it start another boost. */
			return false;
		}
		if (confirm_ticks < EXT_BOOST_CONFIRM_TICKS) {
			confirm_ticks++;
		}
		return confirm_ticks >= EXT_BOOST_CONFIRM_TICKS;
	}

	/*
	 * Below the threshold, or no longer entitled to qualify. Closing the window is also what
	 * clears rearm_blocked: the rider has to ease off and push again to get another boost.
	 */
	bool closed = !may_qualify ||
		(load + EXT_BOOST_RELEASE_HYST_CENTIKG) < trigger_centikg;
	if (closed) {
		window_open = false;
		confirm_ticks = 0;
		candidate_peak_centikg = 0;
		rearm_blocked = false;
		if (state == ASSIST_EXT_BOOST_QUALIFY) {
			state = ASSIST_EXT_BOOST_IDLE;
		}
	}
	return false;
}

void assist_extended_boost_update(
	const assist_extended_boost_input_t *input,
	const assist_extended_boost_config_t *config,
	assist_extended_boost_output_t *output)
{
	if (output != 0) {
		output->iq_target = 0;
		output->active = false;
	}
	if (input == 0 || config == 0) {
		assist_extended_boost_reset(ASSIST_EXT_BOOST_CANCEL_NONE);
		return;
	}

	uint16_t trigger_centikg = valid_trigger_centikg(config->trigger_load_centikg);
	uint16_t duration_ms = valid_duration_ms(config->duration_ms);
	bool disabled = (duration_ms == 0) || (config->strength_pct == 0);

	/*
	 * Every cancelling condition acts in the SAME control tick, before anything else can
	 * look at the state. Order matters only for which reason ends up in the log.
	 */
	uint8_t cancel = ASSIST_EXT_BOOST_CANCEL_NONE;
	if (disabled) {
		cancel = ASSIST_EXT_BOOST_CANCEL_DISABLED;
	} else if (input->safety_cut) {
		cancel = ASSIST_EXT_BOOST_CANCEL_SAFETY_CUT;
	} else if (input->crank_reverse) {
		cancel = ASSIST_EXT_BOOST_CANCEL_REVERSE;
	} else if (!input->torque_sensor_valid || !input->pas_sensor_valid) {
		cancel = ASSIST_EXT_BOOST_CANCEL_SENSOR_INVALID;
	} else if (input->walk_active) {
		cancel = ASSIST_EXT_BOOST_CANCEL_WALK;
	} else if (input->position_calibration_active) {
		cancel = ASSIST_EXT_BOOST_CANCEL_CALIBRATION;
	} else if (input->level_index == 0) {
		cancel = ASSIST_EXT_BOOST_CANCEL_LEVEL_OR_BANK_CHANGE;
	} else if (have_context && (input->bank_index != last_bank_index ||
		input->level_index != last_level_index)) {
		cancel = ASSIST_EXT_BOOST_CANCEL_LEVEL_OR_BANK_CHANGE;
	} else if (state == ASSIST_EXT_BOOST_ACTIVE && !input->motion_valid) {
		cancel = ASSIST_EXT_BOOST_CANCEL_MOTION_LOST;
	} else if (!input->pedaling_active) {
		/*
		 * THE SAFETY RULE OF THIS MODULE (FW-095).
		 *
		 * Real forward pedalling has stopped, so the boost stops — in this tick, before
		 * anything below can look at the state, and whatever the timer still holds. The
		 * module reports zero from here on and ride_control's ordinary release fade owns
		 * the way down, exactly as it would without this feature.
		 *
		 * This is unconditional on purpose. It is not "unless the timer is nearly done",
		 * not "unless the load is still high", and it must never become either: the M820
		 * has no brake-sensor input we can rely on, so anything that kept the motor
		 * pulling here would be motor overrun with no independent way to stop it.
		 *
		 * The predecessor (FW-084) did the exact opposite — this edge was what STARTED
		 * its boost. See the header.
		 */
		cancel = ASSIST_EXT_BOOST_CANCEL_PEDALING_STOPPED;
	} else if (!input->pedal_assist_latched) {
		/* The latch is the proof that assist started legally. Losing it mid-boost means
		 * the start conditions no longer hold, so neither does the boost. */
		cancel = ASSIST_EXT_BOOST_CANCEL_PEDALING_STOPPED;
	}

	last_bank_index = input->bank_index;
	last_level_index = input->level_index;
	have_context = true;

	if (cancel != ASSIST_EXT_BOOST_CANCEL_NONE) {
		/*
		 * ONE PUSH, ONE BOOST — including when something cuts the boost short.
		 *
		 * A boost that reached ACTIVE has already paid out, so it blocks re-arming whatever
		 * ended it. The case that made this necessary: the rider stops the cranks with full
		 * weight still on the pedal. PAS STOP correctly cancels the boost, but the push
		 * itself never ended — so on resuming the cranks the same unbroken press would
		 * confirm a fresh window 30 ms later and hand out a second boost. Not a post-PAS
		 * safety hole (the second boost still needs live pedalling), but it contradicts what
		 * the rider is told, and it lets one press pay out repeatedly across a stop-start.
		 *
		 * A cancel during QUALIFY blocks nothing: that push never produced any current.
		 *
		 * The block is cleared in exactly one place — qualify_push(), when the load finally
		 * drops EXT_BOOST_RELEASE_HYST_CENTIKG below the trigger.
		 */
		bool blocked = rearm_blocked || (state == ASSIST_EXT_BOOST_ACTIVE);
		assist_extended_boost_reset(cancel);
		rearm_blocked = blocked;
		return;
	}

	if (state != ASSIST_EXT_BOOST_ACTIVE) {
		/*
		 * Confirmation starts the boost immediately, while the rider is pushing and still
		 * pedalling. motion_valid is required here and re-checked every tick above.
		 */
		if (qualify_push(input, trigger_centikg) && input->motion_valid) {
			int32_t candidate = compute_boost_iq(
				candidate_peak_centikg,
				trigger_centikg,
				config->strength_pct,
				input->ride_core_iq_limit);
			if (candidate > 0) {
				boost_iq = candidate;
				fired_peak_centikg = candidate_peak_centikg;
				active_ticks_left =
					(uint32_t)duration_ms * EXT_BOOST_CONTROL_TICKS_PER_MS;
				state = ASSIST_EXT_BOOST_ACTIVE;
			} else {
				/* Nothing to give: leave the mode's own result alone. */
				rearm_blocked = true;
			}
		}
	}

	if (state == ASSIST_EXT_BOOST_ACTIVE) {
		if (active_ticks_left == 0) {
			/* Ran its full time with the rider still pedalling. Same rule as an
			 * interrupted boost above: no restart until this push ends. The reset
			 * clears the flag, so it is set after, not before. */
			assist_extended_boost_reset(ASSIST_EXT_BOOST_CANCEL_COMPLETED);
			rearm_blocked = true;
		} else {
			/*
			 * The timer runs on ticks alone. A shared limit trimming the result
			 * does not pause it, and no PAS/Hall/speed pulse may extend it.
			 */
			active_ticks_left--;
			if (output != 0) {
				output->iq_target = boost_iq;
				output->active = true;
			}
		}
	}
}

void assist_extended_boost_get_diag(assist_extended_boost_diag_t *diag)
{
	if (diag == 0) {
		return;
	}
	diag->state = (uint8_t)state;
	/* FW-095: no pending arming can exist any more, so this can never be true. Kept in the
	 * struct because it is byte 53 of the 0x6029 block the app decodes. */
	diag->arm_expired = false;
	diag->peak_load_centikg = (state == ASSIST_EXT_BOOST_ACTIVE) ?
		fired_peak_centikg : candidate_peak_centikg;
	diag->boost_iq = (state == ASSIST_EXT_BOOST_ACTIVE) ? boost_iq : 0;
	/* Round UP: truncating showed 199 ms at the start of a 200 ms boost and 0 ms for the
	 * last three ticks of one that was still ACTIVE — which reads as a finished boost still
	 * driving the motor. Any non-zero remainder is at least 1 ms of boost left. */
	uint32_t remaining_ms = (active_ticks_left + EXT_BOOST_CONTROL_TICKS_PER_MS - 1U) /
		EXT_BOOST_CONTROL_TICKS_PER_MS;
	diag->remaining_ms = (remaining_ms > 65535U) ? 65535U : (uint16_t)remaining_ms;
	diag->cancel_reason = last_cancel_reason;
}
