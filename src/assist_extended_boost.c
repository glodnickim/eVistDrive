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
#define EXT_BOOST_ARM_TIMEOUT_TICKS \
	(EXT_BOOST_ARM_TIMEOUT_MS * EXT_BOOST_CONTROL_TICKS_PER_MS)

static assist_extended_boost_state_t state;
/* Peak of the window currently being qualified. Kept SEPARATE from armed_peak so a spike
 * that never completes its 30 ms cannot wipe an arming that already did. */
static uint16_t candidate_peak_centikg;
static uint16_t armed_peak_centikg;
static uint16_t confirm_ticks;
static uint32_t arm_idle_ticks;
static uint32_t active_ticks_left;
static bool window_open;
static bool previous_pedaling_active;
static bool arm_expired;
static int32_t boost_iq;
static uint8_t last_cancel_reason;
static uint8_t last_bank_index;
static uint8_t last_level_index;
static bool have_context;

static void clear_state(void)
{
	state = ASSIST_EXT_BOOST_IDLE;
	candidate_peak_centikg = 0;
	armed_peak_centikg = 0;
	confirm_ticks = 0;
	arm_idle_ticks = 0;
	active_ticks_left = 0;
	window_open = false;
	boost_iq = 0;
}

void assist_extended_boost_init(void)
{
	clear_state();
	previous_pedaling_active = false;
	arm_expired = false;
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
 * "The latest pedal push" is the latest CONTINUOUS window above the threshold, not the
 * hardest push since the ride began. A window opens on the threshold, keeps its own peak,
 * and closes 0.5 kg below — the hysteresis stabilizes the end of a push and never moves the
 * threshold the rider set. A later, WEAKER confirmed window replaces an earlier stronger
 * one on purpose: what the rider just did is what the boost should reproduce.
 */
static void qualify_and_arm(
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
			arm_expired = false;
			if (state == ASSIST_EXT_BOOST_IDLE) {
				state = ASSIST_EXT_BOOST_QUALIFY;
			}
		}
		if (load > candidate_peak_centikg) {
			candidate_peak_centikg = load;
		}
		if (confirm_ticks < EXT_BOOST_CONFIRM_TICKS) {
			confirm_ticks++;
		}
		if (confirm_ticks >= EXT_BOOST_CONFIRM_TICKS) {
			/* Confirmed: this window now owns the arming, and keeps updating its
			 * peak for as long as it lasts. */
			armed_peak_centikg = candidate_peak_centikg;
			state = ASSIST_EXT_BOOST_ARMED;
			arm_idle_ticks = 0;
		}
		return;
	}

	if (window_open) {
		bool ended = !may_qualify ||
			(load + EXT_BOOST_RELEASE_HYST_CENTIKG) < trigger_centikg;
		if (ended) {
			window_open = false;
			confirm_ticks = 0;
			candidate_peak_centikg = 0;
			/* An unconfirmed window leaves nothing behind; a confirmed one stays
			 * armed until it goes stale below. */
			if (state == ASSIST_EXT_BOOST_QUALIFY) {
				state = ASSIST_EXT_BOOST_IDLE;
			}
		}
		return;
	}

	if (state == ASSIST_EXT_BOOST_ARMED) {
		if (++arm_idle_ticks >= EXT_BOOST_ARM_TIMEOUT_TICKS) {
			arm_expired = true;
			assist_extended_boost_reset(ASSIST_EXT_BOOST_CANCEL_ARM_TIMEOUT);
		}
	}
}

void assist_extended_boost_update(
	const assist_extended_boost_input_t *input,
	const assist_extended_boost_config_t *config,
	assist_extended_boost_output_t *output)
{
	if (output != 0) {
		output->iq_target = 0;
		output->profile_hold_active = false;
		output->armed = false;
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
	} else if (state == ASSIST_EXT_BOOST_ACTIVE && input->pedaling_active) {
		/* Pedalling came back: no leftover arming, no half-finished timer. The next
		 * boost needs a fresh confirmed push. */
		cancel = ASSIST_EXT_BOOST_CANCEL_PEDALING_RESUMED;
	}

	last_bank_index = input->bank_index;
	last_level_index = input->level_index;
	have_context = true;

	if (cancel != ASSIST_EXT_BOOST_CANCEL_NONE) {
		assist_extended_boost_reset(cancel);
		previous_pedaling_active = input->pedaling_active;
		return;
	}

	if (state != ASSIST_EXT_BOOST_ACTIVE) {
		qualify_and_arm(input, trigger_centikg);

		/*
		 * The boost starts on the EDGE of pedalling stopping, never on a level, and
		 * only from a previously confirmed arming.
		 *
		 * pedal_assist_latched is deliberately NOT required here: ride_control drops
		 * the latch in the very same tick as PAS STOP, so demanding it on this edge
		 * would cancel every correct arming. The earlier legal start is already
		 * proven by the state the module is in.
		 */
		bool pedal_stop_edge = previous_pedaling_active && !input->pedaling_active;
		if (state == ASSIST_EXT_BOOST_ARMED && pedal_stop_edge &&
			input->motion_valid) {
			int32_t candidate = compute_boost_iq(
				armed_peak_centikg,
				trigger_centikg,
				config->strength_pct,
				input->ride_core_iq_limit);
			if (candidate > 0) {
				boost_iq = candidate;
				active_ticks_left =
					(uint32_t)duration_ms * EXT_BOOST_CONTROL_TICKS_PER_MS;
				state = ASSIST_EXT_BOOST_ACTIVE;
				window_open = false;
				confirm_ticks = 0;
				candidate_peak_centikg = 0;
			} else {
				/* Nothing to hold: hand target 0 to the ordinary release. */
				clear_state();
			}
		}
	}

	if (state == ASSIST_EXT_BOOST_ACTIVE) {
		if (active_ticks_left == 0) {
			assist_extended_boost_reset(ASSIST_EXT_BOOST_CANCEL_COMPLETED);
		} else {
			/*
			 * The timer runs on ticks alone. A shared limit trimming the result
			 * does not pause it, and no PAS/Hall/speed pulse may extend it.
			 */
			active_ticks_left--;
			if (output != 0) {
				output->iq_target = boost_iq;
				output->profile_hold_active = true;
				output->active = true;
			}
		}
	}

	if (output != 0) {
		output->armed = (state == ASSIST_EXT_BOOST_ARMED) ||
			(state == ASSIST_EXT_BOOST_ACTIVE);
	}
	previous_pedaling_active = input->pedaling_active;
}

void assist_extended_boost_get_diag(assist_extended_boost_diag_t *diag)
{
	if (diag == 0) {
		return;
	}
	diag->state = (uint8_t)state;
	diag->arm_expired = arm_expired;
	diag->peak_load_centikg = armed_peak_centikg;
	diag->boost_iq = (state == ASSIST_EXT_BOOST_ACTIVE) ? boost_iq : 0;
	/* Round UP: truncating showed 199 ms at the start of a 200 ms boost and 0 ms for the
	 * last three ticks of one that was still ACTIVE — which reads as a finished boost still
	 * driving the motor. Any non-zero remainder is at least 1 ms of boost left. */
	uint32_t remaining_ms = (active_ticks_left + EXT_BOOST_CONTROL_TICKS_PER_MS - 1U) /
		EXT_BOOST_CONTROL_TICKS_PER_MS;
	diag->remaining_ms = (remaining_ms > 65535U) ? 65535U : (uint16_t)remaining_ms;
	diag->cancel_reason = last_cancel_reason;
}
