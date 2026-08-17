#include "torque_input.h"

#include "config.h"

#define REST_TARGET_NATIVE ((int32_t)TORQUE_ZERO_TARGET_NATIVE)

static uint16_t span_native = TORQUE_DEFAULT_SPAN_NATIVE;
static uint8_t calibration_source = TORQUE_CAL_SOURCE_DEFAULT;
static torque_snapshot_t snapshot;
static int32_t assist_filter_q;
/* FW-085: RUN estimator — moving average over a crank-angle window. */
static uint16_t run_window_steps = (TORQUE_RUN_WINDOW_DEG_DEFAULT * 4U) / 15U;
static uint16_t run_buffer[TORQUE_RUN_WINDOW_STEPS_MAX];
static uint16_t run_head;
static uint16_t run_filled;
static uint32_t run_sum;
static uint16_t run_value_native;
static uint16_t run_attack_steps; /* FW-090: consecutive steps holding a sustained rise */
/* FW-112 v2: rolling-rearm recovery AUTOMATON - see torque_input_begin_rolling_rearm() in the
 * header for the full lifecycle. IDLE is the normal FW-085 averaging state; WAIT_FRESH_LOAD is
 * entered exactly on the fast-rearm edge (RUN re-seeded to the current fast signal on every
 * forward step); TRACK_FAST is entered once the fresh signal holds at/above the assist deadband
 * (RUN follows the fast signal every control tick). The automaton is closed ONLY by
 * cancel_rolling_rearm() (!latched) or, from TRACK_FAST, by a completed recovery - never by a
 * timeout and never by the forward step count. */
static torque_recovery_state_t recovery_state;
static uint16_t recovery_stable_ticks;

static int32_t offset_correction;
static bool cal_fault;
static int32_t coast_rest_acc;
static bool coast_active;
static uint16_t coast_ticks;
static int32_t last_coast_rest;
static uint8_t reacquire_count;
static uint32_t stuck_ticks;
static bool stuck_fault;
static uint32_t recal_lockout_ticks; /* FW-058: ticks left before the next in-ride re-zero */
static int32_t coast_candidate;      /* FW-059: rest frozen mid-coast, not at its end */
static bool coast_candidate_stable;  /* FW-059: sampling window was quiet enough to trust */
static int32_t coast_window_min;
static int32_t coast_window_max;
/* FW-061 diagnostics. Counters are cumulative and are never cleared by a read:
 * a single sample tells you nothing about whether a zero is drifting. */
static uint8_t coast_last_result = TORQUE_COAST_NONE;
static int16_t coast_last_step;
static uint16_t coast_windows_started;
static uint16_t coast_windows_completed;
static uint16_t coast_applied_count;
static uint16_t coast_reject_too_short;
static uint16_t coast_reject_unstable;
static uint16_t coast_reject_lockout;
static uint16_t coast_reject_out_of_range;
static uint16_t coast_reject_implausible;
static uint16_t coast_no_change_count;
static bool coast_last_was_moving;

static void bump(uint16_t *counter)
{
	if (*counter < 65535U) {
		(*counter)++;
	}
}

static bool span_in_range(uint16_t value)
{
	return value >= TORQUE_SPAN_MIN_NATIVE &&
		value <= TORQUE_SPAN_MAX_NATIVE;
}

static uint16_t default_native_delta_to_centikg(uint16_t delta_native)
{
	uint32_t load;
	if (delta_native <= TORQUE_DEFAULT_LOW_NATIVE) {
		load = ((uint32_t)delta_native * TORQUE_DEFAULT_LOW_CENTIKG +
			TORQUE_DEFAULT_LOW_NATIVE / 2U) / TORQUE_DEFAULT_LOW_NATIVE;
	} else {
		load = TORQUE_DEFAULT_LOW_CENTIKG +
			((uint32_t)(delta_native - TORQUE_DEFAULT_LOW_NATIVE) *
			(TORQUE_DEFAULT_HIGH_CENTIKG - TORQUE_DEFAULT_LOW_CENTIKG) +
			(TORQUE_DEFAULT_HIGH_NATIVE - TORQUE_DEFAULT_LOW_NATIVE) / 2U) /
			(TORQUE_DEFAULT_HIGH_NATIVE - TORQUE_DEFAULT_LOW_NATIVE);
	}
	return (load > TORQUE_INPUT_MAX_CENTIKG) ?
		TORQUE_INPUT_MAX_CENTIKG : (uint16_t)load;
}

static uint16_t default_centikg_to_native_delta(uint16_t centikg)
{
	uint32_t delta;
	if (centikg <= TORQUE_DEFAULT_LOW_CENTIKG) {
		delta = ((uint32_t)centikg * TORQUE_DEFAULT_LOW_NATIVE +
			TORQUE_DEFAULT_LOW_CENTIKG / 2U) / TORQUE_DEFAULT_LOW_CENTIKG;
	} else {
		delta = TORQUE_DEFAULT_LOW_NATIVE +
			((uint32_t)(centikg - TORQUE_DEFAULT_LOW_CENTIKG) *
			(TORQUE_DEFAULT_HIGH_NATIVE - TORQUE_DEFAULT_LOW_NATIVE) +
			(TORQUE_DEFAULT_HIGH_CENTIKG - TORQUE_DEFAULT_LOW_CENTIKG) / 2U) /
			(TORQUE_DEFAULT_HIGH_CENTIKG - TORQUE_DEFAULT_LOW_CENTIKG);
	}
	return (delta > TORQUE_SPAN_MAX_NATIVE) ?
		TORQUE_SPAN_MAX_NATIVE : (uint16_t)delta;
}

static uint16_t native_delta_to_centikg(uint16_t delta_native)
{
	if (calibration_source == TORQUE_CAL_SOURCE_DEFAULT) {
		return default_native_delta_to_centikg(delta_native);
	}
	uint32_t load = ((uint32_t)delta_native *
		TORQUE_PUBLIC_FULL_SCALE_CENTIKG + span_native / 2U) / span_native;
	return (load > TORQUE_INPUT_MAX_CENTIKG) ?
		TORQUE_INPUT_MAX_CENTIKG : (uint16_t)load;
}

static uint16_t centikg_to_native_delta(uint16_t centikg)
{
	if (centikg > TORQUE_INPUT_MAX_CENTIKG) {
		centikg = TORQUE_INPUT_MAX_CENTIKG;
	}
	if (calibration_source == TORQUE_CAL_SOURCE_DEFAULT) {
		return default_centikg_to_native_delta(centikg);
	}
	uint32_t delta = ((uint32_t)centikg * span_native +
		TORQUE_PUBLIC_FULL_SCALE_CENTIKG / 2U) /
		TORQUE_PUBLIC_FULL_SCALE_CENTIKG;
	return (delta > TORQUE_SPAN_MAX_NATIVE) ?
		TORQUE_SPAN_MAX_NATIVE : (uint16_t)delta;
}

static uint16_t update_assist_filter(uint16_t target_native)
{
	const int32_t filter_ticks =
		(int32_t)TORQUE_ASSIST_FILTER_MS * TORQUE_INPUT_TICKS_PER_MS;
	int32_t target_q = (int32_t)target_native << TORQUE_ASSIST_FILTER_Q_SHIFT;
	int32_t error_q = target_q - assist_filter_q;
	if (error_q != 0) {
		int32_t step_q = error_q / filter_ticks;
		if (step_q == 0) {
			step_q = (error_q > 0) ? 1 : -1;
		}
		assist_filter_q += step_q;
	}
	return (uint16_t)((assist_filter_q +
		(1L << (TORQUE_ASSIST_FILTER_Q_SHIFT - 1U))) >>
		TORQUE_ASSIST_FILTER_Q_SHIFT);
}

/*
 * FW-033/085: the RUN effort estimator — a plain moving average of the fast signal
 * over a window of crank steps, so the power calc no longer follows every single
 * leg peak.
 *
 * A true average (not an EMA) is used deliberately: over a whole number of leg
 * periods it CANCELS the pedal ripple rather than merely attenuating it, which is
 * the same reason the TSDZ2 open firmware buffers a full crank revolution. The
 * cost is TORQUE_RUN_WINDOW_STEPS_MAX * 2 = 192 B, which is nothing on this part.
 *
 * The running sum is maintained incrementally, so a step costs one add and one
 * subtract regardless of window size. Worst case 96 * 65535 still fits a uint32.
 */
static void run_window_reset(void)
{
	run_head = 0U;
	run_filled = 0U;
	run_sum = 0U;
	run_attack_steps = 0U; /* FW-090 */
	/* FW-112 v2: a rebuilt window must not inherit a live recovery automaton - the stability
	 * bookkeeping has no meaning against a freshly reset average. */
	recovery_state = TORQUE_RECOVERY_IDLE;
	recovery_stable_ticks = 0U;
}

void torque_input_set_run_window_deg(uint16_t window_deg)
{
	if (window_deg > TORQUE_RUN_WINDOW_DEG_MAX) {
		window_deg = TORQUE_RUN_WINDOW_DEG_MAX;
	}
	/* 3.75 deg per step -> steps = deg * 4 / 15, exact for every 15 deg setting. */
	uint16_t steps = (uint16_t)(((uint32_t)window_deg * 4U) / 15U);
	if (steps > TORQUE_RUN_WINDOW_STEPS_MAX) {
		steps = TORQUE_RUN_WINDOW_STEPS_MAX;
	}
	/*
	 * main.c re-sends the configured value every control tick, so only a REAL
	 * change may disturb the window — rebuilding it every tick would keep the
	 * average permanently empty.
	 */
	if (steps != run_window_steps) {
		run_window_steps = steps;
		run_window_reset();
	}
}

void torque_input_seed_run(uint16_t value_native)
{
	run_value_native = value_native;
	if (run_window_steps == 0U) {
		run_window_reset();
		return;
	}
	/* Fill the whole window, so a seeded launch starts at full magnitude
	 * instead of climbing out of an empty average. */
	for (uint16_t i = 0U; i < run_window_steps; i++) {
		run_buffer[i] = value_native;
	}
	run_head = 0U;
	run_filled = run_window_steps;
	run_sum = (uint32_t)value_native * run_window_steps;
	run_attack_steps = 0U; /* FW-090: the window now holds this level; start counting afresh */
}

/*
 * FW-112 v2: rolling-rearm recovery, driven by src/ride_control.c as a one-shot EVENT (see the
 * header for the full lifecycle and why it replaced v1's latched fast-track).
 *
 * begin: the rearm itself happened - seed the estimator to the current fast signal so the
 * rearmed Iq starts at full magnitude, and enter WAIT_FRESH_LOAD. From here the per-forward-step
 * re-seed (run_filter_step) keeps RUN pinned to the fresh signal in step time, and the automaton
 * advances to TRACK_FAST once the fresh signal holds at/above the assist deadband (confirmed in
 * torque_input_update()) - where RUN tracks the fast signal every control tick until the recovery
 * completes.
 */
void torque_input_begin_rolling_rearm(void)
{
	torque_input_seed_run(snapshot.assist_delta_filtered_native);
	recovery_state = TORQUE_RECOVERY_WAIT_FRESH_LOAD;
	recovery_stable_ticks = 0U;
}

/* cancel: the rearm saga was abandoned - close the automaton immediately so a later NORMAL cold
 * start can never inherit it. The estimator itself is left where it is: closing a recovery is a
 * lifecycle fact, not a filter reset. */
void torque_input_cancel_rolling_rearm(void)
{
	recovery_state = TORQUE_RECOVERY_IDLE;
	recovery_stable_ticks = 0U;
}

/*
 * FW-112 v2: READ side of the recovery automaton, used by ride_control to fix the one-tick
 * stale snapshot on the rearm edge (see the header). recovery_active() is true for every tick
 * the automaton is not IDLE; recovery_run_native() then returns the CURRENT fast signal - the
 * value the recovery is tracking - not the last re-seeded RUN, so the caller can substitute it
 * into the mode calculation's torque_run_filtered input without the one-step lag.
 */
bool torque_input_recovery_active(void)
{
	return recovery_state != TORQUE_RECOVERY_IDLE;
}

uint16_t torque_input_recovery_run_native(void)
{
	return snapshot.assist_delta_filtered_native;
}

torque_recovery_state_t torque_input_recovery_state(void)
{
	return recovery_state;
}

void torque_input_run_filter_step(void)
{
	if (run_window_steps == 0U) {
		return; /* disabled: run tracks the fast signal, see torque_input_update */
	}
	uint16_t sample = snapshot.assist_delta_filtered_native;

	/* FW-112 v2: WAIT_FRESH_LOAD re-seeds to the CURRENT fast sample on every forward step, so a
	 * stale pre-reverse window can never survive the rearm - the estimate follows the fresh pedal
	 * signal honestly, one sample per step, until the pressure is confirmed. Once the automaton
	 * reaches TRACK_FAST, RUN is pinned to the fast signal every control tick in
	 * torque_input_update() instead, so nothing further is done here - the step just advances the
	 * published value that update has already made current. */
	if (recovery_state == TORQUE_RECOVERY_WAIT_FRESH_LOAD) {
		torque_input_seed_run(sample);
		return;
	}

	if (run_filled >= run_window_steps) {
		run_sum -= run_buffer[run_head];
	} else {
		run_filled++;
	}
	run_buffer[run_head] = sample;
	run_sum += sample;
	run_head++;
	if (run_head >= run_window_steps) {
		run_head = 0U;
	}

	/*
	 * FW-090: sustained rise -> re-seed, so a genuine new effort is not held back by the
	 * stale low samples still sitting in the window. Counted BEFORE the average is
	 * recomputed, against the average the sample was actually compared with.
	 *
	 * FW-091: DISABLED by default (STEPS = 0). The re-engagement complaint this was written
	 * for turned out to be caused by the limiter classifying pedal demand from filtered
	 * cadence, not by this average being slow — see FW-091. Ride that fix first; the fast
	 * attack may not be needed at all, and every threshold here is a chance to bring back
	 * the per-leg pulsing FW-085 removed. The guard is explicit because with STEPS = 0 the
	 * "counter reached the target" test below would otherwise be true on every single step.
	 */
	if (TORQUE_RUN_ATTACK_STEPS == 0U) {
		run_value_native = (uint16_t)((run_sum + (run_filled / 2U)) / run_filled);
		return;
	}
	bool attack_sample =
		((uint32_t)sample * TORQUE_RUN_ATTACK_DEN >
			(uint32_t)run_value_native * TORQUE_RUN_ATTACK_NUM) &&
		/* Absolute floor as well as the ratio: near zero the 2x test is met by noise
		 * (average 3, sample 7), which would have the estimator chasing its own jitter
		 * whenever the rider is barely pressing. Both conditions must hold. */
		(sample >= run_value_native + TORQUE_RUN_ATTACK_MIN_DELTA);
	if (attack_sample) {
		if (run_attack_steps < TORQUE_RUN_ATTACK_STEPS) {
			run_attack_steps++;
		}
	} else {
		run_attack_steps = 0U; /* one quiet step breaks the run: only a HELD rise counts */
	}
	if (run_attack_steps >= TORQUE_RUN_ATTACK_STEPS) {
		run_attack_steps = 0U;
		torque_input_seed_run(sample);
		return;
	}
	/* Averaging over however many samples exist keeps the estimate honest before
	 * the first full window — the alternative reads low for a whole revolution. */
	run_value_native = (uint16_t)((run_sum + (run_filled / 2U)) / run_filled);
}

static bool rest_raw_plausible(int32_t raw_rest)
{
	return raw_rest >= TQ_REST_RAW_MIN && raw_rest <= TQ_REST_RAW_MAX;
}

/*
 * FW-059: the rest sample is frozen once the settle window closes, instead of
 * being read at the end of the coast. The old code evaluated the running average
 * at the moment the coast ended - which is the moment the rider has already begun
 * pressing the pedal but the crank has not yet clicked a PAS step, so the sample
 * absorbed that pre-load and dragged the zero towards it. The window is also
 * checked for quiet: a coast noisier than TQ_RECAL_STABLE_MV yields no
 * calibration at all rather than a bad one.
 */
static void coast_accumulate(int16_t torque_corrected_native)
{
	if (!coast_active) {
		coast_active = true;
		coast_ticks = 0;
		coast_rest_acc = (int32_t)torque_corrected_native << 6;
		coast_window_min = torque_corrected_native;
		coast_window_max = torque_corrected_native;
		coast_candidate = torque_corrected_native;
		coast_candidate_stable = false;
		bump(&coast_windows_started); /* FW-061 */
		return;
	}
	if (coast_ticks > TQ_RECAL_SETTLE_TICKS) {
		return; /* candidate already frozen; the rest of the coast is ignored */
	}
	coast_rest_acc -= coast_rest_acc >> 6;
	coast_rest_acc += torque_corrected_native;
	if (torque_corrected_native < coast_window_min) {
		coast_window_min = torque_corrected_native;
	}
	if (torque_corrected_native > coast_window_max) {
		coast_window_max = torque_corrected_native;
	}
	coast_ticks++;
	if (coast_ticks > TQ_RECAL_SETTLE_TICKS) {
		coast_candidate = coast_rest_acc >> 6;
		coast_candidate_stable =
			(coast_window_max - coast_window_min) <= TQ_RECAL_STABLE_MV;
	}
}

static bool drift_confirmed(int32_t rest)
{
	int32_t spread = rest - last_coast_rest;

	if (spread < 0) {
		spread = -spread;
	}
	if (reacquire_count > 0 && spread <= TQ_REACQUIRE_TOL_MV) {
		reacquire_count++;
	} else {
		reacquire_count = 1;
	}
	last_coast_rest = rest;
	if (reacquire_count >= TQ_REACQUIRE_COASTS) {
		reacquire_count = 0;
		return true;
	}
	return false;
}

/*
 * FW-061: returns the step actually taken, after clamping. The caller needs it
 * because a zero-sized step must not arm the lockout - otherwise one pointless
 * evaluation blocks the next correction that is genuinely needed.
 */
static int32_t apply_offset_step(int32_t diff)
{
	if (diff > TQ_RECAL_MAX_STEP) {
		diff = TQ_RECAL_MAX_STEP;
	} else if (diff < -TQ_RECAL_MAX_STEP) {
		diff = -TQ_RECAL_MAX_STEP;
	}
	offset_correction += diff;
	return diff;
}

/*
 * FW-058: applying a correction arms the lockout only while the bike is moving.
 * At standstill the rest reading is trustworthy (the rider usually has a foot on
 * the ground, so nothing loads the pedals) and stays as frequent as before.
 * FW-061: an evaluation that changes nothing is reported as NO_CHANGE and leaves
 * the lockout alone.
 */
static void apply_and_arm(int32_t diff, bool bike_moving)
{
	int32_t step = apply_offset_step(diff);
	coast_last_step = (int16_t)step;
	if (step == 0) {
		coast_last_result = TORQUE_COAST_NO_CHANGE;
		bump(&coast_no_change_count);
		return;
	}
	coast_last_result = TORQUE_COAST_APPLIED;
	bump(&coast_applied_count);
	if (bike_moving) {
		recal_lockout_ticks = TQ_RECAL_MIN_PERIOD_TICKS;
	}
}

static void coast_evaluate(bool bike_moving)
{
	int32_t rest = coast_candidate; /* FW-059: frozen mid-coast, not read at its end */
	int32_t raw_rest = rest - offset_correction;
	int32_t diff;
	int32_t distance;

	/* The plausibility check stays ahead of both gates: sensor-fault detection
	 * (Error 25) must keep running at its old rate. The gates below only ever
	 * suppress moving the zero. */
	coast_last_was_moving = bike_moving;
	bump(&coast_windows_completed);

	if (!rest_raw_plausible(raw_rest)) {
		cal_fault = true;
		coast_last_result = TORQUE_COAST_IMPLAUSIBLE_RAW;
		bump(&coast_reject_implausible);
		return;
	}
	cal_fault = false;

	if (!coast_candidate_stable) { /* FW-059 */
		coast_last_result = TORQUE_COAST_UNSTABLE;
		bump(&coast_reject_unstable);
		return;
	}
	if (bike_moving && recal_lockout_ticks > 0U) {
		coast_last_result = TORQUE_COAST_LOCKOUT;
		bump(&coast_reject_lockout);
		return;
	}

	diff = REST_TARGET_NATIVE - rest;
	distance = diff < 0 ? -diff : diff;
	if (distance <= TQ_RECAL_BAND_MV) {
		reacquire_count = 0;
		apply_and_arm(diff, bike_moving);
	} else if (distance <= TQ_REACQUIRE_MAX_MV) {
		/* 31..40 mV: real drift has to repeat across TQ_REACQUIRE_COASTS coasts,
		 * otherwise a foot resting on the pedal would be adopted as the new zero. */
		if (drift_confirmed(rest)) {
			apply_and_arm(diff, bike_moving);
		} else {
			coast_last_result = TORQUE_COAST_NO_CHANGE;
			bump(&coast_no_change_count);
		}
	} else {
		/* Above 40 mV nothing is corrected automatically - but it must not be
		 * silent, or a stuck/failing sensor looks exactly like a quiet ride. */
		reacquire_count = 0;
		coast_last_result = TORQUE_COAST_OUT_OF_REACQUIRE_RANGE;
		bump(&coast_reject_out_of_range);
	}
}

void torque_input_startup_zero(int32_t rest_raw_native)
{
	offset_correction = REST_TARGET_NATIVE - rest_raw_native;
	if (!rest_raw_plausible(rest_raw_native)) {
		offset_correction = 0;
		cal_fault = true;
	}
}

int16_t torque_input_correct(uint16_t raw_native)
{
	return (int16_t)(raw_native + offset_correction);
}

void torque_input_coast_update(int16_t torque_corrected_native, bool coast_eligible,
	bool bike_moving)
{
	/* Called every 4 kHz tick regardless of coasting, so this is the tick source
	 * for the FW-058 minimum period between in-ride corrections. */
	if (recal_lockout_ticks > 0U) {
		recal_lockout_ticks--;
	}
	if (torque_input_calibration_active()) {
		coast_active = false;
		return;
	}
	if (coast_eligible) {
		coast_accumulate(torque_corrected_native);
		return;
	}
	if (coast_active) {
		if (coast_ticks > TQ_RECAL_SETTLE_TICKS) {
			coast_evaluate(bike_moving);
		} else {
			/* FW-061: a window that never reached the settle time is its own
			 * outcome. Without this it would be invisible, or worse, get blamed
			 * on the stability gate. */
			coast_last_result = TORQUE_COAST_TOO_SHORT;
			bump(&coast_reject_too_short);
		}
	}
	coast_active = false;
}

bool torque_input_cal_fault(void)
{
	return cal_fault || stuck_fault;
}

void torque_input_init(void)
{
	span_native = TORQUE_DEFAULT_SPAN_NATIVE;
	calibration_source = TORQUE_CAL_SOURCE_DEFAULT;
	snapshot = (torque_snapshot_t){0};
	assist_filter_q = 0;
	run_value_native = 0U;
	run_window_reset(); /* FW-085 */
	snapshot.zero_effective_native = TORQUE_ZERO_TARGET_NATIVE;
	snapshot.span_native = span_native;
}

void torque_input_update(uint16_t raw_native, int16_t torque_corrected_native,
	bool sensor_valid)
{
	int32_t delta = (int32_t)torque_corrected_native - REST_TARGET_NATIVE;
	uint16_t assist_delta;

	if (delta < 0) {
		delta = 0;
	}
	if (delta > (int32_t)TORQUE_SPAN_MAX_NATIVE) {
		delta = TORQUE_SPAN_MAX_NATIVE;
	}

	snapshot.raw_native = raw_native;
	snapshot.zero_effective_native =
		(uint16_t)(REST_TARGET_NATIVE - offset_correction);
	snapshot.corrected_native = torque_corrected_native;
	snapshot.delta_native = (uint16_t)delta;
	assist_delta = (delta > (int32_t)TORQUE_ASSIST_DEADBAND_NATIVE) ?
		(uint16_t)(delta - TORQUE_ASSIST_DEADBAND_NATIVE) : 0U;
	if (assist_delta > span_native) {
		assist_delta = span_native;
	}
	snapshot.assist_delta_native = assist_delta;
	snapshot.assist_delta_filtered_native = update_assist_filter(
		sensor_valid ? assist_delta : 0U);

	/*
	 * FW-112 v2: advance the rolling-rearm recovery AUTOMATON (see torque_input_begin_rolling_
	 * rearm() in the header). Once per tick while it is not IDLE:
	 *   - WAIT_FRESH_LOAD -> TRACK_FAST  when the fresh signal holds at/above the assist deadband
	 *                                       (pressure confirmed; run then follows the fast signal
	 *                                       every tick, see the publish block below);
	 *   - TRACK_FAST -> IDLE              when the fresh signal has held at/above the deadband for
	 *                                       TORQUE_ROLLING_REARM_STABLE_TICKS (recovery complete -
	 *                                       re-seed once to that level so ordinary FW-085
	 *                                       averaging resumes from a known-good average);
	 *   - TRACK_FAST -> WAIT_FRESH_LOAD   when the fresh signal drops back below the deadband
	 *                                       (pressure lost mid-recovery - honest collapse);
	 * and from WAIT_FRESH_LOAD there is NO further exit here: no timeout (a resumed-but-
	 * unpressured ride simply keeps re-seeding RUN to the ~0 fast signal on each forward step,
	 * which is already honest) and no dependence on forward steps arriving - the only edges that
	 * close the automaton are cancel_rolling_rearm() (!latched) and the completed recovery above.
	 */
	if (recovery_state == TORQUE_RECOVERY_WAIT_FRESH_LOAD) {
		if (snapshot.assist_delta_filtered_native >= TORQUE_ASSIST_DEADBAND_NATIVE) {
			recovery_state = TORQUE_RECOVERY_TRACK_FAST;
			recovery_stable_ticks = 0U;
		}
	} else if (recovery_state == TORQUE_RECOVERY_TRACK_FAST) {
		if (snapshot.assist_delta_filtered_native >= TORQUE_ASSIST_DEADBAND_NATIVE) {
			if (recovery_stable_ticks < TORQUE_ROLLING_REARM_STABLE_TICKS) {
				recovery_stable_ticks++;
			}
			if (recovery_stable_ticks >= TORQUE_ROLLING_REARM_STABLE_TICKS) {
				torque_input_seed_run(snapshot.assist_delta_filtered_native);
				recovery_state = TORQUE_RECOVERY_IDLE;
				recovery_stable_ticks = 0U;
			}
		} else {
			recovery_state = TORQUE_RECOVERY_WAIT_FRESH_LOAD;
			recovery_stable_ticks = 0U;
		}
	}
	/*
	 * FW-033/085: RUN estimator averages the FAST signal (cascade fast->run).
	 * When disabled (window 0) run tracks fast exactly = old behaviour.
	 *
	 * FW-085: the average itself advances in torque_input_run_filter_step(), driven
	 * by the crank, not by this tick. Here we only publish the value it holds — so
	 * between crank steps the estimate stays put instead of decaying, which is
	 * precisely what stops assist pulsing once per leg at low cadence.
	 *
	 * FW-112 v2: while the recovery automaton is in TRACK_FAST, RUN follows the fast signal
	 * every control tick instead — the re-seed on forward steps alone would lag the fresh
	 * pressure by up to a full step interval at low cadence, and the whole point of the
	 * recovery is fast-filter time (35 ms), not step time.
	 */
	if (recovery_state == TORQUE_RECOVERY_TRACK_FAST) {
		run_value_native = snapshot.assist_delta_filtered_native;
	} else if (run_window_steps == 0U) {
		run_value_native = snapshot.assist_delta_filtered_native;
	}
	snapshot.assist_delta_run_native = run_value_native;
	snapshot.load_centikg = native_delta_to_centikg((uint16_t)delta);
	snapshot.span_native = span_native;
	snapshot.calibration_source = calibration_source;
	snapshot.sensor_valid = sensor_valid;

	if (snapshot.load_centikg >= TQ_STUCK_CENTIKG) {
		if (stuck_ticks < TQ_STUCK_TICKS) {
			stuck_ticks++;
		} else {
			stuck_fault = true;
		}
	} else {
		stuck_ticks = 0;
		stuck_fault = false;
	}
}

const torque_snapshot_t *torque_input_get_snapshot(void)
{
	return &snapshot;
}

uint16_t torque_input_load_centikg(void)
{
	return snapshot.load_centikg;
}

uint16_t torque_input_zero_native(void)
{
	return TORQUE_ZERO_TARGET_NATIVE;
}

uint16_t torque_input_span_native(void)
{
	return span_native;
}

uint16_t torque_input_full_scale_native(void)
{
	return (uint16_t)(TORQUE_ZERO_TARGET_NATIVE + span_native);
}

uint8_t torque_input_calibration_source(void)
{
	return calibration_source;
}

bool torque_input_set_user_span(uint16_t value)
{
	if (!span_in_range(value)) {
		return false;
	}
	span_native = value;
	calibration_source = TORQUE_CAL_SOURCE_USER;
	return true;
}

void torque_input_restore_default_span(void)
{
	span_native = TORQUE_DEFAULT_SPAN_NATIVE;
	calibration_source = TORQUE_CAL_SOURCE_DEFAULT;
}

#define CAL_AVG_SHIFT 6U
#define CAL_AVG_TICKS 256U
#define CAL_STABILITY_MAX_SPREAD 8
#define CAL_DELTA_MIN_NATIVE 100
#define CAL_SATURATION_NATIVE 3200
#define CAL_WAIT_TIMEOUT_TICKS 120000UL

static uint8_t cal_state = TORQUE_CAL_STATE_IDLE;
static uint8_t cal_error = TORQUE_CAL_ERR_NONE;
static uint16_t cal_reference_centikg;
static uint16_t cal_preview_span;
static int32_t cal_zero_avg;
static int32_t cal_acc;
static int16_t cal_window_min;
static int16_t cal_window_max;
static uint16_t cal_window_ticks;
static uint32_t cal_wait_ticks;
static bool cal_persist_request;

static void cal_window_reset(int16_t corrected)
{
	cal_acc = 0;
	cal_window_min = corrected;
	cal_window_max = corrected;
	cal_window_ticks = 0;
}

static void cal_fail(uint8_t error)
{
	cal_state = TORQUE_CAL_STATE_FAILED;
	cal_error = error;
}

bool torque_input_calibration_active(void)
{
	return cal_state >= TORQUE_CAL_STATE_CAPTURE_ZERO &&
		cal_state <= TORQUE_CAL_STATE_PREVIEW;
}

uint8_t torque_input_cal_state(void)
{
	return cal_state;
}

uint8_t torque_input_cal_error(void)
{
	return cal_error;
}

uint16_t torque_input_cal_reference_centikg(void)
{
	return cal_reference_centikg;
}

uint16_t torque_input_cal_preview_span(void)
{
	return cal_preview_span;
}

void torque_input_cal_start(void)
{
	cal_error = TORQUE_CAL_ERR_NONE;
	cal_preview_span = 0;
	cal_reference_centikg = 0;
	cal_wait_ticks = 0;
	cal_window_reset(snapshot.corrected_native);
	cal_state = TORQUE_CAL_STATE_CAPTURE_ZERO;
}

void torque_input_cal_capture_load(uint16_t reference_centikg)
{
	if (cal_state != TORQUE_CAL_STATE_WAIT_REFERENCE) {
		return;
	}
	if (reference_centikg < TORQUE_CAL_REFERENCE_MIN_CENTIKG ||
		reference_centikg > TORQUE_CAL_REFERENCE_MAX_CENTIKG) {
		cal_fail(TORQUE_CAL_ERR_REFERENCE_RANGE);
		return;
	}
	cal_reference_centikg = reference_centikg;
	cal_window_reset(snapshot.corrected_native);
	cal_state = TORQUE_CAL_STATE_CAPTURE_LOAD;
}

bool torque_input_cal_commit(void)
{
	if (cal_state != TORQUE_CAL_STATE_PREVIEW ||
		!torque_input_set_user_span(cal_preview_span)) {
		return false;
	}
	cal_state = TORQUE_CAL_STATE_SUCCESS;
	cal_persist_request = true;
	return true;
}

void torque_input_cal_cancel(void)
{
	if (torque_input_calibration_active()) {
		cal_state = TORQUE_CAL_STATE_CANCELLED;
	}
}

void torque_input_cal_restore_default(void)
{
	torque_input_restore_default_span();
	cal_state = TORQUE_CAL_STATE_IDLE;
	cal_error = TORQUE_CAL_ERR_NONE;
	cal_persist_request = true;
}

bool torque_input_cal_take_persist_request(void)
{
	bool pending = cal_persist_request;
	cal_persist_request = false;
	return pending;
}

void torque_input_cal_tick(int16_t torque_corrected_native, bool stationary)
{
	if (!torque_input_calibration_active()) {
		return;
	}
	if (cal_fault || stuck_fault) {
		cal_fail(TORQUE_CAL_ERR_SENSOR_FAULT);
		return;
	}
	if (!stationary) {
		cal_fail(TORQUE_CAL_ERR_NOT_STATIONARY);
		return;
	}

	if (cal_state == TORQUE_CAL_STATE_WAIT_REFERENCE) {
		if (++cal_wait_ticks > CAL_WAIT_TIMEOUT_TICKS) {
			cal_fail(TORQUE_CAL_ERR_TIMEOUT);
		}
		return;
	}
	if (cal_state == TORQUE_CAL_STATE_PREVIEW) {
		if (++cal_wait_ticks > CAL_WAIT_TIMEOUT_TICKS) {
			cal_fail(TORQUE_CAL_ERR_TIMEOUT);
		}
		return;
	}

	cal_acc += torque_corrected_native;
	if (torque_corrected_native < cal_window_min) {
		cal_window_min = torque_corrected_native;
	}
	if (torque_corrected_native > cal_window_max) {
		cal_window_max = torque_corrected_native;
	}
	if (++cal_window_ticks < CAL_AVG_TICKS) {
		return;
	}

	if ((cal_window_max - cal_window_min) > CAL_STABILITY_MAX_SPREAD) {
		cal_fail(TORQUE_CAL_ERR_UNSTABLE);
		return;
	}
	int32_t average = cal_acc >> (CAL_AVG_SHIFT + 2U);

	if (cal_state == TORQUE_CAL_STATE_CAPTURE_ZERO) {
		cal_zero_avg = average;
		cal_wait_ticks = 0;
		cal_state = TORQUE_CAL_STATE_WAIT_REFERENCE;
		return;
	}

	if (average >= CAL_SATURATION_NATIVE) {
		cal_fail(TORQUE_CAL_ERR_SATURATED);
		return;
	}
	int32_t delta_reference = average - cal_zero_avg;
	if (delta_reference < CAL_DELTA_MIN_NATIVE) {
		cal_fail(TORQUE_CAL_ERR_DELTA_TOO_SMALL);
		return;
	}
	uint32_t span = ((uint32_t)delta_reference *
		TORQUE_PUBLIC_FULL_SCALE_CENTIKG) / cal_reference_centikg;
	if (!span_in_range((uint16_t)span)) {
		cal_fail(TORQUE_CAL_ERR_SPAN_RANGE);
		return;
	}
	cal_preview_span = (uint16_t)span;
	cal_wait_ticks = 0;
	cal_state = TORQUE_CAL_STATE_PREVIEW;
}

#define TORQUE_CAL_PERSIST_MAGIC 0x7C41U
#define TORQUE_CAL_PERSIST_VERSION 1U

static uint16_t torque_cal_crc16(uint8_t version, uint16_t span)
{
	uint8_t buffer[3] = { version, (uint8_t)(span & 0xFFU),
		(uint8_t)(span >> 8) };
	uint16_t crc = 0xFFFFU;
	for (uint8_t i = 0; i < sizeof(buffer); i++) {
		crc ^= (uint16_t)buffer[i] << 8;
		for (uint8_t bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000U) ?
				(uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

bool torque_input_restore_persist(uint16_t magic, uint8_t version,
	uint16_t span_stored, uint16_t crc)
{
	if (magic != TORQUE_CAL_PERSIST_MAGIC ||
		version != TORQUE_CAL_PERSIST_VERSION ||
		crc != torque_cal_crc16(version, span_stored) ||
		!span_in_range(span_stored)) {
		return false;
	}
	span_native = span_stored;
	calibration_source = TORQUE_CAL_SOURCE_USER;
	return true;
}

void torque_input_build_persist(uint16_t *magic, uint8_t *version,
	uint16_t *span_out, uint16_t *crc)
{
	if (calibration_source == TORQUE_CAL_SOURCE_USER) {
		*magic = TORQUE_CAL_PERSIST_MAGIC;
		*version = TORQUE_CAL_PERSIST_VERSION;
		*span_out = span_native;
		*crc = torque_cal_crc16(TORQUE_CAL_PERSIST_VERSION, span_native);
	} else {
		*magic = 0;
		*version = 0;
		*span_out = 0;
		*crc = 0;
	}
}

static void put16(uint8_t *b, uint16_t v)
{
	b[0] = (uint8_t)(v & 0xFFU);
	b[1] = (uint8_t)(v >> 8);
}

uint16_t torque_input_serialize_telemetry(uint8_t *buffer)
{
	buffer[0] = 0x54;  /* 'T' */
	buffer[1] = 0x43;  /* 'C' */
	buffer[2] = 2;     /* FW-061: v2 = v1 layout + coast re-zero diagnostics */
	buffer[3] = TORQUE_CAP_LOAD_TELEMETRY_V1 | TORQUE_CAP_CALIBRATION_V1 |
		TORQUE_CAP_COAST_DIAG_V2;
	put16(&buffer[4], snapshot.load_centikg);
	put16(&buffer[6], snapshot.zero_effective_native);
	put16(&buffer[8], snapshot.delta_native);
	put16(&buffer[10], span_native);
	buffer[12] = calibration_source;
	buffer[13] = cal_state;
	buffer[14] = cal_error;
	buffer[15] = snapshot.sensor_valid ? 1U : 0U;
	put16(&buffer[16], cal_reference_centikg);
	put16(&buffer[18], cal_preview_span);
	put16(&buffer[20], torque_input_full_scale_native());

	/* FW-061 v2 block. Everything needed to explain a moved zero: what was
	 * sampled, how quiet the window was, what step was taken and why not. */
	put16(&buffer[22], snapshot.raw_native);
	put16(&buffer[24], (uint16_t)coast_candidate);
	uint16_t spread = (coast_window_max > coast_window_min) ?
		(uint16_t)(coast_window_max - coast_window_min) : 0U;
	put16(&buffer[26], spread);
	put16(&buffer[28], (uint16_t)coast_last_step); /* signed, two's complement */
	uint32_t lockout_s = (recal_lockout_ticks + (TORQUE_INPUT_TICKS_PER_MS * 1000U) - 1U) /
		(TORQUE_INPUT_TICKS_PER_MS * 1000U);
	buffer[30] = (lockout_s > 255U) ? 255U : (uint8_t)lockout_s;
	buffer[31] = (coast_active ? 0x01U : 0U) |
		(coast_last_was_moving ? 0x02U : 0U) |
		(coast_candidate_stable ? 0x04U : 0U);
	buffer[32] = coast_last_result;
	buffer[33] = 0; /* reserved */
	put16(&buffer[34], coast_windows_started);
	put16(&buffer[36], coast_windows_completed);
	put16(&buffer[38], coast_applied_count);
	put16(&buffer[40], coast_reject_too_short);
	put16(&buffer[42], coast_reject_unstable);
	put16(&buffer[44], coast_reject_lockout);
	put16(&buffer[46], coast_reject_out_of_range);
	put16(&buffer[48], coast_reject_implausible);
	put16(&buffer[50], coast_no_change_count);
	put16(&buffer[52], (uint16_t)offset_correction);

	uint16_t crc = 0xFFFFU;
	for (uint8_t i = 0; i < TORQUE_TELEMETRY_BLOB_LEN - 2U; i++) {
		crc ^= (uint16_t)buffer[i] << 8;
		for (uint8_t bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000U) ?
				(uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
		}
	}
	put16(&buffer[TORQUE_TELEMETRY_BLOB_LEN - 2U], crc);
	return TORQUE_TELEMETRY_BLOB_LEN;
}

uint16_t torque_input_centikg_to_native_delta(uint16_t centikg)
{
	return centikg_to_native_delta(centikg);
}

uint16_t torque_input_native_delta_to_centikg(uint16_t delta_native)
{
	if (delta_native > TORQUE_SPAN_MAX_NATIVE) {
		delta_native = TORQUE_SPAN_MAX_NATIVE;
	}
	return native_delta_to_centikg(delta_native);
}
