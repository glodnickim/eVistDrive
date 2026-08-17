#include "assist_dynamics.h"

#include "config.h"

#define CONTROL_TICKS_PER_MS 4
#define PROFILE_RELEASE_MAX_MS 3000

/* FW-069: fallbacks used only if the caller left a ramp field at 0. The real values are
 * per level and arrive through assist_dynamics_input_t. */
#define RAMP_UP_SLOW_FALLBACK_MS   600
#define RAMP_UP_FAST_FALLBACK_MS   300
#define RAMP_DOWN_SLOW_FALLBACK_MS 1000
#define RAMP_DOWN_FAST_FALLBACK_MS 140

static int32_t ramp_ticks(uint16_t ms, int32_t fallback_ms)
{
	int32_t value = (ms > 0U) ? (int32_t)ms : fallback_ms;
	return value * CONTROL_TICKS_PER_MS;
}

/*
 * FW-072: the release fade is ONE linear ramp again.
 *
 * FW-047 had added a slow tail over the last 15 % of the starting level, on the theory that
 * the gearbox clunk came from torque being removed at a constant rate right down to zero.
 * Testing did not bear that out: the clunk stayed, and the tail silently stretched the fade —
 * release_ms reached 15 %, then a further 600 ms ran before the current actually hit zero.
 * A configured 650 ms behaved like ~1250 ms, which is not what the field says it does.
 *
 * Now: current at the moment pedalling stops -> 0 over exactly release_ms, whatever that
 * current was.
 */

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

#if IQ_RAMP_TIME_MODE
static int32_t iq_reference_q;
/*
 * FW-040: fixed-time release fade. The generic ramp derives its step from the FULL
 * current scale, so a release that starts from a partial level (normal riding is well
 * below full scale) reached zero in a fraction of release_ms — the fade felt far too
 * short. The release path now latches the Iq value at fade start and divides THAT by
 * release_ms, so the configured time is the real time to zero from any level.
 */
static int32_t profile_release_step_q;      /* per-tick step for the active fade */
#endif

int32_t assist_dynamics_apply(
	int32_t iq_target,
	int32_t iq_reference,
	const assist_dynamics_input_t *input)
{
	/*
	 * Leaving WA is a dead-man stop. The WA controller has already discarded
	 * its command; do not feed its last Iq into the normal ride release fade.
	 */
	if (input->immediate_cut) {
#if IQ_RAMP_TIME_MODE
		iq_reference_q = 0;
		profile_release_step_q = 0;
#endif
		return 0;
	}
	/*
	 * FW-112 v2 SAME-TICK ZERO (+ audit S13): force the ramp accumulator to zero this tick.
	 * The caller sets this when the FINAL demand is 0 and (a) this is the exact rolling
	 * fast-rearm tick, or (b) the recovery automaton is in WAIT_FRESH_LOAD. In both cases
	 * the old reference must not keep flowing through the ramp accumulator — the motor
	 * command (and MS.i_q_setpoint) must be exactly 0 in the same tick. The caller only
	 * sets it on a zero final demand, so a cold start, a normal release and any real
	 * positive demand are unaffected.
	 */
	if (input->force_zero_reference) {
#if IQ_RAMP_TIME_MODE
		iq_reference_q = 0;
		profile_release_step_q = 0;
#endif
		return 0;
	}
#if IQ_RAMP_TIME_MODE
	/*
	 * FW-060: WA owns its complete Iq trajectory, including start and release
	 * slew. Keep the shared ramp synchronized for a smooth exit, but do not put
	 * a second dynamic element behind the speed controller.
	 */
	if (input->walk_active) {
		iq_reference_q = iq_target << IQ_RAMP_Q_SHIFT;
		profile_release_step_q = 0;
		return iq_target;
	}
	// FW-037: safety cuts no longer snap Iq to 0. The caller (ride_control) forces
	// iq_target = 0 with a short release ramp on brake/backward/overtemp/torque-fault,
	// so the bridge fades out smoothly (no gearbox klik). Only overcurrent hard-cuts
	// the bridge, in FOC.c. The normal ramp/release path below handles the fade.

	//FW-069: per level, supplied by the caller from the active bank's level config
	int32_t ramp_up_slow = ramp_ticks(input->ramp_up_slow_ms, RAMP_UP_SLOW_FALLBACK_MS);
	int32_t ramp_up_fast = ramp_ticks(input->ramp_up_fast_ms, RAMP_UP_FAST_FALLBACK_MS);
	int32_t ramp_down_slow = ramp_ticks(input->ramp_down_slow_ms, RAMP_DOWN_SLOW_FALLBACK_MS);
	int32_t ramp_down_fast = ramp_ticks(input->ramp_down_fast_ms, RAMP_DOWN_FAST_FALLBACK_MS);

#if IQ_RAMP_ADAPTIVE
	int32_t up_s = map((int32_t)input->speed_x100,
		IQ_RAMP_SPEED_LO, IQ_RAMP_SPEED_HI,
		ramp_up_slow, ramp_up_fast);
	int32_t up_c = map((int32_t)input->cadence_rpm,
		IQ_RAMP_CAD_LO, IQ_RAMP_CAD_HI,
		ramp_up_slow, ramp_up_fast);
	int32_t dn_s = map((int32_t)input->speed_x100,
		IQ_RAMP_SPEED_LO, IQ_RAMP_SPEED_HI,
		ramp_down_slow, ramp_down_fast);
	int32_t dn_c = map((int32_t)input->cadence_rpm,
		IQ_RAMP_CAD_LO, IQ_RAMP_CAD_HI,
		ramp_down_slow, ramp_down_fast);
	int32_t up_ticks = (up_c < up_s) ? up_c : up_s;
	int32_t dn_ticks = (dn_c < dn_s) ? dn_c : dn_s;
#else
	int32_t up_ticks = ramp_up_slow;
	int32_t dn_ticks = ramp_down_slow;
#endif

	/*
	 * FW-048: the motor has slowed to where the commutation angle switches formula and steps.
	 * Finish here so nothing is driven through that transition — the motor coasts out on its
	 * own inertia. By this point the fade has already brought the current low, so releasing it
	 * is not felt; carrying it further is exactly what produced the clunk at standstill.
	 */
	if (input->coast_release && iq_target == 0) {
		iq_reference_q = 0;
		profile_release_step_q = 0;
		return 0;
	}

	bool profile_release_active = iq_target == 0 &&
		!input->profile_pedaling_active &&
		input->profile_release_ms > 0;
	if (profile_release_active) {
		uint16_t release_ms = input->profile_release_ms;
		if (release_ms > PROFILE_RELEASE_MAX_MS) {
			release_ms = PROFILE_RELEASE_MAX_MS;
		}
		dn_ticks = (int32_t)release_ms * CONTROL_TICKS_PER_MS;
	}

	int32_t iq_scale = input->iq_scale;
	if (iq_scale < 1) iq_scale = input->phase_current_max;
	if (iq_scale < 1) iq_scale = PH_CURRENT_MAX;
	if (iq_scale < 1) iq_scale = 1;

	int32_t target_q = iq_target << IQ_RAMP_Q_SHIFT;
	int32_t ticks = (target_q > iq_reference_q) ? up_ticks : dn_ticks;
	if (ticks < 1) ticks = 1;
	int32_t iq_scale_q = iq_scale << IQ_RAMP_Q_SHIFT;
	int32_t step_q;
	if (profile_release_active && target_q < iq_reference_q) {
		/*
		 * FW-040: latch the level this fade starts from and spread it over release_ms,
		 * so the configured time is the real time to zero FROM ANY LEVEL — deriving the
		 * step from the full current scale is what made a partial-level release finish in
		 * a fraction of the configured time.
		 *
		 * FW-072: one step for the whole fade. Rounding UP guarantees the ramp actually
		 * reaches zero within release_ms instead of crawling the last fraction of a step.
		 */
		if (profile_release_step_q == 0) {
			profile_release_step_q = (iq_reference_q + ticks - 1) / ticks;
			if (profile_release_step_q < 1) profile_release_step_q = 1;
		}
		step_q = profile_release_step_q;
	} else {
		profile_release_step_q = 0;
		step_q = (iq_scale_q + ticks - 1) / ticks;
		if (step_q < 1) step_q = 1;
	}

	if (target_q > iq_reference_q) {
		int32_t d = target_q - iq_reference_q;
		iq_reference_q += (d > step_q) ? step_q : d;
	} else if (target_q < iq_reference_q) {
		int32_t d = iq_reference_q - target_q;
		iq_reference_q -= (d > step_q) ? step_q : d;
	}

	iq_reference = (iq_reference_q + (1 << (IQ_RAMP_Q_SHIFT - 1))) >> IQ_RAMP_Q_SHIFT;
	if (iq_target == 0 && iq_reference == 0) {
		iq_reference_q = 0;
		profile_release_step_q = 0;   /* FW-040: fade finished, arm the next one */
	}
	return iq_reference;
#else
	/* FW-060: the WA controller already applies its own bounded Iq slew. */
	if (input->walk_active) {
		return iq_target;
	}
#if IQ_RAMP_ADAPTIVE
	int32_t up_s = map((int32_t)input->speed_x100,
		IQ_RAMP_SPEED_LO, IQ_RAMP_SPEED_HI, IQ_SLEW_UP_SLOW, IQ_SLEW_UP_FAST);
	int32_t up_c = map((int32_t)input->cadence_rpm,
		IQ_RAMP_CAD_LO, IQ_RAMP_CAD_HI, IQ_SLEW_UP_SLOW, IQ_SLEW_UP_FAST);
	int32_t up_step = (up_c > up_s) ? up_c : up_s;
	if (up_step < IQ_SLEW_UP_SLOW) up_step = IQ_SLEW_UP_SLOW;
	int32_t dn_step = map((int32_t)input->speed_x100,
		IQ_RAMP_SPEED_LO, IQ_RAMP_SPEED_HI, IQ_SLEW_DOWN_SLOW, IQ_SLEW_DOWN_FAST);
	if (dn_step < IQ_SLEW_DOWN_SLOW) dn_step = IQ_SLEW_DOWN_SLOW;
#else
	int32_t up_step = IQ_SLEW_UP;
	int32_t dn_step = IQ_SLEW_DOWN;
#endif

	// FW-037: safety cuts ramp (see note above), so no immediate snap here either.
	if (iq_target > iq_reference) {
		int32_t d = iq_target - iq_reference;
		return iq_reference + ((d > up_step) ? up_step : d);
	}

	int32_t d = iq_reference - iq_target;
	return iq_reference - ((d > dn_step) ? dn_step : d);
#endif
}
