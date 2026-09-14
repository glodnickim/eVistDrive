#ifndef AP2_MATH_H_
#define AP2_MATH_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * ASSIST PIPELINE V2 - shared fixed-point helpers.
 *
 * Every block of the pipeline runs in the 4 kHz foreground control tick and must be
 * deterministic, so nothing here is floating point.
 *
 * TIMEBASE. Each helper takes `elapsed_ticks`, the number of 4 kHz control periods this
 * invocation represents. The foreground loop can coalesce periods, so a filter written
 * against the CALL COUNT silently changes its time constant whenever the loop is late.
 * (That is the FW-141 lesson and it is preserved verbatim here: elapsed time in, never
 * call count.)
 *
 * TWO KINDS OF DYNAMICS, AND ONLY TWO. The pipeline deliberately does not stack filters:
 *
 *   ap2_lpf_step()   ESTIMATOR lag. A first-order response with an explicit time constant,
 *                    used where the job is "estimate a slowly true quantity from a noisy or
 *                    pulsating one" (sensor conditioning, base load, aggression, load state,
 *                    auto factor). Each use site must be able to say what it estimates.
 *
 *   ap2_slew_step()  EXPLICIT DYNAMICS. A constant-rate move toward a target, parameterised
 *                    by the time a FULL-SCALE move would take. Used where the job is "decide
 *                    how fast the machine is allowed to change", which is a rider-facing
 *                    behaviour (attack / release / start), not a measurement.
 *
 * If a future change needs a third mechanism in the signal path, that is the signal that the
 * block it sits in is modelled wrongly - see docs/ASSIST_PIPELINE_V2.md §Filtering policy.
 */

#define AP2_TICKS_PER_MS   4U      /* the 4 kHz foreground control tick */
#define AP2_PERMILLE       1000    /* the pipeline's normalized full scale */
#define AP2_Q16_SHIFT      16
#define AP2_Q16_ONE        (1 << AP2_Q16_SHIFT)

/*
 * Q16 inputs are bounded so `value << 16` can never overflow int32_t. 32767 covers every
 * quantity the pipeline carries (permille <= 1000, centikg <= 12000, Iq <= 700).
 */
#define AP2_Q16_INPUT_MAX  32767
#define AP2_Q16_INPUT_MIN  (-32768)

static inline int32_t ap2_clamp(int32_t value, int32_t lo, int32_t hi)
{
	if (value < lo) {
		return lo;
	}
	if (value > hi) {
		return hi;
	}
	return value;
}

/* Linear interpolation with clamped ends. Self-contained: the pipeline never depends on
 * main.c's map(), so every block links and tests identically on host and target. */
static inline int32_t ap2_map(int32_t x, int32_t in_lo, int32_t in_hi,
	int32_t out_lo, int32_t out_hi)
{
	if (in_hi == in_lo) {
		return out_hi;
	}
	if (in_hi > in_lo) {
		if (x <= in_lo) {
			return out_lo;
		}
		if (x >= in_hi) {
			return out_hi;
		}
	} else {
		if (x >= in_lo) {
			return out_lo;
		}
		if (x <= in_hi) {
			return out_hi;
		}
	}
	return out_lo + (int32_t)(((int64_t)(x - in_lo) * (out_hi - out_lo)) / (in_hi - in_lo));
}

/* Scale by a permille factor with rounding away from zero-loss on small values. */
static inline int32_t ap2_scale_permille(int32_t value, int32_t permille)
{
	return (int32_t)(((int64_t)value * permille) / AP2_PERMILLE);
}

static inline int32_t ap2_scale_pct(int32_t value, int32_t pct)
{
	return (int32_t)(((int64_t)value * pct) / 100);
}

/*
 * How finely the lag integrates a long elapsed span.
 *
 * A single Euler step is only first-order accurate, so the answer it gives for one call of N
 * ticks drifts away from N calls of one tick as N grows - and at N >= tau it used to jump
 * straight to the target, which is a completely different response to the same elapsed time.
 * Integrating in sub-steps of at most tau/16 bounds that difference to about 2 % of the step,
 * for at most sixteen iterations, and costs exactly one iteration in the ordinary case where
 * the foreground is on time.
 *
 * THIS IS THE STATED TOLERANCE. "Elapsed-time invariant" means the response to a given elapsed
 * time is the same to within this bound however the time is divided into calls - not that it is
 * bit-identical.
 */
#define AP2_LPF_SUBSTEP_DIV 16U

/*
 * When a stall stops being worth integrating, in time constants.
 *
 * There has to be SOME point past which the state is simply set to the target - integrating an
 * arbitrarily long absence costs iterations for an answer that is already decided. The old code
 * put that point at exactly one time constant, where 37 % of the step is still outstanding: the
 * same elapsed time then gave 634 tick-by-tick and 1000 in one call, which is not a tolerance,
 * it is a different filter.
 *
 * Five time constants leaves under 0.7 % outstanding - below the resolution of everything this
 * pipeline carries - so the shortcut is genuinely indistinguishable from continuing.
 */
#define AP2_LPF_CATCHUP_TAUS 5U

/*
 * ESTIMATOR LAG. Advance a Q16 first-order state toward `target` with time constant
 * `tau_ms`, for `elapsed_ticks` 4 kHz periods.
 *
 *   tau_ms == 0            -> no lag, the state becomes the target
 *   elapsed >= tau_ticks   -> fully caught up. Past one time constant the remaining error is
 *                             under 37 % and shrinking, and continuing to integrate a stall
 *                             that long tells us nothing about riding: the foreground has been
 *                             away for longer than the filter's own memory.
 *
 * Returns the new Q16 state. Read it with ap2_q16_value().
 */
static inline int32_t ap2_lpf_step(int32_t state_q16, int32_t target,
	uint32_t tau_ms, uint32_t elapsed_ticks)
{
	int32_t clamped = ap2_clamp(target, AP2_Q16_INPUT_MIN, AP2_Q16_INPUT_MAX);
	int32_t target_q16 = clamped << AP2_Q16_SHIFT;
	uint32_t tau_ticks;
	uint32_t remaining;
	uint32_t max_substep;

	if (tau_ms == 0U) {
		return target_q16;
	}
	if (elapsed_ticks == 0U) {
		elapsed_ticks = 1U;
	}
	tau_ticks = tau_ms * AP2_TICKS_PER_MS;
	if (tau_ticks == 0U || elapsed_ticks >= tau_ticks * AP2_LPF_CATCHUP_TAUS) {
		return target_q16;
	}

	max_substep = tau_ticks / AP2_LPF_SUBSTEP_DIV;
	if (max_substep == 0U) {
		max_substep = 1U;
	}

	remaining = elapsed_ticks;
	while (remaining > 0U) {
		uint32_t step = (remaining < max_substep) ? remaining : max_substep;
		int64_t err = (int64_t)target_q16 - (int64_t)state_q16;
		state_q16 = (int32_t)((int64_t)state_q16 +
			(err * (int64_t)step) / (int64_t)tau_ticks);
		remaining -= step;
	}
	return state_q16;
}

static inline int32_t ap2_q16_value(int32_t state_q16)
{
	/* Arithmetic shift keeps the sign; the pipeline's signals are non-negative but the
	 * helper stays general so a derivative can use it too. */
	return state_q16 >> AP2_Q16_SHIFT;
}

static inline int32_t ap2_q16_from(int32_t value)
{
	return ap2_clamp(value, AP2_Q16_INPUT_MIN, AP2_Q16_INPUT_MAX) << AP2_Q16_SHIFT;
}

/*
 * EXPLICIT DYNAMICS. Move `value` toward `target` at the constant rate implied by
 * "a full-scale move takes rise_ms / fall_ms".
 *
 * full_scale is the span the time refers to, so the same parameter means the same thing to
 * the rider whatever domain it is applied in ("attack_ms = how long from no assist to full
 * assist"). A zero time means an immediate move, which is a legal and deliberate choice for
 * a safety zero - never an accident, because 0 is rejected at the configuration boundary for
 * every rider-facing value.
 */
static inline int32_t ap2_slew_step(int32_t value, int32_t target, int32_t full_scale,
	uint32_t rise_ms, uint32_t fall_ms, uint32_t elapsed_ticks)
{
	uint32_t ms;
	int64_t step;

	if (target == value) {
		return value;
	}
	if (elapsed_ticks == 0U) {
		elapsed_ticks = 1U;
	}
	ms = (target > value) ? rise_ms : fall_ms;
	if (ms == 0U) {
		return target;
	}
	if (full_scale < 1) {
		full_scale = 1;
	}
	step = ((int64_t)full_scale * (int64_t)elapsed_ticks) /
		((int64_t)ms * (int64_t)AP2_TICKS_PER_MS);
	if (step < 1) {
		step = 1;   /* never stall: a slow ramp must still finish */
	}
	if (target > value) {
		int64_t next = (int64_t)value + step;
		return (next > target) ? target : (int32_t)next;
	}
	{
		int64_t next = (int64_t)value - step;
		return (next < target) ? target : (int32_t)next;
	}
}

#endif /* AP2_MATH_H_ */
