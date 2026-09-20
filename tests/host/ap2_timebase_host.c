/*
 * ASSIST PIPELINE V2 - elapsed-time invariance.
 *
 * inc/ap2_math.h promises that a block's response depends on ELAPSED TIME, not on how many
 * times the foreground happened to call it. The foreground can coalesce 4 kHz periods, so a
 * filter that counts calls silently changes its own time constant whenever the loop is late -
 * and every ride-feel number tuned against it becomes a number tuned against the scheduler.
 *
 * A promise like that is worth exactly as much as its measurement, so this suite measures it:
 * the same signal over the same total time, divided into 1, 2, 4, 8, 16, 40 and 80-tick calls,
 * plus a jittered division, must give the same answer within the tolerance the header states.
 *
 * IT IS NOT A BIT-EXACTNESS TEST. A first-order lag integrated in finite sub-steps cannot be
 * bit-identical across divisions, and demanding that would either be a lie or force a far more
 * expensive integrator into a 4 kHz path. AP2_LPF_SUBSTEP_DIV fixes the bound; this suite holds
 * the implementation to it.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ap2_estimators.h"
#include "ap2_math.h"
#include "ap2_rider_demand.h"

static int failures;

#define CHECK(cond, what) do { \
	if (!(cond)) { \
		printf("  FAIL  %s\n", (what)); \
		failures++; \
	} \
} while (0)

/*
 * The tolerance implied by AP2_LPF_SUBSTEP_DIV sub-steps, as a percentage of full scale.
 *
 * Integrating in steps of at most tau/16 leaves at most a few percent of difference against the
 * tick-by-tick answer for a full-scale step - the worst case this suite drives. Anything inside
 * this band is the documented numerical cost of a cheap integrator; anything outside means the
 * response has started to depend on the scheduler again.
 */
#define TIMEBASE_TOLERANCE_PERMILLE 40

static const uint32_t DIVISIONS[] = { 1U, 2U, 4U, 8U, 16U, 40U, 80U };
#define DIVISION_COUNT (sizeof(DIVISIONS) / sizeof(DIVISIONS[0]))

/* ------------------------------------------------------------------ the lag itself -------- */

static int32_t lpf_over(uint32_t total_ticks, uint32_t chunk, uint32_t tau_ms, int32_t target)
{
	int32_t s = 0;
	uint32_t t;

	for (t = 0; t < total_ticks; t += chunk) {
		uint32_t step = (total_ticks - t < chunk) ? (total_ticks - t) : chunk;
		s = ap2_lpf_step(s, target, tau_ms, step);
	}
	return ap2_q16_value(s);
}

static void test_lpf_invariance(void)
{
	static const uint32_t TAUS[] = { 20U, 120U, 400U, 1500U, 6000U };
	unsigned ti;

	printf("T1 the lag responds to elapsed time, not to call count\n");

	for (ti = 0; ti < sizeof(TAUS) / sizeof(TAUS[0]); ti++) {
		uint32_t tau = TAUS[ti];
		/* One time constant of elapsed time: the steepest part of the response, where any
		 * dependence on the division shows up most clearly. */
		uint32_t total = tau * AP2_TICKS_PER_MS;
		int32_t reference = lpf_over(total, 1U, tau, AP2_PERMILLE);
		unsigned d;
		char label[160];

		snprintf(label, sizeof(label),
			"T1[tau=%u ms]: the tick-by-tick answer is a real first-order response (~632)", tau);
		CHECK(reference > 560 && reference < 700, label);

		for (d = 1; d < DIVISION_COUNT; d++) {
			uint32_t chunk = DIVISIONS[d];
			int32_t got;
			int32_t diff;
			if (chunk > total) {
				continue;
			}
			got = lpf_over(total, chunk, tau, AP2_PERMILLE);
			diff = got - reference;
			if (diff < 0) {
				diff = -diff;
			}
			snprintf(label, sizeof(label),
				"T1[tau=%u ms]: %u-tick calls give %d against %d tick-by-tick "
				"(inside the stated tolerance)", tau, chunk, got, reference);
			CHECK(diff <= TIMEBASE_TOLERANCE_PERMILLE, label);
		}
	}
}

static void test_lpf_jitter(void)
{
	uint32_t tau = 120U;
	uint32_t total = tau * AP2_TICKS_PER_MS;
	int32_t reference = lpf_over(total, 1U, tau, AP2_PERMILLE);
	/* A deliberately irregular division - what a real foreground actually does. */
	static const uint32_t PATTERN[] = { 1U, 1U, 3U, 1U, 9U, 2U, 1U, 21U, 1U, 4U };
	int32_t s = 0;
	uint32_t done = 0U;
	unsigned i = 0;
	int32_t got;
	int32_t diff;

	printf("T2 an irregular division gives the same answer as a regular one\n");

	while (done < total) {
		uint32_t step = PATTERN[i % (sizeof(PATTERN) / sizeof(PATTERN[0]))];
		if (done + step > total) {
			step = total - done;
		}
		s = ap2_lpf_step(s, AP2_PERMILLE, tau, step);
		done += step;
		i++;
	}
	got = ap2_q16_value(s);
	diff = got - reference;
	if (diff < 0) {
		diff = -diff;
	}
	CHECK(diff <= TIMEBASE_TOLERANCE_PERMILLE,
		"T2: a jittered foreground reaches the same state as an on-time one");
}

/* ------------------------------------------------------------------ the demand model ------ */

static int32_t demand_over(uint32_t total_ticks, uint32_t chunk, uint16_t load_ctrl,
	int32_t *base_out)
{
	ap2_demand_input_t in;
	ap2_demand_output_t out;
	uint32_t t;

	ap2_rider_demand_reset();
	memset(&in, 0, sizeof(in));
	memset(&out, 0, sizeof(out));
	in.load_ctrl = load_ctrl;
	in.torque_valid = true;
	in.pedaling = true;
	in.cadence_rpm = 70U;
	in.full_scale_ctrl = 6000U;
	in.base_hold_ms = 350U;

	for (t = 0; t < total_ticks; t += chunk) {
		uint32_t step = (total_ticks - t < chunk) ? (total_ticks - t) : chunk;
		in.elapsed_ticks = step;
		ap2_rider_demand_update(&in, &out);
	}
	*base_out = out.base_permille;
	return out.demand_permille;
}

static void test_demand_invariance(void)
{
	const uint32_t total = 400U * AP2_TICKS_PER_MS;   /* 400 ms of steady pressure */
	int32_t ref_base = 0;
	int32_t ref_demand = demand_over(total, 1U, 1500U, &ref_base);
	unsigned d;

	printf("T3 the demand model is elapsed-time invariant end to end\n");
	CHECK(ref_demand > 0 && ref_base > 0, "T3: setup - a steady pressure produces a demand");

	for (d = 1; d < DIVISION_COUNT; d++) {
		uint32_t chunk = DIVISIONS[d];
		int32_t base = 0;
		int32_t demand = demand_over(total, chunk, 1500U, &base);
		int32_t dd = demand - ref_demand;
		int32_t db = base - ref_base;
		char label[160];
		if (dd < 0) {
			dd = -dd;
		}
		if (db < 0) {
			db = -db;
		}
		snprintf(label, sizeof(label),
			"T3: %u-tick calls give demand %d / base %d against %d / %d tick-by-tick",
			chunk, demand, base, ref_demand, ref_base);
		CHECK(dd <= TIMEBASE_TOLERANCE_PERMILLE && db <= TIMEBASE_TOLERANCE_PERMILLE, label);
	}
}

/* ------------------------------------------------------------------ the derivatives ------- */

static int32_t aggression_rate_over(uint32_t chunk)
{
	ap2_estimator_input_t in;
	ap2_estimator_output_t out;
	uint32_t t;
	const uint32_t total = 400U * AP2_TICKS_PER_MS;

	ap2_estimators_reset();
	memset(&in, 0, sizeof(in));
	memset(&out, 0, sizeof(out));
	in.pedaling = true;
	in.cadence_rpm = 70U;
	in.speed_x100 = 1500U;

	for (t = 0; t < total; t += chunk) {
		uint32_t step = (total - t < chunk) ? (total - t) : chunk;
		/* A demand ramping at a fixed rate in TIME: 500 permille over 400 ms. */
		in.demand_permille = (int32_t)((t * 500U) / total);
		in.base_permille = in.demand_permille;
		in.stroke_peak_permille = in.demand_permille;
		in.elapsed_ticks = step;
		ap2_estimators_update(&in, &out);
	}
	return out.aggr_rate_evidence;
}

static void test_rate_normalisation(void)
{
	int32_t reference = aggression_rate_over(1U);
	unsigned d;

	printf("T4 a derivative is normalised by the time that actually passed\n");

	/*
	 * The rider ramps the demand at a FIXED RATE IN TIME. Whatever the foreground does with its
	 * scheduling, the measured rate must describe the rider. Dividing a window's difference by
	 * the nominal window length instead of the elapsed one inflated exactly this number when
	 * the foreground ran late.
	 */
	for (d = 1; d < DIVISION_COUNT; d++) {
		uint32_t chunk = DIVISIONS[d];
		int32_t got = aggression_rate_over(chunk);
		int32_t diff = got - reference;
		char label[160];
		if (diff < 0) {
			diff = -diff;
		}
		snprintf(label, sizeof(label),
			"T4: %u-tick calls measure rate evidence %d against %d tick-by-tick",
			chunk, got, reference);
		CHECK(diff <= 120, label);
	}
}

int main(void)
{
	puts("Assist Pipeline V2 timebase invariance");
	test_lpf_invariance();
	test_lpf_jitter();
	test_demand_invariance();
	test_rate_normalisation();

	if (failures == 0) {
		puts("Assist Pipeline V2 timebase: ALL CHECKS PASSED");
		return 0;
	}
	printf("Assist Pipeline V2 timebase: %d CHECK(S) FAILED\n", failures);
	return 1;
}
