/*
 * TASK-EVD-TQ-06-G1 acceptance 2: the REAL limiter (g53_g1_limiter.c), the REAL battery-current
 * sampler with its limiter tap (battery_current.c) and the REAL hard trip (battery_trip.c) in a
 * closed loop with the assumed plant of integration/evidence/evd-tq/ADR-013-prep/trip_sweep.py:
 *   battery current = command / 6500 * I_FULL, first-order lag TAU, sampled at 4 kHz,
 *   37 mA per ADC count (CAL_BAT_I), zero 2048, deterministic +-2 count noise.
 * The BDE8 tail is modelled as in that script: Q5C ramps +50 per 1-ms tick to 6500, command =
 * Q5C * g1 >> 12. The plant is an ASSUMPTION (OBSERVATION-class evidence); the arithmetic under
 * test is production code.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "battery_current.h"
#include "battery_trip.h"
#include "g53_g1_limiter.h"
#include "check.h"

#define ZERO 2048
#define MA_PER_COUNT 37.0

static uint32_t rng = 0x2468ACE1u;
static int noise2(void)
{
	rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
	return (int)(rng % 5u) - 2;
}

typedef struct {
	double peak_a, steady_a, tail_pkpk_a;
	int tripped;
} result_t;

static result_t run(double limit_a, double i_full, double tau_ms, int limiter_enabled)
{
	g53_g1_state_t s;
	result_t r = { 0, 0, 0, 0 };
	const g53_g1_config_t c = {
		.step = G53_G1_DEFAULT_STEP, .step_hi = G53_G1_DEFAULT_STEP, .p_offset = G53_G1_DEFAULT_P_OFFSET,
		.kp = G53_G1_DEFAULT_KP, .ki = G53_G1_DEFAULT_KI, .out_min = G53_G1_DEFAULT_OUT_MIN,
		.out_max = G53_G1_DEFAULT_OUT_MAX, .limit = (int16_t)(limit_a * 100.0 + 0.5),
		.knee_soc = G53_G1_DEFAULT_KNEE, .knee_temp = G53_G1_DEFAULT_KNEE, .floor = 0, .limit_reduction = 0
	};
	const g53_g1_limit_input_t li = { .bde8_state = 7, .level_pct = 100, .level_pct_state6 = 100,
		.base_select = 0, .soc_factor = G53_G1_Q12_ONE, .thermal_a = G53_G1_Q12_ONE, .thermal_b = G53_G1_Q12_ONE };
	const double a = 1.0 - exp(-0.25 / tau_ms);
	double i = 0.0, tmin = 1e9, tmax = -1e9, tsum = 0.0;
	int tn = 0, q5c = 0, cmd = 0, g1 = 0;
	const int pre_ms = 600, ms = 1500;

	g53_g1_reset(&s);
	(void)g53_g1_configure(&s, &c, 0);
	battery_current_init();
	battery_current_set_offset(ZERO);
	battery_trip_init();
	battery_trip_set_threshold_counts((int32_t)((limit_a * 1000.0 + BATTERY_TRIP_MARGIN_MA) / MA_PER_COUNT));
	for (int k = 0; k < (pre_ms + ms) * 4; k++) {
		const int t_ms = k / 4;
		if (k % 4 == 0) {
			const int dem = t_ms < pre_ms ? 0 : 6500;
			const int32_t fb = (int32_t)((double)battery_current_limiter_adc() * MA_PER_COUNT / 10.0);
			g1 = g53_g1_step(&s, G53_G1_Q12_ONE, (uint16_t)fb);
			if (!limiter_enabled) g1 = 4096;
			q5c = q5c < dem ? (q5c + 50 > dem ? dem : q5c + 50) : (q5c - 50 < dem ? dem : q5c - 50);
			cmd = battery_trip_latched() ? 0 : (q5c * g1) >> 12;
			if ((t_ms % 10) == 5) (void)g53_g1_limit_update(&s, &li);
		}
		i += a * ((double)cmd / 6500.0 * i_full - i);
		{
			long raw = lround(ZERO + i * 1000.0 / MA_PER_COUNT) + noise2();
			if (raw < 0) raw = 0;
			if (raw > 4095) raw = 4095;
			battery_current_sample((uint16_t)raw, 1U);
			(void)battery_trip_sample(battery_current_last_delta_adc());
		}
		if (t_ms >= pre_ms && i > r.peak_a) r.peak_a = i;
		if (t_ms >= pre_ms + 700) { tsum += i; tn++; if (i < tmin) tmin = i; if (i > tmax) tmax = i; }
	}
	r.steady_a = tn ? tsum / tn : 0.0;
	r.tail_pkpk_a = tmax - tmin;
	r.tripped = battery_trip_latched();
	return r;
}

int main(void)
{
	static const double limits[] = { 10.0, 15.0, 20.0 };
	static const double fulls[] = { 30.0, 45.0, 60.0, 80.0 };
	static const double taus[] = { 5.0, 20.0, 50.0 };
	char label[200];

	for (unsigned l = 0; l < 3; l++) {
		for (unsigned f = 0; f < 4; f++) {
			for (unsigned t = 0; t < 3; t++) {
				const result_t r = run(limits[l], fulls[f], taus[t], 1);
				printf("  limit %4.1f A  I_FULL %4.1f A  tau %4.1f ms : peak %5.2f  steady %5.2f  pk-pk %4.2f  trip %d\n",
					limits[l], fulls[f], taus[t], r.peak_a, r.steady_a, r.tail_pkpk_a, r.tripped);
				snprintf(label, sizeof(label), "CL limit %.0f A / I_FULL %.0f A / tau %.0f ms: settles on the limit (+-0.25 A)",
					limits[l], fulls[f], taus[t]);
				CHECK(fabs(r.steady_a - limits[l]) < 0.25, label);
				snprintf(label, sizeof(label), "CL limit %.0f A / I_FULL %.0f A / tau %.0f ms: no limit cycle (tail pk-pk < 0.3 A)",
					limits[l], fulls[f], taus[t]);
				CHECK(r.tail_pkpk_a < 0.3, label);
				snprintf(label, sizeof(label), "CL limit %.0f A / I_FULL %.0f A / tau %.0f ms: hard trip at limit + 15 A never fires",
					limits[l], fulls[f], taus[t]);
				CHECK(!r.tripped, label);
			}
		}
	}
	/* The trip is not decorative: with the limiter defeated it must fire. */
	{
		const result_t r = run(15.0, 80.0, 5.0, 0);
		CHECK(r.tripped, "CL: with g1 forced to 1.0 and 80 A available, the hard trip fires and cuts demand");
	}

	if (host_test_failures) {
		printf("TQ-06-G1 closed loop: %d CHECK(S) FAILED\n", host_test_failures);
		return 1;
	}
	printf("TQ-06-G1 closed loop: ALL CHECKS PASSED\n");
	return 0;
}
