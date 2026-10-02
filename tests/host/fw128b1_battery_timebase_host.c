/*
 * FW-128B1: the battery-current filter, against the REAL production module.
 *
 * B1-3 IS THE CARD. The same ADC sample stream is run while the "main loop" is serviced every
 * sample, every 2, 5, 10, 37, and on an irregular schedule. The filtered value after sample N
 * must be bit-identical in every run. Under the old architecture it could not be: the filter
 * advanced once per main pass and read only the latest sample, so servicing it half as often
 * halved the number of updates and doubled the real time constant.
 *
 * Everything here links src/battery_current.c directly. The IIR law is not restated in this
 * file - if it were, the test would agree with itself rather than with the firmware. What IS
 * restated is the ORIGINAL production expression, once, in B1_1, so that the new module can be
 * proved to compute exactly the same numbers as the code it replaced.
 */

#include "common/check.h"
#include "../../inc/battery_current.h"
#include "../../inc/config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#ifndef MAIN_C_PATH
#error "MAIN_C_PATH must be defined (see tests/host/run-host-tests.ps1)"
#endif
#define STRINGIZE_(x) #x
#define STRINGIZE(x) STRINGIZE_(x)

/* Comments become blanks of the same length, so a function named in prose cannot be counted as
 * a call and every offset still maps onto the original text. */
static char *strip_comments(char *text)
{
	long i = 0, len = (long)strlen(text);
	while (i < len) {
		if (text[i] == '/' && i + 1 < len && text[i + 1] == '/') {
			while (i < len && text[i] != '\n') { text[i] = ' '; i++; }
		} else if (text[i] == '/' && i + 1 < len && text[i + 1] == '*') {
			text[i] = ' '; text[i + 1] = ' '; i += 2;
			while (i < len && !(text[i] == '*' && i + 1 < len && text[i + 1] == '/')) {
				if (text[i] != '\n') text[i] = ' ';
				i++;
			}
			if (i + 1 < len) { text[i] = ' '; text[i + 1] = ' '; i += 2; }
		} else {
			i++;
		}
	}
	return text;
}

static char *read_whole_file(const char *path)
{
	FILE *f = fopen(path, "rb");
	if (!f) return NULL;
	if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
	long len = ftell(f);
	if (len < 0) { fclose(f); return NULL; }
	if (fseek(f, 0, SEEK_SET) != 0) { fclose(f); return NULL; }
	char *buf = (char *)malloc((size_t)len + 1);
	if (!buf) { fclose(f); return NULL; }
	size_t got = fread(buf, 1, (size_t)len, f);
	fclose(f);
	buf[got] = '\0';
	return buf;
}

/*
 * The ORIGINAL production filter, exactly as it read in reg_ADC_processing() before this card.
 * Present for one purpose only: to prove in B1_1 that relocating the owner did not change a
 * single number. It is not used anywhere else.
 */
typedef struct { int32_t cumulated; } legacy_t;
static int32_t legacy_step(legacy_t *l, int32_t adc_raw, int32_t off)
{
	l->cumulated -= l->cumulated >> 6;
	l->cumulated += (adc_raw - off);
	return l->cumulated >> 6;
}

#define ZERO 2035

static void armed_reset(void)
{
	battery_current_init();
	battery_current_set_offset(ZERO);
}

int main(void)
{
	/* --- B1-1: identical arithmetic to the code it replaced --------------------------------- */
	{
		legacy_t l = {0};
		int i, mismatch = 0;
		int32_t last_new = 0, last_old = 0;
		armed_reset();
		for (i = 0; i < 3000; i++) {
			/* a deliberately awkward waveform: positive, negative, and odd magnitudes, so any
			 * difference in shift/rounding behaviour would show up rather than cancel */
			int32_t raw = ZERO + ((i % 7) * 13) - ((i % 5) * 11);
			battery_current_sample((uint16_t)raw, 1U);
			last_new = battery_current_filtered_adc();
			last_old = legacy_step(&l, raw, ZERO);
			if (last_new != last_old) mismatch++;
		}
		CHECK(mismatch == 0,
		      "B1-1a. the new owner reproduces the OLD production filter sample for sample, exactly");
		printf("      [B1-1] 3000 samples, mismatches vs legacy expression: %d (last %d / %d)\n",
		       mismatch, (int)last_new, (int)last_old);
	}

	/* --- B1-1b: DC gain is exactly 1 --------------------------------------------------------- */
	{
		int i;
		armed_reset();
		for (i = 0; i < 4000; i++) battery_current_sample(ZERO + 100, 1U);
		CHECK(battery_current_filtered_adc() == 100,
		      "B1-1b. a constant delta of 100 counts settles to exactly 100 - DC gain is 1");
	}

	/* --- B1-2: one fresh sample -> exactly one filter update --------------------------------- */
	{
		int i;
		armed_reset();
		for (i = 0; i < 500; i++) battery_current_sample(ZERO + 10, 1U);
		CHECK(battery_current_get_stats()->sample_count == 500u, "B1-2a. 500 samples were offered");
		CHECK(battery_current_get_stats()->update_count == 500u,
		      "B1-2b. ...and exactly 500 filter updates happened - no sample dropped, none doubled");
	}

	/* --- B1-3: MAIN-LOOP INDEPENDENCE - the central regression test -------------------------- */
	{
		static const int service[6] = {1, 2, 5, 10, 37, -1};   /* -1 = irregular */
		int32_t result[6];
		uint32_t updates[6];
		int k, all_same = 1, all_upd_same = 1;

		for (k = 0; k < 6; k++) {
			uint32_t seed = 0xBEEF0001u;
			int n;
			armed_reset();
			for (n = 0; n < 4000; n++) {
				/* the ADC keeps producing, always - that is the hardware guarantee */
				int32_t raw = ZERO + 40 + (int32_t)((n / 400) % 3) * 25 - (int32_t)(n % 9);
				battery_current_sample((uint16_t)raw, 1U);
				/* the main loop consumes at whatever rate it manages; consuming must not
				 * change anything, which is the whole point */
				if (service[k] > 0) {
					if ((n % service[k]) == 0) (void)battery_current_filtered_adc();
				} else {
					seed = seed * 1103515245u + 12345u;
					if (((seed >> 16) & 0x3Fu) == 0u) (void)battery_current_filtered_adc();
				}
			}
			result[k] = battery_current_filtered_adc();
			updates[k] = battery_current_get_stats()->update_count;
			if (result[k] != result[0]) all_same = 0;
			if (updates[k] != updates[0]) all_upd_same = 0;
			printf("      [B1-3] consumer every %3d -> filtered %4d, updates %u\n",
			       service[k], (int)result[k], (unsigned)updates[k]);
		}
		CHECK(all_same,
		      "B1-3a. the filtered value is IDENTICAL at every consumer rate - the defect this card fixes");
		CHECK(all_upd_same,
		      "B1-3b. ...and so is the number of filter updates: 4000 samples, 4000 updates, always");
	}

	/* --- B1-4: step response is the unchanged 1/64 IIR at 4 kHz ------------------------------ */
	{
		int n, n63 = -1, n99 = -1;
		const int32_t target = 1000;
		armed_reset();
		for (n = 1; n <= 1000; n++) {
			battery_current_sample((uint16_t)(ZERO + target), 1U);
			if (n63 < 0 && battery_current_filtered_adc() >= (target * 63) / 100) n63 = n;
			if (n99 < 0 && battery_current_filtered_adc() >= (target * 99) / 100) n99 = n;
		}
		printf("      [B1-4] 63 %% after %d samples (%.2f ms), 99 %% after %d samples (%.1f ms)\n",
		       n63, n63 / 4.0, n99, n99 / 4.0);
		/* discrete pole 63/64 -> tau = -1/ln(63/64) = 63.5 samples */
		CHECK(n63 >= 60 && n63 <= 68,
		      "B1-4a. 63 % is reached in ~63.5 samples - the pole is 63/64, untouched");
		CHECK(n99 >= 270 && n99 <= 310,
		      "B1-4b. ...and 99 % in ~4.6 time constants, as an exponential must");
	}

	/* --- B1-5: signed behaviour across zero (charging / regeneration) ------------------------ */
	{
		int i;
		armed_reset();
		for (i = 0; i < 4000; i++) battery_current_sample(ZERO - 100, 1U);
		CHECK(battery_current_filtered_adc() == -100,
		      "B1-5a. a raw reading BELOW the zero settles to a negative value - charge is signed");
		for (i = 0; i < 4000; i++) battery_current_sample(ZERO + 100, 1U);
		CHECK(battery_current_filtered_adc() == 100,
		      "B1-5b. ...and it crosses back through zero to the same magnitude discharging");
	}

	/* --- B1-6: startup - no stale accumulator can exist -------------------------------------- */
	{
		int i;
		battery_current_init();
		for (i = 0; i < 500; i++) battery_current_sample(ZERO + 500, 0U);   /* before the zero is known */
		CHECK(battery_current_filtered_adc() == 0,
		      "B1-6a. nothing is filtered before the startup zero is established");
		CHECK(battery_current_get_stats()->update_count == 0u, "B1-6b. ...and no update happened");
		CHECK(battery_current_get_stats()->unarmed_count == 500u, "B1-6c. ...and the refusals are counted");

		battery_current_set_offset(ZERO);
		CHECK(battery_current_filtered_adc() == 0,
		      "B1-6d. arming starts from a deterministic zero, not from whatever was in flight");
		battery_current_sample((uint16_t)(ZERO + 64), 1U);
		CHECK(battery_current_filtered_adc() == 1,
		      "B1-6e. the very first armed sample behaves like a first sample: 64/64 = 1");
	}

	/* --- B1-6b: a different accepted zero shifts nothing but the zero ------------------------- */
	{
		int i; int32_t a, b;
		battery_current_init(); battery_current_set_offset(1900);
		for (i = 0; i < 4000; i++) battery_current_sample(1900 + 77, 1U);
		a = battery_current_filtered_adc();
		battery_current_init(); battery_current_set_offset(2150);
		for (i = 0; i < 4000; i++) battery_current_sample(2150 + 77, 1U);
		b = battery_current_filtered_adc();
		CHECK(a == b && a == 77,
		      "B1-6f. the offset moves the zero and nothing else - it is not a gain");
	}

	/* --- B1-7: integer rounding is the characterised >>6 floor ------------------------------- */
	{
		int i;
		armed_reset();
		/* a delta of 1 count: the accumulator settles at 64, and 64>>6 = 1 exactly */
		for (i = 0; i < 4000; i++) battery_current_sample(ZERO + 1, 1U);
		CHECK(battery_current_filtered_adc() == 1, "B1-7a. one count in, one count out");
		armed_reset();
		/* a negative single count: >>6 FLOORS, so -64>>6 = -1 - no truncation toward zero */
		for (i = 0; i < 4000; i++) battery_current_sample(ZERO - 1, 1U);
		CHECK(battery_current_filtered_adc() == -1,
		      "B1-7b. and the negative side is symmetric at settle - the floor bias stays sub-count");
	}

	/* --- B1-8: the sample counter and the update counter agree ------------------------------- */
	{
		int i;
		armed_reset();
		for (i = 0; i < 1234; i++) battery_current_sample(ZERO + (i % 3), 1U);
		CHECK(battery_current_get_stats()->sample_count == battery_current_get_stats()->update_count,
		      "B1-8. while armed, accepted samples and filter updates are the same number");
	}

	/* --- B1-9: a badly starved main loop changes nothing -------------------------------------- */
	{
		int n;
		int32_t reference;
		armed_reset();
		for (n = 0; n < 4000; n++) battery_current_sample((uint16_t)(ZERO + 300), 1U);
		reference = battery_current_filtered_adc();

		armed_reset();
		for (n = 0; n < 4000; n++) {
			battery_current_sample((uint16_t)(ZERO + 300), 1U);
			/* main is asleep for the first 3900 samples, then wakes up */
			if (n > 3900) (void)battery_current_filtered_adc();
		}
		CHECK(battery_current_filtered_adc() == reference,
		      "B1-9. a main loop that does not run for a whole second changes the filter not at all");
	}

	/* --- B1-10: the late-scan evidence is recorded, never used as permission ------------------ */
	{
		int i;
		armed_reset();
		for (i = 0; i < 100; i++) battery_current_sample(ZERO + 20, 0U);   /* scan not complete */
		CHECK(battery_current_get_stats()->late_scan_count == 100u,
		      "B1-10a. an incomplete scan is counted");
		CHECK(battery_current_get_stats()->update_count == 100u,
		      "B1-10b. ...and the sample is still used - rank 0 is written first, so it is valid");
	}

	/* --- B1-13: TQ-06-G1 limiter tap and raw delta (ADR-013 D2) -------------------------------- */
	{
		int i, n63 = -1;
		armed_reset();
		for (i = 0; i < 400; i++) {
			battery_current_sample((uint16_t)(ZERO + 300), 1U);
			if (n63 < 0 && battery_current_limiter_adc() >= 189) n63 = i + 1;
		}
		CHECK(battery_current_limiter_adc() == 300, "B1-13a. limiter tap has DC gain 1");
		CHECK(n63 >= 7 && n63 <= 9, "B1-13b. limiter tap reaches 63 % in about 8 samples (2 ms at 4 kHz)");
		{
			int32_t ref_acc = 0, ref = 0;
			int k;
			for (k = 0; k < 400; k++) { ref_acc -= ref_acc >> 6; ref_acc += 300; ref = ref_acc >> 6; }
			CHECK(battery_current_filtered_adc() == ref,
			      "B1-13c. the 1/64 filter equals the unchanged production law sample for sample");
		}
		battery_current_sample((uint16_t)(ZERO - 40), 1U);
		CHECK(battery_current_last_delta_adc() == -40, "B1-13d. raw delta = raw - zero, unfiltered, signed");
		for (i = 0; i < 400; i++) battery_current_sample((uint16_t)(ZERO - 1), 1U);
		CHECK(battery_current_limiter_adc() == -1, "B1-13e. limiter tap floor bias stays sub-count");
		battery_current_init();
		for (i = 0; i < 50; i++) battery_current_sample((uint16_t)(ZERO + 500), 1U);
		CHECK(battery_current_limiter_adc() == 0 && battery_current_last_delta_adc() == 0,
		      "B1-13f. unarmed: limiter tap and raw delta stay 0");
		battery_current_set_offset(ZERO);
		CHECK(battery_current_limiter_adc() == 0, "B1-13g. arming starts the limiter tap from 0");
	}

	/* --- B1-11 / B1-12: nothing else moved ---------------------------------------------------- */
	{
		char *m = read_whole_file(STRINGIZE(MAIN_C_PATH));
		CHECK(m != NULL, "G0. main.c readable");
		if (m) (void)strip_comments(m);

		/* exactly one owner: the module is fed from the ISR and nowhere else */
		{
			const char *p = strstr(m, "battery_current_sample(");
			int n = 0;
			while (p) { n++; p = strstr(p + 1, "battery_current_sample("); }
			CHECK(n == 1, "B1-2c. battery_current_sample() is called from exactly ONE place in main.c");
		}
		CHECK(strstr(m, "battery_current_cumulated-=") == NULL &&
		      strstr(m, "battery_current_cumulated+=") == NULL,
		      "B1-2d. the old main-loop filter is GONE - there is no second state owner");
		CHECK(strstr(m, "MS.Battery_Current=(int32_t)((float)battery_current_filtered_adc()*CAL_BAT_I);") != NULL,
		      "B1-2e. main only CONSUMES, and the float conversion stays outside the ISR");

		/* CAL_BAT_I is frozen by this card */
		CHECK(strstr(m, "*CAL_BAT_I") != NULL, "B1-7c. CAL_BAT_I is still the one and only scale step");

		/* PAS (3263c4e) untouched */
		CHECK(strstr(m, "pas_sampler_isr_tick(pas_ab, control_time_ticks);") != NULL,
		      "B1-11a. the PAS sampler still runs in the same ISR, from the same single pin read");
		CHECK(strstr(m, "pas_cadence_forward_step(ev.tick, cadence_interval_restart)") != NULL,
		      "B1-11b. the PAS cadence owner is untouched");
		CHECK(strstr(m, "pas_liveness_update(pas_idle_ticks, pas_stop_timeout);") != NULL,
		      "B1-11c. and so is the PAS liveness timebase");

		/* FW-127 acquisition untouched */
		CHECK(strstr(m, "sample_window_reconstruct(sample_ctx->sector,") != NULL,
		      "B1-12a. FW-127 reconstruction is untouched");
		CHECK(strstr(m, "current_feedback_update(sample_ctx->state,") != NULL,
		      "B1-12b. FW-127 sample validity is untouched");
		CHECK(strstr(m, "(void)pwm_geometry_apply(switchtime, pwm_applied, (uint16_t)_T);") != NULL,
		      "B1-12c. FW-127 applied PWM geometry is untouched");

		free(m);
	}

	if (host_test_failures == 0) {
		printf("FW-128B1 battery current timebase: ALL CHECKS PASSED\n");
		return 0;
	}
	printf("FW-128B1 battery current timebase: %d CHECK(S) FAILED\n", host_test_failures);
	return 1;
}
