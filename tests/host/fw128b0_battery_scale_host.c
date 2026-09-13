/*
 * FW-128B0: what the future battery limiter will actually receive.
 *
 * FW-128B's whole premise is that it can work in battery-current physical units and therefore
 * does not need the unknown phase-current gain K. That premise is only worth anything if the
 * battery-current number itself is understood - its equation, its units, its rounding, and the
 * rate at which it is produced. This file establishes all four, and characterises one defect it
 * found on the way.
 *
 * WHY A MODEL PLUS A GUARD. The equation lives inline in reg_ADC_processing(), which cannot be
 * linked on a host. So the arithmetic is restated exactly and then asserted, character by
 * character, against src/main.c and inc/config.h. If either drifts, this fails.
 *
 * AUDIT ONLY. Nothing here changes production behaviour, and nothing here is a fix.
 */

#include "common/check.h"
#include "../../inc/config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#ifndef MAIN_C_PATH
#error "MAIN_C_PATH must be defined (see tests/host/run-host-tests.ps1)"
#endif
#ifndef CONFIG_H_PATH
#error "CONFIG_H_PATH must be defined (see tests/host/run-host-tests.ps1)"
#endif
#ifndef BATTERY_CURRENT_C_PATH
#error "BATTERY_CURRENT_C_PATH must be defined (see tests/host/run-host-tests.ps1)"
#endif
#ifndef RIDE_CONTROL_C_PATH
#error "RIDE_CONTROL_C_PATH must be defined (see tests/host/run-host-tests.ps1)"
#endif
#define STRINGIZE_(x) #x
#define STRINGIZE(x) STRINGIZE_(x)

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

/* --- the production equation, restated exactly ---------------------------------------------
 *   battery_current_cumulated -= battery_current_cumulated >> 6;
 *   battery_current_cumulated += (adc_value[0] - bat_current_offset);
 *   MS.Battery_Current = (int32_t)((float)(battery_current_cumulated >> 6) * CAL_BAT_I);
 *
 * Note what the shift is doing: the accumulator's fixed point is 64x, so >>6 recovers x. The
 * filter therefore has DC gain exactly ONE and contributes no scale of its own - only lag.
 */
typedef struct { int32_t acc; } bat_iir_t;

static int32_t bat_step(bat_iir_t *s, int32_t adc_raw, int32_t offset)
{
	s->acc -= s->acc >> 6;
	s->acc += (adc_raw - offset);
	return (int32_t)((float)(s->acc >> 6) * CAL_BAT_I);   /* mA */
}

/* The stock-derived candidate gain, for comparison only - never used as production scale. */
#define STOCK_MA_PER_COUNT (10.0 / 255.0 * 1000.0)   /* 39.215686 mA/count */

int main(void)
{
	/* ================= A: the source really says what this file models ==================== */
	{
		char *m = read_whole_file(STRINGIZE(MAIN_C_PATH));
		char *c = read_whole_file(STRINGIZE(CONFIG_H_PATH));
		char *b = read_whole_file(STRINGIZE(BATTERY_CURRENT_C_PATH));
		CHECK(m && c && b, "A0. main.c, config.h and battery_current.c are readable");

		/* FW-128B1 moved the filter out of main.c into its own owner. The EQUATION is unchanged,
		 * so these three assertions are unchanged too - only their address is. */
		CHECK(strstr(b, "acc -= acc >> 6;") != NULL,
		      "A1. the accumulator decays by >>6 - a 64-sample exponential");
		CHECK(strstr(b, "acc += (int32_t)raw - offset;") != NULL,
		      "A2. ...and is fed the raw sample MINUS an offset - an offset, not a gain");
		CHECK(strstr(m, "MS.Battery_Current=(int32_t)((float)battery_current_filtered_adc()*CAL_BAT_I);") != NULL,
		      "A3. ...and the only multiply in the whole path is CAL_BAT_I, applied once, in main");

		/* PA0 is ADC0 regular rank 0, and the regular group is triggered by TIMER1 CH1 - i.e.
		 * by hardware at the 4 kHz timer rate, into a circular DMA. That matters for the timing
		 * finding below: the SAMPLES do not depend on the main loop; only their CONSUMPTION does. */
		CHECK(strstr(m, "adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_239POINT5); // PA0 Battery Current") != NULL,
		      "A4. battery current is ADC0 regular rank 0 = PA0");
		CHECK(strstr(m, "adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_T1_CH1);") != NULL,
		      "A5. the regular group is triggered by TIMER1 CH1 - hardware, not the main loop");
		CHECK(strstr(m, "dma_circulation_enable(DMA0, DMA_CH0);") != NULL,
		      "A6. ...and lands in adc_value[] by circular DMA, so it keeps arriving regardless");

		/* the startup zero is an OFFSET and is guarded only by a plausibility window */
		CHECK(strstr(m, "if(acc>CAL_BAT_I_OFFSET-200 && acc<CAL_BAT_I_OFFSET+200) bat_current_offset=acc;") != NULL,
		      "A7. the startup zero is accepted only inside a +/-200 count window - the ONLY guard");
		CHECK(strstr(c, "#define CAL_BAT_I 37.0") != NULL, "A8. CAL_BAT_I is 37.0 in this build");
		CHECK(strstr(c, "#define BATTERYCURRENT_MAX 15000") != NULL,
		      "A9. and the shipped battery limit is 15000 mA");

		free(m); free(c); free(b);
	}

	/* ================= B: units and the filter's own gain ================================== */
	{
		bat_iir_t s = {0};
		int32_t out = 0;
		int i;
		/* a constant 100 counts above zero, held until settled */
		for (i = 0; i < 2000; i++) out = bat_step(&s, 2035 + 100, 2035);
		CHECK(out >= (int32_t)(100 * CAL_BAT_I) - 40 && out <= (int32_t)(100 * CAL_BAT_I),
		      "B1. steady state is delta_counts * CAL_BAT_I - the filter's DC gain is exactly 1");
		printf("      [B1] 100 counts -> %d mA (100 * CAL_BAT_I = %d mA)\n",
		       (int)out, (int)(100 * CAL_BAT_I));
		CHECK(CAL_BAT_I > 1.0,
		      "B2. CAL_BAT_I is therefore in mA PER ADC COUNT - not amperes, not per volt");
	}

	/* ================= C: offset is not gain =============================================== */
	{
		bat_iir_t a = {0}, b = {0};
		int32_t oa = 0, ob = 0;
		int i;
		/* the same PHYSICAL step of 50 counts, measured from two different accepted zeros */
		for (i = 0; i < 2000; i++) {
			oa = bat_step(&a, 1900 + 50, 1900);
			ob = bat_step(&b, 2150 + 50, 2150);
		}
		CHECK(oa == ob,
		      "C1. two different startup zeros give the SAME mA for the same physical delta");
		printf("      [C1] zero 1900 -> %d mA, zero 2150 -> %d mA (identical: offset != gain)\n",
		       (int)oa, (int)ob);
		/* and the size of the window the startup guard tolerates, in amperes */
		printf("      [C2] the +/-200 count acceptance window is +/-%.2f A of contamination\n",
		       200.0 * CAL_BAT_I / 1000.0);
		CHECK(200.0 * CAL_BAT_I / 1000.0 > 5.0,
		      "C2. ...which is several AMPERES - a plausibility window, not proof of zero current");
	}

	/* ================= D: 37 versus the stock-derived 39.215686 ============================ */
	{
		static const int reported_a[5] = {5, 10, 12, 15, 20};
		double ratio = STOCK_MA_PER_COUNT / (double)CAL_BAT_I;
		int i;
		CHECK(ratio > 1.0,
		      "D1. if the stock-derived gain is right, the firmware UNDER-reports battery current");
		printf("      [D] ratio stock/EVist = %.5f  (firmware reads %.2f %% LOW)\n",
		       ratio, 100.0 * (1.0 - 1.0 / ratio));
		for (i = 0; i < 5; i++) {
			printf("      [D] firmware says %2d A -> %.2f A true  (+%.2f A)\n",
			       reported_a[i], reported_a[i] * ratio, reported_a[i] * (ratio - 1.0));
		}
		{
			double limit_a = BATTERYCURRENT_MAX / 1000.0;
			double real_at_limit = limit_a * ratio;
			printf("      [D] configured limit %.1f A -> real %.2f A (+%.2f A, +%.1f %%)\n",
			       limit_a, real_at_limit, real_at_limit - limit_a,
			       100.0 * (real_at_limit / limit_a - 1.0));
			CHECK(real_at_limit > limit_a,
			      "D2. the error direction is UNSAFE: more current flows than the rider configured");
		}
	}

	/* ================= E: the timing defect, characterised (not fixed) ===================== */
	{
		/*
		 * The samples arrive at 4 kHz in hardware. The FILTER, however, advances once per
		 * reg_ADC_processing() call. So its time constant is measured in EXECUTIONS, and its
		 * real-time constant stretches by exactly the main-loop coalescing factor - the same
		 * class of defect the PAS audit found, in the same function.
		 *
		 * This test does not assert that the behaviour is correct. It measures it, so that
		 * whatever FW-128B does about it has a number to be judged against.
		 */
		static const int every[3] = {1, 2, 5};
		int k;
		int settle[3];
		for (k = 0; k < 3; k++) {
			bat_iir_t s = {0};
			int tick;
			int target = (int)(100 * CAL_BAT_I);
			settle[k] = -1;
			for (tick = 0; tick < 4000; tick++) {
				int32_t out;
				if ((tick % every[k]) != 0) continue;      /* main did not run this tick */
				out = bat_step(&s, 2035 + 100, 2035);
				if (settle[k] < 0 && out >= (target * 63) / 100) settle[k] = tick;
			}
			printf("      [E] main every %d tick(s): 63 %% of a step reached after %d REAL ticks"
			       " (%.1f ms)\n", every[k], settle[k], settle[k] / 4.0);
		}
		CHECK(settle[0] > 0 && settle[1] > 0 && settle[2] > 0, "E0. all three runs settled");
		CHECK(settle[1] >= settle[0] * 2 - 2 && settle[2] >= settle[0] * 5 - 5,
		      "E1. the filter's REAL time constant scales with the main-loop period - it counts "
		      "executions, not time");
		CHECK(settle[0] >= 60 && settle[0] <= 70,
		      "E2. at one execution per tick it is the intended ~64 samples = ~16 ms");
	}

	/* ================= F: the battery limiter is an upstream Iq-cap, in the Iq domain ==== */
	{
		/*
		 * QS-3C: the legacy feedback-domain swap is removed. The battery limiter is now an
		 * upstream Iq-domain cap (battery_iq_cap.c), applied by ride_control.c BEFORE the one
		 * final Iq slew. The entry still compares measured mA against configured mA and stays
		 * dimensionally sound.
		 */
		char *m = read_whole_file(STRINGIZE(MAIN_C_PATH));
		char *r = read_whole_file(STRINGIZE(RIDE_CONTROL_C_PATH));
		char *l = read_whole_file(STRINGIZE(AP2_LIMITS_C_PATH));
		CHECK(m != NULL, "F0. main.c readable");
		CHECK(r != NULL, "F0b. ride_control.c readable");
		CHECK(l != NULL, "F0c. ap2_limits.c readable");
		/* Entry: the module is fed the measured mA and the configured mA with no conversion.
		 * The call moved into the one limiter chain; the units argument is the same one. */
		CHECK(strstr(l, "battery_iq_cap_update(in->battery_current_ma, in->battery_current_max,") != NULL,
		      "F1. entry feeds measured mA and configured mA - same units, no conversion");
		/* QS-3C: the legacy exit derived a PREDICTED current from the COMMAND (the exact
		 * signal this limiter never reduced - a real defect). It is gone; the new limiter
		 * exits on the MEASURED current with a 90% hysteresis band instead. The cap is applied
		 * upstream, before fast_iq_slew_publish - not as a post-slew PI clamp. */
		CHECK(strstr(r, "(MP.battery_current_max*0.9)) BC_limit_flag=0") == NULL &&
		      strstr(l, "battery_iq_cap_update(") != NULL &&
		      strstr(l, "iq_battery_cap") != NULL &&
		      strstr(r, "fast_iq_slew_publish(") != NULL &&
		      strstr(m, "PI_iq.setpoint = MP.reverse * i8_reverse_flag * MS.i_q_setpoint;") != NULL,
		      "F2. QS-3C: no command-predictor exit, no post-slew clamp - the cap is upstream "
		      "of the single final 16 kHz slew and PI_iq.setpoint comes only from MS.i_q_setpoint");
		free(l);
		free(r);
		free(m);
	}

	if (host_test_failures == 0) {
		printf("FW-128B0 battery current scale: ALL CHECKS PASSED\n");
		return 0;
	}
	printf("FW-128B0 battery current scale: %d CHECK(S) FAILED\n", host_test_failures);
	return 1;
}
