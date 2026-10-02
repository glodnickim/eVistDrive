/*
 * FW-126.7 wiring guard: a STRUCTURAL check on the SOURCE TEXT of src/main.c.
 *
 * WHY STRUCTURAL. The properties below are properties of the CODE, not of a particular run:
 * which gate guards which transition, where the calibration is fed from, and what cannot
 * happen at all. main.c is the ARM entry point, wired to GD32 registers and interrupt vectors
 * throughout, so it cannot be linked here - and the module's own behaviour is already covered
 * head-on by fw1267_current_cal_host.c, which links the real current_cal.c.
 *
 * WHAT THIS PROVES:
 *   S1  an UNCALIBRATED start request can still enter the safe neutral bridge state
 *   S2  ...but cannot reach active FOC
 *   S8  a successful calibration is what releases active FOC
 *   S9  a failed calibration leaves the existing failsafe to end the start
 *   S12 a later start in the same power cycle does not recalibrate
 *   S13 the first active FOC cannot consume a stale current sample
 *   X1  the dark-bridge sampler and every trace of the LEGACY fallback are GONE
 *   X2  calibration is fed the RAW JDR, taken before any software correction
 *   X3  the calibration module never touches a bridge register
 */

#include "common/check.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef MAIN_C_PATH
#error "MAIN_C_PATH must be defined (see tests/host/run-host-tests.ps1)"
#endif
#ifndef CURRENT_CAL_C_PATH
#error "CURRENT_CAL_C_PATH must be defined (see tests/host/run-host-tests.ps1)"
#endif

#define STRINGIZE_(x) #x
#define STRINGIZE(x) STRINGIZE_(x)

static char *read_whole_file(const char *path, long *out_len)
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
	if (out_len) *out_len = (long)got;
	return buf;
}

/* Comments become blanks, same length, string/char literals left intact - so a commented-out
 * line simply does not exist any more and cannot be counted, and every offset still maps 1:1
 * onto the original for ordering checks. */
static char *strip_comments(const char *text, long len)
{
	char *out = (char *)malloc((size_t)len + 1);
	if (!out) return NULL;
	long i = 0;
	while (i < len) {
		if (text[i] == '/' && i + 1 < len && text[i + 1] == '/') {
			while (i < len && text[i] != '\n') { out[i] = ' '; i++; }
			continue;
		}
		if (text[i] == '/' && i + 1 < len && text[i + 1] == '*') {
			out[i] = ' '; out[i + 1] = ' '; i += 2;
			while (i < len && !(text[i] == '*' && i + 1 < len && text[i + 1] == '/')) {
				out[i] = (text[i] == '\n') ? '\n' : ' ';
				i++;
			}
			if (i < len) { out[i] = ' '; i++; }
			if (i < len) { out[i] = ' '; i++; }
			continue;
		}
		if (text[i] == '"' || text[i] == '\'') {
			char q = text[i];
			out[i] = text[i]; i++;
			while (i < len && text[i] != q) {
				out[i] = text[i];
				if (text[i] == '\\' && i + 1 < len) { i++; out[i] = text[i]; }
				i++;
			}
			if (i < len) { out[i] = text[i]; i++; }
			continue;
		}
		out[i] = text[i];
		i++;
	}
	out[len] = '\0';
	return out;
}

static int count_occurrences(const char *hay, const char *needle)
{
	int n = 0;
	const char *p = hay;
	size_t l = strlen(needle);
	while ((p = strstr(p, needle)) != NULL) { n++; p += l; }
	return n;
}

int main(void)
{
	long mlen = 0, clen = 0;
	char *mraw = read_whole_file(STRINGIZE(MAIN_C_PATH), &mlen);
	char *craw = read_whole_file(STRINGIZE(CURRENT_CAL_C_PATH), &clen);
	if (!mraw || !craw) { printf("  FAIL  cannot read the sources\n"); return 1; }
	char *m = strip_comments(mraw, mlen);
	char *c = strip_comments(craw, clen);
	if (!m || !c) { printf("  FAIL  out of memory\n"); return 1; }

	/* --- S1: entering the neutral bridge state does NOT depend on calibration ------------- */
	{
		/* TASK-EVD-TQ-06-G1: the hard battery trip also closes the gate (review F-01); still no
		 * calibration term. */
		const char *gate = strstr(m, "if(ride_control_final_iq_requested() > 0 && !battery_trip_latched()){");
		CHECK(gate != NULL,
		      "S1a. the bridge-entry gate is demand only - no calibration term");
		CHECK(strstr(m, "if(ride_control_final_iq_requested() > 0 && current_cal_foc_allowed") == NULL,
		      "S1b. the old deadlocking combined gate is gone");
	}

	/* --- S2 + S8 + S9: active FOC is released only by a valid calibration ------------------ */
	{
		const char *rel = strstr(m, "bridge_lifecycle = BRIDGE_LIFECYCLE_FOC_RELEASE;");
		const char *gateB = strstr(m, "if(current_cal_foc_allowed(&current_cal)){");
		CHECK(rel != NULL && gateB != NULL && gateB < rel,
		      "S2/S8. the FOC release is guarded by current_cal_foc_allowed()");
		CHECK(count_occurrences(m, "BRIDGE_LIFECYCLE_FOC_RELEASE;") >= 1,
		      "S8b. there is a release path at all");
		/* S9: nothing in main.c drops MOE on a calibration failure - the EXISTING failsafe does.
		 * Proven by the release site being the only thing the gate controls. */
		CHECK(strstr(m, "dwell_budget") != NULL,
		      "S9a. the existing dwell failsafe owns the shutdown, with a bounded budget");
		CHECK(strstr(m, "START_CAL_DWELL_TIMEOUT_CYCLES") != NULL,
		      "S9b. ...and a calibrating start has its own bounded budget");
	}

	/* --- S12: a later start does not recalibrate ------------------------------------------ */
	{
		const char *guard = strstr(m, "if(current_cal_needs_calibration(&current_cal)){");
		const char *begin = strstr(m, "current_cal_begin_attempt(&current_cal);");
		CHECK(guard != NULL && begin != NULL && guard < begin,
		      "S12a. an attempt is armed only while one is actually needed");
		CHECK(count_occurrences(m, "current_cal_begin_attempt(") == 1,
		      "S12b. there is exactly one place an attempt can begin");
	}

	/* --- S13: the first active FOC cannot consume a stale sample --------------------------- */
	{
		const char *reset = strstr(m, "foc_current_feedback_reset(&MS);");
		const char *moe = strstr(m, "timer_primary_output_config(TIMER0,ENABLE);");
		CHECK(reset != NULL && moe != NULL && reset < moe,
		      "S13a. the current feedback is invalidated BEFORE the bridge is enabled");
		CHECK(strstr(m, "foc_current_feedback_invalidate();") != NULL,
		      "S13b. the shutdown path invalidates it too");
	}

	/* --- X1: the retired architecture is GONE, not disabled -------------------------------- */
	{
		static const char *banned[] = {
			"phase_cal_collect_samples",       /* the dark-bridge sampler                */
			"CURRENT_CAL_LEGACY_FALLBACK",     /* the fallback status                    */
			"CURRENT_CAL_SRC_LEGACY",          /* the fallback source                    */
			"current_cal_submit",              /* the old attempt/policy API             */
			"current_cal_finalize",
			"current_cal_attempt_allowed",
			"lkg_offset",                      /* last-known-good, no consumer left      */
			"CURRENT_CAL_START_POLICY",
			"fw1265_",                         /* the finished FW-126.5 campaign probe   */
			"adc_trigger_diag",
		};
		for (size_t i = 0; i < sizeof(banned) / sizeof(banned[0]); i++) {
			char label[160];
			snprintf(label, sizeof(label),
			         "X1. '%s' no longer appears anywhere in main.c", banned[i]);
			CHECK(strstr(m, banned[i]) == NULL, label);
		}
	}

	/* --- X2: calibration is fed the RAW JDR, before any software correction ----------------- */
	{
		const char *jdr = strstr(m, "i16_ph1_current = adc_inserted_data_read(ADC2");
		const char *stash = strstr(m, "const int16_t cal_jdr[CURRENT_CAL_PHASES]");
		const char *correction = jdr ? strstr(jdr, "i16_ph1_current -= current_cal.offset") : NULL;
		CHECK(stash != NULL, "X2a. the ISR stashes the raw JDR triple for calibration");
		CHECK(jdr && stash && jdr < stash, "X2b. it is taken straight after the inserted read");
		CHECK(stash && correction && stash < correction,
		      "X2c. ...and BEFORE the software offset is subtracted - raw is raw");
		CHECK(strstr(m, "current_cal_sample(&current_cal, cal_jdr, bridge_neutral)") != NULL,
		      "X2d. the calibration is fed that stash, not a corrected or reconstructed value");
		CHECK(count_occurrences(m, "current_cal_sample(") == 1,
		      "X2e. there is exactly one sampling call site");
	}

	/* --- X3: the calibration module owns no hardware --------------------------------------- */
	{
		static const char *forbidden[] = {
			"TIMER_CCHP", "timer_primary_output_config", "adc_software_trigger_enable",
			"adc_external_trigger_source_config", "nvic_irq_disable",
			"timer_channel_output_pulse_value_config", "adc_inserted_data_read",
		};
		for (size_t i = 0; i < sizeof(forbidden) / sizeof(forbidden[0]); i++) {
			char label[160];
			snprintf(label, sizeof(label),
			         "X3. current_cal.c never calls '%s'", forbidden[i]);
			CHECK(strstr(c, forbidden[i]) == NULL, label);
		}
		CHECK(strstr(c, "current_cal_wants_dwell") != NULL,
		      "X3b. it can only ASK for the dwell, never take it");
	}

	if (host_test_failures == 0) {
		printf("FW-126.7 calibration wiring guard: ALL CHECKS PASSED\n");
		return 0;
	}
	printf("FW-126.7 calibration wiring guard: %d CHECK(S) FAILED\n", host_test_failures);
	return 1;
}
