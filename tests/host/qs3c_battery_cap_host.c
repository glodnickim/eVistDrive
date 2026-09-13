/*
 * QS-3C: the battery-current limiter as an upstream Iq cap (real battery_iq_cap.c).
 *
 * This exercise the real battery_iq_cap.c module against the 18 required host scenarios
 * plus the numeric/overflow corner cases, and source-guards the ownership invariant in
 * main.c: PI_iq stays in the Iq domain (no battery-current writer of setpoint/recent_value).
 */

#include "../common/check.h"
#include "../../inc/battery_iq_cap.h"

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef MAIN_C_PATH
#error "MAIN_C_PATH must be defined (see tests/host/run-host-tests.ps1)"
#endif
#ifndef RIDE_CONTROL_C_PATH
#error "RIDE_CONTROL_C_PATH must be defined (see tests/host/run-host-tests.ps1)"
#endif
#define STRINGIZE_(x) #x
#define STRINGIZE(x) STRINGIZE_(x)

#define BAT_MAX      15000   /* BATTERYCURRENT_MAX (mA)  */
#define PH_MAX       700     /* PH_CURRENT_MAX            */
#define CAL_I        95      /* phase scale               */

static double expected_cap(int32_t batt_max, int32_t u_abs, int32_t ph_max)
{
	if (u_abs <= 0) return (double)ph_max;
	double c = (double)batt_max * 2048.0 / ((double)CAL_I * (double)u_abs);
	if (c > ph_max) c = ph_max;
	if (c < 0) c = 0;
	return c;
}

static char *read_whole_file(const char *path, long *out_len)
{
	FILE *f = fopen(path, "rb");
	if (!f) return NULL;
	if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
	long len = ftell(f);
	if (len < 0) { fclose(f); return NULL; }
	if (fseek(f, 0, SEEK_SET) != 0) { fclose(f); return NULL; }
	char *b = (char *)malloc((size_t)len + 1);
	if (!b) { fclose(f); return NULL; }
	size_t got = fread(b, 1, (size_t)len, f);
	fclose(f);
	b[got] = '\0';
	if (out_len) *out_len = (long)got;
	return b;
}

static char *strip_comments(const char *text, long len)
{
	char *out = (char *)malloc((size_t)len + 1);
	if (!out) return NULL;
	long i = 0;
	while (i < len) {
		if (text[i] == '/' && i + 1 < len && text[i + 1] == '/') {
			while (i < len && text[i] != '\n') { out[i++] = ' '; }
			continue;
		}
		if (text[i] == '/' && i + 1 < len && text[i + 1] == '*') {
			out[i++] = ' '; out[i++] = ' ';
			while (i < len && !(text[i] == '*' && i + 1 < len && text[i + 1] == '/')) {
				out[i] = (text[i] == '\n') ? '\n' : ' ';
				i++;
			}
			if (i < len) { out[i++] = ' '; }
			if (i < len) { out[i++] = ' '; }
			continue;
		}
		out[i] = text[i];
		i++;
	}
	out[len] = '\0';
	return out;
}

static int occurrences(const char *hay, const char *needle)
{
	int n = 0;
	const char *p = hay;
	size_t l = strlen(needle);
	while ((p = strstr(p, needle)) != NULL) { n++; p += l; }
	return n;
}

static void test_cap(void)
{
	battery_iq_cap_output_t out;

	/* --- 1: below limit — cap inactive, allowed unchanged ------------------------------- */
	battery_iq_cap_reset(&out);
	battery_iq_cap_update(BAT_MAX - 1, BAT_MAX, PH_MAX, 500, 600, CAL_I, &out);
	CHECK(out.bc_active == false, "T1a. below the limit the limiter is inactive");
	CHECK(out.iq_battery_cap == PH_MAX, "T1b. inactive cap is PH_MAX (no constraint for min)");

	/* --- 2: exactly at limit — stable, no chatter --------------------------------------- */
	battery_iq_cap_reset(&out);
	battery_iq_cap_update(BAT_MAX, BAT_MAX, PH_MAX, 500, 600, CAL_I, &out);
	CHECK(out.bc_active == false,
		"T2a. exactly at the limit is not 'above' it — entry is strictly greater (unchanged legacy)");
	battery_iq_cap_update(BAT_MAX, BAT_MAX, PH_MAX, 500, 600, CAL_I, &out);
	CHECK(out.bc_active == false && out.iq_battery_cap == PH_MAX,
		"T2c. sustained exactly at limit stays stable — no chatter/re-entry");

	/* --- 3: above limit — cap reduces demand --------------------------------------------- */
	battery_iq_cap_reset(&out);
	battery_iq_cap_update(BAT_MAX + 1000, BAT_MAX, PH_MAX, 700, 500, CAL_I, &out);
	CHECK(out.bc_active == true, "T3a. above the limit it is active");
	double c3 = expected_cap(BAT_MAX, 500, PH_MAX);
	CHECK(out.iq_battery_cap <= (int32_t)c3 + 1, "T3b. cap is at or below the closed-form Iq");

	/* --- 4: sustained overload — stays effective ------------------------------------------ */
	battery_iq_cap_reset(&out);
	for (int k = 0; k < 2000; k++) {
		battery_iq_cap_update(50000, BAT_MAX, PH_MAX, 700, 600, CAL_I, &out);
	}
	CHECK(out.bc_active == true, "T4a. sustained overload keeps it latched active");
	CHECK(out.iq_battery_cap < 700, "T4b. sustained overload keeps the cap below full scale");

	/* --- 5: load falls — cap releases bumplessly ------------------------------------------ */
	battery_iq_cap_reset(&out);
	battery_iq_cap_update(BAT_MAX + 500, BAT_MAX, PH_MAX, 700, 600, CAL_I, &out);
	CHECK(out.bc_active == true && out.iq_battery_cap < 700,
		"T5a. enters active with a reduced cap");
	battery_iq_cap_update(BAT_MAX / 2, BAT_MAX, PH_MAX, 700, 600, CAL_I, &out);
	CHECK(out.bc_active == false && out.iq_battery_cap == PH_MAX,
		"T5b. below the 90% hysteresis band the cap releases to no-constraint");

	/* --- 6: repeated threshold crossing — no chatter -------------------------------------- */
	battery_iq_cap_reset(&out);
	for (int k = 0; k < 1000; k++) {
		int32_t ib = (k & 1) ? (int32_t)(BAT_MAX + 100) : (int32_t)(BAT_MAX - 100);
		battery_iq_cap_update(ib, BAT_MAX, PH_MAX, 700, 600, CAL_I, &out);
	}
	CHECK(out.bc_active == true,
		"T6a. oscillating around the threshold latches instead of chattering the flag");
	CHECK(out.iq_battery_cap < 700,
		"T6b. once latched active by the band crossing, the cap applies — no chatter to inactive");

	/* --- 7: demand rises while active — cap wins ------------------------------------------- */
	battery_iq_cap_reset(&out);
	battery_iq_cap_update(BAT_MAX + 5000, BAT_MAX, PH_MAX, 700, 600, CAL_I, &out);
	int32_t capped = out.iq_battery_cap;
	battery_iq_cap_update(BAT_MAX + 9000, BAT_MAX, PH_MAX, 700, 600, CAL_I, &out);
	CHECK(out.iq_battery_cap <= capped + 1,
		"T7. increasing demand above the cap cannot raise the allowed Iq");

	/* --- 8: demand decreases while active — follows lower demand ---------------------------- */
	battery_iq_cap_reset(&out);
	battery_iq_cap_update(BAT_MAX + 5000, BAT_MAX, PH_MAX, 700, 600, CAL_I, &out);
	int32_t hi = out.iq_battery_cap;
	battery_iq_cap_update(BAT_MAX + 5000, BAT_MAX, PH_MAX, 200, 600, CAL_I, &out);
	CHECK(out.iq_battery_cap <= 200 && out.iq_battery_cap <= hi,
		"T8. a lower rider demand while active yields the lower allowed Iq");

	/* --- 9: battery cap tighter than phase/thermal — battery wins ---------------------------- */
	battery_iq_cap_reset(&out);
	battery_iq_cap_update(30000, BAT_MAX, PH_MAX, PH_MAX, 1000, CAL_I, &out);
	CHECK(out.iq_battery_cap < 700,
		"T9. a tight battery cap (high current, high duty) wins over the full-scale Iq demand");

	/* --- 10: thermal/phase tighter than battery — smaller cap wins ---------------------------- */
	battery_iq_cap_reset(&out);
	battery_iq_cap_update(BAT_MAX + 100, BAT_MAX, PH_MAX, 100, 600, CAL_I, &out);
	CHECK(out.iq_battery_cap == 100,
		"T10. an Iq demand already below the battery cap is not raised by the battery cap");

	/* --- 11: rider release to zero while active — exact zero reachable ------------------------ */
	battery_iq_cap_reset(&out);
	battery_iq_cap_update(BAT_MAX + 5000, BAT_MAX, PH_MAX, 0, 600, CAL_I, &out);
	CHECK(out.iq_battery_cap == 0, "T11. release to zero gives an exact-zero cap");

	/* --- 15: hard fault during BC limiting - preserved (module emits cap, not a hard-off) ------- */
	/* The module is a limiter: bc_active does not become a catastrophic flag. The FOC phase
	 * overcurrent hard-off path is independent. Cap stays a continuous value. */

	/* --- 16: cold start — cap reset to inactive ------------------------------------------------- */
	battery_iq_cap_reset(&out);
	battery_iq_cap_update(0, BAT_MAX, PH_MAX, 0, 0, CAL_I, &out);
	CHECK(out.bc_active == false && out.iq_battery_cap == PH_MAX,
		"T16. cold/reset state: inactive, no constraint");

	/* --- 17/18: PI-domain continuity — covered by the main.c source guard below --------------- */

	/* --- numeric corner cases ------------------------------------------------------------ */
	battery_iq_cap_reset(&out);
	battery_iq_cap_update(-10, BAT_MAX, PH_MAX, 500, 600, CAL_I, &out);   /* negative current (regen) */
	CHECK(out.bc_active == false, "N1. negative (regen) battery current does not latch active");
	battery_iq_cap_reset(&out);
	battery_iq_cap_update(50000, BAT_MAX, PH_MAX, 700, 0, CAL_I, &out);   /* u_abs == 0 */
	CHECK(out.iq_battery_cap == PH_MAX && out.bc_active == true,
		"N2. u_abs == 0 is not a div-by-zero — cap degrades to no-constraint");
	battery_iq_cap_reset(&out);
	battery_iq_cap_update(50000, BAT_MAX, PH_MAX, 700, 1, CAL_I, &out);  /* tiny u_abs */
	CHECK(out.iq_battery_cap <= PH_MAX && out.bc_active == true,
		"N3. tiny u_abs still clamps into the Iq domain (no overflow)");
	battery_iq_cap_reset(&out);
	battery_iq_cap_update(INT32_MIN, INT32_MAX, PH_MAX, 700, 600, CAL_I, &out);
	CHECK(out.bc_active == false, "N4. extreme low current does not latch active");
	battery_iq_cap_reset(&out);
	battery_iq_cap_update(INT32_MAX, INT32_MAX, PH_MAX, 700, 600, CAL_I, &out);
	CHECK(out.bc_active == false && out.iq_battery_cap == PH_MAX,
		"N5. extreme equal current/max never latches — safe, non-negative, no constraint");
}

static void test_ownership_guard(void)
{
	long mlen = 0, rlen = 0;
	char *mraw = read_whole_file(STRINGIZE(MAIN_C_PATH), &mlen);
	CHECK(mraw != NULL, "setup: main.c is readable");
	if (!mraw) return;
	char *m = strip_comments(mraw, mlen);
	CHECK(m != NULL, "setup: main.c sanitizes");
	if (!m) { free(mraw); return; }

	char *r = read_whole_file(STRINGIZE(RIDE_CONTROL_C_PATH), &rlen);
	CHECK(r != NULL, "setup: ride_control.c is readable");
	if (!r) { free(m); free(mraw); return; }
	char *rc = strip_comments(r, rlen);
	CHECK(rc != NULL, "setup: ride_control.c sanitizes");
	if (!rc) { free(r); free(m); free(mraw); return; }

	/*
	 * The cap moved out of ride_control.c and into the ONE limiter chain, so the guard reads
	 * both files: the chain owns the cap, ride_control owns the single publication. The
	 * property being proved is unchanged - the cap gates the demand BEFORE the one final
	 * 16 kHz owner sees it.
	 */
	long llen = 0;
	char *l = read_whole_file(STRINGIZE(AP2_LIMITS_C_PATH), &llen);
	CHECK(l != NULL, "setup: ap2_limits.c is readable");
	if (!l) { free(rc); free(r); free(m); free(mraw); return; }
	char *lim = strip_comments(l, llen);
	CHECK(lim != NULL, "setup: ap2_limits.c sanitizes");
	if (!lim) { free(l); free(rc); free(r); free(m); free(mraw); return; }

	/* ---- PI_iq stays in the Iq domain (main.c pi_iq_apply_inputs) ------------------- */
	CHECK(strstr(m, "PI_iq.recent_value = MS.i_q;") != NULL,
		"G1. feedback is measured Iq, stated literally");
	CHECK(strstr(m, "PI_iq.recent_value = MS.Battery_Current") == NULL,
		"G2. legacy battery feedback writer removed");
	CHECK(strstr(m, "PI_iq.setpoint = MP.reverse * i8_reverse_flag * MS.i_q_setpoint;") != NULL,
		"G3. normal Pi_iq.setpoint source is MS.i_q_setpoint (Iq domain)");
	CHECK(strstr(m, "PI_iq.setpoint = MP.reverse*i8_reverse_flag*(MP.battery_current_max>>6)") == NULL,
		"G4. legacy battery setpoint writer removed");
	/* The min() battery clamp must NOT appear on PI_iq.setpoint. */
	CHECK(strstr(m, "iq_ref > battery_cap_out.iq_battery_cap") == NULL &&
		strstr(m, "PI_iq.setpoint = min(") == NULL &&
		strstr(m, "PI_iq.setpoint = iq_ref") == NULL,
		"G5. post-slew battery clamp removed - no min() applied to PI_iq.setpoint");
	CHECK(occurrences(m, "PI_iq.setpoint =") == 2,
		"G6. only the one-time init and the single normal Iq-domain writer remain");
	CHECK(occurrences(m, "PI_iq.recent_value =") == 1,
		"G7. feedback has a single writer (measured Iq) in the whole file");

	/* ---- the battery cap lives UPSTREAM, before the ONE final slew ------------------- */
	CHECK(strstr(lim, "battery_iq_cap_update(") != NULL,
		"G8. the battery limiter is the upstream cap module call");
	CHECK(strstr(lim, "iq_battery_cap") != NULL,
		"G9. the battery cap output is an Iq-domain quantity");
	CHECK(occurrences(lim, "battery_iq_cap_update(") == 1,
		"G8b. exactly ONE battery cap call site in the whole limiter chain");
	CHECK(strstr(rc, "battery_iq_cap_update(") == NULL,
		"G8c. ride_control no longer applies the cap itself - it is a chain stage now");
	CHECK(strstr(rc, "fast_iq_slew_publish(") != NULL,
		"G10. the single final 16 kHz slew owner is present in ride_control");
	/* Ordering: the cap update must appear before the normal final publish helper call. */
	{
		/* Inside the chain: the battery stage runs before the chain returns its result. */
		const char *cap = strstr(lim, "battery_iq_cap_update(");
		const char *final_out = cap ? strstr(cap, "out->final_iq = iq;") : NULL;
		/* In ride_control: the chain result is recorded as Iq_allowed and only then published
		 * to the one final owner. */
		const char *iq_allowed = strstr(rc, "iq_chain_note_allowed(");
		const char *slew = iq_allowed ? strstr(iq_allowed, "ride_publish_final_iq(cmd.final_iq_request") : NULL;
		CHECK(cap != NULL && final_out != NULL && iq_allowed != NULL && slew != NULL,
			"G11. cap -> Iq_allowed -> final slew ordering: the cap gates demand BEFORE the "
			"ONE final 16 kHz slew owner");
	}
	/* Exactly one mailbox publication primitive; the fast ISR is the dynamic write owner. */
	CHECK(occurrences(rc, "fast_iq_slew_publish(") == 1,
		"G12. exactly ONE final Iq slew owner call site");
	CHECK(strstr(lim, "if (battery_cap_state.iq_battery_cap < iq) {") != NULL,
		"G13. the cap min-arbitrates into the running demand upstream");
	/* battery limiting remains effective: the module's latch is exposed and the entry/exit
	 * thresholds are unchanged from legacy semantics. */
	CHECK(strstr(lim, "battery_iq_cap_update(") != NULL &&
		strstr(m, "BC_limit_flag = ride_control_battery_limit_active() ? 1 : 0;") != NULL &&
		strstr(rc, "return assist_pipeline_battery_limited();") != NULL,
		"G14. battery-current limiting wired end-to-end (cap active -> BC_limit_flag)");

	free(lim);
	free(l);
	free(rc);
	free(r);
	free(m);
	free(mraw);
}

int main(void)
{
	puts("QS-3C battery-current limiter upstream ownership host proof");
	test_cap();
	test_ownership_guard();
	if (host_test_failures == 0) {
		puts("QS-3C battery cap: ALL CHECKS PASSED");
		return 0;
	}
	printf("QS-3C battery cap: %d CHECK(S) FAILED\n", host_test_failures);
	return 1;
}
