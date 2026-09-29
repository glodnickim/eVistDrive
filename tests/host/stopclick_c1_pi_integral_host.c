/*
 * STOP-CLICK-C1: PI D/Q integrator continuity through ordinary zero torque.
 *
 * Before this card, main.c's reg_ADC_processing() (4 kHz foreground) periodically zeroed
 * PI_iq.integral_part/PI_id.integral_part whenever MS.i_q_setpoint==0, while runPIcontrol()
 * (16 kHz FOC ISR, FOC.c's PI_control()) kept accumulating into the SAME fields every cycle.
 * Two asynchronous writers of one piece of state - the believed source of the audible click at
 * the final release into ARMED_ZERO (see the frozen G532 reference: ordinary zero torque does
 * not reset PI there either). The fix deletes both foreground reset sites; PI_iq/PI_id integral
 * state during RUN/ARMED_ZERO is now owned exclusively by the 16 kHz PI.
 *
 * main.c and FOC.c are the ARM entry point / ISR core and cannot be linked on a host (same
 * reasoning as every other main.c/FOC.c guard in this suite, e.g. armed_zero_lifecycle_host.c).
 * Two complementary techniques cover the card's T1-T10 and the zero-reference boundedness
 * section:
 *
 *   - a byte-faithful REPLICA of FOC.c's real PI_control() (model_pi_control() below), used to
 *     exercise the actual regulator mathematics (T2, T3, boundedness) - not a fictional stand-in;
 *   - source-text guards against main.c/FOC.c (production_wiring_checks(), below, mirrors the
 *     read/strip/goto-done shape armed_zero_lifecycle_host.c already uses) proving the foreground
 *     writers are gone and every remaining explicit reset site (cold PREPARE, dwell-timeout
 *     failsafe, hard fault, hall calibration) is unchanged.
 *
 * A source guard pins the replica's formula against FOC.c's actual PI_control() body, so a
 * future change to the real algorithm that the replica does not track breaks this suite loudly
 * instead of silently going stale.
 */

#include "../common/check.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STRINGIZE2(x) #x
#define STRINGIZE(x) STRINGIZE2(x)

#ifndef MAIN_C_PATH
#error "MAIN_C_PATH must be supplied by run-host-tests.ps1"
#endif
#ifndef FOC_C_PATH
#error "FOC_C_PATH must be supplied by run-host-tests.ps1"
#endif

/* ------------------------------------------------------------------------------------------- */
/* Byte-faithful replica of FOC.c's PI_control(). Field names/types match PI_control_t          */
/* (inc/main.h); the body is a direct transcription of FOC.c lines ~209-233, including its       */
/* implicit float->int32 truncation on the non-slew-limited output assignment.                   */
/* ------------------------------------------------------------------------------------------- */

typedef struct {
	float   gain_p;
	float   gain_i;
	int16_t limit_i;
	int16_t limit_output;
	int16_t recent_value;
	int32_t setpoint;
	float   integral_part;
	int16_t max_step;
	int32_t out;
	/* FOC-AW1 added tracking anti-windup to the same body. This suite's claim is about the
	 * integrator's CONTINUITY through ordinary zero torque, not about tracking, so the replica
	 * carries the fields and leaves aw_inv_kp_q15 at 0 - which makes the correction an exact
	 * zero and the arithmetic below bit-identical to the pre-FOC-AW1 regulator this suite was
	 * written against. FOC-AW1's own behaviour is covered by focaw1_tracking_aw_host.c. */
	int32_t aw_sat_error;
	int32_t aw_inv_kp_q15;
} model_pi_t;

/* Representative of PI_iq's production init (main.c ~line 970-976 / inc/config.h
 * P_FACTOR_I_Q, I_FACTOR_I_Q): gain_p=1.5, gain_i=0.01, limit_i=limit_output=_U_MAX=1920,
 * max_step=15. Hardcoded here, not included from config.h/FOC.h, for the same reason
 * armed_zero_lifecycle_host.c hardcodes MODEL_POWER_STAGE_STOP_TICKS: those headers pull in
 * hardware-dependent context this host harness does not have. */
#define MODEL_GAIN_P     1.5f
#define MODEL_GAIN_I     0.01f
#define MODEL_LIMIT_I    1920
#define MODEL_LIMIT_OUT  1920
#define MODEL_MAX_STEP   15

static void model_pi_init(model_pi_t *pi)
{
	memset(pi, 0, sizeof(*pi));
	pi->gain_p = MODEL_GAIN_P;
	pi->gain_i = MODEL_GAIN_I;
	pi->limit_i = MODEL_LIMIT_I;
	pi->limit_output = MODEL_LIMIT_OUT;
	pi->max_step = MODEL_MAX_STEP;
}

static int32_t model_pi_control(model_pi_t *PI_c)
{
	float Delta = (float)(PI_c->setpoint - PI_c->recent_value);
	float p_part = Delta * PI_c->gain_p;
	float aw_part = (float)((PI_c->aw_sat_error * PI_c->aw_inv_kp_q15) >> 15);
	PI_c->integral_part += (Delta - aw_part) * PI_c->gain_i;

	if (PI_c->integral_part > PI_c->limit_i) PI_c->integral_part = PI_c->limit_i;
	if (PI_c->integral_part < -(PI_c->limit_i)) PI_c->integral_part = -(PI_c->limit_i);

	if (p_part + PI_c->integral_part > PI_c->out + PI_c->max_step) PI_c->out += PI_c->max_step;
	else if (p_part + PI_c->integral_part < PI_c->out - PI_c->max_step) PI_c->out -= PI_c->max_step;
	else PI_c->out = (int32_t)(p_part + PI_c->integral_part);

	if (PI_c->out > PI_c->limit_output) PI_c->out = PI_c->limit_output;
	if (PI_c->out < -(PI_c->limit_output)) PI_c->out = -(PI_c->limit_output);

	return PI_c->out;
}

static void model_behavior_checks(void)
{
	/* ---- T2: PI remains active at zero reference (real algorithm, non-zero measured Iq) ---- */
	{
		model_pi_t pi; model_pi_init(&pi);
		pi.setpoint = 0;
		pi.recent_value = 400; /* measured Iq residual the regulator must still chase to zero */
		float before = pi.integral_part;
		for (int i = 0; i < 50; ++i) model_pi_control(&pi);
		CHECK(pi.integral_part != before,
			"T2: PI_iq integral changes under the real PI algorithm at Iq_ref=0 with residual Iq");
		CHECK(pi.integral_part < 0.0f,
			"T2: integral moves the correct (opposing) sign against a positive measured residual");
	}

	/* ---- T3: exact zero error holds normal PI state (no external forced reset) ---- */
	{
		model_pi_t pi; model_pi_init(&pi);
		pi.setpoint = 0;
		pi.recent_value = 0;
		pi.integral_part = 733.0f; /* known non-zero value, as the card specifies */
		for (int i = 0; i < 20; ++i) model_pi_control(&pi);
		CHECK(pi.integral_part == 733.0f,
			"T3: Delta==0 leaves the integral exactly where the real PI math puts it (unchanged), not force-reset");
	}

	/* ---- Section 12: zero-reference boundedness (existing clamps only, no new anti-windup) --- */
	{
		model_pi_t pi; model_pi_init(&pi);
		pi.setpoint = 0;
		int32_t noise_pattern[] = {5, -5, 0, 3, -8, 12, -12, 1, -1, 0};
		size_t pattern_len = sizeof(noise_pattern) / sizeof(noise_pattern[0]);
		for (long i = 0; i < 50000L; ++i) {
			pi.recent_value = (int16_t)noise_pattern[(size_t)i % pattern_len];
			model_pi_control(&pi);
			CHECK(isfinite(pi.integral_part),
				"boundedness: PI_iq integral stays finite (no overflow/NaN) under alternating residual noise at zero reference");
			CHECK(pi.integral_part <= (float)MODEL_LIMIT_I && pi.integral_part >= -(float)MODEL_LIMIT_I,
				"boundedness: PI_iq integral stays within its existing clamp at zero reference");
			CHECK(pi.out <= MODEL_LIMIT_OUT && pi.out >= -MODEL_LIMIT_OUT,
				"boundedness: PI_iq output stays within its existing clamp at zero reference");
		}

		/* sustained positive residual must not run the integral to the wrong-sign rail */
		model_pi_t pi2; model_pi_init(&pi2);
		pi2.setpoint = 0;
		pi2.recent_value = 50;
		for (int i = 0; i < 100000; ++i) model_pi_control(&pi2);
		CHECK(pi2.integral_part <= 0.0f && pi2.integral_part >= -(float)MODEL_LIMIT_I,
			"boundedness: sustained positive residual settles on the correct-sign rail, never the wrong one");

		/* sustained negative residual: mirror image, must settle on the opposite correct rail */
		model_pi_t pi3; model_pi_init(&pi3);
		pi3.setpoint = 0;
		pi3.recent_value = -50;
		for (int i = 0; i < 100000; ++i) model_pi_control(&pi3);
		CHECK(pi3.integral_part >= 0.0f && pi3.integral_part <= (float)MODEL_LIMIT_I,
			"boundedness: sustained negative residual settles on the correct-sign rail, never the wrong one");
	}
}

/* ------------------------------------------------------------------------------------------- */
/* Production source-text checks: main.c and FOC.c are the ARM entry point / ISR core and       */
/* cannot be linked on a host - same reasoning and same read/strip/goto-done shape as            */
/* armed_zero_lifecycle_host.c's production_wiring_checks(). Covers T1, T4-T10.                 */
/* ------------------------------------------------------------------------------------------- */

static char *read_whole_file(const char *path, long *out_len)
{
	FILE *f = fopen(path, "rb");
	if (!f) return NULL;
	if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
	long len = ftell(f);
	if (len < 0 || fseek(f, 0, SEEK_SET) != 0) { fclose(f); return NULL; }
	char *text = (char *)malloc((size_t)len + 1U);
	if (!text) { fclose(f); return NULL; }
	size_t got = fread(text, 1, (size_t)len, f);
	fclose(f);
	text[got] = '\0';
	if (out_len) *out_len = (long)got;
	return text;
}

static char *strip_comments(const char *text, long len)
{
	char *out = (char *)malloc((size_t)len + 1U);
	if (!out) return NULL;
	long i = 0;
	while (i < len) {
		if (text[i] == '/' && i + 1 < len && text[i + 1] == '/') {
			while (i < len && text[i] != '\n') { out[i] = ' '; i++; }
			continue;
		}
		if (text[i] == '/' && i + 1 < len && text[i + 1] == '*') {
			out[i++] = ' '; out[i++] = ' ';
			while (i < len && !(text[i] == '*' && i + 1 < len && text[i + 1] == '/')) {
				out[i] = (text[i] == '\n') ? '\n' : ' ';
				i++;
			}
			if (i < len) out[i++] = ' ';
			if (i < len) out[i++] = ' ';
			continue;
		}
		out[i] = text[i];
		i++;
	}
	out[len] = '\0';
	return out;
}

/* CRLF-tolerant span-open locator: finds `signature` immediately followed by an
 * optional '\r', then '\n', then '{' - so a "void f(void)\n{" anchor locates the
 * function body on both LF and CRLF checkouts (Windows git autocrlf yields \r\n).
 * Returns a pointer to the signature start (identical to the old strstr match), or
 * NULL. Buffers stay byte-identical, so raw/stripped offset arithmetic stays 1:1. */
static const char *find_span_open(const char *haystack, const char *signature)
{
	const char *p = strstr(haystack, signature);
	while (p) {
		const char *q = p + strlen(signature);
		if (*q == '\r') q++;
		if (*q == '\n') q++;
		if (*q == '{') return p;
		p = strstr(p + 1, signature);
	}
	return NULL;
}

static bool span_contains(const char *first, const char *last, const char *needle)
{
	const char *found = strstr(first, needle);
	return found != NULL && found < last;
}

/* Counts non-overlapping occurrences of `needle` in [first, last). Used by the T10 single-writer
 * guard, which must distinguish an ASSIGNMENT ("integral_part=") from the two legitimate
 * diagnostic READS inside reg_ADC_processing() (e.g. "(int32_t)PI_iq.integral_part)"), which do
 * not match the assignment needle at all. */
static int count_in_span(const char *first, const char *last, const char *needle)
{
	int count = 0;
	size_t needle_len = strlen(needle);
	const char *p = first;
	for (;;) {
		const char *found = strstr(p, needle);
		if (!found || found >= last) break;
		count++;
		p = found + needle_len;
	}
	return count;
}

static void production_wiring_checks(void)
{
	long main_len = 0, foc_len = 0;
	char *main_raw = read_whole_file(STRINGIZE(MAIN_C_PATH), &main_len);
	char *foc_raw = read_whole_file(STRINGIZE(FOC_C_PATH), &foc_len);
	CHECK(main_raw != NULL && foc_raw != NULL, "setup: main.c and FOC.c are readable");
	if (!main_raw || !foc_raw) goto done;

	char *main_c = strip_comments(main_raw, main_len);
	char *foc_c = strip_comments(foc_raw, foc_len);
	CHECK(main_c != NULL && foc_c != NULL, "setup: main.c/FOC.c sanitize successfully");
	if (!main_c || !foc_c) { free(main_c); free(foc_c); goto done; }

	/* Model-matches-production guard: the replica's math must still track FOC.c's real
	 * PI_control() body. If this ever fails, model_pi_control() above is stale, not this guard. */
	CHECK(strstr(foc_c, "PI_c->integral_part += (Delta - aw_part)*PI_c->gain_i;") != NULL &&
		strstr(foc_c, "float aw_part = (float)((PI_c->aw_sat_error * PI_c->aw_inv_kp_q15) >> 15);") != NULL &&
		strstr(foc_c, "if (PI_c->integral_part > PI_c->limit_i) PI_c->integral_part = PI_c->limit_i;") != NULL &&
		strstr(foc_c, "if (PI_c->integral_part < -(PI_c->limit_i)) PI_c->integral_part = -(PI_c->limit_i);") != NULL &&
		strstr(foc_c, "else PI_c->out=(p_part+PI_c->integral_part);") != NULL,
		"setup: model_pi_control() still matches FOC.c's real PI_control() body");

	/* T1 / T10: reg_ADC_processing() (4 kHz foreground) is the sole domain the card targets.
	 * Locate its exact span and prove zero integral_part ASSIGNMENTS remain inside it - the two
	 * diagnostic reads (diag_clamp16((int32_t)PI_iq.integral_part)) do not match the assignment
	 * needle and are correctly left alone. */
	{
		/* Anchored on the DEFINITION, not the forward-declared prototype (main.c ~line 174/176
		 * declares both "void reg_ADC_processing(void);" and
		 * "int16_t internal_tics_to_speedx100 (uint32_t tics);" long before either body - a bare
		 * substring match without the brace/semicolon distinction finds the prototypes first and
		 * silently collapses this span to two adjacent declaration lines). */
		const char *reg_adc_start = find_span_open(main_c, "void reg_ADC_processing(void)");
		const char *reg_adc_end = strstr(main_c, "int16_t internal_tics_to_speedx100 (uint32_t tics){");
		CHECK(reg_adc_start != NULL && reg_adc_end != NULL && reg_adc_start < reg_adc_end,
			"setup: reg_ADC_processing() span is locatable");
		if (reg_adc_start && reg_adc_end && reg_adc_start < reg_adc_end) {
			CHECK(count_in_span(reg_adc_start, reg_adc_end, "PI_iq.integral_part=") == 0 &&
				count_in_span(reg_adc_start, reg_adc_end, "PI_id.integral_part=") == 0,
				"T1/T10: reg_ADC_processing() (4 kHz foreground) has zero PI_iq/PI_id.integral_part writers");
			CHECK(!span_contains(reg_adc_start, reg_adc_end, "walk_assist_iq_request"),
				"setup: reg_ADC_processing() span boundary is correct (walk_assist function falls outside it)");
			/* "STOP-CLICK-C1" lives in a comment, which strip_comments() blanks out of main_c -
			 * check the corresponding offset in the RAW (unstripped) text instead. strip_comments
			 * preserves length and position 1:1 (comment bytes become spaces/newlines, never
			 * removed), so the same pointer offset into main_raw lands on the same source span. */
			const char *raw_start = main_raw + (reg_adc_start - main_c);
			const char *raw_end = main_raw + (reg_adc_end - main_c);
			CHECK(span_contains(raw_start, raw_end, "STOP-CLICK-C1"),
				"T1: the removal is documented in place, not silently deleted");
		}
	}

	/* T4 / T6: neither RUN->ARMED_ZERO nor ARMED_ZERO->RUN causes a PI integral reset. Same
	 * anchor strings armed_zero_lifecycle_host.c already relies on for the MOE/dwell guarantee;
	 * this suite adds the independent PI-integral-specific claim over the identical span. */
	{
		const char *zero_transition = strstr(main_c,
			"bridge_lifecycle == BRIDGE_LIFECYCLE_RUN && MS.i_q_setpoint == 0");
		const char *armed_transition = strstr(main_c,
			"bridge_lifecycle == BRIDGE_LIFECYCLE_ARMED_ZERO && MS.i_q_setpoint > 0");
		const char *dwell_failsafe = strstr(main_c, "static uint16_t dwell_timeout_counter");
		CHECK(zero_transition != NULL && armed_transition != NULL && dwell_failsafe != NULL &&
			zero_transition < armed_transition && armed_transition < dwell_failsafe,
			"setup: RUN<->ARMED_ZERO transition span is locatable");
		if (zero_transition && dwell_failsafe && zero_transition < dwell_failsafe) {
			CHECK(count_in_span(zero_transition, dwell_failsafe, "PI_iq.integral_part=") == 0 &&
				count_in_span(zero_transition, dwell_failsafe, "PI_id.integral_part=") == 0,
				"T4/T6: neither RUN->ARMED_ZERO nor ARMED_ZERO->RUN resets the PI integrators");
		}

		/* T7: cold PREPARE reset preserved (GATE A: bridge fully off, !ui_8_PWM_ON_Flag, before
		 * MOE ON). */
		const char *cold_prepare = strstr(main_c, "if(!ui_8_PWM_ON_Flag){");
		CHECK(cold_prepare != NULL, "setup: cold PREPARE block (GATE A) is locatable");
		if (cold_prepare) {
			/* ~3480 chars of (heavily commented) source separate the two anchors - measured
			 * against the shipped file, span sized with headroom rather than tight to it.
			 * FOC-AW1 added its foc_aw_tracking_reset() call and comment inside this same
			 * block, and QZERO added its quiet_zero_reset() next to it, which is why the span
			 * is 4200 rather than the original 3400 (the anchor now sits at 3817). */
			CHECK(span_contains(cold_prepare, cold_prepare + 4200, "PI_iq.integral_part=0; PI_iq.out=0;") &&
				span_contains(cold_prepare, cold_prepare + 4200, "PI_id.integral_part=0; PI_id.out=0;") &&
				span_contains(cold_prepare, cold_prepare + 4200, "bridge_lifecycle = BRIDGE_LIFECYCLE_MOE_ON;"),
				"T7: cold PREPARE still zeroes both PI integrators before a fresh bridge-on");
		}

		/* T7b: the dwell-timeout failsafe (bridge shutting down to IDLE) also keeps its reset -
		 * FOC inactive / coherent PREPARE is the allowed reset domain per the card's section 7.
		 *
		 * FW-131 widened this window 1300 -> 1600: rotor_angle_reset() joined the same reset
		 * domain (the angle state belongs to the regulators it feeds). What the guard asserts is
		 * unchanged - both integrator resets are still there, still in this block. */
		CHECK(dwell_failsafe != NULL &&
			span_contains(dwell_failsafe, dwell_failsafe + 1600, "PI_iq.integral_part=0;") &&
			span_contains(dwell_failsafe, dwell_failsafe + 1600, "PI_id.integral_part=0;") &&
			span_contains(dwell_failsafe, dwell_failsafe + 1600, "bridge_lifecycle = BRIDGE_LIFECYCLE_IDLE;"),
			"T7b: dwell-timeout failsafe still zeroes both PI integrators when the bridge goes IDLE");
	}

	/* T8: hard fault preserved (independent, immediate, in FOC.c - unaffected by this card). */
	{
		const char *hard_fault = strstr(foc_c, "if(MS_FOC->i_d>(PH_CURRENT_MAX<<2))");
		CHECK(hard_fault != NULL &&
			span_contains(hard_fault, hard_fault + 900, "timer_primary_output_config(TIMER0,DISABLE)") &&
			span_contains(hard_fault, hard_fault + 900, "bridge_lifecycle=BRIDGE_LIFECYCLE_FAULT") &&
			span_contains(hard_fault, hard_fault + 900, "while(1){}"),
			"T8: independent hard-fault MOE-off path is unchanged");
	}

	/* T9: service/calibration reset preserved, still gated on the bridge going off in the same
	 * breath (hall_calibration_iq_request's own comment: "nothing is left regulating"). The span
	 * is 1200 rather than the original 900 because QZERO added its own reset to the same block
	 * (the DISABLE anchor now sits at 934); sized with headroom, not tight to it. */
	/* M820 comm-inhibit fix: the reset moved, unchanged, into hall_calibration_bridge_off() -
	 * the one exit shared by the verified completion and the comms-loss abort. */
	{
		const char *hall_cal = strstr(main_c, "uint16_t hall_calibration_iq_request(void){");
		const char *exit_fn = strstr(main_c, "static void hall_calibration_bridge_off(void)");
		while (exit_fn && exit_fn[45] == ';') /* skip the prototype, keep the definition */
			exit_fn = strstr(exit_fn + 1, "static void hall_calibration_bridge_off(void)");
		CHECK(hall_cal != NULL &&
			span_contains(hall_cal, hall_cal + 1200, "hall_calibration_bridge_off();") &&
			exit_fn != NULL &&
			span_contains(exit_fn, exit_fn + 1200, "PI_iq.integral_part=0;") &&
			span_contains(exit_fn, exit_fn + 1200, "PI_id.integral_part=0;") &&
			span_contains(exit_fn, exit_fn + 1200, "timer_primary_output_config(TIMER0,DISABLE)"),
			"T9: hall calibration's explicit reset is preserved, still coherent with bridge-off");
	}

	/* Out-of-scope guard: walk_assist_iq_request's own FW-093-restored zero-target reset is a
	 * distinct, already bike-tested WA-mode mechanism (0.0297-vs-0.0299 real-bike evidence: its
	 * removal stopped the motor turning at all) - NOT the ordinary ride-path defect this card
	 * fixes. It must be untouched by this card.
	 *
	 * FW-130 widened both windows 3700 -> 4300: the function grew by the walk_iq_max input (the
	 * bank's Walk current percentage now reaches the motor) and its rationale comment. What the
	 * guard asserts is unchanged - the reset is still there, still in this function. */
	{
		const char *walk_assist = strstr(main_c, "uint16_t walk_assist_iq_request(void){");
		CHECK(walk_assist != NULL &&
			span_contains(walk_assist, walk_assist + 4300, "if(!limited && PI_iq.integral_part){"),
			"out-of-scope guard: Walk Assist's own zero-target reset code is untouched (not in this card's scope)");
		if (walk_assist) {
			/* "restored with the rest of FW-093's revert" lives in a comment - check the RAW text
			 * at the corresponding offset, same technique as the T1 documentation check above. */
			const char *raw_walk_assist = main_raw + (walk_assist - main_c);
			CHECK(span_contains(raw_walk_assist, raw_walk_assist + 4300, "restored with the rest of FW-093's revert"),
				"out-of-scope guard: Walk Assist's FW-093 rationale comment is untouched (not in this card's scope)");
		}
	}

	free(main_c);
	free(foc_c);
done:
	free(main_raw);
	free(foc_raw);
}

int main(void)
{
	puts("STOP-CLICK-C1: PI D/Q integrator continuity through ordinary zero torque");
	model_behavior_checks();
	production_wiring_checks();
	if (host_test_failures == 0) {
		puts("STOP-CLICK-C1 PI integrator continuity: ALL CHECKS PASSED");
		return 0;
	}
	printf("STOP-CLICK-C1 PI integrator continuity: %d CHECK(S) FAILED\n", host_test_failures);
	return 1;
}
