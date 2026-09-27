/*
 * EVD-WALK bit0 guard: source-text checks on src/CAN_Display.c proving that the 0x3202
 * payload bit0 (the stock DPC245 CF80301.2 Walk-icon blink gate) is only ever set while a
 * Walk Assist session is actually RUNNING and the bike is ACTUALLY MOVING - never merely
 * because Walk mode was selected, and never during ordinary pedal starts.
 *
 * WHY SOURCE-TEXT: src/CAN_Display.c cannot be linked here - it is wired directly to GD32
 * CMSIS registers and real CAN peripherals throughout (same reasoning as the FW-110
 * can-blocking guard, whose sibling this is). The behavioural gate this card adds is a
 * simple boolean, so the practical regression check is structural: the gate tokens must
 * coexist in sendCAN_3202()'s body in the right order.
 *
 * WHAT THIS PROVES (T1-T4):
 *   T1: The default payload is still zero (uint8_t d[8] = {0};) - the frame never leaves
 *       with bit0 set without passing through the explicit gate below.
 *   T2: A line `d[0] = 0x01;` exists INSIDE sendCAN_3202() - the sender actually produces
 *       the blinking flag; reverting to the old unconditional 0x00 fails this.
 *   T3: That set is gated on `MS.pushassist_flag != RESET` - Walk must be RUNNING (permission
 *       granted), so an ordinary pedal start can never set the bit.
 *   T4: The same gate references BOTH `MS.Speedx100 > 0` (wheel) AND `ui16_erps >=
 *       RIDE_COAST_RELEASE_ERPS` (motor) - the same wheel-OR-motor motion test FW-134 uses
 *       for 0x3210 (main.c): at walking pace a single wheel pulse can be ~2.6 s apart, right
 *       at the speed-stop timeout, so wheel speed alone flickers exactly during Walk - the
 *       motor is unambiguous there. A regression that set the bit on Walk-selected-but-
 *       stationary, or on motion without Walk, fails these.
 *
 * WHAT THIS DOES NOT PROVE: runtime behaviour on the bus, the exact HMI rendering, or that
 * the condition is evaluated with current data - only that the gated semantics exist in the
 * active source text.
 */

#include "../common/check.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STRINGIZE2(x) #x
#define STRINGIZE(x) STRINGIZE2(x)

#ifndef CAN_DISPLAY_C_PATH
#error "CAN_DISPLAY_C_PATH must be defined (by the build script) to the path of src/CAN_Display.c"
#endif

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

/* Same sanitizer as fw110_can_blocking_guard_host.c: same-length output, comments blanked,
 * string/char literals copied verbatim so a comment-looking sequence is never counted. */
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
			char quote = text[i];
			out[i] = text[i]; i++;
			while (i < len && text[i] != quote) {
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

int main(void)
{
	const char *path = STRINGIZE(CAN_DISPLAY_C_PATH);
	printf("EVD-WALK 0x3202 bit0 guard (source-text check)\n");
	printf("  CAN_DISPLAY_C_PATH = %s\n", path);

	long len = 0;
	char *raw = read_whole_file(path, &len);
	CHECK(raw != NULL, "setup: src/CAN_Display.c was readable at CAN_DISPLAY_C_PATH");
	if (!raw) {
		printf("\n1 EVD-WALK 0x3202 bit0 check(s) FAILED (could not read file).\n");
		return 1;
	}

	char *clean = strip_comments(raw, len);
	CHECK(clean != NULL, "setup: CAN_Display.c comment/string-aware sanitization succeeded");
	free(raw);
	if (!clean) {
		printf("\n1 EVD-WALK 0x3202 bit0 check(s) FAILED (sanitization failed).\n");
		return 1;
	}

	/* --- location of the sender: keep the signature text the FW-110 guard also searches --- */
	const char *s202 = strstr(clean, "bool sendCAN_3202(void){");
	CHECK(s202 != NULL, "setup: bool sendCAN_3202(void){ found in CAN_Display.c");
	const char *s202_end = s202 ? strstr(s202 + 1, "\n}") : NULL;
	CHECK(s202_end != NULL, "setup: end of sendCAN_3202() body found");
	if (!s202 || !s202_end) {
		free(clean);
		printf("\n1 EVD-WALK 0x3202 bit0 check(s) FAILED (could not bound sender body).\n");
		return 1;
	}

	/* --- T1: default payload is still zero ------------------------------------------------ */
	CHECK(strstr(s202, "d[8] = {0}") != NULL && (strstr(s202, "d[8] = {0}") + 10) <= s202_end,
		"T1: sendCAN_3202() initializes uint8_t d[8] = {0}; - the default (non-Walk-riding) payload stays 0x00");

	/* --- T2: the bit0=0x01 set actually exists -------------------------------------------- */
	const char *set01 = strstr(s202, "d[0] = 0x01");
	CHECK(set01 != NULL && set01 < s202_end,
		"T2: sendCAN_3202() contains `d[0] = 0x01;` - the Walk-icon blink flag is actually produced");
	const char *set00 = strstr(s202, "d[0] = 0x00");
	CHECK(set00 == NULL || set00 > s202_end,
		"T2b: the only active d[0] write after the zero-initializer is the gated 0x01 (no unconditional 0x00 override)");

	/* --- T3: Walk RUNNING (permission) gate ------------------------------------------------ */
	const char *wa_gate = set01 ? strstr(s202, "MS.pushassist_flag != RESET") : NULL;
	CHECK(wa_gate != NULL && wa_gate < s202_end && set01 && wa_gate < set01,
		"T3: the 0x01 set is gated on `MS.pushassist_flag != RESET` BEFORE it - Walk must be RUNNING (permission granted), so an ordinary pedal start never sets bit0");

	/* --- T4: ACTUAL MOTION wheel-OR-motor gate ---------------------------------------------- */
	const char *wheel = set01 ? strstr(s202, "MS.Speedx100 > 0") : NULL;
	const char *motor = set01 ? strstr(s202, "ui16_erps >= RIDE_COAST_RELEASE_ERPS") : NULL;
	CHECK(wheel != NULL && wheel < s202_end && wheel < set01,
		"T4a: the 0x01 set tests wheel motion `MS.Speedx100 > 0` before it (bit0 only while actually moving)");
	CHECK(motor != NULL && motor < s202_end && motor < set01,
		"T4b: the 0x01 set also tests motor motion `ui16_erps >= RIDE_COAST_RELEASE_ERPS` - the unambiguous motion signal at walking pace (same test FW-134 uses for 0x3210)");
	if (wheel && motor && wheel < motor) {
		const char *between = wheel + strlen("MS.Speedx100 > 0");
		const char *sep_or = NULL;
		for (; between < motor; between++) {
			if (*between == '|' && between + 1 < motor && between[1] == '|') { sep_or = between; break; }
		}
		const char *sep_and = NULL;
		for (between = wheel + strlen("MS.Speedx100 > 0"); between < motor; between++) {
			if (*between == '&' && between + 1 < motor && between[1] == '&') { sep_and = between; break; }
		}
		CHECK(sep_or != NULL, "T4c: wheel and motor motion tests are combined with `||` (wheel-OR-motor)");
		CHECK(sep_or != NULL && (sep_and == NULL || sep_and > motor),
			"T4d: no `&&` sits between the wheel and motor motion tests - motion is OR, not AND");
	}

	/* --- context: the moving test constants are visible where this file compiles ------------- */
	CHECK(strstr(clean, "#include \"main.h\"") != NULL,
		"T5: CAN_Display.c includes main.h (which pulls config.h, defining RIDE_COAST_RELEASE_ERPS)");

	free(clean);

	if (host_test_failures == 0) {
		printf("EVD-WALK 0x3202 bit0 guard passed - all structural checks verified.\n");
		return 0;
	}
	printf("\n%d EVD-WALK 0x3202 bit0 check(s) FAILED.\n", host_test_failures);
	return 1;
}
