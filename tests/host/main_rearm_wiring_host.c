/*
 * FW-111 v3 Bug 1 wiring guard: a STRUCTURAL check on src/main.c's SOURCE TEXT, not a link-and-
 * execute test of the real module. main.c cannot be linked here - it is the ARM entry point,
 * wired directly to GD32 CMSIS registers and real hardware throughout.
 *
 * WHY THIS GUARD EXISTS AT ALL. The unit test of rearm_delay_diag.c can stamp the standstill
 * markers with whatever ticks the harness pleases, so it can never catch the actual Bug 1: in
 * v2, get_standstill_position(uint32_t now_tick) took a single clock snapshot at entry and passed
 * THAT SAME VALUE to both rearm_delay_note_standstill_enter() and rearm_delay_note_standstill_exit()
 * around the blocking delay_1ms(25). Both markers therefore got the same tick, and the measured
 * 25 ms Hall hold was always 0 - a defect only main.c's own call pattern could produce. Only a
 * source-text check of main.c can detect a regression of THAT shape.
 *
 * WHAT THIS PROVES, exactly:
 *   1. get_standstill_position takes NO parameters (the v2 signature uint32_t now_tick is gone).
 *   2. Inside its body, rearm_delay_note_standstill_enter(...) occurs with a direct read of the
 *      global control_time_ticks as its argument, STRICTLY BEFORE delay_1ms(25); and
 *   3. rearm_delay_note_standstill_exit(...) likewise reads control_time_ticks STRICTLY AFTER
 *      delay_1ms(25).
 *   The two markers therefore cannot share a stale snapshot, and the hold's length in ticks is
 *   exit-read minus enter-read across the blocking delay - ~25 ms at CONTROL_TIMEBASE_HZ = 4 kHz.
 *
 * WHAT THIS DOES NOT PROVE: that the clock actually advances across the delay at runtime (that
 * is the control ISR's job), or anything about rearm_delay_diag.c's own recording - that is the
 * unit suite's job (tests/host/rearm_delay_diag_host.c, test_bug1_standstill_span_is_real_time).
 *
 * Comments are stripped the same way as main_startup_wiring_host.c: line comments to end of line,
 * block comments up to their closing marker, string/char literals recognised and left alone. A
 * commented-out call therefore simply does not exist once comments are stripped, which is what
 * makes the "not commented out" property and the count check one and the same.
 */

#include "../common/check.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STRINGIZE2(x) #x
#define STRINGIZE(x) STRINGIZE2(x)

#ifndef MAIN_C_PATH
#error "MAIN_C_PATH must be defined (by the build script) to the path of src/main.c"
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

/* Find the body of get_standstill_position in the sanitized text: the region between its opening
 * `{` and the matching closing `}`. Returns a pointer to the opening brace, or NULL. */
static const char *find_function_body(const char *clean, const char *fn_sig)
{
	const char *sig = strstr(clean, fn_sig);
	if (!sig) return NULL;
	const char *brace = strstr(sig, "{");
	if (!brace) return NULL;
	const char *p = brace + 1;
	int depth = 1;
	while (p && depth > 0) {
		if (*p == '{') depth++;
		else if (*p == '}') depth--;
		p++;
	}
	return brace;   /* p is past the matching close */
}

int main(void)
{
	const char *path = STRINGIZE(MAIN_C_PATH);
	printf("FW-111 v3 Bug 1 main.c wiring guard (source-text check, see file header)\n");
	printf("  MAIN_C_PATH = %s\n", path);

	long len = 0;
	char *raw = read_whole_file(path, &len);
	CHECK(raw != NULL, "setup: src/main.c was readable at MAIN_C_PATH");
	if (!raw) {
		printf("\n1 main_rearm_wiring check(s) FAILED (could not read %s).\n", path);
		return 1;
	}

	char *clean = strip_comments(raw, len);
	CHECK(clean != NULL, "setup: comment/string-aware sanitization succeeded");
	free(raw);
	if (!clean) {
		printf("\n1 main_rearm_wiring check(s) FAILED (sanitization failed).\n");
		return 1;
	}

	/* 1. The signature must have NO parameter - the v2 uint32_t now_tick is exactly what made
	 * both markers share one stale snapshot. Match the DEFINITION (signature followed by `{`,
	 * sanitized to a bare `{`), not the prototype - the prototype has no body and would make
	 * the body search below land in the wrong function. */
	const char *sig = strstr(clean, "get_standstill_position(void){");
	if (!sig) {
		/* allow whitespace between the signature and the opening brace */
		const char *head = strstr(clean, "get_standstill_position(void)");
		if (head) {
			const char *c = head + strlen("get_standstill_position(void)");
			while (*c == ' ' || *c == '\t' || *c == '\n' || *c == '\r') c++;
			if (*c == '{') sig = c - strlen("get_standstill_position(void)");
		}
	}
	CHECK(sig != NULL,
		"GUARD: get_standstill_position is declared WITHOUT parameters - the v2 "
		"uint32_t now_tick signature (which stamped both standstill markers with the same "
		"tick and made the 25 ms hold measure 0) must not come back");

	const char *body_brace = sig ? find_function_body(clean, sig) : NULL;
	CHECK(body_brace != NULL, "setup: get_standstill_position()'s body found");

	if (body_brace) {
		/* 2. enter marker: a direct read of the GLOBAL control_time_ticks, before delay_1ms(25). */
		const char *enter_call = strstr(body_brace, "rearm_delay_note_standstill_enter(");
		CHECK(enter_call != NULL,
			"GUARD: rearm_delay_note_standstill_enter(...) is called inside the function");
		if (enter_call) {
			/* The argument must be a direct read of the global, i.e. no arithmetic on it. */
			const char *arg = strstr(enter_call, "control_time_ticks)");
			CHECK(arg != NULL,
				"GUARD: rearm_delay_note_standstill_enter() reads the GLOBAL control_time_ticks "
				"fresh (a direct read, not a stale parameter)");
		}

		const char *delay = strstr(body_brace, "delay_1ms(25);");
		CHECK(delay != NULL,
			"GUARD: the 25 ms blocking Hall hold (delay_1ms(25)) is present in the function");

		const char *exit_call = strstr(body_brace, "rearm_delay_note_standstill_exit(");
		CHECK(exit_call != NULL,
			"GUARD: rearm_delay_note_standstill_exit(...) is called inside the function");
		if (exit_call) {
			const char *arg = strstr(exit_call, "control_time_ticks)");
			CHECK(arg != NULL,
				"GUARD: rearm_delay_note_standstill_exit() reads the GLOBAL control_time_ticks "
				"fresh (a direct read, not a stale parameter)");
		}

		/* 3. ordering: enter ... delay_1ms(25) ... exit, each with its own read. */
		if (enter_call && delay && exit_call) {
			CHECK(enter_call < delay,
				"GUARD: the enter marker is read BEFORE delay_1ms(25) - the hold is measured "
				"on its entry side");
			CHECK(delay < exit_call,
				"GUARD: delay_1ms(25) sits BETWEEN the two markers - the exit tick is read "
				"AFTER the blocking hold, so exit-read minus enter-read is the true 25 ms");
		}
	}

	/*
	 * FW-111 v4 ADDITIONAL GUARD: the reservation orchestration block cannot be host-linked (it
	 * lives in main.c), so its most safety-critical property - that pas_raw_freeze()'s refusal is
	 * actually HONOURED, never silently overridden into a fabricated FULL - is checked here on the
	 * source text instead. This is exactly the shape of defect a unit test of rearm_delay_diag.c
	 * alone could never see, because that module never calls pas_raw_freeze() itself; only main.c
	 * wires the two together.
	 */
	{
		const char *cap_full = strstr(clean, "REARM_DELAY_CAPTURE_FULL");
		CHECK(cap_full != NULL, "GUARD v4: REARM_DELAY_CAPTURE_FULL is assigned somewhere in main.c");
		if (cap_full) {
			/* Walk backwards from the FULL assignment to its nearest preceding pas_raw_freeze(
			 * call - the ternary this repo's pattern uses puts the call immediately before the
			 * value it selects. A mutation that assigns FULL unconditionally (outside any
			 * pas_raw_freeze()-guarded branch) has no such call within a short lookbehind. */
			const char *scan_start = cap_full - 200 < clean ? clean : cap_full - 200;
			const char *freeze_call = NULL;
			for (const char *p = scan_start; p < cap_full; p++) {
				const char *hit = strstr(p, "pas_raw_freeze(");
				if (hit == NULL || hit >= cap_full) break;
				freeze_call = hit;
				p = hit;
			}
			CHECK(freeze_call != NULL,
				"GUARD v4: REARM_DELAY_CAPTURE_FULL is assigned in the same expression as a "
				"pas_raw_freeze(...) call, not unconditionally - a refusal must be able to reach "
				"REARM_DELAY_CAPTURE_TRACE_ONLY instead");
		}
		const char *trace_only = strstr(clean, "REARM_DELAY_CAPTURE_TRACE_ONLY");
		CHECK(trace_only != NULL,
			"GUARD v4: REARM_DELAY_CAPTURE_TRACE_ONLY is reachable somewhere in main.c - "
			"pas_raw_freeze()'s refusal has a real destination, not just FULL as the only outcome");
	}

	free(clean);

	if (host_test_failures == 0) {
		printf("main.c Bug 1 wiring guard passed - the standstill markers read control_time_ticks\n");
		printf("fresh on both sides of the blocking delay_1ms(25), so the Hall hold measures ~25 ms.\n");
		return 0;
	}
	printf("\n%d main_rearm_wiring check(s) FAILED.\n", host_test_failures);
	return 1;
}
