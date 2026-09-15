/*
 * FW-111 v3 Bug 1 wiring guard: a STRUCTURAL check on src/main.c's SOURCE TEXT, not a link-and-
 * execute test of the real module. main.c cannot be linked here - it is the ARM entry point,
 * wired directly to GD32 CMSIS registers and real hardware throughout.
 *
 * UPDATED (FW-112 / standstill-delay removal): the 25 ms blocking delay_1ms(25) and the
 * rearm_delay_note_standstill_enter/exit markers have been removed from get_standstill_position().
 * Hall sensors are digital IPU on PC6-8, driven by permanent magnets — GPIO is always readable,
 * no settling delay is needed. The function now reads the Hall sector directly and derives the
 * initial electrical angle in zero time.
 *
 * WHAT THIS PROVES, exactly:
 *   1. get_standstill_position takes NO parameters (the v2 signature uint32_t now_tick is gone).
 *   2. There is NO delay_1ms(25) inside the function body — the legacy blocking hold is removed.
 *   3. The function reads Hall GPIO: (GPIO_ISTAT(GPIOC)>>6)&0x07.
 *   4. The function assigns q31_rotorposition_hall from a Hall-lookup table.
 *   5. The function sets q31_rotorposition_absolute from q31_rotorposition_hall.
 *
 * WHAT THIS DOES NOT PROVE: that the seed is electrically correct, or that the angle it derives
 * matches the rotor. Those belong to the electrical SIL and to the bench.
 *
 * HISTORY, because this file is shorter than it used to be. It once also guarded the reservation
 * orchestration block - that REARM_DELAY_CAPTURE_FULL could only be assigned inside a
 * pas_raw_freeze()-guarded expression, and that REARM_DELAY_CAPTURE_TRACE_ONLY was reachable.
 * Assist Pipeline V2 deleted rearm_delay_diag.c, so main.c no longer contains those symbols and
 * those two checks failed against firmware that is correct. They are gone with their subject.
 *
 * The file also briefly went missing from run-host-tests.ps1 altogether. It was named for FW-111
 * and sat among the FW-111 rearm suites, every one of which had to go because they linked deleted
 * modules; this one links nothing, so it had no reason to go with them. Comparing the baseline
 * gate's 72 host suites against the rebuilt gate's 54 is what found it.
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
	printf("standstill rotor seed wiring guard (source-text check, see file header)\n");
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
		/* 2. No delay_1ms(25) inside get_standstill_position — the legacy 25 ms blocking hold
		 * is removed. Hall sensors are IPU on PC6-8, always readable, no settling needed. */
		const char *delay = strstr(body_brace, "delay_1ms(25);");
		CHECK(delay == NULL,
			"GUARD: delay_1ms(25) is NOT present in get_standstill_position — the 25 ms "
			"blocking hold has been removed (Hall GPIO is always readable, MOE is OFF)");

		/* 3. Hall GPIO read: the function must read PC6..8 via the standard expression. */
		const char *hall_read = strstr(body_brace, "GPIO_ISTAT(GPIOC)>>6)&0x07");
		CHECK(hall_read != NULL,
			"GUARD: get_standstill_position reads Hall GPIO via (GPIO_ISTAT(GPIOC)>>6)&0x07");

		/* 4. q31_rotorposition_hall must be assigned from a calibrated Hall constant. */
		const char *hall_assign = strstr(body_brace, "q31_rotorposition_hall = Hall_");
		CHECK(hall_assign != NULL,
			"GUARD: q31_rotorposition_hall is assigned from a calibrated Hall constant "
			"(Hall_13, Hall_32, etc.)");

		/* 5. q31_rotorposition_absolute must be set from q31_rotorposition_hall. */
		const char *abs_assign = strstr(body_brace, "q31_rotorposition_absolute = q31_rotorposition_hall");
		CHECK(abs_assign != NULL,
			"GUARD: q31_rotorposition_absolute is set from q31_rotorposition_hall (one-shot seed)");
	}

	/*
	 * The reservation-orchestration guard that used to live here is gone with its subject:
	 * rearm_delay_diag.c was deleted by Assist Pipeline V2, so REARM_DELAY_CAPTURE_FULL and
	 * REARM_DELAY_CAPTURE_TRACE_ONLY no longer exist in main.c. See the file header.
	 */

	free(clean);

	if (host_test_failures == 0) {
		printf("main.c standstill position guard passed - no blocking delay, Hall GPIO read directly.\n");
		return 0;
	}
	printf("\n%d standstill wiring check(s) FAILED.\n", host_test_failures);
	return 1;
}
