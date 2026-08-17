/*
 * FW-109 v2 test-wiring guard: a STRUCTURAL check on src/main.c's SOURCE TEXT, not a link-and-
 * execute test of the real module. main.c cannot be linked here - it is the ARM entry point,
 * wired directly to GD32 CMSIS registers, interrupt vectors and real hardware peripherals
 * throughout; stubbing all of that out just to link main() itself would be a far larger
 * undertaking than the one thing this guard exists to protect.
 *
 * WHAT THIS PROVES, exactly:
 *   1. `pas_direction_init();` occurs ACTIVELY (not inside a line comment or a block comment) in
 *      main()'s startup region EXACTLY ONCE - not zero, not two-or-more.
 *   2. It sits in this exact textual order: motor_core_init( ... before ... pas_direction_init();
 *      ... before ... the first ACTIVE call to reg_ADC_processing(); ... before ... while (1){.
 *      The middle step matters as much as the others: reg_ADC_processing() is what reaches
 *      pas_direction_on_step()/on_stop() (main.c's PAS decode block calls it from there), and its
 *      FIRST active call in this file is not inside the main loop - it is main()'s own 256-
 *      iteration filter-settling calibration loop, well before while(1). A pas_direction_init()
 *      placed after THAT call but still before while(1) would have passed a weaker "before
 *      while(1)" check while still leaving the calibration loop's very first reg_ADC_processing()
 *      calls running against not-yet-explicitly-initialised module state - this is the exact gap
 *      an earlier draft of this guard had, found and fixed on review.
 *
 * Comments are stripped (line comments to end of line, block comments up to their closing
 * marker) before any of the above is searched for, with string/char literals recognised and left
 * alone so a comment-looking sequence inside one is never mistaken for a real comment. This is
 * what makes both the exact-count check and the "not commented out" property one and the same
 * check, rather than two separate (and, as found on review, inconsistent) mechanisms: a
 * commented-out occurrence simply does not exist any more once comments are stripped, so it
 * cannot be counted, full stop - whichever comment style disabled it.
 *
 * WHAT THIS DOES NOT PROVE: that main() actually runs, that this call has the intended EFFECT at
 * runtime, or anything about pas_direction.c's own behaviour - that is Test B's job
 * (tests/host/pas_direction_host.c), which links and exercises the real module directly.
 *
 * WHY THIS GUARD EXISTS AT ALL: pas_direction.c's static state (dir_state, fwd_run, the legacy
 * backpedal derivation) currently starts safe only because zero happens to mean FORWARD_SAFE/NONE/
 * empty - the SAME thing .bss zeroing already gives for free. Before FW-109 v2's closeout,
 * pas_direction_init() had ZERO production callers and was silently removed by the linker's
 * --gc-sections (confirmed by a full ELF/MAP audit - no host test could see this, because none of
 * them link main.c). A future edit could reintroduce an equivalent gap invisibly - deleting the
 * call, duplicating it, commenting it out with either comment style, or moving it past the point
 * something else already depends on it running first - without breaking a single other test in
 * this suite. This guard is the minimum wiring needed so THAT class of edit fails loudly here.
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

/*
 * Returns a SANITIZED copy of `text`, the SAME LENGTH, with every line comment and every block
 * comment replaced by blanks (newlines kept as newlines, everything else becomes a space) - and
 * with string/char literals recognised and copied verbatim, backslash
 * escapes included, so a comment-looking sequence inside one is never mistaken for a real
 * comment. Same length, nothing compacted or removed: every pointer/offset found in the
 * sanitized buffer corresponds 1:1 to the same position in the original, so ordering checks via
 * plain pointer comparison stay valid without any separate remapping.
 */
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
			if (i < len) { out[i] = ' '; i++; }   /* the star of the closing marker */
			if (i < len) { out[i] = ' '; i++; }   /* the slash of the closing marker */
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
			if (i < len) { out[i] = text[i]; i++; }   /* the closing quote */
			continue;
		}
		out[i] = text[i];
		i++;
	}
	out[len] = '\0';
	return out;
}

/* Count of ACTIVE (post-strip) occurrences of `needle` at or after `from`, strictly before
 * `before` (pass NULL for no upper bound). Returns the count and, via *out_first, the first one
 * found - both computed in the same left-to-right scan so they can never disagree with each
 * other about what "first" means. */
static int count_active(const char *from, const char *before, const char *needle,
                         const char **out_first)
{
	int n = 0;
	const char *p = from;
	if (out_first) *out_first = NULL;
	while (p) {
		const char *hit = strstr(p, needle);
		if (!hit) break;
		if (before && hit >= before) break;
		n++;
		if (out_first && !*out_first) *out_first = hit;
		p = hit + 1;
	}
	return n;
}

int main(void)
{
	const char *path = STRINGIZE(MAIN_C_PATH);
	printf("FW-109 v2 main.c startup wiring guard (source-text check, see file header)\n");
	printf("  MAIN_C_PATH = %s\n", path);

	long len = 0;
	char *raw = read_whole_file(path, &len);
	CHECK(raw != NULL, "setup: src/main.c was readable at MAIN_C_PATH");
	if (!raw) {
		printf("\n1 main_startup_wiring check(s) FAILED (could not read %s).\n", path);
		return 1;
	}

	char *clean = strip_comments(raw, len);
	CHECK(clean != NULL, "setup: comment/string-aware sanitization succeeded");
	free(raw);
	if (!clean) {
		printf("\n1 main_startup_wiring check(s) FAILED (sanitization failed).\n");
		return 1;
	}

	const char *main_fn = strstr(clean, "int main(void)");
	CHECK(main_fn != NULL, "setup: main()'s own definition found in main.c");

	const char *loop = main_fn ? strstr(main_fn, "while (1){") : NULL;
	CHECK(loop != NULL, "setup: main()'s while(1) control loop found after main()'s definition");

	const char *motor_init = main_fn ? strstr(main_fn, "motor_core_init(") : NULL;
	CHECK(motor_init != NULL, "setup: motor_core_init( found after main()'s definition");

	const char *first_call = NULL;
	int call_count = main_fn ? count_active(main_fn, loop, "pas_direction_init();", &first_call) : 0;
	CHECK(call_count == 1,
		"GUARD: pas_direction_init(); is active (not commented out, either style) EXACTLY "
		"ONCE before while(1) - found some other count. Zero means deleted or fully "
		"commented out; two or more means a duplicate call slipped in.");

	const char *first_adc = main_fn ? strstr(main_fn, "reg_ADC_processing();") : NULL;
	CHECK(first_adc != NULL,
		"setup: at least one active reg_ADC_processing(); call found after main()'s definition");

	if (motor_init && first_call && first_adc && loop) {
		CHECK(motor_init < first_call,
			"GUARD: pas_direction_init(); comes AFTER motor_core_init(, as required");
		CHECK(first_call < first_adc,
			"GUARD: pas_direction_init(); comes BEFORE the FIRST active reg_ADC_processing() "
			"call - that function reaches pas_direction_on_step()/on_stop() (main.c's PAS "
			"decode block), so nothing may call it before this runs, including the filter-"
			"settling calibration loop that calls it well before while(1)");
		CHECK(first_adc < loop,
			"GUARD: the first active reg_ADC_processing(); call comes BEFORE while(1), as "
			"the current startup structure expects - if this ever legitimately changes "
			"(e.g. the calibration loop is restructured), update this guard deliberately "
			"rather than let it silently stop checking the order that matters");
	}

	free(clean);

	if (host_test_failures == 0) {
		printf("main.c startup wiring guard passed - pas_direction_init() is called exactly once,\n");
		printf("in the right place, in the right order.\n");
		return 0;
	}
	printf("\n%d main_startup_wiring check(s) FAILED.\n", host_test_failures);
	return 1;
}
