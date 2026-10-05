/*
 * DISC-010: on/off button power-off as in the original application (src/power_button.c) and its
 * production wiring in main.c (source guards, main.c cannot be linked).
 *
 * The expected numbers follow the original application's machine (inc/power_button.h): a sample
 * every 32 ticks, 3 samples to confirm, the hold count from 2 to 200, then > 2000 ticks.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "power_button.h"
#include "check.h"

#define STR2(x) #x
#define STRINGIZE(x) STR2(x)

#define PRESSED   2400U   /* nominal on/off reading (inside 2048..2730)  */
#define RELEASED  4095U
#define DOWN_BTN  3300U   /* "down" button: Walk window, never power     */
#define NOISE     2900U   /* a stray sample outside the window           */

/* tick at which the long press fires for a clean press starting at tick 0, and the power-off */
#define LONG_TICK  ((3U + (POWER_BUTTON_LONG_COUNT - POWER_BUTTON_HOLD_START)) * POWER_BUTTON_SAMPLE_TICKS)
#define OFF_TICK   (LONG_TICK + POWER_BUTTON_OFF_DELAY_TICKS + 1U)

typedef uint16_t (*pa4_fn)(uint32_t sample_index);

/* Runs the module for `ticks` 4 kHz ticks from `start`, calling it every `step` ticks; returns the
 * tick (relative to start) at which the power-off became due, or 0xFFFFFFFF. */
static uint32_t run(uint32_t start, uint32_t ticks, uint32_t step, pa4_fn pa4)
{
	uint32_t t;
	power_button_init(start);
	for (t = step; t <= ticks; t += step) {
		power_button_update(start + t, pa4(t / POWER_BUTTON_SAMPLE_TICKS));
		if (power_button_power_off_due()) return t;
	}
	return 0xFFFFFFFFU;
}

static uint16_t clean_press(uint32_t i)         { (void)i; return PRESSED; }
static uint16_t released(uint32_t i)            { (void)i; return RELEASED; }
static uint16_t down_button(uint32_t i)         { (void)i; return DOWN_BTN; }
static uint16_t edge_low(uint32_t i)            { (void)i; return POWER_BUTTON_PA4_MIN; }
static uint16_t edge_high(uint32_t i)           { (void)i; return POWER_BUTTON_PA4_MAX; }
static uint16_t below_low(uint32_t i)           { (void)i; return POWER_BUTTON_PA4_MIN - 1U; }
static uint16_t above_high(uint32_t i)          { (void)i; return POWER_BUTTON_PA4_MAX + 1U; }
/* one stray sample in every 20 (once per 160 ms) */
static uint16_t noisy_1_in_20(uint32_t i)       { return (i % 20U == 10U) ? NOISE : PRESSED; }
/* two stray samples in a row every 10 */
static uint16_t noisy_2_in_10(uint32_t i)       { return (i % 10U == 5U || i % 10U == 6U) ? NOISE : PRESSED; }
/* three stray samples in a row every 50 (24 ms dropout every 400 ms) */
static uint16_t noisy_3_in_50(uint32_t i)       { return (i % 50U >= 25U && i % 50U <= 27U) ? NOISE : PRESSED; }
/* a 1.0 s press, then released */
static uint16_t short_press(uint32_t i)         { return (i < 125U) ? PRESSED : RELEASED; }
/* held just long enough to latch (~1.65 s), then released */
static uint16_t release_after_latch(uint32_t i) { return (i < 206U) ? PRESSED : RELEASED; }

/* The inherited M560 rule this card replaces: PA4 < 2800 on each 40 ms pass, > 62 in a row. */
static uint32_t run_inherited_rule(uint32_t ticks, pa4_fn pa4)
{
	uint32_t t;
	unsigned counter = 0U;
	for (t = 160U; t <= ticks; t += 160U) {
		if (pa4(t / POWER_BUTTON_SAMPLE_TICKS) < 2800U) counter++;
		else counter = 0U;
		if (counter > 62U) return t;
	}
	return 0xFFFFFFFFU;
}

static char *read_whole_file(const char *path)
{
	FILE *f = fopen(path, "rb");
	long n;
	char *buf;
	if (!f) return NULL;
	fseek(f, 0, SEEK_END);
	n = ftell(f);
	fseek(f, 0, SEEK_SET);
	buf = (char *)malloc((size_t)n + 1U);
	if (buf && fread(buf, 1, (size_t)n, f) != (size_t)n) { free(buf); buf = NULL; }
	if (buf) buf[n] = '\0';
	fclose(f);
	return buf;
}

static int span_contains(const char *from, const char *to, const char *needle)
{
	const char *hit = strstr(from, needle);
	return hit != NULL && hit < to;
}

int main(void)
{
	uint32_t t;

	/* P1: a clean press, button held the whole time: long press at 1.608 s, power-off at 2.108 s */
	t = run(0U, 20000U, 1U, clean_press);
	CHECK(t == OFF_TICK, "P1a. held button: power-off due exactly LONG + 500 ms + 1 tick (2.108 s)");
	power_button_init(0U);
	for (t = 1U; t < LONG_TICK; t++) power_button_update(t, PRESSED);
	CHECK(!power_button_off_latched() && power_button_hold_count() == POWER_BUTTON_LONG_COUNT - 1U,
	      "P1b. one tick before the 201st sample: hold count 199, nothing latched");
	power_button_update(LONG_TICK, PRESSED);
	CHECK(power_button_off_latched() && !power_button_power_off_due(),
	      "P1c. the 201st sample latches the power-off but does not perform it yet");

	/* P2: values outside the window never switch off */
	CHECK(run(0U, 40000U, 1U, released) == 0xFFFFFFFFU, "P2a. released (4095) never switches off");
	CHECK(run(0U, 40000U, 1U, down_button) == 0xFFFFFFFFU, "P2b. the down/Walk button (3300) never switches off");
	CHECK(run(0U, 40000U, 1U, below_low) == 0xFFFFFFFFU, "P2c. 2047 is outside the window");
	CHECK(run(0U, 40000U, 1U, above_high) == 0xFFFFFFFFU, "P2d. 2731 is outside the window");
	CHECK(run(0U, 20000U, 1U, edge_low) == OFF_TICK, "P2e. 2048 is inside the window");
	CHECK(run(0U, 20000U, 1U, edge_high) == OFF_TICK, "P2f. 2730 is inside the window");

	/* P3: the tolerance of the original application */
	/* Each stray sample costs exactly one sample: it still advances the count, the pressed one
	 * after it only returns to the held state. 11 strays (i = 10, 30 .. 210) before the latch. */
	t = run(0U, 40000U, 1U, noisy_1_in_20);
	CHECK(t == OFF_TICK + 11U * POWER_BUTTON_SAMPLE_TICKS,
	      "P3a. one stray sample in 20 does not lose the hold (88 ms later than clean)");
	t = run(0U, 40000U, 1U, noisy_2_in_10);
	CHECK(t != 0xFFFFFFFFU, "P3b. two stray samples in a row are still tolerated");
	CHECK(run(0U, 40000U, 1U, noisy_3_in_50) == 0xFFFFFFFFU,
	      "P3c. three in a row end the press (24 ms dropout every 400 ms never reaches 1.6 s)");

	/* R1: regression rationale - the inherited rule loses the same noisy press, the new one not */
	CHECK(run_inherited_rule(40000U, clean_press) != 0xFFFFFFFFU,
	      "R1a. the inherited rule works on a perfectly clean signal (why it looked right on paper)");
	CHECK(run_inherited_rule(40000U, noisy_1_in_20) == 0xFFFFFFFFU &&
	      run(0U, 40000U, 1U, noisy_1_in_20) != 0xFFFFFFFFU,
	      "R1b. one stray sample per 160 ms: inherited rule never switches off, the new one does");

	/* P4: a short press does nothing; once latched, releasing does not cancel */
	CHECK(run(0U, 40000U, 1U, short_press) == 0xFFFFFFFFU, "P4a. a 1 s press never switches off");
	CHECK(power_button_state() == 0U && power_button_hold_count() == 0U,
	      "P4b. and the machine is back to idle after the release");
	t = run(0U, 40000U, 1U, release_after_latch);
	CHECK(t == OFF_TICK, "P4c. released right after the latch: the power-off still follows on time");

	/* P5: a late main loop only delays samples, never bursts them */
	t = run(0U, 60000U, 200U, clean_press);   /* 50 ms between calls */
	CHECK(t != 0xFFFFFFFFU && t >= OFF_TICK, "P5a. called every 50 ms: still switches off, not earlier");
	power_button_init(0U);
	power_button_update(10000U, PRESSED);     /* one call after a 2.5 s stall */
	CHECK(power_button_state() == 1U, "P5b. a long stall yields ONE sample, not a burst of 78");
	power_button_update(10000U, PRESSED);
	CHECK(power_button_state() == 1U, "P5c. and no second sample in the same tick");

	/* P6: the free-running tick wraps */
	t = run(0xFFFFF000U, 20000U, 1U, clean_press);
	CHECK(t == OFF_TICK, "P6. identical timing across the 32-bit wrap of control_time_ticks");

	/* P7: init clears a latch */
	(void)run(0U, 20000U, 1U, clean_press);
	power_button_init(0U);
	CHECK(!power_button_off_latched() && !power_button_power_off_due(), "P7. init clears the latch");

	/* W: production wiring */
	{
		char *mainc = read_whole_file(STRINGIZE(MAIN_C_PATH));
		CHECK(mainc != NULL, "W0. main.c readable");
		if (mainc) {
			const char *init = strstr(mainc, "power_button_init(control_time_ticks);");
			const char *loop = strstr(mainc, "    while (1){");
			const char *upd = strstr(mainc, "power_button_update(control_time_ticks, adc_value[5]);");
			const char *slow = strstr(mainc, "if (slow_loop_counter > SLOW_LOOP_TICKS){");
			const char *off = strstr(mainc, "if(power_button_power_off_due()){");
			const char *slow_end = strstr(mainc, "}//end slow loop");
			CHECK(init && loop && init < loop, "W1. initialised once, before while(1)");
			CHECK(upd && loop && slow && upd > loop && upd < slow,
			      "W2. updated every while(1) iteration with the 4 kHz clock and PA4 (adc_value[5]), outside the slow loop");
			CHECK(off && slow && slow_end && off > slow && off < slow_end,
			      "W3. the power-off itself stays in the slow loop");
			CHECK(off && span_contains(off, off + 200, "power_off_controller();") &&
			      !span_contains(off, off + 200, "update_hold_ticks"),
			      "W4. it calls power_off_controller() and is not gated by the update hold");
			CHECK(strstr(mainc, "shutoffcounter") == NULL, "W5. the inherited 40 ms counter is gone");
			free(mainc);
		}
	}

	if (host_test_failures) {
		printf("power button: %d CHECK(S) FAILED\n", host_test_failures);
		return 1;
	}
	printf("power button: ALL CHECKS PASSED\n");
	return 0;
}
