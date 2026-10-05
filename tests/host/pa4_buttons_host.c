/*
 * DISC-010: the PA4 button line as in the original application (src/pa4_buttons.c) and its
 * production wiring in main.c (source guards, main.c cannot be linked).
 *
 * The expected numbers follow the original application's machines (inc/pa4_buttons.h): a sample
 * every 32 ticks, 3 samples to confirm, the hold count from 2 to 200, the on/off power-off
 * > 2000 ticks after the long press, the Walk bridge of 250 one-millisecond samples.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pa4_buttons.h"
#include "check.h"

#define STR2(x) #x
#define STRINGIZE(x) STR2(x)

#define PRESSED   2400U   /* nominal on/off reading            */
#define DOWN      3300U   /* nominal down reading              */
#define RELEASED  4095U
#define NOISE     2800U   /* between the windows: neither      */

#define NONE      0xFFFFFFFFU
#define ST        PA4_SAMPLE_TICKS

/* tick at which the long press fires for a clean press starting at tick 0, and the power-off */
#define LONG_TICK  ((3U + (PA4_LONG_COUNT - PA4_HOLD_START)) * ST)
#define OFF_TICK   (LONG_TICK + PA4_POWER_OFF_DELAY_TICKS + 1U)

typedef uint16_t (*pa4_fn)(uint32_t sample_index);

static bool g_walk;   /* the walk_active input fed to the module */

/* Runs the module for `ticks` from `start`, called every `step` ticks; returns the tick
 * (relative to start) at which the power-off became due, or NONE. */
static uint32_t run(uint32_t start, uint32_t ticks, uint32_t step, pa4_fn pa4)
{
	uint32_t t;
	pa4_buttons_init(start);
	for (t = step; t <= ticks; t += step) {
		pa4_buttons_update(start + t, pa4(t / ST), g_walk);
		if (pa4_buttons_power_off_due()) return t;
	}
	return NONE;
}

static uint16_t clean_press(uint32_t i)         { (void)i; return PRESSED; }
static uint16_t released(uint32_t i)            { (void)i; return RELEASED; }
static uint16_t down_button(uint32_t i)         { (void)i; return DOWN; }
static uint16_t edge_low(uint32_t i)            { (void)i; return PA4_POWER_MIN; }
static uint16_t edge_high(uint32_t i)           { (void)i; return PA4_POWER_MAX; }
static uint16_t below_low(uint32_t i)           { (void)i; return PA4_POWER_MIN - 1U; }
static uint16_t above_high(uint32_t i)          { (void)i; return PA4_POWER_MAX + 1U; }
static uint16_t noisy_1_in_20(uint32_t i)       { return (i % 20U == 10U) ? NOISE : PRESSED; }
static uint16_t noisy_2_in_10(uint32_t i)       { return (i % 10U == 5U || i % 10U == 6U) ? NOISE : PRESSED; }
static uint16_t noisy_3_in_50(uint32_t i)       { return (i % 50U >= 25U && i % 50U <= 27U) ? NOISE : PRESSED; }
static uint16_t short_press(uint32_t i)         { return (i < 125U) ? PRESSED : RELEASED; }
static uint16_t release_after_latch(uint32_t i) { return (i < 206U) ? PRESSED : RELEASED; }

/* The inherited M560 on/off rule: PA4 < 2800 on each 40 ms pass, > 62 in a row. */
static uint32_t run_inherited_power(uint32_t ticks, pa4_fn pa4)
{
	uint32_t t;
	unsigned counter = 0U;
	for (t = 160U; t <= ticks; t += 160U) {
		if (pa4(t / ST) < 2800U) counter++;
		else counter = 0U;
		if (counter > 62U) return t;
	}
	return NONE;
}

/* Drives the module tick by tick with a per-tick PA4 value; returns whether walk is held at the end. */
static uint32_t g_t;
static void feed(uint32_t ticks, uint16_t pa4)
{
	uint32_t k;
	for (k = 0; k < ticks; k++) { g_t++; pa4_buttons_update(g_t, pa4, g_walk); }
}
static void start_at(uint32_t t0) { g_t = t0; pa4_buttons_init(t0); }

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

static int count_of(const char *hay, const char *needle)
{
	int n = 0;
	const char *p = hay;
	while ((p = strstr(p, needle)) != NULL) { n++; p += strlen(needle); }
	return n;
}

int main(void)
{
	uint32_t t;
	int ok;

	/* ==== ON/OFF ==== */
	g_walk = false;

	/* P1: a clean press, button held the whole time: long press at 1.608 s, power-off at 2.108 s */
	CHECK(run(0U, 20000U, 1U, clean_press) == OFF_TICK, "P1a. held button: power-off due LONG + 500 ms + 1 tick (2.108 s)");
	pa4_buttons_init(0U);
	for (t = 1U; t < LONG_TICK; t++) pa4_buttons_update(t, PRESSED, false);
	CHECK(!pa4_buttons_power_off_latched() && pa4_buttons_power_hold() == PA4_LONG_COUNT - 1U,
	      "P1b. one tick before the 201st sample: hold count 199, nothing latched");
	pa4_buttons_update(LONG_TICK, PRESSED, false);
	CHECK(pa4_buttons_power_off_latched() && !pa4_buttons_power_off_due(),
	      "P1c. the 201st sample latches the power-off but does not perform it yet");

	/* P2: values outside the window never switch off */
	CHECK(run(0U, 40000U, 1U, released) == NONE, "P2a. released (4095) never switches off");
	CHECK(run(0U, 40000U, 1U, down_button) == NONE, "P2b. the down/Walk button never switches off");
	CHECK(run(0U, 40000U, 1U, below_low) == NONE, "P2c. 2047 is outside the on/off window");
	CHECK(run(0U, 40000U, 1U, above_high) == NONE, "P2d. 2731 is outside the on/off window");
	CHECK(run(0U, 20000U, 1U, edge_low) == OFF_TICK, "P2e. 2048 is inside the on/off window");
	CHECK(run(0U, 20000U, 1U, edge_high) == OFF_TICK, "P2f. 2730 is inside the on/off window");

	/* P3: tolerance. Each stray sample costs exactly one sample (it still advances the count). */
	CHECK(run(0U, 40000U, 1U, noisy_1_in_20) == OFF_TICK + 11U * ST,
	      "P3a. one stray sample in 20 does not lose the hold (11 strays before the latch = 88 ms later)");
	CHECK(run(0U, 40000U, 1U, noisy_2_in_10) != NONE, "P3b. two stray samples in a row are still tolerated");
	CHECK(run(0U, 40000U, 1U, noisy_3_in_50) == NONE, "P3c. three in a row end the press");

	/* R1: regression rationale - the inherited rule loses the same noisy press, the new one not */
	CHECK(run_inherited_power(40000U, clean_press) != NONE, "R1a. the inherited rule works on a perfectly clean signal");
	CHECK(run_inherited_power(40000U, noisy_1_in_20) == NONE && run(0U, 40000U, 1U, noisy_1_in_20) != NONE,
	      "R1b. one stray sample per 160 ms: inherited rule never switches off, the new one does");

	/* P4: a short press does nothing; once latched, releasing does not cancel */
	CHECK(run(0U, 40000U, 1U, short_press) == NONE, "P4a. a 1 s press never switches off");
	CHECK(pa4_buttons_power_state() == 0U && pa4_buttons_power_hold() == 0U && !pa4_buttons_any_activity(),
	      "P4b. and the machine is back to idle after the release");
	CHECK(run(0U, 40000U, 1U, release_after_latch) == OFF_TICK, "P4c. released right after the latch: power-off still on time");

	/* P5: a late main loop only delays samples, never bursts them */
	t = run(0U, 60000U, 200U, clean_press);
	CHECK(t != NONE && t >= OFF_TICK, "P5a. called every 50 ms: still switches off, not earlier");
	pa4_buttons_init(0U);
	pa4_buttons_update(10000U, PRESSED, false);
	CHECK(pa4_buttons_power_state() == 1U, "P5b. a long stall yields ONE sample, not a burst");
	pa4_buttons_update(10000U, PRESSED, false);
	CHECK(pa4_buttons_power_state() == 1U, "P5c. and no second sample in the same tick");

	/* P6/P7: wrap and init */
	CHECK(run(0xFFFFF000U, 20000U, 1U, clean_press) == OFF_TICK, "P6. identical timing across the 32-bit wrap");
	(void)run(0U, 20000U, 1U, clean_press);
	pa4_buttons_init(0U);
	CHECK(!pa4_buttons_power_off_latched() && !pa4_buttons_power_off_due(), "P7. init clears the latch");

	/* P8: power_pressed is the confirmed press (states 3..5) */
	start_at(0U);
	feed(2U * ST, PRESSED);
	CHECK(!pa4_buttons_power_pressed() && pa4_buttons_any_activity(), "P8a. 2 samples: activity, not yet a confirmed press");
	feed(ST, PRESSED);
	CHECK(pa4_buttons_power_pressed(), "P8b. 3rd sample: confirmed press");

	/* ==== DOWN / WALK ==== */

	/* D1: Walk follows the confirmed press at once - 3 samples (~24 ms), no 1.6 s hold (FT) */
	g_walk = false;
	start_at(0U);
	feed(3U * ST - 1U, DOWN);
	CHECK(!pa4_buttons_walk_held(), "D1a. not held one tick before the 3rd sample");
	feed(1U, DOWN);
	CHECK(pa4_buttons_walk_held(), "D1b. held on the 3rd pressed sample (24 ms) - Walk starts at once");

	/* D2: before ~1.6 s a dropout stops Walk for its duration and Walk resumes with the hold intact */
	g_walk = true;
	start_at(0U);
	feed(20U * ST, DOWN);                       /* 160 ms of Walk */
	feed(ST, RELEASED);                         /* one sample dropout */
	CHECK(!pa4_buttons_walk_held() && pa4_buttons_down_state() == 4U, "D2a. 1-sample dropout before 1.6 s: Walk off during it");
	feed(ST, DOWN);
	CHECK(pa4_buttons_walk_held(), "D2b. back on the next pressed sample, no new 24 ms confirmation");
	feed(2U * ST, RELEASED);
	feed(ST, DOWN);
	CHECK(pa4_buttons_walk_held(), "D2c. a 2-sample dropout also resumes at once");
	feed(3U * ST, RELEASED);
	CHECK(!pa4_buttons_walk_held() && pa4_buttons_down_state() == 0U, "D2d. a 3-sample dropout ends the press");
	feed(ST, DOWN);
	CHECK(!pa4_buttons_walk_held(), "D2e. and then Walk needs a fresh 3-sample confirmation");

	/* D3: after ~1.6 s of Walk a dropout up to 250 ms is bridged */
	g_walk = true;
	start_at(0U);
	feed(LONG_TICK + 2U * ST, DOWN);            /* hold count past 200 with Walk active */
	ok = pa4_buttons_walk_held() && pa4_buttons_down_hold() > PA4_LONG_COUNT;
	feed(240U * 4U, RELEASED);                  /* 240 ms dropout */
	ok = ok && pa4_buttons_walk_held();
	feed(ST, DOWN);
	CHECK(ok && pa4_buttons_walk_held(), "D3a. after 1.6 s of Walk a 240 ms dropout does not stop Walk");
	feed(260U * 4U, RELEASED);                  /* 260 ms dropout */
	CHECK(!pa4_buttons_walk_held(), "D3b. a 260 ms dropout does stop it");

	/* D3c: the bridge expires exactly after 250 one-millisecond samples outside the window */
	start_at(0U);
	feed(LONG_TICK + 2U * ST, DOWN);
	feed(250U * 4U, RELEASED);
	ok = pa4_buttons_walk_held();
	feed(4U, RELEASED);
	CHECK(ok && !pa4_buttons_walk_held(), "D3c. held through 250 samples outside, dropped on the 251st");

	/* D4: no bridge when Walk is not running (the bridge belongs to Walk, as in the original) */
	g_walk = false;
	start_at(0U);
	feed(LONG_TICK + 2U * ST, DOWN);
	feed(ST, RELEASED);
	CHECK(!pa4_buttons_walk_held(), "D4. button held 1.6 s without Walk: a dropout is not bridged");

	/* D5: down window edges */
	g_walk = false;
	start_at(0U); feed(4U * ST, PA4_DOWN_MIN - 1U);
	CHECK(!pa4_buttons_walk_held(), "D5a. 2853 is outside the down window");
	start_at(0U); feed(4U * ST, PA4_DOWN_MIN);
	CHECK(pa4_buttons_walk_held(), "D5b. 2854 is inside");
	start_at(0U); feed(4U * ST, PA4_DOWN_MAX);
	CHECK(pa4_buttons_walk_held(), "D5c. 3723 is inside");
	start_at(0U); feed(4U * ST, PA4_DOWN_MAX + 1U);
	CHECK(!pa4_buttons_walk_held(), "D5d. 3724 is outside");

	/* D6: the two buttons are independent */
	start_at(0U); feed(20000U, DOWN);
	CHECK(!pa4_buttons_power_off_due() && !pa4_buttons_power_pressed(), "D6a. holding down never touches the on/off machine");
	start_at(0U); feed(4U * ST, PRESSED);
	CHECK(!pa4_buttons_walk_held(), "D6b. on/off does not hold Walk");

	/* ==== WIRING ==== */
	{
		char *mainc = read_whole_file(STRINGIZE(MAIN_C_PATH));
		CHECK(mainc != NULL, "W0. main.c readable");
		if (mainc) {
			const char *init = strstr(mainc, "pa4_buttons_init(control_time_ticks);");
			const char *loop = strstr(mainc, "    while (1){");
			const char *upd = strstr(mainc, "pa4_buttons_update(control_time_ticks, adc_value[5], MS.pushassist_flag != RESET);");
			const char *slow = strstr(mainc, "if (slow_loop_counter > SLOW_LOOP_TICKS){");
			const char *off = strstr(mainc, "if(pa4_buttons_power_off_due()){");
			const char *slow_end = strstr(mainc, "}//end slow loop");
			const char *walk = strstr(mainc, "//--- Walk Assist physical button (PA4) ---");
			CHECK(init && loop && init < loop, "W1. initialised once, before while(1)");
			CHECK(upd && loop && slow && upd > loop && upd < slow,
			      "W2. updated every while(1) iteration with the 4 kHz clock, PA4 and the Walk state, outside the slow loop");
			CHECK(off && slow && slow_end && off > slow && off < slow_end, "W3. the power-off itself stays in the slow loop");
			CHECK(off && span_contains(off, off + 200, "power_off_controller();") &&
			      !span_contains(off, off + 200, "update_hold_ticks"),
			      "W4. it calls power_off_controller() and is not gated by the update hold");
			CHECK(strstr(mainc, "shutoffcounter") == NULL, "W5. the inherited 40 ms counter is gone");
			CHECK(count_of(mainc, "adc_value[5]") == 1, "W6. PA4 has ONE reader in main.c: pa4_buttons_update()");
			CHECK(walk && span_contains(walk, walk + 600, "ui8_walk_btn_state = pa4_buttons_walk_held() ? 1U : 0U;"),
			      "W7. the Walk button is the module's down machine");
			CHECK(strstr(mainc, "uint8_t wa_power_pressed=pa4_buttons_power_pressed() ? 1U : 0U;") != NULL &&
			      strstr(mainc, "!pa4_buttons_power_pressed()){") != NULL,
			      "W8. the Walk latch cancel uses the confirmed on/off press, not a raw threshold");
			CHECK(strstr(mainc, "MS.brake_active_flag || pa4_buttons_any_activity()){") != NULL,
			      "W9. auto-off counts a button press as activity through the module");
			CHECK(strstr(mainc, "GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_12);") != NULL &&
			      strstr(mainc, "GPIO_BC(GPIOB) = GPIO_PIN_8;") != NULL,
			      "W10. PB8 is an output driven low, as idle in the original application");
			free(mainc);
		}
	}

	if (host_test_failures) {
		printf("pa4 buttons: %d CHECK(S) FAILED\n", host_test_failures);
		return 1;
	}
	printf("pa4 buttons: ALL CHECKS PASSED\n");
	return 0;
}
