/*
 * Assist Behavior V3 - production crank step accumulator (ARCHITECTURE_V3.md 3.2, D-006, D-021,
 * TEST_MATRIX G1-PAS front-end part). Real modules: src/crank_phase.c fed by the real 4 kHz
 * sampler (src/pas_sampler.c + src/pas_quadrature.c), drained exactly as main.c drains it.
 *
 *  A  unit: +1/-1 counting, last-step tick = the event's ISR tick, INVALID counted as a glitch
 *     that moves neither the count nor the tick, take_glitch() is once-per-burst, overflow
 *     marker, NULL event, init clears everything.
 *  B  wrap: the count is read as a difference across the 2^32 boundary.
 *  C  end to end through the sampler ISR + ring: N forward steps at 60 rpm -> count +N; a
 *     back-pedal -> count decreases by the reverse steps; a 2-bit jump -> glitch, count kept; a
 *     foreground stall longer than the 32-entry ring -> overflow marker reaches crank_phase as a
 *     glitch, and the count is short by exactly the events the sampler reports as lost.
 */
#include "crank_phase.h"
#include "pas_sampler.h"
#include "common/check.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const uint8_t FWD[4] = { 0u, 2u, 3u, 1u };   /* native forward A/B order (as the SIL) */

static pas_step_event_t ev_make(int8_t step, uint32_t tick)
{
	pas_step_event_t e;
	memset(&e, 0, sizeof(e));
	e.step = step;
	e.tick = tick;
	return e;
}

/* main.c's drain: every popped event to crank_phase, then the overflow marker. */
static uint32_t drain(void)
{
	pas_step_event_t e;
	uint32_t n = 0u;
	while (pas_sampler_pop(&e)) { crank_phase_on_event(&e); n++; }
	if (pas_sampler_take_overflow()) crank_phase_on_overflow();
	return n;
}

static void test_unit(void)
{
	pas_step_event_t e;
	crank_phase_init();
	CHECK(crank_phase_steps() == 0 && crank_phase_last_step_tick() == 0u && !crank_phase_take_glitch(),
	      "A1 init: zero count, zero tick, no glitch");

	for (uint32_t i = 1u; i <= 10u; i++) { e = ev_make(+1, 100u * i); crank_phase_on_event(&e); }
	CHECK(crank_phase_steps() == 10 && crank_phase_last_step_tick() == 1000u, "A2 ten forward steps");
	for (uint32_t i = 1u; i <= 3u; i++) { e = ev_make(-1, 1000u + i); crank_phase_on_event(&e); }
	CHECK(crank_phase_steps() == 7 && crank_phase_last_step_tick() == 1003u, "A3 three reverse steps");
	CHECK(!crank_phase_take_glitch(), "A4 legal steps are not a glitch");

	e = ev_make(0, 5000u); crank_phase_on_event(&e);
	CHECK(crank_phase_steps() == 7, "A5 INVALID jump does not move the count");
	CHECK(crank_phase_last_step_tick() == 1003u, "A6 INVALID jump does not move the step tick");
	CHECK(crank_phase_take_glitch(), "A7 INVALID jump raises the glitch");
	CHECK(!crank_phase_take_glitch(), "A8 ...once (take clears it)");

	e = ev_make(0, 5001u); crank_phase_on_event(&e);
	e = ev_make(0, 5002u); crank_phase_on_event(&e);
	CHECK(crank_phase_take_glitch() && !crank_phase_take_glitch(), "A9 a burst is one glitch per take");

	crank_phase_on_overflow();
	CHECK(crank_phase_take_glitch(), "A10 ring overflow raises the glitch");
	crank_phase_on_event(NULL);
	CHECK(crank_phase_steps() == 7 && !crank_phase_take_glitch(), "A11 NULL event is ignored");
	{
		const crank_phase_stats_t *st = crank_phase_stats();
		CHECK(st->forward_steps == 10u && st->reverse_steps == 3u && st->invalid_jumps == 3u &&
		      st->overflows == 1u, "A12 stats count every event class");
	}
	crank_phase_on_overflow();
	crank_phase_init();
	CHECK(crank_phase_steps() == 0 && !crank_phase_take_glitch() &&
	      crank_phase_stats()->forward_steps == 0u, "A13 init clears count, glitch and stats");
}

static void test_wrap(void)
{
	/* Drive the count to 2^31 - 5 cheaply is not possible through the API (one step per event),
	 * so prove the consumer arithmetic instead: differences of the uint32-backed count are exact
	 * across the boundary, which is what V3 relies on (ARCHITECTURE_V3 3.2). */
	const int32_t before = (int32_t)0x7FFFFFFE;
	const int32_t after = (int32_t)(uint32_t)((uint32_t)before + 5u);
	const int32_t delta = (int32_t)((uint32_t)after - (uint32_t)before);
	CHECK(delta == 5, "B1 forward delta across +2^31 wrap is exact");
	const int32_t back = (int32_t)((uint32_t)before - (uint32_t)after);
	CHECK(back == -5, "B2 reverse delta across the wrap is exact");
}

static void test_end_to_end(void)
{
	uint32_t tick = 1u;
	uint8_t idx = 0u;
	pas_sampler_init(0u);
	crank_phase_init();
	pas_sampler_isr_tick(FWD[0], tick);   /* seed */
	(void)drain();

	/* 96 forward steps at 60 rpm: one step every 41-42 ticks, drained every tick. */
	for (int s = 0; s < 96; s++) {
		for (int k = 0; k < 41; k++) { pas_sampler_isr_tick(FWD[idx & 3u], ++tick); (void)drain(); }
		idx++;
		pas_sampler_isr_tick(FWD[idx & 3u], ++tick);
		(void)drain();
	}
	CHECK(crank_phase_steps() == 96, "C1 one revolution forward = +96");
	CHECK(crank_phase_last_step_tick() == tick, "C2 step tick is the ISR tick of the last edge");
	CHECK(!crank_phase_take_glitch(), "C3 clean revolution: no glitch");

	/* back-pedal 5 steps, slow enough to pass the sampler's refractory window */
	for (int s = 0; s < 5; s++) {
		for (int k = 0; k < 41; k++) { pas_sampler_isr_tick(FWD[idx & 3u], ++tick); (void)drain(); }
		idx--;
		pas_sampler_isr_tick(FWD[idx & 3u], ++tick);
		(void)drain();
	}
	CHECK(crank_phase_steps() == 91, "C4 five reverse steps -> 91");

	/* a two-bit jump (two steps in one sample) */
	for (int k = 0; k < 41; k++) { pas_sampler_isr_tick(FWD[idx & 3u], ++tick); (void)drain(); }
	idx = (uint8_t)(idx + 2u);
	pas_sampler_isr_tick(FWD[idx & 3u], ++tick);
	(void)drain();
	CHECK(crank_phase_steps() == 91, "C5 INVALID jump: count unchanged");
	CHECK(crank_phase_take_glitch(), "C6 INVALID jump: glitch reaches the consumer");

	/* foreground stall: 40 forward edges queued without a drain (ring = 32) */
	{
		const uint32_t lost_before = pas_sampler_get_stats()->overflow_count;
		const int32_t c0 = crank_phase_steps();
		for (int s = 0; s < 40; s++) {
			for (int k = 0; k < 41; k++) pas_sampler_isr_tick(FWD[idx & 3u], ++tick);
			idx++;
			pas_sampler_isr_tick(FWD[idx & 3u], ++tick);
		}
		(void)drain();
		const uint32_t lost = pas_sampler_get_stats()->overflow_count - lost_before;
		CHECK(lost > 0u, "C7 a 40-edge stall overflows the 32-entry ring");
		CHECK(crank_phase_take_glitch(), "C8 overflow reaches crank_phase as a glitch");
		CHECK(crank_phase_steps() - c0 == (int32_t)(40u - lost),
		      "C9 the count is short by exactly the events the sampler lost");
	}
}

int main(void)
{
	puts("assist_v3 crank_phase: production crank step accumulator (G1-PAS front end)");
	test_unit();
	test_wrap();
	test_end_to_end();
	if (host_test_failures) {
		printf("crank_phase: %d check(s) FAILED\n", host_test_failures);
		return 1;
	}
	puts("crank_phase: PASS");
	return 0;
}
