/*
 * FW-102 host harness — runs the SHIPPED pas_trace.c, not a copy of it.
 *
 * Build and run (any host compiler; see run-host-tests.ps1):
 *   gcc -std=c11 -Wall -Wextra -I../../inc -o fw102 fw102_pas_trace_host.c \
 *       ../../src/pas_trace.c && ./fw102
 *
 * WHY THIS EXISTS. Same lesson as FW-101 (see fw101_episode_host.c): a recorder that is not
 * exercised by a test is not evidence. This module is the ring buffer that keeps raw quadrature
 * transitions around a suspicious event — the one piece of the investigation that works BELOW
 * the direction decoder rather than trusting its verdict. If ITS bookkeeping were wrong, nobody
 * would know, because the shape it shows is the thing being used to judge everything above it.
 *
 * What each block checks:
 *   1. ordinary transitions, however many, never arm a capture
 *   2. a short gap arms one, and it freezes at exactly PRE+POST once warmed up
 *   3. an illegal two-bit jump arms one too, independent of gap length
 *   4. once frozen, further writes are dropped until release()
 *   5. a latch drop with no transition of its own still triggers (retroactively)
 *   6. ...even with nothing recorded yet, safely
 *   7. a latch drop while already armed tags the flag without moving the trigger or the timer
 *   8. release() fully resets — a short pre-history after it is reported honestly, not padded
 */

#include <stdio.h>
#include <string.h>

#include "pas_trace.h"

static int failures;

static void check(int ok, const char *label)
{
	if (!ok) {
		failures++;
		printf("  FAIL  %s\n", label);
	}
}

/* A single-bit (valid) quadrature transition with an otherwise ordinary, boring context. */
static pas_trace_input_t clean(uint16_t gap, uint16_t disc_pos)
{
	pas_trace_input_t in;
	memset(&in, 0, sizeof(in));
	in.from_state = 0;
	in.to_state = 1;
	in.reverse = false;
	in.gap_ticks = gap;
	in.disc_pos = disc_pos;
	in.load_centikg = 100;
	in.torque_raw_mv = 900;
	in.torque_fast = 50;
	in.iq_setpoint = 30;
	in.brake = false;
	in.rolling = true;
	in.latched = true;
	return in;
}

int main(void)
{
	printf("FW-102 pas_trace, against the shipped C module\n");
	printf("  PRE %u, POST %u, LEN %u, short-gap <= %u ticks\n",
		(unsigned)PAS_TRACE_PRE, (unsigned)PAS_TRACE_POST, (unsigned)PAS_TRACE_LEN,
		(unsigned)PAS_TRACE_SHORT_GAP_TICKS);

	/* --- ordinary transitions never trigger, however many ---------------------------- */
	{
		pas_trace_init();
		for (int i = 0; i < 500; i++) {
			pas_trace_input_t in = clean(200, (uint16_t)i);
			pas_trace_transition(&in);
		}
		check(!pas_trace_ready(), "boring: 500 ordinary transitions never arm a capture");
		check(pas_trace_count() == 0, "boring: nothing to read while not ready");
	}

	/* --- a short gap triggers, and freezes exactly PRE+POST once warmed up ----------- */
	{
		pas_trace_init();
		/* A known, ordered pre-history: disc_pos = 0..199 (exceeds PRE=128, ring keeps last 200). */
		for (int i = 0; i < 200; i++) {
			pas_trace_input_t in = clean(200, (uint16_t)i);
			pas_trace_transition(&in);
		}
		check(!pas_trace_ready(), "short-gap: still just watching before the trigger");

		pas_trace_input_t trig = clean(1, 9999); /* SHORT_GAP_TICKS=3, so 1 tick is short */
		pas_trace_transition(&trig);
		check(!pas_trace_ready(), "short-gap: armed, not yet frozen (1 of POST written)");

		for (unsigned i = 0; i < PAS_TRACE_POST - 1U; i++) {
			pas_trace_input_t in = clean(200, (uint16_t)(1000 + i));
			pas_trace_transition(&in);
		}
		check(pas_trace_ready(), "short-gap: PRE+POST writes after warm-up freezes the capture");
		check(pas_trace_count() == PAS_TRACE_LEN,
			"short-gap: a fully-warmed ring freezes at full length");

		uint16_t trig_idx = pas_trace_trigger_index();
		check(trig_idx == PAS_TRACE_PRE,
			"short-gap: exactly PRE samples of history precede the trigger");

		pas_trace_sample_t s;
		check(pas_trace_get(trig_idx, &s), "short-gap: the trigger index is readable");
		check((s.flags & PAS_TR_TRIGGER) != 0, "short-gap: the trigger sample is flagged TRIGGER");
		check((s.flags & PAS_TR_SHORT_GAP) != 0,
			"short-gap: ...and flagged SHORT_GAP, the reason it triggered");
		check(s.gap_ticks == 1, "short-gap: the trigger sample's own data survived capture");
		check(s.disc_pos == 9999 % 96, "short-gap: disc_pos is reduced modulo 96 by the module");

		pas_trace_sample_t oldest;
		check(pas_trace_get(0, &oldest), "short-gap: index 0 (the oldest sample) is readable");
		check((oldest.flags & PAS_TR_TRIGGER) == 0,
			"short-gap: index 0 is not itself the trigger");
		check(!pas_trace_get(pas_trace_count(), &oldest),
			"short-gap: reading exactly at count is out of range");

		pas_trace_release();
		check(!pas_trace_ready(), "short-gap: release() clears ready");
		check(pas_trace_count() == 0, "short-gap: release() clears count");
	}

	/* --- an illegal two-bit jump triggers too, independent of gap length ------------- */
	{
		pas_trace_init();
		for (int i = 0; i < 50; i++) {
			pas_trace_input_t in = clean(200, (uint16_t)i);
			pas_trace_transition(&in);
		}
		pas_trace_input_t trig = clean(200, 777); /* long gap: not a short-gap trigger */
		trig.from_state = 0;
		trig.to_state = 3; /* both quadrature bits change: impossible from real motion */
		pas_trace_transition(&trig);
		for (unsigned i = 0; i < PAS_TRACE_POST - 1U; i++) {
			pas_trace_input_t in = clean(200, (uint16_t)(2000 + i));
			pas_trace_transition(&in);
		}
		check(pas_trace_ready(), "two-bit: an illegal transition alone arms a capture");

		pas_trace_sample_t s;
		check(pas_trace_get(pas_trace_trigger_index(), &s), "two-bit: trigger index readable");
		check((s.flags & PAS_TR_TWO_BIT) != 0, "two-bit: flagged TWO_BIT");
		check((s.flags & PAS_TR_SHORT_GAP) == 0, "two-bit: NOT flagged SHORT_GAP - the gap was long");
		check((s.flags & PAS_TR_TRIGGER) != 0, "two-bit: flagged as the trigger");
		pas_trace_release();
	}

	/* --- once frozen, further transitions are dropped until release() ---------------- */
	{
		pas_trace_init();
		pas_trace_input_t trig = clean(1, 111);
		pas_trace_transition(&trig);
		for (unsigned i = 0; i < PAS_TRACE_POST - 1U; i++) {
			pas_trace_input_t in = clean(200, (uint16_t)(3000 + i));
			pas_trace_transition(&in);
		}
		check(pas_trace_ready(), "frozen: capture completed");
		uint16_t count_before = pas_trace_count();

		pas_trace_input_t intruder = clean(1, 55555);
		intruder.iq_setpoint = 65535;
		pas_trace_transition(&intruder);

		check(pas_trace_count() == count_before, "frozen: count unchanged by a write while ready");
		pas_trace_sample_t last;
		check(pas_trace_get(pas_trace_count() - 1U, &last), "frozen: last sample still readable");
		check(last.iq_setpoint != 65535, "frozen: the intruding write was dropped, not appended");

		pas_trace_release();
		check(!pas_trace_ready(), "frozen: release() re-opens the watch");
	}

	/* --- a latch drop with no transition of its own still triggers, retroactively ---- */
	{
		pas_trace_init();
		for (int i = 0; i < 10; i++) {
			pas_trace_input_t in = clean(200, (uint16_t)i);
			pas_trace_transition(&in);
		}
		pas_trace_latch_loss();
		check(!pas_trace_ready(), "latch-loss: armed, not yet frozen");

		/* Nothing has been consumed from POST yet: a full POST is still needed. */
		for (unsigned i = 0; i < PAS_TRACE_POST; i++) {
			pas_trace_input_t in = clean(200, (uint16_t)(4000 + i));
			pas_trace_transition(&in);
		}
		check(pas_trace_ready(), "latch-loss: completes after a full POST of further writes");

		pas_trace_sample_t s;
		check(pas_trace_get(pas_trace_trigger_index(), &s), "latch-loss: trigger index readable");
		check((s.flags & PAS_TR_LATCH_LOSS) != 0, "latch-loss: flagged LATCH_LOSS");
		check((s.flags & PAS_TR_TRIGGER) != 0, "latch-loss: flagged TRIGGER");
		check(s.disc_pos == 9,
			"latch-loss: the trigger IS the last write before the call, not a new one");
		pas_trace_release();
	}

	/* --- latch loss with nothing recorded yet is a safe no-op ------------------------ */
	{
		pas_trace_init();
		pas_trace_latch_loss();
		check(!pas_trace_ready(), "latch-loss/empty: no crash, no spurious arm");
		check(pas_trace_count() == 0, "latch-loss/empty: still nothing captured");
	}

	/* --- latch loss while already armed tags the flag, doesn't restart the timer ----- */
	{
		pas_trace_init();
		pas_trace_input_t trig = clean(1, 42); /* triggers immediately via SHORT_GAP */
		pas_trace_transition(&trig);
		pas_trace_latch_loss(); /* already armed: must not move the trigger or the countdown */

		for (unsigned i = 0; i < PAS_TRACE_POST - 1U; i++) {
			pas_trace_input_t in = clean(200, (uint16_t)(5000 + i));
			pas_trace_transition(&in);
		}
		check(pas_trace_ready(),
			"latch-loss/already-armed: still completes at the normal length");

		pas_trace_sample_t s;
		check(pas_trace_get(pas_trace_trigger_index(), &s),
			"latch-loss/already-armed: trigger index readable");
		check((s.flags & PAS_TR_SHORT_GAP) != 0 && (s.flags & PAS_TR_LATCH_LOSS) != 0,
			"latch-loss/already-armed: BOTH reasons recorded on the same original trigger");
		pas_trace_release();
	}

	/* --- release() fully resets: a short pre-history isn't padded with stale data ---- */
	{
		pas_trace_init();
		pas_trace_input_t trig1 = clean(1, 10);
		pas_trace_transition(&trig1);
		for (unsigned i = 0; i < PAS_TRACE_POST - 1U; i++) {
			pas_trace_input_t in = clean(200, (uint16_t)(6000 + i));
			pas_trace_transition(&in);
		}
		check(pas_trace_ready(), "release/reuse: first capture completes");
		pas_trace_release();

		/* Only 5 writes before the second trigger - far short of PAS_TRACE_PRE. */
		for (int i = 0; i < 5; i++) {
			pas_trace_input_t in = clean(200, (uint16_t)(7000 + i));
			pas_trace_transition(&in);
		}
		pas_trace_input_t trig2 = clean(1, 20);
		pas_trace_transition(&trig2);
		for (unsigned i = 0; i < PAS_TRACE_POST - 1U; i++) {
			pas_trace_input_t in = clean(200, (uint16_t)(8000 + i));
			pas_trace_transition(&in);
		}
		check(pas_trace_ready(), "release/reuse: a second, independent capture completes");
		check(pas_trace_count() == 5U + PAS_TRACE_POST,
			"release/reuse: a short pre-history is reported honestly, not padded to full length");
		check(pas_trace_trigger_index() == 5,
			"release/reuse: the trigger sits right after the (short) pre-history");
	}

	if (failures == 0) {
		printf("All FW-102 host checks passed.\n");
		return 0;
	}
	printf("\n%d FW-102 host check(s) FAILED.\n", failures);
	return 1;
}
