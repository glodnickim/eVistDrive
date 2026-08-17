/*
 * FW-106 host harness — runs the SHIPPED ride_episode.c, pas_trace.c and pas_raw.c.
 *
 * Build and run (see run-host-tests.ps1):
 *   gcc -std=c11 -Wall -Wextra -I../../inc -o fw106 fw106_recorder_host.c \
 *       ../../src/ride_episode.c ../../src/pas_trace.c ../../src/pas_raw.c && ./fw106
 *
 * WHY THIS EXISTS. FW-106 was opened because the DIAGNOSTIC recorders were themselves wrong:
 * 22 published episodes carried t_latch EARLIER than t_last_reverse, which is arithmetically
 * impossible if both are measured from one anchor, and the two halves of a trace sample could
 * not be paired because one of them carried no key. Recorders that are not exercised by a test
 * are not evidence - that is the whole lesson of FW-101, and it applies hardest to the code
 * written to check the other code.
 *
 * What each block checks:
 *   A. ride_episode: a second reverse step invalidates the arming (the Bug 1 fix), including
 *      the case that used to be skipped entirely - a reverse arriving at zero current
 *   B. ride_episode: the queue that replaced the single overwritten "published" slot, and the
 *      accounting identity that survives a full queue
 *   C. pas_trace: two captures held at once, the slot budget as a counted limit, and capture ids
 *   D. pas_raw: the ISR ring, freeze/PRE/POST, and the counters that must and must not move
 *   E. the shared capture id: the decoder's view and the raw view of ONE event agree
 *   D4. pas_raw: an ISR event landing WHILE pas_raw_freeze() is copying the pre-history is not
 *       lost - the second-implementation defect this file's own tests found, closed by making
 *       the ISR's and the copy's regions of the frozen slot disjoint (see pas_raw.c). Uses the
 *       PAS_RAW_COPY_HOOK test seam, compiled in only for this harness (see run-host-tests.ps1),
 *       to call the real ISR entry point FROM INSIDE the real copy loop - a single-threaded test
 *       calling the ISR only after pas_raw_freeze() returns cannot reach this at all.
 */

#include <stdio.h>
#include <string.h>

#include "ride_episode.h"
#include "pas_trace.h"
#include "pas_raw.h"

#ifdef PAS_RAW_COPY_HOOK
/*
 * The test-seam definition pas_raw.c calls mid-copy. Armed/disarmed per test block so every
 * OTHER pas_raw_freeze() call in this file (blocks C, D, E, F) is a plain no-op here, exactly as
 * if the seam did not exist.
 */
static bool     hook_armed;
static uint16_t hook_at_index;
static uint8_t  hook_inject_state;
static uint32_t hook_inject_tick;

void pas_raw_copy_hook(uint16_t i)
{
	if (hook_armed && i == hook_at_index) {
		hook_armed = false;
		pas_raw_isr_sample(hook_inject_state, hook_inject_tick);
	}
}
#endif

static int failures;

static void check(int ok, const char *label)
{
	if (!ok) {
		failures++;
		printf("  FAIL  %s\n", label);
	}
}

/* 4 kHz: one millisecond is four ticks. */
#define MS(x) ((uint32_t)(x) * 4U)

static ride_episode_input_t ep_input(int32_t iq, int32_t pre_ramp, uint16_t arm_seq)
{
	ride_episode_input_t in;
	memset(&in, 0, sizeof(in));
	in.iq_setpoint = iq;
	in.iq_pre_ramp = pre_ramp;
	in.arm_seq = arm_seq;
	in.required_steps = 4;
	in.load_threshold_centikg = 70;
	return in;
}

/* Run the recorder for a stretch of time at a steady current and arming counter. */
static void ep_run(uint32_t from_tick, uint32_t to_tick, int32_t iq, uint16_t arm_seq)
{
	for (uint32_t t = from_tick; t < to_tick; t += 4U) {
		ride_episode_input_t in = ep_input(iq, iq, arm_seq);
		ride_episode_tick(&in, t);
	}
}

static pas_trace_input_t clean(uint16_t gap, uint16_t disc_pos)
{
	pas_trace_input_t in;
	memset(&in, 0, sizeof(in));
	in.from_state = 0;
	in.to_state = 1;
	in.gap_ticks = gap;
	in.disc_pos = disc_pos;
	in.load_centikg = 100;
	in.torque_raw_mv = 900;
	in.iq_setpoint = 30;
	return in;
}

/* Drive one full capture into whichever slot is currently watching. Returns its capture id. */
static uint8_t drive_one_capture(void)
{
	pas_trace_input_t trig = clean(1, 111);   /* short gap: triggers immediately */
	uint8_t id = pas_trace_transition(&trig);
	for (unsigned i = 0; i < PAS_TRACE_POST - 1U; i++) {
		pas_trace_input_t in = clean(200, (uint16_t)(3000 + i));
		pas_trace_transition(&in);
	}
	return id;
}

/* Toggle the A line so pas_raw sees `n` real transitions, one per tick from `tick0`. */
static void raw_drive(uint32_t tick0, unsigned n)
{
	static uint8_t s = 0;
	for (unsigned i = 0; i < n; i++) {
		s ^= 1U;
		pas_raw_isr_sample(s, tick0 + i);
	}
}

/*
 * init + one priming sample. The module adopts the line state on its first ever call without
 * inventing a transition out of it (there is no previous state to compare against), so without
 * this the first raw_drive() would yield n-1 events and every count below would be off by one.
 */
static void raw_begin(void)
{
	pas_raw_init();
	pas_raw_isr_sample(0, 0);
}

int main(void)
{
	printf("FW-106 recorders, against the shipped C modules\n");
	printf("  episode queue %u, trace slots %u, raw ring %u (PRE %u / POST %u)\n",
		(unsigned)RIDE_EPISODE_QUEUE_LEN, (unsigned)PAS_TRACE_SLOTS,
		(unsigned)PAS_RAW_RING_LEN, (unsigned)PAS_RAW_FREEZE_PRE,
		(unsigned)PAS_RAW_FREEZE_POST);

	/* --- A1. reverse, latch, SECOND reverse, latch again: the Bug 1 shape ---------------- */
	{
		ride_episode_init();
		ride_episode_set_session_id(7);

		/* The reverse that starts the episode, at a healthy current. */
		ride_episode_reverse_step(100, 0, MS(0));
		/* The drive collapses: WAIT_DIP -> WAIT_RECOVER. */
		ep_run(MS(1), MS(100), 10, 0);
		/* The latch arms 100 ms in. */
		ep_run(MS(100), MS(200), 10, 1);

		/* A SECOND reverse step at 200 ms. Everything about the arming is now stale. */
		ride_episode_reverse_step(10, 1, MS(200));

		/* It arms again at 300 ms, and the drive comes back. */
		ep_run(MS(201), MS(300), 10, 1);
		ep_run(MS(300), MS(400), 10, 2);
		ep_run(MS(400), MS(500), 100, 2);

		ride_episode_result_t r;
		ride_episode_get_result(&r);
		check(r.number == 1, "A1: exactly one episode was published");
		check(r.t_last_reverse_ms == 200,
			"A1: t_last_reverse is the SECOND reverse step, as it always was");
		check(r.t_latch_ms >= r.t_last_reverse_ms,
			"A1: t_latch is not earlier than t_last_reverse - the invariant that failed 22x");
		check(r.t_latch_ms == 300,
			"A1: the published latch is the one that followed the last reverse, not the first");
	}

	/* --- A2. the same, but the second reverse arrives at ZERO current -------------------- */
	{
		ride_episode_init();
		ride_episode_reverse_step(100, 0, MS(0));
		ep_run(MS(1), MS(100), 10, 0);
		ep_run(MS(100), MS(200), 10, 1);          /* latch arms at 100 ms */

		/*
		 * Zero current. The old code returned immediately here and left t_latch, have_arm and
		 * the arm_* snapshot standing - while still advancing nothing - so the episode kept an
		 * arming that the reverse step had just destroyed. This is the case the fix had to
		 * reach BEFORE the early return.
		 */
		ride_episode_reverse_step(0, 1, MS(200));

		ep_run(MS(201), MS(300), 10, 1);
		ep_run(MS(300), MS(400), 10, 2);          /* re-arms at 300 ms */
		ep_run(MS(400), MS(500), 100, 2);

		ride_episode_result_t r;
		ride_episode_get_result(&r);
		check(r.number == 1, "A2: one episode published");
		check(r.t_last_reverse_ms == 200, "A2: the zero-current reverse still moved t_last_reverse");
		check(r.t_latch_ms >= r.t_last_reverse_ms,
			"A2: the invariant holds even when the reverse arrived at zero current");
		check(r.t_latch_ms == 300, "A2: the stale 100 ms latch was discarded, not published");
	}

	/* --- A3. no second reverse: the ordinary case must be untouched ---------------------- */
	{
		ride_episode_init();
		ride_episode_reverse_step(100, 0, MS(0));
		ep_run(MS(1), MS(100), 10, 0);
		ep_run(MS(100), MS(300), 10, 1);
		ep_run(MS(300), MS(400), 100, 1);

		ride_episode_result_t r;
		ride_episode_get_result(&r);
		check(r.number == 1, "A3: one episode published");
		check(r.t_last_reverse_ms == 0, "A3: the anchoring reverse still reads 0 ms");
		check(r.t_latch_ms == 100, "A3: a single arming is timed exactly as before the fix");
		check((r.flags & RIDE_EP_FLAG_LATCH_ARMED) != 0, "A3: the armed flag survives");
	}

	/* --- A4. iq_pre_ramp_at_target_ready is frozen PER RECORD, not shared/live state ------ */
	{
		ride_episode_init();
		ride_episode_set_session_id(1);

		/* Episode 1: iq_ref=100. Dip below 50 %, then reach target-ready with pre_ramp=85 -
		 * DISTINCT from what episode 2 below will use - while iq_setpoint stays low (30) so
		 * target-ready fires WITHOUT also completing the episode on the same tick. */
		ride_episode_reverse_step(100, 0, MS(0));
		ep_run(MS(1), MS(50), 30, 0);                 /* dip: 30 < 50% of 100 -> WAIT_RECOVER */
		{
			ride_episode_input_t in = ep_input(30, 85, 0);   /* pre_ramp=85 >= 80% of 100 */
			ride_episode_tick(&in, MS(51));
		}
		ep_run(MS(52), MS(150), 90, 0);               /* 90 >= 80% of 100 -> recovers, publishes */

		ride_episode_result_t r1;
		check(ride_episode_queue_peek(&r1), "A4: episode 1 is queued");
		check(r1.iq_pre_ramp_at_target_ready == 85,
			"A4: episode 1 recorded pre-ramp 85 at ITS OWN target-ready instant");

		/* Episode 2: a DIFFERENT anchor (iq_ref=60) and a DIFFERENT target-ready value (50). */
		ride_episode_reverse_step(60, 0, MS(2000));
		ep_run(MS(2001), MS(2050), 20, 0);            /* dip: 20 < 50% of 60 */
		{
			ride_episode_input_t in = ep_input(20, 50, 0);   /* pre_ramp=50 >= 80% of 60 (48) */
			ride_episode_tick(&in, MS(2051));
		}
		ep_run(MS(2052), MS(2150), 55, 0);            /* 55 >= 80% of 60 -> recovers, publishes */
		check(ride_episode_queue_depth() == 2, "A4: both episodes are queued at once");

		/*
		 * "Live state change before transmission": more processing happens - a third reverse
		 * step and some ticks that never complete an episode - representing whatever the ride
		 * might be doing by the time a dump actually gets around to reading episode 1's record.
		 */
		ride_episode_reverse_step(200, 0, MS(3000));
		ep_run(MS(3001), MS(3100), 150, 0);

		ride_episode_result_t recheck1;
		check(ride_episode_queue_peek(&recheck1), "A4: episode 1 is still there, still oldest");
		check(recheck1.iq_pre_ramp_at_target_ready == 85,
			"A4: episode 1's value is STILL 85 - untouched by episode 2 or the live churn since. "
			"A live re-read at this point (the old bug) would show whatever is true NOW, not 85");
		check(recheck1.number == r1.number, "A4: ...the same record, not a coincidentally equal one");

		ride_episode_queue_drop();
		ride_episode_result_t r2;
		check(ride_episode_queue_peek(&r2), "A4: episode 2 is now at the front");
		check(r2.iq_pre_ramp_at_target_ready == 50,
			"A4: episode 2 kept its OWN distinct value (50) - neither episode 1's (85) nor "
			"anything from the live churn afterwards");
	}

	/* --- B1. the queue keeps every episode, and stamps the session ----------------------- */
	{
		ride_episode_init();
		ride_episode_set_session_id(3);
		for (int e = 0; e < 3; e++) {
			uint32_t base = MS(1000) * (uint32_t)(e + 1);
			ride_episode_reverse_step(100, (uint16_t)(10 * e), base);
			ep_run(base + MS(1), base + MS(100), 10, (uint16_t)(10 * e));
			ep_run(base + MS(100), base + MS(200), 100, (uint16_t)(10 * e + 1));
		}
		check(ride_episode_queue_depth() == 3, "B1: all three episodes are queued, none overwritten");
		check(ride_episode_queue_enqueued() == 3, "B1: enqueued total agrees");

		ride_episode_result_t r;
		check(ride_episode_queue_peek(&r), "B1: the oldest is readable");
		check(r.session_id == 3, "B1: the session is stamped on the record itself");
		check(r.number == 1, "B1: peek returns the OLDEST, not the newest");
		ride_episode_queue_drop();
		check(ride_episode_queue_depth() == 2, "B1: drop removes exactly one");
		check(ride_episode_queue_enqueued() - ride_episode_queue_removed() == 2,
			"B1: pending = enqueued - removed");
	}

	/* --- B2. a full queue REFUSES the new record and never evicts an old one ------------- */
	{
		ride_episode_init();
		for (unsigned e = 0; e < RIDE_EPISODE_QUEUE_LEN + 3U; e++) {
			uint32_t base = MS(1000) * (uint32_t)(e + 1);
			ride_episode_reverse_step(100, (uint16_t)e, base);
			ep_run(base + MS(1), base + MS(100), 10, (uint16_t)e);
			ep_run(base + MS(100), base + MS(200), 100, (uint16_t)(e + 1));
		}
		check(ride_episode_queue_depth() == RIDE_EPISODE_QUEUE_LEN, "B2: the queue is full, not over-full");
		check(ride_episode_queue_enqueued() == RIDE_EPISODE_QUEUE_LEN,
			"B2: refused records never counted as enqueued");
		check(ride_episode_queue_rejected() == 3, "B2: all three refusals are counted");

		ride_episode_result_t r;
		check(ride_episode_queue_peek(&r), "B2: the queue still reads");
		check(r.number == 1, "B2: the OLDEST record survived - nothing was evicted");

		/*
		 * The identity the whole accounting rests on: every record that was ever accepted has
		 * left exactly once, and the refusals do not disturb the balance.
		 */
		while (ride_episode_queue_depth() > 0) ride_episode_queue_drop();
		check(ride_episode_queue_enqueued() - ride_episode_queue_removed() == 0,
			"B2: removals balance the accepted records, refusals notwithstanding");
	}

	/* --- C1. two captures are held at once, with different ids --------------------------- */
	{
		pas_trace_init();
		uint8_t id1 = drive_one_capture();
		check(pas_trace_slot_ready(0), "C1: the first capture froze");
		uint8_t id2 = drive_one_capture();
		check(pas_trace_slot_ready(1), "C1: the SECOND capture was kept, not lost");
		check(id1 != id2, "C1: the two captures carry different ids");
		check(id1 != PAS_TRACE_NO_CAPTURE && id2 != PAS_TRACE_NO_CAPTURE,
			"C1: a real id is never the no-capture sentinel");
		check(pas_trace_slot_capture_id(0) == id1, "C1: slot 0 kept the id it was given");
		check(pas_trace_slot_capture_id(1) == id2, "C1: slot 1 kept its own");
		check(pas_trace_oldest_ready_slot() == 0, "C1: the older capture drains first");
		check(pas_trace_capture_slots_full() == 0, "C1: nothing was refused yet");
	}

	/* --- C2. with both slots full, a further trigger is COUNTED, not silently lost ------- */
	{
		uint16_t before = pas_trace_capture_slots_full();
		pas_trace_input_t trig = clean(1, 222);
		uint8_t id3 = pas_trace_transition(&trig);
		check(id3 == PAS_TRACE_NO_CAPTURE, "C2: no capture is issued when every slot is frozen");
		check(pas_trace_capture_slots_full() == before + 1U,
			"C2: the trigger that could not be kept is still counted");
		check(pas_trace_slot_ready(0) && pas_trace_slot_ready(1),
			"C2: neither frozen capture was disturbed");

		pas_trace_slot_release(0);
		check(!pas_trace_slot_ready(0), "C2: release frees the slot");
		uint8_t id4 = drive_one_capture();
		check(id4 != PAS_TRACE_NO_CAPTURE, "C2: watching resumes once a slot is free again");
	}

	/* --- C3. ids never collide with the sentinel, however many are issued ---------------- */
	{
		pas_trace_init();
		int saw_sentinel = 0;
		for (int i = 0; i < 600; i++) {
			uint8_t id = drive_one_capture();
			if (id == PAS_TRACE_NO_CAPTURE) saw_sentinel = 1;
			/* Keep a slot free so every attempt really issues an id. */
			pas_trace_slot_release((uint8_t)(pas_trace_oldest_ready_slot() < 0
				? 0 : pas_trace_oldest_ready_slot()));
		}
		check(!saw_sentinel, "C3: 600 captures, and 0xFF was never issued as a real id");
	}

	/* --- D1. the ISR ring records transitions only, and counts its own laps ------------- */
	{
		raw_begin();
		for (unsigned i = 0; i < 100; i++) pas_raw_isr_sample(0, 1000U + i); /* no change at all */
		check(pas_raw_ring_wraps() == 0, "D1: an unchanging line writes nothing");
		check(!pas_raw_slot_ready(), "D1: and freezes nothing");

		raw_drive(2000, PAS_RAW_RING_LEN * 3U);
		check(pas_raw_ring_wraps() >= 2, "D1: wrapping is normal and is counted");
		check(pas_raw_capture_overrun() == 0,
			"D1: wrapping is NOT data loss - the overrun counter stays clean");
	}

	/* --- D2. freeze copies the pre-history, then the ISR fills the post ------------------ */
	{
		raw_begin();
		raw_drive(5000, PAS_RAW_FREEZE_PRE);      /* exactly a full pre-window */
		check(pas_raw_freeze(0x42), "D2: freeze accepts when the slot is free");
		check(!pas_raw_slot_ready(), "D2: not ready yet - the post-window is still to come");

		raw_drive(6000, PAS_RAW_FREEZE_POST);
		check(pas_raw_slot_ready(), "D2: ready once POST events have arrived");
		check(pas_raw_slot_count() == PAS_RAW_SLOT_LEN, "D2: the capture is PRE+POST long");
		check(pas_raw_slot_capture_id() == 0x42, "D2: it carries the id it was frozen with");
		check(pas_raw_slot_trigger_index() == PAS_RAW_FREEZE_PRE - 1U,
			"D2: the newest pre-event IS the trigger");
		check(pas_raw_capture_overrun() == 0, "D2: nothing was overwritten during the copy");

		pas_raw_event_t e;
		check(pas_raw_slot_get(0, &e), "D2: the oldest event reads back");
		check(e.capture_id == 0x42, "D2: every copied event is stamped with the capture id");
		check(!pas_raw_slot_get(PAS_RAW_SLOT_LEN, &e), "D2: reading past the end is refused");

		uint16_t skipped = pas_raw_freeze_skipped();
		check(!pas_raw_freeze(0x43), "D2: a second freeze is refused while the slot is occupied");
		check(pas_raw_freeze_skipped() == skipped + 1U, "D2: ...and counted");

		pas_raw_slot_release();
		check(!pas_raw_slot_ready(), "D2: release empties the slot");
		check(pas_raw_freeze(0x44), "D2: and freezing works again afterwards");
	}

	/* --- D3. the tick really is the ISR's own, and seq never skips ----------------------- */
	{
		raw_begin();
		raw_drive(9000, PAS_RAW_FREEZE_PRE);
		check(pas_raw_freeze(0x01), "D3: frozen");
		raw_drive(9000 + PAS_RAW_FREEZE_PRE, PAS_RAW_FREEZE_POST);
		check(pas_raw_slot_ready(), "D3: ready");

		pas_raw_event_t a, b;
		int monotonic = 1, seq_contiguous = 1;
		for (uint16_t i = 1; i < pas_raw_slot_count(); i++) {
			if (!pas_raw_slot_get((uint16_t)(i - 1U), &a) || !pas_raw_slot_get(i, &b)) break;
			if (b.tick <= a.tick) monotonic = 0;
			if ((uint16_t)(b.seq - a.seq) != 1U) seq_contiguous = 0;
		}
		check(monotonic, "D3: ticks increase across the whole capture");
		check(seq_contiguous, "D3: no gap in seq - nothing was lost between two kept events");
	}

	/* --- E. one physical event, two recorders, one key ----------------------------------- */
	{
		pas_trace_init();
		raw_begin();
		raw_drive(20000, PAS_RAW_FREEZE_PRE);

		/* Exactly what main.c does at the trigger site. */
		pas_trace_input_t trig = clean(1, 55);
		uint8_t id = pas_trace_transition(&trig);
		check(id != PAS_TRACE_NO_CAPTURE, "E: the decoder issued a capture id");
		check(pas_raw_freeze(id), "E: the raw recorder accepted the same id");

		for (unsigned i = 0; i < PAS_TRACE_POST - 1U; i++) {
			pas_trace_input_t in = clean(200, (uint16_t)(4000 + i));
			pas_trace_transition(&in);
		}
		raw_drive(21000, PAS_RAW_FREEZE_POST);

		check(pas_trace_slot_ready(0) && pas_raw_slot_ready(), "E: both captures completed");
		check(pas_trace_slot_capture_id(0) == pas_raw_slot_capture_id(),
			"E: decoder and raw views of the same event share one key");

		pas_raw_event_t ev;
		check(pas_raw_slot_get(0, &ev), "E: raw event readable");
		check(ev.capture_id == pas_trace_slot_capture_id(0),
			"E: ...and the key is on the events themselves, not just the header");
	}

	/* --- F1. every record carries the session that produced it ------------------------------ */
	{
		pas_trace_init();
		raw_begin();
		ride_episode_init();

		pas_trace_set_session_id(5);
		pas_raw_set_session_id(5);
		ride_episode_set_session_id(5);

		uint8_t id5 = drive_one_capture();
		check(pas_trace_slot_session_id(0) == 5, "F1: a trace capture is stamped with its session");
		check(pas_trace_count_session(5) == 1, "F1: ...and is found by that session");
		check(pas_trace_count_session(6) == 0, "F1: ...and NOT by another one");
		check(id5 != PAS_TRACE_NO_CAPTURE, "F1: the capture really was taken");

		/* A second capture, taken after the session rolled over. */
		pas_trace_set_session_id(6);
		drive_one_capture();
		check(pas_trace_count_session(5) == 1, "F1: the old session still owns exactly its own");
		check(pas_trace_count_session(6) == 1, "F1: the new session owns the new one");
		check(pas_trace_oldest_ready_slot_of(5) != pas_trace_oldest_ready_slot_of(6),
			"F1: the two live in different slots");

		/* The raw recorder stamps the same way. */
		raw_drive(30000, PAS_RAW_FREEZE_PRE);
		check(pas_raw_freeze(9), "F1: a raw capture was frozen");
		raw_drive(31000, PAS_RAW_FREEZE_POST);
		check(pas_raw_slot_session_id() == 5, "F1: the raw capture kept the session it was armed in");
		check(pas_raw_count_session(5) == 1, "F1: ...and is found by it");
		check(pas_raw_count_session(6) == 0, "F1: ...and not by the next one");
	}

	/* --- F2. a capture still open when the ride ends is SEALED, not left to span two rides -- */
	{
		pas_trace_init();
		raw_begin();
		pas_trace_set_session_id(11);
		pas_raw_set_session_id(11);

		/* Arm a decoder capture and leave it mid post-window. */
		pas_trace_input_t trig = clean(1, 77);
		uint8_t cap = pas_trace_transition(&trig);
		check(cap != PAS_TRACE_NO_CAPTURE, "F2: the capture armed");
		for (unsigned i = 0; i < 10; i++) {
			pas_trace_input_t in = clean(200, (uint16_t)(9000 + i));
			pas_trace_transition(&in);
		}
		check(!pas_trace_slot_ready(0), "F2: still collecting its post-window");

		/* Arm a raw capture too, also mid post-window. */
		raw_drive(40000, PAS_RAW_FREEZE_PRE);
		check(pas_raw_freeze(cap), "F2: the raw side armed on the same event");
		raw_drive(41000, 10);
		check(!pas_raw_slot_ready(), "F2: the raw capture is mid post-window as well");

		pas_trace_seal_open_captures();
		pas_raw_seal_open_capture();

		check(pas_trace_slot_ready(0), "F2: sealing makes the decoder capture readable");
		check(pas_trace_slot_partial(0), "F2: ...and marks it as cut short");
		check(pas_trace_slot_count(0) == 11U,
			"F2: it keeps exactly what was seen - trigger plus ten, not a padded 256");
		check(pas_trace_slot_session_id(0) == 11, "F2: it still belongs to the session that made it");

		check(pas_raw_slot_ready(), "F2: the raw capture is readable too");
		check(pas_raw_slot_partial(), "F2: ...and flagged partial");
		check(pas_raw_slot_count() == PAS_RAW_FREEZE_PRE + 10U, "F2: it keeps what it had collected");

		/*
		 * The point of all this: nothing armed in the old ride can still be filling when the next
		 * one starts. A fresh session must see no open capture at all.
		 */
		pas_trace_set_session_id(12);
		pas_raw_set_session_id(12);
		check(pas_trace_count_session(12) == 0, "F2: the new session starts with no captures");
		check(pas_raw_count_session(12) == 0, "F2: ...on either recorder");
	}

#ifdef PAS_RAW_COPY_HOOK
	/* --- D4. an ISR event landing mid-copy of the PRE half is kept, not lost --------------- */
	{
		pas_raw_init();
		uint8_t st = 0;
		pas_raw_isr_sample(st, 0);   /* prime: adopt state 0, no event recorded */

		/* Fill PRE fully: PAS_RAW_FREEZE_PRE alternating transitions. */
		uint32_t tick = 1000;
		for (unsigned i = 0; i < PAS_RAW_FREEZE_PRE; i++) {
			st ^= 1U;
			pas_raw_isr_sample(st, tick++);
		}

		hook_armed = true;
		hook_at_index = 64;                    /* mid-way through the PRE copy loop */
		hook_inject_state = (uint8_t)(st ^ 1U);
		hook_inject_tick = 555555U;             /* a distinctive tick, easy to find in the result */

		check(pas_raw_freeze(0x77), "D4: freeze accepts a full pre-history");
		check(!hook_armed, "D4: the hook actually fired during the copy - the scenario ran");
		check(!pas_raw_slot_ready(),
			"D4: not ready yet - only 1 of PAS_RAW_FREEZE_POST events collected so far");

		/* Finish the POST window: one event was already contributed by the mid-copy injection. */
		st = hook_inject_state;
		for (unsigned i = 0; i < PAS_RAW_FREEZE_POST - 1U; i++) {
			st ^= 1U;
			pas_raw_isr_sample(st, tick++);
		}
		check(pas_raw_slot_ready(), "D4: the capture completed despite the mid-copy injection");
		check(pas_raw_slot_count() == PAS_RAW_SLOT_LEN,
			"D4: full PRE+POST length - the mid-copy event was kept, not lost between the halves");
		check(pas_raw_capture_overrun() == 0,
			"D4: no genuine overrun - the ring itself was never lapped, only one extra event "
			"landed mid-copy");
		check(pas_raw_slot_trigger_index() == PAS_RAW_FREEZE_PRE - 1U,
			"D4: trigger index unaffected by the mid-copy injection");

		pas_raw_event_t injected;
		check(pas_raw_slot_get(PAS_RAW_FREEZE_PRE, &injected) && injected.tick == hook_inject_tick,
			"D4: the mid-copy event sits exactly at the PRE/POST boundary, not dropped between them");

		int seq_ok = 1;
		pas_raw_event_t a, b;
		for (uint16_t idx = 1; idx < pas_raw_slot_count(); idx++) {
			pas_raw_slot_get((uint16_t)(idx - 1U), &a);
			pas_raw_slot_get(idx, &b);
			if ((uint16_t)(b.seq - a.seq) != 1U) { seq_ok = 0; break; }
		}
		check(seq_ok,
			"D4: seq is continuous end to end, INCLUDING the PRE/POST boundary - the earlier "
			"design left exactly this gap, for exactly this kind of short bounce");
	}
#endif

	if (failures == 0) {
		printf("All FW-106 host checks passed.\n");
		return 0;
	}
	printf("\n%d FW-106 host check(s) FAILED.\n", failures);
	return 1;
}
