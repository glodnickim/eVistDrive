/*
 * FW-106 host harness for diag_session.c — the SHIPPED session, accounting and dump code.
 *
 * Build and run (see run-host-tests.ps1):
 *   gcc -std=c11 -Wall -Wextra -I../../inc -o fw106s fw106_session_host.c \
 *       ../../src/diag_session.c && ./fw106s
 *
 * WHY THIS EXISTS. This logic first shipped as static functions inside main.c, where nothing can
 * be executed by a test, and it carried four defects that its own output could not reveal: an
 * episode counted as sent twice, the record at risk during an interruption identified by a bool
 * that could not say WHICH record it was, one session's records able to appear inside another's
 * brackets, and trailer error bits that were global flags staying set until reboot. The module
 * takes its whole world through two op tables, so everything below drives the real code with a
 * fake CAN peripheral and fake record sources.
 *
 * What each block checks:
 *   1. NOMAILBOX -> PENDING -> OK: the cursor moves once, and only on OK
 *   2. FAILED a few times then OK: the same fragment is retried, nothing is skipped
 *   3. FAILED past the limit: THAT record is discarded and the next one still goes out
 *   4. a quiet spell cancelled at 2.5 s does not end the session
 *   5. 5 accepted, 3 sent, 2 discarded -> pending 0
 *   6. session A interrupted, then B: A is closed first and no B record sits inside A's brackets
 *   7. an interruption during the aggregate block discards NO record
 *   8. an interruption during a trace record discards the TRACE, not the episode
 *   9. one episode, no trace -> records_sent is exactly 1
 *  10. one episode plus a long trace -> the episode is still counted exactly once
 *  11. dump_complete=1 after an interrupted dump is later finished
 *  12. per-session metric deltas for two consecutive sessions
 *  13. the exact bracket order 0x1021D -> 0x10218 -> this session's records -> 0x1021E
 *  14. a full summary queue discards the orphaned session's records instead of stranding them
 *  15-17. DIAG_ERR_RAW_OVERRUN is a per-session delta, not a sticky global flag
 *  18. an "illegal" PAS edge mid-quiet-window cancels/re-anchors the candidate (not just a
 *      stop->ride transition) - see diag_session_note_activity()
 *  19. the aggregate block is frozen at candidate_end_tick: an interrupted-then-resumed dump
 *      still sends the ORIGINAL session's snapshot, not whatever is live by the time it resumes
 *  20. a whole lost session (full summary queue) is visible on the bus via DIAG_ERR_SESSION_LOST
 *      on the next trailer, as a delta, not silently invisible
 *  21. many calls at the SAME tick start at most one new frame - the regression test for the
 *      real defect: a ride log showed the CAN hardware confirming every frame of a dump while
 *      the USB sniffer capturing it only logged 766 of ~1389 frames sent, because nothing paced
 *      how often main.c's while(1) could start a new one
 *  22. a new frame does not start before DIAG_TX_FRAME_INTERVAL_TICKS have elapsed
 *  23. it CAN start once exactly that many ticks have elapsed - not one short
 *  24. the pacing check survives a uint32_t wraparound of now_tick
 *  25. NOMAILBOX does not arm the pacing clock and does not move the cursor or lose the fragment
 *  26. a FAILED retry offers the identical fragment again and is itself paced like any other
 *      new frame
 *  27. the rider setting off again ends the dump at once even while the pacing timer is still
 *      blocking the next frame
 *  28. a large, realistic dump (15 episodes, a full 256-sample trace, a full 256-sample raw
 *      capture - the shape of the ride log this card investigates) still ends in a correct
 *      trailer under pacing, with records_sent and dump_complete=1 correct
 *  29-31. FW-110: missed_control_events source-counter saturation. main.c's own global changed
 *      from uint16_t to uint32_t (see src/main.c and the FW-110 report) because a session that
 *      lost 12,543 ticks published missed_events=0 - the uint16_t source had already saturated
 *      at 65535 well before that ride even started, so every session's delta read as zero no
 *      matter how many events were actually lost. diag_session.c's own internal delta math was
 *      ALREADY uint32_t throughout before this card (confirmed by audit) - these three checks
 *      exercise that this module's side of the fix (now including the explicit wire clamp added
 *      to the PH_METRICS case) behaves correctly against the exact numbers the bug involved:
 *   29. a running total already past 65535 BEFORE a session starts still produces a correct,
 *       small delta for that session - not zero
 *   30. exactly 0 new events this session publishes exactly 0, even from a huge running total
 *   31. a single session's OWN delta exceeding 65535 clamps to the 0xFFFF ceiling on the wire,
 *       not a silently truncated low 16 bits (70000 would truncate to 4464, a plausible-looking
 *       but wrong number - very different from failing loudly)
 *   32-35. FW-110: allow_new_tx, diag_session_dump_step()'s new second parameter - proof that
 *       critical HMI/Canable frames get real priority over diagnostics, not just a favourable
 *       call order in main.c:
 *   32. allow_new_tx=false blocks a genuinely NEW frame before it ever reaches transmit() -
 *       zero hardware calls, not just zero completions
 *   33. that gate holds across many ticks and many crossed pacing intervals - pacing alone
 *       cannot authorise a new attempt without allow_new_tx too
 *   34. once the gate opens, the blocked frame resumes and completes correctly - the dump does
 *       NOT restart from the header
 *   35. the rider setting off again still aborts the dump immediately even while
 *       allow_new_tx=false - the abort path is never gated, only new-frame starts are
 */

#include <stdio.h>
#include <string.h>

#include "diag_session.h"

static int failures;

static void check(int ok, const char *label)
{
	if (!ok) {
		failures++;
		printf("  FAIL  %s\n", label);
	}
}

/* --- fake CAN peripheral ------------------------------------------------------------------- */

#define LOG_MAX 4096
static uint32_t sent_log[LOG_MAX];
static uint8_t  sent_payload[LOG_MAX][8];
static uint16_t sent_n;
static uint8_t  inflight_data[8];

static uint8_t  fake_nomailbox_for;   /* refuse this many transmit() calls */
static uint8_t  fake_pending_for;     /* answer PENDING this many times per frame */
static uint8_t  fake_fail_for;        /* answer FAILED this many times, then OK */
static uint32_t inflight_efid;
static uint8_t  pending_left;
static bool     busy;
/* FW-110: exact call counts, for allow_new_tx tests that must prove a NEW attempt never reached
 * the peripheral at all - not just that nothing completed (which busy/sent_n alone cannot tell
 * apart from "blocked before even trying"). */
static uint32_t fake_transmit_calls;
static uint32_t fake_state_calls;

static void log_inflight(void)
{
	if (sent_n < LOG_MAX) {
		sent_log[sent_n] = inflight_efid;
		memcpy(sent_payload[sent_n], inflight_data, 8);
		sent_n++;
	}
}

static uint8_t fake_transmit(uint32_t efid, const uint8_t *data)
{
	fake_transmit_calls++;
	if (fake_nomailbox_for > 0) { fake_nomailbox_for--; return DIAG_CAN_NOMAILBOX; }
	if (busy && pending_left > 0) return DIAG_CAN_NOMAILBOX;  /* genuinely still on the wire */
	if (busy) {
		/*
		 * A frame the module walked away from - it aborts the dump the moment the bike moves and
		 * never asks about that mailbox again. The peripheral does not care: it finishes the
		 * frame and frees the mailbox by itself, which is what this models. A fake that held the
		 * mailbox for ever would wedge every later dump and would be modelling nothing real.
		 */
		log_inflight();
	}
	inflight_efid = efid;
	memcpy(inflight_data, data, 8);
	pending_left = fake_pending_for;
	busy = true;
	return 1U;   /* mailbox 1 */
}

static uint8_t fake_state(uint8_t mailbox)
{
	(void)mailbox;
	fake_state_calls++;
	if (!busy) return DIAG_CAN_FAILED;
	if (pending_left > 0) { pending_left--; return DIAG_CAN_PENDING; }
	if (fake_fail_for > 0) { fake_fail_for--; busy = false; return DIAG_CAN_FAILED; }
	busy = false;
	log_inflight();
	return DIAG_CAN_OK;
}

static const diag_can_ops_t fake_can = { fake_transmit, fake_state };

/* --- fake record sources ------------------------------------------------------------------- */

typedef struct {
	uint8_t  session[16];
	uint16_t frags[16];      /* how many fragments each record has */
	uint8_t  n;              /* records held */
	uint32_t accepted, refused;
	uint32_t base_efid;
} fake_src_t;

static fake_src_t fsrc[DIAG_SRC_COUNT];
static uint16_t fake_agg_frames = 3;
static uint16_t seal_calls;

static uint16_t src_count(uint8_t s, uint8_t sid)
{
	uint16_t n = 0;
	for (uint8_t i = 0; i < fsrc[s].n; i++) if (fsrc[s].session[i] == sid) n++;
	return n;
}

static int src_first(uint8_t s, uint8_t sid)
{
	for (uint8_t i = 0; i < fsrc[s].n; i++) if (fsrc[s].session[i] == sid) return i;
	return -1;
}

static bool src_frame(uint8_t s, uint8_t sid, uint16_t frag, uint32_t *efid, uint8_t *data, bool *last)
{
	int i = src_first(s, sid);
	if (i < 0) return false;
	memset(data, 0, 8);
	data[0] = sid;
	data[1] = (uint8_t)frag;
	*efid = fsrc[s].base_efid + frag;
	*last = (frag + 1U) >= fsrc[s].frags[i];
	return true;
}

static void src_release(uint8_t s, uint8_t sid)
{
	int i = src_first(s, sid);
	if (i < 0) return;
	for (uint8_t k = (uint8_t)i; k + 1U < fsrc[s].n; k++) {
		fsrc[s].session[k] = fsrc[s].session[k + 1U];
		fsrc[s].frags[k] = fsrc[s].frags[k + 1U];
	}
	fsrc[s].n--;
}

#define SRC_GLUE(NAME, IDX) \
	static uint16_t NAME##_count(uint8_t sid){ return src_count(IDX, sid); } \
	static bool NAME##_frame(uint8_t sid, uint16_t f, uint32_t *e, uint8_t *d, bool *l){ \
		return src_frame(IDX, sid, f, e, d, l); } \
	static void NAME##_release(uint8_t sid, bool sent){ (void)sent; src_release(IDX, sid); } \
	static uint32_t NAME##_accepted(void){ return fsrc[IDX].accepted; } \
	static uint32_t NAME##_refused(void){ return fsrc[IDX].refused; }

SRC_GLUE(ep, DIAG_SRC_EPISODE)
SRC_GLUE(tr, DIAG_SRC_TRACE)
SRC_GLUE(rw, DIAG_SRC_RAW)

/* FW-106: a stand-in for whatever live MS/torque_input state real main.c would embed - lets a
 * test tell whether an aggregate frame carries a FROZEN value or was re-read live at send time. */
static uint8_t fake_agg_value;

static bool agg_frame(uint16_t index, uint32_t *efid, uint8_t *data)
{
	if (index >= fake_agg_frames) return false;
	memset(data, 0, 8);
	data[0] = fake_agg_value;
	*efid = 0x00010203U + index;
	return true;
}

static void seal_open(void) { seal_calls++; }

/* FW-106: monotonic, controllable from a test - see checks 14-16 below (per-session overrun). */
static uint32_t fake_raw_overrun;
static uint32_t raw_overrun_total(void) { return fake_raw_overrun; }

static const diag_ops_t fake_ops = {
	agg_frame, seal_open, raw_overrun_total,
	{
		{ ep_count, ep_frame, ep_release, ep_accepted, ep_refused },
		{ tr_count, tr_frame, tr_release, tr_accepted, tr_refused },
		{ rw_count, rw_frame, rw_release, rw_accepted, rw_refused }
	}
};

/* --- helpers -------------------------------------------------------------------------------- */

static uint32_t now;

static void reset_all(void)
{
	memset(fsrc, 0, sizeof(fsrc));
	fsrc[DIAG_SRC_EPISODE].base_efid = 0x00010210U;
	fsrc[DIAG_SRC_TRACE].base_efid   = 0x00010215U;
	fsrc[DIAG_SRC_RAW].base_efid     = 0x0001021BU;
	sent_n = 0;
	fake_nomailbox_for = fake_pending_for = fake_fail_for = 0;
	busy = false; pending_left = 0;
	fake_transmit_calls = 0; fake_state_calls = 0;
	seal_calls = 0;
	fake_agg_frames = 3;
	fake_agg_value = 0;
	fake_raw_overrun = 0;
	now = 0;
	diag_session_init(&fake_can, &fake_ops);
}

/* Add a record of `frags` fragments belonging to `sid`, as the source accepting one. */
static void add_record(diag_source_t s, uint8_t sid, uint16_t frags)
{
	fsrc[s].session[fsrc[s].n] = sid;
	fsrc[s].frags[fsrc[s].n] = frags;
	fsrc[s].n++;
	fsrc[s].accepted++;
}

static void ride(uint32_t ticks, uint32_t missed, uint32_t events, uint16_t burst)
{
	for (uint32_t i = 0; i < ticks; i++) {
		diag_session_tick(now++, true, missed, events, (i == 0) ? burst : 0U);
	}
}

static void stand(uint32_t ticks, uint32_t missed, uint32_t events)
{
	for (uint32_t i = 0; i < ticks; i++) diag_session_tick(now++, false, missed, events, 0U);
}

/*
 * Run the dump until it goes quiet or `limit` steps elapse. Each step also advances `now` by
 * exactly one pacing interval - the real main loop calls diag_session_dump_step() far more
 * often than that per interval (that gap is the whole point of FW-106's pacing fix), so
 * advancing by the interval here keeps every existing `pump(N)` call's N meaning what it always
 * meant: "up to N frame-worth steps", not "N calls, most of them paced into doing nothing".
 * The dedicated pacing tests below call diag_session_dump_step() directly instead, for the
 * tick-by-tick control they need.
 */
static void pump(uint32_t limit)
{
	for (uint32_t i = 0; i < limit; i++) {
		diag_session_dump_step(now, true);
		now += DIAG_TX_FRAME_INTERVAL_TICKS;
	}
}

/*
 * FW-106: one tick containing an "illegal" PAS edge - real main.c would call
 * diag_session_note_activity() at the quadrature-transition site BEFORE this tick's
 * diag_session_tick(), and an illegal two-bit jump takes qd[]=0 in the decoder, so it never
 * touches fwd_run/pas_idle_ticks/Backwards_counter - "moving" for THIS tick genuinely reads
 * false even though real activity was just reported. Models that exact shape.
 */
static void illegal_pas_edge(uint32_t missed, uint32_t events)
{
	diag_session_note_activity(now, missed, events);
	diag_session_tick(now, false, missed, events, 0U);
	now++;
}

/*
 * Step until a record of the given kind is the one in flight. The dump sends a header, the
 * metrics and the aggregate block before it touches a record, and each frame takes two steps
 * (offer, then confirm), so counting steps by hand is brittle - this waits for the state that
 * actually matters.
 */
static bool pump_until_kind(diag_record_kind_t k, uint32_t limit)
{
	for (uint32_t i = 0; i < limit; i++) {
		if (diag_dump_record_kind() == k) return true;
		diag_session_dump_step(now, true);
		now += DIAG_TX_FRAME_INTERVAL_TICKS;
	}
	return diag_dump_record_kind() == k;
}

/* Exhaust the retries on whatever fragment is in flight, so its whole record is given up on. */
static void kill_current_record(void)
{
	fake_fail_for = DIAG_TX_MAX_RETRY + 1U;
	for (uint32_t i = 0; i < 200U && fake_fail_for > 0U; i++) {
		diag_session_dump_step(now, true);
		now += DIAG_TX_FRAME_INTERVAL_TICKS;
	}
	diag_session_dump_step(now, true);
	now += DIAG_TX_FRAME_INTERVAL_TICKS;
}

static int count_id(uint32_t efid)
{
	int n = 0;
	for (uint16_t i = 0; i < sent_n; i++) if (sent_log[i] == efid) n++;
	return n;
}

static int index_of(uint32_t efid)
{
	for (uint16_t i = 0; i < sent_n; i++) if (sent_log[i] == efid) return (int)i;
	return -1;
}

/* The n-th (0-based) frame with this id, or -1. */
static int nth_index_of(uint32_t efid, int n)
{
	int seen = 0;
	for (uint16_t i = 0; i < sent_n; i++) {
		if (sent_log[i] != efid) continue;
		if (seen == n) return (int)i;
		seen++;
	}
	return -1;
}

static uint32_t be32(const uint8_t *d) {
	return ((uint32_t)d[0]<<24)|((uint32_t)d[1]<<16)|((uint32_t)d[2]<<8)|d[3];
}
static uint16_t be16(const uint8_t *d) { return (uint16_t)(((uint16_t)d[0]<<8)|d[1]); }

int main(void)
{
	printf("FW-106 session/dump, against the shipped diag_session.c\n");
	printf("  quiet %u ticks, summaries %u, max retry %u, tx pacing %u ms (%u ticks)\n",
		(unsigned)DIAG_SESSION_QUIET_TICKS, (unsigned)DIAG_SESSION_SUMMARIES,
		(unsigned)DIAG_TX_MAX_RETRY, (unsigned)DIAG_TX_FRAME_INTERVAL_MS,
		(unsigned)DIAG_TX_FRAME_INTERVAL_TICKS);

	/* --- 1. NOMAILBOX -> PENDING -> OK ---------------------------------------------------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		add_record(DIAG_SRC_EPISODE, diag_session_current_id(), 2);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		check(diag_summaries_pending() == 1, "1: the session closed and is waiting to be dumped");

		fake_nomailbox_for = 3;
		fake_pending_for = 2;
		pump(200);
		check(count_id(0x0001021DU) == 1, "1: the header went out exactly once");
		check(count_id(0x0001021EU) == 1, "1: ...and so did the trailer");
		check(diag_counter_sent(DIAG_SRC_EPISODE) == 1, "1: the episode counted as sent once");
		check(diag_counter_pending(DIAG_SRC_EPISODE) == 0, "1: nothing is left pending");
	}

	/* --- 2. FAILED a few times, then OK ---------------------------------------------------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		add_record(DIAG_SRC_EPISODE, diag_session_current_id(), 2);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		fake_fail_for = 3;
		pump(300);
		check(diag_counter_sent(DIAG_SRC_EPISODE) == 1, "2: retries do not lose the record");
		check(diag_counter_discarded(DIAG_SRC_EPISODE) == 0, "2: ...and do not discard it either");
		check(count_id(0x00010210U) == 1, "2: the retried fragment was sent exactly once");
		check(count_id(0x00010211U) == 1, "2: the fragment after it went too - nothing skipped");
	}

	/* --- 3. FAILED past the limit: this record dies, the next still goes ------------------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		uint8_t sid = diag_session_current_id();
		add_record(DIAG_SRC_EPISODE, sid, 2);
		add_record(DIAG_SRC_EPISODE, sid, 2);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		/*
		 * Wait until a RECORD is what is being sent. Injecting the failures earlier would have
		 * hit the session header instead, which is deliberately not allowed to cost a record.
		 */
		check(pump_until_kind(DIAG_REC_EPISODE, 500), "3: the dump reached the first record");
		kill_current_record();
		pump(500);
		check(diag_counter_discarded(DIAG_SRC_EPISODE) == 1, "3: exactly one record was given up on");
		check(diag_counter_sent(DIAG_SRC_EPISODE) == 1, "3: the NEXT record still went out");
		check(diag_counter_pending(DIAG_SRC_EPISODE) == 0, "3: pending is clean: 2 - 1 - 1");
		check(count_id(0x0001021EU) == 1, "3: the dump still reached its trailer");
	}

	/* --- 4. a quiet spell cancelled before 3 s does not end the session -------------------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		uint8_t sid_before = diag_session_current_id();
		stand(DIAG_SESSION_QUIET_TICKS - (DIAG_SESSION_QUIET_TICKS / 6U), 0, 0);  /* ~2.5 s */
		check(diag_summaries_pending() == 0, "4: 2.5 s of quiet is not yet a finished session");
		check(diag_session_is_active(), "4: the session is still running");

		ride(5, 0, 0, 0);   /* movement cancels the candidate */
		check(diag_session_current_id() == sid_before, "4: no new session was started by that");
		check(diag_summaries_pending() == 0, "4: the cancelled candidate published nothing");

		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		check(diag_summaries_pending() == 1, "4: a full quiet spell afterwards does close it");
	}

	/* --- 5. 5 accepted, 3 sent, 2 discarded -> pending 0 ----------------------------------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		uint8_t sid = diag_session_current_id();
		for (int i = 0; i < 5; i++) add_record(DIAG_SRC_EPISODE, sid, 2);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		/* Two records die on the fragment in flight; the other three go out normally. */
		check(pump_until_kind(DIAG_REC_EPISODE, 500), "5: the dump reached the records");
		kill_current_record();
		check(pump_until_kind(DIAG_REC_EPISODE, 500), "5: and the next record");
		kill_current_record();
		pump(2000);

		check(diag_counter_enqueued(DIAG_SRC_EPISODE) == 5, "5: five records were accepted");
		check(diag_counter_sent(DIAG_SRC_EPISODE) == 3, "5: three were delivered");
		check(diag_counter_discarded(DIAG_SRC_EPISODE) == 2, "5: two were abandoned");
		check(diag_counter_pending(DIAG_SRC_EPISODE) == 0,
			"5: pending = enqueued - sent - discarded = 0");
	}

	/* --- 6. session A interrupted, then B: A closes first, no B record inside A ------------ */
	{
		reset_all();
		ride(10, 0, 0, 0);
		uint8_t a = diag_session_current_id();
		add_record(DIAG_SRC_EPISODE, a, 6);
		add_record(DIAG_SRC_EPISODE, a, 6);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		pump(6);                     /* start A's dump, then interrupt it part-way */
		ride(10, 0, 0, 0);
		uint8_t b = diag_session_current_id();
		check(b != a, "6: setting off again started a new session");
		add_record(DIAG_SRC_EPISODE, b, 6);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		check(diag_summaries_pending() == 2, "6: both sessions are queued for dumping");

		sent_n = 0;                  /* watch only what happens from here */
		pump(4000);

		/*
		 * A's dump restarts from its own header after the interruption, so seeing 0x1021D again
		 * is correct. What must NOT happen is any frame belonging to session B turning up before
		 * A's trailer. Every header and every record frame carries its session id (the fake puts
		 * it in data[0], as the real record sources do), so this can be checked directly.
		 */
		int a_close = -1;
		int b_frame_before_a_close = -1;
		for (uint16_t i = 0; i < sent_n; i++) {
			if (sent_log[i] == 0x0001021EU) { a_close = (int)i; break; }
			bool is_header = (sent_log[i] == 0x0001021DU);
			uint8_t sid_in_frame = is_header ? sent_payload[i][1] : sent_payload[i][0];
			bool is_record = (sent_log[i] >= 0x00010210U && sent_log[i] <= 0x00010214U);
			if ((is_header || is_record) && sid_in_frame == b && b_frame_before_a_close < 0) {
				b_frame_before_a_close = (int)i;
			}
		}
		check(a_close >= 0, "6: session A was closed by a trailer");
		check(b_frame_before_a_close < 0,
			"6: no frame belonging to session B appeared inside session A's brackets");
		if (a_close >= 0) {
			check(sent_payload[a_close][1] == a, "6: the first trailer closes A, not B");
		}
		check(count_id(0x0001021EU) == 2, "6: both sessions were closed, in turn");
		check(diag_counter_sent(DIAG_SRC_EPISODE) + diag_counter_discarded(DIAG_SRC_EPISODE) == 3,
			"6: all three records ended exactly once");
		check(diag_counter_pending(DIAG_SRC_EPISODE) == 0, "6: nothing left over");
	}

	/* --- 7. an interruption during the aggregate block discards NO record ------------------ */
	{
		reset_all();
		fake_agg_frames = 20;
		ride(10, 0, 0, 0);
		add_record(DIAG_SRC_EPISODE, diag_session_current_id(), 6);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		pump(8);   /* header, metrics, into the aggregate block - no record started yet */
		check(diag_dump_record_kind() == DIAG_REC_NONE,
			"7: no record is in flight during the aggregate block");
		check(count_id(0x00010203U) >= 1, "7: the aggregate block really is under way");
		ride(5, 0, 0, 0);
		diag_session_dump_step(now, true);
		check(diag_counter_discarded(DIAG_SRC_EPISODE) == 0,
			"7: interrupting the aggregate block discards no episode");
		check(fsrc[DIAG_SRC_EPISODE].n == 1, "7: the episode is still queued");
	}

	/* --- 8. an interruption during a TRACE discards the trace, not the episode ------------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		uint8_t sid = diag_session_current_id();
		add_record(DIAG_SRC_EPISODE, sid, 6);
		add_record(DIAG_SRC_TRACE, sid, 40);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		check(pump_until_kind(DIAG_REC_TRACE, 500),
			"8: a trace record is the one in flight");
		ride(5, 0, 0, 0);
		diag_session_dump_step(now, true);
		check(diag_counter_discarded(DIAG_SRC_TRACE) == 1, "8: the TRACE was discarded");
		check(diag_counter_discarded(DIAG_SRC_EPISODE) == 0, "8: the episode was NOT");
		check(diag_counter_sent(DIAG_SRC_EPISODE) == 1, "8: the episode had already been sent");
	}

	/* --- 9. one episode, no trace -> records_sent is exactly 1 ----------------------------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		add_record(DIAG_SRC_EPISODE, diag_session_current_id(), 6);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		pump(400);
		int t = index_of(0x0001021EU);
		check(t >= 0, "9: the trailer went out");
		check(diag_counter_sent(DIAG_SRC_EPISODE) == 1, "9: the episode counted exactly once");
		if (t >= 0) {
			check(be16(&sent_payload[t][2]) == 1, "9: the trailer reports records_sent = 1");
			check(sent_payload[t][4] == 0, "9: ...nothing discarded");
			check(sent_payload[t][7] == 1, "9: ...and the dump is marked complete");
		}
	}

	/* --- 10. one episode plus a long trace: the episode is still counted once -------------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		uint8_t sid = diag_session_current_id();
		add_record(DIAG_SRC_EPISODE, sid, 6);
		add_record(DIAG_SRC_TRACE, sid, 513);   /* a full 256-sample capture */
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		pump(4000);
		check(diag_counter_sent(DIAG_SRC_EPISODE) == 1,
			"10: a long trace does not inflate the episode count");
		check(diag_counter_sent(DIAG_SRC_TRACE) == 1, "10: the trace counted once");
		check(diag_counter_pending(DIAG_SRC_EPISODE) == 0 &&
		      diag_counter_pending(DIAG_SRC_TRACE) == 0, "10: everything resolved");
	}

	/* --- 11. dump_complete=1 once an interrupted dump is later finished -------------------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		uint8_t sid = diag_session_current_id();
		add_record(DIAG_SRC_EPISODE, sid, 6);
		add_record(DIAG_SRC_EPISODE, sid, 6);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		pump(6);
		ride(10, 0, 0, 0);            /* interrupt it */
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		pump(4000);                   /* let everything finish */

		/*
		 * The trailer's last byte is dump_complete. Having been interrupted earlier says nothing
		 * about whether the records were eventually resolved, so it must not veto completeness.
		 */
		int t = index_of(0x0001021EU);
		check(t >= 0, "11: the interrupted session was eventually closed");
		check(diag_counter_pending(DIAG_SRC_EPISODE) == 0, "11: every record ended exactly once");
		if (t >= 0) {
			check(sent_payload[t][7] == 1,
				"11: dump_complete=1 - being interrupted earlier does not veto completeness");
			check((sent_payload[t][6] & DIAG_ERR_INTERRUPTED) != 0,
				"11: ...while the interruption itself is still reported, on its own bit");
			check(be16(&sent_payload[t][2]) + sent_payload[t][4] == 2,
				"11: both records are accounted for, as sent or discarded");
		}
	}

	/* --- 12. per-session metric deltas for two consecutive sessions ------------------------ */
	{
		reset_all();
		/*
		 * Session A runs while the running totals climb to 100/10, with a 40-tick burst.
		 * Session B then starts with those totals ALREADY at 100/10 and adds 50/5 of its own,
		 * with a smaller worst burst - which is the case that catches a global maximum being
		 * published as if it were the session's.
		 */
		ride(5, 0, 0, 0);
		ride(5, 100, 10, 40);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 100, 10);
		ride(2, 100, 10, 0);                      /* B starts with the totals where A left them */
		ride(5, 150, 15, 12);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 150, 15);
		check(diag_summaries_pending() == 2, "12: two sessions closed");

		pump(4000);

		/*
		 * Both metric frames are 0x10218. The second one is session B's and must read 50/5/12 -
		 * NOT 150/15/40. The worst burst is the sharp end: a global maximum cannot be recovered
		 * by subtracting anchors, so 12 here proves it is tracked per session.
		 */
		check(count_id(0x00010218U) == 2, "12: one metric frame per session");
		int m0 = nth_index_of(0x00010218U, 0);
		int m1 = nth_index_of(0x00010218U, 1);
		check(m0 >= 0 && m1 >= 0, "12: both metric frames were captured");
		if (m0 >= 0 && m1 >= 0) {
			check(be32(&sent_payload[m0][0]) == 100, "12: session A reports its own 100 ticks");
			check(be16(&sent_payload[m0][4]) == 10,  "12: ...and its own 10 events");
			check(be16(&sent_payload[m0][6]) == 40,  "12: ...and its own worst burst of 40");
			check(be32(&sent_payload[m1][0]) == 50,
				"12: session B reports 50, NOT the running total of 150");
			check(be16(&sent_payload[m1][4]) == 5,
				"12: session B reports 5 events, not 15");
			check(be16(&sent_payload[m1][6]) == 12,
				"12: session B's worst burst is 12 - a global maximum of 40 cannot be subtracted");
		}
	}

	/* --- 13. the exact bracket order ------------------------------------------------------- */
	{
		reset_all();
		fake_agg_frames = 2;
		ride(10, 0, 0, 0);
		uint8_t sid = diag_session_current_id();
		add_record(DIAG_SRC_EPISODE, sid, 2);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		pump(400);

		check(sent_n >= 6, "13: the whole bracket went out");
		check(sent_log[0] == 0x0001021DU, "13: the session header comes first");
		check(sent_log[1] == 0x00010218U, "13: the session metrics come straight after it");
		check(sent_log[2] == 0x00010203U && sent_log[3] == 0x00010204U,
			"13: then the aggregate block, in order");
		check(sent_log[4] == 0x00010210U && sent_log[5] == 0x00010211U,
			"13: then this session's records");
		check(sent_log[sent_n - 1U] == 0x0001021EU, "13: and the trailer closes it");
		check(seal_calls >= 1, "13: open captures were sealed when the session closed");
	}

	/* --- 14. a full summary queue discards the orphaned session's records, doesn't strand them */
	{
		reset_all();
		/* Fill all four summary slots without draining any of them. */
		for (int i = 0; i < (int)DIAG_SESSION_SUMMARIES; i++) {
			ride(10, 0, 0, 0);
			stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		}
		check(diag_summaries_pending() == (int)DIAG_SESSION_SUMMARIES,
			"14: all four summary slots are occupied");

		/* A fifth session, with a record from every source. */
		ride(10, 0, 0, 0);
		uint8_t sid5 = diag_session_current_id();
		add_record(DIAG_SRC_EPISODE, sid5, 6);
		add_record(DIAG_SRC_TRACE, sid5, 40);
		add_record(DIAG_SRC_RAW, sid5, 20);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		check(diag_summary_rejected() == 1, "14: the fifth summary was refused, counted once");
		check(diag_summaries_pending() == (int)DIAG_SESSION_SUMMARIES,
			"14: the queue still holds only the four older summaries - untouched");

		check(diag_counter_discarded(DIAG_SRC_EPISODE) == 1,
			"14: the episode of the orphaned session was discarded immediately");
		check(diag_counter_discarded(DIAG_SRC_TRACE) == 1,
			"14: ...and the trace capture");
		check(diag_counter_discarded(DIAG_SRC_RAW) == 1,
			"14: ...and the raw capture");
		check(diag_counter_sent(DIAG_SRC_EPISODE) == 0 && diag_counter_sent(DIAG_SRC_TRACE) == 0 &&
			diag_counter_sent(DIAG_SRC_RAW) == 0,
			"14: none of them were counted as sent - they were never on the wire");
		check(diag_counter_rejected(DIAG_SRC_EPISODE) == 0 && diag_counter_rejected(DIAG_SRC_TRACE) == 0 &&
			diag_counter_rejected(DIAG_SRC_RAW) == 0,
			"14: and none inflate the per-source door-refusal counters - they WERE accepted, "
			"just discarded once their session's summary could not be");
		check(fsrc[DIAG_SRC_EPISODE].n == 0 && fsrc[DIAG_SRC_TRACE].n == 0 && fsrc[DIAG_SRC_RAW].n == 0,
			"14: no record of the orphaned session remains queued in any source");

		/* Drain the four real summaries and confirm the books balance for every source. */
		pump(4000);
		check(diag_summaries_pending() == 0, "14: the four real sessions all drained");
		check(diag_counter_pending(DIAG_SRC_EPISODE) == 0 && diag_counter_pending(DIAG_SRC_TRACE) == 0 &&
			diag_counter_pending(DIAG_SRC_RAW) == 0,
			"14: pending is zero for every source - enqueued = sent + discarded, no leftovers");

		/* And session 5 never appears inside anyone else's bracket. */
		bool sid5_seen = false;
		for (uint16_t i = 0; i < sent_n; i++) {
			bool is_header_or_trailer = (sent_log[i] == 0x0001021DU || sent_log[i] == 0x0001021EU);
			uint8_t sid_field = is_header_or_trailer ? sent_payload[i][1] : sent_payload[i][0];
			if (sid_field == sid5) sid5_seen = true;
		}
		check(!sid5_seen, "14: session 5 never appears anywhere in the log - it was discarded, "
			"not smuggled into another session's brackets");
	}

	/* --- 15/16/17. DIAG_ERR_RAW_OVERRUN is per-session, not a sticky global flag ------------ */
	{
		/* 15: an overrun during session A's ride sets the bit in A's trailer. */
		reset_all();
		ride(10, 0, 0, 0);
		fake_raw_overrun += 3;      /* the overrun happens while riding */
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		pump(2000);
		int tA = index_of(0x0001021EU);
		check(tA >= 0, "15: session A's trailer went out");
		if (tA >= 0) {
			check((sent_payload[tA][6] & DIAG_ERR_RAW_OVERRUN) != 0,
				"15: session A's trailer carries DIAG_ERR_RAW_OVERRUN");
		}

		/* 16: session B, no NEW overrun - must NOT inherit A's bit. */
		ride(10, 0, 0, 0);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		pump(2000);
		int tB = nth_index_of(0x0001021EU, 1);
		check(tB >= 0, "16: session B's trailer went out");
		if (tB >= 0) {
			check((sent_payload[tB][6] & DIAG_ERR_RAW_OVERRUN) == 0,
				"16: session B does NOT inherit A's overrun - the bit is a delta, not a sticky flag");
		}

		/*
		 * 17: an overrun landing DURING the 3 s quiet-confirmation window, after the candidate
		 * was already taken, must not attach to the session that is in the middle of closing.
		 */
		ride(10, 0, 0, 0);
		uint32_t half = DIAG_SESSION_QUIET_TICKS / 2U;
		stand(half, 0, 0);                       /* enters quiet; candidate snapshotted here */
		fake_raw_overrun += 5;                    /* overrun while waiting to confirm the stop */
		stand(DIAG_SESSION_QUIET_TICKS - half + 2U, 0, 0);   /* finish the quiet window */
		pump(2000);
		int tC = nth_index_of(0x0001021EU, 2);
		check(tC >= 0, "17: session C's trailer went out");
		if (tC >= 0) {
			check((sent_payload[tC][6] & DIAG_ERR_RAW_OVERRUN) == 0,
				"17: an overrun during the 3 s confirmation window does not contaminate the ride "
				"that just ended - it was already snapshotted before the overrun happened");
		}
	}

	/* --- 18. an illegal PAS edge mid-window cancels/re-anchors the quiet candidate ---------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		uint32_t half = DIAG_SESSION_QUIET_TICKS / 2U;
		stand(half, 0, 0);          /* quiet begins here; the ORIGINAL candidate is taken now */
		check(diag_summaries_pending() == 0, "18: not closed yet - only half the window elapsed");

		/* The "illegal" edge: reported activity, but moving reads false for this very tick -
		 * exactly the shape of a two-bit quadrature jump, which qd[]=0 in the real decoder and
		 * so never touches fwd_run/pas_idle_ticks/Backwards_counter. */
		illegal_pas_edge(0, 0);

		/* Enough MORE quiet to complete the ORIGINAL 3 s window, had it not been cancelled. */
		stand(half, 0, 0);
		check(diag_summaries_pending() == 0,
			"18: the session must NOT have closed 3 s from the FIRST quiet tick - the mid-window "
			"edge had to cancel/re-anchor the candidate, or this quiet run would read as unbroken");

		/* A full, genuinely uninterrupted window from the re-anchor point DOES eventually close it. */
		stand(DIAG_SESSION_QUIET_TICKS + 10U, 0, 0);
		check(diag_summaries_pending() == 1,
			"18: ...and it does close once quiet truly holds for the full window afterwards");
	}

	/* --- 19. aggregate frames are frozen at candidate_end_tick, not re-read live at dump time - */
	{
		reset_all();
		fake_agg_frames = 1;

		fake_agg_value = 0xAAU;
		ride(10, 0, 0, 0);
		uint8_t sidA = diag_session_current_id();
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);   /* A's candidate freezes 0xAA here */
		check(diag_summaries_pending() == 1, "19: session A queued");

		/* Start draining A, but interrupt well before it could reach PH_AGGREGATE. */
		pump(3);
		ride(5, 0, 0, 0);
		uint8_t sidB = diag_session_current_id();
		check(sidB != sidA, "19: setting off again started session B");

		fake_agg_value = 0xBBU;                        /* B's live state differs from A's */
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);    /* B's OWN candidate freezes 0xBB */
		check(diag_summaries_pending() == 2, "19: both A (resumable) and B are queued");

		sent_n = 0;   /* watch only what happens from here */
		pump(20000);  /* A resumes and finishes, THEN B runs */

		int firstAgg = -1, secondAgg = -1;
		for (uint16_t i = 0; i < sent_n; i++) {
			if (sent_log[i] == 0x00010203U) {
				if (firstAgg < 0) firstAgg = i; else if (secondAgg < 0) secondAgg = i;
			}
		}
		check(firstAgg >= 0, "19: A's (resumed) dump sent its aggregate frame");
		check(secondAgg >= 0, "19: B's dump sent its own aggregate frame afterwards");
		if (firstAgg >= 0) {
			check(sent_payload[firstAgg][0] == 0xAAU,
				"19: A's resumed dump still carries A's FROZEN snapshot (0xAA) - not B's live "
				"0xBB, which is what a live re-read at dump time would have sent");
		}
		if (secondAgg >= 0) {
			check(sent_payload[secondAgg][0] == 0xBBU,
				"19: B's own aggregate frame correctly carries B's own value");
		}
	}

	/* --- 20. a whole lost session is visible on the bus via DIAG_ERR_SESSION_LOST ----------- */
	{
		reset_all();
		/* Fill all four summary slots, then lose a fifth (same setup as check 14). */
		for (int i = 0; i < (int)DIAG_SESSION_SUMMARIES; i++) {
			ride(10, 0, 0, 0);
			stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		}
		ride(10, 0, 0, 0);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		check(diag_summary_rejected() == 1, "20: one session was lost to the full summary queue");

		pump(20000);   /* drain the four real summaries */

		int t0 = nth_index_of(0x0001021EU, 0);
		check(t0 >= 0, "20: the first trailer after the loss went out");
		if (t0 >= 0) {
			check((sent_payload[t0][6] & DIAG_ERR_SESSION_LOST) != 0,
				"20: it carries DIAG_ERR_SESSION_LOST - a lost SESSION (not just a lost record) "
				"is visible to a parser, not silently invisible");
		}
		int t1 = nth_index_of(0x0001021EU, 1);
		if (t1 >= 0) {
			check((sent_payload[t1][6] & DIAG_ERR_SESSION_LOST) == 0,
				"20: ...and the NEXT trailer does not repeat it - a delta, not a sticky flag");
		}
	}

	/* --- 21. many calls at the SAME tick start at most one new frame ----------------------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		add_record(DIAG_SRC_EPISODE, diag_session_current_id(), 2);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		uint32_t t0 = now;
		for (int i = 0; i < 20; i++) diag_session_dump_step(t0, true);   /* 20 calls, same tick */
		check(count_id(0x0001021DU) == 1,
			"21: the header is the only frame that went out across 20 calls at the same tick");
		check(count_id(0x00010218U) == 0,
			"21: the metrics frame did NOT start too - one new frame per interval, not per call. "
			"This is the exact shape of the real defect: main.c's while(1) calls dump_step() many "
			"times within one control tick, and the old code had nothing to stop it sending the "
			"whole dump in one burst");
	}

	/* --- 22. before the interval elapses, the next frame does not start -------------------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		add_record(DIAG_SRC_EPISODE, diag_session_current_id(), 2);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		uint32_t t0 = now;
		diag_session_dump_step(t0, true);   /* PH_IDLE -> PH_HEADER; no transmit attempted yet */
		diag_session_dump_step(t0, true);   /* header offered */
		diag_session_dump_step(t0, true);   /* header confirmed; tx_pace_last_tick = t0, now PH_METRICS */
		check(count_id(0x0001021DU) == 1, "22: setup - the header went out");

		/*
		 * Two calls, both one tick short of the interval - not one. A single call cannot tell
		 * "blocked by pacing" apart from "merely offered, not yet confirmed" (both read as
		 * count==0); a second call at the SAME still-too-early tick would confirm an offered
		 * frame if pacing had let it start, so seeing count==0 after both is what actually pins
		 * the blame on pacing specifically.
		 */
		diag_session_dump_step(t0 + DIAG_TX_FRAME_INTERVAL_TICKS - 1U, true);
		diag_session_dump_step(t0 + DIAG_TX_FRAME_INTERVAL_TICKS - 1U, true);
		check(count_id(0x00010218U) == 0,
			"22: one tick short of the interval, the metrics frame still has not started");
	}

	/* --- 23. exactly at the interval, the next frame CAN start ----------------------------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		add_record(DIAG_SRC_EPISODE, diag_session_current_id(), 2);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		uint32_t t0 = now;
		diag_session_dump_step(t0, true);   /* PH_IDLE -> PH_HEADER; no transmit attempted yet */
		diag_session_dump_step(t0, true);   /* header offered */
		diag_session_dump_step(t0, true);   /* header confirmed; tx_pace_last_tick = t0 */
		diag_session_dump_step(t0 + DIAG_TX_FRAME_INTERVAL_TICKS, true);   /* exactly one interval: offered */
		diag_session_dump_step(t0 + DIAG_TX_FRAME_INTERVAL_TICKS, true);   /* confirmed */
		check(count_id(0x00010218U) == 1,
			"23: exactly DIAG_TX_FRAME_INTERVAL_TICKS after the header, the metrics frame starts");
	}

	/* --- 24. the pacing check survives a uint32_t wraparound of now_tick ------------------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		add_record(DIAG_SRC_EPISODE, diag_session_current_id(), 2);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		uint32_t near_wrap = (uint32_t)0xFFFFFFFFU - 5U;   /* 5 ticks before the 32-bit wrap */
		diag_session_dump_step(near_wrap, true);   /* PH_IDLE -> PH_HEADER; no transmit attempted yet */
		diag_session_dump_step(near_wrap, true);   /* header offered */
		diag_session_dump_step(near_wrap, true);   /* header confirmed; tx_pace_last_tick = near_wrap */
		check(count_id(0x0001021DU) == 1, "24: setup - the header went out just before the wrap");

		/*
		 * Only ONE tick has really elapsed - now_tick has not even wrapped yet. A deadline-style
		 * check (`now_tick >= tx_pace_last_tick + DIAG_TX_FRAME_INTERVAL_TICKS`) computes a
		 * deadline that itself silently overflows past UINT32_MAX to a SMALL number, so a large,
		 * not-yet-wrapped now_tick would already read as ">= deadline" after a single tick.
		 * Subtraction does not have this failure mode.
		 */
		diag_session_dump_step(near_wrap + 1U, true);
		check(count_id(0x00010218U) == 0,
			"24: one tick after the header, now_tick has NOT wrapped yet and this is still "
			"correctly paced - a `now_tick >= deadline` check would have failed exactly here");

		/* now_tick itself has now wrapped. 9 ticks have really elapsed - still short. */
		diag_session_dump_step(near_wrap + 10U, true);
		check(count_id(0x00010218U) == 0, "24: 9 ticks elapsed across the wrap - still paced");

		/* A full interval has now really elapsed, entirely on the far side of the wrap. */
		diag_session_dump_step(near_wrap + DIAG_TX_FRAME_INTERVAL_TICKS, true);   /* offered */
		diag_session_dump_step(near_wrap + DIAG_TX_FRAME_INTERVAL_TICKS, true);   /* confirmed */
		check(count_id(0x00010218U) == 1,
			"24: a full interval elapsed across the wrap - the next frame correctly starts");
	}

	/* --- 25. NOMAILBOX does not arm the pacing clock, move the cursor, or lose the fragment - */
	{
		reset_all();
		ride(10, 0, 0, 0);
		add_record(DIAG_SRC_EPISODE, diag_session_current_id(), 2);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		uint32_t t0 = now;
		diag_session_dump_step(t0, true);   /* PH_IDLE -> PH_HEADER; no transmit attempted yet */

		fake_nomailbox_for = 1;
		diag_session_dump_step(t0, true);
		check(count_id(0x0001021DU) == 0, "25: NOMAILBOX really wrote nothing");

		/*
		 * The VERY NEXT tick - nowhere near a 10 ms interval - succeeds. If the refused attempt
		 * had armed the pacing clock, this would still be blocked, just by pacing instead of by
		 * the (now exhausted) NOMAILBOX counter.
		 */
		diag_session_dump_step(t0 + 1U, true);
		diag_session_dump_step(t0 + 1U, true);
		check(count_id(0x0001021DU) == 1,
			"25: one tick after the NOMAILBOX refusal, the SAME header fragment went out - a "
			"refused attempt neither arms the pacing clock nor loses or skips the fragment");
	}

	/* --- 26. a FAILED retry offers the identical fragment again, and is itself paced ------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		add_record(DIAG_SRC_EPISODE, diag_session_current_id(), 2);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		uint32_t t0 = now;
		diag_session_dump_step(t0, true);        /* PH_IDLE -> PH_HEADER; no transmit attempted yet */

		fake_fail_for = 1;
		diag_session_dump_step(t0, true);        /* header offered */
		diag_session_dump_step(t0, true);        /* FAILED */
		check(diag_counter_sent(DIAG_SRC_EPISODE) == 0, "26: setup - nothing sent yet");

		/*
		 * Two calls, both on the same tick as the FAILED result - not one, for the same reason
		 * as check 22: a single call cannot tell "blocked by pacing" apart from "just offered".
		 */
		diag_session_dump_step(t0, true);
		diag_session_dump_step(t0, true);
		check(count_id(0x0001021DU) == 0,
			"26: the retry did not fire on the same tick as the failure - it is paced too");

		diag_session_dump_step(t0 + DIAG_TX_FRAME_INTERVAL_TICKS, true);
		diag_session_dump_step(t0 + DIAG_TX_FRAME_INTERVAL_TICKS, true);
		check(count_id(0x0001021DU) == 1,
			"26: a full interval later, the IDENTICAL header fragment was retried and confirmed "
			"- nothing skipped, nothing duplicated");
	}

	/* --- 27. riding interrupts the dump at once, even while pacing blocks the next frame --- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		uint8_t sid = diag_session_current_id();
		add_record(DIAG_SRC_EPISODE, sid, 2);
		add_record(DIAG_SRC_TRACE, sid, 6);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		check(pump_until_kind(DIAG_REC_TRACE, 500), "27: a trace record is the one in flight");

		/*
		 * Whatever tick pump_until_kind() stopped at, a new frame was recently started (that is
		 * how a record becomes "in flight" at all), so the pacing timer would normally still be
		 * blocking the NEXT new frame for a further DIAG_TX_FRAME_INTERVAL_TICKS. Interrupt on
		 * this SAME tick and confirm the abort does not wait for that timer.
		 */
		diag_session_note_activity(now, 0, 0);
		diag_session_tick(now, true, 0, 0, 0);
		diag_session_dump_step(now, true);

		check(diag_counter_discarded(DIAG_SRC_TRACE) == 1,
			"27: the in-flight trace record was discarded on the SAME tick the ride resumed - "
			"whatever the pacing timer was doing did not delay the abort");
		check(diag_dump_record_kind() == DIAG_REC_NONE, "27: nothing is at risk any more");
	}

	/* --- 28. a large, realistic dump (the shape of the ride log) still ends correctly ------- */
	{
		reset_all();
		/*
		 * src_frame()'s fake fragment id is base_efid + frag, unbounded - reset_all()'s default
		 * bases sit close together (0x10210/0x10215/0x1021B) because no other test offers more
		 * than a handful of fragments per record. A full 513-fragment trace record would overflow
		 * straight through the real header/metrics/trailer ids (0x10218, 0x1021D, 0x1021E) and be
		 * miscounted as one of them. Spread out here, clear of the whole 0x10200-0x102FF block,
		 * since this test is the first to actually exercise a full-size capture.
		 */
		fsrc[DIAG_SRC_EPISODE].base_efid = 0x00020000U;
		fsrc[DIAG_SRC_TRACE].base_efid   = 0x00030000U;
		fsrc[DIAG_SRC_RAW].base_efid     = 0x00040000U;

		ride(10, 0, 0, 0);
		uint8_t sid = diag_session_current_id();
		for (int i = 0; i < 15; i++) add_record(DIAG_SRC_EPISODE, sid, 5);   /* 15 episodes */
		add_record(DIAG_SRC_TRACE, sid, 513);   /* one full 256-sample capture: 1 + 256*2 */
		add_record(DIAG_SRC_RAW, sid, 257);     /* one full 256-sample capture: 1 + 256 */
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		pump(4000);   /* pump() advances `now` by one pacing interval per call - see its comment */

		int t = index_of(0x0001021EU);
		check(t >= 0, "28: the trailer went out");
		check(diag_summaries_pending() == 0, "28: the dump fully drained");
		check(diag_counter_pending(DIAG_SRC_EPISODE) == 0 && diag_counter_pending(DIAG_SRC_TRACE) == 0 &&
			diag_counter_pending(DIAG_SRC_RAW) == 0, "28: every record resolved - nothing orphaned");
		if (t >= 0) {
			check(be16(&sent_payload[t][2]) == 17,
				"28: records_sent = 17 (15 episodes + one trace capture + one raw capture)");
			check(sent_payload[t][4] == 0, "28: nothing discarded");
			check(sent_payload[t][7] == 1, "28: dump_complete = 1");
		}
		/* header + metrics + 3 aggregate (reset_all's default) + 15*5 episode fragments +
		 * 513 trace fragments + 257 raw fragments + trailer = 851, exactly. */
		check(sent_n == 851, "28: exactly the expected number of frames went out");
	}

	/* --- 29. a running total already past 65535 before a session starts -> a correct delta --- */
	{
		reset_all();
		/* Running total already past 65535 - modelling main.c's global AFTER the FW-110 fix
		 * (uint32_t, unsaturated) having accumulated this many missed events over prior uptime,
		 * long before this ride even started. */
		ride(5, 0, 70000, 0);
		ride(5, 0, 70500, 40);                 /* this session: +500 events while riding */
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 70500);
		check(diag_summaries_pending() == 1, "29: the session closed");

		pump(4000);
		int m = index_of(0x00010218U);
		check(m >= 0, "29: the session's metrics frame went out");
		if (m >= 0) {
			check(be16(&sent_payload[m][4]) == 500,
				"29: the session reports a delta of 500 events, not 0 - the running total was "
				"already past 65535 before this session even started, which is exactly what "
				"saturated a uint16_t source global into looking like zero movement forever");
		}
	}

	/* --- 30. exactly 0 lost events publishes 0, even from a huge running total ------------- */
	{
		reset_all();
		ride(5, 0, 123456, 0);
		ride(5, 0, 123456, 0);           /* no NEW events this session */
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 123456);
		pump(4000);
		int m = index_of(0x00010218U);
		check(m >= 0, "30: the metrics frame went out");
		if (m >= 0) {
			check(be16(&sent_payload[m][4]) == 0,
				"30: zero NEW events this session publishes exactly 0, not a stale nonzero delta");
		}
	}

	/* --- 31. a session's OWN delta exceeding 65535 clamps to 0xFFFF, not truncated bits ----- */
	{
		reset_all();
		ride(5, 0, 0, 0);
		ride(5, 0, 70000, 40);           /* this ONE session's own delta is 70000 - itself > 0xFFFF */
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 70000);
		pump(4000);
		int m = index_of(0x00010218U);
		check(m >= 0, "31: the metrics frame went out");
		if (m >= 0) {
			check(be16(&sent_payload[m][4]) == 0xFFFFU,
				"31: a session whose own delta (70000) exceeds the 16-bit wire field clamps to "
				"0xFFFF, the true ceiling - not 70000 & 0xFFFF = 4464, which a plain truncating "
				"cast would have silently produced");
		}
	}

	/* --- 32. allow_new_tx=false: nothing NEW ever reaches transmit() --------------------------
	 * FW-110: diagnostics may not claim a mailbox for a new frame while any critical frame is
	 * still waiting for one. Get the header fully sent and confirmed first (so the metrics
	 * frame is a genuinely NEW claim, not the one already in flight), then gate. */
	{
		reset_all();
		ride(10, 0, 0, 0);
		add_record(DIAG_SRC_EPISODE, diag_session_current_id(), 2);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);

		diag_session_dump_step(now, true);   /* PH_IDLE -> PH_HEADER */
		diag_session_dump_step(now, true);   /* header offered */
		diag_session_dump_step(now, true);   /* header confirmed OK -> PH_METRICS */
		check(count_id(0x0001021DU) == 1, "32: setup - header went out, now on PH_METRICS");

		uint32_t transmit_before = fake_transmit_calls;
		for (int i = 0; i < 10; i++) diag_session_dump_step(now, false);
		check(fake_transmit_calls == transmit_before,
			"32: allow_new_tx=false - the metrics frame NEVER reaches transmit(), across 10 calls");
		check(count_id(0x00010218U) == 0, "32: ...and so it never goes out");
	}

	/* --- 33. sustained gate across many ticks AND pacing intervals - still zero new ----------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		add_record(DIAG_SRC_EPISODE, diag_session_current_id(), 2);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		diag_session_dump_step(now, true);
		diag_session_dump_step(now, true);
		diag_session_dump_step(now, true);
		check(count_id(0x0001021DU) == 1, "33: setup - header went out");

		uint32_t transmit_before = fake_transmit_calls;
		uint32_t t0 = now;
		/* March across several pacing intervals, gated the whole time - rules out any path that
		 * lets pacing alone (without allow_new_tx) authorise a new attempt. */
		for (int i = 0; i < 50; i++) {
			diag_session_dump_step(t0 + (uint32_t)i * DIAG_TX_FRAME_INTERVAL_TICKS, false);
		}
		check(fake_transmit_calls == transmit_before,
			"33: many ticks, many pacing intervals crossed, allow_new_tx=false throughout - "
			"still zero new diagnostic transmit attempts");
	}

	/* --- 34. once allow_new_tx=true again, diagnostics resumes from the SAME fragment -------- */
	{
		reset_all();
		ride(5, 0, 0, 0);      /* anchor this session's event count at 0 first (see check 12) */
		ride(5, 0, 100, 0);    /* then climb to 100 - THIS session's own delta is 100 */
		add_record(DIAG_SRC_EPISODE, diag_session_current_id(), 2);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 100);
		diag_session_dump_step(now, true);
		diag_session_dump_step(now, true);
		diag_session_dump_step(now, true);
		check(count_id(0x0001021DU) == 1, "34: setup - header went out");

		for (int i = 0; i < 10; i++) diag_session_dump_step(now, false);  /* blocked for a while */
		check(count_id(0x00010218U) == 0, "34: still blocked - metrics has not gone out yet");

		pump(4000);   /* gate open (allow_new_tx=true) from here on */
		check(count_id(0x0001021DU) == 1,
			"34: header was NOT re-sent - the dump resumed, it did not restart from PH_IDLE");
		int m = index_of(0x00010218U);
		check(m >= 0, "34: metrics resumed and completed");
		if (m >= 0) {
			check(be16(&sent_payload[m][4]) == 100,
				"34: metrics content is correct - the very fragment that was blocked, sent intact");
		}
		check(count_id(0x00010210U) == 1, "34: the episode record after it also went out");
		check(count_id(0x0001021EU) == 1, "34: and the trailer closed the dump");
	}

	/* --- 35. riding resumes and aborts the dump AT ONCE, even with allow_new_tx=false -------- */
	{
		reset_all();
		ride(10, 0, 0, 0);
		uint8_t sid_a = diag_session_current_id();
		add_record(DIAG_SRC_EPISODE, sid_a, 2);
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		diag_session_dump_step(now, true);
		diag_session_dump_step(now, true);
		diag_session_dump_step(now, true);
		check(count_id(0x0001021DU) == 1, "35: setup - header went out, dump now on PH_METRICS");

		for (int i = 0; i < 5; i++) diag_session_dump_step(now, false);  /* gated for a while */
		check(count_id(0x00010218U) == 0, "35: setup - still blocked before the metrics frame");

		ride(5, 0, 0, 0);   /* the rider sets off again - diag_session_tick(moving=true), same
		                     * call main.c makes every tick regardless of the dump */
		check(diag_session_is_active(), "35: a new session is active - the ride was noticed");
		check(diag_session_current_id() != sid_a, "35: it really is a NEW session (B), not A resuming");

		diag_session_dump_step(now, false);   /* allow_new_tx STILL false - abort must not need it */

		/*
		 * Prove the dump really was reset to PH_IDLE (not left stalled mid-METRICS): once this
		 * new ride (session B) ends too, A's STILL-PENDING summary is dumped again from its OWN
		 * header - a dump merely waiting on allow_new_tx would have resumed from PH_METRICS
		 * instead, and never sent a second 0x1021D for A at all. B closing alongside means two
		 * headers now go out in total (one per session, same shape as check 6), so this looks
		 * for A's header specifically rather than asserting a bare count.
		 */
		stand(DIAG_SESSION_QUIET_TICKS + 2U, 0, 0);
		sent_n = 0;
		pump(4000);
		bool sid_a_header_seen_again = false;
		for (uint16_t i = 0; i < sent_n; i++) {
			if (sent_log[i] == 0x0001021DU && sent_payload[i][1] == sid_a) sid_a_header_seen_again = true;
		}
		check(sid_a_header_seen_again,
			"35: session A's header went out AGAIN on resumption - proof the abort reset the "
			"dump to PH_IDLE, it was not merely left waiting mid-METRICS for allow_new_tx");
	}

	if (failures == 0) {
		printf("All FW-106 session checks passed.\n");
		return 0;
	}
	printf("\n%d FW-106 session check(s) FAILED.\n", failures);
	return 1;
}
