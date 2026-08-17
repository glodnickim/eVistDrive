/*
 * FW-106 integration harness — links the SHIPPED diag_session.c, pas_trace.c, pas_raw.c and
 * ride_episode.c together and replays main.c's REAL call order, not a model of it.
 *
 * Build and run (see run-host-tests.ps1):
 *   gcc -std=c11 -Wall -Wextra -DCAN_DIAGNOSTICS_ENABLE=1 -I../../inc -o fw106i \
 *       fw106_integration_host.c ../../src/diag_session.c ../../src/pas_trace.c \
 *       ../../src/pas_raw.c ../../src/ride_episode.c && ./fw106i
 *
 * WHY THIS EXISTS. tests/host/fw106_session_host.c proves diag_session.c is correct against FAKE
 * record sources, and tests/host/fw106_recorder_host.c proves pas_trace/pas_raw/ride_episode are
 * correct in isolation. Neither can catch a bug that lives in the ORDER two real modules are
 * called in from main.c: the first suspicious PAS edge of a ride used to be able to arm
 * pas_trace/pas_raw while they still carried the PREVIOUS session's id, because
 * pas_trace_transition() ran before diag_session_tick() had bumped it for this tick. A fake
 * source cannot reproduce that, because a fake does not have its own "when was I armed" state to
 * get wrong - only the real pas_trace.c and pas_raw.c do. This harness wires the real modules the
 * way main.c does (same three-line propagation, same session-scoped queries) and drives them
 * through the real sequence: a stop, a first movement that is itself the suspicious transition,
 * more riding, and a stop again.
 */

#include <stdio.h>
#include <string.h>

#include "diag_session.h"
#include "pas_trace.h"
#include "pas_raw.h"
#include "ride_episode.h"

static int failures;
static void check(int ok, const char *label)
{
	if (!ok) { failures++; printf("  FAIL  %s\n", label); }
}

#define MS(x) ((uint32_t)(x) * 4U)   /* 4 kHz: one millisecond is four ticks */

/* --- a CAN fake, the same shape as fw106_session_host.c's ---------------------------------- */

#define LOG_MAX 4096
static uint32_t sent_log[LOG_MAX];
static uint8_t  sent_payload[LOG_MAX][8];
static uint16_t sent_n;
static uint32_t inflight_efid;
static uint8_t  inflight_data[8];
static bool     busy;

static uint8_t fake_transmit(uint32_t efid, const uint8_t *data)
{
	if (busy) return DIAG_CAN_NOMAILBOX;
	inflight_efid = efid;
	memcpy(inflight_data, data, 8);
	busy = true;
	return 1U;
}

static uint8_t fake_state(uint8_t mailbox)
{
	(void)mailbox;
	if (!busy) return DIAG_CAN_FAILED;
	busy = false;
	if (sent_n < LOG_MAX) {
		sent_log[sent_n] = inflight_efid;
		memcpy(sent_payload[sent_n], inflight_data, 8);
		sent_n++;
	}
	return DIAG_CAN_OK;
}

static const diag_can_ops_t fake_can = { fake_transmit, fake_state };

static int count_id(uint32_t efid)
{
	int n = 0;
	for (uint16_t i = 0; i < sent_n; i++) if (sent_log[i] == efid) n++;
	return n;
}

/* --- record sources: the SAME wiring main.c uses, against the REAL modules ----------------- */

static bool ep_frame(uint8_t sid, uint16_t frag, uint32_t *efid, uint8_t *data, bool *last)
{
	ride_episode_result_t ep;
	if (!ride_episode_queue_peek_session(sid, &ep)) return false;
	memset(data, 0, 8);
	*efid = 0x00010210U + frag;   /* real ids do not matter to this test, only the bracketing */
	*last = (frag == 4U);
	data[0] = ep.session_id;
	return true;
}
static uint16_t ep_count(uint8_t sid) { return ride_episode_queue_count_session(sid); }
static void ep_release(uint8_t sid, bool sent) { (void)sent; ride_episode_queue_release_session(sid); }
static uint32_t ep_accepted(void) { return ride_episode_queue_enqueued(); }
static uint32_t ep_refused(void)  { return ride_episode_queue_rejected(); }

static bool tr_frame(uint8_t sid, uint16_t frag, uint32_t *efid, uint8_t *data, bool *last)
{
	int8_t slot = pas_trace_oldest_ready_slot_of(sid);
	if (slot < 0) return false;
	uint16_t n = pas_trace_slot_count((uint8_t)slot);
	memset(data, 0, 8);
	*efid = 0x00010216U;
	*last = (frag + 1U) >= n;
	data[0] = pas_trace_slot_session_id((uint8_t)slot);
	return true;
}
static uint16_t tr_count(uint8_t sid) { return pas_trace_count_session(sid); }
static void tr_release(uint8_t sid, bool sent)
{
	(void)sent;
	int8_t slot = pas_trace_oldest_ready_slot_of(sid);
	if (slot >= 0) pas_trace_slot_release((uint8_t)slot);
}
static uint32_t tr_accepted(void) { return pas_trace_captures_frozen(); }
static uint32_t tr_refused(void)  { return pas_trace_capture_slots_full(); }

static bool rw_frame(uint8_t sid, uint16_t frag, uint32_t *efid, uint8_t *data, bool *last)
{
	if (pas_raw_count_session(sid) == 0) return false;
	uint16_t n = pas_raw_slot_count();
	memset(data, 0, 8);
	*efid = 0x0001021CU;
	*last = (frag + 1U) >= n;
	data[0] = pas_raw_slot_session_id();
	return true;
}
static uint16_t rw_count(uint8_t sid) { return pas_raw_count_session(sid); }
static void rw_release(uint8_t sid, bool sent) { (void)sid; (void)sent; pas_raw_slot_release(); }
static uint32_t rw_accepted(void) { return pas_raw_captures_frozen(); }
static uint32_t rw_refused(void)  { return pas_raw_freeze_skipped(); }

static bool agg_frame(uint16_t index, uint32_t *efid, uint8_t *data)
{
	(void)index; (void)efid; (void)data;
	return false;   /* this test cares about episode/trace/raw bracketing, not the aggregate block */
}
static void seal_open(void)
{
	pas_trace_seal_open_captures();
	pas_raw_seal_open_capture();
}
static uint32_t raw_overrun_total(void) { return pas_raw_capture_overrun(); }

static const diag_ops_t real_ops = {
	agg_frame, seal_open, raw_overrun_total,
	{
		{ ep_count, ep_frame, ep_release, ep_accepted, ep_refused },
		{ tr_count, tr_frame, tr_release, tr_accepted, tr_refused },
		{ rw_count, rw_frame, rw_release, rw_accepted, rw_refused }
	}
};

/* --- the simulated control loop, replaying main.c's REAL integration order ----------------- */

static uint32_t g_missed_ticks, g_missed_events;
static uint8_t  g_pas_state;      /* current simulated GPIOC/GPIOD quadrature state, 0..3 */
static int32_t  g_iq;             /* MS.i_q_setpoint stand-in */
static bool     g_wheel_moving;

/*
 * One control iteration. The order below is deliberately the same order main.c uses:
 *   1. the ISR's raw sample (pas_raw_isr_sample) - modelled as happening first, as it always
 *      does on real hardware, an interrupt ahead of anything the main loop does this tick;
 *   2. IF the quadrature state changed this tick: diag_session_note_activity() FIRST, id
 *      propagated immediately, THEN pas_trace_transition()/pas_raw_freeze() - this exact
 *      ordering is the fix under test;
 *   3. the full diag_session_tick() - always, every iteration, exactly as main.c calls it;
 *   4. diag_session_dump_step() - always, every iteration, exactly as main.c calls it.
 */
static uint16_t g_gap_ticks;

static void control_iteration(uint32_t tick, uint8_t new_pas_state, int32_t iq, bool wheel_moving)
{
	pas_raw_isr_sample(new_pas_state, tick);

	bool transitioned = (new_pas_state != g_pas_state);
	if (transitioned) {
		diag_session_note_activity(tick, g_missed_ticks, g_missed_events);
		ride_episode_set_session_id(diag_session_current_id());
		pas_trace_set_session_id(diag_session_current_id());
		pas_raw_set_session_id(diag_session_current_id());

		pas_trace_input_t in;
		memset(&in, 0, sizeof(in));
		in.from_state = g_pas_state;
		in.to_state = new_pas_state;
		in.gap_ticks = g_gap_ticks;
		in.iq_setpoint = (uint16_t)(iq < 0 ? 0 : iq);
		uint8_t cap = pas_trace_transition(&in);
		if (cap != PAS_TRACE_NO_CAPTURE) pas_raw_freeze(cap);

		g_pas_state = new_pas_state;
		g_gap_ticks = 0;
	} else {
		if (g_gap_ticks < 60000U) g_gap_ticks++;
	}

	g_iq = iq;
	g_wheel_moving = wheel_moving;

	bool moving = wheel_moving || (iq != 0) || (g_gap_ticks < 100U);
	diag_session_tick(tick, moving, g_missed_ticks, g_missed_events, 0);
	diag_session_dump_step(tick, true);
}

static void run_idle(uint32_t from, uint32_t to)
{
	for (uint32_t t = from; t < to; t++) control_iteration(t, g_pas_state, 0, false);
}

int main(void)
{
	printf("FW-106 integration: diag_session + pas_trace + pas_raw + ride_episode together\n");

	ride_episode_init();
	pas_trace_init();
	pas_raw_init();
	diag_session_init(&fake_can, &real_ops);
	g_pas_state = 0;
	g_gap_ticks = 60000U;   /* long idle behind us */

	/* --- stopped: session_id starts at 0, nothing active ---------------------------------- */
	run_idle(0, 200);
	check(diag_session_current_id() == 0, "pre: no session has started yet, id is still 0");
	check(!diag_session_is_active(), "pre: session is not active while stopped");

	/*
	 * --- the first movement IS the suspicious transition -----------------------------------
	 * A quadrature edge with gap_ticks <= PAS_TRACE_SHORT_GAP_TICKS (3): exactly the shape of
	 * the short bounce this whole card was opened to investigate. g_gap_ticks is already huge
	 * from the idle period above, so drive it down to a short value first without a real edge,
	 * then present the short-gap edge on the very next tick - the first live one of the ride.
	 */
	static const uint8_t gray[4] = {0, 1, 3, 2};   /* a real quadrature sequence: one bit at a time */
	uint8_t gray_i = 0;   /* g_pas_state == gray[gray_i] is kept true throughout */

	uint32_t t = 200;
	g_gap_ticks = 2;   /* pretend a tiny amount of time has passed since a (fictional) prior edge */
	gray_i = 1;
	control_iteration(t, gray[gray_i], 50 /* iq now flowing */, false);
	t++;

	check(diag_session_current_id() == 1, "trigger tick: the session has started (id 1)");
	/*
	 * The capture is only ARMED here, not yet frozen - PAS_TRACE_POST more transitions are still
	 * needed before it becomes readable at all (see pas_trace.h). Whether it was stamped with
	 * the right session can only be checked once it IS readable, after the riding loop below -
	 * which is exactly where the next block checks it.
	 */

	/*
	 * --- keep riding: real PAS steps, spaced out, none of them suspicious ------------------
	 * A transition every 10th tick, gap_ticks ~9 (> PAS_TRACE_SHORT_GAP_TICKS, so this does NOT
	 * re-trigger pas_trace), through a genuine one-bit-at-a-time quadrature sequence (so none of
	 * them is a TWO_BIT transition either). >=130 of these give pas_raw's 128-event POST window
	 * time to complete from the ring it started collecting into at the trigger tick.
	 */
	for (uint32_t i = 0; i < 150U; i++) {
		for (uint32_t k = 0; k < 9U; k++) { control_iteration(t, gray[gray_i], 50, true); t++; }
		gray_i = (uint8_t)((gray_i + 1U) & 0x3U);
		control_iteration(t, gray[gray_i], 50, true);
		t++;
	}
	int8_t tr_slot2 = pas_trace_oldest_ready_slot_of(1);
	check(tr_slot2 >= 0, "riding: the trace capture from the trigger tick has now completed");
	if (tr_slot2 >= 0) {
		check(pas_trace_slot_session_id((uint8_t)tr_slot2) == 1,
			"riding: ...and it is stamped session 1, NOT the stale session 0 - THE FIX UNDER TEST");
	}
	check(pas_raw_count_session(1) == 1, "riding: the raw capture completed under session 1");
	check(pas_raw_slot_session_id() == 1, "riding: ...stamped with the NEW session, not the old");
	check(pas_trace_count_session(0) == 0,
		"riding: nothing at all belongs to session 0 - the ordering bug would have put the "
		"trigger-tick capture here instead");
	check(pas_raw_count_session(0) == 0, "riding: ...on the raw side either");

	/*
	 * --- stop: quiet long enough for the session to close -----------------------------------
	 * The synthetic `moving` signal above stays true for a short grace period after the last
	 * transition (mirroring main.c's own pas_idle_ticks<=pas_stop_timeout window), so the true
	 * quiet period only starts once that grace elapses - the loop runs long enough to cover both.
	 */
	for (uint32_t i = 0; i < DIAG_SESSION_QUIET_TICKS + 300U; i++) {
		control_iteration(t, g_pas_state, 0, false);
		t++;
	}
	check(diag_summaries_pending() == 1, "stop: the session closed and is queued to dump");

	/*
	 * --- drain the dump and check the bracket -----------------------------------------------
	 * FW-106: dump_step() now paces every NEW frame to DIAG_TX_FRAME_INTERVAL_TICKS apart, and
	 * this loop advances the tick by exactly 1 per control_iteration() (it must - diag_session_
	 * tick() and the real recorders need genuine tick-by-tick time, not the 40-ticks-per-call
	 * shortcut fw106_session_host.c's pump() takes against fake sources). The trace and raw
	 * captures here can hold up to PAS_TRACE_LEN/PAS_RAW_RING_LEN (256) samples each, so the
	 * dump can need on the order of (1 header + 1 metrics + up to 256 trace + up to 256 raw + 1
	 * trailer) * DIAG_TX_FRAME_INTERVAL_TICKS =~ 20600 ticks in the worst case; 2000 was enough
	 * before pacing existed and is not any more.
	 */
	for (uint32_t i = 0; i < 40000U; i++) control_iteration(t++, g_pas_state, 0, false);

	check(diag_summaries_pending() == 0, "dump: the session's summary was fully drained");
	check(count_id(0x0001021DU) == 1, "dump: exactly one session header went out");
	check(count_id(0x0001021EU) == 1, "dump: exactly one trailer closed it");

	int hdr = -1, trl = -1;
	for (uint16_t i = 0; i < sent_n; i++) {
		if (sent_log[i] == 0x0001021DU) hdr = (int)i;
		if (sent_log[i] == 0x0001021EU) trl = (int)i;
	}
	check(hdr >= 0 && sent_payload[hdr][1] == 1, "dump: the header names session 1");
	check(trl >= 0 && sent_payload[trl][1] == 1, "dump: the trailer names session 1 too");

	check(diag_counter_pending(DIAG_SRC_TRACE) == 0,
		"dump: the trace capture was found and sent - pending is back to 0, not orphaned");
	check(diag_counter_pending(DIAG_SRC_RAW) == 0,
		"dump: the raw capture was found and sent - pending is back to 0, not orphaned");
	check(diag_counter_sent(DIAG_SRC_TRACE) == 1, "dump: exactly one trace record was sent");
	check(diag_counter_sent(DIAG_SRC_RAW) == 1, "dump: exactly one raw record was sent");
	check(diag_counter_discarded(DIAG_SRC_TRACE) == 0 && diag_counter_discarded(DIAG_SRC_RAW) == 0,
		"dump: neither record needed to be discarded - both were where the dump looked");

	if (failures == 0) {
		printf("All FW-106 integration checks passed.\n");
		return 0;
	}
	printf("\n%d FW-106 integration check(s) FAILED.\n", failures);
	return 1;
}
