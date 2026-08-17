/*
 * FW-110: host tests for src/can_tx_queue.c, the bounded, non-blocking TX queue for CAN frames
 * that must be delivered - against the REAL, shipped module, driven by a mock CAN peripheral
 * that never touches actual hardware and can simulate busy/pending/failed/OK on command.
 */

#include "../common/check.h"

#include "can_tx_queue.h"

/* --- mock peripheral: fully scripted, counts every call the module makes on it -------------- */

#define MOCK_SCRIPT_LEN 64
static uint8_t mock_transmit_script[MOCK_SCRIPT_LEN]; /* CANQ_NOMAILBOX or a fake mailbox id */
static uint8_t mock_state_script[MOCK_SCRIPT_LEN];    /* CANQ_OK/PENDING/FAILED per fake mailbox call */
static int mock_transmit_calls;
static int mock_state_calls;
static int mock_transmit_step;
static int mock_state_step;

static void mock_reset(void)
{
	mock_transmit_calls = 0;
	mock_state_calls = 0;
	mock_transmit_step = 0;
	mock_state_step = 0;
	for (int i = 0; i < MOCK_SCRIPT_LEN; i++) {
		mock_transmit_script[i] = 1U;   /* default: always a free mailbox (id 1) */
		mock_state_script[i] = CANQ_OK; /* default: always completes immediately */
	}
}

static uint8_t mock_transmit(uint32_t efid, uint8_t dlen, const uint8_t data[8])
{
	(void)efid; (void)dlen; (void)data;
	mock_transmit_calls++;
	uint8_t v = mock_transmit_script[mock_transmit_step < MOCK_SCRIPT_LEN ? mock_transmit_step : MOCK_SCRIPT_LEN - 1];
	mock_transmit_step++;
	return v;
}

static uint8_t mock_state(uint8_t mailbox)
{
	(void)mailbox;
	mock_state_calls++;
	uint8_t v = mock_state_script[mock_state_step < MOCK_SCRIPT_LEN ? mock_state_step : MOCK_SCRIPT_LEN - 1];
	mock_state_step++;
	return v;
}

static const can_tx_ops_t mock_ops = { mock_transmit, mock_state };

static const uint8_t sample_data[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };

/* --- 1: mailbox busy through many calls - bounded cost, frame survives -------------------- */
static void test_busy_bounded_cost(void)
{
	mock_reset();
	can_tx_queue_init(&mock_ops);
	for (int i = 0; i < MOCK_SCRIPT_LEN; i++) mock_transmit_script[i] = CANQ_NOMAILBOX;

	CHECK(can_tx_queue_enqueue(0x100U, 8U, sample_data), "1: enqueue succeeds");
	CHECK(can_tx_queue_depth() == 1U, "1: one frame queued");

	for (int i = 0; i < 20; i++) {
		int before_transmit = mock_transmit_calls;
		int before_state = mock_state_calls;
		can_tx_queue_service(100U + (uint32_t)i);
		int used = (mock_transmit_calls - before_transmit) + (mock_state_calls - before_state);
		CHECK(used == 1, "1: each service() call touches the mock EXACTLY once (bounded cost)");
	}
	CHECK(can_tx_queue_depth() == 1U, "1: a persistently busy mailbox never loses the frame");
	CHECK(can_tx_queue_busy_count() == 20U, "1: busy counted once per call, no more, no less");
	CHECK(can_tx_queue_failed_count() == 0U, "1: busy is not failure - no give-up yet");
}

/* --- 2: PENDING -> OK ----------------------------------------------------------------------- */
static void test_pending_to_ok(void)
{
	mock_reset();
	can_tx_queue_init(&mock_ops);
	mock_transmit_script[0] = 7U;             /* accepted into mailbox 7 */
	mock_state_script[0] = CANQ_PENDING;      /* still on the wire... */
	mock_state_script[1] = CANQ_PENDING;      /* ...twice... */
	mock_state_script[2] = CANQ_OK;           /* ...then done */

	CHECK(can_tx_queue_enqueue(0x200U, 4U, sample_data), "2: enqueue succeeds");

	can_tx_queue_service(0U);   /* starts transmit -> mailbox 7 */
	CHECK(mock_transmit_calls == 1 && mock_state_calls == 0, "2: first call only transmits");
	CHECK(can_tx_queue_depth() == 1U, "2: still queued/in-flight, not yet complete");

	can_tx_queue_service(1U);   /* state -> PENDING */
	CHECK(can_tx_queue_depth() == 1U, "2: PENDING does not complete or drop the frame");

	can_tx_queue_service(2U);   /* state -> PENDING again */
	CHECK(can_tx_queue_depth() == 1U, "2: still pending on the second check");

	can_tx_queue_service(3U);   /* state -> OK */
	CHECK(can_tx_queue_depth() == 0U, "2: OK retires the frame");
	CHECK(can_tx_queue_failed_count() == 0U, "2: a clean PENDING->OK path never counts as failed");
}

/* --- 3: FAILED -> retry -> OK ---------------------------------------------------------------- */
static void test_failed_retry_ok(void)
{
	mock_reset();
	can_tx_queue_init(&mock_ops);
	mock_transmit_script[0] = 3U;
	mock_state_script[0] = CANQ_FAILED;   /* first attempt fails */
	mock_transmit_script[1] = 3U;         /* re-offered */
	mock_state_script[1] = CANQ_OK;       /* second attempt succeeds */

	CHECK(can_tx_queue_enqueue(0x300U, 8U, sample_data), "3: enqueue succeeds");
	can_tx_queue_service(0U);   /* transmit -> mailbox 3 */
	can_tx_queue_service(1U);   /* state -> FAILED: retry_ctr++, frame stays queued */
	CHECK(can_tx_queue_depth() == 1U, "3: a single FAILED does not drop the frame");
	CHECK(can_tx_queue_retry_count() == 1U, "3: retry counted once");
	CHECK(can_tx_queue_failed_count() == 0U, "3: not a give-up yet");

	can_tx_queue_service(2U);   /* re-transmit -> mailbox 3 */
	can_tx_queue_service(3U);   /* state -> OK */
	CHECK(can_tx_queue_depth() == 0U, "3: the retried frame completes");
	CHECK(can_tx_queue_failed_count() == 0U, "3: recovered - never counted as a permanent failure");
}

/* --- 4: persistent FAILED -> give-up, no deadlock -------------------------------------------- */
static void test_persistent_failed_gives_up(void)
{
	mock_reset();
	can_tx_queue_init(&mock_ops);
	for (int i = 0; i < MOCK_SCRIPT_LEN; i++) {
		mock_transmit_script[i] = 9U;
		mock_state_script[i] = CANQ_FAILED;   /* every attempt fails, forever */
	}

	CHECK(can_tx_queue_enqueue(0x400U, 8U, sample_data), "4: enqueue succeeds");

	/* Bounded number of service() calls must be enough to resolve this - each retry costs
	 * exactly 2 calls (one transmit, one state check), so CANQ_MAX_RETRY+2 retries worth of
	 * calls is a generous, still-FINITE bound - if this test ever needed to be raised because
	 * the module started looping, that itself would be the regression. */
	uint32_t tick = 0U;
	for (int i = 0; i < 64; i++) {
		can_tx_queue_service(tick++);
	}
	CHECK(can_tx_queue_depth() == 0U, "4: a persistently failing frame is eventually abandoned, not stuck forever");
	CHECK(can_tx_queue_failed_count() == 1U, "4: exactly one give-up counted");

	/* No deadlock: the queue is usable again immediately afterward. */
	mock_transmit_script[mock_transmit_step] = 2U;
	mock_state_script[mock_state_step] = CANQ_OK;
	CHECK(can_tx_queue_enqueue(0x401U, 8U, sample_data), "4: queue still accepts new frames after a give-up");
	can_tx_queue_service(tick++);
	can_tx_queue_service(tick++);
	CHECK(can_tx_queue_depth() == 0U, "4: and can still successfully deliver one");
}

/* --- 5: full queue ---------------------------------------------------------------------------- */
static void test_full_queue(void)
{
	mock_reset();
	can_tx_queue_init(&mock_ops);
	for (int i = 0; i < MOCK_SCRIPT_LEN; i++) mock_transmit_script[i] = CANQ_NOMAILBOX; /* nothing drains */

	int accepted = 0;
	for (int i = 0; i < 16; i++) {
		if (can_tx_queue_enqueue((uint32_t)(0x500U + (uint32_t)i), 1U, sample_data)) accepted++;
	}
	CHECK(accepted == 16, "5: exactly capacity (16) frames accepted while nothing drains");
	CHECK(can_tx_queue_dropped_count() == 0U, "5: no drops yet - queue was exactly full, not over");

	bool one_more = can_tx_queue_enqueue(0x5FFU, 1U, sample_data);
	CHECK(!one_more, "5: the 17th frame is refused - explicit policy: drop the newest");
	CHECK(can_tx_queue_dropped_count() == 1U, "5: refusal is counted");
	CHECK(can_tx_queue_depth() == 16U, "5: the already-queued 16 are untouched by the refusal");
}

/* --- 6: a competing (diagnostic-like) consumer never displaces critical frames --------------- */
static int shared_mailboxes_free; /* the ENTIRE simulated hardware: how many free mailboxes exist */

static uint8_t shared_transmit(uint32_t efid, uint8_t dlen, const uint8_t data[8])
{
	(void)efid; (void)dlen; (void)data;
	if (shared_mailboxes_free <= 0) return CANQ_NOMAILBOX;
	shared_mailboxes_free--;
	return 1U; /* fixed fake handle - fine, this test never checks state() */
}
static uint8_t shared_state(uint8_t mailbox) { (void)mailbox; return CANQ_PENDING; }
static const can_tx_ops_t shared_ops = { shared_transmit, shared_state };

static void test_diagnostic_never_displaces_critical(void)
{
	/* Model: ONE shared hardware mailbox pool (1 free slot this tick), the real critical queue,
	 * and a second consumer (a bare call through the SAME ops - standing in for diag_session.c's
	 * own, separate, already non-blocking automaton, which shares the same can_message_transmit()
	 * mailbox pool in production even though it is not linked here). Servicing critical FIRST
	 * each tick, as main.c must, is what this test proves matters. */
	mock_reset();
	can_tx_queue_init(&shared_ops);
	CHECK(can_tx_queue_enqueue(0x600U, 8U, sample_data), "6: critical frame queued");

	shared_mailboxes_free = 1; /* only one mailbox free this tick - both want it */
	can_tx_queue_service(0U);  /* critical serviced FIRST, as main.c's own ordering requires */
	CHECK(shared_mailboxes_free == 0, "6: critical claimed the sole free mailbox");

	/* Now the diagnostic-like second consumer tries for the SAME (now empty) pool. */
	uint8_t diag_result = shared_transmit(0x700U, 8U, sample_data);
	CHECK(diag_result == CANQ_NOMAILBOX, "6: diagnostics sees NOMAILBOX - it never took critical's slot");
}

/* --- 7: even a saturated, always-busy stream never blocks the caller ------------------------- */
static void test_saturated_stream_never_blocks(void)
{
	mock_reset();
	can_tx_queue_init(&mock_ops);
	for (int i = 0; i < MOCK_SCRIPT_LEN; i++) mock_transmit_script[i] = CANQ_NOMAILBOX;

	/* Simulate a long run of periodic 0x3100-style enqueue+service calls against a CAN bus
	 * that never has a free mailbox. Every call must still do O(1) mock work - if this ever
	 * looped internally, this test would simply never finish (no timeout mechanism needed to
	 * prove it: a bounded iteration count that completes at all IS the proof). */
	uint32_t tick = 0U;
	for (int i = 0; i < 1000; i++) {
		can_tx_queue_enqueue(0x3100U, 4U, sample_data);   /* may be dropped once full - fine */
		can_tx_queue_service(tick++);
	}
	CHECK(mock_transmit_calls <= 1000, "7: never more than one mock transmit per iteration - no hidden retry loop");
	CHECK(can_tx_queue_depth() <= 16U, "7: queue stays within capacity throughout, control keeps running");
}

/* --- 8: uint32_t tick wraparound for the deadlock guard --------------------------------------- */
static void test_tick_wraparound(void)
{
	mock_reset();
	can_tx_queue_init(&mock_ops);
	mock_transmit_script[0] = 5U;
	mock_state_script[0] = CANQ_PENDING;
	mock_state_script[1] = CANQ_PENDING;
	mock_state_script[2] = CANQ_PENDING;

	CHECK(can_tx_queue_enqueue(0x800U, 8U, sample_data), "8: enqueue succeeds");
	can_tx_queue_service(0xFFFFFFFEU);   /* starts transmit at a tick just before the wrap */
	CHECK(can_tx_queue_depth() == 1U, "8: in flight, anchored at 0xFFFFFFFE");

	/* One tick later (0xFFFFFFFF, pre-wrap): well under the 400-tick deadline - must NOT give up. */
	can_tx_queue_service(0xFFFFFFFFU);
	CHECK(can_tx_queue_depth() == 1U, "8: still within the deadline right at the wrap boundary");
	CHECK(can_tx_queue_failed_count() == 0U, "8: no premature give-up before the wrap");

	/* Two ticks after the anchor, now on the OTHER side of the wrap (tick 0x00000000): a naive
	 * `now - since` comparison without the signed cast would compute 0-0xFFFFFFFE = 2 correctly
	 * anyway by unsigned wraparound arithmetic - so this alone would not distinguish a broken
	 * implementation. The real proof is deadline enforcement AFTER a long, wrapped gap: */
	can_tx_queue_service(0x00000000U);
	CHECK(can_tx_queue_depth() == 1U, "8: 2 ticks elapsed across the wrap - still well within the deadline");

	/* Push well past the 400-tick deadline, still on the wrapped side. */
	can_tx_queue_service(500U);
	CHECK(can_tx_queue_depth() == 0U, "8 (FIX): deadline correctly enforced across the wrap - frame given up");
	CHECK(can_tx_queue_failed_count() == 1U, "8: counted as a genuine give-up, not silently lost");
}

int main(void)
{
	printf("FW-110 can_tx_queue.c non-blocking TX queue, against the shipped module\n");

	test_busy_bounded_cost();
	test_pending_to_ok();
	test_failed_retry_ok();
	test_persistent_failed_gives_up();
	test_full_queue();
	test_diagnostic_never_displaces_critical();
	test_saturated_stream_never_blocks();
	test_tick_wraparound();

	if (host_test_failures == 0) {
		printf("All can_tx_queue checks passed.\n");
		return 0;
	}
	printf("\n%d can_tx_queue check(s) FAILED.\n", host_test_failures);
	return 1;
}
