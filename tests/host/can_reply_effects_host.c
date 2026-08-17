/*
 * FW-110 v4: host tests for src/can_reply_effects.c, the deferred 0x6029 diag_peak_reset side
 * effect - linked TOGETHER with the real src/can_multiframe.c (whose transfer-id/state this
 * module resolves against) and the real src/can_tx_queue.c underneath it, driven by the same
 * scriptable fake CAN peripheral can_multiframe_host.c uses.
 *
 * WHAT v4 REQUIRES of this module (and why): 0x6029's old handler did
 *
 *     if (send_multiframe(...)) { diag_peak_reset = 1; }
 *
 * send_multiframe() returning true only proves the snapshot was ARMED. If the transfer later
 * aborts (a fragment exhausts its retries in can_tx_queue), the requester never receives a
 * complete snapshot, but the peaks were already cleared - the data it asked for is gone. This
 * module must therefore emit the reset ONLY when the EXACT armed transfer resolves DONE, at
 * most once, and must drop the pending id (emit nothing) on ABORTED/UNKNOWN so a stale id can
 * never fire a reset for a reply that never completed.
 *
 * What each block checks:
 *   E1.     0x6029 armed and CONFIRMED complete -> poll() emits CANFX_EFFECT_DIAG_PEAK_RESET
 *            EXACTLY once, and a second poll() is NONE (at-most-once per armed reply).
 *   E2.     0x6029 armed then ABORTED (GIVEUP a middle DATA fragment) -> poll() is NONE forever,
 *            the pending id is dropped, and a LATER unrelated DONE transfer cannot fire it.
 *   E3.     PENDING (last fragment held) -> poll() is NONE; releasing to DONE -> exactly one reset.
 *   E4.     Armed, then the id is made unaccountable (UNKNOWN) -> poll() NONE, id dropped.
 *   E5.     A second 0x6029 arm replaces the pending one; only the LATEST reply's outcome matters.
 *   E6.     can_reply_effects_init() clears the pending id - a stale arm cannot survive a reset.
 *   E7.     Completion of the same transfer id from TWO sequential replies fires one reset EACH
 *            (the module is per-armed-reply, not a one-shot latch).
 */

#include "../common/check.h"

#include "can_multiframe.h"
#include "can_reply_effects.h"
#include "can_tx_queue.h"

#include <string.h>

/* --- fake CAN peripheral: same scriptable fake can_multiframe_host.c uses ------------------ */

#define CAP_MAX 160
static uint32_t cap_efid[CAP_MAX];
static int      cap_n;

#define TXLOG_MAX 256
static uint32_t txlog_efid[TXLOG_MAX];
static int      txlog_n;

static bool     stage_valid;
static uint32_t stage_efid;

#define SCRIPT_MAX 512
static uint8_t mock_state_script[SCRIPT_MAX];
static int     mock_state_step;

static uint8_t (*state_override)(uint8_t scripted);

static void mock_reset(void)
{
	cap_n = 0;
	txlog_n = 0;
	stage_valid = false;
	mock_state_step = 0;
	state_override = 0;
	for (int i = 0; i < SCRIPT_MAX; i++) mock_state_script[i] = CANQ_OK;
}

static uint8_t mock_transmit(uint32_t efid, uint8_t dlen, const uint8_t data[8])
{
	(void)dlen; (void)data;
	stage_valid = true;
	stage_efid = efid;
	if (txlog_n < TXLOG_MAX) txlog_efid[txlog_n++] = efid;
	return 1U; /* always a free mailbox */
}

static uint8_t mock_state(uint8_t mailbox)
{
	(void)mailbox;
	uint8_t v = mock_state_script[mock_state_step < SCRIPT_MAX ? mock_state_step : SCRIPT_MAX - 1];
	mock_state_step++;
	if (state_override) v = state_override(v);
	if (v == CANQ_OK && stage_valid && cap_n < CAP_MAX) {
		cap_efid[cap_n++] = stage_efid;
		stage_valid = false;
	} else if (v == CANQ_FAILED) {
		stage_valid = false;
	}
	return v;
}

static const can_tx_ops_t mock_ops = { mock_transmit, mock_state };

static int txlog_has(uint32_t efid)
{
	for (int i = 0; i < txlog_n; i++) if (txlog_efid[i] == efid) return 1;
	return 0;
}

#define OP_LONG_START 4U
#define OP_LONG_END   6U

/* low16 is the efid's command field: the REAL command on START (e.g. 0x6029), the fragment
 * index on DATA (0,1,2...) and on END (the DATA fragment count). See build_efid() in
 * src/can_multiframe.c. */
static uint32_t frag_efid(uint32_t low16, uint8_t op, uint8_t target, uint8_t source)
{
	return low16 + ((uint32_t)op << 16) + ((uint32_t)target << 19) + ((uint32_t)source << 24);
}

static void fill_pattern(uint8_t *buf, uint8_t len)
{
	for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)(i * 5U + 1U);
}

static void reset_all(void)
{
	mock_reset();
	can_tx_queue_init(&mock_ops);
	can_multiframe_init();
	can_reply_effects_init();
}

/* The 0x6029 snapshot is exactly 55 bytes -> 6 DATA fragments + 7-byte END. */
#define DG_LEN 55U

/* Drives both real modules for up to `limit` ticks (can_multiframe_step() then
 * can_tx_queue_service(), exactly as main.c's loop does) until the producer goes idle and the
 * queue drains. */
static int run_until_idle(uint32_t start_tick, int limit)
{
	int i;
	for (i = 0; i < limit; i++) {
		can_multiframe_step();
		can_tx_queue_service(start_tick + (uint32_t)i);
		if (!can_multiframe_busy() && can_tx_queue_depth() == 0U) break;
	}
	return i;
}

/* --- permanent-failure helpers -------------------------------------------------------------- */
static uint32_t giveup_efid;

static uint8_t fail_this_efid_override(uint8_t scripted)
{
	(void)scripted;
	return (stage_efid == giveup_efid) ? CANQ_FAILED : CANQ_OK;
}

/* --- "hold the last fragment pending until released" helpers ------------------------------- */
static bool     release_pending;
static uint32_t pending_efid;

static uint8_t pending_until_release_override(uint8_t scripted)
{
	if (!release_pending && stage_efid == pending_efid) return CANQ_PENDING;
	return scripted;
}

int main(void)
{
	printf("FW-110 v4 can_reply_effects.c deferred 0x6029 peak reset, against the shipped modules\n");

	/* --- E1: confirmed DONE -> reset emitted EXACTLY once ---------------------------------- */
	{
		reset_all();
		uint8_t dg[DG_LEN];
		fill_pattern(dg, DG_LEN);

		can_multiframe_id_t xid;
		CHECK(can_multiframe_start(0x6029U, 5U, 0x02U, dg, DG_LEN, &xid), "E1: 0x6029 armed");
		CHECK(xid != CANMF_ID_NONE, "E1: a real transfer id was handed out");
		can_reply_effects_6029_armed(xid);

		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE, "E1: no effect while still PENDING");
		int used = run_until_idle(0U, 300);
		CHECK(used < 300, "E1: completes");
		CHECK(can_multiframe_transfer_state(xid) == CANMF_XFER_DONE, "E1: transfer resolved DONE");

		CHECK(can_reply_effects_poll() == CANFX_EFFECT_DIAG_PEAK_RESET,
			"E1: poll() emits the reset exactly when the DONE transfer is resolved");
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE,
			"E1: second poll() is NONE - the effect fired at most once");
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE,
			"E1: still NONE after more polls - not a recurring latch");
	}

	/* --- E2: ABORTED -> NONE forever, and a later DONE transfer cannot fire the stale id ---- */
	{
		reset_all();
		uint8_t dg[DG_LEN];
		fill_pattern(dg, DG_LEN);

		giveup_efid = frag_efid(3U, OP_LONG_START + 1U, 5U, 0x02U); /* DATA fragment index 3 */
		state_override = fail_this_efid_override;

		can_multiframe_id_t xid;
		CHECK(can_multiframe_start(0x6029U, 5U, 0x02U, dg, DG_LEN, &xid), "E2: 0x6029 armed");
		can_reply_effects_6029_armed(xid);

		int used = run_until_idle(0U, 500);
		CHECK(used < 500, "E2: the give-up resolves in bounded time");
		CHECK(can_multiframe_transfer_state(xid) == CANMF_XFER_ABORTED,
			"E2: transfer resolved ABORTED");

		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE,
			"E2: aborted transfer emits NO reset - the peaks must be kept");
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE,
			"E2: still NONE after repeated polls - the pending id was dropped");

		/* Now run a completely unrelated DONE transfer. The stale id must NOT suddenly fire. */
		uint8_t other[8] = { 9, 9, 9, 9, 9, 9, 9, 9 };
		CHECK(can_multiframe_start(0x6011U, 5U, 0x02U, other, 8U, 0), "E2: unrelated reply armed");
		run_until_idle(0U, 100);
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE,
			"E2: an unrelated DONE transfer cannot resurrect the dropped aborted id");
	}

	/* --- E3: PENDING -> NONE; released to DONE -> exactly one reset ------------------------- */
	{
		reset_all();
		uint8_t dg[DG_LEN];
		fill_pattern(dg, DG_LEN);

		uint32_t end_efid = frag_efid(6U, OP_LONG_END, 5U, 0x02U); /* 55 B -> 6 DATA -> END idx 6 */
		pending_efid = end_efid;
		release_pending = false;
		state_override = pending_until_release_override;

		can_multiframe_id_t xid;
		CHECK(can_multiframe_start(0x6029U, 5U, 0x02U, dg, DG_LEN, &xid), "E3: armed");
		can_reply_effects_6029_armed(xid);

		uint32_t tick = 0U;
		int i;
		for (i = 0; i < 400 && !txlog_has(end_efid) && can_multiframe_busy(); i++) {
			can_multiframe_step();
			can_tx_queue_service(tick++);
		}
		CHECK(txlog_has(end_efid), "E3: END transmitted (held PENDING)");
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE,
			"E3: NONE while the last fragment is PENDING - completion is not assumed from the arm");
		for (int j = 0; j < 20; j++) {
			can_multiframe_step();
			can_tx_queue_service(tick++);
		}
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE,
			"E3: still NONE across many PENDING polls - the reset never fires early");

		release_pending = true;
		run_until_idle(tick, 200);
		(void)i;
		CHECK(can_multiframe_transfer_state(xid) == CANMF_XFER_DONE, "E3: DONE after release");
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_DIAG_PEAK_RESET,
			"E3: reset emitted exactly once after the confirmed DONE");
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE, "E3: and never again");
	}

	/* --- E4: UNKNOWN id -> NONE, id dropped ------------------------------------------------- */
	{
		reset_all();
		/* Arm with an id that is NOT a real transfer (CANMF_ID_NONE). can_multiframe_transfer_state
		 * must resolve it UNKNOWN, so the reset must never fire and the pending must be dropped. */
		can_reply_effects_6029_armed(CANMF_ID_NONE);
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE,
			"E4: CANMF_ID_NONE -> UNKNOWN -> NONE, never a reset");
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE,
			"E4: the UNKNOWN pending id was dropped - no later fire");

		/* A fabricated id that no transfer was ever given (e.g. a wild uint16) must also be NONE. */
		can_reply_effects_6029_armed((can_multiframe_id_t)0x1234U);
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE,
			"E4: a fabricated id resolves UNKNOWN -> NONE");
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE, "E4: and is dropped");
	}

	/* --- E5: a new arm replaces the pending one; only the LATEST armed id matters ----------- */
	{
		/* Sub-test A: arm a real id that resolves DONE, then replace it with CANMF_ID_NONE. The
		 * replacement must DROP the previous pending id - if the module still tracked id_a, a
		 * poll would now fire the reset for a reply that was superseded. It must not. */
		reset_all();
		uint8_t dg[DG_LEN];
		fill_pattern(dg, DG_LEN);
		can_multiframe_id_t id_a;
		CHECK(can_multiframe_start(0x6029U, 5U, 0x02U, dg, DG_LEN, &id_a), "E5A: armed");
		run_until_idle(0U, 300);
		CHECK(can_multiframe_transfer_state(id_a) == CANMF_XFER_DONE, "E5A: id_a resolves DONE");

		can_reply_effects_6029_armed(id_a);
		can_reply_effects_6029_armed(CANMF_ID_NONE);   /* a new arm REPLACES the pending one */
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE,
			"E5A: the newer arm (invalid id) replaced the pending DONE id - no reset for the superseded reply");
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE, "E5A: and the invalid id was dropped");

		/* Sub-test B: two real sequential transfers. Arm B while A is still pending is impossible
		 * (the producer owns one reply at a time), so the meaningful replacement test is arming a
		 * stale-but-real DONE id and then a CURRENT pending transfer's id: only the current one
		 * may fire. */
		reset_all();
		can_multiframe_id_t stale;
		CHECK(can_multiframe_start(0x6011U, 5U, 0x02U, dg, DG_LEN, &stale), "E5B: stale reply armed");
		run_until_idle(0U, 300);
		CHECK(can_multiframe_transfer_state(stale) == CANMF_XFER_DONE, "E5B: stale resolves DONE");

		can_multiframe_id_t current;
		CHECK(can_multiframe_start(0x6029U, 5U, 0x02U, dg, DG_LEN, &current), "E5B: current armed");
		can_reply_effects_6029_armed(stale);     /* arm the stale id first */
		can_reply_effects_6029_armed(current);   /* then the CURRENT transfer's id - replaces */
		CHECK(can_multiframe_transfer_state(current) == CANMF_XFER_PENDING,
			"E5B: current is still PENDING at arm time (producer busy)");

		run_until_idle(0U, 300);
		CHECK(can_multiframe_transfer_state(current) == CANMF_XFER_DONE, "E5B: current completes");
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_DIAG_PEAK_RESET,
			"E5B: only the CURRENT (last-armed) reply's DONE fires the reset - the stale id was replaced");
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE, "E5B: and only once");
	}

	/* --- E6: init() clears the pending id - a stale arm cannot survive a reset --------------- */
	{
		reset_all();
		uint8_t dg[DG_LEN];
		fill_pattern(dg, DG_LEN);

		can_multiframe_id_t xid;
		CHECK(can_multiframe_start(0x6029U, 5U, 0x02U, dg, DG_LEN, &xid), "E6: armed");
		can_reply_effects_6029_armed(xid);
		can_reply_effects_init();   /* e.g. a controller reset while a transfer was pending */

		run_until_idle(0U, 300);
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE,
			"E6: init() cleared the pending id - the later DONE emits nothing");
	}

	/* --- E7: per-armed-reply - TWO sequential replies each fire their own reset -------------- */
	{
		reset_all();
		uint8_t dg[DG_LEN];
		fill_pattern(dg, DG_LEN);

		can_multiframe_id_t xid1;
		CHECK(can_multiframe_start(0x6029U, 5U, 0x02U, dg, DG_LEN, &xid1), "E7: reply 1 armed");
		can_reply_effects_6029_armed(xid1);
		run_until_idle(0U, 300);
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_DIAG_PEAK_RESET,
			"E7: reply 1's DONE fires the reset");
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE, "E7: reply 1 consumed");

		can_multiframe_id_t xid2;
		CHECK(can_multiframe_start(0x6029U, 5U, 0x02U, dg, DG_LEN, &xid2), "E7: reply 2 armed");
		can_reply_effects_6029_armed(xid2);
		run_until_idle(0U, 300);
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_DIAG_PEAK_RESET,
			"E7: reply 2's DONE fires the reset too - the module is per-armed-reply, not a one-shot");
		CHECK(can_reply_effects_poll() == CANFX_EFFECT_NONE, "E7: reply 2 consumed");
	}

	if (host_test_failures == 0) {
		printf("All can_reply_effects checks passed.\n");
		return 0;
	}
	printf("\n%d can_reply_effects check(s) FAILED.\n", host_test_failures);
	return 1;
}