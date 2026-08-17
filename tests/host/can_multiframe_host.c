/*
 * FW-110 v4: host tests for src/can_multiframe.c, the stop-and-wait multiframe
 * (START/DATA/END[/trailer]) producer - against the REAL, shipped module, linked TOGETHER with
 * the real src/can_tx_queue.c it feeds (can_multiframe.c has no ops of its own; it calls
 * can_tx_queue_enqueue_tracked()/can_tx_queue_token_state() directly), driven by a scriptable
 * fake CAN peripheral underneath can_tx_queue.
 *
 * WHAT v4 CHANGED (and why these tests exist):
 *   v1 dropped the last 17 fragments of a 255-byte reply deterministically (33 fragments against
 *   a 16-frame queue, all offered back to back in one call).
 *   v2 (reported here as "v3" in older text) fixed the drop by advancing the producer's cursor on
 *   enqueue()==true. That is still wrong: enqueue()==true only means "accepted into the FIFO",
 *   nothing about transmission. A fragment that later exhausts its retries and is given up in
 *   can_tx_queue (CANQ_TOKEN_FAILED), or whose token can no longer be accounted for
 *   (CANQ_TOKEN_UNKNOWN), was silently skipped: the producer kept emitting the following DATA,
 *   END and trailer, so the client got a structurally complete but logically corrupted reply.
 *   v4 rebuilds the module so the cursor advances ONLY on a confirmed CANQ_TOKEN_DONE for the
 *   exact fragment in flight, and FAILED/UNKNOWN aborts the WHOLE transfer - no later DATA, no
 *   END, no trailer.
 *
 * What each block checks:
 *   A (1-7). exact fragment count/shape for every required length (1, 8, 9, 55, 56, 64, 255),
 *            byte-exact payload reconstruction, dropped_count stays 0, and the producer never
 *            occupies more than ONE slot of the 16-frame queue at any instant.
 *   A-8.     transfer ends ONLY after the last fragment's CAN_TRANSMIT_OK - the last fragment is
 *            held PENDING for many ticks and the transfer demonstrably does not complete (and is
 *            not aborted) until it is released.
 *   B-1..4.  permanent (GIVEUP) errors: START gives up -> nothing follows; a middle DATA gives up
 *            -> no later DATA/END/trailer; END gives up -> trailer never goes out; the trailer
 *            itself gives up -> the transfer is reported FAILED (not completed).
 *   B-5.     an UNKNOWN token (aged out of can_tx_queue's history) aborts safely - no further
 *            fragments.
 *   C.       HMI heartbeat/0x3202/poll frames inserted DURING an active 255-byte 0x6020 reply
 *            all reach the wire; none is refused because the multiframe occupied the queue.
 *   D.       0x6012-style trailer: START->DATA...->END->trailer exactly once, trailer only after
 *            END's CAN_TRANSMIT_OK, under NOMAILBOX/PENDING/FAILED->retry->OK, plus GIVEUP END
 *            and GIVEUP trailer.
 *   9/10.    a second start() while one is active is refused (first reply unharmed); busy() is
 *            true exactly while a reply is in progress.
 */

#include "../common/check.h"

#include "can_multiframe.h"
#include "can_tx_queue.h"

#include <string.h>

/* Test seam declared by src/can_multiframe.c only when it is built with CANMF_REFUSAL_HOOK
 * (compiled out of the firmware - see run-host-tests.ps1). Lets this harness force the
 * producer's defensive enqueue-refusal abort path, which is otherwise unreachable
 * single-threaded because of the free-slot pre-check. */
#ifdef CANMF_REFUSAL_HOOK
void can_multiframe_test_refuse_next_enqueue(void);
#endif

/* --- fake CAN peripheral: scriptable, captures every frame ACTUALLY DELIVERED (state()==OK) --
 * A test hook (state_override) lets a test force PENDING/FAILED for a SPECIFIC staged efid (the
 * GIVEUP and "last fragment" scenarios), which is what makes permanent-loss tests deterministic
 * without hand-counting script indexes. */

#define CAP_MAX 160
static uint32_t cap_efid[CAP_MAX];
static uint8_t  cap_dlen[CAP_MAX];
static uint8_t  cap_data[CAP_MAX][8];
static int      cap_n;

#define TXLOG_MAX 256
static uint32_t txlog_efid[TXLOG_MAX];
static int      txlog_n;

static bool     stage_valid;
static uint32_t stage_efid;
static uint8_t  stage_dlen;
static uint8_t  stage_data[8];

#define SCRIPT_MAX 512
static uint8_t mock_transmit_script[SCRIPT_MAX]; /* CANQ_NOMAILBOX or a fake mailbox id */
static uint8_t mock_state_script[SCRIPT_MAX];    /* CANQ_OK/PENDING/FAILED */
static int mock_transmit_step, mock_state_step;

/* Called for each state() poll with the scripted base value; may override it (typically by
 * inspecting stage_efid). */
static uint8_t (*state_override)(uint8_t scripted);

static void mock_reset(void)
{
	cap_n = 0;
	txlog_n = 0;
	stage_valid = false;
	mock_transmit_step = 0;
	mock_state_step = 0;
	state_override = 0;
	for (int i = 0; i < SCRIPT_MAX; i++) {
		mock_transmit_script[i] = 1U;    /* default: always a free mailbox */
		mock_state_script[i] = CANQ_OK;  /* default: always completes immediately */
	}
}

static uint8_t mock_transmit(uint32_t efid, uint8_t dlen, const uint8_t data[8])
{
	uint8_t v = mock_transmit_script[mock_transmit_step < SCRIPT_MAX ? mock_transmit_step : SCRIPT_MAX - 1];
	mock_transmit_step++;
	if (v != CANQ_NOMAILBOX) {
		stage_valid = true;
		stage_efid = efid;
		stage_dlen = dlen;
		memcpy(stage_data, data, 8);
		if (txlog_n < TXLOG_MAX) txlog_efid[txlog_n++] = efid;
	}
	return v;
}

static uint8_t mock_state(uint8_t mailbox)
{
	(void)mailbox;
	uint8_t v = mock_state_script[mock_state_step < SCRIPT_MAX ? mock_state_step : SCRIPT_MAX - 1];
	mock_state_step++;
	if (state_override) v = state_override(v);
	if (v == CANQ_OK && stage_valid && cap_n < CAP_MAX) {
		cap_efid[cap_n] = stage_efid;
		cap_dlen[cap_n] = stage_dlen;
		memcpy(cap_data[cap_n], stage_data, 8);
		cap_n++;
		stage_valid = false;
	} else if (v == CANQ_FAILED) {
		stage_valid = false; /* discarded - a retry will stage (and may later commit) it again */
	}
	return v;
}

static const can_tx_ops_t mock_ops = { mock_transmit, mock_state };

static int txlog_has(uint32_t efid)
{
	for (int i = 0; i < txlog_n; i++) if (txlog_efid[i] == efid) return 1;
	return 0;
}

/* --- efid decomposition + builder, matching documentation/CAN_PROTOCOL_REFERENCE.md --------- */
#define OP_LONG_START 4U
#define OP_LONG_TRANG 5U
#define OP_LONG_END   6U

static uint16_t efid_command(uint32_t efid) { return (uint16_t)(efid & 0xFFFFU); }
static uint8_t  efid_op(uint32_t efid)      { return (uint8_t)((efid >> 16) & 0x07U); }
static uint8_t  efid_target(uint32_t efid)  { return (uint8_t)((efid >> 19) & 0x1FU); }
static uint8_t  efid_source(uint32_t efid)  { return (uint8_t)((efid >> 24) & 0x1FU); }

/* low16 is the efid's command field: the REAL command on START (e.g. 0x6011), the fragment
 * index on DATA (0,1,2...) and on END (the DATA fragment count). See build_efid() in
 * src/can_multiframe.c. */
static uint32_t frag_efid(uint32_t low16, uint8_t op, uint8_t target, uint8_t source)
{
	return low16 + ((uint32_t)op << 16) + ((uint32_t)target << 19) + ((uint32_t)source << 24);
}

/* Drives BOTH real modules for up to `limit` ticks, exactly as main.c's loop does each tick
 * (can_multiframe_step() first, then can_tx_queue_service()), stopping once the producer goes
 * idle and the queue drains. Tracks the maximum queue depth seen (the "at most one producer
 * fragment" proof). Returns the number of ticks actually used. */
static int run_until_idle(uint32_t start_tick, int limit, int *max_depth)
{
	int i;
	int md = 0;
	for (i = 0; i < limit; i++) {
		can_multiframe_step();
		can_tx_queue_service(start_tick + (uint32_t)i);
		int d = (int)can_tx_queue_depth();
		if (d > md) md = d;
		if (!can_multiframe_busy() && can_tx_queue_depth() == 0U) break;
	}
	if (max_depth) *max_depth = md;
	return i;
}

/* Reconstructs the payload from the captured frame log (frames [from, cap_n)) into `out`,
 * verifying START/DATA/END shape, command/op/target/source on every frame, and returns the
 * reconstructed length (or -1 on a structural mismatch, which CHECK()s along the way).
 * Frames that are NOT part of this transfer (interleaved HMI traffic: different target/source)
 * are skipped, so the interleave test can use the same helper. */
static int reconstruct(int from, uint16_t expect_command, uint8_t expect_target,
                        uint8_t expect_source, uint8_t out[256])
{
	int idx = from;

	while (idx < cap_n) {
		if (efid_op(cap_efid[idx]) == OP_LONG_START &&
		    efid_target(cap_efid[idx]) == expect_target &&
		    efid_source(cap_efid[idx]) == expect_source) break;
		idx++;
	}
	if (idx >= cap_n) { CHECK(false, "reconstruct: no START frame captured"); return -1; }

	CHECK(efid_command(cap_efid[idx]) == expect_command, "reconstruct: START carries the real command");
	CHECK(cap_dlen[idx] == 1U, "reconstruct: START is a single length byte");
	uint8_t length = cap_data[idx][0];
	idx++;

	int out_len = 0;
	uint8_t next_data_frag = 0;
	while (idx < cap_n) {
		if (efid_op(cap_efid[idx]) == OP_LONG_TRANG &&
		    efid_target(cap_efid[idx]) == expect_target &&
		    efid_source(cap_efid[idx]) == expect_source) {
			CHECK(efid_command(cap_efid[idx]) == next_data_frag, "reconstruct: DATA fragment index in order");
			CHECK(cap_dlen[idx] == 8U, "reconstruct: DATA fragment is always 8 bytes");
			memcpy(out + out_len, cap_data[idx], 8);
			out_len += 8;
			next_data_frag++;
			idx++;
		} else if (efid_op(cap_efid[idx]) == OP_LONG_END &&
		           efid_target(cap_efid[idx]) == expect_target &&
		           efid_source(cap_efid[idx]) == expect_source) {
			CHECK(efid_command(cap_efid[idx]) == next_data_frag, "reconstruct: END's command is the DATA fragment count");
			memcpy(out + out_len, cap_data[idx], cap_dlen[idx]);
			out_len += cap_dlen[idx];
			idx++;
			break;
		} else {
			idx++;   /* interleaved HMI frame - skip */
		}
	}

	CHECK(out_len == length, "reconstruct: total reconstructed length matches the announced length");
	/* No further frames belonging to THIS transfer may follow END. */
	while (idx < cap_n) {
		CHECK(!(efid_op(cap_efid[idx]) == OP_LONG_TRANG ||
		        efid_op(cap_efid[idx]) == OP_LONG_END ||
		        efid_op(cap_efid[idx]) == OP_LONG_START) ||
		      efid_target(cap_efid[idx]) != expect_target ||
		      efid_source(cap_efid[idx]) != expect_source,
			"reconstruct: no extra frames after END");
		idx++;
	}
	return out_len;
}

static void fill_pattern(uint8_t *buf, uint8_t len)
{
	for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)(i * 7U + 3U);
}

static void reset_all(void)
{
	mock_reset();
	can_tx_queue_init(&mock_ops);
	can_multiframe_init();
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
	printf("FW-110 v4 can_multiframe.c stop-and-wait producer, against the shipped module + real can_tx_queue\n");

	/* --- A (1-7): exact shape for every required length ---------------------------------- */
	{
		struct { uint8_t length; uint8_t data_frames; const char *label; } lens[] = {
			{ 1U, 0U,  "A1: length=1 (0 DATA fragments, 2 frames total)" },
			{ 8U, 0U,  "A2: length=8 (0 DATA fragments, END carries all 8 bytes)" },
			{ 9U, 1U,  "A3: length=9 (1 DATA fragment + 1-byte END)" },
			{ 55U, 6U, "A4: length=55 (6 DATA fragments + 7-byte END)" },
			{ 56U, 6U, "A5: length=56 (6 DATA fragments + END carries a full 8 bytes)" },
			{ 64U, 7U, "A6: length=64 (7 DATA fragments + END carries a full 8 bytes)" },
			{ 255U, 31U, "A7: length=255 - the v1 bug (33 frames: START + 31 DATA + END)" },
		};
		for (int t = 0; t < (int)(sizeof(lens)/sizeof(lens[0])); t++) {
			reset_all();
			uint8_t payload[256];
			fill_pattern(payload, lens[t].length);
			can_multiframe_id_t xid;
			CHECK(can_multiframe_start(0x1234U, 5U, 0x02U, payload, lens[t].length, &xid),
				lens[t].label);
			CHECK(xid != CANMF_ID_NONE, lens[t].label);

			int md = 0;
			int used = run_until_idle(0U, 300, &md);
			CHECK(used < 300, lens[t].label);
			CHECK(!can_multiframe_busy(), lens[t].label);
			CHECK(can_tx_queue_depth() == 0U, lens[t].label);
			CHECK(can_tx_queue_dropped_count() == 0U, lens[t].label);
			CHECK(md <= 1, lens[t].label);
			CHECK(can_multiframe_completed_count() == 1U, lens[t].label);
			CHECK(can_multiframe_aborted_count() == 0U, lens[t].label);

			int expect_frames = 1 + (int)lens[t].data_frames + 1;
			CHECK(cap_n == expect_frames, lens[t].label);

			uint8_t out[256] = { 0 };
			int out_len = reconstruct(0, 0x1234U, 5U, 0x02U, out);
			CHECK(out_len == lens[t].length, lens[t].label);
			CHECK(memcmp(out, payload, lens[t].length) == 0, lens[t].label);
		}
	}

	/* --- A-8: transfer ends ONLY after the last fragment's CAN_TRANSMIT_OK ---------------- */
	{
		reset_all();
		uint8_t bank[255];
		fill_pattern(bank, 255U);
		uint32_t end_efid = frag_efid(31U, OP_LONG_END, 5U, 0x02U); /* command field = 31 */

		pending_efid = end_efid;
		release_pending = false;
		state_override = pending_until_release_override;

		CHECK(can_multiframe_start(0x6020U, 5U, 0x02U, bank, 255U, 0),
			"A8: 0x6020 reply armed");
		uint32_t tick = 0U;
		int i = 0;
		for (i = 0; i < 400 && !txlog_has(end_efid) && can_multiframe_busy(); i++) {
			can_multiframe_step();
			can_tx_queue_service(tick++);
		}
		CHECK(txlog_has(end_efid), "A8: the last (END) fragment was transmitted");
		CHECK(can_multiframe_busy(), "A8: still busy while the last fragment is PENDING");
		CHECK(can_multiframe_completed_count() == 0U,
			"A8: NOT completed - the last fragment has not reached CAN_TRANSMIT_OK yet");

		for (int j = 0; j < 60; j++) {
			can_multiframe_step();
			can_tx_queue_service(tick++);
		}
		CHECK(can_multiframe_busy(), "A8: still busy after many PENDING polls of the last fragment");
		CHECK(can_multiframe_completed_count() == 0U,
			"A8: still NOT completed - the cursor must not advance on enqueue alone");
		CHECK(can_multiframe_aborted_count() == 0U, "A8: PENDING is not failure - no abort either");

		release_pending = true;   /* the wire clears; the next poll sees CAN_TRANSMIT_OK */
		int md = 0;
		run_until_idle(tick, 200, &md);
		CHECK(can_multiframe_completed_count() == 1U,
			"A8: completed EXACTLY once, and only after the last fragment's CAN_TRANSMIT_OK");
		CHECK(can_multiframe_aborted_count() == 0U, "A8: no abort on the happy completion");
		CHECK(cap_n == 33, "A8: exactly 33 frames reached the wire");
		uint8_t out[256] = { 0 };
		int out_len = reconstruct(0, 0x6020U, 5U, 0x02U, out);
		CHECK(out_len == 255 && memcmp(out, bank, 255) == 0,
			"A8: the 255 B payload is intact end to end");
	}

	/* --- B-1: GIVEUP START -> nothing follows ---------------------------------------------- */
	{
		reset_all();
		uint8_t payload[64];
		fill_pattern(payload, 64U);
		giveup_efid = frag_efid(0x6011U, OP_LONG_START, 5U, 0x02U);
		state_override = fail_this_efid_override;   /* START fails forever, everything else OK */

		can_multiframe_id_t xid;
		CHECK(can_multiframe_start(0x6011U, 5U, 0x02U, payload, 64U, &xid), "B1: armed");
		int md = 0;
		int used = run_until_idle(0U, 400, &md);
		CHECK(used < 400, "B1: the give-up resolves in bounded time");
		CHECK(!can_multiframe_busy(), "B1: producer back to idle");
		CHECK(can_multiframe_aborted_count() == 1U, "B1: transfer ended ABORTED");
		CHECK(can_multiframe_completed_count() == 0U, "B1: NOT completed");
		CHECK(can_multiframe_failed_fragment_count() == 1U, "B1: one failed fragment counted");
		CHECK(cap_n == 0, "B1: ZERO frames on the wire - START never succeeded, so no DATA/END");
	}

	/* --- B-2: GIVEUP a middle DATA -> no later DATA / no END / no trailer ------------------ */
	{
		reset_all();
		uint8_t payload[64];
		fill_pattern(payload, 64U);
		giveup_efid = frag_efid(3U, OP_LONG_TRANG, 5U, 0x02U); /* DATA fragment index 3 */
		state_override = fail_this_efid_override;

		CHECK(can_multiframe_start(0x6011U, 5U, 0x02U, payload, 64U, 0), "B2: armed");
		int md = 0;
		int used = run_until_idle(0U, 500, &md);
		CHECK(used < 500, "B2: bounded");
		CHECK(can_multiframe_aborted_count() == 1U, "B2: transfer ended ABORTED");
		CHECK(can_multiframe_completed_count() == 0U, "B2: NOT completed");
		/* 64 B -> 7 DATA fragments: START(1) + DATA 0,1,2 (3) succeeded before DATA 3 gave up. */
		CHECK(cap_n == 4, "B2: exactly START + DATA 0..2 on the wire - no later DATA, no END");
		CHECK(efid_op(cap_efid[cap_n - 1]) == OP_LONG_TRANG, "B2: the last captured frame is a DATA, never END");
	}

	/* --- B-3: GIVEUP END -> trailer never goes out ----------------------------------------- */
	{
		reset_all();
		uint8_t payload[64];
		fill_pattern(payload, 64U);
		uint32_t end_efid = frag_efid(7U, OP_LONG_END, 5U, 0x02U); /* 64 B -> 7 DATA frags -> END idx 7 */
		giveup_efid = end_efid;
		state_override = fail_this_efid_override;

		can_multiframe_trailer_t trailer;
		trailer.efid = 0x6011U + (3U << 16) + (5U << 19) + (0x02U << 24);
		trailer.dlen = 4U;
		memset(trailer.data, 0, 8);
		trailer.data[0] = 0x01; trailer.data[1] = 0x00; trailer.data[2] = 0x02; trailer.data[3] = 0x06;

		CHECK(can_multiframe_start_with_trailer(0x6011U, 5U, 0x02U, payload, 64U, &trailer, 0),
			"B3: armed with trailer");
		int md = 0;
		int used = run_until_idle(0U, 600, &md);
		CHECK(used < 600, "B3: bounded");
		CHECK(can_multiframe_aborted_count() == 1U, "B3: transfer ended ABORTED");
		CHECK(can_multiframe_completed_count() == 0U, "B3: NOT completed");
		/* START(1) + 7 DATA = 8 frames captured; END and trailer must NOT be there. */
		CHECK(cap_n == 8, "B3: 8 frames (START + 7 DATA) on the wire - no END, no trailer");
		CHECK(efid_op(cap_efid[cap_n - 1]) == OP_LONG_TRANG, "B3: the last captured frame is DATA, never END/trailer");
	}

	/* --- B-4: GIVEUP trailer -> transfer FAILED, not reported completed --------------------- */
	{
		reset_all();
		uint8_t payload[64];
		fill_pattern(payload, 64U);
		uint32_t trailer_efid = 0x6011U + (3U << 16) + (5U << 19) + (0x02U << 24);
		giveup_efid = trailer_efid;
		state_override = fail_this_efid_override;

		can_multiframe_trailer_t trailer;
		trailer.efid = trailer_efid;
		trailer.dlen = 4U;
		memset(trailer.data, 0, 8);
		trailer.data[0] = 0x01; trailer.data[1] = 0x00; trailer.data[2] = 0x02; trailer.data[3] = 0x06;

		CHECK(can_multiframe_start_with_trailer(0x6011U, 5U, 0x02U, payload, 64U, &trailer, 0),
			"B4: armed with trailer");
		int md = 0;
		int used = run_until_idle(0U, 700, &md);
		CHECK(used < 700, "B4: bounded");
		CHECK(can_multiframe_aborted_count() == 1U, "B4: transfer ended ABORTED - the trailer failed");
		CHECK(can_multiframe_completed_count() == 0U,
			"B4: NOT reported completed - the trailer never reached CAN_TRANSMIT_OK");
		CHECK(can_multiframe_failed_fragment_count() == 1U, "B4: one failed fragment counted");
		/* START(1) + 7 DATA + END(1) = 9 frames captured; the trailer itself is absent. */
		CHECK(cap_n == 9, "B4: 9 frames (START + 7 DATA + END) - the trailer never went out");
		CHECK(efid_op(cap_efid[cap_n - 1]) == OP_LONG_END, "B4: the last captured frame is END, never the trailer");
	}

	/* --- B-5: UNKNOWN token -> safe abort, no further fragments ----------------------------- */
	{
		/*
		 * Drive the real can_tx_queue so that the multiframe's first fragment resolves DONE and
		 * is then pushed OUT of the queue's 8-entry resolution history before can_multiframe
		 * ever gets stepped again. When it finally reads the token, it must get UNKNOWN (not
		 * DONE, not FAILED) and abort the whole transfer rather than assume success.
		 */
		reset_all();
		uint8_t payload[64];
		fill_pattern(payload, 64U);
		CHECK(can_multiframe_start(0x6011U, 5U, 0x02U, payload, 64U, 0), "B5: armed");

		/* Tick until the START fragment is transmitted (staged). */
		uint32_t tick = 0U;
		int i;
		for (i = 0; i < 50 && !txlog_has(frag_efid(0x6011U, OP_LONG_START, 5U, 0x02U)); i++) {
			can_multiframe_step();
			can_tx_queue_service(tick++);
		}
		CHECK(txlog_has(frag_efid(0x6011U, OP_LONG_START, 5U, 0x02U)),
			"B5: START fragment was transmitted");
		/* Now resolve START (one more service() -> OK) and, WITHOUT stepping can_multiframe,
		 * resolve 9 unrelated frames so the START token rotates out of the 8-entry history.
		 * Each service() call performs at most ONE queue op (transmit the head OR poll the
		 * in-flight frame), so drain until the queue is empty - that is 9 more resolutions,
		 * enough to push the START token out of the 8-entry history. */
		can_tx_queue_service(tick++);   /* resolves START (OK) */
		uint8_t filler[8] = { 0xAA, 0xBB, 0xCC, 0xDD, 0, 0, 0, 0 };
		for (int f = 0; f < 9; f++) CHECK(can_tx_queue_enqueue(0x1000U + (uint32_t)f, 1U, filler),
			"B5: filler queued behind START");
		for (int s = 0; s < 40 && can_tx_queue_depth() > 0U; s++) can_tx_queue_service(tick++);
		CHECK(can_tx_queue_depth() == 0U, "B5: all 9 fillers transmitted and resolved");

		/* Now step the producer: the START token is no longer in history nor in the queue. */
		can_multiframe_step();
		/* The abort is recorded and the producer returns to idle on the next step. */
		int md = 0;
		run_until_idle(tick, 50, &md);
		CHECK(can_multiframe_aborted_count() == 1U,
			"B5: UNKNOWN token -> transfer aborted, never assumed DONE");
		CHECK(can_multiframe_completed_count() == 0U, "B5: NOT completed");
		CHECK(can_multiframe_failed_fragment_count() == 1U, "B5: one failed fragment counted");
		int mf_cap = 0;
		for (int c = 0; c < cap_n; c++) {
			if (efid_target(cap_efid[c]) == 5U && efid_source(cap_efid[c]) == 0x02U) mf_cap++;
		}
		CHECK(mf_cap == 1, "B5: only START ever reached the wire - no further fragments");
	}

	/* --- C: HMI heartbeat/0x3202/poll frames during an active 255-byte 0x6020 reply ---------- */
	{
		reset_all();
		uint8_t bank[255];
		fill_pattern(bank, 255U);
		CHECK(can_multiframe_start(0x6020U, 5U, 0x02U, bank, 255U, 0), "C: 0x6020 reply armed");

		int hmi_inserted = 0;
		int hmi_during_active = 0;
		uint8_t hb1[8] = { 0 }, hb2[8] = { 0 }, hb3[8] = { 0 }, s202[8] = { 0 }, p01[8] = { 0 };
		uint32_t tick = 0U;
		int i;
		for (i = 0; i < 3000 && (can_multiframe_busy() || can_tx_queue_depth() > 0U); i++) {
			can_multiframe_step();
			if ((i % 16) == 4) {
				/* 3-frame heartbeat + 0x3202 + 0x3201 poll - the display's periodic traffic.
				 * Insertion stays comfortably below the queue's drain rate (each service()
				 * call does at most one op), so nothing is ever dropped. */
				if (can_multiframe_busy()) hmi_during_active++;
				can_tx_queue_enqueue(0x02FF1200U, 1U, hb1);
				can_tx_queue_enqueue(0x02F8320FU, 8U, hb2);
				can_tx_queue_enqueue(0x02F83000U, 4U, hb3);
				can_tx_queue_enqueue(0x02F83202U, 1U, s202);
				can_tx_queue_enqueue(0x02F83201U, 8U, p01);
				hmi_inserted += 5;
			}
			can_tx_queue_service(tick++);
		}
		CHECK(i < 3000, "C: the whole exchange finished within the bound");
		CHECK(hmi_during_active > 0, "C: HMI frames were inserted while the multiframe was active");
		CHECK(can_tx_queue_dropped_count() == 0U,
			"C: NOTHING dropped - the multiframe never filled the queue (<=1 slot), so every HMI frame was accepted");
		CHECK(cap_n == 33 + hmi_inserted,
			"C: all 33 multiframe frames AND every one of the inserted HMI frames reached the wire");

		/* Count the captured HMI frames to prove they are all there, not just the total. */
		int hmi_cap = 0;
		for (int c = 0; c < cap_n; c++) {
			if (efid_command(cap_efid[c]) == 0x1200U || efid_command(cap_efid[c]) == 0x320F ||
			    efid_command(cap_efid[c]) == 0x3000U || efid_command(cap_efid[c]) == 0x3202U ||
			    efid_command(cap_efid[c]) == 0x3201U) hmi_cap++;
		}
		CHECK(hmi_cap == hmi_inserted, "C: exactly the inserted HMI frames are captured, no more");

		uint8_t out[256] = { 0 };
		int out_len = reconstruct(0, 0x6020U, 5U, 0x02U, out);
		(void)out_len; /* reconstruct() already CHECKed the structure */
		CHECK(memcmp(out, bank, 255) == 0,
			"C: the 0x6020 payload is still intact - interleaved HMI traffic did not corrupt it");
	}

	/* --- D: 0x6012-style trailer - part of the same transfer, only after END's CAN_TRANSMIT_OK */
	{
		uint8_t payload[64];
		fill_pattern(payload, 64U);
		uint8_t trailer_data[8] = { 0x01, 0x00, 0x02, 0x06, 0, 0, 0, 0 };
		uint32_t trailer_efid = 0x6012U + (3U << 16) + (5U << 19) + (0x02U << 24);

		/* D1: idle bus - START->DATA->END->trailer exactly once, trailer exactly after END. */
		{
			reset_all();
			can_multiframe_trailer_t tr;
			tr.efid = trailer_efid; tr.dlen = 4U; memcpy(tr.data, trailer_data, 8);
			CHECK(can_multiframe_start_with_trailer(0x6012U, 5U, 0x02U, payload, 64U, &tr, 0),
				"D1: reply + trailer armed atomically");
			int md = 0;
			int used = run_until_idle(0U, 300, &md);
			CHECK(used < 300, "D1: completes");
			CHECK(can_multiframe_completed_count() == 1U, "D1: completed");
			/* length=64 -> 7 DATA -> START(1)+DATA(7)+END(1)=9, trailer is the 10th (index 9). */
			CHECK(cap_n == 10, "D1: exactly 10 frames - 9 for the reply, 1 trailer");
			CHECK(efid_op(cap_efid[8]) == OP_LONG_END, "D1: frame 9 (index 8) is END");
			CHECK(cap_efid[9] == trailer_efid, "D1: the trailer is exactly the 10th frame - after END");
			CHECK(cap_dlen[9] == 4U && memcmp(cap_data[9], trailer_data, 4U) == 0,
				"D1: the trailer payload is exactly 01 00 02 06");
		}

		/* D2: NOMAILBOX/PENDING/FAILED->retry->OK throughout - trailer still exactly once. */
		{
			reset_all();
			mock_transmit_script[0] = CANQ_NOMAILBOX;
			mock_state_script[0] = CANQ_PENDING;
			mock_state_script[1] = CANQ_FAILED;
			can_multiframe_trailer_t tr;
			tr.efid = trailer_efid; tr.dlen = 4U; memcpy(tr.data, trailer_data, 8);
			CHECK(can_multiframe_start_with_trailer(0x6012U, 5U, 0x02U, payload, 64U, &tr, 0),
				"D2: armed");
			int md = 0;
			int used = run_until_idle(1000U, 800, &md);
			CHECK(used < 800, "D2: completes despite the hiccups");
			CHECK(can_multiframe_completed_count() == 1U, "D2: completed");
			CHECK(can_multiframe_failed_fragment_count() == 0U, "D2: transient FAILED recovered, no give-up");
			CHECK(cap_n == 10, "D2: still exactly 10 frames - hiccups retry, they do not duplicate");
			CHECK(cap_efid[9] == trailer_efid, "D2: trailer is still exactly the 10th frame");
		}

		/* D3: trailer only after END's CAN_TRANSMIT_OK - hold END pending, trailer must not go out. */
		{
			reset_all();
			uint32_t end_efid = frag_efid(7U, OP_LONG_END, 5U, 0x02U);
			pending_efid = end_efid;
			release_pending = false;
			state_override = pending_until_release_override;
			can_multiframe_trailer_t tr;
			tr.efid = trailer_efid; tr.dlen = 4U; memcpy(tr.data, trailer_data, 8);
			CHECK(can_multiframe_start_with_trailer(0x6012U, 5U, 0x02U, payload, 64U, &tr, 0),
				"D3: armed");
			uint32_t tick = 0U;
			int i;
			for (i = 0; i < 400 && !txlog_has(end_efid) && can_multiframe_busy(); i++) {
				can_multiframe_step();
				can_tx_queue_service(tick++);
			}
			CHECK(txlog_has(end_efid), "D3: END was transmitted");
			for (int j = 0; j < 40; j++) {
				can_multiframe_step();
				can_tx_queue_service(tick++);
			}
			CHECK(can_multiframe_busy(), "D3: still busy - END not confirmed");
			CHECK(can_multiframe_completed_count() == 0U, "D3: not completed while END is PENDING");
			CHECK(can_tx_queue_depth() <= 1U, "D3: the trailer has NOT been enqueued (still at most 1 frame)");
			CHECK(!txlog_has(trailer_efid), "D3: the trailer never reached the wire before END's CAN_TRANSMIT_OK");

			release_pending = true;
			run_until_idle(tick, 200, 0);
			CHECK(can_multiframe_completed_count() == 1U,
				"D3: completed only after END (and then the trailer) were confirmed");
			CHECK(cap_n == 10, "D3: trailer appears exactly once, as the 10th frame");
			CHECK(cap_efid[9] == trailer_efid, "D3: trailer is the frame right after END");
		}

		/* B-3 above already covers GIVEUP END (trailer never out) and B-4 GIVEUP trailer. */
	}

	/* --- E: an enqueue REFUSAL (queue raced full) aborts the whole transfer ---------------- */
#ifdef CANMF_REFUSAL_HOOK
	{
		reset_all();
		uint8_t payload[64];
		fill_pattern(payload, 64U);
		CHECK(can_multiframe_start(0x6011U, 5U, 0x02U, payload, 64U, 0), "E: armed");
		/* Arm the refusal seam so the producer's very first enqueue (START) is refused by the
		 * real queue (free_slots pre-check passes, then the queue is raced full). The producer
		 * must ABORT - never silently continue. */
		can_multiframe_test_refuse_next_enqueue();

		int md = 0;
		int used = run_until_idle(0U, 500, &md);
		CHECK(used < 500, "E: the refusal resolves in bounded time");
		CHECK(can_multiframe_aborted_count() == 1U,
			"E: an enqueue refusal ABORTS the whole transfer (never silently continued)");
		CHECK(can_multiframe_completed_count() == 0U, "E: NOT completed");
		CHECK(can_multiframe_failed_fragment_count() == 1U,
			"E: the refused fragment is counted as failed");
		/* Zero fragments of THIS transfer reached the wire - the refused START was the first. */
		int mf_cap = 0;
		for (int c = 0; c < cap_n; c++) {
			if (efid_target(cap_efid[c]) == 5U && efid_source(cap_efid[c]) == 0x02U) mf_cap++;
		}
		CHECK(mf_cap == 0, "E: NO fragment of this transfer reached the wire after the refusal");
	}
#endif

	/* --- 9: a second start() while one is active is refused, first reply unharmed ---------- */
	{
		reset_all();
		for (int i = 0; i < SCRIPT_MAX; i++) mock_transmit_script[i] = CANQ_NOMAILBOX; /* keep it mid-flight */

		uint8_t first[255], second[64];
		fill_pattern(first, 255U);
		fill_pattern(second, 64U);
		for (uint8_t i = 0; i < 64U; i++) second[i] = (uint8_t)(0x80U + i);

		CHECK(can_multiframe_start(0x6020U, 5U, 0x02U, first, 255U, 0), "9: first reply armed");
		CHECK(can_multiframe_busy(), "9: producer is busy (nothing can leave - bus fully NOMAILBOX)");
		uint32_t rejected_before = can_multiframe_rejected_busy_count();

		can_multiframe_id_t xid;
		bool second_started = can_multiframe_start(0x6021U, 5U, 0x02U, second, 64U, &xid);
		CHECK(!second_started, "9: the second start() is refused outright while the first is active");
		CHECK(xid == CANMF_ID_NONE, "9: no id handed out for a refused start");
		CHECK(can_multiframe_rejected_busy_count() == rejected_before + 1U, "9: the refusal is counted");

		for (int i = 0; i < SCRIPT_MAX; i++) mock_transmit_script[i] = 1U;
		int used = run_until_idle(0U, 300, 0);
		CHECK(used < 300, "9: the first reply completes");
		uint8_t out[256] = { 0 };
		int out_len = reconstruct(0, 0x6020U, 5U, 0x02U, out);
		CHECK(out_len == 255 && memcmp(out, first, 255) == 0,
			"9: first reply's payload is exactly what was armed - the refused second start() changed nothing");
	}

	/* --- 10: busy() true exactly while a reply is in progress ------------------------------- */
	{
		reset_all();
		CHECK(!can_multiframe_busy(), "10: idle before any start()");
		uint8_t p[8] = { 1,2,3,4,5,6,7,8 };
		CHECK(can_multiframe_start(0x0001U, 5U, 0x02U, p, 8U, 0), "10: start() succeeds");
		CHECK(can_multiframe_busy(), "10: busy immediately after start()");
		int used = run_until_idle(0U, 50, 0);
		CHECK(used < 50, "10: completes");
		CHECK(!can_multiframe_busy(), "10: idle again once the transfer is confirmed complete");
	}

	if (host_test_failures == 0) {
		printf("All can_multiframe checks passed.\n");
		return 0;
	}
	printf("\n%d can_multiframe check(s) FAILED.\n", host_test_failures);
	return 1;
}