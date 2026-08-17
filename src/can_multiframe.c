#include "can_multiframe.h"
#include "can_tx_queue.h"

#include <string.h>

/* See inc/can_multiframe.h for the full contract. */

/* Matches this protocol's own operation codes - documentation/CAN_PROTOCOL_REFERENCE.md. */
#define CANMF_OP_LONG_START 4U
#define CANMF_OP_LONG_TRANG 5U
#define CANMF_OP_LONG_END   6U

/* FW-110 v4: explicit automaton states - see the header. COMPLETE/ABORTED are transient states
 * that consume exactly one extra step() call each before returning to IDLE (which is also what
 * keeps can_multiframe_busy() true until the very end of the controlled finish). */
typedef enum {
	CANMF_IDLE = 0,
	CANMF_PREPARE_CURRENT,      /* build the current fragment's bytes (memory only) */
	CANMF_WAIT_QUEUE_SPACE,     /* wait for a free can_tx_queue slot, then enqueue_tracked */
	CANMF_WAIT_FRAGMENT_RESULT, /* poll the current fragment's token - the ONLY cursor mover */
	CANMF_COMPLETE,             /* last fragment confirmed DONE - counting, then IDLE */
	CANMF_ABORTED               /* a fragment resolved FAILED/UNKNOWN - counting, then IDLE */
} canmf_state_t;

typedef enum {
	CANMF_FRAG_START = 0,
	CANMF_FRAG_DATA,
	CANMF_FRAG_END,
	CANMF_FRAG_TRAILER
} canmf_frag_t;

static bool active;
static canmf_state_t state;
static uint16_t mf_command;
static uint8_t  mf_target;
static uint8_t  mf_source;
static uint8_t  mf_length;
static uint8_t  mf_payload[CANMF_MAX_PAYLOAD];
static uint8_t  mf_nbrofframes;   /* number of DATA fragments - same formula as the original */
static uint8_t  mf_next_frag;     /* next DATA fragment index still to send */
static canmf_frag_t mf_kind;      /* what the CURRENT fragment is */

static bool     mf_trailer_armed;
static can_multiframe_trailer_t mf_trailer;

static can_tx_token_t mf_frag_token;   /* token of the ONE fragment currently enqueued/in flight */
static uint32_t mf_cur_efid;           /* current fragment, built once, offered until accepted */
static uint8_t  mf_cur_dlen;
static uint8_t  mf_cur[8];

#ifdef CANMF_REFUSAL_HOOK
/* Test-only seam, compiled out of the firmware entirely (only tests/host defines
 * CANMF_REFUSAL_HOOK - see run-host-tests.ps1). Simulates the enqueue-race the defensive
 * refusal path below guards against: another context filling the queue between the free-slot
 * pre-check and the enqueue, so can_tx_queue_enqueue_tracked() refuses despite free_slots()>0.
 * Armed by tests/host/can_multiframe_host.c; without it the producer's refusal path cannot be
 * reached from a single-threaded test. */
static bool mf_test_refuse_enqueue;
void can_multiframe_test_refuse_next_enqueue(void) { mf_test_refuse_enqueue = true; }
static bool test_refusal_requested(void)
{
	bool r = mf_test_refuse_enqueue;
	mf_test_refuse_enqueue = false;
	return r;
}
#endif

static uint32_t started_ctr;
static uint32_t completed_ctr;
static uint32_t aborted_ctr;
static uint32_t rejected_busy_ctr;
static uint32_t failed_fragment_ctr;

/* --- transfer id / state (FW-110 v4, for 0x6029's side effect) ------------------------------ */
static uint16_t xfer_next_id;
static uint16_t xfer_current_id;   /* id of the active transfer, CANMF_ID_NONE when idle */

#define CANMF_XFER_HISTORY 4U
static uint16_t xfer_hist_id[CANMF_XFER_HISTORY];
static can_multiframe_xfer_state_t xfer_hist_state[CANMF_XFER_HISTORY];
static uint8_t xfer_hist_write;
static uint8_t xfer_hist_filled;

static uint32_t build_efid(uint16_t command, uint8_t operation, uint8_t target, uint8_t source)
{
	return (uint32_t)command + ((uint32_t)operation << 16) + ((uint32_t)target << 19) +
	       ((uint32_t)source << 24);
}

static void record_xfer_resolution(can_multiframe_xfer_state_t outcome)
{
	if (xfer_current_id == CANMF_ID_NONE) return;
	xfer_hist_id[xfer_hist_write] = xfer_current_id;
	xfer_hist_state[xfer_hist_write] = outcome;
	xfer_hist_write = (uint8_t)((xfer_hist_write + 1U) % CANMF_XFER_HISTORY);
	if (xfer_hist_filled < CANMF_XFER_HISTORY) xfer_hist_filled++;
	xfer_current_id = CANMF_ID_NONE;
}

/* The transfer's CURRENT fragment kind and bytes - the START/DATA/END layout and the
 * fragment-count arithmetic are identical to the original synchronous send_multiframe(). */
static void build_current_fragment(void)
{
	switch (mf_kind) {
	case CANMF_FRAG_START:
		mf_cur_efid = build_efid(mf_command, CANMF_OP_LONG_START, mf_target, mf_source);
		mf_cur_dlen = 1U;
		mf_cur[0] = mf_length;
		break;
	case CANMF_FRAG_DATA:
		memcpy(mf_cur, mf_payload + (uint16_t)mf_next_frag * 8U, 8U);
		mf_cur_dlen = 8U;
		mf_cur_efid = build_efid(mf_next_frag, CANMF_OP_LONG_TRANG, mf_target, mf_source);
		break;
	case CANMF_FRAG_END: {
		uint8_t rem = mf_length % 8U;
		if (rem) {
			mf_cur_dlen = rem;
			memcpy(mf_cur, mf_payload + (uint16_t)mf_nbrofframes * 8U, rem);
		} else {
			mf_cur_dlen = 8U;
			memcpy(mf_cur, mf_payload + (uint16_t)mf_nbrofframes * 8U, 8U);
		}
		mf_cur_efid = build_efid(mf_nbrofframes, CANMF_OP_LONG_END, mf_target, mf_source);
		break;
	}
	case CANMF_FRAG_TRAILER:
		mf_cur_efid = mf_trailer.efid;
		mf_cur_dlen = mf_trailer.dlen;
		memcpy(mf_cur, mf_trailer.data, 8U);
		break;
	}
}

/* Advance the cursor past the JUST-CONFIRMED fragment. Returns true if a further fragment (DATA
 * /END/trailer) still has to be produced, false when this was the transfer's last one. */
static bool advance_cursor(void)
{
	switch (mf_kind) {
	case CANMF_FRAG_START:
		mf_kind = (mf_nbrofframes > 0U) ? CANMF_FRAG_DATA : CANMF_FRAG_END;
		break;
	case CANMF_FRAG_DATA:
		mf_next_frag++;
		if (mf_next_frag >= mf_nbrofframes) mf_kind = CANMF_FRAG_END;
		break;
	case CANMF_FRAG_END:
		if (mf_trailer_armed) mf_kind = CANMF_FRAG_TRAILER;
		else return false;   /* END was the last fragment */
		break;
	case CANMF_FRAG_TRAILER:
		return false;        /* the trailer was the last fragment */
	}
	return true;
}

static bool start_common(uint16_t command, uint8_t target, uint8_t source,
                         const uint8_t *data, uint8_t length,
                         const can_multiframe_trailer_t *trailer,
                         can_multiframe_id_t *out_id)
{
	if (out_id) *out_id = CANMF_ID_NONE;
	uint16_t len = length;   /* widened so the range check is not provably-always-false */
	if (active || len == 0U || len > (uint16_t)CANMF_MAX_PAYLOAD) {
		rejected_busy_ctr++;
		return false;
	}
	mf_command = command;
	mf_target = target;
	mf_source = source;
	mf_length = length;
	memcpy(mf_payload, data, length);
	/* Identical to the original send_multiframe()'s own formula: an exact multiple of 8 gets
	 * one fewer DATA fragment because the END frame absorbs that final full 8-byte chunk. */
	mf_nbrofframes = (length % 8U) ? (uint8_t)(length >> 3) : (uint8_t)((length >> 3) - 1U);
	mf_next_frag = 0U;
	mf_trailer_armed = (trailer != 0);
	if (trailer) mf_trailer = *trailer;
	mf_kind = CANMF_FRAG_START;
	mf_frag_token = CANQ_TOKEN_INVALID;
	state = CANMF_PREPARE_CURRENT;
	active = true;
	started_ctr++;

	xfer_next_id++;
	if (xfer_next_id == CANMF_ID_NONE) xfer_next_id++;   /* 0 is reserved */
	xfer_current_id = xfer_next_id;
	if (out_id) *out_id = xfer_current_id;
	return true;
}

bool can_multiframe_start(uint16_t command, uint8_t target, uint8_t source,
                           const uint8_t *data, uint8_t length,
                           can_multiframe_id_t *out_id)
{
	return start_common(command, target, source, data, length, 0, out_id);
}

bool can_multiframe_start_with_trailer(uint16_t command, uint8_t target, uint8_t source,
                                        const uint8_t *data, uint8_t length,
                                        const can_multiframe_trailer_t *trailer,
                                        can_multiframe_id_t *out_id)
{
	return start_common(command, target, source, data, length, trailer, out_id);
}

can_multiframe_xfer_state_t can_multiframe_transfer_state(can_multiframe_id_t id)
{
	if (id == CANMF_ID_NONE) return CANMF_XFER_UNKNOWN;
	if (id == xfer_current_id) return CANMF_XFER_PENDING;
	for (uint8_t i = 0; i < xfer_hist_filled; i++) {
		uint8_t idx = (uint8_t)((xfer_hist_write + CANMF_XFER_HISTORY - 1U - i) % CANMF_XFER_HISTORY);
		if (xfer_hist_id[idx] == id) return xfer_hist_state[idx];
	}
	return CANMF_XFER_UNKNOWN;
}

bool can_multiframe_busy(void) { return active; }
uint32_t can_multiframe_started_count(void) { return started_ctr; }
uint32_t can_multiframe_completed_count(void) { return completed_ctr; }
uint32_t can_multiframe_aborted_count(void) { return aborted_ctr; }
uint32_t can_multiframe_rejected_busy_count(void) { return rejected_busy_ctr; }
uint32_t can_multiframe_failed_fragment_count(void) { return failed_fragment_ctr; }

void can_multiframe_init(void)
{
	active = false;
	state = CANMF_IDLE;
	mf_trailer_armed = false;
	mf_frag_token = CANQ_TOKEN_INVALID;
	xfer_current_id = CANMF_ID_NONE;
	started_ctr = 0U;
	completed_ctr = 0U;
	aborted_ctr = 0U;
	rejected_busy_ctr = 0U;
	failed_fragment_ctr = 0U;
	xfer_next_id = 0U;
	xfer_hist_write = 0U;
	xfer_hist_filled = 0U;
}

void can_multiframe_step(void)
{
	switch (state) {
	case CANMF_PREPARE_CURRENT:
		/* Build the CURRENT fragment once. Memory only - the queue/hardware is not touched
		 * here, so this is never blocked by a busy bus. */
		build_current_fragment();
		state = CANMF_WAIT_QUEUE_SPACE;
		return;

	case CANMF_WAIT_QUEUE_SPACE:
		/* A full critical queue is as normal as a busy mailbox - the fragment is simply
		 * offered again on a later call. Checked with can_tx_queue_free_slots() BEFORE
		 * enqueue(), so a full queue never reaches enqueue()'s own refusal path and never
		 * touches can_tx_queue_dropped_count() - that counter must stay a count of frames
		 * this firmware actually gave up on, not this producer's ordinary waiting. */
		if (can_tx_queue_free_slots() == 0U) return;
#ifdef CANMF_REFUSAL_HOOK
		if (test_refusal_requested()) {
			/* Test-only (compiled out of the firmware): simulate the enqueue-race this
			 * refusal path exists for - another context stealing the free slot between the
			 * free-slot check above and this producer's enqueue. Fill the queue so the
			 * enqueue_tracked() below refuses, which is the only way a single-threaded test
			 * can reach the abort branch. */
			static const uint8_t filler[8] = { 0 };
			while (can_tx_queue_free_slots() > 0U) can_tx_queue_enqueue(0x02F83202U, 1U, filler);
		}
#endif
		if (!can_tx_queue_enqueue_tracked(mf_cur_efid, mf_cur_dlen, mf_cur, &mf_frag_token)) {
			/* Defensive only: the free-slot check above should have prevented this. A refusal
			 * here still cannot be treated as flow control - it is a real loss of this
			 * fragment, so the whole transfer must abort rather than continue corrupted. */
			failed_fragment_ctr++;
			record_xfer_resolution(CANMF_XFER_ABORTED);
			state = CANMF_ABORTED;
			return;
		}
		state = CANMF_WAIT_FRAGMENT_RESULT;
		return;

	case CANMF_WAIT_FRAGMENT_RESULT:
		switch (can_tx_queue_token_state(mf_frag_token)) {
		case CANQ_TOKEN_PENDING:
			return;   /* still queued or in flight - do NOT build or enqueue the next fragment */
		case CANQ_TOKEN_DONE:
			if (!advance_cursor()) {
				/* This was the transfer's last fragment - confirmed on the wire. */
				record_xfer_resolution(CANMF_XFER_DONE);
				state = CANMF_COMPLETE;
			} else {
				state = CANMF_PREPARE_CURRENT;   /* next fragment, built on a later call */
			}
			return;
		case CANQ_TOKEN_FAILED:
		case CANQ_TOKEN_UNKNOWN:
			/* Permanent loss of THIS fragment (retries exhausted / deadlock give-up / the token
			 * aged out of the queue's history). The transfer is logically corrupted: no further
			 * DATA, no END, no trailer may go out. */
			failed_fragment_ctr++;
			record_xfer_resolution(CANMF_XFER_ABORTED);
			state = CANMF_ABORTED;
			return;
		}
		return;

	case CANMF_COMPLETE:
		completed_ctr++;
		active = false;
		state = CANMF_IDLE;
		return;

	case CANMF_ABORTED:
		aborted_ctr++;
		active = false;
		state = CANMF_IDLE;
		return;

	case CANMF_IDLE:
	default:
		active = false;
		state = CANMF_IDLE;
		return;
	}
}