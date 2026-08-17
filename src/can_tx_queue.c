#include "can_tx_queue.h"

/* See inc/can_tx_queue.h for the full contract. This file is deliberately the smallest thing
 * that satisfies it - one ring buffer, one in-flight slot, four counters. */

#define CANQ_CAPACITY 16U

/* Matches diag_session.c's own DIAG_TX_MAX_RETRY (inc/diag_session.h) - same bus, same class of
 * transient failure, no reason for a different tolerance. */
#define CANQ_MAX_RETRY 8U

/* Deadlock guard only, not a normal-path timer: a real CAN transmit resolves in microseconds.
 * 400 ticks @ 4 kHz = 100 ms - generous enough that no legitimate transmission ever approaches
 * it, short enough that a genuinely wedged mailbox cannot hold this queue hostage for long. */
#define CANQ_MAX_PENDING_TICKS 400U

typedef struct {
	uint32_t efid;
	uint8_t dlen;
	uint8_t data[8];
	uint32_t seq;
} canq_frame_t;

static canq_frame_t ring[CANQ_CAPACITY];
static uint16_t q_head;
static uint16_t q_count;

static const can_tx_ops_t *ops;

static bool in_flight;
static uint8_t in_flight_mailbox;
static uint32_t in_flight_since_tick;
static uint8_t retry_count;

static uint32_t busy_ctr;
static uint32_t retry_ctr;
static uint32_t dropped_ctr;
static uint32_t failed_ctr;

/*
 * FW-110: per-frame completion tracking (can_tx_queue_enqueue_tracked/_token_state). Each tracked
 * frame gets a monotonic seq (0 reserved as CANQ_TOKEN_INVALID); a small ring of the most
 * recently RESOLVED frames (seq + outcome) is kept so a caller can ask "did MY frame make it"
 * without the queue needing per-token storage that outlives the frame itself. A token older than
 * this history (or not found queued/in-flight either) reports UNKNOWN, not DONE - see the header.
 */
static uint32_t next_seq = 1U;

#define CANQ_HISTORY 8U
static uint32_t hist_seq[CANQ_HISTORY];
static bool     hist_ok[CANQ_HISTORY];
static uint8_t  hist_write;
static uint8_t  hist_filled;

static void record_resolution(uint32_t seq, bool ok)
{
	hist_seq[hist_write] = seq;
	hist_ok[hist_write] = ok;
	hist_write = (uint8_t)((hist_write + 1U) % CANQ_HISTORY);
	if (hist_filled < CANQ_HISTORY) hist_filled++;
}

void can_tx_queue_init(const can_tx_ops_t *o)
{
	ops = o;
	q_head = 0U;
	q_count = 0U;
	in_flight = false;
	in_flight_mailbox = 0U;
	in_flight_since_tick = 0U;
	retry_count = 0U;
	busy_ctr = 0U;
	retry_ctr = 0U;
	dropped_ctr = 0U;
	failed_ctr = 0U;
	next_seq = 1U;
	hist_write = 0U;
	hist_filled = 0U;
}

bool can_tx_queue_enqueue_tracked(uint32_t efid, uint8_t dlen, const uint8_t data[8],
                                   can_tx_token_t *out_token)
{
	if (q_count >= CANQ_CAPACITY) {
		dropped_ctr++;
		if (out_token) *out_token = CANQ_TOKEN_INVALID;
		return false;
	}
	uint16_t idx = (uint16_t)((q_head + q_count) % CANQ_CAPACITY);
	ring[idx].efid = efid;
	ring[idx].dlen = dlen;
	for (uint8_t i = 0; i < 8U; i++) ring[idx].data[i] = data[i];
	uint32_t seq = next_seq++;
	if (next_seq == 0U) next_seq = 1U;   /* wraparound: skip over the reserved INVALID value */
	ring[idx].seq = seq;
	q_count++;
	if (out_token) *out_token = seq;
	return true;
}

bool can_tx_queue_enqueue(uint32_t efid, uint8_t dlen, const uint8_t data[8])
{
	return can_tx_queue_enqueue_tracked(efid, dlen, data, 0);
}

can_tx_token_state_t can_tx_queue_token_state(can_tx_token_t token)
{
	if (token == CANQ_TOKEN_INVALID) return CANQ_TOKEN_UNKNOWN;
	for (uint8_t i = 0; i < hist_filled; i++) {
		uint8_t idx = (uint8_t)((hist_write + CANQ_HISTORY - 1U - i) % CANQ_HISTORY);
		if (hist_seq[idx] == token) return hist_ok[idx] ? CANQ_TOKEN_DONE : CANQ_TOKEN_FAILED;
	}
	for (uint16_t i = 0; i < q_count; i++) {
		uint16_t idx = (uint16_t)((q_head + i) % CANQ_CAPACITY);
		if (ring[idx].seq == token) return CANQ_TOKEN_PENDING;
	}
	return CANQ_TOKEN_UNKNOWN;
}

static void pop_head(void)
{
	q_head = (uint16_t)((q_head + 1U) % CANQ_CAPACITY);
	q_count--;
}

/* Wraparound-safe "has more than N ticks elapsed since `since`" - same unsigned-subtraction idiom
 * as ride_episode.c's elapsed_ticks()/ride_session.c's tick_reached(), correct across the 32-bit
 * wrap as long as the true gap never exceeds ~2^31 ticks (~6.2 days @ 4 kHz), always true for a
 * single in-flight frame's own lifetime. */
static bool elapsed_more_than(uint32_t now_tick, uint32_t since, uint32_t ticks)
{
	return (int32_t)((now_tick - since)) > (int32_t)ticks;
}

void can_tx_queue_service(uint32_t now_tick)
{
	if (ops == 0) return;

	if (in_flight) {
		if (elapsed_more_than(now_tick, in_flight_since_tick, CANQ_MAX_PENDING_TICKS)) {
			/* Deadlock guard: this mailbox never resolved. Give up on it unconditionally,
			 * regardless of retry_count - a wedged peripheral will not un-wedge by waiting
			 * longer, and every other queued frame is waiting behind this one. */
			in_flight = false;
			retry_count = 0U;
			failed_ctr++;
			record_resolution(ring[q_head].seq, false);
			pop_head();
			return;
		}

		uint8_t st = ops->state(in_flight_mailbox);
		if (st == CANQ_PENDING) {
			return;   /* still on the wire; nothing else to do this call */
		}
		in_flight = false;
		if (st == CANQ_OK) {
			retry_count = 0U;
			record_resolution(ring[q_head].seq, true);
			pop_head();
			return;
		}
		/* FAILED */
		retry_count++;
		if (retry_count > CANQ_MAX_RETRY) {
			retry_count = 0U;
			failed_ctr++;
			record_resolution(ring[q_head].seq, false);
			pop_head();
		} else {
			retry_ctr++;
			/* Frame stays at the head - offered again on a later call. */
		}
		return;
	}

	if (q_count == 0U) return;   /* nothing queued */

	const canq_frame_t *f = &ring[q_head];
	uint8_t mb = ops->transmit(f->efid, f->dlen, f->data);
	if (mb == CANQ_NOMAILBOX) {
		busy_ctr++;
		return;   /* frame stays queued; try again next call */
	}
	in_flight = true;
	in_flight_mailbox = mb;
	in_flight_since_tick = now_tick;
}

uint16_t can_tx_queue_depth(void) { return q_count; }
uint16_t can_tx_queue_free_slots(void) { return (uint16_t)(CANQ_CAPACITY - q_count); }
uint32_t can_tx_queue_busy_count(void) { return busy_ctr; }
uint32_t can_tx_queue_retry_count(void) { return retry_ctr; }
uint32_t can_tx_queue_dropped_count(void) { return dropped_ctr; }
uint32_t can_tx_queue_failed_count(void) { return failed_ctr; }
