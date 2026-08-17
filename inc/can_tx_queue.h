#ifndef CAN_TX_QUEUE_H_
#define CAN_TX_QUEUE_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * FW-110: the ONE bounded automaton/queue for CAN frames that MUST be delivered ("critical" -
 * HMI protocol replies, the status/telemetry frames the display needs to stay in sync and not
 * exit Walk Assist) but that may NEVER block the caller. Deliberately free of MS/MP, the GD32
 * driver headers and any project type - every hardware operation goes through the small `ops`
 * struct the caller supplies, which is what lets a host test link and drive this REAL module
 * with a fake CAN peripheral instead of a hand-copied mirror of its logic (same discipline
 * diag_session.c already established for the diagnostic frame family this module is the
 * "critical" sibling of).
 *
 * WHY THIS EXISTS. Every one of ~19 call sites across src/CAN_Display.c and src/main.c used to
 * send a frame with:
 *     transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
 *     timeout = 0xFFFF;
 *     while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout))
 *         timeout--;
 * called from the SAME main loop that decodes PAS and runs the 4 kHz control tick's own
 * housekeeping. A busy CAN bus (measured: the pre-existing 0x81F83100 torque-sensor-emulation
 * stream alone at ~92 frames/s) turns that spin into real, measured missed control ticks - see
 * the FW-110 report for the log evidence. This module replaces the pattern everywhere it was
 * reachable during riding with something that does AT MOST ONE hardware operation per call,
 * unconditionally, so the caller's own per-tick cost is fixed and countable no matter how busy
 * the bus is.
 *
 * CONTRACT.
 *   can_tx_queue_enqueue()  O(1), NEVER blocks, NEVER touches hardware. Copies the frame into
 *                           the queue's own storage (a fixed-size ring buffer) or, if full,
 *                           refuses it - explicit policy: DROP THE NEWEST frame, never evict an
 *                           already-queued one (a queued frame might be the middle fragment of a
 *                           multiframe HMI reply; evicting it would corrupt that reply worse than
 *                           refusing one more frame at the door). Returns false on refusal, and
 *                           the caller's own dropped counter (can_tx_queue_dropped_count) already
 *                           reflects it - a caller does not need its own separate bookkeeping.
 *   can_tx_queue_service()  Call once per main-loop tick. Does EXACTLY ONE of: check the state of
 *                           an in-flight frame, or attempt to start the next queued one - never
 *                           both, never a loop, never a wait. FIFO within this queue, so frames
 *                           enqueued in order are sent in that order; PRIORITY over any other
 *                           consumer of the same CAN peripheral (i.e. diag_session.c's own,
 *                           separate, already non-blocking automaton) is a caller-side property,
 *                           not something this module can see - service THIS queue first each
 *                           tick and a busy mailbox always favours whatever is queued here.
 *
 * SAFETY. A frame that keeps failing gives up after CANQ_MAX_RETRY attempts (counted) rather than
 * retrying forever; a frame whose mailbox never resolves within CANQ_MAX_PENDING_TICKS is forced
 * to give up too (wraparound-safe tick comparison, same unsigned-subtraction idiom used
 * throughout this codebase for free-running hardware counters) - both are what make "zabezpieczenie
 * przed zakleszczeniem" (deadlock protection) true even if the CAN peripheral itself wedges.
 */

#define CANQ_NOMAILBOX 0xFFU

#define CANQ_OK      0U
#define CANQ_PENDING 1U
#define CANQ_FAILED  2U

typedef struct {
	/* Hand a frame to the real peripheral. Returns a mailbox handle (any value != CANQ_NOMAILBOX,
	 * meaning "accepted, now in flight"), or CANQ_NOMAILBOX when no mailbox was free - the
	 * driver wrote nothing at all, so this is not a loss and the same frame is simply offered
	 * again on a later call. Must not block. */
	uint8_t (*transmit)(uint32_t efid, uint8_t dlen, const uint8_t data[8]);
	/* Poll the state of a mailbox returned by transmit() above. Must not block. */
	uint8_t (*state)(uint8_t mailbox);
} can_tx_ops_t;

void can_tx_queue_init(const can_tx_ops_t *ops);

/* O(1), never blocks. Returns false (and counts a drop) if the queue is full. */
bool can_tx_queue_enqueue(uint32_t efid, uint8_t dlen, const uint8_t data[8]);

/*
 * FW-110: the GENERAL mechanism for "did THIS SPECIFIC frame actually reach CAN_TRANSMIT_OK" -
 * needed anywhere a caller must not act until one particular frame (not just "the queue is
 * empty", which says nothing about which frames those were or whether they succeeded) is
 * confirmed on the wire. The Hall/position-calibration ACK is the first caller: the motor must
 * not be driven open-loop until the specific NORMAL_ACK for the accepted request is confirmed
 * sent, and neither "enqueue() returned true" (only means "accepted into the FIFO", nothing about
 * transmission) nor "can_tx_queue_depth()==0" (says nothing about WHICH frame, and is true again
 * the instant anything else drains) is that proof.
 *
 * Same enqueue contract as can_tx_queue_enqueue() (O(1), never blocks, "drop the newest" on a
 * full queue), plus a token identifying this exact frame. *out_token is set to CANQ_TOKEN_INVALID
 * when the frame was refused - matching the false return, so a caller cannot forget to check it.
 */
typedef uint32_t can_tx_token_t;
#define CANQ_TOKEN_INVALID 0U

typedef enum {
	CANQ_TOKEN_PENDING = 0,   /* still queued, or in flight - not yet resolved */
	CANQ_TOKEN_DONE,          /* confirmed CAN_TRANSMIT_OK */
	CANQ_TOKEN_FAILED,        /* gave up: retries exhausted, or the deadlock timeout fired */
	CANQ_TOKEN_UNKNOWN        /* CANQ_TOKEN_INVALID, or a real token too old for the kept history -
	                           * ambiguous on purpose: a caller that waited this long must not
	                           * assume DONE either way, and should treat it like FAILED/retry its
	                           * own higher-level request rather than act on an unproven frame */
} can_tx_token_state_t;

bool can_tx_queue_enqueue_tracked(uint32_t efid, uint8_t dlen, const uint8_t data[8],
                                   can_tx_token_t *out_token);

/* The outcome of a token from can_tx_queue_enqueue_tracked(). See can_tx_token_state_t above for
 * what each value means and why UNKNOWN is deliberately not "assume success". */
can_tx_token_state_t can_tx_queue_token_state(can_tx_token_t token);

/* Call once per main-loop tick with the current free-running control tick. Does at most one
 * hardware operation (see the file header). */
void can_tx_queue_service(uint32_t now_tick);

/* Number of frames currently waiting (including one in flight, if any) - for tests and
 * diagnostics only, never read by a decision. */
uint16_t can_tx_queue_depth(void);

/* FW-110: CANQ_CAPACITY - can_tx_queue_depth() - the number of MORE frames enqueue() would
 * currently accept. A caller that needs to offer several related frames as one unit (the
 * multiframe producer below is the reason this exists) checks this BEFORE calling enqueue(),
 * rather than calling enqueue() speculatively and treating a refusal as normal flow control -
 * a refusal is a real, counted drop (see can_tx_queue_dropped_count()), not a signal to poll on. */
uint16_t can_tx_queue_free_slots(void);

/* transmit() returned CANQ_NOMAILBOX - the bus/mailboxes were busy at that instant. */
uint32_t can_tx_queue_busy_count(void);
/* An in-flight frame came back FAILED and is being offered again (not yet given up). */
uint32_t can_tx_queue_retry_count(void);
/* enqueue() refused a frame because the queue was full. */
uint32_t can_tx_queue_dropped_count(void);
/* A frame exhausted its retries (or its pending-mailbox timeout) and was abandoned. */
uint32_t can_tx_queue_failed_count(void);

#endif /* CAN_TX_QUEUE_H_ */
