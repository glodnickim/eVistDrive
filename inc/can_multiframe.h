#ifndef CAN_MULTIFRAME_H_
#define CAN_MULTIFRAME_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * FW-110: non-blocking producer for the multiframe (START / DATA... / END) reply format this
 * protocol uses for payloads over 8 bytes (see documentation/CAN_PROTOCOL_REFERENCE.md,
 * operations LONG_START=4 / LONG_TRANG=5 / LONG_END=6).
 *
 * FW-110 v4: the producer is a TRANSMISSION-RESULT-CONFIRMED stop-and-wait automaton. The
 * first version of this module's fix kept the original send_multiframe() loop structure and
 * just swapped each blocking wait for can_tx_queue_enqueue() - every fragment of a reply was
 * offered to the queue back to back in ONE call, before can_tx_queue_service() had transmitted
 * even the first one. For a 255-byte reply (33 fragments: 1 START + 31 DATA + 1 END) against a
 * 16-frame queue, enqueue() accepted the first 16 and silently refused the other 17 -
 * deterministically, on every call, on a completely idle bus. 0x6020 (profile bank read,
 * exactly 255 bytes - see inc/assist_modes.h's ASSIST_BANK_BLOB_LEN) is exactly this shape.
 *
 * v3 fixed that by advancing the cursor only on enqueue()==true. That fixed the drop, but
 * enqueue()==true only means "accepted into the FIFO", nothing about transmission. A fragment
 * that later exhausts its retries in can_tx_queue and is given up (CANQ_TOKEN_FAILED) or a
 * token that can no longer be accounted for (CANQ_TOKEN_UNKNOWN) was silently skipped over: the
 * producer kept emitting the following DATA, END and trailer anyway, so the client received a
 * structurally complete but logically corrupted reply - bytes missing in the middle, and the
 * trailing marker of a reply that never actually arrived intact.
 *
 * v4 rebuilds the module as an explicit automaton in which the cursor advances ONLY on a
 * confirmed CANQ_TOKEN_DONE for the exact frame it is waiting on:
 *
 *   IDLE -> PREPARE_CURRENT -> WAIT_QUEUE_SPACE -> WAIT_FRAGMENT_RESULT -> (next fragment or)
 *           COMPLETE            IDLE            <-   ABORTED
 *
 * Per fragment: build exactly ONE fragment's bytes, enqueue it via
 * can_tx_queue_enqueue_tracked(), keep its token, and do NOT build or enqueue the next fragment
 * until can_tx_queue_token_state() returns CANQ_TOKEN_DONE for it. PENDING keeps the automaton
 * in WAIT_FRAGMENT_RESULT, doing nothing else. DONE is the ONLY way the cursor advances and the
 * next fragment is prepared. FAILED or UNKNOWN aborts the ENTIRE transfer: no further DATA, no
 * END, no trailer, and can_multiframe_busy() releases only on that controlled abort.
 *
 * REQUIRED PROPERTIES (each one is exercised by tests/host/can_multiframe_host.c):
 *   - at most ONE fragment of a given multiframe is ever queued or in flight (the queue can
 *     never be filled by this producer - at most 1 of its 16 slots is ever this module's);
 *   - can_multiframe_busy() stays true until the last frame is confirmed DONE (or the transfer
 *     is aborted), so main.c's allow_new_tx gate keeps diagnostics and the optional 0x3100
 *     stream off the bus for the whole transfer, not just until the FIFO swallowed it;
 *   - fresh heartbeat/status/poll/ACK HMI frames always have the other 15 slots free;
 *   - no waiting loops, at most one automaton transition per can_multiframe_step() call, no
 *     sleep/delay, no direct access to CAN hardware from this file.
 *
 * This module owns exactly ONE multiframe reply at a time and produces it a fragment at a time,
 * across as many can_multiframe_step() calls as it takes. can_multiframe_start()/..._with_
 * trailer() snapshot command/target/source/length/payload into this module's OWN storage (max
 * 255 B) and arm the producer. They return false, changing NOTHING, if a reply is already
 * active - the caller must send nothing at all in that case, not even a partial START. The
 * Canable app's own request manager already retries a READ that times out (3 attempts, 3 s
 * apart - see request-manager.js's READ_RETRY_LIMIT/DEFAULT_TIMEOUT), so a silently refused
 * second request is recovered by the EXISTING client without this protocol needing an invented
 * BUSY response code. Refusals are counted by can_multiframe_rejected_busy_count().
 *
 * The START/DATA/END byte layout and the exact fragment-count arithmetic are unchanged from the
 * original synchronous send_multiframe() - see can_multiframe.c for the formula, carried over
 * verbatim. The optional trailer (0x6012's factory "config transfer complete" marker) is a
 * PHASE OF THIS SAME TRANSFER, armed atomically at start() time (see below) and only ever sent
 * after the END fragment is confirmed CANQ_TOKEN_DONE.
 */

#define CANMF_MAX_PAYLOAD 255U

/* Optional single frame sent after the multiframe reply's END fragment is CONFIRMED done -
 * 0x6012's factory "config transfer complete" marker (01 00 02 06). */
typedef struct {
	uint32_t efid;
	uint8_t  dlen;
	uint8_t  data[8];
} can_multiframe_trailer_t;

/*
 * FW-110 v4: a transfer id + resolvable state, so a caller can attach a side effect to "this
 * SPECIFIC reply actually reached its end", not to "the producer was armed". 0x6029's
 * diag_peak_reset is the first caller (see src/can_reply_effects.c) - it must fire only when
 * the whole 0x6029 snapshot was confirmed delivered, never when a later fragment was dropped.
 */
typedef uint16_t can_multiframe_id_t;
#define CANMF_ID_NONE 0U

typedef enum {
	CANMF_XFER_UNKNOWN = 0, /* CANMF_ID_NONE, or an id too old for the kept history - a caller
	                           that waited this long must not assume DONE, see can_tx_queue's
	                           same UNKNOWN convention */
	CANMF_XFER_PENDING,     /* armed and still in progress, or resolved within the last tick */
	CANMF_XFER_DONE,        /* the whole transfer (incl. an armed trailer) is confirmed sent */
	CANMF_XFER_ABORTED      /* a fragment resolved FAILED/UNKNOWN - nothing further was sent */
} can_multiframe_xfer_state_t;

/* Snapshot (command, target, source, data[0..length)) and arm the producer. `length` must be
 * 1..CANMF_MAX_PAYLOAD. Returns false, changing nothing, if a reply is already active or the
 * request is malformed. `out_id` may be NULL; if non-NULL it receives the transfer's id on
 * success (CANMF_ID_NONE on refusal). */
bool can_multiframe_start(uint16_t command, uint8_t target, uint8_t source,
                           const uint8_t *data, uint8_t length,
                           can_multiframe_id_t *out_id);

/* Same as can_multiframe_start(), with a trailer attached to THIS transfer atomically. The
 * trailer is produced only after the END fragment is confirmed CANQ_TOKEN_DONE; if START, any
 * DATA or END resolves FAILED/UNKNOWN the transfer ends ABORTED and the trailer is never sent.
 * There is deliberately NO start()-then-attach pair: the automaton can never run without its
 * trailer, and a stray trailer from an unrelated earlier session can never leak into a later
 * reply. */
bool can_multiframe_start_with_trailer(uint16_t command, uint8_t target, uint8_t source,
                                        const uint8_t *data, uint8_t length,
                                        const can_multiframe_trailer_t *trailer,
                                        can_multiframe_id_t *out_id);

/* Outcome of a transfer id handed out by a successful start. See can_multiframe_xfer_state_t. */
can_multiframe_xfer_state_t can_multiframe_transfer_state(can_multiframe_id_t id);

/* Call once per main-loop tick, unconditionally (this is a critical-priority producer, same as
 * can_tx_queue itself - it only ever feeds can_tx_queue_enqueue_tracked(), never touches
 * hardware directly). Does at most one automaton transition per call. Does nothing when idle. */
void can_multiframe_step(void);

/* Clean the whole automaton and every counter (nothing active, no pending transfer id, all
 * counters 0). Called once at startup (main.c's CAN init), and by host tests between scenarios
 * - without it a test that checks "completed_count()==1" would see every earlier test's
 * completions accumulating into the count. */
void can_multiframe_init(void);

/* True while a transfer is being produced (from a successful start() until the last fragment -
 * incl. an armed trailer - is confirmed DONE, or the transfer is aborted). main.c uses this to
 * gate diagnostics' allow_new_tx and the optional 0x3100 stream. */
bool can_multiframe_busy(void);

/* Transfers actually armed. */
uint32_t can_multiframe_started_count(void);
/* Transfers that reached a confirmed DONE of their last fragment (incl. an armed trailer). */
uint32_t can_multiframe_completed_count(void);
/* Transfers aborted because a fragment resolved FAILED/UNKNOWN (a started transfer is either
 * completed or aborted, never both). */
uint32_t can_multiframe_aborted_count(void);
/* start() refused because a transfer was already active (or the request was malformed). */
uint32_t can_multiframe_rejected_busy_count(void);
/* Individual fragments that resolved FAILED/UNKNOWN. At most one per aborted transfer. */
uint32_t can_multiframe_failed_fragment_count(void);

#endif /* CAN_MULTIFRAME_H_ */