#ifndef DIAG_SESSION_H_
#define DIAG_SESSION_H_

#include <stdbool.h>
#include <stdint.h>

#include "config.h"   /* CONTROL_TIMEBASE_HZ only - no MS/MP, no driver, nothing from main.c */

/*
 * FW-106: riding sessions, record accounting and the non-blocking CAN dump.
 *
 * WHY IT IS A MODULE. The first version of all this lived as static functions inside main.c,
 * where nothing can be executed by a test, and it shipped with four defects that its own output
 * could not have revealed: an episode could be counted as sent twice, a record abandoned when
 * the rider set off again was guessed at rather than known, sessions could be interleaved in the
 * log, and the error bits in the trailer were global flags that stayed set until reboot. That is
 * the same lesson FW-101 learned about ride_episode, applied to the code that reports on it.
 *
 * Everything the outside world provides comes in through the two op tables below, so
 * tests/host/fw106_session_host.c drives the REAL code with a fake CAN peripheral and fake
 * record sources.
 *
 * WHAT IT GUARANTEES
 *   - one session at a time in the log: between a session's header and its trailer there are
 *     only that session's records, because every record is asked for BY session id;
 *   - every accepted record ends exactly once, as sent or as discarded, never both and never
 *     neither, for each of the three sources independently;
 *   - the record in flight is always known by kind, so an interruption discards exactly the
 *     record that was interrupted and nothing else;
 *   - nothing blocks: one mailbox action per step, and the fragment cursor moves only on a
 *     confirmed transmit.
 */

/* --- the CAN peripheral, shaped exactly like the GD32 driver so the real one is a thin wrapper */

#define DIAG_CAN_NOMAILBOX 0xFFU

#define DIAG_CAN_OK      0U
#define DIAG_CAN_PENDING 1U
#define DIAG_CAN_FAILED  2U

typedef struct {
	/* Hand a frame over. Returns a mailbox number, or DIAG_CAN_NOMAILBOX when none was free -
	 * in which case NOTHING was written and nothing is lost by trying again later. */
	uint8_t (*transmit)(uint32_t efid, const uint8_t *data);
	/* State of a mailbox previously returned by transmit(). */
	uint8_t (*state)(uint8_t mailbox);
} diag_can_ops_t;

/* --- record sources ---------------------------------------------------------------------- */

typedef enum {
	DIAG_SRC_EPISODE = 0,
	DIAG_SRC_TRACE   = 1,
	DIAG_SRC_RAW     = 2,
	/* FW-111: the delayed-rearm recorder (rearm_delay_diag.c) is the fourth dump source. */
	DIAG_SRC_REARM   = 3,
	/* FW-112-DIAG: the whole-chain event recorder (fw112_diag.c) is the fifth dump source. */
	DIAG_SRC_FW112   = 4,
	DIAG_SRC_COUNT   = 5
} diag_source_t;

typedef struct {
	/* How many records of THIS session are queued right now. Asking by session is what keeps
	 * one session's records out of another session's brackets. */
	uint16_t (*count)(uint8_t session_id);
	/* Build fragment `frag` of the OLDEST queued record of this session.
	 * Sets *last on the final fragment. Returns false when there is no such record. */
	bool (*frame)(uint8_t session_id, uint16_t frag, uint32_t *efid, uint8_t *data, bool *last);
	/* Retire that oldest record. sent=false means it was abandoned, not delivered. */
	void (*release)(uint8_t session_id, bool sent);
	/* Free-running lifetime totals, polled so this module's counters follow the source's own
	 * acceptance decisions without either side having to call the other. */
	uint32_t (*accepted_total)(void);
	uint32_t (*refused_total)(void);
} diag_record_source_t;

typedef struct {
	/* The aggregate block (0x10203..0x1020F, 0x10219): build frame `index`, false past the end. */
	bool (*aggregate_frame)(uint16_t index, uint32_t *efid, uint8_t *data);
	/* Called once when a session closes: any capture still collecting its post-window must be
	 * sealed there and then, or it would go on collecting into the NEXT ride. */
	void (*seal_open_captures)(void);
	/*
	 * FW-106: monotonic count of raw-capture overrun events (pas_raw_capture_overrun()), never
	 * reset except by pas_raw_init(). Anchored and snapshotted at the session's quiet-candidate
	 * moment exactly like missed_ticks_total/missed_events_total, so DIAG_ERR_RAW_OVERRUN in the
	 * trailer reflects THIS session and an overrun during the 3 s confirmation window after a
	 * ride ends cannot attach itself to it. Optional: NULL means "this build has no raw
	 * recorder", and the bit is simply never set.
	 */
	uint32_t (*raw_overrun_total)(void);
	diag_record_source_t src[DIAG_SRC_COUNT];
} diag_ops_t;

/* --- which record is on the wire right now ------------------------------------------------ */

typedef enum {
	DIAG_REC_NONE = 0,   /* header, aggregate block or trailer: no record is at risk */
	DIAG_REC_EPISODE,
	DIAG_REC_TRACE,
	DIAG_REC_RAW,
	DIAG_REC_REARM,      /* FW-111: delayed-rearm recorder - see rearm_delay_diag.h */
	DIAG_REC_FW112       /* FW-112-DIAG: whole-chain event recorder - see fw112_diag.h */
} diag_record_kind_t;

/* --- error bits, per session, reported in the trailer's tx_error_summary ------------------- */

#define DIAG_ERR_TX_FAILED      0x01U  /* at least one frame needed a retry */
#define DIAG_ERR_TX_GAVEUP      0x02U  /* at least one frame exhausted its retries */
#define DIAG_ERR_CAPTURES_FULL  0x04U  /* a trigger found every capture slot occupied */
#define DIAG_ERR_QUEUE_REJECTED 0x08U  /* a full queue refused a record */
#define DIAG_ERR_RAW_OVERRUN    0x10U  /* a raw capture was corrupted while being copied */
#define DIAG_ERR_HEADER_FAILED  0x20U  /* an episode group header could not be sent */
#define DIAG_ERR_INTERRUPTED    0x40U  /* the rider set off again during this session's dump */
/*
 * FW-106: a WHOLE session was lost since the previous trailer - its summary was refused
 * because all DIAG_SESSION_SUMMARIES slots were already occupied when it tried to close (see
 * diag_session.c's full-queue policy). Such a session never gets its own header or trailer, so
 * without this bit it would be completely invisible on the bus: not a missing record inside a
 * session, but a missing SESSION. Carried as a delta on whichever trailer goes out next - once
 * seen, it is not repeated on a later trailer unless another session is lost in the meantime
 * (see diag_summary_rejected() for the raw running total, and the "reported" watermark in
 * diag_session.c that makes this a delta rather than a flag stuck on forever).
 */
#define DIAG_ERR_SESSION_LOST   0x80U

/*
 * Trailer byte 7 flags. Bit 0 is "complete" (see diag_session.c); bit 1 is FW-111: set when the
 * delayed-rearm recorder refused a capture this session, so a reader can tell a FW-111 refusal
 * apart from the shared DIAG_ERR_CAPTURES_FULL (which also covers TRACE/RAW). Bit 2 is
 * FW-112-DIAG: same distinction for the whole-chain event recorder (fw112_diag.c).
 */
#define DIAG_TRAILER_F_COMPLETE       0x01U
#define DIAG_TRAILER_F_REARM_REJECTED 0x02U
#define DIAG_TRAILER_F_FW112_REJECTED 0x04U

/* Retries before a frame - and with it its whole record - is given up on. */
#define DIAG_TX_MAX_RETRY 8U

/* All three must be quiet this long before a session is declared over. */
#define DIAG_SESSION_QUIET_MS    3000U
#define DIAG_SESSION_QUIET_TICKS ((uint32_t)DIAG_SESSION_QUIET_MS * CONTROL_TIMEBASE_HZ / 1000U)

/* Completed sessions that can wait to be dumped. More than one because a dump interrupted by
 * the rider setting off must not be overwritten by the session that ends at the NEXT stop. */
#define DIAG_SESSION_SUMMARIES 4U

/*
 * FW-106: caps how often a NEW frame may be handed to the CAN peripheral during a dump.
 *
 * diag_session_dump_step() is called from main.c's while(1), which runs far more often than
 * once per 4 kHz control tick - "at most one mailbox action per call" therefore did not limit
 * the actual transmit rate at all, only how much work one call could do. A real ride log
 * (log-2026-08-11-17-41-08-n0.log, session 2) showed the CAN hardware confirming every one of
 * 18 records as sent - the firmware's own bookkeeping was correct - while the USB sniffer that
 * captured the dump for analysis only wrote 766 of the ~1389 frames that dump actually sent to
 * its log file, silently dropping the rest because it could not keep up. The frames were never
 * lost on the bus; they were lost downstream of it. Pacing new-frame starts to at most one every
 * DIAG_TX_FRAME_INTERVAL_MS gives that downstream capture path a rate it can sustain.
 */
#define DIAG_TX_FRAME_INTERVAL_MS    10U
#define DIAG_TX_FRAME_INTERVAL_TICKS ((uint32_t)DIAG_TX_FRAME_INTERVAL_MS * CONTROL_TIMEBASE_HZ / 1000U)

void diag_session_init(const diag_can_ops_t *can, const diag_ops_t *ops);

/*
 * FW-106 — CALL THIS FIRST, before anything that might arm a capture on this tick.
 *
 * A caller decoding a raw PAS transition must report a stop->ride transition to this module
 * BEFORE handing that same transition to pas_trace/pas_raw, or the very first suspicious edge of
 * a ride can be stamped with the PREVIOUS session's id - it is armed before diag_session_tick()
 * (called later in the same iteration) has bumped the id. This function does only the narrow
 * thing needed to make that safe: if no session is active, it starts one (bumps the id, sets the
 * metric anchors) and does nothing else. It is idempotent - calling it when a session is already
 * active is a no-op - so the later diag_session_tick() call in the same iteration, seeing
 * `moving`, will NOT start a second session or bump the id again.
 *
 * A caller must, immediately after this returns, propagate diag_session_current_id() to every
 * recorder that might be used before diag_session_tick() runs later in the same iteration.
 *
 * Wheel- or current-triggered starts do not need this: they do not arm pas_trace/pas_raw within
 * the same tick, so diag_session_tick() alone (called every iteration regardless) still starts
 * the session correctly for them.
 */
void diag_session_note_activity(uint32_t now_tick,
                                uint32_t missed_ticks_total, uint32_t missed_events_total);

/*
 * Once per control iteration. `burst_this_tick` is the size of a loss burst that happened on
 * THIS tick (0 when none) - a maximum cannot be recovered by subtracting anchors, so the session
 * keeps its own.
 */
void diag_session_tick(uint32_t now_tick, bool moving,
                       uint32_t missed_ticks_total, uint32_t missed_events_total,
                       uint16_t burst_this_tick);

/*
 * One non-blocking step of the dump: at most one mailbox action, never a wait. `now_tick` is
 * the current hardware tick - passed in, rather than read from a global, so this stays testable
 * with a fake clock. Starting a NEW frame is paced to DIAG_TX_FRAME_INTERVAL_TICKS; everything
 * else (noticing a resumed ride, reading back an in-flight frame's result, retrying a failed
 * fragment) still happens on every single call, exactly as before.
 *
 * FW-110: `allow_new_tx` is the caller's (main.c's) proof that no critical HMI/Canable frame is
 * waiting for a mailbox right now. When false, this function still does everything it always
 * did EXCEPT hand a NEW frame to the peripheral - it still polls an already in-flight frame's
 * result, still notices the rider setting off again and aborts the dump at once (abort_dump()
 * runs before this gate is even consulted - see diag_session.c), and still holds its place: the
 * exact fragment that was about to be offered is offered again, unchanged, the next time
 * allow_new_tx is true. A frame that just came back FAILED counts as "new" for this purpose too
 * - a retry is one more claim on a mailbox, same as any other fresh transmit attempt.
 */
void diag_session_dump_step(uint32_t now_tick, bool allow_new_tx);

/* The id stamped on records created from now on. */
uint8_t diag_session_current_id(void);
bool    diag_session_is_active(void);

/* --- observation, for the diagnostics themselves and for tests ---------------------------- */

uint32_t diag_counter_enqueued(diag_source_t src);
uint32_t diag_counter_sent(diag_source_t src);
uint32_t diag_counter_discarded(diag_source_t src);
uint32_t diag_counter_rejected(diag_source_t src);
/* enqueued - sent - discarded. Zero once every accepted record has been resolved. */
uint32_t diag_counter_pending(diag_source_t src);

diag_record_kind_t diag_dump_record_kind(void);
uint16_t diag_summaries_pending(void);
uint16_t diag_summary_rejected(void);

#endif /* DIAG_SESSION_H_ */
