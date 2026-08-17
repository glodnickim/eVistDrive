#ifndef PAS_TRACE_H_
#define PAS_TRACE_H_

#include <stdbool.h>
#include <stdint.h>

#include "config.h"   /* CAN_DIAGNOSTICS_ENABLE only - this module has no other dependency */

/*
 * FW-111 — REARM RESERVATION, OWNERSHIP AUTOMATON. The delayed-rearm recorder
 * (rearm_delay_diag.h) needs a decoder trace that starts at the REAL initiating event - the
 * ACTIVE -> SUSPENDED_BY_DIRECTION edge, i.e. the reverse/invalid step that began the recovery
 * saga - and survives until the WHOLE recovery is resolved, including a WEAK_TARGET that is only
 * detected AFTER commit (150 ms of continuous low Iq, observed only while the session is already
 * ACTIVE again). Slot ownership is dynamic and explicit, and - the point this file's v5 rewrite
 * exists for - kept SEPARATE from "this slot is still collecting POST or waiting to be dumped":
 *
 *   pas_trace_rearm_prearm()   - starts (or, if a new saga interrupts an unfinished one, first
 *                                ENDS the old ownership and then starts) a fresh reservation, on
 *                                the SAME tick as the initiating reverse/invalid, immediately
 *                                after that edge's own pas_trace_transition() call. Picks
 *                                whichever slot is currently free (not ready, not armed) AND is
 *                                not the active ordinary watcher AND is not a slot RETAINED from
 *                                an earlier, still-unstreamed capture - never a fixed index -
 *                                resets it, stamps the current session id, and SEEDS it with the
 *                                sample just written to the active watcher (the initiating edge
 *                                itself). If ending the old ownership left its slot armed/ready,
 *                                that slot becomes RETAINED (see below) and is never touched by
 *                                the new reservation - not reused, not overwritten, not presented
 *                                as the new saga's history. Returns PAS_TRACE_SLOTS (no slot
 *                                claimed) if nothing is free - the caller then has no reservation
 *                                and must report that; a retained slot does not free up capacity
 *                                until it is actually streamed.
 *   pas_trace_rearm_held()     - true while the CURRENT saga owns a reservation - covers the
 *                                whole saga (PRECOMMIT through POSTCOMMIT: SUSPENDED<->WAIT
 *                                oscillation, the COMMIT tick itself, and the whole recovery
 *                                watch afterwards), until pas_trace_rearm_end_ownership() ends it
 *                                or a new saga's prearm() ends it first. Deliberately does NOT
 *                                reflect a RETAINED slot from an already-ended ownership - that is
 *                                a separate fact (see pas_trace_rearm_retained_slot_index()).
 *                                NOTE: PRECOMMIT/POSTCOMMIT/COMMIT is HISTORICAL vocabulary from
 *                                the v1 two-phase (delayed) rearm; FW-112 v2 re-arms in ONE tick on
 *                                the direction-confirm edge (no commit split), so these phases no
 *                                longer exist as separate production steps - the terms are kept
 *                                only for compatibility with older decoder traces.
 *   pas_trace_rearm_capture(in) - forces EXACTLY ONE trigger on the CURRENTLY OWNED slot. Same
 *                                capture-id contract as pas_trace_transition(). Refuses
 *                                (PAS_TRACE_NO_CAPTURE) when nothing is owned, or when the owned
 *                                slot is already armed or still ready from an earlier, un-streamed
 *                                capture in the SAME saga - counted once per reservation, never
 *                                per retry.
 *   pas_trace_rearm_end_ownership() - the current saga's reservation is over (the recorder's
 *                                record for it closed, or the saga concluded with no record ever
 *                                tracking it). If the owned slot was NEVER armed, it is freed back
 *                                to ordinary use immediately - no orphaned hold. If it IS armed or
 *                                ready, OWNERSHIP ends (pas_trace_rearm_held() goes false, the
 *                                slot is no longer "this saga's") but the slot itself becomes
 *                                RETAINED: still excluded from ordinary use and from a future
 *                                saga's candidate list, dual-write still completes its POST window
 *                                if it was mid-collection, and it is only ever finally freed by
 *                                pas_trace_slot_release() once actually streamed. This is the
 *                                split the v5 rewrite makes explicit: "who owns the reservation"
 *                                and "does this slot still need to keep collecting / wait for the
 *                                dump" are two different facts, tracked separately, so a NEW
 *                                ACTIVE->SUSPENDED during the previous saga's recovery watch can
 *                                never be mistaken for owning the OLD slot (the exact defect a
 *                                single, overloaded rearm_slot field produced).
 *   pas_trace_rearm_slot_index() - the CURRENTLY OWNED slot, or PAS_TRACE_SLOTS when none is
 *                                held - for tests and observability only.
 *   pas_trace_rearm_retained_slot_index() - a slot RETAINED from an ended ownership, still
 *                                armed/ready and excluded from ordinary + new-saga use, or
 *                                PAS_TRACE_SLOTS when none is retained - tests/observability only.
 *
 * FW-106: the raw PAS transition trace — a ring buffer of what the two quadrature lines actually
 * did around a suspicious event.
 *
 * WHY. Every measurement so far has started ABOVE the decoder and worked downwards, so all of
 * them can say "a reverse step was counted" and none can say why. The bike log showed gaps as
 * short as 0.25 ms at live cadence, which is enough to call the PAS input suspect and not
 * enough to name the part. The analysis has to run the other way round:
 *
 *     raw A/B + raw pressure -> direction decoder -> fwd_run/latch -> Iq target -> setpoint
 *
 * This is the bottom of that chain. It records every transition, keeps the last
 * PAS_TRACE_PRE around, and freezes PAS_TRACE_POST more after something suspicious, so the
 * shape either side of the event survives. Nothing is sent while riding: the frozen buffer is
 * streamed afterwards, a few frames per slow loop, so the bus load does not change.
 *
 * WHAT THE SHAPES MEAN, which is the point of recording them:
 *
 *   reverse then straight back, 1-3 ticks     contact bounce or interference
 *   illegal two-bit change (00<->11, 01<->10) interference, a missed edge, or sampling
 *   errors clustered at one disc position     a bad magnet or sensor alignment; once per rev
 *   errors always on the same transition      one channel, one wire, one edge
 *   clean transitions at normal spacing       the crank really did rock back, and the fault
 *                                             is the intolerant fwd_run/latch logic instead
 *   raw pressure rises while the filtered
 *   signal stays at zero                      the fault is in the torque filter chain, not PAS
 *
 * MEASUREMENT ONLY. No decision anywhere reads any of this. It lives in its own file, free of
 * MS/MP, so tests/host/fw102_pas_trace_host.c can drive it — the lesson from FW-101, where a
 * recorder written inline in main.c shipped with three defects its own output could not show.
 *
 * FW-106 — WHY THERE IS NOW MORE THAN ONE SLOT. Under FW-106 nothing is streamed while the
 * bike is moving: the whole dump waits for a standstill (see main.c). With a single buffer the
 * first trigger froze it and every later one in the same ride was simply lost, because release
 * only happens after the dump. PAS_TRACE_SLOTS captures can now be held at once — a queue of
 * captures, not a continuous stream. When they are all full a further trigger only increments
 * pas_trace_capture_slots_full(): counted, but without its detailed trace. That is a deliberate,
 * measurable limit rather than silent loss.
 *
 * FW-106 — CAPTURE ID. Every trigger allocates one id, which is stamped on the frozen decoder
 * slot AND handed back to the caller so the raw ISR recorder (pas_raw.h) can stamp the same id
 * on its own snapshot of the same physical event. That shared key is what lets the two be
 * paired offline; before it, the pairing was by arrival order on the bus, which the ride log
 * proved unreliable (FW-106 Bug 2). PAS_TRACE_NO_CAPTURE is never allocated, so it can be used
 * as "no capture here" by anything holding one of these ids.
 */

#define PAS_TRACE_PRE   128U   /* samples kept before the trigger */
#define PAS_TRACE_POST  128U   /* samples still recorded after it */
#define PAS_TRACE_LEN   (PAS_TRACE_PRE + PAS_TRACE_POST)

/*
 * FW-106: how many frozen captures can be held at once, waiting for the post-ride dump.
 * The second slot is spent only by the diagnostic build - the normal firmware keeps the single
 * buffer it always had, so this card costs it no RAM at all.
 */
#if CAN_DIAGNOSTICS_ENABLE
#define PAS_TRACE_SLOTS 2U
#else
#define PAS_TRACE_SLOTS 1U
#endif

/* FW-106: never allocated as a real id — free to mean "not part of any capture". */
#define PAS_TRACE_NO_CAPTURE 0xFFU

/* A gap this short is not a crank movement; it is the line bouncing or being disturbed. */
#define PAS_TRACE_SHORT_GAP_TICKS 3U

/* sample.flags */
#define PAS_TR_REVERSE    0x01  /* the decoder called this step backwards */
#define PAS_TR_TWO_BIT    0x02  /* both lines changed at once: impossible for real motion */
#define PAS_TR_SHORT_GAP  0x04  /* <= PAS_TRACE_SHORT_GAP_TICKS since the previous change */
#define PAS_TR_BRAKE      0x08
#define PAS_TR_ROLLING    0x10  /* the bike was moving */
#define PAS_TR_LATCHED    0x20  /* the ride latch was armed */
#define PAS_TR_TRIGGER    0x40  /* this is the sample that armed the capture */
#define PAS_TR_LATCH_LOSS 0x80  /* the latch dropped on this sample */

typedef struct {
	uint16_t gap_ticks;      /* 4 kHz ticks since the previous transition */
	uint8_t  from_to;        /* (previous quadrature state << 4) | new state */
	uint8_t  flags;
	uint8_t  disc_pos;       /* pas_fwd_accum % 96: where on the disc this happened */
	uint16_t load_centikg;   /* converted pedal load */
	uint16_t torque_raw_mv;  /* raw sensor reading, before any filtering */
	uint16_t torque_fast;    /* the 35 ms filtered assist delta */
	uint16_t iq_setpoint;    /* what the motor was being asked for */
} pas_trace_sample_t;

typedef struct {
	uint8_t  from_state;     /* 0..3 */
	uint8_t  to_state;       /* 0..3 */
	bool     reverse;        /* the decoder's verdict for this step */
	uint16_t gap_ticks;
	uint16_t disc_pos;
	uint16_t load_centikg;
	uint16_t torque_raw_mv;
	uint16_t torque_fast;
	uint16_t iq_setpoint;
	bool     brake;
	bool     rolling;
	bool     latched;
} pas_trace_input_t;

void pas_trace_init(void);

/*
 * FW-111: claim a dynamic slot for the delayed-rearm recorder at the REAL initiating event (see
 * the file header). If the caller still owns a PREVIOUS reservation (a new ACTIVE->SUSPENDED
 * interrupting the previous saga's recovery watch), that ownership is ended FIRST - internally,
 * atomically, before a new slot is even considered - so the old and new sagas can never be
 * confused. Never picks the current active watcher, never a slot RETAINED from an ended
 * ownership. Resets the chosen slot, stamps it with the current session id, and seeds it with the
 * sample most recently written to the active watcher (the initiating transition itself) so its
 * PRE starts with the real event, not an empty ring. Returns the claimed slot index, or
 * PAS_TRACE_SLOTS if no slot could be claimed.
 */
uint8_t pas_trace_rearm_prearm(void);

/* True while the CURRENT saga owns a reservation from pas_trace_rearm_prearm(). */
bool pas_trace_rearm_held(void);

/* The currently OWNED slot index, or PAS_TRACE_SLOTS when none is held. Tests/observability. */
uint8_t pas_trace_rearm_slot_index(void);

/* A slot RETAINED from an ended ownership (still armed/ready, not yet streamed), or
 * PAS_TRACE_SLOTS when none is retained. Tests/observability only. */
uint8_t pas_trace_rearm_retained_slot_index(void);

/*
 * FW-111: force a trigger on the CURRENTLY OWNED slot NOW, for the delayed-rearm case. Same
 * return contract as pas_trace_transition() - the caller passes a real id straight to
 * pas_raw_freeze(). Returns PAS_TRACE_NO_CAPTURE when nothing is owned, or when the owned slot is
 * already armed / still ready from an earlier un-streamed rearm capture in the SAME saga (such a
 * refusal is counted at most once per reservation).
 */
uint8_t pas_trace_rearm_capture(const pas_trace_input_t *in);

/*
 * FW-111: the CURRENT saga's reservation is over. A never-armed slot is freed back to ordinary
 * use immediately. An armed or ready one has its OWNERSHIP ended (pas_trace_rearm_held() goes
 * false) but the slot itself becomes RETAINED - excluded from ordinary use and from the next
 * saga's candidate list, still fed by dual-write until its POST window completes, and only
 * finally freed once actually streamed via pas_trace_slot_release(). Never silently repurposed.
 */
void pas_trace_rearm_end_ownership(void);

/*
 * Call on EVERY quadrature transition, forward or backward, before the decoder acts.
 * FW-106: returns the capture id if THIS call armed a new capture, else PAS_TRACE_NO_CAPTURE.
 * The caller passes a real id straight to pas_raw_freeze() so both recorders stamp the same
 * physical event with the same key.
 */
uint8_t pas_trace_transition(const pas_trace_input_t *in);

/*
 * Call when the ride latch drops, so a loss with no transition of its own still triggers.
 * FW-106: same return contract as pas_trace_transition().
 */
uint8_t pas_trace_latch_loss(void);

/*
 * FW-106 multi-slot reader. Slots are independent: one may be frozen and waiting to be streamed
 * while another is watching. Index is 0..PAS_TRACE_SLOTS-1; out-of-range is safe and empty.
 */
bool     pas_trace_slot_ready(uint8_t slot);
uint16_t pas_trace_slot_count(uint8_t slot);
bool     pas_trace_slot_get(uint8_t slot, uint16_t index, pas_trace_sample_t *out);
uint16_t pas_trace_slot_trigger_index(uint8_t slot);
uint8_t  pas_trace_slot_capture_id(uint8_t slot);
void     pas_trace_slot_release(uint8_t slot);

/*
 * FW-106: which riding session a capture belongs to. Stamped when the capture is ARMED, not
 * when it is sent, because by the time the dump runs "the current session" no longer identifies
 * whoever produced the record - an interrupted dump can leave two sessions' records side by side.
 */
void    pas_trace_set_session_id(uint8_t session_id);
uint8_t pas_trace_slot_session_id(uint8_t slot);

/* True when the capture was cut short by the end of a ride rather than completing normally. */
bool pas_trace_slot_partial(uint8_t slot);

/*
 * FW-106: freeze any capture still collecting its post-window, short. Called when a session
 * closes: left running it would go on filling from the NEXT ride, producing one record spanning
 * two sessions - exactly the kind of mixed record this card exists to eliminate.
 */
void pas_trace_seal_open_captures(void);

/*
 * The oldest slot still waiting to be streamed, or -1 when none is. The dump drains in this
 * order so captures leave in the order they happened. The session-scoped form is what the dump
 * actually uses, so one session's captures can never appear inside another's brackets.
 */
int8_t   pas_trace_oldest_ready_slot(void);
int8_t   pas_trace_oldest_ready_slot_of(uint8_t session_id);
uint16_t pas_trace_count_session(uint8_t session_id);

/* Lifetime total of captures that reached the frozen state: the "accepted" side of the ledger. */
uint32_t pas_trace_captures_frozen(void);

/*
 * Triggers that found every slot already frozen. The event is counted; its detailed shape is
 * not kept. Non-zero means the slot budget was the binding limit during that ride.
 */
uint16_t pas_trace_capture_slots_full(void);

/*
 * Single-slot compatibility view: these act on the OLDEST slot still waiting to be streamed,
 * which is what a reader with no interest in slots wants. pas_trace_ready() is "any slot is
 * waiting", not "the one slot is waiting".
 */
bool     pas_trace_ready(void);
uint16_t pas_trace_count(void);
bool     pas_trace_get(uint16_t index, pas_trace_sample_t *out);
uint16_t pas_trace_trigger_index(void);
void     pas_trace_release(void);

#endif /* PAS_TRACE_H_ */
