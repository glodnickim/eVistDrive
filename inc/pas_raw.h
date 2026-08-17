#ifndef PAS_RAW_H_
#define PAS_RAW_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * FW-106: the raw PAS line recorder — what the two quadrature GPIOs actually did, stamped by
 * the 4 kHz interrupt itself.
 *
 * WHY A SECOND RECORDER, BELOW pas_trace. pas_trace records what the DECODER saw, and the
 * decoder runs in reg_ADC_processing() from the main loop. The main loop is exactly what the
 * blocking CAN sends were stalling (FW-104 measured it), so every "the decoder missed a step"
 * hypothesis was untestable: the instrument shared the fault with the thing being measured.
 * This module is written from TIMER1_IRQHandler, which cannot be delayed by the main loop, so
 * the difference between the two IS the measurement:
 *
 *     tick deltas here            what really happened on the wires
 *     gap_ticks in pas_trace      what the decoder believed happened
 *     the two disagreeing         the decoder lost or displaced a transition
 *
 * MEASUREMENT ONLY. No decision anywhere reads any of this, and the production decoder in
 * main.c is untouched — this runs alongside it, never instead of it.
 *
 * PAIRING. A capture here is armed by the SAME trigger that arms a pas_trace capture, and both
 * are stamped with the same capture id, so the two views of one physical event can be joined
 * offline by key rather than by arrival order on the bus (which the ride log proved unreliable).
 *
 * OWN FILE, like pas_trace and ride_episode, and for the same reason: the FW-101 recorder was
 * written inline in main.c and shipped with three defects its own output could not reveal.
 * tests/host/fw106_raw_session_host.c drives this one on a PC.
 */

/* Free-running history. 256 events x 8 B = 2048 B. */
#define PAS_RAW_RING_LEN    256U
/* Events kept from before the trigger, and still recorded after it. Same shape as pas_trace. */
#define PAS_RAW_FREEZE_PRE  128U
#define PAS_RAW_FREEZE_POST 128U
#define PAS_RAW_SLOT_LEN    (PAS_RAW_FREEZE_PRE + PAS_RAW_FREEZE_POST)

/*
 * Never used as a real capture id, so it is free to mean "this event belongs to no capture".
 * Matches PAS_TRACE_NO_CAPTURE by construction: the ids come from pas_trace, which never
 * allocates this value.
 */
#define PAS_RAW_NO_CAPTURE  0xFFU

typedef struct {
	uint32_t tick;       /* control_time_ticks read INSIDE the ISR at this transition */
	uint16_t seq;        /* free-running event counter; a gap in it means events were lost */
	uint8_t  from_to;    /* (previous state << 4) | new state, states 0..3 - as in pas_trace */
	uint8_t  capture_id; /* PAS_RAW_NO_CAPTURE in the ring; the real id in a frozen slot */
} pas_raw_event_t;

_Static_assert(sizeof(pas_raw_event_t) == 8,
	"pas_raw_event_t must stay 8 B - one event is one CAN frame (0x1021C)");

void pas_raw_init(void);

/*
 * Call from TIMER1_IRQHandler on EVERY tick with the two PAS lines packed as
 * (A ? 1 : 0) | (B ? 2 : 0) and the current control_time_ticks. Returns immediately when the
 * state has not changed, which is the overwhelmingly common case - the cost on an unchanged
 * tick is one compare.
 */
void pas_raw_isr_sample(uint8_t state, uint32_t tick);

/*
 * Freeze a capture around the events already in the ring, stamped with capture_id. Call from
 * the main loop the moment pas_trace reports a trigger, passing the id it issued.
 * Returns true when a capture was started; false when the single slot is still occupied
 * (pas_raw_freeze_skipped() then counts the miss).
 */
bool pas_raw_freeze(uint8_t capture_id);

/* True once the capture has collected its POST events and is waiting to be streamed out. */
bool pas_raw_slot_ready(void);
uint16_t pas_raw_slot_count(void);
bool pas_raw_slot_get(uint16_t index, pas_raw_event_t *out);
uint16_t pas_raw_slot_trigger_index(void);
uint8_t pas_raw_slot_capture_id(void);
void pas_raw_slot_release(void);

/*
 * FW-106: which riding session this capture belongs to, stamped when it is FROZEN rather than
 * when it is sent - by dump time "the current session" no longer identifies who produced it.
 */
void     pas_raw_set_session_id(uint8_t session_id);
uint8_t  pas_raw_slot_session_id(void);
uint16_t pas_raw_count_session(uint8_t session_id);

/* True when the capture was cut short by the end of a ride rather than completing normally. */
bool pas_raw_slot_partial(void);

/*
 * FW-106: seal a capture still collecting its post-window. Called when a session closes, so a
 * capture armed at the end of one ride cannot keep filling from the next one.
 */
void pas_raw_seal_open_capture(void);

/* Lifetime total of captures that reached the ready state: the "accepted" side of the ledger. */
uint32_t pas_raw_captures_frozen(void);

/*
 * Ring wrapped and started overwriting its oldest entries. NORMAL OPERATION, not a fault - the
 * ring is a rolling pre-history and is meant to wrap. Informational only.
 */
uint32_t pas_raw_ring_wraps(void);

/*
 * The ISR overwrote part of the pre-history WHILE it was being copied into the slot, so that
 * capture is not trustworthy. This is REAL DATA LOSS and must stay at zero; non-zero
 * invalidates the affected capture.
 */
uint16_t pas_raw_capture_overrun(void);

/* Triggers that found the slot already occupied: counted, but with no raw trace of their own. */
uint16_t pas_raw_freeze_skipped(void);

#endif /* PAS_RAW_H_ */
