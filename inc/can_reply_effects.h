#ifndef CAN_REPLY_EFFECTS_H_
#define CAN_REPLY_EFFECTS_H_

#include "can_multiframe.h"

/*
 * FW-110 v4: tiny, linkable holder for side effects that must fire only when a specific
 * multiframe reply is CONFIRMED to have reached its end - not when it was merely armed.
 *
 * WHY THIS EXISTS. 0x6029's handler used to do:
 *
 *     if (send_multiframe(...)) { diag_peak_reset = 1; }
 *
 * send_multiframe() returning true only proves the producer was ARMED. If the transfer is later
 * aborted (a fragment exhausts its retries in can_tx_queue), the client never receives a
 * complete snapshot, but the peaks were already cleared - data the requester never got is gone.
 *
 * This module owns the "remember the 0x6029 transfer id, and emit the reset request only when
 * that exact transfer resolves DONE" decision, decoupled from main.c's diag_peak_reset global
 * (it returns an effect; the caller applies it - see main.c's main loop). Each effect is
 * emitted AT MOST ONCE per armed reply: DONE -> CANFX_EFFECT_DIAG_PEAK_RESET exactly once;
 * ABORTED/UNKNOWN -> nothing, and the pending id is dropped so a stale id can never fire later.
 */

typedef enum {
	CANFX_EFFECT_NONE = 0,
	CANFX_EFFECT_DIAG_PEAK_RESET   /* the armed 0x6029 reply was confirmed fully delivered */
} can_reply_effect_t;

/* Clean state (called once at startup, and by tests). */
void can_reply_effects_init(void);

/* Call when a 0x6029 snapshot was actually ARMED, with the id can_multiframe_start() handed
 * out. At most one pending 0x6029 is tracked - a new arm replaces the previous pending one. */
void can_reply_effects_6029_armed(can_multiframe_id_t xfer_id);

/* Call once per main-loop iteration, AFTER can_multiframe_step() has had a chance to resolve
 * the transfer (reads can_multiframe_transfer_state() on the real producer). Returns the one
 * effect whose precondition just became true, consuming it. Never blocks, never touches
 * hardware, does not depend on any diagnostic global. */
can_reply_effect_t can_reply_effects_poll(void);

#endif /* CAN_REPLY_EFFECTS_H_ */