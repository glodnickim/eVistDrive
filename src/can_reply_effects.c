#include "can_reply_effects.h"

/* See inc/can_reply_effects.h for the full contract. */

static can_multiframe_id_t pending_6029_id;

void can_reply_effects_init(void)
{
	pending_6029_id = CANMF_ID_NONE;
}

void can_reply_effects_6029_armed(can_multiframe_id_t xfer_id)
{
	pending_6029_id = xfer_id;
}

can_reply_effect_t can_reply_effects_poll(void)
{
	if (pending_6029_id == CANMF_ID_NONE) return CANFX_EFFECT_NONE;

	can_multiframe_xfer_state_t st = can_multiframe_transfer_state(pending_6029_id);
	if (st == CANMF_XFER_DONE) {
		pending_6029_id = CANMF_ID_NONE;
		return CANFX_EFFECT_DIAG_PEAK_RESET;
	}
	if (st == CANMF_XFER_ABORTED || st == CANMF_XFER_UNKNOWN) {
		/* The reply never reached its confirmed end (or can no longer be accounted for). The
		 * peak-hold data is KEPT - clearing it would be acting on a transfer that failed. Drop
		 * the pending id so an older, now-unaccountable id can never fire a reset later. */
		pending_6029_id = CANMF_ID_NONE;
		return CANFX_EFFECT_NONE;
	}
	/* PENDING - the transfer is still producing fragments. No effect yet. */
	return CANFX_EFFECT_NONE;
}