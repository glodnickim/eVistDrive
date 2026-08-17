#include "pas_raw.h"

#if CAN_DIAGNOSTICS_ENABLE

#include <string.h>
#include "diag_budget.h"

#ifdef PAS_RAW_COPY_HOOK
/* Test seam declaration - see the note at its call site in pas_raw_freeze(). Defined only by
 * tests/host/fw106_recorder_host.c, which passes -DPAS_RAW_COPY_HOOK to enable it. */
void pas_raw_copy_hook(uint16_t i);
#endif

/*
 * FW-106: see the header for what this measures and why it exists beside pas_trace.
 * Implementation notes only here.
 *
 * TWO WRITERS, ONE SLOT, NO LOCK — and why the earlier version lost events.
 *
 * The ISR appends; the main loop freezes. The first attempt gave the slot a COPYING state in
 * which the ISR deliberately left it alone while the pre-history was copied in. That looked
 * safe and was not: a transition arriving during the copy went into the ring and nowhere else.
 * It was too late for the PRE block already snapshotted and too early for a POST block that had
 * not started, so it vanished from the capture, leaving a gap in `seq` and no overrun to show
 * for it. The events most likely to fall in that window are the very short bounces this whole
 * card exists to catch.
 *
 * The fix is to make the two writers' regions DISJOINT instead of taking turns:
 *
 *   slot[0 .. pre_n)          written by the main loop, copying the pre-history
 *   slot[pre_n .. target)     written by the ISR, one event at a time
 *
 * fill and target are set, and the state moved to FILLING_POST, BEFORE the copy starts. From
 * that instant every new event lands at pre_n or above - the start of POST - while the copy
 * fills the block below it. Neither writer can touch the other's bytes, so no exclusion is
 * needed and interrupts are never masked.
 *
 * A capture is only readable once BOTH are finished, which is what `pre_copied` is for. Without
 * it a reader could in principle see a slot the ISR had already filled to its target while the
 * pre-history block was still half-written.
 *
 * WHY NO __disable_irq() AROUND THE INDEX SNAPSHOT. The one thing that must be consistent is the
 * (head, seq, filled) triple read before copying: taken from different instants they describe a
 * ring that never existed. A seqlock gives that without masking interrupts, which matters on a
 * motor controller whose FOC and Hall handling run from interrupts - a diagnostic build has no
 * business delaying them. The ISR bumps `seq` last on every event, so re-reading it and retrying
 * on a change proves nothing landed in between.
 *
 * OVERRUN IS NOT WRAPPING. The ring wrapping is normal; it is a rolling pre-history. What is not
 * normal is the ISR lapping far enough DURING the copy to overwrite ring entries still being
 * read out of it. That is detected afterwards by how far `seq` moved against the headroom that
 * existed, and it invalidates the capture instead of silently corrupting it.
 */

#define SLOT_FREE         0U
#define SLOT_FILLING_POST 1U
#define SLOT_READY        2U

/*
 * ALL of this module's mutable state, in one object. Grouping it is what lets the budget check
 * below measure the module rather than a hand-picked pair of arrays: adding a counter here grows
 * sizeof(S) and can therefore break the assert, which is the whole point of having one.
 */
static struct pas_raw_state {
	pas_raw_event_t ring[PAS_RAW_RING_LEN];
	pas_raw_event_t slot[PAS_RAW_SLOT_LEN];

	volatile uint16_t ring_head;    /* next write index */
	volatile uint16_t ring_filled;  /* valid entries, capped at PAS_RAW_RING_LEN */
	volatile uint16_t seq;          /* free-running event counter, wraps at 65536 */
	volatile uint32_t ring_wraps;

	volatile uint32_t slot_state;
	volatile uint16_t slot_fill;
	/*
	 * How long THIS capture is: whatever pre-history existed, plus a full PAS_RAW_FREEZE_POST.
	 * Fixed at freeze time rather than always filling the buffer, so a trigger that fires before
	 * a full pre-window has accumulated still gets its whole post-window. A short pre-history is
	 * reported short, never padded and never paid for out of POST.
	 */
	volatile uint16_t slot_target;
	volatile bool     pre_copied;   /* the main loop has finished its half */
	uint16_t slot_trigger_index;
	uint8_t  slot_capture_id;
	uint8_t  slot_session_id;
	bool     slot_partial;

	volatile uint16_t capture_overrun;
	volatile uint16_t freeze_skipped;

	uint8_t  last_state;
	bool     have_last_state;

	uint8_t  current_session_id;
	uint32_t captures_frozen;
} S;

_Static_assert(sizeof(S) <= DIAG_BUDGET_PAS_RAW_BYTES,
	"FW-106: pas_raw's total state exceeds its RAM line item");

void pas_raw_init(void)
{
	memset(&S, 0, sizeof(S));
	S.slot_state = SLOT_FREE;
	S.slot_target = PAS_RAW_SLOT_LEN;
	S.slot_capture_id = PAS_RAW_NO_CAPTURE;
	S.pre_copied = true;   /* nothing outstanding */
}

void pas_raw_set_session_id(uint8_t session_id)
{
	S.current_session_id = session_id;
}

void pas_raw_isr_sample(uint8_t state, uint32_t tick)
{
	state &= 0x3U;

	if (!S.have_last_state) {
		/* First ever sample: adopt the line state without inventing a transition from it. */
		S.last_state = state;
		S.have_last_state = true;
		return;
	}
	if (state == S.last_state) {
		return; /* the common case, and it must stay this cheap: one compare per 4 kHz tick */
	}

	pas_raw_event_t e;
	e.tick = tick;
	e.seq = S.seq;
	e.from_to = (uint8_t)((S.last_state << 4) | state);
	e.capture_id = PAS_RAW_NO_CAPTURE;
	S.last_state = state;

	S.ring[S.ring_head] = e;
	S.ring_head = (uint16_t)((S.ring_head + 1U) % PAS_RAW_RING_LEN);
	if (S.ring_filled < PAS_RAW_RING_LEN) {
		S.ring_filled++;
	} else if (S.ring_head == 0U) {
		/* A full ring just returned to index 0: one more complete lap of overwriting. */
		S.ring_wraps++;
	}

	/*
	 * The POST half. This runs while the main loop may still be copying the PRE half into
	 * slot[0..pre_n) - the two regions do not overlap, which is exactly why an event arriving
	 * mid-copy is kept here instead of being dropped as it was before.
	 */
	if (S.slot_state == SLOT_FILLING_POST) {
		e.capture_id = S.slot_capture_id;
		S.slot[S.slot_fill] = e;
		S.slot_fill++;
		if (S.slot_fill >= S.slot_target) {
			S.slot_state = SLOT_READY;
			S.captures_frozen++;
		}
	}

	/* Bumped LAST so the seqlock reader sees the counter move only after the data has. */
	S.seq = (uint16_t)(S.seq + 1U);
}

bool pas_raw_freeze(uint8_t capture_id)
{
	if (S.slot_state != SLOT_FREE) {
		if (S.freeze_skipped < 0xFFFFU) {
			S.freeze_skipped++;
		}
		return false;
	}

	/* Seqlock: a (head, seq, filled) triple the ISR did not change mid-read. */
	uint16_t head_snapshot;
	uint16_t seq_snapshot;
	uint16_t filled_snapshot;
	for (;;) {
		seq_snapshot = S.seq;
		head_snapshot = S.ring_head;
		filled_snapshot = S.ring_filled;
		if (S.seq == seq_snapshot) {
			break;
		}
	}

	if (filled_snapshot == 0U) {
		/* Nothing has happened on the lines yet; a capture would hold no pre-history at all. */
		return false;
	}

	uint16_t pre_n = (filled_snapshot < PAS_RAW_FREEZE_PRE) ? filled_snapshot : PAS_RAW_FREEZE_PRE;

	S.slot_capture_id = capture_id;
	S.slot_session_id = S.current_session_id;    /* stamped at arming, not at send time */
	S.slot_partial = false;
	S.slot_trigger_index = (uint16_t)(pre_n - 1U); /* the newest pre-event IS the trigger */

	/*
	 * Open the POST region BEFORE copying. From this store onwards the ISR appends at pre_n and
	 * above, so anything that happens during the copy below becomes the first POST event rather
	 * than disappearing between the two halves.
	 */
	S.pre_copied = false;
	S.slot_fill = pre_n;
	S.slot_target = (uint16_t)(pre_n + PAS_RAW_FREEZE_POST);
	S.slot_state = SLOT_FILLING_POST;

	uint16_t start = (uint16_t)((head_snapshot + PAS_RAW_RING_LEN - pre_n) % PAS_RAW_RING_LEN);
	for (uint16_t i = 0; i < pre_n; i++) {
		uint16_t src = (uint16_t)((start + i) % PAS_RAW_RING_LEN);
		S.slot[i] = S.ring[src];
		S.slot[i].capture_id = capture_id;
#ifdef PAS_RAW_COPY_HOOK
		/*
		 * TEST SEAM, compiled out of the firmware entirely. The defect this replaced could only
		 * be reproduced by an interrupt landing INSIDE this loop, and a single-threaded test that
		 * calls the ISR after pas_raw_freeze() has returned cannot do that. See
		 * tests/host/fw106_recorder_host.c.
		 */
		pas_raw_copy_hook(i);
#endif
	}

	/*
	 * How far did the ISR get while that copy ran? It writes forward from head_snapshot, so it
	 * only reaches the oldest entry being copied after (RING_LEN - pre_n) further events.
	 * Anything beyond that means part of what was copied had already been replaced - real loss,
	 * as opposed to the ordinary wrapping counted by ring_wraps.
	 */
	uint16_t advanced = (uint16_t)(S.seq - seq_snapshot);
	if (advanced > (uint16_t)(PAS_RAW_RING_LEN - pre_n)) {
		if (S.capture_overrun < 0xFFFFU) {
			S.capture_overrun++;
		}
	}

	S.pre_copied = true;
	return true;
}

void pas_raw_seal_open_capture(void)
{
	/*
	 * A capture still collecting its post-window when the ride ends would otherwise carry on
	 * into the next one, and a single record covering two sessions is the defect this card
	 * removes. Kept short and flagged, rather than thrown away.
	 */
	if (S.slot_state == SLOT_FILLING_POST) {
		S.slot_partial = true;
		S.slot_state = SLOT_READY;
		S.captures_frozen++;
	}
}

/* Readable only once BOTH writers have finished - see the note at the top of this file. */
bool pas_raw_slot_ready(void)
{
	return (S.slot_state == SLOT_READY) && S.pre_copied;
}

uint16_t pas_raw_slot_count(void)
{
	return pas_raw_slot_ready() ? S.slot_fill : 0U;
}

bool pas_raw_slot_get(uint16_t index, pas_raw_event_t *out)
{
	if (!pas_raw_slot_ready() || out == 0 || index >= S.slot_fill) {
		return false;
	}
	*out = S.slot[index];
	return true;
}

uint16_t pas_raw_slot_trigger_index(void)
{
	return pas_raw_slot_ready() ? S.slot_trigger_index : 0U;
}

uint8_t pas_raw_slot_capture_id(void)
{
	return pas_raw_slot_ready() ? S.slot_capture_id : PAS_RAW_NO_CAPTURE;
}

uint8_t pas_raw_slot_session_id(void)
{
	return pas_raw_slot_ready() ? S.slot_session_id : 0U;
}

bool pas_raw_slot_partial(void)
{
	return pas_raw_slot_ready() && S.slot_partial;
}

uint16_t pas_raw_count_session(uint8_t session_id)
{
	return (pas_raw_slot_ready() && (S.slot_session_id == session_id)) ? 1U : 0U;
}

void pas_raw_slot_release(void)
{
	if (S.slot_state != SLOT_READY) {
		return;
	}
	S.slot_fill = 0;
	S.slot_target = PAS_RAW_SLOT_LEN;
	S.slot_trigger_index = 0;
	S.slot_capture_id = PAS_RAW_NO_CAPTURE;
	S.slot_partial = false;
	S.pre_copied = true;
	S.slot_state = SLOT_FREE;
}

uint32_t pas_raw_ring_wraps(void)      { return S.ring_wraps; }
uint16_t pas_raw_capture_overrun(void) { return S.capture_overrun; }
uint16_t pas_raw_freeze_skipped(void)  { return S.freeze_skipped; }
uint32_t pas_raw_captures_frozen(void) { return S.captures_frozen; }

#else  /* !CAN_DIAGNOSTICS_ENABLE */

/*
 * FW-106: the raw recorder is diagnostic-only, and the normal firmware must not carry its 4 KB
 * of buffers. The build script does link with --gc-sections, which would drop them anyway since
 * nothing references them - but a RAM guarantee that depends on a linker flag somebody could
 * remove is not a guarantee. Compiling the module out says it outright.
 *
 * ISO C forbids an empty translation unit, hence the typedef.
 */
typedef int pas_raw_not_compiled_in;

#endif /* CAN_DIAGNOSTICS_ENABLE */
