#include "pas_trace.h"
#include <string.h>

/*
 * FW-102: see the header for what this measures and why. Implementation notes only here.
 *
 * THE RING. Capacity is exactly PAS_TRACE_PRE + PAS_TRACE_POST (256). That is not a rounding
 * choice: while WATCHING, every transition overwrites the oldest slot, so the ring always holds
 * the most recent up-to-256 transitions. The moment a trigger is written, PAS_TRACE_POST further
 * writes (counting the trigger write itself) exactly fill the rest of the ring without touching
 * anything older than PAS_TRACE_PRE behind the trigger — so freezing needs no copy, no second
 * buffer, and no separate "how much pre-history do we actually have" bookkeeping beyond the
 * ordinary fill counter every ring buffer needs.
 *
 * TWO_BIT and SHORT_GAP are computed HERE, not by the caller, so the quadrature-specific
 * "impossible transition" reasoning has exactly one owner. disc_pos is taken modulo 96 here for
 * the same reason: the caller passes the free-running accumulator, this module owns what "one
 * revolution" means for the purpose of spotting a position that keeps recurring.
 *
 * FW-106 — SLOTS. Each slot is a complete, independent instance of the ring above, so the
 * freeze-in-place trick survives untouched: nothing is ever copied. Exactly one slot WATCHES at
 * a time (`active`); the others are either free or frozen and waiting to be streamed. When the
 * watcher freezes, the next free slot takes over in the same call, so recording never pauses
 * while any slot is left. With none left there is no watcher at all — transitions are then only
 * examined, not stored, purely so a trigger that cannot be captured is still COUNTED.
 */

typedef struct {
	pas_trace_sample_t ring[PAS_TRACE_LEN];
	uint16_t head;         /* next write slot */
	uint16_t filled;       /* valid samples written since the last release, capped at LEN */
	bool     armed;        /* a trigger has fired; counting down PAS_TRACE_POST */
	uint16_t post_left;    /* remaining writes (incl. the trigger write) before freezing */
	uint16_t trigger_slot; /* absolute ring index of the triggering sample */
	bool     ready;        /* capture frozen, waiting to be streamed and released */
	/* Snapshotted at freeze time so the readers need no further arithmetic. */
	uint16_t oldest_slot;
	uint16_t count_snapshot;
	uint16_t trigger_index_snapshot;
	uint8_t  capture_id;   /* FW-106: shared with the raw ISR snapshot of the same event */
	uint32_t frozen_seq;   /* FW-106: freeze order, so "oldest ready" is unambiguous */
	uint8_t  session_id;   /* FW-106: stamped when ARMED - see the header */
	bool     partial;      /* FW-106: sealed by the end of a ride, not completed normally */
} pas_trace_slot_t;

/*
 * ALL of this module's mutable state, in one object. Grouping it is what lets the RAM budget
 * check below measure the module rather than a hand-picked subset of it: any new field added
 * here grows sizeof(T) and can therefore trip the assert, which is the point of having one.
 */
static struct {
	pas_trace_slot_t slots[PAS_TRACE_SLOTS];
	uint8_t  active;              /* watching slot, or PAS_TRACE_SLOTS when none is free */
	uint8_t  next_capture_id;
	uint32_t freeze_counter;
	uint16_t capture_slots_full;
	uint8_t  current_session_id;
	uint32_t captures_frozen;
	/* FW-111 v5: the DYNAMICALLY chosen slot OWNED by the current saga's reservation, or
	 * PAS_TRACE_SLOTS when none is held. Never a fixed index (see the header) - chosen fresh at
	 * every pas_trace_rearm_prearm() call. Deliberately SEPARATE from rearm_retained_slot below -
	 * "who owns the reservation" and "does a slot still need to keep collecting / wait for the
	 * dump" are two different facts (the v4->v5 fix: a single overloaded field let a new saga's
	 * ownership be mistaken for an old, still-unstreamed capture's slot, or vice versa). */
	uint8_t  rearm_slot;
	/* FW-111 v5: a slot armed or ready from an OWNERSHIP THAT HAS ALREADY ENDED, not yet streamed.
	 * Excluded from ordinary use and from a future pas_trace_rearm_prearm()'s candidate list,
	 * still fed by dual-write until its POST window completes, cleared only by
	 * pas_trace_slot_release() once actually streamed. PAS_TRACE_SLOTS when nothing is retained. */
	uint8_t  rearm_retained_slot;
	bool     rearm_refusal_reported; /* the current reservation has already counted its refusal */
} T;

#include "diag_budget.h"
_Static_assert(sizeof(T) <= DIAG_BUDGET_PAS_TRACE_BYTES,
	"FW-106: pas_trace's total state exceeds its RAM line item");

static void reset_slot(pas_trace_slot_t *s)
{
	memset(s->ring, 0, sizeof(s->ring));
	s->head = 0;
	s->filled = 0;
	s->armed = false;
	s->post_left = 0;
	s->trigger_slot = 0;
	s->ready = false;
	s->oldest_slot = 0;
	s->count_snapshot = 0;
	s->trigger_index_snapshot = 0;
	s->capture_id = PAS_TRACE_NO_CAPTURE;
	s->frozen_seq = 0;
	s->session_id = 0;
	s->partial = false;
}

void pas_trace_init(void)
{
	for (uint8_t i = 0; i < PAS_TRACE_SLOTS; i++) {
		reset_slot(&T.slots[i]);
	}
	T.active = 0;
	T.next_capture_id = 0;
	T.freeze_counter = 0;
	T.capture_slots_full = 0;
	T.current_session_id = 0;
	T.captures_frozen = 0;
	T.rearm_slot = PAS_TRACE_SLOTS;
	T.rearm_retained_slot = PAS_TRACE_SLOTS;
	T.rearm_refusal_reported = false;
}

void pas_trace_set_session_id(uint8_t session_id)
{
	T.current_session_id = session_id;
}

/*
 * FW-111 v5: end the CURRENT saga's ownership, if any. A slot that was never armed is genuinely
 * idle - freed back to ordinary use right now. One that IS armed or ready keeps existing
 * (RETAINED) until pas_trace_slot_release() actually streams it - ownership ends, but the data
 * does not evaporate and is never silently repurposed. Shared by the explicit
 * pas_trace_rearm_end_ownership() call and by pas_trace_rearm_prearm()'s own "end the old saga
 * first" step (v5 fix for the v4 defect where a lingering rearm_slot from an ended ownership could
 * be mistaken for - or silently block - a brand new saga's reservation).
 */
static void end_ownership(void)
{
	if (T.rearm_slot >= PAS_TRACE_SLOTS) {
		return;
	}
	pas_trace_slot_t *s = &T.slots[T.rearm_slot];
	if (!s->armed && !s->ready) {
		reset_slot(s);
	} else {
		/* Ownership ends, but the slot's data must survive for its POST window / the dump - hand
		 * it to "retained" bookkeeping so find_free_slot() and a future prearm() keep excluding
		 * it without it being confused with the CURRENT saga. Structurally at most one retained
		 * slot can ever exist while PAS_TRACE_SLOTS <= 2 (one slot is always the active watcher),
		 * so there is nothing to evict here even in principle. */
		T.rearm_retained_slot = T.rearm_slot;
	}
	T.rearm_slot = PAS_TRACE_SLOTS;
}

/*
 * FW-111 v5: claim a slot for the delayed-rearm recorder at the REAL initiating event. Never a
 * fixed index - never the current active watcher, never a slot RETAINED from an ended ownership,
 * never a slot still ready/armed from something else. If the caller still owns an UNFINISHED
 * reservation from a previous saga (a new ACTIVE->SUSPENDED during that saga's recovery watch -
 * see the file header), that ownership is ended FIRST, atomically, before a new slot is even
 * considered - the old and new sagas can never be confused, and the old capture (if armed/ready)
 * is never stolen or overwritten. Resets the chosen slot for a clean history boundary, stamps it
 * with the current session, then seeds it with whatever the active watcher just wrote: this
 * function is called on the SAME tick as the initiating transition, immediately after that
 * transition's own pas_trace_transition call, so the active watcher's most recent sample IS the
 * initiating reverse/invalid. Without the seed the shadow slot would only ever hold what happens
 * AFTER the event it exists to explain.
 */
uint8_t pas_trace_rearm_prearm(void)
{
	if (T.rearm_slot < PAS_TRACE_SLOTS) {
		end_ownership();
	}

	uint8_t chosen = PAS_TRACE_SLOTS;
	for (uint8_t i = 0; i < PAS_TRACE_SLOTS; i++) {
		if (i == T.active) continue;               /* never take over the ordinary watcher */
		if (i == T.rearm_retained_slot) continue;   /* never touch a retained, unstreamed capture */
		if (T.slots[i].ready) continue;             /* still waiting to be streamed */
		if (T.slots[i].armed) continue;             /* mid-capture from an unrelated ordinary trigger */
		chosen = i;
		break;
	}
	if (chosen >= PAS_TRACE_SLOTS) {
		/* Nothing free at the initiating event itself - counted once here (this function is only
		 * ever called once per saga, from main.c's one-shot prearm edge, so no retry storm can
		 * inflate this the way a polled call could). The caller reports NO_TRACE_NO_HISTORY. */
		if (T.capture_slots_full < 0xFFFFU) {
			T.capture_slots_full++;
		}
		return PAS_TRACE_SLOTS;
	}

	pas_trace_slot_t *s = &T.slots[chosen];
	reset_slot(s);
	s->session_id = T.current_session_id;

	if (T.active < PAS_TRACE_SLOTS && T.slots[T.active].filled > 0) {
		pas_trace_slot_t *src = &T.slots[T.active];
		uint16_t last = (uint16_t)((src->head + PAS_TRACE_LEN - 1U) % PAS_TRACE_LEN);
		s->ring[s->head] = src->ring[last];
		s->head = (uint16_t)((s->head + 1U) % PAS_TRACE_LEN);
		s->filled = 1U;
	}

	T.rearm_slot = chosen;
	T.rearm_refusal_reported = false;
	return chosen;
}

bool pas_trace_rearm_held(void)
{
	return T.rearm_slot < PAS_TRACE_SLOTS;
}

uint8_t pas_trace_rearm_slot_index(void)
{
	return T.rearm_slot;
}

uint8_t pas_trace_rearm_retained_slot_index(void)
{
	return T.rearm_retained_slot;
}

/*
 * FW-111 v5: the CURRENT saga's reservation is over (its record closed, or the saga concluded
 * with no record ever tracking it). Just end_ownership() - see that function for the
 * never-armed-vs-retained split.
 */
void pas_trace_rearm_end_ownership(void)
{
	end_ownership();
}

/*
 * FW-106: PAS_TRACE_NO_CAPTURE is a sentinel elsewhere, so it must never be handed out as a
 * real id. Skipping it here — rather than at every use site — keeps that guarantee in one place.
 */
static uint8_t alloc_capture_id(void)
{
	uint8_t id = T.next_capture_id;
	T.next_capture_id++;
	if (T.next_capture_id == PAS_TRACE_NO_CAPTURE) {
		T.next_capture_id = 0;
	}
	return id;
}

/* The next slot that can take over watching, or PAS_TRACE_SLOTS when every one is frozen. Both
 * the currently OWNED rearm slot and a RETAINED (ended-ownership but unstreamed) one are excluded
 * from ordinary use - an armed/ready capture is never handed to the ordinary watcher regardless of
 * which of the two facts currently applies to it (see pas_trace_rearm_end_ownership). */
static uint8_t find_free_slot(void)
{
	for (uint8_t i = 0; i < PAS_TRACE_SLOTS; i++) {
		if (i == T.rearm_slot) continue;
		if (i == T.rearm_retained_slot) continue;
		if (!T.slots[i].ready) {
			return i;
		}
	}
	return PAS_TRACE_SLOTS;
}

/* Snapshot the slot's frozen view and mark it ready. Does NOT touch T.active: the reserved rearm
 * slot is frozen this way too, and it is never the active watcher, so handing watching to a fresh
 * slot here would be wrong for it. The caller decides whether the frozen slot WAS the watcher. */
static void freeze_snapshot(pas_trace_slot_t *s)
{
	s->oldest_slot = (s->filled >= PAS_TRACE_LEN) ? s->head : 0U;
	s->count_snapshot = s->filled;
	s->trigger_index_snapshot =
		(uint16_t)((s->trigger_slot + PAS_TRACE_LEN - s->oldest_slot) % PAS_TRACE_LEN);
	s->ready = true;
	s->armed = false;
	s->frozen_seq = ++T.freeze_counter;
	T.captures_frozen++;
}

static void freeze(pas_trace_slot_t *s)
{
	freeze_snapshot(s);
	/*
	 * Hand watching straight to a free slot. Doing it here rather than at the next transition
	 * means the very next sample after a freeze is still recorded, which is the whole reason
	 * for having more than one slot.
	 */
	T.active = find_free_slot();
}

	/* Append a raw sample to a slot's ring and advance it. Used for the ordinary watcher and for
	 * the FW-111 v3 dual-write into the held rearm slot (which accumulates pre-problem history). */
	static void append_sample(pas_trace_slot_t *s, const pas_trace_input_t *in, uint8_t flags)
	{
		pas_trace_sample_t *slot = &s->ring[s->head];
		slot->gap_ticks = in->gap_ticks;
		slot->from_to = (uint8_t)((in->from_state << 4) | in->to_state);
		slot->flags = flags;
		slot->disc_pos = (uint8_t)(in->disc_pos % 96U);
		slot->load_centikg = in->load_centikg;
		slot->torque_raw_mv = in->torque_raw_mv;
		slot->torque_fast = in->torque_fast;
		slot->iq_setpoint = in->iq_setpoint;
		s->head = (uint16_t)((s->head + 1U) % PAS_TRACE_LEN);
		if (s->filled < PAS_TRACE_LEN) s->filled++;
	}

uint8_t pas_trace_transition(const pas_trace_input_t *in)
{
	if (in == 0) {
		return PAS_TRACE_NO_CAPTURE;
	}

	uint8_t computed = 0;
	/* Both quadrature bits changing in one sample cannot happen from real crank motion. */
	if (((in->from_state ^ in->to_state) & 0x3U) == 0x3U) computed |= PAS_TR_TWO_BIT;
	if (in->gap_ticks <= PAS_TRACE_SHORT_GAP_TICKS) computed |= PAS_TR_SHORT_GAP;
	if (in->reverse) computed |= PAS_TR_REVERSE;
	if (in->brake) computed |= PAS_TR_BRAKE;
	if (in->rolling) computed |= PAS_TR_ROLLING;
	if (in->latched) computed |= PAS_TR_LATCHED;

	bool suspicious = (computed & (PAS_TR_TWO_BIT | PAS_TR_SHORT_GAP)) != 0;

	/*
	 * FW-111 v5 dual-write: EVERY transition is also written into whichever rearm-related slots
	 * are alive right now - the CURRENTLY OWNED reservation (accumulating PRE, or POST once
	 * armed) AND/OR a slot RETAINED from an ended ownership that is still armed and must still
	 * receive its own POST regardless (requirement: ownership ending and "still collecting" are
	 * independent facts - an ended-ownership shadow must still get its complete POST window, and
	 * the ordinary watcher must never be handed a slot that is still filling one). No trigger
	 * logic here: these slots are only ever armed by pas_trace_rearm_capture(). Neither
	 * pas_trace_rearm_prearm() nor end_ownership() ever choose/retain the active watcher, so
	 * T.active never equals either - the checks stay as defensive belt-and-suspenders.
	 */
	if (T.rearm_slot < PAS_TRACE_SLOTS && T.active != T.rearm_slot) {
		pas_trace_slot_t *r = &T.slots[T.rearm_slot];
		if (!r->ready) {
			append_sample(r, in, computed);
			if (r->armed) {
				r->post_left--;
				if (r->post_left == 0) {
					/* The reserved slot is a shadow recorder, never the active watcher - do not
					 * hand watching off to a fresh slot for it. */
					freeze_snapshot(r);
				}
			}
		}
	}
	if (T.rearm_retained_slot < PAS_TRACE_SLOTS && T.rearm_retained_slot != T.active &&
	    T.rearm_retained_slot != T.rearm_slot) {
		pas_trace_slot_t *r = &T.slots[T.rearm_retained_slot];
		if (!r->ready) {
			append_sample(r, in, computed);
			if (r->armed) {
				r->post_left--;
				if (r->post_left == 0) {
					freeze_snapshot(r);
				}
			}
		}
	}

	if (T.active >= PAS_TRACE_SLOTS) {
		/*
		 * Every slot is frozen and waiting for the dump. The sample cannot be kept, but the
		 * fact that something suspicious happened must not vanish: that count is how a reader
		 * knows the slot budget, not the bike, is why there is no trace for it.
		 */
		if (suspicious && T.capture_slots_full < 0xFFFFU) {
			T.capture_slots_full++;
		}
		return PAS_TRACE_NO_CAPTURE;
	}

	pas_trace_slot_t *s = &T.slots[T.active];
	bool was_idle = !s->armed;
	bool triggers_now = was_idle && suspicious;
	uint8_t issued = PAS_TRACE_NO_CAPTURE;
	if (triggers_now) {
		computed |= PAS_TR_TRIGGER;
	}

	append_sample(s, in, computed);

	if (triggers_now) {
		s->trigger_slot = (uint16_t)((s->head + PAS_TRACE_LEN - 1U) % PAS_TRACE_LEN);
		s->armed = true;
		s->post_left = PAS_TRACE_POST;
		issued = alloc_capture_id();
		s->capture_id = issued;
		s->session_id = T.current_session_id;   /* FW-106: stamped when ARMED, not when sent */
	}

	if (s->armed) {
		s->post_left--;
		if (s->post_left == 0) freeze(s);
	}

	return issued;
}

	/* The sample flags for a context tag built outside a transition (the rearm capture). */
	static uint8_t context_flags(const pas_trace_input_t *in)
	{
		uint8_t f = 0;
		if (in->reverse) f |= PAS_TR_REVERSE;
		if (in->brake) f |= PAS_TR_BRAKE;
		if (in->rolling) f |= PAS_TR_ROLLING;
		if (in->latched) f |= PAS_TR_LATCHED;
		return f;
	}

	/*
	 * FW-111 v4: force a trigger on the reserved slot for the first genuinely delayed rearm. It
	 * has been seeded with the initiating event and dual-written with the forward-confirming
	 * history since pas_trace_rearm_prearm(), so arming it NOW captures the whole chain plus the
	 * recovery POST window. Returns the capture id for pas_raw_freeze(). Refusals (slot already
	 * armed or still ready from an earlier un-streamed rearm capture in the same reservation) are
	 * counted at most once per reservation, so a caller polling this every tick cannot inflate
	 * capture_slots_full.
	 */
	uint8_t pas_trace_rearm_capture(const pas_trace_input_t *in)
	{
		if (in == 0 || T.rearm_slot >= PAS_TRACE_SLOTS) {
			return PAS_TRACE_NO_CAPTURE;
		}
		pas_trace_slot_t *s = &T.slots[T.rearm_slot];
		if (s->armed || s->ready) {
			if (!T.rearm_refusal_reported) {
				T.rearm_refusal_reported = true;
				if (T.capture_slots_full < 0xFFFFU) {
					T.capture_slots_full++;
				}
			}
			return PAS_TRACE_NO_CAPTURE;
		}
		/* The slot is the reserved one - it was never handed to an ordinary trigger while held,
		 * so it can only have accumulated the dual-written pre-problem history. It is a SHADOW
		 * recorder: it must NOT take over the active watcher (that would strand any ordinary
		 * capture that is mid-arming); the dual-write keeps feeding it until it freezes. */

		pas_trace_sample_t *slot = &s->ring[s->head];
		slot->gap_ticks = in->gap_ticks;
		slot->from_to = (uint8_t)((in->from_state << 4) | in->to_state);
		slot->flags = (uint8_t)(context_flags(in) | PAS_TR_TRIGGER);
		slot->disc_pos = (uint8_t)(in->disc_pos % 96U);
		slot->load_centikg = in->load_centikg;
		slot->torque_raw_mv = in->torque_raw_mv;
		slot->torque_fast = in->torque_fast;
		slot->iq_setpoint = in->iq_setpoint;

		s->trigger_slot = s->head;
		s->armed = true;
		s->post_left = PAS_TRACE_POST;
		uint8_t issued = alloc_capture_id();
		s->capture_id = issued;
		/* session_id is NOT re-stamped here (unlike the ordinary pas_trace_transition() path) -
		 * requirement 8: the reservation's session is fixed at pas_trace_rearm_prearm() time and
		 * must stay that way through to the eventual arm, which can be many ticks (and even a
		 * diag session boundary) later. Re-stamping here would silently reassign a capture that
		 * began in one session to whatever session happens to be current when it finally arms. */

		s->head = (uint16_t)((s->head + 1U) % PAS_TRACE_LEN);
		if (s->filled < PAS_TRACE_LEN) s->filled++;
		s->post_left--;
		return issued;
	}

uint8_t pas_trace_latch_loss(void)
{
	if (T.active >= PAS_TRACE_SLOTS) {
		/* No watcher at all: nothing to attach the flag to, but still a counted event. */
		if (T.capture_slots_full < 0xFFFFU) {
			T.capture_slots_full++;
		}
		return PAS_TRACE_NO_CAPTURE;
	}

	pas_trace_slot_t *s = &T.slots[T.active];
	if (s->filled == 0) {
		/* Nothing has been recorded yet to attach this to. */
		return PAS_TRACE_NO_CAPTURE;
	}

	uint16_t last = (uint16_t)((s->head + PAS_TRACE_LEN - 1U) % PAS_TRACE_LEN);
	s->ring[last].flags |= PAS_TR_LATCH_LOSS;
	if (s->armed) {
		return PAS_TRACE_NO_CAPTURE;
	}

	/* Retroactive trigger: the sample is already written, so PAS_TRACE_POST counts
	 * only the writes still to come, none of which have happened yet. */
	s->ring[last].flags |= PAS_TR_TRIGGER;
	s->trigger_slot = last;
	s->armed = true;
	s->post_left = PAS_TRACE_POST;
	uint8_t issued = alloc_capture_id();
	s->capture_id = issued;
	s->session_id = T.current_session_id;
	return issued;
}

void pas_trace_seal_open_captures(void)
{
	/*
	 * FW-106: a capture armed in the last seconds of a ride is still counting down its
	 * post-window when the bike stops. Left alone it would go on filling from the NEXT ride and
	 * publish one record straddling two sessions - the exact class of mixed record this card
	 * exists to remove. Sealing keeps what was actually seen, marked short: a truncated view of
	 * a real event is worth more than none, and its length says plainly that it was cut off.
	 */
	for (uint8_t i = 0; i < PAS_TRACE_SLOTS; i++) {
		if (T.slots[i].armed && !T.slots[i].ready) {
			T.slots[i].partial = true;
			freeze(&T.slots[i]);
		}
	}
}

uint8_t pas_trace_slot_session_id(uint8_t slot)
{
	return pas_trace_slot_ready(slot) ? T.slots[slot].session_id : 0U;
}

bool pas_trace_slot_partial(uint8_t slot)
{
	return pas_trace_slot_ready(slot) && T.slots[slot].partial;
}

int8_t pas_trace_oldest_ready_slot_of(uint8_t session_id)
{
	int8_t best = -1;
	uint32_t best_seq = 0;
	for (uint8_t i = 0; i < PAS_TRACE_SLOTS; i++) {
		if (!T.slots[i].ready || T.slots[i].session_id != session_id) continue;
		if (best < 0 || T.slots[i].frozen_seq < best_seq) {
			best = (int8_t)i;
			best_seq = T.slots[i].frozen_seq;
		}
	}
	return best;
}

uint16_t pas_trace_count_session(uint8_t session_id)
{
	uint16_t n = 0;
	for (uint8_t i = 0; i < PAS_TRACE_SLOTS; i++) {
		if (T.slots[i].ready && T.slots[i].session_id == session_id) n++;
	}
	return n;
}

uint32_t pas_trace_captures_frozen(void)
{
	return T.captures_frozen;
}

bool pas_trace_slot_ready(uint8_t slot)
{
	return (slot < PAS_TRACE_SLOTS) && T.slots[slot].ready;
}

uint16_t pas_trace_slot_count(uint8_t slot)
{
	return pas_trace_slot_ready(slot) ? T.slots[slot].count_snapshot : 0U;
}

bool pas_trace_slot_get(uint8_t slot, uint16_t index, pas_trace_sample_t *out)
{
	if (!pas_trace_slot_ready(slot) || out == 0 || index >= T.slots[slot].count_snapshot) {
		return false;
	}
	const pas_trace_slot_t *s = &T.slots[slot];
	uint16_t ring_slot = (uint16_t)((s->oldest_slot + index) % PAS_TRACE_LEN);
	*out = s->ring[ring_slot];
	return true;
}

uint16_t pas_trace_slot_trigger_index(uint8_t slot)
{
	return pas_trace_slot_ready(slot) ? T.slots[slot].trigger_index_snapshot : 0U;
}

uint8_t pas_trace_slot_capture_id(uint8_t slot)
{
	return pas_trace_slot_ready(slot) ? T.slots[slot].capture_id : PAS_TRACE_NO_CAPTURE;
}

void pas_trace_slot_release(uint8_t slot)
{
	if (slot >= PAS_TRACE_SLOTS) {
		return;
	}
	/*
	 * Full reset rather than resuming the old ring: a partially-overwritten pre-window
	 * blending two unrelated captures would be worse than a short one after release.
	 */
	reset_slot(&T.slots[slot]);
	if (slot == T.rearm_slot) {
		/* Streamed while STILL owned by the current saga (its POST completed before the record
		 * even closed) - ownership has nothing left to point at, so it ends here too. */
		T.rearm_slot = PAS_TRACE_SLOTS;
	}
	if (slot == T.rearm_retained_slot) {
		/* FW-111 v5: the retained capture has now actually been streamed - only now is it truly
		 * free for a future prearm or ordinary use. */
		T.rearm_retained_slot = PAS_TRACE_SLOTS;
	}
	if (T.active >= PAS_TRACE_SLOTS) {
		/* Watching had stopped for want of a slot; this one revives it. */
		T.active = slot;
	}
}

int8_t pas_trace_oldest_ready_slot(void)
{
	int8_t best = -1;
	uint32_t best_seq = 0;
	for (uint8_t i = 0; i < PAS_TRACE_SLOTS; i++) {
		if (!T.slots[i].ready) continue;
		if (best < 0 || T.slots[i].frozen_seq < best_seq) {
			best = (int8_t)i;
			best_seq = T.slots[i].frozen_seq;
		}
	}
	return best;
}

uint16_t pas_trace_capture_slots_full(void)
{
	return T.capture_slots_full;
}

/* --- single-slot compatibility view: always the oldest capture still waiting -------------- */

bool pas_trace_ready(void)
{
	return pas_trace_oldest_ready_slot() >= 0;
}

uint16_t pas_trace_count(void)
{
	int8_t s = pas_trace_oldest_ready_slot();
	return (s < 0) ? 0U : pas_trace_slot_count((uint8_t)s);
}

bool pas_trace_get(uint16_t index, pas_trace_sample_t *out)
{
	int8_t s = pas_trace_oldest_ready_slot();
	return (s < 0) ? false : pas_trace_slot_get((uint8_t)s, index, out);
}

uint16_t pas_trace_trigger_index(void)
{
	int8_t s = pas_trace_oldest_ready_slot();
	return (s < 0) ? 0U : pas_trace_slot_trigger_index((uint8_t)s);
}

void pas_trace_release(void)
{
	int8_t s = pas_trace_oldest_ready_slot();
	if (s >= 0) {
		pas_trace_slot_release((uint8_t)s);
	}
}
