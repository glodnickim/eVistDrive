#include "diag_session.h"
#include "diag_budget.h"

#if CAN_DIAGNOSTICS_ENABLE

#include <string.h>

/*
 * FW-106: see the header for what this owns and why it is not in main.c.
 * Implementation notes only here.
 */

/* --- session summary ---------------------------------------------------------------------- */

/*
 * FW-106: how many aggregate frames (main.c's 0x10203..0x1020F/0x10219 block) can be frozen
 * into one summary. main.c builds 13 today; this leaves headroom without needing the two
 * translation units to share a compile-time count.
 *
 * FW-111: reduced 16 -> 15. Adding the fourth record source (DIAG_SRC_REARM) grows the
 * per-summary arrays refused_at_open/close and the module's per-source counters by ~60 B; with
 * DIAG_AGGREGATE_SNAPSHOT_MAX=16 the re-measured session line item plus the rearm module's own
 * line item would exceed the 12 KB ceiling of inc/diag_budget.h. 15 still sits above the 13
 * frames main.c builds, so the aggregate block is unaffected. Raising it again means re-measuring
 * the RAM budget - see inc/diag_budget.h.
 *
 * FW-111 v2: reduced 15 -> 14. The reworked rearm record (guaranteed ENTER/PROBLEM/PERMISSION/
 * CLOSE snapshots + a 24 B timing block, 160 B per record) measures 364 B for the rearm module;
 * its 64 B-headroom line item grew, and to keep the 12 KB ceiling this frees 12 B x 4 summaries
 * = 48 B back. 14 still sits above the 13 frames main.c builds.
 *
 * FW-113.2: main.c now builds exactly 14 aggregate frames (0x10203..0x1020F, 0x10219, 0x10228),
 * filling DIAG_AGGREGATE_SNAPSHOT_MAX exactly. Do not add a 15th aggregate frame without
 * re-measuring the 12 KB budget in inc/diag_budget.h - see the note above.
 */
#define DIAG_AGGREGATE_SNAPSHOT_MAX 14U

typedef struct {
	uint8_t  session_id;
	uint16_t duration_ds;
	uint32_t missed_ticks;    /* THIS session: total minus the anchor taken when it started */
	uint32_t missed_events;   /* likewise */
	uint16_t worst_burst;     /* THIS session: tracked separately, a maximum does not subtract */
	uint16_t episodes;
	uint16_t records_to_send; /* exactly this session's records, counted per source */
	/*
	 * FW-106: THIS session's raw-capture overrun count, same anchor/candidate treatment as
	 * missed_ticks - a non-zero delta sets DIAG_ERR_RAW_OVERRUN in the trailer, and only for the
	 * session that actually saw it.
	 */
	uint32_t raw_overrun_delta;
	/*
	 * FW-106 — THE AGGREGATE BLOCK, FROZEN AT candidate_end_tick, NOT READ LIVE AT DUMP TIME.
	 *
	 * These used to be read live during PH_AGGREGATE, whenever the dump happened to get there.
	 * If session A's dump was interrupted before reaching this phase and session B rode and
	 * ended in between, A's aggregate frames - sent only once A's dump finally resumed - would
	 * silently describe B's live state, mislabelled as A's. Snapshotting them here, at the same
	 * moment every other per-session metric is snapshotted, closes that gap the same way.
	 */
	uint32_t agg_efid[DIAG_AGGREGATE_SNAPSHOT_MAX];
	uint8_t  agg_data[DIAG_AGGREGATE_SNAPSHOT_MAX][8];
	uint8_t  agg_count;
	/* Filled in during the dump. Survives an interruption: the dump resumes into this record. */
	uint16_t records_sent;
	uint16_t records_discarded;
	uint16_t records_rejected;
	uint8_t  err;             /* DIAG_ERR_* accumulated for THIS session only */
	/* Source counter values when the session closed, so the trailer's bits are this session's
	 * deltas rather than flags that stay set until the controller reboots. */
	uint32_t refused_at_close[DIAG_SRC_COUNT];
	uint32_t refused_at_open[DIAG_SRC_COUNT];
} diag_summary_t;

/*
 * ALL of this module's mutable state, in one object. Grouping it is what lets the RAM budget
 * check below measure the module rather than a hand-picked pair of arrays: adding a counter here
 * grows sizeof(D) and can therefore break the assert, which is the point of having one.
 */
static struct {
	const diag_can_ops_t *can_ops;
	const diag_ops_t     *ops;

	diag_summary_t summaries[DIAG_SESSION_SUMMARIES];
	uint8_t  sum_head, sum_tail, sum_depth;
	uint16_t summary_rejected;
	/* FW-106: watermark for DIAG_ERR_SESSION_LOST - the value of summary_rejected as of the
	 * last trailer actually confirmed sent. Advanced only on STEP_DONE (see PH_TRAILER), so a
	 * trailer that itself never made it onto the bus does not silently swallow the report. */
	uint16_t summary_rejected_reported;

	uint32_t cnt_enqueued[DIAG_SRC_COUNT];
	uint32_t cnt_sent[DIAG_SRC_COUNT];
	uint32_t cnt_discarded[DIAG_SRC_COUNT];
	uint32_t cnt_rejected[DIAG_SRC_COUNT];
	/* Where the polling of the sources' own free-running totals has got to. */
	uint32_t seen_accepted[DIAG_SRC_COUNT];
	uint32_t seen_refused[DIAG_SRC_COUNT];

	uint8_t  session_id;
	bool     session_active;
	uint32_t session_start_tick;
	uint32_t session_anchor_missed_ticks;
	uint32_t session_anchor_missed_events;
	uint32_t session_anchor_episodes;
	uint32_t session_anchor_raw_overrun;
	uint32_t session_anchor_refused[DIAG_SRC_COUNT];
	uint16_t session_worst_burst;

	bool     quiet;
	uint32_t quiet_since_tick;
	uint32_t cand_end_tick;
	uint32_t cand_missed_ticks;
	uint32_t cand_missed_events;
	uint32_t cand_raw_overrun;
	uint16_t cand_worst_burst;
	uint16_t cand_episodes;
	/*
	 * FW-106: whether the aggregate block was actually captured for the CURRENT candidate. It is
	 * written directly into summaries[sum_head] (see below) rather than into a separate
	 * candidate buffer, to avoid paying for the ~150 B snapshot twice - false only when the
	 * summary queue was already full at the moment quiet began, so there was no free slot to
	 * write into without corrupting an older, still-valid summary.
	 */
	bool     cand_agg_valid;

	/* --- transmit automaton --- */
	uint8_t tx_state;
	uint8_t tx_mailbox;
	uint8_t tx_retry;
	/*
	 * FW-106: the tick a NEW frame was last handed to the CAN peripheral. Compared by
	 * subtraction (never `now_tick >= deadline`), which stays correct across the 32-bit
	 * wraparound the same way every other tick delta in this codebase does.
	 */
	uint32_t tx_pace_last_tick;

	/* --- dump state --- */
	uint8_t  dump_phase;
	uint16_t agg_index;      /* which aggregate frame is being offered */
	uint8_t  rec_source;     /* which of the three sources the dump is draining */
	uint16_t rec_frag;       /* fragment cursor within the record in flight */
	diag_record_kind_t rec_kind;   /* what is at risk RIGHT NOW, by kind - never a guess */
} D;

_Static_assert(sizeof(D) <= DIAG_BUDGET_SESSION_BYTES,
	"FW-106: diag_session's total state exceeds its RAM line item");

#define TX_IDLE    0U
#define TX_PENDING 1U

#define STEP_BUSY   0U   /* nothing confirmed: offer the SAME fragment again next call */
#define STEP_DONE   1U   /* confirmed on the wire: the cursor may move */
#define STEP_GIVEUP 2U   /* out of retries: abandon whatever this frame belonged to */

#define PH_IDLE      0U
#define PH_HEADER    1U
#define PH_METRICS   2U
#define PH_AGGREGATE 3U
#define PH_RECORDS   4U
#define PH_TRAILER   5U

static const diag_record_kind_t kind_of_source[DIAG_SRC_COUNT] = {
	DIAG_REC_EPISODE, DIAG_REC_TRACE, DIAG_REC_RAW, DIAG_REC_REARM, DIAG_REC_FW112
};

void diag_session_init(const diag_can_ops_t *can, const diag_ops_t *o)
{
	memset(&D, 0, sizeof(D));
	D.can_ops = can;
	D.ops = o;
	D.dump_phase = PH_IDLE;
	D.rec_kind = DIAG_REC_NONE;
}

/*
 * Follow each source's own acceptance decisions. Polling rather than being called keeps the
 * recorders free of any dependency on this module - they simply count, and this reads.
 */
static void poll_source_counters(void)
{
	if (D.ops == 0) return;
	for (uint8_t i = 0; i < DIAG_SRC_COUNT; i++) {
		if (D.ops->src[i].accepted_total != 0) {
			uint32_t now = D.ops->src[i].accepted_total();
			D.cnt_enqueued[i] += now - D.seen_accepted[i];
			D.seen_accepted[i] = now;
		}
		if (D.ops->src[i].refused_total != 0) {
			uint32_t now = D.ops->src[i].refused_total();
			D.cnt_rejected[i] += now - D.seen_refused[i];
			D.seen_refused[i] = now;
		}
	}
}

static uint32_t raw_overrun_total_now(void)
{
	if (D.ops != 0 && D.ops->raw_overrun_total != 0) return D.ops->raw_overrun_total();
	return 0U;
}

static uint16_t records_of_session(uint8_t sid)
{
	uint16_t n = 0;
	if (D.ops == 0) return 0;
	for (uint8_t i = 0; i < DIAG_SRC_COUNT; i++) {
		if (D.ops->src[i].count != 0) n = (uint16_t)(n + D.ops->src[i].count(sid));
	}
	return n;
}

static void abort_dump(diag_summary_t *s);

/*
 * FW-106: the one and only place a session is started, shared by diag_session_note_activity()
 * and diag_session_tick(). Both call sites guard it behind `!session_active`, which is what
 * keeps a PAS-transition-triggered early start and the later full tick from ever bumping the id
 * twice for the same session.
 */
static void start_session(uint32_t now_tick,
                          uint32_t missed_ticks_total, uint32_t missed_events_total)
{
	/*
	 * The rider has set off. If a dump was under way it ends HERE, at the moment the bike moved,
	 * rather than whenever the dump happens to be stepped next - otherwise whether an
	 * interruption was noticed at all would depend on call ordering, and a record could sit in
	 * limbo, neither sent nor discarded.
	 */
	if (D.sum_depth > 0 && D.dump_phase != PH_IDLE) {
		abort_dump(&D.summaries[D.sum_tail]);
	}
	D.session_active = true;
	D.session_id++;                       /* wraps at 256: a key, never a count */
	D.session_start_tick = now_tick;
	D.session_anchor_missed_ticks = missed_ticks_total;
	D.session_anchor_missed_events = missed_events_total;
	D.session_anchor_episodes = D.cnt_enqueued[DIAG_SRC_EPISODE];
	D.session_anchor_raw_overrun = raw_overrun_total_now();
	for (uint8_t i = 0; i < DIAG_SRC_COUNT; i++) {
		D.session_anchor_refused[i] = D.cnt_rejected[i];
	}
	D.session_worst_burst = 0;
	D.quiet = false;
}

void diag_session_note_activity(uint32_t now_tick,
                                uint32_t missed_ticks_total, uint32_t missed_events_total)
{
	poll_source_counters();
	if (!D.session_active) {
		start_session(now_tick, missed_ticks_total, missed_events_total);
		return;
	}
	/*
	 * FW-106 — EVERY call cancels/re-anchors an in-progress quiet candidate, not just a
	 * stop->ride transition.
	 *
	 * The caller reports this because SOMETHING happened on the PAS lines - but main.c's own
	 * "moving" signal for the SAME control tick can still read false: an illegal two-bit
	 * quadrature jump looks up qd[]=0 in the decoder, so it takes neither the forward nor the
	 * backward branch and never touches pas_idle_ticks, which "moving" is built from (FW-109 v2:
	 * it DOES now zero fwd_run - see pas_direction.c - but that makes "moving"'s fwd_run term
	 * read LESS active on an invalid step, never more, so it still cannot be relied on to reveal
	 * one; the backpedal latch is unaffected by an invalid step either way, unchanged). Without
	 * this, a spurious edge exactly like the ones this card was
	 * opened to investigate could sit INSIDE an already-ticking 3 s quiet window without ever
	 * resetting it, and a session could be reported as "3 s of undisturbed silence" when it
	 * demonstrably was not. Clearing `quiet` here is enough: diag_session_tick() re-arms a fresh
	 * candidate, from THIS tick, the next time it sees moving==false - the same re-anchoring
	 * a genuine stop always goes through.
	 */
	D.quiet = false;
}

void diag_session_tick(uint32_t now_tick, bool moving,
                       uint32_t missed_ticks_total, uint32_t missed_events_total,
                       uint16_t burst_this_tick)
{
	poll_source_counters();

	if (moving) {
		D.quiet = false;
		if (!D.session_active) {
			start_session(now_tick, missed_ticks_total, missed_events_total);
		}
		/*
		 * Applied AFTER the session may have been started, so a burst on a session's very first
		 * tick belongs to that session rather than being dropped on the floor.
		 */
		if (burst_this_tick > D.session_worst_burst) D.session_worst_burst = burst_this_tick;
		return;
	}

	if (!D.session_active) return;
	if (burst_this_tick > D.session_worst_burst) D.session_worst_burst = burst_this_tick;

	if (!D.quiet) {
		/*
		 * Quiet has just begun. The candidate is taken NOW, because these are the ride's
		 * figures; three seconds of standing still proving the stop was real must not land in
		 * them. Movement inside the window throws the candidate away and the session continues.
		 */
		D.quiet = true;
		D.quiet_since_tick = now_tick;
		D.cand_end_tick = now_tick;
		D.cand_missed_ticks = missed_ticks_total - D.session_anchor_missed_ticks;
		D.cand_missed_events = missed_events_total - D.session_anchor_missed_events;
		D.cand_worst_burst = D.session_worst_burst;
		D.cand_raw_overrun = raw_overrun_total_now() - D.session_anchor_raw_overrun;
		{
			uint32_t eps = D.cnt_enqueued[DIAG_SRC_EPISODE] - D.session_anchor_episodes;
			D.cand_episodes = (eps > 0xFFFFU) ? 0xFFFFU : (uint16_t)eps;
		}
		/*
		 * FW-106: freeze the aggregate block NOW, straight into the summary slot this candidate
		 * would occupy if confirmed. Written directly rather than into a separate buffer - see
		 * cand_agg_valid's comment. Only safe when that slot is genuinely free: with the queue
		 * already full, sum_head coincides with the OLDEST still-valid, undrained summary, and
		 * scribbling into it here would corrupt data that belongs to an earlier session.
		 */
		D.cand_agg_valid = false;
		if (D.sum_depth < DIAG_SESSION_SUMMARIES && D.ops != 0 && D.ops->aggregate_frame != 0) {
			diag_summary_t *cand_slot = &D.summaries[D.sum_head];
			memset(cand_slot, 0, sizeof(*cand_slot));
			uint8_t n = 0;
			while (n < DIAG_AGGREGATE_SNAPSHOT_MAX) {
				uint32_t efid; uint8_t data[8];
				if (!D.ops->aggregate_frame(n, &efid, data)) break;
				cand_slot->agg_efid[n] = efid;
				memcpy(cand_slot->agg_data[n], data, 8);
				n++;
			}
			cand_slot->agg_count = n;
			D.cand_agg_valid = true;
		}
		return;
	}

	if ((now_tick - D.quiet_since_tick) < DIAG_SESSION_QUIET_TICKS) return;

	/*
	 * The quiet held. Seal anything still collecting a post-window FIRST: a capture armed in the
	 * last moments of this ride would otherwise keep filling from the NEXT one, and a record
	 * spanning two sessions is precisely the class of defect this card exists to remove. Sealed
	 * captures are kept, short, rather than thrown away - a truncated view of a real event is
	 * worth more than no view, and its length says plainly that it is truncated.
	 */
	if (D.ops != 0 && D.ops->seal_open_captures != 0) D.ops->seal_open_captures();
	poll_source_counters();

	if (D.sum_depth < DIAG_SESSION_SUMMARIES) {
		diag_summary_t *s = &D.summaries[D.sum_head];
		if (!D.cand_agg_valid) {
			/*
			 * The summary queue was full when the quiet candidate began, so the aggregate block
			 * could not be frozen without corrupting an older, still-valid summary occupying
			 * this same slot at the time. A slot has since freed up - but the live state from
			 * the moment this session actually went quiet is long gone, so this session's dump
			 * will honestly carry no aggregate frames (agg_count stays 0 from this memset)
			 * rather than send something stale relabelled as fresh.
			 */
			memset(s, 0, sizeof(*s));
		}
		/* Otherwise s->agg_efid/agg_data/agg_count are already the frozen snapshot written at
		 * candidate time, into this exact slot - left untouched here. */
		s->session_id = D.session_id;
		{
			uint32_t ds = (D.cand_end_tick - D.session_start_tick) / (CONTROL_TIMEBASE_HZ / 10U);
			s->duration_ds = (ds > 0xFFFFU) ? 0xFFFFU : (uint16_t)ds;
		}
		s->missed_ticks = D.cand_missed_ticks;
		s->missed_events = D.cand_missed_events;
		s->worst_burst = D.cand_worst_burst;
		s->episodes = D.cand_episodes;
		s->raw_overrun_delta = D.cand_raw_overrun;
		s->records_to_send = records_of_session(D.session_id);
		for (uint8_t i = 0; i < DIAG_SRC_COUNT; i++) {
			s->refused_at_open[i] = D.session_anchor_refused[i];
			s->refused_at_close[i] = D.cnt_rejected[i];
			s->records_rejected = (uint16_t)(s->records_rejected +
				(D.cnt_rejected[i] - D.session_anchor_refused[i]));
		}
		D.sum_head = (uint8_t)((D.sum_head + 1U) % DIAG_SESSION_SUMMARIES);
		D.sum_depth++;
	} else {
		/*
		 * FW-106 — THE FULL-QUEUE POLICY, and why it cannot just "refuse and move on".
		 *
		 * The four older summaries stay exactly as they are: untouched, still able to be dumped
		 * in full. But this session's own records are ALREADY sitting in the episode queue and
		 * the trace/raw slots, accepted there before this session even tried to close - refusing
		 * only the summary would strand them: no summary will ever ask for session_id again, so
		 * `count`/`frame` would keep reporting them and `pending = enqueued - sent - discarded`
		 * would never reach zero. They are therefore discarded here, immediately, one source at
		 * a time, via the same release() the dump itself would have used - so this counts as
		 * DISCARDED, never as sent and never folded into the per-source `rejected` counter (which
		 * means "refused at the door", not "accepted then abandoned").
		 */
		D.summary_rejected++;
		for (uint8_t i = 0; i < DIAG_SRC_COUNT; i++) {
			if (D.ops->src[i].count == 0 || D.ops->src[i].release == 0) continue;
			while (D.ops->src[i].count(D.session_id) > 0) {
				D.ops->src[i].release(D.session_id, false);
				D.cnt_discarded[i]++;
			}
		}
	}

	D.session_active = false;
	D.quiet = false;
}

/* --- one non-blocking transmit step -------------------------------------------------------- */

static uint8_t tx_step(uint32_t now_tick, uint32_t efid, const uint8_t *data, uint8_t *err_out,
                        bool allow_new_tx)
{
	if (D.can_ops == 0) return STEP_BUSY;

	if (D.tx_state == TX_PENDING) {
		/*
		 * FW-106: checking a frame already in flight is NEVER paced - it costs no bus
		 * bandwidth of its own, and delaying it would only delay noticing DONE/FAILED, not
		 * reduce how often a new frame starts. FW-110: also never gated by allow_new_tx, for
		 * the same reason - this is not a NEW claim on a mailbox, it is reading back one this
		 * module already holds.
		 */
		uint8_t st = D.can_ops->state(D.tx_mailbox);
		if (st == DIAG_CAN_PENDING) return STEP_BUSY;   /* still on the wire; never wait here */
		D.tx_state = TX_IDLE;
		if (st == DIAG_CAN_OK) {
			D.tx_retry = 0;
			return STEP_DONE;
		}
		*err_out |= DIAG_ERR_TX_FAILED;
		D.tx_retry++;
		if (D.tx_retry > DIAG_TX_MAX_RETRY) {
			D.tx_retry = 0;
			*err_out |= DIAG_ERR_TX_GAVEUP;
			return STEP_GIVEUP;
		}
		return STEP_BUSY;
	}

	/*
	 * FW-110: not in flight, so this call would be starting a NEW claim on a mailbox - either a
	 * genuinely fresh frame or a retry of one that just FAILED (the branch above already reset
	 * tx_state to TX_IDLE for that case, so it lands here too - correctly, since a retry is just
	 * as much a new transmit() attempt as any other). The caller (main.c, via
	 * diag_session_dump_step()) has proven no critical HMI/Canable frame is waiting for a
	 * mailbox right now; when it has not, diagnostics may not compete for one. Checked BEFORE
	 * pacing and BEFORE touching D.tx_pace_last_tick, so being blocked here costs this frame
	 * nothing - the exact same fragment is offered again, unchanged, the next allowed call.
	 */
	if (!allow_new_tx) return STEP_BUSY;

	/*
	 * FW-106: pace only the START of a new frame (including a retry of a failed one - it is
	 * just as much a new call to transmit() as any other fresh frame). Nothing above this line
	 * is skipped while paced: an in-flight frame's result is still read every call, and a
	 * session that just started is still noticed and aborts the dump every call (see
	 * diag_session_dump_step() below) regardless of what this timer is doing.
	 */
	if ((uint32_t)(now_tick - D.tx_pace_last_tick) < DIAG_TX_FRAME_INTERVAL_TICKS) {
		return STEP_BUSY;
	}

	uint8_t mb = D.can_ops->transmit(efid, data);
	if (mb == DIAG_CAN_NOMAILBOX) {
		/* Normal on a busy bus and NOT a loss: the driver wrote nothing at all. Nothing was
		 * started, so the pacing clock is not touched - the next call may try again at once. */
		return STEP_BUSY;
	}
	D.tx_pace_last_tick = now_tick;
	D.tx_mailbox = mb;
	D.tx_state = TX_PENDING;
	return STEP_BUSY;
}

/* --- record handling ----------------------------------------------------------------------- */

static void finish_record(diag_summary_t *s, bool sent)
{
	uint8_t src = D.rec_source;
	if (D.ops != 0 && D.ops->src[src].release != 0) D.ops->src[src].release(s->session_id, sent);
	if (sent) {
		D.cnt_sent[src]++;
		if (s->records_sent < 0xFFFFU) s->records_sent++;
	} else {
		D.cnt_discarded[src]++;
		if (s->records_discarded < 0xFFFFU) s->records_discarded++;
	}
	D.rec_kind = DIAG_REC_NONE;
	D.rec_frag = 0;
	D.tx_state = TX_IDLE;
	D.tx_retry = 0;
}

/*
 * The rider set off again. Riding wins at once, and exactly ONE record can be at risk: whichever
 * kind rec_kind names. Everything not yet started stays in its queue for the next stop. A record
 * is never resumed later - half of it now and half after the next ride would be a record stitched
 * from two moments, which is the defect this card removes, not one it may introduce.
 */
static void abort_dump(diag_summary_t *s)
{
	s->err |= DIAG_ERR_INTERRUPTED;
	if (D.rec_kind != DIAG_REC_NONE) {
		finish_record(s, false);
	}
	D.dump_phase = PH_IDLE;
	D.tx_state = TX_IDLE;
	D.tx_retry = 0;
}

void diag_session_dump_step(uint32_t now_tick, bool allow_new_tx)
{
	if (D.ops == 0 || D.can_ops == 0 || D.sum_depth == 0) return;

	diag_summary_t *s = &D.summaries[D.sum_tail];

	/*
	 * FW-106: checked first, every call, completely unaffected by tx pacing below - the rider
	 * setting off again must end the dump at once even if the pacing timer would otherwise still
	 * be blocking the next frame from starting.
	 */
	if (D.session_active) {
		if (D.dump_phase != PH_IDLE) abort_dump(s);
		return;
	}

	switch (D.dump_phase) {
	case PH_IDLE:
		/*
		 * Re-entering after an interruption must NOT reset what has already been achieved:
		 * records_sent and the error bits live in the summary and are carried forward. Only the
		 * position in the sequence restarts, and since every source is asked by session id, the
		 * records already retired are simply no longer there.
		 */
		D.agg_index = 0;
		D.rec_source = 0;
		D.rec_frag = 0;
		D.rec_kind = DIAG_REC_NONE;
		D.dump_phase = PH_HEADER;
		break;

	case PH_HEADER: {
		uint8_t d[8];
		d[0] = 1U;                       /* schema version */
		d[1] = s->session_id;
		d[2] = (uint8_t)(s->duration_ds >> 8); d[3] = (uint8_t)s->duration_ds;
		d[4] = (uint8_t)(s->episodes >> 8);    d[5] = (uint8_t)s->episodes;
		d[6] = (uint8_t)(s->records_to_send >> 8); d[7] = (uint8_t)s->records_to_send;
		uint8_t r = tx_step(now_tick, 0x0001021DU, d, &s->err, allow_new_tx);
		/* A header that cannot be sent must not wedge the dump: the records still go out, and
		 * the missing header is itself visible in the log. */
		if (r == STEP_DONE || r == STEP_GIVEUP) D.dump_phase = PH_METRICS;
		break;
	}

	case PH_METRICS: {
		uint8_t d[8];
		/* FW-110: missed_events is uint32_t internally (the source main.c global that used to
		 * saturate a uint16_t before a session even started is now uint32_t too), but the wire
		 * field is 16 bits. Clamp here, at the point of serialization, rather than relying on
		 * the source never exceeding 0xFFFF - a session that genuinely lost more than 65535
		 * events must publish the ceiling, not a silently wrapped low word. */
		uint32_t me = s->missed_events;
		uint16_t me16 = (me > 0xFFFFU) ? 0xFFFFU : (uint16_t)me;
		d[0] = (uint8_t)(s->missed_ticks >> 24); d[1] = (uint8_t)(s->missed_ticks >> 16);
		d[2] = (uint8_t)(s->missed_ticks >> 8);  d[3] = (uint8_t)s->missed_ticks;
		d[4] = (uint8_t)(me16 >> 8);             d[5] = (uint8_t)me16;
		d[6] = (uint8_t)(s->worst_burst >> 8);   d[7] = (uint8_t)s->worst_burst;
		uint8_t r = tx_step(now_tick, 0x00010218U, d, &s->err, allow_new_tx);
		if (r == STEP_DONE || r == STEP_GIVEUP) D.dump_phase = PH_AGGREGATE;
		break;
	}

	case PH_AGGREGATE: {
		/*
		 * FW-106: reads the snapshot frozen at candidate_end_tick (see diag_session_tick()),
		 * NEVER ops->aggregate_frame() live - that is precisely what let an interrupted-then-
		 * resumed dump send a later session's live values mislabelled as this one's.
		 */
		if (D.agg_index >= s->agg_count) {
			D.dump_phase = PH_RECORDS;
			break;
		}
		uint8_t r = tx_step(now_tick, s->agg_efid[D.agg_index], s->agg_data[D.agg_index], &s->err,
		                    allow_new_tx);
		/* Giving up advances too - one unsendable aggregate frame cannot stall the whole dump,
		 * and no record is at risk here (rec_kind is NONE throughout this phase). */
		if (r == STEP_DONE || r == STEP_GIVEUP) D.agg_index++;
		break;
	}

	case PH_RECORDS: {
		if (D.rec_kind == DIAG_REC_NONE) {
			/* Find the next source that still has a record belonging to THIS session. */
			while (D.rec_source < DIAG_SRC_COUNT &&
			       (D.ops->src[D.rec_source].count == 0 ||
			        D.ops->src[D.rec_source].count(s->session_id) == 0)) {
				D.rec_source++;
			}
			if (D.rec_source >= DIAG_SRC_COUNT) {
				D.dump_phase = PH_TRAILER;
				break;
			}
			D.rec_kind = kind_of_source[D.rec_source];
			D.rec_frag = 0;
		}

		uint32_t efid = 0;
		uint8_t d[8];
		bool last = false;
		if (D.ops->src[D.rec_source].frame == 0 ||
		    !D.ops->src[D.rec_source].frame(s->session_id, D.rec_frag, &efid, d, &last)) {
			/* The record vanished under us: treat it as resolved rather than spin. */
			finish_record(s, false);
			break;
		}

		uint8_t r = tx_step(now_tick, efid, d, &s->err, allow_new_tx);
		if (r == STEP_GIVEUP) {
			/*
			 * Out of retries. The whole record goes, not just this fragment - a record missing
			 * one of its parts is worse than an absent one, because it looks complete.
			 * Fragment 0 of an episode is its group header, so that case is named separately.
			 */
			if (D.rec_kind == DIAG_REC_EPISODE && D.rec_frag == 0) s->err |= DIAG_ERR_HEADER_FAILED;
			finish_record(s, false);
			break;
		}
		if (r != STEP_DONE) break;      /* NOMAILBOX or PENDING: cursor does not move */

		D.rec_frag++;
		if (last) finish_record(s, true);
		break;
	}

	case PH_TRAILER:
	default: {
		uint8_t d[8];
		uint8_t err = s->err;
		/* Deltas for THIS session, not flags that stay set until the controller reboots. */
		for (uint8_t i = 0; i < DIAG_SRC_COUNT; i++) {
			if (s->refused_at_close[i] != s->refused_at_open[i]) {
				err |= (i == DIAG_SRC_EPISODE) ? DIAG_ERR_QUEUE_REJECTED : DIAG_ERR_CAPTURES_FULL;
			}
		}
		if (s->raw_overrun_delta > 0U) err |= DIAG_ERR_RAW_OVERRUN;
		/*
		 * FW-106: a session lost to a full summary queue has no trailer of its own - it is
		 * reported as a delta on whichever trailer goes out NEXT, exactly like raw_overrun_delta
		 * is a delta on missed_ticks rather than a sticky global flag.
		 */
		if (D.summary_rejected != D.summary_rejected_reported) err |= DIAG_ERR_SESSION_LOST;
		/*
		 * Complete means every record this session produced has been resolved - sent or
		 * discarded. Having been interrupted earlier says nothing about that, so it gets its
		 * own bit and does not veto completeness.
		 */
		bool complete = (records_of_session(s->session_id) == 0);

		d[0] = 1U;
		d[1] = s->session_id;
		d[2] = (uint8_t)(s->records_sent >> 8); d[3] = (uint8_t)s->records_sent;
		d[4] = (s->records_discarded > 255U) ? 255U : (uint8_t)s->records_discarded;
		d[5] = (s->records_rejected > 255U) ? 255U : (uint8_t)s->records_rejected;
		d[6] = err;
		/*
		 * FW-111: byte 7 distinguishes a refused FW-111 record from the shared
		 * DIAG_ERR_CAPTURES_FULL bit, which TRACE and RAW also set. Bit 0 stays "complete".
		 * FW-112-DIAG: bit 2 does the same for the whole-chain event recorder.
		 */
		{
			uint8_t tf = complete ? DIAG_TRAILER_F_COMPLETE : 0U;
			if (s->refused_at_close[DIAG_SRC_REARM] != s->refused_at_open[DIAG_SRC_REARM]) {
				tf |= DIAG_TRAILER_F_REARM_REJECTED;
			}
			if (s->refused_at_close[DIAG_SRC_FW112] != s->refused_at_open[DIAG_SRC_FW112]) {
				tf |= DIAG_TRAILER_F_FW112_REJECTED;
			}
			d[7] = tf;
		}

		uint8_t r = tx_step(now_tick, 0x0001021EU, d, &s->err, allow_new_tx);
		if (r == STEP_DONE) {
			/* Only advance the watermark once the bit was ACTUALLY seen on the bus - a trailer
			 * that itself gives up must not be treated as having reported the loss. */
			D.summary_rejected_reported = D.summary_rejected;
		}
		if (r == STEP_DONE || r == STEP_GIVEUP) {
			/* Even an unsendable trailer must retire the summary, or the queue would wedge and
			 * no later session could ever be reported. */
			D.sum_tail = (uint8_t)((D.sum_tail + 1U) % DIAG_SESSION_SUMMARIES);
			D.sum_depth--;
			D.dump_phase = PH_IDLE;
		}
		break;
	}
	}
}

uint8_t diag_session_current_id(void) { return D.session_id; }
bool    diag_session_is_active(void)  { return D.session_active; }

uint32_t diag_counter_enqueued(diag_source_t s)  { return (s < DIAG_SRC_COUNT) ? D.cnt_enqueued[s] : 0U; }
uint32_t diag_counter_sent(diag_source_t s)      { return (s < DIAG_SRC_COUNT) ? D.cnt_sent[s] : 0U; }
uint32_t diag_counter_discarded(diag_source_t s) { return (s < DIAG_SRC_COUNT) ? D.cnt_discarded[s] : 0U; }
uint32_t diag_counter_rejected(diag_source_t s)  { return (s < DIAG_SRC_COUNT) ? D.cnt_rejected[s] : 0U; }

uint32_t diag_counter_pending(diag_source_t s)
{
	if (s >= DIAG_SRC_COUNT) return 0U;
	return D.cnt_enqueued[s] - D.cnt_sent[s] - D.cnt_discarded[s];
}

diag_record_kind_t diag_dump_record_kind(void) { return D.rec_kind; }
uint16_t diag_summaries_pending(void)          { return D.sum_depth; }
uint16_t diag_summary_rejected(void)           { return D.summary_rejected; }

#else  /* !CAN_DIAGNOSTICS_ENABLE */

/* FW-106: sessions, accounting and the dump exist only in the diagnostic build - see pas_raw.c
 * for why this is a #if rather than a reliance on --gc-sections. */
typedef int diag_session_not_compiled_in;

#endif /* CAN_DIAGNOSTICS_ENABLE */
