#include "rearm_delay_diag.h"
#include "diag_budget.h"

#if CAN_DIAGNOSTICS_ENABLE

#include <string.h>

/*
 * FW-111: see the header for what this records and why. Implementation notes only.
 *
 * All mutable state lives in the single struct R below so the RAM budget check measures the
 * whole module, and the module is driven entirely through rearm_delay_input_t - the session
 * state arrives as a plain byte, the torque chain arrives as the published torque_snapshot_t,
 * nothing is linked from ride_session.c, ride_control.c or main.c.
 *
 * The TIMING record (snapshots, t_pressure..t_close) and the pas_trace TRACE/RAW RESERVATION are
 * two independent lifecycles on purpose (see the header's TRACE/RAW RESERVATION section for the
 * full rationale): the timing record's own open/close (this file's FSM below) anchors at the
 * PREARM edge (ACTIVE -> SUSPENDED_BY_DIRECTION - the real initiating event, FW-112 v2: with
 * WAIT_REARM_LOAD gone the delay is measured from where it actually starts), while the
 * reservation is bracketed by rearm_delay_prearm_edge()/ownership_end_edge() - fired on the same
 * initiating event and on the saga's real conclusion (a tracking record closing for a reason that
 * ends the saga, or the saga concluding with no record ever tracking it) - independent of how
 * many records open and close, and independent of the PERMISSION (rearm) tick itself: a
 * WEAK_TARGET can be detected up to 150 ms after the rearm, while the record is in RECOVERING, and
 * the reservation must still be held then. reserve_trigger (this record's own
 * one-shot PROBLEM signal) survives its record closing in the same tick and is resolved by an
 * INTERNAL record_uid (a fresh 32-bit identity per opened record) rather than by the wire
 * (record_id, session_id) pair - which is NOT unique across a record_seq wrap - or by the
 * record's current queue position - see note_problem() and rearm_delay_note_reserve_done().
 */

typedef enum {
	REARM_FSM_IDLE = 0,
	REARM_FSM_SUSPENDED = 1,
	REARM_FSM_RECOVERING = 2
} rearm_fsm_t;

/*
 * ALL of this module's mutable state, in one object (same discipline as ride_episode.c's E and
 * diag_session.c's D - see inc/diag_budget.h for why).
 */
static struct {
	rearm_fsm_t fsm_state;
	/* The queue of CLOSED records awaiting dump. In-place: the slot being captured NOW is the
	 * free queue slot (q_head + q_depth) % REARM_DELAY_RECORDS, so a capture never needs a
	 * second copy of the record. */
	rearm_delay_record_t slots[REARM_DELAY_RECORDS];
	/*
	 * v5.1: INTERNAL identity sidecar - one uint32_t per PHYSICAL slot, assigned at
	 * open_record() and moved together with the record on every queue operation. Deliberately
	 * NOT in rearm_delay_record_t and NEVER on the wire: it exists only so
	 * note_reserve_done() can find the exact record that raised a trigger even when the 8-bit
	 * wire (record_id, session_id) pair collides after record_seq wraps (a record can stay
	 * queued across a 256-rearm wrap) and even when the ring physically shifts a queued record
	 * between slots. Validity of the pending value comes from reserve_trigger itself, never
	 * from uid == 0 (0 is a perfectly legal assigned UID).
	 */
	uint32_t record_uid[REARM_DELAY_RECORDS];
	uint8_t q_head;      /* oldest queued record */
	uint8_t q_depth;     /* number of closed records awaiting dump */
	uint8_t cur_slot;    /* slot index of the record being captured, or 0xFF when none */
	uint32_t accepted_total;   /* records committed to the queue */
	uint32_t rejected_total;   /* captures refused because every slot held a queued record */

	uint8_t  session_id;       /* stamped into every record opened now */
	uint8_t  record_seq;       /* monotonic per-record id, wraps at 256 - a key, never a count */
	uint8_t  prev_session_state;
	uint16_t reached;          /* per-record stage mask, bit(id-1): one-shot guard for both the
	                            * four snapshot milestones and the timing stages */

	uint32_t anchor_tick;      /* the prearm edge (ACTIVE->SUSPENDED_BY_DIRECTION, the record's
	                            * origin - elapsed is measured from it) */
	int16_t  last_active_iq;   /* baseline: iq_pre_ramp of the LAST ACTIVE tick before the reverse */
	uint32_t weak_start_tick;  /* first tick of the current continuous below-80 % stretch */
	bool     weak_running;     /* a below-80 % stretch is in progress (0 is unambiguous) */
	uint16_t peak_load_centikg;/* max load_centikg seen since the record opened (SUSPENDED and
	                            * RECOVERING alike) */

	/* v5.1: UID generator. Every open_record() consumes R.uid_next and advances it; a value that
	 * is STILL in use (the open record, a queued record, or a pending trigger's record) is
	 * skipped, so even after a full uint32_t wrap a fresh UID can never alias a live record. */
	uint32_t uid_next;

	/*
	 * v5 TRACE/RAW reservation edges (see the header). prearm_edge/ownership_end_edge are
	 * recomputed fresh at the TOP/END of every rearm_delay_tick() call - not latched, since main.c
	 * reads them immediately after that same call, every tick, exactly like every other one-shot
	 * signal this module exposes.
	 *   prearm_edge        ACTIVE -> SUSPENDED_BY_DIRECTION - the real initiating event.
	 *   ownership_end_edge the saga's reservation is over: EITHER a tracking record just closed
	 *                      for a reason that concludes the saga (in v2 any record close concludes
	 *                      its saga - a WAIT<->SUSPENDED oscillation no longer exists, any
	 *                      re-suspend is simply more SUSPENDED), OR the saga concluded
	 *                      (permission/terminal) with no record ever tracking it at all (the
	 *                      queue was full at the prearm edge). Suppressed on a tick prearm_edge
	 *                      ALSO fires (a new saga interrupting mid-recovery) - see
	 *                      pas_trace_rearm_prearm()'s own "end old ownership first" handling,
	 *                      which already covers that case; firing both here would target the
	 *                      NEW saga's just-claimed slot instead of the old one.
	 * reserve_trigger is still a ONE-SHOT latched by the first PROBLEM instant of the CURRENT
	 * record - but v5 fixes it surviving close_record() (previously cleared before main.c could
	 * ever see it - Bug 2) and being resolvable even after R.cur_slot is invalidated. v5.1
	 * replaces the old pending_record_id/pending_session_id (a wire pair that COLLIDES after a
	 * 256-rearm record_seq wrap) with pending_record_uid: a 32-bit internal identity identifying
	 * WHICH record note_reserve_done() must finish, since the physical slots[] index a closing
	 * record occupies is not stable either (rearm_delay_queue_release_session() can shift the
	 * ring before the pending result is written).
	 */
	bool     prearm_edge;
	bool     ownership_end_edge;
	bool     reserve_trigger;
	uint32_t pending_record_uid;
} R;

_Static_assert(sizeof(R) <= DIAG_BUDGET_REARM_DELAY_BYTES,
	"FW-111: rearm_delay_diag's total state exceeds its RAM line item");

/* i16 clamp for the wire fields. */
static int16_t clamp16(int32_t v)
{
	if (v < -32768) return (int16_t)-32768;
	if (v > 32767)  return (int16_t)32767;
	return (int16_t)v;
}

/* Wraparound-safe elapsed ticks from the record's anchor - the idiom every other module uses. */
static uint32_t elapsed_ticks(uint32_t now_tick)
{
	return now_tick - R.anchor_tick;
}

/*
 * v5.1: is this UID currently assigned to a LIVE record a future note_reserve_done() might still
 * have to reach? A queued record, and - for the short window between the trigger latching and
 * note_reserve_done() consuming it - the pending record itself (even if that record has since
 * been released from the queue, which only happens for records whose dump already ran and whose
 * result is already written or irrelevant). The allocator therefore never re-issues a value that
 * could still alias a live record after a uint32_t wrap. (The record currently OPEN is always the
 * one being replaced by this very open_record(), so it is not a live alias here - it is stamped
 * with the freshly allocated value.)
 */
static bool uid_in_use(uint32_t uid)
{
	if (R.reserve_trigger && R.pending_record_uid == uid) return true;
	for (uint8_t i = 0; i < R.q_depth; i++) {
		uint8_t idx = (uint8_t)((R.q_head + i) % REARM_DELAY_RECORDS);
		if (R.record_uid[idx] == uid) return true;
	}
	return false;
}

/*
 * v5.1: a fresh, non-colliding identity. R.uid_next advances (wrapping through 0); any value
 * currently live (see uid_in_use) is skipped. With at most 2 slots and 1 pending record the loop
 * terminates after a handful of iterations at most - there is never a full-address-space scan.
 */
static uint32_t uid_alloc(void)
{
	uint32_t uid;
	do {
		uid = R.uid_next;
		R.uid_next++;
	} while (uid_in_use(uid));
	return uid;
}

/* The one-shot guard for a stage: returns true the FIRST time, false afterwards. Stage ids run up
 * to 12, so the mask must be uint16_t (a uint8_t truncates bit id 9+ to 0 and the guard would
 * then let PWM_ON fire on every tick). */
static bool first_time(uint8_t id)
{
	uint16_t bit = (uint16_t)(1U << (id - 1U));
	if ((R.reached & bit) != 0U) return false;
	R.reached = (uint16_t)(R.reached | bit);
	return true;
}

static uint16_t timing(uint32_t now_tick)
{
	uint32_t el = elapsed_ticks(now_tick);
	return (el >= REARM_DELAY_T_UNREACHED) ? (uint16_t)(REARM_DELAY_T_UNREACHED - 1U)
	                                       : (uint16_t)el;
}

/* Append a full 32 B snapshot for one of the four snapshot milestones (slots are bounded). */
static void capture_snapshot(const rearm_delay_input_t *in, uint32_t now_tick, uint8_t milestone_id)
{
	if (R.cur_slot == 0xFFU) return;
	rearm_delay_record_t *rec = &R.slots[R.cur_slot];
	if (rec->snapshot_count >= REARM_DELAY_SNAPSHOTS) return;

	rearm_delay_snapshot_t *s = &rec->snapshots[rec->snapshot_count];
	memset(s, 0, sizeof(*s));
	s->elapsed_ticks = elapsed_ticks(now_tick);
	if (in->snapshot != 0) {
		s->raw_native = in->snapshot->raw_native;
		s->zero_effective_native = in->snapshot->zero_effective_native;
		s->corrected_native = in->snapshot->corrected_native;
		s->delta_native = in->snapshot->delta_native;
		s->assist_delta_native = in->snapshot->assist_delta_native;
		s->assist_delta_filtered_native = in->snapshot->assist_delta_filtered_native;
		s->assist_delta_run_native = in->snapshot->assist_delta_run_native;
		s->load_centikg = in->snapshot->load_centikg;
		s->flags = (uint8_t)(
			(in->snapshot->sensor_valid ? REARM_SNAP_F_SENSOR_VALID : 0U) |
			(in->snapshot->calibration_source == 1U ? REARM_SNAP_F_CAL_USER : 0U));
	}
	s->run_deadband = in->run_deadband;
	s->iq_request = in->iq_request;
	s->iq_pre_ramp = in->iq_pre_ramp;
	s->iq_setpoint = in->iq_setpoint;
	s->flags = (uint8_t)(s->flags |
		(in->direction_inhibit_active ? REARM_SNAP_F_DIRECTION_INHIBIT : 0U) |
		(in->real_stop ? REARM_SNAP_F_REAL_STOP : 0U) |
		(in->limiter_zeroed ? REARM_SNAP_F_LIMITER_ZEROED : 0U) |
		(in->pwm_on ? REARM_SNAP_F_PWM_ON : 0U) |
		(in->session_state == 1U ? REARM_SNAP_F_COMMITTED : 0U));
	s->milestone_id = milestone_id;
	s->session_id = rec->session_id;
	s->record_id = rec->record_id;
	rec->snapshot_count++;
}

/*
 * The instant a keep-condition first fires: one snapshot (the torque state AT the problem) plus
 * the TRACE/RAW trigger. Guarded by first_time(PROBLEM) so only the FIRST such instant gets the
 * snapshot - later reason bits just set their own bit. This module has no opinion on whether a
 * reservation is actually held - it always reports the PROBLEM edge; main.c decides what (if
 * anything) it can capture from pas_trace_rearm_held().
 *
 * v5 (Bug 2 fix) + v5.1: pending_record_uid is stamped HERE, the instant the trigger first
 * latches, from the CURRENTLY open record (R.cur_slot is still valid at this point - this runs
 * before any close_record() call the SAME tick could invalidate it). This is the internal token
 * note_reserve_done() uses to find and finish the right record even after close_record() has
 * already run this same tick and reset R.cur_slot to 0xFF - see that function. Unlike the v5
 * wire (record_id, session_id) pair this token is collision-free: two records in the SAME
 * session can carry the same 8-bit wire record_id (after a 256-rearm wrap) yet always have
 * different record_uid values.
 */
static void note_problem(const rearm_delay_input_t *in, uint32_t now_tick)
{
	if (R.cur_slot == 0xFFU) return;
	if (first_time(REARM_MILESTONE_PROBLEM)) {
		capture_snapshot(in, now_tick, REARM_MILESTONE_PROBLEM);
	}
	if (!R.reserve_trigger) {
		R.reserve_trigger = true;
		R.pending_record_uid = R.record_uid[R.cur_slot];
	}
}

/*
 * Sample the recovery-side stages on the CURRENT tick: TARGET_RECOVERED / SETPOINT_RECOVERED,
 * the weak-stretch bookkeeping, and PWM_ON. Shared by the SUSPENDED branch on the
 * SUSPENDED -> ACTIVE (PERMISSION) tick and by every RECOVERING tick, so the exact PERMISSION tick
 * can record t_pressure .. t_pwm_on all at once and never lose a stage to a state change
 * (Bug 2). All times are first-occurrence and wrap-safe via timing().
 *
 * v5 (FW-112 v2) split the recovery benchmark in two:
 *   - WEAK_TARGET / SETPOINT_RECOVERED keep the pre-reverse Iq demand (pre_reverse_iq): they
 *     judge whether the CURRENT (mA) demand ever came back to ~the pre-reverse level.
 *   - TARGET_RECOVERED is measured on the RUN axis: the rolling rearm reseeds the RUN estimator,
 *     so "the recovery landed" means RUN is back to ~80 % of its pre-reverse level
 *     (snapshots[0] - the prearm-edge snapshot, taken while RUN still held the pre-reverse value).
 */
static void sample_recovery_chain(const rearm_delay_input_t *in, uint32_t now_tick)
{
	if (R.cur_slot == 0xFFU) return;
	rearm_delay_record_t *rec = &R.slots[R.cur_slot];
	int32_t pre = rec->pre_reverse_iq;
	/* The 80 % recovery benchmark only means anything if the rider actually had assist
	 * before the reverse. */
	if (pre > 0) {
		bool below = (int32_t)in->iq_pre_ramp * 100 <
			(int32_t)pre * (int32_t)REARM_DELAY_RECOVER_PCT;
		if (below) {
			/* WEAK_TARGET fires after the below-80 % stretch has been CONTINUOUS for the
			 * window - a brief dip mid-recovery must not count. weak_running is the
			 * truth here, so weak_start_tick == 0 is never ambiguous, not even when the
			 * 32-bit control_now wrapped inside the stretch. */
			if (!R.weak_running) {
				R.weak_running = true;
				R.weak_start_tick = now_tick;
				if (rec->t_weak_start == REARM_DELAY_T_UNREACHED) {
					rec->t_weak_start = timing(now_tick);
				}
			}
			if (now_tick - R.weak_start_tick >= REARM_DELAY_WEAK_TARGET_TICKS &&
			    (rec->reason_bits & REARM_DELAY_REASON_WEAK_TARGET) == 0U) {
				rec->reason_bits = (uint8_t)(rec->reason_bits | REARM_DELAY_REASON_WEAK_TARGET);
				note_problem(in, now_tick);
			}
		} else {
			R.weak_running = false;
		}
		if ((int32_t)in->iq_setpoint * 100 >=
		    (int32_t)pre * (int32_t)REARM_DELAY_RECOVER_PCT &&
		    first_time(REARM_MILESTONE_SETPOINT_RECOVERED)) {
			rec->t_setpoint_recovered = timing(now_tick);
		}
	}
	/* TARGET_RECOVERED on the RUN axis - see the function comment. snapshots[0] is always the
	 * prearm-edge snapshot (the record's anchor); a RUN of 0 before the reverse means there was
	 * nothing to recover and the milestone simply never fires. */
	int32_t run_pre = (int32_t)rec->snapshots[0].assist_delta_run_native;
	if (run_pre > 0 && in->snapshot != 0 &&
	    (int32_t)in->snapshot->assist_delta_run_native * 100 >=
	    run_pre * (int32_t)REARM_DELAY_RECOVER_PCT &&
	    first_time(REARM_MILESTONE_TARGET_RECOVERED)) {
		rec->t_target_recovered = timing(now_tick);
	}
	if (in->pwm_on && first_time(REARM_MILESTONE_PWM_ON)) {
		rec->t_pwm_on = timing(now_tick);
	}
}

static void open_record(const rearm_delay_input_t *in, uint32_t now_tick)
{
	uint8_t free_slot = (uint8_t)((R.q_head + R.q_depth) % REARM_DELAY_RECORDS);
	rearm_delay_record_t *rec = &R.slots[free_slot];
	memset(rec, 0, sizeof(*rec));
	rec->session_id = R.session_id;
	rec->record_id = R.record_seq;
	R.record_seq++;
	rec->pre_reverse_iq = clamp16(R.last_active_iq);
	rec->t_pressure = REARM_DELAY_T_UNREACHED;
	rec->t_filter_ready = REARM_DELAY_T_UNREACHED;
	rec->t_run_ready = REARM_DELAY_T_UNREACHED;
	rec->t_demand = REARM_DELAY_T_UNREACHED;
	rec->t_permission = REARM_DELAY_T_UNREACHED;
	rec->t_target_recovered = REARM_DELAY_T_UNREACHED;
	rec->t_setpoint_recovered = REARM_DELAY_T_UNREACHED;
	rec->t_pwm_on = REARM_DELAY_T_UNREACHED;
	rec->t_standstill_enter = REARM_DELAY_T_UNREACHED;
	rec->t_standstill_exit = REARM_DELAY_T_UNREACHED;
	rec->t_weak_start = REARM_DELAY_T_UNREACHED;
	rec->t_close = REARM_DELAY_T_UNREACHED;
	R.cur_slot = free_slot;
	R.reached = 0;
	R.anchor_tick = now_tick;
	R.weak_start_tick = 0;
	R.weak_running = false;
	R.peak_load_centikg = 0;
	/* v5.1: stamp the slot with its fresh internal identity BEFORE the record can ever become
	 * the target of a note_reserve_done() search (note_problem reads record_uid[cur_slot]). */
	R.record_uid[free_slot] = uid_alloc();
	/* The pas_trace reservation is NOT opened here - it started (or failed to start) back at the
	 * real initiating event, bracketed by rearm_delay_prearm_edge()/ownership_end_edge()
	 * independently of this record's own open/close (see the file header). reserve_trigger is
	 * NOT reset here either (v5): it is a one-shot latch cleared ONLY by note_reserve_done(), and
	 * a still-pending trigger from an earlier record cannot coexist with open_record() running -
	 * they belong to different, mutually exclusive FSM cases within the same tick. */
	rec->capture_id = REARM_DELAY_NO_CAPTURE;
	rec->capture_status = REARM_DELAY_CAPTURE_NONE;
	capture_snapshot(in, now_tick, REARM_MILESTONE_ENTER_SUSPEND);
}

/*
 * Close the record being captured. reason==0 means it was a healthy fast rearm - the buffer
 * slot is simply freed (the record was never counted as accepted). reason!=0 commits it to the
 * queue, or refuses it if every slot is occupied.
 */
static void close_record(const rearm_delay_input_t *in, uint32_t now_tick)
{
	if (R.cur_slot == 0xFFU) return;
	rearm_delay_record_t *rec = &R.slots[R.cur_slot];
	rec->t_close = timing(now_tick);

	if (rec->reason_bits != 0U) {
		if (first_time(REARM_MILESTONE_RECORD_CLOSE)) {
			capture_snapshot(in, now_tick, REARM_MILESTONE_RECORD_CLOSE);
		}
		if (R.q_depth < REARM_DELAY_RECORDS) {
			R.q_depth++;
			R.accepted_total++;
			/* cur_slot already equals (q_head + old q_depth) % REARM_DELAY_RECORDS, which is
			 * exactly where the queue grows. */
		} else {
			R.rejected_total++;
		}
	}
	R.cur_slot = 0xFFU;
	R.reached = 0;
	R.weak_start_tick = 0;
	R.weak_running = false;
	/*
	 * v5 (Bug 2 fix): reserve_trigger is DELIBERATELY NOT cleared here, even though R.cur_slot -
	 * this function's only way to reach the record - is about to become invalid. A PROBLEM that
	 * fires and closes its own record in the SAME tick (note_problem() then immediately
	 * close_record(), e.g. a terminal SUSPENDED->COLD before WAIT_LONG) must still leave the
	 * trigger visible to main.c on this tick - clearing it here (the v4 bug) means main.c never
	 * sees it at all, and the record is queued (if reason_bits != 0) with a capture_id/status
	 * that can never be written. R.pending_record_uid (stamped by note_problem(), see there) is
	 * the internal token note_reserve_done() uses to find this SAME record - wherever it now
	 * sits, cur_slot or already in the queue - once main.c calls it, still this same tick. The
	 * pas_trace RESERVATION itself (ownership) is untouched by a record closing - it can outlive
	 * this record (a saga interrupted by a fresh prearm edge, or one that continues into
	 * recovery) and is only ever ended by rearm_delay_ownership_end_edge().
	 */
}

void rearm_delay_init(void)
{
	memset(&R, 0, sizeof(R));
	R.cur_slot = 0xFFU;
	R.prev_session_state = 0xFFU;
}

void rearm_delay_note_standstill_enter(uint32_t now_tick)
{
	if (R.cur_slot == 0xFFU) return;
	rearm_delay_record_t *rec = &R.slots[R.cur_slot];
	if (rec->t_standstill_enter == REARM_DELAY_T_UNREACHED) {
		rec->t_standstill_enter = timing(now_tick);
	}
}

void rearm_delay_note_standstill_exit(uint32_t now_tick)
{
	if (R.cur_slot == 0xFFU) return;
	rearm_delay_record_t *rec = &R.slots[R.cur_slot];
	if (rec->t_standstill_enter != REARM_DELAY_T_UNREACHED &&
	    rec->t_standstill_exit == REARM_DELAY_T_UNREACHED) {
		rec->t_standstill_exit = timing(now_tick);
	}
}

void rearm_delay_set_session_id(uint8_t session_id)
{
	R.session_id = session_id;
}

void rearm_delay_tick(const rearm_delay_input_t *in, uint32_t now_tick)
{
	uint8_t st = in->session_state;

	/*
	 * v5 reservation edges - computed BEFORE anything below touches R.prev_session_state. Not
	 * sticky: overwritten fresh every call, read by the caller immediately after this call
	 * returns.
	 *
	 *   PREARM: ACTIVE (1) -> SUSPENDED_BY_DIRECTION (2) - the real initiating event. In v2 this
	 *           edge IS the record's anchor (see the header).
	 *
	 *   OWNERSHIP END: NOT tied to SUSPENDED_BY_DIRECTION -> ACTIVE (that only means the record
	 *   moves from SUSPENDED to RECOVERING - a WEAK_TARGET detected up to 150 ms later, while
	 *   still RECOVERING, must still find the reservation held). Instead, computed AFTER the FSM
	 *   switch below, from what the FSM itself just did: the saga's reservation is over when a
	 *   tracking record just closed (in v2 every record close concludes its saga - the v1
	 *   WAIT<->SUSPENDED re-suspend that kept the same reservation alive for a second record no
	 *   longer exists) OR when the saga concluded (permission/terminal) with NO record ever
	 *   tracking it (the queue was full at the prearm edge, so fsm stayed IDLE throughout - ending
	 *   ownership must not depend on a record existing). Suppressed on a tick prearm_edge ALSO
	 *   fires: a new saga interrupting the previous one's still-open recovery watch is handled
	 *   ATOMICALLY inside pas_trace_rearm_prearm() itself (it ends the old ownership before
	 *   claiming a new slot) - firing ownership_end_edge here too would race main.c into ending
	 *   the NEW saga's just-claimed reservation instead of the old one.
	 */
	R.prearm_edge = (st == 2U && R.prev_session_state == 1U);

	rearm_fsm_t fsm_before = R.fsm_state;

	/* While ACTIVE and idle, remember the demand of THIS tick - the LAST such tick before the
	 * reverse is the baseline. Not a max: a single strong push earlier in the ride must not
	 * inflate the 80 % recovery benchmark. */
	if (st == 1U && R.fsm_state == REARM_FSM_IDLE) {
		R.last_active_iq = in->iq_pre_ramp;
	}

	bool edge_handled = false;
	switch (R.fsm_state) {
	case REARM_FSM_IDLE:
		/* The PREARM edge (ACTIVE -> SUSPENDED_BY_DIRECTION): v2 anchors the record HERE, the
		 * real initiating event - there is no separate WAIT_REARM_LOAD stage any more. */
		if (R.prearm_edge) {
			edge_handled = true;
			if (R.q_depth >= REARM_DELAY_RECORDS) {
				R.rejected_total++;   /* every slot holds a queued record - capture refused */
				break;
			}
			open_record(in, now_tick);
			R.fsm_state = REARM_FSM_SUSPENDED;
		}
		break;

	case REARM_FSM_SUSPENDED:
		/*
		 * SAMPLE FIRST, CHANGE STATE SECOND (Bug 2): every input-chain stage that is present
		 * THIS tick is recorded before any state change, so a stage that first appears on the
		 * SUSPENDED->ACTIVE tick is measured (== t_permission) instead of being lost as 0xFFFF.
		 * The old ordering handled the st==1U PERMISSION branch first and broke out before
		 * PRESSURE/FILTER_READY/RUN_READY/DEMAND were ever looked at.
		 */
		{
			rearm_delay_record_t *rec = &R.slots[R.cur_slot];
			if (in->snapshot != 0) {
				if (in->snapshot->assist_delta_native > 0U &&
				    first_time(REARM_MILESTONE_PRESSURE)) {
					rec->t_pressure = timing(now_tick);
				}
				if (in->snapshot->assist_delta_filtered_native >= in->run_deadband &&
				    first_time(REARM_MILESTONE_FILTER_READY)) {
					rec->t_filter_ready = timing(now_tick);
				}
				if (in->snapshot->assist_delta_run_native > 0U &&
				    first_time(REARM_MILESTONE_RUN_READY)) {
					rec->t_run_ready = timing(now_tick);
				}
				if (in->snapshot->load_centikg > R.peak_load_centikg) {
					R.peak_load_centikg = in->snapshot->load_centikg;
				}
			}
			if (in->iq_request > 0 && first_time(REARM_MILESTONE_DEMAND)) {
				rec->t_demand = timing(now_tick);
			}
			if (in->pwm_on && first_time(REARM_MILESTONE_PWM_ON)) {
				rec->t_pwm_on = timing(now_tick);
			}
		}

		if (st == 1U) {
			/* SUSPENDED -> ACTIVE: the rearm edge fired (PERMISSION). Sample the recovery side
			 * on the SAME tick (t_target_recovered / t_setpoint_recovered / t_pwm_on can
			 * legitimately equal t_permission here - Bug 2), then capture the permission and
			 * watch the recovery. */
			sample_recovery_chain(in, now_tick);
			rearm_delay_record_t *rec = &R.slots[R.cur_slot];
			if (first_time(REARM_MILESTONE_PERMISSION)) {
				rec->t_permission = timing(now_tick);
				capture_snapshot(in, now_tick, REARM_MILESTONE_PERMISSION);
			}
			R.fsm_state = REARM_FSM_RECOVERING;
			break;
		}
		if (st != 2U) {
			/* Left SUSPENDED without the rearm edge ever firing: a terminal event took it to
			 * COLD. The saga got no permission. */
			rearm_delay_record_t *rec = &R.slots[R.cur_slot];
			rec->reason_bits = (uint8_t)(rec->reason_bits | REARM_DELAY_REASON_NO_PERMISSION);
			note_problem(in, now_tick);
			close_record(in, now_tick);
			R.fsm_state = REARM_FSM_IDLE;
			break;
		}
		/* Still suspended: the two keep-conditions that don't need recovery (WAIT_LONG measures
		 * the rearm delay itself; TIMEOUT = the rearm edge never fired within the budget).
		 * NO_LOAD fires only once recovery is watched - it means the rearm DID grant but no
		 * pedal load ever appeared. */
		{
			rearm_delay_record_t *rec = &R.slots[R.cur_slot];
			uint32_t el = elapsed_ticks(now_tick);
			if (el >= REARM_DELAY_WAIT_LONG_TICKS &&
			    (rec->reason_bits & REARM_DELAY_REASON_WAIT_LONG) == 0U) {
				rec->reason_bits = (uint8_t)(rec->reason_bits | REARM_DELAY_REASON_WAIT_LONG);
				note_problem(in, now_tick);
			}
			if (el >= REARM_DELAY_TIMEOUT_TICKS) {
				/* TIMEOUT_MS of suspension and the rearm edge never fired = exactly the
				 * NO_PERMISSION case (the header names timeout among its causes). */
				rec->reason_bits = (uint8_t)(rec->reason_bits | REARM_DELAY_REASON_NO_PERMISSION);
				note_problem(in, now_tick);
				close_record(in, now_tick);
				R.fsm_state = REARM_FSM_IDLE;
			}
		}
		break;

	case REARM_FSM_RECOVERING:
		if (st != 1U) {
			/* Session left ACTIVE: recovery is over - either terminal (-> COLD, the saga is
			 * done) or a fresh prearm edge (-> SUSPENDED, a NEW saga interrupting this one;
			 * its record is re-opened below, same tick). Keep or discard what was measured. */
			close_record(in, now_tick);
			R.fsm_state = REARM_FSM_IDLE;
			break;
		}
		{
			/* Recovery-side stages (shared with the SUSPENDED branch's PERMISSION tick - Bug 2),
			 * plus the two reasons that need the recovery watch: NO_LOAD (the rearm granted but
			 * no pedal load ever appeared) and TIMEOUT. peak_load_centikg keeps being sampled so
			 * NO_LOAD stays meaningful across the whole saga. */
			rearm_delay_record_t *rec = &R.slots[R.cur_slot];
			if (in->snapshot != 0 && in->snapshot->load_centikg > R.peak_load_centikg) {
				R.peak_load_centikg = in->snapshot->load_centikg;
			}
			sample_recovery_chain(in, now_tick);
			uint32_t el = elapsed_ticks(now_tick);
			if (el >= REARM_DELAY_NO_LOAD_TICKS && R.peak_load_centikg == 0U &&
			    (rec->reason_bits & REARM_DELAY_REASON_NO_LOAD) == 0U) {
				rec->reason_bits = (uint8_t)(rec->reason_bits | REARM_DELAY_REASON_NO_LOAD);
				note_problem(in, now_tick);
			}
			if (el >= REARM_DELAY_TIMEOUT_TICKS) {
				close_record(in, now_tick);
				R.fsm_state = REARM_FSM_IDLE;
			}
		}
		break;
	}

	/*
	 * ownership_end_edge - see the top-of-function comment for the full rationale.
	 *   record_truly_concluded: a record WAS tracking (fsm_before SUSPENDED/RECOVERING) and just
	 *     closed (fsm now IDLE). In v2 every record close concludes its saga - the v1
	 *     WAITING->SUSPENDED re-suspend that kept the same reservation alive no longer exists.
	 *   fallback_saga_end: NO record was tracking (fsm stayed IDLE the whole time - the queue was
	 *     full when the prearm edge fired) and the raw session_state signals the saga concluded
	 *     anyway: permission (SUSPENDED->ACTIVE) or terminal (->COLD from ACTIVE/SUSPENDED).
	 * Computed BEFORE the post-switch re-open below, so an interrupting prearm edge that just
	 * closed the previous RECOVERING record is suppressed (prearm_edge), not reported.
	 */
	bool fsm_closed = (fsm_before != REARM_FSM_IDLE) && (R.fsm_state == REARM_FSM_IDLE);
	bool fallback_saga_end = (fsm_before == REARM_FSM_IDLE) &&
		((st == 1U && R.prev_session_state == 2U) ||
		 (st == 0U && (R.prev_session_state == 1U || R.prev_session_state == 2U)));
	R.ownership_end_edge = (fsm_closed || fallback_saga_end) && !R.prearm_edge;

	/*
	 * v2: a NEW saga may interrupt the previous one's recovery watch ON THIS SAME TICK - the
	 * RECOVERING branch above closed the old record when st left ACTIVE, and prearm_edge fired
	 * for the new reverse. The record's anchor is the prearm edge, so the new saga's record must
	 * open right here (the IDLE branch never ran - fsm was RECOVERING at switch time), not on
	 * some later WAIT entry. ownership_end_edge above was already computed from the OLD fsm state.
	 */
	if (!edge_handled && R.fsm_state == REARM_FSM_IDLE && R.prearm_edge) {
		if (R.q_depth >= REARM_DELAY_RECORDS) {
			R.rejected_total++;
		} else {
			open_record(in, now_tick);
			R.fsm_state = REARM_FSM_SUSPENDED;
		}
	}

	R.prev_session_state = st;
}

bool rearm_delay_prearm_edge(void) { return R.prearm_edge; }
bool rearm_delay_ownership_end_edge(void) { return R.ownership_end_edge; }

bool rearm_delay_reserve_trigger(void) { return R.reserve_trigger; }

/*
 * v5 (Bug 2 fix) + v5.1: report the outcome of the TRACE/RAW attempt. Called exactly once per
 * record (on the trigger tick). The id and status are stored IN the record so an offline parser
 * can join the FW-111 record to its pas_trace / pas_raw captures by id.
 *
 * The record whose PROBLEM fired may ALREADY have closed and been queued THIS SAME TICK (a
 * same-tick note_problem()+close_record(), e.g. a terminal SUSPENDED->COLD before WAIT_LONG) -
 * R.cur_slot is 0xFF by the time this runs, and even if it were not, the
 * PHYSICAL slots[] index a record occupies is not a stable key: rearm_delay_queue_release_session()
 * shifts the ring when an earlier session's record is dequeued, which could happen between the
 * trigger tick and whenever a caller gets around to reporting the result.
 *
 * v5.1: R.pending_record_uid - stamped by note_problem() the instant the trigger first latched,
 * from whichever record was open THEN - is the internal identity used to find that SAME record
 * wherever it now sits: still open (R.cur_slot), already in the queue, or (after the ring
 * shifted) at a different physical slot. It is NOT the wire (record_id, session_id) pair, which
 * after a record_seq wrap can be identical for two different records in the same session; the
 * UID sidecar travels with the record on every queue shift, so a search by UID can never land on
 * the wrong one. If no record matches (the record itself was refused - every slot already held a
 * queued record when it tried to close, already counted via rejected_total; or it was released
 * from the queue before this ran), the result is silently discarded: there is nowhere left to
 * attach it to, exactly like the record itself was discarded. Never writes to the WRONG record,
 * never leaves a stray latch set.
 */
void rearm_delay_note_reserve_done(uint8_t capture_id, uint8_t capture_status)
{
	if (R.reserve_trigger) {
		rearm_delay_record_t *target = 0;
		if (R.cur_slot != 0xFFU &&
		    R.record_uid[R.cur_slot] == R.pending_record_uid) {
			target = &R.slots[R.cur_slot];
		} else {
			for (uint8_t i = 0; i < R.q_depth; i++) {
				uint8_t idx = (uint8_t)((R.q_head + i) % REARM_DELAY_RECORDS);
				if (R.record_uid[idx] == R.pending_record_uid) {
					target = &R.slots[idx];
					break;
				}
			}
		}
		if (target != 0) {
			target->capture_id = capture_id;
			target->capture_status = capture_status;
		}
	}
	R.reserve_trigger = false;
}

/* --- diag_record_source bridge, same shape as ride_episode's ---------------------------------- */

static int32_t find_session_index(uint8_t session_id)
{
	for (uint8_t i = 0; i < R.q_depth; i++) {
		uint8_t idx = (uint8_t)((R.q_head + i) % REARM_DELAY_RECORDS);
		if (R.slots[idx].session_id == session_id) return (int32_t)i;
	}
	return -1;
}

uint16_t rearm_delay_queue_count_session(uint8_t session_id)
{
	uint16_t n = 0;
	for (uint8_t i = 0; i < R.q_depth; i++) {
		uint8_t idx = (uint8_t)((R.q_head + i) % REARM_DELAY_RECORDS);
		if (R.slots[idx].session_id == session_id) n++;
	}
	return n;
}

bool rearm_delay_queue_peek_session(uint8_t session_id, rearm_delay_record_t *out)
{
	if (out == 0) return false;
	int32_t off = find_session_index(session_id);
	if (off < 0) return false;
	*out = R.slots[(uint8_t)((R.q_head + (uint8_t)off) % REARM_DELAY_RECORDS)];
	return true;
}

void rearm_delay_queue_release_session(uint8_t session_id)
{
	int32_t off = find_session_index(session_id);
	if (off < 0) return;
	/* Same general form as ride_episode: shift the ring so the oldest of THIS session goes.
	 * In practice the dump drains oldest-first, so off==0 and this is a plain head pop.
	 * v5.1: the record_uid sidecar MUST shift together with the record - a record that moves
	 * to a different physical slot carries its identity with it, so a later note_reserve_done()
	 * search by UID still finds it at its new slot. */
	for (int32_t i = off; i > 0; i--) {
		uint8_t dst = (uint8_t)((R.q_head + (uint8_t)i) % REARM_DELAY_RECORDS);
		uint8_t src = (uint8_t)((R.q_head + (uint8_t)i - 1U) % REARM_DELAY_RECORDS);
		R.slots[dst] = R.slots[src];
		R.record_uid[dst] = R.record_uid[src];
	}
	R.q_head = (uint8_t)((R.q_head + 1U) % REARM_DELAY_RECORDS);
	R.q_depth--;
}

uint32_t rearm_delay_queue_enqueued(void) { return R.accepted_total; }
uint32_t rearm_delay_queue_rejected(void) { return R.rejected_total; }

uint8_t rearm_delay_fsm_state(void) { return (uint8_t)R.fsm_state; }

#ifdef REARM_UID_SEAM_TEST
/*
 * v5.1 TEST-ONLY SEAM - compiled ONLY when REARM_UID_SEAM_TEST is defined (host tests only;
 * never by any firmware build - verified with nm). Lets a test drive the UID generator right up
 * against UINT32_MAX so the wrap and the skip-in-use rules can be proven deterministically.
 */
void rearm_delay_test_set_uid_next(uint32_t v) { R.uid_next = v; }
uint32_t rearm_delay_test_uid_next(void) { return R.uid_next; }
#endif /* REARM_UID_SEAM_TEST */

#else  /* !CAN_DIAGNOSTICS_ENABLE */

/* FW-111: like every other diagnostic module, the recorder costs ZERO RAM in the production
 * build - see pas_raw.c for why this is a #if rather than a reliance on --gc-sections. */
typedef int rearm_delay_diag_not_compiled_in;

#endif /* CAN_DIAGNOSTICS_ENABLE */
