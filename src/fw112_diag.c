#include "fw112_diag.h"
#include "diag_budget.h"

#if CAN_DIAGNOSTICS_ENABLE

#include <string.h>

/*
 * FW-112-DIAG: see the header for what this records and why. Implementation notes only.
 *
 * All mutable state lives in the single struct R below so the RAM budget check measures the
 * whole module, and the module is driven entirely through fw112_diag_input_t - the session /
 * recovery / direction states arrive as plain bytes, the torque/Iq chain as plain scalars,
 * nothing is linked from ride_session.c, ride_control.c or main.c.
 *
 * Every event is an EDGE captured on the tick it first fires - never a per-tick spam. A record
 * is appended only when the event's contiguous stretch begins (BLOCKED / ZEROED) or the state
 * machine transitions (session / recovery / hold). The queue is per-session and refuses rather
 * than overwrites when full, exactly like rearm_delay_diag.c: a dump must never lose a
 * session's history, and the refusal is surfaced (rejected_total -> DIAG_ERR_CAPTURES_FULL +
 * DIAG_TRAILER_F_FW112_REJECTED) so recorder saturation is visible in the log rather than
 * silent (the card's point 10).
 */

/*
 * ALL of this module's mutable state, in one object (same discipline as rearm_delay_diag.c's
 * R and diag_session.c's D - see inc/diag_budget.h for why).
 */
static struct {
	/* The queue of records awaiting dump. In-place: the slot being appended NOW is the free
	 * queue slot (q_head + q_depth) % FW112_DIAG_RECORDS. */
	fw112_diag_record_t slots[FW112_DIAG_RECORDS];
	uint8_t q_head;         /* oldest queued record */
	uint8_t q_depth;        /* number of records awaiting dump */
	uint32_t accepted_total;   /* records committed to the queue */
	uint32_t rejected_total;   /* events refused because every slot held a queued record */

	uint8_t  session_id;       /* stamped into every record opened now */
	uint16_t event_id;         /* monotonic per-record id, wraps at 65536 - a key, never a count */

	/* Edge detection: the previous tick's observations. */
	uint8_t  prev_session_state;
	uint8_t  prev_recovery_state;
	uint16_t prev_hold_ticks;
	bool     blocked_stretch;  /* fwd_run > 0 && !latched */
	bool     zeroed_stretch;   /* fwd_run > 0 && latched && iq_setpoint == 0 */

	/* The tick of the previously recorded event, so each record carries the gap to its
	 * predecessor. The first-ever event has no predecessor (ever_recorded false) and records 0. */
	uint32_t last_event_tick;
	bool     ever_recorded;
} R;

_Static_assert(sizeof(R) <= DIAG_BUDGET_FW112_DIAG_BYTES,
	"FW-112-DIAG: fw112_diag's total state exceeds its RAM line item");

_Static_assert(sizeof(fw112_diag_record_t) == 32U,
	"FW-112-DIAG: the record must stay exactly 32 B so 4 data frames serialize it with no padding");

static void append_record(const fw112_diag_input_t *in, uint32_t now_tick, uint8_t event_type)
{
	if (R.q_depth >= FW112_DIAG_RECORDS) {
		R.rejected_total++;
		return;
	}
	uint8_t idx = (uint8_t)((R.q_head + R.q_depth) % FW112_DIAG_RECORDS);
	fw112_diag_record_t *rec = &R.slots[idx];
	memset(rec, 0, sizeof(*rec));

	rec->event_id = R.event_id;
	R.event_id++;
	rec->session_id = R.session_id;
	rec->event_type = event_type;
	rec->reason_bits = in->reason_bits;
	rec->flags = (uint8_t)(
		(in->latched ? FW112_FLAG_LATCHED : 0U) |
		(in->pwm_on ? FW112_FLAG_PWM_ON : 0U) |
		(in->torque_sensor_valid ? FW112_FLAG_SENSOR_VALID : 0U) |
		(in->cal_user ? FW112_FLAG_CAL_USER : 0U) |
		(in->wheel_valid ? FW112_FLAG_WHEEL_VALID : 0U) |
		(in->rolling_coast ? FW112_FLAG_ROLLING_COAST : 0U));
	/* A GRANTED record's own edge (which state ACTIVE was re-entered from) tells whether it
	 * was a fast rearm (SUSPENDED_BY_DIRECTION) or a cold start (COLD) - computed here from the
	 * previous session_state, before this tick's update overwrites it. */
	if (event_type == FW112_EVT_PERMISSION_GRANTED) {
		if (R.prev_session_state == 2U) rec->flags |= FW112_FLAG_FAST_REARM;
		else if (R.prev_session_state == 0U) rec->flags |= FW112_FLAG_COLD_ARM;
	}
	rec->session_state = in->session_state;
	rec->dir_state = in->dir_state;
	rec->recovery_state = in->recovery_state;
	rec->fwd_run = in->fwd_run;
	rec->crank_forward_steps = in->crank_forward_steps;
	rec->required_steps = in->required_steps;
	rec->start_steps = in->start_steps;
	rec->cadence_rpm = in->cadence_rpm;
	rec->assist_hold_ticks = in->assist_hold_ticks;
	rec->load_centikg = in->load_centikg;
	rec->load_threshold_centikg = in->load_threshold_centikg;
	rec->iq_request = in->iq_request;
	rec->iq_pre_ramp = in->iq_pre_ramp;
	rec->iq_setpoint = in->iq_setpoint;
	rec->iq_actual = in->iq_actual;
	rec->elapsed_ticks = R.ever_recorded ? (now_tick - R.last_event_tick) : 0U;

	R.q_depth++;
	R.accepted_total++;
	R.last_event_tick = now_tick;
	R.ever_recorded = true;
}

void fw112_diag_init(void)
{
	memset(&R, 0, sizeof(R));
	R.prev_session_state = 0xFFU;
	R.prev_recovery_state = 0xFFU;
}

void fw112_diag_set_session_id(uint8_t session_id)
{
	R.session_id = session_id;
}

void fw112_diag_tick(const fw112_diag_input_t *in, uint32_t now_tick)
{
	if (in == 0) {
		R.prev_session_state = 0xFFU;
		R.prev_recovery_state = 0xFFU;
		R.blocked_stretch = false;
		R.zeroed_stretch = false;
		return;
	}

	uint8_t st = in->session_state;
	uint8_t rec = in->recovery_state;

	/* --- session edges ---------------------------------------------------------------- */
	if (st != R.prev_session_state && R.prev_session_state != 0xFFU) {
		if (st == 1U) {
			/* Entered ACTIVE: cold start or fast rearm - the flag field tells which. */
			append_record(in, now_tick, (uint8_t)FW112_EVT_PERMISSION_GRANTED);
		} else if (R.prev_session_state == 1U) {
			/* Left ACTIVE: a suspension (direction hold, or FW-112.2 rolling coast - the flags
			 * byte's ROLLING_COAST / reason_bits tell which) or a terminal event (COLD). */
			append_record(in, now_tick, (uint8_t)FW112_EVT_PERMISSION_REVOKED);
		}
	}

	/* --- recovery automaton edges ------------------------------------------------------ */
	if (rec != R.prev_recovery_state && R.prev_recovery_state != 0xFFU) {
		if (rec != 0U && R.prev_recovery_state == 0U) {
			/* IDLE -> WAIT/TRACK: begin_rolling_rearm() opened the recovery. */
			append_record(in, now_tick, (uint8_t)FW112_EVT_RECOVERY_ENTER);
		} else if (rec == 0U && R.prev_recovery_state != 0U) {
			/* non-IDLE -> IDLE: recovery completed or was cancelled. */
			append_record(in, now_tick, (uint8_t)FW112_EVT_RECOVERY_EXIT);
		} else if (rec == 1U && R.prev_recovery_state == 2U) {
			/* TRACK_FAST -> WAIT_FRESH_LOAD: pressure lost mid-recovery. */
			append_record(in, now_tick, (uint8_t)FW112_EVT_RECOVERY_COLLAPSE);
		}
	}

	/* --- ride-latch hold grace --------------------------------------------------------- */
	if (in->latched) {
		if (R.prev_hold_ticks == 0U && in->assist_hold_ticks > 0U) {
			/* Armed (or refreshed from a zero state) by a positive mode demand. */
			append_record(in, now_tick, (uint8_t)FW112_EVT_HOLD_ARMED);
		} else if (R.prev_hold_ticks > 0U && in->assist_hold_ticks == 0U) {
			/* Counted down to zero while still latched: the grace expired. */
			append_record(in, now_tick, (uint8_t)FW112_EVT_HOLD_EXPIRED);
		}
	}

	/* --- the card's two questions, as contiguous stretches ------------------------------ */
	bool blocked_now = in->fwd_run > 0U && !in->latched;
	if (blocked_now && !R.blocked_stretch) {
		/* Entered "pedalling forward but no permission". reason_bits names the holding stage. */
		append_record(in, now_tick, (uint8_t)FW112_EVT_BLOCKED);
	}
	R.blocked_stretch = blocked_now;

	bool zeroed_now = in->fwd_run > 0U && in->latched && in->iq_setpoint == 0;
	if (zeroed_now && !R.zeroed_stretch) {
		/* Entered "permission present, nothing reaching the motor" - WHO-ZEROED, live. */
		append_record(in, now_tick, (uint8_t)FW112_EVT_ZEROED);
	}
	R.zeroed_stretch = zeroed_now;

	R.prev_session_state = st;
	R.prev_recovery_state = rec;
	R.prev_hold_ticks = in->assist_hold_ticks;
}

/* --- diag_record_source bridge, same shape as rearm_delay_diag's ---------------------------- */

static int32_t find_session_index(uint8_t session_id)
{
	for (uint8_t i = 0; i < R.q_depth; i++) {
		uint8_t idx = (uint8_t)((R.q_head + i) % FW112_DIAG_RECORDS);
		if (R.slots[idx].session_id == session_id) return (int32_t)i;
	}
	return -1;
}

uint16_t fw112_diag_queue_count_session(uint8_t session_id)
{
	uint16_t n = 0;
	for (uint8_t i = 0; i < R.q_depth; i++) {
		uint8_t idx = (uint8_t)((R.q_head + i) % FW112_DIAG_RECORDS);
		if (R.slots[idx].session_id == session_id) n++;
	}
	return n;
}

bool fw112_diag_queue_peek_session(uint8_t session_id, fw112_diag_record_t *out)
{
	if (out == 0) return false;
	int32_t off = find_session_index(session_id);
	if (off < 0) return false;
	*out = R.slots[(uint8_t)((R.q_head + (uint8_t)off) % FW112_DIAG_RECORDS)];
	return true;
}

void fw112_diag_queue_release_session(uint8_t session_id)
{
	int32_t off = find_session_index(session_id);
	if (off < 0) return;
	for (int32_t i = off; i > 0; i--) {
		uint8_t dst = (uint8_t)((R.q_head + (uint8_t)i) % FW112_DIAG_RECORDS);
		uint8_t src = (uint8_t)((R.q_head + (uint8_t)i - 1U) % FW112_DIAG_RECORDS);
		R.slots[dst] = R.slots[src];
	}
	R.q_head = (uint8_t)((R.q_head + 1U) % FW112_DIAG_RECORDS);
	R.q_depth--;
}

uint32_t fw112_diag_queue_enqueued(void) { return R.accepted_total; }
uint32_t fw112_diag_queue_rejected(void) { return R.rejected_total; }

#else  /* !CAN_DIAGNOSTICS_ENABLE */

/* FW-112-DIAG: like every other diagnostic module, the recorder costs ZERO RAM in the production
 * build - see pas_raw.c for why this is a #if rather than a reliance on --gc-sections. */
typedef int fw112_diag_not_compiled_in;

#endif /* CAN_DIAGNOSTICS_ENABLE */
