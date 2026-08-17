/*
 * FW-112-DIAG: host tests for src/fw112_diag.c - the whole-chain EVENT RECORDER, compiled and
 * run on THIS machine (the REAL module, driven with a synthetic per-tick observation stream).
 *
 * The module is deliberately not linked against ride_session.c/ride_control.c/torque_input.c; its
 * whole input surface is fw112_diag_input_t, so this harness drives every event and every
 * snapshot field directly, with no stubs - exactly the discipline rearm_delay_diag_host.c uses.
 * Session/recovery/direction states arrive as the plain bytes main.c would pass.
 *
 * Session state bytes (the numeric encoding the header documents): 0 COLD, 1 ACTIVE,
 * 2 SUSPENDED_BY_DIRECTION, 3 = the legacy reserved WAIT_REARM_LOAD value (never entered by
 * production since FW-112 v2; the recorder still treats any byte != 1 as "not ACTIVE").
 * Recovery state bytes: 0 IDLE, 1 WAIT_FRESH_LOAD, 2 TRACK_FAST (see torque_recovery_state_t).
 *
 * What these tests pin down (S1..S10) and what each M-mutation is (M1..M5):
 *
 *   S1  a single BLOCKED edge (fwd_run>0 && !latched) is recorded exactly once; the contiguous
 *       stretch does not re-fire on later ticks.
 *   S2  BLOCKED carries the permission reason_bits from the deciding layer (ride_control).
 *   S3  the GRANTED edge (state -> 1) is recorded once, with the correct flags (cold start from
 *       COLD / fast rearm from SUSPENDED) and the chain snapshot.
 *   S4  the REVOKED edge (state 1 -> not 1) is recorded once.
 *   S5  the recovery automaton edges ENTER / EXIT / COLLAPSE are each recorded once on their
 *       transition ticks.
 *   S6  the hold grace edges HOLD_ARMED / HOLD_EXPIRED are recorded on the arming and expiring
 *       ticks.
 *   S7  a ZEROED edge (fwd_run>0 && latched && iq_setpoint==0) is recorded, carrying the
 *       WHO-ZEROED reason.
 *   S8  elapsed_ticks is the gap since the previous event (first event = 0); event_id is
 *       monotonic; each record is exactly 32 B.
 *   S9  the queue is per-session: count/peek/release only touch the named session; releasing one
 *       record shifts the ring correctly.
 *   S10 the queue refuses (not overwrites) when full: rejected_total rises, the first record
 *       survives, and enqueued/rejected counters are exact.
 *   S11 (FW-112-DIAG.1) the capacity is FW112_DIAG_RECORDS=24: 24 driven events are all
 *       accepted, count == capacity, nothing refused below capacity.
 *   S12 (FW-112-DIAG.1) the 25th event is refused: rejected_total rises by one, count stays at
 *       capacity, enqueued unchanged - reject-on-full, never overwrite.
 *   S13 (FW-112-DIAG.1) the accepted records 0..(capacity-1) all survive, in order, none
 *       overwritten by the refused 25th.
 *   S14 (FW-112-DIAG.1) the realistic >8-event sequence that motivated the change - COLD BLOCKED,
 *       cold GRANTED, HOLD_ARMED, ZEROED, REVOKED, BLOCKED, fast-rearm GRANTED, RECOVERY_ENTER,
 *       COLLAPSE, EXIT, ZEROED - is recorded end to end with the exact types and cold/fast flags.
 *
 *   M1  the recorder drops reason_bits entirely (reason always 0) -> S2 and S7 fail, because
 *       both assert the deciding-layer reason reached the record unchanged.
 *   M2  the session edge detection is removed (GRANTED/REVOKED never fire) -> S3 and S4 fail.
 *   M3  the blocked/zeroed stretch flags are removed (per-tick spam instead of an edge) -> S1
 *       and S7 fail, because a stretch then produces one record per tick instead of one.
 *   M4  the recovery/hold edge detection is removed -> S5 and S6 fail.
 *   M5  event_id is not incremented (every record id 0) -> S8 fails on the monotonic ids.
 *
 *   FW-112-DIAG.1 mutations:
 *   M1  FW112_DIAG_RECORDS set back to 8 -> S11 (count == 24 fails, only 8 accepted) and S14
 *       (11 events > 8, the tail is refused) fail.
 *   M2  the reject check allows only capacity-1 records -> S11 fails (count == 23).
 *   M3  the reject guard is removed so the 25th is written out of bounds -> S12 (refused == 0,
 *       count drifts past capacity) and S13 (the oldest record is overwritten) fail.
 *
 * Each mutation is caught by the corresponding BEHAVIOURAL test above (the real module linked),
 * not by a source-text scan: S2/S7 name the reason, S1/S7 name the edge-vs-spam count, S3/S4
 * name the session edge, S5/S6 name the recovery/hold edges, S8 names the monotonic ids, and
 * S11/S12/S13/S14 name the capacity, the reject boundary and the preserved order.
 */

#include "../common/check.h"

#include "fw112_diag.h"
#include "diag_budget.h"

#include <string.h>

static uint32_t tick;
static fw112_diag_input_t in;

/* State byte aliases (documented in the header / module). */
static const uint8_t ST_COLD      = 0U;
static const uint8_t ST_ACTIVE    = 1U;
static const uint8_t ST_SUSPENDED = 2U;
static const uint8_t REC_IDLE     = 0U;
static const uint8_t REC_WAIT     = 1U;
static const uint8_t REC_TRACK    = 2U;

static void reset(void)
{
	tick = 0;
	memset(&in, 0, sizeof(in));
	fw112_diag_init();
	fw112_diag_set_session_id(1);
}

static void step(void)
{
	fw112_diag_tick(&in, tick);
	tick++;
}

static void step_n(uint32_t n)
{
	for (uint32_t i = 0; i < n; i++) step();
}

/* Set the baseline "rider pedalling forward, permission held, demand flowing" observation. */
static void baseline(void)
{
	in.session_state = ST_ACTIVE;
	in.recovery_state = REC_IDLE;
	in.dir_state = 1;
	in.fwd_run = 6;
	in.crank_forward_steps = 6;
	in.required_steps = 4;
	in.start_steps = 4;
	in.latched = true;
	in.iq_setpoint = 500;
	in.iq_request = 500;
	in.iq_pre_ramp = 500;
	in.iq_actual = 490;
	in.reason_bits = FW112_REASON_NONE;
}

static void drain_queue(uint8_t session_id)
{
	fw112_diag_record_t r;
	while (fw112_diag_queue_peek_session(session_id, &r)) {
		fw112_diag_queue_release_session(session_id);
	}
}

/* ---------------- S1 / S3 / S4 / M3 : session edges and BLOCKED stretch ---------------------- */

static void test_session_edges_and_blocked(void)
{
	reset();

	/* Start COLD, pedalling forward, no latch -> BLOCKED edge on the first fwd tick. */
	in.session_state = ST_COLD;
	in.fwd_run = 1;
	in.reason_bits = FW112_REASON_START_STEPS | FW112_REASON_DIRECTION;
	step();
	/* Keep the stretch going for a while - must NOT re-fire. */
	in.fwd_run = 8;
	step_n(50);
	CHECK(fw112_diag_queue_count_session(1) == 1U, "S1: one BLOCKED edge for the whole stretch");
	fw112_diag_record_t r;
	CHECK(fw112_diag_queue_peek_session(1, &r), "S1: record present");
	CHECK(r.event_type == FW112_EVT_BLOCKED, "S1: event is BLOCKED");
	CHECK(r.event_id == 0U, "S1: first event id 0");
	CHECK(r.elapsed_ticks == 0U, "S1: first event elapsed 0");
	CHECK(r.session_state == ST_COLD, "S1: snapshot session COLD");
	CHECK(r.reason_bits == (FW112_REASON_START_STEPS | FW112_REASON_DIRECTION),
	      "S2: BLOCKED carries the deciding-layer reason");
	drain_queue(1);

	/* Cold start: COLD -> ACTIVE = GRANTED with COLD_ARM flag, exactly one. */
	reset();
	in.session_state = ST_COLD;
	in.fwd_run = 1;
	in.latched = false;
	step();                                   /* BLOCKED */
	in.session_state = ST_ACTIVE;
	in.latched = true;
	in.iq_setpoint = 400;
	in.reason_bits = FW112_REASON_NONE;
	step();                                   /* GRANTED (cold) */
	step_n(40);
	CHECK(fw112_diag_queue_count_session(1) == 2U, "S3: BLOCKED + GRANTED");
	/* Release the first (BLOCKED) to reach GRANTED. */
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_type == FW112_EVT_BLOCKED, "S3: first is BLOCKED");
	fw112_diag_queue_release_session(1);
	CHECK(fw112_diag_queue_peek_session(1, &r), "S3: granted present");
	CHECK(r.event_type == FW112_EVT_PERMISSION_GRANTED, "S3: event is GRANTED");
	CHECK((r.flags & FW112_FLAG_COLD_ARM) != 0U, "S3: COLD_ARM set");
	CHECK((r.flags & FW112_FLAG_LATCHED) != 0U, "S3: LATCHED set");
	CHECK((r.flags & FW112_FLAG_FAST_REARM) == 0U, "S3: not a fast rearm");
	CHECK(r.session_state == ST_ACTIVE, "S3: snapshot ACTIVE");
	CHECK(r.iq_setpoint == 400, "S3: snapshot iq_setpoint");
	drain_queue(1);

	/* Fast rearm: ACTIVE -> SUSPENDED -> ACTIVE = REVOKED then GRANTED(FAST_REARM), one each. */
	reset();
	baseline();
	step_n(5);                                 /* steady ACTIVE, no events */
	in.session_state = ST_SUSPENDED;
	in.latched = false;
	in.fwd_run = 0;                            /* rider stops cranking: no BLOCKED edge */
	in.iq_setpoint = 0;
	step();                                    /* REVOKED */
	step_n(3);
	in.session_state = ST_ACTIVE;
	in.latched = true;
	in.fwd_run = 6;
	in.iq_setpoint = 400;
	step();                                    /* GRANTED (fast rearm) */
	step_n(20);
	CHECK(fw112_diag_queue_count_session(1) == 2U, "S4: REVOKED + GRANTED on a rearm");
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_type == FW112_EVT_PERMISSION_REVOKED,
	      "S4: first is REVOKED");
	fw112_diag_queue_release_session(1);
	CHECK(fw112_diag_queue_peek_session(1, &r), "S4: granted present");
	CHECK(r.event_type == FW112_EVT_PERMISSION_GRANTED, "S4: second is GRANTED");
	CHECK((r.flags & FW112_FLAG_FAST_REARM) != 0U, "S4: FAST_REARM set");
	CHECK((r.flags & FW112_FLAG_COLD_ARM) == 0U, "S4: not a cold arm");
	drain_queue(1);
}

/* ---------------- S5 : recovery automaton edges ---------------------------------------------- */

static void test_recovery_edges(void)
{
	reset();
	baseline();
	step_n(5);

	/* IDLE -> WAIT : ENTER. */
	in.recovery_state = REC_WAIT;
	step();
	CHECK(fw112_diag_queue_count_session(1) == 1U, "S5: RECOVERY_ENTER recorded");
	fw112_diag_record_t r;
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_type == FW112_EVT_RECOVERY_ENTER,
	      "S5: event is RECOVERY_ENTER");
	CHECK(r.recovery_state == REC_WAIT, "S5: snapshot recovery WAIT");
	drain_queue(1);

	/* WAIT -> TRACK : no collapse event (only TRACK -> WAIT collapses). */
	in.recovery_state = REC_TRACK;
	step();
	CHECK(fw112_diag_queue_count_session(1) == 0U, "S5: WAIT->TRACK records nothing");
	step_n(3);

	/* TRACK -> WAIT : COLLAPSE. */
	in.recovery_state = REC_WAIT;
	step();
	CHECK(fw112_diag_queue_count_session(1) == 1U, "S5: RECOVERY_COLLAPSE recorded");
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_type == FW112_EVT_RECOVERY_COLLAPSE,
	      "S5: event is RECOVERY_COLLAPSE");
	CHECK(r.recovery_state == REC_WAIT, "S5: snapshot recovery WAIT");
	drain_queue(1);

	/* WAIT -> IDLE : EXIT. */
	in.recovery_state = REC_IDLE;
	step();
	CHECK(fw112_diag_queue_count_session(1) == 1U, "S5: RECOVERY_EXIT recorded");
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_type == FW112_EVT_RECOVERY_EXIT,
	      "S5: event is RECOVERY_EXIT");
	drain_queue(1);

	/* Staying in a recovery state must not spam. */
	baseline();
	in.recovery_state = REC_TRACK;
	step();
	CHECK(fw112_diag_queue_count_session(1) == 1U, "S5: ENTER recorded");
	drain_queue(1);
	step_n(40);
	CHECK(fw112_diag_queue_count_session(1) == 0U, "S5: no re-fire while recovery steady");
}

/* ---------------- S6 : hold grace edges ------------------------------------------------------- */

static void test_hold_edges(void)
{
	reset();
	baseline();
	in.assist_hold_ticks = 0;
	step_n(5);

	/* Armed: 0 -> positive while latched. */
	in.assist_hold_ticks = 40;
	step();
	CHECK(fw112_diag_queue_count_session(1) == 1U, "S6: HOLD_ARMED recorded");
	fw112_diag_record_t r;
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_type == FW112_EVT_HOLD_ARMED,
	      "S6: event is HOLD_ARMED");
	CHECK(r.assist_hold_ticks == 40, "S6: snapshot hold ticks");
	drain_queue(1);

	/* Decrement but stay positive: no edge. */
	in.assist_hold_ticks = 39;
	step();
	in.assist_hold_ticks = 38;
	step();
	CHECK(fw112_diag_queue_count_session(1) == 0U, "S6: no re-fire on decrement");

	/* Expired: positive -> 0 while latched. */
	in.assist_hold_ticks = 0;
	step();
	CHECK(fw112_diag_queue_count_session(1) == 1U, "S6: HOLD_EXPIRED recorded");
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_type == FW112_EVT_HOLD_EXPIRED,
	      "S6: event is HOLD_EXPIRED");
	drain_queue(1);

	/* Hold is only tracked while latched: when not latched, a 0->positive change records nothing. */
	baseline();
	in.latched = false;
	in.fwd_run = 0;                            /* no cranking: no BLOCKED edge */
	in.assist_hold_ticks = 40;
	step();
	CHECK(fw112_diag_queue_count_session(1) == 0U, "S6: hold edge ignored while not latched");
}

/* ---------------- S7 : ZEROED stretch --------------------------------------------------------- */

static void test_zeroed(void)
{
	reset();
	baseline();
	step_n(5);   /* ACTIVE, no events */

	/* Permission present (latched), pedalling, but iq_setpoint 0 -> ZEROED edge, once. */
	in.iq_setpoint = 0;
	in.iq_request = 0;
	in.reason_bits = FW112_REASON_MODE_ZERO;
	step();
	CHECK(fw112_diag_queue_count_session(1) == 1U, "S7: ZEROED recorded on entry");
	fw112_diag_record_t r;
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_type == FW112_EVT_ZEROED,
	      "S7: event is ZEROED");
	CHECK(r.reason_bits == FW112_REASON_MODE_ZERO, "S7: ZEROED carries WHO-ZEROED reason");
	CHECK(r.iq_setpoint == 0, "S7: snapshot iq_setpoint 0");
	CHECK((r.flags & FW112_FLAG_LATCHED) != 0U, "S7: LATCHED set (permission present)");
	drain_queue(1);

	/* Stay zeroed: no re-fire. */
	step_n(30);
	CHECK(fw112_diag_queue_count_session(1) == 0U, "S7: no re-fire while zeroed stretch");

	/* Not zeroed if not latched (that is BLOCKED territory): fwd_run>0 && !latched && iq 0. */
	reset();
	baseline();
	in.latched = false;
	in.iq_setpoint = 0;
	in.reason_bits = FW112_REASON_LEVEL_ZERO;
	step();
	fw112_diag_record_t r2;
	CHECK(fw112_diag_queue_count_session(1) == 1U, "S7: one event when not latched with iq 0");
	CHECK(fw112_diag_queue_peek_session(1, &r2) && r2.event_type == FW112_EVT_BLOCKED,
	      "S7: it is BLOCKED, not ZEROED");
	drain_queue(1);
}

/* ---------------- S8 : event_id monotonicity + record size ------------------------------------ */

static void test_event_id_and_size(void)
{
	reset();
	CHECK(sizeof(fw112_diag_record_t) == 32U, "S8: record is exactly 32 B");

	baseline();
	in.session_state = ST_COLD;
	in.fwd_run = 1;
	in.latched = false;
	step();                        /* BLOCKED, id 0, elapsed 0 */

	step_n(20);                    /* 20 ticks of nothing */
	in.session_state = ST_ACTIVE;
	in.latched = true;
	in.iq_setpoint = 300;
	step();                        /* GRANTED, id 1, elapsed 20 */
	step_n(7);
	in.session_state = ST_SUSPENDED;
	in.latched = false;
	in.fwd_run = 0;                            /* no cranking: no BLOCKED edge */
	in.iq_setpoint = 0;
	step();                        /* REVOKED, id 2, elapsed 8 */

	CHECK(fw112_diag_queue_count_session(1) == 3U, "S8: three events");
	fw112_diag_record_t r;
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_id == 0U && r.elapsed_ticks == 0U,
	      "S8: first event id 0 elapsed 0");
	fw112_diag_queue_release_session(1);
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_id == 1U && r.elapsed_ticks == 21U,
	      "S8: second event id 1 elapsed 21");
	fw112_diag_queue_release_session(1);
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_id == 2U && r.elapsed_ticks == 8U,
	      "S8: third event id 2 elapsed 8");
	drain_queue(1);
}

/* ---------------- S9 : per-session queue + release ring shift --------------------------------- */

static void test_session_separation_and_ring(void)
{
	reset();
	baseline();
	fw112_diag_set_session_id(1);

	/* session 1: BLOCKED then GRANTED. */
	in.session_state = ST_COLD;
	in.fwd_run = 1;
	in.latched = false;
	step();                        /* BLOCKED in s1 */
	in.session_state = ST_ACTIVE;
	in.latched = true;
	in.iq_setpoint = 300;
	step();                        /* GRANTED in s1 */
	step_n(3);

	/* session 2: one event. */
	fw112_diag_set_session_id(2);
	in.session_state = ST_SUSPENDED;
	in.latched = false;
	in.fwd_run = 0;                            /* no cranking: no BLOCKED edge in s2 */
	in.iq_setpoint = 0;
	step();                        /* REVOKED in s2 */

	CHECK(fw112_diag_queue_count_session(1) == 2U, "S9: session 1 has 2 records");
	CHECK(fw112_diag_queue_count_session(2) == 1U, "S9: session 2 has 1 record");

	/* Releasing session 1's first record must not disturb session 2. */
	fw112_diag_record_t r;
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_type == FW112_EVT_BLOCKED,
	      "S9: s1 first is BLOCKED");
	fw112_diag_queue_release_session(1);
	CHECK(fw112_diag_queue_count_session(1) == 1U, "S9: s1 now has 1");
	CHECK(fw112_diag_queue_count_session(2) == 1U, "S9: s2 untouched by s1 release");
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_type == FW112_EVT_PERMISSION_GRANTED,
	      "S9: s1 remaining is GRANTED");
	drain_queue(1);
	drain_queue(2);
	CHECK(fw112_diag_queue_count_session(1) == 0U && fw112_diag_queue_count_session(2) == 0U,
	      "S9: both drained");
}

/* ---------------- S10 : queue full refusal (reject, never overwrite) --------------------------- */

static void test_queue_full_refused(void)
{
	reset();
	baseline();
	fw112_diag_set_session_id(1);

	/* Generate FW112_DIAG_RECORDS + 1 distinct events by oscillating ACTIVE/SUSPENDED. */
	uint32_t made = 0;
	in.session_state = ST_ACTIVE;
	in.latched = true;
	in.iq_setpoint = 300;
	step();                        /* (nothing: first tick from a fresh COLD-less baseline) */

	/* Drive an alternating pattern to create one event per cycle. */
	uint32_t guard = 0;
	while (made < FW112_DIAG_RECORDS + 2U && guard < 1000U) {
		guard++;
		in.session_state = ST_SUSPENDED;
		in.latched = false;
		in.fwd_run = 0;            /* no cranking on the reverse: no BLOCKED edge */
		in.iq_setpoint = 0;
		step();                    /* REVOKED (id increments) */
		made++;
		in.session_state = ST_ACTIVE;
		in.latched = true;
		in.fwd_run = 6;
		in.iq_setpoint = 300;
		step();                    /* GRANTED (id increments) */
		made++;
	}

	/* Count events that actually committed before saturation. */
	fw112_diag_record_t r;
	uint32_t n = 0;
	while (fw112_diag_queue_peek_session(1, &r)) { n++; fw112_diag_queue_release_session(1); }
	/* The queue can only ever hold FW112_DIAG_RECORDS; extra events must have been refused. */
	CHECK(n == FW112_DIAG_RECORDS, "S10: queue holds at most FW112_DIAG_RECORDS");
	CHECK(fw112_diag_queue_rejected() == (made - FW112_DIAG_RECORDS),
	      "S10: refused count = events beyond capacity");
	CHECK(fw112_diag_queue_enqueued() == FW112_DIAG_RECORDS,
	      "S10: enqueued counts the committed (capacity) events");
	CHECK(made > FW112_DIAG_RECORDS, "S10: test really saturated the queue");
}

/* ------- S11 / S12 / S13 : 24-record capacity, reject the 25th, no overwrite -------------------- */

/* FW-112-DIAG.1's concrete requirement: exactly 24 records. Kept as a literal, NOT as
 * FW112_DIAG_RECORDS, so a capacity regression (e.g. mutation M1 set it back to 8) is caught by
 * S11/S12/S13 instead of silently scaling the test to the wrong capacity. */
static const uint32_t CAPACITY_REQUIRED = 24U;

static void test_capacity_reject_preserve(void)
{
	reset();
	baseline();
	fw112_diag_set_session_id(1);

	/* Fill exactly CAPACITY_REQUIRED events by oscillating ACTIVE/SUSPENDED. */
	in.session_state = ST_ACTIVE;
	in.latched = true;
	in.fwd_run = 6;
	in.iq_setpoint = 300;
	step();                        /* sentinel capture, no event */

	uint32_t made = 0;
	uint32_t guard = 0;
	while (made < CAPACITY_REQUIRED && guard < 1000U) {
		guard++;
		in.session_state = ST_SUSPENDED;
		in.latched = false;
		in.fwd_run = 0;
		in.iq_setpoint = 0;
		step();                    /* REVOKED */
		made++;
		in.session_state = ST_ACTIVE;
		in.latched = true;
		in.fwd_run = 6;
		in.iq_setpoint = 300;
		step();                    /* GRANTED */
		made++;
	}

	/* S11: every one of the 24 events was accepted, none lost, count exact. */
	CHECK(made == CAPACITY_REQUIRED, "S11: exactly 24 events were driven");
	CHECK(fw112_diag_queue_count_session(1) == CAPACITY_REQUIRED,
	      "S11: all 24 accepted (record_count == 24)");
	CHECK(fw112_diag_queue_enqueued() == CAPACITY_REQUIRED,
	      "S11: enqueued == 24");
	CHECK(fw112_diag_queue_rejected() == 0U, "S11: nothing refused below capacity");

	/* S12: the 25th event is refused; the queue stays exactly full and still valid. */
	in.session_state = ST_SUSPENDED;
	in.latched = false;
	in.fwd_run = 0;
	in.iq_setpoint = 0;
	step();                        /* REVOKED -> must be refused */
	CHECK(fw112_diag_queue_rejected() == 1U, "S12: the 25th event was refused");
	CHECK(fw112_diag_queue_count_session(1) == CAPACITY_REQUIRED,
	      "S12: record_count unchanged after refusal (reject, never overwrite)");
	CHECK(fw112_diag_queue_enqueued() == CAPACITY_REQUIRED,
	      "S12: enqueued unchanged after refusal");

	/* S13: records 0..23 all survived, in order, none overwritten by the 25th. */
	fw112_diag_record_t r;
	uint32_t n = 0;
	uint16_t expect = 0;
	while (fw112_diag_queue_peek_session(1, &r)) {
		CHECK(r.event_id == expect, "S13: record order preserved (id == position)");
		CHECK(r.session_id == 1U, "S13: record still stamped with session 1");
		n++;
		expect++;
		fw112_diag_queue_release_session(1);
	}
	CHECK(n == CAPACITY_REQUIRED, "S13: all 24 records survived");
	CHECK(expect == CAPACITY_REQUIRED, "S13: ids 0..23 exactly once");
}

/* ------- S14 : the realistic >8-event sequence that triggered FW-112-DIAG.1 --------------------- */

static void test_realistic_sequence(void)
{
	reset();
	baseline();
	fw112_diag_set_session_id(1);

	/*
	 * The previous hardware capture filled all 8 slots exactly at RECOVERY_ENTER, so the tail
	 * (WAIT/TRACK/COLLAPSE, re-grant, final demand) never made it into the log. Drive that whole
	 * chain in one session: COLD BLOCKED -> cold GRANTED -> HOLD_ARMED -> ZEROED -> REVOKED ->
	 * BLOCKED -> fast-rearm GRANTED -> RECOVERY_ENTER -> COLLAPSE -> EXIT -> ZEROED.
	 */
	static const uint8_t expect_types[] = {
		FW112_EVT_BLOCKED,             /* 0 */
		FW112_EVT_PERMISSION_GRANTED,  /* 1  cold arm */
		FW112_EVT_HOLD_ARMED,          /* 2 */
		FW112_EVT_ZEROED,              /* 3 */
		FW112_EVT_PERMISSION_REVOKED,  /* 4 */
		FW112_EVT_BLOCKED,             /* 5  new stretch */
		FW112_EVT_PERMISSION_GRANTED,  /* 6  fast rearm */
		FW112_EVT_RECOVERY_ENTER,      /* 7 */
		FW112_EVT_RECOVERY_COLLAPSE,   /* 8 */
		FW112_EVT_RECOVERY_EXIT,       /* 9 */
		FW112_EVT_ZEROED               /* 10 */
	};
	const uint32_t n = (uint32_t)(sizeof(expect_types) / sizeof(expect_types[0]));
	CHECK(n > 8U, "S14: sequence is longer than the old 8-record capacity");

	/* 1: COLD, pedalling, no latch -> BLOCKED (first stretch). */
	in.session_state = ST_COLD;
	in.latched = false;
	in.fwd_run = 1;
	in.iq_setpoint = 0;
	step();
	/* 2: same stretch, must not re-fire. */
	step();
	/* 3: cold start -> GRANTED (COLD_ARM). */
	in.session_state = ST_ACTIVE;
	in.latched = true;
	in.fwd_run = 6;
	in.iq_setpoint = 400;
	step();
	/* 4: hold armed by a positive mode demand. */
	in.assist_hold_ticks = 40;
	step();
	/* 5: permission present but nothing reaches the motor -> ZEROED. */
	in.iq_setpoint = 0;
	in.iq_request = 0;
	step();
	/* 6: reverse -> REVOKED (hold expires off-latch: no HOLD_EXPIRED edge). */
	in.session_state = ST_SUSPENDED;
	in.latched = false;
	in.fwd_run = 0;
	in.iq_setpoint = 0;
	in.assist_hold_ticks = 0;
	step();
	/* 7: still suspended, pedalling again -> BLOCKED (new stretch). */
	in.fwd_run = 1;
	step();
	/* 8: fast rearm -> GRANTED (FAST_REARM). */
	in.session_state = ST_ACTIVE;
	in.latched = true;
	in.fwd_run = 6;
	in.iq_setpoint = 400;
	step();
	/* 9: recovery opened -> RECOVERY_ENTER. */
	in.recovery_state = REC_WAIT;
	step();
	/* 10: WAIT -> TRACK (no event). */
	in.recovery_state = REC_TRACK;
	step();
	/* 11: TRACK -> WAIT: pressure lost mid-recovery -> RECOVERY_COLLAPSE. */
	in.recovery_state = REC_WAIT;
	step();
	/* 12: recovery finished -> RECOVERY_EXIT. */
	in.recovery_state = REC_IDLE;
	step();
	/* 13: permission present, iq 0 again -> ZEROED (new stretch). */
	in.iq_setpoint = 0;
	in.iq_request = 0;
	step();

	CHECK(fw112_diag_queue_count_session(1) == n, "S14: the whole sequence was recorded");

	fw112_diag_record_t r;
	uint32_t i = 0;
	while (fw112_diag_queue_peek_session(1, &r)) {
		char lab[96];
		if (i < n) {
			snprintf(lab, sizeof(lab), "S14: event %lu is %u (expected %u)",
			         (unsigned long)i, (unsigned)r.event_type, (unsigned)expect_types[i]);
			host_test_check(r.event_type == expect_types[i], lab);
			if (i == 1U) {
				host_test_check((r.flags & FW112_FLAG_COLD_ARM) != 0U,
				                "S14: first GRANTED is a cold arm");
			}
			if (i == 6U) {
				host_test_check((r.flags & FW112_FLAG_FAST_REARM) != 0U,
				                "S14: second GRANTED is a fast rearm");
			}
		} else {
			host_test_check(0, "S14: no event beyond the expected sequence");
		}
		i++;
		fw112_diag_queue_release_session(1);
	}
	CHECK(i == n, "S14: exactly the expected number of records present");
}

/* ------- S15 (FW-112.2): the suspension-reason flags are echoed into the record ------------------ */

static void test_wheel_coast_flags(void)
{
	reset();
	baseline();
	step_n(3);                                 /* steady ACTIVE so prev_session_state is ACTIVE */

	/* A rolling-coast suspension: the rider stopped pedalling (real_stop) while the wheel is
	 * still fresh (wheel_valid). The REVOKED record must carry both new flag bits, so a reader
	 * distinguishes SUSPEND_REASON_ROLLING_COAST (this case) from SUSPEND_REASON_DIRECTION
	 * (reason_bits DIRECTION, no ROLLING_COAST) without any layout change. */
	in.wheel_valid = true;
	in.rolling_coast = true;
	in.session_state = ST_SUSPENDED;
	in.latched = false;
	in.fwd_run = 0;
	in.iq_setpoint = 0;
	step();                                    /* REVOKED */
	fw112_diag_record_t r;
	CHECK(fw112_diag_queue_count_session(1) == 1U, "S15: REVOKED recorded on the coast suspension");
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_type == FW112_EVT_PERMISSION_REVOKED,
	      "S15: event is REVOKED");
	CHECK((r.flags & FW112_FLAG_WHEEL_VALID) != 0U, "S15: WHEEL_VALID echoed");
	CHECK((r.flags & FW112_FLAG_ROLLING_COAST) != 0U, "S15: ROLLING_COAST echoed (SUSPEND_REASON_ROLLING_COAST)");
	CHECK((r.flags & FW112_FLAG_LATCHED) == 0U, "S15: not latched during the coast");
	drain_queue(1);

	/* A direction-hold suspension echoes WHEEL_VALID but NOT ROLLING_COAST. */
	reset();
	baseline();
	step_n(3);                                 /* steady ACTIVE */
	in.wheel_valid = true;
	in.rolling_coast = false;                  /* real_stop not holding: a reverse hold */
	in.reason_bits = FW112_REASON_DIRECTION;
	in.session_state = ST_SUSPENDED;
	in.latched = false;
	in.fwd_run = 0;
	in.iq_setpoint = 0;
	step();                                    /* REVOKED */
	CHECK(fw112_diag_queue_count_session(1) == 1U, "S15: REVOKED recorded on the direction suspension");
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_type == FW112_EVT_PERMISSION_REVOKED,
	      "S15: event is REVOKED");
	CHECK((r.flags & FW112_FLAG_WHEEL_VALID) != 0U, "S15: WHEEL_VALID echoed");
	CHECK((r.flags & FW112_FLAG_ROLLING_COAST) == 0U,
	      "S15: ROLLING_COAST clear -> a reader names SUSPEND_REASON_DIRECTION via reason_bits");
	CHECK((r.reason_bits & FW112_REASON_DIRECTION) != 0U, "S15: direction reason present");
	drain_queue(1);

	/* A cold terminal record has neither new flag set (both facts false at a true stop). */
	reset();
	baseline();
	step_n(3);                                 /* steady ACTIVE */
	in.wheel_valid = false;
	in.rolling_coast = false;
	in.session_state = ST_COLD;
	in.latched = false;
	in.fwd_run = 0;
	in.iq_setpoint = 0;
	step();                                    /* REVOKED (terminal -> COLD) */
	CHECK(fw112_diag_queue_peek_session(1, &r) && r.event_type == FW112_EVT_PERMISSION_REVOKED,
	      "S15: event is REVOKED");
	CHECK((r.flags & FW112_FLAG_WHEEL_VALID) == 0U, "S15: WHEEL_VALID clear at a true stop");
	CHECK((r.flags & FW112_FLAG_ROLLING_COAST) == 0U, "S15: ROLLING_COAST clear at a true stop");
	drain_queue(1);
}

int main(void)
{
	printf("FW-112-DIAG fw112_diag.c (whole-chain event recorder), against the shipped module\n");

	test_session_edges_and_blocked();   /* S1 S2 S3 S4  (also M1, M2, M3) */
	test_recovery_edges();              /* S5            (also M4) */
	test_hold_edges();                  /* S6            (also M4) */
	test_zeroed();                      /* S7            (also M1, M3) */
	test_event_id_and_size();           /* S8            (also M5) */
	test_session_separation_and_ring(); /* S9 */
	test_queue_full_refused();          /* S10 */
	test_capacity_reject_preserve();    /* S11 S12 S13 */
	test_realistic_sequence();          /* S14 */
	test_wheel_coast_flags();           /* S15 (FW-112.2 suspension-reason flags) */

	if (host_test_failures != 0) {
		printf("fw112_diag_host: %d FAILURES\n", host_test_failures);
		return 1;
	}
	printf("fw112_diag_host: ALL PASS\n");
	return 0;
}
