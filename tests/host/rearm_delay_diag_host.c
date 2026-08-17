/*
 * FW-111: host tests for src/rearm_delay_diag.c - the delayed-rearm diagnostic recorder,
 * compiled and run on THIS machine (the REAL module, driven with a synthetic session history).
 *
 * The module is deliberately not linked against ride_session.c/ride_control.c; its whole input
 * surface is rearm_delay_input_t, so a harness can exercise the FSM, the keep/discard decision,
 * the guaranteed snapshot set, the timing block and the queue without stubbing anything except
 * the torque snapshot it feeds in. Session state is fed as the same plain byte main.c would pass
 * (0 COLD, 1 ACTIVE, 2 SUSPENDED_BY_DIRECTION; 3 = the legacy reserved WAIT_REARM_LOAD value,
 * never entered by production since FW-112 v2).
 *
 * v5 (FW-112 v2) semantics these tests pin down - everything below is against the v5 record:
 *   - the record ANCHORS on the PREARM edge (ACTIVE -> SUSPENDED_BY_DIRECTION, the real
 *     initiating event) and milestone 1 is ENTER_SUSPEND - there is NO WAIT_REARM_LOAD stage,
 *     so the old "record opens on WAIT entry" scripts are gone; a record opens the tick the
 *     session state goes ACTIVE -> SUSPENDED and the FSM is IDLE -> SUSPENDED (1);
 *   - milestone 6 is PERMISSION (the SUSPENDED_BY_DIRECTION -> ACTIVE rearm edge, the exact tick
 *     out->fast_rearm_this_tick fires) - the old two-phase COMMIT stage does not exist; the
 *     timing field is t_permission;
 *   - the snapshots are the GUARANTEED set ENTER_SUSPEND / PROBLEM / PERMISSION / CLOSE; DEMAND
 *     and PWM_ON are TIMINGS, never snapshots, so the 4-slot budget can no longer be eaten by
 *     the filter chain before the C..F stages are seen;
 *   - every stage's elapsed time (t_pressure .. t_close) is present in the record even without a
 *     snapshot; 0xFFFF (REARM_DELAY_T_UNREACHED) means never;
 *   - the baseline is the demand of the LAST ACTIVE tick, not the ACTIVE maximum;
 *   - the keep reasons: WAIT_LONG (the SUSPENDED hold > 200 ms), NO_PERMISSION (the saga ended
 *     without the rearm edge ever firing - the terminal / timeout close), WEAK_TARGET (below
 *     80 % of the pre-reverse Iq for > 150 ms continuously, detected while RECOVERING), NO_LOAD
 *     (the rearm DID grant but no pedal load appeared for REARM_DELAY_NO_LOAD_MS - also
 *     RECOVERING); a record with reason_bits == 0 is discarded at close;
 *   - the Hall start is measured by t_standstill_enter / t_standstill_exit markers, not a flag;
 *   - 32-bit control_now WRAPPAROUND (a ride can straddle it) must not break WAIT_LONG or the
 *     continuous WEAK_TARGET window;
 *   - the TRACE/RAW reservation EDGES (prearm_edge/ownership_end_edge, derived from session_state
 *     PLUS the record FSM's own transitions - NOT tied to the PERMISSION tick alone, since a
 *     WEAK_TARGET can be detected up to 150 ms after permission) and the PROBLEM trigger's
 *     one-shot latch/consume cycle via note_reserve_done(), which SURVIVES a same-tick record
 *     close;
 *   - v2: a re-suspend (an extra reverse while SUSPENDED) does NOT close the record and does NOT
 *     re-fire prearm or end ownership - one record spans the whole multi-reverse saga (the v1
 *     WAIT<->SUSPENDED oscillation is gone);
 *   - v5.1: note_reserve_done() resolves its target by an INTERNAL 32-bit record_uid, never by
 *     the wire (record_id, session_id) pair (which collides after a 256-rearm record_seq wrap)
 *     and never by a physical slots[] index (which moves on a ring shift): the 256-record wire
 *     collision, the UID sidecar moving with a physically shifted record, the uint32_t UID wrap
 *     with skip-in-use (via the REARM_UID_SEAM_TEST seam), and the no-match-writes-nothing case;
 *   - the queue is a fixed ring; a capture with both slots occupied is refused and counted.
 */

#include "../common/check.h"

#include "rearm_delay_diag.h"

#include <string.h>

/* Session state bytes - the numeric encoding the header documents. */
static const uint8_t ST_COLD = 0U;
static const uint8_t ST_ACTIVE = 1U;
static const uint8_t ST_SUSPENDED = 2U;

/* CONTROL_TIMEBASE_HZ = 4000 -> 4 ticks per ms. */
static const uint32_t T_MS = CONTROL_TIMEBASE_HZ / 1000U;

static uint32_t tick;
static rearm_delay_input_t in;
static torque_snapshot_t snap;

static void reset(void)
{
	tick = 0;
	memset(&snap, 0, sizeof(snap));
	memset(&in, 0, sizeof(in));
	in.run_deadband = 5;
	in.snapshot = &snap;
	rearm_delay_init();
	rearm_delay_set_session_id(1);
}

/* One control tick with the given session state (everything else is whatever `in` holds now). */
static void step(uint8_t state)
{
	in.session_state = state;
	rearm_delay_tick(&in, tick);
	tick++;
}

static void step_n(uint8_t state, uint32_t n)
{
	for (uint32_t i = 0; i < n; i++) step(state);
}

/* A rider actively pedalling before the reverse, so the baseline gets a last-active value. */
static void active_baseline(int32_t iq_pre_ramp)
{
	in.iq_pre_ramp = (int16_t)iq_pre_ramp;
	step_n(ST_ACTIVE, 5);
}

/* WAIT_LONG keep: > 200 ms stuck SUSPENDED (the record anchors at the PREARM edge and opens the
 * tick ACTIVE goes SUSPENDED), then a terminal stop. The saga never gets permission. */
static void ride_wait_long_keep(void)
{
	active_baseline(100);
	step(ST_SUSPENDED);                             /* PREARM edge: record opens, ENTER_SUSPEND */
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);         /* > 200 ms -> WAIT_LONG */
	step(ST_COLD);                                  /* terminal: NO_PERMISSION, record kept */
}

/* ============================================================================================ */

static void test_healthy_fast_rearm_discarded(void)
{
	reset();
	active_baseline(100);
	CHECK(!rearm_delay_prearm_edge(), "fast rearm: no prearm edge before the reverse");
	step(ST_SUSPENDED);
	/* PREARM is the ACTIVE -> SUSPENDED_BY_DIRECTION edge itself and the v2 record anchor, so it
	 * must fire here even for what turns out to be a healthy fast rearm, and must not still be
	 * true a tick later (one-shot, not sticky). The record opens on this very tick. */
	CHECK(rearm_delay_prearm_edge(), "fast rearm: prearm edge fires on ACTIVE -> SUSPENDED");
	CHECK(!rearm_delay_ownership_end_edge(), "fast rearm: no ownership-end edge on the prearm tick");
	CHECK(rearm_delay_fsm_state() == 1, "fast rearm: record open, FSM in SUSPENDED");
	step(ST_SUSPENDED);
	CHECK(!rearm_delay_prearm_edge(), "fast rearm: prearm edge is one-shot, gone the next tick");
	/* normal rearm: the chain stages appear on later ticks (timings only, no snapshots) */
	snap.assist_delta_native = 50;
	snap.assist_delta_filtered_native = 6;
	snap.assist_delta_run_native = 10;
	in.iq_request = 100;
	in.pwm_on = true;
	step(ST_SUSPENDED);
	step(ST_ACTIVE);                                 /* PERMISSION: SUSPENDED -> ACTIVE (the rearm) */
	/* v5 (fixes the "released too early" bug): PERMISSION alone must NOT end ownership - the
	 * record only moves SUSPENDED -> RECOVERING, and a WEAK_TARGET can still be detected up to
	 * 150 ms later while the reservation is needed. */
	CHECK(!rearm_delay_ownership_end_edge(), "fast rearm: no ownership-end edge on the permission tick itself");
	CHECK(rearm_delay_fsm_state() == 2, "fast rearm: FSM moved to RECOVERING, not IDLE, on permission");
	in.iq_pre_ramp = 100;                            /* target immediately recovered */
	in.iq_setpoint = 100;
	step_n(ST_ACTIVE, 3);
	CHECK(!rearm_delay_ownership_end_edge(), "fast rearm: still no ownership-end edge while RECOVERING");
	in.real_stop = true;
	step(ST_COLD);                                   /* recovery concludes: RECOVERING closes */
	/* Ownership ends exactly when the record actually closes - here, RECOVERING leaving ACTIVE. */
	CHECK(rearm_delay_ownership_end_edge(), "fast rearm: ownership-end edge fires when RECOVERING closes");

	CHECK(rearm_delay_queue_count_session(1) == 0, "fast rearm: nothing queued");
	CHECK(rearm_delay_queue_enqueued() == 0, "fast rearm: not counted as accepted");
	CHECK(rearm_delay_queue_rejected() == 0, "fast rearm: not counted as rejected");
	CHECK(rearm_delay_fsm_state() == 0, "fast rearm: FSM back to idle");
}

static void test_wait_long_kept(void)
{
	reset();
	ride_wait_long_keep();

	CHECK(rearm_delay_queue_count_session(1) == 1, "WAIT_LONG: one record queued");
	CHECK(rearm_delay_queue_enqueued() == 1, "WAIT_LONG: counted as accepted");
	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(1, &r), "WAIT_LONG: peek succeeds");
	CHECK((r.reason_bits & REARM_DELAY_REASON_WAIT_LONG) != 0, "WAIT_LONG: reason bit set");
	CHECK(r.session_id == 1, "WAIT_LONG: session id stamped");
	CHECK(r.pre_reverse_iq == 100, "WAIT_LONG: pre_reverse_iq baseline captured");
	CHECK(r.snapshot_count == 3, "WAIT_LONG: ENTER_SUSPEND + PROBLEM + CLOSE snapshots");
	CHECK(r.snapshots[0].milestone_id == REARM_MILESTONE_ENTER_SUSPEND, "WAIT_LONG: slot 0 ENTER_SUSPEND");
	CHECK(r.snapshots[1].milestone_id == REARM_MILESTONE_PROBLEM, "WAIT_LONG: slot 1 PROBLEM");
	CHECK(r.snapshots[2].milestone_id == REARM_MILESTONE_RECORD_CLOSE, "WAIT_LONG: slot 2 RECORD_CLOSE");
	CHECK(r.t_permission == REARM_DELAY_T_UNREACHED, "WAIT_LONG: never got permission -> t_permission unreached");
	CHECK((r.reason_bits & REARM_DELAY_REASON_NO_PERMISSION) != 0,
		"WAIT_LONG: terminal stop without permission -> NO_PERMISSION also set");
	CHECK(r.t_close != REARM_DELAY_T_UNREACHED, "WAIT_LONG: t_close set");
	CHECK(r.t_close >= REARM_DELAY_WAIT_LONG_TICKS, "WAIT_LONG: t_close after the long wait");
}

static void test_no_permission_kept(void)
{
	reset();
	active_baseline(100);
	step(ST_SUSPENDED);                              /* PREARM edge: record opens */
	step(ST_SUSPENDED);                              /* short wait, never gets permission */
	step(ST_COLD);                                   /* terminal stop */

	CHECK(rearm_delay_queue_count_session(1) == 1, "NO_PERMISSION: one record queued");
	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(1, &r), "NO_PERMISSION: peek succeeds");
	CHECK((r.reason_bits & REARM_DELAY_REASON_NO_PERMISSION) != 0, "NO_PERMISSION: reason bit set");
	CHECK((r.reason_bits & REARM_DELAY_REASON_WAIT_LONG) == 0, "NO_PERMISSION: no WAIT_LONG for a short wait");
	CHECK(r.snapshot_count == 3, "NO_PERMISSION: ENTER_SUSPEND + PROBLEM(terminal) + CLOSE");
}

static void test_no_load_kept(void)
{
	reset();
	active_baseline(100);
	step(ST_SUSPENDED);                              /* PREARM edge: record opens */
	step(ST_SUSPENDED);
	step(ST_ACTIVE);                                 /* PERMISSION granted -> RECOVERING */
	step_n(ST_ACTIVE, T_MS * 2100U);                 /* > NO_LOAD_MS with zero load */
	step(ST_COLD);                                   /* terminal closes and keeps the record */

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_count_session(1) == 1, "NO_LOAD: one record queued");
	CHECK(rearm_delay_queue_peek_session(1, &r), "NO_LOAD: peek succeeds");
	CHECK((r.reason_bits & REARM_DELAY_REASON_NO_LOAD) != 0, "NO_LOAD: reason bit set");
	CHECK((r.reason_bits & REARM_DELAY_REASON_WAIT_LONG) == 0,
		"NO_LOAD: the hold was short - WAIT_LONG is NOT set (v5: NO_LOAD is a RECOVERING reason)");
	CHECK((r.reason_bits & REARM_DELAY_REASON_NO_PERMISSION) == 0,
		"NO_LOAD: permission WAS granted - NO_PERMISSION is NOT set");
	uint8_t n_problem = 0;
	for (uint8_t i = 0; i < r.snapshot_count; i++)
		if (r.snapshots[i].milestone_id == REARM_MILESTONE_PROBLEM) n_problem++;
	CHECK(n_problem == 1, "NO_LOAD: exactly one problem snapshot");
}

static void test_weak_target_kept(void)
{
	reset();
	active_baseline(100);                            /* baseline = 100 -> 80 % benchmark */
	step(ST_SUSPENDED);                              /* PREARM edge: record opens */
	step(ST_SUSPENDED);
	step(ST_ACTIVE);                                 /* PERMISSION -> RECOVERING */
	in.iq_pre_ramp = 50;                             /* below for > 150 ms CONTINUOUSLY */
	step_n(ST_ACTIVE, T_MS * 200U + 1U);
	step(ST_COLD);

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_count_session(1) == 1, "WEAK_TARGET: one record queued");
	CHECK(rearm_delay_queue_peek_session(1, &r), "WEAK_TARGET: peek succeeds");
	CHECK((r.reason_bits & REARM_DELAY_REASON_WEAK_TARGET) != 0, "WEAK_TARGET: reason bit set");
	CHECK(r.t_weak_start != REARM_DELAY_T_UNREACHED, "WEAK_TARGET: t_weak_start recorded");
}

static void test_brief_dip_not_weak(void)
{
	reset();
	active_baseline(100);
	step(ST_SUSPENDED);                              /* PREARM edge: record opens */
	step(ST_SUSPENDED);
	step(ST_ACTIVE);                                 /* PERMISSION -> RECOVERING */
	in.iq_pre_ramp = 50;                             /* below for just 20 ms... */
	step_n(ST_ACTIVE, T_MS * 20U);
	in.iq_pre_ramp = 100;                            /* ...then recovered for 200 ms */
	step_n(ST_ACTIVE, T_MS * 200U);
	step(ST_COLD);

	/* The whole rearm recovered and nothing suspicious fired - the record is DISCARDED like any
	 * healthy fast rearm. Had the 20 ms dip been latched as WEAK_TARGET it would have been kept. */
	CHECK(rearm_delay_queue_count_session(1) == 0, "brief dip: record discarded (healthy, not WEAK_TARGET)");
	CHECK(rearm_delay_queue_rejected() == 0, "brief dip: nothing refused");
}

static void test_baseline_is_last_active_not_max(void)
{
	reset();
	in.iq_pre_ramp = 100;                            /* a strong push earlier in the ride... */
	step_n(ST_ACTIVE, 3);
	in.iq_pre_ramp = 30;                             /* ...then the last tick before the reverse */
	step(ST_ACTIVE);
	step(ST_SUSPENDED);                              /* PREARM edge: record opens */
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);          /* WAIT_LONG keeps the record */
	step(ST_COLD);

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(1, &r), "baseline: peek succeeds");
	CHECK(r.pre_reverse_iq == 30, "baseline: LAST active tick (30), not the ACTIVE max (100)");
}

static void test_guaranteed_snapshot_set_and_timings(void)
{
	reset();
	active_baseline(100);
	snap.assist_delta_run_native = 10;               /* pre-reverse RUN held at the prearm snapshot */
	step(ST_SUSPENDED);                              /* PREARM edge: ENTER_SUSPEND, the anchor */
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);          /* WAIT_LONG -> PROBLEM snapshot */
	/* the whole filter chain appears while still suspended - TIMINGS, not snapshots */
	snap.assist_delta_native = 50;
	snap.assist_delta_filtered_native = 6;
	snap.assist_delta_run_native = 10;
	in.iq_request = 100;
	in.pwm_on = true;
	step(ST_SUSPENDED);
	step(ST_ACTIVE);                                 /* PERMISSION */
	in.iq_pre_ramp = 100;                            /* recovered to the baseline */
	in.iq_setpoint = 100;
	step_n(ST_ACTIVE, 3);
	step(ST_COLD);                                   /* CLOSE */

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(1, &r), "guaranteed set: peek succeeds");
	CHECK(r.snapshot_count == REARM_DELAY_SNAPSHOTS, "guaranteed set: exactly 4 snapshots");
	CHECK(r.snapshots[0].milestone_id == REARM_MILESTONE_ENTER_SUSPEND, "guaranteed set: ENTER_SUSPEND");
	CHECK(r.snapshots[1].milestone_id == REARM_MILESTONE_PROBLEM, "guaranteed set: PROBLEM");
	CHECK(r.snapshots[2].milestone_id == REARM_MILESTONE_PERMISSION, "guaranteed set: PERMISSION");
	CHECK(r.snapshots[3].milestone_id == REARM_MILESTONE_RECORD_CLOSE, "guaranteed set: CLOSE");
	/* the filter chain survived as timings even though none of them got a snapshot */
	CHECK(r.t_pressure != REARM_DELAY_T_UNREACHED, "timings: t_pressure");
	CHECK(r.t_filter_ready != REARM_DELAY_T_UNREACHED, "timings: t_filter_ready");
	CHECK(r.t_run_ready != REARM_DELAY_T_UNREACHED, "timings: t_run_ready");
	CHECK(r.t_demand != REARM_DELAY_T_UNREACHED, "timings: t_demand");
	CHECK(r.t_permission != REARM_DELAY_T_UNREACHED, "timings: t_permission");
	CHECK(r.t_pwm_on != REARM_DELAY_T_UNREACHED, "timings: t_pwm_on");
	CHECK(r.t_target_recovered != REARM_DELAY_T_UNREACHED, "timings: t_target_recovered");
	CHECK(r.t_setpoint_recovered != REARM_DELAY_T_UNREACHED, "timings: t_setpoint_recovered");
	CHECK(r.t_close != REARM_DELAY_T_UNREACHED, "timings: t_close");
	/* the order is guaranteed: PROBLEM before PERMISSION before CLOSE */
	CHECK(r.t_permission >= REARM_DELAY_WAIT_LONG_TICKS, "timings: permission after the long wait");
	CHECK(r.t_close >= r.t_permission, "timings: close after permission");
}

static void test_timings_are_one_shot(void)
{
	reset();
	active_baseline(100);
	step(ST_SUSPENDED);                              /* PREARM edge: anchor at this tick */
	snap.assist_delta_native = 50;                   /* PRESSURE fires on the NEXT tick */
	step(ST_SUSPENDED);                              /* elapsed == 1 -> t_pressure == 1 */
	snap.assist_delta_native = 50;                   /* still present for many more ticks... */
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);          /* WAIT_LONG keeps the record */
	step(ST_COLD);

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(1, &r), "one-shot: peek succeeds");
	CHECK(r.t_pressure == 1, "one-shot: t_pressure is the FIRST occurrence's elapsed (1), not the last");
	CHECK(r.snapshot_count == 3, "one-shot: ENTER_SUSPEND + PROBLEM + CLOSE only");
}

static void test_snapshot_cap_no_stage_evicted(void)
{
	reset();
	active_baseline(100);
	step(ST_SUSPENDED);                              /* ENTER_SUSPEND */
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);          /* PROBLEM (WAIT_LONG) */
	in.iq_request = 100;                             /* DEMAND + PWM_ON: TIMINGS, no snapshot */
	in.pwm_on = true;
	step(ST_SUSPENDED);
	step(ST_ACTIVE);                                 /* PERMISSION */
	in.iq_pre_ramp = 50;                             /* below 80 % -> WEAK fires later */
	step_n(ST_ACTIVE, T_MS * 200U + 1U);             /* WEAK (PROBLEM already taken -> no 5th slot) */
	in.iq_pre_ramp = 100;
	step(ST_ACTIVE);
	step(ST_COLD);                                   /* CLOSE */

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(1, &r), "cap: peek succeeds");
	CHECK(r.snapshot_count == REARM_DELAY_SNAPSHOTS, "cap: exactly 4 snapshots");
	CHECK(r.snapshots[3].milestone_id == REARM_MILESTONE_RECORD_CLOSE, "cap: CLOSE is the 4th slot");
	uint8_t have_demand_snap = 0, have_pwm_snap = 0, n_problem = 0;
	for (uint8_t i = 0; i < r.snapshot_count; i++) {
		if (r.snapshots[i].milestone_id == REARM_MILESTONE_DEMAND) have_demand_snap = 1;
		if (r.snapshots[i].milestone_id == REARM_MILESTONE_PWM_ON) have_pwm_snap = 1;
		if (r.snapshots[i].milestone_id == REARM_MILESTONE_PROBLEM) n_problem++;
	}
	CHECK(!have_demand_snap, "cap: DEMAND is a timing, never a snapshot");
	CHECK(!have_pwm_snap, "cap: PWM_ON is a timing, never a snapshot");
	CHECK(n_problem == 1, "cap: WEAK did not add a second PROBLEM snapshot");
	CHECK(r.t_demand != REARM_DELAY_T_UNREACHED, "cap: DEMAND timing still recorded");
	CHECK(r.t_pwm_on != REARM_DELAY_T_UNREACHED, "cap: PWM_ON timing still recorded");
	CHECK((r.reason_bits & REARM_DELAY_REASON_WEAK_TARGET) != 0, "cap: WEAK_TARGET still set as a reason");
}

static void test_standstill_timings(void)
{
	reset();
	active_baseline(100);
	step(ST_SUSPENDED);                              /* PREARM edge: record opens */
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);          /* WAIT_LONG keeps the record */
	rearm_delay_note_standstill_enter(tick);         /* the 25 ms Hall hold: enter */
	step(ST_ACTIVE);                                 /* PERMISSION */
	rearm_delay_note_standstill_exit(tick);          /* ... and exit (both OUTSIDE the delay) */
	step_n(ST_ACTIVE, 3);
	step(ST_COLD);

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(1, &r), "standstill: peek succeeds");
	CHECK(r.t_standstill_enter != REARM_DELAY_T_UNREACHED, "standstill: enter marker set");
	CHECK(r.t_standstill_exit != REARM_DELAY_T_UNREACHED, "standstill: exit marker set");
	CHECK(r.t_standstill_exit >= r.t_standstill_enter, "standstill: exit not before enter");
	CHECK(r.t_standstill_enter >= REARM_DELAY_WAIT_LONG_TICKS, "standstill: the hold follows the long wait");
	CHECK((uint32_t)(r.t_standstill_exit - r.t_standstill_enter) < T_MS * 2000U,
		"standstill: hold length is sane (25 ms, generous bound)");
}

/*
 * Bug 2 regression: the SUSPENDED branch used to handle the SUSPENDED->ACTIVE (PERMISSION) tick
 * FIRST and break out before PRESSURE / FILTER_READY / RUN_READY / DEMAND were ever sampled. A
 * stage that first appeared EXACTLY on the permission tick was therefore lost as 0xFFFF. v3
 * samples every input-chain stage on the current tick BEFORE handling the state change, so a
 * permission tick that also sees all four stages (plus recovery + PWM) must record all of them
 * equal to t_permission - never 0xFFFF, never t_permission+1.
 */
static void test_bug2_permission_tick_records_all_stages(void)
{
	reset();
	active_baseline(100);
	snap.assist_delta_run_native = 100;              /* pre-reverse RUN held at the prearm snapshot */
	step(ST_SUSPENDED);                              /* PREARM edge: ENTER_SUSPEND captures run=100 */
	snap.assist_delta_run_native = 0;                /* the hold: no run signal */
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);          /* > 200 ms -> WAIT_LONG keeps the record */

	/* the permission tick: every stage appears for the FIRST time on the very tick that flips
	 * SUSPENDED -> ACTIVE. Also satisfy the recovery benchmark so t_target_recovered /
	 * t_setpoint_recovered land on the same tick. */
	snap.assist_delta_native = 50;                   /* PRESSURE */
	snap.assist_delta_filtered_native = 6;           /* FILTER_READY >= run_deadband (5) */
	snap.assist_delta_run_native = 100;              /* RUN_READY + TARGET_RECOVERED (pre=100) */
	in.iq_request = 100;                             /* DEMAND */
	in.pwm_on = true;                                /* PWM_ON */
	in.iq_pre_ramp = 100;                            /* >= 80 % of pre_reverse_iq (100) */
	in.iq_setpoint = 100;                            /* >= 80 % */
	step(ST_ACTIVE);                                 /* PERMISSION + all stages on this one tick */
	step_n(ST_ACTIVE, 3);
	step(ST_COLD);

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(1, &r), "bug2: peek succeeds");
	CHECK(r.t_permission != REARM_DELAY_T_UNREACHED, "bug2: t_permission set");
	CHECK(r.t_pressure == r.t_permission, "bug2: t_pressure recorded on the PERMISSION tick");
	CHECK(r.t_filter_ready == r.t_permission, "bug2: t_filter_ready recorded on the PERMISSION tick");
	CHECK(r.t_run_ready == r.t_permission, "bug2: t_run_ready recorded on the PERMISSION tick");
	CHECK(r.t_demand == r.t_permission, "bug2: t_demand recorded on the PERMISSION tick");
	CHECK(r.t_target_recovered == r.t_permission, "bug2: t_target_recovered recorded on the PERMISSION tick");
	CHECK(r.t_setpoint_recovered == r.t_permission, "bug2: t_setpoint_recovered recorded on the PERMISSION tick");
	CHECK(r.t_pwm_on == r.t_permission, "bug2: t_pwm_on recorded on the PERMISSION tick");
}

/*
 * Bug 1 regression at the RECORD level: the two standstill markers must be stampable with
 * DISTINCT ticks so the hold measures ~25 ms at 4 kHz (T_MS * 25 = 100 ticks). A caller that
 * passes the same now_tick to both (the old main.c bug) measures 0. This test pins the contract
 * the module's side: enter at one tick, exit ~100 ticks later -> span is exactly 100, and the
 * ~25 ms expectation holds.
 */
static void test_bug1_standstill_span_is_real_time(void)
{
	reset();
	active_baseline(100);
	step(ST_SUSPENDED);                              /* PREARM edge: record opens */
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);          /* WAIT_LONG keeps the record */
	uint32_t enter_tick = tick;
	rearm_delay_note_standstill_enter(enter_tick);   /* just before the 25 ms blocking delay */
	step_n(ST_SUSPENDED, T_MS * 25U);                /* the 25 ms hold passes (no ticks inside) */
	rearm_delay_note_standstill_exit(tick);          /* just after - the fresh read */
	step(ST_COLD);

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(1, &r), "bug1: peek succeeds");
	CHECK(r.t_standstill_enter != REARM_DELAY_T_UNREACHED, "bug1: enter set");
	CHECK(r.t_standstill_exit != REARM_DELAY_T_UNREACHED, "bug1: exit set");
	uint32_t span = (uint32_t)(r.t_standstill_exit - r.t_standstill_enter);
	CHECK(span == T_MS * 25U, "bug1: the 25 ms Hall hold measures exactly ~100 ticks");
	CHECK(span > 0U, "bug1: the hold is never 0 (the old same-tick bug)");
}

static void test_queue_full_refused(void)
{
	reset();
	ride_wait_long_keep();                            /* slot 0 queued */
	ride_wait_long_keep();                            /* slot 1 queued */
	CHECK(rearm_delay_queue_count_session(1) == 2, "full queue: two records queued");

	/* a third delayed rearm while both slots are occupied -> refused, never evicts */
	active_baseline(100);
	step(ST_SUSPENDED);
	CHECK(rearm_delay_prearm_edge(), "full queue: prearm edge still fires - pas_trace reservation is independent of the record queue");
	CHECK(rearm_delay_fsm_state() == 0, "full queue: FSM stays IDLE - no record ever tracks this saga");
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);
	CHECK(rearm_delay_fsm_state() == 0, "full queue: still IDLE deep into the wait - nothing to close");
	step(ST_COLD);
	/* v5 requirement 7: a saga whose record was refused (queue full) must still not leave the
	 * pas_trace reservation orphaned - ownership ends via the fallback path (no record was ever
	 * tracking it), not via a record closing (there is no record). */
	CHECK(rearm_delay_ownership_end_edge(),
		"full queue: ownership-end edge still fires via the no-record fallback - not orphaned");

	CHECK(rearm_delay_queue_rejected() == 1, "full queue: capture refused once");
	CHECK(rearm_delay_queue_count_session(1) == 2, "full queue: older records untouched");
	CHECK(rearm_delay_queue_enqueued() == 2, "full queue: accepted total still 2");
}

static void test_release_and_ring(void)
{
	reset();
	ride_wait_long_keep();                            /* record_id 0 */
	ride_wait_long_keep();                            /* record_id 1 */

	rearm_delay_record_t r0, r1;
	CHECK(rearm_delay_queue_peek_session(1, &r0), "ring: first peek");
	CHECK(rearm_delay_queue_peek_session(1, &r1), "ring: second peek is still the same oldest");
	CHECK(r0.record_id == r1.record_id, "ring: peek does not consume");
	CHECK(r0.record_id == 0, "ring: oldest record has record_id 0");

	rearm_delay_queue_release_session(1);
	CHECK(rearm_delay_queue_count_session(1) == 1, "ring: after release, one left");
	CHECK(rearm_delay_queue_peek_session(1, &r0) && r0.record_id == 1, "ring: released the oldest");

	/* freed slot is reusable: a fresh kept record lands in the ring */
	ride_wait_long_keep();
	CHECK(rearm_delay_queue_count_session(1) == 2, "ring: slot reused, two queued again");
	rearm_delay_queue_release_session(1);
	rearm_delay_queue_release_session(1);
	CHECK(rearm_delay_queue_count_session(1) == 0, "ring: drained");
	CHECK(rearm_delay_queue_enqueued() == 3, "ring: enqueued lifetime total is 3");
}

static void test_session_separation(void)
{
	reset();
	ride_wait_long_keep();                            /* stamped session 1 */

	rearm_delay_set_session_id(2);
	active_baseline(100);
	step(ST_SUSPENDED);
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);
	step(ST_COLD);                                   /* stamped session 2 */

	CHECK(rearm_delay_queue_count_session(1) == 1, "sessions: session 1 has its record");
	CHECK(rearm_delay_queue_count_session(2) == 1, "sessions: session 2 has its record");
	CHECK(rearm_delay_queue_count_session(3) == 0, "sessions: unknown session has none");
	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(2, &r) && r.session_id == 2, "sessions: peek by session returns that session");
	CHECK(rearm_delay_queue_peek_session(1, &r) && r.session_id == 1, "sessions: session 1 untouched by 2's peek");
}

static void test_flags_and_committed(void)
{
	reset();
	snap.sensor_valid = true;                        /* so SENSOR_VALID is asserted in the record */
	snap.calibration_source = TORQUE_CAL_SOURCE_USER; /* so CAL_USER is asserted */
	active_baseline(100);
	in.direction_inhibit_active = true;
	in.real_stop = true;
	in.limiter_zeroed = true;
	in.pwm_on = true;
	step(ST_SUSPENDED);                              /* ENTER_SUSPEND */
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);          /* WAIT_LONG keeps the record */
	step(ST_ACTIVE);                                 /* PERMISSION - committed flag set */
	step(ST_COLD);

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(1, &r), "flags: peek succeeds");
	uint8_t f_enter = r.snapshots[0].flags;
	CHECK((f_enter & REARM_SNAP_F_SENSOR_VALID) != 0, "flags: sensor_valid folded in");
	CHECK((f_enter & REARM_SNAP_F_CAL_USER) != 0, "flags: user calibration folded in");
	uint8_t f_permission = 0;
	for (uint8_t i = 0; i < r.snapshot_count; i++)
		if (r.snapshots[i].milestone_id == REARM_MILESTONE_PERMISSION) f_permission = r.snapshots[i].flags;
	CHECK(f_permission != 0, "flags: PERMISSION snapshot exists");
	CHECK((f_permission & REARM_SNAP_F_COMMITTED) != 0, "flags: PERMISSION snapshot is committed");
	CHECK((f_permission & REARM_SNAP_F_DIRECTION_INHIBIT) != 0, "flags: direction inhibit in snapshot");
	CHECK((f_permission & REARM_SNAP_F_LIMITER_ZEROED) != 0, "flags: limiter-zeroed in snapshot");
	CHECK((f_permission & REARM_SNAP_F_REAL_STOP) != 0, "flags: real-stop in snapshot");
	CHECK((f_permission & REARM_SNAP_F_PWM_ON) != 0, "flags: PWM-on in snapshot");
}

static void test_timeout_closes(void)
{
	reset();
	active_baseline(100);
	step(ST_SUSPENDED);                              /* PREARM edge: record opens */
	step_n(ST_SUSPENDED, T_MS * 5000U + 1U);         /* > TIMEOUT_MS still suspended */

	CHECK(rearm_delay_fsm_state() == 0, "timeout: FSM back to idle on timeout");
	CHECK(rearm_delay_queue_count_session(1) == 1, "timeout: the long wait was kept (WAIT_LONG set)");
	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(1, &r), "timeout: peek succeeds");
	CHECK((r.reason_bits & REARM_DELAY_REASON_WAIT_LONG) != 0, "timeout: WAIT_LONG carried");
	CHECK((r.reason_bits & REARM_DELAY_REASON_NO_PERMISSION) != 0,
		"timeout: closed without the rearm edge ever firing -> NO_PERMISSION");
}

/*
 * PREARM/OWNERSHIP-END are not this module's own "reservation wanted" flag tied to record
 * open/close - they are edges derived from session_state PLUS the record FSM's own transitions,
 * bracketing the whole saga (including the whole RECOVERING watch, not just up to PERMISSION)
 * independently of how many records open and close inside it. This module has no pas_trace link,
 * so what is testable here in isolation is exactly the EDGE TIMING and the one-shot PROBLEM
 * trigger - the actual slot mechanics are covered by pas_trace's own tests and by the full-chain
 * integration test.
 */
static void test_reserve_lifecycle(void)
{
	reset();
	CHECK(!rearm_delay_prearm_edge(), "reserve: no prearm edge at start");
	CHECK(!rearm_delay_ownership_end_edge(), "reserve: no ownership-end edge at start");
	CHECK(!rearm_delay_reserve_trigger(), "reserve: no trigger at start");

	active_baseline(100);
	CHECK(!rearm_delay_prearm_edge(), "reserve: no prearm edge while still ACTIVE");
	step(ST_SUSPENDED);
	/* PREARM fires on the ACTIVE -> SUSPENDED_BY_DIRECTION edge ITSELF - before any record opens
	 * in the old sense (in v2 the record opens on this very tick, anchored here). */
	CHECK(rearm_delay_prearm_edge(), "reserve: prearm edge on ACTIVE -> SUSPENDED");
	CHECK(!rearm_delay_ownership_end_edge(), "reserve: no ownership-end edge on the prearm tick");
	step(ST_SUSPENDED);
	CHECK(!rearm_delay_prearm_edge(), "reserve: prearm edge already consumed (one-shot)");
	CHECK(!rearm_delay_reserve_trigger(), "reserve: no trigger before a problem fires");
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);          /* WAIT_LONG -> first genuine problem */
	CHECK(rearm_delay_reserve_trigger(), "reserve: trigger latched on the problem tick");
	CHECK(!rearm_delay_ownership_end_edge(), "reserve: WAIT_LONG firing is not an ownership-end edge");
	/* the trigger stays latched until the caller consumes it - main.c polls it every tick */
	step(ST_SUSPENDED);
	CHECK(rearm_delay_reserve_trigger(), "reserve: trigger stays latched until consumed");
	rearm_delay_note_reserve_done(REARM_DELAY_NO_CAPTURE, REARM_DELAY_CAPTURE_NO_TRACE_BUSY);
	CHECK(!rearm_delay_reserve_trigger(), "reserve: note_reserve_done clears the trigger");
	/* the one-shot guarantee: a STILL-delayed condition after the consume never re-latches */
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);
	CHECK(!rearm_delay_reserve_trigger(), "reserve: never re-latches after being consumed");
	step(ST_ACTIVE);                                 /* PERMISSION: record moves to RECOVERING */
	/* v5 fix: PERMISSION alone must NOT end ownership - a WEAK_TARGET can still be detected up
	 * to 150 ms later, while RECOVERING. */
	CHECK(!rearm_delay_ownership_end_edge(), "reserve: no ownership-end edge on the permission tick itself");
	CHECK(rearm_delay_fsm_state() == 2, "reserve: FSM in RECOVERING after permission");
	step_n(ST_ACTIVE, 3);
	CHECK(!rearm_delay_ownership_end_edge(), "reserve: still no ownership-end edge while RECOVERING");
	in.real_stop = true;
	step(ST_COLD);                                   /* recovery concludes */
	CHECK(rearm_delay_ownership_end_edge(), "reserve: ownership-end edge fires when RECOVERING closes");
	in.real_stop = false;

	/* a second saga: prearm fires again fresh, independent of the first saga's history. Reset
	 * first so the earlier saga's queued records cannot incidentally block this one's record from
	 * opening - this test is about EDGE timing, not queue depth. */
	reset();
	active_baseline(100);
	step(ST_SUSPENDED);
	CHECK(rearm_delay_prearm_edge(), "reserve: second saga gets its own prearm edge");
	step(ST_SUSPENDED);
	step(ST_COLD);                                   /* terminal without ever getting permission */
	/* No record was left tracking (SUSPENDED's NO_PERMISSION close already ran this tick) - this
	 * is the record-concluded path: ownership still ends. */
	CHECK(rearm_delay_ownership_end_edge(), "reserve: terminal SUSPENDED -> COLD ends ownership");

	/* a saga that RE-SUSPENDS mid-wait (multiple reverse/invalid attempts) must NOT re-fire
	 * prearm or end ownership in between - one reservation spans the whole saga (requirement 2),
	 * and in v2 a re-suspend is simply more SUSPENDED (no WAIT state to oscillate through). */
	reset();
	active_baseline(100);
	step(ST_SUSPENDED);
	CHECK(rearm_delay_prearm_edge(), "reserve: third saga prearm edge");
	step(ST_SUSPENDED);
	step(ST_SUSPENDED);                              /* re-suspend mid-saga: a second reverse */
	CHECK(!rearm_delay_prearm_edge(), "reserve: SUSPENDED -> SUSPENDED (re-suspend) is not a fresh prearm edge");
	CHECK(!rearm_delay_ownership_end_edge(),
		"reserve: re-suspend does NOT end ownership - the saga continues");
	step(ST_SUSPENDED);                              /* still suspended, same record */
	CHECK(rearm_delay_fsm_state() == 1, "reserve: ONE record still tracking the whole multi-reverse saga");
	step(ST_ACTIVE);                                 /* the saga finally gets permission */
	CHECK(!rearm_delay_ownership_end_edge(), "reserve: permission still does not end ownership");
	CHECK(rearm_delay_fsm_state() == 2, "reserve: FSM in RECOVERING after the saga's real permission");
	in.real_stop = true;
	step(ST_COLD);
	CHECK(rearm_delay_ownership_end_edge(),
		"reserve: ownership-end edge on the saga's real conclusion, however many reverse attempts it took");
	in.real_stop = false;
}

static void test_wraparound_wait_long(void)
{
	reset();
	tick = 0xFFFFFFF0U;                              /* the ride straddles the 32-bit tick wrap */
	in.iq_pre_ramp = 100;
	step_n(ST_ACTIVE, 5);                            /* baseline */
	step(ST_SUSPENDED);                              /* PREARM edge: anchor just before the wrap */
	step_n(ST_SUSPENDED, T_MS * 250U + 50U);         /* > 200 ms, crossing 0xFFFFFFFF */
	step(ST_COLD);

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_count_session(1) == 1, "wrap: record kept");
	CHECK(rearm_delay_queue_peek_session(1, &r), "wrap: peek succeeds");
	CHECK((r.reason_bits & REARM_DELAY_REASON_WAIT_LONG) != 0, "wrap: WAIT_LONG fired across the wrap");
	uint8_t n_problem = 0;
	for (uint8_t i = 0; i < r.snapshot_count; i++)
		if (r.snapshots[i].milestone_id == REARM_MILESTONE_PROBLEM) {
			n_problem++;
			CHECK(r.snapshots[i].elapsed_ticks < 0xFFFFU,
				"wrap: PROBLEM snapshot elapsed is the true post-wrap value, not a giant number");
			CHECK(r.snapshots[i].elapsed_ticks >= REARM_DELAY_WAIT_LONG_TICKS,
				"wrap: PROBLEM snapshot elapsed >= the 200 ms window");
		}
	CHECK(n_problem == 1, "wrap: one PROBLEM snapshot");
	CHECK(r.t_close != REARM_DELAY_T_UNREACHED, "wrap: t_close set");
	CHECK(r.t_close < 0xFFFFU, "wrap: t_close is a sane post-wrap elapsed");
	CHECK(r.t_close >= REARM_DELAY_WAIT_LONG_TICKS, "wrap: t_close after the long wait");
}

static void test_wraparound_weak(void)
{
	reset();
	tick = 0xFFFFFFF0U;
	in.iq_pre_ramp = 100;
	step_n(ST_ACTIVE, 5);                            /* baseline 100 -> 80 % benchmark */
	step(ST_SUSPENDED);                              /* PREARM edge */
	step(ST_SUSPENDED);
	step(ST_ACTIVE);                                 /* PERMISSION just before the wrap */
	in.iq_pre_ramp = 50;                             /* below 80 % the whole time */
	step_n(ST_ACTIVE, T_MS * 200U + 50U);            /* > 150 ms continuous, crossing the wrap */
	step(ST_COLD);

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_count_session(1) == 1, "wrap-weak: record kept");
	CHECK(rearm_delay_queue_peek_session(1, &r), "wrap-weak: peek succeeds");
	CHECK((r.reason_bits & REARM_DELAY_REASON_WEAK_TARGET) != 0,
		"wrap-weak: WEAK_TARGET window survived the 32-bit wrap");
	CHECK(r.t_weak_start != REARM_DELAY_T_UNREACHED, "wrap-weak: t_weak_start set");
	CHECK(r.t_weak_start < 0xFFFFU, "wrap-weak: t_weak_start is a sane post-wrap elapsed");
}

/*
 * The wraparound case that needs the EXPLICIT weak_running bool: the continuous below-80 %
 * stretch STARTS at tick 0 (the ride wrapped so the permission landed on the last tick before 0).
 * weak_start_tick == 0 is then a LEGITIMATE start, not "never started" - a sentinel that treats
 * 0 as "not running" would re-arm the window on every tick and the 150 ms window would never
 * complete. The v2 bool (weak_running) is the truth here.
 */
static void test_weak_stretch_starting_at_tick_zero(void)
{
	reset();
	tick = 0xFFFFFFFCU;
	in.iq_pre_ramp = 100;
	step(ST_ACTIVE);                                 /* baseline at FC, tick -> FD */
	step(ST_SUSPENDED);                              /* FD = PREARM edge (the anchor), tick -> FE */
	step(ST_SUSPENDED);                              /* FE, tick -> FF */
	step(ST_ACTIVE);                                 /* PERMISSION at FF, tick -> 0 */
	in.iq_pre_ramp = 50;                             /* below 80 % from the very first recovering tick */
	step(ST_ACTIVE);                                 /* the stretch STARTS at tick 0 */
	step_n(ST_ACTIVE, T_MS * 200U + 1U);             /* > 150 ms continuous, now well past the wrap */
	step(ST_COLD);

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_count_session(1) == 1, "wrap-t0: record kept");
	CHECK(rearm_delay_queue_peek_session(1, &r), "wrap-t0: peek succeeds");
	CHECK(r.pre_reverse_iq == 100, "wrap-t0: baseline captured");
	CHECK((r.reason_bits & REARM_DELAY_REASON_WEAK_TARGET) != 0,
		"wrap-t0: WEAK_TARGET fires even though the stretch began at tick 0");
	CHECK(r.t_weak_start != REARM_DELAY_T_UNREACHED, "wrap-t0: t_weak_start set");
	CHECK(r.t_weak_start == 3, "wrap-t0: t_weak_start is the true 3-tick elapsed (anchor at FD, stretch from tick 0)");
}

/*
 * v5 Bug 2 fix: a PROBLEM that closes its own record in the SAME tick (note_problem() then
 * immediately close_record()) must still leave the trigger visible to the caller on that tick,
 * and note_reserve_done() must still be able to write the outcome into the record that raised
 * it - even though R.cur_slot is already 0xFF and the record already sits in the queue. In v2
 * the terminal SUSPENDED -> COLD path before WAIT_LONG is the canonical same-tick close (the old
 * "reverse during WAIT -> NO_COMMIT via re-suspend" path no longer exists - a re-suspend simply
 * stays SUSPENDED, so the terminal case is the one that closes without WAIT_LONG).
 */
static void test_trigger_survives_same_tick_close_terminal(void)
{
	reset();
	active_baseline(100);
	step(ST_SUSPENDED);                              /* PREARM edge: record opens, record_id 0 */
	CHECK(!rearm_delay_reserve_trigger(), "same-tick close: no trigger yet");

	in.real_stop = true;
	step(ST_COLD);                                    /* terminal before WAIT_LONG: NO_PERMISSION + close, same tick */
	in.real_stop = false;
	CHECK(rearm_delay_fsm_state() == 0, "same-tick close: FSM back to IDLE - record closed");
	CHECK(rearm_delay_reserve_trigger(),
		"same-tick close: trigger SURVIVES close_record() on the same tick - Bug 2 fix");
	CHECK(rearm_delay_queue_count_session(1) == 1, "same-tick close: record already queued");

	/* main.c's exact next step: report the outcome. Must land on the record that JUST closed,
	 * not be silently lost because R.cur_slot is already 0xFF. */
	rearm_delay_note_reserve_done(7U, REARM_DELAY_CAPTURE_TRACE_ONLY);
	CHECK(!rearm_delay_reserve_trigger(), "same-tick close: note_reserve_done clears the trigger");

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(1, &r), "same-tick close: peek succeeds");
	CHECK(r.capture_id == 7U, "same-tick close: capture_id written to the QUEUED record");
	CHECK(r.capture_status == REARM_DELAY_CAPTURE_TRACE_ONLY,
		"same-tick close: capture_status written to the QUEUED record");
	CHECK((r.reason_bits & REARM_DELAY_REASON_NO_PERMISSION) != 0,
		"same-tick close: NO_PERMISSION reason");
}

/*
 * v5 Bug 2 fix, the queue-shift case: the pending record has ALREADY had ANOTHER session's
 * record dequeued out from under it (rearm_delay_queue_release_session shifts the ring) by the
 * time note_reserve_done() runs - the token (record_id, session_id) must still find it correctly
 * by CONTENT, not by whatever physical slots[] index it now occupies.
 */
static void test_trigger_result_survives_queue_shift(void)
{
	reset();
	/* First, an unrelated session's record gets queued and sits at the head of the ring. Consume
	 * its own trigger first, exactly as main.c always does the same tick it sees one - a leaked,
	 * never-consumed trigger is a test-harness bug (main.c never leaves one pending), not a
	 * scenario this module needs to tolerate. */
	ride_wait_long_keep();                            /* session 1, record_id 0, queued at q_head */
	CHECK(rearm_delay_reserve_trigger(), "queue shift: setup: session 1's own WAIT_LONG trigger fired");
	rearm_delay_note_reserve_done(1U, REARM_DELAY_CAPTURE_TRACE_ONLY);

	/* Second session begins; its own record is about to close+trigger in the same tick. */
	rearm_delay_set_session_id(2U);
	active_baseline(100);
	step(ST_SUSPENDED);                              /* PREARM edge: session 2 record opens */
	in.real_stop = true;
	step(ST_COLD);                                    /* NO_PERMISSION + close, same tick, session 2 */
	in.real_stop = false;
	CHECK(rearm_delay_reserve_trigger(), "queue shift: trigger latched for session 2's record");
	CHECK(rearm_delay_queue_count_session(2U) == 1, "queue shift: session 2 record queued (ring now has 2)");

	/* Now the FIRST (session 1) record gets dequeued/released - shifting the ring - BEFORE
	 * main.c gets around to reporting session 2's pending capture result. */
	rearm_delay_queue_release_session(1U);
	CHECK(rearm_delay_queue_count_session(1U) == 0, "queue shift: session 1 released");
	CHECK(rearm_delay_queue_count_session(2U) == 1, "queue shift: session 2 still queued, now shifted to q_head");

	rearm_delay_note_reserve_done(42U, REARM_DELAY_CAPTURE_FULL);
	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(2U, &r), "queue shift: session 2 peek succeeds after the shift");
	CHECK(r.capture_id == 42U, "queue shift: capture_id found session 2's record by token, not by stale index");
	CHECK(r.capture_status == REARM_DELAY_CAPTURE_FULL, "queue shift: capture_status likewise correct");
}

/* ============================================================================================ */
/* v5.1: INTERNAL record_uid - collision-proof note_reserve_done() target resolution.          */
/* ============================================================================================ */

/*
 * A WAIT_LONG-kept record whose trigger is consumed the same tick main.c would (capture result
 * lands in the queued record). Returns nothing - the record is queued and its trigger is clean.
 */
static void keep_wait_long_and_consume(uint8_t capture_id, uint8_t capture_status)
{
	active_baseline(100);
	step(ST_SUSPENDED);                              /* PREARM edge: record opens */
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);          /* WAIT_LONG -> problem, trigger latched */
	step(ST_COLD);                                   /* record kept, queued */
	CHECK(rearm_delay_reserve_trigger(), "keep-consume: setup: trigger latched");
	rearm_delay_note_reserve_done(capture_id, capture_status);
}

/* A healthy fast rearm: measured, DISCARDED (reason_bits == 0), slot freed. Advances record_seq. */
static void healthy_fast_rearm(void)
{
	active_baseline(100);
	step(ST_SUSPENDED);                              /* PREARM edge: record opens, ENTER_SUSPEND */
	in.iq_pre_ramp = 100;                            /* target immediately recovered */
	in.iq_setpoint = 100;
	step(ST_SUSPENDED);
	step(ST_ACTIVE);                                 /* PERMISSION -> RECOVERING */
	step_n(ST_ACTIVE, 3);                            /* recovery, healthy */
	step(ST_COLD);                                   /* closes, reason 0 -> discarded */
}

/*
 * v5.1 Test A - the 8-bit wire record_id COLLISION. A record stays queued while 255 healthy
 * rearms advance record_seq to a wrap; the NEXT problem record reuses wire record_id 0 in the
 * SAME session. note_reserve_done() must find the NEW record by its internal UID, never the old
 * one - the old record's capture must be untouched even though both share (session_id=1,
 * record_id=0) exactly. This test FAILS on mutation #1 (search by wire pair instead of UID).
 */
static void test_uid_256_record_collision(void)
{
	reset();
	keep_wait_long_and_consume(0x11U, REARM_DELAY_CAPTURE_TRACE_ONLY);   /* record_id 0, queued */

	rearm_delay_record_t old_rec;
	CHECK(rearm_delay_queue_peek_session(1, &old_rec), "256-collision: setup peek");
	CHECK(old_rec.record_id == 0 && old_rec.session_id == 1, "256-collision: setup record is (1, 0)");

	/* 255 healthy rearms: record_seq goes 1 -> 256, wrapping the uint8_t to 0. The old record
	 * stays queued throughout (each healthy rearm opens the free slot and frees it again). */
	for (uint32_t i = 0; i < 255U; i++) {
		healthy_fast_rearm();
	}
	CHECK(rearm_delay_fsm_state() == 0, "256-collision: FSM idle after the 255 healthy rearms");
	CHECK(rearm_delay_queue_count_session(1) == 1, "256-collision: the old record survived the 255 rearms");

	/* The next problem record wraps to wire record_id 0 - an exact (session, record_id) collision
	 * with the still-queued old record - and closes in the SAME tick as its own problem. */
	active_baseline(100);
	step(ST_SUSPENDED);                              /* new record opens, wire record_id 0 again */
	step(ST_SUSPENDED);                              /* brief wait */
	in.real_stop = true;
	step(ST_COLD);                                   /* terminal before WAIT_LONG: NO_PERMISSION + close, same tick */
	in.real_stop = false;
	CHECK(rearm_delay_reserve_trigger(), "256-collision: new record's trigger latched");
	rearm_delay_note_reserve_done(0x22U, REARM_DELAY_CAPTURE_FULL);

	rearm_delay_record_t new_rec;
	CHECK(rearm_delay_queue_count_session(1) == 2, "256-collision: both records queued");

	/* The OLD record is still the oldest - peek it LIVE and demand its capture is untouched.
	 * A UID-resolved result must land on the NEW record only. */
	CHECK(rearm_delay_queue_peek_session(1, &new_rec), "256-collision: old record is the oldest");
	CHECK(new_rec.record_id == old_rec.record_id && new_rec.session_id == old_rec.session_id,
		"256-collision: the two records really DO share the wire (session, record_id) pair");
	CHECK(new_rec.capture_id == 0x11U && new_rec.capture_status == REARM_DELAY_CAPTURE_TRACE_ONLY,
		"256-collision: old record's capture UNTOUCHED");
	rearm_delay_queue_release_session(1);            /* release the old one */
	CHECK(rearm_delay_queue_peek_session(1, &new_rec), "256-collision: new record peekable after old released");
	CHECK(new_rec.record_id == 0, "256-collision: new record really has wire record_id 0");
	CHECK(new_rec.capture_id == 0x22U && new_rec.capture_status == REARM_DELAY_CAPTURE_FULL,
		"256-collision: new record got the capture result, not the old one");
}

/*
 * v5.1 Test B - the record_uid sidecar must MOVE with its record on a PHYSICAL ring shift
 * (rearm_delay_queue_release_session with off != 0 copies slot[head+1] over slot[head]). The
 * pending record's trigger is deliberately left unresolved across the shift; note_reserve_done()
 * must still find the record at its NEW slot by UID. Fails on mutation #2 (sidecar not moved
 * with the record): the UID stays behind on the stale slot and the search finds nothing.
 */
static void test_uid_sidecar_moves_with_record(void)
{
	reset();
	active_baseline(100);
	step(ST_SUSPENDED);                              /* A opens, record_id 0 */
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);          /* WAIT_LONG -> A's trigger latches (uid 0) */
	CHECK(rearm_delay_reserve_trigger(), "uid-move: A's trigger latched");
	step(ST_COLD);                                   /* A queued at slot 0, trigger STILL pending */
	CHECK(rearm_delay_reserve_trigger(), "uid-move: A's trigger still pending after its close");

	/* B (session 2) opens in the free slot; its own problem must NOT steal the trigger - it is
	 * already reserved by A, so B queues without latching anything. */
	rearm_delay_set_session_id(2U);
	active_baseline(100);
	step(ST_SUSPENDED);                              /* B opens in slot 1 */
	step(ST_SUSPENDED);
	in.real_stop = true;
	step(ST_COLD);                                   /* B: NO_PERMISSION + close same tick, no latch */
	in.real_stop = false;
	CHECK(rearm_delay_queue_count_session(1U) == 1 && rearm_delay_queue_count_session(2U) == 1,
		"uid-move: both records queued");

	/* Release B with off == 1: A is PHYSICALLY copied slot 0 -> slot 1. */
	rearm_delay_queue_release_session(2U);
	CHECK(rearm_delay_queue_count_session(1U) == 1, "uid-move: A survives the shift");
	CHECK(rearm_delay_queue_count_session(2U) == 0, "uid-move: B released");

	rearm_delay_note_reserve_done(0x77U, REARM_DELAY_CAPTURE_FULL);
	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(1U, &r), "uid-move: A peekable at its new slot");
	CHECK(r.capture_id == 0x77U && r.capture_status == REARM_DELAY_CAPTURE_FULL,
		"uid-move: note_reserve_done reached A by UID at its new slot");
	CHECK(r.record_id == 0, "uid-move: it is indeed A");
}

/*
 * v5.1 Test C - the uint32_t UID generator wrap and the skip-in-use rule, proven with the
 * test-only seam (REARM_UID_SEAM_TEST - compiled only into this host build, never the firmware;
 * see the #ifdef in rearm_delay_diag.c). A queued record still holds UID 0 while the generator
 * wraps 0xFFFFFFFF -> 0: the allocator MUST skip 0 and hand the next record UID 1, or a later
 * note_reserve_done() would target the WRONG record. Fails on mutation #3 (UID reused while still
 * queued) and on mutation #5 (pending UID lost before resolve).
 */
static void test_uid_wrap_and_skip(void)
{
	reset();
	keep_wait_long_and_consume(0x10U, REARM_DELAY_CAPTURE_TRACE_ONLY);  /* A: UID 0, queued */

	/* Drive the generator up against the wrap. B then takes 0xFFFFFFFF and the generator wraps
	 * to 0 - the wrap itself is proven by rearm_delay_test_uid_next(). */
	rearm_delay_test_set_uid_next(0xFFFFFFFFU);
	rearm_delay_set_session_id(2U);
	keep_wait_long_and_consume(0x20U, REARM_DELAY_CAPTURE_FULL);
	CHECK(rearm_delay_test_uid_next() == 0U, "uid-wrap: generator wrapped 0xFFFFFFFF -> 0");
	CHECK(rearm_delay_queue_count_session(1U) == 1 && rearm_delay_queue_count_session(2U) == 1,
		"uid-wrap: A (UID 0) and B (UID 0xFFFFFFFF) both queued");

	/* Release B (off == 1): A is physically shifted slot 0 -> slot 1, still holding UID 0. The
	 * freed slot 0 is where the next record will open. */
	rearm_delay_queue_release_session(2U);
	CHECK(rearm_delay_queue_count_session(1U) == 1, "uid-wrap: A alone queued after the shift");

	/* C opens in the freed slot while A (UID 0) is STILL queued. Candidate 0 is in use -> the
	 * allocator must skip it and hand C UID 1. If it reused 0, C's later result would land on A. */
	rearm_delay_set_session_id(3U);
	active_baseline(100);
	step(ST_SUSPENDED);                              /* C opens: candidate 0 is A's -> skip to 1 */
	CHECK(rearm_delay_test_uid_next() == 2U, "uid-wrap: allocator SKIPPED 0 (queued A owns it)");
	step_n(ST_SUSPENDED, T_MS * 250U + 1U);          /* C's WAIT_LONG -> trigger latches on C */
	CHECK(rearm_delay_reserve_trigger(), "uid-wrap: C's trigger latched");
	step(ST_COLD);                                   /* C kept, queued */
	rearm_delay_note_reserve_done(0x30U, REARM_DELAY_CAPTURE_TRACE_ONLY);

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_count_session(3U) == 1, "uid-wrap: C queued");
	CHECK(rearm_delay_queue_peek_session(3U, &r), "uid-wrap: C peekable");
	CHECK(r.capture_id == 0x30U && r.capture_status == REARM_DELAY_CAPTURE_TRACE_ONLY,
		"uid-wrap: C got its own result - not written onto A (the UID-0 owner)");
	CHECK(rearm_delay_queue_peek_session(1U, &r), "uid-wrap: A peekable");
	CHECK(r.capture_id == 0x10U && r.capture_status == REARM_DELAY_CAPTURE_TRACE_ONLY,
		"uid-wrap: A's capture untouched by C's result");
}

/*
 * v5.1 Test D - note_reserve_done() with NO exact UID match must write NOTHING. The pending
 * trigger's record was released from the queue (already dumped) before the result arrived; the
 * only other record in the ring does NOT own the pending UID. The module must silently discard,
 * never fall back to "the first record". Fails on mutation #6 (write to first record after no
 * match).
 */
static void test_uid_no_match_writes_nothing(void)
{
	reset();
	keep_wait_long_and_consume(0x50U, REARM_DELAY_CAPTURE_TRACE_ONLY);   /* A queued, uid 0 */

	/* B (session 2) closes with a problem in the same tick: its trigger latches (uid 1) and it
	 * queues. */
	rearm_delay_set_session_id(2U);
	active_baseline(100);
	step(ST_SUSPENDED);                              /* B opens */
	step(ST_SUSPENDED);
	in.real_stop = true;
	step(ST_COLD);                                   /* B: NO_PERMISSION + close same tick */
	in.real_stop = false;
	CHECK(rearm_delay_reserve_trigger(), "no-match: B's trigger latched");
	CHECK(rearm_delay_queue_count_session(1U) == 1 && rearm_delay_queue_count_session(2U) == 1,
		"no-match: A and B queued");

	/* B is released BEFORE its result arrives - its trigger's record no longer exists anywhere. */
	rearm_delay_queue_release_session(2U);
	CHECK(rearm_delay_queue_count_session(2U) == 0, "no-match: B released (record gone)");
	rearm_delay_note_reserve_done(0x99U, REARM_DELAY_CAPTURE_FULL);
	CHECK(!rearm_delay_reserve_trigger(), "no-match: trigger consumed exactly once");

	rearm_delay_record_t r;
	CHECK(rearm_delay_queue_peek_session(1U, &r), "no-match: A peekable");
	CHECK(r.capture_id == 0x50U && r.capture_status == REARM_DELAY_CAPTURE_TRACE_ONLY,
		"no-match: A's capture UNTOUCHED - no fallback write to the first record");
}

int main(void)
{
	printf("FW-111 v5 rearm_delay_diag.c (FW-112 v2 semantics), against the shipped module\n");
	test_healthy_fast_rearm_discarded();
	test_wait_long_kept();
	test_no_permission_kept();
	test_no_load_kept();
	test_weak_target_kept();
	test_brief_dip_not_weak();
	test_baseline_is_last_active_not_max();
	test_guaranteed_snapshot_set_and_timings();
	test_timings_are_one_shot();
	test_snapshot_cap_no_stage_evicted();
	test_standstill_timings();
	test_bug1_standstill_span_is_real_time();
	test_bug2_permission_tick_records_all_stages();
	test_queue_full_refused();
	test_release_and_ring();
	test_session_separation();
	test_flags_and_committed();
	test_timeout_closes();
	test_reserve_lifecycle();
	test_wraparound_wait_long();
	test_wraparound_weak();
	test_weak_stretch_starting_at_tick_zero();
	test_trigger_survives_same_tick_close_terminal();
	test_trigger_result_survives_queue_shift();
	test_uid_256_record_collision();
	test_uid_sidecar_moves_with_record();
	test_uid_wrap_and_skip();
	test_uid_no_match_writes_nothing();

	if (host_test_failures != 0) {
		printf("rearm_delay_diag_host: %d FAILURES\n", host_test_failures);
		return 1;
	}
	printf("rearm_delay_diag_host: ALL PASS\n");
	return 0;
}
