/*
 * FW-111 v5.1 / FW-112 v2: FULL-CHAIN integration harness. Links the REAL production chain:
 *
 *     src/pas_quadrature.c   the raw quadrature decode table
 *     src/pas_direction.c    the PAS direction safety automaton (FORWARD_SAFE / DIRECTION_INHIBIT
 *                            / FORWARD_CONFIRMING) - classifies every step and derives
 *                            direction_inhibit_active / forward_confirmed_this_tick
 *     src/ride_session.c     the ride SESSION automaton (COLD / ACTIVE / SUSPENDED_BY_DIRECTION),
 *                            the single authoritative owner of whether assist current may flow -
 *                            PERMISSION, NOT DEMAND (FW-112 v2: no WAIT_REARM_LOAD, no commit)
 *     src/rearm_delay_diag.c the FW-111 delayed-rearm recorder and its prearm/ownership-end edges
 *     src/pas_trace.c        the decoder-level PAS trace, dynamic-slot rearm reservation with a
 *                            SEPARATE ownership-vs-retained-capture lifecycle
 *     src/pas_raw.c          the ISR-level raw PAS trace, paired by capture_id
 *
 * ride_control.c/torque_input.c/assist_modes.c are DELIBERATELY NOT linked: FW-112 v2 dropped the
 * two-phase commit entirely - the SUSPENDED_BY_DIRECTION -> ACTIVE rearm edge is a PURE direction
 * fact of ride_session.c, with no pressure/Iq-demand condition, so there is nothing here for a
 * real assist pipeline to "decide" (the caller re-evaluates demand FRESH every tick in
 * ride_control.c, which is tested in tests/host/ride_control_rearm_host.c). What THIS file needs
 * to prove is the direction/session/capture-ownership chain, not assist_modes' own arithmetic, so
 * the per-tick demand the recorder's timing stages judge is a test-controlled "what a real
 * calculation would have returned this tick" value (g_commit_iq_request) fed into iq_request /
 * iq_pre_ramp in exactly the role ride_control.c plays for it - a deliberate, justified scope
 * choice, not a shortcut around linking the real automaton.
 *
 * do_tick() drives ONE control tick in EXACTLY main.c's real order: the ordinary
 * pas_trace_transition() decode-site call (BEFORE the session update, using the PREVIOUS tick's
 * latched flag - main.c's own ordering) -> pas_quadrature_step()/pas_direction_on_step() ->
 * ride_session_update() -> rearm_delay_tick() -> PREARM / trigger+capture / OWNERSHIP-END,
 * in that exact order (see src/main.c's own comment on why that order matters: a same-tick
 * trigger must never lose its reservation before it can use it).
 *
 * SCENARIOS. S1-S12 prove the dynamic-slot ownership automaton and its wire contract (schema,
 * capture_status, session/capture id integrity) against the real chain. S13-S21 additionally
 * prove the reservation's lifecycle is decoupled from both the rearm edge and this record's own
 * queue slot - the two defects this card's rewrite fixes (see inc/pas_trace.h and
 * inc/rearm_delay_diag.h for the full rationale):
 *   S1  real ACTIVE->reverse->SUSPENDED->WAIT_LONG->confirm->ACTIVE: TRACE contains BOTH the
 *       initiating reverse AND the forward-confirming steps; record/TRACE/RAW share capture_id;
 *       status FULL; retries against the already-armed slot count ONE refusal.
 *   S2  re-suspend mid-saga (a reverse while ALREADY SUSPENDED_BY_DIRECTION, before WAIT_LONG): the
 *       real automaton simply stays SUSPENDED - ONE record, ONE reservation, the same slot. The v1
 *       WAIT<->SUSPENDED oscillation (and its second record) no longer exists.
 *   S3  the dynamically-chosen slot would be the ordinary active watcher: freeze slot 0, drive the
 *       ordinary watcher to slot 1, then a real reverse/rearm - prearm must refuse, never silently
 *       reuse the active slot (PAS_TRACE_SLOTS=2 in DIAG builds).
 *   S4  an ordinary capture arms WHILE direction is inhibited - must not corrupt the reservation.
 *   S5  the only other slot is still READY (RETAINED) from an earlier, unstreamed rearm capture -
 *       prearm must refuse outright rather than silently repurpose it.
 *   S6  healthy fast rearm followed by a second, delayed rearm: the second TRACE must contain none
 *       of the first (healthy) event's samples.
 *   S7  two consecutive sessions: session B's capture carries no session A samples.
 *   S8  RAW busy: the record reports TRACE_ONLY, never FULL.
 *   S9  no free TRACE at the initiating event: exactly ONE increment of capture_slots_full, never
 *       per retry-tick - proven against the real one-shot prearm_edge(), not a synthetic retry.
 *   S10 session end during PREARM with no PROBLEM: the reservation is released, no orphaned hold.
 *   S11 session end during POST: TRACE and RAW both partial, same capture_id.
 *   S12 uint32_t control-tick wraparound across the whole reservation cycle.
 *   S13 WEAK_TARGET detected >150 ms AFTER the rearm: the reservation is still held then, TRACE still
 *       contains the initiating reverse and forward-confirmation, status is FULL/TRACE_ONLY, never
 *       NO_TRACE_NO_HISTORY.
 *   S14 healthy rearm: no record kept, and the reservation is not orphaned once RECOVERING
 *       concludes (not "immediately at rearm").
 *   S15 a new reverse during RECOVERING, old slot never armed: old ownership ends cleanly (freed,
 *       not retained), a fresh PREARM seeds from the NEW reverse, no old samples survive.
 *   S16 a new reverse during RECOVERING, old capture armed/ready: it is untouched, the new saga
 *       does not take it over, and gets an explicit NO_TRACE_NO_HISTORY when nothing is free.
 *   S17 re-suspend AFTER WAIT_LONG already armed the capture: the armed capture survives the
 *       re-suspend, still ONE record and ONE reservation - never a second record, never a re-arm.
 *   S18 terminal SUSPENDED_BY_DIRECTION -> COLD before WAIT_LONG (NO_PERMISSION): the record closes
 *       in the SAME tick its trigger fires - the trigger is not lost, and the QUEUED record still
 *       receives its capture_id/status; the armed slot becomes RETAINED, not silently dropped.
 *   S19 record queue full: the reservation is not orphaned even with no record tracking the saga.
 *   S20 two sessions driven through the FULL ownership lifecycle: no id/sample mixing.
 *   S21 a RETAINED armed shadow still receives its complete POST window; the ordinary watcher
 *       never takes the slot over.
 *   S22 capture_id and session_id are fixed at PREARM time - a diag session boundary crossed while
 *       the reservation is open never re-stamps the capture.
 */

#include "../common/check.h"

#include "pas_quadrature.h"
#include "pas_direction.h"
#include "ride_session.h"
#include "rearm_delay_diag.h"
#include "pas_trace.h"
#include "pas_raw.h"
#include "torque_input.h"

#include <stdio.h>
#include <string.h>

/* --- the real forward/reverse/invalid quadrature ring, derived from pas_quadrature.c's own
 * table (qd[16] = {0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0}) TIMES config.h's PAS_DIR_SIGN (-1) -
 * not re-guessed, checked against the real decoder by test_quadrature_ring_matches_decoder()
 * below before anything else trusts it. ---------------------------------------------------- */
static const uint8_t FWD_NEXT[4] = {2U, 0U, 3U, 1U};   /* 0->2->3->1->0, each step dir==+1 */
static const uint8_t REV_NEXT[4] = {1U, 3U, 0U, 2U};   /* 0->1->3->2->0, each step dir==-1 */

#define EV_NONE    0
#define EV_FORWARD 1
#define EV_REVERSE 2
#define EV_INVALID 3

static uint32_t g_tick;
static uint8_t  g_qstate;
static bool     g_was_latched;
static torque_snapshot_t g_snap;
static bool     g_terminal_cut;
static bool     g_assist_off;
static bool     g_real_stop;
static bool     g_cold_start_ready;
static int32_t  g_commit_iq_request;   /* what a real assist-mode calculation would return THIS tick */
static int32_t  g_iq_setpoint;
static bool     g_pwm_on;
static uint8_t  g_session_id;

/* Last-tick outputs, for scenario assertions. */
static ride_session_output_t g_last_sout;
static uint8_t  g_last_trace_cap;      /* the ordinary decode-site capture id, or NO_CAPTURE */

static void reset_all(void)
{
    pas_direction_init();
    ride_session_init();
    rearm_delay_init();
    pas_trace_init();
    pas_raw_init();
    memset(&g_snap, 0, sizeof(g_snap));
    g_tick = 0;
    g_qstate = 0;
    g_was_latched = false;
    g_terminal_cut = false;
    g_assist_off = false;
    g_real_stop = false;
    g_cold_start_ready = false;
    g_commit_iq_request = 0;
    g_iq_setpoint = 0;
    g_pwm_on = false;
    g_session_id = 1;
    pas_trace_set_session_id(g_session_id);
    pas_raw_set_session_id(g_session_id);
    rearm_delay_set_session_id(g_session_id);
    memset(&g_last_sout, 0, sizeof(g_last_sout));
    g_last_trace_cap = PAS_TRACE_NO_CAPTURE;
}

/* One control tick, exactly main.c's real order - see the file header. */
static void do_tick(int event)
{
    g_tick++;
    uint8_t from_state = g_qstate;
    uint8_t to_state = from_state;
    bool had_transition = (event != EV_NONE);
    int8_t decoded_dir = 0;
    if (had_transition) {
        if (event == EV_FORWARD)      to_state = FWD_NEXT[from_state];
        else if (event == EV_REVERSE) to_state = REV_NEXT[from_state];
        else                          to_state = (uint8_t)(from_state ^ 3U);   /* EV_INVALID */
        decoded_dir = pas_quadrature_step(from_state, to_state);
        g_qstate = to_state;
    }

    /* --- ordinary pas_trace decode-site call, BEFORE the session update - main.c's own order,
     * using the PREVIOUS tick's latched flag (ride_control_update() has not run yet this tick). */
    g_last_trace_cap = PAS_TRACE_NO_CAPTURE;
    if (had_transition) {
        pas_raw_isr_sample(to_state, g_tick);
        pas_trace_input_t pt_in = {
            .from_state = from_state, .to_state = to_state,
            .reverse = (decoded_dir < 0),
            .gap_ticks = 50U,
            .disc_pos = (uint16_t)(g_tick % 96U),
            .load_centikg = g_snap.load_centikg,
            .torque_raw_mv = g_snap.raw_native,
            .torque_fast = g_snap.assist_delta_filtered_native,
            .iq_setpoint = (uint16_t)((g_iq_setpoint < 0) ? 0 : g_iq_setpoint),
            .brake = false,
            .rolling = true,
            .latched = g_was_latched
        };
        g_last_trace_cap = pas_trace_transition(&pt_in);
        if (g_last_trace_cap != PAS_TRACE_NO_CAPTURE) pas_raw_freeze(g_last_trace_cap);
    }

    /* --- the real direction automaton, fed the real decoded step ---------------------------- */
    if (had_transition) {
        (void)pas_direction_on_step(decoded_dir);
    }
    bool direction_inhibit = pas_direction_direction_inhibit_active();
    bool forward_confirmed = had_transition && pas_direction_forward_confirmed_last_call();

    /* --- the real session automaton, fed the real direction facts --------------------------- */
    ride_session_input_t sin;
    memset(&sin, 0, sizeof(sin));
    sin.direction_inhibit_active = direction_inhibit;
    sin.forward_confirmed_this_tick = forward_confirmed;
    sin.non_direction_safety_cut = g_terminal_cut;
    sin.assist_off = g_assist_off;
    sin.real_stop = g_real_stop;
    sin.cold_start_ready = g_cold_start_ready;
    ride_session_output_t sout;
    ride_session_update(&sin, &sout);
    g_iq_setpoint = sout.latched ? g_commit_iq_request : 0;
    g_last_sout = sout;

    /* --- the FW-111 recorder, fed this tick's real session state ---------------------------- */
    rearm_delay_input_t rd_in;
    memset(&rd_in, 0, sizeof(rd_in));
    rd_in.session_state = (uint8_t)sout.state;
    rd_in.direction_inhibit_active = direction_inhibit;
    rd_in.real_stop = g_real_stop;
    rd_in.limiter_zeroed = false;
    rd_in.pwm_on = g_pwm_on;
    rd_in.iq_request = (int16_t)g_commit_iq_request;
    rd_in.iq_pre_ramp = (int16_t)(sout.latched ? g_commit_iq_request : 0);
    rd_in.iq_setpoint = (int16_t)g_iq_setpoint;
    rd_in.run_deadband = 5U;
    rd_in.snapshot = &g_snap;
    rearm_delay_tick(&rd_in, g_tick);
    rearm_delay_set_session_id(g_session_id);

    /* --- FW-111 orchestration, byte-for-byte main.c's order: PREARM, then trigger/capture,
     * then OWNERSHIP-END last, so a same-tick trigger is never released out from under itself. -- */
    if (rearm_delay_prearm_edge()) {
        pas_trace_rearm_prearm();
    }
    if (rearm_delay_reserve_trigger()) {
        uint8_t rd_cap = PAS_TRACE_NO_CAPTURE;
        uint8_t rd_status;
        if (!pas_trace_rearm_held()) {
            rd_status = REARM_DELAY_CAPTURE_NO_TRACE_NO_HISTORY;
        } else {
            pas_trace_input_t rt_in = {
                .from_state = g_qstate, .to_state = g_qstate, .reverse = false,
                .gap_ticks = 50U, .disc_pos = (uint16_t)(g_tick % 96U),
                .load_centikg = g_snap.load_centikg, .torque_raw_mv = g_snap.raw_native,
                .torque_fast = g_snap.assist_delta_filtered_native,
                .iq_setpoint = (uint16_t)((g_iq_setpoint < 0) ? 0 : g_iq_setpoint),
                .brake = false, .rolling = true, .latched = sout.latched
            };
            rd_cap = pas_trace_rearm_capture(&rt_in);
            if (rd_cap == PAS_TRACE_NO_CAPTURE) {
                rd_status = REARM_DELAY_CAPTURE_NO_TRACE_BUSY;
            } else {
                rd_status = pas_raw_freeze(rd_cap) ? REARM_DELAY_CAPTURE_FULL
                                                    : REARM_DELAY_CAPTURE_TRACE_ONLY;
            }
        }
        rearm_delay_note_reserve_done(rd_cap, rd_status);
    }
    if (rearm_delay_ownership_end_edge()) {
        pas_trace_rearm_end_ownership();
    }

    g_was_latched = sout.latched;
}

static void fwd(void)  { do_tick(EV_FORWARD); }
static void rev(void)  { do_tick(EV_REVERSE); }
static void inv(void)  { do_tick(EV_INVALID); }
static void hold(uint32_t n) { for (uint32_t i = 0; i < n; i++) do_tick(EV_NONE); }

/*
 * pas_trace's POST window counts real TRANSITIONS dual-written into the ring, not elapsed control
 * ticks - a shadow (or ordinary) capture armed while the crank is stationary stays
 * armed-but-not-ready until real quadrature activity resumes, exactly like a real rider who has
 * stopped. hold() alone can never finish a POST window; these pump real transitions instead, the
 * same way a rider actually pedalling after a rearm would.
 */
static void pump_forward(uint32_t n) { for (uint32_t i = 0; i < n; i++) fwd(); }
static void pump_invalid(uint32_t n) { for (uint32_t i = 0; i < n; i++) inv(); }

/*
 * rearm_delay_diag's RECOVERING state only closes (queuing the record) when the session actually
 * LEAVES ACTIVE - a real ride eventually stops or the rider changes assist level; here a one-tick
 * terminal cut stands in for that so a completed record can be inspected without waiting out its
 * own 5 s internal timeout.
 */
static void end_session_now(void)
{
    g_terminal_cut = true;
    do_tick(EV_NONE);
    g_terminal_cut = false;
}

/* Force COLD -> ACTIVE using ride_session's own ordinary, unchanged cold-start path - not this
 * card's concern, so it is not re-derived, just used for one tick. */
static void establish_active(void)
{
    g_cold_start_ready = true;
    do_tick(EV_NONE);
    g_cold_start_ready = false;
    CHECK(g_last_sout.state == RIDE_SESSION_ACTIVE, "setup: reached ACTIVE via the real cold start");
    CHECK(g_last_sout.latched, "setup: latched after cold start");
}

/* --- a quick, cheap sanity check that the local ring tables above really are the same forward/
 * reverse pairs the real decoder classifies, so every scenario below stands on real ground. --- */
static void test_quadrature_ring_matches_decoder(void)
{
    for (uint8_t s = 0; s < 4U; s++) {
        int8_t f = pas_quadrature_step(s, FWD_NEXT[s]);
        int8_t r = pas_quadrature_step(s, REV_NEXT[s]);
        int8_t iv = pas_quadrature_step(s, (uint8_t)(s ^ 3U));
        char msg[64];
        snprintf(msg, sizeof(msg), "ring: FWD_NEXT[%u] is a real +1 step", (unsigned)s);
        CHECK(f == 1, msg);
        snprintf(msg, sizeof(msg), "ring: REV_NEXT[%u] is a real -1 step", (unsigned)s);
        CHECK(r == -1, msg);
        snprintf(msg, sizeof(msg), "ring: %u^3 is a real illegal (0) step", (unsigned)s);
        CHECK(iv == 0, msg);
    }
}

/* ================================================================================================
 * S1 - full real chain, FULL status, shared capture_id, PRE contains reverse AND forward steps.
 * =============================================================================================== */
static void test_s1_full_chain_full_capture(void)
{
    reset_all();
    establish_active();
    fwd();   /* a real pedalling step first: pas_raw_freeze() needs >=1 recorded RAW event, and the
              * very first transition only adopts the line baseline without recording one - a rider
              * who reverses was pedalling forward just before, so this is the realistic history */

    rev();
    CHECK(g_last_sout.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S1: real SUSPENDED_BY_DIRECTION after reverse");
    CHECK(!g_last_sout.latched, "S1: not latched");
    CHECK(pas_trace_rearm_held(), "S1: prearm claimed a reservation on the initiating edge");
    uint8_t rslot = pas_trace_rearm_slot_index();
    CHECK(rslot < PAS_TRACE_SLOTS, "S1: a real slot index is held");

    /* Let it sit in SUSPENDED_BY_DIRECTION past WAIT_LONG so the problem genuinely fires - no
     * forward confirmation yet (in v2, forward steps ARE exactly what re-arms, so the rearm
     * delay is measured where it really is: the SUSPENDED hold). */
    hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);

    rearm_delay_record_t rec;
    /* No commit yet - still queued as an open capture, but the TRACE side should already be armed
     * (armed-but-not-ready is deliberately invisible to pas_trace_slot_capture_id()/slot_ready() -
     * both only expose a FROZEN capture - so this checks the visible, documented surface only). */
    CHECK(!pas_trace_slot_ready(rslot), "S1: shadow slot still collecting POST at the problem instant");

    /* The reserved slot is now armed (the orchestration's own trigger just fired). A caller
     * retrying pas_trace_rearm_capture() against an already-armed slot - exactly what a second
     * PROBLEM in the same saga, or a defensive re-poll, would do - must count at most ONE
     * refusal, never one per call. */
    uint16_t before_retry = pas_trace_capture_slots_full();
    pas_trace_input_t retry_in = {0};
    for (int i = 0; i < 3; i++) {
        uint8_t r = pas_trace_rearm_capture(&retry_in);
        CHECK(r == PAS_TRACE_NO_CAPTURE, "S1: a retry against the already-armed slot is refused");
    }
    CHECK(pas_trace_capture_slots_full() == before_retry + 1U,
        "S1: three retries against the same busy slot count exactly ONE refusal, not three");

    /* Now let the rider confirm forward - v2 grants permission on direction alone. */
    g_commit_iq_request = 200;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    CHECK(g_last_sout.state == RIDE_SESSION_ACTIVE, "S1: real rearm to ACTIVE on the confirm tick");
    CHECK(g_last_sout.fast_rearm_this_tick, "S1: flagged as fast_rearm");

    pump_forward(PAS_TRACE_POST + 4U);   /* real pedalling after the rearm finishes the POST window */
    CHECK(pas_trace_slot_ready(rslot), "S1: shadow slot frozen and ready after its POST window");
    end_session_now();   /* leaving ACTIVE closes and queues the now-complete record */

    CHECK(rearm_delay_queue_count_session(g_session_id) == 1, "S1: one record queued");
    CHECK(rearm_delay_queue_peek_session(g_session_id, &rec), "S1: peek succeeds");
    CHECK(rec.capture_status == REARM_DELAY_CAPTURE_FULL, "S1: status FULL");
    CHECK(rec.capture_id != REARM_DELAY_NO_CAPTURE, "S1: a real capture id stored");
    CHECK(rec.capture_id == pas_trace_slot_capture_id(rslot), "S1: record capture_id matches the TRACE slot");
    CHECK(pas_raw_slot_capture_id() == rec.capture_id, "S1: RAW capture_id matches the record");

    /* PRE must contain BOTH the initiating reverse AND the two forward-confirming steps. */
    uint16_t n = pas_trace_slot_count(rslot);
    bool saw_reverse = false;
    uint8_t forward_count = 0;
    for (uint16_t i = 0; i < n; i++) {
        pas_trace_sample_t s;
        CHECK(pas_trace_slot_get(rslot, i, &s), "S1: PRE sample readable");
        if ((s.flags & PAS_TR_REVERSE) != 0) saw_reverse = true;
        else forward_count++;
    }
    CHECK(saw_reverse, "S1: TRACE PRE contains the initiating reverse step");
    CHECK(forward_count >= PAS_REVERSE_RECOVERY_CONFIRM_STEPS, "S1: TRACE PRE contains the forward-confirming steps");

    pas_trace_slot_release(rslot);
    pas_raw_slot_release();
}

/* ================================================================================================
 * S2 - re-suspend mid-saga: a reverse while ALREADY SUSPENDED_BY_DIRECTION (before WAIT_LONG). In
 * v2 the real automaton simply STAYS SUSPENDED - ONE record, ONE reservation, the same slot. The
 * v1 WAIT<->SUSPENDED oscillation (and the second record it used to open) no longer exists.
 * =============================================================================================== */
static void test_s2_resuspend_mid_saga_single_record(void)
{
    reset_all();
    establish_active();

    rev();
    CHECK(g_last_sout.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S2: SUSPENDED_BY_DIRECTION");
    uint8_t rslot = pas_trace_rearm_slot_index();

    /* A SECOND reverse, fed to the REAL automaton while already SUSPENDED - in v2 this is simply
     * MORE SUSPENDED: the state does not change, so no fresh prearm edge, no second record, and
     * the ONE reservation is untouched. */
    rev();
    CHECK(g_last_sout.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION,
        "S2: the REAL automaton simply stays SUSPENDED - not manually pinned");
    CHECK(!rearm_delay_prearm_edge(), "S2: re-suspension mid-saga is NOT a fresh prearm edge");
    CHECK(rearm_delay_fsm_state() == 1, "S2: the SAME record stays open - never a second one");
    CHECK(pas_trace_rearm_held(), "S2: the ONE reservation from the first edge is still held");
    CHECK(pas_trace_rearm_slot_index() == rslot, "S2: same slot, not re-chosen");

    /* Let the (single) record reach WAIT_LONG, then rearm. */
    hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);
    g_commit_iq_request = 150;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    CHECK(g_last_sout.state == RIDE_SESSION_ACTIVE, "S2: the saga eventually rearms");
    /* v5 fix: the rearm edge alone must NOT end ownership - a WEAK_TARGET can still fire while
     * RECOVERING. The reservation only ends once recovery actually concludes. */
    CHECK(!rearm_delay_ownership_end_edge(), "S2: no ownership-end edge on the rearm tick itself");
    CHECK(pas_trace_rearm_held(), "S2: reservation still held while RECOVERING");

    pump_forward(PAS_TRACE_POST + 4U);
    end_session_now();
    CHECK(rearm_delay_ownership_end_edge(), "S2: ownership-end edge fires once recovery concludes");

    /* ONE record for the whole saga - the re-suspend must never have opened a second one. */
    rearm_delay_record_t rec;
    CHECK(rearm_delay_queue_count_session(g_session_id) == 1, "S2: exactly ONE record for the whole saga");
    CHECK(rearm_delay_queue_peek_session(g_session_id, &rec), "S2: peek succeeds");
    CHECK((rec.reason_bits & REARM_DELAY_REASON_WAIT_LONG) != 0, "S2: WAIT_LONG reason kept");
    rearm_delay_queue_release_session(g_session_id);
    if (pas_trace_slot_ready(rslot)) pas_trace_slot_release(rslot);
    if (pas_raw_slot_ready()) pas_raw_slot_release();
}

/* ================================================================================================
 * S3 - the dynamically-chosen slot would be the ordinary active watcher: freeze slot 0 first (an
 * ordinary trigger), drive the active watcher to slot 1, THEN start a real reverse/rearm. prearm
 * must refuse rather than reuse the active slot - this is the exact structural regression proof
 * for the v3 bug (fixed index PAS_TRACE_SLOTS-1 could already be T.active).
 * =============================================================================================== */
static void test_s3_reserved_slot_would_be_ordinary_active(void)
{
    reset_all();
    /* Arm and freeze an ordinary capture on slot 0 with two illegal transitions before the ride
     * even starts (pas_trace has no session/latch dependency of its own). */
    inv(); inv();   /* the trigger write ITSELF counts as the first POST write (see pas_trace.h's
                     * ring model), so 2 calls already leave PAS_TRACE_POST-2 remaining */
    pump_invalid(PAS_TRACE_POST - 2U);   /* exactly enough to freeze slot 0, no overshoot into slot 1 */
    CHECK(pas_trace_slot_ready(0), "S3: slot 0 frozen by an ordinary trigger");
    CHECK(pas_trace_rearm_slot_index() == PAS_TRACE_SLOTS, "S3: no reservation yet");

    /* The active watcher has moved on to slot 1 automatically (freeze() hands off). One more
     * illegal transition proves slot 1, not slot 0, is now live. */
    inv();
    CHECK(g_last_trace_cap != PAS_TRACE_NO_CAPTURE, "S3: slot 1 is now the live ordinary watcher");

    establish_active();
    rev();
    CHECK(g_last_sout.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S3: real SUSPENDED_BY_DIRECTION");
    /* Both slots are unavailable: 0 is ready (unstreamed), 1 is the active ordinary watcher.
     * prearm MUST refuse, not silently pick either. */
    CHECK(!pas_trace_rearm_held(), "S3: prearm refuses - no slot free, neither reused nor stolen");
    CHECK(rearm_delay_prearm_edge(), "S3: the edge itself still fired correctly");

    /* The ordinary slot 0 capture must be completely untouched by the refused prearm attempt. */
    CHECK(pas_trace_slot_ready(0), "S3: ordinary slot 0 still ready, untouched");
    pas_trace_slot_release(0);
}

/* ================================================================================================
 * S4 - an ordinary capture arms WHILE direction is inhibited - must not corrupt the reservation.
 * =============================================================================================== */
static void test_s4_ordinary_capture_during_inhibit(void)
{
    reset_all();
    establish_active();
    rev();
    uint8_t rslot = pas_trace_rearm_slot_index();
    CHECK(rslot < PAS_TRACE_SLOTS, "S4: reservation held after the reverse");

    /* While still inhibited (before enough forward steps confirm), an illegal transition should
     * arm an ORDINARY capture on the OTHER slot - the reservation must be untouched. */
    inv();
    CHECK(!pas_trace_slot_ready(rslot), "S4: reservation not armed by the unrelated ordinary trigger");
    CHECK(pas_trace_rearm_held(), "S4: reservation still held");
    CHECK(pas_trace_rearm_slot_index() == rslot, "S4: same reserved slot, not disturbed");

    /* Run the ordinary capture all the way to its own freeze - THIS is what calls
     * find_free_slot() looking for a new active watcher. With PAS_TRACE_SLOTS=2 and one slot
     * reserved, the only other slot is the one that just froze, so find_free_slot() must find
     * NOTHING - never silently hand the reserved slot to ordinary use. */
    pump_invalid(PAS_TRACE_POST - 1U);   /* only ONE trigger call so far (the inv() above), unlike
                                           * the two-call setups elsewhere in this file */
    bool ordinary_became_ready = false;
    for (uint8_t i = 0; i < PAS_TRACE_SLOTS; i++) {
        if (i != rslot && pas_trace_slot_ready(i)) ordinary_became_ready = true;
    }
    CHECK(ordinary_became_ready, "S4: setup: the ordinary capture actually froze");
    CHECK(pas_trace_rearm_slot_index() == rslot, "S4: reservation still untouched after the ordinary freeze");
    CHECK(!pas_trace_slot_ready(rslot), "S4: reserved slot still not armed/ready");

    /* No slot is left for ordinary use now - a further suspicious transition must be COUNTED but
     * NOT captured, never silently steal the still-reserved slot to keep watching. */
    inv();
    CHECK(g_last_trace_cap == PAS_TRACE_NO_CAPTURE,
        "S4: no ordinary watcher available - the reserved slot is never silently reused for one");

    g_commit_iq_request = 100;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    CHECK(g_last_sout.state == RIDE_SESSION_ACTIVE, "S4: the saga still rearms normally");
}

/* ================================================================================================
 * S5 - the only other slot is still READY from an earlier, unstreamed rearm capture. prearm must
 * refuse rather than silently repurpose it (a direct history-corruption risk if it did not).
 * =============================================================================================== */
static void test_s5_reserved_slot_ready_from_earlier_capture(void)
{
    reset_all();
    establish_active();
    rev();
    hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);   /* WAIT_LONG -> capture armed */
    g_commit_iq_request = 100;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    CHECK(g_last_sout.state == RIDE_SESSION_ACTIVE, "S5: first saga rearms");
    pump_forward(PAS_TRACE_POST + 4U);
    uint8_t first_slot = 0xFFU;
    for (uint8_t i = 0; i < PAS_TRACE_SLOTS; i++) if (pas_trace_slot_ready(i)) first_slot = i;
    CHECK(first_slot < PAS_TRACE_SLOTS, "S5: first capture is ready and NOT streamed yet");

    /* v5: the first saga's ownership already ended at its own commit-and-recovery-conclude point
     * is NOT what is being tested here - this scenario is about a SECOND saga's prearm() finding
     * the ready capture already RETAINED (from whenever ownership last ended) with nowhere else
     * to go. Force that explicitly: end the first saga's ownership now (its capture is ready, so
     * it becomes RETAINED, not freed) before starting the second saga's reverse. */
    pas_trace_rearm_end_ownership();
    CHECK(!pas_trace_rearm_held(), "S5: first saga's ownership explicitly ended");
    CHECK(pas_trace_rearm_retained_slot_index() == first_slot,
        "S5: the ready capture is RETAINED, not simply forgotten");

    /* A second saga begins with the first capture still RETAINED, unstreamed. prearm() must
     * never silently repurpose it - with only PAS_TRACE_SLOTS=2 and the other slot being the
     * ordinary watcher, there is NO candidate left for the new saga at all: prearm() must refuse
     * outright (NOT silently point the new saga at the old, retained slot - the v4 defect this
     * card's Problem 3 fixes), and the earlier capture must stay completely untouched. */
    hold(4U);
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS + 1U; i++) fwd();   /* stay confirmed forward */
    rev();
    CHECK(g_last_sout.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S5: second saga's real reverse");
    CHECK(!pas_trace_rearm_held(), "S5: prearm refuses outright - no slot free for the new saga");
    CHECK(pas_trace_rearm_slot_index() == PAS_TRACE_SLOTS,
        "S5: the new saga owns NOTHING - never silently pointed at the old, retained slot");
    CHECK(pas_trace_rearm_retained_slot_index() == first_slot,
        "S5: the first saga's retained capture is undisturbed by the second saga's failed prearm");
    CHECK(pas_trace_slot_ready(first_slot), "S5: the earlier ready capture is completely untouched");

    /* Prove the new saga genuinely cannot obtain a fresh capture at all - not even a busy-refusal,
     * since nothing is held for it to be busy on. */
    pas_trace_input_t forced = {0};
    uint8_t attempt = pas_trace_rearm_capture(&forced);
    CHECK(attempt == PAS_TRACE_NO_CAPTURE, "S5: a capture attempt with nothing held is refused");

    pas_trace_slot_release(first_slot);
    pas_raw_slot_release();
}

/* ================================================================================================
 * S6 - healthy fast rearm, then a SECOND, delayed rearm: the second TRACE must contain none of the
 * first (healthy) event's samples - the clean-history-boundary requirement.
 * =============================================================================================== */
static void test_s6_no_stale_history_between_reservations(void)
{
    reset_all();
    establish_active();

    /* First saga: healthy, fast - commits well before any problem fires. Tag every sample this
     * saga's reservation dual-writes with a distinctive, otherwise-unused torque_raw_mv marker,
     * so a later leak is a content check, not just a structural one. */
    const uint16_t STALE_MARKER = 0xBEEFU;
    g_snap.raw_native = STALE_MARKER;
    rev();
    g_commit_iq_request = 100;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    CHECK(g_last_sout.state == RIDE_SESSION_ACTIVE, "S6: first saga healthy fast rearm rearms");
    /* v5 fix: the rearm edge alone does not end ownership - it is still held while RECOVERING. */
    CHECK(pas_trace_rearm_held(), "S6: reservation still held right after rearm (RECOVERING)");
    end_session_now();   /* recovery concludes: never armed, so ownership ends and frees immediately */
    CHECK(!pas_trace_rearm_held(), "S6: never-armed reservation released once recovery concludes, no orphan");
    CHECK(rearm_delay_queue_count_session(g_session_id) == 0, "S6: healthy fast rearm produces no record");

    /* Second saga: genuinely delayed this time, with a DIFFERENT torque marker. */
    establish_active();
    g_snap.raw_native = 0x1234U;
    rev();
    uint8_t rslot2 = pas_trace_rearm_slot_index();
    CHECK(rslot2 < PAS_TRACE_SLOTS, "S6: second saga gets a fresh reservation");
    hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);
    g_commit_iq_request = 100;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    pump_forward(PAS_TRACE_POST + 4U);
    end_session_now();

    CHECK(pas_trace_slot_ready(rslot2), "S6: second capture frozen");
    uint16_t n = pas_trace_slot_count(rslot2);
    bool saw_stale_marker = false;
    for (uint16_t i = 0; i < n; i++) {
        pas_trace_sample_t s;
        CHECK(pas_trace_slot_get(rslot2, i, &s), "S6: sample readable");
        if (s.torque_raw_mv == STALE_MARKER) saw_stale_marker = true;
    }
    CHECK(!saw_stale_marker, "S6: second capture contains none of the first (healthy) event's samples");
    CHECK(pas_trace_slot_session_id(rslot2) == g_session_id, "S6: second capture stamped with the current session");
    pas_trace_slot_release(rslot2);
    if (pas_raw_slot_ready()) pas_raw_slot_release();
}

/* ================================================================================================
 * S7 - two consecutive sessions: session B's capture carries no session A samples.
 * =============================================================================================== */
static void test_s7_session_boundary(void)
{
    reset_all();
    establish_active();
    rev();
    hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);
    g_commit_iq_request = 100;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    pump_forward(PAS_TRACE_POST + 4U);
    end_session_now();

    rearm_delay_record_t recA;
    CHECK(rearm_delay_queue_count_session(1U) == 1, "S7: session A record queued");
    CHECK(rearm_delay_queue_peek_session(1U, &recA), "S7: session A peek");
    CHECK(recA.session_id == 1U, "S7: session A record stamped session 1");
    uint8_t slotA = 0xFFU;
    for (uint8_t i = 0; i < PAS_TRACE_SLOTS; i++) if (pas_trace_slot_ready(i)) slotA = i;
    CHECK(slotA < PAS_TRACE_SLOTS, "S7: session A capture ready");
    CHECK(pas_trace_slot_session_id(slotA) == 1U, "S7: TRACE stamped session 1");
    pas_trace_slot_release(slotA);
    if (pas_raw_slot_ready()) pas_raw_slot_release();
    rearm_delay_queue_release_session(1U);

    /* Session B begins - a new session id, a fresh ride. */
    ride_session_force_cold();
    g_session_id = 2U;
    pas_trace_set_session_id(g_session_id);
    pas_raw_set_session_id(g_session_id);
    rearm_delay_set_session_id(g_session_id);
    establish_active();

    rev();
    hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);
    g_commit_iq_request = 100;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    pump_forward(PAS_TRACE_POST + 4U);
    end_session_now();

    rearm_delay_record_t recB;
    CHECK(rearm_delay_queue_count_session(2U) == 1, "S7: session B record queued");
    CHECK(rearm_delay_queue_peek_session(2U, &recB), "S7: session B peek");
    CHECK(recB.session_id == 2U, "S7: session B record stamped session 2");
    CHECK(rearm_delay_queue_count_session(1U) == 0, "S7: no session A record leaks into B's query");
    uint8_t slotB = 0xFFU;
    for (uint8_t i = 0; i < PAS_TRACE_SLOTS; i++) if (pas_trace_slot_ready(i)) slotB = i;
    CHECK(slotB < PAS_TRACE_SLOTS, "S7: session B capture ready");
    CHECK(pas_trace_slot_session_id(slotB) == 2U, "S7: session B TRACE stamped session 2, not 1");
    pas_trace_slot_release(slotB);
    if (pas_raw_slot_ready()) pas_raw_slot_release();
}

/* ================================================================================================
 * S8 - RAW busy: the record reports TRACE_ONLY, never FULL.
 * =============================================================================================== */
static void test_s8_raw_busy_trace_only(void)
{
    reset_all();
    /* pas_raw_isr_sample()'s very first-ever call only adopts the line state as a baseline and
     * records no event (nothing to compare it against yet) - a throwaway forward step first means
     * the trigger transition right after it is a genuine, recordable state change. */
    fwd();
    /* Occupy the single pas_raw slot with an unrelated, still-open capture. */
    inv();
    CHECK(g_last_trace_cap != PAS_TRACE_NO_CAPTURE, "S8: setup: an ordinary TRACE+RAW pair armed");
    CHECK(!pas_raw_slot_ready(), "S8: setup: RAW slot busy collecting POST, not yet ready");

    establish_active();
    rev();
    hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);
    g_commit_iq_request = 100;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();

    rearm_delay_record_t rec;
    CHECK(rearm_delay_queue_peek_session(g_session_id, &rec) || rearm_delay_fsm_state() != 0,
        "S8: record path reached");
    /* Real pedalling after the rearm drains the shadow capture's POST window. */
    pump_forward(PAS_TRACE_POST + 4U);
    end_session_now();
    CHECK(rearm_delay_queue_peek_session(g_session_id, &rec), "S8: record queued");
    CHECK(rec.capture_status == REARM_DELAY_CAPTURE_TRACE_ONLY, "S8: TRACE_ONLY, never FULL, when RAW is busy");
    CHECK(rec.capture_status != REARM_DELAY_CAPTURE_FULL, "S8: explicitly not FULL");
}

/* ================================================================================================
 * S9 - no free TRACE at the initiating event: exactly ONE increment of capture_slots_full, proven
 * against the real one-shot prearm_edge() (not a synthetic retry loop).
 * =============================================================================================== */
static void test_s9_no_free_trace_counted_once(void)
{
    reset_all();
    inv(); inv();
    pump_invalid(PAS_TRACE_POST - 2U);   /* exactly enough to freeze slot 0, no overshoot into slot 1 */
    inv();   /* slot 0 ready, slot 1 now the live ordinary watcher */

    uint16_t before = pas_trace_capture_slots_full();
    establish_active();
    rev();
    CHECK(!pas_trace_rearm_held(), "S9: prearm refuses - no free slot");
    uint16_t after_edge_tick = pas_trace_capture_slots_full();
    CHECK(after_edge_tick == before + 1U, "S9: exactly one increment on the one-shot prearm edge");

    /* The edge is one-shot: holding in the same state for many more ticks must not re-fire it or
     * increment the counter again - this is the real guarantee (not a synthetic call loop). */
    for (uint32_t i = 0; i < 500U; i++) {
        do_tick(EV_NONE);
        CHECK(!rearm_delay_prearm_edge(), "S9: prearm edge never re-fires without a fresh ACTIVE->SUSPENDED transition");
    }
    CHECK(pas_trace_capture_slots_full() == before + 1U, "S9: counter still exactly one higher after 500 idle ticks");

    pas_trace_slot_release(0);
}

/* ================================================================================================
 * S10 - session end during PREARM with no PROBLEM: the reservation is released, no orphaned hold.
 * =============================================================================================== */
static void test_s10_session_end_during_prearm(void)
{
    reset_all();
    establish_active();
    rev();
    uint8_t rslot = pas_trace_rearm_slot_index();
    CHECK(rslot < PAS_TRACE_SLOTS, "S10: reservation held after the reverse");
    CHECK(!pas_trace_slot_ready(rslot), "S10: never armed yet");

    /* A terminal cut ends the session right here, before any forward confirmation or PROBLEM. */
    g_terminal_cut = true;
    do_tick(EV_NONE);
    CHECK(g_last_sout.state == RIDE_SESSION_COLD, "S10: session ended (terminal cut)");
    CHECK(rearm_delay_ownership_end_edge(), "S10: ownership-end edge fires on the terminal SUSPENDED -> COLD");
    CHECK(!pas_trace_rearm_held(), "S10: reservation released, no orphaned hold");
    CHECK(!pas_trace_slot_ready(rslot), "S10: the never-armed slot is simply free again, not a stray capture");

    /* Prove it is genuinely free: a fresh ordinary trigger can arm it immediately - an armed-but-
     * collecting slot is not visible through slot_ready()/slot_count() (both only expose a frozen
     * capture, see pas_trace.h), so the trigger's own return value is the signal here. */
    g_terminal_cut = false;
    inv();
    CHECK(g_last_trace_cap != PAS_TRACE_NO_CAPTURE, "S10: the freed slot is usable by ordinary traffic right away");
}

/* ================================================================================================
 * S11 - session end during POST: TRACE and RAW both partial, same capture_id.
 * =============================================================================================== */
static void test_s11_session_end_during_post(void)
{
    reset_all();
    establish_active();
    fwd();   /* seed the RAW ring with a real pre-reverse event, see S1 */
    rev();
    uint8_t rslot = pas_trace_rearm_slot_index();
    hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);   /* PROBLEM fires, capture armed */
    CHECK(!pas_trace_slot_ready(rslot), "S11: capture armed, still collecting POST");
    /* pas_trace_slot_capture_id() only exposes a FROZEN slot's id by design (see pas_trace.h) - an
     * armed-but-collecting capture's id is not readable until it freezes, which sealing forces
     * below. cap_id is read AFTER sealing instead of here. */

    /* Session ends mid-POST (a terminal cut while still SUSPENDED). POST only advances on real
     * transitions (see pump_forward()'s own comment), so a plain hold leaves the capture exactly
     * where it was armed - still collecting, never ready - which is exactly the state this
     * scenario needs before sealing. */
    hold(20U);
    g_terminal_cut = true;
    do_tick(EV_NONE);
    /* Mirror main.c's own session-close sealing (unrelated to the v4 reservation mechanism). */
    pas_trace_seal_open_captures();
    pas_raw_seal_open_capture();

    CHECK(pas_trace_slot_ready(rslot), "S11: sealed TRACE is now readable");
    CHECK(pas_trace_slot_partial(rslot), "S11: TRACE flagged partial");
    uint8_t cap_id = pas_trace_slot_capture_id(rslot);
    CHECK(cap_id != PAS_TRACE_NO_CAPTURE, "S11: a real capture id assigned");
    CHECK(pas_raw_slot_ready(), "S11: sealed RAW is now readable");
    CHECK(pas_raw_slot_partial(), "S11: RAW flagged partial");
    CHECK(pas_raw_slot_capture_id() == cap_id, "S11: RAW shares the SAME capture_id as TRACE");

    pas_trace_slot_release(rslot);
    pas_raw_slot_release();
}

/* ================================================================================================
 * S12 - uint32_t control-tick wraparound across the whole reservation cycle.
 * =============================================================================================== */
static void test_s12_tick_wraparound(void)
{
    reset_all();
    g_tick = 0xFFFFFFF0U;
    establish_active();
    fwd();   /* seed the RAW ring with a real pre-reverse event, see S1 */
    rev();
    uint8_t rslot = pas_trace_rearm_slot_index();
    CHECK(rslot < PAS_TRACE_SLOTS, "S12: reservation held, straddling the coming wrap");
    hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);   /* this hold is what actually crosses 0xFFFFFFFF */
    CHECK(g_tick < 0xFFFFFFF0U, "S12: g_tick actually wrapped during this test");
    g_commit_iq_request = 100;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    CHECK(g_last_sout.state == RIDE_SESSION_ACTIVE, "S12: rearm still resolves correctly across the wrap");
    pump_forward(PAS_TRACE_POST + 4U);
    end_session_now();

    rearm_delay_record_t rec;
    CHECK(rearm_delay_queue_peek_session(g_session_id, &rec), "S12: record queued despite the wrap");
    CHECK(rec.capture_status == REARM_DELAY_CAPTURE_FULL, "S12: still a clean FULL pair across the wrap");
    CHECK(pas_trace_slot_ready(rslot), "S12: TRACE frozen correctly across the wrap");
    pas_trace_slot_release(rslot);
    if (pas_raw_slot_ready()) pas_raw_slot_release();
}

/* ================================================================================================
 * FW-111 v5 SCENARIOS. v4's release-at-commit and R.cur_slot-after-close bugs are fixed by
 * decoupling reservation ownership from both the COMMIT tick and this record's own queue slot -
 * these scenarios prove the fix against the REAL chain, the same way S1-S12 above do for v4's.
 * ================================================================================================
 *
 * S13 - fast REARM, then WEAK_TARGET detected >150 ms AFTER the rearm (the case closest to the
 * user's original symptom: the latch returns, but a real Iq only appears after a harder push).
 * The reservation must still be held then; the record must never report NO_TRACE_NO_HISTORY when
 * PREARM actually got a slot; TRACE must contain both the initiating reverse and the
 * forward-confirming steps.
 * =============================================================================================== */
static void test_s13_weak_target_after_rearm(void)
{
    reset_all();
    establish_active();
    g_commit_iq_request = 200;
    do_tick(EV_NONE);   /* ACTIVE, latched, fsm IDLE - this tick's iq_pre_ramp becomes pre_reverse_iq */

    rev();
    CHECK(g_last_sout.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S13: real SUSPENDED_BY_DIRECTION");
    uint8_t rslot = pas_trace_rearm_slot_index();
    CHECK(rslot < PAS_TRACE_SLOTS, "S13: reservation held after the reverse");

    /* A healthy-looking rearm - good enough Iq to actually latch (still 200, matching the
     * baseline). */
    g_commit_iq_request = 200;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    CHECK(g_last_sout.state == RIDE_SESSION_ACTIVE, "S13: rearms");
    CHECK(pas_trace_rearm_held(), "S13: reservation still held right after the rearm");
    CHECK(pas_trace_rearm_slot_index() == rslot, "S13: same slot, ownership unbroken by the rearm tick");

    /* NOW the target goes weak - well below 80% of the 200-native baseline - and stays weak for
     * more than the 150 ms continuous window, well AFTER commit. WEAK_TARGET's timer counts
     * elapsed CONTROL TICKS, not transitions, so this waits with hold() (no quadrature activity)
     * rather than pump_forward() - at REAL cadence, 150 ms is only a couple of PAS transitions,
     * nowhere near enough to push the 256-deep ring past the initiating reverse; pumping one
     * transition per control tick here would be unrealistically dense and evict the PRE history
     * that S13 exists to prove survives. */
    g_commit_iq_request = 50;   /* 25% of the 200 baseline */
    hold((CONTROL_TIMEBASE_HZ / 1000U) * 200U);   /* > 150 ms elapsed, weak throughout, no transitions */

    rearm_delay_record_t rec;
    CHECK(rearm_delay_queue_count_session(g_session_id) == 0,
        "S13: record still open in RECOVERING - WEAK_TARGET alone does not close it");
    CHECK(rearm_delay_fsm_state() == 2, "S13: FSM still RECOVERING");

    /* The capture is armed now (WEAK_TARGET fired during the hold) but still needs its POST
     * window - THAT part is fine as real, close-together pedalling. */
    pump_forward(PAS_TRACE_POST + 4U);
    end_session_now();   /* recovery concludes - now the record closes and ownership ends */
    CHECK(rearm_delay_ownership_end_edge(), "S13: ownership-end edge on recovery's real conclusion");

    CHECK(rearm_delay_queue_count_session(g_session_id) == 1, "S13: record queued");
    CHECK(rearm_delay_queue_peek_session(g_session_id, &rec), "S13: peek succeeds");
    CHECK((rec.reason_bits & REARM_DELAY_REASON_WEAK_TARGET) != 0, "S13: WEAK_TARGET reason bit set");
    CHECK(rec.capture_status == REARM_DELAY_CAPTURE_FULL || rec.capture_status == REARM_DELAY_CAPTURE_TRACE_ONLY,
        "S13: capture FULL or TRACE_ONLY - a real trace exists");
    CHECK(rec.capture_status != REARM_DELAY_CAPTURE_NO_TRACE_NO_HISTORY,
        "S13: never NO_TRACE_NO_HISTORY when PREARM actually held a slot for this saga");
    CHECK(rec.capture_id != REARM_DELAY_NO_CAPTURE, "S13: a real capture id stored");

    CHECK(pas_trace_slot_ready(rslot), "S13: TRACE frozen");
    CHECK(pas_trace_slot_capture_id(rslot) == rec.capture_id, "S13: record and TRACE share the same capture_id");
    uint16_t n = pas_trace_slot_count(rslot);
    bool saw_reverse = false;
    uint8_t forward_count = 0;
    for (uint16_t i = 0; i < n; i++) {
        pas_trace_sample_t s;
        CHECK(pas_trace_slot_get(rslot, i, &s), "S13: PRE sample readable");
        if ((s.flags & PAS_TR_REVERSE) != 0) saw_reverse = true;
        else forward_count++;
    }
    CHECK(saw_reverse, "S13: TRACE contains the initiating reverse step");
    CHECK(forward_count >= PAS_REVERSE_RECOVERY_CONFIRM_STEPS, "S13: TRACE contains the forward-confirming steps");

    pas_trace_slot_release(rslot);
    if (pas_raw_slot_ready()) pas_raw_slot_release();
}

/* ================================================================================================
 * S14 - healthy rearm: no record kept, and NO orphaned reservation once recovery concludes (not
 * "immediately at commit" - the v4 bug this card fixes).
 * =============================================================================================== */
static void test_s14_healthy_rearm_no_orphan(void)
{
    reset_all();
    establish_active();
    g_commit_iq_request = 100;
    do_tick(EV_NONE);   /* ACTIVE, latched, fsm IDLE - this tick's iq_pre_ramp becomes pre_reverse_iq */
    rev();
    g_commit_iq_request = 100;   /* stays healthy - iq_pre_ramp == pre_reverse_iq, never weak */
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    CHECK(g_last_sout.state == RIDE_SESSION_ACTIVE, "S14: healthy fast rearm rearms");
    CHECK(pas_trace_rearm_held(), "S14: reservation still held right after the rearm (RECOVERING)");

    hold(100U);
    CHECK(pas_trace_rearm_held(), "S14: reservation still held mid-RECOVERING, nothing wrong yet");

    end_session_now();
    CHECK(!pas_trace_rearm_held(), "S14: reservation released once recovery concludes - no orphan");
    CHECK(rearm_delay_queue_count_session(g_session_id) == 0, "S14: no record kept for a healthy rearm");
}

/* ================================================================================================
 * S15 - a new reverse during RECOVERING, previous ownership's slot NEVER ARMED: old ownership ends
 * cleanly (freed, not retained), a fresh PREARM claims a slot, seeded from the NEW reverse, with
 * no samples surviving from the first saga.
 * =============================================================================================== */
static void test_s15_reverse_during_recovering_old_slot_unarmed(void)
{
    reset_all();
    establish_active();
    const uint16_t FIRST_MARKER = 0xAAAAU;
    g_snap.raw_native = FIRST_MARKER;
    rev();
    uint8_t first_slot = pas_trace_rearm_slot_index();
    CHECK(first_slot < PAS_TRACE_SLOTS, "S15: first saga's reservation held");
    g_commit_iq_request = 100;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    CHECK(g_last_sout.state == RIDE_SESSION_ACTIVE, "S15: first saga rearms (healthy, never armed)");
    CHECK(pas_trace_rearm_slot_index() == first_slot, "S15: still owns the same slot while RECOVERING");
    CHECK(!pas_trace_slot_ready(first_slot), "S15: first slot never armed - still just PRE history");

    /* A NEW reverse interrupts RECOVERING before the first saga ever concluded on its own. */
    const uint16_t SECOND_MARKER = 0x5555U;
    g_snap.raw_native = SECOND_MARKER;
    rev();
    CHECK(g_last_sout.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S15: the real automaton reacts to the new reverse");
    CHECK(rearm_delay_prearm_edge(), "S15: prearm edge fires - ACTIVE -> SUSPENDED again");

    uint8_t second_slot = pas_trace_rearm_slot_index();
    CHECK(second_slot < PAS_TRACE_SLOTS, "S15: a fresh reservation was claimed for the new saga");
    CHECK(pas_trace_rearm_retained_slot_index() == PAS_TRACE_SLOTS,
        "S15: nothing retained - the old slot was never armed, so it was freed, not kept");
    /* With PAS_TRACE_SLOTS == 2, the freed old slot IS the only candidate, so it is normal for
     * second_slot == first_slot here - the point is it was properly RESET first (checked below),
     * not that a different physical index was used. */

    /* An unarmed slot's samples are not readable through the public API (slot_ready() gates it -
     * see pas_trace.h) - force a trigger, exactly what a real PROBLEM would do, then let it
     * complete its POST window before reading. */
    pas_trace_input_t forced = {0};
    uint8_t cap = pas_trace_rearm_capture(&forced);
    CHECK(cap != PAS_TRACE_NO_CAPTURE, "S15: setup: forced capture armed the fresh reservation");
    pump_forward(PAS_TRACE_POST + 4U);
    CHECK(pas_trace_slot_ready(second_slot), "S15: setup: capture frozen");

    uint16_t n = pas_trace_slot_count(second_slot);
    bool saw_first_marker = false;
    bool saw_second_marker = false;
    for (uint16_t i = 0; i < n; i++) {
        pas_trace_sample_t s;
        CHECK(pas_trace_slot_get(second_slot, i, &s), "S15: sample readable");
        if (s.torque_raw_mv == FIRST_MARKER) saw_first_marker = true;
        if (s.torque_raw_mv == SECOND_MARKER) saw_second_marker = true;
    }
    CHECK(!saw_first_marker, "S15: no sample survives from the first saga after the reset+reseed");
    CHECK(saw_second_marker, "S15: the new slot is seeded from the NEW reverse");

    pas_trace_slot_release(second_slot);
    if (pas_raw_slot_ready()) pas_raw_slot_release();

    g_snap.raw_native = 0;
    g_commit_iq_request = 100;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    end_session_now();
    if (pas_trace_slot_ready(second_slot)) pas_trace_slot_release(second_slot);
    if (pas_raw_slot_ready()) pas_raw_slot_release();
}

/* ================================================================================================
 * S16 - a new reverse during RECOVERING, previous ownership's capture is ARMED/READY: the old
 * capture is completely untouched, the new saga does NOT take over the old slot, and - with no
 * other slot free - gets an explicit NO_TRACE_NO_HISTORY, never a fabricated BUSY that would
 * actually describe the OLD saga.
 * =============================================================================================== */
static void test_s16_reverse_during_recovering_old_slot_armed(void)
{
    reset_all();
    establish_active();
    rev();
    uint8_t first_slot = pas_trace_rearm_slot_index();
    hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);   /* WAIT_LONG -> first saga's capture ARMED */
    g_commit_iq_request = 100;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    CHECK(g_last_sout.state == RIDE_SESSION_ACTIVE, "S16: first saga rearms with an armed capture pending");
    CHECK(!pas_trace_slot_ready(first_slot), "S16: first slot armed, still collecting its own POST");
    /* An armed-but-not-ready slot's capture_id is not exposed through the public API (it only
     * reveals a FROZEN slot's identity, by design - see pas_trace.h) - the "undisturbed" check
     * below instead confirms the id stays REAL and stable once the slot DOES freeze. */

    /* A new reverse interrupts RECOVERING while the first saga's capture is still armed. */
    rev();
    CHECK(g_last_sout.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S16: the real automaton reacts");
    CHECK(rearm_delay_prearm_edge(), "S16: prearm edge fires for the new saga");
    CHECK(!pas_trace_slot_ready(first_slot) || pas_trace_slot_capture_id(first_slot) != PAS_TRACE_NO_CAPTURE,
        "S16: setup sanity - the old slot's identity is still traceable");

    /* No candidate is free for the new saga: the other slot is the ordinary watcher, and this one
     * is retained (armed) from the old, unfinished saga. */
    CHECK(!pas_trace_rearm_held(), "S16: new saga gets no reservation - nothing free");
    CHECK(pas_trace_rearm_slot_index() == PAS_TRACE_SLOTS, "S16: new saga owns nothing");
    CHECK(pas_trace_rearm_retained_slot_index() == first_slot,
        "S16: the old, armed capture is RETAINED, not silently taken over");

    /* Let the old capture's POST complete - it must still receive it, undisturbed. Note: these
     * forward steps ALSO drive the new saga's own real rearm (SUSPENDED -> ACTIVE) - the point of
     * this scenario is specifically that this activity must not touch the OLD, retained slot's
     * content at all. */
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    pump_forward(PAS_TRACE_POST + 4U);
    CHECK(pas_trace_slot_ready(first_slot), "S16: the OLD capture still completed its full POST window");
    CHECK(pas_trace_slot_capture_id(first_slot) != PAS_TRACE_NO_CAPTURE,
        "S16: the old capture has a real, stable identity once frozen - never touched by the new saga");

    pas_trace_slot_release(first_slot);
    if (pas_raw_slot_ready()) pas_raw_slot_release();

    /* The new saga got NO reservation (nothing free), so when ITS OWN problem finally fires, its
     * record must report NO_TRACE_NO_HISTORY - the distinct "PREARM never found a free slot"
     * outcome, never a fabricated BUSY that would actually describe the OLD saga. NO_LOAD fires
     * after REARM_DELAY_NO_LOAD_MS with no pedal load (the harness's snapshot is all zeros). */
    hold(REARM_DELAY_NO_LOAD_TICKS + 1U);
    end_session_now();
    {
        rearm_delay_record_t rec;
        bool found = false;
        while (rearm_delay_queue_peek_session(g_session_id, &rec)) {
            if (rec.capture_status == REARM_DELAY_CAPTURE_NO_TRACE_NO_HISTORY) {
                found = true;
                CHECK((rec.reason_bits & REARM_DELAY_REASON_NO_LOAD) != 0,
                    "S16: the new saga's own problem fired (NO_LOAD) with nothing held");
            }
            rearm_delay_queue_release_session(g_session_id);
        }
        CHECK(found, "S16: the new saga's record reports NO_TRACE_NO_HISTORY, never a fabricated BUSY");
    }
}

/* ================================================================================================
 * S17 - re-suspend AFTER WAIT_LONG already armed the capture: the armed capture survives the
 * re-suspend, still ONE record and ONE reservation - never a second record, never a re-arm.
 * S18 - terminal SUSPENDED_BY_DIRECTION -> COLD before WAIT_LONG (NO_PERMISSION): the record
 * closes in the SAME tick its trigger fires. The trigger must not be lost, and the QUEUED record
 * must receive its capture_id/status; the armed slot becomes RETAINED, not silently dropped.
 * =============================================================================================== */
static void test_s17_resuspend_after_armed_capture(void)
{
    reset_all();
    establish_active();
    rev();
    uint8_t rslot = pas_trace_rearm_slot_index();
    CHECK(rslot < PAS_TRACE_SLOTS, "S17: reservation held after the reverse");

    /* Let WAIT_LONG fire first - the capture is now ARMED. */
    hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);
    CHECK(!pas_trace_slot_ready(rslot), "S17: capture armed, still collecting POST");

    /* A re-suspend AFTER the capture armed: the real automaton simply stays SUSPENDED - the armed
     * capture survives, still ONE record and ONE reservation, never a second record and never a
     * re-arm (the v1 WAIT<->SUSPENDED oscillation is gone). */
    rev();
    CHECK(g_last_sout.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S17: real re-suspend");
    CHECK(!rearm_delay_prearm_edge(), "S17: re-suspend is NOT a fresh prearm edge");
    CHECK(rearm_delay_fsm_state() == 1, "S17: the SAME record stays open - never a second one");
    CHECK(pas_trace_rearm_held(), "S17: reservation still held");
    CHECK(pas_trace_rearm_slot_index() == rslot, "S17: same slot - ownership never broke");
    CHECK(!pas_trace_slot_ready(rslot), "S17: the armed capture was not disturbed by the re-suspend");

    /* Finish the saga: confirm, complete the POST window, conclude recovery. */
    g_commit_iq_request = 100;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    pump_forward(PAS_TRACE_POST + 4U);
    end_session_now();

    /* Exactly ONE record for the whole saga, with the WAIT_LONG capture fully armed. */
    rearm_delay_record_t rec;
    CHECK(rearm_delay_queue_count_session(g_session_id) == 1, "S17: exactly ONE record - the re-suspend opened nothing");
    CHECK(rearm_delay_queue_peek_session(g_session_id, &rec), "S17: peek succeeds");
    CHECK((rec.reason_bits & REARM_DELAY_REASON_WAIT_LONG) != 0, "S17: WAIT_LONG reason kept");
    CHECK(rec.capture_id != REARM_DELAY_NO_CAPTURE, "S17: the single capture got a real id");
    rearm_delay_queue_release_session(g_session_id);
    if (pas_trace_slot_ready(rslot)) pas_trace_slot_release(rslot);
    if (pas_raw_slot_ready()) pas_raw_slot_release();
}

static void test_s18_same_tick_close_terminal_gets_capture(void)
{
    reset_all();
    establish_active();
    rev();
    uint8_t rslot = pas_trace_rearm_slot_index();
    CHECK(rslot < PAS_TRACE_SLOTS, "S18: reservation held after the reverse");

    /* A terminal cut before WAIT_LONG: NO_PERMISSION closes the record AND concludes the saga in
     * the SAME tick the trigger fires. */
    g_terminal_cut = true;
    do_tick(EV_NONE);
    g_terminal_cut = false;
    CHECK(g_last_sout.state == RIDE_SESSION_COLD, "S18: terminal cut reaches COLD");
    CHECK(rearm_delay_fsm_state() == 0, "S18: record closed this same tick");
    CHECK(rearm_delay_ownership_end_edge(), "S18: ownership-end edge also fires this same tick (terminal)");
    /* main.c's ordering runs capture BEFORE ownership-end (see do_tick()'s own comment) - the
     * terminal NO_PERMISSION trigger is this saga's FIRST ever problem, against a fresh, never-armed
     * reservation, so it succeeds and arms rslot BEFORE ownership ends this same tick. Ownership
     * ending afterwards then finds an ARMED slot, so it is RETAINED, not simply freed. */
    CHECK(!pas_trace_rearm_held(), "S18: reservation ownership released - the saga truly ended");
    CHECK(pas_trace_rearm_retained_slot_index() == rslot,
        "S18: the slot is RETAINED (armed by this same tick's capture), not silently dropped");

    rearm_delay_record_t rec;
    CHECK(rearm_delay_queue_peek_session(g_session_id, &rec), "S18: record queued");
    CHECK((rec.reason_bits & REARM_DELAY_REASON_NO_PERMISSION) != 0, "S18: NO_PERMISSION reason");
    CHECK(rec.capture_id != REARM_DELAY_NO_CAPTURE,
        "S18: the queued record got a REAL capture_id - the trigger was not lost at close");
    CHECK(rec.capture_status != REARM_DELAY_CAPTURE_NONE,
        "S18: the queued record's capture_status was actually written, not left at the NONE placeholder");

    pump_forward(PAS_TRACE_POST + 4U);
    if (pas_trace_slot_ready(rslot)) pas_trace_slot_release(rslot);
    if (pas_raw_slot_ready()) pas_raw_slot_release();
}

/* ================================================================================================
 * S19 - record queue full: the pas_trace reservation must not be orphaned even though no record
 * ever tracks this saga.
 * =============================================================================================== */
static void test_s19_queue_full_no_orphan(void)
{
    reset_all();
    establish_active();

    /* Fill the record queue with two kept records from two ordinary delayed sagas. */
    for (int saga = 0; saga < 2; saga++) {
        rev();
        hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);
        g_commit_iq_request = 100;
        for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
        pump_forward(PAS_TRACE_POST + 4U);
        end_session_now();
        establish_active();
    }
    CHECK(rearm_delay_queue_count_session(g_session_id) == 2, "S19: setup: queue full with 2 records");

    /* A third saga: prearm still succeeds (pas_trace has its own, independent slot budget), but
     * its record cannot open (queue full) - fsm stays IDLE the whole time. */
    rev();
    CHECK(rearm_delay_prearm_edge(), "S19: prearm edge still fires - independent of the record queue");
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    CHECK(rearm_delay_fsm_state() == 0, "S19: no record tracks this saga - queue is full");
    CHECK(g_last_sout.state == RIDE_SESSION_ACTIVE, "S19: the saga still rearms normally");
    /* No record ever tracked this saga, so record_truly_concluded can never fire for it - only
     * the fallback (no record was tracking, but the saga plainly concluded) can end ownership. */
    CHECK(rearm_delay_ownership_end_edge(), "S19: ownership-end edge fires via the no-record fallback, right at the rearm");
    CHECK(!pas_trace_rearm_held(), "S19: reservation not orphaned even though its record was refused");

    /* Drain the queue so later scenarios (if any share state) are unaffected. */
    rearm_delay_record_t rec;
    while (rearm_delay_queue_peek_session(g_session_id, &rec)) {
        rearm_delay_queue_release_session(g_session_id);
    }
}

/* ================================================================================================
 * S20 - two sessions, driven through the FULL v5 ownership lifecycle (PREARM..RECOVERING..end):
 * no session_id or sample mixing.
 * =============================================================================================== */
static void test_s20_two_sessions_full_lifecycle(void)
{
    reset_all();
    establish_active();
    rev();
    uint8_t slotA = pas_trace_rearm_slot_index();
    hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);
    g_commit_iq_request = 100;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    pump_forward(PAS_TRACE_POST + 4U);
    end_session_now();
    CHECK(pas_trace_slot_session_id(slotA) == 1U, "S20: session A's capture stamped session 1");
    rearm_delay_record_t recA;
    CHECK(rearm_delay_queue_peek_session(1U, &recA), "S20: session A record present");
    CHECK(recA.session_id == 1U, "S20: session A record stamped session 1");
    pas_trace_slot_release(slotA);
    if (pas_raw_slot_ready()) pas_raw_slot_release();
    rearm_delay_queue_release_session(1U);

    g_session_id = 2U;
    pas_trace_set_session_id(g_session_id);
    pas_raw_set_session_id(g_session_id);
    rearm_delay_set_session_id(g_session_id);
    establish_active();
    rev();
    uint8_t slotB = pas_trace_rearm_slot_index();
    hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);
    g_commit_iq_request = 100;
    for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd();
    pump_forward(PAS_TRACE_POST + 4U);
    end_session_now();

    CHECK(pas_trace_slot_session_id(slotB) == 2U, "S20: session B's capture stamped session 2, not 1");
    rearm_delay_record_t recB;
    CHECK(rearm_delay_queue_peek_session(2U, &recB), "S20: session B record present");
    CHECK(recB.session_id == 2U, "S20: session B record stamped session 2");
    CHECK(rearm_delay_queue_count_session(1U) == 0, "S20: no session A record leaks into session B's query");
    pas_trace_slot_release(slotB);
    if (pas_raw_slot_ready()) pas_raw_slot_release();
}

/* ================================================================================================
 * S21 - an armed shadow, after its OWNERSHIP has ended (retained), must still receive its
 * complete POST window, and the ordinary watcher must never be handed that slot.
 * =============================================================================================== */
static void test_s21_retained_shadow_still_completes_post(void)
{
    reset_all();

    /* Give the ORDINARY watcher (slot 0) a head start, well before the saga even begins: arm it
     * and run its POST most of the way down. This is what lets it complete (and call
     * find_free_slot()) WHILE the later, freshly-armed retained slot still has most of its own
     * POST left to collect - otherwise, fed the identical transition stream from the moment both
     * are alive, the two would always reach "ready" together, which cannot distinguish correct
     * exclusion from the retained-slot bug (a READY slot is excluded either way). */
    inv();
    CHECK(g_last_trace_cap != PAS_TRACE_NO_CAPTURE, "S21: setup: the ordinary watcher (slot 0) arms first");
    pump_invalid(118U);   /* 127 remaining after arming, minus 118 = 9 left */

    establish_active();
    rev();                                             /* 1 more ordinary transition before the */
    uint8_t rslot = pas_trace_rearm_slot_index();       /* saga's own slot exists - down to 8 left */
    CHECK(rslot != 0U, "S21: setup: the saga's reservation landed on the OTHER slot, not the busy one");
    hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);   /* WAIT_LONG -> capture armed (127 remaining), still collecting POST */
    CHECK(!pas_trace_slot_ready(rslot), "S21: armed, not yet ready");

    /* End ownership explicitly WHILE still armed (mid-POST) - simulates the record closing (e.g.
     * via its own internal TIMEOUT) before the shadow has finished collecting. */
    pas_trace_rearm_end_ownership();
    CHECK(!pas_trace_rearm_held(), "S21: ownership ended");
    CHECK(pas_trace_rearm_retained_slot_index() == rslot, "S21: the still-armed slot is RETAINED");

    /* Ordinary traffic must never be handed this slot while it is retained and still collecting. */
    pump_forward(3U);
    CHECK(!pas_trace_slot_ready(rslot), "S21: still collecting - not done yet");
    CHECK(pas_trace_slot_count(rslot) == 0, "S21: an armed (not ready) slot reports no readable count yet");

    /* The sharper proof: exactly enough MORE forward pumps to complete the ORDINARY watcher's
     * OWN head-started POST (8 left after the reverse, minus the 3 just pumped = 5 left) - NOT
     * rslot's (124 remaining at this point) - so find_free_slot() runs precisely while rslot is
     * still genuinely mid-collection. This is the only moment the retained-slot exclusion bug is
     * observable: once a slot is READY, the ordinary !ready check alone already excludes it,
     * correct code or not. */
        uint16_t slots_full_before = pas_trace_capture_slots_full();
    pump_forward(5U);
    CHECK(!pas_trace_slot_ready(rslot), "S21: rslot still armed, not ready, exactly when the ordinary watcher completes");
    /* If find_free_slot() incorrectly handed the retained slot to become the new active watcher,
     * a further suspicious transition would land ON it (already armed, so no NEW trigger, but
     * also NOT counted as "no watcher available"). If it correctly found nothing, the same
     * transition is counted via capture_slots_full instead - strictly MORE than before. (The
     * transition itself and, separately, the NEW saga's prearm that this reverse also starts both
     * count a full-budget refusal, so it grows by two, never by zero.) */
    inv();
    CHECK(pas_trace_capture_slots_full() > slots_full_before,
        "S21: no ordinary watcher left after the freeze - counted as slot-budget-full, never silently handed the retained slot");

    pump_forward(PAS_TRACE_POST);   /* finish rslot off */
    CHECK(pas_trace_slot_ready(rslot), "S21: the retained shadow completed its FULL POST window regardless");
    CHECK(pas_trace_rearm_retained_slot_index() == rslot, "S21: still retained - not yet streamed");

    pas_trace_slot_release(rslot);
    CHECK(pas_trace_rearm_retained_slot_index() == PAS_TRACE_SLOTS,
        "S21: retained slot finally cleared once actually streamed");
    if (pas_raw_slot_ready()) pas_raw_slot_release();
}

/* ================================================================================================
 * S22 - requirement 8: capture_id and session_id always refer to the event that STARTED the
 * reservation - a diag session boundary crossed WHILE the reservation is open (between PREARM and
 * the eventual capture) must never re-stamp the capture with the NEW session.
 * =============================================================================================== */
static void test_s22_session_id_fixed_at_prearm_not_capture(void)
{
    reset_all();
    establish_active();
    rev();
    uint8_t rslot = pas_trace_rearm_slot_index();
    CHECK(rslot < PAS_TRACE_SLOTS, "S22: setup: reservation held under session 1");

    /* The diag session boundary moves WHILE the reservation is still open, well before the
     * eventual PROBLEM/trigger - a real scenario if the bike sits at a brief standstill between
     * the reverse and the diagnostic recorder's own WAIT_LONG threshold. */
    g_session_id = 9U;
    pas_trace_set_session_id(g_session_id);
    pas_raw_set_session_id(g_session_id);
    rearm_delay_set_session_id(g_session_id);

    hold((CONTROL_TIMEBASE_HZ / 1000U) * 250U + 1U);   /* WAIT_LONG -> capture NOW, under session 9 */
    pump_forward(PAS_TRACE_POST + 4U);
    CHECK(pas_trace_slot_ready(rslot), "S22: capture frozen");
    CHECK(pas_trace_slot_session_id(rslot) == 1U,
        "S22: TRACE session_id is fixed at PREARM time (session 1) - never re-stamped by the later, current session 9");

    pas_trace_slot_release(rslot);
    if (pas_raw_slot_ready()) pas_raw_slot_release();
    /* Restore session 1 bookkeeping so any later scenario (if run after this one) is unaffected. */
    g_session_id = 1U;
}

int main(void)
{
	printf("FW-111 v5.1 / FW-112 v2 full-chain integration: pas_quadrature/pas_direction/ride_session/rearm_delay_diag/pas_trace/pas_raw, real modules\n");

    test_quadrature_ring_matches_decoder();
    test_s1_full_chain_full_capture();
    test_s2_resuspend_mid_saga_single_record();
    test_s3_reserved_slot_would_be_ordinary_active();
    test_s4_ordinary_capture_during_inhibit();
    test_s5_reserved_slot_ready_from_earlier_capture();
    test_s6_no_stale_history_between_reservations();
    test_s7_session_boundary();
    test_s8_raw_busy_trace_only();
    test_s9_no_free_trace_counted_once();
    test_s10_session_end_during_prearm();
    test_s11_session_end_during_post();
    test_s12_tick_wraparound();
    test_s13_weak_target_after_rearm();
    test_s14_healthy_rearm_no_orphan();
    test_s15_reverse_during_recovering_old_slot_unarmed();
    test_s16_reverse_during_recovering_old_slot_armed();
    test_s17_resuspend_after_armed_capture();
    test_s18_same_tick_close_terminal_gets_capture();
    test_s19_queue_full_no_orphan();
    test_s20_two_sessions_full_lifecycle();
    test_s21_retained_shadow_still_completes_post();
    test_s22_session_id_fixed_at_prearm_not_capture();

    if (host_test_failures != 0) {
        printf("rearm_trace_raw_integration_host: %d FAILURES\n", host_test_failures);
        return 1;
    }
    printf("rearm_trace_raw_integration_host: ALL PASS\n");
    return 0;
}
