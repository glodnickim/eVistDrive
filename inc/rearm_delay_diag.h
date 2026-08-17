#ifndef REARM_DELAY_DIAG_H_
#define REARM_DELAY_DIAG_H_

#include <stdbool.h>
#include <stdint.h>

#include "config.h"        /* CONTROL_TIMEBASE_HZ - constants only, no MS/MP */
#include "torque_input.h"  /* torque_snapshot_t, by value/pointer only - no other dependency */

/*
 * FW-111: the delayed-rearm diagnostic recorder.
 *
 * WHY IT EXISTS. The ride log showed assistance resuming only after a STRONGER pedal push after
 * a forward direction was re-confirmed after reverse/invalid. What the log could not say is
 * WHICH stage of the chain spent the extra time - the re-zero/offset, the 35 ms assist filter,
 * the assist-mode calculation, the min-Iq latch, the limiter/ramp, or the ~25 ms PWM/Hall start
 * (see documentation/FW-111_..._PL.md for the full A..F hypothesis list). This module records a
 * compact snapshot of the whole torque->Iq chain plus the elapsed time of every stage, ONLY when
 * the wait was actually suspicious - a normal ~4 ms fast rearm is measured but discarded, so it
 * never fills the buffer. The pre-reverse baseline is the demand of the LAST active tick, not the
 * ACTIVE maximum (a max could fabricate a WEAK_TARGET). The Hall start is measured by its own
 * enter/exit tick markers rather than a flag that no tick ever sees (the 25 ms hold is a blocking
 * delay, so no control tick - and therefore no snapshot - ever runs inside it).
 *
 * WHAT IT MUST NOT DO. This is a measurement, not a fix: it never changes any decision in the
 * assist pipeline. It reads the session automaton's STATE as a plain byte (the caller passes it
 * in - this module is deliberately NOT linked against ride_session.c, exactly the discipline
 * ride_session.h describes for ride_control), reads torque_input's published snapshot, and
 * records. Zero control impact.
 *
 * WHEN A RECORD IS KEPT (any of these, OR-ed into reason_bits):
 *   1. REARM_DELAY_REASON_WAIT_LONG    the SUSPENDED_BY_DIRECTION hold (the rearm delay itself)
 *                                      lasted > 200 ms.
 *   2. REARM_DELAY_REASON_WEAK_TARGET  the rearm edge fired but the target stayed below 80 % of
 *                                      the pre-reverse Iq for > 150 ms CONTINUOUSLY - this can fire
 *                                      well AFTER the rearm edge, while the record is in
 *                                      RECOVERING (see TRACE/RAW RESERVATION below for why the
 *                                      reservation must still be alive then).
 *   3. REARM_DELAY_REASON_NO_PERMISSION the saga ended (stop / re-inhibit / timeout) without the
 *                                      rearm edge (SUSPENDED_BY_DIRECTION -> ACTIVE) ever firing.
 *   4. REARM_DELAY_REASON_NO_LOAD      the rearm edge fired but no pedal load ever appeared for
 *                                      REARM_DELAY_NO_LOAD_MS.
 * A record with reason_bits == 0 is discarded at close (it was a healthy fast rearm).
 *
 * CAPTURE MODEL. On the PREARM edge - session_state ACTIVE -> SUSPENDED_BY_DIRECTION, the real
 * initiating event (FW-112 v2: with WAIT_REARM_LOAD gone the record anchors where the delay
 * actually starts, not where the automaton used to wait) - a record is opened and the first
 * snapshot (REARM_MILESTONE_ENTER_SUSPEND) is taken. This is the record's TIMING anchor:
 * t_pressure .. t_close all measure elapsed ticks from HERE. The remaining slots are filled by
 * the GUARANTEED events as they occur, at most one snapshot each:
 *   REARM_MILESTONE_PROBLEM     the instant the first reason-condition fires (WAIT_LONG elapsed /
 *                               NO_LOAD elapsed / saga ended without permission / WEAK_TARGET
 *                               window completed) - the torque state AT the problem;
 *   REARM_MILESTONE_PERMISSION  the SUSPENDED_BY_DIRECTION -> ACTIVE edge (the rearm), if it ever
 *                               comes;
 *   REARM_MILESTONE_RECORD_CLOSE the final state at close (kept records only).
 * Every stage that has NO full snapshot still has its elapsed time in the record's timing block
 * (t_pressure .. t_close). The buffer is exactly REARM_DELAY_RECORDS records; a capture when both
 * slots are already queued is refused and counted (rearm_delay_queue_rejected()), never evicts.
 * This TIMING anchor is deliberately a SEPARATE fact from the TRACE/RAW reservation's own
 * lifecycle below - they answer different questions ("how long did each stage take, measured from
 * the prearm edge" vs "does a decoder trace of the whole saga still exist right now") and are kept
 * on their own clocks on purpose.
 *
 * TRACE/RAW RESERVATION - AN OWNERSHIP LIFECYCLE INDEPENDENT OF THE TIMING RECORD.
 *
 * The real chain a rider experiences is
 *   ACTIVE -> reverse/invalid -> SUSPENDED_BY_DIRECTION -> forward-confirming steps -> ACTIVE (rearm)
 *     -> (possibly) a WEAK_TARGET detected up to 150 ms later
 * and the pas_trace decoder reservation must span the WHOLE thing - not just the SUSPENDED
 * portion - or it cannot contain the reverse/invalid that caused the saga (everything up to and
 * including "forward-confirming steps" happens BEFORE the rearm edge, always at least
 * PAS_REVERSE_RECOVERY_CONFIRM_STEPS of them earlier), and it cannot survive to catch a WEAK_TARGET
 * detected after the rearm either if it is released the instant SUSPENDED_BY_DIRECTION becomes
 * ACTIVE again.
 *
 * The reservation therefore has its OWN two-state ownership lifecycle in pas_trace.c, started and
 * ended by edges THIS module derives from session_state (it has no session automaton linked - see
 * WHAT IT MUST NOT DO):
 *
 *   1. PREARM     - the tick session_state goes ACTIVE -> SUSPENDED_BY_DIRECTION (the real
 *                    initiating event), reported as rearm_delay_prearm_edge(). main.c calls
 *                    pas_trace_rearm_prearm(): a DYNAMICALLY chosen slot (never a fixed index,
 *                    never the ordinary active watcher, never a slot RETAINED from a still-
 *                    unstreamed earlier capture) is reset for a clean history boundary and seeded
 *                    with the initiating transition itself (see pas_trace.h). If the caller still
 *                    owned an UNFINISHED reservation from a previous saga (a new reverse
 *                    interrupting the previous one's recovery watch), pas_trace_rearm_prearm()
 *                    ends that old ownership FIRST, atomically, before claiming anything new - the
 *                    two sagas can never be confused, and the old capture (if armed/ready) is
 *                    never stolen or overwritten (see pas_trace.h's end_ownership()).
 *   2. HELD       - the reservation persists across ANY extra reverse/invalid while suspended
 *                    (each keeps SUSPENDED_BY_DIRECTION, so it is all one saga - there is no
 *                    WAIT_REARM_LOAD state to oscillate through any more in FW-112 v2), through the
 *                    rearm edge itself, and through the WHOLE subsequent RECOVERING watch - main.c
 *                    never re-PREARMs mid-saga, and pas_trace dual-writes every transition into the
 *                    reserved slot the whole time, so it accumulates the initiating reverse/invalid,
 *                    every later forward-confirming step, AND everything through recovery, no matter
 *                    how many records open and close along the way.
 *   3. PROBLEM    - reserve_trigger() is true for EXACTLY ONE tick per record (the first reason
 *                    instant) and is guaranteed visible to main.c even if its record closes in the
 *                    SAME tick (a same-tick note_problem()+close_record(), e.g. a reverse during
 *                    SUSPENDED before WAIT_LONG). main.c checks pas_trace_rearm_held() first: if
 *                    nothing is held (PREARM never found a free slot), the record gets
 *                    REARM_DELAY_CAPTURE_NO_TRACE_NO_HISTORY and pas_trace is never called at all.
 *                    If held, main.c calls pas_trace_rearm_capture() once; a refusal (the slot is
 *                    already armed/ready from an earlier PROBLEM in the SAME saga) yields
 *                    REARM_DELAY_CAPTURE_NO_TRACE_BUSY.
 *   4. PAIR RAW   - on a successful TRACE arm, main.c passes the same id to pas_raw_freeze(). Its
 *                    BOOL is honoured: a refusal is recorded as TRACE_ONLY, never as a full pair.
 *   5. RESULT     - the id and the outcome are stored IN the record that raised the trigger via
 *                    rearm_delay_note_reserve_done() - found by an INTERNAL record_uid (a fresh
 *                    32-bit identity assigned to every record at open and moved with it through
 *                    the ring), not by the wire (record_id, session_id) pair - which is NOT
 *                    unique after the 8-bit record_seq wraps (a record can stay queued across a
 *                    256-rearm wrap) - and not by which slots[] index the record happens to
 *                    occupy (which can move if an unrelated session's record is dequeued in
 *                    between) - so an offline parser can match the FW-111 record to its
 *                    pas_trace/pas_raw captures by id even when the record closed the very same
 *                    tick its own trigger fired. FULL is trustworthy: it is only ever reported
 *                    when the TRACE actually contains a triggered capture seeded from the real
 *                    initiating event.
 *   6. OWNERSHIP  - the tick the CURRENT saga's reservation is truly over, reported as
 *      END           rearm_delay_ownership_end_edge(): a tracking record just closed for a reason
 *                    that concludes the saga, or the saga concluded (rearm or terminal) with no
 *                    record ever tracking it (the queue was full at the prearm edge). Deliberately
 *                    NOT the SUSPENDED_BY_DIRECTION -> ACTIVE rearm edge itself - that only moves
 *                    the record to RECOVERING, and a WEAK_TARGET detected up to 150 ms later still
 *                    needs the reservation held. main.c calls pas_trace_rearm_end_ownership(): a
 *                    reservation that was never armed is freed immediately (no orphaned hold); one
 *                    that IS armed or ready has its OWNERSHIP end but the slot itself is RETAINED
 *                    - excluded from ordinary use and from the NEXT saga's PREARM candidate list,
 *                    still fed by dual-write until its POST window completes - until it is
 *                    actually streamed via pas_trace_slot_release().
 *
 * This module still requests and reports only: it knows nothing about pas_trace/pas_raw slot
 * mechanics, only the session_state byte the caller feeds it and the edges it derives from it.
 *
 * The caller must stamp records with the diag session id via rearm_delay_set_session_id() every
 * tick, exactly as it does for the other recorders, so a dump can ask by session id.
 */

/*
 * capture_status has five values (REARM_DELAY_CAPTURE_NONE/FULL/TRACE_ONLY/NO_TRACE_BUSY/
 * NO_TRACE_NO_HISTORY below); wire layout and every field's meaning are otherwise unchanged.
 *
 * v5 (FW-112 v2): the record's semantics changed with the session automaton's - the anchor moved
 * from WAIT_REARM_LOAD entry to the PREARM edge (ACTIVE -> SUSPENDED_BY_DIRECTION, the real
 * initiating event), milestone 1 is ENTER_SUSPEND and milestone 6 is PERMISSION (the rearm edge
 * itself, no separate commit stage exists any more), and reason 0x04 is NO_PERMISSION (the saga
 * ended without the rearm edge ever firing). An offline parser MUST key on this version: a v4
 * decoder reading a v5 record would misread every elapsed time (the anchor moved by the whole
 * ACTIVE->SUSPENDED->wait delay) and every "no commit" record.
 */
#define REARM_DELAY_SCHEMA_VERSION 5U

#define REARM_DELAY_RECORDS   2U
#define REARM_DELAY_SNAPSHOTS 4U

/*
 * Wire EFIDs. One record is sent strictly in order: header frame, three timing frames,
 * 4 x 8 B data frames per snapshot (fragment order, not content, tells which snapshot a frame
 * belongs to - the dump never interleaves records), then ONE capture-info frame.
 */
#define REARM_DELAY_EFID_HEADER        0x0001021FU
#define REARM_DELAY_EFID_TIMING        0x00010220U   /* +0/+1/+2 = the three timing frames */
#define REARM_DELAY_EFID_SNAPSHOT_BASE 0x00010223U   /* +0..+3 = one 32 B snapshot */
#define REARM_DELAY_EFID_CAPTURE       0x00010227U   /* capture_id + capture_status */

/* Why a record was kept - see the file header. v5 (FW-112 v2): WAIT_LONG now measures the
 * SUSPENDED_BY_DIRECTION hold (> REARM_DELAY_WAIT_LONG_MS of the rearm delay itself), and reason
 * 0x04 is NO_PERMISSION (the saga ended without the rearm edge ever firing) - the old "no commit"
 * became meaningless once the commit stage was removed. WEAK_TARGET and NO_LOAD keep their
 * meanings. */
#define REARM_DELAY_REASON_WAIT_LONG     0x01U
#define REARM_DELAY_REASON_WEAK_TARGET   0x02U
#define REARM_DELAY_REASON_NO_PERMISSION 0x04U
#define REARM_DELAY_REASON_NO_LOAD       0x08U

/*
 * What the TRACE/RAW reservation achieved for this record, stored in the record's capture_status
 * field and serialized on the wire in its OWN frame, REARM_DELAY_EFID_CAPTURE (0x10227) - NOT in
 * the header frame's data[6]. (The header frame, REARM_DELAY_EFID_HEADER, carries session_id/
 * record_id/reason_bits/snapshot_count/pre_reverse_iq only - see the header struct and the wire
 * EFID list above.)
 *
 * Two of the five values distinguish "no trace" causes that mean very different things to a
 * reader deciding how much to trust the record's PRE history: BUSY says a trace WAS captured for
 * an EARLIER PROBLEM in the same saga and this later one simply found the reservation already
 * spent; NO_HISTORY says PREARM itself never found a free slot, so this saga has NO decoder trace
 * at all, not even a partial one from before this record opened.
 */
#define REARM_DELAY_CAPTURE_NONE             0x00U  /* reservation released without a capture
                                                      * (healthy fast rearm / record closed before
                                                      * PROBLEM) */
#define REARM_DELAY_CAPTURE_FULL             0x01U  /* decoder TRACE armed AND raw paired, same id -
                                                      * only ever reported when the TRACE actually
                                                      * contains a triggered capture seeded from the
                                                      * real initiating reverse/invalid */
#define REARM_DELAY_CAPTURE_TRACE_ONLY       0x02U  /* TRACE armed, pas_raw_freeze() refused the pair */
#define REARM_DELAY_CAPTURE_NO_TRACE_BUSY    0x03U  /* PROBLEM fired, a reservation IS held, but its
                                                      * slot is already armed/ready from an earlier
                                                      * PROBLEM in this same saga */
#define REARM_DELAY_CAPTURE_NO_TRACE_NO_HISTORY 0x04U /* PROBLEM fired but no reservation is held at
                                                      * all - PREARM never found a free slot at the
                                                      * initiating event, so there is no history to
                                                      * arm a trace on */

/* "no capture id" marker for the record's capture_id field. */
#define REARM_DELAY_NO_CAPTURE 0xFFU

#define REARM_DELAY_WAIT_LONG_MS    200U
#define REARM_DELAY_WEAK_TARGET_MS  150U
#define REARM_DELAY_NO_LOAD_MS      2000U
#define REARM_DELAY_TIMEOUT_MS      5000U
#define REARM_DELAY_RECOVER_PCT     80U

#define REARM_DELAY_WAIT_LONG_TICKS    ((uint32_t)REARM_DELAY_WAIT_LONG_MS * CONTROL_TIMEBASE_HZ / 1000U)
#define REARM_DELAY_WEAK_TARGET_TICKS  ((uint32_t)REARM_DELAY_WEAK_TARGET_MS * CONTROL_TIMEBASE_HZ / 1000U)
#define REARM_DELAY_NO_LOAD_TICKS      ((uint32_t)REARM_DELAY_NO_LOAD_MS * CONTROL_TIMEBASE_HZ / 1000U)
#define REARM_DELAY_TIMEOUT_TICKS      ((uint32_t)REARM_DELAY_TIMEOUT_MS * CONTROL_TIMEBASE_HZ / 1000U)

/*
 * A timing field's "stage never reached" marker. 0 is a valid elapsed (a stage can fire on the
 * very tick the record opens), so the sentinel is 0xFFFF, never 0.
 */
#define REARM_DELAY_T_UNREACHED 0xFFFFU

/* Snapshot flags byte. */
#define REARM_SNAP_F_SENSOR_VALID       0x01U
#define REARM_SNAP_F_CAL_USER           0x02U  /* 0 = default span, 1 = user calibration */
#define REARM_SNAP_F_DIRECTION_INHIBIT  0x04U
#define REARM_SNAP_F_REAL_STOP          0x08U
#define REARM_SNAP_F_LIMITER_ZEROED     0x10U
#define REARM_SNAP_F_PWM_ON             0x20U
#define REARM_SNAP_F_COMMITTED          0x80U  /* session was ACTIVE at capture time */
/*
 * There is deliberately no STANDSTILL flag: the 25 ms Hall hold is a BLOCKING delay, so no
 * control tick (and therefore no snapshot) ever runs inside it. The hold is measured instead by
 * the record's t_standstill_enter / t_standstill_exit timing fields.
 */

/*
 * Stage ids. Two classes, one numbering space (the per-record "reached" mask is bit(id-1)):
 *   SNAPSHOT milestones - the guaranteed set, one full 32 B snapshot each:
 *     ENTER_SUSPEND 1 (always the first snapshot - the record's anchor), PROBLEM 12,
 *     PERMISSION 6, RECORD_CLOSE 11;
 *   TIMING stages - no snapshot, only the compact elapsed field in the record:
 *     PRESSURE 2, FILTER_READY 3, RUN_READY 4, DEMAND 5, TARGET_RECOVERED 7,
 *     SETPOINT_RECOVERED 8, PWM_ON 9.
 *
 * v5 (FW-112 v2) renames: milestone 1 was ENTER_WAIT (the WAIT_REARM_LOAD entry anchor) and is
 * now ENTER_SUSPEND (the PREARM edge ACTIVE -> SUSPENDED_BY_DIRECTION - the same physical anchor
 * value the record's timing already ran from in v4); milestone 6 was COMMIT (WAIT -> ACTIVE, the
 * old two-phase commit) and is now PERMISSION (SUSPENDED_BY_DIRECTION -> ACTIVE, the rearm edge -
 * the exact tick out->fast_rearm_this_tick fires). Numeric values are unchanged on the wire.
 */
#define REARM_MILESTONE_ENTER_SUSPEND    1U  /* ACTIVE -> SUSPENDED_BY_DIRECTION (prearm edge): anchor */
#define REARM_MILESTONE_PRESSURE          2U  /* first assist_delta_native > 0 */
#define REARM_MILESTONE_FILTER_READY      3U  /* first filtered >= run_deadband */
#define REARM_MILESTONE_RUN_READY         4U  /* first run-filtered > 0 */
#define REARM_MILESTONE_DEMAND            5U  /* first iq_request > 0 */
#define REARM_MILESTONE_PERMISSION        6U  /* SUSPENDED_BY_DIRECTION -> ACTIVE (the rearm edge) */
#define REARM_MILESTONE_TARGET_RECOVERED  7U  /* iq_pre_ramp >= 80 % of pre_reverse_iq */
#define REARM_MILESTONE_SETPOINT_RECOVERED 8U /* MS.i_q_setpoint >= 80 % of pre_reverse_iq */
#define REARM_MILESTONE_PWM_ON            9U  /* first tick PWM power stage enabled */
#define REARM_MILESTONE_RECORD_CLOSE      11U
#define REARM_MILESTONE_PROBLEM           12U /* first reason-condition instant (a full snapshot) */

/*
 * One 32-byte frame of the torque->Iq chain at a snapshot. Field types are chosen so the struct
 * is exactly 32 B with no padding on the ARM target (4-byte alignment from the uint32_t): the
 * 4 x 8 B wire frames serialize it directly. `span_native` is deliberately NOT stored - the
 * parser reconstructs it from calibration_source (default 1139 or the user span, both known
 * offline) - and `run_deadband` IS stored because it is the threshold the FILTER_READY stage was
 * judged against.
 */
typedef struct {
	uint32_t elapsed_ticks;             /* since the record's prearm-edge anchor */
	uint16_t raw_native;
	uint16_t zero_effective_native;
	int16_t  corrected_native;
	uint16_t delta_native;
	uint16_t assist_delta_native;
	uint16_t assist_delta_filtered_native;
	uint16_t assist_delta_run_native;
	uint16_t load_centikg;
	uint16_t run_deadband;
	int16_t  iq_request;                /* mode_output.iq_request, i16-clamped */
	int16_t  iq_pre_ramp;               /* target after latch+limiters, before the ramp */
	int16_t  iq_setpoint;               /* MS.i_q_setpoint - final commanded current */
	uint8_t  flags;                     /* REARM_SNAP_F_* */
	uint8_t  milestone_id;              /* one of the four SNAPSHOT milestones */
	uint8_t  session_id;
	uint8_t  record_id;
} rearm_delay_snapshot_t;

/*
 * One record. 8 B header + 24 B timing block + 4 x 32 B snapshots = exactly 160 B on the ARM
 * target (4-byte alignment). The timing block carries every stage's elapsed time from the WAIT
 * anchor even when that stage has no full snapshot - REARM_DELAY_T_UNREACHED means it never
 * occurred. All u16 are big-endian on the wire.
 */
typedef struct {
	uint8_t session_id;
	uint8_t record_id;
	uint8_t reason_bits;                /* REARM_DELAY_REASON_* */
	uint8_t snapshot_count;             /* 1..REARM_DELAY_SNAPSHOTS */
	int16_t pre_reverse_iq;             /* the demand of the LAST ACTIVE tick before the reverse */
	uint8_t capture_id;                 /* pas_trace id paired with the raw capture, or
	                                     * REARM_DELAY_NO_CAPTURE when none was obtained */
	uint8_t capture_status;          /* REARM_DELAY_CAPTURE_* */

	uint16_t t_pressure;                /* first assist_delta_native > 0 */
	uint16_t t_filter_ready;            /* first filtered >= run_deadband */
	uint16_t t_run_ready;               /* first run-filtered > 0 */
	uint16_t t_demand;                  /* first iq_request > 0 */
	uint16_t t_permission;              /* SUSPENDED_BY_DIRECTION -> ACTIVE (the rearm edge) */
	uint16_t t_target_recovered;        /* iq_pre_ramp >= 80 % of pre_reverse_iq */
	uint16_t t_setpoint_recovered;      /* iq_setpoint >= 80 % of pre_reverse_iq */
	uint16_t t_pwm_on;                  /* first PWM power-stage on (waiting or recovering) */
	uint16_t t_standstill_enter;        /* get_standstill_position() 25 ms hold: enter tick */
	uint16_t t_standstill_exit;         /* ... and exit tick (the hold's length is exit-enter) */
	uint16_t t_weak_start;              /* first tick of the continuous below-80 % stretch */
	uint16_t t_close;                   /* the record's close */

	rearm_delay_snapshot_t snapshots[REARM_DELAY_SNAPSHOTS];
} rearm_delay_record_t;

/*
 * v5.1 layout guards. The record's 8 B header + 24 B timing block + 4 x 32 B snapshots must
 * stay exactly 160 B: the diagnostic RAM budget line item and the offline join by wire
 * (record_id, session_id) both assume it. The internal 32-bit record_uid is a PRIVATE sidecar in
 * src/rearm_delay_diag.c and deliberately does NOT live here - this struct is the wire view and
 * must not grow. These asserts only protect the sizes; the CAN serializer still encodes every
 * field explicitly (never a struct memcpy).
 */
_Static_assert(sizeof(rearm_delay_snapshot_t) == 32U,
	"FW-111: snapshot wire-source layout changed");
_Static_assert(sizeof(rearm_delay_record_t) == 160U,
	"FW-111: record layout changed; update RAM and serialization audit");

/* Everything the module reads this tick. session_state is ride_session_state_t as a plain byte
 * (0 COLD, 1 ACTIVE, 2 SUSPENDED_BY_DIRECTION; 3 = legacy WAIT_REARM_LOAD, RESERVED and never
 * entered in production since FW-112 v2) - the caller reads it from
 * ride_control_get_session_state(); this module never links ride_session.c. */
typedef struct {
	uint8_t session_state;
	bool    direction_inhibit_active;   /* pas_direction_direction_inhibit_active() */
	bool    real_stop;                  /* pas_real_stop - the caller's independent definition */
	bool    limiter_zeroed;             /* RIDE_DBG_LIMITER_ZEROED from ride_control */
	bool    pwm_on;                     /* ui_8_PWM_ON_Flag */
	int16_t iq_request;                 /* mode_output.iq_request, i16-clamped */
	int16_t iq_pre_ramp;                /* ride_arm_snapshot.iq_pre_ramp, i16-clamped */
	int16_t iq_setpoint;                /* MS.i_q_setpoint, i16-clamped */
	uint16_t run_deadband;              /* tuning_config_run_deadband_mv() */
	const torque_snapshot_t *snapshot;  /* torque_input_get_snapshot() - never NULL in prod */
} rearm_delay_input_t;

void rearm_delay_init(void);

/*
 * Once per control tick, after ride_control_update() has run so the session state reflects THIS
 * tick's transition. Pure observation - nothing it does feeds any decision.
 */
void rearm_delay_tick(const rearm_delay_input_t *in, uint32_t now_tick);

/*
 * Called by main.c around get_standstill_position(): enter = just before the 25 ms blocking
 * delay, exit = after. The tick is the caller's control_time_ticks. The two together are how the
 * Hall start is measured (the delay itself is blocking, so no rearm_delay_tick() runs inside it).
 */
void rearm_delay_note_standstill_enter(uint32_t now_tick);
void rearm_delay_note_standstill_exit(uint32_t now_tick);

/* Stamp every record with the diag session id, called by main.c each tick alongside the other
 * recorders (ride_episode_set_session_id etc.). */
void rearm_delay_set_session_id(uint8_t session_id);

/*
 * TRACE/RAW reservation - see the file header for the full cycle. This module does not own the
 * RESERVATION lifecycle itself; it only detects and reports the edges that bracket it, computed
 * fresh inside rearm_delay_tick() from the session_state byte the caller already feeds every
 * tick, valid to read immediately after that call:
 *
 *   rearm_delay_prearm_edge()        true for EXACTLY the tick session_state went ACTIVE (1) ->
 *                                     SUSPENDED_BY_DIRECTION (2) - the real initiating event.
 *                                     main.c calls pas_trace_rearm_prearm() on this edge (which
 *                                     itself ends any still-open PREVIOUS ownership first, if a
 *                                     new saga is interrupting the previous one's recovery watch).
 *   rearm_delay_ownership_end_edge() true for EXACTLY the tick the CURRENT saga's reservation is
 *                                     over: a tracking record just closed for a reason that
 *                                     concludes the saga (a WAIT<->SUSPENDED oscillation no longer
 *                                     exists in v2 - any re-suspend is simply more SUSPENDED, so
 *                                     every record close concludes its saga), or the saga
 *                                     concluded with no record ever tracking it (the queue was
 *                                     full at the prearm edge). Deliberately NOT tied to
 *                                     SUSPENDED_BY_DIRECTION -> ACTIVE (PERMISSION) alone - a
 *                                     WEAK_TARGET is only detected up to 150 ms AFTER the rearm,
 *                                     while the record is in RECOVERING, and the reservation must
 *                                     still be held then. Suppressed on a tick prearm_edge ALSO
 *                                     fires (see pas_trace_rearm_prearm()'s own handling). main.c
 *                                     calls pas_trace_rearm_end_ownership() on this edge.
 *
 * rearm_delay_reserve_trigger() is true for EXACTLY ONE tick per record (the first reason
 * instant), and now SURVIVES that record closing in the same tick (main.c is guaranteed to see it
 * even when note_problem() and close_record() run back to back). main.c checks
 * pas_trace_rearm_held() itself before acting on it - this module has no opinion on whether a
 * slot is actually held, only on when the record's own PROBLEM condition fires.
 *
 * On the trigger tick main.c arms the decoder capture (if held), pairs it into pas_raw_freeze(),
 * and reports the outcome with rearm_delay_note_reserve_done(capture_id, status). The id and
 * status are written to the record that raised the trigger - found by its INTERNAL record_uid
 * (assigned at open, moved with the record through every queue shift; NOT the wire
 * (record_id, session_id) pair, which can collide after the 8-bit record_seq wraps) and never by
 * R.cur_slot, which may already be invalid by the time this runs - so an offline parser can join
 * the FW-111 record to its captures even when the record closed in the very same tick as its own
 * trigger.
 */
bool rearm_delay_prearm_edge(void);
bool rearm_delay_ownership_end_edge(void);
bool rearm_delay_reserve_trigger(void);
void rearm_delay_note_reserve_done(uint8_t capture_id, uint8_t capture_status);

/* --- v5.1 TEST-ONLY seam (NEVER compiled into any firmware build - see the #ifdef in the .c) ---
 * Lets a host test position the internal record_uid generator right up against UINT32_MAX so the
 * uint32_t wrap and the skip-in-use rules can be proven deterministically. */
#ifdef REARM_UID_SEAM_TEST
void rearm_delay_test_set_uid_next(uint32_t v);
uint32_t rearm_delay_test_uid_next(void);
#endif

/* --- diag_record_source bridge (same shape as ride_episode's) --------------------------------- */

uint16_t rearm_delay_queue_count_session(uint8_t session_id);
bool rearm_delay_queue_peek_session(uint8_t session_id, rearm_delay_record_t *out);
void rearm_delay_queue_release_session(uint8_t session_id);
uint32_t rearm_delay_queue_enqueued(void);
uint32_t rearm_delay_queue_rejected(void);

/* For tests and observation: 0 idle, 1 suspended (waiting for the rearm edge), 2 recovering. */
uint8_t rearm_delay_fsm_state(void);

#endif /* REARM_DELAY_DIAG_H_ */
