#ifndef RIDE_EPISODE_H_
#define RIDE_EPISODE_H_

#include <stdbool.h>
#include <stdint.h>

#include "config.h"   /* FW-104: CONTROL_TIMEBASE_HZ - constants only, no MS/MP dependency */

/*
 * FW-101: recorder for ONE re-engagement episode — the stretch between a reverse quadrature
 * step taking assist away and the rider getting it back.
 *
 * WHY IT IS A MODULE. The periodic diagnostic frames go out every 40 ms while PAS steps arrive
 * every 12-39 ms, so a whole reverse-and-recover transition can happen between two frames. The
 * aggregate counters can say how OFTEN interruptions happen but never where the time inside one
 * went. This times it at the 4 kHz tick and publishes the finished result, so the bus rate does
 * not change.
 *
 * It lives in its own file, free of MS/MP and of any external call, for one reason: the first
 * version of this recorder was written inline in main.c and shipped with three defects that
 * invalidated its own output. Nothing in main.c can be exercised by a test — every existing JS
 * test reads it as TEXT. Here the state machine can be linked and run by
 * tests/host/fw101_episode_host.c, which is the only way this measurement earns any trust.
 *
 * MEASUREMENT ONLY. No decision anywhere reads any of this.
 *
 * THE THREE TIMES, and why three. A single latch-to-recovery split cannot tell the ramp apart
 * from the estimators, because the pipeline may still be asking for very little when the latch
 * arms:
 *
 *   reverse -> latch          the decoder and the forward-step gate (fwd_run, Backwards_counter)
 *   latch  -> target ready    the 4th step, the RUN estimator, the power filter, the limiters
 *   target ready -> recovered the final Iq ramp alone
 *
 * "target ready" is the pre-ramp Iq target reaching RIDE_EPISODE_RECOVER_PCT of what the rider
 * had before the disturbance; "recovered" is the commanded current reaching the same fraction.
 *
 * FW-102 — WHY THOSE THREE TIMES WERE NOT ENOUGH. The first measured ride (0.0314) answered the
 * question it was built for and immediately raised a bigger one. Of the median 901 ms to get
 * assist back, 808 ms passed BEFORE the latch armed and 5 ms after it: the RUN estimator, the
 * power filter, the limiters and the ramp together cost almost nothing. But all three times are
 * anchored on the FIRST reverse step of the episode, so a long one is equally consistent with
 *
 *     the rider genuinely backpedalled for most of that second, or
 *     backpedalling stopped early and re-engagement is what took the time.
 *
 * Those call for opposite fixes, and no frame in that build could separate them. The anchor is
 * kept (all times stay comparable) and the milestones after the LAST reverse step are added:
 *
 *   t_last_reverse   when backpedalling actually stopped -> everything after it is ours
 *   t_first_forward  the crank moving forward again      -> the rider, not the firmware
 *   t_steps_ready    fwd_run reached the required count  -> the cost of the step gate
 *   t_load_ready     pedal load reached its threshold    -> the cost of the pressure gate
 *
 * t_latch - t_last_reverse is therefore the honest cost of re-engagement, and the three
 * milestones inside it say which gate spent it. The thresholds actually in force are published
 * alongside, because both are configurable and reading a measurement against an assumed limit
 * is how the 8-to-5 arithmetic went wrong earlier in this investigation.
 *
 * Times are reset by each further reverse step, because the conditions themselves are: a reverse
 * step zeroes fwd_run. They stay measured from the episode anchor so they remain comparable with
 * t_latch and t_recover.
 *
 * FW-104 — WHY THIS MODULE NO LONGER COUNTS ITS OWN TICKS. The clock used to be a plain
 * `ticks++` once per ride_episode_tick() call. That call happens once per reg_ADC_processing()
 * invocation, which is gated by a single-bit flag set at 4 kHz - if the main loop falls behind
 * for even one period (the CAN diagnostic frames alone can do that), an interrupt sets the same
 * flag twice before it is read once, and a whole 4 kHz period is never counted. Exactly the bug
 * FW-103 fixed for wheel speed. Every t_* value this module ever published was built on that same
 * assumption. The fix here is the same shape: the caller now owns a real hardware tick counter
 * (control_time_ticks in main.c) and passes its CURRENT value into every entry point; this module
 * anchors to it and measures elapsed time as a subtraction, never as a count of its own calls.
 */

/* Fraction of the pre-disturbance current that counts as "the rider has it back". */
#define RIDE_EPISODE_RECOVER_PCT 80
/* Below this fraction the drive is considered genuinely lost, which starts the clock proper. */
#define RIDE_EPISODE_DIP_PCT     50
/*
 * While nothing has been lost yet, a further reverse step RE-ANCHORS the episode — but only
 * while the current is still essentially untouched. Past this fraction a decay is already under
 * way, and re-anchoring there would let successive steps chase the falling current: the episode
 * would start late, count too few steps, and could miss the dip entirely because the reference
 * had been dragged down with it. That was a real defect in the first version.
 */
#define RIDE_EPISODE_INTACT_PCT  90
/* Give up on an episode that never recovers, so one bad event cannot swallow the log. */
#define RIDE_EPISODE_TIMEOUT_MS  2000
/* FW-104: derived from the one shared rate, not RIDE_EPISODE_TICKS_PER_MS (removed - that WAS
 * the assumption that main.c calls this module at a true 4 kHz, which FW-103's finding disproved). */
#define RIDE_EPISODE_TIMEOUT_TICKS ((uint32_t)RIDE_EPISODE_TIMEOUT_MS * CONTROL_TIMEBASE_HZ / 1000U)
#define RIDE_EPISODE_TIME_NONE    0xFFFFU  /* this milestone was never reached */

/* Flags in ride_episode_result_t.flags */
#define RIDE_EP_FLAG_HARD_CUT     0x01  /* brake / reverse / fault was active at some point */
#define RIDE_EP_FLAG_PAS_TIMEOUT  0x02  /* the cranks were judged stopped at some point */
#define RIDE_EP_FLAG_LATCH_ARMED  0x04  /* the ride latch re-armed before the episode ended */
#define RIDE_EP_FLAG_LIMITER      0x08  /* a limiter took a non-zero demand to zero */
#define RIDE_EP_FLAG_TIMED_OUT    0x10  /* gave up after RIDE_EPISODE_TIMEOUT_MS */
#define RIDE_EP_FLAG_TARGET_READY 0x20  /* the pre-ramp target recovered before the current did */
#define RIDE_EP_FLAG_ROLLING      0x40  /* the bike was moving at some point during the episode */
#define RIDE_EP_FLAG_FWD_SEEN     0x80  /* at least one forward step after the last reverse one */

typedef enum {
	RIDE_EP_IDLE = 0,
	RIDE_EP_WAIT_DIP = 1,     /* a reverse step happened; the current has not fallen yet */
	RIDE_EP_WAIT_RECOVER = 2  /* the current did fall; waiting for it to come back */
} ride_episode_state_t;

typedef struct {
	int32_t  iq_setpoint;   /* final commanded current, after the ramp */
	int32_t  iq_pre_ramp;   /* target after latch and limiters, BEFORE the ramp */
	uint16_t arm_seq;       /* ride_control's arming counter; a change means a new arming */
	bool     hard_cut;
	bool     limiter_zeroed;
	bool     pas_timeout;
	/* Values ride_control latched at the arming instant, copied in when arm_seq changes. */
	uint16_t arm_load_centikg;
	uint16_t arm_fast_native;
	uint16_t arm_run_seed_native;
	int32_t  arm_iq_after_limits;
	/* FW-107: whether THIS arming took the fast-rearm path (RIDE_REARM_ARMED, a real RUN-level
	 * demand, no start-load threshold) rather than the normal load-threshold latch. */
	bool     arm_fast_rearm;
	/*
	 * FW-102: the two gates that stand between a forward step and the latch, each with the
	 * threshold ACTUALLY in force this tick rather than a compile-time constant — both are
	 * configurable and both have been misread from an assumed value before.
	 */
	uint8_t  fwd_run;                 /* consecutive forward quadrature steps */
	uint8_t  required_steps;          /* what fwd_run has to reach right now */
	uint16_t load_centikg;            /* pedal load as the latch gate sees it */
	uint16_t load_threshold_centikg;  /* what that load has to reach right now */
	bool     rolling;                 /* the bike is moving (selects the rolling threshold) */
} ride_episode_input_t;

typedef struct {
	uint16_t number;               /* completed episodes; a change means a new record */
	uint8_t  rev_steps;
	uint8_t  flags;
	uint16_t t_latch_ms;
	uint16_t t_target_ready_ms;
	uint16_t t_recover_ms;
	/* All RIDE_EPISODE_TIME_NONE / 0 when the episode had no arming — never stale values
	 * carried over from the previous one. */
	uint16_t arm_load_centikg;
	uint16_t arm_fast_native;
	uint16_t arm_run_seed_native;
	int32_t  arm_iq_after_limits;
	/* FW-102: all measured from the same anchor as the times above, so they subtract cleanly. */
	uint16_t t_last_reverse_ms;
	uint16_t t_first_forward_ms;
	uint16_t t_steps_ready_ms;
	uint16_t t_load_ready_ms;
	/*
	 * LIVE, sampled every tick like iq_pre_ramp above, so these are the gate as it stood on
	 * the LAST tick of the episode - not frozen at the reverse step. A gate that eases mid-wait
	 * (the bike starts rolling, dropping the required step count) shows up here as a change,
	 * which is itself information: the wait outlasted a threshold that got easier.
	 */
	uint8_t  required_steps;
	/*
	 * FW-106: which riding session produced this episode, stamped when it is QUEUED rather than
	 * when it is sent — an interrupted dump can leave records of two sessions side by side in
	 * the queue, and at that point "the current session" no longer says who produced what.
	 * Placed here deliberately: it fits the padding byte that already followed required_steps,
	 * so the struct stays 40 B and the 64-entry queue stays within its 2560 B budget.
	 */
	uint8_t  session_id;
	/*
	 * FW-107: 1 = this episode's latch armed via the fast-rearm path (direction confirmed after
	 * a lone reverse step, a real RUN-level demand, never the start-load threshold), 0 = the
	 * normal load-threshold latch, or no arming at all (see arm_load_centikg's "no stale value"
	 * rule - both read 0 identically when !have_arm). The struct was already exactly 36 B with
	 * no spare padding (verified: every existing field already accounts for the full size), so
	 * this one new byte moves the 64-entry queue from 36 to 40 B/entry - measured and folded
	 * into inc/diag_budget.h, not estimated.
	 */
	uint8_t  fast_rearm;
	uint16_t load_threshold_centikg;
	uint16_t load_peak_centikg;       /* highest load seen since the last reverse step */
	/*
	 * FW-106: the pre-ramp Iq target AT THE INSTANT t_target_ready was set — not read live from
	 * ride_control at dump time. A queued record can be dumped long after the episode itself
	 * completed, once other episodes or even other sessions have run; ride_control's CURRENT
	 * arm snapshot by then has nothing to do with what was true for THIS episode. 0 when
	 * t_target_ready_ms is RIDE_EPISODE_TIME_NONE (never reached) — same "no stale value"
	 * discipline as the arm_* fields above. Placed here because it fits the struct's existing
	 * trailing padding (34->36 B) exactly: the queue's per-record cost does not grow.
	 */
	uint16_t iq_pre_ramp_at_target_ready;
} ride_episode_result_t;

/*
 * FW-106: how many completed episodes can wait for the post-ride dump. Before this there was a
 * single published slot, so a second episode finishing within the 40 ms diagnostic period simply
 * overwrote the first — which is one of the three explanations for the episode missing from the
 * ride log that started this card.
 */
#if CAN_DIAGNOSTICS_ENABLE
#define RIDE_EPISODE_QUEUE_LEN 64U
#else
/* The normal firmware never dumps anything, so it keeps the single record it always had and
 * pays nothing for this card. */
#define RIDE_EPISODE_QUEUE_LEN 1U
#endif

void ride_episode_init(void);

/*
 * FW-106: the session id stamped onto every episode queued from now on. main.c bumps it when a
 * new ride starts; this module only records it.
 */
void ride_episode_set_session_id(uint8_t session_id);

/*
 * Call from the PAS decoder on every BACKWARD quadrature step, before ride_episode_tick().
 * arm_seq_now must be ride_control's current arming counter, so the recorder can tell a later
 * arming from the one that was already in place — getting that wrong made the first version
 * report a latch time of ~0 ms and publish another episode's values.
 *
 * FW-104: now_tick must be control_time_ticks read ONCE at the top of the caller's control
 * iteration and reused for every call this module makes within that same iteration - not
 * re-read per call. An interrupt landing between two re-reads inside one logical iteration
 * would otherwise let reverse_step() and tick() see two different "now"s for what is meant to
 * be a single instant.
 */
void ride_episode_reverse_step(int32_t iq_setpoint_now, uint16_t arm_seq_now, uint32_t now_tick);

/*
 * FW-102: call from the PAS decoder on every FORWARD quadrature step. Only the first one after
 * the last reverse step is timed; the rest are ignored, so this is cheap to call unconditionally.
 * FW-104: now_tick - see ride_episode_reverse_step().
 */
void ride_episode_forward_step(uint32_t now_tick);

/*
 * Call once per control tick, after the assist pipeline has produced this tick's current.
 * FW-104: now_tick - see ride_episode_reverse_step().
 */
void ride_episode_tick(const ride_episode_input_t *input, uint32_t now_tick);

void ride_episode_get_result(ride_episode_result_t *out);
ride_episode_state_t ride_episode_get_state(void);

/*
 * FW-106 queue reader. PEEK then DROP, rather than a single pop, because the dump sends one
 * episode as six CAN frames and a record must not leave the queue until the last of them has
 * been confirmed — or until it is deliberately abandoned. Peeking keeps that decision with the
 * sender, where the transmit result is known.
 */
bool     ride_episode_queue_peek(ride_episode_result_t *out);
void     ride_episode_queue_drop(void);
uint16_t ride_episode_queue_depth(void);

/*
 * FW-106 session-scoped access. The dump asks BY SESSION, never "whatever is at the head", which
 * is what keeps one session's episodes out of another session's brackets when an interrupted
 * dump has left records of two rides in the queue at once.
 */
uint16_t ride_episode_queue_count_session(uint8_t session_id);
bool     ride_episode_queue_peek_session(uint8_t session_id, ride_episode_result_t *out);
void     ride_episode_queue_release_session(uint8_t session_id);

/*
 * Lifetime totals, never reset outside init. This module counts only what it DECIDES: a record
 * accepted into the queue, or refused because the queue was full. Whether a record that left the
 * queue was delivered or abandoned is not knowable here - that is the sender's business, and
 * diag_session.c keeps sent and discarded apart precisely so the two can never be conflated.
 */
uint32_t ride_episode_queue_enqueued(void);
uint32_t ride_episode_queue_removed(void);
uint32_t ride_episode_queue_rejected(void);

#endif /* RIDE_EPISODE_H_ */
