#include "ride_episode.h"

/*
 * FW-101: see the header for what this measures and why it is a module rather than a few
 * variables in main.c. FW-104: see the header for why the clock is now the caller's
 * control_time_ticks rather than a count of this module's own calls.
 *
 * All state is file-local and the module takes its whole world through ride_episode_input_t
 * plus now_tick, so tests/host/fw101_episode_host.c can drive it tick by tick on a PC.
 *
 * FW-106: every mutable variable this file owns lives in the one struct E below, so the RAM
 * budget check measures the module's real footprint rather than a hand-picked array. That
 * includes the queue, its bookkeeping, the in-progress episode, and the published copy.
 */

static struct {
	ride_episode_state_t state;
	uint32_t anchor_tick;      /* control_time_ticks at the anchoring reverse step */
	int32_t  iq_ref;           /* current the rider had at that instant */
	uint8_t  rev_steps;
	uint8_t  flags;
	uint16_t t_latch;
	uint16_t t_target_ready;
	/*
	 * FW-106: captured in the SAME tick t_target_ready is set - see ride_episode_tick() and the
	 * header. Not read live from ride_control at CAN-dump time, which could be long after this
	 * episode, or even this session, ended.
	 */
	uint16_t iq_pre_ramp_at_target_ready;
	uint16_t arm_seq_at_anchor; /* the arming counter as it stood when the episode began */
	bool     have_arm;          /* an arming has been seen inside THIS episode */
	uint16_t arm_load;
	uint16_t arm_fast;
	uint16_t arm_seed;
	int32_t  arm_iq;
	bool     arm_fast_rearm;    /* FW-107: was the (single) arming inside this episode fast-rearm */
	/* FW-102: the anatomy of the wait between the last reverse step and the latch. */
	uint16_t t_last_reverse;
	uint16_t t_first_forward;
	uint16_t t_steps_ready;
	uint16_t t_load_ready;
	uint8_t  req_steps;
	uint16_t load_thr;
	uint16_t load_peak;

	ride_episode_result_t published;

	/*
	 * FW-106: the queue that replaced "one published slot that the next episode overwrites".
	 * Peek/drop rather than pop — see the header for why the sender owns the removal.
	 */
	ride_episode_result_t queue[RIDE_EPISODE_QUEUE_LEN];
	uint16_t q_head;   /* next write */
	uint16_t q_tail;   /* next read */
	uint16_t q_depth;
	uint32_t q_enqueued_total;
	uint32_t q_removed_total;
	uint32_t q_rejected_total;
	uint8_t  current_session_id;
} E;

#include "diag_budget.h"
_Static_assert(sizeof(E) <= DIAG_BUDGET_EPISODE_QUEUE_BYTES,
	"FW-106: ride_episode's total state exceeds its RAM line item");

/* Scale a reference by a percentage without overflowing: iq is well under 32767. */
static bool at_least_pct(int32_t value, int32_t reference, int32_t pct)
{
	return (int32_t)(value * 100) >= (int32_t)(reference * pct);
}

static bool below_pct(int32_t value, int32_t reference, int32_t pct)
{
	return (int32_t)(value * 100) < (int32_t)(reference * pct);
}

/* FW-104: unsigned subtraction from a free-running hardware counter - correct across its wrap. */
static uint32_t elapsed_ticks(uint32_t now_tick)
{
	return now_tick - E.anchor_tick;
}

static uint16_t ticks_to_ms(uint32_t t)
{
	uint32_t ms = t / (CONTROL_TIMEBASE_HZ / 1000U);
	return (ms >= RIDE_EPISODE_TIME_NONE) ? (RIDE_EPISODE_TIME_NONE - 1U) : (uint16_t)ms;
}

/*
 * FW-102: everything that a reverse step physically undoes. fwd_run goes to zero on every
 * backward step, so the forward-step gate genuinely restarts and a milestone recorded before it
 * would describe a run that no longer exists. The load peak restarts with them: the question it
 * answers is "was the pressure gate ever close during THIS wait".
 */
static void clear_forward_milestones(void)
{
	E.t_first_forward = RIDE_EPISODE_TIME_NONE;
	E.t_steps_ready = RIDE_EPISODE_TIME_NONE;
	E.t_load_ready = RIDE_EPISODE_TIME_NONE;
	E.load_peak = 0;
}

/*
 * FW-106 — THE FIX FOR BUG 1. A further reverse step inside a live episode undoes the arming
 * just as surely as it undoes the forward run, because fwd_run goes to zero and the latch drops
 * with it. Before this, only the forward milestones were cleared: t_latch, have_arm and the
 * arm_* snapshot survived from the FIRST arming, while t_last_reverse moved on to the second
 * reverse step. The published record then mixed two cycles, and t_latch - t_last_reverse — the
 * one number the record exists to provide — came out negative in 22 episodes of the ride log.
 *
 * arm_seq_at_anchor is re-baselined to the counter as it stands NOW, so the NEXT genuine change
 * counts as a new arming; leaving the old value would make the very next tick declare an arming
 * that had actually happened before this reverse step. The two flags go with the times they
 * describe: a record claiming LATCH_ARMED with t_latch = NONE would be self-contradictory.
 */
static void clear_arm_state(uint16_t arm_seq_now)
{
	E.have_arm = false;
	E.arm_seq_at_anchor = arm_seq_now;
	E.t_latch = RIDE_EPISODE_TIME_NONE;
	E.t_target_ready = RIDE_EPISODE_TIME_NONE;
	E.iq_pre_ramp_at_target_ready = 0;
	E.arm_load = 0;
	E.arm_fast = 0;
	E.arm_seed = 0;
	E.arm_iq = 0;
	E.arm_fast_rearm = false;
	E.flags &= (uint8_t)~(RIDE_EP_FLAG_LATCH_ARMED | RIDE_EP_FLAG_TARGET_READY);
}

static void clear_episode(uint32_t now_tick)
{
	E.state = RIDE_EP_IDLE;
	E.anchor_tick = now_tick;
	E.iq_ref = 0;
	E.rev_steps = 0;
	E.flags = 0;
	E.t_latch = RIDE_EPISODE_TIME_NONE;
	E.t_target_ready = RIDE_EPISODE_TIME_NONE;
	E.iq_pre_ramp_at_target_ready = 0;
	E.have_arm = false;
	E.arm_load = 0;
	E.arm_fast = 0;
	E.arm_seed = 0;
	E.arm_iq = 0;
	E.arm_fast_rearm = false;
	E.t_last_reverse = RIDE_EPISODE_TIME_NONE;
	clear_forward_milestones();
	E.req_steps = 0;
	E.load_thr = 0;
}

void ride_episode_init(void)
{
	clear_episode(0);
	E.arm_seq_at_anchor = 0;
	E.q_head = 0;
	E.q_tail = 0;
	E.q_depth = 0;
	E.q_enqueued_total = 0;
	E.q_removed_total = 0;
	E.q_rejected_total = 0;
	E.current_session_id = 0;
	E.published.session_id = 0;
	E.published.number = 0;
	E.published.rev_steps = 0;
	E.published.flags = 0;
	E.published.t_latch_ms = RIDE_EPISODE_TIME_NONE;
	E.published.t_target_ready_ms = RIDE_EPISODE_TIME_NONE;
	E.published.iq_pre_ramp_at_target_ready = 0;
	E.published.t_recover_ms = RIDE_EPISODE_TIME_NONE;
	E.published.arm_load_centikg = 0;
	E.published.arm_fast_native = 0;
	E.published.arm_run_seed_native = 0;
	E.published.arm_iq_after_limits = 0;
	E.published.fast_rearm = 0;
	E.published.t_last_reverse_ms = RIDE_EPISODE_TIME_NONE;
	E.published.t_first_forward_ms = RIDE_EPISODE_TIME_NONE;
	E.published.t_steps_ready_ms = RIDE_EPISODE_TIME_NONE;
	E.published.t_load_ready_ms = RIDE_EPISODE_TIME_NONE;
	E.published.required_steps = 0;
	E.published.load_threshold_centikg = 0;
	E.published.load_peak_centikg = 0;
}

void ride_episode_reverse_step(int32_t iq_setpoint_now, uint16_t arm_seq_now, uint32_t now_tick)
{
	/*
	 * FW-106: this runs BEFORE the zero-current return below, and that placement is the fix.
	 * A reverse step arriving while the drive is already at zero used to leave the recorder
	 * untouched — yet it is precisely the step that drops the latch and restarts the whole
	 * wait. Everything it invalidates is cleared here, for any live episode, whatever the
	 * current happens to be. The times stay anchored to the FIRST reverse step of the episode,
	 * so every t_* in a published record remains comparable with every other, and no existing
	 * CAN frame changes meaning.
	 */
	if (E.state != RIDE_EP_IDLE) {
		clear_arm_state(arm_seq_now);
		E.t_last_reverse = ticks_to_ms(elapsed_ticks(now_tick));
		clear_forward_milestones();
	}

	if (iq_setpoint_now <= 0) {
		/* Nothing was flowing, so this step cannot START an episode. */
		return;
	}

	/*
	 * ANCHORING. Two defects in the first version lived here, and both invalidated the
	 * measurement rather than merely blurring it.
	 *
	 * 1. It re-anchored on EVERY step until a dip appeared, taking the CURRENT current as the
	 *    new reference each time. During a shutdown ramp the successive reverse steps then
	 *    chased the falling current downward: the clock restarted late, the step count reset,
	 *    and the reference could be dragged so low that the dip was never recognised at all.
	 *    Re-anchoring is now allowed only while the current is still essentially INTACT —
	 *    i.e. nothing has been lost yet, so this step really is the beginning.
	 *
	 * 2. It set the remembered arming counter to a sentinel, so the next tick saw the stale
	 *    counter from an OLDER arming, called it new, and reported a latch time near zero
	 *    together with another episode's values. The counter as it stands right now is what
	 *    gets remembered.
	 */
	bool intact = (E.state == RIDE_EP_IDLE) ||
		(E.state == RIDE_EP_WAIT_DIP &&
		 at_least_pct(iq_setpoint_now, E.iq_ref, RIDE_EPISODE_INTACT_PCT));

	if (intact) {
		uint8_t carried = (E.state == RIDE_EP_IDLE) ? 0U : E.rev_steps;
		clear_episode(now_tick);
		E.state = RIDE_EP_WAIT_DIP;
		E.iq_ref = iq_setpoint_now;
		E.arm_seq_at_anchor = arm_seq_now;
		/* Steps already seen in an untouched window still belong to this push. */
		E.rev_steps = carried;
		/*
		 * FW-102: this step IS the anchor, so the wait for re-engagement starts here and
		 * reads 0 ms. (For a step that does not re-anchor, the block at the top of this
		 * function has already stamped it against the original anchor.)
		 */
		E.t_last_reverse = 0;
	}

	/* State cannot be IDLE here: either it already was not, or the intact branch left WAIT_DIP. */
	if (E.rev_steps < 255U) {
		E.rev_steps++;
	}
}

void ride_episode_forward_step(uint32_t now_tick)
{
	if (E.state == RIDE_EP_IDLE || E.t_first_forward != RIDE_EPISODE_TIME_NONE) {
		return;
	}
	E.t_first_forward = ticks_to_ms(elapsed_ticks(now_tick));
	E.flags |= RIDE_EP_FLAG_FWD_SEEN;
}

void ride_episode_tick(const ride_episode_input_t *input, uint32_t now_tick)
{
	if (input == 0 || E.state == RIDE_EP_IDLE) {
		return;
	}

	uint32_t elapsed = elapsed_ticks(now_tick);

	if (input->hard_cut) {
		E.flags |= RIDE_EP_FLAG_HARD_CUT;
	}
	if (input->limiter_zeroed) {
		E.flags |= RIDE_EP_FLAG_LIMITER;
	}
	if (input->pas_timeout) {
		E.flags |= RIDE_EP_FLAG_PAS_TIMEOUT;
	}

	/*
	 * A NEW arming is one whose counter differs from the value captured when this episode was
	 * anchored — not from "whatever we saw last tick". Only the first arming inside an episode
	 * is recorded; a second one belongs to the rider having got going again.
	 */
	if (!E.have_arm && input->arm_seq != E.arm_seq_at_anchor) {
		E.have_arm = true;
		E.flags |= RIDE_EP_FLAG_LATCH_ARMED;
		E.t_latch = ticks_to_ms(elapsed);
		E.arm_load = input->arm_load_centikg;
		E.arm_fast = input->arm_fast_native;
		E.arm_seed = input->arm_run_seed_native;
		E.arm_iq = input->arm_iq_after_limits;
		E.arm_fast_rearm = input->arm_fast_rearm;
	}

	if (input->rolling) {
		E.flags |= RIDE_EP_FLAG_ROLLING;
	}

	/*
	 * FW-102: the forward-step and pedal-load gates, timed the same way as the latch. Recorded
	 * once per episode (the first time each is met since the last reverse step), so a gate that
	 * was already satisfied before the anchor still gets a real timestamp rather than 0 ms.
	 */
	E.req_steps = input->required_steps;
	E.load_thr = input->load_threshold_centikg;
	if (input->load_centikg > E.load_peak) {
		E.load_peak = input->load_centikg;
	}
	if (E.t_steps_ready == RIDE_EPISODE_TIME_NONE && input->fwd_run >= input->required_steps) {
		E.t_steps_ready = ticks_to_ms(elapsed);
	}
	if (E.t_load_ready == RIDE_EPISODE_TIME_NONE &&
	    input->load_centikg >= input->load_threshold_centikg) {
		E.t_load_ready = ticks_to_ms(elapsed);
	}

	/*
	 * The pipeline is asking for the rider's power again — before the ramp has delivered it.
	 * FW-102 fix: gated on WAIT_RECOVER, i.e. only after the dip has actually happened. Checked
	 * unconditionally from the anchor tick, this was true on the FIRST tick of every episode —
	 * iq_pre_ramp had not moved yet, so it was still >= RECOVER_PCT of the reference by
	 * definition, and t_target_ready read a few ms while the drive had not been touched.
	 */
	if (E.state == RIDE_EP_WAIT_RECOVER && E.t_target_ready == RIDE_EPISODE_TIME_NONE &&
		at_least_pct(input->iq_pre_ramp, E.iq_ref, RIDE_EPISODE_RECOVER_PCT)) {
		E.t_target_ready = ticks_to_ms(elapsed);
		/*
		 * FW-106: the pre-ramp target AT THIS INSTANT, clamped the same way main.c has always
		 * clamped this exact quantity for CAN (0..65535) - captured here because this tick, not
		 * whenever the record eventually gets dumped, is the only moment it is actually true.
		 */
		E.iq_pre_ramp_at_target_ready = (input->iq_pre_ramp < 0) ? 0U :
			((input->iq_pre_ramp > 65535) ? 65535U : (uint16_t)input->iq_pre_ramp);
		E.flags |= RIDE_EP_FLAG_TARGET_READY;
	}

	if (E.state == RIDE_EP_WAIT_DIP) {
		if (below_pct(input->iq_setpoint, E.iq_ref, RIDE_EPISODE_DIP_PCT)) {
			E.state = RIDE_EP_WAIT_RECOVER;
		} else if (elapsed > RIDE_EPISODE_TIMEOUT_TICKS) {
			/* The drive was never actually lost: nothing worth reporting. */
			clear_episode(now_tick);
		}
		return;
	}

	/* RIDE_EP_WAIT_RECOVER */
	bool recovered = at_least_pct(input->iq_setpoint, E.iq_ref, RIDE_EPISODE_RECOVER_PCT);
	bool gave_up = elapsed > RIDE_EPISODE_TIMEOUT_TICKS;
	if (!recovered && !gave_up) {
		return;
	}
	if (gave_up) {
		E.flags |= RIDE_EP_FLAG_TIMED_OUT;
	}

	E.published.rev_steps = E.rev_steps;
	E.published.flags = E.flags;
	E.published.t_latch_ms = E.t_latch;
	E.published.t_target_ready_ms = E.t_target_ready;
	E.published.iq_pre_ramp_at_target_ready = E.iq_pre_ramp_at_target_ready;
	E.published.t_recover_ms = ticks_to_ms(elapsed);
	/*
	 * Never publish another episode's arming values. Without an arming inside THIS episode the
	 * fields are cleared, so a reader cannot mistake stale numbers for fresh ones — the first
	 * version left them standing and they looked entirely plausible.
	 */
	E.published.arm_load_centikg = E.have_arm ? E.arm_load : 0U;
	E.published.arm_fast_native = E.have_arm ? E.arm_fast : 0U;
	E.published.arm_run_seed_native = E.have_arm ? E.arm_seed : 0U;
	E.published.arm_iq_after_limits = E.have_arm ? E.arm_iq : 0;
	E.published.fast_rearm = (E.have_arm && E.arm_fast_rearm) ? 1U : 0U;
	E.published.t_last_reverse_ms = E.t_last_reverse;
	E.published.t_first_forward_ms = E.t_first_forward;
	E.published.t_steps_ready_ms = E.t_steps_ready;
	E.published.t_load_ready_ms = E.t_load_ready;
	E.published.required_steps = E.req_steps;
	E.published.load_threshold_centikg = E.load_thr;
	E.published.load_peak_centikg = E.load_peak;
	E.published.session_id = E.current_session_id;
	E.published.number++;

	/*
	 * FW-106: queue a copy. When the queue is full the NEW record is refused and the queue is
	 * left alone — never evicting an older one. Two reasons: the older records sit closer to the
	 * event under investigation, and eviction would have to reckon with the record the dump may
	 * already be part-way through sending. The loss is visible either way, through
	 * ride_episode_queue_rejected(). A rejected record never counted as enqueued, which is what
	 * keeps pending = enqueued - removed exact.
	 */
	if (E.q_depth < RIDE_EPISODE_QUEUE_LEN) {
		E.queue[E.q_head] = E.published;
		E.q_head = (uint16_t)((E.q_head + 1U) % RIDE_EPISODE_QUEUE_LEN);
		E.q_depth++;
		E.q_enqueued_total++;
	} else {
		E.q_rejected_total++;
	}

	clear_episode(now_tick);
}

void ride_episode_set_session_id(uint8_t session_id)
{
	E.current_session_id = session_id;
}

bool ride_episode_queue_peek(ride_episode_result_t *out)
{
	if (out == 0 || E.q_depth == 0) {
		return false;
	}
	*out = E.queue[E.q_tail];
	return true;
}

void ride_episode_queue_drop(void)
{
	if (E.q_depth == 0) {
		return;
	}
	E.q_tail = (uint16_t)((E.q_tail + 1U) % RIDE_EPISODE_QUEUE_LEN);
	E.q_depth--;
	E.q_removed_total++;
}

/* Index within the queue of the oldest record of `session_id`, or -1. */
static int32_t find_session_index(uint8_t session_id)
{
	for (uint16_t i = 0; i < E.q_depth; i++) {
		uint16_t idx = (uint16_t)((E.q_tail + i) % RIDE_EPISODE_QUEUE_LEN);
		if (E.queue[idx].session_id == session_id) return (int32_t)i;
	}
	return -1;
}

uint16_t ride_episode_queue_count_session(uint8_t session_id)
{
	uint16_t n = 0;
	for (uint16_t i = 0; i < E.q_depth; i++) {
		uint16_t idx = (uint16_t)((E.q_tail + i) % RIDE_EPISODE_QUEUE_LEN);
		if (E.queue[idx].session_id == session_id) n++;
	}
	return n;
}

bool ride_episode_queue_peek_session(uint8_t session_id, ride_episode_result_t *out)
{
	if (out == 0) return false;
	int32_t off = find_session_index(session_id);
	if (off < 0) return false;
	*out = E.queue[(uint16_t)((E.q_tail + (uint16_t)off) % RIDE_EPISODE_QUEUE_LEN)];
	return true;
}

void ride_episode_queue_release_session(uint8_t session_id)
{
	int32_t off = find_session_index(session_id);
	if (off < 0) return;
	/*
	 * Sessions are dumped oldest-first, so in practice this record is at the head and the loop
	 * below does nothing. The general form is kept anyway: an interrupted dump can leave two
	 * sessions' records interleaved, and a removal that quietly took the wrong one would be
	 * exactly the sort of silent mix-up this card exists to eliminate.
	 */
	for (uint16_t i = (uint16_t)off; i > 0; i--) {
		uint16_t dst = (uint16_t)((E.q_tail + i) % RIDE_EPISODE_QUEUE_LEN);
		uint16_t src = (uint16_t)((E.q_tail + i - 1U) % RIDE_EPISODE_QUEUE_LEN);
		E.queue[dst] = E.queue[src];
	}
	E.q_tail = (uint16_t)((E.q_tail + 1U) % RIDE_EPISODE_QUEUE_LEN);
	E.q_depth--;
	E.q_removed_total++;
}

uint16_t ride_episode_queue_depth(void)
{
	return E.q_depth;
}

uint32_t ride_episode_queue_enqueued(void)
{
	return E.q_enqueued_total;
}

uint32_t ride_episode_queue_removed(void)
{
	return E.q_removed_total;
}

uint32_t ride_episode_queue_rejected(void)
{
	return E.q_rejected_total;
}

void ride_episode_get_result(ride_episode_result_t *out)
{
	if (out != 0) {
		*out = E.published;
	}
}

ride_episode_state_t ride_episode_get_state(void)
{
	return E.state;
}
