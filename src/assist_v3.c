/*
 * Assist Behavior V3 - transient manager and THE trajectory (ARCHITECTURE_V3.md 5, 6).
 * See inc/assist_v3.h for the contract. Milestone C rules; carry (Milestone D) is not here.
 *
 * Per call:
 *   1. rider intent (assist_v3_intent_update), with V3's OWN engaged flag selecting the EB74
 *      threshold 820 / zero+245 (re-check N4);
 *   2. base assist: env_equiv through the G5300 static characteristic, g53_static_target(), with
 *      this module's own G7 ratio-limiter state advanced per ELAPSED 10 ms (2.1, R1-#8);
 *   3. transient manager: start gate (6.2), crank-stop / reverse rules (6.1 Milestone C rows);
 *   4. the trajectory: y moves toward the target at exactly one rate (6.1 table).
 *
 * All rates are expressed per millisecond in "Q8.16" (Iq * 2^24 per ms) so that one fractional
 * accumulator serves every rate: the budget over `el` ticks is rate * el / 4 ms, and the sub-Q8
 * remainder is carried to the next call (no rate is lost to truncation at 4 kHz, where the D3E
 * rate is only ~29 Q8 per tick). Every product is formed in 64 bits.
 */
#include "assist_v3.h"

#include "assist_v3_intent.h"
#include "g53_port_chain.h"

#include <string.h>

#define V3_TICKS_PER_MS 4u
#define V3_P_MAX 8000                 /* E2->Q8 product stays in 32 bits up to this P          */
#define V3_E2_FULL 40960u             /* D7EC E2 full scale (= 0.65 * P Iq)                     */
#define V3_FRAC_SHIFT 18u             /* rate units: Q8 * 2^16 per ms, budget over ticks * 1/4   */
#define V3_EL_CAP 256u                /* one call never moves y by more than 64 ms of rate; the
                                       * G53 port drops catch-up beyond 64 logical ms the same way */
#define V3_RESP_SLOW_MS 600u          /* R(Response) full scale at 0 %   (candidate, 6.1)       */
#define V3_RESP_FAST_MS 150u          /* R(Response) full scale at 100 % (candidate, 6.1)       */

typedef struct {
	uint32_t y_q8;                    /* THE demand state: Iq Q8, pre-g1, pre-limits           */
	uint32_t frac;                    /* sub-Q8 remainder of the moving direction             */
	int8_t   dir;                     /* +1 rising, -1 falling, 0 holding (frac owner)        */
	uint8_t  ms_ticks;                /* 0..3 control ticks not yet worth a full ms           */
	g53_static_ratio_state_t ratio;   /* own G7 ratio limiter state (D+208 equivalent)        */
	bool     stop_target_zero;
	assist_v3_telemetry_t tlm;
} v3_state_t;

static v3_state_t V;

/* ---------------------------------------------------------------- pure helpers */

static int32_t clamp_p(int32_t p)
{
	if (p < 0) return 0;
	if (p > V3_P_MAX) return V3_P_MAX;
	return p;
}

uint32_t assist_v3_e2_to_iq_q8(uint16_t e2, int32_t phase_current_max)
{
	/* E2 -> EE Q12 -> BDE8 Q5A (cap 6500) -> m2aa -> Iq collapses to E2 * 0.65 * P / 40960
	 * (ARCHITECTURE_V3 5). In Q8: E2 * P * 0.65 * 256 / 40960 = E2 * P * 13 / 3200.
	 * 40960 * 8000 * 13 < 2^32. */
	const uint32_t e = e2 > V3_E2_FULL ? V3_E2_FULL : e2;
	return (e * (uint32_t)clamp_p(phase_current_max) * 13u) / 3200u;
}

uint16_t assist_v3_response_full_scale_ms(uint8_t response_pct)
{
	const uint32_t r = response_pct > 100u ? 100u : response_pct;
	return (uint16_t)(V3_RESP_SLOW_MS - ((V3_RESP_SLOW_MS - V3_RESP_FAST_MS) * r) / 100u);
}

/* Rate of an E2-per-10-ms step (the D7EC trajectory units: D+232 rise, D3E fall) in Q8.16/ms:
 * step * (0.65 P / 40960) Iq per 10 ms = step * P * 13 / 32000 Q8 per ms. */
static uint64_t rate_e2_per_10ms(uint32_t step, int32_t p)
{
	return ((uint64_t)step * (uint64_t)clamp_p(p) * 13u << 16) / 32000u;
}

/* BDE8 S5: 50 Q5C units per 1 ms on a 6500 = 0.65 P full scale, i.e. P / 200 Iq/ms (3.5 Iq/ms at
 * P = 700). Q8: P * 256 / 200 = P * 32 / 25. Pre-g1, like y (g1 multiplies after S5, audit A G13). */
static uint64_t rate_bde8(int32_t p)
{
	return ((uint64_t)clamp_p(p) * 32u << 16) / 25u;
}

/* R(Response): full scale 0.65 P in T ms. Q8: 0.65 * P * 256 / T = P * 832 / (5 T). */
static uint64_t rate_response(uint8_t response_pct, int32_t p)
{
	const uint32_t t = assist_v3_response_full_scale_ms(response_pct);
	return ((uint64_t)clamp_p(p) * 832u << 16) / (5u * t);
}

/* Move y toward target by at most rate (Q8.16/ms) over `el` ticks. One direction owns the
 * fractional remainder; a direction change or a reached target clears it, so a later move never
 * inherits budget from an earlier one. */
static void move_toward(uint32_t target, uint64_t rate, uint32_t el)
{
	const int8_t dir = target > V.y_q8 ? 1 : (target < V.y_q8 ? -1 : 0);
	if (dir != V.dir) V.frac = 0u;
	V.dir = dir;
	if (dir == 0) return;
	const uint64_t budget = rate * el + V.frac;     /* units: Q8 * 2^16 * (1/4 ms) */
	const uint64_t step = budget >> V3_FRAC_SHIFT;
	V.frac = (uint32_t)(budget & ((1u << V3_FRAC_SHIFT) - 1u));
	if (dir > 0) {
		const uint32_t room = target - V.y_q8;
		if (step >= room) { V.y_q8 = target; V.frac = 0u; V.dir = 0; }
		else V.y_q8 += (uint32_t)step;
	} else {
		const uint32_t room = V.y_q8 - target;
		if (step >= room) { V.y_q8 = target; V.frac = 0u; V.dir = 0; }
		else V.y_q8 -= (uint32_t)step;
	}
}

/* ---------------------------------------------------------------- lifecycle */

static void trajectory_reset(void)
{
	const uint32_t updates = V.tlm.updates;
	memset(&V, 0, sizeof(V));
	g53_static_ratio_init(&V.ratio);
	V.tlm.updates = updates;
}

void assist_v3_power_on(void)
{
	assist_v3_intent_power_on();
	trajectory_reset();
	V.tlm.updates = 0u;
}

void assist_v3_reset(void)
{
	assist_v3_intent_reset();
	trajectory_reset();
}

bool assist_v3_stop_target_zero(void) { return V.stop_target_zero; }
bool assist_v3_engaged(void) { return V.y_q8 > 0u; }
const assist_v3_telemetry_t *assist_v3_telemetry(void) { return &V.tlm; }

/* ---------------------------------------------------------------- one tick */

int32_t assist_v3_update(const assist_v3_input_t *in)
{
	if (in == 0) return (int32_t)(V.y_q8 >> 8);

	const assist_v3_intent_params_t *ip = assist_v3_intent_params();
	const uint32_t el_raw = in->elapsed_ticks ? in->elapsed_ticks : 1u;
	const uint32_t el = el_raw > V3_EL_CAP ? V3_EL_CAP : el_raw;
	const int32_t p = clamp_p(in->phase_current_max);
	const uint8_t slot = g53_chain_level(in->level);
	/* V3's own engaged flag (re-check N4): the EB74 threshold follows OUR demand, never the shadow
	 * D7EC's, exactly as EB74 follows the D7EC trajectory output it feeds (820 while it drives). */
	const bool engaged = V.y_q8 > 0u;
	const uint16_t thr = engaged ? ip->eb74_thr_engaged :
		(uint16_t)(in->eb74_zero + ip->eb74_deadband);

	/* 1. rider intent ------------------------------------------------------------------ */
	assist_v3_intent_in_t ii;
	assist_v3_intent_out_t io;
	memset(&ii, 0, sizeof(ii));
	ii.elapsed_ticks = el_raw;
	ii.now_tick = in->now_tick;
	ii.load_ctrl = in->load_ctrl;
	ii.torque_valid = in->torque_valid;
	ii.crank_steps = in->crank_steps;
	ii.crank_step_tick = in->crank_step_tick;
	ii.pas_glitch = in->pas_glitch;
	ii.cadence_rpm = in->cadence_rpm;
	ii.g53_true_stop = in->g53_true_stop;
	ii.real_stop = in->real_stop;
	ii.direction_inhibit = in->direction_inhibit;
	ii.inhibit_is_reverse = in->inhibit_is_reverse;
	ii.eb74_zero = in->eb74_zero;
	ii.eb74_armed = in->eb74_armed;
	ii.v3_engaged = engaged;
	assist_v3_intent_update(&ii, &io);

	/* 2. base assist: the G5300 characteristic as a pure function (section 5). Called every
	 * tick so the ratio limiter advances with elapsed time whatever the gates below decide, as
	 * D7EC runs its limiter every 10 ms whatever the trajectory does. Input is env_equiv, never I. */
	uint32_t ms;
	{
		const uint32_t t = (uint32_t)V.ms_ticks + el_raw;
		ms = t / V3_TICKS_PER_MS;
		V.ms_ticks = (uint8_t)(t % V3_TICKS_PER_MS);
	}
	g53_static_input_t si;
	g53_static_diag_t sd;
	si.env = io.env_equiv;
	si.cadence = in->cadence_rpm;
	si.lut_cadence = in->lut_cadence;
	si.speed_native = in->speed_native;
	si.level = slot;
	const uint16_t e2 = g53_static_target(&si, &V.ratio, ms, &sd);
	const uint32_t map_q8 = assist_v3_e2_to_iq_q8(e2, p);

	/* 3. transient manager -------------------------------------------------------------- */
	/* Load released = what the D7EC zero-reset tests (cur == 0): the EB74 output of the CURRENT
	 * load with V3's active threshold is 0. Raw load, not the intent: at a stop the intent holds
	 * the last effort by design (4.4 PEDAL_STOP), the pedal itself is what was released. */
	const bool load_released =
		assist_v3_eb74_active(in->torque_valid ? in->load_ctrl : 0u, thr) == 0u;
	/* Crank stopped: the intent's PEDAL_STOP (no step for T_stop, reverse restart), or either
	 * stop authority the baseline answers to (G53 true-stop, native real_stop). The stop is
	 * "confirmed" - the point where baseline D7EC loses readiness and its target collapses - only
	 * by the latter two (R1-#9). */
	const bool stop_confirmed = in->g53_true_stop || in->real_stop;
	const bool crank_stopped = io.release_class == ASSIST_V3_CLASS_PEDAL_STOP || stop_confirmed;
	/* Reverse: the G53 signed cadence (what makes D7EC hard-clear) or the native reverse
	 * inhibit. Baseline: D7EC hard-clears and BDE8 leaves state 7 with one -50 step, then zeroes Q5C
	 * (m2aa 2385 -> 2335 -> 0 within 3 logical ms, SIL reverse_60 [SIM]); the published Iq then
	 * follows the 6.84 Iq/ms fast slew. Milestone C reverse <= baseline: y is zeroed at once and the
	 * fast slew shapes the published Iq exactly as at baseline (G1-STOP). */
	const bool reverse = in->cadence_rpm < 0 || in->g53_reverse ||
	                     (in->direction_inhibit && in->inhibit_is_reverse);
	/* Start readiness (6.2): the D7EC readiness rule with the chain's own thresholds, applied to
	 * V3's forward step evidence (crank_phase.c, D-006) and env_equiv. */
	uint16_t ready_env = 0u;
	uint8_t ready_evid = 0u;
	g53_chain_d7ec_readiness(&ready_env, &ready_evid);
	const bool ready = io.steps_since_restart >= ready_evid || io.env_equiv >= ready_env;

	uint32_t target;
	uint64_t rate = 0u;
	uint8_t mode;
	if (p <= 0) {
		/* No current scale: nothing to command and no rate exists. */
		V.y_q8 = 0u; V.frac = 0u; V.dir = 0;
		target = 0u;
		mode = ASSIST_V3_RATE_HOLD;
	} else if (reverse) {
		V.y_q8 = 0u; V.frac = 0u; V.dir = 0;
		target = 0u;
		mode = ASSIST_V3_RATE_REVERSE;
	} else if (crank_stopped) {
		if (load_released && stop_confirmed) {
			/* Legacy stop, load released, stop confirmed (R1-#9, measured): at the G53 PAS true-stop
			 * the D7EC zero-reset drops drive permission and BDE8 leaves state 7 with one or two -50
			 * steps, then zeroes Q5C (request 0 within ~12 ms of the true-stop, SIL attack_stop_60
			 * [SIM]); the fast slew shapes the published Iq. y is zeroed at once so the V3 stop is
			 * never later than baseline (G1-STOP). */
			V.y_q8 = 0u; V.frac = 0u; V.dir = 0;
			target = 0u;
			mode = ASSIST_V3_RATE_STOP_RELEASED;
		} else if (load_released) {
			/* V3's own PEDAL_STOP before the stop is confirmed: already earlier than baseline,
			 * which still holds here; fall at the BDE8 rate (-50/ms). */
			target = 0u;
			rate = rate_bde8(p);
			mode = ASSIST_V3_RATE_STOP_RELEASED;
		} else if (stop_confirmed) {
			/* readiness lost -> target 0 -> E2 falls at D3E (legacy stop, R1-#9). */
			const int16_t d3e = g53_chain_d7ec_fall();
			target = 0u;
			rate = rate_e2_per_10ms((uint32_t)(d3e < 0 ? -d3e : d3e), p);
			mode = ASSIST_V3_RATE_STOP_HELD;
		} else {
			/* Crank not moving, load held, stop not yet confirmed: baseline still holds its
			 * D7EC target here, so y holds. Never a rise at a stopped crank. */
			target = V.y_q8;
			mode = ASSIST_V3_RATE_STOP_WAIT;
		}
	} else if (!engaged && !(io.engage_ok && ready)) {
		/* From 0, V3 engages only past the active EB74 engage threshold with EB74 armed
		 * (io.engage_ok: env_equiv > 0 at zero+245, R1-#14) and the D7EC readiness (6.2). */
		target = 0u;
		mode = ASSIST_V3_RATE_START_BLOCKED;
	} else {
		target = map_q8;
		if (target > V.y_q8) {
			/* Legacy attack: the level's D7EC accel rise (D+232), capped by BDE8 50/ms. */
			const uint64_t r_lvl = rate_e2_per_10ms(g53_chain_d7ec_rise(slot), p);
			const uint64_t r_bde8 = rate_bde8(p);
			rate = r_lvl < r_bde8 ? r_lvl : r_bde8;
			mode = ASSIST_V3_RATE_RISE;
		} else {
			/* NORMAL / gradual / TRUE_RELEASE while pedalling: the user's Response. */
			rate = rate_response(in->response_pct, p);
			mode = ASSIST_V3_RATE_FALL_RESPONSE;
		}
	}

	/* 4. THE trajectory: one state, one rate ------------------------------------------- */
	move_toward(target, rate, el);
	/* Telemetry: a pedalling rule that has reached its target is reported as a hold; the stop,
	 * reverse and start rules keep their name while they are in force. */
	if ((mode == ASSIST_V3_RATE_RISE || mode == ASSIST_V3_RATE_FALL_RESPONSE) && V.y_q8 == target)
		mode = ASSIST_V3_RATE_HOLD;

	/* The standstill-predicate term (2.1, N2): released load, or our own stop ramp done. */
	V.stop_target_zero = load_released || (V.y_q8 == 0u && target == 0u);

	/* telemetry (never read back) ------------------------------------------------------- */
	{
		assist_v3_telemetry_t *t = &V.tlm;
		t->intent = io.intent;
		t->env_equiv = io.env_equiv;
		t->kappa_q12 = io.kl_q12;
		t->e_short = io.e_short;
		t->e_long = io.e_long;
		t->template_conf_q12 = io.confidence_q12;
		t->expected_effort = io.expected_effort;
		t->phase = io.phase;
		t->release_class = io.release_class;
		t->phase_aligned = io.phase_aligned;
		t->template_mode = io.template_mode;
		t->base_target_e2 = e2;
		t->applied_ratio = sd.applied_ratio;
		t->base_target_iq = (int32_t)(map_q8 >> 8);
		t->target_iq = (int32_t)(target >> 8);
		t->v3_demand_iq = (int32_t)(V.y_q8 >> 8);
		t->cadence_rpm = in->cadence_rpm;
		t->rate_mode = mode;
		t->response_pct = in->response_pct;
		t->engaged = engaged;
		t->crank_stopped = crank_stopped;
		t->stop_target_zero = V.stop_target_zero;
		t->imu_valid = in->motion.valid;
		t->engine_active = in->engine_active;
		t->engine_requested = in->engine_requested;
		t->updates++;
	}
	return (int32_t)(V.y_q8 >> 8);
}
