/*
 * QZERO: Quiet Zero - controlled PI integral fade after the final Iq reference reaches zero.
 *
 * WHAT THIS SUITE PROVES, and against what.
 *
 * The state machine itself (src/quiet_zero.c) is linked and executed - every entry, exit, fade,
 * hold and abort check below runs the shipped code, not a model of it. The mailbox integration
 * (T10) links the REAL 16 kHz slew owner (src/fast_iq_slew.c) and drives a real 650 ms release
 * and a real fall-to-zero through it, so "the tick Iq_ref first becomes exactly 0" is the
 * production definition rather than a hand-written one.
 *
 * The EFFECT on the regulators (T11/T12) is measured against a byte-faithful replica of FOC.c's
 * PI_control() - the same technique, and the same replica, stopclick_c1_pi_integral_host.c and
 * focaw1_tracking_aw_host.c already use, for the same reason: main.c/FOC.c are the ARM entry
 * point and ISR core and cannot be linked on a host. A source guard pins the replica to FOC.c's
 * real body, so a change to the regulator that the replica does not track fails loudly here.
 *
 * The wiring (T13/T14) is proven by source-text guards over main.c and ride_control.c, in the
 * read/strip/goto-done shape armed_zero_lifecycle_host.c established.
 *
 * The physical claim behind the card, stated once: in persistent ARMED_ZERO the bridge is on and
 * the FOC ISR keeps regulating, so a CONTINUOUS integrator settles at the only equilibrium that
 * produces zero current in a spinning machine - u_q ~= BEMF. That is a free-wheel: no winding
 * current, no braking torque, a long run-on. T11 reproduces exactly that with the real regulator
 * math and then shows the fade removing it without touching PI.out, the reference or the P path.
 */

#include "../common/check.h"

#include "../../inc/fast_iq_slew.h"
#include "../../inc/quiet_zero.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STRINGIZE2(x) #x
#define STRINGIZE(x) STRINGIZE2(x)

#ifndef MAIN_C_PATH
#error "MAIN_C_PATH must be supplied by run-host-tests.ps1"
#endif
#ifndef FOC_C_PATH
#error "FOC_C_PATH must be supplied by run-host-tests.ps1"
#endif
#ifndef ASSIST_PIPELINE_C_PATH
#error "ASSIST_PIPELINE_C_PATH must be supplied by run-host-tests.ps1"
#endif
#ifndef RIDE_CONTROL_C_PATH
#error "RIDE_CONTROL_C_PATH must be supplied by run-host-tests.ps1"
#endif
#ifndef FOC_CURRENT_LOOP_C_PATH
#error "FOC_CURRENT_LOOP_C_PATH must be supplied by run-host-tests.ps1"
#endif

/* Same production numbers the other PI suites hardcode, and for the same reason (config.h/FOC.h
 * pull in hardware context this harness does not have): P_FACTOR_I_Q=1.5, I_FACTOR_I_Q=0.01,
 * limit_i=limit_output=_U_MAX=1920, max_step=15. PH_CURRENT_MAX=700 -> QZERO_ABORT_CURRENT=350. */
#define MODEL_GAIN_P     1.5f
#define MODEL_GAIN_I     0.01f
#define MODEL_LIMIT_I    1920
#define MODEL_LIMIT_OUT  1920
#define MODEL_MAX_STEP   15
#define MODEL_PH_CURRENT_MAX 700
#define MODEL_ABORT_CURRENT (MODEL_PH_CURRENT_MAX >> 1)
/* RIDE_COAST_RELEASE_ERPS from inc/config.h; CRUISE_ERPS is any speed comfortably above it
 * (93 erps = 70 chainring rpm, erps = chainring rpm x 4/3 - a normal riding cadence). */
#define MODEL_MIN_BRAKE_ERPS 10
#define MODEL_CRUISE_ERPS    93

/* ------------------------------------------------------------------------------------------- */
/* Byte-faithful replica of FOC.c's PI_control() - transcribed, pinned by a source guard below. */
/* ------------------------------------------------------------------------------------------- */

typedef struct {
	float   gain_p;
	float   gain_i;
	int16_t limit_i;
	int16_t limit_output;
	int16_t recent_value;
	int32_t setpoint;
	float   integral_part;
	int16_t max_step;
	int32_t out;
	int32_t aw_sat_error;
	int32_t aw_inv_kp_q15;
} model_pi_t;

static void model_pi_init(model_pi_t *pi)
{
	memset(pi, 0, sizeof(*pi));
	pi->gain_p = MODEL_GAIN_P;
	pi->gain_i = MODEL_GAIN_I;
	pi->limit_i = MODEL_LIMIT_I;
	pi->limit_output = MODEL_LIMIT_OUT;
	pi->max_step = MODEL_MAX_STEP;
}

static int32_t model_pi_control(model_pi_t *PI_c)
{
	float Delta = (float)(PI_c->setpoint - PI_c->recent_value);
	float p_part = Delta * PI_c->gain_p;
	float aw_part = (float)((PI_c->aw_sat_error * PI_c->aw_inv_kp_q15) >> 15);
	PI_c->integral_part += (Delta - aw_part) * PI_c->gain_i;

	if (PI_c->integral_part > PI_c->limit_i) PI_c->integral_part = PI_c->limit_i;
	if (PI_c->integral_part < -(PI_c->limit_i)) PI_c->integral_part = -(PI_c->limit_i);

	if (p_part + PI_c->integral_part > PI_c->out + PI_c->max_step) PI_c->out += PI_c->max_step;
	else if (p_part + PI_c->integral_part < PI_c->out - PI_c->max_step) PI_c->out -= PI_c->max_step;
	else PI_c->out = (int32_t)(p_part + PI_c->integral_part);

	if (PI_c->out > PI_c->limit_output) PI_c->out = PI_c->limit_output;
	if (PI_c->out < -(PI_c->limit_output)) PI_c->out = -(PI_c->limit_output);

	return PI_c->out;
}

/*
 * Minimal first-order q-axis machine, only as strong as the claim needs: the current that flows
 * is what the applied voltage does NOT spend on the back-EMF, through the winding.
 *
 *   Iq_next = Iq + ((u_q - bemf) / r_counts - Iq) * alpha
 *
 * r_counts converts the voltage domain (_U_MAX = 1920 full scale) into the current domain
 * (PH_CURRENT_MAX = 700 full scale); alpha is the electrical lag over one 62.5 us tick. Neither
 * number is a claim about this motor - the properties under test (where the integrator settles,
 * and what happens to the applied voltage when it is faded out) hold for any positive pair.
 */
typedef struct {
	float iq;
	float bemf;
	float r_counts;
	float alpha;
} model_machine_t;

static void model_machine_init(model_machine_t *m, float bemf)
{
	m->iq = 0.0f;
	m->bemf = bemf;
	m->r_counts = 2.0f;
	m->alpha = 0.25f;
}

static void model_machine_step(model_machine_t *m, float u_q)
{
	float iq_ss = (u_q - m->bemf) / m->r_counts;
	m->iq += (iq_ss - m->iq) * m->alpha;
}

/* ------------------------------------------------------------------------------------------- */
/* Helpers                                                                                      */
/* ------------------------------------------------------------------------------------------- */

static quiet_zero_input_t make_input(int32_t iq_ref, bool quiet, float iq_i, float id_i)
{
	quiet_zero_input_t in;
	memset(&in, 0, sizeof(in));
	in.iq_ref = iq_ref;
	in.zero_policy_quiet = quiet;
	in.iq_measured = 0;
	in.id_measured = 0;
	in.abort_current = MODEL_ABORT_CURRENT;
	in.rotor_erps = MODEL_CRUISE_ERPS;
	in.speed_fresh = true;
	in.min_brake_erps = MODEL_MIN_BRAKE_ERPS;
	in.iq_integral = iq_i;
	in.id_integral = id_i;
	return in;
}

/* Drive one qualifying release edge: one tick at Iq_ref>0, then the first zero tick. */
static void enter_qzero(quiet_zero_t *qz, quiet_zero_action_t *out, float iq_i, float id_i)
{
	quiet_zero_input_t in = make_input(40, true, iq_i, id_i);
	quiet_zero_tick(qz, &in, out);
	in = make_input(0, true, iq_i, id_i);
	quiet_zero_tick(qz, &in, out);
}

/* ------------------------------------------------------------------------------------------- */
/* T1..T9: the state machine, against the shipped module                                        */
/* ------------------------------------------------------------------------------------------- */

static void state_machine_checks(void)
{
	/* ---- T1: entry requires the non-zero -> exact zero edge AND a QUIET policy ---- */
	{
		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		quiet_zero_input_t in = make_input(40, true, 900.0f, -20.0f);
		quiet_zero_tick(&qz, &in, &out);
		CHECK(out.state == (uint32_t)QZERO_INACTIVE && !out.apply_integral && !out.freeze_aw,
			"T1: a positive reference is not QZERO's business - nothing applied, nothing frozen");

		in = make_input(0, true, 900.0f, -20.0f);
		quiet_zero_tick(&qz, &in, &out);
		CHECK(out.state == (uint32_t)QZERO_BLEND && out.entered && out.apply_integral &&
			out.freeze_aw && out.clear_aw_edge,
			"T1: the first exact-zero tick of a QUIET release enters BLEND, freezes AW, clears the residual");
		CHECK(qz.entries == 1U && qz.aborts == 0U,
			"T1: exactly one entry counted, no abort");
	}

	/* ---- T2: a zero the producer did not mark QUIET is left entirely alone ---- */
	{
		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		quiet_zero_input_t in = make_input(40, false, 900.0f, -20.0f);
		quiet_zero_tick(&qz, &in, &out);
		in = make_input(0, false, 900.0f, -20.0f);
		quiet_zero_tick(&qz, &in, &out);
		CHECK(out.state == (uint32_t)QZERO_INACTIVE && !out.entered && !out.apply_integral &&
			!out.freeze_aw,
			"T2: a limiter/service zero (policy NONE) never arms QZERO - the ordinary PI keeps the axis");
		bool stayed = true;
		for (int i = 0; i < 5000; ++i) {
			quiet_zero_tick(&qz, &in, &out);
			if (out.state != (uint32_t)QZERO_INACTIVE || out.apply_integral) stayed = false;
		}
		CHECK(stayed && qz.entries == 0U,
			"T2: policy NONE stays inactive for as long as the reference stays zero");
	}

	/* ---- T3 (QZERO-2, was the opposite): a reference that was ALREADY zero when the rider
	 *          stopped pedalling MUST arm, on the rising edge of the QUIET verdict.
	 *
	 *          v1 treated this as out of scope, and on the bike that was the whole complaint:
	 *          the stop behaved differently depending on whether the current happened to reach
	 *          zero from the release itself or from a limiter, a closing torque gate, or simply
	 *          easing off before the cranks stopped. Owner decision 2026-09-03 - the stop must be
	 *          the same every time. The verdict is still only ever granted after pedalling has
	 *          ended, so "never a limiter" is unchanged: a limiter alone never sets it.
	 *
	 *          The invariant that survives is "only an EDGE arms it", which the second half
	 *          checks: a verdict that was already standing arms nothing further. ---- */
	{
		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		quiet_zero_input_t in = make_input(0, false, 900.0f, -20.0f);
		for (int i = 0; i < 10; ++i) quiet_zero_tick(&qz, &in, &out);
		CHECK(qz.entries == 0U, "T3: a plain zero with no QUIET verdict still arms nothing");
		in = make_input(0, true, 900.0f, -20.0f);
		quiet_zero_tick(&qz, &in, &out);
		CHECK(qz.entries == 1U && out.state != (uint32_t)QZERO_INACTIVE,
			"T3: the rising edge of the QUIET verdict arms QZERO even with no reference edge");
		uint32_t entries_after = qz.entries;
		for (int i = 0; i < 200; ++i) quiet_zero_tick(&qz, &in, &out);
		CHECK(qz.entries == entries_after,
			"T3: a verdict that is already standing does not arm again - only an edge arms");
	}

	/* ---- T4: the fade is linear, lasts exactly QZERO_BLEND_TICKS, and ends at exact 0.0f ---- */
	{
		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		const float iq0 = 900.0f, id0 = -240.0f;
		quiet_zero_input_t in = make_input(40, true, iq0, id0);
		quiet_zero_tick(&qz, &in, &out);

		in = make_input(0, true, iq0, id0);
		bool linear = true, monotone = true, still_blending = true;
		float prev_iq = iq0, prev_id = id0;
		for (uint32_t k = 1; k <= QZERO_BLEND_TICKS; ++k) {
			quiet_zero_tick(&qz, &in, &out);
			float expect_scale = (float)(QZERO_BLEND_TICKS - k) / (float)QZERO_BLEND_TICKS;
			float expect_iq = iq0 * expect_scale;
			float expect_id = id0 * expect_scale;
			if (fabsf(out.iq_integral - expect_iq) > 1e-3f) linear = false;
			if (fabsf(out.id_integral - expect_id) > 1e-3f) linear = false;
			if (fabsf(out.iq_integral) > fabsf(prev_iq) + 1e-6f) monotone = false;
			if (fabsf(out.id_integral) > fabsf(prev_id) + 1e-6f) monotone = false;
			prev_iq = out.iq_integral; prev_id = out.id_integral;
			if (k < QZERO_BLEND_TICKS && out.state != (uint32_t)QZERO_BLEND) still_blending = false;
			if (!out.apply_integral || !out.freeze_aw) still_blending = false;
		}
		CHECK(linear, "T4: both integrals follow entry*(N-k)/N exactly, on every one of the 160 ticks");
		CHECK(monotone, "T4: the fade only ever moves both integrals toward zero, never away");
		CHECK(still_blending, "T4: BLEND holds for the whole fade and freezes AW on every tick of it");
		CHECK(out.iq_integral == 0.0f && out.id_integral == 0.0f,
			"T4: the last fade point is EXACT zero in both axes, not an accumulated residue");
		CHECK(out.state == (uint32_t)QZERO_HOLD,
			"T4: the tick that reaches exact zero is also the first HOLD tick");
		CHECK(QZERO_BLEND_TICKS == 160U,
			"T4: the fade is 160 ticks = 10.0 ms at 16 kHz, as the card specifies");
	}

	/* ---- T5: HOLD pins both integrals at exact zero for as long as the reference is zero ---- */
	{
		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		enter_qzero(&qz, &out, 1500.0f, 300.0f);
		quiet_zero_input_t in = make_input(0, true, 1500.0f, 300.0f);
		for (uint32_t k = 1; k < QZERO_BLEND_TICKS; ++k) quiet_zero_tick(&qz, &in, &out);

		bool pinned = true;
		for (int i = 0; i < 160000; ++i) {   /* 10 s of standing still at zero torque */
			quiet_zero_tick(&qz, &in, &out);
			if (out.state != (uint32_t)QZERO_HOLD) pinned = false;
			if (out.iq_integral != 0.0f || out.id_integral != 0.0f) pinned = false;
			if (!out.apply_integral || !out.freeze_aw) pinned = false;
		}
		CHECK(pinned, "T5: HOLD keeps both integrals at exact zero and AW frozen indefinitely");
		CHECK(qz.aborts == 0U, "T5: a genuinely quiet hold never trips the abort guard");
	}

	/* ---- T6: exit is immediate on the first positive reference, and applies nothing ---- */
	{
		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		enter_qzero(&qz, &out, 900.0f, -20.0f);
		quiet_zero_input_t in = make_input(0, true, 900.0f, -20.0f);
		for (int i = 0; i < 500; ++i) quiet_zero_tick(&qz, &in, &out);   /* well into HOLD */

		in = make_input(1, true, 0.0f, 0.0f);   /* the very first count of a new pull */
		quiet_zero_tick(&qz, &in, &out);
		CHECK(out.state == (uint32_t)QZERO_INACTIVE && out.exited && out.clear_aw_edge,
			"T6: Iq_ref>0 leaves QZERO on the same tick and clears the AW residual once");
		CHECK(!out.apply_integral && !out.freeze_aw,
			"T6: after the exit tick the integrators are entirely their own owners again");

		in = make_input(200, true, 0.0f, 0.0f);
		quiet_zero_tick(&qz, &in, &out);
		CHECK(!out.exited && !out.apply_integral && !out.freeze_aw && !out.clear_aw_edge,
			"T6: the residual clear is an EDGE - it does not repeat while the demand runs");
	}

	/* ---- T7: the abort guard, both axes, both signs, and it does not re-arm ---- */
	{
		const char *why_q = "T7: |Iq| at the threshold during BLEND ends the P-only hold";
		const char *why_d = "T7: |Id| at the threshold does the same on the d axis";
		for (int axis = 0; axis < 2; ++axis) {
			for (int sign = -1; sign <= 1; sign += 2) {
				quiet_zero_t qz; quiet_zero_reset(&qz);
				quiet_zero_action_t out;
				enter_qzero(&qz, &out, 900.0f, -20.0f);
				quiet_zero_input_t in = make_input(0, true, 900.0f, -20.0f);
				for (int i = 0; i < 20; ++i) quiet_zero_tick(&qz, &in, &out);
				CHECK(out.state == (uint32_t)QZERO_BLEND,
					"T7: setup - still fading before the guard fires");

				if (axis == 0) in.iq_measured = sign * MODEL_ABORT_CURRENT;
				else in.id_measured = sign * MODEL_ABORT_CURRENT;
				quiet_zero_tick(&qz, &in, &out);
				CHECK(out.state == (uint32_t)QZERO_INACTIVE && out.aborted &&
					!out.apply_integral && !out.freeze_aw && out.clear_aw_edge && qz.aborts == 1U,
					(axis == 0) ? why_q : why_d);

				/* No re-arm: the reference is still zero, so there is no new release edge. */
				in.iq_measured = 0; in.id_measured = 0;
				bool rearmed = false;
				for (int i = 0; i < 2000; ++i) {
					quiet_zero_tick(&qz, &in, &out);
					if (out.state != (uint32_t)QZERO_INACTIVE || out.apply_integral) rearmed = true;
				}
				CHECK(!rearmed && qz.entries == 1U,
					"T7: an aborted release does not re-arm - only a new release edge can");
			}
		}

		/* One count below the threshold is NOT an abort: the guard is a bound, not a hair trigger. */
		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		enter_qzero(&qz, &out, 900.0f, -20.0f);
		quiet_zero_input_t in = make_input(0, true, 900.0f, -20.0f);
		in.iq_measured = -(MODEL_ABORT_CURRENT - 1);
		in.id_measured = MODEL_ABORT_CURRENT - 1;
		bool held = true;
		for (int i = 0; i < 1000; ++i) {
			quiet_zero_tick(&qz, &in, &out);
			if (out.state == (uint32_t)QZERO_INACTIVE) held = false;
		}
		CHECK(held && qz.aborts == 0U,
			"T7: one count below the threshold keeps the hold - braking current is the point of it");
	}

	/* ---- T8: continuation is not gated by the policy. A mode change that keeps the reference at
	 *          zero (HOLD, a fall to zero, FW-048's coast FORCE_ZERO) must not drop the hold. ---- */
	{
		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		enter_qzero(&qz, &out, 900.0f, -20.0f);
		quiet_zero_input_t in = make_input(0, false, 900.0f, -20.0f);   /* policy now NONE */
		bool kept = true;
		for (int i = 0; i < 5000; ++i) {
			quiet_zero_tick(&qz, &in, &out);
			if (out.state == (uint32_t)QZERO_INACTIVE || !out.apply_integral) kept = false;
		}
		CHECK(kept && out.state == (uint32_t)QZERO_HOLD,
			"T8: the policy gates ENTRY only - a later FORCE_ZERO/HOLD at zero does not end the hold");
	}

	/* ---- T9: reset drops every field, including the counters ---- */
	{
		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		enter_qzero(&qz, &out, 900.0f, -20.0f);
		quiet_zero_input_t in = make_input(0, true, 900.0f, -20.0f);
		for (int i = 0; i < 500; ++i) quiet_zero_tick(&qz, &in, &out);
		CHECK(qz.state == (uint32_t)QZERO_HOLD && qz.hold_ticks > 0U,
			"T9: setup - a live hold exists");
		quiet_zero_reset(&qz);
		CHECK(qz.state == (uint32_t)QZERO_INACTIVE && qz.blend_tick == 0U &&
			qz.iq_integral_entry == 0.0f && qz.id_integral_entry == 0.0f &&
			qz.prev_iq_ref == 0 && qz.entries == 0U && qz.aborts == 0U && qz.hold_ticks == 0U,
			"T9: quiet_zero_reset() leaves nothing behind for the next bridge run to inherit");

		/*
		 * And a reset state cannot enter without a fresh edge, even at zero with a QUIET policy.
		 * QZERO-2 made this sharper rather than weaker: the reset seeds the verdict history HIGH
		 * precisely so that the reset - which runs where the bridge is being switched off - can
		 * never manufacture the rising edge that would arm a fade nobody asked for.
		 */
		for (int i = 0; i < 100; ++i) quiet_zero_tick(&qz, &in, &out);
		CHECK(qz.entries == 0U && out.state == (uint32_t)QZERO_INACTIVE,
			"T9: after a reset, a standing QUIET verdict at zero still does not arm QZERO");
	}
}

/* ------------------------------------------------------------------------------------------- */
/* T15: the low-speed handback (FW-048's angle-switch zone must stay current-free)               */
/* ------------------------------------------------------------------------------------------- */

static void low_speed_handback_checks(void)
{
	/* ---- T15a: a hold in progress is handed back as soon as the rotor reaches the zone ---- */
	{
		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		enter_qzero(&qz, &out, 900.0f, -20.0f);
		quiet_zero_input_t in = make_input(0, true, 900.0f, -20.0f);
		for (int i = 0; i < 400; ++i) quiet_zero_tick(&qz, &in, &out);
		CHECK(out.state == (uint32_t)QZERO_HOLD, "T15a: setup - a live hold at cruise speed");

		/* One erps below the threshold: the angle is about to change formula. */
		in.rotor_erps = MODEL_MIN_BRAKE_ERPS - 1;
		quiet_zero_tick(&qz, &in, &out);
		CHECK(out.state == (uint32_t)QZERO_HANDBACK && out.low_speed_release &&
			out.clear_aw_edge && qz.low_speed_exits == 1U,
			"T15a: below the threshold the handback BEGINS - reported once, on the tick the rotor "
			"enters the zone");
		/*
		 * FW-136.2 changed this from a step into a fade, and the assertions changed with it.
		 *
		 * QZERO-3 still decides WHAT to hand back: back-EMF is proportional to speed, so the
		 * integral that nulls the current is the entry integral scaled by how much the rotor has
		 * slowed. Handing back a zero instead leaves the applied voltage far below the back-EMF
		 * and braking current keeps flowing through the 60-degree stepping zone.
		 *
		 * What FW-136.2 adds is HOW. Handing the whole value over in one tick removes a real
		 * braking torque instantaneously, and a torque step is a click - which is exactly what
		 * the rider reported hearing at the handback point once FW-136 moved it up to half the
		 * release speed, where there is still torque to remove. So the value is now faded in,
		 * and what this test proves is that the ramp is monotonic and ends on the right value.
		 */
		{
			float expected = 900.0f * (float)(MODEL_MIN_BRAKE_ERPS - 1) /
				(float)MODEL_CRUISE_ERPS;
			CHECK(out.apply_integral && out.iq_integral > 0.0f && out.iq_integral < expected * 0.05f,
				"T15a: the first tick of the fade applies only a sliver of the target, not the lot");

			float prev = out.iq_integral;
			bool monotonic = true;
			int fade_ticks = 1;
			while (out.state == (uint32_t)QZERO_HANDBACK && fade_ticks < 5000) {
				quiet_zero_tick(&qz, &in, &out);
				fade_ticks++;
				if (out.iq_integral < prev) monotonic = false;
				prev = out.iq_integral;
			}
			printf("  T15a handback fade completed in %d ticks (%.1f ms at 16 kHz)\n",
				fade_ticks, (double)fade_ticks / 16.0);
			CHECK(monotonic,
				"T15a: the fade only ever rises - a dip would be a torque reversal, audible for "
				"the same reason the step was");
			CHECK(fade_ticks == (int)QZERO_HANDBACK_FADE_TICKS,
				"T15a: and it takes exactly the configured fade, so changing the constant changes "
				"the behaviour rather than being absorbed somewhere");
			CHECK(out.state == (uint32_t)QZERO_INACTIVE && !out.freeze_aw,
				"T15a: at the end the axis belongs to the full zero-current PI again");
			CHECK(out.iq_integral > expected * 0.99f && out.iq_integral < expected * 1.01f,
				"T15a: and the value it ends on is the back-EMF match for the handback speed");
			CHECK(out.id_integral == 0.0f,
				"T15a: the d axis is handed back at zero - with Iq already null there is nothing to cancel");
			CHECK(out.iq_integral < 900.0f && out.iq_integral > 0.0f,
				"T15a: sanity - the seed is between zero and the entry value, never above it");
		}
		CHECK(!out.aborted && qz.aborts == 0U,
			"T15a: this is a normal end of the release, not the overcurrent abort");
		/* And it does not re-arm while the rotor coasts out - there is no new release edge. */
		bool rearmed = false;
		for (int i = 0; i < 4000; ++i) {
			quiet_zero_tick(&qz, &in, &out);
			if (out.state != (uint32_t)QZERO_INACTIVE || out.apply_integral) rearmed = true;
		}
		CHECK(!rearmed && qz.entries == 1U,
			"T15a: the last stretch of the coast stays un-braked - no re-arm without a new release");
	}

	/* ---- T15b: exactly at the threshold the hold is still allowed (the margin is inclusive) ---- */
	{
		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		quiet_zero_input_t in = make_input(40, true, 900.0f, 0.0f);
		in.rotor_erps = MODEL_MIN_BRAKE_ERPS;
		quiet_zero_tick(&qz, &in, &out);
		in.iq_ref = 0;
		quiet_zero_tick(&qz, &in, &out);
		CHECK(out.state == (uint32_t)QZERO_BLEND && out.entered,
			"T15b: at exactly the threshold the hold is permitted - the boundary is not off by one");
	}

	/* ---- T15c: a release that COMPLETES near standstill never arms the hold at all ---- */
	{
		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		quiet_zero_input_t in = make_input(40, true, 900.0f, 0.0f);
		in.rotor_erps = 4;   /* inside the six-step zone: the angle jump is imminent */
		quiet_zero_tick(&qz, &in, &out);
		in.iq_ref = 0;
		bool armed = false;
		for (int i = 0; i < 2000; ++i) {
			quiet_zero_tick(&qz, &in, &out);
			if (out.state != (uint32_t)QZERO_INACTIVE || out.apply_integral) armed = true;
		}
		CHECK(!armed && qz.entries == 0U && qz.low_speed_exits == 0U,
			"T15c: a release finishing near standstill is left entirely to the ordinary PI");
	}

	/* ---- T15d: the whole spin-down in order - brake at speed, hand back before the angle jump ---- */
	{
		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		quiet_zero_input_t in = make_input(40, true, 1200.0f, 0.0f);
		in.rotor_erps = MODEL_CRUISE_ERPS;
		quiet_zero_tick(&qz, &in, &out);
		in.iq_ref = 0;

		int braked_ticks = 0, handback_tick = -1;
		/* 93 erps down to 0 over 24000 ticks (1.5 s at 16 kHz), roughly linear. */
		for (int t = 1; t <= 24000; ++t) {
			in.rotor_erps = MODEL_CRUISE_ERPS - (MODEL_CRUISE_ERPS * t) / 24000;
			quiet_zero_tick(&qz, &in, &out);
			if (out.apply_integral) braked_ticks++;
			if (out.low_speed_release && handback_tick < 0) handback_tick = t;
		}
		CHECK(handback_tick > 0,
			"T15d: the handback happens once, during the coast, not at the release edge");
		/*
		 * FW-136 changed what "enough braking" means, so this assertion changed with it.
		 *
		 * Before: the hold ran from 93 erps all the way down to 10, i.e. 89 % of the ramp, and
		 * only the last sliver coasted. That sliver sits exactly where the speed reading is least
		 * trustworthy, so the decision to let go arrived late - measured on the bike, about 30 ms
		 * after the last Hall edge, sometimes before the final commutation steps and sometimes
		 * after. That is the "clicks about half the time" the owner reported.
		 *
		 * Now: the handback is taken at QZERO_HANDBACK_PCT of the speed the release started from,
		 * which is a decision made where Hall edges are dense and nothing arrives late. Braking
		 * therefore covers the ramp down to half speed - and because energy goes as the square of
		 * speed, HALF THE SPEED IS STILL THREE QUARTERS OF THE ENERGY. The bike loses almost
		 * nothing in stopping distance and gains a completely current-free low-speed zone.
		 */
		int expected_handback = (24000 * (100 - QZERO_HANDBACK_PCT)) / 100;
		printf("  T15d handback at tick %d of 24000 (expected ~%d), braked %d\n",
			handback_tick, expected_handback, braked_ticks);
		CHECK(handback_tick > expected_handback - 600 && handback_tick < expected_handback + 600,
			"T15d: the handback lands at QZERO_HANDBACK_PCT of the release speed, not just above "
			"standstill");
		CHECK(braked_ticks > expected_handback + (int)QZERO_HANDBACK_FADE_TICKS - 600 &&
			braked_ticks < expected_handback + (int)QZERO_HANDBACK_FADE_TICKS + 600,
			"T15d: the module acts from the release until the handback fade completes - half the "
			"speed is still 75 % of the energy, so the stop stays short while the whole low-speed "
			"zone goes current-free");
		CHECK(qz.entries == 1U && qz.aborts == 0U && qz.low_speed_exits == 1U,
			"T15d: one release, one entry, one handback, no abort");
	}
}

/* ------------------------------------------------------------------------------------------- */
/* T10: integration with the REAL 16 kHz slew owner                                             */

/* ------------------------------------------------------------------------------------------- */

static void mailbox_integration_checks(void)
{
	enum { FOC_TICKS_PER_MS = 16, RELEASE_MS = 650, START_IQ = 300 };

	/* ---- A real 650 ms release, policy QUIET: entry lands exactly on the first zero tick ---- */
	{
		fast_iq_slew_mailbox_t mb;
		memset(&mb, 0, sizeof(mb));
		fast_iq_slew_reset(&mb);
		int32_t iq_out = 0;

		/* Ride up to a real live accumulator first - the release rate is derived from it. */
		fast_iq_slew_publish(&mb, START_IQ, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE);
		(void)fast_iq_slew_tick(&mb, &iq_out);
		CHECK(iq_out == START_IQ, "T10: setup - the real slew owner is holding a non-zero reference");

		fast_iq_slew_publish(&mb, 0, FIS_MODE_RELEASE, 0U,
			(uint32_t)RELEASE_MS * FOC_TICKS_PER_MS, FIS_ZERO_POLICY_QUIET);

		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		long tick_of_zero = -1, tick_of_entry = -1;
		bool entered_more_than_once = false;
		for (long t = 1; t <= 20L * FOC_TICKS_PER_MS * 1000L; ++t) {
			(void)fast_iq_slew_tick(&mb, &iq_out);
			quiet_zero_input_t in = make_input(iq_out, true, 1200.0f, 100.0f);
			quiet_zero_tick(&qz, &in, &out);
			if (iq_out == 0 && tick_of_zero < 0) tick_of_zero = t;
			if (out.entered) {
				if (tick_of_entry >= 0) entered_more_than_once = true;
				else tick_of_entry = t;
			}
			if (tick_of_zero >= 0 && t > tick_of_zero + 2L * (long)QZERO_BLEND_TICKS) break;
		}
		CHECK(tick_of_zero > 0 && tick_of_entry == tick_of_zero,
			"T10: QZERO enters on exactly the tick the real slew owner first publishes Iq_ref == 0");
		CHECK(!entered_more_than_once,
			"T10: one release produces one entry, not one per tick at zero");
		/*
		 * The release itself is untouched by this card, so it must still take its configured
		 * time - to within the slew owner's OWN pre-existing quantisation, which this check
		 * states rather than hides: fis_live_release_rate() takes ceil(accumulator/ticks), so a
		 * release always lands slightly EARLY (here 10223 of 10400 ticks, 98.3 %) and never
		 * late. That rounding is QS-3D's, predates QZERO and is deliberately not changed here.
		 */
		long expected = (long)RELEASE_MS * FOC_TICKS_PER_MS;
		CHECK(tick_of_zero <= expected && tick_of_zero >= expected - expected / 50,
			"T10: the 650 ms release still takes its configured time (never longer, <2% early from ceil) - QZERO is strictly downstream of it");
		CHECK(out.state == (uint32_t)QZERO_HOLD,
			"T10: 160 ticks after the release completed, the hold is established");
	}

	/* ---- A real FALL to zero (a limiter, a level change), policy NONE: no entry at all ---- */
	{
		fast_iq_slew_mailbox_t mb;
		memset(&mb, 0, sizeof(mb));
		fast_iq_slew_reset(&mb);
		int32_t iq_out = 0;
		fast_iq_slew_publish(&mb, START_IQ, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE);
		(void)fast_iq_slew_tick(&mb, &iq_out);
		/* A brisk fall: full scale over 70 ms, i.e. the fast Ramp Down end of the range. */
		uint16_t step_q8 = (uint16_t)((700 << 8) / (70 * 4));
		fast_iq_slew_publish(&mb, 0, FIS_MODE_FALL, step_q8, 0U, FIS_ZERO_POLICY_NONE);

		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		bool any_entry = false, any_apply = false;
		for (long t = 0; t < 100L * FOC_TICKS_PER_MS; ++t) {
			(void)fast_iq_slew_tick(&mb, &iq_out);
			quiet_zero_input_t in = make_input(iq_out, false, 1200.0f, 100.0f);
			quiet_zero_tick(&qz, &in, &out);
			if (out.entered) any_entry = true;
			if (out.apply_integral) any_apply = true;
		}
		CHECK(iq_out == 0 && !any_entry && !any_apply && qz.entries == 0U,
			"T10: a real fall-to-zero with policy NONE reaches zero and QZERO never touches the PI");
	}

	/* ---- Release, then a re-press before the fade finished: short release + reapply ---- */
	{
		fast_iq_slew_mailbox_t mb;
		memset(&mb, 0, sizeof(mb));
		fast_iq_slew_reset(&mb);
		int32_t iq_out = 0;
		fast_iq_slew_publish(&mb, 60, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE);
		(void)fast_iq_slew_tick(&mb, &iq_out);
		fast_iq_slew_publish(&mb, 0, FIS_MODE_RELEASE, 0U, 32U, FIS_ZERO_POLICY_QUIET);

		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t out;
		for (int t = 0; t < 40; ++t) {
			(void)fast_iq_slew_tick(&mb, &iq_out);
			quiet_zero_input_t in = make_input(iq_out, true, 800.0f, 0.0f);
			quiet_zero_tick(&qz, &in, &out);
		}
		CHECK(out.state == (uint32_t)QZERO_BLEND && qz.entries == 1U,
			"T10: setup - a short release is still mid-fade when the rider pushes again");

		fast_iq_slew_publish(&mb, 60, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE);
		(void)fast_iq_slew_tick(&mb, &iq_out);
		quiet_zero_input_t in = make_input(iq_out, false, 800.0f, 0.0f);
		quiet_zero_tick(&qz, &in, &out);
		CHECK(iq_out > 0 && out.state == (uint32_t)QZERO_INACTIVE && out.exited &&
			!out.apply_integral,
			"T10: a re-press mid-fade exits immediately - the partly faded integral is kept, not reset");
	}
}

/* ------------------------------------------------------------------------------------------- */
/* T11/T12: the effect on the real regulator math                                               */
/* ------------------------------------------------------------------------------------------- */

/* One 16 kHz cycle of the q axis: the replica regulator, then the machine, optionally with the
 * Quiet Zero action applied exactly the way runPIcontrol() applies it (before AND after). */
static void cycle(model_pi_t *pi, model_machine_t *m, int32_t iq_ref, const quiet_zero_action_t *qz)
{
	pi->setpoint = iq_ref;
	pi->recent_value = (int16_t)lrintf(m->iq);
	if (qz && qz->apply_integral) pi->integral_part = qz->iq_integral;
	if (qz && (qz->freeze_aw || qz->clear_aw_edge)) pi->aw_sat_error = 0;
	int32_t u_q = model_pi_control(pi);
	if (qz && qz->apply_integral) pi->integral_part = qz->iq_integral;
	model_machine_step(m, (float)u_q);
}

static void regulator_effect_checks(void)
{
	const float BEMF = 700.0f;   /* a spinning rotor: the voltage the integrator has to match */

	/* ---- T11 A (baseline, QUIET_ZERO_ENABLE=0): the integrator parks at the back-EMF ---- */
	float baseline_iq_free_wheel = 0.0f;
	float baseline_integral = 0.0f;
	{
		model_pi_t pi; model_pi_init(&pi);
		model_machine_t m; model_machine_init(&m, BEMF);
		for (int t = 0; t < 16 * 500; ++t) cycle(&pi, &m, 0, NULL);   /* 500 ms at zero reference */
		baseline_integral = pi.integral_part;
		baseline_iq_free_wheel = m.iq;
		CHECK(fabsf(pi.integral_part - BEMF) < 40.0f,
			"T11-A: with a continuous integrator at Iq_ref=0 the integral settles at the back-EMF");
		CHECK(fabsf(m.iq) < 5.0f,
			"T11-A: matching the back-EMF means ~zero winding current - no braking torque, a free-wheel");
	}

	/* ---- T11 B (QZERO): the fade removes the back-EMF match, so current can flow again ---- */
	{
		model_pi_t pi; model_pi_init(&pi);
		model_machine_t m; model_machine_init(&m, BEMF);
		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t act;

		/* Let the baseline equilibrium establish itself first, exactly as a ride does. */
		for (int t = 0; t < 16 * 200; ++t) {
			quiet_zero_input_t in = make_input(40, true, pi.integral_part, 0.0f);
			quiet_zero_tick(&qz, &in, &act);
			cycle(&pi, &m, 40, &act);
		}
		float integral_at_release = pi.integral_part;
		CHECK(integral_at_release > 100.0f,
			"T11-B: setup - the integrator is genuinely charged when the release starts");

		/* The release edge, then the fade. */
		float iq_min = 0.0f;
		int hold_reached_tick = -1;
		for (int t = 1; t <= 16 * 200; ++t) {
			quiet_zero_input_t in = make_input(0, true, pi.integral_part, 0.0f);
			in.iq_measured = (int32_t)lrintf(m.iq);
			quiet_zero_tick(&qz, &in, &act);
			cycle(&pi, &m, 0, &act);
			if (m.iq < iq_min) iq_min = m.iq;
			if (act.state == (uint32_t)QZERO_HOLD && hold_reached_tick < 0) hold_reached_tick = t;
		}
		CHECK(hold_reached_tick == (int)QZERO_BLEND_TICKS,
			"T11-B: the hold is reached exactly 160 ticks (10 ms) after the release edge");
		CHECK(pi.integral_part == 0.0f,
			"T11-B: during the hold the integral is exactly zero - it cannot rebuild the back-EMF match");
		CHECK(iq_min < -20.0f && fabsf(iq_min) < (float)MODEL_ABORT_CURRENT,
			"T11-B: braking current flows again, and it stays inside the abort bound");
		CHECK(fabsf(iq_min) > fabsf(baseline_iq_free_wheel) * 3.0f,
			"T11-B: the braking current is qualitatively larger than the baseline free-wheel");
		CHECK(pi.setpoint == 0,
			"T11-B: the reference never goes negative - no commanded regen, only the P term acting");
		CHECK(baseline_integral > 100.0f,
			"T11-B: and the A/B difference is real, not an artefact of an uncharged baseline");
	}

	/* ---- T12: restart continuity. PI.out is never reset, and the axis recovers. ---- */
	{
		model_pi_t pi; model_pi_init(&pi);
		model_machine_t m; model_machine_init(&m, BEMF);
		quiet_zero_t qz; quiet_zero_reset(&qz);
		quiet_zero_action_t act;

		for (int t = 0; t < 16 * 100; ++t) {
			quiet_zero_input_t in = make_input(40, true, pi.integral_part, 0.0f);
			quiet_zero_tick(&qz, &in, &act);
			cycle(&pi, &m, 40, &act);
		}
		for (int t = 0; t < 16 * 50; ++t) {   /* release + 50 ms of hold */
			quiet_zero_input_t in = make_input(0, true, pi.integral_part, 0.0f);
			in.iq_measured = (int32_t)lrintf(m.iq);
			quiet_zero_tick(&qz, &in, &act);
			cycle(&pi, &m, 0, &act);
		}
		int32_t out_before_exit = pi.out;

		quiet_zero_input_t in = make_input(40, true, pi.integral_part, 0.0f);
		in.iq_measured = (int32_t)lrintf(m.iq);
		quiet_zero_tick(&qz, &in, &act);
		CHECK(act.exited && !act.apply_integral,
			"T12: setup - the re-press exits QZERO on its first tick");
		cycle(&pi, &m, 40, &act);
		CHECK(labs((long)pi.out - (long)out_before_exit) <= MODEL_MAX_STEP,
			"T12: the exit tick moves PI.out by at most max_step - the slew limiter still owns it, no jump");

		for (int t = 0; t < 16 * 300; ++t) {
			quiet_zero_input_t in2 = make_input(40, true, pi.integral_part, 0.0f);
			in2.iq_measured = (int32_t)lrintf(m.iq);
			quiet_zero_tick(&qz, &in2, &act);
			cycle(&pi, &m, 40, &act);
		}
		CHECK(fabsf(m.iq - 40.0f) < 6.0f,
			"T12: after the exit the axis regulates back onto its reference - the integrator rebuilds");
		CHECK(qz.entries == 1U && qz.aborts == 0U,
			"T12: one release, one entry, no abort across the whole release/re-press cycle");
	}
}

/* ------------------------------------------------------------------------------------------- */
/* T13/T14: production wiring, by source text (main.c/FOC.c/ride_control.c cannot be linked)    */
/* ------------------------------------------------------------------------------------------- */

static char *read_whole_file(const char *path, long *out_len)
{
	FILE *f = fopen(path, "rb");
	if (!f) return NULL;
	if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
	long len = ftell(f);
	if (len < 0 || fseek(f, 0, SEEK_SET) != 0) { fclose(f); return NULL; }
	char *text = (char *)malloc((size_t)len + 1U);
	if (!text) { fclose(f); return NULL; }
	size_t got = fread(text, 1, (size_t)len, f);
	fclose(f);
	text[got] = '\0';
	if (out_len) *out_len = (long)got;
	return text;
}

static char *strip_comments(const char *text, long len)
{
	char *out = (char *)malloc((size_t)len + 1U);
	if (!out) return NULL;
	long i = 0;
	while (i < len) {
		if (text[i] == '/' && i + 1 < len && text[i + 1] == '/') {
			while (i < len && text[i] != '\n') { out[i] = ' '; i++; }
			continue;
		}
		if (text[i] == '/' && i + 1 < len && text[i + 1] == '*') {
			out[i++] = ' '; out[i++] = ' ';
			while (i < len && !(text[i] == '*' && i + 1 < len && text[i + 1] == '/')) {
				out[i] = (text[i] == '\n') ? '\n' : ' ';
				i++;
			}
			if (i < len) out[i++] = ' ';
			if (i < len) out[i++] = ' ';
			continue;
		}
		out[i] = text[i];
		i++;
	}
	out[len] = '\0';
	return out;
}

/* CRLF-tolerant span-open locator: finds `signature` immediately followed by an
 * optional '\r', then '\n', then '{' - so a "void f(void)\n{" anchor locates the
 * function body on both LF and CRLF checkouts (Windows git autocrlf yields \r\n).
 * Returns a pointer to the signature start (identical to the old strstr match), or
 * NULL. Buffers stay byte-identical, so raw/stripped offset arithmetic stays 1:1. */
static const char *find_span_open(const char *haystack, const char *signature)
{
	const char *p = strstr(haystack, signature);
	while (p) {
		const char *q = p + strlen(signature);
		if (*q == '\r') q++;
		if (*q == '\n') q++;
		if (*q == '{') return p;
		p = strstr(p + 1, signature);
	}
	return NULL;
}

static int count_in_span(const char *first, const char *last, const char *needle)
{
	int count = 0;
	size_t needle_len = strlen(needle);
	const char *p = first;
	for (;;) {
		const char *found = strstr(p, needle);
		if (!found || found >= last) break;
		count++;
		p = found + needle_len;
	}
	return count;
}

static int count_all(const char *text, const char *needle)
{
	return count_in_span(text, text + strlen(text), needle);
}

static void production_wiring_checks(void)
{
	long main_len = 0, foc_len = 0, ride_len = 0, loop_len = 0;
	char *main_raw = read_whole_file(STRINGIZE(MAIN_C_PATH), &main_len);
	char *foc_raw = read_whole_file(STRINGIZE(FOC_C_PATH), &foc_len);
	char *ride_raw = read_whole_file(STRINGIZE(RIDE_CONTROL_C_PATH), &ride_len);
	long pipe_len = 0;
	char *pipe_raw = read_whole_file(STRINGIZE(ASSIST_PIPELINE_C_PATH), &pipe_len);
	char *loop_raw = read_whole_file(STRINGIZE(FOC_CURRENT_LOOP_C_PATH), &loop_len);
	CHECK(main_raw && foc_raw && ride_raw && loop_raw,
		"setup: main.c, FOC.c, ride_control.c and foc_current_loop.c are readable");
	if (!main_raw || !foc_raw || !ride_raw || !loop_raw) goto done;

	char *main_c = strip_comments(main_raw, main_len);
	char *foc_c = strip_comments(foc_raw, foc_len);
	char *ride_c = strip_comments(ride_raw, ride_len);
	/* The zero POLICY is decided where the final trajectory is decided, which is now the one
	 * assist chain rather than ride_control. ride_control still owns the single publication,
	 * so both files are read and each is checked for the half it owns. */
	char *pipe_c = pipe_raw ? strip_comments(pipe_raw, pipe_len) : NULL;
	char *loop_c = strip_comments(loop_raw, loop_len);
	CHECK(main_c && foc_c && ride_c && loop_c, "setup: all QZERO/current-loop sources sanitize successfully");
	if (!main_c || !foc_c || !ride_c || !loop_c) { free(main_c); free(foc_c); free(ride_c); free(loop_c); goto done; }

	/* Model-matches-production: the replica must still be FOC.c's real PI_control() body. */
	CHECK(strstr(foc_c, "PI_c->integral_part += (Delta - aw_part)*PI_c->gain_i;") != NULL &&
		strstr(foc_c, "float aw_part = (float)((PI_c->aw_sat_error * PI_c->aw_inv_kp_q15) >> 15);") != NULL &&
		strstr(foc_c, "if (PI_c->integral_part > PI_c->limit_i) PI_c->integral_part = PI_c->limit_i;") != NULL &&
		strstr(foc_c, "else PI_c->out=(p_part+PI_c->integral_part);") != NULL,
		"setup: model_pi_control() still matches FOC.c's real PI_control() body");

	/* ---- T13: the consumer sits inside runPIcontrol(), around the regulators, and nowhere else ---- */
	{
		const char *run_pi = strstr(main_c, "void runPIcontrol(void){");
		const char *run_pi_end = strstr(main_c, "void autodetect(void) {");
		CHECK(run_pi && run_pi_end && run_pi < run_pi_end,
			"setup: runPIcontrol() span is locatable");
		if (run_pi && run_pi_end && run_pi < run_pi_end) {
			CHECK(count_in_span(run_pi, run_pi_end, "quiet_zero_tick(&quiet_zero_state") == 1,
				"T13: exactly one quiet_zero_tick() call, and it is inside runPIcontrol()");
			const char *tick_call = strstr(run_pi, "quiet_zero_tick(&quiet_zero_state");
			const char *loop_call = strstr(run_pi, "foc_current_loop_step(&MS, &PI_iq, &PI_id");
			CHECK(tick_call && loop_call && tick_call < loop_call,
				"T13: the decision is made BEFORE either regulator runs, on this tick's Iq_ref");
			CHECK(count_in_span(run_pi, run_pi_end, "PI_iq.integral_part = qz.iq_integral;") == 1 &&
				count_in_span(run_pi, run_pi_end, "PI_id.integral_part = qz.id_integral;") == 1 &&
				strstr(loop_c, "if (reassert_integral)") != NULL &&
				strstr(loop_c, "pi_iq->integral_part = iq_integral;") != NULL &&
				strstr(loop_c, "pi_id->integral_part = id_integral;") != NULL,
				"T13: the commanded integral is applied before the shared loop and re-asserted inside it after both PI calls");
			CHECK(count_in_span(run_pi, run_pi_end, "PI_iq.aw_sat_error = 0;") == 1 &&
				count_in_span(run_pi, run_pi_end, "PI_id.aw_sat_error = 0;") == 1,
				"T13: FOC-AW1's residual is cleared in the ISR that owns it, not via foc_aw_tracking_reset()");
			CHECK(count_in_span(run_pi, run_pi_end, "foc_aw_tracking_reset()") == 0,
				"T13: runPIcontrol() does NOT call the foreground bridge-off AW hook");
			CHECK(count_in_span(run_pi, run_pi_end, "quiet_zero_reset(") == 0,
				"T13: the periodic path never resets the state machine");
			CHECK(count_in_span(run_pi, run_pi_end, "QUIET_ZERO_ENABLE") == 2,
				"T13: the whole consumer is behind the A/B switch - two #if blocks, nothing else");
			/* The card's explicit prohibitions, checked where they would have to appear. */
			CHECK(count_in_span(run_pi, run_pi_end, "timer_primary_output_config") == 0 &&
				count_in_span(run_pi, run_pi_end, "PI_iq.out = 0") == 0 &&
				count_in_span(run_pi, run_pi_end, "PI_iq.out=0") == 0 &&
				count_in_span(run_pi, run_pi_end, "switchtime[0]") == 0,
				"T13: no MOE toggle, no PI.out reset and no direct neutral CCR write in the new path");
		}

		/* STOP-CLICK-C1 must stay fixed: still zero foreground integral writers at ordinary zero. */
		const char *reg_adc_start = find_span_open(main_c, "void reg_ADC_processing(void)");
		const char *reg_adc_end = strstr(main_c, "int16_t internal_tics_to_speedx100 (uint32_t tics){");
		CHECK(reg_adc_start && reg_adc_end && reg_adc_start < reg_adc_end,
			"setup: reg_ADC_processing() span is locatable");
		if (reg_adc_start && reg_adc_end && reg_adc_start < reg_adc_end) {
			CHECK(count_in_span(reg_adc_start, reg_adc_end, "PI_iq.integral_part=") == 0 &&
				count_in_span(reg_adc_start, reg_adc_end, "PI_id.integral_part=") == 0 &&
				count_in_span(reg_adc_start, reg_adc_end, "PI_iq.integral_part =") == 0 &&
				count_in_span(reg_adc_start, reg_adc_end, "PI_id.integral_part =") == 0,
				"T13: STOP-CLICK-C1 stays fixed - the 4 kHz foreground still writes neither integral");
			CHECK(count_in_span(reg_adc_start, reg_adc_end, "quiet_zero_") == 0,
				"T13: and the foreground does not reach into the new state machine either");
		}

		/* quiet_zero_reset() only where the regulators are already reset with FOC inactive. */
		CHECK(count_all(main_c, "quiet_zero_reset(&quiet_zero_state)") == 3,
			"T13: exactly three reset sites - cold PREPARE, dwell-timeout failsafe, hall calibration");
		{
			int paired = 0;
			const char *p = main_c;
			for (;;) {
				const char *hit = strstr(p, "quiet_zero_reset(&quiet_zero_state)");
				if (!hit) break;
				/* Each site must sit next to the FOC-AW1 reset, which is the existing, tested
				 * marker for "regulators zeroed AND FOC inactive or going inactive". */
				const char *window_start = (hit - main_c > 600) ? hit - 600 : main_c;
				if (count_in_span(window_start, hit + 200, "foc_aw_tracking_reset()") > 0) paired++;
				p = hit + 1;
			}
			CHECK(paired == 3,
				"T13: every reset site is paired with foc_aw_tracking_reset() - the same allowed reset domain");
		}

		/* The one instance, and the include. */
		CHECK(count_all(main_c, "static quiet_zero_t quiet_zero_state;") == 1 &&
			count_all(main_c, "#include \"quiet_zero.h\"") == 1,
			"T13: one state object, one include - no second copy of the state anywhere in main.c");
	}

	/* ---- T14: the producer grants QUIET in exactly one place, and defaults to NONE ---- */
	{
		CHECK(pipe_c && count_all(pipe_c, "FIS_ZERO_POLICY_QUIET") == 1 &&
			count_all(ride_c, "FIS_ZERO_POLICY_QUIET") == 0,
			"T14: exactly one place in the whole assist chain can grant Quiet Zero, and it is in the trajectory decision - not in the layer that merely publishes it");
		const char *grant = pipe_c ? strstr(pipe_c, "FIS_ZERO_POLICY_QUIET") : NULL;
		const char *release_branch = pipe_c ?
			strstr(pipe_c, "if (iq_target == 0 && (block_positive || !permitted)) {") : NULL;
		const char *release_return = pipe_c ?
			strstr(pipe_c, "return block_positive ? FIS_MODE_SAFETY : FIS_MODE_RELEASE;") : NULL;
		CHECK(grant && release_branch && release_return &&
			release_branch < grant && grant < release_return,
			"T14: and it is inside the release branch, above its own return");
		CHECK(pipe_c && strstr(pipe_c, "*out_zero_policy = service_cut ? FIS_ZERO_POLICY_NONE : FIS_ZERO_POLICY_QUIET;") != NULL,
			"T14: the pedal-load calibration is excluded at the grant itself");
		CHECK(pipe_c && strstr(pipe_c, "*out_zero_policy = FIS_ZERO_POLICY_NONE;") != NULL,
			"T14: the policy is default-deny - every other path leaves the ordinary PI in charge");
		CHECK(count_all(ride_c, "fast_iq_slew_publish(") == 1,
			"T14: still exactly one publish call site - the policy did not create a second owner");
		CHECK(strstr(main_c, ".service_cut_active = torque_input_calibration_active(),") != NULL,
			"T14: main.c supplies the service subset from the calibration flag itself");
		/* One threshold, two consumers: the ISR and FW-048 must read the same constant. */
		CHECK(strstr(main_c, ".min_brake_erps = RIDE_COAST_RELEASE_ERPS,") != NULL &&
			strstr(main_c, ".rotor_erps = (int32_t)rotor_motion.edge_erps,") != NULL &&
			strstr(main_c, ".speed_fresh = rotor_motion_speed_fresh(&rotor_motion, ui16_erps_counter),") != NULL,
			"T14: QZERO uses a fresh real Hall interval, never the age-decayed liveness speed");
		CHECK(pipe_c && count_all(pipe_c, "#define RIDE_COAST_RELEASE_ERPS") == 0 &&
			strstr(pipe_c, "#define AP2_COAST_RELEASE_ERPS RIDE_COAST_RELEASE_ERPS") != NULL &&
			strstr(pipe_c, "in->motor_erps < AP2_COAST_RELEASE_ERPS") != NULL,
			"T14: the coast threshold is the shared constant, named once and not re-declared");
		/* Nothing this card touches may have changed the release timing constants. */
		CHECK(pipe_c && strstr(pipe_c, "#define AP2_SAFETY_RELEASE_MS 200U") != NULL,
			"T14: the 200 ms firmware-owned safety release is untouched");
		CHECK(pipe_c && strstr(pipe_c, "release_ms = prof.p.release_ms;") != NULL,
			"T14: the normal release still comes from the profile's own release time");
	}

	free(main_c);
	free(foc_c);
	free(pipe_c);
	free(pipe_raw);
	free(ride_c);
	free(loop_c);
done:
	free(main_raw);
	free(foc_raw);
	free(ride_raw);
	free(loop_raw);
}

int main(void)
{
	puts("QZERO: Quiet Zero - PI integral fade after the final Iq reference reaches zero");
	state_machine_checks();
	low_speed_handback_checks();
	mailbox_integration_checks();
	regulator_effect_checks();
	production_wiring_checks();
	if (host_test_failures == 0) {
		puts("QZERO Quiet Zero: ALL CHECKS PASSED");
		return 0;
	}
	printf("QZERO Quiet Zero: %d CHECK(S) FAILED\n", host_test_failures);
	return 1;
}
