/*
 * QZERO: Quiet Zero. Contract, rationale and ownership rules are in inc/quiet_zero.h.
 *
 * Implementation notes:
 *
 *   - No global state. The caller owns the quiet_zero_t instance, so the whole state machine
 *     runs on a host against the same code the ISR executes.
 *   - No division: the linear fade multiplies by a compile-time reciprocal.
 *   - The fade is computed from the CAPTURED entry value every tick (entry * remaining/N), not
 *     by repeated subtraction, so the trajectory is exactly linear and its last point is
 *     exactly 0.0f rather than an accumulated rounding residue.
 *   - The abort guard is checked before the fade is applied, so the tick that detects too much
 *     current is already a full-PI tick, not a P-only one. The low-speed handback is checked
 *     first of the two: it is the expected, benign end of a release, not a fault, and at those
 *     speeds the back-EMF is too small to have raised the current guard anyway.
 */

#include "quiet_zero.h"

#include <stddef.h>

static inline int32_t qz_abs(int32_t v)
{
	return (v < 0) ? -v : v;
}

/*
 * The integral that nulls the q-axis current at the present speed: back-EMF is proportional to
 * speed, so the integral captured at entry, scaled by how much the rotor has slowed since, IS
 * that value to a first order. This is the one law shared by the low-speed HANDBACK fade (as its
 * per-tick target) and by the bumpless exit on a re-engage (applied at once). The speed is
 * clamped to the entry speed so a bad reading can never amplify; 0.0f when there is no usable
 * entry speed or present speed.
 */
static float qz_matched_iq_integral(const quiet_zero_t *qz, int32_t rotor_erps)
{
	if (qz->erps_entry > 0 && rotor_erps > 0) {
		int32_t erps_now = rotor_erps;
		if (erps_now > qz->erps_entry) {
			erps_now = qz->erps_entry;
		}
		return qz->iq_integral_entry * ((float)erps_now / (float)qz->erps_entry);
	}
	return 0.0f;
}

void quiet_zero_reset(quiet_zero_t *qz)
{
	if (qz == NULL) {
		return;
	}
	qz->state = (uint32_t)QZERO_INACTIVE;
	qz->blend_tick = 0U;
	qz->iq_integral_entry = 0.0f;
	qz->id_integral_entry = 0.0f;
	qz->erps_entry = 0;
	qz->prev_iq_ref = 0;
	qz->handback_iq = qz->handback_id = 0.0f;
	qz->handback_step_q = qz->handback_step_d = 0.0f;
	/*
	 * QZERO-2: seeded to 1, not 0, and that is deliberate. This reset runs where the bridge is
	 * being switched off in the same breath (cold PREPARE, dwell failsafe, hall calibration). If
	 * the producer's verdict happens to still be QUIET on the next ISR tick, a zero-seed would
	 * manufacture a rising edge out of the reset itself and arm a fade nobody asked for. Seeding
	 * it high means only a verdict that goes false and true again can arm anything.
	 */
	qz->prev_policy_quiet = 1U;
	qz->entries = 0U;
	qz->aborts = 0U;
	qz->low_speed_exits = 0U;
	qz->hold_ticks = 0U;
}

void quiet_zero_tick(
	quiet_zero_t *qz,
	const quiet_zero_input_t *in,
	quiet_zero_action_t *out)
{
	if (qz == NULL || in == NULL || out == NULL) {
		return;
	}

	out->apply_integral = false;
	out->iq_integral = 0.0f;
	out->id_integral = 0.0f;
	out->freeze_aw = false;
	out->clear_aw_edge = false;
	out->entered = false;
	out->exited = false;
	out->aborted = false;
	out->low_speed_release = false;

	/*
	 * QZERO-2: the rising edge of the QUIET verdict, captured BEFORE any return path so every
	 * exit from this function leaves the same one-tick history behind. See prev_policy_quiet in
	 * the header for why a second entry edge is needed at all.
	 */
	bool policy_edge = in->zero_policy_quiet && (qz->prev_policy_quiet == 0U);
	qz->prev_policy_quiet = in->zero_policy_quiet ? 1U : 0U;

	/*
	 * EXIT. A positive reference means the rider (or anything else entitled to ask) wants
	 * torque again. Leave on the same tick; FOC-AW1 resumes, with the residual cleared once so
	 * the first active cycle does not correct against the held vector.
	 *
	 * I3 (TASK-EVD-TQ-06-G2, bumpless exit). The regulators used to get their integral back
	 * "untouched", which from HOLD means exactly 0: the drive then started from the BRAKING
	 * state (Iq negative for several ms, then a catch-up faster than the reference ramp - the
	 * chain jerk when the rider resumes pedalling while the bike rolls). A stock live PI keeps
	 * the matching integral and re-engages smoothly. So when the rotor is still turning and the
	 * speed is fresh, hand back the integral that nulls the current at the present speed, by the
	 * same law HANDBACK targets, applied at once on this tick: Iq = matched value, Id = 0 (as in
	 * HANDBACK, with the q current at zero there is no cross-coupling left for u_d to cancel).
	 * Applies from BLEND, HOLD and HANDBACK alike. freeze_aw stays false: the PI owns the axis.
	 *
	 * Fallback = the previous behaviour (nothing applied, integral left as it is) when there is
	 * no usable matching value: speed not fresh (an old reading is not a BEMF measurement),
	 * rotor_erps <= 0, or erps_entry == 0 (nothing to scale from).
	 */
	if (in->iq_ref > 0) {
		if (qz->state != (uint32_t)QZERO_INACTIVE) {
			if (in->speed_fresh && in->rotor_erps > 0 && qz->erps_entry > 0) {
				out->apply_integral = true;
				out->iq_integral = qz_matched_iq_integral(qz, in->rotor_erps);
				out->id_integral = 0.0f;
			}
			qz->state = (uint32_t)QZERO_INACTIVE;
			qz->blend_tick = 0U;
			out->exited = true;
			out->clear_aw_edge = true;
		}
		qz->prev_iq_ref = in->iq_ref;
		out->state = qz->state;
		return;
	}

	/*
	 * ENTRY. The reference is zero (a negative reference cannot be produced by the final slew
	 * owner, but treat it as zero rather than trusting that). Two edges arm the fade, and both
	 * describe the same physical event - "this zero is a rider release":
	 *
	 *   - the non-zero -> exact zero edge of the reference (v1), and
	 *   - QZERO-2: the rising edge of the QUIET verdict while the reference is already zero,
	 *     which is what happens when a limiter, a closing torque gate or simply easing off
	 *     reached zero before the cranks stopped.
	 *
	 * Every other zero is still left to the ordinary zero-current PI. The verdict is only ever
	 * granted after pedalling has ended, so this remains "never a limiter" as designed.
	 */
	if (qz->state == (uint32_t)QZERO_INACTIVE) {
		if ((qz->prev_iq_ref > 0 || policy_edge) && in->zero_policy_quiet &&
			in->speed_fresh && in->rotor_erps >= in->min_brake_erps) {
			qz->state = (uint32_t)QZERO_BLEND;
			qz->blend_tick = 0U;
			qz->iq_integral_entry = in->iq_integral;
			qz->id_integral_entry = in->id_integral;
			qz->erps_entry = in->rotor_erps;   /* QZERO-3: the speed this integral matched */
			qz->entries++;
			out->entered = true;
			out->clear_aw_edge = true;
		} else {
			qz->prev_iq_ref = 0;
			out->state = qz->state;
			return;
		}
	}

	qz->prev_iq_ref = 0;

	/* Unknown speed is not a new BEMF measurement. Return the live integral to
	 * ordinary zero-current PI; do not seed, reset or extrapolate it here. */
	if (!in->speed_fresh) {
		qz->state = (uint32_t)QZERO_INACTIVE;
		qz->blend_tick = 0U;
		qz->low_speed_exits++;
		out->low_speed_release = true;
		out->exited = true;
		out->clear_aw_edge = true;
		out->state = qz->state;
		return;
	}

	/*
	 * LOW-SPEED HANDBACK. The rotor has reached the zone where the commutation angle advances in
	 * 60-degree steps (FW-048 / FW-131). Current must not be flowing when it steps.
	 *
	 * QZERO-3. v1 handed the axis back with the integral left at exactly zero and trusted the PI
	 * to "rebuild the back-EMF match again". It does - but not instantly, and until it has, the
	 * applied voltage does not match the back-EMF, so BRAKING CURRENT KEEPS FLOWING all the way
	 * down through the stepping zone. That is the same state the old foreground reset produced,
	 * which FW-048 identified as the click's precondition in the first place.
	 *
	 * The rider's own A/B settled it: a coast with no winding current at all was silent through a
	 * full stop AND a full restart, while the same passage under braking current clicks. So the
	 * last stretch has to be genuinely current-free, not merely heading that way.
	 *
	 * The value that nulls the current is the one matching the back-EMF, and back-EMF is
	 * proportional to speed - so the integral captured at entry, scaled by how much the rotor has
	 * slowed since, IS that value to a first order. Handing THAT back leaves the axis in the
	 * free-coast state directly, with no transient to hear. Energy scales with the square of
	 * speed, so surrendering the last few rev/s of braking costs well under 1 % of the run-on.
	 */
	/*
	 * FW-136: hand the axis back at a FRACTION OF THE RELEASE SPEED, not just above standstill.
	 *
	 * min_brake_erps alone means the whole run-down is braked and only the last sliver coasts -
	 * and that sliver sits in the speed zone where the reading is least trustworthy and where
	 * the decision therefore arrives late. Measured on the bike: the cut fires about 30 ms after
	 * the last Hall edge, sometimes before the last commutation steps and sometimes after, which
	 * is exactly the "clicks about half the time" the owner reported.
	 *
	 * Taking the threshold as a fraction of the speed the release started from moves the decision
	 * to where Hall edges are dense and the estimate is fresh, so there is nothing left to arrive
	 * late for. Energy goes as the square of speed, so handing back at half the speed still lets
	 * the brake take 75 % of it - the run-down stays short, and the whole low-speed zone becomes
	 * current-free, which is the condition already known to be silent.
	 *
	 * min_brake_erps stays as the floor. A release that began below it never gets a relative
	 * threshold under it, and FW-048 keeps its own meaning untouched.
	 */
	int32_t handback_erps = in->min_brake_erps;
	if (qz->erps_entry > 0) {
		int32_t relative = ((int32_t)qz->erps_entry * QZERO_HANDBACK_PCT) / 100;
		if (relative > handback_erps) {
			handback_erps = relative;
		}
	}
	/*
	 * FW-136.2: START the handback, once. The fade itself runs below, after the abort guard, so
	 * a genuine overcurrent can still cut it short.
	 */
	if (qz->state != (uint32_t)QZERO_HANDBACK && in->rotor_erps < handback_erps) {
		qz->handback_iq = qz->state == (uint32_t)QZERO_HOLD ? 0.0f : in->iq_integral;
		qz->handback_id = qz->state == (uint32_t)QZERO_HOLD ? 0.0f : in->id_integral;
		float q = qz->iq_integral_entry < 0 ? -qz->iq_integral_entry : qz->iq_integral_entry;
		float d = qz->id_integral_entry < 0 ? -qz->id_integral_entry : qz->id_integral_entry;
		float start_q = in->iq_integral < 0 ? -in->iq_integral : in->iq_integral;
		float start_d = in->id_integral < 0 ? -in->id_integral : in->id_integral;
		qz->handback_step_q = (q > start_q ? q : start_q) * QZERO_HANDBACK_RECIP;
		qz->handback_step_d = (d > start_d ? d : start_d) * QZERO_HANDBACK_RECIP;
		qz->state = (uint32_t)QZERO_HANDBACK;
		qz->blend_tick = 0U;
		qz->low_speed_exits++;
		out->low_speed_release = true;
		out->clear_aw_edge = true;
	}

	/*
	 * ABORT GUARD.
 During the P-only hold the only thing limiting winding current is the
	 * proportional term, so measured current is the quantity that has to be watched. Too much
	 * of it in either axis ends the hold and hands the axis back to the full zero-current PI -
	 * the integrator is then free to build the BEMF-matching voltage again, which is precisely
	 * the state that produces no current. It deliberately does not re-arm: a release that had
	 * to be aborted must not oscillate between the two behaviours.
	 */
	if (qz_abs(in->iq_measured) >= in->abort_current ||
		qz_abs(in->id_measured) >= in->abort_current) {
		qz->state = (uint32_t)QZERO_INACTIVE;
		qz->blend_tick = 0U;
		qz->aborts++;
		out->aborted = true;
		out->clear_aw_edge = true;
		out->state = qz->state;
		return;
	}

	/*
	 * FW-136.2 HANDBACK FADE. The value that nulls the current is the one matching the back-EMF,
	 * and back-EMF is proportional to speed - so the integral captured at entry, scaled by how
	 * much the rotor has slowed since, IS that value to a first order. The target is recomputed
	 * every tick rather than latched at the start, because the rotor keeps slowing throughout the
	 * fade and a latched target would be handing back a voltage for a speed the motor no longer
	 * has.
	 *
	 * What ramps is the BLEND, not the target: 0 -> 1 over QZERO_HANDBACK_FADE_TICKS. The braking
	 * current therefore decays smoothly to zero instead of being switched off, which is the whole
	 * point - a torque that disappears in one tick is a torque step, and a torque step is a click.
	 *
	 * The d axis is NOT scaled: with the q current already at zero there is no cross-coupling
	 * term left for u_d to cancel, so zero is the value that nulls it.
	 */
	if (qz->state == (uint32_t)QZERO_HANDBACK) {
		float target = qz_matched_iq_integral(qz, in->rotor_erps);
		qz->blend_tick++;
		out->apply_integral = true;
		/* Rate-limit the integral itself, including a changing speed target. Starting
		 * at zero here caused a step whenever handback interrupted the entry fade. */
		float dq = target - qz->handback_iq;
		float dd = -qz->handback_id;
		if (dq > qz->handback_step_q) dq = qz->handback_step_q;
		if (dq < -qz->handback_step_q) dq = -qz->handback_step_q;
		if (dd > qz->handback_step_d) dd = qz->handback_step_d;
		if (dd < -qz->handback_step_d) dd = -qz->handback_step_d;
		qz->handback_iq += dq;
		qz->handback_id += dd;
		out->iq_integral = qz->handback_iq;
		out->id_integral = qz->handback_id;
		if (qz->blend_tick >= QZERO_HANDBACK_FADE_TICKS &&
			qz->handback_iq == target && qz->handback_id == 0.0f) {
			/* Fully handed back. freeze_aw is deliberately NOT set: the PI owns the axis now. */
			qz->state = (uint32_t)QZERO_INACTIVE;
			qz->blend_tick = 0U;
			out->iq_integral = target;
			out->state = qz->state;
			return;
		}
		out->freeze_aw = true;
		out->state = qz->state;
		return;
	}

	if (qz->state == (uint32_t)QZERO_BLEND) {
		qz->blend_tick++;
		if (qz->blend_tick >= QZERO_BLEND_TICKS) {
			/* Last point of the fade is exact zero, and it is also the first HOLD tick. */
			qz->state = (uint32_t)QZERO_HOLD;
			qz->blend_tick = 0U;
			qz->hold_ticks++;
			out->iq_integral = 0.0f;
			out->id_integral = 0.0f;
		} else {
			float remaining = (float)(QZERO_BLEND_TICKS - qz->blend_tick) * QZERO_BLEND_RECIP;
			out->iq_integral = qz->iq_integral_entry * remaining;
			out->id_integral = qz->id_integral_entry * remaining;
		}
	} else {
		qz->hold_ticks++;
		out->iq_integral = 0.0f;
		out->id_integral = 0.0f;
	}

	out->apply_integral = true;
	out->freeze_aw = true;
	out->state = qz->state;
}
