/*
 * FOC-AW1: tracking (back-calculation) anti-windup for the D/Q current regulators.
 *
 * runPIcontrol() runs PI_iq and PI_id independently and then applies a SHARED circle limiter to
 * their combined vector. Each PI's own limit_i/limit_output clamps are blind to that limiter, so
 * an axis whose voltage was cut to give budget to the other axis kept charging its integrator
 * against a demand the bridge was not delivering. This card feeds the residual the limiter threw
 * away back into the integrator: integral_error = current_error - Kaw*sat_error, with
 * Kaw = 1/gain_p derived in inc/main.h and carried as Q15 in PI_control_t.aw_inv_kp_q15.
 *
 * main.c and FOC.c are the ARM entry point / ISR core and cannot be linked on a host, so this
 * harness uses the same two complementary techniques as stopclick_c1_pi_integral_host.c, whose
 * replica it deliberately extends rather than duplicates:
 *
 *   - a byte-faithful REPLICA of the real regulator and the real limiter (model_pi_control() +
 *     model_run_pi_control() below), used to exercise the actual arithmetic - parity, residual
 *     sign and magnitude, the N -> N+1 delay, saturation exit, clamps, overflow bounds;
 *   - source-text guards against main.c/FOC.c (production_wiring_checks()) proving the
 *     production wiring matches the replica and that the reset sites are exactly the three the
 *     card allows - and, critically, that ordinary zero torque / ARMED_ZERO is NOT one of them.
 *
 * Source guards pin the replica to the shipped bodies, so a future change to the real algorithm
 * that the replica does not track breaks this suite loudly instead of going quietly stale.
 */

#include "../common/check.h"

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
#ifndef MAIN_H_PATH
#error "MAIN_H_PATH must be supplied by run-host-tests.ps1"
#endif
#ifndef FOC_CURRENT_LOOP_C_PATH
#error "FOC_CURRENT_LOOP_C_PATH must be supplied by run-host-tests.ps1"
#endif

/* ------------------------------------------------------------------------------------------- */
/* Replica of the production regulator + limiter.                                                */
/*                                                                                               */
/* Constants are hardcoded rather than included, for the same reason                             */
/* stopclick_c1_pi_integral_host.c hardcodes them: inc/config.h and inc/FOC.h pull in             */
/* hardware-dependent context this host harness does not have. production_wiring_checks() below   */
/* proves each of them still matches the shipped source.                                          */
/* ------------------------------------------------------------------------------------------- */

#define MODEL_GAIN_P            1.5f      /* P_FACTOR_I_Q == P_FACTOR_I_D */
#define MODEL_GAIN_I            0.01f     /* I_FACTOR_I_Q == I_FACTOR_I_D */
#define MODEL_U_MAX             1920      /* _U_MAX (inc/FOC.h, non-DISABLE_DYNAMIC_ADC) */
#define MODEL_LIMIT_I_Q         1920      /* PI_iq.limit_i = _U_MAX */
#define MODEL_LIMIT_I_D         1800      /* PI_id.limit_i = 1800 */
#define MODEL_LIMIT_OUT         1920      /* both: limit_output = _U_MAX */
#define MODEL_MAX_STEP          15
#define MODEL_AW_INV_KP_Q15_MAX 262144L   /* FOC_AW_INV_KP_Q15_MAX */
#define MODEL_AW_SAT_ERROR_MAX  4096L     /* FOC_AW_SAT_ERROR_MAX */

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
	int32_t aw_sat_error;    /* FOC-AW1 */
	int32_t aw_inv_kp_q15;   /* FOC-AW1 */
} model_pi_t;

/* Replica of main.c's pi_aw_init(). */
static void model_pi_aw_init(model_pi_t *PI_c)
{
	PI_c->aw_sat_error = 0;
	PI_c->aw_inv_kp_q15 = 0;
	if (PI_c->gain_p > 0.0f) {
		float q15 = 32768.0f / PI_c->gain_p + 0.5f;
		if (q15 > (float)MODEL_AW_INV_KP_Q15_MAX) q15 = (float)MODEL_AW_INV_KP_Q15_MAX;
		PI_c->aw_inv_kp_q15 = (int32_t)q15;
	}
}

static void model_pi_init(model_pi_t *pi, int16_t limit_i, bool aw_enabled)
{
	memset(pi, 0, sizeof(*pi));
	pi->gain_p = MODEL_GAIN_P;
	pi->gain_i = MODEL_GAIN_I;
	pi->limit_i = limit_i;
	pi->limit_output = MODEL_LIMIT_OUT;
	pi->max_step = MODEL_MAX_STEP;
	if (aw_enabled) model_pi_aw_init(pi);
	/* aw_enabled == false leaves aw_inv_kp_q15 at 0, which IS the pre-FOC-AW1 regulator:
	 * the correction term becomes an unconditional exact zero. That is what makes the parity
	 * test below a real comparison against the old behaviour and not against itself. */
}

/* Byte-faithful replica of FOC.c's PI_control(), FOC-AW1 term included. */
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

/* Replica of runPIcontrol()'s vector limiter + FOC-AW1 residual publication. */
typedef struct {
	model_pi_t pi_q;
	model_pi_t pi_d;
	int32_t u_q, u_d, u_abs;                 /* applied */
	int32_t u_q_req, u_d_req, u_abs_req;     /* requested */
	int32_t u_q_sat_err, u_d_sat_err;        /* physical Vd/Vq residual */
	uint8_t saturated;
	uint32_t sat_ticks;
} model_foc_t;

static void model_foc_init(model_foc_t *m, bool aw_enabled)
{
	memset(m, 0, sizeof(*m));
	model_pi_init(&m->pi_q, MODEL_LIMIT_I_Q, aw_enabled);
	model_pi_init(&m->pi_d, MODEL_LIMIT_I_D, aw_enabled);
}

/* Replica of main.c's foc_aw_publish_residual(). */
static void model_publish_residual(model_foc_t *m)
{
	int32_t sat_q = m->u_q_req - m->u_q;
	int32_t sat_d = m->u_d_req - m->u_d;

	if (sat_q >  MODEL_AW_SAT_ERROR_MAX) sat_q =  MODEL_AW_SAT_ERROR_MAX;
	if (sat_q < -MODEL_AW_SAT_ERROR_MAX) sat_q = -MODEL_AW_SAT_ERROR_MAX;
	if (sat_d >  MODEL_AW_SAT_ERROR_MAX) sat_d =  MODEL_AW_SAT_ERROR_MAX;
	if (sat_d < -MODEL_AW_SAT_ERROR_MAX) sat_d = -MODEL_AW_SAT_ERROR_MAX;

	m->u_q_sat_err = sat_q;
	m->u_d_sat_err = sat_d;

	m->pi_q.aw_sat_error =  sat_q;
	m->pi_d.aw_sat_error = -sat_d;   /* Vd = -PI_id.out: the sign flip lives HERE, once */
}

/* Replica of runPIcontrol(). iq_meas/id_meas are this cycle's measured currents. */
static void model_run_pi_control(model_foc_t *m, int32_t iq_ref, int16_t iq_meas,
                                 int32_t id_ref, int16_t id_meas)
{
	int32_t u_q_temp, u_d_temp;

	m->pi_q.recent_value = iq_meas;
	m->pi_q.setpoint = iq_ref;
	u_q_temp = model_pi_control(&m->pi_q);

	m->pi_d.recent_value = id_meas;
	m->pi_d.setpoint = id_ref;
	u_d_temp = -model_pi_control(&m->pi_d);

	m->u_q_req = u_q_temp;
	m->u_d_req = u_d_temp;

	m->u_abs = (int32_t)sqrtf((float)(u_d_temp * u_d_temp + u_q_temp * u_q_temp));
	m->u_abs_req = m->u_abs;

	if (m->u_abs > MODEL_U_MAX) {
		m->u_q = (u_q_temp * MODEL_U_MAX) / m->u_abs;
		m->u_d = (u_d_temp * MODEL_U_MAX) / m->u_abs;
		m->u_abs = MODEL_U_MAX;
		m->saturated = 1;
		m->sat_ticks++;
	} else {
		m->u_q = u_q_temp;
		m->u_d = u_d_temp;
		m->saturated = 0;
	}

	model_publish_residual(m);
}

/* ------------------------------------------------------------------------------------------- */
/* Behaviour                                                                                     */
/* ------------------------------------------------------------------------------------------- */

/* Deterministic LCG - no rand(), so the parity run is identical on every machine and every
 * rerun, which is the only way "bit-identical" means anything as a regression claim. */
static uint32_t lcg_state = 0x12345678u;
static int32_t lcg_next(int32_t lo, int32_t hi)
{
	lcg_state = lcg_state * 1103515245u + 12345u;
	return lo + (int32_t)((lcg_state >> 16) % (uint32_t)(hi - lo + 1));
}

/* T-KAW: the derived tracking gain and its guards. */
static void kaw_derivation_checks(void)
{
	model_pi_t pi;

	model_pi_init(&pi, MODEL_LIMIT_I_Q, true);
	CHECK(pi.aw_inv_kp_q15 == 21845,
		"T-KAW1: gain_p=1.5 derives aw_inv_kp_q15=21845 (Kaw=0.66665649 vs ideal 0.66666667)");
	CHECK(fabs((double)pi.aw_inv_kp_q15 / 32768.0 - 1.0 / (double)MODEL_GAIN_P) < 1e-4,
		"T-KAW2: the Q15 gain is 1/gain_p to better than 1e-4");

	/* Kaw is DERIVED from gain_p, never a free constant: change gain_p and it must follow. */
	pi.gain_p = 3.0f;  model_pi_aw_init(&pi);
	CHECK(pi.aw_inv_kp_q15 == 10923, "T-KAW3: Kaw tracks gain_p (3.0 -> 10923)");
	pi.gain_p = 0.5f;  model_pi_aw_init(&pi);
	CHECK(pi.aw_inv_kp_q15 == 65536, "T-KAW4: Kaw tracks gain_p (0.5 -> 65536)");

	/* Guards: a pathologically small gain_p must clamp rather than let the ISR multiply
	 * overflow, and a non-positive one must disable tracking rather than divide by zero. */
	pi.gain_p = 0.0001f; model_pi_aw_init(&pi);
	CHECK(pi.aw_inv_kp_q15 == MODEL_AW_INV_KP_Q15_MAX,
		"T-KAW5: an absurdly small gain_p clamps at FOC_AW_INV_KP_Q15_MAX instead of overflowing");
	pi.gain_p = 0.0f; model_pi_aw_init(&pi);
	CHECK(pi.aw_inv_kp_q15 == 0, "T-KAW6: gain_p=0 disables tracking, no division by zero");
	pi.gain_p = -1.5f; model_pi_aw_init(&pi);
	CHECK(pi.aw_inv_kp_q15 == 0, "T-KAW7: negative gain_p disables tracking");

	/* The overflow bound the ISR relies on, stated as arithmetic rather than as a comment. */
	{
		int64_t worst = (int64_t)MODEL_AW_SAT_ERROR_MAX * (int64_t)MODEL_AW_INV_KP_Q15_MAX;
		CHECK(worst <= 2147483647LL,
			"T-KAW8: sat_error(2^12) * Kaw_q15(2^18) = 2^30 stays inside int32 - no overflow");
	}
}

/* T-PARITY: with no saturation the new regulator is BIT-IDENTICAL to the old one. */
static void no_saturation_parity_checks(void)
{
	model_foc_t with_aw, without_aw;
	int i;
	bool integral_bitwise_equal = true;
	bool out_equal = true;
	bool never_saturated = true;
	bool residual_always_zero = true;
	bool integral_actually_moved = false;

	model_foc_init(&with_aw, true);
	model_foc_init(&without_aw, false);

	CHECK(with_aw.pi_q.aw_inv_kp_q15 != 0 && without_aw.pi_q.aw_inv_kp_q15 == 0,
		"setup: parity run really compares FOC-AW1 enabled against the pre-card regulator");

	/* 20 000 cycles (1.25 s at 16 kHz) of a demand deliberately kept small enough that the
	 * vector never reaches _U_MAX, with the measurement lagging and dithering around it. */
	for (i = 0; i < 20000; i++) {
		int32_t iq_ref = 60 + lcg_next(-25, 25);
		int16_t iq_meas = (int16_t)(iq_ref - lcg_next(-8, 8));
		int16_t id_meas = (int16_t)lcg_next(-4, 4);

		model_run_pi_control(&with_aw, iq_ref, iq_meas, 0, id_meas);
		model_run_pi_control(&without_aw, iq_ref, iq_meas, 0, id_meas);

		if (with_aw.saturated) never_saturated = false;
		if (with_aw.u_q_sat_err != 0 || with_aw.u_d_sat_err != 0) residual_always_zero = false;
		if (memcmp(&with_aw.pi_q.integral_part, &without_aw.pi_q.integral_part,
		           sizeof(float)) != 0) integral_bitwise_equal = false;
		if (memcmp(&with_aw.pi_d.integral_part, &without_aw.pi_d.integral_part,
		           sizeof(float)) != 0) integral_bitwise_equal = false;
		if (with_aw.u_q != without_aw.u_q || with_aw.u_d != without_aw.u_d) out_equal = false;
		if (with_aw.pi_q.integral_part != 0.0f) integral_actually_moved = true;
	}

	CHECK(never_saturated, "setup: the parity scenario genuinely never saturates the limiter");
	CHECK(integral_actually_moved,
		"setup: the parity scenario genuinely exercises the integrator (not a null run)");
	CHECK(residual_always_zero, "T-PARITY1: an unsaturated cycle publishes an exact zero residual");
	CHECK(integral_bitwise_equal,
		"T-PARITY2: 20000 unsaturated cycles leave both PI integrators BIT-IDENTICAL to pre-FOC-AW1");
	CHECK(out_equal, "T-PARITY3: the applied Vd/Vq vector is identical over the same run");
}

/* T-SAT: entering saturation produces a residual with the right magnitude and the right sign,
 * on BOTH axes and BOTH polarities. */
static void saturation_residual_checks(void)
{
	model_foc_t m;
	int i;

	/* --- Positive demand on both axes. --------------------------------------------------
	 *
	 * A Q-only demand can NEVER reach the circle limiter, and that is worth stating because it
	 * is the whole shape of the problem: each PI's own limit_output is already _U_MAX, so a
	 * lone axis stops at 1920 and |(0, 1920)| == _U_MAX exactly - not greater. The vector
	 * limiter binds only when BOTH axes are loaded and are therefore competing for one budget,
	 * which is precisely the case each PI's local clamps cannot see. Every saturation scenario
	 * below loads both axes for that reason.
	 *
	 * The references are also deliberately MODERATE (120, not 700). At 120 the loop settles
	 * with both PI outputs near 1538 - inside limit_output, and with both integrals inside
	 * limit_i - so the circle limiter is the SOLE binding constraint and the tracking behaviour
	 * is observed on its own rather than underneath a local clamp. */
	model_foc_init(&m, true);
	for (i = 0; i < 6000; i++) model_run_pi_control(&m, 120, 0, 120, 0);

	CHECK(m.saturated == 1 && m.sat_ticks > 0,
		"setup: a sustained demand on both axes does reach the circle limiter");
	CHECK(labs((long)m.pi_q.out) < MODEL_LIMIT_OUT && labs((long)m.pi_d.out) < MODEL_LIMIT_OUT &&
		fabsf(m.pi_q.integral_part) < (float)MODEL_LIMIT_I_Q &&
		fabsf(m.pi_d.integral_part) < (float)MODEL_LIMIT_I_D,
		"setup: no local clamp is binding here - the circle limiter is the only active limit");
	CHECK(m.u_abs_req > MODEL_U_MAX && m.u_abs == MODEL_U_MAX,
		"T-SAT1: requested and applied magnitudes are kept separate (u_abs_req > _U_MAX == u_abs)");
	CHECK(m.u_q_sat_err == m.u_q_req - m.u_q,
		"T-SAT2: sat_error_q is exactly Vq_requested - Vq_applied");
	CHECK(m.u_d_sat_err == m.u_d_req - m.u_d,
		"T-SAT2b: sat_error_d is exactly Vd_requested - Vd_applied");
	CHECK(m.u_q_req > 0 && m.u_q_sat_err > 0,
		"T-SAT3: a positive over-demanded Vq leaves a POSITIVE residual (unwinds the integrator down)");
	CHECK(m.pi_q.aw_sat_error == m.u_q_sat_err,
		"T-SAT4: PI_iq consumes the physical Vq residual unchanged (PI_iq.out IS Vq)");

	/* --- Negative Q demand: every sign must mirror. ------------------------------------ */
	model_foc_init(&m, true);
	for (i = 0; i < 6000; i++) model_run_pi_control(&m, -120, 0, 120, 0);
	CHECK(m.saturated == 1 && m.u_q_req < 0 && m.u_q_sat_err < 0,
		"T-SAT5: a negative over-demanded Vq leaves a NEGATIVE residual (unwinds the integrator up)");

	/* --- D axis: the sign flip is the one thing that can be silently wrong. ------------
	 * Vd = -PI_id.out, so the residual PI_id must see is the NEGATED physical one. Getting
	 * this backwards would drive the D integrator further into saturation instead of out of
	 * it, and nothing else in the system would complain. */
	model_foc_init(&m, true);
	for (i = 0; i < 6000; i++) model_run_pi_control(&m, 120, 0, 120, 0);
	CHECK(m.saturated == 1, "setup: the D-axis scenario reaches the limiter");
	CHECK(m.pi_d.aw_sat_error == -m.u_d_sat_err,
		"T-SAT6: PI_id consumes the NEGATED physical Vd residual (Vd = -PI_id.out)");
	CHECK((m.pi_d.aw_sat_error > 0) == (m.pi_d.out > 0),
		"T-SAT7: the D residual has the same sign as PI_id.out - it unwinds, never winds up");
	CHECK((m.pi_q.aw_sat_error > 0) == (m.pi_q.out > 0),
		"T-SAT7b: the Q residual has the same sign as PI_iq.out - it unwinds, never winds up");

	/* Same demand with the D reference mirrored: both signs must mirror with it. */
	model_foc_init(&m, true);
	for (i = 0; i < 6000; i++) model_run_pi_control(&m, 120, 0, -120, 0);
	CHECK(m.saturated == 1 && (m.pi_d.aw_sat_error > 0) == (m.pi_d.out > 0),
		"T-SAT8: the D residual sign still tracks PI_id.out with the D reference mirrored");
}

/* T-DELAY: the residual computed in cycle N is what corrects the integrator in cycle N+1 -
 * not the same cycle, and not two cycles later. */
static void one_cycle_delay_checks(void)
{
	model_foc_t m;
	int i;
	float integral_before, integral_after, expected;
	int32_t residual_from_cycle_n;

	model_foc_init(&m, true);
	for (i = 0; i < 3000; i++) model_run_pi_control(&m, 120, 0, 120, 0);
	CHECK(m.saturated == 1, "setup: the delay scenario is saturated before the probe");

	/* State at the end of cycle N. */
	residual_from_cycle_n = m.pi_q.aw_sat_error;
	integral_before = m.pi_q.integral_part;
	CHECK(residual_from_cycle_n != 0, "setup: cycle N really left a non-zero residual");

	/* Cycle N+1, with the same inputs, must apply exactly that residual - and nothing else
	 * about the integral update may have changed. Computed here from the card's equation
	 * rather than from the model, so this is an independent statement of the arithmetic. */
	{
		float delta = (float)(120 - 0);
		float aw = (float)((residual_from_cycle_n * m.pi_q.aw_inv_kp_q15) >> 15);
		expected = integral_before + (delta - aw) * MODEL_GAIN_I;
		if (expected > MODEL_LIMIT_I_Q) expected = MODEL_LIMIT_I_Q;
		if (expected < -MODEL_LIMIT_I_Q) expected = -MODEL_LIMIT_I_Q;
	}
	model_run_pi_control(&m, 120, 0, 120, 0);
	integral_after = m.pi_q.integral_part;

	CHECK(integral_after == expected,
		"T-DELAY1: cycle N's residual corrects cycle N+1's integrator, exactly per the card equation");
	CHECK(expected < integral_before + (120.0f * MODEL_GAIN_I),
		"T-DELAY2: the correction genuinely subtracts from what the uncorrected update would have been");
}

/* T-EXIT: leaving saturation must not leave an over-charged integral. This is the behaviour the
 * card exists to produce, so it is checked against the pre-card regulator, not in isolation. */
static void saturation_exit_checks(void)
{
	model_foc_t with_aw, without_aw;
	int i;
	float peak_with = 0.0f, peak_without = 0.0f;
	int settle_with = -1, settle_without = -1;
	int32_t resid_at_settle = -1;
	uint8_t sat_at_settle = 1;

	model_foc_init(&with_aw, true);
	model_foc_init(&without_aw, false);

	/* Wind-up phase: 6000 cycles (~0.4 s at 16 kHz) of a demand on both axes that the vector
	 * limiter cannot fully honour, but that leaves both local clamps slack. See the note in
	 * saturation_residual_checks() for why both axes and why 120 rather than 700. */
	for (i = 0; i < 6000; i++) {
		model_run_pi_control(&with_aw, 120, 0, 120, 0);
		model_run_pi_control(&without_aw, 120, 0, 120, 0);
		if (with_aw.pi_q.integral_part > peak_with) peak_with = with_aw.pi_q.integral_part;
		if (without_aw.pi_q.integral_part > peak_without) peak_without = without_aw.pi_q.integral_part;
	}

	CHECK(with_aw.saturated == 1 && without_aw.saturated == 1,
		"setup: both regulators are saturated at the end of the wind-up phase");
	CHECK(fabsf(with_aw.pi_q.integral_part) < (float)MODEL_LIMIT_I_Q &&
		labs((long)with_aw.pi_q.out) < MODEL_LIMIT_OUT,
		"setup: under FOC-AW1 no local clamp is binding - what bounds the integral is the tracking");

	/* THE steady-state property of Kaw = 1/Kp: under sustained saturation the integrator
	 * settles at the APPLIED voltage, so it can never charge past what the bridge delivers.
	 * The pre-card regulator, by contrast, only stops at its own limit_i. */
	CHECK(fabsf(with_aw.pi_q.integral_part - (float)with_aw.u_q) < 5.0f,
		"T-EXIT1: under sustained saturation the Q integrator settles at the APPLIED Vq (Kaw=1/Kp)");
	CHECK(fabsf(without_aw.pi_q.integral_part - (float)MODEL_LIMIT_I_Q) < 1.0f,
		"T-EXIT1b: the pre-card regulator instead charges all the way to limit_i - the defect");
	CHECK(without_aw.pi_q.integral_part > with_aw.pi_q.integral_part + 400.0f,
		"T-EXIT2: FOC-AW1 ends the same phase several hundred units less charged");
	CHECK(peak_without >= peak_with,
		"T-EXIT2b: FOC-AW1 never charges the integrator higher than the pre-card regulator did");

	/* Release. The demand goes to zero while a real residual current is still measured, so
	 * both regulators have a genuine negative error to unwind against - the same error, from
	 * different starting charges. Count cycles until the applied Vq is essentially gone.
	 *
	 * No plant model: feeding both regulators an IDENTICAL measured-current sequence is what
	 * makes this a clean comparison of the stored charge rather than of invented motor
	 * dynamics. The claim under test is only "exiting saturation leaves no excess windup". */
	for (i = 0; i < 20000; i++) {
		model_run_pi_control(&with_aw, 0, 100, 0, 100);
		model_run_pi_control(&without_aw, 0, 100, 0, 100);
		if (settle_with < 0 && with_aw.u_q <= 20) {
			settle_with = i;
			/* Sampled AT the release, not at the end of the loop: this scenario keeps feeding a
			 * constant residual current, so both regulators eventually charge the other way and
			 * saturate again in the opposite direction. What the card asks about is the moment
			 * saturation is left. */
			resid_at_settle = with_aw.pi_q.aw_sat_error;
			sat_at_settle = with_aw.saturated;
		}
		if (settle_without < 0 && without_aw.u_q <= 20) settle_without = i;
	}

	CHECK(settle_with >= 0, "setup: FOC-AW1 does release after the demand is removed");
	CHECK(settle_without >= 0, "setup: the pre-card regulator also releases (bounded comparison)");
	CHECK(settle_with >= 0 && settle_without >= 0 && settle_with < settle_without,
		"T-EXIT3: FOC-AW1 leaves saturation with less stored charge, so it releases sooner");
	CHECK(resid_at_settle == 0 && sat_at_settle == 0,
		"T-EXIT4: by the time the output has released, the vector is unsaturated and the residual is exactly zero");
}

/* T-CLAMP: the local clamps the card requires to survive, still survive. */
static void local_clamp_checks(void)
{
	model_foc_t m;
	int i;
	bool integral_within_limit = true;
	bool out_within_limit = true;
	bool step_respected = true;
	int32_t prev_out;

	model_foc_init(&m, true);
	prev_out = m.pi_q.out;
	for (i = 0; i < 6000; i++) {
		/* Alternating extreme references: the roughest thing the output slew can be asked to
		 * follow, and the case where a broken clamp shows up immediately. */
		int32_t iq_ref = (i % 200 < 100) ? 700 : -700;
		model_run_pi_control(&m, iq_ref, 0, 400, -400);
		if (fabsf(m.pi_q.integral_part) > (float)MODEL_LIMIT_I_Q + 0.001f) integral_within_limit = false;
		if (fabsf(m.pi_d.integral_part) > (float)MODEL_LIMIT_I_D + 0.001f) integral_within_limit = false;
		if (labs((long)m.pi_q.out) > MODEL_LIMIT_OUT) out_within_limit = false;
		if (labs((long)m.pi_d.out) > MODEL_LIMIT_OUT) out_within_limit = false;
		if (labs((long)(m.pi_q.out - prev_out)) > MODEL_MAX_STEP) step_respected = false;
		prev_out = m.pi_q.out;
	}

	CHECK(integral_within_limit,
		"T-CLAMP1: limit_i still binds on both axes with tracking active (D's tighter 1800 included)");
	CHECK(out_within_limit, "T-CLAMP2: limit_output still binds on both axes");
	CHECK(step_respected, "T-CLAMP3: the max_step output slew is untouched by FOC-AW1");
	CHECK(m.u_abs <= MODEL_U_MAX, "T-CLAMP4: the vector limiter still caps the applied magnitude");
}

/* T-OVF: fixed-point safety under inputs no production path can produce. */
static void overflow_checks(void)
{
	model_foc_t m;
	model_pi_t pi;
	int i;
	bool residual_bounded = true;

	model_foc_init(&m, true);
	for (i = 0; i < 3000; i++) {
		model_run_pi_control(&m, lcg_next(-700, 700), (int16_t)lcg_next(-700, 700),
		                     lcg_next(-700, 700), (int16_t)lcg_next(-700, 700));
		if (labs((long)m.pi_q.aw_sat_error) > MODEL_AW_SAT_ERROR_MAX) residual_bounded = false;
		if (labs((long)m.pi_d.aw_sat_error) > MODEL_AW_SAT_ERROR_MAX) residual_bounded = false;
		if (labs((long)m.u_q_sat_err) > 2 * MODEL_U_MAX) residual_bounded = false;
		if (labs((long)m.u_d_sat_err) > 2 * MODEL_U_MAX) residual_bounded = false;
		if (!isfinite(m.pi_q.integral_part) || !isfinite(m.pi_d.integral_part)) residual_bounded = false;
	}
	CHECK(residual_bounded,
		"T-OVF1: 3000 cycles of adversarial references keep every residual inside its bound and the integrals finite");

	/* The clamp itself, forced. No production path can hand PI_control() a residual this
	 * large - both operands are clamped to limit_output - but the ISR multiply must stay
	 * inside int32 if some future change ever breaks that invariant. */
	model_pi_init(&pi, MODEL_LIMIT_I_Q, true);
	pi.aw_inv_kp_q15 = MODEL_AW_INV_KP_Q15_MAX;
	pi.aw_sat_error = MODEL_AW_SAT_ERROR_MAX;
	pi.setpoint = 700;
	(void)model_pi_control(&pi);
	CHECK(isfinite(pi.integral_part) && fabsf(pi.integral_part) <= (float)MODEL_LIMIT_I_Q,
		"T-OVF2: worst-case residual x worst-case Kaw still produces a finite, clamped integral");

	pi.aw_sat_error = -MODEL_AW_SAT_ERROR_MAX;
	(void)model_pi_control(&pi);
	CHECK(isfinite(pi.integral_part) && fabsf(pi.integral_part) <= (float)MODEL_LIMIT_I_Q,
		"T-OVF3: the same holds for the negative extreme (arithmetic shift, not logical)");

	/* The negative shift really is arithmetic on this toolchain - the fixed-point path
	 * assumes it, so it is asserted rather than trusted. */
	CHECK(((int32_t)(-32768) >> 15) == -1,
		"T-OVF4: >> on a negative int32 is an arithmetic shift on this toolchain");
}

/* ------------------------------------------------------------------------------------------- */
/* Production wiring (source-text guards; main.c/FOC.c cannot be linked on a host)                */
/* ------------------------------------------------------------------------------------------- */

static char *read_whole_file(const char *path, long *out_len)
{
	FILE *f = fopen(path, "rb");
	char *buf;
	long len;
	if (!f) return NULL;
	fseek(f, 0, SEEK_END);
	len = ftell(f);
	fseek(f, 0, SEEK_SET);
	buf = (char *)malloc((size_t)len + 1);
	if (!buf) { fclose(f); return NULL; }
	if (fread(buf, 1, (size_t)len, f) != (size_t)len) { free(buf); fclose(f); return NULL; }
	buf[len] = '\0';
	fclose(f);
	if (out_len) *out_len = len;
	return buf;
}

/* Blanks comment bodies in place, preserving length and byte positions 1:1 (same technique as
 * stopclick_c1_pi_integral_host.c), so a claim about CODE cannot be satisfied by a comment. */
static char *strip_comments(const char *text, long len)
{
	char *out = (char *)malloc((size_t)len + 1);
	long i = 0;
	if (!out) return NULL;
	while (i < len) {
		if (text[i] == '/' && i + 1 < len && text[i + 1] == '/') {
			while (i < len && text[i] != '\n') out[i++] = ' ';
			if (i < len) out[i] = '\n';
			i++;
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

static bool span_contains(const char *first, const char *last, const char *needle)
{
	const char *found = strstr(first, needle);
	return found != NULL && found < last;
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

static void production_wiring_checks(void)
{
	long main_len = 0, foc_len = 0, mainh_len = 0, loop_len = 0;
	char *main_raw = read_whole_file(STRINGIZE(MAIN_C_PATH), &main_len);
	char *foc_raw = read_whole_file(STRINGIZE(FOC_C_PATH), &foc_len);
	char *mainh_raw = read_whole_file(STRINGIZE(MAIN_H_PATH), &mainh_len);
	char *loop_raw = read_whole_file(STRINGIZE(FOC_CURRENT_LOOP_C_PATH), &loop_len);
	char *main_c = NULL, *foc_c = NULL, *main_h = NULL, *loop_c = NULL;

	CHECK(main_raw && foc_raw && mainh_raw && loop_raw,
		"setup: main.c, FOC.c, main.h and foc_current_loop.c are readable");
	if (!main_raw || !foc_raw || !mainh_raw || !loop_raw) goto done;

	main_c = strip_comments(main_raw, main_len);
	foc_c = strip_comments(foc_raw, foc_len);
	main_h = strip_comments(mainh_raw, mainh_len);
	loop_c = strip_comments(loop_raw, loop_len);
	CHECK(main_c && foc_c && main_h && loop_c, "setup: all current-loop sources sanitize successfully");
	if (!main_c || !foc_c || !main_h || !loop_c) goto done;

	/* --- Model-matches-production ------------------------------------------------------ */
	CHECK(strstr(foc_c, "float aw_part = (float)((PI_c->aw_sat_error * PI_c->aw_inv_kp_q15) >> 15);") != NULL &&
		strstr(foc_c, "PI_c->integral_part += (Delta - aw_part)*PI_c->gain_i;") != NULL,
		"W1: FOC.c's PI_control() carries the FOC-AW1 term the replica models");
	CHECK(strstr(foc_c, "p_part= Delta*PI_c->gain_p;") != NULL,
		"W2: the PROPORTIONAL path still uses the raw Delta - tracking corrects only the integrator");
	CHECK(strstr(foc_c, "if (PI_c->integral_part > PI_c->limit_i) PI_c->integral_part = PI_c->limit_i;") != NULL &&
		strstr(foc_c, "if (PI_c->out>PI_c->limit_output) PI_c->out = PI_c->limit_output;") != NULL &&
		strstr(foc_c, "PI_c->out+=PI_c->max_step;") != NULL,
		"W3: the local integral clamp, output clamp and max_step slew are all still present");

	CHECK(strstr(main_h, "#define FOC_AW_INV_KP_Q15_MAX   262144L") != NULL &&
		strstr(main_h, "#define FOC_AW_SAT_ERROR_MAX    4096L") != NULL,
		"W4: the replica's overflow bounds match the shipped FOC_AW_* constants");
	CHECK(strstr(main_h, "int32_t       	aw_sat_error;") != NULL &&
		strstr(main_h, "int32_t       	aw_inv_kp_q15;") != NULL,
		"W5: PI_control_t carries the tracking state the replica models");
	CHECK(strstr(main_h, "int32_t         u_d_req;") != NULL &&
		strstr(main_h, "int32_t         u_q_req;") != NULL &&
		strstr(main_h, "int32_t         u_abs_req;") != NULL &&
		strstr(main_h, "int32_t         u_d_sat_err;") != NULL &&
		strstr(main_h, "int32_t         u_q_sat_err;") != NULL,
		"W6: MotorState_t carries requested Vd/Vq, requested magnitude and both residuals");
	CHECK(strstr(main_h, "extern volatile uint8_t  foc_aw_saturated;") != NULL &&
		strstr(main_h, "extern volatile uint32_t foc_aw_sat_ticks;") != NULL,
		"W7: the saturation flag and counter are published for diagnostics");

	CHECK(strstr(main_c, "float q15 = 32768.0f / PI_c->gain_p + 0.5f;") != NULL &&
		strstr(main_c, "if (q15 > (float)FOC_AW_INV_KP_Q15_MAX) q15 = (float)FOC_AW_INV_KP_Q15_MAX;") != NULL &&
		strstr(main_c, "if (PI_c->gain_p > 0.0f) {") != NULL,
		"W8: pi_aw_init() derives Kaw = 1/gain_p in Q15 with the clamp and the divide-by-zero guard");
	CHECK(strstr(loop_c, "pi_iq->aw_sat_error = sat_q;") != NULL &&
		strstr(loop_c, "pi_id->aw_sat_error = -sat_d;") != NULL,
		"W9: the D-axis sign flip (Vd = -PI_id.out) is applied exactly once in the shared current-loop owner");
	CHECK(strstr(loop_c, "ms->u_q_req = u_q_requested;") != NULL &&
		strstr(loop_c, "ms->u_d_req = u_d_requested;") != NULL &&
		strstr(loop_c, "ms->u_abs_req = ms->u_abs;") != NULL,
		"W10: the shared current-loop owner records the REQUESTED vector before limiting");
	CHECK(strstr(loop_c, "ms->u_q = (u_q_requested * _U_MAX) / ms->u_abs;") != NULL &&
		strstr(loop_c, "ms->u_abs = _U_MAX;") != NULL,
		"W11: the vector limiter itself is preserved in the shared production module");

	/* --- The residual must be published AFTER the limiter, and after the requested store. */
	{
		const char *owner = strstr(loop_c, "void foc_current_loop_step(");
		const char *limiter = owner ? strstr(owner, "if (ms->u_abs > _U_MAX)") : NULL;
		const char *publish = owner ? strstr(owner, "pi_iq->aw_sat_error = sat_q;") : NULL;
		const char *req_store = owner ? strstr(owner, "ms->u_q_req = u_q_requested;") : NULL;
		CHECK(owner && limiter && publish && req_store,
			"setup: shared current-loop limiter and residual publication are locatable");
		if (owner && limiter && publish && req_store) {
			CHECK(req_store < limiter,
				"W12: the requested vector is stored BEFORE the limiter runs");
			CHECK(limiter < publish,
				"W13: the residual is published AFTER the limiter has decided what is applied");
		}
	}

	/* --- ISR cost: the new arithmetic adds no division, no float division, no loop. ----- */
	{
		const char *pi_ctrl = strstr(foc_c, "q31_t PI_control (PI_control_t* PI_c)");
		/* Anchored FROM pi_ctrl, not from the top of the file: FOC.c forward-declares svpwm()
		 * near line 57, long before PI_control(), so a bare strstr() finds the prototype and
		 * collapses the span to nothing. */
		const char *svpwm = pi_ctrl ? strstr(pi_ctrl, "void svpwm(q31_t q31_u_alpha") : NULL;
		CHECK(pi_ctrl && svpwm && pi_ctrl < svpwm, "setup: PI_control()'s body is locatable");
		if (pi_ctrl && svpwm && pi_ctrl < svpwm) {
			CHECK(count_in_span(pi_ctrl, svpwm, "/") == 0,
				"W14: PI_control() contains no division of any kind (integer or float)");
			CHECK(count_in_span(pi_ctrl, svpwm, "while") == 0 &&
				count_in_span(pi_ctrl, svpwm, "for") == 0 &&
				count_in_span(pi_ctrl, svpwm, "goto") == 0,
				"W15: PI_control() contains no loop, bounded or otherwise");
			CHECK(count_in_span(pi_ctrl, svpwm, "sqrt") == 0 &&
				count_in_span(pi_ctrl, svpwm, "__disable_irq") == 0 &&
				count_in_span(pi_ctrl, svpwm, "Mutex") == 0,
				"W16: PI_control() takes no lock and calls no library math");
		}
		/* The float division the derivation needs lives in the foreground init only. */
		CHECK(strstr(foc_c, "32768.0f") == NULL,
			"W17: the 1/gain_p division is nowhere in FOC.c - it happens once, at boot, in main.c");
	}

	/* --- Reset sites: exactly the three the card allows, plus boot init. ---------------- */
	CHECK(count_in_span(main_c, main_c + main_len, "foc_aw_tracking_reset();") == 4,
		"W18: foc_aw_tracking_reset() has exactly four call sites (boot init + the three lifecycle ones)");

	{
		/* Cold PREPARE (GATE A: bridge fully off, before MOE ON). The 4200-char window is sized
		 * with headroom: QZERO added quiet_zero_reset() to this same block, which moved the
		 * MOE_ON anchor to 3817 and past the previous 3800. */
		const char *cold_prepare = strstr(main_c, "if(!ui_8_PWM_ON_Flag){");
		const char *dwell_failsafe = strstr(main_c, "static uint16_t dwell_timeout_counter");
		const char *hall_cal = strstr(main_c, "uint16_t hall_calibration_iq_request(void){");

		CHECK(cold_prepare != NULL, "setup: cold PREPARE block (GATE A) is locatable");
		if (cold_prepare) {
			CHECK(span_contains(cold_prepare, cold_prepare + 4200, "PI_iq.integral_part=0; PI_iq.out=0;") &&
				span_contains(cold_prepare, cold_prepare + 4200, "foc_aw_tracking_reset();") &&
				span_contains(cold_prepare, cold_prepare + 4200, "bridge_lifecycle = BRIDGE_LIFECYCLE_MOE_ON;"),
				"W19: cold PREPARE zeroes the tracking state alongside the PI integrators it already zeroed");
		}

		/* Dwell-timeout failsafe, on the way to IDLE. */
		CHECK(dwell_failsafe != NULL &&
			span_contains(dwell_failsafe, dwell_failsafe + 1500, "foc_aw_tracking_reset();") &&
			span_contains(dwell_failsafe, dwell_failsafe + 1500, "bridge_lifecycle = BRIDGE_LIFECYCLE_IDLE;"),
			"W20: the dwell-timeout failsafe resets tracking as the bridge goes IDLE");

		/* Hall-calibration service path, disabling the bridge in the same breath. M820
		 * comm-inhibit fix: that exit moved, unchanged, into hall_calibration_bridge_off(),
		 * shared by the verified completion and the comms-loss abort. */
		const char *exit_fn = strstr(main_c, "static void hall_calibration_bridge_off(void)");
		while (exit_fn && exit_fn[45] == ';') /* skip the prototype, keep the definition */
			exit_fn = strstr(exit_fn + 1, "static void hall_calibration_bridge_off(void)");
		CHECK(hall_cal != NULL &&
			span_contains(hall_cal, hall_cal + 1100, "hall_calibration_bridge_off();") &&
			exit_fn != NULL &&
			span_contains(exit_fn, exit_fn + 1100, "foc_aw_tracking_reset();") &&
			span_contains(exit_fn, exit_fn + 1100, "timer_primary_output_config(TIMER0,DISABLE)"),
			"W21: the hall-calibration service reset includes tracking, still coherent with bridge-off");
	}

	/* --- What must NOT reset: ordinary zero torque and RUN <-> ARMED_ZERO. -------------
	 * This is STOP-CLICK-C1's invariant restated for the new state. A foreground writer
	 * clearing tracking state up to 4000x/s would race the 16 kHz owner in exactly the way
	 * that card removed, and this card must not reintroduce it in a new field. */
	{
		const char *reg_adc_start = find_span_open(main_c, "void reg_ADC_processing(void)");
		const char *reg_adc_end = strstr(main_c, "int16_t internal_tics_to_speedx100 (uint32_t tics){");
		const char *zero_transition = strstr(main_c,
			"bridge_lifecycle == BRIDGE_LIFECYCLE_RUN && MS.i_q_setpoint == 0");
		const char *dwell_failsafe = strstr(main_c, "static uint16_t dwell_timeout_counter");

		CHECK(reg_adc_start && reg_adc_end && reg_adc_start < reg_adc_end,
			"setup: reg_ADC_processing() span is locatable");
		if (reg_adc_start && reg_adc_end && reg_adc_start < reg_adc_end) {
			CHECK(count_in_span(reg_adc_start, reg_adc_end, "foc_aw_tracking_reset") == 0,
				"W22: the 4 kHz foreground never resets tracking state (no new STOP-CLICK-C1 race)");
			CHECK(count_in_span(reg_adc_start, reg_adc_end, "PI_iq.integral_part=") == 0 &&
				count_in_span(reg_adc_start, reg_adc_end, "PI_id.integral_part=") == 0,
				"W23: STOP-CLICK-C1's removal of the foreground integral writers still stands");
		}

		CHECK(zero_transition && dwell_failsafe && zero_transition < dwell_failsafe,
			"setup: the RUN <-> ARMED_ZERO transition span is locatable");
		if (zero_transition && dwell_failsafe && zero_transition < dwell_failsafe) {
			CHECK(count_in_span(zero_transition, dwell_failsafe, "foc_aw_tracking_reset") == 0,
				"W24: ordinary ARMED_ZERO resets neither the tracking state nor the integrators");
		}
	}

	/* --- Hard fault is untouched. ------------------------------------------------------ */
	{
		const char *hard_fault = strstr(foc_c, "if(MS_FOC->i_d>(PH_CURRENT_MAX<<2))");
		CHECK(hard_fault != NULL &&
			span_contains(hard_fault, hard_fault + 900, "timer_primary_output_config(TIMER0,DISABLE)") &&
			span_contains(hard_fault, hard_fault + 900, "bridge_lifecycle=BRIDGE_LIFECYCLE_FAULT") &&
			span_contains(hard_fault, hard_fault + 900, "while(1){}"),
			"W25: the independent hard-fault MOE-off path is unchanged");
	}

	/* --- Out of scope: nothing this card forbids touching may have moved. -------------- */
	CHECK(strstr(main_c, "PI_iq.gain_p=P_FACTOR_I_Q;") != NULL &&
		strstr(main_c, "PI_id.gain_p=P_FACTOR_I_D;") != NULL &&
		strstr(main_c, "PI_iq.gain_i=I_FACTOR_I_Q;") != NULL &&
		strstr(main_c, "PI_id.gain_i=I_FACTOR_I_D;") != NULL &&
		strstr(main_c, "PI_iq.limit_i=_U_MAX;") != NULL &&
		strstr(main_c, "PI_id.limit_i=1800;") != NULL &&
		strstr(main_c, "PI_iq.max_step=15;") != NULL,
		"W26: Kp, Ki and every clamp are exactly as they were - this card changes no gain");
	CHECK(strstr(main_c, "fast_iq_slew_tick(") != NULL &&
		strstr(main_c, "PI_iq.setpoint = MP.reverse * i8_reverse_flag * MS.i_q_setpoint;") != NULL,
		"W27: the final Iq slew ownership (QS-3D / FW-128A) is untouched");

done:
	free(main_c);
	free(foc_c);
	free(main_h);
	free(loop_c);
	free(main_raw);
	free(foc_raw);
	free(mainh_raw);
	free(loop_raw);
}

int main(void)
{
	puts("FOC-AW1: D/Q voltage tracking anti-windup");
	kaw_derivation_checks();
	no_saturation_parity_checks();
	saturation_residual_checks();
	one_cycle_delay_checks();
	saturation_exit_checks();
	local_clamp_checks();
	overflow_checks();
	production_wiring_checks();
	if (host_test_failures == 0) {
		puts("FOC-AW1 tracking anti-windup: ALL CHECKS PASSED");
		return 0;
	}
	printf("FOC-AW1 tracking anti-windup: %d CHECK(S) FAILED\n", host_test_failures);
	return 1;
}
