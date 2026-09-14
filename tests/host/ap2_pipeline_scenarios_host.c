/*
 * ASSIST PIPELINE V2 - behavioural scenario proof.
 *
 * Every check below runs the SHIPPED pipeline (ap2_*.c + assist_pipeline.c + ap2_limits.c +
 * battery_iq_cap.c + fast_iq_slew.c) over a synthetic but realistic pedal input. Nothing here
 * is a model of the pipeline; the only model is the RIDER.
 *
 * The scenarios are the ones a rider can describe, because those are the ones that can be
 * checked against the bike later: calm riding, a harder push, an aggressive burst, high and low
 * cadence, a climb, easing off, stopping, back-pedalling, restarting, each profile, the
 * adaptive profiles, and each limiter.
 *
 * WHAT IS DELIBERATELY NOT ASSERTED. No check pins an exact Iq count. The numbers in this
 * pipeline are ride-feel settings that are expected to be tuned on the physical bike; a test
 * that froze them would turn every future tuning change into a test failure and teach whoever
 * hits it to edit the expectation. What is asserted are the INVARIANTS and the ORDERINGS -
 * "SPORT+ answers a push sooner than ECO", "a reverse step removes the request in the same
 * tick", "the sustained term does not collapse in the dead spot" - which stay true across
 * tuning and stop being true the moment the architecture regresses.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "fast_iq_slew.h"
#include <string.h>

#include "assist_modes.h"
#include "assist_pipeline.h"
#include "config.h"

static int failures;

#define CHECK(cond, what) do { \
	if (!(cond)) { \
		printf("  FAIL  %s\n", (what)); \
		failures++; \
	} \
} while (0)

/* ------------------------------------------------------------------ the rider model ------- */

#define TICKS_PER_MS 4U
#define MS(x) ((uint32_t)(x) * TICKS_PER_MS)

/*
 * One pedal stroke per half revolution, with a dead spot between them. This is the shape the
 * whole demand model exists to handle, so the tests must not be run against a flat torque.
 */
static uint16_t pedal_force_centikg(uint32_t tick, uint8_t cadence_rpm, uint16_t peak_centikg)
{
	uint32_t period_ticks;
	uint32_t phase;
	int32_t tri;

	if (cadence_rpm == 0U) {
		return 0U;
	}
	/* half a revolution, in 4 kHz ticks */
	period_ticks = (30000U * TICKS_PER_MS) / cadence_rpm;
	if (period_ticks == 0U) {
		period_ticks = 1U;
	}
	phase = tick % period_ticks;
	/* A triangle from a 15 % floor to the peak and back: enough dead spot to be a real test. */
	if (phase * 2U < period_ticks) {
		tri = (int32_t)((phase * 2000U) / period_ticks);
	} else {
		tri = (int32_t)(2000U - ((phase - period_ticks / 2U) * 2000U) / (period_ticks / 2U + 1U));
	}
	if (tri < 0) {
		tri = 0;
	}
	if (tri > 1000) {
		tri = 1000;
	}
	return (uint16_t)(((uint32_t)peak_centikg * (150U + (uint32_t)tri * 850U / 1000U)) / 1000U);
}

typedef struct {
	uint8_t level;
	uint8_t cadence_rpm;
	uint16_t peak_centikg;
	uint32_t speed_x100;
	bool forward;
	bool reverse_step;
	bool safety_cut;
	int32_t battery_current_ma;
	int16_t temperature_c;
	uint16_t speed_limit_x100;
	bool legal;
} ride_t;

static void base_ride(ride_t *r)
{
	memset(r, 0, sizeof(*r));
	r->level = 3U;
	r->cadence_rpm = 70U;
	r->peak_centikg = 900U;
	r->speed_x100 = 1800U;
	r->forward = true;
	r->battery_current_ma = 4000;
	r->temperature_c = 30;
	r->speed_limit_x100 = 2500U;
	r->legal = false;
}

static uint32_t g_tick;

/*
 * THE REAL FINAL OWNER, not a model of it.
 *
 * Every scenario publishes its command to the production mailbox and advances the production
 * 16 kHz owner four times - one 4 kHz control period. That matters for two reasons: the
 * pipeline chooses RISE/FALL/HOLD by reading the owner's live accumulator, so without it the
 * trajectory decisions in the test are not the ones the firmware makes; and a claim about what
 * the current regulator is handed can only be checked where that value is actually produced.
 */
#define FOC_TICKS_PER_CONTROL 4U

static fast_iq_slew_mailbox_t g_mb;
static int32_t g_iq_ref;

/* The Iq reference the current regulator saw on the FIRST ISR tick of the last control tick. */
static int32_t g_iq_ref_first;

static void pipeline_tick(const ride_t *r, assist_pipeline_command_t *cmd)
{
	assist_pipeline_input_t in;

	memset(&in, 0, sizeof(in));
	in.torque_load_centikg = r->forward ?
		pedal_force_centikg(g_tick, r->cadence_rpm, r->peak_centikg) : 0U;
	in.torque_sensor_valid = true;
	in.cadence_rpm = r->forward ? r->cadence_rpm : 0U;
	in.speed_x100 = r->speed_x100;
	in.motor_erps = 400U;
	in.forward_valid = r->forward;
	in.direction_inhibit = r->reverse_step;
	in.inhibit_is_reverse = r->reverse_step;
	in.real_stop = !r->forward;
	in.wheel_valid = r->speed_x100 > 0U;
	in.pas_sensor_valid = true;
	in.forward_steps = r->forward ? 250U : 0U;
	in.required_steps = 4U;
	in.assist_level_index = r->level;
	in.safety_cut = r->safety_cut;
	in.battery_voltage_mv = 42000U;
	in.battery_current_ma = r->battery_current_ma;
	in.battery_current_max = 15000;
	in.u_abs = 1024;
	in.cal_i = 95;
	in.level_iq_limit = (int32_t)PH_CURRENT_MAX;
	in.phase_current_max = (int32_t)PH_CURRENT_MAX;
	in.voltage_raw = 4000U;
	in.voltage_min_raw = 2800;
	in.controller_temperature_c = r->temperature_c;
	in.speed_limit_x100 = r->speed_limit_x100;
	in.legal_enabled = r->legal;
	in.elapsed_ticks = 1U;
	assist_pipeline_update(&in, cmd);

	fast_iq_slew_publish(&g_mb, cmd->final_iq_request, cmd->slew_mode, cmd->step_mag_8,
		cmd->release_ticks_16k, cmd->zero_policy, cmd->iq_ceiling);
	for (unsigned k = 0; k < FOC_TICKS_PER_CONTROL; k++) {
		fast_iq_slew_tick(&g_mb, &g_iq_ref);
		if (k == 0U) {
			g_iq_ref_first = g_iq_ref;
		}
	}
	g_tick++;
}

/* Run for `ticks`, returning the mean and the peak-to-peak of the final request. */
typedef struct {
	int32_t mean;
	int32_t min;
	int32_t max;
	int32_t last;
	int32_t base_min;
	int32_t base_mean;
	uint32_t ticks_to_first_current;
	uint32_t ticks_to_zero;
} run_stats_t;

/*
 * `settle` ticks are run first and excluded from the statistics. A ride starts from zero, so a
 * minimum taken from tick 0 would always be 0 and any check on it would be meaningless - the
 * questions these scenarios ask are about the STEADY behaviour, and the start has its own.
 */
static void run_after(const ride_t *r, uint32_t settle, uint32_t ticks, run_stats_t *st)
{
	assist_pipeline_command_t cmd;
	int64_t sum = 0;
	int64_t base_sum = 0;
	uint32_t i;

	memset(st, 0, sizeof(*st));
	st->min = INT32_MAX;
	st->base_min = INT32_MAX;
	for (i = 0; i < settle; i++) {
		pipeline_tick(r, &cmd);
	}
	for (i = 0; i < ticks; i++) {
		const assist_pipeline_telemetry_t *t;
		pipeline_tick(r, &cmd);
		t = assist_pipeline_telemetry();
		sum += cmd.final_iq_request;
		base_sum += t->assist_base_permille;
		if (cmd.final_iq_request < st->min) {
			st->min = cmd.final_iq_request;
		}
		if (cmd.final_iq_request > st->max) {
			st->max = cmd.final_iq_request;
		}
		if (t->assist_base_permille < st->base_min) {
			st->base_min = t->assist_base_permille;
		}
		if (st->ticks_to_first_current == 0U && cmd.final_iq_request > 0) {
			st->ticks_to_first_current = i + 1U;
		}
		if (st->ticks_to_zero == 0U && st->ticks_to_first_current != 0U &&
			cmd.final_iq_request == 0) {
			st->ticks_to_zero = i + 1U;
		}
		st->last = cmd.final_iq_request;
	}
	st->mean = (int32_t)(sum / (int64_t)ticks);
	st->base_mean = (int32_t)(base_sum / (int64_t)ticks);
}

static void run(const ride_t *r, uint32_t ticks, run_stats_t *st)
{
	run_after(r, 0U, ticks, st);
}

static void reset_all(uint8_t bank)
{
	assist_modes_init();
	assist_modes_set_active_bank(bank);
	assist_pipeline_init();
	memset(&g_mb, 0, sizeof(g_mb));
	fast_iq_slew_reset(&g_mb);
	g_iq_ref = 0;
	g_iq_ref_first = 0;
	g_tick = 0U;
}

/* ------------------------------------------------------------------ scenarios ------------- */

static void scenario_calm_flat(void)
{
	ride_t r;
	run_stats_t st;

	printf("S1 calm riding on the flat\n");
	reset_all(0);
	base_ride(&r);
	r.level = 2U;                /* TRAIL */
	r.peak_centikg = 500U;       /* a light, steady effort */
	run(&r, MS(4000), &st);

	CHECK(st.mean > 0, "S1: calm pedalling produces assist");
	CHECK(st.ticks_to_first_current > 0U && st.ticks_to_first_current < MS(500),
		"S1: assist appears within half a second of starting - no delay to mask a hard start");

	/* Re-run, this time measuring only the settled ride. */
	reset_all(0);
	run_after(&r, MS(2000), MS(4000), &st);
	/*
	 * THE CENTRAL PROPERTY. The pedal force falls to 15 % of peak in the dead spot every half
	 * revolution. The sustained term must not follow it down, because that collapse is exactly
	 * what makes a motor feel like it is pulsing.
	 */
	CHECK(st.base_min > 0, "S1: the sustained term never collapses to zero between strokes");
}

static void scenario_harder_push(void)
{
	ride_t r;
	run_stats_t calm;
	run_stats_t harder;

	printf("S2 a harder push gets more assist\n");
	reset_all(0);
	base_ride(&r);
	r.peak_centikg = 500U;
	run(&r, MS(3000), &calm);
	r.peak_centikg = 1400U;
	run(&r, MS(3000), &harder);

	CHECK(harder.mean > calm.mean,
		"S2: pushing harder produces more assist than pushing lightly");
}

static void scenario_aggression(void)
{
	ride_t r;
	assist_pipeline_command_t cmd;
	int32_t aggression_calm;
	int32_t aggression_sharp;
	uint32_t i;

	printf("S3 rider aggression shapes dynamics, not the whole request\n");
	reset_all(0);
	base_ride(&r);
	r.peak_centikg = 600U;
	for (i = 0; i < MS(3000); i++) {
		pipeline_tick(&r, &cmd);
	}
	aggression_calm = assist_pipeline_telemetry()->rider_aggression_permille;

	/* A sharp burst: the force jumps and the cadence winds up together. */
	r.peak_centikg = 1800U;
	r.cadence_rpm = 95U;
	for (i = 0; i < MS(600); i++) {
		pipeline_tick(&r, &cmd);
	}
	aggression_sharp = assist_pipeline_telemetry()->rider_aggression_permille;

	CHECK(aggression_sharp > aggression_calm,
		"S3: a sharp burst raises rider aggression above a steady effort");
	CHECK(assist_pipeline_telemetry()->attack_ms <=
		ap2_profile_base(AP2_PROFILE_SPORT)->attack_ms,
		"S3: aggression shortens the attack rather than multiplying the request");
}

static void scenario_load_estimator(void)
{
	ride_t r;
	assist_pipeline_command_t cmd;
	int32_t load_flat;
	int32_t load_climb;
	uint32_t i;

	printf("S4 load estimator separates a climb from a sprint\n");

	/* Flat: high cadence, speed keeps rising, moderate effort. */
	reset_all(0);
	base_ride(&r);
	r.cadence_rpm = 95U;
	r.peak_centikg = 900U;
	for (i = 0; i < MS(6000); i++) {
		r.speed_x100 = 1500U + i / 20U;   /* accelerating */
		pipeline_tick(&r, &cmd);
	}
	load_flat = assist_pipeline_telemetry()->load_state_permille;

	/* Climb: low cadence, high sustained effort, speed refuses to rise. */
	reset_all(0);
	base_ride(&r);
	r.cadence_rpm = 42U;
	r.peak_centikg = 2600U;
	r.speed_x100 = 700U;
	for (i = 0; i < MS(6000); i++) {
		pipeline_tick(&r, &cmd);
	}
	load_climb = assist_pipeline_telemetry()->load_state_permille;

	CHECK(load_climb > load_flat,
		"S4: a low-cadence effort that is not producing speed reads as more load than a sprint");
}

/*
 * Reverse is checked from several starting states, because the contract is about the
 * TRANSITION, not about one comfortable operating point. The audit's acceptance list: several
 * positive Iq values, during the start segment, under an adaptive profile, while a limiter is
 * binding, and after a resume.
 */
typedef struct {
	const char *what;
	uint8_t bank;
	uint8_t level;
	uint16_t peak_centikg;
	uint32_t settle_ticks;
	int32_t battery_current_ma;
	bool resume_first;
} reverse_case_t;

static void reverse_case(const reverse_case_t *c)
{
	ride_t r;
	assist_pipeline_command_t cmd;
	uint32_t i;
	int32_t iq_before;
	char label[160];

	reset_all(c->bank);
	base_ride(&r);
	r.level = c->level;
	r.peak_centikg = c->peak_centikg;
	r.battery_current_ma = c->battery_current_ma;

	for (i = 0; i < c->settle_ticks; i++) {
		pipeline_tick(&r, &cmd);
	}

	if (c->resume_first) {
		/* Stop briefly and pick the pedals up again, so the reverse lands on a resumed ride. */
		r.forward = false;
		for (i = 0; i < MS(150); i++) {
			pipeline_tick(&r, &cmd);
		}
		r.forward = true;
		for (i = 0; i < MS(800); i++) {
			pipeline_tick(&r, &cmd);
		}
	}

	iq_before = g_iq_ref;
	snprintf(label, sizeof(label), "S5[%s]: setup - a positive reference exists", c->what);
	CHECK(iq_before > 0, label);

	/* The reverse crank step. */
	r.reverse_step = true;
	pipeline_tick(&r, &cmd);

	snprintf(label, sizeof(label), "S5[%s]: the request is zero in the same tick", c->what);
	CHECK(cmd.final_iq_request == 0, label);

	snprintf(label, sizeof(label),
		"S5[%s]: the REFERENCE the current regulator sees is zero on the FIRST ISR tick - "
		"no 200 ms tail of positive reference after a reverse", c->what);
	CHECK(g_iq_ref_first == 0 && g_iq_ref == 0, label);

	snprintf(label, sizeof(label), "S5[%s]: the lifecycle names the reason", c->what);
	CHECK(assist_pipeline_pas_state() == AP2_PAS_REVERSE, label);

	/* And nothing creeps back while the reverse is held. */
	for (i = 0; i < MS(1500); i++) {
		pipeline_tick(&r, &cmd);
		if (cmd.final_iq_request != 0 || g_iq_ref != 0) {
			break;
		}
	}
	snprintf(label, sizeof(label),
		"S5[%s]: no hold, estimator or ramp re-raises request or reference while reversed",
		c->what);
	CHECK(cmd.final_iq_request == 0 && g_iq_ref == 0, label);
}

static void scenario_stop_and_reverse(void)
{
	ride_t r;
	assist_pipeline_command_t cmd;
	uint32_t i;

	printf("S5 stop, and reverse\n");

	/* --- ordinary stop: the request goes to zero and the release is a bounded ramp --- */
	reset_all(0);
	base_ride(&r);
	for (i = 0; i < MS(3000); i++) {
		pipeline_tick(&r, &cmd);
	}
	CHECK(cmd.final_iq_request > 0, "S5: setup - a ride is established");

	r.forward = false;
	pipeline_tick(&r, &cmd);
	CHECK(cmd.final_iq_request == 0,
		"S5: losing forward pedalling removes the REQUEST immediately");
	CHECK(cmd.slew_mode == FIS_MODE_RELEASE,
		"S5: and hands the CURRENT to a bounded release rather than cutting it");
	CHECK(cmd.zero_policy == FIS_ZERO_POLICY_QUIET,
		"S5: an ordinary end of pedalling is the case Quiet Zero exists for");
	CHECK(g_iq_ref > 0,
		"S5: a STOP is deliberately not a reverse - the current is still being retired, "
		"which is what keeps an ordinary release smooth");

	/* --- a non-direction safety cut keeps the bounded release ------------------------- */
	reset_all(0);
	base_ride(&r);
	for (i = 0; i < MS(3000); i++) {
		pipeline_tick(&r, &cmd);
	}
	r.safety_cut = true;
	pipeline_tick(&r, &cmd);
	CHECK(cmd.final_iq_request == 0 && cmd.slew_mode == FIS_MODE_SAFETY,
		"S5: a brake / overtemperature / torque fault zeroes the request and uses the "
		"firmware-owned safety release - it is a decision about the machine, not about direction");

	/* --- reverse: absolute, and it removes the REFERENCE, not just the request -------- */
	{
		static const reverse_case_t cases[] = {
			{ "steady ride",    0U, 3U,  900U, MS(3000), 4000,  false },
			{ "hard effort",    0U, 4U, 2600U, MS(3000), 4000,  false },
			{ "during start",   0U, 3U, 1400U, MS(120),  4000,  false },
			{ "adaptive AUTO",  1U, 3U, 1800U, MS(6000), 4000,  false },
			{ "battery limit",  0U, 4U, 2600U, MS(3000), 20000, false },
			{ "after a resume", 0U, 3U, 1200U, MS(3000), 4000,  true  },
		};
		unsigned n;
		for (n = 0; n < sizeof(cases) / sizeof(cases[0]); n++) {
			reverse_case(&cases[n]);
		}
	}
}

static void scenario_restart(void)
{
	ride_t r;
	assist_pipeline_command_t cmd;
	uint32_t i;
	uint32_t resume_ticks = 0U;

	printf("S6 restarting after a brief gap\n");
	reset_all(0);
	base_ride(&r);
	for (i = 0; i < MS(3000); i++) {
		pipeline_tick(&r, &cmd);
	}
	/* A gap shorter than the stop grace: one missed leg, not the end of the ride. */
	r.forward = false;
	for (i = 0; i < MS(200); i++) {
		pipeline_tick(&r, &cmd);
	}
	CHECK(assist_pipeline_pas_state() == AP2_PAS_STOPPING,
		"S6: a short gap is a STOPPING window, not a finished ride");

	r.forward = true;
	for (i = 0; i < MS(1000); i++) {
		pipeline_tick(&r, &cmd);
		if (cmd.final_iq_request > 0) {
			resume_ticks = i + 1U;
			break;
		}
	}
	CHECK(resume_ticks > 0U && resume_ticks < MS(400),
		"S6: resuming inside the grace re-engages promptly, without the cold start gate");
}

static void scenario_profiles(void)
{
	ride_t r;
	run_stats_t eco;
	run_stats_t sport;
	run_stats_t sport_plus;

	printf("S7 the profiles are ordered, and differ in more than strength\n");

	reset_all(0);
	base_ride(&r);
	r.level = 1U;   /* ECO */
	r.peak_centikg = 1100U;
	run(&r, MS(4000), &eco);

	reset_all(0);
	base_ride(&r);
	r.level = 3U;   /* SPORT */
	r.peak_centikg = 1100U;
	run(&r, MS(4000), &sport);

	reset_all(0);
	base_ride(&r);
	r.level = 4U;   /* SPORT+ */
	r.peak_centikg = 1100U;
	run(&r, MS(4000), &sport_plus);

	CHECK(eco.mean < sport.mean, "S7: SPORT gives more assist than ECO at the same effort");
	CHECK(sport.mean <= sport_plus.mean,
		"S7: SPORT+ gives at least as much assist as SPORT at the same effort");
	CHECK(ap2_profile_base(AP2_PROFILE_SPORT_PLUS)->attack_ms <
		ap2_profile_base(AP2_PROFILE_SPORT)->attack_ms,
		"S7: SPORT+ answers sooner than SPORT - it is a different behaviour, not a bigger number");
	CHECK(ap2_profile_base(AP2_PROFILE_SPORT_PLUS)->dynamic_gain_pct >
		ap2_profile_base(AP2_PROFILE_SPORT)->dynamic_gain_pct,
		"S7: SPORT+ answers a harder PUSH more, not just pulls harder overall");
	CHECK(ap2_profile_base(AP2_PROFILE_SPORT_PLUS)->base_hold_ms >
		ap2_profile_base(AP2_PROFILE_SPORT)->base_hold_ms,
		"S7: SPORT+ sustains longer under load instead of surging and sagging");
	CHECK(ap2_profile_base(AP2_PROFILE_ECO)->characteristic == AP2_CURVE_SOFT &&
		ap2_profile_base(AP2_PROFILE_SPORT_PLUS)->characteristic == AP2_CURVE_EAGER,
		"S7: the two ends of the range use different characteristics, not one curve scaled");
}

static void scenario_sport_plus_is_not_instant_max(void)
{
	ride_t r;
	assist_pipeline_command_t cmd;
	uint32_t i;
	int32_t first_positive = 0;

	printf("S8 SPORT+ is aggressive but still ramped\n");
	reset_all(0);
	base_ride(&r);
	r.level = 4U;            /* SPORT+ */
	r.peak_centikg = 3000U;  /* a very hard first push */
	for (i = 0; i < MS(2000); i++) {
		pipeline_tick(&r, &cmd);
		if (cmd.final_iq_request > 0 && first_positive == 0) {
			first_positive = cmd.final_iq_request;
		}
	}
	CHECK(first_positive > 0, "S8: setup - SPORT+ engages");
	CHECK(cmd.slew_mode == FIS_MODE_RISE || cmd.slew_mode == FIS_MODE_HOLD ||
		cmd.slew_mode == FIS_MODE_FALL,
		"S8: the current still travels on the one bounded trajectory");
	CHECK(ap2_profile_base(AP2_PROFILE_SPORT_PLUS)->attack_ms >= 20U,
		"S8: SPORT+ has a real attack time - it is not a step to maximum current");
}

static void scenario_auto(void)
{
	ride_t r;
	assist_pipeline_command_t cmd;
	uint32_t i;
	int32_t factor_calm;
	int32_t factor_worked;

	printf("S9 AUTO moves continuously with how the bike is ridden\n");
	reset_all(1);   /* the adaptive bank */
	base_ride(&r);
	r.level = 3U;   /* AUTO */
	r.peak_centikg = 400U;
	r.cadence_rpm = 80U;
	for (i = 0; i < MS(6000); i++) {
		pipeline_tick(&r, &cmd);
	}
	factor_calm = assist_pipeline_telemetry()->auto_factor_permille;
	CHECK(assist_pipeline_telemetry()->profile_id == AP2_PROFILE_AUTO,
		"S9: setup - the adaptive bank selects AUTO");

	r.peak_centikg = 2400U;
	r.cadence_rpm = 45U;
	r.speed_x100 = 800U;
	for (i = 0; i < MS(8000); i++) {
		pipeline_tick(&r, &cmd);
	}
	factor_worked = assist_pipeline_telemetry()->auto_factor_permille;

	CHECK(factor_worked > factor_calm,
		"S9: working the bike harder moves AUTO toward its stronger endpoint");
	CHECK(factor_calm >= 0 && factor_worked <= 1000,
		"S9: the AUTO decision stays a bounded, continuous quantity");
}

static void scenario_auto_sport_plus(void)
{
	ride_t r;
	assist_pipeline_command_t cmd;
	uint32_t i;

	printf("S10 AUTO SPORT+ reaches a stronger envelope than AUTO\n");

	/* Both adaptive profiles, driven identically into their strong end. */
	reset_all(1);
	base_ride(&r);
	r.level = 3U;   /* AUTO */
	r.peak_centikg = 2600U;
	r.cadence_rpm = 45U;
	r.speed_x100 = 800U;
	for (i = 0; i < MS(10000); i++) {
		pipeline_tick(&r, &cmd);
	}
	uint16_t auto_power = assist_pipeline_telemetry()->max_power_w;
	uint16_t auto_gain = assist_pipeline_telemetry()->assist_gain_pct;

	reset_all(1);
	base_ride(&r);
	r.level = 4U;   /* AUTO SPORT+ */
	r.peak_centikg = 2600U;
	r.cadence_rpm = 45U;
	r.speed_x100 = 800U;
	for (i = 0; i < MS(10000); i++) {
		pipeline_tick(&r, &cmd);
	}
	CHECK(assist_pipeline_telemetry()->profile_id == AP2_PROFILE_AUTO_SPORT_PLUS,
		"S10: setup - the adaptive bank selects AUTO SPORT+");
	CHECK(assist_pipeline_telemetry()->max_power_w > auto_power,
		"S10: AUTO SPORT+ offers a larger power reserve than AUTO at the same riding");
	CHECK(assist_pipeline_telemetry()->assist_gain_pct > auto_gain,
		"S10: ...and a stronger characteristic, from the same one decision logic");
}

static void scenario_limits(void)
{
	ride_t r;
	assist_pipeline_command_t cmd;
	uint32_t i;
	int32_t iq_normal;
	int32_t iq_limited;
	int32_t iq_recovered;

	printf("S11 the limiter chain, entering and leaving\n");

	/* --- battery current --- */
	reset_all(0);
	base_ride(&r);
	r.peak_centikg = 2200U;
	for (i = 0; i < MS(3000); i++) {
		pipeline_tick(&r, &cmd);
	}
	iq_normal = cmd.final_iq_request;

	r.battery_current_ma = 20000;   /* over the 15 A configured ceiling */
	for (i = 0; i < MS(1000); i++) {
		pipeline_tick(&r, &cmd);
	}
	iq_limited = cmd.final_iq_request;
	CHECK(assist_pipeline_battery_limited(),
		"S11: the battery limiter latches when the measured current exceeds the ceiling");
	CHECK(iq_limited < iq_normal, "S11: and it actually takes current away");
	CHECK(assist_pipeline_telemetry()->battery_limited,
		"S11: the telemetry names which stage was binding");

	r.battery_current_ma = 4000;    /* well under the exit hysteresis */
	for (i = 0; i < MS(2000); i++) {
		pipeline_tick(&r, &cmd);
	}
	iq_recovered = cmd.final_iq_request;
	CHECK(!assist_pipeline_battery_limited(), "S11: the limiter releases with hysteresis");
	CHECK(iq_recovered > iq_limited, "S11: and the assist comes back");
	CHECK(cmd.slew_mode != FIS_MODE_FORCE_ZERO,
		"S11: neither entering nor leaving the limit steps the current - it is a ramp");

	/* --- thermal --- */
	reset_all(0);
	base_ride(&r);
	r.peak_centikg = 2200U;
	for (i = 0; i < MS(3000); i++) {
		pipeline_tick(&r, &cmd);
	}
	iq_normal = cmd.final_iq_request;
	r.temperature_c = 85;   /* inside the 75..90 derate band */
	for (i = 0; i < MS(1000); i++) {
		pipeline_tick(&r, &cmd);
	}
	CHECK(cmd.final_iq_request < iq_normal && assist_pipeline_telemetry()->thermal_limited,
		"S11: controller temperature derates continuously inside its band");

	/* --- legal speed taper --- */
	reset_all(0);
	base_ride(&r);
	r.legal = true;
	r.peak_centikg = 2200U;
	r.speed_x100 = 1800U;
	for (i = 0; i < MS(3000); i++) {
		pipeline_tick(&r, &cmd);
	}
	iq_normal = cmd.final_iq_request;
	r.speed_x100 = 2700U;   /* above the 25 km/h limit */
	for (i = 0; i < MS(1000); i++) {
		pipeline_tick(&r, &cmd);
	}
	CHECK(cmd.final_iq_request < iq_normal && assist_pipeline_telemetry()->speed_limited,
		"S11: the legal taper is applied above the configured limit");
}

static void scenario_safety_and_level_zero(void)
{
	ride_t r;
	assist_pipeline_command_t cmd;
	uint32_t i;

	printf("S12 safety cut and assist level 0\n");

	reset_all(0);
	base_ride(&r);
	for (i = 0; i < MS(3000); i++) {
		pipeline_tick(&r, &cmd);
	}
	CHECK(cmd.final_iq_request > 0, "S12: setup - a ride is established");
	r.safety_cut = true;
	pipeline_tick(&r, &cmd);
	CHECK(cmd.final_iq_request == 0 && cmd.slew_mode == FIS_MODE_SAFETY,
		"S12: a brake / overtemperature / torque fault zeroes the request in the same tick");

	reset_all(0);
	base_ride(&r);
	r.level = 0U;
	run_stats_t st;
	run(&r, MS(3000), &st);
	CHECK(st.max == 0, "S12: assist level 0 produces no current at all, however hard the push");
}

static void scenario_limiter_state_survives_owner_change(void)
{
	ride_t r;
	assist_pipeline_command_t cmd;
	uint32_t i;
	bool latched_before;
	bool latched_after_reset;

	printf("S14 a limiter latch belongs to the battery, not to the ride\n");

	/*
	 * AUDIT FINDING 2. Walk Assist resets the pipeline so a ride cannot survive the detour.
	 * That reset used to clear the shared limiter chain too, which meant the battery limiter
	 * forgot it was limiting - on every tick of a Walk. A pack sitting between the exit
	 * hysteresis band and the entry threshold therefore saw the cap jump back to full scale
	 * thousands of times a second.
	 *
	 * The limiter describes the BATTERY. Nothing the rider does with the pedals, or with the
	 * Walk button, changes how much current the pack may deliver.
	 */
	reset_all(0);
	base_ride(&r);
	r.peak_centikg = 2400U;
	r.battery_current_ma = 20000;          /* over the 15 A ceiling */
	for (i = 0; i < MS(2000); i++) {
		pipeline_tick(&r, &cmd);
	}
	latched_before = assist_pipeline_battery_limited();
	CHECK(latched_before, "S14: setup - the battery limiter has latched");

	/* Inside the hysteresis band: above the exit threshold, below the entry threshold. */
	r.battery_current_ma = 14000;
	for (i = 0; i < MS(200); i++) {
		pipeline_tick(&r, &cmd);
	}
	CHECK(assist_pipeline_battery_limited(),
		"S14: setup - inside the hysteresis band the latch is still held");

	/* An owner change - Walk, calibration, anything - resets the pedalling lifecycle. */
	assist_pipeline_reset();
	latched_after_reset = assist_pipeline_battery_limited();
	CHECK(latched_after_reset,
		"S14: resetting the pedalling lifecycle does NOT clear the battery limiter latch");

	/* And it still releases properly, on its own terms. */
	reset_all(0);
	base_ride(&r);
	r.peak_centikg = 2400U;
	r.battery_current_ma = 20000;
	for (i = 0; i < MS(2000); i++) {
		pipeline_tick(&r, &cmd);
	}
	r.battery_current_ma = 8000;           /* below the 90 % exit band */
	for (i = 0; i < MS(500); i++) {
		pipeline_tick(&r, &cmd);
	}
	CHECK(!assist_pipeline_battery_limited(),
		"S14: the latch still releases when the measured current genuinely falls");
}

static void scenario_level_iq_ceiling(void)
{
	const assist_level_config_t *level;
	ap2_profile_override_t ovr;
	assist_level_config_t legacy;
	int32_t full;
	int32_t half;
	int32_t fifth;
	int32_t zero_pct;

	printf("S15 the configured per-level Iq ceiling binds\n");

	/*
	 * AUDIT FINDING 3. max_iq_pct is a rider-visible per-level ceiling that has been stored and
	 * round-tripped over CAN for a long time. It reached no control path at all: the pipeline
	 * was handed the GLOBAL ceiling, so setting a level to 20 % changed nothing.
	 */
	assist_modes_init();
	assist_modes_set_active_bank(0);
	level = assist_modes_get_default_level(3U);

	{
		assist_level_config_t cfg = *level;
		cfg.max_iq_pct = 100U;
		full = assist_modes_level_iq_limit(&cfg, (int32_t)PH_CURRENT_MAX, (int32_t)PH_CURRENT_MAX);
		cfg.max_iq_pct = 50U;
		half = assist_modes_level_iq_limit(&cfg, (int32_t)PH_CURRENT_MAX, (int32_t)PH_CURRENT_MAX);
		cfg.max_iq_pct = 20U;
		fifth = assist_modes_level_iq_limit(&cfg, (int32_t)PH_CURRENT_MAX, (int32_t)PH_CURRENT_MAX);
		cfg.max_iq_pct = 0U;
		zero_pct = assist_modes_level_iq_limit(&cfg, (int32_t)PH_CURRENT_MAX, (int32_t)PH_CURRENT_MAX);
	}

	CHECK(full == (int32_t)PH_CURRENT_MAX, "S15: 100 % is the full ceiling");
	CHECK(half == (int32_t)PH_CURRENT_MAX / 2, "S15: 50 % halves it");
	CHECK(fifth == (int32_t)PH_CURRENT_MAX / 5, "S15: 20 % is a fifth of it");
	CHECK(half < full && fifth < half, "S15: the ceilings are ordered");
	CHECK(zero_pct == (int32_t)PH_CURRENT_MAX,
		"S15: 0 %% means NO LEVEL CEILING - it is what an uninitialised or pre-v6 record "
		"carries, and reading it as zero allowed current would disable assist on an old bank");

	/* It can only tighten: a global limp-mode limit still wins when it is lower. */
	{
		assist_level_config_t cfg = *level;
		cfg.max_iq_pct = 100U;
		CHECK(assist_modes_level_iq_limit(&cfg, 200, (int32_t)PH_CURRENT_MAX) == 200,
			"S15: the level ceiling never raises a lower global limit");
	}

	/*
	 * ...and a migration must not RAISE a stored restriction. A level saved under a legacy
	 * mode number keeps its configured power ceiling: watts did not change meaning with the
	 * pipeline, and a firmware update that quietly lifted a rider's limit would be the one
	 * direction a migration must never move one.
	 */
	legacy = *level;
	legacy.mode_type = ASSIST_MODE_POWER_LINEAR;    /* a pre-V2 bank */
	legacy.max_motor_power_w = 300U;
	assist_modes_profile_override(&legacy, &ovr);
	CHECK(ovr.max_power_w == 300U,
		"S15: migrating a legacy level keeps its stored power ceiling");
	CHECK(ovr.assist_trim_pct == 0U && ovr.attack_ms == 0U,
		"S15: ...but does NOT carry its gain or dynamics across, because those numbers meant "
		"something else in the removed request model");
}

static void scenario_ceiling_binds_the_reference(void)
{
	ride_t r;
	assist_pipeline_command_t cmd;
	uint32_t i;
	int32_t ref_high;
	int32_t ref_after;
	uint32_t ticks_to_comply = 0U;

	printf("S16 a limiter binds the reference, not only the request\n");

	/*
	 * AUDIT FINDING 4. The chain capped the TARGET, and the target is approached over the
	 * rider-feel release time. With a 600 ms release, a limiter that started binding left the
	 * current regulator holding a reference above the new cap for most of a second. Capping a
	 * request is not the same thing as limiting a current.
	 */
	reset_all(0);
	base_ride(&r);
	r.level = 4U;                  /* SPORT+: the largest envelope, so the drop is visible */
	r.peak_centikg = 2800U;
	for (i = 0; i < MS(3000); i++) {
		pipeline_tick(&r, &cmd);
	}
	ref_high = g_iq_ref;
	CHECK(ref_high > 100, "S16: setup - a high reference is established");

	/* A hard thermal derate: the protections now allow far less than the rider is asking. */
	r.temperature_c = 88;
	for (i = 0; i < MS(1000); i++) {
		pipeline_tick(&r, &cmd);
		if (ticks_to_comply == 0U && g_iq_ref <= cmd.iq_ceiling) {
			ticks_to_comply = i + 1U;
		}
	}
	ref_after = g_iq_ref;

	CHECK(cmd.iq_ceiling < ref_high,
		"S16: setup - the protections now allow less than the reference was");
	CHECK(ref_after <= cmd.iq_ceiling,
		"S16: the reference ends up at or below what the protections allow");
	CHECK(ticks_to_comply > 0U && ticks_to_comply < MS(250),
		"S16: and it gets there without waiting for a rider-feel release time");

	/* Recovery is a ramp, not a jump: the ceiling opens gradually and the attack does the rest. */
	{
		int32_t prev = g_iq_ref;
		int32_t worst_step = 0;
		r.temperature_c = 30;
		for (i = 0; i < MS(2000); i++) {
			pipeline_tick(&r, &cmd);
			if (g_iq_ref - prev > worst_step) {
				worst_step = g_iq_ref - prev;
			}
			prev = g_iq_ref;
		}
		CHECK(g_iq_ref > ref_after, "S16: leaving the limit gives the assist back");
		CHECK(worst_step <= 8,
			"S16: ...and gives it back as a ramp - no step in the reference when a limit lifts");
	}
}

static void scenario_no_torque_no_assist(void)
{
	ride_t r;
	run_stats_t st;

	printf("S13 no rider effort means no assist\n");
	reset_all(0);
	base_ride(&r);
	r.peak_centikg = 0U;   /* cranks turning, no force on the pedal */
	run(&r, MS(4000), &st);
	CHECK(st.max == 0,
		"S13: a freewheeling crank with no pedal force never produces current");
}

int main(void)
{
	puts("Assist Pipeline V2 behavioural scenarios");
	scenario_calm_flat();
	scenario_harder_push();
	scenario_aggression();
	scenario_load_estimator();
	scenario_stop_and_reverse();
	scenario_restart();
	scenario_profiles();
	scenario_sport_plus_is_not_instant_max();
	scenario_auto();
	scenario_auto_sport_plus();
	scenario_limits();
	scenario_safety_and_level_zero();
	scenario_no_torque_no_assist();
	scenario_limiter_state_survives_owner_change();
	scenario_level_iq_ceiling();
	scenario_ceiling_binds_the_reference();

	if (failures == 0) {
		puts("Assist Pipeline V2 scenarios: ALL CHECKS PASSED");
		return 0;
	}
	printf("Assist Pipeline V2 scenarios: %d CHECK(S) FAILED\n", failures);
	return 1;
}
