/*
 * Assist Behavior V3 - transient manager and THE trajectory, module level (ARCHITECTURE_V3.md
 * 5, 6.1, 6.2; Milestone C rules, shadow integration of Milestone B). Real modules:
 * src/assist_v3.c, src/assist_v3_intent.c, src/assist_motion.c and the G53 chain configuration
 * (src/g53_port_chain.c, read only through g53_static_target() and its accessors).
 *
 * The pipeline-level G1-* gates (G1-SAFE, G1-BACKSTOP, G1-STAND, G1-SEL, G1-VETO, ...) need the
 * activated engine and belong to Milestone C. This suite pins what the module itself promises:
 *
 *  T1  E2 -> Iq conversion: target_E2 * 0.65 * P / 40960 (exact, P clamped).
 *  T2  R(Response): 0 % -> 600 ms, 100 % -> 150 ms full scale, linear, saturating.
 *  T3  power-on: y = 0, output 0, stop target zero with no load.
 *  T4  start gate (6.2, R1-#14): heavy load + pedalling with EB74 NOT armed -> never engages;
 *      armed -> engages.
 *  T5  rise (6.1): never faster than min(level D7EC rise, BDE8 50/ms) and far below the
 *      6.84 Iq/ms electrical guard (G1-TRAJ).
 *  T6  release while pedalling: falls at R(Response) of the level, both ends of the Response range.
 *  T7  PEDAL_STOP, load released: BDE8 3.5 Iq/ms while only V3's own PEDAL_STOP sees it; y = 0 at
 *      once when the stop is confirmed (G53 true-stop / real_stop), as baseline BDE8 zeroes Q5C.
 *  T8  PEDAL_STOP, load held: holds until the stop is confirmed (G53 true-stop / real_stop),
 *      then the legacy D3E 0.455 Iq/ms ramp; the stop-target-zero term stays false until the
 *      ramp reaches 0 (re-check N2) and is true at once for a released load.
 *  T9  reverse: y is 0 on the first reverse call whatever the load (Milestone C: legacy BDE8 zero,
 *      measured G5300 reverse = request 0 within 3 logical ms; the fast slew shapes the published Iq).
 *  T10 output bounds: 0 <= demand <= 0.65 P at all times, integer Iq = y >> 8.
 *  T11 elapsed time: the same scenario at 1 tick/call and at 4 ticks/call ends within 1 Iq.
 *  T12 motion seam: garbage in an invalid/stale IMU sample (sanitised) -> bit-identical output
 *      and telemetry (G1-IMU, module part).
 *  T13 reset: y -> 0, the learned template is kept (intent's contract).
 */
#include "assist_v3.h"
#include "assist_v3_intent.h"
#include "assist_motion.h"
#include "g53_port_chain.h"
#include "common/check.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define P 700
#define TICKS_PER_MIN 240000u
#define GUARD_Q8_PER_TICK ((P * 5 + 4) / 8)   /* fast_iq_slew step: 0.625 P Q8 per tick */

typedef struct {
	uint32_t tick;
	int32_t steps;
	uint32_t step_tick;
	uint32_t acc;
	uint16_t rpm;          /* 0 = crank stopped */
	int8_t dir;            /* +1 forward, -1 reverse */
	uint16_t load;
	uint8_t level;
	uint8_t response;
	bool armed;
	bool true_stop;
	bool real_stop;
	uint32_t el;           /* ticks per call */
	motion_input_t raw_motion;
	uint32_t speed_est, distance_est;
	int16_t rel_accel;
	uint8_t motion_quality, carry_strength;
	uint16_t carry_time_ms, carry_distance_dm;
	int32_t iq_measured;
	bool brake, native_cut;
} rig_t;

static void rig_init(rig_t *r)
{
	memset(r, 0, sizeof(*r));
	r->tick = 1000u;
	r->dir = 1;
	r->level = 3u;
	r->response = 80u;
	r->armed = true;
	r->el = 1u;
}

/* One call: advance the crank by `el` ticks and run assist_v3_update(). */
static int32_t rig_step(rig_t *r)
{
	assist_v3_input_t in;
	for (uint32_t k = 0u; k < r->el; k++) {
		r->tick++;
		r->acc += (uint32_t)r->rpm * ASSIST_V3_STEPS_PER_REV;
		while (r->acc >= TICKS_PER_MIN) {
			r->acc -= TICKS_PER_MIN;
			r->steps += r->dir;
			r->step_tick = r->tick;
		}
	}
	memset(&in, 0, sizeof(in));
	in.elapsed_ticks = r->el;
	in.now_tick = r->tick;
	in.load_ctrl = r->load;
	in.torque_valid = true;
	in.crank_steps = r->steps;
	in.crank_step_tick = r->step_tick;
	in.cadence_rpm = (int16_t)(r->dir < 0 ? -(int16_t)r->rpm : (int16_t)r->rpm);
	in.g53_true_stop = r->true_stop;
	in.real_stop = r->real_stop;
	in.direction_inhibit = r->dir < 0 && r->rpm > 0u;
	in.inhibit_is_reverse = in.direction_inhibit;
	in.speed_x100 = 1500u;
	in.wheel_valid = true;
	in.phase_current_max = P;
	in.g1_q12 = 4096u;
	in.level = r->level;
	in.response_pct = r->response;
	in.speed_est_x100 = r->speed_est;
	in.distance_est_mm = r->distance_est;
	in.rel_accel_permille_s = r->rel_accel;
	in.motion_quality = r->motion_quality;
	in.carry_strength_pct = r->carry_strength;
	in.carry_time_ms = r->carry_time_ms;
	in.carry_distance_dm = r->carry_distance_dm;
	in.iq_measured = r->iq_measured;
	in.brake = r->brake;
	in.native_cut = r->native_cut;
	in.eb74_zero = 750u;
	in.eb74_armed = r->armed;
	assist_motion_sanitize(&r->raw_motion, &in.motion);
	return assist_v3_update(&in);
}

static void fresh(rig_t *r)
{
	g53_chain_reset();
	assist_v3_power_on();
	rig_init(r);
}

/* Pedal until the demand settles (no change for 1.5 s: longer than one revolution at the rig
 * cadences, so the measured-ring kL of the rework is in force) or `max_ms` passes. */
static int32_t settle(rig_t *r, uint32_t max_ms)
{
	int32_t y = 0, last = -1;
	uint32_t still = 0u;
	for (uint32_t t = 0u; t < max_ms * 4u; t++) {
		y = rig_step(r);
		if (y == last) { if (++still > 6000u) break; } else { still = 0u; last = y; }
	}
	return y;
}

/* Largest per-ms change of the output over `ms` milliseconds, signed direction `sgn`. */
static int32_t max_change_per_window(rig_t *r, uint32_t ms, uint32_t window_ms, int sgn,
                                     int32_t *end_value)
{
	int32_t hist[4096];
	uint32_t n = ms * 4u;
	if (n > 4096u) n = 4096u;
	for (uint32_t t = 0u; t < n; t++) hist[t] = rig_step(r);
	int32_t worst = 0;
	const uint32_t w = window_ms * 4u;
	for (uint32_t t = w; t < n; t++) {
		const int32_t d = sgn * (hist[t] - hist[t - w]);
		if (d > worst) worst = d;
	}
	if (end_value) *end_value = hist[n - 1u];
	return worst;
}

static void t1_conversion(void)
{
	CHECK(assist_v3_e2_to_iq_q8(40960u, P) == 116480u, "T1a full-scale E2 = 0.65 P Iq (Q8 116480 at P 700)");
	CHECK(assist_v3_e2_to_iq_q8(20480u, P) == 58240u, "T1b half scale");
	CHECK(assist_v3_e2_to_iq_q8(65535u, P) == 116480u, "T1c E2 above full scale is clamped");
	CHECK(assist_v3_e2_to_iq_q8(40960u, -5) == 0u, "T1d negative P -> 0");
	CHECK(assist_v3_e2_to_iq_q8(40960u, 100000) == assist_v3_e2_to_iq_q8(40960u, 8000),
	      "T1e P clamped at 8000 (32-bit product)");
}

static void t2_response(void)
{
	CHECK(assist_v3_response_full_scale_ms(0u) == 600u, "T2a Response 0 % -> 600 ms");
	CHECK(assist_v3_response_full_scale_ms(100u) == 150u, "T2b Response 100 % -> 150 ms");
	CHECK(assist_v3_response_full_scale_ms(50u) == 375u, "T2c linear midpoint");
	CHECK(assist_v3_response_full_scale_ms(250u) == 150u, "T2d above 100 saturates");
}

static void t3_power_on(void)
{
	rig_t r;
	fresh(&r);
	CHECK(rig_step(&r) == 0, "T3a power-on output 0");
	CHECK(assist_v3_stop_target_zero(), "T3b no load -> stop target zero");
	CHECK(!assist_v3_engaged(), "T3c not engaged");
}

static void t4_start_gate(void)
{
	rig_t r;
	fresh(&r);
	r.armed = false; r.rpm = 60u; r.load = 3000u;
	int32_t y = settle(&r, 3000u);
	CHECK(y == 0 && assist_v3_telemetry()->rate_mode == ASSIST_V3_RATE_START_BLOCKED,
	      "T4a EB74 not armed: heavy pedalling never engages (R1-#14)");
	r.armed = true;
	y = settle(&r, 3000u);
	CHECK(y > 50, "T4b armed: engages and assists");
	CHECK(assist_v3_engaged(), "T4c engaged flag follows y > 0");
}

static void t5_rise(void)
{
	rig_t r;
	fresh(&r);
	r.rpm = 60u; r.load = 2000u;
	(void)settle(&r, 4000u);
	/* attack: load 2000 -> 5000; the static map target jumps, y rises at the rise rate */
	r.load = 5000u;
	int32_t end = 0;
	const int32_t rise_100ms = max_change_per_window(&r, 2000u, 100u, +1, &end);
	const uint8_t slot = g53_chain_level(3u);
	const uint32_t d232 = g53_chain_d7ec_rise(slot);
	/* level rate: d232 E2 per 10 ms -> Iq per 100 ms = d232 * 0.65 P / 40960 * 10 */
	const int32_t lvl_100ms = (int32_t)((d232 * 650u * P / 1000u * 10u) / 40960u);
	const int32_t bde8_100ms = P * 50 * 100 / 10000;
	const int32_t cap = (lvl_100ms < bde8_100ms ? lvl_100ms : bde8_100ms) + 1;
	printf("    T5 rise: slot %u D+232 %u, worst %d Iq/100 ms, level %d, BDE8 %d, end %d\n",
	       slot, (unsigned)d232, rise_100ms, lvl_100ms, bde8_100ms, end);
	CHECK(d232 > 0u, "T5a level rise rate configured");
	CHECK(rise_100ms > 0 && rise_100ms <= cap, "T5b rise <= min(level D7EC rise, BDE8 50/ms)");
	CHECK(rise_100ms * 256 / 400 < GUARD_Q8_PER_TICK, "T5c far below the 6.84 Iq/ms electrical guard");
}

static int32_t release_rate_100ms(uint8_t response)
{
	rig_t r;
	fresh(&r);
	r.response = response; r.rpm = 60u; r.load = 4000u;
	(void)settle(&r, 5000u);
	r.load = 0u;   /* TRUE_RELEASE while pedalling */
	int32_t end = 0;
	/* rework: with the measured flat stroke (kL = 1.0) the demand is no longer saturated, so it
	 * already follows the falling intent before TRUE_RELEASE fires; the Response rate is measured
	 * over 20 ms windows (x5 = per 100 ms) inside the release fall. */
	const int32_t worst = max_change_per_window(&r, 1500u, 20u, -1, &end);
	CHECK(end == 0, "T6 release while pedalling reaches 0");
	return worst * 5;
}

static void t6_release(void)
{
	const int32_t r100 = release_rate_100ms(100u);
	const int32_t r0 = release_rate_100ms(0u);
	const int32_t exp100 = (int32_t)((650 * P / 1000) * 100 / 150);   /* 0.65 P per 150 ms */
	const int32_t exp0 = (int32_t)((650 * P / 1000) * 100 / 600);     /* 0.65 P per 600 ms */
	printf("    T6 release: Response 100 %% %d Iq/100 ms (expect %d), 0 %% %d (expect %d)\n",
	       r100, exp100, r0, exp0);
	CHECK(r100 <= exp100 + 5 && r100 >= exp100 - 10, "T6a Response 100 %: full scale in 150 ms");
	CHECK(r0 <= exp0 + 5 && r0 >= exp0 - 10, "T6b Response 0 %: full scale in 600 ms");
}

static void t7_stop_released(void)
{
	rig_t r;
	fresh(&r);
	r.rpm = 60u; r.load = 4000u;
	const int32_t y0 = settle(&r, 5000u);
	r.rpm = 0u; r.load = 0u;   /* crank stops, foot off */
	int32_t end = 0;
	const int32_t worst = max_change_per_window(&r, 600u, 20u, -1, &end);
	const int32_t bde8_20ms = P * 50 * 20 / 10000;
	printf("    T7 stop released: from %d, worst %d Iq/20 ms (BDE8 %d), end %d\n", y0, worst, bde8_20ms, end);
	CHECK(end == 0, "T7a reaches 0");
	CHECK(worst <= bde8_20ms + 1 && worst >= bde8_20ms - 2, "T7b falls at the BDE8 rate (3.5 Iq/ms)");
	CHECK(assist_v3_stop_target_zero(), "T7c released load: stop target zero");
	/* T7d stop CONFIRMED (G53 true-stop) with the load released: y zeroed at once (baseline BDE8
	 * zeroes Q5C within ~12 ms of the true-stop; Milestone C stop <= baseline) */
	fresh(&r);
	r.rpm = 60u; r.load = 4000u;
	(void)settle(&r, 5000u);
	r.rpm = 0u; r.load = 0u; r.true_stop = true;
	CHECK(rig_step(&r) == 0 && assist_v3_telemetry()->rate_mode == ASSIST_V3_RATE_STOP_RELEASED,
	      "T7d confirmed stop, load released: y = 0 on that call");
}

static void t8_stop_held(void)
{
	rig_t r;
	fresh(&r);
	/* rework: a flat 4000 CLU now gives the measured kL 1.0 and an unsaturated demand, and this rig
	 * drops the cadence to 0 at once (the static map then saturates before PEDAL_STOP fires); a
	 * heavier flat load keeps the demand at the 0.65 P cap so T8b tests the hold, not that artefact */
	r.rpm = 60u; r.load = 9000u;
	const int32_t y0 = settle(&r, 5000u);
	r.rpm = 0u;   /* crank stops, load HELD */
	int32_t y = 0;
	for (int t = 0; t < 4 * 300; t++) y = rig_step(&r);   /* 300 ms: past T_stop, not confirmed */
	CHECK(assist_v3_telemetry()->rate_mode == ASSIST_V3_RATE_STOP_WAIT, "T8a stopped, unconfirmed: STOP_WAIT");
	CHECK(y == y0, "T8b holds until the stop is confirmed");
	CHECK(!assist_v3_stop_target_zero(), "T8c held load: stop target NOT zero (re-check N2)");
	r.true_stop = true;   /* G53 true-stop confirms */
	int32_t end = 0;
	const int32_t worst = max_change_per_window(&r, 800u, 100u, -1, &end);
	const int32_t d3e_100ms = (int32_t)((409u * 650u * P / 1000u * 10u) / 40960u);
	printf("    T8 stop held: from %d, worst %d Iq/100 ms (D3E %d), after 0.8 s %d\n", y0, worst, d3e_100ms, end);
	CHECK(worst <= d3e_100ms + 1 && worst >= d3e_100ms - 1, "T8d confirmed: legacy D3E 0.455 Iq/ms ramp");
	CHECK(end > 0 && !assist_v3_stop_target_zero(), "T8e mid-ramp: stop target NOT zero yet");
	{
		int t = 0;
		/* y is Q8 inside: the integer output reads 0 up to 1 Iq before the state is exactly 0 */
		for (t = 0; t < 4 * 2000 && !assist_v3_stop_target_zero(); t++) y = rig_step(&r);
		printf("    T8 ramp end after %d more ms\n", t / 4);
		CHECK(y == 0 && assist_v3_stop_target_zero() && t < 4 * 300,
		      "T8f ramp reaches exact 0 and then the stop target is zero");
	}
}

static void t9_reverse(void)
{
	rig_t r;
	fresh(&r);
	r.rpm = 60u; r.load = 4000u;
	const int32_t y0 = settle(&r, 5000u);
	r.dir = -1; r.rpm = 30u;   /* back-pedal with the foot still loaded */
	const int32_t first = rig_step(&r);
	int32_t end = 0;
	(void)max_change_per_window(&r, 400u, 20u, -1, &end);
	printf("    T9 reverse: from %d, first reverse call %d, end %d\n", y0, first, end);
	CHECK(y0 > 100 && first == 0, "T9a reverse zeroes y on the first reverse call (legacy BDE8 zero)");
	CHECK(end == 0 && assist_v3_telemetry()->rate_mode == ASSIST_V3_RATE_REVERSE,
	      "T9b stays 0 while reversing, whatever the load");
}

static void t10_bounds(void)
{
	rig_t r;
	int bad = 0;
	fresh(&r);
	uint32_t x = 0x1234567u;
	for (int t = 0; t < 4 * 20000; t++) {
		if ((t % 1000) == 0) {
			x ^= x << 13; x ^= x >> 17; x ^= x << 5;
			r.rpm = (uint16_t)(x % 140u);
			r.load = (uint16_t)((x >> 8) % 7000u);
			r.dir = (x & 0x100000u) ? -1 : 1;
			r.level = (uint8_t)((x >> 22) % 6u);
			r.true_stop = r.rpm == 0u && ((x >> 27) & 1u);
		}
		const int32_t y = rig_step(&r);
		if (y < 0 || y > 650 * P / 1000) bad++;
		if (assist_v3_telemetry()->v3_demand_iq != y) bad++;
	}
	CHECK(bad == 0, "T10 0 <= demand <= 0.65 P, telemetry mirrors the output");
}

static int32_t scenario_end(uint32_t el)
{
	rig_t r;
	fresh(&r);
	r.el = el; r.rpm = 70u; r.load = 3500u; r.response = 0u;   /* slowest fall: 600 ms */
	for (uint32_t t = 0u; t < 12000u / el; t++) (void)rig_step(&r);
	r.load = 0u;
	int32_t y = 0;
	/* rework: the unsaturated demand follows the falling intent, then the 600 ms Response fall:
	 * 300 ms after the release the demand is mid-ramp */
	for (uint32_t t = 0u; t < 1200u / el; t++) y = rig_step(&r);
	return y;
}

static void t11_elapsed(void)
{
	const int32_t a = scenario_end(1u), b = scenario_end(4u);
	printf("    T11 elapsed: 1 tick/call %d, 4 ticks/call %d\n", a, b);
	CHECK(a > 0 && a < 400 && (a - b <= 2 && b - a <= 2),
	      "T11 mid-ramp: rates follow elapsed time, not the call rate");
}

static void run_trace(uint32_t seed, int garbage, int32_t *out, assist_v3_telemetry_t *tl, int n)
{
	rig_t r;
	fresh(&r);
	uint32_t x = seed;
	for (int t = 0; t < n; t++) {
		if ((t % 800) == 0) {
			x ^= x << 13; x ^= x >> 17; x ^= x << 5;
			r.rpm = (uint16_t)(x % 120u); r.load = (uint16_t)((x >> 8) % 6000u);
		}
		if (garbage) {
			unsigned char *p = (unsigned char *)&r.raw_motion;
			for (size_t i = 0; i < sizeof(r.raw_motion); i++) p[i] = (unsigned char)(rand() & 0xFF);
			if (t & 1) memset(&r.raw_motion.valid, 0, 1);
			else { memset(&r.raw_motion.valid, 1, 1); r.raw_motion.age_ms = (uint16_t)(MOTION_MAX_AGE_MS + 1u); }
		} else {
			memset(&r.raw_motion, 0, sizeof(r.raw_motion));
		}
		out[t] = rig_step(&r);
		tl[t] = *assist_v3_telemetry();
	}
}

static void t12_motion(void)
{
	enum { N = 4 * 6000 };
	static int32_t a[N], b[N];
	static assist_v3_telemetry_t ta[N], tb[N];
	run_trace(0xBEEFu, 0, a, ta, N);
	srand(7);
	run_trace(0xBEEFu, 1, b, tb, N);
	CHECK(memcmp(a, b, sizeof(a)) == 0, "T12a invalid/stale IMU garbage: output bit-identical");
	CHECK(memcmp(ta, tb, sizeof(ta)) == 0, "T12b ...and telemetry bit-identical");
}

static void t13_reset(void)
{
	rig_t r;
	fresh(&r);
	r.rpm = 60u; r.load = 3000u;
	(void)settle(&r, 20000u);   /* long enough to learn revolutions */
	const uint16_t learned = assist_v3_intent_debug()->kl_recomputes;
	uint16_t tpl[ASSIST_V3_NB];
	memcpy(tpl, assist_v3_intent_template(), sizeof(tpl));
	assist_v3_reset();
	CHECK(!assist_v3_engaged() && assist_v3_telemetry()->v3_demand_iq >= 0, "T13a reset: trajectory at 0");
	CHECK(memcmp(tpl, assist_v3_intent_template(), sizeof(tpl)) == 0, "T13b reset keeps the learned template");
	(void)learned;
}

static void carry_prepare(rig_t *r, uint16_t time_ms, uint16_t distance_dm)
{
	fresh(r);
	r->rpm=65u; r->load=10000u; r->speed_est=850u; r->motion_quality=2u;
	r->iq_measured=350; r->carry_strength=70u;
	r->carry_time_ms=time_ms; r->carry_distance_dm=distance_dm;
	(void)settle(r,3000u);
	r->rpm=0u; r->load=0u; r->real_stop=true;
	(void)rig_step(r);
}

static void t14_carry(void)
{
	rig_t r;
	carry_prepare(&r,100u,15u);
	CHECK(assist_v3_telemetry()->carry_state==1u &&
	      assist_v3_telemetry()->carry_score_q12>=3000u,
	      "T14a loaded stop activates carry with a frozen score");
	for(unsigned i=0;i<450u;i++) (void)rig_step(&r);
	CHECK(assist_v3_telemetry()->carry_cancel_reason==4u,
	      "T14b time cap retires carry");
	carry_prepare(&r,1200u,1u);
	r.distance_est+=101u; (void)rig_step(&r);
	CHECK(assist_v3_telemetry()->carry_cancel_reason==5u,
	      "T14c distance cap retires carry first");
	carry_prepare(&r,1200u,15u);
	r.brake=true; r.native_cut=true; (void)rig_step(&r);
	CHECK(assist_v3_telemetry()->carry_state!=1u &&
	      assist_v3_telemetry()->carry_cancel_reason==1u,
	      "T14d brake/native cut cancel carry");
	carry_prepare(&r,1200u,15u);
	r.rel_accel=400; (void)rig_step(&r);
	CHECK(assist_v3_telemetry()->carry_cancel_reason==3u,
	      "T14e acceleration cancels carry");
	carry_prepare(&r,1200u,15u);
	r.dir=-1; r.rpm=60u; r.real_stop=false;
	for(unsigned i=0;i<20u;i++) (void)rig_step(&r);
	CHECK(assist_v3_telemetry()->carry_state!=1u,
	      "T14f reverse cancels carry");
	carry_prepare(&r,1200u,15u);
	r.real_stop=false; r.rpm=65u; r.load=10000u;
	for(unsigned i=0;i<50u;i++) (void)rig_step(&r);
	CHECK(assist_v3_telemetry()->carry_state==0u && assist_v3_engaged(),
	      "T14g restart returns to normal from the current trajectory");
	carry_prepare(&r,1200u,15u);
	r.level=0u; (void)rig_step(&r);
	CHECK(assist_v3_telemetry()->carry_cancel_reason==1u,
	      "T14h assist off cancels carry");
}

int main(void)
{
	puts("assist_v3 trajectory / transient manager (Milestone C rules, module level)");
	t1_conversion();
	t2_response();
	t3_power_on();
	t4_start_gate();
	t5_rise();
	t6_release();
	t7_stop_released();
	t8_stop_held();
	t9_reverse();
	t10_bounds();
	t11_elapsed();
	t12_motion();
	t13_reset();
	t14_carry();
	if (host_test_failures) {
		printf("assist_v3 trajectory: %d check(s) FAILED\n", host_test_failures);
		return 1;
	}
	puts("assist_v3 trajectory: PASS");
	return 0;
}
