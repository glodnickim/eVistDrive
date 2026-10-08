/*
 * Assist Behavior V3 - Milestone C ACTIVE engine at the pipeline level (docs/assist-v3/ARCHITECTURE_V3.md
 * 2.1, 2.2, 6, 7.3, 9; TEST_MATRIX G1-SAFE, G1-BACKSTOP, G1-SEL, G1-STAND, G1-STOP, G1-VETO, G1-TRAJ;
 * REVIEW-T section E). Real modules: the production pipeline (src/assist_pipeline.c) with the G53 chain,
 * ap2_limits, the V3 stage (assist_v3 + intent + motion), the V3 config owner (engine written through the
 * real CONFIG_PROTOCOL_V3 transfer) and the 16 kHz final-Iq owner (fast_iq_slew). Built -DASSIST_V3=1.
 *
 * "V3 FORCED" = a link-time wrap (-Wl,--wrap=assist_v3_update / assist_v3_stop_target_zero): the real V3
 * stage still runs, the pipeline receives a forced demand and/or a stop-target term stuck at false - the
 * misbehaving-V3 model of ARCHITECTURE_V3 2.1 / 7.3 (closure R-a). Every published bound below is therefore
 * provided by code outside src/assist_v3*.c.
 *
 * Both engines run the same rider rig: G5300 mode is the baseline behaviour (byte-identical to 25df554 by
 * G-EQ, tools/run_sil.py / run_regression.py / run_assist_v3_matrix.py --equivalence), so "<= baseline"
 * comparisons here are V3 vs G5300 on identical inputs.
 *
 * Rig: 1 call per 1 ms (elapsed 4 control ticks), PAS AB stepped from a signed crank step count (96 per
 * revolution), crank_steps / crank_step_tick / control_tick fed as main.c feeds them, real_stop false (the
 * G53 PAS true-stop is the stop authority here), fast_iq_slew ticked 16 x per call. While the crank turns,
 * the pedal load is a two-stroke shape around the commanded mean, load = mean * (1 + 0.5 sin(2 theta))
 * (peak/mean 1.5, the matrix "steady" ripple); a stopped crank holds the mean. Printed numbers are [SIM]
 * observations.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "assist_modes.h"
#include "assist_pipeline.h"
#include "assist_v3.h"
#include "assist_v3_config.h"
#include "assist_v3_harness.h"
#include "config.h"
#include "fast_iq_slew.h"
#include "g53_port.h"

#if !ASSIST_V3
#error "build with -DASSIST_V3=1"
#endif

static unsigned failures;
#define CHECK(c, m) do { if (!(c)) { printf("  FAIL  %s\n", (m)); ++failures; } } while (0)

#define P 700
#define FORCE_MAX 10000            /* far above every limit: the pipeline must bound it */
#define BDE8_PER_MS_X100 (P * 50 / 100)   /* 3.5 Iq/ms = 350 / 100 */
#define GUARD_PER_MS 7             /* fast slew 0.625 P Q8 per 4 kHz tick = 6.84 Iq/ms, integer ceiling */

/* ------------------------------------------------------------------ V3 forcing (link-time wrap) */

int32_t __real_assist_v3_update(const assist_v3_input_t *in);
bool __real_assist_v3_stop_target_zero(void);
static bool force_on;
static int32_t force_value;
static bool stop_zero_stuck_false;

int32_t __wrap_assist_v3_update(const assist_v3_input_t *in)
{
	const int32_t real = __real_assist_v3_update(in);
	return force_on ? force_value : real;
}

bool __wrap_assist_v3_stop_target_zero(void)
{
	return stop_zero_stuck_false ? false : __real_assist_v3_stop_target_zero();
}

/* ------------------------------------------------------------------ rider rig */

static const uint8_t AB[4] = { 0U, 2U, 3U, 1U };

typedef struct {
	assist_pipeline_input_t in;
	assist_pipeline_command_t cmd;
	fast_iq_slew_mailbox_t mb;
	int32_t ref;
	uint32_t tick;          /* 4 kHz control clock */
	int32_t steps;
	uint32_t step_tick;
	uint32_t acc;           /* rpm * 96 per ms, a step per 60000 */
	int32_t rpm;            /* signed: < 0 back-pedals */
	unsigned ci;
	uint32_t ms;            /* calls so far */
	bool engine_v3;
	uint16_t load_mean;     /* commanded mean pedal load, CLU (shaped while the crank turns) */
} rig_t;

static rig_t R;

static void rig_init(bool engine_v3)
{
	memset(&R, 0, sizeof(R));
	force_on = false; force_value = 0; stop_zero_stuck_false = false;
	assist_modes_init();
	g53_port_init();
	assist_pipeline_init();
	fast_iq_slew_reset(&R.mb);
	if (!assist_v3_harness_select_engine(engine_v3)) { puts("engine write rejected"); exit(2); }
	R.engine_v3 = engine_v3;
	R.tick = 1000U;
	assist_pipeline_input_t *in = &R.in;
	in->torque_sensor_valid = true; in->pas_sensor_valid = true;
	in->forward_valid = true; in->wheel_valid = true;
	in->assist_level_index = 3U; in->phase_current_max = P;
	in->battery_voltage_mv = 48000U; in->battery_current_max = BATTERYCURRENT_MAX;
	in->u_abs = 1024; in->cal_i = 95; in->voltage_raw = 4000U;
	in->voltage_min_raw = 2800; in->controller_temperature_c = 30;
	in->speed_limit_x100 = 6000U; in->elapsed_ticks = 4U;
}

/* One 1 ms call. */
static void rig_ms(void)
{
	assist_pipeline_input_t *in = &R.in;
	const uint32_t mag = (uint32_t)(R.rpm < 0 ? -R.rpm : R.rpm);
	R.tick += 4U;
	R.acc += mag * 96U;
	while (R.acc >= 60000U) {
		R.acc -= 60000U;
		if (R.rpm > 0) { R.steps++; R.ci = (R.ci + 1U) & 3U; }
		else { R.steps--; R.ci = (R.ci + 3U) & 3U; }
		R.step_tick = R.tick;
	}
	{
		const double theta = 2.0 * 3.14159265358979 * (double)(((R.steps % 96) + 96) % 96) / 96.0;
		const double l = R.rpm != 0 ? R.load_mean * (1.0 + 0.5 * sin(2.0 * theta)) : (double)R.load_mean;
		in->torque_load_ctrl = (uint16_t)(l < 0.0 ? 0.0 : l + 0.5);
	}
	in->pas_ab = AB[R.ci];
	in->crank_steps = R.steps;
	in->crank_step_tick = R.step_tick;
	in->control_tick = R.tick;
	in->cadence_rpm = (uint8_t)(R.rpm > 0 ? R.rpm : 0);
	in->torque_load_centikg = (uint16_t)(in->torque_load_ctrl / 2U);
	in->elapsed_ticks = 4U;
	assist_pipeline_update(in, &R.cmd);
	fast_iq_slew_publish(&R.mb, R.cmd.final_iq_request, R.cmd.slew_mode, R.cmd.step_mag_8,
		R.cmd.release_ticks_16k, R.cmd.zero_policy, R.cmd.iq_ceiling);
	for (unsigned j = 0U; j < 16U; ++j) fast_iq_slew_tick(&R.mb, &R.ref);
	in->live_iq_ref = R.ref; in->live_iq_valid = true;
	R.ms++;
}

static void rig_run(uint32_t ms) { for (uint32_t i = 0U; i < ms; ++i) rig_ms(); }

static const assist_pipeline_v3_status_t *st(void) { return assist_pipeline_v3_status(); }

/* Power-on standstill (EB74 arms with the pedal unloaded), then unloaded pedalling at speed (the
 * engine latch finds every demand at 0), then the rider loads the pedal and the demand settles. */
static void rig_ride(bool engine_v3, int32_t rpm, uint16_t load, uint32_t speed_x100, uint32_t settle_ms)
{
	rig_init(engine_v3);
	R.in.speed_x100 = 0U; R.load_mean = 0U; R.rpm = 0;
	rig_run(400U);
	R.in.speed_x100 = speed_x100; R.rpm = rpm;
	rig_run(300U);
	R.load_mean = load;
	rig_run(settle_ms);
}

/* Max published rise over any 1 ms step from now on for `ms` calls. */
static int32_t run_max_rise(uint32_t ms, int32_t *end)
{
	int32_t prev = R.cmd.final_iq_request, worst = 0;
	for (uint32_t i = 0U; i < ms; ++i) {
		rig_ms();
		if (R.cmd.final_iq_request - prev > worst) worst = R.cmd.final_iq_request - prev;
		prev = R.cmd.final_iq_request;
	}
	if (end) *end = prev;
	return worst;
}

/* ms until the published request first reaches 0 (-1 = never within max_ms). */
static int run_until_zero(uint32_t max_ms)
{
	for (uint32_t k = 0U; k < max_ms; ++k) {
		rig_ms();
		if (R.cmd.final_iq_request == 0) return (int)k;
	}
	return -1;
}

static uint8_t caps_engine_active_byte(void)
{
	uint8_t caps[ASSIST_V3_CAPS_LEN] = { 0 }, len = 0U;
	(void)assist_v3_config_can_read(ASSIST_V3_SOURCE_TOOL, ASSIST_V3_CMD_CAPS, 0U, caps, caps, &len);
	return len == ASSIST_V3_CAPS_LEN ? caps[24] : 0xFFU;
}

/* Engine request written through the real protocol, without re-initialising the config owner. */
static void request_engine(bool v3)
{
	assist_v3_values_t v;
	uint8_t blk[ASSIST_V3_BLOCK_LEN];
	assist_v3_config_ram_values(&v);
	v.engine = v3 ? ASSIST_V3_ENGINE_V3 : ASSIST_V3_ENGINE_G5300;
	assist_v3_block_encode(&v, ASSIST_V3_CAPS, assist_v3_config_generation(), blk);
	(void)assist_v3_config_can_declare(ASSIST_V3_SOURCE_TOOL, ASSIST_V3_BLOCK_LEN, 0U);
	for (uint8_t f = 0U; ; f++) {
		const uint16_t off = (uint16_t)(f * 8U);
		const bool end = off + 8U >= ASSIST_V3_BLOCK_LEN;
		(void)assist_v3_config_can_frame(ASSIST_V3_SOURCE_TOOL, f, end, &blk[off],
			(uint8_t)(end ? ASSIST_V3_BLOCK_LEN - off : 8U), 0U);
		if (end) break;
	}
}

/* ------------------------------------------------------------------ G1-SEL + truthful readback */

static void g1_sel(void)
{
	puts(" G1-SEL engine latch / truthful readback");
	rig_init(true);
	CHECK(!assist_v3_config_engine_active() && caps_engine_active_byte() == 0U,
	      "SEL1 boot: engine_active = G5300 (CAPS byte 22 = 0) until the latch, V3 requested");
	R.in.speed_x100 = 0U; R.load_mean = 0U;
	rig_run(400U);
	CHECK(!st()->engine_v3_active && st()->standstill_zero,
	      "SEL2 standstill at power-on is a veto: no latch while the crank is stopped at speed 0");
	R.in.speed_x100 = 1500U; R.rpm = 60;
	bool latched = false, clean = true;
	for (int i = 0; i < 300 && !latched; ++i) {
		rig_ms();
		if (st()->switched) {
			latched = true;
			clean = R.cmd.final_iq_request == 0 && !assist_v3_engaged() &&
			        assist_pipeline_g53()->iq_request_pre_limits == 0 && st()->pulled_down;
		}
	}
	CHECK(latched && clean && assist_v3_config_engine_active() && caps_engine_active_byte() == 1U,
	      "SEL3 latch on the first tick with published, V3 y and G53 demand all 0 and no veto; pulled_down set; CAPS = 1");

	/* riding in V3 with demand: request G5300 -> no latch while any demand is > 0 */
	R.load_mean = 3000U;
	rig_run(1500U);
	request_engine(false);
	bool premature = false;
	for (int i = 0; i < 2000; ++i) {
		rig_ms();
		if (!st()->engine_v3_active && (R.cmd.final_iq_request > 0 || assist_pipeline_g53()->iq_request_pre_limits > 0))
			premature = true;
	}
	CHECK(st()->engine_v3_active && !premature, "SEL4 V3 -> G5300 request while riding with demand: not latched");
	/* during a brake: published 0, demands high */
	R.in.safety_cut = true;
	rig_run(400U);
	CHECK(st()->engine_v3_active, "SEL5 during brake (native_cut, published 0, demands high): not latched");
	R.in.safety_cut = false;
	/* release: load off while pedalling - the G53 shadow holds its envelope for seconds after V3 is 0 */
	R.load_mean = 0U;
	int v3_zero_at = -1, latch_at = -1;
	for (int i = 0; i < 12000 && latch_at < 0; ++i) {
		rig_ms();
		if (v3_zero_at < 0 && !assist_v3_engaged()) v3_zero_at = i;
		if (st()->switched) {
			latch_at = i;
			CHECK(R.cmd.final_iq_request == 0 && assist_pipeline_g53()->iq_request_pre_limits == 0 &&
			      !assist_v3_engaged(), "SEL6 V3 -> G5300 latch only with every demand at 0");
		}
	}
	printf("    release: V3 y = 0 after %d ms, G5300 latched after %d ms (G53 envelope held meanwhile)\n",
	       v3_zero_at, latch_at);
	CHECK(latch_at > v3_zero_at && !st()->engine_v3_active && caps_engine_active_byte() == 0U,
	      "SEL7 during release: no latch while the G53 shadow holds demand; then G5300 active, CAPS = 0");
	/* the climb after the switch is governed by R1 */
	R.load_mean = 3000U;
	int32_t end = 0;
	const int32_t rise = run_max_rise(1500U, &end);
	printf("    after the switch: max rise %d Iq/ms, end %d\n", rise, end);
	CHECK(end > 50 && rise <= 4, "SEL8 after the switch the climb is never a step (R1 / BDE8 <= 3.5 Iq/ms + 1)");

	/* at standstill with demand: speed 0, crank stopped, load held -> G53 holds demand -> no latch */
	rig_ride(false, 60, 3000U, 1500U, 1500U);
	CHECK(!st()->engine_v3_active, "SEL9 setup: riding in G5300");
	request_engine(true);
	R.in.speed_x100 = 0U; R.rpm = 0;
	bool latched_with_demand = false;
	for (int i = 0; i < 600; ++i) {
		rig_ms();
		if (st()->switched && assist_pipeline_g53()->iq_request_pre_limits > 0) latched_with_demand = true;
	}
	CHECK(!latched_with_demand && !st()->engine_v3_active,
	      "SEL10 at standstill with demand (load held): not latched");

	/* pipeline reset (owner change) resets V3 */
	rig_ride(true, 60, 3000U, 1500U, 1500U);
	CHECK(assist_v3_engaged(), "SEL11 setup: V3 engaged");
	assist_pipeline_reset();
	CHECK(!assist_v3_engaged() && assist_v3_config_engine_active(),
	      "SEL12 pipeline reset resets V3 (y = 0); the active engine is kept");
}

/* ------------------------------------------------------------------ G1-SAFE */

static void g1_safe(void)
{
	puts(" G1-SAFE V3 cannot bypass the pipeline");
	/* native_cut with V3 forced to max */
	rig_ride(true, 60, 3000U, 1500U, 1500U);
	CHECK(st()->engine_v3_active && R.cmd.final_iq_request > 50, "SAFE0 setup: V3 publishes");
	force_on = true; force_value = FORCE_MAX;
	R.in.safety_cut = true;
	rig_ms();
	CHECK(R.cmd.final_iq_request == 0 && R.cmd.slew_mode == FIS_MODE_SAFETY && R.cmd.release_ticks_16k == 3200U,
	      "SAFE1 native_cut (brake) with V3 forced to max: 0, SAFETY 200 ms");
	{
		const assist_pipeline_command_t a = R.cmd;
		force_value = 0;
		rig_ms();
		CHECK(R.cmd.slew_mode == a.slew_mode && R.cmd.release_ticks_16k == a.release_ticks_16k &&
		      R.cmd.zero_policy == a.zero_policy && R.cmd.iq_ceiling == a.iq_ceiling && R.cmd.final_iq_request == 0,
		      "SAFE2 V3 output 0 vs max under native_cut: identical command (V3 writes no slew/ceiling/zero policy)");
	}
	R.in.safety_cut = false;
	R.in.torque_sensor_valid = false; force_value = FORCE_MAX;
	rig_ms();
	CHECK(R.cmd.final_iq_request == 0 && R.cmd.slew_mode == FIS_MODE_SAFETY, "SAFE3 torque sensor invalid: SAFETY whatever V3 says");
	R.in.torque_sensor_valid = true;
	/* assist off */
	R.in.assist_level_index = 0U;
	rig_ms();
	CHECK(R.cmd.final_iq_request == 0 && R.cmd.slew_mode == FIS_MODE_BYPASS, "SAFE4 assist off with V3 forced to max: 0 (BYPASS)");
	R.in.assist_level_index = 3U;
	rig_run(300U);
	/* limits: phase-current / power and the protection ceiling */
	int32_t worst_over_p = 0, worst_over_ceiling = 0;
	for (int i = 0; i < 500; ++i) {
		rig_ms();
		if (R.cmd.final_iq_request > worst_over_p) worst_over_p = R.cmd.final_iq_request;
		if (R.ref - R.cmd.iq_ceiling > worst_over_ceiling) worst_over_ceiling = R.ref - R.cmd.iq_ceiling;
	}
	CHECK(worst_over_p <= P && worst_over_ceiling <= 0,
	      "SAFE5 V3 forced to 10000 Iq: published <= phase limit, Iq reference <= protection ceiling");
	/* legal speed limit */
	R.in.legal_enabled = true; R.in.speed_limit_x100 = 2500U; R.in.speed_x100 = 3200U;
	rig_run(400U);
	CHECK(R.cmd.final_iq_request == 0 && assist_pipeline_telemetry()->speed_limited,
	      "SAFE6 above the legal speed limit: ap2_limits zeroes the forced V3 request");
	R.in.speed_x100 = 1500U;
	/* thermal */
	R.in.controller_temperature_c = 95;
	rig_run(400U);
	CHECK(assist_pipeline_telemetry()->thermal_limited && R.cmd.final_iq_request < worst_over_p,
	      "SAFE7 controller overtemperature: thermal derate binds the forced V3 request");
	R.in.controller_temperature_c = 30;
	rig_run(600U);
	/* g1: battery current above the limit -> G53 PI #1 lowers g1 -> (y * g1) >> 12 */
	R.in.battery_current_limiter_centiamp = 3000;   /* 30 A against the 15 A limit */
	rig_run(600U);
	{
		const int32_t g1 = assist_pipeline_g53()->trace.g1;
		printf("    g1 under overcurrent: %d / 4096, V3 request %d (demand %d)\n", (int)g1,
		       (int)st()->v3_request_iq, (int)st()->v3_demand_iq);
		CHECK(g1 < 4096 && st()->v3_request_iq == (int32_t)(((int64_t)FORCE_MAX * g1) >> 12) &&
		      R.cmd.final_iq_request <= st()->v3_request_iq,
		      "SAFE8 g1 multiplies the V3 demand (battery envelope applies to V3)");
	}
	R.in.battery_current_limiter_centiamp = 0;

	/* standstill zero, V3 real: crank stopped + load released at speed 0 */
	force_on = false;
	rig_ride(true, 60, 3000U, 1500U, 1500U);
	R.in.speed_x100 = 0U; R.rpm = 0; R.load_mean = 0U;
	const int z = run_until_zero(3000U);
	CHECK(z >= 0 && R.cmd.slew_mode == FIS_MODE_FORCE_ZERO && st()->standstill_zero,
	      "SAFE9 standstill (speed 0, crank stopped, load released): FORCE_ZERO");

	/* no rider load -> zero Iq (ported property), V3 real: unloaded and sub-threshold pedalling */
	for (int k = 0; k < 2; ++k) {
		/* 200 CLU mean: V3's intent is the mean, and even kL at its 2.5 clamp gives 2.5 x 200 =
		 * 500 CLU -> EB74 954 < 995 engage threshold; G53 sees the 300 CLU peak -> 872 < 995. */
		const uint16_t load = k == 0 ? 0U : 200U;
		rig_ride(true, 70, load, 1500U, 0U);
		int32_t mx = 0;
		for (int i = 0; i < 8000; ++i) { rig_ms(); if (R.cmd.final_iq_request > mx) mx = R.cmd.final_iq_request; }
		CHECK(st()->engine_v3_active && mx == 0, k == 0 ? "SAFE10 no rider load: zero Iq for 8 s (V3 active)" :
		      "SAFE11 sub-threshold load: zero Iq for 8 s (V3 active)");
	}
}

/* ------------------------------------------------------------------ G1-BACKSTOP (V3 forced to max) */

static void backstop_setup(uint32_t speed)
{
	rig_ride(true, 60, 3000U, 1500U, 1500U);
	force_on = true; force_value = FORCE_MAX; stop_zero_stuck_false = true;
	rig_run(400U);
	R.in.speed_x100 = speed;
	rig_run(50U);
}

static void g1_backstop(void)
{
	puts(" G1-BACKSTOP V3 forced to max, stop target stuck false");
	/* a: reverse at speed */
	backstop_setup(1500U);
	const int32_t before = R.cmd.final_iq_request;
	R.rpm = -30; R.in.direction_inhibit = true; R.in.inhibit_is_reverse = true; R.in.forward_valid = false;
	int32_t worst_excess = 0;
	int zero = -1;
	for (int k = 0; k < 1000; ++k) {
		rig_ms();
		int32_t bound = before - (int32_t)(((int64_t)BDE8_PER_MS_X100 * k) / 100) + 1;
		if (bound < 0) bound = 0;
		if (R.cmd.final_iq_request - bound > worst_excess) worst_excess = R.cmd.final_iq_request - bound;
		if (zero < 0 && R.cmd.final_iq_request == 0) zero = k;
	}
	printf("    reverse at speed: from %d, zero after %d ms (bound %d ms), excess over the 3.5 Iq/ms line %d\n",
	       before, zero, before * 100 / BDE8_PER_MS_X100 + 2, worst_excess);
	CHECK(before > 300 && worst_excess <= 0 && zero >= 0 && zero <= before * 100 / BDE8_PER_MS_X100 + 2,
	      "BS1 reverse: ceiling decays from the published value at >= 3.5 Iq/ms to 0");
	/* e: reverse -> forward: re-open only after forward steps, at <= 3.5 Iq/ms from the published value */
	R.in.direction_inhibit = false; R.in.inhibit_is_reverse = false;
	R.rpm = 0;
	rig_run(200U);
	CHECK(R.cmd.final_iq_request == 0, "BS2 no re-open without forward steps");
	R.rpm = 60; R.in.forward_valid = true;
	int32_t end = 0;
	const int32_t rise = run_max_rise(800U, &end);
	printf("    re-open after reverse: max rise %d Iq/ms, end %d\n", rise, end);
	CHECK(rise <= 4 && end > 300, "BS3 re-open after forward steps at <= 3.5 Iq/ms (+1 count)");

	/* rework (pas_glitch rows): an INVALID-transition inhibit (not reverse) while pedalling forward,
	 * V3 real, never closes the backstop and never dips the published request */
	rig_ride(true, 60, 3000U, 1500U, 2500U);
	{
		const int32_t b0 = R.cmd.final_iq_request;
		int32_t lo = b0;
		bool closed = false;
		R.in.direction_inhibit = true; R.in.inhibit_is_reverse = false;
		for (int k = 0; k < 40; ++k) {
			rig_ms();
			if (st()->backstop_state != ASSIST_PIPELINE_BS_OPEN) closed = true;
			if (R.cmd.final_iq_request < lo) lo = R.cmd.final_iq_request;
		}
		R.in.direction_inhibit = false;
		printf("    INVALID inhibit 40 ms while pedalling: published %d -> min %d, backstop closed %d\n",
		       (int)b0, (int)lo, closed);
		CHECK(!closed && lo * 10 >= b0 * 9, "BS8 an INVALID-transition inhibit (not reverse) neither closes the backstop nor dips the request");
	}

	/* reverse at speed 0: standstill predicate (direction term, independent of V3) -> FORCE_ZERO */
	backstop_setup(0U);
	R.in.speed_x100 = 0U;
	R.rpm = -30; R.in.direction_inhibit = true; R.in.inhibit_is_reverse = true; R.in.forward_valid = false;
	rig_ms();
	CHECK(R.cmd.final_iq_request == 0 && R.cmd.slew_mode == FIS_MODE_FORCE_ZERO,
	      "BS4 reverse at speed 0: FORCE_ZERO on the first tick whatever V3 says");

	/* c: crank stopped at speed, load held: hold <= 1500 ms then 0 within 300 ms */
	for (int sp = 0; sp < 2; ++sp) {
		backstop_setup(sp == 0 ? 1500U : 0U);
		const int32_t b0 = R.cmd.final_iq_request;
		R.rpm = 0;   /* load stays 3000 */
		int ts = -1, z0 = -1;
		int32_t above = 0;
		for (int k = 0; k < 3000; ++k) {
			rig_ms();
			if (ts < 0 && g53_port_pas_true_stop()) ts = k;
			if (R.cmd.final_iq_request > b0 && R.cmd.final_iq_request - b0 > above) above = R.cmd.final_iq_request - b0;
			if (z0 < 0 && R.cmd.final_iq_request == 0) z0 = k;
		}
		printf("    stop, load held, speed %s: from %d, true-stop at %d ms, zero at %d ms (bound true-stop + 1800)\n",
		       sp == 0 ? "15 km/h" : "0", b0, ts, z0);
		CHECK(ts >= 0 && z0 > ts && z0 <= ts + 1800 + 2 && above == 0,
		      sp == 0 ? "BS5 crank stopped at speed: never above the published value, 0 by true-stop + 1500 + 300 ms" :
		                "BS6 crank stopped at speed 0, V3 stop term stuck false: backstop alone bounds it (<= 1.8 s)");
		if (sp == 0) {
			/* f: past T_STOP_HARD -> restart: rise <= 3.5 Iq/ms from the published value */
			R.rpm = 60;
			const int32_t r2 = run_max_rise(800U, &end);
			printf("    restart after T_STOP_HARD: max rise %d Iq/ms, end %d\n", r2, end);
			CHECK(r2 <= 4 && end > 300, "BS7 stop past T_STOP_HARD -> restart: re-open at <= 3.5 Iq/ms");
		}
	}
	/* D: wheel intervals give an interpolated distance bound before the 1500 ms time cap. */
	backstop_setup(800U);
	R.in.wheel_pulse_tick=R.tick;
	rig_ms();
	rig_run(1000U);
	R.in.wheel_pulse_tick=R.tick;
	rig_ms();
	R.rpm=0;
	int first_stop=-1, by_distance=-1;
	int32_t prev=R.cmd.final_iq_request, v_stop=0, max_drop=0;
	for(int k=0;k<1800;k++){
		rig_ms();
		if(first_stop<0 && g53_port_pas_true_stop()) { first_stop=k; v_stop=R.cmd.final_iq_request; }
		if(first_stop>=0 && prev-R.cmd.final_iq_request>max_drop) max_drop=prev-R.cmd.final_iq_request;
		prev=R.cmd.final_iq_request;
		if(first_stop>=0 && R.cmd.final_iq_request==0){by_distance=k;break;}
	}
	CHECK(first_stop>=0 && by_distance>first_stop && by_distance<first_stop+1500,
	      "BS9 wheel-interpolated distance closes the independent backstop before its time cap");
	/* REVIEW 2 #4: a hold ended by the distance bound must still DECAY over ~300 ms, never step to 0. */
	printf("    BS9b distance-ended hold: published at stop %d, max drop per ms %d\n", (int)v_stop, (int)max_drop);
	CHECK(v_stop>0 && max_drop*200<=v_stop*1+400,
	      "BS9b a hold ended by the distance bound decays (<= published/200 + 2 Iq per ms), no step to 0");
}

/* ------------------------------------------------------------------ G1-STOP / G1-STAND (V3 vs G5300) */

typedef enum { EV_STOP_HELD, EV_STOP_RELEASED, EV_REVERSE } stop_ev_t;

/* ms from the event until the published request is 0; also the published value at the event */
static int stop_time(bool v3, int32_t rpm, uint32_t speed, stop_ev_t ev, int32_t *at_event)
{
	const uint32_t settle = (uint32_t)(8 * 60000 / rpm) > 3000U ? (uint32_t)(8 * 60000 / rpm) : 3000U;
	rig_ride(v3, rpm, 1500U, 1500U, settle);
	R.in.speed_x100 = speed;
	rig_run(20U);
	*at_event = R.cmd.final_iq_request;
	switch (ev) {
	case EV_STOP_HELD: R.rpm = 0; break;
	case EV_STOP_RELEASED: R.rpm = 0; R.load_mean = 0U; break;
	default:
		R.rpm = -(rpm / 2 > 15 ? rpm / 2 : 15);
		R.in.direction_inhibit = true; R.in.inhibit_is_reverse = true; R.in.forward_valid = false;
		break;
	}
	return run_until_zero(4000U);
}

static void g1_stop_stand(void)
{
	static const char *ev_name[3] = { "stop held", "stop released", "reverse" };
	static const int32_t cad[3] = { 25, 60, 120 };
	puts(" G1-STOP (speed 15 km/h) / G1-STAND (speed 0): zero time V3 vs G5300 on identical inputs");
	for (int s = 0; s < 2; ++s) {
		const uint32_t speed = s == 0 ? 1500U : 0U;
		for (int c = 0; c < 3; ++c) {
			for (int e = 0; e < 3; ++e) {
				int32_t gb = 0, vb = 0;
				const int tg = stop_time(false, cad[c], speed, (stop_ev_t)e, &gb);
				const int tv = stop_time(true, cad[c], speed, (stop_ev_t)e, &vb);
				char msg[200];
				printf("    %-7s %3d rpm %-13s G5300 %4d ms (from %3d)  V3 %4d ms (from %3d)\n",
				       s == 0 ? "STOP" : "STAND", (int)cad[c], ev_name[e], tg, (int)gb, tv, (int)vb);
				if (s == 1 && e == EV_STOP_HELD) {
					/* two-sided: the legacy ramp within +-20 %, no early cut */
					snprintf(msg, sizeof(msg), "STAND %d rpm load held: V3 zero time within +-20 %% of G5300 (%d vs %d ms)",
					         (int)cad[c], tv, tg);
					CHECK(tg > 0 && tv > 0 && tv * 10 >= tg * 8 && tv * 10 <= tg * 12, msg);
				} else {
					snprintf(msg, sizeof(msg), "%s %d rpm %s: V3 zero time <= G5300 (%d vs %d ms)",
					         s == 0 ? "STOP" : "STAND", (int)cad[c], ev_name[e], tv, tg);
					CHECK(tg >= 0 && tv >= 0 && tv <= tg, msg);
				}
			}
		}
	}
	/* start from rest: speed 0 for the first ~4.4 m while the crank turns loaded -> assist flows */
	rig_init(true);
	R.in.speed_x100 = 0U; R.load_mean = 0U;
	rig_run(400U);
	R.rpm = 50; R.load_mean = 3000U;
	int32_t mx = 0;
	for (int i = 0; i < 2000; ++i) { rig_ms(); if (R.cmd.final_iq_request > mx) mx = R.cmd.final_iq_request; }
	printf("    start from rest at speed 0: max published %d, engine V3 %d\n", (int)mx, st()->engine_v3_active);
	CHECK(st()->engine_v3_active && mx > 100, "STAND start from rest (speed 0, crank turning, loaded): V3 assist flows");
}

/* ------------------------------------------------------------------ G1-VETO (both engines) */

static void g1_veto(void)
{
	int32_t all_rise[2] = { 0, 0 };
	puts(" G1-VETO brake release while pedalling / assist off -> on (both engines)");
	for (int eng = 0; eng < 2; ++eng) {
		const bool v3 = eng == 1;
		char msg[160];
		rig_ride(v3, 60, 3000U, 1500U, 2000U);
		const int32_t before = R.cmd.final_iq_request;
		R.in.safety_cut = true;
		rig_run(300U);
		CHECK(R.cmd.final_iq_request == 0 && R.ref == 0, v3 ? "VETO1 V3 brake: 0" : "VETO1 G5300 brake: 0");
		R.in.safety_cut = false;
		int32_t end = 0;
		const int32_t rise = run_max_rise(1500U, &end);
		printf("    %s brake release: before %d, max rise %d Iq/ms, end %d\n", v3 ? "V3   " : "G5300", before, rise, end);
		snprintf(msg, sizeof(msg), "VETO2 %s brake release while pedalling: climb at the R1 rate (<= 3.5 Iq/ms + 1)",
		         v3 ? "V3" : "G5300");
		CHECK(rise <= 4 && end > before / 2, msg);
		/* assist off -> on: the stale demand is never published as a step (R1 governs) */
		R.in.assist_level_index = 0U;
		rig_run(80U);
		CHECK(R.cmd.final_iq_request == 0, v3 ? "VETO3 V3 assist off: 0" : "VETO3 G5300 assist off: 0");
		if (v3) CHECK(assist_v3_engaged(), "VETO4 V3 assist off for 80 ms: y still > 0 (stale demand exists)");
		R.in.assist_level_index = 3U;
		{
			/* While pulled_down (R1 armed) the climb is <= 3.5 Iq/ms + 1 count; afterwards the engine's
			 * own dynamics apply - including the g1 soft start after assist off, which is battery
			 * envelope, identical in both engines - so the overall maximum is compared with G5300. */
			int32_t prev = R.cmd.final_iq_request, r_pulled = 0, r_all = 0;
			bool was_pulled = st()->pulled_down;
			const int32_t first_prev = prev;
			int32_t first = 0;
			for (int i = 0; i < 1500; ++i) {
				rig_ms();
				const int32_t d = R.cmd.final_iq_request - prev;
				if (i == 0) first = R.cmd.final_iq_request - first_prev;
				if (was_pulled && d > r_pulled) r_pulled = d;
				if (d > r_all) r_all = d;
				was_pulled = st()->pulled_down;
				prev = R.cmd.final_iq_request;
			}
			all_rise[eng] = r_all;
			printf("    %s assist off -> on: first step %d, max rise while R1 armed %d, overall %d Iq/ms, end %d\n",
			       v3 ? "V3   " : "G5300", first, r_pulled, r_all, prev);
			snprintf(msg, sizeof(msg), "VETO5 %s assist off -> on: no step (first <= 4), R1 governs while armed (<= 4)",
			         v3 ? "V3" : "G5300");
			CHECK(first <= 4 && r_pulled <= 4 && prev > 50, msg);
		}
	}
	CHECK(all_rise[1] <= all_rise[0] && all_rise[1] < GUARD_PER_MS,
	      "VETO6 V3 assist off -> on: overall climb never steeper than G5300 and below the 6.84 Iq/ms guard");
}

/* ------------------------------------------------------------------ G1-TRAJ */

static void g1_traj(void)
{
	puts(" G1-TRAJ V3 published request never exceeds the 6.84 Iq/ms guard in normal riding");
	static const int32_t cad[4] = { 25, 60, 90, 120 };
	for (int c = 0; c < 4; ++c) {
		rig_ride(true, cad[c], 1500U, 1500U, 0U);
		int32_t prev = R.cmd.final_iq_request, worst = 0;
		uint32_t binds = 0U;
		static const uint16_t loads[] = { 1500U, 4500U, 1500U, 300U, 3000U, 6000U, 2000U };
		for (unsigned seg = 0U; seg < sizeof(loads) / sizeof(loads[0]); ++seg) {
			R.load_mean = loads[seg];
			if (seg == 4U) R.in.assist_level_index = 4U;   /* level change while riding */
			if (seg == 6U) R.in.assist_level_index = 2U;
			for (int i = 0; i < 2500; ++i) {
				rig_ms();
				const int32_t d = R.cmd.final_iq_request - prev;
				if ((d < 0 ? -d : d) > worst) worst = d < 0 ? -d : d;
				if (R.ref != R.cmd.final_iq_request) binds++;
				prev = R.cmd.final_iq_request;
			}
		}
		char msg[160];
		printf("    %3d rpm: max |dIq| %d Iq/ms, fast-slew lag ticks %u\n", (int)cad[c], (int)worst, binds);
		snprintf(msg, sizeof(msg), "TRAJ %d rpm: |published step| <= 6.84 Iq/ms and the fast slew never binds", (int)cad[c]);
		CHECK(worst <= GUARD_PER_MS - 1 && binds == 0U, msg);
	}
}

int main(void)
{
	const uint8_t accel[10] = {1,8,8,8,8,8,8,8,8,8};
	const uint16_t ratio[10] = {1,45,95,155,215,260,310,370,525,525};
	puts("assist_v3 pipeline (Milestone C active engine)");
	g53_chain_set_levels(accel, ratio);
	g1_sel();
	g1_safe();
	g1_backstop();
	g1_stop_stand();
	g1_veto();
	g1_traj();
	if (failures) { printf("assist_v3 pipeline: %u FAILED\n", failures); return 1; }
	puts("assist_v3 pipeline (G1-SEL/SAFE/BACKSTOP/STOP/STAND/VETO/TRAJ): PASS");
	return 0;
}
