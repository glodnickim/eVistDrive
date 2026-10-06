/*
 * M820 UNCONTROLLED-IQ SAFETY REGRESSION (hardware incident on 50b8758: the motor produced
 * torque with no rider demand, stopped, and did it again).
 *
 * Runs the PRODUCTION chain end to end, tick by tick:
 *   pas_sampler -> pas_direction -> pas_liveness (native PAS front-end, real_stop, inhibit)
 *   -> ride_control_update() -> assist_pipeline_update() -> g53_port (A-x, EB74, PAS, chain)
 *   -> ap2_limits -> the final-Iq mailbox -> fast_iq_slew_tick() at 16 kHz -> MS.i_q_setpoint
 * Only main.c's glue is modelled here (how rider_input_t is filled from the PAS front-end, and
 * the cadence measurement, marked [MODEL]). main.c's own Walk / comms-watchdog / 0x6300 /
 * position-calibration code runs from the real source in tests/test_m820_walk_can_safety.py.
 *
 * THROTTLE IS DISABLED FOR THE M820 SAFETY BENCH TEST (src/g53_port.c). This is not the final
 * throttle architecture; a validated M820 PA6 adapter is separate work.
 *
 *   T1  m820_pa6_hidden_throttle_regression - the exact replayed failure. The OLD port seam
 *       (reconstructed below from the public G53 boundary/chain APIs exactly as 50b8758 wired
 *       it) still turns PA6=2048 into A-x 246 -> FSM 2142 -> M2AA 3399 -> Iq 237; the CURRENT
 *       production chain, same inputs, never publishes a positive request or reference.
 *   T2  no-rider matrix: PA6 x speed x level x PAS pattern with load 0 -> Iq stays exactly 0.
 *   T3  loaded matrix: the Iq trajectory is bit-identical for every PA6 (PA6 cannot change
 *       motor demand), !forward_valid never raises Iq, OFF never has Iq.
 *   T4  PA6DIP and hidden-demand-under-veto: no preloaded demand is released by a veto clearing.
 *   T5  hard inhibits: comm_inhibit is exact zero (FORCE_ZERO, ceiling 0) in the same update,
 *       ahead of Walk and position calibration; direction is exact zero; brake/torque fault
 *       request exactly zero and release monotonically. Recovery resumes only with pedal input.
 *   T6  ordinary pedal release is preserved; assist OFF is exact zero in the same update.
 *   T7  contract §10b: on the ported G53 normal path the limiter gets max_power_w = 0 and
 *       level_iq_limit = AP2_LIMITS_NO_LEVEL_CEILING - stored level ceilings change nothing.
 */
#include "check.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "config.h"
#include "assist_modes.h"
#include "assist_pipeline.h"
#include "fast_iq_slew.h"
#include "g53_port.h"
#include "g53_port_boundaries.h"
#include "g53_port_chain.h"
#include "g53_port_pas.h"
#include "motor_core.h"
#include "pas_direction.h"
#include "pas_liveness.h"
#include "pas_sampler.h"
#include "ride_control.h"
#include "rider_input.h"
#include "tuning_config.h"

#define STRINGIZE2(x) #x
#define STRINGIZE(x) STRINGIZE2(x)
#ifndef ASSIST_PIPELINE_C_PATH
#error "ASSIST_PIPELINE_C_PATH must be supplied by run-host-tests.ps1"
#endif

/* ride_control.c's two service owners, driven with fixed, clearly positive requests so that
 * T5 can prove the communication inhibit wins over a Walk or a calibration that WOULD pull. */
static uint16_t walk_iq_stub;
static uint16_t cal_iq_stub;
static unsigned cal_calls;
uint16_t walk_assist_iq_request(void) { return walk_iq_stub; }
uint16_t hall_calibration_iq_request(void) { cal_calls++; return cal_iq_stub; }

static MotorState_t ms;
static fast_iq_slew_mailbox_t *mb;
static const uint8_t PAS_FWD[4] = { 0U, 2U, 3U, 1U };   /* native M820 forward sequence */

typedef struct {
	uint16_t pa6;
	uint16_t load_ctrl;
	int16_t crank_rpm;       /* >0 forward, <0 reverse; used while pas_force < 0 */
	int8_t pas_force;        /* 0..3 hold the native lines, -1 generate from crank_rpm */
	uint32_t speed_x100;
	uint8_t level_index;     /* 0..5 */
	bool brake;
	bool torque_valid;
	bool walk;
	bool calibration;
	bool comm_inhibit;
	bool battery_trip;       /* TQ-06-G1: hard battery-overcurrent trip latched */
	uint16_t soc_derate;     /* TQ-06-G1 step 2: Q12 SOC derate of the G53 limit */
	int16_t temp_c;          /* controller temperature; 0 in base() means the old fixed 25 degC */
} sim_in_t;

static struct {
	uint32_t now;
	double pas_acc;
	uint8_t phase;
	uint8_t start_phase;
	uint8_t cadence;
	uint16_t last_fwd_period;
	uint8_t fwd_since_stop;
	uint16_t gaps[4];
	bool real_stop, forward_valid;
	int32_t iq, prev_iq;
} S;

static sim_in_t base(void)
{
	sim_in_t in;
	memset(&in, 0, sizeof(in));
	in.pas_force = 0;
	in.torque_valid = true;
	in.level_index = 1;
	return in;
}

static void power_on(void)
{
	memset(&S, 0, sizeof(S));
	S.last_fwd_period = PAS_STOP_TICKS;
	memset(&ms, 0, sizeof(ms));
	walk_iq_stub = 0U;
	cal_iq_stub = 0U;
	cal_calls = 0U;
	assist_modes_init();
	assist_modes_set_active_bank(0U);
	motor_core_init(&ms);
	ride_control_init();
	pas_sampler_init(0U);
	pas_direction_init();
	pas_liveness_init();
	mb = ride_control_final_iq_slew_mailbox();
}

static void one_edge(sim_in_t *in, bool forward)
{
	S.phase = (uint8_t)((S.phase + (forward ? 1U : 3U)) & 3U);
	in->pas_force = (int8_t)PAS_FWD[S.phase];
}

/* One 4 kHz control iteration and its four 16 kHz fast-owner ticks. */
static void tick(const sim_in_t *in)
{
	S.now++;
	uint8_t ab;
	if (in->pas_force >= 0) {
		ab = (uint8_t)in->pas_force;
	} else {
		const int r = in->crank_rpm;
		S.pas_acc += (double)(r < 0 ? -r : r) * (double)PAS_TRANSITIONS_PER_REV / 60.0 / 4000.0;
		while (S.pas_acc >= 1.0) {
			S.pas_acc -= 1.0;
			S.phase = (uint8_t)((S.phase + (r > 0 ? 1U : 3U)) & 3U);
		}
		ab = PAS_FWD[S.phase];
	}
	pas_sampler_isr_tick(ab, S.now);
	pas_step_event_t ev;
	while (pas_sampler_pop(&ev)) {
		pas_direction_on_step(ev.step);
		if (ev.step > 0) {
			if (S.cadence == 0U && !S.start_phase && pas_direction_fwd_run() >= START_PHASE_STEPS)
				S.start_phase = 1U;
			S.gaps[S.fwd_since_stop & 3U] = ev.gap;
			if (S.fwd_since_stop < 255U) S.fwd_since_stop++;
			S.last_fwd_period = ev.gap;
			/* [MODEL] a cadence pulse every PAS_STEPS_PER_PULSE forward steps */
			if (S.fwd_since_stop > PAS_STEPS_PER_PULSE &&
			    (S.fwd_since_stop % PAS_STEPS_PER_PULSE) == 1U) {
				const uint32_t g = (uint32_t)S.gaps[0] + S.gaps[1] + S.gaps[2] + S.gaps[3];
				const uint32_t rpm = g ? (60U * 4000U * PAS_STEPS_PER_PULSE) /
					(PAS_TRANSITIONS_PER_REV * g) : 0U;
				S.cadence = (uint8_t)(rpm > 255U ? 255U : rpm);
				S.start_phase = 0U;
			}
		} else {
			S.fwd_since_stop = 0U;
		}
	}
	{	/* main.c: pas_idle_ticks / adaptive stop timeout / real_stop / forward_pedaling */
		uint32_t idle = S.now - pas_sampler_last_transition_tick();
		if (idle > 64000U) idle = 64000U;
		uint32_t to = (uint32_t)S.last_fwd_period * 2U;
		if (to < PAS_STOP_TICKS) to = PAS_STOP_TICKS;
		else if (to > PAS_STOP_TICKS_MAX) to = PAS_STOP_TICKS_MAX;
		pas_liveness_update(idle, (uint16_t)to);
		S.real_stop = pas_liveness_stopped();
		if (S.real_stop) {
			S.cadence = 0U; S.start_phase = 0U; S.fwd_since_stop = 0U;
			pas_direction_on_stop();
		}
		S.forward_valid = (S.cadence > 0U || S.start_phase) && !S.real_stop;
	}
	rider_input_t ri;
	memset(&ri, 0, sizeof(ri));
	ri.torque_load_ctrl = in->load_ctrl;
	ri.torque_load_centikg = in->load_ctrl;
	ri.cadence_rpm = S.cadence;
	ri.wheel_speed_x100 = in->speed_x100;
	ri.pas_forward = S.forward_valid;
	ri.pedaling_active = S.forward_valid;
	ri.pas_backward = pas_direction_backpedal_confirmed();
	ri.crank_forward_steps = pas_direction_fwd_run();
	ri.crank_direction_ok = S.forward_valid;
	ri.real_stop = S.real_stop;
	ri.wheel_valid = in->speed_x100 > 0U;
	ri.direction_inhibit_active = pas_direction_direction_inhibit_active();
	ri.start_phase = S.start_phase != 0U;
	ri.torque_sensor_valid = in->torque_valid;
	ri.pas_sensor_valid = pas_sampler_seeded() != 0U;
	rider_input_update(&ri);

	ride_control_input_t ci;
	memset(&ci, 0, sizeof(ci));
	ci.raw_pa6_adc = in->pa6;
	ci.pas_ab = pas_sampler_state();
	ci.speed_x100 = in->speed_x100;
	ci.cadence_rpm = S.cadence;
	ci.assist_level_index = in->level_index;
	ci.battery_voltage_mv = 42000U;
	ci.iq_scale = PH_CURRENT_MAX;
	ci.ride_core_iq_limit = PH_CURRENT_MAX;
	ci.phase_current_max = PH_CURRENT_MAX;
	ci.battery_current_max = BATTERYCURRENT_MAX;
	ci.u_abs = 1024;
	ci.cal_i = CAL_I;
	ci.current_iq = ms.i_q_setpoint;
	ci.voltage_raw = (uint16_t)(42000 / CAL_BAT_V);
	ci.voltage_min_raw = VOLTAGE_MIN;
	ci.controller_temperature_c = in->temp_c ? in->temp_c : 25;
	ci.cadence_filtered_x8 = (uint16_t)(S.cadence * 8U);
	ci.speed_limit_x100 = SPEEDLIMIT;
	ci.legal_enabled = true;
	ci.walk_active = in->walk;
	ci.position_calibration_active = in->calibration;
	ci.comm_inhibit = in->comm_inhibit;
	ci.battery_trip_latched = in->battery_trip;
	ci.battery_soc_derate_q12 = in->soc_derate;
	ci.safety_cut_non_direction = in->brake || !in->torque_valid;
	ci.start_phase = S.start_phase != 0U;
	ci.elapsed_ticks = 1U;
	ride_control_update(&ci);
	S.prev_iq = S.iq;
	for (int k = 0; k < 4; k++) fast_iq_slew_tick(mb, &ms.i_q_setpoint);
	S.iq = ms.i_q_setpoint;
}

#define SEC(s) ((uint32_t)((s) * 4000.0))

/* Per-run invariant bookkeeping. */
typedef struct {
	int32_t max_iq;
	uint32_t positive_ticks;
	uint32_t nonforward_rise;   /* !forward_valid and Iq rose */
	uint32_t off_positive;      /* level OFF and Iq > 0 */
	uint32_t hash;              /* FNV-1a of the Iq trajectory */
} run_t;

static void observe(run_t *r, const sim_in_t *in)
{
	if (S.iq > r->max_iq) r->max_iq = S.iq;
	if (S.iq > 0) r->positive_ticks++;
	if (!S.forward_valid && !in->walk && S.iq > S.prev_iq) r->nonforward_rise++;
	if (in->level_index == 0U && !in->walk && S.iq > 0) r->off_positive++;
	uint32_t v = (uint32_t)S.iq;
	for (int b = 0; b < 4; b++) { r->hash ^= (v >> (8 * b)) & 0xFFU; r->hash *= 16777619U; }
}

static void run_for(sim_in_t *in, uint32_t ticks, run_t *r)
{
	for (uint32_t k = 0; k < ticks; k++) { tick(in); observe(r, in); }
}

/* PAS patterns, applied after the boot idle with the configured load. */
enum { PAT_STATIONARY, PAT_SINGLE_EDGE, PAT_FORWARD, PAT_GLITCH, PAT_STOP_AFTER, PAT_REVERSE, PAT_N };
static const char *PAT_NAME[PAT_N] = { "stationary", "single-edge", "forward-seq", "glitch",
	"stop-after-pedalling", "reverse" };

static run_t scenario(uint16_t pa6, uint32_t speed, uint8_t level, uint16_t load, int pattern)
{
	run_t r;
	memset(&r, 0, sizeof(r));
	r.hash = 2166136261U;
	power_on();
	sim_in_t in = base();
	in.level_index = level;
	in.speed_x100 = speed;
	in.pa6 = pa6;
	run_for(&in, SEC(2.0), &r);           /* cold boot, rider idle, PA6 present */
	in.load_ctrl = load;
	switch (pattern) {
	case PAT_STATIONARY:
		run_for(&in, SEC(3.0), &r); break;
	case PAT_SINGLE_EDGE:
		run_for(&in, SEC(0.5), &r); one_edge(&in, true); run_for(&in, SEC(2.5), &r); break;
	case PAT_FORWARD:
		in.pas_force = -1; in.crank_rpm = 70; run_for(&in, SEC(3.0), &r); break;
	case PAT_GLITCH:   /* one forward edge and straight back: a bounce */
		one_edge(&in, true); run_for(&in, 2U, &r); one_edge(&in, false); run_for(&in, SEC(3.0), &r); break;
	case PAT_STOP_AFTER:
		in.pas_force = -1; in.crank_rpm = 70; run_for(&in, SEC(1.5), &r);
		in.crank_rpm = 0; in.pas_force = (int8_t)PAS_FWD[S.phase]; run_for(&in, SEC(1.5), &r); break;
	case PAT_REVERSE:
		in.pas_force = -1; in.crank_rpm = 70; run_for(&in, SEC(1.2), &r);
		in.crank_rpm = -40; run_for(&in, SEC(0.6), &r);
		in.crank_rpm = 70; run_for(&in, SEC(1.2), &r); break;
	}
	in.load_ctrl = 0U;
	run_for(&in, SEC(0.5), &r);
	return r;
}

/* ------------------------------------------------------------------------------------------- */
/* T1: the replayed failure.                                                                    */

/* The 50b8758 port seam, reconstructed from the public boundary/chain APIs exactly as it was
 * wired (A-x fed straight from PA6). It exists ONLY to keep the evidence of the old failure
 * executable; production no longer has this path. */
static struct { g53_pas_ctx_t pas; g53_port_trace_t trace; uint8_t rem; } old;
static void old_seam_reset(void)
{
	g53_ax_reset(); g53_ad7ec_reset(); g53_pas_reset(&old.pas, true, 0); g53_chain_reset();
	memset(&old.trace, 0, sizeof(old.trace)); old.rem = 0U;
}
static int32_t old_seam_step(uint16_t pa6, uint8_t native_ab, uint32_t speed_x100, uint8_t level_index)
{
	const uint32_t elapsed = 1U + old.rem;
	uint32_t steps = elapsed / 4U;
	old.rem = (uint8_t)(elapsed % 4U);
	const uint8_t level = g53_chain_level(level_index);
	const uint32_t speed = speed_x100 / 10U;
	while (steps--) {
		g53_chain_input_t ci;
		g53_chain_output_t co;
		memset(&ci, 0, sizeof(ci));
		ci.x = g53_ax_step(pa6);                                  /* THE old wiring */
		ci.pas = g53_pas_step(&old.pas,
			(uint8_t)(((native_ab & 1U) << 1) | ((native_ab & 2U) >> 1)), level);
		const g53_ad7ec_feedback_t fb = {
			.cadence = ci.pas.cadence,
			.speed_native = (uint16_t)(speed > 32767U ? 32767U : speed),
			.d7ec_rider = (uint16_t)old.trace.d7ec_accel,
			.m298 = (uint8_t)old.trace.m298 };
		ci.rider_input_native = g53_ad7ec_step(0U, &fb).rider_input_native;
		ci.level = level;
		ci.speed_native = (int16_t)fb.speed_native;
		ci.g1_q12 = 0x1000; ci.g2_q12 = 0x1000;
		g53_chain_step(&ci, &co);
		old.trace = co.trace;
	}
	return g53_boundary_b_iq_request((uint16_t)old.trace.m2aa, PH_CURRENT_MAX);
}

static void t1_hidden_throttle(void)
{
	puts("T1 m820_pa6_hidden_throttle_regression");
	/* OLD: level 1, 15 km/h, load 0, cadence 0, PA6 0 -> 2048 after 5 s. The demand builds with
	 * no PAS edge at all; on the bike one PAS edge only removed the real_stop veto hiding it. */
	old_seam_reset();
	int32_t old_iq = 0;
	for (uint32_t t = 0; t < SEC(10.0); t++)
		old_iq = old_seam_step(t < SEC(5.0) ? 0U : 2048U, 0U, 1500U, 1U);
	printf("  OLD seam: A-x=%d FSM=%d M2AA=%d Iq=%d\n", old.trace.x, old.trace.fsm_output,
		old.trace.m2aa, old_iq);
	CHECK(old.trace.x >= 240 && old.trace.x <= 250 && old.trace.fsm_output >= 2100 &&
	      old.trace.fsm_output <= 2200 && old.trace.m2aa >= 3300 && old.trace.m2aa <= 3500 &&
	      old_iq >= 230 && old_iq <= 245,
		"T1: the old A-x seam still reproduces the recorded failure (A-x~246, FSM~2142, M2AA~3399, Iq~237)");

	/* NEW: the same inputs through the whole production chain, then one forward PAS edge. */
	power_on();
	sim_in_t in = base();
	in.level_index = 1U; in.speed_x100 = 1500U;
	int32_t max_req = 0, max_iq = 0, max_m2aa = 0, max_fsm = 0;
	for (uint32_t t = 0; t < SEC(20.0); t++) {
		if (t == SEC(5.0)) in.pa6 = 2048U;
		if (t == SEC(10.0)) one_edge(&in, true);
		tick(&in);
		if ((int32_t)mb->target > max_req) max_req = (int32_t)mb->target;
		if (S.iq > max_iq) max_iq = S.iq;
		const g53_port_trace_t *g = &assist_pipeline_g53()->trace;
		if (g->m2aa > max_m2aa && g->m2aa < 32768) max_m2aa = g->m2aa;
		if (g->fsm_output > max_fsm) max_fsm = g->fsm_output;
	}
	printf("  NEW chain: final_iq_request max=%d MS.i_q_setpoint max=%d M2AA max=%d FSM max=%d "
		"PA6 A-x observed=%u\n", max_req, max_iq, max_m2aa, max_fsm, g53_port_pa6_ax_observed());
	CHECK(max_req == 0 && max_iq == 0,
		"T1: PA6=2048 at 15 km/h with no rider demand never publishes a positive request or reference");
	CHECK(max_m2aa == 0 && max_fsm == 0,
		"T1: and no throttle demand exists behind the veto at all (FSM output and M2AA stay 0)");
	CHECK(g53_port_pa6_ax_observed() >= 240U && g53_port_pa6_ax_observed() <= 250U,
		"T1: PA6 is still sampled and filtered for diagnostics");
}

/* ------------------------------------------------------------------------------------------- */
static const uint16_t PA6[] = { 0, 256, 512, 768, 1024, 1536, 2048, 2560, 2992, 3072, 3584, 3888, 4095 };
static const uint32_t SPEED[] = { 0, 499, 500, 501, 1500, 2400 };
static const uint8_t LEVELS[] = { 0, 1, 3, 5 };          /* OFF, low, middle, high */
static const uint16_t LOADS[] = { 30, 70, 600, 2000 };
#define N(a) (sizeof(a) / sizeof((a)[0]))

static void t2_no_rider_matrix(void)
{
	puts("T2 no-rider matrix: PA6 x speed x level x PAS pattern, load 0");
	uint32_t runs = 0, bad = 0, bad_rise = 0;
	for (size_t p = 0; p < N(PA6); p++)
	for (size_t s = 0; s < N(SPEED); s++)
	for (size_t l = 0; l < N(LEVELS); l++)
	for (int pat = 0; pat < PAT_N; pat++) {
		const run_t r = scenario(PA6[p], SPEED[s], LEVELS[l], 0U, pat);
		runs++;
		if (r.positive_ticks) {
			if (bad++ < 5) printf("  FIRST-FAIL PA6=%u speed=%u level=%u pattern=%s maxIq=%d\n",
				PA6[p], SPEED[s], LEVELS[l], PAT_NAME[pat], r.max_iq);
		}
		if (r.nonforward_rise) bad_rise++;
	}
	printf("  runs=%u positive-Iq runs=%u non-forward-rise runs=%u\n", runs, bad, bad_rise);
	CHECK(bad == 0, "T2: load 0 and no cadence never create positive Iq, for any PA6/speed/level/PAS pattern");
	CHECK(bad_rise == 0, "T2: Iq never rises while forward_valid is false");
}

static void t3_loaded_matrix(void)
{
	puts("T3 loaded matrix: PA6 independence, no rise without forward, OFF has no Iq");
	uint32_t runs = 0, pa6_dep = 0, rise = 0, off = 0, positive = 0;
	static const uint16_t PA6_LOADED[] = { 2048, 3888 };
	static const uint32_t SPEED_LOADED[] = { 0, 500, 1500, 2400 };
	for (size_t s = 0; s < N(SPEED_LOADED); s++)
	for (size_t l = 0; l < N(LEVELS); l++)
	for (size_t d = 0; d < N(LOADS); d++)
	for (int pat = 0; pat < PAT_N; pat++) {
		const run_t ref = scenario(0U, SPEED_LOADED[s], LEVELS[l], LOADS[d], pat);
		runs++;
		if (ref.positive_ticks) positive++;
		if (ref.nonforward_rise) rise++;
		if (ref.off_positive) off++;
		for (size_t p = 0; p < N(PA6_LOADED); p++) {
			const run_t r = scenario(PA6_LOADED[p], SPEED_LOADED[s], LEVELS[l], LOADS[d], pat);
			runs++;
			if (r.hash != ref.hash || r.max_iq != ref.max_iq) {
				if (pa6_dep++ < 5) printf("  FIRST-FAIL PA6 changed Iq: PA6=%u speed=%u level=%u load=%u pattern=%s\n",
					PA6_LOADED[p], SPEED_LOADED[s], LEVELS[l], LOADS[d], PAT_NAME[pat]);
			}
			if (r.nonforward_rise) rise++;
			if (r.off_positive) off++;
		}
	}
	printf("  runs=%u runs-with-assist=%u PA6-dependent=%u non-forward-rise=%u OFF-positive=%u\n",
		runs, positive, pa6_dep, rise, off);
	CHECK(positive > 0, "T3: the matrix does exercise real pedal assist (not a vacuous pass)");
	CHECK(pa6_dep == 0, "T3: the Iq trajectory is identical for every PA6 value - PA6 owns no demand");
	CHECK(rise == 0, "T3: Iq never rises while forward_valid is false");
	CHECK(off == 0, "T3: assist OFF never carries Iq");
}

static void t4_veto_and_dip(void)
{
	puts("T4 PA6DIP + hidden demand under real_stop");
	static const uint16_t DIP[] = { 1536, 2048, 3072 };
	for (size_t i = 0; i < N(DIP); i++) {
		run_t r; memset(&r, 0, sizeof(r));
		power_on();
		sim_in_t in = base(); in.level_index = 1U; in.speed_x100 = 1500U; in.pa6 = DIP[i];
		for (int k = 0; k < 26; k++) { one_edge(&in, true); run_for(&in, SEC(0.3), &r); }
		in.pa6 = 0U;
		for (int k = 0; k < 4; k++) { one_edge(&in, true); run_for(&in, SEC(0.3), &r); }
		in.pa6 = DIP[i];
		for (int k = 0; k < 30; k++) { one_edge(&in, true); run_for(&in, SEC(0.3), &r); }
		CHECK(r.positive_ticks == 0, "T4: PA6 present from boot, dipping to 0 and back, arms nothing");
	}
	/* PA6 high for seconds under real_stop with idle/glitch PAS, then the veto clears. */
	for (int with_pedal = 0; with_pedal < 2; with_pedal++) {
		run_t r; memset(&r, 0, sizeof(r)); r.hash = 2166136261U;
		power_on();
		sim_in_t in = base(); in.level_index = 5U; in.speed_x100 = 2400U; in.pa6 = 3072U;
		run_for(&in, SEC(4.0), &r);                                   /* veto, idle */
		one_edge(&in, true); run_for(&in, 2U, &r); one_edge(&in, false);  /* glitch */
		run_for(&in, SEC(3.0), &r);
		const uint32_t before = r.positive_ticks;
		int32_t m2aa_max = 0;
		/* the veto clears: two forward edges, no load */
		one_edge(&in, true); run_for(&in, SEC(0.05), &r); one_edge(&in, true);
		for (uint32_t k = 0; k < SEC(0.3); k++) {
			tick(&in); observe(&r, &in);
			const int32_t m = assist_pipeline_g53()->trace.m2aa;
			if (m > m2aa_max && m < 32768) m2aa_max = m;
		}
		CHECK(before == 0 && r.max_iq == 0 && m2aa_max == 0,
			"T4: removing real_stop releases no preloaded PA6 demand (M2AA and Iq stay 0)");
		if (with_pedal) {
			/* valid pedal demand after the veto starts from zero through the pedal model */
			in.load_ctrl = 2000U; in.pas_force = -1; in.crank_rpm = 70;
			for (uint32_t k = 0; k < SEC(3.0); k++) tick(&in);
			printf("  after-veto pedal start: Iq=%d\n", S.iq);
			CHECK(S.iq > 0, "T4: a valid pedal demand after the veto still produces assist");
		}
	}
}

/* Positive steady pedal assist at `level`, 15 km/h. */
static void establish_assist(sim_in_t *in, uint8_t level)
{
	power_on();
	*in = base(); in->level_index = level; in->speed_x100 = 1500U;
	for (uint32_t k = 0; k < SEC(2.0); k++) tick(in);
	in->load_ctrl = 2000U; in->pas_force = -1; in->crank_rpm = 70;
	for (uint32_t k = 0; k < SEC(3.0); k++) tick(in);
}

/* TASK-EVD-TQ-06-G1 (review F-11): the hard battery-overcurrent trip is an exact-zero owner like
 * the comms inhibit - same update, FORCE_ZERO, ceiling 0, ahead of Walk and calibration - and its
 * release at standstill hands back no step of demand. */
static void t5b_battery_trip(void)
{
	puts("T5b hard battery-overcurrent trip owner");
	sim_in_t in;
	establish_assist(&in, 3U);
	CHECK(S.iq > 0 && mb->target > 0, "T5b: positive assist before the trip");
	in.battery_trip = true;
	tick(&in);
	CHECK(mb->target == 0 && mb->mode == (uint32_t)FIS_MODE_FORCE_ZERO && mb->iq_ceiling == 0 &&
	      S.iq == 0 && ride_control_final_iq_requested() == 0,
		"T5b: trip -> target 0, FORCE_ZERO, ceiling 0 and MS.i_q_setpoint 0 in the same update");
	walk_iq_stub = 120U; in.walk = true;
	cal_iq_stub = 100U; in.calibration = true; cal_calls = 0U;
	bool owners_zero = true;
	for (uint32_t k = 0; k < SEC(0.5); k++) {
		tick(&in);
		if (S.iq != 0 || mb->target != 0 || mb->iq_ceiling != 0) owners_zero = false;
	}
	CHECK(owners_zero && cal_calls == 0U, "T5b: neither Walk nor position calibration can publish Iq while the trip is latched");
	in.walk = false; walk_iq_stub = 0U; in.calibration = false; cal_iq_stub = 0U;
	in.battery_trip = false;
	bool no_step = true;
	for (uint32_t k = 0; k < SEC(1.0); k++) { tick(&in); if (S.iq != 0) no_step = false; }
	CHECK(no_step, "T5b: re-arming mid-pedal releases no step of demand");
}

/* TASK-EVD-TQ-06-G1 step 2 through ride_control_update() (reviews S2-01, S2-02):
 * - the SOC derate reaches the G53 limit: full derate -> limit = 50 % knee of battery_current_max;
 * - Walk keeps the M820 Iq thermal derate: at 95 degC its request is 0, at 25 degC it is not. */
static void t5c_step2_paths(void)
{
	puts("T5c step 2: SOC derate pass-through, Walk thermal derate");
	sim_in_t in;
	establish_assist(&in, 3U);
	in.soc_derate = 0x1000;
	for (uint32_t k = 0; k < 50U; k++) tick(&in);
	CHECK(g53_port_g1_state()->limit == (uint16_t)((BATTERYCURRENT_MAX / 10) * G53_G1_SOC_KNEE_PCT / 100),
		"T5c: full SOC derate through ride_control -> G53 limit at the 50 % knee");
	in.soc_derate = 0;
	for (uint32_t k = 0; k < 50U; k++) tick(&in);
	CHECK(g53_port_g1_state()->limit == (uint16_t)(BATTERYCURRENT_MAX / 10),
		"T5c: no SOC derate -> G53 limit back at the configured limit");
	in.load_ctrl = 0U; in.crank_rpm = 0; in.pas_force = 0;
	walk_iq_stub = 120U; in.walk = true; in.temp_c = 25;
	bool walk_cool = false;
	for (uint32_t k = 0; k < SEC(0.5); k++) { tick(&in); if (S.iq > 0) walk_cool = true; }
	CHECK(walk_cool, "T5c: Walk pulls at 25 degC");
	in.temp_c = 95;
	bool walk_hot_zero = true;
	for (uint32_t k = 0; k < SEC(0.5); k++) { tick(&in); if (k > 10U && mb->target != 0) walk_hot_zero = false; }
	CHECK(walk_hot_zero, "T5c: Walk at 95 degC is taken to zero by the M820 Iq thermal derate");
	in.walk = false; walk_iq_stub = 0U; in.temp_c = 0;
}

static void t5_hard_inhibits(void)
{
	puts("T5 hard inhibits: comm_inhibit / direction / brake / torque fault");
	sim_in_t in;
	/* comm_inhibit during real positive assist: exact zero in the SAME update. */
	establish_assist(&in, 3U);
	CHECK(S.iq > 0 && mb->target > 0, "T5: positive assist before the comms cut");
	in.comm_inhibit = true;
	tick(&in);
	CHECK(mb->target == 0 && mb->mode == (uint32_t)FIS_MODE_FORCE_ZERO && mb->iq_ceiling == 0 &&
	      fast_iq_slew_current_mode() == FIS_MODE_FORCE_ZERO && S.iq == 0 &&
	      ride_control_final_iq_requested() == 0,
		"T5: comm cut -> target 0, FORCE_ZERO, ceiling 0 and MS.i_q_setpoint 0 in the same update");
	bool held = true;
	for (uint32_t k = 0; k < SEC(3.0); k++) {
		tick(&in);
		if (S.iq != 0 || mb->target != 0 || mb->mode != (uint32_t)FIS_MODE_FORCE_ZERO) held = false;
	}
	CHECK(held, "T5: nothing later overwrites the zero while the inhibit lasts, even with pedalling");
	/* Walk and position calibration that WOULD pull, under the inhibit */
	walk_iq_stub = 120U; in.walk = true;
	cal_iq_stub = 100U; in.calibration = true; cal_calls = 0U;
	bool owners_zero = true;
	for (uint32_t k = 0; k < SEC(0.5); k++) {
		tick(&in);
		if (S.iq != 0 || mb->target != 0 || mb->iq_ceiling != 0) owners_zero = false;
	}
	CHECK(owners_zero, "T5: neither Walk nor position calibration can publish Iq under the comms inhibit");
	CHECK(cal_calls == 0U,
		"T5: the calibration owner is not even consulted (its phase-2 step and EEPROM write cannot run)");
	in.walk = false; walk_iq_stub = 0U; in.calibration = false; cal_iq_stub = 0U;
	/* recovery: the level was never touched; pedal input resumes assist, none keeps it zero.
	 * The inhibit reset the G53 chain on purpose - no demand built during the loss may be
	 * released as a step when the bus returns - so, exactly as after a cold boot or a Walk
	 * exit, G53 re-arms its start window on the next pedal start from stopped cranks. */
	in.comm_inhibit = false;
	bool no_step = true;
	for (uint32_t k = 0; k < SEC(1.0); k++) { tick(&in); if (S.iq != 0) no_step = false; }
	CHECK(no_step, "T5: the bus returning mid-pedal releases no step of demand");
	in.load_ctrl = 0U; in.crank_rpm = 0; in.pas_force = (int8_t)PAS_FWD[S.phase];
	for (uint32_t k = 0; k < SEC(1.0); k++) tick(&in);
	in.load_ctrl = 2000U; in.pas_force = -1; in.crank_rpm = 70;
	int32_t rec_max = 0;
	for (uint32_t k = 0; k < SEC(3.0); k++) { tick(&in); if (S.iq > rec_max) rec_max = S.iq; }
	CHECK(rec_max > 0, "T5: after the inhibit clears, valid pedalling resumes assist at the same level");
	in.load_ctrl = 0U; in.crank_rpm = 0; in.pas_force = (int8_t)PAS_FWD[S.phase];
	for (uint32_t k = 0; k < SEC(2.0); k++) tick(&in);
	bool zero = true;
	for (uint32_t k = 0; k < SEC(3.0); k++) { tick(&in); if (S.iq != 0) zero = false; }
	CHECK(zero, "T5: without pedal input Iq stays exactly zero after recovery");

	/* comm cut while Walk alone owns the motor, and while calibration alone owns it */
	for (int owner = 0; owner < 2; owner++) {
		power_on();
		in = base();
		if (owner == 0) { in.walk = true; walk_iq_stub = 120U; }
		else { in.calibration = true; cal_iq_stub = 100U; }
		for (uint32_t k = 0; k < SEC(1.0); k++) tick(&in);
		CHECK(S.iq > 0, "T5: the service/Walk owner pulls before the cut (fixture sanity)");
		in.comm_inhibit = true; tick(&in);
		CHECK(S.iq == 0 && mb->target == 0 && mb->mode == (uint32_t)FIS_MODE_FORCE_ZERO &&
		      mb->iq_ceiling == 0,
			owner == 0 ? "T5: comm cut during Walk is exact zero in the same update"
			           : "T5: comm cut during position calibration is exact zero in the same update");
		walk_iq_stub = 0U; cal_iq_stub = 0U;
	}

	/* direction / brake / torque fault from positive assist */
	static const char *const NAME[3] = { "direction", "brake", "torque fault" };
	for (int kind = 0; kind < 3; kind++) {
		establish_assist(&in, 5U);
		const int32_t at = S.iq;
		if (kind == 0) in.crank_rpm = -60;
		else if (kind == 1) in.brake = true;
		else in.torque_valid = false;
		bool armed = (kind != 0), req_zero = false, mono = true, exact_zero = false;
		int32_t max_rise = 0;
		int32_t prev = at;
		uint32_t zero_at = 0, armed_at = 0;
		for (uint32_t k = 0; k < SEC(0.5); k++) {
			tick(&in);
			if (!armed) {
				if (!pas_direction_direction_inhibit_active()) { prev = S.iq; continue; }
				armed = true; armed_at = k;
			}
			if (!req_zero) req_zero = (mb->target == 0);
			if (S.iq > prev) mono = false;
			if (S.iq - prev > max_rise) max_rise = S.iq - prev;
			prev = S.iq;
			if (S.iq == 0 && !exact_zero) { exact_zero = true; zero_at = k - armed_at; }
		}
		printf("  %s: Iq %d -> 0 after %u control ticks\n", NAME[kind], at, zero_at);
		if (kind == 0) {
			/* OWNER-DEC-2026-10-06-G5300-ONLY: physical reverse is consumed by D7EC/BDE8.
			 * Its request decays through G53; the native 200 ms SAFETY timer does not own it. */
			printf("    G53 reverse: request_zero=%u monotonic=%u exact_zero=%u max_rise=%d/tick\n",
				req_zero ? 1U : 0U, mono ? 1U : 0U, exact_zero ? 1U : 0U, (int)max_rise);
			/* The already-running 16 kHz rise can advance by two Iq counts in one 4 kHz
			 * tick (438 Q8 at P=700); the physical PAS event reaches G53 on the next tick. */
			CHECK(at > 0 && req_zero && exact_zero && zero_at > 0 && max_rise <= 2,
				"T5: moving reverse reaches zero through G53 without an upward jump");
		} else {
			CHECK(at > 0 && req_zero && mono && exact_zero && zero_at <= 3200U / 4U + 4U,
				"T5: native brake/torque cut requests zero at once, SAFETY releases within 200 ms");
		}
	}
}

static void t6_release_and_off(void)
{
	puts("T6 ordinary pedal release preserved; assist OFF is same-update exact zero");
	sim_in_t in;
	establish_assist(&in, 3U);
	for (uint32_t k = 0; k < SEC(1.0); k++) tick(&in);
	const int32_t at = S.iq;
	in.crank_rpm = 0; in.pas_force = (int8_t)PAS_FWD[S.phase];
	bool mono = true, zero = false;
	int32_t prev = at;
	uint32_t k0 = 0;
	for (uint32_t k = 0; k < SEC(2.0); k++) {
		tick(&in);
		if (S.iq > prev) mono = false;
		prev = S.iq;
		if (!zero && S.iq == 0) { zero = true; k0 = k; }
	}
	printf("  release: Iq %d -> 0 in %u ms\n", at, k0 / 4U);
	CHECK(at > 0 && mono && zero, "T6: a valid pedal demand decays monotonically to exact zero on stop");

	for (uint8_t lv = 1; lv <= 5; lv++) {
		establish_assist(&in, lv);
		const int32_t on = S.iq;
		in.level_index = 0U;
		tick(&in);
		CHECK(on > 0 && mb->target == 0 && S.iq == 0 && ride_control_final_iq_requested() == 0,
			"T6: switching the level OFF zeroes the request and the reference in the same update");
		bool stays = true;
		for (uint32_t k = 0; k < SEC(2.0); k++) { tick(&in); if (S.iq != 0 || mb->target != 0) stays = false; }
		CHECK(stays, "T6: and OFF creates no new Iq while the rider keeps pedalling");
	}
}

/* ------------------------------------------------------------------------------------------- */
/* T7: contract §10b. Bank edits go through the production wire format and CRC.               */
static uint16_t crc16(const uint8_t *b, uint16_t n)
{
	uint16_t c = 0xFFFFU;
	for (uint16_t i = 0; i < n; i++) {
		c ^= (uint16_t)b[i] << 8;
		for (int k = 0; k < 8; k++) c = (c & 0x8000U) ? (uint16_t)((c << 1) ^ 0x1021U) : (uint16_t)(c << 1);
	}
	return c;
}
static bool set_level_limits(uint8_t iq_pct, uint16_t power_w)
{
	uint8_t blob[ASSIST_BANK_BLOB_LEN];
	const uint16_t len = assist_modes_serialize_bank(0U, blob);
	if (len != ASSIST_BANK_BLOB_LEN) return false;
	const uint16_t hdr = 13U, rec = blob[5];
	for (int lv = 0; lv < 5; lv++) {
		uint8_t *r = &blob[hdr + lv * rec];
		r[15] = (uint8_t)(power_w & 0xFFU); r[16] = (uint8_t)(power_w >> 8);
		r[17] = iq_pct;
	}
	const uint16_t c = crc16(blob, (uint16_t)(len - 2U));
	blob[len - 2U] = (uint8_t)(c & 0xFFU); blob[len - 1U] = (uint8_t)(c >> 8);
	if (!assist_modes_apply_bank_blob(blob, len)) return false;
	for (int lv = 1; lv <= 5; lv++) {
		const assist_level_config_t *cfg = assist_modes_get_default_level((uint8_t)lv);
		if (cfg->max_iq_pct != iq_pct || cfg->max_motor_power_w != power_w) return false;
	}
	return true;
}

static run_t hard_ride(uint8_t level)
{
	run_t r; memset(&r, 0, sizeof(r)); r.hash = 2166136261U;
	sim_in_t in = base(); in.level_index = level; in.speed_x100 = 1500U;
	run_for(&in, SEC(2.0), &r);
	in.load_ctrl = 6000U; in.pas_force = -1; in.crank_rpm = 90;
	run_for(&in, SEC(4.0), &r);
	return r;
}

static char *load_text(const char *path)
{
	FILE *f = fopen(path, "rb");
	if (!f) return NULL;
	fseek(f, 0, SEEK_END);
	const long n = ftell(f);
	fseek(f, 0, SEEK_SET);
	char *t = (char *)malloc((size_t)n + 1U);
	if (!t) { fclose(f); return NULL; }
	const size_t got = fread(t, 1, (size_t)n, f);
	t[got] = '\0';
	fclose(f);
	return t;
}

static void t7_contract_10b(void)
{
	puts("T7 contract 10b: G53 normal path has no M820 behavioural level/power ceiling");
	char *src = load_text(STRINGIZE(ASSIST_PIPELINE_C_PATH));
	CHECK(src != NULL, "T7: assist_pipeline.c readable");
	CHECK(src && strstr(src, "lim_in.max_power_w=0;") != NULL,
		"T7: G53 NORMAL passes max_power_w == 0 to ap2_limits");
	CHECK(src && strstr(src, "lim_in.level_iq_limit=AP2_LIMITS_NO_LEVEL_CEILING;") != NULL,
		"T7: G53 NORMAL passes level_iq_limit == AP2_LIMITS_NO_LEVEL_CEILING to ap2_limits");
	free(src);
	/* Behaviour: a bank with 20 % / 100 W on every level produces the identical Iq trajectory -
	 * the ceilings are REPLACE_WITH_G53, while the hardware protections stay active. */
	for (uint8_t lv = 1; lv <= 5; lv += 2) {
		power_on();
		const run_t ref = hard_ride(lv);
		power_on();
		CHECK(set_level_limits(20U, 100U), "T7: 20 %/100 W bank applied through the wire format");
		const run_t lim = hard_ride(lv);
		printf("  level %u: default bank max Iq=%d, 20 %%/100 W bank max Iq=%d\n", lv, ref.max_iq, lim.max_iq);
		CHECK(ref.max_iq > 140 || lv == 1, "T7: fixture drives beyond a 20 % ceiling on the upper levels");
		CHECK(ref.hash == lim.hash && ref.max_iq == lim.max_iq,
			"T7: stored max_iq_pct/max_motor_power_w do not change the G53 normal path (contract 10b)");
		CHECK(lim.max_iq <= PH_CURRENT_MAX, "T7: the hardware phase-current ceiling still binds");
	}
}

int main(void)
{
	puts("M820 uncontrolled-Iq safety regression (production ride_control -> G53 -> limits -> FIS)");
	puts("THROTTLE DISABLED FOR SAFETY BENCH TEST - NOT FINAL THROTTLE ARCHITECTURE");
	t1_hidden_throttle();
	t2_no_rider_matrix();
	t3_loaded_matrix();
	t4_veto_and_dip();
	t5_hard_inhibits();
	t5b_battery_trip();
	t5c_step2_paths();
	t6_release_and_off();
	t7_contract_10b();
	if (host_test_failures == 0) {
		puts("M820 uncontrolled-Iq safety regression: ALL CHECKS PASSED");
		return 0;
	}
	printf("M820 uncontrolled-Iq safety regression: %d CHECK(S) FAILED\n", host_test_failures);
	return 1;
}
