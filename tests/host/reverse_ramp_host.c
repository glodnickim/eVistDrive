/*
 * TASK-EVD-TQ-06-G2 I1 - reverse / invalid PAS step while moving: ramp, not a step.
 *
 * Drives the real production pedal path (assist_pipeline_update -> ap2_limits -> G53 chain ->
 * fast_iq_slew owner) with the P2-probe stimulus: PAS and load start together, level 3, 15 km/h,
 * 30 kg, 60 rpm; then the rider stops and the crank settles back one reverse step (and variants:
 * one INVALID step, BACKWARD_CONFIRM_STEPS reverse steps, load still held).
 *
 * Contract under test (owner decision OWNER-DEC-2026-10-05-TQ06G2-A):
 *   moving (speed_x100 >= 10), direction inhibit active -> the published request follows the G53
 *     chain's own decay, never rises, and is exactly zero no later than 200 ms after the inhibit
 *     started;
 *   standstill (speed_x100 <= 9)                          -> same-tick FORCE_ZERO (unchanged);
 *   brake / fault / real stop during the ramp             -> same-tick FORCE_ZERO (unchanged).
 * Usage: reverse_ramp_host [trace]   (trace dumps the per-ms series of every scenario)
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "assist_pipeline.h"
#include "assist_modes.h"
#include "config.h"
#include "fast_iq_slew.h"
#include "g53_port.h"

#define PHASE_MAX 700
#define ENGAGE_MS 500
#define SPEED_ON_MS 1000        /* wheel starts rolling here (engage is at ENGAGE_MS, at rest) */
#define STOP_MS 3000            /* the rider stops here; the inhibit starts on this tick */
#define RUN_AFTER_MS 400
#define RELEASE_BOUND_MS 200

static unsigned failures;
static bool trace_on;
#define CHECK(c, m) do { if (!(c)) { printf("  FAIL  %s\n", (m)); ++failures; } } while (0)

static const uint8_t fwd_cycle[] = {0U, 2U, 3U, 1U};   /* P2 probe: the engaging forward cycle */

typedef enum { STEP_REVERSE, STEP_INVALID } step_kind_t;

typedef struct {
	const char *name;
	uint32_t speed_x100;
	step_kind_t kind;
	unsigned steps;         /* number of reverse/invalid steps, 10 ms apart */
	bool load_held;         /* rider keeps the load on the pedal after the step */
	int fault_at_ms;        /* >=0: safety_cut asserted this many ms after the inhibit */
	bool real_stop_fault;   /* use real_stop instead of safety_cut */
} scn_t;

typedef struct {
	int32_t final_iq[RUN_AFTER_MS];
	int32_t g53_pre[RUN_AFTER_MS];
	int32_t iq_ref[RUN_AFTER_MS];
	int32_t g1[RUN_AFTER_MS];
	int32_t q50[RUN_AFTER_MS];
	fis_mode_t mode[RUN_AFTER_MS];
	uint32_t release_ticks[RUN_AFTER_MS];
	fis_zero_policy_t policy[RUN_AFTER_MS];
	int32_t before_stop;     /* published final request on the tick before the stop */
} result_t;

static fast_iq_slew_mailbox_t mailbox;
static int32_t iq_ref;

static void run(const scn_t *s, result_t *r)
{
	assist_pipeline_input_t in;
	assist_pipeline_command_t cmd;
	memset(&in, 0, sizeof(in));
	in.torque_sensor_valid = true; in.pas_sensor_valid = true; in.forward_valid = true;
	in.wheel_valid = true;
	in.assist_level_index = 3U;
	in.phase_current_max = PHASE_MAX; in.battery_voltage_mv = 48000U;
	in.battery_current_max = BATTERYCURRENT_MAX;
	in.u_abs = 1024; in.cal_i = 95; in.voltage_raw = 4000U; in.voltage_min_raw = 2800;
	in.controller_temperature_c = 30; in.speed_limit_x100 = 6000U; in.elapsed_ticks = 4U;
	memset(&mailbox, 0, sizeof(mailbox)); iq_ref = 0;
	fast_iq_slew_reset(&mailbox);
	assist_modes_init(); g53_port_init(); assist_pipeline_init();

	double ph = 0.0; unsigned ci = 0U; uint8_t pas_ab = 0U;
	unsigned steps_done = 0U;
	memset(r, 0, sizeof(*r));
	for (int t = 0; t < STOP_MS + RUN_AFTER_MS; ++t) {
		const bool rider_on = t >= ENGAGE_MS && t < STOP_MS;
		const int k = t - STOP_MS;                       /* ms since the stop */
		/* The wheel is still at rest while the rider engages (P2: engage with PAS and load
		 * together), then rolls at the scenario speed. Starting the ride already rolling right
		 * after a pipeline reset is a different, G53-internal case (see the report). */
		in.speed_x100 = t < SPEED_ON_MS ? 0U : s->speed_x100;
		if (k >= 0) {
			/* The rider's feet leave the pedals: load goes to 0 (unless held) and the crank
			 * settles back: steps at k = 0, 10, 20, ... */
			in.torque_load_ctrl = s->load_held ? 6000U : 0U;
			if (steps_done < s->steps && k == (int)(steps_done * 10U)) {
				if (s->kind == STEP_REVERSE) {
					ci = (ci + 3U) % 4U;               /* one state back */
				} else {
					ci = (ci + 2U) % 4U;               /* illegal: two states at once */
				}
				pas_ab = fwd_cycle[ci];
				++steps_done;
			}
			if (steps_done > 0U) {
				in.direction_inhibit = true;
				in.inhibit_is_reverse = s->kind == STEP_REVERSE;
				in.forward_valid = false;
			}
			if (s->fault_at_ms >= 0 && k >= s->fault_at_ms) {
				if (s->real_stop_fault) in.real_stop = true; else in.safety_cut = true;
			}
		} else {
			in.torque_load_ctrl = rider_on ? 6000U : 0U;
		}
		in.torque_load_centikg = (uint16_t)(in.torque_load_ctrl / 2U);
		in.cadence_rpm = (rider_on) ? 60U : 0U;
		in.pas_ab = pas_ab;
		assist_pipeline_update(&in, &cmd);
		fast_iq_slew_publish(&mailbox, cmd.final_iq_request, cmd.slew_mode, cmd.step_mag_8,
			cmd.release_ticks_16k, cmd.zero_policy, cmd.iq_ceiling);
		for (unsigned i = 0; i < 4U; ++i) fast_iq_slew_tick(&mailbox, &iq_ref);
		in.live_iq_ref = iq_ref; in.live_iq_valid = true;   /* as ride_control passes MS.i_q_setpoint */
		if (k == -1) r->before_stop = cmd.final_iq_request;
		if (k >= 0) {
			r->final_iq[k] = cmd.final_iq_request;
			r->g53_pre[k] = assist_pipeline_g53()->iq_request_pre_limits;
			r->iq_ref[k] = iq_ref;
			r->g1[k] = assist_pipeline_g53()->trace.g1;
			r->q50[k] = assist_pipeline_g53()->trace.bde8_q50;
			r->mode[k] = cmd.slew_mode;
			r->release_ticks[k] = cmd.release_ticks_16k;
			r->policy[k] = cmd.zero_policy;
		}
		if (rider_on) {
			ph += 96.0 * 60.0 / 60000.0;
			if (ph >= 1.0) { ph -= 1.0; ci = (ci + 1U) % 4U; pas_ab = fwd_cycle[ci]; }
		}
	}
	if (trace_on) {
		printf("# %s speed=%u before_stop=%d\n# k,final,g53_pre,iq_ref,mode,policy,g1,q50\n", s->name,
			(unsigned)s->speed_x100, (int)r->before_stop);
		for (int k = 0; k < RUN_AFTER_MS; ++k)
			printf("%d,%d,%d,%d,%d,%d,%d,%d\n", k, (int)r->final_iq[k], (int)r->g53_pre[k],
				(int)r->iq_ref[k], (int)r->mode[k], (int)r->policy[k], (int)r->g1[k], (int)r->q50[k]);
	}
}

static int first_zero(const int32_t *v)
{
	for (int k = 0; k < RUN_AFTER_MS; ++k) if (v[k] == 0) return k;
	return -1;
}

static void check_moving_ramp(const scn_t *s)
{
	static result_t r;
	char m[200];
	run(s, &r);
	printf("%s\n", s->name);
	snprintf(m, sizeof m, "%s: positive demand established before the stop", s->name);
	CHECK(r.before_stop > 100, m);

	/* Not a step: the first tick after the inhibit still publishes a demand. */
	snprintf(m, sizeof m, "%s: first inhibit tick is not a step to zero (final=%d, before=%d)",
		s->name, (int)r.final_iq[0], (int)r.before_stop);
	CHECK(r.final_iq[0] > 0, m);

	int32_t prev = r.before_stop; int32_t max_fall = 0, max_g53_fall = 0, g53_prev = r.before_stop;
	bool rose = false;
	for (int k = 0; k < RUN_AFTER_MS; ++k) {
		if (r.final_iq[k] > prev) rose = true;
		if (prev - r.final_iq[k] > max_fall) max_fall = prev - r.final_iq[k];
		if (g53_prev - r.g53_pre[k] > max_g53_fall) max_g53_fall = g53_prev - r.g53_pre[k];
		prev = r.final_iq[k]; g53_prev = r.g53_pre[k];
	}
	snprintf(m, sizeof m, "%s: published request never rises", s->name);
	CHECK(!rose, m);
	if (s->load_held) {
		/* The rider keeps pushing: the G53 chain itself does not decay, so the request is held
		 * (not rising) until the 200 ms hard bound forces the zero - that single step is the
		 * bound, not the chain. Everything before it must be the unchanged held value. */
		bool held = true;
		for (int k = 0; k < RELEASE_BOUND_MS - 1; ++k) if (r.final_iq[k] != r.before_stop) held = false;
		snprintf(m, sizeof m, "%s: request held at %d until the 200 ms bound", s->name, (int)r.before_stop);
		CHECK(held, m);
	} else {
		snprintf(m, sizeof m, "%s: max fall %d/ms <= G53 own fall %d/ms + 1", s->name,
			(int)max_fall, (int)max_g53_fall);
		CHECK(max_fall <= max_g53_fall + 1, m);
	}
	const int z = first_zero(r.final_iq);
	snprintf(m, sizeof m, "%s: request reaches 0 within %d ms (at %d ms)", s->name,
		RELEASE_BOUND_MS, z);
	CHECK(z >= 0 && z <= RELEASE_BOUND_MS, m);
	for (int k = z < 0 ? 0 : z; z >= 0 && k < RUN_AFTER_MS; ++k)
		if (r.final_iq[k] != 0) { CHECK(false, "request stays 0 after reaching 0"); break; }
	const int zr = first_zero(r.iq_ref);
	snprintf(m, sizeof m, "%s: owner Iq setpoint reaches 0 within %d ms (at %d ms)", s->name,
		RELEASE_BOUND_MS, zr);
	CHECK(zr >= 0 && zr <= RELEASE_BOUND_MS, m);
	printf("  first-tick final=%d before=%d zero@%d ms owner-zero@%d ms max-fall=%d g53-fall=%d\n",
		(int)r.final_iq[0], (int)r.before_stop, z, zr, (int)max_fall, (int)max_g53_fall);
}

static void check_standstill(const scn_t *s)
{
	static result_t r;
	char m[200];
	run(s, &r);
	printf("%s\n", s->name);
	snprintf(m, sizeof m, "%s: positive demand established before the stop", s->name);
	CHECK(r.before_stop > 100, m);
	snprintf(m, sizeof m, "%s: same-tick exact zero, FORCE_ZERO, owner 0", s->name);
	CHECK(r.final_iq[0] == 0 && r.mode[0] == FIS_MODE_FORCE_ZERO && r.iq_ref[0] == 0 &&
		r.policy[0] == FIS_ZERO_POLICY_QUIET, m);
	bool nz = false;
	for (int k = 0; k < RUN_AFTER_MS; ++k) if (r.final_iq[k] != 0 || r.iq_ref[k] != 0) nz = true;
	snprintf(m, sizeof m, "%s: stays zero", s->name);
	CHECK(!nz, m);
}

static void check_fault_during_ramp(const scn_t *s)
{
	static result_t r;
	char m[200];
	run(s, &r);
	printf("%s\n", s->name);
	const int f = s->fault_at_ms;
	snprintf(m, sizeof m, "%s: ramp was running before the fault (final=%d at %d ms)", s->name,
		(int)r.final_iq[f > 0 ? f - 1 : 0], f - 1);
	CHECK(f > 0 && r.final_iq[f - 1] > 0, m);
	snprintf(m, sizeof m, "%s: fault tick = same-tick FORCE_ZERO, final 0, owner 0 (as before)", s->name);
	CHECK(r.final_iq[f] == 0 && r.mode[f] == FIS_MODE_FORCE_ZERO && r.iq_ref[f] == 0, m);
}


/* ------------------------------------------------------------------------------------------- *
 * R1 (scope amendment): bumpless veto release. An M820 veto only zeroes the PUBLISHED request,
 * the G53 chain keeps its demand while the rider keeps loading the pedal. When the veto clears,
 * the request must climb back at the G5300 BDE8 rate (+50 m2aa/ms = P*50/10000 Iq/ms), never
 * step. G5300 has no such veto, so its output never diverges from the chain and never steps.
 * ------------------------------------------------------------------------------------------- */
#define RESUME_MS 700
#define RATE_IQ_PER_MS ((PHASE_MAX * 50 + 9999) / 10000)   /* 4 at 700 (3.5 exactly) */

typedef struct {
	const char *name;
	int inhibit_ms;        /* direction inhibit (one reverse step at 0, forward again at inhibit_ms) */
	int fwd_false_extra;   /* forward_valid stays false this long after the inhibit cleared */
	int real_stop_ms;      /* real_stop asserted for this long (PAS keeps running forward) */
} resume_t;

static void run_resume(const resume_t *c, int32_t *fin, int32_t *g53)
{
	assist_pipeline_input_t in;
	assist_pipeline_command_t cmd;
	memset(&in, 0, sizeof(in));
	in.torque_sensor_valid = true; in.pas_sensor_valid = true; in.forward_valid = true;
	in.wheel_valid = true; in.assist_level_index = 3U;
	in.phase_current_max = PHASE_MAX; in.battery_voltage_mv = 48000U;
	in.battery_current_max = BATTERYCURRENT_MAX;
	in.u_abs = 1024; in.cal_i = 95; in.voltage_raw = 4000U; in.voltage_min_raw = 2800;
	in.controller_temperature_c = 30; in.speed_limit_x100 = 6000U; in.elapsed_ticks = 4U;
	memset(&mailbox, 0, sizeof(mailbox)); iq_ref = 0; fast_iq_slew_reset(&mailbox);
	assist_modes_init(); g53_port_init(); assist_pipeline_init();
	double ph = 0.0; unsigned ci = 0U; uint8_t pas_ab = 0U;
	for (int t = 0; t < STOP_MS + RESUME_MS; ++t) {
		const int k = t - STOP_MS;
		in.speed_x100 = t < SPEED_ON_MS ? 0U : 1500U;
		in.torque_load_ctrl = t >= ENGAGE_MS ? 6000U : 0U;       /* the rider never lets go */
		in.torque_load_centikg = (uint16_t)(in.torque_load_ctrl / 2U);
		in.cadence_rpm = t >= ENGAGE_MS ? 60U : 0U;
		bool step_fwd = t >= ENGAGE_MS;
		if (k >= 0 && c->inhibit_ms > 0) {
			if (k == 0) { ci = (ci + 3U) % 4U; pas_ab = fwd_cycle[ci]; ph = 0.0; }
			step_fwd = k >= c->inhibit_ms;
			in.direction_inhibit = k < c->inhibit_ms + 22;     /* two forward steps to re-prove */
			in.inhibit_is_reverse = true;
			in.forward_valid = !(k < c->inhibit_ms + 22 + c->fwd_false_extra);
		}
		in.real_stop = c->real_stop_ms > 0 && k >= 0 && k < c->real_stop_ms;
		in.pas_ab = pas_ab;
		assist_pipeline_update(&in, &cmd);
		fast_iq_slew_publish(&mailbox, cmd.final_iq_request, cmd.slew_mode, cmd.step_mag_8,
			cmd.release_ticks_16k, cmd.zero_policy, cmd.iq_ceiling);
		for (unsigned i = 0; i < 4U; ++i) fast_iq_slew_tick(&mailbox, &iq_ref);
		in.live_iq_ref = iq_ref; in.live_iq_valid = true;   /* as ride_control passes MS.i_q_setpoint */
		if (k >= 0) { fin[k] = cmd.final_iq_request; g53[k] = assist_pipeline_g53()->iq_request_pre_limits; }
		if (step_fwd) {
			ph += 96.0 * 60.0 / 60000.0;
			if (ph >= 1.0) { ph -= 1.0; ci = (ci + 1U) % 4U; pas_ab = fwd_cycle[ci]; }
		}
	}
}

static void check_resume(const resume_t *c)
{
	static int32_t fin[RESUME_MS], g53[RESUME_MS];
	char m[200];
	run_resume(c, fin, g53);
	printf("%s\n", c->name);
	int32_t minv = 1 << 30, maxrise = 0; int minat = -1;
	bool over = false;
	for (int k = 0; k < RESUME_MS; ++k) {
		if (fin[k] < minv) { minv = fin[k]; minat = k; }
		if (k > 0 && fin[k] - fin[k - 1] > maxrise) maxrise = fin[k] - fin[k - 1];
		if (fin[k] > g53[k]) over = true;
	}
	snprintf(m, sizeof m, "%s: the veto really pulled the request to 0 (min %d at %d ms)", c->name, (int)minv, minat);
	CHECK(minv == 0, m);
	snprintf(m, sizeof m, "%s: G53 chain still holds demand (%d) at the end", c->name, (int)g53[RESUME_MS - 1]);
	CHECK(g53[RESUME_MS - 1] > 100, m);
	snprintf(m, sizeof m, "%s: no step on resume, max rise %d/ms <= G5300 rate %d/ms + 1", c->name,
		(int)maxrise, RATE_IQ_PER_MS);
	CHECK(maxrise <= RATE_IQ_PER_MS + 1, m);
	snprintf(m, sizeof m, "%s: request never exceeds the G53 request", c->name);
	CHECK(!over, m);
	snprintf(m, sizeof m, "%s: request rejoins the G53 request (final %d, G53 %d)", c->name,
		(int)fin[RESUME_MS - 1], (int)g53[RESUME_MS - 1]);
	CHECK(fin[RESUME_MS - 1] == g53[RESUME_MS - 1], m);
	printf("  min=%d@%dms max-rise=%d/ms end final=%d g53=%d\n", (int)minv, minat, (int)maxrise,
		(int)fin[RESUME_MS - 1], (int)g53[RESUME_MS - 1]);
}

/* R3(d): a normal engage from standstill is untouched - the published request IS the G53 request
 * on every tick and keeps the BDE8 ramp (P1/P2: about 84 / 191 / 293 ms to 10 / 50 / 90 %). */
static void check_engage_unchanged(void)
{
	assist_pipeline_input_t in;
	assist_pipeline_command_t cmd;
	char m[200];
	memset(&in, 0, sizeof(in));
	in.torque_sensor_valid = true; in.pas_sensor_valid = true; in.forward_valid = true;
	in.wheel_valid = true; in.assist_level_index = 3U;
	in.phase_current_max = PHASE_MAX; in.battery_voltage_mv = 48000U;
	in.battery_current_max = BATTERYCURRENT_MAX;
	in.u_abs = 1024; in.cal_i = 95; in.voltage_raw = 4000U; in.voltage_min_raw = 2800;
	in.controller_temperature_c = 30; in.speed_limit_x100 = 6000U; in.elapsed_ticks = 4U;
	in.speed_x100 = 1500U;
	assist_modes_init(); g53_port_init(); assist_pipeline_init();
	double ph = 0.0; unsigned ci = 0U; uint8_t pas_ab = 0U;
	int t10 = -1, t50 = -1, t90 = -1; bool equal = true; int32_t last = 0;
	for (int t = 0; t < 1500; ++t) {
		const bool on = t >= ENGAGE_MS;
		in.torque_load_ctrl = on ? 6000U : 0U; in.torque_load_centikg = (uint16_t)(in.torque_load_ctrl / 2U);
		in.cadence_rpm = on ? 60U : 0U; in.pas_ab = pas_ab;
		assist_pipeline_update(&in, &cmd);
		const int32_t g = assist_pipeline_g53()->iq_request_pre_limits;
		if (cmd.final_iq_request != g) equal = false;
		last = cmd.final_iq_request;
		const int e = t - ENGAGE_MS;
		if (t10 < 0 && last >= 455 / 10) t10 = e;
		if (t50 < 0 && last >= 455 / 2) t50 = e;
		if (t90 < 0 && last >= 455 * 9 / 10) t90 = e;
		if (on) { ph += 96.0 * 60.0 / 60000.0; if (ph >= 1.0) { ph -= 1.0; ci = (ci + 1U) % 4U; pas_ab = fwd_cycle[ci]; } }
	}
	printf("engage from standstill: 10/50/90 %% at %d/%d/%d ms after the stimulus\n", t10, t50, t90);
	snprintf(m, sizeof m, "engage: the published request equals the G53 request on every tick");
	CHECK(equal, m);
	snprintf(m, sizeof m, "engage: BDE8 ramp unchanged (P1/P2 84/191/293 ms, got %d/%d/%d)", t10, t50, t90);
	CHECK(t10 >= 0 && t10 <= 90 && t50 >= 180 && t50 <= 200 && t90 >= 285 && t90 <= 300 && last == 455, m);
}

int main(int argc, char **argv)
{
	trace_on = argc > 1 && strcmp(argv[1], "trace") == 0;
	puts("TASK-EVD-TQ-06-G2 I1: reverse/invalid PAS step while moving -> ramp, standstill -> zero");
	const scn_t moving[] = {
		{"moving 15 km/h, 1 reverse step", 1500U, STEP_REVERSE, 1U, false, -1, false},
		{"moving 15 km/h, 1 INVALID step", 1500U, STEP_INVALID, 1U, false, -1, false},
		{"moving 15 km/h, BACKWARD_CONFIRM_STEPS reverse steps", 1500U, STEP_REVERSE, BACKWARD_CONFIRM_STEPS, false, -1, false},
		{"moving 15 km/h, 7 reverse steps", 1500U, STEP_REVERSE, 7U, false, -1, false},
		{"moving 0.10 km/h (speed_x100=10, boundary), 1 reverse step", 10U, STEP_REVERSE, 1U, false, -1, false},
	};
	for (size_t i = 0; i < sizeof moving / sizeof moving[0]; ++i) check_moving_ramp(&moving[i]);
	const scn_t held = {"moving 15 km/h, 1 reverse step, load still held", 1500U, STEP_REVERSE, 1U, true, -1, false};
	check_moving_ramp(&held);
	const scn_t still[] = {
		{"standstill speed_x100=0, 1 reverse step", 0U, STEP_REVERSE, 1U, false, -1, false},
		{"standstill speed_x100=9, 1 reverse step", 9U, STEP_REVERSE, 1U, false, -1, false},
		{"standstill speed_x100=0, 1 INVALID step", 0U, STEP_INVALID, 1U, false, -1, false},
		{"standstill speed_x100=9, 1 INVALID step", 9U, STEP_INVALID, 1U, false, -1, false},
	};
	for (size_t i = 0; i < sizeof still / sizeof still[0]; ++i) check_standstill(&still[i]);
	const scn_t brake = {"brake during the ramp (+20 ms)", 1500U, STEP_REVERSE, 1U, false, 20, false};
	check_fault_during_ramp(&brake);
	const scn_t rstop = {"real_stop during the ramp (+20 ms)", 1500U, STEP_REVERSE, 1U, false, 20, true};
	check_fault_during_ramp(&rstop);

	{
		const resume_t resumes[] = {
			{"R3a: 15 km/h, load held, 1 reverse step, forward again after 250 ms (deadline zero)", 250, 0, 0},
			{"R3b: same, forward_valid stays false 30 ms after the inhibit cleared", 250, 30, 0},
			{"R3c: real_stop SAFETY release (100 ms), PAS keeps running, immediate re-press", 0, 0, 100},
		};
		for (size_t i = 0; i < sizeof resumes / sizeof resumes[0]; ++i) check_resume(&resumes[i]);
		check_engage_unchanged();
	}
	if (failures == 0U) { puts("reverse ramp: ALL CHECKS PASSED"); return 0; }
	printf("reverse ramp: %u CHECK(S) FAILED\n", failures);
	return 1;
}
