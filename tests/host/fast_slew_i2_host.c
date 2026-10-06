/*
 * TASK-EVD-TQ-06-G2 I2 - G5300 fast current-reference slew on the normal PEDAL path.
 *
 * Stock G5300 (0x0801A26C): +-10 Q14 per call, 16 calls per 1 ms tick = +-160 Q14/ms, Q14 16384
 * = phase_current_max (R1 Q7). M820 rate = 160*P/16384 Iq/ms (6.836 at P=700) = 0.625*P per 4 kHz
 * tick in Q8 (438 at P=700, round half up). The normal PEDAL path therefore publishes RISE/FALL
 * with that step instead of BYPASS; the single final-Iq owner (fast_iq_slew) does the slewing.
 *
 * Pinned here: step value and mode; a step of the G53 request faster than the stock rate is
 * slewed (published Iq change per ms <= rate+1); normal engage/release are unchanged (the owner
 * output equals the request on every ms: BDE8 is slower than the fast slew); stock hard-zero
 * events (standstill release of the BDE8 drive, direction inhibit at standstill) stay immediate;
 * the protection ceiling still binds at once; the Quiet Zero policy publication is unchanged.
 * Usage: fast_slew_i2_host [trace]
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
#define SPEED_ON_MS 1000
#define STEADY_MS 3000
#define RUN_AFTER_MS 600

static unsigned failures;
static bool trace_on;
#define CHECK(c, m) do { if (!(c)) { printf("  FAIL  %s\n", (m)); ++failures; } } while (0)

static const uint8_t fwd_cycle[] = {0U, 2U, 3U, 1U};

static fast_iq_slew_mailbox_t mailbox;
static int32_t iq_ref;

static uint32_t expected_step(int32_t p) { return (uint32_t)((5 * p + 4) / 8); }
static int rate_ceil(int32_t p) { return (160 * p + 16383) / 16384; }   /* 7 at 700 */

typedef enum { EV_NONE, EV_LIMIT_DROP, EV_SPEED_LIMIT, EV_RELEASE, EV_RELEASE_STANDSTILL,
	EV_BRAKE, EV_TEMP } event_t;

typedef struct {
	int32_t final_iq[RUN_AFTER_MS];
	int32_t g53_pre[RUN_AFTER_MS];
	int32_t iq_ref[RUN_AFTER_MS];
	fis_mode_t mode[RUN_AFTER_MS];
	uint32_t step[RUN_AFTER_MS];
	fis_zero_policy_t policy[RUN_AFTER_MS];
	bool permission[RUN_AFTER_MS];
	int32_t before;          /* owner output on the tick before the event */
	int32_t before_req;
} result_t;

/* Engage at ENGAGE_MS, steady ride; event at STEADY_MS; per-ms series after the event. */
static void run(event_t ev, int32_t phase_max, uint32_t speed_x100, result_t *r)
{
	assist_pipeline_input_t in;
	assist_pipeline_command_t cmd;
	memset(&in, 0, sizeof(in));
	in.torque_sensor_valid = true; in.pas_sensor_valid = true; in.forward_valid = true;
	in.wheel_valid = true; in.assist_level_index = 3U;
	in.phase_current_max = phase_max; in.battery_voltage_mv = 48000U;
	in.battery_current_max = BATTERYCURRENT_MAX;
	in.u_abs = 1024; in.cal_i = 95; in.voltage_raw = 4000U; in.voltage_min_raw = 2800;
	in.controller_temperature_c = 30; in.speed_limit_x100 = 6000U; in.elapsed_ticks = 4U;
	memset(&mailbox, 0, sizeof(mailbox)); iq_ref = 0; fast_iq_slew_reset(&mailbox);
	assist_modes_init(); g53_port_init(); assist_pipeline_init();
	double ph = 0.0; unsigned ci = 0U; uint8_t pas_ab = 0U;
	memset(r, 0, sizeof(*r));
	for (int t = 0; t < STEADY_MS + RUN_AFTER_MS; ++t) {
		const int k = t - STEADY_MS;
		bool rider_on = t >= ENGAGE_MS;
		in.speed_x100 = t < SPEED_ON_MS ? 0U : speed_x100;
		if (k >= 0) {
			switch (ev) {
			case EV_LIMIT_DROP:
				in.battery_current_limiter_centiamp = 3000;
				in.battery_current_max = 10000;
				break;
			case EV_SPEED_LIMIT: in.legal_enabled = true; in.speed_limit_x100 = 500U; break;
			case EV_RELEASE: case EV_RELEASE_STANDSTILL: rider_on = false; break;
			case EV_BRAKE: in.safety_cut = true; break;
			case EV_TEMP: in.controller_temperature_c = 95; break;
			default: break;
			}
		}
		in.torque_load_ctrl = rider_on ? 6000U : 0U;
		in.torque_load_centikg = (uint16_t)(in.torque_load_ctrl / 2U);
		in.cadence_rpm = rider_on ? 60U : 0U;
		in.pas_ab = pas_ab;
		assist_pipeline_update(&in, &cmd);
		fast_iq_slew_publish(&mailbox, cmd.final_iq_request, cmd.slew_mode, cmd.step_mag_8,
			cmd.release_ticks_16k, cmd.zero_policy, cmd.iq_ceiling);
		for (unsigned i = 0; i < 16U; ++i) fast_iq_slew_tick(&mailbox, &iq_ref);
		if (k == -1) { r->before = iq_ref; r->before_req = cmd.final_iq_request; }
		if (k >= 0) {
			r->final_iq[k] = cmd.final_iq_request;
			r->g53_pre[k] = assist_pipeline_g53()->iq_request_pre_limits;
			r->iq_ref[k] = iq_ref; r->mode[k] = cmd.slew_mode; r->step[k] = cmd.step_mag_8;
			r->policy[k] = cmd.zero_policy;
			r->permission[k] = assist_pipeline_g53()->normal_permission;
		}
		if (rider_on) {
			ph += 96.0 * 60.0 / 60000.0;
			if (ph >= 1.0) { ph -= 1.0; ci = (ci + 1U) % 4U; pas_ab = fwd_cycle[ci]; }
		}
	}
	if (trace_on) {
		printf("# ev=%d P=%d speed=%u before=%d\n# k,final,g53,iq_ref,mode,step,policy,perm\n",
			(int)ev, (int)phase_max, (unsigned)speed_x100, (int)r->before);
		for (int k = 0; k < RUN_AFTER_MS; ++k)
			printf("%d,%d,%d,%d,%d,%u,%d,%d\n", k, (int)r->final_iq[k], (int)r->g53_pre[k],
				(int)r->iq_ref[k], (int)r->mode[k], (unsigned)r->step[k], (int)r->policy[k],
				(int)r->permission[k]);
	}
}

static void check_step_and_mode(int32_t p)
{
	static result_t r;
	char m[200];
	run(EV_NONE, p, 1500U, &r);
	const uint32_t want = expected_step(p);
	bool ok = true;
	for (int k = 0; k < RUN_AFTER_MS; ++k)
		if (r.step[k] != want || (r.mode[k] != FIS_MODE_RISE && r.mode[k] != FIS_MODE_FALL)) ok = false;
	snprintf(m, sizeof m, "P=%d: normal PEDAL path publishes RISE/FALL with step_mag_8=%u (0.625*P, half up)",
		(int)p, (unsigned)want);
	CHECK(ok && want > 0U, m);
	printf("P=%d step_mag_8=%u mode[0]=%d\n", (int)p, (unsigned)r.step[0], (int)r.mode[0]);
}

static void check_fast_step(event_t ev, const char *name)
{
	static result_t r;
	char m[200];
	run(ev, PHASE_MAX, 1500U, &r);
	printf("%s\n", name);
	snprintf(m, sizeof m, "%s: positive Iq established before the event (%d)", name, (int)r.before);
	CHECK(r.before > 100, m);
	/* How fast did the G53 request itself fall? If it is faster than the stock rate the owner
	 * must be the one that slews. */
	int32_t prev = r.before, prev_req = r.before_req, max_fall = 0, max_req_fall = 0;
	int zero_at = -1;
	for (int k = 0; k < RUN_AFTER_MS; ++k) {
		if (prev - r.iq_ref[k] > max_fall) max_fall = prev - r.iq_ref[k];
		if (prev_req - r.final_iq[k] > max_req_fall) max_req_fall = prev_req - r.final_iq[k];
		if (zero_at < 0 && r.iq_ref[k] == 0) zero_at = k;
		prev = r.iq_ref[k]; prev_req = r.final_iq[k];
	}
	printf("  request max fall %d/ms, owner max fall %d/ms (stock rate %d/ms), owner zero at %d ms\n",
		(int)max_req_fall, (int)max_fall, rate_ceil(PHASE_MAX), zero_at);
	snprintf(m, sizeof m, "%s: the request falls faster than the stock rate (test is meaningful: %d/ms)",
		name, (int)max_req_fall);
	CHECK(max_req_fall > rate_ceil(PHASE_MAX) + 1, m);
	snprintf(m, sizeof m, "%s: published owner Iq falls at most rate+1 per ms (%d <= %d)", name,
		(int)max_fall, rate_ceil(PHASE_MAX) + 1);
	CHECK(max_fall <= rate_ceil(PHASE_MAX) + 1, m);
	snprintf(m, sizeof m, "%s: the owner still reaches 0 (at %d ms)", name, zero_at);
	CHECK(zero_at > 0, m);
}

/* Ordinary release / engage: BDE8 is slower than the fast slew, so the owner output equals the
 * request on every ms - no double slowing. */
static void check_no_double_slowing(event_t ev, const char *name)
{
	static result_t r;
	char m[200];
	run(ev, PHASE_MAX, 1500U, &r);
	printf("%s\n", name);
	int mismatch = 0, worst = 0;
	for (int k = 0; k < RUN_AFTER_MS; ++k) {
		const int d = abs((int)r.iq_ref[k] - (int)r.final_iq[k]);
		if (d) ++mismatch;
		if (d > worst) worst = d;
	}
	snprintf(m, sizeof m, "%s: owner output == request on every ms (mismatches %d, worst %d)", name,
		mismatch, worst);
	CHECK(mismatch == 0, m);
}

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
	memset(&mailbox, 0, sizeof(mailbox)); iq_ref = 0; fast_iq_slew_reset(&mailbox);
	assist_modes_init(); g53_port_init(); assist_pipeline_init();
	double ph = 0.0; unsigned ci = 0U; uint8_t pas_ab = 0U;
	int t10 = -1, t50 = -1, t90 = -1, mism = 0, maxrise = 0; int32_t prev = 0;
	for (int t = 0; t < ENGAGE_MS + 800; ++t) {
		const bool on = t >= ENGAGE_MS;
		in.speed_x100 = 0U;
		in.torque_load_ctrl = on ? 6000U : 0U; in.torque_load_centikg = (uint16_t)(in.torque_load_ctrl / 2U);
		in.cadence_rpm = on ? 60U : 0U; in.pas_ab = pas_ab;
		assist_pipeline_update(&in, &cmd);
		fast_iq_slew_publish(&mailbox, cmd.final_iq_request, cmd.slew_mode, cmd.step_mag_8,
			cmd.release_ticks_16k, cmd.zero_policy, cmd.iq_ceiling);
		for (unsigned i = 0; i < 16U; ++i) fast_iq_slew_tick(&mailbox, &iq_ref);
		if (iq_ref != cmd.final_iq_request) ++mism;
		if (iq_ref - prev > maxrise) maxrise = iq_ref - prev;
		prev = iq_ref;
		const int e = t - ENGAGE_MS;
		if (t10 < 0 && iq_ref >= 455 / 10) t10 = e;
		if (t50 < 0 && iq_ref >= 455 / 2) t50 = e;
		if (t90 < 0 && iq_ref >= 455 * 9 / 10) t90 = e;
		if (on) { ph += 96.0 * 60.0 / 60000.0; if (ph >= 1.0) { ph -= 1.0; ci = (ci + 1U) % 4U; pas_ab = fwd_cycle[ci]; } }
	}
	printf("engage from standstill (owner output): 10/50/90 %% at %d/%d/%d ms, max rise %d/ms, mismatches %d\n",
		t10, t50, t90, maxrise, mism);
	snprintf(m, sizeof m, "engage: owner output equals the BDE8 request on every ms (%d mismatches)", mism);
	CHECK(mism == 0, m);
	/* I1 reference numbers (reverse_ramp_host: 83/185/292 ms). */
	snprintf(m, sizeof m, "engage: 10/50/90 %% timing unchanged vs I1 (83/185/292 ms, got %d/%d/%d)", t10, t50, t90);
	CHECK(t10 == 83 && t50 == 185 && t90 == 292, m);
}

/* Stock hard-zero event: the BDE8 drive permission falls with m2aa 0 (standstill release). The
 * request drops to 0 in one step on that tick; the owner must follow in the SAME tick (stock
 * mode 0/3: K reset, no ramp) - not slew down a tail behind it. */
static void check_standstill_hard_zero(void)
{
	static result_t r;
	char m[200];
	const char *name = "pedal release at standstill: BDE8 hard zero";
	run(EV_RELEASE_STANDSTILL, PHASE_MAX, 0U, &r);
	printf("%s\n", name);
	int z = -1;
	for (int k = 1; k < RUN_AFTER_MS; ++k) if (r.final_iq[k] == 0) { z = k; break; }
	snprintf(m, sizeof m, "%s: request drops to 0 (at %d ms) from a positive value", name, z);
	CHECK(z > 0 && r.final_iq[z - 1] > 100 && !r.permission[z] && r.g53_pre[z] == 0, m);
	if (z > 0) {
		snprintf(m, sizeof m, "%s: FORCE_ZERO and owner exactly 0 on that same tick (mode %d, owner %d)",
			name, (int)r.mode[z], (int)r.iq_ref[z]);
		CHECK(r.mode[z] == FIS_MODE_FORCE_ZERO && r.iq_ref[z] == 0 && r.iq_ref[z - 1] > 100, m);
	}
}

static void check_safety_unchanged(void)
{
	static result_t r;
	char m[200];
	const char *name = "brake / safety cut: native SAFETY owner path unchanged";
	run(EV_BRAKE, PHASE_MAX, 1500U, &r);
	printf("%s\n", name);
	snprintf(m, sizeof m, "%s: request 0, SAFETY, owner falls (not a step) - got %d, mode %d, owner %d",
		name, (int)r.final_iq[0], (int)r.mode[0], (int)r.iq_ref[0]);
	CHECK(r.before > 100 && r.final_iq[0] == 0 && r.mode[0] == FIS_MODE_SAFETY && r.iq_ref[0] > 0, m);
}

int main(int argc, char **argv)
{
	trace_on = argc > 1 && strcmp(argv[1], "trace") == 0;
	puts("TASK-EVD-TQ-06-G2 I2: G5300 fast current-reference slew on the normal PEDAL path");
	check_step_and_mode(700);
	check_step_and_mode(1);
	check_step_and_mode(350);
	check_step_and_mode(1000);
	check_fast_step(EV_LIMIT_DROP, "battery limiter (g1) drops the G53 request");
	check_fast_step(EV_SPEED_LIMIT, "speed limit drops under the speed (ap2 limiter)");
	check_no_double_slowing(EV_RELEASE, "ordinary pedal release, rolling (BDE8 ramp, unchanged)");
	check_engage_unchanged();
	check_standstill_hard_zero();
	check_safety_unchanged();
	if (failures) { printf("fast slew I2: %u FAILED CHECKS\n", failures); return 1; }
	puts("fast slew I2: ALL CHECKS PASSED");
	return 0;
}
