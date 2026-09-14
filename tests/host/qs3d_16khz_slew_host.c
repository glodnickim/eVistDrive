/*
 * QS-3D: 16 kHz final-Iq-slew parity proof.
 *
 * The migration moves the single final Iq slew owner from 4 kHz
 * (assist_dynamics_apply) to 16 kHz (fast_iq_slew_tick). This harness proves the two
 * produce the same PHYSICAL result by simulating the timing: the 16 kHz owner runs 4
 * ISR ticks per one 4 kHz control tick, and asserts the 4 kHz-output sequence equals
 * the 16 kHz-output sequence sampled at each 4 kHz boundary.
 *
 * It links the REAL fast_iq_slew.c (the 16 kHz owner) and the REAL assist_dynamics.c
 * (the 4 kHz owner being replaced) so the parity claim is against the shipped modules,
 * not a model.
 *
 * Scenarios (all with SAME per-control Q8 step computation as the 4 kHz owner):
 *   RISE    : 0 -> full scale, slow ramp (600 ms)
 *   FALL    : full -> 0, ordinary slope (1000 ms)
 *   RELEASE : live full state -> 0 over profile_release_ms (650 ms)
 *   Partial-hold then direction change.
 *
 * Parity rule: the 16 kHz owner uses an exact Q10 fractional accumulator that adds the
 * old per-control Q8 step once per 16 kHz tick, so after 4 consecutive fast_iq_slew_tick()
 * calls the accumulator has advanced by exactly ONE old 4 kHz step. Sampling the 16 kHz
 * output every 4th tick therefore equals the 4 kHz output at every boundary (same round-
 * half-up publication). Between boundaries the slew advances in finer 62.5 us steps - the
 * intended QS-3D improvement, not a G532 rate change.
 *
 * The harness runs BOTH owners in lockstep and asserts byte-for-byte parity of the
 * boundary-sampled Iq sequence and of the total target/release tick counts, plus the
 * intended finer intermediate 62.5 us step when a THIRD in-harness replica of the slew is
 * stepped per 16 kHz tick.
 */

#include "../common/check.h"

#include "fast_iq_slew.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STRINGIZE_INNER(x) #x
#define STRINGIZE(x) STRINGIZE_INNER(x)

#define IDX_ID_SCALE 700
#define RISE_SLOW_MS 600U
#define FALL_SLOW_MS 1000U
#define RELEASE_MS 650U
#define CTRL_TICKS_PER_MS 4U
#define FOC_TICKS_PER_CTRL 4U   /* 16 kHz / 4 kHz */

#define IQ_RAMP_Q 8U   /* matches config.h IQ_RAMP_Q_SHIFT */

/* Deterministic foreground-publication interruption injection. */
static bool hook_enabled;
/* One slot per fis_publish_stage_t. QZERO added FIS_PUBLISH_AFTER_ZERO_POLICY, so the enum is
 * ten stages wide now; sized off the enum's own last member so it cannot drift again. */
static int hook_seen[FIS_PUBLISH_AFTER_SEQ_EVEN + 1];  /* 1 = old complete command, 2 = new */
static int hook_mixed;
static int32_t hook_iq_out;

void fast_iq_slew_test_hook(
	fis_publish_stage_t stage,
	fast_iq_slew_mailbox_t *mb)
{
	if (!hook_enabled) return;
	(void)fast_iq_slew_tick(mb, &hook_iq_out);

	/* QZERO: zero_policy is part of the command generation, so it is part of what "complete"
	 * means here - a torn read that paired the new policy with the old target would show up as
	 * hook_mixed rather than passing as one of the two complete commands. */
	bool old_complete = fast_iq_slew_current_target() == 111 &&
		fast_iq_slew_current_mode() == FIS_MODE_BYPASS &&
		fast_iq_slew_current_step_mag_8() == 3U &&
		fast_iq_slew_current_release_ticks_16k() == 0U &&
		fast_iq_slew_current_zero_policy() == FIS_ZERO_POLICY_NONE;
	bool new_complete = fast_iq_slew_current_target() == 700 &&
		fast_iq_slew_current_mode() == FIS_MODE_RISE &&
		fast_iq_slew_current_step_mag_8() == 77U &&
		fast_iq_slew_current_release_ticks_16k() == 0U &&
		fast_iq_slew_current_zero_policy() == FIS_ZERO_POLICY_QUIET;

	if (old_complete) hook_seen[(unsigned)stage] = 1;
	else if (new_complete) hook_seen[(unsigned)stage] = 2;
	else hook_mixed++;
}

static char *read_source(const char *path)
{
	FILE *f = fopen(path, "rb");
	if (!f) return NULL;
	if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
	long length = ftell(f);
	if (length < 0 || fseek(f, 0, SEEK_SET) != 0) { fclose(f); return NULL; }
	char *text = (char *)malloc((size_t)length + 1U);
	if (!text) { fclose(f); return NULL; }
	if (fread(text, 1U, (size_t)length, f) != (size_t)length) {
		free(text); fclose(f); return NULL;
	}
	text[length] = '\0';
	fclose(f);
	return text;
}

static unsigned source_occurrences(const char *text, const char *needle)
{
	unsigned count = 0U;
	for (const char *p = text; (p = strstr(p, needle)) != NULL; p += strlen(needle)) count++;
	return count;
}

/* Re-implementation of the 4 kHz producer's step computation (ride_final_iq_slew_compute()
 * in ride_control.c): per-control Q8 step, i.e. ceil(scale_q / ticks_4k). The 16 kHz
 * owner divides it across the four inner ticks with its exact Q10 accumulator. */
static uint32_t ceil_div(uint32_t num, uint32_t den)
{
	return (num + den - 1U) / den;
}

static uint16_t step_for(int32_t scale, uint32_t ramp_ms)
{
	uint32_t ticks_4k = ramp_ms * CTRL_TICKS_PER_MS;
	if (ticks_4k < 1U) ticks_4k = 1U;
	uint32_t scale_q = (uint32_t)scale << IQ_RAMP_Q;
	uint32_t step = ceil_div(scale_q, ticks_4k);
	if (step < 1U) step = 1U;
	return (uint16_t)step;
}

/* Run the 16 kHz owner for 4 ISR ticks (one 4 kHz boundary); returns final tick output. */
static int32_t run_16k_quarter(fast_iq_slew_mailbox_t *mb, int32_t *out)
{
	int32_t last = 0;
	for (unsigned i = 0; i < FOC_TICKS_PER_CTRL; i++) {
		last = fast_iq_slew_tick(mb, out);
	}
	return last;
}

/*
 * THE THREE LOCKSTEP-PARITY TESTS ARE GONE, and so is the 4 kHz owner they compared against.
 *
 * They existed to prove one migration: that moving the final Iq ramp from a 4 kHz owner to
 * the 16 kHz one preserved the physical trajectory. Assist Pipeline V2 removed the 4 kHz
 * owner entirely (it had already been dead code), so a parity check now has only one side.
 * Keeping it would mean re-implementing the retired ramp inside the test - which proves that
 * the test agrees with itself, not that the firmware is right.
 *
 * What the parity tests actually protected - that a full-scale rise, fall and release each
 * take the configured time and land on exact zero - is proved directly against the surviving
 * owner by test_exact_release_ceil_sweep, test_live_normal_releases, test_live_safety_releases
 * and the new assist-trajectory suite, which drives the real producer instead of a replica.
 */
static void test_force_zero_and_bypass(void)
{
	fast_iq_slew_mailbox_t mb;
	memset(&mb, 0, sizeof(mb));
	fast_iq_slew_reset(&mb);

	uint16_t up_step = step_for(IDX_ID_SCALE, RISE_SLOW_MS);
	int32_t out = 0;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_RISE, up_step, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	for (unsigned i = 0; i < 2400 && out != IDX_ID_SCALE; i++) {
		run_16k_quarter(&mb, &out);
		fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_RISE, up_step, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	}
	CHECK(out == IDX_ID_SCALE, "QS-3D FORCE_ZERO setup");

	fast_iq_slew_publish(&mb, 0, FIS_MODE_FORCE_ZERO, 0, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	out = fast_iq_slew_tick(&mb, &out);
	CHECK(out == 0, "QS-3D FORCE_ZERO: exact same-tick zero (no residual ramp)");
	CHECK(fast_iq_slew_current_target() == 0,
		"QS-3D FORCE_ZERO: verified command target is exact zero");

	fast_iq_slew_publish(&mb, 37, FIS_MODE_BYPASS, 0, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	out = fast_iq_slew_tick(&mb, &out);
	CHECK(out == 37, "QS-3D BYPASS: Walk Assist passthrough writes target directly");
}

static void test_direction_change(void)
{
	fast_iq_slew_mailbox_t mb;
	memset(&mb, 0, sizeof(mb));
	fast_iq_slew_reset(&mb);

	uint16_t up_step = step_for(IDX_ID_SCALE, RISE_SLOW_MS);
	int32_t out = 0;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_RISE, up_step, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	for (unsigned i = 0; i < 2400 && out != IDX_ID_SCALE; i++) {
		run_16k_quarter(&mb, &out);
		fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_RISE, up_step, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	}
	int32_t before = out;
	uint16_t dn_step = step_for(IDX_ID_SCALE, FALL_SLOW_MS);
	fast_iq_slew_publish(&mb, 10, FIS_MODE_FALL, dn_step, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	int32_t after = fast_iq_slew_tick(&mb, &out);
	CHECK(after <= before && after >= 10,
		"QS-3D DIRECTION: rise->fall cannot produce a positive stale-accumulator jump");
	for (unsigned i = 0; i < 6000 && out != 10; i++) {
		run_16k_quarter(&mb, &out);
		fast_iq_slew_publish(&mb, 10, FIS_MODE_FALL, dn_step, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	}
	CHECK(out == 10, "QS-3D DIRECTION: reversed target clamps exactly");
}

static void run_fast_ticks(
	fast_iq_slew_mailbox_t *mb,
	int32_t *out,
	unsigned ticks)
{
	for (unsigned i = 0; i < ticks; i++) {
		(void)fast_iq_slew_tick(mb, out);
	}
}

static unsigned release_to_zero(
	fast_iq_slew_mailbox_t *mb,
	int32_t *out,
	fis_mode_t mode,
	uint32_t duration_ticks)
{
	fast_iq_slew_publish(mb, 0, mode, 0U, duration_ticks, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	unsigned ticks = 0U;
	while (*out > 0 && ticks <= duration_ticks + 8U) {
		(void)fast_iq_slew_tick(mb, out);
		ticks++;
	}
	return ticks;
}

static uint32_t exact_ceil_u32(uint32_t numerator, uint32_t denominator)
{
	return numerator / denominator + ((numerator % denominator) != 0U ? 1U : 0U);
}

static void test_exact_release_ceil_sweep(void)
{
	const uint32_t durations[] = { RELEASE_MS * 16U, 200U * 16U };
	const uint32_t max_live_q10 = IDX_ID_SCALE << 10;
	bool exact = true;
	uint32_t bad_state = 0U, bad_duration = 0U, bad_got = 0U, bad_expected = 0U;

	/* Exhaust every practical production state, including every fractional Q10 value. */
	for (unsigned d = 0; d < sizeof(durations) / sizeof(durations[0]); d++) {
		for (uint32_t live_q10 = 1U; live_q10 <= max_live_q10; live_q10++) {
			uint32_t got = fast_iq_slew_test_release_rate_q10(live_q10, durations[d]);
			uint32_t expected = exact_ceil_u32(live_q10, durations[d]);
			if (got != expected) {
				exact = false;
				bad_state = live_q10;
				bad_duration = durations[d];
				bad_got = got;
				bad_expected = expected;
				break;
			}
		}
		if (!exact) break;
	}
	if (!exact) {
		printf("  exact-ceil mismatch: live=%u ticks=%u got=%u expected=%u\n",
			bad_state, bad_duration, bad_got, bad_expected);
	}
	CHECK(exact,
		"R2 CEIL: every practical fractional Q10 state equals ceil(live_q10/release_ticks)");

	/* Simulate representative boundary/odd/non-divisible/large releases through the real tick. */
	const uint32_t states[] = {
		1U, 2U, 3U, 1023U, 1024U, 1025U, 3199U, 3200U, 3201U,
		10399U, 10400U, 10401U, 123457U, max_live_q10 - 1U, max_live_q10
	};
	bool simulation_ok = true;
	for (unsigned d = 0; d < sizeof(durations) / sizeof(durations[0]); d++) {
		for (unsigned s = 0; s < sizeof(states) / sizeof(states[0]); s++) {
			fast_iq_slew_mailbox_t mb;
			memset(&mb, 0, sizeof(mb));
			fast_iq_slew_reset(&mb);
			fast_iq_slew_test_set_accumulator_q10((int32_t)states[s]);
			int32_t out = (int32_t)((states[s] + 512U) >> 10);
			fis_mode_t mode = (d == 0U) ? FIS_MODE_RELEASE : FIS_MODE_SAFETY;
			fast_iq_slew_publish(&mb, 0, mode, 0U, durations[d], FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
			uint32_t used = 0U;
			while (fast_iq_slew_current_accumulator_q10() > 0 && used <= durations[d]) {
				(void)fast_iq_slew_tick(&mb, &out);
				used++;
				if (fast_iq_slew_current_accumulator_q10() < 0 || out < 0) {
					simulation_ok = false;
					break;
				}
			}
			if (fast_iq_slew_current_accumulator_q10() != 0 || out != 0 ||
				used > durations[d]) {
				simulation_ok = false;
			}
		}
	}
	CHECK(simulation_ok,
		"R2 CEIL: releases reach exact zero without negative overshoot, overflow or an extra tick");
}

static void test_mailbox_interruptions_and_retry(void)
{
	fast_iq_slew_mailbox_t mb;
	memset(&mb, 0, sizeof(mb));
	fast_iq_slew_reset(&mb);
	fast_iq_slew_publish(&mb, 111, FIS_MODE_BYPASS, 3U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &hook_iq_out);

	memset(hook_seen, 0, sizeof(hook_seen));
	hook_mixed = 0;
	hook_enabled = true;
	fast_iq_slew_publish(&mb, 700, FIS_MODE_RISE, 77U, 0U, FIS_ZERO_POLICY_QUIET, IDX_ID_SCALE);
	hook_enabled = false;

	CHECK(hook_mixed == 0,
		"J1: interruption at every producer boundary never accepts a mixed command");
	for (unsigned stage = FIS_PUBLISH_BEFORE_UPDATE;
		stage <= FIS_PUBLISH_BEFORE_SEQ_EVEN; stage++) {
		CHECK(hook_seen[stage] == 1,
			"J2: before stable publication the consumer retains the OLD complete command");
	}
	CHECK(hook_seen[FIS_PUBLISH_AFTER_SEQ_EVEN] == 2,
		"J3: immediately after stable publication the consumer accepts the NEW complete command");

	/* Establish a different verified command, then hold seq odd for every read attempt. */
	fast_iq_slew_publish(&mb, 222, FIS_MODE_BYPASS, 5U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &hook_iq_out);
	mb.seq |= 1U;
	mb.target = 19;
	mb.step_mag_8 = 999U;
	mb.mode = (uint32_t)FIS_MODE_SAFETY;
	mb.release_ticks_16k = 3200U;
	mb.release_recip_q32 = 123U;
	mb.zero_policy = (uint32_t)FIS_ZERO_POLICY_QUIET;
	(void)fast_iq_slew_tick(&mb, &hook_iq_out);
	CHECK(hook_iq_out == 222 && fast_iq_slew_current_target() == 222 &&
		fast_iq_slew_current_mode() == FIS_MODE_BYPASS &&
		fast_iq_slew_current_step_mag_8() == 5U &&
		fast_iq_slew_current_zero_policy() == FIS_ZERO_POLICY_NONE,
		"K26: all bounded reads fail -> last VERIFIED command remains applied");

	/* Relevant generation wrap: FFFFFFFE -> FFFFFFFF(in progress) -> 0(stable). */
	fast_iq_slew_reset(&mb);
	mb.seq = UINT32_MAX - 1U;
	fast_iq_slew_publish(&mb, 333, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	int32_t wrapped_out = 0;
	(void)fast_iq_slew_tick(&mb, &wrapped_out);
	CHECK(mb.seq == 0U && wrapped_out == 333,
		"K27: even sequence generation wraps through zero and remains verifiable");
}

static void test_q10_small_max_and_rate_identity(void)
{
	fast_iq_slew_mailbox_t mb;
	int32_t out = 0;
	memset(&mb, 0, sizeof(mb));
	fast_iq_slew_reset(&mb);

	/* +7 counts per old 250 us period must be fractionally integrated over four ticks. */
	fast_iq_slew_publish(&mb, 100, FIS_MODE_RISE, (uint16_t)(7U << 8), 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	int32_t q10_samples[4];
	for (unsigned i = 0; i < 4U; i++) q10_samples[i] = fast_iq_slew_tick(&mb, &out);
	CHECK(q10_samples[0] > 0 && q10_samples[0] < 7 && q10_samples[3] == 7,
		"I1: Q10 advances within all four 62.5 us ticks and integrates to +7 counts");

	fast_iq_slew_reset(&mb); out = 0;
	fast_iq_slew_publish(&mb, 1, FIS_MODE_RISE, 1U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	for (unsigned i = 0; i < 2048U && out != 1; i++) (void)fast_iq_slew_tick(&mb, &out);
	CHECK(out == 1, "K3: small target reaches and clamps to one count");

	fast_iq_slew_reset(&mb); out = 0;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_RISE, 32767U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	for (unsigned i = 0; i < 64U && out != IDX_ID_SCALE; i++) (void)fast_iq_slew_tick(&mb, &out);
	CHECK(out == IDX_ID_SCALE, "K4: configured maximum target clamps without overshoot");

	/* Same target and mode, SLOW -> FAST -> SLOW: step is part of command identity. */
	fast_iq_slew_reset(&mb); out = 0;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_RISE, 16U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	int32_t start = out;
	run_fast_ticks(&mb, &out, 256U);
	int32_t slow_delta_1 = out - start;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_RISE, 160U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	start = out;
	run_fast_ticks(&mb, &out, 256U);
	int32_t fast_delta = out - start;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_RISE, 16U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	start = out;
	run_fast_ticks(&mb, &out, 256U);
	int32_t slow_delta_2 = out - start;
	CHECK(fast_delta > slow_delta_1 * 5,
		"K5: unchanged target/mode adopts a newly published faster rate");
	CHECK(slow_delta_2 < fast_delta / 5 && fast_iq_slew_current_step_mag_8() == 16U,
		"K6: unchanged target/mode adopts a newly published slower rate");
}

static void test_limiter_changes(void)
{
	fast_iq_slew_mailbox_t mb;
	int32_t out = 0;
	memset(&mb, 0, sizeof(mb));
	fast_iq_slew_reset(&mb);
	uint16_t up = step_for(IDX_ID_SCALE, RISE_SLOW_MS);
	uint16_t down = step_for(IDX_ID_SCALE, FALL_SLOW_MS);

	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_RISE, up, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	run_fast_ticks(&mb, &out, 3000U);
	int32_t before_tighten = out;
	fast_iq_slew_publish(&mb, 200, FIS_MODE_FALL, down, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	run_fast_ticks(&mb, &out, 256U);
	CHECK(out < before_tighten && out >= 200,
		"K8: battery/limiter tightening during rise reverses monotonically toward the cap");
	int32_t at_cap_release = out;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_RISE, up, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	run_fast_ticks(&mb, &out, 256U);
	CHECK(out > at_cap_release && out <= IDX_ID_SCALE,
		"K9: limiter release during rise resumes monotonically toward demand");
}

static void test_live_normal_releases(void)
{
	fast_iq_slew_mailbox_t mb;
	int32_t out;
	const uint32_t normal_ticks = RELEASE_MS * 16U;
	uint16_t up = step_for(IDX_ID_SCALE, RISE_SLOW_MS);
	uint16_t down = step_for(IDX_ID_SCALE, FALL_SLOW_MS);

	memset(&mb, 0, sizeof(mb)); fast_iq_slew_reset(&mb); out = 0;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &out);
	unsigned settled = release_to_zero(&mb, &out, FIS_MODE_RELEASE, normal_ticks);
	CHECK(out == 0 && settled >= normal_ticks - 64U && settled <= normal_ticks,
		"K10: normal release from settled max reaches exact zero in approximately 650 ms");

	fast_iq_slew_reset(&mb); out = 0;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_RISE, up, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	run_fast_ticks(&mb, &out, 3600U);
	int32_t live_rise = out;
	unsigned during_rise = release_to_zero(&mb, &out, FIS_MODE_RELEASE, normal_ticks);
	CHECK(live_rise > 0 && live_rise < IDX_ID_SCALE &&
		during_rise >= normal_ticks - 300U && during_rise <= normal_ticks,
		"K11: interrupted rise derives the 650 ms release from LIVE fast state");

	fast_iq_slew_reset(&mb); out = 0;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &out);
	fast_iq_slew_publish(&mb, 150, FIS_MODE_FALL, down, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	run_fast_ticks(&mb, &out, 2400U);
	int32_t live_fall = out;
	unsigned during_fall = release_to_zero(&mb, &out, FIS_MODE_RELEASE, normal_ticks);
	CHECK(live_fall > 150 && live_fall < IDX_ID_SCALE &&
		during_fall >= normal_ticks - 300U && during_fall <= normal_ticks,
		"K12: interrupted fall derives the 650 ms release from LIVE fast state");

	fast_iq_slew_reset(&mb); out = 0;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_RISE, up, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	run_fast_ticks(&mb, &out, 6000U);
	fast_iq_slew_publish(&mb, 300, FIS_MODE_FALL, down, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &out);
	unsigned after_cap = release_to_zero(&mb, &out, FIS_MODE_RELEASE, normal_ticks);
	CHECK(after_cap >= normal_ticks - 300U && after_cap <= normal_ticks,
		"K13: release immediately after battery-cap change uses current accumulator, not either target");
}

static void test_live_safety_releases(void)
{
	fast_iq_slew_mailbox_t mb;
	int32_t out;
	const uint32_t safety_ticks = 200U * 16U;
	uint16_t up = step_for(IDX_ID_SCALE, RISE_SLOW_MS);
	uint16_t down = step_for(IDX_ID_SCALE, FALL_SLOW_MS);

	memset(&mb, 0, sizeof(mb)); fast_iq_slew_reset(&mb); out = 0;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &out);
	unsigned settled = release_to_zero(&mb, &out, FIS_MODE_SAFETY, safety_ticks);
	CHECK(settled >= safety_ticks - 32U && settled <= safety_ticks,
		"K14: safety release from settled state reaches zero in approximately 200 ms");

	fast_iq_slew_reset(&mb); out = 0;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_RISE, up, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	run_fast_ticks(&mb, &out, 1800U);
	unsigned during_rise = release_to_zero(&mb, &out, FIS_MODE_SAFETY, safety_ticks);
	CHECK(during_rise >= safety_ticks - 100U && during_rise <= safety_ticks,
		"K15: safety release during rise is derived from LIVE fast state");

	fast_iq_slew_reset(&mb); out = 0;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &out);
	fast_iq_slew_publish(&mb, 300, FIS_MODE_FALL, down, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &out);
	unsigned after_cap = release_to_zero(&mb, &out, FIS_MODE_SAFETY, safety_ticks);
	CHECK(after_cap >= safety_ticks - 100U && after_cap <= safety_ticks,
		"K16: safety release after limiter change ignores stale requested targets");

	fast_iq_slew_reset(&mb); out = 0;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &out);
	fast_iq_slew_publish(&mb, 150, FIS_MODE_FALL, down, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	run_fast_ticks(&mb, &out, 2400U);
	int32_t live_fall = out;
	unsigned during_fall = release_to_zero(&mb, &out, FIS_MODE_SAFETY, safety_ticks);
	CHECK(live_fall > 150 && live_fall < IDX_ID_SCALE &&
		during_fall >= safety_ticks - 100U && during_fall <= safety_ticks,
		"R2 safety: interrupted fall releases from the authoritative LIVE Q10 state");
}

static void test_zero_lifecycle_cold_and_special_modes(void)
{
	fast_iq_slew_mailbox_t mb;
	int32_t out = 0;
	memset(&mb, 0, sizeof(mb)); fast_iq_slew_reset(&mb);
	fast_iq_slew_publish(&mb, 300, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &out);

	enum { MODEL_ACTIVE, MODEL_ARMED_ZERO } lifecycle = MODEL_ACTIVE;
	fast_iq_slew_publish(&mb, 0, FIS_MODE_RELEASE, 0U, RELEASE_MS * 16U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &out);
	if (out == 0) lifecycle = MODEL_ARMED_ZERO;
	CHECK(out > 0 && lifecycle == MODEL_ACTIVE,
		"K18: zero target cannot enter ARMED_ZERO before actual fast output is zero");
	while (out > 0) {
		(void)fast_iq_slew_tick(&mb, &out);
		if (out == 0) lifecycle = MODEL_ARMED_ZERO;
	}
	CHECK(out == 0 && lifecycle == MODEL_ARMED_ZERO,
		"K17/K19: exact fast zero is the event that enters ARMED_ZERO");
	fast_iq_slew_publish(&mb, 100, FIS_MODE_RISE, 512U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	while (out == 0) (void)fast_iq_slew_tick(&mb, &out);
	if (out > 0) lifecycle = MODEL_ACTIVE;
	CHECK(lifecycle == MODEL_ACTIVE,
		"K20: positive real fast output restarts ARMED_ZERO without a cold transaction");

	fast_iq_slew_publish(&mb, 91, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &out);
	fast_iq_slew_cold_prepare(&mb, &out);
	CHECK(out == 0 && mb.target == 0 && mb.mode == (uint32_t)FIS_MODE_FORCE_ZERO &&
		mb.step_mag_8 == 0U && mb.release_ticks_16k == 0U,
		"K21: cold PREPARE clears target, mode, rate input and visible/fractional output");

	/* Service/calibration remain fast-owner BYPASS; comm-loss is exact FORCE_ZERO. */
	fast_iq_slew_publish(&mb, 37, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &out);
	CHECK(out == 37, "K22: service command is applied by fast-owner BYPASS");
	fast_iq_slew_publish(&mb, 0, FIS_MODE_FORCE_ZERO, 0U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &out);
	CHECK(out == 0, "K23: comm-loss command is applied as fast-owner exact zero");
	fast_iq_slew_publish(&mb, 42, FIS_MODE_BYPASS, 0U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &out);
	fast_iq_slew_publish(&mb, 100, FIS_MODE_RISE, 256U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &out);
	CHECK(out >= 42 && fast_iq_slew_current_mode() == FIS_MODE_RISE,
		"K24: calibration BYPASS -> normal slew transition has one continuous owner");

	bool hardware_moe = true;
	fast_iq_slew_publish(&mb, IDX_ID_SCALE, FIS_MODE_RISE, 256U, 0U, FIS_ZERO_POLICY_NONE, IDX_ID_SCALE);
	(void)fast_iq_slew_tick(&mb, &out);
	hardware_moe = false; /* model the independent FOC overcurrent action */
	CHECK(!hardware_moe && out > 0,
		"K25: a hard fault can disable MOE immediately while fast slew state is active");
}

static void test_production_static_guards(void)
{
	char *main_c = read_source(STRINGIZE(MAIN_C_PATH));
	char *ride_c = read_source(STRINGIZE(RIDE_CONTROL_C_PATH));
	/* The trajectory DECISION moved into the one assist chain; ride_control kept the single
	 * publication. Each guard below reads whichever of the two owns the property it asserts. */
	char *pipe_c = read_source(STRINGIZE(ASSIST_PIPELINE_C_PATH));
	char *motor_c = read_source(STRINGIZE(MOTOR_CORE_C_PATH));
	char *foc_c = read_source(STRINGIZE(FOC_C_PATH));
	CHECK(main_c && ride_c && motor_c && foc_c, "STATIC setup: production sources are readable");
	if (!main_c || !ride_c || !motor_c || !foc_c) goto done;

	CHECK(source_occurrences(ride_c, "MS.i_q_setpoint = ") == 0U &&
		strstr(main_c, "motor_core_set_command(") == NULL &&
		strstr(motor_c, "state->i_q_setpoint = 0") != NULL &&
		source_occurrences(motor_c, "state->i_q_setpoint =") == 1U,
		"STATIC: Motor Core has one boot-zero write and no normal/service dynamic Iq write");
	CHECK(strstr(main_c, "fast_iq_slew_tick(") != NULL &&
		strstr(main_c, "&MS.i_q_setpoint") != NULL,
		"STATIC: fast_iq_slew_tick is the sole normal dynamic MS.i_q_setpoint owner");
	CHECK(strstr(main_c, "PI_iq.setpoint = MP.reverse * i8_reverse_flag * MS.i_q_setpoint;") != NULL &&
		strstr(main_c, "PI_iq.recent_value = MS.i_q;") != NULL,
		"STATIC: PI_iq reference and feedback remain entirely in the Iq domain");

	{
		/* The cap is a stage of the limiter chain the pipeline runs before it decides the
		 * trajectory; ride_control then publishes that decision once. Both halves are checked,
		 * so neither can be moved after the final owner without this failing. */
		const char *limits = pipe_c ? strstr(pipe_c, "ap2_limits_apply(&lim_in, &lim);") : NULL;
		const char *traj = limits ? strstr(limits, "cmd->slew_mode = trajectory(") : NULL;
		const char *publish = strstr(ride_c, "ride_publish_final_iq(cmd.final_iq_request");
		CHECK(limits && traj && publish &&
			strstr(main_c, "PI_iq.recent_value = MS.Battery_Current;") == NULL,
			"STATIC: battery cap remains before final slew and no post-slew battery-domain clamp exists");
	}
	CHECK(strstr(ride_c, "assist_dynamics_apply(") == NULL,
		"STATIC: the legacy 4 kHz final owner is gone, not merely inactive");
	CHECK(pipe_c && strstr(pipe_c, "return block_positive ? FIS_MODE_SAFETY : FIS_MODE_RELEASE;") != NULL,
		"STATIC: production explicitly publishes FIS_MODE_SAFETY for the 200 ms hard cut");
	CHECK(pipe_c && strstr(pipe_c, "fast_iq_slew_current_accumulator_q10()") != NULL &&
		strstr(pipe_c, "rising = target_q10 > live_q10;") != NULL &&
		strstr(pipe_c, "return (iq_target > 0) ? FIS_MODE_RISE : FIS_MODE_FALL") == NULL,
		"R2 STATIC: production direction compares target with authoritative live Q10, never target sign");
	CHECK(strstr(main_c, "ride_control_force_final_iq_zero();") != NULL &&
		strstr(ride_c, "hall_calibration_iq_request()") != NULL &&
		strstr(ride_c, "motor_core_set_id_target(input->current_id);") != NULL,
		"STATIC: comm-loss and calibration are explicitly classified under fast-owner commands");
	CHECK(strstr(main_c, "fast_iq_slew_cold_prepare(") != NULL &&
		strstr(main_c, "ride_control_final_iq_requested() > 0") != NULL,
		"STATIC: true bridge-off cold PREPARE resets fast state before torque enable");
	CHECK(strstr(foc_c, "MS_FOC->i_d>(PH_CURRENT_MAX<<2)") != NULL &&
		strstr(foc_c, "timer_primary_output_config(TIMER0,DISABLE)") != NULL &&
		strstr(foc_c, "bridge_lifecycle=BRIDGE_LIFECYCLE_FAULT") != NULL,
		"STATIC: hard overcurrent still disables MOE independently of the slew");

done:
	free(main_c); free(ride_c); free(pipe_c); free(motor_c); free(foc_c);
}

int main(void)
{
	puts("QS-3D 16 kHz final Iq slew parity host proof");
	test_force_zero_and_bypass();
	test_direction_change();
	test_mailbox_interruptions_and_retry();
	test_q10_small_max_and_rate_identity();
	test_exact_release_ceil_sweep();
	test_limiter_changes();
	test_live_normal_releases();
	test_live_safety_releases();
	test_zero_lifecycle_cold_and_special_modes();
	test_production_static_guards();
	if (host_test_failures == 0) {
		puts("QS-3D 16 kHz final Iq slew parity: ALL CHECKS PASSED");
		return 0;
	}
	printf("QS-3D 16 kHz final Iq slew parity: %d CHECK(S) FAILED\n", host_test_failures);
	return 1;
}
