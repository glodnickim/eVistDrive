/*
 * TQ-06 Phase 7 production-path scenarios.
 *
 * The historical filename is retained for the canonical runner, but all scenarios now execute
 * the shipped G53 facade through assist_pipeline_update(), the real limiter chain and the real
 * 16 kHz final-Iq owner. This suite pins the frozen zero-policy and native-veto contract; G53
 * arithmetic itself is independently checked by the exact boundary/PAS/chain differential suites.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "assist_pipeline.h"
#include "assist_modes.h"
#include "config.h"
#include "fast_iq_slew.h"

static unsigned failures;
#define CHECK(c, m) do { if (!(c)) { printf("  FAIL  %s\n", (m)); ++failures; } } while (0)

static const uint8_t pas_cycle[] = {1U, 3U, 2U, 0U};
#define PHASE5_FIXTURE_FIRST_LINE 13202U
#define PHASE5_FIXTURE_POSITIVE_LINE 14422U
#define PHASE5_FIXTURE_TICKS (PHASE5_FIXTURE_POSITIVE_LINE - PHASE5_FIXTURE_FIRST_LINE + 1U)
static uint8_t phase5_pas_ab[PHASE5_FIXTURE_TICKS];
static bool phase5_pas_loaded;
static bool public_vector_active;
static uint32_t control_tick;
static fast_iq_slew_mailbox_t mailbox;
static int32_t iq_ref;

static bool load_phase5_pas_fixture(void)
{
	FILE *fixture = fopen("integration/evidence/evd-tq/TQ-06/host/chain/chain-reference.csv", "r");
	char line[8192];
	uint32_t physical_line = 0U;
	uint32_t sample_count = 0U;
	bool block_started = false;
	bool header = true;
	if (fixture == NULL) return false;
	while (fgets(line, sizeof(line), fixture) != NULL) {
		++physical_line;
		if (header) { header = false; continue; }
		unsigned reset, pas_ab, level, x_input, rider_input, speed_input, diag, inhibit, logical_tick;
		if (sscanf(line, "%u,%u,%u,%u,%u,%u,%u,%u,%u", &reset, &pas_ab, &level,
			&x_input, &rider_input, &speed_input, &diag, &inhibit, &logical_tick) != 9) {
			fclose(fixture);
			return false;
		}
		if (!block_started) {
			if (physical_line == PHASE5_FIXTURE_FIRST_LINE && reset == 1U && pas_ab == 0U &&
				level == 0U && x_input == 500U &&
				rider_input == 3200U && speed_input == 500U && diag == 0U &&
				inhibit == 0U && logical_tick == 0U) {
				block_started = true;
			} else if (physical_line >= PHASE5_FIXTURE_FIRST_LINE) {
				fclose(fixture);
				return false;
			} else {
				continue;
			}
		} else if (reset != 0U || logical_tick != sample_count || pas_ab > 3U) {
			fclose(fixture);
			return false;
		}
		phase5_pas_ab[sample_count++] = (uint8_t)pas_ab;
		if (sample_count == PHASE5_FIXTURE_TICKS) {
			if (physical_line != PHASE5_FIXTURE_POSITIVE_LINE) {
				fclose(fixture);
				return false;
			}
			break;
		}
	}
	fclose(fixture);
	phase5_pas_loaded = block_started && sample_count == PHASE5_FIXTURE_TICKS;
	return phase5_pas_loaded;
}

static assist_pipeline_input_t base_input(void)
{
	assist_pipeline_input_t in;
	memset(&in, 0, sizeof(in));
	in.raw_pa6_adc = 0U;
	in.torque_load_ctrl = 6000U;
	in.torque_sensor_valid = true;
	in.pas_sensor_valid = true;
	in.forward_valid = true;
	in.wheel_valid = true;
	in.assist_level_index = 1U;
	in.speed_x100 = 5000U;
	in.phase_current_max = 900;
	in.battery_voltage_mv = 42000U;
	in.battery_current_max = 15000;
	in.u_abs = 1024;
	in.cal_i = 95;
	in.voltage_raw = 4000U;
	in.voltage_min_raw = 2800;
	in.controller_temperature_c = 30;
	in.speed_limit_x100 = 6000U;
	in.elapsed_ticks = 4U;
	return in;
}

/* TASK-EVD-TQ-06-G2 I2: the normal PEDAL path publishes the G5300 fast slew (RISE/FALL), not BYPASS. */
static bool is_fast_slew(fis_mode_t m) { return m == FIS_MODE_RISE || m == FIS_MODE_FALL; }

static void reset_all(void)
{
	control_tick = 0U;
	public_vector_active = false;
	iq_ref = 0;
	memset(&mailbox, 0, sizeof(mailbox));
	fast_iq_slew_reset(&mailbox);
	assist_modes_init(); /* production initializes the selected level bank before the pipeline */
	assist_pipeline_init();
}

static void pipeline_tick(assist_pipeline_input_t *in, assist_pipeline_command_t *cmd)
{
	const uint32_t logical_tick = control_tick;
	in->elapsed_ticks = 4U;
	if (public_vector_active) {
		in->torque_load_ctrl = logical_tick < 130U ? 0U : 6000U;
		in->pas_ab = !in->forward_valid || logical_tick >= PHASE5_FIXTURE_TICKS ? 0U :
			phase5_pas_ab[logical_tick];
	} else {
		in->pas_ab = !in->forward_valid || logical_tick < 30U ? 0U :
			pas_cycle[((logical_tick - 30U) / 6U) % (sizeof(pas_cycle) / sizeof(pas_cycle[0]))];
	}
	assist_pipeline_update(in, cmd);
	fast_iq_slew_publish(&mailbox, cmd->final_iq_request, cmd->slew_mode, cmd->step_mag_8,
		cmd->release_ticks_16k, cmd->zero_policy, cmd->iq_ceiling);
	for (unsigned i = 0; i < 16U; ++i) fast_iq_slew_tick(&mailbox, &iq_ref);   /* 16 kHz x 1 ms */
	++control_tick;
}

static void establish_positive(assist_pipeline_input_t *in, assist_pipeline_command_t *cmd)
{
	public_vector_active = true;
	for (uint32_t tick = 0U; tick <= 1220U; ++tick) pipeline_tick(in, cmd);
	CHECK(phase5_pas_loaded && control_tick == 1221U,
		"P6/P7/P8: public vector replays the accepted Phase-5 fixture through tick 1220");
	CHECK(assist_pipeline_g53()->m2aa_native > 0 &&
		assist_pipeline_g53()->normal_permission &&
		assist_pipeline_g53()->iq_request_pre_limits > 0 &&
		cmd->final_iq_request > 0 && is_fast_slew(cmd->slew_mode) &&
		cmd->zero_policy == FIS_ZERO_POLICY_NONE,
		"P6/P7/P8: fixture setup establishes real positive demand and RISE|FALL/NONE");
}

static void scenarios(void)
{
	assist_pipeline_input_t in;
	assist_pipeline_command_t cmd;
	const assist_pipeline_telemetry_t *tlm;

	/* P8: real G53 demand is passed through the one production pipeline and fast owner. */
	reset_all(); in = base_input(); establish_positive(&in, &cmd);
	CHECK(assist_pipeline_g53()->m2aa_native > 0 &&
		assist_pipeline_g53()->normal_permission &&
		assist_pipeline_g53()->iq_request_pre_limits > 0 &&
		cmd.final_iq_request > 0 && iq_ref > 0,
		"P8: M2AA, permission, pre-limit and final request are all real and positive");
	CHECK(is_fast_slew(cmd.slew_mode) && cmd.zero_policy == FIS_ZERO_POLICY_NONE,
		"P8: normal positive G53 demand publishes the fast slew (RISE|FALL) with policy NONE");

	/* P1 overrides each class of zero decision without changing its selected mode/request. */
	in.service_cut = true; in.direction_inhibit = true;
	pipeline_tick(&in, &cmd);
	CHECK(cmd.final_iq_request == 0 && cmd.slew_mode == FIS_MODE_FORCE_ZERO &&
		cmd.zero_policy == FIS_ZERO_POLICY_NONE,
		"P1/P2: service cut preserves direction FORCE_ZERO and suppresses QUIET");
	in.direction_inhibit = false; in.safety_cut = true;
	pipeline_tick(&in, &cmd);
	CHECK(cmd.final_iq_request == 0 && cmd.slew_mode == FIS_MODE_SAFETY &&
		cmd.release_ticks_16k == 3200U && cmd.zero_policy == FIS_ZERO_POLICY_NONE,
		"P1/P3: service cut preserves native SAFETY/200 ms and suppresses QUIET");
	in.safety_cut = false; in.real_stop = true;
	pipeline_tick(&in, &cmd);
	CHECK(cmd.final_iq_request == 0 && cmd.slew_mode == FIS_MODE_SAFETY &&
		cmd.release_ticks_16k == 3200U && cmd.zero_policy == FIS_ZERO_POLICY_NONE,
		"P1/P4: service cut preserves real-stop SAFETY/200 ms and suppresses QUIET");

	/* P2: direction inhibit at standstill is same-update exact zero. On a MOVING bike it is a
	 * bounded, never-rising decay of the G53 output instead (TASK-EVD-TQ-06-G2,
	 * OWNER-DEC-2026-10-05-TQ06G2-A; the full ramp is pinned in reverse_ramp_host.c). */
	reset_all(); in = base_input(); establish_positive(&in, &cmd);
	in.speed_x100 = 9U;
	in.direction_inhibit = true;
	pipeline_tick(&in, &cmd);
	CHECK(cmd.final_iq_request == 0 && cmd.slew_mode == FIS_MODE_FORCE_ZERO &&
		cmd.zero_policy == FIS_ZERO_POLICY_QUIET && iq_ref == 0,
		"P2: direction inhibit at standstill commands same-update exact zero and QUIET");
	reset_all(); in = base_input(); establish_positive(&in, &cmd);
	const int32_t moving_before = cmd.final_iq_request;
	in.direction_inhibit = true;
	pipeline_tick(&in, &cmd);
	CHECK(cmd.final_iq_request <= moving_before && cmd.slew_mode != FIS_MODE_SAFETY &&
		(cmd.final_iq_request > 0 ? cmd.slew_mode == FIS_MODE_BYPASS : true),
		"P2: direction inhibit while moving never rises and does not step through the safety release");

	/* P3/P4: hard vetoes own the 200 ms safety release; held references cannot rise. */
	reset_all(); in = base_input(); establish_positive(&in, &cmd);
	in.safety_cut = true;
	const int32_t safety_before = iq_ref;
	pipeline_tick(&in, &cmd);
	CHECK(cmd.final_iq_request == 0 && cmd.slew_mode == FIS_MODE_SAFETY &&
		cmd.release_ticks_16k == 3200U && cmd.zero_policy == FIS_ZERO_POLICY_QUIET &&
		iq_ref <= safety_before,
		"P3: native safety cut uses the 3200-tick SAFETY release and QUIET");
	in.safety_cut = false; in.torque_sensor_valid = false;
	pipeline_tick(&in, &cmd);
	CHECK(cmd.final_iq_request == 0 && cmd.slew_mode == FIS_MODE_SAFETY &&
		cmd.release_ticks_16k == 3200U && cmd.zero_policy == FIS_ZERO_POLICY_QUIET,
		"P3: torque-sensor invalidity uses the native SAFETY release and QUIET");
	in.torque_sensor_valid = true; in.pas_sensor_valid = false;
	pipeline_tick(&in, &cmd);
	CHECK(cmd.final_iq_request == 0 && cmd.slew_mode == FIS_MODE_SAFETY &&
		cmd.release_ticks_16k == 3200U && cmd.zero_policy == FIS_ZERO_POLICY_QUIET,
		"P3: PAS-sensor invalidity uses the native SAFETY release and QUIET");

	reset_all(); in = base_input(); establish_positive(&in, &cmd);
	in.real_stop = true;
	pipeline_tick(&in, &cmd);
	CHECK(cmd.final_iq_request == 0 && cmd.slew_mode == FIS_MODE_SAFETY &&
		cmd.release_ticks_16k == 3200U && cmd.zero_policy == FIS_ZERO_POLICY_QUIET,
		"P4: true stop uses the native 200 ms SAFETY release and QUIET");

	/* P5: level off is still BYPASS; its resulting zero may use QUIET. */
	reset_all(); in = base_input(); in.assist_level_index = 0U;
	for (unsigned i = 0; i < 12000U; ++i) pipeline_tick(&in, &cmd);
	CHECK(cmd.final_iq_request == 0 && cmd.slew_mode == FIS_MODE_BYPASS &&
		cmd.zero_policy == FIS_ZERO_POLICY_QUIET,
		"P5: assist level zero remains BYPASS and grants QUIET at final zero");

	/* P7: a limit-created zero while forward pedalling is not a rider-caused Quiet Zero. */
	reset_all(); in = base_input(); establish_positive(&in, &cmd);
	CHECK(assist_pipeline_g53()->m2aa_native > 0 &&
		assist_pipeline_g53()->normal_permission &&
		assist_pipeline_g53()->iq_request_pre_limits > 0 && cmd.final_iq_request > 0,
		"P7: limiter scenario starts from a real positive G53/final request");
	/* ADR-013 / TASK-EVD-TQ-06-G1: on the PEDAL path the battery current is owned by the ported
	 * G53 PI #1 (g1 inside BDE8), not by the ap2 battery stage. The P7 property is unchanged: a
	 * limiter zero while forward demand remains is BYPASS/NONE, never QUIET. The limiter now
	 * acts BEFORE Boundary B: BDE8 stays in its drive state (normal_permission) while M2AA,
	 * the request before limits and the final request all go to 0. */
	in.battery_current_ma = 15000;
	in.battery_current_max = 10000;                 /* limit 10 A */
	in.battery_current_limiter_centiamp = 3000;     /* measured 30 A: sustained overload */
	for (unsigned i = 0; i < 100U; ++i) pipeline_tick(&in, &cmd);
	tlm = assist_pipeline_telemetry();
	CHECK(cmd.final_iq_request == 0 && is_fast_slew(cmd.slew_mode) &&
		cmd.zero_policy == FIS_ZERO_POLICY_NONE && tlm->battery_limited &&
		assist_pipeline_g53()->trace.g1 == 0 &&
		assist_pipeline_g53()->normal_permission &&
		assist_pipeline_battery_limited(),
		"P7: battery limiter (G53 g1) zero while forward demand remains uses the fast slew with NONE");
	{
		/* P7b: the same limiter gives the current back once the measured current falls. */
		bool recovered = false;
		in.battery_current_limiter_centiamp = 0;
		for (unsigned i = 0; i < 400U && !recovered; ++i) {
			pipeline_tick(&in, &cmd);
			recovered = cmd.final_iq_request > 0;
		}
		CHECK(recovered && assist_pipeline_g53()->trace.g1 > 0 && is_fast_slew(cmd.slew_mode),
			"P7b: g1 recovers and assist returns when the battery current falls below the limit");
		in.battery_current_max = 15000;
	}
	{
		/* P7c (review F-02): the PEDAL path does NOT run the M820 ap2 battery stage any more. A
		 * filtered battery current far above battery_current_max must leave the ap2 latch idle
		 * and the request positive while the G53 limiter (fed by its own tap) sees no overload. */
		int32_t before;
		in.battery_current_limiter_centiamp = 0;
		for (unsigned i = 0; i < 400U; ++i) pipeline_tick(&in, &cmd);
		before = cmd.final_iq_request;
		in.battery_current_ma = 40000;
		in.battery_current_max = 15000;
		for (unsigned i = 0; i < 100U; ++i) pipeline_tick(&in, &cmd);
		CHECK(before > 0 && cmd.final_iq_request > 0 && !ap2_limits_battery_active() &&
			!assist_pipeline_telemetry()->battery_limited,
			"P7c: PEDAL path skips the ap2 battery stage (ADR-013: G53 PI #1 owns battery current)");
		in.battery_current_ma = 15000;
	}

	/* P1 also covers zeroes reached through assist-off, and remains default-deny. The zero must
	 * really be limiter-created at the moment of the check (review NEW-01): P7b/P7c above gave the
	 * current back, so drive g1 to zero again first. */
	in.battery_current_limiter_centiamp = 3000;
	in.battery_current_max = 10000;
	for (unsigned i = 0; i < 100U; ++i) pipeline_tick(&in, &cmd);
	CHECK(cmd.final_iq_request == 0 && assist_pipeline_g53()->trace.g1 == 0,
		"P1/P7: setup - the zero is created by the G53 battery limiter");
	in.service_cut = true;
	pipeline_tick(&in, &cmd);
	CHECK(cmd.final_iq_request == 0 && assist_pipeline_g53()->trace.g1 == 0 &&
		cmd.zero_policy == FIS_ZERO_POLICY_NONE && is_fast_slew(cmd.slew_mode),
		"P1/P7: service policy remains NONE on a limiter-created zero");
	in.battery_current_limiter_centiamp = 0;
	in.battery_current_max = 15000;

	/* P7d (variant A, review S2-04), on a fresh positive request: the M820 Iq thermal derate stays on
	 * the PEDAL path - it limits the phase current that heats the bridge. At 95 degC (above the 90 degC
	 * band end) it takes the request to 0 in the same tick; the fixture's PAS ends at establish. */
	reset_all(); in = base_input(); establish_positive(&in, &cmd);
	in.controller_temperature_c = 95;
	pipeline_tick(&in, &cmd);
	CHECK(cmd.final_iq_request == 0 && assist_pipeline_telemetry()->thermal_limited &&
		assist_pipeline_g53()->trace.g1 == G53_G1_Q12_ONE,
		"P7d: PEDAL keeps the M820 Iq thermal derate; the G53 limit is not involved");
	in.controller_temperature_c = 30;
	in.battery_current_limiter_centiamp = 1000;  /* 10 A: under 15 A, over the 7.5 A SOC knee */
	for (unsigned i = 0; i < 600U; ++i) pipeline_tick(&in, &cmd);
	CHECK(assist_pipeline_g53()->trace.g1 == G53_G1_Q12_ONE, "P7e: setup - g1 back at 1.0 with 10 A under 15 A");
	/* P7e: full SOC derate takes the limit to the 50 % knee (7.5 A of 15 A): 10 A is now over it. */
	in.battery_soc_derate_q12 = 0x1000;
	for (unsigned i = 0; i < 600U; ++i) pipeline_tick(&in, &cmd);
	CHECK(assist_pipeline_g53()->trace.g1 == 0 && g53_port_g1_state()->limit == 750,
		"P7e: the SOC derate acts through the G53 limit (knee 50 % = 7.5 A)");
	in.battery_soc_derate_q12 = 0;
	in.battery_current_limiter_centiamp = 0;

	/* P6: no forward PAS lets the native G53 request decay; QUIET is granted only at zero. */
	reset_all(); in = base_input(); establish_positive(&in, &cmd);
	CHECK(assist_pipeline_g53()->m2aa_native > 0 &&
		assist_pipeline_g53()->normal_permission &&
		assist_pipeline_g53()->iq_request_pre_limits > 0 && cmd.final_iq_request > 0,
		"P6: release scenario starts from a real positive G53/final request");
	in.forward_valid = false; in.pas_ab = 0U;
	bool saw_nonzero = false, quiet_zero = false, invalid_release_mode = false;
	for (unsigned i = 0; i < 8000U; ++i) {
		pipeline_tick(&in, &cmd);
		if (cmd.final_iq_request > 0) {
			saw_nonzero = true;
			if (cmd.zero_policy != FIS_ZERO_POLICY_NONE || !is_fast_slew(cmd.slew_mode))
				invalid_release_mode = true;
		} else if (cmd.zero_policy == FIS_ZERO_POLICY_QUIET) {
			quiet_zero = true;
			break;
		}
	}
	CHECK(saw_nonzero && quiet_zero && !invalid_release_mode &&
		is_fast_slew(cmd.slew_mode) && cmd.final_iq_request == 0,
		"P6: G53 demand decays under the fast slew/NONE and grants QUIET only on the first final zero");

	/* T2b/T3/T7/T9/T11: zero with a live forward chain, limiter, or ordinary coast is NONE;
	 * assist-off/no-forward grants only at zero and the pipeline never adds RISE/FALL shaping. */
	reset_all(); in = base_input(); in.torque_load_ctrl = 0U;
	for (unsigned i = 0; i < 12000U; ++i) pipeline_tick(&in, &cmd);
	/* No load and no PAS: the BDE8 drive permission is down with m2aa 0 - the stock hard-zero
	 * (TQ-06-G2 I2), same exact zero as the former BYPASS 0 - and QUIET is still not granted. */
	CHECK(cmd.final_iq_request == 0 && cmd.slew_mode == FIS_MODE_FORCE_ZERO &&
		cmd.zero_policy == FIS_ZERO_POLICY_NONE,
		"T3/T9/T11: ordinary zero with live forward state is exact zero (stock hard-zero)/NONE");
	CHECK(cmd.slew_mode != FIS_MODE_RELEASE && cmd.slew_mode != FIS_MODE_HOLD &&
		cmd.step_mag_8 == 0U,
		"T2b/T7: normal G53 path has no legacy ride-feel RELEASE/HOLD mode (live fast-slew step is pinned in fast_slew_i2_host.c)");
}

int main(void)
{
	puts("TQ-06: production G53 assist pipeline + limits + zero-policy scenarios");
	phase5_pas_loaded = load_phase5_pas_fixture();
	CHECK(phase5_pas_loaded,
		"P6/P7/P8: load accepted reset block and PAS history from chain-reference.csv");
	scenarios();
	if (failures == 0U) {
		puts("TQ-06 G53 pipeline scenarios: ALL CHECKS PASSED");
		return 0;
	}
	printf("TQ-06 G53 pipeline scenarios: %u CHECK(S) FAILED\n", failures);
	return 1;
}
