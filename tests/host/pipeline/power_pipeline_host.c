/*
 * L2/L3 power pipeline harness — runs the SHIPPED src/torque_input.c, src/rider_input.c
 * and src/assist_modes.c, not copies of them. This is the primary vehicle for
 * RUN_60..RUN_120 and CADENCE_RAMP_50_120 (documentation/testing/REGRESSION_SCENARIOS.md):
 * it answers "did torque/power behaviour vs crank angle/cadence change", which is the
 * question the card asks first, without pulling in ride_control's latch/ramp/limiter
 * state (that is a deliberately SEPARATE, deeper harness —
 * tests/host/pipeline/ride_control_pipeline_host.c — see its header for why keeping
 * them apart is itself a design choice, not a shortcut).
 *
 * Fixed assumptions, documented here because they are NOT scenario parameters (only
 * cadence/torque-shape are — section 7 of the card):
 *   - assist level index 3 of bank 0 (Power bank) — a representative mid-level.
 *   - battery voltage 42000 mV, current limit PH_CURRENT_MAX (config.h) — a plausible
 *     healthy-battery operating point, not swept.
 *   - motor_voltage_utilization = 0 (no FOC in this harness, so no real PWM duty is
 *     known) — this SKIPS the power->duty voltage cross-check clamp inside
 *     finish_power_request(); see KNOWN ISSUES in
 *     documentation/assist/POWER_MODE.md and the OBSERVABILITY GAP in the final report.
 *   - cadence compensation stays at its bank default (OFF) — deliberate, see the card's
 *     section 7: "RUN_60...120 NIE JEST TESTEM CADENCE COMPENSATION".
 *
 * Usage: power_pipeline_host <scenario> <output.csv>
 */

#include <stdio.h>
#include <string.h>

#include "assist_modes.h"
#include "config.h"
#include "crank_model.h"
#include "csv.h"
#include "rider_input.h"
#include "torque_input.h"

static const crank_torque_shape_t SHARED_SHAPE = {
	.mean_native_delta = 300.0,
	.ripple_pct = 40.0,
	.asymmetry_pct = 15.0,
	.dead_spot_depth_pct = 30.0,
	.dead_spot_width_deg = 20.0,
	.phase_shift_deg = 0.0
};

#define RUN_WINDOW_DEG_DEFAULT 180U
#define ASSIST_LEVEL_INDEX 3U
#define BATTERY_VOLTAGE_MV 42000U
#define WHEEL_SPEED_X100_FIXED 1500U /* 15.00 km/h — a representative "already rolling" speed */

typedef struct {
	const char *name;
	double cadence_rpm;
	double duration_s;
	int is_ramp;
	double ramp_start_rpm;
	double ramp_end_rpm;
	double ramp_duration_s;
} scenario_def_t;

static const scenario_def_t SCENARIOS[] = {
	{ "RUN_60",  60.0,  6.0, 0, 0, 0, 0 },
	{ "RUN_80",  80.0,  6.0, 0, 0, 0, 0 },
	{ "RUN_100", 100.0, 6.0, 0, 0, 0, 0 },
	{ "RUN_110", 110.0, 6.0, 0, 0, 0, 0 },
	{ "RUN_120", 120.0, 6.0, 0, 0, 0, 0 },
	{ "CADENCE_RAMP_50_120", 0.0, 14.0, 1, 50.0, 120.0, 10.0 },
};
#define SCENARIO_COUNT (sizeof(SCENARIOS) / sizeof(SCENARIOS[0]))

static const scenario_def_t *find_scenario(const char *name)
{
	for (size_t i = 0; i < SCENARIO_COUNT; i++) {
		if (strcmp(SCENARIOS[i].name, name) == 0) {
			return &SCENARIOS[i];
		}
	}
	return NULL;
}

int main(int argc, char **argv)
{
	if (argc != 3) {
		fprintf(stderr, "usage: %s <scenario> <output.csv>\n", argv[0]);
		return 2;
	}
	const scenario_def_t *sc = find_scenario(argv[1]);
	if (sc == NULL) {
		fprintf(stderr, "unknown scenario: %s\n", argv[1]);
		return 2;
	}

	torque_input_init();
	torque_input_set_run_window_deg(RUN_WINDOW_DEG_DEFAULT);
	assist_modes_init();
	assist_modes_set_active_bank(0); /* Power bank */
	const assist_level_config_t *level = assist_modes_get_default_level(ASSIST_LEVEL_INDEX);

	crank_state_t crank;
	crank_state_init(&crank);

	FILE *out = csv_open_or_die(argv[2],
		"tick,time_s,crank_angle_deg,pas_state,cadence_input,"
		"torque_raw,torque_corrected,torque_fast,torque_run,load_centikg,"
		"human_power_w,motor_power_raw_w,motor_power_w,support_pct,"
		"startup_boost_extra_pct,cadence_comp_permille,iq_request");

	uint32_t total_ticks = (uint32_t)(sc->duration_s * CRANK_MODEL_TICK_HZ);

	for (uint32_t tick = 0; tick < total_ticks; tick++) {
		double t_s = (double)tick / CRANK_MODEL_TICK_HZ;
		double cadence_rpm = sc->is_ramp ?
			crank_cadence_ramp(sc->ramp_start_rpm, sc->ramp_end_rpm, sc->ramp_duration_s, t_s) :
			sc->cadence_rpm;

		uint32_t new_steps = crank_state_advance_tick(&crank, cadence_rpm);
		uint16_t raw_mv = crank_torque_raw_mv(&crank, &SHARED_SHAPE);

		int16_t corrected = torque_input_correct(raw_mv);
		for (uint32_t i = 0; i < new_steps; i++) {
			torque_input_run_filter_step();
		}
		torque_input_coast_update(corrected, false, true);
		torque_input_set_run_window_deg(RUN_WINDOW_DEG_DEFAULT);
		torque_input_update(raw_mv, corrected, true);

		const torque_snapshot_t *snap = torque_input_get_snapshot();

		rider_input_t sample = { 0 };
		sample.torque_raw_mv = raw_mv;
		sample.torque_corrected_mv = corrected;
		sample.torque_filtered = 0; /* not consumed by assist_modes — see file header */
		sample.torque_assist_filtered = snap->assist_delta_filtered_native;
		sample.torque_run_filtered = snap->assist_delta_run_native;
		sample.torque_load_centikg = snap->load_centikg;
		sample.cadence_rpm = (uint8_t)(cadence_rpm > 255.0 ? 255 : cadence_rpm);
		sample.wheel_speed_x100 = WHEEL_SPEED_X100_FIXED;
		sample.motor_erps = 0;
		sample.motor_voltage_utilization = 0; /* see file header: no FOC in this harness */
		sample.pas_forward = true;
		sample.pas_backward = false;
		sample.pedaling_active = true;
		sample.crank_forward_steps = 250;
		sample.crank_direction_ok = true;
		sample.start_phase = false;
		sample.torque_sensor_valid = true;
		sample.pas_sensor_valid = true;
		rider_input_update(&sample);

		assist_mode_output_t output;
		bool supported = assist_modes_calculate(
			rider_input_get(), level, BATTERY_VOLTAGE_MV, (int32_t)PH_CURRENT_MAX, &output);
		(void)supported; /* level 3's mode is always POWER_LINEAR by default -> always true */

		fprintf(out, "%u,%.6f,%.3f,%u,%.3f,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d\n",
			tick, t_s, crank.crank_angle_deg, crank_pas_state(&crank), cadence_rpm,
			(unsigned)raw_mv, (int)corrected,
			(unsigned)snap->assist_delta_filtered_native,
			(unsigned)snap->assist_delta_run_native,
			(unsigned)snap->load_centikg,
			(unsigned)output.human_power_w, (unsigned)output.raw_motor_power_w,
			(unsigned)output.motor_power_w, (unsigned)output.applied_support_ratio_pct,
			(unsigned)output.startup_boost_extra_pct, (unsigned)output.cadence_comp_permille,
			(int)output.iq_request);
	}

	fclose(out);
	printf("%s: %u ticks (%.1f s) -> %s\n", sc->name, total_ticks, sc->duration_s, argv[2]);
	return 0;
}
