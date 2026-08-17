/*
 * L1 torque/RUN filter harness — runs the SHIPPED src/torque_input.c, not a copy of it.
 *
 * Scenario: crank_angle -> PAS quadrature steps + torque waveform (tests/host/common/
 * crank_model.c) feeds torque_input.c tick by tick, in the exact call order and
 * arguments src/main.c:reg_ADC_processing uses (see the comment above main() below).
 * The output is one CSV row per 4 kHz tick with raw/corrected/FAST/RUN torque, so
 * documentation/testing/REGRESSION_SCENARIOS.md's RUN_60..RUN_120 and
 * CADENCE_RAMP_50_120 scenarios can be compared layer by layer.
 *
 * RUN_60 .. RUN_120 use the IDENTICAL torque shape (see SHARED_SHAPE below) — only the
 * commanded cadence differs. That is deliberate (see the audit card, section 7): the
 * point is NOT to test cadence compensation (which defaults OFF and is not touched
 * here), it is to see whether anything in this tor's behaviour vs crank angle changes
 * with cadence when it should not.
 *
 * Usage: torque_trace_host <scenario> <output.csv>
 *   scenario: RUN_60 | RUN_80 | RUN_100 | RUN_110 | RUN_120 | CADENCE_RAMP_50_120
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "crank_model.h"
#include "csv.h"
#include "torque_input.h"

/* Shared pedal-load shape for every RUN_* scenario (section 6/7 of the card): same
 * profile vs crank angle at every cadence. Values are a plausible moderate effort on
 * the default (uncalibrated) sensor curve — not tuned to match any specific rider. */
static const crank_torque_shape_t SHARED_SHAPE = {
	.mean_native_delta = 300.0,   /* ~14 kg on the default piecewise curve */
	.ripple_pct = 40.0,           /* moderate per-leg pulsation */
	.asymmetry_pct = 15.0,        /* one leg pushes harder than the other */
	.dead_spot_depth_pct = 30.0,  /* extra suppression at each leg boundary */
	.dead_spot_width_deg = 20.0,
	.phase_shift_deg = 0.0
};

/* FW-085's RUN window default (tuning_config.c: TUNING_TORQUE_RUN_WINDOW_DEG_DEFAULT).
 * main.c re-sends this every tick from tuning_config_assist_torque_run_window_deg();
 * this harness does not link tuning_config.c, so the value is fixed here instead. */
#define RUN_WINDOW_DEG_DEFAULT 180U

typedef struct {
	const char *name;
	double cadence_rpm;    /* constant-cadence scenarios; ignored for the ramp */
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
	/* Ramp 50->120 rpm over 10 s, then hold 120 rpm for 4 more so the tail is a
	 * steady-state reference the ramp itself can be compared against. */
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
		fprintf(stderr, "scenarios:");
		for (size_t i = 0; i < SCENARIO_COUNT; i++) {
			fprintf(stderr, " %s", SCENARIOS[i].name);
		}
		fprintf(stderr, "\n");
		return 2;
	}

	const scenario_def_t *sc = find_scenario(argv[1]);
	if (sc == NULL) {
		fprintf(stderr, "unknown scenario: %s\n", argv[1]);
		return 2;
	}

	torque_input_init();
	/* No torque_input_startup_zero() call: the module's compiled-in default zero
	 * (740 native, TORQUE_ZERO_TARGET_NATIVE) already matches
	 * CRANK_MODEL_TORQUE_ZERO_NATIVE, so a real boot-time zero average would settle on
	 * the same point here (no sensor noise is modelled) — calling it would add a step
	 * that changes nothing observable and would need to guess a plausible "rest" reading. */
	torque_input_set_run_window_deg(RUN_WINDOW_DEG_DEFAULT);

	crank_state_t crank;
	crank_state_init(&crank);

	FILE *out = csv_open_or_die(argv[2],
		"tick,time_s,crank_angle_deg,pas_state,cadence_input,"
		"torque_raw,torque_corrected,torque_fast,torque_run,load_centikg");

	uint32_t total_ticks = (uint32_t)(sc->duration_s * CRANK_MODEL_TICK_HZ);

	for (uint32_t tick = 0; tick < total_ticks; tick++) {
		double t_s = (double)tick / CRANK_MODEL_TICK_HZ;
		double cadence_rpm = sc->is_ramp ?
			crank_cadence_ramp(sc->ramp_start_rpm, sc->ramp_end_rpm, sc->ramp_duration_s, t_s) :
			sc->cadence_rpm;

		uint32_t new_steps = crank_state_advance_tick(&crank, cadence_rpm);
		uint16_t raw_mv = crank_torque_raw_mv(&crank, &SHARED_SHAPE);

		/* Same order src/main.c:reg_ADC_processing uses: correct first (main.c:1863),
		 * then the RUN estimator advances once per FORWARD quadrature step
		 * (main.c:1938, inside the decoder block), then coast re-zero bookkeeping
		 * (main.c:2105) and the FAST/RUN publish (main.c:2108). */
		int16_t corrected = torque_input_correct(raw_mv);
		for (uint32_t i = 0; i < new_steps; i++) {
			torque_input_run_filter_step();
		}
		torque_input_coast_update(corrected, /*coast_eligible=*/false, /*bike_moving=*/true);
		torque_input_set_run_window_deg(RUN_WINDOW_DEG_DEFAULT);
		torque_input_update(raw_mv, corrected, /*sensor_valid=*/true);

		const torque_snapshot_t *snap = torque_input_get_snapshot();

		fprintf(out, "%u,%.6f,%.3f,%u,%.3f,%u,%d,%u,%u,%u\n",
			tick, t_s, crank.crank_angle_deg, crank_pas_state(&crank), cadence_rpm,
			(unsigned)raw_mv, (int)corrected,
			(unsigned)snap->assist_delta_filtered_native,
			(unsigned)snap->assist_delta_run_native,
			(unsigned)snap->load_centikg);
	}

	fclose(out);
	printf("%s: %u ticks (%.1f s) -> %s\n", sc->name, total_ticks, sc->duration_s, argv[2]);
	return 0;
}
