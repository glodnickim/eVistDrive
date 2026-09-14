/*
 * ASSIST CHAIN TRACE HARNESS - runs the SHIPPED assist pipeline, not a copy of it.
 *
 * This is the primary vehicle for RUN_60..RUN_120 and CADENCE_RAMP_50_120
 * (documentation/testing/REGRESSION_SCENARIOS.md). It answers the question the regression
 * asks first - "did the demand model's behaviour against crank angle and cadence change" -
 * over a realistic pulsating pedal torque, without the surrounding lifecycle noise.
 *
 * It replaces the L2/L3 power_pipeline harness, which drove assist_modes_calculate() - the
 * legacy mode arithmetic that no longer exists. The scenarios, the crank model and the torque
 * shape are unchanged, so the traces remain comparable in SHAPE across the replacement even
 * though the columns now name the new signals.
 *
 * THE COLUMN THAT MATTERS. torque_fast is the rider's pulsating pedal force and iq_request is
 * what the motor is asked for. The whole point of the demand model is that the second is NOT a
 * copy of the first: assist_base holds the sustained level across the dead spot while
 * assist_dynamic carries the genuine changes. A regression that flattens that distinction shows
 * up here as iq_request ripple tracking torque ripple.
 *
 * Fixed assumptions, documented because they are NOT scenario parameters:
 *   - assist level index 3 of bank 0 - a representative mid-level (SPORT in the shipped bank).
 *   - battery 42000 mV, phase-current ceiling PH_CURRENT_MAX (config.h) - a plausible healthy
 *     operating point, not swept.
 *   - u_abs = 1024 (about 50 % duty). Unlike the harness this replaces, it is NOT zero: the
 *     power ceiling converts at the measured duty, and a zero duty would silently disable that
 *     whole limiter stage for every sample in the trace.
 *   - the PAS lifecycle is driven into FORWARD and held there; start/stop behaviour is the
 *     ride_control_pipeline harness's job, deliberately kept separate.
 *
 * Usage: assist_pipeline_host <scenario> <output.csv>
 */

#include <stdio.h>
#include <string.h>

#include "assist_modes.h"
#include "assist_pipeline.h"
#include "config.h"
#include "crank_model.h"
#include "csv.h"
#include "torque_input.h"

static const crank_torque_shape_t SHARED_SHAPE = {
	.mean_native_delta = 300.0,
	.ripple_pct = 40.0,
	.asymmetry_pct = 15.0,
	.dead_spot_depth_pct = 30.0,
	.dead_spot_width_deg = 20.0,
	.phase_shift_deg = 0.0
};

/*
 * A CRUISING effort, for the scenarios that measure ripple attenuation.
 *
 * SHARED_SHAPE is a hard push - about 16 kg of mean pedal force. At a mid assist level that
 * saturates the response at the ceiling, and a saturated output has no ripple at all. Measuring
 * "the motor does not copy the pedal" there would pass for entirely the wrong reason: a clipped
 * signal is smooth because it is clipped, not because the demand model works.
 *
 * These scenarios therefore use a realistic steady-cruise force, where the response sits in the
 * middle of its range and any pulsation the model failed to absorb has somewhere to show.
 */
static const crank_torque_shape_t CRUISE_SHAPE = {
	.mean_native_delta = 90.0,
	.ripple_pct = 40.0,
	.asymmetry_pct = 15.0,
	.dead_spot_depth_pct = 30.0,
	.dead_spot_width_deg = 20.0,
	.phase_shift_deg = 0.0
};

#define ASSIST_LEVEL_INDEX 3U
#define BATTERY_VOLTAGE_MV 42000U
#define WHEEL_SPEED_X100_FIXED 1500U /* 15.00 km/h - a representative "already rolling" speed */
#define U_ABS_FIXED 1024            /* about 50 % duty; see the header note */
#define CAL_I_FIXED 95

typedef struct {
	const char *name;
	double cadence_rpm;
	double duration_s;
	int is_ramp;
	double ramp_start_rpm;
	double ramp_end_rpm;
	double ramp_duration_s;
	const crank_torque_shape_t *shape;
	/* Which assist level of bank 0 to run. 0 uses ASSIST_LEVEL_INDEX. */
	unsigned level;
} scenario_def_t;

/*
 * Ripple attenuation is measured across the CADENCE range and across the PROFILES, not at one
 * comfortable point.
 *
 * 20 and 40 rpm matter most: the dead spot between leg pushes is longest there, which is exactly
 * where a sustained term that collapses shows up, and exactly where the legacy path felt worst.
 * The profiles differ in how much of the request is reactive, so each one has to be shown to
 * absorb the pulsation on its own terms - a result for SPORT says nothing about ECO.
 *
 * Level indices in bank 0: 1 = ECO, 2 = TRAIL, 3 = SPORT, 4 = SPORT+.
 */
#define CRUISE(rpm, lvl, nm) { nm, (double)(rpm), 8.0, 0, 0, 0, 0, &CRUISE_SHAPE, (lvl) }

static const scenario_def_t SCENARIOS[] = {
	{ "RUN_60",  60.0,  6.0, 0, 0, 0, 0, &SHARED_SHAPE, 0U },
	{ "RUN_80",  80.0,  6.0, 0, 0, 0, 0, &SHARED_SHAPE, 0U },
	{ "RUN_100", 100.0, 6.0, 0, 0, 0, 0, &SHARED_SHAPE, 0U },
	{ "RUN_110", 110.0, 6.0, 0, 0, 0, 0, &SHARED_SHAPE, 0U },
	{ "RUN_120", 120.0, 6.0, 0, 0, 0, 0, &SHARED_SHAPE, 0U },
	{ "CADENCE_RAMP_50_120", 0.0, 14.0, 1, 50.0, 120.0, 10.0, &SHARED_SHAPE, 0U },

	CRUISE(20,  1U, "CRUISE_20_ECO"),
	CRUISE(20,  2U, "CRUISE_20_TRAIL"),
	CRUISE(20,  3U, "CRUISE_20_SPORT"),
	CRUISE(20,  4U, "CRUISE_20_SPORTPLUS"),
	CRUISE(40,  1U, "CRUISE_40_ECO"),
	CRUISE(40,  2U, "CRUISE_40_TRAIL"),
	CRUISE(40,  3U, "CRUISE_40_SPORT"),
	CRUISE(40,  4U, "CRUISE_40_SPORTPLUS"),
	CRUISE(60,  1U, "CRUISE_60_ECO"),
	CRUISE(60,  2U, "CRUISE_60_TRAIL"),
	CRUISE(60,  3U, "CRUISE_60_SPORT"),
	CRUISE(60,  4U, "CRUISE_60_SPORTPLUS"),
	CRUISE(80,  3U, "CRUISE_80_SPORT"),
	CRUISE(100, 3U, "CRUISE_100_SPORT"),
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
	assist_modes_init();
	assist_modes_set_active_bank(0);
	assist_pipeline_init();

	crank_state_t crank;
	crank_state_init(&crank);

	FILE *out = csv_open_or_die(argv[2],
		"tick,time_s,crank_angle_deg,pas_state,cadence_input,"
		"torque_raw,torque_corrected,torque_fast,torque_run,load_centikg,"
		"rider_demand,assist_base,assist_dynamic,assist_response,"
		"aggression,load_state,auto_factor,iq_request,iq_final");

	uint32_t total_ticks = (uint32_t)(sc->duration_s * CRANK_MODEL_TICK_HZ);

	for (uint32_t tick = 0; tick < total_ticks; tick++) {
		double t_s = (double)tick / CRANK_MODEL_TICK_HZ;
		double cadence_rpm = sc->is_ramp ?
			crank_cadence_ramp(sc->ramp_start_rpm, sc->ramp_end_rpm, sc->ramp_duration_s, t_s) :
			sc->cadence_rpm;

		(void)crank_state_advance_tick(&crank, cadence_rpm);
		uint16_t raw_mv = crank_torque_raw_mv(&crank, sc->shape);

		int16_t corrected = torque_input_correct(raw_mv);
		torque_input_coast_update(corrected, false, true);
		torque_input_update(raw_mv, corrected, true);

		const torque_snapshot_t *snap = torque_input_get_snapshot();

		assist_pipeline_input_t in;
		assist_pipeline_command_t cmd;
		memset(&in, 0, sizeof(in));
		in.torque_load_centikg = snap->load_centikg;
		in.torque_sensor_valid = true;
		in.cadence_rpm = (uint8_t)(cadence_rpm > 255.0 ? 255 : cadence_rpm);
		in.speed_x100 = WHEEL_SPEED_X100_FIXED;
		in.motor_erps = 300U;
		in.forward_valid = true;
		in.pas_sensor_valid = true;
		in.forward_steps = 250U;
		in.required_steps = 4U;
		in.assist_level_index = (uint8_t)(sc->level ? sc->level : ASSIST_LEVEL_INDEX);
		in.battery_voltage_mv = BATTERY_VOLTAGE_MV;
		in.battery_current_max = 15000;
		in.u_abs = U_ABS_FIXED;
		in.cal_i = CAL_I_FIXED;
		in.level_iq_limit = (int32_t)PH_CURRENT_MAX;
		in.phase_current_max = (int32_t)PH_CURRENT_MAX;
		in.voltage_raw = 4000U;
		in.voltage_min_raw = 2800;
		in.controller_temperature_c = 30;
		in.speed_limit_x100 = 2500U;
		in.elapsed_ticks = 1U;
		assist_pipeline_update(&in, &cmd);

		const assist_pipeline_telemetry_t *t = assist_pipeline_telemetry();

		fprintf(out, "%u,%.6f,%.3f,%u,%.3f,%u,%d,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			tick, t_s, crank.crank_angle_deg, crank_pas_state(&crank), cadence_rpm,
			(unsigned)raw_mv, (int)corrected,
			(unsigned)snap->assist_delta_filtered_native,
			(unsigned)snap->assist_delta_run_native,
			(unsigned)snap->load_centikg,
			(int)t->rider_demand_permille, (int)t->assist_base_permille,
			(int)t->assist_dynamic_permille, (int)t->assist_response_permille,
			(int)t->rider_aggression_permille, (int)t->load_state_permille,
			(int)t->auto_factor_permille,
			(int)t->iq_request_before_limits, (int)cmd.final_iq_request);
	}

	fclose(out);
	printf("%s: %u ticks (%.1f s) -> %s\n", sc->name, total_ticks, sc->duration_s, argv[2]);
	return 0;
}
