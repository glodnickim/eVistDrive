/*
 * L3 full pipeline harness — runs the SHIPPED src/torque_input.c, src/rider_input.c,
 * src/assist_modes.c, src/ride_control.c, src/assist_dynamics.c, src/assist_limits.c,
 * src/assist_start.c, src/assist_extended_boost.c, src/cadence_comp.c,
 * src/power_curve.c, src/tuning_config.c and src/motor_core.c — not copies of them.
 * This is the DEEPEST layer the card asks for: rider/PAS/torque input all the way to
 * the final Iq command src/motor_core.c would hand to FOC (MotorState_t.i_q_setpoint).
 * FOC itself is explicitly out of scope (card section 4) — this harness stops at the
 * motor_core boundary, same place the architecture audit's section 13 draws the line.
 *
 * WHY THIS IS A SEPARATE HARNESS FROM power_pipeline_host.c, not the same one extended:
 * ride_control adds the ride latch (a current floor + hold timer), the Iq ramp
 * (assist_dynamics — several hundred ms to reach a new target) and the shared
 * voltage/temperature/legal-speed limiter. All three are real, wanted behaviour, but
 * they are ABOUT transitions and safety ceilings, not about steady-state torque/cadence
 * response — mixing them into the RUN_60..120 metrics would make a ripple caused by the
 * ramp indistinguishable from a ripple caused by the torque/cadence path itself. Keeping
 * this harness separate lets a comparison start at the simpler layer (power_pipeline)
 * and only reach for this one if the divergence is suspected to be downstream of
 * assist_modes.
 *
 * WHY THIS NEEDS A STUB HEADER DIRECTORY ON THE INCLUDE PATH: inc/motor_core.h includes
 * inc/main.h, which includes the real GD32 CMSIS vendor headers (register-level firmware
 * headers with no meaning on a PC). See tests/host/common/host_stubs/gd32f30x.h and
 * arm_math.h for exactly what is substituted and why - inc/main.h itself, and the
 * MotorState_t/MotorParams_t/PI_control_t structs it defines, are NOT modified; only the
 * two vendor headers underneath it are. Two more link-time adapters are needed and are
 * documented where they live: tests/host/common/map_adapter.c (mirrors src/main.c's
 * map(), see architecture audit finding F4) and
 * tests/host/common/motor_service_stub.c (Walk Assist / Hall calibration Iq producers -
 * out of scope, never reached because every scenario here sets walk_active=false and
 * position_calibration_active=false).
 *
 * Usage: ride_control_pipeline_host <scenario> <output.csv>
 */

#include <stdio.h>
#include <string.h>

#include "assist_modes.h"
#include "config.h"
#include "crank_model.h"
#include "csv.h"
#include "motor_core.h"
#include "rider_input.h"
#include "ride_control.h"
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
#define WHEEL_SPEED_X100_FIXED 1500U /* 15.00 km/h, well under the 25 km/h legal taper */
#define VOLTAGE_RAW_FIXED 2000       /* comfortably above voltage_min_raw+176 -> no clamp */
#define TEMPERATURE_C_FIXED 25       /* room temperature -> no thermal clamp */

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
	assist_modes_set_active_bank(0);

	static MotorState_t MS; /* zero-initialised: static storage duration */
	motor_core_init(&MS);
	ride_control_init();

	crank_state_t crank;
	crank_state_init(&crank);

	FILE *out = csv_open_or_die(argv[2],
		"tick,time_s,crank_angle_deg,pas_state,cadence_input,"
		"torque_raw,torque_corrected,torque_fast,torque_run,load_centikg,"
		"iq_request,iq_final,debug_flags");

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
		sample.torque_assist_filtered = snap->assist_delta_filtered_native;
		sample.torque_run_filtered = snap->assist_delta_run_native;
		sample.torque_load_centikg = snap->load_centikg;
		sample.cadence_rpm = (uint8_t)(cadence_rpm > 255.0 ? 255 : cadence_rpm);
		sample.wheel_speed_x100 = WHEEL_SPEED_X100_FIXED;
		sample.motor_erps = 0;
		sample.motor_voltage_utilization = 0;
		sample.pas_forward = true;
		sample.pas_backward = false;
		sample.pedaling_active = true;
		sample.crank_forward_steps = 250;
		sample.crank_direction_ok = true;
		sample.start_phase = false;
		sample.torque_sensor_valid = true;
		sample.pas_sensor_valid = true;
		rider_input_update(&sample);

		ride_control_input_t ride_input = { 0 };
		ride_input.speed_x100 = WHEEL_SPEED_X100_FIXED;
		ride_input.cadence_rpm = sample.cadence_rpm;
		ride_input.assist_level_index = ASSIST_LEVEL_INDEX;
		ride_input.battery_voltage_mv = BATTERY_VOLTAGE_MV;
		ride_input.iq_scale = (int32_t)PH_CURRENT_MAX;
		ride_input.ride_core_iq_limit = (int32_t)PH_CURRENT_MAX;
		ride_input.phase_current_max = (int32_t)PH_CURRENT_MAX;
		ride_input.current_iq = MS.i_q_setpoint;
		ride_input.current_id = MS.i_d_setpoint;
		ride_input.voltage_raw = VOLTAGE_RAW_FIXED;
		ride_input.voltage_min_raw = VOLTAGE_MIN;
		ride_input.controller_temperature_c = TEMPERATURE_C_FIXED;
		ride_input.cadence_filtered_x8 = (uint16_t)(sample.cadence_rpm * 8U);
		ride_input.speed_limit_x100 = SPEEDLIMIT;
		ride_input.legal_enabled = (LEGALFLAG != 0);
		ride_input.offroad = false;
		ride_input.walk_active = false;
		ride_input.position_calibration_active = false;
		ride_input.safety_cut_non_direction = false;   /* FW-109 v2: safety_cut split - see inc/ride_control.h */
		ride_input.throttle_iq = 0;
		ride_control_update(&ride_input);

		const assist_mode_output_t *mode_out = assist_modes_get_last_output();

		fprintf(out, "%u,%.6f,%.3f,%u,%.3f,%u,%d,%u,%u,%u,%d,%d,0x%02X\n",
			tick, t_s, crank.crank_angle_deg, crank_pas_state(&crank), cadence_rpm,
			(unsigned)raw_mv, (int)corrected,
			(unsigned)snap->assist_delta_filtered_native,
			(unsigned)snap->assist_delta_run_native,
			(unsigned)snap->load_centikg,
			(int)mode_out->iq_request, (int)MS.i_q_setpoint,
			(unsigned)ride_control_get_debug_flags());
	}

	fclose(out);
	printf("%s: %u ticks (%.1f s) -> %s\n", sc->name, total_ticks, sc->duration_s, argv[2]);
	return 0;
}
