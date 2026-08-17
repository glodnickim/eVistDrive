/*
 * TEST-002 / TEST B — power pipeline benchmark, revolution-windowed, controllable
 * motor_voltage_utilization and battery voltage.
 *
 * Runs the SHIPPED src/torque_input.c, rider_input.c, assist_modes.c, cadence_comp.c,
 * power_curve.c, assist_start.c, assist_extended_boost.c, tuning_config.c, ride_control.c,
 * assist_dynamics.c, assist_limits.c, motor_core.c - not copies of them. Same linking
 * pattern (host_stubs + map_adapter + motor_service_stub) as
 * tests/host/pipeline/ride_control_pipeline_host.c from TEST-001 - see
 * documentation/testing/TEST_INTERFACES.md for why those three adapters are needed and
 * why they are safe (zero production files changed).
 *
 * Purpose (card section 8, "TEST B"): whether the power/Iq LOGIC reduces output as
 * cadence or motor_voltage_utilization rises - isolated from the torque filters
 * themselves (that is TEST A's question, in torque_revolution_bench_host.c). Do not mix
 * conclusions from the two (card, same section).
 *
 * motor_voltage_utilization here is a CONTROLLED SWEEP INPUT to the existing algorithm,
 * not a physical model of the motor/FOC (card sections 6, 18). It is fed to
 * rider_input_t.motor_voltage_utilization every tick, at a value fixed for the whole run
 * - this program does not compute it from anything.
 *
 * Usage:
 *   power_revolution_bench_host <cadence_rpm> <profile_name> <battery_voltage_mv> \
 *       <motor_voltage_utilization> <out_dir> <run_tag> [--per-rev]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "assist_modes.h"
#include "config.h"
#include "crank_model.h"
#include "csv.h"
#include "motor_core.h"
#include "rider_input.h"
#include "ride_control.h"
#include "scenario_profiles.h"
#include "signal_stats.h"
#include "torque_input.h"

#define RUN_WINDOW_DEG_DEFAULT 180U
#define ASSIST_LEVEL_INDEX 3U
#define WHEEL_SPEED_X100_FIXED 1500U
#define VOLTAGE_RAW_FIXED 2000 /* comfortably above voltage_min_raw+176 -> assist_limits does not clamp */
#define TEMPERATURE_C_FIXED 25
#define WHOLE_WINDOW_CAPACITY 300000

static void ensure_summary_header(const char *path, const char *header)
{
	FILE *probe = fopen(path, "r");
	if (probe != NULL) { fclose(probe); return; }
	FILE *f = fopen(path, "w");
	if (f == NULL) { fprintf(stderr, "could not create summary file: %s\n", path); exit(1); }
	fprintf(f, "%s\n", header);
	fclose(f);
}

int main(int argc, char **argv)
{
	if (argc < 7) {
		fprintf(stderr,
			"usage: %s <cadence_rpm> <profile_name> <battery_voltage_mv> <motor_voltage_utilization> <out_dir> <run_tag> [--per-rev]\n",
			argv[0]);
		return 2;
	}
	double cadence_rpm = atof(argv[1]);
	const char *profile_name = argv[2];
	uint32_t battery_voltage_mv = (uint32_t)atol(argv[3]);
	int32_t mvu = (int32_t)atol(argv[4]);
	const char *out_dir = argv[5];
	const char *run_tag = argv[6];
	bool per_rev = (argc >= 8 && strcmp(argv[7], "--per-rev") == 0);

	const crank_torque_shape_t *shape = find_named_profile(profile_name);
	if (shape == NULL) {
		fprintf(stderr, "unknown profile: %s\n", profile_name);
		return 2;
	}

	torque_input_init();
	torque_input_set_run_window_deg(RUN_WINDOW_DEG_DEFAULT);
	assist_modes_init();
	assist_modes_set_active_bank(0);

	static MotorState_t MS;
	motor_core_init(&MS);
	ride_control_init();

	crank_state_t crank;
	crank_state_init(&crank);

	uint32_t warmup_revs = DEFAULT_WARMUP_REVOLUTIONS;
	uint32_t measure_revs = DEFAULT_MEASURE_REVOLUTIONS;
	uint32_t total_revs = warmup_revs + measure_revs;

	double *w_human = malloc(WHOLE_WINDOW_CAPACITY * sizeof(double));
	double *w_motor_raw = malloc(WHOLE_WINDOW_CAPACITY * sizeof(double));
	double *w_motor = malloc(WHOLE_WINDOW_CAPACITY * sizeof(double));
	double *w_iq_req = malloc(WHOLE_WINDOW_CAPACITY * sizeof(double));
	double *w_iq_final = malloc(WHOLE_WINDOW_CAPACITY * sizeof(double));
	long w_count = 0;
	if (!w_human || !w_motor_raw || !w_motor || !w_iq_req || !w_iq_final) {
		fprintf(stderr, "out of memory\n");
		return 1;
	}

	char per_rev_path[1024], summary_path[1024];
	snprintf(per_rev_path, sizeof(per_rev_path), "%s/power_per_revolution_%s.csv", out_dir, run_tag);
	snprintf(summary_path, sizeof(summary_path), "%s/power_summary.csv", out_dir);

	FILE *per_rev_f = NULL;
	if (per_rev) {
		per_rev_f = csv_open_or_die(per_rev_path,
			"revolution_index,n_ticks,mean_human_power_w,mean_motor_power_w,mean_iq_request,mean_iq_final,"
			"ripple_motor_power_w,ripple_iq_final");
	}
	double *rev_human = per_rev ? malloc(20000 * sizeof(double)) : NULL;
	double *rev_motor = per_rev ? malloc(20000 * sizeof(double)) : NULL;
	double *rev_iqreq = per_rev ? malloc(20000 * sizeof(double)) : NULL;
	double *rev_iqfin = per_rev ? malloc(20000 * sizeof(double)) : NULL;
	int rev_count = 0;

	uint32_t last_rev_index = 0;
	uint32_t tick = 0;
	while (true) {
		crank_state_advance_tick(&crank, cadence_rpm);
		uint32_t rev_index = (uint32_t)(crank.cumulative_deg / 360.0);
		bool measuring = (rev_index >= warmup_revs);
		bool last_was_measuring = (last_rev_index >= warmup_revs);

		/* Finalize the revolution that just ended, same ordering fix as TEST A (see
		 * torque_revolution_bench_host.c for why order matters here). */
		if (per_rev && rev_index != last_rev_index && last_was_measuring && rev_count > 0) {
			signal_stats_t motor_stats = signal_stats_compute(rev_motor, rev_count);
			signal_stats_t iqfin_stats = signal_stats_compute(rev_iqfin, rev_count);
			double sum_h = 0, sum_iqr = 0, sum_iqf = 0;
			for (int i = 0; i < rev_count; i++) { sum_h += rev_human[i]; sum_iqr += rev_iqreq[i]; }
			(void)sum_iqf;
			fprintf(per_rev_f, "%u,%d,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f\n",
				last_rev_index - warmup_revs, rev_count,
				sum_h / rev_count, motor_stats.mean, sum_iqr / rev_count, iqfin_stats.mean,
				motor_stats.ripple, iqfin_stats.ripple);
			rev_count = 0;
		}

		uint16_t raw_mv = crank_torque_raw_mv(&crank, shape);
		int16_t corrected = torque_input_correct(raw_mv);
		{
			static uint32_t last_step_count = 0;
			uint32_t steps_now = crank.step_count - last_step_count;
			last_step_count = crank.step_count;
			for (uint32_t i = 0; i < steps_now; i++) torque_input_run_filter_step();
		}
		torque_input_coast_update(corrected, false, true);
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
		sample.motor_voltage_utilization = (uint16_t)mvu; /* THE controlled sweep input */
		sample.pas_forward = true;
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
		ride_input.battery_voltage_mv = battery_voltage_mv;
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

		/*
		 * NOTE on inferring whether the power/voltage-utilization cross-check inside
		 * assist_modes.c's finish_power_request() is the binding constraint (card
		 * section 7 - no exported flag exists for this, see
		 * documentation/assist/POWER_MODE.md "KNOWN ISSUES"): an EARLIER version of
		 * this harness tried to answer that in-process, by calling
		 * assist_modes_calculate() a second time per tick with motor_voltage_utilization
		 * forced to 0 and comparing the two outputs. That was WRONG and was removed
		 * during this card's own validation: assist_modes.c's power filter
		 * (filter_motor_power(), a rise/fall-ms low-pass) is STATEFUL across calls, so
		 * the extra probe call altered the filter's trajectory for the REAL run too,
		 * contaminating the very measurement it was trying to take honestly. The clean
		 * way to get an unclamped reference is a SEPARATE process run with
		 * motor_voltage_utilization=0 (MVU_SWEEP already includes 0 for exactly this
		 * reason) - the two runs never share process state, so neither can perturb the
		 * other. The "clamp delta" is computed in post-processing by joining rows of
		 * power_summary.csv at matching cadence/profile/voltage (see
		 * tests/host/tools/HighCadenceTools.ps1).
		 */

		if (measuring) {
			if (w_count < WHOLE_WINDOW_CAPACITY) {
				w_human[w_count] = mode_out->human_power_w;
				w_motor_raw[w_count] = mode_out->raw_motor_power_w;
				w_motor[w_count] = mode_out->motor_power_w;
				w_iq_req[w_count] = mode_out->iq_request;
				w_iq_final[w_count] = MS.i_q_setpoint;
				w_count++;
			}
			if (per_rev && rev_count < 20000) {
				rev_human[rev_count] = mode_out->human_power_w;
				rev_motor[rev_count] = mode_out->motor_power_w;
				rev_iqreq[rev_count] = mode_out->iq_request;
				rev_iqfin[rev_count] = MS.i_q_setpoint;
				rev_count++;
			}
		}

		last_rev_index = rev_index;
		tick++;
		if (rev_index >= total_revs) break;
	}

	if (per_rev_f) fclose(per_rev_f);

	signal_stats_t human_s = signal_stats_compute(w_human, (int)w_count);
	signal_stats_t motor_raw_s = signal_stats_compute(w_motor_raw, (int)w_count);
	signal_stats_t motor_s = signal_stats_compute(w_motor, (int)w_count);
	signal_stats_t iq_req_s = signal_stats_compute(w_iq_req, (int)w_count);
	signal_stats_t iq_final_s = signal_stats_compute(w_iq_final, (int)w_count);

	double duration_s = (double)tick / CRANK_MODEL_TICK_HZ;
	char header[1200];
	snprintf(header, sizeof(header),
		"run_tag,cadence_rpm,profile,battery_voltage_mv,motor_voltage_utilization,"
		"warmup_revolutions,measured_revolutions,ticks_total,duration_s,"
		"human_power_w_mean,human_power_w_min,human_power_w_max,human_power_w_stddev,"
		"motor_power_raw_w_mean,motor_power_w_mean,motor_power_w_min,motor_power_w_max,motor_power_w_stddev,motor_power_w_ripple,"
		"iq_request_mean,iq_request_min,iq_request_max,iq_request_stddev,"
		"iq_final_mean,iq_final_min,iq_final_max,iq_final_stddev");
	ensure_summary_header(summary_path, header);
	FILE *summary_f = fopen(summary_path, "a");
	fprintf(summary_f,
		"%s,%.2f,%s,%u,%d,%u,%u,%u,%.4f,"
		"%.4f,%.4f,%.4f,%.4f,"
		"%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,"
		"%.4f,%.4f,%.4f,%.4f,"
		"%.4f,%.4f,%.4f,%.4f\n",
		run_tag, cadence_rpm, profile_name, battery_voltage_mv, mvu,
		warmup_revs, measure_revs, tick, duration_s,
		human_s.mean, human_s.min, human_s.max, human_s.stddev,
		motor_raw_s.mean, motor_s.mean, motor_s.min, motor_s.max, motor_s.stddev, motor_s.ripple,
		iq_req_s.mean, iq_req_s.min, iq_req_s.max, iq_req_s.stddev,
		iq_final_s.mean, iq_final_s.min, iq_final_s.max, iq_final_s.stddev);
	fclose(summary_f);

	free(w_human); free(w_motor_raw); free(w_motor); free(w_iq_req); free(w_iq_final);
	if (rev_human) free(rev_human);
	if (rev_motor) free(rev_motor);
	if (rev_iqreq) free(rev_iqreq);
	if (rev_iqfin) free(rev_iqfin);

	printf("%s (%s, %.0f rpm, %uV, mvu=%d): iq_request=%.2f iq_final=%.2f -> %s\n",
		run_tag, profile_name, cadence_rpm, battery_voltage_mv / 1000U, mvu,
		iq_req_s.mean, iq_final_s.mean, summary_path);
	return 0;
}
