/*
 * TEST-002 / TEST A — torque filter benchmark, revolution-windowed.
 *
 * Runs the SHIPPED src/torque_input.c, not a copy of it. Inputs: crank angle, PAS
 * progression, torque waveform, cadence (via tests/host/common/crank_model.c, the same
 * stimulus generator TEST-001 used). Outputs: FAST, RUN only — no power, no limiter, no
 * FOC assumptions (card section 8, "TEST A"). Purpose: whether the torque filters
 * THEMSELVES change behaviour with cadence, isolated from everything downstream.
 *
 * Two windowing modes, so the TEST-001 methodology and this card's methodology can be
 * compared directly on identical code (card section 16):
 *
 *   TIME6S  the TEST-001 way: fixed 6.0 s, no warm-up exclusion, recording from tick 0.
 *   REV20   this card's way: DEFAULT_WARMUP_REVOLUTIONS discarded, then
 *           DEFAULT_MEASURE_REVOLUTIONS recorded — same revolution count at every
 *           cadence, so no scenario gets more averaging "for free" than another.
 *
 * A THIRD mode, WARMUP_SCAN, does not benchmark anything - it runs many revolutions and
 * reports how the per-revolution FAST/RUN mean evolves, to let
 * documentation/TEST_002_HIGH_CADENCE_BENCHMARK_REPORT_PL.md state a MEASURED warm-up
 * length instead of assuming the card's suggested 5-10 is enough (card section 5).
 *
 * Usage:
 *   torque_revolution_bench_host <cadence_rpm> <profile_name> <mode> <out_dir> <run_tag> [--full-trace]
 *   mode: REV20 | TIME6S | WARMUP_SCAN
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "crank_model.h"
#include "csv.h"
#include "scenario_profiles.h"
#include "signal_stats.h"
#include "torque_input.h"

#define RUN_WINDOW_DEG_DEFAULT 180U
#define BIN_WIDTH_DEG (360.0 / 96.0) /* same 3.75 deg granularity as the PAS quadrature step */
#define BIN_COUNT 96
#define WHOLE_WINDOW_CAPACITY 300000 /* generous fixed upper bound, see file header */
#define WARMUP_SCAN_REVOLUTIONS 60U

typedef struct {
	double sum_raw, sum_corrected;
	running_stats_t fast_cross_rev; /* per-revolution MEAN of this bin, across revolutions */
	running_stats_t run_cross_rev;
	uint32_t n_revolutions;
} phase_bin_accum_t;

static void ensure_summary_header(const char *path, const char *header)
{
	FILE *probe = fopen(path, "r");
	if (probe != NULL) {
		fclose(probe);
		return; /* already has a header from an earlier run this session */
	}
	FILE *f = fopen(path, "w");
	if (f == NULL) {
		fprintf(stderr, "could not create summary file: %s\n", path);
		exit(1);
	}
	fprintf(f, "%s\n", header);
	fclose(f);
}

int main(int argc, char **argv)
{
	if (argc < 6) {
		fprintf(stderr,
			"usage: %s <cadence_rpm> <profile_name> <REV20|TIME6S|WARMUP_SCAN> <out_dir> <run_tag> [--full-trace]\n",
			argv[0]);
		return 2;
	}
	double cadence_rpm = atof(argv[1]);
	const char *profile_name = argv[2];
	const char *mode = argv[3];
	const char *out_dir = argv[4];
	const char *run_tag = argv[5];
	bool full_trace = (argc >= 7 && strcmp(argv[6], "--full-trace") == 0);

	const crank_torque_shape_t *shape = find_named_profile(profile_name);
	if (shape == NULL) {
		fprintf(stderr, "unknown profile: %s\n", profile_name);
		return 2;
	}

	bool is_time6s = (strcmp(mode, "TIME6S") == 0);
	bool is_warmup_scan = (strcmp(mode, "WARMUP_SCAN") == 0);
	if (!is_time6s && !is_warmup_scan && strcmp(mode, "REV20") != 0) {
		fprintf(stderr, "unknown mode: %s\n", mode);
		return 2;
	}

	uint32_t warmup_revs = is_time6s ? 0U : (is_warmup_scan ? 0U : DEFAULT_WARMUP_REVOLUTIONS);
	uint32_t measure_revs = is_warmup_scan ? WARMUP_SCAN_REVOLUTIONS : DEFAULT_MEASURE_REVOLUTIONS;
	uint32_t total_revs = warmup_revs + measure_revs; /* only meaningful for REV20/WARMUP_SCAN */
	uint32_t time6s_ticks = (uint32_t)(6.0 * CRANK_MODEL_TICK_HZ);

	torque_input_init();
	torque_input_set_run_window_deg(RUN_WINDOW_DEG_DEFAULT);

	crank_state_t crank;
	crank_state_init(&crank);

	/* Whole-window buffers (every MEASURED tick, warm-up excluded) - reproduces exactly
	 * what a TIME6S-style single-window ripple metric measures, for either mode. */
	double *whole_fast = (double *)malloc(WHOLE_WINDOW_CAPACITY * sizeof(double));
	double *whole_run = (double *)malloc(WHOLE_WINDOW_CAPACITY * sizeof(double));
	long whole_count = 0;

	/* Per-revolution buffers - sized generously for the slowest cadence in this sweep. */
	int rev_buf_capacity = (int)(CRANK_MODEL_TICK_HZ * 60.0 / 30.0); /* covers cadence >= 30 rpm */
	double *rev_fast = (double *)malloc((size_t)rev_buf_capacity * sizeof(double));
	double *rev_run = (double *)malloc((size_t)rev_buf_capacity * sizeof(double));
	int rev_count = 0;

	if (!whole_fast || !whole_run || !rev_fast || !rev_run) {
		fprintf(stderr, "out of memory\n");
		return 1;
	}

	phase_bin_accum_t bins[BIN_COUNT];
	memset(bins, 0, sizeof(bins));
	double bin_sum_fast_this_rev[BIN_COUNT];
	double bin_sum_run_this_rev[BIN_COUNT];
	double bin_sum_raw_this_rev[BIN_COUNT];
	double bin_sum_corr_this_rev[BIN_COUNT];
	int bin_count_this_rev[BIN_COUNT];
	memset(bin_sum_fast_this_rev, 0, sizeof(bin_sum_fast_this_rev));
	memset(bin_sum_run_this_rev, 0, sizeof(bin_sum_run_this_rev));
	memset(bin_sum_raw_this_rev, 0, sizeof(bin_sum_raw_this_rev));
	memset(bin_sum_corr_this_rev, 0, sizeof(bin_sum_corr_this_rev));
	memset(bin_count_this_rev, 0, sizeof(bin_count_this_rev));

	char per_rev_path[1024], phase_bin_path[1024], trace_path[1024], summary_path[1024], warmup_scan_path[1024];
	snprintf(per_rev_path, sizeof(per_rev_path), "%s/torque_per_revolution_%s.csv", out_dir, run_tag);
	snprintf(phase_bin_path, sizeof(phase_bin_path), "%s/torque_phase_binned_%s.csv", out_dir, run_tag);
	snprintf(trace_path, sizeof(trace_path), "%s/torque_trace_%s.csv", out_dir, run_tag);
	snprintf(summary_path, sizeof(summary_path), "%s/torque_summary.csv", out_dir);
	snprintf(warmup_scan_path, sizeof(warmup_scan_path), "%s/torque_warmup_scan_%s.csv", out_dir, run_tag);

	FILE *per_rev_f = csv_open_or_die(per_rev_path,
		"revolution_index,n_ticks,mean_fast,p5_fast,p95_fast,ripple_fast,ptp_fast,"
		"mean_run,p5_run,p95_run,ripple_run,ptp_run");
	FILE *warmup_scan_f = NULL;
	if (is_warmup_scan) {
		warmup_scan_f = csv_open_or_die(warmup_scan_path,
			"revolution_index,mean_fast,mean_run,ripple_run");
	}
	FILE *trace_f = NULL;
	if (full_trace) {
		trace_f = csv_open_or_die(trace_path,
			"tick,time_s,crank_angle_deg,phase_bin,revolution_index_measured,"
			"torque_raw,torque_corrected,torque_fast,torque_run");
	}

	running_stats_t agg_mean_run, agg_ripple_run;
	running_stats_reset(&agg_mean_run);
	running_stats_reset(&agg_ripple_run);

	uint32_t last_rev_index = 0;
	uint32_t tick = 0;
	bool done = false;
	while (!done) {
		crank_state_advance_tick(&crank, cadence_rpm);
		uint32_t rev_index = (uint32_t)(crank.cumulative_deg / 360.0);
		bool measuring = is_time6s ? true : (rev_index >= warmup_revs);
		uint32_t measured_rev_index = is_time6s ? rev_index :
			(measuring ? (rev_index - warmup_revs) : 0);

		/*
		 * Revolution boundary crossed since the previous tick: finalize the
		 * revolution that just ended, BEFORE this tick's own sample is added to any
		 * buffer. Order matters - finalizing after accumulating would fold the new
		 * revolution's first sample into the old revolution's stats and then throw it
		 * away on reset, silently shrinking every revolution by one sample. Also
		 * guarded on `last_was_measuring`: the tick where warm-up ends and
		 * measurement begins is a rev_index change too, but the revolution that is
		 * "ending" (still inside warm-up) was never accumulated into rev_fast/rev_run
		 * (the accumulate step below only runs while `measuring`), so there is
		 * nothing to finalize there - without this guard `last_rev_index - warmup_revs`
		 * underflows for that one tick (caught by inspection during this card's own
		 * validation run, not a subtle production issue - see the report).
		 */
		bool last_was_measuring = is_time6s ? (tick > 0) : (last_rev_index >= warmup_revs);
		if (rev_index != last_rev_index && last_was_measuring && rev_count > 0) {
			signal_stats_t fast_stats = signal_stats_compute(rev_fast, rev_count);
			signal_stats_t run_stats = signal_stats_compute(rev_run, rev_count);
			uint32_t finished_measured_index = is_time6s ?
				last_rev_index : (last_rev_index - warmup_revs);
			fprintf(per_rev_f, "%u,%d,%.4f,%.4f,%.4f,%.6f,%.4f,%.4f,%.4f,%.4f,%.6f,%.4f\n",
				finished_measured_index, rev_count,
				fast_stats.mean, fast_stats.p5, fast_stats.p95, fast_stats.ripple, fast_stats.ptp,
				run_stats.mean, run_stats.p5, run_stats.p95, run_stats.ripple, run_stats.ptp);
			running_stats_add(&agg_mean_run, run_stats.mean);
			if (run_stats.ripple != RIPPLE_NOT_MEANINGFUL) {
				running_stats_add(&agg_ripple_run, run_stats.ripple);
			}
			if (warmup_scan_f) {
				fprintf(warmup_scan_f, "%u,%.4f,%.4f,%.6f\n",
					finished_measured_index, fast_stats.mean, run_stats.mean, run_stats.ripple);
			}

			for (int b = 0; b < BIN_COUNT; b++) {
				if (bin_count_this_rev[b] > 0) {
					double mean_fast_b = bin_sum_fast_this_rev[b] / bin_count_this_rev[b];
					double mean_run_b = bin_sum_run_this_rev[b] / bin_count_this_rev[b];
					running_stats_add(&bins[b].fast_cross_rev, mean_fast_b);
					running_stats_add(&bins[b].run_cross_rev, mean_run_b);
					bins[b].sum_raw += bin_sum_raw_this_rev[b] / bin_count_this_rev[b];
					bins[b].sum_corrected += bin_sum_corr_this_rev[b] / bin_count_this_rev[b];
					bins[b].n_revolutions++;
				}
			}
			memset(bin_sum_fast_this_rev, 0, sizeof(bin_sum_fast_this_rev));
			memset(bin_sum_run_this_rev, 0, sizeof(bin_sum_run_this_rev));
			memset(bin_sum_raw_this_rev, 0, sizeof(bin_sum_raw_this_rev));
			memset(bin_sum_corr_this_rev, 0, sizeof(bin_sum_corr_this_rev));
			memset(bin_count_this_rev, 0, sizeof(bin_count_this_rev));
			rev_count = 0;
		}

		uint16_t raw_mv = crank_torque_raw_mv(&crank, shape);
		int16_t corrected = torque_input_correct(raw_mv);
		/* One quadrature step every CRANK_MODEL_STEP_DEG - advance the RUN estimator
		 * exactly as many times as crank_state_advance_tick() reports new steps
		 * (see tests/host/common/crank_model.h; same call pattern as TEST-001). */
		{
			static uint32_t last_step_count = 0;
			uint32_t steps_now = crank.step_count - last_step_count;
			last_step_count = crank.step_count;
			for (uint32_t i = 0; i < steps_now; i++) {
				torque_input_run_filter_step();
			}
		}
		torque_input_coast_update(corrected, false, true);
		torque_input_update(raw_mv, corrected, true);
		const torque_snapshot_t *snap = torque_input_get_snapshot();

		uint32_t bin = (uint32_t)(crank.crank_angle_deg / BIN_WIDTH_DEG);
		if (bin >= BIN_COUNT) bin = BIN_COUNT - 1;

		if (measuring) {
			if (whole_count < WHOLE_WINDOW_CAPACITY) {
				whole_fast[whole_count] = snap->assist_delta_filtered_native;
				whole_run[whole_count] = snap->assist_delta_run_native;
				whole_count++;
			}
			if (rev_count < rev_buf_capacity) {
				rev_fast[rev_count] = snap->assist_delta_filtered_native;
				rev_run[rev_count] = snap->assist_delta_run_native;
				rev_count++;
			}
			bin_sum_fast_this_rev[bin] += snap->assist_delta_filtered_native;
			bin_sum_run_this_rev[bin] += snap->assist_delta_run_native;
			bin_sum_raw_this_rev[bin] += raw_mv;
			bin_sum_corr_this_rev[bin] += corrected;
			bin_count_this_rev[bin]++;
		}

		if (full_trace) {
			fprintf(trace_f, "%u,%.6f,%.3f,%u,%d,%u,%d,%u,%u\n",
				tick, (double)tick / CRANK_MODEL_TICK_HZ, crank.crank_angle_deg, bin,
				measuring ? (int)measured_rev_index : -1,
				(unsigned)raw_mv, (int)corrected,
				(unsigned)snap->assist_delta_filtered_native,
				(unsigned)snap->assist_delta_run_native);
		}

		last_rev_index = rev_index;
		tick++;
		if (is_time6s) {
			done = (tick >= time6s_ticks);
		} else {
			done = (rev_index >= total_revs);
		}
	}

	fclose(per_rev_f);
	if (warmup_scan_f) fclose(warmup_scan_f);
	if (trace_f) fclose(trace_f);

	/* Phase-binned averaged waveform. */
	FILE *bin_f = csv_open_or_die(phase_bin_path,
		"bin_index,bin_angle_deg,n_revolutions,mean_raw,mean_corrected,mean_fast,stddev_fast,mean_run,stddev_run");
	for (int b = 0; b < BIN_COUNT; b++) {
		double n = (bins[b].n_revolutions > 0) ? (double)bins[b].n_revolutions : 1.0;
		fprintf(bin_f, "%d,%.3f,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
			b, b * BIN_WIDTH_DEG, bins[b].n_revolutions,
			bins[b].sum_raw / n, bins[b].sum_corrected / n,
			running_stats_mean(&bins[b].fast_cross_rev), running_stats_stddev(&bins[b].fast_cross_rev),
			running_stats_mean(&bins[b].run_cross_rev), running_stats_stddev(&bins[b].run_cross_rev));
	}
	fclose(bin_f);

	/* Whole-window (single, undivided measurement window) reference metric. */
	signal_stats_t whole_fast_stats = signal_stats_compute(whole_fast, (int)whole_count);
	signal_stats_t whole_run_stats = signal_stats_compute(whole_run, (int)whole_count);

	/* Cross-revolution aggregate of the per-revolution metrics (card section 4: "nie
	 * licz jednej globalnej metryki jako jedynego źródła wniosku" - both whole-window and
	 * per-revolution-aggregate are written). agg_mean_run/agg_ripple_run were already
	 * accumulated inline as each revolution was finalized during the main loop above -
	 * see the running_stats_add() calls next to the per_rev_f fprintf(). */
	uint32_t measured_revolutions_written = (uint32_t)agg_mean_run.count;
	double duration_s = (double)tick / CRANK_MODEL_TICK_HZ;

	char header[1024];
	snprintf(header, sizeof(header),
		"run_tag,cadence_rpm,profile,mode,warmup_revolutions,measured_revolutions,ticks_total,duration_s,"
		"whole_window_mean_fast,whole_window_ripple_fast,whole_window_mean_run,whole_window_ripple_run,"
		"per_rev_mean_run_avg,per_rev_mean_run_min,per_rev_mean_run_max,per_rev_mean_run_stddev,"
		"per_rev_ripple_run_avg,per_rev_ripple_run_min,per_rev_ripple_run_max,per_rev_ripple_run_stddev");
	ensure_summary_header(summary_path, header);
	FILE *summary_f = fopen(summary_path, "a");
	fprintf(summary_f, "%s,%.2f,%s,%s,%u,%u,%u,%.4f,%.4f,%.6f,%.4f,%.6f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f\n",
		run_tag, cadence_rpm, profile_name, mode, warmup_revs, measured_revolutions_written, tick, duration_s,
		whole_fast_stats.mean, whole_fast_stats.ripple, whole_run_stats.mean, whole_run_stats.ripple,
		running_stats_mean(&agg_mean_run), running_stats_min(&agg_mean_run),
		running_stats_max(&agg_mean_run), running_stats_stddev(&agg_mean_run),
		running_stats_mean(&agg_ripple_run), running_stats_min(&agg_ripple_run),
		running_stats_max(&agg_ripple_run), running_stats_stddev(&agg_ripple_run));
	fclose(summary_f);

	free(whole_fast); free(whole_run); free(rev_fast); free(rev_run);

	printf("%s (%s, %.0f rpm, %s): %u ticks, %u revolutions measured -> %s\n",
		run_tag, profile_name, cadence_rpm, mode, tick, measured_revolutions_written, summary_path);
	return 0;
}
