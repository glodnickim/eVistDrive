/*
 * TEST-002: shared statistics helper for per-revolution and phase-binned metrics. Test
 * infrastructure, not firmware. Kept deliberately small - one pass to sort a copy of the
 * buffer for percentiles, plain arithmetic for the rest. No FFT, no framework (card
 * section 14: "nie buduj rozbudowanego FFT frameworka, jeśli nie jest potrzebny").
 */
#ifndef SIGNAL_STATS_H
#define SIGNAL_STATS_H

#include <stdbool.h>
#include <stddef.h>

typedef struct {
	double mean, min, max, p5, p95, ptp; /* peak-to-peak = max-min */
	double ripple;   /* (p95-p5)/mean, or RIPPLE_NOT_MEANINGFUL if |mean|<=1 */
	double stddev;
	int count;
} signal_stats_t;

/* Sentinel used wherever a ripple RATIO would divide by a near-zero mean - same guard
 * RegressionTools.ps1 uses (TEST-001), kept consistent here. */
#define RIPPLE_NOT_MEANINGFUL (-1.0)

/*
 * Computes stats over `values[0..count-1]`. Does not modify `values` - sorts a malloc'd
 * copy internally (freed before returning), so there is no fixed sample-count ceiling:
 * this is called both on a single revolution (a few thousand samples) and on a whole
 * multi-revolution measurement window (tens of thousands), and both need to be exact,
 * not truncated.
 */
signal_stats_t signal_stats_compute(const double *values, int count);

/*
 * Running (mean, stddev) accumulator across many small per-revolution values (e.g. 20
 * per-revolution means) - Welford's method, so it doesn't need to keep all 20 in memory
 * (it doesn't matter at this scale, but avoids yet another SIGNAL_STATS_MAX_SAMPLES-sized
 * buffer for something this small).
 */
typedef struct {
	int count;
	double mean;
	double m2;
	double min, max;
} running_stats_t;

void running_stats_reset(running_stats_t *acc);
void running_stats_add(running_stats_t *acc, double value);
double running_stats_mean(const running_stats_t *acc);
double running_stats_stddev(const running_stats_t *acc);
double running_stats_min(const running_stats_t *acc);
double running_stats_max(const running_stats_t *acc);

#endif /* SIGNAL_STATS_H */
