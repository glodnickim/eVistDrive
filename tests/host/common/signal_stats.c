#include "signal_stats.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int compare_double(const void *a, const void *b)
{
	double da = *(const double *)a;
	double db = *(const double *)b;
	if (da < db) return -1;
	if (da > db) return 1;
	return 0;
}

static double percentile(const double *sorted, int count, double fraction)
{
	if (count == 1) return sorted[0];
	double pos = fraction * (double)(count - 1);
	int lo = (int)floor(pos);
	int hi = (int)ceil(pos);
	if (lo == hi) return sorted[lo];
	double frac = pos - (double)lo;
	return sorted[lo] + (sorted[hi] - sorted[lo]) * frac;
}

signal_stats_t signal_stats_compute(const double *values, int count)
{
	signal_stats_t out;
	memset(&out, 0, sizeof(out));
	out.count = count;
	if (count <= 0) {
		return out;
	}

	double *sorted = (double *)malloc((size_t)count * sizeof(double));
	if (sorted == NULL) {
		/* Test infrastructure, not firmware - abort loudly rather than silently
		 * truncate the sample set and report a wrong percentile. */
		fprintf(stderr, "signal_stats_compute: out of memory for %d samples\n", count);
		exit(1);
	}
	memcpy(sorted, values, (size_t)count * sizeof(double));
	qsort(sorted, (size_t)count, sizeof(double), compare_double);
	int n = count;

	double sum = 0.0;
	for (int i = 0; i < count; i++) sum += values[i];
	out.mean = sum / (double)count;

	double variance = 0.0;
	for (int i = 0; i < count; i++) {
		double d = values[i] - out.mean;
		variance += d * d;
	}
	out.stddev = sqrt(variance / (double)count);

	out.min = sorted[0];
	out.max = sorted[n - 1];
	out.p5 = percentile(sorted, n, 0.05);
	out.p95 = percentile(sorted, n, 0.95);
	out.ptp = out.max - out.min;
	out.ripple = (fabs(out.mean) > 1.0) ? (out.p95 - out.p5) / out.mean : RIPPLE_NOT_MEANINGFUL;
	free(sorted);
	return out;
}

void running_stats_reset(running_stats_t *acc)
{
	acc->count = 0;
	acc->mean = 0.0;
	acc->m2 = 0.0;
	acc->min = 0.0;
	acc->max = 0.0;
}

void running_stats_add(running_stats_t *acc, double value)
{
	acc->count++;
	if (acc->count == 1) {
		acc->min = value;
		acc->max = value;
	} else {
		if (value < acc->min) acc->min = value;
		if (value > acc->max) acc->max = value;
	}
	double delta = value - acc->mean;
	acc->mean += delta / (double)acc->count;
	double delta2 = value - acc->mean;
	acc->m2 += delta * delta2;
}

double running_stats_mean(const running_stats_t *acc)
{
	return acc->mean;
}

double running_stats_stddev(const running_stats_t *acc)
{
	if (acc->count < 2) return 0.0;
	return sqrt(acc->m2 / (double)acc->count);
}

double running_stats_min(const running_stats_t *acc) { return acc->min; }
double running_stats_max(const running_stats_t *acc) { return acc->max; }
