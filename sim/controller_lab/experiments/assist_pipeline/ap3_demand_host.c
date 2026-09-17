#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ap2_rider_demand.h"

#define HZ 4000U
#define PI 3.14159265358979323846

typedef struct {
	const char *name;
	uint8_t cadence;
	double duration_s;
	bool step;
	bool stop_restart;
	bool jitter;
	bool asymmetric;
} scenario_t;

static uint16_t pedal_force(double t, uint8_t cadence, double peak, bool asymmetric)
{
	double crank = t * ((double)cadence / 60.0) * 2.0 * PI;
	double s = fabs(sin(crank));
	double shaped = s * s;
	double half = fmod(crank, 2.0 * PI) < PI ? 1.0 : (asymmetric ? 0.72 : 1.0);
	double v = 120.0 + peak * shaped * half;
	if (v < 0.0) v = 0.0;
	if (v > 12000.0) v = 12000.0;
	return (uint16_t)llround(v);
}

static void run_scenario(FILE *f, const scenario_t *s)
{
	uint64_t wall_ticks = 0U;
	bool was_pedaling = false;
	ap2_rider_demand_reset();

	while ((double)wall_ticks / (double)HZ < s->duration_s) {
		double t = (double)wall_ticks / (double)HZ;
		uint32_t elapsed = 1U;
		bool pedaling = true;
		double peak = 3900.0;
		uint16_t load;
		ap2_demand_input_t in;
		ap2_demand_output_t out;

		if (s->jitter) {
			static const uint8_t pattern[] = {1,1,1,2,1,4,1,1,3,1,2,1,1,1,5,1};
			elapsed = pattern[(wall_ticks / 7U) % (sizeof(pattern) / sizeof(pattern[0]))];
		}
		if (s->step) {
			if (t < 3.0) peak = 2500.0;
			else if (t < 5.0) peak = 4700.0;
			else peak = 2500.0;
		}
		if (s->stop_restart && t >= 3.0 && t < 3.6) {
			pedaling = false;
		}
		load = pedaling ? pedal_force(t, s->cadence, peak, s->asymmetric) : 0U;

		memset(&in, 0, sizeof(in));
		memset(&out, 0, sizeof(out));
		in.load_centikg = load;
		in.torque_valid = true;
		in.pedaling = pedaling;
		in.cadence_rpm = pedaling ? s->cadence : 0U;
		in.full_scale_centikg = 6000U;
		in.base_hold_ms = 350U;
		in.elapsed_ticks = elapsed;
		ap2_rider_demand_update(&in, &out);

		/* Match assist_pipeline.c: seed sustained effort only on the engagement edge. */
		if (pedaling && !was_pedaling) {
			ap2_rider_demand_seed_base(out.demand_permille);
			out.base_permille = out.demand_permille;
			out.dynamic_permille = 0;
		}

		fprintf(f, "%s,%.6f,%u,%u,%u,%d,%d,%d,%d,%d,%u\n",
			s->name, t, (unsigned)s->cadence, pedaling ? 1U : 0U,
			(unsigned)load, out.effort_permille, out.demand_permille,
			out.base_permille, out.dynamic_permille, out.stroke_peak_permille,
			(unsigned)elapsed);
		was_pedaling = pedaling;
		wall_ticks += elapsed;
	}
}

int main(int argc, char **argv)
{
	static const scenario_t scenarios[] = {
		{"steady30",30,8.0,false,false,false,false},
		{"steady40",40,8.0,false,false,false,false},
		{"steady60",60,8.0,false,false,false,false},
		{"steady72",72,8.0,false,false,false,false},
		{"steady90",90,8.0,false,false,false,false},
		{"steady110",110,8.0,false,false,false,false},
		{"asym72",72,8.0,false,false,false,true},
		{"step72",72,8.0,true,false,false,true},
		{"stop_restart72",72,8.0,false,true,false,true},
		{"jitter72",72,8.0,false,false,true,true},
	};
	FILE *f;
	size_t i;
	if (argc != 2) {
		fprintf(stderr, "usage: %s output.csv\n", argv[0]);
		return 2;
	}
	f = fopen(argv[1], "wb");
	if (!f) return 2;
	fprintf(f, "scenario,time_s,cadence_rpm,pedaling,load_centikg,effort,demand,base,dynamic,peak,elapsed_ticks\n");
	for (i = 0U; i < sizeof(scenarios) / sizeof(scenarios[0]); ++i) {
		run_scenario(f, &scenarios[i]);
	}
	fclose(f);
	return 0;
}
