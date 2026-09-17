#include "ap2_rider_demand.h"

#include "ap2_math.h"

/*
 * AP3 EXPERIMENT ONLY - candidate B.
 *
 * Model the sustained rider effort as one constant-rate output state rather than an
 * exponential low-pass state. Small dead-spot dips therefore cannot collapse the base faster
 * merely because the error is small/large; the decay rate is explicit and cadence-aware.
 * The dynamic term is the fast positive residual against the PREVIOUS base.
 *
 * This file is linked in place of src/ap2_rider_demand.c only by the AP3 experiment runner.
 * It is not production firmware.
 */

typedef struct {
	uint16_t hist[3];
	uint8_t hist_count;
	int32_t effort_q16;
	int32_t base;
	int32_t dynamic_q16;
	int32_t peak_q16;
} ap3_slew_ctx_t;

static ap3_slew_ctx_t ctx;

#define AP3_BASE_RISE_MS             180U
#define AP3_BASE_FALL_STROKES_NUM      3U
#define AP3_BASE_FALL_STROKES_DEN      1U
#define AP3_BASE_STOP_FALL_MS         250U
#define AP3_DYNAMIC_RISE_MS            20U
#define AP3_DYNAMIC_FALL_MS           120U

void ap2_rider_demand_reset(void)
{
	ctx.hist[0] = 0U;
	ctx.hist[1] = 0U;
	ctx.hist[2] = 0U;
	ctx.hist_count = 0U;
	ctx.effort_q16 = 0;
	ctx.base = 0;
	ctx.dynamic_q16 = 0;
	ctx.peak_q16 = 0;
}

void ap2_rider_demand_seed_base(int32_t permille)
{
	ctx.base = ap2_clamp(permille, 0, AP2_PERMILLE);
}

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
	if (a > b) { uint16_t t = a; a = b; b = t; }
	if (b > c) { uint16_t t = b; b = c; c = t; }
	if (a > b) { b = a; }
	return b;
}

static uint16_t stroke_period_ms(uint8_t cadence_rpm)
{
	uint32_t ms;
	if (cadence_rpm == 0U) { return (uint16_t)AP2_STROKE_MAX_MS; }
	ms = 30000U / (uint32_t)cadence_rpm;
	if (ms < AP2_STROKE_MIN_MS) { ms = AP2_STROKE_MIN_MS; }
	if (ms > AP2_STROKE_MAX_MS) { ms = AP2_STROKE_MAX_MS; }
	return (uint16_t)ms;
}

static void zero_output(ap2_demand_output_t *out, uint16_t load, uint16_t period)
{
	out->effort_permille = 0;
	out->demand_permille = 0;
	out->base_permille = 0;
	out->dynamic_permille = 0;
	out->stroke_peak_permille = 0;
	out->stroke_period_ms = period;
	out->load_centikg = load;
}

void ap2_rider_demand_update(const ap2_demand_input_t *in, ap2_demand_output_t *out)
{
	uint16_t raw;
	uint16_t full_scale;
	uint16_t period;
	uint32_t base_fall_ms;
	int32_t effort;
	int32_t demand;
	int32_t base_before;
	int32_t excess;
	int32_t dynamic;
	int32_t peak;

	if (out == 0) { return; }
	if (in == 0) {
		ap2_rider_demand_reset();
		zero_output(out, 0U, (uint16_t)AP2_STROKE_MAX_MS);
		return;
	}

	period = stroke_period_ms(in->cadence_rpm);
	if (!in->torque_valid) {
		ap2_rider_demand_reset();
		zero_output(out, in->load_centikg, period);
		return;
	}

	raw = in->load_centikg;
	ctx.hist[2] = ctx.hist[1];
	ctx.hist[1] = ctx.hist[0];
	ctx.hist[0] = raw;
	if (ctx.hist_count < 3U) { ctx.hist_count++; }

	full_scale = (in->full_scale_centikg != 0U) ? in->full_scale_centikg :
		(uint16_t)AP2_FULL_SCALE_DEFAULT_CENTIKG;
	if (full_scale <= AP2_EFFORT_DEADBAND_CENTIKG) {
		full_scale = (uint16_t)(AP2_EFFORT_DEADBAND_CENTIKG + 1U);
	}
	{
		uint16_t robust = (ctx.hist_count >= 3U) ?
			median3(ctx.hist[0], ctx.hist[1], ctx.hist[2]) : raw;
		int32_t above = (int32_t)robust - (int32_t)AP2_EFFORT_DEADBAND_CENTIKG;
		if (above < 0) { above = 0; }
		effort = ap2_map(above, 0,
			(int32_t)full_scale - (int32_t)AP2_EFFORT_DEADBAND_CENTIKG,
			0, AP2_PERMILLE);
	}

	ctx.effort_q16 = ap2_lpf_step(ctx.effort_q16, effort, AP2_EFFORT_LPF_MS,
		in->elapsed_ticks);
	demand = ap2_clamp(ap2_q16_value(ctx.effort_q16), 0, AP2_PERMILLE);

	base_before = ctx.base;
	base_fall_ms = ((uint32_t)period * AP3_BASE_FALL_STROKES_NUM) /
		AP3_BASE_FALL_STROKES_DEN;
	if (in->base_hold_ms > base_fall_ms) { base_fall_ms = in->base_hold_ms; }
	if (!in->pedaling) { base_fall_ms = AP3_BASE_STOP_FALL_MS; }
	ctx.base = ap2_slew_step(ctx.base, demand, AP2_PERMILLE,
		AP3_BASE_RISE_MS, base_fall_ms, in->elapsed_ticks);
	ctx.base = ap2_clamp(ctx.base, 0, AP2_PERMILLE);

	/* A real increase is measured against the state that existed before this tick. */
	excess = demand - base_before;
	if (excess < 0) { excess = 0; }
	if (excess >= ap2_q16_value(ctx.dynamic_q16)) {
		ctx.dynamic_q16 = ap2_lpf_step(ctx.dynamic_q16, excess,
			AP3_DYNAMIC_RISE_MS, in->elapsed_ticks);
	} else {
		ctx.dynamic_q16 = ap2_lpf_step(ctx.dynamic_q16, excess,
			AP3_DYNAMIC_FALL_MS, in->elapsed_ticks);
	}
	dynamic = ap2_clamp(ap2_q16_value(ctx.dynamic_q16), 0, AP2_PERMILLE);

	peak = ap2_q16_value(ctx.peak_q16);
	if (demand >= peak) {
		ctx.peak_q16 = ap2_q16_from(demand);
	} else {
		ctx.peak_q16 = ap2_lpf_step(ctx.peak_q16, demand,
			(uint32_t)period * AP2_STROKE_PEAK_DECAY_STROKES, in->elapsed_ticks);
	}
	peak = ap2_clamp(ap2_q16_value(ctx.peak_q16), 0, AP2_PERMILLE);

	out->effort_permille = effort;
	out->demand_permille = demand;
	out->base_permille = ctx.base;
	out->dynamic_permille = dynamic;
	out->stroke_peak_permille = peak;
	out->stroke_period_ms = period;
	out->load_centikg = in->load_centikg;
}
