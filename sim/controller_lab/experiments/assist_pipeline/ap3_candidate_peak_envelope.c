#include "ap2_rider_demand.h"

#include "ap2_math.h"

/*
 * AP3 EXPERIMENT ONLY - candidate A.
 *
 * Keep the production input contract unchanged, but model the dead spot using a decaying
 * stroke-peak envelope. 65 % of that envelope is allowed to support the sustained base while
 * forward pedalling remains true. The fast dynamic term is computed against the PREVIOUS base,
 * so a genuine harder push is not swallowed by the base estimator on the same tick.
 *
 * This file is deliberately outside src/. It is linked in place of src/ap2_rider_demand.c only
 * by the AP3 experiment runner. It is not production firmware.
 */

typedef struct {
	uint16_t hist[3];
	uint8_t hist_count;
	int32_t effort_q16;
	int32_t base_q16;
	int32_t dynamic_q16;
	int32_t peak_q16;
} ap3_peak_ctx_t;

static ap3_peak_ctx_t ctx;

void ap2_rider_demand_reset(void)
{
	ctx.hist[0] = 0U;
	ctx.hist[1] = 0U;
	ctx.hist[2] = 0U;
	ctx.hist_count = 0U;
	ctx.effort_q16 = 0;
	ctx.base_q16 = 0;
	ctx.dynamic_q16 = 0;
	ctx.peak_q16 = 0;
}

void ap2_rider_demand_seed_base(int32_t permille)
{
	ctx.base_q16 = ap2_q16_from(ap2_clamp(permille, 0, AP2_PERMILLE));
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
	if (cadence_rpm == 0U) {
		return (uint16_t)AP2_STROKE_MAX_MS;
	}
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
	uint32_t peak_decay_ms;
	uint32_t base_fall_ms;
	int32_t effort;
	int32_t demand;
	int32_t base_before;
	int32_t base;
	int32_t peak;
	int32_t floor_target;
	int32_t base_target;
	int32_t excess;
	int32_t dynamic;

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

	/* A stroke peak is accepted immediately and forgotten over two half-revolutions. */
	peak = ap2_q16_value(ctx.peak_q16);
	if (demand >= peak) {
		ctx.peak_q16 = ap2_q16_from(demand);
	} else {
		peak_decay_ms = (uint32_t)period * 2U;
		if (!in->pedaling) { peak_decay_ms = period; }
		ctx.peak_q16 = ap2_lpf_step(ctx.peak_q16, demand, peak_decay_ms,
			in->elapsed_ticks);
	}
	peak = ap2_clamp(ap2_q16_value(ctx.peak_q16), 0, AP2_PERMILLE);

	base_before = ap2_clamp(ap2_q16_value(ctx.base_q16), 0, AP2_PERMILLE);
	floor_target = in->pedaling ? ap2_scale_permille(peak, 650) : 0;
	base_target = (demand > floor_target) ? demand : floor_target;
	base_fall_ms = period;
	if (in->base_hold_ms > base_fall_ms) { base_fall_ms = in->base_hold_ms; }
	if (base_target >= base_before) {
		ctx.base_q16 = ap2_lpf_step(ctx.base_q16, base_target, 80U, in->elapsed_ticks);
	} else {
		ctx.base_q16 = ap2_lpf_step(ctx.base_q16, base_target, base_fall_ms,
			in->elapsed_ticks);
	}
	base = ap2_clamp(ap2_q16_value(ctx.base_q16), 0, AP2_PERMILLE);

	/* Preserve a fast correction before the sustained estimator consumes the same edge. */
	excess = demand - base_before;
	if (excess < 0) { excess = 0; }
	if (excess >= ap2_q16_value(ctx.dynamic_q16)) {
		ctx.dynamic_q16 = ap2_lpf_step(ctx.dynamic_q16, excess, 20U, in->elapsed_ticks);
	} else {
		ctx.dynamic_q16 = ap2_lpf_step(ctx.dynamic_q16, excess, 120U, in->elapsed_ticks);
	}
	dynamic = ap2_clamp(ap2_q16_value(ctx.dynamic_q16), 0, AP2_PERMILLE);

	out->effort_permille = effort;
	out->demand_permille = demand;
	out->base_permille = base;
	out->dynamic_permille = dynamic;
	out->stroke_peak_permille = peak;
	out->stroke_period_ms = period;
	out->load_centikg = in->load_centikg;
}
