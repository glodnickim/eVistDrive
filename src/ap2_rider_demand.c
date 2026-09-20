#include "ap2_rider_demand.h"

#include "ap2_math.h"

/* See inc/ap2_rider_demand.h for the model. This file implements it and owns nothing else. */

typedef struct {
	uint16_t hist[3];        /* last three raw force samples, for single-sample rejection */
	uint8_t hist_count;
	int32_t effort_q16;      /* the one noise filter */
	int32_t base_q16;        /* sustained level */
	int32_t dynamic_q16;     /* fast excess */
	int32_t peak_q16;        /* decaying stroke peak */
} ap2_demand_ctx_t;

static ap2_demand_ctx_t ctx;

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
	int32_t v = ap2_clamp(permille, 0, AP2_PERMILLE);
	ctx.base_q16 = ap2_q16_from(v);
}

/* Median of three: removes a single corrupted ADC sample without adding any lag to a real
 * change, which a filter long enough to do the same job could not. */
static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
	if (a > b) {
		uint16_t t = a; a = b; b = t;
	}
	if (b > c) {
		uint16_t t = b; b = c; c = t;
	}
	if (a > b) {
		b = a;
	}
	return b;
}

static uint16_t stroke_period_ms(uint8_t cadence_rpm)
{
	uint32_t ms;

	if (cadence_rpm == 0U) {
		return (uint16_t)AP2_STROKE_MAX_MS;
	}
	/* One leg push per half revolution. */
	ms = 30000U / (uint32_t)cadence_rpm;
	if (ms < AP2_STROKE_MIN_MS) {
		ms = AP2_STROKE_MIN_MS;
	}
	if (ms > AP2_STROKE_MAX_MS) {
		ms = AP2_STROKE_MAX_MS;
	}
	return (uint16_t)ms;
}

void ap2_rider_demand_update(const ap2_demand_input_t *in, ap2_demand_output_t *out)
{
	uint16_t raw;
	uint16_t full_scale;
	uint16_t period;
	uint32_t base_fall_ms;
	int32_t effort;
	int32_t demand;
	int32_t base;
	int32_t excess;
	int32_t dynamic;

	if (out == 0) {
		return;
	}
	if (in == 0) {
		ap2_rider_demand_reset();
		out->effort_permille = 0;
		out->demand_permille = 0;
		out->base_permille = 0;
		out->dynamic_permille = 0;
		out->stroke_peak_permille = 0;
		out->stroke_period_ms = (uint16_t)AP2_STROKE_MAX_MS;
		out->load_ctrl = 0U;
		return;
	}

	/* ---- INPUT VALIDATION ------------------------------------------------------------
	 * A torque sensor that is faulted or under calibration produces no rider effort at all,
	 * and every state that could outlive the fault is dropped with it. There is no "last
	 * good value" here on purpose: a stuck reading is exactly what a torque fault looks
	 * like, and holding it would keep the motor pulling.
	 */
	if (!in->torque_valid) {
		ap2_rider_demand_reset();
		out->effort_permille = 0;
		out->demand_permille = 0;
		out->base_permille = 0;
		out->dynamic_permille = 0;
		out->stroke_peak_permille = 0;
		out->stroke_period_ms = stroke_period_ms(in->cadence_rpm);
		out->load_ctrl = in->load_ctrl;
		return;
	}

	raw = in->load_ctrl;
	ctx.hist[2] = ctx.hist[1];
	ctx.hist[1] = ctx.hist[0];
	ctx.hist[0] = raw;
	if (ctx.hist_count < 3U) {
		ctx.hist_count++;
	}

	/* ---- TORQUE ZERO / NORMALIZATION -------------------------------------------------
	 * torque_input.c already removed the sensor zero, applied the calibration gain and read
	 * the frozen control characteristic, so the input is CONTROL LOAD. All that is left is the
	 * deadband and the projection onto the ride-feel axis - the ONE normalization of this path.
	 */
	full_scale = (in->full_scale_ctrl != 0U) ?
		in->full_scale_ctrl : (uint16_t)AP2_FULL_SCALE_DEFAULT_CTRL;
	if (full_scale <= AP2_EFFORT_DEADBAND_CTRL) {
		full_scale = (uint16_t)(AP2_EFFORT_DEADBAND_CTRL + 1U);
	}

	{
		uint16_t robust = (ctx.hist_count >= 3U) ?
			median3(ctx.hist[0], ctx.hist[1], ctx.hist[2]) : raw;
		int32_t above = (int32_t)robust - (int32_t)AP2_EFFORT_DEADBAND_CTRL;
		if (above < 0) {
			above = 0;
		}
		effort = ap2_map(above, 0,
			(int32_t)full_scale - (int32_t)AP2_EFFORT_DEADBAND_CTRL,
			0, AP2_PERMILLE);
	}

	/* ---- RIDER DEMAND ---------------------------------------------------------------- */
	ctx.effort_q16 = ap2_lpf_step(ctx.effort_q16, effort, AP2_EFFORT_LPF_MS, in->elapsed_ticks);
	demand = ap2_clamp(ap2_q16_value(ctx.effort_q16), 0, AP2_PERMILLE);

	/* ---- PEDAL CYCLE / BASE LOAD ESTIMATION ------------------------------------------
	 * The base rises quickly toward a higher sustained effort and falls over at least one
	 * and a half pedal strokes, so the dead spot between leg pushes cannot empty it. When
	 * the cranks are not driving, the base is released at the stroke rate so it cannot
	 * survive into the next ride - but the STOP decision itself belongs to the PAS state
	 * machine, not here.
	 */
	period = stroke_period_ms(in->cadence_rpm);
	base_fall_ms = ((uint32_t)period * AP2_BASE_FALL_STROKE_NUM) / AP2_BASE_FALL_STROKE_DEN;
	if (in->base_hold_ms > base_fall_ms) {
		base_fall_ms = in->base_hold_ms;
	}
	if (!in->pedaling) {
		base_fall_ms = period;
	}

	base = ap2_q16_value(ctx.base_q16);
	if (demand >= base) {
		ctx.base_q16 = ap2_lpf_step(ctx.base_q16, demand, AP2_BASE_RISE_MS, in->elapsed_ticks);
	} else {
		ctx.base_q16 = ap2_lpf_step(ctx.base_q16, demand, base_fall_ms, in->elapsed_ticks);
	}
	base = ap2_clamp(ap2_q16_value(ctx.base_q16), 0, AP2_PERMILLE);

	/* ---- DYNAMIC COMPONENT ----------------------------------------------------------- */
	excess = demand - base;
	if (excess < 0) {
		excess = 0;
	}
	if (excess >= ap2_q16_value(ctx.dynamic_q16)) {
		ctx.dynamic_q16 = ap2_lpf_step(ctx.dynamic_q16, excess,
			AP2_DYNAMIC_RISE_MS, in->elapsed_ticks);
	} else {
		ctx.dynamic_q16 = ap2_lpf_step(ctx.dynamic_q16, excess,
			AP2_DYNAMIC_FALL_MS, in->elapsed_ticks);
	}
	dynamic = ap2_clamp(ap2_q16_value(ctx.dynamic_q16), 0, AP2_PERMILLE);

	/* ---- STROKE PEAK (description only) ---------------------------------------------- */
	if (demand >= ap2_q16_value(ctx.peak_q16)) {
		ctx.peak_q16 = ap2_q16_from(demand);
	} else {
		ctx.peak_q16 = ap2_lpf_step(ctx.peak_q16, demand,
			(uint32_t)period * AP2_STROKE_PEAK_DECAY_STROKES, in->elapsed_ticks);
	}

	out->effort_permille = effort;
	out->demand_permille = demand;
	out->base_permille = base;
	out->dynamic_permille = dynamic;
	out->stroke_peak_permille = ap2_clamp(ap2_q16_value(ctx.peak_q16), 0, AP2_PERMILLE);
	out->stroke_period_ms = period;
	out->load_ctrl = in->load_ctrl;
}
