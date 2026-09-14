#include "ap2_estimators.h"

#include "ap2_math.h"

/* See inc/ap2_estimators.h for why these two are separate and why their speeds differ. */

typedef struct {
	/* aggression */
	int32_t rate_ref_permille;      /* demand at the start of the current rate window */
	uint32_t rate_window_ticks;
	int32_t rate_per_100ms;         /* the last closed window, held until the next one */
	int32_t cadence_ref_rpm;
	uint32_t cadence_window_ticks;
	int32_t cadence_rate_per_s;
	int32_t aggr_q16;
	/* load */
	int32_t speed_ref_x100;
	uint32_t speed_window_ticks;
	bool speed_stagnant;
	int32_t load_q16;
} ap2_estimator_ctx_t;

static ap2_estimator_ctx_t ctx;

void ap2_estimators_reset(void)
{
	ctx.rate_ref_permille = 0;
	ctx.rate_window_ticks = 0U;
	ctx.rate_per_100ms = 0;
	ctx.cadence_ref_rpm = 0;
	ctx.cadence_window_ticks = 0U;
	ctx.cadence_rate_per_s = 0;
	ctx.aggr_q16 = 0;
	ctx.speed_ref_x100 = 0;
	ctx.speed_window_ticks = 0U;
	ctx.speed_stagnant = false;
	ctx.load_q16 = 0;
}

/*
 * A sampled-window difference rather than a per-tick derivative. A per-tick derivative of a
 * 4 kHz signal is dominated by quantisation of the sensor least significant bit; a window
 * difference measures the gesture the rider actually made. Returns true on the tick the
 * window closed, and then writes the difference to *delta.
 *
 * *closed_ticks receives the time the window ACTUALLY spanned, which is not the nominal length:
 * a late foreground can overshoot it by however much it coalesced. Dividing the difference by
 * the nominal length would then report a rate higher than the rider produced - the change is
 * real, the time it took is simply longer than assumed - so the caller normalises by this.
 */
static bool window_delta(uint32_t *window_ticks, int32_t *reference, int32_t value,
	uint32_t window_ms, uint32_t elapsed_ticks, int32_t *delta, uint32_t *closed_ticks)
{
	uint32_t limit = window_ms * AP2_TICKS_PER_MS;

	*window_ticks += (elapsed_ticks == 0U) ? 1U : elapsed_ticks;
	if (*window_ticks < limit) {
		return false;
	}
	*delta = value - *reference;
	*reference = value;
	*closed_ticks = *window_ticks;
	*window_ticks = 0U;
	return true;
}

void ap2_estimators_update(const ap2_estimator_input_t *in, ap2_estimator_output_t *out)
{
	int32_t rate_evidence;
	int32_t peak_evidence;
	int32_t cadence_evidence;
	int32_t aggression_raw;
	int32_t effort_evidence;
	int32_t cadence_load_evidence;
	int32_t load_raw;
	int32_t delta;
	uint32_t span;

	if (out == 0) {
		return;
	}
	if (in == 0) {
		ap2_estimators_reset();
		out->aggression_permille = 0;
		out->load_permille = 0;
		out->aggr_rate_evidence = 0;
		out->aggr_peak_evidence = 0;
		out->aggr_cadence_evidence = 0;
		out->load_effort_evidence = 0;
		out->load_cadence_evidence = 0;
		out->load_speed_stagnant = false;
		return;
	}

	/* ---- RIDER AGGRESSION ------------------------------------------------------------ */

	/* 1. How fast the rider raised the demand, in permille per 100 ms. */
	if (window_delta(&ctx.rate_window_ticks, &ctx.rate_ref_permille,
		in->demand_permille, AP2_AGGR_RATE_WINDOW_MS, in->elapsed_ticks, &delta, &span)) {
		/* Normalised by the time that actually passed, not by the nominal window. */
		int32_t span_ms = (int32_t)(span / AP2_TICKS_PER_MS);
		int32_t per_100ms = (span_ms > 0) ? (delta * 100) / span_ms : 0;
		ctx.rate_per_100ms = (per_100ms > 0) ? per_100ms : 0;
	}
	rate_evidence = ap2_map(ctx.rate_per_100ms,
		AP2_AGGR_RATE_LO, AP2_AGGR_RATE_HI, 0, AP2_PERMILLE);

	/* 2. How far the stroke peak stands above the sustained base, relative to the base. */
	{
		int32_t base = in->base_permille;
		int32_t ratio;
		if (base < 100) {
			base = 100;   /* a tiny base makes every ratio enormous; floor it at 10 % */
		}
		ratio = ((in->stroke_peak_permille - in->base_permille) * AP2_PERMILLE) / base;
		if (ratio < 0) {
			ratio = 0;
		}
		peak_evidence = ap2_map(ratio, AP2_AGGR_PEAK_LO, AP2_AGGR_PEAK_HI, 0, AP2_PERMILLE);
	}

	/* 3. How fast the cadence is being wound up, in rpm per second. */
	if (window_delta(&ctx.cadence_window_ticks, &ctx.cadence_ref_rpm,
		(int32_t)in->cadence_rpm, AP2_AGGR_CADENCE_WINDOW_MS, in->elapsed_ticks, &delta, &span)) {
		int32_t span_ms = (int32_t)(span / AP2_TICKS_PER_MS);
		int32_t per_s = (span_ms > 0) ? (delta * 1000) / span_ms : 0;
		ctx.cadence_rate_per_s = (per_s > 0) ? per_s : 0;
	}
	cadence_evidence = ap2_map(ctx.cadence_rate_per_s,
		AP2_AGGR_CADENCE_LO, AP2_AGGR_CADENCE_HI, 0, AP2_PERMILLE);

	/*
	 * Any ONE of the three is enough: a rider who snaps the torque up, one who stamps a hard
	 * peak onto a light stroke, and one who spins the cadence up are all riding sharply, and
	 * averaging them would let two quiet signals hide the one that is speaking.
	 */
	aggression_raw = rate_evidence;
	if (peak_evidence > aggression_raw) {
		aggression_raw = peak_evidence;
	}
	if (cadence_evidence > aggression_raw) {
		aggression_raw = cadence_evidence;
	}
	if (!in->pedaling) {
		aggression_raw = 0;
	}

	if (aggression_raw >= ap2_q16_value(ctx.aggr_q16)) {
		ctx.aggr_q16 = ap2_lpf_step(ctx.aggr_q16, aggression_raw,
			AP2_AGGR_RISE_MS, in->elapsed_ticks);
	} else {
		ctx.aggr_q16 = ap2_lpf_step(ctx.aggr_q16, aggression_raw,
			AP2_AGGR_FALL_MS, in->elapsed_ticks);
	}

	/* ---- LOAD / TERRAIN --------------------------------------------------------------
	 * The signature of a climb is not "high torque". It is high SUSTAINED torque that is
	 * NOT turning into cadence and is NOT turning into speed, held for long enough that a
	 * single hard stroke cannot produce it.
	 */
	effort_evidence = ap2_map(in->base_permille,
		AP2_LOAD_EFFORT_LO, AP2_LOAD_EFFORT_HI, 0, AP2_PERMILLE);
	cadence_load_evidence = ap2_map((int32_t)in->cadence_rpm,
		AP2_LOAD_CADENCE_HI_RPM, AP2_LOAD_CADENCE_LO_RPM, 0, AP2_PERMILLE);

	if (window_delta(&ctx.speed_window_ticks, &ctx.speed_ref_x100,
		(int32_t)in->speed_x100, AP2_LOAD_SPEED_WINDOW_MS, in->elapsed_ticks, &delta, &span)) {
		/* The flat-speed threshold is stated for the nominal window, so scale it to the window
		 * that actually elapsed rather than comparing against a mismatched span. */
		int32_t limit = (int32_t)(((int64_t)AP2_LOAD_SPEED_FLAT_X100 * (int64_t)span) /
			(int64_t)(AP2_LOAD_SPEED_WINDOW_MS * AP2_TICKS_PER_MS));
		if (limit < 1) {
			limit = 1;
		}
		ctx.speed_stagnant = delta < limit;
	}

	/*
	 * The same sustained effort means more load the slower the cranks turn. The cadence term
	 * SCALES the effort term rather than being added to it, so a high cadence on the flat can
	 * never be read as a climb no matter how hard the rider is working.
	 */
	load_raw = ap2_scale_permille(effort_evidence, 500 + (cadence_load_evidence / 2));
	if (ctx.speed_stagnant && effort_evidence > 400) {
		load_raw += AP2_LOAD_STAGNATION_BONUS;
	}
	if (!in->pedaling) {
		load_raw = 0;
	}
	load_raw = ap2_clamp(load_raw, 0, AP2_PERMILLE);

	if (load_raw >= ap2_q16_value(ctx.load_q16)) {
		ctx.load_q16 = ap2_lpf_step(ctx.load_q16, load_raw, AP2_LOAD_RISE_MS, in->elapsed_ticks);
	} else {
		ctx.load_q16 = ap2_lpf_step(ctx.load_q16, load_raw, AP2_LOAD_FALL_MS, in->elapsed_ticks);
	}

	out->aggression_permille = ap2_clamp(ap2_q16_value(ctx.aggr_q16), 0, AP2_PERMILLE);
	out->load_permille = ap2_clamp(ap2_q16_value(ctx.load_q16), 0, AP2_PERMILLE);
	out->aggr_rate_evidence = rate_evidence;
	out->aggr_peak_evidence = peak_evidence;
	out->aggr_cadence_evidence = cadence_evidence;
	out->load_effort_evidence = effort_evidence;
	out->load_cadence_evidence = cadence_load_evidence;
	out->load_speed_stagnant = ctx.speed_stagnant;
}
