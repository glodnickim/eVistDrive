#include "ap2_profiles.h"

#include "ap2_math.h"

/*
 * See inc/ap2_profiles.h for what a profile is and why AUTO is not a second pipeline.
 *
 * THE TABLE IS THE PRODUCT. Everything a rider means by "which mode am I in" lives in these
 * six rows. Read a row left to right and it says: how the pedal maps to the motor, how much
 * of the pull is steady and how much is reactive, how fast it answers and lets go, how gently
 * it starts, how much power it will spend, how long it bridges the dead spot, and how much it
 * lets the rider style and the terrain change all of that.
 */
static const ap2_profile_t profiles[AP2_PROFILE_COUNT] = {
	/* ECO - deliberately restrained. A light touch does very little, the pull is almost all
	 * sustained base, and it lets go calmly. The bike still feels like a bicycle. */
	[AP2_PROFILE_ECO] = {
		.assist_gain_pct = 80,
		.characteristic = AP2_CURVE_SOFT,
		.base_share_pct = 90,
		.dynamic_gain_pct = 60,
		.attack_ms = 500,
		.release_ms = 600,
		.start_ms = 450,
		.max_power_w = 250,
		.base_hold_ms = 300,
		.aggression_influence_pct = 20,
		.load_influence_pct = 40
	},
	/* TRAIL - the honest default. Linear characteristic, balanced base and dynamic, dynamics
	 * fast enough to feel connected and slow enough to stay predictable. */
	[AP2_PROFILE_TRAIL] = {
		.assist_gain_pct = 130,
		.characteristic = AP2_CURVE_LINEAR,
		.base_share_pct = 100,
		.dynamic_gain_pct = 100,
		.attack_ms = 350,
		.release_ms = 450,
		.start_ms = 350,
		.max_power_w = 450,
		.base_hold_ms = 350,
		.aggression_influence_pct = 40,
		.load_influence_pct = 60
	},
	/* SPORT - more of everything, still linear so the rider can predict it. */
	[AP2_PROFILE_SPORT] = {
		.assist_gain_pct = 190,
		.characteristic = AP2_CURVE_LINEAR,
		.base_share_pct = 105,
		.dynamic_gain_pct = 140,
		.attack_ms = 240,
		.release_ms = 380,
		.start_ms = 300,
		.max_power_w = 650,
		.base_hold_ms = 400,
		.aggression_influence_pct = 60,
		.load_influence_pct = 70
	},
	/*
	 * SPORT+ - noticeably more aggressive than SPORT, and NOT "instantly maximum Iq".
	 *
	 * What actually makes it different: an EAGER characteristic (the motor leads the rider in
	 * the middle of the range), a much larger dynamic term (it answers a harder push rather
	 * than just pulling harder overall), the shortest attack, the largest power envelope, and
	 * the longest base hold - so under a big load it SUSTAINS instead of surging and sagging.
	 * The attack is still an explicit, bounded ramp, which is what keeps it smooth.
	 */
	[AP2_PROFILE_SPORT_PLUS] = {
		.assist_gain_pct = 250,
		.characteristic = AP2_CURVE_EAGER,
		.base_share_pct = 115,
		.dynamic_gain_pct = 200,
		.attack_ms = 140,
		.release_ms = 320,
		.start_ms = 250,
		.max_power_w = 900,
		.base_hold_ms = 500,
		.aggression_influence_pct = 100,
		.load_influence_pct = 90
	},
	/* AUTO and AUTO SPORT+ carry their CALM endpoint here. ap2_profiles_resolve() moves them
	 * toward their strong endpoint; nothing ever reads these two rows as a fixed behaviour. */
	[AP2_PROFILE_AUTO] = {
		.assist_gain_pct = 80,
		.characteristic = AP2_CURVE_SOFT,
		.base_share_pct = 90,
		.dynamic_gain_pct = 60,
		.attack_ms = 500,
		.release_ms = 600,
		.start_ms = 450,
		.max_power_w = 250,
		.base_hold_ms = 300,
		.aggression_influence_pct = 20,
		.load_influence_pct = 40
	},
	[AP2_PROFILE_AUTO_SPORT_PLUS] = {
		.assist_gain_pct = 130,
		.characteristic = AP2_CURVE_LINEAR,
		.base_share_pct = 100,
		.dynamic_gain_pct = 100,
		.attack_ms = 350,
		.release_ms = 450,
		.start_ms = 350,
		.max_power_w = 450,
		.base_hold_ms = 350,
		.aggression_influence_pct = 40,
		.load_influence_pct = 60
	}
};

/*
 * The three characteristics as five-point piecewise-linear curves over 0..1000. The knots are
 * at 0, 250, 500, 750 and 1000 of input; the table holds the output at each knot.
 */
static const int16_t curve_knots[AP2_CURVE_COUNT][5] = {
	[AP2_CURVE_SOFT]   = { 0, 120, 300, 580, 1000 },
	[AP2_CURVE_LINEAR] = { 0, 250, 500, 750, 1000 },
	[AP2_CURVE_EAGER]  = { 0, 340, 620, 840, 1000 }
};

typedef struct {
	int32_t auto_q16;
} ap2_profiles_ctx_t;

static ap2_profiles_ctx_t ctx;

void ap2_profiles_reset(void)
{
	ctx.auto_q16 = 0;
}

bool ap2_profile_is_auto(ap2_profile_id_t id)
{
	return (id == AP2_PROFILE_AUTO) || (id == AP2_PROFILE_AUTO_SPORT_PLUS);
}

const ap2_profile_t *ap2_profile_base(ap2_profile_id_t id)
{
	if ((unsigned)id >= (unsigned)AP2_PROFILE_COUNT) {
		id = AP2_PROFILE_TRAIL;
	}
	return &profiles[id];
}

int32_t ap2_profile_shape(ap2_curve_t curve, int32_t permille)
{
	int32_t x = ap2_clamp(permille, 0, AP2_PERMILLE);
	int32_t seg;
	int32_t lo;
	int32_t hi;

	if ((unsigned)curve >= (unsigned)AP2_CURVE_COUNT) {
		curve = AP2_CURVE_LINEAR;
	}
	seg = x / 250;
	if (seg > 3) {
		seg = 3;
	}
	lo = curve_knots[curve][seg];
	hi = curve_knots[curve][seg + 1];
	return ap2_map(x, seg * 250, (seg + 1) * 250, lo, hi);
}

static uint16_t blend_u16(uint16_t calm, uint16_t strong, int32_t factor)
{
	return (uint16_t)ap2_clamp(
		(int32_t)calm + (((int32_t)strong - (int32_t)calm) * factor) / AP2_PERMILLE,
		0, 65535);
}

static uint8_t blend_u8(uint8_t calm, uint8_t strong, int32_t factor)
{
	return (uint8_t)ap2_clamp(
		(int32_t)calm + (((int32_t)strong - (int32_t)calm) * factor) / AP2_PERMILLE,
		0, 255);
}

/*
 * THE AUTO DECISION.
 *
 * Three independent reasons to offer a stronger character, taken as a maximum rather than a
 * mean for the same reason the aggression estimator does: each of them alone is a complete
 * argument, and averaging lets two quiet ones silence the one that is true.
 *
 *   the rider is ASKING            a high sustained demand
 *   the rider is riding SHARPLY    aggression
 *   the bike is WORKING            terrain load
 *
 * Plus one situation that none of the three states loudly enough on its own: a real load at a
 * low cadence, which is precisely where a calm profile feels like it gave up.
 *
 * The result is then moved with an asymmetric lag - quick to give, slow to take back - so the
 * character never flickers between modes mid-stroke.
 */
static int32_t auto_factor_update(const ap2_auto_input_t *in)
{
	int32_t want;

	if (in == 0) {
		return ap2_q16_value(ctx.auto_q16);
	}

	want = in->demand_permille;
	if (in->aggression_permille > want) {
		want = in->aggression_permille;
	}
	if (in->load_permille > want) {
		want = in->load_permille;
	}
	if (in->load_permille > 400 && in->cadence_rpm > 0 &&
		(int32_t)in->cadence_rpm < AP2_AUTO_LOW_CADENCE_RPM) {
		want += AP2_AUTO_LOAD_BONUS;
	}
	if (!in->pedaling) {
		want = 0;
	}
	want = ap2_clamp(want, 0, AP2_PERMILLE);

	if (want >= ap2_q16_value(ctx.auto_q16)) {
		ctx.auto_q16 = ap2_lpf_step(ctx.auto_q16, want, AP2_AUTO_RISE_MS, in->elapsed_ticks);
	} else {
		ctx.auto_q16 = ap2_lpf_step(ctx.auto_q16, want, AP2_AUTO_FALL_MS, in->elapsed_ticks);
	}
	return ap2_clamp(ap2_q16_value(ctx.auto_q16), 0, AP2_PERMILLE);
}

void ap2_profiles_resolve(ap2_profile_id_t id, const ap2_profile_override_t *ovr,
	const ap2_auto_input_t *in, ap2_profile_resolved_t *out)
{
	ap2_profile_t p;
	int32_t factor = 0;

	if (out == 0) {
		return;
	}
	if ((unsigned)id >= (unsigned)AP2_PROFILE_COUNT) {
		id = AP2_PROFILE_TRAIL;
	}

	if (ap2_profile_is_auto(id)) {
		const ap2_profile_t *calm = (id == AP2_PROFILE_AUTO) ?
			&profiles[AP2_PROFILE_ECO] : &profiles[AP2_PROFILE_TRAIL];
		const ap2_profile_t *strong = (id == AP2_PROFILE_AUTO) ?
			&profiles[AP2_PROFILE_SPORT] : &profiles[AP2_PROFILE_SPORT_PLUS];

		factor = auto_factor_update(in);

		p.assist_gain_pct = blend_u16(calm->assist_gain_pct, strong->assist_gain_pct, factor);
		p.base_share_pct = blend_u16(calm->base_share_pct, strong->base_share_pct, factor);
		p.dynamic_gain_pct = blend_u16(calm->dynamic_gain_pct, strong->dynamic_gain_pct, factor);
		p.attack_ms = blend_u16(calm->attack_ms, strong->attack_ms, factor);
		p.release_ms = blend_u16(calm->release_ms, strong->release_ms, factor);
		p.start_ms = blend_u16(calm->start_ms, strong->start_ms, factor);
		p.max_power_w = blend_u16(calm->max_power_w, strong->max_power_w, factor);
		p.base_hold_ms = blend_u16(calm->base_hold_ms, strong->base_hold_ms, factor);
		p.aggression_influence_pct = blend_u8(calm->aggression_influence_pct,
			strong->aggression_influence_pct, factor);
		p.load_influence_pct = blend_u8(calm->load_influence_pct,
			strong->load_influence_pct, factor);
		/*
		 * The characteristic is a SHAPE, not a number, so it switches at the midpoint rather
		 * than being interpolated into a curve neither endpoint defines. The gain and the
		 * dynamic terms already move continuously, so the handover is not a step in behaviour.
		 */
		p.characteristic = (factor >= 500) ? strong->characteristic : calm->characteristic;
	} else {
		p = profiles[id];
		/* A fixed profile must not leave a stale AUTO decision behind for the next time the
		 * rider selects an adaptive one. */
		ctx.auto_q16 = 0;
	}

	/* Per-level overrides. 0 always means "keep the profile value". See the shape rules in
	 * inc/ap2_profiles.h: a trim, a ceiling, and dynamics only where dynamics are the rider's
	 * to choose. */
	if (ovr != 0) {
		if (ovr->assist_trim_pct != 0U && ovr->assist_trim_pct != 100U) {
			p.assist_gain_pct = (uint16_t)ap2_clamp(
				ap2_scale_pct((int32_t)p.assist_gain_pct, (int32_t)ovr->assist_trim_pct),
				0, AP2_ASSIST_GAIN_MAX_PCT);
		}
		if (ovr->max_power_w != 0U &&
			(p.max_power_w == 0U || ovr->max_power_w < p.max_power_w)) {
			p.max_power_w = ovr->max_power_w;
		}
		if (!ap2_profile_is_auto(id)) {
			if (ovr->attack_ms != 0U) {
				p.attack_ms = ovr->attack_ms;
			}
			if (ovr->release_ms != 0U) {
				p.release_ms = ovr->release_ms;
			}
			if (ovr->start_ms != 0U) {
				p.start_ms = ovr->start_ms;
			}
		}
	}

	out->p = p;
	out->id = id;
	out->auto_factor = factor;
}
