#include "assist_pipeline.h"

#include "ap2_math.h"
#include "assist_modes.h"
#include "config.h"
#include "tuning_config.h"

/*
 * THE ONE PATH FROM PEDAL TO CURRENT. See inc/assist_pipeline.h for the block diagram and the
 * ownership rule. This file composes the blocks and owns nothing they own.
 */

/* The bike counts as rolling above 1.0 km/h. One owner of that fact for the whole pipeline:
 * it decides both the eased start-step count and which start-load threshold is in force. */
#define AP2_ROLLING_MIN_SPEED_X100 100

/*
 * The firmware-owned safety release. A brake, a reverse crank step, an overtemperature cut or
 * a torque-sensor fault removes the REQUEST in the same tick; this is how fast the CURRENT
 * already in the motor is retired afterwards. It is a property of the machine, not of the
 * profile, so no rider setting can lengthen it.
 */
#define AP2_SAFETY_RELEASE_MS 200U
/* Hard bound on any release, so a configuration error cannot leave current flowing. */
#define AP2_RELEASE_MAX_MS 3000U

/* The FOC commutation angle changes formula below this electrical speed and the two forms do
 * not agree, so any current still flowing steps with it - the click heard at a standstill.
 * A release finishes before that point and the motor coasts out on its own inertia. A START
 * also happens in this range and must keep its current, hence the zero-demand condition. */
#define AP2_COAST_RELEASE_ERPS RIDE_COAST_RELEASE_ERPS

#define AP2_FOC_TICKS_PER_MS 16U

/* Rider power per (0.01 kg x rpm), in mW, at the 165 mm reference crank:
 * P = m g L 2 pi n / 60. Telemetry only - no control decision reads rider power. */
#define AP2_HUMAN_POWER_NUM 1694U
#define AP2_HUMAN_POWER_DEN 1000U
#define AP2_HUMAN_POWER_REF_CRANK_MM 165U

typedef struct {
	/* The base-decay floor resolved on the PREVIOUS tick. The demand model needs it before
	 * the profile is resolved, and the profile resolution needs this tick's demand: rather
	 * than inventing an algebraic loop, the 0.25 ms old value of a 300..500 ms parameter is
	 * used, which is exact to well within its own resolution. */
	uint16_t base_hold_ms;
	/* START segment: the remaining ticks for which the start attack time is in force. */
	uint32_t start_ticks_left;
	bool start_active;
	/*
	 * THE RELEASE TIME IN FORCE, LATCHED AT THE EDGE.
	 *
	 * The 16 kHz owner derives a release rate once, at the command edge, from its live
	 * accumulator - which is what makes "the current reaches zero in release_ms" true whatever
	 * it was releasing from. A command that keeps CHANGING re-triggers that edge every tick,
	 * and re-deriving "reach zero in release_ms" from an ever-smaller current turns a bounded
	 * ramp into an exponential tail that takes several times as long to actually reach zero.
	 *
	 * Rider aggression shortens the release, and aggression decays during the release itself,
	 * so without this latch the command changed on almost every tick. Freezing the value when
	 * the release starts fixes that and is also the right behaviour on its own: how fast the
	 * motor lets go is decided when the rider stops, not re-negotiated while it is happening.
	 */
	uint16_t release_latched_ms;
	bool release_latched;
	assist_pipeline_telemetry_t tlm;
} ap2_pipeline_ctx_t;

static ap2_pipeline_ctx_t ctx;

void assist_pipeline_reset(void)
{
	const ap2_profile_t *fallback = ap2_profile_base(AP2_PROFILE_TRAIL);
	int i;
	unsigned char *p = (unsigned char *)&ctx.tlm;
	/* The engagement counter is an ANCHOR, not state: a reader detects a new engagement by
	 * seeing it change, so restarting it at a Walk or calibration detour would make two
	 * different engagements look like the same one. It is the only field that survives. */
	uint16_t seq = ctx.tlm.engage_seq;

	for (i = 0; i < (int)sizeof(ctx.tlm); i++) {
		p[i] = 0U;
	}
	ctx.tlm.engage_seq = seq;
	ctx.base_hold_ms = fallback->base_hold_ms;
	ctx.start_ticks_left = 0U;
	ctx.start_active = false;
	ctx.release_latched_ms = 0U;
	ctx.release_latched = false;

	ap2_pas_state_reset();
	ap2_rider_demand_reset();
	ap2_estimators_reset();
	ap2_profiles_reset();
	ap2_limits_reset();
}

void assist_pipeline_init(void)
{
	assist_pipeline_reset();
}

const assist_pipeline_telemetry_t *assist_pipeline_telemetry(void)
{
	return &ctx.tlm;
}

ap2_pas_state_t assist_pipeline_pas_state(void)
{
	return ap2_pas_state_get();
}

uint8_t assist_pipeline_reason_bits(void)
{
	const assist_pipeline_telemetry_t *t = &ctx.tlm;
	uint8_t why = 0U;

	if (!t->assist_permitted) {
		why |= AP2_WHY_NOT_PERMITTED;
	}
	if (t->block_positive) {
		why |= AP2_WHY_BLOCKED;
	}
	if (t->assist_permitted && t->assist_response_permille == 0) {
		why |= AP2_WHY_NO_DEMAND;
	}
	if (t->power_limited || t->battery_limited || t->phase_limited ||
		t->voltage_limited || t->thermal_limited || t->speed_limited) {
		why |= AP2_WHY_LIMITED;
	}
	if (t->limiter_zeroed) {
		why |= AP2_WHY_ZEROED_BY_LIMIT;
	}
	if (t->start_active) {
		why |= AP2_WHY_START;
	}
	if (t->release_active) {
		why |= AP2_WHY_RELEASE;
	}
	if (ap2_profile_is_auto((ap2_profile_id_t)t->profile_id)) {
		why |= AP2_WHY_AUTO;
	}
	return why;
}

uint8_t assist_pipeline_state_byte(void)
{
	return (uint8_t)((ctx.tlm.pas_state & 0x0FU) | ((ctx.tlm.profile_id & 0x0FU) << 4));
}

bool assist_pipeline_battery_limited(void)
{
	return ap2_limits_battery_active();
}

/* Motor input power at the measured duty - the inverse of the power ceiling conversion in
 * ap2_limits.c, so the number reported and the number limited are the same physics. */
static uint16_t motor_power_w(int32_t iq, uint32_t battery_voltage_mv, int32_t u_abs,
	int32_t cal_i)
{
	int64_t p;

	if (iq <= 0 || u_abs <= 0 || cal_i <= 0 || battery_voltage_mv == 0U) {
		return 0U;
	}
	p = ((int64_t)battery_voltage_mv * (int64_t)iq * (int64_t)cal_i * (int64_t)u_abs) /
		((int64_t)AP2_U_ABS_FULL_SCALE * 1000000);
	if (p < 0) {
		p = 0;
	}
	if (p > 65535) {
		p = 65535;
	}
	return (uint16_t)p;
}

static uint16_t rider_power_w(uint16_t load_centikg, uint8_t cadence_rpm)
{
	uint32_t crank_mm = tuning_config_crank_length_mm();
	uint64_t mw;

	if (load_centikg == 0U || cadence_rpm == 0U) {
		return 0U;
	}
	if (crank_mm == 0U) {
		crank_mm = AP2_HUMAN_POWER_REF_CRANK_MM;
	}
	mw = ((uint64_t)load_centikg * (uint64_t)cadence_rpm * AP2_HUMAN_POWER_NUM) /
		AP2_HUMAN_POWER_DEN;
	mw = (mw * crank_mm) / AP2_HUMAN_POWER_REF_CRANK_MM;
	mw /= 1000U;
	return (mw > 65535U) ? 65535U : (uint16_t)mw;
}

/*
 * The 4 kHz producer half of the single final Iq trajectory. It decides the MODE and the STEP
 * only; the 16 kHz owner holds the accumulator and is the sole writer of the reference.
 *
 * The step is the Q8 per-4-kHz increment that walks a full-scale move across the configured
 * time, so "attack_ms" means the same thing here as it does to the rider.
 */
static fis_mode_t trajectory(int32_t iq_target, int32_t iq_full_scale,
	uint16_t attack_ms, uint16_t release_ms, bool permitted, bool block_positive,
	bool service_cut, bool coast_zero,
	uint16_t *out_step_mag, uint32_t *out_release_ticks_16k,
	fis_zero_policy_t *out_zero_policy)
{
	int32_t live_q10;
	int64_t target_q10;
	bool rising;
	uint32_t ms;
	int64_t scale_q;
	int64_t step_4k;
	int32_t ticks;

	*out_step_mag = 0U;
	*out_release_ticks_16k = 0U;
	*out_zero_policy = FIS_ZERO_POLICY_NONE;

	if (iq_full_scale < 1) {
		iq_full_scale = PH_CURRENT_MAX;
	}

	/*
	 * A RELEASE is the end of a demand: the rider stopped pedalling, or the drive is no
	 * longer permitted. It is the only branch that may grant Quiet Zero, and the pedal-load
	 * calibration is excluded from it because that is a workshop procedure, not a release.
	 */
	if (iq_target == 0 && (block_positive || !permitted)) {
		ms = block_positive ? AP2_SAFETY_RELEASE_MS : release_ms;
		if (ms == 0U) {
			ms = AP2_SAFETY_RELEASE_MS;
		}
		if (ms > AP2_RELEASE_MAX_MS) {
			ms = AP2_RELEASE_MAX_MS;
		}
		*out_release_ticks_16k = ms * AP2_FOC_TICKS_PER_MS;
		if (*out_release_ticks_16k == 0U) {
			*out_release_ticks_16k = 1U;
		}
		*out_zero_policy = service_cut ? FIS_ZERO_POLICY_NONE : FIS_ZERO_POLICY_QUIET;
		return block_positive ? FIS_MODE_SAFETY : FIS_MODE_RELEASE;
	}

	/* Finish a zero immediately rather than trickling current through the angle handover. */
	if (iq_target == 0 && coast_zero) {
		return FIS_MODE_FORCE_ZERO;
	}

	live_q10 = fast_iq_slew_current_accumulator_q10();
	target_q10 = (int64_t)iq_target << 10;
	if (target_q10 == live_q10) {
		return FIS_MODE_HOLD;
	}
	rising = target_q10 > live_q10;
	ms = rising ? attack_ms : release_ms;
	if (ms == 0U) {
		ms = 1U;
	}
	ticks = (int32_t)(ms * AP2_TICKS_PER_MS);
	if (ticks < 1) {
		ticks = 1;
	}
	scale_q = (int64_t)iq_full_scale << IQ_RAMP_Q_SHIFT;
	step_4k = (scale_q + ticks - 1) / ticks;
	if (step_4k < 1) {
		step_4k = 1;
	}
	if (step_4k > 32767) {
		step_4k = 32767;
	}
	*out_step_mag = (uint16_t)step_4k;
	return rising ? FIS_MODE_RISE : FIS_MODE_FALL;
}

void assist_pipeline_update(const assist_pipeline_input_t *in, assist_pipeline_command_t *cmd)
{
	const assist_level_config_t *level;
	ap2_pas_input_t pas_in;
	ap2_pas_output_t pas;
	ap2_demand_input_t demand_in;
	ap2_demand_output_t demand;
	ap2_estimator_input_t est_in;
	ap2_estimator_output_t est;
	ap2_auto_input_t auto_in;
	ap2_profile_override_t ovr;
	ap2_profile_resolved_t prof;
	ap2_limits_input_t lim_in;
	ap2_limits_output_t lim;
	ap2_profile_id_t profile_id;
	bool assist_off;
	bool bike_rolling;
	int32_t iq_full_scale;
	int32_t base_shaped;
	int32_t assist_base;
	int32_t assist_dynamic;
	int32_t response;
	int32_t iq_request;
	int32_t throttle_iq;
	uint16_t attack_ms;
	uint16_t release_ms;
	uint32_t used_ticks;

	if (cmd == 0) {
		return;
	}
	if (in == 0) {
		assist_pipeline_reset();
		cmd->final_iq_request = 0;
		cmd->slew_mode = FIS_MODE_FORCE_ZERO;
		cmd->step_mag_8 = 0U;
		cmd->release_ticks_16k = 0U;
		cmd->zero_policy = FIS_ZERO_POLICY_NONE;
		return;
	}

	used_ticks = (in->elapsed_ticks == 0U) ? 1U : in->elapsed_ticks;

	/* ---- LEVEL AND PROFILE SELECTION -------------------------------------------------- */
	level = assist_modes_get_default_level(in->assist_level_index);
	assist_off = (in->assist_level_index == 0U) ||
		(level->mode_type == ASSIST_MODE_RESERVED_0);
	profile_id = assist_modes_profile_for_level(level);
	assist_modes_profile_override(level, &ovr);

	bike_rolling = in->speed_x100 >= (uint32_t)AP2_ROLLING_MIN_SPEED_X100;

	/* ---- PAS / DIRECTION LIFECYCLE ---------------------------------------------------- */
	pas_in.forward_valid = in->forward_valid;
	pas_in.direction_inhibit = in->direction_inhibit;
	pas_in.inhibit_is_reverse = in->inhibit_is_reverse;
	pas_in.real_stop = in->real_stop;
	pas_in.wheel_valid = in->wheel_valid;
	pas_in.pas_sensor_valid = in->pas_sensor_valid;
	pas_in.torque_sensor_valid = in->torque_sensor_valid;
	pas_in.safety_cut = in->safety_cut;
	pas_in.assist_off = assist_off;
	pas_in.forward_steps = in->forward_steps;
	/*
	 * The anti-jiggle step count is eased by one while the bike is already rolling: turning
	 * the cranks on a stationary bike proves nothing, but on a moving one the rider has
	 * already demonstrated intent.
	 */
	pas_in.required_steps = bike_rolling ?
		((in->required_steps > 0U) ? (uint8_t)(in->required_steps - 1U) : 0U) :
		in->required_steps;
	pas_in.load_centikg = in->torque_load_centikg;
	pas_in.engage_load_centikg = bike_rolling ?
		level->riding_start_load_centikg : level->minimum_pedal_load_centikg;
	pas_in.elapsed_ticks = used_ticks;
	ap2_pas_state_update(&pas_in, &pas);

	/* ---- RIDER DEMAND + PEDAL CYCLE --------------------------------------------------- */
	demand_in.load_centikg = in->torque_load_centikg;
	demand_in.torque_valid = in->torque_sensor_valid;
	demand_in.pedaling = (pas.state == AP2_PAS_FORWARD) ||
		(pas.state == AP2_PAS_STARTING_FORWARD);
	demand_in.cadence_rpm = in->cadence_rpm;
	demand_in.full_scale_centikg = tuning_config_assist_torque_full_scale_centikg();
	demand_in.base_hold_ms = ctx.base_hold_ms;
	demand_in.elapsed_ticks = used_ticks;
	ap2_rider_demand_update(&demand_in, &demand);

	/*
	 * At the engagement edge the sustained base is seeded to what the rider is pressing RIGHT
	 * NOW, so the first stroke of a ride (or of a resumed ride) is answered at its real
	 * magnitude instead of being walked up from zero by the base rise time. This is a seed,
	 * not a floor: it cannot produce assist the rider is not asking for.
	 */
	if (pas.engaged_edge) {
		ap2_rider_demand_seed_base(demand.demand_permille);
		demand.base_permille = demand.demand_permille;
		demand.dynamic_permille = 0;
	}

	/* ---- ESTIMATORS ------------------------------------------------------------------- */
	est_in.demand_permille = demand.demand_permille;
	est_in.base_permille = demand.base_permille;
	est_in.stroke_peak_permille = demand.stroke_peak_permille;
	est_in.cadence_rpm = in->cadence_rpm;
	est_in.speed_x100 = in->speed_x100;
	est_in.pedaling = demand_in.pedaling;
	est_in.elapsed_ticks = used_ticks;
	ap2_estimators_update(&est_in, &est);

	/* ---- PROFILE / AUTO --------------------------------------------------------------- */
	auto_in.demand_permille = demand.demand_permille;
	auto_in.aggression_permille = est.aggression_permille;
	auto_in.load_permille = est.load_permille;
	auto_in.cadence_rpm = in->cadence_rpm;
	auto_in.speed_x100 = in->speed_x100;
	auto_in.pedaling = demand_in.pedaling;
	auto_in.elapsed_ticks = used_ticks;
	ap2_profiles_resolve(profile_id, &ovr, &auto_in, &prof);
	ctx.base_hold_ms = prof.p.base_hold_ms;

	/* ---- ASSIST CHARACTERISTIC + BASE / DYNAMIC COMPONENT -----------------------------
	 * The characteristic shapes the SUSTAINED term only. The dynamic term is already an
	 * excess above that sustained level, so putting it through the same curve a second time
	 * would apply the profile shape twice to the same pedal force.
	 */
	base_shaped = ap2_profile_shape((ap2_curve_t)prof.p.characteristic, demand.base_permille);

	assist_base = ap2_scale_pct(base_shaped, (int32_t)prof.p.assist_gain_pct);
	assist_base = ap2_scale_pct(assist_base, (int32_t)prof.p.base_share_pct);
	/* Terrain load lifts the SUSTAINED pull - that is what a climb needs, and it is the one
	 * thing a slow estimator is entitled to change. */
	assist_base = ap2_scale_permille(assist_base,
		AP2_PERMILLE + ap2_scale_pct(est.load_permille, (int32_t)prof.p.load_influence_pct));

	assist_dynamic = ap2_scale_pct(demand.dynamic_permille, (int32_t)prof.p.assist_gain_pct);
	assist_dynamic = ap2_scale_pct(assist_dynamic, (int32_t)prof.p.dynamic_gain_pct);
	/* Rider aggression lifts the REACTIVE pull and nothing else. It never multiplies the
	 * whole request: a sharp rider gets a sharper answer, not a permanently stronger motor. */
	assist_dynamic = ap2_scale_permille(assist_dynamic,
		AP2_PERMILLE + ap2_scale_pct(est.aggression_permille,
			(int32_t)prof.p.aggression_influence_pct));

	response = ap2_clamp(assist_base + assist_dynamic, 0, AP2_PERMILLE);

	/* ---- ASSIST -> Iq ----------------------------------------------------------------
	 * ASSIST is a torque-domain characteristic, so it converts to current directly and is
	 * finite at a standstill, where a power-domain request divides by a duty that has gone
	 * to zero. POWER is a separate ceiling and is applied in the limiter chain.
	 */
	iq_full_scale = in->level_iq_limit;
	if (iq_full_scale < 1) {
		iq_full_scale = in->phase_current_max;
	}
	if (iq_full_scale < 1) {
		iq_full_scale = PH_CURRENT_MAX;
	}

	if (pas.assist_permitted && !pas.block_positive) {
		iq_request = ap2_scale_permille(iq_full_scale, response);
	} else {
		/* Not permitted: the request is zero in the SAME tick, with no hold, no floor and no
		 * estimator allowed to carry a positive value past this point. */
		iq_request = 0;
	}

	/* ---- START SEGMENT ---------------------------------------------------------------
	 * There is no delay before assist: the request exists from the tick the lifecycle
	 * engages. What the start segment changes is the RATE for the first start_ms, so the
	 * first appearance of torque is gentle and then hands over to the normal attack.
	 * A resumed ride gets a third of it - the rider is already moving and has already had a
	 * gentle first appearance on this ride.
	 */
	if (pas.engaged_edge) {
		uint32_t ms = prof.p.start_ms;
		if (pas.resumed) {
			ms /= 3U;
		}
		ctx.start_ticks_left = ms * AP2_TICKS_PER_MS;
		ctx.start_active = ctx.start_ticks_left > 0U;
	}
	if (!pas.assist_permitted) {
		ctx.start_ticks_left = 0U;
		ctx.start_active = false;
	} else if (ctx.start_ticks_left > 0U) {
		if (used_ticks >= ctx.start_ticks_left) {
			ctx.start_ticks_left = 0U;
			ctx.start_active = false;
		} else {
			ctx.start_ticks_left -= used_ticks;
		}
	}

	attack_ms = ctx.start_active ? prof.p.start_ms : prof.p.attack_ms;
	release_ms = prof.p.release_ms;
	/*
	 * Rider aggression shortens the dynamics. Bounded to half, so an aggressive style can
	 * never turn an explicit ramp into a step.
	 */
	if (prof.p.aggression_influence_pct > 0U && est.aggression_permille > 0) {
		int32_t shorten = ap2_scale_pct(est.aggression_permille,
			(int32_t)prof.p.aggression_influence_pct);
		if (shorten > 500) {
			shorten = 500;
		}
		attack_ms = (uint16_t)ap2_clamp(
			ap2_scale_permille((int32_t)attack_ms, AP2_PERMILLE - shorten), 20, 5000);
		release_ms = (uint16_t)ap2_clamp(
			ap2_scale_permille((int32_t)release_ms, AP2_PERMILLE - shorten), 20, 5000);
	}

	/*
	 * Latch the release time on the edge into "not permitted", and release it again the moment
	 * the drive is permitted. See release_latched_ms for why a release time that keeps moving
	 * turns the release into an exponential tail.
	 */
	if (pas.assist_permitted && !pas.block_positive) {
		ctx.release_latched = false;
	} else if (!ctx.release_latched) {
		ctx.release_latched_ms = release_ms;
		ctx.release_latched = true;
	}
	if (ctx.release_latched) {
		release_ms = ctx.release_latched_ms;
	}

	/* ---- THROTTLE --------------------------------------------------------------------
	 * A separate rider input, not an assist override. It joins the SAME limiter chain and is
	 * judged as non-pedal there, so it can never inherit the pedalling speed allowance; and
	 * it is combined by max() BEFORE the limits, so there is still exactly one final request.
	 */
	throttle_iq = (!pas.block_positive && in->throttle_iq > 0) ? in->throttle_iq : 0;

	/* ---- LIMITS ----------------------------------------------------------------------- */
	lim_in.iq_request = iq_request;
	lim_in.source = AP2_LIMIT_SOURCE_PEDAL;
	lim_in.max_power_w = prof.p.max_power_w;
	lim_in.battery_voltage_mv = in->battery_voltage_mv;
	lim_in.u_abs = in->u_abs;
	lim_in.cal_i = in->cal_i;
	lim_in.battery_current_ma = in->battery_current_ma;
	lim_in.battery_current_max = in->battery_current_max;
	lim_in.level_iq_limit = in->level_iq_limit;
	lim_in.phase_current_max = in->phase_current_max;
	lim_in.voltage_raw = in->voltage_raw;
	lim_in.voltage_min_raw = in->voltage_min_raw;
	lim_in.controller_temperature_c = in->controller_temperature_c;
	lim_in.speed_x100 = in->speed_x100;
	lim_in.speed_limit_x100 = in->speed_limit_x100;
	lim_in.legal_enabled = in->legal_enabled;
	lim_in.offroad = in->offroad;
	ap2_limits_apply(&lim_in, &lim);

	if (throttle_iq > 0) {
		ap2_limits_output_t throttle_lim;
		ap2_limits_input_t throttle_in = lim_in;
		throttle_in.iq_request = throttle_iq;
		throttle_in.source = AP2_LIMIT_SOURCE_NON_PEDAL;
		throttle_in.max_power_w = 0U;   /* the profile power ceiling is an ASSIST setting */
		ap2_limits_apply(&throttle_in, &throttle_lim);
		if (throttle_lim.final_iq > lim.final_iq) {
			lim.final_iq = throttle_lim.final_iq;
		}
	}

	/*
	 * ABSOLUTE ZERO. Re-checked here, after everything, and independent of every decision
	 * above: whatever the blocks produced, a blocked lifecycle means exactly zero. This is
	 * deliberately redundant with the request being zeroed at its source - the point of a
	 * final gate is that it holds even if a future change to the logic above gets a case
	 * wrong, and it does not depend on that logic being correct.
	 */
	if (pas.block_positive) {
		lim.final_iq = 0;
	}
	if (lim.final_iq < 0) {
		lim.final_iq = 0;
	}

	/* ---- TRAJECTORY ------------------------------------------------------------------- */
	cmd->final_iq_request = lim.final_iq;
	cmd->slew_mode = trajectory(lim.final_iq, iq_full_scale, attack_ms, release_ms,
		pas.assist_permitted, pas.block_positive, in->service_cut,
		in->motor_erps < AP2_COAST_RELEASE_ERPS,
		&cmd->step_mag_8, &cmd->release_ticks_16k, &cmd->zero_policy);

	/* ---- TELEMETRY -------------------------------------------------------------------- */
	ctx.tlm.torque_load_centikg = in->torque_load_centikg;
	ctx.tlm.torque_normalized_permille = demand.effort_permille;
	ctx.tlm.cadence_rpm = in->cadence_rpm;
	ctx.tlm.pas_state = (uint8_t)pas.state;
	ctx.tlm.rider_demand_permille = demand.demand_permille;
	ctx.tlm.assist_base_permille = assist_base;
	ctx.tlm.assist_dynamic_permille = assist_dynamic;
	ctx.tlm.stroke_period_ms = demand.stroke_period_ms;
	ctx.tlm.rider_aggression_permille = est.aggression_permille;
	ctx.tlm.load_state_permille = est.load_permille;
	ctx.tlm.profile_id = (uint8_t)prof.id;
	ctx.tlm.auto_factor_permille = prof.auto_factor;
	ctx.tlm.assist_gain_pct = prof.p.assist_gain_pct;
	ctx.tlm.attack_ms = attack_ms;
	ctx.tlm.release_ms = release_ms;
	ctx.tlm.max_power_w = prof.p.max_power_w;
	ctx.tlm.assist_response_permille = response;
	ctx.tlm.iq_request_before_limits = iq_request;
	ctx.tlm.final_iq_request = lim.final_iq;
	ctx.tlm.power_limited = lim.power_limited;
	ctx.tlm.battery_limited = lim.battery_limited;
	ctx.tlm.phase_limited = lim.phase_limited;
	ctx.tlm.voltage_limited = lim.voltage_limited;
	ctx.tlm.thermal_limited = lim.thermal_limited;
	ctx.tlm.speed_limited = lim.speed_limited;
	ctx.tlm.assist_permitted = pas.assist_permitted;
	ctx.tlm.start_active = ctx.start_active;
	ctx.tlm.release_active = (cmd->slew_mode == FIS_MODE_RELEASE) ||
		(cmd->slew_mode == FIS_MODE_SAFETY);
	ctx.tlm.block_positive = pas.block_positive;
	ctx.tlm.limiter_zeroed = (iq_request > 0) && (lim.final_iq == 0);
	if (pas.engaged_edge) {
		ctx.tlm.engage_seq++;
	}
	ctx.tlm.required_steps = pas_in.required_steps;
	ctx.tlm.engage_threshold_centikg = pas_in.engage_load_centikg;
	ctx.tlm.bike_rolling = bike_rolling;
	ctx.tlm.rider_power_w = rider_power_w(in->torque_load_centikg, in->cadence_rpm);
	ctx.tlm.motor_power_w = motor_power_w(lim.final_iq, in->battery_voltage_mv,
		in->u_abs, in->cal_i);
	ctx.tlm.applied_support_ratio_pct = (ctx.tlm.rider_power_w > 0U) ?
		(uint16_t)ap2_clamp(((int32_t)ctx.tlm.motor_power_w * 100) /
			(int32_t)ctx.tlm.rider_power_w, 0, 65535) : 0U;
}
