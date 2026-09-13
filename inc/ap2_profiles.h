#ifndef AP2_PROFILES_H_
#define AP2_PROFILES_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * ASSIST PIPELINE V2 - ASSIST MODES, ASSIST CHARACTERISTIC and AUTO.
 *
 * A MODE IS NOT A PERCENTAGE. Every profile below defines a whole BEHAVIOUR: how rider effort
 * maps to motor demand (the characteristic), how much of the request comes from the sustained
 * base and how much from the fast dynamic term, how quickly it attacks and releases, how much
 * power it is willing to spend, and how strongly rider aggression and terrain load are allowed
 * to shape it. Two profiles with the same gain still feel completely different.
 *
 * AUTO IS NOT A SEVENTH PIPELINE. AUTO and AUTO SPORT+ run the exact same blocks as every
 * other profile. All they do is move the parameters of that one pipeline continuously between
 * a CALM endpoint and a STRONG endpoint, driven by how the bike is actually being ridden:
 *
 *      AUTO             calm = ECO     strong = SPORT
 *      AUTO SPORT+      calm = TRAIL   strong = SPORT+
 *
 * The endpoints are ordinary profiles, so AUTO can never reach a behaviour a fixed profile
 * could not. The movement between them is continuous and asymmetric (quick to give, slow to
 * take away) - it never hops between discrete modes, which is what makes a switching assist
 * feel like it is fighting the rider.
 *
 * UNITS. Percentages are of the quantity named in each field. Times are milliseconds for a
 * FULL-SCALE move, so "attack_ms = 240" reads as "from no assist to full assist in 240 ms".
 */

typedef enum {
	AP2_PROFILE_ECO = 0,
	AP2_PROFILE_TRAIL = 1,
	AP2_PROFILE_SPORT = 2,
	AP2_PROFILE_SPORT_PLUS = 3,
	AP2_PROFILE_AUTO = 4,
	AP2_PROFILE_AUTO_SPORT_PLUS = 5,
	AP2_PROFILE_COUNT = 6
} ap2_profile_id_t;

/*
 * THE ASSIST CHARACTERISTIC: the shape of rider effort -> assist response, before any gain.
 * Three shapes, each a five-point piecewise-linear curve over 0..1000 so it costs no pow()
 * and can be read off a table by anyone tuning the bike.
 *
 *   SOFT    output stays below input at low effort - a light touch does very little, which is
 *           what makes ECO feel like a bicycle rather than a moped.
 *   LINEAR  output follows input - the honest, predictable default.
 *   EAGER   output leads input in the lower and middle range, then saturates - the motor is
 *           already there when the rider leans on the pedals.
 */
typedef enum {
	AP2_CURVE_SOFT = 0,
	AP2_CURVE_LINEAR = 1,
	AP2_CURVE_EAGER = 2,
	AP2_CURVE_COUNT = 3
} ap2_curve_t;

typedef struct {
	/* effort -> response gain, in percent. 100 means the characteristic output is used as is;
	 * above 100 the response reaches full assist before the rider reaches full effort. */
	uint16_t assist_gain_pct;
	uint8_t characteristic;          /* ap2_curve_t */
	/* how much of the SUSTAINED base becomes assist, percent */
	uint16_t base_share_pct;
	/* how much of the FAST dynamic excess becomes assist, percent */
	uint16_t dynamic_gain_pct;
	/* explicit dynamics, milliseconds for a full-scale move */
	uint16_t attack_ms;
	uint16_t release_ms;
	/* the start segment: the attack time in force for the first start_ms of a ride */
	uint16_t start_ms;
	/* POWER: the ceiling this profile is willing to spend, in watts. 0 = no profile ceiling
	 * (the level, battery and phase-current limits still apply). */
	uint16_t max_power_w;
	/* floor for the base decay between pedal strokes, milliseconds */
	uint16_t base_hold_ms;
	/* how strongly rider aggression may shorten attack and lift the dynamic term, percent */
	uint8_t aggression_influence_pct;
	/* how strongly terrain load may lift the sustained base, percent */
	uint8_t load_influence_pct;
} ap2_profile_t;

/* Resolved parameters for THIS tick, after AUTO interpolation and per-level overrides. */
typedef struct {
	ap2_profile_t p;
	ap2_profile_id_t id;
	/* 0..1000. For AUTO profiles, where between calm and strong the pipeline currently sits.
	 * For fixed profiles it is reported as 0 and nothing reads it. */
	int32_t auto_factor;
} ap2_profile_resolved_t;

/*
 * Per-level overrides taken from the stored bank. 0 always means "use the profile value", so
 * an old stored bank, a zeroed field and a fresh controller all behave identically.
 *
 * THE OVERRIDES ARE SHAPED SO THEY CANNOT DEFEAT AUTO.
 *
 *   assist_trim_pct   a TRIM on the profile gain, not a replacement: 100 = the profile as
 *                     designed, 120 = twenty percent more assist for the same effort. A trim
 *                     composes with an AUTO blend; a replacement would freeze the gain AUTO
 *                     exists to move, which is how an "adaptive" mode stops being adaptive.
 *
 *   max_power_w       a CEILING, applied as a minimum against the profile value. It can only
 *                     tighten, never loosen - that is what makes it a ceiling and not a second
 *                     way to ask for more power. A rider who wants a larger envelope picks a
 *                     profile that has one.
 *
 *   attack/release/   DYNAMICS, honoured for FIXED profiles only. Choosing AUTO is choosing to
 *   start_ms          let the pipeline pick the dynamics; pinning them by hand would leave a
 *                     mode that adapts its strength but not its character, which is worse than
 *                     either choice made deliberately.
 */
typedef struct {
	uint16_t assist_trim_pct;
	uint16_t max_power_w;
	uint16_t attack_ms;
	uint16_t release_ms;
	uint16_t start_ms;
} ap2_profile_override_t;

typedef struct {
	int32_t demand_permille;
	int32_t aggression_permille;
	int32_t load_permille;
	uint8_t cadence_rpm;
	uint32_t speed_x100;
	bool pedaling;
	uint32_t elapsed_ticks;
} ap2_auto_input_t;

/* Upper bound on the effective gain after a rider trim. Beyond this the characteristic
 * saturates so early that the curve shape stops being visible at all, which makes every
 * profile feel the same - the opposite of what a trim is for. */
#define AP2_ASSIST_GAIN_MAX_PCT 400

/* How fast AUTO moves toward a stronger character, and how slowly it gives it back. */
#define AP2_AUTO_RISE_MS  800U
#define AP2_AUTO_FALL_MS 4000U
/* A heavy load at low cadence is the one situation AUTO must recognise even when the rider is
 * not asking loudly - that is exactly when a calm profile feels like it abandoned them. */
#define AP2_AUTO_LOW_CADENCE_RPM 45
#define AP2_AUTO_LOAD_BONUS 200

void ap2_profiles_reset(void);

/* The compiled behaviour of one fixed profile. AUTO ids return their CALM endpoint. */
const ap2_profile_t *ap2_profile_base(ap2_profile_id_t id);

/*
 * Advance the AUTO decision and resolve the parameters in force this tick. Safe to call for
 * fixed profiles too: they ignore the auto input and return their compiled behaviour.
 */
void ap2_profiles_resolve(ap2_profile_id_t id, const ap2_profile_override_t *ovr,
	const ap2_auto_input_t *in, ap2_profile_resolved_t *out);

/* The assist characteristic: 0..1000 effort in, 0..1000 response out. */
int32_t ap2_profile_shape(ap2_curve_t curve, int32_t permille);

/* True when this profile id is one of the two adaptive ones. */
bool ap2_profile_is_auto(ap2_profile_id_t id);

#endif /* AP2_PROFILES_H_ */
