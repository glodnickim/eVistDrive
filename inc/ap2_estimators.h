#ifndef AP2_ESTIMATORS_H_
#define AP2_ESTIMATORS_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * ASSIST PIPELINE V2 - RIDER AGGRESSION and LOAD / TERRAIN STATE.
 *
 * TWO ESTIMATORS, DELIBERATELY SEPARATE AND DELIBERATELY DIFFERENT SPEEDS.
 *
 *   rider_aggression   FAST. "How sharply is this rider riding right now?" It answers within
 *                      a stroke or two and forgets within a couple of seconds. It describes
 *                      the rider STYLE, so it shapes DYNAMICS - attack, the dynamic assist
 *                      term, release. It is never a global multiplier on the request: making
 *                      the whole motor stronger because the rider pushed sharply once is how
 *                      an assist becomes unpredictable.
 *
 *   load_state         SLOW. "Is the bike working hard - a climb, a headwind, soft ground?"
 *                      It needs several crank revolutions of agreeing evidence and lets go
 *                      even more slowly. It describes the SITUATION, so it shapes how much
 *                      sustained assist is available - the base term and, in AUTO, how much
 *                      power the profile is willing to offer.
 *
 * Mixing the two into one "effort" number is what makes assist behaviour impossible to reason
 * about: a sprint on the flat and a slow climb produce opposite dynamics requirements from
 * similar torque readings.
 *
 * UNITS. Both are permille, 0..1000, with no physical dimension - they are confidences.
 */

/* --- aggression -------------------------------------------------------------------- */

/* The window a demand rate of change is measured over. Long enough to be a gesture rather
 * than a sample, short enough to sit inside one leg push. */
#define AP2_AGGR_RATE_WINDOW_MS 50U
/* Rate of demand rise, in permille per 100 ms, mapped to aggression evidence. */
#define AP2_AGGR_RATE_LO 100
#define AP2_AGGR_RATE_HI 600
/* Stroke peak above the sustained base, in permille of the base, mapped to evidence. */
#define AP2_AGGR_PEAK_LO 100
#define AP2_AGGR_PEAK_HI 800
/* Cadence rise, in rpm per second, mapped to evidence. */
#define AP2_AGGR_CADENCE_WINDOW_MS 250U
#define AP2_AGGR_CADENCE_LO 2
#define AP2_AGGR_CADENCE_HI 15
#define AP2_AGGR_RISE_MS 120U
#define AP2_AGGR_FALL_MS 1500U

/* --- load / terrain ---------------------------------------------------------------- */

/* Sustained effort mapped to load evidence. */
#define AP2_LOAD_EFFORT_LO 250
#define AP2_LOAD_EFFORT_HI 900
/* Cadence, inverted: the same effort at a lower cadence means a heavier load. */
#define AP2_LOAD_CADENCE_HI_RPM 75
#define AP2_LOAD_CADENCE_LO_RPM 35
/* The window the speed trend is judged over: several crank revolutions, per the contract
 * that this estimator must not react to a single stroke. */
#define AP2_LOAD_SPEED_WINDOW_MS 1500U
/* Speed change, in 0.01 km/h over the window, below which speed counts as "not rising". */
#define AP2_LOAD_SPEED_FLAT_X100 15
/* Evidence added when a high sustained effort is not producing speed. */
#define AP2_LOAD_STAGNATION_BONUS 250
#define AP2_LOAD_RISE_MS 2500U
#define AP2_LOAD_FALL_MS 6000U

typedef struct {
	int32_t demand_permille;
	int32_t base_permille;
	int32_t stroke_peak_permille;
	uint8_t cadence_rpm;
	uint32_t speed_x100;
	bool pedaling;
	uint32_t elapsed_ticks;
} ap2_estimator_input_t;

typedef struct {
	int32_t aggression_permille;
	int32_t load_permille;
	/* Observation only: which piece of evidence was carrying each estimator this tick. */
	int32_t aggr_rate_evidence;
	int32_t aggr_peak_evidence;
	int32_t aggr_cadence_evidence;
	int32_t load_effort_evidence;
	int32_t load_cadence_evidence;
	bool load_speed_stagnant;
} ap2_estimator_output_t;

void ap2_estimators_reset(void);
void ap2_estimators_update(const ap2_estimator_input_t *in, ap2_estimator_output_t *out);

#endif /* AP2_ESTIMATORS_H_ */
