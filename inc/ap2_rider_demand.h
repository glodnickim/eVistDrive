#ifndef AP2_RIDER_DEMAND_H_
#define AP2_RIDER_DEMAND_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * ASSIST PIPELINE V2 - INPUT VALIDATION, TORQUE NORMALIZATION, RIDER DEMAND,
 * PEDAL-CYCLE BASE/DYNAMIC SPLIT.
 *
 * WHY THIS BLOCK EXISTS AT ALL
 * ----------------------------
 * Pedal torque pulsates. One leg push per half revolution, with a dead spot between them:
 *
 *          /\              /\
 *      ___/  \____________/  \____
 *
 * Copying that into the motor current gives a motor that surges and drops twice per crank
 * revolution, which is what the rider feels as the assist "pumping". The answer is NOT to
 * low-pass it away - that buys smoothness by throwing away responsiveness, and stacking more
 * filters to get the responsiveness back is how the legacy path ended up with four of them.
 *
 * The answer is to MODEL the signal instead of smoothing it. One pedal stroke carries two
 * separate pieces of information:
 *
 *   how hard the rider is working overall      -> a SUSTAINED level that should not collapse
 *                                                 in the dead spot between strokes
 *   whether the rider just pushed HARDER        -> a FAST excess above that level
 *
 * This module produces both, from one conditioned measurement:
 *
 *   effort      the per-tick normalized pedal force, after deadband and one noise filter
 *   demand      rider_demand: effort made robust to a single bad sample (the rider's intent)
 *   base        the sustained level, held across the dead spot by a cadence-aware decay
 *   dynamic     max(0, demand - base) with its own fast envelope: the "pushed harder" term
 *
 * UNITS. Everything leaving this module is permille (0..1000) of the rider-effort full scale
 * (tuning_config_assist_torque_full_scale_centikg()). That full scale is a RIDE-FEEL setting
 * and has nothing to do with the sensor calibration, which torque_input.c already applied -
 * the input here is a calibrated physical pedal force in 0.01 kgf.
 */

/*
 * The single deadband of the assist path, in physical force. Below it the pipeline reports
 * no effort at all. It exists to keep sensor rest noise and the weight of a resting foot out
 * of the demand, and it is expressed in kgf rather than ADC counts so it means the same thing
 * after a sensor recalibration.
 */
#define AP2_EFFORT_DEADBAND_CENTIKG 15U

/*
 * THE ONE FILTER ON THE MEASUREMENT PATH. Its job is sensor noise, nothing else - it is far
 * too short to touch pedal ripple (a stroke at 90 rpm lasts 333 ms). Pedal ripple is handled
 * by the base/dynamic model below, which is the entire point of this block.
 */
#define AP2_EFFORT_LPF_MS 20U

/* One leg push = half a crank revolution, so a stroke lasts 30000/cadence ms. The clamps
 * cover 20..200 rpm; outside that the cadence reading is not trustworthy as a stroke clock. */
#define AP2_STROKE_MIN_MS  150U
#define AP2_STROKE_MAX_MS 1500U

/* How fast the sustained base may RISE toward a higher effort. Short enough that a genuinely
 * harder effort lifts the base within one stroke, long enough that a single peak does not. */
#define AP2_BASE_RISE_MS 120U

/*
 * How fast the base may FALL. The effective value is max(profile base_hold_ms, 1.5 stroke
 * periods), so at low cadence - where the dead spot is longest and the legacy path collapsed
 * hardest - the base outlives the gap by construction rather than by a tuned constant.
 */
#define AP2_BASE_FALL_STROKE_NUM 3U
#define AP2_BASE_FALL_STROKE_DEN 2U

/* The dynamic term's own envelope: near-immediate to a harder push, unhurried on the way
 * back, so one stroke's excess is delivered as a push rather than as a spike. */
#define AP2_DYNAMIC_RISE_MS  25U
#define AP2_DYNAMIC_FALL_MS 150U

/* Peak-hold decay used only to describe the stroke to the aggression estimator. */
#define AP2_STROKE_PEAK_DECAY_STROKES 2U

typedef struct {
	uint16_t load_centikg;        /* calibrated pedal force from torque_input.c */
	bool torque_valid;            /* sensor healthy and not in calibration */
	bool pedaling;                /* PAS lifecycle says the cranks are driving forward */
	uint8_t cadence_rpm;          /* conditioned control cadence */
	uint16_t full_scale_centikg;  /* ride-feel axis; 0 falls back to the compiled default */
	uint16_t base_hold_ms;        /* profile floor for the base decay */
	uint32_t elapsed_ticks;
} ap2_demand_input_t;

typedef struct {
	int32_t effort_permille;       /* conditioned per-tick pedal force */
	int32_t demand_permille;       /* rider_demand */
	int32_t base_permille;         /* sustained level across the pedal cycle */
	int32_t dynamic_permille;      /* fast excess above the sustained level */
	int32_t stroke_peak_permille;  /* decaying peak-hold, for the aggression estimator */
	uint16_t stroke_period_ms;     /* the pedal clock this tick was evaluated against */
	uint16_t load_centikg;         /* echoed input, for telemetry */
} ap2_demand_output_t;

#define AP2_FULL_SCALE_DEFAULT_CENTIKG 6000U

void ap2_rider_demand_reset(void);
void ap2_rider_demand_update(const ap2_demand_input_t *in, ap2_demand_output_t *out);

/*
 * Seed the sustained base at engagement so the first stroke of a ride (or of a resumed ride)
 * is answered at its real magnitude instead of being ramped up from zero by the base rise
 * time. Called by the pipeline on the PAS engagement edge and nowhere else.
 */
void ap2_rider_demand_seed_base(int32_t permille);

#endif /* AP2_RIDER_DEMAND_H_ */
