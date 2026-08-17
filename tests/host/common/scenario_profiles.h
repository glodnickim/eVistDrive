/*
 * TEST-002: shared scenario constants for the high-cadence benchmark. Single source of
 * truth for TEST A (torque_revolution_bench_host.c) and TEST B
 * (power_revolution_bench_host.c), and for documentation/TEST_002_HIGH_CADENCE_BENCHMARK_REPORT_PL.md
 * (which quotes these values rather than re-stating them, so the two cannot drift apart).
 *
 * Every torque_torque_shape_t here is a TEST STIMULUS definition (see crank_model.h) -
 * none of this is firmware. Nothing in this file is read by any file under src/.
 */
#ifndef SCENARIO_PROFILES_H
#define SCENARIO_PROFILES_H

#include <stddef.h>
#include <string.h>

#include "crank_model.h"

/*
 * PROFILE_BASELINE is byte-for-byte the profile TEST-001 (foundation card) used for
 * RUN_60..RUN_120. Kept identical on purpose: it is both this card's "load = MEDIUM"
 * point and its "asymmetric" point, and it is the one profile for which a direct
 * before/after (TIME6S vs REV20 windowing) comparison is meaningful — see section 9 of
 * the report ("6s vs 20 revolutions").
 */
static const crank_torque_shape_t PROFILE_BASELINE = {
	.mean_native_delta = 300.0, .ripple_pct = 40.0, .asymmetry_pct = 15.0,
	.dead_spot_depth_pct = 30.0, .dead_spot_width_deg = 20.0, .phase_shift_deg = 0.0
};

/* Load sweep (card section 10): same shape, only mean pedal force differs. */
static const crank_torque_shape_t PROFILE_LOAD_LOW = {
	.mean_native_delta = 150.0, .ripple_pct = 40.0, .asymmetry_pct = 15.0,
	.dead_spot_depth_pct = 30.0, .dead_spot_width_deg = 20.0, .phase_shift_deg = 0.0
};
static const crank_torque_shape_t PROFILE_LOAD_HIGH = {
	.mean_native_delta = 500.0, .ripple_pct = 40.0, .asymmetry_pct = 15.0,
	.dead_spot_depth_pct = 30.0, .dead_spot_width_deg = 20.0, .phase_shift_deg = 0.0
};

/* Symmetry sweep (card section 15): same mean/ripple as PROFILE_BASELINE, only leg
 * balance / dead-spot severity differs. PROFILE_BASELINE itself IS the asymmetric case
 * here (asymmetry_pct=15) - not duplicated under a second name. */
static const crank_torque_shape_t PROFILE_SYMMETRIC = {
	.mean_native_delta = 300.0, .ripple_pct = 40.0, .asymmetry_pct = 0.0,
	.dead_spot_depth_pct = 30.0, .dead_spot_width_deg = 20.0, .phase_shift_deg = 0.0
};
static const crank_torque_shape_t PROFILE_DEADSPOT_ENHANCED = {
	.mean_native_delta = 300.0, .ripple_pct = 40.0, .asymmetry_pct = 15.0,
	.dead_spot_depth_pct = 60.0, .dead_spot_width_deg = 35.0, .phase_shift_deg = 0.0
};

typedef struct {
	const char *name;
	const crank_torque_shape_t *shape;
} named_profile_t;

static const named_profile_t NAMED_PROFILES[] = {
	{ "BASELINE", &PROFILE_BASELINE },
	{ "LOAD_LOW", &PROFILE_LOAD_LOW },
	{ "LOAD_HIGH", &PROFILE_LOAD_HIGH },
	{ "SYMMETRIC", &PROFILE_SYMMETRIC },
	{ "DEADSPOT", &PROFILE_DEADSPOT_ENHANCED },
};
#define NAMED_PROFILE_COUNT (sizeof(NAMED_PROFILES) / sizeof(NAMED_PROFILES[0]))

static const crank_torque_shape_t *find_named_profile(const char *name)
{
	for (size_t i = 0; i < NAMED_PROFILE_COUNT; i++) {
		if (strcmp(NAMED_PROFILES[i].name, name) == 0) {
			return NAMED_PROFILES[i].shape;
		}
	}
	return NULL;
}

/*
 * Revolution-window methodology (card section 2). WARMUP_REVOLUTIONS is deliberately
 * larger than the card's suggested 5-10: section 5 asks the harness to MEASURE how many
 * revolutions the filter chain needs to settle, not assume a number - see
 * torque_revolution_bench_host.c's --warmup-scan mode and
 * documentation/TEST_002_HIGH_CADENCE_BENCHMARK_REPORT_PL.md section 3. 8 is the
 * DEFAULT used once that scan confirms it is enough; it is not asserted here as correct
 * a priori.
 */
#define DEFAULT_WARMUP_REVOLUTIONS 8U
#define DEFAULT_MEASURE_REVOLUTIONS 20U

/* Cadence points used across this card's sweeps (section 11: "sensible matrix", not a
 * full cross product). Full set for baseline/torque sweeps; REDUCED set for the more
 * expensive load/voltage matrices (documented per-sweep, not repeated here). */
static const double CADENCE_FULL_SET[] = { 60.0, 80.0, 90.0, 100.0, 110.0, 120.0 };
#define CADENCE_FULL_COUNT (sizeof(CADENCE_FULL_SET) / sizeof(CADENCE_FULL_SET[0]))
static const double CADENCE_REDUCED_SET[] = { 60.0, 80.0, 100.0, 120.0 };
#define CADENCE_REDUCED_COUNT (sizeof(CADENCE_REDUCED_SET) / sizeof(CADENCE_REDUCED_SET[0]))
static const double CADENCE_HIGH_FOCUS_SET[] = { 80.0, 100.0, 110.0, 120.0 };
#define CADENCE_HIGH_FOCUS_COUNT (sizeof(CADENCE_HIGH_FOCUS_SET) / sizeof(CADENCE_HIGH_FOCUS_SET[0]))

/*
 * motor_voltage_utilization sweep (card section 6). Scale is MOTOR_VOLTAGE_UTILIZATION_SCALE
 * = 2048 in src/assist_modes.c (duty-cycle-like quantity, main.c clamps MS.u_abs to
 * [0,2048] before assigning it here) - 1900 is close to that ceiling deliberately, to
 * see whether the power/voltage cross-check inside finish_power_request() ever binds
 * before saturation.
 */
static const int32_t MVU_SWEEP[] = { 0, 800, 1200, 1400, 1600, 1800, 1900 };
#define MVU_SWEEP_COUNT (sizeof(MVU_SWEEP) / sizeof(MVU_SWEEP[0]))
static const int32_t MVU_REPRESENTATIVE[] = { 0, 1600 }; /* unclamped reference + likely-clamped point */
#define MVU_REPRESENTATIVE_COUNT (sizeof(MVU_REPRESENTATIVE) / sizeof(MVU_REPRESENTATIVE[0]))

/* Battery voltage sweep (card section 9), millivolts - matches ride_control_input_t.battery_voltage_mv. */
static const uint32_t BATTERY_VOLTAGE_SWEEP_MV[] = { 42000U, 40000U, 38000U, 36000U };
#define BATTERY_VOLTAGE_SWEEP_COUNT (sizeof(BATTERY_VOLTAGE_SWEEP_MV) / sizeof(BATTERY_VOLTAGE_SWEEP_MV[0]))
#define BATTERY_VOLTAGE_DEFAULT_MV 42000U

#endif /* SCENARIO_PROFILES_H */
