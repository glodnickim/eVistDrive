#ifndef AP2_TORQUE_CHAIN_H_
#define AP2_TORQUE_CHAIN_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * ASSIST PIPELINE V2 - TORQUE CHAIN.
 *
 * This module implements a torque-to-demand chain modeled on the classic
 * 6-state automaton with envelope, linear mapping, asymmetric slew and
 * recovery gate. It replaces the base/dynamic split with a single demand
 * signal shaped by an envelope and rate-limited by an asymmetric slew.
 *
 * UNITS:
 *   Input load: centikg (0.01 kgf) from torque_input.c
 *   Internal thresholds: centikg
 *   Envelope: centikg
 *   Mapping output: 0..4096 (legacy scale)
 *   Final output: permille (0..1000) of rider-effort full scale
 *
 * STATES:
 *   RESET      -> GATE (initial entry)
 *   GATE       -> CONFIRM (threshold held for 400 ms)
 *   CONFIRM    -> ACTIVE (threshold confirmed)
 *   ACTIVE     -> HYSTERESIS (below exit threshold)
 *   HYSTERESIS -> GATE (return to gate)
 *   RECOVERY   -> GATE/CONFIRM (post-reverse recovery gate)
 *
 * ENVELOPE:
 *   Instant attack, cadence-dependent release (k = 8 * cadence, min 40).
 *   Matches the classic envelope behavior: holds effort across dead spots.
 *
 * MAPPING:
 *   Linear from active_threshold to saturation_threshold, then flat.
 *   Output scaled to 0..4096 legacy range, then converted to permille.
 *
 * SLEW:
 *   Asymmetric: rise rate-limited, fall instant (snap to target).
 *
 * RECOVERY GATE:
 *   Entered on reverse/invalid direction. Requires sustained load above
 *   recovery threshold to exit back to GATE, or load drop to enter CONFIRM.
 */

#ifndef AP2_TORQUE_CHAIN_DEADBAND_CENTIKG
#define AP2_TORQUE_CHAIN_DEADBAND_CENTIKG 55U
#endif

#ifndef AP2_TORQUE_CHAIN_GATE_THRESHOLD_CENTIKG
#define AP2_TORQUE_CHAIN_GATE_THRESHOLD_CENTIKG 180U
#endif

#ifndef AP2_TORQUE_CHAIN_CONFIRM_THRESHOLD_CENTIKG
#define AP2_TORQUE_CHAIN_CONFIRM_THRESHOLD_CENTIKG 120U
#endif

#ifndef AP2_TORQUE_CHAIN_ACTIVE_THRESHOLD_CENTIKG
#define AP2_TORQUE_CHAIN_ACTIVE_THRESHOLD_CENTIKG 180U
#endif

#ifndef AP2_TORQUE_CHAIN_SATURATION_CENTIKG
#define AP2_TORQUE_CHAIN_SATURATION_CENTIKG 450U
#endif

#ifndef AP2_TORQUE_CHAIN_RECOVERY_THRESHOLD_CENTIKG
#define AP2_TORQUE_CHAIN_RECOVERY_THRESHOLD_CENTIKG 70U
#endif

#ifndef AP2_TORQUE_CHAIN_GATE_TIME_MS
#define AP2_TORQUE_CHAIN_GATE_TIME_MS 400U
#endif

#ifndef AP2_TORQUE_CHAIN_CONFIRM_TIME_MS
#define AP2_TORQUE_CHAIN_CONFIRM_TIME_MS 250U
#endif

#ifndef AP2_TORQUE_CHAIN_SLEW_RISE_PERMILLE_PER_10MS
#define AP2_TORQUE_CHAIN_SLEW_RISE_PERMILLE_PER_10MS 100U
#endif

/* Crank-angle window for base term (mimics original RUN estimator).
 * 96 steps per revolution = 3.75 deg per step. 180 deg = 48 steps (half revolution). */
#ifndef AP2_TORQUE_CHAIN_BASE_WINDOW_STEPS
#define AP2_TORQUE_CHAIN_BASE_WINDOW_STEPS 48U
#endif
#ifndef AP2_TORQUE_CHAIN_BASE_WINDOW_STEPS_MAX
#define AP2_TORQUE_CHAIN_BASE_WINDOW_STEPS_MAX 96U
#endif

/* Runtime config for crank-angle window stepping. */
#define AP2_TORQUE_CHAIN_BASE_WINDOW_STEP_TICKS_DEFAULT 0U

#define AP2_TORQUE_CHAIN_PERMILLE_MAX 1000
#define AP2_TORQUE_CHAIN_LEGACY_MAX 4096U

/* Reuse the same input/output structs as ap2_rider_demand for compatibility. */
#include "ap2_rider_demand.h"

void ap2_torque_chain_reset(void);
void ap2_torque_chain_update(const ap2_demand_input_t *in, ap2_demand_output_t *out);

/*
 * Seed the internal state at engagement so the first stroke is answered
 * at its real magnitude. Called by the pipeline on the PAS engagement edge.
 */
void ap2_torque_chain_seed(int32_t permille);

/* Runtime configuration (set from profile or tuning config). */
void ap2_torque_chain_set_thresholds(uint16_t gate_ckg, uint16_t confirm_ckg,
                                     uint16_t active_ckg, uint16_t saturation_ckg,
                                     uint16_t recovery_ckg, uint16_t gate_ms,
                                     uint16_t confirm_ms, uint16_t slew_rise_permille_per_10ms);

#endif /* AP2_TORQUE_CHAIN_H_ */