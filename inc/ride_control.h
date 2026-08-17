#ifndef RIDE_CONTROL_H_
#define RIDE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * FW-094: there is no engine type any more. The ride core is the only assist pipeline, so
 * nothing selects, reports or branches on one. The single remaining trace is a constant byte
 * in the 0x6028/0x6029 telemetry blocks that the shipped app still parses — it lives in
 * CAN_Display.c as a protocol constant, not as a runtime state.
 */

typedef struct {
	uint32_t speed_x100;
	uint8_t cadence_rpm;
	uint8_t assist_level_index;
	uint32_t battery_voltage_mv;
	int32_t iq_scale;
	int32_t ride_core_iq_limit;
	int32_t phase_current_max;
	int32_t current_iq;
	int32_t current_id;
	uint16_t voltage_raw;
	int16_t voltage_min_raw;
	int16_t controller_temperature_c;
	uint16_t cadence_filtered_x8;
	uint16_t speed_limit_x100;
	bool legal_enabled;
	bool offroad;
	bool walk_active;
	bool position_calibration_active;
	/*
	 * FW-109 v2: the NON-DIRECTION hard-cut reasons only - brake / overtemp cutoff / torque-
	 * sensor fault / torque calibration. Neither a reverse step nor an illegal PAS transition is
	 * folded in here: both are their own fact, read straight from rider_input_t.
	 * direction_inhibit_active (src/pas_direction.c's direction safety automaton), which is
	 * immediate (the FIRST such step, not a multi-step confirm) and therefore strictly
	 * supersedes the old Backwards_counter-based reverse term this field's FW-108 predecessor
	 * (safety_cut_reverse) carried. ride_control.c combines the two itself (see its own
	 * hard_cut) - see main.c for exactly what feeds this field. (Renamed from
	 * safety_cut_non_reverse: FW-109 v2 added INVALID as a second direction-related cause, so
	 * "non-reverse" no longer precisely describes what this field excludes.)
	 */
	bool safety_cut_non_direction;
	int32_t throttle_iq;   // FW-030: throttle current (mapped from ADC in main.c); floor on ride-core output
} ride_control_input_t;

/*
 * FW-096 diagnostics: WHY is the current zero?
 *
 * Pure observation — nothing here changes a decision, it only records which gate the pipeline
 * went through on the last tick. Added because "MS.i_q_setpoint is 0" on its own says nothing
 * about who zeroed it, which turned a regression hunt into guesswork.
 */
#define RIDE_DBG_WALK             0x01  /* Walk Assist owns Iq this tick */
#define RIDE_DBG_CALIBRATION      0x02  /* position calibration owns Iq this tick */
#define RIDE_DBG_HARD_CUT         0x04  /* brake / reverse / overtemp / torque fault / cal */
#define RIDE_DBG_LEVEL_ZERO       0x08  /* assist level 0 */
#define RIDE_DBG_NOT_LATCHED      0x10  /* ride latch not armed -> assist target forced 0 */
#define RIDE_DBG_MODE_UNSUPPORTED 0x20  /* assist_modes_calculate() reported unsupported */
#define RIDE_DBG_LIMITER_ZEROED   0x40  /* a non-zero demand was taken to 0 by assist_limits */
#define RIDE_DBG_COAST_RELEASE    0x80  /* FW-048 coast-out cut the tail */

uint8_t ride_control_get_debug_flags(void);

/*
 * FW-112-DIAG: WHY current is held at 0 on the last tick, computed fresh by the layer that
 * owns permission and demand. Returns the FW112_REASON_* bitfield from inc/fw112_diag.h
 * (multiple bits can be set - every cause that was true that tick). Observation only: nothing
 * reads this to decide. Guarded so the production build links no diagnostic state.
 */
#if CAN_DIAGNOSTICS_ENABLE
uint8_t ride_control_get_diag_reason(void);
#endif

/*
 * FW-112 v2 HOLD-OWNER observability: the current assist_hold_ticks grace counter, read-only.
 * The hold is armed/refreshed ONLY by a supported mode with a positive mode_output.iq_request
 * (see ride_control_update) - NEVER by the raw filtered torque crossing the run deadband alone.
 * Tests use this to prove that contract ("raw torque above the deadband must not arm the hold
 * when the mode result is 0"). Observation only - nothing reads this to make a decision.
 */
uint16_t ride_control_get_assist_hold_ticks(void);

/*
 * FW-109: the ride SESSION automaton's current state (src/ride_session.h's ride_session_state_t,
 * returned as uint8_t to keep this header free of that one - same discipline the rest of this
 * file already uses). 0=COLD, 1=ACTIVE, 2=SUSPENDED_BY_DIRECTION; 3 is the RESERVED legacy
 * WAIT_REARM_LOAD value from FW-109/FW-112 v1 - never entered by production in v2, kept only so
 * older diagnostic decoders still recognise the numeric byte. Observation only - nothing reads
 * this to decide; the module itself is the single owner of the decision.
 */
uint8_t ride_control_get_session_state(void);

/*
 * FW-101 diagnostics: the values that existed AT THE INSTANT the ride latch armed.
 *
 * Measurement only — nothing reads this to make a decision. It exists because the 40 ms
 * diagnostic frames cannot see a re-engagement: PAS steps arrive every 12-39 ms while riding,
 * so the whole latch/seed/limiter transition can happen between two frames. These are captured
 * where they are produced, at the 4 kHz tick, so an episode record can say WHICH stage of the
 * chain spent the time — the RUN estimator seed, the power filter, the limiter or the ramp.
 *
 * `seq` increments once per arming. A reader detects a new arming by seeing it change, which
 * is what anchors the episode timings in main.c.
 */
typedef struct {
	uint16_t seq;
	uint16_t load_centikg;      /* raw pedal load the latch armed on */
	uint16_t fast_native;       /* the 35 ms filtered assist delta at that moment */
	uint16_t run_seed_native;   /* what was pre-loaded into the RUN estimator */
	int32_t  iq_after_limits;   /* target after latch and limiters, BEFORE the final ramp */
	/* FW-107: whether THIS arming took the fast-rearm path (RIDE_REARM_ARMED, real RUN-level
	 * demand, no start-load threshold) rather than the normal load-threshold latch. */
	bool     fast_rearm;
	/*
	 * LIVE, updated every tick — not part of the arming snapshot. This is what lets an
	 * episode separate the ramp from everything before it: the pipeline can be asking for
	 * full power again while the ramp is still walking the current up to it.
	 */
	int32_t  iq_pre_ramp;
	/*
	 * FW-107: LIVE, updated every tick, same discipline as iq_pre_ramp - the ride-latch
	 * block's OWN output (after the min-Iq floor, BEFORE Extended Boost/hard-cut/the shared
	 * limiter). Lets a test or a measurement tell "the floor alone produced this" apart from
	 * "the real assist-mode calculation produced this", which iq_pre_ramp alone cannot: by the
	 * time iq_pre_ramp is captured, Extended Boost and the limiter have both had a chance to
	 * change the number for reasons that have nothing to do with the latch.
	 */
	int32_t  iq_after_latch_floor;
} ride_arm_snapshot_t;

void ride_control_get_arm_snapshot(ride_arm_snapshot_t *out);

/*
 * FW-102: the start GATE as it actually stood on the LAST tick — live, not an arming snapshot.
 * Exists so a measurement can say "the load gate was met, the step gate was not" instead of
 * re-deriving the same threshold logic a second time in main.c against a stale constant, which
 * is how the 8-to-5 arithmetic went wrong earlier in this investigation.
 */
typedef struct {
	uint8_t  required_steps;          /* what crank_forward_steps has to reach right now */
	uint16_t load_threshold_centikg;  /* what torque_load_centikg has to reach right now */
	bool     bike_rolling;
} ride_gate_snapshot_t;

void ride_control_get_gate_snapshot(ride_gate_snapshot_t *out);

void ride_control_init(void);
void ride_control_update(const ride_control_input_t *input);

#endif /* RIDE_CONTROL_H_ */
