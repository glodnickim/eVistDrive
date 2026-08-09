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
	bool safety_cut;
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
} ride_arm_snapshot_t;

void ride_control_get_arm_snapshot(ride_arm_snapshot_t *out);

void ride_control_init(void);
void ride_control_update(const ride_control_input_t *input);

#endif /* RIDE_CONTROL_H_ */
