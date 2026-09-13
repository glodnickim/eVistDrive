#ifndef RIDE_CONTROL_H_
#define RIDE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "fast_iq_slew.h"   /* the final-Iq-slew mailbox type/producer API */

/*
 * RIDE CONTROL - selection and publication only.
 *
 * The assist algorithm lives in assist_pipeline.c. This layer decides WHICH of the three
 * possible owners of the motor is active this tick - position calibration, Walk Assist, or
 * pedal assist - and publishes that owner's command through the single final-Iq mailbox.
 *
 * The rider-facing diagnostics that used to hang off this file (the latch arming snapshot, the
 * start gate snapshot, the hold-grace counter, the FW-112 permission and reason bitfields) were
 * instrumentation for mechanisms that no longer exist. Their replacement is
 * assist_pipeline_telemetry(), which publishes every stage of the one chain rather than the
 * internals of several competing ones.
 */

typedef struct {
	uint32_t speed_x100;
	uint8_t cadence_rpm;
	uint8_t assist_level_index;
	uint32_t battery_voltage_mv;
	int32_t iq_scale;
	int32_t ride_core_iq_limit;
	int32_t phase_current_max;
	/*
	 * The battery-current limiter is an UPSTREAM Iq cap: it is applied inside the pipeline's
	 * limiter chain, BEFORE the single final Iq trajectory, so PI_iq always remains an
	 * Iq-domain current regulator and never becomes a battery-current regulator.
	 *   battery_current_mA  measured battery current (MS.Battery_Current, mA)
	 *   battery_current_max configured maximum battery current (mA)
	 *   u_abs               voltage-space-vector magnitude, 2048 = 2^11 (FOC _U_MAX domain)
	 *   cal_i               phase-current scale (CAL_I)
	 */
	int32_t battery_current_mA;
	int32_t battery_current_max;
	int32_t u_abs;
	int32_t cal_i;
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
	 * The NON-DIRECTION hard-cut reasons only - brake / overtemp cutoff / torque-sensor fault
	 * / torque calibration. A reverse step and an illegal PAS transition are not folded in
	 * here: both are their own fact, read straight from rider_input_t.direction_inhibit_active
	 * (src/pas_direction.c), and the PAS lifecycle in the pipeline combines them itself.
	 */
	bool safety_cut_non_direction;
	/*
	 * The SERVICE subset of safety_cut_non_direction - the pedal-load calibration, which is a
	 * workshop procedure rather than a riding event. It changes no cut decision; it exists so
	 * the published zero-policy can refuse Quiet Zero on a service cut while still granting it
	 * on a real brake / overtemperature / torque-fault / reverse release.
	 */
	bool service_cut_active;
	/* Throttle current, mapped from the ADC in main.c. A separate rider input, judged as
	 * non-pedal in the shared limiter chain; never an assist override. */
	int32_t throttle_iq;
	bool start_phase;
	/*
	 * Elapsed TIMER1 control periods represented by this update. main.c derives it from the
	 * free-running hardware clock, so no filter or ramp in the pipeline silently changes its
	 * time constant when the main-loop work flag coalesces periods. A zero from an older
	 * caller or a simple test means one ordinary period.
	 */
	uint32_t elapsed_ticks;
} ride_control_input_t;

/*
 * The ride SESSION state as the older diagnostic decoders expect it:
 * 0 = COLD, 1 = ACTIVE, 2 = SUSPENDED_BY_DIRECTION. Projected from the pipeline's PAS
 * lifecycle, which is a superset. Observation only.
 */
uint8_t ride_control_get_session_state(void);

/* The full PAS lifecycle state (ap2_pas_state_t). Observation only. */
uint8_t ride_control_get_pas_state(void);

/*
 * Is the battery-current limiter currently holding the request down? Exposed so main.c can
 * drive the legacy BC_limit_flag diagnostic from the SINGLE source of truth. Observation only.
 */
bool ride_control_battery_limit_active(void);

/*
 * The shared 4 kHz -> 16 kHz final-Iq-slew mailbox. The 4 kHz ride/control domain owns the
 * producer side; the 16 kHz FOC ISR in main.c consumes it via fast_iq_slew_tick().
 */
fast_iq_slew_mailbox_t *ride_control_final_iq_slew_mailbox(void);

/*
 * final_iq_requested is foreground demand used only to decide whether a bridge-off cold start
 * is needed; it is never the PI reference. Service and comm-loss use these explicit mailbox
 * commands, so their requests cannot be overwritten by the next 16 kHz tick from a stale
 * normal command.
 */
int32_t ride_control_final_iq_requested(void);
void ride_control_force_final_iq_zero(void);
void ride_control_request_service_iq(int32_t iq_target);

void ride_control_init(void);
void ride_control_update(const ride_control_input_t *input);

#endif /* RIDE_CONTROL_H_ */
