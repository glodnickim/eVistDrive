#ifndef ASSIST_EXTENDED_BOOST_H_
#define ASSIST_EXTENDED_BOOST_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * FW-095: Extended Boost — extra current DURING a hard pedal push, while the rider is
 * genuinely still pedalling forward.
 *
 * SAFETY SEMANTICS. This function may never keep the motor pulling after real forward
 * pedalling has stopped, and it may never claim that pedalling is happening when it is not.
 * The M820 has no independent brake-sensor input we can rely on, so a genuine PAS STOP is
 * treated conservatively: it ENDS the boost in the same control tick.
 *
 *   - It does NOT set, hold or fake pedaling_active. The rider-input state always reports
 *     what the sensor actually says.
 *   - It does NOT extend the release ramp and is not a release ramp.
 *   - It is not armed by a button, a wheel or motor Hall pulse, or dTorque/dt alone.
 *   - It never copies throttle or Walk Assist current.
 *   - It is off by default (duration 0).
 *
 * FW-084 (superseded) did the opposite: it armed on a push and then STARTED on the edge of
 * pedalling stopping, holding torque for up to a second with the cranks stationary, while
 * raising the profile's "pedalling" flag to suppress the release fade. That is post-PAS motor
 * overrun. It was never confirmed on the bike and has been removed rather than tuned. If such
 * a feature is ever wanted it must be a separate, explicitly reasoned safety function with its
 * own conditions — not a ride-feel setting.
 *
 * How it behaves now: a hard push TRIGGERS a TIMED boost, which then continues for as long as
 * forward pedalling remains active.
 *
 *   Trigger   load held above the configured threshold for EXT_BOOST_CONFIRM_MS, while
 *             pedaling_active and the ride latch are both true and the bike is moving.
 *   Running   the boost current is fixed at the trigger instant, from the peak of that push.
 *             BE PRECISE ABOUT WHAT IS RE-CHECKED WHILE IT RUNS: pedaling_active, the ride
 *             latch, motion, safety cut, crank reverse, sensor validity, level and bank — all
 *             every 4 kHz tick. The PEDAL LOAD IS NOT. Once triggered, easing off the pedal
 *             does not shorten the boost; only the timer or a cancel ends it. That is
 *             deliberate: a pedal stroke has dead spots, and re-testing the load would make
 *             the boost stutter at exactly the cadence it exists to help.
 *   Ends      on whichever comes first — the timer, pedaling_active going false, the latch
 *             dropping, or any other cancel condition.
 *
 * ONE PUSH, ONE BOOST. A boost that reached ACTIVE blocks re-arming until the load falls
 * EXT_BOOST_RELEASE_HYST_CENTIKG below the trigger, whether it ran its full time or was cut
 * short. Otherwise a rider who stops the cranks without releasing the pedal would get a second
 * boost from the same unbroken press on resuming.
 */

/* 4 kHz control loop, same as everywhere else in the ride core. */
#define EXT_BOOST_CONTROL_TICKS_PER_MS 4U
/* The load must be HELD this long above the threshold. A shorter spike is a single ADC
 * glitch, a chain slap or a pothole — never a rider decision. */
#define EXT_BOOST_CONFIRM_MS 30U
/* Ends the current qualifying window, and is also what re-arms the function after a boost has
 * run: the load must drop this far below the trigger before another boost may start.
 * Hysteresis only stabilizes the END of a push; it never moves the threshold the rider set. */
#define EXT_BOOST_RELEASE_HYST_CENTIKG 50U /* 0.50 kg */
/* The bike has to be genuinely moving. Same "actually moving" bar the rest of the ride
 * core uses; motion is passed in as a ready flag (see motion_valid). */
#define EXT_BOOST_MIN_SPEED_X100 100U /* 1.00 km/h */
#define EXT_BOOST_MIN_MOTOR_ERPS 10U

/*
 * Rider-facing limits. The trigger load is a PUBLIC kg value covering the WHOLE sensor
 * scale, 1.0 to 60.0 kg.
 *
 * It does not share the 0.1 kg step of the other start-load fields (FW-077), and that is a
 * deliberate trade: one wire byte at 0.1 kg tops out at 25.5 kg, which would have capped an
 * arming threshold well below what the sensor can read. Nothing else in the bank record was
 * free — the blob is at the 255 B transport ceiling — so the resolution is what had to give.
 *
 * The step is 0.5 kg rather than the 0.25 kg that would also have fit, so that every storable
 * value is exact at ONE decimal place, like every other kilogram field the rider sees. On a
 * 0.25 grid the UI had to show 20.00 and 8.25, which is both uglier and inconsistent. 0.5 kg
 * is 2.5 % of a 20 kg threshold — far finer than a rider can push repeatably.
 *
 * A trigger AT full scale is allowed and simply never fires: the peak is clamped to full
 * scale, so "peak > trigger" can never hold. The current formula returns 0 before it would
 * divide by a zero span; see compute_boost_iq().
 */
#define ASSIST_EXT_BOOST_TRIGGER_MIN_CENTIKG 100U  /* 1.0 kg */
#define ASSIST_EXT_BOOST_TRIGGER_MAX_CENTIKG 6000U /* 60.0 kg = TORQUE_PUBLIC_FULL_SCALE */
#define ASSIST_EXT_BOOST_TRIGGER_WIRE_STEP_CENTIKG 50U /* 0.5 kg per wire unit */
/* 20.0 kg: a shove the rider has to mean. The threshold only STARTS the function — the boost
 * current still comes from how far the push went above it — so a high default costs nothing
 * on an ordinary pedal stroke and keeps the feature from firing during normal riding. */
#define ASSIST_EXT_BOOST_TRIGGER_DEFAULT_CENTIKG 2000U /* 20.0 kg */
#define ASSIST_EXT_BOOST_STRENGTH_DEFAULT_PCT 100U
/*
 * Real milliseconds, not a percentage of anything — the removed Legacy overrun expressed its
 * duration as a percentage of Override_Duration, which is why nobody could say what a value
 * meant.
 *
 * FW-095 DECISION: the ceiling stays at 1000 ms. The semantics of this feature just changed
 * from "hold after the cranks stop" to "extra current while pedalling", and nothing has been
 * ridden yet. Widening the range in the same step would make the first bike test ambiguous —
 * a bad result could be the new semantics or the longer time. Raising it later is a one-line
 * change once the behaviour is confirmed. The boost can no longer outlive real pedalling, so
 * the ceiling is no longer what bounds the risk; it bounds how long one push may pay out.
 */
#define ASSIST_EXT_BOOST_DURATION_MAX_MS 1000U

typedef enum {
	ASSIST_EXT_BOOST_IDLE = 0,
	ASSIST_EXT_BOOST_QUALIFY,
	/* WIRE VALUE ONLY, never entered since FW-095. It meant "a push was confirmed and the
	 * boost is waiting for the cranks to stop" — the waiting state that made this a post-PAS
	 * overrun. A confirmed push now goes straight to ACTIVE. Kept so the app's decoder for
	 * byte 52 of 0x6029 keeps its numbering. */
	ASSIST_EXT_BOOST_ARMED_RESERVED,
	ASSIST_EXT_BOOST_ACTIVE
} assist_extended_boost_state_t;

/*
 * WIRE VALUES — reported in byte 52 of the 0x6029 diagnostics block. Without them a bench
 * log cannot tell "never armed" from "waiting for PAS STOP" from "cut by a limit".
 */
typedef enum {
	ASSIST_EXT_BOOST_CANCEL_NONE = 0,
	ASSIST_EXT_BOOST_CANCEL_DISABLED = 1,
	ASSIST_EXT_BOOST_CANCEL_SAFETY_CUT = 2,
	ASSIST_EXT_BOOST_CANCEL_REVERSE = 3,
	ASSIST_EXT_BOOST_CANCEL_SENSOR_INVALID = 4,
	ASSIST_EXT_BOOST_CANCEL_WALK = 5,
	ASSIST_EXT_BOOST_CANCEL_CALIBRATION = 6,
	ASSIST_EXT_BOOST_CANCEL_LEVEL_OR_BANK_CHANGE = 7,
	ASSIST_EXT_BOOST_CANCEL_MOTION_LOST = 8,
	/* WIRE VALUES ONLY, never reported since FW-095. Both belonged to the removed
	 * wait-for-PAS-STOP design: 9 fired when pedalling came back during a boost that had
	 * started because pedalling ended, and 10 when such a pending arming went stale. */
	ASSIST_EXT_BOOST_CANCEL_PEDALING_RESUMED_RESERVED = 9,
	ASSIST_EXT_BOOST_CANCEL_ARM_TIMEOUT_RESERVED = 10,
	ASSIST_EXT_BOOST_CANCEL_COMPLETED = 11,
	/* A bank was written while the module held state. The index of the bank and level did
	 * not change, so nothing else would have noticed — but the trigger, strength and
	 * duration under which the arming was made no longer exist. */
	ASSIST_EXT_BOOST_CANCEL_CONFIG_CHANGED = 12,
	/* FW-095: real forward pedalling stopped. THE defining cancel of this feature — it is what
	 * guarantees the boost can never become motor overrun past a PAS STOP. */
	ASSIST_EXT_BOOST_CANCEL_PEDALING_STOPPED = 13
} assist_extended_boost_cancel_t;

typedef struct {
	uint16_t trigger_load_centikg;
	uint8_t strength_pct;
	uint16_t duration_ms;
} assist_extended_boost_config_t;

typedef struct {
	/* THE RAW SENSOR STATE, straight from rider_input. The module reads it and never writes
	 * it back, directly or through the caller: a boost may not make the system believe the
	 * rider is pedalling. When this goes false the boost ends in the same tick. */
	bool pedaling_active;
	/* The ride latch. It cannot arm without forward crank direction, the configured PAS
	 * step count and the configured kg start threshold, so it IS the proof that assist
	 * started legally — the module never re-derives that from raw sensors. */
	bool pedal_assist_latched;
	/* Speed AND motor ERPS both above their minimums, decided by the caller so there is
	 * one owner of "the bike is moving" in the ride core. */
	bool motion_valid;
	bool safety_cut;
	bool walk_active;
	bool position_calibration_active;
	bool torque_sensor_valid;
	bool pas_sensor_valid;
	bool crank_reverse;
	uint8_t bank_index;
	uint8_t level_index;
	uint16_t pedal_load_centikg;
	int32_t ride_core_iq_limit;
} assist_extended_boost_input_t;

/*
 * FW-095: profile_hold_active is GONE and must not come back. It told ride_control to hold the
 * profile's "pedalling" flag true while the boost ran, which suppressed the release fade for a
 * rider who had stopped pedalling. That is exactly the "do not fake the input" rule this
 * module now obeys — and it is unnecessary, because the boost cannot outlive real pedalling.
 */
typedef struct {
	int32_t iq_target;
	bool active;
} assist_extended_boost_output_t;

/* Bench diagnostics only (0x6029 v5). Read-only snapshot; no pointer into live state. */
typedef struct {
	uint8_t state;
	bool arm_expired;
	uint16_t peak_load_centikg;
	int32_t boost_iq;
	uint16_t remaining_ms;
	uint8_t cancel_reason;
} assist_extended_boost_diag_t;

void assist_extended_boost_init(void);
void assist_extended_boost_reset(uint8_t reason);
void assist_extended_boost_update(
	const assist_extended_boost_input_t *input,
	const assist_extended_boost_config_t *config,
	assist_extended_boost_output_t *output);

void assist_extended_boost_get_diag(assist_extended_boost_diag_t *diag);

#endif /* ASSIST_EXTENDED_BOOST_H_ */
