#ifndef ASSIST_EXTENDED_BOOST_H_
#define ASSIST_EXTENDED_BOOST_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * FW-084: Extended Boost — deliberate drive hold after the cranks stop.
 *
 * This is NOT a longer release ramp. The rider ARMS it with a hard pedal push; once the
 * cranks are recognized as stopped the motor keeps pulling for a configured time at a
 * current derived from the PEAK LOAD OF THE LATEST QUALIFYING PUSH, and only then does the
 * existing single release ramp (release_ms) take over. It exists for steps, rocks and short
 * breaks in pedalling on a technical climb.
 *
 * Deliberately NOT triggered by: a button, a wheel or motor Hall pulse, or a large
 * dTorque/dt on its own. It never copies throttle or Walk Assist current, and it is off by
 * default (duration 0).
 *
 * The old monolith's EXTENDED_BOOST_ENABLE / Overrun_* block is a different mechanism with
 * different state sources; it starts its counter at the wrong moment and bypasses part of
 * the ride core. Nothing is carried over from it.
 */

/* 4 kHz control loop, same as everywhere else in the ride core. */
#define EXT_BOOST_CONTROL_TICKS_PER_MS 4U
/* The load must be HELD this long above the threshold. A shorter spike is a single ADC
 * glitch, a chain slap or a pothole — never a rider decision. */
#define EXT_BOOST_CONFIRM_MS 30U
/* Ends the current qualifying window. Hysteresis only stabilizes the END of a push; it
 * never moves the threshold the rider configured. */
#define EXT_BOOST_RELEASE_HYST_CENTIKG 50U /* 0.50 kg */
/* An arming goes stale after this long without a qualifying load, so a very hard crank
 * turn cannot be replayed many seconds later. */
#define EXT_BOOST_ARM_TIMEOUT_MS 1500U
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
 * A trigger AT full scale is allowed and simply never arms: the peak is clamped to full
 * scale, so "peak > trigger" can never hold. The current formula returns 0 before it would
 * divide by a zero span; see compute_boost_iq().
 */
#define ASSIST_EXT_BOOST_TRIGGER_MIN_CENTIKG 100U  /* 1.0 kg */
#define ASSIST_EXT_BOOST_TRIGGER_MAX_CENTIKG 6000U /* 60.0 kg = TORQUE_PUBLIC_FULL_SCALE */
#define ASSIST_EXT_BOOST_TRIGGER_WIRE_STEP_CENTIKG 50U /* 0.5 kg per wire unit */
/* 20.0 kg: a shove the rider has to mean. The threshold only ARMS the function — the boost
 * current still comes from how far the push went above it — so a high default costs nothing
 * on an ordinary pedal stroke and keeps the feature from arming during normal riding. */
#define ASSIST_EXT_BOOST_TRIGGER_DEFAULT_CENTIKG 2000U /* 20.0 kg */
#define ASSIST_EXT_BOOST_STRENGTH_DEFAULT_PCT 100U
#define ASSIST_EXT_BOOST_DURATION_MAX_MS 1000U

typedef enum {
	ASSIST_EXT_BOOST_IDLE = 0,
	ASSIST_EXT_BOOST_QUALIFY,
	ASSIST_EXT_BOOST_ARMED,
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
	ASSIST_EXT_BOOST_CANCEL_PEDALING_RESUMED = 9,
	ASSIST_EXT_BOOST_CANCEL_ARM_TIMEOUT = 10,
	ASSIST_EXT_BOOST_CANCEL_COMPLETED = 11,
	/* A bank was written while the module held state. The index of the bank and level did
	 * not change, so nothing else would have noticed — but the trigger, strength and
	 * duration under which the arming was made no longer exist. */
	ASSIST_EXT_BOOST_CANCEL_CONFIG_CHANGED = 12
} assist_extended_boost_cancel_t;

typedef struct {
	uint16_t trigger_load_centikg;
	uint8_t strength_pct;
	uint16_t duration_ms;
} assist_extended_boost_config_t;

typedef struct {
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

typedef struct {
	int32_t iq_target;
	bool profile_hold_active;
	bool armed;
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
