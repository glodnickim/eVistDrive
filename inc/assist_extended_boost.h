#ifndef ASSIST_EXTENDED_BOOST_H_
#define ASSIST_EXTENDED_BOOST_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * FW-100: Extended Boost — a deliberate drive hold AFTER the cranks stop.
 *
 * The rider arms it with a firm push. When pedalling then stops, the motor keeps pulling for a
 * configured time at the current the rider was ALREADY GETTING, and only then does the ordinary
 * release ramp take over. It exists for steps, rocks and short breaks in pedalling on a
 * technical climb.
 *
 *   Arming    pedal load >= the configured trigger, held for EXT_BOOST_CONFIRM_MS, while
 *             pedaling_active and the ride latch are both true and the bike is moving. The
 *             trigger ONLY ARMS — it has no influence on how much current the boost gives.
 *             An arming goes stale after EXT_BOOST_ARM_TIMEOUT_MS so a hard push cannot be
 *             replayed many seconds later.
 *   Start     the EDGE of pedalling stopping. Never a level, never a button, never a wheel or
 *             motor Hall pulse, never dTorque/dt on its own.
 *   Current   the LAST NORMAL, LIMITED Iq REQUEST while pedalling was active, scaled by
 *             strength_pct. Not an average, not the trigger, not a peak the module waited for
 *             — simply "keep giving me what I had a moment ago". The level's own ceiling and
 *             every shared limit still apply afterwards.
 *   Ends      timer, pedalling resumed, or any cancel: brake/hard cut, crank reverse, sensor
 *             fault, walk, calibration, assist level 0, level or bank change, a bank write, or
 *             the bike no longer moving.
 *
 * Resuming pedalling ends the boost IMMEDIATELY as a state, but not as a torque step: the
 * target simply reverts to what the assist mode asks for and assist_dynamics carries the
 * current there smoothly. Deliberately NOT max(boost, normal) — that would lay two sources of
 * torque on top of each other.
 *
 * ==========================================================================================
 * ACCEPTED RISK — READ BEFORE CHANGING ANYTHING HERE
 *
 * This feature drives the motor while the cranks are stationary, on a bike with no independent
 * brake-sensor input.
 *
 * WHAT STOPS A RUNNING BOOST. An earlier version of this block said "only the brake and the
 * timer", which understated it badly — in a description someone uses to judge the risk. The
 * full list, every entry tested each 4 kHz tick at the top of the cancel chain:
 *
 *   rider action    brake · BACKWARD PEDALLING · resuming pedalling · leaving the assist level
 *   bike state      coming to a stop (below EXT_BOOST_MIN_SPEED_X100 or _MOTOR_ERPS) · level 0
 *   fault / service critical overtemperature · torque-sensor fault · torque or PAS sensor
 *                   invalid · Walk Assist · position calibration · load calibration ·
 *                   a bank being written
 *   time            the duration timer
 *
 * Backward pedalling is covered twice over and independently of the brake: directly through
 * crank_reverse (the legacy, slower-confirming backpedal latch - now pas_direction.c's
 * derivation, unchanged timing), and again through safety_cut, which ride_control.c assembles
 * as hard_cut from rider_input_t.direction_inhibit_active (the immediate, FIRST-step direction
 * safety automaton - FW-109) OR'd with the non-direction cuts - a stricter, faster-reacting
 * signal than crank_reverse alone, not the same one twice. See the ordering note in the cancel
 * chain.
 *
 * That list does not make the feature safe by itself — it means the hazard is "the motor
 * drives while the rider is doing nothing", not "nothing can stop it".
 *
 * FW-095 had removed exactly this behaviour, on an instruction that turned out to describe the
 * hazard rather than forbid the feature. The owner has since asked for it back, knowing the
 * above. That is his decision about his own bike, and it is recorded here so nobody
 * "corrects" it later by reading only the safety argument.
 *
 * The same applies to the speed classification: an active boost is judged PEDAL_CONFIRMED, so
 * in legal mode it follows the 25 km/h pedalling limit rather than the 5-7 km/h no-pedalling
 * taper. Under EPAC, assist without pedalling belongs in the second category — this is an
 * owner decision that goes beyond it, not an oversight in the firmware. See ride_control.c.
 * ==========================================================================================
 */

/* 4 kHz control loop, same as everywhere else in the ride core. */
#define EXT_BOOST_CONTROL_TICKS_PER_MS 4U
/* The load must be HELD this long above the threshold. A shorter spike is a single ADC
 * glitch, a chain slap or a pothole — never a rider decision. */
#define EXT_BOOST_CONFIRM_MS 30U
/* Ends the current qualifying window. Hysteresis only stabilizes the END of a push; it never
 * moves the threshold the rider configured. */
#define EXT_BOOST_RELEASE_HYST_CENTIKG 50U /* 0.50 kg */
/* An arming goes stale after this long without a qualifying load, so a very hard crank turn
 * cannot be replayed many seconds later as a boost the rider has forgotten about. */
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
 * A trigger AT full scale is allowed and simply never arms, because the load can never exceed
 * it. That is a usable "off" for one level without touching the duration.
 */
#define ASSIST_EXT_BOOST_TRIGGER_MIN_CENTIKG 100U  /* 1.0 kg */
#define ASSIST_EXT_BOOST_TRIGGER_MAX_CENTIKG 6000U /* 60.0 kg = TORQUE_PUBLIC_FULL_SCALE */
#define ASSIST_EXT_BOOST_TRIGGER_WIRE_STEP_CENTIKG 50U /* 0.5 kg per wire unit */
/* 20.0 kg: a shove the rider has to mean. FW-100 — the threshold ONLY ARMS the function and has
 * no influence whatsoever on how much current the boost gives, so a high default costs nothing
 * except that ordinary pedalling does not arm it. */
#define ASSIST_EXT_BOOST_TRIGGER_DEFAULT_CENTIKG 2000U /* 20.0 kg */
/* Percent of the last normal limited Iq request. 100 = exactly what the rider already had. */
#define ASSIST_EXT_BOOST_STRENGTH_DEFAULT_PCT 100U
/*
 * Real milliseconds, not a percentage of anything — the removed Legacy overrun expressed its
 * duration as a percentage of Override_Duration, which is why nobody could say what a value
 * meant.
 *
 * FW-100: 2000 ms, an owner decision. This is the one number that bounds how long the motor
 * may drive with the cranks stationary, so it IS the risk ceiling — see the accepted-risk block
 * at the top of this header. It fits the existing put_u16 wire field, so raising it changes no
 * stored bank and invalidates no saved profile.
 *
 * Three places clamp against this constant and all of them follow it automatically: the module
 * itself, and the bank serialize/deserialize in assist_modes.c. The app carries its own copy of
 * the limit in profiles.js — that one has to be changed by hand.
 */
#define ASSIST_EXT_BOOST_DURATION_MAX_MS 2000U

typedef enum {
	ASSIST_EXT_BOOST_IDLE = 0,
	ASSIST_EXT_BOOST_QUALIFY,  /* a push is building towards confirmation */
	ASSIST_EXT_BOOST_ARMED,    /* confirmed; waiting for the cranks to stop */
	ASSIST_EXT_BOOST_ACTIVE    /* holding drive with the cranks stationary */
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
	/* Pedalling came back: the rider is driving again, so the boost hands over at once. */
	ASSIST_EXT_BOOST_CANCEL_PEDALING_RESUMED = 9,
	/* A confirmed arming waited too long for the cranks to stop and went stale. */
	ASSIST_EXT_BOOST_CANCEL_ARM_TIMEOUT = 10,
	ASSIST_EXT_BOOST_CANCEL_COMPLETED = 11,
	/* A bank was written while the module held state. The index of the bank and level did
	 * not change, so nothing else would have noticed — but the trigger, strength and
	 * duration under which the arming was made no longer exist. */
	ASSIST_EXT_BOOST_CANCEL_CONFIG_CHANGED = 12,
	/* WIRE VALUE ONLY. FW-095 reported this when pedalling stopped, because there the boost
	 * ran DURING pedalling. Under FW-100 pedalling stopping is what STARTS the boost, so this
	 * can never be reported. The value stays taken so the app's decoder keeps its numbering. */
	ASSIST_EXT_BOOST_CANCEL_PEDALING_STOPPED_RESERVED = 13
} assist_extended_boost_cancel_t;

typedef struct {
	uint16_t trigger_load_centikg;
	uint8_t strength_pct;
	uint16_t duration_ms;
} assist_extended_boost_config_t;

typedef struct {
	/* THE RAW SENSOR STATE, straight from rider_input. The module reads it and never writes it
	 * back, directly or through the caller — the system's idea of "is the rider pedalling"
	 * stays the sensor's. Its FALLING EDGE is what starts the boost. */
	bool pedaling_active;
	/*
	 * FW-100: the last normal, LIMITED Iq request made while pedalling was active — the value
	 * the rider was actually getting a moment ago. Captured by ride_control after the shared
	 * limiter, so the level ceiling, undervoltage, temperature and speed limits are already in
	 * it. This IS the boost current (times strength_pct); the module derives nothing from the
	 * pedal load.
	 */
	int32_t last_pedal_iq;
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
