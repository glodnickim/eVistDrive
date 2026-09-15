#ifndef ASSIST_BANK_WIRE_H_
#define ASSIST_BANK_WIRE_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * ASSIST BANK WIRE RECORDS.
 *
 * These three records are part of the STORED PROFILE BLOB and of the CAN configuration
 * transport the app and canable-web already speak. The legacy assist modules that used to own
 * them (assist_start.c, assist_extended_boost.c, power_curve.c, cadence_comp.c) were removed
 * with the legacy assist pipeline; the bytes were not, because removing them would reinterpret
 * every profile a rider has already saved and would break the shipped decoders.
 *
 * SO: these are DATA, not behaviour. Assist Pipeline V2 reads only what it declares in
 * assist_modes_profile_override(); everything else here is round-tripped verbatim and read by
 * nothing. Do not wire a new feature onto one of these fields - a reader would have no way to
 * tell a value the rider set for the old meaning from one they set for the new one. A future
 * transport version can reclaim the bytes properly; see inc/assist_modes.h for the 255 B
 * ceiling that makes that a versioned change rather than a free one.
 */

typedef enum {
	ASSIST_STARTUP_BOOST_CADENCE = 0,
	ASSIST_STARTUP_BOOST_SPEED = 1,
	ASSIST_STARTUP_BOOST_AUTO = 2
} assist_startup_boost_mode_t;

/* INACTIVE: stored and round-tripped, read by no control path. */
typedef struct {
	bool enabled;
	assist_startup_boost_mode_t mode;
	uint16_t strength_pct;
	uint8_t end_rpm;
} assist_startup_boost_config_t;

/*
 * PARTLY ACTIVE: duration_ms is read by the pipeline as the START segment length override
 * (assist_modes_profile_override). `enabled` is round-tripped only - the new start segment is
 * always present, because a start with no defined rate is not a feature anyone wants off.
 */
typedef struct {
	bool enabled;
	uint16_t duration_ms;
} assist_smooth_start_config_t;

/* INACTIVE: stored and round-tripped, read by no control path. Extended Boost was a second
 * route to Iq that ran with the cranks stopped; Assist Pipeline V2 has exactly one route. */
typedef struct {
	uint16_t trigger_load_centikg;
	uint8_t strength_pct;
	uint16_t duration_ms;
} assist_extended_boost_config_t;

/* Wire bounds and defaults, kept so a stored blob still validates identically. */
/*
 * The trigger load is stored in ONE byte at 0.1 kg per unit, so the wire cannot carry more than
 * 25.5 kg however large the value in RAM is. The maximum used to say 50.0 kg, which no stored
 * blob could ever hold: 5000 / 10 = 500 truncated to a byte would have come back as 24.4 kg.
 * Nothing could reach it - the only sources are a wire byte and the 20.0 kg default - but a
 * bound that cannot be encoded is a trap for whoever sets one next, so it now states the real
 * one. Values already stored are untouched: every one of them is at most 255 wire units.
 */
#define ASSIST_EXT_BOOST_TRIGGER_MIN_CENTIKG 500U
#define ASSIST_EXT_BOOST_TRIGGER_MAX_CENTIKG 2550U
#define ASSIST_EXT_BOOST_TRIGGER_WIRE_STEP_CENTIKG 10U
#define ASSIST_EXT_BOOST_TRIGGER_DEFAULT_CENTIKG 2000U
#define ASSIST_EXT_BOOST_STRENGTH_DEFAULT_PCT 100U
#define ASSIST_EXT_BOOST_DURATION_MAX_MS 3000U

/* The gamma wire values of the removed POWER_CURVE mode. Kept verbatim so a blob written by
 * this firmware and one written before it compare equal, and so a stored value that was
 * valid before still validates now. */
#define POWER_CURVE_EXP_MIN_X10 3U
#define POWER_CURVE_EXP_MAX_X10 25U
#define POWER_CURVE_EXP_DEFAULT_X10 15U

#endif /* ASSIST_BANK_WIRE_H_ */
