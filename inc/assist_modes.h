#ifndef ASSIST_MODES_H_
#define ASSIST_MODES_H_

#include <stdbool.h>
#include <stdint.h>

#include "ap2_profiles.h"
#include "assist_bank_wire.h"

typedef enum {
	/* FW-094: wire value 0 used to select the pre-ride-core assist path per level. That path is
	 * gone and 0 was never a supported ride-core mode, so it is now simply a reserved value:
	 * a bank carrying it produces no assist. It cannot be reused for a new mode — stored banks
	 * and older app builds still write it. */
	ASSIST_MODE_RESERVED_0 = 0,
	/*
	 * WIRE VALUES 1..6 ARE HISTORY. The calculation behind each of them was removed with the
	 * legacy assist pipeline, but the NUMBERS live in every stored bank and in every shipped
	 * app build, so they cannot be reused or renumbered. A level carrying one of them is
	 * MIGRATED to the closest V2 profile on read - see assist_modes_profile_for_level().
	 */
	ASSIST_MODE_POWER_LINEAR = 1,
	ASSIST_MODE_POWER_PROGRESSIVE = 2,
	ASSIST_MODE_EMTB = 3,
	ASSIST_MODE_EMTB_CUSTOM = 4,
	ASSIST_MODE_TORQUE = 5,
	ASSIST_MODE_POWER_CURVE = 6,
	/*
	 * ASSIST PIPELINE V2 PROFILES. New wire values, so a controller running V2 and an app that
	 * knows about V2 can name the same profile, while a stored bank written before V2 still
	 * loads and migrates. They extend the existing byte - nothing about the 255 B blob layout
	 * changes, which is why the whole configuration transport is untouched.
	 */
	ASSIST_MODE_V2_ECO = 7,
	ASSIST_MODE_V2_TRAIL = 8,
	ASSIST_MODE_V2_SPORT = 9,
	ASSIST_MODE_V2_SPORT_PLUS = 10,
	ASSIST_MODE_V2_AUTO = 11,
	ASSIST_MODE_V2_AUTO_SPORT_PLUS = 12
} assist_mode_type_t;

/*
 * WIRE VALUES — byte 0 of every level record in the bank blob, and the mode_type enum in
 * protocol/evistdrive_config_schema.yaml. A stored bank carries the NUMBER, not the name, so
 * renumbering any of these silently reinterprets every profile the rider already saved:
 * a bank written as eMTB would come back as something else after a firmware update, with
 * no error anywhere. Renaming a member is free; changing its value is not.
 *
 * FW-073 renamed the eMTB member. These assertions exist so the next rename cannot quietly
 * take the value with it.
 */
_Static_assert(ASSIST_MODE_RESERVED_0 == 0, "wire value must stay 0");
_Static_assert(ASSIST_MODE_POWER_LINEAR == 1, "wire value must stay 1");
_Static_assert(ASSIST_MODE_POWER_PROGRESSIVE == 2, "wire value must stay 2");
_Static_assert(ASSIST_MODE_EMTB == 3, "eMTB wire value must stay 3");
_Static_assert(ASSIST_MODE_EMTB_CUSTOM == 4, "wire value must stay 4");
_Static_assert(ASSIST_MODE_TORQUE == 5, "wire value must stay 5");
_Static_assert(ASSIST_MODE_POWER_CURVE == 6, "wire value must stay 6");
_Static_assert(ASSIST_MODE_V2_ECO == 7, "V2 profile wire value must stay 7");
_Static_assert(ASSIST_MODE_V2_TRAIL == 8, "V2 profile wire value must stay 8");
_Static_assert(ASSIST_MODE_V2_SPORT == 9, "V2 profile wire value must stay 9");
_Static_assert(ASSIST_MODE_V2_SPORT_PLUS == 10, "V2 profile wire value must stay 10");
_Static_assert(ASSIST_MODE_V2_AUTO == 11, "V2 profile wire value must stay 11");
_Static_assert(ASSIST_MODE_V2_AUTO_SPORT_PLUS == 12, "V2 profile wire value must stay 12");

typedef struct {
	assist_mode_type_t mode_type;
	uint16_t support_ratio_pct;
	uint16_t support_min_pct;
	uint16_t support_max_pct;
	uint16_t reference_power_w;
	uint8_t progression_pct;
	/* FW-056: gamma x10 (3..25) for ASSIST_MODE_POWER_CURVE, one per half of the
	 * support window: _x10 shapes support_min -> window middle, _high_x10 shapes
	 * window middle -> support_max. On the wire they ride in bytes that belong to
	 * other modes and are dead in POWER_CURVE (byte 9 = progression_pct, byte 1 =
	 * low half of support_ratio_pct), so the record stays 35 B and the blob 189 B. */
	uint8_t curve_exponent_x10;
	uint8_t curve_exponent_high_x10;
	uint8_t emtb_parameter;
	bool emtb_based_on_power;
	uint16_t emtb_reference_voltage_mv;
	uint8_t torque_assist_factor;
	uint16_t max_motor_power_w;
	uint8_t max_iq_pct;
	bool assist_without_rotation;
	/*
	 * FW-151: the stored start-load thresholds live in the CONTROL domain (CLU), not in
	 * kilograms. The rider still enters them in 0.1 kg; the conversion happens ONCE, at the
	 * configuration boundary (torque_input_centikg_to_ctrl), and what is stored, compared and
	 * migrated is the control value. That is what makes re-measuring the sensor's kilogram
	 * table a display-only change: a stored threshold keeps its sensor trip point, and only
	 * the number shown next to it moves.
	 *
	 * This threshold is shared by a normal standstill start and the optional
	 * assist-without-rotation path. Native mV stays confined to torque_input.c.
	 */
	uint16_t minimum_pedal_load_ctrl;
	assist_startup_boost_config_t startup_boost;
	assist_smooth_start_config_t smooth_start;
	uint16_t release_ms;
	/*
	 * FW-129B: INACTIVE. Still stored, still round-tripped, no longer read by control.
	 *
	 * These filtered the requested motor POWER. While that power was only a ceiling they were
	 * harmless; once FW-129 made it the request itself, a lag in the target path proved to be
	 * two defects at once - falling, it asked for up to 5.6x what the rider's own input was
	 * worth, for seconds after a release; rising, it delivered 43 of 228 counts 75 ms after
	 * the rider pressed again, a second soft-start in front of the Iq ramp. Neither is fixable
	 * while a lag sits on the target: see the block comment in finish_power_request().
	 *
	 * The two jobs they were meant to do are owned elsewhere, explicitly and in better
	 * domains: dead-spot bridging by the RUN estimator over CRANK ANGLE (FW-085/112.4), and
	 * current slew by the four iq_rise/iq_fall ramps below. They keep their bytes because the
	 * bank record
	 * is at its hard 255 B ceiling and moving fields would reinterpret every stored profile.
	 */
	uint16_t power_rise_filter_ms;
	uint16_t power_fall_filter_ms;
	/* FW-077: direct minimum while already rolling, not an mV reduction the
	 * rider has to subtract mentally. FW-151: control domain, see above. */
	uint16_t riding_start_load_ctrl;
	/* FW-069: Iq ramps, moved here from the global tuning blob. They decide the character
	 * of how power builds, so they belong next to release_ms/power_*_filter_ms, which were
	 * already per level. Per level in a per-bank store also gives per-bank for free. */
	uint16_t iq_rise_slow_ms;
	uint16_t iq_rise_fast_ms;
	uint16_t iq_fall_slow_ms;
	uint16_t iq_fall_fast_ms;
	/* FW-084: Extended Boost, per level like the ramps above. duration_ms = 0 disables
	 * it completely, which is what every new and every migrated profile gets. */
	assist_extended_boost_config_t extended_boost;
} assist_level_config_t;

#define ASSIST_BANK_COUNT 2U
/*
 * FW-084: 13 B header + 5x48 B + CRC = 255 B.
 *
 * HARD CEILING, NOW REACHED: the multiframe write protocol carries the total length in ONE
 * byte (CAN_Display.c rx_data_length, taken from rx_data[0]), and send_multiframe() takes a
 * uint8_t length. A blob above 255 B cannot be transferred at all - the write would fail
 * on CRC with no useful error. v6 therefore placed its three added fields in u8 slots.
 * FW-077 reuses [35] for the rolling kg threshold and reserved the removed [36..37];
 * FW-084 spends those two reserved bytes plus two new ones, which is the last room there is.
 * THE NEXT per-level field cannot simply grow the record: it needs an existing byte reused,
 * a different packing, or a transport version carrying the length as u16.
 */
#define ASSIST_BANK_BLOB_LEN 255U
_Static_assert(ASSIST_BANK_BLOB_LEN == 255U,
	"FW-084 bank blob is exactly 255 B: 13 B header + 5x48 B record + 2 B CRC");
_Static_assert(ASSIST_BANK_BLOB_LEN <= 255U,
	"bank blob length must fit the single length byte of the multiframe protocol");
/* FW-077/151 per-level start-load limits in the CONTROL domain (CLU). Every configured value
 * is rounded to ASSIST_START_LOAD_WIRE_STEP_CTRL, which is what gives the rider-facing
 * precision of one decimal place in kg. */
#define ASSIST_START_LOAD_WIRE_STEP_CTRL 10U
/*
 * FW-151: THE BOOT DEFAULTS, in the frozen control domain.
 *
 * 70 / 30 CLU are the values the bike was last verified on (1d6c6ba), and in CLU they keep
 * their sensor trip points for good: 17 mV standing, 8 mV rolling.
 *
 * The history is worth keeping, because it is the whole reason this domain exists. These
 * numbers were 70/30 CENTIKG in the bike-verified build, where the then-current kg table put
 * them at 17/8 mV. FW-150 re-measured that table and the SAME stored numbers became 5/2 mV -
 * inside the sensor's own rest noise (TQ_RECAL_STABLE_MV = 10), so assist could be permitted by
 * noise. An intermediate build raised them to 250/110 centikg, which did restore 17/8 mV but
 * only by chasing the table; the next build "restored the baseline values" 70/30 and silently
 * reintroduced the 5/2 mV gates. Stored in CLU, the gate is 17/8 mV regardless of what any
 * future kg measurement says.
 *
 * Do not "convert these to kg to make them readable". The kg equivalent is a DISPLAY, computed
 * on the current table by torque_input_ctrl_to_centikg().
 */
#define ASSIST_MIN_PEDAL_LOAD_DEFAULT_CTRL 70U
/* Boot default for the "while riding" threshold only — deliberately lower than the
 * standstill threshold above, so assist stays on through lighter pedalling once you are
 * already moving, without lowering the guard against an accidental start from a stop. */
#define ASSIST_RIDING_MIN_PEDAL_LOAD_DEFAULT_CTRL 30U
#define ASSIST_MIN_PEDAL_LOAD_MAX_CTRL 2250U
/* FW-069 per-level Iq ramp limits (same range the global tuning blob used). */
#define ASSIST_RAMP_MS_MIN 20U
#define ASSIST_RAMP_MS_MAX 5000U

const assist_level_config_t *assist_modes_get_default_level(uint8_t level_index);

void assist_modes_init(void);
void assist_modes_set_active_bank(uint8_t bank_index);
uint8_t assist_modes_get_active_bank(void);

/* FW-043: Walk Assist cut-off wheel speed of the active bank, in 0.01 km/h units
 * (same scale as MS.Speedx100). Configurable per bank from Canable. */
uint16_t assist_modes_get_wa_max_wheel_x100(void);
/* FW-060 compatibility field: still serialized for older Canable, ignored by WA control. */
uint8_t assist_modes_get_wa_current_pct(void);
uint8_t assist_modes_get_wa_target_rpm(void);
bool assist_modes_get_wa_latch_after_release(void);
uint8_t assist_modes_get_wa_latch_timeout_s(void);
void assist_modes_seed_wa_defaults(uint8_t current_pct, uint16_t target_rpm);

/* FW-057: cadence compensation on/off for the active bank. */
bool assist_modes_get_cadence_comp_enabled(void);

/*
 * THE BRIDGE FROM STORED CONFIGURATION TO THE PIPELINE.
 *
 * A level record says which PROFILE the rider chose and, optionally, overrides a few of that
 * profile's numbers. Everything else in the record is wire ballast (see inc/assist_bank_wire.h)
 * and is read by nothing.
 *
 * Wire values 1..6 name assist modes this firmware no longer implements. They are migrated to
 * the nearest V2 profile rather than rejected, so a rider who updates does not lose their
 * levels - a level saved as "Power progressive" comes back as SPORT, not as no assist.
 */
ap2_profile_id_t assist_modes_profile_for_level(const assist_level_config_t *config);

/*
 * THE LEVEL'S OWN PHASE-CURRENT CEILING, resolved against the ceiling the rest of the firmware
 * already imposes.
 *
 * `max_iq_pct` is a per-level percentage of the hardware phase-current maximum, stored in the
 * bank and round-tripped over CAN since long before Assist Pipeline V2. It reached no control
 * path at all: the pipeline was handed the GLOBAL ceiling instead, so setting a level to 20 %
 * changed nothing. It is a rider-visible setting and it now binds.
 *
 *   pct == 0    the level is SWITCHED OFF. Zero allowed current.
 *   pct 1..99   a ceiling: that percentage of the phase-current maximum.
 *   pct == 100  the phase-current maximum, i.e. no ceiling of the level's own.
 *
 * Zero used to be read here as "no level ceiling", on the reasoning that an uninitialised
 * record carries 0 while every shipped default is 100. The second half of that is exactly why
 * it does not hold: a stored 0 cannot have come from a default, so it came from somebody who
 * set it - and the app they set it in has always described it as "Assist is switched off at
 * this level". Two ends of one wire cannot hold two meanings for the same byte, and between a
 * rider's stated intent and a convenience reading, the rider's wins.
 *
 * "No level ceiling at all" is a different statement and does not share this value. It has its
 * own, on the limiter input: AP2_LIMITS_NO_LEVEL_CEILING, for Walk, which has no assist level.
 *
 *   global_limit  the ceiling already in force (limp mode, hardware) - the result never
 *                 exceeds it, so this can only tighten.
 */
int32_t assist_modes_level_iq_limit(const assist_level_config_t *config,
	int32_t global_limit, int32_t phase_current_max);

/*
 * Is this level configured to produce no assist at all?
 *
 * True for a level stored as the reserved mode 0, and for one whose torque ceiling the rider
 * set to zero. Both are the same rider-facing fact - "this level does nothing" - so both take
 * the same route: the pipeline treats the level as OFF, the request is zero at its source, and
 * an existing current is released the ordinary way rather than cut. A ceiling of zero applied
 * only at the end of the limiter chain would produce the same number by a worse road, with the
 * whole demand model still running behind it.
 */
bool assist_modes_level_disables_assist(const assist_level_config_t *config);

/*
 * The per-level overrides the pipeline honours, and the ONLY fields of the level record that
 * reach control. Every one of them is 0 for "use the profile value", so an old stored bank, a
 * zeroed field and a fresh controller all behave identically.
 *
 *   support_ratio_pct  -> assist trim   100 = the profile as designed. Raising it makes the
 *                                       motor reach full assist at a LOWER pedal force: more
 *                                       assist for the same effort.
 *   max_motor_power_w  -> power ceiling a CEILING only - it can tighten the profile envelope,
 *                                       never widen it. Lowering it makes the motor stop
 *                                       pulling harder sooner on a climb, and draw less.
 *   iq_rise_fast_ms    -> attack        lowering it makes the motor answer a change in effort
 *                                       sooner; too low feels twitchy. FIXED profiles only.
 *   release_ms         -> release       lowering it makes the motor let go sooner when the
 *                                       rider eases off; too low feels like it cuts out.
 *                                       FIXED profiles only.
 *   smooth_start.duration_ms -> start   raising it makes the first torque of a ride softer.
 *                                       FIXED profiles only.
 *
 * The three dynamics overrides are ignored for AUTO and AUTO SPORT+ on purpose: choosing an
 * adaptive profile IS choosing to let the pipeline pick the dynamics.
 */
void assist_modes_profile_override(const assist_level_config_t *config,
	ap2_profile_override_t *out);

uint16_t assist_modes_serialize_bank(uint8_t bank_index, uint8_t *buffer);
bool assist_modes_apply_bank_blob(const uint8_t *buffer, uint16_t length);

void assist_modes_reset(void);

#endif /* ASSIST_MODES_H_ */
