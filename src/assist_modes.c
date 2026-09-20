#include "assist_modes.h"

#include "config.h"
#include "torque_input.h"
#include "tuning_config.h"

#define ASSIST_LEVEL_COUNT 5
#define ASSIST_MOTOR_POWER_HARD_MAX_W 1500U
#define ASSIST_SUPPORT_RATIO_MAX_PCT 1000U
/*
 * Bounds for the OLD BANK WIRE FORMAT (blob versions v1..v6), which stored start loads as a
 * calibrated sensor delta in mV - v7..v9 used kg, v10 uses the control domain. They are only
 * ever applied while migrating a stored bank on load — nothing to do with the removed Legacy
 * engine.
 */
#define ASSIST_V6_WIRE_MIN_PEDAL_LOAD_MAX_MV 300U
#define ASSIST_V6_WIRE_START_LOAD_REDUCTION_MAX_MV 100U
/*
 * Wire-format validation bounds for fields that are still STORED and round-tripped but no
 * longer read by control (see inc/assist_bank_wire.h). They stay so a blob written before
 * Assist Pipeline V2 still validates, serializes and compares byte for byte.
 */
#define PROGRESSIVE_REFERENCE_POWER_MIN_W 50U
#define PROGRESSIVE_REFERENCE_POWER_MAX_W 500U
#define PROGRESSION_MAX_PCT 100U
#define EMTB_PARAMETER_MAX 250U
#define EMTB_REFERENCE_VOLTAGE_MIN_MV 24000U
#define EMTB_REFERENCE_VOLTAGE_MAX_MV 60000U

/*
 * One shipped level. Everything here that is not mode_type, the two start-load thresholds or
 * the assist trim is WIRE BALLAST: stored, round-tripped, read by no control path (see
 * inc/assist_bank_wire.h). It is written out explicitly rather than left to implicit zero so
 * a blob this firmware serializes stays byte-comparable with one written before V2.
 *
 * The dynamics and the power envelope come from the PROFILE, not from here - that is what a
 * profile IS. A rider who wants different dynamics sets the per-level override; 0 in each of
 * those fields means "the profile decides".
 */
#define DEFAULT_V2_LEVEL(mode) { \
	.mode_type = (mode), \
	.support_ratio_pct = 100, \
	.support_min_pct = 100, \
	.support_max_pct = 100, \
	.reference_power_w = 200, \
	.progression_pct = 0, \
	.curve_exponent_x10 = POWER_CURVE_EXP_DEFAULT_X10, \
	.curve_exponent_high_x10 = POWER_CURVE_EXP_DEFAULT_X10, \
	.emtb_parameter = 100, \
	.emtb_based_on_power = true, \
	.emtb_reference_voltage_mv = 36000, \
	.torque_assist_factor = 80, \
	.max_motor_power_w = 0, \
	.max_iq_pct = 100, \
	.assist_without_rotation = false, \
	.minimum_pedal_load_ctrl = ASSIST_MIN_PEDAL_LOAD_DEFAULT_CTRL, \
	.startup_boost = {false, ASSIST_STARTUP_BOOST_CADENCE, 100, 27}, \
	.smooth_start = {false, 0}, \
	.release_ms = 0, \
	.power_rise_filter_ms = 0, \
	.power_fall_filter_ms = 0, \
	.riding_start_load_ctrl = ASSIST_RIDING_MIN_PEDAL_LOAD_DEFAULT_CTRL, \
	.iq_rise_slow_ms = 0, \
	.iq_rise_fast_ms = 0, \
	.iq_fall_slow_ms = 0, \
	.iq_fall_fast_ms = 0, \
	.extended_boost = { \
		ASSIST_EXT_BOOST_TRIGGER_DEFAULT_CENTIKG, \
		ASSIST_EXT_BOOST_STRENGTH_DEFAULT_PCT, \
		0 \
	} \
}

#define DEFAULT_IDLE_LEVEL { \
	.mode_type = ASSIST_MODE_V2_TRAIL, \
	.reference_power_w = 200, \
	/* Level 0 never assists, and the level INDEX is what decides that - not this field. \
	 * It is written out explicitly because 0 here now means "this level is switched off", \
	 * which would take the throttle down with it; the throttle is not an assist level's to \
	 * switch off. Level 0 is never serialized into a bank, so this never reaches the wire. */ \
	.max_iq_pct = 100, \
	.curve_exponent_x10 = POWER_CURVE_EXP_DEFAULT_X10, \
	.curve_exponent_high_x10 = POWER_CURVE_EXP_DEFAULT_X10, \
	.emtb_based_on_power = true, \
	.emtb_reference_voltage_mv = 36000, \
	.minimum_pedal_load_ctrl = ASSIST_MIN_PEDAL_LOAD_DEFAULT_CTRL, \
	.startup_boost = {false, ASSIST_STARTUP_BOOST_CADENCE, 0, 45}, \
	.smooth_start = {false, 300}, \
	.riding_start_load_ctrl = ASSIST_RIDING_MIN_PEDAL_LOAD_DEFAULT_CTRL, \
	/* FW-069: level 0 never assists, but the shared Iq ramp still runs through it while \
	 * the current fades out after a level change to 0. Zero here would mean "no ramp". */ \
	.iq_rise_slow_ms = 300, \
	.iq_rise_fast_ms = 150, \
	.iq_fall_slow_ms = 500, \
	.iq_fall_fast_ms = 70, \
	.extended_boost = { \
		ASSIST_EXT_BOOST_TRIGGER_DEFAULT_CENTIKG, \
		ASSIST_EXT_BOOST_STRENGTH_DEFAULT_PCT, \
		0 \
	} \
}

/*
 * THE TWO SHIPPED BANKS, in Assist Pipeline V2 profiles.
 *
 * Bank 0 is the FIXED bank: five levels that each behave the same way every time, which is
 * what a rider wants when they are learning the bike or riding something technical.
 * Bank 1 is the ADAPTIVE bank: the middle of the range hands the character to AUTO, with a
 * fixed profile at each end so there is always something predictable to fall back to.
 *
 * The assist TRIM is 100 in every level - the profile as designed. The trim exists so a rider
 * can lean a level up or down without leaving the profile; shipping anything but 100 would
 * mean the compiled default already disagrees with the profile table.
 */
static const assist_level_config_t default_levels[ASSIST_LEVEL_COUNT + 1] = {
	DEFAULT_IDLE_LEVEL,
	/* LEVEL 1 */ DEFAULT_V2_LEVEL(ASSIST_MODE_V2_ECO),
	/* LEVEL 2 */ DEFAULT_V2_LEVEL(ASSIST_MODE_V2_TRAIL),
	/* LEVEL 3 */ DEFAULT_V2_LEVEL(ASSIST_MODE_V2_SPORT),
	/* LEVEL 4 */ DEFAULT_V2_LEVEL(ASSIST_MODE_V2_SPORT_PLUS),
	/* LEVEL 5 */ DEFAULT_V2_LEVEL(ASSIST_MODE_V2_AUTO_SPORT_PLUS)
};

static const assist_level_config_t adaptive_levels[ASSIST_LEVEL_COUNT + 1] = {
	DEFAULT_IDLE_LEVEL,
	/* LEVEL 1 */ DEFAULT_V2_LEVEL(ASSIST_MODE_V2_ECO),
	/* LEVEL 2 */ DEFAULT_V2_LEVEL(ASSIST_MODE_V2_TRAIL),
	/* LEVEL 3 */ DEFAULT_V2_LEVEL(ASSIST_MODE_V2_AUTO),
	/* LEVEL 4 */ DEFAULT_V2_LEVEL(ASSIST_MODE_V2_AUTO_SPORT_PLUS),
	/* LEVEL 5 */ DEFAULT_V2_LEVEL(ASSIST_MODE_V2_SPORT_PLUS)
};

#undef DEFAULT_V2_LEVEL
#undef DEFAULT_IDLE_LEVEL

static const assist_level_config_t *const bank_defaults[ASSIST_BANK_COUNT] = {
	default_levels,
	adaptive_levels
};

static assist_level_config_t bank_config[ASSIST_BANK_COUNT][ASSIST_LEVEL_COUNT + 1];
static uint8_t active_bank;

/*
 * FW-043: per-bank Walk Assist cut-off wheel speed, in units of 0.1 km/h (70 = 7.0 km/h).
 * Rides in the ONE spare byte of the bank blob header (buffer[7], previously always 0), so the
 * blob length, CRC position and EEPROM layout are all unchanged. A stored 0 means "old blob,
 * not set" and maps to the default — old saved banks keep working without a reset.
 */
/* FW-051 supersedes the wire detail above: v2 uses a 10 B header and 187 B. */
#define BANK_WA_MAX_WHEEL_X10_DEFAULT 70U
#define BANK_WA_MAX_WHEEL_X10_MIN     10U   /* 1.0 km/h */
#define BANK_WA_MAX_WHEEL_X10_MAX     255U  /* 25.5 km/h */
/* FW-130: 30 -> 15 -> 25, matching WALK_ASSIST_CURRENT_DEFAULT. The byte drives the WA ceiling
 * (percent of PH_CURRENT_MAX, clamped to WA_MOTOR_IQ_ABS_MAX = 157). 25 % resolves to that hard
 * ceiling, so a fresh bank now ships with the most force this firmware will give a walk. */
#define BANK_WA_CURRENT_DEFAULT       25U
#define BANK_WA_CURRENT_MIN           1U
#define BANK_WA_CURRENT_MAX           100U
#define BANK_WA_TARGET_RPM_DEFAULT    WALK_ASSIST_RPM_DEFAULT
#define BANK_WA_TARGET_RPM_MIN        WALK_ASSIST_RPM_MIN
#define BANK_WA_TARGET_RPM_MAX        WALK_ASSIST_RPM_MAX
#define BANK_WA_LATCH_DEFAULT         0U
#define BANK_WA_LATCH_TIMEOUT_DEFAULT 30U
#define BANK_WA_LATCH_TIMEOUT_MIN     1U
#define BANK_WA_LATCH_TIMEOUT_MAX     120U
static uint8_t bank_wa_max_wheel_x10[ASSIST_BANK_COUNT];
static uint8_t bank_wa_current_pct[ASSIST_BANK_COUNT];
static uint8_t bank_wa_target_rpm[ASSIST_BANK_COUNT];
static uint8_t bank_wa_latch_after_release[ASSIST_BANK_COUNT];
static uint8_t bank_wa_latch_timeout_s[ASSIST_BANK_COUNT];
/* FW-057: cadence compensation on/off, one setting per bank. */
static uint8_t bank_cadence_comp_enabled[ASSIST_BANK_COUNT];
#define BANK_CADENCE_COMP_DEFAULT 0U

#define BANK_BLOB_MAGIC0 0x45U
#define BANK_BLOB_MAGIC1 0x42U
#define BANK_BLOB_VERSION_V1 1U
#define BANK_BLOB_VERSION_V2 2U
#define BANK_BLOB_VERSION_V3 3U
/* FW-056: v4 has the exact same layout and length as v3. The version byte is
 * purely a capability marker so Canable knows this firmware understands
 * ASSIST_MODE_POWER_CURVE and may offer it; old firmware never sees a v4 blob
 * because Canable only sends v4 to a controller that reported v4. */
#define BANK_BLOB_VERSION_V4 4U
/* FW-057: v5 adds header byte 12 = cadence compensation on/off for this bank.
 * 190 B still fits bank_store[2][192], BankBlob[192] and the 24-frame limit. */
#define BANK_BLOB_VERSION_V5 5U
/* FW-068/069: v6 is the first version to GROW THE RECORD (35 -> 46 B). Everything up to v5
 * assumed one compile-time record length, which is why buffer[5] used to be compared against
 * it instead of being used. From here on buffer[5] is the actual stride, so a shorter (older)
 * record is read field by field and the tail is backfilled - growing the record no longer
 * silently discards the user's whole profile configuration. */
#define BANK_BLOB_VERSION_V6 6U
/* FW-077: v7 changes the start-load domain. The record stays 46 B: minimum
 * load is u16 centikg quantized to decikg at [19..20], and rolling minimum is
 * u8 decikg at [35]. The removed rise-detector bytes [36..37] are reserved. */
#define BANK_BLOB_VERSION_V7 7U
/* FW-084: v8 grows the record 46 -> 48 B for Extended Boost. Trigger load (u8 decikg) and
 * strength (u8) take over the two bytes FW-077 left reserved at [36..37]; the duration
 * (u16 LE) is the growth at [46..47]. That puts the blob at exactly 255 B — the ceiling. */
#define BANK_BLOB_VERSION_V8 8U
/*
 * v9 GROWS NOTHING. Header, record and blob are byte for byte the v8 layout; what the number
 * says is that the PROFILES ARE ASSIST PIPELINE V2 - this controller understands mode ids 7..12
 * and every field means what the V2 contract says it means.
 *
 * That is what a version byte is for here, and there is precedent: v4 had v3's exact layout and
 * existed only to tell the app that the controller understood Power Curve. Without it the app
 * cannot tell a V2 controller from a pre-V2 one - both report 8 and both accept 255 B - so it
 * has to guess the generation from the stored mode numbers, which says nothing at all about a
 * V2 controller whose bank was migrated from legacy ids.
 *
 * Older blobs are still accepted and still load: a rider's stored v1..v8 bank is not lost.
 */
#define BANK_BLOB_VERSION_V9 9U
/*
 * FW-151: v10 GROWS NOTHING EITHER - header, record and blob are byte for byte the v8 layout,
 * which they have to be: the blob is already at the hard 255 B ceiling.
 *
 * What v10 says is that THE TWO START-LOAD FIELDS ARE IN THE CONTROL DOMAIN. Bytes [19..20] and
 * [35] carry CLU (still quantized by ASSIST_START_LOAD_WIRE_STEP_CTRL), not 0.01/0.1 kg. That is
 * a change of MEANING with no change of layout, so it needs a version byte the way v4 and v9 did
 * - a reader cannot tell the two apart from the bytes.
 *
 * WHY. Up to v9 the stored threshold was a kilogram value, compared against a kilogram reading.
 * When FW-150 re-measured the sensor's kg table, every stored bank silently changed what it
 * asked the rider to press: 0.70 kg standing went from 17 mV of sensor signal to 5 mV. A stored
 * configuration must mean the same thing to the motor for as long as it exists, so from v10 the
 * control domain is what is stored and kilograms are a display computed on the current table.
 *
 * Older blobs still load - see the migration in assist_modes_deserialize_bank().
 */
#define BANK_BLOB_VERSION_V10 10U
#define BANK_BLOB_VERSION BANK_BLOB_VERSION_V10
#define BANK_BLOB_HEADER_LEN_V1 8U
#define BANK_BLOB_HEADER_LEN_V2 10U
#define BANK_BLOB_HEADER_LEN_V3 12U
#define BANK_BLOB_HEADER_LEN 13U
#define BANK_RECORD_LEN_V5 35U
/* 46 = 35 + 3 (FW-068 start condition, u8 each) + 8 (FW-069 four u16 ramps). */
#define BANK_RECORD_LEN_V7 46U
/* 48 = 46 + 2 (FW-084 Extended Boost duration). See the 255 B ceiling in assist_modes.h. */
#define BANK_RECORD_LEN_V8 48U
#define BANK_RECORD_LEN BANK_RECORD_LEN_V8

_Static_assert(BANK_BLOB_HEADER_LEN + ASSIST_LEVEL_COUNT * BANK_RECORD_LEN + 2U ==
	ASSIST_BANK_BLOB_LEN,
	"bank blob length must match the header/record/CRC layout");

static uint16_t bank_blob_crc16(const uint8_t *buffer, uint16_t length)
{
	uint16_t crc = 0xFFFFU;
	for (uint16_t i = 0; i < length; i++) {
		crc ^= (uint16_t)buffer[i] << 8;
		for (uint8_t bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000U) ?
				(uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

/*
 * WHICH STORED MODE NUMBERS THIS FIRMWARE CAN INTERPRET.
 *
 * The validator exists to refuse a blob this build cannot READ - not one whose numbers it did
 * not itself write. While the five legacy modes were the only numbers in existence those were
 * the same thing. V2 ships banks made of 7..12, so that list refused THIS FIRMWARE'S OWN
 * DEFAULT BANK: serialize the shipped bank, hand it straight back, and the whole bank was
 * rejected - on the CAN apply path and on the boot restore path alike, so a correctly stored
 * configuration came back as compiled defaults after every restart, the Walk parameters that
 * share the bank with it.
 *
 * The policy is stated once, here, and the app mirrors it:
 *
 *   0       reserved. The level commands nothing - assist_modes_level_disables_assist().
 *   1..6    legacy numbers. Accepted and MIGRATED on read by assist_modes_profile_for_level();
 *           the blob keeps the original number, so a downgrade still finds what it wrote.
 *   7..12   the V2 profiles.
 *   >= 13   unknown. The WHOLE bank is refused, before any of it has been applied: the check
 *           below is its own pass over every record, ahead of the pass that mutates state.
 *
 * 4 is accepted for the same reason as the rest of 1..6 - it is a number an older app could
 * already have stored, and refusing it throws away the rider's other four levels with it.
 */
static bool bank_mode_valid(uint8_t mode)
{
	return mode <= (uint8_t)ASSIST_MODE_V2_AUTO_SPORT_PLUS;
}

static uint16_t clamp_u16(uint16_t value, uint16_t min, uint16_t max)
{
	if (value < min) {
		return min;
	}
	return (value > max) ? max : value;
}

static uint8_t valid_curve_exponent_x10(uint8_t value) //FW-056
{
	return (value < POWER_CURVE_EXP_MIN_X10 || value > POWER_CURVE_EXP_MAX_X10) ?
		POWER_CURVE_EXP_DEFAULT_X10 : value;
}

/*
 * A RAMP TIME OF ZERO IS NOT A RAMP TIME - it is "the profile decides" (see the override rules
 * in inc/ap2_profiles.h). Clamping it up to the 20 ms floor is what turned a plain read -> save
 * with no edit into a real change of behaviour: every shipped level stores 0 in all four ramp
 * fields, so one trip through this parser replaced the profile's own attack time with 20 ms on
 * all five of them. A NON-zero value is still held to the floor - that one is a rider's number,
 * and a ramp shorter than the floor is a step, not a ramp.
 */
static uint16_t valid_ramp_ms(uint16_t value)
{
	if (value == 0U) {
		return 0U;
	}
	return clamp_u16(value, ASSIST_RAMP_MS_MIN, ASSIST_RAMP_MS_MAX);
}

static uint8_t valid_wa_current_pct(uint8_t value)
{
	return (value >= BANK_WA_CURRENT_MIN && value <= BANK_WA_CURRENT_MAX) ?
		value : BANK_WA_CURRENT_DEFAULT;
}

static uint8_t valid_wa_target_rpm(uint16_t value)
{
	return (value >= BANK_WA_TARGET_RPM_MIN && value <= BANK_WA_TARGET_RPM_MAX) ?
		(uint8_t)value : BANK_WA_TARGET_RPM_DEFAULT;
}

static uint8_t valid_wa_max_wheel_x10(uint8_t value)
{
	return (value >= BANK_WA_MAX_WHEEL_X10_MIN &&
		value <= BANK_WA_MAX_WHEEL_X10_MAX) ?
		value : BANK_WA_MAX_WHEEL_X10_DEFAULT;
}

static uint8_t valid_wa_latch_timeout_s(uint8_t value)
{
	return (value >= BANK_WA_LATCH_TIMEOUT_MIN &&
		value <= BANK_WA_LATCH_TIMEOUT_MAX) ?
		value : BANK_WA_LATCH_TIMEOUT_DEFAULT;
}

/*
 * WIRE VALUE -> PROFILE.
 *
 * The six legacy mode numbers are migrated rather than rejected: a rider who updates the
 * firmware keeps five working levels instead of five dead ones. The mapping is by CHARACTER,
 * not by name - what the old mode felt like decides which V2 profile it becomes:
 *
 *   POWER_LINEAR      steady proportional support          -> TRAIL
 *   POWER_PROGRESSIVE more support the harder you push     -> SPORT
 *   EMTB              strong, reactive, load-following     -> SPORT
 *   EMTB_CUSTOM       the same, with a custom curve        -> SPORT
 *   TORQUE            direct, unsmoothed torque following  -> TRAIL
 *   POWER_CURVE       shaped, usually set up strong        -> SPORT
 *
 * A stored bank is NOT rewritten on read. The blob keeps its original numbers, so downgrading
 * the firmware restores the old behaviour exactly and the app still sees what the rider saved.
 */
ap2_profile_id_t assist_modes_profile_for_level(const assist_level_config_t *config)
{
	if (config == 0) {
		return AP2_PROFILE_TRAIL;
	}
	switch (config->mode_type) {
	case ASSIST_MODE_V2_ECO:
		return AP2_PROFILE_ECO;
	case ASSIST_MODE_V2_TRAIL:
		return AP2_PROFILE_TRAIL;
	case ASSIST_MODE_V2_SPORT:
		return AP2_PROFILE_SPORT;
	case ASSIST_MODE_V2_SPORT_PLUS:
		return AP2_PROFILE_SPORT_PLUS;
	case ASSIST_MODE_V2_AUTO:
		return AP2_PROFILE_AUTO;
	case ASSIST_MODE_V2_AUTO_SPORT_PLUS:
		return AP2_PROFILE_AUTO_SPORT_PLUS;
	case ASSIST_MODE_POWER_PROGRESSIVE:
	case ASSIST_MODE_EMTB:
	case ASSIST_MODE_EMTB_CUSTOM:
	case ASSIST_MODE_POWER_CURVE:
		return AP2_PROFILE_SPORT;
	case ASSIST_MODE_POWER_LINEAR:
	case ASSIST_MODE_TORQUE:
	case ASSIST_MODE_RESERVED_0:
	default:
		return AP2_PROFILE_TRAIL;
	}
}

bool assist_modes_level_disables_assist(const assist_level_config_t *config)
{
	if (config == 0) {
		return false;
	}
	return config->mode_type == ASSIST_MODE_RESERVED_0 || config->max_iq_pct == 0U;
}

/*
 * ZERO MEANS SWITCHED OFF - the one meaning it has ever had to a rider.
 *
 * The app has always described max_iq_pct = 0 as "Assist is switched off at this level", while
 * this function read it as "no extra limit" and returned the full global ceiling. Both cannot
 * be true, and the rider's is the one that was chosen deliberately: 100 is the compiled default
 * and always has been, so a stored 0 is a setting somebody made on purpose.
 *
 *   0        the level is off. Zero allowed current, and assist_modes_level_disables_assist()
 *            reports the same fact to the pipeline so the level is off for real and not merely
 *            clamped at the end of the chain.
 *   1..99    a ceiling, that percentage of the phase-current limit.
 *   100      the phase-current limit itself, i.e. no ceiling of the level's own.
 *
 * "No level ceiling at all" is a DIFFERENT statement and has its own value on the limiter
 * input, AP2_LIMITS_NO_LEVEL_CEILING - Walk, which has no assist level, is what needs it.
 */
int32_t assist_modes_level_iq_limit(const assist_level_config_t *config,
	int32_t global_limit, int32_t phase_current_max)
{
	int32_t ceiling = global_limit;

	if (phase_current_max < 0) {
		phase_current_max = 0;
	}
	if (ceiling <= 0 || ceiling > phase_current_max) {
		ceiling = phase_current_max;
	}
	if (assist_modes_level_disables_assist(config)) {
		return 0;
	}
	if (config != 0 && config->max_iq_pct < 100U) {
		int32_t level_cap = (phase_current_max * (int32_t)config->max_iq_pct) / 100;
		if (level_cap < ceiling) {
			ceiling = level_cap;
		}
	}
	return ceiling;
}

void assist_modes_profile_override(const assist_level_config_t *config,
	ap2_profile_override_t *out)
{
	if (out == 0) {
		return;
	}
	out->assist_trim_pct = 0U;
	out->max_power_w = 0U;
	out->attack_ms = 0U;
	out->release_ms = 0U;
	out->start_ms = 0U;
	if (config == 0) {
		return;
	}

	/*
	 * A CEILING IS HONOURED WHATEVER THE MODE NUMBER SAYS.
	 *
	 * max_motor_power_w is a limit the rider set on how much the motor may spend. Its meaning
	 * did not change with the pipeline - watts are watts - and a migration that dropped it
	 * would RAISE a stored restriction as a side effect of a firmware update. That is the one
	 * direction a migration must never move a limit, so it is applied before the legacy check
	 * below.
	 */
	out->max_power_w = config->max_motor_power_w;

	/*
	 * Everything else is migrated as a whole: a level carrying a LEGACY mode number gets the
	 * migrated profile's own numbers, not the old mode's. support_ratio_pct meant "percent of
	 * rider power" in a request model this firmware no longer has, so carrying that number
	 * across as an assist trim would silently reinterpret it - the very thing the wire-value
	 * rules exist to prevent. Only a level explicitly saved as a V2 profile may override these.
	 */
	if (config->mode_type < ASSIST_MODE_V2_ECO) {
		return;
	}

	out->assist_trim_pct = config->support_ratio_pct;
	out->attack_ms = config->iq_rise_fast_ms;
	out->release_ms = config->release_ms;
	out->start_ms = config->smooth_start.duration_ms;
}

const assist_level_config_t *assist_modes_get_default_level(uint8_t level_index)
{
	if (level_index > ASSIST_LEVEL_COUNT) {
		level_index = 0;
	}
	return &bank_config[active_bank][level_index];
}

void assist_modes_init(void)
{
	for (uint8_t bank = 0; bank < ASSIST_BANK_COUNT; bank++) {
		bank_wa_max_wheel_x10[bank] = BANK_WA_MAX_WHEEL_X10_DEFAULT; //FW-043
		bank_wa_current_pct[bank] = BANK_WA_CURRENT_DEFAULT;
		bank_wa_target_rpm[bank] = BANK_WA_TARGET_RPM_DEFAULT;
		bank_wa_latch_after_release[bank] = BANK_WA_LATCH_DEFAULT;
		bank_wa_latch_timeout_s[bank] = BANK_WA_LATCH_TIMEOUT_DEFAULT;
		bank_cadence_comp_enabled[bank] = BANK_CADENCE_COMP_DEFAULT; //FW-057
		for (uint8_t level = 0; level <= ASSIST_LEVEL_COUNT; level++) {
			bank_config[bank][level] = bank_defaults[bank][level];
		}
	}
}

void assist_modes_seed_wa_defaults(uint8_t current_pct, uint16_t target_rpm)
{
	uint8_t current = valid_wa_current_pct(current_pct);
	uint8_t rpm = valid_wa_target_rpm(target_rpm);
	for (uint8_t bank = 0; bank < ASSIST_BANK_COUNT; bank++) {
		bank_wa_current_pct[bank] = current;
		bank_wa_target_rpm[bank] = rpm;
	}
}

//FW-043: Walk Assist cut-off wheel speed of the ACTIVE bank, in 0.01 km/h (matches MS.Speedx100).
//Single source of truth: both the pushassist_flag gate in main.c and the walk module read this,
//so the threshold can no longer drift apart between the two places it used to be hard-coded in.
uint16_t assist_modes_get_wa_max_wheel_x100(void)
{
	uint8_t value = valid_wa_max_wheel_x10(bank_wa_max_wheel_x10[active_bank]);
	return (uint16_t)value * 10U;
}

uint8_t assist_modes_get_wa_current_pct(void)
{
	/* FW-130: read again. main.c resolves this percentage of PH_CURRENT_MAX into the Walk
	 * Assist Iq ceiling. Between FW-060 and FW-130 this function had no caller at all, which
	 * is why the Canable slider looked live while the ceiling was a fixed 40 Iq. */
	return valid_wa_current_pct(bank_wa_current_pct[active_bank]);
}

uint8_t assist_modes_get_wa_target_rpm(void)
{
	return valid_wa_target_rpm(bank_wa_target_rpm[active_bank]);
}

bool assist_modes_get_wa_latch_after_release(void)
{
	return bank_wa_latch_after_release[active_bank] != 0U;
}

uint8_t assist_modes_get_wa_latch_timeout_s(void)
{
	return valid_wa_latch_timeout_s(bank_wa_latch_timeout_s[active_bank]);
}

bool assist_modes_get_cadence_comp_enabled(void) //FW-057
{
	return bank_cadence_comp_enabled[active_bank] != 0U;
}

void assist_modes_set_active_bank(uint8_t bank_index)
{
	if (bank_index >= ASSIST_BANK_COUNT) {
		bank_index = 0;
	}
	if (bank_index != active_bank) {
		active_bank = bank_index;
		assist_modes_reset();
	}
}

uint8_t assist_modes_get_active_bank(void)
{
	return active_bank;
}

static void put_u16(uint8_t *buffer, uint16_t value)
{
	buffer[0] = (uint8_t)(value & 0xFFU);
	buffer[1] = (uint8_t)(value >> 8);
}

static uint16_t get_u16(const uint8_t *buffer)
{
	return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
}

/* FW-151: quantization of a stored start-load threshold, in the CONTROL domain. */
static uint16_t round_start_load_ctrl(uint16_t ctrl, uint16_t maximum_ctrl)
{
	if (ctrl > maximum_ctrl) {
		ctrl = maximum_ctrl;
	}
	return (uint16_t)(((ctrl + ASSIST_START_LOAD_WIRE_STEP_CTRL / 2U) /
		ASSIST_START_LOAD_WIRE_STEP_CTRL) *
		ASSIST_START_LOAD_WIRE_STEP_CTRL);
}

static uint8_t ctrl_to_wire_step(uint16_t ctrl, uint16_t maximum_ctrl)
{
	return (uint8_t)(round_start_load_ctrl(ctrl, maximum_ctrl) /
		ASSIST_START_LOAD_WIRE_STEP_CTRL);
}

/*
 * FW-084: the trigger load covers the whole 60 kg sensor scale, quantized to the 0.5 kg
 * step the wire byte carries. Quantizing here as well as on the wire means the value the
 * rider reads back is exactly the value the control loop compares against — an unquantized
 * 8.37 kg in RAM would engage at a threshold the UI never showed.
 */
static uint16_t valid_ext_boost_trigger_centikg(uint16_t centikg)
{
	if (centikg < ASSIST_EXT_BOOST_TRIGGER_MIN_CENTIKG) {
		centikg = ASSIST_EXT_BOOST_TRIGGER_MIN_CENTIKG;
	}
	if (centikg > ASSIST_EXT_BOOST_TRIGGER_MAX_CENTIKG) {
		centikg = ASSIST_EXT_BOOST_TRIGGER_MAX_CENTIKG;
	}
	return (uint16_t)(((centikg + ASSIST_EXT_BOOST_TRIGGER_WIRE_STEP_CENTIKG / 2U) /
		ASSIST_EXT_BOOST_TRIGGER_WIRE_STEP_CENTIKG) *
		ASSIST_EXT_BOOST_TRIGGER_WIRE_STEP_CENTIKG);
}

static uint16_t valid_ext_boost_duration_ms(uint16_t duration_ms)
{
	return (duration_ms > ASSIST_EXT_BOOST_DURATION_MAX_MS) ?
		ASSIST_EXT_BOOST_DURATION_MAX_MS : duration_ms;
}

uint16_t assist_modes_serialize_bank(uint8_t bank_index, uint8_t *buffer)
{
	if (bank_index >= ASSIST_BANK_COUNT || buffer == 0) {
		return 0;
	}
	buffer[0] = BANK_BLOB_MAGIC0;
	buffer[1] = BANK_BLOB_MAGIC1;
	buffer[2] = BANK_BLOB_VERSION;
	buffer[3] = bank_index;
	buffer[4] = ASSIST_LEVEL_COUNT;
	buffer[5] = BANK_RECORD_LEN;
	buffer[6] = active_bank;
	buffer[7] = valid_wa_max_wheel_x10(bank_wa_max_wheel_x10[bank_index]);
	/* FW-130: byte 8 is live again - it is the WA current ceiling percentage. */
	buffer[8] = valid_wa_current_pct(bank_wa_current_pct[bank_index]);
	buffer[9] = valid_wa_target_rpm(bank_wa_target_rpm[bank_index]);
	buffer[10] = bank_wa_latch_after_release[bank_index] ? 1U : 0U;
	buffer[11] = valid_wa_latch_timeout_s(bank_wa_latch_timeout_s[bank_index]);
	buffer[12] = bank_cadence_comp_enabled[bank_index] ? 1U : 0U; //FW-057
	uint8_t *record = &buffer[BANK_BLOB_HEADER_LEN];
	for (uint8_t level = 1; level <= ASSIST_LEVEL_COUNT; level++) {
		const assist_level_config_t *cfg = &bank_config[bank_index][level];
		record[0] = (uint8_t)cfg->mode_type;
		if (cfg->mode_type == ASSIST_MODE_POWER_CURVE) { //FW-056
			/* support_ratio_pct belongs to POWER_LINEAR only, so its two bytes
			 * carry the upper-half exponent here. Byte 2 stays reserved. */
			record[1] = cfg->curve_exponent_high_x10;
			record[2] = 0;
		} else {
			put_u16(&record[1], cfg->support_ratio_pct);
		}
		put_u16(&record[3], cfg->support_min_pct);
		put_u16(&record[5], cfg->support_max_pct);
		put_u16(&record[7], cfg->reference_power_w);
		/* FW-056: byte 9 carries gamma for POWER_CURVE, progression otherwise.
		 * The two shapes belong to different modes and never coexist, so the
		 * record stays 35 B and the blob stays 189 B. */
		record[9] = (cfg->mode_type == ASSIST_MODE_POWER_CURVE) ?
			cfg->curve_exponent_x10 : cfg->progression_pct;
		record[10] = cfg->emtb_parameter;
		record[11] = cfg->emtb_based_on_power ? 1U : 0U;
		put_u16(&record[12], cfg->emtb_reference_voltage_mv);
		record[14] = cfg->torque_assist_factor;
		put_u16(&record[15], cfg->max_motor_power_w);
		record[17] = cfg->max_iq_pct;
		record[18] = cfg->assist_without_rotation ? 1U : 0U;
		put_u16(&record[19], round_start_load_ctrl(
			cfg->minimum_pedal_load_ctrl,
			ASSIST_MIN_PEDAL_LOAD_MAX_CTRL));
		record[21] = cfg->startup_boost.enabled ? 1U : 0U;
		record[22] = (uint8_t)cfg->startup_boost.mode;
		put_u16(&record[23], cfg->startup_boost.strength_pct);
		record[25] = cfg->startup_boost.end_rpm;
		record[26] = cfg->smooth_start.enabled ? 1U : 0U;
		put_u16(&record[27], cfg->smooth_start.duration_ms);
		put_u16(&record[29], cfg->release_ms);
		put_u16(&record[31], cfg->power_rise_filter_ms);
		put_u16(&record[33], cfg->power_fall_filter_ms);
		/* FW-077/151: both start loads use the same control-domain wire step. */
		record[35] = ctrl_to_wire_step(
			cfg->riding_start_load_ctrl,
			ASSIST_MIN_PEDAL_LOAD_MAX_CTRL);
		/* FW-084: the two bytes FW-077 reserved. Only a v8 reader may interpret
		 * them — v6/v7 gave them a different meaning. Byte 36 is 0.1 kg per unit,
		 * the same step as every other kg field here: 50 = 5.0 kg (the floor),
		 * 200 = 20.0 kg (the default), 255 = 25.5 kg, which is all one byte reaches.
		 * The comment here used to claim 0.5 kg per unit and a 60 kg range; the code
		 * never did that, and the configurator believed the comment - so a plain
		 * read -> save turned every stored 20.0 kg trigger into 60.0 kg. */
		record[36] = (uint8_t)(valid_ext_boost_trigger_centikg(
			cfg->extended_boost.trigger_load_centikg) /
			ASSIST_EXT_BOOST_TRIGGER_WIRE_STEP_CENTIKG);
		record[37] = cfg->extended_boost.strength_pct;
		put_u16(&record[38], cfg->iq_rise_slow_ms);          //FW-069
		put_u16(&record[40], cfg->iq_rise_fast_ms);
		put_u16(&record[42], cfg->iq_fall_slow_ms);
		put_u16(&record[44], cfg->iq_fall_fast_ms);
		put_u16(&record[46], valid_ext_boost_duration_ms(   //FW-084
			cfg->extended_boost.duration_ms));
		record += BANK_RECORD_LEN;
	}
	uint16_t crc_at = BANK_BLOB_HEADER_LEN +
		(uint16_t)ASSIST_LEVEL_COUNT * BANK_RECORD_LEN;
	put_u16(&buffer[crc_at], bank_blob_crc16(buffer, crc_at));
	return ASSIST_BANK_BLOB_LEN;
}

bool assist_modes_apply_bank_blob(const uint8_t *buffer, uint16_t length)
{
	if (buffer == 0 ||
		length < BANK_BLOB_HEADER_LEN_V1 ||
		buffer[0] != BANK_BLOB_MAGIC0 ||
		buffer[1] != BANK_BLOB_MAGIC1 ||
		buffer[3] >= ASSIST_BANK_COUNT ||
		buffer[4] != ASSIST_LEVEL_COUNT) {
		return false;
	}
	/*
	 * FW-068/069: buffer[5] is the record STRIDE, not a constant to compare against. The old
	 * equality check meant that the first time the record ever grew, every previously stored
	 * bank was rejected and the whole profile configuration silently fell back to defaults.
	 * A shorter record is now read up to its own length and the tail is backfilled below.
	 */
	uint8_t record_len = buffer[5];
	if (record_len < BANK_RECORD_LEN_V5 || record_len > BANK_RECORD_LEN) {
		return false;
	}

	uint8_t version = buffer[2];
	uint16_t header_len;
	uint16_t expected_len;
	if (version == BANK_BLOB_VERSION_V1) {
		header_len = BANK_BLOB_HEADER_LEN_V1;
	} else if (version == BANK_BLOB_VERSION_V2) {
		header_len = BANK_BLOB_HEADER_LEN_V2;
	} else if (version == BANK_BLOB_VERSION_V3 ||
		version == BANK_BLOB_VERSION_V4) { //FW-056: v4 == v3 layout
		header_len = BANK_BLOB_HEADER_LEN_V3;
	} else if (version == BANK_BLOB_VERSION_V5 ||
		version == BANK_BLOB_VERSION_V6 ||
		version == BANK_BLOB_VERSION_V7 || //FW-077 keeps the v6 header/stride
		version == BANK_BLOB_VERSION_V8 || //FW-084 grows only the record
		version == BANK_BLOB_VERSION_V9 || //v9 == v8 layout, V2 profiles
		version == BANK_BLOB_VERSION_V10) { //FW-151: v10 == v8 layout, control-domain loads
		header_len = BANK_BLOB_HEADER_LEN;
	} else {
		return false;
	}

	if (version == BANK_BLOB_VERSION_V7 && record_len != BANK_RECORD_LEN_V7) {
		return false;
	}
	if ((version == BANK_BLOB_VERSION_V8 || version == BANK_BLOB_VERSION_V9 ||
		version == BANK_BLOB_VERSION_V10) && record_len != BANK_RECORD_LEN_V8) {
		return false;
	}
	expected_len = header_len + (uint16_t)ASSIST_LEVEL_COUNT * record_len + 2U;
	if (length < expected_len) {
		return false;
	}
	uint16_t crc_at = header_len + (uint16_t)ASSIST_LEVEL_COUNT * record_len;
	if (get_u16(&buffer[crc_at]) != bank_blob_crc16(buffer, crc_at)) {
		return false;
	}
	const uint8_t *record = &buffer[header_len];
	for (uint8_t level = 1; level <= ASSIST_LEVEL_COUNT; level++) {
		if (!bank_mode_valid(record[0])) {
			return false;
		}
		record += record_len;
	}
	uint8_t bank_index = buffer[3];
	bank_wa_max_wheel_x10[bank_index] = valid_wa_max_wheel_x10(buffer[7]);
	if (version >= BANK_BLOB_VERSION_V2) {
		bank_wa_current_pct[bank_index] = valid_wa_current_pct(buffer[8]);
		bank_wa_target_rpm[bank_index] = valid_wa_target_rpm(buffer[9]);
	}
	bank_wa_latch_after_release[bank_index] = BANK_WA_LATCH_DEFAULT;
	bank_wa_latch_timeout_s[bank_index] = BANK_WA_LATCH_TIMEOUT_DEFAULT;
	if (version >= BANK_BLOB_VERSION_V3) {
		bank_wa_latch_after_release[bank_index] = buffer[10] ? 1U : 0U;
		bank_wa_latch_timeout_s[bank_index] =
			valid_wa_latch_timeout_s(buffer[11]);
	}
	/* FW-057: older blobs predate the setting, so it stays off after migration. */
	bank_cadence_comp_enabled[bank_index] = (version >= BANK_BLOB_VERSION_V5) ?
		(buffer[12] ? 1U : 0U) : BANK_CADENCE_COMP_DEFAULT;
	record = &buffer[header_len];
	for (uint8_t level = 1; level <= ASSIST_LEVEL_COUNT; level++) {
		assist_level_config_t *cfg = &bank_config[bank_index][level];
		cfg->mode_type = (assist_mode_type_t)record[0];
		if (cfg->mode_type == ASSIST_MODE_POWER_CURVE) { //FW-056
			cfg->support_ratio_pct = 0;
			cfg->curve_exponent_high_x10 = valid_curve_exponent_x10(record[1]);
		} else {
			cfg->support_ratio_pct =
				clamp_u16(get_u16(&record[1]), 0, ASSIST_SUPPORT_RATIO_MAX_PCT);
			cfg->curve_exponent_high_x10 = POWER_CURVE_EXP_DEFAULT_X10;
		}
		cfg->support_min_pct =
			clamp_u16(get_u16(&record[3]), 0, ASSIST_SUPPORT_RATIO_MAX_PCT);
		cfg->support_max_pct =
			clamp_u16(get_u16(&record[5]), 0, ASSIST_SUPPORT_RATIO_MAX_PCT);
		cfg->reference_power_w = clamp_u16(get_u16(&record[7]),
			PROGRESSIVE_REFERENCE_POWER_MIN_W,
			PROGRESSIVE_REFERENCE_POWER_MAX_W);
		if (cfg->mode_type == ASSIST_MODE_POWER_CURVE) { //FW-056
			cfg->progression_pct = 0;
			cfg->curve_exponent_x10 = valid_curve_exponent_x10(record[9]);
		} else {
			cfg->progression_pct = (record[9] > PROGRESSION_MAX_PCT) ?
				PROGRESSION_MAX_PCT : record[9];
			cfg->curve_exponent_x10 = POWER_CURVE_EXP_DEFAULT_X10;
		}
		cfg->emtb_parameter = (record[10] > EMTB_PARAMETER_MAX) ?
			EMTB_PARAMETER_MAX : record[10];
		cfg->emtb_based_on_power = record[11] != 0;
		cfg->emtb_reference_voltage_mv = clamp_u16(get_u16(&record[12]),
			EMTB_REFERENCE_VOLTAGE_MIN_MV, EMTB_REFERENCE_VOLTAGE_MAX_MV);
		cfg->torque_assist_factor = record[14];
		cfg->max_motor_power_w = clamp_u16(get_u16(&record[15]),
			0, ASSIST_MOTOR_POWER_HARD_MAX_W);
		cfg->max_iq_pct = (record[17] > 100U) ? 100U : record[17];
		cfg->assist_without_rotation = record[18] != 0;
		/*
		 * FW-151: START-LOAD MIGRATION INTO THE CONTROL DOMAIN. One pass, on load, no
		 * round trip through kilograms - the point of the exercise is that control does
		 * not depend on the kg table, so the migration must not either.
		 *
		 *   v10      already CLU. Taken as stored.
		 *   v7..v9   stored 0.01 kg on the PRE-FW-150 characteristic, which is the very
		 *            characteristic the control domain is frozen to. The stored number is
		 *            therefore ALREADY the CLU value and the migration is the identity -
		 *            that is not a coincidence, it is why this domain was seeded from that
		 *            curve, and it is what keeps every rider's stored threshold at the
		 *            sensor trip point they configured.
		 *   v1..v6   stored a calibrated sensor delta in mV. Read straight onto the frozen
		 *            control characteristic: native -> CLU, never native -> kg -> CLU.
		 */
		if (version >= BANK_BLOB_VERSION_V7) {
			cfg->minimum_pedal_load_ctrl = round_start_load_ctrl(
				get_u16(&record[19]),
				ASSIST_MIN_PEDAL_LOAD_MAX_CTRL);
		} else {
			uint16_t v6_threshold_mv = clamp_u16(
				get_u16(&record[19]), 0,
				ASSIST_V6_WIRE_MIN_PEDAL_LOAD_MAX_MV);
			cfg->minimum_pedal_load_ctrl = round_start_load_ctrl(
				torque_input_native_delta_to_ctrl(v6_threshold_mv),
				ASSIST_MIN_PEDAL_LOAD_MAX_CTRL);
		}
		cfg->startup_boost.enabled = record[21] != 0;
		cfg->startup_boost.mode = (record[22] > ASSIST_STARTUP_BOOST_AUTO) ?
			ASSIST_STARTUP_BOOST_CADENCE :
			(assist_startup_boost_mode_t)record[22];
		cfg->startup_boost.strength_pct =
			clamp_u16(get_u16(&record[23]), 0, 300U);
		cfg->startup_boost.end_rpm = (record[25] > 120U) ? 120U : record[25];
		cfg->smooth_start.enabled = record[26] != 0;
		cfg->smooth_start.duration_ms =
			clamp_u16(get_u16(&record[27]), 0, 5000U);
		cfg->release_ms = clamp_u16(get_u16(&record[29]), 0, 3000U);
		cfg->power_rise_filter_ms =
			clamp_u16(get_u16(&record[31]), 0, 3000U);
		cfg->power_fall_filter_ms =
			clamp_u16(get_u16(&record[33]), 0, 3000U);
		/*
		 * FW-068/069: fields past the v5 record. A shorter record means an older writer
		 * that never had them, so they take the compiled default of this level instead of
		 * whatever happens to sit past the end of the record.
		 */
		if (record_len >= BANK_RECORD_LEN_V7) {
			/* FW-151: same migration as the standstill threshold above. */
			if (version >= BANK_BLOB_VERSION_V7) {
				cfg->riding_start_load_ctrl = clamp_u16(
					(uint16_t)record[35] * ASSIST_START_LOAD_WIRE_STEP_CTRL,
					0, ASSIST_MIN_PEDAL_LOAD_MAX_CTRL);
			} else {
				/* v6 carried a reduction in mV. Convert it to the direct
				 * rolling threshold used from v7 on; its rise fields are ignored. */
				uint16_t v6_threshold_mv = clamp_u16(
					get_u16(&record[19]), 0,
					ASSIST_V6_WIRE_MIN_PEDAL_LOAD_MAX_MV);
				uint16_t v6_reduction_mv = clamp_u16(record[35], 0,
					ASSIST_V6_WIRE_START_LOAD_REDUCTION_MAX_MV);
				uint16_t rolling_threshold_mv =
					(v6_reduction_mv >= v6_threshold_mv) ? 0U :
					(uint16_t)(v6_threshold_mv - v6_reduction_mv);
				cfg->riding_start_load_ctrl = round_start_load_ctrl(
					torque_input_native_delta_to_ctrl(rolling_threshold_mv),
					ASSIST_MIN_PEDAL_LOAD_MAX_CTRL);
			}
			cfg->iq_rise_slow_ms = valid_ramp_ms(get_u16(&record[38]));
			cfg->iq_rise_fast_ms = valid_ramp_ms(get_u16(&record[40]));
			cfg->iq_fall_slow_ms = valid_ramp_ms(get_u16(&record[42]));
			cfg->iq_fall_fast_ms = valid_ramp_ms(get_u16(&record[44]));
		} else {
			const assist_level_config_t *fallback =
				&bank_defaults[bank_index][level];
			cfg->riding_start_load_ctrl =
				cfg->minimum_pedal_load_ctrl;
			cfg->iq_rise_slow_ms = fallback->iq_rise_slow_ms;
			cfg->iq_rise_fast_ms = fallback->iq_rise_fast_ms;
			cfg->iq_fall_slow_ms = fallback->iq_fall_slow_ms;
			cfg->iq_fall_fast_ms = fallback->iq_fall_fast_ms;
		}
		/*
		 * FW-084: bytes 36..37 are Extended Boost ONLY from v8 on — in v6/v7 they held
		 * a different, since removed meaning, so reading them from an older blob would
		 * turn a stale rise-detector value into a live boost setting.
		 *
		 * Migration is deliberately not "keep what was there": every older profile
		 * comes back with the function OFF, and the rider switches it on knowingly.
		 */
		if (version >= BANK_BLOB_VERSION_V8 && record_len >= BANK_RECORD_LEN_V8) {
			cfg->extended_boost.trigger_load_centikg =
				valid_ext_boost_trigger_centikg((uint16_t)(record[36] *
					ASSIST_EXT_BOOST_TRIGGER_WIRE_STEP_CENTIKG));
			cfg->extended_boost.strength_pct = record[37];
			cfg->extended_boost.duration_ms =
				valid_ext_boost_duration_ms(get_u16(&record[46]));
		} else {
			cfg->extended_boost.trigger_load_centikg =
				ASSIST_EXT_BOOST_TRIGGER_DEFAULT_CENTIKG;
			cfg->extended_boost.strength_pct =
				ASSIST_EXT_BOOST_STRENGTH_DEFAULT_PCT;
			cfg->extended_boost.duration_ms = 0;
		}
		record += record_len;
	}
	assist_modes_reset();
	return true;
}

/*
 * This module carries CONFIGURATION only - no demand, no current, no per-tick state. A
 * lifecycle reset therefore has nothing here to clear, and that is the point: a store with no
 * carried control state cannot leak any into the next ride. The pipeline owns its own reset
 * (assist_pipeline_reset), which ride_control_init() calls.
 */
void assist_modes_reset(void)
{
}

