#include "tuning_config.h"

#define CONTROL_TICKS_PER_MS 4
#define TUNING_MAGIC0 0x54U /* 'T' */
#define TUNING_MAGIC1 0x55U /* 'U' */
/* FW-068/069: v6 adds start_steps and drops the ramps from this module's ownership.
 * FW-085: v7 keeps v6's layout byte for byte and only changes the UNIT of the field at
 * offset 20, from milliseconds to crank degrees. v2..v6 are still accepted (see the
 * version table in tuning_config_apply_blob). */
/* FW-129: v8 spends two of v6's three reserved u16 (offsets 24 and 26). Length unchanged. */
#define TUNING_VERSION 8U
#define TUNING_VERSION_V6 6U
#define TUNING_VERSION_V7 7U
#define TUNING_VERSION_V8 8U

/*
 * FW-069: the four ramp values are no longer used by anything - assist_dynamics takes them
 * per level from the bank blob. They are kept here purely so the wire format keeps its shape
 * and an older Canable can still write a blob this firmware accepts; nothing reads them back.
 */
static uint16_t rise_slow_ms = 600U;
static uint16_t rise_fast_ms = 300U;
static uint16_t fall_slow_ms = 1000U;
static uint16_t fall_fast_ms = 140U;
static uint16_t cadence_step = 20U;
static uint8_t start_steps = TUNING_START_STEPS_DEFAULT; /* FW-068 */

/* FW-032: ride latch (matches the FW-031 fixed test defaults). */
static uint16_t run_deadband_mv = 5U;
static uint16_t hold_ms = 1400U;
static uint16_t min_iq_pct = 2U;

/* FW-033/085: RUN torque estimator averaging window, in crank degrees. */
static uint16_t torque_run_window_deg = TUNING_TORQUE_RUN_WINDOW_DEG_DEFAULT;

/* FW-129: ride-feel torque axis + crank arm of the rider-power equation (see the header). */
static uint16_t assist_torque_full_scale_ctrl =
	TUNING_ASSIST_TORQUE_FULL_SCALE_CTRL_DEFAULT;
static uint16_t crank_length_mm = TUNING_CRANK_LENGTH_MM_DEFAULT;

static uint16_t clamp_max(uint16_t value, uint16_t max)
{
	return (value > max) ? max : value;
}

static uint16_t clamp_ms(uint16_t value)
{
	if (value < TUNING_RAMP_MS_MIN) {
		return TUNING_RAMP_MS_MIN;
	}
	return (value > TUNING_RAMP_MS_MAX) ? TUNING_RAMP_MS_MAX : value;
}

static uint16_t clamp_cadence_step(uint16_t value)
{
	if (value < TUNING_CADENCE_STEP_MIN) {
		return TUNING_CADENCE_STEP_MIN;
	}
	return (value > TUNING_CADENCE_STEP_MAX) ? TUNING_CADENCE_STEP_MAX : value;
}

uint16_t tuning_config_cadence_step(void)
{
	return cadence_step;
}

uint8_t tuning_config_start_steps(void)
{
	return start_steps;
}

uint16_t tuning_config_run_deadband_mv(void)
{
	return run_deadband_mv;
}

int32_t tuning_config_assist_hold_ticks(void)
{
	return (int32_t)hold_ms * CONTROL_TICKS_PER_MS;
}

uint16_t tuning_config_min_iq_pct(void)
{
	return min_iq_pct;
}

uint16_t tuning_config_assist_torque_run_window_deg(void)
{
	return torque_run_window_deg;
}

uint16_t tuning_config_assist_torque_full_scale_ctrl(void)
{
	return assist_torque_full_scale_ctrl;
}

uint16_t tuning_config_crank_length_mm(void)
{
	return crank_length_mm;
}

/*
 * FW-129: 0 means "written by tooling that does not know this field" - an older Canable
 * negotiates down to v7 and never reaches these bytes, but a v8 blob built by something that
 * left them at their reserved zero must not be read as "0 kg full scale" or "0 mm crank".
 * Both would be division-by-zero or an instantly saturated assist axis.
 */
static uint16_t clamp_or_default(uint16_t value, uint16_t min, uint16_t max,
	uint16_t fallback)
{
	if (value == 0U) {
		return fallback;
	}
	if (value < min) {
		return min;
	}
	return (value > max) ? max : value;
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

static uint16_t tuning_blob_crc16(const uint8_t *buffer, uint16_t length)
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

uint16_t tuning_config_serialize(uint8_t *buffer)
{
	if (buffer == 0) {
		return 0;
	}
	buffer[0] = TUNING_MAGIC0;
	buffer[1] = TUNING_MAGIC1;
	buffer[2] = TUNING_VERSION;
	buffer[3] = 0;
	put_u16(&buffer[4], rise_slow_ms);
	put_u16(&buffer[6], rise_fast_ms);
	put_u16(&buffer[8], fall_slow_ms);
	put_u16(&buffer[10], fall_fast_ms);
	put_u16(&buffer[12], cadence_step);
	put_u16(&buffer[14], run_deadband_mv);
	put_u16(&buffer[16], hold_ms);
	put_u16(&buffer[18], min_iq_pct);
	put_u16(&buffer[20], torque_run_window_deg); /* FW-085: crank degrees, was ms in v6 */
	put_u16(&buffer[22], start_steps); /* FW-068 */
	put_u16(&buffer[24], assist_torque_full_scale_ctrl); /* FW-129/151 */
	put_u16(&buffer[26], crank_length_mm);                  /* FW-129 */
	put_u16(&buffer[28], 0); /* reserve: the LAST spare u16 in this blob length */
	put_u16(&buffer[TUNING_BLOB_LEN - 2U],
		tuning_blob_crc16(buffer, TUNING_BLOB_LEN - 2U));
	return TUNING_BLOB_LEN;
}

bool tuning_config_apply_blob(const uint8_t *buffer, uint16_t length)
{
	/*
	 * FW-033/052/053/068: accept v2 (22 B), v3/v4/v5 (24 B), v6 (32 B). The CRC sits at a
	 * different offset per version, and fields the older version did not have are backfilled
	 * to their defaults, so an older blob keeps the user's settings instead of being rejected.
	 *
	 * The body/length pair MUST be derived per version. Deriving the minimum length from
	 * TUNING_BLOB_LEN alone (as before v6) meant that bumping that constant instantly started
	 * rejecting every still-valid older blob.
	 */
	if (buffer == 0 ||
		length < TUNING_BLOB_LEN_V2 ||
		buffer[0] != TUNING_MAGIC0 ||
		buffer[1] != TUNING_MAGIC1) {
		return false;
	}
	uint8_t version = buffer[2];
	uint16_t body;
	uint16_t min_length;
	if (version == 2U) {
		body = 20U;
		min_length = TUNING_BLOB_LEN_V2;
	} else if (version == 3U || version == 4U || version == 5U) {
		body = 22U;
		min_length = TUNING_BLOB_LEN_V3;
	} else if (version == TUNING_VERSION_V6 || version == TUNING_VERSION_V7 ||
		version == TUNING_VERSION_V8) {
		/* FW-085: v7 is v6's layout with one field reinterpreted, so both share
		 * this geometry exactly; only the meaning of offset 20 differs.
		 * FW-129: v8 is the same geometry again - it only fills two u16 that v6/v7
		 * left reserved, so the body, the CRC position and the length are identical. */
		body = TUNING_BLOB_LEN - 2U;
		min_length = TUNING_BLOB_LEN;
	} else {
		return false;
	}
	if (length < min_length) {
		return false;
	}
	if (get_u16(&buffer[body]) != tuning_blob_crc16(buffer, body)) {
		return false;
	}
	rise_slow_ms = clamp_ms(get_u16(&buffer[4]));
	rise_fast_ms = clamp_ms(get_u16(&buffer[6]));
	fall_slow_ms = clamp_ms(get_u16(&buffer[8]));
	fall_fast_ms = clamp_ms(get_u16(&buffer[10]));
	cadence_step = clamp_cadence_step(get_u16(&buffer[12]));
	run_deadband_mv = clamp_max(get_u16(&buffer[14]), TUNING_RUN_DEADBAND_MV_MAX);
	hold_ms = clamp_max(get_u16(&buffer[16]), TUNING_HOLD_MS_MAX);
	min_iq_pct = clamp_max(get_u16(&buffer[18]), TUNING_MIN_IQ_PCT_MAX);
	/*
	 * FW-053 one-time migrations. These must test against 5, not TUNING_VERSION: they lift the
	 * OLD defaults to the new ones, and a v5 blob already carries the new defaults. Left as
	 * "< TUNING_VERSION" they would fire again on every version bump and quietly overwrite a
	 * value the user had deliberately set back to 700 ms / 4 %.
	 */
	if (version < 5U && hold_ms == 700U) {
		hold_ms = 1400U;
	}
	if (version < 5U && min_iq_pct == 4U) {
		min_iq_pct = 2U;
	}
	/*
	 * FW-085: offset 20 changed UNIT, not position — v6 and older carry milliseconds,
	 * v7 carries crank degrees.
	 *
	 * The old value must NOT be converted. What a given millisecond figure was worth
	 * depended entirely on the cadence the rider tuned it at (that dependence is the
	 * whole reason for this card), so there is no honest conversion. Taking it
	 * literally would be worse than useless: a saved 700 would read as 700 deg and
	 * clamp to 360, silently doubling the smoothing. Only the "off" state carries
	 * over, because 0 means the same thing in both units.
	 */
	/* FW-129: gate on V7, the version that gave offset 20 its CURRENT unit - not on
	 * TUNING_VERSION. Tied to the latter, bumping the version to 8 would have sent every
	 * still-valid v7 blob down the millisecond branch and silently reset a configured
	 * window to the default. Exactly the trap FW-085 documented for start_steps below. */
	if (version >= TUNING_VERSION_V7) {
		torque_run_window_deg = clamp_max(get_u16(&buffer[20]),
			TUNING_TORQUE_RUN_WINDOW_DEG_MAX);
	} else if (version >= 3U) {
		torque_run_window_deg = (get_u16(&buffer[20]) == 0U) ?
			0U : TUNING_TORQUE_RUN_WINDOW_DEG_DEFAULT;
	} else {
		torque_run_window_deg = TUNING_TORQUE_RUN_WINDOW_DEG_DEFAULT; /* v2 backfill */
	}
	/* FW-068: 0 means "written by something that does not know this field" -> keep the
	 * default rather than removing the crank-movement condition altogether.
	 * FW-085: gate on V6, the version that INTRODUCED start_steps — not on
	 * TUNING_VERSION. Tied to the latter, bumping the version to 7 would have made
	 * every v6 blob silently lose its configured start steps. */
	if (version >= TUNING_VERSION_V6) {
		uint16_t steps = get_u16(&buffer[22]);
		if (steps == 0U) {
			start_steps = TUNING_START_STEPS_DEFAULT;
		} else if (steps < TUNING_START_STEPS_MIN) {
			start_steps = TUNING_START_STEPS_MIN;
		} else if (steps > TUNING_START_STEPS_MAX) {
			start_steps = TUNING_START_STEPS_MAX;
		} else {
			start_steps = (uint8_t)steps;
		}
	} else {
		start_steps = TUNING_START_STEPS_DEFAULT;
	}
	/*
	 * FW-129 §24: v8 introduced these two. Every older blob is migrated to the defaults,
	 * which are exactly the values the firmware behaved as before this card (60.0 kg full
	 * scale, 165 mm crank) - so migrating an old profile changes nothing a rider can feel.
	 */
	if (version >= TUNING_VERSION_V8) {
		assist_torque_full_scale_ctrl = clamp_or_default(get_u16(&buffer[24]),
			TUNING_ASSIST_TORQUE_FULL_SCALE_CTRL_MIN,
			TUNING_ASSIST_TORQUE_FULL_SCALE_CTRL_MAX,
			TUNING_ASSIST_TORQUE_FULL_SCALE_CTRL_DEFAULT);
		crank_length_mm = clamp_or_default(get_u16(&buffer[26]),
			TUNING_CRANK_LENGTH_MM_MIN, TUNING_CRANK_LENGTH_MM_MAX,
			TUNING_CRANK_LENGTH_MM_DEFAULT);
	} else {
		assist_torque_full_scale_ctrl =
			TUNING_ASSIST_TORQUE_FULL_SCALE_CTRL_DEFAULT;
		crank_length_mm = TUNING_CRANK_LENGTH_MM_DEFAULT;
	}
	return true;
}
