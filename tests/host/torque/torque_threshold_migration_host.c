/*
 * STORED START-LOAD THRESHOLD MIGRATION INTO THE CONTROL DOMAIN (FW-151, bank v10).
 *
 * A stored configuration must mean the same thing to the motor for as long as it exists. Up to
 * bank v9 it did not: the two start-load thresholds were kilogram values compared against a
 * kilogram reading, so when FW-150 re-measured the sensor's kg table, every stored bank in the
 * field silently changed what it asked the rider to press - 0.70 kg standing went from 17 mV of
 * sensor signal to 5 mV, which is inside the sensor's own rest noise.
 *
 * v10 stores the thresholds in the frozen CONTROL domain instead. This suite proves the
 * migration into it is deterministic and behaviour-preserving:
 *
 *   M1  v10 round trips: serialize -> deserialize -> serialize is byte identical
 *   M2  v7/v8/v9 thresholds migrate to the SAME SENSOR TRIP POINT they were configured at.
 *       Those versions stored kg on the pre-FW-150 characteristic, which is the characteristic
 *       the control domain is frozen to, so the stored number IS the control value and the
 *       migration is the identity. That is the whole reason the domain was seeded from that
 *       curve rather than from the new measurement.
 *   M3  arbitrary rider values migrate, not just the shipped defaults
 *   M4  v1..v6 thresholds (stored as a sensor delta in mV) migrate straight onto the control
 *       characteristic - native -> CLU, never native -> kg -> CLU
 *   M5  V6 vs V9 EQUIVALENCE: a v6 bank and a v9 bank that asked the rider for the same
 *       physical pedal pressure land on equivalent control thresholds
 *   M6  a v10 threshold survives a restart unchanged (the field that matters, not the struct)
 *
 * The CRC is computed here independently of the production one: a round trip that agreed with
 * itself through a shared bug would prove nothing.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "assist_modes.h"
#include "config.h"
#include "torque_input.h"

static int failures;

#define CHECK(cond, what) do { \
	if (!(cond)) { \
		printf("  FAIL  %s\n", (what)); \
		failures++; \
	} \
} while (0)

#define LEVELS 5
#define HEADER_LEN 13
#define RECORD_LEN_V5 35
#define RECORD_LEN_V7 46
#define RECORD_LEN_V8 48

/* Byte offsets of the two thresholds inside a record - protocol, not a local convention. */
#define REC_STANDING_LO 19
#define REC_ROLLING     35

static uint16_t crc16(const uint8_t *buffer, uint16_t length)
{
	uint16_t crc = 0xFFFFU;
	uint16_t i;
	uint8_t bit;

	for (i = 0; i < length; i++) {
		crc ^= (uint16_t)buffer[i] << 8;
		for (bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000U) ?
				(uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

static void wr16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFU);
	p[1] = (uint8_t)(v >> 8);
}

static uint8_t *record_of(uint8_t *blob, uint8_t level, uint8_t stride)
{
	return &blob[HEADER_LEN + (uint16_t)(level - 1U) * stride];
}

/*
 * Build a blob of the requested version carrying the given thresholds in every level, starting
 * from what this firmware itself serializes so every unrelated field is valid.
 */
static uint16_t build_blob(uint8_t *out, uint8_t version, uint8_t stride,
	uint16_t standing_wire, uint8_t rolling_wire)
{
	uint8_t current[ASSIST_BANK_BLOB_LEN];
	uint8_t level;
	uint16_t crc_at;

	assist_modes_init();
	assist_modes_serialize_bank(0, current);

	memset(out, 0, ASSIST_BANK_BLOB_LEN);
	memcpy(out, current, HEADER_LEN);
	out[2] = version;
	out[5] = stride;
	for (level = 1U; level <= LEVELS; level++) {
		uint8_t *src = record_of(current, level, RECORD_LEN_V8);
		uint8_t *dst = record_of(out, level, stride);

		memcpy(dst, src, stride);
		wr16(&dst[REC_STANDING_LO], standing_wire);
		if (stride > REC_ROLLING) {
			dst[REC_ROLLING] = rolling_wire;
		}
	}
	crc_at = (uint16_t)(HEADER_LEN + LEVELS * stride);
	wr16(&out[crc_at], crc16(out, crc_at));
	return (uint16_t)(crc_at + 2U);
}

/* The first native delta at which a stored control threshold is reached: the SENSOR TRIP
 * POINT, which is the only thing the rider can actually feel. */
static uint16_t trip_point(uint16_t threshold_ctrl)
{
	uint32_t d;

	for (d = 0U; d <= TORQUE_SPAN_MAX_NATIVE; d++) {
		if (torque_input_native_delta_to_ctrl((uint16_t)d) >= threshold_ctrl) {
			return (uint16_t)d;
		}
	}
	return (uint16_t)TORQUE_SPAN_MAX_NATIVE;
}

/* ---- M1: v10 round trip ------------------------------------------------------------------ */

static void test_v10_round_trip(void)
{
	uint8_t first[ASSIST_BANK_BLOB_LEN];
	uint8_t second[ASSIST_BANK_BLOB_LEN];

	printf("M1 a v10 bank round trips byte for byte\n");

	assist_modes_init();
	CHECK(assist_modes_serialize_bank(0, first) == ASSIST_BANK_BLOB_LEN,
		"M1: the shipped bank serializes to a full blob");
	CHECK(first[2] == 10U, "M1: this firmware writes bank version 10");
	CHECK(crc16(first, ASSIST_BANK_BLOB_LEN - 2U) ==
		(uint16_t)(first[ASSIST_BANK_BLOB_LEN - 2] |
			((uint16_t)first[ASSIST_BANK_BLOB_LEN - 1] << 8)),
		"M1: the blob CRC matches an independent computation");

	CHECK(assist_modes_apply_bank_blob(first, ASSIST_BANK_BLOB_LEN),
		"M1: a v10 blob is accepted");
	CHECK(assist_modes_serialize_bank(0, second) == ASSIST_BANK_BLOB_LEN,
		"M1: it serializes again");
	CHECK(memcmp(first, second, ASSIST_BANK_BLOB_LEN) == 0,
		"M1: serialize -> deserialize -> serialize is byte identical");
}

/* ---- M2 / M3: v7..v9 migrate to the identity, preserving the trip point ------------------ */

static void check_stored(uint8_t version, uint16_t standing_wire, uint8_t rolling_wire,
	uint16_t expect_standing_ctrl, uint16_t expect_rolling_ctrl, const char *tag)
{
	uint8_t blob[ASSIST_BANK_BLOB_LEN];
	uint16_t len;
	const assist_level_config_t *level;
	char msg[160];

	/* v7 is validated against its own 46 B stride; v8/v9/v10 against 48 B. Getting this
	 * wrong makes the blob be refused on its LENGTH, which would prove nothing about the
	 * threshold migration. */
	len = build_blob(blob, version,
		(version == 7U) ? RECORD_LEN_V7 : RECORD_LEN_V8,
		standing_wire, rolling_wire);
	snprintf(msg, sizeof(msg), "%s: a v%u blob is accepted", tag, (unsigned)version);
	CHECK(assist_modes_apply_bank_blob(blob, len), msg);
	assist_modes_set_active_bank(0);
	level = assist_modes_get_default_level(1U);

	snprintf(msg, sizeof(msg),
		"%s: standing %u -> %u CLU (expected %u), trips at %u mV",
		tag, (unsigned)standing_wire, (unsigned)level->minimum_pedal_load_ctrl,
		(unsigned)expect_standing_ctrl, (unsigned)trip_point(level->minimum_pedal_load_ctrl));
	CHECK(level->minimum_pedal_load_ctrl == expect_standing_ctrl, msg);

	snprintf(msg, sizeof(msg),
		"%s: rolling %u -> %u CLU (expected %u), trips at %u mV",
		tag, (unsigned)rolling_wire, (unsigned)level->riding_start_load_ctrl,
		(unsigned)expect_rolling_ctrl, (unsigned)trip_point(level->riding_start_load_ctrl));
	CHECK(level->riding_start_load_ctrl == expect_rolling_ctrl, msg);
}

static void test_v7_v9_identity(void)
{
	uint8_t version;

	printf("M2 v7/v8/v9 thresholds migrate to the same sensor trip point\n");

	for (version = 7U; version <= 9U; version++) {
		/* The shipped defaults of the bike-verified build: 70 centikg standing (17 mV),
		 * 30 centikg = 3 wire steps rolling (8 mV). */
		check_stored(version, 70U, 3U, 70U, 30U, "M2");
	}

	/* And the trip points are the ones the bike was verified at. */
	CHECK(trip_point(70U) == 17U, "M2: a migrated 70 still trips at 17 mV");
	CHECK(trip_point(30U) == 8U, "M2: a migrated 30 still trips at 8 mV");
}

static void test_arbitrary_rider_values(void)
{
	printf("M3 arbitrary rider-configured thresholds migrate, not just the defaults\n");

	/* Values a rider could have set, spread across the range, all quantized to the wire
	 * step so the round trip is exact. */
	check_stored(9U, 50U, 2U, 50U, 20U, "M3");
	check_stored(9U, 120U, 6U, 120U, 60U, "M3");
	check_stored(9U, 250U, 11U, 250U, 110U, "M3");
	check_stored(9U, 600U, 30U, 600U, 300U, "M3");
	check_stored(9U, 1500U, 100U, 1500U, 1000U, "M3");

	/* The documented maximum must survive as itself, not be clipped to something else. */
	check_stored(9U, ASSIST_MIN_PEDAL_LOAD_MAX_CTRL,
		(uint8_t)(ASSIST_MIN_PEDAL_LOAD_MAX_CTRL / ASSIST_START_LOAD_WIRE_STEP_CTRL),
		ASSIST_MIN_PEDAL_LOAD_MAX_CTRL, ASSIST_MIN_PEDAL_LOAD_MAX_CTRL, "M3");
}

/* ---- M4 / M5: v1..v6 stored millivolts --------------------------------------------------- */

static void test_v6_native_migration(void)
{
	uint8_t blob[ASSIST_BANK_BLOB_LEN];
	uint16_t len;
	const assist_level_config_t *level;
	char msg[160];
	/* A v6 bank asking for 17 mV standing - the trip point the bike was verified at. */
	const uint16_t v6_mv = 17U;
	const uint8_t v6_reduction_mv = 9U;   /* rolling = 17 - 9 = 8 mV */

	printf("M4 v1..v6 millivolt thresholds migrate straight onto the control characteristic\n");

	len = build_blob(blob, 6U, RECORD_LEN_V7, v6_mv, v6_reduction_mv);
	CHECK(assist_modes_apply_bank_blob(blob, len), "M4: a v6 blob is accepted");
	assist_modes_set_active_bank(0);
	level = assist_modes_get_default_level(1U);

	/* 17 mV reads 70 CLU on the frozen characteristic, quantized to the wire step. */
	snprintf(msg, sizeof(msg), "M4: 17 mV -> %u CLU, trips at %u mV",
		(unsigned)level->minimum_pedal_load_ctrl,
		(unsigned)trip_point(level->minimum_pedal_load_ctrl));
	CHECK(level->minimum_pedal_load_ctrl == 70U, msg);
	CHECK(trip_point(level->minimum_pedal_load_ctrl) == 17U,
		"M4: the migrated v6 standing threshold trips at the mV it was stored as");

	snprintf(msg, sizeof(msg), "M4: 8 mV rolling -> %u CLU, trips at %u mV",
		(unsigned)level->riding_start_load_ctrl,
		(unsigned)trip_point(level->riding_start_load_ctrl));
	CHECK(level->riding_start_load_ctrl == 30U, msg);
	CHECK(trip_point(level->riding_start_load_ctrl) == 8U,
		"M4: the migrated v6 rolling threshold trips at the mV it was stored as");
}

static void test_v6_v9_equivalence(void)
{
	uint8_t blob[ASSIST_BANK_BLOB_LEN];
	uint16_t len;
	uint16_t v6_standing;
	uint16_t v6_rolling;
	uint16_t v9_standing;
	uint16_t v9_rolling;
	char msg[160];

	printf("M5 a v6 and a v9 bank asking for the same pressure land on the same control value\n");

	/* v6 stored 17 mV directly. */
	len = build_blob(blob, 6U, RECORD_LEN_V7, 17U, 9U);
	CHECK(assist_modes_apply_bank_blob(blob, len), "M5: the v6 blob is accepted");
	assist_modes_set_active_bank(0);
	v6_standing = assist_modes_get_default_level(1U)->minimum_pedal_load_ctrl;
	v6_rolling = assist_modes_get_default_level(1U)->riding_start_load_ctrl;

	/* v9 stored the kilogram value that the same 17 mV read as on the pre-FW-150 curve: 70. */
	len = build_blob(blob, 9U, RECORD_LEN_V8, 70U, 3U);
	CHECK(assist_modes_apply_bank_blob(blob, len), "M5: the v9 blob is accepted");
	assist_modes_set_active_bank(0);
	v9_standing = assist_modes_get_default_level(1U)->minimum_pedal_load_ctrl;
	v9_rolling = assist_modes_get_default_level(1U)->riding_start_load_ctrl;

	snprintf(msg, sizeof(msg),
		"M5: standing - v6 gives %u CLU (%u mV), v9 gives %u CLU (%u mV)",
		(unsigned)v6_standing, (unsigned)trip_point(v6_standing),
		(unsigned)v9_standing, (unsigned)trip_point(v9_standing));
	CHECK(v6_standing == v9_standing, msg);
	CHECK(trip_point(v6_standing) == trip_point(v9_standing),
		"M5: and therefore the same sensor trip point");

	snprintf(msg, sizeof(msg),
		"M5: rolling - v6 gives %u CLU (%u mV), v9 gives %u CLU (%u mV)",
		(unsigned)v6_rolling, (unsigned)trip_point(v6_rolling),
		(unsigned)v9_rolling, (unsigned)trip_point(v9_rolling));
	CHECK(v6_rolling == v9_rolling, msg);
}

/* ---- M6: the migrated value survives a restart ------------------------------------------- */

static void test_migration_is_persisted(void)
{
	uint8_t blob[ASSIST_BANK_BLOB_LEN];
	uint8_t after[ASSIST_BANK_BLOB_LEN];
	uint16_t len;
	uint16_t migrated;

	printf("M6 a migrated threshold is what gets written back, and it is v10\n");

	len = build_blob(blob, 9U, RECORD_LEN_V8, 250U, 11U);
	CHECK(assist_modes_apply_bank_blob(blob, len), "M6: the v9 blob is accepted");
	assist_modes_set_active_bank(0);
	migrated = assist_modes_get_default_level(1U)->minimum_pedal_load_ctrl;

	CHECK(assist_modes_serialize_bank(0, after) == ASSIST_BANK_BLOB_LEN,
		"M6: the migrated bank serializes");
	CHECK(after[2] == 10U, "M6: it is written back as v10, not as the version it came from");
	CHECK((uint16_t)(record_of(after, 1U, RECORD_LEN_V8)[REC_STANDING_LO] |
		((uint16_t)record_of(after, 1U, RECORD_LEN_V8)[REC_STANDING_LO + 1] << 8)) == migrated,
		"M6: the stored bytes carry the migrated control value");

	/* A restart: reload what was written and confirm nothing moved a second time. */
	CHECK(assist_modes_apply_bank_blob(after, ASSIST_BANK_BLOB_LEN),
		"M6: the written-back v10 blob reloads");
	assist_modes_set_active_bank(0);
	CHECK(assist_modes_get_default_level(1U)->minimum_pedal_load_ctrl == migrated,
		"M6: migration is ONE-SHOT - reloading does not migrate again");
	CHECK(trip_point(assist_modes_get_default_level(1U)->minimum_pedal_load_ctrl) ==
		trip_point(migrated),
		"M6: and the sensor trip point is unchanged across the restart");
}

int main(void)
{
	printf("Start-load threshold migration into the control domain (bank v10)\n");
	printf("=================================================================\n");

	test_v10_round_trip();
	test_v7_v9_identity();
	test_arbitrary_rider_values();
	test_v6_native_migration();
	test_v6_v9_equivalence();
	test_migration_is_persisted();

	if (failures != 0) {
		printf("\nFAILURES: %d\n", failures);
		return 1;
	}
	printf("\nAll start-load migration checks passed.\n");
	return 0;
}
