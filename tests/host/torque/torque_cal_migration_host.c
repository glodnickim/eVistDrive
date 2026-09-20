/*
 * TORQUE CALIBRATION PERSISTENCE MIGRATION TEST
 *
 * Verifies that:
 * - v1 (pre-FW-129) records are rejected with cal_legacy_record_rejected flag
 * - v2 (pre-FW-150, old curve) records are rejected with cal_legacy_record_rejected flag
 * - v3 (FW-150 compatible) records are accepted
 * - new calibrations write v3
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "torque_input.h"

static int failures;

#define CHECK(cond, what) do { \
	if (!(cond)) { \
		printf("  FAIL  %s\n", (what)); \
		failures++; \
	} \
} while (0)

/* Independent CRC-16/CCITT-FALSE matching torque_cal_crc16. */
static uint16_t crc16(uint8_t version, uint16_t span)
{
	uint8_t buffer[3] = { version, (uint8_t)(span & 0xFFU), (uint8_t)(span >> 8) };
	uint16_t crc = 0xFFFFU;
	for (uint8_t i = 0; i < sizeof(buffer); i++) {
		crc ^= (uint16_t)buffer[i] << 8;
		for (uint8_t bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

static void test_v1_rejected(void)
{
	printf("TC1 v1 (pre-FW-129) records are rejected\n");
	torque_input_init();
	
	uint16_t span = TORQUE_DEFAULT_SPAN_NATIVE;
	uint16_t magic = TORQUE_CAL_PERSIST_MAGIC;
	uint16_t crc = crc16(TORQUE_CAL_PERSIST_VERSION_LEGACY, span);
	
	bool ok = torque_input_restore_persist(magic, TORQUE_CAL_PERSIST_VERSION_LEGACY, span, crc);
	CHECK(!ok, "TC1: v1 record is rejected");
	CHECK(torque_input_calibration_source() == TORQUE_CAL_SOURCE_DEFAULT,
		"TC1: calibration source falls back to default");
	CHECK(torque_input_legacy_record_rejected(), "TC1: legacy record rejected flag is set");
}

static void test_v2_rejected(void)
{
	printf("TC2 v2 (pre-FW-150, old curve) records are rejected\n");
	torque_input_init();
	
	uint16_t span = TORQUE_DEFAULT_SPAN_NATIVE;
	uint16_t magic = TORQUE_CAL_PERSIST_MAGIC;
	uint16_t crc = crc16(TORQUE_CAL_PERSIST_VERSION_V2_OLD_CURVE, span);
	
	bool ok = torque_input_restore_persist(magic, TORQUE_CAL_PERSIST_VERSION_V2_OLD_CURVE, span, crc);
	CHECK(!ok, "TC2: v2 record is rejected");
	CHECK(torque_input_calibration_source() == TORQUE_CAL_SOURCE_DEFAULT,
		"TC2: calibration source falls back to default");
	CHECK(torque_input_legacy_record_rejected(), "TC2: legacy record rejected flag is set");
}

static void test_v3_accepted(void)
{
	printf("TC3 v3 (FW-150 compatible) records are accepted\n");
	torque_input_init();
	
	uint16_t span = TORQUE_DEFAULT_SPAN_NATIVE;
	uint16_t magic = TORQUE_CAL_PERSIST_MAGIC;
	uint16_t crc = crc16(TORQUE_CAL_PERSIST_VERSION, span);
	
	bool ok = torque_input_restore_persist(magic, TORQUE_CAL_PERSIST_VERSION, span, crc);
	CHECK(ok, "TC3: v3 record is accepted");
	CHECK(torque_input_calibration_source() == TORQUE_CAL_SOURCE_USER,
		"TC3: calibration source is USER");
	CHECK(!torque_input_legacy_record_rejected(), "TC3: legacy record rejected flag is clear");
	CHECK(torque_input_span_native() == span, "TC3: span is restored correctly");
}

static void test_new_calibration_writes_v3(void)
{
	printf("TC4 new calibration writes v3\n");
	torque_input_init();
	
	/* Simulate a successful calibration */
	torque_input_set_user_span(2500U); /* arbitrary valid span */
	
	uint16_t magic, span, crc_out;
	uint8_t version;
	torque_input_build_persist(&magic, &version, &span, &crc_out);
	
	CHECK(magic == TORQUE_CAL_PERSIST_MAGIC, "TC4: magic is correct");
	CHECK(version == TORQUE_CAL_PERSIST_VERSION, "TC4: version is v3");
	CHECK(span == 2500U, "TC4: span matches what was set");
	CHECK(crc_out == crc16(version, span), "TC4: CRC matches independent computation");
}

static void test_unknown_version_rejected(void)
{
	printf("TC5 unknown versions are rejected\n");
	torque_input_init();
	
	uint16_t span = TORQUE_DEFAULT_SPAN_NATIVE;
	uint16_t magic = TORQUE_CAL_PERSIST_MAGIC;
	uint16_t crc = crc16(99U, span);
	
	bool ok = torque_input_restore_persist(magic, 99U, span, crc);
	CHECK(!ok, "TC5: unknown version is rejected");
	CHECK(torque_input_calibration_source() == TORQUE_CAL_SOURCE_DEFAULT,
		"TC5: calibration source falls back to default");
}

static void test_bad_crc_rejected(void)
{
	printf("TC6 bad CRC is rejected\n");
	torque_input_init();
	
	uint16_t span = TORQUE_DEFAULT_SPAN_NATIVE;
	uint16_t magic = TORQUE_CAL_PERSIST_MAGIC;
	uint16_t crc = crc16(TORQUE_CAL_PERSIST_VERSION, span);
	crc ^= 0xAAAA; /* corrupt */
	
	bool ok = torque_input_restore_persist(magic, TORQUE_CAL_PERSIST_VERSION, span, crc);
	CHECK(!ok, "TC6: corrupted CRC is rejected");
}

static void test_out_of_range_span_rejected(void)
{
	printf("TC7 out-of-range span is rejected\n");
	torque_input_init();
	
	uint16_t magic = TORQUE_CAL_PERSIST_MAGIC;
	uint16_t span = TORQUE_SPAN_MAX_NATIVE + 100U; /* out of range */
	uint16_t crc = crc16(TORQUE_CAL_PERSIST_VERSION, span);
	
	bool ok = torque_input_restore_persist(magic, TORQUE_CAL_PERSIST_VERSION, span, crc);
	CHECK(!ok, "TC7: out-of-range span is rejected");
}

static void test_wrong_magic_rejected(void)
{
	printf("TC8 wrong magic is rejected\n");
	torque_input_init();
	
	uint16_t span = TORQUE_DEFAULT_SPAN_NATIVE;
	uint16_t magic = 0x1234U; /* wrong */
	uint16_t crc = crc16(TORQUE_CAL_PERSIST_VERSION, span);
	
	bool ok = torque_input_restore_persist(magic, TORQUE_CAL_PERSIST_VERSION, span, crc);
	CHECK(!ok, "TC8: wrong magic is rejected");
}

int main(void)
{
	printf("Torque calibration persistence migration test\n");
	printf("============================================\n");
	
	test_v1_rejected();
	test_v2_rejected();
	test_v3_accepted();
	test_new_calibration_writes_v3();
	test_unknown_version_rejected();
	test_bad_crc_rejected();
	test_out_of_range_span_rejected();
	test_wrong_magic_rejected();
	
	if (failures != 0) {
		printf("\nFAILURES: %d\n", failures);
		return 1;
	}
	printf("\nAll torque calibration migration tests passed.\n");
	return 0;
}