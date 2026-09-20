/*
 * TORQUE CALIBRATION PERSISTENCE MIGRATION TEST
 *
 * Verifies that:
 * - v1 (pre-FW-129) records are rejected with cal_legacy_record_rejected flag
 * - v2 (pre-FW-150, old curve) records are rejected with cal_legacy_record_rejected flag
 * - v3 (FW-150 compatible) records are accepted
 * - new calibrations write v3
 * - TC9: a v2 record carrying the span a REAL pre-FW-150 calibration would have written is
 *   rejected on its version, not incidentally on its range. TC2 uses the current default span,
 *   which no v2 writer could ever have produced; if the version check were removed, TC2 would
 *   still pass on some other gate. ~1139 is the pre-FW-150 default span and sits comfortably
 *   inside the accepted range, so only the version byte can reject it.
 * - TC10: the SPAN COMPUTATION overflows uint16_t before it is range-checked, and the real
 *   calibration state machine is what proves it. Testing the helper alone cannot: the overflow
 *   is produced by cal_tick's own arithmetic on a plausible capture, and the narrowing cast
 *   then lands the garbage back inside the accepted window.
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

/* The pre-FW-150 default span: what a genuine v2 record would carry. */
#define V2_REALISTIC_SPAN_NATIVE 1139U

static void test_v2_realistic_span_rejected(void)
{
	printf("TC9 a v2 record with a REALISTIC pre-FW-150 span is rejected on its version\n");
	torque_input_init();

	uint16_t span = V2_REALISTIC_SPAN_NATIVE;
	uint16_t crc = crc16(TORQUE_CAL_PERSIST_VERSION_V2_OLD_CURVE, span);

	/* The premise of the test: this span is NOT rejectable by range. */
	CHECK(span >= TORQUE_SPAN_MIN_NATIVE && span <= TORQUE_SPAN_MAX_NATIVE,
		"TC9: the realistic v2 span is inside the accepted range");

	bool ok = torque_input_restore_persist(TORQUE_CAL_PERSIST_MAGIC,
		TORQUE_CAL_PERSIST_VERSION_V2_OLD_CURVE, span, crc);
	CHECK(!ok, "TC9: realistic v2 record is rejected");
	CHECK(torque_input_calibration_source() == TORQUE_CAL_SOURCE_DEFAULT,
		"TC9: span stays at the default");
	CHECK(torque_input_span_native() == TORQUE_DEFAULT_SPAN_NATIVE,
		"TC9: the stored v2 span did not reach span_native");
	CHECK(torque_input_legacy_record_rejected(), "TC9: legacy record rejected flag is set");
}

/*
 * Drive the REAL calibration state machine through one complete capture.
 * corrected_native is what torque_input_correct() would have returned; the harness feeds it
 * directly, exactly as main.c does.
 */
static void cal_settle(int16_t corrected, unsigned ticks)
{
	for (unsigned i = 0; i < ticks; i++) {
		torque_input_cal_tick(corrected, true);
	}
}

static void test_span_overflow_real_flow(void)
{
	printf("TC10 span overflow is caught by range validation, not hidden by the cast\n");
	torque_input_init();

	/* Publish a plausible resting signal first: cal_start snapshots it as the stability
	 * window's seed, so the zero capture has to start from the value it will average. */
	torque_input_update(TORQUE_ZERO_TARGET_NATIVE, (int16_t)TORQUE_ZERO_TARGET_NATIVE, true);

	torque_input_cal_start();
	CHECK(torque_input_cal_state() == TORQUE_CAL_STATE_CAPTURE_ZERO,
		"TC10: calibration starts in CAPTURE_ZERO");

	/* 256 stable ticks at the zero point -> zero captured, waiting for the reference. */
	cal_settle((int16_t)TORQUE_ZERO_TARGET_NATIVE, 256U);
	CHECK(torque_input_cal_state() == TORQUE_CAL_STATE_WAIT_REFERENCE,
		"TC10: zero capture completes");

	/*
	 * The rider hangs a 5.00 kg reference weight. The DEFAULT sensor would answer that with a
	 * delta of ~71 native; this sensor answers with 1546 - a wildly out-of-spec gain, which is
	 * exactly the case the range check exists for (a miswired or wrong-scale sensor).
	 *
	 *   span = 1546 * 3047 / 71 = 66347, which does not fit uint16_t
	 *   (uint16_t)66347       = 811, which IS inside [800, 4200]
	 *
	 * So the pre-fix code accepts 811 as a calibration and the rider rides on it.
	 */
	/* The weight is on the pedal before the app arms the capture, so the published snapshot
	 * already carries the loaded signal - main.c publishes every tick, cal_capture_load()
	 * seeds its stability window from that snapshot. Arming against a stale unloaded snapshot
	 * would trip the spread check instead, and prove nothing about the span arithmetic. */
	torque_input_update(TORQUE_ZERO_TARGET_NATIVE + 1546,
		(int16_t)(TORQUE_ZERO_TARGET_NATIVE + 1546), true);
	torque_input_cal_capture_load(500U);
	CHECK(torque_input_cal_state() == TORQUE_CAL_STATE_CAPTURE_LOAD,
		"TC10: load capture is armed");

	cal_settle((int16_t)(TORQUE_ZERO_TARGET_NATIVE + 1546), 256U);

	CHECK(torque_input_cal_state() == TORQUE_CAL_STATE_FAILED,
		"TC10: an overflowing span FAILS the calibration");
	CHECK(torque_input_cal_error() == TORQUE_CAL_ERR_SPAN_RANGE,
		"TC10: the reported error is SPAN_RANGE");
	CHECK(torque_input_cal_preview_span() == 0U,
		"TC10: no preview span is offered");
	CHECK(torque_input_calibration_source() == TORQUE_CAL_SOURCE_DEFAULT,
		"TC10: the sensor stays on the default characteristic");
	CHECK(!torque_input_cal_commit(),
		"TC10: a failed calibration cannot be committed");
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
	test_v2_realistic_span_rejected();
	test_span_overflow_real_flow();

	if (failures != 0) {
		printf("\nFAILURES: %d\n", failures);
		return 1;
	}
	printf("\nAll torque calibration migration tests passed.\n");
	return 0;
}