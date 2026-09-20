/*
 * TORQUE CALIBRATION GAIN REFERENCE DECOUPLING TEST
 *
 * FW-151 bug candidate B / audit item "Mutation E": torque_input_cal_tick() must scale a
 * calibration capture's span against the FROZEN TORQUE_GAIN_REFERENCE_NATIVE, never against
 * TORQUE_DEFAULT_SPAN_NATIVE. The two constants are numerically equal today only because the
 * gain reference was seeded from the FW-150 curve measurement - that is coincidence, not
 * contract. A future re-measurement of the physical kg curve (TORQUE_CURVE_P*) changes
 * TORQUE_DEFAULT_SPAN_NATIVE (it is derived from the curve by extrapolation, see
 * torque_input.h) and must NOT retroactively rescale every already-calibrated rider's stored
 * gain.
 *
 * This test builds torque_input.c with TORQUE_DEFAULT_SPAN_NATIVE overridden at compile time
 * (see the #ifndef guard in torque_input.h, added for exactly this purpose) to a value that
 * differs from TORQUE_GAIN_REFERENCE_NATIVE, simulating a future kg-curve re-measurement. It
 * then drives the REAL calibration state machine through one capture at a reference weight
 * chosen so the measured sensor delta equals the DEFAULT delta at that weight exactly (a gain
 * of 1.0x). If cal_tick scales against the (now different) TORQUE_DEFAULT_SPAN_NATIVE, the
 * resulting span drifts away from TORQUE_GAIN_REFERENCE_NATIVE. If it correctly scales against
 * TORQUE_GAIN_REFERENCE_NATIVE, the span for a gain-1.0x sensor is exactly
 * TORQUE_GAIN_REFERENCE_NATIVE regardless of where the kg curve currently sits.
 *
 * Run-host-tests.ps1 builds this suite with -DTORQUE_DEFAULT_SPAN_NATIVE=3600U, a value inside
 * the accepted [TORQUE_SPAN_MIN_NATIVE, TORQUE_SPAN_MAX_NATIVE] range but different from the
 * frozen TORQUE_GAIN_REFERENCE_NATIVE (3047), so a coupled implementation is caught by value,
 * not just by inequality with itself.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "torque_input.h"

static int failures;

#define CHECK(cond, what) do { \
	if (!(cond)) { \
		printf("  FAIL  %s\n", (what)); \
		failures++; \
	} \
} while (0)

static void cal_settle(int16_t corrected, unsigned ticks)
{
	for (unsigned i = 0; i < ticks; i++) {
		torque_input_cal_tick(corrected, true);
	}
}

static void test_span_invariant_to_default_span_native(void)
{
	printf("TC1 calibration span is invariant to TORQUE_DEFAULT_SPAN_NATIVE\n");

	CHECK(TORQUE_DEFAULT_SPAN_NATIVE != TORQUE_GAIN_REFERENCE_NATIVE,
		"TC1 premise: this build overrides TORQUE_DEFAULT_SPAN_NATIVE away from "
		"TORQUE_GAIN_REFERENCE_NATIVE, or the test proves nothing");

	torque_input_init();

	/* 9.00 kg: the default curve answers with a native delta comfortably above
	 * CAL_DELTA_MIN_NATIVE (100), unlike a lighter reference weight which would fail the
	 * calibration on DELTA_TOO_SMALL before the span arithmetic under test even runs. */
	const uint16_t reference_centikg = 900U;
	/* calibration_source is still DEFAULT here, so this is exactly the delta cal_tick's
	 * reference_delta will compute internally - a gain-1.0x sensor reproduces it exactly. */
	uint16_t reference_delta = torque_input_centikg_to_native_delta(reference_centikg);
	CHECK(reference_delta > 0U, "TC1: reference weight maps to a nonzero native delta");

	torque_input_update(TORQUE_ZERO_TARGET_NATIVE, (int16_t)TORQUE_ZERO_TARGET_NATIVE, true);
	torque_input_cal_start();
	CHECK(torque_input_cal_state() == TORQUE_CAL_STATE_CAPTURE_ZERO,
		"TC1: calibration starts in CAPTURE_ZERO");
	cal_settle((int16_t)TORQUE_ZERO_TARGET_NATIVE, 256U);
	CHECK(torque_input_cal_state() == TORQUE_CAL_STATE_WAIT_REFERENCE,
		"TC1: zero capture completes");

	int32_t loaded = (int32_t)TORQUE_ZERO_TARGET_NATIVE + (int32_t)reference_delta;
	torque_input_update((uint16_t)loaded, (int16_t)loaded, true);
	torque_input_cal_capture_load(reference_centikg);
	CHECK(torque_input_cal_state() == TORQUE_CAL_STATE_CAPTURE_LOAD,
		"TC1: load capture is armed");
	cal_settle((int16_t)loaded, 256U);

	CHECK(torque_input_cal_state() == TORQUE_CAL_STATE_PREVIEW,
		"TC1: a gain-1.0x sensor at a valid reference weight calibrates successfully");
	uint16_t span = torque_input_cal_preview_span();
	CHECK(span == TORQUE_GAIN_REFERENCE_NATIVE,
		"TC1: span for a gain-1.0x sensor equals the FROZEN TORQUE_GAIN_REFERENCE_NATIVE, "
		"not the (overridden, differing) TORQUE_DEFAULT_SPAN_NATIVE");
	CHECK(span != TORQUE_DEFAULT_SPAN_NATIVE,
		"TC1: span must NOT equal TORQUE_DEFAULT_SPAN_NATIVE in this build, proving the "
		"two constants are decoupled");
}

int main(void)
{
	printf("Torque calibration gain reference decoupling test\n");
	printf("===================================================\n");

	test_span_invariant_to_default_span_native();

	if (failures != 0) {
		printf("\nFAILURES: %d\n", failures);
		return 1;
	}
	printf("\nAll torque gain decoupling tests passed.\n");
	return 0;
}
