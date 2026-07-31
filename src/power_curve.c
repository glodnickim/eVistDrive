#include "power_curve.h"

#include "power_curve_lut.h"

uint16_t power_curve_eval_permille(uint16_t input_permille, uint8_t exponent_x10)
{
	if (input_permille > POWER_CURVE_LUT_FULL_SCALE) {
		input_permille = POWER_CURVE_LUT_FULL_SCALE;
	}
	if (exponent_x10 < POWER_CURVE_EXP_MIN_X10) {
		exponent_x10 = POWER_CURVE_EXP_MIN_X10;
	}
	if (exponent_x10 > POWER_CURVE_EXP_MAX_X10) {
		exponent_x10 = POWER_CURVE_EXP_MAX_X10;
	}

	uint8_t table_row = exponent_x10 - POWER_CURVE_EXP_MIN_X10;

	/*
	 * Position on the table grid, scaled by 1000 so the fractional part stays
	 * integer: 0 .. (POINTS-1)*1000. Peak intermediate is 1000*64 = 64000.
	 */
	uint32_t position = (uint32_t)input_permille *
		(POWER_CURVE_LUT_POINTS - 1U);
	const uint16_t *row;
	uint32_t last_index;

	if (position < POWER_CURVE_LUT_FULL_SCALE) {
		/*
		 * Inside the first main segment, where a gamma below 1 leaves zero
		 * vertically. The sub-grid resolves it; for gamma >= 1 it is simply a
		 * finer sampling of an almost flat piece of curve.
		 */
		row = POWER_CURVE_LUT_LOW[table_row];
		position *= (POWER_CURVE_LUT_LOW_POINTS - 1U);
		last_index = POWER_CURVE_LUT_LOW_POINTS - 1U;
	} else {
		row = POWER_CURVE_LUT[table_row];
		last_index = POWER_CURVE_LUT_POINTS - 1U;
	}

	uint32_t index = position / POWER_CURVE_LUT_FULL_SCALE;
	if (index >= last_index) {
		return row[last_index];
	}

	uint32_t fraction = position - index * POWER_CURVE_LUT_FULL_SCALE;
	uint32_t low = row[index];
	uint32_t high = row[index + 1U];

	return (uint16_t)(low +
		((high - low) * fraction + POWER_CURVE_LUT_FULL_SCALE / 2U) /
		POWER_CURVE_LUT_FULL_SCALE);
}
