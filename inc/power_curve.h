#ifndef POWER_CURVE_H_
#define POWER_CURVE_H_

#include <stdint.h>

/*
 * FW-056: y = x^gamma shaping for ASSIST_MODE_POWER_CURVE.
 *
 * Integer only — this runs inside the 4 kHz control path, so the curve comes
 * from a generated lookup table (inc/power_curve_lut.h) with linear
 * interpolation between points, never from powf().
 *
 * Contract:
 *   input 0    -> 0
 *   input 1000 -> 1000
 *   monotonic, never above 1000
 *   input above 1000 and out-of-range exponents are clamped, not rejected
 */

#define POWER_CURVE_EXP_MIN_X10 3U   /* gamma 0.3 — bends the other way */
#define POWER_CURVE_EXP_MAX_X10 25U  /* gamma 2.5 */
/* gamma 1.0 (x10 = 10) is the straight line the other shapes are judged against */
#define POWER_CURVE_EXP_DEFAULT_X10 15U

uint16_t power_curve_eval_permille(uint16_t input_permille, uint8_t exponent_x10);

#endif /* POWER_CURVE_H_ */
