#ifndef CADENCE_COMP_H_
#define CADENCE_COMP_H_

#include <stdint.h>

/*
 * FW-057: cadence compensation map.
 *
 * The motor loses effective torque as cadence rises, so the same rider effort
 * produces less assist at 100 rpm than at 60. This map scales the assist request
 * back up, per the measured characteristic:
 *
 *   0..70 rpm : 100%
 *      80 rpm :  82%
 *     100 rpm :  93%
 *     110 rpm : 106%
 *     120 rpm : 132%
 *    >120 rpm : 132% (held, never extrapolated further)
 *
 * Linear interpolation between the points, so the rider never feels a step.
 *
 * This raises the REQUEST only. Every downstream limit — motor power, Iq,
 * battery current, voltage headroom, temperature — still applies unchanged, so
 * 132% cannot push the hardware past any safety ceiling.
 */

#define CADENCE_COMP_UNITY_PERMILLE 1000U
#define CADENCE_COMP_MAX_PERMILLE 1320U

uint16_t cadence_comp_multiplier_permille(uint8_t cadence_rpm);

#endif /* CADENCE_COMP_H_ */
