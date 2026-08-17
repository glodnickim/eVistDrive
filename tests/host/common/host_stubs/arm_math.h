/*
 * HOST TEST STUB - not part of the firmware. See gd32f30x.h in this directory for why
 * this exists at all: inc/main.h (pulled in transitively by inc/motor_core.h, which
 * ride_control.c needs to link) includes the real CMSIS-DSP <arm_math.h> purely to get
 * the q31_t fixed-point typedef used by one field of MotorParams_t (angle_correction).
 * No host test links FOC.c or calls any real CMSIS-DSP function, so nothing else from
 * that header is needed here.
 */
#ifndef HOST_STUB_ARM_MATH_H
#define HOST_STUB_ARM_MATH_H

#include <stdint.h>

typedef int32_t q31_t;

#endif /* HOST_STUB_ARM_MATH_H */
