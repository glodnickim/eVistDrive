#ifndef ASSIST_LIMITS_H_
#define ASSIST_LIMITS_H_

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

/*
 * FW-091: which speed limit a request is entitled to depends on WHERE THE REQUEST CAME
 * FROM, not on a measurement of how fast the cranks happen to be spinning.
 *
 * It used to be decided by filtered cadence > 15 rpm. That filter is zeroed on every pedal
 * stop and rebuilds exponentially, so after a lull the pedal request was classified
 * non-pedal and clamped to 5-7 km/h — a hard zero at riding speed — for 45 deg of crank at
 * 60 rpm, 180 deg at 20 rpm, and FOREVER below 16 rpm, where the filter converges to 120
 * against a threshold of 128. That is the "sometimes a touch, sometimes half a turn or
 * more" the rider reported.
 */
typedef enum {
	/* Legally confirmed pedalling: the ride latch armed, which already required forward
	 * crank direction, the configured PAS step count and pedal load over the configured
	 * kg threshold. Those are conditions the rider can see and set. */
	ASSIST_LIMIT_SOURCE_PEDAL_CONFIRMED = 0,
	/* Throttle, a without-rotation launch, and FW-084 Extended Boost: no crank rotation is
	 * being confirmed, so the low non-pedal speed limit applies. For Extended Boost this is
	 * a decided policy set explicitly in ride_control.c, not a consequence of the latch
	 * having dropped — see the comment there before changing it. */
	ASSIST_LIMIT_SOURCE_NON_PEDAL = 1
} assist_limit_source_t;

typedef struct {
	uint16_t voltage_raw;
	int16_t voltage_min_raw;
	int16_t controller_temperature_c;
	assist_limit_source_t source; /* FW-091: replaces the cadence-based guess */
	uint32_t speed_x100;
	uint16_t speed_limit_x100;
	bool legal_enabled;
	bool offroad;
	bool walk_active;
} assist_limits_input_t;

int32_t assist_limits_apply(
	int32_t iq_request,
	const assist_limits_input_t *input);

int32_t assist_limits_apply_legacy(
	int32_t iq_request,
	uint16_t voltage_raw,
	uint16_t cadence_filtered,
	uint16_t speed_limit_x100,
	const MotorState_t *motor_state,
	const MotorParams_t *motor_params);

#endif /* ASSIST_LIMITS_H_ */
