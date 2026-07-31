#include "cadence_comp.h"

typedef struct {
	uint8_t cadence_rpm;
	uint16_t multiplier_permille;
} cadence_comp_point_t;

/* Breakpoints of the measured characteristic. Must stay sorted by cadence. */
static const cadence_comp_point_t cadence_comp_map[] = {
	{  0, 1000U },
	{ 70, 1000U },
	{ 80,  820U },
	{100,  930U },
	{110, 1060U },
	{120, 1320U },
};

#define CADENCE_COMP_POINTS \
	(sizeof(cadence_comp_map) / sizeof(cadence_comp_map[0]))

uint16_t cadence_comp_multiplier_permille(uint8_t cadence_rpm)
{
	const cadence_comp_point_t *last =
		&cadence_comp_map[CADENCE_COMP_POINTS - 1U];
	if (cadence_rpm >= last->cadence_rpm) {
		return last->multiplier_permille; /* held, never extrapolated */
	}

	for (uint8_t i = 1U; i < CADENCE_COMP_POINTS; i++) {
		const cadence_comp_point_t *high = &cadence_comp_map[i];
		if (cadence_rpm >= high->cadence_rpm) {
			continue;
		}
		const cadence_comp_point_t *low = &cadence_comp_map[i - 1U];
		uint32_t span_rpm = (uint32_t)(high->cadence_rpm - low->cadence_rpm);
		uint32_t into_rpm = (uint32_t)(cadence_rpm - low->cadence_rpm);
		if (high->multiplier_permille >= low->multiplier_permille) {
			uint32_t rise = high->multiplier_permille - low->multiplier_permille;
			return (uint16_t)(low->multiplier_permille +
				(rise * into_rpm + span_rpm / 2U) / span_rpm);
		}
		uint32_t drop = low->multiplier_permille - high->multiplier_permille;
		return (uint16_t)(low->multiplier_permille -
			(drop * into_rpm + span_rpm / 2U) / span_rpm);
	}

	return CADENCE_COMP_UNITY_PERMILLE;
}
