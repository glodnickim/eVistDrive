#include "battery_trip.h"

/* See inc/battery_trip.h. */

static volatile int32_t threshold_counts;   /* written by main, read by the ISR (aligned word) */
static uint16_t consecutive;                 /* ISR-owned                                       */
static volatile uint8_t latched;
static volatile uint32_t trips;
static uint32_t standstill_ms;               /* main-owned                                      */

void battery_trip_init(void)
{
	threshold_counts = 0;
	consecutive = 0U;
	latched = 0U;
	trips = 0U;
	standstill_ms = 0U;
}

void battery_trip_set_threshold_counts(int32_t counts)
{
	threshold_counts = counts;
}

bool battery_trip_sample(int32_t delta_counts)
{
	const int32_t thr = threshold_counts;

	if (thr <= 0 || delta_counts <= thr) {
		consecutive = 0U;
		return false;
	}
	if (consecutive < BATTERY_TRIP_SAMPLES) {
		consecutive++;
	}
	if (consecutive >= BATTERY_TRIP_SAMPLES && !latched) {
		latched = 1U;
		if (trips != UINT32_MAX) {
			trips++;
		}
		return true;
	}
	return false;
}

bool battery_trip_latched(void)
{
	return latched != 0U;
}

void battery_trip_service(bool standstill, uint32_t elapsed_ms)
{
	if (!latched) {
		standstill_ms = 0U;
		return;
	}
	if (!standstill) {
		standstill_ms = 0U;
		return;
	}
	standstill_ms = (standstill_ms > UINT32_MAX - elapsed_ms) ? UINT32_MAX : standstill_ms + elapsed_ms;
	if (standstill_ms >= BATTERY_TRIP_REARM_MS) {
		consecutive = 0U;
		latched = 0U;
		standstill_ms = 0U;
	}
}

uint32_t battery_trip_count(void)
{
	return trips;
}
