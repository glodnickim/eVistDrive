#ifndef BATTERY_TRIP_H_
#define BATTERY_TRIP_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * TASK-EVD-TQ-06-G1 / ADR-013 D1: hard battery-overcurrent trip.
 *
 * WHY. On the PEDAL path the battery current is now limited by the ported G53 PI #1 (g1), a
 * continuous regulator. The G5300 has no soft switch next to it; outside the regulator it only
 * has a hard trip far above the limit (N4 §2.9: unfiltered current, consecutive samples, latch,
 * PWM off, re-arm at standstill). This is the M820 equivalent of that idea - an M820 safety stage
 * sized for M820, not a port of the G5300 thresholds.
 *
 * WHAT. The unfiltered battery current (raw PA0 minus the startup zero, ADC counts, one sample per
 * TIMER1 period = 4 kHz) is compared with a threshold of "configured limit + 15 A" (owner
 * decision; ADR-013-prep/trip_sweep.py shows the G53 transient stays below it with >= ~3 A margin
 * for every simulated plant). BATTERY_TRIP_SAMPLES consecutive samples above it latch the trip;
 * the ISR then switches the bridge outputs off at once and the main loop owns everything else.
 * The latch clears only after the bike has stood still for BATTERY_TRIP_REARM_MS.
 *
 * Single producer of the latch: battery_trip_sample() (TIMER1 ISR). Single clearer:
 * battery_trip_service() (main loop), and it only clears at standstill, where the current - and
 * with it any chance of the ISR latching in the same instant - is zero.
 */

#define BATTERY_TRIP_SAMPLES     16U    /* consecutive 4 kHz samples = 4 ms                   */
#define BATTERY_TRIP_MARGIN_MA   15000  /* threshold = configured battery limit + 15 A        */
#define BATTERY_TRIP_REARM_MS    100U   /* standstill (no wheel, no cadence) before re-arming */

void battery_trip_init(void);

/* Main loop: threshold in ADC counts above the zero. <= 0 disables the trip. */
void battery_trip_set_threshold_counts(int32_t counts);

/* TIMER1 ISR, once per sample, right after battery_current_sample(). Returns true exactly on the
 * sample that latches the trip (the caller switches the bridge off). */
bool battery_trip_sample(int32_t delta_counts);

bool battery_trip_latched(void);

/* Main loop. `standstill` = no wheel speed and no cadence; `elapsed_ms` since the last call. */
void battery_trip_service(bool standstill, uint32_t elapsed_ms);

/* Number of trips since power-on (saturating). */
uint32_t battery_trip_count(void);

#endif
