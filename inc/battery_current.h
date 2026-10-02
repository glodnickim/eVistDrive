#ifndef BATTERY_CURRENT_H_
#define BATTERY_CURRENT_H_

#include <stdint.h>

/*
 * FW-128B1: the battery-current filter, moved onto the sample clock.
 *
 * WHAT WAS WRONG. The samples were never the problem. ADC0's regular group is triggered by
 * TIMER1 CH1 in hardware and lands in adc_value[] through a circular DMA, so a fresh PA0
 * reading arrives every 250 us whatever the CPU is doing. The FILTER, however, lived in
 * reg_ADC_processing(), which the main loop calls when it gets round to it - and it read only
 * whatever adc_value[0] happened to hold at that moment. Every skipped pass therefore threw a
 * complete sample away and advanced the IIR one step less.
 *
 * The IIR is a 64-sample exponential, so its time constant is 64 UPDATES. Measured on the host
 * in FW-128B0: 15.5 ms at one update per tick, 31.0 ms at every second tick, 77.5 ms at every
 * fifth. The filter's bandwidth was a function of CAN traffic and display activity.
 *
 * That is tolerable for a display reading. It is not tolerable underneath a limiter with any
 * state at all: an integrator's Ki is defined against a sample interval, and this one had no
 * fixed sample interval to be defined against. FW-128B0 made that an explicit prerequisite,
 * and this module is it.
 *
 * WHAT THIS MODULE IS NOT. It is not a recalibration. CAL_BAT_I is untouched and stays in
 * main.c, applied once to the value published here - so the absolute-scale question FW-128B0
 * left open (37 versus a stock-derived 39.216, PARTIAL) is neither answered nor disturbed here.
 * Timing and calibration are separate defects and stay separate.
 *
 * It is also not a new filter. The law is character-for-character the production one:
 *
 *     acc -= acc >> 6;  acc += (raw - offset);  published = acc >> 6;
 *
 * The accumulator's fixed point is 64x, so the >>6 recovers the mean: DC gain exactly 1. At the
 * proven 4 kHz sample rate the discrete pole is 63/64, giving a time constant of about 63 sample
 * periods = 15.75 ms and a 99 % settling time of roughly 290 samples = 72 ms.
 *
 * UNITS. Everything here is in ADC counts. The conversion to milliamperes is one multiply by
 * CAL_BAT_I and it happens in main.c, deliberately outside the interrupt - the ISR stays integer.
 */

typedef struct {
	uint32_t sample_count;     /* fresh samples offered by the sampling event               */
	uint32_t update_count;     /* IIR updates performed - must equal sample_count when armed */
	uint32_t unarmed_count;    /* samples refused before the startup zero was established     */
	uint32_t late_scan_count;  /* the regular scan had not finished when the sample was taken */
} battery_current_stats_t;

/*
 * Power-on. Clears everything and leaves the module UNARMED: no sample is accepted and the
 * published value stays 0 until the startup zero is known.
 *
 * Lifecycle option A of the card, chosen over "reseed afterwards": a wrong-domain accumulator
 * is then not merely corrected, it never exists. The startup calibration reads adc_value[0]
 * directly and does not need this module at all, so nothing is lost by refusing samples until
 * it has finished.
 */
void battery_current_init(void);

/*
 * Establish the zero and ARM the module. Called once, straight after the startup calibration -
 * whether that calibration accepted its measurement or fell back to the compile-time default,
 * because either way that is the offset the rest of the ride will use.
 *
 * The accumulator is seeded to 0, not to the first sample: the offset was just measured at
 * essentially zero current, so zero IS the correct expected state, and any other seed would be
 * a guess that takes 64 samples to decay.
 */
void battery_current_set_offset(int32_t offset);

/*
 * ONE fresh regular-ADC sample -> exactly ONE filter update. Called from TIMER1_IRQHandler and
 * from nowhere else.
 *
 * `raw` is adc_value[0] (PA0, ADC0 regular rank 0). `scan_complete` is the caller's evidence
 * that the previous regular scan had finished - see main.c, where it comes from the DMA transfer
 * counter. A scan that had NOT finished is still accepted, because rank 0 is the FIRST transfer
 * of the scan and is therefore written long before this point; the flag is recorded as evidence,
 * not used as permission.
 *
 * Integer only. No float, no division, no loop, no peripheral access.
 */
void battery_current_sample(uint16_t raw, uint8_t scan_complete);

/*
 * The published filtered value, in ADC counts, signed (positive = discharge). One aligned 32-bit
 * word written by the sampler and read by anyone - single-copy atomic on this core, so a
 * consumer can never observe half of an update and no lock is needed.
 */
int32_t battery_current_filtered_adc(void);

/*
 * TASK-EVD-TQ-06-G1 (ADR-013 D2): the same samples through a 1/8 pole (time constant about
 * 7.5 sample periods = 1.9 ms at 4 kHz), in ADC counts, for the G53 PI #1 battery-current
 * limiter only. 0 until armed. Every other consumer keeps battery_current_filtered_adc().
 */
int32_t battery_current_limiter_adc(void);

/* The latest raw sample minus the startup zero, unfiltered, in ADC counts; for the hard
 * battery-overcurrent trip, which runs in the same ISR right after battery_current_sample(). */
int32_t battery_current_last_delta_adc(void);

const battery_current_stats_t *battery_current_get_stats(void);

#endif /* BATTERY_CURRENT_H_ */
