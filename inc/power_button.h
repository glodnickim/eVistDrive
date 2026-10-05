#ifndef POWER_BUTTON_H_
#define POWER_BUTTON_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * DISC-010: on/off button power-off, behaving as the original application does.
 *
 * WHY. The inherited M560 rule (PA4 < 2800 on every 40 ms slow-loop pass, 63 in a row) never
 * switched the M820 off: a single sample outside the condition threw the whole 2.5 s hold away.
 * The original M820 application (and FT 2026-05-22, byte-identical on this path) switches off
 * reliably with the button still held. Reverse: integration/discoveries/DISC-010.md.
 *
 * WHAT (original application, addresses in the stock image CRX30PC3612E102003.5):
 *  - PA4 is sampled every 8 ms (0x08017658, prescaler of 8 one-millisecond calls);
 *  - "pressed" = 2048 <= PA4 <= 2730 (0x08013650);
 *  - 3 pressed samples in a row start a press (states 0 -> 1 -> 2); the hold count starts at 2
 *    and grows by one per sample while held (state 3);
 *  - releasing is tolerated: up to 2 samples in a row outside the window (states 4, 5) and the
 *    press resumes with its hold count intact; the 3rd one ends the press;
 *  - the long press fires once, when the hold count reaches 200 (system on) = ~1.6 s;
 *  - from then on the power-off is latched and happens > 500 ms later (0x080174f4, 0x0801742a),
 *    whatever the button does. Together ~2.1 s with the button held the whole time.
 *
 * Deliberately NOT copied: the CAN notice 0x02F83204 the original sends during the 500 ms
 * (semantics unconfirmed, the patched HMI is cut by PB5 anyway), the combined-button events and
 * the power-on hold check (the M820 is switched on by hardware here; out of scope).
 *
 * Timebase: control_time_ticks (4 kHz). Called from main()'s while(1); a late loop only delays a
 * sample, it never produces a burst of catch-up samples.
 */

#define POWER_BUTTON_PA4_MIN            2048U  /* 0x800, inclusive */
#define POWER_BUTTON_PA4_MAX            2730U  /* 0xAAA, inclusive */
#define POWER_BUTTON_SAMPLE_TICKS       32U    /* 8 ms at 4 kHz                            */
#define POWER_BUTTON_HOLD_START         2U     /* hold count on entering the held state     */
#define POWER_BUTTON_LONG_COUNT         200U   /* hold count that fires the long press      */
#define POWER_BUTTON_OFF_DELAY_TICKS    2000U  /* > 500 ms from the long press to power-off */

void power_button_init(uint32_t now_tick);

/* main()'s while(1), every iteration: the current control_time_ticks and the raw PA4 sample. */
void power_button_update(uint32_t now_tick, uint16_t pa4_raw);

/* True once the latched power-off delay has run out; stays true. */
bool power_button_power_off_due(void);

/* Diagnostics / tests. */
uint8_t  power_button_state(void);       /* 0..5, as in the original application */
uint16_t power_button_hold_count(void);
bool     power_button_off_latched(void);

#endif
