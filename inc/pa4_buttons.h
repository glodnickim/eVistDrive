#ifndef PA4_BUTTONS_H_
#define PA4_BUTTONS_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * DISC-010: the ONE owner of the PA4 button line (on/off and "down"), behaving as the original
 * application does. Every PA4 consumer in main.c reads this module, never adc_value[5] directly.
 *
 * WHY. The inherited M560 rules never switched the M820 off (PA4 < 2800 on every 40 ms pass, 63 in
 * a row, one stray sample threw the hold away) and dropped Walk on any dropout longer than ~5 ms.
 * The original M820 application (and FT 2026-05-22, byte-identical on this path apart from the
 * Walk start, see below) does neither. Reverse: integration/discoveries/DISC-010.md.
 *
 * ORIGINAL APPLICATION (stock CRX30PC3612E102003.5; 1 ms tick, each machine samples every 8 ms):
 *  - windows: on/off 2048..2730 (0x08013630), "down" 2854..3723 (0x080134b0), both inclusive;
 *  - one machine per button (0x20001080 on/off, 0x20001098 down), states 0..5:
 *    0 -> 1 -> 2 on pressed samples (3 in a row confirm the press), 2 -> 3 sets the hold count to 2,
 *    3 counts +1 per sample (also on the first sample outside the window) and goes to 4 when outside,
 *    4 and 5 return to 3 on a pressed sample with the hold count intact; a 3rd sample outside ends
 *    the press (5 -> 0);
 *  - on/off: hold count 200 (~1.6 s) latches the power-off, which happens > 500 ms later whatever
 *    the button does (0x080174d4, 0x0801740a) - ~2.1 s with the button held the whole time;
 *  - down / Walk: Walk is active while the down machine is in state 3 (0x080171d8) and the display
 *    selects Walk (level 6). Once the hold count is >= 200 (~1.6 s) with Walk active the machine
 *    samples every 1 ms and a dropout is bridged for 250 samples = 250 ms (0x08017b58, 0x08017bfa);
 *    the original runs this from its 1 ms timer IRQ, so 250 samples are 250 ms. Here the module
 *    runs in the foreground loop, so the bridge is bounded by elapsed time since the last pressed
 *    sample (PA4_WALK_BRIDGE_TICKS), whatever the loop rate; release is seen on the first sample
 *    after the deadline, i.e. within one loop iteration of it;
 *  - the stock start requires the 1.6 s hold (event 6) before Walk may run; FT removes that
 *    condition (cmp r4,#6 ; bne -> nop ; nop) and so do we: Walk follows state 3 at once.
 *
 *  - button circuit test (0x08017308): while both buttons are idle, every 256 ms PB8 goes up for
 *    ~5 ms and PA4 must fall below 1240 on steps 2..4; three failing samples in a row (without a
 *    good one) set the circuit fault, a passing test clears it. The machines do not sample while
 *    PB8 owns the line; a press freezes the test;
 *  - on/off held > 800 samples (6.4 s) sets the stuck flag (0x080178e0), cleared when the press
 *    ends; either flag is error 36 on the display (0x08018e52) and blocks Walk (0x0800fb7c) -
 *    main.c maps it to MS.error_state = ERR_BUTTON;
 *  - the press the bike was switched on with is consumed by the original's power-on event; here
 *    the press present since init must end before an on/off press can switch the bike off.
 *
 * Deliberately NOT copied: the 0x02F83204 CAN notice during the power-off delay, combined-button
 * events, the service mode (0x20000357) and the power-on hold check (see DISC-010).
 *
 * Timebase: control_time_ticks (4 kHz). Called from main()'s while(1); a late loop only delays a
 * sample, it never produces a burst of catch-up samples.
 */

#define PA4_POWER_MIN               2048U  /* 0x800 */
#define PA4_POWER_MAX               2730U  /* 0xAAA */
#define PA4_DOWN_MIN                2854U  /* 0xB26 */
#define PA4_DOWN_MAX                3723U  /* 0xE8B */
#define PA4_SAMPLE_TICKS            32U    /* 8 ms at 4 kHz                                     */
#define PA4_FAST_SAMPLE_TICKS       4U     /* 1 ms: down machine while Walk is bridged          */
#define PA4_HOLD_START              2U     /* hold count on entering state 3                    */
#define PA4_LONG_COUNT              200U   /* ~1.6 s: on/off long press, Walk bridge armed       */
#define PA4_WALK_BRIDGE_TICKS       1000U  /* 250 ms dropout bridged during Walk (elapsed time)  */
#define PA4_POWER_OFF_DELAY_TICKS   2000U  /* > 500 ms from the long press to power-off         */
#define PA4_STUCK_COUNT             800U   /* on/off hold count above which the button is stuck  */
#define PA4_PROBE_MAX               1240U  /* 0x4D8: PA4 must be below this while PB8 is up      */

void pa4_buttons_init(uint32_t now_tick);

/* main()'s while(1), every iteration: control_time_ticks, the raw PA4 sample, and whether Walk is
 * currently running (MS.pushassist_flag). */
void pa4_buttons_update(uint32_t now_tick, uint16_t pa4_raw, bool walk_active);

/* On/off: true once the latched power-off delay has run out; stays true. */
bool pa4_buttons_power_off_due(void);
/* On/off press confirmed and not yet over (states 3..5). */
bool pa4_buttons_power_pressed(void);
/* Down: held in the original application's sense (state 3) - the Walk button. */
bool pa4_buttons_walk_held(void);
/* Either button between its first pressed sample and the end of the press. */
bool pa4_buttons_any_activity(void);

/* Diagnostics / tests. */
uint8_t  pa4_buttons_power_state(void);
uint16_t pa4_buttons_power_hold(void);
bool     pa4_buttons_power_off_latched(void);
uint8_t  pa4_buttons_down_state(void);
uint16_t pa4_buttons_down_hold(void);

/* Button circuit test output: main.c drives PB8 from this after every update. */
bool pa4_buttons_pb8_high(void);
/* Error 36: circuit fault or stuck on/off button. */
bool pa4_buttons_fault(void);
bool pa4_buttons_fault_circuit(void);
bool pa4_buttons_fault_stuck(void);
uint32_t pa4_buttons_power_samples(void);   /* diagnostics: on/off samples taken since init */

#endif
