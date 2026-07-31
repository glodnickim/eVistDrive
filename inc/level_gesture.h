#ifndef LEVEL_GESTURE_H_
#define LEVEL_GESTURE_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * FW-050: one shared detector for all "bounce the assist level to toggle something" gestures.
 *
 * Before this there were two separate implementations with different rules, and the offroad one
 * carried real defects:
 *  - it accumulated a decimal code with pow(10, n) into a uint16_t; after a toggle the exponent
 *    was deliberately set to 8/9 (it doubled as the display splash value), so the next level
 *    change overflowed the variable — undefined behaviour that could re-trigger the toggle,
 *  - its per-step timing window (0.25-1 s) silently dropped changes made too quickly,
 *  - its digit counter shared a variable with the speed-field splash, so a quick double tap on
 *    the level button made the speedometer read 1 km/h, and a bank switch showed 4 km/h before
 *    the intended 10/20.
 *
 * The unified rule is the simpler one that the bank gesture already used:
 *   an exact sequence of assist levels, all of it inside ONE time window.
 * No minimum gap, no arithmetic, no shared state. Adding a gesture is one table row.
 */

#define LEVEL_GESTURE_MAX_STEPS 6   /* longest sequence a single gesture may have */
#define LEVEL_GESTURE_MAX_COUNT 8   /* how many gestures the table may hold */

typedef struct {
	uint8_t sequence[LEVEL_GESTURE_MAX_STEPS]; /* assist levels to hit, in order */
	uint8_t length;                            /* how many of them are used */
	uint16_t window_ticks;                     /* whole sequence must fit here (4 kHz ticks) */
	uint8_t splash_kmh;                        /* speed-field confirmation, 0 = none */
	void (*action)(void);                      /* run once the sequence completes */
} level_gesture_t;

/* The engine lives here, the policy (which gestures exist, what they do) stays in main.c —
 * that keeps this module free of any dependency on banks, offroad or the display. */
void level_gesture_init(const level_gesture_t *table, uint8_t count);

/* Call once per 4 kHz control tick with the current assist level. Handles both the timing
 * window and the confirmation splash. */
void level_gesture_update(uint8_t assist_level);

/* Speed value (km/h) the display should show instead of real speed, 0 = show real speed. */
uint8_t level_gesture_splash_kmh(void);

/* For gestures whose confirmation depends on the result (e.g. which bank was selected):
 * call from the action to override the table's splash value. */
void level_gesture_set_splash(uint8_t kmh);

void level_gesture_reset(void);

#endif /* LEVEL_GESTURE_H_ */
