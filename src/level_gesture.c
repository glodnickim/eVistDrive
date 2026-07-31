#include "level_gesture.h"

#define GESTURE_SPLASH_TICKS 12000U   /* ~3 s @4 kHz, same as the old bank splash */
#define GESTURE_NO_LEVEL     0xFFU    /* "no previous level seen yet" */

static const level_gesture_t *gesture_table;
static uint8_t gesture_count;

static uint8_t last_level = GESTURE_NO_LEVEL;
static uint8_t match_index[LEVEL_GESTURE_MAX_COUNT];   /* per gesture: how many steps matched */
static uint16_t match_window[LEVEL_GESTURE_MAX_COUNT]; /* per gesture: ticks left in its window */

/*
 * Splash state is deliberately SEPARATE from the match state. In the old code one variable
 * held both the digit position and the value shown on the display, so writing the confirmation
 * corrupted the gesture — that was the root of the overflow, of the bank confirmation being
 * masked, and of the speedometer reading 1 km/h after a quick double tap on the level button.
 */
static uint8_t splash_kmh;
static uint16_t splash_ticks;

void level_gesture_init(const level_gesture_t *table, uint8_t count)
{
	gesture_table = table;
	gesture_count = (count > LEVEL_GESTURE_MAX_COUNT) ?
		LEVEL_GESTURE_MAX_COUNT : count;
	level_gesture_reset();
}

void level_gesture_reset(void)
{
	last_level = GESTURE_NO_LEVEL;
	for (uint8_t i = 0; i < LEVEL_GESTURE_MAX_COUNT; i++) {
		match_index[i] = 0;
		match_window[i] = 0;
	}
	splash_kmh = 0;
	splash_ticks = 0;
}

void level_gesture_set_splash(uint8_t kmh)
{
	splash_kmh = kmh;
	splash_ticks = kmh ? GESTURE_SPLASH_TICKS : 0;
}

uint8_t level_gesture_splash_kmh(void)
{
	return splash_kmh;
}

void level_gesture_update(uint8_t assist_level)
{
	if (splash_ticks > 0 && --splash_ticks == 0) {
		splash_kmh = 0;
	}
	if (gesture_table == 0 || gesture_count == 0) {
		return;
	}

	/* Time out any gesture whose window has run out. */
	for (uint8_t g = 0; g < gesture_count; g++) {
		if (match_window[g] > 0 && --match_window[g] == 0) {
			match_index[g] = 0;
		}
	}

	if (assist_level == last_level) {
		return;                       /* only level CHANGES advance a gesture */
	}
	uint8_t previous = last_level;
	last_level = assist_level;
	if (previous == GESTURE_NO_LEVEL) {
		return;                       /* first level after boot is not a "change" */
	}

	for (uint8_t g = 0; g < gesture_count; g++) {
		const level_gesture_t *gesture = &gesture_table[g];
		if (gesture->length == 0 || gesture->length > LEVEL_GESTURE_MAX_STEPS) {
			continue;
		}
		if (assist_level == gesture->sequence[match_index[g]]) {
			if (match_index[g] == 0) {
				match_window[g] = gesture->window_ticks;  /* arm on first hit */
			}
			match_index[g]++;
			if (match_index[g] >= gesture->length) {
				match_index[g] = 0;
				match_window[g] = 0;
				level_gesture_set_splash(gesture->splash_kmh);
				if (gesture->action) {
					gesture->action();   /* may override the splash */
				}
			}
		} else {
			/* Wrong level: drop this attempt, but let it restart immediately if the
			 * level we just landed on is itself the start of the sequence. */
			match_index[g] = (assist_level == gesture->sequence[0]) ? 1 : 0;
			match_window[g] = match_index[g] ? gesture->window_ticks : 0;
		}
	}
}
