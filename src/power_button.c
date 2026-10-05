/*
 * DISC-010: on/off button power-off as in the original application. See inc/power_button.h.
 */
#include "power_button.h"

enum {
	PB_IDLE = 0,      /* not pressed                                  */
	PB_FIRST,         /* 1 pressed sample                             */
	PB_SECOND,        /* 2 pressed samples                            */
	PB_HELD,          /* press confirmed, hold count running          */
	PB_GAP1,          /* 1 sample outside the window during the press */
	PB_GAP2           /* 2 samples outside the window                 */
};

static uint8_t  pb_state;
static uint16_t pb_hold;
static bool     pb_fired;        /* the long press of THIS press has fired */
static uint32_t pb_last_sample;
static bool     pb_off_latched;
static uint32_t pb_off_since;
static bool     pb_off_due;

void power_button_init(uint32_t now_tick)
{
	pb_state = PB_IDLE;
	pb_hold = 0U;
	pb_fired = false;
	pb_last_sample = now_tick;
	pb_off_latched = false;
	pb_off_since = now_tick;
	pb_off_due = false;
}

static void pb_end_press(void)
{
	pb_state = PB_IDLE;
	pb_hold = 0U;
	pb_fired = false;
}

/* One 8 ms sample of the original application's button machine. */
static void pb_sample(bool pressed, uint32_t now_tick)
{
	switch (pb_state) {
	case PB_IDLE:
		if (pressed) pb_state = PB_FIRST;
		break;
	case PB_FIRST:
		pb_state = pressed ? PB_SECOND : PB_IDLE;
		break;
	case PB_SECOND:
		if (pressed) {
			pb_state = PB_HELD;
			pb_hold = POWER_BUTTON_HOLD_START;
		} else {
			pb_state = PB_IDLE;
		}
		break;
	case PB_HELD:
		/* As in the original: the count advances on this sample even if it is the first one
		 * outside the window, and the long-press test follows the state change. */
		if (pb_hold < 0xFFFFU) pb_hold++;
		pb_state = pressed ? PB_HELD : PB_GAP1;
		if (pb_hold == POWER_BUTTON_LONG_COUNT && !pb_fired) {
			pb_fired = true;
			if (!pb_off_latched) {
				pb_off_latched = true;
				pb_off_since = now_tick;
			}
		}
		break;
	case PB_GAP1:
		pb_state = pressed ? PB_HELD : PB_GAP2;
		break;
	case PB_GAP2:
	default:
		if (pressed) pb_state = PB_HELD;
		else pb_end_press();
		break;
	}
}

void power_button_update(uint32_t now_tick, uint16_t pa4_raw)
{
	if ((uint32_t)(now_tick - pb_last_sample) >= POWER_BUTTON_SAMPLE_TICKS) {
		pb_last_sample += POWER_BUTTON_SAMPLE_TICKS;
		/* More than a whole period behind: drop the backlog instead of bursting samples. */
		if ((uint32_t)(now_tick - pb_last_sample) >= POWER_BUTTON_SAMPLE_TICKS) pb_last_sample = now_tick;
		pb_sample(pa4_raw >= POWER_BUTTON_PA4_MIN && pa4_raw <= POWER_BUTTON_PA4_MAX, now_tick);
	}
	if (pb_off_latched && (uint32_t)(now_tick - pb_off_since) > POWER_BUTTON_OFF_DELAY_TICKS) {
		pb_off_due = true;
	}
}

bool power_button_power_off_due(void)  { return pb_off_due; }
uint8_t power_button_state(void)       { return pb_state; }
uint16_t power_button_hold_count(void) { return pb_hold; }
bool power_button_off_latched(void)    { return pb_off_latched; }
