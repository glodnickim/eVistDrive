/*
 * DISC-010: the PA4 button line (on/off and "down") as in the original application.
 * See inc/pa4_buttons.h.
 */
#include "pa4_buttons.h"

enum {
	PB_IDLE = 0,      /* not pressed                                  */
	PB_FIRST,         /* 1 pressed sample                             */
	PB_SECOND,        /* 2 pressed samples                            */
	PB_HELD,          /* press confirmed, hold count running          */
	PB_GAP1,          /* 1 sample outside the window during the press */
	PB_GAP2           /* 2 samples outside the window                 */
};

typedef struct {
	uint8_t  state;
	uint16_t hold;
	uint32_t last_sample;
} pa4_machine_t;

static pa4_machine_t pw;           /* on/off */
static pa4_machine_t dn;           /* down / Walk */
static bool     pw_fired;          /* the long press of THIS on/off press has fired */
static uint16_t dn_bridge;         /* remaining 1 ms samples of the Walk dropout bridge */
static bool     off_latched;
static uint32_t off_since;
static bool     off_due;

static void machine_init(pa4_machine_t *m, uint32_t now_tick)
{
	m->state = PB_IDLE;
	m->hold = 0U;
	m->last_sample = now_tick;
}

void pa4_buttons_init(uint32_t now_tick)
{
	machine_init(&pw, now_tick);
	machine_init(&dn, now_tick);
	pw_fired = false;
	dn_bridge = 0U;
	off_latched = false;
	off_since = now_tick;
	off_due = false;
}

/* Elapsed-tick sampler: true when a sample is due. A whole period of backlog is dropped. */
static bool sample_due(pa4_machine_t *m, uint32_t now_tick, uint32_t period)
{
	if ((uint32_t)(now_tick - m->last_sample) < period) return false;
	m->last_sample += period;
	if ((uint32_t)(now_tick - m->last_sample) >= period) m->last_sample = now_tick;
	return true;
}

/* States 0..2 and 4..5, common to both machines. Returns true when the press ended. */
static bool machine_common(pa4_machine_t *m, bool pressed)
{
	switch (m->state) {
	case PB_IDLE:
		if (pressed) m->state = PB_FIRST;
		break;
	case PB_FIRST:
		m->state = pressed ? PB_SECOND : PB_IDLE;
		break;
	case PB_SECOND:
		if (pressed) {
			m->state = PB_HELD;
			m->hold = PA4_HOLD_START;
		} else {
			m->state = PB_IDLE;
		}
		break;
	case PB_GAP1:
		m->state = pressed ? PB_HELD : PB_GAP2;
		break;
	case PB_GAP2:
	default:
		if (pressed) {
			m->state = PB_HELD;
		} else {
			m->state = PB_IDLE;
			m->hold = 0U;
			return true;
		}
		break;
	}
	return false;
}

static void power_sample(bool pressed, uint32_t now_tick)
{
	if (pw.state != PB_HELD) {
		if (machine_common(&pw, pressed)) pw_fired = false;
		return;
	}
	/* As in the original: the count advances on this sample even if it is the first one outside
	 * the window, and the long-press test follows the state change. */
	if (pw.hold < 0xFFFFU) pw.hold++;
	pw.state = pressed ? PB_HELD : PB_GAP1;
	if (pw.hold == PA4_LONG_COUNT && !pw_fired) {
		pw_fired = true;
		if (!off_latched) {
			off_latched = true;
			off_since = now_tick;
		}
	}
}

static void down_sample(bool pressed, bool walk_active)
{
	if (dn.state != PB_HELD) {
		(void)machine_common(&dn, pressed);
		return;
	}
	if (dn.hold < 0xFFFFU) dn.hold++;
	if (dn.hold > PA4_LONG_COUNT && walk_active) {
		/* Walk running for ~1.6 s: bridge a dropout of up to 250 one-millisecond samples. */
		if (pressed) dn_bridge = PA4_WALK_BRIDGE_SAMPLES;
		else if (dn_bridge > 0U) dn_bridge--;
		else dn.state = PB_GAP1;
	} else {
		dn.state = pressed ? PB_HELD : PB_GAP1;
	}
}

void pa4_buttons_update(uint32_t now_tick, uint16_t pa4_raw, bool walk_active)
{
	const bool power_pressed = pa4_raw >= PA4_POWER_MIN && pa4_raw <= PA4_POWER_MAX;
	const bool down_pressed = pa4_raw >= PA4_DOWN_MIN && pa4_raw <= PA4_DOWN_MAX;

	if (sample_due(&pw, now_tick, PA4_SAMPLE_TICKS)) power_sample(power_pressed, now_tick);

	/* The original samples the down machine every call (1 ms) while Walk may be bridged. */
	const bool fast = dn.state == PB_HELD && dn.hold >= PA4_LONG_COUNT && walk_active;
	if (sample_due(&dn, now_tick, fast ? PA4_FAST_SAMPLE_TICKS : PA4_SAMPLE_TICKS)) {
		down_sample(down_pressed, walk_active);
	}

	if (off_latched && (uint32_t)(now_tick - off_since) > PA4_POWER_OFF_DELAY_TICKS) off_due = true;
}

bool pa4_buttons_power_off_due(void)   { return off_due; }
bool pa4_buttons_power_pressed(void)   { return pw.state >= PB_HELD; }
bool pa4_buttons_walk_held(void)       { return dn.state == PB_HELD; }
bool pa4_buttons_any_activity(void)    { return pw.state != PB_IDLE || dn.state != PB_IDLE; }
uint8_t pa4_buttons_power_state(void)  { return pw.state; }
uint16_t pa4_buttons_power_hold(void)  { return pw.hold; }
bool pa4_buttons_power_off_latched(void) { return off_latched; }
uint8_t pa4_buttons_down_state(void)   { return dn.state; }
uint16_t pa4_buttons_down_hold(void)   { return dn.hold; }
