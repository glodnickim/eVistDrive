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
static bool     pw_boot_guard;     /* a press present since power-on has not ended yet */
static uint16_t dn_bridge;         /* remaining 1 ms samples of the Walk dropout bridge */
static bool     off_latched;
static uint32_t off_since;
static bool     off_due;

/* Button circuit test (original 0x08017328) and error 36 flags (original 0x20000354). */
static uint32_t probe_last;        /* 1 ms step clock of the test                       */
static uint8_t  probe_step;        /* original 0x20000356: 0 = PB8 up, >5 = PB8 down    */
static uint8_t  probe_fail;        /* original 0x20000355: 0..2                         */
static bool     probe_pb8;
static bool     fault_circuit;     /* bit0: PA4 did not follow PB8                      */
static bool     fault_stuck;       /* bit1: on/off held > 800 samples (6.4 s)           */
static uint32_t pw_samples;        /* diagnostics: on/off samples taken                 */

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
	pw_boot_guard = true;
	dn_bridge = 0U;
	off_latched = false;
	off_since = now_tick;
	off_due = false;
	probe_last = now_tick;
	probe_step = 0U;
	probe_fail = 0U;
	probe_pb8 = false;
	fault_circuit = false;
	fault_stuck = false;
	pw_samples = 0U;
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
	pw_samples++;
	if (pw.state != PB_HELD) {
		/* The press the bike was switched on with is consumed, as the original's power-on
		 * event consumes it: it may not switch the bike off again. The guard ends once the
		 * machine has been idle with the button out of the window. */
		if (pw_boot_guard && pw.state == PB_IDLE && !pressed) pw_boot_guard = false;
		if (machine_common(&pw, pressed)) {
			pw_fired = false;
			fault_stuck = false;               /* original 0x08017930 */
		}
		if (pw.state == PB_HELD && pw_boot_guard) pw_fired = true;
		return;
	}
	/* As in the original: the count advances on this sample even if it is the first one outside
	 * the window, and the long-press test follows the state change. */
	if (pw.hold < 0xFFFFU) pw.hold++;
	pw.state = pressed ? PB_HELD : PB_GAP1;
	if (pw.hold == PA4_LONG_COUNT) {
		if (!pw_fired) {
			pw_fired = true;
			if (!off_latched) {
				off_latched = true;
				off_since = now_tick;
			}
		}
	} else if (pw.hold > PA4_STUCK_COUNT) {
		fault_stuck = true;                    /* original 0x08017900 */
	}
}

/* One 1 ms step of the original button circuit test. Returns true while the test owns the line
 * (PB8 up or settling) - the button machines do not sample then. */
static bool probe_step_1ms(uint16_t pa4_raw)
{
	if (probe_step == 0U) {
		probe_pb8 = true;
	} else if (probe_step > 1U && probe_step < 5U) {
		if (pa4_raw < PA4_PROBE_MAX) {
			if (probe_fail > 0U) probe_fail--;
			else fault_circuit = false;
		} else {
			if (probe_fail < 2U) probe_fail++;
			else fault_circuit = true;
		}
	}
	probe_step++;                              /* uint8_t: wraps, a new test every 256 ms */
	if (probe_step > 5U) probe_pb8 = false;
	return probe_step <= 5U;
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

	/* Button circuit test: only while both buttons are idle; a press freezes it and clears the
	 * failure count (original 0x080176ca..0x080176ee). */
	bool probing = false;
	if ((uint32_t)(now_tick - probe_last) >= PA4_FAST_SAMPLE_TICKS) {
		probe_last += PA4_FAST_SAMPLE_TICKS;
		if ((uint32_t)(now_tick - probe_last) >= PA4_FAST_SAMPLE_TICKS) probe_last = now_tick;
		if (pw.state == PB_IDLE && dn.state == PB_IDLE) probing = probe_step_1ms(pa4_raw);
		else probe_fail = 0U;
	} else {
		probing = probe_pb8 || probe_step <= 5U;
	}
	if (probing && pw.state == PB_IDLE && dn.state == PB_IDLE) {
		/* the machines skip their samples while PB8 owns the line */
		pw.last_sample = now_tick;
		dn.last_sample = now_tick;
	} else {
		if (sample_due(&pw, now_tick, PA4_SAMPLE_TICKS)) power_sample(power_pressed, now_tick);

		/* The original samples the down machine every call (1 ms) while Walk may be bridged. */
		const bool fast = dn.state == PB_HELD && dn.hold >= PA4_LONG_COUNT && walk_active;
		if (sample_due(&dn, now_tick, fast ? PA4_FAST_SAMPLE_TICKS : PA4_SAMPLE_TICKS)) {
			down_sample(down_pressed, walk_active);
		}
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
bool pa4_buttons_pb8_high(void)        { return probe_pb8; }
bool pa4_buttons_fault(void)           { return fault_circuit || fault_stuck; }
bool pa4_buttons_fault_circuit(void)   { return fault_circuit; }
bool pa4_buttons_fault_stuck(void)     { return fault_stuck; }
uint32_t pa4_buttons_power_samples(void) { return pw_samples; }
