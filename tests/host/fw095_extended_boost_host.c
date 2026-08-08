/*
 * FW-095 host harness — runs the SHIPPED assist_extended_boost.c, not a copy of it.
 *
 * Why this exists: tests/fw095_extended_boost_safety.js re-implements the state machine in
 * JavaScript. That catches design mistakes but proves nothing about the C that goes on the
 * bike — a port can agree with a model while the compiled module does something else, and it
 * cannot see integer promotion, an uninitialized field or a changed constant at all.
 *
 * Build and run (any host compiler; see run-host-tests.ps1):
 *   gcc -std=c11 -Wall -Wextra -I../../inc -o fw095 fw095_extended_boost_host.c \
 *       ../../src/assist_extended_boost.c && ./fw095
 *
 * The module is deliberately free of MS/MP and of any external call, so it links with no
 * stubs at all. Keep it that way: the moment this harness needs a stub, the module has
 * grown a dependency that also makes it harder to reason about on the bike.
 *
 * THE TESTS THAT MATTER MOST are the ones that say NO. This feature adds motor current on a
 * bike with no dependable brake-sensor input, so the assertions below spend most of their
 * effort proving that it STOPS: on a real PAS STOP, on losing the latch, on a hard cut, on
 * reverse, on a dead sensor, and that a held push cannot chain one boost into the next.
 */

#include <stdio.h>
#include <string.h>

#include "assist_extended_boost.h"
#include "torque_input.h"

static int failures;

static void check(int ok, const char *label)
{
	if (!ok) {
		failures++;
		printf("  FAIL  %s\n", label);
	}
}

#define IQ_LIMIT 3000
#define CONFIRM_TICKS (EXT_BOOST_CONFIRM_MS * EXT_BOOST_CONTROL_TICKS_PER_MS)
#define TRIGGER 2000U            /* 20.0 kg, the shipped default */
#define HARD_PUSH (TRIGGER + 1000U)
#define DURATION_MS 200U
#define DURATION_TICKS (DURATION_MS * EXT_BOOST_CONTROL_TICKS_PER_MS)

static assist_extended_boost_config_t config(uint16_t trigger, uint8_t strength,
	uint16_t duration)
{
	assist_extended_boost_config_t cfg;
	cfg.trigger_load_centikg = trigger;
	cfg.strength_pct = strength;
	cfg.duration_ms = duration;
	return cfg;
}

/* A rider pedalling normally with the assist latch armed and the bike moving. */
static assist_extended_boost_input_t riding(uint16_t load_centikg)
{
	assist_extended_boost_input_t in;
	memset(&in, 0, sizeof(in));
	in.pedaling_active = true;
	in.pedal_assist_latched = true;
	in.motion_valid = true;
	in.torque_sensor_valid = true;
	in.pas_sensor_valid = true;
	in.bank_index = 0;
	in.level_index = 3;
	in.pedal_load_centikg = load_centikg;
	in.ride_core_iq_limit = IQ_LIMIT;
	return in;
}

/* The same bike with the cranks genuinely stopped: the case this card is about. */
static assist_extended_boost_input_t cranks_stopped(uint16_t load_centikg)
{
	assist_extended_boost_input_t in = riding(load_centikg);
	in.pedaling_active = false;
	in.pedal_assist_latched = false;
	return in;
}

static assist_extended_boost_output_t tick(const assist_extended_boost_input_t *in,
	const assist_extended_boost_config_t *cfg)
{
	assist_extended_boost_output_t out;
	assist_extended_boost_update(in, cfg, &out);
	return out;
}

static uint8_t state_now(void)
{
	assist_extended_boost_diag_t diag;
	assist_extended_boost_get_diag(&diag);
	return diag.state;
}

static uint8_t cancel_now(void)
{
	assist_extended_boost_diag_t diag;
	assist_extended_boost_get_diag(&diag);
	return diag.cancel_reason;
}

/* Push hard while pedalling until the boost is running. Returns the tick it started on. */
static int start_boost(const assist_extended_boost_config_t *cfg)
{
	assist_extended_boost_input_t in = riding(HARD_PUSH);
	for (int t = 1; t <= (int)CONFIRM_TICKS + 4; t++) {
		assist_extended_boost_output_t out = tick(&in, cfg);
		if (out.active) {
			return t;
		}
	}
	return -1;
}

int main(void)
{
	const assist_extended_boost_config_t cfg = config(TRIGGER, 100, DURATION_MS);

	printf("FW-095 Extended Boost safety, against the shipped C module\n");
	printf("  confirm %u ms, duration %u ms, trigger %u.%u kg, hysteresis %u.%02u kg\n",
		(unsigned)EXT_BOOST_CONFIRM_MS, (unsigned)DURATION_MS,
		(unsigned)(TRIGGER / 100U), (unsigned)(TRIGGER % 100U) / 10U,
		(unsigned)(EXT_BOOST_RELEASE_HYST_CENTIKG / 100U),
		(unsigned)(EXT_BOOST_RELEASE_HYST_CENTIKG % 100U));

	/* --- Test 2: the boost runs during a confirmed push, while pedalling ------------- */
	{
		assist_extended_boost_init();
		int started = start_boost(&cfg);
		check(started == (int)CONFIRM_TICKS,
			"starts exactly on the confirm tick, not before and not later");
		check(state_now() == ASSIST_EXT_BOOST_ACTIVE, "state is ACTIVE while boosting");

		/* It keeps paying out for the configured time WHILE PEDALLING CONTINUES. */
		assist_extended_boost_input_t in = riding(HARD_PUSH);
		int ran = 1;
		for (int t = 0; t < (int)DURATION_TICKS + 10; t++) {
			assist_extended_boost_output_t out = tick(&in, &cfg);
			if (!out.active) {
				break;
			}
			check(out.iq_target > 0, "an active boost always asks for current");
			check(out.iq_target <= IQ_LIMIT, "the boost never exceeds the level limit");
			ran++;
		}
		check(ran == (int)DURATION_TICKS,
			"the boost runs for exactly the configured duration");
		check(cancel_now() == ASSIST_EXT_BOOST_CANCEL_COMPLETED,
			"a boost that ran its time reports COMPLETED");
	}

	/* --- Test 3: PAS STOP. THE safety test of this card ------------------------------ */
	{
		assist_extended_boost_init();
		check(start_boost(&cfg) > 0, "precondition: a boost is running");

		/*
		 * The cranks stop with the rider's weight still on the pedal — the worst case,
		 * because the load alone would still satisfy the trigger. The boost must stop
		 * anyway, in this very tick, with plenty of timer left.
		 */
		assist_extended_boost_input_t stopped = cranks_stopped(HARD_PUSH);
		assist_extended_boost_output_t out = tick(&stopped, &cfg);
		check(!out.active, "PAS STOP ends the boost in the same control tick");
		check(out.iq_target == 0, "PAS STOP yields zero boost current immediately");
		check(cancel_now() == ASSIST_EXT_BOOST_CANCEL_PEDALING_STOPPED,
			"PAS STOP is reported as such, not as COMPLETED");

		/* And it stays off for as long as the cranks are stopped. */
		int drove_after_stop = 0;
		for (int t = 0; t < (int)DURATION_TICKS * 4; t++) {
			assist_extended_boost_output_t o = tick(&stopped, &cfg);
			if (o.active || o.iq_target != 0) {
				drove_after_stop = 1;
			}
		}
		check(!drove_after_stop,
			"no motor overrun: nothing is produced after the cranks stop");
	}

	/* --- The removed FW-084 trigger must not come back ------------------------------- */
	{
		assist_extended_boost_init();
		/* Qualify a push, then stop pedalling. Under FW-084 THIS is what started the
		 * boost. It must now do the exact opposite and produce nothing at all. */
		assist_extended_boost_input_t in = riding(HARD_PUSH);
		for (unsigned t = 0; t < CONFIRM_TICKS - 1U; t++) {
			tick(&in, &cfg);
		}
		assist_extended_boost_input_t stopped = cranks_stopped(0);
		int fired = 0;
		for (int t = 0; t < (int)DURATION_TICKS * 4; t++) {
			assist_extended_boost_output_t o = tick(&stopped, &cfg);
			if (o.active || o.iq_target != 0) {
				fired = 1;
			}
		}
		check(!fired, "the pedal-stop EDGE never starts a boost any more");
	}

	/* --- Losing the ride latch mid-boost stops it too -------------------------------- */
	{
		assist_extended_boost_init();
		check(start_boost(&cfg) > 0, "precondition: a boost is running");
		assist_extended_boost_input_t unlatched = riding(HARD_PUSH);
		unlatched.pedal_assist_latched = false;
		assist_extended_boost_output_t out = tick(&unlatched, &cfg);
		check(!out.active && out.iq_target == 0,
			"losing the ride latch stops the boost immediately");
	}

	/* --- Test 5/6/7 + reverse: every hard condition stops it in the same tick -------- */
	{
		struct { const char *name; int which; uint8_t expect; } cases[] = {
			{ "brake / hard cut", 0, ASSIST_EXT_BOOST_CANCEL_SAFETY_CUT },
			{ "backward pedalling", 1, ASSIST_EXT_BOOST_CANCEL_REVERSE },
			{ "torque sensor fault", 2, ASSIST_EXT_BOOST_CANCEL_SENSOR_INVALID },
			{ "PAS sensor fault", 3, ASSIST_EXT_BOOST_CANCEL_SENSOR_INVALID },
			{ "walk assist", 4, ASSIST_EXT_BOOST_CANCEL_WALK },
			{ "position calibration", 5, ASSIST_EXT_BOOST_CANCEL_CALIBRATION },
			{ "assist level 0", 6, ASSIST_EXT_BOOST_CANCEL_LEVEL_OR_BANK_CHANGE },
			{ "bike no longer moving", 7, ASSIST_EXT_BOOST_CANCEL_MOTION_LOST },
		};
		for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
			assist_extended_boost_init();
			check(start_boost(&cfg) > 0, "precondition: a boost is running");
			assist_extended_boost_input_t in = riding(HARD_PUSH);
			switch (cases[i].which) {
			case 0: in.safety_cut = true; break;
			case 1: in.crank_reverse = true; break;
			case 2: in.torque_sensor_valid = false; break;
			case 3: in.pas_sensor_valid = false; break;
			case 4: in.walk_active = true; break;
			case 5: in.position_calibration_active = true; break;
			case 6: in.level_index = 0; break;
			case 7: in.motion_valid = false; break;
			default: break;
			}
			assist_extended_boost_output_t out = tick(&in, &cfg);
			char label[96];
			snprintf(label, sizeof(label), "%s stops the boost in the same tick",
				cases[i].name);
			check(!out.active && out.iq_target == 0, label);
			snprintf(label, sizeof(label), "%s is reported with its own reason",
				cases[i].name);
			check(cancel_now() == cases[i].expect, label);
		}
	}

	/* --- A held push cannot chain boosts back to back ------------------------------- */
	{
		assist_extended_boost_init();
		check(start_boost(&cfg) > 0, "precondition: a boost is running");
		assist_extended_boost_input_t in = riding(HARD_PUSH);
		int active_ticks = 0;
		for (int t = 0; t < (int)DURATION_TICKS * 5; t++) {
			if (tick(&in, &cfg).active) {
				active_ticks++;
			}
		}
		check(active_ticks < (int)DURATION_TICKS,
			"holding the push does not restart the boost for ever");

		/* Easing off below the hysteresis and pushing again gives a new boost. */
		assist_extended_boost_input_t easy = riding(TRIGGER - EXT_BOOST_RELEASE_HYST_CENTIKG - 1U);
		for (int t = 0; t < 8; t++) {
			tick(&easy, &cfg);
		}
		int restarted = 0;
		assist_extended_boost_input_t again = riding(HARD_PUSH);
		for (unsigned t = 0; t < CONFIRM_TICKS + 4U; t++) {
			if (tick(&again, &cfg).active) {
				restarted = 1;
			}
		}
		check(restarted, "a fresh push after easing off does start a new boost");
	}

	/* --- One push, one boost, ACROSS a stop-start with the pedal never released ------ */
	{
		/*
		 * PAS STOP cancels an ACTIVE boost correctly, but the PUSH never ended. If that
		 * cancel does not block re-arming, resuming the cranks under the same unbroken
		 * press confirms a fresh window 30 ms later and pays out a second boost. Not a
		 * post-PAS safety hole — the second one still needs live pedalling — but it
		 * breaks the promise the rider is given.
		 */
		assist_extended_boost_init();
		check(start_boost(&cfg) > 0, "precondition: a boost is running");

		assist_extended_boost_input_t stop_full_load = cranks_stopped(HARD_PUSH);
		tick(&stop_full_load, &cfg);
		check(cancel_now() == ASSIST_EXT_BOOST_CANCEL_PEDALING_STOPPED,
			"precondition: cancelled by PAS STOP");
		for (int t = 0; t < 40; t++) {
			tick(&stop_full_load, &cfg);
		}

		assist_extended_boost_input_t resumed = riding(HARD_PUSH);
		int second = 0;
		for (unsigned t = 0; t < CONFIRM_TICKS * 3U; t++) {
			if (tick(&resumed, &cfg).active) {
				second = 1;
			}
		}
		check(!second,
			"a stop-start under one unbroken push does not pay out a second boost");

		assist_extended_boost_input_t released =
			riding(TRIGGER - EXT_BOOST_RELEASE_HYST_CENTIKG - 1U);
		for (int t = 0; t < 8; t++) {
			tick(&released, &cfg);
		}
		int after_release = 0;
		for (unsigned t = 0; t < CONFIRM_TICKS + 4U; t++) {
			if (tick(&resumed, &cfg).active) {
				after_release = 1;
			}
		}
		check(after_release,
			"easing off and pushing again is what gives the next boost");
	}

	/* --- What is and is not re-checked while the boost runs -------------------------- */
	{
		/*
		 * The pedal load is NOT re-tested once the boost has been triggered. Deliberate: a
		 * pedal stroke has dead spots, and re-testing would make the boost stutter at
		 * exactly the cadence it exists to help.
		 */
		assist_extended_boost_init();
		check(start_boost(&cfg) > 0, "precondition: a boost is running");
		assist_extended_boost_input_t no_load = riding(0);
		int ran = 0;
		for (int t = 0; t < (int)DURATION_TICKS + 10; t++) {
			if (!tick(&no_load, &cfg).active) {
				break;
			}
			ran++;
		}
		check(ran == (int)DURATION_TICKS - 1,
			"easing off after the trigger does not cut the boost short");
	}

	/* --- Off by default, and a short spike is not a decision ------------------------- */
	{
		assist_extended_boost_init();
		const assist_extended_boost_config_t off = config(TRIGGER, 100, 0);
		assist_extended_boost_input_t in = riding(HARD_PUSH);
		int any = 0;
		for (unsigned t = 0; t < CONFIRM_TICKS * 4U; t++) {
			if (tick(&in, &off).active) {
				any = 1;
			}
		}
		check(!any, "duration 0 means the feature does nothing at all");

		assist_extended_boost_init();
		int spike_fired = 0;
		for (unsigned t = 0; t < CONFIRM_TICKS - 1U; t++) {
			if (tick(&in, &cfg).active) {
				spike_fired = 1;
			}
		}
		assist_extended_boost_input_t light = riding(0);
		for (int t = 0; t < 40; t++) {
			if (tick(&light, &cfg).active) {
				spike_fired = 1;
			}
		}
		check(!spike_fired, "a push shorter than the confirm time never starts a boost");
	}

	/* --- The current comes from how far ABOVE the trigger the push went -------------- */
	{
		assist_extended_boost_init();
		assist_extended_boost_input_t small = riding(TRIGGER + 100U);
		int32_t small_iq = 0;
		for (unsigned t = 0; t < CONFIRM_TICKS; t++) {
			assist_extended_boost_output_t o = tick(&small, &cfg);
			if (o.active) {
				small_iq = o.iq_target;
			}
		}
		assist_extended_boost_init();
		assist_extended_boost_input_t big = riding(TRIGGER + 2000U);
		int32_t big_iq = 0;
		for (unsigned t = 0; t < CONFIRM_TICKS; t++) {
			assist_extended_boost_output_t o = tick(&big, &cfg);
			if (o.active) {
				big_iq = o.iq_target;
			}
		}
		check(small_iq > 0 && big_iq > small_iq,
			"a harder push above the trigger yields more current");
		check(big_iq <= IQ_LIMIT, "and never more than the level's own limit");

		/* A trigger at full scale can never be exceeded, so it never fires. */
		assist_extended_boost_init();
		const assist_extended_boost_config_t at_max =
			config(ASSIST_EXT_BOOST_TRIGGER_MAX_CENTIKG, 100, DURATION_MS);
		assist_extended_boost_input_t full = riding(ASSIST_EXT_BOOST_TRIGGER_MAX_CENTIKG);
		int max_fired = 0;
		for (unsigned t = 0; t < CONFIRM_TICKS * 3U; t++) {
			if (tick(&full, &at_max).active) {
				max_fired = 1;
			}
		}
		check(!max_fired, "a trigger at full scale never fires (and never divides by zero)");
	}

	if (failures == 0) {
		printf("All FW-095 host checks passed.\n");
		return 0;
	}
	printf("\n%d FW-095 host check(s) FAILED.\n", failures);
	return 1;
}
