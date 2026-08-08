/*
 * FW-100 host harness — runs the SHIPPED assist_extended_boost.c, not a copy of it.
 *
 * Why this exists: tests/fw100_extended_boost.js re-implements the state machine in
 * JavaScript. That catches design mistakes but proves nothing about the C that goes on the
 * bike — a port can agree with a model while the compiled module does something else, and it
 * cannot see integer promotion, an uninitialized field or a changed constant at all.
 *
 * Build and run (any host compiler; see run-host-tests.ps1):
 *   gcc -std=c11 -Wall -Wextra -I../../inc -o fw100 fw100_extended_boost_host.c \
 *       ../../src/assist_extended_boost.c && ./fw100
 *
 * The module is deliberately free of MS/MP and of any external call, so it links with no
 * stubs at all. Keep it that way.
 *
 * THIS FEATURE DRIVES THE MOTOR WITH THE CRANKS STATIONARY, on a bike whose brake input is
 * the only independent way to stop it. So most of what follows checks that it says NO: a
 * spike must not arm it, a stale arming must not fire, the timer must not be refreshed, and
 * every cancel must act in the same control tick.
 */

#include <stdio.h>
#include <string.h>

#include "assist_extended_boost.h"

static int failures;

static void check(int ok, const char *label)
{
	if (!ok) {
		failures++;
		printf("  FAIL  %s\n", label);
	}
}

#define IQ_LIMIT 3000
#define LAST_IQ  120            /* what the rider was getting before the cranks stopped */
#define CONFIRM_TICKS (EXT_BOOST_CONFIRM_MS * EXT_BOOST_CONTROL_TICKS_PER_MS)
#define ARM_TIMEOUT_TICKS (EXT_BOOST_ARM_TIMEOUT_MS * EXT_BOOST_CONTROL_TICKS_PER_MS)
#define TRIGGER 2000U
#define HARD_PUSH (TRIGGER + 1000U)
#define DURATION_MS 500U
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

/* A rider pedalling normally, with assist already flowing at LAST_IQ. */
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
	in.last_pedal_iq = LAST_IQ;
	return in;
}

/* The same bike with the cranks stopped — the state the boost runs in. */
static assist_extended_boost_input_t cranks_stopped(void)
{
	assist_extended_boost_input_t in = riding(0);
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
	assist_extended_boost_diag_t d;
	assist_extended_boost_get_diag(&d);
	return d.state;
}

static uint8_t cancel_now(void)
{
	assist_extended_boost_diag_t d;
	assist_extended_boost_get_diag(&d);
	return d.cancel_reason;
}

/* Arm with a held push, then stop the cranks. Returns the first boosting tick. */
static assist_extended_boost_output_t arm_and_stop(const assist_extended_boost_config_t *cfg,
	uint16_t load)
{
	assist_extended_boost_input_t in = riding(load);
	for (unsigned t = 0; t < CONFIRM_TICKS; t++) {
		tick(&in, cfg);
	}
	assist_extended_boost_input_t stopped = cranks_stopped();
	return tick(&stopped, cfg);
}

int main(void)
{
	const assist_extended_boost_config_t cfg = config(TRIGGER, 100, DURATION_MS);

	printf("FW-100 Extended Boost, against the shipped C module\n");
	printf("  confirm %u ms, arm timeout %u ms, duration %u ms, ceiling %u ms\n",
		(unsigned)EXT_BOOST_CONFIRM_MS, (unsigned)EXT_BOOST_ARM_TIMEOUT_MS,
		(unsigned)DURATION_MS, (unsigned)ASSIST_EXT_BOOST_DURATION_MAX_MS);

	/* --- it holds what the rider already had ---------------------------------------- */
	{
		assist_extended_boost_init();
		assist_extended_boost_output_t first = arm_and_stop(&cfg, HARD_PUSH);
		check(first.active, "stopping the cranks after a confirmed push starts the boost");
		check(first.iq_target == LAST_IQ,
			"at 100 % the held current is exactly the last pedal request");
		check(state_now() == ASSIST_EXT_BOOST_ACTIVE, "state is ACTIVE while holding");

		assist_extended_boost_input_t stopped = cranks_stopped();
		int ran = 1;
		for (unsigned t = 0; t < DURATION_TICKS + 10U; t++) {
			assist_extended_boost_output_t o = tick(&stopped, &cfg);
			if (!o.active) {
				break;
			}
			check(o.iq_target == LAST_IQ, "the held current does not drift");
			ran++;
		}
		check(ran == (int)DURATION_TICKS, "the boost runs for exactly the configured time");
		check(cancel_now() == ASSIST_EXT_BOOST_CANCEL_COMPLETED,
			"a boost that ran its time reports COMPLETED");
	}

	/* --- the trigger only ARMS: it does not change the current ---------------------- */
	{
		assist_extended_boost_init();
		int32_t hard = arm_and_stop(&cfg, HARD_PUSH).iq_target;
		assist_extended_boost_init();
		int32_t gentle = arm_and_stop(&cfg, TRIGGER).iq_target;
		check(hard == gentle && hard == LAST_IQ,
			"a harder arming push gives the SAME current");

		assist_extended_boost_init();
		const assist_extended_boost_config_t strong = config(TRIGGER, 150, DURATION_MS);
		int32_t scaled = arm_and_stop(&strong, HARD_PUSH).iq_target;
		check(scaled == ((LAST_IQ * 150) + 50) / 100, "strength_pct scales the held current");
	}

	/* --- resuming pedalling hands back at once -------------------------------------- */
	{
		assist_extended_boost_init();
		check(arm_and_stop(&cfg, HARD_PUSH).active, "precondition: boost running");
		assist_extended_boost_input_t stopped = cranks_stopped();
		for (int t = 0; t < 20; t++) {
			tick(&stopped, &cfg);
		}
		assist_extended_boost_input_t back = riding(0);
		assist_extended_boost_output_t o = tick(&back, &cfg);
		check(!o.active && o.iq_target == 0,
			"pedalling again ends the boost in the same tick");
		check(cancel_now() == ASSIST_EXT_BOOST_CANCEL_PEDALING_RESUMED,
			"and reports PEDALING_RESUMED");
	}

	/* --- every hard condition stops it in the same tick ----------------------------- */
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
			check(arm_and_stop(&cfg, HARD_PUSH).active, "precondition: boost running");
			assist_extended_boost_input_t in = cranks_stopped();
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
			snprintf(label, sizeof(label), "%s reports its own reason", cases[i].name);
			check(cancel_now() == cases[i].expect, label);
		}
	}

	/* --- the checks that say NO ------------------------------------------------------ */
	{
		/* A spike shorter than the confirm time never arms. */
		assist_extended_boost_init();
		assist_extended_boost_input_t push = riding(HARD_PUSH);
		for (unsigned t = 0; t < CONFIRM_TICKS - 1U; t++) {
			tick(&push, &cfg);
		}
		assist_extended_boost_input_t stopped = cranks_stopped();
		int fired = 0;
		for (unsigned t = 0; t < DURATION_TICKS; t++) {
			if (tick(&stopped, &cfg).active) {
				fired = 1;
			}
		}
		check(!fired, "a push shorter than the confirm time never fires");

		/* A stale arming expires instead of firing later. */
		assist_extended_boost_init();
		for (unsigned t = 0; t < CONFIRM_TICKS; t++) {
			tick(&push, &cfg);
		}
		check(state_now() == ASSIST_EXT_BOOST_ARMED, "precondition: armed");
		assist_extended_boost_input_t light = riding(0);
		for (unsigned t = 0; t < ARM_TIMEOUT_TICKS + 2U; t++) {
			tick(&light, &cfg);
		}
		check(cancel_now() == ASSIST_EXT_BOOST_CANCEL_ARM_TIMEOUT,
			"an arming that waits too long expires");
		int late = 0;
		for (unsigned t = 0; t < DURATION_TICKS; t++) {
			if (tick(&stopped, &cfg).active) {
				late = 1;
			}
		}
		check(!late, "and cannot be cashed in afterwards");

		/* Off by default. */
		assist_extended_boost_init();
		const assist_extended_boost_config_t off = config(TRIGGER, 100, 0);
		int any = 0;
		for (unsigned t = 0; t < CONFIRM_TICKS; t++) {
			tick(&push, &off);
		}
		for (int t = 0; t < 100; t++) {
			if (tick(&stopped, &off).active) {
				any = 1;
			}
		}
		check(!any, "duration 0 means the feature does nothing at all");

		/* Nothing was flowing, so there is nothing to hold. */
		assist_extended_boost_init();
		assist_extended_boost_input_t no_assist = riding(HARD_PUSH);
		no_assist.last_pedal_iq = 0;
		for (unsigned t = 0; t < CONFIRM_TICKS; t++) {
			tick(&no_assist, &cfg);
		}
		assist_extended_boost_input_t stopped_zero = cranks_stopped();
		stopped_zero.last_pedal_iq = 0;
		check(!tick(&stopped_zero, &cfg).active,
			"with no current flowing beforehand there is nothing to hold");

		/* Nothing may refresh the timer. */
		assist_extended_boost_init();
		check(arm_and_stop(&cfg, HARD_PUSH).active, "precondition: boost running");
		assist_extended_boost_input_t still_loaded = cranks_stopped();
		still_loaded.pedal_load_centikg = HARD_PUSH;
		int ticks = 1;
		while (tick(&still_loaded, &cfg).active) {
			ticks++;
			if (ticks > (int)DURATION_TICKS * 4) {
				break;
			}
		}
		check(ticks == (int)DURATION_TICKS,
			"load still on the pedal does not extend the timer");
	}

	if (failures == 0) {
		printf("All FW-100 host checks passed.\n");
		return 0;
	}
	printf("\n%d FW-100 host check(s) FAILED.\n", failures);
	return 1;
}
