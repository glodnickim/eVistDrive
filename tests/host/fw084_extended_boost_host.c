/*
 * FW-084 host harness — runs the SHIPPED assist_extended_boost.c, not a copy of it.
 *
 * Why this exists: tests/fw084_extended_boost.js re-implements the state machine in
 * JavaScript. That catches design mistakes but proves nothing about the C that goes on the
 * bike — a port can agree with a model while the compiled module does something else, and it
 * cannot see integer promotion, an uninitialized field or a changed constant at all.
 *
 * Build and run (any host compiler; see run-host-tests.ps1):
 *   gcc -std=c11 -Wall -Wextra -I../../inc -o fw084 fw084_extended_boost_host.c \
 *       ../../src/assist_extended_boost.c && ./fw084
 *
 * The module is deliberately free of MS/MP and of any external call, so it links with no
 * stubs at all. Keep it that way: the moment this harness needs a stub, the module has
 * grown a dependency that also makes it harder to reason about on the bike.
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
#define ARM_TIMEOUT_TICKS (EXT_BOOST_ARM_TIMEOUT_MS * EXT_BOOST_CONTROL_TICKS_PER_MS)

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

/* The same bike one tick later, with the cranks stopped. */
static assist_extended_boost_input_t coasting(void)
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

/* Hold a load for n ticks and report whether the module ended up armed. */
static void push(const assist_extended_boost_config_t *cfg, uint16_t load, int ticks)
{
	assist_extended_boost_input_t in = riding(load);
	for (int t = 0; t < ticks; t++) {
		tick(&in, cfg);
	}
}

static bool is_armed(void)
{
	assist_extended_boost_diag_t diag;
	assist_extended_boost_get_diag(&diag);
	return diag.state == ASSIST_EXT_BOOST_ARMED;
}

/* Arm, then stop pedalling. Returns how many ticks reported ACTIVE and the highest target. */
static void boost_run(const assist_extended_boost_config_t *cfg, uint16_t load,
	int after_ticks, int *active_ticks, int32_t *peak_iq)
{
	assist_extended_boost_init();
	push(cfg, load, CONFIRM_TICKS);
	assist_extended_boost_input_t stopped = coasting();
	*active_ticks = 0;
	*peak_iq = 0;
	for (int t = 0; t < after_ticks; t++) {
		assist_extended_boost_output_t out = tick(&stopped, cfg);
		if (out.active) {
			(*active_ticks)++;
			if (out.iq_target > *peak_iq) {
				*peak_iq = out.iq_target;
			}
		}
	}
}

/* The reference arithmetic from the card, independent of the module's own code. */
static int32_t reference_iq(uint32_t peak, uint32_t trigger, uint32_t pct, int32_t limit)
{
	uint32_t span, above, base, scaled;
	if (peak > TORQUE_PUBLIC_FULL_SCALE_CENTIKG) {
		peak = TORQUE_PUBLIC_FULL_SCALE_CENTIKG;
	}
	if (peak <= trigger) {
		return 0;
	}
	span = TORQUE_PUBLIC_FULL_SCALE_CENTIKG - trigger;
	above = peak - trigger;
	base = ((uint32_t)limit * above + span / 2U) / span;
	scaled = (base * pct + 50U) / 100U;
	return (scaled > (uint32_t)limit) ? limit : (int32_t)scaled;
}

int main(void)
{
	assist_extended_boost_config_t cfg = config(800, 100, 200);
	int active;
	int32_t peak;

	printf("FW-084 host harness: real assist_extended_boost.c, %u ticks/ms\n",
		(unsigned)EXT_BOOST_CONTROL_TICKS_PER_MS);

	/* 1. Off by default. */
	{
		assist_extended_boost_config_t off = config(800, 100, 0);
		boost_run(&off, 2000, 4000, &active, &peak);
		check(active == 0, "1. duration 0 never drives");
		off = config(800, 0, 200);
		boost_run(&off, 2000, 4000, &active, &peak);
		check(active == 0, "1. strength 0 never drives");
	}

	/* 2/3. The load must clear the trigger AND be held. */
	{
		boost_run(&cfg, 799, 4000, &active, &peak);
		check(active == 0, "2. below the trigger nothing arms");

		assist_extended_boost_init();
		push(&cfg, 5000, CONFIRM_TICKS - 1);
		check(!is_armed(), "3. a spike one tick short does not arm");
		assist_extended_boost_init();
		push(&cfg, 5000, CONFIRM_TICKS);
		check(is_armed(), "4. exactly the required hold arms");
	}

	/* 5. Hysteresis ends a window; a dip inside it does not. */
	{
		assist_extended_boost_init();
		push(&cfg, 2000, CONFIRM_TICKS);
		push(&cfg, (uint16_t)(800 - EXT_BOOST_RELEASE_HYST_CENTIKG + 1), 50);
		check(is_armed(), "5. a dip inside the hysteresis keeps the arming");
	}

	/* 6/7. The latest confirmed push wins, even when it is weaker. */
	{
		assist_extended_boost_diag_t diag;
		assist_extended_boost_init();
		push(&cfg, 4000, CONFIRM_TICKS);
		assist_extended_boost_get_diag(&diag);
		check(diag.peak_load_centikg == 4000, "6. the window peak is stored");
		push(&cfg, 0, 200);
		push(&cfg, 1200, CONFIRM_TICKS);
		assist_extended_boost_get_diag(&diag);
		check(diag.peak_load_centikg == 1200,
			"7. a weaker later push replaces the earlier one");
	}

	/* 8. An arming goes stale and cannot be replayed. */
	{
		assist_extended_boost_diag_t diag;
		assist_extended_boost_input_t idle = riding(0);
		assist_extended_boost_init();
		push(&cfg, 4000, CONFIRM_TICKS);
		for (uint32_t t = 0; t < ARM_TIMEOUT_TICKS + 1U; t++) {
			tick(&idle, &cfg);
		}
		assist_extended_boost_get_diag(&diag);
		check(diag.state == ASSIST_EXT_BOOST_IDLE &&
			diag.cancel_reason == ASSIST_EXT_BOOST_CANCEL_ARM_TIMEOUT,
			"8. the arming expires and says so");
		assist_extended_boost_input_t stopped = coasting();
		check(!tick(&stopped, &cfg).active, "8. a stale arming cannot fire");
	}

	/* 9. ACTIVE starts on the edge only. */
	{
		assist_extended_boost_input_t held = riding(2000);
		assist_extended_boost_input_t stopped = coasting();
		assist_extended_boost_init();
		push(&cfg, 2000, CONFIRM_TICKS);
		check(!tick(&held, &cfg).active, "9. still pedalling: no boost");
		check(tick(&stopped, &cfg).active, "9. the boost starts on the pedal-stop edge");
	}

	/* 10. Exact durations. */
	{
		const uint16_t durations[] = { 1, 200, 1000 };
		for (unsigned i = 0; i < sizeof(durations) / sizeof(durations[0]); i++) {
			assist_extended_boost_config_t timed = config(800, 100, durations[i]);
			char label[80];
			boost_run(&timed, 2000, 5000, &active, &peak);
			snprintf(label, sizeof(label), "10. %u ms lasts %u ticks (got %d)",
				durations[i],
				(unsigned)(durations[i] * EXT_BOOST_CONTROL_TICKS_PER_MS),
				active);
			check(active == (int)(durations[i] * EXT_BOOST_CONTROL_TICKS_PER_MS),
				label);
		}
	}

	/* 11/12/13. The current formula, the cap and the extremes. */
	{
		const uint8_t strengths[] = { 100, 150, 255 };
		for (unsigned i = 0; i < sizeof(strengths) / sizeof(strengths[0]); i++) {
			assist_extended_boost_config_t scaled = config(800, strengths[i], 200);
			char label[96];
			boost_run(&scaled, 2000, 2000, &active, &peak);
			snprintf(label, sizeof(label), "11. %u %%: got %ld, reference %ld",
				strengths[i], (long)peak,
				(long)reference_iq(2000, 800, strengths[i], IQ_LIMIT));
			check(peak == reference_iq(2000, 800, strengths[i], IQ_LIMIT), label);
		}
		assist_extended_boost_config_t hard = config(800, 255, 200);
		boost_run(&hard, TORQUE_PUBLIC_FULL_SCALE_CENTIKG, 2000, &active, &peak);
		check(peak <= IQ_LIMIT, "12. the level current limit is never exceeded");
		boost_run(&cfg, 810, 2000, &active, &peak);
		check(peak * 20 < IQ_LIMIT, "11. touching the threshold gives almost nothing");
	}

	/* 14/15/16/17. Every cancel path, from ACTIVE. */
	{
		struct { const char *name; int which; uint8_t reason; } paths[] = {
			{ "safety cut", 0, ASSIST_EXT_BOOST_CANCEL_SAFETY_CUT },
			{ "reverse", 1, ASSIST_EXT_BOOST_CANCEL_REVERSE },
			{ "walk", 2, ASSIST_EXT_BOOST_CANCEL_WALK },
			{ "calibration", 3, ASSIST_EXT_BOOST_CANCEL_CALIBRATION },
			{ "torque sensor", 4, ASSIST_EXT_BOOST_CANCEL_SENSOR_INVALID },
			{ "pas sensor", 5, ASSIST_EXT_BOOST_CANCEL_SENSOR_INVALID },
			{ "level 0", 6, ASSIST_EXT_BOOST_CANCEL_LEVEL_OR_BANK_CHANGE },
			{ "bank change", 7, ASSIST_EXT_BOOST_CANCEL_LEVEL_OR_BANK_CHANGE },
			{ "motion lost", 8, ASSIST_EXT_BOOST_CANCEL_MOTION_LOST },
			{ "pedalling resumed", 9, ASSIST_EXT_BOOST_CANCEL_PEDALING_RESUMED },
		};
		for (unsigned i = 0; i < sizeof(paths) / sizeof(paths[0]); i++) {
			assist_extended_boost_diag_t diag;
			assist_extended_boost_input_t cut = coasting();
			char label[96];
			assist_extended_boost_init();
			push(&cfg, 2000, CONFIRM_TICKS);
			{
				assist_extended_boost_input_t stopped = coasting();
				check(tick(&stopped, &cfg).active, "14. boost is running");
			}
			switch (paths[i].which) {
			case 0: cut.safety_cut = true; break;
			case 1: cut.crank_reverse = true; break;
			case 2: cut.walk_active = true; break;
			case 3: cut.position_calibration_active = true; break;
			case 4: cut.torque_sensor_valid = false; break;
			case 5: cut.pas_sensor_valid = false; break;
			case 6: cut.level_index = 0; break;
			case 7: cut.bank_index = 1; break;
			case 8: cut.motion_valid = false; break;
			default: cut.pedaling_active = true; break;
			}
			assist_extended_boost_output_t out = tick(&cut, &cfg);
			assist_extended_boost_get_diag(&diag);
			snprintf(label, sizeof(label),
				"14. %s cancels in the same tick (reason %u)",
				paths[i].name, (unsigned)diag.cancel_reason);
			check(!out.active && diag.state == ASSIST_EXT_BOOST_IDLE &&
				diag.cancel_reason == paths[i].reason, label);
		}
	}

	/* 18. Nothing refreshes a running boost. */
	{
		assist_extended_boost_input_t stopped = coasting();
		assist_extended_boost_init();
		push(&cfg, 2000, CONFIRM_TICKS);
		active = 0;
		for (int t = 0; t < 4000; t++) {
			stopped.pedal_load_centikg = (t % 2) ? 3000 : 0;
			if (tick(&stopped, &cfg).active) {
				active++;
			}
		}
		check(active == 200 * (int)EXT_BOOST_CONTROL_TICKS_PER_MS,
			"18. load wobbling with the cranks stopped does not extend the boost");
	}

	/* 20. One contiguous hold, then the ordinary release owns the fade. */
	{
		assist_extended_boost_input_t stopped = coasting();
		int holds = 0, last_hold = -1;
		assist_extended_boost_init();
		push(&cfg, 2000, CONFIRM_TICKS);
		for (int t = 0; t < 2000; t++) {
			if (tick(&stopped, &cfg).profile_hold_active) {
				holds++;
				last_hold = t;
			}
		}
		check(holds == 200 * (int)EXT_BOOST_CONTROL_TICKS_PER_MS &&
			last_hold == holds - 1,
			"20. the profile hold covers exactly the boost, in one block");
	}

	/* P2 from the audit: remaining_ms must round UP, so a boost that is still ACTIVE never
	 * reports 0 ms left while it is driving. */
	{
		assist_extended_boost_diag_t diag;
		assist_extended_boost_input_t stopped = coasting();
		assist_extended_boost_init();
		push(&cfg, 2000, CONFIRM_TICKS);
		tick(&stopped, &cfg);
		assist_extended_boost_get_diag(&diag);
		check(diag.remaining_ms == 200,
			"P2. the first ACTIVE tick of a 200 ms boost reports 200 ms, not 199");
		for (int t = 0; t < 200 * (int)EXT_BOOST_CONTROL_TICKS_PER_MS - 2; t++) {
			tick(&stopped, &cfg);
		}
		assist_extended_boost_get_diag(&diag);
		check(diag.remaining_ms >= 1,
			"P2. the last full millisecond still reports at least 1 ms");
	}

	/* Defensive: a null config or input must park the module, not fault. */
	{
		assist_extended_boost_output_t out;
		assist_extended_boost_diag_t diag;
		assist_extended_boost_init();
		push(&cfg, 2000, CONFIRM_TICKS);
		assist_extended_boost_update(0, &cfg, &out);
		assist_extended_boost_get_diag(&diag);
		check(!out.active && diag.state == ASSIST_EXT_BOOST_IDLE,
			"null input parks the module");
		assist_extended_boost_update(0, 0, 0); /* must not dereference anything */
	}

	if (failures == 0) {
		printf("\nAll FW-084 host checks passed.\n");
	} else {
		printf("\n%d FW-084 host check(s) FAILED.\n", failures);
	}
	return failures == 0 ? 0 : 1;
}
