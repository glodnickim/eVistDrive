/*
 * FW-101 host harness — runs the SHIPPED ride_episode.c, not a copy of it.
 *
 * Build and run (any host compiler; see run-host-tests.ps1):
 *   gcc -std=c11 -Wall -Wextra -I../../inc -o fw101 fw101_episode_host.c \
 *       ../../src/ride_episode.c && ./fw101
 *
 * WHY THIS EXISTS AT ALL. The first version of this recorder was written inline in main.c and
 * shipped with three defects that invalidated its own output. It could not have been caught,
 * because nothing in main.c is executable by a test — every JS test in this repo reads it as
 * TEXT. A measurement instrument that cannot be tested is not evidence, and a wrong instrument
 * is worse than none: it produces numbers that look plausible and are not.
 *
 * The three defects are the first three test groups below, each written to FAIL against the
 * old behaviour:
 *
 *   1. The remembered arming counter was set to a sentinel at episode start, so the stale
 *      counter from an OLDER arming looked new on the very next tick. Result: a latch time of
 *      about zero, and arming values belonging to a different event.
 *   2. Every reverse step re-anchored the episode and took the CURRENT current as the new
 *      reference. During a shutdown ramp successive steps chased the falling current: the
 *      clock restarted late, steps were undercounted, and the dip could be missed entirely.
 *   3. The arming values were left standing from the previous episode when the new one had no
 *      arming of its own.
 */

#include <stdio.h>
#include <string.h>

#include "ride_episode.h"

static int failures;

/*
 * FW-104: simulates control_time_ticks. reverse_step()/forward_step() now take the CURRENT
 * hardware tick, and run() advances this clock by one 4 kHz period per simulated call - exactly
 * what one reg_ADC_processing() invocation per period looks like. Every existing test below still
 * passes with the SAME expected values as before FW-104, because a clock that always advances by
 * exactly 1 is indistinguishable from the old internal ticks++ - which is itself the regression
 * guard: if that stopped being true, every one of these numbers would move.
 */
static uint32_t g_test_tick;

static void check(int ok, const char *label)
{
	if (!ok) {
		failures++;
		printf("  FAIL  %s\n", label);
	}
}

#define TPMS (CONTROL_TIMEBASE_HZ/1000U)
#define RIDING_IQ 100

/* A quiet tick at a given commanded current, with the pipeline asking for the same. */
static ride_episode_input_t at(int32_t iq, uint16_t arm_seq)
{
	ride_episode_input_t in;
	memset(&in, 0, sizeof(in));
	in.iq_setpoint = iq;
	in.iq_pre_ramp = iq;
	in.arm_seq = arm_seq;
	return in;
}

static void run(const ride_episode_input_t *in, unsigned ticks)
{
	for (unsigned t = 0; t < ticks; t++) {
		g_test_tick++;
		ride_episode_tick(in, g_test_tick);
	}
}

int main(void)
{
	printf("FW-101 episode recorder, against the shipped C module\n");
	printf("  dip %u %%, recover %u %%, intact %u %%, timeout %u ms\n",
		(unsigned)RIDE_EPISODE_DIP_PCT, (unsigned)RIDE_EPISODE_RECOVER_PCT,
		(unsigned)RIDE_EPISODE_INTACT_PCT, (unsigned)RIDE_EPISODE_TIMEOUT_MS);

	/* --- DEFECT 1: a stale arming counter must not read as a fresh arming ------------ */
	{
		/*
		 * The bike has armed several times already, so arm_seq is well above zero when the
		 * episode starts. Nothing arms during the episode. The old code set its remembered
		 * counter to a sentinel, so this same unchanged seq looked new on the next tick.
		 */
		ride_episode_init();
		g_test_tick = 0;
		const uint16_t OLD_SEQ = 7;
		ride_episode_input_t in = at(RIDING_IQ, OLD_SEQ);
		in.arm_load_centikg = 999;   /* values from that OLD arming */
		in.arm_fast_native = 888;
		in.arm_iq_after_limits = 777;
		run(&in, 10);

		ride_episode_reverse_step(RIDING_IQ, OLD_SEQ, g_test_tick);
		/* Drive is lost and never comes back within the timeout. */
		ride_episode_input_t dead = at(0, OLD_SEQ);
		dead.arm_load_centikg = 999;
		dead.arm_fast_native = 888;
		dead.arm_iq_after_limits = 777;
		run(&dead, RIDE_EPISODE_TIMEOUT_TICKS + 20U);

		ride_episode_result_t r;
		ride_episode_get_result(&r);
		check(r.number == 1, "1. the episode completes");
		check(!(r.flags & RIDE_EP_FLAG_LATCH_ARMED),
			"1. an unchanged arming counter is NOT reported as a new arming");
		check(r.t_latch_ms == RIDE_EPISODE_TIME_NONE,
			"1. ...so the latch time stays 'never', not ~0 ms");
		/* DEFECT 3, same episode: no arming here, so no arming values may be published. */
		check(r.arm_load_centikg == 0 && r.arm_fast_native == 0 &&
			r.arm_iq_after_limits == 0,
			"3. an episode without an arming publishes no arming values");
	}

	/* --- a real arming inside the episode IS recorded, with its own values ----------- */
	{
		ride_episode_init();
		g_test_tick = 0;
		ride_episode_input_t in = at(RIDING_IQ, 3);
		run(&in, 5);
		ride_episode_reverse_step(RIDING_IQ, 3, g_test_tick);

		ride_episode_input_t lost = at(0, 3);
		run(&lost, 40);                       /* 10 ms with no drive */

		ride_episode_input_t armed = at(0, 4); /* seq changed => a new arming */
		armed.arm_load_centikg = 85;
		armed.arm_fast_native = 210;
		armed.arm_run_seed_native = 210;
		armed.arm_iq_after_limits = 30;
		armed.iq_pre_ramp = 30;
		run(&armed, 1);

		/* Pipeline comes back to full, then the ramp follows. */
		ride_episode_input_t ready = at(0, 4);
		ready.iq_pre_ramp = RIDING_IQ;
		ready.arm_load_centikg = 85;
		ready.arm_fast_native = 210;
		ready.arm_run_seed_native = 210;
		ready.arm_iq_after_limits = 30;
		run(&ready, 40);                      /* 10 ms of target-ready before the current */

		ride_episode_input_t back = at(RIDING_IQ, 4);
		run(&back, 1);

		ride_episode_result_t r;
		ride_episode_get_result(&r);
		check(r.number == 1, "arming: the episode completes");
		check((r.flags & RIDE_EP_FLAG_LATCH_ARMED) != 0, "arming: the latch is recorded");
		check(r.t_latch_ms == 10, "arming: the latch time is measured from the reverse step");
		check(r.arm_load_centikg == 85 && r.arm_fast_native == 210 &&
			r.arm_run_seed_native == 210 && r.arm_iq_after_limits == 30,
			"arming: the published values are the ones from THIS arming");
		check(r.t_target_ready_ms == 10,
			"three times: target-ready is when the PRE-RAMP target came back");
		check(r.t_recover_ms == 20,
			"three times: recovery is when the CURRENT came back");
		check(r.t_target_ready_ms < r.t_recover_ms,
			"three times: the ramp is the gap between them");
	}

	/* --- DEFECT 3, properly: episode B must not inherit episode A's arming values ---- */
	{
		/*
		 * The first attempt at this check was toothless: it looked at a lone episode with no
		 * arming, where the values are zero anyway, so it passed with the defect present.
		 * The defect only shows across TWO episodes — one that arms, then one that does not.
		 */
		ride_episode_init();
		g_test_tick = 0;

		/* Episode A: arms, with distinctive values. */
		ride_episode_input_t a = at(RIDING_IQ, 1);
		run(&a, 5);
		ride_episode_reverse_step(RIDING_IQ, 1, g_test_tick);
		ride_episode_input_t a_lost = at(0, 1);
		run(&a_lost, 20);
		ride_episode_input_t a_arm = at(0, 2);      /* seq 1 -> 2: armed */
		a_arm.arm_load_centikg = 4242;
		a_arm.arm_fast_native = 3131;
		a_arm.arm_run_seed_native = 2020;
		a_arm.arm_iq_after_limits = 909;
		run(&a_arm, 5);
		ride_episode_input_t a_back = at(RIDING_IQ, 2);
		a_back.arm_load_centikg = 4242;
		a_back.arm_fast_native = 3131;
		a_back.arm_run_seed_native = 2020;
		a_back.arm_iq_after_limits = 909;
		run(&a_back, 1);

		ride_episode_result_t ra;
		ride_episode_get_result(&ra);
		check(ra.number == 1 && ra.arm_load_centikg == 4242,
			"3. precondition: episode A published its own arming values");

		/*
		 * Episode B: no arming at all. The inputs still CARRY episode A's values, exactly as
		 * they do on the bike — ride_control's snapshot keeps the last arming until the next
		 * one. Only the unchanged seq says there was no new arming, and that is what must
		 * keep the values out of the record.
		 */
		ride_episode_input_t b = at(RIDING_IQ, 2);
		b.arm_load_centikg = 4242;
		b.arm_fast_native = 3131;
		b.arm_run_seed_native = 2020;
		b.arm_iq_after_limits = 909;
		run(&b, 5);
		ride_episode_reverse_step(RIDING_IQ, 2, g_test_tick);
		ride_episode_input_t b_lost = b;
		b_lost.iq_setpoint = 0;
		b_lost.iq_pre_ramp = 0;
		run(&b_lost, RIDE_EPISODE_TIMEOUT_TICKS + 20U);

		ride_episode_result_t rb;
		ride_episode_get_result(&rb);
		check(rb.number == 2, "3. episode B completes");
		check(!(rb.flags & RIDE_EP_FLAG_LATCH_ARMED), "3. episode B had no arming");
		check(rb.arm_load_centikg == 0 && rb.arm_fast_native == 0 &&
			rb.arm_run_seed_native == 0 && rb.arm_iq_after_limits == 0,
			"3. episode B publishes zeros, NOT episode A's values");
	}

	/* --- DEFECT 2: steps during a decay must not chase the falling current ----------- */
	{
		/*
		 * The classic shape: drive is lost and ramps down while reverse steps keep arriving.
		 * The old code re-anchored on each one, taking the decayed current as the new
		 * reference — so the clock restarted late, the count reset, and the reference could
		 * be dragged so low that the dip was never seen.
		 */
		ride_episode_init();
		g_test_tick = 0;
		ride_episode_input_t in = at(RIDING_IQ, 1);
		run(&in, 5);

		ride_episode_reverse_step(RIDING_IQ, 1, g_test_tick);      /* anchor at full current */
		int32_t decaying = RIDING_IQ;
		for (int s = 0; s < 5; s++) {
			decaying = (decaying * 7) / 10;           /* current falling away */
			ride_episode_input_t d = at(decaying, 1);
			run(&d, 20);                              /* 5 ms */
			ride_episode_reverse_step(decaying, 1, g_test_tick);   /* another step mid-decay */
		}
		ride_episode_input_t back = at(RIDING_IQ, 1);
		run(&back, 1);

		ride_episode_result_t r;
		ride_episode_get_result(&r);
		check(r.number == 1, "2. the episode completes");
		check(r.rev_steps == 6,
			"2. every reverse step is counted, not reset by the ones during the decay");
		check(r.t_recover_ms >= 25,
			"2. the clock runs from the FIRST step, not from the last one");
	}

	/* --- but a step that cost nothing still re-anchors ------------------------------- */
	{
		/*
		 * The opposite failure, which the re-anchoring exists to prevent: a reverse step
		 * that takes nothing away, then a real one much later. Timing from the first would
		 * merge two unrelated events into one long false reading.
		 */
		ride_episode_init();
		g_test_tick = 0;
		ride_episode_input_t in = at(RIDING_IQ, 1);
		run(&in, 5);

		ride_episode_reverse_step(RIDING_IQ, 1, g_test_tick);   /* costs nothing */
		run(&in, 400);                             /* 100 ms, current untouched */
		ride_episode_reverse_step(RIDING_IQ, 1, g_test_tick);   /* THIS one is the real start */

		ride_episode_input_t lost = at(0, 1);
		run(&lost, 40);                            /* 10 ms without drive */
		ride_episode_input_t back = at(RIDING_IQ, 1);
		run(&back, 1);

		ride_episode_result_t r;
		ride_episode_get_result(&r);
		check(r.number == 1, "re-anchor: the episode completes");
		check(r.t_recover_ms <= 15,
			"re-anchor: a harmless step 100 ms earlier is not counted as the start");
	}

	/* --- the checks that say NO ------------------------------------------------------ */
	{
		/* A reverse step with no current flowing is not an episode at all. */
		ride_episode_init();
		g_test_tick = 0;
		ride_episode_reverse_step(0, 1, g_test_tick);
		check(ride_episode_get_state() == RIDE_EP_IDLE,
			"no episode is started when nothing was flowing");
		ride_episode_input_t idle = at(0, 1);
		run(&idle, 100);
		ride_episode_result_t r;
		ride_episode_get_result(&r);
		check(r.number == 0, "...and nothing is published");

		/* A step that never causes a dip is abandoned, not published. */
		ride_episode_init();
		g_test_tick = 0;
		ride_episode_input_t steady = at(RIDING_IQ, 1);
		run(&steady, 5);
		ride_episode_reverse_step(RIDING_IQ, 1, g_test_tick);
		run(&steady, RIDE_EPISODE_TIMEOUT_TICKS + 20U);
		ride_episode_get_result(&r);
		check(r.number == 0, "a step that never costs anything is not published");
		check(ride_episode_get_state() == RIDE_EP_IDLE, "...and the recorder is free again");

		/* An episode that never recovers is published, flagged. */
		ride_episode_init();
		g_test_tick = 0;
		run(&steady, 5);
		ride_episode_reverse_step(RIDING_IQ, 1, g_test_tick);
		ride_episode_input_t dead = at(0, 1);
		run(&dead, RIDE_EPISODE_TIMEOUT_TICKS + 20U);
		ride_episode_get_result(&r);
		check(r.number == 1 && (r.flags & RIDE_EP_FLAG_TIMED_OUT),
			"an episode that never recovers is published as timed out");
	}

	/*
	 * --- FW-102 DEFECT: t_target_ready fired before the dip was even confirmed -----------
	 *
	 * The 0.0314 bike log showed NEGATIVE (t_target_ready - t_latch) segments, which is
	 * impossible if the three times are truly nested. Root cause: the check ran unconditionally
	 * from the anchor tick, and iq_pre_ramp is still at its PRE-disturbance value on that very
	 * first tick — nothing has had a chance to fall yet. at_least_pct(still-full, ref, 80%) is
	 * true by construction, so it fired ~0 ms into an episode that had not even entered
	 * WAIT_RECOVER. Written to FAIL if the check is ever ungated from RIDE_EP_WAIT_RECOVER.
	 */
	{
		ride_episode_init();
		g_test_tick = 0;
		ride_episode_input_t in = at(RIDING_IQ, 1);
		run(&in, 5);
		ride_episode_reverse_step(RIDING_IQ, 1, g_test_tick);   /* anchor, ticks=0 */

		/* Tick 1: everything still at its pre-disturbance value — the bug's trigger tick. */
		ride_episode_input_t still_full = at(RIDING_IQ, 1);
		run(&still_full, 1);

		/* The cut actually lands: both current and pre-ramp target collapse. */
		ride_episode_input_t cut = at(0, 1);
		run(&cut, 40);                             /* 10 ms; enters WAIT_RECOVER on the 1st */

		/* The pipeline asks for full power again; the ramp has not delivered it yet. */
		ride_episode_input_t target_back = at(0, 1);
		target_back.iq_pre_ramp = RIDING_IQ;
		run(&target_back, 1);

		/* The ramp keeps walking the current up for a while before it actually arrives. */
		ride_episode_input_t ramping = at(50, 1);   /* below RECOVER_PCT of 100 */
		ramping.iq_pre_ramp = RIDING_IQ;
		run(&ramping, 20);                          /* 5 ms */

		ride_episode_input_t back = at(RIDING_IQ, 1);
		run(&back, 1);

		ride_episode_result_t r;
		ride_episode_get_result(&r);
		check(r.number == 1, "FW-102 target-ready: the episode completes");
		check(r.t_target_ready_ms == 10,
			"FW-102 target-ready: fires when the target ACTUALLY recovers post-dip (10 ms)");
		check(r.t_target_ready_ms > 2,
			"FW-102 target-ready: NOT the pre-dip false trigger (would read ~0 ms)");
		check(r.t_recover_ms == 15, "FW-102 target-ready: recovery follows the ramp");
		check(r.t_target_ready_ms < r.t_recover_ms,
			"FW-102 target-ready: still strictly before recovery");
	}

	/* --- FW-102: the gate anatomy fields -------------------------------------------------- */
	{
		ride_episode_init();
		g_test_tick = 0;
		ride_episode_input_t in = at(RIDING_IQ, 1);
		run(&in, 5);
		ride_episode_reverse_step(RIDING_IQ, 1, g_test_tick);   /* anchor, ticks=0 */

		/* 1 ms of drive lost, gates both unmet: neither milestone may fire yet. */
		ride_episode_input_t cut = at(0, 1);
		cut.required_steps = 3;
		cut.load_threshold_centikg = 200;
		cut.load_centikg = 50;
		run(&cut, 4);                              /* ticks=4 -> 1 ms; enters WAIT_RECOVER */

		check(ride_episode_get_state() == RIDE_EP_WAIT_RECOVER,
			"FW-102 gates: precondition — the dip was confirmed");

		/* Crank moving forward again, but not yet enough steps or enough load. */
		ride_episode_forward_step(g_test_tick);
		ride_episode_input_t rebuild = at(0, 1);
		rebuild.required_steps = 3;
		rebuild.load_threshold_centikg = 200;
		rebuild.load_centikg = 120;                /* peak so far, still under threshold */
		run(&rebuild, 7);                          /* ticks=11 */

		ride_episode_result_t mid;
		ride_episode_get_result(&mid);              /* nothing published yet */
		check(mid.number == 0, "FW-102 gates: precondition — not recovered yet");

		/* Steps catch up first: this is tick 12, the first of this block -> exactly 3 ms. */
		ride_episode_input_t steps_ok = at(0, 1);
		steps_ok.required_steps = 3;
		steps_ok.load_threshold_centikg = 200;
		steps_ok.load_centikg = 150;
		steps_ok.fwd_run = 3;
		run(&steps_ok, 4);                          /* ticks=15 */

		/* Load catches up afterwards: tick 16, the first of this block -> exactly 4 ms. */
		ride_episode_input_t load_ok = at(0, 1);
		load_ok.required_steps = 3;
		load_ok.load_threshold_centikg = 200;
		load_ok.load_centikg = 200;
		load_ok.fwd_run = 3;
		run(&load_ok, 4);                           /* ticks=19 */

		/* On the bike this is always populated live from ride_control's gate snapshot, every
		 * tick including the recovery one — never blanked out just because current came back. */
		ride_episode_input_t back = at(RIDING_IQ, 1);
		back.required_steps = 3;
		back.load_threshold_centikg = 200;
		run(&back, 1);                               /* ticks=20 -> recovers */

		ride_episode_result_t r;
		ride_episode_get_result(&r);
		check(r.number == 1, "FW-102 gates: the episode completes");
		check(r.t_first_forward_ms == 1,
			"FW-102 gates: first forward step timed from the SAME anchor as t_latch");
		check(r.t_steps_ready_ms == 3, "FW-102 gates: steps-ready fires when fwd_run catches up");
		check(r.t_load_ready_ms == 4, "FW-102 gates: load-ready fires independently, later");
		check(r.t_steps_ready_ms < r.t_load_ready_ms,
			"FW-102 gates: this episode's wait was spent on the LOAD gate, not the step gate");
		check(r.required_steps == 3 && r.load_threshold_centikg == 200,
			"FW-102 gates: the thresholds actually in force are published, not a constant");
		check(r.load_peak_centikg == 200,
			"FW-102 gates: peak load tracks the highest value seen, not just the last");
		check((r.flags & RIDE_EP_FLAG_FWD_SEEN) != 0,
			"FW-102 gates: a forward step during the wait is flagged");
	}

	/* --- FW-102: a further reverse step restarts the forward-gate milestones ------------- */
	{
		/*
		 * fwd_run is zeroed by every reverse step in main.c (see the PAS decoder), so a
		 * milestone timed against a run that no longer exists would be fiction. The second
		 * push must not inherit the first push's forward-step timing.
		 */
		ride_episode_init();
		g_test_tick = 0;
		ride_episode_input_t in = at(RIDING_IQ, 1);
		run(&in, 5);
		ride_episode_reverse_step(RIDING_IQ, 1, g_test_tick);    /* first push, anchor at ticks=0 */

		ride_episode_input_t mid = at(RIDING_IQ, 1); /* still intact: re-anchors below */
		run(&mid, 4);                                /* ticks=4 -> 1 ms */
		ride_episode_forward_step(g_test_tick);                 /* would-be t_first_forward = 1 ms */

		ride_episode_reverse_step(RIDING_IQ, 1, g_test_tick);      /* second push re-anchors: ticks resets to 0 */

		ride_episode_input_t cut = at(0, 1);
		run(&cut, 12);                                /* ticks=12 -> 3 ms; dip confirmed on tick 1 */
		ride_episode_forward_step(g_test_tick);                   /* the REAL first forward step, at 3 ms */

		ride_episode_input_t back = at(RIDING_IQ, 1);
		run(&back, 1);

		ride_episode_result_t r;
		ride_episode_get_result(&r);
		check(r.number == 1, "FW-102 restart: the episode completes");
		check(r.t_first_forward_ms == 3,
			"FW-102 restart: timed from the LAST reverse step, not the first");
	}

	/*
	 * --- FW-104: a milestone must use the TRUE hardware tick, not "the Nth call" ----------
	 *
	 * FW-104 replaced the module's own ticks++ with control_time_ticks passed in by the
	 * caller, precisely because a call count and real elapsed hardware time stop matching the
	 * moment the main loop falls behind for even one period. This drives the module with EXACT
	 * tick values that skip a stretch - as a busy main loop would - to confirm the published
	 * ms comes from now_tick, not from being the module's 3rd/4th/Nth call.
	 */
	{
		ride_episode_init();
		g_test_tick = 0;
		ride_episode_input_t cut = at(0, 1);

		ride_episode_reverse_step(RIDING_IQ, 1, g_test_tick); /* anchor at tick 0 */
		ride_episode_tick(&cut, 1);  /* dip confirmed: below 50% on the very first call */
		ride_episode_tick(&cut, 2);
		/*
		 * A busy main loop skips ticks 3..9 entirely - the NEXT call lands on hardware tick
		 * 10, but it is only this module's 3rd tick() call. Counting calls would read this
		 * as elapsed=3 (0.75 ms); reading now_tick correctly gives elapsed=10 (2.5 ms).
		 */
		ride_episode_tick(&cut, 10);
		ride_episode_forward_step(10);

		ride_episode_input_t back = at(RIDING_IQ, 1);
		ride_episode_tick(&back, 11); /* recovers */

		ride_episode_result_t r;
		ride_episode_get_result(&r);
		check(r.number == 1, "FW-104 skipped-calls: the episode completes");
		check(r.t_first_forward_ms == 2,
			"FW-104 skipped-calls: ms comes from hardware tick 10 (10/4=2), not the 3rd call");
	}

	/* --- FW-104: elapsed time stays correct across the 32-bit wrap of the hardware clock -- */
	{
		ride_episode_init();
		const uint32_t ANCHOR = 0xFFFFFFF0U;
		ride_episode_reverse_step(RIDING_IQ, 1, ANCHOR); /* anchor just before the wrap */

		ride_episode_input_t cut = at(0, 1);
		ride_episode_tick(&cut, 0xFFFFFFFFU); /* still pre-wrap: elapsed = 15, dip confirmed */

		const uint32_t AFTER_WRAP = 0x00000010U; /* wrapped: 32 ticks after ANCHOR */
		ride_episode_input_t back = at(RIDING_IQ, 1);
		ride_episode_tick(&back, AFTER_WRAP); /* recovers */

		ride_episode_result_t r;
		ride_episode_get_result(&r);
		check(r.number == 1, "FW-104 wrap: the episode completes across the 32-bit wrap");
		check(r.t_recover_ms == 8,
			"FW-104 wrap: 0x10 - 0xFFFFFFF0 = 32 ticks = 8 ms - unsigned subtraction alone handles it");
	}

	/*
	 * --- FW-104: forward_step() must receive now_tick directly, not borrow tick()'s clock -
	 *
	 * The card review that approved this refactor specifically flagged that
	 * ride_episode_forward_step() needs its OWN now_tick parameter: it is called from the PAS
	 * decoder BEFORE ride_control/ride_episode_tick() run in the same main.c iteration, so it
	 * cannot wait for a value tick() computes later. Both must read the SAME control_now
	 * snapshot for one iteration to be self-consistent.
	 */
	{
		ride_episode_init();
		ride_episode_reverse_step(RIDING_IQ, 1, 100); /* anchor at tick 100 */

		ride_episode_input_t cut = at(0, 1);
		ride_episode_tick(&cut, 101); /* dip confirmed */

		/* Forward step and the next tick() call land on the SAME hardware tick, 450 - exactly
		 * what one main.c iteration's single control_now snapshot produces on the bike. */
		ride_episode_forward_step(450);
		ride_episode_tick(&cut, 450);

		ride_episode_result_t r_mid;
		ride_episode_get_result(&r_mid);
		check(r_mid.number == 0, "FW-104 shared-now: precondition - not recovered yet");

		ride_episode_input_t back = at(RIDING_IQ, 1);
		ride_episode_tick(&back, 451); /* recovers */

		ride_episode_result_t r;
		ride_episode_get_result(&r);
		check(r.number == 1, "FW-104 shared-now: the episode completes");
		check(r.t_first_forward_ms == 87,
			"FW-104 shared-now: (450-100)/4 = 87 ms - forward_step used now_tick directly");
	}

	if (failures == 0) {
		printf("All FW-101/FW-102/FW-104 host checks passed.\n");
		return 0;
	}
	printf("\n%d FW-101/FW-102/FW-104 host check(s) FAILED.\n", failures);
	return 1;
}
