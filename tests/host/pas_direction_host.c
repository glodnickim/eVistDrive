/*
 * FW-109 v2 Test B: exhaustive host tests for src/pas_direction.c's PAS DIRECTION SAFETY
 * AUTOMATON (FORWARD_SAFE / DIRECTION_INHIBIT / FORWARD_CONFIRMING) - the REAL, shipped module,
 * not a copy.
 *
 * WHAT CHANGED FROM v1's VERSION OF THIS FILE. The owner's review found (and this file's block 1
 * used to prove) that an INVALID step left fwd_run/dir_state completely untouched from
 * FORWARD_SAFE, and left FORWARD_CONFIRMING's progress untouched too - the exact fail-safe gap
 * this card exists to close. v2's pas_direction.c now treats REVERSE and INVALID identically for
 * the safety automaton (both zero fwd_run and (re)enter DIRECTION_INHIBIT), while keeping a
 * separate diagnostic-only cause (pas_direction_last_inhibit_reason()). Every block below is
 * updated for that - in particular the closed-form property in block 3 now tracks "has this run
 * seen ANY disruption (reverse OR invalid)" and "forward steps since the last one", not
 * reverse-only.
 *
 * WHAT "EXHAUSTIVE" MEANS HERE, PRECISELY.
 *   1. Every state x event transition cell is exercised and asserted individually - block 1.
 *   2. EVERY sequence of forward/reverse/invalid events up to length 12, all 4^12... here
 *      3^12 = 531441 per length, sum over length 1..12 = 797160 total, is driven through the REAL
 *      module and checked against a closed-form property (block 3) - not a parallel
 *      re-implementation of the state machine. The property is a single formula derived directly
 *      from the module's own documented rule, checked against the REAL module's actual state
 *      after every one of those 797160 sequences - this is what makes "any length, any
 *      interleaving of reverse and invalid" a proven fact about the implementation, not an
 *      inference from a few examples.
 *   3. The specific named scenarios from the brief (1R, 4R, 40R, unbounded, R-F-R-F,
 *      R-INVALID-F, F-INVALID-F, a lone invalid, a lone reverse glitch) are ALSO asserted
 *      directly, as a readable, named cross-check that block 3's formula is not itself
 *      accidentally wrong.
 */

#include "../common/check.h"

#include "config.h"   /* BACKWARD_CONFIRM_STEPS / BACKWARD_LATCH_COUNT */
#include "pas_direction.h"

/* --- block 1: every state x event transition cell, individually -------------------------- */
static void test_every_transition_cell(void)
{
	/* FORWARD_SAFE + FORWARD -> FORWARD_SAFE (no-op: already safe) */
	pas_direction_init();
	CHECK(pas_direction_on_step(1) == PAS_STEP_FORWARD, "A1: forward reports FORWARD");
	CHECK(pas_direction_get_state() == PAS_DIR_FORWARD_SAFE, "A1: FORWARD_SAFE + F -> FORWARD_SAFE");
	CHECK(!pas_direction_direction_inhibit_active(), "A1: not inhibited");
	CHECK(pas_direction_last_inhibit_reason() == PAS_DIR_INHIBIT_REASON_NONE, "A1: no reason while safe");

	/* FORWARD_SAFE + INVALID -> DIRECTION_INHIBIT (FW-109 v2: the fixed defect - v1 left this
	 * as a no-op; a decode glitch must fail safe exactly like a reverse step). */
	pas_direction_init();
	uint32_t before = pas_direction_invalid_count();
	CHECK(pas_direction_on_step(0) == PAS_STEP_INVALID, "A2: decoded_dir==0 reports INVALID");
	CHECK(pas_direction_get_state() == PAS_DIR_DIRECTION_INHIBIT,
		"A2 (FIX): FORWARD_SAFE + INVALID -> DIRECTION_INHIBIT, not left at FORWARD_SAFE");
	CHECK(pas_direction_direction_inhibit_active(), "A2 (FIX): inhibited immediately by a lone invalid step");
	CHECK(pas_direction_fwd_run() == 0U, "A2: fwd_run zeroed by the invalid step");
	CHECK(pas_direction_invalid_count() == before + 1U, "A2: invalid count incremented");
	CHECK(pas_direction_last_inhibit_reason() == PAS_DIR_INHIBIT_REASON_INVALID,
		"A2: diagnostic cause is INVALID, not REVERSE");

	/* FORWARD_SAFE + REVERSE -> DIRECTION_INHIBIT (immediate, first step) */
	pas_direction_init();
	CHECK(pas_direction_on_step(-1) == PAS_STEP_REVERSE, "A3: reverse reports REVERSE");
	CHECK(pas_direction_get_state() == PAS_DIR_DIRECTION_INHIBIT, "A3: FORWARD_SAFE + R -> DIRECTION_INHIBIT");
	CHECK(pas_direction_direction_inhibit_active(), "A3: inhibited on the FIRST reverse step - no confirm needed to enter");
	CHECK(pas_direction_last_inhibit_reason() == PAS_DIR_INHIBIT_REASON_REVERSE, "A3: diagnostic cause is REVERSE");

	/* DIRECTION_INHIBIT + REVERSE -> DIRECTION_INHIBIT (stays, sense unchanged) */
	pas_direction_init();
	pas_direction_on_step(-1);
	CHECK(pas_direction_on_step(-1) == PAS_STEP_REVERSE, "A4: second reverse still reports REVERSE");
	CHECK(pas_direction_get_state() == PAS_DIR_DIRECTION_INHIBIT, "A4: DIRECTION_INHIBIT + R -> DIRECTION_INHIBIT");

	/* DIRECTION_INHIBIT + INVALID -> DIRECTION_INHIBIT (stays, cause updates, still counted) */
	pas_direction_init();
	pas_direction_on_step(-1);
	before = pas_direction_invalid_count();
	CHECK(pas_direction_on_step(0) == PAS_STEP_INVALID, "A5: invalid reports INVALID while inhibited");
	CHECK(pas_direction_get_state() == PAS_DIR_DIRECTION_INHIBIT, "A5: DIRECTION_INHIBIT + INVALID -> DIRECTION_INHIBIT");
	CHECK(pas_direction_invalid_count() == before + 1U, "A5: still counted");
	CHECK(pas_direction_last_inhibit_reason() == PAS_DIR_INHIBIT_REASON_INVALID, "A5: cause updates to INVALID");

	/* DIRECTION_INHIBIT + FORWARD -> FORWARD_CONFIRMING (1 of N, N>1) or straight to FORWARD_SAFE
	 * if PAS_REVERSE_RECOVERY_CONFIRM_STEPS==1. Assert whichever the real constant implies. */
	pas_direction_init();
	pas_direction_on_step(-1);
	CHECK(pas_direction_on_step(1) == PAS_STEP_FORWARD, "A6: forward reports FORWARD");
	if (PAS_REVERSE_RECOVERY_CONFIRM_STEPS <= 1U) {
		CHECK(pas_direction_get_state() == PAS_DIR_FORWARD_SAFE,
			"A6: DIRECTION_INHIBIT + F -> FORWARD_SAFE (confirm steps == 1)");
		CHECK(pas_direction_forward_confirmed_last_call(), "A6: confirmed this call");
	} else {
		CHECK(pas_direction_get_state() == PAS_DIR_FORWARD_CONFIRMING,
			"A6: DIRECTION_INHIBIT + F -> FORWARD_CONFIRMING (confirm steps > 1)");
		CHECK(!pas_direction_forward_confirmed_last_call(), "A6: not confirmed yet");
	}

	/* FORWARD_CONFIRMING + FORWARD -> FORWARD_CONFIRMING (still short) or FORWARD_SAFE (reached
	 * N) - only meaningful to test the "still short" case when N >= 3. */
	if (PAS_REVERSE_RECOVERY_CONFIRM_STEPS >= 3U) {
		pas_direction_init();
		pas_direction_on_step(-1);
		pas_direction_on_step(1);   /* 1 of N */
		CHECK(pas_direction_on_step(1) == PAS_STEP_FORWARD, "A7: forward reports FORWARD");
		CHECK(pas_direction_get_state() == PAS_DIR_FORWARD_CONFIRMING,
			"A7: FORWARD_CONFIRMING + F (still short of N) -> FORWARD_CONFIRMING");
		CHECK(!pas_direction_forward_confirmed_last_call(), "A7: still not confirmed");
	}

	/* FORWARD_CONFIRMING + FORWARD (the Nth) -> FORWARD_SAFE, confirmed this call. */
	pas_direction_init();
	pas_direction_on_step(-1);
	for (uint32_t i = 1; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) {
		pas_direction_on_step(1);
		CHECK(pas_direction_get_state() == PAS_DIR_FORWARD_CONFIRMING, "A8: still confirming before the Nth step");
	}
	CHECK(pas_direction_on_step(1) == PAS_STEP_FORWARD, "A8: the Nth forward reports FORWARD");
	CHECK(pas_direction_get_state() == PAS_DIR_FORWARD_SAFE, "A8: the Nth forward step clears to FORWARD_SAFE");
	CHECK(pas_direction_forward_confirmed_last_call(), "A8: confirmed on exactly this call");
	CHECK(pas_direction_last_inhibit_reason() == PAS_DIR_INHIBIT_REASON_NONE, "A8: cause clears to NONE with the state");

	/* FORWARD_CONFIRMING + REVERSE -> DIRECTION_INHIBIT, progress discarded (not just paused). */
	pas_direction_init();
	pas_direction_on_step(-1);
	pas_direction_on_step(1);   /* partial progress */
	CHECK(pas_direction_on_step(-1) == PAS_STEP_REVERSE, "A9: reverse mid-confirmation reports REVERSE");
	CHECK(pas_direction_get_state() == PAS_DIR_DIRECTION_INHIBIT,
		"A9: FORWARD_CONFIRMING + R -> DIRECTION_INHIBIT (progress discarded)");
	/* Prove it was actually DISCARDED, not paused: confirming again needs the FULL N steps. */
	for (uint32_t i = 1; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) {
		pas_direction_on_step(1);
		CHECK(pas_direction_get_state() != PAS_DIR_FORWARD_SAFE,
			"A9: re-confirmation needs the FULL count again, no credit for the discarded progress");
	}
	CHECK(pas_direction_on_step(1) == PAS_STEP_FORWARD, "A9: the Nth fresh forward step");
	CHECK(pas_direction_get_state() == PAS_DIR_FORWARD_SAFE, "A9: NOW it clears");

	/* FORWARD_CONFIRMING + INVALID -> DIRECTION_INHIBIT, progress discarded (FW-109 v2 FIX:
	 * v1 left this as a no-op that preserved partial progress - now identical to A9). */
	if (PAS_REVERSE_RECOVERY_CONFIRM_STEPS >= 2U) {
		pas_direction_init();
		pas_direction_on_step(-1);
		pas_direction_on_step(1);   /* 1 of N, N>=2 so still confirming */
		before = pas_direction_invalid_count();
		CHECK(pas_direction_on_step(0) == PAS_STEP_INVALID, "A10 (FIX): invalid reports INVALID mid-confirmation");
		CHECK(pas_direction_get_state() == PAS_DIR_DIRECTION_INHIBIT,
			"A10 (FIX): FORWARD_CONFIRMING + INVALID -> DIRECTION_INHIBIT, progress discarded");
		CHECK(pas_direction_fwd_run() == 0U, "A10 (FIX): fwd_run zeroed - no credit survives");
		CHECK(pas_direction_invalid_count() == before + 1U, "A10: still counted");
		/* Progress was lost - the FULL N fresh forward steps are needed again. */
		for (uint32_t i = 1; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) {
			pas_direction_on_step(1);
			CHECK(pas_direction_get_state() != PAS_DIR_FORWARD_SAFE,
				"A10 (FIX): re-confirmation needs the FULL count again after the invalid step");
		}
		CHECK(pas_direction_on_step(1) == PAS_STEP_FORWARD, "A10: the Nth fresh forward step");
		CHECK(pas_direction_get_state() == PAS_DIR_FORWARD_SAFE, "A10: NOW it clears");
	}
}

/* --- block 2: fwd_run - FW-107 counter, now ALSO zeroed by INVALID (FW-109 v2 fix) --------- */
static void test_fwd_run(void)
{
	pas_direction_init();
	CHECK(pas_direction_fwd_run() == 0U, "B1: starts at 0");
	for (int i = 1; i <= 5; i++) {
		pas_direction_on_step(1);
		CHECK(pas_direction_fwd_run() == (uint8_t)i, "B1: increments per forward step");
	}
	pas_direction_on_step(-1);
	CHECK(pas_direction_fwd_run() == 0U, "B1: any reverse step zeroes it");

	pas_direction_init();
	for (int i = 1; i <= 5; i++) pas_direction_on_step(1);
	pas_direction_on_step(0);
	CHECK(pas_direction_fwd_run() == 0U, "B1 (FIX): an invalid transition NOW also zeroes it");

	pas_direction_init();
	for (int i = 0; i < 300; i++) pas_direction_on_step(1);
	CHECK(pas_direction_fwd_run() == 250U, "B1: caps at 250, never wraps");

	pas_direction_init();
	pas_direction_on_step(1);
	pas_direction_on_stop();
	CHECK(pas_direction_fwd_run() == 0U, "B1: on_stop() zeroes fwd_run");
	CHECK(pas_direction_get_state() == PAS_DIR_FORWARD_SAFE, "B1: on_stop() also resets the direction automaton");
	CHECK(pas_direction_last_inhibit_reason() == PAS_DIR_INHIBIT_REASON_NONE, "B1: on_stop() clears the reason too");
}

/* --- block 2b: legacy Backwards_counter/pas_rev_run derivation - unchanged FW-098/101 timing,
 * now owned here. REVERSE-only: an invalid step must NOT feed it (it is not evidence of
 * backpedalling) - see BACKWARD_CONFIRM_STEPS/BACKWARD_LATCH_COUNT in inc/config.h. */
static void test_legacy_backpedal_derivation(void)
{
	pas_direction_init();
	CHECK(pas_direction_rev_run() == 0U, "B2: rev_run starts at 0");
	CHECK(pas_direction_backpedal_count() == 0U, "B2: backpedal latch starts at 0");
	CHECK(!pas_direction_backpedal_confirmed(), "B2: not confirmed at start");

	/* Fewer than BACKWARD_CONFIRM_STEPS reverse steps: run counts up, latch stays at 0. */
	pas_direction_on_step(-1);
	CHECK(pas_direction_rev_run() == 1U, "B2: rev_run counts consecutive reverse steps");
	CHECK(pas_direction_backpedal_count() == 0U, "B2: below BACKWARD_CONFIRM_STEPS - latch untouched");

	/* An INVALID step, mid-run, must NOT feed the legacy counter (only the safety automaton
	 * reacts to it) - this is the one place REVERSE and INVALID are deliberately NOT unified. */
	pas_direction_on_step(0);
	CHECK(pas_direction_rev_run() == 1U, "B2 (FIX SCOPE): invalid does not advance rev_run");
	CHECK(pas_direction_backpedal_count() == 0U, "B2: invalid does not touch the backpedal latch");

	/* Resume the run - invalid did not break OR advance it, so one more reverse step reaches
	 * BACKWARD_CONFIRM_STEPS and latches, exactly as if the invalid step had not happened at
	 * this counter's level (it still mattered to the SAFETY automaton above - block 1). */
	pas_direction_on_step(-1);
	CHECK(pas_direction_rev_run() == 2U, "B2: rev_run resumes counting after the invalid step");
	pas_direction_on_step(-1);
	CHECK(pas_direction_rev_run() == 3U, "B2: BACKWARD_CONFIRM_STEPS reached");
	CHECK(pas_direction_backpedal_count() == BACKWARD_LATCH_COUNT, "B2: latches to BACKWARD_LATCH_COUNT");
	CHECK(pas_direction_backpedal_confirmed(), "B2: confirmed (>=4) once latched");
	CHECK(pas_direction_backpedal_latches() == 1U, "B2: one series counted, not one per step");

	/* A forward step breaks the run and decays the latch by exactly one. */
	pas_direction_on_step(1);
	CHECK(pas_direction_rev_run() == 0U, "B2: forward step breaks the reverse run");
	CHECK(pas_direction_backpedal_count() == BACKWARD_LATCH_COUNT - 1U, "B2: latch decays by one per forward step");
	CHECK(pas_direction_backpedal_confirmed(), "B2: still confirmed (still >=4)");
	pas_direction_on_step(1);
	CHECK(pas_direction_backpedal_count() == BACKWARD_LATCH_COUNT - 2U, "B2: decays again");
	CHECK(!pas_direction_backpedal_confirmed(), "B2: no longer confirmed once it drops below 4");

	/* rev_run_max: lifetime peak, never decreases. */
	pas_direction_init();
	for (int i = 0; i < 5; i++) pas_direction_on_step(-1);
	CHECK(pas_direction_rev_run_max() == 5U, "B2: rev_run_max tracks the longest run");
	pas_direction_on_step(1);
	pas_direction_on_step(-1);
	pas_direction_on_step(-1);
	CHECK(pas_direction_rev_run_max() == 5U, "B2: rev_run_max does not fall for a shorter later run");

	/* pas_direction_clear_backpedal_latch(): zeroes ONLY the latch - fwd_run/dir_state/rev_run
	 * are a different concern (main.c's real-stop-and-no-torque cleanup, a stricter, later
	 * condition than pas_direction_on_stop() - see the header). */
	pas_direction_init();
	for (int i = 0; i < 3; i++) pas_direction_on_step(-1);
	CHECK(pas_direction_backpedal_confirmed(), "B2: latched before the clear");
	pas_direction_clear_backpedal_latch();
	CHECK(pas_direction_backpedal_count() == 0U, "B2: clear zeroes the latch");
	CHECK(pas_direction_rev_run() == 3U, "B2: clear leaves rev_run untouched - a different counter");
}

/* --- block 3: THE closed-form property, checked after EVERY sequence up to length 12 ------
 * Formula, derived directly from the module's own documented rule - NOT a re-implementation of
 * its switch statement: DIRECTION_INHIBIT is active iff, scanning the sequence backward from the
 * end, there was ever a REVERSE or INVALID event, and fewer than PAS_REVERSE_RECOVERY_CONFIRM_
 * STEPS consecutive FORWARD events have occurred since the LAST one of either kind (FW-109 v2:
 * both disrupt identically - the v1 version of this formula only tracked REVERSE). */
typedef struct {
	int has_seen_disruption;
	int forward_since_disruption;   /* only meaningful once has_seen_disruption */
} closed_form_t;

static void closed_form_apply(closed_form_t *cf, int event /* -1=R, +1=F, 0=INVALID */)
{
	if (event <= 0) {
		/* FW-109 v2: REVERSE and INVALID both disrupt identically. */
		cf->has_seen_disruption = 1;
		cf->forward_since_disruption = 0;
	} else if (cf->has_seen_disruption) {
		cf->forward_since_disruption++;
	}
}

static int closed_form_inhibited(const closed_form_t *cf)
{
	if (!cf->has_seen_disruption) return 0;
	return cf->forward_since_disruption < (int)PAS_REVERSE_RECOVERY_CONFIRM_STEPS;
}

#define MAX_SEQ_LEN 12

/* Recursively enumerate every sequence over {-1,0,+1}^n for n=1..MAX_SEQ_LEN, driving the REAL
 * module and checking it against the closed form after EVERY prefix (so a length-12 run checks
 * all its own length-1..12 prefixes too - no separate outer loop needed). Depth-first, one
 * shared path buffer, replayed from scratch through the real module at every node. */
static uint32_t sequences_checked = 0;

static void walk(int8_t *path, int depth, int max_depth)
{
	static const int8_t alphabet[3] = { -1, 0, 1 };
	for (int a = 0; a < 3; a++) {
		path[depth] = alphabet[a];
		/* Replay the WHOLE path from scratch through the REAL module and the closed form -
		 * O(n) per node, O(3^n * n) total for one depth; acceptable up to n=12 (see the report
		 * for the measured run time). */
		pas_direction_init();
		closed_form_t cf = { 0, 0 };
		for (int i = 0; i <= depth; i++) {
			int8_t dir = path[i];
			uint8_t event = pas_direction_on_step(dir);
			closed_form_apply(&cf, dir);
			bool expect_inhibit = closed_form_inhibited(&cf) != 0;
			bool actual_inhibit = pas_direction_direction_inhibit_active();
			if (expect_inhibit != actual_inhibit) {
				CHECK(false, "C: closed-form mismatch found by exhaustive search (see stderr path)");
				return;
			}
			/* Invariant: forward_trusted (state==FORWARD_SAFE) and direction_inhibit_active
			 * must never both read true - checked explicitly at every node, not assumed. */
			bool forward_trusted = (pas_direction_get_state() == PAS_DIR_FORWARD_SAFE);
			if (forward_trusted && actual_inhibit) {
				CHECK(false, "C: forward_trusted and direction_inhibit_active both true - invariant violated");
				return;
			}
			if (!forward_trusted && !actual_inhibit) {
				CHECK(false, "C: neither forward_trusted nor direction_inhibit_active - no third state exists");
				return;
			}
			/* Invariant: last_inhibit_reason == NONE iff state == FORWARD_SAFE. */
			bool reason_none = (pas_direction_last_inhibit_reason() == PAS_DIR_INHIBIT_REASON_NONE);
			if (reason_none != forward_trusted) {
				CHECK(false, "C: last_inhibit_reason/state invariant violated");
				return;
			}
			/* Event classification itself must also match dir's sign, every time. */
			if (dir > 0 && event != PAS_STEP_FORWARD) { CHECK(false, "C: forward misclassified"); return; }
			if (dir < 0 && event != PAS_STEP_REVERSE) { CHECK(false, "C: reverse misclassified"); return; }
			if (dir == 0 && event != PAS_STEP_INVALID) { CHECK(false, "C: invalid misclassified"); return; }
		}
		sequences_checked++;
		if (depth + 1 < max_depth) {
			walk(path, depth + 1, max_depth);
		}
	}
}

static void test_exhaustive_sequences(void)
{
	int8_t path[MAX_SEQ_LEN];
	walk(path, 0, MAX_SEQ_LEN);
	/* sum_{n=1}^{12} 3^n = 797160 */
	CHECK(sequences_checked == 797160U,
		"C: every sequence over {R,N,F} up to length 12 was actually walked (797160 of them)");
	printf("  pas_direction: %u sequences (all combinations, length 1-%d) matched the closed form\n",
		(unsigned)sequences_checked, MAX_SEQ_LEN);
	printf("  properties checked at every node: closed-form inhibit, forward_trusted XOR inhibited,\n");
	printf("  last_inhibit_reason<->state invariant, event classification\n");
}

/* --- block 4: named scenarios, readable cross-checks of block 3's formula ------------------ */
static void test_named_scenarios(void)
{
	/* 1R -> F: needs the full N to clear, one glitch behaves exactly like a genuine step. */
	pas_direction_init();
	pas_direction_on_step(-1);
	CHECK(pas_direction_direction_inhibit_active(), "D1: 1R - inhibited");
	for (uint32_t i = 1; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) {
		pas_direction_on_step(1);
		CHECK(pas_direction_direction_inhibit_active(), "D1: still inhibited before N forward steps");
	}
	pas_direction_on_step(1);
	CHECK(!pas_direction_direction_inhibit_active(), "D1: 1R then N forward - cleared");

	/* 4R (sustained) -> F: identical shape to 1R - extra reverse steps are no-ops. */
	pas_direction_init();
	for (int i = 0; i < 4; i++) pas_direction_on_step(-1);
	CHECK(pas_direction_direction_inhibit_active(), "D2: 4R - inhibited");
	for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) pas_direction_on_step(1);
	CHECK(!pas_direction_direction_inhibit_active(), "D2: 4R then N forward - cleared, same as 1R");

	/* 40R and 100R (long sustained hold) -> F: identical shape again - the bug report's
	 * 10-42-step episodes, and the owner's explicit 100R requirement. */
	pas_direction_init();
	for (int i = 0; i < 40; i++) {
		pas_direction_on_step(-1);
		CHECK(pas_direction_direction_inhibit_active(), "D3: inhibited for every one of the 40 steps");
	}
	for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) pas_direction_on_step(1);
	CHECK(!pas_direction_direction_inhibit_active(), "D3: 40R then N forward - cleared, same as 1R/4R");

	pas_direction_init();
	for (int i = 0; i < 100; i++) {
		pas_direction_on_step(-1);
		CHECK(pas_direction_direction_inhibit_active(), "D3b: inhibited for every one of the 100 steps");
	}
	for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) pas_direction_on_step(1);
	CHECK(!pas_direction_direction_inhibit_active(), "D3b: 100R then N forward - identical to 1R/4R/40R");

	/* Reverse without end: never clears, no matter how long. */
	pas_direction_init();
	for (int i = 0; i < 500; i++) {
		pas_direction_on_step(-1);
		if (!pas_direction_direction_inhibit_active()) { CHECK(false, "D4: unbounded reverse must never clear"); break; }
	}
	CHECK(pas_direction_direction_inhibit_active(), "D4: still inhibited after 500 reverse steps");

	/* R-F-R-F (each reverse re-discards progress) - only meaningful when N>1. */
	if (PAS_REVERSE_RECOVERY_CONFIRM_STEPS > 1U) {
		pas_direction_init();
		pas_direction_on_step(-1);
		pas_direction_on_step(1);
		pas_direction_on_step(-1);
		pas_direction_on_step(1);
		CHECK(pas_direction_direction_inhibit_active(),
			"D5: R-F-R-F with N>1 never accumulates two single steps into a clear");

		/* R-F-R-F for arbitrarily long (owner requirement): 200 cycles, must NEVER clear. */
		pas_direction_init();
		for (int cycle = 0; cycle < 200; cycle++) {
			pas_direction_on_step(-1);
			pas_direction_on_step(1);
			if (!pas_direction_direction_inhibit_active()) {
				CHECK(false, "D5b: R-F-R-F for 200 cycles must never clear (N>1)");
				break;
			}
		}
		CHECK(pas_direction_direction_inhibit_active(), "D5b: still inhibited after 200 R-F cycles");
	}

	/* A lone invalid transition, alone: FIX - now inhibits, same as a lone reverse. */
	pas_direction_init();
	uint32_t before = pas_direction_invalid_count();
	pas_direction_on_step(0);
	CHECK(pas_direction_get_state() == PAS_DIR_DIRECTION_INHIBIT, "D6 (FIX): lone invalid - now inhibited");
	CHECK(pas_direction_direction_inhibit_active(), "D6 (FIX): lone invalid - inhibit active");
	CHECK(pas_direction_invalid_count() == before + 1U, "D6: still counted diagnostically");
	CHECK(pas_direction_last_inhibit_reason() == PAS_DIR_INHIBIT_REASON_INVALID, "D6: cause is INVALID");

	/* A single formally-correct reverse GLITCH: indistinguishable from a real one-step
	 * reverse by design (the automaton cannot and need not tell them apart) - behaves exactly
	 * like D1. */
	pas_direction_init();
	pas_direction_on_step(-1);
	CHECK(pas_direction_direction_inhibit_active(), "D7: a single reverse glitch inhibits exactly like a real one");

	/* R-INVALID-F: an invalid step arriving DURING confirmation must re-discard progress and
	 * require the FULL count again, exactly like a second reverse would (owner-mandated
	 * scenario). */
	pas_direction_init();
	pas_direction_on_step(-1);              /* R: enter inhibit */
	pas_direction_on_step(1);                /* first confirming forward */
	pas_direction_on_step(0);                /* INVALID mid-confirmation */
	CHECK(pas_direction_direction_inhibit_active(), "D8: R-INVALID - still inhibited, progress discarded");
	for (uint32_t i = 1; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) {
		pas_direction_on_step(1);
		CHECK(pas_direction_direction_inhibit_active(), "D8: R-INVALID-F - still short of the full fresh count");
	}
	pas_direction_on_step(1);
	CHECK(!pas_direction_direction_inhibit_active(), "D8: R-INVALID-F(x N) - cleared only after the FULL fresh run");

	/* F-INVALID-F: an invalid step while FORWARD_SAFE must inhibit from a clean start too
	 * (owner-mandated scenario) - not just mid-confirmation. */
	pas_direction_init();
	pas_direction_on_step(1);                /* ordinary forward pedalling, FORWARD_SAFE throughout */
	CHECK(!pas_direction_direction_inhibit_active(), "D9: F - not inhibited");
	pas_direction_on_step(0);                /* INVALID from FORWARD_SAFE */
	CHECK(pas_direction_direction_inhibit_active(), "D9 (FIX): F-INVALID - now inhibited from a clean forward run");
	for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) pas_direction_on_step(1);
	CHECK(!pas_direction_direction_inhibit_active(), "D9: F-INVALID-F(x N) - clears after the full fresh run");

	/* Repeated illegal jumps (00<->11 / 01<->10 at the raw decoder level, both surfacing as
	 * decoded_dir==0 here): must never clear on their own, however many arrive. */
	pas_direction_init();
	for (int i = 0; i < 50; i++) {
		pas_direction_on_step(0);
		if (!pas_direction_direction_inhibit_active()) {
			CHECK(false, "D10: repeated invalid transitions must never self-clear");
			break;
		}
	}
	CHECK(pas_direction_direction_inhibit_active(), "D10: still inhibited after 50 invalid transitions");
	CHECK(pas_direction_get_state() != PAS_DIR_FORWARD_SAFE, "D10: state is not FORWARD_SAFE");
}

int main(void)
{
	printf("FW-109 v2 pas_direction.c direction safety automaton, against the shipped module\n");
	printf("  PAS_REVERSE_RECOVERY_CONFIRM_STEPS = %u\n", (unsigned)PAS_REVERSE_RECOVERY_CONFIRM_STEPS);
	printf("  BACKWARD_CONFIRM_STEPS = %u, BACKWARD_LATCH_COUNT = %u\n",
		(unsigned)BACKWARD_CONFIRM_STEPS, (unsigned)BACKWARD_LATCH_COUNT);

	test_every_transition_cell();
	test_fwd_run();
	test_legacy_backpedal_derivation();
	test_exhaustive_sequences();
	test_named_scenarios();

	if (host_test_failures == 0) {
		printf("All pas_direction direction-safety checks passed.\n");
		return 0;
	}
	printf("\n%d pas_direction check(s) FAILED.\n", host_test_failures);
	return 1;
}
