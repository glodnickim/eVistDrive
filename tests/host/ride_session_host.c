/*
 * FW-112 v2 + FW-112.2: exhaustive host tests for src/ride_session.c's RIDE SESSION AUTOMATON
 * (COLD / ACTIVE / SUSPENDED_BY_DIRECTION) - the REAL, shipped module, driven with SYNTHETIC
 * inputs (not through ride_control.c/assist_modes, which tests/host/ride_control_rearm_host.c
 * covers separately as the integration test).
 *
 * WHAT CHANGED FROM v1's VERSION OF THIS FILE. v1 (and the FW-109 two-phase commit before it)
 * made this automaton GUARANTEE a resumed ride only flowed under "fresh pressure AND a real Iq
 * demand": a WAIT_REARM_LOAD state held ACTIVE hostage, and a separate ride_session_commit_rearm()
 * call closed the loop. FW-112 v2 deletes that entirely (see inc/ride_session.h's file header for
 * the full reasoning): the automaton now owns PERMISSION alone - a pure direction fact - and the
 * CALLER owns demand, evaluated fresh every tick. So the whole input surface that existed to
 * resolve demand inside this module is GONE (load_sample_positive, load_filtered_ready,
 * sample_tick, the load-freshness anchor and its wraparound logic), and with it the
 * rearm_candidate_this_tick / ride_session_commit_rearm() split, WAIT_REARM_LOAD, and the
 * candidate commit semantics.
 *
 * FW-112.2 adds the ROLLING-COAST dimension without touching that design (see inc/ride_session.h):
 *   - terminal is now QUALIFIED: non_direction_safety_cut / assist_off / (real_stop &&
 *     !rolling_valid). While the wheel is still fresh (rolling_valid), a PAS timeout is a COAST,
 *     not a true stop - ACTIVE is retained in SUSPENDED_BY_DIRECTION (never latched), so the
 *     coast can never generate assist and a true stop still ends the ride outright.
 *   - a coast resume has a SECOND legitimate permission path: forward_pedaling (the caller's
 *     validated crank_direction_ok), because a forward coast leaves the direction automaton
 *     FORWARD_SAFE and so never produces the confirm edge.
 *
 * What remains is the automaton's OWN job - one decision per tick:
 *   - qualified terminal events always win from any non-COLD state and send it to COLD, never
 *     latched;
 *   - a direction inhibit (a LEVEL) OR a rolling coast (real_stop && rolling_valid) suspends
 *     ACTIVE to SUSPENDED_BY_DIRECTION, arbitrarily long, never to COLD;
 *   - permission returns EXACTLY on a genuine forward path: (forward_confirmed_this_tick edge OR
 *     forward_pedaling) && no inhibit && no terminal - reported as fast_rearm_this_tick for
 *     exactly that tick;
 *   - cold start is unchanged: COLD + cold_start_ready -> ACTIVE (cold_arm_this_tick);
 *   - the reserved legacy value RIDE_SESSION_WAIT_REARM_LOAD (3) is normalized to COLD and can
 *     never grant a latch.
 *
 * Block 1 checks every (real state x 256 boolean input combination) cell against those
 * properties; block 2 covers the reserved-value normalization; block 3 the FW-112 v2 behavioural
 * story (reverse -> suspend -> forward confirm -> immediate permission, no pressure/load
 * condition) extended with the FW-112.2 coast story (pedal release while rolling -> retained
 * SUSPENDED, never latched; resume on forward_pedaling without a confirm edge; true stop -> COLD
 * once the wheel loses freshness); block 4 ride_session_force_cold() from every reachable state.
 */

#include "../common/check.h"

#include "ride_session.h"

/* FW-112.2: the module now takes 8 boolean inputs. Bit layout of the exhaustive mask: */
enum {
	BIT_DIRECTION_INHIBIT = 0x01,
	BIT_FORWARD_CONFIRM   = 0x02,
	BIT_NON_DIR_SAFETY    = 0x04,
	BIT_ASSIST_OFF        = 0x08,
	BIT_REAL_STOP         = 0x10,
	BIT_COLD_START_READY  = 0x20,
	BIT_ROLLING_VALID     = 0x40,
	BIT_FORWARD_PEDALING  = 0x80
};

static ride_session_input_t make_input(
	bool direction_inhibit_active, bool forward_confirmed_this_tick,
	bool non_direction_safety_cut, bool assist_off, bool real_stop,
	bool cold_start_ready, bool rolling_valid, bool forward_pedaling)
{
	ride_session_input_t in;
	in.direction_inhibit_active = direction_inhibit_active;
	in.forward_confirmed_this_tick = forward_confirmed_this_tick;
	in.non_direction_safety_cut = non_direction_safety_cut;
	in.assist_off = assist_off;
	in.real_stop = real_stop;
	in.cold_start_ready = cold_start_ready;
	in.rolling_valid = rolling_valid;
	in.forward_pedaling = forward_pedaling;
	return in;
}

/* The FW-112.2 qualified terminal, mirrored exactly as ride_session.c computes it. */
static bool is_terminal(const ride_session_input_t *in)
{
	return in->non_direction_safety_cut || in->assist_off ||
	       (in->real_stop && !in->rolling_valid);
}

/* Drive the module to a specific starting state with the SMALLEST input sequence that reaches
 * it, using only well-understood single-purpose ticks - not the property under test. */
static void goto_state(ride_session_state_t target)
{
	ride_session_init();
	if (target == RIDE_SESSION_COLD) return;

	ride_session_output_t out;
	{
		ride_session_input_t in = make_input(false, false, false, false, false, true, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_ACTIVE, "setup: reached ACTIVE via cold start");
	}
	if (target == RIDE_SESSION_ACTIVE) return;
	{
		ride_session_input_t in = make_input(true, false, false, false, false, false, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "setup: reached SUSPENDED_BY_DIRECTION");
	}
}

/* --- block 1: every (real state, input combination) cell against the owner's OWN properties. - */
static uint32_t cells_checked = 0;
static void test_exhaustive_transition_properties(void)
{
	static const ride_session_state_t states[3] = {
		RIDE_SESSION_COLD, RIDE_SESSION_ACTIVE, RIDE_SESSION_SUSPENDED_BY_DIRECTION
	};
	for (int s = 0; s < 3; s++) {
		for (int mask = 0; mask < 256; mask++) {
			bool direction_inhibit_active = (mask & BIT_DIRECTION_INHIBIT) != 0;
			bool forward_confirmed        = (mask & BIT_FORWARD_CONFIRM) != 0;
			bool non_direction_safety_cut = (mask & BIT_NON_DIR_SAFETY) != 0;
			bool assist_off               = (mask & BIT_ASSIST_OFF) != 0;
			bool real_stop                = (mask & BIT_REAL_STOP) != 0;
			bool cold_start_ready         = (mask & BIT_COLD_START_READY) != 0;
			bool rolling_valid            = (mask & BIT_ROLLING_VALID) != 0;
			bool forward_pedaling         = (mask & BIT_FORWARD_PEDALING) != 0;

			goto_state(states[s]);
			ride_session_state_t before = ride_session_get_state();
			CHECK(before == states[s], "setup: actually starting from the intended state");

			ride_session_input_t in = make_input(direction_inhibit_active, forward_confirmed,
				non_direction_safety_cut, assist_off, real_stop, cold_start_ready,
				rolling_valid, forward_pedaling);
			ride_session_output_t out;
			ride_session_update(&in, &out);
			bool terminal = is_terminal(&in);

			/* Property 1 (invariant): terminal ALWAYS wins, from ANY non-COLD state,
			 * regardless of anything else - including a simultaneous direction inhibit or
			 * forward-confirm/forward-pedaling. The FW-112.2 qualified terminal is what a
			 * true stop (real_stop && !rolling_valid) must be - a coast (rolling_valid) is
			 * NOT terminal. */
			if (before != RIDE_SESSION_COLD && terminal) {
				CHECK(out.state == RIDE_SESSION_COLD,
					"P1: a terminal input always sends a non-COLD state to COLD, no exceptions");
				CHECK(!out.latched, "P1: terminal -> never latched");
			}

			/* Property 2 (invariant): a direction inhibit alone (no terminal) never reaches
			 * COLD from ACTIVE/SUSPENDED_BY_DIRECTION - it goes to SUSPENDED_BY_DIRECTION (or
			 * stays there), never discarding the session outright, however many prior reverse/
			 * invalid steps occurred (owner's arbitrary-length requirement - this holds
			 * identically regardless of history, because ride_session.c's own state carries
			 * none). */
			if (!terminal && direction_inhibit_active && before != RIDE_SESSION_COLD) {
				CHECK(out.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION,
					"P2: direction_inhibit_active without a terminal event lands in "
					"SUSPENDED_BY_DIRECTION, never COLD");
			}

			/* Property 2c (FW-112.2): a ROLLING COAST (real_stop && rolling_valid, no terminal)
			 * retains an ACTIVE session in SUSPENDED_BY_DIRECTION - never COLD, never latched.
			 * This is the card's core requirement: the coast itself must NEVER grant permission
			 * and a genuine stop's cold-start steps must not be demanded mid-roll. */
			if (before == RIDE_SESSION_ACTIVE && !terminal &&
			    (direction_inhibit_active || (real_stop && rolling_valid))) {
				CHECK(out.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION,
					"P2c: an ACTIVE session is retained in SUSPENDED_BY_DIRECTION by an inhibit "
					"or a rolling coast - never COLD");
				CHECK(!out.latched, "P2c: the coast itself never grants permission");
			}

			/* Property 3: COLD only ever leaves COLD via cold_start_ready (no terminal), and
			 * direction_inhibit_active / rolling coast have NO effect on COLD by themselves. */
			if (before == RIDE_SESSION_COLD) {
				if (!terminal && cold_start_ready) {
					CHECK(out.state == RIDE_SESSION_ACTIVE, "P3: COLD + cold_start_ready -> ACTIVE");
					CHECK(out.cold_arm_this_tick, "P3: flagged as a cold arm");
				} else {
					CHECK(out.state == RIDE_SESSION_COLD, "P3: COLD stays COLD otherwise");
					CHECK(!out.latched, "P3: never latched while COLD");
				}
				CHECK(!out.fast_rearm_this_tick, "P3: COLD never raises the fast-rearm edge");
			}

			/* Property 4 (FW-112 v2 + FW-112.2): fast_rearm_this_tick is the PERMISSION edge -
			 * SUSPENDED_BY_DIRECTION -> ACTIVE - and it fires EXACTLY when a genuine forward
			 * path is present: (forward just confirmed OR forward_pedaling) && no inhibit && no
			 * terminal. No pressure/load/Iq condition exists any more. */
			if (out.fast_rearm_this_tick) {
				CHECK(before == RIDE_SESSION_SUSPENDED_BY_DIRECTION,
					"P4: fast_rearm_this_tick only fires on SUSPENDED_BY_DIRECTION -> ACTIVE");
				CHECK(out.state == RIDE_SESSION_ACTIVE, "P4: and the state is ACTIVE right after");
				CHECK(!terminal && !direction_inhibit_active && (forward_confirmed || forward_pedaling),
					"P4: the rearm edge is a genuine forward path (confirm edge or forward_pedaling) "
					"+ no inhibit + no terminal");
			}

			/* Property 5: SUSPENDED_BY_DIRECTION leaves for ACTIVE iff the rearm edge fires; it
			 * never goes anywhere but ACTIVE (or COLD via terminal), and never waits on any
			 * pressure/load condition. A rolling coast holding real_stop while rolling_valid
			 * stays SUSPENDED until the wheel loses freshness (then terminal -> COLD). */
			if (before == RIDE_SESSION_SUSPENDED_BY_DIRECTION && !terminal) {
				if (!direction_inhibit_active && (forward_confirmed || forward_pedaling)) {
					CHECK(out.state == RIDE_SESSION_ACTIVE,
						"P5: SUSPENDED + genuine forward path (no inhibit) -> ACTIVE, immediately");
					CHECK(out.latched, "P5: permission restored -> latched");
				} else {
					CHECK(out.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION,
						"P5: otherwise SUSPENDED_BY_DIRECTION stays - no load/Iq wait exists to leave through");
					CHECK(!out.latched, "P5: stayed suspended -> never latched (coast grants nothing)");
				}
			}

			/* Property 6: ACTIVE leaves ONLY to SUSPENDED_BY_DIRECTION (inhibit or rolling coast)
			 * or COLD (terminal); cold_arm_this_tick never fires on a plain ACTIVE tick. */
			if (before == RIDE_SESSION_ACTIVE) {
				CHECK(!out.cold_arm_this_tick, "P6: ACTIVE never re-flags a cold arm");
				CHECK(!out.fast_rearm_this_tick, "P6: ACTIVE never re-flags a fast rearm");
			}

			/* Property 7: `latched` is always exactly (state == ACTIVE) - the struct's own
			 * documented contract, checked directly rather than assumed. */
			CHECK(out.latched == (out.state == RIDE_SESSION_ACTIVE),
				"P7: latched is always exactly (state == ACTIVE)");

			cells_checked++;
		}
	}
	printf("  ride_session: %u (state x input-combination) cells checked (3 real states x 256 combos)\n",
		(unsigned)cells_checked);
	CHECK(cells_checked == 768U, "all 768 cells were actually exercised");
}

/* --- block 2: the reserved legacy value (3 = WAIT_REARM_LOAD) is never produced by any input
 * sequence through the REAL module, and can never grant a latch. The module itself normalizes a
 * corrupted/reserved state to COLD at the top of ride_session_update() (a defensive property that
 * needs a seam to inject from outside); the reachable claim here is that no legal input sequence
 * EVER lands in the reserved value, so production can only ever read one of the three real
 * states. -------------------------------------------------------------------------------------- */
static void test_reserved_value_never_reachable(void)
{
	/* Follow the longest plausible saga: cold start, suspend, confirm edge, terminal, repeat -
	 * tracking the module's state after every call. */
	ride_session_init();
	ride_session_output_t out;
	for (int cycle = 0; cycle < 8; cycle++) {
		ride_session_input_t in = make_input(false, false, false, false, false, true, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_ACTIVE, "reach: cold arm");
		in = make_input(true, false, false, false, false, false, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "reach: suspended");
		in = make_input(false, true, false, false, false, false, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_ACTIVE, "reach: rearm edge");
		in = make_input(false, false, false, false, true, false, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_COLD, "reach: terminal -> COLD");
		CHECK(out.state != RIDE_SESSION_WAIT_REARM_LOAD,
			"RESERVED: no legal input sequence ever lands in the reserved legacy value");
		CHECK(!out.latched, "RESERVED: never latched via the reserved path");
	}
	/* Belt and braces: re-init and single-shot into every state confirms the reserved value
	 * stays unreachable even on the very first transition. */
	ride_session_init();
	{
		ride_session_input_t in = make_input(false, true, true, false, false, true, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state != RIDE_SESSION_WAIT_REARM_LOAD, "RESERVED: not reachable on a terminal+cold tick");
	}
}

/* --- block 3: the FW-112 v2 behavioural story (permission returns IMMEDIATELY on a genuine
 * forward path, with no pressure/load/Iq condition) extended with the FW-112.2 rolling-coast
 * story through the real rearm chain shape. ---------------------------------------------------- */
static void test_permission_edge_story(void)
{
	ride_session_init();
	ride_session_output_t out;

	/* cold start -> ACTIVE (latched). */
	{
		ride_session_input_t in = make_input(false, false, false, false, false, true, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_ACTIVE, "story: cold start -> ACTIVE");
		CHECK(out.latched, "story: latched on the cold arm");
	}

	/* a reverse hold: 1, then 4 (arbitrary-length) ticks of direction_inhibit_active - the
	 * session is suspended the whole time, never COLD, never latched. */
	for (int i = 0; i < 5; i++) {
		ride_session_input_t in = make_input(true, false, false, false, false, false, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION,
			"story: the whole reverse hold keeps the session SUSPENDED_BY_DIRECTION");
		CHECK(!out.latched, "story: never latched while suspended");
	}

	/* a forward-confirming step but still inhibited: stays suspended. */
	{
		ride_session_input_t in = make_input(true, true, false, false, false, false, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION,
			"story: confirm step while still inhibited stays suspended");
	}

	/* the confirm EDGE: the exact tick direction stops being inhibited AND forward confirms -
	 * permission returns immediately, with NO load/Iq condition (v2). fast_rearm_this_tick fires
	 * for exactly this tick. */
	{
		ride_session_input_t in = make_input(false, true, false, false, false, false, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_ACTIVE, "story: the confirm edge -> ACTIVE, immediately");
		CHECK(out.latched, "story: permission restored");
		CHECK(out.fast_rearm_this_tick, "story: fast_rearm_this_tick fires for exactly the rearm tick");
	}

	/* next tick, still riding: ACTIVE, no rearm edge, no cold arm. */
	{
		ride_session_input_t in = make_input(false, false, false, false, false, false, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_ACTIVE, "story: still ACTIVE");
		CHECK(!out.fast_rearm_this_tick, "story: the rearm edge is a one-tick edge");
		CHECK(!out.cold_arm_this_tick, "story: not a cold arm");
		CHECK(out.latched, "story: still latched");
	}
}

/* --- block 3b (FW-112.2): the ROLLING-COAST story ---------------------------------------------
 * pedal release while the wheel is still fresh = a COAST: ACTIVE is retained in SUSPENDED, never
 * latched, never COLD; a coast resume re-arms on forward_pedaling alone (a forward coast leaves
 * the direction automaton FORWARD_SAFE, so no confirm edge ever fires); a true stop (wheel loses
 * its conservative freshness while real_stop holds) ends the ride in COLD. */
static void test_rolling_coast_story(void)
{
	ride_session_init();
	ride_session_output_t out;

	/* cold start -> ACTIVE. */
	{
		ride_session_input_t in = make_input(false, false, false, false, false, true, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_ACTIVE, "coast: cold start -> ACTIVE");
	}

	/* riding forward (crank genuinely moving: forward_pedaling true, wheel fresh). */
	{
		ride_session_input_t in = make_input(false, false, false, false, false, false, true, true);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_ACTIVE, "coast: riding forward stays ACTIVE");
		CHECK(out.latched, "coast: latched while riding");
	}

	/* pedal release while rolling: real_stop true + wheel still fresh -> SUSPENDED (COAST). */
	{
		ride_session_input_t in = make_input(false, false, false, false, true, false, true, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION,
			"coast: pedal release while rolling retains the session in SUSPENDED_BY_DIRECTION");
		CHECK(!out.latched, "coast: the coast never grants permission");
		CHECK(!out.fast_rearm_this_tick, "coast: no rearm edge during the coast");
	}

	/* the coast lasts (arbitrarily long, wheel still fresh): never COLD, never latched. */
	for (int i = 0; i < 12; i++) {
		ride_session_input_t in = make_input(false, false, false, false, true, false, true, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "coast: the long coast stays SUSPENDED");
		CHECK(!out.latched, "coast: never latched through the whole coast");
		CHECK(out.state != RIDE_SESSION_COLD, "coast: the rolling coast never visits COLD");
	}

	/* resume by forward_pedaling alone (NO confirm edge - this is the forward-coast path the
	 * card requires): ACTIVE fast rearm, latched, no cold-start steps demanded. */
	{
		ride_session_input_t in = make_input(false, false, false, false, true, false, true, true);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_ACTIVE, "coast: forward_pedaling re-arms ACTIVE without a confirm edge");
		CHECK(out.latched, "coast: latched after the coast resume");
		CHECK(out.fast_rearm_this_tick, "coast: the coast resume is a fast rearm, not a cold arm");
	}

	/* riding again, then a TRUE STOP: real_stop holds AND the wheel loses its 2.65 s freshness
	 * (rolling_valid false) -> qualified terminal -> COLD. */
	{
		ride_session_input_t in = make_input(false, false, false, false, true, false, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_COLD, "coast: real_stop && !rolling_valid (true stop) -> COLD");
		CHECK(!out.latched, "coast: not latched in COLD");
	}

	/* separate true-stop path: the wheel went invalid DURING the coast (not after a resume). */
	ride_session_init();
	{
		ride_session_input_t in = make_input(false, false, false, false, false, true, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_ACTIVE, "coast2: cold start -> ACTIVE");
	}
	{
		ride_session_input_t in = make_input(false, false, false, false, true, false, true, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "coast2: entered the coast");
	}
	{
		ride_session_input_t in = make_input(false, false, false, false, true, false, false, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_COLD, "coast2: wheel invalidity during the coast -> COLD");
	}

	/* a reverse during a coast stays suspended (no rearm while inhibited), then the confirm edge
	 * re-arms exactly as before FW-112.2. */
	ride_session_init();
	{
		ride_session_input_t in = make_input(false, false, false, false, false, true, false, false);
		ride_session_update(&in, &out);
	}
	{
		ride_session_input_t in = make_input(false, false, false, false, true, false, true, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "coast3: coast entered");
	}
	{
		ride_session_input_t in = make_input(true, false, false, false, true, false, true, false);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_SUSPENDED_BY_DIRECTION,
			"coast3: reverse during the coast stays SUSPENDED (never COLD, never re-armed)");
	}
	{
		ride_session_input_t in = make_input(false, true, false, false, true, false, true, true);
		ride_session_update(&in, &out);
		CHECK(out.state == RIDE_SESSION_ACTIVE, "coast3: the confirm edge after the coast -> ACTIVE fast rearm");
		CHECK(out.fast_rearm_this_tick, "coast3: fast rearm flagged");
		CHECK(out.latched, "coast3: latched");
	}
}

/* --- block 4: ride_session_force_cold() from any state ---------------------------------------*/
static void test_force_cold(void)
{
	static const ride_session_state_t states[3] = {
		RIDE_SESSION_COLD, RIDE_SESSION_ACTIVE, RIDE_SESSION_SUSPENDED_BY_DIRECTION
	};
	for (int s = 0; s < 3; s++) {
		goto_state(states[s]);
		ride_session_force_cold();
		CHECK(ride_session_get_state() == RIDE_SESSION_COLD,
			"force_cold: forces COLD immediately from every reachable state");
	}
}

int main(void)
{
	printf("FW-112 v2 + FW-112.2 ride_session.c ride session automaton, against the shipped module\n");

	test_exhaustive_transition_properties();
	test_reserved_value_never_reachable();
	test_permission_edge_story();
	test_rolling_coast_story();
	test_force_cold();

	if (host_test_failures == 0) {
		printf("All ride_session checks passed.\n");
		return 0;
	}
	printf("\n%d ride_session check(s) FAILED.\n", host_test_failures);
	return 1;
}
