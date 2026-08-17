/*
 * FW-109 v2 / FW-112 v2 INTEGRATION HOST HARNESS: real src/ride_control.c, src/pas_direction.c,
 * src/ride_session.c, src/rider_input.c, src/pas_quadrature.c and their real dependency chain
 * (src/torque_input.c, src/assist_modes.c, src/cadence_comp.c, src/power_curve.c,
 * src/assist_start.c, src/assist_extended_boost.c, src/tuning_config.c, src/assist_dynamics.c,
 * src/assist_limits.c, src/motor_core.c) - the SHIPPED code, linked, not a mirror of it.
 *
 * FW-107/FW-108's rearm_state mechanism is GONE, and FW-112 v2 removed the last of it: the
 * WAIT_REARM_LOAD state and the two-phase commit split. The ride SESSION automaton
 * (src/ride_session.c) now answers a single question - MAY current flow - as a PURE DIRECTION
 * fact, restored on the confirm edge alone, while DEMAND is evaluated fresh every tick by
 * assist_modes_calculate(). See the FW-109/FW-109 v2/FW-112 v2 reports for the full diagnosis.
 *
 *   src/pas_direction.c   the PAS DIRECTION SAFETY AUTOMATON (FORWARD_SAFE / DIRECTION_INHIBIT /
 *                         FORWARD_CONFIRMING) - exhaustively tested on its own in
 *                         tests/host/pas_direction_host.c (every one of 797,160 R/F/invalid
 *                         sequences up to length 12, plus every transition cell) and
 *                         tests/host/pas_quadrature_host.c (all 16 raw transition pairs).
 *   src/ride_session.c    the RIDE SESSION AUTOMATON (COLD / ACTIVE / SUSPENDED_BY_DIRECTION,
 *                         with 3 reserved as the legacy WAIT_REARM_LOAD byte) - exhaustively
 *                         tested on its own in tests/host/ride_session_host.c (every one of the
 *                         512 state x input-combination cells against the owner's own transition
 *                         properties, plus tick wraparound).
 *
 * THIS file is the INTEGRATION layer those exhaustive proofs do not cover on their own: real
 * quadrature decode -> the real automatons, wired together -> a REAL assist_modes_calculate()
 * call that evaluates demand from the real torque_input chain -> a real Iq target, including the
 * mandatory named scenarios, the v2 PERMISSION-vs-DEMAND contract, the cold-start-preserved
 * regression, tick wraparound through the real chain, and a seeded generative long-sequence run
 * that checks the cross-module invariants tick by tick rather than re-deriving what SHOULD
 * happen from a parallel model.
 *
 * The torque side is driven through the REAL torque_input.c chain (torque_input_update every
 * tick + torque_input_run_filter_step on every forward step, exactly like main.c), so the
 * recovery automaton and the stale-sample fix on the rearm edge are exercised for real, not
 * mirrored. The start-gate LOAD is still modeled directly (rider_input_t.torque_load_centikg) so
 * the load-threshold scenarios stay independently controllable; the filtered/RUN/current-sample
 * fields come from the real snapshot.
 */

#include <stdint.h>
#include <string.h>

#include "../common/check.h"

#include "assist_modes.h"
#include "config.h"
#include "motor_core.h"
#include "pas_direction.h"
#include "pas_quadrature.h"
#include "rider_input.h"
#include "ride_control.h"
#include "ride_session.h"
#include "torque_input.h"
#include "tuning_config.h"

#define TEST_ASSIST_LEVEL 3U
#define TEST_SPEED_X100   1500U   /* 15.00 km/h: comfortably "rolling" (>= 100) */
#define TEST_BATTERY_MV   42000U
#define TEST_VOLTAGE_RAW  2000
#define TEST_TEMPERATURE_C 25
#define RUN_DEADBAND_MV_DEFAULT 5U   /* tuning_config.c's default */
#define RIDING_START_LOAD_CENTIKG_DEFAULT 30U
#define STANDSTILL_LOAD_CENTIKG_DEFAULT   70U
/* A request well above the ~2% min-Iq floor at PH_CURRENT_MAX - proves a positive result is the
 * real assist-mode output, not just the floor. */
#define FLOOR_ONLY_CEILING_IQ 60
/* A strong resume pressure (assist_delta native): comfortably above the ~2% floor for the
 * "real assist" assertions even while the RUN window is still refilling part-way. */
#define STRONG_ASSIST_DELTA_NATIVE 310U

static MotorState_t MS;
static uint32_t g_tick;   /* mirrors main.c's control_time_ticks - the fast-rearm load anchor */

/* --- a valid tuning blob, patched and re-checksummed to set start_steps from a test ---------- */
static uint16_t crc16_ccitt(const uint8_t *buffer, uint16_t length)
{
	uint16_t crc = 0xFFFFU;
	for (uint16_t i = 0; i < length; i++) {
		crc ^= (uint16_t)buffer[i] << 8;
		for (uint8_t bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

static void set_tuning_start_steps(uint8_t value)
{
	uint8_t buffer[TUNING_BLOB_LEN];
	uint16_t len = tuning_config_serialize(buffer);
	buffer[22] = value;
	buffer[23] = 0;
	uint16_t crc = crc16_ccitt(buffer, TUNING_BLOB_LEN - 2U);
	buffer[TUNING_BLOB_LEN - 2U] = (uint8_t)(crc & 0xFFU);
	buffer[TUNING_BLOB_LEN - 1U] = (uint8_t)(crc >> 8);
	bool ok = tuning_config_apply_blob(buffer, len);
	CHECK(ok, "setup: tuning blob with the patched start_steps applied");
	CHECK(tuning_config_start_steps() == value, "setup: start_steps actually took the new value");
}

/* --- one control tick, the SAME shape main.c builds every tick ------------------------------ */
typedef struct {
	uint16_t torque_load_centikg;   /* MODELED - the start-gate load, independently controllable */
	uint16_t torque_assist_filtered;/* desired steady-state assist pressure fed to the real chain */
	uint16_t torque_assist_now_native; /* driver bookkeeping only - the real current sample comes
					    * from torque_input_get_snapshot() */
	bool     non_direction_safety_cut;   /* brake / overtemp / torque fault / calibration, folded */
	uint8_t  assist_level_index;
	uint32_t speed_x100;
	bool     pedaling_signal_present;  /* main.c's (cadence>0||start_phase) half of crank_direction_ok */
	bool     real_stop;                /* main.c's pas_idle_ticks>pas_stop_timeout - test-controlled directly */
} step_t;

/* event: 0=no transition this tick, +1=forward, -1=reverse, 2=illegal two-bit jump */
#define EV_NONE    0
#define EV_FORWARD 1
#define EV_REVERSE (-1)
#define EV_INVALID 2

static ride_session_state_t g_prev_session_state = RIDE_SESSION_COLD;
static bool g_invariants_ok = true;

static int32_t live_target(void);   /* forward decl, defined below with the other measurements */
static bool is_latched(void);
static bool arm_snapshot_fast_rearm(void);
static int32_t mode_iq_request(void);

/* Cross-module invariants, checked after EVERY tick this harness ever drives - see the FW-109
 * report's invariant list #1-#8 (#9/#10 are checked by the dedicated scenario tests instead,
 * since they are about MULTI-tick outcomes, not a single-tick property). The v1 "a fast rearm
 * requires fresh pressure and a positive iq_request this very tick" invariant (#5/#6) is
 * DELIBERATELY NOT here: FW-112 v2 makes permission a pure direction fact and demand an
 * independent per-tick evaluation, so a fast rearm legitimately restores permission at zero
 * demand. That contract is pinned by the dedicated RFRF-nopressure / filter-lag / 2F7R2F
 * scenarios instead. */
static void check_invariants(const step_t *s, int event)
{
	bool direction_inhibit = pas_direction_direction_inhibit_active();
	ride_session_state_t state = (ride_session_state_t)ride_control_get_session_state();
	bool latched = is_latched();
	bool terminal_fed = s->non_direction_safety_cut || (s->assist_level_index == 0) || s->real_stop;
	(void)event;

	/* #1/#2: direction_inhibit_active => Iq=0 and not latched, unconditionally - the FW-109 v2
	 * DEFAULT-DENY final gate, independent of REVERSE vs INVALID as the cause. */
	if (direction_inhibit) {
		if (live_target() != 0) { CHECK(false, "INV1: direction_inhibit_active but live_target != 0"); g_invariants_ok = false; }
		if (latched) { CHECK(false, "INV2: direction_inhibit_active but latched"); g_invariants_ok = false; }
	}
	/* not-latched => Iq=0 (throttle is always 0 in this harness, so this holds exactly). */
	if (!latched && live_target() != 0) {
		CHECK(false, "INV: not latched but live_target != 0");
		g_invariants_ok = false;
	}
	/* COLD => not latched. */
	if (state == RIDE_SESSION_COLD && latched) {
		CHECK(false, "INV: session COLD but latched");
		g_invariants_ok = false;
	}
	/* SUSPENDED_BY_DIRECTION => not latched (permission is only ever held in ACTIVE). */
	if (state == RIDE_SESSION_SUSPENDED_BY_DIRECTION && latched) {
		CHECK(false, "INV: session SUSPENDED_BY_DIRECTION but latched");
		g_invariants_ok = false;
	}
	/* #3: a direction-inhibit event alone (no terminal fed) never sends ACTIVE straight to COLD -
	 * it must land in SUSPENDED_BY_DIRECTION, whether the cause was REVERSE or INVALID. */
	if (g_prev_session_state == RIDE_SESSION_ACTIVE && state == RIDE_SESSION_COLD && !terminal_fed) {
		CHECK(false, "INV3: ACTIVE -> COLD with no terminal event fed this tick");
		g_invariants_ok = false;
	}
	/* #8: a terminal event fed this tick always ends a non-COLD session THIS tick. */
	if (terminal_fed && g_prev_session_state != RIDE_SESSION_COLD && state != RIDE_SESSION_COLD) {
		CHECK(false, "INV8: a terminal event was fed but the session did not end this tick");
		g_invariants_ok = false;
	}
	/* FW-109 v2: COLD can never gain fast_rearm - only fast_rearm_this_tick (a resumed
	 * ACTIVE) can. Checked structurally by ride_session_host.c's P6; reconfirmed here against
	 * the real integrated chain. */
	if (g_prev_session_state == RIDE_SESSION_COLD && state == RIDE_SESSION_ACTIVE && arm_snapshot_fast_rearm()) {
		CHECK(false, "INV: COLD -> ACTIVE must never be flagged as fast_rearm");
		g_invariants_ok = false;
	}
	g_prev_session_state = state;
}

/* Drive the REAL torque_input.c chain to a desired steady-state assist pressure. offset_correction
 * is 0 after torque_input_init(), so corrected == raw and:
 *   delta = raw - REST_TARGET_NATIVE(=TORQUE_ZERO_TARGET_NATIVE)   -> the configured target,
 *   assist_delta = delta - TORQUE_ASSIST_DEADBAND_NATIVE           -> converges to the target,
 *   assist_delta_filtered_native                                  -> the 35 ms EMA of it.
 * The EMA moves by ~1/140 of the error per tick, so a step change settles in ~150 ms of ticks -
 * scenarios that need a level to have ARRIVED idle long enough for it to settle (the same
 * discipline fw112_run_rearm_recovery_host.c already uses). */
static void feed_pressure(uint16_t filtered_target)
{
	uint32_t delta = (uint32_t)filtered_target + TORQUE_ASSIST_DEADBAND_NATIVE;
	uint32_t raw = (uint32_t)TORQUE_ZERO_TARGET_NATIVE + delta;
	if (raw > TORQUE_SPAN_MAX_NATIVE) {
		raw = TORQUE_SPAN_MAX_NATIVE;
	}
	torque_input_update((uint16_t)raw, torque_input_correct((uint16_t)raw), true);
}

static void do_tick(step_t *s, int event)
{
	g_tick++;
	uint8_t pas_event = PAS_STEP_NONE;
	if (event != EV_NONE) {
		int8_t decoded_dir = (event == EV_INVALID) ? 0 : (int8_t)event;
		pas_event = pas_direction_on_step(decoded_dir);
	}
	(void)pas_event;
	if (event == EV_FORWARD) {
		torque_input_run_filter_step();   /* main.c:2144 - the RUN window advances on the step */
	}
	feed_pressure(s->torque_assist_filtered);   /* main.c:2329 - drive the real torque chain */

	const torque_snapshot_t *snap = torque_input_get_snapshot();
	bool direction_inhibit_active = pas_direction_direction_inhibit_active();
	bool forward_confirmed_this_tick = (event != EV_NONE) && pas_direction_forward_confirmed_last_call();
	uint8_t fwd_run = pas_direction_fwd_run();

	/* PRODUCTION FORMULA, main.c: crank_direction_ok = (cadence>0||start_phase) && !real_stop -
	 * no longer any Backwards_counter term (FW-109 removed it - see main.c). */
	bool crank_direction_ok = s->pedaling_signal_present && !s->real_stop;
	/* PRODUCTION FORMULA, main.c: ride_core_pedaling = crank_direction_ok && fwd_run >=
	 * tuning_config_start_steps(). */
	bool pedaling = crank_direction_ok && (fwd_run >= tuning_config_start_steps());

	rider_input_t r;
	memset(&r, 0, sizeof(r));
	/* Load is MODELED independently of the torque chain so the start-gate load-threshold
	 * scenarios stay controllable; the torque signal itself comes from the REAL snapshot. */
	r.torque_load_centikg = s->torque_load_centikg;
	r.torque_assist_filtered = snap->assist_delta_filtered_native;
	r.torque_run_filtered = snap->assist_delta_run_native;
	r.torque_assist_now_native = snap->assist_delta_native;
	r.cadence_rpm = crank_direction_ok ? 60U : 0U;
	r.wheel_speed_x100 = s->speed_x100;
	r.motor_erps = crank_direction_ok ? 200U : 0U;
	r.pas_forward = pedaling;
	r.pas_backward = false;
	r.pedaling_active = pedaling;
	r.crank_forward_steps = fwd_run;
	r.crank_direction_ok = crank_direction_ok;
	r.real_stop = s->real_stop;
	r.direction_inhibit_active = direction_inhibit_active;
	r.forward_confirmed_this_tick = forward_confirmed_this_tick;
	r.sample_tick = g_tick;
	r.start_phase = false;
	r.torque_sensor_valid = true;
	r.pas_sensor_valid = true;
	rider_input_update(&r);

	ride_control_input_t in;
	memset(&in, 0, sizeof(in));
	in.speed_x100 = s->speed_x100;
	in.cadence_rpm = r.cadence_rpm;
	in.assist_level_index = s->assist_level_index;
	in.battery_voltage_mv = TEST_BATTERY_MV;
	in.iq_scale = (int32_t)PH_CURRENT_MAX;
	in.ride_core_iq_limit = (int32_t)PH_CURRENT_MAX;
	in.phase_current_max = (int32_t)PH_CURRENT_MAX;
	in.current_iq = MS.i_q_setpoint;
	in.current_id = MS.i_d_setpoint;
	in.voltage_raw = TEST_VOLTAGE_RAW;
	in.voltage_min_raw = VOLTAGE_MIN;
	in.controller_temperature_c = TEST_TEMPERATURE_C;
	in.cadence_filtered_x8 = (uint16_t)(r.cadence_rpm * 8U);
	in.speed_limit_x100 = SPEEDLIMIT;
	in.legal_enabled = (LEGALFLAG != 0);
	in.offroad = false;
	in.walk_active = false;
	in.position_calibration_active = false;
	in.safety_cut_non_direction = s->non_direction_safety_cut;
	in.throttle_iq = 0;
	ride_control_update(&in);

	check_invariants(s, event);
}

static void fwd1(step_t *s) { do_tick(s, EV_FORWARD); }
static void rev1(step_t *s) { do_tick(s, EV_REVERSE); }
static void inv1(step_t *s) { do_tick(s, EV_INVALID); }
static void hold(step_t *s, uint32_t n) { for (uint32_t i = 0; i < n; i++) do_tick(s, EV_NONE); }

static void reset_all(void)
{
	torque_input_init();
	torque_input_set_run_window_deg(180U);
	assist_modes_init();
	assist_modes_set_active_bank(0);
	memset(&MS, 0, sizeof(MS));
	motor_core_init(&MS);
	ride_control_init();
	pas_direction_init();
	g_tick = 0;
	g_prev_session_state = RIDE_SESSION_COLD;
	set_tuning_start_steps(TUNING_START_STEPS_DEFAULT);
}

static step_t riding_step(void)
{
	step_t s;
	memset(&s, 0, sizeof(s));
	s.pedaling_signal_present = true;
	s.real_stop = false;
	s.speed_x100 = TEST_SPEED_X100;
	s.assist_level_index = TEST_ASSIST_LEVEL;
	return s;
}

/* Ride forward with real load until the latch is armed and settled, from a clean reset. */
static step_t establish_latch(void)
{
	step_t s = riding_step();
	s.torque_load_centikg = RIDING_START_LOAD_CENTIKG_DEFAULT + 20U;
	s.torque_assist_filtered = 200U;
	s.torque_assist_now_native = 200U;
	uint8_t need = tuning_config_start_steps();
	for (uint8_t i = 0; i < (uint8_t)(need + 2U); i++) fwd1(&s);
	hold(&s, 3000U);
	return s;
}

/* --- measurement ----------------------------------------------------------------------------*/
static int32_t mode_iq_request(void)
{
	const assist_mode_output_t *out = assist_modes_get_last_output();
	return out->iq_request;
}
static int32_t iq_after_latch_floor(void)
{
	ride_arm_snapshot_t snap;
	ride_control_get_arm_snapshot(&snap);
	return snap.iq_after_latch_floor;
}
static int32_t live_target(void)
{
	ride_arm_snapshot_t snap;
	ride_control_get_arm_snapshot(&snap);
	return snap.iq_pre_ramp;
}
static bool is_latched(void)
{
	return (ride_control_get_debug_flags() & RIDE_DBG_NOT_LATCHED) == 0;
}
static bool arm_snapshot_fast_rearm(void)
{
	ride_arm_snapshot_t snap;
	ride_control_get_arm_snapshot(&snap);
	return snap.fast_rearm;
}
static ride_session_state_t session_state(void)
{
	return (ride_session_state_t)ride_control_get_session_state();
}

int main(void)
{
	printf("FW-109 v2 / FW-112 v2 ride_control/pas_direction/ride_session integration, against the shipped modules\n");
	printf("  PAS_REVERSE_RECOVERY_CONFIRM_STEPS=%u  run deadband %u mV  riding threshold %u centikg\n",
		(unsigned)PAS_REVERSE_RECOVERY_CONFIRM_STEPS,
		(unsigned)RUN_DEADBAND_MV_DEFAULT, (unsigned)RIDING_START_LOAD_CENTIKG_DEFAULT);

	/* ==========================================================================================
	 * MANDATORY SCENARIOS
	 * ======================================================================================= */

	/* --- 1R -> F: isolated single reverse step still fast-rearms ---------------------------- */
	{
		reset_all();
		step_t s = establish_latch();
		CHECK(is_latched(), "1R: setup - latched");
		rev1(&s);
		CHECK(live_target() == 0, "1R: Iq cuts to 0 the SAME tick as the reverse");
		CHECK(!is_latched(), "1R: not latched");
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "1R: SUSPENDED_BY_DIRECTION");
		for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
		CHECK(is_latched(), "1R: fast-rearms after N confirmed forward steps with pressure present");
		CHECK(arm_snapshot_fast_rearm(), "1R: flagged as fast_rearm");
		CHECK(mode_iq_request() > FLOOR_ONLY_CEILING_IQ, "1R: real assist, not just the floor");
	}

	/* --- 4R -> F: a short sustained hold behaves identically to 1R -------------------------- */
	{
		reset_all();
		step_t s = establish_latch();
		for (int i = 0; i < 4; i++) rev1(&s);
		CHECK(!is_latched(), "4R: not latched throughout");
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "4R: SUSPENDED_BY_DIRECTION");
		for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
		CHECK(is_latched(), "4R: fast-rearms exactly like 1R");
	}

	/* --- 40R -> F: a long sustained hold (the bug report's 10-42-step range) - identical ----- */
	{
		reset_all();
		step_t s = establish_latch();
		bool iq_stayed_zero = true, floor_stayed_zero = true, stayed_suspended = true;
		for (int i = 0; i < 40; i++) {
			rev1(&s);
			if (live_target() != 0) iq_stayed_zero = false;
			if (iq_after_latch_floor() != 0) floor_stayed_zero = false;
			if (session_state() != RIDE_SESSION_SUSPENDED_BY_DIRECTION) stayed_suspended = false;
		}
		CHECK(iq_stayed_zero, "40R: Iq stayed 0 for every one of the 40 reverse steps");
		CHECK(floor_stayed_zero, "40R: the min-Iq floor never leaked in either");
		CHECK(stayed_suspended, "40R: session stayed SUSPENDED_BY_DIRECTION for every one of the 40 steps - never fell back to COLD");
		for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
		CHECK(is_latched(), "40R: fast-rearms exactly like 1R/4R - no cold-start gate re-imposed");
		CHECK(arm_snapshot_fast_rearm(), "40R: flagged as fast_rearm, not a normal cold arm");
	}

	/* --- 100R -> F: the owner's explicit "identical for 100R" requirement - same shape again,
	 * through the REAL chain including the real assist_modes_calculate() commit check ---------- */
	{
		reset_all();
		step_t s = establish_latch();
		bool iq_stayed_zero = true, stayed_suspended = true;
		for (int i = 0; i < 100; i++) {
			rev1(&s);
			if (live_target() != 0) iq_stayed_zero = false;
			if (session_state() != RIDE_SESSION_SUSPENDED_BY_DIRECTION) stayed_suspended = false;
		}
		CHECK(iq_stayed_zero, "100R: Iq stayed 0 for every one of the 100 reverse steps");
		CHECK(stayed_suspended, "100R: session stayed SUSPENDED_BY_DIRECTION for every one of the 100 steps");
		for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
		CHECK(is_latched(), "100R: fast-rearms exactly like 1R/4R/40R - identical, not a function of step count");
		CHECK(arm_snapshot_fast_rearm(), "100R: flagged as fast_rearm");
		CHECK(mode_iq_request() > FLOOR_ONLY_CEILING_IQ, "100R: real assist-mode output, not just the floor");
	}

	/* --- reverse without end: never rearms, no matter how long ------------------------------ */
	{
		reset_all();
		step_t s = establish_latch();
		bool stayed_unlatched = true;
		for (int i = 0; i < 500; i++) {
			rev1(&s);
			if (is_latched()) stayed_unlatched = false;
		}
		CHECK(stayed_unlatched, "unbounded R: never latches, however long the hold runs");
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "unbounded R: still SUSPENDED_BY_DIRECTION, not COLD");
	}

	/* --- 2F -> 7R -> 2F -> load: confirm forward first (irrelevant once a reverse arrives),
	 * long reverse hold, then the direction-confirm edge re-arms PERMISSION on its own. v2:
	 * permission is a direction fact, NOT a load condition (the v1 WAIT_REARM_LOAD gate is
	 * gone); demand is still evaluated fresh, so the mode honestly follows the real 35 ms
	 * filter, which is still warm from the pre-reverse pressure on the confirm tick. ---------- */
	{
		reset_all();
		step_t s = establish_latch();
		s.torque_assist_filtered = 0U;
		fwd1(&s); fwd1(&s);   /* already forward-only, no-ops on an already-safe direction */
		for (int i = 0; i < 7; i++) rev1(&s);
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "2F7R2F: suspended after the 7R");
		for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
		CHECK(is_latched(), "2F7R2F: permission is re-armed on the confirm edge alone - no load condition (v2)");
		CHECK(session_state() == RIDE_SESSION_ACTIVE, "2F7R2F: ACTIVE");
		/* The real 35 ms filter is still decaying from the pre-reverse pressure, so the mode
		 * asks for real assist on the confirm tick - the honest v2 answer, never the stale ~0
		 * snapshot the v1 two-phase wait used to stall on. */
		CHECK(mode_iq_request() > FLOOR_ONLY_CEILING_IQ, "2F7R2F: real assist from the still-warm real filter");
		s.torque_assist_filtered = 200U;
		hold(&s, 1U);
		CHECK(is_latched(), "2F7R2F: load arrives - still latched, demand follows");
		CHECK(mode_iq_request() > FLOOR_ONLY_CEILING_IQ, "2F7R2F: real assist");
	}

	/* --- R-F-R-F, no pressure ever: permission is restored on EVERY confirm (v2 - a pure
	 * direction fact), but demand stays 0 all the way because there is no real pressure - the
	 * strongest possible demonstration that PERMISSION != DEMAND. The filter is allowed to
	 * fully decay before the sequence so the 0 is the real mode answer, not a hidden sample. --- */
	{
		reset_all();
		step_t s = establish_latch();
		s.torque_assist_filtered = 0U;
		hold(&s, 800U);   /* let the real 35 ms filter fully decay to ~0 */
		rev1(&s);
		CHECK(!is_latched(), "RFRF-nopressure: suspended after the R");
		for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
		CHECK(is_latched(), "RFRF-nopressure: permission restored on the confirm despite zero pressure");
		CHECK(live_target() == 0, "RFRF-nopressure: zero pressure -> zero demand (permission != demand)");
		rev1(&s);
		CHECK(!is_latched(), "RFRF-nopressure: re-suspended by the second R");
		for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
		CHECK(is_latched(), "RFRF-nopressure: permission restored on the second confirm too - never COLD");
		CHECK(live_target() == 0, "RFRF-nopressure: still zero demand");
		CHECK(session_state() != RIDE_SESSION_COLD, "RFRF-nopressure: the session was never discarded to COLD");
	}

	/* --- R-F-R-F with pressure only at the very end: permission returns on the confirm edge,
	 * and the demand then honestly ramps with the real 35 ms filter once the pedal is pressed
	 * (v2 - recovery is bounded by the fast-filter time, never a stale baseline). ------------- */
	{
		reset_all();
		step_t s = establish_latch();
		s.torque_assist_filtered = 0U;
		hold(&s, 800U);   /* decay the real filter so "no pressure" really means no pressure */
		rev1(&s); fwd1(&s); rev1(&s);
		CHECK(!is_latched(), "RFRF-end: still not latched before the final confirm");
		s.torque_assist_filtered = 200U;
		for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
		CHECK(is_latched(), "RFRF-end: fast-rearms once forward confirms AND pressure is present");
		/* The pedal was just pressed, so the real filter is still ramping on the confirm tick -
		 * permission is granted immediately; the demand then catches up as the estimator refills
		 * under real pedalling and the rider keeps pressing (a strong resume, the canonical
		 * rearm case - bounded by the fast filter, never a stale baseline). */
		s.torque_assist_filtered = STRONG_ASSIST_DELTA_NATIVE;
		for (uint32_t i = 0; i < 300U; i++) fwd1(&s);
		CHECK(mode_iq_request() > FLOOR_ONLY_CEILING_IQ, "RFRF-end: real assist once the rider resumes pedalling under strong pressure");
	}

	/* --- pressure held continuously before, during and after the reverse -------------------- */
	{
		reset_all();
		step_t s = establish_latch();
		/* s already carries pressure from establish_latch(); leave it untouched. */
		int32_t iq_before = MS.i_q_setpoint;
		CHECK(iq_before > 0, "held-pressure: setup - real current flowing");
		rev1(&s);
		CHECK(live_target() == 0, "held-pressure: cuts to 0 on the reverse despite pressure never leaving");
		for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
		CHECK(is_latched(), "held-pressure: fast-rearms as soon as direction confirms - pressure was there all along");
		CHECK(arm_snapshot_fast_rearm(), "held-pressure: flagged as fast_rearm");
	}

	/* --- reverse and a non-reverse safety cut in the SAME tick: non-reverse wins ------------- */
	{
		reset_all();
		step_t s = establish_latch();
		/* Model "reverse and brake together": drive the reverse step and set the cut on the
		 * exact same tick by setting the flag before the call that carries the reverse event. */
		s.non_direction_safety_cut = true;
		rev1(&s);
		CHECK(session_state() == RIDE_SESSION_COLD,
			"R+brake same tick: non_direction_safety_cut wins - session goes straight to COLD, not SUSPENDED_BY_DIRECTION");
		CHECK(live_target() == 0, "R+brake same tick: Iq is 0");
		CHECK(!is_latched(), "R+brake same tick: not latched");

		s.non_direction_safety_cut = false;
		s.torque_load_centikg = RIDING_START_LOAD_CENTIKG_DEFAULT + 20U;
		s.torque_assist_now_native = 200U;
		s.torque_assist_filtered = 200U;
		fwd1(&s); fwd1(&s);
		CHECK(!is_latched(),
			"R+brake same tick: 2 forward + pressure do NOT fast-rearm afterward - the session was discarded");
		uint8_t need = tuning_config_start_steps();
		for (uint8_t i = 0; i < need; i++) fwd1(&s);
		hold(&s, 2U);
		CHECK(is_latched(), "R+brake same tick: a full cold start still works normally afterward");
		CHECK(!arm_snapshot_fast_rearm(), "R+brake same tick: this arming is NORMAL, not fast_rearm");
	}

	/* --- reverse, then a REAL stop/PAS timeout: ends the session; normal cold start after ---- */
	{
		reset_all();
		step_t s = establish_latch();
		rev1(&s);
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "R-then-stop: suspended after the reverse");
		s.real_stop = true;
		hold(&s, 1U);
		CHECK(session_state() == RIDE_SESSION_COLD, "R-then-stop: a genuine stop ends the session -> COLD");

		s.real_stop = false;
		s.pedaling_signal_present = true;
		s.torque_load_centikg = RIDING_START_LOAD_CENTIKG_DEFAULT + 20U;
		s.torque_assist_now_native = 200U;
		s.torque_assist_filtered = 200U;
		uint8_t need = tuning_config_start_steps();
		for (uint8_t i = 0; i < (uint8_t)(need + 2U); i++) fwd1(&s);
		hold(&s, 2U);
		CHECK(is_latched(), "R-then-stop: a normal cold start works again afterward");
		CHECK(!arm_snapshot_fast_rearm(), "R-then-stop: NORMAL arm, not fast_rearm");
	}

	/* --- a further reverse arriving DURING the recovery window (session already re-armed to
	 * ACTIVE, recovery automaton open): re-suspends, CANCELS the recovery, discards the confirm
	 * progress - and the next confirm re-arms exactly like the first. Pressure is held
	 * throughout, so the automaton is genuinely mid-window when the reverse lands. -------------- */
	{
		reset_all();
		step_t s = establish_latch();
		rev1(&s);
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "R-during-recovery: setup - suspended");
		for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
		CHECK(is_latched(), "R-during-recovery: re-armed to ACTIVE (recovery automaton now open)");
		CHECK(torque_input_recovery_state() != TORQUE_RECOVERY_IDLE,
			"R-during-recovery: recovery automaton open");
		rev1(&s);
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION,
			"R-during-recovery: a reverse while recovering re-suspends, discarding the confirmed direction");
		CHECK(!is_latched(), "R-during-recovery: not latched");
		CHECK(torque_input_recovery_state() == TORQUE_RECOVERY_IDLE,
			"R-during-recovery: recovery automaton cancelled on the reverse");
		/* Confirming again needs the FULL count again, not a partial credit. */
		for (uint32_t i = 0; i + 1 < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) {
			fwd1(&s);
			CHECK(!is_latched(), "R-during-recovery: still confirming again, not latched yet");
		}
		fwd1(&s);
		CHECK(is_latched(), "R-during-recovery: fully re-confirmed and fast-rearms");
	}

	/* --- a single illegal quadrature transition: FW-109 v2 FIX - now interrupts assist exactly
	 * like a lone reverse step (v1 left this as a no-op, the defect this card exists to close),
	 * and recovers the same way, through the REAL chain. --------------------------------------- */
	{
		reset_all();
		step_t s = establish_latch();
		uint32_t before = pas_direction_invalid_count();
		inv1(&s);
		CHECK(!is_latched(), "invalid (FIX): a lone illegal transition now cuts assist immediately");
		CHECK(live_target() == 0, "invalid (FIX): Iq cuts to 0 the SAME tick as the invalid transition");
		CHECK(pas_direction_invalid_count() == before + 1U, "invalid: still counted diagnostically");
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "invalid (FIX): session suspends, not COLD");
		for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
		CHECK(is_latched(), "invalid: fast-rearms after N confirmed forward steps with pressure present, same as 1R");
		CHECK(arm_snapshot_fast_rearm(), "invalid: flagged as fast_rearm");
	}

	/* --- a single formally-correct reverse GLITCH: behaves exactly like a real 1R ----------- */
	{
		reset_all();
		step_t s = establish_latch();
		rev1(&s);   /* indistinguishable from a real single reverse step, by design */
		CHECK(!is_latched(), "glitch: a single reverse glitch cuts Iq exactly like a real reverse");
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "glitch: suspended, not discarded");
		for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
		CHECK(is_latched(), "glitch: fast-rearms exactly like a real 1R");
	}

	/* --- R-INVALID-F: an invalid step arriving DURING confirmation must re-discard progress,
	 * exactly like a second reverse would - owner-mandated scenario, through the REAL chain. --- */
	{
		reset_all();
		step_t s = establish_latch();
		rev1(&s);
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "R-INVALID-F: setup - suspended after R");
		fwd1(&s);   /* first confirming forward step */
		inv1(&s);   /* invalid mid-confirmation: must discard the partial progress */
		CHECK(!is_latched(), "R-INVALID-F: still not latched after the invalid mid-confirmation");
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION,
			"R-INVALID-F: still suspended - the invalid step re-armed the inhibit");
		/* Confirming again needs the FULL count, no credit for the discarded forward step. */
		for (uint32_t i = 0; i + 1 < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) {
			fwd1(&s);
			CHECK(!is_latched(), "R-INVALID-F: still confirming the fresh run, not latched yet");
		}
		fwd1(&s);
		CHECK(is_latched(), "R-INVALID-F: fully re-confirmed after the invalid step and fast-rearms");
		CHECK(arm_snapshot_fast_rearm(), "R-INVALID-F: flagged as fast_rearm");
	}

	/* --- F-INVALID-F: an invalid step from a clean ACTIVE ride (not mid-confirmation) - owner-
	 * mandated scenario, through the REAL chain. ------------------------------------------------ */
	{
		reset_all();
		step_t s = establish_latch();
		CHECK(is_latched(), "F-INVALID-F: setup - latched, ordinary forward riding");
		inv1(&s);
		CHECK(!is_latched(), "F-INVALID-F: the invalid step alone cuts assist from a clean ACTIVE ride");
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "F-INVALID-F: suspended");
		for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
		CHECK(is_latched(), "F-INVALID-F: fast-rearms after the fresh forward run");
		CHECK(arm_snapshot_fast_rearm(), "F-INVALID-F: flagged as fast_rearm");
	}

	/* --- brake/overtemp/torque-fault arriving a TICK AFTER the session is already suspended (not
	 * on the same tick as the reverse/invalid event) - must still end the session -> COLD. ------ */
	{
		reset_all();
		step_t s = establish_latch();
		rev1(&s);
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "cut-during-suspension: setup - suspended");
		hold(&s, 5U);
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION,
			"cut-during-suspension: still suspended after a few quiet ticks - not itself terminal");
		s.non_direction_safety_cut = true;   /* brake/overtemp/torque-fault, folded as production does */
		hold(&s, 1U);
		CHECK(session_state() == RIDE_SESSION_COLD,
			"cut-during-suspension: a non-direction cut arriving LATER still ends the session -> COLD");
		CHECK(live_target() == 0, "cut-during-suspension: Iq stays 0");

		s.non_direction_safety_cut = false;
		s.torque_load_centikg = RIDING_START_LOAD_CENTIKG_DEFAULT + 20U;
		s.torque_assist_now_native = 200U;
		s.torque_assist_filtered = 200U;
		uint8_t need = tuning_config_start_steps();
		for (uint8_t i = 0; i < (uint8_t)(need + 2U); i++) fwd1(&s);
		hold(&s, 2U);
		CHECK(is_latched(), "cut-during-suspension: a normal cold start works again afterward");
		CHECK(!arm_snapshot_fast_rearm(), "cut-during-suspension: NORMAL arm, not fast_rearm");
	}

	/* --- COLD -> forward without meeting the normal thresholds: no fast-rearm, no cheap entry --
	 * even though fwd_run comfortably exceeds PAS_REVERSE_RECOVERY_CONFIRM_STEPS (the FAST-rearm
	 * confirm count), a COLD session structurally cannot reach WAIT_REARM_LOAD, so it can never
	 * borrow that cheaper gate - proven here against the real integrated chain, not just the
	 * module-level property (tests/host/ride_session_host.c's P6). ------------------------------ */
	{
		reset_all();
		step_t s = riding_step();
		s.speed_x100 = 0;   /* standstill: full threshold applies, no rolling reduction */
		s.torque_load_centikg = 0;             /* load threshold never met */
		s.torque_assist_now_native = 0;
		s.torque_assist_filtered = 0;
		uint32_t many = (uint32_t)PAS_REVERSE_RECOVERY_CONFIRM_STEPS + 50U;
		bool stayed_unlatched = true;
		for (uint32_t i = 0; i < many; i++) {
			fwd1(&s);
			if (is_latched()) stayed_unlatched = false;
		}
		CHECK(stayed_unlatched,
			"COLD-no-cheap-entry: many forward steps without the load threshold never latch, "
			"even though fwd_run far exceeds the fast-rearm confirm count");
		CHECK(session_state() == RIDE_SESSION_COLD, "COLD-no-cheap-entry: session never left COLD");
	}

	/* --- normal cold start from a real stop: UNCHANGED, for start_steps 1, 4 and MAX -------- */
	{
		reset_all();
		set_tuning_start_steps(4U);
		step_t s = riding_step();
		s.speed_x100 = 0;   /* genuine standstill */
		s.torque_assist_now_native = 200U;
		s.torque_assist_filtered = 200U;
		s.torque_load_centikg = RIDING_START_LOAD_CENTIKG_DEFAULT + 10U;   /* above riding, below standstill */
		for (int i = 0; i < 5; i++) fwd1(&s);
		CHECK(!is_latched(), "cold(4): below the STANDSTILL threshold, no arm");
		s.torque_load_centikg = STANDSTILL_LOAD_CENTIKG_DEFAULT + 10U;
		hold(&s, 5U);
		CHECK(is_latched(), "cold(4): crossing the full standstill threshold arms it normally");
		CHECK(!arm_snapshot_fast_rearm(), "cold(4): NORMAL arm");
		CHECK(session_state() == RIDE_SESSION_ACTIVE, "cold(4): ACTIVE");

		reset_all();
		set_tuning_start_steps(1U);
		s = riding_step();
		s.speed_x100 = 0;
		s.torque_assist_now_native = 200U;
		s.torque_assist_filtered = 200U;
		s.torque_load_centikg = STANDSTILL_LOAD_CENTIKG_DEFAULT + 10U;
		fwd1(&s);
		hold(&s, 2U);
		CHECK(is_latched(), "cold(1): a cold start still arms normally once the single required "
			"step and the full standstill threshold are both met");

		reset_all();
		set_tuning_start_steps(TUNING_START_STEPS_MAX);
		s = riding_step();
		s.speed_x100 = 0;
		s.torque_assist_now_native = 200U;
		s.torque_assist_filtered = 200U;
		s.torque_load_centikg = STANDSTILL_LOAD_CENTIKG_DEFAULT + 10U;
		for (unsigned i = 0; i < (unsigned)TUNING_START_STEPS_MAX - 1U; i++) fwd1(&s);
		CHECK(!is_latched(), "cold(MAX): one step short, no arm");
		fwd1(&s);
		hold(&s, 2U);
		CHECK(is_latched(), "cold(MAX): the MAXth step arms it");
	}

	/* --- level 0 / assist_off cancels a suspended session ----------------------------------- */
	{
		reset_all();
		step_t s = establish_latch();
		rev1(&s);
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "assist_off: setup - suspended");
		s.assist_level_index = 0;
		hold(&s, 1U);
		CHECK(session_state() == RIDE_SESSION_COLD, "assist_off: level 0 cancels the session -> COLD");
		CHECK(live_target() == 0, "assist_off: Iq stays 0");
	}

	/* --- the real torque_input.c filter lag: current sample 0, filter still positive. v2: this no
	 * longer BLOCKS permission - the confirm edge restores it unconditionally (pure direction
	 * fact), and demand honestly follows the real (decaying) filter: positive on the rearm tick
	 * (the recent pressure is still real to the 35 ms filter), then 0 once it fully decays and
	 * the hold grace - armed by the still-warm filter on the rearm tick - is exhausted, all while
	 * permission stays granted. The v1 premise this card's "must stay blocked" wording came from
	 * is gone. ---------------------------------------------------------------------------------- */
	{
		reset_all();
		step_t s = establish_latch();
		/* setup already drove the REAL chain to ~200; the current sample is genuinely positive. */
		const torque_snapshot_t *warm = torque_input_get_snapshot();
		CHECK(warm->assist_delta_native > 0, "filter-lag: setup - real current sample positive under load");

		s.torque_assist_filtered = 0U;   /* lift the pedal: feed 0 from now on */
		rev1(&s);
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "filter-lag: suspended after the reverse");
		const torque_snapshot_t *released = torque_input_get_snapshot();
		CHECK(released->assist_delta_native == 0, "filter-lag: setup - current sample reads 0 immediately");
		CHECK(released->assist_delta_filtered_native > 0, "filter-lag: setup - the 35 ms filter is STILL positive");

		for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
		CHECK(is_latched(), "filter-lag: v2 - permission restored on the confirm even though the current sample is 0");
		CHECK(live_target() > 0, "filter-lag: demand follows the real decaying filter - positive on the rearm tick");
		/* Keep pedalling with NO pressure: the real filter decays to ~0, demand falls with it and
		 * reaches 0 once the hold grace armed on the rearm tick is exhausted (1400 ms), while
		 * permission stays granted the whole time. */
		{
			bool demand_zeroed = false;
			for (uint32_t i = 0; i < 7000U; i++) {
				fwd1(&s);
				if (live_target() == 0) demand_zeroed = true;
			}
			CHECK(demand_zeroed, "filter-lag: demand collapsed to 0 after the real filter decayed and the hold grace expired");
		}
		CHECK(live_target() == 0, "filter-lag: demand is 0 - permission != demand");
		CHECK(is_latched(), "filter-lag: still latched - permission is not a demand guarantee");
		/* Fresh pressure resumes normally (recovery is a launch aid, not a latch-on): the rider
		 * presses again and keeps pedalling, the estimator refills by ordinary averaging, and
		 * full-magnitude demand comes back. */
		s.torque_assist_filtered = STRONG_ASSIST_DELTA_NATIVE;
		for (uint32_t i = 0; i < 300U; i++) fwd1(&s);
		CHECK(is_latched(), "filter-lag: pressure resume - still latched");
		CHECK(mode_iq_request() > FLOOR_ONLY_CEILING_IQ, "filter-lag: real assist on resume");
	}

	/* --- FW-109 v2 REQUIRED TEST: uint32_t tick wraparound through the REAL chain. v2 removed the
	 * WAIT/load-freshness anchor from ride_session.c entirely (neither the session automaton nor
	 * the recovery automaton keys on absolute ticks), so the wrap must simply leave the re-armed
	 * session and its demand intact. Pinned explicitly so the rearm edge itself lands at the
	 * wrap boundary. ---------------------------------------------------------------------------- */
	{
		reset_all();
		step_t s = establish_latch();
		s.torque_assist_filtered = 0U;
		rev1(&s);
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "wrap: setup - suspended");
		/* All but the LAST confirming forward step, at ordinary tick values. */
		for (uint32_t i = 0; i + 1 < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
		/* Pin the clock so the Nth (confirmation-completing) forward step lands at 0xFFFFFFFF -
		 * the rearm edge itself sits at the wrap boundary. Pressure is present again. */
		g_tick = 0xFFFFFFFEU;
		s.torque_assist_filtered = 200U;
		fwd1(&s);
		CHECK(is_latched(), "wrap: re-armed at the wrap boundary, pressure present");
		CHECK(session_state() == RIDE_SESSION_ACTIVE, "wrap: ACTIVE");
		/* g_tick wraps to 0, then 1 - the session and the recovery automaton must be untouched,
		 * and the real assist-mode output must keep flowing. */
		hold(&s, 1U);
		CHECK(is_latched(), "wrap: still latched one tick after the wrap");
		CHECK(session_state() == RIDE_SESSION_ACTIVE, "wrap: session intact across the wrap");
		CHECK(mode_iq_request() > FLOOR_ONLY_CEILING_IQ, "wrap: real assist-mode output through the real chain");
	}

	/* ==========================================================================================
	 * INTEGRATION: the REAL quadrature decoder feeding the REAL direction automaton
	 * ======================================================================================= */
	{
		reset_all();
		static const uint8_t forward_cycle[4] = {0, 2, 3, 1};   /* verified against pas_quadrature.c */
		uint8_t cycle_idx = 0;
		uint8_t qstate = forward_cycle[cycle_idx];

		step_t s = riding_step();
		s.torque_load_centikg = RIDING_START_LOAD_CENTIKG_DEFAULT + 20U;
		s.torque_assist_filtered = 200U;
		s.torque_assist_now_native = 200U;

		for (uint32_t i = 0; i < 400U; i++) {
			cycle_idx = (uint8_t)((cycle_idx + 1U) & 0x3U);
			uint8_t next = forward_cycle[cycle_idx];
			int8_t dir = pas_quadrature_step(qstate, next);
			CHECK(dir > 0, "integration: setup - the real decoder reports the cycle as forward");
			qstate = next;
			do_tick(&s, dir);
		}
		CHECK(is_latched(), "integration: real decoder + real automatons latched a genuine ride");
		CHECK(MS.i_q_setpoint > 0, "integration: real settled motor current flowing");

		/* 25 genuine reverse quadrature steps - back through the cycle. */
		for (uint32_t i = 0; i < 25U; i++) {
			uint8_t prev_idx = (uint8_t)((cycle_idx + 3U) & 0x3U);
			uint8_t prev_state = forward_cycle[prev_idx];
			int8_t rev_dir = pas_quadrature_step(qstate, prev_state);
			CHECK(rev_dir < 0, "integration: this transition really does decode as backward");
			qstate = prev_state;
			cycle_idx = prev_idx;
			do_tick(&s, rev_dir);
		}
		CHECK(live_target() == 0, "integration: the real chain cuts Iq through the whole 25-step reverse hold");
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "integration: real chain - suspended");

		for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) {
			cycle_idx = (uint8_t)((cycle_idx + 1U) & 0x3U);
			uint8_t next = forward_cycle[cycle_idx];
			int8_t dir = pas_quadrature_step(qstate, next);
			CHECK(dir > 0, "integration: the resumed cycle really does decode as forward");
			qstate = next;
			do_tick(&s, dir);
		}
		CHECK(is_latched(), "integration: real fast rearm through the whole real chain");
		CHECK(mode_iq_request() > FLOOR_ONLY_CEILING_IQ, "integration: real assist-mode output");
		hold(&s, 500U);
		CHECK(MS.i_q_setpoint > 0, "integration: real target current flowing again");

		/* FW-109 v2: a REAL illegal two-bit jump from the real decoder (the diagonal pair -
		 * qstate and its opposite corner, e.g. 0<->3 - impossible for genuine quadrature motion)
		 * must interrupt assist exactly like the reverse steps above did, through the whole
		 * real chain including the real assist_modes_calculate() commit check. */
		uint8_t diagonal = (uint8_t)(qstate ^ 0x3U);
		int8_t illegal_dir = pas_quadrature_step(qstate, diagonal);
		CHECK(illegal_dir == 0, "integration: setup - the real decoder reports the diagonal jump as illegal");
		do_tick(&s, EV_INVALID);
		CHECK(live_target() == 0, "integration: the real chain cuts Iq on a real illegal transition too");
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "integration: real chain - suspended by the invalid step");
		/* The decoder's own prev-state tracking would now see the diagonal state, wherever main.c
		 * happens to sample next - it is no longer at a known ring position, so recover using the
		 * ring's own next-forward-state map (derived from forward_cycle, not cycle_idx, which the
		 * diagonal jump has made meaningless) rather than assuming cycle_idx still tracks it. */
		{
			static const uint8_t next_forward[4] = { 2, 0, 3, 1 };   /* next_forward[qstate] -> real forward step */
			qstate = diagonal;
			for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) {
				uint8_t next = next_forward[qstate];
				int8_t dir = pas_quadrature_step(qstate, next);
				CHECK(dir > 0, "integration: recovery step after the illegal jump really does decode as forward");
				qstate = next;
				do_tick(&s, dir);
			}
		}
		CHECK(is_latched(), "integration: real fast rearm after a real illegal transition, same as after a reverse");
	}

	/* ==========================================================================================
	 * DETERMINISTIC GENERATIVE LONG-SEQUENCE TEST, seeded
	 * ======================================================================================= */
	{
		/* Tiny xorshift32 PRNG - deterministic across platforms/libc versions, unlike rand(). */
		uint32_t rng_state = 0xC0FFEE01U;   /* THE SEED - fixed, printed, reproducible */
		printf("  generative run: seed=0x%08X\n", (unsigned)rng_state);

		reset_all();
		step_t s = riding_step();

		const int GEN_TICKS = 20000;
		for (int t = 0; t < GEN_TICKS; t++) {
			rng_state ^= rng_state << 13; rng_state ^= rng_state >> 17; rng_state ^= rng_state << 5;
			uint32_t r = rng_state;

			/* Event mix: mostly idle, occasional forward, rarer reverse, rare invalid - closer
			 * to a real ride than a uniform 1/4 split, while still visiting every branch often. */
			uint32_t bucket = r % 100U;
			int event;
			if (bucket < 55U) event = EV_NONE;
			else if (bucket < 85U) event = EV_FORWARD;
			else if (bucket < 97U) event = EV_REVERSE;
			else event = EV_INVALID;

			/* Occasionally flip pressure, safety cut, real_stop, assist level - biased toward
			 * "off" so a genuine ride is still the common case, but every combination gets hit
			 * over 20000 ticks. */
			if ((r >> 8) % 37U == 0U) {
				bool on = ((r >> 15) & 1U) != 0U;
				s.torque_assist_now_native = on ? 200U : 0U;
				s.torque_assist_filtered = on ? 200U : (s.torque_assist_filtered > 2U ? s.torque_assist_filtered - 2U : 0U);
				s.torque_load_centikg = on ? (RIDING_START_LOAD_CENTIKG_DEFAULT + 20U) : 0U;
			}
			if ((r >> 9) % 401U == 0U) s.non_direction_safety_cut = !s.non_direction_safety_cut;
			if ((r >> 10) % 503U == 0U) s.real_stop = !s.real_stop;
			if (s.real_stop) s.pedaling_signal_present = false;
			else if ((r >> 11) % 97U == 0U) s.pedaling_signal_present = !s.pedaling_signal_present;
			if ((r >> 12) % 607U == 0U) {
				s.assist_level_index = (s.assist_level_index == 0U) ? TEST_ASSIST_LEVEL : 0U;
			}

			do_tick(&s, event);
			if (!g_invariants_ok) {
				printf("  generative run FAILED at tick %d (seed=0x%08X)\n", t, (unsigned)0xC0FFEE01U);
				break;
			}
		}
		CHECK(g_invariants_ok, "generative: all cross-module invariants held for 20000 random ticks");
		printf("  generative run: %d ticks completed, session ended in state %d\n",
			GEN_TICKS, (int)session_state());
	}

	if (host_test_failures == 0) {
		printf("All FW-109 v2 / FW-112 v2 integration checks passed.\n");
		return 0;
	}
	printf("\n%d FW-109 v2 / FW-112 v2 integration check(s) FAILED.\n", host_test_failures);
	return 1;
}
