/*
 * FW-112.1 INTEGRATION HOST HARNESS: REAL_STOP semantics during rolling direction suspend.
 *
 * Links the SHIPPED modules - real src/ride_control.c, src/ride_session.c, src/pas_direction.c,
 * src/pas_quadrature.c, src/rider_input.c, src/pas_liveness.c and their real dependency chain
 * (src/torque_input.c, src/assist_modes.c, src/cadence_comp.c, src/power_curve.c,
 * src/assist_start.c, src/assist_extended_boost.c, src/tuning_config.c, src/assist_dynamics.c,
 * src/assist_limits.c, src/motor_core.c) - the same full chain ride_control_rearm_host.c links,
 * PLUS the FW-112.1 liveness module.
 *
 * What this harness proves (the FW-112.1 card, scenario list S1..S11):
 *
 *   S1  a normal forward session is ACTIVE with real assist;
 *   S2  a single reverse step cuts permission the SAME tick (SUSPENDED_BY_DIRECTION, Iq 0) but
 *       is NOT a real stop;
 *   S3  sustained REVERSE activity for longer than the current real-stop timeout is still not a
 *       real stop - the session stays SUSPENDED (and COLD is never visited);
 *   S4  forward again after a long reverse session fast-rearms - ACTIVE, latched, flagged
 *       fast_rearm, real assist - with NO COLD in between;
 *   S5  reverse then a genuine stop (no edges at all) still goes COLD after the timeout -
 *       SUSPENDED is never stuck "alive" forever;
 *   S6  a plain forward stop is COLD after exactly (idle > timeout) ticks - the boundary;
 *   S7  INVALID (illegal two-bit) transitions with continued physical edges are not a real stop
 *       either - permission stays 0, assist stays 0, and the session stays SUSPENDED (this is
 *       the case old pas_idle_ticks-based real_stop got wrong - invalid steps never refreshed
 *       it, so it false-stopped a still-moving crank);
 *   S8  no edges at all is still a real stop even right after reverse/invalid activity - an old
 *       transition never refreshes the liveness timer;
 *   S9  reverse never produces positive assist demand (mode demand 0 / Iq 0 on every reverse
 *       tick, and the min-Iq floor does not leak either);
 *   S10 repeated rocking (R F F R F F ...) for longer than the timeout never false-stops - the
 *       session alternates SUSPENDED/ACTIVE without ever visiting COLD;
 *   S11 rocking then a genuine stop (no edges) still goes COLD after the timeout.
 *
 *   S12 (PRE-FLASH PROOF) sparse/slow reverse: forward establishes a session with the adaptive
 *       timeout driven to its minimum floor, then genuine reverse physical transitions keep
 *       coming but spaced SLOWER than the forward keep-alive rate that set the basis - and near
 *       the timeout boundary. Liveness must survive every reverse edge; no false REAL_STOP;
 *   S13 (PRE-FLASH PROOF) reverse inter-edge gap boundary at the ACTUAL runtime timeout T:
 *       gaps T-1 and T keep liveness alive, a gap of T+1 (idle exceeding T) fires REAL_STOP -
 *       that is the intended physical-stop contract (a no-edge interval LONGER than the stop
 *       interval IS a stop), reported exactly, not forced to pass;
 *   S14 (PRE-FLASH PROOF) REVERSE/INVALID alternation while SUSPENDED at a slow realistic
 *       cadence and again just below the boundary: liveness stays valid, permission stays 0,
 *       no assist, no false COLD.
 *
 * real_stop is NOT test-controlled here: it is COMPUTED each tick exactly the way main.c does it
 * now - a local mirror of pas_idle_ticks/pas_last_period_ticks and the adaptive
 * [PAS_STOP_TICKS..PAS_STOP_TICKS_MAX] timeout, plus the real pas_liveness module fed by the real
 * transitions. The harness therefore proves the PRODUCTION mechanism end to end, not a scenario
 * model of it.
 *
 * FW-112.2 note: this harness models NO wheel pulses, so rider_input.wheel_valid stays false the
 * whole run. That is intentional: the FW-112.2 terminal qualification (real_stop && !rolling_valid)
 * reduces to the pre-change real_stop exactly, so S1-S14 pin down the unchanged FW-112.1 contract
 * while the wheel-valid / coast dimension is covered by fw112_2_rolling_coast_host.c. A reverse
 * hold keeps pas_direction in DIRECTION_INHIBIT (pas_direction.c:84), so the FW-112.2
 * forward_pedaling rearm path can never fire mid-hold here either - the scenarios that exercise
 * that path live in the FW-112.2 harness.
 *
 * Mutations applied against production source and re-run (see the FW-112.1 report):
 *   M1  main.c calls pas_liveness_transition() only on FORWARD steps (old behaviour) -> S3/S7 FAIL
 *   M2  ride_session.c ignores real_stop while SUSPENDED                       -> S5/S8/S11 FAIL
 *   M3  pas_direction.c treats a reverse step like a forward step              -> S2/S3/S9 FAIL
 *   M4  pas_liveness.c refresh is latched by any past transition (never stops) -> S5/S8/S11 FAIL
 * After each FAIL is confirmed the mutation is reverted and the full suite re-run to ALL PASS.
 */

#include <stdint.h>
#include <string.h>

#include "../common/check.h"

#include "assist_modes.h"
#include "config.h"
#include "motor_core.h"
#include "pas_direction.h"
#include "pas_liveness.h"
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
/* The forward keep-alive period used to establish a session and to keep the liveness timer
 * refreshed while scenarios hold a state. The adaptive timeout becomes
 * clamp(2*(period+1), 800, 2000) = 802 ticks, so any gap <= period (401) is comfortably below
 * it and a genuine no-edge stop (>= 403 ticks) is comfortably above it. */
#define FWD_PERIOD 400U
#define FWD_GAP    (FWD_PERIOD + 1U)          /* ticks between consecutive forward steps */
#define TIMEOUT_TICKS ((uint16_t)(2U * FWD_GAP))  /* = 802, the clamped adaptive timeout */
#define HOLD_UNDER_TIMEOUT FWD_PERIOD          /* a gap guaranteed below the timeout */
#define HOLD_TO_STOP 2000U                     /* long enough to cross any timeout in scope */

static MotorState_t MS;
static uint32_t g_tick;   /* mirrors main.c's control_time_ticks */

/* --- FW-112.1 local mirrors of main.c's stop machinery (see main.c:2047-2283) ----------------- */
static uint16_t g_pas_idle;          /* pas_idle_ticks - cadence-gap + adaptive basis, untouched */
static uint16_t g_pas_last_period;   /* pas_last_period_ticks */
static uint16_t g_pas_stop_timeout;  /* pas_stop_timeout */
static bool     g_real_stop;         /* pas_real_stop - THE FW-112.1 verdict, from pas_liveness */
static bool     g_fwd_confirmed_this_tick;

/* event: 0=no transition this tick, +1=forward, -1=reverse, 2=illegal two-bit jump */
#define EV_NONE    0
#define EV_FORWARD 1
#define EV_REVERSE (-1)
#define EV_INVALID 2

typedef struct {
	uint16_t torque_load_centikg;   /* MODELED - the start-gate load, independently controllable */
	uint16_t torque_assist_filtered;/* desired steady-state assist pressure fed to the real chain */
	bool     non_direction_safety_cut;   /* brake / overtemp / torque fault / calibration, folded */
	uint8_t  assist_level_index;
	uint32_t speed_x100;
	bool     pedaling_signal_present;  /* main.c's (cadence>0||start_phase) half of crank_direction_ok */
	/* NOTE: no real_stop member - FW-112.1 computes it from the real pas_liveness module. */
} step_t;

static ride_session_state_t g_prev_session_state = RIDE_SESSION_COLD;
static bool g_invariants_ok = true;
static bool g_ever_cold;   /* scenario-local: did the session visit COLD at any point */

static int32_t live_target(void);
static bool is_latched(void);
static bool arm_snapshot_fast_rearm(void);
static int32_t mode_iq_request(void);
static ride_session_state_t session_state(void);

/* --- the PAS decode + real-stop block, mirroring main.c:2047-2283 ---------------------------------
 * The liveness module is the REAL production one (src/pas_liveness.c) and it is fed exactly the
 * way main.c feeds it: pas_liveness_transition() on EVERY physical edge (forward, reverse and
 * INVALID), pas_liveness_tick(stop_timeout) once per tick after the adaptive timeout is known. */
static void pas_chain(int event)
{
	if (g_pas_idle < 64000) g_pas_idle++;                       /* main.c:2049 */
	if (event != EV_NONE) {
		int8_t decoded_dir = (event == EV_INVALID) ? 0 : (int8_t)event;
		pas_liveness_transition();                               /* main.c:2059 - EVERY edge */
		if (decoded_dir > 0) {
			g_pas_last_period = g_pas_idle;                      /* main.c:2128 */
			g_pas_idle = 0;                                      /* main.c:2129 */
		} else if (decoded_dir < 0) {
			g_pas_idle = 0;                                      /* main.c:2215 */
		}
		/* decoded_dir == 0 (INVALID): g_pas_idle is NOT reset - faithful to main.c:2253-2266 */
		pas_direction_on_step(decoded_dir);                      /* main.c:2147/2227/2264 */
		g_fwd_confirmed_this_tick = pas_direction_forward_confirmed_last_call();
	} else {
		g_fwd_confirmed_this_tick = false;
	}
	/* adaptive stop timeout (main.c:2272-2276) */
	{
		uint32_t calc = (uint32_t)g_pas_last_period * 2U;
		if (calc < PAS_STOP_TICKS) calc = PAS_STOP_TICKS;
		else if (calc > PAS_STOP_TICKS_MAX) calc = PAS_STOP_TICKS_MAX;
		g_pas_stop_timeout = (uint16_t)calc;
	}
	pas_liveness_tick(g_pas_stop_timeout);                       /* main.c:2278 */
	g_real_stop = pas_liveness_stopped();                        /* main.c:2282 */
	if (g_real_stop) {
		pas_direction_on_stop();                                 /* main.c:2283 (session-relevant) */
	}
}

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

/* Cross-module invariants, checked after EVERY tick - the FW-109 invariant list #1-#8 plus the
 * FW-112.1 invariant: real_stop and COLD always land on the SAME tick (ride_session's terminal
 * wins unconditionally in every non-COLD state - ride_session.c:61,72,80). */
static void check_invariants(const step_t *s, int event)
{
	bool direction_inhibit = pas_direction_direction_inhibit_active();
	ride_session_state_t state = session_state();
	bool latched = is_latched();
	bool terminal_fed = s->non_direction_safety_cut || (s->assist_level_index == 0) || g_real_stop;
	(void)event;

	/* #1/#2: direction_inhibit_active => Iq=0 and not latched, unconditionally. */
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
	/* SUSPENDED_BY_DIRECTION => not latched. */
	if (state == RIDE_SESSION_SUSPENDED_BY_DIRECTION && latched) {
		CHECK(false, "INV: session SUSPENDED_BY_DIRECTION but latched");
		g_invariants_ok = false;
	}
	/* #3: a direction-inhibit event alone (no terminal fed) never sends ACTIVE straight to COLD. */
	if (g_prev_session_state == RIDE_SESSION_ACTIVE && state == RIDE_SESSION_COLD && !terminal_fed) {
		CHECK(false, "INV3: ACTIVE -> COLD with no terminal event fed this tick");
		g_invariants_ok = false;
	}
	/* #8: a terminal event fed this tick always ends a non-COLD session THIS tick. */
	if (terminal_fed && g_prev_session_state != RIDE_SESSION_COLD && state != RIDE_SESSION_COLD) {
		CHECK(false, "INV8: a terminal event was fed but the session did not end this tick");
		g_invariants_ok = false;
	}
	/* FW-109 v2: COLD can never gain fast_rearm. */
	if (g_prev_session_state == RIDE_SESSION_COLD && state == RIDE_SESSION_ACTIVE && arm_snapshot_fast_rearm()) {
		CHECK(false, "INV: COLD -> ACTIVE must never be flagged as fast_rearm");
		g_invariants_ok = false;
	}
	/* FW-112.1: real_stop and COLD land on the same tick. */
	if (g_real_stop && state != RIDE_SESSION_COLD) {
		CHECK(false, "INV: real_stop true but session not COLD the same tick");
		g_invariants_ok = false;
	}
	if (state == RIDE_SESSION_COLD) g_ever_cold = true;
	g_prev_session_state = state;
}

/* Drive the REAL torque_input.c chain to a desired steady-state assist pressure (same as
 * ride_control_rearm_host.c). */
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
	pas_chain(event);                       /* main.c:2047-2283 - real liveness verdict */
	if (event == EV_FORWARD) {
		torque_input_run_filter_step();     /* main.c:2158 - the RUN window advances on the step */
	}
	feed_pressure(s->torque_assist_filtered);   /* main.c:2329 - drive the real torque chain */

	const torque_snapshot_t *snap = torque_input_get_snapshot();
	bool direction_inhibit_active = pas_direction_direction_inhibit_active();
	uint8_t fwd_run = pas_direction_fwd_run();

	/* PRODUCTION FORMULA, main.c: crank_direction_ok = (cadence>0||start_phase) && !real_stop. */
	bool crank_direction_ok = s->pedaling_signal_present && !g_real_stop;
	/* PRODUCTION FORMULA, main.c: ride_core_pedaling = crank_direction_ok && fwd_run >=
	 * tuning_config_start_steps(). */
	bool pedaling = crank_direction_ok && (fwd_run >= tuning_config_start_steps());

	rider_input_t r;
	memset(&r, 0, sizeof(r));
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
	r.real_stop = g_real_stop;              /* FW-112.1: the computed verdict, not a test input */
	r.direction_inhibit_active = direction_inhibit_active;
	r.forward_confirmed_this_tick = g_fwd_confirmed_this_tick;
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
/* one forward step followed by a no-edge gap of `period` ticks - the keep-alive cycle that makes
 * the adaptive timeout basis converge to clamp(2*(period+1), PAS_STOP_TICKS..MAX). */
static void fwd_cycle(step_t *s, uint16_t period) { fwd1(s); hold(s, period); }

/* PRE-FLASH PROOF helper: drive one forward interval at gap=FWD_PERIOD so the adaptive basis
 * becomes exactly 2*FWD_PERIOD = 800 and the timeout clamps to its MINIMUM floor - the
 * adversarial case for the card: a fast-forward basis can never be smaller than 800 ticks, so a
 * sparse reverse can only ever be compared against the intended physical-stop floor. */
static void ensure_timeout_min(step_t *s)
{
	fwd1(s);
	hold(s, FWD_PERIOD - 1U);
	fwd1(s);                 /* last_period becomes 400 -> timeout = clamp(800,800,2000) = 800 */
}

/* Drive `cycles` reverse-edge cycles with edges spaced `gap` ticks apart; every per-edge and
 * per-gap-peak sample must show liveness alive, session SUSPENDED, permission 0 and demand 0. */
static bool reverse_gap_phase(step_t *s, uint16_t gap, uint32_t cycles)
{
	bool ok = true;
	for (uint32_t i = 0; i < cycles; i++) {
		rev1(s);
		if (g_real_stop) ok = false;
		if (session_state() != RIDE_SESSION_SUSPENDED_BY_DIRECTION) ok = false;
		if (live_target() != 0) ok = false;
		if (is_latched()) ok = false;
		if (mode_iq_request() != 0) ok = false;
		hold(s, gap - 1U);   /* the no-edge gap; the liveness idle peaks here at exactly `gap` */
		if (g_real_stop) ok = false;
		if (session_state() != RIDE_SESSION_SUSPENDED_BY_DIRECTION) ok = false;
		if (live_target() != 0) ok = false;
		if (is_latched()) ok = false;
		if (mode_iq_request() != 0) ok = false;
	}
	return ok;
}

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
	pas_liveness_init();           /* FW-112.1 */
	g_tick = 0;
	g_pas_idle = 0;
	g_pas_last_period = PAS_STOP_TICKS;
	g_pas_stop_timeout = PAS_STOP_TICKS;
	g_real_stop = false;
	g_fwd_confirmed_this_tick = false;
	g_prev_session_state = RIDE_SESSION_COLD;
	g_invariants_ok = true;
	g_ever_cold = false;
	set_tuning_start_steps(TUNING_START_STEPS_DEFAULT);
}

static step_t riding_step(void)
{
	step_t s;
	memset(&s, 0, sizeof(s));
	s.pedaling_signal_present = true;
	s.speed_x100 = TEST_SPEED_X100;
	s.assist_level_index = TEST_ASSIST_LEVEL;
	return s;
}

/* Ride forward (with load and pressure) until the latch is armed and settled, keeping the
 * liveness timer refreshed so the session survives the settle window. Returns the step to keep
 * driving. On return: session ACTIVE, latched, g_pas_idle = FWD_PERIOD, timeout = TIMEOUT_TICKS. */
static step_t establish_riding(void)
{
	step_t s = riding_step();
	s.torque_load_centikg = RIDING_START_LOAD_CENTIKG_DEFAULT + 20U;
	s.torque_assist_filtered = 200U;
	uint8_t need = tuning_config_start_steps();
	for (uint8_t i = 0; i < (uint8_t)(need + 2U); i++) fwd1(&s);   /* confirm the cold start */
	for (uint8_t i = 0; i < 8; i++) fwd_cycle(&s, FWD_PERIOD);     /* settle EMA + timeout basis */
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

/* --- S1: normal forward session ------------------------------------------------------------ */
static void test_s1_forward_session(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;                  /* the cold-start COLD ticks belong to setup, not the scenario */
	CHECK(session_state() == RIDE_SESSION_ACTIVE, "S1: establish - session ACTIVE");
	CHECK(is_latched(), "S1: establish - latched");
	CHECK(!g_real_stop, "S1: establish - no real stop");
	CHECK(live_target() != 0, "S1: establish - assist Iq flowing (the latch floor is the shipped cold-start contract)");
	fwd_cycle(&s, FWD_PERIOD); fwd_cycle(&s, FWD_PERIOD);
	CHECK(session_state() == RIDE_SESSION_ACTIVE, "S1: still ACTIVE after more forward cycles");
	CHECK(is_latched(), "S1: still latched");
	CHECK(!g_real_stop, "S1: forward activity never real-stops");
	CHECK(!g_ever_cold, "S1: never visited COLD");
}

/* --- S2: single reverse -> SUSPENDED, permission cut the same tick, NOT a real stop --------- */
static void test_s2_reverse_suspends(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	rev1(&s);
	CHECK(live_target() == 0, "S2: Iq cuts to 0 the SAME tick as the reverse");
	CHECK(!is_latched(), "S2: not latched");
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S2: SUSPENDED_BY_DIRECTION");
	CHECK(!g_real_stop, "S2: a single reverse is not a real stop");
	CHECK(!g_ever_cold, "S2: never visited COLD");
}

/* --- S3: sustained reverse activity longer than the timeout is still not a real stop -------- */
static void test_s3_reverse_liveness(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	CHECK(g_pas_stop_timeout == TIMEOUT_TICKS, "S3: adaptive timeout basis is 2x the forward gap");
	bool stayed_suspended = true, no_stop = true, iq_zero = true, not_latched = true;
	for (int i = 0; i < 10; i++) {             /* 10 x (1+400) = 4010 ticks > timeout=802 */
		rev1(&s);
		if (session_state() != RIDE_SESSION_SUSPENDED_BY_DIRECTION) stayed_suspended = false;
		if (g_real_stop) no_stop = false;
		if (live_target() != 0) iq_zero = false;
		if (is_latched()) not_latched = false;
		hold(&s, FWD_PERIOD);
		if (session_state() != RIDE_SESSION_SUSPENDED_BY_DIRECTION) stayed_suspended = false;
		if (g_real_stop) no_stop = false;
		if (live_target() != 0) iq_zero = false;
		if (is_latched()) not_latched = false;
	}
	CHECK(stayed_suspended, "S3: stayed SUSPENDED_BY_DIRECTION throughout the long reverse");
	CHECK(no_stop, "S3: reverse activity longer than the timeout never real-stopped");
	CHECK(iq_zero, "S3: Iq stayed 0 on every reverse tick");
	CHECK(not_latched, "S3: never latched during reverse");
	CHECK(!g_ever_cold, "S3: never visited COLD");
}

/* --- S4: forward after a long reverse fast-rearms with no COLD in between ------------------- */
static void test_s4_rearm_after_reverse(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	bool no_stop = true;
	for (int i = 0; i < 10; i++) {
		rev1(&s);
		if (g_real_stop) no_stop = false;
		hold(&s, FWD_PERIOD);
		if (g_real_stop) no_stop = false;
	}
	CHECK(no_stop, "S4: the long reverse never real-stopped");
	CHECK(!g_ever_cold, "S4: never visited COLD before the rearm");
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S4: SUSPENDED before the rearm");
	for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
	CHECK(session_state() == RIDE_SESSION_ACTIVE, "S4: forward confirms fast-rearm to ACTIVE");
	CHECK(is_latched(), "S4: latched");
	CHECK(arm_snapshot_fast_rearm(), "S4: flagged as fast_rearm, not a normal cold arm");
	CHECK(mode_iq_request() > FLOOR_ONLY_CEILING_IQ, "S4: real assist after the rolling rearm");
	CHECK(!g_ever_cold, "S4: the whole reverse + rearm never visited COLD");
}

/* --- S5: reverse then a genuine no-edge stop still goes COLD --------------------------------- */
static void test_s5_reverse_then_stop(void)
{
	reset_all();
	step_t s = establish_riding();
	rev1(&s);
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S5: SUSPENDED after the reverse");
	CHECK(!g_real_stop, "S5: reverse alone is not a real stop");
	hold(&s, FWD_PERIOD);                    /* still well under the timeout */
	CHECK(!g_real_stop, "S5: no real stop while the gap is under the timeout");
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S5: still SUSPENDED");
	hold(&s, HOLD_TO_STOP);                  /* genuine stop - no edges at all */
	CHECK(g_real_stop, "S5: real_stop fired once the crank genuinely stopped");
	CHECK(session_state() == RIDE_SESSION_COLD, "S5: the genuine stop sent SUSPENDED to COLD");
	CHECK(!is_latched(), "S5: not latched in COLD");
}

/* --- S6: plain forward stop - COLD at exactly (idle > timeout) ------------------------------ */
static void test_s6_forward_stop_boundary(void)
{
	reset_all();
	step_t s = establish_riding();           /* on return g_pas_idle = FWD_PERIOD, timeout=802 */
	CHECK(g_pas_idle == FWD_PERIOD, "S6: setup idle counter is exactly the keep-alive gap");
	CHECK(session_state() == RIDE_SESSION_ACTIVE, "S6: ACTIVE before the stop");
	hold(&s, TIMEOUT_TICKS - FWD_PERIOD - 1U);   /* idle reaches 802 exactly: NOT > 802 */
	CHECK(!g_real_stop, "S6: idle == timeout is still NOT a real stop (strict >)");
	CHECK(session_state() == RIDE_SESSION_ACTIVE, "S6: session still ACTIVE at idle == timeout");
	hold(&s, 1U);                            /* idle reaches 803 > 802: the real stop */
	CHECK(g_real_stop, "S6: real_stop fires the tick idle first exceeds the timeout");
	CHECK(session_state() == RIDE_SESSION_COLD, "S6: plain forward stop goes COLD");
	CHECK(!is_latched(), "S6: not latched in COLD");
}

/* --- S7: INVALID transitions with continued edges are not a real stop ----------------------- */
static void test_s7_invalid_liveness(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	inv1(&s);
	CHECK(live_target() == 0, "S7: INVALID cuts Iq the same tick");
	CHECK(!is_latched(), "S7: not latched");
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S7: INVALID suspends");
	CHECK(!g_real_stop, "S7: the first INVALID transition is not a real stop");
	bool stayed_suspended = true, no_stop = true, iq_zero = true, not_latched = true;
	for (int i = 0; i < 10; i++) {           /* 4010 ticks > timeout - edges keep coming */
		inv1(&s);
		if (session_state() != RIDE_SESSION_SUSPENDED_BY_DIRECTION) stayed_suspended = false;
		if (g_real_stop) no_stop = false;
		if (live_target() != 0) iq_zero = false;
		if (is_latched()) not_latched = false;
		hold(&s, FWD_PERIOD);
		if (session_state() != RIDE_SESSION_SUSPENDED_BY_DIRECTION) stayed_suspended = false;
		if (g_real_stop) no_stop = false;
		if (live_target() != 0) iq_zero = false;
		if (is_latched()) not_latched = false;
	}
	CHECK(stayed_suspended, "S7: stayed SUSPENDED throughout the INVALID transitions");
	CHECK(no_stop, "S7: INVALID edges longer than the timeout never real-stopped");
	CHECK(iq_zero, "S7: Iq stayed 0 on every INVALID tick");
	CHECK(not_latched, "S7: never latched during INVALID");
	CHECK(!g_ever_cold, "S7: never visited COLD - the pas_idle_ticks-era false stop is gone");
}

/* --- S8: no edges at all is still a real stop right after reverse/invalid ------------------- */
static void test_s8_no_edges_still_stops(void)
{
	reset_all();
	step_t s = establish_riding();
	inv1(&s);                                /* recent INVALID activity, then zero edges */
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S8: SUSPENDED after the INVALID");
	hold(&s, HOLD_TO_STOP);
	CHECK(g_real_stop, "S8: no edges at all still fires real_stop");
	CHECK(session_state() == RIDE_SESSION_COLD, "S8: SUSPENDED -> COLD after the real stop");
	CHECK(!is_latched(), "S8: not latched in COLD");
}

/* --- S9: reverse never produces positive assist demand -------------------------------------- */
static void test_s9_no_assist_on_reverse(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	bool iq_stayed_zero = true, floor_stayed_zero = true, demand_zero = true, stayed_suspended = true;
	for (int i = 0; i < 40; i++) {
		rev1(&s);
		if (live_target() != 0) iq_stayed_zero = false;
		if (iq_after_latch_floor() != 0) floor_stayed_zero = false;
		if (mode_iq_request() != 0) demand_zero = false;
		if (session_state() != RIDE_SESSION_SUSPENDED_BY_DIRECTION) stayed_suspended = false;
	}
	CHECK(iq_stayed_zero, "S9: Iq stayed 0 for every one of the 40 reverse steps");
	CHECK(floor_stayed_zero, "S9: the min-Iq floor never leaked in either");
	CHECK(demand_zero, "S9: positive assist-mode demand stayed 0 during reverse");
	CHECK(stayed_suspended, "S9: session stayed SUSPENDED for every reverse step");
	CHECK(!g_ever_cold, "S9: never visited COLD");
}

/* --- S10: repeated rocking (R F F R F F ...) longer than the timeout never false-stops ------- */
static void test_s10_rocking(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	bool no_stop = true, ok_suspended = true, ok_armed = true;
	for (int i = 0; i < 10; i++) {           /* 10 x ~603 ticks = ~6030 > timeout=802 */
		rev1(&s);
		if (g_real_stop) no_stop = false;
		if (session_state() != RIDE_SESSION_SUSPENDED_BY_DIRECTION) ok_suspended = false;
		hold(&s, HOLD_UNDER_TIMEOUT);
		if (g_real_stop) no_stop = false;
		fwd1(&s); fwd1(&s);                  /* PAS_REVERSE_RECOVERY_CONFIRM_STEPS = 2 */
		if (g_real_stop) no_stop = false;
		if (session_state() != RIDE_SESSION_ACTIVE) ok_armed = false;
		if (!is_latched()) ok_armed = false;
		if (!arm_snapshot_fast_rearm()) ok_armed = false;
		hold(&s, HOLD_UNDER_TIMEOUT);
		if (g_real_stop) no_stop = false;
	}
	CHECK(no_stop, "S10: rocking past the timeout never real-stopped");
	CHECK(!g_ever_cold, "S10: rocking never visited COLD");
	CHECK(ok_suspended, "S10: every reverse phase landed in SUSPENDED");
	CHECK(ok_armed, "S10: every forward phase fast-rearmed to ACTIVE and latched");
}

/* --- S11: rocking then a genuine stop still goes COLD ---------------------------------------- */
static void test_s11_rocking_then_stop(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	bool no_stop = true;
	for (int i = 0; i < 10; i++) {
		rev1(&s);
		if (g_real_stop) no_stop = false;
		hold(&s, HOLD_UNDER_TIMEOUT);
		if (g_real_stop) no_stop = false;
		fwd1(&s); fwd1(&s);
		if (g_real_stop) no_stop = false;
		hold(&s, HOLD_UNDER_TIMEOUT);
		if (g_real_stop) no_stop = false;
	}
	CHECK(no_stop, "S11: rocking never real-stopped while edges kept coming");
	CHECK(!g_ever_cold, "S11: rocking never visited COLD");
	rev1(&s);                                /* end the rocking in SUSPENDED, then a genuine stop */
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S11: SUSPENDED after the last reverse");
	CHECK(!g_real_stop, "S11: the last reverse itself is not a real stop");
	hold(&s, HOLD_TO_STOP);                  /* then a genuine stop - zero edges */
	CHECK(g_real_stop, "S11: the genuine stop after rocking fires real_stop");
	CHECK(session_state() == RIDE_SESSION_COLD, "S11: rocking then a stop goes COLD");
	CHECK(!is_latched(), "S11: not latched in COLD");
}

/* --- S12 (PRE-FLASH PROOF): slow reverse below the forward rate is still liveness ------------ */
static void test_s12_slow_reverse(void)
{
	reset_all();
	step_t s = establish_riding();
	ensure_timeout_min(&s);            /* fast forward basis driven to the exact floor: timeout=800 */
	uint16_t T = g_pas_stop_timeout;
	CHECK(T == PAS_STOP_TICKS, "S12: adaptive timeout driven to its minimum floor");
	uint16_t last_period_before = g_pas_last_period;
	g_ever_cold = false;
	rev1(&s);
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S12: SUSPENDED after the first reverse");
	CHECK(!g_real_stop, "S12: the first reverse is not a real stop");

	/* reverse edges spaced 500/600/790 ticks - all SLOWER than the 401-tick forward rate that set
	 * the basis, and 790 is just below the 800-tick floor timeout. */
	static const uint16_t gaps[] = { 500U, 600U, 790U };
	bool all_ok = true;
	for (uint8_t k = 0; k < 3U; k++) {
		if (!reverse_gap_phase(&s, gaps[k], 8U)) all_ok = false;
	}
	CHECK(all_ok, "S12: sparse/slow reverse below the forward rate never false-stopped");
	CHECK(!g_ever_cold, "S12: never visited COLD");
	CHECK(g_pas_last_period == last_period_before, "S12: pas_last_period is frozen during reverse (forward-only basis)");
	CHECK(g_pas_stop_timeout == T, "S12: the timeout stayed at the forward-derived floor the whole reverse");
	printf("  S12: reverse spacing %u/%u/%u ticks vs timeout %u ticks - all alive, no false REAL_STOP\n",
		(unsigned)gaps[0], (unsigned)gaps[1], (unsigned)gaps[2], (unsigned)T);
}

/* --- S13 (PRE-FLASH PROOF): reverse inter-edge gap boundary at the ACTUAL runtime timeout ----- */
static void test_s13_gap_boundary(void)
{
	reset_all();
	step_t s = establish_riding();
	ensure_timeout_min(&s);
	uint16_t T = g_pas_stop_timeout;   /* the ACTUAL runtime timeout, per the card */
	uint16_t last_period_before = g_pas_last_period;
	g_ever_cold = false;

	/* gap = T-1: the liveness idle peaks at T-1, still <= T -> alive. */
	rev1(&s);
	hold(&s, T - 2U);
	CHECK(!g_real_stop, "S13: gap=timeout-1 keeps liveness (peak idle == T-1)");
	CHECK(pas_liveness_idle_ticks() == (uint32_t)(T - 1U), "S13: peak liveness idle is exactly T-1");
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S13: SUSPENDED at gap=T-1");
	CHECK(live_target() == 0 && !is_latched(), "S13: permission still 0 at gap=T-1");

	/* gap = T: peak idle == T, and stopped is strict idle > T -> alive. */
	rev1(&s);
	hold(&s, T - 1U);
	CHECK(!g_real_stop, "S13: gap=timeout exactly keeps liveness (idle == T is NOT > T)");
	CHECK(pas_liveness_idle_ticks() == (uint32_t)T, "S13: peak liveness idle is exactly T");
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S13: SUSPENDED at gap=T");
	CHECK(live_target() == 0 && !is_latched(), "S13: permission still 0 at gap=T");

	/* gap = T+1: the idle exceeds T -> REAL_STOP. This is the CORRECT physical-stop semantics,
	 * not a false stop: a no-edge interval LONGER than the intended stop interval IS a stop, and
	 * here it fires one tick before the next reverse edge would have arrived. */
	rev1(&s);
	hold(&s, T);
	CHECK(g_real_stop, "S13: gap=timeout+1 fires REAL_STOP (idle reached T+1 > T)");
	CHECK(pas_liveness_idle_ticks() == (uint32_t)(T + 1U), "S13: peak liveness idle is exactly T+1");
	CHECK(session_state() == RIDE_SESSION_COLD, "S13: the >interval no-edge gap sent the session to COLD");
	CHECK(!is_latched(), "S13: not latched in COLD");

	CHECK(g_pas_last_period == last_period_before, "S13: pas_last_period unchanged by all reverse activity");
	printf("  S13: exact semantics with runtime T=%u ticks - alive for gap<=T, REAL_STOP the tick idle==T+1 (gap=T+1)\n",
		(unsigned)T);
}

/* --- S14 (PRE-FLASH PROOF): REVERSE/INVALID alternation while SUSPENDED is still liveness ----- */
static void test_s14_invalid_reverse_mix(void)
{
	reset_all();
	step_t s = establish_riding();
	ensure_timeout_min(&s);
	uint16_t T = g_pas_stop_timeout;
	g_ever_cold = false;

	bool mix_ok = true;
	const uint16_t mix_gap = 500U;   /* slow realistic cadence */
	for (uint32_t i = 0; i < 12; i++) {
		if (i & 1U) inv1(&s); else rev1(&s);   /* REVERSE, INVALID, REVERSE, INVALID ... */
		if (g_real_stop) mix_ok = false;
		if (session_state() != RIDE_SESSION_SUSPENDED_BY_DIRECTION) mix_ok = false;
		if (live_target() != 0) mix_ok = false;
		if (is_latched()) mix_ok = false;
		if (mode_iq_request() != 0) mix_ok = false;
		hold(&s, mix_gap - 1U);
		if (g_real_stop) mix_ok = false;
		if (session_state() != RIDE_SESSION_SUSPENDED_BY_DIRECTION) mix_ok = false;
		if (live_target() != 0) mix_ok = false;
		if (is_latched()) mix_ok = false;
	}
	CHECK(mix_ok, "S14: REVERSE/INVALID alternation at gap=500 kept liveness valid, permission 0, no assist");

	bool near_ok = true;
	const uint16_t near_gap = (uint16_t)(T - 10U);   /* just below the boundary */
	for (uint32_t i = 0; i < 6; i++) {
		if (i & 1U) rev1(&s); else inv1(&s);
		if (g_real_stop) near_ok = false;
		if (session_state() != RIDE_SESSION_SUSPENDED_BY_DIRECTION) near_ok = false;
		if (live_target() != 0) near_ok = false;
		if (is_latched()) near_ok = false;
		hold(&s, near_gap - 1U);
		if (g_real_stop) near_ok = false;
		if (session_state() != RIDE_SESSION_SUSPENDED_BY_DIRECTION) near_ok = false;
		if (live_target() != 0) near_ok = false;
		if (is_latched()) near_ok = false;
	}
	CHECK(near_ok, "S14: the mix at gap=T-10 (790 ticks) still never false-stopped");
	CHECK(!g_ever_cold, "S14: the REVERSE/INVALID mix never visited COLD");
	printf("  S14: REVERSE/INVALID mix gap=%u then gap=%u ticks vs timeout %u - no false COLD\n",
		(unsigned)mix_gap, (unsigned)near_gap, (unsigned)T);
}

int main(void)
{
	printf("FW-112.1 REAL_STOP liveness-separation integration, against the shipped modules\n");
	printf("  PAS_STOP_TICKS=%u  PAS_STOP_TICKS_MAX=%u  adaptive basis=%u -> timeout=%u ticks\n",
		(unsigned)PAS_STOP_TICKS, (unsigned)PAS_STOP_TICKS_MAX,
		(unsigned)FWD_GAP, (unsigned)TIMEOUT_TICKS);
	printf("  PAS_REVERSE_RECOVERY_CONFIRM_STEPS=%u  start_steps=%u\n",
		(unsigned)PAS_REVERSE_RECOVERY_CONFIRM_STEPS, (unsigned)TUNING_START_STEPS_DEFAULT);
	printf("PRE-FLASH PROOF (sparse/slow reverse): PAS control tick = 4 kHz (250 us/tick)\n");
	printf("  stop timeout floor = %u ticks = %u ms, ceiling = %u ticks = %u ms (forward-only adaptive basis)\n",
		(unsigned)PAS_STOP_TICKS, (unsigned)(PAS_STOP_TICKS / 4U),
		(unsigned)PAS_STOP_TICKS_MAX, (unsigned)(PAS_STOP_TICKS_MAX / 4U));

	test_s1_forward_session();
	test_s2_reverse_suspends();
	test_s3_reverse_liveness();
	test_s4_rearm_after_reverse();
	test_s5_reverse_then_stop();
	test_s6_forward_stop_boundary();
	test_s7_invalid_liveness();
	test_s8_no_edges_still_stops();
	test_s9_no_assist_on_reverse();
	test_s10_rocking();
	test_s11_rocking_then_stop();
	test_s12_slow_reverse();
	test_s13_gap_boundary();
	test_s14_invalid_reverse_mix();

	if (host_test_failures != 0) {
		printf("FW-112.1: %d FAILURE(S)\n", host_test_failures);
		return 1;
	}
	if (!g_invariants_ok) {
		printf("FW-112.1: invariant failures reported above\n");
		return 1;
	}
	printf("FW-112.1: ALL S1-S14 PASS\n");
	return 0;
}