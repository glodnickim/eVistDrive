/*
 * FW-112.2 INTEGRATION HOST HARNESS: REAL_STOP vs ROLLING COAST.
 *
 * Links the SHIPPED modules - the same full chain fw112_1_realstop_host.c links (real
 * ride_control.c, ride_session.c, pas_direction.c, pas_quadrature.c, rider_input.c,
 * pas_liveness.c + the torque/assist/tuning/motor dependency chain) PLUS the new production
 * wheel-freshness helper src/ride_wheel.c - so both the qualified terminal and the
 * forward_pedaling rearm are the REAL production decisions, driven end to end.
 *
 * The wheel is a MODEL like the PAS chain is a model, but it drives the REAL ride_wheel_valid()
 * the exact way main.c does: the harness maintains last_accepted_pulse_tick (the role of main.c's
 * speed_last_tick) and feeds rider_input.wheel_valid = ride_wheel_valid(now, last_pulse). While
 * g_wheel_rolling is true, a pulse fires every g_wheel_period_ticks (default well inside
 * SPEED_STOP_TICKS = 2.65 s), so the bike is demonstrably rolling; g_wheel_rolling=false is a
 * wheel at standstill, whose freshness expires SPEED_STOP_TICKS after the last pulse. real_stop
 * is NOT test-controlled - it is COMPUTED each tick from the real pas_liveness module, exactly as
 * main.c:2047-2283 wires it.
 *
 * Scenarios (the FW-112.2 card):
 *   S1  a normal forward session is ACTIVE with real assist (unchanged);
 *   S2  pedal release while the wheel is still rolling = a COAST: real_stop fires but the session
 *       is RETAINED in SUSPENDED_BY_DIRECTION - Iq 0, not latched, never COLD;
 *   S3  forward resume after a coast re-arms IMMEDIATELY on forward_pedaling - ACTIVE + fast
 *       rearm, latched, real assist, WITHOUT the cold-start gate (cold_start_ready explicitly
 *       false on the rearm tick) and WITHOUT a direction confirm edge;
 *   S4  reverse -> coast -> forward: reverse suspends, the coast holds SUSPENDED (no COLD), the
 *       confirm edge fast-rearms; no assist during reverse or coast;
 *   S5  INVALID -> coast -> forward: same contract as S4 for the illegal-transition path;
 *   S6  a TRUE forward stop (pedal release AND wheel at standstill): real_stop first retains the
 *       session as a coast while the wheel is still fresh, then the wheel loses its 2.65 s
 *       freshness and the session goes COLD;
 *   S7  a true stop after reverse: SUSPENDED -> wheel invalid -> COLD;
 *   S8  a LONG coast (wheel keeps pulsing for far longer than any PAS timeout) never goes COLD
 *       and never grants permission;
 *   S9  wheel-signal LOSS during an established coast: the wheel stops pulsing -> after
 *       SPEED_STOP_TICKS rolling_valid drops -> COLD;
 *   S10 the low-speed boundary of the production wheel signal: pulses at ~2.58 s intervals
 *       (< SPEED_STOP_TICKS) keep rolling_valid continuous with NO false COLD between pulses,
 *       and a pulse gap of exactly SPEED_STOP_TICKS turns the wheel invalid (the documented
 *       ~3 km/h floor - existing semantics, not a new threshold);
 *   S11 a complete stop then a cold start still goes through the ordinary cold arm (cold_start_
 *       ready -> ACTIVE), NOT a fast rearm;
 *   S12 during the whole coast the FINAL Iq stays 0 even while the torque chain still carries a
 *       full assist pressure (the coast never generates assist - latched=false forces the final
 *       gate to zero, which also structurally closes the assist-without-rotation hole, since that
 *       feature only ever flows while latched);
 *   S13 repeated coast/resume cycles never visit COLD and re-arm every time;
 *   S14 repeated rocking (F R coast F INVALID coast F ...) longer than any timeout never
 *       false-stops - SUSPENDED/ACTIVE alternation, no COLD, no assist on reverse/invalid/coast.
 *
 * Mutations applied against production source and re-run (M1..M5, see the FW-112.2 report):
 *   M1  ride_session.c drops the wheel qualification (terminal = real_stop) -> S2..S10/S12/S13 FAIL
 *       (every real_stop immediately COLDs a still-rolling bike; only S1 pure-forward, S11 true
 *       stop and S14 pure alternating pass);
 *   M2  ride_session.c re-arms on rolling_valid alone (wheel grants permission) -> S4/S5/S7/S12
 *       FAIL (permission flows with no forward evidence - reverse->coast->forward, INVALID->coast
 *       and the long pure coast wrongly rearm or leave latched);
 *   M3  ride_session.c never COLDs on !rolling_valid during a coast -> S6/S7/S9/S11 FAIL (a true
 *       stop never ends the ride);
 *   M4  ride_wheel.c latches the last pulse (valid forever) -> S10 FAIL (the SPEED_STOP_TICKS
 *       freshness boundary - invalid never becomes true - is what breaks first; S9's COLD checks
 *       happen to still pass here because ride_wheel_valid is only sampled, never fed back);
 *   M5  ride_session.c fast-rearms from COLD on cold_start_ready (cold-start bypass) -> detected
 *       by the SESSION-CONTRACT harness ride_session_host (P3/P4), NOT by this system harness:
 *       ride_control.c's arm path is `if (cold_arm_this_tick) ... fast_rearm=false; else if
 *       (fast_rearm_this_tick) ...` so on a COLD->ACTIVE tick cold_arm is authoritative and the
 *       corrupted edge is stamped false in the arm snapshot (the system-observable S11 assertion
 *       cannot see it; the mutation is behaviorally inert through the ride path). S11's assertion
 *       of the CONTRACT ("a cold start is NOT a fast rearm") holds at the session level.
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
#include "ride_wheel.h"
#include "torque_input.h"
#include "tuning_config.h"

#define TEST_ASSIST_LEVEL 3U
#define TEST_SPEED_X100   1500U   /* 15.00 km/h: comfortably "rolling" (>= 100) */
#define TEST_BATTERY_MV   42000U
#define TEST_VOLTAGE_RAW  2000
#define TEST_TEMPERATURE_C 25
#define RIDING_START_LOAD_CENTIKG_DEFAULT 30U
#define FLOOR_ONLY_CEILING_IQ 60
#define FWD_PERIOD 400U
#define FWD_GAP    (FWD_PERIOD + 1U)
#define TIMEOUT_TICKS ((uint16_t)(2U * FWD_GAP))  /* = 802, the clamped adaptive timeout */
#define HOLD_UNDER_TIMEOUT FWD_PERIOD
#define HOLD_TO_STOP 2000U
/* The default coasting-wheel pulse interval - 2.5 s @ 4 kHz, comfortably inside SPEED_STOP_TICKS
 * (2.65 s) so a coast is unambiguously "rolling" for S2/S3/S4/S8/S12/S13. */
#define WHEEL_COAST_PERIOD (SPEED_TIMEBASE_HZ * 2500U / 1000U)   /* 10000 ticks */

static MotorState_t MS;
static uint32_t g_tick;

/* --- wheel model (mirrors main.c's speed_last_tick / ride_wheel_valid wiring) ----------------- */
static uint32_t g_last_pulse;      /* control tick of the last accepted wheel edge */
static bool     g_wheel_rolling;   /* model: the physical wheel is turning */
static uint32_t g_wheel_period;    /* accepted-pulse interval when rolling */

static void wheel_pulse(void) { g_last_pulse = g_tick; }
static bool wheel_valid_now(void) { return ride_wheel_valid(g_tick, g_last_pulse); }

/* --- FW-112.1 local mirrors of main.c's stop machinery (see main.c:2047-2283) ----------------- */
static uint16_t g_pas_idle;
static uint16_t g_pas_last_period;
static uint16_t g_pas_stop_timeout;
static bool     g_real_stop;
static bool     g_fwd_confirmed_this_tick;

#define EV_NONE    0
#define EV_FORWARD 1
#define EV_REVERSE (-1)
#define EV_INVALID 2

typedef struct {
	uint16_t torque_load_centikg;
	uint16_t torque_assist_filtered;
	bool     non_direction_safety_cut;
	uint8_t  assist_level_index;
	uint32_t speed_x100;
	bool     pedaling_signal_present;
} step_t;

static ride_session_state_t g_prev_session_state = RIDE_SESSION_COLD;
static bool g_invariants_ok = true;
static bool g_ever_cold;

static int32_t live_target(void);
static bool is_latched(void);
static bool arm_snapshot_fast_rearm(void);
static int32_t mode_iq_request(void);
static ride_session_state_t session_state(void);

/* --- the PAS decode + real-stop block, mirroring main.c:2047-2283 ------------------------------ */
static void pas_chain(int event)
{
	if (g_pas_idle < 64000) g_pas_idle++;
	if (event != EV_NONE) {
		int8_t decoded_dir = (event == EV_INVALID) ? 0 : (int8_t)event;
		pas_liveness_transition();
		if (decoded_dir > 0) {
			g_pas_last_period = g_pas_idle;
			g_pas_idle = 0;
		} else if (decoded_dir < 0) {
			g_pas_idle = 0;
		}
		pas_direction_on_step(decoded_dir);
		g_fwd_confirmed_this_tick = pas_direction_forward_confirmed_last_call();
	} else {
		g_fwd_confirmed_this_tick = false;
	}
	{
		uint32_t calc = (uint32_t)g_pas_last_period * 2U;
		if (calc < PAS_STOP_TICKS) calc = PAS_STOP_TICKS;
		else if (calc > PAS_STOP_TICKS_MAX) calc = PAS_STOP_TICKS_MAX;
		g_pas_stop_timeout = (uint16_t)calc;
	}
	pas_liveness_tick(g_pas_stop_timeout);
	g_real_stop = pas_liveness_stopped();
	if (g_real_stop) {
		pas_direction_on_stop();
	}
}

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

/* Cross-module invariants, checked after EVERY tick. FW-112.2-rewritten: real_stop alone is NO
 * LONGER required to COLD the session the same tick - it may retain a still-rolling session in
 * SUSPENDED (a coast). The terminal now is the qualified real_stop && !wheel_valid, and that
 * still lands COLD the same tick. */
static void check_invariants(const step_t *s, int event)
{
	bool direction_inhibit = pas_direction_direction_inhibit_active();
	ride_session_state_t state = session_state();
	bool latched = is_latched();
	bool wheel_valid = wheel_valid_now();
	bool terminal_fed = s->non_direction_safety_cut || (s->assist_level_index == 0) ||
	                    (g_real_stop && !wheel_valid);
	(void)event;

	if (direction_inhibit) {
		if (live_target() != 0) { CHECK(false, "INV1: direction_inhibit_active but live_target != 0"); g_invariants_ok = false; }
		if (latched) { CHECK(false, "INV2: direction_inhibit_active but latched"); g_invariants_ok = false; }
	}
	if (!latched && live_target() != 0) {
		CHECK(false, "INV: not latched but live_target != 0");
		g_invariants_ok = false;
	}
	if (state == RIDE_SESSION_COLD && latched) {
		CHECK(false, "INV: session COLD but latched");
		g_invariants_ok = false;
	}
	if (state == RIDE_SESSION_SUSPENDED_BY_DIRECTION && latched) {
		CHECK(false, "INV: session SUSPENDED_BY_DIRECTION but latched");
		g_invariants_ok = false;
	}
	if (g_prev_session_state == RIDE_SESSION_ACTIVE && state == RIDE_SESSION_COLD && !terminal_fed) {
		CHECK(false, "INV3: ACTIVE -> COLD with no terminal event fed this tick");
		g_invariants_ok = false;
	}
	if (terminal_fed && g_prev_session_state != RIDE_SESSION_COLD && state != RIDE_SESSION_COLD) {
		CHECK(false, "INV8: a terminal event was fed but the session did not end this tick");
		g_invariants_ok = false;
	}
	if (g_prev_session_state == RIDE_SESSION_COLD && state == RIDE_SESSION_ACTIVE && arm_snapshot_fast_rearm()) {
		CHECK(false, "INV: COLD -> ACTIVE must never be flagged as fast_rearm");
		g_invariants_ok = false;
	}
	/* FW-112.2: the QUALIFIED terminal (real_stop && !wheel_valid) always COLDs the same tick. */
	if (g_real_stop && !wheel_valid && state != RIDE_SESSION_COLD) {
		CHECK(false, "INV: qualified terminal true but session not COLD the same tick");
		g_invariants_ok = false;
	}
	if (state == RIDE_SESSION_COLD) g_ever_cold = true;
	g_prev_session_state = state;
}

static void feed_pressure(uint16_t filtered_target)
{
	uint32_t delta = (uint32_t)filtered_target + TORQUE_ASSIST_DEADBAND_NATIVE;
	uint32_t raw = (uint32_t)TORQUE_ZERO_TARGET_NATIVE + delta;
	if (raw > TORQUE_SPAN_MAX_NATIVE) raw = TORQUE_SPAN_MAX_NATIVE;
	torque_input_update((uint16_t)raw, torque_input_correct((uint16_t)raw), true);
}

static void do_tick(step_t *s, int event)
{
	g_tick++;
	if (g_wheel_rolling && (g_tick - g_last_pulse) >= g_wheel_period) wheel_pulse();
	pas_chain(event);
	if (event == EV_FORWARD) {
		torque_input_run_filter_step();
	}
	feed_pressure(s->torque_assist_filtered);

	const torque_snapshot_t *snap = torque_input_get_snapshot();
	bool direction_inhibit_active = pas_direction_direction_inhibit_active();
	uint8_t fwd_run = pas_direction_fwd_run();

	/* PRODUCTION FORMULA, main.c: crank_direction_ok = (cadence>0||start_phase) && !real_stop. */
	bool crank_direction_ok = s->pedaling_signal_present && !g_real_stop;
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
	r.real_stop = g_real_stop;
	r.wheel_valid = wheel_valid_now();          /* FW-112.2: ride_wheel_valid(), as main.c:2375 */
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
static void fwd_cycle(step_t *s, uint16_t period) { fwd1(s); hold(s, period); }

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
	pas_liveness_init();
	g_tick = 0;
	g_last_pulse = 0;
	g_wheel_rolling = true;
	g_wheel_period = WHEEL_COAST_PERIOD;
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

static step_t establish_riding(void)
{
	step_t s = riding_step();
	s.torque_load_centikg = RIDING_START_LOAD_CENTIKG_DEFAULT + 20U;
	s.torque_assist_filtered = 200U;
	uint8_t need = tuning_config_start_steps();
	for (uint8_t i = 0; i < (uint8_t)(need + 2U); i++) fwd1(&s);
	for (uint8_t i = 0; i < 8; i++) fwd_cycle(&s, FWD_PERIOD);
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
	g_ever_cold = false;
	CHECK(session_state() == RIDE_SESSION_ACTIVE, "S1: establish - session ACTIVE");
	CHECK(is_latched(), "S1: establish - latched");
	CHECK(!g_real_stop, "S1: establish - no real stop");
	CHECK(live_target() != 0, "S1: establish - assist Iq flowing");
	fwd_cycle(&s, FWD_PERIOD); fwd_cycle(&s, FWD_PERIOD);
	CHECK(session_state() == RIDE_SESSION_ACTIVE, "S1: still ACTIVE after more forward cycles");
	CHECK(is_latched(), "S1: still latched");
	CHECK(!g_real_stop, "S1: forward activity never real-stops");
	CHECK(!g_ever_cold, "S1: never visited COLD");
}

/* --- S2: pedal release while rolling = COAST (retained, not latched, never COLD) ------------- */
static void test_s2_coast_retains(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	step_t coast = s;
	coast.pedaling_signal_present = false;      /* the rider lets go of the pedals */
	/* keep the wheel rolling (g_wheel_rolling already true) and wait for the real stop verdict */
	hold(&coast, HOLD_TO_STOP);
	CHECK(g_real_stop, "S2: the PAS timeout fired (real_stop)");
	CHECK(wheel_valid_now(), "S2: the wheel is still rolling");
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION,
		"S2: the coast RETAINS the session in SUSPENDED_BY_DIRECTION");
	CHECK(!is_latched(), "S2: the coast never grants permission");
	CHECK(live_target() == 0, "S2: Iq is 0 through the coast");
	CHECK(!g_ever_cold, "S2: the coast never visits COLD");
	/* and it stays that way - a long coast keeps the session retained, not ended */
	hold(&coast, HOLD_TO_STOP);
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S2: still retained after a long coast");
	CHECK(!g_ever_cold, "S2: never COLD through the whole coast");
}

/* --- S3: forward resume after a coast re-arms on forward_pedaling, NO cold-start gate --------- */
static void test_s3_coast_resume_no_cold_gate(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	step_t coast = s;
	coast.pedaling_signal_present = false;
	hold(&coast, HOLD_TO_STOP);
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S3: coast entered");
	CHECK(g_real_stop, "S3: real_stop holds through the coast");

	/* resume: the rider genuinely pedals forward again. On the resume tick cold_start_ready is
	 * deliberately FALSE (zero torque load below the engage threshold, fwd_run rebuilt from 0), so
	 * if ACTIVE were reached it MUST be the forward_pedaling rearm path, not the cold gate. */
	step_t resume = coast;
	resume.pedaling_signal_present = true;
	resume.torque_load_centikg = 0;            /* below the engage threshold -> cold_start_ready false */
	fwd1(&resume);                              /* the first real forward edge */
	CHECK(!g_real_stop, "S3: the forward edge cleared real_stop the same tick");
	CHECK(session_state() == RIDE_SESSION_ACTIVE, "S3: resume re-armed ACTIVE immediately");
	CHECK(is_latched(), "S3: latched");
	CHECK(arm_snapshot_fast_rearm(), "S3: flagged as fast_rearm (the rolling rearm, NOT a cold arm)");
	CHECK(!g_ever_cold, "S3: the whole coast + resume never visited COLD");
	/* real assist flows again on the resumed ride */
	uint8_t need = tuning_config_start_steps();
	for (uint8_t i = 0; i < (uint8_t)(need + 2U); i++) fwd1(&resume);
	resume.torque_load_centikg = RIDING_START_LOAD_CENTIKG_DEFAULT + 20U;
	fwd_cycle(&resume, FWD_PERIOD);
	CHECK(session_state() == RIDE_SESSION_ACTIVE, "S3: resumed ride stays ACTIVE");
	CHECK(mode_iq_request() > FLOOR_ONLY_CEILING_IQ, "S3: real assist after the coast resume");
}

/* --- S4: reverse -> coast -> forward (no COLD, no assist during reverse/coast) ----------------- */
static void test_s4_reverse_coast_forward(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	bool iq_zero = true;
	rev1(&s);                                   /* reverse suspends immediately */
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S4: reverse suspends");
	CHECK(!g_real_stop, "S4: the reverse alone is not a real stop");
	if (live_target() != 0) iq_zero = false;
	/* reverse then a coast: no more PAS edges, wheel still rolling -> real_stop fires, session is
	 * retained in SUSPENDED (never COLD), permission and Iq stay 0. */
	step_t coast = s;
	coast.pedaling_signal_present = false;
	hold(&coast, HOLD_TO_STOP);
	CHECK(g_real_stop, "S4: real_stop fired during the reverse->coast hold");
	CHECK(wheel_valid_now(), "S4: the wheel is still rolling");
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S4: still SUSPENDED after the coast");
	CHECK(live_target() == 0, "S4: Iq 0 through the coast");
	CHECK(!g_ever_cold, "S4: no COLD through reverse + coast");
	/* forward again: the confirm edge fast-rearms (no cold gate). */
	for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
	CHECK(session_state() == RIDE_SESSION_ACTIVE, "S4: the confirm edge fast-rearms to ACTIVE");
	CHECK(is_latched(), "S4: latched");
	CHECK(arm_snapshot_fast_rearm(), "S4: fast rearm flagged");
	CHECK(!g_ever_cold, "S4: the whole reverse + coast + rearm never visited COLD");
	CHECK(iq_zero, "S4: no assist during the reverse phase");
}

/* --- S5: INVALID -> coast -> forward (the illegal-transition path, same contract) -------------- */
static void test_s5_invalid_coast_forward(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	inv1(&s);                                   /* INVALID suspends immediately */
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S5: INVALID suspends");
	CHECK(live_target() == 0, "S5: INVALID cuts Iq the same tick");
	CHECK(!g_real_stop, "S5: the INVALID alone is not a real stop");
	step_t coast = s;
	coast.pedaling_signal_present = false;
	hold(&coast, HOLD_TO_STOP);
	CHECK(g_real_stop, "S5: real_stop fired during the INVALID->coast hold");
	CHECK(wheel_valid_now(), "S5: the wheel is still rolling");
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S5: still SUSPENDED through the coast");
	CHECK(live_target() == 0, "S5: Iq 0 through the coast");
	CHECK(!g_ever_cold, "S5: no COLD through INVALID + coast");
	for (uint32_t i = 0; i < PAS_REVERSE_RECOVERY_CONFIRM_STEPS; i++) fwd1(&s);
	CHECK(session_state() == RIDE_SESSION_ACTIVE, "S5: forward fast-rearms to ACTIVE");
	CHECK(arm_snapshot_fast_rearm(), "S5: fast rearm flagged");
	CHECK(!g_ever_cold, "S5: never visited COLD");
}

/* --- S6: a TRUE forward stop (pedal release AND wheel standstill) ends in COLD ----------------- */
static void test_s6_true_stop_forward(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	step_t stopped = s;
	stopped.pedaling_signal_present = false;
	g_wheel_rolling = false;                   /* the wheel also comes to a standstill */
	hold(&stopped, HOLD_TO_STOP);
	CHECK(g_real_stop, "S6: real_stop fired");
	/* the wheel is STILL fresh (<= 2.65 s since its last accepted pulse), so the qualified
	 * terminal has not fired yet - the session was first RETAINED as a short coast. */
	CHECK(wheel_valid_now(), "S6: the wheel is still within its 2.65 s freshness");
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION,
		"S6: the true stop first retains the session (coast) until the wheel goes invalid");
	CHECK(!is_latched(), "S6: not latched");
	/* the conservative wheel timeout expires -> qualified terminal -> COLD. */
	uint32_t remain = SPEED_STOP_TICKS - (g_tick - g_last_pulse) + 1U;
	hold(&stopped, remain);
	CHECK(!wheel_valid_now(), "S6: the wheel lost its 2.65 s freshness");
	CHECK(g_real_stop, "S6: real_stop still holds");
	CHECK(session_state() == RIDE_SESSION_COLD, "S6: the true stop ended the ride in COLD");
	CHECK(!is_latched(), "S6: not latched in COLD");
}

/* --- S7: a true stop after reverse (SUSPENDED -> wheel invalid -> COLD) ------------------------- */
static void test_s7_true_stop_after_reverse(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	rev1(&s);
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S7: reverse suspends");
	step_t stopped = s;
	stopped.pedaling_signal_present = false;
	g_wheel_rolling = false;
	hold(&stopped, HOLD_TO_STOP);
	CHECK(g_real_stop, "S7: real_stop fired after the reverse");
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S7: retained while the wheel is fresh");
	uint32_t remain = SPEED_STOP_TICKS - (g_tick - g_last_pulse) + 1U;
	hold(&stopped, remain);
	CHECK(!wheel_valid_now(), "S7: wheel invalid");
	CHECK(session_state() == RIDE_SESSION_COLD, "S7: SUSPENDED -> COLD after the wheel went invalid");
	CHECK(!is_latched(), "S7: not latched in COLD");
}

/* --- S8: a LONG coast (wheel keeps rolling) never COLDs and never grants permission ------------ */
static void test_s8_long_coast(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	step_t coast = s;
	coast.pedaling_signal_present = false;
	g_wheel_rolling = true;                    /* the bike keeps rolling the whole time */
	uint32_t total = 5U * SPEED_STOP_TICKS;    /* ~13 s - far beyond any PAS timeout */
	hold(&coast, total);
	CHECK(g_real_stop, "S8: real_stop holds through the long coast");
	CHECK(wheel_valid_now(), "S8: the rolling wheel stays valid the whole coast");
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S8: still retained after ~13 s");
	CHECK(!is_latched(), "S8: never granted permission");
	CHECK(live_target() == 0, "S8: Iq 0 the whole time");
	CHECK(!g_ever_cold, "S8: the long coast never visited COLD");
}

/* --- S9: wheel-signal LOSS during an established coast ends the ride --------------------------- */
static void test_s9_wheel_loss_during_coast(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	step_t coast = s;
	coast.pedaling_signal_present = false;
	g_wheel_rolling = true;
	hold(&coast, HOLD_TO_STOP);
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S9: coast established");
	CHECK(wheel_valid_now(), "S9: wheel valid before the loss");
	/* the wheel signal is lost (sensor dead / stopped wheel): no more pulses. */
	g_wheel_rolling = false;
	hold(&coast, SPEED_STOP_TICKS + 1U);
	CHECK(!wheel_valid_now(), "S9: wheel invalid after the loss window");
	CHECK(g_real_stop, "S9: real_stop still holds");
	CHECK(session_state() == RIDE_SESSION_COLD, "S9: wheel loss during the coast -> COLD");
	CHECK(!is_latched(), "S9: not latched in COLD");
}

/* --- S10: the low-speed boundary of the production wheel signal --------------------------------- */
static void test_s10_wheel_boundary(void)
{
	reset_all();
	/* Direct ride_wheel_valid() boundary: strictly < SPEED_STOP_TICKS is valid, == is not. */
	g_last_pulse = 1000U;
	CHECK(ride_wheel_valid(1000U + SPEED_STOP_TICKS - 1U, 1000U), "S10: valid at gap SPEED_STOP_TICKS-1");
	CHECK(!ride_wheel_valid(1000U + SPEED_STOP_TICKS, 1000U), "S10: invalid at gap exactly SPEED_STOP_TICKS");
	CHECK(!ride_wheel_valid(1000U + SPEED_STOP_TICKS + 1U, 1000U), "S10: invalid at gap SPEED_STOP_TICKS+1");

	/* Slowest trustworthy rolling: pulses every 10300 ticks (2.575 s @ 4 kHz, ~3.1 km/h @ 2218 mm),
	 * just inside the 10600-tick window - rolling_valid must never drop between pulses, and a
	 * sustained coast at that speed must never false-COLD. */
	g_wheel_rolling = true;
	g_wheel_period = SPEED_STOP_TICKS - 300U;  /* 10300 < 10600 */
	step_t s = establish_riding();
	g_ever_cold = false;
	step_t coast = s;
	coast.pedaling_signal_present = false;
	uint32_t cycles = 6U;
	bool valid_continuous = true;
	for (uint32_t i = 0; i < cycles; i++) {
		hold(&coast, g_wheel_period / 2U);     /* sample mid-interval: must still be rolling */
		if (!wheel_valid_now()) valid_continuous = false;
		hold(&coast, g_wheel_period / 2U);
		if (!wheel_valid_now()) valid_continuous = false;
	}
	CHECK(valid_continuous, "S10: 2.58 s pulse cadence keeps rolling_valid continuous (no false COLD)");
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S10: still SUSPENDED at the slow rolling speed");
	CHECK(!g_ever_cold, "S10: the low-speed coast never visited COLD");

	/* A pulse cadence AT the window (gap == SPEED_STOP_TICKS) is NOT trustworthy: it flips the
	 * wheel invalid exactly between pulses - the existing ~3 km/h production floor, unchanged. */
	g_wheel_rolling = false;
	hold(&coast, SPEED_STOP_TICKS + 1U);
	CHECK(!wheel_valid_now(), "S10: below the floor the wheel is invalid (existing semantics)");
	printf("  S10: SPEED_STOP_TICKS=%u ticks = %.3f s; min trusted speed ~%.1f km/h (1 pulse/rev @ 2218 mm)\n",
		(unsigned)SPEED_STOP_TICKS, (double)SPEED_STOP_TICKS / (double)SPEED_TIMEBASE_HZ,
		2218.0 * 3600.0 * (double)SPEED_TIMEBASE_HZ / 1000000.0 / (double)SPEED_STOP_TICKS);
}

/* --- S11: complete stop then cold start = ordinary cold arm, NOT a fast rearm ------------------ */
static void test_s11_cold_start_after_stop(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	step_t stopped = s;
	stopped.pedaling_signal_present = false;
	g_wheel_rolling = false;
	hold(&stopped, HOLD_TO_STOP);
	uint32_t remain = SPEED_STOP_TICKS - (g_tick - g_last_pulse) + 1U;
	hold(&stopped, remain);
	CHECK(session_state() == RIDE_SESSION_COLD, "S11: the stop ended in COLD");
	CHECK(!wheel_valid_now(), "S11: wheel invalid at the true stop");

	/* cold start: torque load + forward steps build fwd_run past the cold gate. */
	step_t start = stopped;
	start.pedaling_signal_present = true;
	start.torque_load_centikg = RIDING_START_LOAD_CENTIKG_DEFAULT + 20U;
	uint8_t need = tuning_config_start_steps();
	bool reached = false;
	for (uint8_t i = 0; i < (uint8_t)(need + 2U); i++) {
		fwd1(&start);
		if (session_state() == RIDE_SESSION_ACTIVE) reached = true;
	}
	CHECK(reached, "S11: cold start reached ACTIVE via the ordinary cold arm");
	CHECK(session_state() == RIDE_SESSION_ACTIVE, "S11: ACTIVE");
	CHECK(is_latched(), "S11: latched");
	CHECK(!arm_snapshot_fast_rearm(), "S11: a cold start is NOT a fast rearm");
	CHECK(live_target() != 0, "S11: assist flows after the cold start");
}

/* --- S12: the coast never generates assist - final Iq 0 despite full torque pressure ------------ */
static void test_s12_zero_iq_in_coast(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	step_t coast = s;
	coast.pedaling_signal_present = false;
	coast.torque_load_centikg = RIDING_START_LOAD_CENTIKG_DEFAULT + 20U;
	coast.torque_assist_filtered = 200U;       /* full assist pressure still in the torque chain */
	g_wheel_rolling = true;
	/* advance past the PAS stop delay INTO the coast proper (the pre-real_stop ACTIVE window is
	 * the ordinary stop delay, not a coast - it is not part of this assertion). */
	hold(&coast, HOLD_TO_STOP);
	CHECK(g_real_stop, "S12: real_stop fired");
	CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S12: session retained in the coast");
	CHECK(!is_latched(), "S12: not latched in the coast");
	bool iq_zero = true, floor_zero = true, not_latched = true;
	for (uint32_t i = 0; i < 2000U; i++) {     /* across the whole coast, pressure still held */
		do_tick(&coast, EV_NONE);
		if (live_target() != 0) iq_zero = false;
		if (iq_after_latch_floor() != 0) floor_zero = false;
		if (is_latched()) not_latched = false;
	}
	CHECK(iq_zero, "S12: the FINAL Iq stayed 0 for every coast tick despite full pressure");
	CHECK(floor_zero, "S12: the min-Iq floor never leaked either");
	CHECK(not_latched, "S12: never latched -> the assist-without-rotation hole is structurally closed");
}

/* --- S13: repeated coast/resume cycles never visit COLD and re-arm every time ------------------ */
static void test_s13_repeated_coast_resume(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	g_wheel_rolling = true;
	for (int c = 0; c < 6; c++) {
		step_t coast = s;
		coast.pedaling_signal_present = false;
		hold(&coast, HOLD_TO_STOP);
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S13: coast entered");
		CHECK(!is_latched(), "S13: coast not latched");
		step_t resume = coast;
		resume.pedaling_signal_present = true;
		fwd1(&resume);
		CHECK(session_state() == RIDE_SESSION_ACTIVE, "S13: resume re-armed ACTIVE");
		CHECK(arm_snapshot_fast_rearm(), "S13: fast rearm");
		fwd_cycle(&resume, FWD_PERIOD);
		s = resume;
	}
	CHECK(!g_ever_cold, "S13: repeated coast/resume never visited COLD");
	CHECK(session_state() == RIDE_SESSION_ACTIVE, "S13: ended riding");
}

/* --- S14: repeated rocking with coast (F R coast F INVALID coast F ...) never false-stops ------- */
static void test_s14_rocking_with_coast(void)
{
	reset_all();
	step_t s = establish_riding();
	g_ever_cold = false;
	g_wheel_rolling = true;
	bool no_stop = true, no_assist = true;
	for (int i = 0; i < 10; i++) {
		if (i & 1U) inv1(&s); else rev1(&s);
		if (g_real_stop) no_stop = false;
		if (live_target() != 0) no_assist = false;
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S14: suspended on the reverse/invalid");
		step_t coast = s;
		coast.pedaling_signal_present = false;
		hold(&coast, HOLD_UNDER_TIMEOUT);      /* short coast between the reverse/invalid and the next forward */
		if (g_real_stop) no_stop = false;
		if (live_target() != 0) no_assist = false;
		CHECK(session_state() == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "S14: still suspended through the short coast");
		fwd1(&s); fwd1(&s);                    /* PAS_REVERSE_RECOVERY_CONFIRM_STEPS = 2 */
		if (g_real_stop) no_stop = false;
		CHECK(session_state() == RIDE_SESSION_ACTIVE, "S14: forward phase re-armed ACTIVE");
		if (!is_latched()) no_assist = false;
		if (!arm_snapshot_fast_rearm()) no_assist = false;
		hold(&s, HOLD_UNDER_TIMEOUT);
		if (g_real_stop) no_stop = false;
		if (live_target() == 0) no_assist = false;   /* riding again: assist returns */
	}
	CHECK(no_stop, "S14: the rocking with coast never real-stopped");
	CHECK(!g_ever_cold, "S14: never visited COLD");
	CHECK(no_assist, "S14: no assist during reverse/invalid/coast, real assist on the forward phases");
}

int main(void)
{
	printf("FW-112.2 REAL_STOP vs ROLLING COAST integration, against the shipped modules\n");
	printf("  SPEED_STOP_TICKS=%u ticks = %.3f s @ %u Hz  (existing wheel-freshness window)\n",
		(unsigned)SPEED_STOP_TICKS, (double)SPEED_STOP_TICKS / (double)SPEED_TIMEBASE_HZ,
		(unsigned)SPEED_TIMEBASE_HZ);
	printf("  WHEEL_COAST_PERIOD=%u ticks (default coasting pulse cadence)\n", (unsigned)WHEEL_COAST_PERIOD);
	printf("  adaptive PAS stop timeout basis=%u -> timeout=%u ticks\n",
		(unsigned)FWD_GAP, (unsigned)TIMEOUT_TICKS);

	test_s1_forward_session();
	test_s2_coast_retains();
	test_s3_coast_resume_no_cold_gate();
	test_s4_reverse_coast_forward();
	test_s5_invalid_coast_forward();
	test_s6_true_stop_forward();
	test_s7_true_stop_after_reverse();
	test_s8_long_coast();
	test_s9_wheel_loss_during_coast();
	test_s10_wheel_boundary();
	test_s11_cold_start_after_stop();
	test_s12_zero_iq_in_coast();
	test_s13_repeated_coast_resume();
	test_s14_rocking_with_coast();

	if (host_test_failures != 0) {
		printf("FW-112.2: %d FAILURE(S)\n", host_test_failures);
		return 1;
	}
	if (!g_invariants_ok) {
		printf("FW-112.2: invariant failures reported above\n");
		return 1;
	}
	printf("FW-112.2: ALL S1-S14 PASS\n");
	return 0;
}
