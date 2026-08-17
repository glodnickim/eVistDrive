/*
 * FW-112 v2 INTEGRATION HOST HARNESS: the real RUN estimator's lifecycle around the real
 * rearm session chain - PERMISSION vs DEMAND.
 *
 * Same shipped modules as tests/host/ride_control_rearm_host.c (real torque_input.c,
 * ride_control.c, ride_session.c, pas_direction.c, assist_modes.c, ...), driving the REAL
 * RUN estimator:
 *
 *   - torque_input_update(raw, corrected, valid) every control tick (the 35 ms fast EMA),
 *   - torque_input_run_filter_step() on every forward crank step, exactly like main.c:2144,
 *   - the rider snapshot built from torque_input_get_snapshot() (real assist_delta_filtered
 *     -> torque_assist_filtered, real assist_delta_run -> torque_run_filtered, real
 *     assist_delta_native -> torque_assist_now_native, real load_centikg).
 *
 * Control ticks and crank steps are decoupled exactly as in main.c: the control loop runs
 * every tick and a forward PAS transition lands every STEP_INTERVAL_TICKS ticks (60 rpm ->
 * 96 quadrature steps/rev -> 4000/96 = 42 ticks between steps).
 *
 * WHAT FW-112 v2 CHANGED (v1 was "a latched WAIT-wide fast-track that re-seeded the RUN
 * estimator on every forward step for the WHOLE WAIT"). v2 removes the WAIT stage and the
 * two-phase commit entirely: permission (SUSPENDED_BY_DIRECTION -> ACTIVE) is a PURE
 * DIRECTION fact, granted on the direction-confirm edge with NO torque condition, and demand
 * is evaluated fresh every tick by assist_modes_calculate() (see src/ride_session.h). The
 * recovery aid is now a ONE-SHOT rolling-rearm window (torque_input_begin_rolling_rearm(), a
 * single event at the rearm edge) that re-seeds RUN to the fresh fast signal while it lasts
 * and self-terminates; the session never restores a stale pre-reverse Iq.
 *
 * So the contract this harness pins is the v2 one:
 *   R1. reverse/INVALID forces the assist DEMAND and the PRE-RAMP TARGET to 0 the SAME control
 *       tick (hard gate, unchanged); the ACTUAL current reference is NOT asserted 0 same-tick
 *       here - it follows the existing firmware-owned RIDE_HARD_CUT_RAMP_MS (FW-037 deliberate
 *       ~200 ms release that softens the drivetrain clash). The same-tick MS.i_q_setpoint == 0
 *       is a FW-112 requirement ONLY where force_zero_reference's special semantics applies:
 *       zero-demand fast rearm and WAIT_FRESH_LOAD with a final demand == 0;
 *   R2. PERMISSION is immediate on the confirm edge - it can be true (latched) while Iq is 0,
 *       because permission is a direction fact, not a demand guarantee ("no current unless the
 *       calculation itself asks for it" holds on every tick);
 *   R3. an immediate strong-pressure resume after a reverse is FAST: permission returns within
 *       a couple of confirm steps and the rearmed Iq is full-magnitude immediately because the
 *       rearm edge re-seeds RUN to the fresh fast value (bounded by the FAST filter, never the
 *       48-sample window swap the log caught);
 *   R4. a FULL window of no-pressure pollution after a reverse honestly collapses RUN (no
 *       stale-high baseline) and Iq stays 0 throughout it - permission != demand. When strong
 *       pressure finally resumes, the recovery TRACKS the fresh signal in fast-filter time all
 *       the way to its plateau (TRACK_FAST holds for ~4 fast-filter time constants, i.e. until
 *       the fresh signal has GENUINELY taken over), so the estimator is back to ≥80 % within
 *       the hard ≤150 ms / ≤8-steps bound - never a 26-step crawl behind the slow window;
 *   R5. the rolling-rearm window is a TEMPORARY launch aid - once the recovery completes (the
 *       fresh signal has genuinely plateaued, ~4 time constants) and the session has
 *       re-engaged, ordinary FW-085 averaging resumes (4 no-pressure steps then only drop RUN
 *       by a few samples, never collapse it to the fast signal);
 *   R6. light pressure after a reverse never restores the old high RUN/Iq;
 *   R7. a cold start after a real stop is UNCHANGED (RUN seeds to the fresh fast value).
 */

#include <stdint.h>
#include <string.h>

#include "../common/check.h"

#include "assist_extended_boost.h"
#include "assist_modes.h"
#include "config.h"
#include "motor_core.h"
#include "pas_direction.h"
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

/* 60 rpm, 96 quadrature steps/revolution, 4 kHz control loop. */
#define STEP_INTERVAL_TICKS 42U
#define TICKS_PER_REV       (96U * STEP_INTERVAL_TICKS)   /* a full revolution in ticks */

/* Pressure levels as raw mv (offset_correction is 0 after init, so corrected == raw).
 * assist_delta = delta - TORQUE_ASSIST_DEADBAND_NATIVE(10). */
#define STRONG_DELTA_NATIVE 320U   /* assist_delta ~310, load ~15 kg -> the log's fresh level */
#define LIGHT_DELTA_NATIVE  40U    /* assist_delta ~30, above the 5 mV deadband, well under warm */
#define ZERO_DELTA_NATIVE   0U
#define RAW(delta)          ((uint16_t)(TORQUE_ZERO_TARGET_NATIVE + (uint16_t)(delta)))

/* recovery target: 80 % of the steady warm RUN value */
#define RECOVERY_FRACTION   80U

static MotorState_t MS;
static uint32_t g_tick;
static uint8_t g_assist_level = TEST_ASSIST_LEVEL;

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

/* one probe of the observable state right after a control tick */
typedef struct {
	uint32_t tick;
	uint16_t run;
	uint16_t filtered;
	int32_t  iq_request;
	int32_t  iq_after_floor;   /* iq_target right after the latch hold-floor block (see ride_arm_snapshot_t) */
	int32_t  live_target;
	int32_t  actual_iq;      /* MS.i_q_setpoint after the tick - the real motor command */
	bool     latched;
	uint8_t  session_state;
	uint8_t  recovery_state;
} probe_t;

static probe_t g_probe;
static int32_t g_mode_iq_request;

/* service-mode / terminal inputs driven from the test blocks */
static uint8_t g_walk_active;
static uint8_t g_calibration_active;
static uint8_t g_safety_cut;
static uint8_t g_real_stop;

/* S11: battery_voltage_mv override (default TEST_BATTERY_MV) so a scenario can force the mode
 * into the SUPPORTED-but-zero-demand state (the voltage guard in every assist mode
 * short-circuits the demand to 0) without touching any tuning. */
static uint32_t g_battery_mv = TEST_BATTERY_MV;

static uint16_t run_now(void)   { return torque_input_get_snapshot()->assist_delta_run_native; }
static bool is_latched(void)
{
	return (ride_control_get_debug_flags() & RIDE_DBG_NOT_LATCHED) == 0;
}
static int32_t mode_iq_request(void)
{
	return assist_modes_get_last_output()->iq_request;
}
static uint16_t hold_ticks(void)   { return ride_control_get_assist_hold_ticks(); }
static uint8_t  session_state(void) { return ride_control_get_session_state(); }

/* ---- the one control tick, shaped exactly like main.c's (edge handler -> torque_input ->
 * rider snapshot -> ride_control). event: 0 = none, +1 = forward, -1 = reverse,
 * -2 = invalid (illegal) quadrature transition. */
static void control_tick(uint16_t raw_mv, int event)
{
	g_tick++;

	if (event == 1) {
		pas_direction_on_step(1);
		torque_input_run_filter_step();   /* main.c:2144 - the RUN window advances on the step */
	} else if (event == -1) {
		pas_direction_on_step(-1);
	} else if (event == -2) {
		pas_direction_on_step(0);   /* an illegal transition - same direction-inhibit edge as a reverse */
	}

	torque_input_update(raw_mv, torque_input_correct(raw_mv), true);   /* main.c:2329 */

	const torque_snapshot_t *snap = torque_input_get_snapshot();
	bool direction_inhibit_active = pas_direction_direction_inhibit_active();
	bool forward_confirmed_this_tick = (event != 0) && pas_direction_forward_confirmed_last_call();
	uint8_t fwd_run = pas_direction_fwd_run();
	/* The cranks are modelled as physically present; direction is owned entirely by
	 * pas_direction (direction_inhibit_active), exactly like ride_control_rearm_host.c. */
	bool crank_direction_ok = true;
	(void)direction_inhibit_active;
	bool pedaling = crank_direction_ok && (fwd_run >= tuning_config_start_steps());

	rider_input_t r;
	memset(&r, 0, sizeof(r));
	r.torque_raw_mv = raw_mv;
	r.torque_corrected_mv = torque_input_correct(raw_mv);
	r.torque_assist_filtered = snap->assist_delta_filtered_native;
	r.torque_run_filtered = snap->assist_delta_run_native;
	r.torque_load_centikg = torque_input_load_centikg();
	r.torque_assist_now_native = snap->assist_delta_native;
	r.cadence_rpm = crank_direction_ok ? 60U : 0U;
	r.wheel_speed_x100 = TEST_SPEED_X100;
	r.motor_erps = crank_direction_ok ? 200U : 0U;
	r.pas_forward = pedaling;
	r.pas_backward = false;
	r.pedaling_active = pedaling;
	r.crank_forward_steps = fwd_run;
	r.crank_direction_ok = crank_direction_ok;
	r.real_stop = (g_real_stop != 0);
	r.direction_inhibit_active = pas_direction_direction_inhibit_active();
	r.forward_confirmed_this_tick = forward_confirmed_this_tick;
	r.sample_tick = g_tick;
	r.start_phase = false;
	r.torque_sensor_valid = true;
	r.pas_sensor_valid = true;
	rider_input_update(&r);

	ride_control_input_t in;
	memset(&in, 0, sizeof(in));
	in.speed_x100 = TEST_SPEED_X100;
	in.cadence_rpm = r.cadence_rpm;
	in.assist_level_index = g_assist_level;
	in.battery_voltage_mv = g_battery_mv;
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
	in.walk_active = (g_walk_active != 0);
	in.position_calibration_active = (g_calibration_active != 0);
	in.safety_cut_non_direction = (g_safety_cut != 0);
	in.throttle_iq = 0;
	ride_control_update(&in);

	g_mode_iq_request = assist_modes_get_last_output()->iq_request;
	g_probe.tick = g_tick;
	g_probe.run = snap->assist_delta_run_native;
	g_probe.filtered = snap->assist_delta_filtered_native;
	g_probe.iq_request = g_mode_iq_request;
	g_probe.live_target = 0;
	{
		ride_arm_snapshot_t as;
		ride_control_get_arm_snapshot(&as);
		g_probe.iq_after_floor = as.iq_after_latch_floor;
		g_probe.live_target = as.iq_pre_ramp;
	}
	g_probe.actual_iq = MS.i_q_setpoint;   /* the REAL motor command this tick wrote */
	g_probe.latched = is_latched();
	g_probe.session_state = (uint8_t)ride_control_get_session_state();
	g_probe.recovery_state = (uint8_t)torque_input_recovery_state();
}

static void idle(uint32_t n) { for (uint32_t i = 0; i < n; i++) control_tick(RAW(ZERO_DELTA_NATIVE), 0); }

/* Ride n forward crank steps at the given raw pressure, one every STEP_INTERVAL_TICKS ticks.
 * Returns a per-step probe list (allocated by the caller). */
static uint32_t ride_forward(uint16_t raw_mv, uint32_t n, probe_t *out, uint32_t out_cap)
{
	uint32_t wrote = 0;
	for (uint32_t i = 0; i < n; i++) {
		/* control ticks up to the next step: each tick feeds the EMA, then the step itself */
		for (uint32_t t = 0; t + 1 < STEP_INTERVAL_TICKS; t++) {
			control_tick(raw_mv, 0);
		}
		control_tick(raw_mv, 1);
		if (out != 0 && wrote < out_cap) {
			out[wrote] = g_probe;
		}
		wrote++;
	}
	return wrote;
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
	g_tick = 0;
	memset(&g_probe, 0, sizeof(g_probe));
	g_walk_active = 0;
	g_calibration_active = 0;
	g_safety_cut = 0;
	g_real_stop = 0;
	g_battery_mv = TEST_BATTERY_MV;
	set_tuning_start_steps(TUNING_START_STEPS_DEFAULT);
}

/* Warm the whole chain: strong pressure, enough steps to fill the 48-sample window and arm the
 * latch, then let the motor current settle. */
static void warmup(void)
{
	probe_t scratch[64];
	ride_forward(RAW(STRONG_DELTA_NATIVE), 60U, scratch, 64U);
	idle(1000U);
}

/* Drive the hold grace to 0 while the session STAYS ACTIVE. The RUN window only advances on a
 * forward crank step, so the drain first collapses the estimate with a zero-pressure step window
 * (the honest FW-085 collapse: the mode result falls to 0, the hold stops being refreshed and
 * simply counts down tick by tick). The session has no terminal edge, so it must remain ACTIVE
 * throughout - that is exactly the "expiry of the hold never forces COLD" contract. */
static void drain_hold_to_zero(void)
{
	probe_t scratch[64];
	ride_forward(RAW(ZERO_DELTA_NATIVE), 60U, scratch, 64U);   /* collapse RUN, demand -> 0 */
	uint32_t guard = 12000U;   /* > hold_ticks_full (1400 ms = 5600 ticks) + margin */
	for (uint32_t i = 0; i < guard && hold_ticks() > 0; i++) {
		control_tick(RAW(ZERO_DELTA_NATIVE), 0);
	}
	CHECK(hold_ticks() == 0, "hold setup: the grace drained to 0");
	CHECK(is_latched(), "hold setup: the session stayed ACTIVE while the grace drained");
	CHECK(session_state() == RIDE_SESSION_ACTIVE, "hold setup: state ACTIVE while the grace drained");
}

/* Enable the Extended Boost on assist level 3 by patching a freshly serialized bank blob:
 * a 1.0 kg trigger (minimum), default strength and a 1.0 s duration. Bank 0, level 3 record.
 * Layout: header 13 B, record 48 B, 5 records; trigger wire unit at record+36 (0.5 kg/unit),
 * strength at +37, duration_ms (u16) at +46; CRC-16/CCITT over everything before the CRC. */
static void enable_extended_boost(void)
{
	uint8_t buffer[ASSIST_BANK_BLOB_LEN];
	uint16_t len = assist_modes_serialize_bank(0, buffer);
	CHECK(len == ASSIST_BANK_BLOB_LEN, "boost setup: serialized bank 0");
	uint8_t *rec = &buffer[13U + 2U * 48U];
	rec[36] = (uint8_t)(ASSIST_EXT_BOOST_TRIGGER_MIN_CENTIKG /
		ASSIST_EXT_BOOST_TRIGGER_WIRE_STEP_CENTIKG);
	rec[37] = ASSIST_EXT_BOOST_STRENGTH_DEFAULT_PCT;
	rec[46] = (uint8_t)(1000U & 0xFFU);
	rec[47] = (uint8_t)(1000U >> 8);
	uint16_t crc_at = 13U + 5U * 48U;
	uint16_t crc = crc16_ccitt(buffer, crc_at);
	buffer[crc_at] = (uint8_t)(crc & 0xFFU);
	buffer[crc_at + 1U] = (uint8_t)(crc >> 8);
	CHECK(assist_modes_apply_bank_blob(buffer, ASSIST_BANK_BLOB_LEN),
		"boost setup: bank blob with the boost enabled applied");
}

/* Drive the recovery automaton into the requested state (fresh reverse first). */
static void enter_recovery(uint8_t state)
{
	control_tick(RAW(ZERO_DELTA_NATIVE), -1);
	if (state == TORQUE_RECOVERY_WAIT_FRESH_LOAD) {
		/* confirm edge opens WAIT_FRESH_LOAD; zero pressure keeps it there */
		ride_forward(RAW(ZERO_DELTA_NATIVE), 4U, 0, 0);
	} else {
		/* strong pressure: confirm edge -> WAIT_FRESH_LOAD -> TRACK_FAST */
		ride_forward(RAW(STRONG_DELTA_NATIVE), 5U, 0, 0);
	}
}

typedef enum {
	TERM_REVERSE,
	TERM_INVALID,
	TERM_WALK,
	TERM_CALIBRATION,
	TERM_SAFETY_CUT,
	TERM_LEVEL_ZERO,
	TERM_STOP
} terminal_kind_t;

/* Apply exactly ONE terminal tick, then restore the service-mode inputs so the next tick is a
 * normal ride tick again. */
static void run_terminal_tick(terminal_kind_t term)
{
	uint8_t saved_level;
	switch (term) {
	case TERM_REVERSE:
		control_tick(RAW(ZERO_DELTA_NATIVE), -1);
		break;
	case TERM_INVALID:
		control_tick(RAW(ZERO_DELTA_NATIVE), -2);
		break;
	case TERM_WALK:
		g_walk_active = 1;
		control_tick(RAW(ZERO_DELTA_NATIVE), 0);
		g_walk_active = 0;
		break;
	case TERM_CALIBRATION:
		g_calibration_active = 1;
		control_tick(RAW(ZERO_DELTA_NATIVE), 0);
		g_calibration_active = 0;
		break;
	case TERM_SAFETY_CUT:
		g_safety_cut = 1;
		control_tick(RAW(STRONG_DELTA_NATIVE), 0);
		g_safety_cut = 0;
		break;
	case TERM_LEVEL_ZERO:
		saved_level = g_assist_level;
		g_assist_level = 0;
		control_tick(RAW(STRONG_DELTA_NATIVE), 1);
		g_assist_level = saved_level;
		break;
	case TERM_STOP:
		g_real_stop = 1;
		control_tick(RAW(STRONG_DELTA_NATIVE), 0);
		g_real_stop = 0;
		break;
	}
}

/* Item 4 matrix: every terminal inhibit must close the recovery automaton to IDLE in the SAME
 * tick, from both WAIT_FRESH_LOAD and TRACK_FAST. */
static void check_terminal_cancel(uint8_t enter_state, terminal_kind_t term, const char *name)
{
	char label[160];
	const char *state_name = (enter_state == TORQUE_RECOVERY_WAIT_FRESH_LOAD) ?
		"WAIT_FRESH_LOAD" : "TRACK_FAST";
	reset_all();
	warmup();
	enter_recovery(enter_state);
	snprintf(label, sizeof(label), "%s in %s: recovery entered the target state",
		name, state_name);
	CHECK(torque_input_recovery_state() == enter_state, label);
	CHECK(ride_control_get_session_state() == RIDE_SESSION_ACTIVE,
		"terminal: session is ACTIVE while the recovery is open");
	run_terminal_tick(term);
	snprintf(label, sizeof(label), "%s in %s: recovery closed to IDLE in the SAME tick",
		name, state_name);
	CHECK(torque_input_recovery_state() == TORQUE_RECOVERY_IDLE, label);
	CHECK(ride_control_get_session_state() != RIDE_SESSION_ACTIVE,
		"terminal: the session is no longer ACTIVE after the terminal tick");
}

/* Audit S14 matrix: rearm + recovery after a reverse count. Covers the minimal 2R case AND the
 * > 255R case (rev_run is a uint8_t saturating at 255) - the point is that the forward confirm,
 * the fast rearm and the recovery must NOT depend on the saturating rev_run value. The existing
 * "500R still inhibited" test (pas_direction_host D4) stops before the rearm; here the ride is
 * confirmed forward again and the recovery must work normally. */
static void check_rearm_after_reverses(uint32_t rev_count, const char *name)
{
	char label[160];
	reset_all();
	warmup();
	CHECK(is_latched(), "rearm-after-R: setup - warmup armed the latch");
	uint16_t warm_target = run_now();
	CHECK(warm_target > 0, "rearm-after-R: setup - warm RUN estimate positive");

	/* the whole reverse must hold the session SUSPENDED_BY_DIRECTION with a 0 assist DEMAND /
	 * PRE-RAMP TARGET (the audit contract). Note: MS.i_q_setpoint is NOT asserted 0 here - a
	 * reverse forces the assist demand/pre-ramp TARGET to 0 in the same control tick, while the
	 * ACTUAL current reference follows the existing firmware-owned RIDE_HARD_CUT_RAMP_MS (FW-037
	 * deliberate ~200 ms release that softens the drivetrain clash), so the real motor command
	 * fades out instead of snapping. The FW-112 same-tick MS.i_q_setpoint == 0 is asserted on
	 * the REARM tick below (force_zero_reference), which is where that special semantics
	 * applies. */
	for (uint32_t i = 0; i < rev_count; i++) {
		control_tick(RAW(ZERO_DELTA_NATIVE), -1);
		snprintf(label, sizeof(label), "%s (%uR): session SUSPENDED_BY_DIRECTION throughout the reverse", name, (unsigned)rev_count);
		CHECK(g_probe.session_state == RIDE_SESSION_SUSPENDED_BY_DIRECTION, label);
		snprintf(label, sizeof(label), "%s (%uR): assist demand/pre-ramp TARGET 0 throughout the reverse", name, (unsigned)rev_count);
		CHECK(g_probe.live_target == 0, label);
	}

	/* rev_run is the uint8_t legacy counter: saturates at 255 - the forward rearm below must not
	 * care whether it holds 2, 255 or 255-after-512 */
	if (rev_count > 255U) {
		CHECK(pas_direction_rev_run() == 255U,
			"rearm-after-R: rev_run saturated at its uint8_t max (255) for > 255R");
	} else {
		snprintf(label, sizeof(label), "%s (%uR): rev_run counts the reverse steps", name, (unsigned)rev_count);
		CHECK(pas_direction_rev_run() == (uint8_t)rev_count, label);
	}

	/* forward confirm at zero pressure: ACTIVE (fast rearm) must return */
	probe_t conf[64];
	uint32_t conf_steps = ride_forward(RAW(ZERO_DELTA_NATIVE), 48U, conf, 64U);
	uint32_t rearm_step = 0xFFFFFFFFU;
	for (uint32_t i = 0; i < conf_steps; i++) {
		if (conf[i].latched && conf[i].session_state == RIDE_SESSION_ACTIVE) {
			rearm_step = i;
			break;
		}
	}
	snprintf(label, sizeof(label), "%s (%uR): ACTIVE returned after the forward confirm steps (fast rearm)", name, (unsigned)rev_count);
	CHECK(rearm_step != 0xFFFFFFFFU, label);
	if (rearm_step == 0xFFFFFFFFU) { rearm_step = 0U; }
	snprintf(label, sizeof(label), "%s (%uR): on the fast-rearm tick the recovery opened WAIT_FRESH_LOAD", name, (unsigned)rev_count);
	CHECK(conf[rearm_step].recovery_state == TORQUE_RECOVERY_WAIT_FRESH_LOAD, label);
	snprintf(label, sizeof(label), "%s (%uR): on the fast-rearm tick demand and setpoint are 0 same-tick", name, (unsigned)rev_count);
	CHECK(conf[rearm_step].live_target == 0 && conf[rearm_step].actual_iq == 0, label);
	CHECK(hold_ticks() == 0, "rearm-after-R: no armed grace on the zero-demand rearm");

	/* recovery works normally after the rearm: strong pressure refills RUN and closes IDLE */
	probe_t rec[96];
	uint32_t rec_steps = ride_forward(RAW(STRONG_DELTA_NATIVE), 40U, rec, 96U);
	uint32_t target_abs = (uint32_t)warm_target * RECOVERY_FRACTION / 100U;
	bool recovered = false;
	for (uint32_t i = 0; i < rec_steps; i++) {
		if (rec[i].run >= (uint16_t)target_abs) { recovered = true; break; }
	}
	snprintf(label, sizeof(label), "%s (%uR): RUN recovered to >= 80 %% of warm after the rearm", name, (unsigned)rev_count);
	CHECK(recovered, label);
	bool closed_idle = false;
	for (uint32_t i = 0; i < rec_steps; i++) {
		if (rec[i].recovery_state == TORQUE_RECOVERY_IDLE) { closed_idle = true; break; }
	}
	snprintf(label, sizeof(label), "%s (%uR): the recovery closed to IDLE (works normally after the big reverse)", name, (unsigned)rev_count);
	CHECK(closed_idle, label);
	printf("  %s: %uR -> rearm at confirm step %u, RUN recovered, recovery -> IDLE (rev_run=%u)\n",
		name, (unsigned)rev_count, (unsigned)rearm_step, (unsigned)pas_direction_rev_run());
}

/* Audit S15 matrix: the rearm + recovery behaviour must hold for EVERY supported assist mode.
 * Set the active level's mode by patching a freshly serialized bank blob (the production EEPROM
 * path - the mode byte of the level record, correct CRC), WITHOUT touching any other parameter.
 * Then: warm ACTIVE with positive demand -> reverse blocks -> forward rearm at zero pressure
 * gives 0 -> WAIT may last -> late strong pressure recovers with the hard criterion -> IDLE.
 * Supported: POWER_LINEAR=1, POWER_PROGRESSIVE=2, EMTB=3, TORQUE=5, POWER_CURVE=6. */
static void set_bank_mode(uint8_t mode)
{
	uint8_t buffer[ASSIST_BANK_BLOB_LEN];
	uint16_t len = assist_modes_serialize_bank(0, buffer);
	CHECK(len == ASSIST_BANK_BLOB_LEN, "mode setup: serialized bank 0");
	uint8_t *rec = &buffer[13U + (uint8_t)(g_assist_level - 1U) * 48U];
	rec[0] = mode;
	uint16_t crc_at = 13U + 5U * 48U;
	uint16_t crc = crc16_ccitt(buffer, crc_at);
	buffer[crc_at] = (uint8_t)(crc & 0xFFU);
	buffer[crc_at + 1U] = (uint8_t)(crc >> 8);
	CHECK(assist_modes_apply_bank_blob(buffer, ASSIST_BANK_BLOB_LEN),
		"mode setup: bank blob with the mode applied");
}

static void check_mode_rearm(uint8_t mode, const char *name)
{
	char label[160];
	reset_all();
	set_bank_mode(mode);
	warmup();
	CHECK(is_latched(), "mode: setup - warmup armed the latch");
	CHECK(mode_iq_request() > 0, "mode: premise - positive demand in this mode under strong pressure");
	uint16_t warm_target = run_now();
	CHECK(warm_target > 0, "mode: setup - warm RUN estimate positive");

	/* reverse blocks */
	control_tick(RAW(ZERO_DELTA_NATIVE), -1);
	CHECK(!g_probe.latched, "mode: reverse blocked the ride");
	CHECK(g_probe.session_state == RIDE_SESSION_SUSPENDED_BY_DIRECTION, "mode: session SUSPENDED_BY_DIRECTION");
	CHECK(g_probe.live_target == 0, "mode: assist demand/pre-ramp TARGET 0 on the reverse");

	/* forward rearm at zero pressure -> 0 (same-tick), WAIT opens */
	probe_t conf[64];
	uint32_t conf_steps = ride_forward(RAW(ZERO_DELTA_NATIVE), 48U, conf, 64U);
	uint32_t rearm_step = 0xFFFFFFFFU;
	for (uint32_t i = 0; i < conf_steps; i++) {
		if (conf[i].latched && conf[i].session_state == RIDE_SESSION_ACTIVE) { rearm_step = i; break; }
	}
	snprintf(label, sizeof(label), "%s (mode %u): ACTIVE returned after the forward confirm (fast rearm)", name, (unsigned)mode);
	CHECK(rearm_step != 0xFFFFFFFFU, label);
	if (rearm_step == 0xFFFFFFFFU) { rearm_step = 0U; }
	snprintf(label, sizeof(label), "%s (mode %u): the rearm opened WAIT_FRESH_LOAD", name, (unsigned)mode);
	CHECK(conf[rearm_step].recovery_state == TORQUE_RECOVERY_WAIT_FRESH_LOAD, label);
	snprintf(label, sizeof(label), "%s (mode %u): on the rearm tick demand and setpoint are 0 same-tick", name, (unsigned)mode);
	CHECK(conf[rearm_step].live_target == 0 && conf[rearm_step].actual_iq == 0, label);
	CHECK(hold_ticks() == 0, "mode: no armed grace on the zero-demand rearm");

	/* WAIT may last (no timeout): a short no-pressure window keeps WAIT + ACTIVE + 0 */
	bool wait_ok = true;
	for (uint32_t i = 0; i < 2000U; i++) {
		control_tick(RAW(ZERO_DELTA_NATIVE), 0);
		if (g_probe.recovery_state != TORQUE_RECOVERY_WAIT_FRESH_LOAD ||
		    g_probe.session_state != RIDE_SESSION_ACTIVE ||
		    mode_iq_request() != 0 ||
		    g_probe.live_target != 0 ||
		    g_probe.actual_iq != 0) {
			wait_ok = false;
			break;
		}
	}
	snprintf(label, sizeof(label), "%s (mode %u): WAIT may last - no timeout, demand 0, target 0, setpoint 0", name, (unsigned)mode);
	CHECK(wait_ok, label);

	/* late strong pressure: recover >= 80 % within <= 150 ms / <= 8 steps, restore demand, IDLE */
	probe_t rec[96];
	uint32_t rec_steps = ride_forward(RAW(STRONG_DELTA_NATIVE), 80U, rec, 96U);
	uint32_t target_abs = (uint32_t)warm_target * RECOVERY_FRACTION / 100U;
	uint32_t recovery_steps = 0xFFFFFFFFU;
	for (uint32_t i = 0; i < rec_steps; i++) {
		if (rec[i].run >= (uint16_t)target_abs) { recovery_steps = i + 1U; break; }
	}
	snprintf(label, sizeof(label), "%s (mode %u): the late strong pressure DID refill RUN >= 80 %%", name, (unsigned)mode);
	CHECK(recovery_steps != 0xFFFFFFFFU, label);
	uint32_t recovery_ticks = (recovery_steps == 0xFFFFFFFFU) ? 0xFFFFFFFFU :
		(recovery_steps * STEP_INTERVAL_TICKS);
	snprintf(label, sizeof(label), "%s (mode %u) HARD CRITERION: >= 80 %% within <= 8 forward steps", name, (unsigned)mode);
	CHECK(recovery_steps <= 8U, label);
	snprintf(label, sizeof(label), "%s (mode %u) HARD CRITERION: >= 80 %% within <= 150 ms", name, (unsigned)mode);
	CHECK(recovery_ticks <= 600U, label);
	snprintf(label, sizeof(label), "%s (mode %u): the late strong pressure restored a positive demand", name, (unsigned)mode);
	CHECK(mode_iq_request() > 0, label);
	bool closed_idle = false;
	for (uint32_t i = 0; i < rec_steps; i++) {
		if (rec[i].recovery_state == TORQUE_RECOVERY_IDLE && rec[i].run >= (uint16_t)target_abs) {
			closed_idle = true;
			break;
		}
	}
	snprintf(label, sizeof(label), "%s (mode %u): the recovery closed TRACK_FAST -> IDLE", name, (unsigned)mode);
	CHECK(closed_idle, label);
	printf("  %s: mode %u rearm -> WAIT zero (0/0/0), late resume at step %u (%u ms), IDLE back\n",
		name, (unsigned)mode, (unsigned)recovery_steps, (unsigned)(recovery_ticks / 4U));
}

/* Audit S15 sanity: EMTB_CUSTOM = 4 is explicitly unsupported. The wire guard bank_mode_valid()
 * must REJECT it before it can ever reach a config - the mode stays unsupported/no-assist by
 * construction, and assist_modes_calculate() also has no case for it. */
static void check_mode_rejected(uint8_t mode, const char *name)
{
	uint8_t buffer[ASSIST_BANK_BLOB_LEN];
	uint16_t len = assist_modes_serialize_bank(0, buffer);
	CHECK(len == ASSIST_BANK_BLOB_LEN, "unsupported setup: serialized bank 0");
	uint8_t *rec = &buffer[13U + (uint8_t)(g_assist_level - 1U) * 48U];
	rec[0] = mode;
	uint16_t crc_at = 13U + 5U * 48U;
	uint16_t crc = crc16_ccitt(buffer, crc_at);
	buffer[crc_at] = (uint8_t)(crc & 0xFFU);
	buffer[crc_at + 1U] = (uint8_t)(crc >> 8);
	CHECK(!assist_modes_apply_bank_blob(buffer, ASSIST_BANK_BLOB_LEN),
		"unsupported: mode 4 rejected by bank_mode_valid (never stored)");
	printf("  %s: mode %u rejected by the wire guard bank_mode_valid (stays unsupported/no-assist)\n",
		name, (unsigned)mode);
}

int main(void)
{
	printf("FW-112 v2 RUN estimator lifecycle around the real rearm chain (permission vs demand)\n");
	printf("  window 180 deg = %u steps   cadence 60 rpm (step every %u ticks)   fast filter %u ms\n",
		(unsigned)(180U * 4U / 15U), (unsigned)STEP_INTERVAL_TICKS, (unsigned)TORQUE_ASSIST_FILTER_MS);

	/* ==========================================================================
	 * S1 (R1+R3): reverse/INVALID forces the assist DEMAND and the PRE-RAMP TARGET
	 * to 0 the same control tick (the actual current reference follows the existing
	 * firmware-owned RIDE_HARD_CUT_RAMP_MS, FW-037, NOT asserted 0 same-tick); an
	 * IMMEDIATE strong-pressure resume re-arms within a couple of confirm steps and
	 * the rearmed Iq is full-magnitude fast (re-seeded to the fresh fast value at the
	 * rearm edge, not refilled one sample per step).
	 * ======================================================================== */
	{
		reset_all();
		warmup();
		CHECK(is_latched(), "S1: setup - warmup armed the latch");
		uint16_t target = run_now();
		CHECK(target > 0, "S1: setup - warm RUN estimate positive");
		printf("  S1 warm RUN target = %u\n", (unsigned)target);

		/* reverse forces the demand/pre-ramp TARGET to 0 the SAME tick (the real current
		 * reference follows the existing hard-cut ramp), session suspends */
		control_tick(RAW(ZERO_DELTA_NATIVE), -1);
		CHECK(g_probe.live_target == 0, "S1 R1: reverse forces the assist demand/pre-ramp TARGET to 0 the same tick");
		CHECK(!g_probe.latched, "S1 R1: not latched after the reverse");

		/* a short settle so the reverse is a clean edge, then an IMMEDIATE strong resume */
		idle(10U);
		probe_t rec[96];
		uint32_t rec_steps = ride_forward(RAW(STRONG_DELTA_NATIVE), 20U, rec, 96U);
		CHECK(rec_steps == 20U, "S1: drove the strong resume");

		/* the first latched (permitted) step after the resume, and its run/filtered */
		uint32_t perm_step = 0xFFFFFFFFU;
		uint16_t run_at_perm = 0, filtered_at_perm = 0;
		for (uint32_t i = 0; i < rec_steps; i++) {
			if (rec[i].latched) { perm_step = i; run_at_perm = rec[i].run; filtered_at_perm = rec[i].filtered; break; }
		}
		CHECK(perm_step != 0xFFFFFFFFU, "S1 R3: permission returned after the immediate resume");
		bool have_perm = (perm_step != 0xFFFFFFFFU);
		CHECK(perm_step <= 4U, "S1 R3: permission returned within a couple of confirm steps");

		/* recovery: forward steps from the resume until run >= 80 % of the warm target */
		uint32_t target_abs = (uint32_t)target * RECOVERY_FRACTION / 100U;
		uint32_t recovery_steps = 0xFFFFFFFFU;
		for (uint32_t i = 0; i < rec_steps; i++) {
			if (rec[i].run >= (uint16_t)target_abs) { recovery_steps = i + 1U; break; }
		}
		uint32_t recovery_ticks = (recovery_steps == 0xFFFFFFFFU) ? 0xFFFFFFFFU :
			(recovery_steps * STEP_INTERVAL_TICKS);

		printf("  S1: permission at step %u (run=%u filtered=%u iq=%d)   recovery at step %u (%u ms, %.1f rev)\n",
			(unsigned)(have_perm ? perm_step : 0xFFFFFFFFU),
			(unsigned)run_at_perm, (unsigned)filtered_at_perm,
			(int)(have_perm ? rec[perm_step].iq_request : 0),
			(unsigned)recovery_steps, (unsigned)(recovery_ticks / 4U),
			(double)recovery_steps / 48.0);

		/* R3: recovery bounded by the FAST filter - it must NEVER wait for the 48-sample
		 * window to swap (the log's ~600 ms weak period). The rearm edge re-seeds RUN to the
		 * fresh fast value, so full magnitude arrives in fast-filter time. */
		CHECK(recovery_steps <= 8U, "S1 R3: immediate resume recovered within the fast-filter bound (<= 8 forward steps)");

		/* R3: at the permission tick the RUN estimate already reflects the CURRENT fresh
		 * pressure (re-seeded), not a stale/refilling average - run within a couple of native
		 * units of filtered (the re-seed samples the fast value of the PREVIOUS tick, so run
		 * lags the just-advanced filtered by one EMA step, a few units - nowhere near the old
		 * code's run=2 vs filtered=81 refilling gap). */
		CHECK(have_perm && filtered_at_perm > 0U && run_at_perm + 16U >= filtered_at_perm,
			"S1 R3: RUN estimate at permission within a couple of native units of the fresh fast value (re-seeded, not stale/refilling)");

		/* R1: a further reverse DURING the recovery forces the demand/TARGET to 0 the same tick */
		control_tick(RAW(STRONG_DELTA_NATIVE), -1);
		CHECK(g_probe.live_target == 0, "S1 R1: another reverse during recovery forces the assist demand/pre-ramp TARGET to 0 the same tick");
		CHECK(!g_probe.latched, "S1 R1: not latched after the recovery-phase reverse");
	}

	/* ==========================================================================
	 * S2 (R2+R4): PERMISSION != DEMAND. A no-pressure forward phase after a
	 * reverse is PERMITTED (latched) on the confirm edge, but Iq stays 0 the whole
	 * time and the RUN estimate honestly collapses to ~0 (no stale-high baseline).
	 * ======================================================================== */
	{
		reset_all();
		warmup();
		CHECK(is_latched(), "S2: setup - warmup armed the latch");
		uint16_t target = run_now();
		CHECK(target > 0, "S2: setup - warm RUN estimate positive");

		control_tick(RAW(ZERO_DELTA_NATIVE), -1);
		CHECK(g_probe.live_target == 0, "S2 R2: reverse forces the assist demand/pre-ramp TARGET to 0");

		/* a full window of no-pressure forward strokes: permission may return (confirm edge)
		 * but from the rearm tick INCLUSIVE demand must stay 0 - the rearm re-seeds RUN to the
		 * decayed signal AND the same-tick zero forces the motor command to 0 on the exact
		 * rearm tick (no pre-seed stale sample may flow through the dynamics ramp) - and
		 * RUN must honestly collapse */
		probe_t poll[96];
		uint32_t poll_steps = ride_forward(RAW(ZERO_DELTA_NATIVE), 48U, poll, 96U);
		CHECK(poll_steps == 48U, "S2: drove 48 pollution steps");
		uint16_t run_during_wait = run_now();
		CHECK(run_during_wait < target / 2U,
			"S2 R4: the no-pressure wait honestly collapsed the RUN estimate (no stale-high baseline)");
		uint32_t rearm_tick = 0xFFFFFFFFU;
		for (uint32_t i = 0; i < poll_steps; i++) {
			if (poll[i].latched && rearm_tick == 0xFFFFFFFFU) { rearm_tick = i; }
		}
		CHECK(rearm_tick != 0xFFFFFFFFU, "S2: permission returned (confirm edge fired)");
		if (rearm_tick == 0xFFFFFFFFU) { rearm_tick = 0U; } /* guard: report the FAIL above cleanly instead of indexing the sentinel */
		/* SAME-TICK ZERO: on the EXACT rearm tick (i == rearm_tick) the mode demand, the
		 * pre-ramp target and the real motor setpoint must all be 0 - the pre-reverse
		 * reference must not fade through it. */
		CHECK(poll[rearm_tick].iq_request == 0,
			"S2 SAME-TICK: mode demand is 0 on the exact rearm tick");
		CHECK(poll[rearm_tick].live_target == 0,
			"S2 SAME-TICK: pre-ramp target is 0 on the exact rearm tick");
		CHECK(poll[rearm_tick].actual_iq == 0,
			"S2 SAME-TICK: MS.i_q_setpoint is 0 on the exact rearm tick");
		bool zero_after_rearm = true;
		for (uint32_t i = rearm_tick; i < poll_steps; i++) {
			if (poll[i].live_target != 0 || poll[i].actual_iq != 0) {
				zero_after_rearm = false;
				printf("   S2 -> nonzero at step %u (live=%d actual=%d run=%u)\n",
					(unsigned)i, (int)poll[i].live_target, (int)poll[i].actual_iq,
					(unsigned)poll[i].run);
				break;
			}
		}
		CHECK(zero_after_rearm,
			"S2 R2: once permission returned, Iq stayed 0 from the rearm tick onward - permission != demand");
		printf("  S2: run during the wait = %u (of warm %u)   rearm tick=%u (mode=%d live=%d actual=%d)\n",
			(unsigned)run_during_wait, (unsigned)target, (unsigned)rearm_tick,
			(int)poll[rearm_tick].iq_request, (int)poll[rearm_tick].live_target,
			(int)poll[rearm_tick].actual_iq);

		/* strong fresh pressure resumes, keep riding until it is fully back. The recovery
		 * TRACKS the fresh signal in fast-filter time through TRACK_FAST (which now holds for
		 * ~4 fast-filter time constants - a genuine plateau), so the estimator reaches the hard
		 * ≥80 % target inside the ≤150 ms / ≤8-steps bound - not a 26-step crawl. */
		probe_t rec[96];
		uint32_t rec_steps = ride_forward(RAW(STRONG_DELTA_NATIVE), 80U, rec, 96U);
		uint32_t target_abs = (uint32_t)target * RECOVERY_FRACTION / 100U;
		uint32_t recovery_steps = 0xFFFFFFFFU;
		for (uint32_t i = 0; i < rec_steps; i++) {
			if (rec[i].run >= (uint16_t)target_abs) { recovery_steps = i + 1U; break; }
		}
		bool eventually_full = false;
		for (uint32_t i = 0; i < rec_steps; i++) {
			if (rec[i].run >= (uint16_t)target_abs && rec[i].iq_request > 0) {
				eventually_full = true; break;
			}
		}
		CHECK(recovery_steps != 0xFFFFFFFFU && eventually_full,
			"S2 R4: after the long pollution, strong pressure DOES refill RUN back to full and Iq returns");
		/* HARD CRITERION: ≥80 % of the warm target within ≤150 ms / ≤8 forward steps after
		 * the strong resume (8 steps at 42 ticks = 336 ticks = 84 ms - inside 150 ms). */
		uint32_t recovery_ticks = (recovery_steps == 0xFFFFFFFFU) ? 0xFFFFFFFFU :
			(recovery_steps * STEP_INTERVAL_TICKS);
		CHECK(recovery_steps <= 8U,
			"S2 R4 HARD CRITERION: after a full no-pressure window, RUN back to ≥80 % within ≤8 forward steps");
		CHECK(recovery_ticks <= 600U /* 150 ms */,
			"S2 R4 HARD CRITERION: RUN back to ≥80 % within ≤150 ms after the strong resume");

		/* FSM pins on the resume: the automaton must have re-entered WAIT_FRESH_LOAD at the
		 * rearm edge (probe 0), advanced to TRACK_FAST while the fresh signal was still rising
		 * below the plateau, and finally closed back to IDLE after the fresh signal genuinely
		 * took over - it must never sit in IDLE while the estimator is still below target. */
		CHECK(rec[0].recovery_state == TORQUE_RECOVERY_WAIT_FRESH_LOAD ||
			rec[0].recovery_state == TORQUE_RECOVERY_TRACK_FAST,
			"S2 R4 FSM: recovery automaton open (WAIT_FRESH_LOAD/TRACK_FAST) on the first resume step");
		bool saw_track = false, closed_to_idle = false;
		for (uint32_t i = 0; i < rec_steps; i++) {
			if (rec[i].recovery_state == TORQUE_RECOVERY_TRACK_FAST) { saw_track = true; }
			if (saw_track && rec[i].recovery_state == TORQUE_RECOVERY_IDLE) {
				closed_to_idle = true;
			}
			if (closed_to_idle && rec[i].run >= (uint16_t)target_abs) {
				printf("   S2 FSM -> closed to IDLE at step %u with run=%u (target %u)\n",
					(unsigned)i, (unsigned)rec[i].run, (unsigned)target_abs);
				break;
			}
		}
		CHECK(saw_track, "S2 R4 FSM: the recovery advanced through TRACK_FAST during the resume");
		CHECK(closed_to_idle,
			"S2 R4 FSM: the recovery closed back to IDLE once the fresh signal took over");

		printf("  S2: recovery at step %u (%u ms, %.1f rev)   fsm first step=%u\n",
			(unsigned)recovery_steps, (unsigned)((recovery_steps == 0xFFFFFFFFU) ? 0xFFFFFFFFU : recovery_steps * STEP_INTERVAL_TICKS / 4U),
			(double)((recovery_steps == 0xFFFFFFFFU) ? 0 : recovery_steps) / 48.0,
			(unsigned)rec[0].recovery_state);
	}

	/* ==========================================================================
	 * S3 (R5): the rolling-rearm window is a TEMPORARY launch aid - once the
	 * recovery completes and the session has re-engaged, ordinary FW-085 averaging
	 * resumes. Probe: 4 zero-pressure steps right after an immediate recovery.
	 * With normal averaging the RUN estimate only drops by a few samples (one step
	 * in / one step out); with a still-live re-seed it would collapse to the fast
	 * signal.
	 * ======================================================================== */
	{
		reset_all();
		warmup();
		uint16_t target = run_now();
		CHECK(target > 0, "S3: setup - warm RUN estimate positive");

		control_tick(RAW(ZERO_DELTA_NATIVE), -1);
		idle(10U);
		probe_t rec[96];
		uint32_t rec_steps = ride_forward(RAW(STRONG_DELTA_NATIVE), 20U, rec, 96U);
		uint32_t target_abs = (uint32_t)target * RECOVERY_FRACTION / 100U;
		uint32_t recovery_steps = 0xFFFFFFFFU;
		for (uint32_t i = 0; i < rec_steps; i++) {
			if (rec[i].run >= (uint16_t)target_abs) { recovery_steps = i + 1U; break; }
		}
		CHECK(recovery_steps <= 8U, "S3: immediate recovery was fast (<= 8 forward steps)");

		/* 20 strong resume steps passed above - enough for the fresh window to refill, so the
		 * rolling-rearm window has closed (stability or the ride_control start_steps cancel).
		 * Now probe the estimator with 4 zero-pressure steps. */
		ride_forward(RAW(ZERO_DELTA_NATIVE), 4U, 0, 0);
		uint16_t run_after_soft = run_now();
		printf("  S3: run after 4 zero-pressure steps = %u (of warm %u)   recovery at step %u\n",
			(unsigned)run_after_soft, (unsigned)target, (unsigned)recovery_steps);
		CHECK(run_after_soft >= target / 2U,
			"S3 R5: RUN averaged (not re-seeded) once the recovery completed - rolling rearm self-terminated");
	}

	/* ==========================================================================
	 * S4 (R6): light pressure after the reverse must NOT restore the old high
	 * RUN/Iq. Collapse RUN first with a no-pressure pollution phase, then resume at
	 * LIGHT pressure: permission is immediate, but the demand (and the estimator)
	 * stays at the light level, never snaps back to the warm pre-reverse value.
	 * ======================================================================== */
	{
		reset_all();
		warmup();
		uint16_t warm = run_now();
		CHECK(warm >= (uint16_t)200U, "S4: setup - warm RUN is high");

		control_tick(RAW(ZERO_DELTA_NATIVE), -1);
		ride_forward(RAW(ZERO_DELTA_NATIVE), 48U, 0, 0);   /* pollute the whole window first */

		probe_t rec[64];
		uint32_t rec_steps = ride_forward(RAW(LIGHT_DELTA_NATIVE), 60U, rec, 64U);
		uint32_t max_run = 0;
		for (uint32_t i = 0; i < rec_steps; i++) {
			if (rec[i].run > (uint16_t)max_run) max_run = rec[i].run;
		}
		printf("  S4: warm RUN = %u, light-pressure recovery max RUN = %u\n", (unsigned)warm, (unsigned)max_run);
		CHECK(max_run < warm / 2U,
			"S4 R6: light pressure never restores the old high RUN estimate (no stale baseline)");
	}

	/* ==========================================================================
	 * S5 (R2 + SAME-TICK ZERO): no pressure at all after the reverse. Permission
	 * may return on the confirm edge, and on that EXACT rearm tick the mode demand,
	 * the pre-ramp target AND the real motor setpoint (MS.i_q_setpoint) must all be
	 * 0 - the rearm re-seeds RUN to the decayed ~0 signal and the same-tick zero
	 * forces the dynamics ramp to 0, so no pre-reverse reference may fade through
	 * the rearm tick. From then on Iq must stay 0 for good. The fast EMA decays
	 * below the deadband almost immediately after the reverse.
	 * ======================================================================== */
	{
		reset_all();
		warmup();
		control_tick(RAW(ZERO_DELTA_NATIVE), -1);
		probe_t rec[64];
		uint32_t rec_steps = ride_forward(RAW(ZERO_DELTA_NATIVE), 60U, rec, 64U);
		CHECK(rec_steps == 60U, "S5: drove 60 no-pressure steps");
		uint32_t rearm_tick = 0xFFFFFFFFU;
		for (uint32_t i = 0; i < rec_steps; i++) {
			if (rec[i].latched && rearm_tick == 0xFFFFFFFFU) { rearm_tick = i; }
		}
		CHECK(rearm_tick != 0xFFFFFFFFU, "S5: permission returned (confirm edge fired)");
		if (rearm_tick == 0xFFFFFFFFU) { rearm_tick = 0U; } /* guard: report the FAIL above cleanly instead of indexing the sentinel */
		/* SAME-TICK ZERO: on the EXACT rearm tick (i == rearm_tick) the mode demand, the
		 * pre-ramp target and the REAL motor setpoint (MS.i_q_setpoint) must all be 0 - no
		 * pre-seed stale reference may flow through the dynamics ramp on the tick permission
		 * is restored. */
		CHECK(rec[rearm_tick].iq_request == 0,
			"S5 SAME-TICK: mode demand is 0 on the exact rearm tick");
		CHECK(rec[rearm_tick].live_target == 0,
			"S5 SAME-TICK: pre-ramp target is 0 on the exact rearm tick");
		CHECK(rec[rearm_tick].actual_iq == 0,
			"S5 SAME-TICK: MS.i_q_setpoint is 0 on the exact rearm tick");
		bool zero_after_rearm = true;
		for (uint32_t i = rearm_tick; i < rec_steps; i++) {
			if (rec[i].live_target != 0 || rec[i].actual_iq != 0) {
				zero_after_rearm = false;
				printf("   S5 -> nonzero at step %u (live=%d actual=%d run=%u filtered=%u)\n",
					(unsigned)i, (int)rec[i].live_target, (int)rec[i].actual_iq,
					(unsigned)rec[i].run, (unsigned)rec[i].filtered);
				break;
			}
		}
		CHECK(zero_after_rearm,
			"S5 R2: no pressure after the reverse - Iq stayed 0 from the rearm tick INCLUSIVE (same-tick zero + re-seed)");
		printf("  S5: permission at step %u, rearm tick all-zero (mode=%d live=%d actual=%d), zero for %u steps\n",
			(unsigned)rearm_tick, (int)rec[rearm_tick].iq_request, (int)rec[rearm_tick].live_target,
			(int)rec[rearm_tick].actual_iq, (unsigned)(rec_steps - rearm_tick));
	}

	/* ==========================================================================
	 * S7 (item 2): a LONG (>= 2.0 s) no-pressure forward window after a reverse.
	 * Through the whole window the session stays ACTIVE, the recovery automaton
	 * stays in WAIT_FRESH_LOAD (NO timeout - the FSM has none), the demand, the
	 * pre-ramp target and the real motor setpoint stay 0 (no floor leak: the hold
	 * grace is never armed by zero mode demand), and RUN honestly stays ~0. Only a
	 * LATE strong press finally resumes - and must then hit >= 80 % within
	 * <= 150 ms / <= 8 forward steps and return the recovery to IDLE.
	 * ======================================================================== */
	{
		reset_all();
		warmup();
		CHECK(is_latched(), "S7: setup - warmup armed the latch");
		uint16_t target = run_now();
		CHECK(target > 0, "S7: setup - warm RUN estimate positive");

		control_tick(RAW(ZERO_DELTA_NATIVE), -1);
		CHECK(!g_probe.latched, "S7: reverse suspended the session");

		/* drive just past the confirm edge so the rearm (and its WAIT_FRESH_LOAD) opens */
		probe_t open_poll[8];
		uint32_t open_steps = ride_forward(RAW(ZERO_DELTA_NATIVE), 6U, open_poll, 8U);
		CHECK(open_steps == 6U, "S7: drove the confirm-edge steps");
		uint32_t rearm_tick = 0xFFFFFFFFU;
		for (uint32_t i = 0; i < open_steps; i++) {
			if (open_poll[i].latched) { rearm_tick = i; break; }
		}
		CHECK(rearm_tick != 0xFFFFFFFFU, "S7: permission returned (fast rearm opened)");
		if (rearm_tick == 0xFFFFFFFFU) { rearm_tick = 0U; } /* guard: report the FAIL above cleanly instead of indexing the sentinel */
		CHECK(open_poll[rearm_tick].recovery_state == TORQUE_RECOVERY_WAIT_FRESH_LOAD,
			"S7: the rearm edge opened WAIT_FRESH_LOAD");
		CHECK(open_poll[rearm_tick].live_target == 0 && open_poll[rearm_tick].actual_iq == 0,
			"S7 SAME-TICK: on the rearm edge with no pressure the demand and the motor setpoint are 0");

		/* ride forward with NO pressure for >= 2.0 s (8000 ticks). The recovery must stay
		 * WAIT_FRESH_LOAD (no timeout), the session ACTIVE, and demand + motor current 0. */
		uint32_t steps_2s = (8000U / STEP_INTERVAL_TICKS) + 2U;
		probe_t wait_poll[200];
		uint32_t wait_steps = ride_forward(RAW(ZERO_DELTA_NATIVE), steps_2s, wait_poll, 200U);
		CHECK(wait_steps == steps_2s, "S7: drove the >= 2.0 s no-pressure window");
		bool wait_ok = true;
		for (uint32_t i = 0; i < wait_steps; i++) {
			if (!wait_poll[i].latched ||
			    wait_poll[i].session_state != RIDE_SESSION_ACTIVE ||
			    wait_poll[i].recovery_state != TORQUE_RECOVERY_WAIT_FRESH_LOAD ||
			    wait_poll[i].live_target != 0 ||
			    wait_poll[i].actual_iq != 0) {
				printf("   S7 -> violation at wait step %u (latched=%d session=%u rec=%u live=%d actual=%d)\n",
					(unsigned)i, wait_poll[i].latched ? 1 : 0, (unsigned)wait_poll[i].session_state,
					(unsigned)wait_poll[i].recovery_state, (int)wait_poll[i].live_target,
					(int)wait_poll[i].actual_iq);
				wait_ok = false;
				break;
			}
		}
		CHECK(wait_ok,
			"S7: 2s no-pressure WAIT - ACTIVE + WAIT_FRESH_LOAD throughout, demand/motor 0, no timeout, no floor leak");
		CHECK(run_now() < (uint16_t)(target / 4U),
			"S7: RUN honestly collapsed to ~0 through the 2s no-pressure wait");

		/* LATE strong pressure: recovery must track the fresh signal back to >= 80 % fast and
		 * return to IDLE - even after a 2s empty window, the automaton did not time out. */
		probe_t rec[96];
		uint32_t rec_steps = ride_forward(RAW(STRONG_DELTA_NATIVE), 80U, rec, 96U);
		uint32_t target_abs = (uint32_t)target * RECOVERY_FRACTION / 100U;
		uint32_t recovery_steps = 0xFFFFFFFFU;
		for (uint32_t i = 0; i < rec_steps; i++) {
			if (rec[i].run >= (uint16_t)target_abs) { recovery_steps = i + 1U; break; }
		}
		CHECK(recovery_steps != 0xFFFFFFFFU,
			"S7: the late strong pressure DID refill RUN back to full");
		uint32_t recovery_ticks = (recovery_steps == 0xFFFFFFFFU) ? 0xFFFFFFFFU :
			(recovery_steps * STEP_INTERVAL_TICKS);
		CHECK(recovery_steps <= 8U,
			"S7 HARD CRITERION: after a 2s no-pressure WAIT, RUN back to >= 80 % within <= 8 forward steps");
		CHECK(recovery_ticks <= 600U,
			"S7 HARD CRITERION: RUN back to >= 80 % within <= 150 ms after the late strong pressure");
		CHECK(rec[0].recovery_state == TORQUE_RECOVERY_TRACK_FAST ||
			rec[0].recovery_state == TORQUE_RECOVERY_WAIT_FRESH_LOAD,
			"S7 FSM: the recovery advanced toward TRACK_FAST on the first late-pressure step");
		bool closed_idle = false;
		for (uint32_t i = 0; i < rec_steps; i++) {
			if (rec[i].recovery_state == TORQUE_RECOVERY_IDLE && rec[i].run >= (uint16_t)target_abs) {
				closed_idle = true;
				break;
			}
		}
		CHECK(closed_idle, "S7 FSM: the recovery returned to IDLE after the late strong pressure");
		printf("  S7: 2s wait kept WAIT_FRESH_LOAD (run=0), late resume -> recovery at step %u (%u ms), IDLE back\n",
			(unsigned)recovery_steps, (unsigned)(recovery_ticks / 4U));
	}

	/* ==========================================================================
	 * S8 (item 3): TRACK_FAST tracks the fresh signal on EVERY CONTROL TICK, not
	 * only on forward PAS steps. Enter TRACK_FAST, then drive many control ticks
	 * with NO PAS edge while the fast signal is still rising - RUN must equal the
	 * fast signal after every single tick. A mutation that only published RUN on a
	 * PAS step would leave RUN stale between steps and fail this.
	 * ======================================================================== */
	{
		reset_all();
		warmup();
		uint16_t target = run_now();
		CHECK(target > 0, "S8: setup - warm RUN estimate positive");

		control_tick(RAW(ZERO_DELTA_NATIVE), -1);
		idle(10U);
		/* confirm edge + strong pressure enter WAIT_FRESH_LOAD then TRACK_FAST */
		ride_forward(RAW(STRONG_DELTA_NATIVE), 5U, 0, 0);
		CHECK(torque_input_recovery_state() == TORQUE_RECOVERY_TRACK_FAST,
			"S8: the recovery advanced into TRACK_FAST");

		/* many control ticks, NO PAS edges, while the fast signal is still rising */
		uint16_t first_filtered = g_probe.filtered;
		bool tracked_every_tick = true;
		uint32_t tracked_ticks = 80U;
		for (uint32_t i = 0; i < tracked_ticks; i++) {
			control_tick(RAW(STRONG_DELTA_NATIVE), 0);
			if (g_probe.run != g_probe.filtered) {
				printf("   S8 -> tick %u: run=%u filtered=%u (must be equal in TRACK_FAST)\n",
					(unsigned)g_probe.tick, (unsigned)g_probe.run, (unsigned)g_probe.filtered);
				tracked_every_tick = false;
				break;
			}
		}
		CHECK(tracked_every_tick,
			"S8: RUN tracked the fast signal after EVERY control tick (no PAS edge) - per-tick, not per-step");
		CHECK(g_probe.filtered > first_filtered,
			"S8: the fast signal genuinely rose during the window (tracking a MOVING target)");
		printf("  S8: TRACK_FAST tracked fast every tick (no PAS edges), fast rose %u -> %u\n",
			(unsigned)first_filtered, (unsigned)g_probe.filtered);
	}

	/* ==========================================================================
	 * S9 (item 4): every terminal inhibit closes the recovery automaton to IDLE in
	 * the SAME tick, from BOTH WAIT_FRESH_LOAD and TRACK_FAST: reverse, invalid,
	 * Walk Assist entry, position-sensor calibration, a non-direction safety cut,
	 * assist level 0 and a real stop.
	 * ======================================================================== */
	{
		check_terminal_cancel(TORQUE_RECOVERY_WAIT_FRESH_LOAD, TERM_REVERSE, "reverse");
		check_terminal_cancel(TORQUE_RECOVERY_TRACK_FAST,      TERM_REVERSE, "reverse");
		check_terminal_cancel(TORQUE_RECOVERY_WAIT_FRESH_LOAD, TERM_INVALID, "invalid");
		check_terminal_cancel(TORQUE_RECOVERY_TRACK_FAST,      TERM_INVALID, "invalid");
		check_terminal_cancel(TORQUE_RECOVERY_WAIT_FRESH_LOAD, TERM_WALK, "walk entry");
		check_terminal_cancel(TORQUE_RECOVERY_TRACK_FAST,      TERM_WALK, "walk entry");
		check_terminal_cancel(TORQUE_RECOVERY_WAIT_FRESH_LOAD, TERM_CALIBRATION, "position calibration");
		check_terminal_cancel(TORQUE_RECOVERY_TRACK_FAST,      TERM_CALIBRATION, "position calibration");
		check_terminal_cancel(TORQUE_RECOVERY_WAIT_FRESH_LOAD, TERM_SAFETY_CUT, "non-direction safety cut");
		check_terminal_cancel(TORQUE_RECOVERY_TRACK_FAST,      TERM_SAFETY_CUT, "non-direction safety cut");
		check_terminal_cancel(TORQUE_RECOVERY_WAIT_FRESH_LOAD, TERM_LEVEL_ZERO, "assist level 0");
		check_terminal_cancel(TORQUE_RECOVERY_TRACK_FAST,      TERM_LEVEL_ZERO, "assist level 0");
		check_terminal_cancel(TORQUE_RECOVERY_WAIT_FRESH_LOAD, TERM_STOP, "real stop");
		check_terminal_cancel(TORQUE_RECOVERY_TRACK_FAST,      TERM_STOP, "real stop");
		printf("  S9: terminal-inhibit matrix (%d combinations) all closed the recovery to IDLE in the same tick\n", 14);
	}

	/* ==========================================================================
	 * S10 (item 6): Extended Boost in the rearm context. A reverse cuts the boost in
	 * the SAME tick (cancel chain runs before the state machine), so neither a stale
	 * ARM nor an ACTIVE boost can resurrect pre-reverse Iq through the rearm. After
	 * the rearm the boost needs a fresh qualify window + crank-stop edge and delivers
	 * nothing on its own while the cranks keep turning. Normal boost behaviour outside
	 * the rearm stays covered by fw100_extended_boost_host.c - unchanged.
	 * ======================================================================== */
	{
		reset_all();
		enable_extended_boost();
		warmup();
		CHECK(is_latched(), "S10: setup - warmup armed the latch with the boost enabled");

		assist_extended_boost_diag_t bd;
		/* warmup's strong pressure (about 15 kg = 1547 centikg, far above the patched
		 * 1 kg trigger) already qualified and ARMED the boost; warmup's final idle only
		 * aged the arm below the 1500 ms timeout and decayed the fast signal, exactly
		 * like S5. */
		assist_extended_boost_get_diag(&bd);
		printf("  S10: boost after warmup -> state=%u peak=%u centikg\n",
			(unsigned)bd.state, (unsigned)bd.peak_load_centikg);
		CHECK(bd.state == ASSIST_EXT_BOOST_ARMED,
			"S10: the boost confirmed and armed while pedalling with trigger load");

		/* the reverse must CANCEL the boost in the SAME tick (pre-ramp target -> 0; the
		 * dynamics carry the previous current down smoothly, exactly as without a boost) */
		control_tick(RAW(ZERO_DELTA_NATIVE), -1);
		CHECK(g_probe.live_target == 0, "S10: the reverse cuts the pre-ramp target to 0");
		assist_extended_boost_get_diag(&bd);
		CHECK(bd.state == ASSIST_EXT_BOOST_IDLE,
			"S10: the reverse cancelled the boost in the SAME tick");
		CHECK(bd.cancel_reason == ASSIST_EXT_BOOST_CANCEL_REVERSE ||
			bd.cancel_reason == ASSIST_EXT_BOOST_CANCEL_SAFETY_CUT,
			"S10: the boost logged the reverse/safety-cut cancel reason");

		/* confirm forward with NO pressure: a full window of zero-pressure strokes fills the
		 * RUN window with zeros - the estimate honestly collapses, the confirm edge opens
		 * WAIT_FRESH_LOAD and the rearm re-seeds to the decayed signal. The boost stays
		 * IDLE throughout (a stale ARM would have held the pre-reverse last_pedal_iq); on
		 * the exact rearm tick the mode demand, the pre-ramp target and the real motor
		 * setpoint must all be 0. */
		probe_t rec[96];
		uint32_t rec_steps = ride_forward(RAW(ZERO_DELTA_NATIVE), 48U, rec, 96U);
		CHECK(run_now() < 155U,
			"S10: the no-pressure wait collapsed the RUN estimate with the boost enabled");
		uint32_t rearm_tick = 0xFFFFFFFFU;
		for (uint32_t i = 0; i < rec_steps; i++) {
			if (rec[i].latched) { rearm_tick = i; break; }
		}
		CHECK(rearm_tick != 0xFFFFFFFFU, "S10: permission returned after the rearm");
		if (rearm_tick == 0xFFFFFFFFU) { rearm_tick = 0U; } /* guard: report the FAIL above cleanly instead of indexing the sentinel */
		CHECK(rec[rearm_tick].iq_request == 0 && rec[rearm_tick].live_target == 0 &&
			rec[rearm_tick].actual_iq == 0,
			"S10 SAME-TICK: on the rearm tick the motor setpoint is 0 (no stale boost current)");
		bool no_stale_boost = true;
		for (uint32_t i = rearm_tick; i < rec_steps; i++) {
			if (rec[i].actual_iq != 0 || rec[i].live_target != 0) { no_stale_boost = false; break; }
		}
		CHECK(no_stale_boost, "S10: the rearm wait delivered no stale boost current");
		assist_extended_boost_get_diag(&bd);
		CHECK(bd.state == ASSIST_EXT_BOOST_IDLE,
			"S10: the boost stayed IDLE through the whole rearm wait - no stale ARM survived the reverse");

		/* fresh pressure after the rearm: the boost may re-qualify FRESH, but it can never go
		 * ACTIVE while the cranks keep turning - no pre-reverse stale Iq can be delivered */
		ride_forward(RAW(STRONG_DELTA_NATIVE), 20U, 0, 0);
		assist_extended_boost_get_diag(&bd);
		CHECK(bd.state != ASSIST_EXT_BOOST_ACTIVE,
			"S10: after the rearm the boost never went ACTIVE (it needs a fresh crank-stop edge)");
		CHECK(bd.state == ASSIST_EXT_BOOST_ARMED || bd.state == ASSIST_EXT_BOOST_QUALIFY,
			"S10: after the rearm the boost re-qualified FRESH (not a resurrected stale arm)");
		printf("  S10: reverse cancelled the boost same-tick, rearm delivered 0, post-rearm boost state=%u\n",
			(unsigned)bd.state);
	}

	/* ==========================================================================
	 * S11 (HOLD OWNER - negative): with the session ACTIVE, permission in force, NO terminal
	 * edge and the raw torque signal clearly above the run deadband, the hold grace must NOT be
	 * armed when the mode's own result is 0. The mode result is forced to 0 deterministically
	 * and tuning-independently by feeding the chain strong pressure while battery_voltage_mv ==
	 * 0: the mode GATE still passes (pedalling + valid sensors) so the mode is SUPPORTED, but
	 * the voltage guard short-circuits the demand to 0 (assist_modes.c returns true with
	 * iq_request == 0). The correct owner refresh is `supported && mode_output.iq_request > 0`;
	 * a torque-based owner (`supported && filtered >= run_deadband`) arms the hold here and
	 * raises the min-Iq floor through it - the exact regression this scenario pins.
	 * ======================================================================== */
	{
		reset_all();
		warmup();
		CHECK(is_latched(), "S11: setup - warmup armed the latch");
		CHECK(session_state() == RIDE_SESSION_ACTIVE, "S11: setup - session ACTIVE");
		drain_hold_to_zero();
		CHECK(hold_ticks() == 0, "S11: setup - hold drained to 0 while the session stayed ACTIVE");

		/* strong pressure with battery_voltage_mv == 0: the fast EMA crosses the deadband within
		 * a couple of ticks, but every tick keeps the mode supported with a 0 result */
		g_battery_mv = 0U;
		probe_t rec[64];
		uint32_t rec_steps = ride_forward(RAW(STRONG_DELTA_NATIVE), 40U, rec, 64U);
		CHECK(rec_steps == 40U, "S11: drove strong pressure with battery_voltage_mv == 0");
		CHECK(rec[rec_steps - 1U].filtered > tuning_config_run_deadband_mv(),
			"S11: premise - the FAST signal is clearly above the run deadband");
		CHECK((ride_control_get_debug_flags() & RIDE_DBG_MODE_UNSUPPORTED) == 0,
			"S11: premise - the mode is SUPPORTED (the gate passed)");
		CHECK(rec[rec_steps - 1U].iq_request == 0,
			"S11: premise - the mode's own result is 0 (battery voltage guard)");

		/* THE contract: the hold must NOT be armed, so no floor, no target, no setpoint. */
		CHECK(hold_ticks() == 0,
			"S11 HOLD OWNER: the raw torque above the deadband did NOT arm the hold (the mode result owns it)");
		CHECK(rec[rec_steps - 1U].iq_after_floor == 0,
			"S11 HOLD OWNER: no min-Iq floor result from a hold that was never armed");
		CHECK(rec[rec_steps - 1U].live_target == 0,
			"S11 HOLD OWNER: iq target stays 0");
		CHECK(rec[rec_steps - 1U].actual_iq == 0,
			"S11 HOLD OWNER: MS.i_q_setpoint stays 0");
		g_battery_mv = TEST_BATTERY_MV;
		printf("  S11: held at 0 under torque > deadband with a 0 mode result (filtered=%u)\n",
			(unsigned)rec[rec_steps - 1U].filtered);
	}

	/* ==========================================================================
	 * S12 (HOLD OWNER - positive + expiry): a SUPPORTED mode with mode_output.iq_request > 0
	 * MUST arm/renew the hold. Once armed, the grace survives later zero-demand ticks (the
	 * min-Iq floor keeps its min-pull while it lasts), and its EXPIRY must NEVER change the
	 * session ACTIVE -> COLD - permission is owned by the ride_session automaton, not by the
	 * hold counter.
	 * ======================================================================== */
	{
		reset_all();
		warmup();
		CHECK(is_latched(), "S12: setup - warmup armed the latch");
		drain_hold_to_zero();
		CHECK(hold_ticks() == 0, "S12: setup - hold drained to 0 while ACTIVE");

		/* a real positive mode demand re-arms the hold */
		probe_t pos[32];
		ride_forward(RAW(STRONG_DELTA_NATIVE), 20U, pos, 32U);
		CHECK(mode_iq_request() > 0, "S12: premise - the mode demand is positive");
		CHECK(hold_ticks() > 0, "S12 HOLD OWNER: a positive mode demand armed/renewed the hold");
		uint16_t armed_hold = hold_ticks();

		/* demand vanishes through a zero-pressure window that collapses the RUN estimate */
		probe_t col[96];
		uint32_t col_steps = ride_forward(RAW(ZERO_DELTA_NATIVE), 60U, col, 96U);
		CHECK(col_steps == 60U, "S12: drove the zero-pressure collapse");
		CHECK(mode_iq_request() == 0, "S12: premise - the mode demand has vanished");
		CHECK(hold_ticks() < armed_hold, "S12 HOLD OWNER: the grace counted down as demand vanished");
		CHECK(hold_ticks() > 0, "S12 HOLD OWNER: the grace is still active");
		CHECK(is_latched(), "S12 HOLD OWNER: still latched during the grace");
		CHECK(session_state() == RIDE_SESSION_ACTIVE, "S12 HOLD OWNER: session still ACTIVE during the grace");
		/* the armed hold keeps the min-Iq floor's min pull for the zero-demand grace ticks */
		bool grace_min_pull = false;
		for (uint32_t i = 0; i < col_steps; i++) {
			if (col[i].iq_request == 0 && col[i].iq_after_floor > 0) { grace_min_pull = true; break; }
		}
		CHECK(grace_min_pull,
			"S12 HOLD OWNER: during the grace the armed hold kept the min-Iq floor (min pull)");

		/* let the grace run out: the expiry must not force ACTIVE -> COLD, and once the hold is
		 * gone the floor is gone with it (no demand -> no current) */
		uint32_t guard = (uint32_t)hold_ticks() + 2U;
		for (uint32_t i = 0; i < guard; i++) {
			idle(1U);
		}
		CHECK(hold_ticks() == 0, "S12 HOLD OWNER: the hold expired after its grace ran out");
		CHECK(is_latched(), "S12 HOLD OWNER: expiry of the hold did NOT force ACTIVE -> COLD - still latched");
		CHECK(session_state() == RIDE_SESSION_ACTIVE, "S12 HOLD OWNER: session state ACTIVE after the hold expired");
		CHECK(g_probe.live_target == 0, "S12 HOLD OWNER: with the hold expired and no demand, the pre-ramp target is 0");
		/* the real motor command fades out over the level's release (650 ms = 2600 ticks);
		 * give it a margin to finish, then it must be exactly 0 */
		idle(12000U);
		CHECK(g_probe.actual_iq == 0, "S12 HOLD OWNER: with the hold expired and no demand, the setpoint is 0");
		printf("  S12: positive demand armed the hold (%u), grace min-pull, expiry kept ACTIVE\n",
			(unsigned)armed_hold);
	}

	/* ==========================================================================
	 * S13 (AUDIT BLOCKER - TRACK_FAST -> WAIT_FRESH_LOAD HOLD GRACE LEAK): a hold legitimately
	 * armed by a positive TRACK_FAST demand must NOT keep its min-Iq floor through a subsequent
	 * WAIT_FRESH_LOAD with a 0 mode result. The WAIT+zero-demand contract is absolute: on the
	 * exact tick the recovery drops to WAIT_FRESH_LOAD with iq_request == 0, the pedal target,
	 * the final target AND MS.i_q_setpoint must all be 0, the session must STILL be ACTIVE, and
	 * no floor may arise from the earlier armed grace. Then a >= 2 s forward-pedal-no-pressure
	 * window must keep WAIT + ACTIVE + hold 0 + demand 0 + target 0 + setpoint 0 throughout,
	 * and a LATE strong pressure must still recover >= 80 % within <= 150 ms / <= 8 steps and
	 * close TRACK_FAST -> IDLE. Tuning-robust: the test OBSERVES TRACK_FAST entry + a positive
	 * real demand + a positive setpoint first, THEN takes the pressure away - it never assumes
	 * a fixed step count for the arm.
	 * ======================================================================== */
	{
		reset_all();
		warmup();
		CHECK(is_latched(), "S13: setup - warmup armed the latch");
		uint16_t warm_target = run_now();
		CHECK(warm_target > 0, "S13: setup - warm RUN estimate positive");

		control_tick(RAW(ZERO_DELTA_NATIVE), -1);
		CHECK(!g_probe.latched, "S13: setup - reverse suspended the session");

		/* pedal forward with strong pressure, observing every tick: the FIRST tick where the
		 * recovery is TRACK_FAST with a positive real mode demand and a positive setpoint is
		 * where the pressure is taken away (before the 140 ms stable window can complete). */
		bool track_positive = false;
		uint16_t armed_hold = 0;
		uint32_t since_step = 0;
		for (uint32_t i = 0; i < 2000U; i++) {
			if (since_step == STEP_INTERVAL_TICKS - 1U) {
				control_tick(RAW(STRONG_DELTA_NATIVE), 1);   /* forward PAS step under pressure */
				since_step = 0;
			} else {
				control_tick(RAW(STRONG_DELTA_NATIVE), 0);
				since_step++;
			}
			if (g_probe.recovery_state == TORQUE_RECOVERY_TRACK_FAST &&
			    g_probe.iq_request > 0 && g_probe.actual_iq > 0) {
				track_positive = true;
				armed_hold = hold_ticks();
				break;
			}
		}
		CHECK(track_positive, "S13: observed TRACK_FAST with a positive demand and setpoint");
		CHECK(armed_hold > 0, "S13: premise - the positive TRACK_FAST demand armed the hold (leak precondition)");

		/* NOW take the pressure away (well before the stable window), pedalling forward at
		 * cadence with zero pressure. The fast signal decays below the deadband and the
		 * recovery must fall back TRACK_FAST -> WAIT_FRESH_LOAD. */
		probe_t wait;
		bool in_wait = false;
		since_step = 0;
		for (uint32_t i = 0; i < 4000U; i++) {
			if (since_step == STEP_INTERVAL_TICKS - 1U) {
				control_tick(RAW(ZERO_DELTA_NATIVE), 1);   /* forward PAS step at zero pressure */
				since_step = 0;
			} else {
				control_tick(RAW(ZERO_DELTA_NATIVE), 0);
				since_step++;
			}
			if (g_probe.recovery_state == TORQUE_RECOVERY_WAIT_FRESH_LOAD &&
			    mode_iq_request() == 0) {
				in_wait = true;
				wait = g_probe;
				break;
			}
		}
		CHECK(in_wait, "S13: the pressure taken during TRACK_FAST dropped the recovery to WAIT_FRESH_LOAD with a 0 mode result");

		/* THE CONTRACT - on the EXACT WAIT tick, with a 0 mode result: */
		CHECK(wait.recovery_state == TORQUE_RECOVERY_WAIT_FRESH_LOAD, "S13 WAIT: recovery_state == WAIT_FRESH_LOAD");
		CHECK(wait.iq_request == 0, "S13 WAIT: mode_output.iq_request == 0");
		CHECK(hold_ticks() == 0, "S13 WAIT: the earlier armed grace is suppressed - assist_hold_ticks == 0");
		CHECK(wait.iq_after_floor == 0, "S13 WAIT: no min-Iq floor arose from the suppressed grace (iq_after_latch_floor == 0)");
		CHECK(wait.live_target == 0, "S13 WAIT: iq_pre_ramp == 0 (pedal target 0)");
		CHECK(wait.actual_iq == 0, "S13 WAIT: MS.i_q_setpoint == 0 in the SAME tick");
		CHECK(wait.session_state == RIDE_SESSION_ACTIVE, "S13 WAIT: ride_session is STILL ACTIVE");

		/* >= 2.0 s (8000 ticks) of forward pedalling with no pressure: the WHOLE window must
		 * stay WAIT_FRESH_LOAD + ACTIVE + hold 0 + demand 0 + target 0 + setpoint 0 */
		bool wait_clean = true;
		since_step = 0;
		for (uint32_t i = 0; i < 8000U; i++) {
			if (since_step == STEP_INTERVAL_TICKS - 1U) {
				control_tick(RAW(ZERO_DELTA_NATIVE), 1);
				since_step = 0;
			} else {
				control_tick(RAW(ZERO_DELTA_NATIVE), 0);
				since_step++;
			}
			if (g_probe.recovery_state != TORQUE_RECOVERY_WAIT_FRESH_LOAD ||
			    g_probe.session_state != RIDE_SESSION_ACTIVE ||
			    hold_ticks() != 0 ||
			    mode_iq_request() != 0 ||
			    g_probe.live_target != 0 ||
			    g_probe.actual_iq != 0) {
				printf("   S13 -> violation at tick %u (rec=%u session=%u hold=%u demand=%d live=%d setpoint=%d)\n",
					(unsigned)g_probe.tick, (unsigned)g_probe.recovery_state, (unsigned)g_probe.session_state,
					(unsigned)hold_ticks(), (int)mode_iq_request(), (int)g_probe.live_target, (int)g_probe.actual_iq);
				wait_clean = false;
				break;
			}
		}
		CHECK(wait_clean, "S13: 2 s forward with no pressure kept WAIT_FRESH_LOAD + ACTIVE + hold 0 + demand 0 + target 0 + setpoint 0");

		/* LATE strong pressure: recover >= 80 % of warm within <= 150 ms / <= 8 steps, restore a
		 * positive demand and close TRACK_FAST -> IDLE - the WAIT never timed out. */
		probe_t rec[96];
		uint32_t rec_steps = ride_forward(RAW(STRONG_DELTA_NATIVE), 80U, rec, 96U);
		uint32_t target_abs = (uint32_t)warm_target * RECOVERY_FRACTION / 100U;
		uint32_t recovery_steps = 0xFFFFFFFFU;
		for (uint32_t i = 0; i < rec_steps; i++) {
			if (rec[i].run >= (uint16_t)target_abs) { recovery_steps = i + 1U; break; }
		}
		CHECK(recovery_steps != 0xFFFFFFFFU, "S13: the late strong pressure DID refill RUN back to >= 80 %");
		uint32_t recovery_ticks = (recovery_steps == 0xFFFFFFFFU) ? 0xFFFFFFFFU :
			(recovery_steps * STEP_INTERVAL_TICKS);
		CHECK(recovery_steps <= 8U, "S13 HARD CRITERION: RUN back to >= 80 % within <= 8 forward steps");
		CHECK(recovery_ticks <= 600U, "S13 HARD CRITERION: RUN back to >= 80 % within <= 150 ms");
		CHECK(mode_iq_request() > 0, "S13: the late strong pressure restored a positive mode demand");
		CHECK(rec[0].recovery_state == TORQUE_RECOVERY_TRACK_FAST ||
			rec[0].recovery_state == TORQUE_RECOVERY_WAIT_FRESH_LOAD,
			"S13 FSM: the recovery advanced toward TRACK_FAST on the first late-pressure step");
		bool closed_idle = false;
		for (uint32_t i = 0; i < rec_steps; i++) {
			if (rec[i].recovery_state == TORQUE_RECOVERY_IDLE && rec[i].run >= (uint16_t)target_abs) {
				closed_idle = true;
				break;
			}
		}
		CHECK(closed_idle, "S13 FSM: the recovery closed TRACK_FAST -> IDLE after the late strong pressure");
		printf("  S13: TRACK_FAST -> WAIT grace suppressed (hold %u -> 0, floor 0, target 0, setpoint 0 same-tick), 2s clean, late resume at step %u (%u ms), IDLE back\n",
			(unsigned)armed_hold, (unsigned)recovery_steps, (unsigned)(recovery_ticks / 4U));
	}

	/* ==========================================================================
	 * S14 (audit missing matrix): rearm + recovery after a reverse count - the minimal
	 * 2R case AND the > 255R case (rev_run saturates at 255 as a uint8_t). The rearm,
	 * the recovery and RUN refill must NOT depend on the saturating rev_run value, and
	 * the ride must confirm forward again after the big reverse (the existing 500R
	 * "still inhibited" test stops before that point).
	 * ======================================================================== */
	check_rearm_after_reverses(2U, "S14a");
	check_rearm_after_reverses(300U, "S14b");
	check_rearm_after_reverses(512U, "S14c");

	/* ==========================================================================
	 * S15 (audit parametric rearm matrix): the rearm + recovery must behave identically for
	 * every SUPPORTED assist mode (POWER_LINEAR=1, POWER_PROGRESSIVE=2, EMTB=3, TORQUE=5,
	 * POWER_CURVE=6), with the mode set through the real bank blob/API and CRC. EMTB_CUSTOM=4
	 * stays unsupported by construction (bank_mode_valid rejects it before it ever reaches a
	 * config, and assist_modes_calculate() has no case for it).
	 * ======================================================================== */
	check_mode_rearm(ASSIST_MODE_POWER_LINEAR, "S15a");
	check_mode_rearm(ASSIST_MODE_POWER_PROGRESSIVE, "S15b");
	check_mode_rearm(ASSIST_MODE_EMTB, "S15c");
	check_mode_rearm(ASSIST_MODE_TORQUE, "S15d");
	check_mode_rearm(ASSIST_MODE_POWER_CURVE, "S15e");
	check_mode_rejected(ASSIST_MODE_EMTB_CUSTOM, "S15f");

	/* ==========================================================================
	 * S6 (R7): a cold start after a real stop is UNCHANGED - the RUN seed on a cold
	 * arm is the fresh fast value, and normal riding follows.
	 * ======================================================================== */
	{
		reset_all();
		warmup();
		CHECK(is_latched(), "S6: setup - armed");
		/* terminal -> COLD the production way: assist level 0 (owner: level 0 cancels) */
		g_assist_level = 0;
		control_tick(RAW(STRONG_DELTA_NATIVE), 1);
		CHECK(ride_control_get_session_state() == RIDE_SESSION_COLD, "S6: level 0 took the session to COLD");
		g_assist_level = TEST_ASSIST_LEVEL;

		/* a normal cold start from COLD with strong fresh pressure (start_steps default 4) */
		probe_t rec[64];
		uint32_t rec_steps = ride_forward(RAW(STRONG_DELTA_NATIVE), 12U, rec, 64U);
		CHECK(rec_steps == 12U, "S6: drove the cold-start steps");
		bool armed = false;
		for (uint32_t i = 0; i < rec_steps; i++) {
			if (rec[i].latched) { armed = true; break; }
		}
		CHECK(armed, "S6 R7: a normal cold start still arms with strong fresh pressure");
		uint16_t cold_run = run_now();
		printf("  S6: cold-start run = %u  (fast = %u)\n", (unsigned)cold_run,
			(unsigned)torque_input_get_snapshot()->assist_delta_filtered_native);
		/* The cold path itself (arm + seed) is UNCHANGED by FW-112 v2 - ride_control_rearm_host.c
		 * carries the full cold-start regression. Here we only pin that a cold start still works
		 * with the REAL estimator present, and that Iq is genuinely requested through the real
		 * chain. */
		CHECK(cold_run > 0U, "S6 R7: a cold start produces a positive RUN estimate");
		CHECK(mode_iq_request() > 0, "S6 R7: a cold start produces a real Iq request");
	}

	if (host_test_failures == 0) {
		printf("All FW-112 v2 RUN lifecycle checks passed.\n");
		return 0;
	}
	printf("\n%d FW-112 v2 check(s) FAILED.\n", host_test_failures);
	return 1;
}