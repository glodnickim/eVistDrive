#ifndef TORQUE_INPUT_H_
#define TORQUE_INPUT_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Single owner of the torque sensor chain: raw ADC millivolts -> automatic
 * zero -> corrected signal -> delta above zero -> public kilogram-force
 * scale in 0.01 kg units. The default conversion is piecewise-linear through
 * measured points, so the firmware is usable without load calibration.
 *
 * FW-150: the default characteristic is the 2026-09-17 reference-weight
 * measurement on this bike (165 mm crank), zero 740 mV:
 *
 *     3.00 kg @  760 mV   (delta   20)
 *     9.50 kg @  925 mV   (delta  185)
 *    20.00 kg @ 1520 mV   (delta  780)
 *
 * It REPLACES the previous two-point curve (6 kg @ 886 mV, 84 kg @ 2320 mV),
 * which the same measurement disproved: that curve predicted 25-53 mV MORE
 * than the sensor actually produces below 10 kg, and 377 mV LESS at 20 kg.
 *
 * Three consequences are deliberate, documented here so they are not "fixed"
 * back by someone reading only one of them:
 *
 *  1. The real sensor is strongly PROGRESSIVE - 6.7 mV/kg over the first
 *     3 kg, 25.4 mV/kg to 9.5 kg, 56.7 mV/kg above that. The old curve
 *     assumed the opposite (sensitivity FALLING with load). TORQUE_ASSIST_-
 *     DEADBAND_NATIVE (10 mV) therefore costs ~1.5 kg of real force on its
 *     own; that is a demand-side tuning decision, not a curve error.
 *  2. Reported load RISES for the same real force (5 kg real used to read
 *     3.8 kg). Assist per unit of pedal force goes UP with this curve.
 *     Compensate through assist level / max_iq_pct - never by re-flattening
 *     this curve, which is a measurement, not a tuning knob.
 *  3. Above the last measured point the curve EXTRAPOLATES on the 9.5->20 kg
 *     slope. TORQUE_PUBLIC_FULL_SCALE_CENTIKG (60.00 kg) then sits at delta
 *     ~3047, above the ADC's physical ceiling (~2560 = 3300 mV - zero), so
 *     60 kg is not reachable on this sensor; usable range ends near 51 kg.
 *     TORQUE_SPAN_MAX_NATIVE is raised accordingly so the DEFAULT span stays
 *     representable. No data exists above 20 kg - the extrapolation is the
 *     weakest part of this curve and the first thing to re-measure if a
 *     reference weight above 20 kg becomes available.
 * FW-129: a user load calibration corrects the sensor GAIN only - the measured delta is
 * referred back to the default sensor and then read on the SAME piecewise characteristic,
 * so calibrating moves where the curve sits without changing its shape. span_native keeps
 * its meaning either way: the native delta this sensor produces at 60.00 kg. The zero point
 * is always automatic and never writable. The assist deadband is a separate
 * relative offset so the kg scale starts at the true zero. Integer math
 * only. torque_input_cal_fault() reports overall signal plausibility:
 * implausible zero (startup/coast) or a stuck-high load that never dips.
 */

#define TORQUE_ZERO_TARGET_NATIVE        740U
/*
 * FW-150: the default characteristic as a point table. Points are (native delta
 * above zero, centikg), strictly increasing in both columns; the origin (0,0) is
 * implicit and is NOT in the table. Between points the curve is linear; above the
 * last point it extrapolates on the last segment's slope (see the file header).
 */
#define TORQUE_CURVE_POINT_COUNT         3U
#define TORQUE_CURVE_P1_NATIVE           20U    /*  3.00 kg @  760 mV */
#define TORQUE_CURVE_P1_CENTIKG          300U
#define TORQUE_CURVE_P2_NATIVE           185U   /*  9.50 kg @  925 mV */
#define TORQUE_CURVE_P2_CENTIKG          950U
#define TORQUE_CURVE_P3_NATIVE           780U   /* 20.00 kg @ 1520 mV */
#define TORQUE_CURVE_P3_CENTIKG          2000U
/*
 * Extrapolated native delta corresponding to 60.00 kg on the default curve:
 * 780 + (6000 - 2000) * (780 - 185) / (2000 - 950) = 3047. Above the ADC ceiling
 * by design - 60 kg is a SCALE reference here, not a reachable reading.
 */
#define TORQUE_DEFAULT_SPAN_NATIVE       3047U
#define TORQUE_SPAN_MIN_NATIVE           800U
/*
 * FW-150: raised from 2600 so the DEFAULT span above is inside the accepted range
 * (a user calibration that matches the factory sensor must be representable).
 */
#define TORQUE_SPAN_MAX_NATIVE           4200U
#define TORQUE_PUBLIC_FULL_SCALE_CENTIKG 6000U
#define TORQUE_INPUT_MAX_CENTIKG         12000U
#define TORQUE_ASSIST_DEADBAND_NATIVE    10U
#define TORQUE_ASSIST_FILTER_MS          35U
#define TORQUE_INPUT_TICKS_PER_MS        4U
#define TORQUE_ASSIST_FILTER_Q_SHIFT     8U
/*
 * FW-033/085: slow "RUN" effort estimator (second filter of the fast signal).
 *
 * FW-085: its window is a CRANK ANGLE, not a time. The quadrature decoder steps
 * every 3.75 deg (96 steps per crank revolution), and the estimator advances one
 * step per forward transition instead of once per control tick — so the window
 * stays the same fraction of a pedal stroke at every cadence. Expressed in
 * milliseconds (the FW-033 form) it could not: 300 ms covered 45 % of a turn at
 * 90 rpm but only 25 % at 50 rpm, which is what made assist pulse once per leg on
 * steep climbs. Being clocked by the crank also removes the need for any clamp,
 * fallback or 32-bit guard: a stopped crank simply stops advancing the filter.
 */
#define TORQUE_RUN_WINDOW_DEG_MAX        360U /* one full crank revolution */
#define TORQUE_RUN_WINDOW_DEG_DEFAULT    180U /* half a turn = one leg */
#define TORQUE_RUN_WINDOW_DEG_STEP       15U  /* exactly 4 quadrature steps */
#define TORQUE_RUN_WINDOW_STEPS_MAX      96U  /* quadrature steps per revolution */

/*
 * FW-090: fast attack. The averaging window exists to kill the DIPS between leg pushes;
 * it has no business delaying a genuine RISE in effort by half a crank turn. Without this,
 * re-catching assist after the power faded mid-ride was a lottery: if the buffer still held
 * decent samples a touch was enough, but after coasting it was full of near-zero samples
 * and the rider had to push through ~180 deg before the magnitude caught up. (The ride
 * latch does not rescue it: torque_input_seed_run() only fires when the latch ARMS, and a
 * brief fade never disarms it.)
 *
 * So a sustained rise re-seeds the window. Both thresholds are deliberately set so ordinary
 * pedalling can NEVER trigger it — that would bring back the per-leg pulsing FW-085 fixed:
 *   - a rectified-sine leg push peaks at ~1.57x its own mean, so the 2x margin sits clear
 *     above anything normal pedalling produces;
 *   - a single leg peak is far shorter than 8 steps (30 deg), so it cannot hold the margin.
 */
#define TORQUE_RUN_ATTACK_NUM            2U   /* sample must exceed average x (NUM/DEN) */
#define TORQUE_RUN_ATTACK_DEN            1U
/*
 * FW-091: 0 = OFF (shipped default). Set to 8 (30 deg of crank) to enable. The complaint
 * this mechanism was written for is now understood to come from the limiter, not from this
 * average — ride FW-091 first and only reach for this if re-engagement is still lazy.
 */
#define TORQUE_RUN_ATTACK_STEPS          0U   /* consecutive steps holding the rise; 0 = off */
/* ...and by at least this much in absolute terms, so sensor jitter around a near-zero
 * average cannot satisfy the ratio test (average 3 vs sample 7 would otherwise pass). */
#define TORQUE_RUN_ATTACK_MIN_DELTA      TORQUE_ASSIST_DEADBAND_NATIVE

/*
 * AP-03: the FW-112.4 asymmetric time-domain filter is REMOVED, and with it
 * TORQUE_RUN_ASYM_RISE_MS / TORQUE_RUN_ASYM_FALL_MS.
 *
 * It published a time-domain rate-limited value INSTEAD of the crank-angle window average
 * above, on every 4 kHz tick and in all three recovery states, so the window it was layered
 * on top of never reached the output at all. The problem it was written for was real - a slow
 * rise from near-zero pays the window lag - but the cure discarded the one mechanism that
 * makes the estimate cadence-independent, and the per-stroke ripple came back with it. A time
 * constant is a fixed fraction of a pedal stroke at exactly one cadence; the stroke period
 * runs from 612 ms at 49 rpm to 250 ms at 120 rpm.
 *
 * Measured on the recorded W1 ride at matched assist level, 90 rpm
 * (integration/evidence/ap03-ab-w1/):
 *
 *   autocorrelation at the stroke period   +0.458 with the filter   -0.126 without
 *   10-90 % on a real load rise            11147 ms with           11253 ms without
 *
 * The stroke rhythm goes away and the response does not measurably suffer. The slow-rise
 * concern that motivated FW-112.4 is NOT closed by that ride - it contains no deliberate step
 * change in effort - and needs the AP-01 step scenario before anyone calls it settled.
 *
 * Ordinary RUN is the crank-angle window again. Assist magnitude drops roughly 20-25 % for the
 * same pedalling, because the window reports the true mean where the filter reported a
 * peak-biased value (fast rise, slow fall). That is compensated by the rider's assist level,
 * never by a hidden gain in this module.
 */

typedef enum {
	TORQUE_CAL_SOURCE_DEFAULT = 0,
	TORQUE_CAL_SOURCE_USER = 1
} torque_cal_source_t;

typedef enum {
	TORQUE_CAL_STATE_IDLE = 0,
	TORQUE_CAL_STATE_CAPTURE_ZERO = 1,
	TORQUE_CAL_STATE_WAIT_REFERENCE = 2,
	TORQUE_CAL_STATE_CAPTURE_LOAD = 3,
	TORQUE_CAL_STATE_PREVIEW = 4,
	TORQUE_CAL_STATE_SUCCESS = 5,
	TORQUE_CAL_STATE_FAILED = 6,
	TORQUE_CAL_STATE_CANCELLED = 7
} torque_cal_state_t;

typedef enum {
	TORQUE_CAL_ERR_NONE = 0,
	TORQUE_CAL_ERR_NOT_STATIONARY = 1,
	TORQUE_CAL_ERR_UNSTABLE = 2,
	TORQUE_CAL_ERR_REFERENCE_RANGE = 3,
	TORQUE_CAL_ERR_DELTA_TOO_SMALL = 4,
	TORQUE_CAL_ERR_SATURATED = 5,
	TORQUE_CAL_ERR_SPAN_RANGE = 6,
	TORQUE_CAL_ERR_SENSOR_FAULT = 7,
	TORQUE_CAL_ERR_TIMEOUT = 8
} torque_cal_error_t;

#define TORQUE_CAL_REFERENCE_MIN_CENTIKG 500U
#define TORQUE_CAL_REFERENCE_MAX_CENTIKG 3000U

typedef struct {
	uint16_t raw_native;
	uint16_t zero_effective_native;
	int16_t corrected_native;
	uint16_t delta_native;
	uint16_t assist_delta_native;
	uint16_t assist_delta_filtered_native;
	uint16_t assist_delta_run_native;   /* FW-033: slow RUN estimator of the fast signal */
	uint16_t load_centikg;
	uint16_t span_native;
	uint8_t calibration_source;
	bool sensor_valid;
} torque_snapshot_t;

void torque_input_init(void);
void torque_input_startup_zero(int32_t rest_raw_native);
int16_t torque_input_correct(uint16_t raw_native);
/* FW-058: bike_moving gates the minimum period between in-ride re-zeros. Pass
 * false when the wheel is stopped — standstill re-zero stays unrestricted. */
void torque_input_coast_update(int16_t torque_corrected_native, bool coast_eligible,
	bool bike_moving);
bool torque_input_cal_fault(void);
/* Legacy one-control-tick wrapper, kept for deterministic module tests and callers that
 * genuinely execute at exactly one 4 kHz tick per call. */
void torque_input_update(uint16_t raw_native, int16_t torque_corrected_native,
	bool sensor_valid);

/* FW-141: production timebase-aware update. elapsed_ticks is the real 4 kHz hardware time
 * since the previous processed control pass (main.c control_delta). This keeps the 35 ms FAST
 * filter and the 120/250 ms ordinary-RUN filter tied to elapsed time when foreground processing
 * misses ticks, rather than silently stretching their response by the missed-call ratio. */
void torque_input_update_elapsed(uint16_t raw_native, int16_t torque_corrected_native,
	bool sensor_valid, uint32_t elapsed_ticks);

const torque_snapshot_t *torque_input_get_snapshot(void);

/* FW-033/085: RUN effort estimator. set_run_window_deg configures the averaging
 * window as a crank angle (0 = disabled, run tracks the fast signal = old
 * behaviour). seed_run pre-loads the filter (e.g. to the fast value at ride
 * start) for a crisp launch. */
void torque_input_set_run_window_deg(uint16_t window_deg);
void torque_input_seed_run(uint16_t value_native);
/*
 * FW-085: advance the RUN estimator by one crank step. Call once per FORWARD
 * quadrature transition (3.75 deg), never from the control-tick loop — that is
 * exactly what makes the window cadence-independent.
 */
void torque_input_run_filter_step(void);
/*
 * FW-112 v2: rolling-rearm recovery for the RUN estimator - a separate three-state automaton
 * (IDLE / WAIT_FRESH_LOAD / TRACK_FAST), driven as a ONE-SHOT EVENT by src/ride_control.c. There
 * is no persistent "armed" state any more (the old torque_input_run_rearm_fast_track() latched
 * across the whole wait and was cancelled by exactly the wrong edges; v2 grants permission on
 * the direction confirm alone, so the automaton opens and closes on the actual rearm).
 *
 *   begin_rolling_rearm()   ride_control calls this exactly once, the tick a fast rearm actually
 *                           happens (session_out.fast_rearm_this_tick - ACTIVE re-entered after a
 *                           direction suspension). It seeds the RUN estimator to the CURRENT fast
 *                           signal - the rearmed Iq starts at full magnitude immediately, with no
 *                           dependence on the stale window - and enters WAIT_FRESH_LOAD.
 *   WAIT_FRESH_LOAD         RUN is re-seeded to the current fast signal on every forward PAS step
 *                           (torque_input_run_filter_step), so a stale pre-reverse window can
 *                           never survive the rearm: the estimator follows the fresh pedal signal
 *                           honestly, in step time, until the pressure is confirmed. The
 *                           automaton leaves it on the tick the fast signal first holds at/above
 *                           the assist deadband (fresh pressure confirmed).
 *   TRACK_FAST              run_value_native = the fast signal EVERY control tick
 *                           (torque_input_update), not just on PAS steps - the recovered RUN
 *                           tracks the fresh pressure in fast-filter time (35 ms) regardless of
 *                           step cadence. Two ways out: the fast signal drops back below the
 *                           deadband (pressure lost -> back to WAIT_FRESH_LOAD, honest collapse),
 *                           or it has held at/above it for TORQUE_ROLLING_REARM_STABLE_TICKS
 *                           (recovery complete - the fresh signal has genuinely taken over) ->
 *                           seed_run() once and return to IDLE, where ordinary FW-085 window
 *                           averaging resumes from a known-good average.
 *   cancel_rolling_rearm()  FORCE-closes the automaton immediately - ride_control calls it
 *                           whenever the session stops being ACTIVE (a fresh reverse/invalid,
 *                           COLD, brake, fault, assist level 0 or a real stop), so a later
 *                           NORMAL cold start can never inherit a recovery window. This is the
 *                           forced exit for terminal session edges ONLY: the automaton still
 *                           closes on its own two natural exits above (TRACK_FAST stable
 *                           completion -> seed_run() once and back to IDLE, or the honest
 *                           collapse back to WAIT_FRESH_LOAD) and is never cancelled by the
 *                           forward step count - the rearm GRANTS permission for
 *                           tuning_config_start_steps() steps, but the estimator recovery is
 *                           allowed to finish on its own.
 *
 * recovery_active() / recovery_run_native() are the READ side ride_control uses to fix the
 * one-tick stale snapshot on the rearm edge: the rider snapshot is built in main.c BEFORE
 * ride_control_update() runs, so on the permission tick it still carries the pre-rearm window
 * average. While the automaton is not IDLE, recovery_run_native() returns the current fast
 * signal, which ride_control substitutes into the mode calculation's torque_run_filtered input.
 */
typedef enum {
	TORQUE_RECOVERY_IDLE = 0,
	TORQUE_RECOVERY_WAIT_FRESH_LOAD = 1,
	TORQUE_RECOVERY_TRACK_FAST = 2
} torque_recovery_state_t;

/*
 * The "stable" window must prove the fresh signal has GENUINELY taken over before the
 * recovery hands RUN back to ordinary FW-085 window averaging. One fast-filter time
 * constant does NOT: at t=tau the 35 ms EMA has only covered 1-e^-1 ~= 63 % of the
 * distance to the new level, so an exit then would re-seed RUN mid-ramp (the
 * long-pollution probe shows the slow window then needing ~26 steps to reach 80 %).
 * ~4 time constants (1-e^-4 ~= 98 %) is a settled plateau - and it keeps the whole
 * recovery inside the hard ≥80 % / ≤150 ms / ≤8-steps bound even after a full
 * no-pressure window (RUN follows the fresh signal in TRACK_FAST the entire time).
 */
#define TORQUE_ROLLING_REARM_STABLE_MS      (4U * TORQUE_ASSIST_FILTER_MS)
#define TORQUE_ROLLING_REARM_STABLE_TICKS   (TORQUE_ROLLING_REARM_STABLE_MS * \
	TORQUE_INPUT_TICKS_PER_MS)
void torque_input_begin_rolling_rearm(void);
void torque_input_cancel_rolling_rearm(void);
bool torque_input_recovery_active(void);
uint16_t torque_input_recovery_run_native(void);
torque_recovery_state_t torque_input_recovery_state(void);
/*
 * FW-112-STABILITY: READ-ONLY diagnostic side of the recovery automaton (measurement only, never
 * feeds a decision):
 *   recovery_stable_ticks()      the LIVE stability streak (the variable the automaton itself
 *                                counts up toward TORQUE_ROLLING_REARM_STABLE_TICKS);
 *   recovery_stable_ticks_at_edge()  the streak AS IT STOOD on the tick the automaton left
 *                                TRACK_FAST - completed (== 560), collapsed back to WAIT_FRESH_LOAD,
 *                                or cancelled/reset. The production counter is cleared on those
 *                                same transition ticks (see torque_input.c), so this is the only
 *                                place the pre-transition value still exists for a recorder to
 *                                read. Never written by production logic; 0 until the first
 *                                transition.
 */
uint16_t torque_input_recovery_stable_ticks(void);
uint16_t torque_input_recovery_stable_ticks_at_edge(void);

uint16_t torque_input_load_centikg(void);
uint16_t torque_input_zero_native(void);
uint16_t torque_input_span_native(void);
uint16_t torque_input_full_scale_native(void);
uint8_t torque_input_calibration_source(void);

bool torque_input_set_user_span(uint16_t span_native);
void torque_input_restore_default_span(void);

bool torque_input_calibration_active(void);
uint8_t torque_input_cal_state(void);
uint8_t torque_input_cal_error(void);
uint16_t torque_input_cal_reference_centikg(void);
uint16_t torque_input_cal_preview_span(void);
void torque_input_cal_start(void);
void torque_input_cal_capture_load(uint16_t reference_centikg);
bool torque_input_cal_commit(void);
void torque_input_cal_cancel(void);
void torque_input_cal_restore_default(void);
bool torque_input_cal_take_persist_request(void);
void torque_input_cal_tick(int16_t torque_corrected_native, bool stationary);

/* Persist span with magic/version/CRC. restore returns true when a valid user
 * calibration was loaded; a bad or absent record keeps the default span. */
bool torque_input_restore_persist(uint16_t magic, uint8_t version,
	uint16_t span_native, uint16_t crc);
void torque_input_build_persist(uint16_t *magic, uint8_t *version,
	uint16_t *span_native, uint16_t *crc);

/* Versioned CAN telemetry blob (0x6025): capabilities + load + calibration
 * status + CRC16. Returns byte length written.
 * FW-061 v2 appends the coast re-zero diagnostics: without them a zero that walks
 * cannot be told apart from a zero that is simply never being corrected. */
#define TORQUE_TELEMETRY_BLOB_LEN 56U
#define TORQUE_TELEMETRY_BLOB_LEN_V1 24U
#define TORQUE_CAP_LOAD_TELEMETRY_V1 0x01U
#define TORQUE_CAP_CALIBRATION_V1    0x02U
#define TORQUE_CAP_COAST_DIAG_V2     0x04U
/*
 * FW-129 D8: this controller applies a user calibration as a GAIN on top of the factory
 * characteristic instead of replacing that characteristic with a straight line. The blob
 * layout and length are unchanged - this bit exists so the tool can say which behaviour it is
 * talking to, and so it knows byte 33 carries TORQUE_CAL_FLAG_* rather than a reserved zero.
 */
#define TORQUE_CAP_GAIN_CALIBRATION_V3 0x08U
/* Byte 33 of the telemetry blob (was reserved). */
#define TORQUE_CAL_FLAG_LEGACY_RECORD_DROPPED 0x01U
uint16_t torque_input_serialize_telemetry(uint8_t *buffer);

/* FW-061: outcome of the most recent coast evaluation. Kept deliberately
 * separate from the counters so a single reading answers "what happened last
 * time" as well as "how often". */
typedef enum {
	TORQUE_COAST_NONE = 0,
	TORQUE_COAST_APPLIED = 1,
	TORQUE_COAST_NO_CHANGE = 2,
	TORQUE_COAST_TOO_SHORT = 3,
	TORQUE_COAST_UNSTABLE = 4,
	TORQUE_COAST_LOCKOUT = 5,
	TORQUE_COAST_OUT_OF_REACQUIRE_RANGE = 6,
	TORQUE_COAST_IMPLAUSIBLE_RAW = 7
} torque_coast_result_t;

uint16_t torque_input_centikg_to_native_delta(uint16_t centikg);
uint16_t torque_input_native_delta_to_centikg(uint16_t delta_native);

/*
 * FW-129 D8 / FW-150: calibration persist versions.
 * v1 = pre-FW-129 (delta_ref * 6000 / reference)
 * v2 = pre-FW-150 (old curve, same units as v3 but incompatible curve)
 * v3 = FW-150 compatible (gain relative to factory characteristic)
 * These are exposed for host tests and tooling.
 */
#define TORQUE_CAL_PERSIST_MAGIC 0x7C41U
#define TORQUE_CAL_PERSIST_VERSION_LEGACY 1U
#define TORQUE_CAL_PERSIST_VERSION_V2_OLD_CURVE 2U
#define TORQUE_CAL_PERSIST_VERSION 3U

/* FW-129 D8 / FW-150: whether the last restore attempt dropped a legacy record.
 * Read by the tool to prompt "recalibrate" instead of silently using a wrong curve. */
bool torque_input_legacy_record_rejected(void);

#endif /* TORQUE_INPUT_H_ */
