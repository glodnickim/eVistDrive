/*
 * Test stimulus generator, NOT firmware. Shared by every regression scenario harness so
 * that RUN_60 .. RUN_120 and CADENCE_RAMP_50_120 apply exactly the same pedalling
 * profile and differ ONLY in how fast the crank turns (see
 * documentation/testing/REGRESSION_SCENARIOS.md).
 *
 * One crank-angle model drives both the PAS quadrature step count AND the torque
 * waveform, so a leg's push and the quadrature steps that "measure" it stay in
 * step - the same relationship the real sensor and decoder have on the bike.
 *
 * This is deliberately independent of the firmware's own PAS decoder (which lives
 * inline in src/main.c:reg_ADC_processing and is not a callable unit — see the
 * architecture audit, section 8, "test seam" row for the PAS decoder). The step COUNT
 * this model produces is what a correctly functioning decoder would report; the
 * decoder's own GPIO-level state machine is not exercised. That gap is recorded in
 * documentation/TEST_INFRASTRUCTURE_FOUNDATION_REPORT_PL.md as an observability gap,
 * not hidden.
 */
#ifndef CRANK_MODEL_H
#define CRANK_MODEL_H

#include <stdint.h>

/* Native-mV torque zero, mirrors TORQUE_ZERO_TARGET_NATIVE in inc/torque_input.h. Kept
 * as a local constant (not #included from torque_input.h) so this stimulus generator has
 * zero dependency on the module it feeds — a generator that included its own DUT's
 * header would be able to silently track a future change to that constant instead of
 * exposing it as a real divergence. */
#define CRANK_MODEL_TORQUE_ZERO_NATIVE 740.0

/* One quadrature step every 3.75 deg of crank rotation: 96 steps/rev, matching
 * PAS_STEPS_PER_PULSE=4 (24 pulses/rev) in inc/config.h and TORQUE_RUN_WINDOW_STEPS_MAX=96
 * in inc/torque_input.h. 360/96 = 3.75. */
#define CRANK_MODEL_STEP_DEG (360.0 / 96.0)

/* The control tick rate every module in the ride core assumes, CONTROL_TIMEBASE_HZ in
 * inc/config.h. Duplicated here (not #included) for the same reason as the zero above. */
#define CRANK_MODEL_TICK_HZ 4000.0

typedef struct {
	double mean_native_delta;    /* target mean pedal load above zero, native mV units */
	double ripple_pct;           /* pulsation amplitude as a % of mean_native_delta */
	double asymmetry_pct;        /* left/right leg imbalance, % (0 = symmetric legs) */
	double dead_spot_depth_pct;  /* extra suppression at each leg boundary, 0..100 */
	double dead_spot_width_deg;  /* angular half-width of that suppression, degrees */
	double phase_shift_deg;      /* rotates the whole waveform */
} crank_torque_shape_t;

typedef struct {
	double crank_angle_deg;  /* 0..360, wrapped — for the torque shape lookup and trace */
	double cumulative_deg;   /* monotonic — the only thing step counting reads */
	double cadence_rpm;      /* cadence commanded for the tick just taken */
	uint32_t step_count;     /* cumulative forward quadrature steps since crank_state_init */
} crank_state_t;

void crank_state_init(crank_state_t *state);

/*
 * Advances the model by exactly one 4 kHz control tick at the given instantaneous
 * cadence (constant cadence: pass the same value every tick; a ramp: pass a value that
 * changes tick by tick). Returns how many NEW forward quadrature steps occurred during
 * this tick — 0 most ticks, occasionally 1 at realistic cadences; the caller must not
 * assume it is always <= 1 (a very high or discontinuous cadence could cross more than
 * one 3.75 deg boundary in a single 0.25 ms tick).
 */
uint32_t crank_state_advance_tick(crank_state_t *state, double cadence_rpm);

/* torque_raw_mv the sensor would report at the crank's CURRENT angle under this shape,
 * clamped to a plausible 0..4095 12-bit ADC-scaled mV range. */
uint16_t crank_torque_raw_mv(const crank_state_t *state, const crank_torque_shape_t *shape);

/* Illustrative 0..3 quadrature phase index for the trace's pas_state column. This is
 * step_count % 4, NOT a literal two-GPIO A/B reading — see the file header note above. */
uint8_t crank_pas_state(const crank_state_t *state);

/* Cadence for a linear ramp scenario: rpm_start at t=0, rpm_end at t=duration_s,
 * linear in between, held at rpm_end after. */
double crank_cadence_ramp(double rpm_start, double rpm_end, double duration_s, double t_s);

#endif /* CRANK_MODEL_H */
