/*
 * QZERO: Quiet Zero - controlled PI integral fade after the final Iq reference reaches zero.
 *
 * WHY THIS EXISTS. STOP-CLICK-C1 removed the foreground PI integral resets that used to fire
 * whenever MS.i_q_setpoint==0 (main.c reg_ADC_processing(), up to 4000x/s, racing the 16 kHz PI
 * owner). That removal did what it was meant to do - the click at the final release into
 * ARMED_ZERO is gone - but it also removed the side effect nobody had named: those resets kept
 * collapsing PI_iq's integral, so the applied q-axis voltage stayed near zero and the armed
 * bridge braked the rotor. With persistent ARMED_ZERO (MOE on, FOC/SVPWM/PI live at zero
 * reference) and a continuous integrator, PI_iq now parks at the ONLY equilibrium that produces
 * zero current in a spinning machine: u_q ~= BEMF. The bridge actively matches the back-EMF, no
 * winding current flows, and the rotor free-wheels for far longer than it used to. That is the
 * "long, smooth" motor run-on this module addresses.
 *
 * WHAT IT DOES. On the tick the final Iq reference goes from non-zero to exact 0 for a release
 * the producer marked as QUIET (normal end of pedalling, or a reverse/safety release - never a
 * limiter, never service/calibration), the integral state of both current regulators is faded
 * LINEARLY to exact zero over QZERO_BLEND_TICKS 16 kHz ticks (~10 ms) and then held at zero.
 * PI.out is never touched, the proportional path stays live, and current measurement, FOC, SVPWM
 * and MOE all keep running exactly as they do today. What the hold removes is only the
 * integrator's ability to rebuild the BEMF-compensating voltage while zero torque is commanded.
 *
 * WHAT IT IS NOT. It is not the old foreground reset: there is exactly one writer (the 16 kHz
 * FOC ISR, through this state machine), no MOE toggle, no cold PREPARE, no neutral dwell and no
 * direct CCR write. It does not command negative Iq and it does not request regen - the
 * reference stays exactly 0 and only the P term acts. The current that flows while the rotor
 * spins down is bounded by the abort guard below, not by a commanded torque.
 *
 * LOW-SPEED HANDBACK. The hold ends before the rotor reaches the commutation-angle switch. Below
 * RIDE_COAST_RELEASE_ERPS the angle source changes formula and the angle jumps, so any current
 * still flowing jumps with it - the step historically heard as a clunk at standstill (FW-048).
 * FW-048 keeps that zone current-free by forcing the REFERENCE to zero, which is no longer
 * sufficient on its own here: this module makes current flow from the ABSENCE of the
 * back-EMF-matching voltage, not from a demand. So it hands the axis back to the full
 * zero-current PI below the same threshold. Nothing is given up - energy scales with the square
 * of speed, so the last few chainring rpm hold well under 1 % of the cruise energy.
 *
 * OWNERSHIP. quiet_zero_tick() is called from the 16 kHz FOC ISR only.
 quiet_zero_reset() may be
 * called from the foreground ONLY where the regulators themselves are already being reset and
 * FOC is inactive or being made inactive in the same breath - the same three sites
 * foc_aw_tracking_reset() names (cold PREPARE, dwell-timeout failsafe, hall calibration). It
 * MUST NOT be called on ordinary zero torque or on any periodic tick: that is the STOP-CLICK-C1
 * defect, reintroduced through a different field.
 */

#ifndef QUIET_ZERO_H_
#define QUIET_ZERO_H_

#include <stdbool.h>
#include <stdint.h>

/* State machine. QZERO_INACTIVE means "the regulators are entirely their own owners". */
typedef enum {
	QZERO_INACTIVE = 0,
	QZERO_BLEND    = 1,   /* linear fade of both integrals toward exact zero */
	QZERO_HOLD      = 2,  /* both integrals held at exact zero, P path still live */
	/*
	 * FW-136.2: the mirror of QZERO_BLEND. Entry fades the integral OUT over 10 ms precisely so
	 * the start of braking is inaudible; until FW-137 unfroze the speed reading the exit never
	 * ran at all, so nobody noticed it had no fade of its own and handed the axis back in ONE
	 * tick. That is a real braking torque removed instantaneously - the owner heard it as "you
	 * can hear it disconnect" once FW-136 moved the handback to half the release speed, where
	 * there is still torque to remove. This state fades it back IN instead.
	 */
	QZERO_HANDBACK  = 3
} qzero_state_t;

/*
 * Fade length in 16 kHz FOC ticks. 160 ticks = 10.0 ms at 16 kHz. Long enough that the applied
 * voltage change is a ramp rather than the step the old foreground reset produced (that step is
 * what was audible), short enough that the braking effect starts inside the same release event.
 */
#define QZERO_BLEND_TICKS 160U

/* Compile-time reciprocal so the fade needs a multiply, never a division, in the ISR. */
#define QZERO_BLEND_RECIP (1.0f / (float)QZERO_BLEND_TICKS)

/*
 * FW-136: the handback threshold as a PERCENTAGE of the speed the release started from.
 *
 * The floor (min_brake_erps, RIDE_COAST_RELEASE_ERPS) stays; this only ever raises the point at
 * which the brake lets go. 50 leaves 75 % of the kinetic energy to the brake - energy goes as the
 * square of speed - so the run-down stays short while the entire low-speed zone, where the
 * commutation angle steps, becomes current-free.
 *
 * It is here rather than in config.h on purpose: quiet_zero.c includes only its own header, so
 * the module stays linkable in the host suite without dragging the firmware's configuration in.
 */
#define QZERO_HANDBACK_PCT 50

/*
 * Minimum fresh-speed handback duration: 800 ticks = 50 ms. Its integral slew is bounded by
 * the entry/current integral divided by this duration. A moving target can extend settling;
 * loss of Hall freshness releases the existing integral directly to normal PI, without waiting
 * for this duration or inventing a new BEMF target from a decaying speed estimate.
 */
#define QZERO_HANDBACK_FADE_TICKS 800U
#define QZERO_HANDBACK_RECIP (1.0f / (float)QZERO_HANDBACK_FADE_TICKS)

/* ISR-owned state. One instance, in main.c, beside the regulators it acts on. */
typedef struct {
	uint32_t state;          /* qzero_state_t */
	uint32_t blend_tick;     /* 1..QZERO_BLEND_TICKS while QZERO_BLEND */
	float    iq_integral_entry;  /* PI_iq.integral_part as it stood at the entry edge */
	float    id_integral_entry;  /* PI_id.integral_part as it stood at the entry edge */
	int32_t  erps_entry;         /* QZERO-3: rotor speed at the entry edge, for the handback seed */
	int32_t  prev_iq_ref;    /* previous tick's final Iq reference, for the entry edge */
	float handback_iq, handback_id; /* actual integral at handback, including interrupted BLEND */
	float handback_step_q, handback_step_d;
	/*
	 * QZERO-2: previous tick's QUIET verdict, for the SECOND entry edge. v1 armed only on the
	 * non-zero -> zero edge of the reference, so a release that happened while the current had
	 * ALREADY been taken to zero by something else (riding at the legal speed limit, a battery
	 * or thermal limit, or simply easing off the pedals before the cranks stop) produced no
	 * edge at all and silently fell back to the braked stop. On the bike that was felt as a
	 * stop that behaves differently every time. Owner decision 2026-09-03: the stop must be the
	 * same every time, so the rising edge of the QUIET verdict arms it too. This does NOT arm on
	 * a limiter: the verdict itself is only ever granted once pedalling has ended.
	 */
	uint32_t prev_policy_quiet;

	/* Observability. Free-running counters; they are diagnostics, not limits. */
	uint32_t entries;        /* QZERO_BLEND entered (one per qualifying release) */
	uint32_t aborts;         /* overcurrent guard fired during BLEND/HOLD */
	uint32_t low_speed_exits;/* handed back because the rotor reached the angle-switch zone */
	uint32_t hold_ticks;     /* ticks spent in QZERO_HOLD since the last reset */
} quiet_zero_t;

/*
 * One tick of input. Everything the decision needs, by value - nothing in this module reads a
 * global, so the whole state machine is exercisable on a host.
 *
 * iq_ref            the final Iq reference this tick (MS.i_q_setpoint, the 16 kHz slew output)
 * zero_policy_quiet the producer's verdict on THIS zero: true only for a normal rider release or
 *                   a reverse/safety release. Gates ENTRY only - see quiet_zero_tick().
 * iq_measured       measured Iq (MS.i_q), in PH_CURRENT_MAX counts
 * id_measured       measured Id (MS.i_d), same counts
 * abort_current     |Iq| or |Id| at or above this aborts the hold (QZERO_ABORT_CURRENT)
 * rotor_erps        speed from the latest real valid Hall interval, NOT liveness ui16_erps
 * speed_fresh       that interval is still within its two-period confidence window
 * min_brake_erps    the hold is only allowed at or above this (RIDE_COAST_RELEASE_ERPS)
 * iq_integral       PI_iq.integral_part as it stands right now (captured at the entry edge)
 * id_integral       PI_id.integral_part as it stands right now
 */
typedef struct {
	int32_t iq_ref;
	bool    zero_policy_quiet;
	int32_t iq_measured;
	int32_t id_measured;
	int32_t abort_current;
	int32_t rotor_erps;
	bool speed_fresh;          /* real Hall interval still fresh, never the age-decayed estimate */
	int32_t min_brake_erps;
	float   iq_integral;
	float   id_integral;
} quiet_zero_input_t;

/*
 * What the caller must do with the regulators this tick.
 *
 * apply_integral  write iq_integral/id_integral into PI_iq/PI_id.integral_part. The caller
 *                 applies them BEFORE running the regulators and again AFTER, so the commanded
 *                 trajectory is what an observer sees and PI_control()'s own single-tick
 *                 increment cannot accumulate into a new BEMF-compensating voltage.
 * freeze_aw       FOC-AW1 must not move the integral this tick: zero both aw_sat_error fields
 *                 before the regulators run.
 * clear_aw_edge   entry, exit or abort edge - clear the published residual once, so restarting
 *                 does not act on a residual measured against the held vector.
 * entered/exited/aborted/low_speed_release  edge observations for diagnostics.
 */
typedef struct {
	uint32_t state;
	bool     apply_integral;
	float    iq_integral;
	float    id_integral;
	bool     freeze_aw;
	bool     clear_aw_edge;
	bool     entered;
	bool     exited;
	bool     aborted;
	bool     low_speed_release;
} quiet_zero_action_t;

/* Drop all state. See the ownership note at the top of this file for the only legal sites. */
void quiet_zero_reset(quiet_zero_t *qz);

/*
 * Advance the state machine by one 16 kHz FOC tick and report the action.
 *
 * ENTRY  requires all three: the previous tick's reference was > 0, this tick's is exactly 0,
 *        and the producer marked this zero QUIET. Any other route to zero - a speed, battery or
 *        voltage limiter, a level change, a service/calibration cut, walk assist, a momentary
 *        cap - leaves the regulators exactly as they are today.
 * EXIT   iq_ref > 0 leaves QZERO on the same tick, with no MOE toggle, no cold PREPARE and no
 *        neutral dwell: the very next regulator cycle is normal PI + FOC-AW1 again.
 *        I3 bumpless exit: from BLEND / HOLD / HANDBACK, while the speed is fresh and
 *        rotor_erps > 0 and erps_entry > 0, the exit tick also reports apply_integral with the
 *        integral that nulls the current at the present speed (the HANDBACK target law:
 *        iq = iq_integral_entry * erps_now / erps_entry, speed clamped to erps_entry; id = 0),
 *        so a rider resuming pedalling on a rolling bike does not start from the braking state.
 *        freeze_aw stays false. Otherwise nothing is applied (previous behaviour).
 * ABORT  |Iq| or |Id| >= abort_current during BLEND or HOLD ends the P-only hold and returns the
 *        full zero-current PI (integral free again). It does not re-arm: only a new qualifying
 *        release edge can enter QZERO again.
 * SLOW   rotor_erps < min_brake_erps ends the hold the same way, and also refuses ENTRY - a
 *        release that completes near standstill never arms it. See LOW-SPEED HANDBACK above.
 * HOLD   persists across a mode change that keeps the reference at zero (HOLD, a fall to zero,
 *        FW-048 coast FORCE_ZERO), because the policy gates entry, not continuation.
 */
void quiet_zero_tick(
	quiet_zero_t *qz,
	const quiet_zero_input_t *in,
	quiet_zero_action_t *out);

#endif /* QUIET_ZERO_H_ */
