/*
 * QS-3D-R1: single final Iq slew owner at the 16 kHz FOC rate.
 *
 * The 4 kHz ride/control domain publishes a complete command through a bounded
 * single-producer/single-consumer mailbox. The 16 kHz ISR accepts a command only
 * when both sequence reads identify the same stable generation; otherwise it keeps
 * the last command that was verified. All payload members are explicit aligned
 * 32-bit objects -- there are no C bitfields and no assumed 64-bit atomic transfer.
 *
 * Battery-current limiting remains upstream in ride_control.c (QS-3C contract).
 * Walk Assist, calibration and comm-loss publish explicit BYPASS/FORCE_ZERO
 * commands through the same owner; they never write MS.i_q_setpoint directly.
 */

#ifndef FAST_IQ_SLEW_H_
#define FAST_IQ_SLEW_H_

#include <stdbool.h>
#include <stdint.h>

#define FAST_IQ_SLEW_READ_ATTEMPTS 3U

/* Slew modes published by the 4 kHz domain. */
typedef enum {
	FIS_MODE_RISE        = 0,  /* normal rise toward target */
	FIS_MODE_FALL        = 1,  /* normal fall toward target */
	FIS_MODE_RELEASE     = 2,  /* normal release: live state -> zero in configured time */
	FIS_MODE_SAFETY      = 3,  /* safety release: live state -> zero in configured time */
	FIS_MODE_FORCE_ZERO  = 4,  /* exact same-tick zero (force_zero_reference, coast_release) */
	FIS_MODE_BYPASS      = 5,  /* service/Walk passthrough, still written by fast owner */
	FIS_MODE_HOLD        = 6   /* target equals the authoritative live Q10 state */
} fis_mode_t;

/*
 * QZERO: why THIS zero was commanded. The mode alone cannot answer that question - FIS_MODE_SAFETY
 * covers a reverse, a brake, an overtemperature AND the pedal-load calibration, and a zero that
 * came from the speed or battery limiter arrives as an ordinary FALL to target 0. The policy is
 * therefore published explicitly by the 4 kHz producer alongside the mode, and it is the ONLY
 * thing that may arm Quiet Zero (see inc/quiet_zero.h).
 *
 * QUIET is set for exactly two causes: the normal end of pedalling (the level's release fade) and
 * a reverse/safety release. NONE is everything else, including every limiter, service and
 * calibration path, Walk Assist and force-zero - those keep the ordinary zero-current PI.
 */
typedef enum {
	FIS_ZERO_POLICY_NONE  = 0,
	FIS_ZERO_POLICY_QUIET = 1
} fis_zero_policy_t;

/*
 * One verified behavior command. step_mag_8 is the legacy per-4-kHz Q8 step;
 * adding it to the Q10 accumulator on each of four 16-kHz ticks preserves the
 * original integrated slope exactly. Release commands instead carry their real
 * 16-kHz duration and a producer-computed floor(2^32 / duration) reciprocal. The
 * consumer uses multiply/shift plus a remainder correction at the release edge,
 * so it derives an exact live-state rate without a division instruction in the ISR.
 */
typedef struct {
	int32_t  target;
	uint32_t step_mag_8;
	uint32_t mode;
	uint32_t release_ticks_16k;
	uint32_t release_recip_q32;
	uint32_t zero_policy;
	/*
	 * HARD CEILING ON THE REFERENCE. The accumulator is clamped to this on every tick, in
	 * every mode.
	 *
	 * The target says what the rider is asking for and is approached over the rider-feel
	 * attack/release times. That is the wrong instrument for a protection: when a limiter
	 * starts binding, lowering the target alone leaves the reference above the new cap for as
	 * long as a 600 ms release allows, and the current regulator is handed that reference the
	 * whole time. The ceiling is the protection's own channel to the one reference owner, and
	 * it binds immediately.
	 *
	 * The producer rate-limits how fast the ceiling itself may move (see AP2_CEILING_*_MS), so
	 * "immediately" here means "without waiting for a ride-feel ramp", not "as a step".
	 */
	int32_t  iq_ceiling;
} fast_iq_slew_command_t;

/* Every published field is a naturally aligned Cortex-M4 atomic word access. */
typedef struct {
	volatile uint32_t seq;                   /* odd = update in progress, even = stable */
	volatile int32_t  target;
	volatile uint32_t step_mag_8;
	volatile uint32_t mode;
	volatile uint32_t release_ticks_16k;
	volatile uint32_t release_recip_q32;
	volatile uint32_t zero_policy;
	volatile int32_t  iq_ceiling;
} fast_iq_slew_mailbox_t;

/* Producer API (called from 4 kHz ride/control domain). */

void fast_iq_slew_publish(
	fast_iq_slew_mailbox_t *mb,
	int32_t target,
	fis_mode_t mode,
	uint16_t step_mag_8,
	uint32_t release_ticks_16k,
	fis_zero_policy_t zero_policy,
	int32_t iq_ceiling);

/* Consumer API (called from 16 kHz FOC ISR). */

/* Reset mailbox and consumer state. Legal only at boot or bridge-off cold PREPARE. */
void fast_iq_slew_reset(fast_iq_slew_mailbox_t *mb);

/* True cold PREPARE hook: also makes the externally visible fast output exact zero. */
void fast_iq_slew_cold_prepare(
	fast_iq_slew_mailbox_t *mb,
	int32_t *iq_out);

/*
 * Advance the slew by one 16 kHz tick. Reads the mailbox, advances the fractional
 * accumulator, and writes MS.i_q_setpoint via the provided pointer. Returns the
 * current Iq_ref.
 */
int32_t fast_iq_slew_tick(
	fast_iq_slew_mailbox_t *mb,
	int32_t *iq_out);

/* Read-only word-sized observations used by diagnostics and deterministic tests. */
int32_t fast_iq_slew_current_target(void);
/* The ceiling the ISR is currently clamping the reference to. Observation only. */
int32_t fast_iq_slew_current_ceiling(void);
fis_mode_t fast_iq_slew_current_mode(void);
uint32_t fast_iq_slew_current_step_mag_8(void);
uint32_t fast_iq_slew_current_release_ticks_16k(void);

/* QZERO: the policy of the command the ISR is currently acting on. Consumed by the 16 kHz FOC
 * ISR to decide whether a zero reference may arm Quiet Zero; diagnostics read it too. */
fis_zero_policy_t fast_iq_slew_current_zero_policy(void);

/*
 * Authoritative live final-slew state. The ISR is the sole writer; the 4 kHz producer
 * takes one aligned 32-bit snapshot to choose RISE/FALL/HOLD from trajectory direction,
 * never from target sign or target history.
 */
int32_t fast_iq_slew_current_accumulator_q10(void);

/* Test-only interrupt injection boundaries around every producer store. */
#if defined(FAST_IQ_SLEW_TEST_HOOKS)
typedef enum {
	FIS_PUBLISH_BEFORE_UPDATE = 0,
	FIS_PUBLISH_AFTER_SEQ_ODD,
	FIS_PUBLISH_AFTER_TARGET,
	FIS_PUBLISH_AFTER_STEP,
	FIS_PUBLISH_AFTER_MODE,
	FIS_PUBLISH_AFTER_RELEASE_TICKS,
	FIS_PUBLISH_AFTER_RELEASE_RECIP,
	FIS_PUBLISH_AFTER_ZERO_POLICY,
	FIS_PUBLISH_AFTER_CEILING,
	FIS_PUBLISH_BEFORE_SEQ_EVEN,
	FIS_PUBLISH_AFTER_SEQ_EVEN
} fis_publish_stage_t;

void fast_iq_slew_test_hook(
	fis_publish_stage_t stage,
	fast_iq_slew_mailbox_t *mb);

/* Test access to the real release-rate implementation, not a duplicated model. */
void fast_iq_slew_test_set_accumulator_q10(int32_t accumulator_q10);
uint32_t fast_iq_slew_test_release_rate_q10(
	uint32_t accumulator_q10,
	uint32_t release_ticks_16k);
#endif

#endif /* FAST_IQ_SLEW_H_ */
