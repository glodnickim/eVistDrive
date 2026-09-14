/*
 * QS-3D: single final Iq slew owner at 16 kHz FOC rate.
 *
 * Architecture migration only: the physical slew rate is preserved from the 4 kHz
 * assist_dynamics.c implementation. G532 rate tuning is QS-3E.
 *
 * EXACT fixed-point fractional slew (16  kHz):
 *
 *   The old 4 kHz owner advanced a Q8 accumulator by one per-control step S (Q8)
 *   each 4 kHz tick. The 16 kHz owner runs FOUR times per control tick, so it must
 *   advance by S/4 per 16 kHz tick to keep the identical dIq/dt. Instead of
 *   floor(S/4) (which loses precision), the accumulator lives in Q10 (two extra
 *   fractional bits) and adds the FULL per-control step S (Q8) as a Q10 increment
 *   every 16 kHz tick:
 *
 *       acc_Q10 += S          (S = old per-control Q8 step)
 *       after 4 ticks:  acc_Q10 += 4*S  == S in Q8  (exactly one old control step)
 *       Iq = (acc_Q10 + 512) >> 10        (round-half-up publication)
 *
 *   So at every old 4 kHz observation point the new Iq equals the old Iq exactly
 *   (same rounding rule), and between those points the slew advances in finer
 *   62.5 us increments. Total rise/fall/release timing is unchanged.
 *
 * The 4 kHz producer precomputes S (Q8); the ISR does no division and no float.
 * A release command carries a duration and reciprocal. At the edge INTO the release the ISR
 * computes ceil(live_accumulator / duration) with multiply/shift/remainder only.
 * The seqlock read is bounded and retains the last verified command on failure.
 */

#include "fast_iq_slew.h"

#include <limits.h>
#include <stddef.h>

#define FIS_ACC_SHIFT 10        /* Q10 accumulator (2 extra bits make /4 exact) */
#define FIS_Q_SHIFT   8         /* input step (Q8) matches IQ_RAMP_Q_SHIFT */

/* The eighth word is the protection ceiling. Every member stays a naturally aligned
 * single-word access, which is what the seqlock relies on. */
_Static_assert(sizeof(fast_iq_slew_mailbox_t) == 32U,
	"mailbox must contain exactly eight 32-bit words");
_Static_assert(offsetof(fast_iq_slew_mailbox_t, seq) == 0U,
	"sequence word must lead the mailbox");
_Static_assert(offsetof(fast_iq_slew_mailbox_t, target) == 4U,
	"payload words must be naturally aligned");
_Static_assert(offsetof(fast_iq_slew_mailbox_t, iq_ceiling) == 28U,
	"the ceiling is the last payload word and must stay single-word aligned");

#ifdef __GNUC__
#if defined(__arm__) || defined(__thumb__) || defined(__aarch64__)
#define FIS_MEMORY_BARRIER() __asm volatile ("dmb 0xF" ::: "memory")
#else
#define FIS_MEMORY_BARRIER() __asm volatile ("" ::: "memory")
#endif
#else
#define FIS_MEMORY_BARRIER() ((void)0)
#endif

#if defined(FAST_IQ_SLEW_TEST_HOOKS)
#define FIS_TEST_HOOK(stage_, mb_) fast_iq_slew_test_hook((stage_), (mb_))
#else
#define FIS_TEST_HOOK(stage_, mb_) ((void)0)
#endif

/* ------------------------------------------------------------------ */
/* Internal slew state (owned exclusively by the 16 kHz ISR context).  */
/* ------------------------------------------------------------------ */

typedef struct {
	volatile int32_t accumulator_q10; /* ISR-owned; aligned atomic snapshot for 4 kHz producer */
	int32_t  rate;             /* per-16kHz-tick Q8->Q10 increment (signed) */
	fast_iq_slew_command_t command; /* last command accepted as one coherent generation */
} fis_state_t;

static fis_state_t fis;

/* Round-half-up to counts from a Q10 accumulator. */
static inline int32_t fis_to_iq(int32_t acc_q10)
{
	return (acc_q10 + (1 << (FIS_ACC_SHIFT - 1))) >> FIS_ACC_SHIFT;
}

static bool fis_command_equal(
	const fast_iq_slew_command_t *a,
	const fast_iq_slew_command_t *b)
{
	return a->target == b->target &&
		a->step_mag_8 == b->step_mag_8 &&
		a->mode == b->mode &&
		a->release_ticks_16k == b->release_ticks_16k &&
		a->release_recip_q32 == b->release_recip_q32 &&
		a->iq_ceiling == b->iq_ceiling &&
		a->zero_policy == b->zero_policy;
}

/*
 * Read one stable generation. Candidate fields stay local until BOTH sequence
 * reads verify them. If every bounded attempt fails, the caller's command is not
 * touched and therefore remains the last verified command.
 */
static bool fis_mailbox_read_verified(
	fast_iq_slew_mailbox_t *mb,
	fast_iq_slew_command_t *verified)
{
	for (uint32_t retry = 0; retry < FAST_IQ_SLEW_READ_ATTEMPTS; retry++) {
		fast_iq_slew_command_t candidate;
		uint32_t seq1 = mb->seq;
		FIS_MEMORY_BARRIER();

		candidate.target = mb->target;
		candidate.step_mag_8 = mb->step_mag_8;
		candidate.mode = mb->mode;
		candidate.release_ticks_16k = mb->release_ticks_16k;
		candidate.release_recip_q32 = mb->release_recip_q32;
		candidate.iq_ceiling = mb->iq_ceiling;
		candidate.zero_policy = mb->zero_policy;

		FIS_MEMORY_BARRIER();
		uint32_t seq2 = mb->seq;
		if (seq1 == seq2 && (seq1 & 1U) == 0U) {
			*verified = candidate;
			return true;
		}
	}

	return false;
}

/*
 * Exact ceil(numerator / denominator), using the producer's floor(2^32 / d).
 * The reciprocal estimate can be one below floor(n/d). Correct that first when
 * remainder >= denominator, then apply the final non-zero-remainder ceil. For the
 * practical signed-Q10 range this is mathematically exact without ISR division.
 */
static uint32_t fis_live_release_rate(
	uint32_t accumulator_q10,
	const fast_iq_slew_command_t *command)
{
	uint32_t ticks = command->release_ticks_16k;
	if (accumulator_q10 == 0U || ticks == 0U) {
		return 0U;
	}
	if (ticks == 1U) {
		return accumulator_q10;
	}

	uint32_t quotient = (uint32_t)(((uint64_t)accumulator_q10 *
		command->release_recip_q32) >> 32);
	uint32_t remainder = accumulator_q10 - (quotient * ticks);
	if (remainder >= ticks) {
		quotient++;
		remainder -= ticks;
	}
	if (remainder != 0U) {
		quotient++;
	}
	return (quotient == 0U) ? 1U : quotient;
}

int32_t fast_iq_slew_tick(
	fast_iq_slew_mailbox_t *mb,
	int32_t *iq_out)
{
	fast_iq_slew_command_t candidate;
	bool command_changed = false;
	/*
	 * Captured BEFORE the command is replaced, so the release branch below can tell a new
	 * release from a release whose command merely moved in some other field.
	 */
	fis_mode_t prev_mode = (fis_mode_t)fis.command.mode;
	uint32_t prev_release_ticks = fis.command.release_ticks_16k;
	uint32_t prev_release_recip = fis.command.release_recip_q32;
	if (fis_mailbox_read_verified(mb, &candidate)) {
		command_changed = !fis_command_equal(&candidate, &fis.command);
		if (command_changed) {
			fis.command = candidate;
		}
	}

	fis_mode_t mode = (fis_mode_t)fis.command.mode;
	if (command_changed) {
		int32_t step = (int32_t)fis.command.step_mag_8;
		switch (mode) {
		case FIS_MODE_RISE:
		case FIS_MODE_FALL: {
			int64_t target_q10 = (int64_t)fis.command.target << FIS_ACC_SHIFT;
			int32_t live_q10 = fis.accumulator_q10;
			fis.rate = (target_q10 > live_q10) ? step : -step;
			if (target_q10 == live_q10) {
				fis.rate = 0;
			}
			break;
		}
		case FIS_MODE_HOLD:
			fis.rate = 0;
			break;
		case FIS_MODE_RELEASE:
		case FIS_MODE_SAFETY: {
			/*
			 * This is the critical live-state edge: target history is irrelevant.
			 *
			 * But it is the RELEASE's edge, not the command's. The command carries
			 * fields that move on their own during a release - the protection
			 * ceiling is rate-limited and changes every tick - and re-deriving
			 * "reach zero in release_ms" on each of those turns one linear ramp
			 * into a sequence of ever-smaller ones that approaches zero instead of
			 * arriving at it. Measured before this guard existed: 831 ms to zero
			 * against a 190 ms setting.
			 *
			 * So derive when the release begins, when its own duration changes, or
			 * when the rate has been cancelled - the ceiling clamp zeroes it when it
			 * binds, and a protection cutting in mid-release must take effect at
			 * once. Otherwise keep the rate that was derived at the edge.
			 */
			bool was_release = (prev_mode == FIS_MODE_RELEASE) ||
				(prev_mode == FIS_MODE_SAFETY);
			bool contract_changed =
				(prev_release_ticks != fis.command.release_ticks_16k) ||
				(prev_release_recip != fis.command.release_recip_q32);
			if (!was_release || contract_changed || fis.rate >= 0) {
				fis.rate = -(int32_t)fis_live_release_rate(
					(uint32_t)((fis.accumulator_q10 > 0) ?
						fis.accumulator_q10 : 0),
					&fis.command);
			}
			break;
		}
		case FIS_MODE_FORCE_ZERO:
		case FIS_MODE_BYPASS:
		default:
			fis.rate = 0;
			break;
		}
	}

	int32_t iq_ref;
	switch (mode) {
	case FIS_MODE_FORCE_ZERO:
		fis.accumulator_q10 = 0;
		fis.rate = 0;
		iq_ref = 0;
		break;

	case FIS_MODE_BYPASS:
		fis.accumulator_q10 = fis.command.target << FIS_ACC_SHIFT;
		fis.rate = 0;
		iq_ref = fis.command.target;
		break;

	case FIS_MODE_HOLD:
		fis.rate = 0;
		iq_ref = fis_to_iq(fis.accumulator_q10);
		break;

	case FIS_MODE_RISE:
	case FIS_MODE_FALL: {
		int32_t target_q10 = fis.command.target << FIS_ACC_SHIFT;
		fis.accumulator_q10 += fis.rate;
		if (fis.rate > 0 && fis.accumulator_q10 > target_q10) {
			fis.accumulator_q10 = target_q10;
		} else if (fis.rate < 0 && fis.accumulator_q10 < target_q10) {
			fis.accumulator_q10 = target_q10;
		}
		if (fis.accumulator_q10 == target_q10) {
			fis.rate = 0;
		}
		iq_ref = fis_to_iq(fis.accumulator_q10);
		if (fis.command.target == 0 && iq_ref == 0) {
			fis.accumulator_q10 = 0;
			fis.rate = 0;
		}
		break;
	}

	case FIS_MODE_RELEASE:
	case FIS_MODE_SAFETY:
		fis.accumulator_q10 += fis.rate;
		if (fis.accumulator_q10 <= 0 || fis_to_iq(fis.accumulator_q10) == 0) {
			/* Published zero and fractional state become exact zero on the same tick. */
			fis.accumulator_q10 = 0;
			fis.rate = 0;
			iq_ref = 0;
		} else {
			iq_ref = fis_to_iq(fis.accumulator_q10);
		}
		break;

	default:
		fis.accumulator_q10 = 0;
		fis.rate = 0;
		iq_ref = 0;
		break;
	}

	if (iq_ref < 0) {
		iq_ref = 0;
		fis.accumulator_q10 = 0;
		fis.rate = 0;
	}

	/*
	 * THE HARD CEILING, applied last and in every mode.
	 *
	 * This is the protection channel described in fast_iq_slew_command_t. It binds on the
	 * ACCUMULATOR, not only on the published value, so a reference held above a newly lowered
	 * cap cannot resume from the old state when the cap is raised again - it resumes from where
	 * it was actually allowed to be, and climbs back under the ordinary attack ramp.
	 *
	 * It can only ever LOWER the reference. A ceiling above the current reference does nothing,
	 * so raising one never produces a step; that is what makes recovering from a limit bumpless
	 * without a second mechanism to manage it.
	 *
	 * A ceiling of 0 is honoured literally: zero allowed current. The producer never publishes
	 * a negative one, and a negative value would be meaningless here, so it is treated as zero.
	 */
	{
		int32_t ceiling = fis.command.iq_ceiling;
		if (ceiling < 0) {
			ceiling = 0;
		}
		if (iq_ref > ceiling) {
			iq_ref = ceiling;
			fis.accumulator_q10 = ceiling << FIS_ACC_SHIFT;
			if (fis.rate > 0) {
				fis.rate = 0;
			}
		}
	}

	*iq_out = iq_ref;
	return iq_ref;
}

void fast_iq_slew_publish(
	fast_iq_slew_mailbox_t *mb,
	int32_t target,
	fis_mode_t mode,
	uint16_t step_mag_8,
	uint32_t release_ticks_16k,
	fis_zero_policy_t zero_policy,
	int32_t iq_ceiling)
{
	uint32_t release_recip_q32 = 0U;
	if (release_ticks_16k > 1U) {
		/* Producer-side division only. The ISR receives the finished reciprocal. */
		release_recip_q32 = (uint32_t)(UINT64_C(0x100000000) / release_ticks_16k);
	}

	FIS_TEST_HOOK(FIS_PUBLISH_BEFORE_UPDATE, mb);
	uint32_t seq_odd = (mb->seq + 1U) | 1U;
	mb->seq = seq_odd;
	/* DMB + compiler clobber: odd is globally visible before the first payload store. */
	FIS_MEMORY_BARRIER();
	FIS_TEST_HOOK(FIS_PUBLISH_AFTER_SEQ_ODD, mb);

	mb->target = target;
	FIS_TEST_HOOK(FIS_PUBLISH_AFTER_TARGET, mb);
	mb->step_mag_8 = step_mag_8;
	FIS_TEST_HOOK(FIS_PUBLISH_AFTER_STEP, mb);
	mb->mode = (uint32_t)mode;
	FIS_TEST_HOOK(FIS_PUBLISH_AFTER_MODE, mb);
	mb->release_ticks_16k = release_ticks_16k;
	FIS_TEST_HOOK(FIS_PUBLISH_AFTER_RELEASE_TICKS, mb);
	mb->release_recip_q32 = release_recip_q32;
	FIS_TEST_HOOK(FIS_PUBLISH_AFTER_RELEASE_RECIP, mb);
	mb->zero_policy = (uint32_t)zero_policy;
	FIS_TEST_HOOK(FIS_PUBLISH_AFTER_ZERO_POLICY, mb);
	mb->iq_ceiling = iq_ceiling;
	FIS_TEST_HOOK(FIS_PUBLISH_AFTER_CEILING, mb);

	/* DMB + compiler clobber: all payload stores complete before stable publication. */
	FIS_MEMORY_BARRIER();
	FIS_TEST_HOOK(FIS_PUBLISH_BEFORE_SEQ_EVEN, mb);
	mb->seq = seq_odd + 1U;
	FIS_MEMORY_BARRIER();
	FIS_TEST_HOOK(FIS_PUBLISH_AFTER_SEQ_EVEN, mb);
}

void fast_iq_slew_reset(fast_iq_slew_mailbox_t *mb)
{
	fis.accumulator_q10 = 0;
	fis.rate = 0;
	fis.command.target = 0;
	fis.command.step_mag_8 = 0U;
	fis.command.mode = (uint32_t)FIS_MODE_FORCE_ZERO;
	fis.command.release_ticks_16k = 0U;
	fis.command.release_recip_q32 = 0U;
	fis.command.iq_ceiling = 0;
	fis.command.zero_policy = (uint32_t)FIS_ZERO_POLICY_NONE;

	/* Reset is exclusive with the consumer, but still leave a normal stable generation. */
	mb->seq = 1U;
	FIS_MEMORY_BARRIER();
	mb->target = 0;
	mb->step_mag_8 = 0U;
	mb->mode = (uint32_t)FIS_MODE_FORCE_ZERO;
	mb->release_ticks_16k = 0U;
	mb->release_recip_q32 = 0U;
	mb->iq_ceiling = 0;
	mb->zero_policy = (uint32_t)FIS_ZERO_POLICY_NONE;
	FIS_MEMORY_BARRIER();
	mb->seq = 2U;
}

void fast_iq_slew_cold_prepare(
	fast_iq_slew_mailbox_t *mb,
	int32_t *iq_out)
{
	fast_iq_slew_reset(mb);
	*iq_out = 0;
}

int32_t fast_iq_slew_current_ceiling(void)
{
	return fis.command.iq_ceiling;
}

int32_t fast_iq_slew_current_target(void)
{
	return fis.command.target;
}

fis_mode_t fast_iq_slew_current_mode(void)
{
	return (fis_mode_t)fis.command.mode;
}

uint32_t fast_iq_slew_current_step_mag_8(void)
{
	return fis.command.step_mag_8;
}

uint32_t fast_iq_slew_current_release_ticks_16k(void)
{
	return fis.command.release_ticks_16k;
}

fis_zero_policy_t fast_iq_slew_current_zero_policy(void)
{
	/*
	 * QZERO: the policy of the last VERIFIED command, i.e. of the same generation that produced
	 * this tick's Iq_ref. Read from the consumer's own copy rather than from the mailbox, so a
	 * publish that lands mid-tick can never pair one generation's reference with another
	 * generation's policy.
	 */
	return (fis_zero_policy_t)fis.command.zero_policy;
}

int32_t fast_iq_slew_current_accumulator_q10(void)
{
	/* Aligned 32-bit load is atomic on Cortex-M4; volatile prevents foreground caching. */
	return fis.accumulator_q10;
}

#if defined(FAST_IQ_SLEW_TEST_HOOKS)
void fast_iq_slew_test_set_accumulator_q10(int32_t accumulator_q10)
{
	fis.accumulator_q10 = accumulator_q10;
	fis.rate = 0;
}

uint32_t fast_iq_slew_test_release_rate_q10(
	uint32_t accumulator_q10,
	uint32_t release_ticks_16k)
{
	fast_iq_slew_command_t command = { 0 };
	command.release_ticks_16k = release_ticks_16k;
	if (release_ticks_16k > 1U) {
		command.release_recip_q32 =
			(uint32_t)(UINT64_C(0x100000000) / release_ticks_16k);
	}
	return fis_live_release_rate(accumulator_q10, &command);
}
#endif
