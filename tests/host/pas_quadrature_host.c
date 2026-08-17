/*
 * FW-109 v2 Test A: every one of the 16 (previous_state, new_state) raw quadrature transitions,
 * against the REAL shipped decoder (src/pas_quadrature.c), not a copy of its table.
 *
 * The decoder is a PURE, stateless 2-bit lookup - see its own header for the documented table and
 * sign convention (qd[16] times PAS_DIR_SIGN). This test derives the expected classification for
 * every pair from the PHYSICAL meaning of a 2-bit Gray-code quadrature signal (00/01/11/10 forming
 * a ring; adjacent states are one real physical step; diagonal states - 00<->11 and 01<->10 - are
 * impossible for real motion and can only be reached by a missed edge or noise), not by copying
 * the module's own qd[] array, so this is an independent check of the table's PHYSICS, not a
 * tautology against itself.
 */

#include "../common/check.h"

#include "config.h"   /* PAS_DIR_SIGN */
#include "pas_quadrature.h"

/* The physical ring, independent of PAS_DIR_SIGN: state N's PHYSICALLY NEXT state going forward. */
static const uint8_t forward_ring[4] = { 1, 3, 0, 2 }; /* 0->1->3->2->0 */

static int is_forward_step(uint8_t prev, uint8_t next)
{
	return forward_ring[prev] == next;
}
static int is_backward_step(uint8_t prev, uint8_t next)
{
	return forward_ring[next] == prev;
}

int main(void)
{
	printf("FW-109 v2 pas_quadrature.c raw decoder - all 16 transition pairs, against the shipped module\n");
	printf("  PAS_DIR_SIGN = %d\n", (int)PAS_DIR_SIGN);

	uint32_t pairs_checked = 0;
	for (uint8_t prev = 0; prev < 4; prev++) {
		for (uint8_t next = 0; next < 4; next++) {
			int8_t got = pas_quadrature_step(prev, next);
			char label[96];

			/* Range: the decoder must never report anything outside {-1,0,+1}. */
			snprintf(label, sizeof label, "A(%u->%u): result is -1/0/+1, got %d", prev, next, (int)got);
			CHECK(got == -1 || got == 0 || got == 1, label);

			if (prev == next) {
				/* No-change is not a detected transition in production (the caller only
				 * calls this on new_state != previous_state), but the pure function must
				 * still answer deterministically and safely if asked - it must not report
				 * a false step. */
				snprintf(label, sizeof label, "A(%u->%u): no-change reports 0", prev, next);
				CHECK(got == 0, label);
			} else if (is_forward_step(prev, next)) {
				snprintf(label, sizeof label, "A(%u->%u): valid forward step reports +1*PAS_DIR_SIGN", prev, next);
				CHECK(got == (int8_t)(1 * PAS_DIR_SIGN), label);
			} else if (is_backward_step(prev, next)) {
				snprintf(label, sizeof label, "A(%u->%u): valid backward step reports -1*PAS_DIR_SIGN", prev, next);
				CHECK(got == (int8_t)(-1 * PAS_DIR_SIGN), label);
			} else {
				/* The two diagonal pairs (0<->3, 1<->2): a two-bit jump, impossible for
				 * real quadrature motion - must report 0 (illegal), never a direction. */
				snprintf(label, sizeof label, "A(%u->%u): illegal two-bit jump reports 0", prev, next);
				CHECK(got == 0, label);
			}
			pairs_checked++;
		}
	}
	CHECK(pairs_checked == 16U, "A: all 16 (prev,new) pairs were actually exercised");

	/* Pure function: calling it twice with the same inputs must give the same answer (no hidden
	 * state, no side effects) - checked for every pair, not just spot-checked. */
	for (uint8_t prev = 0; prev < 4; prev++) {
		for (uint8_t next = 0; next < 4; next++) {
			int8_t a = pas_quadrature_step(prev, next);
			int8_t b = pas_quadrature_step(prev, next);
			char label[64];
			snprintf(label, sizeof label, "A(%u->%u): stateless/repeatable", prev, next);
			CHECK(a == b, label);
		}
	}

	if (host_test_failures == 0) {
		printf("All pas_quadrature raw-decoder checks passed (16/16 transition pairs).\n");
		return 0;
	}
	printf("\n%d pas_quadrature check(s) FAILED.\n", host_test_failures);
	return 1;
}
