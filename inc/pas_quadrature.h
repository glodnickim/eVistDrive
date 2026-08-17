#ifndef PAS_QUADRATURE_H_
#define PAS_QUADRATURE_H_

#include <stdint.h>

/*
 * FW-107: the PAS quadrature decode table, pulled out of main.c's reg_ADC_processing() so a
 * host test can drive the REAL production decode logic instead of a hand-copied mirror of it.
 * Deliberately the smallest possible extraction: this is a PURE lookup, no state, no MS/MP, no
 * hardware - main.c still owns pas_qstate and every side effect built on top of a step (cadence,
 * torque EMA, ride_episode, diagnostics); fwd_run/Backwards_counter/pas_rev_run moved on to
 * src/pas_direction.c (FW-109 v2 - see its header for why they are ONE module's derivation now,
 * not read a second time here). Only the "which way did this transition go" table moved to this
 * file.
 *
 * Byte-for-byte the same table and the same sign convention as before
 * (qd[16]={0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0} times PAS_DIR_SIGN) - this file changes WHERE
 * that logic lives, not what it computes.
 */

/* +1 = forward, -1 = backward, 0 = illegal two-bit jump (impossible for real motion). */
int8_t pas_quadrature_step(uint8_t prev_state, uint8_t new_state);

#endif /* PAS_QUADRATURE_H_ */
