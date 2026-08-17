#include "pas_quadrature.h"

#include "config.h"   /* PAS_DIR_SIGN only - see header. */

int8_t pas_quadrature_step(uint8_t prev_state, uint8_t new_state)
{
	static const int8_t qd[16] = {0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};
	uint8_t index = (uint8_t)(((prev_state & 0x3U) << 2) | (new_state & 0x3U));
	return (int8_t)(qd[index] * PAS_DIR_SIGN);
}
