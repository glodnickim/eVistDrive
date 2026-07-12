#include "legacy_assist.h"

/*
 * Transitional bridge. The monolith is renamed explicitly before its body is
 * split into mode, dynamics and limit modules in subsequent commits.
 */
uint16_t legacy_assist_calculate_monolith(void);

uint16_t legacy_assist_calculate(void)
{
	return legacy_assist_calculate_monolith();
}
