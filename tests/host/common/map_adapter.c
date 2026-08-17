/*
 * TEST ADAPTER, not firmware. assist_dynamics.c and assist_limits.c both declare
 * `extern int32_t map(int32_t, int32_t, int32_t, int32_t, int32_t);` locally instead of
 * including a shared header - main.c is the only place that DEFINES it
 * (src/main.c:2826-2841). A host build that links either module without also linking
 * main.c (which pulls in the whole hardware/ISR layer - not something a host test can
 * or should link) needs that symbol to exist somewhere else.
 *
 * This is byte-for-byte the same function body as src/main.c's map(), kept in exact sync
 * by hand. It is a KNOWN testability gap in production code, not a feature of this test
 * infrastructure - see documentation/ARCHITECTURE_AUDIT_MOTOR_AGNOSTIC_PL.md finding F4
 * and documentation/testing/TEST_INTERFACES.md. The proper fix is a shared inc/util.h
 * with one definition; that is a production change and out of scope for this card.
 */
#include <stdint.h>

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
	if (x < in_min)
		return out_min;
	else if (x > in_max)
		return out_max;
	else if ((in_max - in_min) > (out_max - out_min))
		return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
	else
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
