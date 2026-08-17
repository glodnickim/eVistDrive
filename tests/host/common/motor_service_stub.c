/*
 * TEST ADAPTER, not firmware. inc/motor_service.h documents its own real implementations
 * as living in main.c ("both are defined in main.c because they read state that belongs
 * to the motor layer - the Hall capture counters, the TIMER0 PWM registers, the virtual-
 * EEPROM writer"). ride_control.c calls both unconditionally in source
 * (inside `if (input->walk_active)` / `if (input->position_calibration_active)` - see
 * ride_control.c), so the symbols must resolve at link time even in scenarios that never
 * take either branch.
 *
 * Every scenario this test infrastructure runs sets walk_active=false and
 * position_calibration_active=false, so neither stub body below is ever reached at
 * runtime - they exist only so the linker is satisfied. Walk Assist and Hall calibration
 * are out of scope for this card (see documentation/TEST_INFRASTRUCTURE_FOUNDATION_REPORT_PL.md,
 * section 18 "co nadal NIE należy refaktorować" / observability gaps).
 */
#include <stdint.h>

uint16_t walk_assist_iq_request(void)
{
	return 0U;
}

uint16_t hall_calibration_iq_request(void)
{
	return 0U;
}
