/*
 * FW-113.1: normal RUN minimum Iq 2 -> 0 (Walk Assist).
 *
 * The real shipped modules walk_assist_motor.c + walk_speed_controller.c are linked
 * directly (no stubs) and driven tick by tick with a synthetic Hall input. The JS
 * model in tests/fw060_walk_speed_controller.js covers the pure controller; this
 * harness covers the STATE MACHINE the model cannot see:
 *
 *   S1 (new, overspeed): normal RUN with measured_erps > target_erps must descend
 *       to a TRUE iq_cmd == 0. START must NOT be re-armed by the descent
 *       (startup_complete stays latched), no false LIMIT/STALL, and with Hall still
 *       valid the state remains a normal REGULATE/RUN.
 *   S2 (regression): below target the PI must still be able to INCREASE Iq.
 *   S3 (limits unchanged): START ceiling 40, REACQUIRE ceiling 24, LIMIT ceiling 15.
 *
 * The one production change under test is src/walk_assist_motor.c line 44:
 *   #define WA_MOTOR_RUN_MIN_IQ   2   ->  0
 * Every other constant is untouched.
 */

#include "../common/check.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "walk_assist_motor.h"

#define TARGET_RPM 30U
#define TARGET_ERPS_REF 40U

/* Synthetic Hall: a constant motor_erps rotates at a fixed rate. */
static uint16_t hall_period_ticks_4k(uint16_t erps)
{
	/* 6 Hall transitions per electrical rev, caller ticks at 4 kHz. */
	uint32_t denom = 6U * (uint32_t)erps;
	return (uint16_t)((denom > 0U) ? (4000U / denom) : 0U);
}

static uint16_t hall_ticks_500k(uint16_t erps)
{
	/* 500 kHz timer ticks between Hall transitions for a constant ERPS. */
	return (uint16_t)(500000U / (6U * (uint32_t)erps));
}

typedef struct {
	walk_motor_input_t in;
	uint16_t age;              /* 4 kHz ticks since the last synthetic Hall edge */
	uint16_t period;           /* Hall edge period in 4 kHz ticks */
	uint16_t hall_ticks;       /* 500 kHz value to report on each edge */
	int32_t last_iq;           /* feeds motor_iq_reference */
	int32_t iq_actual;         /* feeds the jam watchdog */
	uint16_t tick;
} drive_t;

static void drive_reset(drive_t *d, bool no_hall)
{
	memset(d, 0, sizeof *d);
	d->in.active = 1;
	d->in.target_chainring_rpm = TARGET_RPM;
	d->in.max_wheel_speed_x100 = 700;
	if (no_hall) {
		/* Age beyond the 200 ms timeout, forever: no Hall edges. */
		d->age = 0xFFFFU;
	}
}

/* Set the rotor speed to a constant ERPS and return the running output. */
static void drive_set_erps(drive_t *d, uint16_t erps)
{
	d->period = hall_period_ticks_4k(erps);
	d->hall_ticks = hall_ticks_500k(erps);
}

static void drive_tick(drive_t *d, walk_motor_output_t *out)
{
	if (d->period > 0U) {
		if (++d->age >= d->period) {
			d->age = 0;             /* new Hall edge: report the fresh period */
			d->in.motor_hall_ticks = d->hall_ticks;
		}
		d->in.motor_erps_age_ticks = d->age;
	} else {
		/* No Hall configured: keep the age beyond the timeout. */
		d->in.motor_erps_age_ticks = 0xFFFFU;
		d->in.motor_hall_ticks = 0;
	}
	d->in.motor_iq_reference = d->last_iq;
	d->in.motor_iq_actual = d->iq_actual;
	d->last_iq = walk_motor_update(&d->in, out);
	d->tick++;
}

static bool is_state(const walk_motor_output_t *o, walk_motor_state_t s)
{
	return o->state == (uint8_t)s;
}

int main(void)
{
	printf("FW-113.1 walk_assist_motor.c + walk_speed_controller.c - RUN minimum Iq 2 -> 0\n");
	printf("  target 30 rpm -> %u ERPS, START_MAX 40, RUN_MAX 40, REACQUIRE 24, LIMIT 15\n",
		TARGET_ERPS_REF);

	walk_motor_release(); /* clear the stall latch from boot */

	/* ======================================================================
	 * S1: normal RUN above target must reach a TRUE iq_cmd == 0.
	 * - no re-arm of START (startup_complete latches),
	 * - no false LIMIT/STALL,
	 * - Hall valid -> state stays REGULATE (normal RUN).
	 * ====================================================================== */
	{
		drive_t d;
		walk_motor_release();
		drive_reset(&d, true);      /* start with NO Hall: the START ramp builds torque */
		walk_motor_output_t out;
		bool start_max_seen = false;
		int32_t max_iq = 0;
		/* Let the one-shot START finish its 40 Iq ramp without Hall. */
		for (uint32_t i = 0; i < 4000U; i++) {
			drive_tick(&d, &out);
			if (out.iq_target > max_iq) {
				max_iq = out.iq_target;
			}
			if (out.iq_target == 40) {
				start_max_seen = true;
			}
		}
		CHECK(start_max_seen, "S1: one-shot START reached its 40 Iq ceiling");
		CHECK(max_iq <= 40, "S1: START never exceeded 40 Iq");

		/* Hall returns at exactly the target speed: RUN handover, PI takes over. */
		drive_set_erps(&d, TARGET_ERPS_REF);
		for (uint32_t i = 0; i < 3000U; i++) {
			drive_tick(&d, &out);
		}
		CHECK(out.iq_target < 40, "S1: RUN handover let the PI pull Iq below the START ceiling");
		CHECK(is_state(&out, WA_STATE_REGULATE), "S1: handover left the state in normal RUN (REGULATE)");

		/* Overspeed: measured well above target. The PI must go all the way to 0. */
		drive_set_erps(&d, 90U);
		bool descended_to_zero = false;
		int32_t min_iq = 0x7FFFFFFF;
		bool lim_false = true, stall_false = true;
		for (uint32_t i = 0; i < 20000U; i++) {
			drive_tick(&d, &out);
			if (out.iq_target < min_iq) {
				min_iq = out.iq_target;
			}
			if (out.iq_target == 0) {
				descended_to_zero = true;
			}
			if (is_state(&out, WA_STATE_LIMIT)) {
				lim_false = false;
			}
			if (is_state(&out, WA_STATE_STALL)) {
				stall_false = false;
			}
		}
		CHECK(descended_to_zero, "S1: normal RUN above target descended to a TRUE iq_cmd == 0");
		CHECK(min_iq == 0, "S1: Iq floor is exactly 0 in normal RUN (no positive keepalive)");
		CHECK(out.iq_target == 0, "S1: iq_cmd == 0 with measured_erps > target_erps");
		CHECK(is_state(&out, WA_STATE_REGULATE), "S1: still normal RUN (REGULATE) after the descent to 0");
		CHECK((out.flags & WA_FLAG_HALL_VALID) != 0, "S1: Hall remained valid through the descent");
		CHECK((out.flags & WA_FLAG_START_ACTIVE) == 0, "S1: the descent never re-armed START");
		CHECK(lim_false, "S1: no false LIMIT on the overspeed descent");
		CHECK(stall_false, "S1: no false STALL on the overspeed descent");
	}

	/* ======================================================================
	 * S2: regression - below target the PI must still INCREASE Iq from 0.
	 * ====================================================================== */
	{
		drive_t d;
		walk_motor_release();
		drive_reset(&d, true);
		walk_motor_output_t out;
		for (uint32_t i = 0; i < 4000U; i++) {
			drive_tick(&d, &out);   /* START ramp, no Hall yet */
		}
		drive_set_erps(&d, TARGET_ERPS_REF);
		for (uint32_t i = 0; i < 3000U; i++) {
			drive_tick(&d, &out);   /* handover into RUN */
		}
		drive_set_erps(&d, 90U);
		for (uint32_t i = 0; i < 20000U; i++) {
			drive_tick(&d, &out);   /* overspeed: descend to true 0 */
		}
		CHECK(out.iq_target == 0, "S2: settled at true 0 Iq above target (setup)");

		/* Now the rotor slows below target: the PI must climb back up. */
		drive_set_erps(&d, 10U);
		int32_t iq_before = 0;
		int32_t iq_after = 0;
		for (uint32_t i = 0; i < 4000U; i++) {
			drive_tick(&d, &out);
			if (i == 1999U) {
				iq_before = out.iq_target;
			}
		}
		iq_after = out.iq_target;
		CHECK(iq_after > iq_before, "S2: PI increased Iq below target (regression)");
		CHECK(iq_after > 0, "S2: below target the PI recovered a positive Iq");
		CHECK(is_state(&out, WA_STATE_REGULATE), "S2: below-target climb stayed in normal RUN (REGULATE)");
		CHECK(iq_after <= 40, "S2: below-target climb never exceeded RUN_MAX 40");
	}

	/* ======================================================================
	 * S3: START / REACQUIRE / LIMIT ceilings are unchanged.
	 * ====================================================================== */
	{
		/* START: no Hall -> the ramp must cap exactly at 40. */
		drive_t d;
		walk_motor_release();
		drive_reset(&d, true);
		walk_motor_output_t out;
		int32_t start_peak = 0;
		for (uint32_t i = 0; i < 4000U; i++) {
			drive_tick(&d, &out);
			if (out.iq_target > start_peak) {
				start_peak = out.iq_target;
			}
		}
		CHECK(start_peak == 40, "S3: START ceiling stays 40 Iq");
		if (start_peak != 40) {
			printf("    (START peak was %d)\n", (int)start_peak);
		}

		/* REACQUIRE: after motion, Hall is lost at low torque -> bounded 24 Iq. */
		walk_motor_release();
		drive_reset(&d, true);
		for (uint32_t i = 0; i < 4000U; i++) {
			drive_tick(&d, &out);   /* START ramp */
		}
		drive_set_erps(&d, TARGET_ERPS_REF);
		for (uint32_t i = 0; i < 3000U; i++) {
			drive_tick(&d, &out);   /* RUN handover, Hall present */
		}
		/* Remove Hall completely; iq low, no drive -> bounded reacquire. */
		d.in.motor_erps_age_ticks = 0xFFFFU;
		d.in.motor_hall_ticks = 0;
		d.period = 0;
		int32_t reacquire_peak = 0;
		uint8_t reacquire_seen = 0;
		for (uint32_t i = 0; i < 6000U; i++) {
			drive_tick(&d, &out);
			if (out.iq_target > reacquire_peak) {
				reacquire_peak = out.iq_target;
			}
			if ((out.flags & WA_FLAG_REACQUIRE) != 0) {
				reacquire_seen = 1;
			}
		}
		CHECK(reacquire_seen, "S3: Hall loss entered the bounded reacquire");
		CHECK(reacquire_peak <= 24, "S3: REACQUIRE ceiling stays 24 Iq");
		if (reacquire_peak > 24) {
			printf("    (peak was %d)\n", (int)reacquire_peak);
		}

		/* LIMIT: hold the rotor stalled with current -> jam latches LIMIT (15 Iq). */
		walk_motor_release();
		drive_reset(&d, true);
		for (uint32_t i = 0; i < 4000U; i++) {
			drive_tick(&d, &out);   /* START ramp */
		}
		drive_set_erps(&d, TARGET_ERPS_REF);
		for (uint32_t i = 0; i < 3000U; i++) {
			drive_tick(&d, &out);   /* RUN handover */
		}
		d.in.motor_erps_age_ticks = 0xFFFFU;
		d.in.motor_hall_ticks = 0;
		d.period = 0;
		d.iq_actual = 24;           /* real current present while the rotor is stalled */
		bool limit_seen = false;
		int32_t limit_cap_after = 0;
		for (uint32_t i = 0; i < 12000U; i++) {
			drive_tick(&d, &out);
			if (is_state(&out, WA_STATE_LIMIT)) {
				limit_seen = true;
				limit_cap_after = out.iq_cap;
				break;
			}
		}
		CHECK(limit_seen, "S3: sustained stall latched LIMIT");
		CHECK(limit_cap_after == 15, "S3: LIMIT ceiling stays 15 Iq");
		if (limit_cap_after != 15) {
			printf("    (cap was %d)\n", (int)limit_cap_after);
		}
	}

	if (host_test_failures == 0) {
		printf("All FW-113.1 walk RUN-min checks passed.\n");
		return 0;
	}
	printf("\n%d FW-113.1 walk RUN-min check(s) FAILED.\n", host_test_failures);
	return 1;
}
