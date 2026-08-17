/*
 * FW-113.2: Walk Assist - no wall-clock hold timeout, and full reason-bit diagnostics.
 *
 * Links the real walk_assist_motor.c + walk_speed_controller.c (no stubs) and drives them
 * tick by tick with a synthetic Hall input, exactly like FW-113.1's walk_assist_run_min_host.c.
 *
 * What this card proves:
 *
 *   S1 LONG HOLD: a 60 s hold (240,000 ticks at 4 kHz) with the rotor at the target must stay
 *       live. The module has NO wall-clock hold timeout: no STALL/LIMIT, no JAM, iq stays
 *       positive, and `reason` never carries a stale block bit. Hold time alone can never
 *       stop WA.
 *   S2 BUTTON RELEASE: a short release (active=0) must zero iq and leave no STALL latch; a
 *       re-engage restarts normally (START re-arms) instead of staying blocked.
 *   S3 CAN RELEASE after a long hold: 60 s live, then the request drops (active=0) -> iq 0,
 *       no STALL, and a re-engage drives again (the STALL latch only ever fires on a real
 *       jam/LIMIT, never on request release).
 *   S4 BRAKE: brake=1 during RUN -> iq 0 and reason carries WA_REASON_BRAKE; brake=0 resumes.
 *   S5 SPEED GATE: wheel_speed >= ceiling -> iq 0 and reason carries WA_REASON_SPEED_GATE;
 *       below the ceiling resumes.
 *   S6 FW-113.1 overspeed regression: iq_cmd descends to a TRUE 0, stays REGULATE, no false
 *       LIMIT/STALL (RUN_MIN_IQ == 0 preserved).
 *   S7 LOAD COMP: below target the PI must still INCREASE Iq (regression, FW-113.1 S2).
 *   S8 REACQUIRE ceiling: Hall loss at low torque -> bounded reacquire, never above 24 Iq.
 *   S9 LIMIT/JAM/STALL: a real sustained stall still latches LIMIT (15 Iq) then STALL; reason
 *       carries JAM then LIMIT then STALL; the latch clears ONLY on a complete release.
 *   S10 REASON BITS: each real gate sets exactly its own reason bit (BRAKE / SPEED_GATE /
 *       HALL / JAM / STALL / LIMIT), and NO reason value ever contains a bit outside the
 *       defined set - in particular there is no timeout bit, because no hold timeout exists.
 *
 * The activation-level reason bits (WA_REASON_NO_CAN_REQUEST / WA_REASON_BUTTON_RELEASED)
 * are owned by main.c (it alone knows the request origin) and appear in aggregate frame
 * 0x10228; the module here proves its own half (BRAKE/ERROR/SPEED_GATE from its inputs,
 * HALL/JAM/STALL/LIMIT from its state machine).
 */

#include "../common/check.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "walk_assist_motor.h"

#define TARGET_RPM 30U
#define TARGET_ERPS_REF 40U
#define MAX_WHEEL_X100 700U
#define LONG_HOLD_TICKS 240000U   /* 60 s @ 4 kHz */
#define SPEED_GATE_X100 (MAX_WHEEL_X100 - 10U)

/* Every bit currently defined in the WA_REASON_* set. */
#define KNOWN_REASON_MASK 0x01FFU

static uint16_t hall_period_ticks_4k(uint16_t erps)
{
	uint32_t denom = 6U * (uint32_t)erps;
	return (uint16_t)((denom > 0U) ? (4000U / denom) : 0U);
}

static uint16_t hall_ticks_500k(uint16_t erps)
{
	return (uint16_t)(500000U / (6U * (uint32_t)erps));
}

typedef struct {
	walk_motor_input_t in;
	uint16_t age;
	uint16_t period;
	uint16_t hall_ticks;
	int32_t last_iq;
	int32_t iq_actual;
	uint16_t tick;
} drive_t;

static void drive_reset(drive_t *d, bool no_hall)
{
	memset(d, 0, sizeof *d);
	d->in.active = 1;
	d->in.target_chainring_rpm = TARGET_RPM;
	d->in.max_wheel_speed_x100 = MAX_WHEEL_X100;
	if (no_hall) {
		d->age = 0xFFFFU;
	}
}

static void drive_set_erps(drive_t *d, uint16_t erps)
{
	d->period = hall_period_ticks_4k(erps);
	d->hall_ticks = hall_ticks_500k(erps);
}

static void drive_tick(drive_t *d, walk_motor_output_t *out)
{
	if (d->period > 0U) {
		if (++d->age >= d->period) {
			d->age = 0;
			d->in.motor_hall_ticks = d->hall_ticks;
		}
		d->in.motor_erps_age_ticks = d->age;
	} else {
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

static bool reason_has_only_known_bits(uint16_t reason)
{
	return (reason & ~KNOWN_REASON_MASK) == 0U;
}

/* Drive a clean RUN session up to a stable target and return the last output. */
static void run_session(drive_t *d, walk_motor_output_t *out)
{
	walk_motor_release();
	drive_reset(d, true);
	for (uint32_t i = 0; i < 4000U; i++) {
		drive_tick(d, out);   /* START ramp, no Hall yet */
	}
	drive_set_erps(d, TARGET_ERPS_REF);
	for (uint32_t i = 0; i < 3000U; i++) {
		drive_tick(d, out);   /* RUN handover */
	}
}

int main(void)
{
	printf("FW-113.2 walk_assist_motor.c + walk_speed_controller.c - no hold timeout, reason bits\n");
	printf("  target 30 rpm -> %u ERPS, START 40, RUN 40, REACQUIRE 24, LIMIT 15\n",
		TARGET_ERPS_REF);

	walk_motor_release();

	/* ======================================================================
	 * S1: 60 s LONG HOLD must stay live - there is no wall-clock timeout.
	 * ====================================================================== */
	{
		drive_t d;
		walk_motor_output_t out;
		run_session(&d, &out);
		bool stall_false = true, limit_false = true, blocked_false = true;
		bool iq_live = false, reason_clean = true;
		for (uint32_t i = 0; i < LONG_HOLD_TICKS; i++) {
			drive_tick(&d, &out);
			if (is_state(&out, WA_STATE_STALL)) stall_false = false;
			if (is_state(&out, WA_STATE_LIMIT)) limit_false = false;
			if ((out.flags & WA_FLAG_BLOCKED) != 0) blocked_false = false;
			if (out.iq_target > 0) iq_live = true;
			if (!reason_has_only_known_bits(out.reason)) reason_clean = false;
			if ((out.reason & (WA_REASON_STALL | WA_REASON_LIMIT | WA_REASON_JAM)) != 0) {
				stall_false = false;
			}
		}
		CHECK(stall_false, "S1: 60 s hold never entered STALL (no hold timeout)");
		CHECK(limit_false, "S1: 60 s hold never entered LIMIT");
		CHECK(blocked_false, "S1: 60 s hold never set the BLOCKED latch");
		CHECK(iq_live, "S1: 60 s hold kept the motor driven (iq > 0)");
		CHECK(is_state(&out, WA_STATE_REGULATE), "S1: 60 s hold still in REGULATE at the end");
		CHECK(out.iq_target > 0, "S1: iq still positive at the end of the 60 s hold");
		CHECK(reason_clean, "S1: no unknown reason bit appeared during the hold");
	}

	/* ======================================================================
	 * S2: BUTTON RELEASE - request ends, iq zeroes, no latch; re-engage restarts.
	 * ====================================================================== */
	{
		drive_t d;
		walk_motor_output_t out;
		run_session(&d, &out);
		d.in.active = 0;
		drive_tick(&d, &out);
		CHECK(out.iq_target == 0, "S2: release zeroed iq");
		CHECK((out.flags & WA_FLAG_BLOCKED) == 0, "S2: release left no BLOCKED latch");
		CHECK(is_state(&out, WA_STATE_OFF), "S2: release put the module back to OFF");
		CHECK(reason_has_only_known_bits(out.reason), "S2: release reason is within the defined set");
		/* Re-engage: the same session resumes and drives again. */
		walk_motor_release();
		drive_reset(&d, true);
		for (uint32_t i = 0; i < 4000U; i++) drive_tick(&d, &out);
		drive_set_erps(&d, TARGET_ERPS_REF);
		bool restarted = false;
		for (uint32_t i = 0; i < 3000U; i++) {
			drive_tick(&d, &out);
			if (out.iq_target > 0) restarted = true;
		}
		CHECK(restarted, "S2: after a button release a re-engage drives again");
		CHECK(is_state(&out, WA_STATE_REGULATE), "S2: re-engage reached REGULATE");
	}

	/* ======================================================================
	 * S3: CAN RELEASE after a 60 s LONG HOLD - request drops, no STALL, re-engage works.
	 * ====================================================================== */
	{
		drive_t d;
		walk_motor_output_t out;
		run_session(&d, &out);
		for (uint32_t i = 0; i < LONG_HOLD_TICKS; i++) drive_tick(&d, &out);
		d.in.active = 0;   /* walk_can_request drops */
		drive_tick(&d, &out);
		CHECK(out.iq_target == 0, "S3: CAN release zeroed iq after the 60 s hold");
		CHECK(is_state(&out, WA_STATE_OFF), "S3: CAN release returned to OFF");
		CHECK((out.flags & WA_FLAG_BLOCKED) == 0, "S3: no BLOCKED latch after CAN release");
		walk_motor_release();
		drive_reset(&d, true);
		for (uint32_t i = 0; i < 4000U; i++) drive_tick(&d, &out);
		drive_set_erps(&d, TARGET_ERPS_REF);
		bool restarted = false;
		for (uint32_t i = 0; i < 3000U; i++) {
			drive_tick(&d, &out);
			if (out.iq_target > 0) restarted = true;
		}
		CHECK(restarted, "S3: after CAN release a re-engage drives again");
	}

	/* ======================================================================
	 * S4: BRAKE - iq 0 and WA_REASON_BRAKE; brake release resumes.
	 * ====================================================================== */
	{
		drive_t d;
		walk_motor_output_t out;
		run_session(&d, &out);
		d.in.brake = true;
		drive_tick(&d, &out);
		CHECK(out.iq_target == 0, "S4: brake zeroed iq");
		CHECK((out.reason & WA_REASON_BRAKE) != 0, "S4: reason carries WA_REASON_BRAKE");
		CHECK(reason_has_only_known_bits(out.reason), "S4: brake reason is within the defined set");
		d.in.brake = false;
		drive_tick(&d, &out);
		CHECK((out.reason & WA_REASON_BRAKE) == 0, "S4: reason cleared once the brake released");
	}

	/* ======================================================================
	 * S5: SPEED GATE - wheel at the ceiling -> iq 0 and WA_REASON_SPEED_GATE; resume below.
	 * ====================================================================== */
	{
		drive_t d;
		walk_motor_output_t out;
		run_session(&d, &out);
		d.in.wheel_speed_x100 = MAX_WHEEL_X100;
		drive_tick(&d, &out);
		CHECK(out.iq_target == 0, "S5: wheel at ceiling zeroed iq");
		CHECK((out.reason & WA_REASON_SPEED_GATE) != 0,
			"S5: reason carries WA_REASON_SPEED_GATE");
		CHECK(reason_has_only_known_bits(out.reason), "S5: gate reason is within the defined set");
		d.in.wheel_speed_x100 = SPEED_GATE_X100;
		drive_tick(&d, &out);
		CHECK((out.reason & WA_REASON_SPEED_GATE) == 0, "S5: reason cleared below the ceiling");
	}

	/* ======================================================================
	 * S6: FW-113.1 overspeed regression - TRUE iq 0, REGULATE, no false LIMIT/STALL.
	 * ====================================================================== */
	{
		drive_t d;
		walk_motor_output_t out;
		run_session(&d, &out);
		drive_set_erps(&d, 90U);
		bool descended = false;
		bool lim_false = true, stall_false = true;
		for (uint32_t i = 0; i < 20000U; i++) {
			drive_tick(&d, &out);
			if (out.iq_target == 0) descended = true;
			if (is_state(&out, WA_STATE_LIMIT)) lim_false = false;
			if (is_state(&out, WA_STATE_STALL)) stall_false = false;
		}
		CHECK(descended, "S6: overspeed descended to a TRUE iq_cmd == 0");
		CHECK(out.iq_target == 0, "S6: iq == 0 with measured > target");
		CHECK(is_state(&out, WA_STATE_REGULATE), "S6: still REGULATE after the descent");
		CHECK(lim_false, "S6: no false LIMIT on the overspeed descent");
		CHECK(stall_false, "S6: no false STALL on the overspeed descent");
	}

	/* ======================================================================
	 * S7: LOAD COMP - below target the PI must still INCREASE Iq.
	 * ====================================================================== */
	{
		drive_t d;
		walk_motor_output_t out;
		run_session(&d, &out);
		drive_set_erps(&d, 90U);
		for (uint32_t i = 0; i < 20000U; i++) drive_tick(&d, &out);
		CHECK(out.iq_target == 0, "S7: settled at true 0 above target (setup)");
		drive_set_erps(&d, 10U);
		int32_t before = 0, after = 0;
		for (uint32_t i = 0; i < 4000U; i++) {
			drive_tick(&d, &out);
			if (i == 1999U) before = out.iq_target;
		}
		after = out.iq_target;
		CHECK(after > before, "S7: PI increased Iq below target (load comp regression)");
		CHECK(after > 0, "S7: below target the PI recovered a positive Iq");
		CHECK(after <= 40, "S7: load comp never exceeded RUN_MAX 40");
	}

	/* ======================================================================
	 * S8: REACQUIRE ceiling - Hall loss at low torque stays bounded at 24 Iq.
	 * ====================================================================== */
	{
		drive_t d;
		walk_motor_output_t out;
		run_session(&d, &out);
		d.in.motor_erps_age_ticks = 0xFFFFU;
		d.in.motor_hall_ticks = 0;
		d.period = 0;
		int32_t peak = 0;
		bool seen = false;
		for (uint32_t i = 0; i < 6000U; i++) {
			drive_tick(&d, &out);
			if (out.iq_target > peak) peak = out.iq_target;
			if ((out.flags & WA_FLAG_REACQUIRE) != 0) seen = true;
		}
		CHECK(seen, "S8: Hall loss entered the bounded reacquire");
		CHECK(peak <= 24, "S8: REACQUIRE ceiling stays 24 Iq");
		CHECK((out.reason & WA_REASON_HALL) != 0, "S8: reason carries WA_REASON_HALL in reacquire");
	}

	/* ======================================================================
	 * S9: LIMIT/JAM/STALL - a real sustained stall still latches, reason tracks it,
	 *     and only a complete release clears the latch.
	 * ====================================================================== */
	{
		drive_t d;
		walk_motor_output_t out;
		run_session(&d, &out);
		d.in.motor_erps_age_ticks = 0xFFFFU;
		d.in.motor_hall_ticks = 0;
		d.period = 0;
		d.iq_actual = 24;
		bool limit_seen = false, stall_seen = false;
		bool jam_in_reason = false, limit_in_reason = false, stall_in_reason = false;
		int32_t limit_cap_after = 0;
		for (uint32_t i = 0; i < 12000U; i++) {
			drive_tick(&d, &out);
			if (is_state(&out, WA_STATE_LIMIT)) {
				limit_seen = true;
				limit_cap_after = out.iq_cap;
			}
			if (is_state(&out, WA_STATE_STALL)) stall_seen = true;
			if ((out.reason & WA_REASON_JAM) != 0) jam_in_reason = true;
			if ((out.reason & WA_REASON_LIMIT) != 0) limit_in_reason = true;
			if ((out.reason & WA_REASON_STALL) != 0) stall_in_reason = true;
		}
		CHECK(limit_seen, "S9: sustained stall latched LIMIT");
		CHECK(limit_cap_after == 15, "S9: LIMIT ceiling stays 15 Iq");
		CHECK(stall_seen, "S9: sustained stall then latched STALL");
		CHECK((out.flags & WA_FLAG_BLOCKED) != 0, "S9: STALL latch set WA_FLAG_BLOCKED");
		CHECK(jam_in_reason, "S9: reason carried WA_REASON_JAM through the stall");
		CHECK(limit_in_reason, "S9: reason carried WA_REASON_LIMIT through the stall");
		CHECK(stall_in_reason, "S9: reason carried WA_REASON_STALL after the latch");
		/* The latch survives a plain active=0 (no full release)... */
		d.in.active = 0;
		drive_tick(&d, &out);
		CHECK((out.flags & WA_FLAG_BLOCKED) != 0,
			"S9: STALL latch survives a partial request end (no release call)");
		/* ...and a complete release clears it and drives again. */
		walk_motor_release();
		drive_reset(&d, true);
		for (uint32_t i = 0; i < 4000U; i++) drive_tick(&d, &out);
		drive_set_erps(&d, TARGET_ERPS_REF);
		bool restarted = false;
		for (uint32_t i = 0; i < 3000U; i++) {
			drive_tick(&d, &out);
			if (out.iq_target > 0) restarted = true;
		}
		CHECK(restarted, "S9: after a complete release the motor drives again");
		CHECK((out.flags & WA_FLAG_BLOCKED) == 0, "S9: complete release cleared the latch");
	}

	/* ======================================================================
	 * S10: REASON BITS - each real gate sets exactly its own bit; no unknown bits,
	 *      and specifically NO timeout bit exists anywhere.
	 * ====================================================================== */
	{
		drive_t d;
		walk_motor_output_t out;
		/* BRAKE bit (S4 re-checked compactly): */
		run_session(&d, &out);
		d.in.brake = true;
		drive_tick(&d, &out);
		CHECK((out.reason & WA_REASON_BRAKE) != 0, "S10: BRAKE sets WA_REASON_BRAKE only");
		CHECK((out.reason & (WA_REASON_JAM | WA_REASON_STALL | WA_REASON_LIMIT)) == 0,
			"S10: BRAKE does not set a stall-family bit");
		/* SPEED_GATE bit: */
		d.in.brake = false;
		d.in.wheel_speed_x100 = MAX_WHEEL_X100;
		drive_tick(&d, &out);
		CHECK((out.reason & WA_REASON_SPEED_GATE) != 0,
			"S10: wheel ceiling sets WA_REASON_SPEED_GATE");
		CHECK((out.reason & WA_REASON_BRAKE) == 0, "S10: gate and brake bits stay independent");
		/* HALL bit (no Hall, low torque): */
		d.in.wheel_speed_x100 = 0;
		drive_tick(&d, &out);
		d.in.motor_erps_age_ticks = 0xFFFFU;
		d.in.motor_hall_ticks = 0;
		d.period = 0;
		drive_tick(&d, &out);
		CHECK((out.reason & WA_REASON_HALL) != 0, "S10: no Hall sets WA_REASON_HALL");
		/* Never a bit outside the defined set in any of these states: */
		CHECK(reason_has_only_known_bits(out.reason),
			"S10: reason is within the defined set after HALL");
		/* No timeout bit exists: WA_REASON_* has no value outside the defined mask, so any
		 * reason produced by the module must be representable - and the long-hold run in S1
		 * already proved no time-derived bit ever appears. Static assertion of the contract: */
		CHECK((unsigned)(WA_REASON_NO_CAN_REQUEST | WA_REASON_BUTTON_RELEASED |
			WA_REASON_SPEED_GATE | WA_REASON_BRAKE | WA_REASON_ERROR |
			WA_REASON_HALL | WA_REASON_JAM | WA_REASON_STALL | WA_REASON_LIMIT) == KNOWN_REASON_MASK,
			"S10: the defined reason set matches the KNOWN_REASON_MASK exactly");
	}

	if (host_test_failures == 0) {
		printf("All FW-113.2 walk assist diagnostics checks passed.\n");
		return 0;
	}
	printf("\n%d FW-113.2 walk assist check(s) FAILED.\n", host_test_failures);
	return 1;
}
