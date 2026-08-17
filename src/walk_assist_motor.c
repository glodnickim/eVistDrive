#include "walk_assist_motor.h"

#include "walk_speed_controller.h"

/*
 * Absolute EBICS Iq units (CAL_I = 95 mA), independent of the configured phase
 * current. This ceiling preserves the safe WA scale used before the ride-current
 * ceiling was raised from 157 to 700.
 */
#define WA_MOTOR_IQ_ABS_MAX             157
#define WA_MOTOR_SAFE_LIMIT_IQ           15
#define WA_MOTOR_START_MAX_IQ            40

#define WA_MOTOR_TARGET_RPM_DEFAULT      20
#define WA_MOTOR_TARGET_RPM_MIN          20
#define WA_MOTOR_TARGET_RPM_MAX          60
#define WA_MOTOR_TARGET_ERPS_DEFAULT     27
#define WA_MOTOR_MAX_WHEEL_X100         700

/* M820: 80 electrical revolutions per crank revolution -> rpm * 4 / 3. */
#define WA_ERPS_PER_RPM_NUM               4
#define WA_ERPS_PER_RPM_DEN               3

#define WA_MOTOR_HALL_TIMER_HZ        500000UL
#define WA_MOTOR_HALL_AVG_SAMPLES        12
#define WA_MOTOR_ERPS_TIMEOUT_TICKS     800  /* 200 ms @ 4 kHz */

#define WA_MOTOR_JAM_GRACE_TICKS       6000  /* 1.5 s for energetic start */
#define WA_MOTOR_JAM_NO_HALL_TICKS      400  /* 100 ms */
#define WA_MOTOR_JAM_MIN_ERPS             2
#define WA_MOTOR_JAM_CMD_IQ              24
#define WA_MOTOR_JAM_ACTUAL_IQ           24
#define WA_MOTOR_JAM_NO_MOTION_TICKS   1200  /* 300 ms */
#define WA_MOTOR_JAM_PARTIAL_TICKS      800  /* 200 ms */
#define WA_MOTOR_JAM_TARGET_PCT          15
#define WA_MOTOR_LIMIT_TICKS            1600  /* 400 ms, then latched STALL */
#define WA_MOTOR_RECOVERY_TARGET_PCT      25
#define WA_MOTOR_RECOVERY_MIN_ERPS         4
#define WA_MOTOR_REACQUIRE_IQ              24
#define WA_MOTOR_REACQUIRE_TICKS         6000  /* 1.5 s gentle Hall reacquisition */
#define WA_MOTOR_COAST_RECOVERY_IQ          24
#define WA_MOTOR_COAST_RECOVERY_TICKS    16000  /* 4 s: 0.75 s ramp + 3.25 s at 24 Iq */
#define WA_MOTOR_HALL_LOSS_DRIVE_IQ        30
#define WA_MOTOR_RUN_MIN_IQ                   0
#define WA_MOTOR_RUN_MAX_IQ                  40
#define WA_MOTOR_COAST_EXIT_IQ                2

static walk_motor_state_t wa_state;
static walk_speed_controller_t wa_controller;
static walk_speed_controller_output_t wa_control_output;
static int32_t wa_iq_cmd;
static int32_t wa_erps_filtered;
static uint16_t wa_jam_ticks;
static uint16_t wa_limit_ticks;
static uint16_t wa_grace_ticks;
static uint16_t wa_last_erps_age_ticks;
static uint16_t wa_hall_periods[WA_MOTOR_HALL_AVG_SAMPLES];
static uint32_t wa_hall_period_sum;
static uint8_t wa_hall_period_index;
static uint8_t wa_hall_period_count;
static uint8_t wa_ignore_first_hall;
static uint8_t wa_hall_timeout_seen;
static uint8_t wa_had_motion;
static uint8_t wa_blocked;
static uint8_t wa_jam_active;
static uint8_t wa_reacquire_active;
static uint8_t wa_coast_expected;
static uint8_t wa_coast_recovery_active;
static uint16_t wa_reacquire_ticks;
static int32_t wa_iq_cap_last;

static int32_t abs32(int32_t value)
{
	return (value < 0) ? -value : value;
}

static uint16_t rpm_to_erps(uint16_t chainring_rpm)
{
	return (uint16_t)(((uint32_t)chainring_rpm * WA_ERPS_PER_RPM_NUM +
		(WA_ERPS_PER_RPM_DEN / 2U)) / WA_ERPS_PER_RPM_DEN);
}

static void hall_estimator_reset(void)
{
	wa_erps_filtered = 0;
	wa_last_erps_age_ticks = 0xFFFFU;
	wa_hall_period_sum = 0;
	wa_hall_period_index = 0;
	wa_hall_period_count = 0;
	wa_ignore_first_hall = 1;
	wa_hall_timeout_seen = 0;
}

static bool hall_estimator_update(const walk_motor_input_t *input)
{
	bool hall_valid =
		input->motor_erps_age_ticks <= WA_MOTOR_ERPS_TIMEOUT_TICKS;
	bool hall_event = hall_valid &&
		input->motor_erps_age_ticks < wa_last_erps_age_ticks;

	if (!hall_valid && !wa_hall_timeout_seen) {
		wa_hall_period_sum = 0;
		wa_hall_period_index = 0;
		wa_hall_period_count = 0;
		wa_ignore_first_hall = 1;
		wa_hall_timeout_seen = 1;
		wa_erps_filtered = 0;
	}
	if (hall_event) {
		wa_hall_timeout_seen = 0;
		if (wa_ignore_first_hall) {
			wa_ignore_first_hall = 0;
		} else if (input->motor_hall_ticks > 0U) {
			if (wa_hall_period_count < WA_MOTOR_HALL_AVG_SAMPLES) {
				wa_hall_periods[wa_hall_period_index] =
					input->motor_hall_ticks;
				wa_hall_period_sum += input->motor_hall_ticks;
				wa_hall_period_count++;
			} else {
				wa_hall_period_sum -=
					wa_hall_periods[wa_hall_period_index];
				wa_hall_periods[wa_hall_period_index] =
					input->motor_hall_ticks;
				wa_hall_period_sum += input->motor_hall_ticks;
			}
			if (++wa_hall_period_index >= WA_MOTOR_HALL_AVG_SAMPLES) {
				wa_hall_period_index = 0;
			}
			if (wa_hall_period_sum > 0U) {
				uint32_t numerator = WA_MOTOR_HALL_TIMER_HZ *
					(uint32_t)wa_hall_period_count;
				uint32_t denominator = wa_hall_period_sum * 6U;
				wa_erps_filtered = (int32_t)(numerator / denominator);
			}
		}
	}
	wa_last_erps_age_ticks = input->motor_erps_age_ticks;
	if (!hall_valid) {
		wa_erps_filtered = 0;
	}
	return hall_valid;
}

void walk_motor_reset(void)
{
	wa_state = WA_STATE_OFF;
	wa_iq_cmd = 0;
	wa_jam_ticks = 0;
	wa_limit_ticks = 0;
	wa_grace_ticks = WA_MOTOR_JAM_GRACE_TICKS;
	wa_had_motion = 0;
	wa_jam_active = 0;
	wa_reacquire_active = 0;
	wa_coast_expected = 0;
	wa_coast_recovery_active = 0;
	wa_reacquire_ticks = 0;
	wa_iq_cap_last = 0;
	wa_control_output = (walk_speed_controller_output_t){0};
	walk_speed_controller_reset(&wa_controller);
	hall_estimator_reset();
	/* wa_blocked intentionally survives until the complete WA request release. */
}

void walk_motor_release(void)
{
	walk_motor_reset();
	wa_blocked = 0;
}

static void publish_output(walk_motor_output_t *output, uint16_t target_erps,
	bool hall_valid)
{
	if (output == 0) {
		return;
	}
	output->iq_target = wa_iq_cmd;
	output->state = (uint8_t)wa_state;
	output->target_erps = target_erps;
	output->measured_erps = (uint16_t)
		((wa_erps_filtered > 65535) ? 65535 :
		((wa_erps_filtered < 0) ? 0 : wa_erps_filtered));
	output->error_erps = wa_control_output.error_erps;
	output->iq_cap = wa_iq_cap_last;
	output->integral_iq = wa_control_output.integral_iq;
	output->startup_iq = wa_control_output.startup_iq;
	output->flags = (uint8_t)
		((hall_valid ? WA_FLAG_HALL_VALID : 0) |
		(wa_jam_active ? WA_FLAG_JAM : 0) |
		(wa_blocked ? WA_FLAG_BLOCKED : 0) |
		(wa_control_output.saturated ? WA_FLAG_SATURATED : 0) |
		(wa_control_output.startup_active ? WA_FLAG_START_ACTIVE : 0) |
		(wa_control_output.above_target ? WA_FLAG_ABOVE_TARGET : 0) |
		((wa_state == WA_STATE_LIMIT) ? WA_FLAG_LIMIT : 0) |
		(wa_reacquire_active ? WA_FLAG_REACQUIRE : 0));
	/* FW-113.2: module-level reason bits. Activation bits (NO_CAN_REQUEST /
	 * BUTTON_RELEASED) are owned by main.c and OR-ed into the same field. There is
	 * deliberately no wall-clock hold-timeout bit: only a real gate can stop WA.
	 */
	output->reason = (uint16_t)
		((hall_valid ? 0 : WA_REASON_HALL) |
		(wa_jam_active ? WA_REASON_JAM : 0) |
		(wa_blocked ? WA_REASON_STALL : 0) |
		((wa_state == WA_STATE_STALL) ? WA_REASON_STALL : 0) |
		((wa_state == WA_STATE_LIMIT) ? WA_REASON_LIMIT : 0));
}

static void enter_limit(void)
{
	wa_state = WA_STATE_LIMIT;
	wa_limit_ticks = 0;
	wa_jam_active = 1;
	wa_reacquire_active = 0;
	wa_coast_expected = 0;
	wa_coast_recovery_active = 0;
	wa_reacquire_ticks = 0;
}

int32_t walk_motor_update(const walk_motor_input_t *input,
	walk_motor_output_t *output)
{
	uint16_t target_erps = WA_MOTOR_TARGET_ERPS_DEFAULT;
	if (input != 0 &&
		input->target_chainring_rpm >= WA_MOTOR_TARGET_RPM_MIN &&
		input->target_chainring_rpm <= WA_MOTOR_TARGET_RPM_MAX) {
		target_erps = rpm_to_erps(input->target_chainring_rpm);
	}

	uint16_t max_wheel_x100 =
		(input != 0 && input->max_wheel_speed_x100 > 0U) ?
		input->max_wheel_speed_x100 : WA_MOTOR_MAX_WHEEL_X100;
	if (input == 0 || !input->active || input->brake || input->fault ||
		input->wheel_speed_x100 >= max_wheel_x100) {
		walk_motor_reset();
		publish_output(output, target_erps, false);
		/* FW-113.2: name which real gate stopped the drive. publish_output already set
		 * the HALL bit (no Hall was read); these are the input-level gates. */
		if (output != 0 && input != 0) {
			output->reason |= (uint16_t)
				((input->brake ? WA_REASON_BRAKE : 0) |
				(input->fault ? WA_REASON_ERROR : 0) |
				((input->wheel_speed_x100 >= max_wheel_x100) ?
					WA_REASON_SPEED_GATE : 0));
		}
		return 0;
	}
	if (wa_blocked) {
		wa_iq_cmd = 0;
		wa_state = WA_STATE_STALL;
		wa_jam_active = 1;
		publish_output(output, target_erps, false);
		return 0;
	}

	if (wa_state == WA_STATE_OFF) {
		wa_state = WA_STATE_REGULATE;
	}
	if (wa_grace_ticks > 0U) {
		wa_grace_ticks--;
	}

	bool hall_valid = hall_estimator_update(input);
	if (hall_valid && wa_erps_filtered >= WA_MOTOR_JAM_MIN_ERPS) {
		wa_had_motion = 1;
		if (wa_reacquire_active) {
			wa_reacquire_active = 0;
			wa_coast_recovery_active = 0;
			wa_coast_expected = 0;
			wa_reacquire_ticks = 0;
		}
	}

	if (wa_state == WA_STATE_REGULATE) {
		bool no_motion =
			input->motor_erps_age_ticks > WA_MOTOR_JAM_NO_HALL_TICKS ||
			wa_erps_filtered < WA_MOTOR_JAM_MIN_ERPS;
		bool partial_motion =
			target_erps > 0U &&
			wa_erps_filtered * 100 <
				(int32_t)target_erps * WA_MOTOR_JAM_TARGET_PCT;
		bool current_significant =
			wa_iq_cmd >= WA_MOTOR_JAM_CMD_IQ ||
			abs32(input->motor_iq_actual) >= WA_MOTOR_JAM_ACTUAL_IQ;
		bool drive_without_hall =
			wa_iq_cmd >= WA_MOTOR_HALL_LOSS_DRIVE_IQ ||
			abs32(input->motor_iq_actual) >= WA_MOTOR_HALL_LOSS_DRIVE_IQ;

		if (wa_had_motion && !hall_valid) {
			if (wa_coast_expected || wa_coast_recovery_active) {
				/*
				 * PI intentionally reached the 2 Iq keepalive after an overspeed. The motor can
				 * stop behind the freewheel and lose Hall even though the drivetrain
				 * is healthy. Resume only through the slow RUN slew, but allow it to
				 * reach the bounded recovery ceiling and remain there long enough to produce
				 * a Hall edge. This path never re-arms the 40 Iq START.
				 */
				wa_coast_recovery_active = 1;
				wa_reacquire_active = 1;
				if (wa_reacquire_ticks < 65000U) {
					wa_reacquire_ticks++;
				}
				if (wa_reacquire_ticks >=
					WA_MOTOR_COAST_RECOVERY_TICKS) {
					enter_limit();
				}
			} else if (wa_reacquire_active || !drive_without_hall) {
				/*
				 * A stop at low torque is not a Hall fault under drive.
				 * The gentle reacquire ceiling remains below the hard Hall-loss
				 * threshold. A bounded reacquire cannot classify its own command
				 * as an immediate fault, but it still times out into LIMIT/STALL
				 * if the rotor cannot move.
				 */
				wa_reacquire_active = 1;
				if (wa_reacquire_ticks < 65000U) {
					wa_reacquire_ticks++;
				}
				if (wa_reacquire_ticks >= WA_MOTOR_REACQUIRE_TICKS) {
					enter_limit();
				}
			} else if (drive_without_hall) {
				/* Missing Hall while torque is present is a real safety fault. */
				enter_limit();
			}
		} else if (wa_grace_ticks == 0U && current_significant &&
			(no_motion || partial_motion)) {
			if (wa_jam_ticks < 65000U) {
				wa_jam_ticks++;
			}
			wa_jam_active = 1;
			uint16_t threshold = no_motion ?
				WA_MOTOR_JAM_NO_MOTION_TICKS :
				WA_MOTOR_JAM_PARTIAL_TICKS;
			if (wa_jam_ticks >= threshold) {
				enter_limit();
			}
		} else {
			wa_jam_ticks = 0;
			wa_jam_active = 0;
		}
	}

	if (wa_state == WA_STATE_LIMIT) {
		int32_t recovery_erps =
			((int32_t)target_erps * WA_MOTOR_RECOVERY_TARGET_PCT) / 100;
		if (recovery_erps < WA_MOTOR_RECOVERY_MIN_ERPS) {
			recovery_erps = WA_MOTOR_RECOVERY_MIN_ERPS;
		}
		if (hall_valid && wa_erps_filtered >= recovery_erps) {
			wa_state = WA_STATE_REGULATE;
			wa_limit_ticks = 0;
			wa_jam_ticks = 0;
			wa_jam_active = 0;
			wa_grace_ticks = WA_MOTOR_JAM_PARTIAL_TICKS;
		} else if (++wa_limit_ticks >= WA_MOTOR_LIMIT_TICKS) {
			wa_blocked = 1;
			wa_state = WA_STATE_STALL;
			wa_iq_cmd = 0;
			wa_iq_cap_last = 0;
			walk_speed_controller_reset(&wa_controller);
			wa_control_output = (walk_speed_controller_output_t){0};
			publish_output(output, target_erps, hall_valid);
			return 0;
		}
	}

	wa_iq_cap_last = (wa_state == WA_STATE_LIMIT) ?
		WA_MOTOR_SAFE_LIMIT_IQ :
		(wa_coast_recovery_active ? WA_MOTOR_COAST_RECOVERY_IQ :
		(wa_reacquire_active ? WA_MOTOR_REACQUIRE_IQ :
		(!wa_controller.startup_complete ? WA_MOTOR_START_MAX_IQ :
		WA_MOTOR_IQ_ABS_MAX)));
	walk_speed_controller_input_t control_input = {
		.target_erps = target_erps,
		.measured_erps = (uint16_t)
			((wa_erps_filtered > 65535) ? 65535 : wa_erps_filtered),
		.hall_valid = hall_valid,
		.reacquire = wa_reacquire_active != 0U,
		.iq_ceiling = wa_iq_cap_last,
		.run_iq_min = (wa_state == WA_STATE_REGULATE &&
			wa_controller.startup_complete && !wa_reacquire_active) ?
			WA_MOTOR_RUN_MIN_IQ : 0,
		.run_iq_max = WA_MOTOR_RUN_MAX_IQ,
		.downstream_iq = input->motor_iq_reference
	};
	wa_iq_cmd = walk_speed_controller_update(
		&wa_controller, &control_input, &wa_control_output);
	if (wa_state == WA_STATE_REGULATE && hall_valid &&
		wa_controller.startup_complete && !wa_reacquire_active) {
		/*
		 * Remember the PI's intent before the slower output ramp reaches zero.
		 * Otherwise Hall can expire during an intentional deceleration and the
		 * event is indistinguishable from an unexpected sensor loss.
		 */
		if (wa_control_output.above_target &&
			wa_controller.desired_iq <= 0) {
			wa_coast_expected = 1;
		} else if (wa_controller.desired_iq > 0 &&
			wa_iq_cmd >= WA_MOTOR_COAST_EXIT_IQ) {
			wa_coast_expected = 0;
		}
	}
	publish_output(output, target_erps, hall_valid);
	return wa_iq_cmd;
}
