#include "walk_speed_controller.h"

#define WA_SPEED_IQ_Q_SHIFT             8
#define WA_SPEED_CONTROL_DIV           20  /* 200 Hz PI inside the 4 kHz caller */

#define WA_SPEED_DEADBAND_ERPS           7  /* about 5.25 chainring rpm */
#define WA_SPEED_ERROR_CLAMP_ERPS       12  /* FW-064: soften catch-up after a coast */
#define WA_SPEED_KP_IQ_PER_ERPS          1
#define WA_SPEED_KI_STEP_Q               1
#define WA_SPEED_KI_UNWIND_STEP_Q        8  /* FW-064: unload stale I before the next catch-up */
#define WA_SPEED_INTEGRAL_MAX_IQ       121

#define WA_SPEED_PRELOAD_IQ             18
#define WA_SPEED_PRELOAD_TICKS         320  /* 80 ms */
#define WA_SPEED_BREAKAWAY_IQ           80
#define WA_SPEED_BREAKAWAY_TICKS      1520  /* full floor after 380 ms */
#define WA_SPEED_START_HANDOVER_IQ      36
#define WA_SPEED_START_SEED_MAX_IQ       8
#define WA_SPEED_START_FULL_PCT          8
#define WA_SPEED_START_DONE_PCT         30

#define WA_SPEED_START_RISE_STEP_Q       6  /* FW-066: 93.75 Iq/s, 80 Iq in about 0.85 s */
#define WA_SPEED_RUN_RISE_STEP_Q         1  /* 15.625 Iq/s near target/reacquire */
#define WA_SPEED_FALL_STEP_Q             2  /* 31.25 Iq/s normal RUN/coast decrease */
#define WA_SPEED_TRACK_MARGIN_IQ         6
#define WA_SPEED_HALL_LOSS_UNWIND_Q     64

static int32_t clamp32(int32_t value, int32_t low, int32_t high)
{
	if (value < low) {
		return low;
	}
	return (value > high) ? high : value;
}

static int32_t map_clamped(int32_t value, int32_t in_low, int32_t in_high,
	int32_t out_low, int32_t out_high)
{
	if (in_high <= in_low || value >= in_high) {
		return out_high;
	}
	if (value <= in_low) {
		return out_low;
	}
	return out_low +
		((value - in_low) * (out_high - out_low)) / (in_high - in_low);
}

static int16_t control_error(int32_t raw_error)
{
	int32_t error = 0;
	if (raw_error > WA_SPEED_DEADBAND_ERPS) {
		error = raw_error - WA_SPEED_DEADBAND_ERPS;
	} else if (raw_error < -WA_SPEED_DEADBAND_ERPS) {
		error = raw_error + WA_SPEED_DEADBAND_ERPS;
	}
	return (int16_t)clamp32(error,
		-WA_SPEED_ERROR_CLAMP_ERPS,
		WA_SPEED_ERROR_CLAMP_ERPS);
}

static int32_t startup_floor_iq(const walk_speed_controller_t *controller,
	const walk_speed_controller_input_t *input)
{
	if (controller->startup_complete) {
		return 0;
	}

	int32_t time_floor = WA_SPEED_PRELOAD_IQ;
	if (controller->session_ticks > WA_SPEED_PRELOAD_TICKS) {
		time_floor = map_clamped(
			(int32_t)controller->session_ticks,
			WA_SPEED_PRELOAD_TICKS,
			WA_SPEED_BREAKAWAY_TICKS,
			WA_SPEED_PRELOAD_IQ,
			WA_SPEED_BREAKAWAY_IQ);
	}

	int32_t speed_full = ((int32_t)input->target_erps *
		WA_SPEED_START_FULL_PCT) / 100;
	int32_t speed_done = ((int32_t)input->target_erps *
		WA_SPEED_START_DONE_PCT) / 100;
	if (speed_full < 2) {
		speed_full = 2;
	}
	if (speed_done <= speed_full) {
		speed_done = speed_full + 1;
	}
	if (!input->hall_valid || input->measured_erps <= speed_full) {
		return time_floor;
	}
	return map_clamped(input->measured_erps,
		speed_full, speed_done, time_floor, WA_SPEED_START_HANDOVER_IQ);
}

static void clear_output(walk_speed_controller_output_t *output)
{
	if (output == 0) {
		return;
	}
	output->iq_target = 0;
	output->error_erps = 0;
	output->controlled_error_erps = 0;
	output->p_iq = 0;
	output->integral_iq = 0;
	output->startup_iq = 0;
	output->startup_active = false;
	output->above_target = false;
	output->coast_requested = false;
	output->saturated = false;
}

void walk_speed_controller_reset(walk_speed_controller_t *controller)
{
	if (controller == 0) {
		return;
	}
	controller->integral_q = 0;
	controller->iq_command_q = 0;
	controller->desired_iq = 0;
	controller->session_ticks = 0;
	controller->control_divider = 0;
	controller->startup_complete = false;
	controller->reacquire_active = false;
}

int32_t walk_speed_controller_update(
	walk_speed_controller_t *controller,
	const walk_speed_controller_input_t *input,
	walk_speed_controller_output_t *output)
{
	clear_output(output);
	if (controller == 0 || input == 0 || input->target_erps == 0 ||
		input->iq_ceiling <= 0) {
		if (controller != 0) {
			controller->iq_command_q = 0;
			controller->desired_iq = 0;
		}
		return 0;
	}

	int32_t ceiling = input->iq_ceiling;
	int32_t downstream = clamp32(input->downstream_iq, 0, ceiling);
	if (controller->session_ticks == 0) {
		/* Enter WA without stepping away from an already active motor command. */
		controller->iq_command_q = downstream << WA_SPEED_IQ_Q_SHIFT;
	}
	if (controller->session_ticks < 0xFFFFFFFFUL) {
		controller->session_ticks++;
	}
	bool reacquire_entered =
		input->reacquire && !controller->reacquire_active;
	controller->reacquire_active = input->reacquire;
	if (reacquire_entered) {
		/*
		 * This is not a second START. Drop stale I, run one PI step now and let
		 * the normal run slew apply a small bounded current until Hall returns.
		 */
		controller->integral_q = 0;
		controller->desired_iq = 0;
		controller->control_divider = WA_SPEED_CONTROL_DIV - 1;
	}

	int32_t current_iq =
		(controller->iq_command_q + (1 << (WA_SPEED_IQ_Q_SHIFT - 1))) >>
		WA_SPEED_IQ_Q_SHIFT;
	bool downstream_limited =
		downstream + WA_SPEED_TRACK_MARGIN_IQ < current_iq;
	if (downstream_limited) {
		/*
		 * Voltage/temperature limits are downstream of WA. Follow their actual
		 * output so current cannot jump when a temporary limit disappears.
		 */
		controller->iq_command_q =
			(downstream + WA_SPEED_TRACK_MARGIN_IQ) << WA_SPEED_IQ_Q_SHIFT;
		current_iq = downstream + WA_SPEED_TRACK_MARGIN_IQ;
	}

	int32_t raw_error =
		(int32_t)input->target_erps - (int32_t)input->measured_erps;
	int16_t effective_error = control_error(raw_error);
	int32_t p_iq = (int32_t)effective_error * WA_SPEED_KP_IQ_PER_ERPS;
	int32_t start_done_erps =
		((int32_t)input->target_erps * WA_SPEED_START_DONE_PCT) / 100;
	if (start_done_erps < 2) {
		start_done_erps = 2;
	}

	bool startup_just_completed = false;
	if (!controller->startup_complete && input->hall_valid &&
		input->measured_erps >= start_done_erps) {
		controller->startup_complete = true;
		startup_just_completed = true;
		/* Seed I for a continuous takeover from the start trajectory. */
		int32_t seed_iq = clamp32(current_iq - p_iq, 0,
			WA_SPEED_START_SEED_MAX_IQ);
		controller->integral_q = seed_iq << WA_SPEED_IQ_Q_SHIFT;
	}

	if (++controller->control_divider >= WA_SPEED_CONTROL_DIV) {
		controller->control_divider = 0;
		if (input->force_coast) {
			/* A hard RPM coast discards stale PI demand but not the one-shot START latch. */
			controller->integral_q = 0;
			controller->desired_iq = 0;
		} else if (!controller->startup_complete) {
			controller->integral_q = 0;
		} else if (!input->hall_valid) {
			if (!input->reacquire) {
				controller->integral_q -= WA_SPEED_HALL_LOSS_UNWIND_Q;
				if (controller->integral_q < 0) {
					controller->integral_q = 0;
				}
			}
		} else if (!startup_just_completed) {
			int32_t integral_iq =
				controller->integral_q >> WA_SPEED_IQ_Q_SHIFT;
			int32_t unsaturated_iq = p_iq + integral_iq;
			int32_t integration_ceiling = ceiling;
			if (controller->startup_complete &&
				input->run_iq_max > 0 &&
				input->run_iq_max < integration_ceiling) {
				integration_ceiling = input->run_iq_max;
			}
			bool high_saturated =
				unsaturated_iq >= integration_ceiling &&
				effective_error > 0;
			bool low_saturated =
				controller->integral_q == 0 && effective_error < 0;
			if (!high_saturated && !low_saturated) {
				int32_t ki_step = (effective_error < 0) ?
					WA_SPEED_KI_UNWIND_STEP_Q :
					WA_SPEED_KI_STEP_Q;
				controller->integral_q +=
					(int32_t)effective_error * ki_step;
			}
			controller->integral_q = clamp32(controller->integral_q, 0,
				WA_SPEED_INTEGRAL_MAX_IQ << WA_SPEED_IQ_Q_SHIFT);
		}

		int32_t integral_iq =
			controller->integral_q >> WA_SPEED_IQ_Q_SHIFT;
		if (input->hall_valid) {
			int32_t tracked_max = downstream +
				WA_SPEED_TRACK_MARGIN_IQ - p_iq;
			if (tracked_max < 0) {
				tracked_max = 0;
			}
			if (controller->integral_q >
				(tracked_max << WA_SPEED_IQ_Q_SHIFT) &&
				downstream_limited) {
				controller->integral_q =
					tracked_max << WA_SPEED_IQ_Q_SHIFT;
				integral_iq = tracked_max;
			}
			controller->desired_iq = p_iq + integral_iq;
		} else if (input->reacquire) {
			/*
			 * No speed feedback yet: use the caller's small, explicit
			 * reacquire ceiling. Normal P is deliberately softer in FW-064
			 * and must not accidentally reduce this bounded restart attempt.
			 */
			controller->desired_iq = ceiling;
		} else if (controller->startup_complete) {
			controller->desired_iq = 0;
		} else {
			controller->desired_iq = p_iq;
		}
	}

	if (input->force_coast) {
		/* Do not leave up to 5 ms of stale desired current between PI steps. */
		controller->integral_q = 0;
		controller->desired_iq = 0;
	}

	int32_t start_floor = input->force_coast ?
		0 : startup_floor_iq(controller, input);
	int32_t desired_before_limit = input->force_coast ?
		0 : controller->desired_iq;
	if (!input->force_coast && start_floor > desired_before_limit) {
		desired_before_limit = start_floor;
	}
	if (!input->force_coast && input->iq_floor > desired_before_limit) {
		desired_before_limit = input->iq_floor;
	}
	int32_t desired_ceiling = ceiling;
	if (controller->startup_complete && input->run_iq_max > 0 &&
		input->run_iq_max < desired_ceiling) {
		/*
		 * FW-066: RUN current is a soft ceiling. A start that handed over
		 * above it must descend through the normal fall slew, never hard-clamp.
		 */
		desired_ceiling = input->run_iq_max;
	}
	int32_t desired = clamp32(
		desired_before_limit, 0, desired_ceiling);

	int32_t target_q = desired << WA_SPEED_IQ_Q_SHIFT;
	int32_t rise_step_q = WA_SPEED_RUN_RISE_STEP_Q;
	if (!controller->startup_complete) {
		/* Keep the proven one-shot 80 Iq breakaway capability. */
		rise_step_q = WA_SPEED_START_RISE_STEP_Q;
	}
	if (controller->iq_command_q < target_q) {
		int32_t delta = target_q - controller->iq_command_q;
		controller->iq_command_q +=
			(delta > rise_step_q) ? rise_step_q : delta;
	} else if (controller->iq_command_q > target_q) {
		int32_t delta = controller->iq_command_q - target_q;
		controller->iq_command_q -=
			(delta > WA_SPEED_FALL_STEP_Q) ? WA_SPEED_FALL_STEP_Q : delta;
	}

	/* LIMIT is a safety ceiling and may clamp immediately. */
	int32_t ceiling_q = ceiling << WA_SPEED_IQ_Q_SHIFT;
	if (controller->iq_command_q > ceiling_q) {
		controller->iq_command_q = ceiling_q;
	}
	if (controller->iq_command_q < 0) {
		controller->iq_command_q = 0;
	}
	current_iq =
		(controller->iq_command_q + (1 << (WA_SPEED_IQ_Q_SHIFT - 1))) >>
		WA_SPEED_IQ_Q_SHIFT;

	if (output != 0) {
		output->iq_target = current_iq;
		output->error_erps = (int16_t)clamp32(raw_error, -32768, 32767);
		output->controlled_error_erps = effective_error;
		output->p_iq = (int16_t)clamp32(p_iq, -32768, 32767);
		output->integral_iq = (int16_t)clamp32(
			controller->integral_q >> WA_SPEED_IQ_Q_SHIFT, 0, 32767);
		output->startup_iq =
			(int16_t)clamp32(start_floor, 0, 32767);
		output->startup_active = !controller->startup_complete;
		output->above_target = raw_error < 0;
		output->coast_requested = input->force_coast;
		output->saturated =
			desired_before_limit != desired || current_iq != desired;
	}
	return current_iq;
}
