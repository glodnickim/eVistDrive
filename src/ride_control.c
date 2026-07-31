#include "ride_control.h"

#include "assist_dynamics.h"
#include "assist_limits.h"
#include "assist_modes.h"
#include "config.h"
#include "legacy_assist.h"
#include "motor_core.h"
#include "torque_input.h"
#include "tuning_config.h"

// FW-030: The ride core is the ONLY ride engine. Legacy engine selection removed.
// Walk Assist and Hall calibration still use legacy_assist_calculate() (the monolith
// serves them regardless of engine) — see ride_control_update below.

/*
 * FW-031: ride latch + current floor.
 *
 * Problem: assist was strictly proportional to pedal load, with no "engaged"
 * state. At light pedalling or in the crank dead-spots the torque collapses to
 * near zero, so Iq collapsed too and the motor pulsed on/off/on/off.
 *
 * Fix: once assist has *legally* started (forward pedalling with pedal load above
 * the existing "Minimum pedal load" threshold), latch it. While latched, hold a
 * small current floor so very light pedalling keeps the motor pulling instead of
 * dropping out. safety_cut (brake / backward / fault) and stopping the cranks drop
 * the latch immediately. Below the start threshold, not yet latched, assist stays
 * off (no "start from a touch"). Throttle is applied AFTER this block, so it is
 * unaffected and still works without pedalling.
 *
 * FW-032: run deadband / hold time / floor % are now global ride-feel tuning
 * (tuning_config, Canable Dynamics), persisted with the tuning blob. The START
 * threshold is the existing per-level "Minimum pedal load"
 * (without_rotation_threshold_mv) — no new variable for it.
 */
static bool assist_latched;
static int32_t assist_hold_ticks;

/*
 * FW-068: two additions to the START condition, both per level, both inert at 0.
 *
 * 1. start_load_reduction_mv lowers the engage threshold WHILE THE CRANKS ARE TURNING.
 *    The reason it matters: the latch drops the moment
 *    the cranks stop, so re-catching assist at 25 km/h used to demand exactly the same push
 *    as pulling away from a standstill. Standstill launches keep the full threshold - there
 *    is no crank movement there to vouch for the rider's intent.
 *
 * 2. start_rise_mv engages on a RISE of pedal load measured from the moment the crank
 *    condition was met, held for START_RISE_CONFIRM_MS. This is immune to zero drift (the
 *    auto re-zero moves the absolute threshold around, see FW-058/059) and it is ordered:
 *    crank first, load rise second. A root strike while free-wheeling produces a spike, not
 *    a sustained rise that follows a run of forward PAS steps.
 *
 * The baseline is held at the MINIMUM seen inside the window. Tracking the latest value
 * instead would let a slow, steady push creep the baseline along with it, so the difference
 * never reaches the threshold and path B would never fire.
 */
#define START_RISE_CONFIRM_MS 40
#define START_RISE_CONFIRM_TICKS (START_RISE_CONFIRM_MS * 4)

/*
 * Minimum wheel speed (MS.Speedx100 scale) before start_load_reduction_mv may lower the
 * threshold at all. Crank movement alone is NOT enough: at a standstill the rider can turn
 * the cranks forward with the chain slack and no load, which is exactly the case the full
 * threshold exists to catch. The reduction is for RE-catching assist on a rolling bike.
 *
 * Not literally "> 0": the speed signal can glitch a single pulse at rest (see FW-036), and
 * one glitch must not unlock a lower engage threshold while the bike is standing still.
 * 1.0 km/h is the same "actually moving" bar TQ_RECAL_MOVING_X100 already uses.
 */
#define RIDE_START_REDUCTION_MIN_SPEED_X100 100

static uint16_t start_baseline_mv;
static int32_t start_window_ticks;
static int32_t start_rise_ticks;
static bool start_window_open;

// FW-037: soft-stop release time for safety cuts (brake / backward / overtemp / torque
// fault / torque-cal). Fades assist to 0 over this ramp instead of a hard bridge cut.
// (Renumbered from FW-036 to avoid clashing with the speed-glitch card FW-036.)
#define RIDE_SAFETY_RELEASE_MS 200

/*
 * FW-041: gear preload. Starting from standstill the drivetrain has backlash and
 * static friction: current builds up with the wheel still held, then the rotor breaks
 * away all at once and the gears slap ("wyrwanie" at engage). The Iq ramp controls
 * current, not speed, so it cannot smooth that breakaway on its own.
 *
 * Fix (simple variant): while the motor is standing still at a fresh start, cap the
 * assist target at a small preload level. The existing ramp walks up to that cap
 * gently, taking up the backlash quietly. As soon as the rotor actually moves — or the
 * timeout expires so a stiff drivetrain can never stall here — the cap is released and
 * the normal ramp continues from the current value (bumpless, no reset).
 */
#define PRELOAD_IQ_CAP        10   /* ~1 A phase at CAL_I=95: enough to take up backlash */
#define PRELOAD_ERPS_MOVING   3    /* rotor considered moving above this */
#define PRELOAD_TIMEOUT_TICKS (300 * 4)  /* 300 ms @4 kHz: never hang if preload is too weak */

/*
 * FW-048: coast-out threshold. main.c flips the commutation angle to the six-step formula
 * around 5.5 erps (SIXSTEPTHRESHOLD), and that formula does not line up with the interpolated
 * one — the angle steps. Stop driving a bit above that point so no current is flowing when it
 * happens. Only ever applied while releasing; a start needs current down here.
 */
#define RIDE_COAST_RELEASE_ERPS 10

static bool preload_active;
static int32_t preload_ticks;
static bool walk_was_active;

void ride_control_init(void)
{
	assist_latched = false;
	assist_hold_ticks = 0;
	start_baseline_mv = 0;
	start_window_ticks = 0;
	start_rise_ticks = 0;
	start_window_open = false;
	preload_active = false;
	preload_ticks = 0;
	walk_was_active = false;
	assist_modes_reset();
}

void ride_control_update(const ride_control_input_t *input)
{
	if (input == 0) {
		motor_command_t stop_command = {
			.iq_target = 0,
			.id_target = 0,
			.enable = false,
			.emergency_stop = true
		};
		motor_core_set_command(&stop_command);
		return;
	}
	bool walk_release_cut = walk_was_active && !input->walk_active;
	walk_was_active = input->walk_active;

	/*
	 * Position-sensor calibration is a controller service mode, not an assist
	 * mode. Its second phase currently lives in the frozen Legacy monolith and
	 * must own Iq regardless of the persisted Legacy/ride-core selection. Bypass the
	 * ride-feel ramp as well: on the completion tick the calibration code sets
	 * Iq=0, disables PWM and stores the angle, and no stale ramp value may
	 * re-enable the bridge.
	 */
	if (input->position_calibration_active) {
		int32_t calibration_iq = legacy_assist_calculate();
		motor_command_t calibration_command = {
			.iq_target = calibration_iq,
			.id_target = input->current_id,
			.enable = true,
			.emergency_stop = false
		};
		motor_core_set_command(&calibration_command);
		return;
	}

	int32_t iq_target;
	int32_t dynamics_iq_scale = input->iq_scale;
	bool profile_pedaling_active = true;
	uint16_t profile_release_ms = 0;
	bool coast_release = false;   /* FW-048, see below */
	/*
	 * FW-069: Iq ramps are per level now, filled from the level config in the assist branch
	 * below. They stay 0 on the Walk Assist path ON PURPOSE: assist_dynamics_apply() returns
	 * before the ramp code whenever walk_active is set, because WA owns its complete Iq
	 * trajectory (FW-060/FW-067) and a second dynamic element behind its speed controller
	 * would only make the loop less stable. 0 also selects the compiled fallbacks, so a
	 * future path that does reach the ramp code can never get a zero-length ramp.
	 */
	uint16_t ramp_up_slow_ms = 0;
	uint16_t ramp_up_fast_ms = 0;
	uint16_t ramp_down_slow_ms = 0;
	uint16_t ramp_down_fast_ms = 0;
	if (input->walk_active) {
		/*
		 * The existing Walk controller remains the exclusive source until the
		 * dedicated ERPS-based module replaces it. It must not disappear when
		 * the developer selects the ride core engine.
		 */
		iq_target = legacy_assist_calculate();
	} else {
		const rider_input_t *rider = rider_input_get();
		const assist_level_config_t *level =
			assist_modes_get_default_level(input->assist_level_index);
		assist_mode_output_t mode_output;
		bool supported = assist_modes_calculate(
			rider,
			level,
			input->battery_voltage_mv,
			input->ride_core_iq_limit,
			&mode_output);
		dynamics_iq_scale = input->ride_core_iq_limit;
		iq_target = supported ? mode_output.iq_request : 0;
		profile_pedaling_active =
			rider->pedaling_active || mode_output.assist_without_rotation_active;
		profile_release_ms = level->release_ms;
		ramp_up_slow_ms = level->iq_rise_slow_ms;     //FW-069: per level
		ramp_up_fast_ms = level->iq_rise_fast_ms;
		ramp_down_slow_ms = level->iq_fall_slow_ms;
		ramp_down_fast_ms = level->iq_fall_fast_ms;

		// FW-031: ride latch + current floor (see header comment). Acts on the ASSIST
		// target only, before the throttle floor, so throttle keeps working without pedalling.
		{
			uint16_t torque_mv = rider->torque_assist_filtered;
			uint16_t start_deadband_mv = level->without_rotation_threshold_mv;
			uint16_t run_deadband_mv = tuning_config_run_deadband_mv();
			int32_t hold_ticks_full = tuning_config_assist_hold_ticks();
			// FW-034: assist level 0 = fully off. The latch must never arm or apply its
			// current floor there, otherwise the floor keeps the motor pulling at level 0.
			bool assist_off = (input->assist_level_index == 0);
			// FW-068: the crank condition itself (forward PAS steps, no reverse) already
			// lives in rider->pedaling_active — main.c builds it from the configurable
			// tuning_config_start_steps(). Here it gates BOTH engage paths below.
			bool crank_ok = !input->safety_cut && !assist_off && rider->pedaling_active;
			// FW-068 path A: absolute threshold, lowered only while the bike is genuinely
			// ROLLING and the cranks are turning. A standstill launch (and
			// assist-without-rotation, which runs at zero cadence) always faces the full
			// threshold - turning the cranks on a stationary bike proves nothing.
			bool bike_rolling =
				input->speed_x100 >= RIDE_START_REDUCTION_MIN_SPEED_X100;
			uint16_t engage_threshold_mv = start_deadband_mv;
			if (crank_ok && bike_rolling) {
				uint16_t reduction = level->start_load_reduction_mv;
				engage_threshold_mv = (reduction >= engage_threshold_mv) ?
					0U : (uint16_t)(engage_threshold_mv - reduction);
			}
			if (input->safety_cut || !rider->pedaling_active || assist_off) {
				// Brake / backward / fault, cranks stopped, or level 0 -> disarm immediately.
				assist_latched = false;
				assist_hold_ticks = 0;
			}
			// FW-068 path B: rise detector. The window opens on the rising edge of the
			// crank condition and closes on its loss or on timeout.
			if (!crank_ok || assist_latched) {
				start_window_open = false;
				start_window_ticks = 0;
				start_rise_ticks = 0;
			} else if (!start_window_open) {
				start_window_open = true;
				start_baseline_mv = torque_mv;
				start_window_ticks = (int32_t)level->start_rise_window_ms * 4;
				start_rise_ticks = 0;
			} else {
				if (torque_mv < start_baseline_mv) {
					start_baseline_mv = torque_mv; // hold the minimum, see header comment
				}
				if (start_window_ticks > 0) {
					start_window_ticks--;
				}
			}
			bool rise_engaged = false;
			if (start_window_open && level->start_rise_mv > 0U &&
				start_window_ticks > 0) {
				uint16_t rise = (torque_mv > start_baseline_mv) ?
					(uint16_t)(torque_mv - start_baseline_mv) : 0U;
				if (rise >= level->start_rise_mv) {
					if (start_rise_ticks < START_RISE_CONFIRM_TICKS) {
						start_rise_ticks++;
					}
					rise_engaged = (start_rise_ticks >= START_RISE_CONFIRM_TICKS);
				} else {
					// A single impact must not accumulate towards the next one.
					start_rise_ticks = 0;
				}
			}
			if (!assist_latched) {
				if (crank_ok &&
					(torque_mv >= engage_threshold_mv || rise_engaged)) {
					// Legal start: real forward pedal load -> arm.
					assist_latched = true;
					assist_hold_ticks = hold_ticks_full;
					start_window_open = false;
					// FW-033: seed the RUN estimator to the fast value at the
					// start instant so the launch magnitude is crisp (not rubbery),
					// then it smooths the following leg peaks.
					torque_input_seed_run(torque_mv);
				} else {
					// Not started yet: no assist from a light touch.
					iq_target = 0;
				}
			}
			if (assist_latched) {
				if (torque_mv >= run_deadband_mv) {
					assist_hold_ticks = hold_ticks_full;
				} else if (assist_hold_ticks > 0) {
					assist_hold_ticks--;
				} else {
					assist_latched = false;   // too long with almost no load -> release
				}
				if (assist_latched) {
					int32_t min_iq =
						(input->ride_core_iq_limit *
						tuning_config_min_iq_pct()) / 100;
					if (iq_target < min_iq) {
						iq_target = min_iq;
						// Active hold -> no release fade while latched.
						profile_pedaling_active = true;
					}
				}
			}
		}

		// FW-030: throttle ported from Legacy. Throttle acts as a current FLOOR (drives the
		// motor without pedalling), goes through the same limit/ramp chain as assist below,
		// and is cut by safety_cut (brake / backward / fault) — stricter than the old Legacy.
		if (!input->safety_cut && input->throttle_iq > iq_target) {
			iq_target = input->throttle_iq;
			profile_pedaling_active = true;   // active demand -> no release fade while on throttle
		}

		if (input->safety_cut) {   // FW-037: fade assist/throttle out via release ramp; only overcurrent hard-cuts (FOC.c)
				iq_target = 0;
				profile_pedaling_active = false;
				profile_release_ms = RIDE_SAFETY_RELEASE_MS;
				assist_latched = false;
				assist_hold_ticks = 0;
			}

			assist_limits_input_t limits_input = {
			.voltage_raw = input->voltage_raw,
			.voltage_min_raw = input->voltage_min_raw,
			.controller_temperature_c = input->controller_temperature_c,
			.cadence_filtered_x8 = input->cadence_filtered_x8,
			.speed_x100 = input->speed_x100,
			.speed_limit_x100 = input->speed_limit_x100,
			.legal_enabled = input->legal_enabled,
			.offroad = input->offroad,
			.walk_active = input->walk_active
		};
		iq_target = assist_limits_apply(iq_target, &limits_input);

		assist_smooth_start_input_t smooth_input = {
			.iq_target = iq_target,
			.measured_cadence_rpm = rider->cadence_rpm,
			.motor_erps = rider->motor_erps,
			.safety_cut = input->safety_cut
		};
		iq_target = assist_start_apply_smooth(
			&smooth_input,
			&level->smooth_start,
			0);

		// FW-041: gear preload — cap the target while the rotor is still standing, so the
		// ramp takes up backlash quietly instead of breaking away in one slap.
		if (input->safety_cut || iq_target <= 0) {
			preload_active = false;          // no demand / cut -> arm for the next fresh start
			preload_ticks = 0;
		} else if (rider->motor_erps > PRELOAD_ERPS_MOVING) {
			preload_active = false;          // rotor is turning: normal ramp owns it
			preload_ticks = 0;
		} else if (!preload_active && input->current_iq == 0) {
			preload_active = true;           // fresh start from standstill
			preload_ticks = 0;
		}
		if (preload_active) {
			if (++preload_ticks >= PRELOAD_TIMEOUT_TICKS) {
				preload_active = false;      // stiff drivetrain: release the cap, never hang
			} else if (iq_target > PRELOAD_IQ_CAP) {
				iq_target = PRELOAD_IQ_CAP;
			}
		}

		/*
		 * FW-048: below ~5.5 erps main.c switches the commutation angle from the interpolated
		 * FOC formula to the fixed six-step one, and the two do not agree — the angle jumps.
		 * Any current still flowing then jumps with it, which is the clunk heard exactly at
		 * standstill (and why stretching the release ramp never helped: it is a step, not a
		 * slope). So stop feeding current before that point and let the motor coast out on
		 * its own inertia. `iq_target == 0` restricts this to a release: a START also happens
		 * in this speed range and must keep its current.
		 */
		if (iq_target == 0 && rider->motor_erps < RIDE_COAST_RELEASE_ERPS) {
			coast_release = true;
		}
	}
	assist_dynamics_input_t dynamics_input = {
		.speed_x100 = input->speed_x100,
		.cadence_rpm = input->cadence_rpm,
		.iq_scale = dynamics_iq_scale,
		.phase_current_max = input->phase_current_max,
		.walk_active = input->walk_active,
		.immediate_cut = walk_release_cut,
		.safety_cut = input->safety_cut,
		.profile_pedaling_active = profile_pedaling_active,
		.profile_release_ms = profile_release_ms,
		.ramp_up_slow_ms = ramp_up_slow_ms,       //FW-069: per level
		.ramp_up_fast_ms = ramp_up_fast_ms,
		.ramp_down_slow_ms = ramp_down_slow_ms,
		.ramp_down_fast_ms = ramp_down_fast_ms,
		.coast_release = coast_release   //FW-048
	};
	int32_t iq_reference = assist_dynamics_apply(
		iq_target,
		input->current_iq,
		&dynamics_input);
	motor_command_t command = {
		.iq_target = iq_reference,
		.id_target = input->current_id,
		.enable = true,
		.emergency_stop = false
	};
	motor_core_set_command(&command);
}

// FW-030: single engine. Kept as a stub because telemetry (FW-028, 0x6029) still
// references it; always reports the ride core.
ride_engine_t ride_control_get_engine(void)
{
	return RIDE_ENGINE_CORE;
}
