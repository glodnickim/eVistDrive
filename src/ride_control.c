#include "ride_control.h"

#include "assist_dynamics.h"
#include "assist_extended_boost.h"
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
 * threshold is the existing per-level "Minimum pedal load".
 */
static bool assist_latched;
static int32_t assist_hold_ticks;

/*
 * FW-068/077: riding_start_load_centikg is the direct engage threshold while the bike is
 *    rolling and the cranks are turning. All public start-load values now use kg.
 *    The reason it matters: the latch drops the moment
 *    the cranks stop, so re-catching assist at 25 km/h used to demand exactly the same push
 *    as pulling away from a standstill. Standstill launches keep the full threshold - there
 *    is no crank movement there to vouch for the rider's intent.
 */

/*
 * Minimum wheel speed (MS.Speedx100 scale) before the direct rolling threshold may replace
 * the standstill threshold. Crank movement alone is NOT enough: at a standstill the rider can turn
 * the cranks forward with the chain slack and no load, which is exactly the case the full
 * threshold exists to catch. The reduction is for RE-catching assist on a rolling bike.
 *
 * Not literally "> 0": the speed signal can glitch a single pulse at rest (see FW-036), and
 * one glitch must not unlock a lower engage threshold while the bike is standing still.
 * 1.0 km/h is the same "actually moving" bar TQ_RECAL_MOVING_X100 already uses.
 */
#define RIDE_START_REDUCTION_MIN_SPEED_X100 100

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
	preload_active = false;
	preload_ticks = 0;
	walk_was_active = false;
	assist_extended_boost_init();   //FW-084
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
		//FW-084: this path never reaches the Extended Boost block below, so clear it here
		//or an arming survives the whole service mode and fires on the way out.
		assist_extended_boost_reset(ASSIST_EXT_BOOST_CANCEL_CALIBRATION);
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
		assist_extended_boost_reset(ASSIST_EXT_BOOST_CANCEL_WALK);   //FW-084
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
			// FW-083: the start GATE compares against the fully raw kg reading (no
			// assist deadband, no 35 ms filter) — those exist to protect the smoothed
			// RUN estimator from sensor noise, which the start threshold (0.3-0.7 kg,
			// set by the rider) already has plenty of margin above. Using the
			// filtered/deadbanded signal here silently raised every configured start
			// threshold by ~0.4 kg. torque_load_centikg is the same raw value already
			// shown to the rider as their live pedal load.
			uint16_t torque_centikg = rider->torque_load_centikg;
			uint16_t standstill_threshold_centikg =
				level->minimum_pedal_load_centikg;
			uint16_t run_deadband_mv = tuning_config_run_deadband_mv();
			int32_t hold_ticks_full = tuning_config_assist_hold_ticks();
			// FW-034: assist level 0 = fully off. The latch must never arm or apply its
			// current floor there, otherwise the floor keeps the motor pulling at level 0.
			bool assist_off = (input->assist_level_index == 0);
			// FW-083: crank_ok, required_steps computed below (after bike_rolling).
			// Moved out of pedaling_active so only the step count can be relaxed.
			// Direction is still required in full; only the step count eases.
			// (eased by one step while rolling; never below 0; see below.)
			// FW-068: absolute threshold, lowered only while the bike is genuinely
			// ROLLING and the cranks are turning. A standstill launch (and
			// assist-without-rotation, which runs at zero cadence) always faces the full
			// threshold - turning the cranks on a stationary bike proves nothing.
			bool bike_rolling =
				input->speed_x100 >= RIDE_START_REDUCTION_MIN_SPEED_X100;
				uint8_t standstill_steps = tuning_config_start_steps();
				uint8_t required_steps = bike_rolling ?
					(standstill_steps > 0 ? standstill_steps - 1 : 0) : standstill_steps;
				bool crank_moving_enough = rider->crank_direction_ok &&
					rider->crank_forward_steps >= required_steps;
				bool crank_ok = !input->safety_cut && !assist_off && crank_moving_enough;
			uint16_t engage_threshold_centikg = standstill_threshold_centikg;
			if (crank_ok && bike_rolling) {
				engage_threshold_centikg = level->riding_start_load_centikg;
			}
			if (input->safety_cut || !crank_moving_enough || assist_off) {
				// Brake / backward / fault, cranks stopped, or level 0 -> disarm immediately.
				assist_latched = false;
				assist_hold_ticks = 0;
			}
			if (!assist_latched) {
				if (crank_ok && torque_centikg >= engage_threshold_centikg) {
					// Legal start: real forward pedal load -> arm.
					assist_latched = true;
					assist_hold_ticks = hold_ticks_full;
					// FW-033: seed the RUN estimator to the fast value at the
					// start instant so the launch magnitude is crisp (not rubbery),
					// then it smooths the following leg peaks.
					torque_input_seed_run(rider->torque_assist_filtered);
				} else {
					// Not started yet: no assist from a light touch.
					iq_target = 0;
				}
			}
			if (assist_latched) {
				if (rider->torque_assist_filtered >= run_deadband_mv) {
					assist_hold_ticks = hold_ticks_full;
				} else if (assist_hold_ticks > 0) {
					assist_hold_ticks--;
				} else {
					assist_latched = false;   // too long with almost no load -> release
				}
				if (assist_latched) {
					/* FW-091: round UP, so a small percentage of a small limit does not
					 * vanish in integer division and quietly leave no floor at all. */
					int32_t min_iq =
						((input->ride_core_iq_limit *
						tuning_config_min_iq_pct()) + 99) / 100;
					if (iq_target < min_iq) {
						iq_target = min_iq;
						// Active hold -> no release fade while latched.
						profile_pedaling_active = true;
					}
				}
			}
		}

		bool boost_active = false;   //FW-084: decides the limiter source explicitly, below
		/*
		 * FW-084: Extended Boost. It acts on the PEDAL-ONLY target, while iq_target still
		 * holds exactly that — before the throttle floor is merged in below. Putting it
		 * here is what makes "the boost can never copy throttle current" structural
		 * rather than a rule someone has to remember.
		 *
		 * It also sits BEFORE safety_cut and both limiter calls, so brake, reverse,
		 * speed, power, voltage and temperature all still have the last word.
		 */
		{
			assist_extended_boost_input_t boost_input = {
				/* The RAW pedalling state, not profile_pedaling_active: the latter
				 * is also raised by the latch floor and by throttle, and the boost
				 * must trigger on the cranks actually stopping. */
				.pedaling_active = rider->pedaling_active,
				/* The latch is the proof that assist started legally. The module
				 * only reads it while the cranks are still turning — see the note
				 * on the PAS STOP edge in assist_extended_boost.c. */
				.pedal_assist_latched = assist_latched,
				.motion_valid =
					input->speed_x100 >= EXT_BOOST_MIN_SPEED_X100 &&
					rider->motor_erps >= EXT_BOOST_MIN_MOTOR_ERPS,
				.safety_cut = input->safety_cut,
				.walk_active = false,                    /* not this branch */
				.position_calibration_active = false,    /* returned above */
				.torque_sensor_valid = rider->torque_sensor_valid,
				.pas_sensor_valid = rider->pas_sensor_valid,
				.crank_reverse = rider->pas_backward,
				.bank_index = assist_modes_get_active_bank(),
				.level_index = input->assist_level_index,
				.pedal_load_centikg = rider->torque_load_centikg,
				.ride_core_iq_limit = input->ride_core_iq_limit
			};
			assist_extended_boost_output_t boost_output;
			assist_extended_boost_update(
				&boost_input,
				&level->extended_boost,
				&boost_output);
			if (boost_output.active) {
				/*
				 * The boost REPLACES the mode's result, and the level's own
				 * ceiling — Maximum motor current and Maximum motor power — was
				 * applied inside assist_modes_calculate() to that result. Without
				 * re-applying it here a level limited to 20 % would be handed the
				 * full global limit by one hard pedal push.
				 */
				int32_t profile_ceiling = assist_modes_profile_iq_ceiling(
					level,
					rider,
					input->battery_voltage_mv,
					input->ride_core_iq_limit);
				iq_target = (boost_output.iq_target > profile_ceiling) ?
					profile_ceiling : boost_output.iq_target;
				boost_active = true;
			}
			if (boost_output.profile_hold_active) {
				/* Hold the profile "pedalling" while the boost runs, so the single
				 * release ramp starts ONCE, when the boost timer ends — not now. */
				profile_pedaling_active = true;
			}
		}

		if (input->safety_cut) {   // FW-037: fade assist/throttle out via release ramp; only overcurrent hard-cuts (FOC.c)
			iq_target = 0;
			profile_pedaling_active = false;
			profile_release_ms = RIDE_SAFETY_RELEASE_MS;
			assist_latched = false;
			assist_hold_ticks = 0;
		}

		/*
		 * FW-091: the two demands are limited SEPARATELY and only then combined.
		 *
		 * Throttle used to be merged into iq_target before the limiter (FW-030), which was
		 * fine while the limiter guessed the source from cadence. It stops being fine the
		 * moment the ride latch decides the classification: a latched rider would hand the
		 * throttle the full pedal speed limit, letting it drive past 7 km/h with no
		 * pedalling at all. Separate limiter calls make that structurally impossible —
		 * whatever the latch says, throttle current is only ever judged as non-pedal.
		 *
		 * Voltage and temperature limits are inside assist_limits_apply(), so both sources
		 * still get them, exactly as before.
		 */
		assist_limits_input_t limits_input = {
			.voltage_raw = input->voltage_raw,
			.voltage_min_raw = input->voltage_min_raw,
			.controller_temperature_c = input->controller_temperature_c,
			.source = ASSIST_LIMIT_SOURCE_PEDAL_CONFIRMED,
			.speed_x100 = input->speed_x100,
			.speed_limit_x100 = input->speed_limit_x100,
			.legal_enabled = input->legal_enabled,
			.offroad = input->offroad,
			.walk_active = input->walk_active
		};

		/*
		 * The latch IS the confirmation: it cannot arm without forward crank direction and
		 * the configured PAS step count (crank_moving_enough above), on top of the kg
		 * threshold, the assist level and the safety cuts. Nothing else needs asking.
		 *
		 * An earlier version also demanded !assist_without_rotation_active here, which was
		 * wrong twice over. That flag is raised purely on "cadence reads 0 and load exceeds
		 * the STANDSTILL threshold" (assist_modes.c:563) — it never checks whether the
		 * cranks are turning. So mid-ride, before the first cadence pulse, a firm push
		 * raised it even though the rider was demonstrably pedalling, dropping the request
		 * to the non-pedal limit and zeroing it at riding speed. Worse, it made the outcome
		 * depend on HOW HARD you pushed: a load between the riding and standstill thresholds
		 * left the flag clear and assist worked, while pushing harder set the flag and
		 * killed it. Harder pedalling giving less assist is exactly backwards.
		 */
		limits_input.source = assist_latched ?
			ASSIST_LIMIT_SOURCE_PEDAL_CONFIRMED :
			ASSIST_LIMIT_SOURCE_NON_PEDAL;
		/*
		 * FW-084 — DECIDED POLICY, not a side effect: while Extended Boost is running the
		 * request is classified NON_PEDAL.
		 *
		 * The cranks are stopped, so nothing is confirming pedalling in that moment; in
		 * legal mode this means the boost tapers from 5 km/h and is zero from 7 km/h. That
		 * is the conservative reading of a feature that keeps the motor pulling with the
		 * cranks stationary, and it is what the rider is told in the Canable help text.
		 *
		 * It is written out here rather than left to fall out of assist_latched == false on
		 * the PAS STOP edge: the latch dropping in that same tick made the classification an
		 * accident of ordering, which the next change to the latch could silently reverse.
		 *
		 * The alternative — earlier confirmed pedalling authorizing PEDAL_CONFIRMED for the
		 * duration of ACTIVE — is a product/legal decision, not a code cleanup. It must not
		 * be introduced by deleting these three lines.
		 */
		if (boost_active) {
			limits_input.source = ASSIST_LIMIT_SOURCE_NON_PEDAL;
		}
		int32_t pedal_iq = assist_limits_apply(iq_target, &limits_input);

		int32_t throttle_iq = 0;
		if (!input->safety_cut && input->throttle_iq > 0) {
			limits_input.source = ASSIST_LIMIT_SOURCE_NON_PEDAL;
			throttle_iq = assist_limits_apply(input->throttle_iq, &limits_input);
		}

		iq_target = (throttle_iq > pedal_iq) ? throttle_iq : pedal_iq;
		if (throttle_iq > 0 && throttle_iq >= pedal_iq) {
			profile_pedaling_active = true; // active demand -> no release fade while on throttle
		}

		assist_smooth_start_input_t smooth_input = {
			.iq_target = iq_target,
			.measured_cadence_rpm = rider->cadence_rpm,
			.motor_erps = rider->motor_erps,
			/* FW-092: one owner of "the bike is rolling" — the same constant the eased
			 * start-step count above uses. assist_start holds no threshold of its own. */
			.bike_rolling =
				input->speed_x100 >= RIDE_START_REDUCTION_MIN_SPEED_X100,
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
