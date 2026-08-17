#ifndef RIDER_INPUT_H_
#define RIDER_INPUT_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Normalized view of rider-related sensors in the existing EBICS native
 * units. Fixed-point throughout: the assist pipeline runs in the 4 kHz control
 * tick and must be deterministic, so no value here is floating point.
 */
typedef struct {
	uint16_t torque_raw_mv;
	int16_t torque_corrected_mv;
	uint16_t torque_filtered;
	uint16_t torque_assist_filtered;   /* fast 35 ms: start, safety, start-gate */
	uint16_t torque_run_filtered;      /* FW-033: slow RUN estimator: power/eMTB/torque, boost */
	uint16_t torque_load_centikg;
	/*
	 * FW-107: the CURRENT sample, after the existing assist deadband
	 * (TORQUE_ASSIST_DEADBAND_NATIVE) but BEFORE the 35 ms filter above -
	 * torque_snapshot_t.assist_delta_native, unchanged computation, just exposed here too.
	 * The 35 ms filter can stay positive for a little while after real pressure is gone (that
	 * is what a low-pass filter is FOR), which is exactly long enough to let a rearm that
	 * should not happen arm anyway on a stale reading. This field answers "is there real
	 * pressure RIGHT NOW", not "was there recently".
	 */
	uint16_t torque_assist_now_native;

	uint8_t cadence_rpm;
	uint32_t wheel_speed_x100;
	uint16_t motor_erps;
	uint16_t motor_voltage_utilization;

	bool pas_forward;
	bool pas_backward;
	bool pedaling_active;
	/* FW-083: raw consecutive-forward-step count and direction (the two
	 * components pedaling_active above is built from — cadence>0, not
	 * reversed, not idle-timed-out), exposed separately so ride_control can
	 * apply a lower step requirement specifically while the bike is already
	 * rolling, without re-deriving the direction check from scratch. */
	uint8_t crank_forward_steps;
	/*
	 * FW-109: the cadence/timeout half of "is the crank trustworthy right now" -
	 * (cadence>0||start_phase) && !real_stop. Deliberately no LONGER includes any
	 * Backwards_counter/reverse term (main.c's old forward_pedaling did) - direction safety is
	 * now pas_direction.c's automaton alone (see direction_inhibit_active below), so this field
	 * only ever answers "is the crank moving forward, or has it gone idle", never "is a
	 * reverse or an invalid transition in progress". Removing that term does not weaken the
	 * cold-start gate: fwd_run already resets on every reverse OR invalid step independently
	 * (src/pas_direction.c), which is what actually kept a reverse from starting a cold ride,
	 * and continues to.
	 */
	bool crank_direction_ok;
	/*
	 * FW-109: genuine stop / PAS timeout - pas_idle_ticks > pas_stop_timeout in main.c, an
	 * adaptive window (2x the last forward-transition gap, clamped to 200-500 ms @ 4 kHz) reset
	 * to zero by ANY valid PAS transition, forward OR reverse (not invalid - an illegal jump is
	 * not a real transition). Independent on purpose: it must never be reconstructed from
	 * crank_direction_ok, Backwards_counter or direction_inhibit_active, all of which can be
	 * true or false for reasons that have nothing to do with whether the crank has genuinely
	 * stopped turning.
	 */
	bool real_stop;
	/*
	 * FW-112.2: the wheel-rolling half of the true-stop decision - ride_wheel_valid()
	 * on the current control tick, i.e. (control_time_ticks - speed_last_tick) <
	 * SPEED_STOP_TICKS (an ACCEPTED wheel edge within the last 2.65 s; see inc/ride_wheel.h).
	 * When this is true the bike is demonstrably rolling, so a PAS timeout (real_stop) is
	 * a coast and NOT a true stop; when it goes false while real_stop holds, the session's
	 * qualified terminal ends the ride. The same production fact main.c already feeds the
	 * speed display, diag_moving, auto-off and the comm watchdog - not a new threshold.
	 */
	bool wheel_valid;
	/*
	 * FW-109 v2: pas_direction.c's direction safety automaton, this tick - see
	 * inc/pas_direction.h for the full state machine. direction_inhibit_active is a LEVEL (true
	 * for the whole duration a reverse OR invalid-transition hold lasts, whichever caused it -
	 * see pas_direction_last_inhibit_reason() for the diagnostic-only cause, not read by any
	 * decision downstream of this field); forward_confirmed_this_tick is an EDGE (true only on
	 * the tick confirmation completed) - never latched/historical, same discipline the FW-107
	 * field these two replace (pas_step_event) always had. (Renamed from reverse_inhibit_active:
	 * FW-109 v2 made an illegal PAS transition inhibit direction exactly like a reverse step
	 * does, so the field is no longer reverse-specific.)
	 */
	bool direction_inhibit_active;
	bool forward_confirmed_this_tick;
	/*
	 * The control tick THIS snapshot was built on (main.c's control_now, the same hardware
	 * tick source ride_episode.c already anchors its own timings to). FW-109 introduced this as
	 * the load-freshness anchor for the ride-session automaton; FW-112 v2 moved that guarantee
	 * into the RUN estimator's rolling-rearm recovery (inc/torque_input.h), which substitutes
	 * the current fast signal on the rearm tick and needs no sample_tick, and removed the
	 * session's consumption of this field. Retained - and still written by main.c and the host
	 * harnesses - purely as an observability anchor for the tick the snapshot was built on; no
	 * consumer reads it.
	 */
	uint32_t sample_tick;
	bool start_phase;
	bool torque_sensor_valid;
	bool pas_sensor_valid;
} rider_input_t;

void rider_input_update(const rider_input_t *sample);
const rider_input_t *rider_input_get(void);

#endif /* RIDER_INPUT_H_ */
