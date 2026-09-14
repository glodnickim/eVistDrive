#ifndef ASSIST_PIPELINE_H_
#define ASSIST_PIPELINE_H_

#include <stdbool.h>
#include <stdint.h>

#include "ap2_estimators.h"
#include "ap2_limits.h"
#include "ap2_pas_state.h"
#include "ap2_profiles.h"
#include "ap2_rider_demand.h"
#include "fast_iq_slew.h"

/*
 * ASSIST PIPELINE V2 - the whole pedal-assist path, in one place.
 *
 *      TORQUE + PAS + CADENCE + SPEED + MOTOR STATE
 *              |
 *              v   INPUT VALIDATION            ap2_rider_demand.c
 *              v   TORQUE ZERO / NORMALIZATION ap2_rider_demand.c
 *              v   RIDER DEMAND                ap2_rider_demand.c
 *              v   PEDAL CYCLE: BASE + DYNAMIC ap2_rider_demand.c
 *              v   RIDER AGGRESSION + LOAD     ap2_estimators.c
 *              v   PAS / DIRECTION LIFECYCLE   ap2_pas_state.c
 *              v   PROFILE / AUTO              ap2_profiles.c
 *              v   ASSIST CHARACTERISTIC       ap2_profiles.c
 *              v   BASE + DYNAMIC COMPONENT    this file
 *              v   START / ATTACK / RELEASE    this file
 *              v   LIMITS                      ap2_limits.c
 *              |
 *              v
 *          final_iq_request  ->  the single 16 kHz final Iq owner  ->  FOC
 *
 * ONE PATH. There is exactly one place in this firmware where pedal assist becomes a current
 * request, and it is assist_pipeline_update(). No mode, boost, floor, latch, limiter or
 * adaptive feature may write Iq beside it or after it. Walk Assist and the position
 * calibration are separate OWNERS of the same single mailbox, selected before this function
 * is called - they are not a second assist path.
 *
 * THREE THINGS THAT ARE NOT THE SAME, AND ARE NOT MERGED INTO ONE PERCENTAGE:
 *
 *   ASSIST        the characteristic: rider effort -> motor demand   (profile + gain)
 *   POWER         the ceiling: how much the profile may spend        (max_power_w)
 *   ACCELERATION  the dynamics: how fast the demand may move         (attack / release)
 *
 * WHAT THE RIDER FEELS WHEN EACH TUNING PARAMETER GOES UP is documented next to it, in
 * inc/ap2_profiles.h and docs/ASSIST_PIPELINE_V2.md.
 */

typedef struct {
	/* --- rider sensors (already conditioned by their existing owners) --- */
	uint16_t torque_load_centikg;   /* torque_input.c: zeroed, calibrated, physical */
	bool torque_sensor_valid;
	uint8_t cadence_rpm;            /* cadence_filter.c: the one control cadence */
	uint32_t speed_x100;
	uint16_t motor_erps;

	/* --- PAS / direction facts (pas_sampler.c, pas_direction.c) --- */
	bool forward_valid;
	bool direction_inhibit;
	bool inhibit_is_reverse;
	bool real_stop;
	bool wheel_valid;
	bool pas_sensor_valid;
	uint8_t forward_steps;
	uint8_t required_steps;

	/* --- rider selection --- */
	uint8_t assist_level_index;

	/* --- safety --- */
	bool safety_cut;      /* brake / critical overtemperature / torque fault / calibration */
	bool service_cut;     /* the calibration subset of the line above, for the zero policy */

	/* --- electrical --- */
	uint32_t battery_voltage_mv;
	int32_t battery_current_ma;
	int32_t battery_current_max;
	int32_t u_abs;
	int32_t cal_i;
	int32_t level_iq_limit;
	int32_t phase_current_max;
	uint16_t voltage_raw;
	int16_t voltage_min_raw;
	int16_t controller_temperature_c;

	/* --- legal --- */
	uint16_t speed_limit_x100;
	bool legal_enabled;
	bool offroad;

	/* --- other rider inputs --- */
	int32_t throttle_iq;   /* already mapped from the ADC; 0 when absent or released */

	uint32_t elapsed_ticks;
} assist_pipeline_input_t;

/*
 * The complete command for the single final Iq owner. The pipeline decides it; the caller
 * only publishes it. Keeping the decision here is what makes "one path to Iq" checkable by
 * reading one function instead of auditing every writer in the firmware.
 */
typedef struct {
	int32_t final_iq_request;
	fis_mode_t slew_mode;
	uint16_t step_mag_8;
	uint32_t release_ticks_16k;
	fis_zero_policy_t zero_policy;
	/* The hard ceiling the single reference owner must clamp to - see fast_iq_slew_command_t. */
	int32_t iq_ceiling;
} assist_pipeline_command_t;

/*
 * OBSERVABILITY. Every stage of the pipeline is published here so the bike can be tuned from
 * a log instead of from guesswork. Nothing reads this to make a decision.
 */
typedef struct {
	/* inputs as the pipeline saw them */
	uint16_t torque_load_centikg;
	int32_t torque_normalized_permille;
	uint8_t cadence_rpm;
	uint8_t pas_state;              /* ap2_pas_state_t */

	/* demand model */
	int32_t rider_demand_permille;
	int32_t assist_base_permille;
	int32_t assist_dynamic_permille;
	uint16_t stroke_period_ms;

	/* estimators */
	int32_t rider_aggression_permille;
	int32_t load_state_permille;

	/* profile */
	uint8_t profile_id;
	int32_t auto_factor_permille;
	uint16_t assist_gain_pct;
	uint16_t attack_ms;
	uint16_t release_ms;
	uint16_t max_power_w;

	/* request */
	int32_t assist_response_permille;
	int32_t iq_request_before_limits;
	int32_t final_iq_request;

	/* the protection ceiling in force, after its own rate limit */
	int32_t iq_ceiling;

	/* limiter states */
	bool power_limited;
	bool battery_limited;
	bool phase_limited;
	bool voltage_limited;
	bool thermal_limited;
	bool speed_limited;

	/* lifecycle */
	bool assist_permitted;
	bool start_active;
	bool release_active;
	bool block_positive;
	/* The DIRECTION subset of block_positive: a reverse crank step or an illegal PAS
	 * transition. It is the case that removes the Iq reference itself, not only the request. */
	bool direction_block;
	/*
	 * A non-zero request was taken all the way to zero by the limiter chain. The single fact
	 * that separates "the rider was not asking" from "a limit took it away", which is the first
	 * question any no-assist investigation has to answer.
	 */
	bool limiter_zeroed;
	/*
	 * Increments once per engagement (each STOPPED/STOPPING -> FORWARD transition). A reader
	 * detects a new engagement by seeing this change, which is what anchors the episode timings
	 * in main.c - the 40 ms diagnostic frame period is far too slow to see the transition
	 * itself.
	 */
	uint16_t engage_seq;
	/* The start gate as it stood THIS tick, from its one owner rather than re-derived by a
	 * reader against a constant that may have drifted. */
	uint8_t required_steps;
	uint16_t engage_threshold_centikg;
	bool bike_rolling;

	/* rider power, for the existing support-ratio telemetry */
	uint16_t rider_power_w;
	uint16_t motor_power_w;
	uint16_t applied_support_ratio_pct;
} assist_pipeline_telemetry_t;

/*
 * WHY THE REQUEST IS WHAT IT IS, in one byte. Every bit is a fact the pipeline already
 * decided this tick - nothing here is re-derived by a reader, and nothing reads it to decide.
 * It exists because "the current is zero" on its own says nothing about who zeroed it, which
 * is what turns a regression hunt into guesswork.
 */
#define AP2_WHY_NOT_PERMITTED  0x01U  /* the PAS lifecycle is not in FORWARD */
#define AP2_WHY_BLOCKED        0x02U  /* reverse, illegal PAS, safety cut, or assist off */
#define AP2_WHY_NO_DEMAND      0x04U  /* permitted, but the rider is asking for nothing */
#define AP2_WHY_LIMITED        0x08U  /* a limiter stage is holding the request down */
#define AP2_WHY_ZEROED_BY_LIMIT 0x10U /* a positive request was taken all the way to zero */
#define AP2_WHY_START          0x20U  /* the start segment is in force */
#define AP2_WHY_RELEASE        0x40U  /* a release or safety release is running */
#define AP2_WHY_AUTO           0x80U  /* an adaptive profile is selected */

uint8_t assist_pipeline_reason_bits(void);

/*
 * The lifecycle and the profile in one byte: low nibble = ap2_pas_state_t, high nibble =
 * ap2_profile_id_t. Both are small closed enums, so one byte carries the whole answer to
 * "which profile was in force and where in the pedalling lifecycle was it".
 */
uint8_t assist_pipeline_state_byte(void);

void assist_pipeline_init(void);
void assist_pipeline_reset(void);

/*
 * One 4 kHz control tick of the whole pedal-assist path. Produces the final Iq request and
 * the trajectory command for the single 16 kHz owner.
 */
void assist_pipeline_update(const assist_pipeline_input_t *in, assist_pipeline_command_t *cmd);

const assist_pipeline_telemetry_t *assist_pipeline_telemetry(void);

/* The PAS lifecycle state, for diagnostics and for the legacy session-state byte. */
ap2_pas_state_t assist_pipeline_pas_state(void);

/* True while the battery-current limiter is holding the request down. */
bool assist_pipeline_battery_limited(void);

#endif /* ASSIST_PIPELINE_H_ */
