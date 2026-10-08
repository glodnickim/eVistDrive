#ifndef ASSIST_V3_H_
#define ASSIST_V3_H_

#include <stdbool.h>
#include <stdint.h>

#include "assist_motion.h"

/* Shared independent bounds: carry caps are strictly inside the pipeline backstop. */
#define ASSIST_V3_CARRY_HARD_MAX_MS 1200u
#define ASSIST_V3_CARRY_HARD_MAX_MM 1500u
#define ASSIST_V3_BACKSTOP_HARD_MAX_MS 1500u
#define ASSIST_V3_BACKSTOP_HARD_MAX_MM 2000u
_Static_assert(ASSIST_V3_CARRY_HARD_MAX_MS <= ASSIST_V3_BACKSTOP_HARD_MAX_MS,
               "carry time must fit inside pipeline backstop");
_Static_assert(ASSIST_V3_CARRY_HARD_MAX_MM <= ASSIST_V3_BACKSTOP_HARD_MAX_MM,
               "carry distance must fit inside pipeline backstop");

/*
 * Assist Behavior V3 - transient manager and THE trajectory (ARCHITECTURE_V3.md sections 5, 6;
 * V3-4 base assist, V3-5 transient manager, V3-6 trajectory).
 *
 * ONE DEMAND STATE. This module owns `y`, the single rate-limited V3 demand (Iq, Q8, pre-g1,
 * pre-limits). It is the only temporal shaper of the V3 demand (ARCHITECTURE_V3 2.1): it reads the
 * rider intent (assist_v3_intent.c), maps env_equiv through the G5300 static characteristic
 * (g53_static_target(), no envelope/history/accel state of the transcription is consumed), and
 * moves y toward that target at the legacy rates of the section 6.1 table.
 *
 * WHAT IT NEVER DOES (ARCHITECTURE_V3 3.1, 9): it writes no slew mode, ceiling, zero policy or
 * safety state, and nothing it outputs can bypass native_cut, owner arbitration, ap2_limits, the
 * iq_ceiling, g1, the fast slew, the standstill zero or the backstop - all of those are applied by
 * the pipeline AFTER this output. It holds no pointer into the pipeline and no copy of the
 * published value as a seed (y is never re-seeded from limiters, 6.1).
 *
 * Milestone C rules (no carry yet): rise = min(level D7EC rise, BDE8 50/ms); fall while pedalling
 * = R(Response); PEDAL_STOP load released = y zeroed at once when the stop is confirmed (G53 true-stop
 * or real_stop; baseline BDE8 zeroes Q5C within ~12 ms of it), BDE8 3.5 Iq/ms before; load held = D3E
 * 0.455 Iq/ms once the crank stop is confirmed (G53 true-stop or native real_stop); reverse = y zeroed
 * at once (baseline BDE8 zeroes Q5C within 3 logical ms; the fast slew shapes the published Iq).
 *
 * Integer only, deterministic, no malloc, no float. One static instance. Context: the 4 kHz
 * foreground (assist_pipeline_update), like the rest of the pipeline.
 *
 * Units: P = phase_current_max. Iq = firmware Iq units. Q8: 256 = 1 Iq. E2 = D7EC E2 domain,
 * 0..40960 = 0..0.65*P Iq. Ticks = 4 kHz control periods; ms = 4 ticks.
 */

typedef struct {
	uint32_t elapsed_ticks;      /* 4 kHz periods since the last call (>= 1; 0 is read as 1)    */
	uint32_t now_tick;           /* 4 kHz clock of this call (same clock as crank_step_tick)    */
	uint16_t load_ctrl;          /* CLU, unfiltered (torque_input)                               */
	bool     torque_valid;
	int32_t  crank_steps;        /* crank_phase.c signed step count (differences only)          */
	uint32_t crank_step_tick;    /* tick of the last counted step                                */
	bool     pas_glitch;         /* INVALID jump or sampler ring overflow since the last call    */
	int16_t  cadence_rpm;        /* G53 PAS signed cadence (shadow chain; D7EC `fp`)             */
	uint16_t lut_cadence;        /* D7EC `sb` (trace d7ec_m50); inert while D+33 selects fp     */
	uint16_t speed_native;       /* D7EC `sl` (trace d7ec_speed); inert while the D+34 taper is off */
	bool     g53_true_stop;      /* G53 PAS crank stopped (accessor)                             */
	bool     g53_reverse;        /* G53 PAS direction < 0 (observation only: an illegal PAS pattern flips it) */
	bool     direction_inhibit;  /* native observations                                          */
	bool     inhibit_is_reverse;
	bool     real_stop;
	uint32_t speed_x100;
	bool     wheel_valid;
	uint32_t wheel_pulse_tick;   /* tick of the last accepted wheel pulse (observation)          */
	uint16_t motor_erps;
	int32_t  iq_measured;        /* MS.i_q (observation)                                         */
	int32_t  last_published_iq;  /* pipeline ctx.last_final_iq (observation; never a seed)       */
	int32_t  phase_current_max;  /* P                                                            */
	uint16_t g1_q12;             /* observation: the pipeline applies g1, not this module        */
	uint8_t  level;              /* assist level index 0..5 (0 = assist off)                     */
	uint8_t  response_pct;       /* V3 Response of this level, 0..100 (assist_v3_config)         */
	uint8_t  carry_strength_pct;
	uint16_t carry_time_ms;
	uint16_t carry_distance_dm;
	uint32_t speed_est_x100;
	uint32_t distance_est_mm;
	int16_t  rel_accel_permille_s;
	uint8_t  motion_quality;
	bool     native_cut;
	bool     brake;              /* observation only: the cut is native_cut in the pipeline      */
	uint16_t eb74_zero;          /* EB74 zero from the shadow chain accessor (750 today)         */
	bool     eb74_armed;         /* EB74 startup window done + pedal seen unloaded (R1-#14)      */
	bool     engine_active;      /* telemetry only (assist_v3_config)                            */
	bool     engine_requested;   /* telemetry only                                               */
	motion_input_t motion;       /* SANITISED copy (assist_motion_sanitize), section 3.3         */
} assist_v3_input_t;

/* Which rule moved (or held) y on the last call. Telemetry only. */
typedef enum {
	ASSIST_V3_RATE_HOLD = 0,          /* y == target                                     */
	ASSIST_V3_RATE_RISE = 1,          /* min(level D7EC rise, BDE8 50/ms)                */
	ASSIST_V3_RATE_FALL_RESPONSE = 2, /* R(Response), pedalling                          */
	ASSIST_V3_RATE_STOP_RELEASED = 3, /* PEDAL_STOP, load released: 0 once confirmed, else BDE8 */
	ASSIST_V3_RATE_STOP_HELD = 4,     /* PEDAL_STOP, load held, stop confirmed: D3E      */
	ASSIST_V3_RATE_STOP_WAIT = 5,     /* PEDAL_STOP, load held, stop not yet confirmed   */
	ASSIST_V3_RATE_REVERSE = 6,       /* reverse: y zeroed at once (legacy BDE8 zero)    */
	ASSIST_V3_RATE_START_BLOCKED = 7, /* at 0: engage gate (EB74 armed/threshold/readiness) closed */
	ASSIST_V3_RATE_CARRY = 8,
	ASSIST_V3_RATE_CARRY_RELEASE = 9
} assist_v3_rate_mode_t;

/*
 * Telemetry: the ARCHITECTURE_V3 section 11 fields that exist in Milestone B/C. NEVER a control
 * input (the pipeline reads assist_v3_stop_target_zero(), not this). Milestone D carry fields
 * (carry_score, carry_state, carry_remaining_ms/cm), rel_accel and terrain_est do not exist yet;
 * backstop_iq and final_iq belong to the pipeline.
 */
typedef struct {
	uint16_t intent;             /* I, CLU                                               */
	uint16_t env_equiv;          /* EB74/D7EC envelope units                             */
	uint16_t kappa_q12;          /* load-domain envelope factor kL, Q12                  */
	uint16_t e_short;            /* CLU                                                  */
	uint16_t e_long;             /* CLU                                                  */
	uint16_t template_conf_q12;  /* 0..4096                                              */
	uint16_t expected_effort;    /* CLU                                                  */
	uint8_t  phase;              /* template-relative crank phase 0..95                  */
	uint8_t  release_class;      /* assist_v3_release_class_t                            */
	bool     phase_aligned;
	bool     template_mode;
	uint16_t base_target_e2;     /* g53_static_target() output, E2 domain                */
	uint16_t applied_ratio;      /* G7 ratio limiter state (D+208 equivalent)            */
	int32_t  base_target_iq;     /* static map target, Iq (before the start/stop rules)  */
	int32_t  target_iq;          /* target y moved toward this call, Iq                  */
	int32_t  v3_demand_iq;       /* the output: y, Iq (pre-g1, pre-limits)               */
	int16_t  cadence_rpm;        /* G53 PAS signed cadence used this call                */
	uint8_t  rate_mode;          /* assist_v3_rate_mode_t                                */
	uint8_t  response_pct;
	uint16_t carry_score_q12;
	uint16_t carry_remaining_ms;
	uint16_t carry_remaining_cm;
	uint32_t speed_est_x100;
	int16_t rel_accel_permille_s;
	uint8_t motion_quality;
	uint8_t carry_state;
	uint8_t carry_cancel_reason;
	bool     engaged;            /* y > 0 at the start of the call (selects EB74 820)    */
	bool     crank_stopped;      /* PEDAL_STOP or G53 true-stop or native real_stop      */
	bool     stop_target_zero;   /* the value assist_v3_stop_target_zero() returns       */
	bool     imu_valid;          /* sanitised motion input valid                         */
	bool     engine_active;
	bool     engine_requested;
	uint32_t updates;            /* calls since power-on                                 */
} assist_v3_telemetry_t;

/* Boot: prior template, trajectory at 0. Called once from assist_pipeline_init(). */
void assist_v3_power_on(void);

/* Pipeline owner change / pipeline reset (ARCHITECTURE_V3 2.2): trajectory to 0, ratio limiter to
 * its chain-reset state, intent reset with the learned template KEPT (it describes the rider, not
 * the ride). */
void assist_v3_reset(void);

/* One control tick. Returns the single V3 output: iq_demand >= 0, Iq, pre-g1, pre-limits. */
int32_t assist_v3_update(const assist_v3_input_t *in);

/*
 * The "V3 stop target == 0" term of the pipeline standstill predicate (ARCHITECTURE_V3 2.1, re-check
 * N2, D-019): the D7EC zero-reset equivalent. True when the rider's load is released (EB74 output
 * with V3's active threshold is 0), or when V3's own stop ramp has brought both its target and y to
 * 0. A HELD load at a stopped crank is NOT a zero target until the legacy D3E ramp has reached 0, so
 * the predicate reproduces the baseline ~1 s ramp instead of a cut. State after the last update.
 */
bool assist_v3_stop_target_zero(void);

/* y > 0 after the last update (the V3 "engaged" flag of ARCHITECTURE_V3 4.3 / 6.2). */
bool assist_v3_engaged(void);

const assist_v3_telemetry_t *assist_v3_telemetry(void);

/* Pure helpers, exported for the host tests. */
/* E2 (0..40960) -> Iq Q8: E2 * 0.65 * P / 40960 * 256 (ARCHITECTURE_V3 5), P clamped to 0..8000. */
uint32_t assist_v3_e2_to_iq_q8(uint16_t e2, int32_t phase_current_max);
/* R(Response): ms for a full-scale (0.65*P) fall. 0 % -> 600 ms, 100 % -> 150 ms, linear
 * (candidate, ARCHITECTURE_V3 6.1); values above 100 read as 100. */
uint16_t assist_v3_response_full_scale_ms(uint8_t response_pct);

#endif /* ASSIST_V3_H_ */
