#ifndef G53_PORT_CHAIN_H
#define G53_PORT_CHAIN_H

#include <stdbool.h>
#include <stdint.h>
#include "g53_port_pas.h"

typedef struct {
    uint16_t x;
    uint16_t rider_input_native;
    g53_pas_output_t pas;
    uint8_t level;
    uint32_t diag_word;
    uint8_t external_inhibit;
    uint16_t g1_q12;
    uint16_t g2_q12;
    int16_t speed_native;
} g53_chain_input_t;

typedef struct {
    uint32_t logical_tick;
    int32_t fast_phase;
    int32_t supervisor_phase;
    int32_t phases_executed;
    int32_t x;
    int32_t fsm_state;
    int32_t fsm_output;
    int32_t fsm_t34;
    int32_t fsm_t36;
    int32_t fsm_t38;
    int32_t fsm_t3a;
    int32_t fsm_t3c;
    int32_t fsm_t40;
    int32_t fsm_t44;
    int32_t fsm_t46;
    int32_t fsm_t48;
    int32_t fsm_t4a;
    int32_t fsm_t4b;
    int32_t fsm_t4c;
    int32_t fsm_t4d;
    int32_t fsm_g1fe;
    int32_t fsm_g1ff;
    int32_t fsm_g200;
    int32_t fsm_g203;
    int32_t pas_direction;
    int32_t pas_direction_candidate;
    int32_t cadence;
    int32_t evidence;
    int32_t movement;
    int32_t base_timeout;
    int32_t full_timeout;
    int32_t max_timeout;
    int32_t no_transition;
    int32_t pas_code;
    int32_t pas_current_delta;
    int32_t pas_previous_delta;
    int32_t pas_transition_count;
    int32_t pas_elapsed;
    int32_t pas_span;
    int32_t pas_magnitude;
    int32_t pas_raw_cadence;
    int32_t pas_filter_accumulator;
    int32_t pas_filtered_cadence;
    int32_t pas_plausibility_budget;
    int32_t pas_plausibility_base;
    int32_t pas_plausibility_latch;
    int32_t pas_plausibility_previous_latch;
    int32_t pas_plausibility_flag;
    int32_t pas_anti_rock;
    int32_t d7ec_mode;
    int32_t d7ec_selected_current;
    int32_t d7ec_speed;
    int32_t d7ec_m50;
    int32_t d7ec_m52;
    int32_t d7ec_m29f;
    int32_t d7ec_base_timeout;
    int32_t d7ec_no_transition;
    int32_t d7ec_max_timeout;
    int32_t d7ec_envelope;
    int32_t d7ec_readiness;
    int32_t d7ec_rider_intermediate;
    int32_t d7ec_rider;
    int32_t d7ec_assist_ratio;
    int32_t d7ec_ratio;
    int32_t d7ec_taper;
    int32_t d7ec_requested;
    int32_t d7ec_lut;
    int32_t d7ec_converted;
    int32_t d7ec_hold;
    int32_t d7ec_accel_target;
    int32_t d7ec_accel_live;
    int32_t d7ec_accel_rise;
    int32_t d7ec_accel;
    int32_t d7ec_accel_window;
    int32_t d7ec_accel_counter;
    int32_t d7ec_history_delta;
    int32_t d7ec_history_a;
    int32_t d7ec_history_b;
    int32_t d7ec_history_selected;
    int32_t d7ec_external_guard;
    int32_t d7ec_d19;
    int32_t e1e8_state;
    int32_t e1e8_factor;
    int32_t e1e8_taper;
    int32_t e1e8_pi;
    int32_t e1e8_target;
    int32_t e1e8_output;
    int32_t e1e8_output_factor;
    int32_t e1e8_taper_factor;
    int32_t e1e8_latch;
    int32_t bde8_q50;
    int32_t bde8_q5a;
    int32_t bde8_q5c;
    int32_t bde8_mode;
    int32_t bde8_demand;
    int32_t bde8_angle;
    int32_t bde8_g04;
    int32_t g1;
    int32_t g2;
    int32_t m298;
    int32_t m299;
    int32_t m28;
    int32_t m2a;
    int32_t m2c;
    int32_t m34;
    int32_t m40;
    int32_t m50;
    int32_t m52;
    int32_t m29f;
    int32_t m2a2;
    int32_t m2a4;
    int32_t m2a8;
    int32_t m2aa;
    int32_t m2ac;
    int32_t m2ae;
    int32_t m2b0;
    uint32_t diag;
    int32_t external_inhibit;
    int32_t fatal_fault;
    int32_t nonfatal_fault;
    uint32_t dropped_logical_ticks;
    uint16_t raw_pa6_adc;
    uint16_t rider_input_native;
} g53_port_trace_t;

typedef struct {
    uint16_t m2aa_native;
    bool normal_permission;
    g53_port_trace_t trace;
} g53_chain_output_t;

uint8_t g53_chain_level(uint8_t assist_level);
void g53_chain_reset(void);
void g53_chain_set_levels(const uint8_t accel[10], const uint16_t ratio[10]);
void g53_chain_set_auto(uint8_t enable, uint16_t scale, uint8_t rise_step);
void g53_chain_get_auto(uint8_t *enable, uint16_t *scale, uint8_t *rise_step);
uint16_t g53_chain_rise_step(uint8_t slot);
uint16_t g53_chain_ratio(uint8_t slot);
void g53_chain_step(const g53_chain_input_t *in, g53_chain_output_t *out);

/*
 * ASSIST-V3 read-only D7EC views and the G5300 static characteristic (ARCHITECTURE_V3.md sec. 5).
 * None of these functions writes the chain image. `slot` is a chain level (g53_chain_level()).
 */
/* Sport+ (AUTO) slot as D7EC computes it (D+200): 8 for five HMI levels. */
uint8_t g53_chain_sport_slot(void);
/* D+232 value D7EC loads for `slot`: the accel rise per 10 ms of the D7EC trajectory, incl. the
 * AUTO slot; 0 for slot 0 (as the transcription). Equals g53_chain_rise_step() for slot >= 1. */
uint16_t g53_chain_d7ec_rise(uint8_t slot);
/* D+62 (D3E) signed fall per 10 ms of the D7EC trajectory (-409 in the fixed profile). */
int16_t g53_chain_d7ec_fall(void);
/* Readiness: ready = evidence >= *evidence_min (D+38) || env >= *env_min (D+36). NULL-safe. */
void g53_chain_d7ec_readiness(uint16_t *env_min, uint8_t *evidence_min);
/* D+88 per-slot floor limit (source of the D+222 floor); 0 in this configuration (floor inert). */
uint16_t g53_chain_d7ec_floor_limit(uint8_t slot);

/* Caller-owned G7 ratio rise limiter state (the D+208 equivalent). */
typedef struct {
    uint16_t applied_ratio;   /* rate-limited ratio actually applied (D+208 semantics) */
    uint16_t acc_ms;          /* elapsed-time accumulator, 0..9 ms */
} g53_static_ratio_state_t;

typedef struct {
    uint16_t env;           /* D7EC envelope units (the D+170 domain), i.e. env_equiv */
    int16_t cadence;        /* fp: signed PAS cadence raw as D7EC reads it (D+12 = M+0x92) */
    uint16_t lut_cadence;   /* sb (D+8 = M+0x50): LUT input only when D+33 != 1 (D+33 == 1 here: fp) */
    uint16_t speed_native;  /* sl (D+6): used only by the D+34 speed taper (D+34 == 0 here: off) */
    uint8_t level;          /* chain slot, g53_chain_level(hmi); the Sport+ slot runs AUTO if D+120 */
} g53_static_input_t;

typedef struct {
    uint16_t c2;            /* D+194 */
    uint16_t c4;            /* D+196 */
    uint16_t ratio;         /* D+204: desired ratio (fixed / AUTO, after taper) */
    uint16_t applied_ratio; /* D+208: after the rise limiter */
    uint16_t d4;            /* D+212 */
    uint16_t d6;            /* D+214: rider_lut(cadence) */
    uint16_t conv;          /* D+218 (final, after the 1000 clamp) */
    uint16_t target;        /* D+224 */
    uint8_t auto_active;    /* AUTO interpolation path taken */
} g53_static_diag_t;

/* Ratio state as after g53_chain_reset(): applied ratio 0, accumulator 0. */
void g53_static_ratio_init(g53_static_ratio_state_t *state);
/*
 * g53_static_target(): the D7EC part after the envelope (c2 -> c4 -> ratio/AUTO -> G7 rise
 * limiter -> d4 -> rider_lut -> conv -> clamp -> target), as a pure function of `in` and the
 * caller-owned ratio state. Returns the target in the E2 domain, 0..40960 (D+224); `diag` may be
 * NULL. Reads the same configuration as the transcription (D+64 ratios, D+120 AUTO enable,
 * D+110/D+64 AUTO den, D+122 rise step, Sport+ slot, LUT, D+33 LUT select, D+34 taper) from the
 * live chain image, and writes nothing to it.
 *  - Readiness (G6) is not applied: c2 is always the `ready` value. Gate with
 *    g53_chain_d7ec_readiness() if needed.
 *  - The ratio limiter advances once per full 10 ms of accumulated `elapsed_ms` (as D7EC once per
 *    call every 10 ms), never per call. With no full 10 ms elapsed the held ratio is applied,
 *    clipped to the current desired ratio, and the state is not changed.
 *  - cadence < 0: returns 0 and consumes the elapsed time without a limiter step (D7EC returns
 *    before the limiter on a negative cadence).
 *  - D+222 floor: inert (D+88 table never written -> D+220 == 0 -> D+222 == 0), not modelled.
 *  - The D7EC stop zeroing (cur == 0 with cadence 0 / M29F) is not part of the static map.
 * Parity: tests/host/assist_v3_static_map_host.c (G1-STATIC, G1-STATIC-T, no-mutation).
 */
uint16_t g53_static_target(const g53_static_input_t *in, g53_static_ratio_state_t *state,
                           uint32_t elapsed_ms, g53_static_diag_t *diag);

#endif
