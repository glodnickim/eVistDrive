/*
 * Assist Behavior V3 - Rider Intent V1 (ARCHITECTURE_V3.md section 4: V3-1 crank phase tracker,
 * V3-2 rider intent, V3-3 release classifier).
 *
 * Pure production module: integer only, deterministic, no malloc, no float, no hardware access,
 * no time source other than the inputs. One static instance (the firmware has one rider).
 *
 * Units: CLU = control load units (torque_input load_ctrl). step = one PAS quadrature transition =
 * 3.75 deg (96 per revolution). tick = 4 kHz control period. Q12: 4096 = 1.0.
 *
 * Call order:
 *   assist_v3_intent_power_on()   once at boot (prior template, confidence 0, unaligned)
 *   assist_v3_intent_reset()      on a pipeline owner change (keeps the learned template; the phase
 *                                 is marked unaligned and confidence is restored after a clear
 *                                 re-alignment, because steps may have been missed meanwhile)
 *   assist_v3_intent_update()     every control tick (elapsed_ticks >= 1)
 *
 * Telemetry accessors (assist_v3_intent_debug, assist_v3_intent_template) are never control inputs.
 */
#ifndef ASSIST_V3_INTENT_H
#define ASSIST_V3_INTENT_H

#include <stdbool.h>
#include <stdint.h>

/* Template bins per revolution. Candidates 24 (default) and 12 (ARCHITECTURE_V3 4.2). 32 is not
 * allowed without a learned latency offset (R1-#15). */
#ifndef ASSIST_V3_NB
#define ASSIST_V3_NB 24
#endif
#if (ASSIST_V3_NB != 24) && (ASSIST_V3_NB != 12)
#error "ASSIST_V3_NB must be 24 or 12"
#endif

#define ASSIST_V3_STEPS_PER_REV 96u
#define ASSIST_V3_STEPS_PER_BIN (ASSIST_V3_STEPS_PER_REV / ASSIST_V3_NB)
#define ASSIST_V3_Q12_ONE 4096u

typedef enum {
    ASSIST_V3_CLASS_NORMAL_PRESSURE = 0,
    ASSIST_V3_CLASS_PHASE_DIP = 1,
    ASSIST_V3_CLASS_ATTACK = 2,
    ASSIST_V3_CLASS_TRUE_RELEASE = 3,
    ASSIST_V3_CLASS_PEDAL_STOP = 4
} assist_v3_release_class_t;

typedef struct {
    uint32_t elapsed_ticks;    /* 4 kHz periods since the last call (>= 1; 0 is treated as 1)     */
    uint32_t now_tick;         /* 4 kHz timestamp of this call (wrap-safe differences only)       */
    uint16_t load_ctrl;        /* CLU, unfiltered (torque_input)                                  */
    bool     torque_valid;
    int32_t  crank_steps;      /* signed cumulative quadrature step count (wrap-safe differences) */
    uint32_t crank_step_tick;  /* 4 kHz timestamp of the last step                                */
    bool     pas_glitch;       /* INVALID jump or sampler ring overflow since the last call       */
    int16_t  cadence_rpm;      /* G53 PAS signed cadence (shadow chain)                           */
    bool     g53_true_stop;    /* G53 PAS true-stop                                               */
    bool     real_stop;        /* native real_stop                                                */
    bool     direction_inhibit;
    bool     inhibit_is_reverse;
    uint16_t eb74_zero;        /* EB74 zero from the shadow chain accessor (750 today)            */
    bool     eb74_armed;       /* EB74 armed (startup window done, pedal seen unloaded once)      */
    bool     v3_engaged;       /* V3 demand engaged (owned by the trajectory): selects the EB74
                                * threshold 820 (engaged) / zero + 245 (to engage)                */
} assist_v3_intent_in_t;

typedef struct {
    uint16_t intent;           /* I, physical rider effort, CLU (never rescaled)                  */
    uint16_t e_short;          /* expected-effort-normalised short window (template mode) or the
                                * 180 deg mean (fallback), CLU                                    */
    uint16_t e_long;           /* normal intent estimate (360 deg, 180 deg at high confidence)    */
    uint16_t env_equiv;        /* EB74/D7EC envelope units: EB74_active(kL * I)                   */
    uint16_t kl_q12;           /* load-domain envelope factor kL, Q12, 1.0..2.5                   */
    uint16_t confidence_q12;   /* template confidence 0..4096                                     */
    uint16_t expected_effort;  /* I * s[current bin], CLU                                         */
    uint16_t revolutions_learned;
    uint16_t steps_since_restart;
    uint8_t  release_class;    /* assist_v3_release_class_t                                       */
    uint8_t  phase;            /* template-relative crank phase 0..95                             */
    bool     phase_aligned;
    bool     template_mode;    /* true: template classifier; false: fallback classifier           */
    bool     kl_from_prior;    /* kL currently taken from the prior template                      */
    bool     engage_ok;        /* env_equiv > 0 with the active threshold and EB74 armed (6.2)    */
} assist_v3_intent_out_t;

/* Internal constants. Every value is a *candidate* for the simulation matrix
 * (SIMULATION_REPORT.md section 3) unless marked as a transcription constant. */
typedef struct {
    uint8_t  alpha_shift;           /* template learning rate 2^-n per learned revolution (1/8)   */
    uint8_t  alpha_shift_low_conf;  /* faster rate while confidence < c_min (1/4)                 */
    uint8_t  a_min_steps;           /* short window minimum, steps (8 = 30 deg)                    */
    uint8_t  w_max_steps;           /* window cap, steps (48 = 180 deg)                            */
    uint8_t  fb_rel_steps;          /* fallback release: max over the last 46 steps (172.5 deg; any
                                     * window >= 172.5 deg still holds a power-stroke peak)         */
    uint32_t s_min_q12;             /* short window: sum of s must reach this (8.0 mean steps)     */
    uint16_t s_strong_q12;          /* short window must contain a bin with s >= this (1.0)        */
    uint16_t r_att_q12;             /* ATTACK enter, rho > 1.4                                     */
    uint16_t r_att_fb_q12;          /* fallback ATTACK: 180 deg mean > 1.25 * the 360 deg mean (the
                                     * 360 deg window contains the 180 deg one, so a 2x step peaks
                                     * at 1.33 and 1.4 could never fire below a 2.33x step)        */
    uint16_t r_rel_q12;             /* TRUE_RELEASE enter, rho < 0.5                               */
    uint16_t exit_band_q12;         /* override exit, |rho - 1| < 0.2                              */
    uint16_t dip_q12;               /* PHASE_DIP report: obs < 0.5 * I                             */
    uint8_t  dwell_steps;           /* minimum dwell in ATTACK / TRUE_RELEASE (24 = 90 deg)        */
    uint16_t rel_min_clu;           /* no release decision below this normal intent                */
    uint16_t att_min_clu;           /* no attack decision below this short-window effort           */
    uint16_t agree_floor_clu;       /* both windows below this: they agree (release to zero)       */
    uint16_t band_min_rpm;          /* trusted cadence band for template mode (15..150)            */
    uint16_t band_max_rpm;
    uint8_t  t_stop_periods;        /* PEDAL_STOP after this many expected step periods (4)        */
    uint16_t t_stop_min_ticks;      /* clamp 60 ms                                                 */
    uint16_t t_stop_max_ticks;      /* clamp 400 ms                                                */
    uint16_t stable_q12;            /* learning gate: |I_rev - I_prev| <= 15 % of I_prev           */
    uint16_t learn_min_clu;         /* no learning / alignment below this revolution mean          */
    uint16_t c_min_q12;             /* template mode needs confidence >= 0.5                       */
    uint16_t c_high_q12;            /* I over 180 deg at confidence >= 0.75                        */
    uint16_t c_step_q12;            /* +0.25 per revolution with residual < res_good               */
    uint16_t c_fall_q12;            /* -0.125 per revolution with residual in between              */
    uint16_t c_mismatch_q12;        /* residual > res_mismatch: confidence capped at 0.25          */
    uint16_t c_stop_decay_q12;      /* -16/4096 per second while stopped                           */
    uint16_t res_good_q12;          /* 0.15 mean |obs/I - s| per bin                               */
    uint16_t res_mismatch_q12;      /* 0.35 (N5 mismatch threshold, matrix chooses)                */
    uint16_t align_contrast_q12;    /* re-alignment needs (Cmax - Cmin) >= 0.15 * Cmax             */
    uint16_t kl_min_q12;            /* kL clamp 1.0 .. 2.5                                         */
    uint16_t kl_max_q12;
    uint16_t kl_floor_clu;          /* I_rev below this: prior kL (300 CLU)                        */
    uint16_t kl_prior_ref_clu;      /* load at which the prior kL is evaluated (1000 CLU)          */
    uint16_t kl_recompute_q12;      /* recompute kL when cadence changes by > 10 %                 */
    uint16_t kl_cad_min_rpm;        /* recurrence cadence clamp (15..250 rpm; <= 400 steps)        */
    uint16_t kl_cad_max_rpm;
    uint16_t eb74_offset;           /* transcription constant: source = 750 + L*2450/6000          */
    uint16_t eb74_gain_num;         /* transcription constant 2450                                 */
    uint16_t eb74_gain_den;         /* transcription constant 6000                                 */
    uint16_t eb74_saturation;       /* transcription constant 3200                                 */
    uint16_t eb74_thr_engaged;      /* transcription constant 820                                  */
    uint16_t eb74_deadband;         /* transcription constant 245 (threshold = zero + 245)         */
} assist_v3_intent_params_t;

/* Telemetry / test observability. Never a control input. */
typedef struct {
    uint32_t total_steps;
    uint32_t max_walk_steps;        /* longest per-step window walk seen                           */
    uint32_t max_steps_per_call;
    uint32_t last_kl_iterations;    /* D7EC recurrence iterations of the last kL evaluation        */
    uint32_t max_kl_iterations;
    uint32_t kl_recomputes;
    uint32_t alignments;
    uint32_t alignment_failures;
    uint32_t mismatch_drops;
    /* D-039 host op-count of one call: window-walk entries + steps + ring reads/MACs of the
     * deferred unit (alignment offsets, learning pass, kL recurrence iterations). Never a control
     * input; the worst case over a run is the host estimate of the per-call cost. */
    uint32_t work_this_call;
    uint32_t max_work_per_call;
    uint16_t last_rev_mean;         /* I_rev of the last completed revolution                      */
    uint16_t last_residual_q12;
    uint16_t env_ss;                /* steady-state envelope of the last kL evaluation             */
    uint8_t  tpl_offset;            /* raw phase -> template phase offset                          */
    uint8_t  class_state;           /* latched NORMAL / ATTACK / TRUE_RELEASE                      */
} assist_v3_intent_debug_t;

void assist_v3_intent_power_on(void);
void assist_v3_intent_reset(void);
void assist_v3_intent_update(const assist_v3_intent_in_t *in, assist_v3_intent_out_t *out);

const assist_v3_intent_params_t *assist_v3_intent_params(void);
const assist_v3_intent_debug_t *assist_v3_intent_debug(void);
/* Current template s[NB] (Q12, mean 4096) and the fixed prior. */
const uint16_t *assist_v3_intent_template(void);
const uint16_t *assist_v3_intent_prior(void);

/* Pure helpers (no state), exported for tests and for the static-map stage. */
/* EB74 transfer with an explicit threshold: max(0, min(750 + L*2450/6000, 3200) - thr). */
uint16_t assist_v3_eb74_active(uint32_t load_clu, uint16_t thr);
/* Load-domain envelope factor kL (Q12, clamped) of template s at revolution mean i_rev, cadence
 * cad_rpm and threshold thr, from the exact D7EC recurrence over one reconstructed revolution.
 * Returns 0 when the stroke never exceeds the threshold (kL undefined). env_ss/iterations may be
 * NULL. */
uint16_t assist_v3_intent_compute_kl(const uint16_t *s_q12, uint16_t i_rev, uint16_t cad_rpm,
                                     uint16_t thr, uint16_t *env_ss, uint32_t *iterations);

#endif /* ASSIST_V3_INTENT_H */
