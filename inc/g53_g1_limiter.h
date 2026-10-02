#ifndef G53_G1_LIMITER_H
#define G53_G1_LIMITER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * TASK-EVD-TQ-06-G1 / ADR-013: producer of the G53 BDE8 gain g1 - the original battery-current
 * limiter (PI #1) of the G5300 image, ported bit-exact.
 *
 * Source of truth: integration/evidence/evd-tq/TQ-02B-N4 (TASK-EVD-TQ-02B-N4 = PASS). Every
 * function below mirrors one transcription in tools/tq02b_n4.py of that package, which was
 * differentially tested against the original machine code (100 000 vectors each, 0 mismatches):
 *
 *   g53_g1_configure()     0x0800C520  CFG1   parameter load + validity flag
 *   g53_g1_limit_update()  0x0800C5BA  LIM    10-ms limit: base * level % / 100, SOC knee, thermal knee
 *   g53_g1_step()          0x0800C6B4  PI1    1-ms ramped setpoint, PI #1, g1 = max(0, out)
 *   (internal)             0x08019ABC / 0x08019AB2  PI step / PI reset
 *
 * Field order and widths follow the original RAM block (N = 0x20000FF8, PI object at N+0x30) so
 * the golden-vector harness can compare byte images. Units: limit, feedback and knees are in the
 * domain of the measured battery current (0.01 A on the G5300 by inference, N4 R3); factors are
 * Q12 (0x1000 = 1.0); g1 is Q12.
 */

typedef struct {                /* PI object, G5300 0x08019ABC layout (base N+0x30) */
    uint16_t sp;                /* +0x00 setpoint                                  */
    uint16_t fb;                /* +0x02 feedback                                  */
    int16_t  kp;                /* +0x04                                           */
    int16_t  ki;                /* +0x06                                           */
    uint16_t p_offset;          /* +0x08 subtracted from the error for P only      */
    uint16_t reserved_0a;       /* +0x0A never written                             */
    int32_t  hi;                /* +0x0C clamp of I, P and P+I                     */
    int32_t  lo;                /* +0x10                                           */
    uint16_t out_hi;            /* +0x14 output clamp                              */
    uint16_t out_lo;            /* +0x16                                           */
    uint16_t err;               /* +0x18 last (error - p_offset)                   */
    uint16_t reserved_1a;       /* +0x1A                                           */
    uint32_t p_term;            /* +0x1C                                           */
    uint32_t i_term;            /* +0x20                                           */
    uint32_t sum;               /* +0x24                                           */
    uint16_t out;               /* +0x28                                           */
} g53_g1_pi_t;

typedef struct {                /* block N = 0x20000FF8                            */
    uint8_t  base_select;       /* +0x00 copy of M+0x2ED                           */
    uint16_t level_pct;         /* +0x02 H+0x5E / H+0x62                           */
    uint16_t soc_factor;        /* +0x04 G+0x652, Q12                              */
    uint16_t thermal_a;         /* +0x06 M+0x2C0, Q12                              */
    uint16_t thermal_b;         /* +0x08 M+0x2C2, Q12                              */
    uint16_t taper;             /* +0x0A M+0x2A4 copy, Q12                         */
    uint16_t base;              /* +0x0C limit - reduction                         */
    uint16_t base_alt;          /* +0x0E limit                                     */
    uint16_t knee_soc;          /* +0x10                                           */
    uint16_t knee_temp;         /* +0x12                                           */
    uint16_t step;              /* +0x14 setpoint rise per tick                    */
    uint16_t step_hi;           /* +0x16 loaded, not read by PI #1                 */
    uint16_t floor;             /* +0x18 setpoint floor                            */
    uint16_t base_used;         /* +0x1A                                           */
    uint16_t limit;             /* +0x1C final limit (PI #1 input)                 */
    uint16_t soc_excess;        /* +0x1E                                           */
    uint16_t limit_soc;         /* +0x20                                           */
    uint16_t temp_excess;       /* +0x22                                           */
    uint16_t limit_temp;        /* +0x24                                           */
    uint16_t temp_add;          /* +0x26                                           */
    uint16_t limit_final;       /* +0x28                                           */
    uint16_t target;            /* +0x2A limit * taper >> 12                       */
    uint16_t diff;              /* +0x2C target - ramped                           */
    uint16_t ramped;            /* +0x2E                                           */
    g53_g1_pi_t pi;             /* +0x30                                           */
    uint16_t g1;                /* +0x5C                                           */
    uint8_t  cfg_invalid;       /* +0x5E                                           */
} g53_g1_state_t;

typedef struct {                /* master-block fields read by CFG1 (M = 0x200039A4) */
    uint16_t step;              /* M+0xF6  */
    uint16_t step_hi;           /* M+0xF8  */
    uint16_t p_offset;          /* M+0xFA  */
    int16_t  kp;                /* M+0xFC  */
    int16_t  ki;                /* M+0xFE  */
    int16_t  out_min;           /* M+0x100 */
    int16_t  out_max;           /* M+0x102 */
    int16_t  limit;             /* M+0x104 */
    int16_t  knee_soc;          /* M+0x106 */
    int16_t  knee_temp;         /* M+0x108 */
    uint16_t floor;             /* M+0x1EE */
    uint16_t limit_reduction;   /* M+0x288 */
} g53_g1_config_t;

typedef struct {
    uint8_t  bde8_state;        /* Q+0x50: state 6 selects level_pct_state6           */
    uint16_t level_pct;         /* H+0x5E */
    uint16_t level_pct_state6;  /* H+0x62 */
    uint8_t  base_select;       /* M+0x2ED: 1 selects base_alt                        */
    uint16_t soc_factor;        /* G+0x652 */
    uint16_t thermal_a;         /* M+0x2C0 */
    uint16_t thermal_b;         /* M+0x2C2 */
} g53_g1_limit_input_t;

/* G5300 compiled defaults (Parameter3 + loader 0x0801B50C), N4 §2.2. */
#define G53_G1_DEFAULT_STEP      10
#define G53_G1_DEFAULT_P_OFFSET  500
#define G53_G1_DEFAULT_KP        2048
#define G53_G1_DEFAULT_KI        200
#define G53_G1_DEFAULT_OUT_MIN   (-4096)
#define G53_G1_DEFAULT_OUT_MAX   4096
#define G53_G1_DEFAULT_KNEE      500     /* P1[9] "max current on low charge" 5 A; P2[59] 5 A */
#define G53_G1_Q12_ONE           0x1000u

/*
 * M820 INPUT ADAPTER (TASK-EVD-TQ-06-G1 step 2, ADR-013 D3). NOT part of the G5300 port: the G5300
 * SOC estimator is not ported (N4 R1/R11). This maps M820's own SOC onto the LIM SOC-knee factor, Q12.
 *
 * SOC (owner decision): full limit above point 1 (Para1[10]), linear down to the knee at point 2
 * (Para1[11]), and the knee - a fixed 50 % of the limit (owner 2026-10-02, was 15 %) - below it.
 * Point 1 = 0 or 0xFF disables;
 * point 2 = 0, 0xFF or >= point 1 means "ramp all the way to 0 % SOC".
 * Temperature (owner, variant A after review S2-04): NOT through this limiter. The M820 Iq thermal
 * derate (ap2_limits stage 5, 75 -> 90 degC) stays on every path, because it limits the phase
 * current that heats the bridge; a battery-current limit barely acts at low speed. The LIM thermal
 * factors are therefore held at 1.0.
 */
#define G53_G1_SOC_KNEE_PCT        50
#define G53_G1_SOC_POINT_DISABLED  0xFFu
uint16_t g53_g1_soc_factor_m820(int32_t soc_x10, uint8_t point1_pct, uint8_t point2_pct);

/* Zero state, as after the original RAM clear. */
void g53_g1_reset(g53_g1_state_t *s);

/* CFG1. Returns the new value of the M+0x314 flag word (bit0 = configuration invalid). */
uint16_t g53_g1_configure(g53_g1_state_t *s, const g53_g1_config_t *c, uint16_t m314);

/* LIM. Returns the M+0x54 value (max(limit, 100)); level % == 0 also resets PI #1. */
uint16_t g53_g1_limit_update(g53_g1_state_t *s, const g53_g1_limit_input_t *in);

/* PI1. One G53 tick. `feedback` is the measured battery current as the u16 the original reads. */
uint16_t g53_g1_step(g53_g1_state_t *s, uint16_t taper_q12, uint16_t feedback);

#endif
