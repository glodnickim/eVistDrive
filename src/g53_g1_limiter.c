#include "g53_g1_limiter.h"

#include <string.h>

/*
 * Bit-exact port of the G5300 PI #1 battery-current limiter. See inc/g53_g1_limiter.h.
 * Instruction addresses in the comments are those of the pinned G5300 D-1 image; the arithmetic
 * follows integration/evidence/evd-tq/TQ-02B-N4/tools/tq02b_n4.py line for line, including the
 * 32-bit wrap, the logical shifts and the 16-bit stores. No floating point, no division.
 */

static int32_t sxth(uint32_t v)
{
    return (int32_t)(int16_t)(uint16_t)v;
}

/* 0x08019AB2 */
static void pi_reset(g53_g1_pi_t *pi)
{
    pi->i_term = 0;
    pi->sum = 0;
    pi->out = 0;
}

/* 0x08019ABC: clamped PI, Q12 output */
static void pi_step(g53_g1_pi_t *pi)
{
    uint32_t ip = pi->i_term;
    uint32_t r3 = (uint32_t)pi->sp - (uint32_t)pi->fb;                 /* 08019AC4 */
    uint32_t r2;
    uint32_t p;
    int32_t out;

    ip = (uint32_t)(sxth((uint16_t)pi->ki) * sxth(r3)) + ip;          /* 08019ACC mla */
    pi->err = (uint16_t)r3;
    if ((int32_t)ip > pi->hi) {                                        /* 08019ADC */
        ip = (uint32_t)pi->hi;
    } else if ((int32_t)ip < pi->lo) {                                 /* 08019AE2 */
        ip = (uint32_t)pi->lo;
    }
    pi->i_term = ip;
    r2 = r3 - (uint32_t)pi->p_offset;                                  /* 08019AE8 */
    pi->err = (uint16_t)r2;
    p = (uint32_t)(sxth((uint16_t)pi->kp) * sxth(r2));                 /* 08019AF4 */
    if ((int32_t)p > pi->hi) {
        p = (uint32_t)pi->hi;
    } else if ((int32_t)p < pi->lo) {
        p = (uint32_t)pi->lo;
    }
    pi->p_term = p;
    r2 = p + ip;                                                       /* 08019B08 */
    if ((int32_t)r2 > pi->hi) {
        r2 = (uint32_t)pi->hi;
    } else if ((int32_t)r2 < pi->lo) {
        r2 = (uint32_t)pi->lo;
    }
    pi->sum = r2;
    pi->out = (uint16_t)(r2 >> 12);                                    /* 08019B1C lsrs; strh */
    out = sxth(r2 >> 12);                                              /* 08019B20 sbfx */
    if (out > sxth(pi->out_hi)) {
        pi->out = pi->out_hi;
    } else if (out < sxth(pi->out_lo)) {
        pi->out = pi->out_lo;
    }
}

/* ---- M820 input adapters (see the header) -------------------------------------------------- */
uint16_t g53_g1_soc_factor_m820(int32_t soc_x10, uint8_t point1_pct, uint8_t point2_pct)
{
    int32_t p2;
    if (point1_pct == 0u || point1_pct == G53_G1_SOC_POINT_DISABLED || point1_pct > 100u) {
        return (uint16_t)G53_G1_Q12_ONE;
    }
    p2 = (point2_pct == 0u || point2_pct == G53_G1_SOC_POINT_DISABLED || point2_pct >= point1_pct)
        ? 0 : (int32_t)point2_pct;
    if (soc_x10 >= (int32_t)point1_pct * 10) return (uint16_t)G53_G1_Q12_ONE;
    if (soc_x10 <= p2 * 10) return 0u;
    return (uint16_t)(((soc_x10 - p2 * 10) * (int32_t)G53_G1_Q12_ONE) / (((int32_t)point1_pct - p2) * 10));
}

void g53_g1_reset(g53_g1_state_t *s)
{
    if (s) {
        memset(s, 0, sizeof(*s));
    }
}

/* 0x0800C520 */
uint16_t g53_g1_configure(g53_g1_state_t *s, const g53_g1_config_t *c, uint16_t m314)
{
    const int32_t ip = c->out_max;
    const int32_t r3 = c->out_min;
    const int32_t r2 = c->limit;
    const int32_t r5 = c->knee_soc;
    const int32_t lr = c->knee_temp;
    const uint32_t base = (uint32_t)r2 - (uint32_t)c->limit_reduction;  /* 0800C56A */
    const int32_t r4 = sxth(base);

    s->step = c->step;
    s->step_hi = c->step_hi;
    s->pi.p_offset = c->p_offset;
    s->pi.kp = c->kp;
    s->pi.ki = c->ki;
    s->pi.hi = (int32_t)((uint32_t)ip << 12);
    s->pi.lo = (int32_t)((uint32_t)r3 << 12);
    s->pi.out_hi = (uint16_t)ip;
    s->pi.out_lo = (uint16_t)r3;
    s->base = (uint16_t)base;
    s->base_alt = (uint16_t)r2;
    s->knee_soc = (uint16_t)r5;
    s->knee_temp = (uint16_t)lr;
    s->floor = c->floor;
    /* 0800C582..C594 */
    if (r5 >= 1 && r4 > r5 && ip > r3 && !(r2 < r4) && r4 > lr) {
        s->cfg_invalid = 0;
        return (uint16_t)(m314 & ~1u);
    }
    s->cfg_invalid = 1;
    return (uint16_t)(m314 | 1u);
}

/* 0x0800C5BA */
uint16_t g53_g1_limit_update(g53_g1_state_t *s, const g53_g1_limit_input_t *in)
{
    const uint16_t pct = (in->bde8_state == 6) ? in->level_pct_state6 : in->level_pct;
    const int32_t base = sxth(in->base_select == 1 ? s->base_alt : s->base);
    uint32_t lim;
    uint32_t r4 = in->soc_factor;
    uint32_t r7;
    uint32_t r3;
    uint32_t r4o;

    s->level_pct = pct;
    s->base_select = in->base_select;
    s->soc_factor = in->soc_factor;
    s->thermal_a = in->thermal_a;
    s->thermal_b = in->thermal_b;
    /* 0800C626..C63A: base * % / 100 via umull 0x51EB851F, >>5 */
    lim = (uint32_t)(((uint64_t)((uint32_t)base * (uint32_t)pct) * 0x51EB851Fu) >> 37);
    s->base_used = (uint16_t)base;
    r7 = lim - (uint32_t)s->knee_soc;
    s->limit = (uint16_t)lim;
    s->soc_excess = (uint16_t)r7;
    if (sxth(r7) > -1) {                                               /* 0800C648 */
        r4 = (uint32_t)s->knee_soc + ((r4 * (r7 & 0xFFFFu)) >> 12);
    } else {
        s->soc_excess = 0;
        r4 = lim;
    }
    s->limit_soc = (uint16_t)r4;
    r7 = lim - (uint32_t)s->knee_temp;
    s->temp_excess = (uint16_t)r7;
    r3 = lim;
    if (sxth(r7) > -1) {                                               /* 0800C668 */
        const uint32_t t = (uint32_t)sxth(((r7 & 0xFFFFu) * in->thermal_a) >> 12);
        const uint32_t r0 = t * in->thermal_b;
        s->temp_add = (uint16_t)(r0 >> 12);
        r3 = (uint32_t)s->knee_temp + (r0 >> 12);
    } else {
        s->temp_excess = 0;
    }
    s->limit_temp = (uint16_t)r3;
    if (sxth(r3) < sxth(r4)) {                                         /* 0800C686 */
        r4 = r3;
    }
    s->limit_final = (uint16_t)r4;
    s->limit = (uint16_t)r4;
    r4o = (sxth(r4) <= 0x64) ? 0x64u : r4;                             /* 0800C68E */
    if (pct == 0) {                                                    /* 0800C698 */
        s->limit = 0;
        pi_reset(&s->pi);
    }
    return (uint16_t)r4o;
}

/* 0x0800C6B4 */
uint16_t g53_g1_step(g53_g1_state_t *s, uint16_t taper_q12, uint16_t feedback)
{
    const int32_t r3 = sxth(s->limit);
    const uint32_t r5 = s->ramped;
    const uint32_t prod = (uint32_t)r3 * (uint32_t)taper_q12;         /* 0800C6D4 */
    const uint32_t r0 = prod >> 12;
    const uint32_t r1 = r0 - r5;                                       /* 0800C6E2 */
    const int32_t step = sxth(s->step);
    const int32_t floor = sxth(s->floor);
    uint32_t r6 = r0;
    uint32_t sp;
    int32_t out;

    if (sxth(r1) > step) {                                             /* 0800C6EE addgt */
        r6 = (uint32_t)step + r5;
    }
    sp = r6;
    if (sxth(r6) < floor) {                                            /* 0800C6F6 movlt */
        sp = (uint32_t)floor;
    }
    if (r3 <= floor) {                                                 /* 0800C6FE movle */
        sp = r6;
    }
    s->pi.sp = (uint16_t)sp;
    s->target = (uint16_t)r0;
    s->pi.fb = feedback;
    s->taper = taper_q12;
    s->diff = (uint16_t)r1;
    s->ramped = (uint16_t)r6;
    pi_step(&s->pi);
    out = sxth(s->pi.out);
    if (out <= 0) {                                                    /* 0800C728 */
        out = 0;
        s->pi.out = 0;
    }
    s->g1 = (uint16_t)out;
    return s->g1;
}
