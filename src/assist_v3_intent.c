/*
 * Assist Behavior V3 - Rider Intent V1 (ARCHITECTURE_V3.md section 4).
 *
 *   V3-1 crank phase tracker   relative phase mod 96 from the signed step count; per-step torque
 *   V3-2 rider intent          learned phase template + expected-effort-normalised windows; I and
 *                              the envelope-equivalent env_equiv (load-domain kL, re-check N1)
 *   V3-3 release classifier    NORMAL / PHASE_DIP / ATTACK / TRUE_RELEASE / PEDAL_STOP, template
 *                              mode and template-free fallback mode (R1-#5), hysteresis + dwell
 *
 * Integer only, deterministic, no malloc, no float. Cost: O(1) per control tick; per crank step
 * O(1) plus one <= 48-entry window walk (only for the last step of a call); once per revolution the
 * learning (96 + NB), the re-alignment when unaligned (96 x NB) and one kL evaluation (<= 400
 * recurrence iterations at the 15 rpm clamp, 300 at 20 rpm, 46 at 130 rpm).
 *
 * PRIOR TEMPLATE. Population-typical two-stroke shape: s(theta) proportional to
 * 0.1 + 1.8*|sin(theta)| (each leg a half-sine power stroke over a 10 % floor), averaged over each
 * bin and normalised to mean 1.0 (Q12 sum = NB*4096). peak/mean = 1.51 (NB 24) / 1.46 (NB 12),
 * dead spots (s = 0.27 / 0.45 of the mean) at 0 and 180 deg of the prior's OWN phase, symmetric
 * legs. The phase is relative (no TDC), so the prior only becomes a phase reference after the first
 * full revolution, by the same circular cross-correlation that re-aligns after a glitch. Until then
 * the classifier runs in fallback mode (no short window) and env_equiv uses the prior's kL at the
 * current cadence, which does not depend on the phase.
 */
#include "assist_v3_intent.h"

#include <string.h>

#define NB ASSIST_V3_NB
#define SPR ((uint8_t)ASSIST_V3_STEPS_PER_REV)
#define SPB ((uint8_t)ASSIST_V3_STEPS_PER_BIN)
#define Q12 4096u
#define S_SUM ((uint32_t)NB * Q12)
#define S_MAX 32768u                 /* 8.0: a single bin can never hold more than 8x the mean */
#define REC_ACC_PER_REV 6000u        /* D7EC runs every 10 ms: one minute = 6000 calls        */
#define REC_ACC_PER_BIN (REC_ACC_PER_REV / NB)
#define PERIOD_Q 4u                  /* step period estimate in 1/16 ticks                     */
#define OBS_MAX 16383u               /* per-step load clip (physical range < 6000 CLU); keeps
                                      * every window product in 32 bits (no 64-bit division)   */

enum { CLS_NORMAL = 0, CLS_ATTACK = 1, CLS_RELEASE = 2 };

static const assist_v3_intent_params_t P = {
    .alpha_shift = 3u,
    .alpha_shift_low_conf = 2u,
    .a_min_steps = 8u,
    .w_max_steps = 48u,
    .fb_rel_steps = 46u,
    .s_min_q12 = 8u * Q12,
    .s_strong_q12 = Q12,
    .r_att_q12 = 5734u,              /* 1.4 */
    .r_att_fb_q12 = 5120u,           /* 1.25 */
    .r_rel_q12 = 2048u,              /* 0.5 */
    .exit_band_q12 = 819u,           /* 0.2 */
    .dip_q12 = 2048u,                /* 0.5 */
    .dwell_steps = 24u,
    .rel_min_clu = 60u,
    .att_min_clu = 120u,
    .agree_floor_clu = 40u,
    .band_min_rpm = 15u,
    .band_max_rpm = 150u,
    .t_stop_periods = 4u,
    .t_stop_min_ticks = 240u,        /* 60 ms */
    .t_stop_max_ticks = 1600u,       /* 400 ms */
    .stable_q12 = 614u,              /* 0.15 */
    .learn_min_clu = 150u,
    .c_min_q12 = 2048u,
    .c_high_q12 = 3072u,
    .c_step_q12 = 1024u,
    .c_fall_q12 = 512u,
    .c_mismatch_q12 = 1024u,
    .c_stop_decay_q12 = 16u,
    .res_good_q12 = 614u,            /* 0.15 */
    .res_mismatch_q12 = 1434u,       /* 0.35 */
    .align_contrast_q12 = 614u,      /* 0.15 */
    .kl_min_q12 = 4096u,             /* 1.0 */
    .kl_max_q12 = 10240u,            /* 2.5 */
    .kl_floor_clu = 300u,
    .kl_prior_ref_clu = 1000u,
    .kl_recompute_q12 = 410u,        /* 0.10 */
    .kl_cad_min_rpm = 15u,
    .kl_cad_max_rpm = 250u,
    .eb74_offset = 750u,
    .eb74_gain_num = 2450u,
    .eb74_gain_den = 6000u,
    .eb74_saturation = 3200u,
    .eb74_thr_engaged = 820u,
    .eb74_deadband = 245u,
};

#if NB == 24
static const uint16_t PRIOR[NB] = {
    1099u, 2587u, 3921u, 5010u, 5780u, 6179u, 6179u, 5780u, 5010u, 3921u, 2587u, 1099u,
    1099u, 2587u, 3921u, 5010u, 5780u, 6179u, 6179u, 5780u, 5010u, 3921u, 2587u, 1099u };
#else
static const uint16_t PRIOR[NB] = {
    1843u, 4465u, 5980u, 5980u, 4465u, 1843u, 1843u, 4465u, 5980u, 5980u, 4465u, 1843u };
#endif

typedef struct {
    /* template */
    uint16_t s[NB];
    uint16_t conf;
    uint16_t revs_learned;
    bool aligned;
    uint8_t tpl_offset;              /* template phase = (raw phase + offset) mod 96 */
    /* phase tracker */
    bool primed;
    int32_t last_steps;
    uint8_t raw_phase;
    uint16_t ring[96];               /* per-step obs, indexed by raw phase */
    uint32_t sum96;                  /* sum of the last min(n, 96) post-restart obs */
    uint16_t steps_since_restart;
    uint8_t rev_count;
    bool rev_invalid;
    uint16_t prev_rev_mean;
    bool prev_rev_valid;
    /* load accumulation between steps */
    uint32_t load_sum;
    uint32_t load_ticks;
    uint16_t last_obs;
    /* step timing / stop */
    bool have_step_tick;
    uint32_t last_step_tick;
    uint32_t period_q;               /* expected step period, ticks << PERIOD_Q */
    bool stopped;
    uint32_t stop_decay_acc;
    /* classifier */
    uint8_t class_state;
    uint8_t dwell;
    uint8_t reported_class;
    bool template_mode;
    uint16_t intent;
    uint16_t e_short;
    uint16_t e_long;
    /* kL */
    uint16_t kl;
    bool kl_prior;
    uint16_t kl_cad;
    uint16_t kl_thr;
    uint16_t kl_irev;
} intent_state_t;

static intent_state_t S;
static assist_v3_intent_debug_t D;

/* ------------------------------------------------------------------ small helpers */

static uint16_t sat16(uint32_t v) { return v > 65535u ? 65535u : (uint16_t)v; }
static uint32_t absdiff(uint32_t a, uint32_t b) { return a > b ? a - b : b - a; }

/* a and b agree within the exit band of b, or both are at the release floor. */
static bool agree(uint32_t a, uint32_t b)
{
    if (a < P.agree_floor_clu && b < P.agree_floor_clu) return true;
    return absdiff(a, b) * Q12 < (uint32_t)P.exit_band_q12 * b;
}

static uint8_t tpl_phase_of(uint8_t raw) { return (uint8_t)((raw + S.tpl_offset) % SPR); }
static uint16_t s_at_raw(uint8_t raw) { return S.s[tpl_phase_of(raw) / SPB]; }

uint16_t assist_v3_eb74_active(uint32_t load_clu, uint16_t thr)
{
    uint32_t l = load_clu > 1000000u ? 1000000u : load_clu;
    uint32_t source = P.eb74_offset + (l * P.eb74_gain_num) / P.eb74_gain_den;
    if (source > P.eb74_saturation) source = P.eb74_saturation;
    return source > thr ? (uint16_t)(source - thr) : 0u;
}

uint16_t assist_v3_intent_compute_kl(const uint16_t *s_q12, uint16_t i_rev, uint16_t cad_rpm,
                                     uint16_t thr, uint16_t *env_ss, uint32_t *iterations)
{
    uint32_t cad = cad_rpm;
    uint32_t k, k1, env = 0u, sum = 0u, cnt = 0u, acc, end, peak = 0u, x = 0u;
    uint8_t pb = 0u, xbin = 0xFFu;

    if (env_ss) *env_ss = 0u;
    if (iterations) *iterations = 0u;
    if (cad < P.kl_cad_min_rpm) cad = P.kl_cad_min_rpm;
    if (cad > P.kl_cad_max_rpm) cad = P.kl_cad_max_rpm;
    /* reconstructed stroke x(b) = EB74_active(I_rev * s[b]) is evaluated in place (no table:
     * stack budget), once per bin change in the loop below */
    for (uint8_t b = 0u; b < NB; ++b) {
        uint32_t xv = assist_v3_eb74_active(((uint32_t)i_rev * s_q12[b]) >> 12, thr);
        if (xv > peak) { peak = xv; pb = b; }
    }
    if (peak == 0u || i_rev == 0u) return 0u;
    /* D7EC envelope, chain_d7ec_model: k = 8*cad (40 at cad 0, never here), attack env = cur,
     * else env = udiv(prev*k16, k16|1), one iteration per 10 ms. The reconstructed stroke advances
     * cad/6000 revolution per iteration. Starting at the peak bin with env = 0 makes the first
     * iteration the attack, i.e. this one revolution is the periodic steady state. */
    k = (cad * 8u) & 0xFFFFu;
    k1 = k | 1u;
    acc = (uint32_t)pb * REC_ACC_PER_BIN;
    end = acc + REC_ACC_PER_REV;
    while (acc < end) {
        uint8_t b = (uint8_t)((acc / REC_ACC_PER_BIN) % NB);
        if (b != xbin) {
            xbin = b;
            x = assist_v3_eb74_active(((uint32_t)i_rev * s_q12[b]) >> 12, thr);
        }
        if (x > env) env = x;
        else env = (env * k) / k1;
        sum += env;
        ++cnt;
        acc += cad;
    }
    {
        uint32_t ess = sum / cnt;
        /* back to the load domain (re-check N1): the constant load EB74 maps to env_ss */
        uint32_t l_eq = ((ess + thr - P.eb74_offset) * P.eb74_gain_den) / P.eb74_gain_num;
        uint32_t kl = (l_eq * Q12) / i_rev;
        if (kl < P.kl_min_q12) kl = P.kl_min_q12;
        if (kl > P.kl_max_q12) kl = P.kl_max_q12;
        if (env_ss) *env_ss = (uint16_t)ess;
        if (iterations) *iterations = cnt;
        return (uint16_t)kl;
    }
}

static uint16_t active_thr(const assist_v3_intent_in_t *in)
{
    return in->v3_engaged ? P.eb74_thr_engaged : (uint16_t)(in->eb74_zero + P.eb74_deadband);
}

static void kl_evaluate(bool from_prior, uint16_t i_rev, uint16_t cad, uint16_t thr)
{
    uint16_t kl = 0u, ess = 0u;
    uint32_t it = 0u, it2 = 0u;
    if (!from_prior) {
        kl = assist_v3_intent_compute_kl(S.s, i_rev, cad, thr, &ess, &it);
        if (kl == 0u) from_prior = true;   /* stroke never above threshold: kL undefined */
    }
    if (from_prior) {
        kl = assist_v3_intent_compute_kl(PRIOR, P.kl_prior_ref_clu, cad, thr, &ess, &it2);
        it += it2;
        if (kl == 0u) kl = P.kl_min_q12;
    }
    S.kl = kl;
    S.kl_prior = from_prior;
    S.kl_cad = cad;
    S.kl_thr = thr;
    S.kl_irev = i_rev;
    D.env_ss = ess;
    D.last_kl_iterations = it;
    if (it > D.max_kl_iterations) D.max_kl_iterations = it;
    D.kl_recomputes++;
}

/* ------------------------------------------------------------------ template */

static void renormalise(void)
{
    uint32_t sum = 0u, nsum = 0u;
    uint8_t maxb = 0u;
    for (uint8_t b = 0u; b < NB; ++b) {
        if (S.s[b] > S_MAX) S.s[b] = S_MAX;
        sum += S.s[b];
    }
    if (sum == 0u) {
        memcpy(S.s, PRIOR, sizeof(S.s));
        return;
    }
    for (uint8_t b = 0u; b < NB; ++b) {
        uint32_t v = ((uint32_t)S.s[b] * S_SUM + sum / 2u) / sum;   /* <= 3.23e9: fits u32 */
        if (v > S_MAX) v = S_MAX;
        S.s[b] = (uint16_t)v;
        nsum += v;
        if (S.s[b] > S.s[maxb]) maxb = b;
    }
    /* exact mean: the rounding remainder goes to the largest bin */
    if (nsum > S_SUM) S.s[maxb] = (uint16_t)(S.s[maxb] - (nsum - S_SUM));
    else S.s[maxb] = (uint16_t)(S.s[maxb] + (S_SUM - nsum));
}

/* Sum of the ring over the raw phases whose template phase lies in bin b for offset off:
 * raw p = (b*SPB + j - off) mod 96, j < SPB. Read in place (no scratch arrays: stack budget). */
static uint32_t bin_sum(uint8_t b, uint8_t off)
{
    uint32_t sum = 0u;
    uint32_t start = ((uint32_t)b * SPB + SPR - off) % SPR;
    for (uint8_t j = 0u; j < SPB; ++j) sum += S.ring[(start + j) % SPR];
    return sum;
}

/* Circular cross-correlation of the last revolution (ring) against the template, at step
 * resolution: 96 offsets x 96 ring reads (= 96 x NB bin products), O(1) memory. */
static void try_align(void)
{
    uint64_t cmax = 0u, cmin = UINT64_MAX;
    uint8_t best = 0u;
    for (uint8_t off = 0u; off < SPR; ++off) {
        uint64_t c = 0u;
        for (uint8_t b = 0u; b < NB; ++b) c += (uint64_t)bin_sum(b, off) * S.s[b];
        if (c > cmax) { cmax = c; best = off; }
        if (c < cmin) cmin = c;
    }
    if (cmax > 0u && (cmax - cmin) * Q12 >= (uint64_t)P.align_contrast_q12 * cmax) {
        S.tpl_offset = best;
        S.aligned = true;
        D.alignments++;
    } else {
        D.alignment_failures++;
        S.conf = S.conf > P.c_fall_q12 ? (uint16_t)(S.conf - P.c_fall_q12) : 0u;
    }
}

/* Bin mean of the last revolution relative to its mean, Q12, clamped to S_MAX. */
static uint32_t bin_ratio(uint8_t b, uint16_t i_rev)
{
    uint32_t rb = (bin_sum(b, S.tpl_offset) * Q12) / ((uint32_t)SPB * i_rev);   /* <= 8*OBS_MAX */
    return rb > S_MAX ? S_MAX : rb;
}

static void revolution_complete(const assist_v3_intent_in_t *in)
{
    uint16_t i_rev = (uint16_t)(S.sum96 / SPR);
    bool valid = !S.rev_invalid && i_rev >= P.learn_min_clu;
    bool stable = S.prev_rev_valid && S.prev_rev_mean > 0u &&
                  absdiff(i_rev, S.prev_rev_mean) * Q12 <= (uint32_t)P.stable_q12 * S.prev_rev_mean;
    uint16_t cad = (uint16_t)(in->cadence_rpm > 0 ? in->cadence_rpm : 0);

    D.last_rev_mean = i_rev;
    if (valid && !S.aligned) try_align();
    if (valid && S.aligned) {
        uint32_t res = 0u;
        /* two passes over the bins (residual, then learning) recompute the bin ratio in place
         * instead of keeping NB-entry scratch arrays (stack budget) */
        for (uint8_t b = 0u; b < NB; ++b) res += absdiff(bin_ratio(b, i_rev), S.s[b]);
        res /= NB;
        D.last_residual_q12 = sat16(res);
        if (stable) {
            uint8_t sh = S.conf < P.c_min_q12 ? P.alpha_shift_low_conf : P.alpha_shift;
            if (res > P.res_mismatch_q12) {
                if (S.conf > P.c_mismatch_q12) { S.conf = P.c_mismatch_q12; D.mismatch_drops++; }
            } else if (res < P.res_good_q12) {
                uint32_t c = (uint32_t)S.conf + P.c_step_q12;
                S.conf = (uint16_t)(c > Q12 ? Q12 : c);
            } else {
                S.conf = S.conf > P.c_fall_q12 ? (uint16_t)(S.conf - P.c_fall_q12) : 0u;
            }
            for (uint8_t b = 0u; b < NB; ++b) {
                int32_t d = (int32_t)bin_ratio(b, i_rev) - (int32_t)S.s[b];
                int32_t step = d >= 0 ? (d >> sh) : -((-d) >> sh);
                int32_t v = (int32_t)S.s[b] + step;
                S.s[b] = (uint16_t)(v < 0 ? 0 : v);
            }
            renormalise();
            if (S.revs_learned < 65535u) S.revs_learned++;
        }
    }
    /* kL (section 4.3): learned template when the revolution is stable and above the floor,
     * otherwise the prior's kL at this cadence. */
    if (cad > 0u) {
        bool use_learned = valid && stable && i_rev >= P.kl_floor_clu;
        kl_evaluate(!use_learned, i_rev, cad, active_thr(in));
    }
    S.prev_rev_mean = i_rev;
    S.prev_rev_valid = !S.rev_invalid;
    S.rev_invalid = false;
    S.rev_count = 0u;
}

/* ------------------------------------------------------------------ phase tracker */

static void restart(void)
{
    S.steps_since_restart = 0u;
    S.rev_count = 0u;
    S.sum96 = 0u;
    S.class_state = CLS_NORMAL;
    S.dwell = 0u;
    S.prev_rev_valid = false;
    S.period_q = ((uint32_t)P.t_stop_max_ticks / P.t_stop_periods) << PERIOD_Q;
}

static void mark_unaligned(void)
{
    S.aligned = false;
    S.rev_count = 0u;   /* the next full revolution is entirely post-glitch */
}

/* Ring + counters for one forward step. */
static void push_step(uint16_t obs, const assist_v3_intent_in_t *in)
{
    S.raw_phase = (uint8_t)((S.raw_phase + 1u) % SPR);
    if (S.steps_since_restart >= SPR) S.sum96 -= S.ring[S.raw_phase];
    S.ring[S.raw_phase] = obs;
    S.sum96 += obs;
    if (S.steps_since_restart < 65535u) S.steps_since_restart++;
    if (S.dwell < 255u) S.dwell++;
    D.total_steps++;
    S.rev_count++;
    if (S.rev_count >= SPR) revolution_complete(in);
}

/* Windows + classification after the last step of a call. */
static void classify(const assist_v3_intent_in_t *in)
{
    uint32_t avail = S.steps_since_restart < SPR ? S.steps_since_restart : SPR;
    uint32_t wmax = avail < P.w_max_steps ? avail : P.w_max_steps;
    uint32_t so = 0u, ss = 0u, mo = 0u, ms = 0u, sho = 0u, shs = 0u;
    bool short_done = false;
    uint16_t cad = (uint16_t)(in->cadence_rpm >= 0 ? in->cadence_rpm : -in->cadence_rpm);
    uint32_t e_s, e_l, mean48, e48t;
    bool rel, att, tmode;

    if (wmax == 0u) return;
    for (uint32_t i = 0u; i < wmax; ++i) {
        uint8_t rp = (uint8_t)((S.raw_phase + SPR - i) % SPR);
        uint32_t o = S.ring[rp];
        uint32_t sb = s_at_raw(rp);
        so += o;
        ss += sb;
        if (o > mo && i < P.fb_rel_steps) mo = o;
        if (sb > ms) ms = sb;
        if (!short_done && i + 1u >= P.a_min_steps && ss >= P.s_min_q12 && ms >= P.s_strong_q12) {
            sho = so; shs = ss; short_done = true;
        }
    }
    if (wmax > D.max_walk_steps) D.max_walk_steps = wmax;
    if (!short_done) { sho = so; shs = ss; }
    mean48 = so / wmax;
    e48t = ss ? (so * Q12) / ss : mean48;   /* so <= 48*OBS_MAX: fits u32 */

    tmode = S.aligned && S.conf >= P.c_min_q12 && cad >= P.band_min_rpm && cad <= P.band_max_rpm &&
            avail >= P.a_min_steps;
    S.template_mode = tmode;
    if (tmode) {
        e_s = shs ? (sho * Q12) / shs : mean48;
        e_l = (avail >= SPR && S.conf < P.c_high_q12) ? S.sum96 / SPR : e48t;
        rel = e_l >= P.rel_min_clu && e_s * Q12 < (uint32_t)P.r_rel_q12 * e_l;
        att = e_s >= P.att_min_clu && e_s * Q12 > (uint32_t)P.r_att_q12 * e_l;
    } else {
        bool full_half = avail >= P.fb_rel_steps;
        e_s = mean48;
        e_l = avail >= SPR ? S.sum96 / SPR : (avail >= P.w_max_steps ? mean48 : S.sum96 / avail);
        /* every 180 deg holds a power stroke: release needs the MAXIMUM below R_rel * I */
        rel = full_half && e_l >= P.rel_min_clu && mo * Q12 < (uint32_t)P.r_rel_q12 * e_l;
        att = avail >= P.w_max_steps && e_s >= P.att_min_clu && e_s * Q12 > (uint32_t)P.r_att_fb_q12 * e_l;
    }

    switch (S.class_state) {
    case CLS_NORMAL:
        if (rel) { S.class_state = CLS_RELEASE; S.dwell = 0u; }
        else if (att) { S.class_state = CLS_ATTACK; S.dwell = 0u; }
        break;
    case CLS_RELEASE:
        if (att) { S.class_state = CLS_ATTACK; S.dwell = 0u; }
        else if (S.dwell >= P.dwell_steps && agree(e_s, e_l)) S.class_state = CLS_NORMAL;
        break;
    default: /* CLS_ATTACK */
        if (rel) { S.class_state = CLS_RELEASE; S.dwell = 0u; }
        else if (S.dwell >= P.dwell_steps && agree(e_s, e_l)) S.class_state = CLS_NORMAL;
        break;
    }

    S.e_short = sat16(e_s);
    S.e_long = sat16(e_l);
    if (S.class_state == CLS_NORMAL) {
        S.intent = S.e_long;
        S.reported_class = ((uint32_t)S.last_obs * Q12 < (uint32_t)P.dip_q12 * S.intent)
            ? ASSIST_V3_CLASS_PHASE_DIP : ASSIST_V3_CLASS_NORMAL_PRESSURE;
    } else {
        S.intent = S.e_short;
        S.reported_class = S.class_state == CLS_RELEASE ? ASSIST_V3_CLASS_TRUE_RELEASE
                                                        : ASSIST_V3_CLASS_ATTACK;
    }
}

/* ------------------------------------------------------------------ public API */

void assist_v3_intent_power_on(void)
{
    memset(&S, 0, sizeof(S));
    memset(&D, 0, sizeof(D));
    memcpy(S.s, PRIOR, sizeof(S.s));
    restart();
    S.stopped = true;
    kl_evaluate(true, 0u, 60u, (uint16_t)(750u + P.eb74_deadband));
    S.kl_cad = 0u;      /* first real cadence triggers a recompute */
    D.kl_recomputes = 0u;
}

void assist_v3_intent_reset(void)
{
    uint16_t s[NB];
    uint16_t conf = S.conf, revs = S.revs_learned, kl = S.kl, irev = S.kl_irev, thr = S.kl_thr;
    bool klp = S.kl_prior;
    uint8_t off = S.tpl_offset;
    memcpy(s, S.s, sizeof(s));
    memset(&S, 0, sizeof(S));
    memcpy(S.s, s, sizeof(s));
    S.conf = conf;
    S.revs_learned = revs;
    S.tpl_offset = off;
    S.aligned = false;  /* steps may have been missed: re-align (confidence kept for the restore) */
    S.kl = kl;
    S.kl_prior = klp;
    S.kl_irev = irev;
    S.kl_thr = thr;
    S.kl_cad = 0u;      /* next cadence recomputes kL from the kept source */
    restart();
    S.stopped = true;
}

void assist_v3_intent_update(const assist_v3_intent_in_t *in, assist_v3_intent_out_t *out)
{
    uint32_t el = in->elapsed_ticks ? in->elapsed_ticks : 1u;
    uint32_t el_acc = el > P.t_stop_max_ticks ? P.t_stop_max_ticks : el;
    uint16_t load = in->torque_valid ? in->load_ctrl : 0u;
    uint16_t cad = (uint16_t)(in->cadence_rpm > 0 ? in->cadence_rpm : 0);
    uint16_t thr = active_thr(in);
    int32_t delta = 0;
    bool forward = false;

    /* load accumulation (mean per step; older load is forgotten beyond ~400 ms) */
    S.load_sum += (uint32_t)load * el_acc;
    S.load_ticks += el_acc;
    while (S.load_ticks > P.t_stop_max_ticks) { S.load_sum >>= 1; S.load_ticks >>= 1; }
    if (!in->torque_valid) { S.rev_invalid = true; S.conf = 0u; }

    if (!S.primed) {
        S.primed = true;
        S.last_steps = in->crank_steps;
        S.last_step_tick = in->crank_step_tick;
        S.have_step_tick = false;
    } else {
        delta = (int32_t)((uint32_t)in->crank_steps - (uint32_t)S.last_steps);
        S.last_steps = in->crank_steps;
    }

    if (in->pas_glitch) mark_unaligned();

    if (delta > (int32_t)SPR || delta < -(int32_t)SPR) {
        /* more than a revolution in one call: the order and the load are lost */
        S.raw_phase = (uint8_t)(((int32_t)S.raw_phase + (delta % (int32_t)SPR) + (int32_t)SPR) % (int32_t)SPR);
        mark_unaligned();
        restart();
        S.stopped = true;
        S.load_sum = 0u; S.load_ticks = 0u;
    } else if (delta < 0) {
        /* back-pedalling: phase moves backwards and survives; intent restarts afterwards */
        S.raw_phase = (uint8_t)(((int32_t)S.raw_phase + delta + (int32_t)SPR) % (int32_t)SPR);
        restart();
        S.stopped = true;
        S.load_sum = 0u; S.load_ticks = 0u;
        S.have_step_tick = false;
    } else if (delta > 0) {
        uint32_t n = (uint32_t)delta;
        uint32_t obs32 = S.load_ticks ? S.load_sum / S.load_ticks : 0u;
        uint16_t obs = (uint16_t)(obs32 > OBS_MAX ? OBS_MAX : obs32);
        uint32_t since_prev = in->crank_step_tick - S.last_step_tick;
        forward = true;
        if (S.stopped) {
            restart();
            S.stopped = false;
        } else if (S.have_step_tick) {
            int32_t per = (int32_t)((since_prev / n) << PERIOD_Q);
            int32_t cur = (int32_t)S.period_q;
            S.period_q = (uint32_t)(cur + (per - cur) / 4);
        }
        S.have_step_tick = true;
        S.last_step_tick = in->crank_step_tick;
        S.load_sum = 0u; S.load_ticks = 0u;
        S.last_obs = obs;
        if (n > D.max_steps_per_call) D.max_steps_per_call = n;
        for (uint32_t i = 0u; i < n; ++i) push_step(obs, in);
        classify(in);
    }

    /* PEDAL_STOP: no step for T_stop = clamp(4 expected step periods, 60, 400 ms); G53 true-stop,
     * native real_stop or a reverse inhibit make it earlier, but never before 60 ms without a
     * step (they lag the crank at a restart). */
    if (!forward && !S.stopped) {
        uint32_t since = in->now_tick - in->crank_step_tick;
        uint32_t t_stop = ((S.period_q * P.t_stop_periods) >> PERIOD_Q);
        if (t_stop < P.t_stop_min_ticks) t_stop = P.t_stop_min_ticks;
        if (t_stop > P.t_stop_max_ticks) t_stop = P.t_stop_max_ticks;
        if (since >= P.t_stop_min_ticks &&
            (since > t_stop || in->g53_true_stop || in->real_stop ||
             (in->direction_inhibit && in->inhibit_is_reverse))) {
            S.stopped = true;
        }
    }
    if (S.stopped) {
        S.reported_class = ASSIST_V3_CLASS_PEDAL_STOP;
        S.template_mode = false;
        S.stop_decay_acc += el_acc;
        while (S.stop_decay_acc >= 4000u) {
            S.stop_decay_acc -= 4000u;
            S.conf = S.conf > P.c_stop_decay_q12 ? (uint16_t)(S.conf - P.c_stop_decay_q12) : 0u;
        }
    }

    /* kL recompute: threshold change, or cadence moved by more than 10 % since the last value */
    if (cad > 0u) {
        bool cad_moved = S.kl_cad == 0u ||
            absdiff(cad, S.kl_cad) * Q12 > (uint32_t)P.kl_recompute_q12 * S.kl_cad;
        if (cad_moved || thr != S.kl_thr) kl_evaluate(S.kl_prior, S.kl_irev, cad, thr);
    } else if (thr != S.kl_thr) {
        kl_evaluate(S.kl_prior, S.kl_irev, S.kl_cad ? S.kl_cad : P.kl_cad_min_rpm, thr);
    }

    if (out) {
        uint8_t tp = tpl_phase_of(S.raw_phase);
        uint32_t keq = ((uint32_t)S.kl * S.intent) >> 12;
        memset(out, 0, sizeof(*out));
        out->intent = S.intent;
        out->e_short = S.e_short;
        out->e_long = S.e_long;
        out->env_equiv = assist_v3_eb74_active(keq, thr);
        out->kl_q12 = S.kl;
        out->confidence_q12 = S.conf;
        out->expected_effort = sat16(((uint32_t)S.intent * S.s[tp / SPB]) >> 12);
        out->revolutions_learned = S.revs_learned;
        out->steps_since_restart = S.steps_since_restart;
        out->release_class = S.reported_class;
        out->phase = tp;
        out->phase_aligned = S.aligned;
        out->template_mode = S.template_mode;
        out->kl_from_prior = S.kl_prior;
        out->engage_ok = out->env_equiv > 0u && in->eb74_armed;
    }
    D.tpl_offset = S.tpl_offset;
    D.class_state = S.class_state;
}

const assist_v3_intent_params_t *assist_v3_intent_params(void) { return &P; }
const assist_v3_intent_debug_t *assist_v3_intent_debug(void) { return &D; }
const uint16_t *assist_v3_intent_template(void) { return S.s; }
const uint16_t *assist_v3_intent_prior(void) { return PRIOR; }
