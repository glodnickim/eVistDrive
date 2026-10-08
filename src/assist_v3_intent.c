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
 * O(1) plus one <= 48-entry window walk (only for the last step of a call). The per-revolution work
 * (D-039, REVIEW-T #13) is SPREAD over the following calls by a small job runner that does at most
 * ONE bounded unit per call:
 *   re-alignment (when unaligned)  bin sums of the ring once (96 reads), then the circular
 *                                  cross-correlation over the 96 step offsets, ALIGN_OFFSETS_PER_CALL
 *                                  offsets per call with sliding bin sums (NB MACs + 2 NB reads per
 *                                  offset) - the full-resolution search, same order, same tie rule;
 *   learning + residual            one fused pass over the NB bins + renormalisation (one call);
 *   kL                             the D7EC recurrence, KL_ITER_PER_CALL iterations per call (<= 400
 *                                  iterations at the 15 rpm clamp), the prior fallback likewise;
 *                                  a cadence/threshold-triggered kL is queued on the same runner.
 * A revolution's job therefore completes a few ms after the revolution (<= ~25 calls); a step that
 * lands meanwhile overwrites one ring entry with the same phase of the next revolution. Worst case
 * per call: one window walk (48) + the steps of the call + one unit (<= 4 x 3 NB = 288 inner ops).
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
#define ALIGN_OFFSETS_PER_CALL 4u    /* D-039: 96 offsets in 24 calls                          */
#define KL_ITER_PER_CALL 64u         /* D-039: <= 400 recurrence iterations in <= 7 calls       */

enum { JOB_IDLE = 0, JOB_ALIGN_INIT, JOB_ALIGN, JOB_LEARN, JOB_KL };

/* One resumable evaluation of the D7EC recurrence over a reconstructed revolution (kL). */
typedef struct {
    const uint16_t *v;               /* template / prior (Q12, n = NB) or the ring (CLU, n = 96) */
    uint32_t cad, k, k1, env, sum, cnt, acc, end, x;
    uint16_t i_rev, thr;
    uint8_t xbin, n;
    bool direct;                     /* v holds loads (ring), not template ratios */
} klc_t;

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
    uint16_t clean_steps;            /* steps since the last restart / glitch / invalid torque */
    bool ring_stable;                /* the last completed revolution was valid, stable, clean */
    uint8_t rev_run;                 /* consecutive reverse steps (a forward step clears it) */
    uint16_t rev_peak;               /* largest per-step obs of the revolution in progress */
    uint16_t prev_rev_peak;          /* ... of the last completed revolution */
    uint16_t shape_q12;              /* peak / mean of the last STABLE revolution, Q12 (0 = none) */
    uint16_t shape_early_q12;        /* before one: max of (180 deg peak / 180 deg mean) since the
                                      * restart while NORMAL (one stroke period of a two-leg stroke) */
    bool ring_dirty;                 /* since then a step replaced its same-phase entry by > 15 % */
    uint16_t peak_restart;           /* largest per-step obs since the last restart */
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
    /* kL: the value in force, and the parameters of the latest REQUEST (the triggers compare
     * against these, so a queued evaluation is never re-requested) */
    uint16_t kl;
    uint8_t kl_src;                  /* KL_SRC_* of the kL value in force */
    uint16_t kl_cad;
    uint16_t kl_thr;
    uint16_t kl_irev;
    /* deferred per-revolution job (D-039) */
    uint8_t job;
    uint8_t job_off;                 /* next alignment offset */
    uint8_t job_best;
    bool job_valid, job_stable, job_clean;
    uint16_t job_irev, job_cad, job_thr;
    uint64_t job_cmax, job_cmin;
    uint16_t bs[NB];                 /* sliding bin sums: bin_sum(b, job_off) */
    /* deferred kL run + one pending request (latest wins) */
    bool kl_active, kl_pending;
    uint8_t kl_run_src, kl_req_src;
    uint16_t kl_req_irev, kl_req_cad, kl_req_thr;
    uint32_t kl_iters;
    klc_t klc;
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

/* kL sources (section 4.3, rework of 2026-10-07): the measured stroke of the last clean revolution
 * (the 96-entry ring, exact step resolution) whenever one exists; the learned template, then the
 * prior, only as fallback. The binned template (4 steps averaged per bin) and the template's learning
 * lag both under-estimate the stroke peaks the D7EC envelope holds (G1-LEVEL rework). */
enum { KL_SRC_PRIOR = 0, KL_SRC_TPL = 1, KL_SRC_RING = 2 };

/* x(position) of the stroke under evaluation: ring = the measured per-step obs, template / prior =
 * EB74 of I_rev * s[bin]. */
static uint32_t klc_x(const klc_t *c, uint8_t pos)
{
    const uint32_t l = c->direct ? c->v[pos] : (((uint32_t)c->i_rev * c->v[pos]) >> 12);
    return assist_v3_eb74_active(l, c->thr);
}

/* Start one evaluation over n positions per revolution (NB bins or 96 ring steps), evaluated in
 * place (no table: stack budget). Returns false when kL is undefined (stroke never above
 * threshold). */
static bool klc_start(klc_t *c, const uint16_t *v, uint8_t n, bool direct, uint16_t i_rev,
                      uint16_t cad_rpm, uint16_t thr)
{
    uint32_t cad = cad_rpm, peak = 0u;
    uint8_t pb = 0u;
    if (cad < P.kl_cad_min_rpm) cad = P.kl_cad_min_rpm;
    if (cad > P.kl_cad_max_rpm) cad = P.kl_cad_max_rpm;
    c->v = v; c->n = n; c->direct = direct; c->i_rev = i_rev; c->thr = thr;
    for (uint8_t b = 0u; b < n; ++b) {
        uint32_t xv = klc_x(c, b);
        if (xv > peak) { peak = xv; pb = b; }
    }
    D.work_this_call += n;
    if (peak == 0u || i_rev == 0u) return false;
    /* D7EC envelope, chain_d7ec_model: k = 8*cad (40 at cad 0, never here), attack env = cur,
     * else env = udiv(prev*k16, k16|1), one iteration per 10 ms. The reconstructed stroke advances
     * cad/6000 revolution per iteration. Starting at the peak position with env = 0 makes the first
     * iteration the attack, i.e. this one revolution is the periodic steady state. Position of an
     * accumulator value: acc * n / 6000 (the first acc rounds up into the peak position). */
    c->cad = cad;
    c->k = (cad * 8u) & 0xFFFFu;
    c->k1 = c->k | 1u;
    c->acc = ((uint32_t)pb * REC_ACC_PER_REV + n - 1u) / n;
    c->end = c->acc + REC_ACC_PER_REV;
    c->env = 0u; c->sum = 0u; c->cnt = 0u; c->x = 0u;
    c->xbin = 0xFFu;
    return true;
}

/* Advance by at most max_iter iterations; true when the revolution is complete. */
static bool klc_run(klc_t *c, uint32_t max_iter)
{
    uint32_t n = 0u;
    while (c->acc < c->end && n < max_iter) {
        uint8_t b = (uint8_t)(((c->acc * c->n) / REC_ACC_PER_REV) % c->n);
        if (b != c->xbin) {
            c->xbin = b;
            c->x = klc_x(c, b);
        }
        if (c->x > c->env) c->env = c->x;
        else c->env = (c->env * c->k) / c->k1;
        c->sum += c->env;
        ++c->cnt;
        c->acc += c->cad;
        ++n;
    }
    D.work_this_call += n;
    return c->acc >= c->end;
}

static uint16_t klc_result(const klc_t *c, uint16_t *env_ss)
{
    uint32_t ess = c->sum / c->cnt;
    /* back to the load domain (re-check N1): the constant load EB74 maps to env_ss */
    uint32_t l_eq = ((ess + c->thr - P.eb74_offset) * P.eb74_gain_den) / P.eb74_gain_num;
    uint32_t kl = (l_eq * Q12) / c->i_rev;
    if (kl < P.kl_min_q12) kl = P.kl_min_q12;
    if (kl > P.kl_max_q12) kl = P.kl_max_q12;
    if (env_ss) *env_ss = (uint16_t)ess;
    return (uint16_t)kl;
}

uint16_t assist_v3_intent_compute_kl(const uint16_t *s_q12, uint16_t i_rev, uint16_t cad_rpm,
                                     uint16_t thr, uint16_t *env_ss, uint32_t *iterations)
{
    klc_t c;
    uint16_t kl;
    if (env_ss) *env_ss = 0u;
    if (iterations) *iterations = 0u;
    if (!klc_start(&c, s_q12, NB, false, i_rev, cad_rpm, thr)) return 0u;
    (void)klc_run(&c, UINT32_MAX);
    kl = klc_result(&c, env_ss);
    if (iterations) *iterations = c.cnt;
    return kl;
}

static uint16_t ring_mean(const uint16_t *ring)
{
    uint32_t sum = 0u;
    for (uint8_t p = 0u; p < SPR; ++p) sum += ring[p];
    D.work_this_call += SPR;
    return (uint16_t)(sum / SPR);
}

uint16_t assist_v3_intent_compute_kl_ring(const uint16_t ring[ASSIST_V3_STEPS_PER_REV], uint16_t cad_rpm,
                                          uint16_t thr, uint16_t *env_ss, uint32_t *iterations)
{
    klc_t c;
    uint16_t kl;
    if (env_ss) *env_ss = 0u;
    if (iterations) *iterations = 0u;
    if (!klc_start(&c, ring, SPR, true, ring_mean(ring), cad_rpm, thr)) return 0u;
    (void)klc_run(&c, UINT32_MAX);
    kl = klc_result(&c, env_ss);
    if (iterations) *iterations = c.cnt;
    return kl;
}

static uint16_t active_thr(const assist_v3_intent_in_t *in)
{
    return in->v3_engaged ? P.eb74_thr_engaged : (uint16_t)(in->eb74_zero + P.eb74_deadband);
}

/* The ring holds one clean revolution: >= 96 steps since the last restart / glitch / invalid torque. */
static bool ring_clean(void) { return S.clean_steps >= SPR; }

/* The ring may be used as the measured stroke: one clean revolution, the last completed one stable,
 * and every step since then replaced its same-phase entry within the stability band. The ring is read
 * in place by the spread kL run, so a transition (attack / release) entering it right after the
 * revolution completed would otherwise put one heavy step into a light stroke (N1: kL clamps at
 * 2.5 - found in the attack_stop matrix rows). */
static bool ring_usable(void) { return S.ring_stable && !S.ring_dirty && ring_clean(); }

/* Queue a kL evaluation (latest request wins). The request parameters are recorded at once, so the
 * cadence / threshold triggers compare against what is queued and never re-request it. */
static void kl_request(uint8_t src, uint16_t i_rev, uint16_t cad, uint16_t thr)
{
    S.kl_pending = true;
    S.kl_req_src = src;
    S.kl_req_irev = i_rev;
    S.kl_req_cad = cad;
    S.kl_req_thr = thr;
    S.kl_cad = cad;
    S.kl_thr = thr;
    S.kl_irev = i_rev;
}

/* Start the pending request: ring (measured stroke) or learned template, the prior when kL is
 * undefined for the requested source. The ring is read in place over the run (<= 7 calls): a step
 * landing meanwhile replaces one entry by the same phase of the next revolution. */
static void kl_begin(void)
{
    bool ok = false;
    S.kl_pending = false;
    S.kl_active = true;
    S.kl_iters = 0u;
    S.kl_run_src = S.kl_req_src;
    if (S.kl_run_src == KL_SRC_RING && !ring_usable()) S.kl_run_src = KL_SRC_TPL;
    if (S.kl_run_src == KL_SRC_RING)
        ok = klc_start(&S.klc, S.ring, SPR, true, ring_mean(S.ring), S.kl_req_cad, S.kl_req_thr);
    else if (S.kl_run_src == KL_SRC_TPL)
        ok = klc_start(&S.klc, S.s, NB, false, S.kl_req_irev, S.kl_req_cad, S.kl_req_thr);
    if (!ok) {
        S.kl_run_src = KL_SRC_PRIOR;   /* stroke never above threshold: kL undefined */
        ok = klc_start(&S.klc, PRIOR, NB, false, P.kl_prior_ref_clu, S.kl_req_cad, S.kl_req_thr);
    }
    if (!ok) {
        /* the prior never undefined in practice; keep the floor */
        S.kl_active = false;
        S.kl = P.kl_min_q12;
        S.kl_src = KL_SRC_PRIOR;
        D.last_kl_iterations = 0u;
        D.kl_recomputes++;
    }
}

/* One bounded chunk of the active kL run; applies the result when the run completes. */
static void kl_step(uint32_t max_iter)
{
    bool done = klc_run(&S.klc, max_iter);
    uint16_t ess = 0u;
    if (!done) return;
    S.kl_iters += S.klc.cnt;
    S.kl_active = false;
    if (S.kl_run_src == KL_SRC_RING && !ring_usable()) {
        /* the ring changed under the run: discard, re-run the same request from the template */
        if (!S.kl_pending) kl_request(KL_SRC_TPL, S.kl_req_irev, S.kl_req_cad, S.kl_req_thr);
        return;
    }
    S.kl = klc_result(&S.klc, &ess);
    S.kl_src = S.kl_run_src;   /* the source of the value now in force (set with the value) */
    D.env_ss = ess;
    D.last_kl_iterations = S.kl_iters;
    if (S.kl_iters > D.max_kl_iterations) D.max_kl_iterations = S.kl_iters;
    D.kl_recomputes++;
}

/* Synchronous evaluation (power-on only: not on the control path). */
static void kl_now(uint8_t src, uint16_t i_rev, uint16_t cad, uint16_t thr)
{
    kl_request(src, i_rev, cad, thr);
    kl_begin();
    while (S.kl_active) kl_step(UINT32_MAX);
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
 * resolution, SPREAD over calls (D-039): ALIGN_INIT computes the bin sums of offset 0 once
 * (96 reads); every ALIGN call evaluates ALIGN_OFFSETS_PER_CALL offsets in increasing order
 * (NB MACs each) and slides the bin sums to the next offset (bin_sum(b, off + 1) = bin_sum(b, off)
 * + ring[start - 1] - ring[start + 3], start = 4b - off mod 96). Same 96 offsets, same order and
 * tie rule (first maximum) as the one-shot search. */
static void align_init(void)
{
    for (uint8_t b = 0u; b < NB; ++b) S.bs[b] = (uint16_t)bin_sum(b, 0u);
    S.job_off = 0u;
    S.job_best = 0u;
    S.job_cmax = 0u;
    S.job_cmin = UINT64_MAX;
    D.work_this_call += SPR;
}

static bool align_chunk(void)
{
    for (uint8_t n = 0u; n < ALIGN_OFFSETS_PER_CALL && S.job_off < SPR; ++n) {
        const uint8_t off = S.job_off;
        uint64_t c = 0u;
        for (uint8_t b = 0u; b < NB; ++b) c += (uint64_t)S.bs[b] * S.s[b];
        if (c > S.job_cmax) { S.job_cmax = c; S.job_best = off; }
        if (c < S.job_cmin) S.job_cmin = c;
        S.job_off = (uint8_t)(off + 1u);
        D.work_this_call += NB;
        if (S.job_off >= SPR) break;
        for (uint8_t b = 0u; b < NB; ++b) {
            /* window of bin b at `off` starts at raw (b*SPB - off) mod 96 */
            const uint32_t start = ((uint32_t)b * SPB + SPR - off) % SPR;
            const uint32_t in_i = start == 0u ? SPR - 1u : start - 1u;
            const uint32_t out_i = (start + SPB - 1u) % SPR;
            /* int32 and clamped: a step that landed meanwhile can make the sum inconsistent */
            int32_t v = (int32_t)S.bs[b] + (int32_t)S.ring[in_i] - (int32_t)S.ring[out_i];
            S.bs[b] = (uint16_t)(v < 0 ? 0 : (v > 65535 ? 65535 : v));
        }
        D.work_this_call += 2u * NB;
    }
    return S.job_off >= SPR;
}

static void align_decide(void)
{
    const uint64_t cmax = S.job_cmax, cmin = S.job_cmin;
    if (cmax > 0u && (cmax - cmin) * Q12 >= (uint64_t)P.align_contrast_q12 * cmax) {
        S.tpl_offset = S.job_best;
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

/* Residual + learning in ONE pass over the bins (the residual of bin b uses s[b] before its own
 * update, exactly as the two-pass form did), then renormalisation. One job unit. */
static void learn_pass(void)
{
    const uint16_t i_rev = S.job_irev;
    uint32_t res = 0u;
    uint8_t sh = S.conf < P.c_min_q12 ? P.alpha_shift_low_conf : P.alpha_shift;
    for (uint8_t b = 0u; b < NB; ++b) {
        const uint32_t r = bin_ratio(b, i_rev);
        res += absdiff(r, S.s[b]);
        if (S.job_stable) {
            int32_t d = (int32_t)r - (int32_t)S.s[b];
            int32_t step = d >= 0 ? (d >> sh) : -((-d) >> sh);
            int32_t v = (int32_t)S.s[b] + step;
            S.s[b] = (uint16_t)(v < 0 ? 0 : v);
        }
    }
    D.work_this_call += (uint32_t)NB * (SPB + 2u);
    res /= NB;
    D.last_residual_q12 = sat16(res);
    if (S.job_stable) {
        if (res > P.res_mismatch_q12) {
            if (S.conf > P.c_mismatch_q12) { S.conf = P.c_mismatch_q12; D.mismatch_drops++; }
        } else if (res < P.res_good_q12) {
            uint32_t c = (uint32_t)S.conf + P.c_step_q12;
            S.conf = (uint16_t)(c > Q12 ? Q12 : c);
        } else {
            S.conf = S.conf > P.c_fall_q12 ? (uint16_t)(S.conf - P.c_fall_q12) : 0u;
        }
        renormalise();
        D.work_this_call += 2u * NB;
        if (S.revs_learned < 65535u) S.revs_learned++;
    }
}

/* A revolution completed (inside a step): only bookkeeping here; the work is queued (D-039). A job
 * still running from the previous revolution is superseded by the newer one. */
static void revolution_complete(const assist_v3_intent_in_t *in)
{
    uint16_t i_rev = (uint16_t)(S.sum96 / SPR);
    bool valid = !S.rev_invalid && i_rev >= P.learn_min_clu;
    bool stable = S.prev_rev_valid && S.prev_rev_mean > 0u &&
                  absdiff(i_rev, S.prev_rev_mean) * Q12 <= (uint32_t)P.stable_q12 * S.prev_rev_mean;

    D.last_rev_mean = i_rev;
    S.job_irev = i_rev;
    S.job_valid = valid;
    S.job_stable = stable;
    /* The measured stroke is used only when the SHAPE is stable too: a transition at the end of
     * the revolution (one heavy attack step in a light stroke) leaves the mean inside 15 % but
     * moves the peak - and the peak is what the envelope holds (N1, attack_stop rows). */
    const bool peak_stable = S.prev_rev_peak > 0u &&
        absdiff(S.rev_peak, S.prev_rev_peak) * Q12 <= (uint32_t)P.stable_q12 * S.prev_rev_peak;
    S.job_clean = valid && stable && peak_stable && i_rev >= P.kl_floor_clu && ring_clean();
    if (valid && stable && peak_stable && i_rev > 0u)
        S.shape_q12 = sat16(((uint32_t)S.rev_peak * Q12) / i_rev);
    S.prev_rev_peak = !S.rev_invalid ? S.rev_peak : 0u;
    S.rev_peak = 0u;
    S.ring_stable = S.job_clean;
    S.ring_dirty = false;
    S.job_cad = (uint16_t)(in->cadence_rpm > 0 ? in->cadence_rpm : 0);
    S.job_thr = active_thr(in);
    S.job = valid ? (S.aligned ? JOB_LEARN : JOB_ALIGN_INIT) : JOB_KL;
    S.prev_rev_mean = i_rev;
    S.prev_rev_valid = !S.rev_invalid;
    S.rev_invalid = false;
    S.rev_count = 0u;
}

/* At most ONE bounded unit of deferred work per call (D-039). An active kL run is finished before a
 * learning pass may change the template it reads. */
static void run_deferred(void)
{
    if (S.kl_active) { kl_step(KL_ITER_PER_CALL); return; }
    switch (S.job) {
    case JOB_ALIGN_INIT:
        align_init();
        S.job = JOB_ALIGN;
        return;
    case JOB_ALIGN:
        if (!align_chunk()) return;
        align_decide();
        S.job = S.aligned ? JOB_LEARN : JOB_KL;
        return;
    case JOB_LEARN:
        learn_pass();
        S.job = JOB_KL;
        return;
    case JOB_KL:
        /* kL (section 4.3): learned template when the revolution is stable and above the floor,
         * otherwise the prior's kL at this cadence. */
        S.job = JOB_IDLE;
        if (S.job_cad > 0u) {
            /* the measured stroke of this clean revolution; the learned template when the ring
             * is not one clean revolution (glitch / restart / invalid torque inside it). A
             * revolution that is neither (attack / release transition, light spinning) KEEPS the
             * measured or learned kL in force: it still describes this rider's stroke, and falling
             * back to the prior there made the level jump at every transition (rework). Only while
             * nothing measured exists yet is the prior refreshed. */
            const bool use_learned = S.job_valid && S.job_stable && S.job_irev >= P.kl_floor_clu;
            if (S.job_clean) kl_request(KL_SRC_RING, S.job_irev, S.job_cad, S.job_thr);
            else if (use_learned) kl_request(KL_SRC_TPL, S.job_irev, S.job_cad, S.job_thr);
            else if (S.kl_src == KL_SRC_PRIOR) kl_request(KL_SRC_PRIOR, S.job_irev, S.job_cad, S.job_thr);
        }
        break;
    default:
        break;
    }
    if (S.kl_pending) { kl_begin(); if (S.kl_active) kl_step(KL_ITER_PER_CALL); }
}

/* ------------------------------------------------------------------ phase tracker */

static void restart(void)
{
    S.steps_since_restart = 0u;
    S.clean_steps = 0u;
    S.peak_restart = 0u;
    S.shape_early_q12 = 0u;
    S.ring_stable = false;
    S.rev_peak = 0u;
    S.prev_rev_peak = 0u;
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
    S.clean_steps = 0u;
    /* a queued alignment/learning of the pre-glitch revolution must not re-align the phase */
    if (S.job == JOB_ALIGN_INIT || S.job == JOB_ALIGN || S.job == JOB_LEARN) S.job = JOB_KL;
}

/* Ring + counters for one forward step. */
static void push_step(uint16_t obs, const assist_v3_intent_in_t *in)
{
    S.raw_phase = (uint8_t)((S.raw_phase + 1u) % SPR);
    if (S.steps_since_restart >= SPR) {
        const uint32_t old = S.ring[S.raw_phase];
        const uint32_t ref = S.prev_rev_mean > P.learn_min_clu ? S.prev_rev_mean : P.learn_min_clu;
        if (absdiff(obs, old) * Q12 > (uint32_t)P.stable_q12 * ref) S.ring_dirty = true;
        S.sum96 -= old;
    }
    S.ring[S.raw_phase] = obs;
    S.sum96 += obs;
    if (S.clean_steps < 65535u) S.clean_steps++;
    if (obs > S.rev_peak) S.rev_peak = obs;
    if (obs > S.peak_restart) S.peak_restart = obs;
    if (S.steps_since_restart < 65535u) S.steps_since_restart++;
    if (S.dwell < 255u) S.dwell++;
    D.total_steps++;
    D.work_this_call++;
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
    D.work_this_call += wmax;
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
        /* Less than one revolution since the restart: a plain mean is biased by WHERE in the
         * stroke the window started (first stroke peak -> over-estimate, dead spot -> late start).
         * While the phase is aligned, normalise by the expected effort of those steps
         * (E = sum obs / sum s, section 4.3) - the restart rework; unaligned: plain means. */
        if (avail >= SPR) e_l = S.sum96 / SPR;
        else if (S.aligned && ss > 0u) e_l = e48t;
        else e_l = avail >= P.w_max_steps ? mean48 : S.sum96 / avail;
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
    /* early stroke shape (prior phase only): steady windows only - NORMAL class with the short and
     * long estimates agreeing. A release window (falling mean, old peak still inside) or an attack
     * window must never raise it: that would hold the envelope-equivalent up during a release. */
    if (S.shape_q12 == 0u && avail >= P.w_max_steps && S.class_state == CLS_NORMAL &&
        mean48 >= P.learn_min_clu && absdiff(e_s, e_l) * Q12 <= 205u * e_l) {   /* within 5 % */
        const uint32_t sh = (mo * Q12) / mean48;
        if (sh > S.shape_early_q12) S.shape_early_q12 = sat16(sh);
    }
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
    kl_now(KL_SRC_PRIOR, 0u, 60u, (uint16_t)(750u + P.eb74_deadband));
    S.kl_cad = 0u;      /* first real cadence triggers a recompute */
    D.kl_recomputes = 0u;
    D.max_kl_iterations = 0u;
}

void assist_v3_intent_reset(void)
{
    uint16_t s[NB];
    uint16_t conf = S.conf, revs = S.revs_learned, kl = S.kl, irev = S.kl_irev, thr = S.kl_thr;
    uint8_t klsrc = S.kl_src;
    uint8_t off = S.tpl_offset;
    memcpy(s, S.s, sizeof(s));
    memset(&S, 0, sizeof(S));
    memcpy(S.s, s, sizeof(s));
    S.conf = conf;
    S.revs_learned = revs;
    S.tpl_offset = off;
    S.aligned = false;  /* steps may have been missed: re-align (confidence kept for the restore) */
    S.kl = kl;
    S.kl_src = klsrc;
    S.kl_irev = irev;
    S.kl_thr = thr;
    S.kl_cad = 0u;      /* next cadence recomputes kL from the kept source */
    /* the memset above dropped any queued revolution job and kL run (stale ride data) */
    restart();
    S.stopped = true;
}

/* kL in force for env_equiv. While it comes from the PRIOR (no measured / learned stroke yet, the
 * first revolutions after power-on), the prior's kL is scaled by the measured peak / mean of the last
 * STABLE revolution (mean and peak within 15 % of the one before; an attack revolution never sets it)
 * over the prior's own peak / mean (1.51): kL grows roughly with the stroke's
 * peakiness, so a deep dead spot is not under-assisted by -30 % until the first stable revolution
 * and the hand-over to the measured-ring kL is not a step (G1-LEVEL prior case, rework). */
static uint16_t effective_kl(void)
{
    uint32_t k = S.kl;
    const uint32_t shape_q12 = S.shape_q12 ? S.shape_q12 : S.shape_early_q12;
    if (S.kl_src == KL_SRC_PRIOR && shape_q12 > 0u) {
        uint16_t prior_peak = PRIOR[0];
        for (uint8_t b = 1u; b < NB; ++b) if (PRIOR[b] > prior_peak) prior_peak = PRIOR[b];
        /* three quarters of the shape excess: kL is not proportional to peak / mean (an
         * asymmetric stroke has a high peak / mean but the envelope decays across the weak leg),
         * and a 180 deg window estimate is uncertain by +-25 %; 3/4 keeps the prior phase inside
         * the G1-LEVEL +-15 % band on the dead-spot and the asymmetric profiles (matrix sweep of
         * 1/2, 3/4, 1, rework). */
        k = (k * (prior_peak + 3u * shape_q12)) / (4u * prior_peak);
        if (k < P.kl_min_q12) k = P.kl_min_q12;
        if (k > P.kl_max_q12) k = P.kl_max_q12;
    }
    return (uint16_t)k;
}

/* Source of a cadence / threshold triggered recompute: the kept source, the ring only while it holds
 * one clean revolution (else the learned template describes the same rider). */
static uint8_t trigger_src(void)
{
    return (S.kl_src == KL_SRC_RING && !ring_usable()) ? KL_SRC_TPL : S.kl_src;
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
    bool deferred_done = false;

    D.work_this_call = 0u;

    /* load accumulation (mean per step; older load is forgotten beyond ~400 ms) */
    S.load_sum += (uint32_t)load * el_acc;
    S.load_ticks += el_acc;
    while (S.load_ticks > P.t_stop_max_ticks) { S.load_sum >>= 1; S.load_ticks >>= 1; }
    if (!in->torque_valid) { S.rev_invalid = true; S.conf = 0u; S.clean_steps = 0u; }

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
        const uint32_t back = (uint32_t)(-delta);
        /* the phase moves backwards and survives either way */
        S.raw_phase = (uint8_t)(((int32_t)S.raw_phase + delta + (int32_t)SPR) % (int32_t)SPR);
        S.rev_run = (uint8_t)(S.rev_run + back > 255u ? 255u : S.rev_run + back);
        if (S.rev_run >= 2u) {
            /* back-pedalling (two or more reverse steps in a row): intent restarts afterwards */
            restart();
            S.stopped = true;
            S.load_sum = 0u; S.load_ticks = 0u;
            S.have_step_tick = false;
        } else {
            /* ONE reverse step inside forward pedalling is PAS jitter (A-B-A): the intent, the
             * classification and the trajectory are kept - a glitch never lowers intent (rework,
             * pas_glitch rows). Only this revolution is not learned from. */
            S.rev_invalid = true;
        }
    } else if (delta > 0) {
        S.rev_run = 0u;
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
        /* the deferred unit runs BEFORE the classification of a stepping call, so a learning pass
         * queued by the step that completed a revolution is seen by that same classification */
        run_deferred();
        deferred_done = true;
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
        if (cad_moved || thr != S.kl_thr) kl_request(trigger_src(), S.kl_irev, cad, thr);
    } else if (thr != S.kl_thr) {
        kl_request(trigger_src(), S.kl_irev, S.kl_cad ? S.kl_cad : P.kl_cad_min_rpm, thr);
    }

    /* one bounded unit of the per-revolution / kL work per call (D-039) */
    if (!deferred_done) run_deferred();
    if (D.work_this_call > D.max_work_per_call) D.max_work_per_call = D.work_this_call;

    if (out) {
        uint8_t tp = tpl_phase_of(S.raw_phase);
        const uint16_t kl_eff = effective_kl();
        uint32_t keq = ((uint32_t)kl_eff * S.intent) >> 12;
        /* Engage parity (G1-START): a kL that is not measured on this ride's stroke (prior /
         * template) cannot lift the envelope-equivalent load above the largest per-step load seen
         * since the restart - the D7EC envelope never exceeds the EB74 of the loads it saw. A flat
         * light load therefore engages exactly when EB74 of that load would. */
        if (S.kl_src != KL_SRC_RING && keq > S.peak_restart) keq = S.peak_restart;
        memset(out, 0, sizeof(*out));
        out->intent = S.intent;
        out->e_short = S.e_short;
        out->e_long = S.e_long;
        out->env_equiv = assist_v3_eb74_active(keq, thr);
        out->kl_q12 = kl_eff;
        out->confidence_q12 = S.conf;
        out->expected_effort = sat16(((uint32_t)S.intent * S.s[tp / SPB]) >> 12);
        out->revolutions_learned = S.revs_learned;
        out->steps_since_restart = S.steps_since_restart;
        out->release_class = S.reported_class;
        out->phase = tp;
        out->phase_aligned = S.aligned;
        out->template_mode = S.template_mode;
        out->kl_from_prior = S.kl_src == KL_SRC_PRIOR;
        out->engage_ok = out->env_equiv > 0u && in->eb74_armed;
        out->reverse_step = delta < 0 && S.rev_run >= 2u;
        out->reverse_any = delta < 0;
        out->forward_step = forward;
    }
    D.tpl_offset = S.tpl_offset;
    D.class_state = S.class_state;
}

const assist_v3_intent_params_t *assist_v3_intent_params(void) { return &P; }
const assist_v3_intent_debug_t *assist_v3_intent_debug(void) { return &D; }
const uint16_t *assist_v3_intent_template(void) { return S.s; }
const uint16_t *assist_v3_intent_ring(void) { return S.ring; }
uint8_t assist_v3_intent_kl_source(void) { return S.kl_src; }
const uint16_t *assist_v3_intent_prior(void) { return PRIOR; }
