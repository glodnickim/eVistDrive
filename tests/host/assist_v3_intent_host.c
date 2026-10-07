/*
 * Assist Behavior V3 - Rider Intent V1 host tests (real module: src/assist_v3_intent.c).
 *
 * Stimulus: a crank-angle driven rider (this file) whose torque shape is the shared SIL law
 * rider_script_shape() (tests/host/common/rider_script.c), so the profiles are the ones the V3
 * matrix uses. Load is fed as CLU 1:1 from the scripted centi-kg (the module is unit-agnostic except
 * for the EB74 transfer). Steps are counted from the crank angle (96 per revolution); glitches are
 * injected as lost steps + pas_glitch, as crank_phase.c will report them.
 *
 * Rows covered (docs/assist-v3/TEST_MATRIX.md): G1-PHASE, G1-TPL, G1-CLS, G1-FALLBACK, G1-STYLE,
 * G1-OSC, G1-PAS, G1-IMU (determinism part; the sanitiser is assist_motion_host.c), env_equiv/kL
 * (float reference of the D7EC recurrence, clamps, N1 light spin -> attack), CPU sanity.
 * Float is used ONLY here, never in the module. Printed numbers are [SIM] observations.
 */
#include "assist_v3_intent.h"
#include "common/check.h"
#include "common/rider_script.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NBINS ASSIST_V3_NB
#define SPBIN ASSIST_V3_STEPS_PER_BIN
#define TICK_HZ 4000.0
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------------ shapes */

static rider_script_t make_shape_ph(double ripple, double asym, double dsdepth, double dswidth, double phase)
{
    rider_script_t sc;
    char txt[256], err[128];
    snprintf(txt, sizeof(txt),
             "name s\nshape ripple=%g asym=%g dsdepth=%g dswidth=%g phase=%g\nseg steady dur=1 rpm=60 mean=1000\n",
             ripple, asym, dsdepth, dswidth, phase);
    if (rider_script_parse_text(txt, &sc, err, sizeof(err)) != 0) {
        printf("shape parse failed: %s\n", err);
        exit(2);
    }
    return sc;
}

static rider_script_t make_shape(double ripple, double asym, double dsdepth, double dswidth)
{
    return make_shape_ph(ripple, asym, dsdepth, dswidth, 0.0);
}

static rider_script_t SH_STEADY, SH_DEAD, SH_DEEP, SH_ASYM, SH_SIT, SH_STAND, SH_ASYM40, SH_DEAD90;

/* ------------------------------------------------------------------ crank-angle rider */

typedef struct {
    const rider_script_t *shape;
    double rpm, mean, noise;
    uint32_t rng;
    double cum_deg;
    int32_t true_pos;          /* physical steps */
    int32_t lost;              /* steps the observed counter missed */
    int32_t extra;             /* one-shot extra observed steps (overflow > 96 test) */
    uint32_t step_tick, tick;
    bool glitch, g53_stop, real_stop, inhibit_rev, torque_invalid;
    int engaged_mode;          /* 0: engaged = env_equiv > 0 (trajectory emulation), 1: true, 2: false */
    double clean;
    assist_v3_intent_out_t out;
} sim_t;

static double gauss(uint32_t *st)
{
    double u1, u2;
    do {
        *st ^= *st << 13; *st ^= *st >> 17; *st ^= *st << 5;
        u1 = (double)(*st >> 8) / 16777216.0;
    } while (u1 <= 1e-12);
    *st ^= *st << 13; *st ^= *st >> 17; *st ^= *st << 5;
    u2 = (double)(*st >> 8) / 16777216.0;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static void sim_init(sim_t *s, const rider_script_t *shape, double rpm, double mean, double noise, uint32_t seed)
{
    memset(s, 0, sizeof(*s));
    s->shape = shape; s->rpm = rpm; s->mean = mean; s->noise = noise;
    s->rng = seed ? seed : 1u;
    s->tick = 1000u;
    assist_v3_intent_power_on();
}

static int32_t obs_steps(const sim_t *s)
{
    return (int32_t)((uint32_t)s->true_pos - (uint32_t)s->lost + (uint32_t)s->extra);
}

static void sim_tick(sim_t *s)
{
    assist_v3_intent_in_t in;
    double t;
    int32_t pos;
    s->tick++;
    s->cum_deg += s->rpm * 6.0 / TICK_HZ;
    pos = (int32_t)floor(s->cum_deg / 3.75 + 1e-9);
    if (pos != s->true_pos) { s->true_pos = pos; s->step_tick = s->tick; }
    s->clean = s->mean > 0.0 ? s->mean * rider_script_shape(s->shape, s->cum_deg) : 0.0;
    t = s->clean + (s->noise > 0.0 ? s->noise * gauss(&s->rng) : 0.0);
    if (t < 0.0) t = 0.0;
    if (t > 60000.0) t = 60000.0;
    memset(&in, 0, sizeof(in));
    in.elapsed_ticks = 1u;
    in.now_tick = s->tick;
    in.load_ctrl = (uint16_t)lround(t);
    in.torque_valid = !s->torque_invalid;
    in.crank_steps = obs_steps(s);
    in.crank_step_tick = s->step_tick;
    in.pas_glitch = s->glitch;
    s->glitch = false;
    in.cadence_rpm = (int16_t)lround(s->rpm);
    in.g53_true_stop = s->g53_stop;
    in.real_stop = s->real_stop;
    in.direction_inhibit = s->inhibit_rev;
    in.inhibit_is_reverse = s->inhibit_rev;
    in.eb74_zero = 750u;
    in.eb74_armed = true;
    in.v3_engaged = s->engaged_mode == 1 ? true : s->engaged_mode == 2 ? false : (s->out.env_equiv > 0u);
    assist_v3_intent_update(&in, &s->out);
}

static void sim_seconds(sim_t *s, double sec)
{
    uint32_t n = (uint32_t)(sec * TICK_HZ);
    for (uint32_t i = 0; i < n; ++i) sim_tick(s);
}

static void sim_revs(sim_t *s, double revs)
{
    double target = s->cum_deg + revs * 360.0 * (s->rpm >= 0 ? 1.0 : -1.0);
    if (s->rpm == 0.0) return;
    while ((s->rpm > 0 && s->cum_deg < target) || (s->rpm < 0 && s->cum_deg > target)) sim_tick(s);
}

/* advance until the crank angle (mod 360) crosses phase_deg */
static void sim_to_phase(sim_t *s, double phase_deg)
{
    double base = floor(s->cum_deg / 360.0) * 360.0 + phase_deg;
    if (base <= s->cum_deg) base += 360.0;
    while (s->cum_deg < base) sim_tick(s);
}

static void sim_drop(sim_t *s, int32_t n) { s->lost += n; s->glitch = true; }

/* ------------------------------------------------------------------ references */

/* Expected template of a shape for the module's current offset: template phase tp maps to the
 * observed step count k = tp - off (mod 96); the observation of step k is the load over the crank
 * interval [(k_phys-1)*3.75, k_phys*3.75) with k_phys = k + lost. */
static void expected_template(const rider_script_t *sh, int off, int lost, double e[NBINS])
{
    double tot = 0.0;
    for (int b = 0; b < NBINS; ++b) {
        double sum = 0.0;
        for (int j = 0; j < (int)SPBIN; ++j) {
            int tp = b * (int)SPBIN + j;
            int k = ((tp - off) % 96 + 96) % 96;
            double a0 = ((double)(k + lost) - 1.0) * 3.75;
            for (int q = 0; q < 32; ++q) sum += rider_script_shape(sh, a0 + (q + 0.5) * 3.75 / 32.0);
        }
        e[b] = sum / (SPBIN * 32.0);
        tot += e[b];
    }
    for (int b = 0; b < NBINS; ++b) e[b] *= NBINS / tot;
}

static double template_err(const rider_script_t *sh, int lost)
{
    double e[NBINS], m = 0.0;
    const uint16_t *s = assist_v3_intent_template();
    expected_template(sh, assist_v3_intent_debug()->tpl_offset, lost, e);
    for (int b = 0; b < NBINS; ++b) {
        double d = fabs(s[b] / 4096.0 - e[b]);
        if (d > m) m = d;
    }
    return m;
}

static uint32_t template_sum(void)
{
    uint32_t sum = 0;
    for (int b = 0; b < NBINS; ++b) sum += assist_v3_intent_template()[b];
    return sum;
}

/* Long time-domain run of the D7EC recurrence (k = 8*cad, attack env = x, else env*k/(k+1), one
 * iteration per 10 ms) over the same bin-reconstructed stroke, float state; floor_mode applies the
 * integer truncation of udiv. Starts from env = 0 at phase 0 and averages the second half. */
static double float_env_ss(const uint16_t *xb_src, double cad, int floor_mode)
{
    double k = 8.0 * cad, env = 0.0, sum = 0.0, phase = 0.0;
    long n = (long)(300.0 * 6000.0 / cad), cnt = 0;
    for (long i = 0; i < n; ++i) {
        int b = (int)floor(phase * NBINS) % NBINS;
        double x = xb_src[b];
        if (x > env) env = x;
        else env = floor_mode ? floor(env * k / (k + 1.0)) : env * k / (k + 1.0);
        if (i >= n / 2) { sum += env; cnt++; }
        phase += cad / 6000.0;
        if (phase >= 1.0) phase -= 1.0;
    }
    return sum / (double)cnt;
}

static double float_kl(const uint16_t *s, double irev, double cad, unsigned thr, int floor_mode, double *ess)
{
    uint16_t xb[NBINS];
    double e, leq, kl;
    if (cad < 15.0) cad = 15.0;
    for (int b = 0; b < NBINS; ++b) xb[b] = assist_v3_eb74_active(((uint32_t)irev * s[b]) >> 12, (uint16_t)thr);
    e = float_env_ss(xb, cad, floor_mode);
    if (ess) *ess = e;
    leq = (e + thr - 750.0) * 6000.0 / 2450.0;
    kl = leq / irev;
    return kl < 1.0 ? 1.0 : kl > 2.5 ? 2.5 : kl;
}

/* Expected template as an integer Q12 table (for kL references with a "learned" shape). */
static void shape_q12(const rider_script_t *sh, uint16_t q[NBINS])
{
    double e[NBINS];
    long sum = 0;
    expected_template(sh, 0, 0, e);
    for (int b = 0; b < NBINS; ++b) { q[b] = (uint16_t)lround(e[b] * 4096.0); sum += q[b]; }
    q[NBINS / 4] = (uint16_t)(q[NBINS / 4] + (NBINS * 4096 - sum));
}

/* ------------------------------------------------------------------ observers */

typedef struct {
    uint32_t ticks, rel, att, dip, stopc, tmode, transitions, tmode_toggles, gt_dip, gt_dip_rel;
    uint16_t imin, imax;
    uint8_t last_state;
    bool last_tmode, first;
} obs_t;

static void obs_reset(obs_t *o) { memset(o, 0, sizeof(*o)); o->imin = 65535u; o->first = true; }

static void obs_take(obs_t *o, const sim_t *s)
{
    const assist_v3_intent_debug_t *d = assist_v3_intent_debug();
    o->ticks++;
    if (s->out.release_class == ASSIST_V3_CLASS_TRUE_RELEASE) o->rel++;
    if (s->out.release_class == ASSIST_V3_CLASS_ATTACK) o->att++;
    if (s->out.release_class == ASSIST_V3_CLASS_PHASE_DIP) o->dip++;
    if (s->out.release_class == ASSIST_V3_CLASS_PEDAL_STOP) o->stopc++;
    if (s->out.template_mode) o->tmode++;
    if (s->mean > 0.0 && s->clean < 0.5 * s->mean) {
        o->gt_dip++;
        if (s->out.release_class == ASSIST_V3_CLASS_TRUE_RELEASE) o->gt_dip_rel++;
    }
    if (s->out.intent < o->imin) o->imin = s->out.intent;
    if (s->out.intent > o->imax) o->imax = s->out.intent;
    if (!o->first) {
        if (d->class_state != o->last_state) o->transitions++;
        if (s->out.template_mode != o->last_tmode) o->tmode_toggles++;
    }
    o->first = false;
    o->last_state = d->class_state;
    o->last_tmode = s->out.template_mode;
}

static void run_obs_revs(sim_t *s, obs_t *o, double revs)
{
    double target = s->cum_deg + revs * 360.0;
    while (s->cum_deg < target) { sim_tick(s); obs_take(o, s); }
}

/* ------------------------------------------------------------------ G1-PHASE */

static void test_phase(void)
{
    sim_t s;
    int ok_rel = 1, ok_cont = 1;
    int32_t prev_obs;
    uint8_t prev_phase;
    const assist_v3_intent_debug_t *d = assist_v3_intent_debug();

    sim_init(&s, &SH_ASYM, 60.0, 1000.0, 10.0, 11u);
    sim_revs(&s, 12.0);
    CHECK(s.out.phase_aligned, "G1-PHASE aligned after the first revolutions");
    {
        uint8_t off0 = d->tpl_offset;
        /* mod-96 relation + continuity through stop, back-pedal, restart */
        prev_obs = obs_steps(&s);
        prev_phase = s.out.phase;
        for (int stage = 0; stage < 4; ++stage) {
            if (stage == 0) { s.rpm = 0.0; s.mean = 0.0; }
            if (stage == 1) { s.rpm = -30.0; s.mean = 200.0; }
            if (stage == 2) { s.rpm = 0.0; s.mean = 0.0; }
            if (stage == 3) { s.rpm = 60.0; s.mean = 1000.0; }
            for (int i = 0; i < 6000; ++i) {
                int32_t o = obs_steps(&s);
                int dphase, dsteps;
                sim_tick(&s);
                o = obs_steps(&s);
                if (s.out.phase != (uint8_t)(((o % 96) + 96 + d->tpl_offset) % 96)) ok_rel = 0;
                dsteps = (int)(o - prev_obs);
                dphase = ((int)s.out.phase - (int)prev_phase + 96 + 48) % 96 - 48;
                if (dphase != dsteps) ok_cont = 0;
                prev_obs = o;
                prev_phase = s.out.phase;
            }
            if (stage == 1) CHECK(s.out.release_class == ASSIST_V3_CLASS_PEDAL_STOP, "G1-PHASE back-pedal reports PEDAL_STOP");
        }
        CHECK(ok_rel, "G1-PHASE phase == (steps + offset) mod 96 through stop/back-pedal/restart");
        CHECK(ok_cont, "G1-PHASE phase moves exactly with the signed step count (no jumps)");
        CHECK(d->tpl_offset == off0 && s.out.phase_aligned, "G1-PHASE alignment survives stop and back-pedal");
        sim_revs(&s, 6.0);
        CHECK(s.out.template_mode, "G1-PHASE template mode resumes after restart");
    }

    /* glitch: 2 lost steps -> unaligned -> re-aligned after one revolution, offset +2 */
    for (int shape = 0; shape < 2; ++shape) {
        const rider_script_t *sh = shape ? &SH_ASYM : &SH_DEAD;
        int modv = shape ? 96 : 48;   /* symmetric legs: a 180 deg twin alignment is equivalent */
        sim_init(&s, sh, 60.0, 1000.0, 10.0, 12u + (uint32_t)shape);
        sim_revs(&s, 30.0);
        {
            uint8_t off0 = d->tpl_offset;
            uint16_t c0 = s.out.confidence_q12;
            double revs_to_align = -1.0, start;
            sim_drop(&s, 2);
            sim_tick(&s);
            CHECK(!s.out.phase_aligned && !s.out.template_mode, "G1-PHASE glitch -> unaligned, fallback");
            start = s.cum_deg;
            while (s.cum_deg < start + 3.0 * 360.0) {
                sim_tick(&s);
                if (s.out.phase_aligned && revs_to_align < 0) revs_to_align = (s.cum_deg - start) / 360.0;
            }
            {
                int shift = ((int)d->tpl_offset - (int)off0 - 2 + 960) % modv;
                if (shift > modv / 2) shift -= modv;
                printf("  G1-PHASE glitch (%s): realigned after %.2f rev, offset %u -> %u (expected +2 mod %d), conf %u -> %u\n",
                       shape ? "asym" : "sym", revs_to_align, off0, d->tpl_offset, modv, c0, s.out.confidence_q12);
                CHECK(revs_to_align > 0 && revs_to_align <= 2.05, "G1-PHASE re-aligned within one full revolution after the glitch");
                CHECK(abs(shift) <= 1, "G1-PHASE re-alignment restores the physical phase (+2 steps)");
                CHECK(s.out.confidence_q12 >= c0 - 512 && s.out.template_mode, "G1-PHASE confidence restored, template mode back");
            }
        }
    }
}

/* counter / tick wrap (G1-LONG part): step counter crosses INT32_MAX, tick crosses 2^32 */
static void test_wrap(void)
{
    sim_t s;
    obs_t o;
    int ok_cont = 1;
    int32_t prev_obs;
    uint8_t prev_phase;
    sim_init(&s, &SH_ASYM, 90.0, 1000.0, 20.0, 31u);
    s.tick = 0xFFFFFFFFu - 4000u * 40u;          /* wraps after 40 s */
    s.lost = (int32_t)(0x80000000u + 96u * 60u); /* observed count = true - lost: wraps after ~60 rev */
    sim_revs(&s, 30.0);
    prev_obs = obs_steps(&s);
    prev_phase = s.out.phase;
    obs_reset(&o);
    for (int i = 0; i < 4000 * 50; ++i) {
        int dsteps, dphase;
        sim_tick(&s);
        obs_take(&o, &s);
        dsteps = (int)(int32_t)((uint32_t)obs_steps(&s) - (uint32_t)prev_obs);
        dphase = ((int)s.out.phase - (int)prev_phase + 96 + 48) % 96 - 48;
        if (dphase != dsteps) ok_cont = 0;
        prev_obs = obs_steps(&s);
        prev_phase = s.out.phase;
    }
    CHECK(s.tick < 0x80000000u && obs_steps(&s) < 0, "wrap test: tick counter and step counter wrapped");
    CHECK(ok_cont, "wrap: phase continuous across step-counter and tick wrap");
    CHECK(o.transitions == 0 && o.tmode_toggles == 0 && o.stopc == 0, "wrap: no class/mode change, no false PEDAL_STOP");
    CHECK(s.out.template_mode && s.out.phase_aligned, "wrap: template mode kept");
}

/* ------------------------------------------------------------------ G1-TPL */

static void test_template(void)
{
    static const double cads[] = { 20, 25, 30, 60, 120, 130 };
    const rider_script_t *shapes[] = { &SH_STEADY, &SH_DEAD, &SH_ASYM };
    const char *names[] = { "steady", "deadspot", "asym" };
    sim_t s;
    double worst = 0.0;
    for (int si = 0; si < 3; ++si) {
        printf("  G1-TPL %-8s max|s-s_true| after 40 rev:", names[si]);
        for (unsigned ci = 0; ci < sizeof(cads) / sizeof(cads[0]); ++ci) {
            uint32_t first_tmode = 0;
            obs_t o;
            sim_init(&s, shapes[si], cads[ci], 1000.0, 20.0, 100u + ci);
            obs_reset(&o);
            for (int r = 0; r < 40; ++r) {
                run_obs_revs(&s, &o, 1.0);
                if (!first_tmode && s.out.template_mode) first_tmode = (uint32_t)r + 1u;
            }
            {
                double e = template_err(shapes[si], 0);
                if (e > worst) worst = e;
                printf(" %g:%.3f(tm@%u)", cads[ci], e, first_tmode);
                CHECK(e < 0.10, "G1-TPL template converges within 0.10 of the true stroke");
                CHECK(template_sum() == NBINS * 4096u, "G1-TPL template renormalised (mean exactly 1.0)");
                CHECK(s.out.template_mode && s.out.confidence_q12 >= 3072u, "G1-TPL template mode with high confidence");
                CHECK(o.rel == 0 && o.att == 0, "G1-TPL no override class during convergence");
            }
        }
        printf("\n");
    }
    printf("  G1-TPL worst template error %.3f\n", worst);

    /* fallback when cadence is outside the trusted band or confidence is low */
    sim_init(&s, &SH_DEAD, 30.0, 1000.0, 20.0, 7u);
    sim_revs(&s, 1.5);
    CHECK(!s.out.template_mode, "G1-TPL power-on (prior, confidence 0) -> fallback");
    sim_revs(&s, 20.0);
    CHECK(s.out.template_mode, "G1-TPL converged at 30 rpm -> template mode");
    s.rpm = 12.0;
    sim_revs(&s, 0.3);
    CHECK(!s.out.template_mode, "G1-TPL 12 rpm (below band) -> fallback");
    s.rpm = 160.0;
    sim_revs(&s, 1.0);
    CHECK(!s.out.template_mode, "G1-TPL 160 rpm (above band) -> fallback");
    s.rpm = 60.0;
    sim_revs(&s, 1.0);
    CHECK(s.out.template_mode, "G1-TPL back in band -> template mode");
    s.torque_invalid = true;
    sim_revs(&s, 0.2);
    CHECK(!s.out.template_mode && s.out.confidence_q12 == 0, "G1-TPL torque invalid -> confidence 0, fallback");
    s.torque_invalid = false;
    sim_revs(&s, 6.0);
    CHECK(s.out.template_mode, "G1-TPL confidence rebuilt with the kept template");
}

/* ------------------------------------------------------------------ G1-CLS */

static void test_classifier(void)
{
    static const double cads[] = { 20, 25, 30, 40, 60, 80, 100, 110, 120, 130 };
    sim_t s;
    printf("  G1-CLS TRUE_RELEASE detection angle, template mode, 24 release phases (deg):\n");
    for (unsigned ci = 0; ci < sizeof(cads) / sizeof(cads[0]); ++ci) {
        double maxa = 0, suma = 0, maxobs = 0;
        int n = 0, all_tmode = 1, missed = 0;
        obs_t steady;
        obs_reset(&steady);
        sim_init(&s, &SH_DEAD, cads[ci], 1000.0, 20.0, 300u + ci);
        sim_revs(&s, 30.0);
        for (int k = 0; k < 24; ++k) {
            double ph = k * 15.0 + 7.0, a0, adet = -1, aobs;
            run_obs_revs(&s, &steady, 4.0);          /* steady riding (phase-dip FP is counted) */
            sim_to_phase(&s, ph);
            if (!s.out.template_mode) all_tmode = 0;
            s.mean = 0.0;
            a0 = s.cum_deg;
            /* where the release becomes observable: the first crank angle at which the rider's
             * stroke would have reached the mean effort (shape >= 1.0). Before that point a release
             * cannot be told from a deeper-than-learned dead spot (style change, G1-STYLE); the
             * module waits for a strong-expectation bin by design (s_strong). */
            aobs = a0;
            while (rider_script_shape(&SH_DEAD, aobs) < 1.0) aobs += 0.25;
            while (s.cum_deg < a0 + 2.0 * 360.0) {
                sim_tick(&s);
                if (s.out.release_class == ASSIST_V3_CLASS_TRUE_RELEASE) { adet = s.cum_deg - a0; break; }
            }
            if (adet < 0) { missed++; adet = 720; }
            if (adet > maxa) maxa = adet;
            if (adet - (aobs - a0) > maxobs) maxobs = adet - (aobs - a0);
            suma += adet;
            n++;
            sim_revs(&s, 1.0);
            s.mean = 1000.0;
            sim_revs(&s, 4.0);
        }
        printf("    %3g rpm: max %.1f mean %.1f, max after first strong angle %.1f | steady: PHASE_DIP %u ticks, TRUE_RELEASE %u, gt-dip FP %u/%u, I min %u\n",
               cads[ci], maxa, suma / n, maxobs, steady.dip, steady.rel, steady.gt_dip_rel, steady.gt_dip, steady.imin);
        CHECK(missed == 0, "G1-CLS every release detected");
        CHECK(all_tmode, "G1-CLS releases taken in template mode");
        CHECK(maxobs <= 45.0, "G1-CLS template-mode release detected within 45 deg of becoming observable");
        CHECK(suma / n <= 45.0, "G1-CLS template-mode mean release angle <= 45 deg");
        CHECK(steady.rel == 0 && steady.att == 0, "G1-CLS no override in steady riding (phase-dip FP = 0)");
        CHECK(steady.dip > 0, "G1-CLS dead spots reported as PHASE_DIP");
        CHECK(steady.imin >= 850u, "G1-CLS PHASE_DIP never lowers I (I >= 0.85 mean)");
    }

    printf("  G1-CLS TRUE_RELEASE detection angle, fallback mode (first revolutions after power-on), deg:\n   ");
    for (unsigned ci = 0; ci < sizeof(cads) / sizeof(cads[0]); ++ci) {
        double maxa = 0;
        for (int k = 0; k < 8; ++k) {
            double a0, adet = 720;
            sim_init(&s, &SH_DEAD, cads[ci], 1000.0, 20.0, 400u + ci * 8u + (uint32_t)k);
            sim_revs(&s, 1.6 + k * 0.125);
            CHECK(!s.out.template_mode, "G1-CLS fallback release taken in fallback mode");
            s.mean = 0.0;
            a0 = s.cum_deg;
            while (s.cum_deg < a0 + 720.0) {
                sim_tick(&s);
                if (s.out.release_class == ASSIST_V3_CLASS_TRUE_RELEASE) { adet = s.cum_deg - a0; break; }
            }
            if (adet > maxa) maxa = adet;
        }
        printf(" %g:%.1f", cads[ci], maxa);
        CHECK(maxa <= 180.0, "G1-CLS fallback release detected within 180 deg");
    }
    printf("\n");

    /* ATTACK, template and fallback */
    printf("  G1-CLS ATTACK (600 -> 1200 CLU) detection angle, template | fallback:");
    for (unsigned ci = 0; ci < sizeof(cads) / sizeof(cads[0]); ci += 3) {
        double at = 720, af = 720, a0;
        sim_init(&s, &SH_DEAD, cads[ci], 600.0, 20.0, 500u + ci);
        sim_revs(&s, 30.25);
        CHECK(s.out.template_mode, "G1-CLS attack test starts in template mode");
        s.mean = 1200.0; a0 = s.cum_deg;
        while (s.cum_deg < a0 + 720.0) {
            sim_tick(&s);
            if (s.out.release_class == ASSIST_V3_CLASS_ATTACK) { at = s.cum_deg - a0; break; }
        }
        CHECK(s.out.intent >= 900u, "G1-CLS ATTACK raises I to the short window at once");
        sim_revs(&s, 4.0);
        CHECK(assist_v3_intent_debug()->class_state == 0 && s.out.intent > 1100u, "G1-CLS ATTACK exits to NORMAL at the new level");
        sim_init(&s, &SH_DEAD, cads[ci], 600.0, 20.0, 510u + ci);
        sim_revs(&s, 1.6);
        s.mean = 1200.0; a0 = s.cum_deg;
        while (s.cum_deg < a0 + 720.0) {
            sim_tick(&s);
            if (s.out.release_class == ASSIST_V3_CLASS_ATTACK) { af = s.cum_deg - a0; break; }
        }
        printf(" %g:%.0f|%.0f", cads[ci], at, af);
        CHECK(at <= 45.0, "G1-CLS template ATTACK within 45 deg");
        CHECK(af <= 180.0, "G1-CLS fallback ATTACK within 180 deg");
    }
    printf("\n");

    /* PEDAL_STOP timing (load held) */
    {
        static const double sc[] = { 20, 60, 120 };
        printf("  G1-CLS PEDAL_STOP after the last step (ms), expected clamp(4*625/cad, 60, 400):");
        for (int i = 0; i < 3; ++i) {
            uint32_t t_last, t_det = 0;
            double exp_ms = 4.0 * 625.0 / sc[i];
            if (exp_ms < 60.0) exp_ms = 60.0;
            sim_init(&s, &SH_DEAD, sc[i], 1000.0, 0.0, 600u);
            sim_revs(&s, 5.0);
            s.rpm = 0.0;
            t_last = s.step_tick;
            for (int k = 0; k < 4000; ++k) {
                sim_tick(&s);
                if (s.out.release_class == ASSIST_V3_CLASS_PEDAL_STOP) { t_det = s.tick; break; }
            }
            {
                double ms = (t_det - t_last) / 4.0;
                printf(" %g:%.2f(exp %.1f)", sc[i], ms, exp_ms);
                CHECK(t_det && fabs(ms - exp_ms) <= exp_ms * 0.25 + 1.0, "G1-CLS PEDAL_STOP at T_stop");
                CHECK(s.out.intent >= 850u, "G1-CLS PEDAL_STOP holds the last intent");
            }
            /* G53 true-stop makes it earlier, but never before 60 ms without a step */
            sim_init(&s, &SH_DEAD, sc[i], 1000.0, 0.0, 601u);
            sim_revs(&s, 5.0);
            s.rpm = 0.0; s.g53_stop = true; t_last = s.step_tick; t_det = 0;
            for (int k = 0; k < 4000; ++k) {
                sim_tick(&s);
                if (s.out.release_class == ASSIST_V3_CLASS_PEDAL_STOP) { t_det = s.tick; break; }
            }
            CHECK(t_det && (t_det - t_last) >= 240u && (t_det - t_last) <= 242u, "G1-CLS G53 true-stop -> PEDAL_STOP at 60 ms");
        }
        printf("\n");
    }
}

/* ------------------------------------------------------------------ G1-FALLBACK */

static void test_fallback(void)
{
    const rider_script_t *shapes[] = { &SH_DEEP, &SH_ASYM };
    sim_t s;
    for (int si = 0; si < 2; ++si) {
        obs_t o;
        /* a) after power-on (start) */
        sim_init(&s, shapes[si], 20.0, 1000.0, 20.0, 700u + si);
        obs_reset(&o);
        run_obs_revs(&s, &o, 3.0);
        printf("  G1-FALLBACK %s start: rel %u att %u dip %u template ticks %u\n", si ? "asym" : "deep", o.rel, o.att, o.dip, o.tmode);
        CHECK(o.rel == 0 && o.att == 0, "G1-FALLBACK first 3 rev after start: no TRUE_RELEASE/ATTACK");
        /* b) after reverse */
        sim_revs(&s, 20.0);
        s.rpm = -30.0; s.mean = 200.0; sim_seconds(&s, 1.0);
        s.rpm = 20.0; s.mean = 1000.0;
        obs_reset(&o);
        run_obs_revs(&s, &o, 3.0);
        CHECK(o.rel == 0 && o.att == 0, "G1-FALLBACK first 3 rev after reverse: no TRUE_RELEASE/ATTACK");
        /* c) after a glitch */
        sim_drop(&s, 2);
        obs_reset(&o);
        run_obs_revs(&s, &o, 3.0);
        CHECK(o.rel == 0 && o.att == 0, "G1-FALLBACK first 3 rev after glitch: no TRUE_RELEASE/ATTACK");
        /* d) after a stop */
        s.rpm = 0.0; s.mean = 0.0; sim_seconds(&s, 2.0);
        s.rpm = 20.0; s.mean = 1000.0;
        obs_reset(&o);
        run_obs_revs(&s, &o, 3.0);
        CHECK(o.rel == 0 && o.att == 0, "G1-FALLBACK first 3 rev after stop/restart: no TRUE_RELEASE/ATTACK");
    }
}

/* ------------------------------------------------------------------ G1-STYLE */

static void style_case(const rider_script_t *from, const rider_script_t *to, double cad, const char *name, int expect_drop)
{
    sim_t s;
    obs_t o;
    double a0, fb_rev = -1, tm_rev = -1;
    int dropped = 0;
    sim_init(&s, from, cad, 1000.0, 20.0, 800u + (uint32_t)cad);
    sim_revs(&s, 30.0);
    CHECK(s.out.template_mode, "G1-STYLE converged before the change");
    s.shape = to;
    a0 = s.cum_deg;
    obs_reset(&o);
    while (s.cum_deg < a0 + 40.0 * 360.0) {
        sim_tick(&s);
        obs_take(&o, &s);
        if (!s.out.template_mode && fb_rev < 0) { fb_rev = (s.cum_deg - a0) / 360.0; dropped = 1; }
        if (dropped && s.out.template_mode && tm_rev < 0) tm_rev = (s.cum_deg - a0) / 360.0;
    }
    printf("  G1-STYLE %-12s %3g rpm: I min %u (mean 1000), TRUE_RELEASE %u ATTACK %u ticks, fallback at %.2f rev, template again at %.2f rev, final tpl err %.3f\n",
           name, cad, o.imin, o.rel, o.att, fb_rev, tm_rev, template_err(to, 0));
    if (!expect_drop) {
        CHECK(o.imin >= 850u, "G1-STYLE no intent dip (I >= 0.85 mean)");
        CHECK(o.rel == 0, "G1-STYLE no TRUE_RELEASE on a style change");
    }
    CHECK(fb_rev < 0 || fb_rev <= 2.0, "G1-STYLE fallback within one revolution of the first mismatch revolution");
    if (expect_drop) CHECK(fb_rev > 0 && fb_rev <= 2.0, "G1-STYLE mismatch above threshold drops to fallback within 2 rev (N5)");
    CHECK(s.out.template_mode && template_err(to, 0) < 0.10, "G1-STYLE template re-converges to the new style");
}

static void test_style(void)
{
    style_case(&SH_SIT, &SH_STAND, 20.0, "sit->stand", 0);
    style_case(&SH_SIT, &SH_STAND, 60.0, "sit->stand", 0);
    style_case(&SH_SIT, &SH_STAND, 130.0, "sit->stand", 0);
    style_case(&SH_SIT, &SH_ASYM40, 20.0, "sym->asym40", 0);
    style_case(&SH_SIT, &SH_ASYM40, 130.0, "sym->asym40", 0);
    /* N5 mechanism: a stroke whose dead spots moved by 90 deg (residual far above the mismatch
     * threshold) must drop to fallback within one revolution and re-converge. Physically
     * impossible (dead spots sit at the crank's own TDC/BDC), so the transient inside that first
     * revolution is printed, not asserted. */
    style_case(&SH_DEAD, &SH_DEAD90, 20.0, "shift90", 1);
    style_case(&SH_DEAD, &SH_DEAD90, 60.0, "shift90", 1);
    style_case(&SH_DEAD, &SH_DEAD90, 130.0, "shift90", 1);
}

/* ------------------------------------------------------------------ G1-OSC */

static void test_osc(void)
{
    static const double cads[] = { 25, 60, 120 };
    const rider_script_t *shapes[] = { &SH_DEAD, &SH_ASYM, &SH_DEEP };
    sim_t s;
    for (int si = 0; si < 3; ++si)
        for (int ci = 0; ci < 3; ++ci) {
            obs_t all, post;
            sim_init(&s, shapes[si], cads[ci], 1000.0, 30.0, 900u + (uint32_t)(si * 3 + ci));
            obs_reset(&all);
            run_obs_revs(&s, &all, 20.0);
            obs_reset(&post);
            run_obs_revs(&s, &post, 200.0);
            printf("  G1-OSC shape %d %3g rpm noise 30: transitions convergence %u / steady 200 rev %u, template toggles %u, I %u..%u\n",
                   si, cads[ci], all.transitions, post.transitions, post.tmode_toggles, post.imin, post.imax);
            CHECK(post.transitions == 0 && post.tmode_toggles == 0, "G1-OSC zero class transitions in 200 rev steady riding");
            CHECK(all.transitions == 0, "G1-OSC zero class transitions while converging from power-on");
            CHECK(template_sum() == NBINS * 4096u, "G1-OSC/LONG template mean stays exactly 1.0");
        }
}

/* ------------------------------------------------------------------ G1-PAS */

static void test_pas(void)
{
    sim_t s;
    obs_t o;
    const assist_v3_intent_debug_t *d = assist_v3_intent_debug();
    uint8_t off0;
    /* sampler overflow: 37 events lost in a stall, flagged */
    sim_init(&s, &SH_ASYM, 60.0, 1000.0, 20.0, 1000u);
    sim_revs(&s, 30.0);
    off0 = d->tpl_offset;
    sim_drop(&s, 37);
    sim_tick(&s);
    CHECK(!s.out.phase_aligned, "G1-PAS overflow -> unaligned");
    obs_reset(&o);
    run_obs_revs(&s, &o, 3.0);
    CHECK(o.rel == 0 && o.att == 0, "G1-PAS overflow: no false class");
    CHECK(s.out.phase_aligned && s.out.template_mode, "G1-PAS overflow: re-aligned, template mode");
    CHECK((d->tpl_offset - off0 - 37 + 960) % 96 <= 1 || (d->tpl_offset - off0 - 37 + 960) % 96 == 95,
          "G1-PAS overflow: re-alignment recovers the 37 lost steps");
    /* more than a revolution in one call */
    s.extra = 200; s.glitch = true;
    sim_tick(&s);
    CHECK(!s.out.phase_aligned && s.out.steps_since_restart == 0, "G1-PAS >96 step jump -> unaligned + restart");
    obs_reset(&o);
    run_obs_revs(&s, &o, 3.0);
    CHECK(o.rel == 0 && o.att == 0, "G1-PAS >96 jump: no false class");
    CHECK(s.out.phase_aligned, "G1-PAS >96 jump: re-aligned");
    /* INVALID with no step loss (illegal state and back) */
    off0 = d->tpl_offset;
    s.glitch = true;
    obs_reset(&o);
    run_obs_revs(&s, &o, 3.0);
    CHECK(d->tpl_offset == off0 && s.out.phase_aligned, "G1-PAS INVALID without loss: same alignment");
    CHECK(o.rel == 0 && o.att == 0, "G1-PAS INVALID: no false class");
    /* glitch burst: every 0.5 s for 5 s, alternating +-2 losses */
    obs_reset(&o);
    for (int i = 0; i < 10; ++i) {
        sim_drop(&s, (i & 1) ? -2 : 2);
        for (int k = 0; k < 2000; ++k) { sim_tick(&s); obs_take(&o, &s); }
    }
    CHECK(o.rel == 0 && o.att == 0, "G1-PAS glitch burst: no false class");
    run_obs_revs(&s, &o, 3.0);
    CHECK(s.out.phase_aligned && s.out.template_mode, "G1-PAS glitch burst: re-aligned afterwards");
    /* glitch at 20 rpm with deep dead spots */
    sim_init(&s, &SH_DEEP, 20.0, 1000.0, 20.0, 1001u);
    sim_revs(&s, 25.0);
    sim_drop(&s, 2);
    obs_reset(&o);
    run_obs_revs(&s, &o, 3.0);
    CHECK(o.rel == 0 && o.att == 0 && s.out.phase_aligned, "G1-PAS 20 rpm deep dead spots: glitch handled");
}

/* ------------------------------------------------------------------ determinism (G1-IMU part) */

static uint64_t composite_hash(void)
{
    sim_t s;
    uint64_t h = 1469598103934665603ull;
    sim_init(&s, &SH_ASYM, 45.0, 900.0, 25.0, 4242u);
    for (int seg = 0; seg < 12; ++seg) {
        switch (seg) {
        case 1: s.mean = 0.0; break;
        case 2: s.mean = 1500.0; break;
        case 3: s.rpm = 0.0; break;
        case 4: s.rpm = -25.0; s.mean = 300.0; break;
        case 5: s.rpm = 90.0; s.mean = 1200.0; sim_drop(&s, 2); break;
        case 6: s.rpm = 22.0; break;
        case 7: s.rpm = 128.0; s.mean = 600.0; break;
        case 8: s.extra = 150; s.glitch = true; break;
        case 9: s.torque_invalid = true; break;
        case 10: s.torque_invalid = false; s.rpm = 60.0; break;
        default: break;
        }
        for (int k = 0; k < 24000; ++k) {
            const unsigned char *p = (const unsigned char *)&s.out;
            sim_tick(&s);
            for (size_t i = 0; i < sizeof(s.out); ++i) { h ^= p[i]; h *= 1099511628211ull; }
        }
    }
    return h;
}

static void test_determinism(void)
{
    uint64_t a = composite_hash(), b = composite_hash();
    printf("  G1-IMU determinism: output hash %016llx / %016llx\n", (unsigned long long)a, (unsigned long long)b);
    CHECK(a == b, "G1-IMU intent output bit-identical across runs (power_on resets all state)");
}

/* ------------------------------------------------------------------ env_equiv / kL */

static void test_kl(void)
{
    static const double cads[] = { 20, 25, 30, 40, 60, 80, 100, 120, 130 };
    static const uint16_t loads[] = { 400, 800, 1500, 3000 };
    static const uint16_t thrs[] = { 820, 995 };
    uint16_t tpl[3][NBINS];
    const char *tn[] = { "prior", "deadspot", "steady" };
    double worst = 0, worst_real = 0;
    sim_t s;
    memcpy(tpl[0], assist_v3_intent_prior(), sizeof(tpl[0]));
    shape_q12(&SH_DEAD, tpl[1]);
    shape_q12(&SH_STEADY, tpl[2]);
    for (int t = 0; t < 3; ++t) {
        double wt = 0;
        for (unsigned ci = 0; ci < sizeof(cads) / sizeof(cads[0]); ++ci)
            for (int li = 0; li < 4; ++li)
                for (int ti = 0; ti < 2; ++ti) {
                    uint16_t ess;
                    uint16_t kl = assist_v3_intent_compute_kl(tpl[t], loads[li], (uint16_t)cads[ci], thrs[ti], &ess, NULL);
                    double fess, fk = float_kl(tpl[t], loads[li], cads[ci], thrs[ti], 1, &fess);
                    double fr = float_kl(tpl[t], loads[li], cads[ci], thrs[ti], 0, NULL);
                    double e, er;
                    if (kl == 0) { CHECK(fess < 1.0, "kL undefined only when the stroke never exceeds the threshold"); continue; }
                    e = fabs(kl / 4096.0 - fk) / fk;
                    er = fabs(kl / 4096.0 - fr) / fr;
                    if (e > wt) wt = e;
                    if (er > worst_real) worst_real = er;
                }
        printf("  kL %-8s vs float D7EC (udiv truncation) max rel err %.4f\n", tn[t], wt);
        if (wt > worst) worst = wt;
    }
    printf("  kL vs real-valued recurrence (no truncation, informational) max rel err %.4f\n", worst_real);
    CHECK(worst <= 0.02, "kL steady state within 2 % of the float D7EC simulation");
    {
        uint16_t flat[NBINS], spike[NBINS];
        for (int b = 0; b < NBINS; ++b) { flat[b] = 4096; spike[b] = 0; }
        spike[3] = (uint16_t)(NBINS > 8 ? 32768 : 32768);
        for (int b = 0; b < NBINS; ++b) if (b != 3) spike[b] = (uint16_t)((NBINS * 4096 - 32768) / (NBINS - 1));
        CHECK(assist_v3_intent_compute_kl(flat, 1000, 60, 820, NULL, NULL) == 4096, "kL flat stroke = 1.0 (clamp floor)");
        CHECK(assist_v3_intent_compute_kl(spike, 1000, 120, 820, NULL, NULL) == 10240, "kL peaky stroke clamped at 2.5");
        CHECK(assist_v3_intent_compute_kl(flat, 100, 60, 995, NULL, NULL) == 0, "kL undefined below the deadband");
        printf("  kL prior @1000 CLU thr 820: 20 rpm %.3f, 60 rpm %.3f, 120 rpm %.3f\n",
               assist_v3_intent_compute_kl(tpl[0], 1000, 20, 820, NULL, NULL) / 4096.0,
               assist_v3_intent_compute_kl(tpl[0], 1000, 60, 820, NULL, NULL) / 4096.0,
               assist_v3_intent_compute_kl(tpl[0], 1000, 120, 820, NULL, NULL) / 4096.0);
    }

    /* in the loop: learned template kL, env_equiv formula, prior below the floor, thresholds */
    for (int mode = 1; mode <= 2; ++mode) {
        uint16_t thr = mode == 1 ? 820 : 995;
        uint16_t ref;
        sim_init(&s, &SH_DEAD, 60.0, 1000.0, 0.0, 1100u);
        s.engaged_mode = mode;
        sim_revs(&s, 30.0);
        ref = assist_v3_intent_compute_kl(assist_v3_intent_template(), assist_v3_intent_debug()->last_rev_mean, 60, thr, NULL, NULL);
        CHECK(!s.out.kl_from_prior && s.out.kl_q12 == ref, "kL from the learned template at a stable revolution");
        CHECK(s.out.env_equiv == assist_v3_eb74_active(((uint32_t)s.out.kl_q12 * s.out.intent) >> 12, thr),
              "env_equiv = EB74_active(kL * I) with V3's own threshold");
        CHECK(s.out.engage_ok == (s.out.env_equiv > 0), "engage_ok follows env_equiv and EB74 armed");
    }
    sim_init(&s, &SH_DEAD, 60.0, 200.0, 0.0, 1101u);
    sim_revs(&s, 10.0);
    CHECK(s.out.kl_from_prior, "I_rev < 300 CLU -> prior kL");
    {
        uint32_t r0 = assist_v3_intent_debug()->kl_recomputes;
        s.rpm = 75.0;
        sim_revs(&s, 0.1);
        CHECK(assist_v3_intent_debug()->kl_recomputes > r0, "kL recomputed when cadence moves > 10 %");
    }

    /* N1: light spin, then a hard push within one revolution: env_equiv bounded */
    {
        static const double nc[] = { 20, 60 };
        for (int i = 0; i < 2; ++i)
            for (int base = 200; base <= 400; base += 200) {
                uint16_t maxenv = 0, maxkl = 0, kt;
                double envref;
                sim_init(&s, &SH_DEAD, nc[i], base, 10.0, 1200u + (uint32_t)base);
                sim_revs(&s, 12.0);
                s.mean = 2000.0;
                {
                    double a0 = s.cum_deg;
                    while (s.cum_deg < a0 + 720.0) {
                        sim_tick(&s);
                        if (s.out.env_equiv > maxenv) maxenv = s.out.env_equiv;
                        if (s.out.kl_q12 > maxkl) maxkl = s.out.kl_q12;
                    }
                }
                kt = assist_v3_intent_compute_kl(tpl[1], 2000, (uint16_t)nc[i], 820, NULL, NULL);
                envref = assist_v3_eb74_active((2000u * kt) >> 12, 820);
                printf("  N1 spin %d CLU -> 2000 CLU @%g rpm: max env_equiv %u vs converged-stroke envelope %.0f (kL max %.3f)\n",
                       base, nc[i], maxenv, envref, maxkl / 4096.0);
                CHECK(maxenv <= 1.10 * envref, "N1 light spin -> attack: env_equiv <= 1.1 x the true stroke envelope");
                CHECK(maxkl <= 10240u, "N1 kL never above 2.5");
            }
    }
}

/* ------------------------------------------------------------------ CPU sanity */

static void test_cpu(void)
{
    sim_t s;
    const assist_v3_intent_debug_t *d = assist_v3_intent_debug();
    uint32_t steps, mw, mk;
    sim_init(&s, &SH_DEAD, 130.0, 1000.0, 20.0, 1300u);
    sim_revs(&s, 20.0);
    s.rpm = 15.0; sim_revs(&s, 3.0);
    s.rpm = 20.0; sim_revs(&s, 3.0);
    steps = d->total_steps; mw = d->max_walk_steps; mk = d->max_kl_iterations;
    printf("  CPU: steps %u, max window walk %u entries/step, max kL recurrence %u iterations (15 rpm clamp), "
           "alignment %u x %u MAC per unaligned revolution, kL recomputes %u, max steps/call %u\n",
           steps, mw, mk, 96u, (unsigned)NBINS, d->kl_recomputes, d->max_steps_per_call);
    CHECK(mw <= 48u, "CPU window walk <= 48 entries per step");
    CHECK(mk <= 400u, "CPU kL recurrence <= 400 iterations");
}

int main(void)
{
    SH_STEADY = make_shape(50, 0, 0, 0);
    SH_DEAD = make_shape(100, 0, 60, 30);
    SH_DEEP = make_shape(100, 0, 90, 45);
    SH_ASYM = make_shape(80, 30, 40, 30);
    SH_SIT = make_shape(60, 0, 0, 0);
    SH_STAND = make_shape(100, 0, 80, 40);
    SH_ASYM40 = make_shape(80, 40, 0, 0);
    SH_DEAD90 = make_shape_ph(100, 0, 60, 30, 90);
    printf("assist_v3_intent host tests, NB = %d\n", NBINS);
    test_phase();
    test_wrap();
    test_template();
    test_classifier();
    test_fallback();
    test_style();
    test_osc();
    test_pas();
    test_determinism();
    test_kl();
    test_cpu();
    if (host_test_failures) {
        printf("assist_v3_intent: %d FAIL\n", host_test_failures);
        return 1;
    }
    puts("assist_v3_intent (G1-PHASE/TPL/CLS/FALLBACK/STYLE/OSC/PAS/IMU-determinism/kL/CPU) PASS");
    return 0;
}
