/*
 * TASK-EVD-TQ-06-G1: src/g53_g1_limiter.c against the G5300 original.
 *
 * The expected digests in g53_g1/g53_g1_golden.h come from the Python transcription of the
 * original machine code (TQ-02B-N4, PASS). This harness draws the same vectors from the same
 * xorshift32 stream (spec: g53_g1/gen_g53_g1_golden.py), runs the C module on a byte image of
 * the original RAM block and compares FNV-1a-64 digests per block of 1000 vectors.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "g53_g1_limiter.h"
#include "common/check.h"
#include "g53_g1/g53_g1_golden.h"

#define NB 0x60

static uint32_t rng_x;
static uint32_t nxt(void)
{
    uint32_t x = rng_x;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_x = x;
    return x;
}
static uint32_t rnd(uint32_t n) { return nxt() % n; }
static uint16_t r16(void)
{
    static const uint16_t spec[] = { 0, 1, 0x1000, 0x7FFF, 0x8000, 0xFFFF, 100, 500, 1900 };
    const uint32_t k = rnd(12);
    if (k == 0) return (uint16_t)(nxt() & 0xFFFFu);
    if (k == 1) return (uint16_t)rnd(5000);
    if (k == 2) return (uint16_t)(rnd(400) - 200u);
    return spec[k - 3];
}

static uint16_t g16(const uint8_t *b, int o) { return (uint16_t)(b[o] | (b[o + 1] << 8)); }
static uint32_t g32(const uint8_t *b, int o)
{
    return (uint32_t)b[o] | ((uint32_t)b[o + 1] << 8) | ((uint32_t)b[o + 2] << 16) | ((uint32_t)b[o + 3] << 24);
}
static void p16(uint8_t *b, int o, uint16_t v) { b[o] = (uint8_t)v; b[o + 1] = (uint8_t)(v >> 8); }
static void p32(uint8_t *b, int o, uint32_t v)
{
    for (int i = 0; i < 4; i++) b[o + i] = (uint8_t)(v >> (8 * i));
}

static void from_image(g53_g1_state_t *s, const uint8_t *b)
{
    s->base_select = b[0x00];
    s->level_pct = g16(b, 0x02); s->soc_factor = g16(b, 0x04); s->thermal_a = g16(b, 0x06);
    s->thermal_b = g16(b, 0x08); s->taper = g16(b, 0x0A); s->base = g16(b, 0x0C);
    s->base_alt = g16(b, 0x0E); s->knee_soc = g16(b, 0x10); s->knee_temp = g16(b, 0x12);
    s->step = g16(b, 0x14); s->step_hi = g16(b, 0x16); s->floor = g16(b, 0x18);
    s->base_used = g16(b, 0x1A); s->limit = g16(b, 0x1C); s->soc_excess = g16(b, 0x1E);
    s->limit_soc = g16(b, 0x20); s->temp_excess = g16(b, 0x22); s->limit_temp = g16(b, 0x24);
    s->temp_add = g16(b, 0x26); s->limit_final = g16(b, 0x28); s->target = g16(b, 0x2A);
    s->diff = g16(b, 0x2C); s->ramped = g16(b, 0x2E);
    s->pi.sp = g16(b, 0x30); s->pi.fb = g16(b, 0x32); s->pi.kp = (int16_t)g16(b, 0x34);
    s->pi.ki = (int16_t)g16(b, 0x36); s->pi.p_offset = g16(b, 0x38); s->pi.reserved_0a = g16(b, 0x3A);
    s->pi.hi = (int32_t)g32(b, 0x3C); s->pi.lo = (int32_t)g32(b, 0x40);
    s->pi.out_hi = g16(b, 0x44); s->pi.out_lo = g16(b, 0x46); s->pi.err = g16(b, 0x48);
    s->pi.reserved_1a = g16(b, 0x4A); s->pi.p_term = g32(b, 0x4C); s->pi.i_term = g32(b, 0x50);
    s->pi.sum = g32(b, 0x54); s->pi.out = g16(b, 0x58);
    s->g1 = g16(b, 0x5C); s->cfg_invalid = b[0x5E];
}

static void to_image(const g53_g1_state_t *s, uint8_t *b)
{
    b[0x00] = s->base_select;
    p16(b, 0x02, s->level_pct); p16(b, 0x04, s->soc_factor); p16(b, 0x06, s->thermal_a);
    p16(b, 0x08, s->thermal_b); p16(b, 0x0A, s->taper); p16(b, 0x0C, s->base);
    p16(b, 0x0E, s->base_alt); p16(b, 0x10, s->knee_soc); p16(b, 0x12, s->knee_temp);
    p16(b, 0x14, s->step); p16(b, 0x16, s->step_hi); p16(b, 0x18, s->floor);
    p16(b, 0x1A, s->base_used); p16(b, 0x1C, s->limit); p16(b, 0x1E, s->soc_excess);
    p16(b, 0x20, s->limit_soc); p16(b, 0x22, s->temp_excess); p16(b, 0x24, s->limit_temp);
    p16(b, 0x26, s->temp_add); p16(b, 0x28, s->limit_final); p16(b, 0x2A, s->target);
    p16(b, 0x2C, s->diff); p16(b, 0x2E, s->ramped);
    p16(b, 0x30, s->pi.sp); p16(b, 0x32, s->pi.fb); p16(b, 0x34, (uint16_t)s->pi.kp);
    p16(b, 0x36, (uint16_t)s->pi.ki); p16(b, 0x38, s->pi.p_offset); p16(b, 0x3A, s->pi.reserved_0a);
    p32(b, 0x3C, (uint32_t)s->pi.hi); p32(b, 0x40, (uint32_t)s->pi.lo);
    p16(b, 0x44, s->pi.out_hi); p16(b, 0x46, s->pi.out_lo); p16(b, 0x48, s->pi.err);
    p16(b, 0x4A, s->pi.reserved_1a); p32(b, 0x4C, s->pi.p_term); p32(b, 0x50, s->pi.i_term);
    p32(b, 0x54, s->pi.sum); p16(b, 0x58, s->pi.out);
    p16(b, 0x5C, s->g1); b[0x5E] = s->cfg_invalid;
}

static uint64_t fnv(uint64_t h, const uint8_t *d, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        h ^= d[i];
        h *= 0x100000001B3ULL;
    }
    return h;
}

static void random_image(uint8_t *img)
{
    for (int i = 0; i < NB; i++) img[i] = (uint8_t)(nxt() & 0xFFu);
}

static size_t vec_cfg1(uint8_t *out)
{
    uint8_t img[NB];
    g53_g1_state_t s;
    uint16_t f[12];
    random_image(img);
    for (int i = 0; i < 12; i++) f[i] = r16();
    const uint16_t m314 = r16();
    const g53_g1_config_t c = {
        .step = f[0], .step_hi = f[1], .p_offset = f[2], .kp = (int16_t)f[3], .ki = (int16_t)f[4],
        .out_min = (int16_t)f[5], .out_max = (int16_t)f[6], .limit = (int16_t)f[7],
        .knee_soc = (int16_t)f[8], .knee_temp = (int16_t)f[9], .floor = f[10], .limit_reduction = f[11]
    };
    from_image(&s, img);
    const uint16_t m314_out = g53_g1_configure(&s, &c, m314);
    to_image(&s, img);
    memcpy(out, img, NB);
    p16(out, NB, m314_out);
    return NB + 2;
}

static size_t vec_lim(uint8_t *out)
{
    uint8_t img[NB];
    g53_g1_state_t s;
    g53_g1_limit_input_t in;
    random_image(img);
    if (rnd(2) == 0) {
        p16(img, 0x0C, (uint16_t)rnd(4000)); p16(img, 0x0E, (uint16_t)rnd(4000));
        p16(img, 0x10, (uint16_t)rnd(3000)); p16(img, 0x12, (uint16_t)rnd(3000));
    }
    uint32_t k = rnd(4);
    in.bde8_state = (uint8_t)(k < 3 ? (uint32_t[]){ 6, 7, 4 }[k] : rnd(256));
    uint16_t pct[2];
    for (int i = 0; i < 2; i++) {
        k = rnd(5);
        pct[i] = (uint16_t)(k < 4 ? (uint32_t[]){ 0, 15, 40, 100 }[k] : (nxt() & 0xFFFFu));
    }
    in.level_pct = pct[0];
    in.level_pct_state6 = pct[1];
    k = rnd(4);
    in.base_select = (uint8_t)(k < 3 ? k : rnd(256));
    if (rnd(2) == 0) {
        in.soc_factor = (uint16_t)rnd(0x1001); in.thermal_a = (uint16_t)rnd(0x1001); in.thermal_b = (uint16_t)rnd(0x1001);
    } else {
        in.soc_factor = r16(); in.thermal_a = r16(); in.thermal_b = r16();
    }
    from_image(&s, img);
    const uint16_t m54 = g53_g1_limit_update(&s, &in);
    to_image(&s, img);
    memcpy(out, img, NB);
    p16(out, NB, m54);
    return NB + 2;
}

static size_t vec_pi1(uint8_t *out)
{
    uint8_t img[NB];
    g53_g1_state_t s;
    uint16_t taper;
    random_image(img);
    if (rnd(2) == 0) {
        p16(img, 0x34, rnd(2) ? 64 : 2048);
        p16(img, 0x36, rnd(2) ? 96 : 200);
        p16(img, 0x38, rnd(2) ? 0 : 500);
        p32(img, 0x3C, 4096u << 12);
        p32(img, 0x40, (uint32_t)-(int32_t)(4096 << 12));
        p16(img, 0x44, 4096);
        p16(img, 0x46, (uint16_t)-4096);
        p32(img, 0x50, rnd(1u << 25) - (1u << 24));
        p16(img, 0x1C, (uint16_t)rnd(3000));
    }
    if (rnd(4) == 0) {                     /* review F-03: floor == limit (0x0800C6FE movle edge) */
        img[0x18] = img[0x1C];
        img[0x19] = img[0x1D];
    }
    const uint32_t k = rnd(3);
    if (k == 0) taper = 0x1000;
    else if (k == 1) taper = (uint16_t)(nxt() & 0xFFFFu);
    else taper = (uint16_t)rnd(0x1001);
    const uint16_t fb = r16();
    from_image(&s, img);
    (void)g53_g1_step(&s, taper, fb);
    to_image(&s, img);
    memcpy(out, img, NB);
    return NB;
}

static int run_golden(const char *name, uint32_t xor_seed, size_t (*fn)(uint8_t *), const uint64_t *golden)
{
    uint8_t buf[NB + 2];
    int bad = 0;
    uint64_t h = 0xCBF29CE484222325ULL;
    rng_x = G53_G1_GOLDEN_SEED ^ xor_seed;
    for (int i = 0; i < G53_G1_GOLDEN_VECTORS; i++) {
        const size_t n = fn(buf);
        h = fnv(h, buf, n);
        if ((i + 1) % G53_G1_GOLDEN_BLOCK == 0) {
            const int blk = i / G53_G1_GOLDEN_BLOCK;
            if (h != golden[blk]) {
                if (bad < 5) printf("  %s block %d digest mismatch\n", name, blk);
                bad++;
            }
            h = 0xCBF29CE484222325ULL;
        }
    }
    printf("%s: %d vectors, %d/%d blocks mismatched\n", name, G53_G1_GOLDEN_VECTORS, bad,
           G53_G1_GOLDEN_VECTORS / G53_G1_GOLDEN_BLOCK);
    return bad;
}

static g53_g1_config_t stock_config(int16_t limit)
{
    const g53_g1_config_t c = {
        .step = G53_G1_DEFAULT_STEP, .step_hi = G53_G1_DEFAULT_STEP, .p_offset = G53_G1_DEFAULT_P_OFFSET,
        .kp = G53_G1_DEFAULT_KP, .ki = G53_G1_DEFAULT_KI, .out_min = G53_G1_DEFAULT_OUT_MIN,
        .out_max = G53_G1_DEFAULT_OUT_MAX, .limit = limit, .knee_soc = 500, .knee_temp = 500,
        .floor = 0, .limit_reduction = 0
    };
    return c;
}

static void behaviour_checks(void)
{
    g53_g1_state_t s;
    const g53_g1_limit_input_t full = { .bde8_state = 7, .level_pct = 100, .level_pct_state6 = 100,
        .base_select = 0, .soc_factor = 0x1000, .thermal_a = 0x1000, .thermal_b = 0x1000 };
    g53_g1_config_t c = stock_config(1500);

    g53_g1_reset(&s);
    CHECK((g53_g1_configure(&s, &c, 0) & 1u) == 0, "stock config (15 A, knees 5 A) is valid");
    CHECK(g53_g1_limit_update(&s, &full) == 1500 && s.limit == 1500, "limit = base at 100 %, factors 1.0");

    /* No load: g1 climbs to full scale. */
    for (int t = 0; t < 400; t++) (void)g53_g1_step(&s, 0x1000, 0);
    CHECK(s.g1 == 4096, "g1 reaches 1.0 with zero battery current");
    CHECK(s.pi.sp == 1500, "setpoint ramp reached the limit");

    /* Persistent overload: g1 must fall, monotonically, and never below 0. */
    uint16_t prev = s.g1;
    int monotone = 1;
    for (int t = 0; t < 2000; t++) {
        const uint16_t g = g53_g1_step(&s, 0x1000, 2500);
        if (g > prev) monotone = 0;
        prev = g;
    }
    CHECK(monotone, "g1 decreases monotonically under constant overload");
    CHECK(s.g1 == 0, "g1 reaches 0 under sustained overload (no plant)");

    /* Halving the limit acts at once (no downward ramp). */
    const g53_g1_limit_input_t half = { .bde8_state = 7, .level_pct = 50, .level_pct_state6 = 50,
        .base_select = 0, .soc_factor = 0x1000, .thermal_a = 0x1000, .thermal_b = 0x1000 };
    (void)g53_g1_limit_update(&s, &half);
    (void)g53_g1_step(&s, 0x1000, 0);
    CHECK(s.pi.sp == 750, "a lower limit is applied in one tick");

    /* SOC factor 0: limit collapses to the 5 A knee. */
    const g53_g1_limit_input_t soc0 = { .bde8_state = 7, .level_pct = 100, .level_pct_state6 = 100,
        .base_select = 0, .soc_factor = 0, .thermal_a = 0x1000, .thermal_b = 0x1000 };
    CHECK(g53_g1_limit_update(&s, &soc0) == 500 && s.limit == 500, "SOC factor 0 -> knee 5 A");

    /* Level % 0 resets PI #1 and zeroes the limit (original 0x0800C698). */
    const g53_g1_limit_input_t off = { .bde8_state = 7, .level_pct = 0, .level_pct_state6 = 0,
        .base_select = 0, .soc_factor = 0x1000, .thermal_a = 0x1000, .thermal_b = 0x1000 };
    s.pi.i_term = 12345;
    (void)g53_g1_limit_update(&s, &off);
    CHECK(s.limit == 0 && s.pi.i_term == 0 && s.pi.out == 0, "level 0 % resets PI #1");

    /* Step 2 M820 adapters (ADR-013 D3). */
    CHECK(g53_g1_soc_factor_m820(500, 0xFF, 0xFF) == 0x1000, "SOC: point 1 disabled -> 1.0");
    CHECK(g53_g1_soc_factor_m820(500, 0, 10) == 0x1000, "SOC: point 1 = 0 -> disabled");
    CHECK(g53_g1_soc_factor_m820(300, 25, 5) == 0x1000, "SOC above point 1 -> 1.0");
    CHECK(g53_g1_soc_factor_m820(150, 25, 5) == 2048, "SOC midway between points -> 0.5");
    CHECK(g53_g1_soc_factor_m820(50, 25, 5) == 0 && g53_g1_soc_factor_m820(0, 25, 5) == 0,
          "SOC at/below point 2 -> 0 (limit = knee), never rising again (DISC-008)");
    CHECK(g53_g1_soc_factor_m820(125, 25, 0xFF) == 2048 && g53_g1_soc_factor_m820(125, 25, 30) == 2048,
          "SOC: point 2 disabled or >= point 1 -> ramp to 0 % SOC");
    {
        int monotone = 1;
        uint16_t prev = 0;
        for (int soc = 0; soc <= 1000; soc++) {
            const uint16_t f = g53_g1_soc_factor_m820(soc, 25, 5);
            if (f < prev) monotone = 0;
            prev = f;
        }
        CHECK(monotone, "SOC factor is monotonic in SOC over 0..100 %");
    }
    CHECK(g53_g1_soc_factor_m820(250, 25, 25) == 0x1000 && g53_g1_soc_factor_m820(249, 25, 25) < 0x1000 &&
          g53_g1_soc_factor_m820(125, 25, 25) == 2048,
          "SOC: point 2 == point 1 ramps to 0 % SOC (no step from the full limit to the knee, review S2-05)");

    /* Invalid configuration is flagged, not silently accepted. */
    c = stock_config(400); /* limit below the knees */
    CHECK((g53_g1_configure(&s, &c, 0) & 1u) == 1 && s.cfg_invalid == 1, "limit below knee is flagged invalid");
}

int main(void)
{
    int bad = 0;
    bad += run_golden("CFG1 0x0800C520", 0x11111111u, vec_cfg1, g53_g1_golden_cfg1);
    bad += run_golden("LIM  0x0800C5BA", 0x22222222u, vec_lim, g53_g1_golden_lim);
    bad += run_golden("PI1  0x0800C6B4", 0x33333333u, vec_pi1, g53_g1_golden_pi1);
    CHECK(bad == 0, "golden digests from the G5300 transcription");
    behaviour_checks();
    if (host_test_failures) {
        printf("FAIL: %d check(s)\n", host_test_failures);
        return 1;
    }
    printf("PASS: g53_g1_limiter bit-exact vs G5300 transcription + behaviour checks\n");
    return 0;
}
