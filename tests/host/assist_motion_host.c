/*
 * Assist Behavior V3 - motion / IMU seam sanitiser (TEST_MATRIX G1-IMU, sanitiser part).
 * Real module: src/assist_motion.c.
 *
 *  - random bytes in every field (including the bool's byte) with valid = false -> output is the
 *    all-zero struct, byte for byte (padding included);
 *  - random bytes with valid = true but age_ms > MOTION_MAX_AGE_MS -> all-zero;
 *  - valid and fresh (age 0 and exactly MOTION_MAX_AGE_MS) -> exact field copy, valid = true;
 *  - NULL raw -> all-zero; aliasing raw == out works.
 */
#include "assist_motion.h"
#include "common/check.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t rng = 0x2545F491u;
static uint32_t xr(void)
{
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return rng;
}

static void fill_random(motion_input_t *m)
{
    unsigned char *p = (unsigned char *)m;
    for (size_t i = 0; i < sizeof(*m); ++i) p[i] = (unsigned char)xr();
}

static int is_zero_invalid(const motion_input_t *m)
{
    motion_input_t z;
    memset(&z, 0, sizeof(z));
    return memcmp(m, &z, sizeof(z)) == 0;
}

int main(void)
{
    motion_input_t raw, out;
    int bad_invalid = 0, bad_stale = 0, bad_fresh = 0;

    for (int i = 0; i < 200000; ++i) {
        fill_random(&raw);
        memset(&raw.valid, 0, 1);                       /* valid = false, everything else garbage */
        memset(&out, 0xA5, sizeof(out));
        assist_motion_sanitize(&raw, &out);
        if (!is_zero_invalid(&out)) bad_invalid++;

        fill_random(&raw);                              /* garbage bool byte: any non-zero = valid */
        if (*(unsigned char *)&raw.valid == 0u) memset(&raw.valid, 1, 1);
        raw.age_ms = (uint16_t)(MOTION_MAX_AGE_MS + 1u + (xr() % (65535u - MOTION_MAX_AGE_MS)));
        memset(&out, 0x5A, sizeof(out));
        assist_motion_sanitize(&raw, &out);
        if (!is_zero_invalid(&out)) bad_stale++;

        fill_random(&raw);
        raw.valid = true;
        raw.age_ms = (uint16_t)(xr() % (MOTION_MAX_AGE_MS + 1u));
        assist_motion_sanitize(&raw, &out);
        if (!out.valid || out.age_ms != raw.age_ms || out.pitch_cdeg != raw.pitch_cdeg ||
            out.roll_cdeg != raw.roll_cdeg || out.pitch_rate_cdps != raw.pitch_rate_cdps ||
            out.accel_long_mg != raw.accel_long_mg || out.accel_vert_mg != raw.accel_vert_mg)
            bad_fresh++;
    }
    CHECK(bad_invalid == 0, "valid=false with random fields -> all-zero output");
    CHECK(bad_stale == 0, "valid but stale (age > MOTION_MAX_AGE_MS) -> all-zero output");
    CHECK(bad_fresh == 0, "valid and fresh -> exact copy");

    memset(&raw, 0, sizeof(raw));
    raw.valid = true; raw.age_ms = MOTION_MAX_AGE_MS; raw.pitch_cdeg = -1234; raw.accel_vert_mg = 999;
    assist_motion_sanitize(&raw, &out);
    CHECK(out.valid && out.pitch_cdeg == -1234 && out.accel_vert_mg == 999, "age == limit accepted");

    memset(&out, 0x77, sizeof(out));
    assist_motion_sanitize(NULL, &out);
    CHECK(is_zero_invalid(&out), "NULL raw -> all-zero");

    fill_random(&raw);
    memset(&raw.valid, 0, 1);
    assist_motion_sanitize(&raw, &raw);                 /* aliasing */
    CHECK(is_zero_invalid(&raw), "raw == out aliasing, invalid -> all-zero");

    if (host_test_failures) {
        printf("assist_motion: %d FAIL\n", host_test_failures);
        return 1;
    }
    puts("assist_motion sanitiser (G1-IMU): 600000 random vectors + edges PASS");
    return 0;
}
