#include "g53_port_boundaries.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

/* A failing negative test must exit in unattended Windows runs, not open the
 * CRT's abort/report dialog. Keep each assertion active regardless of NDEBUG. */
#undef assert
#define assert(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
    exit(1); \
} } while (0)

static void ax_vectors(void)
{
    /* Frozen verification contract M-x. Values are evidence, not recomputed
     * from the implementation under test. 4096 is an artificial overrange. */
    static const uint16_t vectors[][6] = {
        {0, 0, 0, 0, 0, 0},
        {32, 0, 0, 0, 0, 0},
        {33, 1, 1, 1, 1, 1},
        {1024, 31, 54, 84, 110, 121},
        {2048, 62, 108, 169, 223, 246},
        {3072, 93, 163, 254, 334, 370},
        {4095, 124, 217, 339, 446, 494},
        {4096, 124, 217, 339, 447, 495}
    };
    static const unsigned checkpoints[] = {1, 2, 4, 8, 128};
    puts("case,invocation,expected,actual");
    for (unsigned row = 0; row < sizeof(vectors)/sizeof(vectors[0]); ++row) {
        g53_ax_reset();
        unsigned checkpoint = 0;
        for (unsigned invocation = 1; invocation <= 128; ++invocation) {
            const uint16_t actual = g53_ax_step(vectors[row][0]);
            if (invocation == checkpoints[checkpoint]) {
                const uint16_t expected = vectors[row][checkpoint + 1];
                printf("ax-%u,%u,%u,%u\n", vectors[row][0], invocation, expected, actual);
                assert(actual == expected);
                ++checkpoint;
            }
        }
    }
    static const uint16_t release[] = {
        370,277,207,155,116,87,65,48,36,27,20,15,11,8,6,4,3,2,1,0
    };
    g53_ax_reset();
    uint16_t value = 0;
    for (unsigned n = 0; n < 128; ++n) value = g53_ax_step(4095);
    assert(value == 494);
    for (unsigned n = 0; n < sizeof(release)/sizeof(release[0]); ++n) {
        value = g53_ax_step(0);
        printf("ax-release,%u,%u,%u\n", n+1, release[n], value);
        assert(value == release[n]);
    }
    for (unsigned n = 0; n < 128; ++n) assert(g53_ax_step(0) == 0);
    (void)g53_ax_step(4095);
    g53_ax_reset();
    assert(g53_ax_step(0) == 0);
}

static void eb74_vectors(void)
{
    /* Persisted output of the pinned, independent REV-EB74 semantic reference.
     * Source and fixture SHA-256 are recorded beside the CSV. Never regenerate
     * expected values in this test, or compare the port against itself. */
    FILE *file = fopen("integration/evidence/evd-tq/TQ-06/host/boundaries/eb74-reference.csv", "r");
    assert(file != NULL);
    char line[512];
    assert(fgets(line, sizeof(line), file) != NULL);
    unsigned rows = 0;
    while (fgets(line, sizeof(line), file) != NULL) {
        unsigned reset, load, speed, dee, m298, pre, output, zero, threshold, startup, check, m29e;
        int cadence;
        assert(sscanf(line, "%u,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
            &reset,&load,&cadence,&speed,&dee,&m298,&pre,&output,&zero,&threshold,&startup,&check,&m29e) == 13);
        if (reset) g53_ad7ec_reset();
        const g53_ad7ec_feedback_t feedback = {
            .cadence=(int16_t)cadence, .speed_native=(uint16_t)speed,
            .d7ec_rider=(uint16_t)dee, .m298=(uint8_t)m298
        };
        const g53_ad7ec_output_t actual = g53_ad7ec_step((uint16_t)load, &feedback);
        if (actual.pre_eb74 != pre || actual.rider_input_native != output || actual.zero != zero ||
            actual.threshold != threshold || actual.startup_count != startup ||
            actual.check_count != check || actual.m29e != m29e) {
            fprintf(stderr, "EB74 first divergence row %u input %sactual: %u,%u,%u,%u,%u,%u,%u\n",
                rows+1,line,actual.pre_eb74,actual.rider_input_native,actual.zero,
                actual.threshold,actual.startup_count,actual.check_count,actual.m29e);
            exit(1);
        }
        ++rows;
    }
    assert(!ferror(file));
    assert(fclose(file) == 0);
    assert(rows == 29516);
    printf("EB74 independent reference: %u vectors, 0 mismatches PASS\n", rows);
}

static void boundary_b_vectors(void)
{
    static const struct { uint16_t raw; int32_t full, expected; } cases[] = {
        {0, 500, 0},
        {1, 500, 0},
        {2500, 500, 125},
        {5000, 500, 250},
        {7500, 500, 375},
        {10000, 500, 500},
        {12000, 500, 600},
        {32767, 500, 1638},
        {32768, 500, -1638},
        {64536, 500, -50},
        {65535, 500, 0},
        {0, 700, 0},
        {1, 700, 0},
        {2500, 700, 175},
        {5000, 700, 350},
        {7500, 700, 525},
        {10000, 700, 700},
        {12000, 700, 840},
        {32767, 700, 2293},
        {32768, 700, -2293},
        {64536, 700, -70},
        {65535, 700, 0},
        {0, 900, 0},
        {1, 900, 0},
        {2500, 900, 225},
        {5000, 900, 450},
        {7500, 900, 675},
        {10000, 900, 900},
        {12000, 900, 1080},
        {32767, 900, 2949},
        {32768, 900, -2949},
        {64536, 900, -90},
        {65535, 900, 0},
        {0, 1000000, 0},
        {1, 1000000, 100},
        {2500, 1000000, 250000},
        {5000, 1000000, 500000},
        {7500, 1000000, 750000},
        {10000, 1000000, 1000000},
        {12000, 1000000, 1200000},
        {32767, 1000000, 3276700},
        {32768, 1000000, -3276800},
        {64536, 1000000, -100000},
        {65535, 1000000, -100},
        {0, -900, 0},
        {1, -900, 0},
        {2500, -900, -225},
        {5000, -900, -450},
        {7500, -900, -675},
        {10000, -900, -900},
        {12000, -900, -1080},
        {32767, -900, -2949},
        {32768, -900, 2949},
        {64536, -900, 90},
        {65535, -900, 0},
        {0, 0, 0},
        {1, 0, 0},
        {2500, 0, 0},
        {5000, 0, 0},
        {7500, 0, 0},
        {10000, 0, 0},
        {12000, 0, 0},
        {32767, 0, 0},
        {32768, 0, 0},
        {64536, 0, 0},
        {65535, 0, 0},
    };
    for(unsigned i=0;i<sizeof(cases)/sizeof(cases[0]);++i)
        assert(g53_boundary_b_iq_request(cases[i].raw,cases[i].full)==cases[i].expected);
    puts("Boundary B: 66 signed/runtime/full-scale/truncation vectors PASS");
}

int main(void)
{
    boundary_b_vectors();
    ax_vectors();
    eb74_vectors();
    const g53_ad7ec_feedback_t feedback = {0};
    g53_ax_reset();
    g53_ad7ec_reset();
    assert(g53_ax_step(0) == 0);
    assert(g53_ad7ec_step(0, &feedback).rider_input_native == 0);
    assert(g53_boundary_b_iq_request(0, 900) == 0);
    puts("G53 boundaries: M-x, A-D7EC/EB74 and Boundary B PASS");
    return 0;
}
