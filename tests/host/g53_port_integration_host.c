#include "g53_port.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "pas_quadrature.h"

static int contains_text(const char *path, const char *needle)
{
    char line[512];
    FILE *file = fopen(path, "r");
    if (!file) return 0;
    while (fgets(line, sizeof(line), file)) {
        if (strstr(line, needle)) { fclose(file); return 1; }
    }
    fclose(file);
    return 0;
}

static unsigned bit_count2(uint8_t value)
{
    value &= 3U;
    return (unsigned)(value & 1U) + (unsigned)((value >> 1) & 1U);
}

static void p9_g5_exhaustive_pairs(void)
{
    static const uint8_t expected_map[4] = {0U, 2U, 1U, 3U};
    bool seen[4] = {false, false, false, false};
    unsigned pairs = 0U;
    assert(PAS_DIR_SIGN == -1);

    for (uint8_t native = 0U; native < 4U; ++native) {
        const uint8_t mapped = (uint8_t)(((native & 1U) << 1) | ((native & 2U) >> 1));
        assert(mapped == expected_map[native]);
        assert(!seen[mapped]);
        seen[mapped] = true;
        assert((uint8_t)(((mapped & 1U) << 1) | ((mapped & 2U) >> 1)) == native);
    }
    for (uint8_t native = 0U; native < 4U; ++native) assert(seen[native]);

    for (uint8_t prev = 0U; prev < 4U; ++prev) {
        for (uint8_t next = 0U; next < 4U; ++next) {
            const int8_t native_step = pas_quadrature_step(prev, next);
            const unsigned distance = bit_count2((uint8_t)(prev ^ next));
            g53_port_input_t in = {
                .assist_level = 1U,
                .elapsed_ticks = 4U,
                .phase_current_max = 900,
                .torque_sensor_valid = true
            };
            g53_port_output_t out;
            g53_port_reset();
            if (prev == 0U) {
                /* Consume the G53 boot-edge skip, then return to native code 0. */
                in.pas_ab = 2U;
                g53_port_update(&in, &out);
                in.pas_ab = 0U;
                g53_port_update(&in, &out);
            } else {
                in.pas_ab = prev;
                g53_port_update(&in, &out); /* consume boot edge and establish previous code */
            }
            const g53_port_trace_t before = out.trace;
            in.pas_ab = next;
            g53_port_update(&in, &out);

            if (prev == next) {
                assert(native_step == 0);
                assert(out.trace.pas_direction == before.pas_direction);
                assert(out.trace.pas_current_delta == before.pas_current_delta);
                assert(out.trace.pas_transition_count == before.pas_transition_count);
            } else if (distance == 2U) {
                assert(native_step == 0);
                assert(out.trace.pas_direction != 1);
                assert(out.trace.pas_current_delta == 2 || out.trace.pas_current_delta == -2);
            } else {
                assert(native_step == 1 || native_step == -1);
                if (out.trace.pas_direction != native_step) {
                    fprintf(stderr, "P9-G5 pair prev=%u next=%u native_step=%d mapped_prev=%u mapped_next=%u g53_dir=%ld delta=%ld\n",
                        prev, next, native_step, expected_map[prev], expected_map[next],
                        (long)out.trace.pas_direction, (long)out.trace.pas_current_delta);
                }
                assert(out.trace.pas_direction == native_step);
            }
            assert(out.trace.pas_code == expected_map[next]);
            ++pairs;
        }
    }
    assert(pairs == 16U);

    /* The native sampler/decoder and safety consumer stay on unswapped codes; only the
     * behavioral G53 handoff in g53_port.c receives the coordinate-normalized code. */
    assert(contains_text("src/pas_sampler.c", "pas_quadrature_step(from, ab)"));
    assert(contains_text("src/main.c", "pas_direction_on_step(st)"));
    assert(contains_text("src/main.c", ".pas_ab = pas_sampler_state()"));
    assert(contains_text("src/assist_pipeline.c", ".pas_ab=in->pas_ab"));
    assert(contains_text("src/g53_port.c", "g53_pas_step(&pas,g53_pas_coordinate(in->pas_ab),level)"));
    puts("P9-G5 native/G53 PAS seam: 16/16 pairs, bijection, self-inverse and native-safety ownership PASS");
}

int main(void)
{
    p9_g5_exhaustive_pairs();
    const g53_port_input_t in = {.phase_current_max = 900, .elapsed_ticks = 1};
    g53_port_output_t out;
    memset(&out, 0xa5, sizeof(out));
    g53_port_init();
    g53_port_update(&in, &out);
    assert(out.iq_request_pre_limits == 0 && out.m2aa_native == 0);
    assert(!out.normal_permission && g53_port_trace()->m2aa == 0);
    g53_port_reset();
    assert(g53_port_trace()->m2aa == 0);
    puts("G53 integration: phase-1 facade reset/zero smoke PASS");
    return 0;
}
