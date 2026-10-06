/* TASK-EVD-STOP-RAMP-001: PEDAL follows the G5300 D7EC/PAS/BDE8 stop path.
 * OWNER-DEC-2026-10-06-G5300-ONLY. Real production pipeline and final-Iq owner. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "assist_pipeline.h"
#include "assist_modes.h"
#include "config.h"
#include "fast_iq_slew.h"
#include "g53_port.h"

#define PHASE_MAX 700
#define ENGAGE_MS 500
#define SPEED_ON_MS 1000
#define STOP_MS 3000
#define AFTER_MS 3000
#define BDE8_FALL_BOUND ((PHASE_MAX * 50 + 9999) / 10000 + 1)

static unsigned failures;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL %s\n", msg); ++failures; } } while (0)
static const uint8_t cycle[] = {0U, 2U, 3U, 1U};
typedef enum { STEP_REVERSE, NO_STEP } step_t;
typedef struct {
    const char *name;
    uint32_t speed;
    step_t step;
    bool hold_load;
    bool repress;
    int brake_at;
} scenario_t;
typedef struct {
    int32_t before, request[AFTER_MS], ref[AFTER_MS], m2aa[AFTER_MS];
    fis_mode_t mode[AFTER_MS];
    int first_pas_zero, first_pas_reverse;
} trace_t;

static void init_input(assist_pipeline_input_t *in)
{
    memset(in, 0, sizeof(*in));
    in->torque_sensor_valid = true; in->pas_sensor_valid = true;
    in->forward_valid = true; in->wheel_valid = true;
    in->assist_level_index = 3U; in->phase_current_max = PHASE_MAX;
    in->battery_voltage_mv = 48000U; in->battery_current_max = BATTERYCURRENT_MAX;
    in->u_abs = 1024; in->cal_i = 95; in->voltage_raw = 4000U;
    in->voltage_min_raw = 2800; in->controller_temperature_c = 30;
    in->speed_limit_x100 = 6000U; in->elapsed_ticks = 4U;
}

static void run(const scenario_t *s, trace_t *r)
{
    assist_pipeline_input_t in;
    assist_pipeline_command_t cmd;
    fast_iq_slew_mailbox_t mailbox;
    int32_t ref = 0;
    double ph = 0.0;
    unsigned ci = 0U;
    uint8_t ab = 0U;
    init_input(&in);
    memset(&mailbox, 0, sizeof(mailbox)); memset(r, 0, sizeof(*r));
    r->first_pas_zero = -1; r->first_pas_reverse = -1;
    fast_iq_slew_reset(&mailbox);
    assist_modes_init(); g53_port_init(); assist_pipeline_init();
    for (int t = 0; t < STOP_MS + AFTER_MS; ++t) {
        const int k = t - STOP_MS;
        const bool riding = t >= ENGAGE_MS && k < 0;
        in.speed_x100 = t < SPEED_ON_MS ? 0U : s->speed;
        if (k >= 0) {
            in.torque_load_ctrl = (s->hold_load || (s->repress && k >= 40)) ? 6000U : 0U;
            if (s->step == STEP_REVERSE && k == 0) {
                ci = (ci + 3U) % 4U;
                ab = cycle[ci];
            }
            if (s->step == STEP_REVERSE) {
                in.direction_inhibit = true;
                in.inhibit_is_reverse = true;
                in.forward_valid = false;
            } else in.forward_valid = false; /* liveness observation only */
            if (s->brake_at >= 0 && k >= s->brake_at) in.safety_cut = true;
        } else in.torque_load_ctrl = riding ? 6000U : 0U;
        in.torque_load_centikg = (uint16_t)(in.torque_load_ctrl / 2U);
        in.cadence_rpm = riding ? 60U : 0U;
        in.pas_ab = ab;
        assist_pipeline_update(&in, &cmd);
        fast_iq_slew_publish(&mailbox, cmd.final_iq_request, cmd.slew_mode,
            cmd.step_mag_8, cmd.release_ticks_16k, cmd.zero_policy, cmd.iq_ceiling);
        for (unsigned j = 0; j < 4U; ++j) fast_iq_slew_tick(&mailbox, &ref);
        in.live_iq_ref = ref; in.live_iq_valid = true;
        if (k == -1) r->before = cmd.final_iq_request;
        if (k >= 0) {
            const int32_t pas_dir = assist_pipeline_g53()->trace.pas_direction;
            if (r->first_pas_zero < 0 && pas_dir == 0) r->first_pas_zero = k;
            if (r->first_pas_reverse < 0 && pas_dir < 0) r->first_pas_reverse = k;
            r->request[k] = cmd.final_iq_request;
            r->ref[k] = ref;
            r->m2aa[k] = assist_pipeline_g53()->m2aa_native;
            r->mode[k] = cmd.slew_mode;
        }
        if (riding) {
            ph += 96.0 * 60.0 / 60000.0;
            if (ph >= 1.0) { ph -= 1.0; ci = (ci + 1U) % 4U; ab = cycle[ci]; }
        }
    }
}

static int first_at_most(const trace_t *r, unsigned pct)
{
    for (int k = 0; k < AFTER_MS; ++k)
        if ((int64_t)r->request[k] * 100 <= (int64_t)r->before * (100 - pct)) return k;
    return -1;
}
static int first_zero(const trace_t *r)
{
    for (int k = 0; k < AFTER_MS; ++k) if (r->request[k] == 0) return k;
    return -1;
}
static void check_release(const scenario_t *s)
{
    trace_t r;
    int32_t prev, max_fall = 0, max_rise = 0;
    run(s, &r);
    CHECK(r.before > 100, "positive precondition");
    prev = r.before;
    for (int k = 0; k < AFTER_MS; ++k) {
        if (prev - r.request[k] > max_fall) max_fall = prev - r.request[k];
        if (r.request[k] - prev > max_rise) max_rise = r.request[k] - prev;
        prev = r.request[k];
    }
    const int z = first_zero(&r);
    printf("%s: t10=%d t50=%d t90=%d zero=%d max_fall=%d max_rise=%d first=%d m2aa_zero=%d pas_reverse=%d pas_stop=%d\n",
        s->name, first_at_most(&r, 10), first_at_most(&r, 50),
        first_at_most(&r, 90), z, (int)max_fall, (int)max_rise,
        (int)r.request[0], z >= 0 ? (int)r.m2aa[z] : -1,
        r.first_pas_reverse, r.first_pas_zero);
    if (s->brake_at >= 0) {
        CHECK(r.request[s->brake_at - 1] > 0, "brake during active ramp");
        CHECK(r.request[s->brake_at] == 0 && r.mode[s->brake_at] == FIS_MODE_SAFETY,
            "native brake retains SAFETY 200 ms");
    } else if (s->speed == 0U) {
        CHECK(z >= 0 && z <= 2 && r.mode[z] == FIS_MODE_FORCE_ZERO,
            "standstill uses G53 stock hard zero at its next logical tick");
    } else {
        CHECK(r.request[0] > 0, "moving release has no first-tick drop to zero");
        CHECK(max_fall <= BDE8_FALL_BOUND, "moving fall bounded by BDE8 + 1 count/ms");
        CHECK(z >= 0, "moving release reaches zero through G53");
        if (s->repress) CHECK(max_rise <= BDE8_FALL_BOUND, "repress has no upward jump");
        else CHECK(max_rise == 0, "release does not rise");
        /* Boundary B's integer scaling may round the last few native units to zero. */
        if (z >= 0) CHECK(r.m2aa[z] <= 9, "G53 BDE8 caused zero");
        if (s->hold_load) CHECK(r.first_pas_zero >= 0 && z > r.first_pas_zero,
            "held load reaches zero after G53 PAS true stop");
    }
}

static void check_start(void)
{
    assist_pipeline_input_t in;
    assist_pipeline_command_t cmd;
    double ph = 0.0;
    unsigned ci = 0U;
    uint8_t ab = 0U;
    int t10 = -1, t50 = -1, t90 = -1;
    bool equal = true;
    init_input(&in);
    in.speed_x100 = 1500U;
    assist_modes_init(); g53_port_init(); assist_pipeline_init();
    for (int t = 0; t < 1500; ++t) {
        const bool on = t >= ENGAGE_MS;
        in.torque_load_ctrl = on ? 6000U : 0U;
        in.torque_load_centikg = (uint16_t)(in.torque_load_ctrl / 2U);
        in.cadence_rpm = on ? 60U : 0U; in.pas_ab = ab;
        assist_pipeline_update(&in, &cmd);
        if (cmd.final_iq_request != assist_pipeline_g53()->iq_request_pre_limits) equal = false;
        if (on) {
            int e = t - ENGAGE_MS;
            if (t10 < 0 && cmd.final_iq_request >= 46) t10 = e;
            if (t50 < 0 && cmd.final_iq_request >= 228) t50 = e;
            if (t90 < 0 && cmd.final_iq_request >= 410) t90 = e;
            ph += 96.0 * 60.0 / 60000.0;
            if (ph >= 1.0) { ph -= 1.0; ci = (ci + 1U) % 4U; ab = cycle[ci]; }
        }
    }
    printf("g start: t10=%d t50=%d t90=%d\n", t10, t50, t90);
    CHECK(equal, "start published request equals G53");
    CHECK(t10 >= 0 && t10 <= 90 && t50 >= 180 && t50 <= 200 &&
        t90 >= 285 && t90 <= 300, "start timing matches 0.634");
}

int main(void)
{
    const uint8_t accel[10] = {1,8,8,8,8,8,8,8,8,8};
    const uint16_t ratio[10] = {1,45,95,155,215,260,310,370,525,525};
    g53_chain_set_levels(accel, ratio);
    const scenario_t cases[] = {
        {"a STEP_REVERSE, load removed", 1500U, STEP_REVERSE, false, false, -1},
        {"b STEP_REVERSE, load held", 1500U, STEP_REVERSE, true, false, -1},
        {"c stop, load removed", 1500U, NO_STEP, false, false, -1},
        {"c stop, load held", 1500U, NO_STEP, true, false, -1},
        {"d standstill", 0U, STEP_REVERSE, false, false, -1},
        {"e brake at 20ms", 1500U, STEP_REVERSE, false, false, 20},
        {"f repress at 40ms", 1500U, NO_STEP, false, true, -1},
    };
    for (unsigned i = 0; i < sizeof(cases)/sizeof(cases[0]); ++i) check_release(&cases[i]);
    check_start();
    if (failures) { printf("STEP_REVERSE ramp: %u CHECK(S) FAILED\n", failures); return 1; }
    puts("STEP_REVERSE ramp: ALL CHECKS PASSED");
    return 0;
}
