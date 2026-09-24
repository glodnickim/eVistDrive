#include "g53_port_boundaries.h"
#include <string.h>

/* Boundary A-x: contract M-x / G53 0x080164B0. Raw PA6, not torque or mapped Iq.
 * State and updates are in G53 logical invocations, with one integer truncation
 * at each of the two shifts. The facade supplies the logical tick schedule. */
static uint16_t ax;
void g53_ax_reset(void) { ax = 0; }
uint16_t g53_ax_step(uint16_t raw_pa6_adc)
{
    const uint16_t u = (uint16_t)(((uint32_t)raw_pa6_adc * 498u) >> 12);
    ax = (uint16_t)((3u * (uint32_t)ax + u) >> 2);
    return ax;
}

/* EB74 fixed local-source configuration, REV-EB74 eb74_diff.py STOCK_B.
 * The owner-adapter CLU normalization is not a physical mV equivalence.
 * P50/P52/P53 stay at the accepted default 0 (contract section 6); therefore
 * the CAN-source alive fault cannot gate this local source and M29E stays 0. */
enum {
    EB74_START_LIMIT = 300,
    EB74_START_STEP = 10,
    EB74_CHECK_LENGTH = 100,
    EB74_CHECK_BACK = 10,
    EB74_AUTO_ZERO_WINDOW = 301,
    EB74_ZERO_MIN = 750,
    EB74_ZERO_MAX = 995,
    EB74_INITIAL_THRESHOLD = 960,
    EB74_DEADBAND = 245,
    EB74_FEEDBACK_THRESHOLD = 820,
    EB74_SATURATION = 3200,
    EB74_FILTER_COEFFICIENT = 6176
};

static struct {
    g53_ad7ec_output_t out;
    uint16_t auto_zero_count;
    uint16_t sample_count;
    uint32_t zero_sum;
    uint32_t filter_accumulator;
    uint16_t filtered;
    uint8_t auto_zero_armed;
} eb74;

/* ARM arithmetic shift with a defined C result, including negative feedback. */
static int32_t eb74_asr(int32_t value, unsigned shift)
{
    if (value >= 0) return value / (int32_t)(1u << shift);
    return (int32_t)(-((-(int64_t)value + ((1u << shift) - 1u)) >> shift));
}

void g53_ad7ec_reset(void)
{
    memset(&eb74, 0, sizeof(eb74));
    eb74.out.zero = EB74_ZERO_MIN;
    eb74.out.threshold = EB74_INITIAL_THRESHOLD;
}

static void eb74_auto_zero(uint16_t source, const g53_ad7ec_feedback_t *feedback)
{
    if (feedback->cadence != 0 || feedback->speed_native <= 1000 || feedback->m298 != 0) {
        eb74.auto_zero_armed = 0;
        eb74.auto_zero_count = 0;
        eb74.zero_sum = 0;
        eb74.sample_count = 0;
        return;
    }
    eb74.auto_zero_count = (uint16_t)(eb74.auto_zero_count + 1u);
    if (eb74.auto_zero_count >= EB74_AUTO_ZERO_WINDOW) {
        eb74.auto_zero_armed = 1;
        eb74.auto_zero_count = EB74_AUTO_ZERO_WINDOW;
    } else if (eb74.auto_zero_armed != 1) {
        eb74.zero_sum = 0;
        eb74.sample_count = 0;
        return;
    }
    eb74.sample_count = (uint16_t)(eb74.sample_count + 1u);
    if (eb74.sample_count <= 128) {
        eb74.zero_sum += source;
        return;
    }
    if (eb74.sample_count <= EB74_AUTO_ZERO_WINDOW) return;
    uint16_t average = (uint16_t)(eb74.zero_sum >> 7);
    if (average < EB74_ZERO_MIN) average = EB74_ZERO_MIN;
    else if (average > EB74_ZERO_MAX) average = EB74_ZERO_MAX;
    eb74.out.zero = average;
    eb74.zero_sum = 0;
    eb74.sample_count = 0;
}

g53_ad7ec_output_t g53_ad7ec_step(uint16_t load_ctrl,
                                 const g53_ad7ec_feedback_t *feedback)
{
    /* PROJECT_OWNER_DECISION: exact CLU -> synthetic pre-EB74 coordinates. */
    uint32_t source = 750u + ((uint32_t)load_ctrl * 2450u) / 6000u;
    if (source > EB74_SATURATION) source = EB74_SATURATION;
    eb74.out.pre_eb74 = (uint16_t)source;
    if (eb74.out.startup_count < EB74_START_LIMIT) {
        eb74.out.startup_count = (uint16_t)(eb74.out.startup_count + EB74_START_STEP);
        eb74.out.rider_input_native = 0;
        return eb74.out;
    }
    eb74.out.startup_count = EB74_START_LIMIT;
    eb74.out.m29e = 0; /* P52 == 0: the qualifying P52==31 condition is false. */
    if (eb74.out.check_count < EB74_CHECK_LENGTH) {
        ++eb74.out.check_count;
        if (source > eb74.out.threshold) {
            eb74.out.check_count = EB74_CHECK_LENGTH - EB74_CHECK_BACK;
        }
        eb74.out.rider_input_native = 0;
        return eb74.out;
    }
    eb74_auto_zero((uint16_t)source, feedback);
    eb74.out.threshold = (feedback->d7ec_rider == 0)
        ? (uint16_t)(eb74.out.zero + EB74_DEADBAND) : EB74_FEEDBACK_THRESHOLD;
    if (source <= eb74.out.threshold) {
        /* Exact-threshold has zero filter input and resets too, per EB74. */
        eb74.filtered = 0;
        eb74.filter_accumulator = 0;
    } else {
        /* Saturation is before subtraction. Positive local-domain input,
         * signed feedback difference, signed ASR, wrapped u32 accumulator. */
        const int32_t input = (int32_t)source - eb74.out.threshold;
        const int32_t correction = eb74_asr(
            (input - (int32_t)eb74.filtered) * EB74_FILTER_COEFFICIENT, 12);
        eb74.filter_accumulator += (uint32_t)correction;
        const int32_t accumulator = eb74.filter_accumulator <= INT32_MAX
            ? (int32_t)eb74.filter_accumulator
            : (int32_t)((int64_t)eb74.filter_accumulator - INT64_C(4294967296));
        eb74.filtered = (uint16_t)eb74_asr(accumulator, 2);
    }
    eb74.out.rider_input_native = eb74.filtered;
    return eb74.out;
}

int32_t g53_boundary_b_iq_request(uint16_t m2aa_native, int32_t phase_current_max)
{
    /* Contract 9.3: explicit s16 interpretation, widened product, trunc0.
     * Native ap2_limits owns negative suppression and hardware ceilings. */
    const int32_t signed_demand = m2aa_native <= INT16_MAX
        ? (int32_t)m2aa_native : (int32_t)m2aa_native - 65536;
    return (int32_t)(((int64_t)signed_demand * phase_current_max) / 10000);
}
