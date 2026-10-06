#include "g53_port.h"
#include "g53_port_boundaries.h"
#include <string.h>

/* M820 adapter: 4 control periods (4 kHz) per logical tick. This is an explicit
 * product timebase, not a claim about the original G53 oscillator. */
enum { CONTROL_TICKS_PER_LOGICAL=4, MAX_CATCHUP_LOGICAL_TICKS=64 };

/*
 * M820 PRODUCT POLICY: THROTTLE IS NOT ENABLED.
 *
 * Boundary A-x feeds the imported G53 throttle state machine. On M820 that turned raw PA6
 * into a hidden, level-independent motor owner: with no pedal load, no cadence and no
 * forward pedalling, PA6=2048 at 15 km/h gave A-x 246 -> throttle output 2142 -> M2AA 3399
 * -> Iq 237, released as a one-tick step by a single PAS edge (offline root-cause replay,
 * m820_pa6_hidden_throttle_regression). The G53 arming thresholds (x>120, raw ~1000) do not
 * match the stock M820 PA6 mapper (min 1737, own fault machine) and no validated M820
 * throttle mapping exists.
 *
 * So PA6 is still sampled and filtered for diagnostics (g53_port_pa6_ax_observed()), but the
 * G53 command input is held at 0 - the cold-boot value, below every throttle threshold, which
 * keeps the FSM in its idle state with zero output. Enabling a throttle is a new, separately
 * validated feature, not a flag flip.
 */
#define M820_G53_THROTTLE_ENABLED 0
#define G53_AX_THROTTLE_INACTIVE  0u
_Static_assert(M820_G53_THROTTLE_ENABLED == 0,
    "M820 G53 throttle requires a validated PA6 mapping before it may be enabled");

static g53_port_trace_t port_trace;
static g53_pas_ctx_t pas;
static uint8_t control_remainder;
static bool normal_permission;
static uint32_t dropped_ticks;
static uint16_t pa6_ax_observed;

/*
 * TASK-EVD-TQ-06-G1 / ADR-013: the G53 PI #1 battery-current limiter, the producer of g1.
 * Step 2: the SOC knee factor comes from M820 (g53_g1_soc_factor_m820 in main.c), passed as a
 * derate so a zero-initialised caller means "no derate". Thermal factors stay 1.0 (variant A).
 *
 * Its state is NOT part of g53_port_reset(). On the G5300 it lives outside the BDE8 command
 * state and survives stops, vetoes and owner changes; only power-on (and a level percentage of
 * 0) clears it. Resetting it with the pipeline would replay the ~0.2 s soft start on every
 * restart. Inputs: level 100 % and taper 1.0 (ADR-013 D4); SOC factor from M820 (step 2); M820
 * keeps its own Iq thermal derate, speed taper, cutoff and undervoltage protections.
 */
static g53_g1_state_t g1_limiter;
static int32_t g1_configured_limit;
static uint8_t g1_phase;
static uint16_t g1_m314;

static void g1_configure(int32_t limit_centiamp)
{
    if (limit_centiamp < 0) limit_centiamp = 0;
    if (limit_centiamp > 32767) limit_centiamp = 32767;
    const g53_g1_config_t c = {
        .step = G53_G1_DEFAULT_STEP, .step_hi = G53_G1_DEFAULT_STEP,
        .p_offset = G53_G1_DEFAULT_P_OFFSET, .kp = G53_G1_DEFAULT_KP, .ki = G53_G1_DEFAULT_KI,
        .out_min = G53_G1_DEFAULT_OUT_MIN, .out_max = G53_G1_DEFAULT_OUT_MAX,
        .limit = (int16_t)limit_centiamp,
        /* Step 2 (ADR-013 D3): SOC knee = 50 % of the limit (owner). The thermal knee is inert -
         * its factors are held at 1.0 (variant A) - and 0 keeps CFG1 valid for every limit.
         * The G5300 used 5 A / 5 A (N4 §2.2); the knees are M820 product values. */
        .knee_soc = (int16_t)((limit_centiamp * G53_G1_SOC_KNEE_PCT) / 100 > 0 ?
                              (limit_centiamp * G53_G1_SOC_KNEE_PCT) / 100 : 1),
        .knee_temp = 0,
        .floor = 0, .limit_reduction = 0
    };
    g1_m314 = g53_g1_configure(&g1_limiter, &c, g1_m314);
    g1_configured_limit = limit_centiamp;
}

/* P9-G5: translate native M820 A/B coordinates only at the G53 behavior seam.
 * Native PAS decoding and safety continue to consume the original packed value. */
static uint8_t g53_pas_coordinate(uint8_t native_pas_ab)
{
    return (uint8_t)(((native_pas_ab & 0x01u) << 1) |
                     ((native_pas_ab & 0x02u) >> 1));
}

void g53_port_reset(void)
{
    g53_ax_reset();
    g53_ad7ec_reset();
    g53_pas_reset(&pas,true,0);
    g53_chain_reset();
    memset(&port_trace,0,sizeof(port_trace));
    control_remainder=0;
    normal_permission=false;
    dropped_ticks=0;
    pa6_ax_observed=0;
}
static uint8_t level_power[10] = {0,100,100,100,100,100,100,100,100,100};

void g53_port_set_levels(const uint8_t accel[10], const uint16_t ratio[10], const uint8_t power[10])
{
    uint8_t slot;
    g53_chain_set_levels(accel,ratio);
    for(slot=1;slot<10;slot++) level_power[slot]=power[slot]>100 ? 100 : power[slot];
}
void g53_port_init(void)
{
    g53_port_reset();
    g53_g1_reset(&g1_limiter);
    g1_configured_limit = -1;
    g1_phase = 0;
    g1_m314 = 0;
}

void g53_port_update(const g53_port_input_t *in,g53_port_output_t *out)
{
    const uint64_t elapsed=(uint64_t)(in->elapsed_ticks ? in->elapsed_ticks : 1u)+control_remainder;
    uint32_t steps=(uint32_t)(elapsed/CONTROL_TICKS_PER_LOGICAL);
    control_remainder=(uint8_t)(elapsed%CONTROL_TICKS_PER_LOGICAL);
    if(steps>MAX_CATCHUP_LOGICAL_TICKS) {
        const uint32_t lost=steps-MAX_CATCHUP_LOGICAL_TICKS;
        dropped_ticks=UINT32_MAX-dropped_ticks<lost ? UINT32_MAX : dropped_ticks+lost;
        steps=MAX_CATCHUP_LOGICAL_TICKS;
    }
    const uint8_t level=g53_chain_level(in->assist_level);
    const uint32_t speed=in->speed_x100/10u;
    const uint32_t diag_word=(in->safety_cut || !in->torque_sensor_valid) ? 0x10u : 0u;
    if(in->battery_limit_centiamp!=g1_configured_limit) g1_configure(in->battery_limit_centiamp);
    const uint16_t g1_feedback=(uint16_t)in->battery_feedback_centiamp; /* G5300 reads it as u16 */
    const uint16_t g1_soc_factor=(uint16_t)(in->battery_soc_derate_q12>=G53_G1_Q12_ONE ?
        0u : G53_G1_Q12_ONE-in->battery_soc_derate_q12);
    /* Missed ticks use this update's held input. Lost edge history is never
     * reconstructed. Catch-up is bounded; any excess is explicitly counted. */
    while(steps--) {
        g53_chain_input_t ci={0};
        g53_chain_output_t co;
        pa6_ax_observed=g53_ax_step(in->raw_pa6_adc);   /* diagnostics only */
        ci.x=M820_G53_THROTTLE_ENABLED ? pa6_ax_observed : G53_AX_THROTTLE_INACTIVE;
        ci.pas=g53_pas_step(&pas,g53_pas_coordinate(in->pas_ab),level);
        const g53_ad7ec_feedback_t feedback={
            .cadence=ci.pas.cadence,
            .speed_native=(uint16_t)(speed>32767u ? 32767u : speed),
            .d7ec_rider=(uint16_t)port_trace.d7ec_accel,
            .m298=(uint8_t)port_trace.m298
        };
        ci.rider_input_native=g53_ad7ec_step(in->load_ctrl,&feedback).rider_input_native;
        ci.level=level;
        ci.speed_native=(int16_t)feedback.speed_native;
        ci.diag_word=diag_word;
        ci.external_inhibit=0;
        /* G5300 tick order: PI #1 (0x0800C6B4) before BDE8, LIM (0x0800C5BA) after it in
         * phase 5 of 10. g2 stays 1.0: PI #2 is disabled in the G5300 default (M+0x27A = 0). */
        ci.g1_q12=g53_g1_step(&g1_limiter,G53_G1_Q12_ONE,g1_feedback);
        ci.g2_q12=0x1000;
        g53_chain_step(&ci,&co);
        port_trace=co.trace;
        normal_permission=co.normal_permission;
        if(++g1_phase>=10u) g1_phase=0;
        if(g1_phase==5u) {
            const g53_g1_limit_input_t li={
                .bde8_state=(uint8_t)co.trace.bde8_q50,
                .level_pct=level_power[level], .level_pct_state6=100, .base_select=0,
                .soc_factor=g1_soc_factor, .thermal_a=G53_G1_Q12_ONE, .thermal_b=G53_G1_Q12_ONE
            };
            (void)g53_g1_limit_update(&g1_limiter,&li);
        }
    }
    port_trace.raw_pa6_adc=in->raw_pa6_adc;
    port_trace.diag=diag_word;
    port_trace.dropped_logical_ticks=dropped_ticks;
    out->trace=port_trace;
    out->m2aa_native=(uint16_t)port_trace.m2aa;
    out->normal_permission=normal_permission;
    out->iq_request_pre_limits =
        g53_boundary_b_iq_request(out->m2aa_native,in->phase_current_max);
    /* direction_inhibit and real_stop stay native pipeline vetoes, never G53
     * external_inhibit, diag aliases or extra behavioral demand gates. */
}
const g53_port_trace_t *g53_port_trace(void) { return &port_trace; }
const g53_g1_state_t *g53_port_g1_state(void) { return &g1_limiter; }
uint16_t g53_port_pa6_ax_observed(void) { return pa6_ax_observed; }
