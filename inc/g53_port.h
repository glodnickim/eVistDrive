#ifndef G53_PORT_H
#define G53_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include "g53_port_chain.h"
#include "g53_g1_limiter.h"

/* Frozen TQ-06 public inputs. Runtime full phase-current reference is Boundary B only. */
typedef struct {
    uint16_t raw_pa6_adc;
    uint16_t load_ctrl;
    uint8_t pas_ab;
    uint8_t assist_level;
    uint32_t speed_x100;
    uint32_t elapsed_ticks;
    int32_t phase_current_max;
    /* TASK-EVD-TQ-06-G1 / ADR-013: G53 PI #1 battery-current limiter (g1). Feedback is the
     * fast-tap battery current; the limit is the configured battery current limit. Both 0.01 A. */
    int32_t battery_feedback_centiamp;
    int32_t battery_limit_centiamp;
    /* Step 2 (ADR-013 D3): how much the SOC knee takes away, Q12. 0 = nothing - the value every
     * zero-initialised caller gets; the LIM SOC factor is 0x1000 - derate. Temperature is not an
     * input here (variant A): the M820 Iq thermal derate stays in ap2_limits. */
    uint16_t battery_soc_derate_q12;
    bool torque_sensor_valid;
    bool direction_inhibit;
    bool real_stop;
    bool safety_cut;
} g53_port_input_t;

typedef struct {
    int32_t iq_request_pre_limits;
    uint16_t m2aa_native;
    bool normal_permission;
    g53_port_trace_t trace;
} g53_port_output_t;

void g53_port_init(void);
void g53_port_reset(void);
void g53_port_set_levels(const uint8_t accel[10], const uint16_t ratio[10], const uint8_t power[10]);
void g53_port_update(const g53_port_input_t *in, g53_port_output_t *out);
const g53_port_trace_t *g53_port_trace(void);
/* The G53 PI #1 limiter state (g1, its limit and setpoint), for diagnostics only. */
const g53_g1_state_t *g53_port_g1_state(void);
/* Filtered PA6 (Boundary A-x arithmetic) for diagnostics. On M820 it never reaches the G53
 * throttle input: throttle is disabled by product policy (src/g53_port.c). */
uint16_t g53_port_pa6_ax_observed(void);
/* ASSIST-V3 read-only accessors (state after the last g53_port_update()). */
/* EB74 zero (750 at start, 750..995 once auto-zeroed; 0 before the first update - gate on armed). */
uint16_t g53_port_eb74_zero(void);
/* EB74 active threshold: 960 during the check, then zero+245 (engage) or 820 (while D7EC drives). */
uint16_t g53_port_eb74_threshold(void);
/* EB74 armed: startup window done and the pedal seen unloaded for the check after reset. */
bool g53_port_eb74_armed(void);
/* G53 PAS crank stopped: movement cleared (true-stop timeout fired, or no transition since reset). */
bool g53_port_pas_true_stop(void);
/* G53 PAS signed cadence (M+0x92 / D7EC fp). */
int16_t g53_port_pas_cadence(void);

#endif
