#ifndef G53_PORT_H
#define G53_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include "g53_port_chain.h"

/* Frozen TQ-06 public inputs. Runtime full phase-current reference is Boundary B only. */
typedef struct {
    uint16_t raw_pa6_adc;
    uint16_t load_ctrl;
    uint8_t pas_ab;
    uint8_t assist_level;
    uint32_t speed_x100;
    uint32_t elapsed_ticks;
    int32_t phase_current_max;
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
void g53_port_update(const g53_port_input_t *in, g53_port_output_t *out);
const g53_port_trace_t *g53_port_trace(void);

#endif
