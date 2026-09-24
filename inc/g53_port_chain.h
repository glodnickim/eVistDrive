#ifndef G53_PORT_CHAIN_H
#define G53_PORT_CHAIN_H

#include <stdbool.h>
#include <stdint.h>
#include "g53_port_pas.h"

typedef struct {
    uint16_t x;
    uint16_t rider_input_native;
    g53_pas_output_t pas;
    uint8_t level;
    uint32_t diag_word;
    uint8_t external_inhibit;
    uint16_t g1_q12;
    uint16_t g2_q12;
    int16_t speed_native;
} g53_chain_input_t;

typedef struct {
    uint32_t logical_tick;
    uint8_t fast_phase;
    uint8_t supervisor_phase;
    uint16_t raw_pa6_adc;
    uint16_t x;
    uint16_t rider_input_native;
    uint16_t d7ec_rider;
    uint16_t d7ec_envelope;
    uint16_t d7ec_accel;
    uint16_t e1e8_output;
    uint8_t e1e8_state;
    uint8_t bde8_q50;
    int16_t pas_direction;
    uint16_t m2aa;
    uint8_t m298;
} g53_port_trace_t;

typedef struct {
    uint16_t m2aa_native;
    bool normal_permission;
    g53_port_trace_t trace;
} g53_chain_output_t;

void g53_chain_reset(void);
void g53_chain_step(const g53_chain_input_t *in, g53_chain_output_t *out);

#endif
