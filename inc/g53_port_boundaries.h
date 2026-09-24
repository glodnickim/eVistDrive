#ifndef G53_PORT_BOUNDARIES_H
#define G53_PORT_BOUNDARIES_H

#include <stdint.h>

/* TQ-06: separate PA6 and CLU boundaries; neither owns native safety or Iq. */
typedef struct {
    int16_t cadence;
    uint16_t speed_native;
    uint16_t d7ec_rider;
    uint8_t m298;
} g53_ad7ec_feedback_t;

typedef struct {
    uint16_t pre_eb74;
    uint16_t rider_input_native;
    uint16_t zero;
    uint16_t threshold;
    uint16_t startup_count;
    uint8_t check_count;
    uint8_t m29e;
} g53_ad7ec_output_t;

void g53_ax_reset(void);
uint16_t g53_ax_step(uint16_t raw_pa6_adc);
void g53_ad7ec_reset(void);
g53_ad7ec_output_t g53_ad7ec_step(uint16_t load_ctrl,
                                 const g53_ad7ec_feedback_t *feedback);
int32_t g53_boundary_b_iq_request(uint16_t m2aa_native, int32_t phase_current_max);

#endif
