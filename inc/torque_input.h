#ifndef TORQUE_INPUT_H_
#define TORQUE_INPUT_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Single owner of the torque conversion chain: automatic zero -> corrected
 * native EBICS signal -> public kilogram-force scale in 0.01 kg units.
 * The zero point is fixed by the autozero normalization and is never
 * writable. Full scale comes from MP.torque_full_scale_native, where 0
 * means "not calibrated" and selects the default. Integer math only.
 * Factory sensor characteristic (reverse engineered): linear 40 mV per kg,
 * so 60 kg = 750 + 2400 = 3150 native; readings above clamp to 60 kg.
 * torque_input_cal_fault() reports overall signal plausibility: implausible
 * zero (startup/coast) or a stuck-high signal that never dips between legs.
 */

#define TORQUE_INPUT_ZERO_NATIVE               750U
#define TORQUE_INPUT_FULL_SCALE_DEFAULT_NATIVE 3150U
#define TORQUE_INPUT_FULL_SCALE_MIN_NATIVE     1000U
#define TORQUE_INPUT_FULL_SCALE_MAX_NATIVE     3300U
#define TORQUE_INPUT_FULL_SCALE_CENTIKG        6000U

void torque_input_init(uint16_t stored_full_scale_native);
void torque_input_startup_zero(int32_t rest_raw_native);
int16_t torque_input_correct(uint16_t raw_native);
void torque_input_coast_update(int16_t torque_corrected_native, bool coast_eligible);
bool torque_input_cal_fault(void);
void torque_input_update(int16_t torque_corrected_native);

uint16_t torque_input_load_centikg(void);
uint16_t torque_input_zero_native(void);
uint16_t torque_input_full_scale_native(void);
uint16_t torque_input_span_native(void);
bool torque_input_full_scale_stored_valid(void);
bool torque_input_set_full_scale_native(uint16_t full_scale_native);

uint16_t torque_input_centikg_to_native_delta(uint16_t centikg);
uint16_t torque_input_native_delta_to_centikg(uint16_t delta_native);

#endif /* TORQUE_INPUT_H_ */
