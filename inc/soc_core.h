#ifndef SOC_CORE_H_
#define SOC_CORE_H_

#include <stdint.h>

/*
 * FW-144 / Level-4 testability boundary.
 *
 * This module owns ONLY the pure/stateful SOC estimator math that used to live inline in
 * main.c: coulomb integration, IR-compensated OCV correction, display filtering and the
 * boot-time 100%% full-charge anchor. It deliberately does NOT own flash persistence, range
 * learning or capacity-cycle learning; main.c keeps those platform/product responsibilities.
 *
 * The reason for the split is verification, not a new SOC algorithm. The exact same state
 * transition can now run on GD32 and in the virtual-bike SIL, so a battery/SOC test cannot
 * accidentally validate a Python/C reimplementation instead of production math.
 */

typedef struct {
	float remaining_mah;
	float soc_real;
	float soc_display;
	int8_t soc_voltage;
	uint32_t rest_seconds;
	uint8_t full_anchor;
	uint8_t boot_full_done;
	uint8_t boot_settle_s;
	uint16_t boot_vmin_mv;
	uint16_t boot_vmax_mv;
	float anchor_start_mah;
} soc_core_state_t;

typedef struct {
	uint32_t voltage_mv;
	int32_t battery_current_ma;
	float delta_mah;                    /* signed charge integrated over this 1 s update */
	uint16_t capacity_estimated_mah;
	uint16_t r_batt_mohm;
	int8_t system_voltage;
	uint16_t soc_full_magic;
	uint16_t soc_full_pack_10mv;
} soc_core_input_t;

/* Existing production LG-M58T OCV lookup, pack voltage in mV. */
int8_t soc_core_calculate_ocv(uint16_t voltage_mv, uint8_t cells_in_series);

/* Conservative full-at-boot predicate. Requires configured full threshold, low current,
 * and a margin above threshold. Does not change the 10 s verification fallback. */
uint8_t soc_core_can_mark_full_at_boot(uint32_t voltage_mv, int32_t battery_current_ma,
    uint16_t full_magic, uint16_t full_pack_10mv);

/* Existing production low-SOC power scale, expressed as 0.0..1.0. */
float soc_core_limp_factor(float soc, uint8_t limit_pct, uint8_t stage2_pct);

/* One production 1 Hz SOC estimator transition. */
void soc_core_step_1hz(soc_core_state_t *state, const soc_core_input_t *input);

#endif /* SOC_CORE_H_ */
