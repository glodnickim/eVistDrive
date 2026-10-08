#include "soc_core.h"

#include "config.h"

#include <stddef.h>

int8_t soc_core_calculate_ocv(uint16_t voltage_mv, uint8_t cells_in_series)
{
	/* Measured LG M58T discharge curve @3A (legacy production table, unchanged). */
	static const float voltages[] = {
		2.799f, 2.968f, 3.086f, 3.247f, 3.450f, 3.569f,
		3.681f, 3.774f, 3.853f, 3.946f, 3.989f, 4.070f
	};
	static const float soc_values[] = {
		0.0f, 5.0f, 10.0f, 20.0f, 30.0f, 40.0f,
		50.0f, 60.0f, 70.0f, 80.0f, 90.0f, 100.0f
	};
	const size_t length = sizeof(voltages) / sizeof(voltages[0]);

	if (cells_in_series == 0U) {
		return 0;
	}
	float cell_voltage = (float)voltage_mv / ((float)cells_in_series * 1000.0f);
	if (cell_voltage <= voltages[0]) {
		return (int8_t)soc_values[0];
	}
	if (cell_voltage >= voltages[length - 1U]) {
		return (int8_t)soc_values[length - 1U];
	}

	for (size_t i = 0U; i + 1U < length; i++) {
		if (cell_voltage < voltages[i + 1U]) {
			float slope = (soc_values[i + 1U] - soc_values[i]) /
				(voltages[i + 1U] - voltages[i]);
			float soc = soc_values[i] + slope * (cell_voltage - voltages[i]);
			return (int8_t)soc;
		}
	}
	return (int8_t)soc_values[length - 1U];
}

uint8_t soc_core_can_mark_full_at_boot(uint32_t voltage_mv, int32_t battery_current_ma,
    uint16_t full_magic, uint16_t full_pack_10mv)
{
	const uint32_t full_mv = (uint32_t)full_pack_10mv * 10U;
	if (full_magic != SOC_FULL_MAGIC || full_mv < SOC_FULL_PACK_MIN_MV ||
	    full_mv > SOC_FULL_PACK_MAX_MV) {
		return 0U;
	}
	if (battery_current_ma <= -I_REST_MA || battery_current_ma >= I_REST_MA) {
		return 0U;
	}
	return voltage_mv >= (full_mv + SOC_FULL_EARLY_MARGIN_MV) ? 1U : 0U;
}

float soc_core_limp_factor(float soc, uint8_t limit_pct, uint8_t stage2_pct)
{
	if (limit_pct == LIMP_DISABLED || limit_pct == 0U) {
		return 1.0f;
	}
	float floor_factor = (float)LIMP_FLOOR_PCT / 100.0f;
	if (soc < 0.0f) {
		soc = 0.0f;
	}
	if (soc >= (float)limit_pct) {
		return 1.0f;
	}

	float factor;
	if (stage2_pct != LIMP_DISABLED && stage2_pct > 0U && stage2_pct < limit_pct) {
		float stage2_factor = (float)LIMP_STAGE2_PCT / 100.0f;
		if (soc > (float)stage2_pct) {
			factor = stage2_factor + (1.0f - stage2_factor) *
				(soc - (float)stage2_pct) / (float)(limit_pct - stage2_pct);
		} else {
			factor = floor_factor + (stage2_factor - floor_factor) *
				soc / (float)stage2_pct;
		}
	} else {
		factor = floor_factor + (1.0f - floor_factor) * soc / (float)limit_pct;
	}
	if (factor < floor_factor) {
		factor = floor_factor;
	}
	if (factor > 1.0f) {
		factor = 1.0f;
	}
	return factor;
}

void soc_core_step_1hz(soc_core_state_t *state, const soc_core_input_t *input)
{
	if (state == NULL || input == NULL || input->capacity_estimated_mah == 0U) {
		return;
	}

	/* FW-018 boot-time full-charge detection, unchanged from main.c. */
	if (!state->boot_full_done) {
		if (input->soc_full_magic == SOC_FULL_MAGIC) {
			if (input->voltage_mv < state->boot_vmin_mv) {
				state->boot_vmin_mv = (uint16_t)input->voltage_mv;
			}
			if (input->voltage_mv > state->boot_vmax_mv) {
				state->boot_vmax_mv = (uint16_t)input->voltage_mv;
			}
			if (++state->boot_settle_s >= SOC_FULL_BOOT_SETTLE_S) {
				state->boot_full_done = 1U;
				if ((uint16_t)(state->boot_vmax_mv - state->boot_vmin_mv) <=
					SOC_FULL_BOOT_STABLE_MV &&
					input->voltage_mv >= (uint32_t)input->soc_full_pack_10mv * 10U) {
					state->remaining_mah = (float)input->capacity_estimated_mah;
					state->soc_real = 100.0f;
					state->soc_display = 100.0f;
					state->full_anchor = 1U;
					state->anchor_start_mah = state->remaining_mah;
				}
			}
		} else {
			state->boot_full_done = 1U;
		}
	}

	/* Coulomb counter. Positive delta_mah means discharge. */
	state->remaining_mah -= input->delta_mah;
	if (state->remaining_mah > (float)input->capacity_estimated_mah) {
		state->remaining_mah = (float)input->capacity_estimated_mah;
	}
	if (state->remaining_mah < 0.0f) {
		state->remaining_mah = 0.0f;
	}
	state->soc_real = state->remaining_mah /
		(float)input->capacity_estimated_mah * 100.0f;

	/* Existing IR-compensated OCV lookup. */
	uint8_t cells = (uint8_t)((float)input->system_voltage / 3.6f);
	float current_a = (float)input->battery_current_ma / 1000.0f;
	float compensated_mv = (float)input->voltage_mv +
		current_a * (float)input->r_batt_mohm;
	if (compensated_mv < 0.0f) {
		compensated_mv = 0.0f;
	}
	if (compensated_mv > 65535.0f) {
		compensated_mv = 65535.0f;
	}
	state->soc_voltage = soc_core_calculate_ocv((uint16_t)compensated_mv, cells);

	/* Slow OCV correction only at rest. */
	if (input->battery_current_ma < I_REST_MA && input->battery_current_ma > -I_REST_MA) {
		if (state->rest_seconds < 65000U) {
			state->rest_seconds++;
		}
		if (state->rest_seconds >= REST_TIME_S) {
			state->soc_real += OCV_CORR_GAIN *
				((float)state->soc_voltage - state->soc_real);
			state->remaining_mah = state->soc_real / 100.0f *
				(float)input->capacity_estimated_mah;
		}
	} else {
		state->rest_seconds = 0U;
	}

	/* Display follows the integrated charge every second, WITHOUT the 1%/min cap.
	 * The cap applies only to the estimator correction, not to real consumption.
	 * Otherwise a sustained 12-15 A discharge on a 17.4 Ah pack outruns the HMI,
	 * and a reboot suddenly exposes the hidden SOC_real - SOC_display gap. */
	const float display_after_charge = state->soc_display -
		(input->delta_mah / (float)input->capacity_estimated_mah) * 100.0f;
	float diff = state->soc_real - display_after_charge;
	float step = SOC_DISP_GAIN * diff;
	float max_step = SOC_DISP_MAX_STEP / 60.0f;
	if (state->soc_real < 10.0f) {
		step = diff;
	}
	if (step > max_step) {
		step = max_step;
	}
	if (step < -max_step) {
		step = -max_step;
	}
	state->soc_display = display_after_charge + step;
	if (state->soc_display < 0.0f) {
		state->soc_display = 0.0f;
	}
	if (state->soc_display > 100.0f) {
		state->soc_display = 100.0f;
	}

	/* Full-charge anchor: hold display at 100%% until 1%% capacity is actually consumed. */
	if (state->full_anchor) {
		if ((state->anchor_start_mah - state->remaining_mah) <
			SOC_FULL_RELEASE_FRAC * (float)input->capacity_estimated_mah) {
			state->soc_display = 100.0f;
		} else {
			state->full_anchor = 0U;
			state->soc_display = state->soc_real; /* release 100%% at the true 1%% consumed */
		}
	}
}
