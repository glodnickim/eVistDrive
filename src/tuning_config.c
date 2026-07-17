#include "tuning_config.h"

#define CONTROL_TICKS_PER_MS 4
#define TUNING_MAGIC0 0x54U /* 'T' */
#define TUNING_MAGIC1 0x55U /* 'U' */
#define TUNING_VERSION 1U

static uint16_t rise_slow_ms = 600U;
static uint16_t rise_fast_ms = 300U;
static uint16_t fall_slow_ms = 1000U;
static uint16_t fall_fast_ms = 140U;
static uint16_t cadence_step = 20U;

static uint16_t clamp_ms(uint16_t value)
{
	if (value < TUNING_RAMP_MS_MIN) {
		return TUNING_RAMP_MS_MIN;
	}
	return (value > TUNING_RAMP_MS_MAX) ? TUNING_RAMP_MS_MAX : value;
}

static uint16_t clamp_cadence_step(uint16_t value)
{
	if (value < TUNING_CADENCE_STEP_MIN) {
		return TUNING_CADENCE_STEP_MIN;
	}
	return (value > TUNING_CADENCE_STEP_MAX) ? TUNING_CADENCE_STEP_MAX : value;
}

int32_t tuning_config_ramp_up_slow_ticks(void)
{
	return (int32_t)rise_slow_ms * CONTROL_TICKS_PER_MS;
}

int32_t tuning_config_ramp_up_fast_ticks(void)
{
	return (int32_t)rise_fast_ms * CONTROL_TICKS_PER_MS;
}

int32_t tuning_config_ramp_down_slow_ticks(void)
{
	return (int32_t)fall_slow_ms * CONTROL_TICKS_PER_MS;
}

int32_t tuning_config_ramp_down_fast_ticks(void)
{
	return (int32_t)fall_fast_ms * CONTROL_TICKS_PER_MS;
}

uint16_t tuning_config_cadence_step(void)
{
	return cadence_step;
}

static void put_u16(uint8_t *buffer, uint16_t value)
{
	buffer[0] = (uint8_t)(value & 0xFFU);
	buffer[1] = (uint8_t)(value >> 8);
}

static uint16_t get_u16(const uint8_t *buffer)
{
	return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
}

static uint16_t tuning_blob_crc16(const uint8_t *buffer, uint16_t length)
{
	uint16_t crc = 0xFFFFU;
	for (uint16_t i = 0; i < length; i++) {
		crc ^= (uint16_t)buffer[i] << 8;
		for (uint8_t bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000U) ?
				(uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

uint16_t tuning_config_serialize(uint8_t *buffer)
{
	if (buffer == 0) {
		return 0;
	}
	buffer[0] = TUNING_MAGIC0;
	buffer[1] = TUNING_MAGIC1;
	buffer[2] = TUNING_VERSION;
	buffer[3] = 0;
	put_u16(&buffer[4], rise_slow_ms);
	put_u16(&buffer[6], rise_fast_ms);
	put_u16(&buffer[8], fall_slow_ms);
	put_u16(&buffer[10], fall_fast_ms);
	put_u16(&buffer[12], cadence_step);
	put_u16(&buffer[14], tuning_blob_crc16(buffer, 14));
	return TUNING_BLOB_LEN;
}

bool tuning_config_apply_blob(const uint8_t *buffer, uint16_t length)
{
	if (buffer == 0 ||
		length < TUNING_BLOB_LEN ||
		buffer[0] != TUNING_MAGIC0 ||
		buffer[1] != TUNING_MAGIC1 ||
		buffer[2] != TUNING_VERSION) {
		return false;
	}
	if (get_u16(&buffer[14]) != tuning_blob_crc16(buffer, 14)) {
		return false;
	}
	rise_slow_ms = clamp_ms(get_u16(&buffer[4]));
	rise_fast_ms = clamp_ms(get_u16(&buffer[6]));
	fall_slow_ms = clamp_ms(get_u16(&buffer[8]));
	fall_fast_ms = clamp_ms(get_u16(&buffer[10]));
	cadence_step = clamp_cadence_step(get_u16(&buffer[12]));
	return true;
}
