#ifndef ASSIST_V3_CONFIG_H_
#define ASSIST_V3_CONFIG_H_
#include <stdbool.h>
#include <stdint.h>
/* CONFIG_PROTOCOL_V3 v2 rev 2: main-loop owner. */
#define ASSIST_V3_LEVELS 5U
#define ASSIST_V3_MODES 6U
#define ASSIST_V3_PARAM_COUNT 24U
#define ASSIST_V3_BLOCK_HEADER_LEN 16U
#define ASSIST_V3_PROFILE_LEN 66U
#define ASSIST_V3_EFFECTIVE_LEN 90U
#define ASSIST_V3_GLOBAL_LEN 34U
#define ASSIST_V3_BLOCK_LEN ASSIST_V3_GLOBAL_LEN
#define ASSIST_V3_CAPS_LEN 30U
#define ASSIST_V3_SCHEMA_VERSION 2U
#define ASSIST_V3_PROFILE_SCHEMA 2U
#define ASSIST_V3_GLOBAL_SCHEMA 3U
#define ASSIST_V3_CAPS 0x0000007FUL
#define ASSIST_V3_CAP_BEHAVIOR 1UL
#define ASSIST_V3_PARAM_MASK 0x001F3F3FUL
#define ASSIST_V3_UNSET 0xFFFFU
#define ASSIST_V3_KEEP 0xFFFEU
#define ASSIST_V3_ENGINE_G5300 0U
#define ASSIST_V3_ENGINE_V3 1U
#define ASSIST_V3_PARAM_ASSIST 0U
#define ASSIST_V3_PARAM_MAX_TORQUE 1U
#define ASSIST_V3_PARAM_MAX_POWER 2U
#define ASSIST_V3_PARAM_RESPONSE 3U
#define ASSIST_V3_PARAM_START 4U
#define ASSIST_V3_PARAM_CARRY 5U
#define ASSIST_V3_PARAM_ASSIST_BASE 8U
#define ASSIST_V3_PARAM_RANGE_MIN 9U
#define ASSIST_V3_PARAM_RANGE_MAX 10U
#define ASSIST_V3_PARAM_PROGRESSION 11U
#define ASSIST_V3_PARAM_ATTACK 12U
#define ASSIST_V3_PARAM_RELEASE 13U
#define ASSIST_V3_PARAM_CARRY_STRENGTH 16U
#define ASSIST_V3_PARAM_CARRY_TIME 17U
#define ASSIST_V3_PARAM_CARRY_DISTANCE 18U
#define ASSIST_V3_PARAM_TERRAIN 19U
#define ASSIST_V3_PARAM_CADENCE_BIAS 20U
#define ASSIST_V3_REASON_SCHEMA 0U
#define ASSIST_V3_REASON_LENGTH 1U
#define ASSIST_V3_REASON_CRC 2U
#define ASSIST_V3_REASON_RANGE 3U
#define ASSIST_V3_REASON_CAPABILITY 4U
#define ASSIST_V3_REASON_BUSY 5U
#define ASSIST_V3_REASON_STALE 6U
#define ASSIST_V3_REASON_RESERVED 7U
#define ASSIST_V3_INDEX_NONE 0xFFU
#define ASSIST_V3_CMD_CAPS 0x6035U
#define ASSIST_V3_CMD_BLOCK 0x6036U
#define ASSIST_V3_CMD_CONTROL 0x6037U
#define ASSIST_V3_SOURCE_TOOL 5U
#define ASSIST_V3_OP_PERSIST 1U
#define ASSIST_V3_OP_REVERT 2U
#define ASSIST_V3_OP_DEFAULTS 3U
#define ASSIST_V3_OP_RESTORE_MODE 4U
#define ASSIST_V3_PERSIST_CLEAN 0U
#define ASSIST_V3_PERSIST_DIRTY 1U
#define ASSIST_V3_PERSIST_PENDING 2U
#define ASSIST_V3_PERSIST_FAILED 3U
#define ASSIST_V3_PERSIST_STALE 4U
#define ASSIST_V3_FLASH_VALID 0U
#define ASSIST_V3_FLASH_ABSENT 1U
#define ASSIST_V3_FLASH_NEWER 2U
#define ASSIST_V3_FLASH_CORRUPT 3U
#define ASSIST_V3_FLASH_V1_IGNORED 4U
#define ASSIST_V3_VIEW_SAVED 0U
#define ASSIST_V3_VIEW_EFFECTIVE 1U
#define ASSIST_V3_VIEW_DEFAULTS 2U
#define ASSIST_V3_VIEW_CONFIGURED 3U
#define ASSIST_V3_XFER_TIMEOUT_MS 1000U
#define ASSIST_V3_FLASH_PAGE_SIZE 2048U
#define ASSIST_V3_SLOT_SIZE 320U
#define ASSIST_V3_SLOT_COUNT (ASSIST_V3_FLASH_PAGE_SIZE / ASSIST_V3_SLOT_SIZE)
typedef struct {
    uint16_t mode[ASSIST_V3_MODES][ASSIST_V3_PARAM_COUNT];
    uint16_t engine;
    uint8_t level_mode[ASSIST_V3_LEVELS];
    uint8_t reserved;
} assist_v3_values_t;
typedef struct { uint16_t value[ASSIST_V3_PARAM_COUNT]; uint8_t source[ASSIST_V3_PARAM_COUNT]; } assist_v3_effective_t;
typedef enum { ASSIST_V3_REPLY_NONE=0, ASSIST_V3_REPLY_ACK, ASSIST_V3_REPLY_ERROR, ASSIST_V3_REPLY_DECL_ACK } assist_v3_reply_kind_t;
typedef struct { uint8_t kind, reason, index, target; uint16_t generation; } assist_v3_reply_t;
typedef struct { const uint8_t *page; bool (*erase)(void); bool (*program)(uint32_t offset,const uint8_t *data,uint32_t len); } assist_v3_flash_t;
uint16_t assist_v3_crc16(const uint8_t *data,uint16_t len);
uint32_t assist_v3_crc32(const uint8_t *data,uint32_t len);
uint16_t assist_v3_generation_next(uint16_t generation);
void assist_v3_block_encode(const assist_v3_values_t *values,uint32_t caps,uint16_t generation,uint8_t out[ASSIST_V3_BLOCK_LEN]);
bool assist_v3_block_validate(const uint8_t *blk,uint16_t len,uint16_t current_generation,assist_v3_values_t *out,uint8_t *reason,uint8_t *index);
void assist_v3_config_init(const assist_v3_flash_t *flash);
const assist_v3_effective_t *assist_v3_effective(uint8_t hmi_level);
uint8_t assist_v3_effective_release_pct(uint8_t hmi_level);
void assist_v3_config_set_legacy(uint8_t level,uint16_t ratio,uint8_t power,uint8_t accel);
void assist_v3_config_set_pack_voltage_mv(uint32_t mv);
bool assist_v3_config_engine_requested(void);
bool assist_v3_config_engine_active(void);
void assist_v3_config_set_engine_active(bool active);
uint32_t assist_v3_config_caps(void);
uint16_t assist_v3_config_generation(void);
uint8_t assist_v3_config_persist_state(void);
uint8_t assist_v3_config_flash_record_state(void);
void assist_v3_config_ram_values(assist_v3_values_t *out);
void assist_v3_config_saved_values(assist_v3_values_t *out);
bool assist_v3_config_xfer_live(void);
bool assist_v3_config_owns_command(uint16_t command);
assist_v3_reply_t assist_v3_config_can_read(uint8_t source,uint16_t command,uint8_t dlen,const uint8_t *data,uint8_t *out,uint8_t *out_len);
assist_v3_reply_t assist_v3_config_can_declare(uint8_t source,uint8_t declared_len,uint32_t now_ms);
assist_v3_reply_t assist_v3_config_can_frame(uint8_t source,uint8_t frame_no,bool is_end,const uint8_t *data,uint8_t dlen,uint32_t now_ms);
assist_v3_reply_t assist_v3_config_can_control(uint8_t source,uint8_t dlen,const uint8_t *data);
bool assist_v3_config_other_declaration(uint8_t source,assist_v3_reply_t *abort_reply);
assist_v3_reply_t assist_v3_config_service(uint32_t now_ms,bool standstill);
#endif
