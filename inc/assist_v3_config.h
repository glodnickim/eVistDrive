#ifndef ASSIST_V3_CONFIG_H_
#define ASSIST_V3_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Assist Behavior V3 - configuration OWNER module (docs/assist-v3/CONFIG_PROTOCOL_V3.md).
 *
 * This module owns the user-tunable V3 parameters (the "V3 block"): RAM state, compiled
 * defaults, the wire block codec, the CAN transfer state machine, and the append-only flash
 * log. It implements NO behaviour: the pipeline reads the values through the getters below.
 * It is hardware free - the flash page is reached through assist_v3_flash_t - so the same
 * object file runs on the target and in the host tests.
 *
 * Context rule: every function is called from the main loop (processCAN_Rx and the persist
 * service both run there). The getters must be called from the same context, like
 * apply_assist_levels().
 */

#define ASSIST_V3_LEVELS            5U
#define ASSIST_V3_RECORD_WORDS      8U
#define ASSIST_V3_RECORD_STRIDE     16U
#define ASSIST_V3_BLOCK_LEN         114U
#define ASSIST_V3_CAPS_LEN          26U
#define ASSIST_V3_BLOCK_HEADER_LEN  16U
#define ASSIST_V3_SCHEMA_ID         1U
#define ASSIST_V3_SCHEMA_VERSION    1U

/* Parameter ids == bit positions in param_mask. */
#define ASSIST_V3_PARAM_RESPONSE        0U
#define ASSIST_V3_PARAM_START_RESPONSE  1U
#define ASSIST_V3_PARAM_CARRY_STRENGTH  2U
#define ASSIST_V3_PARAM_CARRY_EXTENT    3U
#define ASSIST_V3_PARAM_MAX_TORQUE      4U
#define ASSIST_V3_PARAM_ASSIST_RANGE    5U
#define ASSIST_V3_PARAM_ENGINE          15U

/* Milestone C firmware implements response (bit 0) and engine (bit 15) only. */
#define ASSIST_V3_PARAM_MASK        ((uint16_t)((1U << ASSIST_V3_PARAM_RESPONSE) | (1U << ASSIST_V3_PARAM_ENGINE)))
#define ASSIST_V3_PARAM_COUNT       7U

/* Caps bits 0..5 of the contract (behavior_v1, effective, saved, defaults, deferred persist, revert).
 * An image built without ASSIST_V3 clears bit 0 (no V3 behaviour) and rejects engine = V3 with
 * reason 4 (CONFIG_PROTOCOL_V3 section 4, REVIEW-T #20); assist_v3_config_caps() is the value this
 * image reports and accepts. */
#define ASSIST_V3_CAPS              0x0000003FUL
#define ASSIST_V3_CAP_BEHAVIOR      0x00000001UL

#define ASSIST_V3_UNSET             0xFFFFU   /* "use the firmware default" / reserved */
#define ASSIST_V3_RESPONSE_MAX      100U
#define ASSIST_V3_ENGINE_G5300      0U
#define ASSIST_V3_ENGINE_V3         1U

/* Reject reasons (ERROR_ACK payload byte 0). */
#define ASSIST_V3_REASON_SCHEMA     0U
#define ASSIST_V3_REASON_LENGTH     1U
#define ASSIST_V3_REASON_CRC        2U
#define ASSIST_V3_REASON_RANGE      3U
#define ASSIST_V3_REASON_CAPABILITY 4U
#define ASSIST_V3_REASON_BUSY       5U
#define ASSIST_V3_REASON_STALE      6U
#define ASSIST_V3_REASON_RESERVED   7U
#define ASSIST_V3_INDEX_NONE        0xFFU

/* CAN ids (allocation pending in COMMUNICATION_REGISTRY, see audit D). */
#define ASSIST_V3_CMD_CAPS          0x6035U
#define ASSIST_V3_CMD_BLOCK         0x6036U
#define ASSIST_V3_CMD_CONTROL       0x6037U
#define ASSIST_V3_SOURCE_TOOL       5U

/* 0x6037 ops. */
#define ASSIST_V3_OP_PERSIST        1U
#define ASSIST_V3_OP_REVERT         2U
#define ASSIST_V3_OP_DEFAULTS       3U

/* 0x6035 persist_state / flash_record_state. */
#define ASSIST_V3_PERSIST_CLEAN     0U
#define ASSIST_V3_PERSIST_DIRTY     1U
#define ASSIST_V3_PERSIST_PENDING   2U
#define ASSIST_V3_PERSIST_FAILED    3U
#define ASSIST_V3_FLASH_VALID       0U
#define ASSIST_V3_FLASH_ABSENT      1U
#define ASSIST_V3_FLASH_NEWER       2U
#define ASSIST_V3_FLASH_CORRUPT     3U

/* View selector of READ 0x6036. */
#define ASSIST_V3_VIEW_SAVED        0U
#define ASSIST_V3_VIEW_EFFECTIVE    1U
#define ASSIST_V3_VIEW_DEFAULTS     2U

/* Idle time after which a live V3 transfer is dropped (any received frame restarts it). */
#define ASSIST_V3_XFER_TIMEOUT_MS   1000U

/* Flash log: CONFIG_A page, 16 slots of 128 B, record layout in assist_v3_config.c. */
#define ASSIST_V3_FLASH_PAGE_SIZE   2048U
#define ASSIST_V3_SLOT_SIZE         128U
#define ASSIST_V3_SLOT_COUNT        (ASSIST_V3_FLASH_PAGE_SIZE / ASSIST_V3_SLOT_SIZE)

/* Raw parameter store: 0xFFFF = "firmware default". global[0] = engine. level[i] = HMI level i+1. */
typedef struct {
	uint16_t global[ASSIST_V3_RECORD_WORDS];
	uint16_t level[ASSIST_V3_LEVELS][ASSIST_V3_RECORD_WORDS];
} assist_v3_values_t;

/* Result of one wire event. target = node that gets the frame. */
typedef enum {
	ASSIST_V3_REPLY_NONE = 0,
	ASSIST_V3_REPLY_ACK,       /* result: NORMAL_ACK */
	ASSIST_V3_REPLY_ERROR,     /* result: ERROR_ACK, payload [reason, index] */
	ASSIST_V3_REPLY_DECL_ACK   /* transport ACK of an accepted declaration (flow control, not a result) */
} assist_v3_reply_kind_t;

typedef struct {
	uint8_t kind;
	uint8_t reason;
	uint8_t index;
	uint8_t target;
} assist_v3_reply_t;

/* Flash page access. page = memory-mapped base (read), erase/program = HAL wrappers.
 * program() receives a word-aligned offset and a length that is a multiple of 4. */
typedef struct {
	const uint8_t *page;
	bool (*erase)(void);
	bool (*program)(uint32_t offset, const uint8_t *data, uint32_t len);
} assist_v3_flash_t;

/* ---- pure codec (no state) ---- */
uint16_t assist_v3_crc16(const uint8_t *data, uint16_t len);
uint32_t assist_v3_crc32(const uint8_t *data, uint32_t len);
void assist_v3_block_encode(const assist_v3_values_t *values, uint32_t caps,
                            uint16_t generation, uint8_t out[ASSIST_V3_BLOCK_LEN]);
/* Validates the whole block in contract order. No state is touched. */
bool assist_v3_block_validate(const uint8_t *blk, uint16_t len, uint16_t current_generation,
                              assist_v3_values_t *out, uint8_t *reason, uint8_t *index);
uint16_t assist_v3_generation_next(uint16_t generation);

/* ---- lifecycle ---- */
/* Boot: defaults, then the newest valid record of the log. Also the "restart" of the tests. */
void assist_v3_config_init(const assist_v3_flash_t *flash);

/* ---- consumers (effective values, defaults resolved) ---- */
uint16_t assist_v3_config_get(uint8_t level_1_to_5, uint8_t param_id); /* 0xFFFF = no such param */
uint8_t  assist_v3_config_response_pct(uint8_t level_1_to_5);          /* 0 for an invalid level */
/* Release (Response) of one HMI level, 0..100, for the V3 trajectory's fall while pedalling.
 * Milestone C: the v1 per-level response; the v2 resolver replaces the body (one call site). */
uint8_t  assist_v3_effective_release_pct(uint8_t hmi_level_1_to_5);    /* 0 for an invalid level */
bool     assist_v3_config_engine_requested(void);                      /* true = V3 (false without ASSIST_V3) */
/* TRUTHFUL: true while V3 publishes Iq. False (G5300) at boot until the pipeline latch reports. */
bool     assist_v3_config_engine_active(void);
void     assist_v3_config_set_engine_active(bool v3_active);           /* pipeline latch reports here */
uint32_t assist_v3_config_caps(void);                                  /* caps this image reports/accepts */

/* ---- state for tests / tools ---- */
uint16_t assist_v3_config_generation(void);
uint8_t  assist_v3_config_persist_state(void);
uint8_t  assist_v3_config_flash_record_state(void);
void     assist_v3_config_ram_values(assist_v3_values_t *out);
void     assist_v3_config_saved_values(assist_v3_values_t *out);
bool     assist_v3_config_xfer_live(void);

/* ---- CAN protocol ---- */
bool assist_v3_config_owns_command(uint16_t command);
/* READ 0x6035 / 0x6036. On success *out_len > 0 and out[] holds the multiframe payload. */
assist_v3_reply_t assist_v3_config_can_read(uint8_t source, uint16_t command, uint8_t dlen,
                                            const uint8_t *data, uint8_t *out, uint8_t *out_len);
/* WRITE 0x6036 declaration (DLC 1, declared length in data[0]). */
assist_v3_reply_t assist_v3_config_can_declare(uint8_t source, uint8_t declared_len, uint32_t now_ms);
/* Transfer frame. frame_no 0 = START, last = END (is_end). */
assist_v3_reply_t assist_v3_config_can_frame(uint8_t source, uint8_t frame_no, bool is_end,
                                             const uint8_t *data, uint8_t dlen, uint32_t now_ms);
/* WRITE 0x6037 [op]. */
assist_v3_reply_t assist_v3_config_can_control(uint8_t source, uint8_t dlen, const uint8_t *data);
/* Another command's declaration arrived. Returns false if it must be ignored (foreign owner);
 * when a live V3 transfer is dropped, *abort_reply carries its single ERROR result. */
bool assist_v3_config_other_declaration(uint8_t source, assist_v3_reply_t *abort_reply);

/* ---- main loop service: transfer timeout + deferred persist at standstill ---- */
assist_v3_reply_t assist_v3_config_service(uint32_t now_ms, bool standstill);

#endif /* ASSIST_V3_CONFIG_H_ */
