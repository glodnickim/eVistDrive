#include "assist_v3_config.h"

#include "config.h"   /* ASSIST_V3: whether this image can run the V3 engine at all */

#include <string.h>

/*
 * Assist Behavior V3 configuration owner. See inc/assist_v3_config.h and
 * docs/assist-v3/CONFIG_PROTOCOL_V3.md (the normative contract).
 *
 * Flash record (one slot = 128 B, 16 slots in the 2 KB CONFIG_A page, little-endian):
 *   [0..3]    magic 'A','3','C','F'
 *   [4]       schema_id      [5] schema_version   [6] record_stride   [7] level_count
 *   [8..9]    generation     [10..11] payload_len = 6 * record_stride
 *   [12..107] payload: global record, then level records 1..5 (record_stride bytes each)
 *   [108..123] 0xFF (reserved, covered by the crc)
 *   [124..127] crc32 over [0..123], programmed LAST
 * A slot that is not entirely 0xFF but fails validation is "used": the log appends after the
 * highest used slot, so a half-written slot is skipped and never reused before the page erase.
 */

#define REC_MAGIC       0x46433341UL
#define REC_CRC_OFFSET  124U
#define REC_PAYLOAD_OFF 12U
#define GLOBAL_WORDS    ASSIST_V3_RECORD_WORDS
#define BLOCK_GLOBAL_OFF 16U
#define BLOCK_LEVEL_OFF  32U
#define BLOCK_CRC_OFF    112U
#define FRAME_COUNT     ((ASSIST_V3_BLOCK_LEN + 7U) / 8U)   /* 15 */
#define FRAME_MASK_ALL  ((uint16_t)((1UL << FRAME_COUNT) - 1UL))

/* Compiled defaults (CONFIG_PROTOCOL_V3 section 5): response, carry strength, carry extent, max torque. */
static const uint8_t k_defaults[ASSIST_V3_LEVELS][4] = {
	{ 40U, 20U, 30U, 100U },   /* 1 ECO    */
	{ 60U, 70U, 60U, 100U },   /* 2 TRAIL  */
	{ 80U, 50U, 50U, 100U },   /* 3 SPORT  */
	{ 90U, 80U, 80U, 100U },   /* 4 SPORT+ */
	{ 80U, 50U, 50U, 100U }    /* 5 BOOST  */
};

/* Absent-record engine: V3 in the candidate build (D-020); an image built without the V3 stage
 * has no V3 engine, so its only engine is G5300 (REVIEW-T #20, CONFIG_PROTOCOL_V3 section 4). */
#if ASSIST_V3
#define ENGINE_DEFAULT ASSIST_V3_ENGINE_V3
#define CAPS_SUPPORTED ASSIST_V3_CAPS
#else
#define ENGINE_DEFAULT ASSIST_V3_ENGINE_G5300
#define CAPS_SUPPORTED (ASSIST_V3_CAPS & ~ASSIST_V3_CAP_BEHAVIOR)
#endif

static assist_v3_values_t ram_values;
static assist_v3_values_t saved_values;
static const assist_v3_flash_t *flash_hal;
static uint16_t generation;
static bool persist_pending;
static bool persist_failed;
static uint8_t flash_state;
static bool engine_active_v3;

static struct {
	bool live;
	uint8_t owner;
	uint8_t declared;
	uint8_t frames;
	uint16_t mask;
	uint32_t last_ms;
	uint8_t staging[ASSIST_V3_BLOCK_LEN];
} xfer;

/* ---------------- pure helpers ---------------- */

static uint16_t rd16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t rd32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFFU); p[1] = (uint8_t)(v >> 8); }
static void wr32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFU); p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

uint16_t assist_v3_crc16(const uint8_t *data, uint16_t len)
{
	uint16_t crc = 0xFFFFU;
	uint16_t i;
	uint8_t bit;
	for (i = 0U; i < len; i++) {
		crc ^= (uint16_t)((uint16_t)data[i] << 8);
		for (bit = 0U; bit < 8U; bit++)
			crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
	}
	return crc;
}

uint32_t assist_v3_crc32(const uint8_t *data, uint32_t len)
{
	uint32_t crc = 0xFFFFFFFFUL;
	uint32_t i;
	uint8_t b;
	for (i = 0U; i < len; i++) {
		crc ^= data[i];
		for (b = 0U; b < 8U; b++)
			crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)(-(int32_t)(crc & 1U)));
	}
	return ~crc;
}

uint16_t assist_v3_generation_next(uint16_t g)
{
	g++;
	if (g == ASSIST_V3_UNSET) g = 0U;   /* 0xFFFF means "no check" on the wire */
	return g;
}

static void values_all_unset(assist_v3_values_t *v)
{
	memset(v, 0xFF, sizeof(*v));
}

/* Compiled default of one level parameter, 0xFFFF when the parameter has none. */
static uint16_t default_of(uint8_t li, uint8_t param)
{
	switch (param) {
	case ASSIST_V3_PARAM_RESPONSE:       return k_defaults[li][0];
	case ASSIST_V3_PARAM_CARRY_STRENGTH: return k_defaults[li][1];
	case ASSIST_V3_PARAM_CARRY_EXTENT:   return k_defaults[li][2];
	case ASSIST_V3_PARAM_MAX_TORQUE:     return k_defaults[li][3];
	default:                             return ASSIST_V3_UNSET;
	}
}

static bool param_in_mask(uint8_t param)
{
	return ((ASSIST_V3_PARAM_MASK >> param) & 1U) != 0U;
}

/* 0 = ok, 3 = out of range. 0xFFFF is always acceptable here. */
static bool param_range_ok(uint8_t param, uint16_t v)
{
	if (v == ASSIST_V3_UNSET) return true;
	switch (param) {
	case ASSIST_V3_PARAM_ENGINE:     return v <= ASSIST_V3_ENGINE_V3;
	case ASSIST_V3_PARAM_MAX_TORQUE: return v >= 10U && v <= 100U;
	default:                         return v <= ASSIST_V3_RESPONSE_MAX;
	}
}

/* ---------------- wire block ---------------- */

void assist_v3_block_encode(const assist_v3_values_t *values, uint32_t caps,
                            uint16_t gen, uint8_t out[ASSIST_V3_BLOCK_LEN])
{
	uint8_t i, w;
	memset(out, 0, ASSIST_V3_BLOCK_LEN);
	out[0] = 'B'; out[1] = 'V';
	out[2] = ASSIST_V3_SCHEMA_ID;
	out[3] = ASSIST_V3_SCHEMA_VERSION;
	wr16(&out[4], ASSIST_V3_BLOCK_LEN);
	out[6] = 0U;
	out[7] = ASSIST_V3_LEVELS;
	out[8] = ASSIST_V3_RECORD_STRIDE;
	out[9] = 0U;
	wr32(&out[10], caps);
	wr16(&out[14], gen);
	for (w = 0U; w < GLOBAL_WORDS; w++) wr16(&out[BLOCK_GLOBAL_OFF + 2U * w], values->global[w]);
	for (i = 0U; i < ASSIST_V3_LEVELS; i++)
		for (w = 0U; w < ASSIST_V3_RECORD_WORDS; w++)
			wr16(&out[BLOCK_LEVEL_OFF + (uint16_t)i * ASSIST_V3_RECORD_STRIDE + 2U * w], values->level[i][w]);
	wr16(&out[BLOCK_CRC_OFF], assist_v3_crc16(out, BLOCK_CRC_OFF));
}

static bool fail(uint8_t *reason, uint8_t *index, uint8_t r, uint8_t i)
{
	if (reason) *reason = r;
	if (index) *index = i;
	return false;
}

bool assist_v3_block_validate(const uint8_t *blk, uint16_t len, uint16_t current_generation,
                              assist_v3_values_t *out, uint8_t *reason, uint8_t *index)
{
	assist_v3_values_t v;
	uint16_t off;
	uint8_t i, w;
	uint16_t wgen;

	if (len >= 2U && (blk[0] != 'B' || blk[1] != 'V')) return fail(reason, index, ASSIST_V3_REASON_SCHEMA, 0U);
	if (len < ASSIST_V3_BLOCK_HEADER_LEN) return fail(reason, index, ASSIST_V3_REASON_LENGTH, 4U);
	if (blk[2] != ASSIST_V3_SCHEMA_ID) return fail(reason, index, ASSIST_V3_REASON_SCHEMA, 2U);
	if (blk[3] != ASSIST_V3_SCHEMA_VERSION) return fail(reason, index, ASSIST_V3_REASON_SCHEMA, 3U);
	if (rd16(&blk[4]) != ASSIST_V3_BLOCK_LEN || len != ASSIST_V3_BLOCK_LEN)
		return fail(reason, index, ASSIST_V3_REASON_LENGTH, 4U);
	if (blk[7] != ASSIST_V3_LEVELS) return fail(reason, index, ASSIST_V3_REASON_LENGTH, 7U);
	if (blk[8] != ASSIST_V3_RECORD_STRIDE) return fail(reason, index, ASSIST_V3_REASON_LENGTH, 8U);
	if (blk[6] != 0U) return fail(reason, index, ASSIST_V3_REASON_RANGE, 6U);

	/* reserved values */
	if (blk[9] != 0U) return fail(reason, index, ASSIST_V3_REASON_RESERVED, 9U);
	for (w = 1U; w < GLOBAL_WORDS; w++) {
		off = (uint16_t)(BLOCK_GLOBAL_OFF + 2U * w);
		if (rd16(&blk[off]) != ASSIST_V3_UNSET) return fail(reason, index, ASSIST_V3_REASON_RESERVED, (uint8_t)off);
	}
	for (i = 0U; i < ASSIST_V3_LEVELS; i++) {
		for (w = 6U; w < ASSIST_V3_RECORD_WORDS; w++) {
			off = (uint16_t)(BLOCK_LEVEL_OFF + (uint16_t)i * ASSIST_V3_RECORD_STRIDE + 2U * w);
			if (rd16(&blk[off]) != ASSIST_V3_UNSET) return fail(reason, index, ASSIST_V3_REASON_RESERVED, (uint8_t)off);
		}
	}

	if ((rd32(&blk[10]) & ~(uint32_t)CAPS_SUPPORTED) != 0UL)
		return fail(reason, index, ASSIST_V3_REASON_CAPABILITY, 10U);

	wgen = rd16(&blk[14]);
	if (wgen != ASSIST_V3_UNSET && wgen != current_generation)
		return fail(reason, index, ASSIST_V3_REASON_STALE, 14U);

	if (rd16(&blk[BLOCK_CRC_OFF]) != assist_v3_crc16(blk, BLOCK_CRC_OFF))
		return fail(reason, index, ASSIST_V3_REASON_CRC, (uint8_t)BLOCK_CRC_OFF);

	/* every value: reserved parameter (mask bit off) must be 0xFFFF, the rest in range */
	values_all_unset(&v);
	off = BLOCK_GLOBAL_OFF;
	v.global[0] = rd16(&blk[off]);
	if (v.global[0] != ASSIST_V3_UNSET) {
		if (!param_in_mask(ASSIST_V3_PARAM_ENGINE)) return fail(reason, index, ASSIST_V3_REASON_CAPABILITY, (uint8_t)off);
		if (!param_range_ok(ASSIST_V3_PARAM_ENGINE, v.global[0])) return fail(reason, index, ASSIST_V3_REASON_RANGE, (uint8_t)off);
#if !ASSIST_V3
		/* No V3 engine in this image: never accept and ignore (D-012) - capability reject. */
		if (v.global[0] == ASSIST_V3_ENGINE_V3) return fail(reason, index, ASSIST_V3_REASON_CAPABILITY, (uint8_t)off);
#endif
	}
	for (i = 0U; i < ASSIST_V3_LEVELS; i++) {
		for (w = 0U; w < 6U; w++) {
			off = (uint16_t)(BLOCK_LEVEL_OFF + (uint16_t)i * ASSIST_V3_RECORD_STRIDE + 2U * w);
			v.level[i][w] = rd16(&blk[off]);
			if (v.level[i][w] == ASSIST_V3_UNSET) continue;
			if (!param_in_mask(w)) return fail(reason, index, ASSIST_V3_REASON_CAPABILITY, (uint8_t)off);
			if (!param_range_ok(w, v.level[i][w])) return fail(reason, index, ASSIST_V3_REASON_RANGE, (uint8_t)off);
		}
	}
	if (out) *out = v;
	return true;
}

/* ---------------- views / consumers ---------------- */

static uint16_t resolve(uint16_t raw, uint16_t def)
{
	return (raw == ASSIST_V3_UNSET) ? def : raw;
}

static void build_view(uint8_t view, assist_v3_values_t *out)
{
	uint8_t i, p;
	values_all_unset(out);
	if (view == ASSIST_V3_VIEW_SAVED) {
		out->global[0] = saved_values.global[0];
	} else if (view == ASSIST_V3_VIEW_EFFECTIVE) {
		out->global[0] = resolve(ram_values.global[0], ENGINE_DEFAULT);
	} else {
		out->global[0] = ENGINE_DEFAULT;
	}
	for (i = 0U; i < ASSIST_V3_LEVELS; i++) {
		for (p = 0U; p < 6U; p++) {
			if (!param_in_mask(p)) continue;   /* reserved params read 0xFFFF in every view */
			if (view == ASSIST_V3_VIEW_SAVED)          out->level[i][p] = saved_values.level[i][p];
			else if (view == ASSIST_V3_VIEW_EFFECTIVE) out->level[i][p] = resolve(ram_values.level[i][p], default_of(i, p));
			else                                       out->level[i][p] = default_of(i, p);
		}
	}
}

uint16_t assist_v3_config_get(uint8_t level, uint8_t param)
{
	if (param == ASSIST_V3_PARAM_ENGINE) return resolve(ram_values.global[0], ENGINE_DEFAULT);
	if (param > ASSIST_V3_PARAM_ASSIST_RANGE || level < 1U || level > ASSIST_V3_LEVELS) return ASSIST_V3_UNSET;
	return resolve(ram_values.level[level - 1U][param], default_of((uint8_t)(level - 1U), param));
}

uint8_t assist_v3_config_response_pct(uint8_t level)
{
	uint16_t v = assist_v3_config_get(level, ASSIST_V3_PARAM_RESPONSE);
	return (v <= ASSIST_V3_RESPONSE_MAX) ? (uint8_t)v : 0U;
}

/* Milestone C release source (CONFIG_PROTOCOL_V3 status note): the v1 per-level "response" field
 * ("how quickly assist drops when you ease off"). The ONE place the pipeline asks for the release
 * of a level, so the v2 resolver (assist_v3_effective(level), ARCHITECTURE_V3 section 5) replaces
 * this body without re-plumbing the pipeline (REVIEW-T #5, #14). */
uint8_t assist_v3_effective_release_pct(uint8_t hmi_level)
{
	return assist_v3_config_response_pct(hmi_level);
}

bool assist_v3_config_engine_requested(void)
{
#if ASSIST_V3
	return assist_v3_config_get(1U, ASSIST_V3_PARAM_ENGINE) == ASSIST_V3_ENGINE_V3;
#else
	return false;   /* a record written by a V3 image cannot request an engine this image lacks */
#endif
}

bool assist_v3_config_engine_active(void) { return engine_active_v3; }
void assist_v3_config_set_engine_active(bool v3_active) { engine_active_v3 = v3_active; }

uint32_t assist_v3_config_caps(void) { return CAPS_SUPPORTED; }
uint16_t assist_v3_config_generation(void) { return generation; }
void assist_v3_config_ram_values(assist_v3_values_t *out) { *out = ram_values; }
void assist_v3_config_saved_values(assist_v3_values_t *out) { *out = saved_values; }
bool assist_v3_config_xfer_live(void) { return xfer.live; }
uint8_t assist_v3_config_flash_record_state(void) { return flash_state; }

uint8_t assist_v3_config_persist_state(void)
{
	if (persist_failed) return ASSIST_V3_PERSIST_FAILED;
	if (persist_pending) return ASSIST_V3_PERSIST_PENDING;
	if (memcmp(&ram_values, &saved_values, sizeof(ram_values)) != 0) return ASSIST_V3_PERSIST_DIRTY;
	return ASSIST_V3_PERSIST_CLEAN;
}

/* ---------------- flash log ---------------- */

typedef enum { SLOT_FREE, SLOT_INVALID, SLOT_NEWER, SLOT_VALID } slot_class_t;

static bool slot_is_free(const uint8_t *s)
{
	uint8_t i;
	for (i = 0U; i < ASSIST_V3_SLOT_SIZE; i++)
		if (s[i] != 0xFFU) return false;
	return true;
}

static slot_class_t classify_slot(const uint8_t *s, assist_v3_values_t *out, uint16_t *gen)
{
	uint8_t stride, r, w;
	uint16_t base;
	assist_v3_values_t v;

	if (slot_is_free(s)) return SLOT_FREE;
	if (rd32(s) != REC_MAGIC) return SLOT_INVALID;
	if (assist_v3_crc32(s, REC_CRC_OFFSET) != rd32(&s[REC_CRC_OFFSET])) return SLOT_INVALID;
	if (s[4] != ASSIST_V3_SCHEMA_ID || s[5] > ASSIST_V3_SCHEMA_VERSION) return SLOT_NEWER;
	if (s[5] == 0U) return SLOT_INVALID;
	stride = s[6];
	if (stride > ASSIST_V3_RECORD_STRIDE || s[7] != ASSIST_V3_LEVELS) return SLOT_NEWER;
	if (stride < 2U || (stride & 1U) != 0U) return SLOT_INVALID;
	if (rd16(&s[10]) != (uint16_t)(6U * stride)) return SLOT_INVALID;

	values_all_unset(&v);   /* migration: a shorter stride is back-filled with 0xFFFF */
	for (r = 0U; r < 6U; r++) {
		for (w = 0U; w < (uint8_t)(stride / 2U); w++) {
			uint16_t x;
			base = (uint16_t)(REC_PAYLOAD_OFF + (uint16_t)r * stride + 2U * w);
			x = rd16(&s[base]);
			if (r == 0U) {
				if (w == 0U) {
					if (x != ASSIST_V3_UNSET && !param_range_ok(ASSIST_V3_PARAM_ENGINE, x)) return SLOT_INVALID;
					v.global[0] = x;
				}
			} else if (w < 6U && param_in_mask(w)) {
				if (!param_range_ok(w, x)) return SLOT_INVALID;
				v.level[r - 1U][w] = x;
			}
			/* reserved parameters and reserved words are dropped (stay 0xFFFF) */
		}
	}
	if (out) *out = v;
	if (gen) *gen = rd16(&s[8]);
	return SLOT_VALID;
}

/* Newest valid slot wins. Returns the flash_record_state; fills *used_top (highest non-free
 * slot index or -1) and, for a valid record, the values and generation. */
static uint8_t scan_log(assist_v3_values_t *out, uint16_t *gen, int *used_top)
{
	int i, top = -1;
	bool corrupt = false;
	uint8_t st = ASSIST_V3_FLASH_ABSENT;
	for (i = 0; i < (int)ASSIST_V3_SLOT_COUNT; i++)
		if (!slot_is_free(&flash_hal->page[(uint32_t)i * ASSIST_V3_SLOT_SIZE])) top = i;
	if (used_top) *used_top = top;
	for (i = top; i >= 0; i--) {
		slot_class_t c = classify_slot(&flash_hal->page[(uint32_t)i * ASSIST_V3_SLOT_SIZE], out, gen);
		if (c == SLOT_VALID) return ASSIST_V3_FLASH_VALID;
		if (c == SLOT_NEWER) return ASSIST_V3_FLASH_NEWER;
		if (c == SLOT_INVALID) corrupt = true;
	}
	if (corrupt) st = ASSIST_V3_FLASH_CORRUPT;
	return st;
}

static void build_slot(uint8_t *s, const assist_v3_values_t *v, uint16_t gen)
{
	uint8_t r, w;
	memset(s, 0xFF, ASSIST_V3_SLOT_SIZE);
	wr32(s, REC_MAGIC);
	s[4] = ASSIST_V3_SCHEMA_ID;
	s[5] = ASSIST_V3_SCHEMA_VERSION;
	s[6] = ASSIST_V3_RECORD_STRIDE;
	s[7] = ASSIST_V3_LEVELS;
	wr16(&s[8], gen);
	wr16(&s[10], (uint16_t)(6U * ASSIST_V3_RECORD_STRIDE));
	for (w = 0U; w < GLOBAL_WORDS; w++) wr16(&s[REC_PAYLOAD_OFF + 2U * w], v->global[w]);
	for (r = 0U; r < ASSIST_V3_LEVELS; r++)
		for (w = 0U; w < ASSIST_V3_RECORD_WORDS; w++)
			wr16(&s[REC_PAYLOAD_OFF + (uint16_t)(r + 1U) * ASSIST_V3_RECORD_STRIDE + 2U * w], v->level[r][w]);
	wr32(&s[REC_CRC_OFFSET], assist_v3_crc32(s, REC_CRC_OFFSET));
}

static bool persist_now(void)
{
	uint8_t slot[ASSIST_V3_SLOT_SIZE];
	assist_v3_values_t cur;
	uint16_t g;
	int top;
	uint32_t next;
	uint32_t off;
	uint8_t st;

	if (flash_hal == 0 || flash_hal->page == 0 || flash_hal->program == 0 || flash_hal->erase == 0) return false;

	st = scan_log(&cur, &g, &top);
	if (st == ASSIST_V3_FLASH_VALID && memcmp(&cur, &ram_values, sizeof(cur)) == 0) {
		saved_values = ram_values;     /* identical content already in flash: no wear */
		flash_state = ASSIST_V3_FLASH_VALID;
		return true;
	}

	build_slot(slot, &ram_values, generation);
	next = (uint32_t)(top + 1);
	if (next >= ASSIST_V3_SLOT_COUNT) {
		if (!flash_hal->erase()) goto failed;   /* power loss from here to the first program = "absent" */
		next = 0U;
	}
	off = next * ASSIST_V3_SLOT_SIZE;
	if (!flash_hal->program(off, slot, REC_CRC_OFFSET)) goto failed;
	if (!flash_hal->program(off + REC_CRC_OFFSET, &slot[REC_CRC_OFFSET], 4U)) goto failed;   /* crc last */
	if (memcmp(&flash_hal->page[off], slot, ASSIST_V3_SLOT_SIZE) != 0) goto failed;
	saved_values = ram_values;
	flash_state = ASSIST_V3_FLASH_VALID;
	return true;

failed:
	flash_state = scan_log(0, 0, 0);
	return false;
}

/* ---------------- lifecycle ---------------- */

void assist_v3_config_init(const assist_v3_flash_t *flash)
{
	assist_v3_values_t loaded;
	uint16_t gen = 0U;

	flash_hal = flash;
	values_all_unset(&ram_values);
	values_all_unset(&saved_values);
	generation = 1U;
	persist_pending = false;
	persist_failed = false;
	memset(&xfer, 0, sizeof(xfer));
	flash_state = ASSIST_V3_FLASH_ABSENT;
	if (flash_hal != 0 && flash_hal->page != 0) {
		flash_state = scan_log(&loaded, &gen, 0);
		if (flash_state == ASSIST_V3_FLASH_VALID) {
			ram_values = loaded;
			saved_values = loaded;
			generation = assist_v3_generation_next(gen);
		}
	}
	/* TRUTHFUL readback (D-037, REVIEW-T #20): engine_active is what publishes Iq. At boot that is
	 * G5300 until the pipeline latch (ARCHITECTURE_V3 2.2) first finds every demand at 0 with no
	 * veto and reports the requested engine through assist_v3_config_set_engine_active(). */
	engine_active_v3 = false;
}

/* ---------------- CAN protocol ---------------- */

static assist_v3_reply_t reply_none(void)
{
	assist_v3_reply_t r = { ASSIST_V3_REPLY_NONE, 0U, 0U, 0U };
	return r;
}

static assist_v3_reply_t reply_make(uint8_t kind, uint8_t reason, uint8_t index, uint8_t target)
{
	assist_v3_reply_t r;
	r.kind = kind; r.reason = reason; r.index = index; r.target = target;
	return r;
}

bool assist_v3_config_owns_command(uint16_t command)
{
	return command >= ASSIST_V3_CMD_CAPS && command <= ASSIST_V3_CMD_CONTROL;
}

static void build_caps(uint8_t out[ASSIST_V3_CAPS_LEN])
{
	memset(out, 0, ASSIST_V3_CAPS_LEN);
	out[0] = 'B'; out[1] = 'V';
	out[2] = 1U;                               /* format */
	out[3] = 1U;                               /* protocol_version */
	out[4] = ASSIST_V3_SCHEMA_VERSION;         /* schema_min */
	out[5] = ASSIST_V3_SCHEMA_VERSION;         /* schema_max */
	out[6] = ASSIST_V3_LEVELS;
	out[7] = 1U;                               /* set_count */
	wr16(&out[8], ASSIST_V3_BLOCK_LEN);
	out[10] = ASSIST_V3_RECORD_STRIDE;
	out[11] = ASSIST_V3_PARAM_COUNT;
	wr32(&out[12], CAPS_SUPPORTED);
	wr16(&out[16], ASSIST_V3_PARAM_MASK);
	wr16(&out[18], generation);
	out[20] = assist_v3_config_persist_state();
	out[21] = flash_state;
	out[22] = engine_active_v3 ? 1U : 0U;
	out[23] = assist_v3_config_engine_requested() ? 1U : 0U;
	wr16(&out[24], assist_v3_crc16(out, 24U));
}

assist_v3_reply_t assist_v3_config_can_read(uint8_t source, uint16_t command, uint8_t dlen,
                                            const uint8_t *data, uint8_t *out, uint8_t *out_len)
{
	*out_len = 0U;
	if (source != ASSIST_V3_SOURCE_TOOL) return reply_none();
	if (command == ASSIST_V3_CMD_CAPS) {
		build_caps(out);
		*out_len = ASSIST_V3_CAPS_LEN;
		return reply_none();
	}
	if (command == ASSIST_V3_CMD_BLOCK) {
		assist_v3_values_t v;
		if (dlen < 2U) return reply_make(ASSIST_V3_REPLY_ERROR, ASSIST_V3_REASON_LENGTH, 0U, source);
		if (data[0] > ASSIST_V3_VIEW_DEFAULTS) return reply_make(ASSIST_V3_REPLY_ERROR, ASSIST_V3_REASON_RANGE, 0U, source);
		if (data[1] != 0U) return reply_make(ASSIST_V3_REPLY_ERROR, ASSIST_V3_REASON_RANGE, 1U, source);
		build_view(data[0], &v);
		assist_v3_block_encode(&v, CAPS_SUPPORTED, generation, out);
		*out_len = ASSIST_V3_BLOCK_LEN;
	}
	return reply_none();
}

static assist_v3_reply_t xfer_abort(uint8_t reason, uint8_t index)
{
	assist_v3_reply_t r = reply_make(ASSIST_V3_REPLY_ERROR, reason, index, xfer.owner);
	xfer.live = false;
	return r;
}

bool assist_v3_config_other_declaration(uint8_t source, assist_v3_reply_t *abort_reply)
{
	*abort_reply = reply_none();
	if (!xfer.live) return true;
	if (source != xfer.owner) return false;   /* foreign declaration while a transfer is live */
	*abort_reply = xfer_abort(ASSIST_V3_REASON_BUSY, ASSIST_V3_INDEX_NONE);   /* its owner moved on */
	return true;
}

assist_v3_reply_t assist_v3_config_can_declare(uint8_t source, uint8_t declared_len, uint32_t now_ms)
{
	if (source != ASSIST_V3_SOURCE_TOOL) return reply_none();
	if (xfer.live) return reply_make(ASSIST_V3_REPLY_ERROR, ASSIST_V3_REASON_BUSY, ASSIST_V3_INDEX_NONE, source);
	if (declared_len != ASSIST_V3_BLOCK_LEN)
		return reply_make(ASSIST_V3_REPLY_ERROR, ASSIST_V3_REASON_LENGTH, 4U, source);
	memset(&xfer, 0, sizeof(xfer));
	xfer.live = true;
	xfer.owner = source;
	xfer.declared = declared_len;
	xfer.last_ms = now_ms;
	return reply_make(ASSIST_V3_REPLY_DECL_ACK, 0U, 0U, source);
}

assist_v3_reply_t assist_v3_config_can_frame(uint8_t source, uint8_t frame_no, bool is_end,
                                             const uint8_t *data, uint8_t dlen, uint32_t now_ms)
{
	uint16_t off;
	uint8_t i;
	assist_v3_values_t v;
	uint8_t reason = 0U, index = 0U;

	if (!xfer.live || source != xfer.owner) return reply_none();
	xfer.last_ms = now_ms;

	if (frame_no >= FRAME_COUNT || dlen == 0U || dlen > 8U) return xfer_abort(ASSIST_V3_REASON_LENGTH, frame_no);
	off = (uint16_t)frame_no * 8U;
	if (off + dlen > xfer.declared) return xfer_abort(ASSIST_V3_REASON_LENGTH, frame_no);
	if (is_end ? (off + dlen != xfer.declared) : (dlen != 8U)) return xfer_abort(ASSIST_V3_REASON_LENGTH, frame_no);
	if ((xfer.mask >> frame_no) & 1U) return xfer_abort(ASSIST_V3_REASON_LENGTH, frame_no);   /* duplicate */

	memcpy(&xfer.staging[off], data, dlen);
	xfer.mask |= (uint16_t)(1U << frame_no);
	xfer.frames++;
	if (!is_end) return reply_none();

	if (xfer.mask != FRAME_MASK_ALL) {
		for (i = 0U; i < FRAME_COUNT; i++)
			if (!((xfer.mask >> i) & 1U)) break;
		return xfer_abort(ASSIST_V3_REASON_LENGTH, i);
	}
	if (!assist_v3_block_validate(xfer.staging, xfer.declared, generation, &v, &reason, &index))
		return xfer_abort(reason, index);

	ram_values = v;                              /* the one commit: main-loop context */
	generation = assist_v3_generation_next(generation);
	xfer.live = false;
	return reply_make(ASSIST_V3_REPLY_ACK, 0U, 0U, source);
}

assist_v3_reply_t assist_v3_config_can_control(uint8_t source, uint8_t dlen, const uint8_t *data)
{
	if (source != ASSIST_V3_SOURCE_TOOL) return reply_none();
	if (dlen < 1U) return reply_make(ASSIST_V3_REPLY_ERROR, ASSIST_V3_REASON_LENGTH, 0U, source);
	switch (data[0]) {
	case ASSIST_V3_OP_PERSIST:
		persist_pending = true;
		persist_failed = false;
		return reply_make(ASSIST_V3_REPLY_ACK, 0U, 0U, source);
	case ASSIST_V3_OP_REVERT:
		if (xfer.live) return reply_make(ASSIST_V3_REPLY_ERROR, ASSIST_V3_REASON_BUSY, ASSIST_V3_INDEX_NONE, source);
		ram_values = saved_values;
		generation = assist_v3_generation_next(generation);
		return reply_make(ASSIST_V3_REPLY_ACK, 0U, 0U, source);
	case ASSIST_V3_OP_DEFAULTS:
		if (xfer.live) return reply_make(ASSIST_V3_REPLY_ERROR, ASSIST_V3_REASON_BUSY, ASSIST_V3_INDEX_NONE, source);
		values_all_unset(&ram_values);
		generation = assist_v3_generation_next(generation);
		return reply_make(ASSIST_V3_REPLY_ACK, 0U, 0U, source);
	default:
		return reply_make(ASSIST_V3_REPLY_ERROR, ASSIST_V3_REASON_RANGE, 0U, source);
	}
}

assist_v3_reply_t assist_v3_config_service(uint32_t now_ms, bool standstill)
{
	if (xfer.live && (uint32_t)(now_ms - xfer.last_ms) > ASSIST_V3_XFER_TIMEOUT_MS)
		return xfer_abort(ASSIST_V3_REASON_LENGTH, xfer.frames);   /* incomplete: index = frames received */
	if (persist_pending && standstill && !xfer.live) {
		persist_pending = false;
		persist_failed = !persist_now();
	}
	return reply_none();
}
