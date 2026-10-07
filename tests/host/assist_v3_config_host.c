/*
 * Assist V3 configuration owner (src/assist_v3_config.c) - contract tests for
 * docs/assist-v3/CONFIG_PROTOCOL_V3.md. Real module, host flash stub with fault injection.
 *
 *   G1  shared byte vectors: CAPS and the BEHAVIOR block, bytes produced by an independent encoder
 *   G2  a valid write round-trips through all three views
 *   G3  every reject reason, each with a no-mutation assertion (RAM, saved, generation, flash)
 *   G4  reserved parameters: read 0xFFFF in every view, any other written value is refused (reason 4)
 *   G5  generation: skips 0xFFFF on wrap, stale base refused, 0xFFFF = no check
 *   G6  transfer: foreign owner, redeclaration, timeout, oversize / bad-index / duplicate / missing frames,
 *       and exactly one result frame in every case
 *   G7  persist -> restart -> readback; deferred to standstill; identical content costs no slot
 *   G8  corrupt record, newer-schema record left untouched, shorter-stride migration
 *   G9  log full -> erase -> continue; erase-window power loss -> defaults, never a corrupt config
 *   G10 revert / defaults / engine request vs active / consumer getters / READ guards
 *   G11 wiring guards on src/CAN_Display.c: P0/P1/P2, bank and tuning code paths are textually untouched
 *
 * The shared vectors were produced by a separate Python encoder, not by this module.
 */

#include "check.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "assist_v3_config.h"

#ifndef CAN_DISPLAY_C_PATH
#error "CAN_DISPLAY_C_PATH must be defined by the build script"
#endif

/* ---------------- shared vectors (independent encoder) ---------------- */

static const uint8_t golden_caps_boot[26] = {
	0x42, 0x56, 0x01, 0x01, 0x01, 0x01, 0x05, 0x01, 0x72, 0x00, 0x10, 0x07,
	0x3F, 0x00, 0x00, 0x00, 0x01, 0x80, 0x01, 0x00, 0x00, 0x01, 0x01, 0x01,
	0xBB, 0x3E,
};
static const uint8_t golden_block_effective_boot[114] = {
	0x42, 0x56, 0x01, 0x01, 0x72, 0x00, 0x00, 0x05, 0x10, 0x00, 0x3F, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x28, 0x00, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0x3C, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0x50, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x5A, 0x00, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0x50, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xC5, 0xE9,
};
static const uint8_t golden_block_write_a[114] = {
	0x42, 0x56, 0x01, 0x01, 0x72, 0x00, 0x00, 0x05, 0x10, 0x00, 0x3F, 0x00,
	0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0A, 0x00, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0x14, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0x1E, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x28, 0x00, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0x32, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xE5, 0xDF,
};

/* ---------------- flash stub with fault injection ---------------- */

static uint8_t flash_mem[ASSIST_V3_FLASH_PAGE_SIZE];
static unsigned erase_count;
static unsigned program_calls;
static long fi_word_budget = -1;     /* words that may still be programmed; -1 = unlimited */
static bool fi_erase_half;           /* erase wipes only the first half and reports failure */

static bool stub_erase(void)
{
	erase_count++;
	if (fi_erase_half) {
		memset(flash_mem, 0xFF, ASSIST_V3_FLASH_PAGE_SIZE / 2U);
		return false;
	}
	memset(flash_mem, 0xFF, sizeof(flash_mem));
	return true;
}

static bool stub_program(uint32_t offset, const uint8_t *data, uint32_t len)
{
	uint32_t i;
	program_calls++;
	if ((offset & 3U) != 0U || (len & 3U) != 0U || offset + len > sizeof(flash_mem)) return false;
	for (i = 0U; i < len; i += 4U) {
		uint32_t b;
		if (fi_word_budget == 0) return false;     /* power lost: this and the later words never land */
		if (fi_word_budget > 0) fi_word_budget--;
		for (b = 0U; b < 4U; b++) flash_mem[offset + i + b] &= data[i + b];   /* flash only clears bits */
	}
	return true;
}

static const assist_v3_flash_t hal = { flash_mem, stub_erase, stub_program };

static void flash_blank(void)
{
	memset(flash_mem, 0xFF, sizeof(flash_mem));
	erase_count = 0U;
	program_calls = 0U;
	fi_word_budget = -1;
	fi_erase_half = false;
}

static void restart(void) { assist_v3_config_init(&hal); }

/* ---------------- helpers ---------------- */

#define NOW0 1000U

static uint16_t rd16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFFU); p[1] = (uint8_t)(v >> 8); }
static void wr32(uint8_t *p, uint32_t v) { wr16(p, (uint16_t)(v & 0xFFFFU)); wr16(p + 2, (uint16_t)(v >> 16)); }

static void fix_crc(uint8_t *blk) { wr16(&blk[112], assist_v3_crc16(blk, 112U)); }

typedef struct {
	assist_v3_reply_t result;   /* the (single) result frame */
	unsigned results;           /* number of result frames (ACK or ERROR, not the declaration ACK) */
	bool decl_acked;
} sent_t;

static void note(sent_t *s, assist_v3_reply_t r)
{
	if (r.kind == ASSIST_V3_REPLY_ACK || r.kind == ASSIST_V3_REPLY_ERROR) { s->result = r; s->results++; }
	else if (r.kind == ASSIST_V3_REPLY_DECL_ACK) s->decl_acked = true;
}

/* One complete WRITE 0x6036 the way the tool sends it: declaration, then frames 0..N. */
static sent_t send_block_from(uint8_t src, const uint8_t *blk, uint8_t declared, uint8_t actual_len, uint32_t now)
{
	sent_t s;
	uint8_t f, nframes;
	assist_v3_reply_t ab, r;
	memset(&s, 0, sizeof(s));
	(void)assist_v3_config_other_declaration(src, &ab);
	note(&s, ab);
	r = assist_v3_config_can_declare(src, declared, now);
	note(&s, r);
	if (!s.decl_acked) return s;
	nframes = (uint8_t)((actual_len + 7U) / 8U);
	for (f = 0U; f < nframes; f++) {
		uint8_t off = (uint8_t)(f * 8U);
		uint8_t n = (uint8_t)(((int)actual_len - (int)off) > 8 ? 8 : ((int)actual_len - (int)off));
		note(&s, assist_v3_config_can_frame(src, f, f == nframes - 1U, &blk[off], n, now));
	}
	return s;
}

static sent_t send_block(const uint8_t *blk) { return send_block_from(5U, blk, 114U, 114U, NOW0); }

static assist_v3_reply_t control(uint8_t src, uint8_t op)
{
	return assist_v3_config_can_control(src, 1U, &op);
}

typedef struct {
	assist_v3_values_t ram, saved;
	uint16_t gen;
	uint8_t persist, flashst;
	uint8_t page[ASSIST_V3_FLASH_PAGE_SIZE];
	bool live;
} snap_t;

static void snap_take(snap_t *s)
{
	assist_v3_config_ram_values(&s->ram);
	assist_v3_config_saved_values(&s->saved);
	s->gen = assist_v3_config_generation();
	s->persist = assist_v3_config_persist_state();
	s->flashst = assist_v3_config_flash_record_state();
	memcpy(s->page, flash_mem, sizeof(flash_mem));
	s->live = assist_v3_config_xfer_live();
}

static bool snap_same(const snap_t *a, const snap_t *b)
{
	return memcmp(&a->ram, &b->ram, sizeof(a->ram)) == 0 && memcmp(&a->saved, &b->saved, sizeof(a->saved)) == 0 &&
	       a->gen == b->gen && a->persist == b->persist && a->flashst == b->flashst &&
	       memcmp(a->page, b->page, sizeof(a->page)) == 0 && a->live == b->live;
}

static void read_view(uint8_t view, uint8_t out[ASSIST_V3_BLOCK_LEN], bool *ok)
{
	uint8_t req[2] = { view, 0U };
	uint8_t len = 0U;
	assist_v3_reply_t r = assist_v3_config_can_read(5U, ASSIST_V3_CMD_BLOCK, 2U, req, out, &len);
	*ok = (r.kind == ASSIST_V3_REPLY_NONE && len == ASSIST_V3_BLOCK_LEN);
}

static uint16_t view_level(const uint8_t *blk, uint8_t level, uint8_t param)
{
	return rd16(&blk[32U + (uint16_t)(level - 1U) * 16U + 2U * param]);
}

/* block with response per level and engine, generation = base */
static void make_block(uint8_t out[114], uint16_t engine, const uint16_t resp[5], uint16_t gen)
{
	assist_v3_values_t v;
	uint8_t i;
	memset(&v, 0xFF, sizeof(v));
	v.global[0] = engine;
	for (i = 0U; i < 5U; i++) v.level[i][0] = resp[i];
	assist_v3_block_encode(&v, ASSIST_V3_CAPS, gen, out);
}

static const uint16_t RESP_A[5] = { 10, 20, 30, 40, 50 };

static void boot_blank(void) { flash_blank(); restart(); }

static bool persist_service(void)
{
	(void)assist_v3_config_service(NOW0, true);
	return assist_v3_config_persist_state() != ASSIST_V3_PERSIST_FAILED;
}

/* ---------------- G1 ---------------- */

static void g1_vectors(void)
{
	uint8_t caps[26], blk[114], len = 0U;
	bool ok;
	assist_v3_reply_t r;
	boot_blank();
	r = assist_v3_config_can_read(5U, ASSIST_V3_CMD_CAPS, 0U, caps, caps, &len);
	CHECK(r.kind == ASSIST_V3_REPLY_NONE && len == 26U, "G1 caps read length 26");
	CHECK(memcmp(caps, golden_caps_boot, 26U) == 0, "G1 caps bytes == shared vector");
	read_view(ASSIST_V3_VIEW_EFFECTIVE, blk, &ok);
	CHECK(ok && memcmp(blk, golden_block_effective_boot, 114U) == 0, "G1 effective block bytes == shared vector");
	CHECK(assist_v3_crc16((const uint8_t *)"123456789", 9U) == 0x29B1U, "G1 crc16 check value");
	CHECK(assist_v3_crc32((const uint8_t *)"123456789", 9U) == 0xCBF43926UL, "G1 crc32 check value");
	{
		uint8_t enc[114];
		make_block(enc, 0U, RESP_A, 0xFFFFU);
		CHECK(memcmp(enc, golden_block_write_a, 114U) == 0, "G1 encoder == independent encoder (write vector)");
	}
	{
		assist_v3_values_t v;
		uint8_t reason = 99U, index = 99U;
		CHECK(assist_v3_block_validate(golden_block_write_a, 114U, 1U, &v, &reason, &index), "G1 decode accepts the write vector");
		CHECK(v.global[0] == 0U && v.level[0][0] == 10U && v.level[4][0] == 50U && v.level[2][3] == 0xFFFFU, "G1 decoded values");
	}
	/* source other than the tool gets nothing */
	r = assist_v3_config_can_read(3U, ASSIST_V3_CMD_CAPS, 0U, caps, caps, &len);
	CHECK(r.kind == ASSIST_V3_REPLY_NONE && len == 0U, "G1 READ from source 3 is silent");
}

/* ---------------- G2 ---------------- */

static void g2_roundtrip(void)
{
	uint8_t blk[114], out[114];
	bool ok;
	sent_t s;
	snap_t before;
	boot_blank();
	snap_take(&before);
	s = send_block(golden_block_write_a);
	CHECK(s.decl_acked && s.results == 1U && s.result.kind == ASSIST_V3_REPLY_ACK && s.result.target == 5U, "G2 valid write: declaration ACK + exactly one NORMAL_ACK");
	CHECK(assist_v3_config_generation() == assist_v3_generation_next(before.gen), "G2 generation advanced once");
	read_view(ASSIST_V3_VIEW_EFFECTIVE, blk, &ok);
	CHECK(ok && view_level(blk, 1, 0) == 10U && view_level(blk, 5, 0) == 50U && rd16(&blk[16]) == 0U, "G2 effective view shows the written values");
	read_view(ASSIST_V3_VIEW_SAVED, out, &ok);
	CHECK(ok && view_level(out, 1, 0) == 0xFFFFU && rd16(&out[16]) == 0xFFFFU, "G2 saved view still raw 0xFFFF (nothing persisted)");
	read_view(ASSIST_V3_VIEW_DEFAULTS, out, &ok);
	CHECK(ok && view_level(out, 1, 0) == 40U && view_level(out, 4, 0) == 90U && rd16(&out[16]) == 1U, "G2 defaults view = compiled table, engine 1");
	CHECK(assist_v3_config_persist_state() == ASSIST_V3_PERSIST_DIRTY, "G2 persist_state dirty after write");
	/* a write of the explicit default value, and of 0xFFFF, both resolve to it */
	{
		const uint16_t r2[5] = { 0xFFFF, 60, 0xFFFF, 90, 80 };
		make_block(blk, 0xFFFFU, r2, 0xFFFFU);
		s = send_block(blk);
		CHECK(s.result.kind == ASSIST_V3_REPLY_ACK, "G2 0xFFFF accepted as 'default'");
		CHECK(assist_v3_config_response_pct(1) == 40U && assist_v3_config_response_pct(3) == 80U && assist_v3_config_engine_requested(), "G2 0xFFFF resolves to the compiled default");
	}
}

/* ---------------- G3 ---------------- */

static void expect_reject(const char *what, const uint8_t *blk, uint8_t declared, uint8_t actual, uint8_t reason, uint8_t index)
{
	snap_t a, b;
	sent_t s;
	char msg[160];
	snap_take(&a);
	s = send_block_from(5U, blk, declared, actual, NOW0);
	snap_take(&b);
	snprintf(msg, sizeof(msg), "G3 %s: exactly one ERROR_ACK", what);
	CHECK(s.results == 1U && s.result.kind == ASSIST_V3_REPLY_ERROR && s.result.target == 5U, msg);
	snprintf(msg, sizeof(msg), "G3 %s: reason %u index %u (got %u/%u)", what, reason, index, s.result.reason, s.result.index);
	CHECK(s.result.reason == reason && s.result.index == index, msg);
	snprintf(msg, sizeof(msg), "G3 %s: zero mutation (RAM, saved, generation, flash, transfer)", what);
	CHECK(snap_same(&a, &b), msg);
}

static void g3_rejects(void)
{
	uint8_t b[114];
	const uint16_t cur_gen_marker = 0U;
	(void)cur_gen_marker;
	boot_blank();
	/* leave a non-trivial state first so "no mutation" is meaningful */
	{
		uint8_t seed[114];
		make_block(seed, 0U, RESP_A, 0xFFFFU);
		CHECK(send_block(seed).result.kind == ASSIST_V3_REPLY_ACK, "G3 seed write");
		(void)control(5U, ASSIST_V3_OP_PERSIST);
		CHECK(persist_service(), "G3 seed persist");
	}

	memcpy(b, golden_block_write_a, 114U); b[0] = 'X'; fix_crc(b);
	expect_reject("bad magic", b, 114U, 114U, ASSIST_V3_REASON_SCHEMA, 0U);
	memcpy(b, golden_block_write_a, 114U); b[2] = 2U; fix_crc(b);
	expect_reject("unknown schema_id", b, 114U, 114U, ASSIST_V3_REASON_SCHEMA, 2U);
	memcpy(b, golden_block_write_a, 114U); b[3] = 2U; fix_crc(b);
	expect_reject("schema_version 2 (exact match only)", b, 114U, 114U, ASSIST_V3_REASON_SCHEMA, 3U);
	memcpy(b, golden_block_write_a, 114U); wr16(&b[4], 113U); fix_crc(b);
	expect_reject("header total_len != 114", b, 114U, 114U, ASSIST_V3_REASON_LENGTH, 4U);
	expect_reject("declared length 255", golden_block_write_a, 255U, 114U, ASSIST_V3_REASON_LENGTH, 4U);
	expect_reject("declared length 113", golden_block_write_a, 113U, 114U, ASSIST_V3_REASON_LENGTH, 4U);
	memcpy(b, golden_block_write_a, 114U); b[7] = 4U; fix_crc(b);
	expect_reject("level_count 4", b, 114U, 114U, ASSIST_V3_REASON_LENGTH, 7U);
	memcpy(b, golden_block_write_a, 114U); b[8] = 18U; fix_crc(b);
	expect_reject("record_stride 18", b, 114U, 114U, ASSIST_V3_REASON_LENGTH, 8U);
	memcpy(b, golden_block_write_a, 114U); b[6] = 1U; fix_crc(b);
	expect_reject("set_index 1", b, 114U, 114U, ASSIST_V3_REASON_RANGE, 6U);
	memcpy(b, golden_block_write_a, 114U); b[9] = 1U; fix_crc(b);
	expect_reject("flags != 0", b, 114U, 114U, ASSIST_V3_REASON_RESERVED, 9U);
	memcpy(b, golden_block_write_a, 114U); wr16(&b[20], 5U); fix_crc(b);
	expect_reject("global reserved word != 0xFFFF", b, 114U, 114U, ASSIST_V3_REASON_RESERVED, 20U);
	memcpy(b, golden_block_write_a, 114U); wr16(&b[32U + 16U + 14U], 0U); fix_crc(b);
	expect_reject("level record reserved word != 0xFFFF", b, 114U, 114U, ASSIST_V3_REASON_RESERVED, 62U);
	memcpy(b, golden_block_write_a, 114U); wr32(&b[10], 0x00000040UL); fix_crc(b);
	expect_reject("required cap bit 6 not offered", b, 114U, 114U, ASSIST_V3_REASON_CAPABILITY, 10U);
	memcpy(b, golden_block_write_a, 114U); wr32(&b[10], 0x80000000UL); fix_crc(b);
	expect_reject("required cap bit 31 not offered", b, 114U, 114U, ASSIST_V3_REASON_CAPABILITY, 10U);
	memcpy(b, golden_block_write_a, 114U); wr16(&b[14], (uint16_t)(assist_v3_config_generation() + 1U)); fix_crc(b);
	expect_reject("stale generation", b, 114U, 114U, ASSIST_V3_REASON_STALE, 14U);
	memcpy(b, golden_block_write_a, 114U); b[50] ^= 0x01U;   /* corrupt a value, keep the old CRC */
	expect_reject("crc mismatch", b, 114U, 114U, ASSIST_V3_REASON_CRC, 112U);
	memcpy(b, golden_block_write_a, 114U); wr16(&b[32U], 101U); fix_crc(b);
	expect_reject("response 101 (range)", b, 114U, 114U, ASSIST_V3_REASON_RANGE, 32U);
	memcpy(b, golden_block_write_a, 114U); wr16(&b[32U + 3U * 16U], 0x7FFFU); fix_crc(b);
	expect_reject("response 0x7FFF at level 4", b, 114U, 114U, ASSIST_V3_REASON_RANGE, 80U);
	memcpy(b, golden_block_write_a, 114U); wr16(&b[16], 2U); fix_crc(b);
	expect_reject("engine 2 (range)", b, 114U, 114U, ASSIST_V3_REASON_RANGE, 16U);
	memcpy(b, golden_block_write_a, 114U); wr16(&b[32U + 2U * 16U + 4U], 50U); fix_crc(b);
	expect_reject("reserved param carry_strength=50", b, 114U, 114U, ASSIST_V3_REASON_CAPABILITY, 68U);
	memcpy(b, golden_block_write_a, 114U); wr16(&b[32U + 5U * 2U], 0U); fix_crc(b);   /* level 1, word 5 assist_range */
	expect_reject("reserved param assist_range=0", b, 114U, 114U, ASSIST_V3_REASON_CAPABILITY, 42U);
	memcpy(b, golden_block_write_a, 114U); memset(&b[0], 0, 2U);
	expect_reject("all-zero magic", b, 114U, 114U, ASSIST_V3_REASON_SCHEMA, 0U);

	/* pure validator agrees (state-free) */
	{
		uint8_t reason = 0U, index = 0U;
		memcpy(b, golden_block_write_a, 114U);
		CHECK(!assist_v3_block_validate(b, 113U, 0U, 0, &reason, &index) && reason == ASSIST_V3_REASON_LENGTH, "G3 validator: wrong buffer length");
		CHECK(!assist_v3_block_validate(b, 1U, 0U, 0, &reason, &index), "G3 validator: 1 byte buffer");
	}
}

/* ---------------- G4 ---------------- */

static void g4_reserved(void)
{
	uint8_t out[114], b[114];
	uint8_t v, lvl, p;
	bool ok;
	assist_v3_values_t vals;
	boot_blank();
	for (v = 0U; v <= 2U; v++) {
		read_view(v, out, &ok);
		CHECK(ok, "G4 view readable");
		for (lvl = 1U; lvl <= 5U; lvl++)
			for (p = 1U; p <= 5U; p++) {
				if (p == 0U) continue;
				if (view_level(out, lvl, p) != 0xFFFFU) { CHECK(0, "G4 reserved param reads 0xFFFF in every view"); lvl = 6U; v = 3U; break; }
			}
		if (v > 2U) break;
	}
	CHECK(ASSIST_V3_PARAM_MASK == 0x8001U, "G4 param_mask is bits 0 and 15 in Milestone C");
	/* every reserved parameter, every level, refused with reason 4 and index = the value's offset */
	for (lvl = 1U; lvl <= 5U; lvl++) {
		for (p = 1U; p <= 5U; p++) {
			snap_t a, c;
			sent_t s;
			memcpy(b, golden_block_write_a, 114U);
			wr16(&b[32U + (uint16_t)(lvl - 1U) * 16U + 2U * p], 50U);
			fix_crc(b);
			snap_take(&a);
			s = send_block(b);
			snap_take(&c);
			CHECK(s.results == 1U && s.result.kind == ASSIST_V3_REPLY_ERROR && s.result.reason == ASSIST_V3_REASON_CAPABILITY &&
			      s.result.index == (uint8_t)(32U + (lvl - 1U) * 16U + 2U * p), "G4 reserved param write -> reason 4 at its offset");
			CHECK(snap_same(&a, &c), "G4 reserved param write mutates nothing");
		}
	}
	/* the getter still resolves a reserved parameter to its compiled default (consumer side) */
	CHECK(assist_v3_config_get(2U, ASSIST_V3_PARAM_CARRY_STRENGTH) == 70U && assist_v3_config_get(1U, ASSIST_V3_PARAM_MAX_TORQUE) == 100U, "G4 getter resolves defaults");
	CHECK(assist_v3_config_get(1U, ASSIST_V3_PARAM_START_RESPONSE) == 0xFFFFU && assist_v3_config_get(0U, 0U) == 0xFFFFU && assist_v3_config_get(6U, 0U) == 0xFFFFU, "G4 no default / invalid level -> 0xFFFF");
	/* ram unchanged => still all unset */
	assist_v3_config_ram_values(&vals);
	CHECK(vals.global[0] == 0xFFFFU && vals.level[0][2] == 0xFFFFU, "G4 RAM store untouched");
}

/* ---------------- G5 ---------------- */

static void g5_generation(void)
{
	uint8_t b[114];
	sent_t s;
	uint32_t i;
	bool bad = false;
	uint16_t g0, g1;
	assist_v3_reply_t r;
	CHECK(assist_v3_generation_next(0U) == 1U && assist_v3_generation_next(0xFFFEU) == 0U && assist_v3_generation_next(0xFFFFU) == 0U, "G5 next(): 0xFFFE wraps to 0, never 0xFFFF");
	boot_blank();
	g0 = assist_v3_config_generation();
	/* walk the whole 16-bit space through the control op and check 0xFFFF never appears */
	for (i = 0U; i < 70000U; i++) {
		r = control(5U, ASSIST_V3_OP_REVERT);
		if (r.kind != ASSIST_V3_REPLY_ACK || assist_v3_config_generation() == 0xFFFFU) bad = true;
	}
	CHECK(!bad, "G5 generation wraps without ever being 0xFFFF");
	CHECK(assist_v3_config_generation() == (uint16_t)((70000U + g0) % 65535U), "G5 sequence length is 65535 (0xFFFF skipped)");
	/* stale base */
	g1 = assist_v3_config_generation();
	make_block(b, 0U, RESP_A, g1);
	s = send_block(b);
	CHECK(s.result.kind == ASSIST_V3_REPLY_ACK, "G5 current base accepted");
	make_block(b, 0U, RESP_A, g1);   /* same base again: now stale */
	{
		snap_t a, c;
		snap_take(&a);
		s = send_block(b);
		snap_take(&c);
		CHECK(s.result.kind == ASSIST_V3_REPLY_ERROR && s.result.reason == ASSIST_V3_REASON_STALE && s.result.index == 14U, "G5 stale base refused (reason 6)");
		CHECK(snap_same(&a, &c), "G5 stale write mutates nothing");
	}
	make_block(b, 1U, RESP_A, 0xFFFFU);
	CHECK(send_block(b).result.kind == ASSIST_V3_REPLY_ACK, "G5 0xFFFF base = no check");
}

/* ---------------- G6 ---------------- */

static void g6_transfer(void)
{
	uint8_t frame[8] = { 0 };
	assist_v3_reply_t r;
	sent_t s;
	snap_t a, c;
	unsigned f;

	/* foreign source while a transfer is live: frames ignored, declaration refused, no effect */
	boot_blank();
	CHECK(assist_v3_config_can_declare(5U, 114U, NOW0).kind == ASSIST_V3_REPLY_DECL_ACK, "G6 declaration accepted");
	CHECK(assist_v3_config_xfer_live(), "G6 transfer live");
	snap_take(&a);
	r = assist_v3_config_can_frame(3U, 0U, false, frame, 8U, NOW0);
	CHECK(r.kind == ASSIST_V3_REPLY_NONE, "G6 frame from a foreign source is ignored");
	{
		assist_v3_reply_t ab;
		CHECK(!assist_v3_config_other_declaration(3U, &ab) && ab.kind == ASSIST_V3_REPLY_NONE, "G6 foreign declaration of another command is refused, nothing dropped");
	}
	r = assist_v3_config_can_declare(3U, 114U, NOW0);
	CHECK(r.kind == ASSIST_V3_REPLY_NONE, "G6 foreign source declaration of 0x6036 is silent (source 5 only)");
	snap_take(&c);
	CHECK(snap_same(&a, &c), "G6 foreign activity does not disturb the live transfer");
	r = assist_v3_config_can_declare(5U, 114U, NOW0);
	CHECK(r.kind == ASSIST_V3_REPLY_ERROR && r.reason == ASSIST_V3_REASON_BUSY, "G6 declaration while live (not dropped first) -> busy");
	/* the live transfer still completes */
	for (f = 0U; f < 15U; f++) {
		uint8_t off = (uint8_t)(f * 8U);
		uint8_t n = (uint8_t)((114 - (int)off) > 8 ? 8 : (114 - (int)off));
		r = assist_v3_config_can_frame(5U, (uint8_t)f, f == 14U, &golden_block_write_a[off], n, NOW0);
	}
	CHECK(r.kind == ASSIST_V3_REPLY_ACK && !assist_v3_config_xfer_live(), "G6 the owner's transfer completed after the foreign noise");

	/* the owner declares again, or declares another command: old transfer ends with ONE error */
	boot_blank();
	(void)assist_v3_config_can_declare(5U, 114U, NOW0);
	(void)assist_v3_config_can_frame(5U, 0U, false, golden_block_write_a, 8U, NOW0);
	s = send_block(golden_block_write_a);
	CHECK(s.results == 2U, "G6 redeclaration: error for the dropped transfer + result of the new one");
	boot_blank();
	(void)assist_v3_config_can_declare(5U, 114U, NOW0);
	{
		assist_v3_reply_t ab;
		CHECK(assist_v3_config_other_declaration(5U, &ab) && ab.kind == ASSIST_V3_REPLY_ERROR && ab.reason == ASSIST_V3_REASON_BUSY && ab.target == 5U, "G6 owner declares P0/P1/bank: V3 transfer dropped with one busy error");
		CHECK(!assist_v3_config_xfer_live(), "G6 transfer gone");
		r = assist_v3_config_can_frame(5U, 1U, false, frame, 8U, NOW0);
		CHECK(r.kind == ASSIST_V3_REPLY_NONE, "G6 late frames of the dropped transfer are ignored");
	}

	/* timeout */
	boot_blank();
	(void)assist_v3_config_can_declare(5U, 114U, 5000U);
	CHECK(assist_v3_config_service(5000U + 1000U, false).kind == ASSIST_V3_REPLY_NONE, "G6 not timed out at exactly 1000 ms idle");
	(void)assist_v3_config_can_frame(5U, 0U, false, golden_block_write_a, 8U, 5900U);
	CHECK(assist_v3_config_service(6800U, false).kind == ASSIST_V3_REPLY_NONE, "G6 any frame restarts the idle timer");
	snap_take(&a);
	r = assist_v3_config_service(6901U, false);
	CHECK(r.kind == ASSIST_V3_REPLY_ERROR && r.reason == ASSIST_V3_REASON_LENGTH && r.index == 1U && r.target == 5U, "G6 timeout: one ERROR_ACK [length, frames received]");
	snap_take(&c);
	CHECK(!c.live && memcmp(&a.ram, &c.ram, sizeof(a.ram)) == 0 && a.gen == c.gen, "G6 timeout discards staging, no mutation");
	CHECK(assist_v3_config_service(9999U, false).kind == ASSIST_V3_REPLY_NONE, "G6 timeout reported once");
	r = assist_v3_config_can_frame(5U, 14U, true, &golden_block_write_a[112], 2U, 7000U);
	CHECK(r.kind == ASSIST_V3_REPLY_NONE, "G6 END after timeout -> no second result");
	/* timeout across the 32-bit ms wrap */
	boot_blank();
	(void)assist_v3_config_can_declare(5U, 114U, 0xFFFFFF00UL);
	CHECK(assist_v3_config_service(0xFFFFFF00UL + 900U, false).kind == ASSIST_V3_REPLY_NONE, "G6 wrap: 900 ms idle ok");
	CHECK(assist_v3_config_service(0xFFFFFF00UL + 1001U, false).kind == ASSIST_V3_REPLY_ERROR, "G6 wrap: timeout across the 2^32 ms wrap");

	/* bad frames: each aborts with exactly one error and no mutation, later frames are silent */
	{
		const struct { const char *what; uint8_t frame_no; bool end; uint8_t dlen; } bad[] = {
			{ "dlen 9", 0U, false, 9U },
			{ "dlen 0", 0U, false, 0U },
			{ "frame index 15 (past the block)", 15U, false, 8U },
			{ "frame index 255", 255U, false, 8U },
			{ "non-END frame shorter than 8", 3U, false, 5U },
			{ "frame 14 not END (would run past 114)", 14U, false, 8U },
			{ "END at frame 3", 3U, true, 8U },
			{ "END with wrong tail length", 14U, true, 3U },
		};
		unsigned i;
		for (i = 0U; i < sizeof(bad) / sizeof(bad[0]); i++) {
			char msg[120];
			boot_blank();
			(void)assist_v3_config_can_declare(5U, 114U, NOW0);
			snap_take(&a);
			r = assist_v3_config_can_frame(5U, bad[i].frame_no, bad[i].end, golden_block_write_a, bad[i].dlen, NOW0);
			snap_take(&c);
			snprintf(msg, sizeof(msg), "G6 %s: one ERROR_ACK [1, frame]", bad[i].what);
			CHECK(r.kind == ASSIST_V3_REPLY_ERROR && r.reason == ASSIST_V3_REASON_LENGTH && r.index == bad[i].frame_no, msg);
			snprintf(msg, sizeof(msg), "G6 %s: transfer ended, RAM untouched", bad[i].what);
			CHECK(!c.live && memcmp(&a.ram, &c.ram, sizeof(a.ram)) == 0 && a.gen == c.gen, msg);
			r = assist_v3_config_can_frame(5U, 14U, true, &golden_block_write_a[112], 2U, NOW0);
			snprintf(msg, sizeof(msg), "G6 %s: no second result", bad[i].what);
			CHECK(r.kind == ASSIST_V3_REPLY_NONE, msg);
		}
	}
	/* duplicate frame */
	boot_blank();
	(void)assist_v3_config_can_declare(5U, 114U, NOW0);
	(void)assist_v3_config_can_frame(5U, 2U, false, &golden_block_write_a[16], 8U, NOW0);
	r = assist_v3_config_can_frame(5U, 2U, false, &golden_block_write_a[16], 8U, NOW0);
	CHECK(r.kind == ASSIST_V3_REPLY_ERROR && r.reason == ASSIST_V3_REASON_LENGTH && r.index == 2U, "G6 duplicate frame -> error");
	/* missing frame: END arrives with a hole -> error names the first missing frame, RAM untouched */
	boot_blank();
	(void)assist_v3_config_can_declare(5U, 114U, NOW0);
	snap_take(&a);
	for (f = 0U; f < 15U; f++) {
		uint8_t off = (uint8_t)(f * 8U);
		uint8_t n = (uint8_t)((114 - (int)off) > 8 ? 8 : (114 - (int)off));
		if (f == 6U) continue;
		r = assist_v3_config_can_frame(5U, (uint8_t)f, f == 14U, &golden_block_write_a[off], n, NOW0);
	}
	snap_take(&c);
	CHECK(r.kind == ASSIST_V3_REPLY_ERROR && r.reason == ASSIST_V3_REASON_LENGTH && r.index == 6U, "G6 missing frame -> error [1, first missing]");
	CHECK(snap_same(&a, &c) || (!c.live && memcmp(&a.ram, &c.ram, sizeof(a.ram)) == 0), "G6 missing frame: RAM untouched");
	/* a frame without any declaration */
	boot_blank();
	r = assist_v3_config_can_frame(5U, 0U, false, golden_block_write_a, 8U, NOW0);
	CHECK(r.kind == ASSIST_V3_REPLY_NONE, "G6 frame without declaration is ignored");
	/* an old transfer's frames never leak into the next one (staging is cleared on declare) */
	boot_blank();
	(void)assist_v3_config_can_declare(5U, 114U, NOW0);
	for (f = 0U; f < 14U; f++) (void)assist_v3_config_can_frame(5U, (uint8_t)f, false, &golden_block_write_a[f * 8U], 8U, NOW0);
	(void)assist_v3_config_can_frame(5U, 14U, false, golden_block_write_a, 8U, NOW0);   /* bad -> abort */
	(void)assist_v3_config_can_declare(5U, 114U, NOW0);
	r = assist_v3_config_can_frame(5U, 14U, true, &golden_block_write_a[112], 2U, NOW0);
	CHECK(r.kind == ASSIST_V3_REPLY_ERROR && r.index == 0U, "G6 new transfer starts from an empty frame mask");
}

/* ---------------- G7 ---------------- */

static void g7_persist(void)
{
	uint8_t blk[114], out[114];
	assist_v3_values_t ram_before, ram_after, saved_after;
	uint16_t gen_before;
	bool ok;
	assist_v3_reply_t r;
	boot_blank();
	CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_ABSENT, "G7 blank flash -> absent");
	CHECK(send_block(golden_block_write_a).result.kind == ASSIST_V3_REPLY_ACK, "G7 write");
	{
		uint8_t b2[114];
		uint16_t r2[5] = { 15, 25, 35, 45, 55 };
		(void)r2;
		(void)b2;
	}
	r = control(5U, ASSIST_V3_OP_PERSIST);
	CHECK(r.kind == ASSIST_V3_REPLY_ACK, "G7 persist request ACKed (deferred)");
	CHECK(assist_v3_config_persist_state() == ASSIST_V3_PERSIST_PENDING, "G7 persist_state = pending");
	(void)assist_v3_config_service(NOW0, false);
	CHECK(program_calls == 0U && assist_v3_config_persist_state() == ASSIST_V3_PERSIST_PENDING, "G7 nothing written while riding");
	(void)assist_v3_config_service(NOW0, true);
	CHECK(program_calls > 0U && assist_v3_config_persist_state() == ASSIST_V3_PERSIST_CLEAN, "G7 written at standstill, state clean");
	CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_VALID, "G7 flash_record_state valid");
	CHECK(rd16(&flash_mem[0]) == 0x3341U && flash_mem[2] == 'C' && flash_mem[3] == 'F', "G7 record magic in slot 0");
	assist_v3_config_ram_values(&ram_before);
	gen_before = assist_v3_config_generation();
	/* power cycle */
	restart();
	assist_v3_config_ram_values(&ram_after);
	assist_v3_config_saved_values(&saved_after);
	CHECK(memcmp(&ram_before, &ram_after, sizeof(ram_after)) == 0 && memcmp(&saved_after, &ram_after, sizeof(ram_after)) == 0, "G7 restart restores RAM and saved view");
	CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_VALID && assist_v3_config_persist_state() == ASSIST_V3_PERSIST_CLEAN, "G7 restart: valid, clean");
	CHECK(assist_v3_config_generation() != gen_before && assist_v3_config_generation() != 0xFFFFU, "G7 generation changes across a restart");
	read_view(ASSIST_V3_VIEW_SAVED, out, &ok);
	CHECK(ok && view_level(out, 3, 0) == 30U && rd16(&out[16]) == 0U, "G7 saved view readback after restart");
	CHECK(assist_v3_config_response_pct(2) == 20U && !assist_v3_config_engine_requested(), "G7 consumers see the restored values");
	/* persisting identical content does not consume a slot */
	{
		uint8_t before[ASSIST_V3_FLASH_PAGE_SIZE];
		memcpy(before, flash_mem, sizeof(before));
		(void)control(5U, ASSIST_V3_OP_PERSIST);
		(void)assist_v3_config_service(NOW0, true);
		CHECK(memcmp(before, flash_mem, sizeof(before)) == 0 && assist_v3_config_persist_state() == ASSIST_V3_PERSIST_CLEAN, "G7 identical persist writes nothing");
	}
	/* a changed value appends slot 1, newest wins */
	{
		const uint16_t r3[5] = { 11, 22, 33, 44, 55 };
		make_block(blk, 1U, r3, 0xFFFFU);
		CHECK(send_block(blk).result.kind == ASSIST_V3_REPLY_ACK, "G7 second write");
		(void)control(5U, ASSIST_V3_OP_PERSIST);
		(void)assist_v3_config_service(NOW0, true);
		CHECK(flash_mem[128] == 'A' && flash_mem[128 + 1] == '3', "G7 second record in slot 1");
		restart();
		CHECK(assist_v3_config_response_pct(1) == 11U && assist_v3_config_engine_requested(), "G7 newest valid record wins");
	}
	/* persist is never implicit: write + restart without persist loses the RAM change */
	{
		const uint16_t r4[5] = { 99, 99, 99, 99, 99 };
		make_block(blk, 0U, r4, 0xFFFFU);
		(void)send_block(blk);
		restart();
		CHECK(assist_v3_config_response_pct(1) == 11U, "G7 unpersisted write is gone after restart");
	}
	/* persist request does not run while a V3 transfer is live */
	{
		(void)control(5U, ASSIST_V3_OP_PERSIST);
		(void)assist_v3_config_can_declare(5U, 114U, NOW0);
		program_calls = 0U;
		(void)assist_v3_config_service(NOW0 + 10U, true);
		CHECK(program_calls == 0U, "G7 flash is not touched while a transfer is live");
	}
}

/* ---------------- G8 ---------------- */

static void put_slot(unsigned idx, const uint8_t *slot) { memcpy(&flash_mem[idx * 128U], slot, 128U); }

static void make_slot(uint8_t slot[128], uint8_t schema_id, uint8_t version, uint8_t stride, uint8_t levels, uint16_t gen,
                      const uint8_t *payload, uint16_t payload_len)
{
	memset(slot, 0xFF, 128U);
	wr32(slot, 0x46433341UL);
	slot[4] = schema_id; slot[5] = version; slot[6] = stride; slot[7] = levels;
	wr16(&slot[8], gen);
	wr16(&slot[10], payload_len);
	memcpy(&slot[12], payload, payload_len);
	wr32(&slot[124], assist_v3_crc32(slot, 124U));
}

static void g8_corrupt_newer_migration(void)
{
	uint8_t slot[128], payload[96];
	uint8_t i;
	assist_v3_values_t v;

	/* corrupt: one flipped payload bit with a valid-looking slot */
	boot_blank();
	(void)send_block(golden_block_write_a);
	(void)control(5U, ASSIST_V3_OP_PERSIST);
	(void)assist_v3_config_service(NOW0, true);
	flash_mem[12U + 16U + 1U] ^= 0x04U;
	restart();
	assist_v3_config_ram_values(&v);
	CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_CORRUPT, "G8 corrupt record -> state 3");
	CHECK(v.global[0] == 0xFFFFU && assist_v3_config_response_pct(1) == 40U && assist_v3_config_engine_requested(), "G8 corrupt record -> defaults");
	/* the corrupt record is skipped, the next persist goes to the next slot and then wins */
	(void)send_block(golden_block_write_a);
	(void)control(5U, ASSIST_V3_OP_PERSIST);
	(void)assist_v3_config_service(NOW0, true);
	CHECK(flash_mem[128] == 'A', "G8 persist after a corrupt slot appends after it");
	restart();
	CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_VALID && assist_v3_config_response_pct(1) == 10U, "G8 recovery: new record valid");

	/* corrupt NEWEST slot, older valid one: the older one wins (interrupted persist) */
	boot_blank();
	(void)send_block(golden_block_write_a);
	(void)control(5U, ASSIST_V3_OP_PERSIST);
	(void)assist_v3_config_service(NOW0, true);
	{
		const uint16_t r5[5] = { 77, 77, 77, 77, 77 };
		uint8_t b[114];
		make_block(b, 0U, r5, 0xFFFFU);
		(void)send_block(b);
		(void)control(5U, ASSIST_V3_OP_PERSIST);
		(void)assist_v3_config_service(NOW0, true);
	}
	flash_mem[128U + 124U] ^= 0xFFU;   /* newest slot's crc broken */
	restart();
	CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_VALID && assist_v3_config_response_pct(1) == 10U, "G8 corrupt newest slot -> previous record");

	/* newer schema: defaults, flash untouched */
	boot_blank();
	memset(payload, 0xFF, sizeof(payload));
	wr16(&payload[0], 0U);          /* engine 0 */
	wr16(&payload[16], 55U);        /* level 1 response */
	make_slot(slot, 1U, 2U, 16U, 5U, 9U, payload, 96U);
	put_slot(0U, slot);
	{
		uint8_t before[2048];
		memcpy(before, flash_mem, sizeof(before));
		restart();
		CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_NEWER, "G8 newer schema -> state 2");
		CHECK(assist_v3_config_response_pct(1) == 40U && assist_v3_config_engine_requested(), "G8 newer schema -> defaults in RAM");
		CHECK(memcmp(before, flash_mem, sizeof(before)) == 0 && program_calls == 0U && erase_count == 0U, "G8 newer record left untouched at boot");
		(void)send_block(golden_block_write_a);
		CHECK(memcmp(before, flash_mem, sizeof(before)) == 0, "G8 still untouched by a RAM-only write");
		(void)control(5U, ASSIST_V3_OP_PERSIST);
		(void)assist_v3_config_service(NOW0, true);
		CHECK(memcmp(before, flash_mem, 128U) == 0 && flash_mem[128] == 'A', "G8 explicit persist appends after the newer record");
		restart();
		CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_VALID && assist_v3_config_response_pct(1) == 10U, "G8 our record is now the newest valid one");
	}
	/* different schema_id / more levels: also 'newer' (cannot be interpreted) */
	boot_blank();
	make_slot(slot, 7U, 1U, 16U, 5U, 1U, payload, 96U);
	put_slot(0U, slot);
	restart();
	CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_NEWER, "G8 foreign schema_id -> state 2");

	/* migration: stride 8 record is back-filled with 0xFFFF */
	boot_blank();
	memset(payload, 0xFF, sizeof(payload));
	wr16(&payload[0], 0U);                           /* global engine = 0 */
	for (i = 0U; i < 5U; i++) {
		wr16(&payload[8U + (uint16_t)i * 8U], (uint16_t)(30U + i));      /* response */
		wr16(&payload[8U + (uint16_t)i * 8U + 2U], 60U);                 /* start_response (reserved here: dropped) */
	}
	make_slot(slot, 1U, 1U, 8U, 5U, 4U, payload, 48U);
	put_slot(0U, slot);
	restart();
	assist_v3_config_ram_values(&v);
	CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_VALID, "G8 short-stride record loads");
	CHECK(v.global[0] == 0U && v.level[0][0] == 30U && v.level[4][0] == 34U, "G8 short-stride values loaded");
	CHECK(v.level[0][1] == 0xFFFFU && v.level[0][4] == 0xFFFFU && v.level[0][7] == 0xFFFFU, "G8 missing / reserved words are 0xFFFF");
	CHECK(assist_v3_config_generation() == 5U, "G8 generation continues from the record");
	/* a record with an out-of-range value (valid crc) is not trusted */
	boot_blank();
	memset(payload, 0xFF, sizeof(payload));
	wr16(&payload[16], 101U);
	make_slot(slot, 1U, 1U, 16U, 5U, 1U, payload, 96U);
	put_slot(0U, slot);
	restart();
	CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_CORRUPT && assist_v3_config_response_pct(1) == 40U, "G8 out-of-range stored value -> corrupt, defaults");
	/* an interrupted blank-ish page: only a partial slot -> corrupt, defaults, nothing crashes */
	boot_blank();
	memset(flash_mem, 0, 8U);
	restart();
	CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_CORRUPT && assist_v3_config_response_pct(1) == 40U, "G8 stray partial slot -> corrupt, defaults");
	/* NULL / missing flash never crashes and reports absent */
	assist_v3_config_init(0);
	CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_ABSENT && assist_v3_config_response_pct(1) == 40U, "G8 no flash -> absent, defaults");
	(void)control(5U, ASSIST_V3_OP_PERSIST);
	(void)assist_v3_config_service(NOW0, true);
	CHECK(assist_v3_config_persist_state() == ASSIST_V3_PERSIST_FAILED, "G8 persist without flash -> failed (reported)");
}

/* ---------------- G9 ---------------- */

static void write_and_persist(uint16_t resp1)
{
	uint8_t b[114];
	const uint16_t r[5] = { resp1, 20, 30, 40, 50 };
	make_block(b, 0U, r, 0xFFFFU);
	(void)send_block(b);
	(void)control(5U, ASSIST_V3_OP_PERSIST);
	(void)assist_v3_config_service(NOW0, true);
}

static void fill_log(void)
{
	uint16_t k;
	for (k = 0U; k < ASSIST_V3_SLOT_COUNT; k++) write_and_persist((uint16_t)(1U + k));
}

static void g9_log_and_power_loss(void)
{
	unsigned i;
	assist_v3_values_t v;

	CHECK(ASSIST_V3_SLOT_COUNT == 16U, "G9 16 slots per page");
	/* 16 persists, no erase; the 17th erases once and continues */
	boot_blank();
	fill_log();
	CHECK(erase_count == 0U && assist_v3_config_persist_state() == ASSIST_V3_PERSIST_CLEAN, "G9 16 persists cost no erase");
	restart();
	CHECK(assist_v3_config_response_pct(1) == 16U, "G9 slot 15 holds the newest");
	write_and_persist(90U);
	CHECK(erase_count == 1U && assist_v3_config_persist_state() == ASSIST_V3_PERSIST_CLEAN, "G9 17th persist erases exactly once");
	CHECK(flash_mem[0] == 'A' && flash_mem[128] == 0xFF, "G9 new record is first in the fresh page");
	restart();
	CHECK(assist_v3_config_response_pct(1) == 90U && assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_VALID, "G9 readback after erase");
	write_and_persist(91U);
	CHECK(erase_count == 1U && flash_mem[128] == 'A', "G9 continues in slot 1 without another erase");
	restart();
	CHECK(assist_v3_config_response_pct(1) == 91U, "G9 continue readback");

	/* erase-window power loss: erase done, nothing programmed */
	boot_blank();
	fill_log();
	fi_word_budget = 0;
	write_and_persist(95U);
	CHECK(assist_v3_config_persist_state() == ASSIST_V3_PERSIST_FAILED, "G9 failed persist is reported (state 3)");
	CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_ABSENT, "G9 erased page, no record -> absent");
	fi_word_budget = -1;
	restart();
	assist_v3_config_ram_values(&v);
	CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_ABSENT && assist_v3_config_response_pct(1) == 40U && v.global[0] == 0xFFFFU, "G9 power loss in the erase window -> defaults, not a corrupt config");
	/* retry after the loss succeeds */
	write_and_persist(33U);
	restart();
	CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_VALID && assist_v3_config_response_pct(1) == 33U, "G9 retry after power loss succeeds");

	/* power loss at every word of the first program after an erase: never a corrupt/mixed config */
	for (i = 0U; i <= 32U; i++) {
		unsigned r1;
		boot_blank();
		fill_log();
		fi_word_budget = (long)i;
		write_and_persist(95U);
		fi_word_budget = -1;
		restart();
		r1 = assist_v3_config_response_pct(1);
		if (i >= 32U) {
			CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_VALID && r1 == 95U, "G9 full program -> new record");
		} else {
			CHECK((assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_ABSENT ||
			       assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_CORRUPT) && r1 == 40U,
			      "G9 interrupted program (crc not written) -> defaults");
		}
	}
	/* interrupted program mid-log (no erase): the previous record survives */
	for (i = 0U; i < 32U; i += 7U) {
		boot_blank();
		write_and_persist(21U);
		fi_word_budget = (long)i;
		write_and_persist(22U);
		fi_word_budget = -1;
		restart();
		CHECK(assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_VALID && assist_v3_config_response_pct(1) == 21U, "G9 interrupted append -> previous record wins");
		write_and_persist(23U);          /* appends after the dead slot */
		restart();
		CHECK(assist_v3_config_response_pct(1) == 23U, "G9 recovery after an interrupted append");
	}
	/* an interrupted erase (half the page wiped) never produces a mixed record */
	boot_blank();
	fill_log();
	fi_erase_half = true;
	write_and_persist(77U);
	CHECK(assist_v3_config_persist_state() == ASSIST_V3_PERSIST_FAILED, "G9 failed erase reported");
	fi_erase_half = false;
	restart();
	CHECK((assist_v3_config_flash_record_state() == ASSIST_V3_FLASH_VALID && assist_v3_config_response_pct(1) == 16U) ||
	      (assist_v3_config_flash_record_state() != ASSIST_V3_FLASH_VALID && assist_v3_config_response_pct(1) == 40U),
	      "G9 half-erased page: old newest record or defaults, never a mix");
}

/* ---------------- G10 ---------------- */

static void g10_control_engine_getters(void)
{
	uint8_t blk[114], caps[26], req[2], len = 0U;
	bool ok;
	assist_v3_reply_t r;
	uint8_t op;
	snap_t a, c;
	uint8_t lvl;
	const uint8_t want_resp[5] = { 40, 60, 80, 90, 80 };

	boot_blank();
	/* consumers at boot */
	for (lvl = 1U; lvl <= 5U; lvl++) CHECK(assist_v3_config_response_pct(lvl) == want_resp[lvl - 1U], "G10 compiled default response per level");
	CHECK(assist_v3_config_response_pct(0U) == 0U && assist_v3_config_response_pct(6U) == 0U, "G10 invalid level -> 0");
	CHECK(assist_v3_config_get(3U, ASSIST_V3_PARAM_CARRY_STRENGTH) == 50U && assist_v3_config_get(4U, ASSIST_V3_PARAM_CARRY_EXTENT) == 80U, "G10 table values for carry");

	/* engine: requested vs active */
	CHECK(assist_v3_config_engine_requested() && assist_v3_config_engine_active(), "G10 boot: engine default V3, active == requested");
	{
		uint8_t b[114];
		make_block(b, 0U, RESP_A, 0xFFFFU);
		(void)send_block(b);
	}
	CHECK(!assist_v3_config_engine_requested() && assist_v3_config_engine_active(), "G10 request G5300: active unchanged until the pipeline latches");
	(void)assist_v3_config_can_read(5U, ASSIST_V3_CMD_CAPS, 0U, caps, caps, &len);
	CHECK(caps[22] == 1U && caps[23] == 0U, "G10 caps report engine_active=1, engine_requested=0");
	assist_v3_config_set_engine_active(false);
	(void)assist_v3_config_can_read(5U, ASSIST_V3_CMD_CAPS, 0U, caps, caps, &len);
	CHECK(caps[22] == 0U && caps[23] == 0U && rd16(&caps[24]) == assist_v3_crc16(caps, 24U), "G10 pipeline setter shows in caps, crc valid");
	(void)control(5U, ASSIST_V3_OP_PERSIST);
	(void)assist_v3_config_service(NOW0, true);
	restart();
	CHECK(!assist_v3_config_engine_requested() && !assist_v3_config_engine_active(), "G10 restart: engine_active = engine_requested");

	/* revert / defaults */
	{
		uint8_t b[114];
		const uint16_t r6[5] = { 1, 2, 3, 4, 5 };
		make_block(b, 1U, r6, 0xFFFFU);
		(void)send_block(b);
	}
	CHECK(assist_v3_config_response_pct(1) == 1U, "G10 RAM changed");
	r = control(5U, ASSIST_V3_OP_REVERT);
	CHECK(r.kind == ASSIST_V3_REPLY_ACK && assist_v3_config_response_pct(1) == 10U && !assist_v3_config_engine_requested(), "G10 revert -> saved values");
	CHECK(assist_v3_config_persist_state() == ASSIST_V3_PERSIST_CLEAN, "G10 revert: clean again");
	r = control(5U, ASSIST_V3_OP_DEFAULTS);
	read_view(ASSIST_V3_VIEW_EFFECTIVE, blk, &ok);
	{
		uint8_t def[114];
		bool ok2;
		read_view(ASSIST_V3_VIEW_DEFAULTS, def, &ok2);
		CHECK(r.kind == ASSIST_V3_REPLY_ACK && ok && ok2 && memcmp(&blk[16], &def[16], 98U) == 0, "G10 defaults op: effective == defaults view");
	}
	CHECK(assist_v3_config_persist_state() == ASSIST_V3_PERSIST_DIRTY, "G10 defaults op leaves RAM dirty vs saved");
	{
		assist_v3_values_t sv;
		assist_v3_config_saved_values(&sv);
		CHECK(sv.level[0][0] == 10U, "G10 defaults op does not touch the saved copy");
	}
	/* control guards */
	snap_take(&a);
	op = 9U;
	r = assist_v3_config_can_control(5U, 1U, &op);
	snap_take(&c);
	CHECK(r.kind == ASSIST_V3_REPLY_ERROR && r.reason == ASSIST_V3_REASON_RANGE && snap_same(&a, &c), "G10 unknown op -> error, no change");
	r = assist_v3_config_can_control(5U, 0U, &op);
	CHECK(r.kind == ASSIST_V3_REPLY_ERROR && r.reason == ASSIST_V3_REASON_LENGTH, "G10 DLC 0 -> error");
	op = ASSIST_V3_OP_DEFAULTS;
	r = assist_v3_config_can_control(3U, 1U, &op);
	CHECK(r.kind == ASSIST_V3_REPLY_NONE, "G10 control from source 3 is silent");
	(void)assist_v3_config_can_declare(5U, 114U, NOW0);
	snap_take(&a);
	r = control(5U, ASSIST_V3_OP_REVERT);
	snap_take(&c);
	CHECK(r.kind == ASSIST_V3_REPLY_ERROR && r.reason == ASSIST_V3_REASON_BUSY && snap_same(&a, &c), "G10 revert during a live transfer -> busy, no change");
	r = control(5U, ASSIST_V3_OP_DEFAULTS);
	CHECK(r.kind == ASSIST_V3_REPLY_ERROR && r.reason == ASSIST_V3_REASON_BUSY, "G10 defaults during a live transfer -> busy");

	/* READ guards */
	boot_blank();
	req[0] = 3U; req[1] = 0U;
	r = assist_v3_config_can_read(5U, ASSIST_V3_CMD_BLOCK, 2U, req, blk, &len);
	CHECK(r.kind == ASSIST_V3_REPLY_ERROR && r.reason == ASSIST_V3_REASON_RANGE && len == 0U, "G10 READ view 3 -> error");
	req[0] = 1U; req[1] = 1U;
	r = assist_v3_config_can_read(5U, ASSIST_V3_CMD_BLOCK, 2U, req, blk, &len);
	CHECK(r.kind == ASSIST_V3_REPLY_ERROR && len == 0U, "G10 READ set 1 -> error");
	r = assist_v3_config_can_read(5U, ASSIST_V3_CMD_BLOCK, 1U, req, blk, &len);
	CHECK(r.kind == ASSIST_V3_REPLY_ERROR && r.reason == ASSIST_V3_REASON_LENGTH && len == 0U, "G10 READ without view/set -> error");
	r = assist_v3_config_can_read(3U, ASSIST_V3_CMD_BLOCK, 2U, req, blk, &len);
	CHECK(r.kind == ASSIST_V3_REPLY_NONE && len == 0U, "G10 READ 0x6036 from source 3 is silent");
	r = assist_v3_config_can_read(5U, ASSIST_V3_CMD_CONTROL, 0U, req, blk, &len);
	CHECK(r.kind == ASSIST_V3_REPLY_NONE && len == 0U, "G10 READ 0x6037 has no reply");
	CHECK(assist_v3_config_owns_command(0x6035U) && assist_v3_config_owns_command(0x6036U) && assist_v3_config_owns_command(0x6037U) &&
	      !assist_v3_config_owns_command(0x6034U) && !assist_v3_config_owns_command(0x6038U), "G10 command range is exactly 0x6035..0x6037");
}

/* ---------------- G11 ---------------- */

static char *read_whole_file(const char *path)
{
	FILE *f = fopen(path, "rb");
	char *buf;
	long len;
	size_t got;
	if (!f) return NULL;
	fseek(f, 0, SEEK_END);
	len = ftell(f);
	fseek(f, 0, SEEK_SET);
	buf = (char *)malloc((size_t)len + 1U);
	if (!buf) { fclose(f); return NULL; }
	got = fread(buf, 1, (size_t)len, f);
	fclose(f);
	buf[got] = '\0';
	return buf;
}

static unsigned count_of(const char *hay, const char *needle)
{
	unsigned n = 0U;
	const char *p = hay;
	while ((p = strstr(p, needle)) != NULL) { n++; p += strlen(needle); }
	return n;
}

#define STR2(x) #x
#define STR(x) STR2(x)

static void g11_wiring_guards(void)
{
	char *src = read_whole_file(STR(CAN_DISPLAY_C_PATH));
	const char *v3_write, *legacy_decl, *ack_excl, *v3_read, *v3_end;
	if (!src) { CHECK(0, "G11 cannot read CAN_Display.c"); return; }

	/* legacy P0/P1/P2, bank and tuning transfer code, character for character */
	CHECK(count_of(src, "append_multiframe(0, &Para0[0]);") == 1U, "G11 P0 START unchanged");
	CHECK(count_of(src, "append_multiframe(Ext_ID_Rx.command+1, &Para0[0]);") == 2U, "G11 P0 TRANG/END unchanged");
	CHECK(count_of(src, "append_multiframe(Ext_ID_Rx.command+1, &Para1[0]);") == 2U, "G11 P1 TRANG/END unchanged");
	CHECK(count_of(src, "append_multiframe(Ext_ID_Rx.command+1, &Para2[0]);") == 2U, "G11 P2 TRANG/END unchanged");
	CHECK(count_of(src, "append_multiframe(0, (char*)&BankBlob[0]);") == 1U, "G11 bank START unchanged");
	CHECK(count_of(src, "if(Ext_ID_Rx.command < 31) append_multiframe(Ext_ID_Rx.command+1, (char*)&BankBlob[0]);") == 2U, "G11 bank frame bound unchanged");
	CHECK(count_of(src, "if(Ext_ID_Rx.command < 3) append_multiframe(Ext_ID_Rx.command+1, (char*)&TuningBlob[0]);") == 2U, "G11 tuning frame bound unchanged");
	CHECK(count_of(src, "send_multiframe(Ext_ID_Rx.command, &Para0[0],64 );") == 1U, "G11 P0 READ unchanged");
	CHECK(count_of(src, "send_multiframe(Ext_ID_Rx.command, &Para1[0],64 );") == 1U, "G11 P1 READ unchanged");
	CHECK(count_of(src, "send_multiframe_trailer(Ext_ID_Rx.command, &Para2[0], 64, &trailer);") == 1U, "G11 P2 READ unchanged");
	CHECK(count_of(src, "send_multiframe(Ext_ID_Rx.command, (char*)&BankBlob[0], ASSIST_BANK_BLOB_LEN);") == 1U, "G11 bank READ unchanged");
	CHECK(count_of(src, "else if(Ext_ID_Rx.command==0x6022 && Ext_ID_Rx.source==5){") == 1U, "G11 persist trigger 0x6022 unchanged");
	CHECK(count_of(src, "parse_DPparams(MP);") == 1U && count_of(src, "write_virtual_eeprom();") >= 3U, "G11 P0/P1/P2 completion path present once");
	CHECK(count_of(src, "uint8_t applied = assist_modes_apply_bank_blob(&BankBlob[0], rx_data_length) ? 1U : 0U;") == 1U, "G11 bank apply unchanged");
	CHECK(count_of(src, "uint8_t applied = tuning_config_apply_blob(&TuningBlob[0], rx_data_length) ? 1U : 0U;") == 1U, "G11 tuning apply unchanged");

	/* V3 wiring */
	v3_write = strstr(src, "else if(assist_v3_config_owns_command(Ext_ID_Rx.command)){");
	legacy_decl = strstr(src, "else if (receive_message.rx_dlen==1 && receive_message.rx_data[0]>8 && Ext_ID_Rx.source==5){");
	ack_excl = strstr(src, "&& !assist_v3_config_owns_command(Ext_ID_Rx.command)");
	CHECK(v3_write != NULL && legacy_decl != NULL && v3_write < legacy_decl, "G11 V3 WRITE branch precedes the generic declaration branch");
	CHECK(ack_excl != NULL, "G11 generic ACK is excluded for 0x6035..0x6037");
	CHECK(count_of(src, "assist_v3_config_owns_command(Ext_ID_Rx.command)") == 3U, "G11 V3 dispatch: WRITE, READ and ACK exclusion only");
	if (v3_write && legacy_decl) {
		char saved = *legacy_decl;
		*(char *)legacy_decl = '\0';
		CHECK(strstr(v3_write, "Para0") == NULL && strstr(v3_write, "Para1") == NULL && strstr(v3_write, "Para2") == NULL &&
		      strstr(v3_write, "BankBlob") == NULL && strstr(v3_write, "TuningBlob") == NULL && strstr(v3_write, "write_virtual_eeprom") == NULL,
		      "G11 V3 WRITE branch never touches P0/P1/P2, bank, tuning or the core flash record");
		*(char *)legacy_decl = saved;
	}
	v3_read = strstr(src, "/* Assist V3 config: tool only; replies are multiframe payloads built by the owner. */");
	v3_end = v3_read ? strstr(v3_read, "send_multiframe(Ext_ID_Rx.command,(char*)v3buf,v3len);") : NULL;
	CHECK(v3_read != NULL && v3_end != NULL, "G11 V3 READ branch present");
	CHECK(strstr(src, "case ASSIST_V3_CMD_BLOCK: //Assist V3: own staging buffer inside the owner module") != NULL &&
	      count_of(src, "v3_cfg_frame(") >= 4U, "G11 V3 frames go to the owner module's own staging buffer");
	free(src);
}

int main(void)
{
	g1_vectors();
	g2_roundtrip();
	g3_rejects();
	g4_reserved();
	g5_generation();
	g6_transfer();
	g7_persist();
	g8_corrupt_newer_migration();
	g9_log_and_power_loss();
	g10_control_engine_getters();
	g11_wiring_guards();
	if (host_test_failures != 0) {
		printf("assist_v3_config: %d FAILED\n", host_test_failures);
		return 1;
	}
	printf("assist_v3_config: PASS\n");
	return 0;
}
