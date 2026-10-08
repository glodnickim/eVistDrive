#ifndef ASSIST_V3_HARNESS_H_
#define ASSIST_V3_HARNESS_H_
/*
 * Harness helper shared by the SIL, Level-4 and host suites that run the pipeline with
 * -DASSIST_V3: select the V3 config engine EXPLICITLY (TEST_MATRIX G-EQ rule 1).
 *
 * WHY. The candidate default for an absent CONFIG_A record is V3 (DECISIONS D-020) and no
 * harness has a flash record, so an implicit engine would make every equivalence depend on that
 * default. The value is written through the real CONFIG_PROTOCOL_V3 transfer path (declaration +
 * 15 frames of 0x6036, source = tool), exactly as CANable writes it - no module state is poked.
 *
 * Header-only (static) so each single-translation-unit harness can include it without a
 * build-script change. Returns false if the write was rejected; the caller must fail loudly.
 */
#include <stdbool.h>
#include <stdint.h>

#include "assist_v3_config.h"

static bool assist_v3_harness_select_engine(bool v3)
{
	assist_v3_values_t v;
	uint8_t blk[ASSIST_V3_BLOCK_LEN];
	assist_v3_reply_t r;
	uint8_t f;

	assist_v3_config_init(0);   /* absent record: compiled defaults */
	assist_v3_config_ram_values(&v);
	v.engine = v3 ? ASSIST_V3_ENGINE_V3 : ASSIST_V3_ENGINE_G5300;
	assist_v3_block_encode(&v, ASSIST_V3_CAPS, assist_v3_config_generation(), blk);
	r = assist_v3_config_can_declare(ASSIST_V3_SOURCE_TOOL, ASSIST_V3_BLOCK_LEN, 0U);
	if (r.kind != ASSIST_V3_REPLY_DECL_ACK) return false;
	for (f = 0U; ; f++) {
		const uint16_t off = (uint16_t)((uint16_t)f * 8U);
		const bool end = off + 8U >= ASSIST_V3_BLOCK_LEN;
		const uint8_t dlen = (uint8_t)(end ? ASSIST_V3_BLOCK_LEN - off : 8U);
		r = assist_v3_config_can_frame(ASSIST_V3_SOURCE_TOOL, f, end, &blk[off], dlen, 0U);
		if (end) break;
		if (r.kind != ASSIST_V3_REPLY_NONE) return false;   /* an early abort */
	}
	if (r.kind != ASSIST_V3_REPLY_ACK || assist_v3_config_engine_requested() != v3) return false;
	/* engine_active is NOT poked: it stays G5300 (boot value) until the pipeline latch
	 * (ARCHITECTURE_V3 2.2, src/assist_pipeline.c) finds every demand at 0 with no veto - the
	 * harness exercises the production latch exactly as the firmware does. */
	return true;
}

#endif /* ASSIST_V3_HARNESS_H_ */
