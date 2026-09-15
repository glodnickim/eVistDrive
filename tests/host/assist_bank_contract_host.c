/*
 * THE STORED CONFIGURATION CONTRACT - bank blob round trip, restart, and the meaning of zero.
 *
 * This suite exists because a configuration audit found the firmware REFUSING ITS OWN DEFAULT
 * BANK: the validator listed the five legacy mode numbers, while every shipped V2 bank is made
 * of 7..12. Nothing caught it. Every host suite drove the pipeline through the in-RAM config
 * structs, and the one path that turns those structs into bytes and back - the path the app
 * writes through and the path the controller restores through at every boot - had no test at
 * all. A bank could be written, acknowledged, stored, and then silently come back as compiled
 * defaults on the next power-up.
 *
 * So the checks here are deliberately about BYTES AND RESTARTS, not about ride feel:
 *
 *   B1  the shipped defaults survive their own round trip, byte for byte, in both banks
 *   B2  a restart restores what was saved - resolved settings, not just structure
 *   B3  every mode number this firmware claims to support is accepted, unknown ones are not
 *   B4  a rejected bank mutates nothing - not even the levels ahead of the bad record
 *   B5  zero in a ramp field stays zero: "the profile decides" is not 20 ms
 *   B6  zero in max_iq_pct means the level is switched OFF, and means it everywhere
 *   B7  banks written by older firmware still load
 *
 * A CRC is computed here independently of the production one on purpose: a round trip that
 * agreed with itself through a shared bug would prove nothing.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ap2_profiles.h"
#include "assist_modes.h"
#include "config.h"

static int failures;

#define CHECK(cond, what) do { \
	if (!(cond)) { \
		printf("  FAIL  %s\n", (what)); \
		failures++; \
	} \
} while (0)

#define LEVELS 5
#define BANKS 2
#define HEADER_LEN 13
#define RECORD_LEN_V5 35
#define RECORD_LEN_V7 46
#define RECORD_LEN_V8 48

/* Independent CRC-16/CCITT-FALSE over the blob, excluding its own two trailing bytes. */
static uint16_t crc16(const uint8_t *buffer, uint16_t length)
{
	uint16_t crc = 0xFFFFU;
	uint16_t i;
	uint8_t bit;

	for (i = 0; i < length; i++) {
		crc ^= (uint16_t)buffer[i] << 8;
		for (bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) :
				(uint16_t)(crc << 1);
		}
	}
	return crc;
}

static uint16_t rd16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void wr16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFU);
	p[1] = (uint8_t)(v >> 8);
}

static uint8_t *record_of(uint8_t *blob, uint8_t level_index_from_1)
{
	return &blob[HEADER_LEN + (level_index_from_1 - 1) * RECORD_LEN_V8];
}

/*
 * WHAT THE FIRMWARE WOULD ACTUALLY DO WITH A LEVEL - the only comparison worth making across a
 * save/restore. Two configurations that serialize to different bytes but resolve to the same
 * behaviour are equivalent; two that look alike in a struct dump and resolve differently are
 * not. This is the "compare the resolved C settings, not the structure" the audit asked for.
 */
typedef struct {
	uint8_t profile_id;
	uint16_t gain_pct;
	uint16_t attack_ms;
	uint16_t release_ms;
	uint16_t start_ms;
	uint16_t max_power_w;
	uint8_t characteristic;
	int32_t iq_ceiling;
	bool disabled;
} resolved_t;

static void resolve_level(uint8_t level, resolved_t *out)
{
	const assist_level_config_t *cfg = assist_modes_get_default_level(level);
	ap2_profile_override_t ovr;
	ap2_profile_resolved_t res;
	ap2_auto_input_t in;

	memset(&in, 0, sizeof(in));
	/* A fixed, ordinary riding point. AUTO is a state machine, so it is reset first and
	 * evaluated from the same starting state every time - otherwise "the same settings"
	 * could not be compared at all. */
	in.demand_permille = 400;
	in.aggression_permille = 200;
	in.load_permille = 300;
	in.cadence_rpm = 60;
	in.speed_x100 = 1800;
	in.pedaling = true;
	in.elapsed_ticks = 4;

	assist_modes_profile_override(cfg, &ovr);
	ap2_profiles_reset();
	ap2_profiles_resolve(assist_modes_profile_for_level(cfg), &ovr, &in, &res);

	out->profile_id = (uint8_t)res.id;
	out->gain_pct = res.p.assist_gain_pct;
	out->attack_ms = res.p.attack_ms;
	out->release_ms = res.p.release_ms;
	out->start_ms = res.p.start_ms;
	out->max_power_w = res.p.max_power_w;
	out->characteristic = res.p.characteristic;
	out->iq_ceiling = assist_modes_level_iq_limit(cfg, (int32_t)PH_CURRENT_MAX,
		(int32_t)PH_CURRENT_MAX);
	out->disabled = assist_modes_level_disables_assist(cfg);
}

static void resolve_bank(uint8_t bank, resolved_t out[LEVELS + 1])
{
	uint8_t level;

	assist_modes_set_active_bank(bank);
	for (level = 1; level <= LEVELS; level++) {
		resolve_level(level, &out[level]);
	}
}

static bool resolved_equal(const resolved_t *a, const resolved_t *b)
{
	return a->profile_id == b->profile_id &&
		a->gain_pct == b->gain_pct &&
		a->attack_ms == b->attack_ms &&
		a->release_ms == b->release_ms &&
		a->start_ms == b->start_ms &&
		a->max_power_w == b->max_power_w &&
		a->characteristic == b->characteristic &&
		a->iq_ceiling == b->iq_ceiling &&
		a->disabled == b->disabled;
}

/* ------------------------------------------------------------------------------------------ */

static void b1_defaults_round_trip(void)
{
	uint8_t first[BANKS][ASSIST_BANK_BLOB_LEN];
	uint8_t again[ASSIST_BANK_BLOB_LEN];
	uint8_t bank;

	printf("B1 the shipped default banks survive their own round trip\n");

	assist_modes_init();
	for (bank = 0; bank < BANKS; bank++) {
		uint16_t len = assist_modes_serialize_bank(bank, first[bank]);
		CHECK(len == ASSIST_BANK_BLOB_LEN, "B1: a bank serializes to the full blob length");
		CHECK(crc16(first[bank], ASSIST_BANK_BLOB_LEN - 2) ==
			rd16(&first[bank][ASSIST_BANK_BLOB_LEN - 2]),
			"B1: the stored CRC matches an independent one");
	}

	for (bank = 0; bank < BANKS; bank++) {
		/*
		 * THE FINDING ITSELF. This returned false: bank_mode_valid() listed the five legacy
		 * modes, the shipped banks are made of 7..12, so the firmware rejected the bytes it
		 * had just written. The same parser runs on the boot restore path.
		 */
		CHECK(assist_modes_apply_bank_blob(first[bank], ASSIST_BANK_BLOB_LEN),
			"B1: the firmware accepts a bank it serialized itself");
		assist_modes_serialize_bank(bank, again);
		CHECK(memcmp(first[bank], again, ASSIST_BANK_BLOB_LEN) == 0,
			"B1: serialize -> apply -> serialize is byte identical");
	}
}

static void b2_restart_restores_behaviour(void)
{
	uint8_t flash[BANKS][ASSIST_BANK_BLOB_LEN];
	resolved_t before[BANKS][LEVELS + 1];
	resolved_t after[BANKS][LEVELS + 1];
	uint8_t bank;
	uint8_t level;
	uint8_t edited[ASSIST_BANK_BLOB_LEN];

	printf("B2 a restart restores the saved configuration, not the compiled defaults\n");

	/* A rider edits bank 0: level 2 becomes SPORT with a hand-set attack and a power ceiling,
	 * level 4 gets a torque ceiling of 40 %. Everything else stays as shipped. */
	assist_modes_init();
	assist_modes_serialize_bank(0, edited);
	record_of(edited, 2)[0] = 9U;                 /* ASSIST_MODE_V2_SPORT */
	wr16(&record_of(edited, 2)[40], 350U);        /* iq_rise_fast_ms -> attack override */
	wr16(&record_of(edited, 2)[15], 420U);        /* max_motor_power_w */
	record_of(edited, 4)[17] = 40U;               /* max_iq_pct */
	wr16(&edited[ASSIST_BANK_BLOB_LEN - 2], crc16(edited, ASSIST_BANK_BLOB_LEN - 2));
	CHECK(assist_modes_apply_bank_blob(edited, ASSIST_BANK_BLOB_LEN),
		"B2: an edited bank is accepted");

	/* ... and it is what gets written to flash. */
	for (bank = 0; bank < BANKS; bank++) {
		assist_modes_serialize_bank(bank, flash[bank]);
		resolve_bank(bank, before[bank]);
	}

	CHECK(before[0][2].attack_ms == 350U, "B2: the edited attack time is in force before reset");
	CHECK(before[0][2].max_power_w == 420U, "B2: the edited power ceiling is in force");
	CHECK(before[0][4].iq_ceiling == (int32_t)PH_CURRENT_MAX * 40 / 100,
		"B2: the edited torque ceiling is in force");

	/* POWER CYCLE: exactly what src/main.c does - init to compiled defaults, then hand both
	 * stored blobs to the same parser. */
	assist_modes_init();
	for (bank = 0; bank < BANKS; bank++) {
		CHECK(assist_modes_apply_bank_blob(flash[bank], ASSIST_BANK_BLOB_LEN),
			"B2: the stored bank is accepted on the restore path");
	}
	for (bank = 0; bank < BANKS; bank++) {
		resolve_bank(bank, after[bank]);
	}

	for (bank = 0; bank < BANKS; bank++) {
		for (level = 1; level <= LEVELS; level++) {
			CHECK(resolved_equal(&before[bank][level], &after[bank][level]),
				"B2: every level behaves after the restart as it did before it");
		}
	}
}

static void b3_supported_mode_numbers(void)
{
	uint8_t blob[ASSIST_BANK_BLOB_LEN];
	uint8_t mode;

	printf("B3 every mode number the firmware claims to support is accepted\n");

	/* 0..12 is the whole supported space: 0 reserved, 1..6 legacy and migrated, 7..12 V2. */
	for (mode = 0U; mode <= 12U; mode++) {
		assist_modes_init();
		assist_modes_serialize_bank(0, blob);
		record_of(blob, 3)[0] = mode;
		wr16(&blob[ASSIST_BANK_BLOB_LEN - 2], crc16(blob, ASSIST_BANK_BLOB_LEN - 2));
		CHECK(assist_modes_apply_bank_blob(blob, ASSIST_BANK_BLOB_LEN),
			"B3: a supported stored mode number is accepted");
		assist_modes_set_active_bank(0);
		CHECK(assist_modes_get_default_level(3U)->mode_type == (assist_mode_type_t)mode,
			"B3: the stored number is kept, not rewritten");
	}

	/* Legacy numbers migrate by character, and the migration never invents assist for the
	 * reserved 0 - that level is off, which is a different thing from a rejected bank. */
	{
		assist_level_config_t cfg;

		memset(&cfg, 0, sizeof(cfg));
		cfg.max_iq_pct = 100U;
		cfg.mode_type = ASSIST_MODE_EMTB_CUSTOM;
		CHECK(assist_modes_profile_for_level(&cfg) == AP2_PROFILE_SPORT,
			"B3: mode 4 maps to SPORT - the validator and the migration agree about it");
		cfg.mode_type = ASSIST_MODE_RESERVED_0;
		CHECK(assist_modes_level_disables_assist(&cfg),
			"B3: the reserved mode 0 is a switched-off level");
	}

	for (mode = 13U; mode < 16U; mode++) {
		assist_modes_init();
		assist_modes_serialize_bank(0, blob);
		record_of(blob, 3)[0] = mode;
		wr16(&blob[ASSIST_BANK_BLOB_LEN - 2], crc16(blob, ASSIST_BANK_BLOB_LEN - 2));
		CHECK(!assist_modes_apply_bank_blob(blob, ASSIST_BANK_BLOB_LEN),
			"B3: an unknown mode number is refused");
	}
	assist_modes_init();
	assist_modes_serialize_bank(0, blob);
	record_of(blob, 3)[0] = 255U;
	wr16(&blob[ASSIST_BANK_BLOB_LEN - 2], crc16(blob, ASSIST_BANK_BLOB_LEN - 2));
	CHECK(!assist_modes_apply_bank_blob(blob, ASSIST_BANK_BLOB_LEN),
		"B3: 255 is refused as well");
}

static void b4_rejection_mutates_nothing(void)
{
	uint8_t good[ASSIST_BANK_BLOB_LEN];
	uint8_t bad[ASSIST_BANK_BLOB_LEN];
	uint8_t readback[ASSIST_BANK_BLOB_LEN];

	printf("B4 a refused bank leaves the live configuration untouched\n");

	assist_modes_init();
	assist_modes_serialize_bank(0, good);
	record_of(good, 1)[17] = 55U;                 /* something recognisable to come back to */
	wr16(&record_of(good, 1)[15], 333U);
	wr16(&good[ASSIST_BANK_BLOB_LEN - 2], crc16(good, ASSIST_BANK_BLOB_LEN - 2));
	CHECK(assist_modes_apply_bank_blob(good, ASSIST_BANK_BLOB_LEN), "B4: the good bank applies");

	/* The bad record is the LAST one, so a parser that validated as it wrote would already
	 * have changed the four levels ahead of it by the time it noticed. */
	memcpy(bad, good, sizeof(bad));
	record_of(bad, 5)[0] = 200U;
	record_of(bad, 1)[17] = 12U;
	wr16(&bad[ASSIST_BANK_BLOB_LEN - 2], crc16(bad, ASSIST_BANK_BLOB_LEN - 2));
	CHECK(!assist_modes_apply_bank_blob(bad, ASSIST_BANK_BLOB_LEN), "B4: the bad bank is refused");

	assist_modes_serialize_bank(0, readback);
	CHECK(memcmp(good, readback, ASSIST_BANK_BLOB_LEN) == 0,
		"B4: nothing from the refused bank reached the live configuration");

	/* A corrupt CRC is refused the same way, and equally without partial mutation. */
	memcpy(bad, good, sizeof(bad));
	record_of(bad, 1)[17] = 12U;
	CHECK(!assist_modes_apply_bank_blob(bad, ASSIST_BANK_BLOB_LEN),
		"B4: a bank whose CRC does not match is refused");
	assist_modes_serialize_bank(0, readback);
	CHECK(memcmp(good, readback, ASSIST_BANK_BLOB_LEN) == 0,
		"B4: a CRC failure mutates nothing either");
}

static void b5_zero_ramp_means_profile(void)
{
	uint8_t blob[ASSIST_BANK_BLOB_LEN];
	uint8_t again[ASSIST_BANK_BLOB_LEN];
	resolved_t shipped;
	resolved_t after;
	uint8_t level;

	printf("B5 zero in a ramp field stays zero - the profile decides\n");

	assist_modes_init();
	assist_modes_set_active_bank(0);
	resolve_level(2U, &shipped);

	assist_modes_serialize_bank(0, blob);
	for (level = 1; level <= LEVELS; level++) {
		CHECK(rd16(&record_of(blob, level)[40]) == 0U,
			"B5: the shipped bank really does store 0 for the attack time");
	}
	CHECK(assist_modes_apply_bank_blob(blob, ASSIST_BANK_BLOB_LEN), "B5: it applies");
	assist_modes_serialize_bank(0, again);
	for (level = 1; level <= LEVELS; level++) {
		/*
		 * THE FINDING. The parser clamped every ramp to the 20 ms floor, so a plain read ->
		 * save with no edit at all turned "the profile decides" into "20 ms" on all five
		 * levels - a real change of behaviour produced by pressing Save.
		 */
		CHECK(rd16(&record_of(again, level)[40]) == 0U,
			"B5: a stored 0 comes back as 0, not as the 20 ms floor");
	}
	resolve_level(2U, &after);
	CHECK(after.attack_ms == shipped.attack_ms && after.attack_ms > 20U,
		"B5: the level still attacks at its profile's rate after the round trip");

	/* A non-zero value is still a rider's number and is still held to the floor. */
	wr16(&record_of(blob, 2)[40], 5U);
	wr16(&record_of(blob, 3)[40], 900U);
	wr16(&blob[ASSIST_BANK_BLOB_LEN - 2], crc16(blob, ASSIST_BANK_BLOB_LEN - 2));
	CHECK(assist_modes_apply_bank_blob(blob, ASSIST_BANK_BLOB_LEN), "B5: it applies");
	assist_modes_serialize_bank(0, again);
	CHECK(rd16(&record_of(again, 2)[40]) == 20U,
		"B5: a non-zero ramp below the floor is still raised to it");
	CHECK(rd16(&record_of(again, 3)[40]) == 900U, "B5: an ordinary ramp is kept exactly");
	resolve_level(3U, &after);
	CHECK(after.attack_ms == 900U, "B5: an explicit override reaches the resolved settings");
}

static void b6_zero_torque_ceiling_is_off(void)
{
	uint8_t blob[ASSIST_BANK_BLOB_LEN];
	uint8_t flash[ASSIST_BANK_BLOB_LEN];
	assist_level_config_t cfg;
	resolved_t r;

	printf("B6 zero in the torque ceiling switches the level off, everywhere\n");

	assist_modes_init();
	assist_modes_set_active_bank(0);
	cfg = *assist_modes_get_default_level(3U);

	cfg.max_iq_pct = 100U;
	CHECK(assist_modes_level_iq_limit(&cfg, (int32_t)PH_CURRENT_MAX,
		(int32_t)PH_CURRENT_MAX) == (int32_t)PH_CURRENT_MAX,
		"B6: 100 % is the phase-current limit itself");
	CHECK(!assist_modes_level_disables_assist(&cfg), "B6: 100 % is not off");

	cfg.max_iq_pct = 50U;
	CHECK(assist_modes_level_iq_limit(&cfg, (int32_t)PH_CURRENT_MAX,
		(int32_t)PH_CURRENT_MAX) == (int32_t)PH_CURRENT_MAX / 2, "B6: 50 % halves it");

	cfg.max_iq_pct = 1U;
	CHECK(assist_modes_level_iq_limit(&cfg, (int32_t)PH_CURRENT_MAX,
		(int32_t)PH_CURRENT_MAX) == (int32_t)PH_CURRENT_MAX / 100,
		"B6: 1 % is a hundredth - a very weak level, not a disabled one");
	CHECK(!assist_modes_level_disables_assist(&cfg), "B6: 1 % is not off");

	/*
	 * THE FINDING. The app has always said "Assist is switched off at this level" for zero,
	 * while the firmware read the same byte as "no extra limit" and handed back the full
	 * ceiling. One byte, two meanings, at the two ends of one wire.
	 */
	cfg.max_iq_pct = 0U;
	CHECK(assist_modes_level_iq_limit(&cfg, (int32_t)PH_CURRENT_MAX,
		(int32_t)PH_CURRENT_MAX) == 0, "B6: 0 % allows no current at all");
	CHECK(assist_modes_level_disables_assist(&cfg),
		"B6: 0 % reports the level as off, so the request is zero at its source");

	/* And it survives a save and a restart, which is where a setting is normally lost. */
	assist_modes_init();
	assist_modes_serialize_bank(0, blob);
	record_of(blob, 3)[17] = 0U;
	wr16(&blob[ASSIST_BANK_BLOB_LEN - 2], crc16(blob, ASSIST_BANK_BLOB_LEN - 2));
	CHECK(assist_modes_apply_bank_blob(blob, ASSIST_BANK_BLOB_LEN),
		"B6: a bank with a switched-off level is accepted");
	assist_modes_serialize_bank(0, flash);
	assist_modes_init();
	CHECK(assist_modes_apply_bank_blob(flash, ASSIST_BANK_BLOB_LEN), "B6: and restores");
	assist_modes_set_active_bank(0);
	resolve_level(3U, &r);
	CHECK(r.iq_ceiling == 0 && r.disabled,
		"B6: the level is still switched off after the restart");
	resolve_level(2U, &r);
	CHECK(r.iq_ceiling == (int32_t)PH_CURRENT_MAX && !r.disabled,
		"B6: switching one level off does not touch its neighbours");

	/* Level 0 is not a rider-configurable level and must not be dragged into this: it is off
	 * because of its INDEX, and the throttle is not an assist level's to switch off. */
	CHECK(!assist_modes_level_disables_assist(assist_modes_get_default_level(0U)),
		"B6: the idle level does not report itself as a switched-off assist level");
}

static void b7_older_banks_still_load(void)
{
	uint8_t v8[ASSIST_BANK_BLOB_LEN];
	uint8_t older[ASSIST_BANK_BLOB_LEN];
	uint16_t crc_at;
	uint8_t level;
	uint8_t stride;
	uint8_t version;

	printf("B7 banks written by older firmware still load\n");

	/* v8 is the layout this firmware still writes; v9 only renames it - see the version
	 * comment in src/assist_modes.c. A rider's v5/v6/v7 bank has to keep loading too. */
	for (version = 5U; version <= 8U; version++) {
		stride = (version >= 8U) ? RECORD_LEN_V8 :
			((version >= 6U) ? RECORD_LEN_V7 : RECORD_LEN_V5);

		assist_modes_init();
		assist_modes_serialize_bank(0, v8);
		/* A level saved under an old mode number, which is what an old bank would carry. */
		record_of(v8, 1)[0] = 2U;          /* Power Progressive */
		record_of(v8, 2)[0] = 5U;          /* Torque */

		memset(older, 0, sizeof(older));
		memcpy(older, v8, HEADER_LEN);
		older[2] = version;
		older[5] = stride;
		for (level = 1; level <= LEVELS; level++) {
			memcpy(&older[HEADER_LEN + (level - 1) * stride],
				record_of(v8, level), stride);
		}
		crc_at = (uint16_t)(HEADER_LEN + LEVELS * stride);
		wr16(&older[crc_at], crc16(older, crc_at));

		CHECK(assist_modes_apply_bank_blob(older, (uint16_t)(crc_at + 2U)),
			"B7: a shorter, older bank record is accepted");
		assist_modes_set_active_bank(0);
		CHECK(assist_modes_get_default_level(1U)->mode_type == ASSIST_MODE_POWER_PROGRESSIVE &&
			assist_modes_get_default_level(2U)->mode_type == ASSIST_MODE_TORQUE,
			"B7: the old mode numbers are preserved, not rewritten");
		CHECK(assist_modes_profile_for_level(assist_modes_get_default_level(1U)) ==
			AP2_PROFILE_SPORT,
			"B7: and they migrate to a V2 profile for control");
		CHECK(assist_modes_get_default_level(1U)->iq_rise_fast_ms == 0U,
			"B7: a record too short to carry the ramps falls back to 'the profile decides'");
	}

	/* A version this firmware has never heard of is still refused. */
	assist_modes_init();
	assist_modes_serialize_bank(0, older);
	older[2] = 99U;
	wr16(&older[ASSIST_BANK_BLOB_LEN - 2], crc16(older, ASSIST_BANK_BLOB_LEN - 2));
	CHECK(!assist_modes_apply_bank_blob(older, ASSIST_BANK_BLOB_LEN),
		"B7: an unknown blob version is refused");
}

int main(void)
{
	printf("Assist bank configuration contract\n");
	printf("==================================\n");

	b1_defaults_round_trip();
	b2_restart_restores_behaviour();
	b3_supported_mode_numbers();
	b4_rejection_mutates_nothing();
	b5_zero_ramp_means_profile();
	b6_zero_torque_ceiling_is_off();
	b7_older_banks_still_load();

	if (failures != 0) {
		printf("\nFAILURES: %d\n", failures);
		return 1;
	}
	printf("\nAll assist bank configuration contract checks passed.\n");
	return 0;
}
