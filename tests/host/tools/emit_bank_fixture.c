/*
 * Writes the bank blobs the PRODUCTION serializer produces, for the configurator's tests to
 * read. Not a test itself: it is the one place a fixture used by the other repository can come
 * from, so that fixture is real firmware output and not a hand-typed table that agrees with
 * whatever the app happens to do.
 *
 *   gcc -I inc tests/host/tools/emit_bank_fixture.c src/assist_modes.c src/torque_input.c \
 *       src/tuning_config.c -o emit_bank_fixture
 *   emit_bank_fixture <out-dir>
 *
 * Regenerate whenever the bank wire format changes - and when you do, the configurator's
 * round-trip test is the thing that tells you whether it still speaks the same format.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "assist_modes.h"

static int write_bank(const char *dir, uint8_t bank)
{
	uint8_t blob[ASSIST_BANK_BLOB_LEN];
	char path[512];
	FILE *f;
	uint16_t len;

	len = assist_modes_serialize_bank(bank, blob);
	if (len != ASSIST_BANK_BLOB_LEN) {
		fprintf(stderr, "bank %u serialized %u bytes, expected %u\n",
			(unsigned)bank, (unsigned)len, (unsigned)ASSIST_BANK_BLOB_LEN);
		return 1;
	}
	snprintf(path, sizeof(path), "%s/bank_defaults_%u.bin", dir, (unsigned)bank);
	f = fopen(path, "wb");
	if (f == NULL) {
		fprintf(stderr, "cannot write %s\n", path);
		return 1;
	}
	if (fwrite(blob, 1, len, f) != len) {
		fprintf(stderr, "short write to %s\n", path);
		fclose(f);
		return 1;
	}
	fclose(f);
	printf("wrote %s (%u B, blob version %u)\n", path, (unsigned)len, (unsigned)blob[2]);
	return 0;
}

int main(int argc, char **argv)
{
	const char *dir = (argc > 1) ? argv[1] : ".";

	assist_modes_init();
	if (write_bank(dir, 0) != 0) {
		return 1;
	}
	return write_bank(dir, 1);
}
