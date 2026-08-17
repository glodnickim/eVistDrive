/*
 * Tiny CSV-open helper shared by the trace-writing harnesses. Deliberately not a whole
 * CSV library: each harness owns its own column list (they differ — see
 * documentation/testing/TEST_INTERFACES.md, "Trace format") and writes rows with plain
 * fprintf, because the column sets are fixed per harness and known at compile time.
 */
#ifndef HOST_TEST_CSV_H
#define HOST_TEST_CSV_H

#include <stdio.h>
#include <stdlib.h>

static FILE *csv_open_or_die(const char *path, const char *header_line)
{
	FILE *f = fopen(path, "w");
	if (f == NULL) {
		fprintf(stderr, "could not open trace file for writing: %s\n", path);
		exit(1);
	}
	fprintf(f, "%s\n", header_line);
	return f;
}

#endif /* HOST_TEST_CSV_H */
