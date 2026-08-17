/*
 * Shared assertion helper for host harnesses, in the same style every existing
 * tests/host/fw*_host.c file already uses (see fw102_pas_trace_host.c): a local failure
 * counter and a check() that prints a FAIL line and keeps going, so one bad assertion
 * does not hide the rest. `static` at file scope: each .c file that includes this header
 * gets its own private copy (internal linkage), so multiple harness .c files can each
 * include it without a multiple-definition link error.
 */
#ifndef HOST_TEST_CHECK_H
#define HOST_TEST_CHECK_H

#include <stdio.h>

static int host_test_failures;

static void host_test_check(int ok, const char *label)
{
	if (!ok) {
		host_test_failures++;
		printf("  FAIL  %s\n", label);
	}
}

#define CHECK(ok, label) host_test_check((ok), (label))

#endif /* HOST_TEST_CHECK_H */
