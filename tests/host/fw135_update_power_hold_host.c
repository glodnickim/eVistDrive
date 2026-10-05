/*
 * FW-135: a display firmware update must never be able to cut its own supply.
 *
 * The controller powers itself off after COMM_OFF_TICKS of silence at standstill, and that
 * power-off drops the display rail too. Until this card the silence was measured by ONE counter
 * that only frames addressed to node 2 could reset - so a DISPLAY update, where the updater talks
 * to node 3 and the display itself sits mute in its bootloader, looked exactly like a pulled
 * cable. Ten seconds in, the controller cut power in the middle of a flash write.
 *
 * That was not new code. It was old code that the pre-FW-132 NVIC_SystemReset() on every 0x3005
 * had been hiding: the reset cleared the arm flag and restarted the boot grace period before the
 * counter could ever reach its threshold.
 *
 * main.c and CAN_Display.c are the ARM entry point and the CAN receive path and cannot be linked
 * on a host (same reasoning as armed_zero_lifecycle_host.c and stopclick_c1_pi_integral_host.c).
 * The invariants below are therefore proven against the production source text.
 *
 *   T1  BUS COUNTER IS OUTSIDE THE ADDRESS GATE   any frame keeps us powered, whoever it is for.
 *   T2  ASSIST COUNTER IS SOURCE-QUALIFIED        only the display can vouch for the display.
 *   T3  POWER-OFF IS GUARDED                      the silence power-off needs the BUS counter, a
 *                                                 standstill, and no update session in progress.
 *   T4  THE HOLD IS ARMED BY 0x3005               and by every 0x3005, broadcast included.
 *   T5  THE BIKE CAN ALWAYS BE SWITCHED OFF       the hold gates the SILENCE path only - never
 *                                                 the on/off button, never the inactivity timer.
 *   T6  THE FAIL-SAFE SURVIVES                    a dead bus still cuts assist, whether or not
 *                                                 the display ever identified itself.
 *   T7  THE HOLD OUTLASTS WHAT IT SUSPENDS        a hold shorter than the silence threshold
 *                                                 would suspend nothing.
 */

#include "../common/check.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STRINGIZE2(x) #x
#define STRINGIZE(x) STRINGIZE2(x)

#ifndef MAIN_C_PATH
#error "MAIN_C_PATH must be supplied by run-host-tests.ps1"
#endif
#ifndef CAN_DISPLAY_C_PATH
#error "CAN_DISPLAY_C_PATH must be supplied by run-host-tests.ps1"
#endif
#ifndef CONFIG_H_PATH
#error "CONFIG_H_PATH must be supplied by run-host-tests.ps1"
#endif

static char *read_whole_file(const char *path, long *out_len)
{
	FILE *f = fopen(path, "rb");
	if (!f) return NULL;
	if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
	long len = ftell(f);
	if (len < 0 || fseek(f, 0, SEEK_SET) != 0) { fclose(f); return NULL; }
	char *text = (char *)malloc((size_t)len + 1U);
	if (!text) { fclose(f); return NULL; }
	size_t got = fread(text, 1, (size_t)len, f);
	fclose(f);
	text[got] = '\0';
	if (out_len) *out_len = (long)got;
	return text;
}

/* Comments are where the intent is explained, so they must not be able to satisfy a guard. */
static char *strip_comments(const char *text, long len)
{
	char *out = (char *)malloc((size_t)len + 1U);
	if (!out) return NULL;
	long i = 0, o = 0;
	while (i < len) {
		if (text[i] == '/' && i + 1 < len && text[i + 1] == '/') {
			while (i < len && text[i] != '\n') i++;
			continue;
		}
		if (text[i] == '/' && i + 1 < len && text[i + 1] == '*') {
			i += 2;
			while (i + 1 < len && !(text[i] == '*' && text[i + 1] == '/')) i++;
			i += 2;
			out[o++] = ' ';
			continue;
		}
		out[o++] = text[i++];
	}
	out[o] = '\0';
	return out;
}

/* Collapse every run of whitespace to one space, so a guard survives reindentation. */
static char *squeeze(const char *text)
{
	size_t len = strlen(text);
	char *out = (char *)malloc(len + 1U);
	if (!out) return NULL;
	size_t o = 0;
	bool in_space = false;
	for (size_t i = 0; i < len; i++) {
		unsigned char c = (unsigned char)text[i];
		if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
			if (!in_space && o > 0) out[o++] = ' ';
			in_space = true;
			continue;
		}
		out[o++] = (char)c;
		in_space = false;
	}
	while (o > 0 && out[o - 1] == ' ') o--;
	out[o] = '\0';
	return out;
}

static char *load(const char *path)
{
	long len = 0;
	char *raw = read_whole_file(path, &len);
	if (!raw) return NULL;
	char *stripped = strip_comments(raw, len);
	free(raw);
	if (!stripped) return NULL;
	char *flat = squeeze(stripped);
	free(stripped);
	return flat;
}

/* Value of a plain "#define NAME <integer>" read from the stripped, squeezed config text. */
static long define_value(const char *text, const char *name, bool *ok)
{
	char needle[128];
	snprintf(needle, sizeof needle, "#define %s ", name);
	const char *at = strstr(text, needle);
	if (!at) { *ok = false; return 0; }
	*ok = true;
	return strtol(at + strlen(needle), NULL, 10);
}

/* Does [from, to) contain needle? Used to prove a call is NOT gated by something. */
static bool span_contains(const char *from, const char *to, const char *needle)
{
	if (!from || !to || to < from) return false;
	size_t span = (size_t)(to - from);
	size_t nlen = strlen(needle);
	if (nlen > span) return false;
	for (size_t i = 0; i + nlen <= span; i++) {
		if (memcmp(from + i, needle, nlen) == 0) return true;
	}
	return false;
}

int main(void)
{
	printf("FW-135 update-session power hold - production wiring guards\n");

	char *mainc = load(STRINGIZE(MAIN_C_PATH));
	char *canc  = load(STRINGIZE(CAN_DISPLAY_C_PATH));
	char *cfg   = load(STRINGIZE(CONFIG_H_PATH));
	CHECK(mainc && canc && cfg, "sources readable");
	if (!mainc || !canc || !cfg) {
		printf("\n1 FW-135 update-session power hold check FAILED (sources unreadable).\n");
		return 1;
	}

	/* ==== T1: the bus counter is reset before, and therefore outside, the address gate ==== */
	{
		const char *bus_reset = strstr(canc, "bus_lost_ticks=0; bus_seen=1;");
		const char *gate = strstr(canc, "if(Ext_ID_Rx.target==2){");
		CHECK(bus_reset != NULL, "T1: any received frame resets the bus counter");
		CHECK(gate != NULL, "T1: the target==2 gate still exists for everything else");
		CHECK(bus_reset != NULL && gate != NULL && bus_reset < gate,
			"T1: the bus reset is OUTSIDE the target==2 gate - a display update is traffic too");
	}

	/* ==== T2: only the display may vouch for the display ==== */
	{
		CHECK(strstr(canc, "if(Ext_ID_Rx.source==3){ hmi_lost_ticks=0; hmi_seen=1; }") != NULL,
			"T2: the assist counter is reset by SOURCE==3, not by anything addressed to node 2");
		CHECK(strstr(canc, "comm_lost_ticks") == NULL && strstr(mainc, "comm_lost_ticks") == NULL,
			"T2: the old single counter is gone - a Canable can no longer impersonate the display");
	}

	/* ==== T3: the silence power-off is guarded on all three conditions ==== */
	{
		CHECK(strstr(mainc,
			"if(bus_seen && bus_lost_ticks >= COMM_OFF_TICKS && MS.Speedx100==0 && update_hold_ticks==0){")
			!= NULL,
			"T3: silence power-off needs the BUS counter, a standstill AND no update in progress");
		CHECK(strstr(mainc, "if(update_hold_ticks > 0) update_hold_ticks--;") != NULL,
			"T3: the hold expires on its own, so it can never latch the bike on forever");
	}

	/* ==== T4: every 0x3005 arms the hold, broadcast included ==== */
	{
		const char *at = strstr(canc, "if(Ext_ID_Rx.command==0x3005){");
		CHECK(at != NULL, "T4: the 0x3005 handler exists");
		const char *reset = at ? strstr(at, "NVIC_SystemReset();") : NULL;
		CHECK(reset != NULL, "T4: an addressed 0x3005 still enters the bootloader");
		CHECK(span_contains(at, reset, "update_hold_ticks=UPDATE_HOLD_TICKS;"),
			"T4: the hold is armed BEFORE the broadcast test, so a display update arms it too");
	}

	/* ==== T5: the hold gates the silence path only ==== */
	{
		const char *button = strstr(mainc, "if(pa4_buttons_power_off_due()){"); /* DISC-010 */
		CHECK(button != NULL, "T5: the on/off button power-off exists");
		const char *button_off = button ? strstr(button, "power_off_controller();") : NULL;
		CHECK(button_off != NULL, "T5: and still calls power_off_controller()");
		CHECK(button_off != NULL && !span_contains(button, button_off, "update_hold_ticks"),
			"T5: the on/off button is NOT gated by the update hold - the bike always switches off");

		const char *idle = strstr(mainc, "if(auto_off_minutes>0 &&");
		CHECK(idle != NULL, "T5: the inactivity auto-off exists");
		const char *idle_off = idle ? strstr(idle, "power_off_controller();") : NULL;
		CHECK(idle_off != NULL, "T5: and still calls power_off_controller()");
		CHECK(idle_off != NULL && !span_contains(idle, idle_off, "update_hold_ticks"),
			"T5: the inactivity timer is NOT gated by the update hold either");
	}

	/* ==== T6: a dead bus still stops the motor ====
	 * UPDATED (M820 uncontrolled-Iq/CAN safety fix): the old expectation - the watchdog writes
	 * MS.assist_level=0 and calls ride_control_force_final_iq_zero() - was unsafe/wrong on two
	 * counts proven by replay: the forced zero was overwritten by the next ride_control_update(),
	 * and the level never came back because the 0x6300 debounce stayed saturated on the same
	 * code. The cut is now a separate comm_inhibit that the single final-Iq arbitration owns
	 * (behaviour: m820_uncontrolled_iq_safety_host.c, tests/test_m820_walk_can_safety.py). */
	{
		const char *cut = strstr(mainc,
			"comm_inhibit = ((hmi_seen && hmi_lost_ticks >= COMM_CUT_TICKS) || (bus_seen && bus_lost_ticks >= COMM_CUT_TICKS)) ? 1U : 0U;");
		CHECK(cut != NULL,
			"T6: assist is cut by EITHER counter - a pulled cable stops the motor even if the "
			"display never identified itself as source 3");
		CHECK(strstr(mainc, "MS.assist_level=0;") == NULL,
			"T6: the comms watchdog no longer destroys the rider's selected level");
		CHECK(strstr(mainc, ".comm_inhibit = comm_inhibit != 0U,") != NULL,
			"T6: and the inhibit reaches the one final-Iq arbitration in ride_control_update()");
	}

	/* ==== T7: a hold shorter than the threshold it suspends would suspend nothing ==== */
	{
		bool ok_hold = false, ok_off = false;
		long hold = define_value(cfg, "UPDATE_HOLD_TICKS", &ok_hold);
		long off  = define_value(cfg, "COMM_OFF_TICKS", &ok_off);
		CHECK(ok_hold && ok_off, "T7: both constants are plain integer defines in config.h");
		printf("  T7 hold %ld ticks (%.0f s) vs silence threshold %ld ticks (%.0f s)\n",
			hold, (double)hold * 0.04, off, (double)off * 0.04);
		CHECK(hold > off,
			"T7: the hold must outlast the silence threshold it suspends");
		CHECK(hold >= 10 * off,
			"T7: and by a wide margin - a flash erase is long compared with 10 s of quiet");
	}

	free(mainc); free(canc); free(cfg);

	if (host_test_failures == 0) {
		printf("All FW-135 update-session power hold checks passed.\n");
		return 0;
	}
	printf("\n%d FW-135 update-session power hold check(s) FAILED.\n", host_test_failures);
	return 1;
}
