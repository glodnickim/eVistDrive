/*
 * FW-110 v4 guard: SOURCE-TEXT checks on src/main.c and src/CAN_Display.c, the same technique
 * tests/host/main_startup_wiring_host.c uses for pas_direction_init() - neither file can be
 * linked here (both are wired directly to GD32 CMSIS registers and real hardware peripherals
 * throughout), so a regression in what this card removed has to be caught structurally instead.
 *
 * WHAT THIS PROVES, exactly:
 *   1. src/CAN_Display.c contains ZERO active occurrences of the blocking wait pattern
 *      ("while((CAN_TRANSMIT_OK != can_transmit_states(") - every one of the 16 call sites this
 *      card converted stays converted to the non-blocking can_tx_queue.
 *   2. src/main.c contains EXACTLY ONE active occurrence of that same pattern, and it falls
 *      strictly inside autodetect() (the one deliberately blocking, service-only Hall
 *      calibration routine, kept from CAN by this card - see its own header comment). Zero would
 *      hide a silent behaviour change there; a second occurrence anywhere else in main.c would
 *      be exactly the kind of new blocking call this card exists to prevent from ever being
 *      reintroduced on a riding path; one outside autodetect()'s bounds is just as much a
 *      regression as a second one, even though the count alone would look fine.
 *   3. src/main.c declares `missed_control_events` as uint32_t, not uint16_t - the FW-110 fix for
 *      the saturation bug that made a session which lost 12,543 ticks publish 0 missed events
 *      (see documentation/FW-106_PAS_DIAGNOSTICS_RECORDER_BUGS_PL.md and the FW-110 report for the
 *      confirmed log evidence). Checked as "the uint32_t spelling exists and the uint16_t spelling
 *      does not", not just "uint16_t is absent", so a rename that dropped the fix a different way
 *      cannot slip past an incomplete check.
 *   4. src/main.c services can_tx_queue_service( BEFORE diag_dump_step( on every main-loop tick.
 *      This call ORDER is a real fact and worth keeping true, but on its own it is NOT proof
 *      that diagnostics actually yields a mailbox to critical frames - the real proof is
 *      behavioural: tests/host/fw106_session_host.c's checks 32-35 drive the REAL
 *      diag_session_dump_step(now, allow_new_tx) and show a new diagnostic transmit attempt
 *      genuinely never happens while allow_new_tx is false, across many ticks and pacing
 *      intervals, and that the abort-on-ride-resume path is never gated by it. This check is
 *      the cheap, fast structural half of that guarantee, not the whole of it.
 *   5. FW-110 v4: src/CAN_Display.c contains ZERO active `autodetect();` calls AND ZERO active
 *      `hall_calibration_request(` calls. The v3 supervisor (src/hall_calibration.c) is REMOVED;
 *      Hall/position calibration is DISABLED in both firmware variants. 0x6200's WRITE handler
 *      (operation and source already checked there) answers with EXACTLY ONE ERROR_ACK
 *      (sendWriteResult(0x6200, 0) - operation 3) and nothing else: no NORMAL_ACK, no code path
 *      to autodetect(). A READ 0x6200 does nothing at all - there is deliberately NO `case
 *      0x6200` anywhere in sendCAN_Tx(). This card is what makes "autodetect() is unreachable
 *      from CAN" true structurally.
 *   6. src/CAN_Display.c's sendCAN_3100() (the 0x81F83100 torque-sensor emulation stream)
 *      contains NO call to can_tx_queue_enqueue( anywhere in its body - it must never compete
 *      with critical HMI/ACK/multiframe frames for can_tx_queue's 16 reserved slots. Its one
 *      can_message_transmit( call - confirmed to be the ONLY one left in this file, everything
 *      else having moved to can_tx_queue in this same card - is its own separate, best-effort,
 *      single-attempt path.
 *   7. FW-110 v4: src/main.c contains ZERO active `autodetect();` CALLS, and wires the REAL
 *      completed-reply side-effect module instead of the removed calibration supervisor:
 *      can_reply_effects_init_wrapper() is called EXACTLY once at startup, and the (exactly one)
 *      `diag_peak_reset = 1;` statement sits inside main.c's own `if (fx ==
 *      CANFX_EFFECT_DIAG_PEAK_RESET)` block right after can_reply_effects_poll() - NOT in
 *      CAN_Display.c. This is the structural half of the 0x6029 gating: the behavioural proof
 *      that DONE fires the reset exactly once and ABORTED never does lives in
 *      tests/host/can_reply_effects_host.c, which drives the REAL can_reply_effects.c against
 *      the REAL can_multiframe.c + can_tx_queue.c.
 *   8. autodetect()'s OWN definition begins with `if(!hall_calibration_standstill_confirmed())
 *      return;` as its literal FIRST statement (only whitespace between the opening brace and
 *      it) - the defense-in-depth layer that still holds even if a future card re-enables a call
 *      site above it.
 *   9. 0x6012's trailing marker (the factory "config transfer complete" marker) is armed via
 *      send_multiframe_trailer() - the producer's OWN trailer phase, part of the same
 *      stop-and-wait transfer - never via a direct can_tx_queue_enqueue() (which used to race
 *      the reply's START) and never via the removed can_multiframe_set_trailer().
 *
 * Comments are stripped first (line and block, string/char literals left alone), same
 * implementation as main_startup_wiring_host.c, so a commented-out blocking wait or a stale
 * comment mentioning the pattern can never be counted as if it were active code.
 *
 * WHAT THIS DOES NOT PROVE: that can_tx_queue.c/can_multiframe.c/can_reply_effects.c/diag_session.c
 * themselves behave correctly (their own host tests link and drive the real modules for that), or
 * that main()/CAN_Display.c actually run correctly on real hardware - only that the source text
 * has not silently regressed back to a pattern this card confirmed was a real, reachable bug.
 */

#include "../common/check.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STRINGIZE2(x) #x
#define STRINGIZE(x) STRINGIZE2(x)

#ifndef MAIN_C_PATH
#error "MAIN_C_PATH must be defined (by the build script) to the path of src/main.c"
#endif
#ifndef CAN_DISPLAY_C_PATH
#error "CAN_DISPLAY_C_PATH must be defined (by the build script) to the path of src/CAN_Display.c"
#endif

static char *read_whole_file(const char *path, long *out_len)
{
	FILE *f = fopen(path, "rb");
	if (!f) return NULL;
	if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
	long len = ftell(f);
	if (len < 0) { fclose(f); return NULL; }
	if (fseek(f, 0, SEEK_SET) != 0) { fclose(f); return NULL; }
	char *buf = (char *)malloc((size_t)len + 1);
	if (!buf) { fclose(f); return NULL; }
	size_t got = fread(buf, 1, (size_t)len, f);
	fclose(f);
	buf[got] = '\0';
	if (out_len) *out_len = (long)got;
	return buf;
}

/* Same sanitizer as main_startup_wiring_host.c: same-length output, comments blanked, string/char
 * literals copied verbatim so a comment-looking sequence inside one is never mistaken for real. */
static char *strip_comments(const char *text, long len)
{
	char *out = (char *)malloc((size_t)len + 1);
	if (!out) return NULL;
	long i = 0;
	while (i < len) {
		if (text[i] == '/' && i + 1 < len && text[i + 1] == '/') {
			while (i < len && text[i] != '\n') { out[i] = ' '; i++; }
			continue;
		}
		if (text[i] == '/' && i + 1 < len && text[i + 1] == '*') {
			out[i] = ' '; out[i + 1] = ' '; i += 2;
			while (i < len && !(text[i] == '*' && i + 1 < len && text[i + 1] == '/')) {
				out[i] = (text[i] == '\n') ? '\n' : ' ';
				i++;
			}
			if (i < len) { out[i] = ' '; i++; }   /* the star of the block comment's own close */
			if (i < len) { out[i] = ' '; i++; }   /* the slash right after it */
			continue;
		}
		if (text[i] == '"' || text[i] == '\'') {
			char quote = text[i];
			out[i] = text[i]; i++;
			while (i < len && text[i] != quote) {
				out[i] = text[i];
				if (text[i] == '\\' && i + 1 < len) { i++; out[i] = text[i]; }
				i++;
			}
			if (i < len) { out[i] = text[i]; i++; }   /* the closing quote */
			continue;
		}
		out[i] = text[i];
		i++;
	}
	out[len] = '\0';
	return out;
}

static int count_active(const char *from, const char *before, const char *needle,
                         const char **out_first)
{
	int n = 0;
	const char *p = from;
	if (out_first) *out_first = NULL;
	while (p) {
		const char *hit = strstr(p, needle);
		if (!hit) break;
		if (before && hit >= before) break;
		n++;
		if (out_first && !*out_first) *out_first = hit;
		p = hit + 1;
	}
	return n;
}

static const char *BLOCKING_WAIT = "while((CAN_TRANSMIT_OK != can_transmit_states(";

int main(void)
{
	printf("FW-110 can-blocking guard (source-text check, see file header)\n");

	/* --- CAN_Display.c: zero blocking waits --------------------------------------------- */
	{
		const char *path = STRINGIZE(CAN_DISPLAY_C_PATH);
		printf("  CAN_DISPLAY_C_PATH = %s\n", path);
		long len = 0;
		char *raw = read_whole_file(path, &len);
		CHECK(raw != NULL, "setup: src/CAN_Display.c was readable at CAN_DISPLAY_C_PATH");
		if (raw) {
			char *clean = strip_comments(raw, len);
			CHECK(clean != NULL, "setup: CAN_Display.c comment/string-aware sanitization succeeded");
			free(raw);
			if (clean) {
				int n = count_active(clean, NULL, BLOCKING_WAIT, NULL);
				CHECK(n == 0,
					"GUARD: src/CAN_Display.c contains ZERO active blocking "
					"while((CAN_TRANSMIT_OK != can_transmit_states( waits - every call site "
					"this card converted must stay converted to can_tx_queue_enqueue");

				/* --- FW-110 v4: zero autodetect() calls AND zero hall_calibration_request() -- */
				int autodetect_calls = count_active(clean, NULL, "autodetect();", NULL);
				CHECK(autodetect_calls == 0,
					"GUARD: src/CAN_Display.c contains ZERO active autodetect() calls - "
					"processCAN_Rx() runs at any time, including while riding, and must never "
					"run the >5 s open-loop procedure");
				int calib_request_calls = count_active(clean, NULL, "hall_calibration_request(", NULL);
				CHECK(calib_request_calls == 0,
					"GUARD: src/CAN_Display.c contains ZERO active hall_calibration_request( "
					"calls - the FW-110 v3 supervisor is REMOVED, calibration is disabled, and "
					"0x6200 must answer with the single ERROR_ACK checked below, nothing more");

				/* --- 0x6200: exactly one ERROR_ACK, no NORMAL_ACK, no READ case ------------- */
				int err_ack = count_active(clean, NULL, "sendWriteResult(0x6200, 0);", NULL);
				CHECK(err_ack == 1,
					"GUARD: src/CAN_Display.c sends EXACTLY ONE `sendWriteResult(0x6200, 0);` "
					"(operation 3 = ERROR_ACK) for WRITE 0x6200 - one reply, never a NORMAL_ACK, "
					"never a second reply");
				int read_case = count_active(clean, NULL, "case 0x6200:", NULL);
				CHECK(read_case == 0,
					"GUARD: src/CAN_Display.c contains NO `case 0x6200:` - a READ 0x6200 must do "
					"nothing at all, so there is no second, operation-blind route to anything");

				/* --- sendCAN_3100() must never compete for can_tx_queue's reserved slots ----- */
				const char *mf3100_start = strstr(clean, "void sendCAN_3100(MotorState_t* MS){");
				const char *mf3100_end = mf3100_start
					? strstr(mf3100_start, "void sendCAN_3202(void){") : NULL;
				CHECK(mf3100_start != NULL, "setup: sendCAN_3100(MotorState_t* MS){ found in CAN_Display.c");
				CHECK(mf3100_end != NULL, "setup: sendCAN_3202(void){ found after it, to bound its body");
				if (mf3100_start && mf3100_end) {
					const char *enq = strstr(mf3100_start, "can_tx_queue_enqueue(");
					CHECK(enq == NULL || enq >= mf3100_end,
						"GUARD: sendCAN_3100() contains NO call to can_tx_queue_enqueue( - the "
						"0x81F83100 stream (measured ~92 frames/s when enabled) must never "
						"compete with critical HMI/ACK/multiframe frames for can_tx_queue's 16 "
						"reserved slots; it is its own separate, best-effort, single-attempt path");
				}

				/* can_message_transmit( is now used in exactly one place in this whole file -
				 * sendCAN_3100()'s own direct, single-attempt, best-effort transmit. */
				int direct_transmit_calls = count_active(clean, NULL, "can_message_transmit(", NULL);
				CHECK(direct_transmit_calls == 1,
					"GUARD: src/CAN_Display.c calls can_message_transmit( directly exactly once "
					"(sendCAN_3100's own best-effort path) - every other frame in this file goes "
					"through can_tx_queue_enqueue/can_multiframe_start instead");

				/* --- 0x6012's trailing marker must be the producer's OWN trailer phase, never -
				 * enqueued directly and never a removed set_trailer() - a direct enqueue used to
				 * race the multiframe reply's own START (send_multiframe() only ARMS the producer,
				 * it does not itself enqueue anything), so the marker could reach the wire before
				 * Para2's START did. FW-110 v4: the trailer is armed ATOMICALLY at start() time
				 * (can_multiframe_start_with_trailer, wrapped as send_multiframe_trailer). */
				const char *sendcantx_start = strstr(clean, "void sendCAN_Tx(MotorParams_t* MP, MotorState_t* MS){");
				CHECK(sendcantx_start != NULL, "setup: sendCAN_Tx(MotorParams_t* MP, MotorState_t* MS){ found");
				const char *mf6012_start = sendcantx_start ? strstr(sendcantx_start, "case 0x6012:") : NULL;
				const char *mf6012_end = mf6012_start ? strstr(mf6012_start + 1, "case 0x6017:") : NULL;
				CHECK(mf6012_start != NULL, "setup: case 0x6012: found inside sendCAN_Tx()");
				CHECK(mf6012_end != NULL, "setup: case 0x6017: found after it, to bound 0x6012's body");
				if (mf6012_start && mf6012_end) {
					const char *enq = strstr(mf6012_start, "can_tx_queue_enqueue(");
					CHECK(enq == NULL || enq >= mf6012_end,
						"GUARD: 0x6012's handler contains NO direct call to can_tx_queue_enqueue( - "
						"its trailing marker is a phase of the SAME stop-and-wait transfer that "
						"sends Para2, never enqueued separately");
					int set_trailer = count_active(mf6012_start, mf6012_end, "can_multiframe_set_trailer(", NULL);
					CHECK(set_trailer == 0,
						"GUARD: 0x6012's handler contains NO can_multiframe_set_trailer( - that "
						"start-then-attach pair is REMOVED in v4; the trailer must be armed "
						"atomically with the reply");
					const char *trailer = strstr(mf6012_start, "send_multiframe_trailer(");
					CHECK(trailer != NULL && trailer < mf6012_end,
						"GUARD: 0x6012's handler arms its marker via send_multiframe_trailer( - "
						"the atomic start-with-trailer wrapper");
				}

				/* --- 0x6029: diag_peak_reset must NOT be set in this file at all - it lives in -
				 * main.c, gated on the transfer CONFIRMED complete (see main.c's block below). */
				int peak_reset_here = count_active(clean, NULL, "diag_peak_reset=1;", NULL)
				                    + count_active(clean, NULL, "diag_peak_reset = 1;", NULL);
				CHECK(peak_reset_here == 0,
					"GUARD: src/CAN_Display.c contains ZERO `diag_peak_reset=1;` statements - the "
					"0x6029 handler may only REMEMBER the transfer id (can_reply_effects_6029_armed), "
					"never clear the peaks itself; the reset fires from main.c only on a CONFIRMED "
					"complete transfer (see can_reply_effects.h)");
				const char *mf6029_start = sendcantx_start ? strstr(sendcantx_start, "case 0x6029:") : NULL;
				const char *mf6029_end = mf6029_start ? strstr(mf6029_start + 1, "case 0x6028:") : NULL;
				CHECK(mf6029_start != NULL, "setup: case 0x6029: found inside sendCAN_Tx()");
				CHECK(mf6029_end != NULL, "setup: case 0x6028: found after it, to bound 0x6029's body");
				if (mf6029_start && mf6029_end) {
					const char *tracked = strstr(mf6029_start, "send_multiframe_tracked(Ext_ID_Rx.command, (char*)&dg[0], 55, &xfer_id)");
					CHECK(tracked != NULL && tracked < mf6029_end,
						"GUARD: 0x6029's handler arms the snapshot via send_multiframe_tracked( "
						"with a real transfer id - it must remember WHICH reply, not just 'armed'");
					const char *arm = strstr(mf6029_start, "can_reply_effects_6029_armed(");
					CHECK(arm != NULL && arm < mf6029_end,
						"GUARD: 0x6029's handler feeds the transfer id to "
						"can_reply_effects_6029_armed( - the deferred-reset module, applied from "
						"main.c only when that exact transfer is CONFIRMED complete");
					if (tracked && arm) CHECK(tracked < arm,
						"GUARD: the tracked start comes BEFORE can_reply_effects_6029_armed( - the "
						"id must be remembered only after a successful arm");
				}

				free(clean);
			}
		}
	}

	/* --- main.c: exactly one blocking wait, and it must be inside autodetect() ----------- */
	{
		const char *path = STRINGIZE(MAIN_C_PATH);
		printf("  MAIN_C_PATH = %s\n", path);
		long len = 0;
		char *raw = read_whole_file(path, &len);
		CHECK(raw != NULL, "setup: src/main.c was readable at MAIN_C_PATH");
		if (raw) {
			char *clean = strip_comments(raw, len);
			CHECK(clean != NULL, "setup: main.c comment/string-aware sanitization succeeded");
			free(raw);
			if (clean) {
				const char *autodetect_start = strstr(clean, "void autodetect(void)");
				CHECK(autodetect_start != NULL, "setup: autodetect(void) found in main.c");
				const char *after_autodetect = autodetect_start
					? strstr(autodetect_start, "void ADC0_1_IRQHandler(void)") : NULL;
				CHECK(after_autodetect != NULL,
					"setup: the function following autodetect() found, to bound its body");

				const char *first_hit = NULL;
				int n = count_active(clean, NULL, BLOCKING_WAIT, &first_hit);
				CHECK(n == 1,
					"GUARD: src/main.c contains EXACTLY ONE active blocking "
					"while((CAN_TRANSMIT_OK != can_transmit_states( wait - zero would hide a "
					"silent behaviour change inside autodetect() itself; more than one means a "
					"blocking wait exists somewhere this card did not account for");

				if (n == 1 && autodetect_start && after_autodetect && first_hit) {
					CHECK(first_hit > autodetect_start && first_hit < after_autodetect,
						"GUARD: main.c's one remaining blocking wait sits INSIDE autodetect() - "
						"the one deliberately blocking, service-only Hall calibration routine, "
						"kept unreachable from CAN by this card (no call site anywhere can reach "
						"it). A blocking wait found outside these bounds is a regression even "
						"though the total count would still read as 1");
				}

				/* --- critical frames must be serviced BEFORE the diagnostics dump each tick -- */
				const char *crit_service = strstr(clean, "can_tx_queue_service(control_time_ticks);");
				const char *diag_step = strstr(clean, "diag_dump_step();");
				CHECK(crit_service != NULL,
					"setup: can_tx_queue_service(control_time_ticks); found in main.c's main loop");
				CHECK(diag_step != NULL, "setup: diag_dump_step(); found in main.c's main loop");
				if (crit_service && diag_step) {
					CHECK(crit_service < diag_step,
						"GUARD: can_tx_queue_service( runs BEFORE diag_dump_step( in main.c's main "
						"loop, every tick - this is what gives critical HMI frames priority over "
						"diagnostic ones on a busy bus, as required. Diagnostics running first "
						"would let a busy bus starve the HMI/status frames the display needs to "
						"stay in sync instead of the diagnostic stream, which is backwards");
				}

				/* --- FW-110 v4: main.c never CALLS autodetect() itself ---------------------- */
				int autodetect_calls_main = count_active(clean, NULL, "autodetect();", NULL);
				CHECK(autodetect_calls_main == 0,
					"GUARD: src/main.c contains ZERO active autodetect(); CALLS - it must never "
					"be called directly from main.c's loop again");

				/* --- FW-110 v4: the removed supervisor is gone, the side-effect module wired -- */
				int sup_init = count_active(clean, NULL, "hall_cal_supervisor_init();", NULL);
				CHECK(sup_init == 0,
					"GUARD: src/main.c contains NO hall_cal_supervisor_init() - the FW-110 v3 "
					"calibration supervisor is REMOVED");
				int fx_init = count_active(clean, NULL, "can_reply_effects_init_wrapper();", NULL);
				CHECK(fx_init == 1,
					"GUARD: src/main.c calls can_reply_effects_init_wrapper() exactly once at "
					"startup - the completed-reply side-effect module must start with no pending "
					"transfer id");

				/* --- 0x6029's reset lives in main.c, gated on the CONFIRMED-complete effect ---- */
				int peak_reset_main = count_active(clean, NULL, "diag_peak_reset = 1;", NULL)
				                    + count_active(clean, NULL, "diag_peak_reset=1;", NULL);
				CHECK(peak_reset_main == 1,
					"GUARD: src/main.c contains EXACTLY ONE `diag_peak_reset = 1;` statement - "
					"the only place the peaks may be cleared, and it is the gated effect block "
					"checked next");
				const char *fx_poll = strstr(clean, "can_reply_effects_poll();");
				const char *peak_reset_stmt = strstr(clean, "diag_peak_reset = 1;");
				const char *effect_check = strstr(clean, "CANFX_EFFECT_DIAG_PEAK_RESET");
				CHECK(fx_poll != NULL,
					"setup: can_reply_effects_poll() found in main.c's main loop");
				if (fx_poll && effect_check && peak_reset_stmt) {
					CHECK(effect_check < peak_reset_stmt,
						"GUARD: the CANFX_EFFECT_DIAG_PEAK_RESET comparison comes BEFORE "
						"diag_peak_reset = 1; - the reset is only ever emitted inside that gated block");
					long gap = (long)(peak_reset_stmt - effect_check);
					CHECK(gap > 0 && gap < 250,
						"GUARD: diag_peak_reset = 1; sits close enough after the "
						"CANFX_EFFECT_DIAG_PEAK_RESET comparison to be inside its own if() body, "
						"not merely somewhere later in the file");
					bool brace_between = false;
					for (const char *p = effect_check; p < peak_reset_stmt; p++) {
						if (*p == '}') { brace_between = true; break; }
					}
					CHECK(!brace_between,
						"GUARD: no block-closing '}' between the CANFX_EFFECT_DIAG_PEAK_RESET "
						"comparison and diag_peak_reset = 1; - it is still inside that if()'s own body");
				}

				/* --- autodetect()'s own defense-in-depth: the guard is its literal FIRST stmt - */
				const char *autodetect_def = strstr(clean, "void autodetect(void) {");
				CHECK(autodetect_def != NULL, "setup: autodetect(void) { definition found in main.c");
				if (autodetect_def) {
					const char *p = autodetect_def + strlen("void autodetect(void) {");
					while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
					const char *expect = "if(!hall_calibration_standstill_confirmed()) return;";
					CHECK(strncmp(p, expect, strlen(expect)) == 0,
						"GUARD: autodetect()'s literal FIRST statement is "
						"`if(!hall_calibration_standstill_confirmed()) return;` - defense-in-depth "
						"that holds even if some future card re-enables a call site above it");
				}

				/* --- missed_control_events: uint32_t present, uint16_t spelling gone -------- */
				int u32_count = count_active(clean, NULL, "uint32_t missed_control_events", NULL);
				int u16_count = count_active(clean, NULL, "uint16_t missed_control_events", NULL);
				CHECK(u32_count >= 1,
					"GUARD: src/main.c declares `uint32_t missed_control_events` - the FW-110 fix "
					"for the saturation bug (a session that lost 12,543 ticks published 0 missed "
					"events; see the FW-110 report)");
				CHECK(u16_count == 0,
					"GUARD: src/main.c must NOT declare `uint16_t missed_control_events` - that "
					"spelling is exactly the saturating global this card removed");

				/*
				 * --- the 0x81F83100 gate: src/main.c/src/CAN_Display.c cannot be linked here
				 * (real hardware headers throughout), so this structural check is the ONLY
				 * automated evidence for it, and is deliberately named as such in the FW-110
				 * report rather than presented as behaviourally proven. It confirms the exact
				 * gating expression is present, unconditionally, guarding the one t3100_counter
				 * call site - not that the runtime call counts are actually zero under load,
				 * which needs either linking main.c (not feasible) or a bench/hardware test.
				 */
				int torque_gate = count_active(clean, NULL,
					"if(t3100_counter > 40 && can_tx_queue_depth() == 0U && !can_multiframe_busy()){",
					NULL);
				CHECK(torque_gate == 1,
					"GUARD: main.c's 0x81F83100 call site is gated on can_tx_queue_depth()==0 AND "
					"!can_multiframe_busy() - not attempted at all while any critical frame is "
					"queued, in flight, or still being produced by the multiframe automaton");

				free(clean);
			}
		}
	}

	if (host_test_failures == 0) {
		printf("FW-110 can-blocking guard passed - CAN_Display.c has no blocking waits and no\n");
		printf("autodetect()/calibration path, main.c's one wait is inside autodetect(), and\n");
		printf("0x6029's peak reset is gated in main.c on a CONFIRMED complete transfer.\n");
		return 0;
	}
	printf("\n%d fw110_can_blocking_guard check(s) FAILED.\n", host_test_failures);
	return 1;
}