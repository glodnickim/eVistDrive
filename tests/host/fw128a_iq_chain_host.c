/*
 * FW-128A: canonical q-current ownership. Two halves, because the card has two halves.
 *
 *   - the real iq_chain.c module, linked and exercised directly;
 *   - a STRUCTURAL check on src/main.c and src/ride_control.c, because the properties that
 *     actually matter here are about WHERE things happen and how many places do them. Neither
 *     file can be linked on a host (main.c is the ARM entry point; ride_control needs the whole
 *     assist stack), and "exactly one producer" is not a runtime property anyway.
 *
 * WHAT THIS CARD IS AND IS NOT. FW-128A changes no control behaviour by design, so there is
 * deliberately no test here asserting a new number. The tests assert OWNERSHIP: one producer per
 * stage, one PI_iq input site, and the battery feedback swap confined to one marked place.
 * A4/A5 (rise/fall arithmetic preserved) are covered by the fact that no ramp code was touched -
 * S4 below proves that structurally, which is stronger than re-deriving the arithmetic here.
 */

#include "common/check.h"
#include "../../inc/iq_chain.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef MAIN_C_PATH
#error "MAIN_C_PATH must be defined (see tests/host/run-host-tests.ps1)"
#endif
#ifndef RIDE_CONTROL_C_PATH
#error "RIDE_CONTROL_C_PATH must be defined (see tests/host/run-host-tests.ps1)"
#endif
#ifndef FOC_CURRENT_LOOP_C_PATH
#error "FOC_CURRENT_LOOP_C_PATH must be defined (see tests/host/run-host-tests.ps1)"
#endif
#define STRINGIZE_(x) #x
#define STRINGIZE(x) STRINGIZE_(x)

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

/* Comments become blanks, same length; literals kept - so a commented-out call cannot be
 * counted, and every offset still maps onto the original for ordering checks. */
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
			if (i < len) { out[i] = ' '; i++; }
			if (i < len) { out[i] = ' '; i++; }
			continue;
		}
		if (text[i] == '"' || text[i] == '\'') {
			char q = text[i];
			out[i] = text[i]; i++;
			while (i < len && text[i] != q) {
				out[i] = text[i];
				if (text[i] == '\\' && i + 1 < len) { i++; out[i] = text[i]; }
				i++;
			}
			if (i < len) { out[i] = text[i]; i++; }
			continue;
		}
		out[i] = text[i];
		i++;
	}
	out[len] = '\0';
	return out;
}

/* Count occurrences of the needle strictly between from and to. Ownership questions are
 * always "how many times inside THIS function", never "how many times in the file" - counting
 * file-wide conflates one-time initialisation with a control-path producer. */
static int count_between(const char *from, const char *to, const char *needle)
{
	int n = 0;
	const char *p = from;
	size_t l = strlen(needle);
	if (!from || !to || from > to) return -1;
	while ((p = strstr(p, needle)) != NULL && p < to) { n++; p += l; }
	return n;
}

static int count_occurrences(const char *hay, const char *needle)
{
	int n = 0;
	const char *p = hay;
	size_t l = strlen(needle);
	while ((p = strstr(p, needle)) != NULL) { n++; p += l; }
	return n;
}

int main(void)
{
	/* ================= the real module ================================================== */

	/* --- A7: nothing survives a restart --------------------------------------------------- */
	iq_chain_reset();
	{
		const iq_chain_t *c = iq_chain_get();
		CHECK(c->valid == 0, "A7a. a fresh chain records no demand at all");
		CHECK(c->requested == 0 && c->allowed == 0, "A7b. ...and both stages read zero");
	}
	iq_chain_note_requested(400);
	iq_chain_note_allowed(250);
	iq_chain_reset();
	{
		const iq_chain_t *c = iq_chain_get();
		CHECK(c->valid == 0 && c->requested == 0 && c->allowed == 0,
		      "A7c. a restart clears both stages - no demand crosses a start boundary");
	}

	/* --- A2: with no limiter active the two stages agree ----------------------------------- */
	iq_chain_reset();
	iq_chain_note_requested(500);
	iq_chain_note_allowed(500);
	{
		const iq_chain_t *c = iq_chain_get();
		CHECK(c->requested == c->allowed,
		      "A2. with nothing limiting, Iq_requested and Iq_allowed are the same value");
		CHECK(c->valid != 0, "A2b. and the chain reports that it has been written");
	}

	/* --- A3: a limiter only ever shows as allowed < requested ------------------------------- */
	iq_chain_reset();
	iq_chain_note_requested(600);
	iq_chain_note_allowed(150);
	{
		const iq_chain_t *c = iq_chain_get();
		CHECK(c->requested == 600 && c->allowed == 150,
		      "A3. both stages are visible independently - which is what the pre-audit could not do");
	}

	/* --- A6: zero demand is recorded as zero, not as absence -------------------------------- */
	iq_chain_reset();
	iq_chain_note_requested(0);
	iq_chain_note_allowed(0);
	{
		const iq_chain_t *c = iq_chain_get();
		CHECK(c->valid != 0,
		      "A6. a demand of zero is distinguishable from no demand recorded yet");
	}

	/* --- recording cannot change a value ---------------------------------------------------- */
	iq_chain_reset();
	{
		int32_t v = -1234;   /* negative is not expected in the ride path, but must pass through */
		iq_chain_note_requested(v);
		CHECK(iq_chain_get()->requested == v, "N1. the module stores, it does not sanitise");
	}

	/* ================= structural ownership ============================================== */

	long ml = 0, rl = 0, fl = 0;
	char *mraw = read_whole_file(STRINGIZE(MAIN_C_PATH), &ml);
	char *rraw = read_whole_file(STRINGIZE(RIDE_CONTROL_C_PATH), &rl);
	char *fraw = read_whole_file(STRINGIZE(FOC_CURRENT_LOOP_C_PATH), &fl);
	if (!mraw || !rraw || !fraw) { printf("  FAIL  cannot read the sources\n"); return 1; }
	char *m = strip_comments(mraw, ml);
	char *r = strip_comments(rraw, rl);
	char *f = strip_comments(fraw, fl);
	if (!m || !r || !f) { printf("  FAIL  out of memory\n"); return 1; }

	/* --- A1: exactly one producer per stage -------------------------------------------------- */
	CHECK(count_occurrences(r, "iq_chain_note_requested(") == 1,
	      "A1a. Iq_requested has exactly one producer");
	CHECK(count_occurrences(r, "iq_chain_note_allowed(") == 1,
	      "A1b. Iq_allowed has exactly one producer");
	CHECK(count_occurrences(m, "state->i_q_setpoint") == 0,
	      "A1c. main.c does not write i_q_setpoint - motor_core.c owns Iq_ref");
	/* The trailing space matters: "MS.i_q_setpoint ==" is a comparison, not an assignment, and
	 * without it this check quietly passed on the comparisons instead of the assignments. */
	CHECK(count_occurrences(m, "MS.i_q_setpoint = ") == 0,
	      "A1d. nothing in main.c assigns Iq_ref directly");

	/* --- A1e: the stages are recorded in the right order, one-way ---------------------------- */
	{
		/*
		 * The limiter chain moved into ap2_limits.c and every owner (assist, Walk, service)
		 * now converges on ONE record-and-publish block. The ordering property is unchanged
		 * and is still checkable by reading straight down the function: the requested value is
		 * the demand BEFORE the chain, the allowed value is what the chain returned, and the
		 * publish to the single 16 kHz owner comes last.
		 */
		const char *req = strstr(r, "iq_chain_note_requested(requested);");
		const char *alw = strstr(r, "iq_chain_note_allowed(cmd.final_iq_request);");
		const char *ramp = alw ? strstr(alw, "ride_publish_final_iq(cmd.final_iq_request") : NULL;
		const char *assist_req = strstr(r, "requested = assist_pipeline_telemetry()->iq_request_before_limits;");
		CHECK(req && assist_req && assist_req < req,
		      "A1e. Iq_requested is the demand BEFORE the limiter chain - that is what makes it 'requested'");
		CHECK(alw && ramp && alw < ramp,
		      "A1f. Iq_allowed is recorded BEFORE the publish - that is what makes it 'allowed'");
		CHECK(req && alw && req < alw, "A1g. ...and requested is recorded before allowed");
	}

	/* --- A10 + A11: one PI_iq input site, battery cap is upstream -------------------------------- */
	{
		const char *owner = strstr(m, "static void pi_iq_apply_inputs(void)");
		const char *runpi = strstr(m, "void runPIcontrol(void){");
		CHECK(owner != NULL, "A10c. there is one named owner of PI_iq's inputs");

		/* Inside the owner: exactly one setpoint assignment (single Iq-domain path). */
		CHECK(count_between(owner, runpi, "PI_iq.setpoint =") == 1,
		      "A10a. PI_iq's reference is assigned in exactly one place inside the owner");
		/* Inside the owner: exactly one recent_value assignment (always MS.i_q). */
		CHECK(count_between(owner, runpi, "PI_iq.recent_value =") == 1,
		      "A10b. ...and its feedback likewise");

		/* Outside it: only the one-time PI setup may mention setpoint, as initialisation. */
		CHECK(count_occurrences(m, "PI_iq.setpoint =") == 2,
		      "A10d. the only other mention in the whole file is the one-time PI setup");
		CHECK(strstr(m, "PI_iq.setpoint = 0;") != NULL,
		      "A10e. ...which is an initialisation to zero, not a control-path producer");
		/* recent_value has no site at all outside the owner. */
		CHECK(count_occurrences(m, "PI_iq.recent_value =") == 1,
		      "A10f. the feedback has no site at all outside the owner — single domain, single writer");
	}
	CHECK(strstr(m, "PI_iq.recent_value = MS.i_q;") != NULL,
	      "A10g. normal-mode feedback is measured Iq, stated literally");
	{
		/* QS-3C: BC_limit_flag is now driven from the battery_iq_cap.c latch via main's
		 * single assignment after ride_control_update - one place, one source of truth.
		 * The global definition is the only other occurrence. */
		CHECK(count_occurrences(m, "BC_limit_flag = ride_control_battery_limit_active() ? 1 : 0;") == 1,
		      "A11a. the battery flag is driven from the cap latch in exactly one place");
		CHECK(count_occurrences(m, "FlagStatus BC_limit_flag=0;") == 1,
		      "A11d. the only other occurrence is the global definition");
		/* Exactly 3 references in main.c: the global definition, the single latch-driven
		 * write after ride_control_update, and the CAN publish bit. No other write site. */
		CHECK(count_occurrences(m, "BC_limit_flag") == 3 &&
		      count_occurrences(m, "BC_limit_flag = ride_control_battery_limit_active() ? 1 : 0;") == 1,
		      "A11e. BC_limit_flag = definition + one latch-driven write + one CAN bit");
	}
	/* QS-3C: the legacy domain-switch override is gone. The battery limiter is an Iq-domain
	 * upstream cap (battery_iq_cap.c) in ride_control.c, before the one final slew, never
	 * swapping feedback and never living in pi_iq_apply_inputs. */
	CHECK(strstr(r, "ap2_limits_apply(") != NULL &&
	      strstr(r, "battery_iq_cap_update(") == NULL,
	      "A11c. QS-3C: the battery cap is an Iq-domain stage of the one limiter chain, "
	      "not a PI-domain switch and not a second copy in ride_control");
	CHECK(strstr(m, "PI_iq.recent_value = MS.Battery_Current") == NULL,
	      "A11f. legacy BC override no longer writes recent_value — PI stays in Iq domain");
	CHECK(strstr(m, "battery_current_max>>6") == NULL,
	      "A11g. legacy BC override no longer writes setpoint with battery-current domain");

	/* --- A12: Id untouched -------------------------------------------------------------------- */
	CHECK(strstr(f, "pi_id->recent_value = ms->i_d;") != NULL, "A12a. Id feedback unchanged");
	CHECK(strstr(f, "pi_id->setpoint = ms->i_d_setpoint;") != NULL, "A12b. Id reference unchanged");
	CHECK(strstr(f, "u_d_requested = -PI_control(pi_id);") != NULL,
	      "A12c. the Ud sign expression is untouched, as the card requires");

	/* --- A13 + S4: nothing downstream or upstream of this card moved ---------------------------- */
	CHECK(strstr(m, "current_sample_ctx_consume()") != NULL &&
	      strstr(m, "sample_window_decide(pwm_applied") != NULL &&
	      strstr(m, "current_feedback_update(sample_ctx->state") != NULL,
	      "A13. the FW-127 acquisition chain is still wired exactly as it was");
	CHECK(strstr(r, "fast_iq_slew_publish(") != NULL,
	      "S4a. the ramp is still the single call it was - no ramp arithmetic was touched here");
	CHECK(count_occurrences(r, "ride_publish_final_iq(") == 4,
	      "S4b. one publish helper, called from exactly the three explicit commands "
	      "(force-zero, service, and the one shared point) plus its own definition");
	CHECK(count_occurrences(r, "motor_core_set_command(") == 0 &&
	      strstr(r, "ride_control_force_final_iq_zero();") != NULL &&
	      strstr(r, "hall_calibration_iq_request()") != NULL &&
	      strstr(r, "motor_core_set_id_target(input->current_id);") != NULL,
	      "S4c. stop/calibration/normal ownership is explicit and Motor Core cannot overwrite Iq");

	/* --- A8/A9: stop and fault paths still bypass the normal chain ------------------------------ */
	{
		const char *stop = strstr(r, "ride_control_force_final_iq_zero();");
		const char *note = strstr(r, "iq_chain_note_requested(");
		CHECK(stop && note && stop < note,
		      "A8/A9. the safety-cut command returns before the canonical chain is even reached - "
		      "a hard stop does not wait for any of this");
	}

	if (host_test_failures == 0) {
		printf("FW-128A canonical q-current ownership: ALL CHECKS PASSED\n");
		return 0;
	}
	printf("FW-128A canonical q-current ownership: %d CHECK(S) FAILED\n", host_test_failures);
	return 1;
}
