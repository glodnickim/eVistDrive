/*
 * MISSED_TICK_BURST — demonstrates, against the SHIPPED src/torque_input.c and
 * src/ride_episode.c (not copies), the distinction the card asks for (section 9):
 *
 *   A. ELAPSED-TIME BEHAVIOUR   — a module that takes an explicit hardware tick count
 *                                  and computes elapsed time by subtraction. Correct
 *                                  under missed calls BY CONSTRUCTION.
 *   B. CONTROL-UPDATE BEHAVIOUR — a module that assumes "one call = one 4 kHz tick" and
 *                                  counts its OWN calls. Runs slower than its configured
 *                                  millisecond value whenever calls are skipped.
 *   C. LOST PHYSICAL SAMPLE     — information that cannot be recovered by ANY amount of
 *                                  elapsed-time bookkeeping, because the thing that would
 *                                  have been measured (a PAS quadrature transition, a
 *                                  torque sample) never got sampled while nothing was
 *                                  polling it.
 *
 * This program does NOT fix anything (card section 9: "NIE naprawiaj znalezionych
 * różnic"). It only measures and reports. See documentation/testing/TIMEBASES.md (via
 * documentation/architecture/TIMEBASES.md) and the architecture audit's finding F1 for
 * the production-code background this test exists to make concrete and repeatable.
 *
 * Usage: missed_tick_burst_host <output_summary.csv>
 */

#include <stdio.h>
#include <string.h>

#include "crank_model.h"
#include "csv.h"
#include "ride_episode.h"
#include "torque_input.h"

/*
 * ---- A. ELAPSED-TIME: ride_episode.c takes an explicit now_tick every call ---------
 *
 * ONE ground-truth timeline, so "normal" and "burst" are two different SAMPLINGS of the
 * same real event, not two different events. Anchor (reverse step) at tick 1000 while
 * 500 (native Iq units) was flowing:
 *   arm_seq flips 0 -> 1 at tick 1100  (the ride latch genuinely re-arms first)
 *   iq_setpoint recovers to 500 at tick 1500 (the ramp genuinely delivers current later)
 * A module call at tick T sees whatever this ground truth truly is AT T.
 */
static void episode_ground_truth(uint32_t tick, int32_t *iq_setpoint, uint16_t *arm_seq)
{
	*iq_setpoint = (tick < 1500) ? 100 : 500;
	*arm_seq = (tick < 1100) ? 0U : 1U;
}

static void run_episode_scenario(int dense_calls, uint16_t *out_t_latch_ms, uint16_t *out_t_recover_ms)
{
	ride_episode_init();
	ride_episode_reverse_step(/*iq_setpoint_now=*/500, /*arm_seq_now=*/0, /*now_tick=*/1000);

	if (dense_calls) {
		/* "Normal": one call per real 4 kHz tick — the regime every module in this
		 * firmware is DESIGNED around. */
		for (uint32_t tick = 1000; tick <= 1600; tick++) {
			ride_episode_input_t in = { 0 };
			episode_ground_truth(tick, &in.iq_setpoint, &in.arm_seq);
			in.iq_pre_ramp = in.iq_setpoint;
			ride_episode_tick(&in, tick);
		}
	} else {
		/* "Burst": the main loop falls behind and this module is only invoked TWICE in
		 * the same real interval — but each call still carries the TRUE hardware tick
		 * (control_time_ticks in main.c, read from the free-running TIMER1 ISR counter -
		 * see main.c:1560 and reg_ADC_processing's `const uint32_t control_now =
		 * control_time_ticks;`). That is the one thing that must stay true for this
		 * category to behave correctly: main.c does NOT reconstruct now_tick from a
		 * count of its own calls at this module's call sites (FW-104).
		 */
		uint32_t sparse_ticks[2] = { 1000U, 1600U };
		for (int i = 0; i < 2; i++) {
			ride_episode_input_t in = { 0 };
			episode_ground_truth(sparse_ticks[i], &in.iq_setpoint, &in.arm_seq);
			in.iq_pre_ramp = in.iq_setpoint;
			ride_episode_tick(&in, sparse_ticks[i]);
		}
	}

	ride_episode_result_t result;
	ride_episode_get_result(&result);
	*out_t_latch_ms = result.t_latch_ms;
	*out_t_recover_ms = result.t_recover_ms;
}

/* ---- B. CONTROL-UPDATE: torque_input.c's 35 ms FAST filter counts its OWN calls ---- */

static uint16_t run_fast_filter_scenario(int call_count)
{
	torque_input_init();
	/* A step input: pedal load jumps from zero to a steady ~19 kg equivalent and holds. */
	const uint16_t raw_mv = 740 + 400;
	uint16_t fast = 0;
	for (int i = 0; i < call_count; i++) {
		int16_t corrected = torque_input_correct(raw_mv);
		torque_input_coast_update(corrected, false, true);
		torque_input_update(raw_mv, corrected, true);
		fast = torque_input_get_snapshot()->assist_delta_filtered_native;
	}
	return fast;
}

/* ---- C. LOST PHYSICAL SAMPLE: illustrative only, see the file header -------------- */

static void run_lost_sample_scenario(uint32_t *out_true_steps, uint32_t *out_apparent_steps)
{
	/* 90 rpm is a plausible mid-ride cadence. 400 ticks (100 ms) is a SEVERE main-loop
	 * stall, well beyond a routine CAN-diagnostics burst — chosen deliberately large so
	 * the aliasing this illustrates is unambiguous rather than a rounding coincidence at
	 * a shorter, more realistic gap. */
	const double cadence_rpm = 90.0;
	const uint32_t gap_ticks = 400;

	crank_state_t state;
	crank_state_init(&state);
	/* Run up to a representative mid-revolution point before the gap. */
	for (uint32_t i = 0; i < 400; i++) {
		crank_state_advance_tick(&state, cadence_rpm);
	}
	uint32_t steps_before = state.step_count;
	uint8_t phase_before = crank_pas_state(&state);

	for (uint32_t i = 0; i < gap_ticks; i++) {
		crank_state_advance_tick(&state, cadence_rpm); /* the crank keeps turning... */
	}
	uint32_t steps_after = state.step_count;
	uint8_t phase_after = crank_pas_state(&state);

	*out_true_steps = steps_after - steps_before;
	/* What a decoder that can only compare two SNAPSHOTS of quadrature phase (not a
	 * continuous 4 kHz read of the two GPIO lines) would infer: the phase difference
	 * modulo 4. This is the aliasing a state-sampling decoder is exposed to when more
	 * than a few steps happen between the samples it actually takes. */
	*out_apparent_steps = (uint32_t)((phase_after + 4 - phase_before) % 4);
}

int main(int argc, char **argv)
{
	if (argc != 2) {
		fprintf(stderr, "usage: %s <output_summary.csv>\n", argv[0]);
		return 2;
	}

	printf("MISSED_TICK_BURST\n");

	uint16_t t_latch_normal, t_recover_normal, t_latch_burst, t_recover_burst;
	run_episode_scenario(1, &t_latch_normal, &t_recover_normal);
	run_episode_scenario(0, &t_latch_burst, &t_recover_burst);
	printf("  A. ride_episode (elapsed-time, explicit now_tick) - SAME ground-truth timeline,\n");
	printf("     sampled densely vs sparsely (arm_seq truly flips at tick 1100, iq truly\n");
	printf("     recovers at tick 1500; anchor at tick 1000):\n");
	printf("     normal (601 calls): t_latch=%u ms  t_recover=%u ms  (matches ground truth: 25 ms / 125 ms)\n",
		t_latch_normal, t_recover_normal);
	printf("     burst    (2 calls): t_latch=%u ms  t_recover=%u ms\n",
		t_latch_burst, t_recover_burst);
	printf("     %s\n", (t_recover_normal == t_recover_burst && t_latch_normal == t_latch_burst) ?
		"IDENTICAL - immune to the missed calls." :
		"LATER, NOT WRONG: both timestamps use the real tick the module was actually\n"
		"        called with (no clock drift, no negative error) — but a state change that\n"
		"        happened BETWEEN two surviving calls can only be reported at the tick of\n"
		"        the NEXT call that observes it, never earlier. Duration measured FROM an\n"
		"        anchor stays correct; the anchor/edge timestamp itself is bounded by call\n"
		"        density. See finding F1-b in the final report.");

	/* 140 calls = 35 ms of real ticks at the assumed 4 kHz rate
	 * (TORQUE_ASSIST_FILTER_MS * TORQUE_INPUT_TICKS_PER_MS in inc/torque_input.h). */
	uint16_t fast_normal = run_fast_filter_scenario(140);
	/* Burst: only 35 of those 140 calls actually happen before the checkpoint - the
	 * same "4x fewer calls than real ticks" ratio as the ride_episode burst above. */
	uint16_t fast_burst = run_fast_filter_scenario(35);
	printf("  B. torque_input FAST filter (35 ms, call-counted):\n");
	printf("     normal (140 calls): assist_delta_filtered_native=%u\n", fast_normal);
	printf("     burst   (35 calls): assist_delta_filtered_native=%u\n", fast_burst);
	printf("     %s\n", (fast_normal == fast_burst) ?
		"IDENTICAL (unexpected for a call-counted filter)." :
		"DIFFERENT - the filter is measurably LESS settled under the same nominal 35 ms (see finding F1).");

	uint32_t true_steps, apparent_steps;
	run_lost_sample_scenario(&true_steps, &apparent_steps);
	printf("  C. PAS quadrature steps across a 400-tick (100 ms) gap @ 90 rpm (illustrative):\n");
	printf("     true steps that physically occurred: %u\n", true_steps);
	printf("     steps a 2-snapshot phase comparison would infer: %u\n", apparent_steps);
	printf("     %s\n", (true_steps == apparent_steps) ?
		"no aliasing at this gap length" :
		"ALIASED - the difference is genuinely unrecoverable, not merely mistimed.");

	FILE *out = csv_open_or_die(argv[1],
		"category,module,normal_value,burst_value,unit,note");
	fprintf(out, "A_elapsed_time,ride_episode.t_recover_ms,%u,%u,ms,later_not_wrong_duration_from_anchor_still_correct\n",
		t_recover_normal, t_recover_burst);
	fprintf(out, "A_elapsed_time,ride_episode.t_latch_ms,%u,%u,ms,later_not_wrong_edge_bounded_by_call_density\n",
		t_latch_normal, t_latch_burst);
	fprintf(out, "B_control_update,torque_input.assist_delta_filtered_native,%u,%u,native,diverges_under_burst\n",
		fast_normal, fast_burst);
	fprintf(out, "C_lost_sample,pas_steps_in_400tick_gap,%u,%u,steps,aliased_not_recoverable\n",
		true_steps, apparent_steps);
	fclose(out);

	printf("summary -> %s\n", argv[1]);
	return 0;
}
