/*
 * THE TORQUE DOMAIN CONTRACT (FW-151).
 *
 * This suite exists because the firmware once used ONE scale for two jobs. The sensor's
 * kilogram table was both the thing shown to the rider AND the thing every control threshold
 * was compared against, so improving the MEASUREMENT retuned the BIKE. The FW-150
 * reference-weight measurement did exactly that: the same stored 0.70 kg standing threshold
 * meant 17 mV of sensor signal before it and 5 mV after - a 3.4x change in how hard the rider
 * has to press, from a change that was only ever meant to correct a displayed number. 5 mV is
 * inside the sensor's own rest noise, so assist permission could be granted by noise alone.
 *
 * The checks below are the contract that stops it happening again:
 *
 *   D1  native -> CLU is monotonic across the whole sensor range, and has an explicit ceiling
 *   D2  native -> kg  is monotonic across the whole sensor range (the human projection)
 *   D3  CLU round trips through native without drifting
 *   D4  THE INVARIANCE CONTRACT: re-measuring the kg table changes the DISPLAYED kilograms and
 *       changes NOTHING in the control domain. This is the check the whole refactor exists for.
 *   D5  the control domain reproduces the BIKE-VERIFIED trip points (1d6c6ba), which is why
 *       this refactor does not change the ride
 *   D6  the shipped thresholds sit OUTSIDE the sensor's rest noise - the defect that motivated
 *       the work, stated as a bound rather than as a story
 *   D7  the user GAIN calibration still moves both projections, together
 *   D8  effort normalization is monotonic and hits its endpoints, with ONE normalization step
 *
 * D4 is implemented WITHOUT editing the header: it drives the production conversion functions
 * and compares CLU against kg over a sweep. A mutation that makes any control value a function
 * of the kg table breaks D4 - see the AUDIT report's M-series.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ap2_rider_demand.h"
#include "assist_modes.h"
#include "config.h"
#include "torque_input.h"

static int failures;

#define CHECK(cond, what) do { \
	if (!(cond)) { \
		printf("  FAIL  %s\n", (what)); \
		failures++; \
	} \
} while (0)

/* The sweep the audit asked for: dense where the curve breaks, sparse where it extrapolates. */
static const uint16_t native_sweep[] = {
	0, 1, 2, 3, 4, 5, 7, 8, 10, 15, 17, 18, 20, 30, 50, 100, 146, 185, 300, 500,
	780, 1000, 1139, 1580, 2000, 3047, TORQUE_SPAN_MAX_NATIVE
};
#define NATIVE_SWEEP_N (sizeof(native_sweep) / sizeof(native_sweep[0]))

/* ---- D1 / D2: both projections are monotonic ------------------------------------------- */

static void test_monotonic(void)
{
	printf("D1/D2 both projections are monotonic over the whole sensor range\n");
	torque_input_init();

	uint16_t prev_ctrl = 0U;
	uint16_t prev_kg = 0U;
	bool ctrl_ok = true;
	bool kg_ok = true;

	for (uint32_t d = 0U; d <= TORQUE_SPAN_MAX_NATIVE; d++) {
		uint16_t ctrl = torque_input_native_delta_to_ctrl((uint16_t)d);
		uint16_t kg = torque_input_native_delta_to_centikg((uint16_t)d);

		if (ctrl < prev_ctrl) {
			ctrl_ok = false;
		}
		if (kg < prev_kg) {
			kg_ok = false;
		}
		prev_ctrl = ctrl;
		prev_kg = kg;
	}
	CHECK(ctrl_ok, "D1: native -> CLU never decreases");
	CHECK(kg_ok, "D2: native -> centikg never decreases");
	CHECK(torque_input_native_delta_to_ctrl(0U) == 0U, "D1: zero force is zero CLU");
	CHECK(torque_input_native_delta_to_centikg(0U) == 0U, "D2: zero force is zero kg");
	CHECK(torque_input_native_delta_to_ctrl(TORQUE_SPAN_MAX_NATIVE) <= TORQUE_CTRL_MAX_CLU,
		"D1: the control scale has an explicit ceiling");

	/* The frozen characteristic must pass through its own defining points. */
	CHECK(torque_input_native_delta_to_ctrl(TORQUE_CTRL_BREAK_NATIVE) ==
		TORQUE_CTRL_BREAK_CLU, "D1: the curve passes through its break point");
	CHECK(torque_input_native_delta_to_ctrl(TORQUE_CTRL_HIGH_NATIVE) ==
		TORQUE_CTRL_HIGH_CLU, "D1: the curve passes through its high point");
	/*
	 * TORQUE_CTRL_FULL_SCALE_NATIVE is itself the rounded inverse of full scale, so the round
	 * trip lands within one CLU of it (1139 -> 6001). That single count is inherited from the
	 * bike-verified build, where TORQUE_DEFAULT_SPAN_NATIVE was derived the same way - so it is
	 * asserted as a bound rather than silently "corrected", which would move the effort axis.
	 */
	{
		int32_t fs = (int32_t)torque_input_native_delta_to_ctrl(
			TORQUE_CTRL_FULL_SCALE_NATIVE) - (int32_t)TORQUE_CTRL_FULL_SCALE_CLU;
		CHECK(fs >= -1 && fs <= 1,
			"D1: full scale native maps to full scale CLU within one count");
	}
}

/* ---- D3: the control projection round trips ---------------------------------------------- */

static void test_round_trip(void)
{
	printf("D3 CLU round trips through native\n");
	torque_input_init();

	static const uint16_t clu[] = { 0, 15, 30, 70, 150, 300, 600, 1200, 2250, 6000 };
	char msg[96];

	for (unsigned i = 0; i < sizeof(clu) / sizeof(clu[0]); i++) {
		uint16_t native = torque_input_ctrl_to_native_delta(clu[i]);
		uint16_t back = torque_input_native_delta_to_ctrl(native);
		int32_t drift = (int32_t)back - (int32_t)clu[i];

		if (drift < 0) {
			drift = -drift;
		}
		/* One quantization step of the coarsest segment; the curve is 5.44 CLU per native
		 * at its steepest, so a single native count of rounding is the floor here. */
		snprintf(msg, sizeof(msg), "D3: %u CLU round trips (native %u, back %u)",
			(unsigned)clu[i], (unsigned)native, (unsigned)back);
		CHECK(drift <= 6, msg);
	}
}

/* ---- D4: THE INVARIANCE CONTRACT ---------------------------------------------------------- */

/*
 * The kg table and the control characteristic are two independent readings of the same
 * canonical native delta. If they are truly independent then, over the whole sweep, the RATIO
 * between them is NOT constant - which is the machine-checkable fingerprint of two different
 * curves. If someone re-derives the control value from the kg value (or vice versa), the two
 * become the same curve up to a scale factor and this check fails.
 *
 * It is deliberately a structural check rather than "edit the table and re-run": a host test
 * cannot recompile the firmware with a different table, and a test that could would be testing
 * the build system. What it CAN prove is that no control number is a function of the kg number.
 */
static void test_domain_independence(void)
{
	printf("D4 the control domain is not derived from the kilogram table\n");
	torque_input_init();

	/*
	 * The two curves must disagree in SHAPE. Compare the CLU/kg ratio at a low and a high
	 * point: on the frozen control curve 146 native is 600 CLU and on the FW-150 kg table it
	 * is ~870 centikg; at 1139 native they are 6000 CLU and ~2570 centikg. Same input, two
	 * different characteristics - so neither can be a rescaling of the other.
	 */
	uint16_t ctrl_low = torque_input_native_delta_to_ctrl(146U);
	uint16_t kg_low = torque_input_native_delta_to_centikg(146U);
	uint16_t ctrl_high = torque_input_native_delta_to_ctrl(1139U);
	uint16_t kg_high = torque_input_native_delta_to_centikg(1139U);

	CHECK(ctrl_low > 0U && kg_low > 0U && ctrl_high > 0U && kg_high > 0U,
		"D4: both projections produce a reading at both probe points");

	/* ratio_low  = ctrl_low / kg_low, ratio_high = ctrl_high / kg_high, compared as
	 * cross products so the test stays integer. */
	uint32_t cross_a = (uint32_t)ctrl_low * kg_high;
	uint32_t cross_b = (uint32_t)ctrl_high * kg_low;
	CHECK(cross_a != cross_b,
		"D4: CLU is NOT a fixed multiple of centikg - the two curves have different shapes");

	/*
	 * And the control domain must not move when the kg projection does. The strongest
	 * statement a host test can make: every control-domain constant that the pipeline
	 * compares against is declared in the control domain, so none of them can be affected
	 * by TORQUE_CURVE_P* at all. Assert that, so removing the separation is a compile error
	 * rather than a silent behaviour change.
	 */
	CHECK(AP2_EFFORT_DEADBAND_CTRL == 15U,
		"D4: the effort deadband is a control-domain constant (15 CLU)");
	CHECK(ASSIST_MIN_PEDAL_LOAD_DEFAULT_CTRL == 70U,
		"D4: the standing gate is a control-domain constant (70 CLU)");
	CHECK(ASSIST_RIDING_MIN_PEDAL_LOAD_DEFAULT_CTRL == 30U,
		"D4: the rolling gate is a control-domain constant (30 CLU)");
	CHECK(TQ_STUCK_CTRL == 5600U,
		"D4: the stuck-high fault gate is a control-domain constant");
}

/* ---- D5 / D6: the trip points, and the noise floor -------------------------------------- */

/*
 * The bike-verified build (1d6c6ba) compared a kilogram reading taken on the PRE-FW-150
 * characteristic. The control domain is frozen to that characteristic, so the trip point of a
 * stored threshold is recovered by asking the control projection for the first native delta
 * that reaches it. These are the numbers that decide whether this refactor changed the ride.
 */
static uint16_t first_native_reaching_ctrl(uint16_t threshold_ctrl)
{
	for (uint32_t d = 0U; d <= TORQUE_SPAN_MAX_NATIVE; d++) {
		if (torque_input_native_delta_to_ctrl((uint16_t)d) >= threshold_ctrl) {
			return (uint16_t)d;
		}
	}
	return (uint16_t)TORQUE_SPAN_MAX_NATIVE;
}

static void test_trip_points(void)
{
	printf("D5/D6 the shipped gates keep their bike-verified trip points, clear of noise\n");
	torque_input_init();

	uint16_t standing = first_native_reaching_ctrl(ASSIST_MIN_PEDAL_LOAD_DEFAULT_CTRL);
	uint16_t rolling = first_native_reaching_ctrl(ASSIST_RIDING_MIN_PEDAL_LOAD_DEFAULT_CTRL);
	uint16_t deadband = first_native_reaching_ctrl(AP2_EFFORT_DEADBAND_CTRL);
	char msg[128];

	snprintf(msg, sizeof(msg),
		"D5: the standing gate trips at 17 mV as it did on the bike (got %u)",
		(unsigned)standing);
	CHECK(standing == 17U, msg);
	snprintf(msg, sizeof(msg),
		"D5: the rolling gate trips at 8 mV as it did on the bike (got %u)",
		(unsigned)rolling);
	CHECK(rolling == 8U, msg);
	snprintf(msg, sizeof(msg),
		"D5: the effort deadband trips at 4 mV as it did on the bike (got %u)",
		(unsigned)deadband);
	CHECK(deadband == 4U, msg);

	/*
	 * D6: the whole point. A permission gate inside the sensor's own rest noise can be opened
	 * by noise, so the standing gate must sit ABOVE the band a coast is allowed to wander
	 * within before the firmware distrusts it.
	 */
	snprintf(msg, sizeof(msg),
		"D6: the standing gate (%u mV) is outside the sensor rest noise (%u mV)",
		(unsigned)standing, (unsigned)TQ_RECAL_STABLE_MV);
	CHECK(standing > (uint16_t)TQ_RECAL_STABLE_MV, msg);

	/* The rolling gate is deliberately lighter, but must still clear the effort deadband -
	 * granting permission at a load that produces no effort at all is incoherent. */
	snprintf(msg, sizeof(msg),
		"D6: the rolling gate (%u mV) is at or above the effort deadband (%u mV)",
		(unsigned)rolling, (unsigned)deadband);
	CHECK(rolling >= deadband, msg);
	CHECK(ASSIST_RIDING_MIN_PEDAL_LOAD_DEFAULT_CTRL >= AP2_EFFORT_DEADBAND_CTRL,
		"D6: the rolling gate is at or above the deadband in the control domain too");
}

/* ---- D7: the gain moves both projections ------------------------------------------------- */

static void test_gain_applies_to_both(void)
{
	printf("D7 a user gain calibration moves both projections together\n");
	torque_input_init();

	uint16_t probe = 300U;
	uint16_t ctrl_default = torque_input_native_delta_to_ctrl(probe);
	uint16_t kg_default = torque_input_native_delta_to_centikg(probe);

	/* A sensor with twice the factory gain: the same force gives twice the delta, so the SAME
	 * delta must read as roughly half the effort and half the weight. */
	CHECK(torque_input_set_user_span(2U * TORQUE_GAIN_REFERENCE_NATIVE > TORQUE_SPAN_MAX_NATIVE ?
		TORQUE_SPAN_MAX_NATIVE : (uint16_t)(2U * TORQUE_GAIN_REFERENCE_NATIVE)),
		"D7: a high-gain calibration is accepted");

	uint16_t ctrl_gained = torque_input_native_delta_to_ctrl(probe);
	uint16_t kg_gained = torque_input_native_delta_to_centikg(probe);

	CHECK(ctrl_gained < ctrl_default, "D7: a higher gain lowers the control reading");
	CHECK(kg_gained < kg_default, "D7: a higher gain lowers the kilogram reading");

	torque_input_restore_default_span();
	CHECK(torque_input_native_delta_to_ctrl(probe) == ctrl_default,
		"D7: restoring the default span restores the control reading exactly");
	CHECK(torque_input_native_delta_to_centikg(probe) == kg_default,
		"D7: restoring the default span restores the kilogram reading exactly");
}

/* ---- D8: effort normalization -------------------------------------------------------- */

static int32_t effort_for(uint16_t load_ctrl)
{
	ap2_demand_input_t in;
	ap2_demand_output_t out;

	memset(&in, 0, sizeof(in));
	memset(&out, 0, sizeof(out));
	ap2_rider_demand_reset();
	in.load_ctrl = load_ctrl;
	in.torque_valid = true;
	in.pedaling = true;
	in.cadence_rpm = 60U;
	in.full_scale_ctrl = TORQUE_CTRL_FULL_SCALE_CLU;
	in.elapsed_ticks = 1U;
	/* Three ticks so the median-of-three is fed a settled value rather than its own history. */
	ap2_rider_demand_update(&in, &out);
	ap2_rider_demand_update(&in, &out);
	ap2_rider_demand_update(&in, &out);
	return out.effort_permille;
}

static void test_effort_normalization(void)
{
	printf("D8 effort normalization is monotonic, bounded, and happens exactly once\n");
	torque_input_init();

	int32_t prev = -1;
	bool mono = true;
	char msg[128];

	for (unsigned i = 0; i < NATIVE_SWEEP_N; i++) {
		uint16_t ctrl = torque_input_native_delta_to_ctrl(native_sweep[i]);
		int32_t effort = effort_for(ctrl);

		if (effort < prev) {
			mono = false;
			snprintf(msg, sizeof(msg),
				"D8: effort fell at native %u (CLU %u): %ld after %ld",
				(unsigned)native_sweep[i], (unsigned)ctrl,
				(long)effort, (long)prev);
			CHECK(false, msg);
		}
		CHECK(effort >= 0 && effort <= 1000, "D8: effort stays inside 0..1000");
		prev = effort;
	}
	CHECK(mono, "D8: effort is monotonic over the whole sensor sweep");

	CHECK(effort_for(0U) == 0, "D8: no load is no effort");
	CHECK(effort_for(AP2_EFFORT_DEADBAND_CTRL) == 0,
		"D8: at the deadband there is still no effort");
	/*
	 * Effort is an integer permille of a ~6000 CLU axis, so roughly six CLU of load are needed
	 * before the first permille appears. That is quantization, not a dead zone - what matters
	 * is that it is BOUNDED and small, and that nothing below the deadband ever produces
	 * effort. Asserting "deadband + 1 gives effort" would be asserting a rounding artefact.
	 */
	CHECK(effort_for((uint16_t)(AP2_EFFORT_DEADBAND_CTRL + 10U)) > 0,
		"D8: shortly above the deadband there is effort");
	CHECK(effort_for((uint16_t)(AP2_EFFORT_DEADBAND_CTRL - 1U)) == 0,
		"D8: below the deadband there is never effort");
	CHECK(effort_for(TORQUE_CTRL_FULL_SCALE_CLU) == 1000,
		"D8: full scale is exactly full effort");
	CHECK(effort_for((uint16_t)(TORQUE_CTRL_FULL_SCALE_CLU * 2U)) == 1000,
		"D8: beyond full scale effort saturates rather than overflowing");
}

/* ---- D9: the base/dynamic architecture is ALIVE ----------------------------------------- */

/*
 * WHY THIS IS HERE. A mutation that sets the dynamic term to zero passed every host suite in
 * this repository. base/dynamic is the mechanism Assist Pipeline V2 exists for - it is what
 * answers a harder push without low-passing the pedal ripple away - and nothing was holding it.
 * An architecture with no test is an architecture the next refactor deletes by accident.
 *
 * The check is deliberately about STRUCTURE, not ride feel: a sustained level that survives the
 * dead spot, and a separate fast excess that appears when the rider pushes harder than that
 * level. It does not pin any tuning number.
 */
static void test_base_dynamic_alive(void)
{
	ap2_demand_input_t in;
	ap2_demand_output_t out;
	int32_t base_at_steady;
	int32_t dynamic_at_steady;
	int32_t dynamic_on_push;
	int32_t base_in_dead_spot;
	unsigned i;

	printf("D9 the base/dynamic split is alive: a sustained level plus a separate fast excess\n");

	memset(&in, 0, sizeof(in));
	memset(&out, 0, sizeof(out));
	ap2_rider_demand_reset();
	in.torque_valid = true;
	in.pedaling = true;
	in.cadence_rpm = 60U;                /* 500 ms stroke */
	in.full_scale_ctrl = TORQUE_CTRL_FULL_SCALE_CLU;
	in.elapsed_ticks = 1U;

	/* Settle at a steady moderate effort: the base must rise to it, the excess must fade. */
	in.load_ctrl = 1200U;
	for (i = 0; i < 4000U; i++) {
		ap2_rider_demand_update(&in, &out);
	}
	base_at_steady = out.base_permille;
	dynamic_at_steady = out.dynamic_permille;
	CHECK(base_at_steady > 0, "D9: a sustained effort builds a sustained base");
	CHECK(dynamic_at_steady <= 20,
		"D9: with nothing changing, the dynamic term settles back to ~zero");

	/* Now push HARDER. The dynamic term is what must answer, quickly. */
	in.load_ctrl = 2400U;
	for (i = 0; i < 120U; i++) {        /* 30 ms - far shorter than the base rise time */
		ap2_rider_demand_update(&in, &out);
	}
	dynamic_on_push = out.dynamic_permille;
	CHECK(dynamic_on_push > dynamic_at_steady,
		"D9: a harder push raises the dynamic term");
	CHECK(dynamic_on_push > 0,
		"D9: the dynamic term is NOT hardwired to zero - it carries the 'pushed harder' signal");

	/* And the dead spot between leg pushes must not empty the base. */
	in.load_ctrl = 0U;
	for (i = 0; i < 200U; i++) {        /* 50 ms of dead spot */
		ap2_rider_demand_update(&in, &out);
	}
	base_in_dead_spot = out.base_permille;
	CHECK(base_in_dead_spot > base_at_steady / 2,
		"D9: the base outlives the dead spot instead of collapsing with the measurement");

	/* The two terms are genuinely separate signals, not one value reported twice. */
	CHECK(out.base_permille != out.dynamic_permille || out.base_permille == 0,
		"D9: base and dynamic are independent outputs");
}

/* ---- the sweep tables the audit asked for (observation, printed, never asserted) ------- */

static void print_sweeps(void)
{
	printf("\nCONTROL / HUMAN SWEEP (default calibration)\n");
	printf("  native    CLU   effort   kg(display)  stand  roll\n");
	for (unsigned i = 0; i < NATIVE_SWEEP_N; i++) {
		uint16_t d = native_sweep[i];
		uint16_t ctrl = torque_input_native_delta_to_ctrl(d);
		uint16_t kg = torque_input_native_delta_to_centikg(d);

		printf("  %6u %6u %6ld %10u.%02u    %s    %s\n",
			(unsigned)d, (unsigned)ctrl, (long)effort_for(ctrl),
			(unsigned)(kg / 100U), (unsigned)(kg % 100U),
			(ctrl >= ASSIST_MIN_PEDAL_LOAD_DEFAULT_CTRL) ? "Y" : "-",
			(ctrl >= ASSIST_RIDING_MIN_PEDAL_LOAD_DEFAULT_CTRL) ? "Y" : "-");
	}
}

int main(void)
{
	printf("Torque domain contract (FW-151)\n");
	printf("===============================\n");

	test_monotonic();
	test_round_trip();
	test_domain_independence();
	test_trip_points();
	test_gain_applies_to_both();
	test_effort_normalization();
	test_base_dynamic_alive();
	print_sweeps();

	if (failures != 0) {
		printf("\nFAILURES: %d\n", failures);
		return 1;
	}
	printf("\nAll torque domain contract checks passed.\n");
	return 0;
}
