/*
 * TASK-EVD-TQ-06-G1 / ADR-013 D1: hard battery-overcurrent trip (src/battery_trip.c) and its
 * production wiring in main.c / ride_control.c (source guards, main.c cannot be linked).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "battery_trip.h"
#include "check.h"

#define STR2(x) #x
#define STRINGIZE(x) STR2(x)

static char *read_whole_file(const char *path)
{
	FILE *f = fopen(path, "rb");
	long n;
	char *buf;
	if (!f) return NULL;
	fseek(f, 0, SEEK_END);
	n = ftell(f);
	fseek(f, 0, SEEK_SET);
	buf = (char *)malloc((size_t)n + 1U);
	if (buf && fread(buf, 1, (size_t)n, f) != (size_t)n) { free(buf); buf = NULL; }
	if (buf) buf[n] = '\0';
	fclose(f);
	return buf;
}

int main(void)
{
	int i;
	int edges;

	/* T1: exactly BATTERY_TRIP_SAMPLES consecutive samples above the threshold latch, 15 do not */
	battery_trip_init();
	battery_trip_set_threshold_counts(1000);
	for (i = 0; i < (int)BATTERY_TRIP_SAMPLES - 1; i++) (void)battery_trip_sample(1001);
	CHECK(!battery_trip_latched(), "T1a. 15 consecutive samples above the threshold do not trip");
	(void)battery_trip_sample(999);
	for (i = 0; i < (int)BATTERY_TRIP_SAMPLES - 1; i++) (void)battery_trip_sample(5000);
	CHECK(!battery_trip_latched(), "T1b. one sample at/below the threshold restarts the count");
	CHECK(battery_trip_sample(5000) && battery_trip_latched(), "T1c. the 16th consecutive sample latches and reports the edge");
	CHECK(battery_trip_count() == 1U, "T1d. the trip is counted");

	/* T2: equal to the threshold is not above it */
	battery_trip_init();
	battery_trip_set_threshold_counts(1000);
	for (i = 0; i < 100; i++) (void)battery_trip_sample(1000);
	CHECK(!battery_trip_latched(), "T2. a value equal to the threshold never trips");

	/* T3: the edge is reported once; the latch holds while current continues */
	battery_trip_init();
	battery_trip_set_threshold_counts(10);
	edges = 0;
	for (i = 0; i < 200; i++) edges += battery_trip_sample(100) ? 1 : 0;
	CHECK(edges == 1 && battery_trip_count() == 1U, "T3. one latching edge for one continuous overcurrent");

	/* T4: re-arm only after standstill held for BATTERY_TRIP_REARM_MS */
	battery_trip_service(false, 1000U);
	CHECK(battery_trip_latched(), "T4a. moving: the latch holds");
	battery_trip_service(true, 40U);
	battery_trip_service(true, 40U);
	CHECK(battery_trip_latched(), "T4b. 80 ms of standstill is not enough");
	battery_trip_service(false, 40U);
	battery_trip_service(true, 40U);
	battery_trip_service(true, 40U);
	CHECK(battery_trip_latched(), "T4c. movement restarts the standstill timer");
	battery_trip_service(true, 40U);
	CHECK(!battery_trip_latched(), "T4d. 120 ms of uninterrupted standstill re-arms");
	for (i = 0; i < (int)BATTERY_TRIP_SAMPLES; i++) (void)battery_trip_sample(100);
	CHECK(battery_trip_latched() && battery_trip_count() == 2U, "T4e. a re-armed trip trips and counts again");

	/* T5: threshold <= 0 disables (unknown limit / unarmed sampler) */
	battery_trip_init();
	battery_trip_set_threshold_counts(0);
	for (i = 0; i < 1000; i++) (void)battery_trip_sample(30000);
	CHECK(!battery_trip_latched(), "T5. a non-positive threshold never trips");

	/* T6: production wiring (source guards) */
	{
		char *m = read_whole_file(STRINGIZE(MAIN_C_PATH));
		char *r = read_whole_file(STRINGIZE(RIDE_CONTROL_C_PATH));
		const char *isr = m ? strstr(m, "void TIMER1_IRQHandler(void)") : NULL;
		const char *s1 = isr ? strstr(isr, "battery_current_sample((uint16_t)adc_value[0],") : NULL;
		const char *s2 = s1 ? strstr(s1, "if (battery_trip_sample(battery_current_last_delta_adc())) {") : NULL;
		const char *s3 = s2 ? strstr(s2, "timer_primary_output_config(TIMER0, DISABLE);") : NULL;
		CHECK(m != NULL && r != NULL, "T6a. sources readable");
		CHECK(s2 != NULL && s3 != NULL && (s3 - s2) < 200,
		      "T6b. TIMER1 ISR: the trip sees the same sample right after the filter and clears MOE at once");
		CHECK(m && strstr(m, "if(ride_control_final_iq_requested() > 0 && !battery_trip_latched()){") != NULL,
		      "T6g. Gate A itself is closed by the latch (stale-demand window, review F-01)");
		CHECK(m && strstr(m, "if(battery_trip_latched() && MS.hall_angle_detect_flag > 1U) hall_calibration_abort();") != NULL &&
		      strstr(m, "if(battery_trip_latched()) return false;") != NULL &&
		      strstr(m, "if(battery_trip_latched()) return;") != NULL &&
		      strstr(m, "if(battery_trip_latched()) break;") != NULL,
		      "T6h. the trip refuses and aborts position calibration like the comms inhibit (review F-06)");
		CHECK(m && strstr(m, ".battery_soc_derate_q12 = (uint16_t)(G53_G1_Q12_ONE - g53_g1_soc_factor_m820(") != NULL &&
		      strstr(m, "(int32_t)(MS.soc_display*10.0f), MP.limp_soc_limit, MP.limp_soc_limit_stage2)),") != NULL,
		      "T6i. step 2: main.c feeds SOC x10 and Para1[10]/[11] in the right order (review S2-01)");
		CHECK(m && strstr(m, "thermal_factor_m820") == NULL && strstr(m, "battery_thermal_derate_q12") == NULL,
		      "T6j. variant A: no thermal input to the G53 limit - the M820 Iq derate owns temperature");
		CHECK(m && strstr(m, ".battery_trip_latched = battery_trip_latched(),") != NULL,
		      "T6c. ride_control gets the latch every control tick");
		CHECK(m && strstr(m, "battery_trip_service(MS.Speedx100==0 && MS.cadence==0, 40U);") != NULL,
		      "T6d. re-arm is evaluated in the 40 ms slow loop on wheel + cadence standstill");
		CHECK(m && strstr(m, "battery_trip_set_threshold_counts((int32_t)(((float)MP.battery_current_max +") != NULL &&
		      strstr(m, "(float)BATTERY_TRIP_MARGIN_MA) / CAL_BAT_I));") != NULL,
		      "T6e. threshold = configured limit + 15 A, same CAL_BAT_I scale");
		CHECK(r && strstr(r, "} else if (input->battery_trip_latched) {") != NULL &&
		      strstr(r, "ride_enter_owner(RIDE_OWNER_BATTERY_TRIP);") != NULL,
		      "T6f. ride_control: the latch is an exact-zero owner ahead of calibration, Walk and assist");
		free(m);
		free(r);
	}

	if (host_test_failures) {
		printf("battery trip: %d CHECK(S) FAILED\n", host_test_failures);
		return 1;
	}
	printf("battery trip: ALL CHECKS PASSED\n");
	return 0;
}
