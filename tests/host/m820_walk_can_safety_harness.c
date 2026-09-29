/*
 * M820 WALK / CAN / POSITION-CALIBRATION SAFETY HARNESS - the main.c half of the uncontrolled-Iq
 * fix. Built and run ONLY by tests/test_m820_walk_can_safety.py, which extracts the CURRENT
 * production text of these blocks into the generated *.inc files below and fails if any
 * anchor is missing:
 *
 *   src/main.c        comm_watchdog_step() + can_rx_consume_liveness_events()
 *                     the Walk block (PA4 debounce .. ui8_wa_level_prev), walk_assist_iq_request()
 *                     hall_calibration_standstill_confirmed/_request/_service, autodetect(),
 *                     hall_calibration_iq_request(), hall_calibration_bridge_off() .. _abort()
 *                     the phase-2 abort statement of reg_ADC_processing()
 *   src/CAN_Display.c the 0x6300 level / Walk-code decoder
 *
 * They are linked with the production ride_control -> G53 pipeline -> ap2_limits -> 16 kHz owner
 * and the production PAS front-end. Only hardware is stubbed (timers, delays, CAN transmit, the
 * EEPROM writer - which counts), plus the main-loop glue documented at each use ([GLUE]).
 *
 *   W1  same-level CAN recovery for low / middle / high / OFF: the cut is an inhibit, the level
 *       survives, the same code needs no toggle, no Iq without pedal input, pedalling resumes.
 *   W2  Walk + comms loss: request, debounce and latch cancelled, COMM_INHIBIT final owner, exact
 *       zero in the same update; recovery keeps level L and needs a PA4 release / re-press.
 *   W3  Walk -> normal keeps the stored level.
 *   W4  position calibration: refused under the inhibit; comms loss in phase 1 or phase 2 aborts
 *       with exact zero, no EEPROM write, RAM result restored and no resume. Positive control: a
 *       converged phase 2 does write, so the harness can see a write.
 */
#include "check.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "config.h"
#include "FOC.h"
#include "assist_modes.h"
#include "assist_pipeline.h"
#include "can_rx_queue.h"
#include "fast_iq_slew.h"
#include "motor_core.h"
#include "motor_service.h"
#include "pas_direction.h"
#include "pas_liveness.h"
#include "pas_sampler.h"
#include "quiet_zero.h"
#include "ride_control.h"
#include "rider_input.h"
#include "rotor_angle.h"
#include "tuning_config.h"
#include "torque_input.h"
#include "walk_assist_motor.h"

/* ---------------- main.c / CAN_Display.c globals, same names and types ---------------- */
MotorState_t MS;
MotorParams_t MP;
uint16_t adc_value[9];
PI_control_t PI_iq, PI_id;
uint8_t wa_engaged = 0;
walk_motor_output_t wa_diag;
uint8_t ui8_wa_latch_active = 0, ui8_wa_latch_cancel_block = 0, ui8_wa_hold_armed = 0;
uint8_t ui8_wa_btn_prev = 0, ui8_wa_up_prev = 0, ui8_wa_down_prev = 0, ui8_wa_light_prev = 0;
uint8_t ui8_wa_level_prev = 0;
uint32_t ui32_wa_latch_ticks = 0;
uint16_t wa_activation_reason = 0;
uint8_t wa_gates = 0;
uint16_t wa_hold_ticks = 0;
uint16_t ui16_timertics = 0;
uint8_t ui8_walk_btn_counter = 0, ui8_walk_btn_state = 0, ui8_wa_speed_paused = 0;
uint8_t ui8_wa_comm_block = 0;
uint8_t torque_fault = 0;
volatile uint16_t ui16_erps = 0, ui16_erps_counter = 0;
static uint8_t bank_toggle_pending = 0;
static uint8_t wa_bank_switch_locked = 0;
static void apply_bank_toggle(void) {}
static const uint8_t level_to_array_element[10] = { 0, 1, 1, 2, 2, 3, 3, 4, 4, 5 };
volatile uint16_t hmi_lost_ticks = 0, bus_lost_ticks = 0;
volatile uint8_t hmi_seen = 0, bus_seen = 0;
volatile uint8_t comm_inhibit = 0;
volatile uint8_t can_rx_event_flags;
uint8_t level_code, level_code_old, level_counter;
uint8_t walk_can_counter, walk_can_release_counter;
/* position calibration */
q31_t q31_rotorposition_absolute = 0;
const int32_t one_deg = 11930465;
uint8_t ui8_hall_state = 1, ui8_hall_state_old = 1, ui8_hall_case = 0;
int32_t i32_hall_order = 1;
int32_t Hall_13 = 1001, Hall_32 = 1002, Hall_26 = 1003, Hall_64 = 1004, Hall_45 = 1005, Hall_51 = 1006;
uint32_t uint32_tics_filtered = 128000;
int8_t i8_recent_rotor_direction = 1;
uint8_t ui_8_PWM_ON_Flag = 0;
uint8_t pwm_cutoff_active = 0;
uint16_t pwm_cutoff_tick = 0;
uint16_t uint16_half_rotation_counter = 0;
static quiet_zero_t quiet_zero_state;
static rotor_angle_state_t rotor_angle_state;
static uint32_t autodetect_standstill_ticks = 0xFFFFFFFFU;
q31_t temp6;
uint16_t p = 0;
uint32_t timeout = 0xFFFF;
uint8_t transmit_mailbox = 0;

/* ---------------- hardware stubs ---------------- */
#ifndef ENABLE
#define ENABLE 1U
#endif
#define CAN0 0U
#define CAN_TRANSMIT_OK 1
#define CAN_FT_DATA 0U
#define CAN_FF_EXTENDED 1U
static struct { uint8_t tx_data[8]; uint32_t tx_sfid, tx_efid; uint8_t tx_ft, tx_ff, tx_dlen; } hx_tx;
#define transmit_message hx_tx
static uint8_t can_message_transmit(uint32_t periph, void *msg) { (void)periph; (void)msg; return 0U; }
static int can_transmit_states(uint32_t periph, uint8_t box) { (void)periph; (void)box; return CAN_TRANSMIT_OK; }
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))

static bool pwm_enabled;
void timer_primary_output_config(uint32_t timer_periph, uint32_t newvalue) { (void)timer_periph; pwm_enabled = newvalue != 0U; }
void timer_channel_output_pulse_value_config(uint32_t t, uint16_t c, uint32_t v) { (void)t; (void)c; (void)v; }
void foc_aw_tracking_reset(void) {}
void quiet_zero_reset(quiet_zero_t *qz) { (void)qz; }
void rotor_angle_reset(rotor_angle_state_t *s) { (void)s; }
void foc_current_feedback_invalidate(void) {}
void fwdgt_counter_reload(void);
void fwdgt_counter_reload(void) {}
bool sendCAN_Poll(MotorParams_t *mp, MotorState_t *s, uint16_t command);
bool sendCAN_Poll(MotorParams_t *mp, MotorState_t *s, uint16_t command) { (void)mp; (void)s; (void)command; return true; }
void sendCAN_status_broadcast(MotorState_t *s);
void sendCAN_status_broadcast(MotorState_t *s) { (void)s; }

static unsigned eeprom_writes;
static int32_t eeprom_hall64, eeprom_angle;
void write_virtual_eeprom(void) { eeprom_writes++; eeprom_hall64 = Hall_64; eeprom_angle = MP.angle_correction; }

/* The CAN RX ISR, simulated: while the bus is alive the display frames raise the physical
 * liveness events (CAN_Display.c's decoder runs separately, from hmi_frame()). */
static bool bus_alive = true;
static uint32_t sim_ms;
/* delay_1ms() is only reached from autodetect() phase 1: simulated time, bus traffic, and a
 * rotor that crosses a Hall edge every 50 ms so phase 1 really overwrites the Hall table. */
void delay_1ms(uint32_t count)
{
	for (uint32_t k = 0; k < count; k++) {
		sim_ms++;
		if (bus_alive && (sim_ms % 100U) == 0U) can_rx_event_flags |= CAN_RX_EVENT_BUS | CAN_RX_EVENT_HMI;
		if ((sim_ms % 50U) == 0U) {
			ui8_hall_state = (uint8_t)(ui8_hall_state % 6U + 1U);
			ui8_hall_case = 64U;
		}
	}
}

/* ---------------- the production main.c / CAN_Display.c text ---------------- */
static void hall_calibration_snapshot(void);
static void hall_calibration_restore(void);
static void hall_calibration_bridge_off(void);
static void hall_calibration_abort(void);
#include "x_defines.inc"          /* #define AUTODETECT_STANDSTILL_* */
#include "x_standstill.inc"       /* hall_calibration_standstill_confirmed() */
#include "x_watchdog.inc"         /* can_rx_consume_liveness_events() + comm_watchdog_step() */
#include "x_hall_request.inc"     /* hall_calibration_pending .. hall_calibration_service() */
#include "x_autodetect.inc"       /* autodetect() */
#include "x_walk_iq.inc"          /* walk_assist_iq_request() */
#include "x_hall_iq.inc"          /* hall_calibration_iq_request() */
#include "x_hall_exit.inc"        /* hall_calibration_bridge_off() .. hall_calibration_abort() */

static void walk_block(void)
{
#include "x_walk_block.inc"
}

static void can_6300(MotorState_t *MSp, const uint8_t d[8])
{
#define MS MSp
	struct { uint16_t command; } Ext_ID_Rx = { 0x6300 };
	struct { uint8_t rx_data[8]; } receive_message;
	memcpy(receive_message.rx_data, d, 8);
#include "x_can6300.inc"
#undef MS
}

/* ---------------- main-loop glue ---------------- */
static fast_iq_slew_mailbox_t *mb;
static const uint8_t PAS_FWD[4] = { 0U, 2U, 3U, 1U };
static struct {
	uint32_t now, slow;
	double pas_acc;
	uint8_t phase, start_phase, fwd_since_stop;
	uint16_t last_fwd_period, gaps[4], hall_age;
	bool real_stop, forward_valid;
	int16_t crank_rpm;
	uint16_t load;
	int hmi_code;          /* -1: display silent (with the bus) */
} G;

static void hmi_frame(uint8_t code)
{
	uint8_t d[8] = { 0 };
	d[1] = code;
	can_6300(&MS, d);
}

static void power_on(void)
{
	memset(&G, 0, sizeof(G));
	G.last_fwd_period = PAS_STOP_TICKS;
	G.hmi_code = -1;
	memset(&MS, 0, sizeof(MS)); memset(&MP, 0, sizeof(MP));
	MS.assist_level = 2;                         /* main.c boot default */
	MS.pushassist_flag = RESET; MS.walk_can_request = RESET;
	memset(adc_value, 0, sizeof(adc_value)); adc_value[5] = 4095U;   /* PA4 idle */
	ui8_wa_latch_active = ui8_wa_latch_cancel_block = ui8_wa_hold_armed = 0;
	ui8_wa_btn_prev = ui8_wa_up_prev = ui8_wa_down_prev = ui8_wa_light_prev = ui8_wa_level_prev = 0;
	ui32_wa_latch_ticks = 0; ui8_walk_btn_counter = ui8_walk_btn_state = ui8_wa_speed_paused = 0;
	ui8_wa_comm_block = 0; wa_engaged = 0; torque_fault = 0;
	hmi_lost_ticks = bus_lost_ticks = 0; hmi_seen = bus_seen = 0; comm_inhibit = 0;
	can_rx_event_flags = 0; level_code = level_code_old = level_counter = 0;
	walk_can_counter = walk_can_release_counter = 0;
	MS.hall_angle_detect_flag = 1;
	MP.angle_correction = 71582790;
	MP.reverse = -1;                             /* this drive: forward is negative Park q */
	Hall_13 = 1001; Hall_32 = 1002; Hall_26 = 1003; Hall_64 = 1004; Hall_45 = 1005; Hall_51 = 1006;
	i32_hall_order = 1; eeprom_writes = 0; bus_alive = true; p = 0; temp6 = 0;
	torque_input_init();
	assist_modes_init(); assist_modes_set_active_bank(0U);
	motor_core_init(&MS); ride_control_init();
	pas_sampler_init(0U); pas_direction_init(); pas_liveness_init();
	walk_motor_reset(); walk_motor_release();
	mb = ride_control_final_iq_slew_mailbox();
}

/* One 4 kHz control iteration (reg_ADC_processing order) plus the 40 ms slow loop. */
static void tick(void)
{
	G.now++;
	/* [GLUE] display traffic at 10 Hz while it is alive; the IRQ raises the liveness events */
	if (G.hmi_code >= 0 && bus_alive && (G.now % 400U) == 0U) {
		hmi_frame((uint8_t)G.hmi_code);
		can_rx_event_flags |= CAN_RX_EVENT_BUS | CAN_RX_EVENT_HMI;
	}
	/* [GLUE] PAS front-end, as in m820_uncontrolled_iq_safety_host.c */
	const int r = G.crank_rpm;
	G.pas_acc += (double)(r < 0 ? -r : r) * (double)PAS_TRANSITIONS_PER_REV / 60.0 / 4000.0;
	while (G.pas_acc >= 1.0) { G.pas_acc -= 1.0; G.phase = (uint8_t)((G.phase + (r > 0 ? 1U : 3U)) & 3U); }
	pas_sampler_isr_tick(PAS_FWD[G.phase], G.now);
	pas_step_event_t ev;
	uint8_t cadence = MS.cadence;
	while (pas_sampler_pop(&ev)) {
		pas_direction_on_step(ev.step);
		if (ev.step > 0) {
			if (cadence == 0U && !G.start_phase && pas_direction_fwd_run() >= START_PHASE_STEPS) G.start_phase = 1U;
			G.gaps[G.fwd_since_stop & 3U] = ev.gap;
			if (G.fwd_since_stop < 255U) G.fwd_since_stop++;
			G.last_fwd_period = ev.gap;
			if (G.fwd_since_stop > PAS_STEPS_PER_PULSE && (G.fwd_since_stop % PAS_STEPS_PER_PULSE) == 1U) {
				const uint32_t g = (uint32_t)G.gaps[0] + G.gaps[1] + G.gaps[2] + G.gaps[3];
				cadence = (uint8_t)(g ? (60U * 4000U * PAS_STEPS_PER_PULSE) / (PAS_TRANSITIONS_PER_REV * g) : 0U);
				G.start_phase = 0U;
			}
		} else G.fwd_since_stop = 0U;
	}
	uint32_t idle = G.now - pas_sampler_last_transition_tick(); if (idle > 64000U) idle = 64000U;
	uint32_t to = (uint32_t)G.last_fwd_period * 2U;
	if (to < PAS_STOP_TICKS) to = PAS_STOP_TICKS; else if (to > PAS_STOP_TICKS_MAX) to = PAS_STOP_TICKS_MAX;
	pas_liveness_update(idle, (uint16_t)to);
	G.real_stop = pas_liveness_stopped();
	if (G.real_stop) { cadence = 0U; G.start_phase = 0U; G.fwd_since_stop = 0U; pas_direction_on_stop(); }
	MS.cadence = cadence;
	G.forward_valid = (cadence > 0U || G.start_phase) && !G.real_stop;
	rider_input_t ri; memset(&ri, 0, sizeof(ri));
	ri.torque_load_ctrl = G.load; ri.torque_load_centikg = G.load; ri.cadence_rpm = cadence;
	ri.pas_forward = G.forward_valid; ri.pedaling_active = G.forward_valid;
	ri.crank_forward_steps = pas_direction_fwd_run(); ri.crank_direction_ok = G.forward_valid;
	ri.real_stop = G.real_stop; ri.direction_inhibit_active = pas_direction_direction_inhibit_active();
	ri.start_phase = G.start_phase != 0U; ri.torque_sensor_valid = true;
	ri.pas_sensor_valid = pas_sampler_seeded() != 0U;
	rider_input_update(&ri);
	/* [GLUE] crude motor/Hall response to the reference, for Walk only */
	if (MS.i_q_setpoint > 0) { ui16_erps = (uint16_t)(MS.i_q_setpoint / 4 + 1); G.hall_age = 0; ui16_timertics = (uint16_t)(500000U / (6U * ui16_erps)); }
	else { if (G.hall_age < 0xFFFFU) G.hall_age++; ui16_erps = 0U; }
	ui16_erps_counter = G.hall_age;
	MS.i_q = MS.i_q_setpoint;
	/* [GLUE] the bridge-on path of the main loop, which phase 2 relies on for ui_8_PWM_ON_Flag */
	if (MS.i_q_setpoint > 0) { ui_8_PWM_ON_Flag = 1U; pwm_enabled = true; }

	walk_block();
#include "x_phase2_abort.inc"
	ride_control_input_t ci; memset(&ci, 0, sizeof(ci));
	ci.pas_ab = pas_sampler_state(); ci.speed_x100 = MS.Speedx100; ci.cadence_rpm = cadence;
	ci.assist_level_index = level_to_array_element[MS.assist_level > 9 ? 0 : MS.assist_level];
	ci.battery_voltage_mv = 42000U; ci.iq_scale = PH_CURRENT_MAX; ci.ride_core_iq_limit = PH_CURRENT_MAX;
	ci.phase_current_max = PH_CURRENT_MAX; ci.battery_current_max = BATTERYCURRENT_MAX;
	ci.cal_i = CAL_I; ci.current_iq = MS.i_q_setpoint;
	ci.voltage_raw = (uint16_t)(42000 / CAL_BAT_V); ci.voltage_min_raw = VOLTAGE_MIN;
	ci.controller_temperature_c = 25; ci.speed_limit_x100 = SPEEDLIMIT; ci.legal_enabled = true;
	ci.walk_active = MS.pushassist_flag != RESET;
	ci.position_calibration_active = MS.hall_angle_detect_flag > 1;
	ci.comm_inhibit = comm_inhibit != 0U;
	ci.start_phase = G.start_phase != 0U; ci.elapsed_ticks = 1U;
	ride_control_update(&ci);
	for (int k = 0; k < 4; k++) fast_iq_slew_tick(mb, &MS.i_q_setpoint);
	/* [GLUE] the slow loop runs after reg_ADC_processing(); p++ is its unconditional counter */
	if (++G.slow >= 160U) { G.slow = 0U; comm_watchdog_step(); p++; }
}

static void run(uint32_t ticks) { for (uint32_t k = 0; k < ticks; k++) tick(); }
#define SEC(s) ((uint32_t)((s) * 4000.0))

static void start_pedalling(void) { G.load = 2000U; G.crank_rpm = 70; }
static void stop_pedalling(void) { G.load = 0U; G.crank_rpm = 0; }

/* ------------------------------------------------------------------------------------------- */
static void w1_same_level_recovery(void)
{
	static const struct { uint8_t code, level; const char *name; } L[] = {
		{ 0x01, 1, "low (HMI 1)" }, { 0x0C, 3, "middle (HMI 3)" }, { 0x03, 9, "high (HMI 9)" }, { 0x00, 0, "OFF" } };
	for (size_t i = 0; i < sizeof(L) / sizeof(L[0]); i++) {
		power_on();
		MS.Speedx100 = 1500U;
		G.hmi_code = L[i].code;
		run(SEC(2.0));
		CHECK(MS.assist_level == L[i].level, "W1: the HMI level code is accepted before the test");
		start_pedalling(); run(SEC(3.0));
		const int32_t before = MS.i_q_setpoint;
		CHECK(L[i].level == 0 ? before == 0 : before > 0, "W1: assist works at the selected level (OFF: none)");
		/* the display and the bus go silent */
		bus_alive = false; G.hmi_code = -1;
		uint32_t cut_at = 0; bool same_update = false;
		for (uint32_t k = 0; k < SEC(4.0); k++) {
			const uint8_t was = comm_inhibit;
			tick();
			if (!was && comm_inhibit) {
				cut_at = k;
				/* the watchdog runs after this iteration's publication; the next iteration is the
				 * first one that sees the inhibit - it must be the zero */
				tick();
				same_update = mb->target == 0 && mb->mode == (uint32_t)FIS_MODE_FORCE_ZERO &&
					mb->iq_ceiling == 0 && MS.i_q_setpoint == 0;
			}
		}
		printf("  %s: comm_inhibit after %.2f s\n", L[i].name, (double)cut_at / 4000.0);
		CHECK(comm_inhibit == 1U, "W1: comm_inhibit asserts after COMM_CUT_TICKS of silence");
		CHECK(same_update, "W1: the first update under the inhibit is FORCE_ZERO, target 0, ceiling 0, Iq 0");
		CHECK(MS.i_q_setpoint == 0 && MS.assist_level == L[i].level,
			"W1: Iq 0 under the inhibit and the selected level is NOT destroyed");
		/* the display returns with the SAME code; the rider is still pedalling */
		bus_alive = true; G.hmi_code = L[i].code;
		bool zero_through = true;
		for (uint32_t k = 0; k < SEC(0.5); k++) { tick(); if (MS.i_q_setpoint != 0) zero_through = false; }
		CHECK(comm_inhibit == 0U, "W1: the inhibit clears by itself when the bus returns");
		CHECK(MS.assist_level == L[i].level, "W1: the same level code leaves the selected level in force - no toggle");
		CHECK(zero_through, "W1: recovery mid-pedal releases no step of demand");
		stop_pedalling(); run(SEC(1.5));
		bool idle_zero = true;
		for (uint32_t k = 0; k < SEC(2.0); k++) { tick(); if (MS.i_q_setpoint != 0) idle_zero = false; }
		CHECK(idle_zero, "W1: without pedal input Iq stays zero after recovery");
		start_pedalling();
		int32_t resumed = 0;
		for (uint32_t k = 0; k < SEC(3.0); k++) { tick(); if (MS.i_q_setpoint > resumed) resumed = MS.i_q_setpoint; }
		printf("  %s: assist after recovery max Iq=%d\n", L[i].name, resumed);
		CHECK(L[i].level == 0 ? resumed == 0 : resumed > 0,
			"W1: a valid pedal start after recovery resumes assist at the kept level (OFF: none)");
	}
}

static void enter_walk(uint8_t normal_code)
{
	MS.Speedx100 = 0U;
	G.hmi_code = normal_code; run(SEC(2.0));
	G.hmi_code = 0x06; run(SEC(1.0));
	adc_value[5] = 3300U; run(SEC(2.0));              /* PA4 pressed */
	MS.Speedx100 = 400U; run(SEC(1.0));
}

/* The optional Walk latch is a bank setting: set it through the production wire format + CRC. */
static bool set_walk_latch(bool on)
{
	uint8_t blob[ASSIST_BANK_BLOB_LEN];
	const uint16_t len = assist_modes_serialize_bank(0U, blob);
	if (len != ASSIST_BANK_BLOB_LEN) return false;
	blob[10] = on ? 1U : 0U;
	uint16_t c = 0xFFFFU;
	for (uint16_t i = 0; i < (uint16_t)(len - 2U); i++) {
		c ^= (uint16_t)blob[i] << 8;
		for (int k = 0; k < 8; k++) c = (c & 0x8000U) ? (uint16_t)((c << 1) ^ 0x1021U) : (uint16_t)(c << 1);
	}
	blob[len - 2U] = (uint8_t)(c & 0xFFU); blob[len - 1U] = (uint8_t)(c >> 8);
	return assist_modes_apply_bank_blob(blob, len) && assist_modes_get_wa_latch_after_release() == on;
}

static void w2_walk_comm_loss(int latch)
{
	printf("  latch %s\n", latch ? "enabled" : "disabled");
	power_on();
	CHECK(set_walk_latch(latch != 0), "W2: Walk latch bank setting applied");
	enter_walk(0x0C);
	CHECK(MS.pushassist_flag == SET && MS.i_q_setpoint > 0, "W2: Walk is active and pulling before the loss");
	if (latch) { adc_value[5] = 4095U; run(SEC(0.5)); CHECK(ui8_wa_latch_active == 1U && MS.pushassist_flag == SET,
		"W2: the latch holds Walk after PA4 release (fixture)"); }
	bus_alive = false; G.hmi_code = -1;
	bool cut = false, ok = false;
	for (uint32_t k = 0; k < SEC(4.0) && !cut; k++) {
		const uint8_t was = comm_inhibit;
		tick();
		if (!was && comm_inhibit) {
			cut = true;
			tick();      /* the first iteration that sees the inhibit */
			ok = MS.walk_can_request == RESET && walk_can_counter == 0U && ui8_wa_latch_active == 0U &&
			     MS.pushassist_flag == RESET && mb->target == 0 && mb->iq_ceiling == 0 &&
			     mb->mode == (uint32_t)FIS_MODE_FORCE_ZERO && MS.i_q_setpoint == 0;
		}
	}
	CHECK(cut && ok, "W2: comms loss cancels request, debounce and latch; COMM_INHIBIT owns: target 0, ceiling 0, FORCE_ZERO, Iq 0");
	bool held_zero = true;
	for (uint32_t k = 0; k < SEC(2.0); k++) { tick(); if (MS.i_q_setpoint != 0 || MS.pushassist_flag == SET) held_zero = false; }
	CHECK(held_zero, "W2: no Walk owner publishes Iq for as long as the inhibit lasts");
	/* the display returns, still in Walk; PA4 is (still or again) held */
	adc_value[5] = 3300U;
	bus_alive = true; G.hmi_code = 0x06;
	bool no_rearm = true;
	for (uint32_t k = 0; k < SEC(2.0); k++) { tick(); if (MS.pushassist_flag == SET || MS.i_q_setpoint != 0) no_rearm = false; }
	CHECK(comm_inhibit == 0U && no_rearm, "W2: a held/stale PA4 does not re-arm Walk after recovery");
	adc_value[5] = 4095U; run(SEC(0.5));
	adc_value[5] = 3300U; run(SEC(1.0));
	CHECK(MS.pushassist_flag == SET, "W2: a PA4 release and re-press re-arms Walk");
	adc_value[5] = 4095U; G.hmi_code = 0x0C; run(SEC(1.0));
	CHECK(MS.assist_level == 3, "W2: the stored normal level L survives Walk and the comms loss");
}

static void w3_walk_keeps_level(void)
{
	power_on();
	enter_walk(0x0D);
	CHECK(MS.assist_level == 4 && MS.pushassist_flag == SET, "W3: Walk active with normal level 4 stored");
	adc_value[5] = 4095U; G.hmi_code = 0x0D; run(SEC(1.0));
	CHECK(MS.pushassist_flag == RESET && MS.assist_level == 4, "W3: Walk -> normal keeps the previous level");
}

static void w4_position_calibration(void)
{
	/* refused while inhibited */
	power_on();
	G.hmi_code = 0x0C; run(SEC(1.0));
	bus_alive = false; G.hmi_code = -1; run(SEC(3.5));
	CHECK(comm_inhibit == 1U && !hall_calibration_request(), "W4: a calibration request is refused under the inhibit");

	/* positive control: a converged phase 2 writes the EEPROM once */
	power_on();
	G.hmi_code = 0x0C; run(SEC(1.0));
	CHECK(hall_calibration_request(), "W4: calibration request accepted at standstill with comms alive");
	hall_calibration_service();                     /* phase 1, blocking */
	CHECK(MS.hall_angle_detect_flag == 2 && Hall_64 != 1004, "W4: phase 1 ran and overwrote the Hall table (RAM)");
	MS.u_d = 0;
	for (uint32_t k = 0; k < SEC(10.0) && MS.hall_angle_detect_flag == 2; k++) tick();
	CHECK(MS.hall_angle_detect_flag == 1 && eeprom_writes == 1U,
		"W4: positive control - a converged phase 2 is stored exactly once");

	/* comms loss in phase 2: abort, restore, no write, no resume */
	power_on();
	G.hmi_code = 0x0C; run(SEC(1.0));
	CHECK(hall_calibration_request(), "W4: calibration request accepted");
	hall_calibration_service();
	CHECK(MS.hall_angle_detect_flag == 2, "W4: phase 2 running");
	MS.u_d = -4000;                                 /* keep phase 2 adjusting the angle */
	int32_t cal_iq_seen = 0;
	/* phase 2 steps once p > 70 slow-loop periods (~2.84 s) */
	for (uint32_t k = 0; k < SEC(4.0); k++) { tick(); if (MS.i_q_setpoint > cal_iq_seen) cal_iq_seen = MS.i_q_setpoint; }
	CHECK(cal_iq_seen > 0 && MP.angle_correction != 71582790,
		"W4: phase 2 drives its probe Iq and moves angle_correction (fixture)");
	bus_alive = false; G.hmi_code = -1;
	bool cut = false, zero_now = false;
	for (uint32_t k = 0; k < SEC(4.0) && !cut; k++) {
		const uint8_t was = comm_inhibit;
		tick();
		if (!was && comm_inhibit) {
			cut = true;
			tick();
			zero_now = MS.hall_angle_detect_flag == 1 && mb->target == 0 &&
				mb->mode == (uint32_t)FIS_MODE_FORCE_ZERO && mb->iq_ceiling == 0 && MS.i_q_setpoint == 0 &&
				!pwm_enabled;
		}
	}
	CHECK(cut && zero_now, "W4: comms loss in phase 2 -> service mode left, FORCE_ZERO, Iq 0, bridge off, same update");
	CHECK(eeprom_writes == 0U, "W4: no EEPROM write on the phase-2 abort");
	CHECK(Hall_64 == 1004 && MP.angle_correction == 71582790 && i32_hall_order == 1,
		"W4: the unverified RAM result is restored, so no later save can persist it");
	bus_alive = true; G.hmi_code = 0x0C;
	bool no_resume = true;
	for (uint32_t k = 0; k < SEC(3.0); k++) { tick(); if (MS.hall_angle_detect_flag != 1 || MS.i_q_setpoint != 0) no_resume = false; }
	CHECK(no_resume && eeprom_writes == 0U, "W4: calibration does not resume or overwrite the zero after recovery");

	/* comms loss during the blocking phase 1 */
	power_on();
	G.hmi_code = 0x0C; run(SEC(1.0));
	CHECK(hall_calibration_request(), "W4: calibration request accepted");
	bus_alive = false; G.hmi_code = -1;          /* the bus dies as phase 1 starts */
	const uint32_t t0 = sim_ms;
	hall_calibration_service();
	printf("  phase-1 abort after %u ms of the 5.4 s procedure\n", sim_ms - t0);
	CHECK(comm_inhibit == 1U && MS.hall_angle_detect_flag == 1 && !pwm_enabled && sim_ms - t0 < 5000U,
		"W4: comms loss in phase 1 aborts the open-loop drive early, bridge off, no phase 2");
	CHECK(eeprom_writes == 0U && Hall_64 == 1004 && i32_hall_order == 1 && MP.angle_correction == 71582790,
		"W4: no EEPROM write and the previous Hall table/angle are restored after a phase-1 abort");
	run(SEC(1.0));
	CHECK(MS.i_q_setpoint == 0 && MS.hall_angle_detect_flag == 1, "W4: nothing pulls after the phase-1 abort");
}

int main(void)
{
	puts("M820 Walk / CAN / position-calibration safety (production main.c + CAN_Display.c text)");
	puts("W1 same-level CAN recovery"); w1_same_level_recovery();
	puts("W2 Walk + comms loss"); w2_walk_comm_loss(0); w2_walk_comm_loss(1);
	puts("W3 Walk -> normal keeps the level"); w3_walk_keeps_level();
	puts("W4 position calibration under comms loss"); w4_position_calibration();
	if (host_test_failures == 0) { puts("M820 Walk/CAN/calibration safety: ALL CHECKS PASSED"); return 0; }
	printf("M820 Walk/CAN/calibration safety: %d CHECK(S) FAILED\n", host_test_failures);
	return 1;
}
