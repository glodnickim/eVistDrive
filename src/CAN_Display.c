/*
 * CAN_Display.c
 *
 *  Created on: 19.10.2025
 *  Author: stancecoke
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "main.h"
#include "CAN_Display.h"
#include "parser.h"
#include "FOC.h"
#include "current_cal.h"
#include "pwm_geometry.h"
#include "current_feedback.h"
#include "current_sample_ctx.h"
#include "assist_pipeline.h"
#include "assist_modes.h"
#include "tuning_config.h"
#include "torque_input.h"
#include "ride_control.h"
#include "rider_input.h"
#include "level_gesture.h"
#include "pas_direction.h"
#include "can_tx_queue.h"
#include "can_multiframe.h"
#include "can_reply_effects.h"
#include "stop_trace.h"
#if CAN_DIAGNOSTICS_ENABLE
#include "qs_transition_diag.h"
#include "qs_transition_dump.h"
#endif

/* Build version string for the HMI info field (0x6001). The tracked build script
   generates build_version.h inside .build and adds that directory before inc/.
   Direct IDE builds fall back to "dev". */
#if defined(__has_include)
#  if __has_include("build_version.h")
#    include "build_version.h"
#  endif
#endif
#ifndef EBICS_BUILD_VERSION
#define EBICS_BUILD_VERSION "dev"
#endif

Ext_ID_t Ext_ID_Rx;
Ext_ID_t Ext_ID_Tx;
void processCAN_Rx(MotorParams_t* MP, MotorState_t* MS);
void sendCAN_Tx(MotorParams_t* MP, MotorState_t* MS);
void sendCAN_Poll(MotorParams_t* MP, MotorState_t* MS, uint16_t command);
void sendAcknoledge(void);
//FW-068/076: the result of a config write has to reach the tool. Used by the multiframe
//blobs and by the short 0x3203 write; a rejected frame must never read as a success.
void sendWriteResult(uint16_t command, uint8_t applied);
#if CAN_DIAGNOSTICS_ENABLE
static void send_qs_transition_status(void);
#endif
bool send_multiframe(uint16_t command, char* data, uint8_t length );
bool send_multiframe_tracked(uint16_t command, char* data, uint8_t length,
                              can_multiframe_id_t *out_id); //FW-110 v4
bool send_multiframe_trailer(uint16_t command, char* data, uint8_t length,
                              const can_multiframe_trailer_t *trailer); //FW-110 v4
void append_multiframe(uint16_t command, char* data);
void update_checksum(void);
//int16_t abs(int16_t value);
char tx_data[64];
uint8_t Para0[64];
uint8_t Para1[64];
uint8_t Para2[64];
uint8_t BankBlob[256]; //FW-006/../FW-069/FW-084: one serialized profile bank (255 B used, 32 frames)
uint8_t TuningBlob[32]; //FW-010: global ride-feel tuning blob (FW-068: 32 B used, 4 frames)
uint8_t TorqueBlob[56]; //FW-013/FW-061: torque telemetry + calibration + coast re-zero diagnostics (v2 = 56 B)
extern volatile uint8_t bank_save_request; //FW-006/FW-010: set by 0x6022, consumed in main.c at standstill
extern volatile uint8_t soc_full_persist;    //FW-018: set by 0x602B, flash-persisted in main.c at standstill

/*
 * DEPRECATED PROTOCOL FIELD — kept on the wire, gone from the firmware.
 *
 * Byte 3 of the 0x6028 status block and of the 0x6029 diagnostics block used to carry which
 * ride engine was running: 0 = Legacy, 1 = ride core. FW-030 removed the ability to select an
 * engine and FW-094 removed the last Legacy code, so this is a constant now. It stays in the
 * frames because the shipped Canable app parses both blocks positionally — dropping the byte
 * would shift every field after it and needs a coordinated block-version bump (see the audit's
 * protocol migration list).
 *
 * Do not reintroduce a variable behind this. There is one assist pipeline.
 */
#define DIAG_ENGINE_ID_RIDE_CORE 1
#if CAN_DIAGNOSTICS_ENABLE
extern volatile uint8_t diag_peak_reset, diag_peak_cadence; //FW-015b: peak-hold diagnostics
extern uint16_t pas_idle_ticks; //FW-017: ticks since last PAS transition (for pas_idle_ms diagnostic)
//FW-109 v2: fwd_run/Backwards_counter used to be extern'd straight from main.c globals. fwd_run
//was already stale even before this card (moved into src/pas_direction.c under FW-107 and never
//actually read here since - dead declaration, removed). Backwards_counter moved the same way this
//card - see pas_direction_backpedal_confirmed() at the one call site below.
extern uint8_t torque_fault;
extern uint8_t ui_8_PWM_ON_Flag;
extern FlagStatus BC_limit_flag; //FW-033: battery-current limiter active (diagnostics)
extern volatile uint16_t diag_peak_torque, diag_peak_human_w, diag_peak_support, diag_peak_motor_w;
extern volatile int32_t diag_peak_iq_req, diag_peak_iq_set;
extern volatile uint16_t diag_peak_assist_dynamic, diag_peak_u_abs;
extern int32_t i32_hall_order;
extern int32_t Hall_13, Hall_32, Hall_26, Hall_64, Hall_45, Hall_51;
extern uint8_t param_record_state; //FW-023: 0 = valid record, 1 = defaults, 2 = halls rejected
extern current_cal_t current_cal;
//FW-126.7: the dark-bridge sampler, its counters and its self-test are gone. The
//calibration now reports itself through current_cal - one source, one report.
#endif
uint8_t tx_data_length;
uint8_t rx_data_length;
float last_distance =0;
float last_kilometer =0;
float distance =0;
uint16_t display_distance=0;
uint16_t delay_counter =0;
uint16_t k=0;
uint8_t level_code;
uint8_t level_code_old;
uint8_t level_counter;
extern volatile uint16_t hmi_lost_ticks;    //FW-135 assist watchdog (main.c) - reset only on frames whose SOURCE is the display
extern volatile uint8_t  hmi_seen;          //FW-135 assist watchdog arm flag (main.c) - set on the first display frame
extern volatile uint16_t bus_lost_ticks;    //FW-135 power watchdog (main.c) - reset on ANY received frame
extern volatile uint8_t  bus_seen;          //FW-135 power watchdog arm flag (main.c) - set on the first frame of any kind
extern volatile uint16_t update_hold_ticks; //FW-135 update session hold (main.c) - suspends the silence power-off
extern volatile uint16_t ride_seconds;    //FW-134: seconds of motion (defined in main.c)
extern MotorState_t MS;                   //global ride state (defined in main.c): 0x3202 bit0 reads pushassist_flag+Speedx100
extern volatile uint16_t ui16_erps;       //motor electrical rev/s (defined in main.c): unambiguous wheel-motion at walk pace
extern volatile uint16_t click_release_count;   //FW-136.0 (main.c) - quiet descents to zero Iq reference
extern volatile uint16_t coast_peak_iq;         //FW-136.0 (main.c) - peak |Iq| over the whole coast
extern volatile uint16_t coast_peak_erps;       //FW-136.0 (main.c) - rotor speed at that peak
extern volatile uint16_t coast_min_erps;        //FW-136.0 (main.c) - lowest erps ever reported during a coast
//FW-132: evidence for tightening the 0x3005 ownership later - how many arrived and whom the last
//one was addressed to. Diagnostics only; nothing in the control path reads these.
volatile uint16_t cmd3005_seen=0;
volatile uint8_t  cmd3005_last_target=0xFF;
extern uint8_t auto_off_minutes;          //runtime auto-off timeout [min] (defined in main.c) - set from HMI 0x6303
extern volatile uint32_t control_time_ticks; //FW-114: 4 kHz free-running clock (main.c) - drives the 0x3000 session counter
uint8_t walk_can_counter;
uint8_t walk_can_release_counter;

uint16_t Rx_MF_active=0;
uint16_t checksum=0;

#if CAN_DIAGNOSTICS_ENABLE
static void put_i32_le(uint8_t *dst, int32_t value)
{
	uint32_t raw = (uint32_t)value;
	dst[0] = raw & 0xFF;
	dst[1] = (raw >> 8) & 0xFF;
	dst[2] = (raw >> 16) & 0xFF;
	dst[3] = (raw >> 24) & 0xFF;
}

static void put_u16_le(uint8_t *dst, uint16_t value)
{
	dst[0] = (uint8_t)(value & 0xFFU);
	dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static uint16_t crc16_ccitt(const uint8_t *data, uint8_t len)
{
	uint16_t crc = 0xFFFFU;
	uint8_t i, bit;
	for (i = 0U; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (bit = 0U; bit < 8U; bit++)
			crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
	}
	return crc;
}

extern uint8_t fw1267_diag_abort_armed;   /* FW-126.7 DIAG post-validation stop */

/*
 * 0x602F - the FW-127 acquisition report, DIAG only, read-only. Schema 1.
 *
 * This is the whole evidence set for the single consolidated hardware session, as compact
 * counters rather than a recorder: the decisions it has to unlock are all "how often" and
 * "did it ever", and a counter answers those without costing RAM or perturbing the control path.
 * Nothing here is read by the control loop, so a lost update can never change behaviour.
 *
 * 64-byte payload, CRC16-CCITT over bytes 0..61, little-endian throughout.
 */
static void fw127_serialize_report(uint8_t out[64])
{
	const pwm_geometry_stats_t *g = pwm_geometry_get_stats();
	const current_feedback_state_t *f = current_feedback_get();
	current_sample_context_t ctx;
	const uint8_t ctx_ok = current_sample_ctx_snapshot(&ctx);
	uint8_t i;

	for (i = 0U; i < 64U; i++) out[i] = 0U;
	out[0] = 'A'; out[1] = 'Q'; out[2] = 1U;
	/* bit0: the snapshot was coherent; bit1: an illegal CH3 was NEVER programmed, which after
	 * FW-127C is structural - sample_window_decide() returns INVALID instead of computing one. */
	out[3] = (uint8_t)((ctx_ok ? 0x01U : 0U) | 0x02U);

	/* --- PWM geometry: how far the request really went outside the legal range ------------ */
	put_i32_le(&out[4],  (int32_t)g->clamp_total);
	put_i32_le(&out[8],  (int32_t)g->clamp_phase[0]);
	put_i32_le(&out[12], (int32_t)g->clamp_phase[1]);
	put_i32_le(&out[16], (int32_t)g->clamp_phase[2]);
	put_i32_le(&out[20], g->peak_requested);
	put_i32_le(&out[24], g->min_requested);
	put_u16_le(&out[28], g->peak_applied);
	put_u16_le(&out[30], g->min_applied);

	/* --- sampling states: the distribution the window derivation predicted ---------------- */
	put_i32_le(&out[32], (int32_t)f->primary_count);
	put_i32_le(&out[36], (int32_t)f->alternate_count);
	put_i32_le(&out[40], (int32_t)f->invalid_count);
	put_i32_le(&out[44], (int32_t)f->reuse_count);

	/* --- last-valid behaviour ------------------------------------------------------------- */
	put_u16_le(&out[48], (uint16_t)((f->max_sample_age > 65535u) ? 65535u : f->max_sample_age));
	put_u16_le(&out[50], (uint16_t)((f->starved_count > 65535u) ? 65535u : f->starved_count));
	for (i = 0U; i < CURRENT_FEEDBACK_AGE_BUCKETS; i++) {
		const uint32_t v = f->age_bucket[i];
		put_u16_le(&out[52U + 2U * i], (uint16_t)((v > 65535u) ? 65535u : v));
	}

	/* --- transaction ownership ------------------------------------------------------------- */
	{
		const uint32_t orph = current_sample_ctx_orphan_count();
		put_u16_le(&out[60], (uint16_t)((orph > 65535u) ? 65535u : orph));
	}
	put_u16_le(&out[62], crc16_ccitt(out, 62U));
}

/*
 * 0x602D - the phase-current calibration report, DIAG only, read-only. Schema 2 (FW-126.7).
 *
 * THE THREE DOMAINS ARE KEPT APART ON THE WIRE, because conflating them is what took four
 * cards to untangle:
 *
 *     physical ADC result  =  JDR + IOFF   (reported as `midpoint`, ~2020 at zero current)
 *     JDR                  =  hardware-offset-corrected inserted result (small, signed)
 *     software offset      =  current_cal.offset[], what the FOC ISR subtracts from JDR
 *
 * Nothing here is labelled "raw ADC" without qualification. 66-byte payload, CRC16-CCITT over
 * bytes 0..63, little-endian throughout.
 */
static void current_cal_serialize_dump(uint8_t out[66])
{
	uint8_t i;
	static const uint16_t hw_ioff[CURRENT_CAL_PHASES] = {
		(uint16_t)CURRENT_HW_OFFSET_A, (uint16_t)CURRENT_HW_OFFSET_B, (uint16_t)CURRENT_HW_OFFSET_C
	};
	/* Bit 3 is a constant 1 and says so: the only caller of current_cal_sample() is the dwell
	 * branch of the FOC ISR, which is mutually exclusive with FOC_calculation(). It is on the
	 * wire so a decoder need not know that, not because it was measured. */
	uint8_t flags = (uint8_t)((current_cal.valid ? 0x01U : 0U)
		| (current_cal.timeout_hit ? 0x02U : 0U)
		| (current_cal.neutral_ok ? 0x04U : 0U)      /* MOE on + compares neutral, every sample */
		| 0x08U                                       /* FOC blocked: structural                 */
		| (fw1267_diag_abort_armed ? 0x10U : 0U));

	for (i = 0U; i < 66U; i++) out[i] = 0U;
	out[0] = 'C'; out[1] = 'C'; out[2] = 2U; out[3] = flags;
	out[4] = (uint8_t)current_cal.state;
	out[5] = (uint8_t)current_cal.failure_reason;
	out[6] = (uint8_t)current_cal.source;
	out[7] = current_cal.attempts;
	put_u16_le(&out[8],  current_cal.cycles);
	put_u16_le(&out[10], current_cal.stable_count);
	put_u16_le(&out[12], current_cal.eligible);
	put_u16_le(&out[14], current_cal.restarts);
	for (i = 0U; i < CURRENT_CAL_PHASES; i++) {
		put_u16_le(&out[16U + 2U * i], (uint16_t)current_cal.offset[i]);   /* software offset */
		put_u16_le(&out[22U + 2U * i], (uint16_t)current_cal.mean[i]);     /* JDR mean        */
		put_u16_le(&out[28U + 2U * i], (uint16_t)current_cal.acc[i].min);  /* JDR min         */
		put_u16_le(&out[34U + 2U * i], (uint16_t)current_cal.acc[i].max);  /* JDR max         */
		put_u16_le(&out[40U + 2U * i], current_cal.p2p[i]);                /* JDR P2P         */
		put_u16_le(&out[46U + 2U * i], hw_ioff[i]);                        /* hardware IOFF   */
		/* The physical ADC result implied by the calibrated zero - the number that has to land
		 * near mid-scale for the measurement to have been taken in a valid electrical state. */
		put_u16_le(&out[52U + 2U * i], (uint16_t)((int32_t)hw_ioff[i] + (int32_t)current_cal.mean[i]));
	}
	/* Gate constants echoed, so a report can be read without the build's config.h to hand. */
	out[58] = (uint8_t)CURRENT_CAL_STABLE_CYCLES;
	out[59] = (uint8_t)CURRENT_CAL_COLLECT_SAMPLES;
	put_u16_le(&out[60], (uint16_t)CURRENT_CAL_MAX_CYCLES);
	put_u16_le(&out[62], (uint16_t)CURRENT_CAL_RESIDUAL_MAX_DEVIATION);
	put_u16_le(&out[64], crc16_ccitt(out, 64U));
}

#endif

/*
 * FW-136.0: READ 0x6032 -> NORMAL_ACK 0x022A6032, DLC 8. Four little-endian u16s, cumulative
 * since power-on:
 *
 *   0..1  quiet descents to an exact zero Iq reference  (the denominator - and note that Walk
 *                                                        Assist can NEVER move this: ride_control
 *                                                        returns BYPASS before the zero policy is
 *                                                        granted, so only a PEDAL release counts)
 *   2..3  peak |measured Iq| over the whole coast       (1 unit = CAL_I = 95 mA)
 *   4..5  rotor erps at that peak                       (says WHERE the current was)
 *   6..7  lowest erps ever reported during a coast      (0xFFFF = no coast sampled yet; a value
 *                                                        that never drops below
 *                                                        RIDE_COAST_RELEASE_ERPS means the speed
 *                                                        reading freezes as the rotor stops, and
 *                                                        QZERO's handback can never fire)
 *
 * Little-endian to match the 0x60xx command family this lives in, NOT the big-endian diag
 * logger frames - the two conventions exist side by side and mixing them up is how a reading
 * gets misinterpreted. Example: bytes "32 00" in 2..3 are 0x0032 = 50 units = ~4,75 A.
 *
 * Present in EVERY build, not only DIAG: the image that clicks is the image that has to be
 * measured, and it produces no bus traffic at all until something asks for it.
 */
static void send_click_zone_status(void)
{
	uint8_t d[8];
	d[0] = (uint8_t)(click_release_count & 0xFFU);
	d[1] = (uint8_t)((click_release_count >> 8) & 0xFFU);
	d[2] = (uint8_t)(coast_peak_iq & 0xFFU);
	d[3] = (uint8_t)((coast_peak_iq >> 8) & 0xFFU);
	d[4] = (uint8_t)(coast_peak_erps & 0xFFU);
	d[5] = (uint8_t)((coast_peak_erps >> 8) & 0xFFU);
	d[6] = (uint8_t)(coast_min_erps & 0xFFU);
	d[7] = (uint8_t)((coast_min_erps >> 8) & 0xFFU);
	can_tx_queue_enqueue(0x022A6032U, 8U, d);
}

void processCAN_Rx(MotorParams_t* MP, MotorState_t* MS){

	Ext_ID_Rx.command = (receive_message.rx_efid)&0xFFFF;
	Ext_ID_Rx.operation = (receive_message.rx_efid>>16)&0x07; //only 3 bit width
	Ext_ID_Rx.target = (receive_message.rx_efid>>19)&0x1F; //only 5 bit width
	Ext_ID_Rx.source = (receive_message.rx_efid>>24)&0x1F;

	/*
	 * FW-135: two independent liveness questions, so two counters.
	 *
	 * BUS - ANY frame, whoever it is addressed to. This is the only thing the self power-off
	 * may depend on. During a DISPLAY firmware update the updater talks to node 3, so nothing
	 * is addressed to us for minutes at a time even though the bus is plainly alive - and
	 * powering off there cuts the display's own supply in the middle of a flash write.
	 *
	 * HMI - frames whose SOURCE is the display, addressed or broadcast alike. This is what the
	 * right to assist depends on. The old code reset that counter on any frame with
	 * target == 2, so a Canable, a BESST or an updater kept a DEAD display looking alive.
	 */
	bus_lost_ticks=0;
	bus_seen=1;
	if(Ext_ID_Rx.source==3){ hmi_lost_ticks=0; hmi_seen=1; }

	if(Ext_ID_Rx.target==2){ //controller answers on target=2 only (factory ignores tgt=4 info queries; answering duplicates pollutes target=3)
		switch (Ext_ID_Rx.operation){
			case WRITE_CMD:

				if(Ext_ID_Rx.command==0x6033 || Ext_ID_Rx.command==0x6034){
					/* Explicit ARM / replay. Read requests never change the capture. */
					uint8_t accepted=0U;
					if(Ext_ID_Rx.source==5U && receive_message.rx_dlen==0U){
						accepted=(Ext_ID_Rx.command==0x6033 ? stop_trace_arm() :
							stop_trace_dump_request()) ? 1U : 0U;
					}
					sendWriteResult(Ext_ID_Rx.command,accepted);
				}
				else if (receive_message.rx_dlen==1 && receive_message.rx_data[0]>8 && Ext_ID_Rx.source==5){
					Rx_MF_active=Ext_ID_Rx.command;
					rx_data_length=receive_message.rx_data[0];
				}
				else if(Ext_ID_Rx.command==0x6022 && Ext_ID_Rx.source==5){ //FW-006: persist banks (deferred to standstill)
					bank_save_request=1;
				}
				else if(Ext_ID_Rx.command==0x6026 && Ext_ID_Rx.source==5){ //FW-013: torque load calibration operations
					switch(receive_message.rx_data[0]){
						case 1: torque_input_cal_start(); break;
						case 2: torque_input_cal_capture_load(receive_message.rx_data[1]+(receive_message.rx_data[2]<<8)); break;
						case 3: torque_input_cal_commit(); break;
						case 4: torque_input_cal_cancel(); break;
						case 5: torque_input_cal_restore_default(); break;
					}
				}
				else if(Ext_ID_Rx.command==0x602B && Ext_ID_Rx.source==5){ //FW-018: set full-charge PACK-voltage threshold
					//byte0=ver(1); byte1..2=soc_full_pack_10mv LE; byte3..6=0; byte7=CRC-8/SMBUS over bytes 0..6
					if(receive_message.rx_dlen>=8 && receive_message.rx_data[0]==1){
						uint8_t crc=0;
						for(uint8_t i=0;i<7;i++){ crc^=receive_message.rx_data[i]; for(uint8_t b=0;b<8;b++) crc=(crc&0x80)?((crc<<1)^0x07):(crc<<1); }
						if(crc==receive_message.rx_data[7]){
							uint16_t v10=receive_message.rx_data[1]+(receive_message.rx_data[2]<<8);
							uint32_t mv=(uint32_t)v10*10U;
							if(mv>=SOC_FULL_PACK_MIN_MV && mv<=SOC_FULL_PACK_MAX_MV){
								MP->soc_full_pack_10mv=v10;   //RAM apply now; flash write deferred to standstill
								MP->soc_full_magic=SOC_FULL_MAGIC;
								soc_full_persist=1;
							}
						}
					}
				}
#if CAN_DIAGNOSTICS_ENABLE
				else if(Ext_ID_Rx.command==0x602C){
					/* RETIRED: the frozen rolling no-assist replay observed the legacy assist
					 * pipeline and went with it. The COMMAND is answered rather than dropped,
					 * because a shipped tool that still asks for it must get a definite "not
					 * accepted" instead of a timeout it cannot distinguish from a dead bus. */
					sendWriteResult(0x602C, 0U);
				}
				else if(Ext_ID_Rx.command==0x6030){
					/* QS-1: explicit replay of a complete, immutable FOC-rate capture. */
					uint8_t accepted = (Ext_ID_Rx.source == 5U && receive_message.rx_dlen == 0U &&
					                    qs_transition_dump_request()) ? 1U : 0U;
					sendWriteResult(0x6030, accepted);
				}
				else if(Ext_ID_Rx.command==0x6031){
					/* QS-1R: one-shot request. The ISR owns the actual ring reset. */
					uint8_t accepted = (Ext_ID_Rx.source == 5U && receive_message.rx_dlen == 0U &&
					                    !qs_transition_dump_busy() && qs_transition_diag_request_new_capture()) ? 1U : 0U;
					sendWriteResult(0x6031, accepted);
				}
#endif
				else if(Ext_ID_Rx.command==0x3203){ //FW-076: speed limit + wheel diameter code + circumference
					/*
					 * Validate the WHOLE frame first, then apply. The old code applied
					 * whatever it got and afterwards silently substituted a default for
					 * anything out of range, so a bad frame left the bike with one new
					 * value and one invented one, and nothing said so. It also ignored
					 * bytes 2-3 entirely, which is why the wheel diameter never stuck.
					 */
					uint8_t ok = (receive_message.rx_dlen >= 6);
					uint16_t new_limit = 0, new_circumference = 0;
					if(ok){
						new_limit = receive_message.rx_data[0] + (receive_message.rx_data[1]<<8);
						new_circumference = receive_message.rx_data[4] + (receive_message.rx_data[5]<<8);
						if(new_limit < SPEEDLIMIT_X100_MIN || new_limit > SPEEDLIMIT_X100_MAX) ok = 0;
						if(new_circumference < WHEEL_CIRCUMFERENCE_MIN ||
						   new_circumference > WHEEL_CIRCUMFERENCE_MAX) ok = 0;
					}
					if(ok){
						MP->speedLimitx100 = new_limit;
						MP->wheel_cirumference = new_circumference;
						//Two raw bytes, never interpreted here: the code is metadata for the
						//tools and the display. Speed still comes from the circumference alone.
						MP->wheel_diameter_code[0] = receive_message.rx_data[2];
						MP->wheel_diameter_code[1] = receive_message.rx_data[3];
						MP->wheel_diameter_magic = WHEEL_DIAMETER_MAGIC;
						write_virtual_eeprom();
					}
					sendWriteResult(0x3203, ok);
				}
				else if(Ext_ID_Rx.command==0x62D9){ //Startup angle, used as multiplyer here
					MP->TS_coeff=receive_message.rx_data[0]+(receive_message.rx_data[1]<<8);
					//save received setting
					write_virtual_eeprom();
				}
				else if(Ext_ID_Rx.command==0x6200 && Ext_ID_Rx.source==5){ //Hall/position sensor calibration (Canable/BESST only)
					/*
					 * FW-110 v5: a properly directed WRITE 0x6200 from Canable/BESST (source=5) is
					 * offered to hall_calibration_request() (src/main.c), which accepts it ONLY if
					 * the standstill gate has held continuously for a full second, and then merely
					 * arms it - main()'s while(1) runs the procedure, never this parser.
					 *
					 * The reply is therefore meaningful again instead of always refusing:
					 *   NORMAL_ACK (2) - accepted, the motor is about to turn for >5 s;
					 *   ERROR_ACK  (3) - refused, the bike was not confirmed standing still.
					 *
					 * FW-110 v4 answered ERROR_ACK unconditionally because the supervisor that
					 * mediated this was deleted with it, which left every controller stuck on the
					 * compiled-in HALL_DEF_* table permanently. See inc/main.h for the honest
					 * limitation that remains: the check is point-in-time, not continuous.
					 */
					sendWriteResult(0x6200, hall_calibration_request() ? 1U : 0U);
				}

				else sendCAN_Tx(MP,MS);
				//Factory sends NO WRITE-ACK for the continuous operational data 0x6300-0x6304; only for
				//config writes. ACKing 0x630x floods 0x822A630x that the factory never emits -> exact-match test.
				//FW-076: 0x3203 sends its own result (ACK on success, ERROR_ACK on a rejected
				//frame), so the generic ACK must not fire for it — the tool would see a
				//success alongside a failure and believe the first one it matched.
				//FW-110 v4: 0x6200 is the same case, now handled above (one ERROR_ACK only) - the
				//generic ACK must not fire for it either.
				if(!(Ext_ID_Rx.command>=0x6300 && Ext_ID_Rx.command<=0x6304)
				   && Ext_ID_Rx.command!=0x3203 && Ext_ID_Rx.command!=0x6200
				   && Ext_ID_Rx.command!=0x6033 && Ext_ID_Rx.command!=0x6034
#if CAN_DIAGNOSTICS_ENABLE
				   && Ext_ID_Rx.command!=0x602C
				   && Ext_ID_Rx.command!=0x6030
				   && Ext_ID_Rx.command!=0x6031
#endif
				  ) sendAcknoledge();
				break;
			case READ_CMD:
				/* FW-136.0: answered in every build, so the image that clicks is the image that
				 * gets measured. Tool only (source 5) and zero-length, like every other
				 * side-effect-free status read here. */
				if(Ext_ID_Rx.command==0x6033){
					if(Ext_ID_Rx.source==5U && receive_message.rx_dlen==0U){
						uint8_t data[8]; stop_trace_status(data);
						can_tx_queue_enqueue(0x022A6033U,8U,data);
					}
				}
				else if(Ext_ID_Rx.command==0x6032 && Ext_ID_Rx.source==5U && receive_message.rx_dlen==0U){
					send_click_zone_status();
				}
#if CAN_DIAGNOSTICS_ENABLE
				else if(Ext_ID_Rx.command==0x6031 && Ext_ID_Rx.source==5U && receive_message.rx_dlen==0U){
					send_qs_transition_status();
				}
#endif
				else {
					sendCAN_Tx(MP,MS);
				}
				break;
			case NORMAL_ACK:
				//FW-114: the display confirms a multiframe reply's START with op=2 (e.g. 83126000
				//after 821C6000) before the controller may send DATA - the factory's flow control.
				//Forward the command to the multiframe producer, which gates the DATA phase on it
				//for target=3 replies (with CANMF_ACK_WAIT_TIMEOUT_TICKS as the fallback for
				//displays that never ACK). Only the HMI's ACK (source=3) is relevant - a write-ACK
				//from the tool (source=5) must not release the gate.
				if(Ext_ID_Rx.source==3) can_multiframe_hmi_ack(Ext_ID_Rx.command);
				break;
			case ERROR_ACK:
				break;
			case LONG_START_CMD:
				switch (Rx_MF_active){
					case 0x6010: //Para0
						append_multiframe(0, &Para0[0]);
						break;
					case 0x6011: //Para1
						append_multiframe(0, &Para1[0]);
						break;
					case 0x6012: //Para2
						append_multiframe(0, &Para2[0]);
						break;
					case 0x6021: //FW-006: bank write
						append_multiframe(0, (char*)&BankBlob[0]);
						break;
					case 0x6024: //FW-010: tuning write
						append_multiframe(0, (char*)&TuningBlob[0]);
						break;
				}

				break;
			case LONG_TRANG_CMD:
				switch (Rx_MF_active){
					case 0x6010: //Para0
						append_multiframe(Ext_ID_Rx.command+1, &Para0[0]);
						break;
					case 0x6011: //Para1
						append_multiframe(Ext_ID_Rx.command+1, &Para1[0]);
						break;
					case 0x6012: //Para2
						append_multiframe(Ext_ID_Rx.command+1, &Para2[0]);
						break;
					//FW-068/069: these bounds cap the frame index that may be written. They must grow
					//with the blob or the closing frames are dropped in silence and the write fails
					//on CRC with nothing to point at the cause.
					case 0x6021: //FW-084: bank write, 255 B -> last frame index 31 (7 B), BankBlob[256]
						if(Ext_ID_Rx.command < 31) append_multiframe(Ext_ID_Rx.command+1, (char*)&BankBlob[0]);
						break;
					case 0x6024: //tuning write: 32 B -> last frame index 3, TuningBlob[32]
						if(Ext_ID_Rx.command < 3) append_multiframe(Ext_ID_Rx.command+1, (char*)&TuningBlob[0]);
						break;
				}
				break;

			case LONG_END_CMD:
				switch (Rx_MF_active){
					case 0x6010: //Para0
						append_multiframe(Ext_ID_Rx.command+1, &Para0[0]);
						break;
					case 0x6011: //Para1
						append_multiframe(Ext_ID_Rx.command+1, &Para1[0]);

						break;
					case 0x6012: //Para2
						append_multiframe(Ext_ID_Rx.command+1, &Para2[0]);

						break;
					case 0x6021: //FW-006/FW-068/FW-069/FW-084: bank write, 255 B -> last frame index 31
						if(Ext_ID_Rx.command < 31) append_multiframe(Ext_ID_Rx.command+1, (char*)&BankBlob[0]);
						break;
					case 0x6024: //FW-010/FW-068: tuning write, 32 B -> last frame index 3
						if(Ext_ID_Rx.command < 3) append_multiframe(Ext_ID_Rx.command+1, (char*)&TuningBlob[0]);
						break;
				}
				k = ((Ext_ID_Rx.command+1)<<3)+receive_message.rx_dlen;
				if(((Ext_ID_Rx.command+1)<<3)+receive_message.rx_dlen==rx_data_length){
					//to do send acknoledge OK
					uint16_t completed_cmd = Rx_MF_active;
					Rx_MF_active=0;
					//save received setting
					if(completed_cmd==0x6021){
						//FW-006: RAM apply only; flash write happens via 0x6022 at standstill.
						//FW-068: answer with the REAL result - a blob rejected for a bad CRC or an
						//unsupported version must never be reported to the tool as written.
						uint8_t applied = assist_modes_apply_bank_blob(&BankBlob[0], rx_data_length) ? 1U : 0U;
						sendWriteResult(completed_cmd, applied);
					}
					else if(completed_cmd==0x6024){
						//FW-010: RAM apply only; flash write happens via 0x6022 at standstill
						uint8_t applied = tuning_config_apply_blob(&TuningBlob[0], rx_data_length) ? 1U : 0U;
						sendWriteResult(completed_cmd, applied);
					}
					else{
						parse_DPparams(MP);
						write_virtual_eeprom();
					}
					rx_data_length=0;

				}
				else{
					//FW-068: the transfer did not add up to the announced length (dropped or
					//extra frame). For the two config blobs answer ERROR_ACK instead of going
					//silent, so the tool fails immediately with a reason rather than waiting
					//out its timeout with no clue what went wrong.
					uint16_t incomplete_cmd = Rx_MF_active;
					if(incomplete_cmd==0x6021 || incomplete_cmd==0x6024){
						sendWriteResult(incomplete_cmd, 0);
					}
					Rx_MF_active=0;
					rx_data_length=0;
				}

				break;
		}


		if(Ext_ID_Rx.command==0x6300){
			level_code=receive_message.rx_data[1];
			if(level_code==level_code_old&&level_counter<3)level_counter++;
			if(level_code!=level_code_old){

				level_counter=0;
				}
			level_code_old=level_code;

			if(level_counter==3){

				level_counter=4;
				switch (level_code){
					case 0:
						MS->assist_level=0;
						break;
					case 1:
						MS->assist_level=1;
						break;
					case 0x0B:
						MS->assist_level=2; //Eco
						break;
					case 0x0C:
						MS->assist_level=3;
						break;
					case 0x0D:
						MS->assist_level=4; //Tour
						break;
					case 0x02:
						MS->assist_level=5;
						break;
					case 0x15:
						MS->assist_level=6;//Sport
						break;
					case 0x16:
						MS->assist_level=7;
						break;
					case 0x17:
						MS->assist_level=8; //Sport +
						break;
					case 0x03:
						MS->assist_level=9; //Boost
						break;
					//Some displays send direct numeric levels. Keep 0x06 reserved for Walk Assist below.
					case 0x04:
						MS->assist_level=4;
						break;
					case 0x05:
						MS->assist_level=5;
						break;
					case 0x07:
						MS->assist_level=7;
						break;
					case 0x08:
						MS->assist_level=8;
						break;
					case 0x09:
						MS->assist_level=9;
						break;
				}
			}
			// Walk Assist request from display: only sets walk_can_request (with debounce).
			// pushassist_flag itself is derived in main.c (requires physical PA4 button held).
			if (receive_message.rx_data[1]==6){
				walk_can_release_counter=0;
				if(walk_can_counter<3)walk_can_counter++;
				if(walk_can_counter>=3)MS->walk_can_request=SET;
			}else{
				walk_can_counter=0;
				if(walk_can_release_counter<3)walk_can_release_counter++;
				if(walk_can_release_counter>=3)MS->walk_can_request=RESET;
			}
			if (receive_message.rx_data[2]&0b1)MS->light_flag=SET;
			else MS->light_flag=RESET;
			if (receive_message.rx_data[2]&0b10)MS->button_up_flag=SET;
			else MS->button_up_flag=RESET;
			if (receive_message.rx_data[2]&0b100000)MS->button_down_flag=SET;
			else MS->button_down_flag=RESET;

		}

		if(Ext_ID_Rx.command==0x6303 && receive_message.rx_dlen>=1){ //auto-off timeout [min] set on the HMI
			auto_off_minutes=receive_message.rx_data[0]; //overwrites the AUTO_OFF_MINUTES default
		}

		//FW-076: the 0x3203 write moved into the WRITE_CMD chain above, so the frame is
		//validated before anything is applied and answers with exactly one ACK. It used to
		//sit here, AFTER the generic handler had already replied with the OLD values.
		//FW-110 v4: 0x6200 stays handled ONLY in the WRITE_CMD chain above (operation and
		//source checked there) and answers with exactly one ERROR_ACK - no calibration path.

		if(Ext_ID_Rx.command==0x6101){ //Torque sensor calibration normally, here used for factory settings reset
			InitEEPROM(MP);
			read_virtual_eeprom();
			parse_MOparams(MP);

		}



	}
	// NOTE: 0x6400/0x6401 (single-frame 0x8228 version/model) handlers REMOVED for exact factory match —
	// the factory M820 does NOT answer these and the HMI shows info without them. See git history if needed.
	/*
	 * FW-132 P0: 0x3005 means "jump to the bootloader", and until now ANY 0x3005 did it - this
	 * check sits OUTSIDE the target==2 block above, so it saw broadcasts too.
	 *
	 * The DP-C245 display updater announces its session with 0x85FF3005, and decoding that id with
	 * this file's own rule gives target = (0x85FF3005 >> 19) & 0x1F = 31, i.e. BROADCAST. So
	 * updating the DISPLAY restarted the CONTROLLER, in the middle of the transfer - which matches
	 * the reported "HMI comes back with a communication error after an update".
	 *
	 * The fix is deliberately the NARROWEST one that removes the harm: ignore the broadcast, honour
	 * every addressed 0x3005 exactly as before. Tightening this further to target==2 only is what
	 * the guidance card asks for (its P0), but that needs a log of a REAL controller update first:
	 * if the controller updater also announces by broadcast, requiring target==2 would break our
	 * own firmware update. The card is explicit about not guessing which frame distinguishes the
	 * two sessions, and cmd3005_seen/cmd3005_last_target below exist to collect exactly that
	 * evidence from the bike.
	 */
	if(Ext_ID_Rx.command==0x3005){
		if(cmd3005_seen<0xFFFFU)cmd3005_seen++;
		//FW-135: an update session has been announced. Suspend the silence power-off, so a quiet
		//stretch of the DISPLAY flash cannot cut its own supply. Armed by EVERY 0x3005, addressed
		//and broadcast alike - both kinds mean somebody is flashing something on this bus.
		update_hold_ticks=UPDATE_HOLD_TICKS;
		cmd3005_last_target=(uint8_t)Ext_ID_Rx.target;
		if(Ext_ID_Rx.target!=31){ //31 = broadcast: belongs to the display update session, not to us
			NVIC_SystemReset();
		}
	}
}
//FW-006/FW-010: explicit write-ACK for a completed multiframe block. At LONG_END the
//received command id is the last frame index, so the acknowledged command must be passed in.
/*
 * FW-068: report what actually happened to a multiframe config write.
 *
 * Until now the controller answered every completed transfer with operation 2 (NORMAL_ACK)
 * without looking at the return value of the apply function. A blob rejected for a bad CRC
 * or an unsupported version therefore looked to Canable exactly like a successful write: the
 * tool reported "written", the user pressed Save to Flash, and the OLD settings were kept.
 * Operation 3 (ERROR_ACK) is already understood by the tool's request manager, which resolves
 * it as a failure, so a rejected blob now surfaces as one.
 */
void sendWriteResult(uint16_t command, uint8_t applied){
	Ext_ID_Tx.command = command;
	Ext_ID_Tx.operation = applied ? 2 : 3; //2 = NORMAL_ACK, 3 = ERROR_ACK
	Ext_ID_Tx.target = Ext_ID_Rx.source; //reply to the requester (Canable/BESST = 5)
	Ext_ID_Tx.source = 0x02;
	uint32_t efid = Ext_ID_Tx.command+(Ext_ID_Tx.operation<<16)+(Ext_ID_Tx.target<<19)+(Ext_ID_Tx.source<<24);
	uint8_t d[8] = {0};
	can_tx_queue_enqueue(efid, 0U, d); //FW-110: was a blocking can_message_transmit/can_transmit_states wait
}

#if CAN_DIAGNOSTICS_ENABLE
/* QS-1R STATUS, READ 0x6031 -> NORMAL_ACK 0x822A6031, DLC 8:
 * schema, state(IDLE/ARMED/TRIGGERED/COMPLETE), generation, sample_count,
 * flags(bit0 export-ready, bit1 export busy), trigger events, trigger index, reserved. */
static void send_qs_transition_status(void)
{
	qs_transition_status_t s;
	uint8_t d[8] = {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
	qs_transition_diag_status(&s);
	d[1] = (uint8_t)s.state;
	d[2] = s.generation;
	d[3] = s.sample_count;
	d[4] = (uint8_t)((s.export_ready ? 0x01U : 0U) | (qs_transition_dump_busy() ? 0x02U : 0U));
	d[5] = s.trigger_events;
	d[6] = s.trigger_index;
	can_tx_queue_enqueue(0x022A6031U, 8U, d);
}
#endif

void sendAcknoledge(void){
	Ext_ID_Tx.command = Ext_ID_Rx.command;
	Ext_ID_Tx.operation = 2; //acknoledge OK
	Ext_ID_Tx.target = 5; //WRITE-ACK ALWAYS to target=5 (HMI write-ACK port). Replying to source(=3) pollutes
	                      //the target=3 READ channel with ACKs and breaks the info multiframe -> blank HMI info.
	Ext_ID_Tx.source = 0x02; //controller
	uint32_t efid = Ext_ID_Tx.command+(Ext_ID_Tx.operation<<16)+(Ext_ID_Tx.target<<19)+(Ext_ID_Tx.source<<24);
	uint8_t d[8] = {0};
	can_tx_queue_enqueue(efid, 0U, d); //FW-110: was a blocking can_message_transmit/can_transmit_states wait
}

#if CAN_TORQUE_STREAM_ENABLE
static uint32_t torque_stream_dropped;

/* Attempts refused because the peripheral had no free mailbox this call - never retried, never
 * queued (see sendCAN_3100's own header comment for why). */
uint32_t sendCAN_3100_dropped_count(void) { return torque_stream_dropped; }

void sendCAN_3100(MotorState_t* MS){
	/* Emulates the separate torque-sensor CAN node (source=01) that the ORIGINAL Bafang system
	 * has as a genuinely separate physical board. On that original system the HMI reads cadence
	 * and torque from this frame - but on THIS firmware that comment does not hold: this stream
	 * is compiled out of every normal riding build (CAN_TORQUE_STREAM_ENABLE defaults to 0, see
	 * inc/config.h), and normal riding already works with cadence/torque shown on the HMI without
	 * it ever being sent - the display gets that data through the regular poll frames below, not
	 * this one.
	 *
	 * FW-110: deliberately NOT routed through can_tx_queue. That queue's 16 slots exist for
	 * frames that must be delivered; this stream, measured at ~92 frames/s when enabled, would
	 * compete for the exact same slots as HMI/ACK/multiframe replies purely by existing, which
	 * is precisely the "0x3100 must never displace a critical frame" requirement this card
	 * exists to satisfy. So this is its own, separate, best-effort path: at most ONE transmit
	 * attempt, no retry, no wait, no queue - a busy peripheral this call is simply skipped and
	 * counted, exactly like the next 10 ms tick will try again on its own regardless. */
	static uint8_t ctr = 0;
	transmit_message.tx_sfid = 0x00;
	transmit_message.tx_efid = 0x01F83100U; //source=1 (torque sensor), target=31 broadcast
	transmit_message.tx_ft   = CAN_FT_DATA;
	transmit_message.tx_ff   = CAN_FF_EXTENDED;
	transmit_message.tx_dlen = 4U;
	transmit_message.tx_data[0] = (uint8_t)((uint16_t)MS->torque_on_crank & 0xFF);
	transmit_message.tx_data[1] = (uint8_t)((uint16_t)MS->torque_on_crank >> 8);
	transmit_message.tx_data[2] = MS->cadence;
	transmit_message.tx_data[3] = ctr++;
	uint8_t mb = can_message_transmit(CAN0, &transmit_message);
	if (mb == CAN_NOMAILBOX) { torque_stream_dropped++; }
}
#endif

void sendCAN_3202(void){
	//0x3202 byte0 bit0 drives the stock HMI (DPC245 CF80301.2) Walk-icon blink: it is copied
	//to state[0].bit4 and gates the periodic FF/FE callback. It must be 0x01 ONLY while a Walk
	//session is actually RUNNING and the bike is ACTUALLY MOVING - not merely when Walk mode
	//was selected. "Moving" is the same wheel-OR-motor test FW-134 uses for 0x3210 (main.c):
	//at walking pace a single wheel pulse can be ~2.6 s apart, right at the speed-stop timeout,
	//so wheel speed alone would flicker to zero exactly during Walk - the motor is unambiguous
	//there. pushassist_flag gates the whole expression to Walk only, so an ordinary pedal start
	//never sets the bit (per user decision 2026-09-07).
	uint8_t d[8] = {0};
	if((MS.pushassist_flag != RESET)
	   && (MS.Speedx100 > 0 || ui16_erps >= RIDE_COAST_RELEASE_ERPS)){
		d[0] = 0x01;
	}
	can_tx_queue_enqueue(0x02F83202U, 1U, d); //FW-110: was a blocking can_message_transmit/can_transmit_states wait
}

void sendCAN_Poll(MotorParams_t* MP, MotorState_t* MS, uint16_t command){

	switch (command){

		case 0x3201: //speed and power
			{
			uint8_t d[8];
			//FW-050: one confirmation source for every level gesture. Previously offroadtics
			//(which was ALSO the gesture's digit counter) took priority over the bank splash,
			//so a bank switch first showed ~4 km/h, and any quick double tap on the level
			//button made the speedometer read 1 km/h. Now the splash has its own state and is
			//non-zero only while a gesture is actually being confirmed.
			uint8_t gesture_splash = level_gesture_splash_kmh();
			if(gesture_splash){
				d[0] = (gesture_splash*100)&0xFF;
				d[1] = ((gesture_splash*100)>>8)&0xFF;
			}
			else{
				d[0] = (MS->Speedx100)&0xFF;
				d[1] = ((MS->Speedx100)>>8)&0xFF;
			}
			d[2] = (abs(MS->Battery_Current)/10)&0xFF;
			d[3] = ((abs(MS->Battery_Current)/10)>>8)&0xFF;
			d[4] = (MS->Voltage/10)&0xFF;
			d[5] = ((MS->Voltage/10)>>8)&0xFF;
			d[6] = MS->int_Temperature+40; //temp sterownika (M820: jeden czujnik); +40 = offset protokolu, parser PC odejmuje 40
			d[7] = MS->int_Temperature+40; //ta sama temp sterownika jako "motor temp"; +40 = offset protokolu (NIE przeklamanie)
			can_tx_queue_enqueue(0x02F83201U, 8U, d); //FW-110: was a blocking can_message_transmit/can_transmit_states wait
			}
			break;

		case 0x3200: //battery and distance
			if(delay_counter<10)delay_counter++;
			else{
				delay_counter=0;
				distance=(MS->distance_since_startup)-last_kilometer;
				if(MS->distance_since_startup!=last_distance)display_distance = distance/10;
				else{
					display_distance = 0;
					last_kilometer=MS->distance_since_startup;
				}
				if(distance>1000)last_kilometer=MS->distance_since_startup;
				last_distance=MS->distance_since_startup;
			}
			{
			uint8_t d[8];
			d[0] = MS->SOC;//battery percentage
			d[1] = (uint8_t)display_distance; // in 10m
			d[2] = 0x00;
			d[3] = MS->cadence; //cadence
			d[4] = MS->torque_on_crank&0xFF; //torque mV LSB
			d[5] = (MS->torque_on_crank>>8)&0xFF; //torque mv MSB
			//protocol unit for remaining range is 0.01 km (display divides by 100) -> send km*100
			uint16_t range_x100 = (MS->range < 650) ? (uint16_t)(MS->range*100) : 64999;
			d[6] = range_x100&0xFF;//range LSB (0.01 km)
			d[7] = (range_x100>>8)&0xFF;//range MSB
			can_tx_queue_enqueue(0x02F83200U, 8U, d); //FW-110: was a blocking can_message_transmit/can_transmit_states wait
			}
			break;

		case 0x3205: //to do
			{
			uint8_t d[8] = {0};
#if CAN_DIAGNOSTICS_ENABLE
			/* DIAG-only: live final Iq setpoint replaces calories in 0x3205 bytes 0-1.
			 * MS.calories and int_Temperature are NOT modified — this only changes
			 * what is serialized into this CAN frame.
			 * Range 0..700 (PH_CURRENT_MAX) fits uint16 LE directly. */
			uint16_t cal_val = (uint16_t)(MS->i_q_setpoint);
#else
			uint16_t cal_val = MS->calories;
#endif
			d[0] = cal_val & 0xFF;
			d[1] = (cal_val >> 8) & 0xFF;
			can_tx_queue_enqueue(0x02F83205U, 2U, d); //FW-110: was a blocking can_message_transmit/can_transmit_states wait
			}
			break;


	}//end case
}

/*
 * Controller alive/status broadcasts to the HMI (source=2, target=31). Without them the display
 * does not blink the Walk Assist icon and drops out of WA after a few seconds.
 *
 * FW-133: these three used to go out TOGETHER every 480 ms, because one function sent all of them.
 * The stock descriptor table (BAFANG_CAN_STOCK_VERIFIED_REFERENCE.md SS10) gives each its own, very
 * different period:
 *
 *     0x1200   495 ms
 *     0x320F  1980 ms
 *     0x3000  9900 ms
 *
 * So 0x320F was going out 4x and 0x3000 20x more often than the factory sends them, and all three
 * landed in the SAME tick - three frames at once into a 16-slot queue that silently drops on
 * overflow (FW-132). Split so the caller can schedule each on its own period and normally only
 * one frame is offered per pass.
 */
void sendCAN_status_frame(MotorState_t* MS, uint8_t index){
	static const uint32_t hb_efid[3] = {0x02FF1200, 0x02F8320F, 0x02F83000};
	static const uint8_t  hb_dlen[3] = {1, 8, 4};
	if(index > 2U) return;
	uint8_t d[8] = {0};
	if(index==0) d[0] = (MS->brake_active_flag==SET) ? 0x01 : 0x00; //bit0=brake
	else if(index==1) d[0] = 0x01;
	else { //0x3000: byte0 = session counter, +1 every 10 s since boot (matches the m510
	       //logs 01,02,03...; bytes1-3 stay 0).
	       d[0] = (uint8_t)(control_time_ticks / (CONTROL_TIMEBASE_HZ * 10U)); }
	can_tx_queue_enqueue(hb_efid[index], hb_dlen[index], d); //FW-110: was a blocking wait
}

/* All three at once. Only for the one-shot startup/calibration path, never for the cyclic
 * scheduler - that one owns a separate period per frame (see FW-133 above). */
void sendCAN_status_broadcast(MotorState_t* MS){
	for(uint8_t i=0;i<3;i++) sendCAN_status_frame(MS, i);
}

void sendCAN_Tx(MotorParams_t* MP, MotorState_t* MS){

	switch (Ext_ID_Rx.command){


		// Controller info as MULTIFRAME (like factory firmware) so the HMI shows name/version.
		// Strings kept factory-identical except the customer/serial field (0x6001) -> eVistDrive (eVD) + build version.
		case 0x6000: //manufacturer / hw id (factory: "CR X30P.250.FC 2.1")
			if(Ext_ID_Rx.operation==1){
				tx_data_length=sprintf(tx_data, "CR X30P.250.FC 2.1");
				send_multiframe(Ext_ID_Rx.command, &tx_data[0], tx_data_length);
			}
			break;

		case 0x6001: //serial/customer field -> eVistDrive (eVD) + current build version (factory was "FAKE TAXI ...")
			if(Ext_ID_Rx.operation==1){
				tx_data_length=sprintf(tx_data, "eVD %s", EBICS_BUILD_VERSION);
				while(tx_data_length<20) tx_data[tx_data_length++]=' '; //pad to factory serial length (20)
				send_multiframe(Ext_ID_Rx.command, &tx_data[0], tx_data_length);
			}
			break;

		case 0x6002: //model number shown on HMI (factory: "CR X30P.250.FC")
			if(Ext_ID_Rx.operation==1){
				tx_data_length=sprintf(tx_data, "CR X30P.250.FC");
				send_multiframe(Ext_ID_Rx.command, &tx_data[0], tx_data_length);
			}
			break;

		case 0x62D9: //startup angle used as multiplyer here
			Ext_ID_Tx.command = 0x62D9;
			Ext_ID_Tx.operation = 2; //NORMAL_ACK — factory sends 0x821A62D9 (op=NORMAL_ACK), NOT WRITE. HMI needs this for wheel data.
			Ext_ID_Tx.target = Ext_ID_Rx.source; //reply to the requester (display=3, BESST=5...)
			Ext_ID_Tx.source = 0x02; //controller
			{
			uint32_t efid = Ext_ID_Tx.command+(Ext_ID_Tx.operation<<16)+(Ext_ID_Tx.target<<19)+(Ext_ID_Tx.source<<24);
			uint8_t d[8] = {0};
			d[0] =  (MP->TS_coeff)&0xFF;
			d[1] =  (MP->TS_coeff>>8)&0xFF;
			can_tx_queue_enqueue(efid, 2U, d); //FW-110: was a blocking can_message_transmit/can_transmit_states wait
			}
			break;

		case 0x3203: //FW-076: speed limit + wheel diameter code + circumference
			Ext_ID_Tx.command = 0x3203;
			//Was operation 0 (a WRITE), so the reply looked like the controller writing to
			//the tool rather than answering it, and the request manager had nothing to match.
			Ext_ID_Tx.operation = NORMAL_ACK;
			Ext_ID_Tx.target = Ext_ID_Rx.source; //reply to the requester (display=3, BESST=5...) - was hardcoded 5
			Ext_ID_Tx.source = 0x02; //controller
			{
			uint32_t efid = Ext_ID_Tx.command+(Ext_ID_Tx.operation<<16)+(Ext_ID_Tx.target<<19)+(Ext_ID_Tx.source<<24);
			uint8_t d[8] = {0};
			d[0] = MP->speedLimitx100&0xFF;
			d[1] = (MP->speedLimitx100>>8)&0xFF;
			//Was the constant "A1". The stored code is echoed now, so what the tool reads
			//back is what it wrote — that is the whole point of persisting it.
			d[2] = MP->wheel_diameter_code[0];
			d[3] = MP->wheel_diameter_code[1];
			d[4] = MP->wheel_cirumference&0xFF;
			d[5] = (MP->wheel_cirumference>>8)&0xFF;
			can_tx_queue_enqueue(efid, 6U, d); //FW-110: was a blocking can_message_transmit/can_transmit_states wait
			}
			break;

		//FW-110 v4: 0x6200 removed from here, and from everywhere else reachable from CAN.
		//This switch fires for READ_CMD too (and, via the WRITE_CMD chain's own `else`, for any
		//WRITE this file does not otherwise recognise) - a case here could not tell those apart,
		//which is exactly how a READ used to reach the old post-switch autodetect() gate. There
		//is deliberately NO case 0x6200 anywhere: a READ 0x6200 must do nothing at all, and the
		//only WRITE 0x6200 reply in this file is the single ERROR_ACK in the WRITE_CMD chain.

		case 0x6003: //to do
			/* initialize transmit message */
			if(Ext_ID_Rx.operation==1){
			tx_data_length=sprintf(tx_data, "MMG532.250.CF3YA120681");
			send_multiframe(Ext_ID_Rx.command, &tx_data[0],tx_data_length );
			}
			break;
		case 0x6007: //Error codes read by HMI; report active code (e.g. 10 overtemp). FORMAT TBD - verify vs real HMI
			if(Ext_ID_Rx.operation==1){
				if(MS->error_state) tx_data_length=sprintf(tx_data, "%d", MS->error_state); //active error as ASCII
				else                tx_data_length=sprintf(tx_data, "0");                    //no error
				send_multiframe(Ext_ID_Rx.command, &tx_data[0], tx_data_length);
			}
			break;
		case 0x6010: //reply depends on WHO asks:
			//- HMI display (source=3): factory 4-byte mini config block (0x821B6003 Data:01 00 02 06), NOT Para0.
			//  Sending Para0 (64B multiframe) to the display breaks the HMI info handshake -> Info/Settings blank.
			//- BESST/Canable tool (source=5): full Para0 multiframe - the Full Assist tab reads its data here;
			//  without it the tab stays inactive (nothing downloads).
			if(Ext_ID_Rx.operation==1){
				if(Ext_ID_Rx.source==5){
					send_multiframe(Ext_ID_Rx.command, &Para0[0],64 );
				}else{
					Ext_ID_Tx.command  = 0x6003;              //factory tags this reply as 0x6003, op=3
					Ext_ID_Tx.operation= 3;
					Ext_ID_Tx.target   = Ext_ID_Rx.source;    //reply to requester (display=3)
					Ext_ID_Tx.source   = 0x02;                //controller
					uint32_t efid = Ext_ID_Tx.command+(Ext_ID_Tx.operation<<16)+(Ext_ID_Tx.target<<19)+(Ext_ID_Tx.source<<24);
					uint8_t d[8] = {0x01,0x00,0x02,0x06,0,0,0,0};
					can_tx_queue_enqueue(efid, 4U, d); //FW-110: was a blocking can_message_transmit/can_transmit_states wait
				}
			}
			break;
		case 0x6011: //to do
			/* initialize transmit message */
			if(Ext_ID_Rx.operation==1){
			send_multiframe(Ext_ID_Rx.command, &Para1[0],64 );
			}
			break;
		case 0x6020: //FW-006: read one profile bank (Canable only); rx_data[0] = bank index
			if(Ext_ID_Rx.operation==1 && Ext_ID_Rx.source==5){
				uint8_t bank = (receive_message.rx_dlen>=1) ? receive_message.rx_data[0] : 0;
				if(assist_modes_serialize_bank(bank, &BankBlob[0])){
					send_multiframe(Ext_ID_Rx.command, (char*)&BankBlob[0], ASSIST_BANK_BLOB_LEN);
				}
			}
			break;
		case 0x6023: //FW-010: read global ride-feel tuning (Canable only)
			if(Ext_ID_Rx.operation==1 && Ext_ID_Rx.source==5){
				tuning_config_serialize(&TuningBlob[0]);
				send_multiframe(Ext_ID_Rx.command, (char*)&TuningBlob[0], TUNING_BLOB_LEN);
			}
			break;
#if CAN_DIAGNOSTICS_ENABLE
		case 0x6025: //FW-013: read torque load telemetry + calibration status (Canable diagnostics only)
			if(Ext_ID_Rx.operation==1 && Ext_ID_Rx.source==5){
				torque_input_serialize_telemetry(&TorqueBlob[0]);
				send_multiframe(Ext_ID_Rx.command, (char*)&TorqueBlob[0], TORQUE_TELEMETRY_BLOB_LEN);
			}
			break;
		case 0x6029: //FW-015/017: read ride diagnostics v2 (peak-hold + current) (Canable only)
			if(Ext_ID_Rx.operation==1 && Ext_ID_Rx.source==5){
				const assist_pipeline_telemetry_t* cur = assist_pipeline_telemetry();
				const rider_input_t* rin = rider_input_get();
				uint32_t pas_ms = pas_idle_ticks/4U; if(pas_ms>65535)pas_ms=65535; //~4 ticks/ms @4kHz
				uint8_t flags = (cur->assist_permitted?0x01:0) |
				                (rin->pedaling_active?0x02:0) |
				                (MS->brake_active_flag?0x04:0) |
				                (torque_fault?0x08:0) |
				                (pas_direction_backpedal_confirmed()?0x10:0) |
				                (torque_input_calibration_active()?0x20:0) |
				                ((hmi_seen && hmi_lost_ticks>=COMM_CUT_TICKS)?0x40:0) |
				                (ui_8_PWM_ON_Flag?0x80:0);
				//One pipeline, one source: the assist request before the limiter chain.
				int32_t cur_iqr = cur->iq_request_before_limits;
				if(cur_iqr<0)cur_iqr=0;
				if(cur_iqr>32767)cur_iqr=32767;
				int32_t cur_iqs = MS->i_q_setpoint; if(cur_iqs<0)cur_iqs=0; if(cur_iqs>32767)cur_iqs=32767;
				uint8_t dg[72];
				/*
				 * 'D''G' ver7 (71 B). SAME LENGTH, SAME CRC, NEW MEANINGS for the slots whose
				 * concepts went away with the legacy assist pipeline (Extended Boost state, the
				 * launch/measured-duty crossfade, cadence compensation). The version byte is what
				 * makes that a versioned change rather than a silent reinterpretation: a decoder
				 * that knows only ver6 sees 7 and stops instead of reading Extended Boost numbers
				 * out of the AUTO factor. See protocol/RIDE_DIAGNOSTICS_6029.md for the map.
				 */
				dg[0]=0x44; dg[1]=0x47; dg[2]=7;
				dg[3]=DIAG_ENGINE_ID_RIDE_CORE; //deprecated protocol field, see the define
				uint32_t bcur = diag_peak_motor_w ? ((uint32_t)diag_peak_motor_w*1000000UL)/(MS->Voltage?MS->Voltage:40000) : 0; if(bcur>65535)bcur=65535;
				int32_t iqr = diag_peak_iq_req; if(iqr>32767)iqr=32767; else if(iqr<0)iqr=0;
				dg[4]=diag_peak_cadence; dg[5]=flags;
				dg[6]=diag_peak_torque&0xFF; dg[7]=(diag_peak_torque>>8)&0xFF;
				dg[8]=diag_peak_human_w&0xFF; dg[9]=(diag_peak_human_w>>8)&0xFF;
				dg[10]=diag_peak_support&0xFF; dg[11]=(diag_peak_support>>8)&0xFF;
				dg[12]=diag_peak_motor_w&0xFF; dg[13]=(diag_peak_motor_w>>8)&0xFF;
				dg[14]=bcur&0xFF; dg[15]=(bcur>>8)&0xFF;
				dg[16]=iqr&0xFF; dg[17]=(iqr>>8)&0xFF;
				dg[18]=diag_peak_iq_set&0xFF; dg[19]=(diag_peak_iq_set>>8)&0xFF; //peak setpoint reaching FOC
				dg[20]=MS->Speedx100&0xFF; dg[21]=(MS->Speedx100>>8)&0xFF;
				dg[22]=pas_ms&0xFF; dg[23]=(pas_ms>>8)&0xFF;                       //current pas_idle_ms
				dg[24]=cur->rider_demand_permille&0xFF; dg[25]=(cur->rider_demand_permille>>8)&0xFF; //v7: rider_demand [permille]
				dg[26]=cur_iqr&0xFF; dg[27]=(cur_iqr>>8)&0xFF;                     //current iq request before final ramp (effective)
				dg[28]=cur_iqs&0xFF; dg[29]=(cur_iqs>>8)&0xFF;                     //current iq_setpoint
				dg[30]=cur->assist_base_permille&0xFF; dg[31]=(cur->assist_base_permille>>8)&0xFF; //v7: assist_base [permille]
				int32_t cur_iqm=MS->i_q; if(cur_iqm>32767)cur_iqm=32767; else if(cur_iqm<-32768)cur_iqm=-32768; //measured i_q (signed, actual FOC current)
				dg[32]=cur_iqm&0xFF; dg[33]=(cur_iqm>>8)&0xFF;                     //FW-033: measured i_q (command vs actual test)
				dg[34]=(BC_limit_flag?0x01:0);                                    //FW-033: bit0 = battery-current limiter active
				dg[35]=cur->assist_dynamic_permille&0xFF; dg[36]=(cur->assist_dynamic_permille>>8)&0xFF; //v7: assist_dynamic [permille]
				dg[37]=cur->rider_aggression_permille&0xFF; dg[38]=(cur->rider_aggression_permille>>8)&0xFF; //v7: rider_aggression [permille]
				dg[39]=diag_peak_u_abs&0xFF; dg[40]=(diag_peak_u_abs>>8)&0xFF;                     //peak u_abs (saturates at _U_MAX)
				uint16_t pack_mv=(MS->Voltage>65535)?65535:(uint16_t)MS->Voltage;
				dg[41]=pack_mv&0xFF; dg[42]=(pack_mv>>8)&0xFF;                                     //pack voltage [mV]
				dg[43]=rin->cadence_rpm;                                                           //current cadence (not peak-held)
				//v7: which stage of the ONE limiter chain was binding this tick.
				dg[44]=(uint8_t)((cur->power_limited?0x01:0) | (cur->battery_limited?0x02:0) |
				                 (cur->phase_limited?0x04:0) | (cur->voltage_limited?0x08:0) |
				                 (cur->thermal_limited?0x10:0) | (cur->speed_limited?0x20:0) |
				                 (cur->start_active?0x40:0) | (cur->release_active?0x80:0));
				//v7: the pipeline lifecycle and the adaptive decision. These four answer
				//"which profile was in force, how adaptive was it being, and how much of the
				//request was sustained versus reactive" - the questions tuning actually asks.
				dg[45]=cur->pas_state;                                                   //ap2_pas_state_t
				dg[46]=cur->load_state_permille&0xFF; dg[47]=(cur->load_state_permille>>8)&0xFF; //terrain load [permille]
				dg[48]=cur->auto_factor_permille&0xFF; dg[49]=(cur->auto_factor_permille>>8)&0xFF; //AUTO calm..strong [permille]
				dg[50]=cur->assist_response_permille&0xFF; dg[51]=(cur->assist_response_permille>>8)&0xFF; //assist response [permille]
				dg[52]=cur->profile_id;                                                  //ap2_profile_id_t
				/*
				 * v7: the tuning block. Everything here is LIVE (not peak-held): it has to
				 * answer "at THIS operating point, what were the dynamics and the ceiling, and
				 * what did the chain ask for before the limits" - a peak-hold would mix samples
				 * from different profile states and make the answer unreadable.
				 */
				{
					int32_t v;
					uint16_t u16_uabs = (MS->u_abs<0) ? 0 :
						((MS->u_abs>65535) ? 65535 : (uint16_t)MS->u_abs);
					dg[53]=cur->torque_load_centikg&0xFF; dg[54]=(cur->torque_load_centikg>>8)&0xFF; //calibrated pedal load [centikg]
					v=cur->torque_normalized_permille; if(v<0)v=0; if(v>65535)v=65535;
					dg[55]=v&0xFF; dg[56]=(v>>8)&0xFF;                                              //normalized effort [permille]
					dg[57]=cur->attack_ms&0xFF; dg[58]=(cur->attack_ms>>8)&0xFF;                    //attack in force [ms]
					dg[59]=cur->release_ms&0xFF; dg[60]=(cur->release_ms>>8)&0xFF;                  //release in force [ms]
					dg[61]=cur->max_power_w&0xFF; dg[62]=(cur->max_power_w>>8)&0xFF;                //power ceiling in force [W]
					v=cur->iq_request_before_limits; if(v<0)v=0; if(v>65535)v=65535;
					dg[63]=v&0xFF; dg[64]=(v>>8)&0xFF;                                              //Iq request BEFORE the limiter chain
					dg[65]=cur->motor_power_w&0xFF; dg[66]=(cur->motor_power_w>>8)&0xFF;            //requested motor power [W], live
					dg[67]=u16_uabs&0xFF; dg[68]=(u16_uabs>>8)&0xFF;                                //live u_abs, pairs with dg[43] cadence
				}
				uint16_t c=0xFFFF; for(uint8_t i=0;i<69;i++){c^=(uint16_t)dg[i]<<8; for(uint8_t b=0;b<8;b++)c=(c&0x8000)?((c<<1)^0x1021):(c<<1);}
				dg[69]=c&0xFF; dg[70]=(c>>8)&0xFF;
				//FW-110 v4: diag_peak_reset is NOT set here. send_multiframe() returning true only
				//proves the snapshot was ARMED; the reset must fire only when this exact transfer
				//is CONFIRMED delivered end to end (its last fragment reaches CAN_TRANSMIT_OK),
				//and must NOT fire if it is later aborted - see can_reply_effects.c, applied by
				//main.c's loop. The transfer id is remembered here, at arm time.
				{
					can_multiframe_id_t xfer_id;
					if(send_multiframe_tracked(Ext_ID_Rx.command, (char*)&dg[0], 71, &xfer_id)){
						can_reply_effects_6029_armed(xfer_id);
					}
				}
			}
			break;
		case 0x602D: //FW-126.7: read-only phase-current calibration report (Canable diagnostics only)
			if(Ext_ID_Rx.operation==1 && Ext_ID_Rx.source==5){
				uint8_t cal_dump[66];
				current_cal_serialize_dump(cal_dump);
				send_multiframe(Ext_ID_Rx.command, (char*)&cal_dump[0], sizeof(cal_dump));
			}
			break;
		case 0x602F: //FW-127: read-only current-acquisition evidence (Canable diagnostics only)
			if(Ext_ID_Rx.operation==1 && Ext_ID_Rx.source==5){
				uint8_t rep[64];
				fw127_serialize_report(rep);
				send_multiframe(Ext_ID_Rx.command, (char*)&rep[0], sizeof(rep));
			}
			break;
#endif
		case 0x6028: //FW-014/018: read system status (deprecated engine bytes + full-charge threshold) (Canable only)
			if(Ext_ID_Rx.operation==1 && Ext_ID_Rx.source==5){
				uint8_t sys[8]; sys[0]=0x53; sys[1]=0x59; sys[2]=2; //'S''Y' ver2 (bytes 5..6 = soc_full_pack_10mv)
				sys[3]=DIAG_ENGINE_ID_RIDE_CORE; //deprecated protocol field, see the define
				sys[4]=0xFF;                     //deprecated: no pending engine switch, ever
				sys[5]=MP->soc_full_pack_10mv&0xFF; sys[6]=(MP->soc_full_pack_10mv>>8)&0xFF; //FW-018: 0 = not configured
				uint16_t c=0xFFFF; for(uint8_t i=0;i<7;i++){c^=(uint16_t)sys[i]<<8; for(uint8_t b=0;b<8;b++)c=(c&0x8000)?((c<<1)^0x1021):(c<<1);}
				sys[7]=(uint8_t)(c&0xFF); //1-byte CRC lo (8-byte frame; single-frame status)
				send_multiframe(Ext_ID_Rx.command, (char*)&sys[0], 8);
			}
			break;
		case 0x6012: //to do
			/* initialize transmit message */
			if(Ext_ID_Rx.operation==1){
			//Trailing mini-block AFTER Para2: factory sends 0x821B6012 Data:01 00 02 06 as the
			//"config transfer complete" marker. Without it the HMI never renders the Info/Settings screen.
			//FW-110 v4: the trailer is armed ATOMICALLY with the reply, as a phase of the same
			//stop-and-wait transfer (can_multiframe_start_with_trailer) - it is produced only
			//after the END fragment is confirmed CAN_TRANSMIT_OK. A separate start();set_trailer()
			//pair is gone: the automaton can never run without a properly attached trailer, and a
			//stale trailer cannot leak into a later reply.
			{
				can_multiframe_trailer_t trailer;
				trailer.efid = Ext_ID_Rx.command + (3U << 16) + ((uint32_t)Ext_ID_Rx.source << 19) + (0x02U << 24);
				trailer.dlen = 4U;
				trailer.data[0] = 0x01; trailer.data[1] = 0x00;
				trailer.data[2] = 0x02; trailer.data[3] = 0x06;
				trailer.data[4] = 0;    trailer.data[5] = 0;
				trailer.data[6] = 0;    trailer.data[7] = 0;
				send_multiframe_trailer(Ext_ID_Rx.command, &Para2[0], 64, &trailer);
			}
			}
			break;
#if CAN_DIAGNOSTICS_ENABLE
		case 0x6017: //FW-022: exact Hall calibration telemetry (Canable/BESST only)
			if(Ext_ID_Rx.operation==1 && Ext_ID_Rx.source==5){
				uint8_t hall[37]; //FW-023: format v2 appends the EEPROM record state
				hall[0]='H'; hall[1]='C'; hall[2]=2; hall[3]=MS->hall_angle_detect_flag;
				put_i32_le(&hall[4], i32_hall_order);
				put_i32_le(&hall[8], Hall_13);
				put_i32_le(&hall[12], Hall_32);
				put_i32_le(&hall[16], Hall_26);
				put_i32_le(&hall[20], Hall_64);
				put_i32_le(&hall[24], Hall_45);
				put_i32_le(&hall[28], Hall_51);
				put_i32_le(&hall[32], MP->angle_correction);
				hall[36] = param_record_state;
				send_multiframe(Ext_ID_Rx.command, (char*)&hall[0], sizeof(hall));
			}
			break;
#endif

	}//end case
}

bool send_multiframe(uint16_t command, char* data, uint8_t length ){
	/*
	 * FW-110: delegates to can_multiframe.c's non-blocking producer instead of building and
	 * enqueueing every fragment synchronously in this one call. The old synchronous version
	 * enqueued all of a reply's fragments back to back before can_tx_queue_service() had sent
	 * even the first one - for a 255-byte reply (33 fragments) against a 16-frame queue, that
	 * dropped the last 17 fragments deterministically, on every call, even on an idle bus. See
	 * inc/can_multiframe.h for the full rationale.
	 *
	 * can_multiframe_start() returns false only when a reply is already active; per that
	 * module's contract this function must then send NOTHING - not a partial START, not an
	 * error frame - and let the caller's own retry (the Canable app already retries a timed-out
	 * READ) recover it. That refusal is counted by can_multiframe_rejected_busy_count().
	 *
	 * FW-110 v4: the return value still means "armed", nothing more. A caller with a side effect
	 * that must wait until the reply is CONFIRMED delivered (0x6029's diag_peak_reset) uses
	 * send_multiframe_tracked() and feeds the id to can_reply_effects.c, which resolves it
	 * against the real producer later - it never acts on the arm alone.
	 */
	return can_multiframe_start(command, Ext_ID_Rx.source, 0x02U, (const uint8_t*)data, length, 0);
}

/* FW-110 v4: like send_multiframe(), but also hands back the transfer id of the reply, for a
 * caller that must attach a post-completion effect to THIS specific reply. */
bool send_multiframe_tracked(uint16_t command, char* data, uint8_t length,
                              can_multiframe_id_t *out_id){
	return can_multiframe_start(command, Ext_ID_Rx.source, 0x02U, (const uint8_t*)data, length, out_id);
}

/* FW-110 v4: like send_multiframe(), with the reply's trailing marker attached ATOMICALLY to the
 * same transfer (0x6012's factory "config transfer complete" marker). The trailer is produced
 * only after the END fragment is confirmed sent - never merely queued. */
bool send_multiframe_trailer(uint16_t command, char* data, uint8_t length,
                              const can_multiframe_trailer_t *trailer){
	return can_multiframe_start_with_trailer(command, Ext_ID_Rx.source, 0x02U,
	                                         (const uint8_t*)data, length, trailer, 0);
}

void append_multiframe(uint16_t command, char* data){
	memcpy(data+command*8, &receive_message.rx_data,receive_message.rx_dlen );

}

void update_checksum(void){
	checksum=0;
	for (k=0; k < 63; k++){
		//Para0[k]=k;
		checksum+=Para0[k];
	}
	Para0[63]=checksum%256;
	checksum=0;
	for (k=0; k < 63; k++){
		//Para1[k]=k+64;
		checksum+=Para1[k];
	}
	Para1[63]=checksum%256;
	checksum=0;
	for (k=0; k < 63; k++){
		//Para2[k]=k+128;
		checksum+=Para2[k];
	}
	Para2[63]=checksum%256;
	checksum=0;
}

//int16_t abs(int16_t value){
//
//	if(value>0)return value;
//	else return -value;
//
//}

/*
 * FW-134: cyclic broadcast 0x3210, ~1 s, DLC 8. The factory M510 sends it and we never did.
 *
 * WHY IT EXISTS HERE. Walk Assist on the stock bike shows an icon that APPEARS when the mode is
 * selected and BLINKS while the bike is actually moving. The appearing half needs nothing from us
 * - the display sent 0x6300 itself, so it already knows. The blinking half had never worked on
 * eVistDrive, and the owner's own capture (ON/WA/OFF on the same display) shows why: bytes 4..5 of
 * this frame step +1 once per second, and that window matched the MOVING window exactly - it began
 * 3.4 s after the mode was selected and ended 2.8 s before it was released, while the bike was
 * standing still at both ends with the button still held.
 *
 * So the controller is not silent during Walk Assist after all. It reports motion in a frame we
 * never sent. This is the smallest change that gives the display that signal.
 *
 * WHAT IS HONEST AND WHAT IS A GUESS:
 *   bytes 4..5  a REAL seconds-of-motion counter - the one field whose behaviour the capture
 *               proves. Session-scoped, not a persistent odometer: the reference marks the
 *               physical meaning as unproven, so inventing a stored lifetime value would be
 *               claiming more than we know.
 *   bytes 0..3  constant in the capture (1080 and 4242 on that bike) and documented as
 *               SEMANTIC_UNKNOWN. Sent as zero rather than copied: another bike's constants are
 *               not our data.
 *   bytes 6..7  zero in the capture too.
 */
void sendCAN_3210(void){
	uint8_t d[8] = {0};
	d[4] = (uint8_t)(ride_seconds & 0xFF);        //LE16 seconds of motion
	d[5] = (uint8_t)((ride_seconds >> 8) & 0xFF);
	can_tx_queue_enqueue(0x82F83210, 8, d);
}
