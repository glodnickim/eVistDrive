/*!
    \file    main.c
    \brief   led spark with systick, USART print and key example

   \version 2024-12-20, V3.0.1, firmware for GD32F30x
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "main.h"
#include "FOC.h"
#include "assist_limits.h"
#include "motor_core.h"
#include "motor_service.h"
#include "rider_input.h"
#include "ride_control.h"
#include "torque_input.h"
#include "assist_modes.h"
#include "tuning_config.h"
#include "walk_assist_motor.h"
#include "level_gesture.h"
#include "CAN_Display.h"
#include "parser.h"
#include <math.h>
#include <string.h>
uint16_t adc_value[9];

#define FMC_PAGE_SIZE           ((uint16_t)0x800U)
#define FMC_WRITE_START_ADDR    ((uint32_t)0x0803F000U) //Page 126, Page size 2kB
#define FMC_WRITE_END_ADDR      ((uint32_t)0x0803F800U) //just one page

//--- SOC state persistence: dedicated wear-leveled page 127 (above the param page) ---
#define SOC_FLASH_ADDR          ((uint32_t)0x0803F800U) //Page 127, separate from params
#define SOC_SLOT_SIZE           ((uint32_t)32U)         //bytes per slot
#define SOC_SLOT_WORDS          (SOC_SLOT_SIZE/4U)      //8 words
#define SOC_NUM_SLOTS           (FMC_PAGE_SIZE/SOC_SLOT_SIZE) //64 slots

typedef struct {
	uint32_t seq;             //monotonic sequence (0xFFFFFFFF = empty)
	float    remaining_mah;
	float    used_wh;
	uint16_t capacity_est_mah;
	uint16_t soc_real_x10;    //SOC*10
	uint32_t last_voltage_mv;
	uint32_t cycle_charge_mah;
	uint32_t reserved;
	uint32_t crc;             //crc32 over first 28 bytes
} soc_slot_t;
//#define FMC_OFFSET_PARA0      	((uint32_t)28) //starts after hall angles
//#define FMC_OFFSET_PARA1      	FMC_OFFSET_PARA0 + ((uint32_t)64) //starts after Para1
//#define FMC_OFFSET_PARA2      	FMC_OFFSET_PARA1 + ((uint32_t)64) //starts after Para1
#define FMC_OFFSET_MP			((uint32_t)28) //starts after hall angles

//--- FW-023: commit-last record footer. Written after the payload, so a write cut short
//    by a power loss never produces a record that passes validation.
#define FMC_OFFSET_FOOTER	(FMC_OFFSET_MP + (((uint32_t)sizeof(MotorParams_t)+3U)/4U)*4U)
#define PARAM_REC_MAGIC		((uint32_t)0xEB1C5001U)
#define PARAM_REC_VERSION	((uint16_t)1U)

typedef struct {
	uint32_t magic;    //PARAM_REC_MAGIC
	uint16_t version;  //PARAM_REC_VERSION
	uint16_t length;   //bytes covered by crc (= FMC_OFFSET_FOOTER)
	uint32_t reserved; //0
	uint32_t crc;      //crc32 over the payload; LAST word programmed
} param_footer_t;

//record state reported over 0x6017: 0 = valid, 1 = no valid record (defaults), 2 = halls rejected
#define PARAM_REC_STATE_OK			0U
#define PARAM_REC_STATE_DEFAULTS	1U
#define PARAM_REC_STATE_HALL_BAD	2U
uint8_t param_record_state = PARAM_REC_STATE_DEFAULTS;

//plausible spacing between two neighbouring hall transitions: 60 deg +/- 15 deg in q31 units
#define HALL_GAP_MIN_Q31	((uint32_t)536870925U) //45 deg
#define HALL_GAP_MAX_Q31	((uint32_t)894784875U) //75 deg

uint32_t *ptrd;
uint32_t address = 0x00000000U;
uint32_t data0   = 0x01234567U;
/* calculate the number of page to be programmed/erased */
uint32_t PageNum = (FMC_WRITE_END_ADDR - FMC_WRITE_START_ADDR) / FMC_PAGE_SIZE;
/* calculate the number of words to be programmed/erased */
uint32_t WordNum = ((FMC_WRITE_END_ADDR - FMC_WRITE_START_ADDR) >> 2);

can_trasnmit_message_struct transmit_message;
can_receive_message_struct receive_message;
FlagStatus receive_flag;
FlagStatus PAS_flag=0;
FlagStatus Speed_flag=0;
FlagStatus reg_ADC_flag=0;
FlagStatus OnOffButton_flag=0;
FlagStatus BC_limit_flag=0;
uint8_t Backwards_counter=0;

void nvic_config(void);
void led_config(void);
void gpio_config(void);
void rcu_config(void);
void dma_config(void);
void adc_config(void);
void timer0_config(void); //PWM for Mosfet driver
void timer1_config(void); //PWM for triggering regular ADC
void timer2_config(void); // Hall sensor input (encoder in UVW-mode=
ErrStatus can_networking(void);
void can_networking_init(void);
int32_t speed_PLL (int32_t ist, int32_t soll, uint8_t speedadapt);
void runPIcontrol(void);
void autodetect(void);
int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
void get_standstill_position();
void dyn_adc_state(q31_t angle);
void fmc_program_hall_angles(void);
void fmc_erase_pages(void);
void PAS_processing(void);
void reg_ADC_processing(void);
void UART4_init(void);
int16_t internal_tics_to_speedx100 (uint32_t tics);
int16_t external_tics_to_speedx100 (uint32_t tics);
fmc_state_enum fmc_multi_word_program(uint32_t offset, uint8_t* data, uint8_t words);
void write_virtual_eeprom(void);
void read_virtual_eeprom(void);
uint8_t param_record_valid(void);   //FW-023: magic + version + length + crc of the stored record
uint8_t hall_angles_plausible(void);//FW-023: six transitions ~60 deg apart, order is +/-1
void hall_load_defaults(void);      //FW-023: restore the calibrated values compiled into this build
int8_t calculate_SOC(uint16_t voltage, uint8_t cells_in_series);
#if CAN_DIAGNOSTICS_ENABLE
void print_debug_on_CAN(void);
#endif
//--- SOC / Range ---
void soc_init(void);
void soc_update(void);            //called at ~1 Hz
uint8_t soc_state_load(void);
void soc_state_save(void);
void power_off_controller(void);  //self power-off: save SOC, stop PWM, drop DC/DC + display (button/auto-off/comms watchdog)
uint32_t soc_crc32(const uint8_t* data, uint32_t len);
float compute_limp_factor(float soc);
float default_wh_km_for_level(uint8_t lvl);
void Speed_processing(void);
int16_t T_NTC(uint16_t ADC);
float u32_to_deg=0.00000008381903171539;
uint16_t slow_loop_counter=0;
#if CAN_DIAGNOSTICS_ENABLE
uint16_t t3100_counter=0;
#endif
uint16_t PAS_counter=0;
uint16_t torque_counter=0;
uint8_t overtemp_stage=0;           //thermal protection stage: 0 ok, 1 derate/warn, 2 cutoff
uint8_t wa_engaged=0;              //FW-060: one controller reset per complete WA request
walk_motor_output_t wa_diag;       //FW-060: last WA state for diagnostics 0x10205/0x10206
uint8_t ui8_wa_latch_active=0;
uint8_t ui8_wa_latch_cancel_block=0;
uint8_t ui8_wa_hold_armed=0;
uint8_t ui8_wa_btn_prev=0;
uint8_t ui8_wa_up_prev=0;
uint8_t ui8_wa_down_prev=0;
uint8_t ui8_wa_light_prev=0;
uint8_t ui8_wa_level_prev=0;
uint32_t ui32_wa_latch_ticks=0;
uint16_t err_pulse_counter=0;       //seconds counter for pulsed Error 10 in stage 1
uint16_t Speed_counter=0;
uint8_t coast_wheel_moved=1;  //FW-061: wheel turned since pedalling stopped; starts "moving" so an unknown state is treated as riding
int32_t ButtonVoltageCumulated=620<<6;
#define iabs(x) (((x) >= 0)?(x):-(x))
#define sign(x) (((x) >= 0)?(1):(-1))
MotorState_t MS;
MotorParams_t MP;
_Static_assert(FMC_OFFSET_FOOTER + sizeof(param_footer_t) <= (FMC_WRITE_END_ADDR - FMC_WRITE_START_ADDR),
	"MotorParams_t + record footer no longer fit the virtual EEPROM page");
_Static_assert(sizeof(param_footer_t) == 16, "param_footer_t must stay 4 words with crc last");
/*
 * FW-095: the whole stored record, pinned.
 *
 * The FW-023 validity check includes the record LENGTH, so sizeof(MotorParams_t) is part of
 * the persistent format. Change it by a single byte — add a field, remove one of the orphaned
 * ones listed in main.h, reorder anything — and every record already written to a bike fails
 * validation on the next boot. The controller then lays down defaults and the rider silently
 * loses every setting: both profile banks, the ride-feel tuning, the torque calibration, the
 * wheel code, the full-charge voltage.
 *
 * That is why the dead Legacy parameters are still in the struct. The comment in main.h says
 * so; this assertion is what actually enforces it.
 *
 * If you are changing the persistent format ON PURPOSE, this line is the checklist: bump the
 * record version, write the migration, and only then update the number here.
 */
_Static_assert(sizeof(MotorParams_t) == 724,
	"persistent record size changed: every stored setting on every bike would reset");
/*
 * FW-076: the wheel-diameter code took over the four bytes the ride-engine choice used
 * before FW-030 removed engine selection. It has to stay EXACTLY four bytes in exactly that
 * place: any change to sizeof(MotorParams_t) fails the FW-023 length check on the stored
 * record, and every setting on the bike silently reverts to defaults on the next boot.
 * Asserting the neighbour's offset is what actually pins that down — a same-sized field in
 * the wrong place would still shift everything after it.
 */
_Static_assert(__builtin_offsetof(MotorParams_t, soc_full_magic)
	== __builtin_offsetof(MotorParams_t, wheel_diameter_magic) + 4,
	"wheel diameter must occupy exactly the 4 bytes the ride-engine fields did");
_Static_assert(sizeof(((MotorParams_t*)0)->wheel_diameter_code) == 2,
	"the Bafang wheel code is two raw bytes");
//structs for PI_control
PI_control_t PI_iq;
PI_control_t PI_id;
PI_control_t PI_speed;

uint16_t ui16_timertics=0;
uint8_t ui8_6step_flag=0;
uint8_t ui8_hall_state=0;
uint8_t ui8_hall_state_old=0;
uint8_t ui8_hall_case=0;
uint32_t uint32_tics_filtered=128000;
uint16_t uint16_cadence_filtered=0;
//--- Quadrature PAS decoder state ---
uint8_t pas_qstate=0xFF;     //last quadrature state (0..3), 0xFF=uninit
int8_t pas_fwd_steps=0;      //net forward steps toward one magnet pulse (PAS_STEPS_PER_PULSE)
uint16_t pas_cycle_ticks=0;  //ticks since last forward magnet pulse (for cadence)
uint8_t start_phase=0;       //FW-087: 1 = pedalling has begun but no cadence measured yet (was cadence_seeded)
uint16_t pas_idle_ticks=0;   //ticks since last quadrature transition (for stop detection)
uint16_t pas_last_period_ticks=PAS_STOP_TICKS; //ticks between the two most recent forward transitions -> adaptive stop-timeout basis
uint16_t pas_stop_timeout=PAS_STOP_TICKS; //this tick's adaptive stop threshold, clamp(2*pas_last_period_ticks, PAS_STOP_TICKS, PAS_STOP_TICKS_MAX)
uint8_t forward_pedaling=0;  //1 = cranks turning forward (cadence>0, not reverse, not stopped)
uint8_t fwd_run=0;           //consecutive forward quadrature steps (reset on any backward step or stop) -> jiggle-proof engage gate
volatile uint16_t pas_fwd_accum=0; //FW-027 diag: free-running count of forward quadrature steps (never reset, wraps at 65535). Log analysis diffs consecutive frames; nonzero delta while crank is stopped => phantom (EMI) transitions.
//FW-097 MEASUREMENT ONLY: every reverse quadrature step, latched for the 0x0001020A frame.
//No decision anywhere reads these. See the note at the backward-step branch in the decoder.
uint16_t pas_rev_events=0;        //total reverse steps since boot
uint8_t  pas_rev_last_trans=0;    //(previous qstate << 4) | new qstate
uint16_t pas_rev_last_gap=0;      //pas_idle_ticks at the event: <4 = bounce, ~48 @52rpm = real
uint16_t pas_rev_last_period=0;   //the last genuine forward-step gap, for scale
uint8_t  pas_rev_last_cadence=0;
uint8_t  pas_rev_last_fwdrun=0;   //forward-step run that this reverse step destroyed
uint16_t pas_rev_min_gap=0xFFFF;  //smallest gap ever seen at a reverse step
//FW-098: consecutive-reverse-step run, and the two counters that separate "a reverse step was
//seen" from "the long penalty actually fired".
uint8_t  pas_rev_run=0;           //current unbroken run of reverse steps (any forward step clears it)
uint8_t  pas_rev_run_max=0;       //longest run seen this session
uint16_t pas_rev_latches=0;       //times BACKWARD_CONFIRM_STEPS was reached
/*
 * FW-098 SUCCESS METRIC. The counter dropping is not the point — the point is how much of the
 * time the rider is pedalling forward and getting nothing. Counted in the 4 kHz control tick.
 */
uint32_t metric_pedal_ticks=0;    //ticks with the cranks turning forward
uint32_t metric_iq_zero_ticks=0;  //...of which the motor was given no current at all
//...and why. Not a pure reverse-latch measure on its own: the ordinary start delay counts too.
uint32_t metric_zero_backward=0;   //backward latch was up
uint32_t metric_zero_notlatched=0; //ride latch not armed yet (start conditions, incl. fwd_run)
uint8_t ui8_overflow_flag=0;
uint8_t ui8_SPEED_control_flag=0;
uint8_t ui8_walk_btn_counter=0;
uint8_t ui8_walk_btn_state=0;
uint8_t ui8_wa_speed_paused=0;
uint32_t voltage_raw_cumulated=0;
uint16_t voltage_raw_filtered=0;
//--- torque sensor fault state (zero/drift ownership moved to torque_input) ---
uint8_t  torque_fault=0;        // Error 25 active (out-of-range signal debounced, or implausible rest)
uint16_t tq_fault_ticks=0;      // debounce for out-of-range signal
uint32_t tq_fault_hold=0;       // min hold after the cause clears (~5 s) - no flicker/assist chatter
volatile uint8_t bank_save_request=0; //FW-006: 0x6022 received -> persist banks at next standstill
volatile uint8_t soc_full_persist=0;       //FW-018: 0x602B sets 1 = MP.soc_full_* changed, flash-persist at standstill
#if CAN_DIAGNOSTICS_ENABLE
//FW-015b: peak-hold diagnostics (reset on each 0x6029 read) - lets a brief bench press be captured
volatile uint8_t diag_peak_reset=0;
volatile uint8_t diag_peak_cadence=0;
volatile uint16_t diag_peak_torque=0, diag_peak_human_w=0, diag_peak_support=0, diag_peak_motor_w=0;
volatile uint16_t diag_peak_precomp_motor_w=0, diag_peak_cadence_comp=1000, diag_peak_u_abs=0; //FW-057
volatile int32_t diag_peak_iq_req=0, diag_peak_iq_set=0;
#endif
uint32_t ui32_erps_cumulated=0;
int32_t q31_rotorposition_hall=0;
q31_t q31_rotorposition_absolute=0;
int8_t i8_recent_rotor_direction=0;


uint16_t ui16_tim2_recent=0;
uint16_t uint16_full_rotation_counter=0;
uint16_t uint16_half_rotation_counter=0;
uint16_t speedlimitx100_scaled=0;
int16_t phase_current_max_scaled=0;
int16_t ride_core_iq_limit_scaled=0;
int8_t assist_level_old=0;
q31_t q31_u_d_temp=0;
q31_t q31_u_q_temp=0;
//Hall64	691967230
//Hall26	-11930205
//Hall32	-811271360
//Hall13	-1479377400
//Hall51	2123622926
//Hall45	1348142805

//FW-022: defaults from the 0x6200 calibration measured on this M820 (2026-07-23).
//Angles in q31; spacing is a clean 60 deg: -134, -74, -12, +48, +107, +167.
//Previous (pre-calibration) set: order=-1, 13=1825361405, 32=-1789569490,
//26=-966367405, 64=-322122295, 45=381775140, 51=1169185830.
#define HALL_DEF_ORDER	(1)
#define HALL_DEF_13		(-882854150)  //-74 deg
#define HALL_DEF_32		(-1598682050) //-134 deg
#define HALL_DEF_26		(1992387915)  //+167 deg
#define HALL_DEF_64		(1276560015)  //+107 deg
#define HALL_DEF_45		(572662580)   //+48 deg
#define HALL_DEF_51		(-143165320)  //-12 deg

int32_t i32_hall_order = HALL_DEF_ORDER;
int32_t Hall_13 = HALL_DEF_13;
int32_t Hall_32 = HALL_DEF_32;
int32_t Hall_26 = HALL_DEF_26;
int32_t Hall_64 = HALL_DEF_64;
int32_t Hall_45 = HALL_DEF_45;
int32_t Hall_51 = HALL_DEF_51;

const int32_t one_deg = 11930465; //one degree in 2^32 logic

int32_t i32_full_rotation_flag =-1;

int32_t q31_PLL_error=0;
int32_t q31_rotorposition_PLL=0;
uint8_t ui_8_PLL_counter=0;
uint8_t shutoffcounter=0;
//FW-050: offroadcode / offroadcounter removed — the gesture no longer builds a decimal number.
uint16_t pulse_counter=0;
uint8_t ui_8_PWM_ON_Flag=0;
uint8_t  pwm_cutoff_active=0;    // trwa miekkie zwolnienie stopnia mocy przed DISABLE
uint16_t pwm_cutoff_tick=0;      // licznik cykli okna zwolnienia
uint16_t pwm_cutoff_st[3]={0,0,0}; // snapshot switchtime na starcie okna
int32_t q31_angle_per_tic=0;
//Rotor angle scaled from degree to q31 for arm_math. -180Ã‚Â°-->-2^31, 0Ã‚Â°-->0, +180Ã‚Â°-->+2^31
const int32_t deg_30 = 357913941;
uint16_t switchtime[3];
//FW-042: written in TIMER2_IRQHandler (Hall capture), read in the main loop and by Walk
//Assist. volatile so the compiler cannot cache them — this is the WA "motor stopped" timeout,
//i.e. a safety path.
volatile uint16_t ui16_erps=0;
volatile uint16_t ui16_erps_counter=0;
char char_dyn_adc_state_old=1;
int16_t i16_ph1_current=0;
int16_t i16_ph2_current=0;
int16_t i16_ph3_current=0;

int8_t i8_reverse_flag = 1;
const q31_t tics_lower_limit = WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*SPEEDLIMIT*10); //tics=wheelcirc*timerfrequency/(no. of hallevents per rev*gear-ratio*speedlimit)*3600/1000000
const q31_t tics_higher_limit = WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*(SPEEDLIMIT+2)*10);
uint8_t i = 0;
uint16_t p = 0;
uint32_t timeout = 0xFFFF;
uint8_t transmit_mailbox = 0;
int32_t battery_current_cumulated=0;
//--- SOC / Range runtime globals ---
int32_t bat_current_offset=CAL_BAT_I_OFFSET; //zero-current ADC offset, calibrated at startup
float soc_mAs_acc=0;                 //charge accumulator [mA*s] within current 1s window
uint16_t soc_tick_counter=0;        //counts reg_ADC ticks (~4kHz) towards 1s
uint8_t soc_one_second_flag=0;
uint32_t rest_seconds=0;            //consecutive seconds with |I| < I_REST_MA
uint32_t soc_save_seconds=0;        //seconds since last flash save
float soc_last_saved=0;            //SOC_real at last save
float trip_distance_m_last=0;      //distance marker for range-learning window
float wh_km_level[10]={0};         //per-assist-level learned consumption [Wh/km] -> per-level range
float limp_factor=1.0f;            //motor power scale from limp mode (1.0 = full)
uint32_t soc_seq=0;                //current highest slot sequence
int32_t soc_slot_index=-1;        //index of latest written slot (-1 = none)
float cycle_start_soc=-1.0f;      //SOC at start of a discharge cycle (-1 = none)
float cycle_discharge_mah=0;      //accumulated discharge during the cycle
uint8_t shutdown_saved=0;         //guard: save state only once on shutdown
//--- FW-018: boot-time full-charge detection (pack-voltage threshold -> 100% anchor) ---
uint8_t soc_full_anchor=0;        //1 = SOC display pinned at 100% after a detected full charge
uint8_t soc_boot_full_done=0;     //boot full-charge check finished (runs once)
uint8_t soc_boot_settle_s=0;      //seconds counted inside the boot settle window
uint16_t soc_boot_vmin=0xFFFF;    //min pack voltage seen during the settle window [mV]
uint16_t soc_boot_vmax=0;         //max pack voltage seen during the settle window [mV]
float soc_anchor_start_mah=0;     //remaining_mah captured when the anchor was set
uint32_t idle_ticks_slow=0;       //auto-off: slow-loop (40 ms) ticks with no rider/comms activity
volatile uint16_t comm_lost_ticks=0; //comms watchdog: slow-loop ticks since last HMI frame (reset in processCAN_Rx)
volatile uint8_t comm_seen=0;     //comms watchdog: 1 after first HMI frame -> arms the watchdog (grace period at boot)
uint8_t auto_off_minutes=AUTO_OFF_MINUTES; //runtime auto-off timeout [min]; overwritten by HMI 0x6303
uint32_t Speedx100_cumulated=0;
uint16_t last_valid_speed_x100=0;          //FW-036: baseline for false-pulse (impossible-rise) rejection
volatile uint16_t speed_glitch_count=0;    //FW-036: rejected speed pulses since boot (diagnostics)
uint32_t torque_cumulated=0;
uint8_t array_temp[88];

uint8_t level_to_array_element[10]={0,1,1,2,2,3,3,4,4,5}; //map 10 HMI assist slots to 5 real assist profiles

//--- FW-050: level gestures -------------------------------------------------------------
//Set by the bank gesture, consumed by the standstill-persist block further down.
static uint8_t bank_save_pending=0;
static uint8_t bank_toggle_pending=0;
static uint8_t wa_bank_switch_locked=0;

static void apply_bank_toggle(void)
{
	uint8_t next_bank = assist_modes_get_active_bank() ? 0 : 1;
	assist_modes_set_active_bank(next_bank);
	walk_motor_reset();
	level_gesture_set_splash(next_bank ? 20 : 10);
	bank_save_pending=1;
}

//FW-060: changing the target during WA would step the regulator. Remember the gesture and
//apply it only after the request has ended and the commanded current has reached zero.
static void gesture_toggle_bank(void)
{
	if(wa_bank_switch_locked){
		bank_toggle_pending=1;
		return;
	}
	apply_bank_toggle();
}

//Offroad: lifts the legal speed limit until the bike is switched off. Confirmation 9 = on, 8 = off.
static void gesture_toggle_offroad(void)
{
	MS.offroadflag = MS.offroadflag ? RESET : SET;
	level_gesture_set_splash(MS.offroadflag ? 9 : 8);
}

//The HMI exposes five non-contiguous assist levels: 2, 4, 6, 8 and 9 (plus level 0).
//Reserved adjacent-level gestures only show their two-digit code for now; actions can be
//attached later without changing the detector.
static const level_gesture_t level_gestures[]={
	{ .sequence={2,0,2},   .length=3, .window_ticks=10000, .splash_kmh=0, .action=gesture_toggle_offroad },
	{ .sequence={2,4,2,4}, .length=4, .window_ticks=10000, .splash_kmh=24, .action=0 },
	{ .sequence={4,6,4,6}, .length=4, .window_ticks=10000, .splash_kmh=46, .action=0 },
	{ .sequence={6,8,6,8}, .length=4, .window_ticks=10000, .splash_kmh=68, .action=0 },
	{ .sequence={8,9,8,9}, .length=4, .window_ticks=10000, .splash_kmh=0,  .action=gesture_toggle_bank },
};
_Static_assert(sizeof(level_gestures)/sizeof(level_gestures[0]) <= LEVEL_GESTURE_MAX_COUNT,
	"level_gestures exceeds detector capacity");
//----------------------------------------------------------------------------------------
int32_t ic1value = 0,AngleFromPWM = 0;
__IO uint16_t dutycycle = 0;
__IO uint16_t frequency = 0;



/*!
    \brief      toggle the led every 500ms
    \param[in]  none
    \param[out] none
    \retval     none
*/
void led_spark(void)
{
    static __IO uint32_t timingdelaylocal = 0U;

    if(timingdelaylocal){

        if(timingdelaylocal < 500U){
            gd_eval_led_on(LED2);
        }else{
            gd_eval_led_off(LED2);
        }

        timingdelaylocal--;
    }else{
        timingdelaylocal = 1000U;
    }
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

int main(void)
{
#if (BOOTLOADER== 3)
	nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0x4000); //for bootloader v3

#elif (BOOTLOADER== 38)
	nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0xA800); //for bootloader v3.8

#elif (BOOTLOADER== 820)
	nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0x5000); //for bootloader from M820
#endif


    __enable_irq();

	//SCB->VTOR = 0x08004000;
	fwdgt_config(65000, FWDGT_PSC_DIV256);
#ifdef __FIRMWARE_VERSION_DEFINE
     uint32_t fw_ver = 0;
#endif
    fwdgt_counter_reload();
    //receive_flag = RESET;
    SystemInit();
    /* system clocks configuration */

    rcu_config();
    /* configure systick */
    systick_config();

    /* initialize the LED */
    gd_eval_led_init(LED2);
    gd_eval_hall_init();
    //gd_eval_com_init(EVAL_COM0);
    nvic_config();
    gpio_config();
    /* TIMER configuration */
    timer0_config(); // PWM for Mosfet driver
    timer1_config(); //trigger regular ADC for testing
    timer2_config(); //for hall sensor handling



    /* DMA configuration */
    dma_config();
    /* ADC configuration */
    adc_config();

    /* initialize CAN and CAN filter */
    can_networking_init();

    /* enable CAN receive FIFO0 not empty interrupt */
    receive_flag = RESET;

    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE1);
#ifdef PRINTDEBUG_UART
    //start UART4 for debug messages
    UART4_init();
#endif
    /* initialize transmit message */
    transmit_message.tx_sfid = 0x7ab;
    transmit_message.tx_efid = 0x00;
    transmit_message.tx_ft = CAN_FT_DATA;
    transmit_message.tx_ff = CAN_FF_STANDARD;
    transmit_message.tx_dlen = 8;
    //write_virtual_eeprom();


    //initialize MS struct.
	MS.hall_angle_detect_flag=1;
	MS.Speedx100=0; //in km/h*100
	MS.assist_level=2;
	motor_core_init(&MS);
	motor_command_t initial_motor_command = {0};
	motor_core_set_command(&initial_motor_command);
	ride_control_init();
	level_gesture_init(level_gestures, sizeof(level_gestures)/sizeof(level_gestures[0])); //FW-050
	MS.angle_est=SPEED_PLL;
	MS.pushassist_flag=SET;
	MS.light_flag=SET;
	MS.button_up_flag=SET;
	MS.button_down_flag=SET;
	MS.offroadflag=RESET;
	MS.offroadtics=0;      //FW-050: unused by the gesture engine; kept zeroed for the struct
	MS.bank_splash_kmh=0;  //FW-050: splash now lives in level_gesture.c
	MS.pushassist_flag=RESET;
	MS.walk_can_request=RESET;
	MS.distance_since_startup=0;

	MP.pulses_per_revolution = PULSES_PER_REVOLUTION;
	MP.wheel_cirumference = WHEEL_CIRCUMFERENCE;
	MP.speedLimitx100=SPEEDLIMIT;
	MP.battery_current_max = BATTERYCURRENT_MAX;
	MP.phase_current_max = PH_CURRENT_MAX;
	MP.TS_coeff = TS_COEF;
	MP.reverse = REVERSE;
	MP.MagicNumber=202;
	MP.Override_Duration=8000;
	MP.decay_base=16;
	MP.angle_correction=71582790; //FW-022: 6 deg, phase-2 result of the 0x6200 calibration (was 0)


	//init PI structs
	PI_id.gain_i=I_FACTOR_I_D;
	PI_id.gain_p=P_FACTOR_I_D;
	PI_id.setpoint = 0;
	PI_id.limit_output = _U_MAX;
	PI_id.max_step=15;
	PI_id.shift=11;
	PI_id.limit_i=1800;

	PI_iq.gain_i=I_FACTOR_I_Q;
	PI_iq.gain_p=P_FACTOR_I_Q;
	PI_iq.setpoint = 0;
	PI_iq.limit_output = _U_MAX;
	PI_iq.max_step=15;
	PI_iq.shift=11;
	PI_iq.limit_i=_U_MAX;

    //FW-023: no trustworthy record (never written, half-written, or written by a build with a
    //different MotorParams_t layout) -> lay down a fresh one built from the defaults above.
    if(!param_record_valid()){
    	InitEEPROM(&MP);
    }
    //read parameters from virtual EEPROM and overwrite the default values
    read_virtual_eeprom();
    parse_MOparams(&MP);
	//FW-030/dev: force the fixed phase ceiling (700) regardless of any stored Para1[9], so the
	//software value always wins. Battery still protected at BATTERYCURRENT_MAX by the PI limiter.
	MP.phase_current_max = PH_CURRENT_MAX;
    ride_core_iq_limit_scaled = MP.phase_current_max;
	    torque_input_init(); //MP.torque_full_scale_native is deprecated (no magic/version); user span arrives with the calibration persist block
	    //FW-013/FW-016/FW-077: restore calibration before v1..v6 bank mV -> kg migration.
	    //The migration and live control must use the same sensor span.
	    torque_input_restore_persist(MP.torque_cal_magic, MP.torque_cal_version,
	        MP.torque_cal_span_native, MP.torque_cal_crc);
	    assist_modes_init();
    assist_modes_seed_wa_defaults(MP.walk_assist_current, MP.walk_assist_speed); //FW-051: v1 bank migration seed
    if(MP.bank_store_magic==0xB16B){ //FW-006: restore user bank configs (bad blobs are rejected -> defaults stay)
        //FW-068/069: pass the STORE size, not the current wire length. The blob carries its own
        //version and record length, so a record written by an older build still validates.
        assist_modes_apply_bank_blob(&MP.bank_store[0][0], sizeof(MP.bank_store[0]));
        assist_modes_apply_bank_blob(&MP.bank_store[1][0], sizeof(MP.bank_store[1]));
    }
    assist_modes_set_active_bank((uint8_t)MP.active_profile_bank);
    if(MP.tuning_store_magic==0x7501){ //FW-010: restore user ramp/boost tuning (bad blob rejected -> defaults stay)
        tuning_config_apply_blob(&MP.tuning_store[0], sizeof(MP.tuning_store));
    }
    //FW-030: engine selection removed (ride core only) — no ride-engine restore.
    //FW-076: the same four bytes now hold the Bafang wheel-diameter code. A controller
    //flashed before this change has the old ride-engine content there (magic 0 or 0x5E01),
    //which is not a wheel code — the magic is what tells the two apart. Anything that is
    //not our magic becomes the 27.5" default, so the app never reads a nonsense wheel.
    if(MP.wheel_diameter_magic!=WHEEL_DIAMETER_MAGIC){
        MP.wheel_diameter_magic=WHEEL_DIAMETER_MAGIC;
        MP.wheel_diameter_code[0]=WHEEL_DIAMETER_CODE_0;
        MP.wheel_diameter_code[1]=WHEEL_DIAMETER_CODE_1;
    }

    for (int i = 0; i < 2000; i++) {//let the ADC stabilize
    	while(!reg_ADC_flag);
    	reg_ADC_flag=0;
    }

    {// torquesensor zero: average 64 samples; torque_input owns normalization + sanity check
        int32_t acc=0;
        for (int i = 0; i < 64; i++) {
            acc+=adc_value[2];
            while(!reg_ADC_flag);
            reg_ADC_flag=0;
        }
        torque_input_startup_zero(((acc>>6)*3300)>>12);
    }
  //  while((adc_value[1])>3000);//safety for bricked throttle

    //autodetect();

    // calibrate battery current zero offset (motor off, ~no load) for coulomb counting
    {
        int32_t acc=0;
        for (int i = 0; i < 64; i++){
            acc+=adc_value[0];
            while(!reg_ADC_flag);
            reg_ADC_flag=0;
        }
        acc>>=6;
        if(acc>CAL_BAT_I_OFFSET-200 && acc<CAL_BAT_I_OFFSET+200) bat_current_offset=acc;
    }
    // settle voltage/current filters, then seed SOC from flash or open-circuit voltage
    for (int i = 0; i < 256; i++){
        while(!reg_ADC_flag);
        reg_ADC_processing();
    }
    soc_init();

    while (1){
    	fwdgt_counter_reload();

#if (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG)
    	if(receive_flag){
    		receive_flag = RESET;
    		processCAN_Rx(&MP, &MS);
    	}

#endif
    	//FW-050: the offroad decimal-code accumulator is gone (see level_gesture.c).
    	//if(PAS_flag)PAS_processing(); //disabled: cadence/direction now from quadrature decoder in reg_ADC_processing
    	PAS_flag=0;
    	if(Speed_flag)Speed_processing();
    	if(reg_ADC_flag)reg_ADC_processing();




    	// switch lights
    	if(MS.light_flag&&!gpio_input_bit_get(GPIOB,GPIO_PIN_10))GPIO_BOP(GPIOB) = GPIO_PIN_10;
    	if(!MS.light_flag&&gpio_input_bit_get(GPIOB,GPIO_PIN_10)) GPIO_BC(GPIOB) = GPIO_PIN_10;

    	//check brake sensor state
    	if(!gpio_input_bit_get(GPIOC,GPIO_PIN_13))MS.brake_active_flag=1;
    	else MS.brake_active_flag=0;
    	//FW-049: these three MUST be recomputed continuously, not only when the assist level
    	//changes. They depend on values that change independently of the level: speedLimitx100
    	//(HMI/Canable 0x3203), assist_settings (Para1 write) and limp_factor (SoC). Computing
    	//them only on a level change meant a newly written speed limit did nothing until the
    	//rider happened to switch levels, and after a restart speedlimitx100_scaled stayed 0 —
    	//which with the legal flag on cuts assist from ~2 km/h. Same staleness hit the ride-core
    	//current limit and the low-SoC limp mode. Cost here is a few multiplies per main loop.
    	{
    		uint8_t lvl_idx = level_to_array_element[MS.assist_level];
    		speedlimitx100_scaled=MP.speedLimitx100*MP.assist_settings[lvl_idx][1]/100;
    		phase_current_max_scaled=MP.phase_current_max*MP.assist_settings[lvl_idx][0]/100;
    		ride_core_iq_limit_scaled=(int16_t)((float)MP.phase_current_max*limp_factor);
    	}
    	// per-level settings + trip/offroad bookkeeping: only on an actual level change
    	if(MS.assist_level!=assist_level_old){
    		//range learns per level -> reset the learning window so a window stays within one level
    		trip_distance_m_last=MS.distance_since_startup; MS.used_wh=0;
        	MS.TQfilter=level_to_array_element[MS.assist_level];
        	MS.TQfilter=MP.assist_settings[MS.TQfilter][2];
        	//SAFETY: TQfilter is used as a bit-shift (torque_cumulated>>TQfilter). Ride-mode values >7 (or, via
        	//int8_t, negative) make the shift undefined -> torque_filtered=0 -> that level's assist dies (hit S+/Boost).
        	if(MS.TQfilter<1 || MS.TQfilter>7) MS.TQfilter=4;
        	//FW-050: the offroad gesture moved to the shared level_gesture engine. The old code
        	//built a decimal number with pow(10, offroadtics) into a uint16_t, and offroadtics
        	//doubled as the display splash value — so after a toggle it was set to 8/9 and the
        	//next level change computed ~10^9 into a 16-bit variable (undefined behaviour that
        	//could re-trigger the toggle by itself). No arithmetic is involved any more.
    		assist_level_old=MS.assist_level;

    	}

#if CAN_DIAGNOSTICS_ENABLE
            if(t3100_counter > 40){ t3100_counter=0; sendCAN_3100(&MS); } //40/4000Hz=10ms torque sensor emulation (dev telemetry - OFF by default; floods bus & can block HMI info at startup)
#endif

            if (slow_loop_counter > 160){ //slow loop base tick 40ms (160/4000Hz); CAN messages use own counters
            	gd_eval_led_toggle(LED2);
#ifdef PRINTDEBUG_UART

            	//printf("%d, %d, %d, %d, %d\r\n",MS.Battery_Current,MS.i_q_setpoint,MP.reverse*MS.i_q,ui16_erps,temp2);
            	printf("%d, %d, %d, %d, %d\r\n",MS.Battery_Current,MS.i_q_setpoint,MP.reverse*MS.i_q,MS.p_human,MS.Speedx100);
#endif

#if CAN_DIAGNOSTICS_ENABLE
            	print_debug_on_CAN();
#endif
            	p++;

            	static uint8_t hb_tick=0, speed_tick=0, cad_tick=0, misc_tick=0, s202_tick=0;
            	if(++hb_tick    >=12){hb_tick=0;    sendCAN_status_broadcast(&MS);}     //12x40ms=480ms  (orig 490ms)
            	if(++speed_tick >= 7){speed_tick=0; sendCAN_Poll(&MP,&MS,0x3201);}   // 7x40ms=280ms  (orig 280ms)
            	if(++cad_tick   >=37){cad_tick=0;   sendCAN_Poll(&MP,&MS,0x3200);}   //37x40ms=1480ms (orig 1500ms)
            	if(++misc_tick  >= 8){misc_tick=0;  sendCAN_Poll(&MP,&MS,0x3205);}
            	if(++s202_tick >= 3){s202_tick=0;  sendCAN_3202();}              // 3x40ms=120ms  (orig 100ms)
            	// filtr EMA /16 surowego ADC temperatury (wzorzec jak filtr napiecia), tlumi szum/glitch
            	// TODO(temp-sensor): detekcja rozwartego (ADC~4095) / zwartego (ADC~0) NTC i fail-safe
            	static uint32_t temp_adc_cumulated = 0;
            	if(temp_adc_cumulated == 0) temp_adc_cumulated = (uint32_t)adc_value[6] << 4; //init, brak zimnego startu od 0
            	temp_adc_cumulated -= temp_adc_cumulated >> 4;
            	temp_adc_cumulated += adc_value[6];
            	MS.int_Temperature = T_NTC(temp_adc_cumulated >> 4) + TEMP_OFFSET_C; //offset at source -> affects CAN/thermal/HMI
            	if(soc_one_second_flag){
            		soc_one_second_flag=0;
            		soc_update(); //1 Hz: coulomb -> SOC_real, OCV correction, SOC_display, range, periodic save
            		//apply limp-mode power scaling to the phase current limit (recompute base to avoid compounding)
            		limp_factor = compute_limp_factor(MS.soc_display);
            		{
            			uint16_t base_phase = MP.phase_current_max*MP.assist_settings[level_to_array_element[MS.assist_level]][0]/100;
					phase_current_max_scaled = (int16_t)((float)base_phase * limp_factor);
					ride_core_iq_limit_scaled = (int16_t)((float)MP.phase_current_max * limp_factor);
            		}
            		//--- thermal protection state (hysteresis) + Error 10 signalling ---
            		if(overtemp_stage==0){
            			if(MS.int_Temperature>=TEMP_CUTOFF) overtemp_stage=2;
            			else if(MS.int_Temperature>=TEMP_WARN) overtemp_stage=1;
            		} else { //already warm: drop only below TEMP_CLEAR (hysteresis)
            			if(MS.int_Temperature>=TEMP_CUTOFF) overtemp_stage=2;
            			else if(MS.int_Temperature<TEMP_CLEAR) overtemp_stage=0;
            			else overtemp_stage=1;
            		}
            		if(overtemp_stage==2){ MS.error_state=ERR_OVERTEMP; err_pulse_counter=0; } //solid (highest priority)
            		else if(torque_fault){ MS.error_state=ERR_TORQUE; err_pulse_counter=0; } //torque sensor signal failure
            		else if(overtemp_stage==1){ //pulsed: ON for ERR_PULSE_ON_S, OFF for ERR_PULSE_OFF_S (HMI blinks)
            			if(++err_pulse_counter>=(ERR_PULSE_ON_S+ERR_PULSE_OFF_S)) err_pulse_counter=0;
            			MS.error_state=(err_pulse_counter<ERR_PULSE_ON_S)?ERR_OVERTEMP:0;
            		} else { MS.error_state=0; err_pulse_counter=0; }
            	}
            	//toggle speed pin
            	//gpio_bit_write(GPIOB, GPIO_PIN_0,(bit_status)(1-gpio_input_bit_get(GPIOB, GPIO_PIN_0)));
            	//Speed display: hard zero after SPEED_STOP_TICKS of silence (~2.65 s, min ~3 km/h);
            	//between pulses cap the shown speed at the value implied by the silence so far (+25% grace),
            	//so braking reads as a smooth fall instead of a value frozen until the timeout.
            	if(Speed_counter>SPEED_STOP_TICKS){ MS.Speedx100=0; last_valid_speed_x100=0; } //FW-036: clear baseline so first pulse after a stop isn't rejected
            	else if(MS.Speedx100>0 && Speed_counter>400){ //>0.1 s since last pulse (guards div and leaves fresh pulses alone)
            		uint32_t implied_x100 = (uint32_t)MP.wheel_cirumference*4*360/((uint32_t)MP.pulses_per_revolution*Speed_counter);
            		if((uint32_t)MS.Speedx100*100 > implied_x100*(100+SPEED_DECAY_MARGIN_PCT)){ MS.Speedx100=(uint16_t)implied_x100; last_valid_speed_x100=MS.Speedx100; } //FW-036: track decayed baseline
            	}
				slow_loop_counter = 0;

				if(adc_value[5]<2800)shutoffcounter++; //raw value is 4095 without button pressed, about 3300 with "down" button pressed and about 2400 with on/off button pressed.
				else shutoffcounter=0;
				if(shutoffcounter>62){ //62x40ms=2480ms, poprzednio 50x50ms=2500ms
					power_off_controller(); //on/off button held -> self power-off
				}

				//--- Auto-off after inactivity: reset counter on any rider/comms activity, else count up.
				//FW-087: start_phase counts as activity. It used to be covered implicitly by the
				//fake 1 rpm sitting in MS.cadence; pedalling that has begun but not yet produced a
				//cadence reading must not look like inactivity to the auto-off timer.
				if(MS.Speedx100>0 || MS.cadence>0 || start_phase || MS.i_q_setpoint>0 || MS.brake_active_flag || adc_value[5]<3300){
					idle_ticks_slow=0;
				}
				else if(idle_ticks_slow < 0xFFFFFFFF){
					idle_ticks_slow++;
				}
				// threshold = minutes * 60 s * (1000/40) ticks per s = minutes * 1500 ticks
				if(auto_off_minutes>0 && idle_ticks_slow >= (uint32_t)auto_off_minutes*1500){
					power_off_controller(); //no activity for auto_off_minutes -> self power-off
				}

				//--- Comms watchdog: armed only after the first HMI frame (comm_seen) so it never
				//    fires during the boot grace period before the display starts talking.
				//    comm_lost_ticks is reset in processCAN_Rx on each HMI frame.
				if(comm_seen){
					if(comm_lost_ticks < 60000) comm_lost_ticks++;
					if(comm_lost_ticks >= COMM_CUT_TICKS){ //3 s no HMI frame -> kill assist (fail-safe: broken cable / dead HMI)
						MS.assist_level=0;
						motor_command_t comm_stop_command = {
							.iq_target = 0,
							.id_target = MS.i_d_setpoint,
							.enable = true,
							.emergency_stop = false
						};
						motor_core_set_command(&comm_stop_command);
					}
					if(comm_lost_ticks >= COMM_OFF_TICKS && MS.Speedx100==0){ //10 s no HMI frame -> power off, but only at standstill
						power_off_controller();
					}
				}

            }//end slow loop

            if(MS.i_q_setpoint){
            	if(!ui_8_PWM_ON_Flag){
            		pwm_cutoff_active=0;        //przerwij ewentualne miekkie zwolnienie - wracamy do FOC
            		get_standstill_position();
            		//FW-035: bumpless enable. PI_control slews PI.out by max_step, so a stale
            		//.out from before the bridge was disabled would kick the gearbox on the first
            		//FOC cycle after re-enable. Start every bridge-on from a clean neutral state:
            		//zero both regulators and phase voltages, force 50/50 PWM, THEN enable. FOC then
            		//ramps up through the existing Iq ramp. (FW-028 already zeroed integral_part.)
            		PI_iq.integral_part=0; PI_iq.out=0;
            		PI_id.integral_part=0; PI_id.out=0;
            		MS.u_q=0; MS.u_d=0; MS.u_abs=0;
            		switchtime[0]=_T>>1; switchtime[1]=_T>>1; switchtime[2]=_T>>1;
            		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,_T>>1);
            		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,_T>>1);
            		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,_T>>1);
					timer_primary_output_config(TIMER0,ENABLE);
					uint16_half_rotation_counter=0;
					ui_8_PWM_ON_Flag=1;
            	}
            }
#if SOFT_CUTOFF_ENABLE
            //miekkie zwolnienie: zjedz napiecia faz do neutral (_T/2) przez SOFT_CUTOFF_TICKS cykli, dopiero potem DISABLE
            if(uint16_half_rotation_counter>POWER_STAGE_STOP_TICKS && ui_8_PWM_ON_Flag && !pwm_cutoff_active){
            	ui_8_PWM_ON_Flag=0;            //stop nadpisywania switchtime przez FOC; mostek zostaje ENABLE
            	pwm_cutoff_st[0]=(uint16_t)switchtime[0];
            	pwm_cutoff_st[1]=(uint16_t)switchtime[1];
            	pwm_cutoff_st[2]=(uint16_t)switchtime[2];
            	pwm_cutoff_tick=0;
            	pwm_cutoff_active=1;
            }
            if(pwm_cutoff_active){
            	//pwm_cutoff_tick jest inkrementowany w reg_ADC_processing (~4 kHz), nie tutaj (petla glowna jest szybsza)
            	if(pwm_cutoff_tick>=SOFT_CUTOFF_TICKS){
            		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,_T>>1);
            		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,_T>>1);
            		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,_T>>1);
            		timer_primary_output_config(TIMER0,DISABLE); //dopiero teraz odetnij mostek
            		i8_recent_rotor_direction=0;
            		PI_iq.integral_part=0;
            		PI_id.integral_part=0;
            		pwm_cutoff_active=0;
            	}else{
            		int32_t neutral=_T>>1;
            		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,(uint16_t)(pwm_cutoff_st[0]+(neutral-(int32_t)pwm_cutoff_st[0])*pwm_cutoff_tick/SOFT_CUTOFF_TICKS));
            		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,(uint16_t)(pwm_cutoff_st[1]+(neutral-(int32_t)pwm_cutoff_st[1])*pwm_cutoff_tick/SOFT_CUTOFF_TICKS));
            		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,(uint16_t)(pwm_cutoff_st[2]+(neutral-(int32_t)pwm_cutoff_st[2])*pwm_cutoff_tick/SOFT_CUTOFF_TICKS));
            	}
            }//end soft cut-off
#else
            if(uint16_half_rotation_counter>POWER_STAGE_STOP_TICKS) {
            	if(ui_8_PWM_ON_Flag){
					timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,_T>>1);
					timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,_T>>1);
					timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,_T>>1);
					timer_primary_output_config(TIMER0,DISABLE); //Disable PWM if motor is not turning
					ui_8_PWM_ON_Flag=0;
					i8_recent_rotor_direction=0;
					PI_iq.integral_part=0;
					PI_id.integral_part=0;

            	}
            }//end half rotation counter
#endif


    }
}

/*!
    \brief      configure the different system clocks
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    /* enable GPIOC clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    /* enable GPIOA clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA0);
    /* enable TIMER0 clock */
    rcu_periph_clock_enable(RCU_TIMER0);
    /* enable ADC0 clock */
    rcu_periph_clock_enable(RCU_ADC0);
    /* enable ADC1 clock */
    rcu_periph_clock_enable(RCU_ADC1);
    /* enable ADC1 clock */
    rcu_periph_clock_enable(RCU_ADC2);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);
}

/*!
    \brief      initialize CAN and filter
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_networking_init(void)
{
    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;
    /* initialize CAN structures */
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    /* initialize CAN register */
    can_deinit(CAN0);

    /* initialize CAN */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_7TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;
    /* baudrate 1Mbps */
    can_parameter.prescaler = 24;
    can_init(CAN0, &can_parameter);

    /* initialize filter */
    /* CAN0 filter number */
    can_filter.filter_number = 0;

    /* initialize filter */
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;
    can_filter.filter_fifo_number = CAN_FIFO1;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);


}

/*!
    \brief      configure GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    /* enable can clock */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);

    rcu_periph_clock_enable(RCU_AF);
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
    /* configure CAN0 GPIO, CAN0_TX(PA12) and CAN0_RX(PA11) */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
	
    /* config the GPIO as analog mode */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_3|GPIO_PIN_4); //Battery Voltage
    gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_1); // Motor Temp

    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_12);

	//delay_1ms(200);
    GPIO_BOP(GPIOB) = GPIO_PIN_4; //set Pin4 (set by Bootloader on BL38)
    //delay_1ms(200);
    GPIO_BOP(GPIOB) = GPIO_PIN_3; //12V on
    //delay_1ms(200);
    GPIO_BOP(GPIOB) = GPIO_PIN_5; // Display on
    //delay_1ms(200);
	//GPIO_BOP(GPIOB) = GPIO_PIN_6; //DC/DC on

    //PD2 Dual PAS2 input pin
    gpio_init(GPIOD, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    //PC0 light short circuit detectionß1
    gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
	//PC10 PAS1 (white), PC11 PAS2 (pink), PC13 brake
    gpio_init(GPIOC, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13);
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOC, GPIO_PIN_SOURCE_12); //Pas1 interrupt
    /* configure key EXTI line */
    exti_init(EXTI_12, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_12);

//    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOC, GPIO_PIN_SOURCE_8); //Encoder z-Pulse interrupt
//    exti_init(EXTI_8, EXTI_INTERRUPT, EXTI_TRIG_RISING);
//    exti_interrupt_flag_clear(EXTI_8);
    //PB2 for external speed sensor
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_2);
    /* configure key EXTI line */
    exti_init(EXTI_2, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_2);

    /*configure PA8 PA9 PA10(TIMER0 CH0 CH1 CH2) as alternate function*/
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_8);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_10);

    /*configure PB13 PB14 PB15(TIMER0 CH0N CH1N CH2N) as alternate function*/
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_13);
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_14);
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_15);

}
/*!
    \brief      configure the DMA peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dma_config(void)
{
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;

    /* ADC DMA_channel configuration */
    dma_deinit(DMA0, DMA_CH0);

    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA(ADC0));
    dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr  = (uint32_t)(&adc_value);
    dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number       = 9;
    dma_data_parameter.priority     = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH0, &dma_data_parameter);

    dma_circulation_enable(DMA0, DMA_CH0);

    /* enable DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);
}
/*!
    \brief      configure the ADC peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_config(void)
{

    /* configure the ADC sync mode */
    adc_mode_config(ADC_DAUL_INSERTED_PARALLEL_REGULAL_FOLLOWUP_FAST);
    /* ADC scan mode function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);
    adc_special_function_config(ADC1, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, DISABLE);
    adc_special_function_config(ADC2, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC2, ADC_CONTINUOUS_MODE, DISABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
    adc_data_alignment_config(ADC2, ADC_DATAALIGN_RIGHT);

    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL,9);
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL,1);
    adc_channel_length_config(ADC1, ADC_INSERTED_CHANNEL,1);
    adc_channel_length_config(ADC2, ADC_INSERTED_CHANNEL,1);

    /* ADC regular channel config */
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_239POINT5); // PA0 Battery Current
    adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_6, ADC_SAMPLETIME_239POINT5); // PA6 Throttle?
    adc_regular_channel_config(ADC0, 2, ADC_CHANNEL_7, ADC_SAMPLETIME_239POINT5); // PA7 Torque
    adc_regular_channel_config(ADC0, 3, ADC_CHANNEL_13, ADC_SAMPLETIME_239POINT5);// PC3 battery voltage
    adc_regular_channel_config(ADC0, 4, ADC_CHANNEL_5, ADC_SAMPLETIME_239POINT5); // Phase3 current
    adc_regular_channel_config(ADC0, 5, ADC_CHANNEL_4, ADC_SAMPLETIME_239POINT5); // on/off button
    adc_regular_channel_config(ADC0, 6, ADC_CHANNEL_9, ADC_SAMPLETIME_239POINT5);//Motor temperature
    adc_regular_channel_config(ADC0, 7, ADC_CHANNEL_2, ADC_SAMPLETIME_239POINT5);// Phase1 current
    adc_regular_channel_config(ADC0, 8, ADC_CHANNEL_3, ADC_SAMPLETIME_239POINT5);// Phase2 current

    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_5, ADC_SAMPLETIME_55POINT5);
    adc_inserted_channel_offset_config(ADC0, ADC_INSERTED_CHANNEL_0, 2012); //hardcoded, to be improved

    adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_3, ADC_SAMPLETIME_55POINT5);
//    adc_inserted_channel_config(ADC1, 1, ADC_CHANNEL_5, ADC_SAMPLETIME_55POINT5);

    adc_inserted_channel_offset_config(ADC1, ADC_INSERTED_CHANNEL_0, 2028); //hardcoded, to be improved
//    adc_inserted_channel_offset_config(ADC1, ADC_INSERTED_CHANNEL_1, 2033); //hardcoded, to be improved


    adc_inserted_channel_config(ADC2, 0, ADC_CHANNEL_2, ADC_SAMPLETIME_55POINT5);
    adc_inserted_channel_offset_config(ADC2, ADC_INSERTED_CHANNEL_0, 2020); //hardcoded, to be improved


    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_T1_CH1);
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T0_CH3);
    adc_external_trigger_source_config(ADC1, ADC_INSERTED_CHANNEL, ADC0_1_2_EXTTRIG_INSERTED_NONE);
    adc_external_trigger_source_config(ADC2, ADC_INSERTED_CHANNEL, ADC2_EXTTRIG_INSERTED_T0_CH3);
    /* ADC external trigger enable */
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
    adc_external_trigger_config(ADC1, ADC_INSERTED_CHANNEL, ENABLE);
    adc_external_trigger_config(ADC2, ADC_INSERTED_CHANNEL, ENABLE);

    /* enable ADC interface */
    adc_enable(ADC0);
    delay_1ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
    /* enable ADC interface */
    adc_enable(ADC1);
    delay_1ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC1);
//     /* enable ADC interface */
    adc_enable(ADC2);
    delay_1ms(1);
//    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC2);
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOC);
    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOIC);
    /* enable ADC interrupt */
    adc_interrupt_enable(ADC1, ADC_INT_EOIC);

    /* ADC DMA function enable */
    adc_dma_mode_enable(ADC0);
}

/*!
    \brief      configure the timer peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void timer0_config(void)
{
	/* -----------------------------------------------------------------------
	    TIMER0 configuration to:
	    generate 3 complementary PWM signals with 3 different duty cycles:
	    TIMER0CLK is fixed to systemcoreclock, the TIMER0 prescaler is equal to 6000 so the
	    TIMER0 counter clock used is 20KHz.
	    the three duty cycles are computed as the following description:
	    the channel 0 duty cycle is set to 25% so channel 1N is set to 75%.
	    the channel 1 duty cycle is set to 50% so channel 2N is set to 50%.
	    the channel 2 duty cycle is set to 75% so channel 3N is set to 25%.
	  ----------------------------------------------------------------------- */
	    timer_oc_parameter_struct timer_ocintpara;
	    timer_parameter_struct timer_initpara;
	    timer_break_parameter_struct timer_breakpara;
	    rcu_periph_clock_enable(RCU_TIMER0);

	    timer_deinit(TIMER0);

	    /* TIMER0 configuration */
	    timer_initpara.prescaler         = 0;
	    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_BOTH;
	    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
	    timer_initpara.period            = _T; //for 32 kHz center aligned --> 16kHz PWM frequency
	    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
	    timer_initpara.repetitioncounter = 0;
	    timer_init(TIMER0,&timer_initpara);

	     /* CH1,CH2 and CH3 configuration in PWM mode */
	    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
	    timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
	    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;//inverted logic to make ouput in timer logic
	    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_LOW;
	    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
	    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;

	    timer_channel_output_config(TIMER0,TIMER_CH_0,&timer_ocintpara);
	    timer_channel_output_config(TIMER0,TIMER_CH_1,&timer_ocintpara);
	    timer_channel_output_config(TIMER0,TIMER_CH_2,&timer_ocintpara);
	    timer_channel_output_config(TIMER0,TIMER_CH_3,&timer_ocintpara);

	    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,_T>>1);//grün
	    timer_channel_output_mode_config(TIMER0,TIMER_CH_0,TIMER_OC_MODE_PWM0);
	    timer_channel_output_shadow_config(TIMER0,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

	    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,_T>>1);//gelb
	    timer_channel_output_mode_config(TIMER0,TIMER_CH_1,TIMER_OC_MODE_PWM0);
	    timer_channel_output_shadow_config(TIMER0,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

	    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,_T>>1);//blau
	    timer_channel_output_mode_config(TIMER0,TIMER_CH_2,TIMER_OC_MODE_PWM0);
	    timer_channel_output_shadow_config(TIMER0,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);

	    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,(TRIGGER_DEFAULT));//(_T>>1)+500 in the middle of the PWM cycle
	    timer_channel_output_mode_config(TIMER0,TIMER_CH_3,TIMER_OC_MODE_PWM0);
	    timer_channel_output_shadow_config(TIMER0,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);
	    timer_automatic_output_disable(TIMER0);
	    /* automatic output enable, break, dead time and lock configuration*/
	    timer_breakpara.runoffstate      = TIMER_ROS_STATE_DISABLE;
	    timer_breakpara.ideloffstate     = TIMER_IOS_STATE_DISABLE ;
	    timer_breakpara.deadtime         = 32;
	    timer_breakpara.breakpolarity    = TIMER_BREAK_POLARITY_HIGH;
	    timer_breakpara.outputautostate  = TIMER_OUTAUTO_DISABLE;
	    timer_breakpara.protectmode      = TIMER_CCHP_PROT_0;
	    timer_breakpara.breakstate       = TIMER_BREAK_DISABLE;
	    timer_break_config(TIMER0,&timer_breakpara);
		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,(_T>>1)-0);
		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,(_T>>1)+0);
		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,(_T>>1)+0);
	    timer_primary_output_config(TIMER0,DISABLE);

	    /* auto-reload preload disable */
	    timer_auto_reload_shadow_disable(TIMER0);
	    timer_enable(TIMER0);

}

void timer1_config(void) //running at 6kHz interrupt frequency
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    /* TIMER0 configuration */
    timer_initpara.prescaler         = 2;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 9999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1, &timer_initpara);

    /* CH0 configuration in PWM mode0 */
    timer_channel_output_struct_para_init(&timer_ocintpara);
    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocintpara);

    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 2000);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);



    /* TIMER0 primary output enable */
    timer_primary_output_config(TIMER1, ENABLE);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_UP);
        /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER1,TIMER_INT_FLAG_UP);

    /* enable TIMER0 */
    timer_enable(TIMER1);
}


void timer2_config(void) //for hall sensor processing.
{

    /* TIMER2 configuration: input capture mode */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER2);


    timer_deinit(TIMER2);
    /* hall mode config */
    timer_hall_mode_config(TIMER2, TIMER_HALLINTERFACE_ENABLE);



    /* TIMER2 configuration */
    timer_initpara.prescaler         = 239; //120MHz/(239+1)=500kHz
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 0xFFFF;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2,&timer_initpara);

    /* TIMER2  configuration */
    /* TIMER2 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_ITS;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0xFF;
    timer_input_capture_config(TIMER2,TIMER_CH_0,&timer_icinitpara);

    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_ITS;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0xFF;
    timer_input_capture_config(TIMER2,TIMER_CH_1,&timer_icinitpara);

    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_ITS;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0xFF;
    timer_input_capture_config(TIMER2,TIMER_CH_2,&timer_icinitpara);

    /* slave mode selection: TIMER2 */
    timer_input_trigger_source_select(TIMER2,TIMER_SMCFG_TRGSEL_CI0F_ED);
    timer_slave_mode_select(TIMER2,TIMER_SLAVE_MODE_RESTART);

    /* hall mode config */
    timer_hall_mode_config(TIMER2, TIMER_HALLINTERFACE_ENABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER2,TIMER_INT_FLAG_CH0);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER2,TIMER_INT_CH0);

    /* TIMER2 counter enable */
    timer_enable(TIMER2);

}



/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void nvic_config(void)
{
	/* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX1_IRQn,0,0);
    nvic_irq_enable(EXTI10_15_IRQn, 2U, 0U); //for PAS
    nvic_irq_enable(EXTI2_IRQn, 2U, 0U); //for external speed sensor


    //timer2 interrupt for Halls
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER1_IRQn, 0, 0);
    nvic_irq_enable(TIMER2_IRQn, 0, 0);
    nvic_irq_enable(ADC0_1_IRQn, 0, 0);
    nvic_irq_enable(TIMER4_IRQn, 0, 0);

}

void UART4_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOC); //for UART4 Tx on PC12
    //rcu_periph_clock_enable(RCU_GPIOD); //for UART4 Rx on PD2

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_UART4);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    /* connect port to USARTx_Rx */
   // gpio_init(GPIOD, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

    /* USART configure */
    usart_deinit(UART4);
    usart_baudrate_set(UART4, 115200U);
    //usart_receive_config(UART4, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART4, USART_TRANSMIT_ENABLE);
    usart_enable(UART4);

    printf("\n\ra Bafang Debug on UART4!\n\r");

}



void TIMER1_IRQHandler(void) // regular ADC processing and common slow timing tasks
{
	fwdgt_counter_reload();
	if(SET == timer_interrupt_flag_get(TIMER1,TIMER_INT_FLAG_UP)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_UP);

        reg_ADC_flag=1;
//        pulse_counter++;
//        if(pulse_counter>1000)gpio_bit_set(GPIOB,GPIO_PIN_8);
//        else gpio_bit_reset(GPIOB,GPIO_PIN_8);
//        if(pulse_counter>1020)pulse_counter=0;
    }
}


void TIMER2_IRQHandler(void)
{
	fwdgt_counter_reload();
    if(SET == timer_interrupt_flag_get(TIMER2,TIMER_INT_FLAG_CH0)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER2,TIMER_INT_FLAG_CH0);

       // if(TIM2->CCR1>20)ui16_timertics = TIM2->CCR1; //debounce hall signals
            /* read channel 0 capture value */

        	ui16_timertics=timer_channel_capture_value_register_read(TIMER2,TIMER_CH_0);
        	ui32_erps_cumulated-=ui32_erps_cumulated>>5;
        	ui32_erps_cumulated+=500000/(ui16_timertics*6);
        	ui16_erps=ui32_erps_cumulated>>5;
        	ui16_erps_counter=0; //FW-029: age of the last Hall event. Without this reset the
        	                     //counter only ever grew, so a "motor stopped" timeout was
        	                     //impossible and ui16_erps could report a stale speed forever.
                  	//Hall sensor event processing

            		ui8_hall_state = (GPIO_ISTAT(GPIOC)>>6)&0x07; //Mask input register with Hall 1 - 3 bits

            		ui8_hall_case=ui8_hall_state_old*10+ui8_hall_state;

            		if(MS.hall_angle_detect_flag){ //only process, if autodetect procedere is fininshed
            		ui8_hall_state_old=ui8_hall_state;
            		}

            			uint32_tics_filtered-=uint32_tics_filtered>>3;
            			uint32_tics_filtered+=ui16_timertics;

            		   ui8_overflow_flag=0;
            		   ui8_SPEED_control_flag=1;



            		switch (ui8_hall_case) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
            				{
            			//6 cases for forward direction
            		//6 cases for forward direction
            		case 64:
            			q31_rotorposition_hall = Hall_64;

            			i8_recent_rotor_direction = -i32_hall_order;
            			uint16_full_rotation_counter = 0;
            			break;
            		case 45:
            			q31_rotorposition_hall = Hall_45;

            			i8_recent_rotor_direction = -i32_hall_order;
            			break;
            		case 51:
            			q31_rotorposition_hall = Hall_51;

            			i8_recent_rotor_direction = -i32_hall_order;
            			break;
            		case 13:
            			q31_rotorposition_hall = Hall_13;

            			i8_recent_rotor_direction = -i32_hall_order;
            			uint16_half_rotation_counter = 0;
            			break;
            		case 32:
            			q31_rotorposition_hall = Hall_32;

            			i8_recent_rotor_direction = -i32_hall_order;
            			break;
            		case 26:
            			q31_rotorposition_hall = Hall_26;

            			i8_recent_rotor_direction = -i32_hall_order;
            			break;

            			//6 cases for reverse direction
            		case 46:
            			q31_rotorposition_hall = Hall_64;

            			i8_recent_rotor_direction = i32_hall_order;
            			break;
            		case 62:
            			q31_rotorposition_hall = Hall_26;

            			i8_recent_rotor_direction = i32_hall_order;
            			break;
            		case 23:
            			q31_rotorposition_hall = Hall_32;

            			i8_recent_rotor_direction = i32_hall_order;
            			uint16_half_rotation_counter = 0;
            			break;
            		case 31:
            			q31_rotorposition_hall = Hall_13;

            			i8_recent_rotor_direction = i32_hall_order;
            			break;
            		case 15:
            			q31_rotorposition_hall = Hall_51;

            			i8_recent_rotor_direction = i32_hall_order;
            			break;
            		case 54:
            			q31_rotorposition_hall = Hall_45;

            			i8_recent_rotor_direction = i32_hall_order;
            			uint16_full_rotation_counter = 0;
            			break;

            		} // end case

            		if(MS.angle_est){
            			q31_PLL_error=q31_rotorposition_PLL-q31_rotorposition_hall;
            			if(iabs(q31_PLL_error) < deg_30){
            				if(ui_8_PLL_counter<12)ui_8_PLL_counter++;
            			}
            			else ui_8_PLL_counter=0;
            			q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall,0);
            		}

            	#ifdef SPEED_PLL
            		if(ui16_erps>30){   //360 interpolation at higher erps
            			if(ui8_hall_case==32||ui8_hall_case==23){
            				q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall, SPDSHFT*tics_higher_limit/(uint32_tics_filtered>>3));

            			}
            		}
            		else{

            			q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall, SPDSHFT*tics_higher_limit/(uint32_tics_filtered>>3));
            		}

            	#endif


    }

}



void EXTI10_15_IRQHandler(void)
{
    if(RESET != exti_interrupt_flag_get(EXTI_12)) {
    	PAS_flag = 1;
        exti_interrupt_flag_clear(EXTI_12);
    }
}

void EXTI2_IRQHandler(void)
{
    if(RESET != exti_interrupt_flag_get(EXTI_2)) {
    	Speed_flag = 1;
        exti_interrupt_flag_clear(EXTI_2);
    }
}

void PAS_processing(void)
{
	if(PAS_counter>70){
		MS.cadence=10000/PAS_counter;//24 Pulses per crank revolution, 4000 Hz Timer interrupt frequency (for M560 about 48 pulses on speed/direction pin)(4000*60/24)=10000
		uint16_cadence_filtered-=uint16_cadence_filtered>>3;
		uint16_cadence_filtered+=MS.cadence;


		PAS_flag = 0;
		if(gpio_input_bit_get(GPIOD,GPIO_PIN_2)){
			if(Backwards_counter<10)Backwards_counter++;

		}
		else{
			if(Backwards_counter)Backwards_counter--;
			PAS_counter=0;
		}
		torque_cumulated-=torque_cumulated>>MS.TQfilter;
		if(MS.torque_on_crank>750){
			torque_cumulated+=(MS.torque_on_crank-750);
		}
		//Power=2*Pi*speed*torque, calibration factors: rpm to 1/s for cadence: /60, mV to Nm: 750 to 3200 --> 0 to 80 Nm. (from Bafang data sheet)
		MS.torque_filtered=(torque_cumulated>>MS.TQfilter);
		MS.p_human=(uint16_t)((float)(MS.cadence*MS.torque_filtered)*0.00342); //in Watt

		//PAS_counter=0;
	}
}

//FW-036: false-speed-pulse rejection. A hard current cut / backpedal can couple a spurious
//edge onto the speed line (PB2/EXTI2); with 1 pulse/rev one glitch sets the whole speed, the
//limiter then zeroes assist for ~2-3 s. Physics: the wheel can DECELERATE fast but cannot
//ACCELERATE by a big step in a fraction of a second. So validate every pulse against the max
//physical rise (relative to the last valid speed and the time since it), plus an absolute
//backstop. A rejected pulse updates nothing (speed / distance / counter), so the reading stays
//on the last good value and decays naturally, and a series of glitches can't hold false speed.
#define SPEED_TICKS_PER_S          4000U   // Speed_counter increments at the 4 kHz loop
#define SPEED_MAX_INSTANT_X100     7000U   // 70 km/h absolute backstop (no e-bike wheel does this)
#define SPEED_MAX_ACCEL_X100_PER_S 2500U   // 25 km/h/s max physical rise (generous; blocks glitches)

void Speed_processing(void)
{
		uint16_t ticks = Speed_counter;
		if(ticks==0){ Speed_flag=0; return; } //two edges in one tick -> glitch (also guards /0)
		uint32_t instant = MP.wheel_cirumference*4*360/(MP.pulses_per_revolution*ticks); //km/h x100
		uint32_t allowed = (uint32_t)last_valid_speed_x100 +
			((uint32_t)SPEED_MAX_ACCEL_X100_PER_S*ticks)/SPEED_TICKS_PER_S; //max physically-possible rise
		if(instant>SPEED_MAX_INSTANT_X100 || instant>allowed){ //impossible jump up -> reject, keep last good
			speed_glitch_count++;
			Speed_flag=0;                 //do NOT reset Speed_counter / cumulated / distance
			return;
		}
		Speedx100_cumulated-=Speedx100_cumulated/MP.pulses_per_revolution;
		Speedx100_cumulated+=instant;// 4000 Hz Timer interrupt frequency
		MS.Speedx100=Speedx100_cumulated/MP.pulses_per_revolution;
		last_valid_speed_x100=MS.Speedx100;
		Speed_counter=0;
		Speed_flag=0;
		MS.distance_since_startup+=MP.wheel_cirumference/(MP.pulses_per_revolution*1000); //in m
}

void reg_ADC_processing(void)
{
	battery_current_cumulated-=battery_current_cumulated>>6;
	battery_current_cumulated+= (adc_value[0]-bat_current_offset);
	MS.Battery_Current=(int32_t)((float)(battery_current_cumulated>>6)*CAL_BAT_I); //Battery current in mA
	voltage_raw_cumulated-=voltage_raw_cumulated>>6;
	voltage_raw_cumulated+=adc_value[3];
	voltage_raw_filtered=voltage_raw_cumulated>>6;

	temp1=MS.Battery_Current;
	temp2=MS.u_q;
	temp3=MS.i_q;
	temp4=MS.i_q_setpoint;

	MS.Voltage=voltage_raw_filtered*CAL_BAT_V;//Battery voltage in mV
	MS.calories=(uint16_t)(MS.int_Temperature); //temp sterownika na pole calories w HMI (offset +3 juz w int_Temperature)
	uint16_t torque_raw_mv=((adc_value[2])*3300)>>12; //map ADC value to mV
	MS.torque_on_crank=torque_input_correct(torque_raw_mv);
	if(MS.torque_on_crank>760&&PAS_counter<MP.PAS_timeout)torque_counter=0;//reset counter, if pressure on pedal and pedals rotating
	//--- Quadrature PAS decoder (PC12=A, PD2=B) @4kHz -> feeds cadence/Backwards/torque/p_human/PAS_counter ---
	{
		static const int8_t qd[16]={0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};
		uint8_t s = ((GPIO_ISTAT(GPIOC)&GPIO_PIN_12)?1:0) | ((GPIO_ISTAT(GPIOD)&GPIO_PIN_2)?2:0);
		if(pas_idle_ticks<64000) pas_idle_ticks++;
		if(pas_cycle_ticks<64000) pas_cycle_ticks++;
		if(pas_qstate==0xFF){ pas_qstate=s; }
		else if(s!=pas_qstate){
			int8_t st = qd[(pas_qstate<<2)|s]*PAS_DIR_SIGN; //+1 = forward
			uint8_t pas_qstate_prev=pas_qstate;   //FW-097 diag: which transition it was
			pas_qstate=s;
			if(st>0){            //forward step
				pas_last_period_ticks=pas_idle_ticks; //gap since the previous forward transition -> adaptive stop-timeout basis
				pas_idle_ticks=0;
				pas_fwd_accum++;            //FW-027 diag: free-running forward-step counter (EMI test)
				//FW-086: first forward step after a stop (or after a reverse step) begins a NEW
				//cadence interval. pas_cycle_ticks is reset ONLY when a cadence pulse fires, and
				//the stop branch below never touched it, so it kept counting through the whole
				//standstill and saturated at 64000. The next pulse then computed
				//10000/64000 = 0 rpm and cleared the start phase with that zero - which cut assist
				//(prepare_assist_input rejects cadence 0) AND prematurely enabled the power-derived
				//Iq ceiling that assist_modes.c:695 deliberately bypasses during launch. Assist only
				//recovered on the SECOND pulse, i.e. after 30 deg of crank instead of 15 deg.
				//THIS step must be the interval's ORIGIN, not its first count: if it were counted,
				//the pulse would fire one step early and span only PAS_STEPS_PER_PULSE-1 gaps,
				//reading 4/3 too high. So the step counter is held at 0 for this one step (see the
				//short-circuit at the pulse test below) and the pulse lands a full interval later.
				uint8_t cadence_interval_restart = (fwd_run==0);
				if(cadence_interval_restart){ pas_cycle_ticks=0; pas_fwd_steps=0; }
				if(fwd_run<250)fwd_run++;   //consecutive forward steps (jiggle-proof engage)
				pas_rev_run=0;              //FW-098: a forward step breaks the reverse run
				if(Backwards_counter)Backwards_counter--;
				// torque EMA @ 3.75deg (96 updates/rev) - full quadrature resolution so all algorithms see torque every step, not only every 15deg
				torque_cumulated-=torque_cumulated>>MS.TQfilter;
				if(MS.torque_on_crank>750) torque_cumulated+=(MS.torque_on_crank-750);
				MS.torque_filtered=(torque_cumulated>>MS.TQfilter);
				//FW-085: the RUN estimator advances HERE, on the crank step, not in the 4 kHz
				//control loop. That is what makes its window a fixed slice of the pedal stroke
				//at every cadence - the whole point of the card. Same place, same cadence-proof
				//reasoning as the torque EMA directly above.
				torque_input_run_filter_step();
				if(MS.torque_filtered>0) torque_counter=0; // cadence-gate: hold motor engaged while pedalling with any torque
#if START_PHASE_ENABLE
				//FW-087: declare the start phase, and ONLY that. This used to write a fake 1 rpm
				//into MS.cadence (plus the filtered cadence and p_human) so the control path would
				//not read "no cadence" as "not pedalling". No assist calculation ever read that 1 -
				//every consumer substitutes 0 while the flag is up - so it bought nothing except a
				//second meaning for MS.cadence, a fake 1 on the HMI, and a launch protection that
				//collapsed whenever something cleared the flag (the FW-086 defect).
				//FW-089: crank movement alone declares the start phase. It used to also demand
				//MS.torque_on_crank > 750+TQ_GATE_MIN - a raw-ADC threshold sitting 29 counts above
				//the 740 zero, i.e. ~1.19 kg on the default curve and ~1.53 kg after a user
				//calibration. The rider's OWN configured start load is 0.70 kg standing / 0.30 kg
				//rolling, so anything in between cleared the threshold the rider set and was still
				//refused by a constant they cannot see and that moves with sensor calibration.
				//Pressure is not lost as a condition - it moved to where it belongs: the kg
				//threshold in ride_control, which still has to be met before the latch arms and
				//before a single milliamp flows. Crank jiggle stays blocked by fwd_run, which any
				//reverse step resets.
				if(MS.cadence==0 && !start_phase && fwd_run>=START_PHASE_STEPS){
					start_phase=1;
				}
#endif
				//FW-086: the short-circuit keeps pas_fwd_steps at 0 on the restart step, so the
				//interval that follows spans a full PAS_STEPS_PER_PULSE and reads a true cadence.
				if(!cadence_interval_restart && ++pas_fwd_steps>=PAS_STEPS_PER_PULSE){ //one cadence pulse every PAS_STEPS_PER_PULSE forward transitions (see config.h)
					pas_fwd_steps=0;
					if(pas_cycle_ticks>70){
						MS.cadence=10000/pas_cycle_ticks;
						start_phase=0;                  //FW-087: a real measurement ends the start phase
						uint16_cadence_filtered-=uint16_cadence_filtered>>3;
						uint16_cadence_filtered+=MS.cadence;
						MS.p_human=(uint16_t)((float)(MS.cadence*MS.torque_filtered)*0.00342);
					}
					pas_cycle_ticks=0;
					PAS_counter=0;
				}
			}else if(st<0){      //backward step
				/*
				 * FW-097 MEASUREMENT ONLY — nothing here changes a decision.
				 *
				 * A reverse step hard-cuts assist (FW-024 latches on the FIRST one), and the
				 * bike log caught 43 of them while pedalling FORWARD at 16-52 rpm. The one
				 * number that tells bounce from real backpedalling is the GAP since the
				 * previous transition:
				 *   ~48 ticks at 52 rpm  = a real, full quadrature step -> genuine reverse
				 *   1-3 ticks (<1 ms)    = the same line toggling at its own edge -> bounce
				 * The decoder polls two GPIOs at 4 kHz with no debounce and no hysteresis, so
				 * one bounce yields 0->1->0: a forward step, then a REAL backward one.
				 */
				pas_rev_events++;
				pas_rev_last_trans=(uint8_t)((pas_qstate_prev<<4)|s);
				pas_rev_last_gap=pas_idle_ticks;
				pas_rev_last_period=pas_last_period_ticks;
				pas_rev_last_cadence=MS.cadence;
				pas_rev_last_fwdrun=fwd_run;
				if(pas_idle_ticks<pas_rev_min_gap) pas_rev_min_gap=pas_idle_ticks;
				pas_idle_ticks=0;
				pas_fwd_steps=0;
				/*
				 * UNCHANGED, and deliberately so: every reverse step clears the forward run.
				 * ride_core_pedaling needs fwd_run >= tuning_config_start_steps(), so this
				 * alone drops the ride latch and removes assist in the same tick. The motor
				 * cannot help while the crank is actually moving backwards — that guarantee
				 * does not depend on anything below.
				 */
				fwd_run=0;
				/*
				 * FW-098: the LONG penalty now needs confirmation.
				 *
				 * FW-024 latched on the FIRST reverse step, because with the old net +1/-1
				 * counting crank jitter during real backpedalling kept cancelling the count
				 * and it never reached the >=4 cut threshold (measured: 28 s of backpedalling,
				 * never hit 4). Latching fixed that, but it cannot tell deliberate
				 * backpedalling from the crank rocking back in the dead spot — and the bike
				 * log showed the latter is common at 16-52 rpm and left the counter pinned at
				 * 8 for a large part of the ride.
				 *
				 * A run of consecutive reverse steps separates them: real backpedalling is an
				 * unbroken run, dead-spot rocking is one or two steps with forward steps
				 * either side. Any forward step resets the run (see the forward branch), so
				 * dithering can never accumulate its way to the threshold.
				 */
				if(pas_rev_run<255) pas_rev_run++;
				if(pas_rev_run>pas_rev_run_max) pas_rev_run_max=pas_rev_run;
				if(pas_rev_run>=BACKWARD_CONFIRM_STEPS){
					Backwards_counter=BACKWARD_LATCH_COUNT;
					pas_rev_latches++;   //FW-098 diag: how often the long penalty really fired
				}
			}
		}
		//FW-0xx: adaptive stop timeout - 2x the last real forward-transition gap, clamped to
		//[PAS_STOP_TICKS..PAS_STOP_TICKS_MAX], so a momentary gap at low/uneven cadence doesn't
		//misread as a full stop the way the fixed PAS_STOP_TICKS alone did. Recomputed every
		//tick, before the stop check below; also reused later this tick (Backwards_counter cleanup).
		{
			uint32_t stop_timeout_calc = (uint32_t)pas_last_period_ticks*2U; //uint32_t: pas_last_period_ticks can be up to 64000, *2 overflows uint16_t
			if(stop_timeout_calc<PAS_STOP_TICKS) stop_timeout_calc=PAS_STOP_TICKS;
			else if(stop_timeout_calc>PAS_STOP_TICKS_MAX) stop_timeout_calc=PAS_STOP_TICKS_MAX;
			pas_stop_timeout = (uint16_t)stop_timeout_calc;
		}
		if(pas_idle_ticks>pas_stop_timeout){ MS.cadence=0; start_phase=0; uint16_cadence_filtered=0; pas_fwd_steps=0; fwd_run=0; } //stop
		//FW-087: the start phase counts as forward pedalling. The fake 1 rpm used to carry this
		//implicitly through MS.cadence>0; without saying so explicitly, dropping the fake would
		//close the assist gate a SECOND way, because forward_pedaling feeds pedaling_active
		//(ride_core_pedaling below) which assist_modes checks alongside the cadence itself.
		forward_pedaling = ((MS.cadence>0 || start_phase) && Backwards_counter<4 && pas_idle_ticks<=pas_stop_timeout);
	}
	//FW-061: latch "the wheel turned at some point since pedalling stopped". Sampling
	//the speed at the END of a coast misclassifies a coast that finishes at a
	//standstill as a standstill re-zero. Speedx100 alone is also not enough: below
	//~3 km/h it periodically falls to zero between pulses. So OR three sources over
	//the whole episode and reset only when pedalling resumes. Uncertain => moving.
	{
		static uint16_t prev_speed_counter=0;
		if(pas_idle_ticks==0) coast_wheel_moved=0;                 //pedalling -> new episode
		if(Speed_counter<prev_speed_counter ||                     //a wheel pulse reset the counter
		   MS.Speedx100>=TQ_RECAL_MOVING_X100 ||
		   Speed_counter<SPEED_STOP_TICKS) coast_wheel_moved=1;    //pulse within the stop window
		prev_speed_counter=Speed_counter;
	}
	//--- torque sensor fault detection (debounced) -> Error 25 ---
	if(MS.torque_on_crank<TQ_FAULT_LOW_MV || MS.torque_on_crank>TQ_FAULT_HIGH_MV){
		if(tq_fault_ticks<64000) tq_fault_ticks++;
	}else tq_fault_ticks=0;
	{
		uint8_t fault_now = (tq_fault_ticks>TQ_FAULT_TICKS) || torque_input_cal_fault();
		if(fault_now){ torque_fault=1; tq_fault_hold=TQ_FAULT_HOLD_TICKS; }
		else if(tq_fault_hold){ tq_fault_hold--; }
		else torque_fault=0;
	}
	//--- cyclic offset re-zero on coast (pedals idle >= TQ_RECAL_IDLE_TICKS): owned by torque_input ---
	torque_input_coast_update(MS.torque_on_crank, pas_idle_ticks>TQ_RECAL_IDLE_TICKS && tq_fault_ticks==0 && MS.i_q_setpoint==0,
		coast_wheel_moved!=0); //FW-058/FW-061: latched over the episode, not sampled at its end
	torque_input_set_run_window_deg(tuning_config_assist_torque_run_window_deg()); //FW-085: RUN estimator window in crank degrees (Canable)
	torque_input_update(torque_raw_mv, MS.torque_on_crank, torque_fault==0);
	//Publish one coherent, read-only rider snapshot: the single input the assist pipeline reads.
	{
		const torque_snapshot_t *torque_snapshot = torque_input_get_snapshot();
		//FW-068: the crank-movement half of the start condition is configurable from Canable
		//(Dynamics); TUNING_START_STEPS_DEFAULT is its default.
		bool ride_core_pedaling = forward_pedaling != 0 &&
			fwd_run >= tuning_config_start_steps();
		rider_input_t input = {
			.torque_raw_mv = torque_raw_mv,
			.torque_corrected_mv = MS.torque_on_crank,
			.torque_filtered = MS.torque_filtered,
			.torque_assist_filtered = torque_snapshot->assist_delta_filtered_native,
			.torque_run_filtered = torque_snapshot->assist_delta_run_native, //FW-033
			.torque_load_centikg = torque_input_load_centikg(),
			.cadence_rpm = MS.cadence,
			.wheel_speed_x100 = MS.Speedx100,
			.motor_erps = ui16_erps,
			.motor_voltage_utilization = (MS.u_abs > 2048) ? 2048U :
				(MS.u_abs > 0 ? (uint16_t)MS.u_abs : 0U),
			.pas_forward = ride_core_pedaling,
			.pas_backward = Backwards_counter >= 4,
			.pedaling_active = ride_core_pedaling,
			.crank_forward_steps = fwd_run, //FW-083: raw step count for ride_control's rolling-start reduction
			.crank_direction_ok = forward_pedaling != 0, //FW-083: direction half of ride_core_pedaling, without the step count
			.start_phase = start_phase != 0,
			.torque_sensor_valid = torque_fault == 0 &&
				!torque_input_calibration_active(),
			.pas_sensor_valid = pas_qstate != 0xFF
		};
		rider_input_update(&input);
	}
	//--- coulomb counting (signed: discharge>0 reduces charge, regen<0 adds back) ---
	soc_mAs_acc += (float)MS.Battery_Current / 4000.0f; //mA * (1/4000 s) per ~4kHz tick
	if(++soc_tick_counter >= 4000){                      //~1 second elapsed
		soc_tick_counter = 0;
		soc_one_second_flag = 1;
    }
    slow_loop_counter++;
#if CAN_DIAGNOSTICS_ENABLE
    t3100_counter++;
#endif
    if(torque_counter<64000)torque_counter++;
    if(PAS_counter<64000)PAS_counter++;
    if(Speed_counter<64000)Speed_counter++;
    if(uint16_half_rotation_counter<64000)uint16_half_rotation_counter++;
    if(pwm_cutoff_active && pwm_cutoff_tick<SOFT_CUTOFF_TICKS)pwm_cutoff_tick++; //taktowanie okna miekkiego zwolnienia @4kHz
    if(ui16_erps_counter<64000)ui16_erps_counter++;

    //--- Walk Assist physical button (PA4), debounce z histereza (press + release) ---
    uint8_t wa_btn_in_range=(adc_value[5]>=WA_BUTTON_THRESHOLD_LOW && adc_value[5]<=WA_BUTTON_THRESHOLD_HIGH);
    if(!ui8_walk_btn_state){
        if(wa_btn_in_range){ if(++ui8_walk_btn_counter>=WA_BUTTON_DEBOUNCE){ui8_walk_btn_state=1; ui8_walk_btn_counter=0;} }
        else ui8_walk_btn_counter=0;
    }else{
        if(!wa_btn_in_range){ if(++ui8_walk_btn_counter>=WA_BUTTON_RELEASE){ui8_walk_btn_state=0; ui8_walk_btn_counter=0;} }
        else ui8_walk_btn_counter=0;
    }

    //--- wheel-speed safety pause; resume automatically 0.5 km/h below the cut-off ---
    uint16_t wa_speed_limit=assist_modes_get_wa_max_wheel_x100();
    uint16_t wa_speed_resume=(wa_speed_limit>WA_SPEED_RESUME_HYST_X100) ?
        wa_speed_limit-WA_SPEED_RESUME_HYST_X100 : 0;
    if(ui8_wa_speed_paused){
        if(MS.Speedx100<wa_speed_resume)ui8_wa_speed_paused=0;
    }else if(MS.Speedx100>=wa_speed_limit){
        ui8_wa_speed_paused=1;
    }
    uint8_t walk_speed_ok=!ui8_wa_speed_paused;

    //--- FW-054: optional per-bank Walk Assist latch after button release ---
    uint8_t wa_latch_enabled=assist_modes_get_wa_latch_after_release();
    uint8_t wa_press_edge=ui8_walk_btn_state && !ui8_wa_btn_prev;
    uint8_t wa_up_edge=(MS.button_up_flag!=RESET) && !ui8_wa_up_prev;
    uint8_t wa_down_edge=(MS.button_down_flag!=RESET) && !ui8_wa_down_prev;
    uint8_t wa_light_changed=(uint8_t)(MS.light_flag!=RESET) != ui8_wa_light_prev;
    uint8_t wa_level_changed=MS.assist_level != ui8_wa_level_prev;
    uint8_t wa_power_pressed=adc_value[5]<2800U;
    uint8_t wa_cancel_event=wa_up_edge || wa_down_edge || wa_light_changed ||
                            wa_level_changed || wa_power_pressed ||
                            (wa_press_edge && ui8_wa_latch_active);
    uint8_t wa_hard_stop=MS.brake_active_flag || MS.error_state || !walk_speed_ok;

    if(!wa_latch_enabled){
        ui8_wa_latch_active=0;
        ui8_wa_latch_cancel_block=0;
        ui8_wa_hold_armed=0;
        ui32_wa_latch_ticks=0;
    }else{
        if(ui8_wa_latch_active){
            if(wa_cancel_event || wa_hard_stop || ui32_wa_latch_ticks==0U){
                ui8_wa_latch_active=0;
                ui32_wa_latch_ticks=0;
                ui8_wa_latch_cancel_block=wa_cancel_event ? 1U : 0U;
                walk_motor_release();
            }else{
                ui32_wa_latch_ticks--;
            }
        }
        if(ui8_wa_latch_cancel_block &&
           !ui8_walk_btn_state &&
           MS.button_up_flag==RESET &&
           MS.button_down_flag==RESET &&
           adc_value[5]>=2800U){
            ui8_wa_latch_cancel_block=0;
        }
        if(!ui8_wa_latch_active && !ui8_wa_latch_cancel_block &&
           MS.walk_can_request && ui8_walk_btn_state && !wa_hard_stop){
            ui8_wa_hold_armed=1;
        }
        if(ui8_wa_btn_prev && !ui8_walk_btn_state && ui8_wa_hold_armed){
            ui8_wa_hold_armed=0;
            if(MS.walk_can_request && !wa_hard_stop && !ui8_wa_latch_cancel_block){
                ui8_wa_latch_active=1;
                ui32_wa_latch_ticks=
                    (uint32_t)assist_modes_get_wa_latch_timeout_s()*SPEED_TICKS_PER_S;
            }
        }
        if(wa_hard_stop || ui8_wa_latch_cancel_block){
            ui8_wa_hold_armed=0;
        }
    }

    // Normal dead-man request or the optional timed latch, with the same safety gates.
    uint8_t walk_request=(MS.walk_can_request && ui8_walk_btn_state &&
                          !ui8_wa_latch_cancel_block) ||
                         ui8_wa_latch_active;
    uint8_t walk_active=walk_request
                     && walk_speed_ok
                     && !MS.brake_active_flag
                     && !MS.error_state;

    //--- pushassist_flag — tylko main.c ustawia ---
    MS.pushassist_flag=walk_active?SET:RESET;

    if(walk_request || walk_active)wa_bank_switch_locked=1;
    // Clear the motor controller whenever neither the held request nor timed latch is active.
    if(!walk_request){
        wa_engaged=0; //FW-060: a complete request release starts the next WA session
        walk_motor_release();
    }
    if(wa_bank_switch_locked && !walk_request && MS.i_q_setpoint==0){
        wa_bank_switch_locked=0;
        if(bank_toggle_pending){
            bank_toggle_pending=0;
            apply_bank_toggle();
        }
    }

    ui8_wa_btn_prev=ui8_walk_btn_state;
    ui8_wa_up_prev=MS.button_up_flag!=RESET;
    ui8_wa_down_prev=MS.button_down_flag!=RESET;
    ui8_wa_light_prev=MS.light_flag!=RESET;
    ui8_wa_level_prev=MS.assist_level;

    //FW-050: both level gestures (bank switch, offroad) now run through one shared detector.
    //Table and actions are defined at the top of this file; the engine keeps the confirmation
    //splash in its own state, so a gesture in progress can no longer falsify the speed reading.
    level_gesture_update(MS.assist_level);
    {
        //FW-013: torque load calibration state machine (stationary = no cadence, no motor current, not rolling)
        {
            uint8_t cal_stationary = (MS.cadence==0 && MS.i_q_setpoint==0 && MS.Speedx100==0);
            torque_input_cal_tick(MS.torque_on_crank, cal_stationary);
        }
        //FW-030: engine selection removed (ride core only). engine_persist stays 0 for the
        //shared standstill-persist condition below (no ride-engine switching anymore).
        uint8_t engine_persist = 0;
        //persist only at full standstill: flash write stalls the CPU, so never while driving
        uint8_t torque_cal_persist = torque_input_cal_take_persist_request();
        if((bank_save_pending || bank_save_request || torque_cal_persist || engine_persist || soc_full_persist) && MS.i_q_setpoint==0 && MS.cadence==0 && MS.Speedx100==0){
            MP.active_profile_bank = assist_modes_get_active_bank();
            if(bank_save_request){ //FW-006/FW-010: 0x6022 -> persist banks and ride-feel tuning together
                assist_modes_serialize_bank(0, &MP.bank_store[0][0]);
                assist_modes_serialize_bank(1, &MP.bank_store[1][0]);
                MP.bank_store_magic=0xB16B;
                tuning_config_serialize(&MP.tuning_store[0]);
                MP.tuning_store_magic=0x7501;
            }
            if(torque_cal_persist){ //FW-013: persist user torque span (or clear on restore-default)
                torque_input_build_persist(&MP.torque_cal_magic, &MP.torque_cal_version,
                    &MP.torque_cal_span_native, &MP.torque_cal_crc);
            }
            write_virtual_eeprom();
            bank_save_pending=0;
            bank_save_request=0;
            soc_full_persist=0; //FW-018: threshold now in flash
        }
    }
    {
        ride_control_input_t ride_input = {
            .speed_x100 = MS.Speedx100,
            .cadence_rpm = MS.cadence,
			.assist_level_index = level_to_array_element[MS.assist_level],
			.battery_voltage_mv = MS.Voltage,
            .iq_scale = phase_current_max_scaled,
			.ride_core_iq_limit = ride_core_iq_limit_scaled,
            .phase_current_max = MP.phase_current_max,
            .current_iq = MS.i_q_setpoint,
            .current_id = MS.i_d_setpoint,
			.voltage_raw = voltage_raw_filtered,
			.voltage_min_raw = MP.voltage_min,
			.controller_temperature_c = MS.int_Temperature,
			.cadence_filtered_x8 = uint16_cadence_filtered,
			.speed_limit_x100 = speedlimitx100_scaled,
			.legal_enabled = MP.legalflag != 0,
			.offroad = MS.offroadflag != RESET,
            .walk_active = MS.pushassist_flag != RESET,
			.position_calibration_active = MS.hall_angle_detect_flag > 1,
            .safety_cut = MS.brake_active_flag || Backwards_counter >= 4 ||
				overtemp_stage >= 2 || torque_fault ||
				torque_input_calibration_active(),
            //FW-030: throttle ported to the ride core. map() returns 0 while ADC < throttle_offset,
            //so a disconnected/unused throttle contributes nothing (offset is the natural gate).
            //Scaled to full phase_current_max (throttle is level-independent, like a real throttle).
            .throttle_iq = (int32_t)map(adc_value[1], MP.throttle_offset, MP.throttle_max, 0, MP.phase_current_max)
        };
        ride_control_update(&ride_input);
        /*
         * FW-098 success metric, measured AFTER the whole pipeline has run so it counts what
         * actually reached the motor, not what was requested.
         *
         * "Pedalling forward" is taken from the raw quadrature run, not from the ride latch or
         * forward_pedaling — both of those are downstream of the very cut being measured, so
         * using them would hide the defect inside its own metric. fwd_run > 0 with a recent
         * transition means the cranks are turning forward right now, whatever the control path
         * decided to do about it.
         */
        if(fwd_run>0 && pas_idle_ticks<=pas_stop_timeout){
            metric_pedal_ticks++;
            /*
             * MS.i_q_setpoint is the FINAL commanded current, written by
             * motor_core_set_command() after the ramps — not the measured MS.i_q. This asks
             * "did the control path ask for anything", which is the question this card is
             * about. Whether the power stage then delivered it is a different measurement.
             */
            if(MS.i_q_setpoint==0){
                metric_iq_zero_ticks++;
                /*
                 * Zero current while pedalling forward has several causes and the total on
                 * its own cannot tell them apart. fwd_run > 0 deliberately starts counting at
                 * the FIRST forward step, so the ordinary, intended start delay
                 * (tuning_config_start_steps, then the kg threshold) is inside this number
                 * too. That is the right definition from the saddle — "I am turning the
                 * cranks and getting nothing" — but it means the total is not a pure measure
                 * of the reverse latch. Split it here so one ride can answer which it was.
                 */
                uint8_t why = ride_control_get_debug_flags();
                if(Backwards_counter >= 4) metric_zero_backward++;
                else if(why & RIDE_DBG_NOT_LATCHED) metric_zero_notlatched++;
                /* everything else = total - these two, computed off the log */
            }
        }
        //FW-028: the ride core bypasses the legacy monolith's zero-target PI cleanup.
        //When the final command is zero, drop stale controller integral immediately so
        //the bridge cannot keep making torque after the assist target has disappeared.
        //FW-037: safety cuts are no longer reset here — they ramp down (integral clears when the
        //setpoint ramp reaches 0), so brake/backward/etc. fade smoothly. FW-028 zero-target reset stays.
        if(MS.i_q_setpoint==0){
			PI_iq.integral_part=0;
			PI_id.integral_part=0;
        }
        //FW-037: the old hard "safety_cut -> immediate neutral PWM + bridge DISABLE" path was
        //removed. Brake / backward / overtemp / torque-fault now fade via the Iq release ramp
        //(ride_control forces iq_target=0 + 200 ms release) and the normal soft cutoff after the
        //rotor stops. Only a real motor fault (overcurrent) hard-disables the bridge, in FOC.c.
        //FW-015b: peak-hold of ride-core diagnostics so a brief press on the bench is catchable
#if CAN_DIAGNOSTICS_ENABLE
        {
            const assist_mode_output_t* do_ = assist_modes_get_last_output();
            //FW-094: one engine, so one source. The old ternary picked the monolith's
            //MS.i_q_setpoint_temp for the removed Legacy engine and was already dead.
            int32_t current_iq_req = do_->iq_request;
            if(diag_peak_reset){ diag_peak_cadence=0; diag_peak_torque=0; diag_peak_human_w=0; diag_peak_support=0; diag_peak_motor_w=0; diag_peak_iq_req=0; diag_peak_iq_set=0;
                diag_peak_precomp_motor_w=0; diag_peak_cadence_comp=1000; diag_peak_u_abs=0; diag_peak_reset=0; } //FW-057
            if(do_->cadence_for_assist_rpm>diag_peak_cadence) diag_peak_cadence=do_->cadence_for_assist_rpm;
            if(do_->torque_for_assist_mv>diag_peak_torque) diag_peak_torque=do_->torque_for_assist_mv;
            if(do_->human_power_w>diag_peak_human_w) diag_peak_human_w=do_->human_power_w;
            if(do_->applied_support_ratio_pct>diag_peak_support) diag_peak_support=do_->applied_support_ratio_pct;
            if(do_->motor_power_w>diag_peak_motor_w) diag_peak_motor_w=do_->motor_power_w;
            if(current_iq_req>diag_peak_iq_req) diag_peak_iq_req=current_iq_req;
            if(MS.i_q_setpoint>diag_peak_iq_set) diag_peak_iq_set=MS.i_q_setpoint;
            //FW-057: pre-compensation power and the multiplier that was applied, so the
            //ride log can separate "the map asked for more" from "a limiter took it away".
            if(do_->precomp_motor_power_w>diag_peak_precomp_motor_w) diag_peak_precomp_motor_w=do_->precomp_motor_power_w;
            if(do_->cadence_comp_permille>diag_peak_cadence_comp) diag_peak_cadence_comp=do_->cadence_comp_permille;
            if(MS.u_abs>0 && (uint32_t)MS.u_abs>diag_peak_u_abs) diag_peak_u_abs=(MS.u_abs>65535)?65535:(uint16_t)MS.u_abs;
        }
#endif
    }
    if (torque_counter>4000){ //reset after one second without torque on the pedal
    	if (PAS_counter>MP.PAS_timeout){
			//FW-024b: clear the reverse flag ONLY once the crank is truly STOPPED. Backpedalling has no
			//forward torque and no forward cadence pulse, so this "1 s without torque" cleanup was firing
			//every tick during backpedalling and wiping Backwards_counter -> the FW-024 latch never held
			//>=4 (measured on 0.0193: WSTECZ flag never lit). Gating on pas_idle_ticks fixes that: while the
			//crank still moves (backward) the latch survives; once it stops the stale reverse flag clears.
			if(pas_idle_ticks>pas_stop_timeout) Backwards_counter=0;
			MS.cadence=0;
			MS.p_human=0;
			uint16_cadence_filtered=0;
    	}
		torque_cumulated=0;
		if (!MS.i_q_setpoint){//reset integral part, if no power from throttle signal is wanted
			PI_iq.integral_part=0;
			PI_id.integral_part=0;
		}
    }

	reg_ADC_flag=0;
}

int16_t internal_tics_to_speedx100 (uint32_t tics){
	return WHEEL_CIRCUMFERENCE*50*3600/(6*GEAR_RATIO*tics);
}

int16_t external_tics_to_speedx100 (uint32_t tics){
	return MP.wheel_cirumference*4*360/(MP.pulses_per_revolution*tics);
}

int32_t speed_PLL (int32_t ist, int32_t soll, uint8_t speedadapt)
  {
    int32_t q31_p;
    static int32_t q31_d_i = 0;
    static int32_t q31_d_dc = 0;
    //temp6 = soll-ist;
  //  temp5 = speedadapt;
    q31_p=(soll - ist)>>(P_FACTOR_PLL-speedadapt);   				//7 for Shengyi middrive, 10 for BionX IGH3
    q31_d_i+=(soll - ist)>>(I_FACTOR_PLL-speedadapt);				//11 for Shengyi middrive, 10 for BionX IGH3

    //clamp i part to twice the theoretical value from hall interrupts
    if (q31_d_i>((deg_30>>18)*500/ui16_timertics)<<16) q31_d_i = ((deg_30>>18)*500/ui16_timertics)<<16;
    if (q31_d_i<-((deg_30>>18)*500/ui16_timertics)<<16) q31_d_i =- ((deg_30>>18)*500/ui16_timertics)<<16;


    if (!ist&&!soll)q31_d_i=0;

    q31_d_dc=q31_p+q31_d_i;
    return (q31_d_dc);
  }

void runPIcontrol(void){

	//check, if Battery Current limit is exceeded
	if(MS.Battery_Current>MP.battery_current_max) BC_limit_flag=1;
	//check, if theoretical Battery current would be below limit with some hysteresis
	if((MS.i_q_setpoint*CAL_I*MS.u_abs)>>11<(MP.battery_current_max*0.9)) BC_limit_flag=0; //duty cycle is scaled to 2048 = 2^11

	if(!BC_limit_flag){
	//control iq
	  PI_iq.recent_value = MS.i_q;
	  PI_iq.setpoint = MP.reverse*i8_reverse_flag*MS.i_q_setpoint;

	}
	else{
	 //control Battery_Current
	  PI_iq.recent_value = MP.reverse*i8_reverse_flag*MS.Battery_Current>>6;
	  PI_iq.setpoint = MP.reverse*i8_reverse_flag*(MP.battery_current_max>>6);

	}
	q31_u_q_temp =  PI_control(&PI_iq);
	//control id
	  PI_id.recent_value = MS.i_d;
	  PI_id.setpoint = MS.i_d_setpoint;
	  q31_u_d_temp = -PI_control(&PI_id); //control direct current to zero

	  //circle limitation

	  MS.u_abs = (int32_t)sqrtf((float)(q31_u_d_temp*q31_u_d_temp+q31_u_q_temp*q31_u_q_temp));
//	  arm_sqrt_q31((q31_u_d_temp*q31_u_d_temp+q31_u_q_temp*q31_u_q_temp)<<1,&MS.u_abs);
//	  MS.u_abs = (MS.u_abs>>16)+1;

	  if (MS.u_abs > _U_MAX){
			MS.u_q = (q31_u_q_temp*_U_MAX)/MS.u_abs; //division!
			MS.u_d = (q31_u_d_temp*_U_MAX)/MS.u_abs; //division!
			MS.u_abs = _U_MAX;
		}
	  else{
			MS.u_q=q31_u_q_temp;
			MS.u_d=q31_u_d_temp;
		}
	  PI_flag=0;

}

void autodetect(void) {
	// Position calibration owns the bridge directly. Cancel any pending normal
	// soft cut-off before phase 1 so its delayed state cannot interfere here.
	pwm_cutoff_active=0;
	pwm_cutoff_tick=0;
	uint16_half_rotation_counter=0;
	timer_primary_output_config(TIMER0,ENABLE);
	ui_8_PWM_ON_Flag=1;
	MS.hall_angle_detect_flag = 0; //set uq to contstant value in FOC.c for open loop control
	q31_rotorposition_absolute = 1 << 31;
	i32_hall_order = 1;//reset hall order
	motor_command_t detect_command = {
		.iq_target = 0,
		.id_target = 200, //set MS.id to appr. 2000mA
		.enable = true,
		.emergency_stop = false
	};
	motor_core_set_command(&detect_command);

	for (int i = 0; i < 1080; i++) {
		q31_rotorposition_absolute += one_deg; //drive motor in open loop with steps of 1 deg
		delay_1ms(5);

		fwdgt_counter_reload(); //procedure blocks main loop >5 s: keep watchdog from resetting
		if((i%50)==0){ //~every 250 ms: keep HMI link alive (heartbeat+telemetry) -> no E30 (comm timeout)
			sendCAN_status_broadcast(&MS);
			sendCAN_Poll(&MP,&MS,0x3201);
			sendCAN_Poll(&MP,&MS,0x3200);
			sendCAN_Poll(&MP,&MS,0x3205);
		}

		if (ui8_hall_state_old != ui8_hall_state) {
//			printf_("angle: %d, hallstate:  %d, hallcase %d \n",
//					(int16_t) (((q31_rotorposition_absolute >> 23) * 180) >> 8),
//					ui8_hall_state, ui8_hall_case);

			switch (ui8_hall_case) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
			{
			//6 cases for forward direction
			case 64:
				Hall_64=q31_rotorposition_absolute;
				break;
			case 45:
				Hall_45=q31_rotorposition_absolute;
				break;
			case 51:
				Hall_51=q31_rotorposition_absolute;
				break;
			case 13:
				Hall_13=q31_rotorposition_absolute;
				break;
			case 32:
				Hall_32=q31_rotorposition_absolute;
				break;
			case 26:
				Hall_26=q31_rotorposition_absolute;
				break;

				//6 cases for reverse direction
			case 46:
				Hall_64=q31_rotorposition_absolute;
				break;
			case 62:
				Hall_26=q31_rotorposition_absolute;
				break;
			case 23:
				Hall_32=q31_rotorposition_absolute;
				break;
			case 31:
				Hall_13=q31_rotorposition_absolute;
				break;
			case 15:
				Hall_51=q31_rotorposition_absolute;
				break;
			case 54:
				Hall_45=q31_rotorposition_absolute;
				break;

			} // end case

            transmit_message.tx_data[0] = (int8_t) (((Hall_64 >> 23) * 180) >> 9);//scale q31 angle to -90 .. +90 for 1 Byte representation
            transmit_message.tx_data[1] = (int8_t) (((Hall_45 >> 23) * 180) >> 9);
            transmit_message.tx_data[2] = (int8_t) (((Hall_51 >> 23) * 180) >> 9);
            transmit_message.tx_data[3] = (int8_t) (((Hall_13 >> 23) * 180) >> 9);
            transmit_message.tx_data[4] = (int8_t) (((Hall_32 >> 23) * 180) >> 9);
            transmit_message.tx_data[5] = (int8_t) (((Hall_26 >> 23) * 180) >> 9);
            transmit_message.tx_data[6] = (adc_value[1]>>8)&0xFF;
            transmit_message.tx_data[7] = (adc_value[1])&0xFF;
            //self-contained hall-report frame (was relying on stale efid/dlen=0 from prior 0x6200 ACK)
            transmit_message.tx_sfid = 0x00;
            transmit_message.tx_efid = 0x6200 + (NORMAL_ACK<<16) + (0x05<<19) + (0x02<<24); //to BESST(5), source controller(2)
            transmit_message.tx_ft = CAN_FT_DATA;
            transmit_message.tx_ff = CAN_FF_EXTENDED;
            transmit_message.tx_dlen = 8;

            /* transmit message */
            transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
            /* waiting for transmit completed */
            timeout = 0xFFFF;
            while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
                timeout--;
            	}
			ui8_hall_state_old = ui8_hall_state;
		}
	}
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,0);
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,0);
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,0);
	delay_1ms(25);
	timer_primary_output_config(TIMER0,DISABLE); //Disable PWM if motor is not turning
	// Phase 1 disabled the hardware bridge, therefore the software state must
	// also say OFF. Phase 2 requests Iq=100 below; the normal bridge-on path will
	// then re-enable PWM immediately instead of waiting for the rotor-stop timer.
	ui_8_PWM_ON_Flag=0;
	pwm_cutoff_active=0;
	pwm_cutoff_tick=0;
	uint16_half_rotation_counter=0;

    MS.i_d = 0;
    MS.i_q = 0;
    MS.u_d=0;
    MS.u_q=0;
	detect_command.iq_target = MS.i_q_setpoint;
	detect_command.id_target = 0;
	motor_core_set_command(&detect_command);
    uint32_tics_filtered=1000000;


	if (i8_recent_rotor_direction == 1) {

		i32_hall_order = 1;
	} else {

		i32_hall_order = -1;
	}

	//write_virtual_eeprom();
	temp6=0; //phase-2 u_d accumulator must not inherit an earlier FOC/calibration value
	p=0;
	MS.hall_angle_detect_flag = 2;

	delay_1ms(20);
   // ui8_KV_detect_flag = 30;


}



void ADC0_1_IRQHandler(void)
{
    /* clear the ADC flag */
	fwdgt_counter_reload();
    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOIC);
    /* read ADC inserted group data register */

    __disable_irq();
    i16_ph1_current = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_0);
    i16_ph2_current = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0);
    i16_ph3_current = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0);


	switch (MS.char_dyn_adc_state) //read in according to state
		{
		case 1: //Phase C at high dutycycles, read from A+B directly
			{
				//first reading is correct, do nothing
			}
			break;
		case 2: //Phase A at high dutycycles, read from B+C (A = -B -C)
			{

				//overwrite A with -B-C
				i16_ph1_current = -i16_ph2_current-i16_ph3_current;

			}
			break;
		case 3: //Phase B at high dutycycles, read from A+C (B=-A-C)
			{

				//overwrite B with -A-C
				i16_ph2_current = -i16_ph1_current-i16_ph3_current;
			}
			break;

		case 0: //timeslot too small for ADC
			{
				//do nothing
			}
			break;




		} // end case

    //get the recent timer value from the Hall timer
    ui16_tim2_recent = timer_counter_read(TIMER2);
    if (ui16_tim2_recent>SIXSTEPTHRESHOLD<<1){
    	ui16_timertics=SIXSTEPTHRESHOLD<<1;
    	uint32_tics_filtered=ui16_timertics<<3;
    }
    //check the speed for sixstep threshold
	if (ui16_timertics < SIXSTEPTHRESHOLD && ui16_tim2_recent < 200)
		ui8_6step_flag = 0;
	if (ui16_timertics > (SIXSTEPTHRESHOLD * 6) >> 2)
		ui8_6step_flag = 1;

    // extrapolate rotorposition from filtered speed reading
    if(MS.hall_angle_detect_flag){//q31_rotorposition_absolute = q31_rotorposition_hall + (q31_t) ((float)(i8_recent_rotor_direction * (deg_30<<1) * ui16_tim2_recent)/(float)(uint32_tics_filtered>>3));//
//Speed PLL not implemented yet.
    	if(!ui8_6step_flag){
    	q31_rotorposition_absolute = q31_rotorposition_hall + MP.angle_correction +
    									+ (q31_t) (i8_recent_rotor_direction
    											* ((10923 * ui16_tim2_recent)
    													/ (uint32_tics_filtered>>3)) << 16);//interpolate angle between two hallevents by scaling timer2 tics, 10923<<16 is 715827883 = 60deg
    	}
    	else q31_rotorposition_absolute = q31_rotorposition_hall - MP.reverse * deg_30; //offset of 30 degree to get the middle of the sector

    }

	//get the Phase with highest duty cycle for dynamic phase current reading
	dyn_adc_state(q31_rotorposition_absolute);

    //q31_rotorposition_absolute=(int16_t)((180.0/75.0)*(float)(1<<31));
    if(ui_8_PWM_ON_Flag){
		FOC_calculation(i16_ph1_current, i16_ph2_current,
					q31_rotorposition_absolute,
					(((int16_t) MP.reverse * i8_reverse_flag)
							* MS.i_q_setpoint), &MS, &MP);

		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,switchtime[0]);
		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,switchtime[1]);
		timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,switchtime[2]);

    }
    __enable_irq();

}

int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  // if input is smaller/bigger than expected return the min/max out ranges value
  if (x < in_min)
    return out_min;
  else if (x > in_max)
    return out_max;

  // map the input to the output range.
  // round up if mapping bigger ranges to smaller ranges
  else  if ((in_max - in_min) > (out_max - out_min))
    return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
  // round down if mapping smaller ranges to bigger ranges
  else
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//assuming, a proper AD conversion takes 350 timer tics, to be confirmed. DT+TR+TS deadtime + noise subsiding + sample time
void dyn_adc_state(q31_t angle){
	if (switchtime[2]>switchtime[0] && switchtime[2]>switchtime[1]){
		MS.char_dyn_adc_state = 1; // -90Ã‚Â° .. +30Ã‚Â°: Phase C at high dutycycles
		if(switchtime[2]>DYNAMIC_ADC_THRESHOLD)timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,(switchtime[2]-TRIGGER_OFFSET_ADC));
		else timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,TRIGGER_DEFAULT);
	}

	if (switchtime[0]>switchtime[1] && switchtime[0]>switchtime[2]) {
		MS.char_dyn_adc_state = 2; // +30Ã‚Â° .. 150Ã‚Â° Phase A at high dutycycles
		if(switchtime[0]>DYNAMIC_ADC_THRESHOLD)timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,(switchtime[0]-TRIGGER_OFFSET_ADC));
		else timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,TRIGGER_DEFAULT);
	}

	if (switchtime[1]>switchtime[0] && switchtime[1]>switchtime[2]){
		MS.char_dyn_adc_state = 3; // +150 .. -90Ã‚Â° Phase B at high dutycycles
		if(switchtime[1]>DYNAMIC_ADC_THRESHOLD)timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,(switchtime[1]-TRIGGER_OFFSET_ADC));
		else timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,TRIGGER_DEFAULT);
	}
}

#if CAN_DIAGNOSTICS_ENABLE
void print_debug_on_CAN(void){


	//FW-027 diag: ride-core runaway telemetry. Logged standalone (ID 0x00010203 -> logger Data1-4, big-endian).
	//Data1 iq_setpoint | Data2 hi=cadence lo=flags | Data3 pas_fwd_accum (free-run) | Data4 torque delta.
	int32_t dbg_iq = MS.i_q_setpoint; if(dbg_iq<0)dbg_iq=0; if(dbg_iq>65535)dbg_iq=65535;
	int32_t dbg_tq = (int32_t)MS.torque_on_crank - 750; if(dbg_tq<0)dbg_tq=0; if(dbg_tq>65535)dbg_tq=65535;
	int32_t dbg_iq_actual = MS.i_q; if(dbg_iq_actual<-32768)dbg_iq_actual=-32768; if(dbg_iq_actual>32767)dbg_iq_actual=32767;
	int32_t dbg_u_abs = MS.u_abs; if(dbg_u_abs<0)dbg_u_abs=0; if(dbg_u_abs>65535)dbg_u_abs=65535;
	int32_t dbg_u_q = MS.u_q; if(dbg_u_q<-32768)dbg_u_q=-32768; if(dbg_u_q>32767)dbg_u_q=32767;
	uint8_t dbg_safety_cut = (MS.brake_active_flag || Backwards_counter >= 4 ||
		overtemp_stage >= 2 || torque_fault || torque_input_calibration_active()) ? 1 : 0;
	const assist_mode_output_t* dbg_mo = assist_modes_get_last_output();
	uint8_t dbg_flags = (forward_pedaling?0x01:0)
	                  | ((Backwards_counter>=4)?0x02:0)
	                  | (ui_8_PWM_ON_Flag?0x04:0)
	                  | (start_phase?0x08:0)
	                  | ((dbg_mo && dbg_mo->assist_without_rotation_active)?0x10:0);
	transmit_message.tx_sfid = 0x00;
	transmit_message.tx_efid = 0x00010203; //ID for debug message
	transmit_message.tx_ft = CAN_FT_DATA;
	transmit_message.tx_ff = CAN_FF_EXTENDED;
	transmit_message.tx_dlen = 8;
	transmit_message.tx_data[0] = (dbg_iq>>8)&0xFF;   //Data1: iq_setpoint (motor current) - is motor driven?
	transmit_message.tx_data[1] = (dbg_iq)&0xFF;
	transmit_message.tx_data[2] = MS.cadence&0xFF;    //Data2 hi: cadence rpm - stuck >0 while crank stopped?
	transmit_message.tx_data[3] = dbg_flags;          //Data2 lo: gate flags (pedaling/backward/pwm/seed/no-rot)
	transmit_message.tx_data[4] = (pas_fwd_accum>>8)&0xFF; //Data3: free-running fwd-step counter (EMI test via diff)
	transmit_message.tx_data[5] = (pas_fwd_accum)&0xFF;
	transmit_message.tx_data[6] = (dbg_tq>>8)&0xFF;   //Data4: torque delta - is torque sustaining assist?
	transmit_message.tx_data[7] = (dbg_tq)&0xFF;

	/* transmit message */
	transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
	/* waiting for transmit completed */
	timeout = 0xFFFF;
	while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
		timeout--;
		}

	//FW-028 diag extension (ID 0x00010204): actual FOC/PWM state after the command path.
	//Data1 signed i_q | Data2 u_abs | Data3 signed u_q | Data4 hi=flags2 lo=halfrot_4ms.
	uint8_t dbg_flags2 = (dbg_safety_cut?0x01:0)
	                   | (ui_8_PWM_ON_Flag?0x02:0)
	                   | (pwm_cutoff_active?0x04:0)
	                   | (MS.brake_active_flag?0x08:0)
	                   | ((Backwards_counter>=4)?0x10:0)
	                   | (torque_fault?0x20:0)
	                   | (BC_limit_flag?0x40:0);
	                   //FW-094: bit 0x80 was the Legacy overrun flag. That mechanism is gone and
	                   //the flag could only ever read 0, so the wire value is unchanged; the bit
	                   //is now free for a future diagnostic.
	uint8_t dbg_halfrot_4ms = (uint16_half_rotation_counter >= 4080) ?
		255 : (uint8_t)(uint16_half_rotation_counter >> 4);
	transmit_message.tx_efid = 0x00010204;
	transmit_message.tx_data[0] = ((uint16_t)dbg_iq_actual>>8)&0xFF;
	transmit_message.tx_data[1] = ((uint16_t)dbg_iq_actual)&0xFF;
	transmit_message.tx_data[2] = ((uint16_t)dbg_u_abs>>8)&0xFF;
	transmit_message.tx_data[3] = ((uint16_t)dbg_u_abs)&0xFF;
	transmit_message.tx_data[4] = ((uint16_t)dbg_u_q>>8)&0xFF;
	transmit_message.tx_data[5] = ((uint16_t)dbg_u_q)&0xFF;
	transmit_message.tx_data[6] = dbg_flags2;
	transmit_message.tx_data[7] = dbg_halfrot_4ms;

	transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
	timeout = 0xFFFF;
	while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
		timeout--;
		}

	//FW-060 diag (ID 0x00010205): Walk Assist motor-speed controller.
	//Data1 hi=state lo=flags | Data2 target_erps | Data3 measured_erps | Data4 iq_cmd.
	int32_t dbg_wa_iq = wa_diag.iq_target; if(dbg_wa_iq<0)dbg_wa_iq=0; if(dbg_wa_iq>65535)dbg_wa_iq=65535;
	transmit_message.tx_efid = 0x00010205;
	transmit_message.tx_data[0] = wa_diag.state;
	transmit_message.tx_data[1] = wa_diag.flags;
	transmit_message.tx_data[2] = (wa_diag.target_erps>>8)&0xFF;
	transmit_message.tx_data[3] = (wa_diag.target_erps)&0xFF;
	transmit_message.tx_data[4] = (wa_diag.measured_erps>>8)&0xFF;
	transmit_message.tx_data[5] = (wa_diag.measured_erps)&0xFF;
	transmit_message.tx_data[6] = ((uint16_t)dbg_wa_iq>>8)&0xFF;
	transmit_message.tx_data[7] = ((uint16_t)dbg_wa_iq)&0xFF;

	transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
	timeout = 0xFFFF;
	while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
		timeout--;
		}

	//FW-060 diag (ID 0x00010206): PI/start internals; the first frame stays wire-compatible.
	//Data1 error_erps (signed) | Data2 integral_iq | Data3 startup_iq | Data4 hall_age_ms.
	uint32_t dbg_hall_ms = ui16_erps_counter/4U; if(dbg_hall_ms>65535)dbg_hall_ms=65535; //~4 ticks/ms @4kHz
	transmit_message.tx_efid = 0x00010206;
	transmit_message.tx_data[0] = ((uint16_t)wa_diag.error_erps>>8)&0xFF;
	transmit_message.tx_data[1] = ((uint16_t)wa_diag.error_erps)&0xFF;
	transmit_message.tx_data[2] = ((uint16_t)wa_diag.integral_iq>>8)&0xFF;
	transmit_message.tx_data[3] = ((uint16_t)wa_diag.integral_iq)&0xFF;
	transmit_message.tx_data[4] = ((uint16_t)wa_diag.startup_iq>>8)&0xFF;
	transmit_message.tx_data[5] = ((uint16_t)wa_diag.startup_iq)&0xFF;
	transmit_message.tx_data[6] = (dbg_hall_ms>>8)&0xFF;
	transmit_message.tx_data[7] = (dbg_hall_ms)&0xFF;

	transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
	timeout = 0xFFFF;
	while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
		timeout--;
		}

	/*
	 * FW-096 diag (ID 0x00010208): WHO ZEROED THE CURRENT.
	 *
	 * "MS.i_q_setpoint is 0" says nothing about why, which is what turned a regression hunt
	 * into guesswork. This frame names the gate. Diagnostics-only; it changes no decision.
	 *
	 *   Data1 hi = safety causes, each one an INDEPENDENT reading (not the aggregate):
	 *              0x01 brake      0x02 backward pedalling   0x04 critical overtemp
	 *              0x08 torque fault (Error 25)              0x10 torque load calibration
	 *              0x20 walk active                          0x40 PWM/bridge on
	 *              0x80 assist level 0
	 *   Data1 lo = ride_control stage flags, see RIDE_DBG_* in ride_control.h
	 *   Data2    = assist level index | cadence rpm
	 *   Data3    = mode iq_request  (what the assist pipeline asked for, before limits)
	 *   Data4    = MS.i_q_setpoint  (what actually reached the motor)
	 */
	uint8_t why_safety = (MS.brake_active_flag?0x01:0)
	                   | ((Backwards_counter>=4)?0x02:0)
	                   | ((overtemp_stage>=2)?0x04:0)
	                   | (torque_fault?0x08:0)
	                   | (torque_input_calibration_active()?0x10:0)
	                   | ((MS.pushassist_flag!=RESET)?0x20:0)
	                   | (ui_8_PWM_ON_Flag?0x40:0)
	                   | ((level_to_array_element[MS.assist_level]==0)?0x80:0);
	const assist_mode_output_t* why_mo = assist_modes_get_last_output();
	int32_t why_req = why_mo ? why_mo->iq_request : 0;
	if(why_req<0) why_req=0;
	if(why_req>65535) why_req=65535;
	int32_t why_set = MS.i_q_setpoint;
	if(why_set<0) why_set=0;
	if(why_set>65535) why_set=65535;
	transmit_message.tx_efid = 0x00010208;
	transmit_message.tx_data[0] = why_safety;
	transmit_message.tx_data[1] = ride_control_get_debug_flags();
	transmit_message.tx_data[2] = level_to_array_element[MS.assist_level];
	transmit_message.tx_data[3] = MS.cadence;
	transmit_message.tx_data[4] = (why_req>>8)&0xFF;
	transmit_message.tx_data[5] = (why_req)&0xFF;
	transmit_message.tx_data[6] = (why_set>>8)&0xFF;
	transmit_message.tx_data[7] = (why_set)&0xFF;
	transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
	timeout = 0xFFFF;
	while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
		timeout--;
		}

	/*
	 * FW-096 diag (ID 0x00010209): the two things that silently zero BOTH assist and Walk
	 * without raising any HMI error.
	 *
	 *   Data1 = voltage_raw_filtered   the live pack reading the limiter sees
	 *   Data2 = MP.voltage_min         its cut-off. If Data1 <= Data2 the shared undervoltage
	 *                                  limiter maps every request to 0, on both paths.
	 *   Data3 = ride_core_iq_limit_scaled   assist ceiling after limp mode
	 *   Data4 = phase_current_max_scaled    per-level ceiling after limp mode
	 *                                  Either at 0 means no current can be asked for at all.
	 */
	int32_t why_vraw = voltage_raw_filtered; if(why_vraw<0)why_vraw=0; if(why_vraw>65535)why_vraw=65535;
	int32_t why_vmin = MP.voltage_min; if(why_vmin<0)why_vmin=0; if(why_vmin>65535)why_vmin=65535;
	int32_t why_rcl = ride_core_iq_limit_scaled; if(why_rcl<0)why_rcl=0; if(why_rcl>65535)why_rcl=65535;
	int32_t why_pcm = phase_current_max_scaled; if(why_pcm<0)why_pcm=0; if(why_pcm>65535)why_pcm=65535;
	transmit_message.tx_efid = 0x00010209;
	transmit_message.tx_data[0] = (why_vraw>>8)&0xFF;
	transmit_message.tx_data[1] = (why_vraw)&0xFF;
	transmit_message.tx_data[2] = (why_vmin>>8)&0xFF;
	transmit_message.tx_data[3] = (why_vmin)&0xFF;
	transmit_message.tx_data[4] = (why_rcl>>8)&0xFF;
	transmit_message.tx_data[5] = (why_rcl)&0xFF;
	transmit_message.tx_data[6] = (why_pcm>>8)&0xFF;
	transmit_message.tx_data[7] = (why_pcm)&0xFF;
	transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
	timeout = 0xFFFF;
	while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
		timeout--;
		}

	/*
	 * FW-097 diag (ID 0x0001020A): WHY was a reverse step counted.
	 *
	 * A single reverse quadrature step hard-cuts assist, and the bike log caught 43 of them
	 * while pedalling forward. This frame answers the only question that matters: was it a
	 * real direction change, or the PAS line bouncing at its own edge?
	 *
	 *   Data1 hi = transition, (previous qstate << 4) | new qstate. One changed bit is a legal
	 *              step; the decoder already ignores two-bit jumps, so a missed step is NOT
	 *              what produces these.
	 *   Data1 lo = forward-step run destroyed by it (fwd_run before the reset)
	 *   Data2    = GAP: control ticks since the previous transition. THE decisive number.
	 *              1-3 (<1 ms) = bounce on one line. ~48 @52 rpm = a genuine quadrature step.
	 *   Data3    = the last genuine forward-step gap, so Data2 has a scale to be read against
	 *   Data4 hi = cadence at the event, lo = smallest gap ever seen (clamped)
	 *   Data5    = running total of reverse steps (diff between frames = new events)
	 */
	uint16_t rev_gap = pas_rev_last_gap;
	uint16_t rev_per = pas_rev_last_period;
	uint16_t rev_min = (pas_rev_min_gap>255) ? 255 : pas_rev_min_gap;
	transmit_message.tx_efid = 0x0001020A;
	transmit_message.tx_data[0] = pas_rev_last_trans;
	transmit_message.tx_data[1] = pas_rev_last_fwdrun;
	transmit_message.tx_data[2] = (rev_gap>>8)&0xFF;
	transmit_message.tx_data[3] = (rev_gap)&0xFF;
	transmit_message.tx_data[4] = (rev_per>>8)&0xFF;
	transmit_message.tx_data[5] = (rev_per)&0xFF;
	transmit_message.tx_data[6] = pas_rev_last_cadence;
	transmit_message.tx_data[7] = (uint8_t)rev_min;
	transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
	timeout = 0xFFFF;
	while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
		timeout--;
		}

	/*
	 * Counters in their own frame so a lost 0x20A cannot hide that events happened.
	 * FW-098 split the two numbers that used to be one: how many reverse STEPS were seen,
	 * and how many of them actually fired the long penalty.
	 */
	transmit_message.tx_efid = 0x0001020B;
	transmit_message.tx_data[0] = (pas_rev_events>>8)&0xFF;
	transmit_message.tx_data[1] = (pas_rev_events)&0xFF;
	transmit_message.tx_data[2] = (pas_rev_latches>>8)&0xFF;
	transmit_message.tx_data[3] = (pas_rev_latches)&0xFF;
	transmit_message.tx_data[4] = (uint8_t)(Backwards_counter);
	transmit_message.tx_data[5] = fwd_run;
	transmit_message.tx_data[6] = pas_rev_run;
	transmit_message.tx_data[7] = pas_rev_run_max;
	transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
	timeout = 0xFFFF;
	while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
		timeout--;
		}

	/*
	 * FW-098 diag (ID 0x0001020C): THE metric this card is judged by.
	 *
	 *   Data1..2 = control ticks with the cranks turning forward   (u32, big-endian)
	 *   Data3..4 = ...of which the motor was given no current      (u32, big-endian)
	 *
	 * The ratio is the fraction of pedalling time the rider gets nothing. Backwards_counter
	 * falling is not success on its own — a shorter penalty that still lands on every stroke
	 * would look good on the counter and feel identical on the bike.
	 */
	transmit_message.tx_efid = 0x0001020C;
	transmit_message.tx_data[0] = (metric_pedal_ticks>>24)&0xFF;
	transmit_message.tx_data[1] = (metric_pedal_ticks>>16)&0xFF;
	transmit_message.tx_data[2] = (metric_pedal_ticks>>8)&0xFF;
	transmit_message.tx_data[3] = (metric_pedal_ticks)&0xFF;
	transmit_message.tx_data[4] = (metric_iq_zero_ticks>>24)&0xFF;
	transmit_message.tx_data[5] = (metric_iq_zero_ticks>>16)&0xFF;
	transmit_message.tx_data[6] = (metric_iq_zero_ticks>>8)&0xFF;
	transmit_message.tx_data[7] = (metric_iq_zero_ticks)&0xFF;
	transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
	timeout = 0xFFFF;
	while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
		timeout--;
		}

	/*
	 * FW-098 diag (ID 0x0001020D): WHY the current was zero, so the total in 0x20C can be
	 * read for what it is.
	 *
	 *   Data1..2 = ticks the backward latch was up      (u32)
	 *   Data3..4 = ticks the ride latch was not armed    (u32) — this is where the ordinary,
	 *              intended start delay lands, and it is NOT a defect on its own
	 *   everything else = 0x20C zero-ticks minus these two
	 *
	 * Without this split a result like "zero-Iq fell from 17 % to 15 %" cannot say whether
	 * the reverse latch stopped mattering or never was the main cause.
	 */
	transmit_message.tx_efid = 0x0001020D;
	transmit_message.tx_data[0] = (metric_zero_backward>>24)&0xFF;
	transmit_message.tx_data[1] = (metric_zero_backward>>16)&0xFF;
	transmit_message.tx_data[2] = (metric_zero_backward>>8)&0xFF;
	transmit_message.tx_data[3] = (metric_zero_backward)&0xFF;
	transmit_message.tx_data[4] = (metric_zero_notlatched>>24)&0xFF;
	transmit_message.tx_data[5] = (metric_zero_notlatched>>16)&0xFF;
	transmit_message.tx_data[6] = (metric_zero_notlatched>>8)&0xFF;
	transmit_message.tx_data[7] = (metric_zero_notlatched)&0xFF;
	transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
	timeout = 0xFFFF;
	while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
		timeout--;
		}
}
#endif

int16_t T_NTC(uint16_t ADC) // ADC 12 Bit, 10k NTC, RÃ¼ckgabewert in Â°C

{
	int R = R_TEMP_PULLUP;                                    // Spannungsteiler, fester Widerstand
	float Rn = 5000;                                         // gemessen (Ohm)
	float Tn = 23;                                            // gemessen (°C)
	float B = 3398;
    float U_ntc = (3.3 * ADC) / 4095;             // Spannung
    float R_ntc = (U_ntc * R) / (3.3 - U_ntc);           // Widerstand
                                                          // Temperatur-Berechnung
        // Rt = Rn * e hoch B*(1/T - 1/Tn)                // Ausgangsformel
        // T = 1 / [(log(Rt/Rn)/B + 1/Tn] - 273,15        // umgestellt in °K
        // Tnk = 26,2 + 273,15                            // °K
    float A1 = log(R_ntc / Rn) / B;
    float A2 = A1 + 1 / (Tn + 273.15);
    float T = (1 / A2) - 273.15;
	return (int)T; // Rundung

}

void get_standstill_position(){

	  delay_1ms(25);
	  ui8_hall_state = (GPIO_ISTAT(GPIOC)>>6)&0x07;
		switch (ui8_hall_state) {
			//6 cases for forward direction
			case 2:
				q31_rotorposition_hall = Hall_32;
				break;
			case 6:
				q31_rotorposition_hall = Hall_26;
				break;
			case 4:
				q31_rotorposition_hall = Hall_64;
				break;
			case 5:
				q31_rotorposition_hall = Hall_45;
				break;
			case 1:
				q31_rotorposition_hall = Hall_51;

				break;
			case 3:
				q31_rotorposition_hall = Hall_13;
				break;

			}

			q31_rotorposition_absolute = q31_rotorposition_hall;
}

int8_t calculate_SOC(uint16_t voltage, uint8_t cells_in_series){ //interpolate from lookup table
    //measured LG M58T discharge curve @3A (home measurements, "Srednia LG" per-cell average), ascending
    float voltages[]   = {2.799, 2.968, 3.086, 3.247, 3.450, 3.569, 3.681, 3.774, 3.853, 3.946, 3.989, 4.070};
    float soc_values[] = {0,     5,     10,    20,    30,    40,    50,    60,    70,    80,    90,    100};
    int length = sizeof(voltages) / sizeof(voltages[0]);
    float cell_voltage = (float)voltage/((float)cells_in_series*1000);
    if (cell_voltage <= voltages[0]) {
        return (int8_t)soc_values[0];
    }
    if (cell_voltage >= voltages[length - 1]) {
        return (int8_t)soc_values[length - 1];
    }

    for (int i = 0; i < length - 1; i++) {
        if (cell_voltage < voltages[i+1]) {
            float slope = (soc_values[i+1] - soc_values[i]) / (voltages[i+1] - voltages[i]);
            float soc = soc_values[i] + slope * (cell_voltage - voltages[i]);
            return (int8_t)soc;
        }
    }
    return (int8_t)soc_values[length - 1];
}

//=====================  SOC / Range implementation  =====================

uint32_t soc_crc32(const uint8_t* data, uint32_t len){
	uint32_t crc=0xFFFFFFFFU;
	for(uint32_t i=0;i<len;i++){
		crc^=data[i];
		for(int b=0;b<8;b++){
			crc=(crc>>1)^(0xEDB88320U & (uint32_t)(-(int32_t)(crc&1U)));
		}
	}
	return ~crc;
}

float compute_limp_factor(float soc){
	uint8_t lim=MP.limp_soc_limit;
	if(lim==LIMP_DISABLED || lim==0) return 1.0f;          //disabled
	float fl=(float)LIMP_FLOOR_PCT/100.0f;
	if(soc<0) soc=0;
	if(soc>=lim) return 1.0f;
	uint8_t s2=MP.limp_soc_limit_stage2;
	float f;
	if(s2!=LIMP_DISABLED && s2>0 && s2<lim){
		float p2=(float)LIMP_STAGE2_PCT/100.0f;
		if(soc>s2) f=p2+(1.0f-p2)*(soc-(float)s2)/(float)(lim-s2);  //s2..lim : p2 -> 1.0
		else       f=fl+(p2-fl)*soc/(float)s2;                      //0..s2  : floor -> p2
	} else {
		f=fl+(1.0f-fl)*soc/(float)lim;                             //0..lim : floor -> 1.0
	}
	if(f<fl) f=fl;
	if(f>1.0f) f=1.0f;
	return f;
}

float default_wh_km_for_level(uint8_t lvl){
	//seed consumption per assist level (0..9). Eco=2:7, Tour=4:9, Sport=6:12, Sport+=8:16, Boost=9:18
	static const uint8_t t[10]={6,6,7,7,9,9,12,13,16,18};
	if(lvl>9) lvl=5;
	return (float)t[lvl];
}

uint8_t soc_state_load(void){
	soc_slot_t* slot;
	uint32_t best_seq=0; int32_t best=-1;
	for(int i=0;i<SOC_NUM_SLOTS;i++){
		slot=(soc_slot_t*)(SOC_FLASH_ADDR+(uint32_t)i*SOC_SLOT_SIZE);
		if(slot->seq==0xFFFFFFFFU) continue;
		if(soc_crc32((const uint8_t*)slot,28)!=slot->crc) continue;
		if(slot->seq>=best_seq){ best_seq=slot->seq; best=i; }
	}
	if(best<0) return 0;
	slot=(soc_slot_t*)(SOC_FLASH_ADDR+(uint32_t)best*SOC_SLOT_SIZE);
	MS.remaining_mah=slot->remaining_mah;
	MS.soc_real=(float)slot->soc_real_x10/10.0f;
	if(slot->capacity_est_mah && slot->capacity_est_mah!=0xFFFF) MP.battery_capacity_estimated_mah=slot->capacity_est_mah;
	soc_seq=best_seq; soc_slot_index=best;
	return 1;
}

// Self power-off: persist SOC once, stop PWM, release DC/DC enable + display latch.
// Shared by the on/off button, auto-off (inactivity) and the comms watchdog.
void power_off_controller(void){
	if(!shutdown_saved){ soc_state_save(); shutdown_saved=1; } //persist SOC before power down
	timer_primary_output_config(TIMER0,DISABLE); //stop PWM output
	GPIO_BC(GPIOB) = GPIO_PIN_4; //DC/DC enable off (self power-off)
	GPIO_BC(GPIOB) = GPIO_PIN_5; //Display off
}

void soc_state_save(void){
	fwdgt_counter_reload();
	soc_slot_t s;
	memset(&s,0,sizeof(s));
	s.seq=++soc_seq;
	s.remaining_mah=MS.remaining_mah;
	s.used_wh=MS.used_wh;
	s.capacity_est_mah=MP.battery_capacity_estimated_mah;
	s.soc_real_x10=(uint16_t)(MS.soc_real*10.0f);
	s.last_voltage_mv=(uint32_t)MS.Voltage;
	s.cycle_charge_mah=0;
	s.reserved=0;
	s.crc=soc_crc32((const uint8_t*)&s,28);

	int32_t next=soc_slot_index+1;
	uint32_t* chk=(uint32_t*)(SOC_FLASH_ADDR+(uint32_t)((next>=0 && next<SOC_NUM_SLOTS)?next:0)*SOC_SLOT_SIZE);
	if(next>=SOC_NUM_SLOTS || next<0 || *chk!=0xFFFFFFFFU){
		//page full or target slot not erased -> erase whole page and restart at slot 0
		fmc_unlock();
		fmc_flag_clear(FMC_FLAG_BANK0_END); fmc_flag_clear(FMC_FLAG_BANK0_WPERR); fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
		fmc_page_erase(SOC_FLASH_ADDR);
		fmc_flag_clear(FMC_FLAG_BANK0_END); fmc_flag_clear(FMC_FLAG_BANK0_WPERR); fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
		fmc_lock();
		fwdgt_counter_reload();
		next=0;
	}
	uint32_t addr=SOC_FLASH_ADDR+(uint32_t)next*SOC_SLOT_SIZE;
	uint32_t* w=(uint32_t*)&s;
	fmc_unlock();
	for(uint32_t k=0;k<SOC_SLOT_WORDS;k++){
		fmc_word_program(addr,w[k]);
		addr+=4;
		fmc_flag_clear(FMC_FLAG_BANK0_END); fmc_flag_clear(FMC_FLAG_BANK0_WPERR); fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
	}
	fmc_lock();
	soc_slot_index=next;
	soc_last_saved=MS.soc_real;
	soc_save_seconds=0;
	fwdgt_counter_reload();
}

void soc_init(void){
	//sanitize EEPROM-loaded params (new fields read 0xFF on first boot after firmware upgrade)
	if(MP.battery_capacity_mah==0 || MP.battery_capacity_mah==0xFFFF) MP.battery_capacity_mah=BATTERY_CAPACITY_MAH;
	if(MP.battery_capacity_estimated_mah==0 || MP.battery_capacity_estimated_mah==0xFFFF) MP.battery_capacity_estimated_mah=MP.battery_capacity_mah;
	if(MP.r_batt_mohm==0 || MP.r_batt_mohm==0xFFFF) MP.r_batt_mohm=R_BATT_MOHM;

	soc_mAs_acc=0; soc_tick_counter=0; soc_one_second_flag=0;
	uint8_t cells=(uint8_t)((float)MP.system_voltage/3.6f);
	float i_a=(float)MS.Battery_Current/1000.0f;
	uint16_t u_comp=(uint16_t)((float)MS.Voltage + i_a*(float)MP.r_batt_mohm);
	int8_t soc_ocv=calculate_SOC(u_comp,cells);

	//seed per-level range consumption with per-mode defaults
	for(uint8_t i=0;i<10;i++) wh_km_level[i]=default_wh_km_for_level(i);
	MS.avg_wh_per_km=wh_km_level[MS.assist_level<10?MS.assist_level:5];
	MS.used_wh=0;
	trip_distance_m_last=MS.distance_since_startup;

	if(soc_state_load()){
		//restart: detect recharge while powered off (no RTC -> OCV jump vs stored SOC)
		if((float)soc_ocv-MS.soc_real > (float)RECHARGE_MARGIN_PCT){
			MS.soc_real=(float)soc_ocv;                 //battery was charged -> trust OCV
			if(MS.soc_real>100)MS.soc_real=100;
			MS.remaining_mah=MS.soc_real/100.0f*(float)MP.battery_capacity_estimated_mah;
		}
	} else {
		//first ever boot: seed from OCV
		MS.soc_real=(float)soc_ocv;
		MS.remaining_mah=MS.soc_real/100.0f*(float)MP.battery_capacity_estimated_mah;
	}
	MS.soc_voltage=soc_ocv;
	MS.soc_display=MS.soc_real;
	MS.SOC=(uint8_t)(MS.soc_real+0.5f);
	soc_last_saved=MS.soc_real;
	soc_save_seconds=0;
	cycle_start_soc=-1.0f;
	cycle_discharge_mah=0;
}

void soc_update(void){
	//--- FW-018: boot-time full-charge detection (once, over the first SOC_FULL_BOOT_SETTLE_S seconds) ---
	//Compares the WHOLE-PACK voltage directly against the user threshold - no cell count, no /3.6.
	if(!soc_boot_full_done){
		if(MP.soc_full_magic==SOC_FULL_MAGIC){
			if(MS.Voltage<soc_boot_vmin) soc_boot_vmin=MS.Voltage;
			if(MS.Voltage>soc_boot_vmax) soc_boot_vmax=MS.Voltage;
			if(++soc_boot_settle_s>=SOC_FULL_BOOT_SETTLE_S){
				soc_boot_full_done=1;
				if((uint16_t)(soc_boot_vmax-soc_boot_vmin)<=SOC_FULL_BOOT_STABLE_MV &&
				   (uint32_t)MS.Voltage>=(uint32_t)MP.soc_full_pack_10mv*10U){
					MS.remaining_mah=(float)MP.battery_capacity_estimated_mah; //battery is full
					MS.soc_real=100.0f; MS.soc_display=100.0f; MS.SOC=100;
					soc_full_anchor=1;
					soc_anchor_start_mah=MS.remaining_mah;
				}
			}
		} else {
			soc_boot_full_done=1; //feature not configured -> skip, keep the coulomb counter
		}
	}

	//--- integrate this second's charge ---
	float dmah=soc_mAs_acc/3600.0f;   //mA*s -> mAh (signed)
	soc_mAs_acc=0;
	MS.remaining_mah-=dmah;            //discharge reduces; regen (dmah<0) adds back
	if(MS.remaining_mah>(float)MP.battery_capacity_estimated_mah) MS.remaining_mah=(float)MP.battery_capacity_estimated_mah;
	if(MS.remaining_mah<0) MS.remaining_mah=0;
	MS.used_wh+=(dmah/1000.0f)*((float)MS.Voltage/1000.0f); //Wh this second (signed)
	MS.soc_real=MS.remaining_mah/(float)MP.battery_capacity_estimated_mah*100.0f;

	//--- IR-compensated OCV lookup ---
	uint8_t cells=(uint8_t)((float)MP.system_voltage/3.6f);
	float i_a=(float)MS.Battery_Current/1000.0f;
	uint16_t u_comp=(uint16_t)((float)MS.Voltage + i_a*(float)MP.r_batt_mohm);
	MS.soc_voltage=calculate_SOC(u_comp,cells);

	//--- slow OCV correction only at rest (anti-drift), never a hard jump ---
	if(MS.Battery_Current<I_REST_MA && MS.Battery_Current>-I_REST_MA){
		if(rest_seconds<65000) rest_seconds++;
		if(rest_seconds>=REST_TIME_S){
			MS.soc_real+=OCV_CORR_GAIN*((float)MS.soc_voltage-MS.soc_real);
			MS.remaining_mah=MS.soc_real/100.0f*(float)MP.battery_capacity_estimated_mah;
		}
	} else {
		rest_seconds=0;
	}

	//--- SOC_display low-pass with max step per minute (anti-jump) ---
	float diff=MS.soc_real-MS.soc_display;
	float step=SOC_DISP_GAIN*diff;
	float max_step=SOC_DISP_MAX_STEP/60.0f; //per second
	if(MS.soc_real<10.0f) step=diff;        //near cutoff: converge fast, don't lag high
	if(step>max_step)step=max_step;
	if(step<-max_step)step=-max_step;
	MS.soc_display+=step;
	if(MS.soc_display<0)MS.soc_display=0;
	if(MS.soc_display>100)MS.soc_display=100;
	MS.SOC=(uint8_t)(MS.soc_display+0.5f);

	//--- FW-018: hold display at 100% right after a detected full charge (anti flicker to 99%) ---
	//Released once ~SOC_FULL_RELEASE_FRAC of capacity has actually been consumed; soc_real keeps tracking underneath.
	if(soc_full_anchor){
		if((soc_anchor_start_mah-MS.remaining_mah) < SOC_FULL_RELEASE_FRAC*(float)MP.battery_capacity_estimated_mah){
			MS.soc_display=100.0f; MS.SOC=100;
		} else {
			soc_full_anchor=0; //enough used -> resume normal display tracking
		}
	}

	//--- Range from remaining energy / PER-LEVEL average consumption ---
	float remaining_wh=(MS.remaining_mah/1000.0f)*(float)MP.system_voltage;
	uint8_t lvl=MS.assist_level<10?MS.assist_level:5;
	float dist_m=MS.distance_since_startup-trip_distance_m_last;
	if(dist_m>=RANGE_LEARN_MIN_M && MS.Speedx100>300){ //moving > 3 km/h
		float wh_km_now=MS.used_wh/(dist_m/1000.0f);
		if(wh_km_now>1.0f && wh_km_now<100.0f)
			wh_km_level[lvl]+=RANGE_EMA_ALPHA*(wh_km_now-wh_km_level[lvl]); //learn THIS level's consumption
		trip_distance_m_last=MS.distance_since_startup;
		MS.used_wh=0;
	}
	MS.avg_wh_per_km=wh_km_level[lvl]; //for display/debug
	if(wh_km_level[lvl]>0.5f) MS.range=(uint16_t)(remaining_wh/wh_km_level[lvl]);
	else MS.range=0;

	//--- capacity adaptation over (near) full discharge cycles ---
	if(dmah>0) cycle_discharge_mah+=dmah;
	if(cycle_start_soc<0 && MS.soc_real>92.0f && rest_seconds>=REST_TIME_S){
		cycle_start_soc=MS.soc_real; cycle_discharge_mah=0;
	}
	if(cycle_start_soc>90.0f && MS.soc_real<12.0f){
		float frac=(cycle_start_soc-MS.soc_real)/100.0f;
		if(frac>0.7f){
			float measured=cycle_discharge_mah/frac;
			float lo=(float)MP.battery_capacity_mah*0.5f, hi=(float)MP.battery_capacity_mah*1.5f;
			if(measured>lo && measured<hi)
				MP.battery_capacity_estimated_mah=(uint16_t)(0.95f*(float)MP.battery_capacity_estimated_mah+0.05f*measured);
		}
		cycle_start_soc=-1.0f; cycle_discharge_mah=0;
	}

	//--- periodic save (flash wear protection) ---
	if(soc_save_seconds<4000000000U) soc_save_seconds++;
	if(fabsf(MS.soc_real-soc_last_saved)>=(float)SOC_SAVE_DELTA && soc_save_seconds>=SAVE_MIN_INTERVAL_S)
		soc_state_save();
}

/*!
    \brief      erase fmc pages from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_erase_pages(void)
{
    uint32_t EraseCounter;

    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    /* erase the flash pages */
    for(EraseCounter = 0; EraseCounter < PageNum; EraseCounter++){
        fmc_page_erase(FMC_WRITE_START_ADDR + (FMC_PAGE_SIZE * EraseCounter));
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }

    /* lock the main FMC after the erase operation */
    fmc_lock();
}

/*!
    \brief      program fmc word by word from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_program_hall_angles(void)
{
    /* unlock the flash program/erase controller */
    fmc_unlock();

    address = FMC_WRITE_START_ADDR;

    /* program flash */

        fmc_word_program(address, (uint32_t)i32_hall_order);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Hall_13);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Hall_32);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Hall_26);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Hall_64);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Hall_45);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

        fmc_word_program(address, (uint32_t)Hall_51);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);


    /* lock the main FMC after the program operation */
    fmc_lock();
}

fmc_state_enum fmc_multi_word_program(uint32_t offset, uint8_t* data, uint8_t words)
{
	uint32_t temp=0;
	fmc_state_enum returnvalue;
	uint32_t target_address = FMC_WRITE_START_ADDR+offset;
    fmc_unlock();
            	for (k=0; k < words; k++){
            		memcpy(&temp, data+k*4,4);
            		returnvalue = fmc_word_program(target_address, (uint32_t)temp);
                    target_address += 4;
                    fmc_flag_clear(FMC_FLAG_BANK0_END);
                    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
                    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
            	}


     fmc_lock();
    return returnvalue;
}

//FW-023: a record counts as valid only once its trailing crc word has been programmed.
uint8_t param_record_valid(void)
	{
	const param_footer_t* f = (const param_footer_t*)(FMC_WRITE_START_ADDR+FMC_OFFSET_FOOTER);
	if(f->magic != PARAM_REC_MAGIC) return 0;
	if(f->version != PARAM_REC_VERSION) return 0;
	if(f->length != (uint16_t)FMC_OFFSET_FOOTER) return 0; //different MotorParams_t layout
	if(soc_crc32((const uint8_t*)FMC_WRITE_START_ADDR, f->length) != f->crc) return 0;
	return 1;
	}

//FW-023: a half-written record leaves most angles at 0xFFFFFFFF, which collapses the
//commutation table and makes the motor buzz instead of turning. Six real transitions always
//sit ~60 deg apart, so check that before trusting them.
uint8_t hall_angles_plausible(void)
	{
	int32_t a[6] = {Hall_13, Hall_32, Hall_26, Hall_64, Hall_45, Hall_51};
	if(i32_hall_order != 1 && i32_hall_order != -1) return 0;

	for(int i=1;i<6;i++){ //insertion sort, ascending
		int32_t key=a[i];
		int j=i-1;
		while(j>=0 && a[j]>key){ a[j+1]=a[j]; j--; }
		a[j+1]=key;
	}
	for(int i=0;i<6;i++){ //i==5 wraps past a full turn; unsigned subtraction handles it
		uint32_t gap = (uint32_t)a[(i+1)%6] - (uint32_t)a[i];
		if(gap < HALL_GAP_MIN_Q31 || gap > HALL_GAP_MAX_Q31) return 0;
	}
	return 1;
	}

void hall_load_defaults(void)
	{
	i32_hall_order = HALL_DEF_ORDER;
	Hall_13 = HALL_DEF_13;
	Hall_32 = HALL_DEF_32;
	Hall_26 = HALL_DEF_26;
	Hall_64 = HALL_DEF_64;
	Hall_45 = HALL_DEF_45;
	Hall_51 = HALL_DEF_51;
	}

void write_virtual_eeprom(void)
	{
		//FW-023: settings frames from the display repeat unchanged values on every boot. Erasing
		//and rewriting the page for those costs flash wear and opens the corruption window for
		//nothing, so skip the cycle when flash already holds exactly this content.
		int32_t halls[7] = {i32_hall_order, Hall_13, Hall_32, Hall_26, Hall_64, Hall_45, Hall_51};
		if(param_record_valid() &&
		   memcmp((const void*)FMC_WRITE_START_ADDR, halls, sizeof(halls))==0 &&
		   memcmp((const void*)(FMC_WRITE_START_ADDR+FMC_OFFSET_MP), &MP, sizeof(MP))==0){
			return;
		}

		fmc_erase_pages();
		fmc_program_hall_angles();
		fmc_multi_word_program(FMC_OFFSET_MP, (uint8_t*)&MP, (sizeof(MP)+3)/4); //Did not know padding yet :-)

		//footer last: crc is the final word, so an interrupted write can never validate
		param_footer_t f;
		f.magic = PARAM_REC_MAGIC;
		f.version = PARAM_REC_VERSION;
		f.length = (uint16_t)FMC_OFFSET_FOOTER;
		f.reserved = 0;
		f.crc = soc_crc32((const uint8_t*)FMC_WRITE_START_ADDR, FMC_OFFSET_FOOTER);
		fmc_multi_word_program(FMC_OFFSET_FOOTER, (uint8_t*)&f, sizeof(f)/4);
	}

void read_virtual_eeprom(void)
	{
    if(!param_record_valid()){
    	//FW-023: keep the calibrated hall values and MP defaults compiled into this build
    	hall_load_defaults();
    	param_record_state = PARAM_REC_STATE_DEFAULTS;
    	return;
    }

    //read individual hall angles from virtual EEPROM
    ptrd = (uint32_t *)FMC_WRITE_START_ADDR;
	i32_hall_order=(int32_t)(*ptrd);
	ptrd++;
	Hall_13 = (int32_t)(*ptrd);
	ptrd++;
	Hall_32 = (int32_t)(*ptrd);
	ptrd++;
	Hall_26 = (int32_t)(*ptrd);
	ptrd++;
	Hall_64 = (int32_t)(*ptrd);
	ptrd++;
	Hall_45 = (int32_t)(*ptrd);
	ptrd++;
	Hall_51 = (int32_t)(*ptrd);
	ptrd++;

     memcpy(&MP,(uint32_t *)(FMC_WRITE_START_ADDR+FMC_OFFSET_MP),sizeof(MP));

    if(hall_angles_plausible()){
    	param_record_state = PARAM_REC_STATE_OK;
    }
    else {
    	hall_load_defaults();
    	param_record_state = PARAM_REC_STATE_HALL_BAD;
    }
	}

/*
 * FW-094: Walk Assist Iq. Declared in motor_service.h, called by ride_control_update() on the
 * walk_active branch.
 *
 * Before FW-094 this lived inside legacy_assist_calculate_monolith(), which ran the whole
 * pre-ride-core cadence-assist body (cadence^exponent map, pressure floor, throttle override,
 * Overrun) and then overwrote the result on the Walk branch. None of that math reached the
 * motor from here; what follows is the part that did.
 */
uint16_t walk_assist_iq_request(void){
	//FW-060: reset once at the beginning of a complete WA request. wa_engaged is cleared only
	//when the complete request ends (main loop), so a brake/speed pause must not re-arm the
	//standstill kick in the same session.
	if(MS.pushassist_flag && !wa_engaged){
		walk_motor_reset();
		wa_engaged=1;
	}

	int32_t wa_iq;
	//Brake has first priority; then Error 25 (implausible torque signal), which cuts assist and
	//throttle alike and lets the shared adaptive ramp bring the running current down without a
	//jerk; then FW-013, no motor power while a load calibration runs (no Error 25, just zero).
	if(MS.brake_active_flag || torque_fault || torque_input_calibration_active()){
		wa_iq=0;
	}
	else{
		//FW-060: walk_assist_motor owns Hall/safety; walk_speed_controller owns the one-shot
		//energetic start, PI, anti-windup and complete Iq slew.
		//Both Hall values are snapshotted first: they are written by the capture ISR and must
		//be read as one coherent pair.
		uint16_t wa_erps_age = ui16_erps_counter;
		uint16_t wa_hall_ticks = ui16_timertics;
		walk_motor_input_t wa_in = {
			.active = 1,
			.brake = MS.brake_active_flag != RESET,
			.fault = MS.error_state != 0 || torque_fault != 0,
			.wheel_speed_x100 = MS.Speedx100,
			.max_wheel_speed_x100 = assist_modes_get_wa_max_wheel_x100(), //FW-043: per bank
			.motor_hall_ticks = wa_hall_ticks,
			.motor_erps_age_ticks = wa_erps_age,
			.motor_iq_actual = MS.i_q,
			.motor_iq_reference = MS.i_q_setpoint,
			//FW-042/043/051: walk_assist_speed repurposed as target CHAINRING rpm, now banked.
			//It was x100 (km/h), but that made every value above 6 unreachable: Canable
			//clamped at 6. Raw rpm now uses a validated 20..60 range; stale stored
			//values such as 600 are repaired to the default.
			.target_chainring_rpm = assist_modes_get_wa_target_rpm()
		};
		wa_iq=(int32_t)walk_motor_update(&wa_in, &wa_diag);
	}

	/*
	 * Shared limiter: undervoltage and controller temperature. walk_active suppresses the legal
	 * speed taper, exactly as it did when this call went through the removed
	 * assist_limits_apply_legacy() wrapper — Walk Assist has its own wheel-speed ceiling inside
	 * walk_motor_update(). `source` is not read at all while walk_active is set; NON_PEDAL is
	 * the honest value for a request made with the cranks stationary.
	 */
	assist_limits_input_t wa_limits = {
		.voltage_raw = voltage_raw_filtered,
		.voltage_min_raw = MP.voltage_min,
		.controller_temperature_c = MS.int_Temperature,
		.source = ASSIST_LIMIT_SOURCE_NON_PEDAL,
		.speed_x100 = MS.Speedx100,
		.speed_limit_x100 = speedlimitx100_scaled,
		.legal_enabled = MP.legalflag != 0,
		.offroad = MS.offroadflag != RESET,
		.walk_active = true
	};
	int32_t limited = assist_limits_apply(wa_iq, &wa_limits);
	if(limited<0) limited=0;
	if(limited>65535) limited=65535;

	/*
	 * Zero-target integral reset, restored with the rest of FW-093's revert.
	 *
	 * FW-093 removed this on the theory that wiping the integral leaves the current loop
	 * proportional-only exactly when it has to null the current against the back-EMF. The
	 * reasoning still looks right on paper — but the build that removed it (0.0299) does not
	 * turn the motor at all, and the build that had it (0.0297) does. Behaviour on the bike
	 * beats the argument, so this goes back exactly as it was.
	 *
	 * If the Hi-Z coast is attempted again, this is one of the two things that must be
	 * revisited deliberately rather than in passing.
	 */
	if(!limited && PI_iq.integral_part){
		PI_iq.integral_part=0;
		PI_id.integral_part=0;
	}
	return (uint16_t)limited;
}

/*
 * FW-094: phase 2 of position-sensor calibration. Declared in motor_service.h, called by
 * ride_control_update() on the position_calibration_active branch.
 *
 * The monolith computed a full assist value here too and then unconditionally replaced it with
 * the fixed probe current below, so removing that computation changes nothing.
 */
uint16_t hall_calibration_iq_request(void){
	int32_t cal_iq=100;
	temp6-=temp6>>4;
	temp6+=MS.u_d;
	// Never accept/store a phase-2 result while the bridge is off. At the phase transition
	// ui_8_PWM_ON_Flag is deliberately zero; the normal main-loop start path sets it after it
	// physically enables PWM.
	if (p>70 && ui_8_PWM_ON_Flag){
		p=60;
		if ((MP.reverse*temp6>>4)>100)MP.angle_correction+=one_deg;
		else if ((MP.reverse*temp6>>4)<-50)MP.angle_correction-=one_deg;
		else {
			cal_iq=0;
			PI_iq.integral_part=0;
			PI_id.integral_part=0;
			timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,_T>>1);
			timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,_T>>1);
			timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,_T>>1);
			timer_primary_output_config(TIMER0,DISABLE); //Disable PWM
			ui_8_PWM_ON_Flag=0;
			pwm_cutoff_active=0;
			pwm_cutoff_tick=0;
			uint16_half_rotation_counter=0;
			write_virtual_eeprom();
			MS.hall_angle_detect_flag=1;
		}
	}
	//The deliberate integral reset above stays: there the bridge is switched off in the same
	//breath, so nothing is left regulating.
	return (uint16_t)cal_iq;
}


#ifdef GD_ECLIPSE_GCC
/* retarget the C library printf function to the USART, in Eclipse GCC environment */
int __io_putchar(int ch)
{
    usart_data_transmit(UART4, (uint8_t)ch);
    while(RESET == usart_flag_get(UART4, USART_FLAG_TBE));

    return ch;
}
#else
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(UART4, (uint8_t)ch);
    while(RESET == usart_flag_get(UART4, USART_FLAG_TBE));

    return ch;
}
#endif /* GD_ECLIPSE_GCC */
