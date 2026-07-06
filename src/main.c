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
uint8_t interpolate_assistfactor(void);
int8_t calculate_SOC(uint16_t voltage, uint8_t cells_in_series);
void print_debug_on_CAN(void);
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
uint16_t map_rezi(int32_t actual_value, int32_t actual_time, int32_t timeout, int32_t decay_base);
uint16_t update_setpoint(void);
int16_t T_NTC(uint16_t ADC);
float u32_to_deg=0.00000008381903171539;
uint16_t slow_loop_counter=0;
uint16_t t3100_counter=0;
uint16_t PAS_counter=0;
uint16_t torque_counter=0;
uint8_t overtemp_stage=0;           //thermal protection stage: 0 ok, 1 derate/warn, 2 cutoff
uint16_t wa_ramp_ticks=0;           //counts up while Walk Assist engaged: kickstart slew envelope over WA_RAMP_TICKS
int32_t wa_integral=0;              //Walk Assist speed PI integrator
uint8_t wa_engaged=0;              //edge-detect: WA engaged this cycle (decide kick vs resume once)
uint16_t err_pulse_counter=0;       //seconds counter for pulsed Error 10 in stage 1
uint16_t Speed_counter=0;
int32_t ButtonVoltageCumulated=620<<6;
#define iabs(x) (((x) >= 0)?(x):-(x))
#define sign(x) (((x) >= 0)?(1):(-1))
MotorState_t MS;
MotorParams_t MP;
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
uint16_t pas_idle_ticks=0;   //ticks since last quadrature transition (for stop detection)
uint8_t forward_pedaling=0;  //1 = cranks turning forward (cadence>0, not reverse, not stopped)
uint8_t fwd_run=0;           //consecutive forward quadrature steps (reset on any backward step or stop) -> jiggle-proof engage gate
uint8_t ui8_overflow_flag=0;
uint8_t ui8_SPEED_control_flag=0;
uint8_t ui8_walk_btn_counter=0;
uint8_t ui8_walk_btn_state=0;
uint8_t ui8_WA_blocked=0;
uint32_t ui32_WA_timer=0;
uint32_t voltage_raw_cumulated=0;
uint16_t voltage_raw_filtered=0;
int32_t torque_offset_correction=0;
//--- torque sensor fault + cyclic offset re-zero state ---
uint8_t  torque_fault=0;        // Error 25 active (out-of-range signal debounced, or implausible rest)
uint16_t tq_fault_ticks=0;      // debounce for out-of-range signal
uint8_t  tq_cal_fault=0;        // implausible rest at startup/coast (raw outside plausible window)
int32_t  tq_rest_acc=0;         // EMA(<<6) of torque_on_crank accumulated during a coast
uint8_t  tq_coast_active=0;     // currently in a qualifying coast (pedals idle)
uint16_t tq_coast_ticks=0;      // ticks accumulated in current coast (settle)
int32_t  tq_last_coast_rest=0;  // rest from previous out-of-band coast (consistency check)
uint8_t  tq_reacq_count=0;      // consecutive consistent out-of-band coasts
uint32_t ui32_erps_cumulated=0;
int32_t q31_rotorposition_hall=0;
q31_t q31_rotorposition_absolute=0;
int8_t i8_recent_rotor_direction=0;


uint16_t ui16_tim2_recent=0;
uint16_t uint16_full_rotation_counter=0;
uint16_t uint16_half_rotation_counter=0;
uint16_t speedlimitx100_scaled=0;
int16_t phase_current_max_scaled=0;
int8_t assist_level_old=0;
q31_t q31_u_d_temp=0;
q31_t q31_u_q_temp=0;
//Hall64	691967230
//Hall26	-11930205
//Hall32	-811271360
//Hall13	-1479377400
//Hall51	2123622926
//Hall45	1348142805

int32_t i32_hall_order =-1;
int32_t Hall_13 = 1825361405;
int32_t Hall_32 = -1789569490;
int32_t Hall_26 = -966367405;
int32_t Hall_64 = -322122295;
int32_t Hall_45 = 381775140;
int32_t Hall_51 = 1169185830;

const int32_t one_deg = 11930465; //one degree in 2^32 logic

int32_t i32_full_rotation_flag =-1;

float 	helper=0;
int32_t q31_PLL_error=0;
int32_t q31_rotorposition_PLL=0;
uint8_t ui_8_PLL_counter=0;
uint8_t shutoffcounter=0;
uint16_t offroadcode=0;
uint16_t offroadcounter=0;
uint16_t pulse_counter=0;
uint8_t ui_8_PWM_ON_Flag=0;
int32_t q31_angle_per_tic=0;
//Rotor angle scaled from degree to q31 for arm_math. -180Ã‚Â°-->-2^31, 0Ã‚Â°-->0, +180Ã‚Â°-->+2^31
const int32_t deg_30 = 357913941;
uint16_t switchtime[3];
uint16_t ui16_erps=0;
uint16_t ui16_erps_counter=0;
uint16_t mapped_throttle=0;
uint16_t mapped_torque=0;
static int32_t iq_setpoint_q=0;
char char_dyn_adc_state_old=1;
int16_t i16_ph1_current=0;
int16_t i16_ph2_current=0;
int16_t i16_ph3_current=0;

int8_t i8_reverse_flag = 1;
const q31_t tics_lower_limit = WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*SPEEDLIMIT*10); //tics=wheelcirc*timerfrequency/(no. of hallevents per rev*gear-ratio*speedlimit)*3600/1000000
const q31_t tics_higher_limit = WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*(SPEEDLIMIT+2)*10);
uint8_t i = 0;
uint16_t p = 0;
uint16_t Overrun_strength = 0;
uint16_t Overrun_counter = 0;
uint8_t Overrun_flag = 0;
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
uint32_t idle_ticks_slow=0;       //auto-off: slow-loop (40 ms) ticks with no rider/comms activity
volatile uint16_t comm_lost_ticks=0; //comms watchdog: slow-loop ticks since last HMI frame (reset in processCAN_Rx)
volatile uint8_t comm_seen=0;     //comms watchdog: 1 after first HMI frame -> arms the watchdog (grace period at boot)
uint8_t auto_off_minutes=AUTO_OFF_MINUTES; //runtime auto-off timeout [min]; overwritten by HMI 0x6303
uint32_t Speedx100_cumulated=0;
uint32_t torque_cumulated=0;
uint8_t array_temp[88];

uint8_t level_to_array_element[10]={0,0,1,0,2,0,3,0,4,5}; //map assist Level to array element
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
	MS.i_q_setpoint = 0;
	MS.i_d_setpoint = 0;
	MS.angle_est=SPEED_PLL;
	MS.pushassist_flag=SET;
	MS.light_flag=SET;
	MS.button_up_flag=SET;
	MS.button_down_flag=SET;
	MS.offroadflag=RESET;
	MS.offroadtics=0;
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
	MP.angle_correction=0;


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

    //Check, if virtual EEPROM was ever written. If not, fill it with default values
    ptrd = (uint32_t *)FMC_WRITE_START_ADDR;
    if(0xFFFFFFFF == (*(ptrd+1))){
    	InitEEPROM(&MP);
    }
    //read parameters from virtual EEPROM and overwrite the default values
    read_virtual_eeprom();
    parse_MOparams(&MP);

    for (int i = 0; i < 2000; i++) {//let the ADC stabilize
    	while(!reg_ADC_flag);
    	reg_ADC_flag=0;
    }

    for (int i = 0; i < 64; i++) {// get torquesensor offset
    	torque_offset_correction+=adc_value[2];
    	while(!reg_ADC_flag);
    	reg_ADC_flag=0;

    }
    torque_offset_correction=(torque_offset_correction>>6);
    torque_offset_correction=740-((torque_offset_correction*3300)>>12);
    //sanity-check: raw unloaded baseline must be plausible; else pedal pressed/sensor bad at boot -> don't trust zero
    {
        int32_t raw_baseline = 740 - torque_offset_correction; // = (avg_adc*3300)>>12
        if(raw_baseline<TQ_REST_RAW_MIN || raw_baseline>TQ_REST_RAW_MAX){
            torque_offset_correction = 0;  // nominal (read sensor as-is) instead of a bad zero
            tq_cal_fault = 1;              // raise Error 25 until a valid coast re-zero
        }
    }
  //  while((adc_value[1])>3000);//safety for bricked throttle

	helper=((float)1.0/((float)1.0+(float)MP.Cadence_exponent));
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
    	if(offroadcounter>4000){
    		offroadcode=0;
    		MS.offroadtics=0;
    	}
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
    	// update scaled current and speed
    	if(MS.assist_level!=assist_level_old){
    		//range learns per level -> reset the learning window so a window stays within one level
    		trip_distance_m_last=MS.distance_since_startup; MS.used_wh=0;
    		speedlimitx100_scaled=MP.speedLimitx100*MP.assist_settings[level_to_array_element[MS.assist_level]][1]/100;
    		phase_current_max_scaled=MP.phase_current_max*MP.assist_settings[level_to_array_element[MS.assist_level]][0]/100;
        	MS.TQfilter=level_to_array_element[MS.assist_level];
        	MS.TQfilter=MP.assist_settings[MS.TQfilter][2];
        	//SAFETY: TQfilter is used as a bit-shift (torque_cumulated>>TQfilter). Ride-mode values >7 (or, via
        	//int8_t, negative) make the shift undefined -> torque_filtered=0 -> that level's assist dies (hit S+/Boost).
        	if(MS.TQfilter<1 || MS.TQfilter>7) MS.TQfilter=4;
        	MS.ext_boost_duration=MP.ext_boost_duration[level_to_array_element[MS.assist_level]];
        	MS.ext_boost_strength=MP.ext_boost_strength[level_to_array_element[MS.assist_level]];
        	if(offroadcounter<4000&&offroadcounter>1000){
        		offroadcode+=pow(10,MS.offroadtics)*MS.assist_level;
        		MS.offroadtics++;
        	}

        	if(offroadcode==MP.MagicNumber){
        		MS.offroadflag=!MS.offroadflag;
        		if(MS.offroadflag)MS.offroadtics=9;
        		else MS.offroadtics=8;
        	}
        	offroadcounter=0;

    		assist_level_old=MS.assist_level;

    	}

#if SEND_DEV_TELEMETRY
            if(t3100_counter > 40){ t3100_counter=0; sendCAN_3100(&MS); } //40/4000Hz=10ms torque sensor emulation (dev telemetry - OFF by default; floods bus & can block HMI info at startup)
#endif

            if (slow_loop_counter > 160){ //slow loop base tick 40ms (160/4000Hz); CAN messages use own counters
            	gd_eval_led_toggle(LED2);
#ifdef PRINTDEBUG_UART

            	//printf("%d, %d, %d, %d, %d\r\n",MS.Battery_Current,MS.i_q_setpoint,MP.reverse*MS.i_q,ui16_erps,temp2);
            	printf("%d, %d, %d, %d, %d\r\n",MS.Battery_Current,MS.i_q_setpoint,MP.reverse*MS.i_q,MS.p_human,MS.Speedx100);
#endif

#ifdef PRINTDEBUG_CAN
            	print_debug_on_CAN();
#endif
//            	if((Overrun_strength-mapped_torque)>>3>0){
//            		Overrun_strength-=(Overrun_strength-mapped_torque)>>3;
//            	}
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
            	if(Speed_counter>20000) MS.Speedx100=0;
				slow_loop_counter = 0;

				if(adc_value[5]<2800)shutoffcounter++; //raw value is 4095 without button pressed, about 3300 with "down" button pressed and about 2400 with on/off button pressed.
				else shutoffcounter=0;
				if(shutoffcounter>62){ //62x40ms=2480ms, poprzednio 50x50ms=2500ms
					power_off_controller(); //on/off button held -> self power-off
				}

				//--- Auto-off after inactivity: reset counter on any rider/comms activity, else count up.
				if(MS.Speedx100>0 || MS.cadence>0 || MS.i_q_setpoint>0 || MS.brake_active_flag || adc_value[5]<3300){
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
						MS.i_q_setpoint=0;
					}
					if(comm_lost_ticks >= COMM_OFF_TICKS && MS.Speedx100==0){ //10 s no HMI frame -> power off, but only at standstill
						power_off_controller();
					}
				}

            }//end slow loop

            if(MS.i_q_setpoint){
            	if(!ui_8_PWM_ON_Flag){
            		get_standstill_position();
					timer_primary_output_config(TIMER0,ENABLE);
					uint16_half_rotation_counter=0;
					ui_8_PWM_ON_Flag=1;
            	}
            }
            if(uint16_half_rotation_counter>4000) {
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

void Speed_processing(void)
{
		Speedx100_cumulated-=Speedx100_cumulated/MP.pulses_per_revolution;
		Speedx100_cumulated+=MP.wheel_cirumference*4*360/(MP.pulses_per_revolution*Speed_counter);// 4000 Hz Timer interrupt frequency
		MS.Speedx100=Speedx100_cumulated/MP.pulses_per_revolution;
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
	MS.torque_on_crank=(((adc_value[2])*3300)>>12)+torque_offset_correction; //map ADC value to mV
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
			pas_qstate=s;
			if(st>0){            //forward step
				pas_idle_ticks=0;
				if(fwd_run<250)fwd_run++;   //consecutive forward steps (jiggle-proof engage)
				if(Backwards_counter)Backwards_counter--;
				// torque EMA @ 3.75deg (96 updates/rev) - full quadrature resolution so all algorithms see torque every step, not only every 15deg
				torque_cumulated-=torque_cumulated>>MS.TQfilter;
				if(MS.torque_on_crank>750) torque_cumulated+=(MS.torque_on_crank-750);
				MS.torque_filtered=(torque_cumulated>>MS.TQfilter);
				if(MS.torque_filtered>0) torque_counter=0; // cadence-gate: hold motor engaged while pedalling with any torque
#if START_CADENCE_SEED_ENABLE
				if(MS.cadence==0 && fwd_run>=START_CADENCE_SEED_STEPS && MS.torque_on_crank>(750+TQ_GATE_MIN)){
					MS.cadence=START_CADENCE_SEED_RPM;
					uint16_cadence_filtered=(uint16_t)START_CADENCE_SEED_RPM<<3;
					MS.p_human=(uint16_t)((float)(MS.cadence*MS.torque_filtered)*0.00342);
				}
#endif
				if(++pas_fwd_steps>=PAS_STEPS_PER_PULSE){ //one cadence pulse every PAS_STEPS_PER_PULSE forward transitions (see config.h)
					pas_fwd_steps=0;
					if(pas_cycle_ticks>70){
						MS.cadence=10000/pas_cycle_ticks;
						uint16_cadence_filtered-=uint16_cadence_filtered>>3;
						uint16_cadence_filtered+=MS.cadence;
						MS.p_human=(uint16_t)((float)(MS.cadence*MS.torque_filtered)*0.00342);
					}
					pas_cycle_ticks=0;
					PAS_counter=0;
				}
			}else if(st<0){      //backward step
				pas_idle_ticks=0;
				pas_fwd_steps=0;
				fwd_run=0;                  //any reverse step cancels the forward run -> rejects back/forth crank jiggle
				if(Backwards_counter<10)Backwards_counter++;
			}
		}
		if(pas_idle_ticks>PAS_STOP_TICKS){ MS.cadence=0; uint16_cadence_filtered=0; pas_fwd_steps=0; fwd_run=0; } //stop
		forward_pedaling = (MS.cadence>0 && Backwards_counter<4 && pas_idle_ticks<=PAS_STOP_TICKS);
	}
	//--- torque sensor fault detection (debounced) -> Error 25 ---
	if(MS.torque_on_crank<TQ_FAULT_LOW_MV || MS.torque_on_crank>TQ_FAULT_HIGH_MV){
		if(tq_fault_ticks<64000) tq_fault_ticks++;
	}else tq_fault_ticks=0;
	torque_fault = (tq_fault_ticks>TQ_FAULT_TICKS) || tq_cal_fault;
	//--- cyclic offset re-zero on coast (pedals idle >= TQ_RECAL_IDLE_TICKS; catches in-ride coasting) ---
	if(pas_idle_ticks>TQ_RECAL_IDLE_TICKS && tq_fault_ticks==0){
		if(!tq_coast_active){ tq_coast_active=1; tq_coast_ticks=0; tq_rest_acc=(int32_t)MS.torque_on_crank<<6; } //coast start: seed EMA
		else { tq_rest_acc-=tq_rest_acc>>6; tq_rest_acc+=MS.torque_on_crank; if(tq_coast_ticks<64000)tq_coast_ticks++; } //accumulate rest
	}else{
		if(tq_coast_active && tq_coast_ticks>TQ_RECAL_SETTLE_TICKS){ //a settled coast just ended -> evaluate re-zero
			int32_t rest = tq_rest_acc>>6;
			int32_t raw_rest = rest - torque_offset_correction;          //physical unloaded reading (pre-normalization)
			if(raw_rest<TQ_REST_RAW_MIN || raw_rest>TQ_REST_RAW_MAX){
				tq_cal_fault=1;                                          //implausible zero -> Error 25, no re-zero (anti-infinite-drift)
			}else{
				tq_cal_fault=0;
				int32_t diff = 740-rest;                                 //>0: reading too low, raise it
				int32_t adiff = diff<0?-diff:diff;
				uint8_t do_rezero=0;
				if(adiff<=TQ_RECAL_BAND_MV){ do_rezero=1; tq_reacq_count=0; }       //in band -> re-zero now
				else{                                                              //out of band -> need consistency over coasts
					int32_t d2 = rest-tq_last_coast_rest; if(d2<0)d2=-d2;
					if(tq_reacq_count>0 && d2<=TQ_REACQUIRE_TOL_MV) tq_reacq_count++; else tq_reacq_count=1;
					tq_last_coast_rest=rest;
					if(tq_reacq_count>=TQ_REACQUIRE_COASTS){ do_rezero=1; tq_reacq_count=0; } //real drift confirmed
				}
				if(do_rezero){ //rate-limited step; absolute bound guaranteed by raw_rest window check above
					int32_t step=diff;
					if(step>TQ_RECAL_MAX_STEP)step=TQ_RECAL_MAX_STEP; else if(step<-TQ_RECAL_MAX_STEP)step=-TQ_RECAL_MAX_STEP;
					torque_offset_correction += step;
				}
			}
		}
		tq_coast_active=0;
	}
	//--- coulomb counting (signed: discharge>0 reduces charge, regen<0 adds back) ---
	soc_mAs_acc += (float)MS.Battery_Current / 4000.0f; //mA * (1/4000 s) per ~4kHz tick
	if(++soc_tick_counter >= 4000){                      //~1 second elapsed
		soc_tick_counter = 0;
		soc_one_second_flag = 1;
	}
    slow_loop_counter++;
    t3100_counter++;
    if(torque_counter<64000)torque_counter++;
    if(PAS_counter<64000)PAS_counter++;
    if(Speed_counter<64000)Speed_counter++;
    if(uint16_half_rotation_counter<64000)uint16_half_rotation_counter++;
    if(offroadcounter<64000)offroadcounter++;
    if(ui16_erps_counter<64000)ui16_erps_counter++;
    if(Overrun_counter<64000)Overrun_counter++;

    //--- Walk Assist physical button (PA4), debounce z histereza (press + release) ---
    uint8_t wa_btn_in_range=(adc_value[5]>=WA_BUTTON_THRESHOLD_LOW && adc_value[5]<=WA_BUTTON_THRESHOLD_HIGH);
    if(!ui8_walk_btn_state){
        if(wa_btn_in_range){ if(++ui8_walk_btn_counter>=WA_BUTTON_DEBOUNCE){ui8_walk_btn_state=1; ui8_walk_btn_counter=0;} }
        else ui8_walk_btn_counter=0;
    }else{
        if(!wa_btn_in_range){ if(++ui8_walk_btn_counter>=WA_BUTTON_RELEASE){ui8_walk_btn_state=0; ui8_walk_btn_counter=0;} }
        else ui8_walk_btn_counter=0;
    }

    //--- walk_active = AND wszystkich warunkow ---
    uint8_t walk_speed_ok=(MS.Speedx100<700);
    uint8_t walk_active=MS.walk_can_request
                     && ui8_walk_btn_state
                     && walk_speed_ok
                     && !MS.brake_active_flag
                     && !MS.error_state
                     && !ui8_WA_blocked;

    //--- pushassist_flag — tylko main.c ustawia ---
    MS.pushassist_flag=walk_active?SET:RESET;

    //--- WA timer - timeout 10s ---
    if(walk_active){
        if(ui32_WA_timer<(WA_TIMEOUT_TICKS+1))ui32_WA_timer++;
    }else{
        ui32_WA_timer=0;
    }
    if(ui32_WA_timer>WA_TIMEOUT_TICKS){
        ui8_WA_blocked=1;
        MS.pushassist_flag=RESET;
        ui32_WA_timer=0;
    }
    //--- WA_blocked release: tylko po puszczeniu PA4 i zaniku walk_can_request ---
    if(ui8_WA_blocked && !MS.walk_can_request && !ui8_walk_btn_state){
        ui8_WA_blocked=0;
    }

    {
        // Engage/disengage ramp on i_q. Time mode keeps TSDZ2-like feel even when integer steps would be too coarse.
        int32_t iq_target = update_setpoint();
#if IQ_RAMP_TIME_MODE
        if(MS.brake_active_flag || Backwards_counter>=4 || overtemp_stage>=2){
            MS.i_q_setpoint = iq_target;                                       // safety cuts = immediate, no ramp
            iq_setpoint_q = iq_target << IQ_RAMP_Q_SHIFT;
        }else{
#if IQ_RAMP_ADAPTIVE
            int32_t up_s = map((int32_t)MS.Speedx100, IQ_RAMP_SPEED_LO, IQ_RAMP_SPEED_HI, IQ_RAMP_UP_SLOW_TICKS, IQ_RAMP_UP_FAST_TICKS);
            int32_t up_c = map((int32_t)MS.cadence,   IQ_RAMP_CAD_LO,   IQ_RAMP_CAD_HI,   IQ_RAMP_UP_SLOW_TICKS, IQ_RAMP_UP_FAST_TICKS);
            int32_t dn_s = map((int32_t)MS.Speedx100, IQ_RAMP_SPEED_LO, IQ_RAMP_SPEED_HI, IQ_RAMP_DOWN_SLOW_TICKS, IQ_RAMP_DOWN_FAST_TICKS);
            int32_t dn_c = map((int32_t)MS.cadence,   IQ_RAMP_CAD_LO,   IQ_RAMP_CAD_HI,   IQ_RAMP_DOWN_SLOW_TICKS, IQ_RAMP_DOWN_FAST_TICKS);
            int32_t up_ticks = (up_c<up_s)?up_c:up_s;                         // smaller tick count = faster ramp
            int32_t dn_ticks = (dn_c<dn_s)?dn_c:dn_s;
#else
            int32_t up_ticks = IQ_RAMP_UP_SLOW_TICKS;
            int32_t dn_ticks = IQ_RAMP_DOWN_SLOW_TICKS;
#endif
            int32_t iq_scale = phase_current_max_scaled;
            if(iq_scale<1) iq_scale = MP.phase_current_max;
            if(iq_scale<1) iq_scale = PH_CURRENT_MAX;
            if(iq_scale<1) iq_scale = 1;

            int32_t target_q = iq_target << IQ_RAMP_Q_SHIFT;
            int32_t ticks = (target_q > iq_setpoint_q) ? up_ticks : dn_ticks;
            if(ticks<1) ticks=1;
            int32_t step_q = ((iq_scale << IQ_RAMP_Q_SHIFT) + ticks - 1) / ticks;
            if(step_q<1) step_q=1;

            if(target_q > iq_setpoint_q){
                int32_t d = target_q - iq_setpoint_q;
                iq_setpoint_q += (d>step_q)?step_q:d;
            }else if(target_q < iq_setpoint_q){
                int32_t d = iq_setpoint_q - target_q;
                iq_setpoint_q -= (d>step_q)?step_q:d;
            }

            MS.i_q_setpoint = (iq_setpoint_q + (1 << (IQ_RAMP_Q_SHIFT - 1))) >> IQ_RAMP_Q_SHIFT;
            if(iq_target==0 && MS.i_q_setpoint==0) iq_setpoint_q=0;
        }
#else
#if IQ_RAMP_ADAPTIVE
        int32_t up_s = map((int32_t)MS.Speedx100, IQ_RAMP_SPEED_LO, IQ_RAMP_SPEED_HI, IQ_SLEW_UP_SLOW, IQ_SLEW_UP_FAST);
        int32_t up_c = map((int32_t)MS.cadence,   IQ_RAMP_CAD_LO,   IQ_RAMP_CAD_HI,   IQ_SLEW_UP_SLOW, IQ_SLEW_UP_FAST);
        int32_t up_step = (up_c>up_s)?up_c:up_s; if(up_step<IQ_SLEW_UP_SLOW) up_step=IQ_SLEW_UP_SLOW;
        int32_t dn_step = map((int32_t)MS.Speedx100, IQ_RAMP_SPEED_LO, IQ_RAMP_SPEED_HI, IQ_SLEW_DOWN_SLOW, IQ_SLEW_DOWN_FAST);
        if(dn_step<IQ_SLEW_DOWN_SLOW) dn_step=IQ_SLEW_DOWN_SLOW;
#else
    	int32_t up_step = IQ_SLEW_UP, dn_step = IQ_SLEW_DOWN;
#endif
    	if(MS.brake_active_flag || Backwards_counter>=4 || overtemp_stage>=2){
    		MS.i_q_setpoint = iq_target;                                       //safety cuts = immediate, no ramp
    	}else if(iq_target > MS.i_q_setpoint){
    		int32_t d=iq_target-MS.i_q_setpoint; MS.i_q_setpoint += (d>up_step)?up_step:d;
        }else{
            int32_t d=MS.i_q_setpoint-iq_target; MS.i_q_setpoint -= (d>dn_step)?dn_step:d;
        }
#endif
    }
    if (torque_counter>4000&&!Overrun_flag){ //reset after one second without torque on the pedal
    	if (PAS_counter>MP.PAS_timeout){
			Backwards_counter=0;
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
	timer_primary_output_config(TIMER0,ENABLE);
	ui_8_PWM_ON_Flag=1;
	MS.hall_angle_detect_flag = 0; //set uq to contstant value in FOC.c for open loop control
	q31_rotorposition_absolute = 1 << 31;
	i32_hall_order = 1;//reset hall order
	MS.i_d_setpoint= 200; //set MS.id to appr. 2000mA
	MS.i_q_setpoint= 0;

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

	//ui_8_PWM_ON_Flag=0;

    MS.i_d = 0;
    MS.i_q = 0;
    MS.u_d=0;
    MS.u_q=0;
    MS.i_d_setpoint= 0;
    uint32_tics_filtered=1000000;


	if (i8_recent_rotor_direction == 1) {

		i32_hall_order = 1;
	} else {

		i32_hall_order = -1;
	}

	//write_virtual_eeprom();
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

uint8_t interpolate_assistfactor(void){
	uint16_t interval= speedlimitx100_scaled/5 ;
	uint8_t ui8_speedfactor=0;
	uint8_t ui8_speedcase=0;
	if (MS.Speedx100 < interval)ui8_speedcase=0;
	else if (MS.Speedx100 < 2*interval)ui8_speedcase=1;
	else if (MS.Speedx100 < 3*interval)ui8_speedcase=2;
	else if (MS.Speedx100 < 4*interval)ui8_speedcase=3;
	else ui8_speedcase=4;

	ui8_speedfactor = map(
			MS.Speedx100,
			ui8_speedcase*interval,
			(ui8_speedcase+1)*interval,
			MP.assist_profile[level_to_array_element[MS.assist_level]-1][ui8_speedcase],
			MP.assist_profile[level_to_array_element[MS.assist_level]-1][ui8_speedcase+1]);
	return ui8_speedfactor;
}

void print_debug_on_CAN(void){


	transmit_message.tx_sfid = 0x00;
	transmit_message.tx_efid = 0x00010203; //ID for debug message
	transmit_message.tx_ft = CAN_FT_DATA;
	transmit_message.tx_ff = CAN_FF_EXTENDED;
	transmit_message.tx_dlen = 8;
	transmit_message.tx_data[0] = (temp1>>8)&0xFF;//(GPIO_ISTAT(GPIOC)>>6)&0x07;
	transmit_message.tx_data[1] = (temp1)&0xFF; //ui16_timertics>>8;//(GPIO_ISTAT(GPIOA)>>8)&0xFF;
	transmit_message.tx_data[2] = (temp2>>8)&0xFF;;
	transmit_message.tx_data[3] = (temp2)&0xFF;
	transmit_message.tx_data[4] = (temp3>>8)&0xFF;//
	transmit_message.tx_data[5] = (temp3)&0xFF;
	transmit_message.tx_data[6] = (temp4>>8)&0xFF;
	transmit_message.tx_data[7] = (temp4)&0xFF;

	/* transmit message */
	transmit_mailbox = can_message_transmit(CAN0, &transmit_message);
	/* waiting for transmit completed */
	timeout = 0xFFFF;
	while((CAN_TRANSMIT_OK != can_transmit_states(CAN0, transmit_mailbox)) && (0 != timeout)){
		timeout--;
		}

}

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

void write_virtual_eeprom(void)
	{
		fmc_erase_pages();
		fmc_program_hall_angles();
		fmc_multi_word_program(FMC_OFFSET_MP, (uint8_t*)&MP, (sizeof(MP)+3)/4); //Did not know padding yet :-)
	}

void read_virtual_eeprom(void)
	{
    //read individual hall angles from virtual EEPROM
    ptrd = (uint32_t *)FMC_WRITE_START_ADDR;
    if(0xFFFFFFFF != (*(ptrd+1))){
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
    }

     memcpy(&MP,(uint32_t *)(FMC_WRITE_START_ADDR+FMC_OFFSET_MP),sizeof(MP));
	}

uint16_t map_rezi(int32_t actual_value, int32_t actual_time, int32_t timeout, int32_t decay_base){
    if(actual_time<timeout)return actual_value;
    else if(actual_time<4000) return (uint16_t)((float)actual_value/((1+(float)(actual_value*decay_base)/175000*(float)(actual_time-timeout))));
    else return 0;
}

uint16_t update_setpoint(void){

				//Walk Assist engage edge: decide kick (from standstill) vs smooth resume (already rolling), clear integrator
				if(MS.pushassist_flag && !wa_engaged){
					wa_integral=0;                                                      //bumpless: clean integrator each engage
					wa_ramp_ticks=(MS.Speedx100<WA_KICK_SPEED)?0:WA_RAMP_TICKS;          //kick only from standstill; rolling -> cap open, no extra kick
					wa_engaged=1;
				}
				if(!MS.pushassist_flag) wa_engaged=0;
				//calculate iq setpoint
	            //check brake with first priority
	            if(MS.brake_active_flag)MS.i_q_setpoint_temp=0;
	            // check push assist active: closed-loop speed PI holding MP.walk_assist_speed
	            else if(MS.pushassist_flag){
	            	int32_t wa_max = MP.phase_current_max*MP.walk_assist_current/100;     //WA current ceiling
	            	int32_t err    = (int32_t)MP.walk_assist_speed - (int32_t)MS.Speedx100; //speed error (0.01 km/h)
	            	int32_t out = ((err*WA_KP_NUM)>>WA_KP_SHIFT) + (wa_integral>>WA_KI_SHIFT); //P + I
	            	//anti-windup: integrate only when not pushing further into saturation (kills >6km/h overshoot)
	            	if(!((out>=wa_max && err>0) || (out<=0 && err<0)) && wa_ramp_ticks>=WA_RAMP_TICKS) wa_integral += err;
	            	int32_t imax = wa_max << WA_KI_SHIFT;
	            	if(wa_integral>imax) wa_integral=imax; else if(wa_integral<0) wa_integral=0;
	            	out = ((err*WA_KP_NUM)>>WA_KP_SHIFT) + (wa_integral>>WA_KI_SHIFT);    //recompute after I update
	            	if(out>wa_max) out=wa_max; else if(out<0) out=0;                      //clamp (0 = coast, no braking)
	            	if(wa_ramp_ticks<WA_RAMP_TICKS) wa_ramp_ticks++;                      //kickstart slew ~180 ms (firm, no jerk)
	            	int32_t cap = wa_max*(int32_t)wa_ramp_ticks/WA_RAMP_TICKS;
	            	if(out>cap) out=cap;
	            	MS.i_q_setpoint_temp=(uint32_t)out;
	            }
	            //calculate setpoint, if brake is not activated
	            else{
					mapped_throttle= map(adc_value[1], MP.throttle_offset, MP.throttle_max, 0, phase_current_max_scaled);
					mapped_torque= map(MS.torque_on_crank, MP.TQO_threshold[level_to_array_element[MS.assist_level]], TQ_FULL_SCALE_MV, 0, phase_current_max_scaled); //#4 upper span configurable (3300=old; lower=more pressure-linear)

					if(Backwards_counter<4){//normal ride mode, motor power only if pedals are not turned backwards
						//CONSISTENT ENGAGEMENT with HYSTERESIS: arm on firm press + >=START_MIN_STEPS forward steps;
						//once armed, HOLD until pressure drops near rest (TQ_GATE_RELEASE) or crank movement stops.
						//This stops the shudder (assist unloads pedal -> pressure dips -> would cut without hold).
						//Reverse/stop resets fwd_run -> can't false-engage on descent jiggle or dead-spots.
						static uint8_t assist_latched=0;
						uint8_t fwd_ok=(fwd_run>=START_MIN_STEPS);
						if(!assist_latched){
							if(fwd_ok && MS.torque_on_crank > (750+TQ_GATE_MIN)) assist_latched=1;
						}else{
							if(!fwd_ok) assist_latched=0;   //HOLD assist the whole time cranks turn forward; release ONLY when pedalling stops/reverses (not on pressure dips)
						}
						uint8_t engaged = assist_latched;
						if(!engaged){
							MS.i_q_setpoint_temp=0;
						}else{
#if ASSIST_TORQUE_MODE
							//Bosch-like PRESSURE mode: assist ∝ pedal pressure (cadence not used)
							MS.i_q_setpoint_temp = (uint32_t)mapped_torque;
#else
							//cadence-based assist (TS_coeff * cadence^helper * torque_filtered)
							MS.i_q_setpoint_temp=(uint32_t)((float) (MP.TS_coeff*powf((float)MS.cadence,helper))*(MS.torque_filtered)*0.0005*interpolate_assistfactor());//factor 0.0005 from various constants
							if(MS.i_q_setpoint_temp>phase_current_max_scaled)MS.i_q_setpoint_temp = phase_current_max_scaled;
							MS.i_q_setpoint_temp=map_rezi(MS.i_q_setpoint_temp, torque_counter, MP.PAS_timeout, MP.decay_base);
							//torque override: immediate pressure floor (only while engaged)
							if(mapped_torque>MS.i_q_setpoint_temp){
								if(mapped_torque>Overrun_strength){ Overrun_strength=mapped_torque; Overrun_counter = 0; }
								MS.i_q_setpoint_temp=mapped_torque;
							}
#endif
						}
					}
					else MS.i_q_setpoint_temp=0; //cut motor power on pedaling backwards
#if SMOOTH_START_ENABLE
					{	//#2 soft pull-away: scale pedal assist 0->100% over START_RAMP_TICKS after a standstill (throttle below is exempt)
						static uint16_t start_ramp=0;
						if(MS.Speedx100==0 && MS.cadence==0) start_ramp=0;
						else if(start_ramp<START_RAMP_TICKS) start_ramp++;
						if(start_ramp<START_RAMP_TICKS) MS.i_q_setpoint_temp=(uint32_t)((uint32_t)MS.i_q_setpoint_temp*start_ramp/START_RAMP_TICKS);
					}
#endif
						//throttle override
					if(mapped_throttle>MS.i_q_setpoint_temp)MS.i_q_setpoint_temp=mapped_throttle;

					//apply Extended Boost (holds power after pedal stops -> "przeciąganie". Gated by EXTENDED_BOOST_ENABLE; default OFF for smooth power-down)
					if(EXTENDED_BOOST_ENABLE && Overrun_counter<(MP.Override_Duration*MS.ext_boost_duration)/100&&Backwards_counter<4){
						MS.i_q_setpoint_temp=(Overrun_strength*MS.ext_boost_strength)/100;
						if(MS.i_q_setpoint_temp>MP.phase_current_max)MS.i_q_setpoint_temp = MP.phase_current_max;
						Overrun_flag=1;
						torque_counter=MP.PAS_timeout+1;
					}
					else {
						Overrun_strength=0;
						Overrun_flag=0;
					}


	            }// else brake not active
	        	//if(PAS_counter>MP.PAS_timeout)MS.i_q_setpoint_temp=0;

	            //low battery ramp down with 3V above battery min voltage
	            MS.i_q_setpoint_temp=map(voltage_raw_filtered, MP.voltage_min,(MP.voltage_min+176),0,MS.i_q_setpoint_temp);

	            //controller temperature ramp down between 75 and 90°C (M820: NTC mierzy temp sterownika)
	            MS.i_q_setpoint_temp=map(MS.int_Temperature,75,90,MS.i_q_setpoint_temp,0);
	            if(MP.legalflag&&!MS.offroadflag&&!MS.pushassist_flag){

					if((uint16_cadence_filtered>>3)>15){
						MS.i_q_setpoint_temp=map(MS.Speedx100, speedlimitx100_scaled,(speedlimitx100_scaled+200),MS.i_q_setpoint_temp,0);
					}
					else{ //limit to 6km/h if pedals are not turning
						MS.i_q_setpoint_temp=map(MS.Speedx100, 500,700,MS.i_q_setpoint_temp,0);
					}

				}//end legalflag
				if(MS.hall_angle_detect_flag>1){ // part 2 of positions calibration
					MS.i_q_setpoint_temp=100;
					temp6-=temp6>>4;
					temp6+=MS.u_d;
					if (p>70){
						p=60;
						if ((MP.reverse*temp6>>4)>100)MP.angle_correction+=one_deg;
						else if ((MP.reverse*temp6>>4)<-50)MP.angle_correction-=one_deg;
						else {
							MS.i_q_setpoint_temp=0;
							PI_iq.integral_part=0;
							PI_id.integral_part=0;
							timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,_T>>1);
							timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,_T>>1);
							timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,_T>>1);
							timer_primary_output_config(TIMER0,DISABLE); //Disable PWM
							write_virtual_eeprom();
							MS.hall_angle_detect_flag=1;
						}
					}

				}
				if(!MS.i_q_setpoint_temp&&PI_iq.integral_part){
					PI_iq.integral_part=0;
					PI_id.integral_part=0;
				}


	    		return MS.i_q_setpoint_temp;

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

