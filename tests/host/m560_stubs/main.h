#ifndef M560_HOST_MAIN_H
#define M560_HOST_MAIN_H
#define MAIN_H
#include <stdint.h>
#include <string.h>
#define CAL_I 1.0f
#define CAL_BAT_V 1
#define SYSTEM_VOLTAGE 48
#define MAX_VOLTAGE 54
#define BATTERYCURRENT_MAX 15000
#define PH_CURRENT_MAX 700
#define VOLTAGE_MIN 35000
#define SPEEDLIMIT 2500
#define WHEEL_CIRCUMFERENCE 2200
#define TS_COEF 100
#define GEAR_RATIO 40
#define THROTTLE_OFFSET 10
#define THROTTLE_MAX 4000
#define PULSES_PER_REVOLUTION 12
#define PAS_TIMEOUT 4000
#define RAMP_END 100
#define WALK_ASSIST_RPM_MIN 10
#define WALK_ASSIST_RPM_MAX 60
#define WALK_ASSIST_RPM_DEFAULT 30
#define WALK_ASSIST_CURRENT_DEFAULT 20
#define REVERSE 1
#define LEGALFLAG 1
#define BATTERY_CAPACITY_MAH 10000
#define R_BATT_MOHM 100
#define LIMP_DISABLED 255
#define WHEEL_DIAMETER_MAGIC 0x5744
#define WHEEL_DIAMETER_CODE_0 0xB5
#define WHEEL_DIAMETER_CODE_1 1
typedef struct {
    uint16_t wheel_cirumference,decay_base,Cadence_exponent,Override_Duration,MagicNumber;
    uint16_t TS_coeff,PAS_timeout,ramp_end,throttle_offset,throttle_max,active_profile_bank;
    uint16_t torque_full_scale_native,gear_ratio,phase_current_max,battery_current_max;
    int16_t voltage_min;
    uint16_t speedLimitx100,walk_assist_speed;
    uint8_t walk_assist_current;
    uint16_t TQO_threshold[6];
    int8_t system_voltage,max_voltage,reverse,legalflag;
    uint8_t pulses_per_revolution,assist_profile[5][6],assist_settings[6][3];
    uint8_t ext_boost_duration[6],ext_boost_strength[6];
    uint16_t battery_capacity_mah,battery_capacity_estimated_mah,r_batt_mohm;
    uint8_t limp_soc_limit,limp_soc_limit_stage2;
    uint16_t bank_store_magic,torque_cal_magic,wheel_diameter_magic,soc_full_magic;
    uint8_t wheel_diameter_code[2];
    uint16_t soc_full_pack_10mv,tuning_store_magic,assist_levels_magic;
} MotorParams_t;
extern uint16_t k;
void write_virtual_eeprom(void);
#endif
