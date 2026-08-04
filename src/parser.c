/*
 * parser.c
 *
 *  Created on: 30.11.2025
 *      Author: stancecoke
 */

#include "main.h"
#include "CAN_Display.h"

uint16_t l=0;

/*
 * Old/partially erased EEPROM records can still pass the historical hall-angle
 * sentinel and leave the shared motor limits at 0xFF/0xFFFF. In particular,
 * voltage_min then sits above the live battery voltage and all ride engines,
 * including Walk Assist, are cut by the common undervoltage limiter.
 *
 * Repair only the legacy/core fields. Appended profile banks, tuning and torque
 * calibration records remain untouched.
 */
static uint8_t repair_motor_params(MotorParams_t* MP){
	uint8_t repaired=0;
	//FW-030/dev: raised so the fixed 700 phase ceiling (PH_CURRENT_MAX) is not clamped on parse.
	uint16_t phase_current_sanity_max=(uint16_t)(80000.0f/CAL_I);

	if(MP->system_voltage<20 || MP->system_voltage>90){
		MP->system_voltage=SYSTEM_VOLTAGE;
		repaired=1;
	}
	if(MP->max_voltage<20 || MP->max_voltage>90){
		MP->max_voltage=MAX_VOLTAGE;
		repaired=1;
	}
	if(MP->battery_current_max<1000 || MP->battery_current_max>40000){
		MP->battery_current_max=BATTERYCURRENT_MAX;
		repaired=1;
	}
	if(MP->phase_current_max==0 || MP->phase_current_max>phase_current_sanity_max){
		MP->phase_current_max=PH_CURRENT_MAX;
		repaired=1;
	}
	{
		uint16_t max_cutoff_raw=(uint16_t)(((uint16_t)MP->max_voltage*1000U)/CAL_BAT_V);
		if(MP->voltage_min<=0 || (uint16_t)MP->voltage_min>=max_cutoff_raw){
			MP->voltage_min=VOLTAGE_MIN;
			repaired=1;
		}
	}
	if(MP->speedLimitx100==0 || MP->speedLimitx100>6000){
		MP->speedLimitx100=SPEEDLIMIT;
		repaired=1;
	}
	if(MP->wheel_cirumference==0 || MP->wheel_cirumference>4000){
		MP->wheel_cirumference=WHEEL_CIRCUMFERENCE;
		repaired=1;
	}
	if(MP->TS_coeff==0 || MP->TS_coeff==0xFFFF){
		MP->TS_coeff=TS_COEF;
		repaired=1;
	}
	if(MP->gear_ratio==0 || MP->gear_ratio>200){
		MP->gear_ratio=GEAR_RATIO;
		repaired=1;
	}
	if(MP->throttle_offset>4095 || MP->throttle_max>4095 ||
	   MP->throttle_offset>=MP->throttle_max){
		MP->throttle_offset=THROTTLE_OFFSET;
		MP->throttle_max=THROTTLE_MAX;
		repaired=1;
	}
	if(MP->Cadence_exponent>100){
		MP->Cadence_exponent=10;
		repaired=1;
	}
	if(MP->pulses_per_revolution==0 || MP->pulses_per_revolution>100){
		MP->pulses_per_revolution=PULSES_PER_REVOLUTION;
		repaired=1;
	}
	if(MP->PAS_timeout==0 || MP->PAS_timeout>60000){
		MP->PAS_timeout=PAS_TIMEOUT;
		repaired=1;
	}
	if(MP->ramp_end==0 || MP->ramp_end>20000){
		MP->ramp_end=RAMP_END;
		repaired=1;
	}
	if(MP->walk_assist_speed<WALK_ASSIST_RPM_MIN || MP->walk_assist_speed>WALK_ASSIST_RPM_MAX){
		MP->walk_assist_speed=WALK_ASSIST_RPM_DEFAULT;
		repaired=1;
	}
	if(MP->walk_assist_current==0 || MP->walk_assist_current>100){
		MP->walk_assist_current=WALK_ASSIST_CURRENT_DEFAULT;
		repaired=1;
	}
	if(MP->reverse!=-1 && MP->reverse!=1){
		MP->reverse=REVERSE;
		repaired=1;
	}
	if(MP->legalflag!=0 && MP->legalflag!=1){
		MP->legalflag=LEGALFLAG;
		repaired=1;
	}
	if(MP->battery_capacity_mah<1000 || MP->battery_capacity_mah>60000){
		MP->battery_capacity_mah=BATTERY_CAPACITY_MAH;
		repaired=1;
	}
	if(MP->battery_capacity_estimated_mah<1000 ||
	   MP->battery_capacity_estimated_mah>60000){
		MP->battery_capacity_estimated_mah=MP->battery_capacity_mah;
		repaired=1;
	}
	if(MP->r_batt_mohm==0 || MP->r_batt_mohm>1000){
		MP->r_batt_mohm=R_BATT_MOHM;
		repaired=1;
	}
	if(MP->limp_soc_limit>100 && MP->limp_soc_limit!=LIMP_DISABLED){
		MP->limp_soc_limit=LIMP_DISABLED;
		repaired=1;
	}
	if(MP->limp_soc_limit_stage2>100 &&
	   MP->limp_soc_limit_stage2!=LIMP_DISABLED){
		MP->limp_soc_limit_stage2=LIMP_DISABLED;
		repaired=1;
	}

	for(k=1;k<6;k++){
		if(MP->assist_settings[k][0]>100){
			MP->assist_settings[k][0]=(uint8_t)(k*20U);
			repaired=1;
		}
		if(MP->assist_settings[k][1]>100){
			MP->assist_settings[k][1]=100;
			repaired=1;
		}
		if(MP->assist_settings[k][2]<1 || MP->assist_settings[k][2]>7){
			MP->assist_settings[k][2]=TQFILTER;
			repaired=1;
		}
		if(MP->TQO_threshold[k]==0 ||
		   MP->TQO_threshold[k]>=TQ_FULL_SCALE_MV){
			MP->TQO_threshold[k]=TQ_PRESSURE_FLOOR_START_MV;
			repaired=1;
		}
		for(l=0;l<6;l++){
			if(MP->assist_profile[k-1][l]>100){
				MP->assist_profile[k-1][l]=(uint8_t)(k*20U);
				repaired=1;
			}
		}
	}
	MP->assist_settings[0][0]=0;
	MP->assist_settings[0][1]=0;
	MP->assist_settings[0][2]=0;

	return repaired;
}

void parse_DPparams(MotorParams_t* MP){
	MP->system_voltage = Para1[0];
	MP->battery_current_max=Para1[1]*1000;
	MP->max_voltage = Para1[2];
	MP->phase_current_max=Para1[9]*1000/CAL_I; //uses field Max Current on Low Charge
	// gear_ratio is intentionally NOT taken from Para1[19] here: it must match the
	// physical motor (pole pairs x gearbox ratio) and every speed/cadence/Walk Assist
	// RPM calculation depends on it. A wrong value sent over CAN (accidentally or by a
	// tool that does not know this motor) would silently break all of those. Stays a
	// firmware-side constant (GEAR_RATIO in config.h) for now; a future motor with a
	// different ratio can still get its own build.
	MP->MagicNumber=Para1[24]+(Para1[25]<<8);
	MP->throttle_offset=(Para1[34]<<12)/33; //map 3.3V to 12 bit ADC resolution
	MP->throttle_max=(Para1[35]<<12)/33; //map 3.3V to 12 bit ADC resolution
	MP->voltage_min=(Para1[3]+(Para1[4]<<8))/CAL_BAT_V;
	MP->Cadence_exponent=Para1[12];
	MP->legalflag=Para1[14];
	// reverse (motor direction) is intentionally NOT taken from Para1[18] here, for the
	// same reason as gear_ratio above: a wrong value over CAN makes the motor fight
	// itself or spin the wrong way. Stays a firmware-side constant (REVERSE in
	// config.h) — change it there and rebuild if a specific motor's wiring needs it.
	MP->pulses_per_revolution=Para1[20];
	MP->decay_base=Para1[21];
	MP->Override_Duration=Para1[37]*40;
	MP->PAS_timeout= Para1[38]*400; //in Zehntelsekunden, use field Current Loading Time (Ramp Up)
	MP->ramp_end = Para1[39] ? 11250/Para1[39] : RAMP_END; //use field Current Shedding Time (Ramp Down), calculate timer tics from theshold cadence
	MP->walk_assist_speed = Para1[60]+(Para1[61]<<8);
	if (MP->walk_assist_speed < WALK_ASSIST_RPM_MIN ||
		MP->walk_assist_speed > WALK_ASSIST_RPM_MAX) {
		MP->walk_assist_speed = WALK_ASSIST_RPM_DEFAULT;
	}
	MP->walk_assist_current = Para1[36];
	if (MP->walk_assist_current == 0 || MP->walk_assist_current > 100) MP->walk_assist_current = WALK_ASSIST_CURRENT_DEFAULT; // fallback: start boost fits under the phase ceiling

	// Battery capacity (Canable "Expected Battery Capacity", Para1[7..8], mAh)
	MP->battery_capacity_mah = Para1[7] + (Para1[8]<<8);
	if (MP->battery_capacity_mah == 0 || MP->battery_capacity_mah == 0xFFFF)
		MP->battery_capacity_mah = BATTERY_CAPACITY_MAH; // fallback when not set in Canable
	// Limp mode SoC thresholds (Canable Para1[10] / Para1[11], 0xFF = disabled)
	MP->limp_soc_limit        = Para1[10];
	MP->limp_soc_limit_stage2 = Para1[11];

	memcpy(&MP->assist_profile[0][0],&Para2[0],30);
	memcpy(&MP->ext_boost_duration[0]+1,&Para2[0]+31,5);
	memcpy(&MP->ext_boost_strength[0]+1,&Para2[0]+37,5);

	for (k=0; k < 4; k++){
		MP->assist_settings[k+1][0]=Para1[k*2+41]; //current limit (%)
		MP->assist_settings[k+1][1]=Para1[k*2+50]; //speed limit (%)
		MP->assist_settings[k+1][2]=Para0[k*2+2];  //ride mode (Acceleraton in Canable Tool)
	}
	MP->assist_settings[5][0]=Para1[48];
	MP->assist_settings[5][1]=Para1[57];
	MP->assist_settings[5][2]=Para0[9];

	MP->assist_settings[0][0]=0;
	MP->assist_settings[0][1]=0;
	MP->assist_settings[0][2]=0;
	MP->ext_boost_strength[0]=0;
	MP->ext_boost_duration[0]=0;

	//Torque override Threshold
	for (k=0; k < 4; k++){
		MP->TQO_threshold[k+1]=Para0[k*4+12]+(Para0[k*4+13]<<8);  // use field Assist ratio
	}
	MP->TQO_threshold[5]=Para0[26]+(Para0[27]<<8);
	MP->TQO_threshold[0]=3299;
	repair_motor_params(MP);
}


void parse_MOparams(MotorParams_t* MP){
	uint8_t repaired=repair_motor_params(MP);
	Para1[0] = MP->system_voltage;
	Para1[1] = MP->battery_current_max/1000;
	Para1[2] = MP->max_voltage;
	Para1[3] = (MP->voltage_min*CAL_BAT_V)&0xFF;
	Para1[4] = ((MP->voltage_min*CAL_BAT_V)>>8)&0xFF;
	Para1[9]= (MP->phase_current_max*CAL_I/1000);
	Para1[12]= MP->Cadence_exponent;
	Para1[14]= MP->legalflag;
	if (MP->reverse==-1)Para1[18]=0;
	else Para1[18]=1;
	Para1[19]= MP->gear_ratio;
	Para1[20]= MP->pulses_per_revolution;
	Para1[21]= MP->decay_base;
	Para1[24]= (MP->MagicNumber)&0xFF;
	Para1[25]= ((MP->MagicNumber)>>8)&0xFF;
	Para1[34]= ((MP->throttle_offset*33)>>12)+1; //map 3.3V to 12 bit ADC resolution
	Para1[35]= ((MP->throttle_max*33)>>12)+1; //map 3.3V to 12 bit ADC resolution
	Para1[37]= MP->Override_Duration/40;// used for override duration
	Para1[38]= MP->PAS_timeout*10/4000; //in Zehntelsekunden, use field Current Loading Time (Ramp Up)
	Para1[39]= 11250/MP->ramp_end; //use field Current Shedding Time (Ramp Down), calculate threshold cadence from timer tics
	Para1[36]= MP->walk_assist_current;
	Para1[60]= MP->walk_assist_speed&0xFF;
	Para1[61]= (MP->walk_assist_speed>>8)&0xFF;
	// Battery capacity + limp mode (echo back so Canable shows current values)
	Para1[7] = MP->battery_capacity_mah & 0xFF;
	Para1[8] = (MP->battery_capacity_mah>>8) & 0xFF;
	Para1[10]= MP->limp_soc_limit;
	Para1[11]= MP->limp_soc_limit_stage2;
	memcpy(&Para2[0],&MP->assist_profile[0][0],30);
	memcpy(&Para2[0]+31,&MP->ext_boost_duration[0]+1,5);
	memcpy(&Para2[0]+37,&MP->ext_boost_strength[0]+1,5);
	for (k=0; k < 4; k++){
		Para1[k*2+41]= MP->assist_settings[k+1][0]; //current limit (%)
		Para1[k*2+50]= MP->assist_settings[k+1][1]; //speed limit (%)
		Para0[k*2+2]= MP->assist_settings[k+1][2];  //ride mode (Acceleraton in Canable Tool)
	}
	Para1[48]= MP->assist_settings[5][0];
	Para1[57]= MP->assist_settings[5][1];
	Para0[9]= MP->assist_settings[5][2];

	//Torque override Threshold
	for (k=0; k < 4; k++){
		Para0[k*4+12]=MP->TQO_threshold[k+1]&0xFF;
		Para0[k*4+13]=MP->TQO_threshold[k+1]>>8;  // use field Assist ratio
	}


	Para0[26]= MP->TQO_threshold[5]&0xFF;
	Para0[27]= MP->TQO_threshold[5]>>8;

	update_checksum();
	if(repaired)write_virtual_eeprom();
}

void InitEEPROM(MotorParams_t* MP){
	MP->TS_coeff=TS_COEF;
	MP->MagicNumber=202;
	MP->battery_current_max=BATTERYCURRENT_MAX;
	MP->gear_ratio=GEAR_RATIO;
	MP->throttle_offset=THROTTLE_OFFSET; //map 3.3V to 12 bit ADC resolution
	MP->throttle_max=THROTTLE_MAX; //map 3.3V to 12 bit ADC resolution
	MP->torque_full_scale_native=0; //0 = not calibrated -> torque_input uses its default
	MP->active_profile_bank=0; //FW-005: boot into Power bank
	MP->bank_store_magic=0; //FW-006: no stored banks -> compiled-in defaults
	MP->torque_cal_magic=0; //FW-013: no user calibration -> default span 1620
	//FW-076: a factory reset gets a real wheel code, not a blank one — the app would
	//otherwise show "unknown wheel" on a controller that has simply never been written to.
	MP->wheel_diameter_magic=WHEEL_DIAMETER_MAGIC;
	MP->wheel_diameter_code[0]=WHEEL_DIAMETER_CODE_0;
	MP->wheel_diameter_code[1]=WHEEL_DIAMETER_CODE_1;
	MP->soc_full_magic=0; //FW-018: no configured full-charge voltage -> boot 100% anchor inactive
	MP->soc_full_pack_10mv=0;
	MP->tuning_store_magic=0; //FW-010: no stored tuning -> compiled-in defaults
	MP->reverse=REVERSE;
	MP->Cadence_exponent=10;
	MP->pulses_per_revolution=PULSES_PER_REVOLUTION;
	MP->phase_current_max = PH_CURRENT_MAX;
	MP->speedLimitx100 = SPEEDLIMIT;
	MP->wheel_cirumference = WHEEL_CIRCUMFERENCE;
	MP->voltage_min=VOLTAGE_MIN;
	MP->legalflag = LEGALFLAG;
	MP->Override_Duration=4000;
	MP->PAS_timeout = PAS_TIMEOUT;
	MP->ramp_end = RAMP_END;
	MP->walk_assist_speed = WALK_ASSIST_RPM_DEFAULT;
	MP->walk_assist_current = WALK_ASSIST_CURRENT_DEFAULT;
	MP->system_voltage = SYSTEM_VOLTAGE;
	MP->max_voltage = MAX_VOLTAGE;
	MP->decay_base =255;
	MP->battery_capacity_mah = BATTERY_CAPACITY_MAH;
	MP->battery_capacity_estimated_mah = BATTERY_CAPACITY_MAH;
	MP->r_batt_mohm = R_BATT_MOHM;
	MP->limp_soc_limit = LIMP_DISABLED;
	MP->limp_soc_limit_stage2 = LIMP_DISABLED;
	for (k=0; k < 5; k++){
		for (l=0; l < 6; l++){
			MP->assist_profile[k][l]=(k+1)*20;
		}
	}
	for (l=0; l < 6; l++){
		MP->ext_boost_duration[l]=l*20;
		MP->ext_boost_strength[l]=l*20-5;
	}

	for (k=0; k < 4; k++){
		MP->assist_settings[k+1][0]=(k+1)*20; //current limit: 20/40/60/80%
		MP->assist_settings[k+1][1]=100; //speed limit (%)
		MP->assist_settings[k+1][2]=TQFILTER;  //ride mode (Acceleraton in Canable Tool)
	}
	MP->assist_settings[5][0]=100; //level 5 current limit
	MP->assist_settings[5][1]=100;
	MP->assist_settings[5][2]=TQFILTER;

	MP->assist_settings[0][0]=0;
	MP->assist_settings[0][1]=0;
	MP->assist_settings[0][2]=0;

	for (k=0; k < 5; k++){
		MP->TQO_threshold[k+1]=TQ_PRESSURE_FLOOR_START_MV;
	}
	MP->TQO_threshold[0]=TQ_PRESSURE_FLOOR_START_MV;

	write_virtual_eeprom();
}
