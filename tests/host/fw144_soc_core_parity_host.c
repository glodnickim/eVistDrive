#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "soc_core.h"

static uint32_t rng_state = 0x1445C0C5U;
static uint32_t rng_u32(void)
{
	uint32_t x = rng_state;
	x ^= x << 13; x ^= x >> 17; x ^= x << 5;
	rng_state = x;
	return x;
}
static float frand(float lo, float hi)
{
	return lo + (hi - lo) * (float)(rng_u32() & 0x00FFFFFFU) / 16777215.0f;
}

static int8_t ref_ocv(uint16_t voltage, uint8_t cells)
{
	float voltages[] = {2.799f,2.968f,3.086f,3.247f,3.450f,3.569f,3.681f,3.774f,3.853f,3.946f,3.989f,4.070f};
	float socs[] = {0,5,10,20,30,40,50,60,70,80,90,100};
	int n = (int)(sizeof(voltages)/sizeof(voltages[0]));
	float cv = (float)voltage / ((float)cells * 1000.0f);
	if (cv <= voltages[0]) return (int8_t)socs[0];
	if (cv >= voltages[n-1]) return (int8_t)socs[n-1];
	for (int i=0;i<n-1;i++) if (cv < voltages[i+1]) {
		float slope=(socs[i+1]-socs[i])/(voltages[i+1]-voltages[i]);
		return (int8_t)(socs[i]+slope*(cv-voltages[i]));
	}
	return (int8_t)socs[n-1];
}

static float ref_limp(float soc, uint8_t lim, uint8_t s2)
{
	if(lim==LIMP_DISABLED || lim==0) return 1.0f;
	float fl=(float)LIMP_FLOOR_PCT/100.0f;
	if(soc<0) soc=0;
	if(soc>=lim) return 1.0f;
	float f;
	if(s2!=LIMP_DISABLED && s2>0 && s2<lim){
		float p2=(float)LIMP_STAGE2_PCT/100.0f;
		if(soc>s2) f=p2+(1.0f-p2)*(soc-(float)s2)/(float)(lim-s2);
		else       f=fl+(p2-fl)*soc/(float)s2;
	}else f=fl+(1.0f-fl)*soc/(float)lim;
	if(f<fl) f=fl;
	if(f>1.0f) f=1.0f;
	return f;
}

static void ref_step(soc_core_state_t *s, const soc_core_input_t *in)
{
	if(!s->boot_full_done){
		if(in->soc_full_magic==SOC_FULL_MAGIC){
			if(in->voltage_mv<s->boot_vmin_mv) s->boot_vmin_mv=(uint16_t)in->voltage_mv;
			if(in->voltage_mv>s->boot_vmax_mv) s->boot_vmax_mv=(uint16_t)in->voltage_mv;
			if(++s->boot_settle_s>=SOC_FULL_BOOT_SETTLE_S){
				s->boot_full_done=1;
				if((uint16_t)(s->boot_vmax_mv-s->boot_vmin_mv)<=SOC_FULL_BOOT_STABLE_MV &&
				   in->voltage_mv>=(uint32_t)in->soc_full_pack_10mv*10U){
					s->remaining_mah=(float)in->capacity_estimated_mah;
					s->soc_real=100.0f; s->soc_display=100.0f;
					s->full_anchor=1; s->anchor_start_mah=s->remaining_mah;
				}
			}
		}else s->boot_full_done=1;
	}
	float dmah=in->delta_mah;
	s->remaining_mah-=dmah;
	if(s->remaining_mah>(float)in->capacity_estimated_mah) s->remaining_mah=(float)in->capacity_estimated_mah;
	if(s->remaining_mah<0) s->remaining_mah=0;
	s->soc_real=s->remaining_mah/(float)in->capacity_estimated_mah*100.0f;
	uint8_t cells=(uint8_t)((float)in->system_voltage/3.6f);
	float ia=(float)in->battery_current_ma/1000.0f;
	uint16_t ucomp=(uint16_t)((float)in->voltage_mv+ia*(float)in->r_batt_mohm);
	s->soc_voltage=ref_ocv(ucomp,cells);
	if(in->battery_current_ma<I_REST_MA && in->battery_current_ma>-I_REST_MA){
		if(s->rest_seconds<65000) s->rest_seconds++;
		if(s->rest_seconds>=REST_TIME_S){
			s->soc_real+=OCV_CORR_GAIN*((float)s->soc_voltage-s->soc_real);
			s->remaining_mah=s->soc_real/100.0f*(float)in->capacity_estimated_mah;
		}
	}else s->rest_seconds=0;
	/* independent reference: charge drain bypasses the display correction clamp */
	float after_charge=s->soc_display - in->delta_mah/(float)in->capacity_estimated_mah*100.0f;
	float diff=s->soc_real-after_charge;
	float step=SOC_DISP_GAIN*diff;
	float max_step=SOC_DISP_MAX_STEP/60.0f;
	if(s->soc_real<10.0f) step=diff;
	if(step>max_step)step=max_step;
	if(step<-max_step)step=-max_step;
	s->soc_display=after_charge+step;
	if(s->soc_display<0)s->soc_display=0;
	if(s->soc_display>100)s->soc_display=100;
	if(s->full_anchor){
		if((s->anchor_start_mah-s->remaining_mah)<SOC_FULL_RELEASE_FRAC*(float)in->capacity_estimated_mah)
			s->soc_display=100.0f;
		else { s->full_anchor=0; s->soc_display=s->soc_real; }
	}
}

static int samef(float a, float b){ return fabsf(a-b) <= 1e-5f; }
static int state_equal(const soc_core_state_t *a,const soc_core_state_t *b)
{
	return samef(a->remaining_mah,b->remaining_mah) && samef(a->soc_real,b->soc_real) &&
		samef(a->soc_display,b->soc_display) && a->soc_voltage==b->soc_voltage &&
		a->rest_seconds==b->rest_seconds && a->full_anchor==b->full_anchor &&
		a->boot_full_done==b->boot_full_done && a->boot_settle_s==b->boot_settle_s &&
		a->boot_vmin_mv==b->boot_vmin_mv && a->boot_vmax_mv==b->boot_vmax_mv &&
		samef(a->anchor_start_mah,b->anchor_start_mah);
}

int main(void)
{
	for(uint16_t mv=30000; mv<=46000; mv+=17){
		if(soc_core_calculate_ocv(mv,11)!=ref_ocv(mv,11)){
			fprintf(stderr,"OCV parity fail at %u mV\n",mv); return 1;
		}
	}
	for(int s=-5;s<=105;s++){
		float a=soc_core_limp_factor((float)s,20,5), b=ref_limp((float)s,20,5);
		if(!samef(a,b)){ fprintf(stderr,"limp parity fail at %d\n",s); return 1; }
	}

	for(unsigned case_i=0; case_i<200; case_i++){
		uint16_t cap=(uint16_t)(1000U+(rng_u32()%59000U));
		soc_core_state_t a={0},b={0};
		a.remaining_mah=b.remaining_mah=frand(0.0f,(float)cap);
		a.soc_real=b.soc_real=a.remaining_mah/(float)cap*100.0f;
		a.soc_display=b.soc_display=frand(0.0f,100.0f);
		a.soc_voltage=b.soc_voltage=(int8_t)(rng_u32()%101U);
		a.rest_seconds=b.rest_seconds=rng_u32()%100U;
		a.full_anchor=b.full_anchor=(uint8_t)(rng_u32()&1U);
		a.boot_full_done=b.boot_full_done=(uint8_t)(rng_u32()&1U);
		a.boot_settle_s=b.boot_settle_s=(uint8_t)(rng_u32()%12U);
		a.boot_vmin_mv=b.boot_vmin_mv=(uint16_t)(44000U+rng_u32()%1000U);
		a.boot_vmax_mv=b.boot_vmax_mv=(uint16_t)(a.boot_vmin_mv+rng_u32()%180U);
		a.anchor_start_mah=b.anchor_start_mah=frand(a.remaining_mah,(float)cap);
		for(unsigned step=0;step<100;step++){
			soc_core_input_t in={
				.voltage_mv=(uint32_t)(30000U+rng_u32()%16000U),
				.battery_current_ma=(int32_t)(-1000+(int32_t)(rng_u32()%17000U)),
				.delta_mah=frand(-0.3f,5.0f),
				.capacity_estimated_mah=cap,
				.r_batt_mohm=(uint16_t)(20U+rng_u32()%180U),
				.system_voltage=40,
				.soc_full_magic=(rng_u32()&1U)?SOC_FULL_MAGIC:0U,
				.soc_full_pack_10mv=4598U
			};
			ref_step(&a,&in); soc_core_step_1hz(&b,&in);
			if(!state_equal(&a,&b)){
				fprintf(stderr,"SOC core parity fail case=%u step=%u\n",case_i,step); return 1;
			}
		}
	}
	/* Regression: HMI must not overstate SOC after sustained 12 A discharge. */
	{
		const float cap=(float)BATTERY_CAPACITY_MAH;
		if(BATTERY_CAPACITY_MAH != 17400 || BATTERY_CELLS_SERIES != 11U || BATTERY_CELLS_PARALLEL != 3U){
			fprintf(stderr,"LG M58T 11S3P default capacity mismatch\n"); return 1;
		}
		soc_core_state_t s={0};
		s.remaining_mah=cap; s.soc_real=100.0f; s.soc_display=100.0f; s.boot_full_done=1;
		soc_core_input_t in={0};
		in.voltage_mv=42500U; in.battery_current_ma=12000; in.delta_mah=12000.0f/3600.0f;
		in.capacity_estimated_mah=BATTERY_CAPACITY_MAH; in.r_batt_mohm=80; in.system_voltage=40;
		for(int i=0;i<3600;i++) soc_core_step_1hz(&s,&in);
		float expected=100.0f - 12000.0f/cap*100.0f;
		if(fabsf(s.soc_real-expected)>0.1f || fabsf(s.soc_display-expected)>0.1f){
			fprintf(stderr,"HMI SOC discharge drift: real=%.3f display=%.3f expected=%.3f\n",
				s.soc_real,s.soc_display,expected); return 1;
		}
	}
	/* A high-confidence full 11S pack anchors immediately; a lower/loaded one waits. */
	if(!soc_core_can_mark_full_at_boot(46200U,0, SOC_FULL_MAGIC,4598U) ||
	   soc_core_can_mark_full_at_boot(45500U,0,SOC_FULL_MAGIC,4598U) ||
	   soc_core_can_mark_full_at_boot(46200U,1500,SOC_FULL_MAGIC,4598U) ||
	   soc_core_can_mark_full_at_boot(46200U,0,0U,4598U)){
		fprintf(stderr,"early boot full-charge qualification failed\n"); return 1;
	}
	printf("FW144 SOC core parity + LG M58T 11S3P regressions: PASS\n");
	return 0;
}
