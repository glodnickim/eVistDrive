/*
 * TASK-EVD-TQ-06-G2 / I3: bumpless QZERO exit when the rider resumes pedalling while rolling.
 *
 * Problem (P3 analysis): after a QUIET release with the rotor still spinning, QZERO fades both
 * current-PI integrals to 0 (BLEND) and holds them at 0 (HOLD). On re-engage (iq_ref > 0) it used
 * to leave the integral where it was - 0 from HOLD - so the drive started from the braking state:
 * Iq stayed negative for several ms and then caught up faster than the reference ramp (the chain
 * jerk). The exit now hands the regulators the integral that nulls the current at the present
 * speed (the HANDBACK target law), applied on the exit tick.
 *
 * Section A drives the REAL quiet_zero.c state machine only (no plant) and pins its behaviour:
 *   A1 exit values and conditions, A2 fall-backs, A3 a release that is never re-engaged is
 *   byte-for-byte what it was before this change (FNV-1a pin of every output over a full stop).
 * Section B runs the production FOC.c + foc_current_loop.c + pwm_geometry.c + quiet_zero.c against
 * the dq plant of sim/foc_electrical_sil.c with the P3 probe wiring (a re-creation of the
 * src/main.c QZERO wiring). THE PLANT IS A SIL PLACEHOLDER (R=0.060, L=80uH, flux=0.015, not the
 * M820 motor): observation grade - it shows direction and rough size, not a bike result.
 */
#include "main.h"
#include "FOC.h"
#include "foc_current_loop.h"
#include "pwm_geometry.h"
#include "quiet_zero.h"
#include "check.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Pinned bounds (see the report for how they were chosen). */
#define A3_PIN      0xA82BFFC6u   /* FNV-1a of the no-re-engage stop, measured on d23278a */
#define B_CROSS_MS  2.0           /* Iq must be >= 0 within this long after the first iq_ref > 0 */
#define B_NEG_MS    2.0           /* total time with Iq below NEG_BOUND */
#define B_OVER      110.0         /* max (Iq - reference) in the 20 ms after engage, counts (observed <= 100) */
#define NEG_BOUND   (-3.0)

/* ------------------------------------------------------------------------------------------ */
/* Section A: state machine only                                                              */
/* ------------------------------------------------------------------------------------------ */

static quiet_zero_input_t in_make(int32_t ref, int32_t erps, bool fresh, float iqi, float idi)
{
	quiet_zero_input_t in;
	memset(&in, 0, sizeof(in));
	in.iq_ref = ref;
	in.zero_policy_quiet = true;
	in.abort_current = 350;
	in.rotor_erps = erps;
	in.speed_fresh = fresh;
	in.min_brake_erps = 10;
	in.iq_integral = iqi;
	in.id_integral = idi;
	return in;
}

/* Release from a pull (integrals 900 / -20 at 93 erps), then `ticks` zero-reference ticks. */
static void a_release(quiet_zero_t *qz, int ticks, int32_t erps_hold)
{
	quiet_zero_action_t out;
	quiet_zero_input_t in = in_make(100, 93, true, 900.0f, -20.0f);
	quiet_zero_reset(qz);
	in.zero_policy_quiet = false;               /* prev_iq_ref := 100 */
	quiet_zero_tick(qz, &in, &out);
	in = in_make(0, 93, true, 900.0f, -20.0f);  /* entry edge */
	quiet_zero_tick(qz, &in, &out);
	for (int i = 0; i < ticks; i++) {
		in = in_make(0, erps_hold, true, 0.0f, 0.0f);
		quiet_zero_tick(qz, &in, &out);
	}
}

static uint32_t fnv(uint32_t h, uint32_t v)
{
	for (int i = 0; i < 4; i++) { h ^= (v >> (8 * i)) & 0xFFU; h *= 16777619U; }
	return h;
}
static uint32_t fbits(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }

static void section_a(void)
{
	quiet_zero_t qz;
	quiet_zero_action_t out;
	quiet_zero_input_t in;

	/* A1: exit from HOLD at the entry speed hands back the speed-scaled entry integral, Id = 0. */
	a_release(&qz, 500, 93);
	CHECK(qz.state == (uint32_t)QZERO_HOLD, "A1 setup: HOLD");
	in = in_make(3, 93, true, 0.0f, 0.0f);
	quiet_zero_tick(&qz, &in, &out);
	CHECK(out.exited && out.clear_aw_edge && out.state == (uint32_t)QZERO_INACTIVE,
		"A1: exit edge unchanged (exited, clear_aw_edge, INACTIVE)");
	CHECK(out.apply_integral && !out.freeze_aw, "A1: integral applied on the exit tick, AW not frozen");
	CHECK(out.iq_integral == 900.0f && out.id_integral == 0.0f,
		"A1: Iq integral = entry value at entry speed, Id integral = 0 (HANDBACK law)");

	/* scaled by present speed (60 of 93 erps) */
	a_release(&qz, 500, 60);
	in = in_make(3, 60, true, 0.0f, 0.0f);
	quiet_zero_tick(&qz, &in, &out);
	CHECK(out.apply_integral && fabsf(out.iq_integral - 900.0f * 60.0f / 93.0f) < 1e-3f,
		"A1: Iq integral scales with erps_now / erps_entry");

	/* a speed above the entry speed is clamped, never amplified */
	a_release(&qz, 500, 93);
	in = in_make(3, 150, true, 0.0f, 0.0f);
	quiet_zero_tick(&qz, &in, &out);
	CHECK(out.apply_integral && out.iq_integral == 900.0f, "A1: erps_now above erps_entry is clamped");

	/* from BLEND (mid-fade) the same law applies, not the partly faded value */
	a_release(&qz, 20, 93);
	CHECK(qz.state == (uint32_t)QZERO_BLEND, "A1 setup: BLEND");
	in = in_make(3, 93, true, 800.0f, -18.0f);
	quiet_zero_tick(&qz, &in, &out);
	CHECK(out.exited && out.apply_integral && out.iq_integral == 900.0f && out.id_integral == 0.0f,
		"A1: exit from BLEND applies the matching integral");

	/* from HANDBACK (target not yet reached) */
	a_release(&qz, 500, 40);                   /* 40 < 50 % of 93 -> handback started */
	CHECK(qz.state == (uint32_t)QZERO_HANDBACK, "A1 setup: HANDBACK");
	in = in_make(3, 40, true, 10.0f, 0.0f);
	quiet_zero_tick(&qz, &in, &out);
	CHECK(out.exited && out.apply_integral && fabsf(out.iq_integral - 900.0f * 40.0f / 93.0f) < 1e-3f,
		"A1: exit from HANDBACK jumps to the target");

	/* A2: fallbacks keep the previous behaviour (nothing applied) */
	a_release(&qz, 500, 93);
	in = in_make(3, 93, false, 0.0f, 0.0f);
	quiet_zero_tick(&qz, &in, &out);
	CHECK(out.exited && out.clear_aw_edge && !out.apply_integral && !out.freeze_aw,
		"A2: speed not fresh -> old behaviour (exit, nothing applied)");
	a_release(&qz, 500, 93);
	in = in_make(3, 0, true, 0.0f, 0.0f);
	quiet_zero_tick(&qz, &in, &out);
	CHECK(out.exited && !out.apply_integral, "A2: zero rotor speed -> old behaviour");
	{
		quiet_zero_t q0;
		quiet_zero_reset(&q0);
		in = in_make(100, 0, true, 5.0f, 0.0f);
		in.zero_policy_quiet = false;
		in.min_brake_erps = 0;
		quiet_zero_tick(&q0, &in, &out);
		in = in_make(0, 0, true, 5.0f, 0.0f);
		in.min_brake_erps = 0;
		quiet_zero_tick(&q0, &in, &out);     /* entered with erps_entry == 0 */
		in = in_make(3, 7, true, 0.0f, 0.0f);
		in.min_brake_erps = 0;
		quiet_zero_tick(&q0, &in, &out);
		CHECK(q0.entries == 1U && out.exited && !out.apply_integral,
			"A2: erps_entry == 0 -> old behaviour");
	}
	/* INACTIVE with a positive reference: nothing happens (no spurious apply) */
	quiet_zero_reset(&qz);
	in = in_make(50, 93, true, 900.0f, 0.0f);
	quiet_zero_tick(&qz, &in, &out);
	CHECK(!out.exited && !out.apply_integral && !out.clear_aw_edge, "A2: ordinary drive is untouched");

	/* A3: release never re-engaged -> outputs identical to the pre-change state machine. */
	{
		uint32_t h = 2166136261U;
		float iqi = 900.0f, idi = -20.0f;   /* what the regulators hold; follows the applied value */
		quiet_zero_reset(&qz);
		in = in_make(100, 93, true, iqi, idi);
		in.zero_policy_quiet = false;
		quiet_zero_tick(&qz, &in, &out);
		for (int t = 0; t < 6400; t++) {   /* 400 ms, erps 93 -> 0 */
			int32_t e = 93 - (93 * t) / 6400;
			in = in_make(0, e, e > 2, iqi, idi);
			quiet_zero_tick(&qz, &in, &out);
			if (out.apply_integral) { iqi = out.iq_integral; idi = out.id_integral; }
			h = fnv(h, out.state);
			h = fnv(h, (uint32_t)out.apply_integral);
			h = fnv(h, fbits(out.iq_integral));
			h = fnv(h, fbits(out.id_integral));
			h = fnv(h, (uint32_t)out.freeze_aw | ((uint32_t)out.clear_aw_edge << 1) |
				((uint32_t)out.entered << 2) | ((uint32_t)out.exited << 3) |
				((uint32_t)out.aborted << 4) | ((uint32_t)out.low_speed_release << 5));
		}
		printf("  A3 stop-trajectory FNV-1a = 0x%08X\n", (unsigned)h);
		CHECK(h == A3_PIN, "A3: no re-engage -> every output of a full stop is byte-identical to d23278a");
	}
}

/* ------------------------------------------------------------------------------------------ */
/* Section B: production FOC + dq plant (SIL placeholder), P3 probe wiring                     */
/* ------------------------------------------------------------------------------------------ */
#define DT (1.0/16000.0)
#define A_PER_COUNT 0.095
#define VBUS 40.0
#define SQRT3 1.7320508075688772

typedef struct { double r,l_d,l_q,flux,id,iq,theta_e,erps; } pmsm_t;

MotorState_t MS;
MotorParams_t MP;
PI_control_t PI_iq,PI_id;
uint8_t ui_8_PWM_ON_Flag=1U;
uint8_t bridge_lifecycle=6U;
int32_t switchtime[3];
uint16_t pwm_applied[3];
static int32_t g_ref;
static int g_quiet;
static double g_erps_report;
static quiet_zero_t QZ;
static int pwm_polarity=-1;

void timer_channel_output_pulse_value_config(uint32_t t,uint16_t c,uint32_t v){(void)t;(void)c;(void)v;}
void timer_primary_output_config(uint32_t t,uint32_t e){(void)t;(void)e;}

static void pi_init(PI_control_t *p,int limit_i){
	memset(p,0,sizeof(*p));p->gain_p=1.5f;p->gain_i=.01f;p->limit_i=(int16_t)limit_i;
	p->limit_output=1920;p->max_step=15;p->shift=11;p->aw_inv_kp_q15=21845;
}

/* Re-creation of the QZERO wiring of src/main.c (the source-text wiring guards live in
 * qzero_quiet_zero_host.c T13): input, tick, apply_integral, AW clears, then the current loop. */
void runPIcontrol(void){
	PI_iq.recent_value=(int16_t)MS.i_q;
	PI_iq.setpoint=g_ref;
	quiet_zero_action_t qz; memset(&qz,0,sizeof(qz));
	quiet_zero_input_t in={
		.iq_ref=g_ref,.zero_policy_quiet=g_quiet?true:false,.iq_measured=MS.i_q,.id_measured=MS.i_d,
		.abort_current=350,.rotor_erps=(int32_t)g_erps_report,.speed_fresh=true,.min_brake_erps=10,
		.iq_integral=PI_iq.integral_part,.id_integral=PI_id.integral_part};
	quiet_zero_tick(&QZ,&in,&qz);
	if(qz.apply_integral){PI_iq.integral_part=qz.iq_integral;PI_id.integral_part=qz.id_integral;}
	if(qz.freeze_aw||qz.clear_aw_edge){PI_iq.aw_sat_error=0;PI_id.aw_sat_error=0;MS.u_q_sat_err=0;MS.u_d_sat_err=0;}
	foc_current_loop_result_t r;
	foc_current_loop_step(&MS,&PI_iq,&PI_id,qz.apply_integral?1U:0U,qz.iq_integral,qz.id_integral,&r);
}

static double wrap_pi(double a){while(a>=M_PI)a-=2*M_PI;while(a< -M_PI)a+=2*M_PI;return a;}
static q31_t angle_q31(double a){a=wrap_pi(a);double x=a/M_PI*2147483648.0;if(x>=2147483647.0)x=2147483647.0;if(x<-2147483648.0)x=-2147483648.0;return (q31_t)llround(x);}
static void dq_to_ab(double d,double q,double th,double *a,double *b){double c=cos(th),s=sin(th);*a=d*c-q*s;*b=d*s+q*c;}
static void ab_to_dq(double a,double b,double th,double *d,double *q){double c=cos(th),s=sin(th);*d=a*c+b*s;*q=-a*s+b*c;}
static void phase_counts(const pmsm_t *p,int16_t *ia,int16_t *ib){
	double a,b;dq_to_ab(p->id,p->iq,p->theta_e,&a,&b);double ib_a=(-a+SQRT3*b)*0.5;
	long ca=llround(a/A_PER_COUNT),cb=llround(ib_a/A_PER_COUNT);
	if(ca>32767)ca=32767;
	if(ca<-32768)ca=-32768;
	if(cb>32767)cb=32767;
	if(cb<-32768)cb=-32768;
	*ia=(int16_t)ca;*ib=(int16_t)cb;
}
static void pwm_to_ab(const uint16_t p[3],double *al,double *be){
	double da=(double)p[0]/3750.0,db=(double)p[1]/3750.0,dc=(double)p[2]/3750.0,mean=(da+db+dc)/3.0;
	double va=pwm_polarity*VBUS*(da-mean),vb=pwm_polarity*VBUS*(db-mean);*al=va;*be=(va+2.0*vb)/SQRT3;
}
static void plant_step(pmsm_t *p,double vd,double vq){
	const int sub=8;const double h=DT/sub;double we=2.0*M_PI*p->erps;
	for(int k=0;k<sub;k++){
		double did=(vd-p->r*p->id+we*p->l_q*p->iq)/p->l_d;
		double diq=(vq-p->r*p->iq-we*(p->l_d*p->id+p->flux))/p->l_q;
		p->id+=did*h;p->iq+=diq*h;p->theta_e=wrap_pi(p->theta_e+we*h);
	}
}
static void reset_all(pmsm_t *p,double theta,double erps){
	memset(&MS,0,sizeof(MS));memset(&MP,0,sizeof(MP));memset(switchtime,0,sizeof(switchtime));memset(pwm_applied,0,sizeof(pwm_applied));
	pi_init(&PI_iq,1920);pi_init(&PI_id,1800);MS.hall_angle_detect_flag=1;MS.i_d_setpoint=0;MP.com_mode=Hallsensor;
	memset(p,0,sizeof(*p));p->r=.060;p->l_d=80e-6;p->l_q=80e-6;p->flux=.015;p->theta_e=wrap_pi(theta);p->erps=erps;
	foc_current_feedback_reset(&MS);pwm_geometry_init();quiet_zero_reset(&QZ);g_erps_report=erps;
}
static void tick(pmsm_t *p,int32_t ref,int quiet){
	int16_t ia,ib;phase_counts(p,&ia,&ib);g_ref=ref;g_quiet=quiet;
	FOC_calculation(ia,ib,angle_q31(p->theta_e),(int16_t)ref,&MS,&MP);
	pwm_geometry_apply(switchtime,pwm_applied,3750);
	double a,b,vd,vq;pwm_to_ab(pwm_applied,&a,&b);ab_to_dq(a,b,p->theta_e,&vd,&vq);plant_step(p,vd,vq);
}
static void polarity(void){
	pmsm_t p;reset_all(&p,0,0);g_ref=100;int16_t ia=0,ib=0;FOC_calculation(ia,ib,angle_q31(0),100,&MS,&MP);
	pwm_geometry_apply(switchtime,pwm_applied,3750);double v[2];
	for(int k=0;k<2;k++){pwm_polarity=k?1:-1;double a,b,d,q;pwm_to_ab(pwm_applied,&a,&b);ab_to_dq(a,b,0,&d,&q);v[k]=q;}
	pwm_polarity=(MS.u_q>=0)?(v[0]>v[1]?-1:1):(v[0]<v[1]?-1:1);
}
/* 3 at the first tick, then +0.875 per 4 kHz tick (3.5 Iq/ms), cap 120. */
static int32_t ramp_ref(int n16){double v=3.0+0.875*(double)(n16/4);return (int32_t)(v>120.0?120.0:v);}

typedef struct {
	double iq_pre;          /* Iq at the engage tick (braking state), counts */
	double min_iq;          /* minimum Iq in the 20 ms after engage */
	double t_cross_ms;      /* first time Iq >= 0, -1 = never */
	double t_neg_ms;        /* total time with Iq < NEG_BOUND */
	double max_over;        /* max (Iq - ref) in the 20 ms, counts */
	double iq5, iq10;       /* Iq at 5 and 10 ms */
	double min_id, max_id;
	uint32_t aborted;       /* abort guard fired before the engage: QZERO already INACTIVE, out of scope */
} bump_t;

/* Rolling release at `erps`, QUIET hold of hold_ms, then engage. `decel` = fraction of the entry
 * speed the rotor has once held (1.0 = no slow-down). */
static bump_t scenario(double erps, double hold_ms, double decel)
{
	pmsm_t p; bump_t r; memset(&r,0,sizeof(r));
	reset_all(&p,0.3,erps);
	for(int i=0;i<1600;i++) tick(&p,100,0);
	for(int i=0;i<320;i++) tick(&p,100-(int32_t)(100.0*(i+1)/320.0),0);
	int hold=(int)(hold_ms*16.0);
	for(int i=0;i<hold;i++){
		if(i==2){p.erps=erps*decel;g_erps_report=erps*decel;}
		tick(&p,0,1);
	}
	r.iq_pre=p.iq/A_PER_COUNT;
	r.aborted=QZ.aborts;
	r.min_iq=1e9;r.t_cross_ms=-1;r.min_id=1e9;r.max_id=-1e9;
	int neg=0;
	for(int n=0;n<320;n++){
		int32_t ref=ramp_ref(n);tick(&p,ref,0);
		double iq=p.iq/A_PER_COUNT,id=p.id/A_PER_COUNT;
		if(iq<r.min_iq)r.min_iq=iq;
		if(id<r.min_id)r.min_id=id;
		if(id>r.max_id)r.max_id=id;
		if(r.t_cross_ms<0&&iq>=0.0)r.t_cross_ms=(n+1)/16.0;
		if(iq<NEG_BOUND)neg++;
		if(iq-ref>r.max_over)r.max_over=iq-ref;
		if(n+1==80)r.iq5=iq;
		if(n+1==160)r.iq10=iq;
	}
	r.t_neg_ms=neg/16.0;
	return r;
}

static void section_b(void)
{
	const double erpss[]={60.0,120.0};
	const double holds[]={2.0,6.0,12.0,40.0};
	const double decels[]={1.0,0.8};
	int aborted_cases=0;
	polarity();
	printf("  PWM polarity=%+d (SIL calibration). Plant = SIL placeholder, observation grade.\n",pwm_polarity);
	printf("  %5s %6s %5s | %8s %8s %7s %7s %8s %8s %8s %16s\n","erps","hold","decel","Iq_pre","minIq","tcross","tNeg","maxOver","Iq@5ms","Iq@10ms","Id[min,max]");
	for(int e=0;e<2;e++)for(int d=0;d<2;d++)for(int h=0;h<4;h++){
		bump_t r=scenario(erpss[e],holds[h],decels[d]);
		char label[200];
		printf("  %5.0f %4.0fms %5.2f | %8.1f %8.1f %5.2fms %5.2fms %8.1f %8.1f %8.1f [%+6.1f,%+6.1f]%s\n",
			erpss[e],holds[h],decels[d],r.iq_pre,r.min_iq,r.t_cross_ms,r.t_neg_ms,r.max_over,r.iq5,r.iq10,r.min_id,r.max_id,
			r.aborted?"  (abort guard fired in HOLD before engage - QZERO already INACTIVE, out of scope)":"");
		if(r.aborted){aborted_cases++;continue;}
		snprintf(label,sizeof(label),"B: erps %.0f hold %.0f ms decel %.2f: Iq crosses 0 within %.1f ms of re-engage",erpss[e],holds[h],decels[d],B_CROSS_MS);
		CHECK(r.t_cross_ms>=0.0&&r.t_cross_ms<=B_CROSS_MS,label);
		snprintf(label,sizeof(label),"B: erps %.0f hold %.0f ms decel %.2f: Iq below %.0f counts for at most %.1f ms",erpss[e],holds[h],decels[d],NEG_BOUND,B_NEG_MS);
		CHECK(r.t_neg_ms<=B_NEG_MS,label);
		snprintf(label,sizeof(label),"B: erps %.0f hold %.0f ms decel %.2f: overshoot above the reference <= %.0f counts",erpss[e],holds[h],decels[d],B_OVER);
		CHECK(r.max_over<=B_OVER,label);
	}
	/* Only the four erps-120 cases with a 12 ms / 40 ms hold abort in this placeholder plant; if that
	 * count changes, the matrix no longer tests what this file says it tests. */
	CHECK(aborted_cases==4,"B: exactly the four abort-guard cases are excluded from the bumpless bounds");
}

int main(void)
{
	puts("QZERO bumpless exit (TASK-EVD-TQ-06-G2 I3)");
	section_a();
	section_b();
	if (host_test_failures == 0) { puts("QZERO bumpless exit: ALL CHECKS PASSED"); return 0; }
	printf("QZERO bumpless exit: %d CHECK(S) FAILED\n", host_test_failures);
	return 1;
}
