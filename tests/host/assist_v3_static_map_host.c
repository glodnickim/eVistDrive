/*
 * ASSIST-V3 B-MAP: parity of g53_static_target() (ARCHITECTURE_V3.md section 5) against the
 * transcribed G53 D7EC chain, plus the read-only V3 accessors.
 *
 *  (a) G1-STATIC    steady state: real chain (g53_port_update, constant load and PAS cadence),
 *                   g53_static_target(D+170, cadence, level) == D+224 exactly. Levels 1..5
 *                   (S+ AUTO on HMI 4), cadences 10..130 rpm, loads light..heavy.
 *  (b) G1-STATIC-T  level changes while riding and S+ AUTO attacks: the pure function is driven
 *                   every 1 ms (elapsed 1 ms) with the chain's env sequence and its own ratio
 *                   state; target, ratio and limiter state compared at every D7EC (10 ms) tick.
 *  (c) no mutation  the chain image (every state array and phase counter) is byte-identical
 *                   before and after many calls of the pure function and the accessors.
 *
 * The chain source is included directly so (c) can snapshot its private state arrays; the
 * registry entry therefore lists every G53 module except g53_port_chain.c.
 */
#include "../../src/g53_port_chain.c"
#include "g53_port.h"
#include <stdio.h>
#include <stdlib.h>

static unsigned checks, failures;
#define CHECK(cond, ...) do { ++checks; if(!(cond)) { ++failures; \
    if(failures<=20) { printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); puts(""); } } } while(0)

static const uint8_t pas_cycle[4]={0,2,3,1};   /* native forward order (as m560_auto_splus_host) */
static const uint8_t hmi_slot[6]={0,2,4,6,8,9};

typedef struct {
    uint32_t phase;      /* accumulated cadence*96 per ms */
    uint32_t t;
} rider_t;

static void ride_tick(rider_t *r, uint8_t hmi, uint16_t load, uint16_t cadence_rpm,
                      g53_port_output_t *out)
{
    g53_port_input_t in={0};
    in.load_ctrl=load;
    in.assist_level=hmi;
    in.speed_x100=2000;
    in.elapsed_ticks=4;                      /* one G53 logical tick = 1 ms */
    in.phase_current_max=900;
    in.battery_limit_centiamp=1500;
    in.torque_sensor_valid=true;
    r->phase+=(uint32_t)cadence_rpm*96u;
    in.pas_ab=pas_cycle[(r->phase/60000u)%4u];
    g53_port_update(&in,out);
    ++r->t;
}

static g53_static_input_t static_in_from_trace(const g53_port_trace_t *tr, uint8_t slot)
{
    g53_static_input_t s;
    s.env=(uint16_t)tr->d7ec_envelope;
    s.cadence=(int16_t)tr->cadence;
    s.lut_cadence=(uint16_t)tr->d7ec_m50;
    s.speed_native=(uint16_t)tr->d7ec_speed;
    s.level=slot;
    return s;
}

static unsigned hold_nonzero;   /* D+222 seen non-zero anywhere (floor-inert claim) */
static unsigned armed_violations;

static void common_invariants(const g53_port_output_t *o)
{
    if(o->trace.d7ec_hold!=0) ++hold_nonzero;
    if(o->trace.rider_input_native>0 && !g53_port_eb74_armed()) ++armed_violations;
}

/* ---------------- (a) G1-STATIC ---------------- */
static void test_steady(void)
{
    static const uint16_t cadences[]={10,15,20,25,30,45,60,80,100,130};
    static const uint16_t loads[]={800,1500,3000,6000};
    unsigned compared=0, binding=0, not_ready=0, runs=0;
    for(unsigned hmi=1;hmi<=5;hmi++)
    for(unsigned ci=0;ci<sizeof(cadences)/sizeof(cadences[0]);ci++)
    for(unsigned li=0;li<sizeof(loads)/sizeof(loads[0]);li++) {
        rider_t r={0,0};
        g53_port_output_t o;
        const uint8_t slot=g53_chain_level((uint8_t)hmi);
        unsigned run_compared=0;
        g53_port_init();
        for(unsigned t=0;t<4000;t++) {
            ride_tick(&r,(uint8_t)hmi,t<300 ? 0 : loads[li],cadences[ci],&o);
            common_invariants(&o);
            if(t<3000 || !(o.trace.phases_executed&8)) continue;
            /* steady state: readiness held and the G7 limiter not binding */
            if(!o.trace.d7ec_readiness) { ++not_ready; continue; }
            if(o.trace.d7ec_ratio!=o.trace.d7ec_assist_ratio) { ++binding; continue; }
            g53_static_ratio_state_t st;
            g53_static_diag_t d;
            g53_static_ratio_init(&st);
            const g53_static_input_t si=static_in_from_trace(&o.trace,slot);
            const uint16_t tgt=g53_static_target(&si,&st,100000u,&d);   /* converged limiter */
            CHECK(tgt==(uint16_t)o.trace.d7ec_accel_target,
                "steady hmi=%u cad=%u load=%u t=%u env=%u: static %u != D224 %ld",
                hmi,cadences[ci],loads[li],t,si.env,tgt,(long)o.trace.d7ec_accel_target);
            CHECK(d.c4==(uint16_t)o.trace.d7ec_rider && d.ratio==(uint16_t)o.trace.d7ec_assist_ratio &&
                  d.d4==(uint16_t)o.trace.d7ec_requested && d.d6==(uint16_t)o.trace.d7ec_lut &&
                  d.conv==(uint16_t)o.trace.d7ec_converted,
                "steady diag hmi=%u cad=%u load=%u", hmi,cadences[ci],loads[li]);
            CHECK(d.auto_active==(slot==g53_chain_sport_slot()), "auto flag hmi=%u", hmi);
            ++compared; ++run_compared;
        }
        CHECK(run_compared>=90, "steady hmi=%u cad=%u load=%u: only %u comparable ticks",
            hmi,cadences[ci],loads[li],run_compared);
        /* S+ AUTO at the lightest load is legitimately 0 (AUTO ratio ~1.1 -> d4 == 0) */
        CHECK(o.trace.d7ec_accel_target>0 || (slot==g53_chain_sport_slot() && loads[li]<1500),
            "steady hmi=%u cad=%u load=%u: no assist",
            hmi,cadences[ci],loads[li]);
        ++runs;
    }
    printf("(a) G1-STATIC: %u runs, %u D7EC ticks exact, %u skipped limiter-binding, %u not-ready\n",
        runs,compared,binding,not_ready);
}

/* ---------------- (b) G1-STATIC-T ---------------- */
typedef struct { uint32_t until_ms; uint8_t hmi; uint16_t load; uint16_t cadence; } seg_t;

static void run_timeline(const char *name, const seg_t *seg, unsigned nseg)
{
    rider_t r={0,0};
    g53_port_output_t o;
    g53_static_ratio_state_t st;
    unsigned compared=0, mism=0, not_ready=0, rises=0, auto_ticks=0;
    int32_t prev_ratio=0;
    g53_port_init();
    g53_static_ratio_init(&st);
    for(unsigned s=0,t=0;s<nseg;s++) {
        for(;t<seg[s].until_ms;t++) {
            const uint8_t slot=g53_chain_level(seg[s].hmi);
            ride_tick(&r,seg[s].hmi,seg[s].load,seg[s].cadence,&o);
            common_invariants(&o);
            /* tick 0 is a D7EC tick: first call covers one 10 ms period, then 1 ms per call */
            const g53_static_input_t si=static_in_from_trace(&o.trace,slot);
            g53_static_diag_t d;
            const uint16_t tgt=g53_static_target(&si,&st,t==0 ? 10u : 1u,&d);
            if(!(o.trace.phases_executed&8)) {
                /* between D7EC calls the limiter state must not move */
                CHECK(st.applied_ratio==(uint16_t)o.trace.d7ec_ratio,
                    "%s t=%u: state moved between D7EC ticks", name, t);
                continue;
            }
            if(!o.trace.d7ec_readiness) ++not_ready;
            const int ok=tgt==(uint16_t)o.trace.d7ec_accel_target &&
                st.applied_ratio==(uint16_t)o.trace.d7ec_ratio &&
                d.ratio==(uint16_t)o.trace.d7ec_assist_ratio;
            if(!ok) ++mism;
            CHECK(ok, "%s t=%u hmi=%u load=%u env=%u: static %u/%u/%u vs D224 %ld D208 %ld D204 %ld",
                name,t,seg[s].hmi,seg[s].load,si.env,tgt,st.applied_ratio,d.ratio,
                (long)o.trace.d7ec_accel_target,(long)o.trace.d7ec_ratio,
                (long)o.trace.d7ec_assist_ratio);
            /* the per-level rise accessor is what D7EC loaded into D+232 */
            if(slot>=1) CHECK(g53_chain_d7ec_rise(slot)==(uint16_t)o.trace.d7ec_accel_rise,
                "%s t=%u rise accessor", name, t);
            if(o.trace.d7ec_ratio>prev_ratio) ++rises;
            if(d.auto_active) ++auto_ticks;
            prev_ratio=o.trace.d7ec_ratio;
            ++compared;
        }
    }
    CHECK(rises>0, "%s: limiter never exercised", name);
    printf("(b) G1-STATIC-T %s: %u D7EC ticks, %u mismatches, %u limiter rises, %u AUTO ticks, %u not-ready\n",
        name,compared,mism,rises,auto_ticks,not_ready);
}

static void test_timeline(void)
{
    static const seg_t levels[]={
        { 300,1,   0,60}, {2000,1,2500,60}, {3500,3,2500,60}, {5000,5,2500,75},
        {6000,2,2500,75}, {7500,4,2500,90}, {8500,5,2500,90}, {9500,1,2500,60},
        {10500,0,2500,60}, {12000,5,4000,45}, {13000,3,4000,110}, {14000,4,1200,110},
    };
    static const seg_t attack[]={
        { 300,4,   0,70}, {2000,4,1000,70}, {2600,4,6000,70}, {3600,4,1000,70},
        {3900,4,6000,90}, {4100,4,1500,90}, {4400,4,6000,40}, {5500,4, 900,40},
        {6500,4,4000,20}, {7500,4,6000,130},{8000,4, 800,130},
    };
    run_timeline("level-change",levels,sizeof(levels)/sizeof(levels[0]));
    run_timeline("splus-auto-attack",attack,sizeof(attack)/sizeof(attack[0]));
}

/* ---------------- (c) no mutation ---------------- */
typedef struct {
    uint8_t s0[sizeof(state_0)], s1[sizeof(state_1)], s2[sizeof(state_2)], s3[sizeof(state_3)];
    uint8_t s4[sizeof(state_4)], s5[sizeof(state_5)], s6[sizeof(state_6)], s7[sizeof(state_7)];
    uint32_t tick; uint8_t fp, sp; uint8_t ae; uint16_t as; uint8_t ar;
    uint8_t la[10]; uint16_t lr[10];
    g53_port_trace_t trace;
} image_t;

static void snapshot(image_t *im)
{
    memset(im,0,sizeof(*im));
    memcpy(im->s0,state_0,sizeof(state_0)); memcpy(im->s1,state_1,sizeof(state_1));
    memcpy(im->s2,state_2,sizeof(state_2)); memcpy(im->s3,state_3,sizeof(state_3));
    memcpy(im->s4,state_4,sizeof(state_4)); memcpy(im->s5,state_5,sizeof(state_5));
    memcpy(im->s6,state_6,sizeof(state_6)); memcpy(im->s7,state_7,sizeof(state_7));
    im->tick=chain_tick; im->fp=fast_phase; im->sp=supervisor_phase;
    im->ae=auto_enable; im->as=auto_scale; im->ar=auto_rise_step;
    memcpy(im->la,level_accel,sizeof(level_accel)); memcpy(im->lr,level_ratio,sizeof(level_ratio));
    im->trace=*g53_port_trace();
}

static void test_no_mutation(void)
{
    rider_t r={0,0};
    g53_port_output_t o;
    image_t before, after;
    g53_port_init();
    for(unsigned t=0;t<2500;t++) ride_tick(&r,4,t<300 ? 0 : 3000,70,&o);
    snapshot(&before);
    srand(12345);
    g53_static_ratio_state_t st;
    g53_static_ratio_init(&st);
    uint32_t sum=0;
    for(unsigned i=0;i<200000;i++) {
        g53_static_input_t si;
        g53_static_diag_t d;
        si.env=(uint16_t)(rand()&0xffff);
        si.cadence=(int16_t)((rand()%400)-60);
        si.lut_cadence=(uint16_t)(rand()&0xff);
        si.speed_native=(uint16_t)(rand()&0xffff);
        si.level=(uint8_t)(i%13==12 ? 255 : i%12);
        const uint32_t el=(i%97==0) ? 0xffffffffu : (uint32_t)(rand()%25);
        const uint16_t tgt=g53_static_target(&si,&st,el,(i&1) ? &d : NULL);
        CHECK(tgt<=40960u, "target range %u", tgt);
        CHECK(st.acc_ms<10u, "acc range");
        sum+=tgt;
        sum+=g53_chain_d7ec_rise((uint8_t)i)+(uint16_t)g53_chain_d7ec_fall()+g53_chain_sport_slot();
        sum+=g53_chain_d7ec_floor_limit((uint8_t)(i%12));
        uint16_t e; uint8_t v; g53_chain_d7ec_readiness(&e,&v); g53_chain_d7ec_readiness(NULL,NULL);
        sum+=e+v+g53_port_eb74_zero()+g53_port_eb74_threshold()+g53_port_eb74_armed()+
             g53_port_pas_true_stop()+(uint16_t)g53_port_pas_cadence();
    }
    CHECK(g53_static_target(NULL,&st,10,NULL)==0, "NULL input");
    snapshot(&after);
    CHECK(memcmp(&before,&after,sizeof(before))==0, "chain image mutated by the static map/accessors");
    /* and the chain continues exactly as an uninterrupted twin */
    rider_t r2={0,0};
    g53_port_output_t o2;
    for(unsigned t=0;t<500;t++) ride_tick(&r,4,3000,70,&o);
    image_t a1; snapshot(&a1);
    g53_port_init();
    for(unsigned t=0;t<3000;t++) ride_tick(&r2,4,t<300 ? 0 : 3000,70,&o2);
    image_t a2; snapshot(&a2);
    CHECK(memcmp(&a1,&a2,sizeof(a1))==0, "chain diverged from its twin after static-map calls");
    printf("(c) no mutation: 200000 calls, chain image byte-identical, twin identical (sum %lu)\n",
        (unsigned long)sum);
}

/* ---------------- configuration / accessors ---------------- */
static void test_accessors(void)
{
    uint16_t e; uint8_t v;
    g53_port_init();
    CHECK(g53_chain_sport_slot()==8, "sport slot");
    CHECK(g53_chain_d7ec_fall()==-409, "D3E fall %d", g53_chain_d7ec_fall());
    g53_chain_d7ec_readiness(&e,&v);
    CHECK(e==2500 && v==4, "readiness %u/%u", e, v);
    CHECK(g53_chain_d7ec_rise(0)==0, "rise slot 0");
    for(uint8_t s=1;s<10;s++) CHECK(g53_chain_d7ec_rise(s)==g53_chain_rise_step(s), "rise slot %u", s);
    for(uint8_t s=0;s<10;s++) CHECK(g53_chain_d7ec_floor_limit(s)==0, "floor table slot %u", s);
    for(unsigned h=0;h<6;h++) CHECK(g53_chain_level((uint8_t)h)==hmi_slot[h], "level map %u", h);
    /* custom tables keep the floor table at zero (no setter writes D+88) */
    {
        const uint8_t a[10]={1,8,1,7,4,6,8,3,8,8};
        const uint16_t rr[10]={1,45,1000,155,800,260,120,370,0,525};
        const uint8_t p[10]={0,100,100,100,100,100,100,100,100,100};
        g53_port_set_levels(a,rr,p);
        g53_chain_set_auto(1,7,3);
        for(uint8_t s=0;s<10;s++) CHECK(g53_chain_d7ec_floor_limit(s)==0, "floor table after set slot %u", s);
        const uint8_t a0[10]={1,4,4,5,5,6,6,7,8,8};
        const uint16_t r0[10]={1,45,95,155,215,260,310,370,525,525};
        g53_port_set_levels(a0,r0,p);
        g53_chain_set_auto(1,2,10);
    }
    /* EB74 armed: 30 startup ticks (+10 to 300), then a 100-tick unloaded check */
    {
        rider_t r={0,0};
        g53_port_output_t o;
        g53_port_init();
        CHECK(!g53_port_eb74_armed(), "armed after reset");
        for(unsigned t=0;t<129;t++) ride_tick(&r,2,0,0,&o);
        CHECK(!g53_port_eb74_armed(), "armed too early");
        ride_tick(&r,2,0,0,&o);
        CHECK(g53_port_eb74_armed(), "not armed after 130 unloaded ticks");
        CHECK(g53_port_eb74_threshold()==960, "threshold on the arming tick");
        ride_tick(&r,2,0,0,&o);   /* the engage threshold is published from the next tick */
        CHECK(g53_port_eb74_zero()==750 && g53_port_eb74_threshold()==995,
            "EB74 zero/threshold %u/%u", g53_port_eb74_zero(), g53_port_eb74_threshold());
        CHECK(g53_port_pas_true_stop(), "no pedalling must read true-stop");
        /* loaded during the check -> back to 90: arming is delayed by the load */
        g53_port_init();
        for(unsigned t=0;t<100;t++) ride_tick(&r,2,0,0,&o);
        for(unsigned t=0;t<50;t++) ride_tick(&r,2,4000,0,&o);
        CHECK(!g53_port_eb74_armed(), "armed while loaded during the check");
        for(unsigned t=0;t<10;t++) ride_tick(&r,2,0,0,&o);
        CHECK(g53_port_eb74_armed(), "not armed 10 unloaded ticks after a loaded check");
        /* pedalling -> not true-stop, signed cadence published; stop -> true-stop again */
        int32_t prev_accel=0;
        for(unsigned t=0;t<1500;t++) { prev_accel=o.trace.d7ec_accel; ride_tick(&r,2,2000,60,&o); }
        CHECK(!g53_port_pas_true_stop() && g53_port_pas_cadence()>0 &&
              g53_port_pas_cadence()==(int16_t)o.trace.cadence, "riding cadence %d", g53_port_pas_cadence());
        CHECK(g53_port_eb74_threshold()==(prev_accel!=0 ? 820 : g53_port_eb74_zero()+245),
            "threshold while driving %u (D238 %ld)", g53_port_eb74_threshold(), (long)prev_accel);
        unsigned stop_ms=0;
        while(!g53_port_pas_true_stop() && stop_ms<1000) { ride_tick(&r,2,2000,0,&o); ++stop_ms; }
        CHECK(stop_ms>0 && stop_ms<=320, "true-stop after %u ms", stop_ms);
        CHECK(g53_port_pas_cadence()==0, "cadence at true-stop");
    }
    printf("accessors: sport=8 fall=-409 ready=2500/4 rise=D+232 floor table 0, EB74 armed/zero/threshold, true-stop\n");
}

/* ---------------- limiter time base ---------------- */
static void test_time_base(void)
{
    g53_port_init();
    g53_static_input_t si={2000,60,0,200,9};   /* fixed slot 9, ratio 525 */
    g53_static_ratio_state_t a, b, c;
    g53_static_diag_t d;
    g53_static_ratio_init(&a); g53_static_ratio_init(&b); g53_static_ratio_init(&c);
    /* same 300 ms in 1 ms, 3 ms+7 ms jitter and one 300 ms call -> same state */
    for(unsigned i=0;i<300;i++) (void)g53_static_target(&si,&a,1,NULL);
    for(unsigned i=0;i<30;i++) { (void)g53_static_target(&si,&b,3,NULL); (void)g53_static_target(&si,&b,7,NULL); }
    (void)g53_static_target(&si,&c,300,NULL);
    CHECK(a.applied_ratio==300 && b.applied_ratio==300 && c.applied_ratio==300,
        "elapsed-time invariance %u/%u/%u", a.applied_ratio, b.applied_ratio, c.applied_ratio);
    /* many calls without a full 10 ms do not advance the limiter */
    g53_static_ratio_init(&a);
    for(unsigned i=0;i<1000;i++) (void)g53_static_target(&si,&a,0,NULL);
    CHECK(a.applied_ratio==0, "per-call advance %u", a.applied_ratio);
    /* falls are immediate in the output, state only moves on a 10 ms boundary */
    g53_static_ratio_init(&a);
    (void)g53_static_target(&si,&a,100000,NULL);
    CHECK(a.applied_ratio==525, "converge");
    si.level=2;   /* ratio 95 */
    (void)g53_static_target(&si,&a,3,&d);
    CHECK(d.applied_ratio==95 && a.applied_ratio==525, "held fall %u/%u", d.applied_ratio, a.applied_ratio);
    (void)g53_static_target(&si,&a,7,&d);
    CHECK(d.applied_ratio==95 && a.applied_ratio==95, "fall on boundary %u/%u", d.applied_ratio, a.applied_ratio);
    /* negative cadence: no assist, time consumed, no limiter step */
    si.level=9; si.cadence=-5;
    CHECK(g53_static_target(&si,&a,50,&d)==0 && a.applied_ratio==95 && a.acc_ms==0, "backpedal");
    printf("time base: limiter per elapsed 10 ms (1/3+7/300 ms equal), no per-call advance, backpedal hold\n");
}

int main(void)
{
    test_accessors();
    test_time_base();
    test_steady();
    test_timeline();
    test_no_mutation();
    CHECK(hold_nonzero==0, "D+222 floor non-zero on %u ticks", hold_nonzero);
    CHECK(armed_violations==0, "rider input before EB74 armed on %u ticks", armed_violations);
    printf("floor: D+222 == 0 on every observed tick (inert)\n");
    if(failures) { printf("ASSIST-V3 static map: FAIL (%u/%u)\n", failures, checks); return 1; }
    printf("ASSIST-V3 static map (G1-STATIC, G1-STATIC-T, no-mutation): PASS (%u checks)\n", checks);
    return 0;
}
