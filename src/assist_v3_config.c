#include "assist_v3_config.h"
#include "config.h"
#include <string.h>

/* One 320-byte CONFIG_A slot: 12-byte header, 296-byte image, 8 reserved bytes,
 * CRC32 at 316 programmed last. Six slots fit the 2 KiB page. */
#define REC_MAGIC 0x46433341UL
#define REC_IMAGE_OFF 12U
#define REC_CRC_OFF 316U
#define FRAME_MAX ((ASSIST_V3_EFFECTIVE_LEN+7U)/8U)
#if ASSIST_V3
#define CAPS_SUPPORTED ASSIST_V3_CAPS
#define ENGINE_DEFAULT ASSIST_V3_ENGINE_V3
#else
#define CAPS_SUPPORTED (ASSIST_V3_CAPS & ~ASSIST_V3_CAP_BEHAVIOR)
#define ENGINE_DEFAULT ASSIST_V3_ENGINE_G5300
#endif
_Static_assert(sizeof(assist_v3_values_t)==296U,"configured image layout");
typedef struct { uint16_t lo, hi, response, start, carry, torque, power, floor_rpm;
                 uint32_t growth_lo_q16, growth_hi_q16; } profile_t;
/* ENVELOPE_STUDY section 3 candidate profiles; autonomous SIL and bike tuning pending. */
static const profile_t profile[ASSIST_V3_MODES]={
    {47,190,40,40,20,80,350,20,66465,66451}, {119,387,60,60,70,100,65000,40,66316,66311},
    {188,512,80,80,50,100,65000,30,66195,66197}, {394,700,90,90,80,100,65000,30,65913,65914},
    {292,945,80,80,50,100,65000,30,66309,66311}, {119,387,60,60,60,100,65000,35,66316,66311}
};
static const uint16_t factory_ratio[ASSIST_V3_LEVELS]={95,215,310,525,525};
static const uint16_t anchor_ratio[ASSIST_V3_MODES]={95,215,310,525,525,215};
static const uint8_t factory_accel[ASSIST_V3_LEVELS]={4,5,6,8,8};
static assist_v3_values_t configured;
static assist_v3_effective_t effective_cache;
static uint16_t generation, saved_generation, cache_generation;
static uint8_t cache_level, flash_state, persist_outcome;
static uint16_t persist_expected;
static bool persist_pending, engine_active;
static const assist_v3_flash_t *flash_hal;
static uint16_t legacy_ratio[ASSIST_V3_LEVELS];
static uint8_t legacy_power[ASSIST_V3_LEVELS], legacy_accel[ASSIST_V3_LEVELS];
static uint32_t pack_voltage_mv;
static struct {
    bool live;
    uint8_t owner, declared, frames;
    uint16_t mask;
    uint32_t last_ms;
    uint8_t staging[ASSIST_V3_EFFECTIVE_LEN];
} xfer;

static uint16_t rd16(const uint8_t *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static uint32_t rd32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void wr16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void wr32(uint8_t *p,uint32_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);p[2]=(uint8_t)(v>>16);p[3]=(uint8_t)(v>>24);}
uint16_t assist_v3_crc16(const uint8_t *p,uint16_t n){
    uint16_t c=0xFFFFU;uint16_t i;uint8_t b;
    for(i=0;i<n;i++){c^=(uint16_t)p[i]<<8;for(b=0;b<8;b++)c=(c&0x8000U)?(uint16_t)((c<<1)^0x1021U):(uint16_t)(c<<1);}
    return c;
}
uint32_t assist_v3_crc32(const uint8_t *p,uint32_t n){
    uint32_t c=0xFFFFFFFFUL,i;uint8_t b;
    for(i=0;i<n;i++){c^=p[i];for(b=0;b<8;b++)c=(c>>1)^(0xEDB88320UL&(uint32_t)(-(int32_t)(c&1U)));}
    return ~c;
}
uint16_t assist_v3_generation_next(uint16_t g){g++;if(g>=ASSIST_V3_KEEP)g=0U;return g;}
static void unset(assist_v3_values_t *v){uint8_t i;memset(v,0xFF,sizeof(*v));for(i=0;i<ASSIST_V3_LEVELS;i++)v->level_mode[i]=(uint8_t)(i+1U);}
static uint16_t hw_power(void){uint32_t w=(pack_voltage_mv*15U+500U)/1000U;return (uint16_t)(w>65000U?65000U:w);}
static uint16_t clamp16(uint32_t x,uint16_t hi){return (uint16_t)(x>hi?hi:x);}
static uint16_t assist_curve(uint8_t m,uint16_t a){
    const profile_t *q=&profile[m];uint32_t v,f;uint16_t n,ceiling;
    if(a==0U)return q->lo;
    if(a==50U)return anchor_ratio[m];
    if(a>=100U)return q->hi;
    if(a<50U){v=(uint32_t)q->lo<<16;n=a;f=q->growth_lo_q16;ceiling=anchor_ratio[m];}
    else{v=(uint32_t)anchor_ratio[m]<<16;n=(uint16_t)(a-50U);f=q->growth_hi_q16;ceiling=q->hi;}
    while(n--){v=(uint32_t)(((uint64_t)v*f+32768U)>>16);}
    return clamp16((v+32768U)>>16,ceiling);
}
static uint16_t profile_default(uint8_t m,uint8_t p){
    const profile_t *q=&profile[m];switch(p){
    case 0:return 50U;case 1:return q->torque;case 2:return q->power;case 3:return q->response;
    case 4:return q->start;case 5:return q->carry;
    case 8:return m==3U?1U:assist_curve(m,50U);
    case 9:return m==5U?119U:ASSIST_V3_UNSET;
    case 10:return m==3U?525U:ASSIST_V3_UNSET;
    case 11:return m==3U?50U:0U;
    case 12:case 13:return q->response;
    case 16:return q->carry;case 17:return 1200U;case 18:return 15U;
    case 19:return m==5U?50U:ASSIST_V3_UNSET;
    case 20:return 0x8000U;default:return ASSIST_V3_UNSET;
    }
}
static bool param_mask(uint8_t p){return p<24U&&((ASSIST_V3_PARAM_MASK>>p)&1UL)!=0UL;}
static bool range_ok(uint8_t p,uint16_t x){
    if(x==ASSIST_V3_UNSET)return true;
    if(p==1U)return x>=10U&&x<=100U;
    if(p==2U)return x>=50U&&x<=65000U;
    if(p==8U||p==9U||p==10U)return x<=1000U;
    if(p==17U)return x<=1200U;
    if(p==18U)return x<=15U;
    if(p==20U)return x>=0x7FCEU&&x<=0x8032U;
    return x<=100U;
}
static bool fail(uint8_t *reason,uint8_t *index,uint8_t r,uint8_t i){if(reason)*reason=r;if(index)*index=i;return false;}
static bool applicable(uint8_t m,uint8_t p,uint16_t progression){
    if(!param_mask(p))return false;
    if(p==9U||p==19U)return m==5U;
    if(p==10U)return progression>0U;
    return true;
}
static bool image_ok(const assist_v3_values_t *v){
    uint8_t m,p,i;
    if(v->engine!=ASSIST_V3_UNSET&&v->engine>1U)return false;
#if !ASSIST_V3
    if(v->engine==1U)return false;
#endif
    for(i=0;i<ASSIST_V3_LEVELS;i++)if(v->level_mode[i]<1U||v->level_mode[i]>6U)return false;
    if(v->reserved!=0xFFU)return false;
    for(m=0;m<6U;m++)for(p=0;p<24U;p++){
        uint16_t x=v->mode[m][p];
        if(x==ASSIST_V3_KEEP)return false;
        if(x!=ASSIST_V3_UNSET&&(!param_mask(p)||!range_ok(p,x)))return false;
    }
    return true;
}
/* The common engine harness emits only the GLOBAL object. */
void assist_v3_block_encode(const assist_v3_values_t *v,uint32_t caps,uint16_t gen,uint8_t out[ASSIST_V3_BLOCK_LEN]){
    uint8_t i;memset(out,0xFF,ASSIST_V3_GLOBAL_LEN);
    out[0]='B';out[1]='V';out[2]=3U;out[3]=2U;wr16(out+4,ASSIST_V3_GLOBAL_LEN);
    out[6]=0U;out[7]=0U;out[8]=1U;out[9]=0U;wr32(out+10,caps);wr16(out+14,gen);
    wr16(out+16,v->engine);for(i=0;i<5U;i++)out[18U+i]=v->level_mode[i];
    wr16(out+32,assist_v3_crc16(out,32U));
}
/* Decode a write into a private candidate. KEEP and short count preserve current values. */
static bool decode(const uint8_t *b,uint16_t len,uint16_t curgen,const assist_v3_values_t *base,assist_v3_values_t *out,uint8_t *reason,uint8_t *index){
    uint8_t obj,idx,count,p;uint16_t expect,oldprog,newprog;uint32_t caps;
    if(len<2U||b[0]!='B'||b[1]!='V')return fail(reason,index,0,0);
    if(len<16U)return fail(reason,index,1,4);
    obj=b[2];idx=b[6];count=b[7];
    if((obj!=2U&&obj!=3U)||b[3]!=2U)return fail(reason,index,0,2);
    expect=obj==2U?(uint16_t)(18U+2U*count):ASSIST_V3_GLOBAL_LEN;
    if(rd16(b+4)!=expect||len!=expect||count>(obj==2U?24U:0U)||b[8]!=(obj==2U?2U:1U))return fail(reason,index,1,4);
    if(obj==2U?(idx<1U||idx>6U):(idx!=0U))return fail(reason,index,3,6);
    if(b[9]!=0U)return fail(reason,index,7,9);
    caps=rd32(b+10);if((caps&~CAPS_SUPPORTED)!=0U)return fail(reason,index,4,10);
    if(rd16(b+14)!=ASSIST_V3_UNSET&&rd16(b+14)!=curgen)return fail(reason,index,6,14);
    if(rd16(b+len-2U)!=assist_v3_crc16(b,(uint16_t)(len-2U)))return fail(reason,index,2,(uint8_t)(len-2U));
    *out=*base;
    if(obj==3U){
        uint16_t e=rd16(b+16);uint8_t i;
        if(e!=ASSIST_V3_KEEP){
            if(e!=ASSIST_V3_UNSET&&e>1U)return fail(reason,index,3,16);
#if !ASSIST_V3
            if(e==1U)return fail(reason,index,4,16);
#endif
            out->engine=e;
        }
        for(i=0;i<5U;i++)if(b[18U+i]!=0xFEU){if(b[18U+i]<1U||b[18U+i]>6U)return fail(reason,index,3,(uint8_t)(18U+i));out->level_mode[i]=b[18U+i];}
        for(i=23U;i<32U;i++)if(b[i]!=0xFFU)return fail(reason,index,4,i);
        return true;
    }
    oldprog=base->mode[idx-1U][11U];
    newprog=oldprog==ASSIST_V3_UNSET?profile_default((uint8_t)(idx-1U),11U):oldprog;
    if(count>11U&&rd16(b+38U)!=ASSIST_V3_KEEP){uint16_t x=rd16(b+38U);newprog=x==ASSIST_V3_UNSET?profile_default((uint8_t)(idx-1U),11U):x;}
    for(p=0;p<count;p++){
        uint16_t x=rd16(b+16U+2U*p);uint8_t off=(uint8_t)(16U+2U*p);
        if(x==ASSIST_V3_KEEP)continue;
        if(!param_mask(p)||!applicable((uint8_t)(idx-1U),p,newprog)){
            if(x!=ASSIST_V3_UNSET)return fail(reason,index,4,off);
        }else if(!range_ok(p,x))return fail(reason,index,3,off);
        out->mode[idx-1U][p]=x;
    }
    return true;
}
bool assist_v3_block_validate(const uint8_t *b,uint16_t len,uint16_t gen,assist_v3_values_t *out,uint8_t *reason,uint8_t *index){
    assist_v3_values_t candidate;
    if(!decode(b,len,gen,&configured,&candidate,reason,index))return false;
    if(out)*out=candidate;
    return true;
}

void assist_v3_config_set_legacy(uint8_t level,uint16_t ratio,uint8_t power,uint8_t accel){
    uint8_t i;if(level<1U||level>5U)return;i=(uint8_t)(level-1U);
    legacy_ratio[i]=ratio;legacy_power[i]=power;legacy_accel[i]=accel;cache_level=0U;
}
void assist_v3_config_set_pack_voltage_mv(uint32_t mv){
    if(mv>100000U)mv=100000U;
    pack_voltage_mv=mv;
}
static uint16_t macro_value(uint8_t m,uint8_t level,uint8_t p,uint8_t *src){
    uint16_t x=configured.mode[m][p];const profile_t *q=&profile[m];
    if(x!=ASSIST_V3_UNSET){*src=3U;return x;}
    *src=0U;x=profile_default(m,p);
    if(p==0U&&legacy_ratio[level]!=factory_ratio[level]){
        uint32_t anchor=anchor_ratio[m];
        uint32_t target=(anchor*legacy_ratio[level]+factory_ratio[level]/2U)/factory_ratio[level];
        if(target<=q->lo)x=0U;
        else if(target<=anchor)x=clamp16((target-q->lo)*50U/(anchor-q->lo),100U);
        else x=clamp16(50U+(target-anchor)*50U/(q->hi-anchor),100U);
        *src=2U;
    }
    if(p==2U){
        uint16_t hw=hw_power();uint16_t pct=q->power>=hw?100U:(uint16_t)(((uint32_t)q->power*100U+hw/2U)/hw);
        uint8_t lp=legacy_power[level];
        uint16_t scaled=(uint16_t)(((uint32_t)pct*lp+50U)/100U);
        x=clamp16(((uint32_t)hw*scaled+50U)/100U,hw);
        if(lp!=100U)*src=2U;
    }
    return x;
}
static void resolve_cache(uint8_t level,uint8_t m){
    uint8_t p,macro_src[6];uint16_t macro[6],progression;
    for(p=0;p<6U;p++)macro[p]=macro_value(m,(uint8_t)(level-1U),p,&macro_src[p]);
    progression=configured.mode[m][11U]==ASSIST_V3_UNSET?
        (m==3U?macro[0]:0U):configured.mode[m][11U];
    for(p=0;p<24U;p++){
        uint16_t raw=configured.mode[m][p],v=ASSIST_V3_UNSET;uint8_t s=5U;
        if(!applicable(m,p,progression)){effective_cache.value[p]=v;effective_cache.source[p]=s;continue;}
        if(p<6U){v=macro[p];s=macro_src[p];}
        else if(raw!=ASSIST_V3_UNSET){v=raw;s=3U;}
        else{
            s=1U;
            switch(p){
            case 8U:v=m==3U?1U:assist_curve(m,macro[0]);break;
            case 9U:v=profile_default(m,p);s=0U;break;
            case 10U:v=m==3U?assist_curve(m,macro[0]):1000U;break;
            case 11U:v=m==3U?macro[0]:0U;break;
            case 12U:case 13U:v=macro[3];break;
            case 16U:v=macro[5];break;
            case 17U:v=(uint16_t)(macro[5]*12U);break;
            case 18U:v=(uint16_t)(macro[5]*15U/100U);break;
            default:v=profile_default(m,p);s=0U;break;
            }
        }
        if(p==12U&&raw==ASSIST_V3_UNSET&&configured.mode[m][3U]==ASSIST_V3_UNSET&&
           legacy_accel[level-1U]!=factory_accel[level-1U]){
            v=clamp16(((uint32_t)v*legacy_accel[level-1U]+factory_accel[level-1U]/2U)/factory_accel[level-1U],100U);s=2U;
        }
        if(p==2U&&raw!=ASSIST_V3_UNSET&&v>hw_power()){v=hw_power();s=4U;}
        effective_cache.value[p]=v;effective_cache.source[p]=s;
    }
    /* BASIC macros with an ADVANCED override have no effect in the corresponding dimension. */
    if((m==3U&&(configured.mode[m][11U]!=ASSIST_V3_UNSET&&configured.mode[m][10U]!=ASSIST_V3_UNSET))||
       (m!=3U&&configured.mode[m][8U]!=ASSIST_V3_UNSET))
        effective_cache.source[0]=6U;
    if(configured.mode[m][12U]!=ASSIST_V3_UNSET&&configured.mode[m][13U]!=ASSIST_V3_UNSET)
        effective_cache.source[3]=6U;
    if(configured.mode[m][16U]!=ASSIST_V3_UNSET&&configured.mode[m][17U]!=ASSIST_V3_UNSET&&configured.mode[m][18U]!=ASSIST_V3_UNSET)
        effective_cache.source[5]=6U;
    cache_level=level;cache_generation=generation;
}
const assist_v3_effective_t *assist_v3_effective(uint8_t level){
    uint8_t m,s;uint16_t v;
    if(level<1U||level>5U)return 0;
    if(cache_level!=level||cache_generation!=generation)
        resolve_cache(level,(uint8_t)(configured.level_mode[level-1U]-1U));
    /* Pack voltage changes the W readback, not the mode's character curve. */
    m=(uint8_t)(configured.level_mode[level-1U]-1U);
    v=macro_value(m,(uint8_t)(level-1U),2U,&s);
    if(configured.mode[m][2U]!=ASSIST_V3_UNSET&&v>hw_power()){v=hw_power();s=4U;}
    effective_cache.value[2U]=v;effective_cache.source[2U]=s;
    return &effective_cache;
}
uint8_t assist_v3_effective_release_pct(uint8_t level){
    const assist_v3_effective_t *e=assist_v3_effective(level);
    return e&&e->value[13U]<=100U?(uint8_t)e->value[13U]:0U;
}
bool assist_v3_config_engine_requested(void){
#if ASSIST_V3
    return configured.engine==ASSIST_V3_UNSET||configured.engine==ASSIST_V3_ENGINE_V3;
#else
    return false;
#endif
}
bool assist_v3_config_engine_active(void){return engine_active;}
void assist_v3_config_set_engine_active(bool active){engine_active=active;}
uint32_t assist_v3_config_caps(void){return CAPS_SUPPORTED;}
uint16_t assist_v3_config_generation(void){return generation;}
uint8_t assist_v3_config_flash_record_state(void){return flash_state;}
void assist_v3_config_ram_values(assist_v3_values_t *out){if(out)*out=configured;}
bool assist_v3_config_xfer_live(void){return xfer.live;}

typedef enum { SLOT_FREE,SLOT_INVALID,SLOT_V1,SLOT_NEWER,SLOT_VALID } slot_class_t;
static bool slot_free(const uint8_t *s,uint16_t n){uint16_t i;for(i=0;i<n;i++)if(s[i]!=0xFFU)return false;return true;}
static slot_class_t classify(const uint8_t *s,assist_v3_values_t *v,uint16_t *g){
    assist_v3_values_t candidate;uint16_t i;
    if(slot_free(s,ASSIST_V3_SLOT_SIZE))return SLOT_FREE;
    if(rd32(s)!=REC_MAGIC)return SLOT_INVALID;
    if(s[4]==1U)return rd32(s+124U)==assist_v3_crc32(s,124U)?SLOT_V1:SLOT_INVALID;
    if(s[4]!=2U||s[5]>2U)return SLOT_NEWER;
    if(s[5]!=2U||rd16(s+10)!=sizeof(candidate)||rd16(s+6)!=ASSIST_V3_SLOT_SIZE)return SLOT_INVALID;
    if(rd32(s+REC_CRC_OFF)!=assist_v3_crc32(s,REC_CRC_OFF))return SLOT_INVALID;
    if(rd16(s+8)>=ASSIST_V3_KEEP)return SLOT_INVALID;
    for(i=REC_IMAGE_OFF+sizeof(candidate);i<REC_CRC_OFF;i++)if(s[i]!=0xFFU)return SLOT_INVALID;
    memcpy(&candidate,s+REC_IMAGE_OFF,sizeof(candidate));
    if(!image_ok(&candidate))return SLOT_INVALID;
    if(v)*v=candidate;
    if(g)*g=rd16(s+8);
    return SLOT_VALID;
}
static uint8_t scan(assist_v3_values_t *v,uint16_t *g,int *top_out){
    int top=-1,i;bool invalid=false,v1=false,newer=false;
    if(!flash_hal||!flash_hal->page){if(top_out)*top_out=-1;return ASSIST_V3_FLASH_ABSENT;}
    for(i=0;i<(int)ASSIST_V3_SLOT_COUNT;i++)
        if(!slot_free(flash_hal->page+(uint32_t)i*ASSIST_V3_SLOT_SIZE,ASSIST_V3_SLOT_SIZE))top=i;
    if(top_out)*top_out=top;
    for(i=top;i>=0;i--){
        slot_class_t c=classify(flash_hal->page+(uint32_t)i*ASSIST_V3_SLOT_SIZE,v,g);
        if(c==SLOT_VALID)return ASSIST_V3_FLASH_VALID;
        if(c==SLOT_NEWER)newer=true;
        if(c==SLOT_V1)v1=true;
        if(c==SLOT_INVALID)invalid=true;
    }
    if(!v1){
        /* v1 used 128-byte slots; an old valid record may start inside a v2 slot. */
        for(i=0;i<16;i++){
            const uint8_t *s=flash_hal->page+(uint32_t)i*128U;
            if(rd32(s)==REC_MAGIC&&s[4]==1U&&
               rd32(s+124U)==assist_v3_crc32(s,124U)){v1=true;break;}
        }
    }
    return newer?ASSIST_V3_FLASH_NEWER:(v1?ASSIST_V3_FLASH_V1_IGNORED:
           (invalid?ASSIST_V3_FLASH_CORRUPT:ASSIST_V3_FLASH_ABSENT));
}
void assist_v3_config_saved_values(assist_v3_values_t *out){
    uint16_t g;if(!out)return;unset(out);(void)scan(out,&g,0);
}
uint8_t assist_v3_config_persist_state(void){
    assist_v3_values_t saved;uint16_t g;
    if(persist_pending)return ASSIST_V3_PERSIST_PENDING;
    if(persist_outcome>=ASSIST_V3_PERSIST_FAILED)return persist_outcome;
    unset(&saved);if(scan(&saved,&g,0)==ASSIST_V3_FLASH_VALID&&memcmp(&saved,&configured,sizeof(saved))==0)return ASSIST_V3_PERSIST_CLEAN;
    return ASSIST_V3_PERSIST_DIRTY;
}
static bool persist_now(void){
    uint8_t slot[ASSIST_V3_SLOT_SIZE];assist_v3_values_t old;uint16_t oldgen;int top;uint32_t off;
    uint8_t st=scan(&old,&oldgen,&top);
    if(st==ASSIST_V3_FLASH_VALID&&memcmp(&old,&configured,sizeof(old))==0){saved_generation=oldgen;flash_state=st;return true;}
    if(!flash_hal||!flash_hal->erase||!flash_hal->program)return false;
    memset(slot,0xFF,sizeof(slot));wr32(slot,REC_MAGIC);slot[4]=2U;slot[5]=2U;
    wr16(slot+6,ASSIST_V3_SLOT_SIZE);wr16(slot+8,generation);wr16(slot+10,sizeof(configured));
    memcpy(slot+REC_IMAGE_OFF,&configured,sizeof(configured));
    wr32(slot+REC_CRC_OFF,assist_v3_crc32(slot,REC_CRC_OFF));
    top++;
    if(top>=(int)ASSIST_V3_SLOT_COUNT){if(!flash_hal->erase())goto failure;top=0;}
    off=(uint32_t)top*ASSIST_V3_SLOT_SIZE;
    if(!flash_hal->program(off,slot,REC_CRC_OFF))goto failure;
    if(!flash_hal->program(off+REC_CRC_OFF,slot+REC_CRC_OFF,4U))goto failure;
    if(memcmp(flash_hal->page+off,slot,ASSIST_V3_SLOT_SIZE)!=0)goto failure;
    saved_generation=generation;flash_state=ASSIST_V3_FLASH_VALID;return true;
failure:flash_state=scan(0,0,0);return false;
}
void assist_v3_config_init(const assist_v3_flash_t *flash){
    uint8_t i;uint16_t g=0U;flash_hal=flash;unset(&configured);
    generation=1U;saved_generation=0U;persist_pending=false;persist_outcome=0U;
    engine_active=false;cache_level=0U;cache_generation=0U;pack_voltage_mv=48000U;
    memset(&xfer,0,sizeof(xfer));
    for(i=0;i<5U;i++){legacy_ratio[i]=factory_ratio[i];legacy_power[i]=100U;legacy_accel[i]=factory_accel[i];}
    flash_state=scan(&configured,&g,0);
    if(flash_state==ASSIST_V3_FLASH_VALID){saved_generation=g;generation=assist_v3_generation_next(g);}
}
static assist_v3_reply_t reply(uint8_t kind,uint8_t reason,uint8_t index,uint8_t target){
    assist_v3_reply_t r={kind,reason,index,target,generation};return r;
}
bool assist_v3_config_owns_command(uint16_t cmd){return cmd>=ASSIST_V3_CMD_CAPS&&cmd<=ASSIST_V3_CMD_CONTROL;}
static uint8_t shadow_mask(void){
    uint8_t i,mask=0U;for(i=0;i<5U;i++){
        uint8_t m=(uint8_t)(configured.level_mode[i]-1U);
        if(configured.mode[m][0U]!=ASSIST_V3_UNSET||configured.mode[m][8U]!=ASSIST_V3_UNSET||
           configured.mode[m][2U]!=ASSIST_V3_UNSET||configured.mode[m][3U]!=ASSIST_V3_UNSET||
           configured.mode[m][12U]!=ASSIST_V3_UNSET)mask|=(uint8_t)(1U<<i);
    }return mask;
}
static void caps(uint8_t *b){
    memset(b,0,ASSIST_V3_CAPS_LEN);b[0]='B';b[1]='V';b[2]=2U;b[3]=2U;
    b[4]=2U;b[5]=2U;b[6]=6U;b[7]=(1U<<2)|(1U<<3);
    wr16(b+8,ASSIST_V3_EFFECTIVE_LEN);b[10]=24U;b[11]=shadow_mask();
    wr32(b+12,CAPS_SUPPORTED);wr32(b+16,ASSIST_V3_PARAM_MASK);
    wr16(b+20,generation);b[22]=assist_v3_config_persist_state();b[23]=flash_state;
    b[24]=engine_active?1U:0U;b[25]=assist_v3_config_engine_requested()?1U:0U;
    wr16(b+26,hw_power());wr16(b+28,assist_v3_crc16(b,28U));
}
static void header(uint8_t *b,uint8_t obj,uint8_t idx,uint8_t count,uint16_t len,uint8_t flags,uint16_t gen){
    memset(b,0,len);b[0]='B';b[1]='V';b[2]=obj;b[3]=2U;wr16(b+4,len);
    b[6]=idx;b[7]=count;b[8]=obj==2U?2U:1U;b[9]=flags;
    wr32(b+10,CAPS_SUPPORTED);wr16(b+14,gen);
}
assist_v3_reply_t assist_v3_config_can_read(uint8_t source,uint16_t cmd,uint8_t dlen,const uint8_t *data,uint8_t *out,uint8_t *out_len){
    uint8_t obj,idx,view,level,m,p;assist_v3_values_t saved;uint16_t sg=0U,len;
    *out_len=0U;if(source!=ASSIST_V3_SOURCE_TOOL)return reply(0,0,0,source);
    if(cmd==ASSIST_V3_CMD_CAPS){caps(out);*out_len=ASSIST_V3_CAPS_LEN;return reply(0,0,0,source);}
    if(cmd!=ASSIST_V3_CMD_BLOCK)return reply(0,0,0,source);
    if(dlen!=4U)return reply(ASSIST_V3_REPLY_ERROR,ASSIST_V3_REASON_LENGTH,0,source);
    obj=data[0];idx=data[1];view=data[2];level=data[3];
    if((obj!=2U&&obj!=3U)||view>3U||((obj==2U)&&(idx<1U||idx>6U))||(obj==3U&&idx!=0U)||
       (view==1U&&(level<1U||level>5U))||(view!=1U&&level!=0U))
        return reply(ASSIST_V3_REPLY_ERROR,ASSIST_V3_REASON_RANGE,0,source);
    if(view==0U){unset(&saved);if(scan(&saved,&sg,0)!=ASSIST_V3_FLASH_VALID)sg=0U;}
    if(obj==3U){
        const assist_v3_values_t *v=view==0U?&saved:&configured;uint8_t i;
        len=ASSIST_V3_GLOBAL_LEN;header(out,obj,0U,0U,len,view,view==0U?sg:generation);
        wr16(out+16,view==2U?ENGINE_DEFAULT:(view==1U?(assist_v3_config_engine_requested()?1U:0U):v->engine));
        for(i=0;i<5U;i++)out[18U+i]=view==2U?(uint8_t)(i+1U):v->level_mode[i];
        memset(out+23,0xFF,9U);wr16(out+32,assist_v3_crc16(out,32U));*out_len=(uint8_t)len;
        return reply(0,0,0,source);
    }
    m=(uint8_t)(idx-1U);len=view==1U?ASSIST_V3_EFFECTIVE_LEN:ASSIST_V3_PROFILE_LEN;
    header(out,obj,idx,24U,len,view==1U?level:view,view==0U?sg:generation);
    if(view==1U){
        if(configured.level_mode[level-1U]==idx)(void)assist_v3_effective(level);
        else resolve_cache(level,m);
    }
    for(p=0;p<24U;p++){
        uint16_t v=ASSIST_V3_UNSET;
        if(view==0U)v=saved.mode[m][p];
        else if(view==3U)v=configured.mode[m][p];
        else if(view==2U)v=profile_default(m,p);
        else v=effective_cache.value[p];
        wr16(out+16U+2U*p,v);
    }
    if(view==1U){
        memcpy(out+64,effective_cache.source,24U);
        if(configured.level_mode[level-1U]!=idx)cache_level=0U;
    }
    wr16(out+len-2U,assist_v3_crc16(out,(uint16_t)(len-2U)));*out_len=(uint8_t)len;
    return reply(0,0,0,source);
}
static assist_v3_reply_t abort_xfer(uint8_t reason,uint8_t index){
    uint8_t owner=xfer.owner;xfer.live=false;return reply(ASSIST_V3_REPLY_ERROR,reason,index,owner);
}
bool assist_v3_config_other_declaration(uint8_t source,assist_v3_reply_t *abort_reply){
    *abort_reply=reply(0,0,0,source);if(!xfer.live)return true;
    if(source!=xfer.owner)return false;
    *abort_reply=abort_xfer(ASSIST_V3_REASON_BUSY,ASSIST_V3_INDEX_NONE);return true;
}
assist_v3_reply_t assist_v3_config_can_declare(uint8_t source,uint8_t len,uint32_t now_ms){
    if(source!=ASSIST_V3_SOURCE_TOOL)return reply(0,0,0,source);
    if(xfer.live)return reply(ASSIST_V3_REPLY_ERROR,ASSIST_V3_REASON_BUSY,ASSIST_V3_INDEX_NONE,source);
    if(len<18U||len>ASSIST_V3_EFFECTIVE_LEN||((len-18U)&1U)!=0U)
        return reply(ASSIST_V3_REPLY_ERROR,ASSIST_V3_REASON_LENGTH,4U,source);
    memset(&xfer,0,sizeof(xfer));xfer.live=true;xfer.owner=source;xfer.declared=len;xfer.last_ms=now_ms;
    return reply(ASSIST_V3_REPLY_DECL_ACK,0,0,source);
}
assist_v3_reply_t assist_v3_config_can_frame(uint8_t source,uint8_t frame_no,bool is_end,const uint8_t *data,uint8_t dlen,uint32_t now_ms){
    uint16_t off;uint8_t i,reason=0U,index=0U;assist_v3_values_t candidate;
    if(!xfer.live||source!=xfer.owner)return reply(0,0,0,source);
    xfer.last_ms=now_ms;off=(uint16_t)frame_no*8U;
    if(frame_no>=FRAME_MAX||dlen==0U||dlen>8U||off+dlen>xfer.declared||
       (is_end?(off+dlen!=xfer.declared):(dlen!=8U))||((xfer.mask>>frame_no)&1U)!=0U)
        return abort_xfer(ASSIST_V3_REASON_LENGTH,frame_no);
    memcpy(xfer.staging+off,data,dlen);xfer.mask|=(uint16_t)(1U<<frame_no);xfer.frames++;
    if(!is_end)return reply(0,0,0,source);
    for(i=0;i<=(uint8_t)frame_no;i++)if(((xfer.mask>>i)&1U)==0U)return abort_xfer(ASSIST_V3_REASON_LENGTH,i);
    if(!decode(xfer.staging,xfer.declared,generation,&configured,&candidate,&reason,&index))
        return abort_xfer(reason,index);
    configured=candidate;generation=assist_v3_generation_next(generation);cache_level=0U;
    xfer.live=false;persist_outcome=0U;return reply(ASSIST_V3_REPLY_ACK,0,0,source);
}
assist_v3_reply_t assist_v3_config_can_control(uint8_t source,uint8_t dlen,const uint8_t *data){
    uint8_t op,arg;
    if(source!=ASSIST_V3_SOURCE_TOOL)return reply(0,0,0,source);
    if(dlen!=4U)return reply(ASSIST_V3_REPLY_ERROR,ASSIST_V3_REASON_LENGTH,0,source);
    op=data[0];arg=data[1];
    if(op<1U||op>4U)return reply(ASSIST_V3_REPLY_ERROR,ASSIST_V3_REASON_RANGE,0,source);
    if((op==4U?(arg<1U||arg>6U):arg!=0U))return reply(ASSIST_V3_REPLY_ERROR,ASSIST_V3_REASON_RANGE,1,source);
    if(xfer.live)return reply(ASSIST_V3_REPLY_ERROR,ASSIST_V3_REASON_BUSY,ASSIST_V3_INDEX_NONE,source);
    if(op==1U){persist_expected=rd16(data+2);persist_pending=true;persist_outcome=0U;}
    else{
        assist_v3_values_t saved;uint16_t g;
        if(rd16(data+2)!=ASSIST_V3_UNSET&&rd16(data+2)!=generation)
            return reply(ASSIST_V3_REPLY_ERROR,ASSIST_V3_REASON_STALE,2,source);
        if(op==2U){unset(&saved);(void)scan(&saved,&g,0);configured=saved;}
        else if(op==3U)unset(&configured);
        else memset(configured.mode[arg-1U],0xFF,sizeof(configured.mode[0]));
        generation=assist_v3_generation_next(generation);cache_level=0U;persist_outcome=0U;
    }
    return reply(ASSIST_V3_REPLY_ACK,0,0,source);
}
assist_v3_reply_t assist_v3_config_service(uint32_t now_ms,bool standstill){
    if(xfer.live&&(uint32_t)(now_ms-xfer.last_ms)>ASSIST_V3_XFER_TIMEOUT_MS)
        return abort_xfer(ASSIST_V3_REASON_LENGTH,xfer.frames);
    if(persist_pending&&standstill&&!xfer.live){
        persist_pending=false;
        if(persist_expected!=ASSIST_V3_UNSET&&persist_expected!=generation)persist_outcome=ASSIST_V3_PERSIST_STALE;
        else persist_outcome=persist_now()?0U:ASSIST_V3_PERSIST_FAILED;
    }
    return reply(0,0,0,0);
}
