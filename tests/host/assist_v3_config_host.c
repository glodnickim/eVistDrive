#include "assist_v3_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static int failures, erases, budget=-1;
static uint8_t flash_page[ASSIST_V3_FLASH_PAGE_SIZE];
#define CHECK(x,msg) do{if(!(x)){printf("FAIL: %s\n",msg);failures++;}}while(0)
static uint16_t rd16(const uint8_t *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static uint32_t rd32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void wr16(uint8_t *p,uint16_t x){p[0]=(uint8_t)x;p[1]=(uint8_t)(x>>8);}
static void wr32(uint8_t *p,uint32_t x){p[0]=(uint8_t)x;p[1]=(uint8_t)(x>>8);p[2]=(uint8_t)(x>>16);p[3]=(uint8_t)(x>>24);}
static bool erase_page(void){erases++;memset(flash_page,0xFF,sizeof(flash_page));return true;}
static bool program(uint32_t off,const uint8_t *p,uint32_t n){
    uint32_t i;if(off+n>sizeof(flash_page)||(off&3U)||(n&3U))return false;
    for(i=0;i<n;i+=4U){if(budget==0)return false;if(budget>0)budget--;flash_page[off+i]&=p[i];flash_page[off+i+1]&=p[i+1];flash_page[off+i+2]&=p[i+2];flash_page[off+i+3]&=p[i+3];}
    return true;
}
static const assist_v3_flash_t hal={flash_page,erase_page,program};
static void blank(void){memset(flash_page,0xFF,sizeof(flash_page));erases=0;budget=-1;assist_v3_config_init(&hal);}
static void restart(void){assist_v3_config_init(&hal);}
static void profile(uint8_t *b,uint8_t mode,uint8_t count){
    uint8_t p;uint16_t len=(uint16_t)(18U+2U*count);memset(b,0xFF,90U);
    b[0]='B';b[1]='V';b[2]=2U;b[3]=2U;wr16(b+4,len);
    b[6]=mode;b[7]=count;b[8]=2U;b[9]=0U;wr32(b+10,ASSIST_V3_CAPS);
    wr16(b+14,assist_v3_config_generation());
    for(p=0;p<count;p++)wr16(b+16U+2U*p,ASSIST_V3_KEEP);
    wr16(b+len-2U,assist_v3_crc16(b,(uint16_t)(len-2U)));
}
static void seal(uint8_t *b){uint16_t n=rd16(b+4);wr16(b+n-2U,assist_v3_crc16(b,(uint16_t)(n-2U)));}
static assist_v3_reply_t send(const uint8_t *b,uint8_t n){
    assist_v3_reply_t r;uint8_t f;
    r=assist_v3_config_can_declare(5U,n,100U);
    if(r.kind!=ASSIST_V3_REPLY_DECL_ACK)return r;
    for(f=0U;;f++){
        uint16_t off=(uint16_t)f*8U;bool end=off+8U>=n;
        r=assist_v3_config_can_frame(5U,f,end,b+off,(uint8_t)(end?(uint16_t)(n-off):8U),100U);
        if(end||r.kind!=ASSIST_V3_REPLY_NONE)break;
    }return r;
}
static assist_v3_reply_t control(uint8_t op,uint8_t arg,uint16_t gen){
    uint8_t b[4]={op,arg,(uint8_t)gen,(uint8_t)(gen>>8)};
    return assist_v3_config_can_control(5U,4U,b);
}
static uint8_t read_object(uint8_t obj,uint8_t idx,uint8_t view,uint8_t level,uint8_t *b){
    uint8_t req[4]={obj,idx,view,level},len=0U;
    assist_v3_reply_t r=assist_v3_config_can_read(5U,ASSIST_V3_CMD_BLOCK,4U,req,b,&len);
    CHECK(r.kind==ASSIST_V3_REPLY_NONE,"read accepted");
    CHECK(len==0U||rd16(b+len-2U)==assist_v3_crc16(b,(uint16_t)(len-2U)),"read CRC");
    return len;
}
static void reject(const char *name,uint8_t *b,uint8_t n,uint8_t reason){
    assist_v3_values_t before,after;uint16_t gen=assist_v3_config_generation();assist_v3_reply_t r;
    assist_v3_config_ram_values(&before);r=send(b,n);assist_v3_config_ram_values(&after);
    CHECK(r.kind==ASSIST_V3_REPLY_ERROR&&r.reason==reason,name);
    CHECK(gen==assist_v3_config_generation()&&memcmp(&before,&after,sizeof(before))==0,"reject is atomic");
}
static void vectors(void){
    uint8_t b[90],req[4]={0},len=0U;assist_v3_reply_t r;
    blank();
    (void)assist_v3_config_can_read(5U,ASSIST_V3_CMD_CAPS,0U,req,b,&len);
    CHECK(len==30U&&b[0]=='B'&&b[1]=='V'&&b[2]==2U&&b[3]==2U,"CAPS format 2");
    CHECK(b[4]==2U&&b[5]==2U&&b[6]==6U&&b[7]==12U&&rd16(b+8)==90U&&b[10]==24U,"CAPS schema/table");
    CHECK(rd32(b+12)==assist_v3_config_caps()&&rd32(b+16)==ASSIST_V3_PARAM_MASK,"CAPS masks");
    CHECK(rd16(b+26)==720U&&rd16(b+28)==assist_v3_crc16(b,28U),"CAPS power and CRC");
    CHECK(assist_v3_generation_next(0xFFFD)==0U&&assist_v3_generation_next(0xFFFF)==0U,"generation sentinels skipped");
    CHECK(read_object(2U,1U,2U,0U,b)==66U&&rd16(b+16)==50U&&rd16(b+18)==80U&&rd16(b+20)==350U,"profile default vector");
    CHECK(read_object(3U,0U,2U,0U,b)==34U&&rd16(b+16)==ASSIST_V3_ENGINE_V3&&b[18]==1U&&b[22]==5U,"global default vector");
    profile(b,1U,24U);wr16(b+16U+2U*3U,55U);seal(b);r=send(b,66U);
    CHECK(r.kind==ASSIST_V3_REPLY_ACK&&r.generation==2U,"profile ACK generation");
    CHECK(read_object(2U,1U,3U,0U,b)==66U&&rd16(b+22)==55U,"configured view");
}
static void rejects(void){
    uint8_t b[90];blank();profile(b,1U,24U);
    b[0]='X';reject("magic",b,66U,0U);b[0]='B';
    b[3]=9U;reject("schema",b,66U,0U);b[3]=2U;
    wr16(b+4,65U);reject("length",b,66U,1U);wr16(b+4,66U);
    b[9]=1U;reject("reserved flag",b,66U,7U);b[9]=0U;
    wr32(b+10,0x80000000UL);reject("required cap",b,66U,4U);wr32(b+10,ASSIST_V3_CAPS);
    wr16(b+14,9U);reject("stale write",b,66U,6U);wr16(b+14,1U);
    b[64]^=1U;reject("CRC",b,66U,2U);b[64]^=1U;
    wr16(b+16U+2U*1U,9U);seal(b);reject("torque range",b,66U,3U);
    wr16(b+16U+2U*1U,ASSIST_V3_KEEP);
    wr16(b+16U+2U*6U,0U);seal(b);reject("reserved parameter",b,66U,4U);
    wr16(b+16U+2U*6U,ASSIST_V3_KEEP);
    wr16(b+16U+2U*9U,100U);seal(b);reject("inapplicable AUTO field",b,66U,4U);
    wr16(b+16U+2U*9U,ASSIST_V3_KEEP);
    wr16(b+16U+2U*10U,500U);seal(b);reject("range_max needs progression",b,66U,4U);
    b[6]=7U;seal(b);reject("mode index",b,66U,3U);
}
static void keep_and_resolution(void){
    uint8_t b[90],caps[90],n=0U;assist_v3_reply_t r;const assist_v3_effective_t *e;
    blank();assist_v3_config_set_legacy(1U,190U,50U,8U);
    e=assist_v3_effective(1U);CHECK(e&&e->source[0]==2U&&e->source[2]==2U&&e->source[12]==2U,"per-level legacy sources");
    CHECK(e->value[2]<=350U&&e->source[13]==1U,"P1 percentage and acceleration exception");
    assist_v3_config_set_pack_voltage_mv(54000U);
    assist_v3_config_set_legacy(2U,215U,100U,5U);
    e=assist_v3_effective(2U);CHECK(e->value[2]==810U&&e->source[2]==0U,"HW-max default tracks pack voltage");
    assist_v3_config_set_legacy(2U,215U,50U,5U);
    e=assist_v3_effective(2U);CHECK(e->value[2]==405U&&e->source[2]==2U,"P1 scales hardware percentage");
    profile(b,2U,3U);wr16(b+20U,65000U);seal(b);
    CHECK(send(b,24U).kind==ASSIST_V3_REPLY_ACK,"hardware maximum override accepted");
    e=assist_v3_effective(2U);CHECK(e->value[2]==810U&&e->source[2]==4U,"firmware-limited source only when capped");
    (void)control(4U,2U,ASSIST_V3_UNSET);
    profile(b,1U,14U);wr16(b+16U+2U*0U,70U);wr16(b+16U+2U*3U,65U);
    wr16(b+16U+2U*8U,300U);wr16(b+16U+2U*12U,33U);seal(b);
    r=send(b,46U);CHECK(r.kind==ASSIST_V3_REPLY_ACK,"short count + KEEP");
    e=assist_v3_effective(1U);CHECK(e->value[0]==70U&&e->source[0]==6U&&e->value[8]==300U&&e->source[8]==3U,"override and shadowed source");
    CHECK(e->value[12]==33U&&e->source[12]==3U&&e->value[13]==65U,"attack exception shadowed, release macro");
    CHECK(assist_v3_effective_release_pct(1U)==65U,"Milestone C release resolver signature");
    assist_v3_config_set_legacy(1U,95U,100U,4U);
    e=assist_v3_effective(1U);CHECK(e->value[0]==70U&&e->value[8]==300U,"P0 write retains V3 overrides");
    (void)assist_v3_config_can_read(5U,ASSIST_V3_CMD_CAPS,0U,caps,caps,&n);
    CHECK((caps[11]&1U)!=0U,"legacy shadowed mask");
    profile(b,4U,12U);wr16(b+16U+2U*11U,0U);seal(b);
    r=send(b,42U);CHECK(r.kind==ASSIST_V3_REPLY_ACK,"progression can become zero");
    CHECK(read_object(2U,4U,1U,4U,b)==90U&&b[64U+10U]==5U,"dormant range max source 5");
    blank();
    {const uint16_t anchors[6]={95U,215U,310U,525U,525U,215U};
     for(uint8_t m=1U;m<=6U;m++){
         CHECK(read_object(2U,m,1U,1U,b)==90U,"candidate profile effective read");
         CHECK(rd16(b+16U+2U*(m==4U?10U:8U))==anchors[m-1U],"Assist 50 candidate legacy anchor");
     }}
    for(uint8_t m=1U;m<=6U;m++)for(uint8_t a=0U;a<=100U;a+=50U){
        profile(b,m,1U);wr16(b+16U,a);seal(b);r=send(b,20U);
        CHECK(r.kind==ASSIST_V3_REPLY_ACK,"Assist 0/50/100 write");
        for(uint8_t l=1U;l<=5U;l++){
            const assist_v3_effective_t *v=assist_v3_effective(l);
            CHECK(v&&v->value[0]<=100U&&v->value[1]>=10U&&v->value[1]<=100U&&v->value[2]<=65000U,"effective basic ranges");
        }
    }
}
static void storage_and_control(void){
    uint8_t b[90];assist_v3_reply_t r;unsigned i;
    blank();profile(b,1U,4U);wr16(b+22,35U);seal(b);CHECK(send(b,26U).kind==ASSIST_V3_REPLY_ACK,"first object");
    profile(b,2U,4U);wr16(b+22,45U);seal(b);CHECK(send(b,26U).kind==ASSIST_V3_REPLY_ACK,"second object");
    r=control(1U,0U,assist_v3_config_generation());CHECK(r.kind==ASSIST_V3_REPLY_ACK,"persist queued");
    CHECK(assist_v3_config_persist_state()==2U,"pending state");
    (void)assist_v3_config_service(100U,false);CHECK(flash_page[0]==0xFFU,"moving: flash untouched");
    (void)assist_v3_config_service(101U,true);CHECK(flash_page[0]=='A'&&assist_v3_config_persist_state()==0U,"standstill persist");
    restart();CHECK(read_object(2U,1U,0U,0U,b)==66U&&rd16(b+22)==35U,"saved view from log");
    CHECK(read_object(2U,2U,3U,0U,b)==66U&&rd16(b+22)==45U,"multiple objects survive restart");
    profile(b,1U,4U);wr16(b+22,66U);seal(b);CHECK(send(b,26U).kind==ASSIST_V3_REPLY_ACK,"change after saved");
    CHECK(control(2U,0U,ASSIST_V3_UNSET).kind==ASSIST_V3_REPLY_ACK,"revert");
    CHECK(read_object(2U,1U,3U,0U,b)==66U&&rd16(b+22)==35U,"revert reads flash");
    CHECK(control(4U,1U,ASSIST_V3_UNSET).kind==ASSIST_V3_REPLY_ACK,"restore one mode");
    CHECK(read_object(2U,1U,3U,0U,b)==66U&&rd16(b+22)==ASSIST_V3_UNSET,"one mode cleared");
    CHECK(read_object(2U,2U,3U,0U,b)==66U&&rd16(b+22)==45U,"other mode retained");
    CHECK(control(3U,0U,ASSIST_V3_UNSET).kind==ASSIST_V3_REPLY_ACK,"restore all");
    CHECK(read_object(2U,2U,3U,0U,b)==66U&&rd16(b+22)==ASSIST_V3_UNSET,"all overrides cleared");
    /* The expected generation is checked at standstill, after an intervening write. */
    {uint16_t expected=assist_v3_config_generation();CHECK(control(1U,0U,expected).kind==ASSIST_V3_REPLY_ACK,"queued expected generation");
     profile(b,1U,4U);wr16(b+22,77U);seal(b);CHECK(send(b,26U).kind==ASSIST_V3_REPLY_ACK,"intervening write");
     (void)assist_v3_config_service(200U,true);CHECK(assist_v3_config_persist_state()==4U,"stale deferred persist outcome");}
    for(i=0U;i<ASSIST_V3_SLOT_COUNT;i++){
        profile(b,1U,4U);wr16(b+22,(uint16_t)(10U+i));seal(b);CHECK(send(b,26U).kind==ASSIST_V3_REPLY_ACK,"append object");
        (void)control(1U,0U,ASSIST_V3_UNSET);(void)assist_v3_config_service(300U+i,true);
    }
    CHECK(erases==1,"erase only after six full slots");
    restart();CHECK(read_object(2U,1U,0U,0U,b)==66U&&rd16(b+22)==15U,"newest CRC-valid slot");
    /* Interrupted append leaves the prior slot valid. */
    profile(b,1U,4U);wr16(b+22,88U);seal(b);(void)send(b,26U);budget=2;
    (void)control(1U,0U,ASSIST_V3_UNSET);(void)assist_v3_config_service(400U,true);budget=-1;
    CHECK(assist_v3_config_persist_state()==3U,"flash program failure visible");
    restart();CHECK(read_object(2U,1U,0U,0U,b)==66U&&rd16(b+22)==15U,"interrupted append keeps previous");
    /* v1 record is ignored without erasing it. */
    blank();flash_page[0]='A';flash_page[1]='3';flash_page[2]='C';flash_page[3]='F';flash_page[4]=1U;
    wr32(flash_page+124U,assist_v3_crc32(flash_page,124U));
    restart();CHECK(assist_v3_config_flash_record_state()==4U&&flash_page[0]=='A',"v1 record ignored, untouched");
    /* Erase-window failure produces absent defaults, never a mixed image. */
    blank();for(i=0U;i<ASSIST_V3_SLOT_COUNT;i++){
        profile(b,1U,4U);wr16(b+22,(uint16_t)(20U+i));seal(b);(void)send(b,26U);
        (void)control(1U,0U,ASSIST_V3_UNSET);(void)assist_v3_config_service(500U+i,true);
    }
    profile(b,1U,4U);wr16(b+22,99U);seal(b);(void)send(b,26U);budget=0;
    (void)control(1U,0U,ASSIST_V3_UNSET);(void)assist_v3_config_service(600U,true);budget=-1;
    restart();CHECK(assist_v3_config_flash_record_state()==1U&&read_object(2U,1U,3U,0U,b)==66U&&rd16(b+22)==ASSIST_V3_UNSET,"erase window resets to defaults");
}
static void transfer(void){
    uint8_t b[90];assist_v3_reply_t r;blank();profile(b,1U,24U);
    r=assist_v3_config_can_declare(5U,66U,100U);CHECK(r.kind==ASSIST_V3_REPLY_DECL_ACK,"declaration ACK");
    CHECK(assist_v3_config_can_declare(5U,66U,100U).reason==5U,"live transfer busy");
    CHECK(assist_v3_config_can_frame(3U,0U,false,b,8U,100U).kind==ASSIST_V3_REPLY_NONE,"foreign frame ignored");
    CHECK(!assist_v3_config_other_declaration(3U,&r),"foreign declaration cannot steal owner");
    r=assist_v3_config_service(1102U,false);CHECK(r.kind==ASSIST_V3_REPLY_ERROR&&r.reason==1U&&!assist_v3_config_xfer_live(),"timeout abort");
    r=assist_v3_config_can_declare(5U,66U,100U);CHECK(r.kind==ASSIST_V3_REPLY_DECL_ACK,"redeclaration");
    (void)assist_v3_config_can_frame(5U,0U,false,b,8U,100U);
    r=assist_v3_config_can_frame(5U,0U,false,b,8U,100U);CHECK(r.kind==ASSIST_V3_REPLY_ERROR&&r.reason==1U,"duplicate frame abort");
    CHECK(control(4U,1U,ASSIST_V3_UNSET).kind==ASSIST_V3_REPLY_ACK,"control after abort");
    CHECK(assist_v3_config_can_declare(5U,99U,100U).reason==1U,"oversized declaration");
}
static void global_and_no_v3(void){
    uint8_t b[ASSIST_V3_GLOBAL_LEN],caps[ASSIST_V3_CAPS_LEN],n=0U;
    assist_v3_values_t v;assist_v3_reply_t r;blank();assist_v3_config_ram_values(&v);
    CHECK(!assist_v3_config_engine_active(),"engine active truthful at boot");
    (void)assist_v3_config_can_read(5U,ASSIST_V3_CMD_CAPS,0U,caps,caps,&n);
    CHECK(caps[24]==0U&&caps[25]==(assist_v3_config_caps()&1U),"CAPS engine bytes");
    v.engine=1U;assist_v3_block_encode(&v,assist_v3_config_caps(),assist_v3_config_generation(),b);
    r=send(b,34U);
#if ASSIST_V3
    CHECK(r.kind==ASSIST_V3_REPLY_ACK,"V3 engine accepted");
    assist_v3_config_set_engine_active(true);
    (void)assist_v3_config_can_read(5U,ASSIST_V3_CMD_CAPS,0U,caps,caps,&n);
    CHECK(caps[24]==1U,"pipeline setter owns active readback");
#else
    CHECK(r.kind==ASSIST_V3_REPLY_ERROR&&r.reason==4U,"no-V3 image rejects V3 engine");
    CHECK((assist_v3_config_caps()&1U)==0U,"no-V3 caps bit 0 clear");
#endif
    v.engine=0U;assist_v3_block_encode(&v,assist_v3_config_caps(),assist_v3_config_generation(),b);
    CHECK(send(b,34U).kind==ASSIST_V3_REPLY_ACK,"G5300 engine accepted");
}
static char *file_text(const char *path){
    FILE *f=fopen(path,"rb");long n;char *s;if(!f)return NULL;fseek(f,0,SEEK_END);n=ftell(f);fseek(f,0,SEEK_SET);
    s=(char*)malloc((size_t)n+1U);if(!s){fclose(f);return NULL;}s[fread(s,1,(size_t)n,f)]='\0';fclose(f);return s;
}
#define STR2(x) #x
#define STR(x) STR2(x)
static void can_guard(void){
    char *s=file_text(STR(CAN_DISPLAY_C_PATH));CHECK(s!=NULL,"CAN_Display text available");if(!s)return;
    CHECK(strstr(s,"append_multiframe(0, &Para0[0]);")!=NULL,"legacy P0 transfer");
    CHECK(strstr(s,"append_multiframe(Ext_ID_Rx.command+1, &Para1[0]);")!=NULL,"legacy P1 transfer");
    CHECK(strstr(s,"append_multiframe(Ext_ID_Rx.command+1, &Para2[0]);")!=NULL,"legacy P2 transfer");
    CHECK(strstr(s,"(char*)&BankBlob[0]")!=NULL&&strstr(s,"(char*)&TuningBlob[0]")!=NULL,"bank and tuning transfer");
    CHECK(strstr(s,"sendAssistV3Result")!=NULL&&strstr(s,"!assist_v3_config_owns_command(Ext_ID_Rx.command)")!=NULL,"V3 result isolated");
    free(s);
}
int main(void){
#if !ASSIST_V3
    if(0){vectors();rejects();keep_and_resolution();storage_and_control();transfer();}
    global_and_no_v3();can_guard();
#else
    vectors();rejects();keep_and_resolution();storage_and_control();transfer();global_and_no_v3();can_guard();
#endif
    if(failures){printf("assist_v3_config v2: %d FAILED\n",failures);return 1;}
    puts("assist_v3_config v2: PASS");return 0;
}
