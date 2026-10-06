#include "m560_stubs/main.h"
#include "parser.h"
#include <assert.h>
#include <stdio.h>

uint8_t Para0[64],Para1[64],Para2[64];
uint16_t k;
static unsigned writes;
static uint8_t applied_accel[10],applied_power[10];
static uint16_t applied_ratio[10];

void write_virtual_eeprom(void) { writes++; }
void update_checksum(void) {
    unsigned sum=0;
    Para0[63]=0;
    for(unsigned i=0;i<63;i++) sum+=Para1[i];
    Para1[63]=(uint8_t)sum;
}
void g53_port_set_levels(const uint8_t accel[10],const uint16_t ratio[10],const uint8_t power[10]) {
    memcpy(applied_accel,accel,sizeof(applied_accel));
    memcpy(applied_ratio,ratio,sizeof(applied_ratio));
    memcpy(applied_power,power,sizeof(applied_power));
}
void g53_chain_get_auto(uint8_t *enable,uint16_t *scale,uint8_t *rise_step) {
    *enable=1; *scale=2; *rise_step=10;
}

int main(void)
{
    static const uint8_t slot[5]={2,4,6,8,9};
    static const uint8_t accel[5]={8,1,7,2,4};
    static const uint16_t ratio[5]={1000,850,0,333,700};
    static const uint8_t power[5]={90,80,60,30,10};
    MotorParams_t mp={0};
    InitEEPROM(&mp);
    assert(writes==1);
    parse_MOparams(&mp);
    assert(Para0[51]==2 && Para0[52]==0 && Para0[53]==1 && Para0[54]==10);
    assert(Para0[1]==4 && Para0[3]==5 && Para0[5]==6 && Para0[7]==7);
    assert(Para1[40]==15 && Para1[42]==30 && Para1[44]==60 && Para1[46]==70);
    assert(Para0[28]==232 && Para0[29]==3 && Para0[63]==0);
    for(unsigned i=0;i<5;i++) {
        unsigned s=slot[i];
        Para0[s]=accel[i];
        Para0[10+2*(s-1)]=(uint8_t)ratio[i];
        Para0[11+2*(s-1)]=(uint8_t)(ratio[i]>>8);
        Para1[39+s]=power[i];
    }
    Para0[1]=8; Para0[3]=8; Para0[5]=8; Para0[7]=8;
    Para1[40]=99; Para1[42]=99; Para1[44]=99; Para1[46]=99;
    Para0[28]=0; Para0[29]=0; Para0[63]=99;
    Para0[51]=0; Para0[52]=255; Para0[53]=0; Para0[54]=0;
    parse_DPparams(&mp);
    apply_assist_levels(&mp);
    for(unsigned i=0;i<5;i++) {
        unsigned s=slot[i];
        assert(mp.assist_settings[i+1][2]==accel[i]);
        assert(mp.TQO_threshold[i+1]==ratio[i]);
        assert(mp.assist_settings[i+1][0]==power[i]);
        assert(applied_accel[s]==accel[i] && applied_ratio[s]==ratio[i]);
        assert(applied_power[s]==power[i]);
    }
    assert(applied_accel[1]==4 && applied_ratio[1]==45);
    assert(mp.assist_settings[1][0]>mp.assist_settings[5][0]);
    parse_MOparams(&mp);
    assert(Para0[51]==2 && Para0[52]==0 && Para0[53]==1 && Para0[54]==10);
    assert(Para0[1]==4 && Para0[3]==5 && Para0[5]==6 && Para0[7]==7);
    assert(Para1[40]==15 && Para1[42]==30 && Para1[44]==60 && Para1[46]==70);
    assert(Para0[28]==232 && Para0[29]==3 && Para0[63]==0);
    for(unsigned i=0;i<5;i++) {
        unsigned s=slot[i];
        assert(Para0[s]==accel[i]);
        assert((unsigned)(Para0[10+2*(s-1)]|(Para0[11+2*(s-1)]<<8))==ratio[i]);
        assert(Para1[39+s]==power[i]);
    }
    Para0[2]=0; Para0[4]=255; Para0[12]=255; Para0[13]=255;
    Para1[41]=255;
    parse_DPparams(&mp);
    assert(mp.assist_settings[1][2]==1 && mp.assist_settings[2][2]==8);
    assert(mp.TQO_threshold[1]==1000 && mp.assist_settings[1][0]==20);
    mp.assist_levels_magic=0;
    mp.assist_settings[1][2]=7;
    mp.TQO_threshold[1]=3200;
    mp.assist_settings[1][0]=73;
    mp.assist_settings[1][1]=46;
    unsigned before=writes;
    parse_MOparams(&mp);
    assert(writes==before+1);
    assert(mp.assist_settings[1][2]==4 && mp.TQO_threshold[1]==95);
    assert(mp.assist_settings[2][2]==5 && mp.TQO_threshold[2]==215);
    assert(mp.assist_settings[3][2]==6 && mp.TQO_threshold[3]==310);
    assert(mp.assist_settings[4][2]==8 && mp.TQO_threshold[4]==525);
    assert(mp.assist_settings[5][2]==8 && mp.TQO_threshold[5]==525);
    assert(mp.assist_settings[1][0]==73 && mp.assist_settings[1][1]==46);
    assert(mp.assist_settings[2][0]==80 && mp.assist_settings[2][1]==100);
    assert(mp.assist_levels_magic==0xA560U);
    puts("M560 parser roundtrip, clipping, migration and P0 AUTO READ/WRITE: PASS");
    return 0;
}
