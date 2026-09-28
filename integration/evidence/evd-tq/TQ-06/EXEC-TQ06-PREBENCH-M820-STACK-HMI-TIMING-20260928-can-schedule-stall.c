#include <stdio.h>
#include <stdlib.h>
#include "can_periodic_due.h"
static unsigned long fails=0, checks=0;
#define CK(c) do{ checks++; if(!(c)) fails++; }while(0)
int main(void){
  const unsigned ms[]={1,10,50,100,500,5000}; const uint32_t starts[]={0u,0x7FFFFF00u,0xFFFFF000u};
  for(unsigned w=0;w<3;w++) for(unsigned k=0;k<6;k++) for(int rf=0;rf<2;rf++){
    const uint32_t st=starts[w], P0=4000u*37u+123u, L=ms[k]*4u, T=4000u*600u; srand(w*100+k*10+rf);
    can_periodic_schedule_t s; can_periodic_init(&s,st);
    uint32_t last_offer=0; int have=0; uint32_t lastk[8]; uint32_t lastsent[8]; unsigned n[8]={0}; uint32_t maxgap=0;
    for(int i=0;i<8;i++){lastk[i]=0; lastsent[i]=0;}
    for(uint32_t e=0;e<T;e++){ uint32_t now=st+e;
      if(e>=P0 && e<P0+L) continue;
      int i=can_periodic_next_due(&s,now); if(i<0) continue;
      CK(!have || (uint32_t)(now-last_offer)>=4u); have=1; last_offer=now;
      if(rf && rand()%3==0) continue;                          /* refused: must stay due */
      CK(can_periodic_reached(now,s.next_due[i]));
      uint32_t P=can_periodic_periods[i];
      uint32_t kdue=(uint32_t)(s.next_due[i]-st)/P;            /* deadline index being served */
      uint32_t kcur=(uint32_t)(now-st)/P;                      /* latest reached index */
      CK(kdue>lastk[i]);                                        /* never the same deadline twice */
      if(n[i]){ uint32_t g=now-lastsent[i]; if(g>P && g-P>maxgap && !(lastsent[i]<st+P0 && e>=P0)) maxgap=g-P; }
      can_periodic_accepted(&s,(unsigned)i,now);
      CK((uint32_t)(s.next_due[i]-st)%P==0u);                   /* phase anchor */
      CK((uint32_t)(s.next_due[i]-st)/P==kcur+1u);             /* next = first deadline after now: latest only */
      lastk[i]=kcur; lastsent[i]=now; n[i]++; }
    printf("start=%08x stall=%4ums refuse=%d  worst steady jitter=%u ticks  sent:",st,ms[k],rf,maxgap);
    for(int i=0;i<8;i++) printf(" %u",n[i]);
    printf("\n");
  }
  printf("checks=%lu fails=%lu => %s\n",checks,fails,fails?"FAIL":"PASS"); return fails!=0; }
