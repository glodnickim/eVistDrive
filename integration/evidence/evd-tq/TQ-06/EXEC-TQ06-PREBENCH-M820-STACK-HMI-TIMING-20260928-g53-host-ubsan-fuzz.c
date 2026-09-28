#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "g53_port.h"
static uint64_t s=88172645463325252ull; static uint32_t r(void){s^=s<<13;s^=s>>7;s^=s<<17;return (uint32_t)s;}
int main(void){ uint64_t h=1469598103934665603ull; unsigned long n=0;
 for(int seed=0;seed<400;seed++){ g53_port_reset(); uint8_t idx=0; static const uint8_t f[4]={0,1,3,2};
  for(int c=0;c<1500;c++){ g53_port_input_t in; memset(&in,0,sizeof in);
   int m=seed%4;
   in.raw_pa6_adc=(uint16_t)(m==0? r(): r()%4096); in.load_ctrl=(uint16_t)r();
   if(m==0) in.pas_ab=(uint8_t)r(); else { if(r()%3==0) idx=(uint8_t)((idx+(r()%10?1:3))&3); in.pas_ab=f[idx]; }
   in.assist_level=(uint8_t)(m==0? r(): r()%10); in.speed_x100=(m==0? r(): r()%8000);
   in.elapsed_ticks=(m==0? r(): (r()%20? 4: r()%600)); in.phase_current_max=(int32_t)(m==0? r(): 1500);
   in.torque_sensor_valid=r()&1; in.direction_inhibit=(r()%50)==0; in.real_stop=(r()%50)==0; in.safety_cut=(r()%50)==0;
   g53_port_output_t o; g53_port_update(&in,&o); const uint8_t*p=(const uint8_t*)&o; for(size_t i=0;i<sizeof o;i++){h^=p[i];h*=1099511628211ull;} n++; }}
 printf("fuzz calls=%lu hash=%016llx\n",n,(unsigned long long)h); return 0; }
