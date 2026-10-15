#pragma once
#include <math.h>
#include <stdint.h>
namespace agro {
constexpr float L=.37f, BF=.143f, BR=.323f; // via = ancho exterior - neumático .047m ESTIMADO en simulador
constexpr float V_SCALE=.55f, W_SCALE=1.5f;
constexpr float K_MAX=.826299f; // tan(17 grados)/0.37; margen frente a tope mecanico 20 grados
inline float clip(float x,float a,float b){return x<a?a:x>b?b:x;}
constexpr bool newer(uint16_t n,uint16_t old){uint16_t d=n-old;return d && d<32768;}
constexpr uint8_t crc8(const uint8_t *p,unsigned n){uint8_t c=0;while(n--){c^=*p++;for(int b=0;b<8;b++)c=(c&128)?uint8_t((c<<1)^7):uint8_t(c<<1);}return c;}
static_assert(newer(0,65535),"sequence wrap must work");
static_assert(!newer(20,20),"duplicate rejected");
static_assert(!newer(19,20),"old packet rejected");
static_assert(!newer(32768,0),"ambiguous half-range rejected");
constexpr uint8_t GOLD_ZERO[]={0xA5,0,0,98,0,0,0};
constexpr uint8_t GOLD_MOVE[]={0xA5,50,0xEC,98,1,0x34,0x12};
static_assert(crc8(GOLD_ZERO,7)==248,"zero frame CRC Jetson");
static_assert(crc8(GOLD_MOVE,7)==26,"movement frame CRC Jetson");
struct Mix {float v,w,duty[4];}; // R1,L1,L2,R2
inline Mix mix(float v,float w,float limit){
 if(!isfinite(v)||!isfinite(w)||!isfinite(limit)||limit<=0)return {};
 v=clip(v,-limit,limit);
 if(fabsf(v)<.002f)return {}; // no giro sobre el sitio en conduccion
 w=clip(w,-fabsf(v)*K_MAX,fabsf(v)*K_MAX);
 const float delta=atanf(L*w/v); // REFERENCIA hipotetica, no medicion del pivote
 const float front=v/cosf(delta);
 float a[4]={front+w*BF*.5f,front-w*BF*.5f,v-w*BR*.5f,v+w*BR*.5f};
 float peak=0;for(float s:a)peak=fmaxf(peak,fabsf(s));
 float scale=peak>limit?limit/peak:1;
 Mix m={v*scale,w*scale,{}};
 for(int i=0;i<4;i++)m.duty[i]=100*a[i]*scale/V_SCALE;
 return m;
}
}
