#pragma once
#include <math.h>
#include <stdint.h>
namespace agro {
// Transport scales only: no encoder, no measured m/s or rad/s.
constexpr float V_SCALE=.55f, W_SCALE=1.5f;
constexpr float MAX_DUTY=60.f, REVERSE_MAX_DUTY=35.f, INNER_REDUCTION=.65f;
inline float clip(float x,float a,float b){return x<a?a:x>b?b:x;}
constexpr bool newer(uint16_t n,uint16_t old){uint16_t d=n-old;return d && d<32768;}
constexpr uint8_t crc8(const uint8_t *p,unsigned n){uint8_t c=0;while(n--){c^=*p++;for(int b=0;b<8;b++)c=(c&128)?uint8_t((c<<1)^7):uint8_t(c<<1);}return c;}
static_assert(newer(0,65535),"wrap");
static_assert(!newer(20,20)&&!newer(19,20),"reject replay");
struct Mix {float v,w,duty[4];}; // R1 front right, L1 front left, L2 rear left, R2 rear right
// Limites CONFIGURABLES (1.2). Los valores por defecto son los de la 1.1, asi que
// sin configurar nada el comportamiento no cambia. fwd/rev/minp en % de PWM,
// inner en fraccion 0..1 (reduccion de la rueda interior en curva).
struct Limits {float fwd=MAX_DUTY,rev=REVERSE_MAX_DUTY,inner=INNER_REDUCTION,minp=0.f;};
inline Mix mix(float v,float w,float limit,const Limits &L=Limits{}){
 if(!isfinite(v)||!isfinite(w)||!isfinite(limit)||limit<=0)return {};
 v=clip(v,-fminf(limit,V_SCALE),fminf(limit,V_SCALE));
 if(fabsf(v)<.002f)return {}; // steering alone never powers wheels
 float base=fminf(100*fabsf(v)/V_SCALE,v<0?L.rev:L.fwd);
 float steer=v<0?0:clip(w/W_SCALE,-1,1); // reverse straight only
 float left=base*(1-L.inner*fmaxf(steer,0.f));
 float right=base*(1-L.inner*fmaxf(-steer,0.f));
 // Arranque minimo: un duty distinto de cero pero por debajo de `minp` no
 // vence la friccion estatica y el motor solo zumba. Se sube a `minp`.
 // minp=0 (defecto) lo desactiva. La velocidad media se baja con el troceado
 // de la Jetson, no con duties bajos.
 if(L.minp>0){
  if(left>0&&left<L.minp)left=L.minp;
  if(right>0&&right<L.minp)right=L.minp;
 }
 float sign=v<0?-1:1;
 return {sign*base*.01f*V_SCALE,steer*W_SCALE,{sign*right,sign*left,sign*left,sign*right}};
}
}
