// Prueba en PC (sin hardware) de control_model.h de la 1.2.
// Compilar:  g++ -std=c++17 -Wall -Wextra -I../agrobot_c3_demo_trasero_1_2 test_mix.cpp -o test_mix && ./test_mix
// OJO: NO ponerlo dentro de la carpeta del sketch: Arduino compila todo .cpp
// que haya ahi y lo meteria en el firmware.
#include <cstdio>
#include <cmath>
#include "control_model.h"
using namespace agro;
static int fallos=0,total=0;
#define CHECK(c) do{total++;if(!(c)){fallos++;printf("FALLO linea %d: %s\n",__LINE__,#c);}}while(0)
static bool near(float a,float b,float e=1e-3f){return fabsf(a-b)<e;}

int main(){
 // --- por defecto = comportamiento de la 1.1 ---
 { Mix m=mix(.55f,0,.55f);
   CHECK(near(m.duty[0],60)&&near(m.duty[1],60)&&near(m.duty[2],60)&&near(m.duty[3],60)); }
 { Mix m=mix(-.55f,0,.55f);
   for(int i=0;i<4;i++)CHECK(near(m.duty[i],-35)); }         // atras: -35 % y recto
 { Mix m=mix(-.55f,1.5f,.55f);                                 // atras con giro: sin curva
   CHECK(near(m.duty[0],m.duty[1])&&near(m.w,0)); }
 // --- giro sin avance NUNCA mueve ruedas ---
 { Mix m=mix(0,1.5f,.55f); for(int i=0;i<4;i++)CHECK(m.duty[i]==0); }
 { Mix m=mix(.001f,1.5f,.55f); for(int i=0;i<4;i++)CHECK(m.duty[i]==0); }
 // --- nunca contrarrotacion: en un lado no hay signos opuestos ---
 for(float v=-.55f;v<=.551f;v+=.05f)for(float w=-1.5f;w<=1.501f;w+=.25f){
   Mix m=mix(v,w,.55f);
   for(int i=0;i<4;i++)for(int j=0;j<4;j++)CHECK(m.duty[i]*m.duty[j]>=0);
 }
 // --- curva izquierda: lado izquierdo (L1=1,L2=2) mas lento que el derecho ---
 { Mix m=mix(.55f,1.5f,.55f);
   CHECK(m.duty[1]<m.duty[0]&&m.duty[2]<m.duty[3]);
   CHECK(near(m.duty[1],60*(1-.65f))); CHECK(near(m.duty[0],60)); }
 { Mix m=mix(.55f,-1.5f,.55f);
   CHECK(m.duty[0]<m.duty[1]&&m.duty[3]<m.duty[2]); }
 // --- entradas invalidas -> todo a cero ---
 { Mix m=mix(NAN,0,.55f); for(int i=0;i<4;i++)CHECK(m.duty[i]==0); }
 { Mix m=mix(.3f,INFINITY,.55f); for(int i=0;i<4;i++)CHECK(m.duty[i]==0); }
 { Mix m=mix(.3f,0,0.f); for(int i=0;i<4;i++)CHECK(m.duty[i]==0); }
 // --- limites configurables ---
 { Limits L; L.fwd=100;  Mix m=mix(.55f,0,.55f,L); CHECK(near(m.duty[0],100)); }
 { Limits L; L.fwd=30;   Mix m=mix(.55f,0,.55f,L); CHECK(near(m.duty[0],30)); }
 { Limits L; L.rev=20;   Mix m=mix(-.55f,0,.55f,L); CHECK(near(m.duty[0],-20)); }
 { Limits L; L.inner=0;  Mix m=mix(.55f,1.5f,.55f,L); CHECK(near(m.duty[1],m.duty[0])); }
 { Limits L; L.inner=.9f;Mix m=mix(.55f,1.5f,.55f,L); CHECK(near(m.duty[1],60*(1-.9f))); }
 // --- arranque minimo ---
 { Limits L; L.minp=40; Mix m=mix(.15f,0,.55f,L);          // ~27 % pedido -> sube a 40
   for(int i=0;i<4;i++)CHECK(near(m.duty[i],40)); }
 { Limits L; L.minp=40; Mix m=mix(.55f,0,.55f,L);          // 60 % no se toca
   for(int i=0;i<4;i++)CHECK(near(m.duty[i],60)); }
 { Limits L; L.minp=40; Mix m=mix(0,0,.55f,L);             // parado SIGUE parado
   for(int i=0;i<4;i++)CHECK(m.duty[i]==0); }
 { Limits L; L.minp=40; Mix m=mix(-.15f,0,.55f,L);         // marcha atras: sube con signo -
   for(int i=0;i<4;i++)CHECK(near(m.duty[i],-40)); }
 { Limits L; L.minp=0;  Mix m=mix(.15f,0,.55f,L);          // desactivado = sin cambio
   CHECK(near(m.duty[0],100*.15f/.55f)); }
 // el arranque minimo no puede romper la regla de no-contrarrotacion
 { Limits L; L.minp=50; for(float v=.05f;v<=.55f;v+=.05f)for(float w=-1.5f;w<=1.5f;w+=.5f){
     Mix m=mix(v,w,.55f,L); for(int i=0;i<4;i++)for(int j=0;j<4;j++)CHECK(m.duty[i]*m.duty[j]>=0);} }
 // --- seguridad numerica: nunca fuera de [-100,100] ---
 { Limits L; L.fwd=100;L.rev=100;L.minp=60;
   for(float v=-1;v<=1;v+=.1f)for(float w=-3;w<=3;w+=.5f){Mix m=mix(v,w,.55f,L);
     for(int i=0;i<4;i++)CHECK(fabsf(m.duty[i])<=100.001f);} }
 // --- anti-repeticion de secuencia y CRC ---
 CHECK(newer(1,0)); CHECK(newer(0,65535)); CHECK(!newer(5,5)); CHECK(!newer(4,5));
 { uint8_t t[7]={0xA5,50,0,98,1,0x34,0x12}; uint8_t c=crc8(t,7); t[0]^=1; CHECK(crc8(t,7)!=c); }
 printf("%d comprobaciones, %d fallos\n",total,fallos);
 return fallos?1:0;
}
