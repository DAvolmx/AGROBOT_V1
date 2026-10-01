#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
typedef std::string String;
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define constrain(a,l,h) ((a)<(l)?(l):((a)>(h)?(h):(a)))
// El core de Arduino para ESP32 trae min/max; el firmware los usa desde el
// 2026-10-01 (traseros de apoyo, protección térmica).
#include <algorithm>
using std::min;
using std::max;
inline long map(long x,long a,long b,long c,long d){return (x-a)*(d-c)/(b-a)+c;}
// Reloj y pines INSTRUMENTADOS para comportamiento_test.cpp: el test mueve
// g_millis y lee lo que el firmware dejó en cada pin (nivel o PWM). Para la
// verificación de sintaxis da igual que hagan algo.
inline unsigned long g_millis = 0;
inline int  g_nivel[32] = {0};        // último digitalWrite
inline int  g_pwm[32] = {0};          // duty si el pin está en PWM, -1 si no
inline int  g_canal_pin[16] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
inline unsigned long millis(){return g_millis;}
inline void delay(unsigned long){}
inline void pinMode(int,int){}
inline void digitalWrite(int p,int v){ if(p>=0&&p<32){ g_nivel[p]=v; g_pwm[p]=-1; } }
// El firmware pasa por Serial.printf(), lo usa como bool (`if (!Serial)`) y
// abre un UART aparte para el mando (`MandoUart`). Los stubs se ampliaron el
// 2026-09-08: sin esto verificar.sh fallaba con 13 errores que NO eran del
// sketch, solo de la cascara.
#define SERIAL_8N1 0x800001c
struct SerialT{ SerialT(){} void begin(int){}
  void begin(unsigned long,int,int,int){}
  int  available(){return 0;} int read(){return -1;}
  size_t write(const uint8_t*,size_t){return 0;}
  void flush(){} void setRxBufferSize(size_t){}
  explicit operator bool() const { return true; }
  template<class... A> int printf(const char*,A...){return 0;}
  template<class T> void print(T){} template<class T> void println(T){} void println(){} };
SerialT Serial;
// UART1: el sketch declara el suyo (`HardwareSerial MandoUart(1)`), asi que
// aca solo hace falta el TIPO, no la instancia.
struct HardwareSerial : SerialT { HardwareSerial(int){} };
#define portMUX_TYPE int
#define portMUX_INITIALIZER_UNLOCKED 0
inline void portENTER_CRITICAL(int*){}
inline void portEXIT_CRITICAL(int*){}
inline bool ledcSetup(uint8_t,uint32_t,uint8_t){return true;}
inline bool ledcAttachPin(uint8_t,uint8_t){return true;}
inline void ledcDetachPin(uint8_t){}
inline bool ledcWrite(uint8_t,uint32_t){return true;}
inline bool ledcAttachChannel(uint8_t p,uint32_t,uint8_t,uint8_t c){ if(c<16) g_canal_pin[c]=p; if(p<32) g_pwm[p]=0; return true;}
inline bool ledcDetach(uint8_t p){ for(int c=0;c<16;c++) if(g_canal_pin[c]==p) g_canal_pin[c]=-1; if(p<32) g_pwm[p]=-1; return true;}
inline bool ledcWriteChannel(uint8_t c,uint32_t d){ if(c<16 && g_canal_pin[c]>=0) g_pwm[g_canal_pin[c]]=(int)d; return true;}
struct IPAddress{ IPAddress(){} IPAddress(int,int,int,int){}
  bool operator==(const IPAddress&) const {return true;} String toString() const {return String();} };
