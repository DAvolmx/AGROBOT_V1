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
inline long map(long x,long a,long b,long c,long d){return (x-a)*(d-c)/(b-a)+c;}
inline unsigned long millis(){return 0;}
inline void delay(unsigned long){}
inline void pinMode(int,int){}
inline void digitalWrite(int,int){}
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
inline bool ledcAttachChannel(uint8_t,uint32_t,uint8_t,uint8_t){return true;}
inline bool ledcDetach(uint8_t){return true;}
inline bool ledcWriteChannel(uint8_t,uint32_t){return true;}
struct IPAddress{ IPAddress(){} IPAddress(int,int,int,int){}
  bool operator==(const IPAddress&) const {return true;} String toString() const {return String();} };
