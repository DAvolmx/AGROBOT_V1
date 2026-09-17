#pragma once
#include "Arduino.h"
struct WiFiUDP{ void begin(int){} int parsePacket(){return 0;} int read(uint8_t*,int){return 0;}
  void flush(){} IPAddress remoteIP(){return IPAddress();} uint16_t remotePort(){return 0;}
  void beginPacket(IPAddress,uint16_t){} void write(uint8_t*,int){} void endPacket(){} };
