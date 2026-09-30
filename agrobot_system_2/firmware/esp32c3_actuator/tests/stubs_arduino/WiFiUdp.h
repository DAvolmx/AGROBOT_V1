#pragma once
#include "Arduino.h"
struct WiFiUDP{ uint8_t begin(uint16_t){return 1;} void stop(){} int parsePacket(){return 0;} int read(uint8_t*,size_t){return 0;}
  void flush(){} IPAddress remoteIP(){return IPAddress();} uint16_t remotePort(){return 0;}
  int beginPacket(IPAddress,uint16_t){return 1;} size_t write(const uint8_t*,size_t){return 0;} int endPacket(){return 1;} };
