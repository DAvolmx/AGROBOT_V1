#pragma once
#include "Arduino.h"
#define WIFI_STA 1
#define WL_CONNECTED 3
struct WiFiT{ void mode(int){} bool config(IPAddress,IPAddress,IPAddress){return true;}
  void begin(const char*,const char*){} int status(){return WL_CONNECTED;} IPAddress localIP(){return IPAddress();} } WiFi;
