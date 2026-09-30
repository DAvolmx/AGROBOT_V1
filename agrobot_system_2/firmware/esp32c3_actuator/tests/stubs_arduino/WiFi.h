#pragma once
#include "Arduino.h"
#define WIFI_STA 1
#define WL_CONNECTED 3
typedef enum { WIFI_POWER_19_5dBm = 78, WIFI_POWER_17dBm = 68, WIFI_POWER_13dBm = 52,
               WIFI_POWER_8_5dBm = 34 } wifi_power_t;
struct WiFiT{ void mode(int){} bool config(IPAddress,IPAddress,IPAddress){return true;}
  void begin(const char*,const char*){} int status(){return WL_CONNECTED;} IPAddress localIP(){return IPAddress();}
  IPAddress gatewayIP(){return IPAddress();} int RSSI(){return 0;} bool disconnect(){return true;}
  bool setHostname(const char*){return true;} bool setAutoReconnect(bool){return true;}
  bool setTxPower(wifi_power_t){return true;} } WiFi;
