#pragma once
#include "Arduino.h"
// Cáscaras de la API BLE del core ESP32 (actualizadas 2026-09-30).
#define ESP_PWR_LVL_P9 9
struct BLE2902{};
struct BLECharacteristic{ static const int PROPERTY_WRITE=1,PROPERTY_WRITE_NR=2,PROPERTY_READ=4,PROPERTY_NOTIFY=8;
  uint8_t* getData(){return nullptr;} size_t getLength(){return 0;} String getValue(){return String();}
  void setCallbacks(void*){} void addDescriptor(BLE2902*){} void setValue(uint8_t*,int){} void notify(){} };
struct BLECharacteristicCallbacks{ virtual void onWrite(BLECharacteristic*){} virtual ~BLECharacteristicCallbacks(){} };
struct BLEService{ BLECharacteristic* createCharacteristic(const char*,int){return new BLECharacteristic();} void start(){} };
struct BLEServer;
struct BLEServerCallbacks{ virtual void onConnect(BLEServer*){} virtual void onDisconnect(BLEServer*){} virtual ~BLEServerCallbacks(){} };
struct BLEServer{ void setCallbacks(BLEServerCallbacks*){} BLEService* createService(const char*){return new BLEService();}
  void startAdvertising(){} void requestConnParams(uint16_t,uint16_t,uint16_t,uint16_t,uint16_t){} };
struct BLEAdvertising{ void addServiceUUID(const char*){} void setScanResponse(bool){}
  void setMinPreferred(int){} void setMaxPreferred(int){} };
struct BLEDeviceT{ static void init(const char*){} static BLEServer* createServer(){return new BLEServer();}
  static void setPower(int){}
  static BLEAdvertising* getAdvertising(){return new BLEAdvertising();} static void startAdvertising(){} };
#define BLEDevice BLEDeviceT
