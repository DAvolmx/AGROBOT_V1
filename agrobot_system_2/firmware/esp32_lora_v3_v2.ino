/*
 * AGROBOT CARRITO - OTAA + SENSORES v2 (MEJORAS 2026-10-06)
 * =========================================================
 * - UDP separado: 20 Hz independiente del ciclo LoRaWAN
 * - Mayor robustez en I2C y WiFi
 * - Mejor logging de estado
 */
#include "LoRaWan_APP.h"
#include "esp_system.h"
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// =============================================
// ===== WIFI / UDP ROBUSTO =====
// =============================================
#define WIFI_SSID "JETSON_AP"
#define WIFI_PASSWORD "12345678"
#define JETSON_IP "192.168.12.1"
#define JETSON_UDP_PORT 5001
WiFiUDP udp;
bool wifi_connected = false;
unsigned long ultimoWifiCheck = 0;
const unsigned long WIFI_CHECK_MS = 10000;
uint32_t udpEnviados = 0, udpFallidos = 0;

// =============================================
// ===== CONFIGURACIÓN I2C =====
// =============================================
#define I2C_SDA 41
#define I2C_SCL 42
#define MPU6050_ADDR 0x68
#define SCD40_ADDR 0x62

// =============================================
// ===== CREDENCIALES OTAA =====
// =============================================
uint8_t devEui[] = { 0xf5, 0x9d, 0x08, 0x34, 0x59, 0x85, 0xb2, 0x7d };
uint8_t appEui[] = { 0x90, 0xe3, 0x84, 0xcc, 0x4d, 0x0d, 0x35, 0x02 };
uint8_t appKey[] = {
  0x6e, 0xc3, 0xb2, 0x10, 0xea, 0xc7, 0x1f, 0x74,
  0xb0, 0x08, 0x2c, 0xa9, 0xc1, 0xd6, 0xaa, 0xd2
};

uint8_t nwkSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint32_t devAddr = 0x00000000;

// =============================================
// ===== CONFIGURACIÓN LORAWAN =====
// =============================================
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t loraWanClass = CLASS_C;
uint32_t appTxDutyCycle = 30000;
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = false;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

// =============================================
// ===== VARIABLES GLOBALES =====
// =============================================
int sendCount = 0;
uint32_t udpSeq = 0;

// ⭐ NUEVO: ritmos independientes
const unsigned long UDP_PERIODO_MS      = 50;   // 20 Hz
const unsigned long AMBIENTE_PERIODO_MS = 5000; // 5 seg
unsigned long lastUdpMillis = 0;
unsigned long lastAmbienteMillis = 0;
unsigned long lastJoinAttempt = 0;
unsigned long lastSensorLog = 0;

RTC_DATA_ATTR bool joined = false;
bool mpu_ok = false, scd40_ok = false;

struct SensorData {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  uint16_t co2;
  float temperature_scd;
  float humidity;
} sensorData = {0, 0, 0, 0, 0, 0, 0, 0.0, 0.0};

// =============================================
// ===== FUNCIONES I2C BÁSICAS =====
// =============================================
void i2c_write(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t i2c_read(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.read();
}

void i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, len);
  for(uint8_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }
}

// =============================================
// ===== MPU6050 =====
// =============================================
void mpu6050_init() {
  Serial.print("  📌 MPU6050 (I2C)... ");
  i2c_write(MPU6050_ADDR, 0x6B, 0x00);
  delay(100);
  uint8_t whoami = i2c_read(MPU6050_ADDR, 0x75);
  if(whoami == 0x68) {
    Serial.println("✅");
    mpu_ok = true;
  } else {
    Serial.print("❌ (0x");
    Serial.print(whoami, HEX);
    Serial.println(")");
    mpu_ok = false;
  }
}

void mpu6050_read(float* accelX, float* accelY, float* accelZ,
                   float* gyroX, float* gyroY, float* gyroZ) {
  uint8_t data[14];
  if(mpu_ok) {
    i2c_read_bytes(MPU6050_ADDR, 0x3B, data, 14);
    int16_t ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t az = (int16_t)((data[4] << 8) | data[5]);
    *accelX = ax / 16384.0;
    *accelY = ay / 16384.0;
    *accelZ = az / 16384.0;
    int16_t gx = (int16_t)((data[8] << 8) | data[9]);
    int16_t gy = (int16_t)((data[10] << 8) | data[11]);
    int16_t gz = (int16_t)((data[12] << 8) | data[13]);
    *gyroX = gx / 131.0;
    *gyroY = gy / 131.0;
    *gyroZ = gz / 131.0;
  } else {
    *accelX = *accelY = *accelZ = 0.0;
    *gyroX = *gyroY = *gyroZ = 0.0;
  }
}

// =============================================
// ===== SCD40 =====
// =============================================
void scd40_init() {
  Serial.print("  🌡️  SCD40 (I2C)... ");
  Wire.beginTransmission(SCD40_ADDR);
  Wire.write(0x21);
  Wire.write(0xB1);
  Wire.endTransmission();
  delay(1000);
  scd40_ok = true;
  Serial.println("✅");
}

void scd40_read(uint16_t* co2, float* temperature, float* humidity) {
  if (!scd40_ok) return;
  Wire.beginTransmission(SCD40_ADDR);
  Wire.write(0xE4); Wire.write(0xB8);
  if (Wire.endTransmission() != 0) return;
  delay(2);
  if (Wire.requestFrom((uint8_t)SCD40_ADDR, (uint8_t)3) < 3) return;
  uint8_t st[3];
  for (int i = 0; i < 3; i++) st[i] = Wire.read();
  if (((((uint16_t)st[0] << 8) | st[1]) & 0x07FF) == 0) return;

  Wire.beginTransmission(SCD40_ADDR);
  Wire.write(0xEC); Wire.write(0x05);
  if (Wire.endTransmission() != 0) return;
  delay(2);
  if (Wire.requestFrom((uint8_t)SCD40_ADDR, (uint8_t)9) < 9) return;
  uint8_t data[9];
  for (int i = 0; i < 9; i++) data[i] = Wire.read();

  uint16_t co2_raw = ((uint16_t)data[0] << 8) | data[1];
  if (co2_raw == 0) return;

  *co2 = co2_raw;
  int16_t temp_raw = (int16_t)(((uint16_t)data[3] << 8) | data[4]);
  *temperature = -45.0 + 175.0 * (temp_raw / 65536.0);
  uint16_t hum_raw = ((uint16_t)data[6] << 8) | data[7];
  *humidity = 100.0 * (hum_raw / 65536.0);
}

// =============================================
// ===== INICIALIZAR SENSORES =====
// =============================================
void initSensors() {
  Serial.println("\n📡 Inicializando sensores (I2C directo)...\n");
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(500);
  mpu6050_init();
  scd40_init();
  Serial.println("\n📊 Sensores listos\n");
}

// =============================================
// ===== LEER SENSORES =====
// =============================================
void readIMU() {
  mpu6050_read(&sensorData.accelX, &sensorData.accelY, &sensorData.accelZ,
               &sensorData.gyroX, &sensorData.gyroY, &sensorData.gyroZ);
}

void readAmbiente() {
  scd40_read(&sensorData.co2, &sensorData.temperature_scd, &sensorData.humidity);
}

// =============================================
// ===== CONECTAR A WIFI (ROBUSTO) =====
// =============================================
void connectToWiFi() {
  if (wifi_connected) {
    // Revisar cada WIFI_CHECK_MS si sigue viva
    if (millis() - ultimoWifiCheck > WIFI_CHECK_MS) {
      ultimoWifiCheck = millis();
      if (WiFi.status() != WL_CONNECTED) {
        wifi_connected = false;
        Serial.println("⚠️  WiFi desconectado — reintentando");
      }
    }
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    ultimoWifiCheck = millis();
    Serial.println("✅ WiFi conectado!");
    Serial.print("   IP: ");
    Serial.println(WiFi.localIP());
    return;
  }

  static bool wifiStarted = false;
  if (!wifiStarted) {
    Serial.print("📶 Conectando a WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    wifiStarted = true;
  }
}

// =============================================
// ===== ENVIAR DATOS POR UDP (ROBUSTO) =====
// =============================================
void sendUDP() {
  if (!wifi_connected) return;

  char udp_payload[256];
  snprintf(udp_payload, sizeof(udp_payload),
    "{\"accelX\":%.2f,\"accelY\":%.2f,\"accelZ\":%.2f,"
    "\"gyroX\":%.2f,\"gyroY\":%.2f,\"gyroZ\":%.2f,"
    "\"co2\":%d,\"tempSCD\":%.2f,\"humidity\":%.1f,"
    "\"seq\":%lu}",
    sensorData.accelX, sensorData.accelY, sensorData.accelZ,
    sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ,
    sensorData.co2, sensorData.temperature_scd, sensorData.humidity,
    udpSeq);

  if (udp.beginPacket(JETSON_IP, JETSON_UDP_PORT)) {
    udp.write((uint8_t*)udp_payload, strlen(udp_payload));
    if (udp.endPacket()) {
      udpEnviados++;
    } else {
      udpFallidos++;
    }
  } else {
    udpFallidos++;
  }
  udpSeq++;
}

// =============================================
// ===== DOWNLINKS =====
// =============================================
#define DOWNLINK_PORT 10
#define DL_EMERGENCY_STOP 0x01
#define DL_PARK           0x02
#define DL_PING           0x03
#define DL_ACK            0x04

uint32_t cntDownlinks = 0;
uint8_t ultimoComando = 0;

void reenviarComandoAJetson(const char* nombre) {
  if (!wifi_connected) {
    Serial.printf("     ⚠️  '%s' NO se pudo reenviar: sin WiFi.\n", nombre);
    return;
  }
  char msg[96];
  snprintf(msg, sizeof(msg), "{\"type\":\"lora_downlink\",\"cmd\":\"%s\"}", nombre);
  udp.beginPacket(JETSON_IP, JETSON_UDP_PORT);
  udp.write((uint8_t*)msg, strlen(msg));
  udp.endPacket();
  Serial.printf("     ➡️  '%s' reenviado a Jetson\n", nombre);
}

void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
  cntDownlinks++;
  Serial.printf("\n📥 DOWNLINK #%lu — puerto %d, %d bytes, RSSI %d dBm, SNR %d\n",
                cntDownlinks, mcpsIndication->Port, mcpsIndication->BufferSize,
                mcpsIndication->Rssi, mcpsIndication->Snr);

  if (mcpsIndication->Port != DOWNLINK_PORT) {
    Serial.printf("     ignorado: puerto %d\n", mcpsIndication->Port);
    return;
  }
  if (mcpsIndication->BufferSize < 1) {
    Serial.println("     ignorado: payload vacío");
    return;
  }

  ultimoComando = mcpsIndication->Buffer[0];
  switch (ultimoComando) {
    case DL_EMERGENCY_STOP:
      Serial.println("     🛑 PARADA DE EMERGENCIA");
      reenviarComandoAJetson("emergency_stop");
      break;
    case DL_PARK:
      Serial.println("     🅿️  APARCAR");
      reenviarComandoAJetson("park");
      break;
    case DL_PING:
      Serial.println("     🏓 PING — enlace ok");
      break;
    case DL_ACK:
      Serial.println("     ✅ ACK recibido");
      break;
    default:
      Serial.printf("     comando desconocido: 0x%02X\n", ultimoComando);
      break;
  }
  Serial.println();
}

// =============================================
// ===== EMPAQUETAR PARA LORAWAN =====
// =============================================
static void prepareTxFrame(uint8_t port) {
  readIMU();  // dato más fresco para este paquete
  appDataSize = 0;
  uint8_t* buf = appData;

  int16_t ax = (int16_t)(sensorData.accelX * 100);
  int16_t ay = (int16_t)(sensorData.accelY * 100);
  int16_t az = (int16_t)(sensorData.accelZ * 100);
  buf[0] = (ax >> 8) & 0xFF; buf[1] = ax & 0xFF;
  buf[2] = (ay >> 8) & 0xFF; buf[3] = ay & 0xFF;
  buf[4] = (az >> 8) & 0xFF; buf[5] = az & 0xFF;
  appDataSize = 6;

  int16_t gx = (int16_t)(sensorData.gyroX * 10);
  int16_t gy = (int16_t)(sensorData.gyroY * 10);
  int16_t gz = (int16_t)(sensorData.gyroZ * 10);
  buf[6] = (gx >> 8) & 0xFF; buf[7] = gx & 0xFF;
  buf[8] = (gy >> 8) & 0xFF; buf[9] = gy & 0xFF;
  buf[10] = (gz >> 8) & 0xFF; buf[11] = gz & 0xFF;
  appDataSize = 12;

  buf[12] = (sensorData.co2 >> 8) & 0xFF;
  buf[13] = sensorData.co2 & 0xFF;
  appDataSize = 14;

  int16_t ts = (int16_t)(sensorData.temperature_scd * 100);
  buf[14] = (ts >> 8) & 0xFF; buf[15] = ts & 0xFF;
  appDataSize = 16;

  buf[16] = (uint8_t)sensorData.humidity;
  appDataSize = 17;

  sendCount++;
  Serial.print("📤 Enviando #");
  Serial.print(sendCount);
  Serial.print(" (");
  Serial.print(appDataSize);
  Serial.println(" bytes)");
}

// =============================================
// ===== MOTIVO DEL REINICIO =====
// =============================================
void mostrarMotivoDelReinicio() {
  esp_reset_reason_t motivo = esp_reset_reason();
  Serial.print("[boot] motivo: ");
  switch (motivo) {
    case ESP_RST_POWERON:  Serial.println("POWERON"); break;
    case ESP_RST_BROWNOUT: Serial.println("BROWNOUT — CAÍDA DE TENSIÓN"); break;
    case ESP_RST_TASK_WDT: Serial.println("TASK_WDT (watchdog)"); break;
    default:               Serial.printf("otro (%d)\n", (int)motivo); break;
  }
}

// =============================================
// ===== SETUP =====
// =============================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  mostrarMotivoDelReinicio();

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  🚗 LoRa V3 - OTAA + UDP v2          ║");
  Serial.println("║  I2C directo (sin librerías)         ║");
  Serial.println("║  MPU6050 + SCD40                     ║");
  Serial.println("╚════════════════════════════════════════╝\n");

  Serial.print("📋 DevEUI: ");
  for(int i = 0; i < 8; i++) {
    Serial.printf("%02X", devEui[i]);
    if(i < 7) Serial.print(":");
  }
  Serial.println();

  Serial.println("🚀 Inicializando LoRaWAN...");
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  Serial.println("✅ MCU listo");

  initSensors();
  connectToWiFi();

  lastJoinAttempt = millis();
  lastSensorLog = millis();
  Serial.println("⏳ Esperando JOIN en el primer ciclo...\n");
}

// =============================================
// ===== LOOP =====
// =============================================
void loop() {
  connectToWiFi();

  // ⭐ NUEVO: UDP a 20 Hz INDEPENDIENTE
  if (millis() - lastUdpMillis >= UDP_PERIODO_MS) {
    lastUdpMillis = millis();
    readIMU();
    sendUDP();
  }

  // ⭐ NUEVO: ambiental a su ritmo
  if (millis() - lastAmbienteMillis >= AMBIENTE_PERIODO_MS) {
    lastAmbienteMillis = millis();
    readAmbiente();
  }

  // Status cada 30 s
  if (millis() - lastSensorLog > 30000) {
    static uint32_t udpAnterior = 0;
    uint32_t udpEnEsteTramo = udpEnviados - udpAnterior;
    udpAnterior = udpEnviados;
    Serial.printf("[Status] State=%d | Joined=%s | WiFi=%s | UDP %lu/%lu en 30s (~%lu Hz)\n\n",
                  deviceState,
                  joined ? "✅" : "❌",
                  wifi_connected ? "✅" : "❌",
                  udpEnEsteTramo, udpEnEsteTramo + udpFallidos, udpEnEsteTramo / 30);
    lastSensorLog = millis();
  }

  // LoRaWAN state machine
  switch(deviceState) {
    case DEVICE_STATE_INIT:
      Serial.println("[State] DEVICE_STATE_INIT");
      LoRaWAN.init(loraWanClass, loraWanRegion);
      LoRaWAN.setDefaultDR(0);
      break;
    case DEVICE_STATE_JOIN:
      Serial.println("[State] 🔗 DEVICE_STATE_JOIN");
      LoRaWAN.join();
      break;
    case DEVICE_STATE_SEND:
      if(joined) {
        Serial.println("[State] 📤 DEVICE_STATE_SEND");
        prepareTxFrame(appPort);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
      } else {
        if(millis() - lastJoinAttempt > 60000) {
          Serial.println("⚠️  JOIN fallido, reintentando...\n");
          deviceState = DEVICE_STATE_JOIN;
          lastJoinAttempt = millis();
        }
      }
      break;
    case DEVICE_STATE_CYCLE:
      Serial.println("[State] ⏱️  DEVICE_STATE_CYCLE");
      LoRaWAN.cycle(appTxDutyCycle);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    case DEVICE_STATE_SLEEP:
      LoRaWAN.sleep(loraWanClass);
      if(!joined) joined = true;
      break;
    default:
      deviceState = DEVICE_STATE_INIT;
      break;
  }
  delay(10);
}
