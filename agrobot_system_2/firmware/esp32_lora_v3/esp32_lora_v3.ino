/*
 * AGROBOT CARRITO2 - OTAA + SENSORES (I2C DIRECTO)
 *

 *
 * Sensores (I2C directo, sin librerías):
 * - MPU6050: 0x68 (Acelerómetro + Giroscopio)
 * - SCD40: 0x62 (CO2 + Temperatura + Humedad)
 *
 * Basado en código COMPROBADO que hace JOIN
 */
#include "LoRaWan_APP.h"
#include "esp_system.h"   // esp_reset_reason(): por que se reinicio la placa
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
// =============================================
// ===== CONFIGURACIÓN WIFI / UDP =====
// =============================================
#define WIFI_SSID "JETSON_AP"
#define WIFI_PASSWORD "12345678"
#define JETSON_IP "192.168.12.1"
#define JETSON_UDP_PORT 5001
WiFiUDP udp;
bool wifi_connected = false;
// =============================================
// ===== CONFIGURACIÓN I2C =====
// =============================================
#define I2C_SDA 41
#define I2C_SCL 42
// Direcciones I2C
#define MPU6050_ADDR 0x68
#define SCD40_ADDR 0x62
// =============================================
// ===== CREDENCIALES OTAA (ChirpStack) =====
// =============================================
uint8_t devEui[] = { 0xf5, 0x9d, 0x08, 0x34, 0x59, 0x85, 0xb2, 0x7d };
uint8_t appEui[] = { 0x90, 0xe3, 0x84, 0xcc, 0x4d, 0x0d, 0x35, 0x02 };
uint8_t appKey[] = {
  0x6e, 0xc3, 0xb2, 0x10, 0xea, 0xc7, 0x1f, 0x74,
  0xb0, 0x08, 0x2c, 0xa9, 0xc1, 0xd6, 0xaa, 0xd2
};
// ===== VARIABLES ABP (REQUERIDAS) =====
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
// CLASE C, no A. Un dispositivo Clase C escucha de forma CONTINUA, asi que
// no puede dormirse: `Mcu.sleep()` deja de entrar en deep sleep y el sketch
// deja de reiniciarse en cada ciclo. Dos consecuencias buscadas:
//
//   - El WiFi se mantiene asociado entre ciclos, en vez de caerse en cada
//     sueño y perder el primer datagrama UDP al despertar. Es el paso previo
//     para poder sacar sendUDP() de prepareTxFrame() y mandar el IMU a su
//     propio ritmo en vez de una vez por ciclo LoRaWAN.
//   - Los downlinks dejan de esperar al siguiente uplink. En Clase A la
//     parada de emergencia podia tardar hasta 30 s; en Clase C llega en
//     segundos.
//
// El precio es el consumo: la radio queda en recepcion permanente. En un
// nodo a pilas seria inaceptable; aqui la Jetson y los motores gastan mucho
// mas.
//
// ⚠️ HAY QUE MARCARLO TAMBIEN EN CHIRPSTACK: perfil del dispositivo,
// pestaña "Class-C". Si el servidor cree que el nodo es Clase A, seguira
// encolando los downlinks para la ventana posterior al uplink y no se
// notara ninguna mejora.
DeviceClass_t loraWanClass = CLASS_C;
// 30 segundos. Se probo subirlo a 5 minutos —y sigue siendo defendible,
// porque el CO2, la temperatura y la humedad no cambian tan rapido y
// recortaria el tiempo en antena a una sexta parte— pero la version que el
// usuario dejo verificada en el robot es esta. No se cambia sin volver a
// comprobarlo en campo.
//
// Subirlo NO afecta a los downlinks: en Clase C el nodo escucha de forma
// continua, asi que la parada de emergencia llega en segundos y no espera al
// siguiente uplink.
uint32_t appTxDutyCycle = 30000;  // 30 segundos
bool overTheAirActivation = true;  // OTAA
bool loraWanAdr = true;
bool isTxConfirmed = false;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;
// =============================================
// ===== VARIABLES GLOBALES =====
// =============================================
int sendCount = 0;
uint32_t udpSeq = 0;        // secuencia propia del flujo UDP

// Ritmos del lazo, independientes del ciclo LoRaWAN.
//
// 50 ms = 20 Hz, que es exactamente la `frequency: 20.0` del EKF en
// ros2_ws/src/agrobot_navigation/config/ekf.yaml. Mandar mas rapido que eso
// no mejora la estimacion, solo carga la red y el backend.
//
// 5000 ms para el ambiental porque es el periodo real de medida del SCD40:
// pedirselo antes devuelve "sin dato" y ocupa el bus para nada.
const unsigned long UDP_PERIODO_MS      = 50;
const unsigned long AMBIENTE_PERIODO_MS = 5000;
unsigned long lastUdpMillis = 0;
unsigned long lastAmbienteMillis = 0;
unsigned long lastJoinAttempt = 0;
unsigned long lastSensorLog = 0;
// RTC_DATA_ATTR: la memoria RTC sobrevive al deep sleep, la RAM normal no.
// Sin esto, al despertar `joined` volvia a false, el case SEND caia en el
// `else` y a los 60 s forzaba otro JOIN. La libreria marca asi todas sus
// variables de estado, `deviceState` incluida.
RTC_DATA_ATTR bool joined = false;
bool mpu_ok = false;
bool scd40_ok = false;
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
// ===== MPU6050 (0x68) - I2C DIRECTO =====
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
    Serial.print("❌ (WHOAMI=0x");
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
// ===== SCD40 (0x62) - I2C DIRECTO =====
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
  Wire.write(0xE4); Wire.write(0xB8);          // get_data_ready_status
  if (Wire.endTransmission() != 0) return;
  delay(2);
  if (Wire.requestFrom((uint8_t)SCD40_ADDR, (uint8_t)3) < 3) return;
  uint8_t st[3];
  for (int i = 0; i < 3; i++) st[i] = Wire.read();
  if (((((uint16_t)st[0] << 8) | st[1]) & 0x07FF) == 0) return;

  Wire.beginTransmission(SCD40_ADDR);
  Wire.write(0xEC); Wire.write(0x05);          // read_measurement
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
// Las dos lecturas van SEPARADAS porque tienen ritmos muy distintos.
//
// El MPU6050 se lee en cada vuelta del envio UDP (20 Hz): es una lectura I2C
// de 14 bytes, cuesta menos de un milisegundo y es lo que alimenta al EKF de
// robot_localization, que corre a 20 Hz.
//
// El SCD40 entrega UNA medida nueva cada 5 segundos. Consultarlo a 20 Hz
// seria pedirle 100 veces lo mismo: 99 de ellas devuelve "sin dato" y la
// funcion sale por el `return` temprano, pero el bus I2C se ocupa igual y
// esa ocupacion se le quita al IMU. Por eso tiene su propio temporizador.
void readIMU() {
  mpu6050_read(&sensorData.accelX, &sensorData.accelY, &sensorData.accelZ,
               &sensorData.gyroX, &sensorData.gyroY, &sensorData.gyroZ);
}

void readAmbiente() {
  scd40_read(&sensorData.co2, &sensorData.temperature_scd, &sensorData.humidity);
}
// =============================================
// ===== CONECTAR A WIFI =====
// =============================================
// WiFi con reintentos y DHCP timeout mejorado (2026-10-04)
const unsigned long WIFI_DHCP_TIMEOUT_MS = 60000;    // 60s para DHCP
const unsigned long WIFI_RETRY_INTERVAL_MS = 10000;  // 10s entre reintentos
static unsigned long wifiLastRetry = 0;
static uint8_t wifiRetryCount = 0;

void connectToWiFi() {
  if (wifi_connected && WiFi.status() == WL_CONNECTED) {
    return;  // Ya conectado, no hacer nada
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    Serial.println("✅ WiFi conectado!");
    Serial.print("   IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("   RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    return;
  }

  // Si WiFi se perdió, intentar reconectar
  if (wifi_connected && WiFi.status() != WL_CONNECTED) {
    wifi_connected = false;
    Serial.println("⚠️  WiFi desconectado - reintentando...");
    wifiLastRetry = millis();
    wifiRetryCount = 0;
    return;
  }

  // Reintentos periódicos
  unsigned long ahora = millis();
  if (ahora - wifiLastRetry < WIFI_RETRY_INTERVAL_MS) {
    return;  // Esperar antes de reintentar
  }

  wifiLastRetry = ahora;
  wifiRetryCount++;

  Serial.printf("📶 WiFi intento #%u...", wifiRetryCount);
  WiFi.mode(WIFI_STA);
  // Configurar timeout DHCP largo antes de conectar
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.setTimeout(WIFI_DHCP_TIMEOUT_MS);  // 60s timeout
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Esperar hasta WIFI_DHCP_TIMEOUT_MS ms
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - wifiStart < WIFI_DHCP_TIMEOUT_MS) {
    delay(100);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    wifiRetryCount = 0;
    Serial.println(" ✅ conectado!");
    Serial.print("   IP: ");
    Serial.println(WiFi.localIP());
  } else {
    wifi_connected = false;
    Serial.printf(" ❌ timeout (intento %u)\n", wifiRetryCount);
  }
}
// =============================================
// ===== ENVIAR DATOS POR UDP =====
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
  udp.beginPacket(JETSON_IP, JETSON_UDP_PORT);
  udp.write((uint8_t*)udp_payload, strlen(udp_payload));
  udp.endPacket();
  udpSeq++;
  // AQUI NO SE IMPRIME NADA. A 20 Hz una linea por datagrama son 1200
  // mensajes por minuto: hace ilegible el log y ademas cada Serial.print
  // bloquea el lazo mientras vacia el buffer. El recuento sale en el
  // [Status] cada 30 s, que es donde se ve si el flujo va bien.
}
// =============================================
// ===== DOWNLINKS (comandos desde el gateway) =====
// =============================================
// OJO CON EL NOMBRE: la libreria de Heltec llama a `downLinkDataHandle`,
// con L MAYUSCULA en medio. Es el nombre que usan sus propios ejemplos.
// Escrito de cualquier otra forma el compilador no protesta —queda como una
// funcion normal que nadie llama— y los downlinks se pierden en silencio.
//
// Limites que conviene tener claros y no se pueden programar:
//
//   - El nodo es CLASE C desde el 2026-09-06 (antes era Clase A, y este
//     comentario decia lo contrario). Escucha de forma CONTINUA, asi que un
//     downlink llega en segundos en vez de esperar a la ventana posterior al
//     siguiente uplink. Aun asi, para frenar en el acto la via buena sigue
//     siendo el enlace UART directo Jetson -> ESP32-C3: LoRa es el respaldo
//     para cuando ese camino no esta.
//   - Este nodo NO controla motores. Solo puede reenviar la orden a la
//     Jetson, que es quien habla con el actuador. Si el WiFi esta caido, la
//     orden llega por radio pero no tiene por donde salir. Es un limite de
//     la arquitectura, no un fallo: se avisa por serie en vez de fingir que
//     se ejecuto.
#define DOWNLINK_PORT 10
#define DL_EMERGENCY_STOP 0x01
#define DL_PARK           0x02
#define DL_PING           0x03
#define DL_ACK            0x04

uint32_t cntDownlinks = 0;
uint8_t ultimoComando = 0;

// Reenvia el comando a la Jetson por el mismo UDP de la telemetria.
// jetson_server.py ya escucha estos mensajes en el puerto 5001 y ejecuta la
// parada contra el ESP32-C3 (ver el caso "lora_downlink" de su callback UDP).
void reenviarComandoAJetson(const char* nombre) {
  if (!wifi_connected) {
    Serial.printf("     ⚠️  '%s' NO se pudo reenviar: sin WiFi.\n", nombre);
    Serial.println("        La orden llego por LoRa pero este nodo no mueve motores.");
    return;
  }
  char msg[96];
  snprintf(msg, sizeof(msg), "{\"type\":\"lora_downlink\",\"cmd\":\"%s\"}", nombre);
  udp.beginPacket(JETSON_IP, JETSON_UDP_PORT);
  udp.write((uint8_t*)msg, strlen(msg));
  udp.endPacket();
  Serial.printf("     ➡️  '%s' reenviado a la Jetson (%s:%d)\n",
                nombre, JETSON_IP, JETSON_UDP_PORT);
}

void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
  cntDownlinks++;
  Serial.printf("\n📥 [RX] DOWNLINK #%lu — puerto %d, %d byte(s), RSSI %d dBm, SNR %d\n",
                cntDownlinks, mcpsIndication->Port, mcpsIndication->BufferSize,
                mcpsIndication->Rssi, mcpsIndication->Snr);

  Serial.print("     HEX: ");
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++)
    Serial.printf("%02X", mcpsIndication->Buffer[i]);
  Serial.println();

  // El puerto separa comandos (10) de cualquier otra cosa que baje. Sin este
  // filtro, un downlink de otra aplicacion o una trama de control MAC podria
  // interpretarse como una parada de emergencia.
  if (mcpsIndication->Port != DOWNLINK_PORT) {
    Serial.printf("     ignorado: puerto %d, se esperaba %d\n",
                  mcpsIndication->Port, DOWNLINK_PORT);
    return;
  }
  if (mcpsIndication->BufferSize < 1) {
    Serial.println("     ignorado: payload vacio");
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
      // No hace nada en el robot a proposito: sirve para comprobar que el
      // camino gateway -> ChirpStack -> radio -> nodo esta completo, sin
      // mover nada. Es la prueba que se puede hacer con el robot en marcha.
      Serial.println("     🏓 PING — el enlace de bajada funciona");
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
// ===== EMPAQUETAR DATOS PARA LORAWAN =====
// =============================================
static void prepareTxFrame(uint8_t port) {
  // Ya no se llama a readSensors() ni a sendUDP() desde aqui. El loop mantiene
  // `sensorData` fresco por su cuenta (IMU a 20 Hz, ambiental cada 5 s), asi
  // que esta funcion solo EMPAQUETA lo que ya hay.
  //
  // Que sendUDP() viviera aqui dentro era el motivo de que la telemetria a la
  // Jetson llegara una vez cada ciclo LoRaWAN: el IMU llegaba cada 30 s a un
  // EKF configurado a 20 Hz.
  readIMU();                 // el dato mas fresco posible para ESTE paquete
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
  Serial.print("📤 [TX] Enviando #");
  Serial.print(sendCount);
  Serial.print(" (");
  Serial.print(appDataSize);
  Serial.println(" bytes):");
  Serial.printf("     📍 Acelerómetro: %.2f, %.2f, %.2f g\n",
                sensorData.accelX, sensorData.accelY, sensorData.accelZ);
  Serial.printf("     🔄 Giroscopio:   %.2f, %.2f, %.2f °/s\n",
                sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ);
  Serial.printf("     💨 CO2:          %d ppm\n", sensorData.co2);
  Serial.printf("     🌡️  Temperatura:  %.2f°C (SCD)\n",
                sensorData.temperature_scd);
  Serial.printf("     💧 Humedad:      %.1f%%\n", sensorData.humidity);
}
// =============================================
// ===== MOTIVO DEL ULTIMO REINICIO =====
// =============================================
// La placa se estaba reiniciando cada 3 envios, en ciclos de 40.8 s
// clavados, y desde fuera era indistinguible de un fallo de la maquina de
// estados: en el gateway solo se veian rafagas de 3 paquetes. El chip
// guarda el motivo del ultimo arranque en un registro que sobrevive al
// reset; leerlo cuesta una linea y convierte "se reinicia" en un
// diagnostico concreto.
//
// Que esperar de cada uno:
//   POWERON            arranque normal, cable o bateria recien puestos
//   BROWNOUT           cayo la tension: alimentacion insuficiente para el
//                      pico de la radio. Es la sospecha principal.
//   TASK_WDT / INT_WDT un bucle se quedo sin ceder tiempo al watchdog
//   PANIC              excepcion del firmware (deberia dejar backtrace)
//   SW                 reinicio pedido por software
void mostrarMotivoDelReinicio() {
  esp_reset_reason_t motivo = esp_reset_reason();
  Serial.print("[boot] motivo del reinicio: ");
  switch (motivo) {
    case ESP_RST_POWERON:  Serial.println("POWERON (arranque normal)"); break;
    case ESP_RST_BROWNOUT: Serial.println("BROWNOUT — CAYO LA TENSION. "
                                          "La alimentacion no aguanta el pico de la radio."); break;
    case ESP_RST_TASK_WDT: Serial.println("TASK_WDT (watchdog de tarea)"); break;
    case ESP_RST_INT_WDT:  Serial.println("INT_WDT (watchdog de interrupciones)"); break;
    case ESP_RST_WDT:      Serial.println("WDT (watchdog)"); break;
    case ESP_RST_PANIC:    Serial.println("PANIC (excepcion del firmware)"); break;
    case ESP_RST_SW:       Serial.println("SW (reinicio pedido por software)"); break;
    case ESP_RST_DEEPSLEEP:Serial.println("DEEPSLEEP"); break;
    case ESP_RST_EXT:      Serial.println("EXT (pin de reset)"); break;
    default:               Serial.printf("otro (codigo %d)\n", (int)motivo); break;
  }
  Serial.printf("[boot] heap libre: %u bytes\n", ESP.getFreeHeap());
}
// =============================================
// ===== SETUP =====
// =============================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  mostrarMotivoDelReinicio();
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║  🚗 CARRITO2 - OTAA + SENSORES      ║");
  Serial.println("║  I2C DIRECTO (SIN LIBRERÍAS)        ║");
  Serial.println("║  MPU6050 + SCD40                    ║");
  Serial.println("╚═══════════════════════════════════════╝\n");
  Serial.print("📋 DevEUI: ");
  for(int i = 0; i < 8; i++) {
    Serial.printf("%02X", devEui[i]);
    if(i < 7) Serial.print(":");
  }
  Serial.println();
  Serial.print("🔑 AppKey: ");
  for(int i = 0; i < 16; i++) {
    Serial.printf("%02X", appKey[i]);
    if(i < 15) Serial.print(" ");
  }
  Serial.println("\n");
  Serial.println("🚀 Inicializando LoRaWAN...");
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  // LoRaWAN.init() NO se llama aqui. Mirando LoRaWan_APP.cpp de Heltec, su
  // ultima linea es `deviceState = DEVICE_STATE_JOIN;` y antes reinicia la
  // pila entera con LoRaMacInitialization(). Y setup() se ejecuta en CADA
  // despertar del deep sleep, no solo al encender: por eso el nodo hacia un
  // OTAA completo cada 40 s y ChirpStack le daba un DevAddr nuevo cada vez.
  //
  // Va en el case DEVICE_STATE_INIT del loop, que es donde lo pone Heltec en
  // sus propios ejemplos. `deviceState` esta marcado RTC_DATA_ATTR, asi que
  // sobrevive al sueño: al despertar vale DEVICE_STATE_SEND y ese case ya no
  // se vuelve a pisar. La sesion se conserva y los downlinks pasan a tener
  // una sesion viva a la que llegar.
  Serial.println("✅ MCU listo (LoRaWAN se inicializa en el primer ciclo)\n");
  initSensors();
  connectToWiFi();
  lastJoinAttempt = millis();
  lastSensorLog = millis();
  Serial.println("⏳ Esperando JOIN...\n");
}
// =============================================
// ===== MAIN LOOP =====
// =============================================
void loop() {
  connectToWiFi();

  // ---- IMU + UDP a 20 Hz, ya NO atado al ciclo LoRaWAN ----
  // Este bloque es el motivo de todo el trabajo de hoy: antes el IMU salia
  // hacia la Jetson una vez cada 30 s, colgado de prepareTxFrame(), y el EKF
  // se quedaba sin datos entre medias. Ahora va a su propio ritmo.
  if (millis() - lastUdpMillis >= UDP_PERIODO_MS) {
    lastUdpMillis = millis();
    readIMU();
    sendUDP();
  }

  // ---- ambiental a su ritmo: el SCD40 no da nada nuevo antes de 5 s ----
  if (millis() - lastAmbienteMillis >= AMBIENTE_PERIODO_MS) {
    lastAmbienteMillis = millis();
    readAmbiente();
  }

  if (millis() - lastSensorLog > 30000) {
    // `udpSeq` es acumulado; la resta da los datagramas de ESTOS 30 s. A
    // 20 Hz lo esperado son ~600. Un numero muy por debajo significa que el
    // lazo se esta bloqueando en algun sitio, y es la unica forma de verlo
    // sin instrumentar mas.
    static uint32_t udpSeqAnterior = 0;
    uint32_t udpEnEsteTramo = udpSeq - udpSeqAnterior;
    udpSeqAnterior = udpSeq;
    Serial.printf("[Status] State=%d | Joined=%s | WiFi=%s | heap=%u | UDP %lu en 30s (~%lu Hz)\n\n",
                  deviceState,
                  joined ? "✅ YES" : "❌ NO",
                  wifi_connected ? "✅ ON" : "❌ OFF",
                  ESP.getFreeHeap(),
                  udpEnEsteTramo, udpEnEsteTramo / 30);
    lastSensorLog = millis();
  }
  switch(deviceState) {
    case DEVICE_STATE_INIT:
      // Solo se entra aqui en el arranque en frio. Tras un despertar del
      // deep sleep `deviceState` ya vale DEVICE_STATE_SEND y este case se
      // salta, que es justo lo que conserva la sesion.
      Serial.println("[State] DEVICE_STATE_INIT — inicializando la pila LoRaWAN");
      LoRaWAN.init(loraWanClass, loraWanRegion);   // deja deviceState en JOIN
      LoRaWAN.setDefaultDR(0);
      break;
    case DEVICE_STATE_JOIN:
      Serial.println("[State] 🔗 DEVICE_STATE_JOIN - Enviando JoinRequest...");
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

    // LoRaWAN.sleep() NO se puede quitar: ademas de dormir, es donde se
    // llama a Radio.IrqProcess(). Se probo sacarlo y el nodo dejo de unirse
    // por completo, porque el JoinAccept no se llegaba a atender.
    //
    // En CLASE A esta llamada entraba en deep sleep y setup() se volvia a
    // ejecutar al despertar. Con CLASE C ya NO duerme —un dispositivo que
    // escucha en continuo no puede—, asi que la placa deja de reiniciarse en
    // cada ciclo y el lazo de los 20 Hz por UDP puede correr de verdad.
    //
    // El fallo de aquella epoca era llamar a LoRaWAN.init() desde setup():
    // reiniciaba la pila y forzaba un JOIN en cada despertar. Esta movido al
    // case DEVICE_STATE_INIT, y ahi se queda aunque ya no se duerma: es donde
    // lo pone Heltec en sus propios ejemplos.
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
