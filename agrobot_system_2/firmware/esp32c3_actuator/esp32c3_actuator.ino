/*
 * AGROBOT — ESP32-C3 Mini — NODO ACTUADOR (sin encoder, sin PID)
 * ================================================================
 * Rol único: recibir comandos y aplicar PWM directo a motores/servo/LED.
 * Cero lógica de control aquí — eso vive en la Jetson (robot_localization
 * + nav2). Este firmware solo ejecuta y protege (watchdog/failsafe).
 *
 * DOS VIAS DE MANDO EN PARALELO (2026-09-30): BLE (GATT) y WiFi UDP por el
 * AP de la Jetson. La Jetson manda cada orden por las dos; aquí se aplica
 * la más NUEVA por número de secuencia, venga por donde venga, y el estado
 * vuelve por las dos. Si una se cae, la otra sigue alimentando el watchdog
 * y el robot ni se entera. Las dos son inalámbricas: sigue sin haber ningún
 * cable entre la batería de motores y la Jetson (ver abajo).
 *
 * BATERIAS SEPARADAS (diseño post-accidente 2026-09-20):
 *   · Batería de MOTORES: alimenta los motores, el puente H (TA6586) y
 *     este ESP32-C3. El pico de corriente de un motor trabado se queda en
 *     este dominio.
 *   · Batería de LOGICA: alimenta solo la Jetson Orin Nano. CERO cables
 *     eléctricos entre los dos dominios — el enlace es BLE, inalámbrico.
 *   Esto es un REQUISITO de seguridad, no una conveniencia. El accidente
 *   del 2026-09-20 (back-EMF por la línea de GND común con UART) destruyó
 *   la C3 anterior y pudo dañar la Jetson. Con baterías separadas y BLE,
 *   no hay ruta eléctrica por donde pueda viajar un pico de voltaje.
 *
 * Historia de transportes (para no repetirla):
 *   - WiFi + UDP: el C3 tiene UNA sola radio de 2.4 GHz. La asociación al
 *     AP se caía sola y no se recuperaba. Eliminado 2026-09-04.
 *   - BLE (intento 1): sin WiFi la radio quedaba entera, pero se usó la
 *     pila Bluedroid (pesada) y la placa dejó de anunciarse. Eliminado
 *     2026-09-04.
 *   - UART: funcionó bien, pero el GND compartido creó una ruta para
 *     back-EMF. Destruyó la C3. Eliminado 2026-09-27.
 *   - BLE (intento 2): sin WiFi, sin UART, sin GND común. Funcionó, pero
 *     era un único punto de fallo: la placa dejaba de anunciarse, bluetoothd
 *     se caía en la Jetson y el adaptador Intel se quedaba sordo.
 *   - BLE + WiFi UDP (ESTE FIRMWARE, 2026-09-30). La caída del WiFi del
 *     2026-09-04 era el AP y el cliente de la Jetson en canales distintos
 *     sobre una sola radio: ya lo sincroniza agrobot-sync-ap-channel, y la
 *     LoRa V3 va estable por ese mismo AP. La radio de la C3 se reparte
 *     entre BLE y WiFi (coexistencia del ESP-IDF); por eso el WiFi va en
 *     modo de ahorro (obligatorio con BLE activo) y la Jetson no depende de
 *     él para la latencia: el WiFi es el respaldo, el BLE el principal.
 *     Con ENLACE_WIFI 0 se vuelve al firmware solo-BLE.
 *
 * PROTOCOLO BLE (8 bytes por trama, con magic y CRC-8). Idéntico al
 * protocolo UART: se puede reusar el mismo pack/unpack sin cambios.
 * Debe coincidir byte a byte con jetson/backend/ble_link.py.
 *
 * COMPILAR CON "USB CDC On Boot: Enabled" para que los logs salgan por USB.
 * Y con "Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)": WiFi + BLE no
 * caben en los 1,2 MB del esquema por defecto.
 *
 * ---------------------------------------------------------------------
 * PINOUT Y LADOS — CONFIRMADOS EN EL ROBOT REAL (2026-09-02)
 *
 *   LADO DERECHO:   M1 FI=GPIO0 BI=GPIO1   +   M4 FI=GPIO6 BI=GPIO7
 *   LADO IZQUIERDO: M2 FI=GPIO3 BI=GPIO2   +   M3 FI=GPIO4 BI=GPIO5
 *   GPIO8  -> FOCO (LED)     GPIO10 -> SERVO-CAM
 *   GPIO9, 20, 21 libres (9 = BOOT, 20/21 = UART0 de consola)
 *
 * El M2 tiene FI y BI al revés que los demás — así está cableado.
 *
 * ⚠️  EL ROBOT ES DIFERENCIAL (SKID-STEER). NO TIENE RUEDAS MECANUM.
 */

// BLE: el core de ESP32 Arduino 3.x usa NimBLE por defecto.
// En 2.x usa Bluedroid. La API de usuario es la misma.
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_system.h"   // esp_reset_reason(): por qué se reinició la placa

// Segunda vía de mando. 0 = firmware solo-BLE, como antes del 2026-09-30.
#define ENLACE_WIFI 1
#if ENLACE_WIFI
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

// Declaración adelantada. El IDE de Arduino genera solo los prototipos de
// TODAS las funciones y los pone antes de la primera función del sketch,
// que está más arriba que la definición de CmdFrame: sin esta línea,
// aplicarComando(const CmdFrame &) no compila ("does not name a type").
struct CmdFrame;


// =============================================
// ===== PINES Y RUEDAS (confirmados en el robot real) =====
// =============================================
#define R1_FI  0
#define R1_BI  1
#define L1_FI  3   // ⚠️ invertidos respecto de los otros: así está
#define L1_BI  2   //    cableado y verificado. GPIO2 es pin de strapping.
#define L2_FI  4
#define L2_BI  5
#define R2_FI  6
#define R2_BI  7

#define LED_PIN    8    // FOCO. ⚠️ pin de strapping — ver aviso más abajo

const bool LED_ACTIVO_EN_BAJO = true;

#define LED_NIVEL(encendido) \
  ((LED_ACTIVO_EN_BAJO ? !(encendido) : (encendido)) ? HIGH : LOW)
// Valor de `ledOn` que deja el foco FÍSICAMENTE apagado en este robot: su
// etapa es de lógica normal, así que es el "encendido" del firmware (pin en
// BAJO). La Jetson ya lo compensa en el protocolo (ble_link.pack).
const bool FOCO_APAGADO_FW = true;
#define SERVO_PIN 10    // SERVO-CAM

const bool INVERT_R1 = false;
const bool INVERT_R2 = false;
const bool INVERT_L1 = false;
const bool INVERT_L2 = false;

const uint32_t MOTOR_PWM_HZ = 1000;
const uint8_t MOTOR_PWM_BITS = 8;

// =============================================
// ===== PARÁMETROS FÍSICOS =====
// =============================================
const float WHEEL_RADIUS = 0.06f;   // m
const float TRACK_WIDTH  = 0.33f;   // m
const float K_ROT        = TRACK_WIDTH / 2.0f;

const float V_MAX        = 0.55f;   // m/s
const float OMEGA_MAX    = 1.5f;    // rad/s
const int   PWM_MAX      = 255;

// =============================================
// ===== ARRANQUE: ZONA MUERTA + PULSOS DE TORQUE =====
// =============================================
const int PWM_MIN = 200;

// Rueda parada = FRENO (FI=BI=HIGH: el TA6586 pone en corto el motor), no
// rueda libre (FI=BI=LOW). Sin freno el robot seguía rodando por inercia al
// soltar el mando o al saltar el watchdog. Con el motor quieto el corto no
// consume corriente; solo se nota al empujar el robot a mano encendido.
const bool FRENO_ACTIVO = true;

// =============================================
// ===== MOTORES TRASEROS DE APOYO =====
// =============================================
// Los delanteros (R1, L1) tienen mucho torque y llevan la tracción. Los
// traseros (R2, L2) son más rápidos y más débiles y se recalentaban: a igual
// PWM querían ir más rápido que los delanteros y terminaban peleando contra
// ellos, cargados. Ahora reciben una FRACCIÓN del duty del delantero de su
// lado: en recta apenas acompañan, en giro (el skid-steer necesita torque
// para arrastrar las ruedas de costado) ayudan más.
//
// Ojo, física: bajar el PWM NO les da más torque; les baja la velocidad, la
// corriente y el calor. El torque lo ponen los delanteros. Más torque real en
// los traseros solo se consigue con una reducción mecánica.
//
// AJUSTE: con el robot en el aire, mismo PWM en todas, contar vueltas por
// minuto de una delantera y de una trasera. FACTOR_TRASERO_RECTO ≈
// rpm_delantera / rpm_trasera: así la trasera gira a la velocidad del suelo y
// no pelea. Los valores de abajo son un punto de partida, no medidos.
const float FACTOR_TRASERO_RECTO = 0.60f;
const float FACTOR_TRASERO_GIRO  = 0.85f;
// Por debajo de esto la trasera no aporta nada útil: rueda LIBRE (no freno:
// frenada mientras el robot avanza sería un ancla).
const int   PWM_MIN_TRASERO      = 110;
// Tope de los pulsos de giro en las traseras (los delanteros van a 255).
const int   PWM_TOPE_TRASERO     = 215;
// Protección térmica ESTIMADA (no hay sensor): media móvil del duty² de las
// traseras con constante de tiempo TAU. Pasado CARGA_LIMITE, el aporte de las
// traseras baja linealmente hasta cero en CARGA_CORTE.
const float TERMICO_TAU_S        = 60.0f;
const float TERMICO_CARGA_LIMITE = 0.35f;
const float TERMICO_CARGA_CORTE  = 0.60f;
float cargaTrasera = 0.0f;   // 0..1, estimación del calentamiento

// =============================================
// ===== RAMPA DE ACELERACIÓN =====
// =============================================
// Acelerar de golpe (más los pulsos de arranque) hacía patinar las ruedas:
// la odometría por LIDAR (rf2o) y el EKF ven un salto que no es real y el
// SLAM se deforma. La velocidad SUBE como mucho a este ritmo; BAJAR y PARAR
// son inmediatos (la seguridad no espera a ninguna rampa).
const float RAMPA_V_PCT_S     = 250.0f;   // % de V_MAX por segundo (0→100 % en 0,4 s)
const float RAMPA_OMEGA_PCT_S = 300.0f;   // % de OMEGA_MAX por segundo

const uint8_t       KICK_PULSOS  = 3;
const unsigned long KICK_ON_MS   = 200;
const unsigned long KICK_OFF_MS  = 30;
const unsigned long KICK_REARME_MS = 400;
const uint16_t KICK_DESFASE_MS = 60;

// =============================================
// ===== GIRO: PISO DE DUTY + TREN DE PULSOS SOSTENIDO =====
// =============================================
const int PWM_MIN_GIRO = 235;
const float GIRO_UMBRAL = 1.0f;
const unsigned long GIRO_ON_MS  = 140;
const unsigned long GIRO_OFF_MS = 90;

bool          giroActivo    = false;
bool          giroAlto      = false;
unsigned long giroFaseHasta = 0;

// =============================================
// ===== SEGURIDAD =====
// =============================================
// BLE tiene latencia variable (intervalo de conexión 7.5..30 ms por
// defecto). 500 ms da margen suficiente: a 20 Hz, 10 tramas perdidas
// seguidas para que salte el failsafe. Vale para las DOS vías juntas: el
// watchdog lo alimenta cualquier trama nueva, llegue por BLE o por WiFi.
// Con solo WiFi las tramas llegan en ráfagas cada ~200 ms (el modo de
// ahorro despierta la radio en cada beacon DTIM): sigue dentro de 500 ms.
const unsigned long CMD_TIMEOUT_MS = 500;
unsigned long lastCmdMillis = 0;
bool inFailsafe = true;

// =============================================
// ===== SERVO DE CÁMARA + LED =====
// =============================================
const int SERVO_LEDC_CANAL   = 0;
const int SERVO_LEDC_BITS    = 14;
const int SERVO_LEDC_HZ      = 50;
const int SERVO_PULSO_MIN_US = 500;
const int SERVO_PULSO_MAX_US = 2400;
bool servoListo = false;

bool scanning = false;
unsigned long scanStartMillis = 0;
const unsigned long SCAN_SWEEP_MS = 10000;

const int SERVO_MIN_ANGLE  = 25;
const int SERVO_MAX_ANGLE  = 110;
const int SERVO_PARK_ANGLE = 98;

int  currentServoAngle = SERVO_PARK_ANGLE;
bool ledOn = false;

// =============================================
// ===== TIPOS DE LAS RUEDAS =====
// =============================================
enum RuedaCanal : uint8_t { CH_R1 = 2, CH_R2 = 3, CH_L1 = 4, CH_L2 = 5 };

struct Rueda {
  const uint8_t pinFI;
  const uint8_t pinBI;
  const uint8_t canal;
  const bool    invertida;
  const uint16_t kickDesfaseMs;
  int8_t        sentido;
  int           dutyPedido;
  uint8_t       kickRestantes;
  bool          kickAlto;
  unsigned long kickFaseHasta;
  unsigned long quietaDesde;
  bool          dcCompleto;
  bool          apoyo;   // trasera: motor de apoyo (ver MOTORES TRASEROS DE APOYO)
  bool          libre;   // parada en rueda libre en vez de freno (apoyo apagado en marcha)
};

static uint32_t servoDuty(int angulo) {
  int us = map(angulo, 0, 180, SERVO_PULSO_MIN_US, SERVO_PULSO_MAX_US);
  return (uint32_t)((float)us * ((float)(1UL << SERVO_LEDC_BITS) / 20000.0f));
}

static void servoEscribir(int angulo) {
  if (!servoListo) return;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(SERVO_LEDC_CANAL, servoDuty(angulo));
#else
  ledcWrite(SERVO_LEDC_CANAL, servoDuty(angulo));
#endif
}

// Servo SUAVE: setServo() solo fija el objetivo y updateServo() lo alcanza a
// SERVO_GRADOS_S. El salto directo sacudía la cámara (fotos movidas para el
// YOLO) y el pico de corriente del servo metía ruido en la C3.
const float SERVO_GRADOS_S = 90.0f;
int   servoObjetivo = SERVO_PARK_ANGLE;
float servoPosicion = SERVO_PARK_ANGLE;

void setServo(int angle) {
  servoObjetivo = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
}

void updateServo() {
  static unsigned long ultimo = millis();
  unsigned long ahora = millis();
  float dt = (ahora - ultimo) / 1000.0f;
  if (dt < 0.02f) return;
  ultimo = ahora;
  float d = servoObjetivo - servoPosicion;
  float paso = SERVO_GRADOS_S * dt;
  servoPosicion += constrain(d, -paso, paso);
  int angulo = (int)lroundf(servoPosicion);
  if (angulo != currentServoAngle) {
    currentServoAngle = angulo;
    servoEscribir(angulo);
  }
}

void setLed(bool on) {
  if (on != ledOn) {
    ledOn = on;
    digitalWrite(LED_PIN, LED_NIVEL(on));
  }
}

// =============================================
// ===== PROTOCOLO BLE (8 bytes, idéntico al UART) =====
// =============================================
#pragma pack(push, 1)
struct CmdFrame {
  uint8_t  magic;        // 0xA5
  int8_t   v_pct;        // -100..100
  int8_t   omega_pct;    // -100..100
  uint8_t  servo_angle;  // 0..180
  uint8_t  flags;        // bit0 LED, bit1 SCAN
  uint16_t seq;
  uint8_t  crc;          // CRC-8
};
struct StatusFrame {
  uint8_t  magic;        // 0x5A
  int8_t   v_pct;
  int8_t   omega_pct;
  uint8_t  servo_angle;
  uint8_t  flags;        // bit0 LED, bit1 escaneando, bit2 failsafe
  uint16_t seq_echo;
  uint8_t  crc;
};
#pragma pack(pop)

const uint8_t CMD_MAGIC    = 0xA5;
const uint8_t STATUS_MAGIC = 0x5A;

#define FLAG_LED     0x01
#define FLAG_SCAN    0x02

#define ST_LED       0x01
#define ST_SCANNING  0x02
#define ST_FAILSAFE  0x04
// La C3 se reinició por caída de tensión (brownout) en el último arranque.
// El 2026-09-28 la placa entró en bucle de reinicios con la batería baja y
// desde la Jetson solo se veían desconexiones: esto lo dice sin cable USB.
#define ST_BROWNOUT  0x08
// Arrancó hace menos de ARRANQUE_RECIENTE_MS: la Jetson cuenta reinicios.
#define ST_RECIEN    0x10
const unsigned long ARRANQUE_RECIENTE_MS = 15000;
// Estado de las dos vías, para que la Jetson vea cuál está viva.
#define ST_VIA_WIFI  0x20   // la última orden aplicada llegó por WiFi
#define ST_WIFI_OK   0x40   // asociada al AP y con IP
#define ST_BLE_OK    0x80   // con un cliente BLE conectado

const uint8_t VIA_BLE  = 1;
const uint8_t VIA_WIFI = 2;

static uint8_t crc8(const uint8_t *datos, size_t n) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < n; i++) {
    crc ^= datos[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// =============================================
// ===== BLE GATT =====
// =============================================
// UUIDs propios. Deben coincidir con ble_link.py.
#define BLE_SERVICE_UUID     "a0e1b000-7f3c-4a5e-9d21-6b8e2c4f1a00"
#define BLE_CMD_CHAR_UUID    "a0e1b001-7f3c-4a5e-9d21-6b8e2c4f1a00"
#define BLE_STATUS_CHAR_UUID "a0e1b002-7f3c-4a5e-9d21-6b8e2c4f1a00"

BLEServer         *pServer     = nullptr;
BLECharacteristic *pStatusChar = nullptr;
bool bleConnected  = false;
bool bleWasConnected = false;

// El callback de escritura BLE corre en la tarea del stack Bluetooth, NO en
// loop(). Se copia el comando a un buzón protegido y loop() lo consume.
portMUX_TYPE bleMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool blePending = false;
CmdFrame bleInbox;

float lastV = 0, lastOmega = 0;                 // lo pedido por la Jetson
float vAplicada = 0.0f, omegaAplicada = 0.0f;   // lo aplicado tras la rampa
uint8_t lastServo = 0;
bool lastLed = false;
uint16_t lastSeq = 0;
unsigned long tramasMalas = 0;
// Contadores por vía y tramas descartadas por llegar después de una más
// nueva (típico del WiFi en modo de ahorro, que entrega en ráfagas).
unsigned long tramasBle = 0, tramasWifi = 0, tramasViejas = 0;
uint8_t viaUltima = 0;

// =============================================
// ===== WIFI UDP (vía de respaldo) =====
// =============================================
#if ENLACE_WIFI
// Las mismas credenciales que usa el nodo LoRa V3 (esp32_lora_v3.ino).
#define WIFI_SSID     "JETSON_AP"
#define WIFI_PASSWORD "12345678"
// La C3 escucha órdenes en este puerto y manda el estado al mismo puerto de
// la Jetson (jetson/backend/udp_actuator_link.py). Es el 5002 que usaba el
// enlace UDP retirado el 2026-09-04: el backend ya lo tenía reservado.
const uint16_t C3_UDP_PUERTO     = 5002;
const uint16_t JETSON_UDP_PUERTO = 5002;
const unsigned long WIFI_REINTENTO_MS = 5000;
// Potencia de emisión WiFi. Se empieza baja: el AP está a ~50 cm, los
// picos de corriente son menores (la placa ya se reinició por caída de
// tensión) y en las placas "C3 SuperMini" la potencia alta DISTORSIONA la
// señal y no asocian. Si tras WIFI_INTENTOS_POR_POTENCIA intentos no
// asocia, se sube un escalón; al asociar se queda en esa.
const wifi_power_t WIFI_POTENCIAS[] = {WIFI_POWER_8_5dBm, WIFI_POWER_13dBm,
                                       WIFI_POWER_17dBm, WIFI_POWER_19_5dBm};
const uint8_t WIFI_N_POTENCIAS = sizeof(WIFI_POTENCIAS) / sizeof(WIFI_POTENCIAS[0]);
const uint8_t WIFI_INTENTOS_POR_POTENCIA = 3;
uint8_t wifiPotenciaIdx = 0;
uint8_t wifiIntentos = 0;
WiFiUDP udpC3;
bool wifiListo  = false;    // asociada y con IP
bool udpAbierto = false;
// La Jetson es la puerta de enlace del AP: así no hay IP fija que mantener.
IPAddress ipJetson;
#endif

// Lo pone el propio evento de desconexión. Antes loop() lo deducía de
// `bleWasConnected`, y si conectar y desconectar pasaban entre dos vueltas
// de loop() (el bucle de reconexiones con la batería baja) el anuncio no se
// reabría NUNCA: la placa encendida pero invisible hasta reiniciarla.
volatile bool reabrirAnuncio = false;
esp_reset_reason_t motivoReinicio = ESP_RST_UNKNOWN;

// Parámetros de conexión que la C3 le pide a la Jetson al conectar.
// Medido el 2026-09-30 con la placa nueva: la Jetson la oye a -95 dBm
// (a 50 cm; lo normal sería -40..-60) y la conexión se caía por tiempo de
// supervisión (0x08) o no llegaba a establecerse (0x3e). Con el intervalo
// de 48,75 ms y la supervisión por defecto bastaban unos pocos eventos
// perdidos para cortar. Más eventos por segundo = más oportunidades de
// pasar; 4 s de supervisión = aguanta los desvanecimientos. La seguridad
// NO depende de esto: el watchdog de CMD_TIMEOUT_MS para los motores a los
// 500 ms sin órdenes, esté la conexión viva o no.
const uint16_t BLE_INTERVALO_MIN = 12;    // x1,25 ms = 15 ms
const uint16_t BLE_INTERVALO_MAX = 24;    // x1,25 ms = 30 ms
const uint16_t BLE_SUPERVISION   = 400;   // x10 ms   = 4 s

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {
    bleConnected = true;
    Serial.println("[ble] cliente conectado");
  }
#if defined(CONFIG_NIMBLE_ENABLED)
  void onConnect(BLEServer *s, ble_gap_conn_desc *desc) override {
    s->requestConnParams(desc->conn_handle, BLE_INTERVALO_MIN, BLE_INTERVALO_MAX,
                         0, BLE_SUPERVISION);
  }
#elif defined(CONFIG_BLUEDROID_ENABLED)
  void onConnect(BLEServer *s, esp_ble_gatts_cb_param_t *param) override {
    s->updateConnParams(param->connect.remote_bda, BLE_INTERVALO_MIN, BLE_INTERVALO_MAX,
                        0, BLE_SUPERVISION);
  }
#endif
  void onDisconnect(BLEServer *s) override {
    bleConnected = false;
    reabrirAnuncio = true;
    Serial.println("[ble] cliente desconectado — reabriendo advertising");
  }
};

class CmdWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String val = c->getValue();
    if (val.length() != sizeof(CmdFrame)) {
      tramasMalas++;
      return;
    }
    CmdFrame frame;
    memcpy(&frame, val.c_str(), sizeof(frame));
    if (frame.magic != CMD_MAGIC) {
      tramasMalas++;
      return;
    }
    if (crc8((const uint8_t *)&frame, sizeof(frame) - 1) != frame.crc) {
      tramasMalas++;
      return;
    }
    portENTER_CRITICAL(&bleMux);
    memcpy(&bleInbox, &frame, sizeof(frame));
    blePending = true;
    portEXIT_CRITICAL(&bleMux);
  }
};

// =============================================
// ===== MOTORES =====
// =============================================
// R1/L1 = delanteras (tracción), R2/L2 = traseras (apoyo). Agrupación por
// lado verificada en el robot: MANIFEST §3, "Agrupación de lados".
Rueda ruedaR1 = {R1_FI, R1_BI, CH_R1, INVERT_R1, KICK_DESFASE_MS, 0, 0, 0, false, 0, 0, false, false, false};
Rueda ruedaR2 = {R2_FI, R2_BI, CH_R2, INVERT_R2, KICK_DESFASE_MS, 0, 0, 0, false, 0, 0, false, true,  false};
Rueda ruedaL1 = {L1_FI, L1_BI, CH_L1, INVERT_L1, 0,               0, 0, 0, false, 0, 0, false, false, false};
Rueda ruedaL2 = {L2_FI, L2_BI, CH_L2, INVERT_L2, 0,               0, 0, 0, false, 0, 0, false, true,  false};
Rueda *RUEDAS[4]     = {&ruedaR1, &ruedaR2, &ruedaL1, &ruedaL2};
Rueda *LADO_DER[2]   = {&ruedaR1, &ruedaR2};   // {delantera, trasera}
Rueda *LADO_IZQ[2]   = {&ruedaL1, &ruedaL2};   // {delantera, trasera}

static void pwmAttach(uint8_t pin, uint8_t canal) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, MOTOR_PWM_HZ, MOTOR_PWM_BITS, canal);
#else
  ledcSetup(canal, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  ledcAttachPin(pin, canal);
#endif
}

static void pwmDetach(uint8_t pin) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach(pin);
#else
  ledcDetachPin(pin);
#endif
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

static void pwmWriteCanal(uint8_t canal, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(canal, duty);
#else
  ledcWrite(canal, duty);
#endif
}

static void aplicarDuty(Rueda &r) {
  if (r.sentido == 0) {
    const uint8_t nivel = (FRENO_ACTIVO && !r.libre) ? HIGH : LOW;
    digitalWrite(r.pinFI, nivel);
    digitalWrite(r.pinBI, nivel);
    r.dcCompleto = false;
    return;
  }
  int duty = r.dutyPedido;
  const int tope = r.apoyo ? PWM_TOPE_TRASERO : PWM_MAX;
  if (r.kickRestantes > 0) {
    duty = r.kickAlto ? tope : 0;
  } else if (giroActivo && giroAlto) {
    duty = max(duty, tope);
  }

  uint8_t pinActivo = (r.sentido > 0) ? r.pinFI : r.pinBI;
  if (duty >= PWM_MAX) {
    if (!r.dcCompleto) {
      pwmDetach(pinActivo);
      r.dcCompleto = true;
    }
    digitalWrite(pinActivo, HIGH);
  } else {
    if (r.dcCompleto) {
      pwmAttach(pinActivo, r.canal);
      r.dcCompleto = false;
    }
    pwmWriteCanal(r.canal, duty);
  }
}

void setRueda(Rueda &r, int duty, int sentido) {
  duty = constrain(duty, 0, PWM_MAX);
  if (r.invertida) sentido = -sentido;
  if (duty == 0) sentido = 0;

  if (sentido != r.sentido) {
    if (r.sentido != 0) {
      if (r.dcCompleto) {
        digitalWrite(r.sentido > 0 ? r.pinFI : r.pinBI, LOW);
        r.dcCompleto = false;
      } else {
        pwmWriteCanal(r.canal, 0);
      }
      pwmDetach(r.sentido > 0 ? r.pinFI : r.pinBI);
    }
    if (sentido != 0) {
      uint8_t pinPasivo = (sentido > 0) ? r.pinBI : r.pinFI;
      pinMode(pinPasivo, OUTPUT);
      digitalWrite(pinPasivo, LOW);
      pwmAttach(sentido > 0 ? r.pinFI : r.pinBI, r.canal);

      // Las traseras no reciben pulsos de arranque: el robot lo arrancan
      // los delanteros, y los picos a fondo eran lo que más las calentaba.
      if (KICK_PULSOS > 0 && !r.apoyo && millis() - r.quietaDesde >= KICK_REARME_MS) {
        r.kickRestantes = KICK_PULSOS;
        r.kickAlto      = false;
        r.kickFaseHasta = millis() + r.kickDesfaseMs;
      }
    } else {
      r.kickRestantes = 0;
      r.quietaDesde   = millis();
    }
    r.sentido = sentido;
  }

  r.dutyPedido = duty;
  aplicarDuty(r);
}

void updateKicks() {
  unsigned long ahora = millis();
  for (int i = 0; i < 4; i++) {
    Rueda &r = *RUEDAS[i];
    if (r.kickRestantes == 0) continue;
    if ((long)(ahora - r.kickFaseHasta) < 0) continue;

    if (r.kickAlto) {
      r.kickAlto = false;
      r.kickRestantes--;
      r.kickFaseHasta = ahora + KICK_OFF_MS;
    } else {
      r.kickAlto = true;
      r.kickFaseHasta = ahora + KICK_ON_MS;
    }
    aplicarDuty(r);
  }
}

void updateGiro() {
  if (!giroActivo) {
    if (giroAlto) {
      giroAlto = false;
      for (int i = 0; i < 4; i++) aplicarDuty(*RUEDAS[i]);
    }
    return;
  }
  unsigned long ahora = millis();
  if ((long)(ahora - giroFaseHasta) < 0) return;

  giroAlto      = !giroAlto;
  giroFaseHasta = ahora + (giroAlto ? GIRO_ON_MS : GIRO_OFF_MS);
  for (int i = 0; i < 4; i++) aplicarDuty(*RUEDAS[i]);
}

// Aporte de las traseras según la carga térmica estimada: 1 = completo,
// 0 = apagadas (rueda libre) hasta que se enfríen.
static float factorTermicoTrasero() {
  if (cargaTrasera <= TERMICO_CARGA_LIMITE) return 1.0f;
  return constrain(1.0f - (cargaTrasera - TERMICO_CARGA_LIMITE) /
                          (TERMICO_CARGA_CORTE - TERMICO_CARGA_LIMITE), 0.0f, 1.0f);
}

// lado[0] = delantera (tracción), lado[1] = trasera (apoyo).
static void setLado(Rueda *lado[2], float w, float escala, float wRef) {
  float frac = constrain(fabs(w) * escala / wRef, 0.0f, 1.0f);
  const int piso = giroActivo ? PWM_MIN_GIRO : PWM_MIN;
  int duty = 0;
  if (frac > 0.02f) duty = piso + (int)lroundf(frac * (PWM_MAX - piso));
  int sentido = (w >= 0) ? 1 : -1;

  Rueda &del = *lado[0];
  Rueda &tra = *lado[1];
  del.libre = false;
  setRueda(del, duty, sentido);

  const float factor = (giroActivo ? FACTOR_TRASERO_GIRO : FACTOR_TRASERO_RECTO)
                       * factorTermicoTrasero();
  int dutyTra = (int)lroundf(duty * factor);
  if (dutyTra < PWM_MIN_TRASERO) dutyTra = 0;
  dutyTra = min(dutyTra, PWM_TOPE_TRASERO);
  // Delantera en marcha y trasera sin aporte: la trasera rueda libre.
  tra.libre = (duty > 0 && dutyTra == 0);
  setRueda(tra, dutyTra, sentido);
}

// Llamar en cada vuelta del loop: integra el calentamiento estimado.
void updateTermico() {
  static unsigned long ultimo = millis();
  unsigned long ahora = millis();
  float dt = (ahora - ultimo) / 1000.0f;
  if (dt < 0.05f) return;
  ultimo = ahora;
  float u = 0.0f;
  if (ruedaR2.sentido != 0) u = max(u, ruedaR2.dutyPedido / (float)PWM_MAX);
  if (ruedaL2.sentido != 0) u = max(u, ruedaL2.dutyPedido / (float)PWM_MAX);
  cargaTrasera += (u * u - cargaTrasera) * min(dt / TERMICO_TAU_S, 1.0f);
}

void applyMotors(float vx, float omega) {
  vx    = constrain(vx,    -V_MAX,     V_MAX);
  omega = constrain(omega, -OMEGA_MAX, OMEGA_MAX);

  float wIzq = (vx - K_ROT * omega) / WHEEL_RADIUS;
  float wDer = (vx + K_ROT * omega) / WHEEL_RADIUS;

  const float wRef = V_MAX / WHEEL_RADIUS;
  float wPico = fmaxf(fabs(wIzq), fabs(wDer));
  float escala = 1.0f;
  if (wPico > wRef && wPico > 0.0f) escala = wRef / wPico;

  bool giroAntes = giroActivo;
  giroActivo = (fabsf(K_ROT * omega) >= GIRO_UMBRAL * fabsf(vx))
               && (fabsf(omega) > 0.05f);
  if (giroActivo && !giroAntes) {
    giroAlto      = true;
    giroFaseHasta = millis() + GIRO_ON_MS;
  }

  setLado(LADO_IZQ, wIzq, escala, wRef);
  setLado(LADO_DER, wDer, escala, wRef);
}

void stopMotors() {
  giroActivo = false;
  giroAlto   = false;
  vAplicada = 0.0f;
  omegaAplicada = 0.0f;
  // Parado = las cuatro frenadas (también las traseras que iban libres).
  for (int i = 0; i < 4; i++) {
    RUEDAS[i]->libre = false;
    setRueda(*RUEDAS[i], 0, 0);
  }
}

// Rampa: lleva lo aplicado hacia lo pedido (lastV/lastOmega). Subir en
// magnitud está limitado; bajar, cambiar de sentido hacia cero y parar es
// inmediato. Se llama desde el loop, así la rampa avanza aunque las tramas
// lleguen a saltos.
static float rampa(float actual, float objetivo, float pasoMax) {
  if (fabsf(objetivo) <= fabsf(actual) || (objetivo * actual < 0.0f)) {
    // Frenar o invertir: primero se baja de golpe a lo pedido si va hacia
    // cero; si invierte, se pasa por cero y luego sube con rampa.
    if (objetivo * actual < 0.0f) return 0.0f;
    return objetivo;
  }
  float d = objetivo - actual;
  return actual + constrain(d, -pasoMax, pasoMax);
}

void updateRampa() {
  static unsigned long ultimo = millis();
  unsigned long ahora = millis();
  float dt = (ahora - ultimo) / 1000.0f;
  if (dt < 0.01f) return;
  ultimo = ahora;
  if (inFailsafe) return;
  float v = rampa(vAplicada, lastV, RAMPA_V_PCT_S / 100.0f * V_MAX * dt);
  float w = rampa(omegaAplicada, lastOmega, RAMPA_OMEGA_PCT_S / 100.0f * OMEGA_MAX * dt);
  if (v != vAplicada || w != omegaAplicada) {
    vAplicada = v;
    omegaAplicada = w;
    applyMotors(vAplicada, omegaAplicada);
  }
}

// =============================================
// ===== ESCANEO =====
// =============================================
void startScan() {
  scanning = true;
  scanStartMillis = millis();
}

void updateScan() {
  if (!scanning) return;
  unsigned long elapsed = millis() - scanStartMillis;
  if (elapsed >= SCAN_SWEEP_MS) {
    scanning = false;
    setServo(SERVO_PARK_ANGLE);
    return;
  }
  setServo(map(elapsed, 0, SCAN_SWEEP_MS, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE));
}

// =============================================
// ===== FAILSAFE =====
// =============================================
void enterFailsafe(const char *motivo) {
  inFailsafe = true;
  lastV = 0;
  lastOmega = 0;
  stopMotors();
  scanning = false;
  // Apagado DE VERDAD (ver FOCO_APAGADO_FW): setLed(false) lo encendía.
  setLed(FOCO_APAGADO_FW);
  lastLed = FOCO_APAGADO_FW;
  Serial.print("⚠️  FAILSAFE: ");
  Serial.println(motivo);
}

// =============================================
// ===== PROCESAR COMANDOS (BLE y WiFi) =====
// =============================================
// Aplica una trama válida venga de la vía que venga. Solo la más NUEVA por
// número de secuencia: la Jetson numera cada trama al construirla, así que
// el orden de `seq` es el orden en que se decidieron. Una trama del WiFi
// que llega tarde (ráfaga tras el beacon) detrás de otra más nueva por BLE
// NO puede volver a aplicar una orden vieja, p. ej. "avanza" después de
// "para". Tras un failsafe se acepta cualquier `seq`: el backend pudo
// reiniciarse y volver a contar desde cero.
bool aplicarComando(const CmdFrame &c, uint8_t via) {
  unsigned long ahora = millis();
  int16_t d = (int16_t)(c.seq - lastSeq);
  if (!inFailsafe && d <= 0 && d > -1000) {
    tramasViejas++;
    return false;
  }

  lastCmdMillis = ahora;
  lastSeq = c.seq;
  viaUltima = via;
  if (via == VIA_WIFI) tramasWifi++; else tramasBle++;

  lastV     = (constrain((int)c.v_pct,     -100, 100) / 100.0f) * V_MAX;
  lastOmega = (constrain((int)c.omega_pct, -100, 100) / 100.0f) * OMEGA_MAX;
  lastServo = c.servo_angle;
  lastLed   = (c.flags & FLAG_LED) != 0;

  inFailsafe = false;
  // Bajar o parar se aplica ya (paso 0 = sin subida); subir lo hace
  // updateRampa() en el loop.
  vAplicada     = rampa(vAplicada, lastV, 0.0f);
  omegaAplicada = rampa(omegaAplicada, lastOmega, 0.0f);
  applyMotors(vAplicada, omegaAplicada);
  if ((c.flags & FLAG_SCAN) && !scanning) startScan();
  setLed(lastLed);
  if (!scanning) {
    setServo(lastServo);
  }
  return true;
}

void procesarBLE() {
  if (!blePending) return;

  CmdFrame c;
  portENTER_CRITICAL(&bleMux);
  memcpy(&c, &bleInbox, sizeof(c));
  blePending = false;
  portEXIT_CRITICAL(&bleMux);

  aplicarComando(c, VIA_BLE);
}

#if ENLACE_WIFI
void procesarWifi() {
  if (!udpAbierto) return;
  // Vacía la cola (en modo de ahorro llegan varias juntas), con tope.
  for (int n = 0; n < 8; n++) {
    int tam = udpC3.parsePacket();
    if (tam <= 0) return;
    // Solo órdenes de la Jetson y del tamaño exacto.
    if (!(udpC3.remoteIP() == ipJetson) || tam != (int)sizeof(CmdFrame)) {
      tramasMalas++;
      continue;
    }
    CmdFrame c;
    if (udpC3.read((uint8_t *)&c, sizeof(c)) != (int)sizeof(c)
        || c.magic != CMD_MAGIC
        || crc8((const uint8_t *)&c, sizeof(c) - 1) != c.crc) {
      tramasMalas++;
      continue;
    }
    aplicarComando(c, VIA_WIFI);
  }
}

// Asociación con reintento propio: `setAutoReconnect` no siempre vuelve
// cuando el AP se reinicia (agrobot-sync-ap-channel reinicia hostapd si el
// router cambia de canal). No bloquea: WiFi.begin() vuelve enseguida.
void gestionarWifi() {
  static unsigned long ultimoIntento = 0;
  bool asociada = (WiFi.status() == WL_CONNECTED);
  if (asociada && !wifiListo) {
    wifiListo = true;
    ipJetson = WiFi.gatewayIP();
    udpAbierto = udpC3.begin(C3_UDP_PUERTO);
    wifiIntentos = 0;
    Serial.printf("[wifi] asociada: IP %s, Jetson %s, RSSI %d dBm, potencia #%u, UDP %s\n",
                  WiFi.localIP().toString().c_str(), ipJetson.toString().c_str(),
                  WiFi.RSSI(), wifiPotenciaIdx, udpAbierto ? "abierto" : "FALLO");
  } else if (!asociada && wifiListo) {
    wifiListo = false;
    if (udpAbierto) udpC3.stop();
    udpAbierto = false;
    ultimoIntento = millis();
    Serial.println("[wifi] asociación perdida — reintento (el BLE sigue)");
  }
  if (!asociada && millis() - ultimoIntento > WIFI_REINTENTO_MS) {
    ultimoIntento = millis();
    if (++wifiIntentos > WIFI_INTENTOS_POR_POTENCIA) {
      wifiIntentos = 1;
      wifiPotenciaIdx = (wifiPotenciaIdx + 1) % WIFI_N_POTENCIAS;
      Serial.printf("[wifi] no asocia: pruebo con la potencia #%u\n", wifiPotenciaIdx);
    }
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.setTxPower(WIFI_POTENCIAS[wifiPotenciaIdx]);
  }
}
#endif

void enviarEstado() {
  bool porBle = bleConnected && pStatusChar != nullptr;
#if ENLACE_WIFI
  bool porWifi = wifiListo && udpAbierto;
#else
  bool porWifi = false;
#endif
  if (!porBle && !porWifi) return;

  StatusFrame st;
  st.magic = STATUS_MAGIC;
  st.v_pct = (int8_t)constrain((int)lroundf(lastV / V_MAX * 100.0f), -100, 100);
  st.omega_pct = (int8_t)constrain((int)lroundf(lastOmega / OMEGA_MAX * 100.0f), -100, 100);
  st.servo_angle = (uint8_t)currentServoAngle;
  st.flags = 0;
  if (ledOn)      st.flags |= ST_LED;
  if (scanning)   st.flags |= ST_SCANNING;
  if (inFailsafe) st.flags |= ST_FAILSAFE;
  if (motivoReinicio == ESP_RST_BROWNOUT) st.flags |= ST_BROWNOUT;
  if (millis() < ARRANQUE_RECIENTE_MS)    st.flags |= ST_RECIEN;
  if (viaUltima == VIA_WIFI) st.flags |= ST_VIA_WIFI;
  if (porWifi)                st.flags |= ST_WIFI_OK;
  if (bleConnected)           st.flags |= ST_BLE_OK;
  st.seq_echo = lastSeq;
  st.crc = crc8((uint8_t *)&st, sizeof(st) - 1);

  if (porBle) {
    pStatusChar->setValue((uint8_t *)&st, sizeof(st));
    pStatusChar->notify();
  }
#if ENLACE_WIFI
  if (porWifi) {
    udpC3.beginPacket(ipJetson, JETSON_UDP_PUERTO);
    udpC3.write((const uint8_t *)&st, sizeof(st));
    udpC3.endPacket();
  }
#endif
}

// =============================================
// ===== SETUP =====
// =============================================
void setup() {
  // Foco APAGADO antes que nada. En este robot la etapa del foco es de
  // lógica normal (enciende con el pin en ALTO) y el firmware la trata como
  // activa en bajo (LED_ACTIVO_EN_BAJO, que no se toca: GPIO8 es pin de
  // strapping). Así que "apagado" es setLed(FOCO_APAGADO_FW) = pin en BAJO,
  // el mismo nivel que usa en marcha normal cuando la Jetson pide apagarlo.
  // Antes arrancaba con el pin en ALTO: el foco quedaba ENCENDIDO al
  // arrancar y en failsafe, justo cuando falta el enlace, gastando la
  // batería que ya estaba baja (2026-09-28). El strapping se lee durante el
  // reset, con el pin sin manejar: lo que haga el firmware después no cuenta.
  pinMode(LED_PIN, OUTPUT);
  ledOn = !FOCO_APAGADO_FW;
  setLed(FOCO_APAGADO_FW);
  motivoReinicio = esp_reset_reason();

  Serial.begin(115200);
  unsigned long t0Serial = millis();
  while (!Serial && millis() - t0Serial < 1500) { delay(10); }
  delay(200);

  Serial.println();
  Serial.printf("=== AGROBOT C3 BLE+WiFi — compilado el %s a las %s ===\n",
                __DATE__, __TIME__);
  Serial.printf("[boot] motivo del reinicio: %d%s\n", (int)motivoReinicio,
                motivoReinicio == ESP_RST_BROWNOUT ? " — CAÍDA DE TENSIÓN (batería baja)" : "");

  // Autoprueba del foco: apagada. Eran 8 s de arranque con el foco
  // parpadeando; en un bucle de reinicios por tensión baja, 8 s de consumo
  // extra por vuelta. Para probar el foco, poner 1.
  #define FOCO_AUTOPRUEBA 0
#if FOCO_AUTOPRUEBA
  Serial.printf("[foco] autoprueba — LED_ACTIVO_EN_BAJO = %s\n",
                LED_ACTIVO_EN_BAJO ? "true" : "false");
  for (int i = 0; i < 2; i++) {
    Serial.println("[foco] deberia estar ENCENDIDO ahora (2 s)");
    digitalWrite(LED_PIN, LED_NIVEL(true));
    delay(2000);
    Serial.println("[foco] deberia estar APAGADO ahora (2 s)");
    digitalWrite(LED_PIN, LED_NIVEL(false));
    delay(2000);
  }
  ledOn = !FOCO_APAGADO_FW;
  setLed(FOCO_APAGADO_FW);
  Serial.println("[foco] fin de la autoprueba, queda apagado");
#endif

  // Motores: todas las entradas a OUTPUT/LOW
  const int pinesMotor[] = {R1_FI, R1_BI, R2_FI, R2_BI,
                            L1_FI, L1_BI, L2_FI, L2_BI};
  for (int p : pinesMotor) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  stopMotors();

  Serial.print("[PWM] motores a ");
  Serial.print(MOTOR_PWM_HZ);
  Serial.print(" Hz, ");
  Serial.print(MOTOR_PWM_BITS);
  Serial.println(" bits, 1 canal LEDC por motor");
  Serial.println("[CINEMATICA] diferencial / skid-steer: vx, omega");
  Serial.print("[GIRO] piso PWM_MIN_GIRO=");
  Serial.print(PWM_MIN_GIRO);
  Serial.print(", pulsos ");
  Serial.print(GIRO_ON_MS);
  Serial.print("/");
  Serial.print(GIRO_OFF_MS);
  Serial.println(" ms");
  Serial.print("[ARRANQUE] zona muerta PWM_MIN=");
  Serial.print(PWM_MIN);
  Serial.print(", pulsos de torque: ");
  Serial.print(KICK_PULSOS);
  Serial.print(" x ");
  Serial.print(KICK_ON_MS);
  Serial.println(" ms");

  // Servo
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  servoListo = ledcAttachChannel(SERVO_PIN, SERVO_LEDC_HZ, SERVO_LEDC_BITS,
                                 SERVO_LEDC_CANAL);
#else
  servoListo = (ledcSetup(SERVO_LEDC_CANAL, SERVO_LEDC_HZ, SERVO_LEDC_BITS) != 0);
  if (servoListo) ledcAttachPin(SERVO_PIN, SERVO_LEDC_CANAL);
#endif
  if (servoListo) {
    Serial.printf("[servo] OK — GPIO%d, canal LEDC %d, %d Hz, %d bits\n",
                  SERVO_PIN, SERVO_LEDC_CANAL, SERVO_LEDC_HZ, SERVO_LEDC_BITS);
  } else {
    Serial.printf("[servo] ⚠️  FALLO al enganchar GPIO%d\n", SERVO_PIN);
  }
  servoEscribir(SERVO_PARK_ANGLE);
  currentServoAngle = SERVO_PARK_ANGLE;
  servoObjetivo = SERVO_PARK_ANGLE;
  servoPosicion = SERVO_PARK_ANGLE;

  // ── BLE ────────────────────────────────────────────────────────────────
  BLEDevice::init("AGROBOT-C3");

  // Potencia de transmisión al máximo: la Jetson está a ~50 cm pero los
  // motores generan mucho ruido electromagnético.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  BLEDevice::setPower(ESP_PWR_LVL_P9);
#else
  BLEDevice::setPower(ESP_PWR_LVL_P9);
#endif

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);

  // Característica de COMANDOS: la Jetson escribe aquí.
  BLECharacteristic *pCmdChar = pService->createCharacteristic(
    BLE_CMD_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCmdChar->setCallbacks(new CmdWriteCallback());

  // Característica de ESTADO: el C3 notifica aquí.
  pStatusChar = pService->createCharacteristic(
    BLE_STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
#if !defined(CONFIG_NIMBLE_ENABLED)
  // NimBLE (core 3.x en la C3) añade el 2902 solo al tener NOTIFY.
  pStatusChar->addDescriptor(new BLE2902());
#endif

  pService->start();

  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(BLE_SERVICE_UUID);
  pAdv->setScanResponse(true);
  // Intervalo de advertising: rápido para que la Jetson lo encuentre pronto.
  pAdv->setMinPreferred(0x06);  // 7.5 ms
  pAdv->setMaxPreferred(0x12);  // 22.5 ms
  BLEDevice::startAdvertising();

  Serial.println("[ble] servidor GATT activo, esperando conexión...");
  Serial.printf("[ble] nombre: AGROBOT-C3 | servicio: %s\n", BLE_SERVICE_UUID);

#if ENLACE_WIFI
  // ── WiFi (respaldo) ────────────────────────────────────────────────────
  // Sin WiFi.setSleep(false): con BLE activo el ESP-IDF exige el modo de
  // ahorro del módem y aborta si se desactiva.
  WiFi.setHostname("agrobot-c3");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Potencia inicial baja; gestionarWifi() la sube si no asocia.
  WiFi.setTxPower(WIFI_POTENCIAS[0]);
  Serial.printf("[wifi] asociando a %s (UDP %u)...\n", WIFI_SSID, C3_UDP_PUERTO);
#endif

  lastCmdMillis = millis();
}

// =============================================
// ===== LOOP =====
// =============================================
unsigned long ultimoEstado = 0;
const unsigned long ESTADO_PERIODO_MS = 100;  // 10 Hz

void loop() {
  // Órdenes pendientes de las dos vías
  procesarBLE();
#if ENLACE_WIFI
  procesarWifi();
  gestionarWifi();
#endif

  // Reconexión: reabrir el anuncio al desconectarse (lo marca el propio
  // evento) y, por si acaso, cada 5 s mientras no haya nadie conectado.
  {
    static unsigned long ultimoRearme = 0;
    if (!bleConnected && (reabrirAnuncio || millis() - ultimoRearme > 5000)) {
      reabrirAnuncio = false;
      ultimoRearme = millis();
      delay(20);
      pServer->startAdvertising();
    }
    bleWasConnected = bleConnected;
  }

  // Watchdog: sin comando fresco, el robot para
  if (!inFailsafe && millis() - lastCmdMillis > CMD_TIMEOUT_MS) {
    enterFailsafe("sin comando (ni por BLE ni por WiFi)");
  }

  updateScan();
  updateRampa();
  updateKicks();
  updateGiro();
  updateTermico();
  updateServo();

  // Traza de motores (2 Hz)
  {
    static unsigned long ultimaTraza = 0;
    static bool huboActividad = false;
    float vT = lastV;
    float wT = lastOmega;
    bool  activo = (fabsf(vT) > 0.001f) || (fabsf(wT) > 0.001f);
    for (int i = 0; i < 4 && !activo; i++) activo = (RUEDAS[i]->sentido != 0);

    if ((activo || huboActividad) && millis() - ultimaTraza >= 500) {
      ultimaTraza = millis();
      huboActividad = activo;
      const char *src = viaUltima == VIA_WIFI ? "WIFI" : (viaUltima == VIA_BLE ? "BLE" : "SIN ENLACE");
      Serial.printf("[mot] src=%s ble=%lu wifi=%lu viejas=%lu fs=%d crc_err=%lu v=%.2f w=%.2f | R1 %d/%d R2 %d/%d L1 %d/%d L2 %d/%d\n",
                    src, tramasBle, tramasWifi, tramasViejas, inFailsafe ? 1 : 0, tramasMalas, vT, wT,
                    ruedaR1.sentido, ruedaR1.dutyPedido,
                    ruedaR2.sentido, ruedaR2.dutyPedido,
                    ruedaL1.sentido, ruedaL1.dutyPedido,
                    ruedaL2.sentido, ruedaL2.dutyPedido);
    }
  }

  // Estado al cliente BLE (10 Hz)
  if (millis() - ultimoEstado >= ESTADO_PERIODO_MS) {
    ultimoEstado = millis();
    enviarEstado();
  }
}
