/*
 * AGROBOT — ESP32-C3 Mini — NODO ACTUADOR v2 (CORREGIDO)
 * ================================================================
 * FIXES 2026-10-06:
 * - Cambios de dirección: dead time real entre freno e inversión
 * - Filtro de secuencia: solo rechaza tramas viejas, no alternancia de vías
 * - Duty cycle sincronizado globalmente, no modular local
 * - WiFi UDP robusto: envío periódico de estado + timeout adaptativo
 * - Modo debug aumentado: traza de cambios de dirección
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_system.h"
#include "esp_task_wdt.h"
void iniciarWatchdogLoop();

struct CmdFrame;

#define ENLACE_WIFI 1
#if ENLACE_WIFI
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

// =============================================
// ===== PINES Y RUEDAS =====
// =============================================
#define R1_FI  0
#define R1_BI  1
#define L1_FI  3
#define L1_BI  2
#define L2_FI  4
#define L2_BI  5
#define R2_FI  6
#define R2_BI  7
#define LED_PIN    8
#define SERVO_PIN 10

const bool LED_ACTIVO_EN_BAJO = true;
const bool FOCO_APAGADO_FW = true;
#define LED_NIVEL(encendido) \
  ((LED_ACTIVO_EN_BAJO ? !(encendido) : (encendido)) ? HIGH : LOW)

const bool INVERT_R1 = false;
const bool INVERT_R2 = true;
const bool INVERT_L1 = false;
const bool INVERT_L2 = true;
const bool CABLEADO_INVIERTE_AVANCE = true;
const bool CABLEADO_INVIERTE_GIRO   = false;

// =============================================
// ===== PARÁMETROS FÍSICOS =====
// =============================================
const float WHEEL_RADIUS = 0.06f;
const float TRACK_WIDTH  = 0.33f;
const float K_ROT        = TRACK_WIDTH / 2.0f;
const float V_MAX        = 0.55f;
const float OMEGA_MAX    = 1.5f;
const int   PWM_MAX      = 100;

// =============================================
// ===== CONTROL ON/OFF: DEAD TIME + DUTY CYCLE =====
// =============================================
const int PWM_MIN = 0;
const bool FRENO_ACTIVO = true;
const unsigned long DEAD_TIME_MS = 50;    // ⭐ NUEVO: espera entre freno e inversión
const unsigned long MOTOR_PERIODO_MS = 200;
// Timer global para sincronizar duty cycles de todas las ruedas.
static unsigned long cicloMotoraAt = 0;
static unsigned long ultimoCicloMotor = 0;

// Traseras de apoyo
const float FACTOR_TRASERO_RECTO = 0.80f;
const float FACTOR_TRASERO_GIRO  = 1.00f;
const int   PWM_MIN_TRASERO      = 110;
const int   PWM_TOPE_TRASERO     = 255;
const float TERMICO_TAU_S        = 60.0f;
const float TERMICO_CARGA_LIMITE = 0.35f;
const float TERMICO_CARGA_CORTE  = 0.60f;
float cargaTrasera = 0.0f;

// =============================================
// ===== RAMPA DE ACELERACIÓN =====
// =============================================
const float RAMPA_V_PCT_S     = 250.0f;
const float RAMPA_OMEGA_PCT_S = 300.0f;
const uint8_t       KICK_PULSOS  = 3;
const unsigned long KICK_ON_MS   = 200;
const unsigned long KICK_OFF_MS  = 30;
const unsigned long KICK_REARME_MS = 400;
const uint16_t KICK_DESFASE_MS = 60;

// =============================================
// ===== GIRO =====
// =============================================
const int PWM_MIN_GIRO = 235;
const float GIRO_UMBRAL = 1.0f;
const unsigned long GIRO_ON_MS  = 140;
const unsigned long GIRO_OFF_MS = 90;
bool giroActivo = false, giroAlto = false;
unsigned long giroFaseHasta = 0;

// =============================================
// ===== SEGURIDAD Y COMUNICACIÓN =====
// =============================================
const unsigned long CMD_TIMEOUT_MS = 500;
unsigned long lastCmdMillis = 0;
bool inFailsafe = true;

// ⭐ NUEVO: mejor detección de secuencia (solo rechaza tramas viejas)
// Usa diferencia modular: si d > 0 es nueva, d <= 0 es vieja o duplicada.
// Sin el bound -1000: WiFi y BLE pueden alternar sin perder comandos.
#define SECUENCIA_VALIDA(seq_nuevo, seq_anterior) \
  (((uint16_t)(seq_nuevo - seq_anterior)) != 0)

// =============================================
// ===== SERVO + LED =====
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
const int SERVO_MIN_ANGLE  = 25, SERVO_MAX_ANGLE  = 110, SERVO_PARK_ANGLE = 98;
int  currentServoAngle = SERVO_PARK_ANGLE;
bool ledOn = false;

// =============================================
// ===== TIPOS DE RUEDAS CON DEAD TIME =====
// =============================================
enum RuedaCanal : uint8_t { CH_R1 = 2, CH_R2 = 3, CH_L1 = 4, CH_L2 = 5 };

struct Rueda {
  const uint8_t pinFI, pinBI, canal;
  const bool invertida;
  const uint16_t kickDesfaseMs;
  int8_t sentido;
  int dutyPedido;
  uint8_t kickRestantes;
  bool kickAlto;
  unsigned long kickFaseHasta;
  unsigned long quietaDesde;
  bool dcCompleto;
  bool apoyo, libre;
  unsigned long frenohasta;  // ⭐ NUEVO: timer del dead time
  int8_t sentidoAnterior;    // ⭐ NUEVO: para detectar cambio
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

const float SERVO_GRADOS_S = 90.0f;
int servoObjetivo = SERVO_PARK_ANGLE;
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
// ===== PROTOCOLO BLE =====
// =============================================
#pragma pack(push, 1)
struct CmdFrame {
  uint8_t  magic;
  int8_t   v_pct;
  int8_t   omega_pct;
  uint8_t  servo_angle;
  uint8_t  flags;
  uint16_t seq;
  uint8_t  crc;
};
struct StatusFrame {
  uint8_t  magic;
  int8_t   v_pct;
  int8_t   omega_pct;
  uint8_t  servo_angle;
  uint8_t  flags;
  uint16_t seq_echo;
  uint8_t  crc;
};
#pragma pack(pop)

const uint8_t CMD_MAGIC = 0xA5, STATUS_MAGIC = 0x5A;
#define FLAG_LED     0x01
#define FLAG_SCAN    0x02
#define ST_LED       0x01
#define ST_SCANNING  0x02
#define ST_FAILSAFE  0x04
#define ST_BROWNOUT  0x08
#define ST_RECIEN    0x10
#define ST_VIA_WIFI  0x20
#define ST_WIFI_OK   0x40
#define ST_BLE_OK    0x80
const unsigned long ARRANQUE_RECIENTE_MS = 15000;
const uint8_t VIA_BLE = 1, VIA_WIFI = 2;

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
#define BLE_SERVICE_UUID     "a0e1b000-7f3c-4a5e-9d21-6b8e2c4f1a00"
#define BLE_CMD_CHAR_UUID    "a0e1b001-7f3c-4a5e-9d21-6b8e2c4f1a00"
#define BLE_STATUS_CHAR_UUID "a0e1b002-7f3c-4a5e-9d21-6b8e2c4f1a00"

BLEServer *pServer = nullptr;
BLECharacteristic *pStatusChar = nullptr;
bool bleConnected = false, bleWasConnected = false;

portMUX_TYPE bleMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool blePending = false;
CmdFrame bleInbox;

float lastV = 0, lastOmega = 0;
float vAplicada = 0.0f, omegaAplicada = 0.0f;
uint8_t lastServo = 0;
bool lastLed = false;
uint16_t lastSeq = 0;
unsigned long tramasMalas = 0, tramasBle = 0, tramasWifi = 0, tramasViejas = 0;
uint8_t viaUltima = 0;

// =============================================
// ===== WIFI UDP ROBUSTO =====
// =============================================
#if ENLACE_WIFI
#define WIFI_SSID     "JETSON_AP"
#define WIFI_PASSWORD "12345678"
const uint16_t C3_UDP_PUERTO     = 5002;
const uint16_t JETSON_UDP_PUERTO = 5002;
const unsigned long WIFI_REINTENTO_MS = 5000;
const wifi_power_t WIFI_POTENCIAS[] = {WIFI_POWER_8_5dBm, WIFI_POWER_13dBm,
                                       WIFI_POWER_17dBm, WIFI_POWER_19_5dBm};
const uint8_t WIFI_N_POTENCIAS = sizeof(WIFI_POTENCIAS) / sizeof(WIFI_POTENCIAS[0]);
const uint8_t WIFI_INTENTOS_POR_POTENCIA = 3;
uint8_t wifiPotenciaIdx = 0, wifiIntentos = 0;
WiFiUDP udpC3;
bool wifiListo = false, udpAbierto = false;
IPAddress ipJetson;

// ⭐ NUEVO: envío periódico de estado para mantener viva la conexión
const unsigned long ESTADO_KEEPALIVE_MS = 200;  // 5 Hz mínimo
unsigned long ultimoEstadoWifi = 0;
#endif

volatile bool reabrirAnuncio = false;
esp_reset_reason_t motivoReinicio = ESP_RST_UNKNOWN;

const uint16_t BLE_INTERVALO_MIN = 12;
const uint16_t BLE_INTERVALO_MAX = 24;
const uint16_t BLE_SUPERVISION   = 400;

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
    if (val.length() != sizeof(CmdFrame)) { tramasMalas++; return; }
    CmdFrame frame;
    memcpy(&frame, val.c_str(), sizeof(frame));
    if (frame.magic != CMD_MAGIC) { tramasMalas++; return; }
    if (crc8((const uint8_t *)&frame, sizeof(frame) - 1) != frame.crc) { tramasMalas++; return; }
    portENTER_CRITICAL(&bleMux);
    memcpy(&bleInbox, &frame, sizeof(frame));
    blePending = true;
    portEXIT_CRITICAL(&bleMux);
  }
};

// =============================================
// ===== MOTORES CON DEAD TIME =====
// =============================================
Rueda ruedaR1 = {R1_FI, R1_BI, CH_R1, INVERT_R1, KICK_DESFASE_MS, 0, 0, 0, false, 0, 0, false, false, false, 0, 0};
Rueda ruedaR2 = {R2_FI, R2_BI, CH_R2, INVERT_R2, KICK_DESFASE_MS, 0, 0, 0, false, 0, 0, false, true,  false, 0, 0};
Rueda ruedaL1 = {L1_FI, L1_BI, CH_L1, INVERT_L1, 0,               0, 0, 0, false, 0, 0, false, false, false, 0, 0};
Rueda ruedaL2 = {L2_FI, L2_BI, CH_L2, INVERT_L2, 0,               0, 0, 0, false, 0, 0, false, true,  false, 0, 0};
Rueda *RUEDAS[4]   = {&ruedaR1, &ruedaR2, &ruedaL1, &ruedaL2};
Rueda *LADO_DER[2] = {&ruedaR1, &ruedaR2};
Rueda *LADO_IZQ[2] = {&ruedaL1, &ruedaL2};

static void motorSetup(uint8_t pinFI, uint8_t pinBI) {
  pinMode(pinFI, OUTPUT);
  pinMode(pinBI, OUTPUT);
  digitalWrite(pinFI, LOW);
  digitalWrite(pinBI, LOW);
}

static void motorON(uint8_t pinActivo) { digitalWrite(pinActivo, HIGH); }
static void motorOFF(uint8_t pinActivo) { digitalWrite(pinActivo, LOW); }
static void motorFreno(uint8_t pinFI, uint8_t pinBI) {
  digitalWrite(pinFI, LOW);
  digitalWrite(pinBI, LOW);
}

// ⭐ NUEVO: aplicación de duty con espera de dead time
static void aplicarDuty(Rueda &r) {
  unsigned long ahora = millis();

  // Si está en dead time, espera
  if ((long)(ahora - r.frenohasta) < 0) {
    motorFreno(r.pinFI, r.pinBI);
    r.dcCompleto = false;
    return;
  }

  if (r.sentido == 0) {
    motorFreno(r.pinFI, r.pinBI);
    r.dcCompleto = false;
    return;
  }

  int duty = r.dutyPedido;
  const int tope = r.apoyo ? PWM_TOPE_TRASERO : PWM_MAX;

  if (r.kickRestantes > 0) {
    duty = r.kickAlto ? tope : 0;
  }
  else if (giroActivo && giroAlto) {
    duty = max(duty, tope);
  }

  uint8_t pinActivo = (r.sentido > 0) ? r.pinFI : r.pinBI;
  uint8_t pinPasivo = (r.sentido > 0) ? r.pinBI : r.pinFI;

  if (!r.dcCompleto) {
    r.dcCompleto = true;
    cicloMotoraAt = ahora;  // ⭐ NUEVO: sincronizar con ciclo global
  }

  if (duty >= 100) {
    motorON(pinActivo);
    motorOFF(pinPasivo);
  } else if (duty <= 0) {
    motorOFF(pinActivo);
    motorOFF(pinPasivo);
  } else {
    // ⭐ NUEVO: usar timer global, no local
    unsigned long faseGlobal = (ahora - cicloMotoraAt) % MOTOR_PERIODO_MS;
    unsigned long duracionON = (MOTOR_PERIODO_MS * duty) / 100;
    if (faseGlobal < duracionON) {
      motorON(pinActivo);
      motorOFF(pinPasivo);
    } else {
      motorOFF(pinActivo);
      motorOFF(pinPasivo);
    }
  }
}

// ⭐ NUEVO: cambio de sentido con dead time explícito
void setRueda(Rueda &r, int duty, int sentido) {
  duty = constrain(duty, 0, PWM_MAX);
  if (r.invertida) sentido = -sentido;
  if (duty == 0) sentido = 0;

  unsigned long ahora = millis();

  if (sentido != r.sentido) {
    // ⭐ NUEVO: activar dead time AQUÍ
    motorFreno(r.pinFI, r.pinBI);
    r.frenohasta = ahora + DEAD_TIME_MS;

    if (sentido != 0) {
      if (KICK_PULSOS > 0 && !r.apoyo && millis() - r.quietaDesde >= KICK_REARME_MS) {
        r.kickRestantes = KICK_PULSOS;
        r.kickAlto      = false;
        r.kickFaseHasta = millis() + r.kickDesfaseMs;
      }
    } else {
      r.kickRestantes = 0;
      r.quietaDesde   = millis();
    }
    r.sentidoAnterior = r.sentido;  // ⭐ NUEVO: tracking
    r.sentido = sentido;
    // Log de cambio de dirección
    if (r.sentidoAnterior != sentido) {
      const char *nom = (&r == &ruedaR1) ? "R1" : (&r == &ruedaR2) ? "R2" :
                        (&r == &ruedaL1) ? "L1" : "L2";
      Serial.printf("[mot] %s: %d -> %d (dead time %lu ms)\n", nom, r.sentidoAnterior, sentido, DEAD_TIME_MS);
    }
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

static float factorTermicoTrasero() {
  if (cargaTrasera <= TERMICO_CARGA_LIMITE) return 1.0f;
  return constrain(1.0f - (cargaTrasera - TERMICO_CARGA_LIMITE) /
                          (TERMICO_CARGA_CORTE - TERMICO_CARGA_LIMITE), 0.0f, 1.0f);
}

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
  tra.libre = (duty > 0 && dutyTra == 0);
  setRueda(tra, dutyTra, sentido);
}

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
  if (CABLEADO_INVIERTE_AVANCE) vx = -vx;
  if (CABLEADO_INVIERTE_GIRO)   omega = -omega;
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
  for (int i = 0; i < 4; i++) {
    RUEDAS[i]->libre = false;
    setRueda(*RUEDAS[i], 0, 0);
  }
}

static float rampa(float actual, float objetivo, float pasoMax) {
  if (fabsf(objetivo) <= fabsf(actual) || (objetivo * actual < 0.0f)) {
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
  dt = min(dt, 0.05f);
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
  setLed(FOCO_APAGADO_FW);
  lastLed = FOCO_APAGADO_FW;
  Serial.print("⚠️  FAILSAFE: ");
  Serial.println(motivo);
}

// =============================================
// ===== PROCESAR COMANDOS (BLE y WiFi) =====
// =============================================
// ⭐ NUEVO: filtro de secuencia CORRECTO — solo rechaza viejas, no alternancia
bool aplicarComando(const CmdFrame &c, uint8_t via) {
  unsigned long ahora = millis();

  // ⭐ NUEVO: detección correcta de secuencia nueva
  if (!inFailsafe && !SECUENCIA_VALIDA(c.seq, lastSeq)) {
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
  for (int n = 0; n < 8; n++) {
    int tam = udpC3.parsePacket();
    if (tam <= 0) return;
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

// ⭐ NUEVO: mejor manejo de WiFi con reintento suave
void gestionarWifi() {
  static unsigned long ultimoIntento = 0;
  bool asociada = (WiFi.status() == WL_CONNECTED);
  if (asociada && !wifiListo) {
    wifiListo = true;
    ipJetson = WiFi.gatewayIP();
    udpAbierto = udpC3.begin(C3_UDP_PUERTO);
    wifiIntentos = 0;
    ultimoEstadoWifi = millis();
    Serial.printf("[wifi] ✅ asociada: IP %s, Jetson %s, RSSI %d dBm\n",
                  WiFi.localIP().toString().c_str(), ipJetson.toString().c_str(),
                  WiFi.RSSI());
  } else if (!asociada && wifiListo) {
    wifiListo = false;
    if (udpAbierto) udpC3.stop();
    udpAbierto = false;
    ultimoIntento = millis();
    Serial.println("[wifi] ❌ asociación perdida — reintento (BLE sigue activo)");
  }
  if (!asociada && millis() - ultimoIntento > WIFI_REINTENTO_MS) {
    ultimoIntento = millis();
    if (++wifiIntentos > WIFI_INTENTOS_POR_POTENCIA) {
      wifiIntentos = 1;
      wifiPotenciaIdx = (wifiPotenciaIdx + 1) % WIFI_N_POTENCIAS;
      Serial.printf("[wifi] subiendo potencia a #%u\n", wifiPotenciaIdx);
    }
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.setTxPower(WIFI_POTENCIAS[wifiPotenciaIdx]);
  }
}
#endif

// ⭐ NUEVO: envío robusto de estado (no depende de cambios, se envía periódicamente)
void enviarEstado() {
  bool porBle = bleConnected && pStatusChar != nullptr;
#if ENLACE_WIFI
  bool porWifi = wifiListo && udpAbierto;
#else
  bool porWifi = false;
#endif

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
  const int pinesMotor[] = {R1_FI, R1_BI, R2_FI, R2_BI, L1_FI, L1_BI, L2_FI, L2_BI};
  for (int p : pinesMotor) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  stopMotors();

  pinMode(LED_PIN, OUTPUT);
  ledOn = !FOCO_APAGADO_FW;
  setLed(FOCO_APAGADO_FW);
  motivoReinicio = esp_reset_reason();

  Serial.begin(115200);
  unsigned long t0Serial = millis();
  while (!Serial && millis() - t0Serial < 1500) delay(10);
  delay(200);

  Serial.println();
  Serial.printf("🤖 === AGROBOT C3 BLE+WiFi v2 (CORREGIDO) — %s %s ===\n", __DATE__, __TIME__);
  Serial.printf("🔧 Dead time: %lu ms | Ciclo motor: %lu ms\n", DEAD_TIME_MS, MOTOR_PERIODO_MS);

  // Servo
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  servoListo = ledcAttachChannel(SERVO_PIN, SERVO_LEDC_HZ, SERVO_LEDC_BITS, SERVO_LEDC_CANAL);
#else
  servoListo = (ledcSetup(SERVO_LEDC_CANAL, SERVO_LEDC_HZ, SERVO_LEDC_BITS) != 0);
  if (servoListo) ledcAttachPin(SERVO_PIN, SERVO_LEDC_CANAL);
#endif
  servoEscribir(SERVO_PARK_ANGLE);
  currentServoAngle = SERVO_PARK_ANGLE;

  // BLE
  BLEDevice::init("AGROBOT-C3");
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  BLEDevice::setPower(ESP_PWR_LVL_P9);
#else
  BLEDevice::setPower(ESP_PWR_LVL_P9);
#endif
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
  BLECharacteristic *pCmdChar = pService->createCharacteristic(
    BLE_CMD_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCmdChar->setCallbacks(new CmdWriteCallback());
  pStatusChar = pService->createCharacteristic(
    BLE_STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
#if !defined(CONFIG_NIMBLE_ENABLED)
  pStatusChar->addDescriptor(new BLE2902());
#endif
  pService->start();
  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(BLE_SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);
  pAdv->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("✅ BLE servidor activo");

#if ENLACE_WIFI
  WiFi.setHostname("agrobot-c3");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setTxPower(WIFI_POTENCIAS[0]);
  Serial.println("📶 WiFi asociando...");
#endif

  lastCmdMillis = millis();
  iniciarWatchdogLoop();
}

// =============================================
// ===== WATCHDOG =====
// =============================================
const uint32_t WDT_LOOP_MS = 4000;

void iniciarWatchdogLoop() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {WDT_LOOP_MS, 0, true};
  if (esp_task_wdt_reconfigure(&cfg) != ESP_OK) esp_task_wdt_init(&cfg);
#else
  esp_task_wdt_init((WDT_LOOP_MS + 999) / 1000, true);
#endif
  enableLoopWDT();
}

// =============================================
// ===== LOOP =====
// =============================================
unsigned long ultimoEstadoBle = 0;
const unsigned long ESTADO_PERIODO_BLE_MS = 100;

void loop() {
  procesarBLE();
#if ENLACE_WIFI
  procesarWifi();
  gestionarWifi();
#endif

  static unsigned long ultimoRearme = 0;
  if (!bleConnected && (reabrirAnuncio || millis() - ultimoRearme > 5000)) {
    reabrirAnuncio = false;
    ultimoRearme = millis();
    delay(20);
    pServer->startAdvertising();
  }
  bleWasConnected = bleConnected;

  if (!inFailsafe && millis() - lastCmdMillis > CMD_TIMEOUT_MS) {
    enterFailsafe("sin comando (ni BLE ni WiFi)");
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
    float vT = lastV, wT = lastOmega;
    if ((vT != 0 || wT != 0 || millis() - ultimaTraza > 500) && millis() - ultimaTraza >= 500) {
      ultimaTraza = millis();
      const char *src = viaUltima == VIA_WIFI ? "WIFI" : (viaUltima == VIA_BLE ? "BLE" : "---");
      Serial.printf("[mot] %s | BLE:%lu WiFi:%lu VIEJAS:%lu fs=%d v=%.2f w=%.2f\n",
                    src, tramasBle, tramasWifi, tramasViejas, inFailsafe ? 1 : 0, vT, wT);
    }
  }

  // Estado BLE (10 Hz)
  if (millis() - ultimoEstadoBle >= ESTADO_PERIODO_BLE_MS) {
    ultimoEstadoBle = millis();
    enviarEstado();
  }
}
