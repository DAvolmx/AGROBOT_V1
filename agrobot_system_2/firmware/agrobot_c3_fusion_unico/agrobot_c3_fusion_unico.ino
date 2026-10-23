/*
 * AGROBOT — ESP32-C3 — NODO ACTUADOR "FUSION" (BLE + WiFi UDP)
 * v3 (skid-steer, tension COMPLETA en rafagas: maximo par para un robot pesado) + suave
 * (rampas, pausa al invertir, deadman, config por UDP). BLE, WiFi, protocolo, servo y failsafe
 * SIN CAMBIOS respecto al v3/suave.
 *
 *   - RAFAGAS de tension completa: la rampa suaviza la ENVOLVENTE (duty) y el motor la recibe como
 *     pulsos de 100 % con periodo corto ("periodo", 100 ms; el v3 usaba 200 ms sin rampa). Vence la
 *     zona muerta de un robot pesado, que el PWM fino de 500 Hz no vence. periodo = 0: PWM continuo.
 *   - DELANTERAS: avance; toman el "dif_del" % del giro (0 = solo longitudinales como el suave,
 *     100 = skid-steer puro como el v3). TRASERAS: giro y MAS fuerza ("tras").
 *   - ANTI-HUNDIMIENTO del v3 (opcional, "refuerzo" > 100 %): esfuerzo sostenido => mas potencia.
 *   - Parada normal = rampa; failsafe (500 ms sin ordenes) = ruedas libres al instante.
 *   - Sin test de motores al encender. Pines, inversion y limites configurables desde el DASHBOARD.
 * Los limites de fabrica son VALIDOS (en el suave pegado el 2026-10-18 pivote=150 y minp=150
 * estaban fuera de rango y todo se movia a tension completa sin control).
 * Compila con: ESP32C3 Dev Module, esp32 3.x. Sube con la potencia de motores DESCONECTADA.
 * Fuente de verdad del modelo: fusion_model.h (probado contra el simulador de Python).
 */
#define ENLACE_BLE 0        // 2026-10-23: como el firmware que corria (maxfrente): BLE apagado, el CABLE USB es la via principal
#if ENLACE_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#endif
#include "esp_system.h"
#include "esp_task_wdt.h"
#include <Preferences.h>

// ===== MEZCLA, RAMPAS Y RAFAGAS (antes fusion_model.h, incrustado para un solo archivo) =====
// FUSION = lo mejor del v3 (skid-steer, tension COMPLETA en rafagas = maximo par para un robot
// pesado) + lo mejor del "suave" (rampas, pausa al invertir, deadman, config por UDP).
// SIN dependencias de Arduino: se prueba en el PC contra el port de Python (simulador_pasivo).
//
// Ruedas (indices): 0=R1 delantera, 1=R2 trasera, 2=L1 delantera, 3=L2 trasera.
// Lado "IZQ" = R1+R2, lado "DER" = L1+L2 (como el original).
//  - TRASERAS (1 y 3): llevan el giro y MAS fuerza longitudinal ("tras").
//  - DELANTERAS (0 y 2): llevan el avance; toman un % del giro ("dif_del"): 0 = como el suave
//    (solo longitudinales), 100 = como el v3 (skid-steer puro, ambas del lado igual).
//  - "periodo" > 0: el motor recibe RAFAGAS de tension completa (como el v3, pero con periodo
//    corto y la envolvente suavizada por la rampa) en vez de PWM fino: vence la zona muerta de
//    un robot pesado. periodo = 0: PWM de hardware continuo (como el suave).
//  - "refuerzo" > 100: anti-hundimiento del v3 (esfuerzo sostenido => mas potencia).
#include <math.h>
#include <stdint.h>
namespace fusion {
constexpr float V_MAX = .55f, OMEGA_MAX = 1.5f;

struct Lim {            // % de duty (rampas en %/s; periodo en ms; refuerzo y dif_del en %)
  float adel = 80;      // tope hacia adelante
  float atras = 60;     // tope hacia atras
  float pivote = 100;   // potencia diferencial al girar
  float minp = 30;      // arranque minimo de la envolvente
  float acel = 200;     // rampa de subida (%/s)
  float frena = 400;    // rampa de bajada (%/s)
  float tras = 130;     // fuerza EXTRA de las traseras (tope 100 %)
  float dif_del = 0;    // % del giro que hacen las delanteras (0 = solo longitudinales)
  float periodo = 100;  // ms de la rafaga (0 = PWM continuo)
  float refuerzo = 100; // % de anti-hundimiento (100 = apagado)
  float freno_ms = 80;  // FRENO ACTIVO (ambos pines a HIGH) al parar de verdad; 0 = rueda libre como antes
};
constexpr int N_LIM = 11;
inline float clip(float x, float a, float b) { return x < a ? a : x > b ? b : x; }

inline bool limValido(const Lim &L) {
  return L.adel >= 10 && L.adel <= 100 && L.atras >= 10 && L.atras <= 100 &&
         L.pivote >= 30 && L.pivote <= 100 && L.minp >= 0 && L.minp <= 100 &&
         L.minp <= L.adel && L.minp <= L.atras && L.acel >= 20 && L.acel <= 600 &&
         L.frena >= 50 && L.frena <= 1000 && L.tras >= 100 && L.tras <= 200 &&
         L.dif_del >= 0 && L.dif_del <= 100 &&
         (L.periodo == 0 || (L.periodo >= 40 && L.periodo <= 400)) &&
         L.refuerzo >= 100 && L.refuerzo <= 130 && L.freno_ms >= 0 && L.freno_ms <= 400;
}

struct Out { float d[4]; };   // duty firmado (%), ANTES de aplicar la inversion de cada rueda

inline Out mezclar(float v, float omega, const Lim &L, float boost = 1.0f) {
  Out o = {{0, 0, 0, 0}};
  if (!isfinite(v) || !isfinite(omega)) return o;
  float fv = clip(v / V_MAX, -1, 1), wn = clip(omega / OMEGA_MAX, -1, 1);
  if (fabsf(fv) < .02f) fv = 0;
  if (fabsf(wn) < .02f) wn = 0;
  if (fv == 0 && wn == 0) return o;
  float base = fv > 0 ? fminf(fv * 100, L.adel) : -fminf(-fv * 100, L.atras);   // delanteras
  float baseR = clip(base * L.tras / 100.f, -100.f, 100.f);                      // traseras
  float dif = wn * L.pivote;
  float kd = L.dif_del / 100.f;
  float izq = baseR - dif, der = baseR + dif;        // omega>0 = giro a la izquierda
  float fi = base - kd * dif, fd = base + kd * dif;  // delanteras (R1 = izq, L1 = der)
  float m = fmaxf(fmaxf(fabsf(izq), fabsf(der)), fmaxf(fabsf(fi), fabsf(fd)));
  if (m > 100) { float s = 100 / m; izq *= s; der *= s; fi *= s; fd *= s; }
  o.d[0] = fi;  o.d[2] = fd;
  o.d[1] = izq; o.d[3] = der;
  for (int i = 0; i < 4; i++) {
    o.d[i] = clip(o.d[i] * boost, -100.f, 100.f);
    if (fabsf(o.d[i]) < .5f) o.d[i] = 0;          // residuo de redondeo (p. ej. fi = base - dif = 0)
    float a = fabsf(o.d[i]);
    if (a > 0 && a < L.minp) o.d[i] = (o.d[i] > 0 ? 1 : -1) * L.minp;   // arranque minimo
  }
  return o;
}

// Un paso de rampa de UNA rueda (dt en s). cur y target son duty firmado en %.
inline float rampa(float cur, float t, float dt, const Lim &L) {
  if (t == 0 || (cur != 0 && ((t > 0) != (cur > 0)))) {      // bajar a cero (tambien antes de invertir)
    float s = L.frena * dt;
    return fabsf(cur) <= s ? 0.f : cur - (cur > 0 ? s : -s);
  }
  if (cur == 0) cur = (t > 0 ? 1 : -1) * fminf(fabsf(t), L.minp);   // arranque desde parado
  if (fabsf(t) > fabsf(cur)) return cur + clip(t - cur, -L.acel * dt, L.acel * dt);
  return cur + clip(t - cur, -L.frena * dt, L.frena * dt);
}

// De la envolvente (duty firmado, %) a lo que recibe el motor en este instante (%):
// rafaga de tension completa o PWM continuo. t_ms = reloj en ms (comun a las 4 ruedas).
inline float rafaga(float env, uint32_t t_ms, const Lim &L) {
  if (L.periodo <= 0) return env;
  float a = fabsf(env);
  if (a <= 0) return 0;
  if (a >= 98) return env > 0 ? 100.f : -100.f;
  float fase = (float)(t_ms % (uint32_t)L.periodo);
  return fase < L.periodo * a / 100.f ? (env > 0 ? 100.f : -100.f) : 0.f;
}

// Anti-hundimiento del v3: cada 100 ms guarda el duty medio pedido; si la media de los ultimos
// 10 supera 85 %, el multiplicador pasa a refuerzo/100.
struct Refuerzo {
  float hist[10] = {0};
  int idx = 0;
  float boost = 1.0f;
  inline void muestra(float duty_medio, const Lim &L) {
    hist[idx] = duty_medio; idx = (idx + 1) % 10;
    float s = 0; for (int i = 0; i < 10; i++) s += hist[i];
    boost = (L.refuerzo > 100 && s / 10.f > 85.f) ? L.refuerzo / 100.f : 1.0f;
  }
};
}  // namespace fusion


void iniciarWatchdogLoop();
struct CmdFrame;
struct StatusFrame;

#define ENLACE_WIFI 1
#if ENLACE_WIFI
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

// ===== PINES Y RUEDAS (valores de fabrica; se cambian desde el dashboard) =====
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

// ===== INVERSIONES DE RUEDA (las que te funcionan; tambien configurables) =====
const bool INVERT_R1 = true;
const bool INVERT_R2 = true;
const bool INVERT_L1 = true;
const bool INVERT_L2 = true;
const bool CABLEADO_INVIERTE_AVANCE = false;
const bool CABLEADO_INVIERTE_GIRO   = false;

// ===== PARAMETROS DE TRANSPORTE =====
const float V_MAX     = fusion::V_MAX;
const float OMEGA_MAX = fusion::OMEGA_MAX;
const uint32_t MOTOR_HZ = 1000;            // PWM de hardware (8 bits). 10 kHz era demasiado alto: los motores no arrancaban
const unsigned long GAP_MS = 25;           // pausa con la rueda libre antes de invertir el sentido
const unsigned long DEADMAN_MS_FW = 500;  // sin ordenes en este tiempo = ruedas libres (ademas del failsafe de loop)

// ===== SEGURIDAD =====
// 2026-10-21 (como maxfrente): por WiFi 1200 ms (huecos del aire), por el cable USB 400 ms (enlace fiable).
const unsigned long CMD_TIMEOUT_MS = 1200;
const unsigned long CMD_TIMEOUT_USB_MS = 400;
unsigned long lastCmdMillis = 0;

#define SECUENCIA_VALIDA(seq_nuevo, seq_anterior) \
  (((int16_t)((uint16_t)(seq_nuevo - seq_anterior))) > 0)   /* solo una trama MAS NUEVA (WiFi lento + cable) */

// ===== SERVO + LED (canal 5: los canales 0..3 son de las ruedas) =====
const int SERVO_LEDC_CANAL   = 5;
const int SERVO_LEDC_BITS    = 14;
const int SERVO_LEDC_HZ      = 50;
const int SERVO_PULSO_MIN_US = 500;
const int SERVO_PULSO_MAX_US = 2400;
bool servoListo = false;
bool scanning = false;
unsigned long scanStartMillis = 0;
const unsigned long SCAN_SWEEP_MS = 10000;
const int SERVO_MIN_ANGLE  = 25, SERVO_MAX_ANGLE  = 160, SERVO_PARK_ANGLE = 98;
int  currentServoAngle = SERVO_PARK_ANGLE;
bool ledOn = false;

// ===== SERVO =====
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

// ===== PROTOCOLO BLE — SIN MODIFICAR =====
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
#define ST_BLE_OK    0x80   // con BLE apagado este bit significa «cable USB activo» (ST_USB_OK)
#define ST_USB_OK    0x80
const unsigned long ARRANQUE_RECIENTE_MS = 15000;
const uint8_t VIA_BLE = 1, VIA_WIFI = 2, VIA_USB = 3;

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

// ===== BLE GATT — SIN MODIFICAR =====
#define BLE_SERVICE_UUID     "a0e1b000-7f3c-4a5e-9d21-6b8e2c4f1a00"
#define BLE_CMD_CHAR_UUID    "a0e1b001-7f3c-4a5e-9d21-6b8e2c4f1a00"
#define BLE_STATUS_CHAR_UUID "a0e1b002-7f3c-4a5e-9d21-6b8e2c4f1a00"

#if ENLACE_BLE
BLEServer *pServer = nullptr;
BLECharacteristic *pStatusChar = nullptr;
bool bleConnected = false, bleWasConnected = false;

portMUX_TYPE bleMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool blePending = false;
CmdFrame bleInbox;
#else
bool bleConnected = false, bleWasConnected = false;
#endif

float lastV = 0, lastOmega = 0;
float vAplicada = 0.0f, omegaAplicada = 0.0f;
uint8_t lastServo = 0;
bool lastLed = false;
uint16_t lastSeq = 0;
unsigned long tramasMalas = 0, tramasBle = 0, tramasWifi = 0, tramasViejas = 0, tramasUsb = 0;
uint8_t viaUltima = 0;
bool inFailsafe = false;

// ===== WIFI UDP — SIN MODIFICAR =====
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
// 2026-10-21 (maxfrente): arrancaba en el indice 0 (8,5 dBm) y ahi se quedaba asociada: el AP la veia a -86/-89 dBm.
// Se arranca en 17 dBm (indice 2). Si la placa se reinicia al conectar (bateria baja), bajar a 1 (13 dBm).
const uint8_t WIFI_POTENCIA_INICIAL_IDX = 2;
const bool    WIFI_SIN_AHORRO = true;   // sin modem-sleep: menos latencia y menos tramas perdidas
uint8_t wifiPotenciaIdx = WIFI_POTENCIA_INICIAL_IDX, wifiIntentos = 0;
WiFiUDP udpC3;
bool wifiListo = false, udpAbierto = false;
IPAddress ipJetson;
#endif

volatile bool reabrirAnuncio = false;
esp_reset_reason_t motivoReinicio = ESP_RST_UNKNOWN;

#if ENLACE_BLE
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
    s->requestConnParams(desc->conn_handle, BLE_INTERVALO_MIN, BLE_INTERVALO_MAX, 0, BLE_SUPERVISION);
  }
#elif defined(CONFIG_BLUEDROID_ENABLED)
  void onConnect(BLEServer *s, esp_ble_gatts_cb_param_t *param) override {
    s->updateConnParams(param->connect.remote_bda, BLE_INTERVALO_MIN, BLE_INTERVALO_MAX, 0, BLE_SUPERVISION);
  }
#endif
  void onDisconnect(BLEServer *s) override {
    bleConnected = false;
    reabrirAnuncio = true;
    Serial.println("[ble] cliente desconectado");
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
#endif


// ===== MOTORES: PWM por hardware, rampas, delanteras solo longitudinales =====
// Indices de rueda: 0=R1 (delantera)  1=R2 (trasera)  2=L1 (delantera)  3=L2 (trasera)
// Lado "IZQ" = R1+R2, lado "DER" = L1+L2 (convencion del firmware original).
const uint8_t DEF_FI[4] = {R1_FI, R2_FI, L1_FI, L2_FI};
const uint8_t DEF_BI[4] = {R1_BI, R2_BI, L1_BI, L2_BI};
const bool DEF_INV[4]   = {INVERT_R1, INVERT_R2, INVERT_L1, INVERT_L2};
uint8_t pinFI[4] = {R1_FI, R2_FI, L1_FI, L2_FI};
uint8_t pinBI[4] = {R1_BI, R2_BI, L1_BI, L2_BI};
bool invertida[4] = {INVERT_R1, INVERT_R2, INVERT_L1, INVERT_L2};

Preferences prefs;
fusion::Refuerzo refuerzo;                              // anti-hundimiento del v3 (solo si lim.refuerzo > 100)
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;       // loop() <-> motorTask()
fusion::Lim lim;                                         // limites vigentes (protegidos por mux)
float targets[4] = {}, actual[4] = {};                  // duty firmado objetivo / aplicado (%)
uint32_t refreshedAt = 0;
bool timedOut = true, hwFault = false, hold = false, killNow = false;
// prueba de una rueda pedida desde el dashboard
uint32_t testHasta = 0;
uint8_t testRueda = 0;
float testDuty = 0;

static bool ensayoActivo() { return testHasta && (int32_t)(testHasta - millis()) > 0; }

static void setTargets(const float *t) {
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < 4; i++) targets[i] = t[i];
  refreshedAt = millis();
  portEXIT_CRITICAL(&mux);
}

// Parada NORMAL: las ruedas bajan por rampa.
void stopMotors() {
  float z[4] = {};
  setTargets(z);
  vAplicada = 0.0f;
  omegaAplicada = 0.0f;
}

// Parada de EMERGENCIA: ruedas libres al instante (failsafe, cambio de configuracion).
static void soltarYa() {
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < 4; i++) targets[i] = 0;
  killNow = true;
  portEXIT_CRITICAL(&mux);
}

// ---- tarea de motores (cada 10 ms): rampas + PWM de hardware + deadman ----
static int8_t mPin[4] = {-1, -1, -1, -1};
// FRENO ACTIVO (2026-10-23): el TA6586 frena con ambos pines a HIGH. Se frena al PARAR de verdad, no en cada hueco del troceo
// de la Jetson (suavidad_duty: ventanas apagadas de ~130 ms): hace falta FRENO_ESPERA_MS seguidos con orden cero. Un corte
// de emergencia/failsafe frena al instante. Sin frenar el robot a 100 % seguia rodando 35-40 cm tras cada pulso.
const uint32_t FRENO_ESPERA_MS = 200;
static uint32_t mFrenoHasta[4] = {};
static uint32_t mCeroDesde[4] = {};
static bool mFrenado[4] = {};
static float mCur[4] = {};
static uint32_t mZeroAt[4] = {};

static void soltarRueda(int i);
static void frenarRueda(int i, uint32_t now, uint32_t ms) {   // ambos pines a HIGH = freno del puente
  soltarRueda(i);
  digitalWrite(pinFI[i], HIGH); digitalWrite(pinBI[i], HIGH);
  mFrenoHasta[i] = now + ms;
}

static void soltarRueda(int i) {      // ambos pines a LOW = rueda libre
  if (mPin[i] >= 0) { ledcWriteChannel(i, 0); ledcDetach(mPin[i]); mPin[i] = -1; }
  pinMode(pinFI[i], OUTPUT); pinMode(pinBI[i], OUTPUT);
  digitalWrite(pinFI[i], LOW); digitalWrite(pinBI[i], LOW);
}

void motorTask(void *) {
  TickType_t wake = xTaskGetTickCount();
  bool libre[4] = {true, true, true, true};
  for (;;) {
    uint32_t now = millis();
    fusion::Lim L; float want[4]; bool inmediato;
    portENTER_CRITICAL(&mux);
    L = lim;
    timedOut = (uint32_t)(now - refreshedAt) > DEADMAN_MS_FW;
    bool ensayo = ensayoActivo();
    inmediato = hwFault || hold || killNow || (timedOut && !ensayo);
    killNow = false;
    for (int i = 0; i < 4; i++) want[i] = ensayo ? (i == testRueda ? testDuty : 0.f) : targets[i];
    portEXIT_CRITICAL(&mux);

    for (int i = 0; i < 4; i++) {
      // --- freno activo en curso: se mantiene mientras no haya orden de moverse y no haya vencido ---
      if (mFrenoHasta[i]) {
        bool sigue = (int32_t)(mFrenoHasta[i] - now) > 0 && want[i] == 0 && !hwFault && !hold;
        if (!sigue) { soltarRueda(i); mFrenoHasta[i] = 0; mZeroAt[i] = now; }
        else { mCur[i] = 0; portENTER_CRITICAL(&mux); actual[i] = 0; portEXIT_CRITICAL(&mux); continue; }
      }
      if (want[i] != 0) { mCeroDesde[i] = 0; mFrenado[i] = false; }
      else if (mCeroDesde[i] == 0) mCeroDesde[i] = now ? now : 1;
      if (inmediato) {
        mCur[i] = 0;
        if (!libre[i]) {
          if (L.freno_ms > 0 && !hwFault && !hold) { frenarRueda(i, now, (uint32_t)L.freno_ms); mFrenado[i] = true; }
          else soltarRueda(i);
          libre[i] = true; mZeroAt[i] = now;
        }
      } else {
        float t = want[i];
        if (mCur[i] == 0 && t != 0 && (uint32_t)(now - mZeroAt[i]) < GAP_MS) t = 0;   // pausa al invertir
        mCur[i] = fusion::rampa(mCur[i], t, 0.01f, L);
        if (mCur[i] == 0) {
          if (!libre[i]) { soltarRueda(i); libre[i] = true; mZeroAt[i] = now; }
          // parada de verdad (orden cero sostenida): frena un momento en vez de dejar la rueda libre
          if (L.freno_ms > 0 && !mFrenado[i] && want[i] == 0 && mCeroDesde[i] && (uint32_t)(now - mCeroDesde[i]) >= FRENO_ESPERA_MS
              && !hwFault && !hold && mFrenoHasta[i] == 0) {
            frenarRueda(i, now, (uint32_t)L.freno_ms); mFrenado[i] = true;
          }
        } else {
          int pin = mCur[i] > 0 ? pinFI[i] : pinBI[i];   // el sentido lo da la envolvente
          if (mPin[i] != pin) {                       // arranque o cambio de sentido
            soltarRueda(i);
            if (!ledcAttachChannel(pin, MOTOR_HZ, 8, i)) {
              portENTER_CRITICAL(&mux); hwFault = true; portEXIT_CRITICAL(&mux);
              soltarRueda(i); mCur[i] = 0; continue;
            }
            mPin[i] = pin;
          }
          libre[i] = false;
          // Rafaga: tension completa o nada segun la fase; el pin queda asignado (fase apagada =
          // duty 0 = rueda libre) para no hacer attach/detach en cada ciclo.
          float salida = fabsf(fusion::rafaga(mCur[i], now, L));
          ledcWriteChannel(i, lroundf(fminf(salida, 100.f) * 255.f / 100.f));
        }
      }
      portENTER_CRITICAL(&mux); actual[i] = mCur[i]; portEXIT_CRITICAL(&mux);
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
  }
}

// Mezcla (v, omega) -> duty por rueda, con la inversion de cada una.
void aplicarMezcla(float v, float omega) {
  fusion::Lim L;
  portENTER_CRITICAL(&mux); L = lim; portEXIT_CRITICAL(&mux);
  fusion::Out o = fusion::mezclar(v, omega, L, refuerzo.boost);
  float t[4];
  for (int i = 0; i < 4; i++) t[i] = invertida[i] ? -o.d[i] : o.d[i];
  setTargets(t);
}

// ===== CONFIGURACION DESDE EL DASHBOARD (UDP, sin web ni IP propia) =====
// Orden: [0xC5][op][a][b][c][d][e][crc]. Respuesta de ruedas 0xC6, de limites 0xC7.
const uint8_t CFG_MAGIC = 0xC5, CFG_REPLY_MAGIC = 0xC6, LIM_REPLY_MAGIC = 0xC7, FW_VERSION = 5;
enum CfgOp : uint8_t { OP_SET = 1, OP_TEST = 2, OP_RESET = 3, OP_GET = 4, OP_SWAP = 5,
                       OP_LIM_SET = 6, OP_LIM_GET = 7, OP_LIM_RESET = 8 };
const unsigned long TEST_MS = 400;     // la prueba se corta sola si el dashboard deja de repetirla

static float *limCampo(fusion::Lim &L, int id) {
  float *c[fusion::N_LIM] = {&L.adel, &L.atras, &L.pivote, &L.minp, &L.acel, &L.frena, &L.tras,
                             &L.dif_del, &L.periodo, &L.refuerzo, &L.freno_ms};
  return id >= 0 && id < fusion::N_LIM ? c[id] : nullptr;
}

static bool pinesValidos(const uint8_t *fi, const uint8_t *bi) {
  unsigned mask = 0;
  for (int i = 0; i < 4; i++) {
    if (fi[i] > 7 || bi[i] > 7 || fi[i] == bi[i]) return false;
    if ((mask & (1u << fi[i])) || (mask & (1u << bi[i]))) return false;
    mask |= (1u << fi[i]) | (1u << bi[i]);
  }
  return true;
}

// Cambia pines/inversion con las ruedas LIBRES (la tarea de motores queda en pausa).
static void aplicarPines(const uint8_t *fi, const uint8_t *bi, const bool *inv) {
  portENTER_CRITICAL(&mux); hold = true; testHasta = 0;
  for (int i = 0; i < 4; i++) targets[i] = 0;
  portEXIT_CRITICAL(&mux);
  delay(40);                                           // la tarea suelta las 4 ruedas
  for (int i = 0; i < 4; i++) {
    digitalWrite(pinFI[i], LOW); digitalWrite(pinBI[i], LOW);
    pinFI[i] = fi[i]; pinBI[i] = bi[i]; invertida[i] = inv[i];
    pinMode(fi[i], OUTPUT); pinMode(bi[i], OUTPUT);
    digitalWrite(fi[i], LOW); digitalWrite(bi[i], LOW);
  }
  portENTER_CRITICAL(&mux); hold = false; portEXIT_CRITICAL(&mux);
}

static void guardarRueda(int i) {
  char k[8];
  snprintf(k, sizeof(k), "fi%d", i); prefs.putUChar(k, pinFI[i]);
  snprintf(k, sizeof(k), "bi%d", i); prefs.putUChar(k, pinBI[i]);
  snprintf(k, sizeof(k), "iv%d", i); prefs.putBool(k, invertida[i]);
}

static void guardarLimite(int id, float v) {
  char k[6]; snprintf(k, sizeof(k), "l%d", id); prefs.putUShort(k, (uint16_t)lroundf(v));
}

static void cargarConfig() {
  uint8_t fi[4], bi[4]; bool inv[4]; char k[8];
  for (int i = 0; i < 4; i++) {
    snprintf(k, sizeof(k), "fi%d", i); fi[i] = prefs.getUChar(k, DEF_FI[i]);
    snprintf(k, sizeof(k), "bi%d", i); bi[i] = prefs.getUChar(k, DEF_BI[i]);
    snprintf(k, sizeof(k), "iv%d", i); inv[i] = prefs.getBool(k, DEF_INV[i]);
  }
  if (!pinesValidos(fi, bi)) {
    Serial.println("[cfg] pines guardados invalidos: de fabrica");
    memcpy(fi, DEF_FI, 4); memcpy(bi, DEF_BI, 4); memcpy(inv, DEF_INV, 4);
  }
  aplicarPines(fi, bi, inv);
  fusion::Lim L;                                        // limites: todo o nada
  for (int id = 0; id < fusion::N_LIM; id++) {
    snprintf(k, sizeof(k), "l%d", id);
    float def = *limCampo(L, id);
    *limCampo(L, id) = prefs.isKey(k) ? (float)prefs.getUShort(k, (uint16_t)def) : def;
  }
  if (fusion::limValido(L)) { portENTER_CRITICAL(&mux); lim = L; portEXIT_CRITICAL(&mux); }
  else Serial.println("[cfg] limites guardados invalidos: de fabrica");
}

void enterFailsafe(const char *motivo) {
  inFailsafe = true;
  lastV = 0;
  lastOmega = 0;
  vAplicada = omegaAplicada = 0;
  soltarYa();
  scanning = false;
  setLed(FOCO_APAGADO_FW);
  lastLed = FOCO_APAGADO_FW;
  Serial.print("FAILSAFE: ");
  Serial.println(motivo);
}

// ===== APLICA COMANDO (BLE o WiFi) =====
bool aplicarComando(const CmdFrame &c, uint8_t via) {
  unsigned long ahora = millis();

  if (!inFailsafe && !SECUENCIA_VALIDA(c.seq, lastSeq)) {
    tramasViejas++;
    return false;
  }

  lastCmdMillis = ahora;
  lastSeq   = c.seq;
  viaUltima = via;
  if (via == VIA_WIFI) tramasWifi++; else if (via == VIA_USB) tramasUsb++; else tramasBle++;

  lastV     = (constrain((int)c.v_pct,     -100, 100) / 100.0f) * V_MAX;
  lastOmega = (constrain((int)c.omega_pct, -100, 100) / 100.0f) * OMEGA_MAX;
  lastServo = c.servo_angle;
  lastLed   = (c.flags & FLAG_LED) != 0;

  inFailsafe = false;

  vAplicada     = CABLEADO_INVIERTE_AVANCE ? -lastV : lastV;
  omegaAplicada = CABLEADO_INVIERTE_GIRO ? -lastOmega : lastOmega;
  if (!ensayoActivo()) aplicarMezcla(vAplicada, omegaAplicada);   // en prueba de rueda no se pisa

  if ((c.flags & FLAG_SCAN) && !scanning) {
    scanning = true;
    scanStartMillis = millis();
  }
  setLed(lastLed);
  if (!scanning) setServo(lastServo);
  return true;
}

#if ENLACE_BLE
void procesarBLE() {
  if (!blePending) return;
  CmdFrame c;
  portENTER_CRITICAL(&bleMux);
  memcpy(&c, &bleInbox, sizeof(c));
  blePending = false;
  portEXIT_CRITICAL(&bleMux);
  aplicarComando(c, VIA_BLE);
}
#endif

#if ENLACE_WIFI
// Respuesta de rueda: [0xC6][rueda][fi][bi][invertida][ok][version][crc]
static void enviarCfg(uint8_t rueda, uint8_t ok) {
  if (!wifiListo || !udpAbierto || rueda > 3) return;
  uint8_t f[8] = {CFG_REPLY_MAGIC, rueda, pinFI[rueda], pinBI[rueda], (uint8_t)invertida[rueda], ok, FW_VERSION, 0};
  f[7] = crc8(f, 7);
  udpC3.beginPacket(ipJetson, JETSON_UDP_PUERTO); udpC3.write(f, 8); udpC3.endPacket();
}

// Respuesta de limite: [0xC7][id][valor lo][valor hi][ok][version][0][crc]
static void enviarLim(uint8_t id, uint8_t ok) {
  if (!wifiListo || !udpAbierto || id >= fusion::N_LIM) return;
  fusion::Lim L; portENTER_CRITICAL(&mux); L = lim; portEXIT_CRITICAL(&mux);
  uint16_t v = (uint16_t)lroundf(*limCampo(L, id));
  uint8_t f[8] = {LIM_REPLY_MAGIC, id, (uint8_t)(v & 0xFF), (uint8_t)(v >> 8), ok, FW_VERSION, 0, 0};
  f[7] = crc8(f, 7);
  udpC3.beginPacket(ipJetson, JETSON_UDP_PUERTO); udpC3.write(f, 8); udpC3.endPacket();
}

static void procesarCfg(const uint8_t *b) {
  switch (b[1]) {
    case OP_SET: {                          // a=rueda b=FI c=BI d=invertida
      uint8_t w = b[2];
      if (w > 3) return;
      uint8_t fi[4], bi[4]; bool inv[4];
      for (int i = 0; i < 4; i++) { fi[i] = pinFI[i]; bi[i] = pinBI[i]; inv[i] = invertida[i]; }
      fi[w] = b[3]; bi[w] = b[4]; inv[w] = b[5] != 0;
      if (pinesValidos(fi, bi)) {
        aplicarPines(fi, bi, inv);
        for (int i = 0; i < 4; i++) guardarRueda(i);
        enviarCfg(w, 1);
      } else {
        enviarCfg(w, 0);
      }
      break;
    }
    case OP_TEST: {                         // a=rueda b=0 parar|1 adelante|2 atras c=potencia %
      uint8_t w = b[2];
      if (w > 3 || b[3] == 0 || b[3] > 2) { portENTER_CRITICAL(&mux); testHasta = 0; portEXIT_CRITICAL(&mux); return; }
      float duty = constrain((int)b[4], 0, 100) * (b[3] == 1 ? 1.f : -1.f) * (invertida[w] ? -1.f : 1.f);
      portENTER_CRITICAL(&mux);
      testRueda = w; testDuty = duty; testHasta = millis() + TEST_MS;
      portEXIT_CRITICAL(&mux);
      break;
    }
    case OP_RESET: {
      aplicarPines(DEF_FI, DEF_BI, DEF_INV);
      for (int i = 0; i < 4; i++) { guardarRueda(i); enviarCfg(i, 1); }
      break;
    }
    case OP_GET:
      for (int i = 0; i < 4; i++) enviarCfg(i, 1);
      break;
    case OP_SWAP: {                         // a,b = ruedas que intercambian sus pines FI/BI
      uint8_t x = b[2], y = b[3];
      if (x > 3 || y > 3 || x == y) return;
      uint8_t fi[4], bi[4]; bool inv[4];
      for (int i = 0; i < 4; i++) { fi[i] = pinFI[i]; bi[i] = pinBI[i]; inv[i] = invertida[i]; }
      uint8_t t = fi[x]; fi[x] = fi[y]; fi[y] = t;
      t = bi[x]; bi[x] = bi[y]; bi[y] = t;
      aplicarPines(fi, bi, inv);
      for (int i = 0; i < 4; i++) guardarRueda(i);
      enviarCfg(x, 1); enviarCfg(y, 1);
      break;
    }
    case OP_LIM_SET: {                      // a=id  b,c = valor (uint16 little-endian)
      fusion::Lim L; portENTER_CRITICAL(&mux); L = lim; portEXIT_CRITICAL(&mux);
      float *campo = limCampo(L, b[2]);
      if (!campo) return;
      *campo = (float)(b[3] | (b[4] << 8));
      if (fusion::limValido(L)) {                  // todo o nada
        soltarYa();                                // cambiar limites detiene el robot
        portENTER_CRITICAL(&mux); lim = L; portEXIT_CRITICAL(&mux);
        guardarLimite(b[2], *campo);
        enviarLim(b[2], 1);
      } else {
        enviarLim(b[2], 0);                        // rechazado: se devuelve el valor vigente
      }
      break;
    }
    case OP_LIM_GET:
      for (int i = 0; i < fusion::N_LIM; i++) enviarLim(i, 1);
      break;
    case OP_LIM_RESET: {
      fusion::Lim d; soltarYa();
      portENTER_CRITICAL(&mux); lim = d; portEXIT_CRITICAL(&mux);
      for (int i = 0; i < fusion::N_LIM; i++) { guardarLimite(i, *limCampo(d, i)); enviarLim(i, 1); }
      break;
    }
  }
}

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
    int leidos = udpC3.read((uint8_t *)&c, sizeof(c));
    if (leidos == (int)sizeof(c) && c.magic == CFG_MAGIC
        && crc8((const uint8_t *)&c, sizeof(c) - 1) == c.crc) {
      procesarCfg((const uint8_t *)&c);
      continue;
    }
    if (leidos != (int)sizeof(c)
        || c.magic != CMD_MAGIC
        || crc8((const uint8_t *)&c, sizeof(c) - 1) != c.crc) {
      tramasMalas++;
      continue;
    }
    aplicarComando(c, VIA_WIFI);
  }
}

void gestionarWifi() {
  static unsigned long ultimoIntento = 0;
  bool asociada = (WiFi.status() == WL_CONNECTED);
  if (asociada && !wifiListo) {
    wifiListo = true;
    ipJetson  = WiFi.gatewayIP();
    udpAbierto = udpC3.begin(C3_UDP_PUERTO);
    wifiIntentos = 0;
    Serial.printf("[wifi] OK IP=%s Jetson=%s\n",
                  WiFi.localIP().toString().c_str(),
                  ipJetson.toString().c_str());
  } else if (!asociada && wifiListo) {
    wifiListo  = false;
    if (udpAbierto) udpC3.stop();
    udpAbierto = false;
    ultimoIntento = millis();
    Serial.println("[wifi] perdida");
  }
  if (!asociada && millis() - ultimoIntento > WIFI_REINTENTO_MS) {
    ultimoIntento = millis();
    if (++wifiIntentos > WIFI_INTENTOS_POR_POTENCIA) {
      wifiIntentos = 1;
      wifiPotenciaIdx = (wifiPotenciaIdx + 1) % WIFI_N_POTENCIAS;
    }
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.setTxPower(WIFI_POTENCIAS[wifiPotenciaIdx]);
    if (WIFI_SIN_AHORRO) WiFi.setSleep(false);
  }
}
#endif

// ===== CABLE USB (CDC) Jetson <-> C3: mismas tramas binarias de 8 bytes que el WiFi (CmdFrame / StatusFrame) =====
// Canal fiable: no depende del aire. Cada trama valida se contesta AL INSTANTE con un estado (eco de la secuencia).
// Por el puerto serie solo viajan tramas binarias (nada de texto tras el arranque); se resincroniza por byte de inicio + CRC.
unsigned long ultimoCmdUsb = 0;
uint8_t usbBuf[sizeof(CmdFrame)];
uint8_t usbN = 0;
bool usbActivo() { return ultimoCmdUsb != 0 && millis() - ultimoCmdUsb < 1500; }

void armarEstado(StatusFrame &st) {
#if ENLACE_WIFI
  bool porWifi = wifiListo && udpAbierto;
#else
  bool porWifi = false;
#endif
  st.magic     = STATUS_MAGIC;
  st.v_pct     = (int8_t)constrain((int)lroundf(lastV     / V_MAX     * 100.0f), -100, 100);
  st.omega_pct = (int8_t)constrain((int)lroundf(lastOmega / OMEGA_MAX * 100.0f), -100, 100);
  st.servo_angle = (uint8_t)currentServoAngle;
  st.flags = 0;
  if (ledOn)      st.flags |= ST_LED;
  if (scanning)   st.flags |= ST_SCANNING;
  if (inFailsafe) st.flags |= ST_FAILSAFE;
  if (motivoReinicio == ESP_RST_BROWNOUT)  st.flags |= ST_BROWNOUT;
  if (millis() < ARRANQUE_RECIENTE_MS)     st.flags |= ST_RECIEN;
  if (viaUltima == VIA_WIFI)               st.flags |= ST_VIA_WIFI;
  if (porWifi)                             st.flags |= ST_WIFI_OK;
#if ENLACE_BLE
  if (bleConnected)                        st.flags |= ST_BLE_OK;
#else
  if (usbActivo())                         st.flags |= ST_USB_OK;
#endif
  st.seq_echo = lastSeq;
  st.crc = crc8((uint8_t *)&st, sizeof(st) - 1);
}

void enviarEstadoUsb() {
  StatusFrame st;
  armarEstado(st);
  Serial.write((const uint8_t *)&st, sizeof(st));
}

void procesarUsb() {
  int presupuesto = 96;
  while (presupuesto-- > 0 && Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    if (usbN == 0 && b != CMD_MAGIC) continue;           // basura o texto: se espera el byte de inicio
    usbBuf[usbN++] = b;
    if (usbN < sizeof(CmdFrame)) continue;
    CmdFrame c;
    memcpy(&c, usbBuf, sizeof(c));
    if (crc8(usbBuf, sizeof(c) - 1) == c.crc) {
      usbN = 0;
      ultimoCmdUsb = millis();
      aplicarComando(c, VIA_USB);
      enviarEstadoUsb();                                  // eco inmediato (latencia de ida y vuelta medible)
    } else {
      tramasMalas++;
      uint8_t resto = 0;                                  // resincroniza: busca otro inicio dentro de lo que quedo
      for (uint8_t i = 1; i < sizeof(CmdFrame); i++) {
        if (usbBuf[i] == CMD_MAGIC) { memmove(usbBuf, usbBuf + i, sizeof(CmdFrame) - i); resto = sizeof(CmdFrame) - i; break; }
      }
      usbN = resto;
    }
  }
}

void enviarEstado() {
#if ENLACE_BLE
  bool porBle = bleConnected && pStatusChar != nullptr;
#endif
#if ENLACE_WIFI
  bool porWifi = wifiListo && udpAbierto;
#else
  bool porWifi = false;
#endif
  StatusFrame st;
  armarEstado(st);
#if ENLACE_BLE
  if (porBle) {
    pStatusChar->setValue((uint8_t *)&st, sizeof(st));
    pStatusChar->notify();
  }
#endif
#if ENLACE_WIFI
  if (porWifi) {
    udpC3.beginPacket(ipJetson, JETSON_UDP_PUERTO);
    udpC3.write((const uint8_t *)&st, sizeof(st));
    udpC3.endPacket();
  }
#endif
  if (usbActivo()) enviarEstadoUsb();                    // latido del estado por el cable aunque no lleguen ordenes
}

void setup() {
  const int pinesMotor[] = {R1_FI, R1_BI, R2_FI, R2_BI, L1_FI, L1_BI, L2_FI, L2_BI};
  for (int p : pinesMotor) {                 // ninguna rueda se mueve al encender
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  prefs.begin("agro-pines", false);
  cargarConfig();

  pinMode(LED_PIN, OUTPUT);
  ledOn = !FOCO_APAGADO_FW;
  setLed(FOCO_APAGADO_FW);
  motivoReinicio = esp_reset_reason();

  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);                       // si la Jetson no lee el USB, escribir no debe frenar el lazo
  unsigned long t0Serial = millis();
  while (!Serial && millis() - t0Serial < 1500) delay(10);
  delay(200);

  Serial.println();
  Serial.println("=== AGROBOT C3 FUSION v5 — rafagas de tension completa + rampas ===");

  if (xTaskCreate(motorTask, "motor", 4096, nullptr, 3, nullptr) != pdPASS) hwFault = true;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  servoListo = ledcAttachChannel(SERVO_PIN, SERVO_LEDC_HZ, SERVO_LEDC_BITS, SERVO_LEDC_CANAL);
#else
  servoListo = (ledcSetup(SERVO_LEDC_CANAL, SERVO_LEDC_HZ, SERVO_LEDC_BITS) != 0);
  if (servoListo) ledcAttachPin(SERVO_PIN, SERVO_LEDC_CANAL);
#endif
  servoEscribir(SERVO_PARK_ANGLE);
  currentServoAngle = SERVO_PARK_ANGLE;

#if ENLACE_BLE
  BLEDevice::init("AGROBOT-C3");
  BLEDevice::setPower(ESP_PWR_LVL_P9);
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
  Serial.println("BLE activo");
#else
  Serial.println("BLE apagado (WiFi + cable USB)");
#endif

#if ENLACE_WIFI
  WiFi.setHostname("agrobot-c3");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setTxPower(WIFI_POTENCIAS[wifiPotenciaIdx]);
  if (WIFI_SIN_AHORRO) WiFi.setSleep(false);
  Serial.println("WiFi asociando...");
#endif

  lastCmdMillis = millis();
  iniciarWatchdogLoop();
}

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

unsigned long ultimoEstadoBle = 0;
const unsigned long ESTADO_PERIODO_BLE_MS = 100;

void loop() {
  procesarUsb();
#if ENLACE_BLE
  procesarBLE();
#endif
#if ENLACE_WIFI
  procesarWifi();
  gestionarWifi();
#endif

#if ENLACE_BLE
  static unsigned long ultimoRearme = 0;
  if (!bleConnected && (reabrirAnuncio || millis() - ultimoRearme > 5000)) {
    reabrirAnuncio = false;
    ultimoRearme   = millis();
    delay(20);
    pServer->startAdvertising();
  }
  bleWasConnected = bleConnected;
#endif

  const unsigned long timeoutActual = (viaUltima == VIA_USB) ? CMD_TIMEOUT_USB_MS : CMD_TIMEOUT_MS;
  if (!inFailsafe && millis() - lastCmdMillis > timeoutActual) {
    enterFailsafe("sin comando");
  }

  // Actualización de sistemas
  updateServo();
  static unsigned long ultimoRefuerzo = 0;                 // anti-hundimiento: muestra cada 100 ms
  if (millis() - ultimoRefuerzo >= 100) {
    ultimoRefuerzo = millis();
    fusion::Lim L; float a[4];
    portENTER_CRITICAL(&mux); L = lim; for (int i = 0; i < 4; i++) a[i] = actual[i]; portEXIT_CRITICAL(&mux);
    refuerzo.muestra((fabsf(a[0]) + fabsf(a[1]) + fabsf(a[2]) + fabsf(a[3])) / 4.f, L);
  }

  // (la traza de texto cada 500 ms se quito: el puerto serie lleva tramas binarias hacia la Jetson)

  if (millis() - ultimoEstadoBle >= ESTADO_PERIODO_BLE_MS) {
    ultimoEstadoBle = millis();
    enviarEstado();
  }
}

