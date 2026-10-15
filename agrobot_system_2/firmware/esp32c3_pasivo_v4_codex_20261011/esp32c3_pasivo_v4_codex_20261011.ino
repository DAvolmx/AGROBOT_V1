// AGROBOT v4: experimental, motor outputs locked until hardware profile verified.

#include <cmath>
#include <cstdint>
namespace pasivo {
// Unlock only after mapping, driver PWM truth table and geometry verification.
constexpr bool PROFILE_VERIFIED=false;
constexpr float WHEELBASE_M=.37f;
constexpr float REAR_OUTSIDE_M=.37f, FRONT_OUTSIDE_M=.19f;
constexpr float TRACK_CENTERS_M=.30f; // Placeholder, NOT measured!
constexpr float MAX_V=.12f, MAX_REVERSE=.06f;
// Operational margin below mechanical +/-20 deg. Does NOT measure passive yaw.
constexpr float MAX_CURVATURE=.826299f; // tan(17 deg)/0.37, approx.
inline bool newer(uint16_t n,uint16_t old){uint16_t d=n-old;return d!=0 && d<32768;}
inline float clamp(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
inline float slew(float v,float goal,float step){return v+clamp(goal-v,-step,step);}
struct Target{float v,w,left,right;};
inline Target mix(float v,float w){
 if(!std::isfinite(v)||!std::isfinite(w))return {0,0,0,0};
 v=clamp(v,-MAX_REVERSE,MAX_V);
 if(std::fabs(v)<.002f)return {0,0,0,0};
 w=clamp(w,-std::fabs(v)*MAX_CURVATURE,std::fabs(v)*MAX_CURVATURE);
 return {v,w,v-w*TRACK_CENTERS_M*.5f,v+w*TRACK_CENTERS_M*.5f};
}
}

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

// ===== PINES Y RUEDAS =====
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
const bool FOCO_APAGADO_FW = false;
#define LED_NIVEL(encendido) \
  ((LED_ACTIVO_EN_BAJO ? !(encendido) : (encendido)) ? HIGH : LOW)

// ===== INVERSIONES DE RUEDA =====
const bool INVERT_R1 = false;
const bool INVERT_R2 = false;
const bool INVERT_L1 = false;
const bool INVERT_L2 = false;
const bool CABLEADO_INVIERTE_AVANCE = false;
const bool CABLEADO_INVIERTE_GIRO   = false;

// ===== PARÁMETROS FÍSICOS =====
const float WHEEL_RADIUS = 0.06f;
const float TRACK_WIDTH = pasivo::TRACK_CENTERS_M; // NOT exterior width
const float V_MAX        = 0.55f;
const float OMEGA_MAX    = 1.5f;
const int   PWM_MAX      = 100;

// ===== CONTROL PWM PURO (sin pulsos, sin rampa) =====
const int PWM_MIN = 0;
const bool FRENO_ACTIVO = true;
const unsigned long DEAD_TIME_MS = 20;
const unsigned long MOTOR_PERIODO_MS = 200;
static unsigned long cicloMotoraAt = 0;

// ===== SEGURIDAD =====
const unsigned long CMD_TIMEOUT_MS = 500;
unsigned long lastCmdMillis = 0;

#define SECUENCIA_VALIDA(seq_nuevo, seq_anterior) \
  (((uint16_t)(seq_nuevo - seq_anterior)) != 0)

// ===== SERVO + LED =====
const int SERVO_LEDC_CANAL   = 0;
const int SERVO_LEDC_BITS    = 14;
const int SERVO_LEDC_HZ      = 50;
const int SERVO_PULSO_MIN_US = 500;
const int SERVO_PULSO_MAX_US = 2400;
bool servoListo = false;
bool scanning = false;
bool scanRequested = false;
unsigned long scanStartMillis = 0;
const unsigned long SCAN_SWEEP_MS = 10000;
const int SERVO_MIN_ANGLE  = 25, SERVO_MAX_ANGLE  = 110, SERVO_PARK_ANGLE = 98;
int  currentServoAngle = SERVO_PARK_ANGLE;
bool ledOn = false;

// ===== ESTRUCTURA DE RUEDA =====
enum RuedaCanal : uint8_t { CH_R1 = 2, CH_R2 = 3, CH_L1 = 4, CH_L2 = 5 };

struct Rueda {
  const uint8_t pinFI, pinBI, canal;
  const bool invertida;
  int8_t sentido;
  int dutyPedido;
  bool dcCompleto;
  unsigned long frenohasta;
  int8_t sentidoAnterior;
};

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

// ===== BLE GATT — SIN MODIFICAR =====
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
bool inFailsafe = true;
bool sequenceSeen = false;
bool hardwareFault = false;

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
uint8_t wifiPotenciaIdx = 0, wifiIntentos = 0;
WiFiUDP udpC3;
bool wifiListo = false, udpAbierto = false;
IPAddress ipJetson;
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
    auto val = c->getValue();
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

// ===== MOTORES =====
Rueda ruedaR1 = {R1_FI, R1_BI, CH_R1, INVERT_R1, 0, 0, false, 0, 0};
Rueda ruedaR2 = {R2_FI, R2_BI, CH_R2, INVERT_R2, 0, 0, false, 0, 0};
Rueda ruedaL1 = {L1_FI, L1_BI, CH_L1, INVERT_L1, 0, 0, false, 0, 0};
Rueda ruedaL2 = {L2_FI, L2_BI, CH_L2, INVERT_L2, 0, 0, false, 0, 0};
Rueda *RUEDAS[4]   = {&ruedaR1, &ruedaR2, &ruedaL1, &ruedaL2};
Rueda *LADO_DER[2] = {&ruedaL2, &ruedaL1};  // DER = L2/L1 (invertido)
Rueda *LADO_IZQ[2] = {&ruedaR2, &ruedaR1};  // IZQ = R2/R1 (invertido)

static void motorSetup(uint8_t pinFI, uint8_t pinBI) {
  pinMode(pinFI, OUTPUT);
  pinMode(pinBI, OUTPUT);
  digitalWrite(pinFI, LOW);
  digitalWrite(pinBI, LOW);
}

static void motorON(uint8_t pinActivo)  { digitalWrite(pinActivo, HIGH); }
static void motorOFF(uint8_t pinActivo) { digitalWrite(pinActivo, LOW);  }
static void motorFreno(uint8_t pinFI, uint8_t pinBI) {
  digitalWrite(pinFI, LOW);
  digitalWrite(pinBI, LOW);
}

// Four motor channels 2..5; servo channel 0. One PWM pin per bridge.
// Both bridge inputs LOW means de-energized; braking depends on driver truth table.
static int attachedPin[6] = {-1,-1,-1,-1,-1,-1};
static void detachMotor(Rueda &r) {
  int pin=attachedPin[r.canal];
  if(pin>=0) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWriteChannel(r.canal,0); ledcDetach(pin);
#else
    ledcWrite(r.canal,0); ledcDetachPin(pin);
#endif
    attachedPin[r.canal]=-1;
  }
  motorFreno(r.pinFI,r.pinBI);
}
static void aplicarDuty(Rueda &r) {
  if(hardwareFault || !pasivo::PROFILE_VERIFIED || r.sentido==0 || r.dutyPedido==0 ||
     (int32_t)(millis()-r.frenohasta)<0) { detachMotor(r); return; }
  int active=r.sentido>0?r.pinFI:r.pinBI;
  int passive=r.sentido>0?r.pinBI:r.pinFI;
  if(attachedPin[r.canal]!=active) {
    detachMotor(r); digitalWrite(passive,LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    if(!ledcAttachChannel(active,20000,10,r.canal)) {hardwareFault=true;return;}
#else
    if(ledcSetup(r.canal,20000,10)==0) {hardwareFault=true;return;}
    ledcAttachPin(active,r.canal);
#endif
    attachedPin[r.canal]=active;
  }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(r.canal,(uint32_t)(r.dutyPedido*1023/100));
#else
  ledcWrite(r.canal,(uint32_t)(r.dutyPedido*1023/100));
#endif
}

void setRueda(Rueda &r, int duty, int sentido) {
  duty = constrain(duty, 0, PWM_MAX);
  if (r.invertida) sentido = -sentido;
  if (duty == 0) sentido = 0;

  unsigned long ahora = millis();

  if (sentido != r.sentido) {
    detachMotor(r);
    r.frenohasta = ahora + DEAD_TIME_MS;
    r.sentidoAnterior = r.sentido;
    r.sentido = sentido;
  }

  r.dutyPedido = duty;
  aplicarDuty(r);
}

// Passive-pivot command envelope (open-loop hypothesis).
void applyMotors(float vx,float omega) {
  if(CABLEADO_INVIERTE_AVANCE) vx=-vx;
  if(CABLEADO_INVIERTE_GIRO) omega=-omega;
  auto target=pasivo::mix(vx,omega);
  for(int i=0;i<2;i++) {
    setRueda(*LADO_IZQ[i],lroundf(fabsf(target.left)/V_MAX*100),target.left>=0?1:-1);
    setRueda(*LADO_DER[i],lroundf(fabsf(target.right)/V_MAX*100),target.right>=0?1:-1);
  }
}
void updateDrive() {
  static uint32_t previous=millis(); uint32_t now=millis();
  float dt=fminf((uint32_t)(now-previous)*.001f,.05f);previous=now;
  if(inFailsafe || hardwareFault || !pasivo::PROFILE_VERIFIED) return;
  auto target=pasivo::mix(lastV,lastOmega);
  vAplicada=pasivo::slew(vAplicada,target.v,.12f*dt);
  // Curvature envelope re-applied after ramp, including reversal through zero.
  auto limited=pasivo::mix(vAplicada,pasivo::slew(omegaAplicada,target.w,.3f*dt));
  omegaAplicada=limited.w;
  applyMotors(vAplicada,omegaAplicada);
  for(auto r:RUEDAS) aplicarDuty(*r);
}

void stopMotors() {
  vAplicada     = 0.0f;
  omegaAplicada = 0.0f;
  for (int i = 0; i < 4; i++) setRueda(*RUEDAS[i], 0, 0);
}

void enterFailsafe(const char *motivo) {
  inFailsafe = true;
  lastV     = 0;
  lastOmega = 0;
  stopMotors();
  scanning = false;
  scanRequested=false;
  setLed(FOCO_APAGADO_FW);
  lastLed = FOCO_APAGADO_FW;
  Serial.print("FAILSAFE: ");
  Serial.println(motivo);
}

// ===== PROTOCOL: same 8-byte layout, new neutral rearm semantics =====
bool aplicarComando(const CmdFrame &c, uint8_t via) {
  unsigned long ahora = millis();

  if(c.v_pct < -100 || c.v_pct > 100 || c.omega_pct < -100 || c.omega_pct > 100 ||
     c.servo_angle<SERVO_MIN_ANGLE || c.servo_angle>SERVO_MAX_ANGLE || (c.flags & ~3)) return false;
  if(!pasivo::PROFILE_VERIFIED || hardwareFault) return false;
  // Timeout is checked before processing queued packets, too.
  if(!inFailsafe && (uint32_t)(ahora-lastCmdMillis)>CMD_TIMEOUT_MS) enterFailsafe("timeout");
  if(sequenceSeen && !pasivo::newer(c.seq,lastSeq)) {tramasViejas++;return false;}
  // After boot or loss: first NEW command must be neutral. Same sequence space BLE/UDP.
  if(inFailsafe && (c.v_pct!=0 || c.omega_pct!=0)) return false;
  // BLE and UDP share the Jetson sequence counter; accept newer frames from either.
  sequenceSeen=true;
  lastCmdMillis = ahora;
  lastSeq   = c.seq;
  viaUltima = via;
  if (via == VIA_WIFI) tramasWifi++; else tramasBle++;

  lastV     = (constrain((int)c.v_pct,     -100, 100) / 100.0f) * V_MAX;
  lastOmega = (constrain((int)c.omega_pct, -100, 100) / 100.0f) * OMEGA_MAX;
  lastServo = c.servo_angle;
  lastLed   = (c.flags & FLAG_LED) != 0;

  inFailsafe = false;

  // Bounded open-loop motion request
  // A neutral command is an immediate de-energizing stop, including legacy emergency_stop.
  if(lastV==0 && lastOmega==0) stopMotors();
  // Other motion is serviced by updateDrive(), not by packet frequency.

  if ((c.flags & FLAG_SCAN) && !scanRequested) {
    scanning = true;
    scanStartMillis = millis();
  }
  scanRequested=(c.flags & FLAG_SCAN)!=0; // backend sends a one-shot pulse
  setLed(lastLed);
  if (!scanning) setServo(lastServo);
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
  }
}
#endif

void enviarEstado() {
  bool porBle  = bleConnected && pStatusChar != nullptr;
#if ENLACE_WIFI
  bool porWifi = wifiListo && udpAbierto;
#else
  bool porWifi = false;
#endif

  StatusFrame st;
  st.magic     = STATUS_MAGIC;
  st.v_pct     = (int8_t)constrain((int)lroundf(vAplicada / V_MAX     * 100.0f), -100, 100);
  st.omega_pct = (int8_t)constrain((int)lroundf(omegaAplicada / OMEGA_MAX * 100.0f), -100, 100);
  st.servo_angle = (uint8_t)currentServoAngle;
  st.flags = 0;
  if (ledOn)      st.flags |= ST_LED;
  if (scanning)   st.flags |= ST_SCANNING;
  if (inFailsafe) st.flags |= ST_FAILSAFE;
  if (motivoReinicio == ESP_RST_BROWNOUT)  st.flags |= ST_BROWNOUT;
  if (millis() < ARRANQUE_RECIENTE_MS)     st.flags |= ST_RECIEN;
  if (viaUltima == VIA_WIFI)               st.flags |= ST_VIA_WIFI;
  if (porWifi)                             st.flags |= ST_WIFI_OK;
  if (bleConnected)                        st.flags |= ST_BLE_OK;
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

#define TEST_MOTORES 0

#if TEST_MOTORES
void testArranque() {
  Serial.println("[TEST] R1 adelante...");
  digitalWrite(R1_FI, HIGH); delay(300); digitalWrite(R1_FI, LOW); delay(100);

  Serial.println("[TEST] R2 adelante...");
  digitalWrite(R2_FI, HIGH); delay(300); digitalWrite(R2_FI, LOW); delay(100);

  Serial.println("[TEST] L1 adelante...");
  digitalWrite(L1_FI, HIGH); delay(300); digitalWrite(L1_FI, LOW); delay(100);

  Serial.println("[TEST] L2 adelante...");
  digitalWrite(L2_FI, HIGH); delay(300); digitalWrite(L2_FI, LOW); delay(100);

  Serial.println("[TEST] Todas adelante 500ms...");
  digitalWrite(R1_FI, HIGH);
  digitalWrite(R2_FI, HIGH);
  digitalWrite(L1_FI, HIGH);
  digitalWrite(L2_FI, HIGH);
  delay(500);
  digitalWrite(R1_FI, LOW);
  digitalWrite(R2_FI, LOW);
  digitalWrite(L1_FI, LOW);
  digitalWrite(L2_FI, LOW);
  Serial.println("[TEST] OK");
}
#endif

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
  Serial.println("=== AGROBOT C3 v4 - PASSIVE PIVOT / PROFILE LOCK ===");

#if TEST_MOTORES
  testArranque();
#endif

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  servoListo = ledcAttachChannel(SERVO_PIN, SERVO_LEDC_HZ, SERVO_LEDC_BITS, SERVO_LEDC_CANAL);
#else
  servoListo = (ledcSetup(SERVO_LEDC_CANAL, SERVO_LEDC_HZ, SERVO_LEDC_BITS) != 0);
  if (servoListo) ledcAttachPin(SERVO_PIN, SERVO_LEDC_CANAL);
#endif
  if(!servoListo) hardwareFault=true;
  servoEscribir(SERVO_PARK_ANGLE);
  currentServoAngle = SERVO_PARK_ANGLE;

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

#if ENLACE_WIFI
  WiFi.setHostname("agrobot-c3");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setTxPower(WIFI_POTENCIAS[0]);
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
  if(!inFailsafe && millis()-lastCmdMillis>CMD_TIMEOUT_MS) enterFailsafe("timeout");
  procesarBLE();
#if ENLACE_WIFI
  procesarWifi();
  gestionarWifi();
#endif

  static unsigned long ultimoRearme = 0;
  if (!bleConnected && (reabrirAnuncio || millis() - ultimoRearme > 5000)) {
    reabrirAnuncio = false;
    ultimoRearme   = millis();
    delay(20);
    pServer->startAdvertising();
  }
  bleWasConnected = bleConnected;

  if (!inFailsafe && millis() - lastCmdMillis > CMD_TIMEOUT_MS) {
    enterFailsafe("sin comando");
  }

  // Actualización de sistemas
  updateDrive();
  if(hardwareFault) {stopMotors();inFailsafe=true;}
  if(scanning) {
    // Timed triangular camera sweep, not a steering actuator.
    uint32_t elapsed=millis()-scanStartMillis;
    if(elapsed>=SCAN_SWEEP_MS) {scanning=false;setServo(SERVO_PARK_ANGLE);}
    else {float t=elapsed/(float)SCAN_SWEEP_MS;float triangle=t<.5f?2*t:2*(1-t);
      setServo(lroundf(74+32*triangle));}
  }
  updateServo();

  // Log cada 500ms
  static unsigned long ultimaTraza = 0;
  if (millis() - ultimaTraza >= 500) {
    ultimaTraza = millis();
    Serial.printf("[pasivo-v4] v=%.2f w=%.2f fs=%d gain=%.2f/%.2f | "
                  "L:%+d/%3d D:%+d/%3d\n",
                  lastV, lastOmega, (int)inFailsafe,
                  1.0f, 1.0f,
                  ruedaL1.sentido, ruedaL1.dutyPedido,
                  ruedaR1.sentido, ruedaR1.dutyPedido);
  }

  if (millis() - ultimoEstadoBle >= ESTADO_PERIODO_BLE_MS) {
    ultimoEstadoBle = millis();
    enviarEstado();
  }
}