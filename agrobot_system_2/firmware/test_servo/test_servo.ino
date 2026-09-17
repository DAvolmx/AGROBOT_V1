/*
 * AGROBOT — PRUEBA AISLADA DEL SERVO (ESP32-C3) — v3
 *
 * QUE CAMBIA RESPECTO DE LA v2
 * ----------------------------
 * 1. TODO el diagnostico se imprime en cada vuelta del loop, no solo en
 *    setup(). En la v2 el bloque importante salia una vez, antes de que diera
 *    tiempo a abrir el monitor serie, y lo unico que se veia era el resumen
 *    inutil de "attach fallo".
 *
 * 2. Prueba VARIOS PINES, no solo el GPIO10. En la v2 ninguna resolucion
 *    engancho en GPIO10 — ni siquiera 8 bits, que el chip acepta de sobra.
 *    Cuando falla hasta la configuracion mas modesta, el sospechoso deja de
 *    ser la resolucion y pasa a ser el pin o el estado del periferico LEDC.
 *    Si otro pin SI engancha, el problema es del GPIO10 concreto; si no
 *    engancha ninguno, es del chip o de como esta compilado.
 *
 * PINES QUE PRUEBA, Y POR QUE ESOS
 *   GPIO10  el del servo.
 *   GPIO9   libre en el robot (es el boton BOOT, no va a ningun actuador).
 *   GPIO8   el foco. Vas a ver la lampara parpadear: es esperado.
 *   GPIO21  TX del enlace con la Jetson. Solo se toca en esta prueba aislada.
 *
 * ⚠️ NO se prueban los GPIO 0..7: son las ocho entradas de los drivers de
 * motor. Mandarles PWM movería las ruedas con el robot sobre la mesa.
 */

#include <ESP32Servo.h>

const int PINES[]   = {10, 9, 8, 21};
const int N_PINES   = sizeof(PINES) / sizeof(PINES[0]);
const int SERVO_PIN = 10;

const int PULSO_MIN_US = 500;
const int PULSO_MAX_US = 2400;
const int LEDC_HZ      = 50;
const int LEDC_CANAL   = 4;      // el C3 tiene 6 canales (0..5)

Servo servo;
int canalServo = -1;
int pinBueno = -1, bitsBuenos = 0;

uint32_t usADuty(int us, int bits) {
  return (uint32_t)((float)us * ((float)(1UL << bits) / 20000.0f));
}

bool engancha(int pin, int bits) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  bool ok = ledcAttachChannel(pin, LEDC_HZ, bits, LEDC_CANAL);
  if (ok) ledcDetach(pin);
  return ok;
#else
  bool ok = (ledcSetup(LEDC_CANAL, LEDC_HZ, bits) != 0);
  if (ok) { ledcAttachPin(pin, LEDC_CANAL); ledcDetachPin(pin); }
  return ok;
#endif
}

void diagnostico() {
  Serial.println("\n==================================================");
  Serial.printf ("  DIAGNOSTICO — compilado %s %s\n", __DATE__, __TIME__);
#ifdef ARDUINO_BOARD
  Serial.printf ("  placa: %s\n", ARDUINO_BOARD);
#else
  Serial.println("  placa: (ARDUINO_BOARD sin definir)");
#endif
#ifdef ARDUINO_ESP32C3_DEV
  Serial.println("  ARDUINO_ESP32C3_DEV: SI");
#else
  Serial.println("  ARDUINO_ESP32C3_DEV: NO  <-- deberia decir SI");
  Serial.println("     Herramientas > Placa > ESP32C3 Dev Module");
#endif
#ifdef ESP_ARDUINO_VERSION_MAJOR
  Serial.printf ("  core ESP32 para Arduino: v%d\n", ESP_ARDUINO_VERSION_MAJOR);
#endif
  Serial.println("==================================================");

  Serial.println("  pin   8b  10b  12b  14b  16b");
  pinBueno = -1; bitsBuenos = 0;
  for (int i = 0; i < N_PINES; i++) {
    int p = PINES[i];
    Serial.printf("  %3d ", p);
    for (int b = 8; b <= 16; b += 2) {
      bool ok = engancha(p, b);
      Serial.printf("%5s", ok ? "si" : "NO");
      // Nos quedamos con la mayor resolucion valida del primer pin que responda.
      if (ok && b <= 14 && (pinBueno == -1 || pinBueno == p)) { pinBueno = p; bitsBuenos = b; }
      delay(20);
    }
    Serial.println();
  }

  if (pinBueno < 0) {
    Serial.println("\n  ⚠️  NINGUN pin engancha con NINGUNA resolucion.");
    Serial.println("      Eso no es el servo ni el cableado: es el periferico");
    Serial.println("      LEDC o como esta compilado el sketch. Mira la placa.");
  } else {
    Serial.printf("\n  mejor combinacion: GPIO%d a %d bits\n", pinBueno, bitsBuenos);
    if (pinBueno != SERVO_PIN)
      Serial.printf("  ⚠️  OJO: el GPIO%d del servo NO engancha, pero el %d SI.\n"
                    "      El problema es de ese pin en concreto.\n", SERVO_PIN, pinBueno);
  }

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo.setPeriodHertz(50);
  canalServo = servo.attach(SERVO_PIN, PULSO_MIN_US, PULSO_MAX_US);
  Serial.printf("  ESP32Servo attach(GPIO%d) -> %d %s\n\n",
                SERVO_PIN, canalServo, canalServo > 0 ? "(OK)" : "(FALLO)");
}

void barrido() {
  if (pinBueno < 0) { Serial.println("  sin combinacion valida, no se barre\n"); return; }
  Serial.printf("  --- barrido en GPIO%d a %d bits ---\n", pinBueno, bitsBuenos);
  if (canalServo > 0) servo.detach();

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!ledcAttachChannel(pinBueno, LEDC_HZ, bitsBuenos, LEDC_CANAL)) return;
#else
  ledcSetup(LEDC_CANAL, LEDC_HZ, bitsBuenos);
  ledcAttachPin(pinBueno, LEDC_CANAL);
#endif
  for (int a = 0; a <= 180; a += 45) {
    int us = map(a, 0, 180, PULSO_MIN_US, PULSO_MAX_US);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWriteChannel(LEDC_CANAL, usADuty(us, bitsBuenos));
#else
    ledcWrite(LEDC_CANAL, usADuty(us, bitsBuenos));
#endif
    Serial.printf("    %3d grados  pulso %4d us\n", a, us);
    delay(800);
  }
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach(pinBueno);
#else
  ledcDetachPin(pinBueno);
#endif
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);
  delay(400);
}

void loop() {
  diagnostico();   // se repite SIEMPRE: es lo que hay que leer
  barrido();
  Serial.println("  ===== se repite en 4 s =====");
  delay(4000);
}
