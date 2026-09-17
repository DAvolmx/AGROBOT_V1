/*
 * SERVO — PRUEBA MINIMA. Sin librerias, sin nada mas.
 *
 * POR QUE ESTE SKETCH
 * En las pruebas anteriores el servo NUNCA llego a recibir señal en su pin.
 * El orden era: se comprobaba que el GPIO10 aceptaba LEDC (y lo aceptaba),
 * despues ESP32Servo intentaba engancharlo, fallaba, y al fallar dejaba el
 * pin medio tomado. A partir de ahi ningun ledcAttachChannel volvia a
 * funcionar sobre ese GPIO, asi que el barrido se saltaba solo. El unico
 * barrido que llego a ejecutarse fue en GPIO9, donde no hay servo conectado.
 *
 * Aqui no hay libreria que pueda estropear el pin. Solo LEDC.
 *
 * PARAMETROS, y por que estos
 *   50 Hz    lo que espera un servo de hobby.
 *   14 bits  medido en esta placa: a 50 Hz el C3 acepta 10, 12 y 14. Con 8 el
 *            core responde "duty resolution 8 can not be achieved" porque el
 *            divisor se queda sin margen, y con 16 se pasa del maximo del
 *            chip (SOC_LEDC_TIMER_BIT_WIDTH = 14).
 *   canal 0  libre. Los motores del robot usan 2..5.
 *
 * CABLEADO — si esto no mueve el servo, el problema ya no es software
 *   señal -> GPIO10
 *   masa  -> GND del C3, comun con la fuente del servo. OBLIGATORIO.
 *   +     -> 5 V con corriente propia. NUNCA del pin de 3.3 V: un servo pide
 *            picos de 0.5 a 1 A y el regulador del C3 no los da. Por debajo
 *            de 4.8 V muchos servos ni se mueven.
 *
 * IDE: placa "ESP32C3 Dev Module", USB CDC On Boot = Enabled,
 *      Core Debug Level = Error (para ver el motivo si algo falla).
 */

#define SERVO_PIN  10
#define CANAL       0
#define BITS       14
#define HZ         50
#define PULSO_MIN 500     // us, 0 grados
#define PULSO_MAX 2400    // us, 180 grados

bool listo = false;

void escribir(int grados) {
  int us = map(grados, 0, 180, PULSO_MIN, PULSO_MAX);
  uint32_t duty = (uint32_t)((float)us * ((float)(1UL << BITS) / 20000.0f));
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(CANAL, duty);
#else
  ledcWrite(CANAL, duty);
#endif
  Serial.printf("  %3d grados -> pulso %4d us -> duty %lu de %lu\n",
                grados, us, (unsigned long)duty, (unsigned long)((1UL << BITS) - 1));
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);
  delay(400);

  Serial.println();
  Serial.printf("=== SERVO MINIMO — %s %s ===\n", __DATE__, __TIME__);
  Serial.printf("GPIO%d, canal %d, %d Hz, %d bits\n", SERVO_PIN, CANAL, HZ, BITS);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  listo = ledcAttachChannel(SERVO_PIN, HZ, BITS, CANAL);
#else
  listo = (ledcSetup(CANAL, HZ, BITS) != 0);
  if (listo) ledcAttachPin(SERVO_PIN, CANAL);
#endif

  if (listo) {
    Serial.println("enganche: OK. Si el servo no se mueve, el problema es"
                   " electrico: alimentacion, masa comun o el propio servo.");
  } else {
    Serial.println("enganche: FALLO. Con Core Debug Level en Error, el core"
                   " imprime el motivo justo encima de esta linea.");
  }
  Serial.println();
}

void loop() {
  if (!listo) { delay(2000); return; }
  // Movimientos amplios y lentos: un servo con poca corriente a veces no llega
  // a un extremo pero si tiembla, y eso tambien es informacion.
  Serial.println("--- barrido ---");
  for (int a = 0; a <= 180; a += 45) { escribir(a); delay(900); }
  for (int a = 180; a >= 0; a -= 45) { escribir(a); delay(900); }
  Serial.println();
  delay(1000);
}
