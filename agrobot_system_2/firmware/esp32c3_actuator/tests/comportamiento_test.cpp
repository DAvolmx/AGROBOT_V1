// Test de COMPORTAMIENTO del firmware real (no una copia): incluye el .ino
// tal cual, con el Arduino simulado de stubs_arduino/ que registra el nivel
// y el PWM de cada pin y deja mover el reloj. Cubre lo agregado el
// 2026-10-01: freno activo, traseros de apoyo, rampa de aceleración y servo
// suave. Lo corre verificar.sh.
#include "../esp32c3_actuator.ino"

#include <cstdio>

static int fallos = 0;
static void check(const char *txt, bool ok) {
  std::printf("  %s %s\n", ok ? "OK  " : "FALLA", txt);
  if (!ok) fallos++;
}

// Duty efectivo de una rueda: 255 si un pin está fijo en HIGH (DC completo),
// el PWM si está en PWM, 0 si ambos en LOW. Freno = ambos HIGH.
static bool frenada(const Rueda &r) { return g_pwm[r.pinFI] < 0 && g_pwm[r.pinBI] < 0 &&
                                              g_nivel[r.pinFI] == HIGH && g_nivel[r.pinBI] == HIGH; }
static bool libre(const Rueda &r)   { return g_pwm[r.pinFI] < 0 && g_pwm[r.pinBI] < 0 &&
                                              g_nivel[r.pinFI] == LOW && g_nivel[r.pinBI] == LOW; }
static int duty(const Rueda &r) {
  int d = 0;
  for (uint8_t p : {r.pinFI, r.pinBI}) {
    if (g_pwm[p] >= 0) d = std::max(d, g_pwm[p]);
    else if (g_nivel[p] == HIGH && !frenada(r)) d = std::max(d, 255);
  }
  return d;
}
static void avanzar(unsigned long ms) {
  for (unsigned long t = 0; t < ms; t += 10) {
    g_millis += 10;
    updateRampa(); updateKicks(); updateGiro(); updateTermico(); updateServo();
  }
}
static void reiniciar() {
  inFailsafe = false;
  cargaTrasera = 0.0f;
  lastV = lastOmega = 0.0f;
  stopMotors();
  g_millis += 2000;              // rearma los pulsos de arranque
}

int main() {
  std::printf("=== Freno activo ===\n");
  reiniciar();
  applyMotors(0.4f, 0.0f); avanzar(1000);
  stopMotors();
  bool todas = true;
  for (Rueda *r : RUEDAS) todas = todas && frenada(*r);
  check("al parar, las cuatro ruedas quedan FRENADAS (FI=BI=HIGH)", todas);

  std::printf("=== Traseros de apoyo (recta) ===\n");
  reiniciar();
  applyMotors(0.4f, 0.0f);
  bool pulsoDel = false, pulsoTra = false;
  for (int i = 0; i < 40; i++) {          // primeros 400 ms
    avanzar(10);
    pulsoDel = pulsoDel || (duty(ruedaR1) == 255 && duty(ruedaL1) == 255);
    pulsoTra = pulsoTra || duty(ruedaR2) == 255 || duty(ruedaL2) == 255;
  }
  check("al arrancar, la delantera da pulsos a fondo", pulsoDel);
  check("la trasera NO da pulsos de arranque", !pulsoTra);
  avanzar(1500);
  int del = duty(ruedaR1), tra = duty(ruedaR2);
  std::printf("    delantera %d · trasera %d\n", del, tra);
  check("en recta la trasera va al ~60 % de la delantera",
        tra > 0 && std::abs(tra - (int)lroundf(del * FACTOR_TRASERO_RECTO)) <= 2);
  check("izquierda y derecha simétricas", duty(ruedaL1) == del && duty(ruedaL2) == tra);

  std::printf("=== Traseros de apoyo (giro) ===\n");
  reiniciar();
  applyMotors(0.0f, 1.2f); avanzar(1500);
  int gd = ruedaR1.dutyPedido, gt = ruedaR2.dutyPedido;
  std::printf("    pedido delantera %d · trasera %d\n", gd, gt);
  check("en giro la trasera aporta más (~85 %) pero con tope",
        gt > 0 && gt <= PWM_TOPE_TRASERO && gt >= (int)(gd * 0.80f));
  bool topeOk = true;
  for (int i = 0; i < 100; i++) { avanzar(10); topeOk = topeOk && duty(ruedaR2) <= PWM_TOPE_TRASERO; }
  check("los pulsos de giro de la trasera no pasan de PWM_TOPE_TRASERO", topeOk);

  std::printf("=== Protección térmica ===\n");
  reiniciar();
  cargaTrasera = TERMICO_CARGA_CORTE + 0.05f;
  applyMotors(0.4f, 0.0f); avanzar(1500);
  check("con carga térmica alta la trasera queda LIBRE (no frenada)", libre(ruedaR2) && libre(ruedaL2));
  check("y la delantera sigue tirando", duty(ruedaR1) > 0);
  cargaTrasera = 0.0f;
  reiniciar();
  for (int i = 0; i < 600; i++) { applyMotors(0.55f, 0.0f); avanzar(100); }
  std::printf("    carga trasera tras 60 s a fondo: %.2f\n", cargaTrasera);
  check("tras 60 s a fondo la carga estimada sube (protege)", cargaTrasera > 0.2f);

  std::printf("=== Rampa de aceleración ===\n");
  reiniciar();
  avanzar(20);                       // el loop ya venía corriendo
  lastV = V_MAX;                     // como si llegara una orden a fondo
  avanzar(100);
  float v100 = vAplicada;
  avanzar(500);
  std::printf("    v a 100 ms %.3f · a 600 ms %.3f (V_MAX %.2f)\n", v100, vAplicada, V_MAX);
  check("subir es progresivo (a 100 ms todavía no está a fondo)", v100 > 0.0f && v100 < 0.5f * V_MAX);
  check("llega a lo pedido en menos de 0,6 s", std::fabs(vAplicada - V_MAX) < 1e-3);
  lastV = 0.0f; avanzar(10);
  check("parar es INMEDIATO (sin rampa)", vAplicada == 0.0f);
  lastV = V_MAX; avanzar(600);
  lastV = -V_MAX; avanzar(10);
  check("invertir pasa primero por cero", vAplicada == 0.0f);
  lastV = 0.0f; avanzar(10);
  g_millis += 3000;                  // el loop se traba 3 s (BLE, WiFi...)
  lastV = V_MAX; avanzar(10);
  check("un loop trabado NO se convierte en un tirón a fondo", vAplicada < 0.3f * V_MAX);

  std::printf("=== Servo suave ===\n");
  setServo(SERVO_PARK_ANGLE); avanzar(2000);
  setServo(SERVO_MIN_ANGLE); avanzar(100);
  int a100 = currentServoAngle;
  avanzar(1500);
  std::printf("    a 100 ms %d° · al final %d°\n", a100, currentServoAngle);
  check("el servo NO salta: a 100 ms recorrió ~9°",
        a100 < SERVO_PARK_ANGLE && a100 > SERVO_PARK_ANGLE - 15);
  check("y llega al objetivo", currentServoAngle == SERVO_MIN_ANGLE);

  std::printf("\nRESULTADO: %s\n", fallos ? "HAY FALLOS" : "todo correcto");
  return fallos ? 1 : 0;
}
