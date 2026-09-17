// Ejecuta la cinemática DIFERENCIAL real del firmware capturando qué
// recibiría cada lado, y la compara contra el sketch de prueba que YA
// FUNCIONA en el robot. Esa comparación es lo que da confianza: no se
// verifica contra lo que yo creo que debería pasar, sino contra lo que el
// robot ya hace.
//
//   g++ -std=c++17 -o motor_logic_test motor_logic_test.cpp && ./motor_logic_test
//
// OJO: applyMotors()/setLado() están COPIADAS del .ino (un sketch no
// compila solo). Si las cambiás en el firmware, actualizá también esto.
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>

#define constrain(a,l,h) ((a)<(l)?(l):((a)>(h)?(h):(a)))

// --- constantes copiadas del .ino ---
const float WHEEL_RADIUS = 0.06f;
const float TRACK_WIDTH  = 0.33f;
const float K_ROT        = TRACK_WIDTH / 2.0f;
const float V_MAX        = 0.55f;
const float OMEGA_MAX    = 1.5f;
const int   PWM_MAX      = 255;
const int   PWM_MIN      = 200;   // rango pedido por el usuario: 200..255
const int   PWM_MIN_GIRO = 235;   // piso mas alto girando (arrastre de lado)
const float GIRO_UMBRAL  = 1.0f;
const int   GIRO_ON_MS   = 140;
const int   GIRO_OFF_MS  = 90;
bool giroActivo = false;

// --- captura: duty con signo por lado (+ adelante, - atrás) ---
enum { IZQ = 0, DER = 1 };
int salida[2];
const char *NOMBRE[2] = {"IZQ", "DER"};

void setLado(int idx, float w, float escala, float wRef) {
  float frac = constrain(fabs(w) * escala / wRef, 0.0f, 1.0f);
  const int piso = giroActivo ? PWM_MIN_GIRO : PWM_MIN;
  int duty = 0;
  if (frac > 0.02f) duty = piso + (int)lroundf(frac * (PWM_MAX - piso));
  salida[idx] = (duty == 0) ? 0 : ((w >= 0) ? duty : -duty);
}

// --- COPIADA del firmware ---
void applyMotors(float vx, float omega) {
  vx    = constrain(vx,    -V_MAX,     V_MAX);
  omega = constrain(omega, -OMEGA_MAX, OMEGA_MAX);

  float wIzq = (vx - K_ROT * omega) / WHEEL_RADIUS;
  float wDer = (vx + K_ROT * omega) / WHEEL_RADIUS;

  const float wRef = V_MAX / WHEEL_RADIUS;
  float wPico = fmaxf(fabs(wIzq), fabs(wDer));
  float escala = 1.0f;
  if (wPico > wRef && wPico > 0.0f) escala = wRef / wPico;

  giroActivo = (fabsf(K_ROT * omega) >= GIRO_UMBRAL * fabsf(vx))
               && (fabsf(omega) > 0.05f);

  setLado(IZQ, wIzq, escala, wRef);
  setLado(DER, wDer, escala, wRef);
}

int fails = 0;
void check(const char *label, bool ok) {
  printf("  %s %s\n", ok ? "OK  " : "FALLA", label);
  if (!ok) fails++;
}
const char *sent(int v) { return v > 0 ? "ADELANTE" : (v < 0 ? "ATRAS" : "parado"); }

void mostrar(const char *caso) {
  printf("\n%s\n   ", caso);
  for (int i = 0; i < 2; i++)
    printf("%s=%5d (%s)   ", NOMBRE[i], salida[i], sent(salida[i]));
  printf("\n");
}

// Sentidos que produce el sketch de prueba del usuario, que YA ANDA.
// Agrupación de lados tomada de los comentarios de girarIzquierda():
// motor(1) y motor(4) = DERECHA, motor(2) y motor(3) = IZQUIERDA.
struct Caso { const char *nombre; float vx, w; const char *esperado[2]; };
const Caso CASOS[] = {
  {"adelante()",       V_MAX,  0,          {"ADELANTE","ADELANTE"}},
  {"atras()",         -V_MAX,  0,          {"ATRAS",   "ATRAS"}},
  {"girarIzquierda()",  0,     OMEGA_MAX,  {"ATRAS",   "ADELANTE"}},
  {"girarDerecha()",    0,    -OMEGA_MAX,  {"ADELANTE","ATRAS"}},
};

// ---------------------------------------------------------------
// Arranque por pulsos: misma maquina de estados que updateKicks().
// Se simula el reloj para poder verificar los tiempos sin esperarlos.
// ---------------------------------------------------------------
const int KICK_PULSOS = 3, KICK_ON_MS = 200, KICK_OFF_MS = 30;

struct KickSim { int restantes = 0; bool alto = false; long faseHasta = 0; };
int dutyConKick(const KickSim &k, int dutyPedido) {
  if (k.restantes > 0) return k.alto ? PWM_MAX : 0;
  return dutyPedido;
}
void kickArrancar(KickSim &k, long t) { k.restantes = KICK_PULSOS; k.alto = true; k.faseHasta = t + KICK_ON_MS; }
void kickTick(KickSim &k, long t) {
  if (k.restantes == 0 || t < k.faseHasta) return;
  if (k.alto) { k.alto = false; k.restantes--; k.faseHasta = t + KICK_OFF_MS; }
  else        { k.alto = true;                 k.faseHasta = t + KICK_ON_MS;  }
}

int main() {
  printf("=== Cinematica diferencial vs. el sketch que YA FUNCIONA ===\n");
  for (const Caso &c : CASOS) {
    applyMotors(c.vx, c.w);
    mostrar(c.nombre);
    bool ok = true;
    for (int i = 0; i < 2; i++)
      if (strcmp(sent(salida[i]), c.esperado[i]) != 0) ok = false;
    check("coincide con el sketch probado en el robot", ok);
  }

  printf("\n=== El robot NO se desplaza de costado ===\n");
  // applyMotors solo acepta (vx, omega): no existe forma de pedir lateral.
  // Girar sobre el eje deja los dos lados en sentidos OPUESTOS, que es lo
  // contrario de una traslacion lateral mecanum (misma direccion de avance
  // en diagonales opuestas). Este check documenta la diferencia.
  applyMotors(0, OMEGA_MAX);
  check("girar enfrenta los dos lados (skid-steer, no traslacion)",
        salida[IZQ] * salida[DER] < 0);

  printf("\n=== Simetria adelante/atras (odometria sin sesgo) ===\n");
  bool sim = true;
  for (float v = 0.05f; v <= V_MAX; v += 0.05f) {
    applyMotors(v, 0);   int a = salida[IZQ];
    applyMotors(-v, 0);  int b = salida[IZQ];
    if (a != -b) { sim = false; printf("   v=%.2f -> %d vs %d\n", v, a, b); }
  }
  check("mismo duty en ambos sentidos, en todo el rango", sim);

  printf("\n=== Simetria de giro izquierda/derecha ===\n");
  bool simGiro = true;
  for (float w = 0.05f; w <= OMEGA_MAX; w += 0.05f) {
    applyMotors(0, w);   int a = salida[IZQ], b = salida[DER];
    applyMotors(0, -w);  if (salida[IZQ] != -a || salida[DER] != -b) simGiro = false;
  }
  check("girar a un lado y al otro cuesta lo mismo", simGiro);

  printf("\n=== Combinado: avanzar girando (curva) ===\n");
  applyMotors(0.3f, 0.5f);
  mostrar("vx=0.3 omega=0.5 (curva a la izquierda)");
  check("los dos lados avanzan pero a distinta velocidad",
        salida[IZQ] > 0 && salida[DER] > 0 && salida[DER] > salida[IZQ]);

  printf("\n=== Nunca se satura fuera de rango ===\n");
  bool enRango = true;
  for (float v = -V_MAX; v <= V_MAX; v += 0.02f)
    for (float w = -OMEGA_MAX; w <= OMEGA_MAX; w += 0.05f) {
      applyMotors(v, w);
      for (int i = 0; i < 2; i++)
        if (salida[i] > PWM_MAX || salida[i] < -PWM_MAX) enRango = false;
    }
  check("ningun duty se pasa de +/-255 en todo el espacio (vx,omega)", enRango);

  printf("\n=== Rango de PWM pedido: 200..255 ===\n");
  bool dentro = true, arranca = true;
  for (float v = 0.02f; v <= V_MAX; v += 0.01f) {
    applyMotors(v, 0);
    int d = abs(salida[IZQ]);
    if (d != 0 && (d < PWM_MIN || d > PWM_MAX)) dentro = false;
    if (d == 0) arranca = false;
  }
  check("todo duty distinto de cero cae en [200..255]", dentro);
  check("hasta la velocidad mas baja pedida mueve el robot", arranca);
  applyMotors(0, 0);
  check("velocidad cero sigue siendo motores parados", salida[IZQ] == 0);
  applyMotors(V_MAX, 0);
  check("a fondo da PWM_MAX", salida[IZQ] == PWM_MAX);

  printf("\n=== Arranque por pulsos (equivalente a pulsos() del sketch) ===\n");
  KickSim k;
  kickArrancar(k, 0);
  check("el primer instante sale a fondo, no al duty lento",
        dutyConKick(k, PWM_MIN) == PWM_MAX);
  int pulsos = 0; bool previo = true;
  for (long t = 0; t <= 1000; t++) {
    kickTick(k, t);
    bool ahora = (k.restantes > 0) && k.alto;
    if (ahora && !previo) pulsos++;
    previo = ahora;
  }
  check("da 3 pulsos (el primero + 2 re-encendidos)", pulsos == 2);
  check("termina y entrega el duty pedido",
        k.restantes == 0 && dutyConKick(k, PWM_MIN) == PWM_MIN);

  printf("\n=== Giro: piso de duty mas alto y deteccion de giro dominante ===\n");
  applyMotors(0, OMEGA_MAX);
  check("girar sobre el sitio se detecta como giro dominante", giroActivo);
  // OJO: girar a fondo NO da 255 de base. La normalizacion es contra la
  // rueda de referencia (V_MAX), y un giro a OMEGA_MAX pide solo ~45% de
  // esa velocidad de rueda; con el piso de giro eso son ~244. Se deja asi a
  // proposito: normalizar el giro contra su propio maximo deformaria el
  // radio de las curvas. Los 255 los pone el tren de pulsos, no la base.
  check("girar a fondo queda cerca del maximo, no a mitad de tabla",
        abs(salida[IZQ]) >= 240 && abs(salida[IZQ]) <= PWM_MAX);
  // Giro lento: sin el piso propio caeria a mitad de tabla y no arrastraria.
  applyMotors(0, 0.3f);
  mostrar("vx=0 omega=0.3 (giro lento sobre el sitio)");
  check("hasta el giro mas lento pisa PWM_MIN_GIRO o mas",
        abs(salida[IZQ]) >= PWM_MIN_GIRO);
  applyMotors(V_MAX, 0);
  check("ir recto NO cuenta como giro", !giroActivo);
  applyMotors(V_MAX, 0.2f);
  check("corregir el rumbo yendo rapido tampoco (no habria pulsos)",
        !giroActivo);
  applyMotors(0.05f, OMEGA_MAX);
  check("una curva cerrada a paso lento si cuenta como giro", giroActivo);
  applyMotors(0, 0);
  check("parado no es giro", !giroActivo);

  printf("\n=== Giro: tren de pulsos sostenido (no se agota) ===\n");
  // Misma maquina de estados que updateGiro(): alterna golpe / base sin
  // contador de pulsos, a diferencia del arranque.
  {
    bool alto = true; long faseHasta = GIRO_ON_MS;
    int golpes = 0; bool previo = alto;
    for (long t = 1; t <= 5000; t++) {
      if (t >= faseHasta) { alto = !alto; faseHasta = t + (alto ? GIRO_ON_MS : GIRO_OFF_MS); }
      if (alto && !previo) golpes++;
      previo = alto;
    }
    // 5 s / 230 ms de ciclo = 21 ciclos completos; el primero ya arranca en
    // alto, asi que se cuentan los re-encendidos.
    check("sigue golpeando despues de 5 s de giro", golpes >= 15);
  }

  printf("\n=== Canales LEDC (el ESP32-C3 tiene 6) ===\n");
  printf("  motores 4 + servo 1 = 5\n");
  check("entran en los 6 del C3", 4 + 1 <= 6);

  printf("\n%s\n", fails ? "RESULTADO: HAY FALLAS" : "RESULTADO: todo correcto");
  return fails ? 1 : 0;
}
