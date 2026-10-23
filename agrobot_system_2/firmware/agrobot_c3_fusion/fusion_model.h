#pragma once
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
  float comp_neg = 0;   // COMPENSACION DEL GIRO (omega<0 = izquierda fisica en este robot): % del ciclo en que las ruedas que van HACIA ATRAS se APAGAN
  float comp_pos = 0;   // idem para omega>0 (derecha). Medido 2026-10-23: a la izquierda retrocede ~9 cm por toque; a la derecha avanza ~2,5
};
constexpr int N_LIM = 13;
inline float clip(float x, float a, float b) { return x < a ? a : x > b ? b : x; }

inline bool limValido(const Lim &L) {
  return L.adel >= 10 && L.adel <= 100 && L.atras >= 10 && L.atras <= 100 &&
         L.pivote >= 30 && L.pivote <= 100 && L.minp >= 0 && L.minp <= 100 &&
         L.minp <= L.adel && L.minp <= L.atras && L.acel >= 20 && L.acel <= 600 &&
         L.frena >= 50 && L.frena <= 1000 && L.tras >= 100 && L.tras <= 200 &&
         L.dif_del >= 0 && L.dif_del <= 100 &&
         (L.periodo == 0 || (L.periodo >= 40 && L.periodo <= 400)) &&
         L.refuerzo >= 100 && L.refuerzo <= 130 && L.freno_ms >= 0 && L.freno_ms <= 400 &&
         L.comp_neg >= 0 && L.comp_neg <= 80 && L.comp_pos >= 0 && L.comp_pos <= 80;
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

// Compensacion del giro: en un giro (omega != 0) las ruedas que van hacia atras (salida < 0) solo se alimentan la fraccion
// (1 - comp/100) del ciclo; el resto de ruedas, siempre. esc[i] = fraccion de ciclo ENCENDIDA de la rueda i (1 = sin recorte).
inline void escala_giro(float omega, const Out &o, const Lim &L, float esc[4]) {
  float wn = clip(omega / OMEGA_MAX, -1, 1);
  float c = fabsf(wn) < .02f ? 0.f : (wn < 0 ? L.comp_neg : L.comp_pos);
  for (int i = 0; i < 4; i++) esc[i] = (c > 0 && o.d[i] < 0) ? 1.f - c / 100.f : 1.f;
}
inline float rebanada(float salida, uint32_t t_ms, float esc) {   // ciclo fijo de 100 ms
  if (esc >= 1.f || salida == 0) return salida;
  return (float)(t_ms % 100) < 100.f * esc ? salida : 0.f;
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
