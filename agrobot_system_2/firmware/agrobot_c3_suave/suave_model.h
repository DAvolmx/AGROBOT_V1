#pragma once
// Mezcla y rampas de las ruedas, SIN dependencias de Arduino (se prueba en el PC contra Python).
// Ruedas (indices): 0=R1 delantera, 1=R2 trasera, 2=L1 delantera, 3=L2 trasera.
// Convencion del firmware: lado "IZQ" = R1+R2, lado "DER" = L1+L2 (como el original).
//  - DELANTERAS (0 y 2): solo longitudinales, las dos IGUAL, nunca en sentido contrario.
//  - TRASERAS (1 y 3): hacen todo el giro por diferencial y llevan la potencia de pivote.
#include <math.h>
#include <stdint.h>
namespace suave {
constexpr float V_MAX = .55f, OMEGA_MAX = 1.5f;

struct Lim {            // todos en % de duty (o %/s para las rampas)
  float adel = 80;      // tope hacia adelante
  float atras = 60;     // tope hacia atras
  float pivote = 100;   // potencia diferencial de las traseras al girar
  float minp = 50;      // arranque minimo: por debajo de esto el motor solo zumba
  float acel = 120;     // rampa de subida (%/s)
  float frena = 300;    // rampa de bajada (%/s)
  float tras = 130;     // fuerza EXTRA de las traseras: multiplica su potencia longitudinal (tope 100 %)
};
constexpr int N_LIM = 7;
inline float clip(float x, float a, float b) { return x < a ? a : x > b ? b : x; }

// Rangos duros (el firmware rechaza todo lo de fuera, igual que el backend).
inline bool limValido(const Lim &L) {
  return L.adel >= 10 && L.adel <= 100 && L.atras >= 10 && L.atras <= 100 &&
         L.pivote >= 30 && L.pivote <= 100 && L.minp >= 0 && L.minp <= 90 &&
         L.minp <= L.adel && L.minp <= L.atras && L.acel >= 20 && L.acel <= 600 &&
         L.frena >= 50 && L.frena <= 1000 && L.tras >= 100 && L.tras <= 200;
}

struct Out { float d[4]; };   // duty firmado (%), ANTES de aplicar la inversion de cada rueda

inline Out mezclar(float v, float omega, const Lim &L) {
  Out o = {{0, 0, 0, 0}};
  if (!isfinite(v) || !isfinite(omega)) return o;
  float fv = clip(v / V_MAX, -1, 1), wn = clip(omega / OMEGA_MAX, -1, 1);
  if (fabsf(fv) < .02f) fv = 0;
  if (fabsf(wn) < .02f) wn = 0;
  if (fv == 0 && wn == 0) return o;
  float base = fv > 0 ? fminf(fv * 100, L.adel) : -fminf(-fv * 100, L.atras);   // delanteras
  float baseR = clip(base * L.tras / 100.f, -100.f, 100.f);                      // traseras: mas fuerza
  float dif = wn * L.pivote;
  float izq = baseR - dif, der = baseR + dif;        // omega>0 = giro a la izquierda
  float m = fmaxf(fabsf(izq), fabsf(der));
  if (m > 100) { izq *= 100 / m; der *= 100 / m; }
  o.d[0] = base;  o.d[2] = base;                      // delanteras: solo longitudinal
  o.d[1] = izq;   o.d[3] = der;                       // traseras: giro
  for (int i = 0; i < 4; i++) {                       // arranque minimo
    float a = fabsf(o.d[i]);
    if (a > 0 && a < L.minp) o.d[i] = (o.d[i] > 0 ? 1 : -1) * L.minp;
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
}  // namespace suave
