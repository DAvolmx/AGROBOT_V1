"""Controlador CONTINUO de pasillo para el firmware «suave» (PWM de hardware con rampas).

Sustituye a los pulsos del explorador cuando el firmware ya permite conducir suave: cada 50 ms recibe
las distancias libres del LIDAR (adelante / izquierda / derecha, en metros desde la carrocería, None =
nada a la vista) y la velocidad de giro del giroscopio, y devuelve (v m/s, omega rad/s).

  · v: velocidad de crucero; baja cuando hay algo delante; 0 si está muy cerca (entonces gira en el sitio
    hacia el lado más libre, con histéresis para no oscilar).
  · omega: centra el robot entre dos paredes/macetas (error = (izq - der)/2); con una sola pared mantiene la
    distancia nominal; sin paredes solo amortigua el giro. Siempre resta el giro medido (amortiguación),
    y está limitada y suavizada.

Puro (sin hardware ni asyncio): se prueba en el simulador con el mismo código que correría en el robot.
NO está enganchado a ningún modo del robot todavía: ver LEEME_CONTROL_PASILLO.md.
"""
from __future__ import annotations

import math


def _libre(d):
    return math.inf if d is None else d


class PasilloController:
    def __init__(self, v_cruise=0.30, kp=1.6, kd=0.6, w_max=0.7, nominal=0.35, d_stop=0.30,
                 d_slow=0.80, wall_max=1.2, giro_sitio=0.5, suavizado=0.5, kh=1.2, kl=2.0,
                 pasillo_max=1.0, seguir_max=0.6):
        self.v_cruise, self.kp, self.kd, self.w_max = v_cruise, kp, kd, w_max
        self.nominal, self.d_stop, self.d_slow, self.wall_max = nominal, d_stop, d_slow, wall_max
        self.giro_sitio, self.alpha = giro_sitio, suavizado
        self.kh, self.kl = kh, kl
        self.pasillo_max, self.seguir_max = pasillo_max, seguir_max
        self._w = 0.0
        self._girando = 0          # -1 derecha, +1 izquierda, 0 no gira en el sitio
        self.yaw = 0.0             # rumbo integrado del giroscopio (rad, relativo al arranque)
        self.ref = 0.0             # rumbo de referencia: el error lateral lo va moviendo (curvas)

    def reset(self):
        self._w = 0.0
        self._girando = 0
        self.yaw = self.ref = 0.0

    def step(self, d: dict, yaw_rate: float, dt: float = 0.05):
        self.yaw += yaw_rate * dt
        front = _libre(d.get("adelante"))
        izq, der = _libre(d.get("izquierda")), _libre(d.get("derecha"))
        # --- bloqueado delante: girar en el sitio hacia el lado más libre (con histéresis) ---
        if front <= self.d_stop or (self._girando and front <= self.d_stop + 0.15):
            if not self._girando:
                self._girando = 1 if izq >= der else -1
            self._w = self.alpha * self._w + (1 - self.alpha) * self._girando * self.giro_sitio
            self.ref = self.yaw                              # al terminar de girar, el nuevo rumbo es la referencia
            return 0.0, self._w
        self._girando = 0
        # --- velocidad ---
        v = self.v_cruise if front >= self.d_slow else self.v_cruise * 0.6
        # --- dirección ---
        # centrar solo con DOS paredes cercanas (pasillo); con una sola, seguirla solo si está cerca;
        # lejos (sala grande) no se persigue nada: se mantiene el rumbo.
        if max(izq, der) <= self.pasillo_max:
            e = (izq - der) / 2.0                           # >0: más sitio a la izquierda -> girar a la izquierda
        elif izq <= self.seguir_max and izq < der:
            e = izq - self.nominal                          # muy cerca de la izquierda -> girar a la derecha
        elif der <= self.seguir_max and der < izq:
            e = self.nominal - der
        else:
            e = 0.0
        # cascada: el error lateral mueve el rumbo de referencia (así sigue curvas) y un lazo de rumbo
        # con el giroscopio lo mantiene recto; amortiguación con la velocidad de giro.
        self.ref += self.kl * e * dt
        self.ref = max(self.yaw - 0.6, min(self.yaw + 0.6, self.ref))
        w = self.kh * (self.ref - self.yaw) + self.kp * e - self.kd * yaw_rate
        w = max(-self.w_max, min(self.w_max, w))
        self._w = self.alpha * self._w + (1 - self.alpha) * w
        return v, self._w
