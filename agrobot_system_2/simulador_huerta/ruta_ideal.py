"""
Ruta ideal por el centro del sendero, y cómo seguirla.
=====================================================

Dos piezas:

  1. `trazar()`: convierte la polilínea del sendero en una ruta con puntos
     cada pocos centímetros, redondeando las esquinas. Es "por dónde habría
     que pasar", la referencia contra la que se mide todo lo demás.

  2. `Piloto`: persecución pura (pure pursuit), el método clásico para seguir
     una línea: se mira un punto de la ruta a cierta distancia por delante y
     se gira hacia él. Adaptado a ESTE chasis, que no sabe girar poco: en vez
     de una curvatura continua devuelve la misma orden que usa el resto del
     sistema (adelante / izquierda / derecha), en pulsos.

Sirve para tres cosas que pidió el usuario:
  · el "recorrido ideal" con el que comparar al modo autónomo,
  · el "filtro de conducción ideal", que corrige al robot para que no se
    salga de la ruta,
  · y generar ejemplos de conducción buena para entrenar.
"""
import math

import numpy as np

PASO_M = 0.10                 # separación entre puntos de la ruta
MIRADA_M = 0.45               # a qué distancia se mira por delante
TOLERANCIA_RUMBO_DEG = 12.0   # menos que esto se considera "de frente"
LLEGADA_M = 0.25              # se da por alcanzado un punto a esta distancia


def trazar(sendero, paso=PASO_M, cerrar=None) -> np.ndarray:
    """Polilínea del sendero -> ruta densa (N,2) en metros."""
    pts = np.asarray(sendero, np.float64).reshape(-1, 2)
    if len(pts) < 2:
        return pts
    if cerrar is None:                      # si el sendero vuelve al inicio, es un bucle
        cerrar = bool(math.dist(pts[0], pts[-1]) < 1.0 and len(pts) > 2)
    if cerrar and math.dist(pts[0], pts[-1]) > 1e-6:
        pts = np.vstack([pts, pts[0]])

    ruta = []
    for a, b in zip(pts[:-1], pts[1:]):
        largo = math.dist(a, b)
        n = max(1, int(largo / paso))
        for i in range(n):
            f = i / n
            ruta.append(a + (b - a) * f)
    ruta.append(pts[-1])
    return np.array(ruta, np.float64)


def progreso(ruta, punto) -> int:
    """Índice del punto de la ruta más cercano a `punto`."""
    if not len(ruta):
        return 0
    return int(np.argmin(np.hypot(ruta[:, 0] - punto[0], ruta[:, 1] - punto[1])))


def desvio(ruta, punto) -> float:
    """Distancia del robot a la ruta, en metros."""
    if not len(ruta):
        return 0.0
    return float(np.hypot(ruta[:, 0] - punto[0], ruta[:, 1] - punto[1]).min())


class Piloto:
    """Sigue la ruta con persecución pura, en pulsos como el resto del sistema."""

    def __init__(self, ruta, mirada=MIRADA_M, cerrado=False):
        self.ruta = np.asarray(ruta, np.float64).reshape(-1, 2)
        self.mirada = mirada
        self.cerrado = cerrado
        self.i = 0                      # hasta dónde se ha avanzado
        self.vueltas = 0
        self.terminado = False

    def objetivo(self, x, y):
        """Punto de la ruta a `mirada` metros por delante del robot."""
        if not len(self.ruta):
            return None
        # Solo se mira hacia adelante en la ruta: si no, al volver cerca del
        # inicio el robot se engancharía al tramo de atrás.
        self.i = max(self.i, progreso(self.ruta, (x, y)))
        j = self.i
        while j < len(self.ruta) - 1 and math.dist(self.ruta[j], (x, y)) < self.mirada:
            j += 1
        if j >= len(self.ruta) - 1 and math.dist(self.ruta[-1], (x, y)) < LLEGADA_M:
            if self.cerrado:
                self.i = 0
                self.vueltas += 1
            else:
                self.terminado = True
        return self.ruta[min(j, len(self.ruta) - 1)]

    def orden(self, pose) -> tuple[str, dict]:
        """Qué debe hacer el robot ahora: (accion, detalle).

        `pose` = (x, y, theta_rad). La acción es la misma que usa el manejo
        por pulsos, porque este chasis no admite una curvatura fina."""
        x, y, th = pose
        obj = self.objetivo(x, y)
        if obj is None or self.terminado:
            return "parar", {"motivo": "ruta terminada"}
        rumbo = math.atan2(obj[1] - y, obj[0] - x)
        error = math.degrees((rumbo - th + math.pi) % (2 * math.pi) - math.pi)
        detalle = {"objetivo": [round(float(obj[0]), 2), round(float(obj[1]), 2)],
                   "error_rumbo_deg": round(error, 1),
                   "desvio_cm": round(desvio(self.ruta, (x, y)) * 100, 1),
                   "avance_pct": round(100 * self.i / max(len(self.ruta) - 1, 1), 1)}
        if abs(error) <= TOLERANCIA_RUMBO_DEG:
            return "adelante", detalle
        return ("izquierda" if error > 0 else "derecha"), detalle
