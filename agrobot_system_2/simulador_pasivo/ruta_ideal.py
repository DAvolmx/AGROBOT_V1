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
        self._inicio = False
        self._n_ciclo = (len(self.ruta) - 1 if cerrado and len(self.ruta) > 2
                         and np.linalg.norm(self.ruta[0] - self.ruta[-1]) < 1e-8
                         else len(self.ruta))

    def _avanzar_progreso_local(self, x, y):
        """Actualiza progreso en una ventana local para no saltar de fila.

        En un sendero en U, dos tramos paralelos pueden estar separados por
        menos que el error de pose. La búsqueda global engancha el tramo de
        vuelta y crea una orden de giro brusca. Tras la inicialización solo se
        consideran unos centímetros hacia atrás y hasta 1.5 m por delante.
        """
        n = self._n_ciclo
        if n <= 0:
            return
        if not self._inicio:
            self.i = int(np.argmin(np.linalg.norm(self.ruta[:n] - [x, y], axis=1)))
            self._inicio = True
            return
        if self.cerrado:
            paso = float(np.median(np.linalg.norm(np.diff(self.ruta, axis=0), axis=1))) if len(self.ruta) > 2 else self.mirada
            ventana = max(8, int(math.ceil((self.mirada + 1.5) / max(paso, 0.02))))
            lo, hi = self.i - 2, self.i + ventana
            candidatos = range(lo, hi + 1)
            mejor = min(candidatos, key=lambda k: float(np.sum((self.ruta[k % n] - [x, y]) ** 2)) - 1e-8 * k)
            self.i = max(lo, mejor)
            self.vueltas = max(self.vueltas, self.i // n)
        else:
            paso = float(np.median(np.linalg.norm(np.diff(self.ruta, axis=0), axis=1))) if len(self.ruta) > 1 else self.mirada
            ventana = max(8, int(math.ceil((self.mirada + 1.5) / max(paso, 0.02))))
            lo, hi = max(0, self.i - 2), min(n - 1, self.i + ventana)
            mejor = min(range(lo, hi + 1), key=lambda k: float(np.sum((self.ruta[k] - [x, y]) ** 2)))
            self.i = max(self.i, mejor)

    def objetivo(self, x, y):
        """Punto de la ruta a `mirada` metros por delante del robot."""
        if not len(self.ruta):
            return None
        # Solo se mira hacia adelante en la ruta: si no, al volver cerca del
        # inicio el robot se engancharía al tramo de atrás.
        self._avanzar_progreso_local(x, y)
        j = int(self.i)
        limite = self._n_ciclo if self.cerrado else len(self.ruta) - 1
        distancia = 0.0
        while distancia < self.mirada and (self.cerrado or j < limite):
            actual = j % self._n_ciclo if self.cerrado else j
            siguiente = (actual + 1) % self._n_ciclo if self.cerrado else min(actual + 1, len(self.ruta) - 1)
            distancia += math.dist(self.ruta[actual], self.ruta[siguiente])
            j += 1
        if not self.cerrado and j >= limite and math.dist(self.ruta[-1], (x, y)) < LLEGADA_M:
            self.terminado = True
        return self.ruta[j % self._n_ciclo] if self.cerrado else self.ruta[min(j, len(self.ruta) - 1)]

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
                   "avance_pct": round(100 * (self.i % max(self._n_ciclo, 1)) / max(self._n_ciclo - 1, 1), 1)}
        if abs(error) <= TOLERANCIA_RUMBO_DEG:
            return "adelante", detalle
        return ("izquierda" if error > 0 else "derecha"), detalle
