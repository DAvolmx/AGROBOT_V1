"""Calibración de NIVEL del IMU: con el robot quieto y nivelado, la gravedad medida define el «arriba» del sensor.

El IMU no va montado perfectamente plano (el 2026-10-19, con el robot nivelado en una mesa, marcaba
accelX=0,25 g: ~14° de balanceo falso). Ese error sumado a la inclinación real disparaba el aviso de vuelco y
sesgaba la compensación de cabeceo de la cámara. Aquí se guarda el vector de gravedad medido en nivel y se
calcula la rotación que lo lleva a (0, 0, |g|); se aplica a acelerómetro y giroscopio antes de usarlos.

Ejes del sensor: Y adelante, X lateral, Z arriba (ver jetson_server: remapeo a ROS).
"""
from __future__ import annotations

import math

import numpy as np

G_MIN, G_MAX = 0.9, 1.1        # |g| aceptable en reposo (el sensor mide en g)
RUIDO_MAX_G = 0.03             # desviación máxima de una muestra a otra: más = se está moviendo


def matriz_nivel(g_nivel):
    """Matriz 3x3 que gira el vector `g_nivel` hasta (0, 0, |g|). None si el vector no es válido."""
    try:
        g = np.asarray([float(v) for v in g_nivel], float)
    except (TypeError, ValueError):
        return None
    if g.shape != (3,) or not np.all(np.isfinite(g)):
        return None
    n = float(np.linalg.norm(g))
    if not (G_MIN <= n <= G_MAX) or g[2] <= 0:
        return None
    u = g / n
    eje = np.cross(u, [0.0, 0.0, 1.0])
    s = float(np.linalg.norm(eje))
    c = float(u[2])
    if s < 1e-9:
        return np.eye(3)
    eje /= s
    K = np.array([[0, -eje[2], eje[1]], [eje[2], 0, -eje[0]], [-eje[1], eje[0], 0]])
    return np.eye(3) + s * K + (1 - c) * (K @ K)


def aplicar(R, vec):
    """Gira un vector (x, y, z) con la matriz de nivel; sin matriz devuelve el vector igual."""
    if R is None:
        return tuple(vec)
    r = R @ np.asarray(vec, float)
    return float(r[0]), float(r[1]), float(r[2])


def inclinacion_deg(g_nivel) -> tuple[float, float] | None:
    """(balanceo, cabeceo) en grados del montaje respecto a la vertical; para informar al operador."""
    try:
        ax, ay, az = (float(v) for v in g_nivel)
    except (TypeError, ValueError):
        return None
    return math.degrees(math.atan2(ax, az)), math.degrees(math.atan2(ay, az))


def promediar(muestras):
    """Media de muestras (ax, ay, az) en reposo. Devuelve (vector, mensaje_de_error|None)."""
    a = np.asarray(muestras, float)
    if a.ndim != 2 or a.shape[0] < 10:
        return None, "pocas muestras del IMU (¿está encendido el nodo de sensores?)"
    if float(np.max(np.std(a, axis=0))) > RUIDO_MAX_G:
        return None, "el IMU no está quieto: no muevas el robot durante la calibración"
    m = a.mean(axis=0)
    n = float(np.linalg.norm(m))
    if not (G_MIN <= n <= G_MAX):
        return None, f"|g| = {n:.2f} fuera de {G_MIN}-{G_MAX}: el acelerómetro no mide 1 g en reposo"
    if m[2] <= 0:
        return None, "el IMU está boca abajo"
    return [round(float(v), 4) for v in m], None
