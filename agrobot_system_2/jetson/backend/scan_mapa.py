"""Scan del LIDAR llevado al marco del mapa, para pintarlo EN VIVO sobre él.

El radar del dashboard dibuja el scan en el marco del robot (el robot siempre
en el centro). Sobre el mapa de slam_toolbox hace falta el scan en coordenadas
del mapa: pose del robot en `map` (TF de slam_toolbox, no el EKF de `odom`,
que deriva respecto al mapa) + rotación + traslación de cada rayo.

Se difunde en centímetros enteros y aplanado ([x0, y0, x1, y1, ...]): un
mensaje de 270 puntos son ~2 kB en vez de ~10 kB con objetos {x, y}.
"""
import math

# El C1 es fiable hasta ~8 m en interiores; más allá domina el multipath y los
# puntos son fantasmas (mismo límite que slam_toolbox_mapping.yaml).
RANGO_MAX_M = 8.0
RANGO_MIN_M = 0.12
MAX_PUNTOS = 270


def scan_a_mapa(points: list[dict], pose: dict,
                maximo: int = MAX_PUNTOS) -> list[int]:
    """`points`: [{"angle_deg", "distance_m"}] en el marco del robot (0° =
    adelante, antihorario). `pose`: {"x", "y", "theta_deg"} en el marco mapa.
    Devuelve [x0_cm, y0_cm, x1_cm, ...] diezmado a `maximo` puntos."""
    validos = [p for p in points if RANGO_MIN_M <= p["distance_m"] <= RANGO_MAX_M]
    if len(validos) > maximo:
        paso = len(validos) / maximo
        validos = [validos[int(i * paso)] for i in range(maximo)]
    th = math.radians(pose["theta_deg"])
    c, s = math.cos(th), math.sin(th)
    out: list[int] = []
    for p in validos:
        a = math.radians(p["angle_deg"])
        lx = p["distance_m"] * math.cos(a)
        ly = p["distance_m"] * math.sin(a)
        out.append(round((pose["x"] + c * lx - s * ly) * 100))
        out.append(round((pose["y"] + s * lx + c * ly) * 100))
    return out
