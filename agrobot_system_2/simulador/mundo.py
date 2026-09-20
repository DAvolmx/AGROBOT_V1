"""
Mundo 2D del simulador: qué hay en el terreno y qué ve cada sensor.
===================================================================

La distinción que da sentido a todo el simulador (y que es el problema real
del robot, medido el 2026-09-19):

  · ALTO (paredes, cercas, taludes, cajas): lo ve el LIDAR, que va a ~30 cm
    del suelo, y frena por él.
  · BAJO (sendero, paja del mulch, plantas chicas): el LIDAR pasa por
    ENCIMA y no lo ve. Solo existe para la cámara.

Así, en el simulador, seguir el sendero obliga a usar la cámara igual que en
el campo, y el LIDAR sirve para no chocar.

Ejes del mundo en metros: x a la derecha, y hacia arriba, ángulos en radianes
con 0 = +x (igual que el mapa del SLAM).
"""
import math

import numpy as np

# Colores BGR del render de cámara, tomados de las fotos de la huerta real.
COLOR_TIERRA = (78, 96, 112)
COLOR_PAJA = (105, 165, 195)
COLOR_PLANTA = (50, 150, 70)
COLOR_PARED = (120, 120, 125)
COLOR_CIELO = (205, 195, 185)


class Mundo:
    """Terreno: paredes (LIDAR), sendero con paja (cámara) y plantas."""

    def __init__(self, nombre, paredes=(), sendero=(), ancho_sendero=0.6,
                 ancho_paja=0.35, plantas=(), limites=(-6, -6, 6, 6)):
        self.nombre = nombre
        self.paredes = np.array(paredes, dtype=np.float32).reshape(-1, 4)   # x1,y1,x2,y2
        self.sendero = np.array(sendero, dtype=np.float32).reshape(-1, 2)   # polilínea del centro
        self.ancho_sendero = ancho_sendero
        self.ancho_paja = ancho_paja
        self.plantas = np.array(plantas, dtype=np.float32).reshape(-1, 4)   # x,y,radio,plaga(0/1)
        self.limites = limites

    # ---------------------------------------------------------------- LIDAR
    def rayos(self, x, y, theta, angulos, alcance=8.0):
        """Distancia a la primera pared para cada ángulo (relativo a theta).

        Vectorizado: todos los rayos contra todas las paredes de una vez.
        Devuelve `alcance` donde no hay nada (el driver marca esos haces como
        fuera de rango, igual que el C1 real)."""
        if len(self.paredes) == 0:
            return np.full(len(angulos), alcance, np.float32)
        ang = theta + np.asarray(angulos, np.float32)
        dx, dy = np.cos(ang), np.sin(ang)                       # (R,)
        x1, y1, x2, y2 = self.paredes.T                          # (P,)
        ex, ey = (x2 - x1)[None, :], (y2 - y1)[None, :]          # (1,P)
        den = dx[:, None] * ey - dy[:, None] * ex                # (R,P)
        seguro = np.where(np.abs(den) < 1e-9, np.nan, den)
        px, py = (x1 - x)[None, :], (y1 - y)[None, :]
        t = (px * ey - py * ex) / seguro                         # avance sobre el rayo
        u = (px * dy[:, None] - py * dx[:, None]) / seguro       # posición en la pared 0..1
        valido = (t > 0.02) & (u >= 0) & (u <= 1) & np.isfinite(t)
        t = np.where(valido, t, np.inf)
        return np.minimum(t.min(axis=1), alcance).astype(np.float32)

    def scan(self, x, y, theta, n=720, alcance=8.0, ruido_m=0.01, rng=None):
        """Scan de 360° como el del RPLIDAR C1: lista de puntos con el ángulo
        en grados relativo al robot (0 = adelante, + = izquierda)."""
        ang = np.linspace(-math.pi, math.pi, n, endpoint=False, dtype=np.float32)
        d = self.rayos(x, y, theta, ang, alcance)
        if ruido_m and rng is not None:
            d = d + rng.normal(0, ruido_m, d.shape).astype(np.float32)
        visible = d < alcance - 1e-3
        return [{"angle_deg": float(math.degrees(a)), "distance_m": float(r)}
                for a, r in zip(ang[visible], d[visible])]

    # ------------------------------------------------------------- geometría
    def distancia_al_sendero(self, px, py):
        """Distancia (m) de cada punto al eje del sendero. Vectorizado."""
        if len(self.sendero) < 2:
            return np.full(np.shape(px), 1e3, np.float32)
        a = self.sendero[:-1]
        b = self.sendero[1:]
        ab = b - a
        largo2 = np.maximum((ab ** 2).sum(axis=1), 1e-9)
        ap_x = px[..., None] - a[:, 0]
        ap_y = py[..., None] - a[:, 1]
        t = np.clip((ap_x * ab[:, 0] + ap_y * ab[:, 1]) / largo2, 0, 1)
        dx = ap_x - t * ab[:, 0]
        dy = ap_y - t * ab[:, 1]
        return np.sqrt(dx * dx + dy * dy).min(axis=-1)

    def clases_suelo(self, px, py, rng=None):
        """Clasifica puntos del suelo: 0 tierra, 1 paja, 2 planta.

        La paja se dibuja fibrosa (ruido determinista según la posición) y
        con algunas pajas sueltas sobre el camino, como en la huerta real."""
        d = self.distancia_al_sendero(px, py)
        media = self.ancho_sendero / 2
        fibra = (np.sin(px * 173 + py * 41) * np.sin(px * 57 - py * 211))
        if rng is not None:
            fibra = fibra + rng.normal(0, 0.35, fibra.shape)
        en_banda = (d > media) & (d < media + self.ancho_paja)
        clase = np.where(en_banda & (fibra > -0.4), 1, 0)
        clase = np.where((d <= media) & (fibra > 0.85), 1, clase)   # paja suelta en el camino
        clase = np.where(d >= media + self.ancho_paja, 1 * (fibra > 0.2), clase)  # fuera: rastrojo ralo
        for cx, cy, r, _ in self.plantas:
            clase = np.where((px - cx) ** 2 + (py - cy) ** 2 < r * r, 2, clase)
        return clase

    # ---------------------------------------------------------------- cámara
    def camara(self, x, y, theta, geo, ancho=640, alto=480, rng=None):
        """Imagen BGR sintética desde la cámara del robot.

        Cada píxel se proyecta al suelo con la misma geometría que usa
        `sendero.py` (altura, inclinación, campo y adelantamiento de la
        cámara). Lo que cae por encima del horizonte, o más allá de la
        primera pared en ese rumbo, se pinta como pared o cielo."""
        h = geo["camara_altura_m"]
        phi = math.radians(geo["camara_inclinacion_deg"])
        fx = (ancho / 2) / math.tan(math.radians(geo["camara_hfov_deg"]) / 2)
        u, v = np.meshgrid(np.arange(ancho, dtype=np.float32),
                           np.arange(alto, dtype=np.float32))
        xn, yn = (u - ancho / 2) / fx, (v - alto / 2) / fx
        baja = yn * math.cos(phi) + math.sin(phi)
        suelo = baja > 1e-3
        t = np.where(suelo, h / np.where(suelo, baja, 1.0), 0.0)
        # En ejes del robot y luego al mundo.
        xr = t * (math.cos(phi) - yn * math.sin(phi)) + geo["camara_adelante_m"]
        yr = -t * xn
        px = x + xr * math.cos(theta) - yr * math.sin(theta)
        py = y + xr * math.sin(theta) + yr * math.cos(theta)

        clase = self.clases_suelo(px, py, rng)
        img = np.empty((alto, ancho, 3), np.float32)
        img[:] = COLOR_TIERRA
        img[clase == 1] = COLOR_PAJA
        img[clase == 2] = COLOR_PLANTA
        if rng is not None:
            img += rng.normal(0, 9, img.shape)

        # Paredes: un rayo por columna; el suelo más lejano que la pared queda tapado.
        if len(self.paredes):
            ang = np.arctan2(-xn[0], 1.0)          # rumbo relativo de cada columna
            dist_pared = self.rayos(x, y, theta, ang, alcance=30.0)
            dist_suelo = np.hypot(px - x, py - y)
            tapado = dist_suelo > dist_pared[None, :]
            img[tapado] = COLOR_PARED
        img[~suelo] = COLOR_CIELO
        return np.clip(img, 0, 255).astype(np.uint8)

    # ------------------------------------------------------------ vista cenital
    def dibujo(self, px_por_m=60):
        """Vista desde arriba del mundo (BGR), para ver los episodios."""
        x0, y0, x1, y1 = self.limites
        ancho = int((x1 - x0) * px_por_m)
        alto = int((y1 - y0) * px_por_m)
        gx, gy = np.meshgrid(np.linspace(x0, x1, ancho, dtype=np.float32),
                             np.linspace(y1, y0, alto, dtype=np.float32))
        clase = self.clases_suelo(gx, gy)
        img = np.empty((alto, ancho, 3), np.uint8)
        img[:] = COLOR_TIERRA
        img[clase == 1] = COLOR_PAJA
        img[clase == 2] = COLOR_PLANTA
        return img

    def a_pixel(self, x, y, px_por_m=60):
        x0, y0, x1, y1 = self.limites
        return int((x - x0) * px_por_m), int((y1 - y) * px_por_m)


# ---------------------------------------------------------------------------
# Escenarios. Cada uno devuelve (mundo, pose inicial) y entrena algo distinto.
# ---------------------------------------------------------------------------
def _rect(x0, y0, x1, y1):
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def _plantas_en_linea(p0, p1, paso=0.35, radio=0.07, plagas=0.15, semilla=0):
    rng = np.random.default_rng(semilla)
    largo = math.dist(p0, p1)
    n = max(1, int(largo / paso))
    out = []
    for i in range(n + 1):
        f = i / max(n, 1)
        x = p0[0] + (p1[0] - p0[0]) * f + rng.normal(0, 0.03)
        y = p0[1] + (p1[1] - p0[1]) * f + rng.normal(0, 0.03)
        out.append((x, y, radio * rng.uniform(0.7, 1.4), 1.0 if rng.random() < plagas else 0.0))
    return out


def bucle():
    """La huerta del usuario: sendero en bucle alrededor de un cantero central,
    cercado. Entrena seguir el sendero y dar la vuelta completa."""
    paredes = _rect(-4.5, -3.5, 4.5, 3.5)
    ruta = [(-3, -2), (3, -2), (3.4, -1.6), (3.4, 1.6), (3, 2), (-3, 2),
            (-3.4, 1.6), (-3.4, -1.6), (-3, -2)]
    plantas = (_plantas_en_linea((-2.6, -0.9), (2.6, -0.9), semilla=1)
               + _plantas_en_linea((-2.6, 0.9), (2.6, 0.9), semilla=2)
               + _plantas_en_linea((-2.6, 0.0), (2.6, 0.0), semilla=3))
    return Mundo("bucle", paredes, ruta, 0.65, 0.35, plantas, (-5, -4, 5, 4)), (-3.0, -2.0, 0.0)


def hileras():
    """Cultivo en hileras paralelas: entrena centrarse entre dos hileras."""
    paredes = _rect(-5, -4, 5, 4)
    ruta = [(-4.2, 0), (4.2, 0)]
    plantas = []
    for k, yy in enumerate((-0.85, 0.85, -2.2, 2.2)):
        plantas += _plantas_en_linea((-4.2, yy), (4.2, yy), semilla=10 + k)
    return Mundo("hileras", paredes, ruta, 0.7, 0.3, plantas, (-5.5, -4.5, 5.5, 4.5)), (-4.0, 0.0, 0.0)


def pasillo_estrecho():
    """Pasillo de 0,9 m con un giro: entrena no rozar y girar en sitio justo."""
    paredes = [(-4, -0.45, 1.5, -0.45), (-4, 0.45, 0.6, 0.45),
               (0.6, 0.45, 0.6, 3.5), (1.5, -0.45, 1.5, 3.5),
               (-4, -0.45, -4, 0.45), (0.6, 3.5, 1.5, 3.5)]
    ruta = [(-3.6, 0), (1.05, 0), (1.05, 3.2)]
    return Mundo("pasillo", paredes, ruta, 0.7, 0.2, [], (-4.5, -1.5, 2.5, 4.5)), (-3.5, 0.0, 0.0)


def callejon_sin_salida():
    """Bolsillo cerrado: entrena reconocer que no hay salida y volver atrás."""
    paredes = (_rect(-5, -3, 5, 3)
               + [(0.5, -3, 0.5, 1.2), (2.5, -3, 2.5, 1.2), (0.5, 1.2, 2.5, 1.2)])
    ruta = [(-4, -1.5), (1.5, -1.5), (1.5, 0.8)]
    return Mundo("callejon", paredes, ruta, 0.7, 0.25, [], (-5.5, -3.5, 5.5, 3.5)), (-4.0, -1.5, 0.0)


def trampa_bucle():
    """Sala con un obstáculo central simétrico: el clásico sitio donde una
    política reactiva se queda dando vueltas. Entrena detectar el bucle."""
    paredes = _rect(-3.5, -3.5, 3.5, 3.5) + _rect(-0.8, -0.8, 0.8, 0.8)
    ruta = [(-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (-2.5, 2.5), (-2.5, -2.5)]
    return Mundo("trampa_bucle", paredes, ruta, 0.8, 0.3, [], (-4, -4, 4, 4)), (-2.5, -2.5, 0.0)


ESCENARIOS = {
    "bucle": bucle,
    "hileras": hileras,
    "pasillo": pasillo_estrecho,
    "callejon": callejon_sin_salida,
    "trampa_bucle": trampa_bucle,
}
