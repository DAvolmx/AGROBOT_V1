"""
Vista 3D del mundo simulado, sin dependencias de gráficos 3D.

La idea que evita traer una librería entera (y que no necesita internet ni
GPU): el suelo es un PLANO, así que su proyección en perspectiva es una
homografía. Se toma la vista cenital que ya dibuja `mundo.py` (tierra, paja,
plantas) y se deforma con `cv2.warpPerspective`. Encima se pintan las
paredes como planos verticales y el robot como una caja, ordenados de lejos a
cerca (algoritmo del pintor).

Cámara orbital: `yaw` y `pitch` alrededor de un objetivo, a `distancia`.
"""
import math

import cv2
import numpy as np

ALTURA_PARED_M = 0.6
ALTURA_PLANTA_M = 0.14
COLOR_PLANTA_3D = (55, 150, 70)
COLOR_PLAGA_3D = (60, 200, 220)
COLOR_ESCANEO = (230, 90, 230)
ROBOT_LARGO, ROBOT_ANCHO, ROBOT_ALTO = 0.40, 0.35, 0.22
CIELO = (150, 125, 100)


class Camara:
    """Cámara en órbita: mira a `objetivo` desde yaw/pitch/distancia."""

    def __init__(self, objetivo=(0.0, 0.0), yaw=-60.0, pitch=32.0, distancia=7.0,
                 ancho=720, alto=460, fov=60.0):
        self.objetivo = objetivo
        self.yaw, self.pitch, self.distancia = yaw, pitch, distancia
        self.ancho, self.alto, self.fov = ancho, alto, fov

    def matriz(self):
        """(R, t, f): rotación mundo->cámara, posición de la cámara y focal."""
        ty, tp = math.radians(self.yaw), math.radians(max(5.0, min(85.0, self.pitch)))
        ojo = np.array([
            self.objetivo[0] - self.distancia * math.cos(tp) * math.cos(ty),
            self.objetivo[1] - self.distancia * math.cos(tp) * math.sin(ty),
            self.distancia * math.sin(tp)], np.float64)
        objetivo = np.array([self.objetivo[0], self.objetivo[1], 0.0])
        adelante = objetivo - ojo
        adelante /= np.linalg.norm(adelante)
        derecha = np.cross(adelante, [0, 0, 1.0])
        derecha /= np.linalg.norm(derecha)
        abajo = np.cross(adelante, derecha)
        R = np.stack([derecha, abajo, adelante])          # filas: x, y, z de cámara
        f = (self.ancho / 2) / math.tan(math.radians(self.fov) / 2)
        return R, ojo, f

    def proyectar(self, puntos):
        """(N,3) mundo -> (N,2) píxeles + máscara de los que están delante."""
        R, ojo, f = self.matriz()
        cam = (np.asarray(puntos, np.float64) - ojo) @ R.T
        z = cam[:, 2]
        delante = z > 0.05
        z = np.where(delante, z, 1.0)
        u = self.ancho / 2 + f * cam[:, 0] / z
        v = self.alto / 2 + f * cam[:, 1] / z
        return np.stack([u, v], axis=1), delante, cam[:, 2]


def _quad(img, esquinas, color, borde=None):
    pts = np.round(esquinas).astype(np.int32)
    cv2.fillConvexPoly(img, pts, color, cv2.LINE_AA)
    if borde is not None:
        cv2.polylines(img, [pts], True, borde, 1, cv2.LINE_AA)


def render(mundo, pose=None, cam: Camara = None, traza=None, px_por_m=40):
    """Imagen BGR del mundo en perspectiva. `pose` = (x, y, theta) del robot."""
    cam = cam or Camara()
    img = np.empty((cam.alto, cam.ancho, 3), np.uint8)
    img[:] = CIELO

    # --- Suelo: un rayo por píxel contra el plano z=0 ---
    # (Antes era una homografía desde la vista cenital, pero exigía que las
    # cuatro esquinas del mundo estuvieran delante de la cámara: de cerca o
    # de lado el suelo desaparecía.)
    R, ojo, f = cam.matriz()
    cenital = mundo.dibujo(px_por_m)
    alto_c, ancho_c = cenital.shape[:2]
    x0, y0, x1, y1 = mundo.limites
    u, v = np.meshgrid(np.arange(cam.ancho, dtype=np.float32),
                       np.arange(cam.alto, dtype=np.float32))
    rayos = np.stack([(u - cam.ancho / 2) / f, (v - cam.alto / 2) / f,
                      np.ones_like(u)], axis=-1) @ R          # a ejes del mundo
    baja = rayos[..., 2]
    toca = baja < -1e-6
    t = np.where(toca, -ojo[2] / np.where(toca, baja, -1.0), 0.0)
    gx = ojo[0] + t * rayos[..., 0]
    gy = ojo[1] + t * rayos[..., 1]
    mapa_x = ((gx - x0) / (x1 - x0) * ancho_c).astype(np.float32)
    mapa_y = ((y1 - gy) / (y1 - y0) * alto_c).astype(np.float32)
    dentro = toca & (mapa_x >= 0) & (mapa_x < ancho_c) & (mapa_y >= 0) & (mapa_y < alto_c)
    suelo = cv2.remap(cenital, mapa_x, mapa_y, cv2.INTER_LINEAR,
                      borderMode=cv2.BORDER_CONSTANT, borderValue=CIELO)
    img[dentro] = suelo[dentro]

    # --- Traza recorrida ---
    if traza is not None and len(traza) > 1:
        pts3 = np.array([[x, y, 0.01] for x, y in traza], np.float64)
        uv, ok, _ = cam.proyectar(pts3)
        buenos = uv[ok]
        if len(buenos) > 1:
            cv2.polylines(img, [np.round(buenos).astype(np.int32)], False, (220, 80, 220), 2, cv2.LINE_AA)

    piezas = []

    # --- Plantas: cruces de dos planos verticales (truco clásico: parecen
    # volumen desde cualquier ángulo y cuestan cuatro polígonos) ---
    for cx, cy, r, plaga in getattr(mundo, "plantas", []):
        alto_p = max(ALTURA_PLANTA_M, float(r) * 2.2)
        color = COLOR_PLAGA_3D if plaga else COLOR_PLANTA_3D
        for dx, dy in ((r, 0.0), (0.0, r)):
            hoja = np.array([[cx - dx, cy - dy, 0], [cx + dx, cy + dy, 0],
                             [cx + dx, cy + dy, alto_p], [cx - dx, cy - dy, alto_p]], np.float64)
            piezas.append((math.dist((cx, cy), (ojo[0], ojo[1])), hoja, color))

    # --- Puntos de escaneo: un poste fino para verlos desde lejos ---
    for ex, ey in getattr(mundo, "escaneos", []):
        poste = np.array([[ex - 0.03, ey, 0], [ex + 0.03, ey, 0],
                          [ex + 0.03, ey, 0.45], [ex - 0.03, ey, 0.45]], np.float64)
        piezas.append((math.dist((ex, ey), (ojo[0], ojo[1])), poste, COLOR_ESCANEO))

    # --- Paredes: planos verticales, de lejos a cerca ---
    for x1w, y1w, x2w, y2w in mundo.paredes:
        medio = ((x1w + x2w) / 2, (y1w + y2w) / 2)
        d = math.dist(medio, (ojo[0], ojo[1]))
        esquinas = np.array([[x1w, y1w, 0], [x2w, y2w, 0],
                             [x2w, y2w, ALTURA_PARED_M], [x1w, y1w, ALTURA_PARED_M]], np.float64)
        # Sombreado simple según la orientación, para que se distingan las caras.
        ang = math.atan2(y2w - y1w, x2w - x1w)
        tono = 105 + int(40 * abs(math.cos(ang)))
        piezas.append((d, esquinas, (tono, tono, tono + 6)))

    if pose is not None:
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        L, A, H_ = ROBOT_LARGO / 2, ROBOT_ANCHO / 2, ROBOT_ALTO
        base = [(x + c * dx - s * dy, y + s * dx + c * dy)
                for dx, dy in ((L, A), (L, -A), (-L, -A), (-L, A))]
        techo = np.array([[bx, by, H_] for bx, by in base], np.float64)
        piezas.append((math.dist((x, y), (ojo[0], ojo[1])) - 0.01, techo, (60, 60, 210)))
        for i in range(4):
            a, b = base[i], base[(i + 1) % 4]
            lado = np.array([[a[0], a[1], 0], [b[0], b[1], 0],
                             [b[0], b[1], H_], [a[0], a[1], H_]], np.float64)
            piezas.append((math.dist(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2), (ojo[0], ojo[1])),
                           lado, (45, 45, 170)))
        # Flecha del rumbo
        punta = np.array([[x + c * (L + 0.22), y + s * (L + 0.22), H_ + 0.01],
                          [x + c * L - s * 0.08, y + s * L + c * 0.08, H_ + 0.01],
                          [x + c * L + s * 0.08, y + s * L - c * 0.08, H_ + 0.01]], np.float64)
        piezas.append((-1.0, punta, (255, 255, 255)))

    for _, esquinas, color in sorted(piezas, key=lambda p: -p[0]):
        uv, ok, _ = cam.proyectar(esquinas)
        if ok.all():
            _quad(img, uv, color, borde=(40, 40, 40))
    return img
