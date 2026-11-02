"""Sendero por FILTROS de imagen (2026-10-29): combinación logística de color, textura y BORDES VERTICALES.

Por qué: medido sobre 790 fotos de huerta con etiquetas de la trayectoria recorrida (docs/SESION_CAMPO_20261029/filtros_sendero.json), los rasgos que mejor separan
el camino de las plantas son los bordes verticales (Gabor 0°, Sobel en x) y la textura local; el color solo y el método por apariencia puro quedan por debajo
(AUC por bandas 0,65 frente a 0,80 de la combinación). A igual recall de camino (76 %) marca ~35 % de «peligro» frente a ~53 % del método por apariencia.
Modelo podado a 6 rasgos (con 24 separaba igual: AUC 0,81 frente a 0,80). Pesos entrenados en 6 sesiones de exterior y medidos en 2 sesiones reservadas (28 y 29 de octubre). Se activa con autonomia.sendero_apariencia_filtros = true; por defecto está apagado.
Entrada: imagen BGR de 320x240 sin igualar la luz (así se entrenó). Salida: máscara 0/255 del camino conectado a la franja de abajo al centro y el puntaje por píxel."""
from __future__ import annotations

import numpy as np

USO = ['color_ref', 'std_L15', 'sobel_v9', 'L', 'V', 'a']
MU = np.array([-1.58611, 10.18077, 22.23095, 125.9793, 123.71332, 129.01103], np.float32)
SD = np.array([1.41581, 5.93901, 16.45988, 38.02747, 36.58819, 2.44208], np.float32)
W = np.array([0.75646, -0.37309, -0.2316, -0.00036, 0.38026, 0.29587, 0.20943], np.float32)
UMBRAL = 0.0            # 0,25 = más estricto (recall 57 %, peligro 5 %); -0,25 = más permisivo (76 %, 35 %)


def _box(cv2, x, k):
    return cv2.boxFilter(x, -1, (k, k))


def _std(cv2, x, k):
    return np.sqrt(np.maximum(_box(cv2, x * x, k) - _box(cv2, x, k) ** 2, 0))


def rasgos(cv2, bgr):
    """Solo los 6 rasgos que usa el modelo (color respecto al suelo de delante, textura local, bordes verticales, brillo): ~10 ms en CPU."""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    h, w = gray.shape
    F = {"L": lab[..., 0], "a": lab[..., 1], "V": bgr.max(axis=2).astype(np.float32),
         "std_L15": _std(cv2, lab[..., 0], 15),
         "sobel_v9": _box(cv2, np.abs(cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)), 9)}
    X = np.dstack([lab[..., 1], lab[..., 2], lab[..., 0], _std(cv2, lab[..., 0], 9)])
    ref = X[int(h * .8):int(h * .98), int(w * .36):int(w * .64)].reshape(-1, 4)
    med = np.median(ref, 0)
    mad = np.maximum(np.median(np.abs(ref - med), 0) * 1.4826, [2, 2, 6, 1.5])
    F["color_ref"] = -np.sqrt((((X[..., :3] - med[:3]) / mad[:3]) ** 2 * np.array([1, 1, .35]) ** 2).sum(2))
    return F


def puntaje(cv2, bgr):
    """Puntaje por píxel (mayor = más parecido a camino), suavizado."""
    F = rasgos(cv2, bgr)
    X = np.stack([F[n] for n in USO], -1)
    s = ((X - MU) / SD) @ W[:-1] + W[-1]
    return cv2.GaussianBlur(s.astype(np.float32), (0, 0), 4)


def mascara(cv2, bgr, umbral: float = UMBRAL):
    s = puntaje(cv2, bgr)
    h, w = s.shape
    m = (s > umbral).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    ids = [i for i in np.unique(lab[int(h * 0.92), int(w * 0.40):int(w * 0.60)]) if i != 0]
    if not ids:
        return np.zeros_like(m), s
    j = max(ids, key=lambda i: st[i, cv2.CC_STAT_AREA])
    return ((lab == j) * 255).astype(np.uint8), s
