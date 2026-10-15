"""DESCARTADO (2026-10-15) - NO USAR. Probado sobre 4 fotos reales: marca 291-791 "insectos" por foto
(reflejos y textura de hoja), el agujero grande de la acelga no se detecta y las "lesiones" son ~3 %
constante (sombras y nervaduras). Se conserva solo como registro de lo que no funcionó.

Indicios de plaga y enfermedad en hojas (col, coliflor, acelga) con visión clásica.

NO es un modelo entrenado: no hay fotos etiquetadas suficientes para entrenar uno (el 2026-10-15
solo había 4 fotos reales sin etiquetas). Son reglas de imagen explícitas, validadas A OJO sobre
esas 4 fotos. Sirve para producir INDICIOS y para PRE-ETIQUETAR fotos que luego se corrigen a mano
y alimentan un entrenamiento real (ver `preetiquetar.py`). No diagnostica.

Tres indicios:
  · insectos_pequenos : manchas pequeñas y CLARAS (mosca blanca, trips, pulgón claro) sobre la hoja.
  · agujeros          : tejido ausente rodeado de hoja (masticadores: orugas, caracoles, escarabajos).
  · lesiones          : zonas oscuras/pardas dentro de la hoja (candidato a tizón u otra necrosis).

LÍMITES CONOCIDOS: sombras y la tierra del fondo pueden dar falsos positivos de 'lesiones' y de
'agujeros'; reflejos brillantes dan falsos 'insectos'. Las cifras son relativas a la imagen: un
umbral de 'infestación' solo vale si se calibra con las fotos de la planta real.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class Indicios:
    cobertura_hoja: float = 0.0
    insectos: list = field(default_factory=list)      # [(x, y, r)] en píxeles de la imagen original
    agujeros: list = field(default_factory=list)      # [(x, y, area_px)]
    area_agujeros_pct: float = 0.0
    lesiones_pct: float = 0.0
    mascara_hoja: np.ndarray | None = None
    mascara_lesion: np.ndarray | None = None

    def resumen(self):
        return {"cobertura_hoja_pct": round(100 * self.cobertura_hoja, 1), "insectos_pequenos": len(self.insectos),
                "agujeros": len(self.agujeros), "area_agujeros_pct_hoja": round(self.area_agujeros_pct, 2),
                "lesiones_pct_hoja": round(self.lesiones_pct, 2)}


def mascara_hoja(bgr):
    """Verde y verde-azulado (col/coliflor tiran a azul-gris): ExG relajado + tono entre amarillo y cian."""
    small = bgr
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    b, g, r = [small[..., i].astype(np.float32) for i in range(3)]
    exg = 2 * g - r - b
    m = ((h >= 28) & (h <= 100) & (s >= 28) & (v >= 40)) | ((exg > 12) & (s >= 18))
    m = m.astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    return m


def agujeros(m_hoja, min_pct=0.0004, max_pct=0.08):
    """Regiones NO-hoja completamente rodeadas de hoja (flood fill desde el borde)."""
    h, w = m_hoja.shape
    inv = (m_hoja == 0).astype(np.uint8) * 255
    relleno = inv.copy()
    mask = np.zeros((h + 2, w + 2), np.uint8)
    for (x, y) in ((0, 0), (w - 1, 0), (0, h - 1), (w - 1, h - 1)):
        if relleno[y, x] == 255:
            cv2.floodFill(relleno, mask, (x, y), 0)
    for x in range(0, w, max(1, w // 40)):                      # y desde todo el contorno
        for y in (0, h - 1):
            if relleno[y, x] == 255:
                cv2.floodFill(relleno, mask, (x, y), 0)
    for y in range(0, h, max(1, h // 40)):
        for x in (0, w - 1):
            if relleno[y, x] == 255:
                cv2.floodFill(relleno, mask, (x, y), 0)
    n, lab, st, cen = cv2.connectedComponentsWithStats(relleno, 8)
    out = []
    for i in range(1, n):
        a = st[i, cv2.CC_STAT_AREA]
        if min_pct * h * w <= a <= max_pct * h * w:
            out.append((int(cen[i][0]), int(cen[i][1]), int(a)))
    return out


def insectos_pequenos(bgr, m_hoja, lado_min=3, lado_max=40):
    """Manchas claras y pequeñas sobre la hoja: top-hat sobre la luminosidad."""
    h, w = bgr.shape[:2]
    esc = max(1.0, w / 1000.0)
    gris = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    k = int(round(lado_max * esc)) | 1
    th = cv2.morphologyEx(gris, cv2.MORPH_TOPHAT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    cand = ((th > 28) & (hsv[..., 2] > 120)).astype(np.uint8) * 255
    cerca = cv2.dilate(m_hoja, np.ones((15, 15), np.uint8)) > 0
    cand = cand & (cerca.astype(np.uint8) * 255)
    cand = cv2.morphologyEx(cand, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(cand, 8)
    out = []
    for i in range(1, n):
        a = st[i, cv2.CC_STAT_AREA]
        wi, hi = st[i, cv2.CC_STAT_WIDTH], st[i, cv2.CC_STAT_HEIGHT]
        if lado_min * esc * lado_min * esc <= a <= (lado_max * esc) ** 2 and max(wi, hi) <= lado_max * esc * 1.4:
            out.append((int(cen[i][0]), int(cen[i][1]), int(max(wi, hi) / 2)))
    return out


def lesiones(bgr, m_hoja):
    """Zonas oscuras o pardas DENTRO de la hoja: candidato a tizón/necrosis. Descarta sombras suaves
    exigiendo que sean claramente más oscuras y más pardas que su entorno inmediato."""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    L, A, B = lab[..., 0], lab[..., 1], lab[..., 2]
    local = cv2.GaussianBlur(L, (0, 0), 25)
    pardo = (A > 128 + 2) | (B > 128 + 18)              # menos verde, más amarillo/rojo
    oscuro = (L < local - 22) & (L < 110)
    m = (pardo & oscuro).astype(np.uint8) * 255
    interior = cv2.erode(m_hoja, np.ones((7, 7), np.uint8))
    m = m & interior
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return m


def analizar(bgr, ancho_trabajo=1000):
    h0, w0 = bgr.shape[:2]
    f = ancho_trabajo / w0
    im = cv2.resize(bgr, (ancho_trabajo, int(h0 * f)), interpolation=cv2.INTER_AREA) if f < 1 else bgr
    m = mascara_hoja(im)
    ind = Indicios(mascara_hoja=m)
    area_hoja = max(int((m > 0).sum()), 1)
    ind.cobertura_hoja = area_hoja / m.size
    ind.insectos = [(int(x / f), int(y / f), int(r / f) + 1) for x, y, r in insectos_pequenos(im, m)]
    ag = agujeros(m)
    ind.agujeros = [(int(x / f), int(y / f), int(a / f / f)) for x, y, a in ag]
    ind.area_agujeros_pct = 100 * sum(a for _, _, a in ag) / (area_hoja + sum(a for _, _, a in ag))
    les = lesiones(im, m)
    ind.mascara_lesion = les
    ind.lesiones_pct = 100 * (les > 0).sum() / area_hoja
    return ind, f


def dibujar(bgr, ind: Indicios, f):
    out = bgr.copy()
    ov = out.copy()
    m = cv2.resize(ind.mascara_hoja, (out.shape[1], out.shape[0]), interpolation=cv2.INTER_NEAREST)
    ov[m > 0] = (0.6 * ov[m > 0] + 0.4 * np.array([0, 140, 0])).astype(np.uint8)
    if ind.mascara_lesion is not None:
        l = cv2.resize(ind.mascara_lesion, (out.shape[1], out.shape[0]), interpolation=cv2.INTER_NEAREST)
        ov[l > 0] = (0, 0, 255)
    out = cv2.addWeighted(ov, 0.55, out, 0.45, 0)
    for (x, y, a) in ind.agujeros:
        cv2.circle(out, (x, y), max(8, int(math.sqrt(a / math.pi)) + 6), (255, 0, 255), 3)
    for (x, y, r) in ind.insectos:
        cv2.circle(out, (x, y), max(6, r + 5), (0, 255, 255), 2)
    return out


import math  # noqa: E402
