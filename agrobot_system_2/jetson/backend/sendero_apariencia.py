"""Sendero por APARIENCIA (color + textura) respecto a lo que el robot tiene delante (2026-10-22).

Por qué: en la huerta del operador (videos del 2026-10-22) el sendero y las camas son la MISMA tierra; lo que cambia es la textura
(el camino está pisado y liso; las camas tienen terrones, plantas, paja). El detector por contraste de color de `sendero.detectar` veía
«un solo borde» en ~55 % de los fotogramas, y en interiores marcaba «bordes por todas partes» con los reflejos de la baldosa.

Método (detección por apariencia, Ulrich y Nourbakhsh 2000, con textura añadida):
  1. Rasgos por píxel: a y b de Lab (color sin brillo), L atenuado y desviación local de L (textura, ventana 9x9).
  2. Referencia = franja de abajo al centro (lo que el robot va a pisar): mediana y dispersión robusta (MAD), con memoria entre fotogramas.
  3. Sendero = píxeles a menos de UMBRAL sigmas de la referencia, conectados a esa franja.
  4. Se proyecta al suelo (vista cenital) y, a cada distancia de 0,4 a 1,4 m, se busca el tramo libre que contiene el centro anterior.
     Una recta por los centros da el desvío lateral y el rumbo del sendero; la mediana de los anchos, el ancho.
Devuelve el mismo diccionario que `sendero.detectar`, así `VisionSendero` lo usa igual (autonomia.sendero_metodo = "apariencia").
"""
from __future__ import annotations

import math

import numpy as np

import sendero as S

UMBRAL = 2.6              # sigmas (rasgos normalizados) para «se parece al camino»
REF_F, REF_C = (0.80, 0.98), (0.36, 0.64)
MEMORIA = 0.7
PESOS = np.array([1.0, 1.0, 0.35, 1.2], dtype=np.float32)   # a, b, L, textura
X_MIN, X_MAX = 0.40, 1.40
ANCHO_MIN, ANCHO_MAX = 0.22, 1.60


class MemoriaApariencia:
    def __init__(self):
        self.med = None
        self.mad = None

    def actualizar(self, ref):
        med = np.median(ref, axis=0)
        mad = np.maximum(np.median(np.abs(ref - med), axis=0) * 1.4826, np.array([2.0, 2.0, 6.0, 1.5], np.float32))
        if self.med is None:
            self.med, self.mad = med, mad
        else:
            self.med = MEMORIA * self.med + (1 - MEMORIA) * med
            self.mad = MEMORIA * self.mad + (1 - MEMORIA) * mad


def rasgos(cv2, img):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB).astype(np.float32)
    L = lab[..., 0]
    m = cv2.boxFilter(L, -1, (9, 9))
    m2 = cv2.boxFilter(L * L, -1, (9, 9))
    tex = np.sqrt(np.maximum(m2 - m * m, 0))
    return np.dstack([lab[..., 1], lab[..., 2], L, tex])


def mascara_camino(cv2, img, memoria: MemoriaApariencia, umbral: float = UMBRAL, peso_textura: float = 1.2):
    """Umbral/peso por entorno (barrido sobre los videos del operador, 2026-10-22): interior 2,6 / 1,2 marca ~65 % de la imagen como
    suelo (la baldosa, sin paredes ni canasto); huerta 1,4 / 2,5 marca ~30–43 % (el camino pisado, sin las camas de la misma tierra)."""
    h, w = img.shape[:2]
    F = rasgos(cv2, img)
    ref = F[int(h * REF_F[0]):int(h * REF_F[1]), int(w * REF_C[0]):int(w * REF_C[1])].reshape(-1, 4)
    memoria.actualizar(ref)
    pesos = PESOS.copy()
    pesos[3] = peso_textura
    d = (F - memoria.med) / memoria.mad * pesos
    dist = np.sqrt((d * d).sum(axis=2) / (pesos * pesos).sum())
    m = (dist < umbral).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    semilla = lab[int(h * 0.92), int(w * 0.40):int(w * 0.60)]
    ids = [i for i in np.unique(semilla) if i != 0]
    if not ids:
        return np.zeros_like(m), dist
    j = max(ids, key=lambda i: st[i, cv2.CC_STAT_AREA])
    return ((lab == j) * 255).astype(np.uint8), dist


def detectar_apariencia(cv2, frame, cfg, geo, cenital, memoria: MemoriaApariencia, centro_previo: float = 0.0) -> dict:
    chico = cv2.resize(frame, (S.ANCHO_ANALISIS, S.ALTO_ANALISIS), interpolation=cv2.INTER_AREA)
    if cfg.get("sendero_igualar_luz", True):
        chico = S.igualar_luz(cv2, chico)
    camino, _ = mascara_camino(cv2, chico, memoria, float(cfg.get("sendero_apariencia_umbral", UMBRAL)),
                               float(cfg.get("sendero_apariencia_textura", 1.2)))
    res = {"lateral_m": None, "angulo_deg": None, "error_m": None, "ancho_m": None, "bordes_m": (None, None), "confianza": 0.0,
           "estado": "", "mascara": camino, "metodo": "apariencia"}
    if camino.mean() < 2:
        res["estado"] = "sin camino delante (la franja de referencia no se parece a nada conectado)"
        return res
    bev = cenital.proyectar(cv2, camino) > 0                          # filas: lejos arriba; columnas: +Y (izquierda) primero
    X, Y = cenital.X[:, 0], cenital.Y[0, :]
    x_vis = float(cenital.Xv.min()) if len(cenital.Xv) else X_MIN          # lo más cerca que la cámara ve el suelo (≈0,86 m con 0,4 m/15,5°)
    x0, x1 = max(X_MIN, x_vis + 0.02), max(X_MIN, x_vis + 0.02) + (X_MAX - X_MIN)
    filas = [i for i, x in enumerate(X) if x0 <= x <= x1]
    xs, cs, anchos = [], [], []
    centro = centro_previo
    for i in sorted(filas, key=lambda i: X[i]):                      # de cerca a lejos: el centro se arrastra
        libre = bev[i]
        if libre.sum() < 5:
            continue
        k = int(np.argmin(np.abs(Y - centro)))
        if not libre[k]:
            cand = np.nonzero(libre)[0]
            k = int(cand[np.argmin(np.abs(cand - k))])
            if abs(Y[k] - centro) > 0.25:
                continue
        a = k
        while a > 0 and libre[a - 1]:
            a -= 1
        z = k
        while z < len(libre) - 1 and libre[z + 1]:
            z += 1
        izq, der = float(Y[a]), float(Y[z])                           # izq > der (Y positivo a la izquierda)
        ancho = izq - der
        if not (ANCHO_MIN <= ancho <= ANCHO_MAX):
            continue
        xs.append(float(X[i])); cs.append((izq + der) / 2); anchos.append(ancho)
        centro = cs[-1]
    if len(xs) < 8:
        res["estado"] = f"camino visto en pocas distancias ({len(xs)})"
        return res
    pend, orig = np.polyfit(xs, cs, 1)
    resid = float(np.std(np.array(cs) - (pend * np.array(xs) + orig)))
    lateral = float(orig + pend * min(xs))                            # centro del sendero a la distancia más cercana medida
    ang = math.degrees(math.atan(pend))
    ancho = float(np.median(anchos))
    if abs(lateral) > float(cfg.get("sendero_max_lateral_cm", 60)) / 100.0:
        res["estado"] = "camino demasiado lateral; rechazo"
        return res
    cobertura = len(xs) / max(1, len(filas))
    res.update(lateral_m=lateral, angulo_deg=ang, error_m=lateral + S.MIRADA_M * math.sin(math.atan(pend)), ancho_m=ancho,
               bordes_m=(lateral + ancho / 2, lateral - ancho / 2),
               confianza=float(np.clip(cobertura * (1.0 - resid / 0.15), 0.0, 1.0)),
               estado=f"camino a {lateral*100:+.0f} cm, rumbo {ang:+.0f}°, ancho {ancho*100:.0f} cm")
    return res
