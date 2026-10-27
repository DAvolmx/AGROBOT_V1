"""
Suelo libre por cámara: ve lo que el LIDAR no ve.
=================================================

El LIDAR barre un plano a ~20 cm del suelo. El 2026-10-02, en el pasillo,
una cobija y una pesa en el suelo no existían para él: el freno marcaba
1-2 m libres con la pesa a 60 cm. Esto lo cubre la cámara.

Método (detección por apariencia, Ulrich y Nourbakhsh, 2000): el suelo justo
delante del robot es, casi siempre, suelo libre — el robot viene de pisarlo
o está a punto. Se toma esa franja como REFERENCIA de cómo se ve el suelo
hoy (color e iluminación cambian), y se marca como obstáculo lo que no se le
parece. Columna a columna, de abajo hacia arriba, el primer tramo que deja
de ser suelo es la base de un obstáculo; con la geometría de la cámara esa
fila se convierte en distancia sobre el suelo. Sale un "scan virtual" con
la misma forma que el del LIDAR: [{angle_deg, distance_m}], 0° = adelante.

Límites que hay que saber:
  · Un cambio de suelo (sendero -> paja, baldosa -> alfombra, una sombra
    dura) también "no se parece": puede dar falsos obstáculos. Por eso se
    exige que el obstáculo aparezca en varios fotogramas seguidos y el
    freno lo usa solo en el camino que barre la carrocería.
  · Si la referencia no es suelo (el robot ya tiene algo pegado delante),
    se detecta por la dispersión de colores y se informa `referencia_dudosa`.
  · Solo ve lo que entra en el campo de la cámara (~±35°, desde ~0,3 m).
"""
from __future__ import annotations

import math
import time

import numpy as np

ANCHO = 160               # resolución de análisis (rápido en la Jetson)
ALTO = 120
# Franja de referencia: abajo en la imagen, centrada (lo que el robot pisará).
REF_FILAS = (0.86, 0.99)  # fracción del alto
REF_COLUMNAS = (0.30, 0.70)
# Histograma de color en HSV (tono x saturación) + brillo aparte.
BINS_H, BINS_S, BINS_V = 18, 8, 8
UMBRAL_SUELO = 0.30       # probabilidad mínima (relativa al máximo) para "es suelo"
RACHA_OBSTACULO = 4       # filas seguidas de no-suelo para declarar obstáculo
DISTANCIA_MAX_M = 2.5
REF_DISPERSION_MAX = 0.45 # fracción de la referencia que no cae en sus 3 bins más llenos
# Memoria del suelo: cuánto de cada referencia nueva entra en el recuerdo, y
# a partir de qué distancia (Bhattacharyya, 0 = idéntico, 1 = nada en común)
# la referencia ya no es suelo sino algo que el robot tiene encima.
MEMORIA_ALFA = 0.1
MEMORIA_EXTRANA = 0.55
# Si con el suelo RECORDADO no coincide casi nada de la imagen, el que está
# mal es el recuerdo (aprendido de un fotograma raro al arrancar, otra luz,
# otro sitio), no la escena: se reaprende. Con un obstáculo real delante el
# resto del suelo sí coincide (la cobija: > 30 %). 2026-10-02: sin esto la
# memoria quedó mal tras reiniciar y bloqueó el freno con el suelo despejado.
MEMORIA_INVALIDA = 0.05
EXTRANA_MIN = 3             # fotogramas seguidos con la franja «rara» antes de declarar hueco 0 (objeto pegado)


def _suelo_por_histograma(cv2, hsv, hist, hist_v):
    """Máscara de "parece suelo" por retroproyección de los histogramas."""
    # Ensancha un bin en cada sentido: el suelo real no es de un solo color.
    hist = cv2.GaussianBlur(hist, (3, 3), 0)
    hist_v = cv2.GaussianBlur(hist_v, (1, 3), 0)
    cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
    cv2.normalize(hist_v, hist_v, 0, 255, cv2.NORM_MINMAX)
    p_hs = cv2.calcBackProject([hsv], [0, 1], hist, [0, 180, 0, 256], 1)
    p_v = cv2.calcBackProject([hsv], [2], hist_v, [0, 256], 1)
    prob = (p_hs.astype(np.float32) / 255.0) * (p_v.astype(np.float32) / 255.0) ** 0.5
    return prob >= UMBRAL_SUELO


class SueloLibre:
    def __init__(self, cfg: dict | None = None, reloj=time.monotonic):
        self.cfg = cfg or {}
        self._reloj = reloj          # el simulador pasa su reloj virtual
        self.ultimo: dict | None = None
        self._historia: list[np.ndarray] = []
        self._extrana_seguidas = 0
        # Cómo se ve el suelo, aprendido de fotogramas anteriores. Sin esto,
        # un objeto que ya está en la franja de referencia (la cobija del
        # paso 1, 2026-10-02) se aprendía como "suelo" y desaparecía.
        self._mem_hs = None
        self._mem_v = None
        # 2026-10-22: «apariencia» = el suelo es lo conectado a la franja de delante que se le parece en color Y textura
        # (sendero_apariencia). Con la cámara inclinada (servo 64) daba el hueco bien donde el histograma decía «0 m» el 47 % del tiempo.
        self.metodo = self.cfg.get("metodo", "histograma")
        self._mem_ap = None
        self.segmentador = None                  # callable frame BGR -> mapa de clases (transitabilidad.Transitabilidad); con metodo="modelo"

    # ------------------------------------------------------------ geometría
    @staticmethod
    def distancia_fila(v: np.ndarray, geo: dict, alto: int, ancho: int):
        """Fila de la imagen -> distancia sobre el suelo delante de la cámara.

        Misma cámara que `sendero.Cenital`: altura h, inclinada phi hacia
        abajo. Inversa de v = cy + f·yc/zc. NaN por encima del horizonte."""
        h = geo["camara_altura_m"]
        phi = math.radians(geo["camara_inclinacion_deg"])
        f = (ancho / 2) / math.tan(math.radians(geo["camara_hfov_deg"]) / 2)
        t = (np.asarray(v, dtype=np.float64) - alto / 2) / f
        den = math.sin(phi) + t * math.cos(phi)
        with np.errstate(divide="ignore", invalid="ignore"):
            dx = h * (math.cos(phi) - t * math.sin(phi)) / den
        return np.where(den > 1e-3, dx, np.nan), f

    def a_suelo(self, u, v, geo, alto=ALTO, ancho=ANCHO):
        """Píxel (u, v) -> (x, y) en el marco del robot (x adelante, y izquierda)."""
        dx, f = self.distancia_fila(v, geo, alto, ancho)
        phi = math.radians(geo["camara_inclinacion_deg"])
        zc = dx * math.cos(phi) + geo["camara_altura_m"] * math.sin(phi)
        y = -(np.asarray(u, dtype=np.float64) - ancho / 2) * zc / f
        return dx + geo["camara_adelante_m"], y

    # ------------------------------------------------------------ detección
    def analizar(self, cv2, frame, geo: dict) -> dict:
        img = cv2.resize(frame, (ANCHO, ALTO), interpolation=cv2.INTER_AREA)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        f0, f1 = int(REF_FILAS[0] * ALTO), int(REF_FILAS[1] * ALTO)
        c0, c1 = int(REF_COLUMNAS[0] * ANCHO), int(REF_COLUMNAS[1] * ANCHO)
        ref = hsv[f0:f1, c0:c1]
        hist = cv2.calcHist([ref], [0, 1], None, [BINS_H, BINS_S], [0, 180, 0, 256])
        hist_v = cv2.calcHist([ref], [2], None, [BINS_V], [0, 256])
        total = float(hist.sum()) or 1.0
        dispersion = 1.0 - float(np.sort(hist.ravel())[-3:].sum()) / total
        hist /= total
        hist_v /= float(hist_v.sum()) or 1.0

        # ¿La referencia se parece al suelo recordado?
        extrana = False
        diferencia = None
        if self._mem_hs is not None:
            diferencia = float(cv2.compareHist(self._mem_hs, hist, cv2.HISTCMP_BHATTACHARYYA))
            extrana = diferencia > MEMORIA_EXTRANA
        ref_hs, ref_v = hist.copy(), hist_v.copy()
        if extrana:
            # Hay algo delante: se mira la imagen con el suelo RECORDADO.
            hist, hist_v = self._mem_hs.copy(), self._mem_v.copy()
        elif self._mem_hs is None:
            self._mem_hs, self._mem_v = hist.copy(), hist_v.copy()
        else:
            self._mem_hs = (1 - MEMORIA_ALFA) * self._mem_hs + MEMORIA_ALFA * hist
            self._mem_v = (1 - MEMORIA_ALFA) * self._mem_v + MEMORIA_ALFA * hist_v

        # Horizonte: filas sin intersección con el suelo no cuentan.
        filas = np.arange(ALTO)
        dist_fila, _ = self.distancia_fila(filas, geo, ALTO, ANCHO)
        validas = np.isfinite(dist_fila) & (dist_fila > 0) & (dist_fila < DISTANCIA_MAX_M)

        suelo = _suelo_por_histograma(cv2, hsv, hist, hist_v)
        if self.metodo == "apariencia":
            import sendero_apariencia
            if self._mem_ap is None:
                self._mem_ap = sendero_apariencia.MemoriaApariencia()
            m_ap, _ = sendero_apariencia.mascara_camino(cv2, cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA), self._mem_ap,
                                                        float(self.cfg.get("umbral", 2.6)), float(self.cfg.get("textura", 1.2)))
            suelo = cv2.resize(m_ap, (ANCHO, ALTO), interpolation=cv2.INTER_NEAREST) > 0
            extrana = False                       # la referencia se reaprende cada fotograma con memoria propia
        if self.metodo == "modelo" and self.segmentador is not None:
            # UNet de transitabilidad: suelo = clase 1 (camino libre); lo demás dentro de las filas válidas es obstáculo (planta, borde bajo, pared, objeto)
            suelo = cv2.resize(self.segmentador(frame), (ANCHO, ALTO), interpolation=cv2.INTER_NEAREST) == 1
            extrana = False
        memoria_reaprendida = False
        if extrana and validas.any() and float(suelo[validas].mean()) < MEMORIA_INVALIDA:
            self._mem_hs, self._mem_v = ref_hs.copy(), ref_v.copy()
            extrana, memoria_reaprendida = False, True
            suelo = _suelo_por_histograma(cv2, hsv, ref_hs, ref_v)

        # Columna a columna, de abajo hacia arriba: primera racha de no-suelo.
        base = np.full(ANCHO, -1, dtype=np.int32)
        no = (~suelo) & validas[:, None]
        racha = np.zeros(ANCHO, dtype=np.int32)
        for fila in range(ALTO - 1, -1, -1):
            if not validas[fila]:
                if fila < ALTO // 2:
                    break
                continue
            nueva = no[fila] & (base < 0)
            racha = np.where(nueva, racha + 1, np.where(base < 0, 0, racha))
            recien = (racha >= RACHA_OBSTACULO) & (base < 0)
            base[recien] = fila + RACHA_OBSTACULO - 1

        # Obstáculo que persiste en varios fotogramas (contra sombras sueltas).
        self._historia.append(base)
        self._historia = self._historia[-3:]
        firme = base.copy()
        for previa in self._historia[:-1]:
            ambos = (firme >= 0) & (previa >= 0)
            firme = np.where(ambos, np.minimum(firme, previa), -1)
        if len(self._historia) < 2:
            firme = np.full_like(base, -1)

        if extrana:
            # El obstáculo empieza en el borde de abajo: lo más cerca visible.
            firme = np.where(firme >= 0, firme, ALTO - 1)
            firme[: int(REF_COLUMNAS[0] * ANCHO)] = -1
            firme[int(REF_COLUMNAS[1] * ANCHO):] = -1

        puntos = []
        cols = np.nonzero(firme >= 0)[0]
        if len(cols):
            x, y = self.a_suelo(cols.astype(np.float64), firme[cols].astype(np.float64), geo)
            for xi, yi in zip(x, y):
                if np.isfinite(xi) and np.isfinite(yi):
                    puntos.append({"angle_deg": math.degrees(math.atan2(yi, xi)),
                                   "distance_m": float(math.hypot(xi, yi))})
        res = {
            "t": self._reloj(),
            "puntos": puntos,
            "referencia_dudosa": dispersion > REF_DISPERSION_MAX and not extrana,
            "objeto_en_referencia": extrana,
            "memoria_reaprendida": memoria_reaprendida,
            "diferencia_con_memoria": None if diferencia is None else round(diferencia, 3),
            "distancia_min_visible_m": float(np.nanmin(np.where(validas, dist_fila, np.nan))
                                             + geo["camara_adelante_m"]) if validas.any() else None,
            "dispersion_ref": round(dispersion, 3),
            "suelo_pct": round(float(suelo[validas].mean()) * 100, 1) if validas.any() else None,
            "mascara": suelo,
            "base": firme,
        }
        # Persistencia de «hay algo en la franja»: en interiores (baldosa oscura, reflejos de ventana) la referencia cambia de un
        # fotograma a otro y daba hueco 0 durante medio segundo sin obstáculo real (2026-10-22: 93 rachas, mediana 0,4 s).
        self._extrana_seguidas = self._extrana_seguidas + 1 if extrana else 0
        res["extrana_seguidas"] = self._extrana_seguidas
        self.ultimo = res
        return res

    # ------------------------------------------------------------ para el freno
    def hueco_delante(self, ancho_robot_m: float, holgura_m: float = 0.05,
                      max_edad_s: float = 0.6) -> float | None:
        """Distancia al primer obstáculo del suelo en el camino del robot (m
        desde el CENTRO del robot), o None si no hay o el dato es viejo."""
        r = self.ultimo
        if r is None or self._reloj() - r["t"] > max_edad_s:
            return None
        if r.get("objeto_en_referencia") and r.get("extrana_seguidas", EXTRANA_MIN) < EXTRANA_MIN:
            return None                         # un destello de un fotograma: sin dato, no «hueco cero»
        if r.get("objeto_en_referencia"):
            # Algo ocupa la franja que el robot va a pisar, y la cámara no ve
            # los primeros ~65 cm: puede estar pegado. Hueco CERO.
            return 0.0
        if r["referencia_dudosa"]:
            return None
        mejor = None
        for p in r["puntos"]:
            a = math.radians(p["angle_deg"])
            lateral = abs(p["distance_m"] * math.sin(a))
            frente = p["distance_m"] * math.cos(a)
            if frente > 0 and lateral <= ancho_robot_m / 2 + holgura_m:
                mejor = frente if mejor is None else min(mejor, frente)
        return mejor

    def dibujar(self, cv2, frame):
        """Imagen de diagnóstico: suelo en verde, base de obstáculos en rojo."""
        r = self.ultimo
        img = cv2.resize(frame, (ANCHO, ALTO), interpolation=cv2.INTER_AREA)
        if r is None:
            return img
        verde = img.copy()
        verde[r["mascara"]] = (0.5 * verde[r["mascara"]] + (0, 127, 0)).astype(np.uint8)
        for c, f in enumerate(r["base"]):
            if f >= 0:
                cv2.circle(verde, (int(c), int(f)), 1, (0, 0, 255), -1)
        c0, c1 = int(REF_COLUMNAS[0] * ANCHO), int(REF_COLUMNAS[1] * ANCHO)
        f0, f1 = int(REF_FILAS[0] * ALTO), int(REF_FILAS[1] * ALTO)
        cv2.rectangle(verde, (c0, f0), (c1, f1), (255, 255, 0), 1)
        return verde
