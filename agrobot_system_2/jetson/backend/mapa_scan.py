"""Mapa dibujado con los PUNTOS DEL SCAN acumulados (en vez de la rejilla de slam_toolbox).

Cada scan, llevado al marco del mapa con la pose de slam_toolbox, se acumula en una rejilla de log-odds
(rayo libre + impacto). Para que la deriva no se acumule en el dibujo:
  · scan-to-map: antes de insertar, el scan se alinea contra lo ya acumulado buscando una pequeña corrección
    (±`BUSQUEDA_M` y ±`BUSQUEDA_DEG`); la corrección persiste y queda acotada (`CORR_MAX_*`).
  · no se inserta mientras el robot gira rápido (el scan sale deformado por el movimiento) ni con pose saltando.
La salida tiene el mismo formato que el /map del puente ROS (data_b64, origin, resolution), recortada a lo explorado.
"""
from __future__ import annotations

import base64
import math

import cv2
import numpy as np

RES_M = 0.05
LADO_M = 40.0                   # rejilla de 40 x 40 m centrada en el origen del mapa
LOG_LIBRE, LOG_OCUPADO, LOG_MAX = -0.35, 0.9, 5.0
UMBRAL_OCUPADO, UMBRAL_LIBRE = 1.0, -0.6
RANGO_MIN_M, RANGO_MAX_M = 0.12, 8.0
OMEGA_MAX_RPS = 0.6             # más rápido que esto, el scan sale torcido: no se inserta
SALTO_MAX_M = 0.5               # una pose que salta más que esto entre scans (cierre de bucle) no se inserta ese cuadro
BUSQUEDA_M, PASO_M = 0.06, 0.03
BUSQUEDA_DEG, PASO_DEG = 2.0, 1.0
CORR_MAX_M, CORR_MAX_DEG = 0.30, 10.0
MIN_IMPACTOS_ALINEAR = 150      # con menos mapa que esto no hay contra qué alinear
MARGEN_RECORTE = 10             # celdas
# Registro scan-a-mapa de dos etapas (2026-10-26): la pose del SLAM a veces salta o gira respecto al mapa ya dibujado (recolocaciones, cierres de bucle,
# reinicios) y el dibujo acumulaba COPIAS giradas de la misma sala («se sobremonta»). Ahora cada scan se registra contra un campo de verosimilitud del mapa
# (distancia al muro más cercano) con búsqueda GRUESA (±40 cm, ±12°) y luego FINA; solo se acepta si mejora claramente lo que daba la corrección actual.
REG_GRUESO_M, REG_GRUESO_PASO_M, REG_GRUESO_DEG, REG_GRUESO_PASO_DEG = 0.40, 0.10, 12.0, 3.0
REG_FINO_M, REG_FINO_PASO_M, REG_FINO_DEG, REG_FINO_PASO_DEG = 0.08, 0.02, 3.0, 0.75
REG_SIGMA_M = 0.06
REG_MEJORA_MIN = 0.06           # mejora mínima de la puntuación (0-1) para aceptar un salto grande
REG_TOPE_M, REG_TOPE_DEG = 1.5, 40.0
MARGEN_LIBRE_CELDAS = 1.0       # el rayo libre se detiene tantas celdas antes del impacto (con el ruido del LIDAR, 1 borraba muros vistos de refilón)
# Ancla de rumbo por paredes (2026-10-24): en interiores los muros son ortogonales, así que el ángulo de los muros módulo 90° es una
# referencia global que la deriva del giroscopio/ICP no tiene. Se fija con los primeros scans nítidos y cada scan nítido empuja
# suavemente el rumbo corregido hacia ella. Solo actúa con muros claros (concentración alta) y errores pequeños: en exteriores o con
# curvas no hay concentración y no hace nada.
MANHATTAN_CONC_MIN = 0.60       # |Σ w e^{i4θ}| / Σ w: 1 = todos los segmentos paralelos o perpendiculares
MANHATTAN_ERR_MAX_DEG = 8.0     # más error que esto = probablemente no es la misma orientación (otra sala girada): no se toca
MANHATTAN_GANANCIA = 0.15
MANHATTAN_SCANS_INICIO = 5      # scans nítidos para fijar el eje


class MapaScan:
    def __init__(self, res: float = RES_M, lado_m: float = LADO_M, manhattan: bool = False, registro: bool = True):
        self.registro = registro
        self.manhattan = manhattan
        self.res = res
        self.n = int(round(lado_m / res))
        self.origen = -lado_m / 2
        self.reiniciar()

    def reiniciar(self):
        self.L = np.zeros((self.n, self.n), dtype=np.float32)
        self.corr = np.zeros(3)                    # dx, dy, dtheta(rad) sobre la pose de slam
        self._eje_z = 0j                           # eje del edificio como número complejo e^{i4θ} (se promedia en 4θ por la simetría de 90°)
        self._eje_n = 0
        self.eje_deg = None                        # eje fijado (grados, módulo 90) o None
        self.anclajes = 0                          # veces que el ancla empujó el rumbo
        self._campo = None                         # campo de verosimilitud (distancia al muro) y la versión del mapa con que se calculó
        self._campo_v = -1
        self.saltos_registro = 0                   # veces que el registro grueso corrigió un salto de la pose
        self._previa = None                        # (t, pose) para la velocidad angular
        self.version = 0
        self.impactos = 0
        self._ymin = self._xmin = self.n
        self._ymax = self._xmax = 0

    # ---- geometría
    @staticmethod
    def _puntos_robot(angulos_deg, dist_m):
        a = np.radians(angulos_deg)
        return dist_m * np.cos(a), dist_m * np.sin(a)

    def _a_mapa(self, lx, ly, x, y, th):
        c, s = math.cos(th), math.sin(th)
        return x + c * lx - s * ly, y + s * lx + c * ly

    def _celdas(self, wx, wy):
        return ((wy - self.origen) / self.res).astype(np.int64), ((wx - self.origen) / self.res).astype(np.int64)

    def _dentro(self, iy, ix):
        return (iy >= 0) & (iy < self.n) & (ix >= 0) & (ix < self.n)

    # ---- ancla de rumbo por paredes
    @staticmethod
    def _angulo_muros(lx, ly):
        """(φ, concentración): orientación dominante de los muros del scan módulo 90° (marco del robot) y qué tan clara es."""
        o = np.argsort(np.arctan2(ly, lx))
        px, py = lx[o], ly[o]
        K = 4                                   # base de 4 puntos: con el ruido del LIDAR (~1-2 cm) los vecinos inmediatos dan direcciones al azar
        dx, dy = px[K:] - px[:-K], py[K:] - py[:-K]
        L = np.hypot(dx, dy)
        ok = (L > 0.12) & (L < 0.60)
        if ok.sum() < 20:
            return None, 0.0
        z = (L[ok] * np.exp(4j * np.arctan2(dy[ok], dx[ok]))).sum()
        return float(np.angle(z) / 4.0), float(abs(z) / L[ok].sum())

    def _anclar(self, lx, ly, th_total):
        """Devuelve la corrección de rumbo (rad) que acerca el rumbo total a la ortogonalidad del eje fijado, o 0."""
        phi, conc = self._angulo_muros(lx, ly)
        if phi is None or conc < MANHATTAN_CONC_MIN:
            return 0.0
        z = np.exp(4j * (phi + th_total))
        if self._eje_n < MANHATTAN_SCANS_INICIO:
            self._eje_z += z
            self._eje_n += 1
            if self._eje_n == MANHATTAN_SCANS_INICIO:
                self.eje_deg = math.degrees(np.angle(self._eje_z) / 4.0) % 90.0
            return 0.0
        err = np.angle(z / np.exp(1j * np.angle(self._eje_z))) / 4.0          # en (-45°, 45°]
        if abs(math.degrees(err)) > MANHATTAN_ERR_MAX_DEG:
            return 0.0
        self.anclajes += 1
        return -MANHATTAN_GANANCIA * err

    # ---- registro scan-a-mapa (campo de verosimilitud)
    def _actualizar_campo(self):
        if self._campo is not None and self.version - self._campo_v < 6:
            return
        occ = (self.L > UMBRAL_OCUPADO)
        dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 3) * self.res            # m al muro más cercano
        self._campo = np.exp(-0.5 * (dist / REG_SIGMA_M) ** 2).astype(np.float32)
        self._campo_v = self.version

    def _puntuar(self, lx, ly, x, y, th, dx, dy, dth) -> float:
        wx, wy = self._a_mapa(lx, ly, x + dx, y + dy, th + dth)
        iy, ix = self._celdas(wx, wy)
        ok = self._dentro(iy, ix)
        if ok.sum() < 20:
            return 0.0
        return float(self._campo[iy[ok], ix[ok]].sum()) / max(1, len(lx))

    def _registrar(self, lx, ly, x, y, th):
        """Mejor corrección (dx, dy, dth) en dos etapas alrededor de la actual. Devuelve la corrección y si fue un salto grande."""
        if self.impactos < MIN_IMPACTOS_ALINEAR:
            return self.corr, False
        self._actualizar_campo()
        c0 = self.corr.copy()
        base = self._puntuar(lx, ly, x, y, th, *c0)
        mejor, mejor_s = c0.copy(), base
        # etapa gruesa alrededor de la corrección actual (solo si la actual puntúa mal: ahorra CPU cuando todo cuadra)
        if base < 0.55:
            n_xy = int(round(REG_GRUESO_M / REG_GRUESO_PASO_M))
            n_th = int(round(REG_GRUESO_DEG / REG_GRUESO_PASO_DEG))
            for k in range(-n_th, n_th + 1):
                dth = c0[2] + math.radians(k * REG_GRUESO_PASO_DEG)
                for i in range(-n_xy, n_xy + 1):
                    for j in range(-n_xy, n_xy + 1):
                        dx, dy = c0[0] + i * REG_GRUESO_PASO_M, c0[1] + j * REG_GRUESO_PASO_M
                        if math.hypot(dx, dy) > REG_TOPE_M or abs(dth) > math.radians(REG_TOPE_DEG):
                            continue
                        sc = self._puntuar(lx, ly, x, y, th, dx, dy, dth)
                        if sc > mejor_s:
                            mejor_s, mejor = sc, np.array([dx, dy, dth])
        salto = False
        if mejor_s > base + REG_MEJORA_MIN:
            salto = bool(np.hypot(*(mejor[:2] - c0[:2])) > 0.12 or abs(mejor[2] - c0[2]) > math.radians(4))
        else:
            mejor, mejor_s = c0.copy(), base
        # etapa fina alrededor de lo mejor
        c1 = mejor.copy()
        n_xy = int(round(REG_FINO_M / REG_FINO_PASO_M))
        n_th = int(round(REG_FINO_DEG / REG_FINO_PASO_DEG))
        for k in range(-n_th, n_th + 1):
            dth = c1[2] + math.radians(k * REG_FINO_PASO_DEG)
            for i in range(-n_xy, n_xy + 1):
                for j in range(-n_xy, n_xy + 1):
                    dx, dy = c1[0] + i * REG_FINO_PASO_M, c1[1] + j * REG_FINO_PASO_M
                    if math.hypot(dx, dy) > REG_TOPE_M or abs(dth) > math.radians(REG_TOPE_DEG):
                        continue
                    sc = self._puntuar(lx, ly, x, y, th, dx, dy, dth) - 0.02 * (abs(i) + abs(j)) / max(n_xy, 1) * 0.05
                    if sc > mejor_s:
                        mejor_s, mejor = sc, np.array([dx, dy, dth])
        return mejor, salto

    # ---- alineación scan-to-map
    def _alinear(self, lx, ly, x, y, th):
        """Mejor (dx, dy, dth) cerca de la corrección actual: el que más puntos del scan pone sobre celdas ya ocupadas."""
        if self.impactos < MIN_IMPACTOS_ALINEAR:
            return self.corr
        ocupado = np.clip(self.L, 0.0, LOG_MAX)
        mejor, mejor_s = self.corr, -1.0
        n_xy = int(round(BUSQUEDA_M / PASO_M))
        n_th = int(round(BUSQUEDA_DEG / PASO_DEG))
        for k in range(-n_th, n_th + 1):
            dth = self.corr[2] + math.radians(k * PASO_DEG)
            if abs(dth) > math.radians(CORR_MAX_DEG):
                continue
            for i in range(-n_xy, n_xy + 1):
                for j in range(-n_xy, n_xy + 1):
                    dx, dy = self.corr[0] + i * PASO_M, self.corr[1] + j * PASO_M
                    if math.hypot(dx, dy) > CORR_MAX_M:
                        continue
                    wx, wy = self._a_mapa(lx, ly, x + dx, y + dy, th + dth)
                    iy, ix = self._celdas(wx, wy)
                    ok = self._dentro(iy, ix)
                    sc = float(ocupado[iy[ok], ix[ok]].sum()) / max(1, len(lx))
                    sc -= 0.15 * (abs(dx - self.corr[0]) + abs(dy - self.corr[1])) / PASO_M * 0.05   # sesgo: quedarse donde está
                    if sc > mejor_s:
                        mejor_s, mejor = sc, np.array([dx, dy, dth])
        return mejor

    # ---- inserción
    def agregar(self, t: float, pose: dict, angulos_deg, dist_m) -> bool:
        """Acumula un scan (marco del robot) con la pose de slam. True si se insertó."""
        x, y, th = pose["x"], pose["y"], math.radians(pose["theta_deg"])
        previa, self._previa = self._previa, (t, x, y, th)
        if previa is not None:
            dt = max(1e-3, t - previa[0])
            dth = math.atan2(math.sin(th - previa[3]), math.cos(th - previa[3]))
            if abs(dth) / dt > OMEGA_MAX_RPS or math.hypot(x - previa[1], y - previa[2]) > SALTO_MAX_M:
                return False
        angulos_deg, dist_m = np.asarray(angulos_deg, float), np.asarray(dist_m, float)
        v = (dist_m >= RANGO_MIN_M) & (dist_m <= RANGO_MAX_M)
        if v.sum() < 30:
            return False
        lx, ly = self._puntos_robot(angulos_deg[v], dist_m[v])
        if self.registro:
            self.corr, salto = self._registrar(lx, ly, x, y, th)
            self.saltos_registro += int(salto)
        else:
            self.corr = self._alinear(lx, ly, x, y, th)
        if self.manhattan:
            d = self._anclar(lx, ly, th + self.corr[2])
            if d:
                self.corr[2] = float(np.clip(self.corr[2] + d, -math.radians(CORR_MAX_DEG), math.radians(CORR_MAX_DEG)))
        cx, cy, cth = x + self.corr[0], y + self.corr[1], th + self.corr[2]
        wx, wy = self._a_mapa(lx, ly, cx, cy, cth)
        # libre: muestras a lo largo de cada rayo hasta justo antes del impacto
        m = int(RANGO_MAX_M / self.res)
        largo = np.hypot(wx - cx, wy - cy)
        ux, uy = (wx - cx) / largo, (wy - cy) / largo
        paso = (np.arange(m) * self.res * 0.9)[None, :]                    # muestras cada ~4,5 cm: ninguna celda del rayo se salta
        fx, fy = cx + ux[:, None] * paso, cy + uy[:, None] * paso
        mascara = paso < (largo[:, None] - MARGEN_LIBRE_CELDAS * self.res)
        iy, ix = self._celdas(fx[mascara], fy[mascara])
        ok = self._dentro(iy, ix)
        libre = np.zeros_like(self.L, dtype=bool)
        libre[iy[ok], ix[ok]] = True                     # una vez por celda y scan
        self.L[libre] = np.maximum(self.L[libre] + LOG_LIBRE, -LOG_MAX)
        iy, ix = self._celdas(wx, wy)
        ok = self._dentro(iy, ix)
        iy, ix = iy[ok], ix[ok]
        golpe = np.zeros_like(self.L, dtype=bool)
        golpe[iy, ix] = True
        self.L[golpe] = np.minimum(self.L[golpe] + LOG_OCUPADO - LOG_LIBRE, LOG_MAX)   # compensa el libre de arriba
        if len(iy):
            self._ymin, self._ymax = min(self._ymin, int(iy.min())), max(self._ymax, int(iy.max()))
            self._xmin, self._xmax = min(self._xmin, int(ix.min())), max(self._xmax, int(ix.max()))
        self.impactos = int((self.L > UMBRAL_OCUPADO).sum())
        self.version += 1
        return True

    # ---- salida
    @staticmethod
    def _adelgazar(b: np.ndarray, max_iter: int = 40) -> np.ndarray:
        """Adelgazamiento de Zhang-Suen (vectorizado): deja los muros en líneas de UNA celda conservando la conectividad. Dos franjas pegadas por la
        deriva (paredes dobles) se funden en una sola línea; dos muros separados por >= 1 celda libre siguen siendo dos."""
        im = np.pad(b.astype(np.uint8), 1)
        for _ in range(max_iter):
            cambio = False
            for paso in (0, 1):
                P = [im[:-2, 1:-1], im[:-2, 2:], im[1:-1, 2:], im[2:, 2:], im[2:, 1:-1], im[2:, :-2], im[1:-1, :-2], im[:-2, :-2]]     # P2..P9 (N, NE, E, SE, S, SW, W, NW)
                c = im[1:-1, 1:-1]
                vecinos = sum(p.astype(np.int16) for p in P)
                trans = sum(((P[k] == 0) & (P[(k + 1) % 8] == 1)).astype(np.int16) for k in range(8))
                if paso == 0:
                    cond = (P[0] * P[2] * P[4] == 0) & (P[2] * P[4] * P[6] == 0)
                else:
                    cond = (P[0] * P[2] * P[6] == 0) & (P[0] * P[4] * P[6] == 0)
                borrar = (c == 1) & (vecinos >= 2) & (vecinos <= 6) & (trans == 1) & cond
                if borrar.any():
                    im[1:-1, 1:-1][borrar] = 0
                    cambio = True
            if not cambio:
                break
        return im[1:-1, 1:-1].astype(bool)

    def _clasificar(self, sub: np.ndarray, limpio: bool) -> np.ndarray:
        """log-odds recortados -> celdas 100 (muro) / 0 (libre) / -1 (desconocido).
        limpio=True: se suaviza la evidencia, se umbraliza, se cierran los huecos de una celda, se ADELGAZA a líneas de una celda (las paredes dobles por deriva
        se funden en una) y se quitan las motas sueltas (< 5 celdas)."""
        out = np.full(sub.shape, -1, dtype=np.int8)
        if not limpio:
            out[sub < UMBRAL_LIBRE] = 0
            out[sub > UMBRAL_OCUPADO] = 100
            return out
        ls = cv2.GaussianBlur(sub.astype(np.float32), (3, 3), 0.8)
        muro = (ls > UMBRAL_OCUPADO).astype(np.uint8)
        muro = cv2.morphologyEx(muro, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        n, lab, st, _ = cv2.connectedComponentsWithStats(muro, connectivity=8)
        for i in range(1, n):
            if st[i, cv2.CC_STAT_AREA] < 5 and max(st[i, cv2.CC_STAT_WIDTH], st[i, cv2.CC_STAT_HEIGHT]) < 5:
                muro[lab == i] = 0
        fino = self._adelgazar(muro.astype(bool))
        libre = (ls < UMBRAL_LIBRE) & (cv2.dilate(muro, np.ones((3, 3), np.uint8)) == 0)
        out[libre] = 0
        out[fino] = 100
        return out

    def como_mapa(self, limpio: bool = False) -> dict | None:
        """Mismo formato que el /map del puente (recortado a lo explorado). None si aún no hay nada."""
        if self._ymax < self._ymin:
            return None
        y0, y1 = max(0, self._ymin - MARGEN_RECORTE), min(self.n, self._ymax + MARGEN_RECORTE + 1)
        x0, x1 = max(0, self._xmin - MARGEN_RECORTE), min(self.n, self._xmax + MARGEN_RECORTE + 1)
        out = self._clasificar(self.L[y0:y1, x0:x1], limpio)
        return {"width": int(x1 - x0), "height": int(y1 - y0), "resolution": self.res,
                "origin_x": self.origen + x0 * self.res, "origin_y": self.origen + y0 * self.res,
                "data_b64": base64.b64encode(out.tobytes()).decode("ascii"),
                "extension_m": max(x1 - x0, y1 - y0) * self.res, "fuente": "scan"}
