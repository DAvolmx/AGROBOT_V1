"""Planificador de ruta sobre el mapa que el robot va construyendo (2026-10-25): A* con inflado por el radio del robot, lo desconocido cuenta como
transitable (optimista; se replanifica al ver paredes). Sirve para ir a la meta en laberintos y puertas, volver al punto de inicio y acercarse a una planta.

· `actualizar(lx, ly, psi, scan)`: acumula el scan del LIDAR (marco del LIDAR, 0° = adelante, + = izquierda) en una rejilla log-odds de 5 cm.
· `ruta(origen, destino, tol_m)`: lista de puntos (x, y) en metros, o None si no hay camino. Planifica a 10 cm y suaviza.
· `siguiente(ruta, pos, mirada_m)`: punto de la ruta a `mirada_m` por delante (persecución pura).
Sin ROS: el mismo código lo usan el simulador y el robot. La pose viene del SLAM.
"""
from __future__ import annotations

import heapq
import math

import cv2
import numpy as np

LOG_LIBRE, LOG_OCUPADO, LOG_MAX = -0.4, 0.9, 4.0
UMBRAL_OCUPADO = 0.8


class Planificador:
    def __init__(self, res: float = 0.05, lado_m: float = 36.0, inflar_m: float = 0.27, res_plan: float = 0.10, rango_m: float = 6.0,
                 centro: tuple[float, float] = (0.0, 0.0)):
        self.res, self.inflar, self.res_plan, self.rango = res, inflar_m, res_plan, rango_m
        self.n = int(round(lado_m / res))
        self.o = (centro[0] - lado_m / 2, centro[1] - lado_m / 2)
        self.L = np.zeros((self.n, self.n), np.float32)
        self.f = int(round(res_plan / res))
        self.holgura = 0.6                      # m: más cerca que esto de una pared cuesta extra
        self.peso_holgura = 3.0
        self._cache = None
        self.version = 0

    # ---------------------------------------------------------------- mapa
    def _celda(self, x, y):
        return int((y - self.o[1]) / self.res), int((x - self.o[0]) / self.res)

    def actualizar(self, lx: float, ly: float, psi: float, scan: list[dict]):
        if not scan:
            return
        a = np.radians([q["angle_deg"] for q in scan]) + psi
        d = np.array([q["distance_m"] for q in scan], float)
        v = (d > 0.12) & (d <= self.rango)
        if v.sum() < 20:
            return
        a, d = a[v], d[v]
        hx, hy = lx + d * np.cos(a), ly + d * np.sin(a)
        m = int(self.rango / (self.res * 0.9))
        paso = (np.arange(m) * self.res * 0.9)[None, :]
        fx, fy = lx + np.cos(a)[:, None] * paso, ly + np.sin(a)[:, None] * paso
        mask = paso < (d[:, None] - self.res * 1.5)
        iy = ((fy[mask] - self.o[1]) / self.res).astype(np.int64)
        ix = ((fx[mask] - self.o[0]) / self.res).astype(np.int64)
        ok = (iy >= 0) & (iy < self.n) & (ix >= 0) & (ix < self.n)
        libre = np.zeros(self.L.shape, bool)
        libre[iy[ok], ix[ok]] = True
        self.L[libre] = np.maximum(self.L[libre] + LOG_LIBRE, -LOG_MAX)
        iy = ((hy - self.o[1]) / self.res).astype(np.int64)
        ix = ((hx - self.o[0]) / self.res).astype(np.int64)
        ok = (iy >= 0) & (iy < self.n) & (ix >= 0) & (ix < self.n)
        golpe = np.zeros(self.L.shape, bool)
        golpe[iy[ok], ix[ok]] = True
        self.L[golpe] = np.minimum(self.L[golpe] + LOG_OCUPADO - LOG_LIBRE, LOG_MAX)
        self.version += 1
        self._cache = None

    def _bloqueado(self):
        """Máscara (a res_plan) de celdas no transitables: ocupado e inflado por el radio del robot."""
        if self._cache is not None and self._cache[0] == self.version:
            return self._cache[1]
        occ = (self.L > UMBRAL_OCUPADO).astype(np.uint8)
        k = int(round(self.inflar / self.res))
        occ = cv2.dilate(occ, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1)))
        h = (self.n // self.f) * self.f
        blq = occ[:h, :h].reshape(h // self.f, self.f, h // self.f, self.f).max(axis=(1, 3)).astype(bool)
        self._cache = (self.version, blq)
        return blq

    # ---------------------------------------------------------------- ruta
    def ruta(self, origen, destino, tol_m: float = 0.3, max_nodos: int = 60000):
        blq = self._bloqueado()
        g = self.res_plan

        def a_celda(p):
            return int((p[1] - self.o[1]) / g), int((p[0] - self.o[0]) / g)

        def a_mundo(c):
            return self.o[0] + (c[1] + 0.5) * g, self.o[1] + (c[0] + 0.5) * g
        s, t = a_celda(origen), a_celda(destino)
        H, W = blq.shape
        if not (0 <= s[0] < H and 0 <= s[1] < W and 0 <= t[0] < H and 0 <= t[1] < W):
            return None
        blq = blq.copy()
        r = int(math.ceil(self.inflar / g)) + 1                      # el robot está donde está: su entorno inmediato no cuenta como pared
        blq[max(0, s[0] - r):s[0] + r + 1, max(0, s[1] - r):s[1] + r + 1] = False
        tol = int(math.ceil(tol_m / g))
        # costo extra por rozar paredes: el camino prefiere el centro del pasillo (no se pega a las esquinas interiores, donde luego no cabe girar)
        dist = cv2.distanceTransform((~blq).astype(np.uint8), cv2.DIST_L2, 3) * g
        pen = np.clip((self.holgura - dist) / self.holgura, 0.0, 1.0) * self.peso_holgura
        s8 = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414))
        costo = {s: 0.0}
        padre = {}
        cola = [(0.0, s)]
        cerrado = set()
        mejor, mejor_h = None, 1e9
        n = 0
        while cola and n < max_nodos:
            _, c = heapq.heappop(cola)
            if c in cerrado:
                continue
            cerrado.add(c)
            n += 1
            h = math.hypot(c[0] - t[0], c[1] - t[1])
            if h < mejor_h:
                mejor, mejor_h = c, h
            if h <= tol:
                mejor = c
                break
            for dy, dx, w in s8:
                nc = (c[0] + dy, c[1] + dx)
                if not (0 <= nc[0] < H and 0 <= nc[1] < W) or blq[nc] or nc in cerrado:
                    continue
                nuevo = costo[c] + w * (1.0 + pen[nc])
                if nuevo < costo.get(nc, 1e18):
                    costo[nc] = nuevo
                    padre[nc] = c
                    heapq.heappush(cola, (nuevo + math.hypot(nc[0] - t[0], nc[1] - t[1]), nc))
        if mejor is None or mejor == s or mejor_h > tol + 2:
            return None if mejor_h > tol + 2 else [origen]
        cam = [mejor]
        while cam[-1] in padre:
            cam.append(padre[cam[-1]])
        cam.reverse()
        pts = [a_mundo(c) for c in cam]
        return self._suavizar(pts)

    @staticmethod
    def _suavizar(pts, cada=3):
        if len(pts) <= 2:
            return pts
        out = pts[::cada]
        if out[-1] != pts[-1]:
            out.append(pts[-1])
        return out

    @staticmethod
    def siguiente(ruta, pos, mirada_m: float = 0.7):
        """Punto de `ruta` a `mirada_m` por delante del punto de la ruta más cercano a `pos` (persecución pura)."""
        if not ruta:
            return None
        i = min(range(len(ruta)), key=lambda k: math.hypot(ruta[k][0] - pos[0], ruta[k][1] - pos[1]))
        acum = 0.0
        for k in range(i, len(ruta) - 1):
            acum += math.hypot(ruta[k + 1][0] - ruta[k][0], ruta[k + 1][1] - ruta[k][1])
            if acum >= mirada_m:
                return ruta[k + 1]
        return ruta[-1]
