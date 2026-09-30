"""
Costmap local a partir del escaneo LIDAR 2D.
=============================================

Construye una rejilla de ocupación centrada en el robot que se actualiza
con cada scan del RPLIDAR C1. El LIDAR va a 40 cm del suelo: todo lo que
refleja es algo con altura (paredes, plantas altas, macetas).

Usos:
  1. Alimentar la detección de senderos: el perfil lateral del costmap
     muestra dónde hay "materia" a cada lado del robot.
  2. Evasión de obstáculos mejorada: en vez de solo 4 sectores (adelante/
     atrás/izq/der), se puede consultar cualquier dirección.
  3. Exportar como OccupancyGrid de ROS2 para nav2.

La rejilla se mantiene en ejes del robot (se rota con la pose), no del
mundo, para que funcione con o sin SLAM. Los puntos viejos decaen
exponencialmente: si el robot se mueve, los obstáculos que ya no están en
el scan desaparecen del costmap.
"""

import math
import time
import numpy as np


GRID_SIZE_M = 6.0
GRID_RES_M = 0.05
GRID_CELLS = int(GRID_SIZE_M / GRID_RES_M)
HALF = GRID_CELLS // 2
DECAY = 0.92
HIT_VALUE = 0.7
MISS_VALUE = 0.05
FREE_THRESHOLD = 0.3
OCCUPIED_THRESHOLD = 0.55
MIN_RANGE_M = 0.08
MAX_RANGE_M = 5.0


class CostmapLocal:
    """Rejilla de ocupación centrada en el robot, actualizada con cada scan."""

    def __init__(self, res_m=GRID_RES_M, size_m=GRID_SIZE_M):
        self.res = res_m
        self.size = size_m
        self.cells = int(size_m / res_m)
        self.half = self.cells // 2
        self.grid = np.full((self.cells, self.cells), 0.5, dtype=np.float32)
        self.last_update = None
        self._scan_count = 0

    def update(self, scan_points: list[dict], robot_largo_cm=40, robot_ancho_cm=35):
        """Ingesta de un scan. Cada punto: {angle_deg, distance_m}."""
        ml = robot_largo_cm / 200.0
        ma = robot_ancho_cm / 200.0

        self.grid *= DECAY

        for p in scan_points:
            d = p.get("distance_m", 0)
            if not (MIN_RANGE_M < d < MAX_RANGE_M):
                continue
            a_rad = math.radians(p.get("angle_deg", 0))
            x = d * math.cos(a_rad)
            y = d * math.sin(a_rad)
            if abs(x) < ml and abs(y) < ma:
                continue
            ci = self.half + int(x / self.res)
            cj = self.half + int(y / self.res)
            if 0 <= ci < self.cells and 0 <= cj < self.cells:
                self.grid[ci, cj] = min(1.0, self.grid[ci, cj] + HIT_VALUE)

        self.last_update = time.monotonic()
        self._scan_count += 1

    def perfil_lateral(self, x_min=0.15, x_max=2.0):
        """Densidad de obstáculos a cada distancia lateral, mirando adelante.

        Devuelve (centros_y_m, densidad) — el perfil que muestra dónde hay
        "materia" a izquierda y derecha. Sirve para detectar bordes del sendero.
        """
        ci_min = self.half + int(x_min / self.res)
        ci_max = min(self.cells, self.half + int(x_max / self.res))
        if ci_max <= ci_min:
            return np.array([]), np.array([])

        franja = self.grid[ci_min:ci_max, :]
        perfil = franja.mean(axis=0)
        centros = (np.arange(self.cells) - self.half) * self.res
        return centros, perfil

    def hueco_libre(self, x_min=0.15, x_max=1.5):
        """Ancho y centro del hueco libre que contiene al robot (y=0).

        Si hay paredes a ambos lados, el hueco es el espacio entre ellas.
        Devuelve (centro_m, ancho_m) o (None, None) si no hay hueco claro.
        """
        centros, perfil = self.perfil_lateral(x_min, x_max)
        if len(perfil) == 0:
            return None, None

        libre = perfil < FREE_THRESHOLD
        if not libre.any():
            return None, None

        runs = []
        start = None
        for i, v in enumerate(libre):
            if v and start is None:
                start = i
            elif not v and start is not None:
                runs.append((start, i - 1))
                start = None
        if start is not None:
            runs.append((start, len(libre) - 1))

        if not runs:
            return None, None

        idx_zero = int(np.clip(np.searchsorted(centros, 0.0), 0, len(centros) - 1))
        best = None
        for r in runs:
            if r[0] <= idx_zero <= r[1]:
                best = r
                break
        if best is None:
            best = min(runs, key=lambda r: min(abs(r[0] - idx_zero), abs(r[1] - idx_zero)))

        a, b = best
        margen = max(3, int(0.10 / self.res))
        ocupado_izq = any(perfil[max(0, a - margen):a] >= OCCUPIED_THRESHOLD)
        ocupado_der = any(perfil[b + 1:min(len(perfil), b + 1 + margen)] >= OCCUPIED_THRESHOLD)
        if not ocupado_izq and not ocupado_der:
            return None, None

        centro = (centros[a] + centros[b]) / 2
        ancho = centros[b] - centros[a]
        return float(centro), float(ancho)

    def distancia_sector(self, angulo_deg, semi_deg=15, rango_max=3.0):
        """Distancia al obstáculo más cercano en un sector angular."""
        a_rad = math.radians(angulo_deg)
        semi_rad = math.radians(semi_deg)
        mejor = rango_max
        for ci in range(self.cells):
            for cj in range(self.cells):
                if self.grid[ci, cj] < OCCUPIED_THRESHOLD:
                    continue
                x = (ci - self.half) * self.res
                y = (cj - self.half) * self.res
                d = math.sqrt(x * x + y * y)
                if d < MIN_RANGE_M or d > rango_max:
                    continue
                a = math.atan2(y, x)
                diff = abs((a - a_rad + math.pi) % (2 * math.pi) - math.pi)
                if diff <= semi_rad:
                    mejor = min(mejor, d)
        return mejor if mejor < rango_max else None

    def as_occupancy_grid(self):
        """Devuelve la rejilla como array uint8 (0=libre, 100=ocupado, 50=desconocido)."""
        result = np.full((self.cells, self.cells), 50, dtype=np.int8)
        result[self.grid < FREE_THRESHOLD] = 0
        result[self.grid > OCCUPIED_THRESHOLD] = 100
        return result

    def resumen(self) -> dict:
        ocupadas = int((self.grid > OCCUPIED_THRESHOLD).sum())
        libres = int((self.grid < FREE_THRESHOLD).sum())
        return {
            "celdas_ocupadas": ocupadas,
            "celdas_libres": libres,
            "scans": self._scan_count,
            "res_m": self.res,
            "size_m": self.size,
        }
