"""Cámara como LIDAR frontal (2026-10-27).

suelo_libre ya convierte la imagen en puntos de obstáculo sobre el suelo ([{angle_deg, distance_m}], marco del robot, 0° = adelante, + = izquierda). Este filtro
los limpia y los deja con la forma de un scan, para fusionarlos con el del LIDAR y que el explorador (E11, sonar, predictor) vea también lo que el plano del
LIDAR no ve: cojines, cajas y macetas bajas (el LIDAR está a 37,5 cm).

Filtro: (1) solo datos frescos y fiables (sin referencia dudosa), dentro del campo de la cámara y de DISTANCIA_MAX_M; (2) se agrupa en sectores de
SECTOR_DEG tomando el punto más cercano; (3) un sector solo cuenta si lo acompaña un vecino (≥ MIN_VECINOS), contra columnas sueltas por sombras; (4) objeto pegado
a la franja de referencia → pared virtual a la cara delantera; (5) persistencia: un sector que desaparece se conserva PERSISTE_S (parpadeo del detector).
Fusión: el LIDAR manda; la cámara solo añade puntos más cercanos que el rayo del LIDAR en ese sector (o puntos nuevos). Con solo_camara=True el frente
(±FOV/2) se reemplaza por el de la cámara."""
from __future__ import annotations

import math
import time

SECTOR_DEG = 3.0
MIN_VECINOS = 1
FOV_DEG = 55.0                  # campo HORIZONTAL medido (los 71° del config eran la diagonal)
DISTANCIA_MAX_M = 2.0
DISTANCIA_MIN_M = 0.5            # zona ciega del suelo visible (cámara a 24 cm, 1,9° abajo): lo de más cerca no se ve
EDAD_MAX_S = 0.6
PERSISTE_S = 0.5
EXTRANA_MIN = 3


class CamaraComoLidar:
    def __init__(self, reloj=time.monotonic):
        self._reloj = reloj
        self._memoria: dict[int, tuple[float, float]] = {}      # sector -> (distancia, instante)

    def scan_camara(self, ultimo: dict | None, largo_robot_m: float = 0.0) -> list[dict]:
        """Scan virtual filtrado a partir de `suelo_libre.ultimo`; [] si no hay dato fiable."""
        ahora = self._reloj()
        mitad = FOV_DEG / 2
        nuevos: dict[int, float] = {}
        fresco = ultimo is not None and ahora - ultimo["t"] <= EDAD_MAX_S and not ultimo.get("referencia_dudosa")
        if fresco:
            for p in ultimo.get("puntos", []):
                a = ((p["angle_deg"] + 180.0) % 360.0) - 180.0
                d = p["distance_m"]
                if abs(a) <= mitad and DISTANCIA_MIN_M <= d <= DISTANCIA_MAX_M:
                    k = int(round(a / SECTOR_DEG))
                    nuevos[k] = min(d, nuevos.get(k, d))
            nuevos = {k: d for k, d in nuevos.items() if sum((k + j) in nuevos for j in range(-MIN_VECINOS, MIN_VECINOS + 1) if j) >= 1}
            if ultimo.get("objeto_en_referencia") and ultimo.get("extrana_seguidas", 0) >= EXTRANA_MIN:
                for a in (-10.0, 0.0, 10.0):
                    k = int(round(a / SECTOR_DEG))
                    nuevos[k] = min(largo_robot_m / 2 + 0.05, nuevos.get(k, 9.0))
        for k, d in nuevos.items():
            self._memoria[k] = (d, ahora)
        self._memoria = {k: v for k, v in self._memoria.items() if ahora - v[1] <= PERSISTE_S}
        return [{"angle_deg": k * SECTOR_DEG, "distance_m": d} for k, (d, _) in sorted(self._memoria.items())]

    def fusionar(self, scan_lidar: list[dict] | None, ultimo: dict | None, largo_robot_m: float = 0.0, solo_camara: bool = False) -> list[dict]:
        """Scan del LIDAR + cámara. Sin dato de la cámara devuelve el del LIDAR tal cual."""
        base = list(scan_lidar or [])
        cam = self.scan_camara(ultimo, largo_robot_m)
        if not cam:
            return base
        mitad = FOV_DEG / 2

        def en_frente(a):
            return abs(((a + 180.0) % 360.0) - 180.0) <= mitad

        def norm(c):                                   # el scan del LIDAR va en 0..360°
            return {"angle_deg": c["angle_deg"] % 360.0, "distance_m": c["distance_m"]}

        if solo_camara:
            return [p for p in base if not en_frente(p["angle_deg"])] + [norm(c) for c in cam]
        # rayo del LIDAR más cercano a cada sector de la cámara (±SECTOR_DEG/2)
        salida = list(base)
        for c in cam:
            vecinos = [p["distance_m"] for p in base
                       if abs(((p["angle_deg"] - c["angle_deg"] + 180.0) % 360.0) - 180.0) <= SECTOR_DEG / 2 and p["distance_m"] > 0]
            if not vecinos or c["distance_m"] < min(vecinos) - 0.03:
                salida.append(norm(c))
        return salida
