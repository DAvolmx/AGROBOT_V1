"""«Sonar» sobre el mapa acumulado (2026-10-26): en vez de mirar solo cuatro conos (adelante, atrás, izquierda, derecha) del scan actual, el robot
traza ECOS en N direcciones (cada 5°) sobre el mapa que lleva construido —lo que vio hace segundos y ya no está en el scan— fusionados con el scan de ahora.
Con el perfil polar saca lo que un cono no ve:
  · HUECOS: arcos libres por donde cabe el robot (el ancho angular mínimo crece al estrechar la profundidad), con su profundidad;
  · EJE DEL PASILLO y su ancho (par de direcciones opuestas más despejadas) y cuánto queda libre a lo largo;
  · SIN SALIDA (callejón): ningún hueco por delante ni por detrás;
  · MEJOR RUMBO: el hueco que más conviene según la meta, la profundidad y lo poco pisado que esté (memoria de visitas), incluida la media vuelta.
Sin ROS. Usa la rejilla de `planificador.Planificador` (log-odds de 5 cm; lo desconocido cuenta como libre) y el scan en el marco del robot (0° = adelante, + = izquierda).
"""
from __future__ import annotations

import math

import numpy as np

UMBRAL_OCUPADO = 0.8


class Sonar:
    def __init__(self, plan, n: int = 72, r_max: float = 4.0, paso_m: float = 0.05, ancho_robot_m: float = 0.40, solo_scan: bool = False):
        self.plan, self.n, self.r_max, self.paso, self.ancho = plan, n, r_max, paso_m, ancho_robot_m
        self.solo_scan = solo_scan                                     # True: ignora el mapa acumulado y usa solo el scan de ahora (para comparar)
        self.rel = np.arange(n) * (2 * math.pi / n) - math.pi          # ángulos relativos -π..π (0 = adelante)
        self._k = np.arange(1, int(r_max / paso_m) + 1) * paso_m

    # ------------------------------------------------------------------ eco
    def perfil(self, x: float, y: float, psi: float, scan: list[dict] | None = None) -> np.ndarray:
        """Distancia libre (m) en cada dirección relativa: primer obstáculo del MAPA, o el del SCAN si es más cercano. Tope r_max."""
        L = self.plan.L
        a = psi + self.rel
        cx = x + np.cos(a)[:, None] * self._k[None, :]
        cy = y + np.sin(a)[:, None] * self._k[None, :]
        ix = ((cx - self.plan.o[0]) / self.plan.res).astype(np.int64)
        iy = ((cy - self.plan.o[1]) / self.plan.res).astype(np.int64)
        ok = (ix >= 0) & (ix < self.plan.n) & (iy >= 0) & (iy < self.plan.n)
        occ = np.zeros(ix.shape, bool)
        occ[ok] = (L[iy[ok], ix[ok]] > UMBRAL_OCUPADO) & (not self.solo_scan)
        primero = np.where(occ.any(1), occ.argmax(1), len(self._k) - 1)
        r = np.where(occ.any(1), self._k[primero], self.r_max)
        if scan:
            ang = np.array([q["angle_deg"] for q in scan]) * math.pi / 180.0
            d = np.array([q["distance_m"] for q in scan])
            v = (d > 0.18) & (d < self.r_max)
            idx = np.round(((ang[v] + math.pi) % (2 * math.pi)) / (2 * math.pi) * self.n).astype(int) % self.n
            rs = np.full(self.n, np.inf)
            np.minimum.at(rs, idx, d[v])
            r = np.minimum(r, rs)
        return r

    # ------------------------------------------------------------------ análisis
    def huecos(self, r: np.ndarray, prof_min: float = 1.0) -> list[dict]:
        """Arcos libres (r >= prof_min) con ancho suficiente para el robot a la profundidad mínima del arco."""
        libre = r >= prof_min
        out, n, i = [], self.n, 0
        if libre.all():
            return [{"centro": 0.0, "ancho": 2 * math.pi, "prof": float(r.max()), "i0": 0, "i1": n - 1}]
        # empieza en un índice NO libre para no partir un arco por la mitad
        start = int(np.argmin(libre))
        idx = [(start + k) % n for k in range(n)]
        arc = []
        for j in idx + [None]:
            if j is not None and libre[j]:
                arc.append(j)
                continue
            if arc:
                prof = float(min(np.min(r[arc]), 2.0))
                ancho_ang = len(arc) * (2 * math.pi / n)
                necesario = 2 * math.atan2(self.ancho / 2 + 0.05, max(prof, 0.3))
                if ancho_ang >= necesario:
                    c = self.rel[arc[len(arc) // 2]] if arc[-1] >= arc[0] else self.rel[arc[(len(arc) // 2)]]
                    out.append({"centro": float(c), "ancho": ancho_ang, "prof": float(np.mean(r[arc])), "i0": arc[0], "i1": arc[-1]})
                arc = []
        return out

    def eje_pasillo(self, r: np.ndarray) -> dict:
        """Par de direcciones opuestas con más espacio libre en el peor de los dos lados = a lo largo del pasillo; el ancho es lo libre perpendicular."""
        n = self.n
        mitad = n // 2
        par = np.minimum(r[:mitad], r[mitad:2 * mitad])
        k = int(np.argmax(par))
        eje = float(self.rel[k])                                         # ángulo relativo del eje (módulo π)
        perp1, perp2 = r[(k + n // 4) % n], r[(k - n // 4) % n]
        return {"eje_rel_rad": eje, "largo_libre_m": float(r[k] + r[(k + mitad) % n]), "ancho_m": float(perp1 + perp2),
                "es_pasillo": bool(perp1 + perp2 < 2.4 and r[k] + r[(k + mitad) % n] > 2.0)}

    def lados(self, r: np.ndarray, desde_deg: float = 60.0, hasta_deg: float = 120.0) -> tuple[float, float]:
        """(libre a la izquierda, libre a la derecha) = mínimo del eco en ventanas de 60°-120° a cada lado. Es la «medición por lados»: no depende del cono frontal."""
        rel = np.degrees(self.rel)
        izq = (rel >= desde_deg) & (rel <= hasta_deg)
        der = (rel <= -desde_deg) & (rel >= -hasta_deg)
        return float(r[izq].min()), float(r[der].min())

    def sin_salida(self, huecos: list[dict]) -> bool:
        return not huecos

    def mejor_rumbo(self, r: np.ndarray, huecos: list[dict], meta_rel: float | None = None, visitas_fn=None, w_meta: float = 1.0,
                    w_prof: float = 0.15, w_visitas: float = 0.5, x: float = 0.0, y: float = 0.0, psi: float = 0.0,
                    lado_w: float = 0.0, r_izq: float = 9.0, r_der: float = 9.0, margen_lado: float = 0.45, max_giro_rad: float = math.pi):
        """Hueco con mejor puntuación: cercanía a la meta, profundidad y poco pisado. Devuelve (ángulo_rel_rad, hueco) o (None, None)."""
        mejor, mejor_p, mejor_c = None, -1e9, 0.0
        for h in huecos:
            c = h["centro"]
            if meta_rel is not None:                                   # si la meta cae dentro del arco libre (con margen para el robot), se apunta a ella, no al centro
                margen = math.atan2(self.ancho / 2 + 0.05, max(min(h["prof"], 2.0), 0.3))
                dm = math.atan2(math.sin(meta_rel - c), math.cos(meta_rel - c))
                if h["ancho"] >= 2 * math.pi - 1e-6 or abs(dm) <= h["ancho"] / 2 - margen:
                    c = meta_rel
            p = w_prof * min(h["prof"], 3.0) + 0.01 * math.degrees(min(h["ancho"], math.pi)) / 5.0
            if meta_rel is not None:
                p -= w_meta * abs(math.atan2(math.sin(c - meta_rel), math.cos(c - meta_rel)))
            if visitas_fn is not None:
                p -= w_visitas * visitas_fn(x + 1.2 * math.cos(psi + c), y + 1.2 * math.sin(psi + c))
            if abs(c) > math.radians(120):                              # dar la vuelta cuesta muchos pulsos
                p -= 0.3
            if lado_w > 0:                                              # no girar hacia la pared que ya está pegada: castiga el lado con poco libre
                if c > 0.1:
                    p -= lado_w * max(0.0, margen_lado - r_izq)
                elif c < -0.1:
                    p -= lado_w * max(0.0, margen_lado - r_der)
            if abs(c) > max_giro_rad:
                c = math.copysign(max_giro_rad, c)
            if p > mejor_p:
                mejor, mejor_p, mejor_c = h, p, c
        return (mejor_c, mejor) if mejor else (None, None)

    def analisis(self, x: float, y: float, psi: float, scan: list[dict] | None = None, prof_min: float = 1.0) -> dict:
        """Todo junto, para el dashboard/telemetría."""
        r = self.perfil(x, y, psi, scan)
        h = self.huecos(r, prof_min)
        return {"r": [round(float(v), 2) for v in r], "huecos": [{"centro_deg": round(math.degrees(g["centro"])), "ancho_deg": round(math.degrees(g["ancho"])),
                                                                 "prof_m": round(g["prof"], 2)} for g in h],
                "pasillo": self.eje_pasillo(r), "sin_salida": self.sin_salida(h), "libre_360_m": round(float(r.min()), 2)}
