"""Capa de MISIÓN del simulador (2026-10-24): inspeccionar, escanear y CONTAR plantas mientras el robot conduce.

Modelo (hipótesis declaradas; sin calibrar con el robot real):
  · La cámara mira al frente con `fov_deg`; solo ve plantas a menos de `rango_m` dentro de ese cono.
  · Detección pasiva (en marcha, cada tick de 0,25 s): p = P_PASIVA·(1 - d/rango)·(1 - 0,6·min(v/0,25, 1)); el movimiento la emborrona.
  · Escaneo (robot PARADO `escaneo_s` s, planta a <= `dist_insp_m`): p = 0,92·min(1, escaneo_s/6)·(1 si d <= 0,7 m, 0,7 si más lejos).
    El escaneo se dispara una vez por planta (y no si ya hay otro escaneo a < 0,3 m).
  · Cada detección se anota en la posición verdadera + ruido (6 cm + deriva ~1 % del recorrido). CONTAR = agrupar detecciones a < `dedup_m`
    (0,30 m): si la pose deriva más que eso, la misma planta cuenta dos veces; si dos plantas están más juntas que eso, cuentan una.
  · Falsos positivos: 3 % por escaneo, 0,2 % por tick pasivo con un objeto cualquiera en el cono (patas, macetas sin planta).
Resumen: plantas reales, detectadas (grupos), recall, precisión, error de conteo, escaneos, tiempo parado.
"""
from __future__ import annotations

import math

P_PASIVA = 0.18
DEDUP_M = 0.30


class MisionSim:
    def __init__(self, esc, cfg, rng):
        self.plantas = [(float(x), float(y)) for x, y in (esc.plantas or [])]
        self.cfg, self.rng = cfg, rng
        self.escaneada = [False] * len(self.plantas)
        self.det: list[tuple[float, float, str]] = []       # (x, y, 'escaneo'|'pasivo'|'fp')
        self.ver_gt = [False] * len(self.plantas)
        self.escaneos = 0
        self.escaneos_fallidos = 0
        self.t_parado = 0.0
        self.intentos = [0] * len(self.plantas)
        self.pend: dict[int, list] = {}                     # plantas vistas pero aún sin escanear: i -> [x_est, y_est, t_primera_vez]
        self.abandonadas: set[int] = set()
        self.off = [0.0, 0.0]                                 # deriva acumulada de la pose (m)
        self.recorrido = 0.0
        self._prev = None
        self.obstaculos = [(x, y) for x, y, r in esc.macetas if all(math.hypot(x - px, y - py) > 0.12 for px, py in self.plantas)]

    def _en_cono(self, cx, cy, psi, px, py, rango, fov_deg=None):
        dx, dy = px - cx, py - cy
        d = math.hypot(dx, dy)
        if d > rango or d < 0.05:
            return None
        b = math.atan2(math.sin(math.atan2(dy, dx) - psi), math.cos(math.atan2(dy, dx) - psi))
        return d if abs(b) <= math.radians(self.cfg.mis_fov_deg if fov_deg is None else fov_deg) / 2 else None

    def _anotar(self, i, tipo, t=0.0, d=1.0):
        # Error de la posición anotada = deriva ACUMULADA de la pose (un desplazamiento común a todo lo visto en el mismo momento: dos vistas de la
        # misma planta seguidas coinciden) + ruido por detección que crece con la distancia (3 cm + 4 cm por metro). Antes la deriva se sumaba como
        # ruido independiente por detección y fabricaba duplicados (error de modelado corregido el 2026-10-24).
        sig = 0.03 + 0.04 * d
        x, y = self.plantas[i]
        ex, ey = x + self.off[0] + float(self.rng.normal(0, sig)), y + self.off[1] + float(self.rng.normal(0, sig))
        self.det.append((ex, ey, tipo))
        self.ver_gt[i] = True
        if tipo == "pasivo" and not self.escaneada[i] and i not in self.abandonadas:
            self.pend.setdefault(i, [ex, ey, t])                  # queda pendiente de visita y escaneo
        elif tipo == "escaneo":
            self.pend.pop(i, None)

    def objetivo(self, t, cx, cy, alcance_m, max_s=45.0):
        """Planta pendiente más cercana (x, y estimados) a <= alcance_m, o None. Una planta que no se logra escanear en `max_s` s se abandona."""
        mejor, md = None, 1e9
        for i, (ex, ey, t0) in list(self.pend.items()):
            if t - t0 > max_s:
                self.abandonadas.add(i)
                del self.pend[i]
                continue
            d = math.hypot(ex - cx, ey - cy)
            if d <= alcance_m and d < md:
                mejor, md = (ex, ey), d
        return mejor

    def tick(self, t, cx, cy, psi, dt=0.25) -> float:
        """Devuelve los segundos que el robot debe quedarse PARADO escaneando (0 = nada)."""
        c = self.cfg
        if self._prev is not None:
            paso = math.hypot(cx - self._prev[0], cy - self._prev[1])
            self.recorrido += paso
            sq = math.sqrt(paso)
            self.off[0] += float(self.rng.normal(0, 0.04 * sq))
            self.off[1] += float(self.rng.normal(0, 0.04 * sq))
            v = paso / dt
        else:
            v = 0.0
        self._prev = (cx, cy)
        pedir = 0.0
        for i, (px, py) in enumerate(self.plantas):
            d = self._en_cono(cx, cy, psi, px, py, c.mis_rango_m)
            dist_trig = max(c.mis_dist_insp_m, c.mis_dist_visita_m) if i in self.pend else c.mis_dist_insp_m    # una planta que se está visitando se escanea desde más lejos
            d_esc = self._en_cono(cx, cy, psi, px, py, dist_trig, fov_deg=160.0 if i not in self.pend else 90.0)      # al parar, la cámara con el servo abarca ±80°
            if d is None and d_esc is None:
                continue
            if d is None:
                d = d_esc
            if c.mis_inspeccionar and not self.escaneada[i] and d_esc is not None and \
                    all(math.hypot(px - self.plantas[j][0], py - self.plantas[j][1]) > 0.3 or not self.escaneada[j] for j in range(len(self.plantas)) if j != i):
                d = d_esc
                self.escaneada[i] = True
                self.escaneos += 1
                pedir = max(pedir, c.mis_escaneo_s)
                p = 0.92 * min(1.0, c.mis_escaneo_s / 6.0) * (1.0 if d <= 0.7 else 0.7)
                if self.rng.random() < p:
                    self._anotar(i, "escaneo", t, d)
                else:
                    self.escaneos_fallidos += 1
                    if self.intentos[i] < c.mis_reintentos:         # el escaneo no vio nada: se repite
                        self.intentos[i] += 1
                        self.escaneada[i] = False
                if self.rng.random() < 0.03:
                    self.det.append((cx + float(self.rng.normal(0, 0.5)), cy + float(self.rng.normal(0, 0.5)), "fp"))
                continue
            p = P_PASIVA * (1 - d / c.mis_rango_m) * (1 - 0.6 * min(v / 0.25, 1.0))
            if self.rng.random() < p:
                self._anotar(i, "pasivo", t, d)
        for (ox, oy) in self.obstaculos:                          # objetos que no son planta en el cono
            if self._en_cono(cx, cy, psi, ox, oy, c.mis_rango_m) is not None and self.rng.random() < 0.002:
                self.det.append((ox + float(self.rng.normal(0, 0.06)), oy + float(self.rng.normal(0, 0.06)), "fp"))
        self.t_parado += pedir
        return pedir

    def grupos(self):
        gs: list[list[float]] = []
        for x, y, _t in self.det:
            for g in gs:
                if math.hypot(x - g[0] / g[2], y - g[1] / g[2]) < DEDUP_M:
                    g[0] += x
                    g[1] += y
                    g[2] += 1
                    break
            else:
                gs.append([x, y, 1])
        return [(g[0] / g[2], g[1] / g[2]) for g in gs]

    def resumen(self) -> dict:
        """Conteo honesto: cada grupo se empareja con la planta libre más cercana a < 0,45 m (una vez cada una). Faltan (FN) y sobran (FP) se SUMAN:
        un grupo falso no tapa una planta perdida. error_conteo = (FN + FP) / plantas; error_neto = |detectadas - plantas| / plantas (el de antes)."""
        n = len(self.plantas)
        gs = self.grupos()
        libres = set(range(n))
        pares = sorted(((math.hypot(gx - px, gy - py), gi, i) for gi, (gx, gy) in enumerate(gs) for i, (px, py) in enumerate(self.plantas)
                        if math.hypot(gx - px, gy - py) < 0.45))
        usados_g, emparejados = set(), 0
        for _d, gi, i in pares:
            if gi in usados_g or i not in libres:
                continue
            usados_g.add(gi)
            libres.discard(i)
            emparejados += 1
        fn, fp = n - emparejados, len(gs) - emparejados
        return {"plantas": n, "detectadas": len(gs), "plantas_vistas": int(sum(self.ver_gt)), "emparejadas": emparejados,
                "recall": round(emparejados / max(n, 1), 3), "precision": round(emparejados / max(len(gs), 1), 3),
                "error_conteo": round((fn + fp) / max(n, 1), 3), "error_neto": round(abs(len(gs) - n) / max(n, 1), 3),
                "faltan": fn, "sobran": fp, "escaneos": self.escaneos, "escaneos_fallidos": self.escaneos_fallidos,
                "t_parado_s": round(self.t_parado, 1), "recorrido_m": round(self.recorrido, 1)}
