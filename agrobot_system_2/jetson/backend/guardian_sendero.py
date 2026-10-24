"""Guardián del sendero: mantiene al robot sobre la línea de referencia y APRENDE de cada corrección que hace.

Qué hace
--------
1. Mide el error contra la referencia (recta o polilínea): `e_y` (m, + = el robot está a la izquierda del sendero) y `e_h` (rad, + = apunta
   a la izquierda del sendero), con la pose del SLAM.
2. Decide un pulso correctivo ('izquierda'/'derecha') y su escala. La ley base es de tipo Stanley: gira `e_h + atan(k·e_y / L)` hacia el
   sendero, dividido por lo que ESTE robot gira de verdad por pulso (se aprende midiéndolo, no se supone).
3. CORRELACIONA lo que percibe (distancias del LIDAR, IMU, cámara y el propio error) en una firma continua y la compara con su memoria
   de correcciones anteriores (k vecinos más parecidos). Si en situaciones parecidas una acción redujo el error, la repite con la escala
   que funcionó; si en ellas empeoró, la evita. Cada decisión sale con su explicación («parecido a 5 casos: derecha ×0,6 redujo el error un 62 %»).
4. APUNTA cada corrección real: antes, después, qué se mandó, cuánto giró y se desplazó de verdad, y si mejoró. Va a un JSONL
   (`~/.agrobot/aprendizaje/correcciones/`) y a la memoria (`guardian_memoria.json`), que persiste entre sesiones.

Sin ROS ni NumPy: corre en la Jetson y en el simulador (`sim_logica_actual`, `guardian=True`) con el mismo código.
Seguridades: no corrige si hay algo a menos de `margen_lateral_m` del lado al que giraría, ni con el frente casi bloqueado; si tres correcciones
seguidas no mejoran nada, ensancha la zona muerta un rato (anti-oscilación) y lo anota.
"""
from __future__ import annotations

import json
import math
import random
import time
from pathlib import Path

DIR_CORRECCIONES = Path.home() / ".agrobot" / "aprendizaje" / "correcciones"
RUTA_MEMORIA = Path.home() / ".agrobot" / "aprendizaje" / "guardian_memoria.json"

# firma: (nombre, divisor de normalización, peso)   — pesos por familia de sensor
_FIRMA = (
    ("lidar_izq", 2.0, 1.0), ("lidar_der", 2.0, 1.0), ("lidar_frente", 3.0, 0.8), ("lidar_atras", 2.0, 0.4), ("lidar_centrado", 1.0, 1.0),
    ("cam_lateral", 0.5, 1.0), ("cam_rumbo", 30.0, 1.0),
    ("imu_gz", 1.0, 0.5), ("imu_roll", 15.0, 0.4), ("imu_pitch", 15.0, 0.4),
    ("e_y", 0.4, 2.0), ("e_h", 30.0, 2.0),
)


def _ang(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _clip(v, lo, hi):
    return max(lo, min(hi, v))


def firma(sensores: dict, e_y: float, e_h: float) -> tuple[list[float], list[float]]:
    """-> (vector normalizado, pesos). Un dato que falta (None) o una cámara poco fiable pesan 0: no cuentan para parecerse."""
    def lid(k, tope):
        v = sensores.get(k)
        return tope if v is None else _clip(float(v), 0.0, tope)
    izq, der = lid("lidar_izquierda_m", 2.0), lid("lidar_derecha_m", 2.0)
    conf = float(sensores.get("sendero_confianza") or 0.0)
    cam_ok = conf >= 0.3 and sensores.get("sendero_lateral_m") is not None
    v = {
        "lidar_izq": izq, "lidar_der": der, "lidar_frente": lid("lidar_adelante_m", 3.0), "lidar_atras": lid("lidar_atras_m", 2.0),
        "lidar_centrado": (izq - der) / max(izq + der, 0.2),
        "cam_lateral": _clip(float(sensores.get("sendero_lateral_m") or 0.0), -0.5, 0.5),
        "cam_rumbo": _clip(float(sensores.get("sendero_rumbo_deg") or 0.0), -30.0, 30.0),
        "imu_gz": _clip(float(sensores.get("imu_gz") or 0.0), -1.0, 1.0),
        "imu_roll": _clip(float(sensores.get("imu_roll_deg") or 0.0), -15.0, 15.0),
        "imu_pitch": _clip(float(sensores.get("imu_pitch_deg") or 0.0), -15.0, 15.0),
        "e_y": _clip(e_y, -0.4, 0.4), "e_h": _clip(math.degrees(e_h), -30.0, 30.0),
    }
    tiene_imu = sensores.get("imu_gz") is not None or sensores.get("imu_roll_deg") is not None
    x, w = [], []
    for nombre, div, peso in _FIRMA:
        x.append(v[nombre] / div)
        if nombre.startswith("cam_") and not cam_ok:
            peso = 0.0
        if nombre.startswith("imu_") and not tiene_imu:
            peso = 0.0
        w.append(peso)
    return x, w


class MemoriaCorrecciones:
    """Casos (firma -> acción, escala, mejora medida). Búsqueda de vecinos por distancia ponderada; sin dependencias."""

    def __init__(self, ruta: Path | str | None = RUTA_MEMORIA, max_casos: int = 3000):
        self.ruta = Path(ruta) if ruta else None
        self.max_casos = max_casos
        self.casos: list[dict] = []
        self.sucio = 0
        if self.ruta and self.ruta.exists():
            try:
                self.casos = json.loads(self.ruta.read_text()).get("casos", [])
            except (OSError, ValueError):
                self.casos = []

    def agregar(self, x, accion, escala, mejora, extra=None):
        self.casos.append({"x": [round(v, 4) for v in x], "a": accion, "s": round(escala, 3), "m": round(mejora, 3), **(extra or {})})
        if len(self.casos) > self.max_casos:
            # olvida primero lo viejo que no sirvió (mejora <= 0); conserva lo útil
            malos = [i for i, c in enumerate(self.casos) if c["m"] <= 0]
            self.casos.pop(malos[0] if malos else 0)
        self.sucio += 1
        if self.ruta and self.sucio >= 20:
            self.guardar()

    def guardar(self):
        if not self.ruta:
            return
        self.ruta.parent.mkdir(parents=True, exist_ok=True)
        self.ruta.write_text(json.dumps({"casos": self.casos}))
        self.sucio = 0

    def vecinos(self, x, w, k=7, radio=0.35):
        """Los k casos más parecidos dentro de `radio` (distancia euclídea ponderada y normalizada por la suma de pesos)."""
        sw = sum(w) or 1.0
        out = []
        for c in self.casos:
            d = math.sqrt(sum(wi * (a - b) ** 2 for wi, a, b in zip(w, x, c["x"])) / sw)
            if d <= radio:
                out.append((d, c))
        out.sort(key=lambda t: t[0])
        return out[:k]

    def recomendar(self, x, w, min_casos=3, k=9):
        """-> (accion, escala, mejora_esperada, n, explicación) o None. Pondera por cercanía; por acción promedia la mejora."""
        vec = self.vecinos(x, w, k=k)
        if len(vec) < min_casos:
            return None
        por = {}
        for d, c in vec:
            p = 1.0 / (0.05 + d)
            e = por.setdefault(c["a"], {"p": 0.0, "m": 0.0, "s": 0.0, "n": 0})
            e["p"] += p
            e["m"] += p * c["m"]
            e["s"] += p * c["s"] * max(c["m"], 0.0)
            e["n"] += 1
        mejor = max(((a, e) for a, e in por.items() if e["n"] >= 2), key=lambda t: t[1]["m"] / t[1]["p"], default=None)
        if mejor is None:
            return None
        a, e = mejor
        m = e["m"] / e["p"]
        if m <= 0.05:
            return None
        peso_util = sum(max(c["m"], 0.0) / (0.05 + d) for d, c in vec if c["a"] == a)
        escala = e["s"] / peso_util if peso_util > 0 else 1.0
        expl = f"parecido a {len(vec)} casos: {a} ×{escala:.2f} redujo el error un {100 * m:.0f} % de media ({e['n']} casos)"
        return a, _clip(escala, 0.25, 1.6), m, e["n"], expl


class GuardianSendero:
    def __init__(self, memoria: MemoriaCorrecciones | None = None, *, tol_cm: float = 6.0, tol_deg: float = 4.0, k_y: float = 1.0,
                 mirada_m: float = 0.6, margen_lateral_m: float = 0.14, frente_min_m: float = 0.30, explorar: float = 0.0,
                 efecto_deg: float = 5.0, usar_memoria: bool = True, log_dir: Path | str | None = DIR_CORRECCIONES, rng=None,
                 min_casos: int = 3):
        self.memoria = memoria if memoria is not None else MemoriaCorrecciones()
        self.tol_y, self.tol_h = tol_cm / 100.0, math.radians(tol_deg)
        self.k_y, self.mirada, self.margen_lat, self.frente_min = k_y, mirada_m, margen_lateral_m, frente_min_m
        self.explorar = explorar
        self.usar_memoria = usar_memoria
        self.min_casos = min_casos
        self.rng = rng or random.Random(0)
        # giro medido por pulso a escala 1 (grados, sin signo); se aprende por separado hacia cada lado
        self.efecto = {"izquierda": efecto_deg, "derecha": efecto_deg}
        self.ref: list[tuple[float, float]] = []
        self.pend: dict | None = None
        self.zona_extra = 1.0                       # ensanche temporal de la zona muerta (anti-oscilación)
        self._sin_mejora = 0
        self.stats = {"correcciones": 0, "de_memoria": 0, "de_ley": 0, "mejoraron": 0, "empeoraron": 0, "bloqueadas": 0, "oscilacion": 0}
        self._log = None
        if log_dir:
            Path(log_dir).mkdir(parents=True, exist_ok=True)
            self._log = Path(log_dir) / f"guardian_{time.strftime('%Y%m%d')}.jsonl"

    # ------------------------------------------------------------ referencia y errores
    def fijar_recta(self, pose: dict, largo_m: float = 200.0):
        th = math.radians(pose["theta_deg"])
        self.ref = [(pose["x"], pose["y"]), (pose["x"] + largo_m * math.cos(th), pose["y"] + largo_m * math.sin(th))]

    def fijar_polilinea(self, pts):
        self.ref = [(float(x), float(y)) for x, y in pts]

    def errores(self, pose: dict):
        """(e_y, e_h) respecto del punto más cercano de la referencia; None si no hay referencia."""
        if len(self.ref) < 2:
            return None
        x, y, th = pose["x"], pose["y"], math.radians(pose["theta_deg"])
        mejor = None
        for (ax, ay), (bx, by) in zip(self.ref, self.ref[1:]):
            ex, ey = bx - ax, by - ay
            L2 = ex * ex + ey * ey
            if L2 < 1e-9:
                continue
            u = _clip(((x - ax) * ex + (y - ay) * ey) / L2, 0.0, 1.0)
            px, py = ax + u * ex, ay + u * ey
            d = math.hypot(x - px, y - py)
            if mejor is None or d < mejor[0]:
                lado = (ex * (y - ay) - ey * (x - ax)) / math.sqrt(L2)           # + = a la izquierda del sentido de la referencia
                mejor = (d, lado, math.atan2(ey, ex))
        if mejor is None:
            return None
        return mejor[1], _ang(th - mejor[2])

    # ------------------------------------------------------------ decisión
    def _ley(self, e_y, e_h):
        """Giro deseado (rad, + = izquierda) según Stanley con mirada fija."""
        return -(e_h + math.atan2(self.k_y * e_y, self.mirada))

    def decidir(self, pose: dict, sensores: dict) -> dict | None:
        err = self.errores(pose)
        if err is None or self.pend is not None:
            return None
        e_y, e_h = err
        z = self.zona_extra
        if abs(e_y) <= self.tol_y * z and abs(e_h) <= self.tol_h * z:
            return None
        delta = self._ley(e_y, e_h)
        if abs(delta) < self.tol_h * 0.8 * z:
            return None
        accion = "izquierda" if delta > 0 else "derecha"
        escala = _clip(abs(math.degrees(delta)) / max(self.efecto[accion], 1.0), 0.3, 1.5)
        origen, expl, mejora_esp = "ley", f"ley: e_y {100 * e_y:+.0f} cm, e_h {math.degrees(e_h):+.0f}° → {accion} ×{escala:.2f}", None
        x, w = firma(sensores, e_y, e_h)
        if self.usar_memoria:
            rec = self.memoria.recomendar(x, w, min_casos=self.min_casos)
            if rec is not None:
                a, s, m, n, ex = rec
                # la memoria manda solo si coincide en el SENTIDO de la corrección que pide la ley (no gira lejos del sendero)
                if a == accion or abs(e_y) < self.tol_y:
                    accion, escala, origen, expl, mejora_esp = a, s, "memoria", ex, m
        if self.explorar > 0 and self.rng.random() < self.explorar:
            escala = _clip(escala * self.rng.choice((0.6, 0.8, 1.25, 1.5)), 0.25, 1.6)
            origen, expl = "exploración", f"exploro: {accion} ×{escala:.2f}"
        # seguridades
        lado = "izquierda" if accion == "izquierda" else "derecha"
        d_lado = sensores.get(f"lidar_{lado}_m")
        d_frente = sensores.get("lidar_adelante_m")
        cerca = sensores.get("cerca")
        if (d_lado is not None and d_lado < self.margen_lat) or (d_frente is not None and d_frente < self.frente_min) \
                or (cerca is not None and cerca < self.margen_lat * 0.8):
            self.stats["bloqueadas"] += 1
            return None
        return {"accion": accion, "escala": round(escala, 3), "origen": origen, "explicacion": expl, "e_y": e_y, "e_h": e_h,
                "mejora_esperada": mejora_esp}

    # ------------------------------------------------------------ registro y aprendizaje
    def _costo(self, e_y, e_h):
        return abs(e_y) / max(self.tol_y, 1e-3) + abs(e_h) / max(self.tol_h, 1e-3)

    def iniciar(self, dec: dict, pose: dict, sensores: dict):
        x, w = firma(sensores, dec["e_y"], dec["e_h"])
        self.pend = {"dec": dec, "pose": dict(pose), "x": x, "sens": {k: sensores.get(k) for k in
                                                                        ("lidar_izquierda_m", "lidar_derecha_m", "lidar_adelante_m", "sendero_lateral_m", "imu_gz")},
                     "t": time.time()}

    def terminar(self, pose: dict, sensores: dict | None = None, colision: bool = False) -> dict | None:
        """Cierra la corrección pendiente con la pose ya asentada. Devuelve el registro (y lo guarda)."""
        if self.pend is None:
            return None
        p, self.pend = self.pend, None
        d0 = p["dec"]
        err = self.errores(pose)
        if err is None:
            return None
        e_y1, e_h1 = err
        c0, c1 = self._costo(d0["e_y"], d0["e_h"]), self._costo(e_y1, e_h1)
        mejora = (c0 - c1) / max(c0, 1e-6)
        if colision:
            mejora = min(mejora, -1.0)
        mejora = _clip(mejora, -1.5, 1.0)
        giro_real = math.degrees(_ang(math.radians(pose["theta_deg"] - p["pose"]["theta_deg"])))
        desp = math.hypot(pose["x"] - p["pose"]["x"], pose["y"] - p["pose"]["y"])
        a, s = d0["accion"], d0["escala"]
        # aprende cuánto gira este robot por pulso: promedio móvil del giro real en el sentido pedido
        pedido = 1.0 if a == "izquierda" else -1.0
        if s > 0.2 and abs(giro_real) < 40:
            medido = pedido * giro_real / s
            if medido > 0.3:
                self.efecto[a] = _clip(0.8 * self.efecto[a] + 0.2 * medido, 0.5, 30.0)
        self.memoria.agregar(p["x"], a, s, mejora, {"o": d0["origen"]})
        self.stats["correcciones"] += 1
        self.stats["de_memoria" if d0["origen"] == "memoria" else "de_ley"] += 1
        if mejora > 0.05:
            self.stats["mejoraron"] += 1
            self._sin_mejora = 0
            self.zona_extra = max(1.0, self.zona_extra * 0.9)
        else:
            self.stats["empeoraron"] += 1
            self._sin_mejora += 1
            if self._sin_mejora >= 3:
                self.zona_extra = min(2.0, self.zona_extra * 1.5)
                self.stats["oscilacion"] += 1
                self._sin_mejora = 0
        reg = {"t": round(p["t"], 2), "accion": a, "escala": s, "origen": d0["origen"], "explicacion": d0["explicacion"],
               "antes": {"e_y_cm": round(100 * d0["e_y"], 1), "e_h_deg": round(math.degrees(d0["e_h"]), 1), **p["sens"]},
               "despues": {"e_y_cm": round(100 * e_y1, 1), "e_h_deg": round(math.degrees(e_h1), 1)},
               "giro_real_deg": round(giro_real, 2), "desplazamiento_cm": round(100 * desp, 1), "mejora": round(mejora, 3),
               "efecto_aprendido_deg": {k: round(v, 2) for k, v in self.efecto.items()}, "colision": colision}
        if self._log is not None:
            try:
                with self._log.open("a") as f:
                    f.write(json.dumps(reg, ensure_ascii=False) + "\n")
            except OSError:
                pass
        return reg

    def resumen(self) -> dict:
        s = dict(self.stats)
        s["casos_memoria"] = len(self.memoria.casos)
        s["efecto_deg_por_pulso"] = {k: round(v, 2) for k, v in self.efecto.items()}
        s["zona_muerta_x"] = round(self.zona_extra, 2)
        return s
