"""Entorno automático: ¿el robot está en INTERIOR o en EXTERIOR (huerta)?  (2026-10-20)

Decide el perfil de conducción (`modo_manejo` + `perfiles_entorno`) a partir de lo que MIDE el robot, en vez de
que el operador pulse Interior/Exterior. Dos fuentes:

  · LIDAR (peso principal): un recinto tiene casi todos los rayos con retorno cercano y puntos sobre rectas largas
    (paredes); la huerta tiene rayos sin retorno o lejanos y retornos dispersos de plantas.
  · Cámara (apoyo, peso menor): cielo azul y mucho verde empujan a exterior; cielo no se ve en un interior.

Puntuación en [-1, +1]: +1 = interior seguro, -1 = exterior seguro. Una puntuación instantánea no cambia nada:
se suaviza (EMA) y el cambio exige evidencia SOSTENIDA, con histéresis y asimetría (pasar a EXTERIOR sube la
velocidad de 0,13 a 0,5 y quita márgenes: es lo peligroso si se equivoca, así que pide más tiempo y más margen
que pasar a INTERIOR). Nunca cambia con el robot en movimiento, ni con algo a menos de 60 cm delante, y el
operador manda: si fuerza un modo a mano, el automático calla 10 minutos y esa pulsación queda como ETIQUETA.

Aprende de las correcciones: cada muestra (rasgos, puntuación, modo en uso, etiqueta del operador si la hay)
se guarda en ~/.agrobot/entorno/AAAAMMDD.jsonl. `tools/entorno_entrenar.py` ajusta con eso una regresión
logística y deja un CANDIDATO; solo se usa si se copia a ~/.agrobot/entorno/modelo.json (no se despliega solo).

LÍMITE HONESTO: las reglas se ajustaron con escaneos de INTERIOR reales y exteriores SIMULADOS. Aún no hay datos
reales de huerta: hasta tenerlos, el automático empieza en modo «solo sugerir» (apagado) y se activa a mano.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np

RASGOS = ("cercano", "cobertura", "mediana_norm", "rectitud", "rugosidad", "cielo", "verde", "brillo")
RANGO_MAX_M = 8.0            # más allá, el retorno no cuenta como «recinto»
RECINTO_M = 4.0
LINEA_TOL_M = 0.03
LINEA_MIN_PTS = 14
LINEA_MIN_LARGO_M = 0.7

CARPETA = Path.home() / ".agrobot" / "entorno"


# ----------------------------------------------------------------------------------------------
# Rasgos
# ----------------------------------------------------------------------------------------------
def _a_arrays(puntos):
    """Lista de {angle_deg, distance_m} -> (ángulos °, distancias m) con inf donde no hay retorno."""
    ang = np.array([float(p["angle_deg"]) for p in puntos], float)
    dis = np.array([float(p["distance_m"]) for p in puntos], float)
    return ang, dis


def _rectas(x, y, rng):
    """Fracción de puntos sobre rectas largas (RANSAC sobre hasta 4 rectas). Las paredes dan alto; las plantas, bajo."""
    n0 = len(x)
    if n0 < LINEA_MIN_PTS:
        return 0.0
    pts = np.stack([x, y], 1)
    restantes = np.ones(n0, bool)
    en_recta = 0
    for _ in range(4):
        idx = np.flatnonzero(restantes)
        if len(idx) < LINEA_MIN_PTS:
            break
        mejor, mejor_n = None, 0
        for _ in range(60):
            a, b = rng.choice(idx, 2, replace=False)
            d = pts[b] - pts[a]
            L = float(np.hypot(*d))
            if L < 0.05:
                continue
            nrm = np.array([-d[1], d[0]]) / L
            dist = np.abs((pts[idx] - pts[a]) @ nrm)
            ok = idx[dist < LINEA_TOL_M]
            if len(ok) > mejor_n:
                mejor, mejor_n = ok, len(ok)
        if mejor is None or mejor_n < LINEA_MIN_PTS:
            break
        # Largo de la recta = extensión de sus inliers sobre su dirección principal
        q = pts[mejor] - pts[mejor].mean(0)
        u = np.linalg.svd(q, full_matrices=False)[2][0]
        largo = float(np.ptp(q @ u))
        if largo >= LINEA_MIN_LARGO_M:
            en_recta += mejor_n
            restantes[mejor] = False
        else:
            restantes[mejor] = False       # un montón de puntos cercanos NO es una pared
    return min(1.0, en_recta / n0)


def rasgos_lidar(puntos, semilla=0):
    """Rasgos del LIDAR (todos en [0,1]) o None si el escaneo es demasiado pobre para opinar."""
    if not puntos:
        return None
    ang, dis = _a_arrays(puntos)
    n = len(dis)
    valido = np.isfinite(dis) & (dis > 0.05) & (dis <= RANGO_MAX_M)
    if int(valido.sum()) < 20:
        return {"cercano": 0.0, "cobertura": 0.0, "mediana_norm": 1.0, "rectitud": 0.0, "rugosidad": 1.0}
    # Escaneos de resolución distinta (360 o 720 rayos): todo se normaliza por el total de rayos.
    cercano = float(((dis <= RECINTO_M) & valido).sum()) / n
    cobertura = float(valido.sum()) / n
    mediana_norm = float(np.median(dis[valido])) / RANGO_MAX_M
    orden = np.argsort(ang)
    v = valido[orden]
    d = np.where(v, dis[orden], 0.0)       # sin retorno = 0: evita inf-inf; esos pares ya se descartan con `par`
    par = v[1:] & v[:-1]
    saltos = np.abs(np.diff(d))[par]
    rugosidad = float(np.clip(np.mean(saltos > 0.15) if saltos.size else 1.0, 0.0, 1.0))
    th = np.radians(ang[valido])
    rect = _rectas(dis[valido] * np.cos(th), dis[valido] * np.sin(th), np.random.default_rng(semilla))
    return {"cercano": cercano, "cobertura": cobertura, "mediana_norm": min(1.0, mediana_norm),
            "rectitud": rect, "rugosidad": rugosidad}


def rasgos_camara(frame_bgr):
    """Rasgos de la imagen (cielo azul, verde, brillo), o None sin imagen. Solo apoyo."""
    if frame_bgr is None:
        return None
    try:
        import cv2
        chica = cv2.resize(frame_bgr, (160, 120), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(chica, cv2.COLOR_BGR2HSV)
    except Exception:
        return None
    h, s, v = hsv[..., 0].astype(np.int32), hsv[..., 1].astype(np.int32), hsv[..., 2].astype(np.int32)
    arriba = slice(0, 42)                                   # el tercio de arriba
    cielo = float(np.mean((h[arriba] >= 95) & (h[arriba] <= 130) & (s[arriba] > 50) & (v[arriba] > 110)))
    verde = float(np.mean((h >= 32) & (h <= 85) & (s > 60) & (v > 40)))
    brillo = float(v.mean()) / 255.0
    return {"cielo": cielo, "verde": verde, "brillo": brillo}


def rasgos_nulos():
    return {k: None for k in RASGOS}


# ----------------------------------------------------------------------------------------------
# Puntuación
# ----------------------------------------------------------------------------------------------
def _clip(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


def puntuacion_lidar(r):
    """+1 interior … -1 exterior, solo con el LIDAR."""
    if r is None:
        return None
    # Pesos ajustados con 160 escaneos REALES de interior (cercano 1,00, rectitud 0,67, rugosidad 0,03) y
    # escenas simuladas de huerta abierta/dispersa (cercano ~0,55, rectitud ~0,08, rugosidad 0,18-0,23).
    # Una calle entre hileras densas se parece a un pasillo (rectas + cerca): puede salir «interior», que es el
    # error SEGURO (velocidad y márgenes de interior). El error peligroso, exterior dentro de una sala, no.
    s = (3.5 * (r["cercano"] - 0.78)            # casi todo el contorno cerca = recinto
         + 1.5 * (r["rectitud"] - 0.30)         # paredes = rectas largas
         - 9.0 * (r["rugosidad"] - 0.08)        # follaje: retornos que saltan de un rayo al siguiente
         + 1.0 * (0.5 - r["mediana_norm"]))     # retornos lejanos = abierto
    return _clip(s)


def puntuacion_camara(c):
    """+1 interior … -1 exterior, solo con la cámara (apoyo: pesa poco)."""
    if c is None:
        return None
    s = -4.0 * c["cielo"] - 0.8 * max(0.0, c["verde"] - 0.25) - 0.6 * max(0.0, c["brillo"] - 0.65)
    return _clip(0.35 + s)                      # sin cielo ni exceso de verde/brillo: ligeramente interior


def puntuacion(rasgos, modelo=None, peso_camara=0.3):
    """Puntuación final y desglose. Con `modelo` (regresión logística entrenada con tus datos) se usa ese."""
    if modelo:
        x = np.array([0.0 if rasgos.get(k) is None else float(rasgos[k]) for k in modelo["rasgos"]])
        z = float(((x - np.array(modelo["media"])) / np.array(modelo["desv"])) @ np.array(modelo["pesos"]) + modelo["sesgo"])
        return _clip(2.0 / (1.0 + math.exp(-z)) - 1.0), {"fuente": "modelo_entrenado"}
    sl = puntuacion_lidar(rasgos if rasgos.get("cercano") is not None else None)
    sc = puntuacion_camara(rasgos if rasgos.get("cielo") is not None else None)
    if sl is None and sc is None:
        return None, {"fuente": "sin_datos"}
    if sl is None:
        return sc * 0.5, {"fuente": "camara", "lidar": None, "camara": sc}
    if sc is None:
        return sl, {"fuente": "lidar", "lidar": sl, "camara": None}
    return _clip((1 - peso_camara) * sl + peso_camara * sc), {"fuente": "lidar+camara", "lidar": sl, "camara": sc}


# ----------------------------------------------------------------------------------------------
# Decisión con memoria
# ----------------------------------------------------------------------------------------------
class EntornoAuto:
    """Estado del decisor. No toca el robot: `paso()` devuelve qué haría; quien llama lo aplica."""

    ALFA = 0.15                       # EMA por muestra a ~1 Hz (constante de tiempo ~6 s)
    UMBRAL_INTERIOR = 0.25            # EMA >= esto sostenido -> pasar a INTERIOR
    UMBRAL_EXTERIOR = -0.40           # EMA <= esto sostenido -> pasar a EXTERIOR (más exigente)
    SOSTENIDO_INTERIOR_S = 10.0
    SOSTENIDO_EXTERIOR_S = 30.0
    HUECO_FRENTE_EXTERIOR_M = 0.6     # no se pasa a exterior con algo tan cerca delante
    SILENCIO_OPERADOR_S = 600.0

    def __init__(self, reloj=time.monotonic, modelo=None):
        self._reloj = reloj
        self.modelo = modelo
        self.activo = False                 # False = solo sugiere
        self.ema = None
        self.sugerido = None                # "interior" | "exterior" | None (sin evidencia clara)
        self._desde = None                  # desde cuándo la EMA apoya el cambio
        self._candidato = None
        self.silencio_hasta = 0.0
        self.ultimo_cambio = None
        self.muestras = 0
        self.rasgos = rasgos_nulos()
        self.desglose = {}
        self.puntuacion = None

    # --- el operador manda ---
    def operador_fijo(self, modo):
        self.silencio_hasta = self._reloj() + self.SILENCIO_OPERADOR_S
        self._candidato, self._desde = None, None

    def etiqueta_vigente(self):
        """Modo que el operador fijó a mano hace menos de SILENCIO_OPERADOR_S (para etiquetar muestras)."""
        return self._modo_operador if self._reloj() < self.silencio_hasta else None

    _modo_operador = None

    def fijar_operador(self, modo):
        self._modo_operador = modo if modo in ("interior", "exterior") else None
        self.operador_fijo(modo)

    # --- una muestra (≈1 Hz) ---
    def paso(self, rasgos, modo_actual, puede_cambiar=True, hueco_frente_m=None):
        """Devuelve {'cambiar_a': modo|None, ...}. `modo_actual` es el modo de manejo en uso."""
        t = self._reloj()
        self.rasgos = {**rasgos_nulos(), **{k: v for k, v in rasgos.items() if k in RASGOS}}
        p, self.desglose = puntuacion(self.rasgos, self.modelo)
        self.puntuacion = p
        if p is None:
            return {"cambiar_a": None, "motivo": "sin datos"}
        self.muestras += 1
        self.ema = p if self.ema is None else (1 - self.ALFA) * self.ema + self.ALFA * p
        if self.ema >= self.UMBRAL_INTERIOR:
            self.sugerido = "interior"
        elif self.ema <= self.UMBRAL_EXTERIOR:
            self.sugerido = "exterior"
        else:
            self.sugerido = None
        # Sólo se gestiona interior <-> exterior; «preciso» es una decisión del operador.
        if self.sugerido is None or self.sugerido == modo_actual or modo_actual not in ("interior", "exterior"):
            self._candidato, self._desde = None, None
            return {"cambiar_a": None, "motivo": "sin cambio"}
        if self._candidato != self.sugerido:
            self._candidato, self._desde = self.sugerido, t
        espera = self.SOSTENIDO_INTERIOR_S if self.sugerido == "interior" else self.SOSTENIDO_EXTERIOR_S
        if t - self._desde < espera:
            return {"cambiar_a": None, "motivo": f"esperando evidencia sostenida ({t - self._desde:.0f}/{espera:.0f} s)"}
        if not self.activo:
            return {"cambiar_a": None, "motivo": "automático apagado: solo sugiere"}
        if t < self.silencio_hasta:
            return {"cambiar_a": None, "motivo": "el operador fijó el modo a mano"}
        if not puede_cambiar:
            return {"cambiar_a": None, "motivo": "el robot se está moviendo"}
        if self.sugerido == "exterior" and hueco_frente_m is not None and hueco_frente_m < self.HUECO_FRENTE_EXTERIOR_M:
            return {"cambiar_a": None, "motivo": "obstáculo cerca delante: no se pasa a exterior"}
        self.ultimo_cambio = t
        self._candidato, self._desde = None, None
        return {"cambiar_a": self.sugerido, "motivo": "evidencia sostenida"}

    def estado(self, modo_actual=None):
        return {"activo": self.activo, "modo_actual": modo_actual, "sugerido": self.sugerido,
                "puntuacion": None if self.puntuacion is None else round(self.puntuacion, 3),
                "ema": None if self.ema is None else round(self.ema, 3),
                "rasgos": {k: (None if v is None else round(float(v), 3)) for k, v in self.rasgos.items()},
                "desglose": {k: (round(v, 3) if isinstance(v, float) else v) for k, v in self.desglose.items()},
                "silencio_operador_s": max(0, round(self.silencio_hasta - self._reloj())),
                "modelo_entrenado": bool(self.modelo), "muestras": self.muestras}


# ----------------------------------------------------------------------------------------------
# Registro y modelo entrenado
# ----------------------------------------------------------------------------------------------
def registrar(carpeta, muestra: dict):
    """Una línea por muestra, para entrenar después. No lanza nunca: el registro no puede tumbar el robot."""
    try:
        carpeta = Path(carpeta)
        carpeta.mkdir(parents=True, exist_ok=True)
        with open(carpeta / (time.strftime("%Y%m%d") + ".jsonl"), "a") as f:
            f.write(json.dumps(muestra, ensure_ascii=False) + "\n")
    except OSError:
        pass


def cargar_modelo(carpeta=CARPETA):
    """modelo.json (copiado a mano desde modelo_candidato.json) o None."""
    try:
        m = json.loads((Path(carpeta) / "modelo.json").read_text())
        if (set(m["rasgos"]) <= set(RASGOS) and len(m["pesos"]) == len(m["rasgos"])
                == len(m["media"]) == len(m["desv"])):
            return m
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return None
