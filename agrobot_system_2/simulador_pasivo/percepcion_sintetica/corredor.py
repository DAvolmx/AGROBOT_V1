"""Corredor libre y decisión de marcha a partir de la máscara de segmentación.

Pasos (todo en ejes del robot: x adelante, y izquierda, metros):
  1. La máscara de clases (alto x ancho) se proyecta al SUELO con `sendero.Cenital`: la MISMA
     proyección que usa el robot para el sendero visual, con la inclinación real de la cámara.
  2. Se mide el hueco libre (clase `camino_libre`) que contiene la trayectoria a tres distancias
     de mirada: su centro lateral y su ancho.
  3. Se mide la distancia al primer obstáculo (planta/maceta, borde bajo, obstáculo) dentro del
     PASILLO QUE BARRE el robot (su ancho + holgura). Esto detecta lo que el LIDAR a 18 cm NO ve.
  4. Una celda sin clase de suelo visible NO es transitable: "sin retorno" no significa libre.
  5. `Decision`: avanzar / reducir / parar, con el motivo.

LÍMITES: la proyección a un plano supone suelo plano. En surcos y pendientes el error crece
(2° de inclinación desplazan el suelo ~5 cm a 1 m); aquí se mide con inclinación conocida.
La holgura por defecto (5 cm por lado) es una hipótesis: la que salió de la simulación de
movimiento era numéricamente inestable y NO se usa (ver memoria del proyecto).
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI.parent.parent / "jetson" / "backend"))
import sendero  # noqa: E402

ANCHO_ROBOT_M = 0.37            # geometria_robot.ANCHO_ENVOLVENTE_M (eje delantero, el más ancho)
CLASE_LIBRE = 1
CLASES_BLOQUEO = (2, 3, 4)
MIRADAS_M = (0.6, 1.0, 1.4)

_CACHE: dict = {}


def cenital_para(inclinacion_deg, altura=0.40, hfov=71.0, adelante=0.42):
    clave = (round(inclinacion_deg * 4) / 4, altura, hfov, adelante)
    c = _CACHE.get(clave)
    if c is None:
        c = sendero.Cenital({"camara_altura_m": altura, "camara_inclinacion_deg": clave[0],
                             "camara_hfov_deg": hfov, "camara_adelante_m": adelante})
        if len(_CACHE) > 64:
            _CACHE.pop(next(iter(_CACHE)))
        _CACHE[clave] = c
    return c


def a_suelo(mascara, inclinacion_deg):
    """Máscara de clases (H,W) uint8 -> rejilla del suelo con 0 = no visible, clase+1 = clase."""
    import cv2
    c = cenital_para(inclinacion_deg)
    img = cv2.resize((mascara.astype(np.uint8) + 1), (sendero.ANCHO_ANALISIS, sendero.ALTO_ANALISIS),
                     interpolation=cv2.INTER_NEAREST)
    return c.proyectar(cv2, img), c


@dataclass
class Corredor:
    centro_m: dict = field(default_factory=dict)        # {mirada: y del centro del hueco libre}
    ancho_m: dict = field(default_factory=dict)         # {mirada: ancho del hueco libre (COTA INFERIOR si truncado)}
    izq_m: dict = field(default_factory=dict)           # {mirada: y del borde izquierdo}
    der_m: dict = field(default_factory=dict)
    trunc: dict = field(default_factory=dict)           # {mirada: (izq_truncado, der_truncado)} por campo de visión
    cabe: dict = field(default_factory=dict)            # {mirada: bool}
    obstaculo_m: float | None = None                    # distancia al primer obstáculo en el pasillo barrido
    clase_obstaculo: int | None = None
    en_zona_ciega: bool = False                         # el obstáculo llega a la zona que la cámara no ve
    libre_visible: float = 0.0                          # fracción del pasillo barrido, VISIBLE, que es suelo libre
    visible_m: float = 0.0                              # hasta dónde hay suelo libre continuo desde el borde visible
    zona_ciega_m: float = 0.0                           # distancia del centro del robot a la que EMPIEZA a verse suelo

    def resumen(self):
        f = lambda v: None if v is None else round(v * 100, 1)
        return {"centro_cm": {k: f(v) for k, v in self.centro_m.items()},
                "ancho_cm": {k: f(v) for k, v in self.ancho_m.items()},
                "obstaculo_cm": f(self.obstaculo_m), "en_zona_ciega": self.en_zona_ciega,
                "zona_ciega_cm": round(self.zona_ciega_m * 100), "libre_visible": round(self.libre_visible, 2),
                "visible_cm": round(self.visible_m * 100)}


def zona_ciega(inclinacion_deg):
    """Distancia (m) desde el centro del robot a la que empieza a verse el suelo, para esa inclinación."""
    G = np.ones((sendero.ALTO_ANALISIS, sendero.ANCHO_ANALISIS), np.uint8) * 2
    g, cen = a_suelo(G, inclinacion_deg)
    vis = np.nonzero((g > 0).any(1))[0]
    return float(cen.X[vis.max(), 0]) if vis.size else math.inf


def corredor(mascara, inclinacion_deg, holgura_m=0.05) -> Corredor:
    G, cen = a_suelo(mascara, inclinacion_deg)
    X, Y = cen.X, cen.Y                                  # (filas, cols): x adelante, y izquierda
    res = sendero.CENITAL_RES_M
    xs, ys = X[:, 0], Y[0, :]
    visible = G > 0
    libre = G == CLASE_LIBRE + 1
    bloq = np.isin(G, [c + 1 for c in CLASES_BLOQUEO])
    out = Corredor()
    filas_vis = np.nonzero(visible.any(1))[0]
    if not filas_vis.size:
        return out
    r_cerca = int(filas_vis.max())                       # fila visible MÁS cercana al robot
    x_cerca = float(xs[r_cerca])
    out.zona_ciega_m = x_cerca
    for la in MIRADAS_M:
        banda = np.nonzero((np.abs(xs - la) <= 0.08) & visible.any(1))[0]
        if not banda.size:
            out.centro_m[la] = out.ancho_m[la] = None
            out.cabe[la] = False
            continue
        fila = libre[banda].mean(0) > 0.5
        vis_c = visible[banda].mean(0) > 0.5
        c0 = int(np.argmin(np.abs(ys)))
        if not fila[c0]:
            cand = np.nonzero(fila)[0]
            cand = cand[np.abs(ys[cand]) < 0.35] if cand.size else cand
            if not cand.size:
                out.centro_m[la] = out.ancho_m[la] = None
                out.cabe[la] = False
                continue
            c0 = int(cand[np.argmin(np.abs(ys[cand]))])
        a = b = c0
        while a > 0 and fila[a - 1]:
            a -= 1
        while b < len(fila) - 1 and fila[b + 1]:
            b += 1
        # Truncado: el hueco llega hasta donde YA NO SE VE (campo de visión), no hasta un obstáculo.
        tr_i = (a == 0) or (not vis_c[a - 1])
        tr_d = (b == len(fila) - 1) or (not vis_c[b + 1])
        ancho = (b - a + 1) * res
        out.izq_m[la], out.der_m[la] = float(ys[a]), float(ys[b])
        out.centro_m[la] = float((ys[a] + ys[b]) / 2)
        out.ancho_m[la] = float(ancho)
        out.trunc[la] = (bool(tr_i), bool(tr_d))
        out.cabe[la] = bool(ancho >= ANCHO_ROBOT_M + 2 * holgura_m)
    # pasillo que barre el robot, SOLO en lo visible
    medio = ANCHO_ROBOT_M / 2 + holgura_m
    cols = np.nonzero(np.abs(ys) <= medio)[0]
    filas_rango = np.arange(0, r_cerca + 1)
    filas_rango = filas_rango[xs[filas_rango] <= 1.5]
    vis_p = visible[np.ix_(filas_rango, cols)]
    lib_p = libre[np.ix_(filas_rango, cols)]
    out.libre_visible = float(lib_p.sum() / max(vis_p.sum(), 1))
    # Una PARED (clase `fondo`) también bloquea: sus píxeles proyectan al suelo desde la línea de su base
    # hacia atrás, así que la primera fila con `fondo` en el pasillo barrido es el pie de la pared.
    # Antes del 2026-10-15 solo bloqueaban plantas/bordes/obstáculos y con una pared a 0,8 m decidía
    # "camino libre". Solo cuenta si hay suelo libre visible delante (si no, ya lo cubre `libre_visible`).
    bloq_total = bloq | (G == 1)
    filas_b = np.nonzero(bloq_total[:, cols].any(1))[0]
    if filas_b.size:
        i = filas_b[np.argmin(xs[filas_b])]
        out.obstaculo_m = float(xs[i])
        out.clase_obstaculo = int(G[i, cols][bloq_total[i, cols]][0]) - 1
        # Si el obstáculo llega a la fila visible más cercana, su BASE está en la zona ciega: lo que
        # se ve es su parte alta, cuya proyección al suelo cae MÁS LEJOS que el objeto real.
        # La distancia medida es entonces una cota SUPERIOR; se toma la conservadora.
        if i >= r_cerca - 3:
            out.en_zona_ciega = True
            out.obstaculo_m = min(out.obstaculo_m, x_cerca)
        elif out.clase_obstaculo == 0:
            pass                                     # pared con el pie visible: la distancia es fiable
    # suelo libre continuo desde el borde visible hacia delante
    zona_l = np.array([lib_p[k].mean() if vis_p[k].any() else 0.0 for k in range(len(filas_rango))])
    cont = x_cerca
    ok = False
    for k in range(len(filas_rango) - 1, -1, -1):        # de cerca a lejos
        if zona_l[k] >= 0.8:
            cont, ok = float(xs[filas_rango[k]]), True
        else:
            break
    out.visible_m = cont if ok else 0.0
    return out


@dataclass
class Decision:
    accion: str                  # "avanzar" | "reducir" | "parar"
    motivo: str
    velocidad: float = 1.0       # factor 0..1 sobre la velocidad normal


def _decidir_camara(c: Corredor, edad_s=0.0, d_parar=0.55, d_reducir=0.95, edad_max_s=0.5,
                    visible_min=0.8, libre_min=0.35) -> Decision:
    """Decisión SOLO con lo que ve la cámara ahora."""
    if edad_s > edad_max_s:
        return Decision("parar", f"percepción vieja ({edad_s:.2f} s > {edad_max_s} s)", 0.0)
    if c.libre_visible < libre_min:
        return Decision("parar", f"sin suelo libre visible en el pasillo ({c.libre_visible:.0%})", 0.0)
    if c.en_zona_ciega:
        return Decision("parar", f"obstáculo (clase {c.clase_obstaculo}) en la zona que la cámara no ve "
                                 f"(<{c.zona_ciega_m*100:.0f} cm)", 0.0)
    if c.obstaculo_m is not None and c.obstaculo_m < d_parar:
        return Decision("parar", f"obstáculo (clase {c.clase_obstaculo}) a {c.obstaculo_m*100:.0f} cm", 0.0)
    if c.obstaculo_m is not None and c.obstaculo_m < d_reducir:
        return Decision("reducir", f"obstáculo a {c.obstaculo_m*100:.0f} cm", 0.5)
    anchos = [c.cabe.get(k) for k in MIRADAS_M if k in c.cabe]
    if anchos and not any(anchos):
        return Decision("parar", "el hueco libre no admite la envolvente del robot", 0.0)
    if c.visible_m and c.visible_m < visible_min:
        return Decision("reducir", f"solo se ve suelo libre hasta {c.visible_m*100:.0f} cm", 0.5)
    return Decision("avanzar", "camino libre", 1.0)


_GRAVEDAD = {"avanzar": 0, "reducir": 1, "parar": 2}


def decidir(c: Corredor, edad_s: float = 0.0, d_parar=0.55, d_reducir=0.95, edad_max_s=0.5,
            visible_min=0.8, libre_min=0.35, dist_recordada=None) -> Decision:
    """Reglas explícitas y conservadoras. Datos viejos o falta de suelo visible => PARAR.

    `dist_recordada`: distancia a un obstáculo ya visto que salió de la vista por la zona ciega
    (MemoriaObstaculo). La memoria solo AÑADE cautela: se toma la más conservadora entre lo que
    ve la cámara ahora y lo que se recuerda. (Hasta el 2026-10-15 la memoria devolvía antes de
    evaluar a la cámara y podía rebajar un "parar" a "reducir": en lazo cerrado subió los
    contactos con patas de silla del 19 % al 94 %.)
    """
    base = _decidir_camara(c, edad_s, d_parar, d_reducir, edad_max_s, visible_min, libre_min)
    if dist_recordada is None:
        return base
    if dist_recordada < d_parar:
        mem = Decision("parar", f"obstáculo recordado a {dist_recordada*100:.0f} cm (fuera de la vista)", 0.0)
    elif dist_recordada < d_reducir:
        mem = Decision("reducir", f"obstáculo recordado a {dist_recordada*100:.0f} cm", 0.5)
    else:
        return base
    return mem if _GRAVEDAD[mem.accion] > _GRAVEDAD[base.accion] else base


class MemoriaObstaculo:
    """Recuerda un obstáculo visto ANTES de que entrara en la zona ciega de la cámara.

    La cámara solo ve suelo a partir de ~0,9 m (15,5°) o ~0,67 m (33°). Un objeto bajo que se
    acerca desaparece de la vista a esa distancia aunque siga delante. Esto lo mantiene: guarda la
    distancia vista y le resta el avance medido por odometría. Caduca si el robot gira mucho (el
    objeto ya no está en su pasillo), si pasa demasiado tiempo sin confirmarlo, o si el robot ha
    avanzado más de lo que quedaba (se supone que ya lo pasó o lo tocó: se avisa).

    IMPORTANTE: depende de que la odometría de avance sea buena. Con el robot real la odometría
    visual/EKF tuvo velocidades absurdas en la auditoría (9.684 registros): aquí se parametriza el
    avance y NO se asume que el que entrega el robot sea fiable.
    """

    def __init__(self, caducidad_s=8.0, giro_max_rad=math.radians(20)):
        self.caducidad_s, self.giro_max = caducidad_s, giro_max_rad
        self.dist = None
        self.clase = None
        self.t_visto = None
        self.giro = 0.0

    def actualizar(self, c: Corredor, avance_m: float, giro_rad: float, t_s: float):
        if self.dist is not None:
            self.dist -= avance_m
            self.giro += abs(giro_rad)
            if self.giro > self.giro_max or (t_s - self.t_visto) > self.caducidad_s:
                self.dist = self.clase = None
                self.giro = 0.0
        if c.obstaculo_m is not None:
            obs = c.obstaculo_m
            if self.dist is None or c.en_zona_ciega is False:
                # observación directa con la base visible: más fiable que lo recordado
                self.dist, self.clase, self.t_visto, self.giro = obs, c.clase_obstaculo, t_s, 0.0
            else:
                self.dist = min(self.dist, obs)
                self.t_visto = t_s
        return self.dist


class SeguidorCorredor:
    """Continuidad temporal: filtra el centro/ancho y exige datos frescos. Parada ante pérdida."""

    def __init__(self, alfa=0.5, perdidas_max=3):
        self.alfa, self.perdidas_max = alfa, perdidas_max
        self.centro = self.ancho = None
        self.perdidas = 0
        self.t_ultimo = None

    def actualizar(self, c: Corredor, t_s: float, mirada=1.0):
        self.t_ultimo = t_s
        cen, anc = c.centro_m.get(mirada), c.ancho_m.get(mirada)
        if cen is None:
            self.perdidas += 1
        else:
            self.perdidas = 0
            self.centro = cen if self.centro is None else self.alfa * cen + (1 - self.alfa) * self.centro
            self.ancho = anc if self.ancho is None else self.alfa * anc + (1 - self.alfa) * self.ancho
        return self.estado(t_s)

    def estado(self, t_s):
        edad = math.inf if self.t_ultimo is None else t_s - self.t_ultimo
        perdido = self.perdidas >= self.perdidas_max
        return {"centro_m": self.centro, "ancho_m": self.ancho, "edad_s": edad, "perdido": perdido}
