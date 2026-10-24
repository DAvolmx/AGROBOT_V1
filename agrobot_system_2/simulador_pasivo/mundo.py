"""
Mundo 2D del simulador: qué hay en el terreno y qué ve cada sensor.
===================================================================

La distinción que da sentido a todo el simulador (y que es el problema real
del robot, medido el 2026-09-19):

  · ALTO (paredes, cercas, taludes, cajas): lo ve el LIDAR, que va a ~40 cm
    del suelo, y frena por él.
  · BAJO (sendero, paja del mulch, plantas chicas): el LIDAR pasa por
    ENCIMA y no lo ve. Solo existe para la cámara.

Así, en el simulador, seguir el sendero obliga a usar la cámara igual que en
el campo, y el LIDAR sirve para no chocar.

Ejes del mundo en metros: x a la derecha, y hacia arriba, ángulos en radianes
con 0 = +x (igual que el mapa del SLAM).
"""
import functools
import os
import json
import math

import numpy as np

# Colores BGR, tomados de las fotos de la huerta real del usuario.
COLOR_TIERRA = (78, 96, 112)
COLOR_TIERRA_SECA = (110, 132, 150)     # el terreno del 2026-09-19, seco y claro
COLOR_PAJA = (105, 165, 195)
COLOR_PAJA_CLARA = (140, 190, 215)
COLOR_PLANTA = (50, 150, 70)
COLOR_PLANTA_ALTA = (38, 105, 52)      # mata crecida: verde más oscuro y cerrado
COLOR_PLANTA_SECA = (60, 130, 120)
COLOR_PIEDRA = (120, 125, 130)
COLOR_HIERBA = (55, 120, 75)
COLOR_PARED = (120, 120, 125)
COLOR_CIELO = (205, 195, 185)
COLOR_CIELO_ZENITH = (170, 148, 105)       # cenit azul-gris (BGR)
COLOR_CIELO_HORIZ = (225, 218, 212)        # horizonte cálido
COLOR_BRUMA = (215, 208, 198)              # neblina de distancia
COLOR_PARED_BASE = (95, 100, 112)          # base de pared (oscura, tinte azul)
COLOR_PARED_TOPE = (148, 148, 155)         # tope de pared (más claro)

# Clases del suelo (las devuelve `clases_suelo`).
TIERRA, PAJA, PLANTA, PIEDRA, HIERBA, BORDE_TIERRA, PLANTA_ALTA = 0, 1, 2, 3, 4, 5, 6

# De qué está hecho el CAMINO (lo que pisa el robot). El detector de senderos
# decide por contraste de color y textura entre el camino y sus bordes, y en
# el campo el camino no siempre es tierra limpia. Colores BGR:
#   tierra     la de siempre (entre húmeda y seca según `seco`)
#   grava      gris con granos claros y oscuros muy marcados
#   cesped     hierba pisada: verde apagado con calvas de tierra
#   hojarasca  hojas secas: manchas marrón-naranja sobre tierra
#   barro      tierra oscura y brillante con charcos
SUPERFICIES = ("tierra", "grava", "cesped", "hojarasca", "barro", "baldosa")
COLOR_GRAVA = (128, 132, 136)
COLOR_CESPED = (62, 118, 88)
COLOR_HOJARASCA = (48, 92, 142)
COLOR_BARRO = (48, 60, 72)

# Cámara por defecto (la medida en el robot). entrenar.aleatorizar() sortea
# otros valores por episodio: la misma escena con otra exposición, balance de
# blancos, ruido o compresión es otra imagen para el detector.
CAMARA_POR_DEFECTO = {
    "ganancia": 1.0,            # exposición: <1 subexpuesta, >1 quemada
    "balance": (1.0, 1.0, 1.0), # multiplicador B, G, R (luz fría / cálida)
    "ruido": 1.0,               # escala de RUIDO_BASE / RUIDO_EN_SOMBRA
    "vineta": 1.0,              # escala de VIÑETEO
    "jpeg": 85,                 # calidad JPEG (0 = sin comprimir); la NexiGo da MJPEG
}
COLOR_BORDE_TIERRA = (52, 62, 78)      # caballón de tierra removida, ni paja ni verde
COLOR_MACETA = (70, 95, 140)
COLOR_RAMA = (60, 125, 85)

# --- Realismo de los sensores -----------------------------------------------
# Todo esto se puede apagar (`realismo=0`) para que las tandas largas corran
# más rápido, o subir para entrenar contra condiciones peores de las reales.
# Los valores por defecto salen de las fotos y los registros del robot:
# imágenes movidas al avanzar, viñeteo del gran angular, ruido en sombra, y
# un LIDAR que pierde rayos sobre tierra suelta y hierba.
DESENFOQUE_POR_VELOCIDAD = 26.0    # px de arrastre por (m/s) a 320 px de ancho
# Exposición de la NexiGo N60 en la huerta: 156 × 100 µs = 15,6 ms por
# defecto en el robot (v4l2 exposure_time_absolute), más en poca luz.
EXPOSICION_CAMARA_S = 0.02
VIÑETEO = 0.35                     # cuánto cae la luz en las esquinas
# Ruido temporal medido en la NexiGo N60 del robot (2026-09-30, 10 fotogramas
# con el robot quieto, interior con poca luz): 1,0 niveles de desviación por
# píxel. La webcam ya filtra el ruido en el propio chip. Antes 4,0/14,0: el
# detector se entrenaba contra una cámara el triple de ruidosa.
RUIDO_BASE, RUIDO_EN_SOMBRA = 0.85, 3.2
SOL_ACIMUT_DEG, SOL_ALTURA_DEG = 135.0, 42.0
SOMBRA_OSCURIDAD = 0.45
# Terreno pintado una vez (ver Mundo.raster_suelo): resolución, tope de
# píxeles para mapas grandes y cuánto se pinta más allá de los límites.
RASTER_M_POR_PX = 0.008
RASTER_MAX_PX = 6_000_000
RASTER_MARGEN_M = 2.0
GROSOR_PARED_M = 0.07             # el mismo que pinta el visor 3D

# LIDAR. El C1 no devuelve un rayo limpio por ángulo: sobre tierra suelta,
# hierba o superficies oscuras pierde retornos, el error crece con la
# distancia, y en los bordes de un objeto un mismo rayo recoge dos
# profundidades y devuelve algo intermedio que no existe.
#
# Calibrado con 100 scans del C1 real, robot quieto en la habitación del
# usuario (2026-09-30; análisis en el MANIFEST). Antes el ruido era ~10 veces
# el real y un tercio de los bordes salían mezclados: el SLAM y el freno se
# entrenaban contra un sensor mucho peor que el que lleva el robot.
#   ruido (sigma):  1,3 mm a 0,6 m · 1,8 a 1,5 · 3,3 a 2,5 · 13,5 mm a 4 m
#                   ->  1,2 mm + 0,18 mm/m³ · d³
#   atípicos:       2,2 % de las lecturas a más de 5 sigma (colas de 1-10 cm)
#   sin retorno:    0,8 % a menos de 1 m y ~4,5 % a partir de 1,5 m
#   bordes:         0,3 % de lecturas "entre medias"; lo normal es que el
#                   haz salte entre el objeto y el fondo
# En exterior (hojas, tierra oscura, sol) todo esto empeora: `realismo` y la
# aleatorización de dominio de entrenar.py lo escalan hacia arriba.
LIDAR_PERDIDA_BASE = 0.005         # fracción de rayos sin retorno, de cerca
LIDAR_PERDIDA_LEJOS = 0.05         # a partir de LIDAR_PERDIDA_RAMPA_M
LIDAR_PERDIDA_RAMPA_M = 1.5
LIDAR_RUIDO_M = 0.0012             # ruido de base
LIDAR_RUIDO_CUBICO = 0.00018       # m de sigma por m³ de distancia
# Fracción de lecturas con error de cola. Mayor que el 2,2 % medido "a más de
# 5 sigma" porque parte de la cola de la Laplace cae por debajo de 5 sigma.
LIDAR_ATIPICOS = 0.045
LIDAR_ATIPICO_ESCALA_M = 0.02      # escala (Laplace) de ese error
LIDAR_MEZCLA_BORDE = 0.05          # probabilidad de píxel mezclado en un salto
# El propio robot en el scan (medido): el mástil de la cámara a 15,5 cm hacia
# 20-26° y un cable a 7,8 cm hacia -133..-122° (ángulos del robot, 0 =
# adelante). El robot real los filtra a 0,18 m en ROS y por huella en el
# freno; el simulado tiene que verlos para que esos filtros se prueben.
LIDAR_AUTOIMPACTOS = ((20.0, 26.0, 0.155, 0.95), (-133.0, -122.0, 0.078, 0.95))
# Recorte de alcance del SLAM real (filtro_scan_slam, launch de ROS).
LIDAR_RANGO_MIN_SLAM = 0.18

# ALTURA A LA QUE BARRE EL LIDAR. Es el número que explica casi todos los
# problemas del robot real:
#   · Lo que llega a esta altura, el LIDAR lo ve — aunque sean hojas por las
#     que se podría pasar: frena por un obstáculo que no existe.
#   · Lo que no llega, no existe para el LIDAR — y el robot PISA las plantas
#     chicas creyendo que el camino está libre.
ALTURA_LIDAR_M = 0.30

# Catálogo de objetos que se pueden poner en el mundo (y en el editor).
#   altura  : hasta dónde llega (m)
#   solido  : si el robot choca de verdad contra él
#   radio   : tamaño por defecto (m)
TIPOS_OBJETO = {
    "maceta":  {"altura": 0.35, "solido": True,  "radio": 0.16, "color": COLOR_MACETA},
    "rama":    {"altura": 0.55, "solido": False, "radio": 0.28, "color": COLOR_RAMA},
    "arbusto": {"altura": 0.70, "solido": False, "radio": 0.40, "color": COLOR_RAMA},
    "piedra":  {"altura": 0.32, "solido": True,  "radio": 0.20, "color": COLOR_PIEDRA},
    "planta":  {"altura": 0.12, "solido": False, "radio": 0.10, "color": COLOR_PLANTA},
}


def _hash2(i, j):
    """Pseudoaleatorio estable por celda entera del plano (0..1)."""
    n = np.sin(i * 12.9898 + j * 78.233) * 43758.5453
    return n - np.floor(n)


def ruido(px, py, escala=1.0, octavas=2):
    """Ruido de valor en coordenadas del MUNDO, no de la imagen.

    Que dependa del mundo es lo importante: el mismo trozo de suelo se ve
    igual desde cualquier sitio y no “hierve” al moverse el robot. El
    producto de senos que había antes era periódico y salía un cuadriculado
    de tela."""
    total = np.zeros(np.shape(px), np.float32)
    amplitud, peso = 1.0, 0.0
    for o in range(octavas):
        e = escala * (2 ** o)
        x, y = np.asarray(px, np.float32) * e, np.asarray(py, np.float32) * e
        i, j = np.floor(x), np.floor(y)
        fx, fy = x - i, y - j
        sx, sy = fx * fx * (3 - 2 * fx), fy * fy * (3 - 2 * fy)
        a = _hash2(i, j); b = _hash2(i + 1, j)
        c = _hash2(i, j + 1); d = _hash2(i + 1, j + 1)
        total += amplitud * ((a * (1 - sx) + b * sx) * (1 - sy) + (c * (1 - sx) + d * sx) * sy)
        peso += amplitud
        amplitud *= 0.5
    return total / peso


# Render de la cámara en la GPU (camara_gpu.py). None = sin decidir todavía,
# False = no hay GPU usable (o AGROBOT_SIM_GPU=0) y se usa la CPU.
_GPU = None


def _render_gpu():
    global _GPU
    if _GPU is None:
        _GPU = False
        if os.environ.get("AGROBOT_SIM_GPU", "auto").lower() not in ("0", "no", "false"):
            try:
                import warnings
                # La rueda cu130 avisa de que no trae kernels para la Orin
                # (CC 8.7); funciona igual (ver camara_gpu.py).
                warnings.filterwarnings("ignore", category=UserWarning, module="torch.cuda")
                import torch
                if torch.cuda.is_available():
                    from camara_gpu import RenderGPU
                    _GPU = RenderGPU()
                    print("[camara] render de la cámara simulada en la GPU", flush=True)
            except Exception as exc:
                print(f"[camara] sin GPU ({type(exc).__name__}: {exc}): render en CPU", flush=True)
    return _GPU or None


def resolucion_camara():
    """La de la NexiGo del robot si la GPU puede con ella; si no, la mitad
    (lo que el detector usa de todas formas)."""
    return (640, 480) if _render_gpu() is not None else (320, 240)


@functools.lru_cache(maxsize=8)
def _base_camara(ancho, alto, hfov_deg):
    """Lo de la cámara que no depende ni del robot ni de la inclinación:
    la rejilla de píxeles, el cielo con sus nubes y el degradado de las
    paredes. El servo barre ángulos todo el rato (búsqueda del sendero,
    escaneos) y recalcular esto en cada ángulo costaba más que la imagen."""
    fx = (ancho / 2) / math.tan(math.radians(hfov_deg) / 2)
    u, v = np.meshgrid(np.arange(ancho, dtype=np.float32), np.arange(alto, dtype=np.float32))
    xn, yn = (u - ancho / 2) / fx, (v - alto / 2) / fx
    fila = np.clip((v / alto - 0.05) / 0.42, 0, 1)[..., None]
    cielo = (np.array(COLOR_CIELO_ZENITH, np.float32) * (1 - fila)
             + np.array(COLOR_CIELO_HORIZ, np.float32) * fila)
    cielo += 14 * (ruido(u * 0.004, v * 0.009, 2.0, 2) - 0.5)[..., None]
    filas = np.arange(alto, dtype=np.float32)
    mezcla_p = np.clip((filas / alto - 0.15) / 0.5, 0, 1)[:, None]
    pared_fila = (np.array(COLOR_PARED_TOPE, np.float32) * (1 - mezcla_p)
                  + np.array(COLOR_PARED_BASE, np.float32) * mezcla_p)
    pared_fila *= (1.0 + 0.03 * np.sin(filas * 3.5 + 0.7))[:, None]
    return {"fx": fx, "xn": xn, "yn": yn,
            "cielo": np.clip(cielo, 0, 255).astype(np.float32),
            "pared_fila": pared_fila.astype(np.float32),
            "angulos": np.arctan2(-xn[0], 1.0)}


class Mundo:
    """Terreno: paredes (LIDAR), sendero con paja (cámara) y plantas."""

    def __init__(self, nombre, paredes=(), sendero=(), ancho_sendero=0.6,
                 ancho_paja=0.35, plantas=(), limites=(-6, -6, 6, 6),
                 escaneos=(), seco=0.35, objetos=(), borde="paja", superficie="tierra"):
        self.nombre = nombre
        self.especies = []
        self.paredes = np.array(paredes, dtype=np.float32).reshape(-1, 4)   # x1,y1,x2,y2
        self.sendero = np.array(sendero, dtype=np.float32).reshape(-1, 2)   # polilínea del centro
        self.ancho_sendero = ancho_sendero
        self.ancho_paja = ancho_paja
        # x, y, radio, plaga(0/1), altura(m). Las de 4 columnas (mapas
        # antiguos) se completan con una altura baja: eran plantas chicas.
        self.plantas = self._normalizar_plantas(plantas)
        # Puntos de escaneo: plantas marcadas para pararse a mirarlas de cerca,
        # igual que los 🌱 del dashboard del robot real.
        self.escaneos = np.array(escaneos, dtype=np.float32).reshape(-1, 2)
        # Objetos sueltos: [x, y, radio, altura, solido(0/1)] + su tipo aparte.
        self.objetos = np.zeros((0, 5), np.float32)
        self.tipos = []
        self.poner_objetos(objetos or [])
        self.pisadas = set()               # plantas que el robot ya aplastó
        self.limites = limites
        self.seco = float(seco)            # 0 = tierra húmeda oscura, 1 = seca y clara
        # De qué es el borde del sendero. No siempre es paja: en muchos
        # surcos lo que lo delimita es vegetación verde y alta.
        #   "paja" | "vegetacion" | "mixto"
        self.borde = borde
        self.superficie = superficie if superficie in SUPERFICIES else "tierra"
        # Sol de ESTE mundo (dirección de las sombras y del brillo del suelo).
        self.sol_acimut = SOL_ACIMUT_DEG
        self.sol_altura = SOL_ALTURA_DEG
        self.sombra_oscuridad = SOMBRA_OSCURIDAD
        self.camara_params = dict(CAMARA_POR_DEFECTO)
        # Cuánto peor que en la habitación (donde se midió) está el LIDAR:
        # hojas, tierra oscura y sol directo pierden más haces. 1 = medido.
        self.lidar_peor = 1.0

    @staticmethod
    def _normalizar_plantas(plantas):
        arr = np.array(plantas, dtype=np.float32)
        if arr.size == 0:
            return np.zeros((0, 5), np.float32)
        arr = arr.reshape(-1, arr.shape[-1])
        if arr.shape[1] == 4:
            alturas = np.clip(arr[:, 2] * 1.6, 0.06, 0.25)[:, None]
            arr = np.hstack([arr, alturas.astype(np.float32)])
        return arr[:, :5].astype(np.float32)

    @property
    def plantas_altas(self):
        """Las que llegan al LIDAR: el robot las ve y frena por ellas.

        Es la diferencia que trae problemas en el campo: una mata crecida
        aparece en el scan como si fuera un muro, y una plantita de 10 cm no
        existe para el LIDAR aunque el robot la aplaste."""
        if not len(self.plantas):
            return self.plantas
        return self.plantas[self.plantas[:, 4] >= ALTURA_LIDAR_M]

    def poner_objetos(self, objetos):
        """`objetos` = [{"tipo": "maceta", "x":.., "y":.., "radio":opcional}, ...]"""
        filas, tipos = [], []
        for o in objetos:
            base = TIPOS_OBJETO.get(o.get("tipo", "maceta"), TIPOS_OBJETO["maceta"])
            filas.append([float(o["x"]), float(o["y"]),
                          float(o.get("radio", base["radio"])),
                          float(o.get("altura", base["altura"])),
                          1.0 if o.get("solido", base["solido"]) else 0.0])
            tipos.append(o.get("tipo", "maceta"))
        self.objetos = np.array(filas, np.float32).reshape(-1, 5)
        self.tipos = tipos
        self._cache_dibujo = None

    @property
    def remates_paredes(self):
        """Los extremos de cada pared, como círculos del grosor de la pared.

        Las paredes son segmentos sin grosor: un rayo que llega de canto no
        las corta, así que el LIDAR simulado no veía la PUNTA de un bancal
        mirándola de frente. En el invernadero el robot se estrellaba contra
        ella una y otra vez (81 choques en 114 s el 2026-09-30); el C1 real
        sí la ve, porque la pared tiene 7 cm de grosor."""
        cache = getattr(self, "_remates", None)
        if cache is not None and cache[0] == len(self.paredes):
            return cache[1]
        if not len(self.paredes):
            remates = np.zeros((0, 5), np.float32)
        else:
            puntos = np.unique(np.round(np.vstack([self.paredes[:, :2], self.paredes[:, 2:]]), 3), axis=0)
            remates = np.column_stack([puntos, np.full(len(puntos), GROSOR_PARED_M / 2),
                                       np.full(len(puntos), 0.6), np.ones(len(puntos))]).astype(np.float32)
        self._remates = (len(self.paredes), remates)
        return remates

    @property
    def objetos_visibles_lidar(self):
        """Los que llegan a la altura del LIDAR: los ve, sean o no sólidos."""
        if not len(self.objetos):
            return self.objetos
        return self.objetos[self.objetos[:, 3] >= ALTURA_LIDAR_M]

    # ---------------------------------------------------------------- LIDAR
    def rayos(self, x, y, theta, angulos, alcance=8.0, solo_paredes=False):
        """Distancia a la primera pared para cada ángulo (relativo a theta).

        Vectorizado: todos los rayos contra todas las paredes de una vez.
        Devuelve `alcance` donde no hay nada (el driver marca esos haces como
        fuera de rango, igual que el C1 real)."""
        if len(self.paredes) == 0:
            return np.full(len(angulos), alcance, np.float32)
        ang = theta + np.asarray(angulos, np.float32)
        dx, dy = np.cos(ang), np.sin(ang)                       # (R,)
        x1, y1, x2, y2 = self.paredes.T                          # (P,)
        ex, ey = (x2 - x1)[None, :], (y2 - y1)[None, :]          # (1,P)
        den = dx[:, None] * ey - dy[:, None] * ex                # (R,P)
        seguro = np.where(np.abs(den) < 1e-9, np.nan, den)
        px, py = (x1 - x)[None, :], (y1 - y)[None, :]
        t = (px * ey - py * ex) / seguro                         # avance sobre el rayo
        u = (px * dy[:, None] - py * dx[:, None]) / seguro       # posición en la pared 0..1
        valido = (t > 0.02) & (u >= 0) & (u <= 1) & np.isfinite(t)
        t = np.where(valido, t, np.inf)
        d = t.min(axis=1)

        # Objetos redondos que llegan a la altura del LIDAR. Aquí es donde una
        # rama sin nada sólido detrás produce un eco igual que una pared.
        # La cámara solo se tapa con paredes: con plantas altas y objetos
        # dentro, se pintaban como losas grises de pared en la imagen.
        obj = self.objetos_visibles_lidar if not solo_paredes else np.zeros((0, 5), np.float32)
        altas = self.plantas_altas if not solo_paredes else np.zeros((0, 5), np.float32)
        if len(altas):
            obj = (np.vstack([obj, np.column_stack(
                [altas[:, 0], altas[:, 1], altas[:, 2], altas[:, 4],
                 np.zeros(len(altas), np.float32)])]) if len(obj)
                else np.column_stack([altas[:, 0], altas[:, 1], altas[:, 2], altas[:, 4],
                                      np.zeros(len(altas), np.float32)]))
        remates = self.remates_paredes
        if len(remates):
            obj = np.vstack([obj, remates]) if len(obj) else remates
        if len(obj):
            cx, cy, r = obj[:, 0] - x, obj[:, 1] - y, obj[:, 2]
            proy = dx[:, None] * cx[None, :] + dy[:, None] * cy[None, :]
            perp2 = (cx * cx + cy * cy)[None, :] - proy ** 2
            corta = perp2 < r[None, :] ** 2
            dentro = np.sqrt(np.maximum(r[None, :] ** 2 - perp2, 0))
            t_obj = np.where(corta & (proy - dentro > 0.02), proy - dentro, np.inf)
            d = np.minimum(d, t_obj.min(axis=1))
        return np.minimum(d, alcance).astype(np.float32)

    def scan(self, x, y, theta, n=720, alcance=8.0, rng=None, realismo=1.0,
             omega=0.0, periodo=0.1):
        """Scan de 360° como el del RPLIDAR C1.

        Con `realismo` se añaden los defectos del sensor de verdad: rayos
        perdidos (más cuanto más lejos), ruido que crece con la distancia y
        píxeles mezclados en los bordes. Un scan perfecto le pone las cosas
        demasiado fáciles al casamiento de scans y al freno.

        Y la distorsión por giro: el C1 tarda `periodo` en dar la vuelta y
        cada haz sale desde el rumbo que tenía el robot EN ESE INSTANTE, pero
        el scan llega como si todo se hubiera medido a la vez. Girando a
        2 rad/s el principio y el final de la vuelta difieren 11°: las
        paredes salen curvadas, y eso es parte de por qué el casamiento de
        scans falla en los giros."""
        ang = np.linspace(-math.pi, math.pi, n, endpoint=False, dtype=np.float32)
        retraso = periodo * (1.0 - np.arange(n, dtype=np.float32) / n) * realismo
        d = self.rayos(x, y, theta, ang - omega * retraso, alcance)
        visible = d < alcance - 1e-3

        if rng is not None and realismo > 0:
            # Ruido que crece con el cuadrado de la distancia (ver arriba).
            sigma = (LIDAR_RUIDO_M + LIDAR_RUIDO_CUBICO * d * d * d) * realismo * self.lidar_peor
            d = d + rng.normal(0, 1, d.shape).astype(np.float32) * sigma
            # Colas: unas pocas lecturas se van 1-10 cm.
            atipico = rng.random(d.shape) < LIDAR_ATIPICOS * realismo
            if atipico.any():
                d = d + np.where(atipico, rng.laplace(0, LIDAR_ATIPICO_ESCALA_M, d.shape), 0).astype(np.float32)
            # Píxeles mezclados: donde dos rayos vecinos ven cosas a
            # profundidades muy distintas, el de en medio cae a mitad camino.
            salto = np.abs(np.diff(d, append=d[:1])) > 0.35
            mezcla = salto & (rng.random(d.shape) < LIDAR_MEZCLA_BORDE * realismo)
            if mezcla.any():
                vecino = np.roll(d, -1)
                d = np.where(mezcla, (d + vecino) / 2, d)
            # Rayos perdidos, más frecuentes cuanto más lejos.
            p_perdida = (LIDAR_PERDIDA_BASE + (LIDAR_PERDIDA_LEJOS - LIDAR_PERDIDA_BASE)
                         * np.clip(d / LIDAR_PERDIDA_RAMPA_M, 0, 1))
            visible &= rng.random(d.shape) > p_perdida * realismo * self.lidar_peor
            # El propio robot, que tapa lo que haya detrás.
            grados = np.degrees(ang)
            for a0, a1, dist, prob in LIDAR_AUTOIMPACTOS:
                tapa = (grados >= a0) & (grados <= a1) & (rng.random(d.shape) < prob)
                if tapa.any():
                    d = np.where(tapa, dist + rng.normal(0, 0.002, d.shape), d).astype(np.float32)
                    visible |= tapa

        return [{"angle_deg": float(math.degrees(a)), "distance_m": float(r)}
                for a, r in zip(ang[visible], d[visible])]

    # ------------------------------------------------------------- geometría
    def distancia_al_sendero(self, px, py):
        """Distancia (m) de cada punto al eje del sendero. Vectorizado."""
        if len(self.sendero) < 2:
            return np.full(np.shape(px), 1e3, np.float32)
        a = self.sendero[:-1]
        b = self.sendero[1:]
        ab = b - a
        largo2 = np.maximum((ab ** 2).sum(axis=1), 1e-9)
        ap_x = px[..., None] - a[:, 0]
        ap_y = py[..., None] - a[:, 1]
        t = np.clip((ap_x * ab[:, 0] + ap_y * ab[:, 1]) / largo2, 0, 1)
        dx = ap_x - t * ab[:, 0]
        dy = ap_y - t * ab[:, 1]
        return np.sqrt(dx * dx + dy * dy).min(axis=-1)

    def clases_suelo(self, px, py, rng=None, con_plantas=True):
        """Clasifica cada punto del suelo (ver TIERRA, PAJA, PLANTA…).

        Las texturas imitan lo que se ve en las fotos de la huerta: paja
        fibrosa a los lados, pajas sueltas y piedras sobre el camino, y
        rastrojo ralo más allá de las bandas. Todo es ruido DETERMINISTA
        según la posición, así que el terreno no “hierve” entre fotogramas:
        es el mismo suelo visto desde otro sitio."""
        d = self.distancia_al_sendero(px, py)
        media = self.ancho_sendero / 2
        # Fibras de la paja: ruido estirado, para que se vean hebras y no puntos.
        fibra = ruido(px * 4.0 + py * 9.0, py * 4.0, 7.0, 2)
        grano = ruido(px, py, 55.0, 2)              # granulado fino del suelo
        mata = ruido(px, py, 3.5, 2)                # matas y calvas grandes
        if rng is not None:
            fibra = fibra + rng.normal(0, 0.06, fibra.shape)

        en_banda = (d > media) & (d < media + self.ancho_paja)
        clase = np.full(np.shape(px), TIERRA, np.int8)
        # Dentro de la banda el borde es denso; se ralea hacia los extremos.
        borde = np.clip((d - media) / max(self.ancho_paja, 1e-3), 0, 1)
        denso = en_banda & (fibra > 0.34 + 0.22 * np.abs(borde - 0.5))
        if self.borde == "tierra":
            # El surco delimitado por un caballón de tierra removida: ni paja
            # ni verde, solo otro tono de tierra.
            clase = np.where(en_banda, BORDE_TIERRA, clase)
        elif self.borde == "paja_en_camino":
            # El caso al revés: el CAMINO está cubierto de paja y los bordes
            # son tierra desnuda. Aquí el criterio "paja = borde" se equivoca.
            clase = np.where(d <= media, PAJA, clase)
            clase = np.where(en_banda, BORDE_TIERRA, clase)
        elif self.borde == "vegetacion":
            clase = np.where(denso, HIERBA, clase)
        elif self.borde == "mixto":
            # Franjas alternas de paja y hierba, como cuando el surco tiene
            # rastrojo por un lado y maleza crecida por el otro.
            verde = mata > 0.5
            clase = np.where(denso & verde, HIERBA, clase)
            clase = np.where(denso & ~verde, PAJA, clase)
        else:
            clase = np.where(denso, PAJA, clase)
        # Sobre el camino: alguna paja suelta y alguna piedra.
        clase = np.where((d <= media) & (fibra > 0.80), PAJA, clase)
        clase = np.where((d <= media) & (grano > 0.90) & (mata > 0.5), PIEDRA, clase)
        # Más allá de la banda: rastrojo ralo con matas de hierba.
        fuera = d >= media + self.ancho_paja
        clase = np.where(fuera & (fibra > 0.55), PAJA, clase)
        clase = np.where(fuera & (mata > 0.62), HIERBA, clase)
        if not con_plantas:
            return clase
        for cx, cy, r, _, altura in self.plantas:
            dentro = (px - cx) ** 2 + (py - cy) ** 2 < r * r
            # Las altas tapan más suelo visto desde arriba (la copa) y se
            # pintan más oscuras; las bajas, como manchas claras.
            clase = np.where(dentro, PLANTA_ALTA if altura >= ALTURA_LIDAR_M else PLANTA, clase)
        return clase

    def sombras(self, px, py):
        """Sombras suaves con penumbra en el suelo."""
        sombra = np.zeros(np.shape(px), np.float32)
        acimut = math.radians(self.sol_acimut)
        largo = 1.0 / max(math.tan(math.radians(self.sol_altura)), 0.2)
        dx, dy = math.cos(acimut) * largo, math.sin(acimut) * largo
        altas = self.plantas_altas
        piezas = [(o[0], o[1], o[2], o[3]) for o in self.objetos] + \
                 [(p[0], p[1], p[2], p[4]) for p in altas]
        for cx, cy, r, altura in piezas:
            sx, sy = cx + dx * altura, cy + dy * altura
            r_ext = r * 1.45
            d2 = (px - sx) ** 2 + (py - sy) ** 2
            nucleo = np.clip(1.0 - np.sqrt(d2) / r_ext, 0, 1)
            sombra = np.maximum(sombra, nucleo)
        return sombra

    # ------------------------------------------------------- terreno pintado
    def raster_suelo(self):
        """El terreno visto desde arriba, pintado UNA vez (BGR + sombra en A).

        La cámara antes clasificaba y pintaba cada píxel en cada fotograma,
        con un recorrido de la imagen entera por cada planta y cada sombra:
        640-840 ms por imagen. El suelo no cambia durante el episodio, así
        que se pinta aquí a unos milímetros por píxel y cada fotograma solo
        lo proyecta. Devuelve ([(textura, m_por_px), ...], (x0, y1)): tres
        niveles de detalle para que lo lejano no parpadee."""
        clave = (tuple(round(float(v), 3) for v in self.limites), round(self.seco, 3),
                 self.borde, round(float(self.ancho_sendero), 3),
                 round(float(self.ancho_paja), 3), hash(self.sendero.tobytes()),
                 hash(self.plantas.tobytes()), hash(self.objetos.tobytes()),
                 self.superficie, round(self.sol_acimut, 1), round(self.sol_altura, 1))
        cache = getattr(self, "_cache_raster", None)
        if cache is not None and cache[0] == clave:
            return cache[1]
        import cv2

        x0, y0, x1, y1 = self.limites
        x0, y0 = x0 - RASTER_MARGEN_M, y0 - RASTER_MARGEN_M
        x1, y1 = x1 + RASTER_MARGEN_M, y1 + RASTER_MARGEN_M
        res = max(RASTER_M_POR_PX, math.sqrt((x1 - x0) * (y1 - y0) / RASTER_MAX_PX))
        ancho, alto = int(math.ceil((x1 - x0) / res)), int(math.ceil((y1 - y0) / res))
        xs = x0 + (np.arange(ancho, dtype=np.float32) + 0.5) * res
        ys = y1 - (np.arange(alto, dtype=np.float32) + 0.5) * res
        paso = max(1, 300_000 // ancho)

        def recuadro(cx, cy, r):
            c0, c1 = max(0, int((cx - r - x0) / res)), min(ancho, int((cx + r - x0) / res) + 2)
            f0, f1 = max(0, int((y1 - cy - r) / res)), min(alto, int((y1 - cy + r) / res) + 2)
            if c0 >= c1 or f0 >= f1:
                return None
            gx, gy = np.meshgrid(xs[c0:c1], ys[f0:f1])
            return (slice(f0, f1), slice(c0, c1)), gx, gy

        clase = np.empty((alto, ancho), np.int8)
        for f in range(0, alto, paso):
            gx, gy = np.meshgrid(xs, ys[f:f + paso])
            clase[f:f + paso] = self.clases_suelo(gx, gy, con_plantas=False)
        for cx, cy, r, _, altura in self.plantas:
            rq = recuadro(cx, cy, r)
            if rq is not None:
                sl, gx, gy = rq
                clase[sl][(gx - cx) ** 2 + (gy - cy) ** 2 < r * r] = (
                    PLANTA_ALTA if altura >= ALTURA_LIDAR_M else PLANTA)

        img = np.empty((alto, ancho, 4), np.uint8)
        media = self.ancho_sendero / 2
        for f in range(0, alto, paso):
            gx, gy = np.meshgrid(xs, ys[f:f + paso])
            camino = (self.distancia_al_sendero(gx, gy) <= media
                      if self.superficie != "tierra" else None)
            img[f:f + paso, :, :3] = self.pintar(clase[f:f + paso], gx, gy, seco=self.seco,
                                                 camino=camino)
        for (ox, oy, r, _, _), tipo in zip(self.objetos, self.tipos):
            rq = recuadro(ox, oy, r)
            if rq is not None:
                sl, gx, gy = rq
                color = TIPOS_OBJETO.get(tipo, TIPOS_OBJETO["maceta"])["color"]
                img[sl][(gx - ox) ** 2 + (gy - oy) ** 2 < r * r, :3] = color

        # Sombras en el canal alfa: la cámara decide cuánto oscurecer según
        # el realismo pedido, sin tener que volver a pintar el terreno.
        sombra = np.zeros((alto, ancho), np.float32)
        acimut = math.radians(self.sol_acimut)
        largo = 1.0 / max(math.tan(math.radians(self.sol_altura)), 0.2)
        dx, dy = math.cos(acimut) * largo, math.sin(acimut) * largo
        piezas = [(o[0], o[1], o[2], o[3]) for o in self.objetos] + \
                 [(p[0], p[1], p[2], p[4]) for p in self.plantas_altas]
        for cx, cy, r, altura in piezas:
            sx, sy, r_ext = cx + dx * altura, cy + dy * altura, r * 1.45
            rq = recuadro(sx, sy, r_ext)
            if rq is not None:
                sl, gx, gy = rq
                vista = sombra[sl]
                np.maximum(vista, np.clip(1.0 - np.hypot(gx - sx, gy - sy) / r_ext, 0, 1),
                           out=vista)
        img[..., 3] = (sombra * 255).astype(np.uint8)

        niveles = [(img, res)]
        for _ in range(2):
            niveles.append((cv2.pyrDown(niveles[-1][0]), niveles[-1][1] * 2))
        resultado = (niveles, (x0, y1))
        self._cache_raster = (clave, resultado)
        return resultado

    def _suelo_proyectado(self, px, py, g, realismo):
        """Muestrea el terreno pintado en los puntos de suelo (px, py).

        El nivel de detalle se elige por la huella de cada píxel en el suelo,
        como el mipmapping de una GPU: a lo lejos un píxel cubre decenas de
        centímetros y muestrear el nivel fino daría un parpadeo que el
        detector de senderos se tomaría por textura."""
        import cv2

        niveles, (x0, y1) = self.raster_suelo()
        res0 = niveles[0][1]
        pesos = g.get("pesos_lod")
        if pesos is None or pesos[0] != res0:
            huella = g["dist"] ** 1.5 / (g["fx"] * math.sqrt(max(g["h"], 0.05)))
            lod = np.clip(np.log2(np.maximum(huella / res0, 1.0)), 0, len(niveles) - 1)
            # Mezcla en dos pasos (0 con 1, y el resultado con 2), cada uno
            # con cv2.blendLinear, que lo hace en C sobre uint8.
            w_a = np.clip(1.0 - lod, 0, 1).astype(np.float32)
            w_b = np.clip(2.0 - lod, 0, 1).astype(np.float32)
            pesos = (res0, w_a, 1 - w_a, w_b, 1 - w_b, bool((w_a < 1).any()), bool((w_b < 1).any()))
            g["pesos_lod"] = pesos
        _, w_a, w_a1, w_b, w_b1, usa_1, usa_2 = pesos

        def muestra(k):
            tex, res = niveles[k]
            mx = (px - x0) * (1.0 / res) - 0.5
            my = (y1 - py) * (1.0 / res) - 0.5
            return cv2.remap(tex, mx.astype(np.float32), my.astype(np.float32),
                             cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)

        mezcla = muestra(0)
        if usa_1:
            mezcla = cv2.blendLinear(mezcla, muestra(1), w_a, w_a1)
            if usa_2:
                mezcla = cv2.blendLinear(mezcla, muestra(2), w_b, w_b1)
        img = mezcla[..., :3].astype(np.float32)
        if realismo > 0:
            img *= (1 - mezcla[..., 3:].astype(np.float32) * (self.sombra_oscuridad * realismo / 255.0))
        return img

    def pintar_objetos(self, img, px, py):
        """Macetas, ramas y demás sobre una imagen ya pintada del suelo."""
        for (ox, oy, r, altura, solido), tipo in zip(self.objetos, self.tipos):
            color = TIPOS_OBJETO.get(tipo, TIPOS_OBJETO["maceta"])["color"]
            dentro = (px - ox) ** 2 + (py - oy) ** 2 < r * r
            if dentro.any():
                img[dentro] = color
        return img

    def pintar(self, clase, px=None, py=None, rng=None, seco=0.0, camino=None):
        """Clases -> imagen BGR con variación de tono dentro de cada clase.

        `seco` (0..1) lleva el terreno de húmedo/oscuro a seco/claro, que es
        la diferencia que rompió el detector de paja en el campo: con el suelo
        seco, paja y tierra tienen el mismo brillo."""
        img = np.empty(clase.shape + (3,), np.float32)
        tierra = np.array(COLOR_TIERRA) * (1 - seco) + np.array(COLOR_TIERRA_SECA) * seco
        paja = np.array(COLOR_PAJA) * (1 - seco) + np.array(COLOR_PAJA_CLARA) * seco
        img[:] = tierra
        img[clase == PAJA] = paja
        img[clase == PLANTA] = COLOR_PLANTA
        img[clase == PLANTA_ALTA] = COLOR_PLANTA_ALTA
        img[clase == PIEDRA] = COLOR_PIEDRA
        img[clase == HIERBA] = COLOR_HIERBA
        img[clase == BORDE_TIERRA] = COLOR_BORDE_TIERRA
        if px is not None:
            manchas = ruido(px, py, 0.8, 3) - 0.5
            surcos = ruido(px, py, 14.0, 1) - 0.5
            micro = ruido(px, py, 42.0, 1) - 0.5
            img *= (1 + 0.22 * manchas + 0.10 * surcos + 0.06 * micro)[..., None]
            if seco < 0.5:
                charcos = ruido(px * 0.6, py * 0.6, 1.2, 2)
                mojado = charcos > 0.72
                img[mojado] *= 0.78
            if self.superficie == "baldosa":
                self._pintar_baldosa(img, clase, px, py)
            elif camino is not None and self.superficie != "tierra":
                self._pintar_superficie(img, clase, px, py, camino)
        if rng is not None:
            img += rng.normal(0, 7, img.shape)
        return np.clip(img, 0, 255)

    def _pintar_baldosa(self, img, clase, px, py):
        """Suelo de interior (2026-10-24): baldosas de 50 cm con junta oscura, damero de dos tonos y mota fina. Cubre tierra, paja,
        hierba y borde; deja plantas y piedras (macetas y obstáculos) como están."""
        zona = (clase == TIERRA) | (clase == PAJA) | (clase == HIERBA) | (clase == BORDE_TIERRA)
        if not zona.any():
            return
        lado = 0.5
        i, j = np.floor(px / lado).astype(int), np.floor(py / lado).astype(int)
        damero = ((i + j) % 2).astype(np.float32)
        fx, fy = np.abs(px / lado - np.round(px / lado)), np.abs(py / lado - np.round(py / lado))
        junta = (np.minimum(fx, fy) * lado) < 0.006
        base = np.array((172, 176, 182), np.float32)                      # BGR: gris cálido
        tono = 1 + 0.07 * (damero - 0.5) + 0.10 * (ruido(px, py, 55.0, 1) - 0.5) + 0.04 * (ruido(px, py, 3.0, 2) - 0.5)
        color = base * tono[..., None]
        color[junta] = np.array((96, 100, 104), np.float32)
        img[zona] = color[zona]

    def _pintar_superficie(self, img, clase, px, py, camino):
        """Repinta la tierra DEL CAMINO según `self.superficie`. Lo que no es
        tierra (paja suelta, piedras) se deja: en el campo también está."""
        zona = camino & (clase == TIERRA)
        if not zona.any():
            return
        grueso = ruido(px, py, 2.2, 2)
        fino = ruido(px, py, 60.0, 1)
        if self.superficie == "grava":
            base = np.array(COLOR_GRAVA, np.float32)
            tono = (1 + 0.55 * (fino - 0.5) + 0.12 * (grueso - 0.5))[..., None]
            color = base * tono
        elif self.superficie == "cesped":
            calva = ruido(px * 1.3, py * 1.3, 3.0, 2) > 0.62
            base = np.where(calva[..., None], np.array(COLOR_TIERRA, np.float32),
                            np.array(COLOR_CESPED, np.float32))
            hebra = ruido(px * 3.0, py * 9.0, 30.0, 1)
            color = base * (1 + 0.25 * (hebra - 0.5) + 0.15 * (grueso - 0.5))[..., None]
        elif self.superficie == "hojarasca":
            hoja = ruido(px + 7.1, py - 3.3, 18.0, 2) > 0.55
            base = np.where(hoja[..., None], np.array(COLOR_HOJARASCA, np.float32),
                            np.array(COLOR_TIERRA, np.float32) * 0.9)
            color = base * (1 + 0.30 * (fino - 0.5) + 0.2 * (grueso - 0.5))[..., None]
        else:   # barro
            charco = ruido(px * 0.9, py * 0.9, 1.6, 2) > 0.64
            base = np.array(COLOR_BARRO, np.float32) * (1 + 0.18 * (grueso - 0.5))[..., None]
            color = np.where(charco[..., None], base * 0.62 + np.array(COLOR_BRUMA, np.float32) * 0.12,
                             base)
        img[zona] = color[zona]

    # ---------------------------------------------------------------- cámara
    def _geometria_camara(self, geo, ancho, alto, cabeceo_deg):
        """Todo lo que depende solo de la cámara, no de dónde está el robot:
        a qué punto del suelo (en ejes del robot) mira cada píxel, la niebla,
        el cielo y el degradado de las paredes. Se guarda por geometría; la
        inclinación total se redondea a 0,25° (su efecto a 1 m es < 5 mm).
        Tiene que ser la TOTAL: la que llega ya trae el cabeceo compensado
        por el IMU con decimales, y redondeando solo el cabeceo la caché no
        acertaba nunca."""
        incl = round((float(geo["camara_inclinacion_deg"]) + cabeceo_deg) * 4) / 4
        clave = (ancho, alto, round(float(geo["camara_altura_m"]), 4), incl,
                 round(float(geo["camara_hfov_deg"]), 2), round(float(geo["camara_adelante_m"]), 4))
        caches = self.__dict__.setdefault("_cache_geo", {})
        g = caches.get(clave)
        if g is not None:
            return g
        h = float(geo["camara_altura_m"])
        # El cabeceo del chasis (baches, suspensión) mueve la cámara: con 2°
        # de más, lo que la cámara cree que está a 1 m está en realidad a 1,2.
        phi = math.radians(incl)
        base = _base_camara(ancho, alto, round(float(geo["camara_hfov_deg"]), 2))
        fx, xn, yn = base["fx"], base["xn"], base["yn"]
        baja = yn * math.cos(phi) + math.sin(phi)
        suelo = baja > 1e-3
        t = np.where(suelo, h / np.where(suelo, baja, 1.0), 0.0).astype(np.float32)
        xr = (t * (math.cos(phi) - yn * math.sin(phi)) + geo["camara_adelante_m"]).astype(np.float32)
        yr = (-t * xn).astype(np.float32)
        dist = np.hypot(xr, yr).astype(np.float32)
        dist_seg = np.maximum(dist, 0.1)
        niebla = np.clip(1.0 - np.exp(-0.065 * np.maximum(dist - 1.0, 0)), 0, 0.42)[..., None]

        g = {"h": h, "fx": fx, "suelo": suelo, "xr": xr, "yr": yr, "dist": dist,
             "inv_dist": (1.0 / dist_seg).astype(np.float32),
             "dist_seg": dist_seg.astype(np.float32),
             "niebla": niebla.astype(np.float32),
             "bruma_niebla": (np.array(COLOR_BRUMA, np.float32) * niebla).astype(np.float32),
             "cielo": base["cielo"], "no_suelo": ~suelo, "pared_fila": base["pared_fila"],
             "angulos": base["angulos"]}
        if len(caches) > 32:
            caches.pop(next(iter(caches)))
        caches[clave] = g
        return g

    def camara(self, x, y, theta, geo, ancho=640, alto=480, rng=None,
               velocidad=0.0, realismo=1.0, cabeceo_deg=0.0, omega=0.0):
        """Imagen BGR sintética desde la cámara del robot.

        Cada píxel se proyecta al suelo con la misma geometría que usa
        `sendero.py` (altura, inclinación, campo y adelantamiento de la
        cámara). Lo que cae por encima del horizonte, o más allá de la
        primera pared en ese rumbo, se pinta como pared o cielo."""
        global _GPU
        rg = _render_gpu()
        if rg is not None:
            try:
                return rg.render(self, x, y, theta, geo, ancho, alto, rng, velocidad,
                                 realismo, cabeceo_deg, omega)
            except Exception as exc:
                # Una vez y para siempre: reintentar cada fotograma llenaría
                # el log y podría dejar memoria de la GPU sin liberar.
                print(f"[camara] la GPU falló ({type(exc).__name__}: {exc}): "
                      f"se sigue en CPU", flush=True)
                _GPU = False
        g = self._geometria_camara(geo, ancho, alto, cabeceo_deg)
        c, s = math.cos(theta), math.sin(theta)
        xr, yr = g["xr"], g["yr"]
        px = x + xr * c - yr * s
        py = y + xr * s + yr * c

        img = self._suelo_proyectado(px, py, g, realismo)
        # Niebla exponencial con color cálido de distancia.
        img *= (1 - g["niebla"])
        img += g["bruma_niebla"]

        # Especular del sol sobre el suelo (más en húmedo). La dirección de
        # la vista en el mundo es la del robot girada theta: basta girar el
        # sol al revés y usar los ejes del robot, que están en la caché.
        if realismo > 0 and self.seco < 0.8:
            rel = math.radians(self.sol_acimut) - theta
            altura_sol = math.sin(math.radians(self.sol_altura)) * g["h"] / g["dist_seg"]
            d = np.clip((xr * math.cos(rel) + yr * math.sin(rel)) * g["inv_dist"]
                        + altura_sol, 0, 1)
            d2 = d * d; d4 = d2 * d2; d8 = d4 * d4; d16 = d8 * d8
            img *= (1 + (d16 * d8 * ((1 - self.seco) * 0.18))[..., None])

        # Paredes: gradiente vertical + niebla de distancia.
        if len(self.paredes):
            dist_pared = self.rayos(x, y, theta, g["angulos"], alcance=30.0,
                                    solo_paredes=True).astype(np.float32)
            tapado = g["dist"] > dist_pared[None, :]
            if tapado.any():
                filas, cols = np.nonzero(tapado)
                nf = np.clip(1.0 - np.exp(-0.04 * np.maximum(dist_pared - 2, 0)), 0, 0.5)[cols, None]
                img[filas, cols] = (g["pared_fila"][filas] * (1 - nf)
                                    + np.array(COLOR_BRUMA, np.float32) * nf)

        # Cielo con gradiente de cenit a horizonte y bandas de nubes.
        np.copyto(img, g["cielo"], where=g["no_suelo"][..., None])
        np.clip(img, 0, 255, out=img)
        img = img.astype(np.uint8)
        if realismo > 0:
            img = self._degradar(img, rng, velocidad, realismo, omega, g["fx"])
        return img

    def _degradar(self, img, rng, velocidad, realismo, omega=0.0, fx=None):
        """Le quita a la imagen lo que una cámara real no puede dar.

        Sin esto el detector se entrena contra una imagen perfecta y en el
        campo se encuentra otra cosa: movida al avanzar, con las esquinas
        apagadas y con grano en las zonas oscuras."""
        import cv2

        alto, ancho = img.shape[:2]
        # 1. Movida. Al AVANZAR el suelo corre hacia abajo en la imagen
        #    (arrastre vertical); al GIRAR toda la escena barre de lado
        #    (horizontal, omega × exposición × focal). Antes era solo
        #    horizontal y proporcional a v: justo al revés de lo que pasa.
        px_avance = DESENFOQUE_POR_VELOCIDAD * abs(velocidad) * realismo * (ancho / 320)
        if px_avance > 1.2:
            img = cv2.blur(img, (1, int(min(21, px_avance)) | 1))
        if fx is not None:
            px_giro = abs(omega) * EXPOSICION_CAMARA_S * fx * realismo
            if px_giro > 1.2:
                img = cv2.blur(img, (int(min(31, px_giro)) | 1, 1))
        cam = self.camara_params
        # 2. Exposición y balance de blancos del episodio.
        ganancia = float(cam.get("ganancia", 1.0))
        balance = cam.get("balance", (1.0, 1.0, 1.0))
        if ganancia != 1.0 or tuple(balance) != (1.0, 1.0, 1.0):
            img = cv2.multiply(img, tuple(float(ganancia * b) for b in balance) + (1.0,))
        # 3. Viñeteo del gran angular.
        vineta = VIÑETEO * float(cam.get("vineta", 1.0))
        if vineta > 0:
            clave = (alto, ancho, round(realismo, 3), round(vineta, 3))
            if getattr(self, "_vineta", (None,))[0] != clave:
                yy, xx = np.mgrid[0:alto, 0:ancho]
                r = np.hypot((xx - ancho / 2) / (ancho / 2), (yy - alto / 2) / (alto / 2))
                factor = 1 - realismo * vineta * np.clip(r, 0, 1.4) ** 2
                self._vineta = (clave, cv2.merge([np.clip(factor * 255, 0, 255).astype(np.uint8)] * 3))
            img = cv2.multiply(img, self._vineta[1], scale=1 / 255)
        # 4. Ruido: mucho más visible en lo oscuro, como en cualquier sensor.
        #    Se genera a media resolución y se amplía: el grano queda algo más
        #    grueso (que es lo que hace una cámara con ganancia alta) y cuesta
        #    la cuarta parte, que importa en tandas de miles de episodios.
        if rng is not None:
            medio = (alto // 2, ancho // 2)
            grano = cv2.resize(rng.standard_normal(medio + (3,), dtype=np.float32),
                               (ancho, alto), interpolation=cv2.INTER_LINEAR)
            luz = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)[..., None]
            sigma = ((RUIDO_EN_SOMBRA - (RUIDO_EN_SOMBRA - RUIDO_BASE) / 255.0 * luz)
                     * realismo * float(cam.get("ruido", 1.0)))
            grano *= sigma
            grano += img
            img = np.clip(grano, 0, 255).astype(np.uint8)
        # 5. Compresión: la NexiGo entrega MJPEG, y sus bloques de 8x8 y el
        #    color a media resolución llegan tal cual al detector.
        calidad = int(cam.get("jpeg", 0))
        if calidad > 0:
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, calidad])
            if ok:
                img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return img

    # ------------------------------------------------------------ vista cenital
    def dibujo(self, px_por_m=60):
        """Vista desde arriba del mundo (BGR), para ver los episodios.

        Se cachea: la textura del terreno no cambia, y generarla en cada
        fotograma de la vista 3D costaba ~150 ms de los 197 que tardaba."""
        clave = (round(px_por_m, 2), round(self.seco, 3), len(self.plantas),
                 len(self.objetos), self.borde)
        cache = getattr(self, "_cache_dibujo", None)
        if cache is not None and cache[0] == clave:
            return cache[1]
        img = self._dibujar(px_por_m)
        self._cache_dibujo = (clave, img)
        return img

    def _dibujar(self, px_por_m):
        x0, y0, x1, y1 = self.limites
        ancho = int((x1 - x0) * px_por_m)
        alto = int((y1 - y0) * px_por_m)
        gx, gy = np.meshgrid(np.linspace(x0, x1, ancho, dtype=np.float32),
                             np.linspace(y1, y0, alto, dtype=np.float32))
        clase = self.clases_suelo(gx, gy)
        img = self.pintar(clase, gx, gy, seco=self.seco)
        return self.pintar_objetos(img, gx, gy).astype(np.uint8)

    def a_pixel(self, x, y, px_por_m=60):
        x0, y0, x1, y1 = self.limites
        return int((x - x0) * px_por_m), int((y1 - y) * px_por_m)


    # ------------------------------------------------------------ persistencia
    def a_dict(self) -> dict:
        return {"nombre": self.nombre, "paredes": self.paredes.tolist(),
                "sendero": self.sendero.tolist(), "ancho_sendero": self.ancho_sendero,
                "ancho_paja": self.ancho_paja, "plantas": self.plantas.tolist(),
                "escaneos": self.escaneos.tolist(), "seco": self.seco,
                "borde": self.borde, "superficie": self.superficie,
                "objetos": [{"tipo": t, "x": float(o[0]), "y": float(o[1]),
                             "radio": float(o[2]), "altura": float(o[3]),
                             "solido": bool(o[4])}
                            for o, t in zip(self.objetos, self.tipos)],
                "limites": list(self.limites)}

    @staticmethod
    def desde_dict(d: dict) -> "Mundo":
        return Mundo(d.get("nombre", "dibujado"), d.get("paredes", []), d.get("sendero", []),
                     float(d.get("ancho_sendero", 0.65)), float(d.get("ancho_paja", 0.35)),
                     d.get("plantas", []), tuple(d.get("limites", (-6, -6, 6, 6))),
                     d.get("escaneos", []), float(d.get("seco", 0.35)),
                     d.get("objetos", []), d.get("borde", "paja"),
                     d.get("superficie", "tierra"))


def desde_ocupacion(pgm: "Path", yaml_texto: str, nombre="mapa_real", sendero=(), plantas=()):
    """Mapa REAL del robot (SLAM) -> mundo del simulador.

    El mapa de slam_toolbox es una rejilla de ocupación en PGM: negro =
    ocupado, blanco = libre, gris = desconocido. Se extraen los CONTORNOS de
    lo ocupado y se simplifican a segmentos, que es lo que el simulador
    entiende por "pared" (lo alto, lo que ve el LIDAR).
    """
    import cv2
    import numpy as np

    datos = {}
    for linea in yaml_texto.splitlines():
        if ":" in linea:
            k, v = linea.split(":", 1)
            datos[k.strip()] = v.strip()
    res = float(datos.get("resolution", 0.05))
    origen = [float(x) for x in datos.get("origin", "[0,0,0]").strip("[]").split(",")[:2]]
    umbral = float(datos.get("occupied_thresh", 0.65))

    img = cv2.imread(str(pgm), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"no se pudo leer {pgm}")
    alto = img.shape[0]
    ocupado = (img < (1.0 - umbral) * 255).astype(np.uint8)
    # Cerrar huecos: el mapa real tiene celdas sueltas que no son paredes.
    ocupado = cv2.morphologyEx(ocupado, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

    paredes = []
    contornos, _ = cv2.findContours(ocupado, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    for c in contornos:
        if cv2.arcLength(c, True) < 6 * 1:          # ruido suelto
            continue
        aprox = cv2.approxPolyDP(c, 1.5, True).reshape(-1, 2)
        if len(aprox) < 2:
            continue
        pts = [(origen[0] + px * res, origen[1] + (alto - py) * res) for px, py in aprox]
        for i in range(len(pts)):
            a_, b_ = pts[i], pts[(i + 1) % len(pts)]
            if math.dist(a_, b_) > 0.08:
                paredes.append((a_[0], a_[1], b_[0], b_[1]))

    xs = [p[i] for p in paredes for i in (0, 2)] or [0.0]
    ys = [p[i] for p in paredes for i in (1, 3)] or [0.0]
    margen = 0.6
    limites = (min(xs) - margen, min(ys) - margen, max(xs) + margen, max(ys) + margen)
    return Mundo(nombre, paredes, sendero, 0.65, 0.35, plantas, limites)


def endurecer(datos: dict, nivel=1.0, semilla=None) -> dict:
    """Versión más difícil de un mapa dibujado, sin tocar el original.

    Qué se endurece, y por qué cada cosa:
      · El sendero se ESTRECHA: menos margen para el mismo chasis, que es lo
        que obliga a afinar el seguimiento en vez de ir a ojo.
      · Más PUNTOS DE ESCANEO, repartidos por el recorrido: el robot tiene
        que llegar a ellos, no solo pasear.
      · Algunas plantas del borde crecen por encima de los 40 cm del LIDAR:
        aparecen en el scan como obstáculos aunque no lo sean.
      · Ramas, macetas y piedras junto al camino, y algún estrechamiento con
        dos paredes cortas enfrentadas (un paso angosto de verdad).
    """
    rng = np.random.default_rng(semilla)
    d = json.loads(json.dumps(datos))          # copia, el original no se toca
    d["nombre"] = f"{datos.get('nombre', 'mapa')}_dificil"
    ruta = np.array(d.get("sendero", []), np.float64).reshape(-1, 2)

    d["ancho_sendero"] = max(0.42, float(d.get("ancho_sendero", 0.65)) * (1 - 0.30 * nivel))
    d["ancho_paja"] = max(0.15, float(d.get("ancho_paja", 0.35)) * (1 - 0.15 * nivel))
    d["borde"] = str(rng.choice(["paja", "mixto", "vegetacion"]))

    if len(ruta) >= 2:
        # Puntos de escaneo repartidos a lo largo del recorrido, a un lado.
        escaneos = list(d.get("escaneos", []))
        cuantos = int(6 + 10 * nivel)
        for k in range(cuantos):
            i = int(k * (len(ruta) - 1) / max(cuantos - 1, 1))
            j = min(i + 1, len(ruta) - 1)
            dx, dy = ruta[j] - ruta[i]
            largo = math.hypot(dx, dy) or 1.0
            lado = 1 if k % 2 == 0 else -1
            off = d["ancho_sendero"] / 2 + float(rng.uniform(0.10, 0.28))
            escaneos.append([float(ruta[i][0] - lado * dy / largo * off),
                             float(ruta[i][1] + lado * dx / largo * off)])
        d["escaneos"] = escaneos

        # Matas altas: las plantas del borde que crecieron.
        plantas = [list(p) + [0.12] if len(p) == 4 else list(p)
                   for p in d.get("plantas", [])]
        for p in plantas:
            if rng.random() < 0.18 * nivel:
                p[2] = float(p[2]) * float(rng.uniform(1.6, 2.6))
                p[4] = float(rng.uniform(0.34, 0.65))       # por encima del LIDAR
        d["plantas"] = plantas

        # Estorbos junto al camino y pasos estrechos.
        objetos = list(d.get("objetos", []))
        for _ in range(int(3 + 5 * nivel)):
            i = int(rng.integers(0, len(ruta)))
            ang = float(rng.uniform(0, 2 * math.pi))
            r = d["ancho_sendero"] / 2 + float(rng.uniform(0.12, 0.5))
            objetos.append({"tipo": str(rng.choice(["rama", "arbusto", "maceta", "piedra"])),
                            "x": float(ruta[i][0] + r * math.cos(ang)),
                            "y": float(ruta[i][1] + r * math.sin(ang))})
        d["objetos"] = objetos

        paredes = list(d.get("paredes", []))
        for _ in range(int(1 + 2 * nivel)):
            i = int(rng.integers(1, max(len(ruta) - 1, 2)))
            dx, dy = ruta[min(i + 1, len(ruta) - 1)] - ruta[i - 1]
            largo = math.hypot(dx, dy) or 1.0
            nx, ny = -dy / largo, dx / largo
            hueco = d["ancho_sendero"] / 2 + 0.06     # deja justo el ancho del robot
            for lado in (1, -1):
                cx = ruta[i][0] + lado * nx * hueco
                cy = ruta[i][1] + lado * ny * hueco
                paredes.append([float(cx - dx / largo * 0.25), float(cy - dy / largo * 0.25),
                                float(cx + dx / largo * 0.25), float(cy + dy / largo * 0.25)])
        d["paredes"] = paredes
    return d


def aleatorio(semilla=None):
    """Escenario al azar: sala, sendero y obstáculos distintos cada vez.

    Entrenar y calibrar siempre en el mismo mapa enseña ese mapa, no a
    conducir. Variar la sala, el ancho del camino, los recodos y los
    obstáculos es lo que hace que lo aprendido sirva en el campo.
    """
    rng = np.random.default_rng(semilla)
    ancho = float(rng.uniform(5, 11))
    alto = float(rng.uniform(4.5, 9))
    paredes = _rect(-ancho / 2, -alto / 2, ancho / 2, alto / 2)

    forma = rng.choice(["bucle", "serpiente", "recto"])
    margen = float(rng.uniform(0.9, 1.5))
    if forma == "bucle":
        x, y = ancho / 2 - margen, alto / 2 - margen
        ruta = [(-x, -y), (x, -y), (x, y), (-x, y), (-x, -y)]
    elif forma == "serpiente":
        calles = int(rng.integers(2, 4))
        x = ancho / 2 - margen
        ruta = []
        for i in range(calles):
            y = -alto / 2 + margen + i * (alto - 2 * margen) / max(calles - 1, 1)
            ruta += [(-x, y), (x, y)] if i % 2 == 0 else [(x, y), (-x, y)]
    else:
        y = float(rng.uniform(-alto / 4, alto / 4))
        ruta = [(-ancho / 2 + margen, y), (ancho / 2 - margen, y)]

    # Obstáculos altos sueltos (cajas): el LIDAR sí los ve.
    for _ in range(int(rng.integers(0, 4))):
        cx = float(rng.uniform(-ancho / 2 + 1, ancho / 2 - 1))
        cy = float(rng.uniform(-alto / 2 + 1, alto / 2 - 1))
        lado = float(rng.uniform(0.3, 0.8))
        paredes += _rect(cx - lado, cy - lado, cx + lado, cy + lado)

    plantas = []
    if forma != "recto" and rng.random() < 0.8:
        separacion = float(rng.uniform(0.55, 0.95))
        for i in range(len(ruta) - 1):
            for lado in (1, -1):
                ax, ay = ruta[i]
                bx, by = ruta[i + 1]
                largo = math.hypot(bx - ax, by - ay)
                if largo < 0.5:
                    continue
                nx, ny = -(by - ay) / largo, (bx - ax) / largo
                plantas += _plantas_en_linea((ax + lado * separacion * nx, ay + lado * separacion * ny),
                                             (bx + lado * separacion * nx, by + lado * separacion * ny),
                                             paso=float(rng.uniform(0.3, 0.6)),
                                             semilla=int(rng.integers(0, 10000)))
    mundo = Mundo(f"aleatorio_{forma}", paredes, ruta, float(rng.uniform(0.5, 0.9)),
                  float(rng.uniform(0.25, 0.5)), plantas,
                  (-ancho / 2 - 0.5, -alto / 2 - 0.5, ancho / 2 + 0.5, alto / 2 + 0.5))
    rumbo = math.atan2(ruta[1][1] - ruta[0][1], ruta[1][0] - ruta[0][0])
    return mundo, (ruta[0][0], ruta[0][1], rumbo)


# ---------------------------------------------------------------------------
# Escenarios. Cada uno devuelve (mundo, pose inicial) y entrena algo distinto.
# ---------------------------------------------------------------------------
from pathlib import Path   # noqa: E402  (solo para los tipos de desde_ocupacion)


def _rect(x0, y0, x1, y1):
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def _plantas_en_linea(p0, p1, paso=0.35, radio=0.07, plagas=0.15, semilla=0):
    rng = np.random.default_rng(semilla)
    largo = math.dist(p0, p1)
    n = max(1, int(largo / paso))
    out = []
    for i in range(n + 1):
        f = i / max(n, 1)
        x = p0[0] + (p1[0] - p0[0]) * f + rng.normal(0, 0.03)
        y = p0[1] + (p1[1] - p0[1]) * f + rng.normal(0, 0.03)
        out.append((x, y, radio * rng.uniform(0.7, 1.4), 1.0 if rng.random() < plagas else 0.0))
    return out


def bucle():
    """La huerta del usuario: sendero en bucle alrededor de un cantero central,
    cercado. Entrena seguir el sendero y dar la vuelta completa."""
    paredes = _rect(-4.5, -3.5, 4.5, 3.5)
    ruta = [(-3, -2), (3, -2), (3.4, -1.6), (3.4, 1.6), (3, 2), (-3, 2),
            (-3.4, 1.6), (-3.4, -1.6), (-3, -2)]
    plantas = (_plantas_en_linea((-2.6, -0.9), (2.6, -0.9), semilla=1)
               + _plantas_en_linea((-2.6, 0.9), (2.6, 0.9), semilla=2)
               + _plantas_en_linea((-2.6, 0.0), (2.6, 0.0), semilla=3))
    return Mundo("bucle", paredes, ruta, 0.65, 0.35, plantas, (-5, -4, 5, 4)), (-3.0, -2.0, 0.0)


def hileras():
    """Cultivo en hileras paralelas: entrena centrarse entre dos hileras."""
    paredes = _rect(-5, -4, 5, 4)
    ruta = [(-4.2, 0), (4.2, 0)]
    plantas = []
    for k, yy in enumerate((-0.85, 0.85, -2.2, 2.2)):
        plantas += _plantas_en_linea((-4.2, yy), (4.2, yy), semilla=10 + k)
    return Mundo("hileras", paredes, ruta, 0.7, 0.3, plantas, (-5.5, -4.5, 5.5, 4.5)), (-4.0, 0.0, 0.0)


def pasillo_estrecho():
    """Pasillo de 0,9 m con un giro: entrena no rozar y girar en sitio justo."""
    paredes = [(-4, -0.45, 1.5, -0.45), (-4, 0.45, 0.6, 0.45),
               (0.6, 0.45, 0.6, 3.5), (1.5, -0.45, 1.5, 3.5),
               (-4, -0.45, -4, 0.45), (0.6, 3.5, 1.5, 3.5)]
    ruta = [(-3.6, 0), (1.05, 0), (1.05, 3.2)]
    return Mundo("pasillo", paredes, ruta, 0.7, 0.2, [], (-4.5, -1.5, 2.5, 4.5)), (-3.5, 0.0, 0.0)


def pasillo_real():
    """El pasillo REAL donde se probó el robot el 2026-10-02 (plano del LIDAR
    y fotos de la cámara, logs/sesion_20261002_manejo).

    Pasillo de ~65 cm (el robot mide 35: ~15 cm por lado) que desemboca en
    un cuarto de ~2 × 1,3 m, con un poste de madera a la derecha de la salida
    y objetos BAJOS que el LIDAR no ve y la cámara sí: una pesa y una chancla
    (la cobija se retiró). Es lo que más falló de verdad: el freno marcaba
    1-2 m libres con la pesa a 60 cm. Girar sobre el eje dentro del pasillo
    roza las paredes; hay que salir al cuarto para girar."""
    paredes = [
        (-0.7, 0.32, 1.1, 0.32),      # pared izquierda del pasillo
        (-0.7, -0.33, 1.0, -0.33),    # pared derecha del pasillo
        (-0.7, -0.33, -0.7, 0.32),    # fondo del pasillo (detrás del robot)
        (1.1, 0.32, 1.1, 1.45),       # el cuarto se abre a la izquierda
        (1.1, 1.45, 2.35, 1.45),
        (2.35, 1.45, 2.35, -0.65),    # pared del fondo del cuarto (la de la pesa)
        (2.35, -0.65, 1.0, -0.65),    # puerta cerrada a la derecha
        (1.0, -0.65, 1.0, -0.33),
    ]
    objetos = [
        {"tipo": "maceta", "x": 0.95, "y": -0.27, "radio": 0.05, "altura": 1.5},   # poste de madera
        {"tipo": "maceta", "x": 2.1, "y": 0.75, "radio": 0.16, "altura": 0.35},    # balde
        {"tipo": "piedra", "x": 1.85, "y": 0.2, "radio": 0.13, "altura": 0.14},    # pesa (invisible al LIDAR)
        # La cobija (1,35, 0,05) la retiró el usuario al pasar; no está.
        {"tipo": "planta", "x": 1.55, "y": 0.45, "radio": 0.08, "altura": 0.03},   # chancla (se pisa)
    ]
    # Sale RECTO del pasillo y gira ya en el cuarto: girando antes, la
    # carrocería corta la esquina de la salida (el freno no deja pasar).
    # Al final, a media distancia entre la pared izquierda del cuarto
    # (x = 1,1) y el balde: antes acababa a 35 cm de la pared y rozaba.
    # En el pasillo, 5 cm a la izquierda del centro: por el centro el borde
    # del poste queda a 22 cm del eje, dentro del corredor del robot (en el
    # robot real pasó a 2 cm).
    ruta = [(-0.4, 0.05), (1.42, 0.05), (1.48, 0.75)]
    return Mundo("pasillo_real", paredes, ruta, 0.45, 0.0, [], (-1.0, -1.0, 2.6, 1.8),
                 objetos=objetos), (-0.4, 0.0, 0.0)


def callejon_sin_salida():
    """Bolsillo cerrado: entrena reconocer que no hay salida y volver atrás."""
    paredes = (_rect(-5, -3, 5, 3)
               + [(0.5, -3, 0.5, 1.2), (2.5, -3, 2.5, 1.2), (0.5, 1.2, 2.5, 1.2)])
    ruta = [(-4, -1.5), (1.5, -1.5), (1.5, 0.8)]
    return Mundo("callejon", paredes, ruta, 0.7, 0.25, [], (-5.5, -3.5, 5.5, 3.5)), (-4.0, -1.5, 0.0)


def trampa_bucle():
    """Sala con un obstáculo central simétrico: el clásico sitio donde una
    política reactiva se queda dando vueltas. Entrena detectar el bucle."""
    paredes = _rect(-3.5, -3.5, 3.5, 3.5) + _rect(-0.8, -0.8, 0.8, 0.8)
    ruta = [(-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (-2.5, 2.5), (-2.5, -2.5)]
    return Mundo("trampa_bucle", paredes, ruta, 0.8, 0.3, [], (-4, -4, 4, 4)), (-2.5, -2.5, 0.0)


def validar(m: "Mundo") -> list[dict]:
    """Problemas de un mapa que hacen inútil entrenar en él.

    Es la comprobación que encontró el invernadero roto (el sendero
    atravesaba 4 paredes y el robot daba vueltas en la entrada) y las
    plantas pisadas por el recorrido ideal en serpentina y campo abierto."""
    problemas = []
    ruta = [tuple(map(float, p)) for p in m.sendero]
    paredes = [tuple(map(float, w)) for w in m.paredes]
    if len(ruta) < 2:
        return [{"nivel": "error", "texto": "el sendero necesita al menos dos puntos"}]

    def cruza(p1, p2, p3, p4):
        d = lambda a, b, c: (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0])
        return d(p3, p4, p1) * d(p3, p4, p2) < 0 and d(p1, p2, p3) * d(p1, p2, p4) < 0

    def dist_seg(p, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
        return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)

    for i, (a, b) in enumerate(zip(ruta, ruta[1:])):
        for w in paredes:
            if cruza(a, b, w[:2], w[2:]):
                x = (a[0] + b[0]) / 2; y = (a[1] + b[1]) / 2
                problemas.append({"nivel": "error", "x": round(x, 2), "y": round(y, 2),
                                  "texto": f"el tramo {i + 1} del sendero atraviesa una pared"})
    radio = 0.20                     # medio ancho del chasis (35 cm) + margen
    for i, (a, b) in enumerate(zip(ruta, ruta[1:])):
        for k in range(11):
            p = (a[0] + (b[0] - a[0]) * k / 10, a[1] + (b[1] - a[1]) * k / 10)
            cerca = min((dist_seg(p, w[:2], w[2:]) for w in paredes), default=9)
            if cerca < radio:
                problemas.append({"nivel": "aviso", "x": round(p[0], 2), "y": round(p[1], 2),
                                  "texto": f"el sendero pasa a {cerca * 100:.0f} cm de una pared: "
                                           f"el robot (35 cm de ancho) no cabe"})
                break
    tramos = list(zip(ruta, ruta[1:]))
    pisadas = [p for p in m.plantas
               if min(dist_seg((p[0], p[1]), a, b) for a, b in tramos) < m.ancho_sendero / 2]
    if pisadas:
        problemas.append({"nivel": "aviso", "texto": f"{len(pisadas)} plantas caen sobre el "
                          "sendero: el recorrido ideal las pisaría", "plantas_sobre_sendero": len(pisadas)})
    x0, y0, x1, y1 = m.limites
    fuera = [p for p in ruta if not (x0 <= p[0] <= x1 and y0 <= p[1] <= y1)]
    if fuera:
        problemas.append({"nivel": "error", "texto": f"{len(fuera)} puntos del sendero fuera de los límites"})
    return problemas


def _fuera_del_sendero(plantas, ruta, ancho, margen=0.12):
    """Quita las plantas que caen sobre el sendero.

    Las hileras se trazan paralelas a cada tramo, y en las curvas la hilera
    de un tramo invade el siguiente: el recorrido ideal las pisaba y el
    entrenamiento aprendía que pisar plantas es normal."""
    limite = ancho / 2 + margen
    tramos = list(zip(ruta, ruta[1:]))

    def dist(p, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
        return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)

    return [p for p in plantas if min(dist(p, a, b) for a, b in tramos) > limite]


def invernadero():
    """Invernadero con pasillos estrechos entre bancales: entrena navegación
    precisa en espacio confinado con muchas plantas a los lados.

    Los bancales terminan a 0,8 m de los giros: con las paredes hasta el
    borde, el sendero las atravesaba en cada vuelta y el robot se quedaba
    dando vueltas en la entrada del pasillo."""
    paredes = _rect(-6, -4, 6, 4)
    for yc in (-1.5, 0.3, 2.1):
        paredes += [(-3.8, yc - 0.33, 3.8, yc - 0.33), (-3.8, yc + 0.33, 3.8, yc + 0.33)]
    ruta = [(-5.2, -1.5), (3.8, -1.5), (4.6, -1.1), (4.6, -0.1), (3.8, 0.3),
            (-3.8, 0.3), (-4.6, 0.7), (-4.6, 1.7), (-3.8, 2.1), (5.2, 2.1)]
    plantas = []
    for yy in (-2.6, -0.6, 1.2, 3.0):
        plantas += _plantas_en_linea((-3.6, yy), (3.6, yy), paso=0.28, semilla=int(abs(yy * 100)))
    plantas = _fuera_del_sendero(plantas, ruta, 0.50)
    return Mundo("invernadero", paredes, ruta, 0.50, 0.20, plantas,
                 (-6.5, -4.5, 6.5, 4.5), seco=0.15, borde="vegetacion"), (-5.2, -1.5, 0.0)


def serpentina():
    """Sendero sinuoso entre vegetación densa: entrena anticipar curvas y
    corregir rumbo sin perder el sendero."""
    paredes = _rect(-7, -5, 7, 5)
    ruta = [(-5.5, -3), (-3, -3.5), (-0.5, -2), (1.5, 0.5),
            (0, 2.5), (-2.5, 3), (-4.5, 1.5), (-3.5, -0.5),
            (-1, -1.5), (2, -3), (4, -2), (5.5, 0),
            (4.5, 2.5), (2, 3.5), (0, 2)]
    plantas = []
    for k in range(len(ruta) - 1):
        ax, ay = ruta[k]
        bx, by = ruta[k + 1]
        largo = math.hypot(bx - ax, by - ay)
        if largo < 0.3:
            continue
        nx, ny = -(by - ay) / largo, (bx - ax) / largo
        for lado in (1, -1):
            plantas += _plantas_en_linea(
                (ax + lado * 0.7 * nx, ay + lado * 0.7 * ny),
                (bx + lado * 0.7 * nx, by + lado * 0.7 * ny),
                paso=0.35, semilla=int(abs(ax * 100 + lado * 50)))
    plantas = _fuera_del_sendero(plantas, ruta, 0.55)
    return Mundo("serpentina", paredes, ruta, 0.55, 0.40, plantas,
                 (-7.5, -5.5, 7.5, 5.5), seco=0.45, borde="mixto"), (-5.5, -3.0, 0.3)


def campo_abierto():
    """Campo sin paredes: solo hileras de plantas marcan el camino. Entrena
    navegar donde el LIDAR ve poco y la cámara es la guía principal."""
    ruta = [(-6, -2), (6, -2), (6.5, -0.5), (6, 1), (-6, 1),
            (-6.5, 2.5), (-6, 4), (6, 4)]
    plantas = []
    for k in range(len(ruta) - 1):
        ax, ay = ruta[k]
        bx, by = ruta[k + 1]
        largo = math.hypot(bx - ax, by - ay)
        if largo < 0.3:
            continue
        nx, ny = -(by - ay) / largo, (bx - ax) / largo
        for lado in (1, -1):
            off = 0.55
            plantas += _plantas_en_linea(
                (ax + lado * off * nx, ay + lado * off * ny),
                (bx + lado * off * nx, by + lado * off * ny),
                paso=0.30, semilla=int(abs(ax * 77 + ay * 33 + lado * 11)))
    plantas = _fuera_del_sendero(plantas, ruta, 0.60)
    objetos = [{"tipo": "piedra", "x": 2.0, "y": -1.5},
               {"tipo": "rama", "x": -1.5, "y": 1.3},
               {"tipo": "piedra", "x": 4.0, "y": 4.2}]
    return Mundo("campo_abierto", [], ruta, 0.60, 0.30, plantas,
                 (-7.5, -3.5, 7.5, 5.5), seco=0.60, objetos=objetos,
                 borde="paja"), (-6.0, -2.0, 0.0)


ESCENARIOS = {
    "aleatorio": aleatorio,
    "bucle": bucle,
    "hileras": hileras,
    "pasillo": pasillo_estrecho,
    "pasillo_real": pasillo_real,
    "callejon": callejon_sin_salida,
    "trampa_bucle": trampa_bucle,
    "invernadero": invernadero,
    "serpentina": serpentina,
    "campo_abierto": campo_abierto,
}
