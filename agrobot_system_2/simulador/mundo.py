"""
Mundo 2D del simulador: qué hay en el terreno y qué ve cada sensor.
===================================================================

La distinción que da sentido a todo el simulador (y que es el problema real
del robot, medido el 2026-09-19):

  · ALTO (paredes, cercas, taludes, cajas): lo ve el LIDAR, que va a ~30 cm
    del suelo, y frena por él.
  · BAJO (sendero, paja del mulch, plantas chicas): el LIDAR pasa por
    ENCIMA y no lo ve. Solo existe para la cámara.

Así, en el simulador, seguir el sendero obliga a usar la cámara igual que en
el campo, y el LIDAR sirve para no chocar.

Ejes del mundo en metros: x a la derecha, y hacia arriba, ángulos en radianes
con 0 = +x (igual que el mapa del SLAM).
"""
import math

import numpy as np

# Colores BGR, tomados de las fotos de la huerta real del usuario.
COLOR_TIERRA = (78, 96, 112)
COLOR_TIERRA_SECA = (110, 132, 150)     # el terreno del 2026-09-19, seco y claro
COLOR_PAJA = (105, 165, 195)
COLOR_PAJA_CLARA = (140, 190, 215)
COLOR_PLANTA = (50, 150, 70)
COLOR_PLANTA_SECA = (60, 130, 120)
COLOR_PIEDRA = (120, 125, 130)
COLOR_HIERBA = (55, 120, 75)
COLOR_PARED = (120, 120, 125)
COLOR_CIELO = (205, 195, 185)

# Clases del suelo (las devuelve `clases_suelo`).
TIERRA, PAJA, PLANTA, PIEDRA, HIERBA, BORDE_TIERRA = 0, 1, 2, 3, 4, 5
COLOR_BORDE_TIERRA = (52, 62, 78)      # caballón de tierra removida, ni paja ni verde
COLOR_MACETA = (70, 95, 140)
COLOR_RAMA = (60, 125, 85)

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


class Mundo:
    """Terreno: paredes (LIDAR), sendero con paja (cámara) y plantas."""

    def __init__(self, nombre, paredes=(), sendero=(), ancho_sendero=0.6,
                 ancho_paja=0.35, plantas=(), limites=(-6, -6, 6, 6),
                 escaneos=(), seco=0.35, objetos=(), borde="paja"):
        self.nombre = nombre
        self.paredes = np.array(paredes, dtype=np.float32).reshape(-1, 4)   # x1,y1,x2,y2
        self.sendero = np.array(sendero, dtype=np.float32).reshape(-1, 2)   # polilínea del centro
        self.ancho_sendero = ancho_sendero
        self.ancho_paja = ancho_paja
        self.plantas = np.array(plantas, dtype=np.float32).reshape(-1, 4)   # x,y,radio,plaga(0/1)
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
    def objetos_visibles_lidar(self):
        """Los que llegan a la altura del LIDAR: los ve, sean o no sólidos."""
        if not len(self.objetos):
            return self.objetos
        return self.objetos[self.objetos[:, 3] >= ALTURA_LIDAR_M]

    # ---------------------------------------------------------------- LIDAR
    def rayos(self, x, y, theta, angulos, alcance=8.0):
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
        obj = self.objetos_visibles_lidar
        if len(obj):
            cx, cy, r = obj[:, 0] - x, obj[:, 1] - y, obj[:, 2]
            proy = dx[:, None] * cx[None, :] + dy[:, None] * cy[None, :]
            perp2 = (cx * cx + cy * cy)[None, :] - proy ** 2
            corta = perp2 < r[None, :] ** 2
            dentro = np.sqrt(np.maximum(r[None, :] ** 2 - perp2, 0))
            t_obj = np.where(corta & (proy - dentro > 0.02), proy - dentro, np.inf)
            d = np.minimum(d, t_obj.min(axis=1))
        return np.minimum(d, alcance).astype(np.float32)

    def scan(self, x, y, theta, n=720, alcance=8.0, ruido_m=0.01, rng=None):
        """Scan de 360° como el del RPLIDAR C1: lista de puntos con el ángulo
        en grados relativo al robot (0 = adelante, + = izquierda)."""
        ang = np.linspace(-math.pi, math.pi, n, endpoint=False, dtype=np.float32)
        d = self.rayos(x, y, theta, ang, alcance)
        if ruido_m and rng is not None:
            d = d + rng.normal(0, ruido_m, d.shape).astype(np.float32)
        visible = d < alcance - 1e-3
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

    def clases_suelo(self, px, py, rng=None):
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
        for cx, cy, r, _ in self.plantas:
            clase = np.where((px - cx) ** 2 + (py - cy) ** 2 < r * r, PLANTA, clase)
        return clase

    def pintar_objetos(self, img, px, py):
        """Macetas, ramas y demás sobre una imagen ya pintada del suelo."""
        for (ox, oy, r, altura, solido), tipo in zip(self.objetos, self.tipos):
            color = TIPOS_OBJETO.get(tipo, TIPOS_OBJETO["maceta"])["color"]
            dentro = (px - ox) ** 2 + (py - oy) ** 2 < r * r
            if dentro.any():
                img[dentro] = color
        return img

    def pintar(self, clase, px=None, py=None, rng=None, seco=0.0):
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
        img[clase == PIEDRA] = COLOR_PIEDRA
        img[clase == HIERBA] = COLOR_HIERBA
        img[clase == BORDE_TIERRA] = COLOR_BORDE_TIERRA
        if px is not None:
            # Manchas grandes de humedad y de luz: sin esto el suelo parece papel.
            manchas = ruido(px, py, 0.8, 3) - 0.5
            surcos = ruido(px, py, 14.0, 1) - 0.5          # relieve fino, huellas
            img *= (1 + 0.22 * manchas + 0.10 * surcos)[..., None]
        if rng is not None:
            img += rng.normal(0, 7, img.shape)
        return np.clip(img, 0, 255)

    # ---------------------------------------------------------------- cámara
    def camara(self, x, y, theta, geo, ancho=640, alto=480, rng=None):
        """Imagen BGR sintética desde la cámara del robot.

        Cada píxel se proyecta al suelo con la misma geometría que usa
        `sendero.py` (altura, inclinación, campo y adelantamiento de la
        cámara). Lo que cae por encima del horizonte, o más allá de la
        primera pared en ese rumbo, se pinta como pared o cielo."""
        h = geo["camara_altura_m"]
        phi = math.radians(geo["camara_inclinacion_deg"])
        fx = (ancho / 2) / math.tan(math.radians(geo["camara_hfov_deg"]) / 2)
        u, v = np.meshgrid(np.arange(ancho, dtype=np.float32),
                           np.arange(alto, dtype=np.float32))
        xn, yn = (u - ancho / 2) / fx, (v - alto / 2) / fx
        baja = yn * math.cos(phi) + math.sin(phi)
        suelo = baja > 1e-3
        t = np.where(suelo, h / np.where(suelo, baja, 1.0), 0.0)
        # En ejes del robot y luego al mundo.
        xr = t * (math.cos(phi) - yn * math.sin(phi)) + geo["camara_adelante_m"]
        yr = -t * xn
        px = x + xr * math.cos(theta) - yr * math.sin(theta)
        py = y + xr * math.sin(theta) + yr * math.cos(theta)

        clase = self.clases_suelo(px, py, rng)
        img = self.pintar(clase, px, py, rng, self.seco)
        img = self.pintar_objetos(img, px, py)
        # Lo lejano se ve más claro y con menos contraste (neblina).
        dist = np.hypot(px - x, py - y)
        niebla = np.clip((dist - 1.5) / 8.0, 0, 0.35)[..., None]
        img = img * (1 - niebla) + np.array(COLOR_CIELO, np.float32) * niebla

        # Paredes: un rayo por columna; el suelo más lejano que la pared queda tapado.
        if len(self.paredes):
            ang = np.arctan2(-xn[0], 1.0)          # rumbo relativo de cada columna
            dist_pared = self.rayos(x, y, theta, ang, alcance=30.0)
            dist_suelo = np.hypot(px - x, py - y)
            tapado = dist_suelo > dist_pared[None, :]
            img[tapado] = COLOR_PARED
        img[~suelo] = COLOR_CIELO
        return np.clip(img, 0, 255).astype(np.uint8)

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
                "borde": self.borde,
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
                     d.get("objetos", []), d.get("borde", "paja"))


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


ESCENARIOS = {
    "aleatorio": aleatorio,
    "bucle": bucle,
    "hileras": hileras,
    "pasillo": pasillo_estrecho,
    "callejon": callejon_sin_salida,
    "trampa_bucle": trampa_bucle,
}
