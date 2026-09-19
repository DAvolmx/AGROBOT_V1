"""
Seguir senderos: centrado entre hileras (LIDAR) + sendero por cámara.
====================================================================

Dos medidas independientes del MISMO error —cuánto se ha desviado el robot
del centro del camino— que se combinan con un peso configurable. Las dos en
metros y con el mismo signo (+ = corregir a la izquierda).

  1. LIDAR. Si hay hilera o pared a los dos lados, el robot va centrado
     cuando la distancia izquierda y la derecha son iguales. Con cultivo
     joven NO sirve: el LIDAR va a ~30 cm del suelo y pasa por encima de las
     plantas. Queda para pasillos, invernaderos y cultivo alto.

  2. Cámara. En la huerta de pruebas (fotos del usuario, 2026-09-19) el
     sendero es TIERRA OSCURA desnuda y lo que lo delimita a cada lado es
     PAJA seca (mulch, clara) con pocas plantas chicas. La versión anterior
     seguía el centroide de lo "no verde" y no medía nada. Ahora:
       a. Bordes = vegetación (Excess Green + Otsu, AND rango HSV) OR paja
          (más brillante que el suelo de ESA imagen, tono amarillento). El
          umbral de la paja es adaptativo: sale del propio fotograma, así
          aguanta sol y sombra.
       b. Los bordes se proyectan al suelo (vista cenital, en metros) con la
          geometría de la cámara. En perspectiva los bordes convergen; en el
          suelo son paralelos, y las pajas sueltas sobre el camino no forman
          línea.
       c. Se busca el ángulo que deja los bordes más alineados: el rumbo del
          robot respecto del sendero (hasta ±40°, para seguir las curvas del
          bucle). El perfil lateral, normalizado por cuánto suelo ve la
          cámara a cada distancia, se corta en "borde" / "libre"; el sendero
          es el HUECO libre que contiene la posición anterior del sendero.
          Sus bordes interiores dan centro y ancho reales del camino.
       d. El error se mide MIRADA_M por delante del robot: pesan a la vez el
          desplazamiento y el ir torcido.

La corrección es TODO-O-NADA (girar un pulso o avanzar), no proporcional:
los motores no bajan del 78 % de potencia (PWM 200..255), así que "gira un
poquito" no existe — se gira un pulso corto y se vuelve a medir.
"""
import math

ANCHO_ANALISIS, ALTO_ANALISIS = 320, 240

# Vista cenital: rejilla del suelo en ejes del robot (x adelante, y izquierda).
# Más cerca de X_MIN la cámara no ve el suelo; más lejos de X_MAX un píxel
# cubre demasiado terreno.
CENITAL_X = (0.30, 1.80)
CENITAL_Y = (-1.00, 1.00)
CENITAL_RES_M = 0.01
# El borde inferior de la imagen puede mostrar el propio chasis.
FILA_MAX_IMAGEN = 0.94
# Estadística del suelo (para el umbral de la paja): mitad baja de la imagen.
FILA_SUELO = 0.50

# Vegetación. El suelo desnudo da ExG ~0..0,04; sin este piso Otsu partiría
# la tierra en dos y vería hileras donde no hay nada.
EXG_MIN = 0.06
BRILLO_MIN = 60           # suma R+G+B; en sombras muy oscuras el color es ruido

# Paja: al menos esto más brillante que la mediana del suelo, y amarillenta
# (H de OpenCV 0..180). La saturación mínima deja fuera cielo, plástico y
# papel blancos.
PAJA_BRILLO_REL = 1.25
PAJA_H = (8, 35)
PAJA_S_MIN = 35

# Búsqueda del rumbo del sendero respecto del robot.
ANGULOS_DEG = list(range(-40, 41, 2))
PERFIL_BIN_M = 0.02
MIN_PUNTOS = 60           # celdas de borde mínimas para intentar nada
# Una columna del perfil es "borde" si esta fracción del suelo visible ahí
# es paja/planta (con piso relativo al máximo del perfil).
BORDE_ABS, BORDE_REL = 0.10, 0.45
# Ancho plausible del sendero (m).
ANCHO_MIN_M, ANCHO_MAX_M = 0.20, 1.50
# Si el sendero anterior quedó en un borde, se busca un hueco libre a esta
# distancia como mucho.
BUSQUEDA_M = 0.35

MIRADA_M = 0.6
SUAVIZADO = 0.5           # media móvil exponencial entre medidas


def error_lidar(distancias: dict, cfg: dict) -> float | None:
    """Desvío del centro entre hileras, en metros. None = no hay dos lados.

    Positivo: sobra sitio a la izquierda, hay que corregir a la izquierda.
    """
    izq, der = distancias.get("izquierda"), distancias.get("derecha")
    if izq is None or der is None:
        return None
    limite = cfg["sendero_pared_max_cm"] / 100.0
    if izq > limite or der > limite:
        return None            # un lado está lejísimos: no es un pasillo
    return izq - der


def mascara_vegetacion(cv2, img, cfg):
    """uint8 0/255 con las plantas de una imagen BGR."""
    import numpy as np

    f = img.astype(np.float32)
    suma = f.sum(axis=2)
    exg = (2 * f[..., 1] - f[..., 2] - f[..., 0]) / (suma + 1e-6)
    u8 = np.clip((exg + 0.5) / 1.5 * 255, 0, 255).astype(np.uint8)
    otsu, _ = cv2.threshold(u8, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    umbral = max(otsu / 255 * 1.5 - 0.5, EXG_MIN)
    veg = (exg > umbral) & (suma > BRILLO_MIN)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    tono = cv2.inRange(hsv, (cfg["sendero_verde_h_min"], cfg["sendero_verde_s_min"], 40),
                       (cfg["sendero_verde_h_max"], 255, 255)) > 0
    return cv2.medianBlur(((veg & tono) * 255).astype(np.uint8), 3)


def mascara_paja(cv2, img):
    """uint8 0/255 con la paja/mulch seco: más clara que el suelo, amarillenta."""
    import numpy as np

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    suelo_v = np.ascontiguousarray(v[int(img.shape[0] * FILA_SUELO):])
    otsu, _ = cv2.threshold(suelo_v, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    umbral = max(float(otsu), float(np.median(suelo_v)) * PAJA_BRILLO_REL)
    paja = (v > umbral) & (h >= PAJA_H[0]) & (h <= PAJA_H[1]) & (s >= PAJA_S_MIN)
    # La paja es fibrosa: se cierra en manchas para que cuente como franja.
    m = (paja * 255).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    return cv2.medianBlur(m, 5)


class Cenital:
    """Mapas de remapeo imagen -> vista cenital del suelo, para una geometría.

    Mismo modelo de cámara que odometria_webcam._Geometria: cámara a
    `camara_altura_m`, inclinada `camara_inclinacion_deg` hacia abajo,
    `camara_adelante_m` por delante del centro del robot."""

    CLAVES = ("camara_altura_m", "camara_inclinacion_deg", "camara_hfov_deg", "camara_adelante_m")

    def __init__(self, geo: dict):
        import numpy as np

        self.clave = tuple(geo[k] for k in self.CLAVES)
        h, phi = geo["camara_altura_m"], math.radians(geo["camara_inclinacion_deg"])
        fx = (ANCHO_ANALISIS / 2) / math.tan(math.radians(geo["camara_hfov_deg"]) / 2)
        xs = np.arange(CENITAL_X[1], CENITAL_X[0], -CENITAL_RES_M, dtype=np.float32)   # filas: lejos arriba
        ys = np.arange(CENITAL_Y[1], CENITAL_Y[0], -CENITAL_RES_M, dtype=np.float32)   # columnas: izquierda a la izquierda
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        dx = X - geo["camara_adelante_m"]
        zc = dx * math.cos(phi) + h * math.sin(phi)
        yc = -dx * math.sin(phi) + h * math.cos(phi)
        with np.errstate(divide="ignore", invalid="ignore"):
            self.map_u = (ANCHO_ANALISIS / 2 + fx * (-Y) / zc).astype(np.float32)
            self.map_v = (ALTO_ANALISIS / 2 + fx * yc / zc).astype(np.float32)
        visible = ((zc > 0.05) & (self.map_u >= 0) & (self.map_u < ANCHO_ANALISIS)
                   & (self.map_v >= 0) & (self.map_v < ALTO_ANALISIS * FILA_MAX_IMAGEN))
        self.map_v[~visible] = -1
        self.X, self.Y = X, Y
        # Celdas del suelo que la cámara ve: para normalizar el perfil.
        fv, cv = np.nonzero(visible)
        self.Xv, self.Yv = X[fv, cv], Y[fv, cv]

    def proyectar(self, cv2, img):
        return cv2.remap(img, self.map_u, self.map_v, cv2.INTER_NEAREST,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)


def _hueco(libre, inicio: int, max_salto: int):
    """Tramo libre que contiene `inicio` (o el más cercano a él), como
    (desde, hasta) en índices, o None."""
    n = len(libre)
    if not libre[inicio]:
        cand = [i for i in range(max(0, inicio - max_salto), min(n, inicio + max_salto + 1)) if libre[i]]
        if not cand:
            return None
        inicio = min(cand, key=lambda i: abs(i - inicio))
    a = inicio
    while a > 0 and libre[a - 1]:
        a -= 1
    z = inicio
    while z < n - 1 and libre[z + 1]:
        z += 1
    return a, z


def detectar(cv2, frame, cfg, geo: dict, cenital: "Cenital",
             centro_previo: float = 0.0, ancho_previo: float | None = None) -> dict:
    """Análisis de un fotograma. No guarda estado.

    lateral_m: centro del sendero respecto del centro del robot (+ = a la
    IZQUIERDA). angulo_deg: rumbo del sendero respecto del robot (+ = se va
    hacia la izquierda). error_m: desvío MIRADA_M adelante (+ = corregir a la
    izquierda). bordes_m: (izquierdo, derecho) interiores; None = no visto.
    """
    import numpy as np

    chico = cv2.resize(frame, (ANCHO_ANALISIS, ALTO_ANALISIS), interpolation=cv2.INTER_AREA)
    veg = mascara_vegetacion(cv2, chico, cfg)
    paja = mascara_paja(cv2, chico) if cfg.get("sendero_paja", True) else np.zeros_like(veg)
    suelo = cenital.proyectar(cv2, cv2.bitwise_or(veg, paja))
    filas, cols = np.nonzero(suelo)
    res = {"veg": veg, "paja": paja, "cenital": suelo, "lateral_m": None, "angulo_deg": None,
           "error_m": None, "bordes_m": None, "ancho_m": None, "confianza": 0.0}

    if len(filas) < MIN_PUNTOS:
        res["estado"] = "sin bordes a la vista (ni paja ni plantas)"
        return res

    X, Y = cenital.X[filas, cols], cenital.Y[filas, cols]
    # Lo cercano es más fiable y ocupa más píxeles de imagen por celda.
    peso, peso_v = 1.0 / X, 1.0 / cenital.Xv
    nbins = int(round(4 * CENITAL_Y[1] / PERFIL_BIN_M))
    rango = (2 * CENITAL_Y[0], 2 * CENITAL_Y[1])
    kernel = np.array([1, 2, 3, 2, 1], np.float32) / 9

    mejor = None
    for a in ANGULOS_DEG:
        t = math.radians(a)
        hist, bins = np.histogram(-X * math.sin(t) + Y * math.cos(t),
                                  bins=nbins, range=rango, weights=peso)
        hist = np.convolve(hist, kernel, mode="same")
        nitidez = float((hist ** 2).sum())
        if mejor is None or nitidez > mejor[0]:
            mejor = (nitidez, a, hist, bins)
    _, angulo, hist, bins = mejor
    t = math.radians(angulo)
    cobertura, _ = np.histogram(-cenital.Xv * math.sin(t) + cenital.Yv * math.cos(t),
                                bins=nbins, range=rango, weights=peso_v)
    cobertura = np.convolve(cobertura, kernel, mode="same")
    centros = (bins[:-1] + bins[1:]) / 2
    visto = cobertura > cobertura.max() * 0.05
    densidad = np.where(visto, hist / np.maximum(cobertura, 1e-9), 0.0)

    umbral = max(BORDE_ABS, BORDE_REL * float(densidad[visto].max()))
    libre = (densidad < umbral) & visto
    inicio = int(np.clip(np.searchsorted(centros, centro_previo), 0, nbins - 1))
    tramo = _hueco(libre, inicio, int(BUSQUEDA_M / PERFIL_BIN_M))
    if tramo is None:
        res["estado"] = "bordes por todas partes: no se ve el sendero"
        return res
    a, z = tramo
    # Si el tramo termina donde la cámara deja de ver suelo, ese lado está
    # "abierto": su límite no es un borde real.
    abierto_der = a == 0 or not visto[a - 1]
    abierto_izq = z == nbins - 1 or not visto[z + 1]
    borde_der, borde_izq = float(bins[a]), float(bins[z + 1])

    if not abierto_der and not abierto_izq:
        ancho = borde_izq - borde_der
        if not ANCHO_MIN_M <= ancho <= ANCHO_MAX_M:
            res["estado"] = f"hueco de {ancho*100:.0f} cm: no parece un sendero"
            return res
        medio = (borde_izq + borde_der) / 2
        res["ancho_m"] = ancho
    elif ancho_previo and not abierto_izq:
        medio = borde_izq - ancho_previo / 2
    elif ancho_previo and not abierto_der:
        medio = borde_der + ancho_previo / 2
    else:
        res["estado"] = ("un solo borde visible (falta el ancho del sendero)"
                         if abierto_izq != abierto_der else "sin bordes a los lados")
        return res

    res["bordes_m"] = (None if abierto_izq else borde_izq, None if abierto_der else borde_der)
    res["lateral_m"] = medio
    res["angulo_deg"] = float(angulo)
    res["error_m"] = medio + MIRADA_M * math.sin(t)
    # Confianza: contraste entre los bordes y el sendero.
    dentro = float(densidad[a:z + 1].mean())
    fuera = float(densidad[visto].max())
    res["confianza"] = float(np.clip((fuera - dentro) / (fuera + 1e-9), 0, 1))
    res["estado"] = (f"sendero a {medio*100:+.0f} cm, rumbo {angulo:+.0f}°"
                     + (f", ancho {res['ancho_m']*100:.0f} cm" if res["ancho_m"] else ", un borde + ancho previo"))
    return res


def dibujar(cv2, frame, res) -> "np.ndarray":
    """Cámara con plantas (verde) y paja (amarillo) + vista cenital con el sendero."""
    import numpy as np

    img = cv2.resize(frame, (ANCHO_ANALISIS, ALTO_ANALISIS), interpolation=cv2.INTER_AREA)
    tinte = img.copy()
    tinte[res["paja"] > 0] = (0, 220, 255)
    tinte[res["veg"] > 0] = (0, 255, 0)
    img = cv2.addWeighted(img, 0.5, tinte, 0.5, 0)

    cen = (cv2.cvtColor(res["cenital"], cv2.COLOR_GRAY2BGR) * 0.6).astype(np.uint8)
    hc, wc = cen.shape[:2]

    def a_px(x, y):   # ejes del robot -> píxel de la vista cenital
        return (int((CENITAL_Y[1] - y) / CENITAL_RES_M), int((CENITAL_X[1] - x) / CENITAL_RES_M))

    t = math.radians(res["angulo_deg"] or 0.0)
    d, n = (math.cos(t), math.sin(t)), (-math.sin(t), math.cos(t))

    def linea(c, color, grosor):
        p0 = (CENITAL_X[0] * d[0] + c * n[0], CENITAL_X[0] * d[1] + c * n[1])
        p1 = (CENITAL_X[1] * d[0] + c * n[0], CENITAL_X[1] * d[1] + c * n[1])
        cv2.line(cen, a_px(*p0), a_px(*p1), color, grosor)

    cv2.line(cen, a_px(CENITAL_X[0], 0), a_px(CENITAL_X[1], 0), (255, 255, 255), 1)
    for c in res["bordes_m"] or ():
        if c is not None:
            linea(c, (0, 200, 255), 2)
    if res["lateral_m"] is not None:
        linea(res["lateral_m"], (255, 0, 255), 2)
    cen = cv2.resize(cen, (int(wc * ALTO_ANALISIS / hc), ALTO_ANALISIS))

    lienzo = np.hstack([img, cen])
    cv2.rectangle(lienzo, (0, 0), (lienzo.shape[1], 16), (0, 0, 0), -1)
    cv2.putText(lienzo, res["estado"][:75], (4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                (255, 255, 255), 1)
    return lienzo


class VisionSendero:
    """Sendero por cámara con seguimiento entre fotogramas."""

    def __init__(self):
        self.lateral_m = None       # suavizados
        self.angulo_deg = None
        self.error_m = None
        self.ancho_m = None
        self.confianza = 0.0
        self.estado = "sin datos"
        self._cenital: Cenital | None = None

    def cenital(self, geo) -> Cenital:
        clave = tuple(geo[k] for k in Cenital.CLAVES)
        if self._cenital is None or self._cenital.clave != clave:
            self._cenital = Cenital(geo)
        return self._cenital

    def detectar(self, cv2, frame, cfg, geo) -> dict:
        return detectar(cv2, frame, cfg, geo, self.cenital(geo),
                        centro_previo=self.lateral_m or 0.0, ancho_previo=self.ancho_m)

    def analizar(self, cv2, frame, cfg, geo) -> float | None:
        """Desvío en metros MIRADA_M adelante (+ = corregir a la izquierda)."""
        res = self.detectar(cv2, frame, cfg, geo)
        self.confianza = res["confianza"]
        self.estado = res["estado"]
        if res["ancho_m"] is not None:
            self.ancho_m = res["ancho_m"] if self.ancho_m is None else 0.8 * self.ancho_m + 0.2 * res["ancho_m"]
        if res["error_m"] is None:
            self.lateral_m = self.angulo_deg = self.error_m = None
            return None

        def _ema(previo, nuevo):
            return nuevo if previo is None else SUAVIZADO * nuevo + (1 - SUAVIZADO) * previo

        self.lateral_m = _ema(self.lateral_m, res["lateral_m"])
        self.angulo_deg = _ema(self.angulo_deg, res["angulo_deg"])
        self.error_m = self.lateral_m + MIRADA_M * math.sin(math.radians(self.angulo_deg))
        return self.error_m


class SeguidorSendero:
    """Junta las dos medidas y dice qué hacer: avanzar o corregir el rumbo."""

    def __init__(self, get_config, obtener_frame, camara_al_frente=None, get_geometria=None):
        self._get_config = get_config
        self._obtener_frame = obtener_frame
        # () -> (bool, motivo). Con la cámara girada o escaneando, lo que ve
        # no es lo que hay delante del robot.
        self._camara_al_frente = camara_al_frente or (lambda: (True, ""))
        # Geometría de la cámara (sección `odometria_webcam` de la config).
        self._get_geometria = get_geometria or (lambda: {
            "camara_altura_m": 0.30, "camara_inclinacion_deg": 15.5,
            "camara_hfov_deg": 71, "camara_adelante_m": 0.0})
        self.vision = VisionSendero()
        self.error_lidar = None
        self.error_vision = None
        self.estado = "inactivo"

    def correccion(self, cv2, distancias: dict) -> str | None:
        """Devuelve "izquierda", "derecha" o None (seguir de frente)."""
        cfg = self._get_config()
        if not cfg["sendero_activo"]:
            self.estado = "inactivo"
            self.error_lidar = self.error_vision = None
            return None

        self.error_lidar = error_lidar(distancias, cfg)

        self.error_vision = None
        if cfg["sendero_vision"]:
            al_frente, motivo = self._camara_al_frente()
            dato = self._obtener_frame() if al_frente else None
            if not al_frente:
                self.vision.estado = f"cámara no usable: {motivo}"
            elif dato is not None:
                try:
                    self.error_vision = self.vision.analizar(cv2, dato[0], cfg,
                                                            self._get_geometria())
                except Exception as exc:
                    self.vision.estado = f"error: {type(exc).__name__}"

        tol_m = max(cfg["sendero_tolerancia_cm"] / 100.0, 1e-3)
        peso = cfg["sendero_vision_peso"]

        terminos = []
        if self.error_lidar is not None:
            terminos.append(((1.0 - peso), self.error_lidar / tol_m))
        if self.error_vision is not None:
            terminos.append((peso, self.error_vision / tol_m))

        if not terminos:
            self.estado = "sin referencia (ni hileras ni sendero visible)"
            return None

        total_peso = sum(w for w, _ in terminos) or 1.0
        error = sum(w * e for w, e in terminos) / total_peso   # 1.0 = una tolerancia

        if abs(error) < 1.0:
            self.estado = f"centrado (error {error:+.2f})"
            return None
        self.estado = f"corrigiendo a la {'izquierda' if error > 0 else 'derecha'} (error {error:+.2f})"
        return "izquierda" if error > 0 else "derecha"

    def snapshot(self) -> dict:
        v = self.vision
        return {
            "estado": self.estado,
            "error_lidar_cm": None if self.error_lidar is None else round(self.error_lidar * 100),
            "error_vision_cm": None if v.error_m is None else round(v.error_m * 100),
            "sendero_lateral_cm": None if v.lateral_m is None else round(v.lateral_m * 100),
            "sendero_rumbo_deg": None if v.angulo_deg is None else round(v.angulo_deg),
            "sendero_ancho_cm": None if v.ancho_m is None else round(v.ancho_m * 100),
            "confianza": round(v.confianza, 2),
            "vision": v.estado,
        }
