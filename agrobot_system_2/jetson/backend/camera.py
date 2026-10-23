"""
Captura de cámara e inferencia YOLO, FUERA del event loop de asyncio.

Por qué existe este módulo
--------------------------
Antes todo esto vivía dentro de `video_handler`, un handler async de
aiohttp, y hacía tres llamadas BLOQUEANTES por frame:

    cap.read()      ~32 ms      cv2.imencode()  ~5 ms      YOLO  ~100 ms+

Un `await` no interrumpe una llamada bloqueante: mientras corrían, el event
loop entero estaba parado. Y ese mismo loop es el que manda los comandos UDP
al ESP32-C3 a 20 Hz. Con el modelo YOLO activo, una inferencia de más de
300 ms deja al C3 sin comando fresco y su watchdog dispara el failsafe —
**el robot se frena solo en medio de una maniobra**. No era un problema de
FPS: era un problema de control.

Además `video_handler` hacía `cv2.VideoCapture(0)` POR PETICIÓN: dos
pestañas del dashboard abiertas eran dos capturas compitiendo por la misma
cámara, y la segunda fallaba o le robaba los frames a la primera.

Cómo se resuelve
----------------
Dos hilos y un solo `VideoCapture` compartido:

  * Hilo de CAPTURA: lee, dibuja las últimas cajas conocidas y comprime a
    JPEG. Marca el ritmo del video.
  * Hilo de INFERENCIA: toma el frame más reciente y corre YOLO a su propio
    ritmo. Va aparte para que una inferencia lenta baje la tasa de
    DETECCIÓN pero no la de VIDEO — si compartieran hilo, el video se
    congelaría en cada inferencia.

Los clientes HTTP reciben los frames por una `asyncio.Queue(maxsize=1)` cada
uno: si un cliente no da abasto se le DESCARTA el frame viejo en vez de
encolar. Encolar haría crecer la latencia sin límite y terminaría comiendo
memoria — en video en vivo, un frame viejo no sirve para nada.
"""
import asyncio
import threading
import time

# Periodo mínimo por defecto entre inferencias. No se corre YOLO en cada
# frame: 30 inferencias/s no hacen falta para una plaga en una planta. El
# valor real sale de detection.infer_hz_max (config), por defecto 10 Hz.
INFER_PERIOD_S = 0.1

# Una misma plaga aparece en decenas de frames seguidos. Sin esto se emitía
# un mensaje WebSocket y un toast POR CAJA Y POR FRAME: a 30 FPS eso inunda
# el dashboard y la galería se llena de la misma foto repetida.
DETECTION_COOLDOWN_S = 3.0


def textura_bordes(frame, xyxy) -> float:
    """Densidad de bordes (Canny) sobre los píxeles de vegetación de la caja, con la caja llevada a 256 px de lado mayor.

    2026-10-22 (fotos del operador): pasto/paja ≈ 0,35; coles (con tierra alrededor) 0,06–0,26; baldosa ≈ 0,01. El detector de plagas
    marcaba «col_amarillamiento» en todo el pasto seco y la puerta de vegetación no lo frena (el pasto ES vegetación): este sí."""
    import cv2
    x1, y1, x2, y2 = xyxy
    c = frame[max(0, y1):y2, max(0, x1):x2]
    if c.size == 0 or min(c.shape[:2]) < 4:
        return 0.0
    k = 256.0 / max(c.shape[:2])
    c = cv2.resize(c, (max(8, int(c.shape[1] * k)), max(8, int(c.shape[0] * k))))
    hsv = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    veg = (hsv[..., 0] >= 20) & (hsv[..., 0] <= 95) & (hsv[..., 1] >= 40) & (hsv[..., 2] >= 40)
    if veg.mean() < 0.02:
        return 0.0
    e = cv2.Canny(cv2.GaussianBlur(cv2.cvtColor(c, cv2.COLOR_BGR2GRAY), (3, 3), 0), 60, 150) > 0
    return float(e[veg].mean())


_PROPS_CAMARA = {"auto_exposure": "CAP_PROP_AUTO_EXPOSURE", "exposure": "CAP_PROP_EXPOSURE", "gain": "CAP_PROP_GAIN",
                 "brightness": "CAP_PROP_BRIGHTNESS", "contrast": "CAP_PROP_CONTRAST", "autofocus": "CAP_PROP_AUTOFOCUS",
                 "focus": "CAP_PROP_FOCUS", "backlight": "CAP_PROP_BACKLIGHT"}


def _aplicar_ajustes_camara(cv2, cap, ajustes: dict):
    """Ajustes de exposición/enfoque de la cámara USB, SOLO los que estén en hardware.camara_ajustes (vacío = no toca nada).

    2026-10-22: en interiores el 89 % de las fotos salió borrosa (var. del Laplaciano < 60) y oscura (brillo medio 65/255). Valores
    por probar con tools/ajustar_camara.py (con el backend parado); una propiedad que el driver no acepta se ignora con un aviso."""
    for clave, valor in ajustes.items():
        nombre = _PROPS_CAMARA.get(clave)
        prop = getattr(cv2, nombre, None) if nombre else None
        if prop is None:
            print(f"[camara] ajuste desconocido: {clave}", flush=True)
            continue
        try:
            ok = cap.set(prop, float(valor))
        except Exception as exc:
            ok = False
            print(f"[camara] {clave}: {exc}", flush=True)
        if not ok:
            print(f"[camara] el driver no aceptó {clave}={valor}", flush=True)


def fraccion_vegetacion(frame, xyxy) -> float:
    """Fracción de píxeles verdes/amarillo-verdosos (H 20-95, S>=60, V>=40) dentro de la caja.

    2026-10-22: el detector de plagas (sin clase «fondo») daba col_dano_insecto al 77-95 % sobre pufs, una puerta, un parlante y unos
    zapatos. Esos fondos tienen <0,5 % de vegetación; una hoja, bastante más. Sirve de puerta barata ANTES de avisar y guardar."""
    import cv2
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = xyxy
    x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
    if x2 - x1 < 4 or y2 - y1 < 4:
        return 0.0
    hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
    # S >= 60: la baldosa beige-amarillenta tiene tono de hoja pero poca saturación (fotos del operador, 2026-10-22)
    m = (hsv[..., 0] >= 20) & (hsv[..., 0] <= 95) & (hsv[..., 1] >= 60) & (hsv[..., 2] >= 40)
    return float(m.mean())

# Reintento de apertura de la cámara. No se reintenta en bucle cerrado: si
# está desenchufada, martillar el driver no la trae de vuelta.
REOPEN_DELAY_S = 2.0


class CameraWorker:
    def __init__(self, loop, on_detection, get_config, on_beat):
        self._loop = loop
        # callback(deteccion, frame_o_None) — se invoca YA en el event loop.
        # El frame solo viaja cuando la detección merece guardarse; mandarlo
        # siempre obligaría a copiar la imagen entera en cada aviso.
        self._on_detection = on_detection
        self._get_config = get_config       # devuelve el dict de config vivo
        self._on_beat = on_beat             # marca de vida para /api/health

        self._stop = threading.Event()
        self._hilos = []
        self._lock = threading.Lock()

        self._jpeg = None            # último frame comprimido (bytes)
        self._frame_crudo = None     # último frame sin dibujar, para YOLO y odometría
        self._frame_ts = 0.0         # time.monotonic() de la captura
        self._frame_seq = 0          # sube con cada frame: permite saber si hay uno nuevo
        self.zoom = 1.0              # zoom de la cámara (1 = campo completo, hasta ZOOM_MAX)
        self._res_abierta = (640, 480)
        self._cajas = []             # últimas detecciones dibujables
        self._cajas_ts = 0.0
        self._ultimo_aviso = {}      # clase -> ts del último aviso emitido
        self._hist_clases = []       # clases vistas en los últimos fotogramas (persistencia)

        self._suscriptores = set()   # asyncio.Queue(maxsize=1) por cliente
        self.fps = 0.0
        self.estado = "detenida"
        self.modelo = None
        # Dispositivo de inferencia y contador de fallos. Ver _bucle_inferencia:
        # esta placa tiene una build de PyTorch que a veces revienta al usar
        # CUDA, y reintentar en bucle costaba 4.5 GB de RAM.
        self._device = None
        self._fallos_seguidos = 0
        # Métricas de inferencia. Sin esto no había forma de saber si YOLO
        # estaba corriendo de verdad: el video se ve igual de fluido tanto si
        # la detección funciona como si el hilo murió en silencio.
        self.infer_ms = 0.0
        self.infer_hz = 0.0
        self.detecciones = 0

    # ---------------- API pública (se llama desde el event loop) ----------
    def start(self):
        if self._hilos:
            return
        self._stop.clear()
        for destino, nombre in ((self._bucle_captura, "cam-captura"),
                                (self._bucle_inferencia, "cam-inferencia")):
            h = threading.Thread(target=destino, name=nombre, daemon=True)
            h.start()
            self._hilos.append(h)

    def stop(self):
        self._stop.set()
        for h in self._hilos:
            h.join(timeout=3)
        self._hilos = []
        self.estado = "detenida"

    def set_model(self, modelo):
        """Cambia el modelo YOLO en caliente. `None` = solo video."""
        self.modelo = modelo
        # Cada modelo nuevo merece su propia oportunidad en GPU: puede que el
        # anterior fallara por algo suyo. Se reinicia el estado del respaldo.
        self._device = None          # None = que decida ultralytics
        self._fallos_seguidos = 0
        if modelo is None:
            with self._lock:
                self._cajas = []

    def set_zoom(self, z: float) -> float:
        """Zoom 1.0-ZOOM_MAX. Con zoom > 1 la cámara se abre a 1280x720 y se recorta el centro (detalle real,
        no solo un reescalado); el recorte vuelve a 640x480 para que YOLO y las cajas sigan alineados.
        Solo para inspeccionar con el robot quieto: SLAM/sendero esperan el campo completo (1.0)."""
        self.zoom = max(1.0, min(ZOOM_MAX, float(z)))
        return self.zoom

    def subscribe(self) -> asyncio.Queue:
        q = asyncio.Queue(maxsize=1)
        self._suscriptores.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue):
        self._suscriptores.discard(q)

    def ultimo_frame(self):
        """(frame BGR sin dibujar, instante de captura, número de frame) o None.

        No se copia: el hilo de captura crea un array nuevo por lectura y ya
        no dibuja sobre él, así que quien lo reciba puede leerlo sin carrera
        mientras no lo modifique."""
        with self._lock:
            if self._frame_crudo is None:
                return None
            return self._frame_crudo, self._frame_ts, self._frame_seq

    def snapshot(self) -> dict:
        return {"fps": round(self.fps, 1), "estado": self.estado,
                "clientes": len(self._suscriptores),
                "modelo": self.modelo is not None,
                "infer_ms": round(self.infer_ms, 1),
                "infer_hz": round(self.infer_hz, 1),
                "detecciones": self.detecciones}

    # ---------------- hilo de captura ------------------------------------
    def _abrir(self):
        import cv2
        from pathlib import Path
        if not Path("/dev/video0").exists():
            return None
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        # MJPG antes que el tamaño: en V4L2 el formato condiciona qué
        # resoluciones y tasas se pueden negociar después.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        ancho, alto = (1280, 720) if self.zoom > 1.0 else (640, 480)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, ancho)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, alto)
        self._res_abierta = (ancho, alto)
        cap.set(cv2.CAP_PROP_FPS, 30)
        # Buffer de 1: si el consumidor se atrasa queremos el frame NUEVO,
        # no ir sacando frames viejos de una cola.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        _fijar_framerate_camara()
        _aplicar_ajustes_camara(cv2, cap, (self._get_config().get("hardware") or {}).get("camara_ajustes") or {})
        return cap

    def _bucle_captura(self):
        import cv2
        cap = None
        t_prev = time.perf_counter()
        try:
            while not self._stop.is_set():
                if not self._get_config()["hardware"]["camera_enabled"]:
                    if cap is not None:
                        cap.release(); cap = None
                    self.estado = "deshabilitada"
                    self._stop.wait(0.5)
                    continue

                if cap is None:
                    cap = self._abrir()
                    if cap is None:
                        with self._lock:
                            self._frame_crudo = None
                            self._jpeg = None
                        self.fps = 0.0
                        self.estado = "sin cámara"
                        self._stop.wait(REOPEN_DELAY_S)
                        continue
                    self.estado = "capturando"

                # Cambiar entre 1x (640x480) y zoom (1280x720) pide reabrir la cámara.
                if (self.zoom > 1.0) != (self._res_abierta[0] > 640):
                    cap.release(); cap = None
                    continue

                ok, frame = cap.read()
                if not ok:
                    # Un fallo de lectura suele ser desconexión USB. Se cierra
                    # y se reabre: seguir leyendo de un handle muerto devuelve
                    # False para siempre y el video no vuelve solo.
                    cap.release(); cap = None
                    self.estado = "lectura fallida, reabriendo"
                    continue

                self._on_beat("camera")
                if self.zoom > 1.0:
                    frame = _recorte_zoom(cv2, frame, self.zoom)
                with self._lock:
                    self._frame_crudo = frame
                    self._frame_ts = time.monotonic()
                    self._frame_seq += 1
                    cajas = list(self._cajas)

                # Se dibuja sobre una COPIA. Antes se dibujaba sobre el mismo
                # array guardado como "crudo": YOLO analizaba imágenes con sus
                # propias cajas pintadas encima, y la odometría visual habría
                # seguido esos rectángulos como si fueran el terreno.
                if cajas:
                    frame = frame.copy()

                for c in cajas:
                    x1, y1, x2, y2 = c["xyxy"]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (60, 200, 90), 2)
                    cv2.putText(frame, f"{c['class']} {c['confidence']:.2f}",
                                (x1, max(0, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, (60, 200, 90), 2)

                ok, jpeg = cv2.imencode(".jpg", frame,
                                        [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    continue
                datos = jpeg.tobytes()
                self._jpeg = datos
                self._publicar(datos)

                ahora = time.perf_counter()
                dt = ahora - t_prev
                t_prev = ahora
                if dt > 0:
                    # media móvil: el valor instantáneo salta demasiado para
                    # mostrarlo en pantalla
                    self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt) if self.fps else 1.0 / dt
        finally:
            if cap is not None:
                cap.release()

    def _publicar(self, datos: bytes):
        """Reparte el frame a cada cliente. Corre en el hilo de captura, así
        que todo lo que toque el loop de asyncio va por call_soon_threadsafe."""
        if not self._suscriptores:
            return
        def repartir():
            for q in list(self._suscriptores):
                if q.full():
                    # cliente atrasado: se tira el frame VIEJO, no el nuevo
                    try: q.get_nowait()
                    except asyncio.QueueEmpty: pass
                try: q.put_nowait(datos)
                except asyncio.QueueFull: pass
        self._loop.call_soon_threadsafe(repartir)

    # ---------------- hilo de inferencia ---------------------------------
    def _modelo_maceta(self, cfg):
        """Carga perezosa del detector de macetas (detection.modelo_maceta, en models/). None si no está o falló."""
        nombre = cfg.get("modelo_maceta", "agrobot_maceta_det.pt")
        if not nombre or getattr(self, "_maceta_mod", None) is False:
            return None
        if getattr(self, "_maceta_mod", None) is None or getattr(self, "_maceta_nombre", None) != nombre:
            from pathlib import Path
            ruta = Path(__file__).resolve().parent / "models" / nombre
            if not ruta.exists():
                self._maceta_mod = False
                return None
            try:
                from ultralytics import YOLO
                self._maceta_mod, self._maceta_nombre = YOLO(str(ruta)), nombre
                print(f"[camara] detector de macetas cargado: {nombre}", flush=True)
            except Exception as exc:
                print(f"[camara] no se pudo cargar {nombre}: {exc}", flush=True)
                self._maceta_mod = False
                return None
        return self._maceta_mod

    def _bucle_inferencia(self):
        while not self._stop.is_set():
            modelo = self.modelo
            if modelo is None:
                self._stop.wait(0.3)
                continue
            with self._lock:
                frame = None if self._frame_crudo is None else self._frame_crudo.copy()
            if frame is None:
                self._stop.wait(0.1)
                continue

            t0 = time.perf_counter()
            imgsz = int(self._get_config()["detection"].get("imgsz", 640))
            try:
                if self._device is None:
                    resultados = modelo(frame, imgsz=imgsz, verbose=False)[0]
                else:
                    resultados = modelo(frame, device=self._device, imgsz=imgsz, verbose=False)[0]
                self.infer_ms = (time.perf_counter() - t0) * 1000
                self._fallos_seguidos = 0
            except Exception as exc:
                # OJO CON REINTENTAR AQUI. Ultralytics rehace la preparacion del
                # modelo en CADA llamada que falla —se ve en el log como un
                # "YOLO11s summary (fused)" repetido— y esa preparacion deja
                # memoria sin liberar. El 2026-09-06 eso llevo al backend de
                # 992 MB a 4.5 GB en dos minutos y lo mato el OOM killer.
                #
                # Asi que un fallo NO se reintenta con la misma configuracion:
                #   1er fallo en GPU -> se baja a CPU para siempre. Es mas lento
                #      pero funciona, y es preferible a no detectar nada.
                #   3 fallos seguidos ya en CPU -> se deja de inferir. El video
                #      sigue, y el estado lo dice para que se vea en el panel.
                self._fallos_seguidos += 1
                if self._device is None:
                    self._device = "cpu"
                    self.estado = (f"inferencia en GPU no disponible, se pasa a "
                                   f"CPU (mas lenta). Motivo: {str(exc)[:90]}")
                    print(f"[camara] {self.estado}")
                    continue
                if self._fallos_seguidos >= 3:
                    self.estado = (f"inferencia DETENIDA tras 3 fallos en CPU. "
                                   f"El video sigue. Motivo: {str(exc)[:90]}")
                    print(f"[camara] {self.estado}")
                    self.modelo = None
                    continue
                self.estado = f"error de inferencia: {str(exc)[:120]}"
                self._stop.wait(2.0)
                continue

            cfg = self._get_config()["detection"]
            cajas, nuevas = [], []
            for box in resultados.boxes:
                conf = float(box.conf[0])
                if conf < cfg["confidence_threshold"]:
                    continue
                nombre = modelo.names[int(box.cls[0])]
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
                if cfg.get("plaga_requiere_vegetacion", True) and nombre not in cfg.get("clases_planta", []) \
                        and fraccion_vegetacion(frame, (x1, y1, x2, y2)) < float(cfg.get("plaga_vegetacion_min", 0.08)):
                    continue                       # caja sin hojas dentro: fondo, no planta
                if nombre not in cfg.get("clases_planta", []) and \
                        textura_bordes(frame, (x1, y1, x2, y2)) > float(cfg.get("plaga_bordes_max", 0.30)):
                    continue                       # textura de pasto (hojas finas por todas partes), no hojas de col
                cajas.append({"class": nombre, "confidence": conf,
                              "xyxy": (x1, y1, x2, y2)})
                nuevas.append((nombre, conf))

            # Segundo modelo: DETECTOR DE MACETAS (2026-10-22, Kaggle agrobot-macetas: 20/20 cuadros de la cámara del robot con la col en
            # maceta y 0/30 sin maceta). Sus cajas se suman a las del modelo activo con la clase «maceta» (para la visión activa y la misión).
            mod_mac = self._modelo_maceta(cfg)
            if mod_mac is not None:
                try:
                    kw = {} if self._device is None else {"device": self._device}
                    rm = mod_mac(frame, imgsz=int(cfg.get("maceta_imgsz", 640)), conf=float(cfg.get("maceta_conf", 0.40)), verbose=False, **kw)[0]
                    for box in rm.boxes:
                        x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
                        cajas.append({"class": "maceta", "confidence": float(box.conf[0]), "xyxy": (x1, y1, x2, y2)})
                        nuevas.append(("maceta", float(box.conf[0])))
                except Exception as exc:
                    print(f"[camara] detector de macetas desactivado: {exc}", flush=True)
                    self._maceta_mod = False

            with self._lock:
                self._cajas = cajas
                self._cajas_ts = time.time()
            self.detecciones = len(cajas)

            # Persistencia: una clase se avisa solo si salió en >= N de los últimos M fotogramas (un destello de un cuadro no es una plaga).
            M = int(cfg.get("plaga_persistencia_ventana", 5))
            N = int(cfg.get("plaga_persistencia_min", 3))
            self._hist_clases.append({n for n, _ in nuevas})
            del self._hist_clases[:-max(1, M)]
            if N > 1:
                nuevas = [(n, c) for n, c in nuevas if sum(1 for h in self._hist_clases if n in h) >= N]

            self._emitir(nuevas, frame, cfg)

            # Ritmo: al menos un periodo entre inferencias, descontando lo
            # que ya tardó. Si el modelo es lento, esto no suma espera.
            hz_max = float(cfg.get("infer_hz_max", 1.0 / INFER_PERIOD_S) or 1.0 / INFER_PERIOD_S)
            periodo = 1.0 / max(hz_max, 0.5)
            ciclo = time.perf_counter() - t0
            self.infer_hz = 1.0 / max(ciclo, periodo)
            resto = periodo - ciclo
            if resto > 0:
                self._stop.wait(resto)

    def _emitir(self, detecciones, frame, cfg):
        ahora = time.time()
        for nombre, conf in detecciones:
            if ahora - self._ultimo_aviso.get(nombre, 0) < DETECTION_COOLDOWN_S:
                continue
            self._ultimo_aviso[nombre] = ahora
            guardar = conf >= cfg["clip_record_threshold"]
            self._loop.call_soon_threadsafe(
                self._on_detection,
                {"class": nombre, "confidence": conf, "ts": ahora},
                frame if guardar else None)


ZOOM_MAX = 4.0


def _recorte_zoom(cv2, frame, z):
    """Centro del cuadro con el campo de un 4:3 a 1x (alto completo / z), reescalado a 640x480."""
    h, w = frame.shape[:2]
    ch = max(60, int(h / z))
    cw = min(w, int(ch * 4 / 3))
    y0, x0 = (h - ch) // 2, (w - cw) // 2
    return cv2.resize(frame[y0:y0 + ch, x0:x0 + cw], (640, 480), interpolation=cv2.INTER_CUBIC)


def _fijar_framerate_camara():
    """Desactiva `exposure_dynamic_framerate` del driver V4L2.

    Con esa opción la cámara alarga la exposición cuando hay poca luz y BAJA
    los FPS para conseguirlo. Medido en esta Jetson: 17.9 FPS con la opción
    activa contra 31.2 FPS sin ella, con la misma cámara y la misma escena.
    Se prefiere una imagen algo más oscura a una tasa que se derrumba justo
    cuando el robot está entre las plantas, que es donde hay sombra.

    Es un ajuste de la cámara, no del sistema: se pierde al desconectarla y
    se vuelve a aplicar en cada apertura. Si `v4l2-ctl` no está, se sigue
    igual — solo se pierden FPS.
    """
    import subprocess
    try:
        subprocess.run(
            ["v4l2-ctl", "-d", "/dev/video0",
             "--set-ctrl=exposure_dynamic_framerate=0"],
            check=False, capture_output=True, timeout=3)
    except (FileNotFoundError, subprocess.SubprocessError):
        pass
