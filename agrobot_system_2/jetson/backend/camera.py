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

# Periodo mínimo entre inferencias. No se corre YOLO en cada frame: a 30 FPS
# serían 30 inferencias/s que la Jetson no da, y para detectar una plaga en
# una planta no hace falta esa cadencia — el robot se mueve lento.
INFER_PERIOD_S = 0.25

# Una misma plaga aparece en decenas de frames seguidos. Sin esto se emitía
# un mensaje WebSocket y un toast POR CAJA Y POR FRAME: a 30 FPS eso inunda
# el dashboard y la galería se llena de la misma foto repetida.
DETECTION_COOLDOWN_S = 3.0

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
        self._frame_crudo = None     # último frame sin dibujar, para YOLO
        self._cajas = []             # últimas detecciones dibujables
        self._cajas_ts = 0.0
        self._ultimo_aviso = {}      # clase -> ts del último aviso emitido

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

    def subscribe(self) -> asyncio.Queue:
        q = asyncio.Queue(maxsize=1)
        self._suscriptores.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue):
        self._suscriptores.discard(q)

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
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            return None
        # MJPG antes que el tamaño: en V4L2 el formato condiciona qué
        # resoluciones y tasas se pueden negociar después.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        # Buffer de 1: si el consumidor se atrasa queremos el frame NUEVO,
        # no ir sacando frames viejos de una cola.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        _fijar_framerate_camara()
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
                        self.estado = "sin cámara"
                        self._stop.wait(REOPEN_DELAY_S)
                        continue
                    self.estado = "capturando"

                ok, frame = cap.read()
                if not ok:
                    # Un fallo de lectura suele ser desconexión USB. Se cierra
                    # y se reabre: seguir leyendo de un handle muerto devuelve
                    # False para siempre y el video no vuelve solo.
                    cap.release(); cap = None
                    self.estado = "lectura fallida, reabriendo"
                    continue

                self._on_beat("camera")
                with self._lock:
                    self._frame_crudo = frame
                    cajas = list(self._cajas)

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
            try:
                if self._device is None:
                    resultados = modelo(frame, verbose=False)[0]
                else:
                    resultados = modelo(frame, device=self._device, verbose=False)[0]
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
                cajas.append({"class": nombre, "confidence": conf,
                              "xyxy": (x1, y1, x2, y2)})
                nuevas.append((nombre, conf))

            with self._lock:
                self._cajas = cajas
                self._cajas_ts = time.time()
            self.detecciones = len(cajas)

            self._emitir(nuevas, frame, cfg)

            # Ritmo: al menos INFER_PERIOD_S entre inferencias, descontando
            # lo que ya tardó. Si el modelo es lento, esto no suma espera.
            ciclo = time.perf_counter() - t0
            self.infer_hz = 1.0 / max(ciclo, INFER_PERIOD_S)
            resto = INFER_PERIOD_S - ciclo
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
