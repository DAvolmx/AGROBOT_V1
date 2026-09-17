"""
Odometría visual con la webcam (monocular) por modelo de suelo plano.
====================================================================

Una cámara sola no mide distancias: la misma imagen sale de un objeto chico
cerca o de uno grande lejos. La escala se recupera con una suposición que en
un cultivo se cumple bastante: la parte baja de la imagen es SUELO, plano, a
una altura conocida de la cámara. Con la altura y la inclinación, cada píxel
bajo el horizonte corresponde a un punto concreto del suelo en metros.

Por cuadro:
  1. Se siguen rasgos (esquinas) de la mitad inferior con flujo óptico
     Lucas-Kanade, entre el cuadro anterior y el actual.
  2. Cada rasgo se proyecta al suelo, en metros, en los ejes del robot.
  3. Se ajusta con RANSAC el movimiento plano (giro + desplazamiento) que
     lleva los puntos de un cuadro al otro. RANSAC descarta lo que no es
     suelo o no se mueve como el suelo: plantas, piedras altas, personas.
  4. De ahí salen avance (m/s) y giro (rad/s).

Límites, dichos claro:
  · La escala depende de CAMARA_ALTURA, INCLINACION y HFOV, que están SIN
    MEDIR. Un 10 % de error en la altura es un 10 % de error en el avance.
    Por eso no se manda al EKF hasta que el operador lo compara con el EKF
    en el dashboard y activa `publicar_ros`.
  · La cámara va sobre el servo de escaneo: si no está al frente o está
    barriendo, se PAUSA. Un giro del servo se leería como giro del robot.
  · Suelo sin textura (barro liso, charcos, sombra dura) o cámara tapada:
    pocos rasgos y se pausa.
  · Lente gran angular sin calibrar: la distorsión de los bordes mete error.
"""
import math
import threading
import time

import numpy as np

# Resolución de trabajo. 640x480 no aporta precisión que compense el doble de
# cómputo en una Jetson que también corre YOLO y SLAM.
ANCHO, ALTO = 320, 240
HZ_MAX = 15.0
MIN_RASGOS = 60               # por debajo se vuelven a buscar
MAX_RASGOS = 220
MIN_INLIERS = 12
SUELO_MIN_M, SUELO_MAX_M = 0.25, 2.5   # lejos del robot el error por píxel crece mucho
RANSAC_UMBRAL_M = 0.05
QUIETO_PX = 0.25              # flujo mediano menor que esto = robot quieto
SERVO_TOLERANCIA_DEG = 3
SERVO_ASENTADO_S = 0.7        # tras volver al frente, el servo oscila un poco


class OdometriaWebcam:
    def __init__(self, *, obtener_frame, get_config, servo_al_frente, publicar):
        self._obtener_frame = obtener_frame      # () -> (bgr, ts, seq) | None
        self._get_config = get_config            # () -> dict sección odometria_webcam
        self._servo_al_frente = servo_al_frente  # () -> (bool, motivo)
        self._publicar = publicar                # (vx, vyaw, var_vx, var_vyaw)

        self._stop = threading.Event()
        self._hilo = None
        self._prev_gris = None
        self._prev_pts = None
        self._prev_ts = 0.0
        self._ultimo_seq = -1
        self._al_frente_desde = None

        self.estado = "detenida"
        self.rasgos = 0
        self.inliers = 0
        self.vx = 0.0
        self.vyaw = 0.0
        self.hz = 0.0
        self.medida_ts = 0.0

    # ------------------------------------------------------------------
    def start(self):
        if self._hilo is None:
            self._stop.clear()
            self._hilo = threading.Thread(target=self._bucle, name="odom-webcam", daemon=True)
            self._hilo.start()

    def stop(self):
        self._stop.set()
        if self._hilo is not None:
            self._hilo.join(timeout=2)
            self._hilo = None

    def snapshot(self) -> dict:
        return {"estado": self.estado, "rasgos": self.rasgos, "inliers": self.inliers,
                "vx": round(self.vx, 3), "vyaw": round(self.vyaw, 3),
                "hz": round(self.hz, 1),
                "edad_s": round(time.monotonic() - self.medida_ts, 1) if self.medida_ts else None}

    # ------------------------------------------------------------------
    def _reiniciar(self, estado: str):
        self._prev_gris = None
        self._prev_pts = None
        self.estado = estado
        self.rasgos = self.inliers = 0

    def _bucle(self):
        import cv2
        periodo = 1.0 / HZ_MAX
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                self._paso(cv2)
            except Exception as exc:        # un cuadro raro no debe matar el hilo
                self._reiniciar(f"error: {type(exc).__name__}")
            self._stop.wait(max(0.0, periodo - (time.monotonic() - t0)))

    def _paso(self, cv2):
        cfg = self._get_config()
        if not cfg["habilitada"]:
            self._reiniciar("deshabilitada")
            return

        al_frente, motivo = self._servo_al_frente()
        ahora = time.monotonic()
        if not al_frente:
            self._al_frente_desde = None
            self._reiniciar(f"pausada: {motivo}")
            return
        if self._al_frente_desde is None:
            self._al_frente_desde = ahora
        if ahora - self._al_frente_desde < SERVO_ASENTADO_S:
            self._reiniciar("pausada: servo asentándose")
            return

        dato = self._obtener_frame()
        if dato is None:
            self._reiniciar("pausada: sin cámara")
            return
        bgr, ts, seq = dato
        if seq == self._ultimo_seq:
            return
        self._ultimo_seq = seq
        gris = cv2.cvtColor(cv2.resize(bgr, (ANCHO, ALTO), interpolation=cv2.INTER_AREA),
                            cv2.COLOR_BGR2GRAY)

        geo = _Geometria(cfg)
        if self._prev_gris is None or self._prev_pts is None or len(self._prev_pts) < MIN_RASGOS:
            self._prev_pts = self._buscar_rasgos(cv2, gris, geo)
            self._prev_gris, self._prev_ts = gris, ts
            self.estado = "buscando rasgos"
            return

        dt = ts - self._prev_ts
        if not (0.01 < dt < 0.5):
            self._reiniciar("pausada: hueco entre cuadros")
            return

        sig, st, _err = cv2.calcOpticalFlowPyrLK(
            self._prev_gris, gris, self._prev_pts, None, winSize=(21, 21), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        ok = st.reshape(-1) == 1
        p0 = self._prev_pts.reshape(-1, 2)[ok]
        p1 = sig.reshape(-1, 2)[ok]
        self.rasgos = len(p1)

        # El cuadro actual pasa a ser el anterior, con los rasgos que siguen
        # vivos; si quedan pocos se buscan de nuevo en el próximo paso.
        self._prev_gris, self._prev_ts = gris, ts
        self._prev_pts = p1.reshape(-1, 1, 2).astype(np.float32)

        if len(p1) < MIN_INLIERS:
            self.estado = "pausada: pocos rasgos (suelo sin textura o cámara tapada)"
            self._prev_pts = None
            return

        if float(np.median(np.linalg.norm(p1 - p0, axis=1))) < QUIETO_PX:
            self._medida(0.0, 0.0, len(p1), dt, cfg, quieto=True)
            return

        s0, v0 = geo.a_suelo(p0)
        s1, v1 = geo.a_suelo(p1)
        validos = v0 & v1
        if validos.sum() < MIN_INLIERS:
            self.estado = "pausada: pocos rasgos en el suelo"
            return

        M, mascara = cv2.estimateAffinePartial2D(
            s0[validos], s1[validos], method=cv2.RANSAC,
            ransacReprojThreshold=RANSAC_UMBRAL_M, maxIters=200, confidence=0.99)
        if M is None:
            self.estado = "pausada: sin movimiento coherente"
            return
        inliers = int(mascara.sum())
        escala = math.hypot(M[0, 0], M[1, 0])
        if inliers < MIN_INLIERS or abs(escala - 1.0) > 0.2:
            # Una escala lejos de 1 dice que los puntos no se comportan como
            # suelo plano a la altura supuesta: calibración mala u obstáculo.
            self.estado = f"pausada: ajuste dudoso (escala {escala:.2f})"
            return

        # P1 = R(-dθ)·(P0 - T)  =>  ángulo de M = -dθ,  T = -R(dθ)·b
        dtheta = -math.atan2(M[1, 0], M[0, 0])
        c, s = math.cos(dtheta), math.sin(dtheta)
        bx, by = M[0, 2], M[1, 2]
        tx = -(c * bx - s * by)
        vx, vyaw = tx / dt, dtheta / dt
        if abs(vx) > 2.0 or abs(vyaw) > 4.0:
            self.estado = "pausada: medida imposible descartada"
            return
        self._medida(vx, vyaw, inliers, dt, cfg)

    def _medida(self, vx, vyaw, inliers, dt, cfg, quieto=False):
        self.vx, self.vyaw, self.inliers = vx, vyaw, inliers
        ahora = time.monotonic()
        if self.medida_ts:
            self.hz = 0.8 * self.hz + 0.2 / max(1e-3, ahora - self.medida_ts)
        self.medida_ts = ahora
        self.estado = "midiendo (quieto)" if quieto else "midiendo"
        if cfg["publicar_ros"]:
            # Pocos inliers = medida menos fiable = varianza mayor. El EKF
            # pondera solo con esto.
            factor = max(1.0, 40.0 / max(inliers, 1))
            self._publicar(vx, vyaw, 0.04 * factor, 0.02 * factor)

    def _buscar_rasgos(self, cv2, gris, geo):
        mascara = np.zeros_like(gris)
        mascara[geo.fila_minima_suelo():, :] = 255
        pts = cv2.goodFeaturesToTrack(gris, maxCorners=MAX_RASGOS, qualityLevel=0.01,
                                      minDistance=7, mask=mascara, blockSize=7)
        return pts


class _Geometria:
    """Proyección píxel -> suelo en ejes del robot (x adelante, y izquierda)."""

    def __init__(self, cfg):
        self.h = cfg["camara_altura_m"]
        self.phi = math.radians(cfg["camara_inclinacion_deg"])  # positiva = mira hacia abajo
        self.adelante = cfg["camara_adelante_m"]
        self.fx = (ANCHO / 2) / math.tan(math.radians(cfg["camara_hfov_deg"]) / 2)
        self.cx, self.cy = ANCHO / 2, ALTO / 2

    def fila_minima_suelo(self) -> int:
        """Primera fila de imagen donde el suelo queda a SUELO_MAX_M o menos.
        Más arriba los rasgos caen demasiado lejos y se descartarían igual."""
        v = self.a_pixel(np.array([[SUELO_MAX_M, 0.0]], np.float32))[0, 1]
        return int(min(ALTO - 20, max(0, math.ceil(v))))

    def a_pixel(self, suelo):
        """Inversa de a_suelo: punto del suelo (ejes del robot) -> píxel."""
        cph, sph = math.cos(self.phi), math.sin(self.phi)
        dx = suelo[:, 0] - self.adelante
        xc = -suelo[:, 1]
        yc = -dx * sph + self.h * cph
        zc = dx * cph + self.h * sph
        return np.stack([self.cx + self.fx * xc / zc, self.cy + self.fx * yc / zc], axis=1)

    def a_suelo(self, pts):
        xn = (pts[:, 0] - self.cx) / self.fx
        yn = (pts[:, 1] - self.cy) / self.fx
        cph, sph = math.cos(self.phi), math.sin(self.phi)
        # Rayo en ejes del robot con la cámara inclinada `phi` hacia abajo:
        #   (cosφ − yn·sinφ,  −xn,  −(yn·cosφ + sinφ))
        baja = yn * cph + sph
        with np.errstate(divide="ignore", invalid="ignore"):
            t = self.h / baja
            x = t * (cph - yn * sph) + self.adelante
            y = -t * xn
            validos = (baja > 1e-3) & (x > SUELO_MIN_M) & (x < SUELO_MAX_M)
        return np.stack([x, y], axis=1).astype(np.float32), validos
