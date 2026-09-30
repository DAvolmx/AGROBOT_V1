"""
Robot simulado: cinemática skid-steer con los vicios del chasis real.
=====================================================================

No se busca un modelo fino, sino reproducir lo que condiciona las decisiones
del software real (medido en el robot, ver MANIFEST):

  · Los motores NO tienen gama lenta: el firmware mapea cualquier velocidad
    pedida a PWM 200..255. Pedir la mitad no frena a la mitad.
  · Al parar, las ruedas quedan sueltas y el robot DESLIZA (~80 cm a plena
    marcha): por eso el freno del LIDAR corta con tanta antelación.
  · Girando patina: la omega real es menor que la comandada y la tracción se
    pierde, más todavía en tierra blanda.
  · Rampa de aceleración limitada (`safety.max_*_accel`), igual que el
    backend real aplica antes de mandar la trama al ESP32-C3.
"""
import math
from collections import deque

import numpy as np
import mundo as mundo_mod

# Piso de velocidad efectiva: por debajo de esto el motor no mueve el robot,
# y por encima del piso cualquier orden sale casi a fondo (PWM 200..255).
V_MIN_EFECTIVA = 0.28
OMEGA_MIN_EFECTIVA = 0.55
# Cuánto de la omega comandada se convierte en giro real (el resto patina).
RENDIMIENTO_GIRO = 0.65
# Deceleración al soltar: v²/(2·d) con d ≈ 0,8 m desde 0,55 m/s.
DECEL_INERCIA = 0.19
RADIO_ROBOT_M = 0.24          # media diagonal de la carrocería (40 × 35 cm)

# Servo de la cámara. Medido en el robot real el 2026-09-19: INCLINA (no
# panea), recorrido 25°..110°, reposo 98° ≈ 15,5° bajo el horizonte, y 1° de
# servo ≈ 0,8° de inclinación. Se mueve a ~120°/s y tarda en asentarse.
SERVO_MIN, SERVO_MAX, SERVO_HOME = 25.0, 110.0, 98.0
SERVO_VEL_DEG_S = 120.0
SERVO_ASENTADO_S = 0.35
SERVO_GRADOS_POR_GRADO = 0.8
INCLINACION_EN_HOME = 15.5

# Latencia del mando Jetson -> C3 por BLE, medida con btmon el 2026-09-28:
# el backend manda a 20 Hz (una orden nueva espera 0-50 ms al siguiente
# envío) y la radio tarda 28 ms de mediana, 75 ms en el p95 y 173 ms en el
# peor caso visto (intervalo de conexión de ~48,75 ms).
PERIODO_ENVIO_S = 0.05
LATENCIA_MEDIANA_S = 0.028
LATENCIA_SIGMA = 0.6           # lognormal: p95 ≈ 75 ms con esa mediana
LATENCIA_MAX_S = 0.2


class RobotSim:
    """Estado del robot y su integración. Recibe (v, omega) COMANDADAS."""

    def __init__(self, mundo, pose, cfg, rng=None):
        self.mundo = mundo
        self.x, self.y, self.theta = pose
        self.cfg = cfg
        self.rng = rng or np.random.default_rng(0)
        self.v_obj = self.omega_obj = 0.0      # lo último que pidió el software
        self.v_rampa = self.omega_rampa = 0.0  # tras la rampa del backend
        self.v = self.omega = 0.0              # lo que realmente hace el robot
        self.recorrido = 0.0
        self.colisiones = 0          # episodios de choque, no pasos de física
        # Cabeceo del chasis: la suspensión es de recorrido largo y el
        # terreno tiene baches, así que la cámara se mueve arriba y abajo
        # mientras rueda. Dos grados bastan para que lo que la cámara cree
        # que está a 1 m esté en realidad a 1,2.
        self.cabeceo_deg = 0.0
        self._v_anterior = 0.0
        self.realismo = 1.0
        self.servo = SERVO_HOME      # ángulo actual del servo de la cámara
        self.servo_objetivo = SERVO_HOME
        self._servo_quieto_desde = 0.0
        self.plantas_pisadas = 0     # las que el LIDAR no ve y el robot aplasta
        self.roces_rama = 0          # frenadas por hojas que no son obstáculo
        self.contacto_s = 0.0        # tiempo pegado a un obstáculo
        self.en_contacto = False
        self.t = 0.0
        self._imu = None             # cabeceo.CabeceoIMU, el del robot real
        self._cabeceo_anterior = 0.0
        self.latencia_mando = True
        self._mandos = deque()       # (instante en que llega a la C3, v, omega)
        self._ultimo_mando_t = 0.0
        # Lo que cambia de un día (o de un suelo) a otro. Por defecto, lo
        # medido; el entrenador los sortea por episodio para que la red no
        # aprenda UN chasis ideal (aleatorización de dominio).
        self.rendimiento_giro = RENDIMIENTO_GIRO
        self.decel_inercia = DECEL_INERCIA
        self.escala_latencia = 1.0

    def ordenar(self, v, omega):
        """La orden llega a los motores con el retraso del enlace real.

        Sin esto el simulador reaccionaba al instante y lo aprendido en él
        frenaba tarde en el robot: 55 ms típicos a 0,55 m/s son 3 cm más."""
        if self.realismo <= 0 or not self.latencia_mando:
            self.v_obj, self.omega_obj = float(v), float(omega)
            return
        retraso = (self.rng.uniform(0, PERIODO_ENVIO_S)
                   + min(LATENCIA_MAX_S, self.rng.lognormal(math.log(LATENCIA_MEDIANA_S * self.escala_latencia),
                                                            LATENCIA_SIGMA)))
        # Las escrituras BLE llegan en orden: una orden no adelanta a otra.
        cuando = max(self.t + retraso * self.realismo, self._ultimo_mando_t)
        self._ultimo_mando_t = cuando
        self._mandos.append((cuando, float(v), float(omega)))

    def _aplicar_mandos(self):
        while self._mandos and self._mandos[0][0] <= self.t:
            _, self.v_obj, self.omega_obj = self._mandos.popleft()

    # Orden mínima que el firmware considera "moverse". Muy baja a propósito:
    # el ESP32-C3 mapea CUALQUIER velocidad distinta de cero a PWM 200..255.
    # Con el umbral en 0,02 el robot simulado casi no se movía, porque con
    # `max_linear_accel` = 0,1 m/s² y el troceado de pulsos la rampa nunca
    # llegaba a ese valor — y el robot real sí se mueve en esas condiciones.
    UMBRAL_ORDEN = 0.004

    # Suavizado del cabeceo del IMU (paso bajo de ~0,3 s a 20 Hz).
    ALFA_IMU = 0.25

    def _efectiva(self, pedida, piso):
        """Piso de los motores: cualquier orden no nula sale casi a fondo."""
        if abs(pedida) < self.UMBRAL_ORDEN:
            return 0.0
        signo = math.copysign(1.0, pedida)
        return signo * max(abs(pedida), piso)

    def mover_servo(self, angulo):
        """Pedir una inclinación de cámara (grados de servo, 25..110)."""
        self.servo_objetivo = max(SERVO_MIN, min(SERVO_MAX, float(angulo)))

    @property
    def servo_asentado(self) -> bool:
        """Tras moverse, el servo oscila: hasta que no se asienta, lo que ve
        la cámara no sirve (el robot real lo comprueba igual)."""
        return (abs(self.servo - self.servo_objetivo) < 0.5
                and self.t - self._servo_quieto_desde > SERVO_ASENTADO_S)

    @property
    def inclinacion_camara(self) -> float:
        """Grados que la cámara mira hacia abajo, según dónde esté el servo."""
        return INCLINACION_EN_HOME + SERVO_GRADOS_POR_GRADO * (SERVO_HOME - self.servo)

    def _paso_cabeceo(self, dt):
        """Cabeceo del chasis: lo marca el TERRENO bajo las ruedas.

        Se toma el desnivel del suelo entre delante y detrás del robot (el
        mismo ruido de terreno que dibuja `mundo.py`), así el mismo bache da
        siempre el mismo bandazo y el episodio es repetible. Se le suma el
        cabeceo de acelerar o frenar, y la suspensión lo sigue con retardo."""
        if self.realismo <= 0:
            self.cabeceo_deg = 0.0
            return
        from mundo import ruido

        c, sn = math.cos(self.theta), math.sin(self.theta)
        L = 0.2                                   # medio eje, aproximado
        delante = float(ruido(np.float32(self.x + c * L), np.float32(self.y + sn * L), 2.5, 2))
        detras = float(ruido(np.float32(self.x - c * L), np.float32(self.y - sn * L), 2.5, 2))
        # El desnivel se convierte en grados: ±1 de ruido ≈ ±4 cm de altura.
        pendiente = math.degrees(math.atan2((delante - detras) * 0.08, 2 * L))
        a_long = (self.v - self._v_anterior) / max(dt, 1e-3)
        empuje = -6.0 * a_long * 0.05
        self._v_anterior = self.v
        objetivo = float(np.clip((pendiente + empuje) * self.realismo, -5.0, 5.0))
        anterior = self.cabeceo_deg
        self.cabeceo_deg += (objetivo - self.cabeceo_deg) * min(1.0, dt * 8.0)
        self._alimentar_imu(dt, (self.cabeceo_deg - anterior) / max(dt, 1e-3), a_long)

    def _alimentar_imu(self, dt, giro_dps, a_long):
        """Sintetiza la cuenta del MPU6050 y se la pasa al estimador REAL.

        No se le entrega el cabeceo verdadero ni una imitación del filtro: se
        arma lo que mediría el sensor —gravedad inclinada, MÁS la aceleración
        propia del robot, más ruido— y se mete en `cabeceo.CabeceoIMU`, el
        mismo módulo que corre en la Jetson. Así lo que se mide en simulación
        es el estimador de verdad, con sus fallos, y no una versión ideal.
        Se llama en cada paso de física porque en el robot el IMU llega a
        20 Hz, muy por encima del ritmo al que la cámara pide la geometría."""
        if self._imu is None:
            from cabeceo import CabeceoIMU
            self._imu = CabeceoIMU()
            # El sensor no está perfectamente a nivel en el chasis. Ese
            # desalineo es lo ÚNICO que debe absorber el cero, y por eso se
            # fija aquí y no con `poner_cero()` en la primera muestra: en el
            # robot la calibración se hace parado en suelo plano, mientras que
            # la primera muestra del episodio cae donde caiga. Fijándolo con
            # la primera muestra, el cero se tragaba los 2° de pendiente del
            # punto de salida y el estimador quedaba peor que no corregir.
            self._sesgo_imu = float(self.rng.normal(0, 0.3))
            self._imu.cero_deg = self._sesgo_imu
        r = math.radians(self.cabeceo_deg + self._sesgo_imu)
        # Aceleración longitudinal en g: es justo lo que confunde a un
        # estimador que solo mire el acelerómetro.
        propia = a_long / 9.80665
        n = lambda sigma: float(self.rng.normal(0, sigma))
        self._imu.actualizar(math.sin(r) + propia + n(0.012), 0.0,
                             math.cos(r) + n(0.012), giro_dps + n(0.3), self.t)

    def _paso_servo(self, dt):
        objetivo = self.servo_objetivo
        if abs(objetivo - self.servo) < 1e-3:
            return
        paso = SERVO_VEL_DEG_S * dt
        self.servo += max(-paso, min(paso, objetivo - self.servo))
        self._servo_quieto_desde = self.t

    def paso(self, dt):
        self._aplicar_mandos()
        self._paso_servo(dt)
        self._paso_cabeceo(dt)
        seg = self.cfg["safety"]
        # Orden IGUAL que en el robot real: primero la rampa del backend
        # (`safety.max_*_accel`), y el piso de los motores DESPUÉS, ya en el
        # firmware. Al revés el robot no se movería casi nada con
        # aceleraciones bajas, y el real sí se mueve.
        if abs(self.v_obj) >= abs(self.v_rampa):
            dv = seg["max_linear_accel_mps2"] * dt
        else:
            dv = self.decel_inercia * dt
        self.v_rampa += max(-dv, min(dv, self.v_obj - self.v_rampa))
        dom = (seg["max_angular_accel_rps2"] if abs(self.omega_obj) >= abs(self.omega_rampa) else 3.0) * dt
        self.omega_rampa += max(-dom, min(dom, self.omega_obj - self.omega_rampa))

        self.v = self._efectiva(self.v_rampa, V_MIN_EFECTIVA)
        self.omega = self._efectiva(self.omega_rampa, OMEGA_MIN_EFECTIVA) * self.rendimiento_giro

        # Patinaje: un skid-steer nunca va exactamente recto.
        v = self.v * (1 + self.rng.normal(0, 0.03))
        om = self.omega + self.rng.normal(0, 0.02) + 0.01 * abs(self.v)
        th = self.theta + om * dt / 2            # ángulo medio del paso
        nx = self.x + v * math.cos(th) * dt
        ny = self.y + v * math.sin(th) * dt

        self._pisar(nx, ny)
        if self._choca(nx, ny):
            # Un roce mantenido es UN choque, no uno por paso: si no, girar
            # pegado a una pared daba miles de "colisiones".
            if not self.en_contacto:
                self.colisiones += 1
            self.en_contacto = True
            self.contacto_s += dt
            self.v = 0.0                          # se queda contra el obstáculo
        else:
            self.en_contacto = False
            self.recorrido += math.hypot(nx - self.x, ny - self.y)
            self.x, self.y = nx, ny
        self.theta = (th + om * dt / 2 + math.pi) % (2 * math.pi) - math.pi
        self.t += dt

    def _pisar(self, x, y):
        """Plantas aplastadas: las que quedan bajo la huella del robot.

        Es EL daño que no se ve venir: el LIDAR barre a 40 cm y una plantita
        de 10 no existe para él, así que el robot cree el camino libre, se
        sale del sendero y la pisa. Aquí se cuenta."""
        m = self.mundo
        candidatas = []
        if len(m.plantas):
            bajas = m.plantas[m.plantas[:, 4] < 0.40] if m.plantas.shape[1] > 4 else m.plantas
            if len(bajas):
                candidatas.append((bajas[:, 0], bajas[:, 1], "c"))
        bajas = m.objetos[m.objetos[:, 3] < 0.40] if len(m.objetos) else None
        if bajas is not None and len(bajas):
            candidatas.append((bajas[:, 0], bajas[:, 1], "o"))
        for px, py, marca in candidatas:
            cerca = np.nonzero(np.hypot(px - x, py - y) < RADIO_ROBOT_M)[0]
            for i in cerca:
                clave = (marca, int(i))
                if clave not in m.pisadas:
                    m.pisadas.add(clave)
                    self.plantas_pisadas += 1

    def _choca(self, x, y):
        """Choque real: paredes y objetos SÓLIDOS (macetas, piedras).

        Las ramas y arbustos NO cuentan: el LIDAR los ve y el robot frena por
        ellos, pero se puede pasar. Esa diferencia es la que enseña al robot a
        no quedarse bloqueado por un falso obstáculo."""
        if len(self.mundo.paredes):
            x1, y1, x2, y2 = self.mundo.paredes.T
            ex, ey = x2 - x1, y2 - y1
            largo2 = np.maximum(ex * ex + ey * ey, 1e-9)
            t = np.clip(((x - x1) * ex + (y - y1) * ey) / largo2, 0, 1)
            d = np.hypot(x - (x1 + t * ex), y - (y1 + t * ey))
            if bool((d < RADIO_ROBOT_M).any()):
                return True
        obj = self.mundo.objetos
        if len(obj):
            solidos = obj[obj[:, 4] > 0.5]
            if len(solidos):
                d = np.hypot(solidos[:, 0] - x, solidos[:, 1] - y)
                if bool((d < RADIO_ROBOT_M + solidos[:, 2]).any()):
                    return True
        return False

    def falso_obstaculo_delante(self, alcance=0.8) -> bool:
        """¿Lo que tengo delante es solo follaje? (para medir frenadas inútiles)"""
        obj = self.mundo.objetos
        if not len(obj):
            return False
        follaje = obj[(obj[:, 4] < 0.5) & (obj[:, 3] >= 0.40)]
        if not len(follaje):
            return False
        dx, dy = follaje[:, 0] - self.x, follaje[:, 1] - self.y
        delante = dx * math.cos(self.theta) + dy * math.sin(self.theta)
        lado = abs(-dx * math.sin(self.theta) + dy * math.cos(self.theta))
        return bool(((delante > 0) & (delante < alcance + follaje[:, 2]) &
                     (lado < RADIO_ROBOT_M + follaje[:, 2])).any())

    # -------------------------------------------------------------- sensores
    def cabeceo_medido(self) -> float:
        """Cabeceo tal como lo vería el robot, pasando por SU estimador.

        No se devuelve el cabeceo verdadero ni una imitación del filtro: se
        sintetizan las cuentas del MPU6050 (gravedad inclinada + la
        aceleración propia del robot + ruido) y se meten en `cabeceo.CabeceoIMU`,
        el mismo módulo que corre en la Jetson. Así el simulador mide el
        estimador de verdad, con sus fallos, y no una versión idealizada."""
        if self.realismo <= 0:
            return self.cabeceo_deg
        medido = None if self._imu is None else self._imu.grados
        return self.cabeceo_deg if medido is None else medido

    def pose(self) -> dict:
        return {"x": self.x, "y": self.y, "theta_deg": math.degrees(self.theta) % 360}

    def scan(self):
        return self.mundo.scan(self.x, self.y, self.theta, rng=self.rng,
                               realismo=self.realismo, omega=self.omega)

    def frame(self, geo, ancho=None, alto=None):
        # Con GPU, la resolución de la NexiGo (640x480): el detector la reduce
        # a 320x240 igual que en el robot, y eso también suaviza el ruido.
        if ancho is None:
            ancho, alto = mundo_mod.resolucion_camara()
        return self.mundo.camara(self.x, self.y, self.theta, geo, ancho, alto, rng=self.rng,
                                 velocidad=self.v, realismo=self.realismo,
                                 cabeceo_deg=self.cabeceo_deg, omega=self.omega)

    def desvio_del_sendero(self) -> float:
        return float(self.mundo.distancia_al_sendero(np.array(self.x), np.array(self.y)))
