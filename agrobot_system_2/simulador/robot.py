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

import numpy as np

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
        self.servo = SERVO_HOME      # ángulo actual del servo de la cámara
        self.servo_objetivo = SERVO_HOME
        self._servo_quieto_desde = 0.0
        self.plantas_pisadas = 0     # las que el LIDAR no ve y el robot aplasta
        self.roces_rama = 0          # frenadas por hojas que no son obstáculo
        self.contacto_s = 0.0        # tiempo pegado a un obstáculo
        self.en_contacto = False
        self.t = 0.0

    def ordenar(self, v, omega):
        self.v_obj, self.omega_obj = float(v), float(omega)

    def _efectiva(self, pedida, piso):
        """Piso de los motores: por debajo no mueven, y cualquier orden por
        encima sale casi a fondo (el firmware mapea a PWM 200..255)."""
        if abs(pedida) < 0.02:
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

    def _paso_servo(self, dt):
        objetivo = self.servo_objetivo
        if abs(objetivo - self.servo) < 1e-3:
            return
        paso = SERVO_VEL_DEG_S * dt
        self.servo += max(-paso, min(paso, objetivo - self.servo))
        self._servo_quieto_desde = self.t

    def paso(self, dt):
        self._paso_servo(dt)
        seg = self.cfg["safety"]
        # Orden IGUAL que en el robot real: primero la rampa del backend
        # (`safety.max_*_accel`), y el piso de los motores DESPUÉS, ya en el
        # firmware. Al revés el robot no se movería casi nada con
        # aceleraciones bajas, y el real sí se mueve.
        if abs(self.v_obj) >= abs(self.v_rampa):
            dv = seg["max_linear_accel_mps2"] * dt
        else:
            dv = DECEL_INERCIA * dt
        self.v_rampa += max(-dv, min(dv, self.v_obj - self.v_rampa))
        dom = (seg["max_angular_accel_rps2"] if abs(self.omega_obj) >= abs(self.omega_rampa) else 3.0) * dt
        self.omega_rampa += max(-dom, min(dom, self.omega_obj - self.omega_rampa))

        self.v = self._efectiva(self.v_rampa, V_MIN_EFECTIVA)
        self.omega = self._efectiva(self.omega_rampa, OMEGA_MIN_EFECTIVA) * RENDIMIENTO_GIRO

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

        Es EL daño que no se ve venir: el LIDAR barre a 30 cm y una plantita
        de 10 no existe para él, así que el robot cree el camino libre, se
        sale del sendero y la pisa. Aquí se cuenta."""
        m = self.mundo
        candidatas = []
        if len(m.plantas):
            candidatas.append((m.plantas[:, 0], m.plantas[:, 1], "c"))
        bajas = m.objetos[m.objetos[:, 3] < 0.30] if len(m.objetos) else None
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
        follaje = obj[(obj[:, 4] < 0.5) & (obj[:, 3] >= 0.30)]
        if not len(follaje):
            return False
        dx, dy = follaje[:, 0] - self.x, follaje[:, 1] - self.y
        delante = dx * math.cos(self.theta) + dy * math.sin(self.theta)
        lado = abs(-dx * math.sin(self.theta) + dy * math.cos(self.theta))
        return bool(((delante > 0) & (delante < alcance + follaje[:, 2]) &
                     (lado < RADIO_ROBOT_M + follaje[:, 2])).any())

    # -------------------------------------------------------------- sensores
    def pose(self) -> dict:
        return {"x": self.x, "y": self.y, "theta_deg": math.degrees(self.theta) % 360}

    def scan(self):
        return self.mundo.scan(self.x, self.y, self.theta, rng=self.rng)

    def frame(self, geo, ancho=320, alto=240):
        return self.mundo.camara(self.x, self.y, self.theta, geo, ancho, alto, rng=self.rng)

    def desvio_del_sendero(self) -> float:
        return float(self.mundo.distancia_al_sendero(np.array(self.x), np.array(self.y)))
