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

    def paso(self, dt):
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

    def _choca(self, x, y):
        """Choque contra una pared: el LIDAR y el chasis ven lo mismo (alto)."""
        if len(self.mundo.paredes) == 0:
            return False
        x1, y1, x2, y2 = self.mundo.paredes.T
        ex, ey = x2 - x1, y2 - y1
        largo2 = np.maximum(ex * ex + ey * ey, 1e-9)
        t = np.clip(((x - x1) * ex + (y - y1) * ey) / largo2, 0, 1)
        d = np.hypot(x - (x1 + t * ex), y - (y1 + t * ey))
        return bool((d < RADIO_ROBOT_M).any())

    # -------------------------------------------------------------- sensores
    def pose(self) -> dict:
        return {"x": self.x, "y": self.y, "theta_deg": math.degrees(self.theta) % 360}

    def scan(self):
        return self.mundo.scan(self.x, self.y, self.theta, rng=self.rng)

    def frame(self, geo, ancho=320, alto=240):
        return self.mundo.camara(self.x, self.y, self.theta, geo, ancho, alto, rng=self.rng)

    def desvio_del_sendero(self) -> float:
        return float(self.mundo.distancia_al_sendero(np.array(self.x), np.array(self.y)))
