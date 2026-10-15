"""Manejo en ZIGZAG para el eje delantero pasivo.

El eje delantero del AGROBOT pivota libre (±20°). El operador comprobó en el
robot real que se maneja mejor así, y este controlador lo reproduce:

  · EN RECTO: un zigzag suave (alternar leve izquierda/derecha, de media cero)
    mantiene el pivote ACTIVO y centrado, en vez de dejar que derive y se
    quede trabado contra un tope. El rumbo neto sigue recto.

  · EN CURVA: no se gira en arco continuo (que arrastra el pivote saturado
    contra el tope y patina), sino A PASOS: pulso de giro, pulso de avance,
    pulso de giro, pulso de avance. Entre pulsos el pivote se realinea y la
    curva se traza mejor.

Entrega órdenes (v, omega) continuas; no toca hardware. Pensado para el sim y,
una vez validado, para trasladar la lógica al backend/firmware del robot real.
"""
import math


class ControladorZigzag:
    def __init__(self, v=0.5, zz_amplitud_rps=0.6, zz_periodo_s=1.2,
                 giro_omega_rps=1.5, giro_pulso_s=0.5, avance_pulso_s=0.4,
                 giro_avance_mps=0.12, umbral_giro_rad=math.radians(12)):
        self.v = v
        self.zz_amplitud = zz_amplitud_rps
        self.zz_periodo = zz_periodo_s
        self.giro_omega = giro_omega_rps
        self.giro_pulso = giro_pulso_s
        self.avance_pulso = avance_pulso_s
        # Avance leve DURANTE el pulso de giro: es "giro adelante", no giro en
        # seco. El avance deja que el pivote se realinee en vez de clavarse en
        # el tope (girar con v=0 lo satura).
        self.giro_avance = giro_avance_mps
        self.umbral = umbral_giro_rad

    def modo(self, error_rumbo):
        return "giro" if abs(error_rumbo) > self.umbral else "recto"

    def ordenar(self, error_rumbo, t):
        """error_rumbo: ángulo (rad) del rumbo deseado respecto al actual
        (normalizado a [-pi, pi]). t: tiempo (s). Devuelve (v, omega)."""
        e = math.atan2(math.sin(error_rumbo), math.cos(error_rumbo))
        if abs(e) <= self.umbral:
            # Recto con zigzag: onda cuadrada de media cero mantiene vivo el pivote.
            signo = 1.0 if math.sin(2 * math.pi * t / self.zz_periodo) >= 0 else -1.0
            return self.v, self.zz_amplitud * signo
        # Curva a pasos: alterna un pulso de giro (en sitio) y uno de avance.
        ciclo = self.giro_pulso + self.avance_pulso
        if (t % ciclo) < self.giro_pulso:
            return self.giro_avance, math.copysign(self.giro_omega, e)  # giro + avance leve
        return self.v, 0.0                                    # avanza recto
