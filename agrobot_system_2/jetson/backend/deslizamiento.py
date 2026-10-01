"""
Detección de patinaje y de atasco — la idea de los róveres de Marte de la
NASA/JPL (Spirit, Opportunity, Curiosity): comparar lo que se le ORDENA a las
ruedas con lo que el robot REALMENTE se movió según un sensor que no depende
de las ruedas. Allá era la odometría visual de las cámaras; acá es la
odometría del LIDAR (rf2o, `/odom_rf2o`).

Por qué hace falta en este robot: no hay encoders, y en un skid-steer las
ruedas patinan siempre un poco (más en barro, pasto mojado o pendiente). Sin
esto el robot "cree" que avanza mientras escarba en el lugar, los motores se
recalientan y el EKF se desvía.

Qué NO se sabe de antemano: cuánto avanza el robot por cada m/s ordenado
(la velocidad al 100 % nunca se midió). Por eso la eficiencia `k`
(= v_medida / v_ordenada en marcha normal) se APRENDE sola, despacio, con el
robot andando bien, y el patinaje se mide contra ese `k`:

    patinaje = 1 - (v_medida / v_ordenada) / k        (0 = nada, 1 = total)

Atasco: se ordena avanzar con ganas y el LIDAR dice que no se mueve, ni
avanza ni gira, durante ATASCO_S seguidos.

Este módulo no toca ROS ni el actuador: recibe números y devuelve un estado
y un factor de velocidad (1 = sin recorte). Se prueba sin el robot.
"""
from __future__ import annotations

import time
from collections import deque

# Por debajo de esto no se mide: a velocidad muy baja el ruido del LIDAR
# domina y cualquier cociente miente.
V_MIN_MEDIR = 0.08          # m/s ordenados
OMEGA_MAX_RECTA = 0.35      # rad/s: con más giro, el avance no es comparable
TAU_RAPIDO_S = 0.8          # suavizado del cociente instantáneo
# rf2o llega con ~0,2 s de retraso y la rampa tarda en llevar el robot a la
# velocidad pedida: medir en cuanto empieza la orden daría un "patinaje" del
# 100 % en CADA arranque. Se espera a que la orden lleve esto estable.
ASENTAMIENTO_S = 0.7
# Además, cada medida se compara con la orden de hace LATENCIA_S (lo que el
# LIDAR está viendo ahora es el efecto de esa orden, no de la última).
LATENCIA_S = 0.3
TAU_K_S = 20.0              # aprendizaje lento de la eficiencia normal
K_INICIAL = 0.8             # hasta tener datos propios
K_MIN, K_MAX = 0.2, 1.6     # fuera de esto el dato es basura, no se aprende

PATINAJE_AVISO = 0.40       # se recorta la velocidad
PATINAJE_FUERTE = 0.70
FACTOR_AVISO = 0.6
FACTOR_FUERTE = 0.35
RECUPERACION_POR_S = 0.25   # cuánto vuelve el factor hacia 1 por segundo

ATASCO_S = 2.0
ATASCO_V_ORDEN = 0.12       # m/s ordenados como mínimo para hablar de atasco
ATASCO_V_MEDIDA = 0.02      # m/s medidos: "no se mueve"
ATASCO_W_MEDIDA = 0.05      # rad/s medidos: "no gira"


def _ema(previo: float, nuevo: float, dt: float, tau: float) -> float:
    a = min(1.0, max(0.0, dt / tau))
    return previo + (nuevo - previo) * a


class MonitorDeslizamiento:
    def __init__(self, reloj=time.monotonic):
        self._reloj = reloj
        self.v_orden = 0.0
        self.w_orden = 0.0
        self.v_medida = 0.0
        self.w_medida = 0.0
        self.k = K_INICIAL
        self.k_muestras_s = 0.0      # segundos de marcha normal aprendidos
        self.cociente = None         # v_medida / v_orden suavizado
        self.patinaje = 0.0
        self.factor = 1.0
        self.atascado = False
        self._atasco_desde = None
        self._t_medida = None
        self._t_factor = reloj()
        self._orden_estable_desde = None
        self._historial = deque()       # (t, v_orden), últimos ~2 s

    # ---- entradas -------------------------------------------------------
    def on_orden(self, v: float, w: float):
        """Velocidad que SALIÓ hacia la C3 (tras rampa y freno por LIDAR)."""
        comparable = abs(v) >= V_MIN_MEDIR and abs(w) <= OMEGA_MAX_RECTA
        # Estable = comparable y sin cambios grandes (>20 %) respecto de la
        # orden anterior; un cambio fuerte reinicia la espera.
        cambio = abs(float(v) - self.v_orden) > 0.2 * max(abs(self.v_orden), V_MIN_MEDIR)
        if not comparable:
            self._orden_estable_desde = None
        elif self._orden_estable_desde is None or cambio:
            self._orden_estable_desde = self._reloj()
        self.v_orden, self.w_orden = float(v), float(w)
        ahora = self._reloj()
        self._historial.append((ahora, self.v_orden))
        while self._historial and ahora - self._historial[0][0] > 2.0:
            self._historial.popleft()

    def _orden_hace(self, segundos: float) -> float:
        """Orden vigente hace `segundos` (la última anterior a ese instante)."""
        limite = self._reloj() - segundos
        v = self._historial[0][1] if self._historial else self.v_orden
        for t, vo in self._historial:
            if t <= limite:
                v = vo
            else:
                break
        return v

    def on_medida(self, v: float, w: float):
        """Velocidad medida por la odometría del LIDAR (rf2o)."""
        ahora = self._reloj()
        dt = 0.1 if self._t_medida is None else max(0.0, min(ahora - self._t_medida, 1.0))
        self._t_medida = ahora
        self.v_medida, self.w_medida = float(v), float(w)
        self._actualizar_patinaje(dt)
        self._actualizar_atasco(ahora)

    # ---- lógica ---------------------------------------------------------
    def _actualizar_patinaje(self, dt: float):
        midiendo = (self._orden_estable_desde is not None
                    and self._reloj() - self._orden_estable_desde >= ASENTAMIENTO_S)
        if not midiendo:
            self.cociente = None     # el próximo tramo arranca de cero
            # Sin dato comparable el patinaje se olvida poco a poco.
            self.patinaje = _ema(self.patinaje, 0.0, dt, TAU_RAPIDO_S * 2)
            return
        # Mismo sentido que la orden: retroceder cuando se pide avanzar es
        # patinaje total (cociente 0), no un número negativo raro.
        v_ref = self._orden_hace(LATENCIA_S)
        if abs(v_ref) < V_MIN_MEDIR or v_ref * self.v_orden <= 0:
            return
        c = max(0.0, self.v_medida / v_ref)
        self.cociente = c if self.cociente is None else _ema(self.cociente, c, dt, TAU_RAPIDO_S)
        self.patinaje = min(1.0, max(0.0, 1.0 - self.cociente / self.k))
        # Se aprende k solo en marcha sana: si patina, aprender ahí le
        # enseñaría que patinar es lo normal.
        if self.patinaje < 0.25 and K_MIN <= self.cociente <= K_MAX:
            self.k = _ema(self.k, self.cociente, dt, TAU_K_S)
            self.k_muestras_s += dt

    def _actualizar_atasco(self, ahora: float):
        empujando = abs(self.v_orden) >= ATASCO_V_ORDEN
        quieto = (abs(self.v_medida) < ATASCO_V_MEDIDA
                  and abs(self.w_medida) < ATASCO_W_MEDIDA)
        if empujando and quieto:
            if self._atasco_desde is None:
                self._atasco_desde = ahora
            self.atascado = (ahora - self._atasco_desde) >= ATASCO_S
        else:
            self._atasco_desde = None
            self.atascado = False

    def factor_velocidad(self) -> float:
        """Recorte a aplicar a la velocidad (1 = nada). Baja de golpe cuando
        patina y vuelve hacia 1 despacio, para no oscilar."""
        ahora = self._reloj()
        dt = max(0.0, min(ahora - self._t_factor, 1.0))
        self._t_factor = ahora
        if self.patinaje >= PATINAJE_FUERTE:
            objetivo = FACTOR_FUERTE
        elif self.patinaje >= PATINAJE_AVISO:
            objetivo = FACTOR_AVISO
        else:
            objetivo = 1.0
        if objetivo < self.factor:
            self.factor = objetivo
        else:
            self.factor = min(objetivo, self.factor + RECUPERACION_POR_S * dt)
        return self.factor

    def estado(self) -> dict:
        return {
            "patinaje": round(self.patinaje, 2),
            "factor": round(self.factor, 2),
            "atascado": self.atascado,
            "eficiencia_k": round(self.k, 2),
            "k_aprendido_s": round(self.k_muestras_s, 1),
            "v_orden": round(self.v_orden, 3),
            "v_medida": round(self.v_medida, 3),
        }
