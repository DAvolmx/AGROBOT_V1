"""Barrido activo del servo de tilt + freno proactivo por envolvente de vuelco.

El LIDAR está ALTO y casi no ve los montículos bajos (5-8 cm) que vuelcan y
atrancan el robot (accidente del 2026-10-09). La cámara, con el servo de tilt
mirando ACTIVAMENTE el suelo cercano, sí los ve (ya vía suelo_libre.py). Este
módulo aporta dos cosas, como DECISIÓN pura (sin hardware, testeable):

  1) BarridoTilt: programa el ángulo del servo en barrido — mira lejos (el
     sendero) y cerca (el suelo próximo) alternando; a más velocidad, más
     tiempo mirando lejos (como un conductor).
  2) FrenoProactivo: a partir de la distancia a un obstáculo (suelo_libre) y su
     altura (si se estima), limita la velocidad a la SEGURA de la envolvente de
     vuelco, frenando ANTES de llegar en vez de parar en seco o chocar.

Geometría del servo (medida/MANIFEST): HOME 98° ≈ 15,5° abajo; el sendero se ve
a ~76°. Menor ángulo = mira más LEJOS; mayor ángulo = mira más CERCA/abajo.
1° de servo ≈ 0,8° de inclinación de cámara (ver robot.py / cabeceo.py).

La ENVOLVENTE es la del chasis ACTUAL (12 kg, CG 20 cm) de fallos_3d /
catalogo_fallos.py. Si se cambia el chasis (p. ej. bajar el CG) hay que
regenerarla. Sin estimar altura, se asume el peor montículo plausible, que es
el comportamiento seguro.
"""
import math
from collections import deque

from calibracion_camara import SERVO_HOME_DEG, delta_inclinacion  # calibración única

# Ángulos del servo para el barrido (grados; dentro de 25..110 del firmware).
# CONVENCIÓN (ver calibracion_camara.py): MENOR ángulo de servo = cámara MÁS
# abajo. Por eso "cerca" (inspeccionar el suelo próximo y los montículos bajos)
# es un ángulo BAJO, y "lejos" (anticipar al frente) es un ángulo ALTO.
SERVO_CERCA_DEG = 74     # mira ~34° abajo: suelo cercano / montículos bajos
SERVO_LEJOS_DEG = 106    # mira ~9° abajo: anticipa al frente

# Envolvente de vuelco del chasis ACTUAL: (altura_cm, velocidad_segura_mps).
# De catalogo_fallos.py. Por encima de esa velocidad, ese montículo vuelca.
_ENVOLVENTE = ((0.0, 1.5), (2.0, 0.93), (3.0, 0.75), (5.0, 0.41),
               (8.0, 0.0), (10.0, 0.0))

# Si no se estima la altura, se asume este montículo (peor caso plausible del
# terreno): conservador = seguro.
ALTURA_ASUMIDA_M = 0.05


def inclinacion_por_servo(servo_angle):
    """Inclinación extra de la cámara (grados, + = más abajo) respecto a HOME,
    para que la geometría de suelo_libre valga a cualquier ángulo del barrido.
    Alias de la calibración compartida (signo corregido 2026-10-10)."""
    return delta_inclinacion(servo_angle)


def velocidad_segura(altura_m):
    """Velocidad máxima sin vuelco al subir un montículo de esa altura (m/s),
    interpolando la envolvente. 0 = vuelca hasta parado (hay que esquivar)."""
    h = max(0.0, altura_m) * 100.0
    env = _ENVOLVENTE
    if h <= env[0][0]:
        return env[0][1]
    if h >= env[-1][0]:
        return env[-1][1]
    for (h0, v0), (h1, v1) in zip(env, env[1:]):
        if h0 <= h <= h1:
            return v0 + (v1 - v0) * (h - h0) / (h1 - h0)
    return env[-1][1]


def altura_por_paralaje(d1, d2, baseline_m, h_cam):
    """Altura (m) de un punto por PARALAJE entre dos observaciones, avanzando.

    Con la cámara a altura h_cam, la distancia APARENTE (suelo plano) de un
    punto a altura h y distancia real x es d_app = x / (1 - h/h_cam). Al
    avanzar un baseline b, d_app baja b/(1 - h/h_cam). Así:
        d1 - d2 = b / (1 - h/h_cam)   ->   h = h_cam * (1 - b/(d1 - d2))
    Suelo (h=0): d1-d2 = b. Montículo: d1-d2 > b. Hoyo: d1-d2 < b (h<0).
    Devuelve None si el dato no sirve (no se acerca, o baseline nulo)."""
    if d1 is None or d2 is None or baseline_m <= 1e-4:
        return None
    caida = d1 - d2
    if caida <= 1e-4:                 # no se acercó al punto: sin información
        return None
    return h_cam * (1.0 - baseline_m / caida)


class EstimadorAltura:
    """Estima la altura del montículo al que el robot se acerca, por paralaje.

    Se le alimenta con (distancia_recorrida, distancia_aparente_al_obstaculo) a
    lo largo del acercamiento; ajusta la pendiente de d_app vs recorrido por
    mínimos cuadrados (más robusto que dos puntos): m = d(d_app)/d(recorrido) =
    -1/(1 - h/h_cam), de donde h = h_cam*(1 + 1/m). Suelo plano: m=-1 -> h=0;
    montículo: m<-1; hoyo: -1<m<0. El barrido a dos ángulos mantiene el
    montículo a la vista durante el acercamiento."""

    def __init__(self, h_cam=0.4, min_muestras=4, min_baseline_m=0.06,
                 salto_m=0.3, max_edad_s=2.0):
        self.h_cam = h_cam
        self.min_muestras = min_muestras
        self.min_baseline = min_baseline_m
        self.salto = salto_m
        self.max_edad = max_edad_s
        self.muestras = deque()       # (recorrido_m, d_app_m, t)

    def reset(self):
        self.muestras.clear()

    def alimentar(self, recorrido_m, d_app, t):
        """Una muestra. d_app None = no hay obstáculo en el camino -> se purga
        el historial viejo pero no se borra todo (puede reaparecer en el barrido)."""
        while self.muestras and t - self.muestras[0][2] > self.max_edad:
            self.muestras.popleft()
        if d_app is None:
            return
        # Si la distancia aparente SALTA hacia arriba, es otro obstáculo (el
        # anterior quedó atrás): empieza una tanda nueva.
        if self.muestras and d_app - self.muestras[-1][1] > self.salto:
            self.muestras.clear()
        self.muestras.append((recorrido_m, d_app, t))

    def altura(self):
        """Altura estimada (m), o None si no hay señal suficiente. Negativa = hoyo."""
        if len(self.muestras) < self.min_muestras:
            return None
        xs = [m[0] for m in self.muestras]
        ds = [m[1] for m in self.muestras]
        if max(xs) - min(xs) < self.min_baseline:
            return None
        mx = sum(xs) / len(xs)
        md = sum(ds) / len(ds)
        sxx = sum((x - mx) ** 2 for x in xs)
        sxd = sum((x - mx) * (d - md) for x, d in zip(xs, ds))
        if sxx <= 1e-9:
            return None
        m = sxd / sxx
        if m > -0.3:                  # no se acerca de forma clara: sin estimación
            return None
        h = self.h_cam * (1.0 + 1.0 / m)
        return max(-0.1, min(self.h_cam * 0.9, h))


class BarridoTilt:
    """Programa el ángulo del servo en barrido lejos<->cerca. A más velocidad,
    más tiempo mirando lejos (anticipar); parado/lento, inspecciona cerca."""

    def __init__(self, lejos=SERVO_LEJOS_DEG, cerca=SERVO_CERCA_DEG,
                 dwell_lejos_s=1.2, dwell_cerca_s=0.8, v_ref=0.6):
        self.lejos, self.cerca = lejos, cerca
        self.dwell_lejos, self.dwell_cerca = dwell_lejos_s, dwell_cerca_s
        self.v_ref = v_ref

    def _dwells(self, v):
        # A v alta, alarga la mirada lejana; a v baja, alarga la cercana.
        if v is None:
            return self.dwell_lejos, self.dwell_cerca
        f = max(0.3, min(2.0, v / self.v_ref))
        return self.dwell_lejos * f, self.dwell_cerca / f

    def fase(self, t, v=None):
        """'lejos' o 'cerca' según el instante t (s)."""
        dl, dc = self._dwells(v)
        return "lejos" if (t % (dl + dc)) < dl else "cerca"

    def angulo(self, t, v=None):
        """Ángulo objetivo del servo (grados) en el instante t."""
        return self.lejos if self.fase(t, v) == "lejos" else self.cerca


class FrenoProactivo:
    """Limita la velocidad para no volcar con lo que ve la cámara delante.

    Dado el hueco (distancia al obstáculo, m) y su altura estimada, calcula el
    tope de velocidad: la velocidad segura de la envolvente, o menos si hay que
    frenar dentro del hueco disponible. Si el obstáculo vuelca hasta parado o
    está encima, ordena parar (0) para esquivarlo."""

    def __init__(self, accel_max=3.0, holgura_frenado=1.3, parar_bajo_m=0.15):
        self.accel = accel_max
        self.holgura = holgura_frenado      # margen sobre la distancia de frenado
        self.parar_bajo = parar_bajo_m      # más cerca que esto -> parar

    def tope_velocidad(self, hueco_m, v_actual, altura_m=None):
        """Tope de velocidad (m/s). hueco_m None = sin obstáculo -> sin tope."""
        if hueco_m is None:
            return v_actual
        altura = ALTURA_ASUMIDA_M if altura_m is None else altura_m
        v_seg = velocidad_segura(altura)
        if v_seg <= 0.0 or hueco_m <= self.parar_bajo:
            return 0.0                       # vuelca hasta parado o pegado: parar
        # ¿Cabe frenar de v_actual a v_seg dentro del hueco (con holgura)?
        dist_frenado = (v_actual ** 2 - v_seg ** 2) / (2 * self.accel)
        if dist_frenado <= hueco_m / self.holgura:
            return v_seg
        # No cabe: el tope ahora es la mayor v que SÍ frena a v_seg en el hueco.
        usable = max(0.0, hueco_m / self.holgura)
        return math.sqrt(max(0.0, v_seg ** 2 + 2 * self.accel * usable))
