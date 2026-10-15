"""Modelo 3D reducido de estabilidad y fallos del AGROBOT.

El simulador de conducción (dinamica_pasiva.py) es PLANO (X, Y, psi, delta):
no puede volcar. Este módulo añade la física FUERA del plano que el plano no
ve, para estudiar los fallos que provocaron el accidente del 2026-10-09:

  · vuelco lateral en giro (aceleración centrípeta contra una vía estrecha),
  · vuelco por obstáculo / "trip" (una rueda sube un bulto y el avance la
    tumba),
  · cabeceo / vuelco hacia delante por frenada brusca,
  · despegue de ruedas por transferencia de carga.

No hay IO de hardware. Unidades SI.

Datos: geometría 0.19/0.37/0.37 m y pivote ±20° MEDIDOS; masa ~12 kg y CG
alto ~0.20 m CONFIRMADOS por el operador (2026-10-09). Las inercias de balanceo
y el acople del obstáculo son HIPÓTESIS documentadas, no identificación del
robot real: fijan la escala de tiempo del vuelco, no el que vuelque o no (eso
lo decide la barrera de energía, que es pura geometría + masa).

Modelo de vuelco: péndulo invertido sobre la arista de apoyo. El cuerpo gira
un ángulo phi alrededor de la arista exterior; la gravedad lo devuelve mientras
el CG esté por dentro de la arista y lo tumba cuando lo pasa (phi_tip = atan(d/h),
con d = distancia horizontal del CG a la arista y h = altura del CG).
"""
from dataclasses import dataclass
import math
import numpy as np

from dinamica_pasiva import Parametros

G = 9.80665


def _dist_punto_recta(p, a, b):
    """Distancia perpendicular del punto p a la recta que pasa por a y b (2D)."""
    a = np.asarray(a, float); b = np.asarray(b, float); p = np.asarray(p, float)
    d = b - a
    n = np.array([-d[1], d[0]])
    n /= np.linalg.norm(n)
    return abs(float(n @ (p - a)))


@dataclass
class Estabilidad:
    """Polígono de apoyo y umbrales estáticos a partir de los Parametros.

    Esquinas en el plano del suelo (x hacia delante desde el eje trasero, y a
    la izquierda). El polígono es un trapecio: estrecho delante (0.19 m),
    ancho detrás (0.37 m). El CG se proyecta en (cg_x, 0)."""
    p: Parametros = None

    def __post_init__(self):
        self.p = self.p or Parametros()
        q = self.p
        self.hf = (q.front_outer - q.tire_width) / 2   # semivía delantera (contacto)
        self.hr = (q.rear_outer - q.tire_width) / 2    # semivía trasera
        self.L = q.wheelbase
        self.cgx = q.cg_x
        self.h = q.cg_height
        self.RL = (0., self.hr); self.RR = (0., -self.hr)
        self.FL = (self.L, self.hf); self.FR = (self.L, -self.hf)
        self.cg = (self.cgx, 0.)
        self.m = q.chassis_mass + q.front_mass

    # --- distancias horizontales del CG a cada arista de vuelco ---
    def d_lateral(self):
        """A la arista lateral derecha (= izquierda por simetría). Es la recta
        que une la rueda trasera y la delantera de un mismo lado; como el eje
        delantero es estrecho, esta arista va en diagonal y queda CERCA del CG."""
        return _dist_punto_recta(self.cg, self.RR, self.FR)

    def d_adelante(self):
        return abs(self.L - self.cgx)

    def d_atras(self):
        return abs(self.cgx)

    def angulo_vuelco(self, d):
        return math.atan2(d, self.h)

    def resumen(self):
        dl, da, dt = self.d_lateral(), self.d_adelante(), self.d_atras()
        return {
            'masa_kg': self.m,
            'semivia_delantera_cm': round(self.hf * 100, 1),
            'semivia_trasera_cm': round(self.hr * 100, 1),
            'cg_altura_cm': round(self.h * 100, 1),
            'd_lateral_cm': round(dl * 100, 1),
            'ang_vuelco_lateral_deg': round(math.degrees(self.angulo_vuelco(dl)), 1),
            'ang_vuelco_adelante_deg': round(math.degrees(self.angulo_vuelco(da)), 1),
            'ang_vuelco_atras_deg': round(math.degrees(self.angulo_vuelco(dt)), 1),
            'SSF_lateral': round(dl / self.h, 3),            # a_y/g de vuelco cuasiestático
            'a_lateral_vuelco_mps2': round((dl / self.h) * G, 2),
        }

    def cargas_ruedas(self, a_x=0., a_y=0.):
        """Normales aproximadas [RL, RR, FL, FR] con transferencia de carga
        cuasiestática. a_x>0 acelera (pasa carga al eje trasero); a_x<0 frena
        (al delantero). a_y>0 empuja la masa hacia +y y descarga las ruedas +y.

        Reparto lateral por eje con la vía propia de cada eje, proporcional a
        su carga vertical. Es un INDICADOR de qué rueda despega antes (siempre
        una delantera, por la vía estrecha); el veredicto de vuelco lo da la
        integración del balanceo, no esta cuenta."""
        W = self.m * G
        f_front = self.cgx / self.L
        Nf, Nr = W * f_front, W * (1 - f_front)
        dN_x = self.m * a_x * self.h / self.L          # transferencia longitudinal
        Nf -= dN_x; Nr += dN_x
        Nf = max(Nf, 0.); Nr = max(Nr, 0.)

        def eje(N, hv):
            dN = self.m * a_y * self.h / (2 * hv) * (N / W)  # transferencia lateral
            return N / 2 - dN, N / 2 + dN                    # (izquierda, derecha)

        RLz, RRz = eje(Nr, self.hr)
        FLz, FRz = eje(Nf, self.hf)
        return np.array([RLz, RRz, FLz, FRz])


def _inercia_balanceo(m, ancho, alto):
    """Inercia de una caja equivalente alrededor de su CG (eje longitudinal
    para balanceo, transversal para cabeceo). HIPÓTESIS: fija la escala de
    tiempo, no el umbral de vuelco."""
    return m * (ancho * ancho + alto * alto) / 12.0


def _simular_pendulo(forcing, d, i_cg, m, h, phi0=0., phidot0=0.,
                     t_max=3.0, dt=5e-4):
    """Integra el vuelco sobre una arista: péndulo invertido con base en la
    arista de apoyo.

    forcing: pseudo-aceleración horizontal (m/s^2) que empuja el CG hacia fuera
             de la arista (a_y en giro, |a_x| en frenada). Puede ser constante.
    d, h:    distancia horizontal y altura del CG respecto a la arista.
    i_cg:    inercia de balanceo/cabeceo respecto al CG.
    phi0, phidot0: condiciones iniciales (p. ej. de un golpe contra obstáculo).

    Vuelca si phi alcanza phi_tip = atan(d/h) con velocidad positiva (el CG
    pasa sobre la arista y la gravedad deja de devolverlo)."""
    r = math.hypot(d, h)
    a0 = math.atan2(h, d)
    phi_tip = math.atan2(d, h)
    i_pivot = i_cg + m * r * r          # Steiner: eje en la arista
    phi, w = phi0, phidot0
    phimax = phi
    if phi >= phi_tip:
        return {'vuelca': True, 'phi_max_deg': math.degrees(phi),
                'phi_tip_deg': math.degrees(phi_tip), 't_vuelco_s': 0.0}
    n = int(t_max / dt)
    for k in range(n):
        # torque: fuerza inercial hacia fuera (brazo = altura del CG girado) menos
        # restitución de la gravedad (brazo = distancia horizontal del CG girado)
        tau = m * forcing * r * math.sin(a0 + phi) - m * G * r * math.cos(a0 + phi)
        w += (tau / i_pivot) * dt
        phi += w * dt
        phimax = max(phimax, phi)
        if phi >= phi_tip:
            return {'vuelca': True, 'phi_max_deg': math.degrees(phi),
                    'phi_tip_deg': math.degrees(phi_tip), 't_vuelco_s': (k + 1) * dt}
        if phi <= 0. and w <= 0.:
            phi, w = 0., 0.            # vuelve a apoyarse, sin rebote elástico
    return {'vuelca': False, 'phi_max_deg': math.degrees(phimax),
            'phi_tip_deg': math.degrees(phi_tip), 't_vuelco_s': None}


# ----------------------------------------------------------- escenarios de fallo
def vuelco_lateral_giro(v, omega, p=None, est=None):
    """Vuelco lateral sostenido en un giro a velocidad v y giro omega.

    La aceleración lateral del cuerpo es a_y = v*omega (centrípeta). Por encima
    del umbral cuasiestático (a_y > SSF*g) el balanceo crece hasta volcar."""
    est = est or Estabilidad(p)
    p = est.p
    a_y = abs(v * omega)
    i_cg = _inercia_balanceo(est.m, p.rear_outer, 2 * est.h)
    r = _simular_pendulo(a_y, est.d_lateral(), i_cg, est.m, est.h)
    r.update({'v': v, 'omega': omega, 'a_y_mps2': round(a_y, 2)})
    return r


def condiciones_obstaculo(h_obs, v, est, acople=0.30):
    """Condiciones iniciales de balanceo al subir UNA rueda delantera un
    escalón de altura h_obs a velocidad de avance v.

    phi0: balanceo geométrico cuando esa rueda del eje estrecho sube h_obs
          (las otras siguen en el suelo): el cuerpo se inclina atan(h_obs/2hf).
    phidot0: la velocidad de avance redirigida a subir el borde inyecta
          velocidad de balanceo ~ acople*v/(2hf). 'acople' (0..1) es la
          fracción de v que se convierte en subida vertical del neumático al
          golpear; depende de la forma del bulto (HIPÓTESIS, por defecto 0.30).
    """
    phi0 = math.atan2(h_obs, 2 * est.hf)
    # El amortiguador delantero, al comprimirse y RECULAR contra el montículo,
    # devuelve energía al balanceo: multiplica la velocidad de vuelco. susp_recoil
    # = 0 (rígido, ideal) no cambia nada; en el chasis rústico lo empeora.
    susp = 1.0 + getattr(est.p, "susp_recoil", 0.0)
    phidot0 = acople * v / (2 * est.hf) * susp
    return phi0, phidot0


def vuelco_obstaculo(h_obs, v, p=None, est=None, acople=0.30):
    """Vuelco por subir un obstáculo de altura h_obs a velocidad v con una
    rueda delantera (el caso del accidente del 2026-10-09)."""
    est = est or Estabilidad(p)
    p = est.p
    phi0, phidot0 = condiciones_obstaculo(h_obs, v, est, acople)
    i_cg = _inercia_balanceo(est.m, p.rear_outer, 2 * est.h)
    r = _simular_pendulo(0., est.d_lateral(), i_cg, est.m, est.h,
                         phi0=phi0, phidot0=phidot0)
    r.update({'h_obs_cm': round(h_obs * 100, 1), 'v': v,
              'phi0_deg': round(math.degrees(phi0), 1),
              'acople': acople})
    return r


def vuelca_obstaculo_energia(h_obs, v, p=None, est=None, acople=0.30):
    """Versión analítica (barrera de energía) de vuelco_obstaculo, sin integrar.

    Vuelca si la inclinación geométrica ya pasa el ángulo de vuelco, o si la
    energía de balanceo inyectada por el avance supera la barrera de potencial
    que separa el apoyo del punto de vuelco. Coincide con la integración (el
    péndulo no tiene amortiguamiento, así que conserva energía) y es O(1): útil
    para barrer miles de combinaciones en el Monte Carlo."""
    est = est or Estabilidad(p)
    p = est.p
    phi0, phidot0 = condiciones_obstaculo(h_obs, v, est, acople)
    d, h = est.d_lateral(), est.h
    r = math.hypot(d, h)
    a0 = math.atan2(h, d)
    phi_tip = math.atan2(d, h)
    if phi0 >= phi_tip:
        return True
    i_pivot = _inercia_balanceo(est.m, p.rear_outer, 2 * h) + est.m * r * r
    ke = 0.5 * i_pivot * phidot0 * phidot0
    barrera = est.m * G * (r - r * math.sin(a0 + phi0))
    return ke >= barrera


def v_critica_obstaculo_rapida(h_obs, p=None, est=None, v_max=1.6, acople=0.30):
    """Menor velocidad que vuelca al subir un obstáculo de h_obs, por bisección
    sobre el criterio de energía (O(1) por evaluación). 0 = vuelca hasta parado
    (geométrico); None = no vuelca hasta v_max."""
    est = est or Estabilidad(p)
    if vuelca_obstaculo_energia(h_obs, 0.0, est=est, acople=acople):
        return 0.0
    if not vuelca_obstaculo_energia(h_obs, v_max, est=est, acople=acople):
        return None
    lo, hi = 0.0, v_max
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        if vuelca_obstaculo_energia(h_obs, mid, est=est, acople=acople):
            hi = mid
        else:
            lo = mid
    return hi


def vuelca_giro_rapido(v, omega, p=None, est=None):
    """Vuelco lateral en giro por el umbral cuasiestático a_y > SSF*g (sin
    integrar). Coincide con la integración para giros sostenidos."""
    est = est or Estabilidad(p)
    return abs(v * omega) > (est.d_lateral() / est.h) * G


def cabeceo_frenada(v, decel, p=None, est=None):
    """Cabeceo / vuelco hacia delante al frenar de v a 0 con desaceleración
    'decel' (m/s^2, positiva). Vuelca de morro si la transferencia hacia el
    eje delantero pasa el CG sobre la arista delantera."""
    est = est or Estabilidad(p)
    p = est.p
    i_cg = _inercia_balanceo(est.m, p.wheelbase, 2 * est.h)
    r = _simular_pendulo(decel, est.d_adelante(), i_cg, est.m, est.h)
    # tiempo de frenado, por si vuelca antes de parar
    t_parada = v / decel if decel > 0 else float('inf')
    r.update({'v': v, 'decel_mps2': decel, 't_parada_s': round(t_parada, 2),
              'rueda_trasera_despega': est.cargas_ruedas(a_x=-decel)[0] <= 0})
    return r


# --------------------------------------------------------- fronteras de fallo
def max_omega_sin_vuelco(v, est, omega_max=6.0, paso=0.05):
    """Mayor giro omega (rad/s) a velocidad v antes de volcar de lado."""
    seguro = 0.0
    o = paso
    while o <= omega_max:
        if vuelco_lateral_giro(v, o, est=est)['vuelca']:
            return seguro
        seguro = o
        o += paso
    return omega_max


def v_critica_obstaculo(h_obs, est, v_max=3.0, paso=0.02, acople=0.30):
    """Menor velocidad de avance a la que subir un obstáculo de h_obs vuelca.
    Devuelve None si no vuelca hasta v_max."""
    v = 0.0
    while v <= v_max:
        if vuelco_obstaculo(h_obs, v, est=est, acople=acople)['vuelca']:
            return round(v, 2)
        v += paso
    return None
