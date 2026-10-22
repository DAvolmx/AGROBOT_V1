"""Rumbo fusionado para ir en LÍNEA RECTA (2026-10-22). Módulo puro: sin hardware, asyncio ni ROS.

El robot real nunca va derecho: siempre cae hacia un lado (ruedas desiguales, chasis desalineado, suelo). Un solo sensor no basta:
  · el GIROSCOPIO es suave y rápido pero su sesgo hace derivar el rumbo;
  · las PAREDES vistas por el LIDAR dan el ángulo respecto al pasillo, sin deriva, pero solo hay paredes a veces;
  · la CÁMARA (eje del sendero) da el ángulo respecto al camino, con ruido y pérdidas.
RumboFusion es un Kalman de 2 estados (error de rumbo e = rumbo - rumbo deseado, y sesgo del giroscopio) que integra el giroscopio y se
corrige con cada referencia disponible ponderada por su varianza. CorrectorRumbo decide, con zona muerta e histéresis, cuándo dar un
pulso de corrección y aprende el SESGO POR PULSO (hacia qué lado y cuánto cae el robot en cada pulso recto) para corregir antes.
No se engancha a nada: el simulador y el explorador lo usan con el mismo código.
"""
from __future__ import annotations

import math


def _ang(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class RumboFusion:
    def __init__(self, sigma_gyro=math.radians(0.25), sigma_sesgo=math.radians(0.02), e0_sigma=math.radians(5.0)):
        self.sg, self.sb = sigma_gyro, sigma_sesgo      # ruido del giroscopio (rad/s/√Hz aprox.) y paseo aleatorio del sesgo
        self.e = 0.0                                    # error de rumbo estimado (rad), + = apunta a la izquierda del rumbo deseado
        self.b = 0.0                                    # sesgo del giroscopio (rad/s)
        self.P = [[e0_sigma ** 2, 0.0], [0.0, math.radians(0.5) ** 2]]
        self.n_ref = 0

    def predecir(self, w_gyro: float, dt: float):
        """w_gyro en rad/s (+ = izquierda). Llamar a cada muestra del giroscopio."""
        if dt <= 0:
            return
        self.e = _ang(self.e + (w_gyro - self.b) * dt)
        P = self.P
        p00 = P[0][0] - dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + (self.sg ** 2) * dt
        p01 = P[0][1] - dt * P[1][1]
        p10 = P[1][0] - dt * P[1][1]
        p11 = P[1][1] + (self.sb ** 2) * dt
        self.P = [[p00, p01], [p10, p11]]

    def actualizar(self, z: float, sigma: float) -> float:
        """Referencia que mide DIRECTAMENTE el error de rumbo (pared, eje del sendero). Devuelve la innovación (rad)."""
        r = max(sigma, 1e-4) ** 2
        P = self.P
        y = _ang(z - self.e)
        s = P[0][0] + r
        k0, k1 = P[0][0] / s, P[1][0] / s
        self.e = _ang(self.e + k0 * y)
        self.b += k1 * y
        self.P = [[(1 - k0) * P[0][0], (1 - k0) * P[0][1]], [P[1][0] - k1 * P[0][0], P[1][1] - k1 * P[0][1]]]
        self.n_ref += 1
        return y

    def calibrar_quieto(self, w_gyro: float, peso=0.02):
        """Parado el giro real es 0: lo que marque el giroscopio es sesgo."""
        self.b += peso * (w_gyro - self.b)

    def nuevo_objetivo(self, giro_rad: float):
        """Giro DESEADO (maniobra): el rumbo deseado se mueve con él y el error no cambia por eso."""
        self.e = _ang(self.e - giro_rad)

    @property
    def incertidumbre_deg(self) -> float:
        return math.degrees(math.sqrt(max(self.P[0][0], 0.0)))


def angulo_pared(pts_xy, x_min=-0.5, x_max=1.2, y_max=1.6, n_min=8, r2_min=0.85):
    """Ángulo del rumbo respecto a UNA pared (rad, + = el robot apunta a la izquierda de la pared) por ajuste de recta y = m x + c.
    pts_xy: puntos (x adelante, y izquierda) en metros, ya del lado de la pared. Devuelve (e, sigma) o None si no hay pared fiable."""
    p = [(x, y) for x, y in pts_xy if x_min <= x <= x_max and abs(y) <= y_max]
    n = len(p)
    if n < n_min:
        return None
    mx = sum(x for x, _ in p) / n
    my = sum(y for _, y in p) / n
    sxx = sum((x - mx) ** 2 for x, _ in p)
    if sxx < 0.04:                                   # poca extensión en x: la pendiente no está determinada
        return None
    sxy = sum((x - mx) * (y - my) for x, y in p)
    syy = sum((y - my) ** 2 for _, y in p)
    m = sxy / sxx
    res = sum((y - (my + m * (x - mx))) ** 2 for x, y in p)
    r2 = 1.0 if syy < 1e-9 else 1.0 - res / syy
    if syy > 1e-6 and r2 < r2_min:
        return None
    sig = max(math.radians(1.5), math.sqrt(res / max(n - 2, 1) / sxx))     # error de la pendiente ~ σ_y/√Sxx
    return -math.atan(m), sig


def _recta(pts_xy, x_min=-0.5, x_max=1.2, y_max=1.6, n_min=8, r2_min=0.85):
    """Ajuste y = m x + c de una pared: (m, c) o None."""
    p = [(x, y) for x, y in pts_xy if x_min <= x <= x_max and abs(y) <= y_max]
    n = len(p)
    if n < n_min:
        return None
    mx = sum(x for x, _ in p) / n
    my = sum(y for _, y in p) / n
    sxx = sum((x - mx) ** 2 for x, _ in p)
    if sxx < 0.04:
        return None
    m = sum((x - mx) * (y - my) for x, y in p) / sxx
    syy = sum((y - my) ** 2 for _, y in p)
    res = sum((y - (my + m * (x - mx))) ** 2 for x, y in p)
    if syy > 1e-6 and 1.0 - res / syy < r2_min:
        return None
    return m, my - m * mx


def desvio_lateral(pts_izq, pts_der, **kw):
    """Desplazamiento del robot respecto al CENTRO del pasillo (m, + = el centro queda a la izquierda del robot), con las dos paredes
    ajustadas como rectas (más robusto que restar las distancias mínimas de los sectores). None si no se ven las dos paredes.
    Idea de arXiv 2106.15029 (seguimiento de hileras con LIDAR: ángulo + desplazamiento respecto a la hilera)."""
    a, b = _recta(pts_izq, **kw), _recta(pts_der, **kw)
    if a is None or b is None or a[1] <= 0 or b[1] >= 0:
        return None
    return (a[1] + b[1]) / 2.0


def error_con_mirada(e_rumbo: float, desvio_m, mirada_m: float) -> float:
    """Error de rumbo respecto a un punto objetivo `mirada_m` por delante, sobre la línea central (persecución pura): si el robot está a
    la derecha del centro (desvío > 0) conviene apuntar algo a la izquierda, y viceversa."""
    if desvio_m is None or mirada_m <= 0:
        return e_rumbo
    return _ang(e_rumbo - math.atan2(desvio_m, mirada_m))


def angulo_paredes(pts_izq, pts_der, **kw):
    """Combina la pared izquierda y la derecha (media ponderada por varianza). None si ninguna sirve."""
    a = [r for r in (angulo_pared(pts_izq, **kw), angulo_pared(pts_der, **kw)) if r is not None]
    if not a:
        return None
    w = [1.0 / s ** 2 for _, s in a]
    return sum(e * wi for (e, _), wi in zip(a, w)) / sum(w), math.sqrt(1.0 / sum(w))


class CorrectorRumbo:
    """Decide el pulso de corrección ('izquierda'/'derecha'/None) a partir del error de rumbo fusionado."""

    def __init__(self, zona_muerta_deg=3.0, histeresis=0.5, aprende=0.3, sesgo_max_deg=6.0):
        self.zm = math.radians(zona_muerta_deg)
        self.hist = histeresis
        self.alfa = aprende
        self.sesgo_max = math.radians(sesgo_max_deg)
        self.sesgo = 0.0                # caída típica del rumbo por pulso recto (rad); + = hacia la izquierda
        self._e_antes = None
        self._lado = 0

    def inicio_pulso_recto(self, e: float):
        self._e_antes = e

    def fin_pulso_recto(self, e: float):
        """Mide cuánto cayó el rumbo en el pulso recto que acaba de terminar y actualiza el sesgo aprendido."""
        if self._e_antes is None:
            return
        d = _ang(e - self._e_antes)
        self.sesgo = max(-self.sesgo_max, min(self.sesgo_max, (1 - self.alfa) * self.sesgo + self.alfa * d))
        self._e_antes = None

    def decidir(self, e: float):
        """e = error de rumbo (+ = apunta a la izquierda). Anticipa el sesgo del próximo pulso recto."""
        ep = e + self.sesgo
        umbral = self.zm * (self.hist if self._lado != 0 else 1.0)
        if abs(ep) < umbral:
            self._lado = 0
            return None
        self._lado = 1 if ep > 0 else -1
        return "derecha" if ep > 0 else "izquierda"
