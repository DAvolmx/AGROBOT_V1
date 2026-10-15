"""Compound footprint. Measured axles; tire/body lengths are hypotheses.
Pose is wheelbase midpoint. All distances in metres.

La geometría es PARAMETRIZABLE (2026-10-11): por defecto es el chasis actual,
pero se le puede pasar una `Geometria` distinta (p. ej. el chasis mejorado con
otra vía) para que las colisiones respeten el ancho real. Sin `geo`, el
comportamiento es idéntico al anterior.
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Geometria:
    cuerpo_largo: float = .35
    cuerpo_ancho: float = .25
    wheelbase: float = .37
    front_outer: float = .19
    rear_outer: float = .37
    tire_width: float = .047
    rueda_largo: float = .172


GEO = Geometria()               # chasis actual (por defecto)
LARGO = .542
ANCHO = GEO.rear_outer          # ancho máximo = vía trasera exterior


def geo_de_parametros(p):
    """Construye la Geometria de la huella desde unos Parametros de dinámica."""
    return Geometria(wheelbase=p.wheelbase, front_outer=p.front_outer,
                     rear_outer=p.rear_outer, tire_width=p.tire_width)


def local(px, py, x, y, theta):
    c, s = math.cos(theta), math.sin(theta)
    return (px - x) * c + (py - y) * s, -(px - x) * s + (py - y) * c


def partes(x, y, theta, delta=0, geo=GEO):
    c, s = math.cos(theta), math.sin(theta)
    yield x, y, theta, geo.cuerpo_largo, geo.cuerpo_ancho
    for front in (False, True):
        a = theta + (delta if front else 0)
        ax = (geo.wheelbase / 2 if front else -geo.wheelbase / 2)
        track = (geo.front_outer if front else geo.rear_outer) - geo.tire_width
        for sign in (-1, 1):
            yield (x + ax * c - sign * track / 2 * math.sin(a),
                   y + ax * s + sign * track / 2 * math.cos(a), a,
                   geo.rueda_largo, geo.tire_width)


def _circle(px, py, r, box):
    x, y, t, l, w = box
    a, b = local(px, py, x, y, t)
    return max(abs(a) - l / 2, 0) ** 2 + max(abs(b) - w / 2, 0) ** 2 <= r * r


def _segment(ax, ay, bx, by, box):
    x, y, t, l, w = box
    a, b = local(ax, ay, x, y, t)
    c, d = local(bx, by, x, y, t)
    lo, hi = 0., 1.
    for p, v, h in ((a, c - a, l / 2), (b, d - b, w / 2)):
        if abs(v) < 1e-12:
            if abs(p) > h:
                return False
        else:
            q1, q2 = (-h - p) / v, (h - p) / v
            lo = max(lo, min(q1, q2)); hi = min(hi, max(q1, q2))
            if lo > hi:
                return False
    return True


def circulo(px, py, r, x, y, theta, delta=0, geo=GEO):
    return any(_circle(px, py, r, b) for b in partes(x, y, theta, delta, geo))


def segmento(ax, ay, bx, by, x, y, theta, delta=0, geo=GEO):
    return any(_segment(ax, ay, bx, by, b) for b in partes(x, y, theta, delta, geo))


def choca(mundo, x, y, theta, delta=0, geo=GEO):
    return any(segmento(*p, x, y, theta, delta, geo) for p in mundo.paredes) or any(
        circulo(o[0], o[1], o[2], x, y, theta, delta, geo)
        for o in mundo.objetos if o[4] > .5)


def barrido(mundo, x, y, theta, nx, ny, ntheta, delta=0, ndelta=0, geo=GEO):
    da = math.atan2(math.sin(ntheta - theta), math.cos(ntheta - theta))
    n = max(1, math.ceil((math.hypot(nx - x, ny - y) + .34 * abs(da)
                          + .13 * abs(ndelta - delta)) / .004))
    for i in range(n + 1):
        f = i / n
        if choca(mundo, x + (nx - x) * f, y + (ny - y) * f,
                 theta + da * f, delta + (ndelta - delta) * f, geo):
            return True
    return False
