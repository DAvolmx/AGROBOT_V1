import math
import numpy as np
from mapa_scan import MapaScan


def _scan_cuarto(x, y, th_deg, lado=4.0):
    """Cuarto cuadrado de `lado` m centrado en (0,0), visto desde (x, y, th)."""
    ang = np.arange(0, 360, 1.0)
    th = math.radians(th_deg)
    d = []
    for a in ang:
        dx, dy = math.cos(math.radians(a) + th), math.sin(math.radians(a) + th)
        ts = [t for t in ((lado / 2 - x) / dx if dx > 1e-9 else None, (-lado / 2 - x) / dx if dx < -1e-9 else None,
                          (lado / 2 - y) / dy if dy > 1e-9 else None, (-lado / 2 - y) / dy if dy < -1e-9 else None) if t]
        d.append(min(ts))
    return ang, np.array(d)


def test_acumula_paredes_y_formato():
    m = MapaScan()
    assert m.como_mapa() is None
    for i in range(5):
        a, d = _scan_cuarto(0.0, 0.0, 0.0)
        assert m.agregar(i * 0.1, {"x": 0.0, "y": 0.0, "theta_deg": 0.0}, a, d)
    r = m.como_mapa()
    g = np.frombuffer(__import__("base64").b64decode(r["data_b64"]), dtype=np.int8).reshape(r["height"], r["width"])
    assert (g == 100).sum() > 200 and (g == 0).sum() > 1000
    # el centro del cuarto es libre, la pared está a ~2 m
    cx = int((0 - r["origin_x"]) / r["resolution"]); cy = int((0 - r["origin_y"]) / r["resolution"])
    assert g[cy, cx] == 0
    assert g[cy, int((2.0 - r["origin_x"]) / r["resolution"])] == 100


def test_no_inserta_girando_rapido():
    m = MapaScan()
    a, d = _scan_cuarto(0, 0, 0)
    m.agregar(0.0, {"x": 0, "y": 0, "theta_deg": 0}, a, d)
    assert not m.agregar(0.1, {"x": 0, "y": 0, "theta_deg": 20}, a, d)    # 3,5 rad/s


def test_alineacion_reduce_la_deriva():
    """Pose con 5 cm de deriva: el mapa no debe engordar la pared (sin alinear quedaría un doble contorno)."""
    m = MapaScan()
    a, d = _scan_cuarto(0, 0, 0)
    for i in range(6):
        m.agregar(i * 0.1, {"x": 0, "y": 0, "theta_deg": 0}, a, d)
    a2, d2 = _scan_cuarto(0.0, 0.0, 0.0)
    m.agregar(0.7, {"x": 0.06, "y": 0.0, "theta_deg": 0.0}, a2, d2)     # slam dice 6 cm a la derecha, el cuarto no se movió
    assert abs(m.corr[0] + 0.06) <= 0.031


def test_reiniciar_borra():
    m = MapaScan()
    a, d = _scan_cuarto(0, 0, 0)
    m.agregar(0.0, {"x": 0, "y": 0, "theta_deg": 0}, a, d)
    m.reiniciar()
    assert m.como_mapa() is None


def _sala_scan(x, y, th, paredes):
    """LIDAR ideal contra segmentos (marco del robot)."""
    import math as _m
    out_a, out_d = [], []
    for a in range(0, 360, 2):
        ang = _m.radians(a) + th
        dx, dy = _m.cos(ang), _m.sin(ang)
        mejor = None
        for (x0, y0, x1, y1) in paredes:
            ex, ey = x1 - x0, y1 - y0
            den = dx * ey - dy * ex
            if abs(den) < 1e-9:
                continue
            t = ((x0 - x) * ey - (y0 - y) * ex) / den
            u = ((x0 - x) * dy - (y0 - y) * dx) / den
            if t > 0.05 and 0 <= u <= 1 and (mejor is None or t < mejor):
                mejor = t
        if mejor is not None and mejor < 7:
            out_a.append(float(a)); out_d.append(mejor)
    return out_a, out_d


def test_el_registro_recupera_un_salto_de_pose_sin_duplicar_paredes():
    """La pose del SLAM gira 12° de golpe (recolocación): sin registro quedan DOS copias de la sala; con registro, una."""
    import math as _m
    sala = [(-3, -2, 3, -2), (3, -2, 3, 2), (3, 2, -3, 2), (-3, 2, -3, -2), (0.5, -2, 0.5, -0.5)]
    res = {}
    for registro in (False, True):
        m = MapaScan(registro=registro)
        for k in range(120):
            x, y, th = -2.0 + 0.04 * k, 0.5 * _m.sin(k / 15.0), 0.0
            sesgo = _m.radians(12.0) if k >= 60 else 0.0                       # a partir del scan 60 la pose viene girada 12°
            a, d = _sala_scan(x, y, th, sala)
            m.agregar(k * 0.2, {"x": x, "y": y, "theta_deg": _m.degrees(th + sesgo)}, a, d)
        res[registro] = int((m.L > 1.0).sum())
    assert res[True] < res[False] * 0.85
