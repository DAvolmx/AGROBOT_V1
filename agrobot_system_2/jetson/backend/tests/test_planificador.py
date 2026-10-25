"""Planificador A* sobre el mapa acumulado (sin robot).   python3 -m pytest tests/test_planificador.py -q"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planificador import Planificador  # noqa: E402


def _scan(paredes, x, y, psi):
    """LIDAR ideal contra segmentos."""
    out = []
    for a in range(360):
        ang = math.radians(a) + psi
        dx, dy = math.cos(ang), math.sin(ang)
        mejor = math.inf
        for (x0, y0, x1, y1) in paredes:
            ex, ey = x1 - x0, y1 - y0
            den = dx * ey - dy * ex
            if abs(den) < 1e-9:
                continue
            t = ((x0 - x) * ey - (y0 - y) * ex) / den
            u = ((x0 - x) * dy - (y0 - y) * dx) / den
            if t > 0.02 and 0 <= u <= 1:
                mejor = min(mejor, t)
        if mejor < 6.0:
            out.append({"angle_deg": float(a), "distance_m": mejor})
    return out


def test_rodea_un_muro_y_no_lo_atraviesa():
    paredes = [(2, -2, 2, 0.6), (-1, -2, 5, -2), (-1, 2, 5, 2), (-1, -2, -1, 2), (5, -2, 5, 2)]   # muro con hueco arriba (y > 0,6)
    p = Planificador(centro=(2, 0), inflar_m=0.25)
    for pos in ((0, 0), (0.5, 0.5), (1.2, 1.0), (1.5, 1.4)):
        p.actualizar(pos[0], pos[1], 0.0, _scan(paredes, pos[0], pos[1], 0.0))
    r = p.ruta((0, 0), (4, 0), tol_m=0.3)
    assert r is not None and max(q[1] for q in r) > 0.8              # sube por el hueco
    assert all(not (1.8 < q[0] < 2.2 and q[1] < 0.5) for q in r)     # nunca cruza el muro


def test_sin_camino_devuelve_none():
    paredes = [(2, -2, 2, 2), (-1, -2, 5, -2), (-1, 2, 5, 2), (-1, -2, -1, 2), (5, -2, 5, 2)]
    p = Planificador(centro=(2, 0), inflar_m=0.25)
    for pos in ((0, 0), (1, 0.5), (1, -0.5)):
        p.actualizar(pos[0], pos[1], 0.0, _scan(paredes, pos[0], pos[1], 0.0))
    assert p.ruta((0, 0), (4, 0), tol_m=0.3) is None


def test_siguiente_mira_por_delante():
    ruta = [(0, 0), (1, 0), (2, 0), (3, 0)]
    assert Planificador.siguiente(ruta, (0.1, 0.0), 1.5) == (2, 0)
