"""Prueba del modelo de evasión (evasion.py) con escenas armadas a mano.

    cd jetson/backend && python3 -m pytest tests/test_evasion.py -q
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from evasion import decidir  # noqa: E402


def pared(x0, y0, x1, y1, paso=0.03):
    """Puntos de una pared (marco del robot) como los daría un scan."""
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / paso))
    out = []
    for i in range(n + 1):
        x = x0 + (x1 - x0) * i / n
        y = y0 + (y1 - y0) * i / n
        out.append({"angle_deg": math.degrees(math.atan2(y, x)), "distance_m": math.hypot(x, y)})
    return out


def pasillo(ancho=0.65, largo=3.0):
    return pared(-1, ancho / 2, largo, ancho / 2) + pared(-1, -ancho / 2, largo, -ancho / 2)


def test_pasillo_libre_avanza():
    d = decidir(pasillo())
    assert d["accion"] == "avanzar" and 0 < d["cm"] <= 30


def test_pesa_invisible_al_lidar_la_ve_la_camara():
    """El caso real: el LIDAR ve el pasillo libre, la cámara ve la pesa."""
    pesa = [{"angle_deg": 0.0, "distance_m": 0.75}, {"angle_deg": 8.0, "distance_m": 0.76}]
    sin_camara = decidir(pasillo())
    con_camara = decidir(pasillo(), pesa)
    assert sin_camara["accion"] == "avanzar"
    assert con_camara["accion"] != "avanzar"


def test_obstaculo_delante_gira_hacia_el_lado_libre():
    """Pared delante a 0,6 m; a la izquierda abierto, a la derecha una pared."""
    pts = pared(0.6, -1.5, 0.6, 0.3) + pared(-1, -0.4, 0.6, -0.4)
    d = decidir(pts)
    assert d["accion"] == "girar" and d["grados"] > 0


def test_callejon_retrocede():
    pts = pared(-1.5, 0.3, 0.5, 0.3) + pared(-1.5, -0.3, 0.5, -0.3) + pared(0.5, -0.3, 0.5, 0.3)
    d = decidir(pts)
    assert d["accion"] == "retroceder"


def test_encerrado_atascado():
    pts = (pared(-0.4, 0.3, 0.45, 0.3) + pared(-0.4, -0.3, 0.45, -0.3)
           + pared(0.45, -0.3, 0.45, 0.3) + pared(-0.4, -0.3, -0.4, 0.3))
    assert decidir(pts)["accion"] == "atascado"


def test_respeta_el_objetivo():
    """Todo libre: con objetivo a la izquierda, gira a la izquierda."""
    d = decidir([], objetivo_deg=90)
    assert d["accion"] == "girar" and d["grados"] == 90


def test_en_pasillo_estrecho_no_propone_girar():
    """Pasillo de 60 cm con pared delante: no cabe girando, retrocede."""
    pts = pasillo(0.60) + pared(0.7, -0.3, 0.7, 0.3)
    d = decidir(pts)
    assert d["accion"] == "retroceder"
