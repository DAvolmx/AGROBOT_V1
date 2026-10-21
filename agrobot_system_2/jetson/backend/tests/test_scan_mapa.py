import math
from scan_mapa import scan_a_mapa


def test_robot_en_origen_mira_adelante():
    pts = [{"angle_deg": 0.0, "distance_m": 1.0}]
    assert scan_a_mapa(pts, {"x": 0, "y": 0, "theta_deg": 0}) == [100, 0]


def test_rumbo_90_gira_el_scan():
    pts = [{"angle_deg": 0.0, "distance_m": 2.0}]
    x, y = scan_a_mapa(pts, {"x": 1.0, "y": 1.0, "theta_deg": 90})
    assert (x, y) == (100, 300)


def test_rayo_a_la_izquierda():
    pts = [{"angle_deg": 90.0, "distance_m": 1.0}]
    assert scan_a_mapa(pts, {"x": 0, "y": 0, "theta_deg": 0}) == [0, 100]


def test_descarta_fantasmas_y_diezma():
    pts = [{"angle_deg": a * 1.0, "distance_m": 3.0} for a in range(360)]
    pts += [{"angle_deg": 0.0, "distance_m": 12.0}, {"angle_deg": 0.0, "distance_m": 0.01}]
    out = scan_a_mapa(pts, {"x": 0, "y": 0, "theta_deg": 0}, maximo=100)
    assert len(out) == 200
    assert all(math.hypot(out[i], out[i + 1]) <= 301 for i in range(0, 200, 2))
