import math

from camara_lidar import CamaraComoLidar


def _ultimo(puntos, t=0.0, **extra):
    return {"t": t, "puntos": puntos, "referencia_dudosa": False, "objeto_en_referencia": False, **extra}


def _f(t=[0.0]):
    return lambda: t[0]


def test_sin_dato_devuelve_el_lidar():
    f = CamaraComoLidar(reloj=lambda: 0.0)
    lidar = [{"angle_deg": 0.0, "distance_m": 2.0}]
    assert f.fusionar(lidar, None) == lidar


def test_columna_suelta_se_descarta():
    f = CamaraComoLidar(reloj=lambda: 0.0)
    assert f.scan_camara(_ultimo([{"angle_deg": 0.0, "distance_m": 0.8}])) == []


def test_obstaculo_bajo_se_anade_si_el_lidar_no_lo_ve():
    f = CamaraComoLidar(reloj=lambda: 0.0)
    pts = [{"angle_deg": a, "distance_m": 0.7} for a in (-3.0, 0.0, 3.0)]
    lidar = [{"angle_deg": a, "distance_m": 3.0} for a in (-3.0, 0.0, 3.0)]
    r = f.fusionar(lidar, _ultimo(pts))
    assert min(p["distance_m"] for p in r) == 0.7 and len(r) == 6


def test_el_lidar_manda_si_ya_ve_mas_cerca():
    f = CamaraComoLidar(reloj=lambda: 0.0)
    pts = [{"angle_deg": a, "distance_m": 1.0} for a in (-3.0, 0.0, 3.0)]
    lidar = [{"angle_deg": a, "distance_m": 0.6} for a in (-3.0, 0.0, 3.0)]
    assert len(f.fusionar(lidar, _ultimo(pts))) == 3


def test_dato_viejo_o_referencia_dudosa_no_cuentan():
    t = [0.0]
    f = CamaraComoLidar(reloj=lambda: t[0])
    pts = [{"angle_deg": a, "distance_m": 0.7} for a in (-3.0, 0.0, 3.0)]
    assert f.scan_camara(_ultimo(pts, referencia_dudosa=True)) == []
    t[0] = 5.0
    assert f.scan_camara(_ultimo(pts, t=0.0)) == []


def test_persistencia_corta_del_parpadeo():
    t = [0.0]
    f = CamaraComoLidar(reloj=lambda: t[0])
    pts = [{"angle_deg": a, "distance_m": 0.7} for a in (-3.0, 0.0, 3.0)]
    assert len(f.scan_camara(_ultimo(pts))) == 3
    t[0] = 0.3
    assert len(f.scan_camara(None)) == 3
    t[0] = 0.9
    assert f.scan_camara(None) == []


def test_objeto_pegado_pone_pared_virtual():
    f = CamaraComoLidar(reloj=lambda: 0.0)
    r = f.scan_camara(_ultimo([], objeto_en_referencia=True, extrana_seguidas=3), largo_robot_m=0.5)
    assert r and all(abs(p["distance_m"] - 0.3) < 1e-9 for p in r)


def test_fuera_del_campo_o_lejos_se_ignora():
    f = CamaraComoLidar(reloj=lambda: 0.0)
    pts = [{"angle_deg": a, "distance_m": 0.7} for a in (60.0, 63.0, 66.0)] + [{"angle_deg": a, "distance_m": 3.5} for a in (0.0, 3.0)]
    assert f.scan_camara(_ultimo(pts)) == []


def test_solo_camara_reemplaza_el_frente():
    f = CamaraComoLidar(reloj=lambda: 0.0)
    pts = [{"angle_deg": a, "distance_m": 1.5} for a in (-3.0, 0.0, 3.0)]
    lidar = [{"angle_deg": 0.0, "distance_m": 0.4}, {"angle_deg": 90.0, "distance_m": 1.0}]
    r = f.fusionar(lidar, _ultimo(pts), solo_camara=True)
    assert {p["distance_m"] for p in r if min(p["angle_deg"], 360 - p["angle_deg"]) <= 35} == {1.5} and any(p["angle_deg"] == 90.0 for p in r)
    assert all(0.0 <= p["angle_deg"] < 360.0 for p in r)
