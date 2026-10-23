"""Detección trasera (2026-10-23): el sector de atrás es más ancho que el frontal para cubrir el arco al retroceder en curva."""
import math
from slam import LidarReactive

CFG = {"robot_largo_cm": 49, "robot_ancho_cm": 38, "cono_frontal_deg": 45, "cono_lateral_deg": 45,
       "lidar_adelante_cm": 18.5, "lidar_atras_cm": 30.5, "corredor_holgura_cm": 17}


def _lidar(puntos):
    l = LidarReactive()
    l.ingest_scan([{"angle_deg": a, "distance_m": d} for a, d in puntos])
    return l


def test_obstaculo_atras_en_diagonal_se_ve():
    # a 150° (30° fuera del eje trasero) y 0,7 m: antes (cono 45°) caía en ningún sector
    d = _lidar([(150.0, 0.70)]).distancias(CFG)
    assert d["atras"] is not None and d["atras"] < 0.45


def test_el_cono_trasero_se_puede_estrechar_por_config():
    d = _lidar([(150.0, 0.70)]).distancias({**CFG, "cono_trasero_deg": 45})
    assert d["atras"] is None


def test_lo_de_atras_no_cambia_el_frente():
    d = _lidar([(0.0, 1.0), (180.0, 0.6)]).distancias(CFG)
    assert abs(d["adelante"] - (1.0 - 0.185)) < 1e-3 and abs(d["atras"] - (0.6 - 0.305)) < 1e-3


def test_pared_paralela_lejos_del_costado_no_bloquea_atras():
    # pared a 0,6 m del eje (lateral >> semiancho + holgura): no es el camino de retroceso
    d = _lidar([(165.0, 0.6 / math.sin(math.radians(15)))]).distancias(CFG)
    assert d["atras"] is None
