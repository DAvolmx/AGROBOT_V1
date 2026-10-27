import cv2
import numpy as np

from suelo_libre import SueloLibre

GEO = {"camara_altura_m": 0.4, "camara_inclinacion_deg": 15.5, "camara_hfov_deg": 71.0, "camara_adelante_m": 0.42}


def _mascara(con_obstaculo):
    m = np.zeros((240, 320), np.uint8)
    m[100:, :] = 1                                   # suelo libre bajo el horizonte
    if con_obstaculo:
        m[150:190, 130:190] = 2                      # una maceta en el camino
    return m


def _frame():
    return np.full((480, 640, 3), 120, np.uint8)


def test_modelo_sin_obstaculo_no_da_puntos():
    s = SueloLibre(cfg={"metodo": "modelo"})
    s.segmentador = lambda f: _mascara(False)
    for _ in range(3):
        r = s.analizar(cv2, _frame(), GEO)
    assert r["puntos"] == []


def test_modelo_con_obstaculo_da_hueco_delante():
    s = SueloLibre(cfg={"metodo": "modelo"})
    s.segmentador = lambda f: _mascara(True)
    for _ in range(3):
        r = s.analizar(cv2, _frame(), GEO)
    assert r["puntos"]
    h = s.hueco_delante(0.5)
    assert h is not None and 0.3 < h < 2.5
