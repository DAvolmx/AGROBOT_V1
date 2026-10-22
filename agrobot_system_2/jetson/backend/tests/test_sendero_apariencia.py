import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import cv2
import sendero
import sendero_apariencia as A

GEO = {"camara_altura_m": 0.4, "camara_inclinacion_deg": 15.5, "camara_hfov_deg": 71, "camara_adelante_m": 0.42}


def _escena():
    """Camino liso gris en el centro, camas con textura fuerte a los lados (perspectiva aproximada)."""
    rng = np.random.default_rng(0)
    img = (rng.integers(40, 200, (240, 320, 3))).astype(np.uint8)            # camas: ruido = textura
    pts = np.array([[110, 240], [210, 240], [175, 80], [145, 80]], np.int32)   # sendero que se estrecha hacia el horizonte
    cv2.fillPoly(img, [pts], (110, 112, 115))
    return img


def test_detecta_camino_liso_entre_camas_con_textura():
    mem = A.MemoriaApariencia()
    cen = sendero.Cenital(GEO)
    r = None
    for _ in range(3):
        r = A.detectar_apariencia(cv2, _escena(), {"sendero_apariencia_umbral": 1.4, "sendero_apariencia_textura": 2.5}, GEO, cen, mem)
    assert r["lateral_m"] is not None, r["estado"]
    assert abs(r["lateral_m"]) < 0.12
    assert 0.15 < r["ancho_m"] < 1.0


def test_vision_sendero_elige_el_metodo_por_config():
    vs = sendero.VisionSendero()
    r = vs.detectar(cv2, _escena(), {"sendero_metodo": "apariencia"}, GEO)
    assert r.get("metodo") == "apariencia"
