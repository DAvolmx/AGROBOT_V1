import cv2
import numpy as np

from camera import dibujar_cajas


def _borde_verde(img):
    return int(((img[:, :, 1] == 200) & (img[:, :, 0] == 60) & (img[:, :, 2] == 90)).sum())


def test_caja_de_plaga_de_imagen_completa_no_dibuja_marco():
    img = np.zeros((480, 640, 3), np.uint8)
    dibujar_cajas(cv2, img, [{"class": "col_amarillamiento", "confidence": 0.5, "xyxy": (0, 0, 639, 479)}], 0.55, ["maceta"])
    assert _borde_verde(img) == 0
    assert img[430:480, :60].any()                  # el rótulo sí aparece abajo a la izquierda


def test_caja_pequena_de_plaga_si_se_dibuja():
    img = np.zeros((480, 640, 3), np.uint8)
    dibujar_cajas(cv2, img, [{"class": "col_amarillamiento", "confidence": 0.5, "xyxy": (100, 100, 200, 200)}], 0.55, ["maceta"])
    assert _borde_verde(img) > 100


def test_maceta_grande_siempre_se_dibuja():
    img = np.zeros((480, 640, 3), np.uint8)
    dibujar_cajas(cv2, img, [{"class": "maceta", "confidence": 0.9, "xyxy": (10, 10, 630, 470)}], 0.55, ["maceta"])
    assert _borde_verde(img) > 500
