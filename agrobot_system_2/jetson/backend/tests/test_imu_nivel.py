import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from imu_nivel import aplicar, inclinacion_deg, matriz_nivel, promediar


def test_la_gravedad_de_nivel_queda_vertical():
    g = (0.25, 0.01, 0.99)                       # lo medido con el robot nivelado en la mesa
    R = matriz_nivel(g)
    x, y, z = aplicar(R, g)
    n = math.sqrt(sum(v * v for v in g))
    assert abs(x) < 1e-9 and abs(y) < 1e-9 and abs(z - n) < 1e-9


def test_conserva_la_norma_y_es_una_rotacion():
    R = matriz_nivel((0.25, 0.01, 0.99))
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert abs(np.linalg.det(R) - 1) < 1e-9
    v = (0.1, -0.2, 0.9)
    assert abs(np.linalg.norm(aplicar(R, v)) - np.linalg.norm(v)) < 1e-9


def test_ya_nivelado_no_cambia_nada():
    R = matriz_nivel((0.0, 0.0, 1.0))
    assert np.allclose(R, np.eye(3))


def test_vectores_invalidos():
    assert matriz_nivel((0, 0, 0)) is None
    assert matriz_nivel((0, 0, -1)) is None          # boca abajo
    assert matriz_nivel((float("nan"), 0, 1)) is None
    assert matriz_nivel((1, 1, 1)) is None           # |g| = 1,73
    assert aplicar(None, (1, 2, 3)) == (1, 2, 3)


def test_inclinacion_del_montaje():
    roll, pitch = inclinacion_deg((0.25, 0.01, 0.99))
    assert 14 < roll < 14.5 and abs(pitch) < 1


def test_promediar_rechaza_movimiento_y_pocas_muestras():
    quieto = [(0.25, 0.01, 0.99)] * 30
    m, err = promediar(quieto)
    assert err is None and m == [0.25, 0.01, 0.99]
    mov = [(0.25 + 0.2 * (-1) ** i, 0.01, 0.99) for i in range(30)]
    assert promediar(mov)[0] is None
    assert promediar(quieto[:3])[0] is None
