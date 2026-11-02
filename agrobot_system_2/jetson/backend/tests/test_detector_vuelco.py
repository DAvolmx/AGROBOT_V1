import math

import numpy as np

from detector_vuelco import NORMAL, PRECAUCION, VUELCO, DetectorVuelco, inclinaciones


def _g(roll_deg=0.0, pitch_deg=0.0):
    r, p = math.radians(roll_deg), math.radians(pitch_deg)
    return (math.sin(r) * math.cos(p), math.sin(p), math.cos(r) * math.cos(p))


def _correr(det, muestras, t0=0.0, hz=10.0, rng=None):
    rng = rng or np.random.default_rng(0)
    estados = []
    for i, (acc, gyro) in enumerate(muestras):
        acc = tuple(a + rng.normal(0, 0.01) for a in acc)          # ruido: nunca dos lecturas idénticas
        estados.append(det.paso(t0 + i / hz, acc, gyro))
    return estados


def test_inclinaciones():
    b, c, t = inclinaciones(_g(30, 0))
    assert abs(b - 30) < 1e-6 and abs(c) < 1e-6 and abs(t - 30) < 1e-6
    b, c, t = inclinaciones(_g(0, -20))
    assert abs(c + 20) < 1e-6
    assert inclinaciones((0, 0, -1))[2] > 170


def test_golpes_de_los_pulsos_no_son_vuelco():
    """Lo medido en la telemetría real: golpes de ±2 g y 190 °/s durante 1–3 muestras al arrancar cada pulso."""
    rng = np.random.default_rng(1)
    m = []
    for k in range(600):
        if k % 12 in (0, 1, 2):
            m.append(((rng.uniform(-2.4, 2.4), rng.uniform(-0.5, 0.5), rng.uniform(-1.5, 1.9)), (rng.uniform(-60, 60), rng.uniform(-190, 190), 30)))
        else:
            m.append((_g(4, 1), (0.5, 0.3, 0.1)))
    assert VUELCO not in _correr(DetectorVuelco(), m, rng=rng)


def test_tumbado_de_lado_es_vuelco_en_menos_de_dos_segundos():
    m = [(_g(0, 0), (0, 0, 0))] * 20 + [(_g(75, 0), (0, 0, 0))] * 30
    est = _correr(DetectorVuelco(), m)
    k = est.index(VUELCO)
    assert 20 < k <= 20 + 20                       # persistencia 1,07 s + ventana: ~1,6 s con los parámetros entrenados


def test_boca_abajo_es_vuelco():
    m = [(_g(0, 0), (0, 0, 0))] * 10 + [((0.05, 0.0, -1.0), (0, 0, 0))] * 30
    assert VUELCO in _correr(DetectorVuelco(), m)


def test_caida_de_morro():
    m = [(_g(0, 0), (0, 0, 0))] * 10 + [(_g(0, 70), (0, 0, 0))] * 30
    assert VUELCO in _correr(DetectorVuelco(), m)


def test_paquetes_congelados_no_cuentan():
    """El nodo del IMU repite el mismo valor 2–5 s (a veces el de un golpe): no es una inclinación sostenida."""
    det = DetectorVuelco()
    _correr(det, [(_g(2, 0), (0, 0, 0))] * 20)
    congelado = ((0.8, 0.39, -0.54), (-18.0, -15.0, -28.0))
    est = [det.paso(2.0 + i / 10, *congelado) for i in range(50)]
    assert VUELCO not in est


def test_talud_sostenido_es_precaucion_no_vuelco():
    m = [(_g(0, 0), (0, 0, 0))] * 10 + [(_g(22, 5), (0, 0, 0))] * 60
    est = _correr(DetectorVuelco(), m)
    assert PRECAUCION in est and VUELCO not in est


def test_se_libera_al_enderezarlo():
    det = DetectorVuelco()
    est = _correr(det, [(_g(80, 0), (0, 0, 0))] * 30)
    assert est[-1] == VUELCO
    est = _correr(det, [(_g(2, 0), (0, 0, 0))] * 50, t0=3.0)
    assert est[-1] == NORMAL and det.vuelcos == 1


def test_sin_gravedad_fiable_mantiene_el_estado():
    det = DetectorVuelco()
    _correr(det, [(_g(80, 0), (0, 0, 0))] * 30)
    assert det.estado == VUELCO
    rng = np.random.default_rng(3)
    est = _correr(det, [((rng.uniform(1.5, 2.5), 0, 0), (200, 200, 0)) for _ in range(40)], t0=3.0)
    assert est[-1] == VUELCO                       # solo golpes: no se libera por no tener medidas

