import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "jetson" / "backend"))
import rumbo_fusion as R


def test_giroscopio_con_sesgo_deriva_y_la_referencia_lo_corrige():
    rng = random.Random(1)
    f = R.RumboFusion()
    sesgo = math.radians(0.8)
    e_real = 0.0
    for k in range(3000):                                  # 60 s a 50 Hz, el robot parado con el giro marcando +0,8 °/s
        f.predecir(sesgo + rng.gauss(0, math.radians(0.2)), 0.02)
        if k % 10 == 0:
            f.actualizar(e_real + rng.gauss(0, math.radians(3)), math.radians(3))
    assert abs(math.degrees(f.e)) < 2.5
    assert abs(f.b - sesgo) < math.radians(0.3)


def test_sin_referencias_solo_integra():
    f = R.RumboFusion()
    for _ in range(50):
        f.predecir(math.radians(10), 0.02)
    assert abs(math.degrees(f.e) - 10) < 0.01


def test_nuevo_objetivo_no_deja_error():
    f = R.RumboFusion()
    for _ in range(50):
        f.predecir(math.radians(90), 0.02)                  # gira 90° a propósito
    f.nuevo_objetivo(math.radians(90))
    assert abs(f.e) < 1e-9


def test_angulo_pared_recta_y_pendiente():
    # pared a la izquierda a y=0,5 pero el robot apunta 10° a la derecha de ella: la pared se ve subir hacia la izquierda
    ang = math.radians(10)
    pts = [(x, 0.5 + x * math.tan(ang)) for x in [i * 0.1 for i in range(-4, 11)]]
    e, s = R.angulo_pared(pts)
    assert abs(math.degrees(e) + 10) < 0.5               # el robot apunta a la derecha -> e negativo
    assert s > 0


def test_angulo_pared_rechaza_puntos_dispersos():
    rng = random.Random(3)
    pts = [(i * 0.1, rng.uniform(-1, 1)) for i in range(-4, 11)]
    assert R.angulo_pared(pts) is None


def test_corrector_zona_muerta_y_lado():
    c = R.CorrectorRumbo(zona_muerta_deg=3)
    assert c.decidir(math.radians(1)) is None
    assert c.decidir(math.radians(6)) == "derecha"        # apunta a la izquierda -> corrige a la derecha
    assert c.decidir(math.radians(-6)) == "izquierda"


def test_corrector_aprende_el_sesgo_y_anticipa():
    c = R.CorrectorRumbo(zona_muerta_deg=3)
    e = 0.0
    for _ in range(12):
        c.inicio_pulso_recto(e)
        e += math.radians(1.5)                            # cae 1,5° a la izquierda en cada pulso
        c.fin_pulso_recto(e)
    assert abs(math.degrees(c.sesgo) - 1.5) < 0.2
    assert c.decidir(math.radians(1.8)) == "derecha"      # 1,8 + 1,5 > 3: corrige antes de pasarse
