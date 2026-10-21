"""Controlador continuo de pasillo y giro en el sitio opcional (sin hardware)."""
import math
import control_pasillo as C
import manual_demo as M


def _c(**kw):
    return C.PasilloController(**kw)


def test_centrado_no_gira():
    c = _c()
    for _ in range(40):
        v, w = c.step({"adelante": 3.0, "izquierda": 0.12, "derecha": 0.12}, 0.0)
    assert v > 0 and abs(w) < 0.02


def test_pegado_a_la_derecha_gira_a_la_izquierda_y_viceversa():
    a, b = _c(), _c()
    for _ in range(20):
        _, wa = a.step({"adelante": 3.0, "izquierda": 0.30, "derecha": 0.05}, 0.0)   # sobra sitio a la izquierda
        _, wb = b.step({"adelante": 3.0, "izquierda": 0.05, "derecha": 0.30}, 0.0)
    assert wa > 0.02 and wb < -0.02


def test_bloqueado_delante_gira_en_el_sitio_hacia_el_lado_libre_con_histeresis():
    c = _c()
    for _ in range(10):
        v, w = c.step({"adelante": 0.2, "izquierda": 1.5, "derecha": 0.2}, 0.0)
    assert v == 0 and w > 0.2                           # más libre a la izquierda
    # sigue girando hacia el mismo lado aunque ahora el otro parezca mejor (no oscila)
    v, w = c.step({"adelante": 0.3, "izquierda": 0.2, "derecha": 1.5}, 0.0)
    assert v == 0 and w > 0
    v, w = c.step({"adelante": 2.0, "izquierda": 0.2, "derecha": 1.5}, 0.0)         # despejado: avanza
    assert v > 0


def test_sala_grande_no_persigue_una_pared_lejana():
    c = _c()
    for _ in range(200):
        v, w = c.step({"adelante": 3.0, "izquierda": 2.0, "derecha": 0.95}, 0.0)
    assert abs(w) < 0.05 and abs(c.ref - c.yaw) < 0.05


def test_sin_paredes_mantiene_el_rumbo_y_corrige_la_deriva():
    c = _c()
    yaw = 0.0
    for _ in range(100):                                # el chasis deriva a +0,1 rad/s
        v, w = c.step({"adelante": 4.0, "izquierda": None, "derecha": None}, 0.1)
        yaw += 0.1 * 0.05
    assert w < 0 and c.yaw > 0                          # ordena girar en contra de la deriva


def test_salidas_acotadas_y_finitas():
    c = _c(w_max=0.6)
    for d in ({"adelante": None, "izquierda": None, "derecha": None},
              {"adelante": 0.0, "izquierda": 0.0, "derecha": 0.0},
              {"adelante": 5.0, "izquierda": 0.0, "derecha": 5.0}):
        for yr in (-3.0, 0.0, 3.0):
            v, w = c.step(d, yr)
            assert math.isfinite(v) and math.isfinite(w) and abs(w) <= 0.6 + 1e-9 and 0 <= v <= 0.31


def test_pivote_en_el_sitio_es_opcional_y_apagado_por_defecto():
    assert M.command(0.0, 0.5) == (0.0, 0.0)                       # como siempre
    assert M.reference(0.0, 0.5) == (0.0, 0.0)
    assert M.command(0.0, 0.5, pivote=True) == (0.0, 0.5 * M.W_SCALE)
    assert M.command(0.0, 0.005, pivote=True) == (0.0, 0.0)        # un ruido de giro no mueve nada
    assert M.command(0.5, 0.5) == M.command(0.5, 0.5, pivote=False)  # avanzar no cambia
