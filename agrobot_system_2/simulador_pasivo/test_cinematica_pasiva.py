"""Radio de giro mínimo del eje pasivo y factibilidad de rutas.

Diagnóstico del 2026-10-14: dos de los seis escenarios del banco pedían
radios por debajo de lo que el pivote pasivo permite, así que su coste era un
suelo que ningún controlador podía bajar.
"""
import math

import cinematica_pasiva as K


def test_radio_minimo_sale_de_ackermann():
    # R = batalla / tan(pivote).
    assert round(K.radio_minimo(17.0, 0.37), 3) == 1.210
    assert round(K.radio_minimo(20.0, 0.37), 3) == 1.017


def test_pivote_y_radio_son_inversos():
    for r in (0.8, 1.0, 1.5, 3.0):
        ang = K.pivote_para_radio(r, 0.37)
        assert abs(K.radio_minimo(ang, 0.37) - r) < 1e-9


def test_sin_giro_el_radio_es_infinito():
    assert K.radio_minimo(0.0) == math.inf


def test_radio_local_de_una_recta_es_infinito():
    assert K.radio_local((0, 0), (1, 0), (2, 0)) == math.inf


def test_radio_local_de_un_circulo_conocido():
    # Tres puntos de un círculo de radio 2 centrado en el origen.
    pts = [(2 * math.cos(a), 2 * math.sin(a)) for a in (0.0, 0.5, 1.0)]
    assert abs(K.radio_local(*pts) - 2.0) < 1e-6


def test_una_recta_es_factible():
    r = K.factibilidad([(x, 0.0) for x in (0, 1, 2, 3)])
    assert r["veredicto"] == "ok" and r["factible"]


def _arco(radio, n=40):
    return [(radio * math.sin(a), radio - radio * math.cos(a))
            for a in [i * (math.pi / 2) / (n - 1) for i in range(n)]]


def test_el_radio_del_escenario_ese_es_imposible():
    # r=1,0 m exige ~20,3°, por encima del tope mecánico de 20°.
    r = K.factibilidad(_arco(1.0))
    assert r["veredicto"] == "imposible"
    assert not r["factible"]
    assert r["pivote_necesario_deg"] > r["pivote_tope_deg"]


def test_el_radio_del_escenario_curva_queda_al_limite():
    # r=1,2 m exige ~17,1°: pasa el límite útil pero no el tope.
    r = K.factibilidad(_arco(1.2))
    assert r["veredicto"] == "al_limite"
    assert r["factible"]          # se puede, pero sin margen para corregir


def test_el_radio_seguro_deja_margen_sobre_el_limite_util():
    r = K.factibilidad(_arco(K.radio_seguro()))
    assert r["veredicto"] == "ok"
    assert r["pivote_necesario_deg"] < r["pivote_util_deg"]


def test_el_radio_seguro_es_mayor_que_el_minimo():
    assert K.radio_seguro() > K.radio_minimo(K._contrato()["steer_limit_deg"])


def test_factibilidad_localiza_el_tramo_mas_cerrado():
    # Recta larga con una esquina cerrada al final.
    ruta = [(x, 0.0) for x in range(5)] + [(4.2, 0.3), (4.2, 1.0)]
    r = K.factibilidad(ruta)
    assert r["veredicto"] == "imposible"
    assert r["indice"] >= 4          # la esquina, no la recta
