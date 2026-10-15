"""Escenarios de interior: macetas sólidas, pasillos y sala.

Para la defensa de tesis 2026-10-16. Lo que se comprueba aquí es la
CONSTRUCCIÓN de los escenarios (que las macetas dejen el hueco que se pide y
que sean sólidas), no el comportamiento del robot: eso lo mide
`validar_interiores.py` y depende del controlador.
"""
import math

import huella_pasiva as H
import validar_interiores as V


def test_la_maceta_es_un_objeto_solido():
    m = V._maceta(1.0, 0.5)
    x, y, r, alto, solido = m
    assert (x, y) == (1.0, 0.5)
    assert r == V.MACETA_R
    assert solido > 0.5          # huella_pasiva.choca exige o[4] > .5
    assert alto > 0.2            # alta: el LIDAR a 18 cm SÍ la ve


def test_el_pasillo_deja_el_hueco_pedido():
    for ancho in (0.45, 0.50, 0.70):
        objetos = V._pasillo_macetas(ancho)
        izq = min(o[1] for o in objetos if o[1] > 0)
        der = max(o[1] for o in objetos if o[1] < 0)
        # Entre los BORDES de las macetas, no entre sus centros.
        hueco = (izq - V.MACETA_R) - (der + V.MACETA_R)
        assert abs(hueco - ancho) < 1e-9, ancho


def test_el_pasillo_tiene_macetas_a_los_dos_lados():
    objetos = V._pasillo_macetas(0.5)
    assert any(o[1] > 0 for o in objetos)
    assert any(o[1] < 0 for o in objetos)


def test_el_eslalon_alterna_de_lado():
    objetos = V._eslalon_macetas(0.5)
    lados = [1 if o[1] > 0 else -1 for o in objetos]
    assert len(lados) >= 2
    # Alternancia estricta: obliga a corregir en los dos sentidos.
    assert all(a != b for a, b in zip(lados, lados[1:]))


def test_un_pasillo_estrecho_hace_chocar_al_robot():
    """Cordura del detector de colisiones con macetas."""
    mundo = V._MundoConMacetas = type("M", (), {})()
    import evaluacion as E
    # Pasillo imposible: 10 cm de hueco para una vía de 37.
    m = E._Mundo([], V._pasillo_macetas(0.10))
    assert H.choca(m, 0.8, 0.0, 0.0, 0.0)
    # El mismo sitio con hueco de sobra: libre.
    ancho = E._Mundo([], V._pasillo_macetas(1.2))
    assert not H.choca(ancho, 0.8, 0.0, 0.0, 0.0)


def test_los_escenarios_de_interior_existen_y_tienen_mundo():
    escs = V.escenarios(0.5)
    assert set(escs) == {"sala_vacia", "pasillo_macetas",
                         "eslalon_macetas", "esquina_maceta"}
    for nombre, e in escs.items():
        assert e["ruta"] and e["mundo"] is not None, nombre
        # Suelo de interior: agarra más que la tierra.
        assert e["mu"] >= 0.8, nombre


def test_la_sala_vacia_no_tiene_macetas():
    assert V.escenarios(0.5)["sala_vacia"]["mundo"].objetos == []


def test_las_rutas_caben_en_una_sala():
    for nombre, e in V.escenarios(0.5).items():
        largo = sum(math.dist(a, b) for a, b in zip(e["ruta"], e["ruta"][1:]))
        assert largo <= V.LARGO_INTERIOR_M + 0.05, nombre


def test_la_puntuacion_de_interior_castiga_sobre_todo_el_contacto():
    """Tocar una maceta tiene que pesar más que ir algo torcido."""
    toca = {"contactos": 1, "progreso": 1.0, "pico_cm": 5.0}
    torcido = {"contactos": 0, "progreso": 1.0, "pico_cm": 30.0}
    assert V.puntuar(toca) > V.puntuar(torcido)
