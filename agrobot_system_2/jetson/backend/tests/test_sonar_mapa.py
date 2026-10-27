"""Sonar sobre el mapa acumulado (sin robot).   python3 -m pytest tests/test_sonar_mapa.py -q"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planificador import Planificador  # noqa: E402
from sonar_mapa import Sonar  # noqa: E402
from test_planificador import _scan  # noqa: E402


def _mapa(paredes, poses):
    p = Planificador(centro=(2, 0))
    for (x, y, psi) in poses:
        p.actualizar(x, y, psi, _scan(paredes, x, y, psi))
    return p


PASILLO = [(-1, -0.6, 8, -0.6), (-1, 0.6, 8, 0.6), (-1, -0.6, -1, 0.6)]          # pasillo de 1,2 m cerrado atrás, abierto adelante


def test_ve_el_hueco_adelante_y_el_ancho_del_pasillo():
    p = _mapa(PASILLO, [(0.0, 0, 0), (1.0, 0, 0), (2.0, 0, 0)])
    s = Sonar(p)
    a = s.analisis(2.0, 0.0, 0.0, _scan(PASILLO, 2.0, 0.0, 0.0))
    assert not a["sin_salida"] and any(abs(h["centro_deg"]) <= 10 and h["prof_m"] > 2.0 for h in a["huecos"])
    assert a["pasillo"]["es_pasillo"] and abs(a["pasillo"]["ancho_m"] - 1.2) < 0.25
    assert min(abs(math.degrees(a["pasillo"]["eje_rel_rad"])) % 180, 180 - abs(math.degrees(a["pasillo"]["eje_rel_rad"])) % 180) < 12     # el eje vale módulo 180°


def test_recuerda_lo_que_ya_no_esta_en_el_scan():
    """Tras girar 90° a la izquierda el scan ya no ve el fondo del pasillo, pero el MAPA sí: el sonar lo mantiene."""
    p = _mapa(PASILLO, [(0.0, 0, 0), (1.0, 0, 0), (2.0, 0, 0)])
    s = Sonar(p)
    r_mapa = s.perfil(2.0, 0.0, math.pi / 2, scan=[])                          # sin scan, solo mapa
    i_atras_pasillo = int(round((-math.pi / 2 + math.pi) / (2 * math.pi) * s.n)) % s.n      # dirección del eje (hacia -x del mundo) relativa al robot
    assert r_mapa.max() >= 3.0 and r_mapa.min() < 1.0                          # lejos a lo largo del pasillo, cerca a los lados


def test_callejon_sin_salida():
    cerrado = PASILLO + [(1.5, -0.6, 1.5, 0.6)]
    p = _mapa(cerrado, [(0.0, 0, 0), (0.5, 0, 0)])
    s = Sonar(p)
    r = s.perfil(0.5, 0.0, 0.0, _scan(cerrado, 0.5, 0.0, 0.0))
    # el pasillo sigue abierto hacia atrás? está cerrado en x=-1 (a 1,5 m): con prof_min 2 m no hay hueco en ninguna dirección
    assert s.sin_salida(s.huecos(r, prof_min=2.0))


def test_mejor_rumbo_prefiere_la_meta_y_penaliza_lo_pisado():
    p = _mapa([(-3, -3, 3, -3), (-3, 3, 3, 3), (-3, -3, -3, 3), (3, -3, 3, 3)], [(0, 0, 0)])
    s = Sonar(p, r_max=2.5)
    r = s.perfil(0.0, 0.0, 0.0, [])
    g = s.huecos(r, 1.0)
    c, _ = s.mejor_rumbo(r, g, meta_rel=math.radians(80))
    assert c is not None and abs(math.degrees(c) - 80) < 25


def test_mejor_rumbo_con_meta_en_un_arco_parcial_no_falla():
    """Regresión: `margin` sin definir hacía fallar TODO episodio con meta (los barridos del 2026-10-26 salieron mal por esto)."""
    p = _mapa(PASILLO, [(0.0, 0, 0), (1.0, 0, 0), (2.0, 0, 0)])
    s = Sonar(p)
    r = s.perfil(2.0, 0.0, 0.0, _scan(PASILLO, 2.0, 0.0, 0.0))
    g = s.huecos(r, 1.0)
    c, h = s.mejor_rumbo(r, g, meta_rel=math.radians(10), lado_w=1.0, r_izq=0.5, r_der=0.5)
    assert c is not None and abs(math.degrees(c)) < 30
