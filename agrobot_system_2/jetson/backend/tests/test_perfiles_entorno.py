"""Perfiles de entorno: que interior deje mover el robot y que el margen de
parada no se separe nunca de la velocidad que lo justifica.

Contexto (2026-10-14, presentación de tesis en interiores): `modo_manejo` solo
cambiaba la duración de los pulsos. Los umbrales ESPACIALES seguían siendo de
huerta —100 cm libres por delante para lanzar un pulso, corte a 52 cm— y en una
sala de 4x3 m con macetas eso deja el robot clavado.

El margen de parada existe porque al soltar el mando las ruedas quedan libres
y el robot sigue rodando. Ese recorrido crece mucho con la velocidad (medido en
simulador, mu 0,8: 14 cm a 0,2 m/s, 27,5 a 0,3, 86,6 a 0,4), así que recortarlo
sin recortar la velocidad es quitarle la distancia de frenado.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import perfiles_entorno as P


# --- El perfil de interior desbloquea el movimiento en sitio chico --------

def test_interior_exige_mucho_menos_espacio_que_huerta():
    dentro, fuera = P.PERFILES["interior"], P.PERFILES["exterior"]
    assert dentro["avance_libre_cm"] < fuera["avance_libre_cm"]
    assert dentro["lado_libre_cm"] < fuera["lado_libre_cm"]
    # En una sala de 4 m el robot no puede pedir 1 m libre para moverse.
    assert dentro["avance_libre_cm"] <= 50


def test_interior_corta_mas_tarde_porque_va_mas_despacio():
    dentro, fuera = P.PERFILES["interior"], P.PERFILES["exterior"]
    corte_dentro = dentro["margen_parada_cm"] + dentro["deslizamiento_cm"]
    corte_fuera = fuera["margen_parada_cm"] + fuera["deslizamiento_cm"]
    assert corte_dentro < corte_fuera
    assert dentro["escala_velocidad"] < fuera["escala_velocidad"]


def test_los_dos_perfiles_son_coherentes():
    for nombre in P.PERFILES:
        P.validar(P.PERFILES[nombre])


# --- El acoplamiento velocidad <-> frenado no se puede romper ------------

def test_recortar_el_deslizamiento_sin_recortar_la_velocidad_se_rechaza():
    malo = {**P.PERFILES["interior"], "escala_velocidad": 0.5}
    with pytest.raises(P.PerfilIncoherente):
        P.validar(malo)


def test_la_velocidad_de_interior_esta_dentro_de_lo_medido():
    v = P.PERFILES["interior"]["escala_velocidad"] * 1.5   # control.max_v_mps
    assert v <= P.V_SEGURA_INTERIOR_MPS


def test_pedir_menos_espacio_que_el_corte_se_rechaza():
    """Arrancar un pulso que el freno corta de inmediato es no moverse."""
    malo = {**P.PERFILES["interior"], "avance_libre_cm": 10}
    with pytest.raises(P.PerfilIncoherente):
        P.validar(malo)


def test_el_avance_libre_cubre_el_corte_en_los_dos_perfiles():
    for nombre, p in P.PERFILES.items():
        corte = p["margen_parada_cm"] + p["deslizamiento_cm"]
        assert p["avance_libre_cm"] >= corte, nombre


# --- Aplicar: atómico y auditable ---------------------------------------

class _StoreFalso:
    def __init__(self, cfg):
        self.cfg = cfg
        self.escrituras = []

    def update_section(self, seccion, valores):
        self.escrituras.append((seccion, dict(valores)))
        self.cfg.setdefault(seccion, {}).update(valores)
        return self.cfg


def _config_huerta():
    return {"autonomia": {"avance_libre_cm": 100, "lado_libre_cm": 60,
                          "escala_velocidad": 0.5, "retroceso_libre_cm": 30,
                          "distancia_puntos_cm": 290,
                          "sendero_pared_max_cm": 150},
            "slam": {"deslizamiento_cm": 40, "margen_parada_cm": 12,
                     "cono_frontal_deg": 10}}


def test_diferencias_no_escribe_nada():
    cfg = _config_huerta()
    antes = {k: dict(v) for k, v in cfg.items()}
    difs = P.diferencias(cfg, "interior")
    assert difs                       # hay cambios que proponer
    assert cfg == antes               # pero no se tocó la config


def test_diferencias_dice_antes_y_despues():
    difs = P.diferencias(_config_huerta(), "interior")
    assert difs["avance_libre_cm"] == (100, P.PERFILES["interior"]["avance_libre_cm"])
    assert difs["deslizamiento_cm"][0] == 40


def test_aplicar_escribe_en_la_seccion_correcta():
    cfg = _config_huerta()
    store = _StoreFalso(cfg)
    P.aplicar(store, cfg, "interior")
    assert cfg["slam"]["deslizamiento_cm"] == 20
    assert cfg["autonomia"]["avance_libre_cm"] == 40
    secciones = {s for s, _ in store.escrituras}
    assert secciones == {"autonomia", "slam"}


def test_aplicar_un_perfil_desconocido_no_escribe():
    cfg = _config_huerta()
    store = _StoreFalso(cfg)
    with pytest.raises(KeyError):
        P.aplicar(store, cfg, "submarino")
    assert store.escrituras == []


def test_cada_clave_tiene_seccion():
    for nombre, p in P.PERFILES.items():
        for clave in p:
            assert clave in P.SECCION, f"{nombre}.{clave} sin sección"
