"""Presupuesto de tiempo y terminación del episodio.

Dos sesgos del banco corregidos el 2026-10-14:

  1. `t_max` era 10 s FIJO y `progreso = s_max/largo`, así que una ruta más
     larga puntuaba peor con el mismo control: el banco penalizaba rediseñar
     la ruta. Se vio al sustituir `ese` (r=1,0, imposible para el pivote) por
     `ese_amplia` (r=1,51, factible): la factible salía PEOR, solo por ser un
     51 % más larga.
  2. El episodio no terminaba al completar la ruta. `_proyecta` se satura en
     el último tramo, así que el error seguía creciendo mientras el robot se
     alejaba: el RMSE medía vagabundeo posterior. Con el presupuesto
     proporcional el artefacto creció y `recta` pasó de 0,18 a 0,66 sin que
     el control cambiase.
"""
import math

import numpy as np

import evaluacion as E


def _recta(largo):
    return [(x, 0.0) for x in np.linspace(0, largo, 20)]


# --- 1. Presupuesto proporcional ----------------------------------------

def test_el_presupuesto_crece_con_la_longitud():
    corto = E.presupuesto_s(_recta(3.0))
    largo = E.presupuesto_s(_recta(9.0))
    assert largo > corto
    assert abs(largo / corto - 3.0) < 1e-6


def test_el_presupuesto_respeta_un_minimo():
    # Una ruta muy corta no se queda sin tiempo por ser corta.
    assert E.presupuesto_s(_recta(0.2)) == E.T_MAX_MIN_S


def test_el_presupuesto_son_segundos_por_metro():
    assert E.presupuesto_s(_recta(10.0)) == 10.0 * E.SEGUNDOS_POR_METRO


def test_la_ese_imposible_no_cabia_en_los_10s_viejos():
    """El motivo por el que `ese` nunca se completaba.

    Mide 5,34 m: a la velocidad nominal del contrato (0,5 m/s) hacen falta
    10,7 s y el banco daba 10. No había tiempo material ni a tope.
    """
    ruta = E.escenarios()["ese"]["ruta"]
    largo = sum(math.dist(a, b) for a, b in zip(ruta, ruta[1:]))
    assert largo > 0.5 * 10.0          # imposible en los 10 s viejos
    assert E.presupuesto_s(ruta) > largo / 0.5   # el nuevo sí da margen


# --- 2. Terminación al completar la ruta --------------------------------

def _episodio(largo_ruta, t_max=None):
    from robot_rustico import parametros_rusticos
    esc = {"ruta": _recta(largo_ruta), "mu": 0.65, "h_obs": 0.0,
           "obs_pos": None, "mundo": E._Mundo()}
    ctrl = E.Controlador(kp=3.5, ki=2.0, v=0.5, freno=True)
    ctrl.envolvente = E.envolvente_chasis(parametros_rusticos(0))
    return E.simular(esc, ctrl, parametros_rusticos(0), 0, t_max=t_max)


def test_el_episodio_termina_antes_del_presupuesto_si_acaba_la_ruta():
    m = _episodio(2.0)
    assert m["progreso"] > 0.95
    assert m["t_usado_s"] < m["t_max_s"]


def test_dar_mas_tiempo_ya_no_empeora_el_rmse():
    """Lo que se rompía: con la ruta hecha, más tiempo = más error."""
    corto = _episodio(2.0, t_max=20.0)
    largo = _episodio(2.0, t_max=60.0)
    assert corto["progreso"] > 0.95 and largo["progreso"] > 0.95
    # Mismo episodio efectivo: corta al acabar, no al agotar el reloj.
    assert corto["t_usado_s"] == largo["t_usado_s"]
    assert corto["rmse_cm"] == largo["rmse_cm"]


def test_las_metricas_dicen_cuanto_tiempo_se_uso():
    m = _episodio(3.0)
    for k in ("t_max_s", "t_usado_s", "largo_m"):
        assert k in m


# --- 3. Factibilidad en el reporte --------------------------------------

def test_evaluar_marca_el_veredicto_de_cada_episodio():
    r = E.evaluar(lambda: E.Controlador(kp=3.5, ki=2.0, v=0.5, freno=True), [0])
    assert r["puntos_factibles"] is not None
    veredictos = {e["escenario"]: e["veredicto"] for e in r["episodios"]}
    assert veredictos["ese"] == "imposible"
    assert veredictos["curva"] == "al_limite"
    assert veredictos["ese_amplia"] == "ok"


def test_el_objetivo_factible_excluye_las_rutas_imposibles():
    r = E.evaluar(lambda: E.Controlador(kp=3.5, ki=2.0, v=0.5, freno=True), [0])
    imposibles = [e for e in r["episodios"] if e["veredicto"] == "imposible"]
    assert imposibles                      # las sigue midiendo y reportando
    # ...pero no entran en el número que se optimiza.
    assert r["puntos_factibles"] != r["puntos_medios"]
