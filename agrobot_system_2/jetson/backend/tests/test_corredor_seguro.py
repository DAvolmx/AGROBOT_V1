"""Corredor seguro: que cabe la envolvente, y parada por incertidumbre.

Cubre los dos puntos ciegos corregidos el 2026-10-14:

  1. El corredor se aceptaba desde 20 cm de ancho cuando el eje TRASERO mide
     37: el robot entraba y el trasero pisaba la hilera.
  2. "Sin referencia" devolvía lo mismo que "centrado" (None), así que la
     autonomía seguía recto a ciegas.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import geometria_robot as geo
import sendero


# --- 1. Envolvente -------------------------------------------------------

def test_envolvente_la_manda_el_eje_delantero():
    # Pivote movido atrás el 2026-10-15: el eje ancho es el delantero.
    assert geo.ANCHO_ENVOLVENTE_M == geo.ANCHO_DELANTERO_M == 0.37
    assert geo.ANCHO_TRASERO_M == 0.19


def test_no_cabe_por_donde_pasa_solo_el_morro():
    # 25 cm: pasa el módulo estrecho (19) y NO el eje ancho (37).
    assert not geo.cabe(0.25)
    # Justo la envolvente, sin holgura para el seguimiento: tampoco.
    assert not geo.cabe(0.37)
    # Con la holgura medida (19 cm/lado) hace falta un pasillo de 75 cm.
    assert not geo.cabe(0.49)
    assert geo.cabe(0.65)
    assert geo.cabe(0.75)
    assert geo.cabe(0.90)
    # Una holgura explícita más suelta sí deja pasar 49 cm.
    assert geo.cabe(0.49, holgura_m=0.06)


def test_margen_cambia_de_signo_en_la_envolvente():
    assert geo.margen(0.37) == 0.0
    assert geo.margen(0.25) < 0
    assert round(geo.margen(0.49), 3) == 0.06


def test_la_holgura_sale_del_pico_de_desvio_no_del_rmse():
    # El RMSE (6 cm) subestima 2,6x lo que hace falta: manda el PICO.
    assert geo.PICO_DESVIO_LATERAL_M > geo.RMSE_DESVIO_LATERAL_M
    assert geo.HOLGURA_LATERAL_M > geo.RMSE_DESVIO_LATERAL_M * 2
    assert geo.HOLGURA_PROCEDENCIA


def test_el_minimo_de_sendero_py_no_es_transitabilidad():
    # El piso de plausibilidad es más bajo que lo que cabe, a propósito:
    # una medida de 25 cm es real y hay que reportarla, no descartarla.
    assert sendero.ANCHO_MIN_M < geo.ANCHO_CORREDOR_MIN_M


# --- 2. Parada por sendero estrecho y por incertidumbre ------------------

class _VisionFalsa:
    def __init__(self, cabe=None):
        self.cabe = cabe
        self.lateral_m = self.angulo_deg = self.error_m = None
        self.ancho_m = self.margen_m = None
        self.confianza = 0.0
        self.estado = "falsa"


def _seguidor(cabe=None):
    s = sendero.SeguidorSendero(lambda: {}, lambda: None)
    s.vision = _VisionFalsa(cabe)
    return s


CFG = {"sendero_activo": True, "sendero_parada_incierta_ciclos": 3}


def test_por_defecto_el_sendero_estrecho_avisa_sin_frenar():
    # Decisión del operador: primero recoger datos, después decidir.
    assert _seguidor(cabe=False).estrecho(CFG) == "aviso"


def test_modo_frenar_para():
    assert _seguidor(cabe=False).estrecho({**CFG, "sendero_si_no_cabe": "frenar"}) == "frenar"


def test_no_avisa_si_cabe_ni_si_no_hay_medida():
    assert _seguidor(cabe=True).estrecho(CFG) == ""
    # None = no se midió el ancho. No saber no es no caber.
    assert _seguidor(cabe=None).estrecho(CFG) == ""


def test_la_comprobacion_de_estrechez_se_puede_desactivar():
    assert _seguidor(cabe=False).estrecho({**CFG, "sendero_si_no_cabe": "off"}) == ""


def test_sendero_inactivo_no_frena_nunca():
    apagado = {"sendero_activo": False, "sendero_parada_incierta_ciclos": 1}
    s = _seguidor(cabe=False)
    s.ciclos_sin_referencia = 99
    assert s.estrecho(apagado) == ""
    assert not s.incierto(apagado)


def test_incierto_exige_una_racha_no_un_fotograma():
    s = _seguidor()
    for esperado in (False, False, True):
        s.ciclos_sin_referencia += 1
        assert s.incierto(CFG) is esperado


def test_cero_ciclos_desactiva_la_parada_por_incertidumbre():
    s = _seguidor()
    s.ciclos_sin_referencia = 50
    assert not s.incierto({**CFG, "sendero_parada_incierta_ciclos": 0})


# --- 3. La config por defecto trae las dos protecciones activas ----------

def test_defaults_de_config():
    import config_store
    d = config_store.DEFAULTS["autonomia"]
    assert d["sendero_parada_incierta_ciclos"] >= 1
    assert d["sendero_si_no_cabe"] == "aviso"
