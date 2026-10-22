import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import suelo_libre as SL


def _res(seguidas, objeto=True):
    return {"t": 0.0, "puntos": [], "referencia_dudosa": False, "objeto_en_referencia": objeto, "extrana_seguidas": seguidas}


def test_un_destello_no_da_hueco_cero():
    s = SL.SueloLibre(reloj=lambda: 0.0)
    s.ultimo = _res(1)
    assert s.hueco_delante(0.38) is None


def test_objeto_persistente_da_hueco_cero():
    s = SL.SueloLibre(reloj=lambda: 0.0)
    s.ultimo = _res(SL.EXTRANA_MIN)
    assert s.hueco_delante(0.38) == 0.0
