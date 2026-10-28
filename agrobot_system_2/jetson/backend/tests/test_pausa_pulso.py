import asyncio
import time

import explorador as X


def _expl(cb):
    e = object.__new__(X.ExploradorReactivo)
    e.activo, e.pausado, e.ocupado_cb = True, False, cb
    return e


def test_sin_callback_solo_espera_la_base():
    t0 = time.monotonic()
    asyncio.run(_expl(None)._pausa_pulso(base=0.05))
    assert time.monotonic() - t0 < 0.3


def test_espera_a_que_el_conductor_quede_libre():
    estado = {"hasta": time.monotonic() + 0.6}
    e = _expl(lambda: time.monotonic() < estado["hasta"])
    t0 = time.monotonic()
    asyncio.run(e._pausa_pulso(base=0.05))
    assert time.monotonic() - t0 >= 0.55


def test_tiene_tope_si_el_conductor_no_se_libera():
    e = _expl(lambda: True)
    t0 = time.monotonic()
    asyncio.run(e._pausa_pulso(base=0.05, tope=0.4))
    assert 0.35 <= time.monotonic() - t0 < 1.0


def test_si_el_explorador_se_para_no_espera():
    e = _expl(lambda: True)
    e.activo = False
    t0 = time.monotonic()
    asyncio.run(e._pausa_pulso(base=0.05, tope=3.0))
    assert time.monotonic() - t0 < 0.5
