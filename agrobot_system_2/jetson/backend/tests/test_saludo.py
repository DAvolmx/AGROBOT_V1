"""El saludo hace los pasos en orden, apaga el foco y deja la cámara en reposo, también si lo cancelan."""
import asyncio
import saludo as m
from protocol import SERVO_HOME_DEG


def _montar(falla_en=None):
    ev = []

    async def girar(a):
        ev.append(("girar", a))
        if falla_en == a:
            raise RuntimeError("x")

    async def servo(a): ev.append(("servo", a))
    async def foco(e): ev.append(("foco", e))
    async def dormir(t): ev.append(("t", t))
    return ev, m.Saludo(girar, servo, foco, dormir)


def test_orden_del_saludo():
    ev, s = _montar()
    pasos = asyncio.run(s.ejecutar())
    assert pasos == ["izquierda", "derecha", "mirar_arriba", "foco", "barrido"]
    assert [e for e in ev if e[0] == "girar"] == [("girar", "izquierda"), ("girar", "derecha")]
    assert ev.index(("servo", m.ANG_ARRIBA)) > ev.index(("girar", "derecha"))             # mira arriba después de girar
    focos = [e[1] for e in ev if e[0] == "foco"]
    assert focos[:6] == [True, False, True, False, True, False]                          # 3 parpadeos
    assert focos.count(True) == 3 and focos[-1] is False
    servos = [e[1] for e in ev if e[0] == "servo"]
    assert min(servos) == m.BARRIDO_MIN and servos[-1] == SERVO_HOME_DEG                 # barrido y vuelta a reposo
    assert all(25 <= a <= 110 for a in servos[:-1])


def test_si_falla_un_paso_deja_el_foco_apagado_y_la_camara_en_reposo():
    ev, s = _montar(falla_en="derecha")
    try:
        asyncio.run(s.ejecutar())
        assert False
    except RuntimeError:
        pass
    assert ev[-2:] == [("foco", False), ("servo", SERVO_HOME_DEG)]


def test_si_lo_cancelan_tambien_limpia():
    async def prueba():
        ev = []

        async def girar(a): ev.append(("girar", a))
        async def servo(a): ev.append(("servo", a))
        async def foco(e): ev.append(("foco", e))
        s = m.Saludo(girar, servo, foco, dormir=asyncio.sleep, t_mirar=5.0)
        t = asyncio.create_task(s.ejecutar())
        await asyncio.sleep(0.05)                      # está en "mirar arriba"
        t.cancel()
        try:
            await t
        except asyncio.CancelledError:
            pass
        return ev
    ev = asyncio.run(prueba())
    assert ev[-2:] == [("foco", False), ("servo", SERVO_HOME_DEG)]
