"""Tramas de configuración de la C3 y validación de los endpoints (sin hardware)."""
import asyncio, json
import c3_pines as m
from ble_link import crc8


class FakeWifi:
    conectado = True
    def __init__(self, pines):
        self.enviadas, self.pines, self.cb = [], pines, None
    def enviar(self, t):
        self.enviadas.append(t)
        # C3 simulada mínima: responde a SET con el estado resultante
        if t[1] == m.OP_SET:
            w, fi, bi, inv = t[2:6]
            self.pines[w] = (fi, bi, bool(inv))
            self.cb(bytes([0xC6, w, fi, bi, inv, 1, 3]) + bytes([crc8(bytes([0xC6, w, fi, bi, inv, 1, 3]))]))
        return True


class Req:
    def __init__(self, d): self.d = d
    async def json(self): return self.d


def _setup():
    w = FakeWifi({})
    enlace = type("E", (), {"wifi": w})()
    p = m.C3Pines(lambda: enlace)
    w.cb = p.on_cfg
    for i, (f, b, v) in enumerate(m.PINES_DEFECTO):
        p.on_cfg(bytes([0xC6, i, f, b, int(v), 1, 3]) + bytes([crc8(bytes([0xC6, i, f, b, int(v), 1, 3]))]))
    return p, w


def test_pack_y_crc():
    t = m.pack(m.OP_SET, 2, 5, 6, 1)
    assert len(t) == 8 and t[0] == 0xC5 and crc8(t[:7]) == t[7]
    assert m.unpack_reply(t) is None                       # no es una respuesta
    r = bytes([0xC6, 1, 6, 7, 1, 1, 3]); r += bytes([crc8(r)])
    assert m.unpack_reply(r) == {"rueda": 1, "fi": 6, "bi": 7, "invertida": True, "ok": True, "version": 3}
    assert m.unpack_reply(r[:7] + b"\x00") is None          # crc malo


def run(c): return asyncio.run(c)


def test_set_valido_e_invalidos():
    p, w = _setup()
    r = run(p.set_handler(Req({"rueda": 0, "fi": 0, "bi": 1, "invertida": True})))
    assert r.status == 200 and w.enviadas[-1][1] == m.OP_SET and w.enviadas[-1][5] == 1
    n = len(w.enviadas)
    for malo in ({"rueda": 9, "fi": 0, "bi": 1}, {"rueda": 0, "fi": 8, "bi": 1}, {"rueda": 0, "fi": 3, "bi": 3},
                 {"rueda": 0, "fi": 3, "bi": 1},            # el 3 ya es de otra rueda
                 {"rueda": "x", "fi": 0, "bi": 1}, {"rueda": True, "fi": 0, "bi": 1}):
        assert run(p.set_handler(Req(malo))).status == 400, malo
    assert len(w.enviadas) == n                              # nada inválido sale a la C3


def test_prueba_valida_rangos():
    p, w = _setup()
    assert run(p.test_handler(Req({"rueda": 1, "sentido": 1, "pct": 40}))).status == 200
    t = w.enviadas[-1]; assert t[1] == m.OP_TEST and t[2:5] == bytes([1, 1, 40])
    run(p.test_handler(Req({"rueda": 1, "sentido": -1, "pct": 40}))); assert w.enviadas[-1][3] == 2
    run(p.test_handler(Req({"rueda": 1, "sentido": 0, "pct": 40}))); assert w.enviadas[-1][3] == 0
    for malo in ({"rueda": 4}, {"rueda": 0, "pct": 101}, {"rueda": 0, "sentido": 2}):
        assert run(p.test_handler(Req(malo))).status == 400


def test_sin_enlace():
    p = m.C3Pines(lambda: None)
    assert run(p.test_handler(Req({"rueda": 0, "sentido": 1, "pct": 30}))).status == 503


def test_limites_trama_y_validacion():
    r = bytes([0xC7, 3, 55, 0, 1, 4, 0]); r += bytes([crc8(r)])
    assert m.unpack_lim_reply(r) == {"id": 3, "nombre": "minp", "valor": 55, "ok": True}
    assert m.unpack_lim_reply(r[:7] + b"\x00") is None
    assert m.limites_validos({"adel": 80, "atras": 60, "minp": 50}) is None
    assert m.limites_validos({k: v[2] for k, v in m.LIMITES.items()}) is None      # defecto valido
    assert m.limites_validos({"adel": 80, "atras": 50, "minp": 60}) is not None      # minp > atras
    assert m.limites_validos({"pivote": 20}) is not None


def test_lim_set_handler_envia_y_rechaza_invalidos():
    p, w = _setup()
    p.limites.update({k: v[2] for k, v in m.LIMITES.items()})
    for malo in ({"nombre": "x", "valor": 5}, {"nombre": "adel", "valor": 5}, {"nombre": "minp", "valor": 70},
                 {"nombre": "acel", "valor": True}):
        assert run(p.lim_set_handler(Req(malo))).status == 400, malo
    assert not w.enviadas
    p2 = m.C3Pines(lambda: type("E", (), {"wifi": w})())
    p2.limites.update(p.limites)
    w.enviadas.clear()
    run(p2.lim_set_handler(Req({"nombre": "acel", "valor": 200})))     # sin respuesta de la C3: 504, pero la orden sale
    t = w.enviadas[-1]
    assert t[1] == m.OP_LIM_SET and t[2] == 4 and (t[3] | t[4] << 8) == 200 and crc8(t[:7]) == t[7]
