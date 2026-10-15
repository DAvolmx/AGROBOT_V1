"""Puente backend <-> C3 (c3_setup.py) contra una C3 SIMULADA.

La C3 simulada implementa el contrato HTTP del firmware demo-trasero (el que
exige c3_setup.py): /status, /claim, /motor, /map, /invert, /verify, /jetson,
/actuator, /config, /config/reset, /halt, /stop, con autenticacion Basic.

QUE VALIDA Y QUE NO: valida el lado Jetson (lista blanca de acciones, validacion
de parametros ANTES de reenviar, puerta por version de firmware, parada previa,
rutas). NO valida el firmware real ni ningun motor: eso solo se comprueba con la
placa. Las pruebas del firmware estan en firmware/tests_firmware_1_2.
"""
import asyncio
import sys
from pathlib import Path

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import c3_setup as mod


class C3Falsa:
    """Registra lo que le llega. `firmware` se puede cambiar por prueba."""
    def __init__(self, firmware="demo-trasero-1.2"):
        self.firmware = firmware
        self.llamadas = []

    def app(self):
        a = web.Application()
        a.router.add_get("/status", self.status)
        for ruta in ("claim", "stop", "halt", "motor", "map", "invert", "verify",
                     "jetson", "actuator", "drive", "config", "config/reset"):
            a.router.add_post("/" + ruta, self.generico)
        return a

    def _auth(self, request):
        import base64
        h = request.headers.get("Authorization", "")
        esperado = "Basic " + base64.b64encode(b"agrobot:12345678").decode()
        return h == esperado

    async def status(self, request):
        if not self._auth(request):
            return web.Response(status=401, text="no autorizado")
        return web.json_response({
            "mode": "WEB", "verified": True, "fault": False,
            "firmware": self.firmware, "motors": [],
            "cfg": {"fwd": 60, "rev": 35, "minp": 0, "inner": 65, "slew": 60, "gap": 25}})

    async def generico(self, request):
        if not self._auth(request):
            return web.Response(status=401, text="no autorizado")
        cuerpo = dict(await request.post())
        self.llamadas.append((request.path, cuerpo))
        return web.Response(text="OK")


def _correr(firmware, escenario):
    """Levanta C3 falsa + backend con el puente real y ejecuta `escenario`."""
    async def main():
        c3 = C3Falsa(firmware)
        servidor_c3 = TestServer(c3.app())
        await servidor_c3.start_server()
        puente = mod.C3Setup()
        puente.url = f"http://{servidor_c3.host}:{servidor_c3.port}"
        paradas = []
        puente.on_claim = lambda: paradas.append(True)
        app = web.Application()
        app.router.add_get("/api/c3/setup", puente.get_handler)
        app.router.add_post("/api/c3/setup/{action}", puente.post_handler)
        cliente = TestClient(TestServer(app))
        await cliente.start_server()
        try:
            return await escenario(cliente, c3, paradas, puente)
        finally:
            await cliente.close()
            await servidor_c3.close()
    return asyncio.run(main())


# --- compatibilidad de firmware ------------------------------------------

@pytest.mark.parametrize("fw,compatible,config", [
    ("demo-trasero-1.2", True, True),
    ("demo-trasero-1.1", True, False),
    ("demo-trasero-1.0", False, False),
    ("v6", False, False),
])
def test_compatibilidad_por_version(fw, compatible, config):
    async def esc(cli, c3, paradas, puente):
        r = await cli.get("/api/c3/setup")
        d = await r.json()
        assert d["compatible"] is compatible
        assert d["config_disponible"] is config
    _correr(fw, esc)


# --- configuracion de limites --------------------------------------------

def test_config_valida_se_reenvia_con_enteros_y_para_antes():
    async def esc(cli, c3, paradas, puente):
        r = await cli.post("/api/c3/setup/config", json={"fwd": 80, "minp": 30})
        assert r.status == 200, await r.text()
        assert paradas, "debe parar el robot ANTES de cambiar limites"
        ruta, cuerpo = c3.llamadas[-1]
        assert ruta == "/config"
        assert cuerpo == {"fwd": "80", "minp": "30"}
    _correr("demo-trasero-1.2", esc)


@pytest.mark.parametrize("body", [
    {"fwd": 5}, {"fwd": 101}, {"rev": 0}, {"minp": 61}, {"minp": -1},
    {"inner": 91}, {"slew": 10}, {"slew": 301}, {"gap": 9}, {"gap": 201},
    {"fwd": "abc"}, {"fwd": 50, "minp": 51},
])
def test_config_fuera_de_rango_no_llega_a_la_c3(body):
    async def esc(cli, c3, paradas, puente):
        r = await cli.post("/api/c3/setup/config", json=body)
        assert r.status == 400, (body, await r.text())
        assert not [l for l in c3.llamadas if l[0] == "/config"], \
            "un valor invalido no debe reenviarse a la C3"
    _correr("demo-trasero-1.2", esc)


def test_config_vacia_se_rechaza():
    async def esc(cli, c3, paradas, puente):
        r = await cli.post("/api/c3/setup/config", json={})
        assert r.status == 400
    _correr("demo-trasero-1.2", esc)


def test_config_con_campo_desconocido_o_booleano_se_rechaza():
    async def esc(cli, c3, paradas, puente):
        assert (await cli.post("/api/c3/setup/config", json={"pin": 3})).status == 400
        assert (await cli.post("/api/c3/setup/config", json={"fwd": True})).status == 400
    _correr("demo-trasero-1.2", esc)


def test_config_con_firmware_1_1_se_rechaza_con_409():
    async def esc(cli, c3, paradas, puente):
        r = await cli.post("/api/c3/setup/config", json={"fwd": 80})
        assert r.status == 409
        assert not [l for l in c3.llamadas if l[0] == "/config"]
    _correr("demo-trasero-1.1", esc)


def test_config_reset_usa_su_ruta_y_para():
    async def esc(cli, c3, paradas, puente):
        r = await cli.post("/api/c3/setup/config_reset", json={})
        assert r.status == 200
        assert c3.llamadas[-1][0] == "/config/reset"
        assert paradas
    _correr("demo-trasero-1.2", esc)


# --- seguridad del puente -------------------------------------------------

def test_accion_fuera_de_la_lista_blanca_es_404():
    async def esc(cli, c3, paradas, puente):
        assert (await cli.post("/api/c3/setup/formatear", json={})).status == 404
        assert not c3.llamadas
    _correr("demo-trasero-1.2", esc)


def test_claim_e_invert_paran_el_robot():
    async def esc(cli, c3, paradas, puente):
        await cli.post("/api/c3/setup/claim", json={})
        await cli.post("/api/c3/setup/invert", json={"m": 1})
        assert len(paradas) == 2
    _correr("demo-trasero-1.2", esc)


def test_el_mapeo_se_valida_en_la_c3_y_el_puente_solo_reenvia_ints():
    async def esc(cli, c3, paradas, puente):
        r = await cli.post("/api/c3/setup/map", json={"r0": 0, "r1": 1, "r2": 2, "r3": 3})
        assert r.status == 200
        assert (await cli.post("/api/c3/setup/map", json={"r0": "x; rm"})).status in (200, 400)
        assert (await cli.post("/api/c3/setup/map", json={"r0": 1.5})).status == 400
    _correr("demo-trasero-1.2", esc)


def test_credenciales_incorrectas_no_dan_acceso():
    async def esc(cli, c3, paradas, puente):
        import aiohttp
        puente.auth = aiohttp.BasicAuth("agrobot", "mala")
        r = await cli.get("/api/c3/setup")
        assert r.status == 503
        d = await r.json()
        assert d["compatible"] is False
    _correr("demo-trasero-1.2", esc)


def test_c3_apagada_se_reporta_sin_romper():
    async def main():
        puente = mod.C3Setup()
        puente.url = "http://127.0.0.1:9"      # nadie escucha
        app = web.Application()
        app.router.add_get("/api/c3/setup", puente.get_handler)
        cli = TestClient(TestServer(app)); await cli.start_server()
        try:
            r = await cli.get("/api/c3/setup")
            assert r.status == 503
            assert (await r.json())["connected"] is False
        finally:
            await cli.close()
    asyncio.run(main())
