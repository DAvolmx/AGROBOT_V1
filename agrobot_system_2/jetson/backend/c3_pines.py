"""Configuración de las ruedas de la C3 (pines FI/BI, inversión) y prueba de motores.

Va por el MISMO enlace UDP de las órdenes de movimiento: la C3 no tiene web ni IP
propia, solo se conecta al dashboard. Tramas de 8 bytes con crc8 (igual que el resto):
  Jetson -> C3: [0xC5][op][a][b][c][d][e][crc]
      op 1 SET   a=rueda b=FI c=BI d=invertida
      op 2 TEST  a=rueda b=0 parar|1 adelante|2 atrás c=potencia %  (400 ms; se repite)
      op 3 RESET (valores de fábrica)   op 4 GET (pide las 4 ruedas)
      op 5 SWAP  a,b = ruedas que intercambian sus pines
  C3 -> Jetson: [0xC6][rueda][FI][BI][invertida][ok][versión][crc]
Ruedas: 0=R1, 1=R2, 2=L1, 3=L2. Pines permitidos GPIO 0..7. El firmware valida todo
(pines distintos entre las 8 salidas); aquí se valida también antes de enviar.
"""
from __future__ import annotations

import asyncio

from aiohttp import web

from ble_link import crc8

CFG_MAGIC = 0xC5
REPLY_MAGIC = 0xC6
OP_SET, OP_TEST, OP_RESET, OP_GET, OP_SWAP = 1, 2, 3, 4, 5
NOMBRES = ["R1", "R2", "L1", "L2"]
PINES_DEFECTO = [(0, 1, False), (6, 7, False), (3, 2, False), (4, 5, False)]
PRUEBA_MAX_PCT = 100


def pack(op: int, a: int = 0, b: int = 0, c: int = 0, d: int = 0, e: int = 0) -> bytes:
    cuerpo = bytes([CFG_MAGIC, op, a, b, c, d, e])
    return cuerpo + bytes([crc8(cuerpo)])


def unpack_reply(trama: bytes) -> dict | None:
    if len(trama) != 8 or trama[0] != REPLY_MAGIC or crc8(trama[:7]) != trama[7]:
        return None
    return {"rueda": trama[1], "fi": trama[2], "bi": trama[3],
            "invertida": bool(trama[4]), "ok": bool(trama[5]), "version": trama[6]}


def _entero(v, lo, hi, nombre):
    if isinstance(v, bool) or not isinstance(v, (int, float, str)):
        raise ValueError(f"{nombre}: valor inválido")
    try:
        n = int(v)
    except (TypeError, ValueError):
        raise ValueError(f"{nombre}: no es un entero")
    if not lo <= n <= hi:
        raise ValueError(f"{nombre} fuera de rango ({lo}..{hi})")
    return n


class C3Pines:
    def __init__(self, enlace_getter):
        self._enlace = enlace_getter           # callable -> EnlaceC3 | None
        self.ruedas: dict[int, dict] = {}      # última configuración vista en la C3
        self._evento = asyncio.Event()

    def on_cfg(self, trama: bytes):
        r = unpack_reply(trama)
        if r is None or r["rueda"] > 3:
            return
        self.ruedas[r["rueda"]] = r
        self._evento.set()

    def _wifi(self):
        enlace = self._enlace()
        wifi = getattr(enlace, "wifi", None)
        if wifi is None or not wifi.conectado:
            raise ConnectionError("C3 sin enlace")
        return wifi

    async def _esperar(self, cuantas: int, timeout=1.0):
        fin = asyncio.get_running_loop().time() + timeout
        while len(self.ruedas) < cuantas:
            resto = fin - asyncio.get_running_loop().time()
            if resto <= 0:
                break
            self._evento.clear()
            try:
                await asyncio.wait_for(self._evento.wait(), resto)
            except asyncio.TimeoutError:
                break

    def _estado(self):
        return [{"rueda": i, "nombre": NOMBRES[i], **self.ruedas[i]}
                for i in sorted(self.ruedas)]

    # ----- handlers HTTP -----
    async def get_handler(self, request):
        enlace = self._enlace()
        wifi = getattr(enlace, "wifi", None)
        conectada = bool(wifi is not None and wifi.conectado)
        if conectada:
            self.ruedas.clear()
            wifi.enviar(pack(OP_GET))
            await self._esperar(4)
        return web.json_response({"conectada": conectada, "ruedas": self._estado(),
                                  "nombres": NOMBRES, "defecto": [
                                      {"fi": f, "bi": b, "invertida": i} for f, b, i in PINES_DEFECTO]})

    async def set_handler(self, request):
        try:
            d = await request.json()
            w = _entero(d.get("rueda"), 0, 3, "rueda")
            fi = _entero(d.get("fi"), 0, 7, "fi")
            bi = _entero(d.get("bi"), 0, 7, "bi")
            inv = 1 if d.get("invertida") else 0
            if fi == bi:
                raise ValueError("FI y BI no pueden ser el mismo pin")
            otros = {p for i, r in self.ruedas.items() if i != w for p in (r["fi"], r["bi"])}
            if {fi, bi} & otros:
                raise ValueError("ese pin ya lo usa otra rueda: cámbialo primero")
            wifi = self._wifi()
        except ConnectionError as e:
            return web.json_response({"error": str(e)}, status=503)
        except (ValueError, TypeError) as e:
            return web.json_response({"error": str(e)}, status=400)
        self.ruedas.pop(w, None)
        wifi.enviar(pack(OP_SET, w, fi, bi, inv))
        await self._esperar_rueda(w)
        r = self.ruedas.get(w)
        if r is None:
            return web.json_response({"error": "la C3 no confirmó el cambio"}, status=504)
        if not (r["ok"] and (r["fi"], r["bi"], r["invertida"]) == (fi, bi, bool(inv))):
            return web.json_response({"error": "la C3 rechazó el cambio (pines repetidos)",
                                      "ruedas": self._estado()}, status=409)
        return web.json_response({"ok": True, "ruedas": self._estado()})

    async def _esperar_rueda(self, w, timeout=1.0):
        fin = asyncio.get_running_loop().time() + timeout
        while w not in self.ruedas:
            resto = fin - asyncio.get_running_loop().time()
            if resto <= 0:
                return
            self._evento.clear()
            try:
                await asyncio.wait_for(self._evento.wait(), resto)
            except asyncio.TimeoutError:
                return

    async def test_handler(self, request):
        try:
            d = await request.json()
            w = _entero(d.get("rueda"), 0, 3, "rueda")
            sentido = _entero(d.get("sentido", 0), -1, 1, "sentido")
            pct = _entero(d.get("pct", 0), 0, PRUEBA_MAX_PCT, "pct")
            wifi = self._wifi()
        except ConnectionError as e:
            return web.json_response({"error": str(e)}, status=503)
        except (ValueError, TypeError) as e:
            return web.json_response({"error": str(e)}, status=400)
        direccion = 0 if sentido == 0 or pct == 0 else (1 if sentido > 0 else 2)
        wifi.enviar(pack(OP_TEST, w, direccion, pct))
        return web.json_response({"ok": True})

    async def reset_handler(self, request):
        try:
            wifi = self._wifi()
        except ConnectionError as e:
            return web.json_response({"error": str(e)}, status=503)
        self.ruedas.clear()
        wifi.enviar(pack(OP_RESET))
        await self._esperar(4)
        return web.json_response({"ok": len(self.ruedas) == 4, "ruedas": self._estado()})

    async def swap_handler(self, request):
        try:
            d = await request.json()
            a = _entero(d.get("a"), 0, 3, "a")
            b = _entero(d.get("b"), 0, 3, "b")
            if a == b:
                raise ValueError("elige dos ruedas distintas")
            wifi = self._wifi()
        except ConnectionError as e:
            return web.json_response({"error": str(e)}, status=503)
        except (ValueError, TypeError) as e:
            return web.json_response({"error": str(e)}, status=400)
        self.ruedas.pop(a, None)
        self.ruedas.pop(b, None)
        wifi.enviar(pack(OP_SWAP, a, b))
        await self._esperar_rueda(a)
        await self._esperar_rueda(b)
        return web.json_response({"ok": a in self.ruedas and b in self.ruedas, "ruedas": self._estado()})
