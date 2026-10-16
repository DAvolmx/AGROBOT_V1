"""
Vía WiFi UDP con el ESP32-C3 actuador: el RESPALDO del BLE (2026-09-30).

La C3 va asociada al AP de la Jetson (JETSON_AP) y manda su trama de estado
de 8 bytes a 10 Hz al puerto 5002 de la puerta de enlace, que es la Jetson.
De ahí se aprende su IP: no hay IP fija que mantener, y si el DHCP le da
otra, la siguiente trama la corrige. Las órdenes vuelven a esa IP, al mismo
puerto, con exactamente la misma trama que viaja por BLE (ver ble_link.py).

Solo se aceptan tramas de la red del AP: la Jetson también está en la red
del router como cliente y nadie de ahí tiene que poder hacerse pasar por la
C3 para recibir las órdenes del robot.

Esta clase no decide nada de movimiento: la orquesta `enlace_c3.EnlaceC3`,
que manda cada orden por las dos vías.
"""
from __future__ import annotations

import asyncio
import ipaddress
import time

from ble_link import STATUS_MAGIC, STATUS_SIZE, crc8

PUERTO = 5002
CFG_REPLY_MAGIC = 0xC6     # respuesta de configuración de la C3 (ver firmware skid-steer)
RED_AP = ipaddress.ip_network("192.168.12.0/24")
IP_C3_FIJA = ipaddress.ip_address("192.168.0.50")
# La C3 manda estado a 10 Hz. En modo de ahorro del WiFi (obligatorio con
# BLE activo) llega en ráfagas cada ~200 ms: 1,5 s sin nada = vía caída.
VIVA_S = 1.5


class UdpActuatorLink(asyncio.DatagramProtocol):
    def __init__(self, on_trama, on_cfg=None):
        self._on_trama = on_trama        # callback(bytes) con cada estado válido
        self.on_cfg = on_cfg             # callback(bytes) con cada respuesta de configuración (0xC6)
        self.transport: asyncio.DatagramTransport | None = None
        self.ip_c3: str | None = None
        self._ultimo_estado = 0.0
        self.recibidas = 0
        self.enviadas = 0
        self.errores = 0
        self.ajenas = 0

    # ----- asyncio -----
    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        try:
            ip = ipaddress.ip_address(addr[0])
        except ValueError:
            return
        if ip not in RED_AP and ip != IP_C3_FIJA:
            self.ajenas += 1
            return
        if len(data) == STATUS_SIZE and data[0] == CFG_REPLY_MAGIC and crc8(bytes(data[:-1])) == data[-1]:
            if self.ip_c3 is None:
                self.ip_c3 = addr[0]
            if self.on_cfg is not None:
                self.on_cfg(bytes(data))
            return
        if (len(data) != STATUS_SIZE or data[0] != STATUS_MAGIC
                or crc8(bytes(data[:-1])) != data[-1]):
            self.errores += 1
            return
        if self.ip_c3 != addr[0]:
            print(f"[c3-wifi] la C3 responde por WiFi desde {addr[0]}", flush=True)
            self.ip_c3 = addr[0]
        self._ultimo_estado = time.monotonic()
        self.recibidas += 1
        self._on_trama(bytes(data))

    def error_received(self, exc):
        self.errores += 1

    # ----- API -----
    @property
    def conectado(self) -> bool:
        return (self.ip_c3 is not None
                and time.monotonic() - self._ultimo_estado < VIVA_S)

    def enviar(self, trama: bytes) -> bool:
        """Manda una orden a la última IP conocida de la C3. Se sigue
        mandando aunque su estado no llegue: la ida puede funcionar con la
        vuelta caída, y no cuesta nada."""
        if self.transport is None or self.ip_c3 is None:
            return False
        try:
            self.transport.sendto(trama, (self.ip_c3, PUERTO))
        except OSError:
            self.errores += 1
            return False
        self.enviadas += 1
        return True

    def resumen(self) -> dict:
        edad = time.monotonic() - self._ultimo_estado if self._ultimo_estado else None
        return {"conectado": self.conectado, "ip": self.ip_c3,
                "recibidas": self.recibidas, "enviadas": self.enviadas,
                "errores": self.errores, "ajenas": self.ajenas,
                "edad_estado_s": None if edad is None else round(edad, 2)}


async def abrir(on_trama) -> UdpActuatorLink | None:
    """Escucha en 0.0.0.0:5002. Sin la IP del AP levantada también arranca:
    la C3 aparecerá cuando el AP vuelva. None si el puerto está ocupado."""
    loop = asyncio.get_running_loop()
    try:
        _, proto = await loop.create_datagram_endpoint(
            lambda: UdpActuatorLink(on_trama), local_addr=("0.0.0.0", PUERTO))
    except OSError as exc:
        print(f"[c3-wifi] no se pudo abrir el puerto UDP {PUERTO}: {exc} — "
              f"solo queda el BLE", flush=True)
        return None
    print(f"[c3-wifi] esperando a la C3 por WiFi en el puerto UDP {PUERTO}", flush=True)
    return proto
