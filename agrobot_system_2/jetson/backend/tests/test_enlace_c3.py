"""
Prueba del enlace redundante con la C3 (enlace_c3.py + udp_actuator_link.py)
contra una C3 SIMULADA por UDP en localhost. No toca el BLE ni el robot.

La C3 simulada copia la regla del firmware (aplicarComando en
esp32c3_actuator.ino): solo aplica la trama más nueva por número de
secuencia, salvo tras un failsafe.

    cd jetson/backend && python3 -m pytest tests/test_enlace_c3.py -q
"""
import asyncio
import ipaddress
import os
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import ble_link  # noqa: E402
import udp_actuator_link  # noqa: E402
from ble_link import (CMD_FORMAT, CMD_MAGIC, FLAG_SCAN, STATUS_FORMAT,  # noqa: E402
                      STATUS_MAGIC, BleActuatorLink, crc8)
from enlace_c3 import EnlaceC3  # noqa: E402

CMD_TIMEOUT_S = 0.5


class C3Simulada(asyncio.DatagramProtocol):
    """Lo mínimo del firmware: aplica órdenes y manda estado a 10 Hz."""

    def __init__(self):
        self.transport = None
        self.aplicadas = []        # (seq, v_pct, flags)
        self.viejas = 0
        self.last_seq = 0
        self.last_cmd = 0.0
        self.failsafe = True

    def connection_made(self, t):
        self.transport = t

    def datagram_received(self, data, addr):
        magic, v, w, servo, flags, seq, crc = struct.unpack(CMD_FORMAT, data)
        assert magic == CMD_MAGIC and crc8(data[:-1]) == crc
        self.aplicar(seq, v, flags)

    def aplicar(self, seq, v, flags):
        d = (seq - self.last_seq + 0x8000) % 0x10000 - 0x8000   # int16 como en C
        if not self.failsafe and d <= 0 and d > -1000:
            self.viejas += 1
            return False
        self.last_seq, self.last_cmd, self.failsafe = seq, time.monotonic(), False
        self.aplicadas.append((seq, v, flags))
        return True

    def estado(self):
        cuerpo = struct.pack(STATUS_FORMAT[:-1], STATUS_MAGIC, 0, 0, 98, 0x40, self.last_seq)
        return cuerpo + bytes([crc8(cuerpo)])


async def _montar():
    udp_actuator_link.RED_AP = ipaddress.ip_network("127.0.0.0/8")
    loop = asyncio.get_running_loop()
    # Puerto efímero: cada prueba abre el suyo.
    _, c3 = await loop.create_datagram_endpoint(C3Simulada, local_addr=("127.0.0.1", 0))
    udp_actuator_link.PUERTO = c3.transport.get_extra_info("sockname")[1]
    _, wifi = await loop.create_datagram_endpoint(
        lambda: udp_actuator_link.UdpActuatorLink(lambda t: None), local_addr=("127.0.0.2", 0))
    ble = BleActuatorLink(v_max=0.55, omega_max=1.5)       # sin start(): no toca el BLE
    enlace = EnlaceC3(ble, wifi)
    estados = []
    enlace._on_status = estados.append
    # La C3 se presenta mandando su estado (así la Jetson aprende su IP).
    c3.transport.sendto(c3.estado(), wifi.transport.get_extra_info("sockname"))
    await asyncio.sleep(0.05)
    enlace._tarea = asyncio.create_task(enlace._bucle_wifi())
    return c3, wifi, ble, enlace, estados


def test_ordenes_por_wifi_y_estado_de_vuelta():
    async def prueba():
        c3, wifi, ble, enlace, estados = await _montar()
        assert wifi.ip_c3 == "127.0.0.1"
        assert estados and estados[0]["via_estado"] == "wifi"
        assert estados[0]["c3_wifi_ok"] is True
        enlace.send_command(0.275, 0.0)            # 50 %
        await asyncio.sleep(0.3)
        assert len(c3.aplicadas) >= 4              # ~20 Hz
        assert c3.aplicadas[-1][1] == 50
        assert enlace.connected                    # vía WiFi viva, BLE caído
        enlace._tarea.cancel()
    asyncio.run(prueba())


def test_parada_de_emergencia_sale_al_instante():
    async def prueba():
        c3, wifi, ble, enlace, _ = await _montar()
        enlace.send_command(0.55, 0.0)
        await asyncio.sleep(0.12)
        enlace._tarea.cancel()                     # sin bucle: solo la parada directa
        n = len(c3.aplicadas)
        enlace.emergency_stop()
        await asyncio.sleep(0.02)
        assert len(c3.aplicadas) == n + 1
        assert c3.aplicadas[-1][1] == 0
    asyncio.run(prueba())


def test_barrido_de_camara_sale_una_sola_vez():
    async def prueba():
        c3, wifi, ble, enlace, _ = await _montar()
        enlace.send_command(0.0, 0.0, scan_cmd=1)
        await asyncio.sleep(0.3)
        con_scan = [a for a in c3.aplicadas if a[2] & FLAG_SCAN]
        assert len(con_scan) == 1
        assert ble._scan_once is False
        enlace._tarea.cancel()
    asyncio.run(prueba())


def test_trama_atrasada_no_deshace_una_orden_nueva():
    """La regla del firmware: 'avanza' (seq 10) que llega tarde por WiFi
    detrás de 'para' (seq 11) por BLE NO se aplica."""
    c3 = C3Simulada()
    assert c3.aplicar(11, 0, 0)                    # "para" por BLE
    assert not c3.aplicar(10, 50, 0)               # "avanza" atrasado por WiFi
    assert c3.aplicadas[-1][1] == 0 and c3.viejas == 1
    # Vuelta del contador de 16 bits: 65535 -> 0 es una trama NUEVA.
    c3.last_seq = 65535
    assert c3.aplicar(0, 20, 0)
    # Tras un failsafe (backend reiniciado, seq desde cero) se acepta.
    c3.failsafe = True
    assert c3.aplicar(1, 30, 0)


def test_estado_del_wifi_no_duplica_al_del_ble():
    ble = BleActuatorLink(v_max=0.55, omega_max=1.5)
    enlace = EnlaceC3(ble, None)
    recibidos = []
    enlace._on_status = recibidos.append
    c3 = C3Simulada()
    trama = c3.estado()
    ble._handle_status(trama)                      # llega por BLE
    enlace._trama_wifi(trama)                      # y la misma por WiFi enseguida
    assert len(recibidos) == 1
    time.sleep(0.35)                               # el BLE calla
    enlace._trama_wifi(trama)
    assert len(recibidos) == 2 and recibidos[-1]["via_estado"] == "wifi"


def test_ignora_tramas_de_fuera_del_ap():
    udp_actuator_link.RED_AP = ipaddress.ip_network("192.168.12.0/24")
    link = udp_actuator_link.UdpActuatorLink(lambda t: None)
    c3 = C3Simulada()
    # 192.168.0.50 es la IP FIJA reservada a la C3 en el router (IP_C3_FIJA) y
    # SI se acepta; cualquier otra IP de fuera del AP se ignora. (Antes este test
    # usaba 192.168.0.50 como "ajena", pero udp_actuator_link la admite a proposito.)
    link.datagram_received(c3.estado(), ("192.168.0.77", 5002))
    assert link.ip_c3 is None and link.ajenas == 1
    link.datagram_received(c3.estado(), ("192.168.12.37", 5002))
    assert link.ip_c3 == "192.168.12.37"
    malo = bytearray(c3.estado())
    malo[2] ^= 0xFF
    link.datagram_received(bytes(malo), ("192.168.12.37", 5002))
    assert link.errores == 1


def test_cada_trama_wifi_sale_duplicada_con_la_misma_secuencia():
    """2026-10-21: DUPLICADOS_WIFI copias de la MISMA trama (misma secuencia), para sobrevivir a una pérdida suelta."""
    import enlace_c3

    class Falso:
        ip_c3 = "192.168.12.171"
        conectado = True

        def __init__(self):
            self.salidas = []

        def enviar(self, trama):
            self.salidas.append(bytes(trama))
            return True

    async def prueba():
        ble = BleActuatorLink(0.55, 1.5)
        wifi = Falso()
        e = EnlaceC3(ble, wifi)
        e._enviar_wifi()
        await asyncio.sleep(enlace_c3.SEPARACION_COPIAS_S * enlace_c3.DUPLICADOS_WIFI + 0.05)
        return wifi.salidas

    salidas = asyncio.run(prueba())
    assert len(salidas) == enlace_c3.DUPLICADOS_WIFI
    assert len(set(salidas)) == 1


def test_el_cable_usb_recibe_la_misma_trama_y_cuenta_como_conectado():
    """2026-10-21: el cable USB es la tercera vía: misma trama (misma secuencia) que el WiFi, y `connected` si está vivo."""
    class UartFalso:
        connected = True
        vivo = True
        recibidas = 0
        _on_status = None

        def __init__(self):
            self.salidas = []

        def enviar(self, trama):
            self.salidas.append(bytes(trama))
            return True

    class WifiFalso:
        ip_c3 = "192.168.12.171"
        conectado = False

        def __init__(self):
            self.salidas = []

        def enviar(self, trama):
            self.salidas.append(bytes(trama))
            return True

    ble = BleActuatorLink(0.55, 1.5)
    uart, wifi = UartFalso(), WifiFalso()
    e = EnlaceC3(ble, wifi, wifi_only=True, uart=uart)
    assert e.connected                                  # solo el cable está vivo
    e._enviar_wifi()
    assert len(uart.salidas) == 1 and uart.salidas[0] == wifi.salidas[0]
