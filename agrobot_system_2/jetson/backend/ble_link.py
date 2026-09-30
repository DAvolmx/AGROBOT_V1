"""
Enlace BLE entre la Jetson y el ESP32-C3 actuador.

Reemplaza uart_link.py tras el accidente del 2026-09-20: el GND compartido
del cable UART creó una ruta para back-EMF que destruyó la C3. Con BLE no
hay ningún cable eléctrico entre los dos dominios de potencia.

ARQUITECTURA DE BATERÍAS:
  · Batería de motores → motores + puente H + ESP32-C3.
  · Batería de lógica  → solo la Jetson Orin Nano.
  · Enlace entre ambos  → BLE (inalámbrico, aislamiento galvánico total).

PROTOCOLO: idéntico al UART (8 bytes con magic y CRC-8), transportado
como escritura/notificación de una característica GATT en vez de un chorro
de bytes por serie. Debe coincidir byte a byte con el firmware
esp32c3_actuator.ino.

Interfaz pública idéntica a UartActuatorLink: send_command(),
set_actuators(), emergency_stop(), start(), stop(), connected.
"""

from __future__ import annotations

import asyncio
import struct
import time

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

from protocol import SERVO_HOME_DEG, clamp_servo

# ===== UUIDs (deben coincidir con el firmware) =====
BLE_SERVICE_UUID     = "a0e1b000-7f3c-4a5e-9d21-6b8e2c4f1a00"
BLE_CMD_CHAR_UUID    = "a0e1b001-7f3c-4a5e-9d21-6b8e2c4f1a00"
BLE_STATUS_CHAR_UUID = "a0e1b002-7f3c-4a5e-9d21-6b8e2c4f1a00"

# ===== Protocolo (idéntico al UART) =====
CMD_MAGIC = 0xA5
STATUS_MAGIC = 0x5A
CMD_FORMAT = "<BbbBBHB"      # magic, v_pct, omega_pct, servo, flags, seq, crc
STATUS_FORMAT = "<BbbBBHB"
CMD_SIZE = struct.calcsize(CMD_FORMAT)
STATUS_SIZE = struct.calcsize(STATUS_FORMAT)

FLAG_LED = 0x01
FLAG_SCAN = 0x02

ST_LED = 0x01
ST_SCANNING = 0x02
ST_FAILSAFE = 0x04
ST_BROWNOUT = 0x08     # la C3 arrancó tras una caída de tensión (firmware 2026-09-30)
ST_RECIEN = 0x10       # la C3 arrancó hace menos de 15 s
# Firmware BLE+WiFi (2026-09-30): qué vías ve vivas la propia C3.
ST_VIA_WIFI = 0x20     # la última orden aplicada llegó por WiFi
ST_WIFI_OK = 0x40      # asociada al AP
ST_BLE_OK = 0x80       # con cliente BLE conectado

SEND_HZ = 20
SEND_PERIOD = 1.0 / SEND_HZ

DEVICE_NAME = "AGROBOT-C3"
SCAN_TIMEOUT_S = 10.0
CONNECT_TIMEOUT_S = 12.0
RECONNECT_DELAY_S = 3.0
# La C3 manda estado a 10 Hz. BlueZ puede dar la conexión por viva mucho
# después de que la C3 se reinicie o se quede sin batería; sin esta
# comprobación el backend seguía escribiendo al vacío con `connected=True`.
ESTADO_ZOMBI_S = 3.0
# Tras tantos fallos seguidos conectando por dirección, se vuelve a
# escanear: la C3 pudo cambiar (placa nueva tras el accidente del 20-09).
FALLOS_ANTES_DE_ESCANEAR = 3
# Si la conexión muere a los pocos segundos una y otra vez (la C3 se
# reinicia por caída de tensión), se espera cada vez más antes de volver a
# intentarlo. El 2026-09-28 ese bucle de 130 reconexiones tumbó dos veces
# a bluetoothd (segfault) y dejó la C3 con el foco encendido en failsafe.
CONEXION_CORTA_S = 5.0
ESPERA_MAX_S = 30.0
# Mientras la C3 no aparece solo se escribe un resumen por minuto. Antes
# cada búsqueda dejaba dos líneas: 10 500 en un día con la C3 apagada.
RESUMEN_BUSQUEDA_S = 60.0
# Escaneos seguidos sin oír NINGÚN dispositivo BLE (ni la C3 ni otro) antes
# de apagar y encender el adaptador. El Intel 8265 comparte radio con el
# Wi-Fi en modo AP y puede quedarse sordo; con la C3 simplemente apagada se
# suele oír algún otro aparato.
ESCANEOS_SORDOS_ANTES_DE_REINICIAR = 6
REINICIO_RADIO_ENFRIAMIENTO_S = 600.0


def crc8(datos: bytes) -> int:
    crc = 0
    for byte in datos:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _pct(value: float, maximo: float) -> int:
    if maximo <= 0:
        return 0
    return max(-100, min(100, round(value / maximo * 100.0)))


class BleActuatorLink:
    """Enlace BLE con la misma interfaz que UartActuatorLink.

    Quien llama no necesita saber si el BLE está conectado. Si no lo está,
    el comando queda guardado y se aplicará al reconectar; el robot mientras
    tanto está parado por su propio watchdog.
    """

    def __init__(self, v_max: float, omega_max: float,
                 device_name: str = DEVICE_NAME,
                 on_status=None):
        self.v_max = v_max
        self.omega_max = omega_max
        self.device_name = device_name
        self._on_status = on_status

        self._client: BleakClient | None = None
        self._connected = False
        self._seq = 0
        self._task: asyncio.Task | None = None
        self._stop = asyncio.Event()
        self.crc_errors = 0
        self._reconnects = 0
        self.zombis = 0
        self.escrituras_fallidas = 0
        self.direccion: str | None = None
        self._fallos_directos = 0
        self._ultimo_estado = 0.0
        self._conectado_en = 0.0
        self._cortas_seguidas = 0
        self.reinicios_c3 = 0
        self._c3_recien = False
        # búsqueda por escaneo
        self.busquedas_fallidas = 0
        self._buscando_desde = 0.0
        self._ultimo_resumen = 0.0
        self.dispositivos_oidos = None
        self._escaneos_sordos = 0
        self.reinicios_radio = 0
        self._ultimo_reinicio_radio = 0.0

        self._v = 0.0
        self._omega = 0.0
        self._servo = SERVO_HOME_DEG
        self._led = False
        self._scan_once = False

    @property
    def connected(self) -> bool:
        return self._connected

    def next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        return self._seq

    # ----- API que consume jetson_server -----
    def send_command(self, v: float, omega: float, scan_cmd: int = 0):
        self._v = v
        self._omega = omega
        if scan_cmd:
            self._scan_once = True

    def set_actuators(self, servo_angle: int | None = None, led: bool | None = None):
        if servo_angle is not None:
            self._servo = clamp_servo(servo_angle)
        if led is not None:
            self._led = bool(led)

    def emergency_stop(self):
        self._v = 0.0
        self._omega = 0.0
        self._led = False
        self._scan_once = False

    def pack(self) -> bytes:
        flags = 0
        # Bit invertido a propósito: ver el comentario de uart_link.pack().
        # Este foco es de lógica normal y el firmware no puede cambiar su
        # LED_ACTIVO_EN_BAJO (GPIO8 es pin de strapping).
        if not self._led:
            flags |= FLAG_LED
        if self._scan_once:
            flags |= FLAG_SCAN
        cuerpo = struct.pack(
            CMD_FORMAT[:-1], CMD_MAGIC,
            _pct(self._v, self.v_max), _pct(self._omega, self.omega_max),
            self._servo, flags, self.next_seq(),
        )
        return cuerpo + bytes([crc8(cuerpo)])

    def resumen(self) -> dict:
        edad = time.monotonic() - self._ultimo_estado if self._ultimo_estado else None
        buscando = (time.monotonic() - self._buscando_desde
                    if self._buscando_desde and not self._connected else None)
        return {"conectado": self._connected, "direccion": self.direccion,
                "reconexiones": self._reconnects, "zombis": self.zombis,
                "errores_crc": self.crc_errors,
                "escrituras_fallidas": self.escrituras_fallidas,
                "cortas_seguidas": self._cortas_seguidas,
                "edad_estado_s": None if edad is None else round(edad, 2),
                "reinicios_c3": self.reinicios_c3,
                "buscando_s": None if buscando is None else round(buscando),
                "busquedas_fallidas": self.busquedas_fallidas,
                "dispositivos_oidos": self.dispositivos_oidos,
                "reinicios_radio": self.reinicios_radio}

    # ----- ciclo de vida -----
    def start(self):
        self._stop.clear()
        self._task = asyncio.create_task(self._run())

    async def stop(self):
        self._stop.set()
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):
                pass
        await self._disconnect()

    async def _disconnect(self):
        client = self._client
        self._client = None
        self._connected = False
        if client and client.is_connected:
            try:
                await asyncio.wait_for(client.disconnect(), timeout=5.0)
            except Exception:
                pass

    def _on_notify(self, _sender, data: bytearray):
        if len(data) != STATUS_SIZE:
            self.crc_errors += 1
            return
        if data[0] != STATUS_MAGIC:
            self.crc_errors += 1
            return
        if crc8(bytes(data[:-1])) != data[-1]:
            self.crc_errors += 1
            return
        self._ultimo_estado = time.monotonic()
        self._handle_status(bytes(data))

    def _handle_status(self, trama: bytes):
        (_magic, v_pct, omega_pct, servo, flags, seq_echo,
         _crc) = struct.unpack(STATUS_FORMAT, trama)
        status = {
            "v_cmd": v_pct / 100.0 * self.v_max,
            "omega_cmd": omega_pct / 100.0 * self.omega_max,
            "scanning": bool(flags & ST_SCANNING),
            "failsafe": bool(flags & ST_FAILSAFE),
            "servo_angle": servo,
            # Mismo desfase que en pack(): lo que el firmware llama
            # "encendido" es el foco apagado. Sin invertir, el dashboard
            # decía ON con el foco apagado.
            "led_state": not bool(flags & ST_LED),
            "seq_echo": seq_echo,
            "reinicio_por_tension": bool(flags & ST_BROWNOUT),
            "recien_arrancada": bool(flags & ST_RECIEN),
            # Solo significan algo con el firmware BLE+WiFi; con el anterior
            # salen siempre en falso.
            "orden_por_wifi": bool(flags & ST_VIA_WIFI),
            "c3_wifi_ok": bool(flags & ST_WIFI_OK),
            "c3_ble_ok": bool(flags & ST_BLE_OK),
            "reinicios_vistos": self.reinicios_c3,
            "_received_at": time.time(),
        }
        # Un reinicio de la C3 se ve como "recién arrancada" tras haberla
        # visto funcionando: se cuenta y se dice por qué (si el firmware lo
        # sabe). Es lo que el 2026-09-28 hubo que deducir de los logs de BlueZ.
        if status["recien_arrancada"] and not self._c3_recien:
            self.reinicios_c3 += 1
            status["reinicios_vistos"] = self.reinicios_c3
            motivo = ("CAÍDA DE TENSIÓN: revisá la batería de motores"
                      if status["reinicio_por_tension"] else "arranque normal")
            print(f"[ble] la C3 acaba de arrancar ({motivo})", flush=True)
        self._c3_recien = status["recien_arrancada"]
        if self._on_status is not None:
            self._on_status(status)

    async def _escanear(self):
        """Escanea hasta ver la C3 o agotar SCAN_TIMEOUT_S.

        Cuenta además cuántos dispositivos BLE distintos se oyen: cero
        durante muchos escaneos seguidos es un radio sordo, no una C3
        apagada, y eso se arregla distinto (ver `_reiniciar_radio`)."""
        encontrado = asyncio.get_running_loop().create_future()
        vistos = set()

        def al_ver(dev, adv):
            vistos.add(dev.address)
            nombre = adv.local_name or dev.name
            if nombre == self.device_name and not encontrado.done():
                encontrado.set_result(dev)

        async with BleakScanner(detection_callback=al_ver):
            try:
                dev = await asyncio.wait_for(encontrado, SCAN_TIMEOUT_S)
            except asyncio.TimeoutError:
                dev = None
        return dev, len(vistos)

    async def _reiniciar_radio(self):
        """Apaga y enciende el adaptador con bluetoothctl (no pide root)."""
        self._ultimo_reinicio_radio = time.monotonic()
        self.reinicios_radio += 1
        print(f"[ble] {self._escaneos_sordos} escaneos sin oír ningún dispositivo BLE: "
              f"reinicio el adaptador (vez #{self.reinicios_radio})", flush=True)
        for orden in ("off", "on"):
            try:
                proc = await asyncio.create_subprocess_exec(
                    "bluetoothctl", "power", orden,
                    stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL)
                await asyncio.wait_for(proc.wait(), timeout=10.0)
            except (OSError, asyncio.TimeoutError) as exc:
                print(f"[ble] no pude reiniciar el adaptador: {exc}", flush=True)
                return
            await asyncio.sleep(2.0)
        self._escaneos_sordos = 0

    async def _find_device(self):
        """La dirección ya conocida primero (reconectar en <1 s); si falla
        varias veces seguidas, escanear por nombre."""
        if self.direccion and self._fallos_directos < FALLOS_ANTES_DE_ESCANEAR:
            return self.direccion
        ahora = time.monotonic()
        if not self._buscando_desde:
            self._buscando_desde = ahora
            self._ultimo_resumen = ahora
            print(f"[ble] buscando '{self.device_name}'...", flush=True)
        device, oidos = await self._escanear()
        self.dispositivos_oidos = oidos
        if device:
            minutos = (time.monotonic() - self._buscando_desde) / 60
            print(f"[ble] encontrado: {device.name} ({device.address}) tras "
                  f"{self.busquedas_fallidas + 1} búsquedas ({minutos:.1f} min)", flush=True)
            self.direccion = device.address
            self._fallos_directos = 0
            self.busquedas_fallidas = 0
            self._buscando_desde = 0.0
            self._escaneos_sordos = 0
            return device

        self.busquedas_fallidas += 1
        self._escaneos_sordos = self._escaneos_sordos + 1 if oidos == 0 else 0
        ahora = time.monotonic()
        if ahora - self._ultimo_resumen >= RESUMEN_BUSQUEDA_S:
            self._ultimo_resumen = ahora
            print(f"[ble] '{self.device_name}' sigue sin aparecer: "
                  f"{self.busquedas_fallidas} búsquedas en "
                  f"{(ahora - self._buscando_desde) / 60:.0f} min, "
                  f"{oidos} dispositivos BLE oídos en la última", flush=True)
        if (self._escaneos_sordos >= ESCANEOS_SORDOS_ANTES_DE_REINICIAR
                and ahora - self._ultimo_reinicio_radio > REINICIO_RADIO_ENFRIAMIENTO_S):
            await self._reiniciar_radio()
        return None

    def _espera_reconexion(self) -> float:
        """Más espera cuanto más veces seguidas se cayó enseguida."""
        return min(ESPERA_MAX_S, RECONNECT_DELAY_S * (2 ** min(self._cortas_seguidas, 4)))

    async def _run(self):
        while not self._stop.is_set():
            try:
                device = await self._find_device()
                if device is None:
                    await asyncio.sleep(RECONNECT_DELAY_S)
                    continue

                self._client = BleakClient(
                    device,
                    disconnected_callback=self._on_disconnect,
                    timeout=CONNECT_TIMEOUT_S,
                )
                try:
                    await self._client.connect()
                except (BleakError, asyncio.TimeoutError, OSError):
                    self._fallos_directos += 1
                    raise
                self._fallos_directos = 0
                self._buscando_desde = 0.0
                self._connected = True
                self._reconnects += 1
                self._conectado_en = time.monotonic()
                self._ultimo_estado = 0.0
                print(f"[ble] conectado a {self.direccion or device} "
                      f"(reconexión #{self._reconnects})", flush=True)

                await self._client.start_notify(
                    BLE_STATUS_CHAR_UUID, self._on_notify)

                await self._send_loop()

            except (BleakError, asyncio.TimeoutError, OSError) as exc:
                if not self._stop.is_set():
                    print(f"[ble] error: {type(exc).__name__}: {exc}", flush=True)
            except asyncio.CancelledError:
                break
            except Exception as exc:
                # Nunca dejar morir esta tarea: sin ella el robot queda sin
                # mando hasta reiniciar el backend.
                print(f"[ble] fallo inesperado: {type(exc).__name__}: {exc}", flush=True)
            finally:
                duro = time.monotonic() - self._conectado_en if self._conectado_en else 0.0
                if self._conectado_en and duro < CONEXION_CORTA_S:
                    self._cortas_seguidas += 1
                elif self._conectado_en:
                    self._cortas_seguidas = 0
                self._conectado_en = 0.0
                await self._disconnect()
                if not self._stop.is_set():
                    espera = self._espera_reconexion()
                    if self._cortas_seguidas >= 2:
                        print(f"[ble] la C3 se cae al conectar ({self._cortas_seguidas} veces "
                              f"seguidas): ¿batería baja? Espero {espera:.0f} s", flush=True)
                    await asyncio.sleep(espera)

    def _on_disconnect(self, _client):
        # BlueZ avisa una vez por cada intento de conexión que no cuaja (con
        # la señal débil, 7 avisos seguidos por intento): solo cuenta la
        # caída de una conexión que estaba viva.
        estaba = self._connected
        self._connected = False
        if estaba and not self._stop.is_set():
            vivo = time.monotonic() - self._conectado_en if self._conectado_en else 0.0
            print(f"[ble] desconectado tras {vivo:.0f} s — reconectando...", flush=True)

    def _es_zombi(self) -> bool:
        ahora = time.monotonic()
        referencia = self._ultimo_estado or self._conectado_en
        return ahora - referencia > ESTADO_ZOMBI_S

    async def _send_loop(self):
        while self._connected and not self._stop.is_set():
            inicio = time.monotonic()
            if self._es_zombi():
                self.zombis += 1
                print(f"[ble] sin estado de la C3 en {ESTADO_ZOMBI_S:.0f} s: "
                      f"conexión zombi, se fuerza la reconexión", flush=True)
                return
            try:
                data = self.pack()
                await self._client.write_gatt_char(
                    BLE_CMD_CHAR_UUID, data, response=False)
                self._scan_once = False
            except (BleakError, OSError) as exc:
                self.escrituras_fallidas += 1
                print(f"[ble] escritura fallida: {type(exc).__name__}: {exc}", flush=True)
                self._connected = False
                return
            resto = SEND_PERIOD - (time.monotonic() - inicio)
            if resto > 0:
                await asyncio.sleep(resto)
