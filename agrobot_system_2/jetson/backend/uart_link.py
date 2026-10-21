"""
Enlace UART por cable entre la Jetson y el ESP32-C3 actuador.

Es el tercer transporte de esta pareja, y el ultimo. Los dos anteriores se
retiraron el 2026-09-04 y conviene saber por que para no reintroducirlos:

  - WiFi + UDP: el ESP32-C3 tiene UNA sola radio de 2.4 GHz. La asociacion
    al AP de la Jetson se caia sola y no se recuperaba.
  - BLE: quitar el WiFi le dejaba la radio entera, pero la placa dejo de
    anunciarse y el enlace nunca llego a establecerse.

Las dos placas van atornilladas al mismo robot, a veinte centimetros una de
otra. Un enlace por radio entre ellas nunca compro nada y aportaba todos los
modos de fallo. Un cable de tres hilos no se desasocia, no comparte antena y
no negocia nada.

CABLEADO
    Jetson pin 8  (TXD, /dev/ttyTHS1)  ->  C3 GPIO20 (RX)
    Jetson pin 10 (RXD, /dev/ttyTHS1)  <-  C3 GPIO21 (TX)
    Jetson pin 6  (GND)                 -  C3 GND

TRAMADO
Un UART entrega un chorro de bytes, no paquetes: no hay nada que diga donde
empieza una trama. De ahi el byte de inicio (para resincronizar) y el CRC-8
(para que un magic aparecido por casualidad dentro de los datos no se tome
por el principio de una trama). El bootloader del C3 escupe su mensaje de
arranque por estos mismos pines en cada reset, asi que basura en la linea no
es una hipotesis remota: es lo normal al encender.

Debe coincidir byte a byte con
firmware/esp32c3_actuator/esp32c3_actuator.ino.
"""

from __future__ import annotations

import asyncio
import struct
import time

import serial_asyncio

from protocol import SERVO_HOME_DEG, clamp_servo

# ===== Puerto =====
# ttyTHS1 es el UART del cabezal de 40 pines de la Orin. El getty de consola
# viene desactivado en esta maquina (la consola del kernel va por ttyTCU0);
# si alguna vez se reactiva, se pelea por el puerto y hay que apagarlo con
#   sudo systemctl disable --now serial-getty@ttyTHS1
DEFAULT_PORT = "/dev/ttyTHS1"
DEFAULT_BAUD = 115200

# ===== Protocolo binario (little endian, empaquetado) =====
CMD_MAGIC = 0xA5
STATUS_MAGIC = 0x5A
CMD_FORMAT = "<BbbBBHB"      # magic, v_pct, omega_pct, servo, flags, seq, crc
STATUS_FORMAT = "<BbbBBHB"   # magic, v_pct, omega_pct, servo, flags, seq, crc
CMD_SIZE = struct.calcsize(CMD_FORMAT)
STATUS_SIZE = struct.calcsize(STATUS_FORMAT)

FLAG_LED = 0x01
FLAG_SCAN = 0x02

ST_LED = 0x01
ST_SCANNING = 0x02
ST_FAILSAFE = 0x04

# El firmware entra en failsafe si pasan 300 ms sin una trama. A 20 Hz hacen
# falta 6 tramas perdidas seguidas para que salte.
#
# Este envio periodico NO es un detalle del transporte: es la correccion de
# un fallo real que traia el camino UDP, donde solo se mandaba un paquete
# cuando llegaba un mensaje del navegador. Cualquier hueco del WebSocket
# disparaba el failsafe y el robot avanzaba a tirones. El enlace tiene que
# latir por su cuenta, no al ritmo de la interfaz.
SEND_HZ = 20
SEND_PERIOD = 1.0 / SEND_HZ

RECONNECT_DELAY_S = 2.0


def crc8(datos: bytes) -> int:
    """CRC-8 con polinomio 0x07, el mismo bucle que el firmware."""
    crc = 0
    for byte in datos:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _pct(value: float, maximo: float) -> int:
    """Convierte una magnitud fisica a porcentaje de su maximo, recortado.

    El recorte va aqui y no en el firmware: el actuador es un actuador, no el
    sitio donde se decide que es un comando valido.
    """
    if maximo <= 0:
        return 0
    return max(-100, min(100, round(value / maximo * 100.0)))


class UartActuatorLink:
    """Mantiene el puerto abierto y expone la misma interfaz que usaban los
    transportes anteriores: send_command(), set_actuators(), emergency_stop().

    Quien llama no necesita saber si el cable esta conectado. Si no lo esta,
    el comando queda guardado como ultimo deseado y se aplicara al reabrir;
    el robot mientras tanto esta parado por su propio watchdog.
    """

    def __init__(self, v_max: float, omega_max: float,
                 port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD,
                 on_status=None, esclavo: bool = False):
        self.v_max = v_max
        self.omega_max = omega_max
        self.port = port
        self.baud = baud
        self._on_status = on_status
        # esclavo=True: NO tiene bucle de envío propio; EnlaceC3 le pasa las MISMAS tramas (misma secuencia) que van por el WiFi
        # mediante enviar(). Así la C3 deduplica por secuencia y gana siempre la trama más nueva, venga por el cable o por el aire.
        self.esclavo = esclavo
        self._ultimo_estado = 0.0
        self.recibidas = 0
        self._ultimo_error = None

        self._reader = None
        self._writer = None
        self._connected = False
        self._seq = 0
        self._tasks: list[asyncio.Task] = []
        self._stop = asyncio.Event()
        self.crc_errors = 0

        self._v = 0.0
        self._omega = 0.0
        # Reposo: camara al frente. Debe coincidir con SERVO_PARK_ANGLE del
        # firmware, para que la primera trama tras conectar no mueva el servo.
        self._servo = SERVO_HOME_DEG
        self._led = False
        # El barrido es un EVENTO, no un estado: si se reenviara en cada
        # trama el servo arrancaria un barrido nuevo 20 veces por segundo.
        self._scan_once = False

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def vivo(self) -> bool:
        """Puerto abierto Y la C3 contestando (estado en el último segundo)."""
        return self._connected and time.monotonic() - self._ultimo_estado < 1.0

    def enviar(self, trama: bytes) -> bool:
        """Escribe una trama ya armada (esclavo). No espera: si el cable no está, no pasa nada."""
        if not self._connected or self._writer is None:
            return False
        try:
            self._writer.write(trama)
        except Exception:
            return False
        return True

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
        # ⚠️ EL BIT VA INVERTIDO, y es necesario. `_led` True = foco
        # ENCENDIDO, pero se manda el flag al revés porque:
        #   · el firmware tiene LED_ACTIVO_EN_BAJO = true, así que "encender"
        #     (FLAG_LED puesto) escribe el pin en BAJO;
        #   · la etapa de potencia del foco de este robot es de lógica NORMAL:
        #     se enciende con el pin en ALTO.
        # Cambiar la constante del firmware no es opción: GPIO8 es pin de
        # strapping y con el valor contrario la placa no arranca (ver la
        # cabecera de esp32c3_actuator.ino). Así que la compensación vive
        # aquí, en un solo sitio y explicada.
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

    # ----- ciclo de vida -----
    def start(self):
        self._stop.clear()
        self._tasks = [asyncio.create_task(self._run())]

    async def stop(self):
        self._stop.set()
        for t in self._tasks:
            t.cancel()
        for t in self._tasks:
            try:
                await t
            except (asyncio.CancelledError, Exception):
                pass
        await self._close()

    async def _close(self):
        writer, self._writer = self._writer, None
        self._reader = None
        self._connected = False
        if writer is not None:
            try:
                writer.close()
            except Exception:
                pass

    async def _run(self):
        while not self._stop.is_set():
            try:
                self._reader, self._writer = await serial_asyncio.open_serial_connection(
                    url=self.port, baudrate=self.baud, dsrdtr=False, rtscts=False)
                try:                                  # DTR/RTS en reposo: en el USB-JTAG del C3 podrían provocar un reset
                    ser = self._writer.transport.serial
                    ser.dtr = False
                    ser.rts = False
                except Exception:
                    pass
                self._connected = True
                self._ultimo_error = None
                print(f"[serie] abierto {self.port} a {self.baud} baudios", flush=True)
                tareas = [self._read_loop()] + ([] if self.esclavo else [self._send_loop()])
                await asyncio.gather(*tareas)
            except Exception as exc:
                if not self._stop.is_set() and str(exc) != self._ultimo_error:     # un aviso por cambio, no uno cada 2 s
                    print(f"[serie] {self.port}: {exc}", flush=True)
                    self._ultimo_error = str(exc)
                await self._close()
                await asyncio.sleep(RECONNECT_DELAY_S)

    async def _send_loop(self):
        while self._connected and not self._stop.is_set():
            inicio = time.monotonic()
            self._writer.write(self.pack())
            await self._writer.drain()
            # El barrido ya viajo: se limpia DESPUES de escribir, no antes, o
            # un fallo de escritura se lo tragaria sin que nadie lo note.
            self._scan_once = False
            resto = SEND_PERIOD - (time.monotonic() - inicio)
            if resto > 0:
                await asyncio.sleep(resto)

    async def _read_loop(self):
        """Resincroniza byte a byte igual que el firmware.

        readexactly() sobre el flujo entero no sirve: si se pierde un byte,
        todas las tramas siguientes quedan corridas para siempre. Buscar el
        magic en cada vuelta es lo que permite recuperarse.
        """
        while self._connected and not self._stop.is_set():
            b = await self._reader.readexactly(1)
            if b[0] != STATUS_MAGIC:
                continue                      # basura o log del bootloader
            resto = await self._reader.readexactly(STATUS_SIZE - 1)
            trama = b + resto
            if crc8(trama[:-1]) != trama[-1]:
                self.crc_errors += 1
                continue
            self._handle_status(trama)

    def _handle_status(self, trama: bytes):
        (_magic, v_pct, omega_pct, servo, flags, seq_echo,
         _crc) = struct.unpack(STATUS_FORMAT, trama)
        status = {
            "v_cmd": v_pct / 100.0 * self.v_max,
            "omega_cmd": omega_pct / 100.0 * self.omega_max,
            "scanning": bool(flags & ST_SCANNING),
            "failsafe": bool(flags & ST_FAILSAFE),
            "servo_angle": servo,
            # Mismo desfase que en pack(): el firmware cree "encendido" lo
            # que en este foco es apagado. Sin invertir, el dashboard decía
            # ON con el foco apagado.
            "led_state": not bool(flags & ST_LED),
            "seq_echo": seq_echo,
            "_received_at": time.time(),
        }
        self._ultimo_estado = time.monotonic()
        self.recibidas += 1
        if self._on_status is not None:
            self._on_status(status)
