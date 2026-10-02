"""
Enlace REDUNDANTE con el ESP32-C3 actuador: BLE + WiFi UDP a la vez.

La C3 es el enlace más importante del robot: sin ella no hay motores. Hasta
el 2026-09-30 era solo BLE, y el BLE falló de todas las formas posibles: la
placa dejaba de anunciarse, bluetoothd se caía en la Jetson, el adaptador
Intel se quedaba sordo y la placa nueva llega a -95 dBm (se cae cada pocos
minutos). Cada una de esas cosas dejaba el robot sin mando.

Cómo funciona:
  · Cada 50 ms se construye UNA trama (con su número de secuencia) y se
    manda por el WiFi; el BLE manda la suya en su propio bucle. Las dos
    salen del mismo estado (velocidad, servo, foco) de `BleActuatorLink`.
  · La C3 aplica la trama más NUEVA por número de secuencia venga por donde
    venga: una trama del WiFi que llega tarde no puede deshacer un "para"
    que ya llegó por BLE (ver aplicarComando() en el firmware).
  · El estado vuelve por las dos. Se reenvía al backend el del BLE; el del
    WiFi solo cuando el BLE lleva un rato callado, para no duplicar.
  · El watchdog de la C3 (500 ms sin órdenes -> motores parados) sigue
    siendo la red de seguridad: si caen las DOS vías, el robot para.

Con el firmware anterior (solo BLE) la vía WiFi simplemente nunca aparece y
todo funciona como antes.
"""
from __future__ import annotations

import asyncio
import time

from ble_link import SEND_PERIOD, BleActuatorLink
from udp_actuator_link import UdpActuatorLink

# Sin estado por BLE durante esto, el del WiFi pasa a ser el que se reenvía.
ESTADO_BLE_VIEJO_S = 0.3

# El robot gira AL REVÉS de lo que pide la C3: "izquierda" (omega > 0, como en
# ROS) lo giraba a la derecha (2026-10-02, visto por el usuario y medido por
# EKF/LIDAR, tras reconectar los drivers). Avanzar va bien, así que son los
# lados izquierdo/derecho cruzados. Se corrige aquí, el único punto por donde
# salen las órdenes a la C3 (BLE y WiFi), para que valga para botones,
# exploración, patrulla y Nav2. Si se recablean los drivers, volver a False.
INVERTIR_GIRO = True


class EnlaceC3:
    def __init__(self, ble: BleActuatorLink, wifi: UdpActuatorLink | None = None):
        self.ble = ble
        self.wifi = wifi
        self._on_status = None
        self._t_estado_ble = 0.0
        self._via = "ble"
        self._tarea: asyncio.Task | None = None
        self.cambios_de_via = 0
        self._vias_antes: tuple[bool, bool] | None = None
        ble._on_status = self._estado
        if wifi is not None:
            wifi._on_trama = self._trama_wifi
            # Con el WiFi vivo, el BLE reintenta con calma (ver ble_link).
            ble.otra_via_viva = lambda: wifi.conectado

    # ----- lo que usa jetson_server (misma API que BleActuatorLink) -----
    @property
    def connected(self) -> bool:
        return self.ble.connected or (self.wifi is not None and self.wifi.conectado)

    def send_command(self, v: float, omega: float, scan_cmd: int = 0):
        self.ble.send_command(v, -omega if INVERTIR_GIRO else omega, scan_cmd)

    def set_actuators(self, servo_angle: int | None = None, led: bool | None = None):
        self.ble.set_actuators(servo_angle, led)

    def emergency_stop(self):
        self.ble.emergency_stop()
        # La parada sale YA por el WiFi, sin esperar al próximo tick.
        self._enviar_wifi()

    def start(self):
        self.ble.start()
        if self.wifi is not None:
            self._tarea = asyncio.create_task(self._bucle_wifi())

    async def stop(self):
        if self._tarea is not None:
            self._tarea.cancel()
            try:
                await self._tarea
            except (asyncio.CancelledError, Exception):
                pass
        await self.ble.stop()

    def resumen(self) -> dict:
        r = self.ble.resumen()
        r["ble"] = self.ble.connected
        r["wifi"] = self.wifi.resumen() if self.wifi is not None else None
        r["conectado"] = self.connected
        r["via_estado"] = self._via
        r["cambios_de_via"] = self.cambios_de_via
        return r

    # ----- internos -----
    def _enviar_wifi(self):
        if self.wifi is None or self.wifi.ip_c3 is None:
            return
        trama = self.ble.pack()
        if self.wifi.enviar(trama):
            # El barrido de cámara es de un solo disparo: si salió por aquí,
            # que no vuelva a salir en la próxima trama del BLE.
            self.ble._scan_once = False

    async def _bucle_wifi(self):
        while True:
            inicio = time.monotonic()
            try:
                self._enviar_wifi()
                self._vigilar_vias()
            except Exception as exc:
                # Nunca dejar morir esta tarea: es la vía de respaldo.
                print(f"[c3-wifi] fallo: {type(exc).__name__}: {exc}", flush=True)
            resto = SEND_PERIOD - (time.monotonic() - inicio)
            await asyncio.sleep(max(resto, 0.005))

    def _vigilar_vias(self):
        """Avisa en el log cuando una vía cae o vuelve."""
        vias = (self.ble.connected, self.wifi is not None and self.wifi.conectado)
        if vias != self._vias_antes and self._vias_antes is not None:
            self.cambios_de_via += 1
            nombres = {(True, True): "BLE + WiFi", (True, False): "solo BLE",
                       (False, True): "solo WiFi", (False, False): "NINGUNA (la C3 para sola)"}
            print(f"[c3] vías activas: {nombres[vias]}", flush=True)
        self._vias_antes = vias

    def _estado(self, status: dict):
        """Estado ya decodificado por BleActuatorLink._handle_status."""
        if self._via == "ble":
            self._t_estado_ble = time.monotonic()
        elif time.monotonic() - self._t_estado_ble < ESTADO_BLE_VIEJO_S:
            return          # el BLE está vivo: su estado basta
        if INVERTIR_GIRO and "omega_cmd" in status:
            status["omega_cmd"] = -status["omega_cmd"]   # el eco, en el sentido del robot
        status["via_estado"] = self._via
        status["vias"] = {"ble": self.ble.connected,
                          "wifi": self.wifi is not None and self.wifi.conectado}
        if self._on_status is not None:
            self._on_status(status)

    def _trama_wifi(self, trama: bytes):
        self._via = "wifi"
        try:
            self.ble._handle_status(trama)
        finally:
            self._via = "ble"
