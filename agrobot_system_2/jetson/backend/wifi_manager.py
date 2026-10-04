"""
Gestor de WiFi con máxima potencia y modos de operación (Estación/Patrullaje).

Responsabilidades:
  · Maximiza la potencia TX del AP (30 dBm)
  · Detecta la red Lora_Wan y reconecta automáticamente en modo Estación
  · Mantiene el AP de patrullaje con reintentos cada 60s si falla
  · Monitorea salud del WiFi y reporta caídas
"""
from __future__ import annotations

import asyncio
import subprocess
import time
import re
from typing import Callable
from pathlib import Path


class WiFiManager:
    def __init__(self, modo_inicial: str = "patrullaje"):
        """
        ARQUITECTURA CORRECTA (2026-10-04):
        - Jetson LEVANTA el AP "JETSON_AP" (192.168.12.1)
        - C3 se CONECTA como cliente WiFi al AP de Jetson
        - LoRa V3 se CONECTA como cliente WiFi al AP de Jetson
        - Backup: ambos tienen BLE para redundancia si WiFi cae

        Los "modos" son ahora obsoletos (mantienen compatibilidad):
        - "patrullaje": monitorea AP Jetson (modo normal, por defecto)
        - "estacion": también intenta conectar a Lora_Wan router (opcional)
        """
        self.modo = modo_inicial
        # AP LEVANTADO POR JETSON (no por C3)
        self.jetson_ap_ssid = "JETSON_AP"
        self.jetson_ap_password = "agrobot123"  # Contraseña del AP de Jetson
        self.lora_wan_ssid = "Lora_Wan"        # Red router opcional
        self.lora_wan_pass = ""  # Sin contraseña o configurable

        self.conectado = False
        self._on_cambio = None
        self._tarea_monitoreo: asyncio.Task | None = None
        self._tarea_reintentos: asyncio.Task | None = None

        # Estadísticas
        self.caidas_total = 0
        self.reconexiones = 0
        self.tiempo_ultimo_estado = 0.0

    def set_on_cambio(self, callback: Callable[[dict], None]):
        """Callback cuando cambia el estado (conectado/desconectado/cambio_modo)."""
        self._on_cambio = callback

    async def start(self):
        """Inicia monitoreo de WiFi y mantiene los modos activos."""
        # Maximizar potencia TX del AP
        await self._set_max_power()

        # Levantar AP o conectarse según modo
        if self.modo == "estacion":
            await self._conectar_lora_wan()
        else:
            await self._levantar_ap()

        # Monitoreo continuo
        self._tarea_monitoreo = asyncio.create_task(self._bucle_monitoreo())
        self._tarea_reintentos = asyncio.create_task(self._bucle_reintentos())

    async def stop(self):
        """Detiene monitoreo y limpia tareas."""
        if self._tarea_monitoreo:
            self._tarea_monitoreo.cancel()
        if self._tarea_reintentos:
            self._tarea_reintentos.cancel()
        try:
            if self._tarea_monitoreo:
                await self._tarea_monitoreo
            if self._tarea_reintentos:
                await self._tarea_reintentos
        except asyncio.CancelledError:
            pass

    async def cambiar_modo(self, nuevo_modo: str):
        """Cambia entre 'estacion' (Lora_Wan) y 'patrullaje' (AP propio)."""
        if nuevo_modo == self.modo:
            return

        print(f"[wifi] cambiando de modo: {self.modo} -> {nuevo_modo}", flush=True)
        self.modo = nuevo_modo

        if nuevo_modo == "estacion":
            # Desactivar AP, conectar a Lora_Wan
            await self._apagar_ap()
            await asyncio.sleep(2)
            await self._conectar_lora_wan()
        else:
            # Desconectar de Lora_Wan, levantar AP
            await self._desconectar_lora_wan()
            await asyncio.sleep(2)
            await self._levantar_ap()

        self._avisar_cambio()

    async def _set_max_power(self):
        """Maximiza la potencia TX del AP a 30 dBm (máximo legal en muchas regiones)."""
        try:
            # Verificar región e inalámbrica
            resultado = subprocess.run(
                ["iw", "reg", "get"],
                capture_output=True, text=True, timeout=5
            )
            print(f"[wifi] región reguladora: {resultado.stdout.strip()}", flush=True)

            # Establecer potencia en la interfaz wlan0
            subprocess.run(
                ["iw", "set", "txpower", "fixed", "3000"],  # 3000 = 30 dBm en 1/100 dBm
                capture_output=True, timeout=5
            )

            # Verificar que se aplicó
            resultado = subprocess.run(
                ["iw", "wlan0", "get", "txpower"],
                capture_output=True, text=True, timeout=5
            )
            if resultado.returncode == 0:
                print(f"[wifi] potencia TX: {resultado.stdout.strip()}", flush=True)
        except Exception as e:
            print(f"[wifi] error al maximizar potencia: {e}", flush=True)

    async def _levantar_ap(self):
        """Jetson ya LEVANTA el AP "JETSON_AP" en setup(); esto solo verifica conexión."""
        try:
            # El AP es levantado por ap_vigilante.py continuamente
            # Solo verificamos que estamos conectados (si acaso)
            resultado = subprocess.run(
                ["nmcli", "device", "status"],
                capture_output=True, text=True, timeout=5
            )
            if "wlan0" in resultado.stdout and "ap" in resultado.stdout.lower():
                self.conectado = True
                print(f"[wifi] AP Jetson 'JETSON_AP' está levantado", flush=True)
                self._avisar_cambio()
            else:
                print(f"[wifi] AP Jetson no está activo; ap_vigilante.py debería levantarlo", flush=True)
        except Exception as e:
            print(f"[wifi] excepción verificando AP: {e}", flush=True)

    async def _apagar_ap(self):
        """NO apagar el AP de Jetson; siempre debe estar activo.
        Esta función es un legacy de la arquitectura antigua."""
        try:
            print(f"[wifi] aviso: intentó apagar AP pero Jetson DEBE ser AP siempre", flush=True)
            # No hacer nada: Jetson es el AP y no debe apagarse
        except Exception as e:
            print(f"[wifi] error: {e}", flush=True)

    async def _conectar_lora_wan(self):
        """Conecta a la red Lora_Wan en la estación base."""
        try:
            # Buscar la red
            resultado = subprocess.run(
                ["nmcli", "dev", "wifi", "list"],
                capture_output=True, text=True, timeout=10
            )

            if self.lora_wan_ssid in resultado.stdout:
                # Conectar
                cmd = ["nmcli", "device", "wifi", "connect", self.lora_wan_ssid]
                if self.lora_wan_pass:
                    cmd.extend(["password", self.lora_wan_pass])

                resultado = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
                if resultado.returncode == 0:
                    self.conectado = True
                    print(f"[wifi] conectado a {self.lora_wan_ssid}", flush=True)
                    self._avisar_cambio()
                else:
                    print(f"[wifi] error conectando a {self.lora_wan_ssid}: {resultado.stderr}", flush=True)
            else:
                print(f"[wifi] {self.lora_wan_ssid} no encontrada", flush=True)
        except Exception as e:
            print(f"[wifi] excepción conectando a Lora_Wan: {e}", flush=True)

    async def _desconectar_lora_wan(self):
        """Desconecta de la red Lora_Wan."""
        try:
            subprocess.run(
                ["nmcli", "device", "disconnect", "wlan0"],
                capture_output=True, timeout=5
            )
            self.conectado = False
            print(f"[wifi] desconectado de {self.lora_wan_ssid}", flush=True)
        except Exception as e:
            print(f"[wifi] error desconectando: {e}", flush=True)

    async def _bucle_monitoreo(self):
        """Monitorea la salud del WiFi cada 10 segundos."""
        while True:
            try:
                await asyncio.sleep(10)
                estado_actual = await self._verificar_conectividad()

                if not estado_actual and self.conectado:
                    self.caidas_total += 1
                    self.conectado = False
                    print(f"[wifi] CAÍDA DETECTADA (total: {self.caidas_total})", flush=True)
                    self._avisar_cambio()
                elif estado_actual and not self.conectado:
                    self.reconexiones += 1
                    self.conectado = True
                    print(f"[wifi] reconectado (total: {self.reconexiones})", flush=True)
                    self._avisar_cambio()

                self.tiempo_ultimo_estado = time.monotonic()
            except Exception as e:
                print(f"[wifi-monitoreo] error: {e}", flush=True)

    async def _bucle_reintentos(self):
        """Si el modo es 'patrullaje' y AP cae, reinicia cada 60s."""
        while True:
            try:
                await asyncio.sleep(60)

                if self.modo == "patrullaje" and not self.conectado:
                    print(f"[wifi] reintentando AP (pasó 1 minuto sin conexión)", flush=True)
                    await self._levantar_ap()
            except Exception as e:
                print(f"[wifi-reintentos] error: {e}", flush=True)

    async def _verificar_conectividad(self) -> bool:
        """Verifica que el AP de Jetson esté levantado (arquitectura 2026-10-04)."""
        try:
            resultado = subprocess.run(
                ["nmcli", "device", "status"],
                capture_output=True, text=True, timeout=5
            )
            # Jetson SIEMPRE debe ser AP; verificamos que wlan0 está en modo AP
            return "wlan0" in resultado.stdout and ("ap" in resultado.stdout.lower() or "activated" in resultado.stdout.lower())
        except Exception:
            return False

    def resumen(self) -> dict:
        """Retorna estado actual para reportes."""
        return {
            "modo": self.modo,
            "conectado": self.conectado,
            "caidas_total": self.caidas_total,
            "reconexiones": self.reconexiones,
            "tiempo_estado": self.tiempo_ultimo_estado
        }

    def _avisar_cambio(self):
        """Notifica callback de cambios."""
        if self._on_cambio:
            try:
                self._on_cambio(self.resumen())
            except Exception as e:
                print(f"[wifi] error en callback: {e}", flush=True)


async def abrir(modo_inicial: str = "patrullaje") -> WiFiManager:
    """Factory para crear e iniciar el gestor de WiFi."""
    manager = WiFiManager(modo_inicial)
    await manager.start()
    return manager
