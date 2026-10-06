"""
AUTO COMUNICACIÓN: Integración automática de TODOS los sistemas de comunicación.

Incluye:
  - AP Vigilante (verifica cada 5s, recrea si se cae)
  - Comunicación Mejorada (potencia, canal, latencia)
  - Diagnóstico (rastrea caídas)
  - Torque Optimizer (adapta torque según terreno)
  - WiFi Manager (modo estación/patrullaje)

Se inicializa automáticamente, no requiere configuración manual.
"""
from __future__ import annotations

import asyncio
import sys
from pathlib import Path

# Importar todos los módulos
try:
    from ap_vigilante import crear_vigilante
except ImportError:
    crear_vigilante = None

try:
    from comunicacion_mejorada import crear_comunicacion_mejorada
except ImportError:
    crear_comunicacion_mejorada = None

try:
    from diagnostico_wifi import crear_diagnostico
except ImportError:
    crear_diagnostico = None

try:
    from torque_optimizer import crear_optimizer
except ImportError:
    crear_optimizer = None

try:
    from wifi_manager import abrir as abrir_wifi
except ImportError:
    abrir_wifi = None


class AutoComunicacion:
    """Gestor central de comunicación automática."""

    def __init__(self, config_store=None):
        self.config_store = config_store

        # Componentes
        self.ap_vigilante = None
        self.comunicacion = None
        self.diagnostico = None
        self.torque_opt = None
        self.wifi_mgr = None

        # Estado
        self._activo = False

    async def inicializar(self, modo_wifi: str = "patrullaje"):
        """Inicia TODOS los sistemas automáticamente."""
        print("[AUTO-COMUNICACION] iniciando sistema automático...", flush=True)

        # 1. AP Vigilante (lo más crítico)
        if crear_vigilante:
            try:
                self.ap_vigilante = await crear_vigilante()
                self.ap_vigilante.set_on_evento(self._evento_ap)
                print("[AUTO] ✓ AP Vigilante activo", flush=True)
            except Exception as e:
                print(f"[AUTO] ✗ Error AP Vigilante: {e}", flush=True)

        # 2. Comunicación Mejorada (WiFi + BLE)
        if crear_comunicacion_mejorada:
            try:
                self.comunicacion = await crear_comunicacion_mejorada()
                self.comunicacion.set_on_cambio(self._evento_comm)
                print("[AUTO] ✓ Comunicación Mejorada activa", flush=True)
            except Exception as e:
                print(f"[AUTO] ✗ Error Comunicación: {e}", flush=True)

        # 3. Diagnóstico de WiFi
        if crear_diagnostico:
            try:
                self.diagnostico = await crear_diagnostico()
                print("[AUTO] ✓ Diagnóstico WiFi activo", flush=True)
            except Exception as e:
                print(f"[AUTO] ✗ Error Diagnóstico: {e}", flush=True)

        # 4. Torque Optimizer
        if crear_optimizer:
            try:
                self.torque_opt = await crear_optimizer(self.config_store)
                self.torque_opt.set_on_cambio(self._evento_torque)
                print("[AUTO] ✓ Torque Optimizer activo", flush=True)
            except Exception as e:
                print(f"[AUTO] ✗ Error Torque: {e}", flush=True)

        # 5. WiFi Manager (modo)
        if abrir_wifi:
            try:
                self.wifi_mgr = await abrir_wifi(modo_inicial=modo_wifi)
                self.wifi_mgr.set_on_cambio(self._evento_wifi)
                print(f"[AUTO] ✓ WiFi Manager activo (modo: {modo_wifi})", flush=True)
            except Exception as e:
                print(f"[AUTO] ✗ Error WiFi Manager: {e}", flush=True)

        self._activo = True
        print("[AUTO-COMUNICACION] ✓ Sistema completo activo\n", flush=True)

    async def detener(self):
        """Detiene todos los sistemas."""
        if self.ap_vigilante:
            await self.ap_vigilante.stop()
        if self.comunicacion:
            await self.comunicacion.stop()
        if self.wifi_mgr:
            await self.wifi_mgr.stop()
        self._activo = False

    async def cambiar_modo_wifi(self, nuevo_modo: str):
        """Cambia entre estación y patrullaje."""
        if self.wifi_mgr:
            await self.wifi_mgr.cambiar_modo(nuevo_modo)

    def actualizar_sensores(self, inclinacion: float, temp_izq: float, temp_der: float):
        """Actualiza sensores para torque optimizer."""
        if self.torque_opt:
            self.torque_opt.actualizar_sensores(inclinacion, temp_izq, temp_der)

    def actualizar_estado_vias(self, ble_conectado: bool, wifi_conectado: bool):
        """Actualiza estado de BLE/WiFi para diagnóstico."""
        if self.diagnostico:
            self.diagnostico.actualizar_estado(ble_conectado, wifi_conectado)

    def resumen_completo(self) -> dict:
        """Retorna estado de TODOS los sistemas."""
        return {
            "ap_vigilante": self.ap_vigilante.resumen() if self.ap_vigilante else None,
            "comunicacion": self.comunicacion.resumen() if self.comunicacion else None,
            "diagnostico": self.diagnostico.resumen() if self.diagnostico else None,
            "torque": self.torque_opt.resumen() if self.torque_opt else None,
            "wifi": self.wifi_mgr.resumen() if self.wifi_mgr else None,
        }

    def _evento_ap(self, tipo: str, datos: dict):
        """Callback de eventos del AP."""
        print(f"[evento-ap] {tipo}: {datos}", flush=True)

    def _evento_comm(self, tipo: str, datos: dict):
        """Callback de eventos de comunicación."""
        print(f"[evento-comm] {tipo}: {datos}", flush=True)

    def _evento_torque(self, datos: dict):
        """Callback de ajustes de torque."""
        print(f"[evento-torque] {datos}", flush=True)

    def _evento_wifi(self, datos: dict):
        """Callback de cambios de WiFi."""
        print(f"[evento-wifi] {datos}", flush=True)


# Instancia global (singleton)
_instancia_global = None


async def inicializar_auto_comunicacion(config_store=None, modo_wifi: str = "patrullaje") -> AutoComunicacion:
    """
    Inicializa el sistema automático de comunicación.

    Uso en jetson_server.py:
        from auto_comunicacion import inicializar_auto_comunicacion

        # En setup:
        self.auto_comm = await inicializar_auto_comunicacion(self.config_store)

        # En bucle:
        self.auto_comm.actualizar_estado_vias(ble_ok, wifi_ok)
        self.auto_comm.actualizar_sensores(cabeceo, temp_izq, temp_der)

        # En cleanup:
        await self.auto_comm.detener()
    """
    global _instancia_global

    _instancia_global = AutoComunicacion(config_store)
    await _instancia_global.inicializar(modo_wifi)

    return _instancia_global


def obtener_auto_comunicacion() -> AutoComunicacion | None:
    """Retorna la instancia global si existe."""
    return _instancia_global
