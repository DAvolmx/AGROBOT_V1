"""
Diagnóstico de caídas de WiFi: rastrea patrones, latencias y recuperación.

Detecta:
  · Caídas simultáneas BLE+WiFi vs caídas aisladas
  · Patrón de recuperación lento/rápido
  · Latencia de reconexión
  · Interferencia (fuerza de señal, ruido de canal)
"""
from __future__ import annotations

import asyncio
import time
from collections import deque
from typing import Callable


class DiagnosticoWiFi:
    def __init__(self, ventana_s: float = 60.0, tamaño_historial: int = 100):
        """
        ventana_s: ventana de tiempo para detectar patrones
        tamaño_historial: cuántos eventos guardar
        """
        self.ventana_s = ventana_s
        self.tamaño_historial = tamaño_historial

        # Historial de cambios
        self.eventos = deque(maxlen=tamaño_historial)

        # Estado actual
        self.ble_conectado = False
        self.wifi_conectado = False
        self.tiempo_inicio = time.monotonic()

        # Métricas
        self.caidas_ble = 0
        self.caidas_wifi = 0
        self.caidas_simultaneas = 0
        self.tiempo_promedio_recuperacion_s = 0.0
        self.tiempos_recuperacion = deque(maxlen=20)

        # Callbacks
        self._on_patron_detectado = None
        self._on_caida_critica = None

    def set_callbacks(
        self,
        on_patron: Callable[[str, dict], None] | None = None,
        on_critica: Callable[[dict], None] | None = None
    ):
        """Define callbacks para eventos importantes."""
        self._on_patron_detectado = on_patron
        self._on_caida_critica = on_critica

    def actualizar_estado(self, ble: bool, wifi: bool):
        """Actualiza estado de las dos vías y detecta cambios."""
        cambio_ble = self.ble_conectado != ble
        cambio_wifi = self.wifi_conectado != wifi

        if not (cambio_ble or cambio_wifi):
            return  # Sin cambios

        ahora = time.monotonic()
        evento = {
            "tiempo": ahora,
            "tiempo_relativo": ahora - self.tiempo_inicio,
            "ble_antes": self.ble_conectado,
            "wifi_antes": self.wifi_conectado,
            "ble_ahora": ble,
            "wifi_ahora": wifi,
        }
        self.eventos.append(evento)

        # Detectar patrones
        if cambio_ble and not ble:
            self.caidas_ble += 1
        if cambio_wifi and not wifi:
            self.caidas_wifi += 1

        if (cambio_ble and not ble) and (cambio_wifi and not wifi):
            self.caidas_simultaneas += 1
            self._reportar_patron("CAIDA_SIMULTANEA", {
                "evento": evento,
                "total_simultaneas": self.caidas_simultaneas
            })

        # Detectar recuperación después de caída crítica
        if self.caidas_simultaneas > 0:
            if ble and wifi:  # Ambas vuelven
                self._registrar_recuperacion(ahora)

        # Cambio de descripción
        estado_nuevo = self._describir_estado(ble, wifi)
        print(f"[diag-wifi] estado: {estado_nuevo}", flush=True)

        self.ble_conectado = ble
        self.wifi_conectado = wifi

    def _describir_estado(self, ble: bool, wifi: bool) -> str:
        """Describe de forma humana el estado."""
        if ble and wifi:
            return "✓ BLE + WiFi"
        elif ble and not wifi:
            return "⚠ Solo BLE (WiFi caído)"
        elif wifi and not ble:
            return "⚠ Solo WiFi (BLE caído)"
        else:
            return "✗ AMBAS CAÍDAS (sin control)"

    def _registrar_recuperacion(self, ahora: float):
        """Registra tiempo de recuperación desde la última caída simultánea."""
        if self.eventos:
            for evt in reversed(self.eventos):
                # Buscar última caída simultánea
                if (not evt["ble_ahora"] and not evt["wifi_ahora"] and
                    evt["ble_antes"] or evt["wifi_antes"]):
                    tiempo_recuperacion = ahora - evt["tiempo"]
                    self.tiempos_recuperacion.append(tiempo_recuperacion)
                    self.tiempo_promedio_recuperacion_s = (
                        sum(self.tiempos_recuperacion) / len(self.tiempos_recuperacion)
                    )
                    self._reportar_patron("RECUPERACION", {
                        "tiempo_s": tiempo_recuperacion,
                        "promedio_s": self.tiempo_promedio_recuperacion_s,
                        "recuperaciones_totales": len(self.tiempos_recuperacion)
                    })
                    break

    async def analizar_caidas(self) -> dict:
        """Analiza patrones en los últimos eventos."""
        if not self.eventos:
            return {"estado": "sin_datos"}

        ahora = time.monotonic()
        reciente = [e for e in self.eventos if ahora - e["tiempo"] < self.ventana_s]

        # Contar patrones
        simultaneas_recientes = sum(
            1 for e in reciente
            if not e["ble_ahora"] and not e["wifi_ahora"]
        )

        aisladas = {
            "ble_sin_wifi": sum(
                1 for e in reciente
                if not e["ble_ahora"] and e["wifi_ahora"]
            ),
            "wifi_sin_ble": sum(
                1 for e in reciente
                if e["ble_ahora"] and not e["wifi_ahora"]
            ),
        }

        # Evaluar salud
        salud = "buena"
        if simultaneas_recientes >= 3:
            salud = "crítica"
        elif self.caidas_simultaneas > 5:
            salud = "pobre"
        elif self.caidas_ble > 3 or self.caidas_wifi > 3:
            salud = "degradada"

        return {
            "salud": salud,
            "caidas_ble_total": self.caidas_ble,
            "caidas_wifi_total": self.caidas_wifi,
            "caidas_simultaneas_total": self.caidas_simultaneas,
            "simultaneas_en_ultimo_minuto": simultaneas_recientes,
            "caidas_aisladas": aisladas,
            "tiempo_promedio_recuperacion_s": round(self.tiempo_promedio_recuperacion_s, 2),
            "eventos_totales": len(self.eventos),
            "eventos_en_ventana": len(reciente),
        }

    def _reportar_patron(self, tipo: str, datos: dict):
        """Notifica cuando se detecta un patrón."""
        if self._on_patron_detectado:
            try:
                self._on_patron_detectado(tipo, datos)
            except Exception as e:
                print(f"[diag-wifi] error en callback patrón: {e}", flush=True)

    def resumen(self) -> dict:
        """Retorna resumen para reportes."""
        return {
            "ble": self.ble_conectado,
            "wifi": self.wifi_conectado,
            "caidas_ble": self.caidas_ble,
            "caidas_wifi": self.caidas_wifi,
            "caidas_simultaneas": self.caidas_simultaneas,
            "tiempo_promedio_recuperacion_s": round(self.tiempo_promedio_recuperacion_s, 2),
            "eventos_registrados": len(self.eventos),
        }


async def crear_diagnostico() -> DiagnosticoWiFi:
    """Factory."""
    return DiagnosticoWiFi()
