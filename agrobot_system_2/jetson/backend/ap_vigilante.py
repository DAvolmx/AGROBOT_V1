"""
VIGILANTE DEL AP: Loop agresivo que mantiene JETSON_AP vivo 24/7.

Responsabilidades:
  · Verifica cada 5s que el AP esté activo
  · Si cae: lo recrea INMEDIATAMENTE (sin esperar)
  · Maximiza potencia TX cada ciclo
  · Detecta estado del AP en tiempo real
  · Reporta intentos y éxitos
"""
from __future__ import annotations

import asyncio
import subprocess
import time
from typing import Callable


class APVigilante:
    def __init__(self):
        self.ap_nombre = "JETSON_AP"
        self.ap_password = "agrobot123"
        self.ap_ip_base = "192.168.12.1"
        self.ap_red = "192.168.12.0/24"

        # Estado
        self.ap_activo = False
        self.tiempo_ultimo_check = 0.0
        self.tiempo_ultimo_recreo = 0.0
        self.tiempo_ap_levantado = 0.0

        # Estadísticas
        self.verificaciones = 0
        self.caidas_detectadas = 0
        self.recreos_exitosos = 0
        self.recreos_fallidos = 0
        self.potencia_ajustes = 0

        # Callback
        self._on_evento = None

        # Tareas
        self._bucle_vigilancia = None
        self._bucle_potencia = None
        self._running = False

    def set_on_evento(self, callback: Callable[[str, dict], None]):
        """Callback(tipo_evento, datos)."""
        self._on_evento = callback

    async def start(self):
        """Inicia vigilancia agresiva del AP."""
        self._running = True

        # Loop principal: verificar AP cada 5 segundos
        self._bucle_vigilancia = asyncio.create_task(self._vigilancia_ap())

        # Loop secundario: maximizar potencia cada 30 segundos
        self._bucle_potencia = asyncio.create_task(self._vigilancia_potencia())

        print(f"[ap-vigilante] iniciado - vigilando {self.ap_nombre}", flush=True)

    async def stop(self):
        """Detiene vigilancia."""
        self._running = False
        if self._bucle_vigilancia:
            self._bucle_vigilancia.cancel()
        if self._bucle_potencia:
            self._bucle_potencia.cancel()

        try:
            if self._bucle_vigilancia:
                await self._bucle_vigilancia
            if self._bucle_potencia:
                await self._bucle_potencia
        except asyncio.CancelledError:
            pass

    async def _vigilancia_ap(self):
        """Loop principal: verifica cada 5 segundos."""
        while self._running:
            try:
                self.verificaciones += 1
                self.tiempo_ultimo_check = time.monotonic()

                # 1. Verificar si el AP está UP
                ap_esta_up = await self._verificar_ap_up()

                # 2. Verificar si hay cliente conectado
                cliente_conectado = await self._verificar_cliente()

                # 3. Tomar decisión
                if not ap_esta_up:
                    self.caidas_detectadas += 1
                    self._log(f"[CAÍDA] AP detectado DOWN (caída #{self.caidas_detectadas})")

                    # Recrear INMEDIATAMENTE
                    await self._recrear_ap()

                elif ap_esta_up and not self.ap_activo:
                    # Recuperación: AP vuelve después de caída
                    self.ap_activo = True
                    self.tiempo_ap_levantado = time.monotonic()
                    self._avisar("AP_RECUPERADO", {
                        "verificacion": self.verificaciones,
                        "caidas_totales": self.caidas_detectadas,
                    })

                # Estado actual
                self.ap_activo = ap_esta_up

                # Esperar para siguiente verificación
                await asyncio.sleep(5)

            except Exception as e:
                self._log(f"[ERROR vigilancia] {e}")
                await asyncio.sleep(5)

    async def _vigilancia_potencia(self):
        """Loop secundario: maximiza potencia cada 30s."""
        while self._running:
            try:
                await asyncio.sleep(30)

                # Establecer potencia máxima
                resultado = await self._ejecutar(
                    ["iw", "wlan0", "set", "txpower", "fixed", "3000"]
                )

                if resultado["exitoso"]:
                    self.potencia_ajustes += 1
                    self._log(f"[potencia] maximizada a 30 dBm (ajuste #{self.potencia_ajustes})")

            except Exception as e:
                self._log(f"[ERROR potencia] {e}")

    async def _verificar_ap_up(self) -> bool:
        """Verifica si el AP está levantado (interfaz UP y activo)."""
        try:
            # Método 1: nmcli device status
            resultado = await self._ejecutar(["nmcli", "device", "status"])

            if resultado["exitoso"]:
                lineas = resultado["salida"].split('\n')
                for linea in lineas:
                    if "wlan0" in linea and "connected" in linea:
                        return True

            return False
        except Exception:
            return False

    async def _verificar_cliente(self) -> bool:
        """Verifica si hay clientes conectados al AP."""
        try:
            # Verificar IPs en la red del AP
            resultado = await self._ejecutar(
                ["iw", "dev", "wlan0", "station", "dump"]
            )

            if resultado["exitoso"]:
                # Si hay líneas de estaciones, hay clientes
                return len([l for l in resultado["salida"].split('\n')
                           if l.strip() and 'Station' in l]) > 0

            return False
        except Exception:
            return False

    async def _recrear_ap(self):
        """Recrea el AP INMEDIATAMENTE."""
        self._log(f"[RECREAR] intentando levantar {self.ap_nombre}...")

        # Paso 1: Apagar conexión anterior
        await self._ejecutar(["nmcli", "device", "disconnect", "wlan0"])
        await asyncio.sleep(1)

        # Paso 2: Levantar AP
        resultado = await self._ejecutar([
            "nmcli", "device", "wifi", "hotspot",
            "ifname", "wlan0",
            "ssid", self.ap_nombre,
            "password", self.ap_password,
            "band", "bg",  # 2.4 GHz (más alcance que 5 GHz)
        ])

        if resultado["exitoso"]:
            self.recreos_exitosos += 1
            self.ap_activo = True
            self.tiempo_ap_levantado = time.monotonic()

            self._avisar("AP_RECREADO", {
                "intento": self.recreos_exitosos + self.recreos_fallidos,
                "exitosos": self.recreos_exitosos,
                "fallidos": self.recreos_fallidos,
            })

            self._log(f"[✓] {self.ap_nombre} levantado (recreo #{self.recreos_exitosos})")
        else:
            self.recreos_fallidos += 1
            self._log(f"[✗] Error recreando AP: {resultado['error']}")

            # Reintentar más agresivamente
            await asyncio.sleep(2)
            await self._ejecutar(["iw", "phy", "phy0", "set", "antenna", "all"])
            await asyncio.sleep(1)

            # Segundo intento sin opciones avanzadas
            resultado2 = await self._ejecutar([
                "nmcli", "device", "wifi", "hotspot",
                "ifname", "wlan0",
                "ssid", self.ap_nombre,
                "password", self.ap_password,
            ])

            if resultado2["exitoso"]:
                self.recreos_exitosos += 1
                self.ap_activo = True
                self._log(f"[✓] {self.ap_nombre} levantado en reintentos (recreo #{self.recreos_exitosos})")
            else:
                self._log(f"[✗✗] FALLO CRÍTICO recreando AP: {resultado2['error']}")

        self.tiempo_ultimo_recreo = time.monotonic()

    async def _ejecutar(self, cmd: list) -> dict:
        """Ejecuta comando shell de forma segura."""
        try:
            resultado = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=10,
            )
            return {
                "exitoso": resultado.returncode == 0,
                "salida": resultado.stdout,
                "error": resultado.stderr,
                "codigo": resultado.returncode,
            }
        except subprocess.TimeoutExpired:
            return {"exitoso": False, "error": "Timeout", "codigo": -1}
        except Exception as e:
            return {"exitoso": False, "error": str(e), "codigo": -1}

    def _log(self, mensaje: str):
        """Registra log."""
        print(f"{mensaje}", flush=True)

    def _avisar(self, evento: str, datos: dict):
        """Notifica callback."""
        if self._on_evento:
            try:
                self._on_evento(evento, datos)
            except Exception as e:
                self._log(f"[ERROR callback] {e}")

    def resumen(self) -> dict:
        """Retorna estado actual."""
        tiempo_uptime = time.monotonic() - self.tiempo_ap_levantado if self.ap_activo else 0
        return {
            "ap_activo": self.ap_activo,
            "tiempo_uptime_s": round(tiempo_uptime, 1),
            "verificaciones_totales": self.verificaciones,
            "caidas_detectadas": self.caidas_detectadas,
            "recreos_exitosos": self.recreos_exitosos,
            "recreos_fallidos": self.recreos_fallidos,
            "potencia_ajustes": self.potencia_ajustes,
            "tiempo_ultimo_check": round(time.monotonic() - self.tiempo_ultimo_check, 1),
        }


async def crear_vigilante() -> APVigilante:
    """Factory."""
    vigilante = APVigilante()
    await vigilante.start()
    return vigilante
