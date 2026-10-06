"""
Ejemplo mínimo de integración de WiFi + Diagnóstico + Torque en jetson_server.py

Copiar las partes relevantes a tu servidor y adaptarlas.
"""

# ===== En __init__ de tu servidor =====

async def setup_conectividad(self):
    """Inicializa WiFi, diagnóstico y torque."""

    # 1. WiFi Manager
    from wifi_manager import abrir as abrir_wifi
    self.wifi_mgr = await abrir_wifi(modo_inicial="patrullaje")
    self.wifi_mgr.set_on_cambio(self._on_cambio_wifi)
    print("[setup] WiFi manager iniciado", flush=True)

    # 2. Diagnóstico
    from diagnostico_wifi import crear_diagnostico
    self.diag_wifi = await crear_diagnostico()
    print("[setup] Diagnóstico WiFi iniciado", flush=True)

    # 3. Torque Optimizer
    from torque_optimizer import crear_optimizer
    self.torque_opt = await crear_optimizer(self.config_store)
    self.torque_opt.set_on_cambio(self._on_cambio_torque)
    print("[setup] Torque optimizer iniciado", flush=True)


# ===== Callbacks de cambios =====

def _on_cambio_wifi(self, estado: dict):
    """Se llama cuando cambia el estado del WiFi."""
    print(f"[callback-wifi] {estado}", flush=True)
    # Aquí podrías enviar notificación al dashboard, etc.


def _on_cambio_torque(self, cambios: dict):
    """Se llama cuando el optimizer ajusta parámetros."""
    print(f"[callback-torque] Ajustado: {cambios}", flush=True)


# ===== En el bucle principal de control =====

async def bucle_control(self):
    """Bucle principal del robot."""
    while not self._stop.is_set():
        try:
            # ... tu código actual ...

            # Actualizar diagnóstico (importante: llamar cada ciclo)
            if self.enlace_c3:
                self.diag_wifi.actualizar_estado(
                    ble=self.enlace_c3.ble.connected,
                    wifi=(self.enlace_c3.wifi.conectado
                          if self.enlace_c3.wifi else False)
                )

            # Actualizar sensores del torque (si tienes IMU)
            if hasattr(self, 'imu'):
                self.torque_opt.actualizar_sensores(
                    inclinacion=self.imu.get_cabeceo_deg(),
                    temp_izq=self.motor_temp_izq or 25,
                    temp_der=self.motor_temp_der or 25,
                )

            # Procesar control del robot...
            # await self.procesar_comando()

            await asyncio.sleep(0.05)  # 20 Hz

        except Exception as e:
            print(f"[error bucle control] {e}", flush=True)


# ===== En la ruta de estado/telemetría =====

def obtener_estado(self) -> dict:
    """Retorna estado completo del robot."""
    estado = {
        # ... estado existente ...

        "wifi": self.wifi_mgr.resumen() if self.wifi_mgr else None,
        "diagnostico_wifi": self.diag_wifi.resumen() if self.diag_wifi else None,
        "torque": self.torque_opt.resumen() if self.torque_opt else None,
    }
    return estado


# ===== Cambio de modo (cuando vuelve a estación) =====

async def cambiar_a_modo_estacion(self):
    """Llamar cuando el robot llega al dock/estación."""
    if self.wifi_mgr:
        await self.wifi_mgr.cambiar_modo("estacion")
        print("[servidor] Modo estación activado", flush=True)


async def cambiar_a_modo_patrullaje(self):
    """Llamar cuando el robot sale a patrullar."""
    if self.wifi_mgr:
        await self.wifi_mgr.cambiar_modo("patrullaje")
        print("[servidor] Modo patrullaje activado", flush=True)


# ===== Cleanup =====

async def shutdown(self):
    """Detiene todos los servicios."""
    if self.wifi_mgr:
        await self.wifi_mgr.stop()
    if self.diag_wifi:
        # El diagnóstico no tiene stop(), pero no hay recurso que liberar
        pass
    if self.torque_opt:
        # El optimizer tampoco necesita cleanup
        pass


# ===== Rutas HTTP/WebSocket (si usas FastAPI o similar) =====

@app.get("/api/diagnostico/wifi")
async def get_diagnostico_wifi():
    """Endpoint para ver diagnóstico de WiFi."""
    if not servidor.diag_wifi:
        return {"error": "diagnostico no disponible"}

    return {
        "resumen": servidor.diag_wifi.resumen(),
        "analisis": await servidor.diag_wifi.analizar_caidas(),
    }


@app.post("/api/wifi/cambiar-modo")
async def post_cambiar_modo(modo: str):
    """Cambiar entre 'estacion' y 'patrullaje'."""
    if modo not in ["estacion", "patrullaje"]:
        return {"error": "modo inválido"}

    if servidor.wifi_mgr:
        await servidor.wifi_mgr.cambiar_modo(modo)
        return {"ok": True, "modo": modo}

    return {"error": "wifi_mgr no disponible"}


@app.get("/api/torque/ajustes")
async def get_ajustes_torque():
    """Ver ajustes actuales de torque."""
    if servidor.torque_opt:
        return servidor.torque_opt.resumen()
    return {"error": "torque_opt no disponible"}
