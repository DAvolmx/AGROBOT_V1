#!/usr/bin/env python3
"""
🌐 WiFi Auto-Connect: Jetson intenta conectar a LoRa_Wan como fallback.

ARQUITECTURA CORRECTA 2026-10-06:
  · DIR-615 (Router) → Cliente de CNT-RUIZ (internet)
  · JETSON_AP → AP local (192.168.12.1) sin dependencia de Internet
  · Jetson (opcional) → Se conecta a LoRa_Wan si está disponible (redundancia)

La Jetson SOLO intenta LoRa_Wan (no CNT-RUIZ):
  · iface ap0  → JETSON_AP (siempre levantado)
  · iface wlan0 → intenta LoRa_Wan (solo para redundancia)
"""
import subprocess
import time
import logging
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format="[wifi-auto] %(asctime)s — %(message)s"
)
log = logging.getLogger()

REDES = [
    {"ssid": "LoRa_Wan", "password": "admin1", "priority": 1},
]

def scan_networks():
    """Escanea redes disponibles."""
    try:
        result = subprocess.run(
            ["sudo", "nmcli", "dev", "wifi", "list"],
            capture_output=True,
            text=True,
            timeout=10
        )
        return result.stdout
    except Exception as e:
        log.error(f"Error escaneando: {e}")
        return ""

def connect_to_network(ssid: str, password: str, timeout: int = 30):
    """Conecta a una red WiFi."""
    try:
        log.info(f"Intentando conectar a {ssid}...")

        # Crear conexión con nmcli
        result = subprocess.run(
            ["sudo", "nmcli", "dev", "wifi", "connect", ssid, "password", password],
            capture_output=True,
            text=True,
            timeout=timeout
        )

        if result.returncode == 0:
            log.info(f"✅ Conectado a {ssid}")
            return True
        else:
            log.warning(f"❌ No se conectó a {ssid}: {result.stderr}")
            return False

    except subprocess.TimeoutExpired:
        log.warning(f"⏱️ Timeout intentando {ssid}")
        return False
    except Exception as e:
        log.error(f"Error conectando: {e}")
        return False

def get_connection_status():
    """Lee el estado actual de conexión."""
    try:
        result = subprocess.run(
            ["nmcli", "connection", "show", "--active"],
            capture_output=True,
            text=True,
            timeout=5
        )
        return result.stdout
    except:
        return ""

def main():
    """Bucle principal: intenta conectar a redes externas."""
    log.info("🤖 WiFi Auto-Connect iniciado")

    # Esperar a que AP esté listo (5 seg)
    time.sleep(5)

    reintentos = 0
    max_reintentos = 5

    while reintentos < max_reintentos:
        log.info(f"Intento {reintentos + 1}/{max_reintentos}")

        # Escanear
        redes_disponibles = scan_networks()

        # Intentar cada red por orden de prioridad
        conectado = False
        for red in sorted(REDES, key=lambda x: x["priority"]):
            if red["ssid"] in redes_disponibles:
                if connect_to_network(red["ssid"], red["password"]):
                    conectado = True
                    reintentos = max_reintentos  # Salir del loop
                    break
                time.sleep(2)

        if not conectado:
            reintentos += 1
            espera = min(30, 5 * (reintentos + 1))
            log.info(f"Reintentando en {espera}s...")
            time.sleep(espera)
        else:
            # Conectado: monitoreo pasivo cada 60s
            while True:
                time.sleep(60)
                status = get_connection_status()
                if "wlan0" not in status or "CNT-RUIZ" not in status and "LoRa_Wan" not in status:
                    log.warning("Conexión externa perdida, reintentando...")
                    reintentos = 0
                    break

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        log.info("Detenido")
    except Exception as e:
        log.error(f"Error fatal: {e}")
