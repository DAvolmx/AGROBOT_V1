#!/usr/bin/env python3
"""
📱 Bluetooth Gateway: Expone el Dashboard del AgroBot sobre Bluetooth.

La Jetson levanta un RFCOMM socket en BT que redirige HTTP al puerto 8000 local.
Desde un móvil con Android, se conecta:
  · Bluetooth → pareamiento automático
  · Conexión RFCOMM → ve 192.168.101.1:8000 (emulado por rfcomm)

Alternativa simplificada: usar iptables + rfcomm para redirigir.
"""
import subprocess
import logging
import time
import os

logging.basicConfig(level=logging.INFO, format="[bt-gw] %(message)s")
log = logging.getLogger()

def setup_bluetooth():
    """Configura el Bluetooth en la Jetson."""
    try:
        log.info("🔧 Configurando Bluetooth...")

        # Asegurar que bluetoothd está corriendo
        subprocess.run(["sudo", "systemctl", "start", "bluetooth"], check=False)
        time.sleep(1)

        # Enable discoverability en 0 (siempre)
        subprocess.run([
            "sudo", "btmgmt", "connectable", "on"
        ], check=False)

        subprocess.run([
            "sudo", "btmgmt", "discgap", "on"
        ], check=False)

        # Set friendly name
        subprocess.run([
            "sudo", "hciconfig", "hci0", "name", "AGROBOT"
        ], check=False)

        log.info("✅ Bluetooth configurado")
        return True

    except Exception as e:
        log.error(f"Error configurando BT: {e}")
        return False

def setup_rfcomm():
    """Abre puerto RFCOMM para HTTP."""
    try:
        log.info("🔌 Abriendo puerto RFCOMM 1...")

        # Bind /dev/rfcomm1
        subprocess.run([
            "sudo", "rfcomm", "bind", "/dev/rfcomm1", "00:00:00:00:00:00", "1"
        ], check=False, capture_output=True)

        time.sleep(1)

        # Verificar si se abrió
        if os.path.exists("/dev/rfcomm1"):
            log.info("✅ RFCOMM 1 abierto")
            return "/dev/rfcomm1"
        else:
            log.warning("⚠️ /dev/rfcomm1 no aparece, usando alternativa socat...")
            return None

    except Exception as e:
        log.error(f"Error RFCOMM: {e}")
        return None

def redirect_http_to_rfcomm(rfcomm_dev: str):
    """Redirige HTTP 8000 → RFCOMM usando socat o netcat."""
    if not rfcomm_dev:
        log.info("📡 HTTP solo disponible en 192.168.12.1:8000 (WiFi AP)")
        return False

    try:
        # Usar socat para redirigir
        cmd = [
            "socat",
            f"RFCOMM-LISTEN:1,bind=00:00:00:00:00:00",
            "TCP:127.0.0.1:8000"
        ]

        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(2)

        if proc.poll() is None:
            log.info("✅ HTTP redirigido a RFCOMM 1")
            return True
        else:
            log.error("socat no está disponible o falló")
            return False

    except FileNotFoundError:
        log.warning("⚠️ socat no instalado. Instalando...")
        try:
            subprocess.run(["sudo", "apt-get", "update"], check=False)
            subprocess.run(["sudo", "apt-get", "install", "-y", "socat"], check=False)
            # Reintentar
            return redirect_http_to_rfcomm(rfcomm_dev)
        except:
            return False

def main():
    log.info("🚀 Bluetooth Gateway iniciando...")

    # Configurar BT
    if not setup_bluetooth():
        log.error("No se pudo configurar Bluetooth")
        return False

    # Abrir RFCOMM
    rfcomm_dev = setup_rfcomm()

    # Redirigir HTTP
    if not redirect_http_to_rfcomm(rfcomm_dev):
        log.warning("Redireccionamiento RFCOMM no disponible")

    log.info("📱 Esperando conexiones Bluetooth...")

    # Mantener corriendo
    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        log.info("Detenido")

if __name__ == "__main__":
    main()
