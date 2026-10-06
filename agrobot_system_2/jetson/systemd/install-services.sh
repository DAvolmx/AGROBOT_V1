#!/bin/bash
# 🚀 AgroBot System - Instalador de Servicios (completo y autónomo)
set -e

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     AGROBOT — Instalación de Sistema Autónomo             ║"
echo "╚════════════════════════════════════════════════════════════╝"

REPO_DIR="/home/jetson/Desktop/AGROBOT/agrobot_system_2"

# Detectar si es root
if [[ $EUID -ne 0 ]]; then
   echo "⚠️  Este script debe ejecutarse como root"
   echo "Ejecuta: sudo bash $0"
   exit 1
fi

echo ""
echo "📋 PASO 1: Copiar configuración de hostapd"
mkdir -p /etc/hostapd
cp "$REPO_DIR/jetson/systemd/etc/hostapd/agrobot.conf" /etc/hostapd/agrobot.conf
echo "✅ Hostapd configurado con contraseña 'agrobot123'"

echo ""
echo "📋 PASO 2: Copiar configuración de dnsmasq"
mkdir -p /etc/dnsmasq.d
cp "$REPO_DIR/jetson/systemd/etc/dnsmasq.d/agrobot-ap.conf" /etc/dnsmasq.d/agrobot-ap.conf
echo "✅ DNSMASQ configurado (DHCP en 192.168.12.50-200)"

echo ""
echo "📋 PASO 3: Instalar servicios systemd"
mkdir -p /etc/systemd/system

# Copiar services
cp "$REPO_DIR/jetson/systemd/agrobot-ap0.service" /etc/systemd/system/
cp "$REPO_DIR/jetson/systemd/agrobot-hostapd.service" /etc/systemd/system/
cp "$REPO_DIR/jetson/systemd/agrobot-dnsmasq.service" /etc/systemd/system/
cp "$REPO_DIR/jetson/systemd/agrobot-wifi-auto.service" /etc/systemd/system/
cp "$REPO_DIR/jetson/systemd/agrobot-bluetooth-gw.service" /etc/systemd/system/

# Copiar timers (si existen)
if [ -d "$REPO_DIR/jetson/systemd/drop-ins" ]; then
    cp -r "$REPO_DIR/jetson/systemd/drop-ins"/* /etc/systemd/system/ 2>/dev/null || true
fi

systemctl daemon-reload
echo "✅ Servicios systemd instalados"

echo ""
echo "📋 PASO 4: Habilitar servicios para iniciar al boot"
systemctl enable agrobot-ap0.service
systemctl enable agrobot-hostapd.service
systemctl enable agrobot-dnsmasq.service
systemctl enable agrobot-wifi-auto.service
systemctl enable agrobot-bluetooth-gw.service
echo "✅ Servicios habilitados en boot"

echo ""
echo "📋 PASO 5: Verificar instalación de herramientas"

# Verificar dependencias
TOOLS=("hostapd" "dnsmasq" "rfcomm" "socat" "networkmanager")
for tool in "${TOOLS[@]}"; do
    if ! command -v $tool &> /dev/null; then
        echo "⚠️  Instalando $tool..."
        apt-get update > /dev/null
        apt-get install -y $tool > /dev/null 2>&1 || true
    else
        echo "✅ $tool disponible"
    fi
done

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║           ✅ INSTALACIÓN COMPLETADA                        ║"
echo "╚════════════════════════════════════════════════════════════╝"

echo ""
echo "🚀 Próximos pasos:"
echo "  1. Iniciar los servicios: sudo systemctl start agrobot-ap0.service"
echo "  2. Verificar estado: sudo systemctl status agrobot-*.service"
echo "  3. Ver logs: sudo journalctl -u agrobot-ap0.service -f"
echo ""
echo "📡 Redes disponibles:"
echo "  • JETSON_AP (192.168.12.1) — Contraseña: agrobot123"
echo "  • CNT-RUIZ (auto) — Contraseña: 1850370113"
echo "  • LoRa_Wan (auto) — Usuario: admin, Contraseña: admin1"
echo ""
echo "🌐 Dashboard:"
echo "  • WiFi: http://192.168.12.1:8000"
echo "  • Bluetooth: RFCOMM puerto 1 (desde Android)"
echo ""
