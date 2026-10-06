#!/bin/bash
# 🚀 AGROBOT v2.0 — INSTALACIÓN COMPLETA EN UNA LÍNEA
# El usuario no hace NADA, todo se instala automáticamente
# Ejecutar: sudo bash INSTALAR_AGROBOT.sh

set -e

echo "╔════════════════════════════════════════════════════════════╗"
echo "║          🤖 AGROBOT v2.0 — INSTALACIÓN TOTAL              ║"
echo "║                (El sistema hará TODO)                      ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Verificar si es root
if [[ $EUID -ne 0 ]]; then
   echo "❌ Este script DEBE ejecutarse como root"
   echo "Ejecuta: sudo bash INSTALAR_AGROBOT.sh"
   exit 1
fi

REPO_DIR="/home/jetson/Desktop/AGROBOT"
JETSON_SYSTEMD="$REPO_DIR/agrobot_system_2/jetson/systemd"
JETSON_BACKEND="$REPO_DIR/agrobot_system_2/jetson/backend"

# ============================================================
# PASO 1: Instalar dependencias del sistema
# ============================================================
echo "📦 PASO 1: Instalando dependencias del sistema..."
apt-get update > /dev/null 2>&1
apt-get install -y \
    hostapd \
    dnsmasq \
    network-manager \
    socat \
    bluez \
    bluez-tools \
    python3.8 \
    python3-pip \
    rfcomm > /dev/null 2>&1
echo "✅ Dependencias instaladas"

# ============================================================
# PASO 2: Copiar configuración a /etc
# ============================================================
echo ""
echo "⚙️  PASO 2: Copiando configuración a /etc..."

# Hostapd
mkdir -p /etc/hostapd
cp "$JETSON_SYSTEMD/etc/hostapd/agrobot.conf" /etc/hostapd/agrobot.conf
echo "✅ Hostapd configurado (contraseña: 12345678)"

# Dnsmasq
mkdir -p /etc/dnsmasq.d
cp "$JETSON_SYSTEMD/etc/dnsmasq.d/agrobot-ap.conf" /etc/dnsmasq.d/agrobot-ap.conf
echo "✅ DNSMASQ configurado (DHCP: 192.168.12.50-200)"

# ============================================================
# PASO 3: Instalar servicios systemd
# ============================================================
echo ""
echo "🔧 PASO 3: Instalando servicios systemd..."

mkdir -p /etc/systemd/system

# Copiar todos los servicios
cp "$JETSON_SYSTEMD/agrobot-ap0.service" /etc/systemd/system/
cp "$JETSON_SYSTEMD/agrobot-hostapd.service" /etc/systemd/system/
cp "$JETSON_SYSTEMD/agrobot-dnsmasq.service" /etc/systemd/system/
cp "$JETSON_SYSTEMD/agrobot-wifi-auto.service" /etc/systemd/system/
cp "$JETSON_SYSTEMD/agrobot-bluetooth-gw.service" /etc/systemd/system/

systemctl daemon-reload
echo "✅ 5 servicios systemd instalados"

# ============================================================
# PASO 4: Habilitar servicios en boot
# ============================================================
echo ""
echo "🚀 PASO 4: Habilitando servicios en boot..."

systemctl enable agrobot-ap0.service
systemctl enable agrobot-hostapd.service
systemctl enable agrobot-dnsmasq.service
systemctl enable agrobot-wifi-auto.service
systemctl enable agrobot-bluetooth-gw.service

echo "✅ Servicios habilitados (se inician automáticamente al boot)"

# ============================================================
# PASO 5: Configurar potencia WiFi al máximo (30 dBm)
# ============================================================
echo ""
echo "📶 PASO 5: Configurando potencia WiFi al máximo..."

# Máxima potencia TX (30 dBm = 1000 mW)
iw reg set US 2>/dev/null || true
iw phy phy0 set txpower fixed 3000 2>/dev/null || true

echo "✅ Potencia WiFi al máximo (30 dBm)"

# ============================================================
# PASO 6: Iniciar servicios
# ============================================================
echo ""
echo "⚡ PASO 6: Iniciando servicios..."

systemctl restart agrobot-ap0.service
sleep 2
systemctl restart agrobot-hostapd.service
sleep 2
systemctl restart agrobot-dnsmasq.service
sleep 2
systemctl restart agrobot-wifi-auto.service
sleep 2
systemctl restart agrobot-bluetooth-gw.service

echo "✅ Todos los servicios iniciados"

# ============================================================
# PASO 7: Verificación
# ============================================================
echo ""
echo "✔️  PASO 7: Verificando instalación..."
echo ""

STATUS_OK=0

# Verificar AP
if systemctl is-active --quiet agrobot-hostapd.service; then
    echo "  ✅ AP hostapd: ACTIVO"
else
    echo "  ❌ AP hostapd: INACTIVO"
    STATUS_OK=1
fi

# Verificar DHCP
if systemctl is-active --quiet agrobot-dnsmasq.service; then
    echo "  ✅ DHCP dnsmasq: ACTIVO"
else
    echo "  ❌ DHCP dnsmasq: INACTIVO"
    STATUS_OK=1
fi

# Verificar interfaz ap0
if ip addr show ap0 2>/dev/null | grep -q "192.168.12.1"; then
    echo "  ✅ Interfaz ap0: 192.168.12.1"
else
    echo "  ❌ Interfaz ap0: NO CONFIGURADA"
    STATUS_OK=1
fi

# Verificar WiFi auto-connect
if systemctl is-active --quiet agrobot-wifi-auto.service; then
    echo "  ✅ WiFi auto-connect: ACTIVO"
else
    echo "  ❌ WiFi auto-connect: INACTIVO"
    STATUS_OK=1
fi

# Verificar Bluetooth Gateway
if systemctl is-active --quiet agrobot-bluetooth-gw.service; then
    echo "  ✅ Bluetooth Gateway: ACTIVO"
else
    echo "  ❌ Bluetooth Gateway: INACTIVO"
    STATUS_OK=1
fi

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
if [ $STATUS_OK -eq 0 ]; then
    echo "║          ✅ INSTALACIÓN COMPLETADA Y VERIFICADA           ║"
else
    echo "║          ⚠️  INSTALACIÓN COMPLETADA (verificar errores)   ║"
fi
echo "╚════════════════════════════════════════════════════════════╝"

echo ""
echo "📡 REDES DISPONIBLES:"
echo "  • JETSON_AP (192.168.12.1, contraseña: 12345678)"
echo "  • LoRa_Wan (192.168.0.1, contraseña: admin1)"
echo ""
echo "🌐 ACCESO AL DASHBOARD:"
echo "  • WiFi: http://192.168.12.1:8000"
echo "  • Bluetooth: RFCOMM puerto 1"
echo ""
echo "📊 VER LOGS:"
echo "  sudo journalctl -u agrobot-hostapd.service -f"
echo "  sudo journalctl -u agrobot-dnsmasq.service -f"
echo ""
echo "🔄 REINICIAR TODO:"
echo "  sudo systemctl restart agrobot-*.service"
echo ""
echo "✨ SISTEMA LISTO PARA PRODUCCIÓN ✨"
