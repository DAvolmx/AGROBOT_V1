#!/bin/bash
# 🚀 AGROBOT v2.0 — INSTALACIÓN (Versión simplificada)
# Solo copia archivos (asume que las dependencias ya están instaladas)

set -e

echo "╔════════════════════════════════════════════════════════════╗"
echo "║          🤖 AGROBOT v2.0 — INSTALACIÓN RÁPIDA             ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Verificar si es root
if [[ $EUID -ne 0 ]]; then
   echo "❌ Este script DEBE ejecutarse como root"
   echo "Ejecuta: sudo bash INSTALAR_AGROBOT_SIMPLE.sh"
   exit 1
fi

REPO_DIR="/home/jetson/Desktop/AGROBOT"
JETSON_SYSTEMD="$REPO_DIR/agrobot_system_2/jetson/systemd"

echo "⚙️  Copiando configuración a /etc..."

# Hostapd
mkdir -p /etc/hostapd
cp "$JETSON_SYSTEMD/etc/hostapd/agrobot.conf" /etc/hostapd/agrobot.conf
echo "✅ Hostapd configurado"

# Dnsmasq
mkdir -p /etc/dnsmasq.d
cp "$JETSON_SYSTEMD/etc/dnsmasq.d/agrobot-ap.conf" /etc/dnsmasq.d/agrobot-ap.conf
echo "✅ DNSMASQ configurado"

echo ""
echo "🔧 Instalando servicios systemd..."

mkdir -p /etc/systemd/system

# Copiar servicios
cp "$JETSON_SYSTEMD/agrobot-ap0.service" /etc/systemd/system/
cp "$JETSON_SYSTEMD/agrobot-hostapd.service" /etc/systemd/system/
cp "$JETSON_SYSTEMD/agrobot-dnsmasq.service" /etc/systemd/system/
cp "$JETSON_SYSTEMD/agrobot-wifi-auto.service" /etc/systemd/system/
cp "$JETSON_SYSTEMD/agrobot-bluetooth-gw.service" /etc/systemd/system/

systemctl daemon-reload
echo "✅ Servicios instalados"

echo ""
echo "🚀 Habilitando servicios en boot..."

systemctl enable agrobot-ap0.service
systemctl enable agrobot-hostapd.service
systemctl enable agrobot-dnsmasq.service
systemctl enable agrobot-wifi-auto.service
systemctl enable agrobot-bluetooth-gw.service

echo "✅ Servicios habilitados"

echo ""
echo "⚡ Iniciando servicios..."

systemctl restart agrobot-ap0.service
sleep 2
systemctl restart agrobot-hostapd.service
sleep 2
systemctl restart agrobot-dnsmasq.service
sleep 2
systemctl restart agrobot-wifi-auto.service
sleep 2
systemctl restart agrobot-bluetooth-gw.service

echo "✅ Servicios iniciados"

echo ""
echo "✔️  Verificando instalación..."
echo ""

# Verificar AP
if systemctl is-active --quiet agrobot-hostapd.service; then
    echo "  ✅ Hostapd ACTIVO"
else
    echo "  ❌ Hostapd INACTIVO"
fi

# Verificar DHCP
if systemctl is-active --quiet agrobot-dnsmasq.service; then
    echo "  ✅ DNSMASQ ACTIVO"
else
    echo "  ❌ DNSMASQ INACTIVO"
fi

# Verificar interfaz ap0
if ip addr show ap0 2>/dev/null | grep -q "192.168.12.1"; then
    echo "  ✅ Interfaz ap0 CONFIGURADA (192.168.12.1)"
else
    echo "  ❌ Interfaz ap0 NO CONFIGURADA"
fi

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║          ✅ INSTALACIÓN COMPLETADA                        ║"
echo "╚════════════════════════════════════════════════════════════╝"

echo ""
echo "📡 REDES DISPONIBLES:"
echo "  • JETSON_AP (192.168.12.1, contraseña: 12345678)"
echo ""
echo "🌐 DASHBOARD:"
echo "  • http://192.168.12.1:8000"
echo ""
echo "📊 VER LOGS:"
echo "  sudo journalctl -u agrobot-hostapd.service -f"
echo ""
