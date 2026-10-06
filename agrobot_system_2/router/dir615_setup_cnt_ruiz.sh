#!/bin/bash
# 🌐 DIR-615 — Configuración para conectarse a CNT-RUIZ
#
# OBJETIVO: El router DIR-615 actúa como cliente de CNT-RUIZ (internet)
# y distribuye conectividad a toda la red LoRa_Wan.
#
# MODO OPERATIVO:
#   • AP: LoRa_Wan (192.168.0.1)
#   • Cliente: CNT-RUIZ (internet externo)
#   • DHCP: Sirve IPs 192.168.0.x a dispositivos conectados
#
# EJECUCIÓN: bash dir615_setup_cnt_ruiz.sh desde router o via SSH admin/admin1

echo "╔════════════════════════════════════════════════════════════╗"
echo "║   DIR-615 — Configuración CNT-RUIZ (Cliente)              ║"
echo "╚════════════════════════════════════════════════════════════╝"

# Variables
ROUTER_IP="192.168.0.1"
ROUTER_USER="admin"
ROUTER_PASS="admin1"
CNT_RUIZ_SSID="CNT-RUIZ"
CNT_RUIZ_PASS="1850370113"

echo ""
echo "⚠️  INSTRUCCIONES MANUALES (si no tienes SSH configurado):"
echo ""
echo "1. Accede al router en navegador:"
echo "   http://192.168.0.1"
echo ""
echo "2. Usuario: admin"
echo "   Contraseña: admin1"
echo ""
echo "3. Menú: Wireless → WiFi Settings"
echo ""
echo "4. Configura:"
echo "   - SSID: LoRa_Wan (para AP local)"
echo "   - Channel: 1 (fijo)"
echo "   - Security: WPA2"
echo "   - Password: admin1"
echo ""
echo "5. Menú: Wireless → Site Survey / Scan"
echo "   - Busca: CNT-RUIZ"
echo "   - Conecta como CLIENTE (no AP)"
echo ""
echo "6. Ingresa contraseña: 1850370113"
echo ""
echo "7. Guarda y reinicia"
echo ""
echo "✅ El router ahora:"
echo "   • AP: LoRa_Wan (para robots locales)"
echo "   • Cliente: CNT-RUIZ (para internet)"
echo "   • DHCP: Distribuye IPs 192.168.0.x"
echo ""
echo "📡 Red final:"
echo "   Internet → CNT-RUIZ → DIR-615 (Router) → Jetson + Robots"
echo ""

# Si tienes SSH activado en el router, descomenta y modifica:
# echo "Intentando via SSH..."
# sshpass -p "$ROUTER_PASS" ssh -o StrictHostKeyChecking=no \
#   $ROUTER_USER@$ROUTER_IP \
#   "nvram set wl0_ssid='LoRa_Wan' && \
#    nvram set wl0_channel='1' && \
#    nvram set wl0_auth_mode='open' && \
#    nvram set wl0_wpa_psk='admin1' && \
#    nvram commit && \
#    reboot"

echo ""
echo "🚀 Una vez configurado, verifica:"
echo "   • Red LoRa_Wan disponible"
echo "   • Conexión a CNT-RUIZ activa"
echo "   • IPs 192.168.0.x asignadas a clientes"
