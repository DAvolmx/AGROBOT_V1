#!/bin/bash

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║    VERIFICACIÓN DE CAMBIOS - ARQUITECTURA C3-AP + JETSON      ║"
echo "║                      2026-10-03                               ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

ROJO='\033[0;31m'
VERDE='\033[0;32m'
AMARILLO='\033[1;33m'
NC='\033[0m' # No Color

# Contador de verificaciones
TOTAL=0
PASS=0
FAIL=0

verificar() {
    local desc="$1"
    local cmd="$2"

    TOTAL=$((TOTAL + 1))
    echo -n "[$TOTAL] $desc... "

    if eval "$cmd" > /dev/null 2>&1; then
        echo -e "${VERDE}✓${NC}"
        PASS=$((PASS + 1))
    else
        echo -e "${ROJO}✗${NC}"
        FAIL=$((FAIL + 1))
    fi
}

echo "📁 VERIFICACIÓN DE ARCHIVOS"
echo "────────────────────────────────────────────────────────────"

verificar "Firmware C3 existe" "[ -f '/tmp/AGROBOT_C3_FIRMWARE_MEJORADO.ino' ]"
verificar "WiFi Manager existe" "[ -f 'agrobot_system_2/jetson/backend/wifi_manager.py' ]"
verificar "Auto-Comunicación existe" "[ -f 'agrobot_system_2/jetson/backend/auto_comunicacion.py' ]"
verificar "AP Vigilante existe" "[ -f 'agrobot_system_2/jetson/backend/ap_vigilante.py' ]"
verificar "Servidor Jetson existe" "[ -f 'agrobot_system_2/jetson/backend/jetson_server.py' ]"

echo ""
echo "🔍 VERIFICACIÓN DE CÓDIGO"
echo "────────────────────────────────────────────────────────────"

verificar "WiFi Manager: usa c3_ap_ssid" "grep -q 'self.c3_ap_ssid' agrobot_system_2/jetson/backend/wifi_manager.py"
verificar "WiFi Manager: conecta a AP" "grep -q 'device.*wifi.*connect.*self.c3_ap_ssid' agrobot_system_2/jetson/backend/wifi_manager.py"
verificar "Servidor importa auto_comunicacion" "grep -q 'from auto_comunicacion import' agrobot_system_2/jetson/backend/jetson_server.py"
verificar "Servidor inicia auto_comunicacion" "grep -q 'await inicializar_auto_comunicacion' agrobot_system_2/jetson/backend/jetson_server.py"
verificar "Servidor actualiza estado vías" "grep -q 'actualizar_estado_vias' agrobot_system_2/jetson/backend/jetson_server.py"

echo ""
echo "⚙️  VERIFICACIÓN DE SINTAXIS PYTHON"
echo "────────────────────────────────────────────────────────────"

verificar "jetson_server.py sintaxis" "python3 -m py_compile agrobot_system_2/jetson/backend/jetson_server.py"
verificar "wifi_manager.py sintaxis" "python3 -m py_compile agrobot_system_2/jetson/backend/wifi_manager.py"
verificar "auto_comunicacion.py sintaxis" "python3 -m py_compile agrobot_system_2/jetson/backend/auto_comunicacion.py"

echo ""
echo "💾 VERIFICACIÓN DE FIRMWARE C3"
echo "────────────────────────────────────────────────────────────"

verificar "Firmware contiene ENLACE_WIFI=0" "grep -q '#define ENLACE_WIFI.*0' /tmp/AGROBOT_C3_FIRMWARE_MEJORADO.ino"
verificar "Firmware contiene C3_LEVANTA_AP=1" "grep -q '#define C3_LEVANTA_AP.*1' /tmp/AGROBOT_C3_FIRMWARE_MEJORADO.ino"
verificar "Firmware PWM_MIN=215" "grep -q 'PWM_MIN.*=.*215' /tmp/AGROBOT_C3_FIRMWARE_MEJORADO.ino"
verificar "Firmware KICK_PULSOS=5" "grep -q 'KICK_PULSOS.*=.*5' /tmp/AGROBOT_C3_FIRMWARE_MEJORADO.ino"
verificar "Firmware levanta AP JETSON_AP" "grep -q 'JETSON_AP' /tmp/AGROBOT_C3_FIRMWARE_MEJORADO.ino"

echo ""
echo "════════════════════════════════════════════════════════════"
echo "                        RESULTADO FINAL"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "Total verificaciones: $TOTAL"
echo -e "Pasadas: ${VERDE}$PASS${NC}"
echo -e "Fallidas: ${ROJO}$FAIL${NC}"
echo ""

if [ $FAIL -eq 0 ]; then
    echo -e "${VERDE}✅ TODOS LOS CAMBIOS ESTÁN CORRECTOS${NC}"
    echo ""
    echo "Próximos pasos:"
    echo "1. Flashea el firmware en Arduino IDE:"
    echo "   Archivo: /tmp/AGROBOT_C3_FIRMWARE_MEJORADO.ino"
    echo ""
    echo "2. Una vez flasheado, reinicia el servidor Jetson:"
    echo "   sudo systemctl restart agrobot-jetson"
    echo ""
    echo "3. Verifica los logs:"
    echo "   journalctl -u agrobot-jetson -f | grep -E '[AUTO]|[wifi]'"
    echo ""
    exit 0
else
    echo -e "${ROJO}❌ ALGUNAS VERIFICACIONES FALLARON${NC}"
    echo ""
    echo "Por favor, revisa:"
    for desc in "$@"; do
        echo "  - $desc"
    done
    exit 1
fi
