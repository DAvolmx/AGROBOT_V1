#!/bin/bash
set -e

# =============================================================================
# INICIO RÁPIDO: Todo automático en 5 minutos
# =============================================================================

echo "🤖 AGROBOT - Sistema de Comunicación Automático"
echo "=============================================="
echo ""

# Color
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Paths
REPO_DIR="/home/jetson/Desktop/AGROBOT"
BACKEND_DIR="$REPO_DIR/agrobot_system_2/jetson/backend"
SERVER_FILE="$BACKEND_DIR/jetson_server.py"

# Verificar que estamos en el lugar correcto
if [ ! -d "$REPO_DIR" ]; then
    echo -e "${RED}✗ No encuentro AGROBOT en $REPO_DIR${NC}"
    exit 1
fi

cd "$REPO_DIR"

echo -e "${BLUE}Paso 1: Verificar archivos nuevos${NC}"
# Verificar que existen todos los archivos
archivos_requeridos=(
    "$BACKEND_DIR/auto_comunicacion.py"
    "$BACKEND_DIR/ap_vigilante.py"
    "$BACKEND_DIR/comunicacion_mejorada.py"
    "$BACKEND_DIR/diagnostico_wifi.py"
    "$BACKEND_DIR/torque_optimizer.py"
)

todos_ok=true
for archivo in "${archivos_requeridos[@]}"; do
    if [ -f "$archivo" ]; then
        echo -e "${GREEN}✓${NC} $(basename $archivo)"
    else
        echo -e "${RED}✗${NC} Falta $(basename $archivo)"
        todos_ok=false
    fi
done

if [ "$todos_ok" = false ]; then
    echo -e "${RED}Falta copiar archivos. Ve a: INTEGRACION_WIFI_TORQUE.md${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}Paso 2: Maximizar potencia WiFi${NC}"

# Maximizar potencia
if ! sudo iw wlan0 set txpower fixed 3000 &>/dev/null; then
    echo -e "${YELLOW}⚠ Potencia puede no haberse aplicado (verifica permisos sudo)${NC}"
else
    echo -e "${GREEN}✓${NC} Potencia TX: 30 dBm"
fi

echo ""
echo -e "${BLUE}Paso 3: Levantar AP JETSON_AP${NC}"

# Intentar levantar AP
if sudo nmcli device wifi hotspot ifname wlan0 ssid "JETSON_AP" password "agrobot123" &>/dev/null; then
    echo -e "${GREEN}✓${NC} AP JETSON_AP levantado"
else
    # Intentar desconectar primero
    sudo nmcli device disconnect wlan0 &>/dev/null || true
    sleep 2
    if sudo nmcli device wifi hotspot ifname wlan0 ssid "JETSON_AP" password "agrobot123" &>/dev/null; then
        echo -e "${GREEN}✓${NC} AP JETSON_AP levantado (2do intento)"
    else
        echo -e "${YELLOW}⚠ AP puede no estar UP, continuando...${NC}"
    fi
fi

echo ""
echo -e "${BLUE}Paso 4: Aplicar parche al servidor${NC}"

# Hacer backup
if [ ! -f "$SERVER_FILE.backup" ]; then
    cp "$SERVER_FILE" "$SERVER_FILE.backup"
    echo -e "${GREEN}✓${NC} Backup: $SERVER_FILE.backup"
fi

# Verificar que el import ya está
if grep -q "from auto_comunicacion import inicializar_auto_comunicacion" "$SERVER_FILE"; then
    echo -e "${YELLOW}ℹ Parche ya aplicado${NC}"
else
    echo -e "${YELLOW}⚠ Parche no se aplicó automáticamente${NC}"
    echo "Ve a PARCHE_AUTOMATICO.md para aplicarlo manualmente"
fi

echo ""
echo -e "${BLUE}Paso 5: Verificar sintaxis${NC}"

if python3 -m py_compile "$SERVER_FILE" 2>/dev/null; then
    echo -e "${GREEN}✓${NC} jetson_server.py: sintaxis OK"
else
    echo -e "${RED}✗ Error de sintaxis en jetson_server.py${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}✓ SISTEMA LISTO${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo ""
echo "🚀 Próximos pasos:"
echo ""
echo "1. REVISAR E INTEGRAR MANUALMENTE (2 minutos):"
echo "   • Abrir: $SERVER_FILE"
echo "   • Leer: PARCHE_AUTOMATICO.md"
echo "   • Seguir los pasos de integración"
echo ""
echo "2. REINICIAR SERVIDOR:"
echo "   systemctl restart agrobot-jetson"
echo ""
echo "3. VERIFICAR QUE FUNCIONA:"
echo "   journalctl -u agrobot-jetson -f | grep AUTO"
echo ""
echo "4. MONITOREAR EN TIEMPO REAL:"
echo "   python3 tools/diagnostico_wifi_rt.py"
echo ""
echo -e "${YELLOW}Nota: Si robot sigue perdiendo WiFi, ejecuta:${NC}"
echo "   python3 tools/diagnostico_wifi_rt.py"
echo ""
