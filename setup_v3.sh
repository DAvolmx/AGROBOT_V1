#!/bin/bash
# Setup Script — AGROBOT v3 Completo
# =================================
# Instala dependencias e integra todo el sistema

set -e

echo "🚀 AGROBOT v3 — Setup Completo"
echo "================================"

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Directorios
BACKEND="/home/jetson/Desktop/AGROBOT/agrobot_system_2/jetson/backend"
FIRMWARE="/home/jetson/Desktop/AGROBOT/agrobot_system_2/firmware"

echo -e "${YELLOW}[1/5]${NC} Verificando dependencias Python..."

# Dependencias críticas
DEPS=(
    "opencv-contrib-python"
    "numpy"
    "scipy"
    "asyncio"
)

for dep in "${DEPS[@]}"; do
    if python3 -c "import ${dep}" 2>/dev/null; then
        echo -e "${GREEN}  ✅${NC} $dep"
    else
        echo -e "${YELLOW}  ⚠️  Falta $dep (instala manualmente si es necesario)${NC}"
    fi
done

echo -e "${YELLOW}[2/5]${NC} Verificando archivos backend..."

# Archivos necesarios
FILES_BACKEND=(
    "slam_v2.py"
    "camera_zoom.py"
    "path_detection.py"
    "vision_system_v3.py"
    "agrobot_config.json"
)

for file in "${FILES_BACKEND[@]}"; do
    if [ -f "$BACKEND/$file" ]; then
        echo -e "${GREEN}  ✅${NC} $file"
    else
        echo -e "${RED}  ❌ Falta $file${NC}"
        exit 1
    fi
done

echo -e "${YELLOW}[3/5]${NC} Verificando archivo config..."

# Verifica que la config tiene parámetros v3
if grep -q "slam_v2" "$BACKEND/agrobot_config.json"; then
    echo -e "${GREEN}  ✅${NC} Configuración v3 (con slam_v2)"
else
    echo -e "${RED}  ❌ Configuración sin parámetros v3${NC}"
    exit 1
fi

if grep -q "sendero_h_min" "$BACKEND/agrobot_config.json"; then
    echo -e "${GREEN}  ✅${NC} Path detection calibrado"
else
    echo -e "${RED}  ❌ Path detection sin calibración${NC}"
    exit 1
fi

echo -e "${YELLOW}[4/5]${NC} Creando directorios..."

# Directorios necesarios
mkdir -p "$BACKEND/slam_maps"
mkdir -p "$BACKEND/captures"
mkdir -p "$BACKEND/models"
echo -e "${GREEN}  ✅${NC} Directorios listos"

echo -e "${YELLOW}[5/5]${NC} Verificando firmware..."

if [ -f "$FIRMWARE/esp32c3_actuador_v3_skidsteer.ino" ]; then
    echo -e "${GREEN}  ✅${NC} Firmware v3 presente"
else
    echo -e "${YELLOW}  ⚠️  Firmware v3 no encontrado${NC}"
    echo -e "${YELLOW}     Descargalo de: esp32c3_actuador_v3_skidsteer.ino${NC}"
fi

echo ""
echo -e "${GREEN}╔════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  ✨ AGROBOT v3 LISTO PARA USAR ✨    ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════╝${NC}"
echo ""
echo "📋 Próximos pasos:"
echo ""
echo "1️⃣  Flashea firmware en ESP32-C3:"
echo "   Arduino IDE → ${FIRMWARE}/esp32c3_actuador_v3_skidsteer.ino"
echo ""
echo "2️⃣  Arranca backend:"
echo "   cd ${BACKEND}"
echo "   python3 jetson_server.py"
echo ""
echo "3️⃣  Abre dashboard:"
echo "   http://192.168.12.1:8080"
echo ""
echo "🌾 Sistema configurado para:"
echo "   • Sendero: H:10-45, S:20-90, V:30-150"
echo "   • Plagas: Mosca Blanca, Mildiu, Daños"
echo "   • Zoom: Automático 1x/2x/4x"
echo ""
echo -e "${YELLOW}📸 Si necesitas ajustar HSV:${NC}"
echo "   Edita: $BACKEND/agrobot_config.json"
echo "   Secciones: path_detection, plant_detection, damage_detection"
echo ""
