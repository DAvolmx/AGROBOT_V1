#!/bin/bash
# DIAGNOSTICO EMERGENCIA — Por qué el robot no se mueve
# Ejecuta: bash DIAGNOSTICO_EMERGENCIA.sh

echo "🚨 DIAGNOSTICO EMERGENCIA — ROBOT NO SE MUEVE"
echo "=================================================="
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 1. Verifica comunicación Jetson ↔ C3
echo -e "${BLUE}[1/6]${NC} Verificando comunicación Jetson ↔ C3..."
echo ""

# BLE
echo -e "${YELLOW}  • Buscando BLE...${NC}"
hcitool lescan 2>/dev/null | grep -i "agrobot" && echo -e "${GREEN}    ✅ BLE encontrado${NC}" || echo -e "${RED}    ❌ BLE NO encontrado${NC}"
echo ""

# WiFi
echo -e "${YELLOW}  • Buscando WiFi C3...${NC}"
ping -c 1 192.168.12.10 2>/dev/null && echo -e "${GREEN}    ✅ WiFi OK${NC}" || echo -e "${RED}    ❌ WiFi NO responde${NC}"
echo ""

# 2. Verifica puerto serial (si está conectado USB)
echo -e "${BLUE}[2/6]${NC} Verificando puerto serial..."
ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null && echo -e "${GREEN}✅ Puerto encontrado${NC}" || echo -e "${RED}❌ Sin puerto serial${NC}"
echo ""

# 3. Verifica que el firmware esté flasheado
echo -e "${BLUE}[3/6]${NC} Verificando firmware v3..."
if [ -f "$HOME/Desktop/AGROBOT/agrobot_system_2/firmware/esp32c3_actuador_v3_skidsteer.ino" ]; then
    echo -e "${GREEN}✅ Firmware v3 existe${NC}"
    echo "   Pero necesitas flashearlo en Arduino IDE:"
    echo "   1. Abre Arduino IDE"
    echo "   2. File → Open → $HOME/Desktop/AGROBOT/agrobot_system_2/firmware/esp32c3_actuador_v3_skidsteer.ino"
    echo "   3. Tools → Board: ESP32C3 Dev Module"
    echo "   4. Tools → Port: /dev/ttyACM0"
    echo "   5. Sketch → Upload"
else
    echo -e "${RED}❌ Firmware v3 NO existe${NC}"
fi
echo ""

# 4. Verifica backend
echo -e "${BLUE}[4/6]${NC} Verificando backend..."
ps aux | grep -E "jetson_server|python.*jetson" | grep -v grep
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Backend corriendo${NC}"
else
    echo -e "${RED}❌ Backend NO está corriendo${NC}"
    echo "   Inicia con:"
    echo "   cd $HOME/Desktop/AGROBOT/agrobot_system_2/jetson/backend"
    echo "   python3 jetson_server.py"
fi
echo ""

# 5. Verifica pines de motor en C3
echo -e "${BLUE}[5/6]${NC} Verificando pines GPIO C3..."
echo "   Pines esperados:"
echo "   • R1_FI: GPIO0   (Forward)"
echo "   • R1_BI: GPIO1   (Backward)"
echo "   • L1_FI: GPIO3   (Forward)"
echo "   • L1_BI: GPIO2   (Backward)"
echo "   • L2_FI: GPIO4   (Forward)"
echo "   • L2_BI: GPIO5   (Backward)"
echo "   • R2_FI: GPIO6   (Forward)"
echo "   • R2_BI: GPIO7   (Backward)"
echo ""
echo -e "${YELLOW}   Conecta USB y lee Serial Monitor:${NC}"
echo "   picocom /dev/ttyACM0 -b 115200"
echo ""

# 6. Checklist rápido
echo -e "${BLUE}[6/6]${NC} CHECKLIST RÁPIDO:"
echo ""

# Verifica config
if [ -f "$HOME/Desktop/AGROBOT/agrobot_system_2/jetson/backend/agrobot_config.json" ]; then
    echo -e "${GREEN}✅${NC} Config JSON presente"
else
    echo -e "${RED}❌${NC} Config JSON FALTA"
fi

# Verifica SLAM v2
if [ -f "$HOME/Desktop/AGROBOT/agrobot_system_2/jetson/backend/slam_v2.py" ]; then
    echo -e "${GREEN}✅${NC} SLAM v2 presente"
else
    echo -e "${RED}❌${NC} SLAM v2 FALTA"
fi

# Verifica path detection
if [ -f "$HOME/Desktop/AGROBOT/agrobot_system_2/jetson/backend/path_detection.py" ]; then
    echo -e "${GREEN}✅${NC} Path detection presente"
else
    echo -e "${RED}❌${NC} Path detection FALTA"
fi

echo ""
echo "=================================================="
echo ""
echo -e "${YELLOW}🔧 PASOS PARA REPARAR:${NC}"
echo ""
echo "1️⃣  PRIMERO: Flashea firmware v3"
echo "   Arduino IDE → esp32c3_actuador_v3_skidsteer.ino → Upload"
echo ""
echo "2️⃣  SEGUNDO: Verifica comunicación"
echo "   Abre Serial Monitor en Arduino IDE"
echo "   Deberías ver: === AGROBOT C3 v3 — SKID-STEER + PWM PURO ==="
echo ""
echo "3️⃣  TERCERO: Arranca backend"
echo "   cd $HOME/Desktop/AGROBOT/agrobot_system_2/jetson/backend"
echo "   python3 jetson_server.py"
echo ""
echo "4️⃣  CUARTO: Abre dashboard"
echo "   http://192.168.12.1:8080"
echo ""
echo "5️⃣  QUINTO: Manual en dashboard"
echo "   Botón ADELANTE → Robot debe avanzar"
echo "   Botón GIRO → Debe girar SIN cruzarse"
echo ""
echo "=================================================="
echo ""
echo -e "${RED}¿AÚN NO SE MUEVE?${NC} Consulta:"
echo "  • ARRANCA_AQUI.md → Troubleshooting"
echo "  • AUDITORIA_COMPLETA_v3.md → Diagnóstico detallado"
echo ""
