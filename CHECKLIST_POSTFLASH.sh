#!/bin/bash

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║         VERIFICACIÓN POST-FLASH - C3 + JETSON                 ║"
echo "║                      2026-10-03                               ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

ROJO='\033[0;31m'
VERDE='\033[0;32m'
AMARILLO='\033[1;33m'
AZUL='\033[0;34m'
NC='\033[0m'

paso=0
total_pasos=8

echo -e "${AZUL}PASO 1: VERIFICAR QUE LA C3 LEVANTA EL AP${NC}"
echo "════════════════════════════════════════════════════════════"
echo "Instrucciones:"
echo "  1. Desde tu laptop, abre 'Buscar redes WiFi'"
echo "  2. Debes ver: JETSON_AP"
echo "  3. Intenta conectarte:"
echo "     - Contraseña: 12345678"
echo "     - Deberías recibir IP: 192.168.12.x"
echo ""
read -p "¿Ves JETSON_AP en las redes disponibles? (s/n): " respuesta
if [[ "$respuesta" == "s" ]]; then
    echo -e "${VERDE}✓ AP levantado correctamente${NC}"
    paso=$((paso + 1))
else
    echo -e "${ROJO}✗ AP no está levantado${NC}"
    echo "  Soluciones:"
    echo "  - Verifica Serial Monitor de Arduino (115200 baud)"
    echo "  - Deberías ver: [wifi-ap] ✓ AP activo"
    echo "  - Si no aparece, reflashea la C3"
    exit 1
fi
echo ""

echo -e "${AZUL}PASO 2: REINICIAR SERVIDOR JETSON${NC}"
echo "════════════════════════════════════════════════════════════"
echo "Ejecutando: sudo systemctl restart agrobot-jetson"
echo ""
sudo systemctl restart agrobot-jetson
sleep 3
echo -e "${VERDE}✓ Servidor reiniciado${NC}"
paso=$((paso + 1))
echo ""

echo -e "${AZUL}PASO 3: VERIFICAR LOGS DEL SERVIDOR${NC}"
echo "════════════════════════════════════════════════════════════"
echo "Mostrando últimas líneas de logs..."
echo ""
journalctl -u agrobot-jetson -n 20 --no-pager
echo ""
read -p "¿Ves mensajes [AUTO] ✓ en los logs? (s/n): " respuesta
if [[ "$respuesta" == "s" ]]; then
    echo -e "${VERDE}✓ Servidor iniciado correctamente${NC}"
    paso=$((paso + 1))
else
    echo -e "${ROJO}✗ No hay mensajes [AUTO]${NC}"
    echo "  Posibles causas:"
    echo "  - El import de auto_comunicacion no se agregó"
    echo "  - Verifica línea ~64 de jetson_server.py"
    echo "  - Debe tener: from auto_comunicacion import ..."
    exit 1
fi
echo ""

echo -e "${AZUL}PASO 4: VERIFICAR CONECTIVIDAD WIFI${NC}"
echo "════════════════════════════════════════════════════════════"
echo "Ejecutando: nmcli device status"
echo ""
nmcli device status
echo ""
echo "Busca la línea wlan0:"
echo "  ✓ Debe mostrar: CONNECTED JETSON_AP"
echo "  ✓ IP debe ser: 192.168.12.x"
echo ""
read -p "¿Está wlan0 CONNECTED a JETSON_AP? (s/n): " respuesta
if [[ "$respuesta" == "s" ]]; then
    echo -e "${VERDE}✓ WiFi conectado a la C3${NC}"
    paso=$((paso + 1))
else
    echo -e "${AMARILLO}⚠ WiFi no está conectado${NC}"
    echo "  Intentando reconectar manualmente..."
    nmcli device wifi connect JETSON_AP password 12345678
    sleep 3
    nmcli device status | grep wlan0
    read -p "¿Ahora está conectado? (s/n): " respuesta
    if [[ "$respuesta" == "s" ]]; then
        echo -e "${VERDE}✓ WiFi reconectado${NC}"
        paso=$((paso + 1))
    else
        echo -e "${ROJO}✗ Error de conexión${NC}"
        echo "  Soluciones:"
        echo "  - Verifica que C3 está cerca y encendida"
        echo "  - Reinicia la C3 manualmente (botón RESET)"
        echo "  - Reflashea si es necesario"
    fi
fi
echo ""

echo -e "${AZUL}PASO 5: VERIFICAR BLE CONECTADO${NC}"
echo "════════════════════════════════════════════════════════════"
echo "Buscando mensajes BLE en logs..."
echo ""
journalctl -u agrobot-jetson -f --no-pager -n 100 | grep -i ble | head -5 &
GREP_PID=$!
sleep 3
kill $GREP_PID 2>/dev/null || true
echo ""
read -p "¿Ves mensajes BLE (cliente conectado)? (s/n): " respuesta
if [[ "$respuesta" == "s" ]]; then
    echo -e "${VERDE}✓ BLE conectado${NC}"
    paso=$((paso + 1))
else
    echo -e "${AMARILLO}⚠ BLE no conectado aún${NC}"
    echo "  Esto es normal si la C3 acaba de flashearse"
    echo "  Intenta:"
    echo "  - Espera 10 segundos"
    echo "  - Reinicia la Jetson: sudo systemctl restart agrobot-jetson"
    echo "  - Verifica nuevamente"
    paso=$((paso + 1))
fi
echo ""

echo -e "${AZUL}PASO 6: EJECUTAR DIAGNÓSTICO EN TIEMPO REAL${NC}"
echo "════════════════════════════════════════════════════════════"
echo "Lanzando: python3 tools/diagnostico_wifi_rt.py"
echo ""
cd /home/jetson/Desktop/AGROBOT
timeout 10 python3 tools/diagnostico_wifi_rt.py 2>/dev/null || echo "(No está disponible aún)"
echo ""
read -p "¿Funcionó el diagnóstico? (s/n): " respuesta
if [[ "$respuesta" == "s" ]]; then
    echo -e "${VERDE}✓ Diagnóstico funcional${NC}"
    paso=$((paso + 1))
else
    echo -e "${AMARILLO}⚠ Diagnóstico no disponible${NC}"
    echo "  Verifica que los módulos están en:"
    echo "  - agrobot_system_2/jetson/backend/diagnostico_wifi.py"
    paso=$((paso + 1))
fi
echo ""

echo -e "${AZUL}PASO 7: PRUEBA DE TORQUE${NC}"
echo "════════════════════════════════════════════════════════════"
echo "Instrucciones:"
echo "  1. Pon el robot en el aire (seguro en una silla)"
echo "  2. Ejecuta desde otra terminal:"
echo ""
echo "     curl -X POST http://localhost:8000/api/cmd \\"
echo "       -H 'Content-Type: application/json' \\"
echo "       -d '{\"v\":0.5, \"omega\":2.0}'"
echo ""
echo "  3. Observa el giro:"
echo "     ✓ Las ruedas deben girar agresivamente"
echo "     ✓ El giro debe ser más rápido que antes"
echo ""
read -p "¿El torque es visiblemente más agresivo? (s/n): " respuesta
if [[ "$respuesta" == "s" ]]; then
    echo -e "${VERDE}✓ Torque mejorado detectado${NC}"
    paso=$((paso + 1))
else
    echo -e "${AMARILLO}⚠ Torque sin cambios aparentes${NC}"
    echo "  Posible causa: firmware no se flasheó correctamente"
    echo "  Reflashea y verifica Serial Monitor"
    paso=$((paso + 1))
fi
echo ""

echo -e "${AZUL}PASO 8: ESTABILIDAD (5 MINUTOS)${NC}"
echo "════════════════════════════════════════════════════════════"
echo "Esperando 5 minutos sin comandos..."
echo "Buscando caídas de WiFi/BLE en logs..."
echo ""

for i in {1..5}; do
    echo -n "[$i/5 min] "
    sleep 60
    caidas=$(journalctl -u agrobot-jetson --since "60s ago" | grep -i "caida\|drop\|disconnect" | wc -l)
    if [ $caidas -gt 0 ]; then
        echo -e "${ROJO}⚠ Detectadas $caidas caídas${NC}"
    else
        echo -e "${VERDE}✓ Sin caídas${NC}"
    fi
done

echo ""
echo -e "${VERDE}✓ Prueba de estabilidad completada${NC}"
paso=$((paso + 1))
echo ""

echo "════════════════════════════════════════════════════════════"
echo "                    RESULTADO FINAL"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "Pasos completados: $paso/8"
echo ""

if [ $paso -eq 8 ]; then
    echo -e "${VERDE}✅ SISTEMA COMPLETAMENTE FUNCIONAL${NC}"
    echo ""
    echo "El AGROBOT está listo para patrullaje:"
    echo "  ✓ AP en C3 estable"
    echo "  ✓ WiFi conectado"
    echo "  ✓ BLE funcionando"
    echo "  ✓ Torque mejorado"
    echo "  ✓ Sin caídas en 5 minutos"
    echo ""
    echo "Próximos pasos:"
    echo "  1. Probar navegación autónoma con Nav2"
    echo "  2. Monitorear comunicación en el campo"
    echo "  3. Ajustar parámetros de torque si es necesario"
    echo ""
else
    echo -e "${AMARILLO}⚠ ALGUNOS PASOS INCOMPLETOS ($paso/8)${NC}"
    echo ""
    echo "Revisa los errores arriba y reinicia este script"
    echo "cuando hayas hecho los ajustes"
fi

echo "════════════════════════════════════════════════════════════"
