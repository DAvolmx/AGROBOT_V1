#!/bin/bash
# Resumen rápido de la sesión de 10 h (lo usa el loop de revisión cada 30 min).
A=~/.agrobot/aprendizaje
echo "hora: $(date '+%F %T') · fase: $(cat $A/sesion_10h.estado 2>/dev/null)"
echo "orquestador vivo: $(pgrep -f '[s]esion_10h.sh' | wc -l) · entrenamientos: $(pgrep -f '[e]ntrenar_|[p]ruebas_geometria' | wc -l)"
echo "--- sesion_10h.log"; tail -n 6 $A/sesion_10h.log | cut -c1-220
echo "--- manejo v5"; tail -n 5 $A/sesion_manejo_optimo.log | cut -c1-300
echo "--- recursos"; free -m | sed -n 2p; uptime; cat /sys/devices/virtual/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -1
echo "--- informes"; ls -t ~/.agrobot/informes | head -3
