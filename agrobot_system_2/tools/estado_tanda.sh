#!/bin/bash
# Resumen del flujo de la tanda de 10 h (lo usa el loop de revisión cada 30 min).
A=~/.agrobot/aprendizaje
echo "hora: $(date '+%F %T') · fase: $(cat $A/tanda_10h.estado 2>/dev/null)"
echo "orquestador vivo: $(pgrep -f '[s]esion_tanda_10h.sh' | wc -l) · procesos de entrenamiento: $(pgrep -f '[e]ntrenar_|[p]ruebas_geometria|[g]enerar_dataset_slam|[c]alibrar_pulsos' | wc -l)"
echo "--- registro"; grep -aE "FASE|INICIO|FIN|sin tiempo|STOP" $A/tanda_10h.log | tail -6 | cut -c1-170
echo "--- progreso del trabajo actual"; tail -n 4 $A/tanda_10h.log | grep -av "^\s*$" | cut -c1-230
echo "--- recursos"; free -m | sed -n 2p; uptime
cd /home/jetson/Desktop/AGROBOT/agrobot_system_2 && python3 tools/diagrama_flujo_tanda.py 2>/dev/null | grep -a "^  \[\|diagrama"
