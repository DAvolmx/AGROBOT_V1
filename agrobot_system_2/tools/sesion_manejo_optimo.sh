#!/bin/bash
# Sesión «modelo óptimo de manejo» (2026-10-20), simulador con la geometría corregida y el firmware «maxfrente».
# Cada episodio cuesta ~31 s de CPU: la sesión está dimensionada para ~3,5 h con los 6 núcleos.
#   FASE 1  entrenar_robusto.py   embudo (filtro -> validación -> prueba con semillas nuevas + estrés + repetición),
#                                 anchos de pasillo 50 y 75 cm, hardware sorteado; control continuo (filtro YA calculado:
#                                 se reanuda) y pulsos (filtro nuevo) -> modelo_manejo_optimo_<fecha>.json
#   FASE 2  entrenar_fallas.py    15 fallas + choques + paradas para escanear + manejo ASISTIDO que enseña lecciones;
#                                 repite las MISMAS pruebas; mide con semillas nuevas
# NO despliega nada.
#   setsid nohup bash tools/sesion_manejo_optimo.sh > ~/.agrobot/aprendizaje/sesion_manejo_optimo.log 2>&1 &
#   tail -f ~/.agrobot/aprendizaje/sesion_manejo_optimo.log
set -u
RAIZ="$(cd "$(dirname "$0")/.." && pwd)"
cd "$RAIZ/simulador_pasivo" || exit 1
echo 1000 > /proc/self/oom_score_adj 2>/dev/null
echo "[$(date '+%F %T')] === SESIÓN MANEJO ÓPTIMO (v5: giro calibrado con el toque real (10°)) ==="
nice -n -10 python3 -u entrenar_robusto.py --control ambos --procs 6 --n 24 --horas 3.0 --huecos 0.50,0.75 \
    --sem-filtro 2 --sem-val 3 --sem-test 5 --top 6 --finalistas 3
echo "[$(date '+%F %T')] === FASE 1 TERMINADA (código $?) ==="
nice -n -10 python3 -u entrenar_fallas.py --config optimo --procs 6 --rondas 3 --semillas 2
echo "[$(date '+%F %T')] === FASE 2 TERMINADA (código $?) ==="
echo "[$(date '+%F %T')] === SESIÓN TERMINADA ==="
