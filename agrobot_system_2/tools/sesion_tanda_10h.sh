#!/bin/bash
# TANDA DE ENTRENAMIENTO DE 10 H con todo CALIBRADO con datos reales (2026-10-22). Un entrenamiento a la vez; no despliega nada.
#   FASE 0  calibración: ruido del robot PARADO (LIDAR/IMU) y variabilidad real de los PULSOS -> simulador calibrado (calibracion_real.py)
#   FASE 1  manejo robusto: embudo filtro -> validación -> prueba, control continuo y pulsos, 3 anchos de pasillo, física sorteada + variabilidad real
#   FASE 2  fallas con manejo asistido (15 fallas, choques, paradas de escaneo) con la config óptima
#   FASE 3  escenarios (recto a tope, sala cuadrada, hileras, zigzag + clásicos) y todas las fallas: tablas y capturas
#   FASE 4  dataset de SLAM con el ruido calibrado + publicación en Kaggle (dataset y notebook, privados)
#   FASE 5  informe final con diagramas
# Parar: touch ~/.agrobot/aprendizaje/tanda_10h.STOP      Seguir: tail -f ~/.agrobot/aprendizaje/tanda_10h.log
set -u
RAIZ="$(cd "$(dirname "$0")/.." && pwd)"
SIM="$RAIZ/simulador_pasivo"; OUT="$HOME/.agrobot/aprendizaje"
STOP="$OUT/tanda_10h.STOP"; ESTADO="$OUT/tanda_10h.estado"; MARCA="$OUT/tanda.inicio"
KG="$HOME/.agrobot/kaggle-cli/bin/kaggle"
FIN_EPOCH=$(( $(date +%s) + 36000 ))
rm -f "$STOP"; touch "$MARCA"
echo 1000 > /proc/self/oom_score_adj 2>/dev/null
log() { echo "[$(date '+%F %T')] $*"; }
fase() { echo "$1" > "$ESTADO"; log "=== FASE $1 ==="; }
quedan_min() { echo $(( (FIN_EPOCH - $(date +%s)) / 60 )); }
paso() {   # paso <min_estimados> <nombre> <cmd...>
  local est="$1" nombre="$2"; shift 2
  [ -e "$STOP" ] && { log "STOP pedido: se salta '$nombre'"; return; }
  if [ "$(quedan_min)" -lt $((est + 10)) ]; then log "sin tiempo para '$nombre' (necesita ~${est} min, quedan $(quedan_min)): se salta"; return; fi
  log "---- INICIO: $nombre (~${est} min) ----"; local t0=$(date +%s)
  ( echo 1000 > /proc/self/oom_score_adj 2>/dev/null; "$@" ); local rc=$?
  log "---- FIN: $nombre · código $rc · $(( ($(date +%s) - t0) / 60 )) min ----"
}
log "=== TANDA 10 H · fin previsto $(date -d @$FIN_EPOCH '+%F %T') ==="
fase "0 calibración con datos reales"
paso 8  "ruido del robot parado" bash -c "cd '$RAIZ' && python3 tools/caracterizar_parado.py --bloques 160 > /dev/null"
paso 10 "variabilidad real de los pulsos" bash -c "cd '$SIM' && python3 calibrar_pulsos.py --semillas 120 --procs 5 | tail -4"
fase "1 manejo robusto (continuo + pulsos)"
paso 260 "búsqueda de manejo robusta" bash -c "cd '$SIM' && nice -n 5 python3 -u entrenar_robusto.py --control ambos --procs 5 --n 36 --horas 4.4 --huecos 0.50,0.60,0.75 --sem-filtro 2 --sem-val 4 --sem-test 6 --top 8 --finalistas 3"
fase "2 fallas con manejo asistido"
paso 110 "aprendizaje de fallas" bash -c "cd '$SIM' && nice -n 5 python3 -u entrenar_fallas.py --config optimo --procs 5 --rondas 3 --semillas 3"
fase "3 pruebas de geometría + informe"
paso 35 "informe de escenarios y fallas" bash -c "cd '$SIM' && nice -n 5 python3 -u pruebas_geometria.py --config optimo --procs 5 --semillas 5"
fase "4 dataset de SLAM calibrado + Kaggle"
paso 100 "dataset de SLAM calibrado" bash -c "cd '$SIM' && nice -n 5 python3 -u generar_dataset_slam.py --config optimo --procs 5 --semillas 6 --salida $HOME/.agrobot/slam/dataset_calibrado.npz > $HOME/.agrobot/slam/generar_calibrado.log 2>&1; python3 extraer_real_slam.py --max-sesiones 80 >> $HOME/.agrobot/slam/generar_calibrado.log 2>&1"
paso 15 "publicar dataset y notebook en Kaggle" bash -c "[ -f $HOME/.agrobot/slam/dataset_calibrado.npz ] && bash '$RAIZ/tools/kaggle/publicar/publicar_kaggle.sh' $HOME/.agrobot/slam/dataset_calibrado.npz $KG $HOME/.agrobot/slam/real.npz"
fase "5 informe final"
paso 10 "informe final con diagramas" bash -c "cd '$RAIZ' && python3 tools/diagrama_flujo_tanda.py > /dev/null; python3 tools/informe_tanda.py"
fase "terminada"
log "=== TANDA 10 H TERMINADA ==="
