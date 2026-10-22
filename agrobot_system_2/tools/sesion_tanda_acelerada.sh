#!/bin/bash
# TANDA ACELERADA (numba, 6 núcleos, búsqueda evolutiva) con todo CALIBRADO con datos reales (2026-10-22). Reemplaza a sesion_tanda_10h.sh (misma ventana de 10 h desde tanda.inicio). Un entrenamiento a la vez; no despliega nada.
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
PY="$HOME/.agrobot/venv_sim/bin/python"      # numpy 2.2 + numba: dinamica_rapida compila el paso de la física (~60x)
FIN_EPOCH=$(( $(stat -c %Y "$MARCA") + 36000 ))   # la ventana de 10 h cuenta desde el inicio original
rm -f "$STOP"
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
log "=== TANDA ACELERADA · fin previsto $(date -d @$FIN_EPOCH '+%F %T') ==="
fase "1 manejo robusto (continuo + pulsos)"
paso 215 "búsqueda de manejo robusta" bash -c "cd '$SIM' && nice -n 5 $PY -u entrenar_robusto.py --control ambos --procs 6 --n 120 --evolucion 6 --hijos 60 --horas 3.6 --huecos 0.50,0.60,0.75 --sem-filtro 3 --sem-val 6 --sem-test 10 --top 12 --finalistas 4"
fase "2 fallas con manejo asistido"
paso 110 "aprendizaje de fallas" bash -c "cd '$SIM' && nice -n 5 $PY -u entrenar_fallas.py --config optimo --procs 6 --rondas 5 --semillas 4"
fase "3 pruebas de geometría + informe"
paso 35 "informe de escenarios y fallas" bash -c "cd '$SIM' && nice -n 5 $PY -u pruebas_geometria.py --config optimo --procs 6 --semillas 8"
fase "4 dataset de SLAM calibrado + Kaggle"
paso 100 "dataset de SLAM calibrado" bash -c "cd '$SIM' && nice -n 5 $PY -u generar_dataset_slam.py --config optimo --procs 6 --semillas 12 --salida $HOME/.agrobot/slam/dataset_calibrado.npz > $HOME/.agrobot/slam/generar_calibrado.log 2>&1; python3 extraer_real_slam.py --max-sesiones 80 >> $HOME/.agrobot/slam/generar_calibrado.log 2>&1"
paso 15 "publicar dataset y notebook en Kaggle" bash -c "[ -f $HOME/.agrobot/slam/dataset_calibrado.npz ] && bash '$RAIZ/tools/kaggle/publicar/publicar_kaggle.sh' $HOME/.agrobot/slam/dataset_calibrado.npz $KG $HOME/.agrobot/slam/real.npz"
fase "5 informe final"
paso 10 "informe final con diagramas" bash -c "cd '$RAIZ' && python3 tools/diagrama_flujo_tanda.py > /dev/null; python3 tools/informe_tanda.py"
fase "terminada"
log "=== TANDA ACELERADA TERMINADA ==="
