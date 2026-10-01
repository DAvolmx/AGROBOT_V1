#!/usr/bin/env bash
# Supervisor de un entrenamiento largo del simulador.
#
#   tools/supervisor_entrenamiento.sh <hora_fin_epoch> <log> [args extra de entrenar.py]
#
# Si entrenar.py muere antes de la hora de fin (el 2026-09-30 lo mató el OOM
# killer a los 80 min y la "noche de entrenamiento" fue de una tanda), lo
# relanza con las horas que quedan y SIGUIENDO la numeración de tandas: los
# datos se escriben episodio a episodio y el modelo de cada tanda ya está en
# disco, así que no se pierde nada más que la tanda en curso.
#
# Cada proceso de entrenamiento queda con oom_score_adj = 1000: si falta
# memoria, el kernel mata ESTO y no el backend del robot.
set -u
cd "$(dirname "$0")/.."
FIN="$1"; LOG="$2"; shift 2
EXTRA=("$@")
export AGROBOT_INICIO_ENTRENAMIENTO="${AGROBOT_INICIO_ENTRENAMIENTO:-$(date +%Y%m%d_%H%M)}"

proteger() {
  for p in $(pgrep -f "simulador/entrenar.py"); do
    echo 1000 > "/proc/$p/oom_score_adj" 2>/dev/null
  done
}

while [ "$(date +%s)" -lt "$((FIN - 600))" ]; do
  if pgrep -f "simulador/entrenar.py" > /dev/null; then
    proteger
    sleep 30
    continue
  fi
  QUEDAN=$(python3 -c "print(round(($FIN - $(date +%s)) / 3600, 3))")
  ULT=$(grep -oE -- "--- tanda [0-9]+" "$LOG" | tail -1 | grep -oE "[0-9]+")
  SIG=${ULT:-1}
  echo "$(date '+%Y-%m-%d %H:%M:%S')  [supervisor] entrenar.py no corre: relanzo en la tanda $SIG con $QUEDAN h" | tee -a "$LOG"
  nohup nice -n 10 python3 -u simulador/entrenar.py --horas "$QUEDAN" --tanda-inicial "$SIG" \
      "${EXTRA[@]}" >> "$LOG" 2>&1 &
  sleep 20
  proteger
done
echo "$(date '+%Y-%m-%d %H:%M:%S')  [supervisor] fin de la ventana de entrenamiento" | tee -a "$LOG"
