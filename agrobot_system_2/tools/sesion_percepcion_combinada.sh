#!/bin/bash
# Sesión de entrenamiento COMBINADA de percepción: senderos/interiores + objetos + coles + mosca blanca + plagas.
# Un entrenamiento a la vez (la Jetson tiene 7,5 GB compartidos con la GPU). Cada paso es un proceso propio con
# oom_score_adj 1000 (si falta memoria, muere el entrenamiento, no ROS). Si un paso falla, sigue con el siguiente.
# NO despliega nada: los modelos quedan en simulador_pasivo/percepcion_sintetica/{modelos,yolo_*}.
#
#   setsid nohup bash tools/sesion_percepcion_combinada.sh > ~/.agrobot/aprendizaje/sesion_percepcion.log 2>&1 &
#   tail -f ~/.agrobot/aprendizaje/sesion_percepcion.log        # seguirla
#   touch ~/.agrobot/aprendizaje/sesion_percepcion.STOP         # pedir que pare al acabar el paso actual
#   pkill -f sesion_percepcion_combinada.sh; pkill -f entrenar_  # parar ya
set -u
RAIZ="$(cd "$(dirname "$0")/.." && pwd)"
P="$RAIZ/simulador_pasivo/percepcion_sintetica"
OUT="$HOME/.agrobot/aprendizaje"
TS="$(date +%Y%m%d_%H%M)"
RESP="$OUT/respaldo_percepcion_$TS"
STOP="$OUT/sesion_percepcion.STOP"
mkdir -p "$OUT" "$RESP"
rm -f "$STOP"

echo "[$(date '+%F %T')] === SESIÓN COMBINADA $TS ==="
echo "[respaldo] copiando lo ya entrenado a $RESP"
cp -a "$P/modelos" "$RESP/modelos" 2>/dev/null
for d in yolo_col yolo_obj yolo_plagas yolo_whitefly; do [ -d "$P/$d" ] && cp -a "$P/$d" "$RESP/$d" 2>/dev/null; done
echo "[respaldo] listo"

paso() {   # paso <nombre> <comando...>
  local nombre="$1"; shift
  if [ -e "$STOP" ]; then echo "[$(date '+%T')] STOP pedido: se salta '$nombre'"; return; fi
  echo; echo "[$(date '+%F %T')] ---- INICIO: $nombre ----"
  local t0=$(date +%s)
  ( echo 1000 > /proc/self/oom_score_adj 2>/dev/null; cd "$P" && exec "$@" )
  local rc=$?
  echo "[$(date '+%F %T')] ---- FIN: $nombre · código $rc · $(( ($(date +%s) - t0) / 60 )) min ----"
  free -h | sed -n 2p
}

paso "1/5 senderos e interiores (segmentación de navegación)" python3 -u entrenar_seg.py --epocas 24 --batch 16 --nombre "seg_sesion_$TS"
paso "2/5 objetos de interior (YOLO)"                        python3 -u entrenar_yolo.py --tarea obj --epocas 40
paso "3/5 coles: planta, cogollo y daño de hoja"             python3 -u entrenar_col.py --epocas 25 --batch 8
paso "4/5 mosca blanca"                                      python3 -u entrenar_whitefly.py --epocas 25 --reusar
paso "5/5 plagas (sintético)"                                python3 -u entrenar_yolo.py --tarea plagas --epocas 40

echo; echo "[$(date '+%F %T')] === SESIÓN TERMINADA ==="
