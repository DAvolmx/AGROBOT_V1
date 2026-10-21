#!/bin/bash
# RONDA 2 («exígele más») de la sesión de percepción: SOLO lo que quedó flojo en la ronda 1 (2026-10-20):
#   coles  (mAP50 0,52; dano_hoja 0,09) y mosca blanca (0,50) pararon pronto por la parada temprana (paciencia 8/10),
#   y la segmentación seguía mejorando en la última época (24/24).
# Ahora: modelo YOLO11s (más grande), más épocas, paciencia alta, y la segmentación a 40 épocas. Nombres NUEVOS
# (col_s, whitefly_s, seg_exigente_*): no pisan los de la ronda 1. NO despliega nada.
#
#   setsid nohup bash tools/sesion_percepcion_exigente.sh > ~/.agrobot/aprendizaje/sesion_percepcion2.log 2>&1 &
#   touch ~/.agrobot/aprendizaje/sesion_percepcion.STOP     # parar al acabar el paso actual
set -u
RAIZ="$(cd "$(dirname "$0")/.." && pwd)"
P="$RAIZ/simulador_pasivo/percepcion_sintetica"
OUT="$HOME/.agrobot/aprendizaje"
TS="$(date +%Y%m%d_%H%M)"
STOP="$OUT/sesion_percepcion.STOP"
rm -f "$STOP"
echo "[$(date '+%F %T')] === RONDA 2 (exigente) $TS ==="

paso() {
  local nombre="$1"; shift
  if [ -e "$STOP" ]; then echo "[$(date '+%T')] STOP pedido: se salta '$nombre'"; return; fi
  echo; echo "[$(date '+%F %T')] ---- INICIO: $nombre ----"
  local t0=$(date +%s)
  ( echo 1000 > /proc/self/oom_score_adj 2>/dev/null; cd "$P" && exec "$@" )
  local rc=$?
  echo "[$(date '+%F %T')] ---- FIN: $nombre · código $rc · $(( ($(date +%s) - t0) / 60 )) min ----"
  free -h | sed -n 2p
}

paso "1/3 coles con YOLO11s (60 épocas, paciencia 20)"      python3 -u entrenar_col.py --base yolo11s.pt --nombre col_s --epocas 60 --patience 20 --batch 8
paso "2/3 mosca blanca con YOLO11s (60 épocas, paciencia 20)" python3 -u entrenar_whitefly.py --base yolo11s.pt --nombre whitefly_s --epocas 60 --patience 20 --reusar --batch 8
paso "3/3 senderos e interiores (segmentación, 40 épocas)"  python3 -u entrenar_seg.py --epocas 40 --batch 16 --nombre "seg_exigente_$TS"

echo; echo "[$(date '+%F %T')] === RONDA 2 TERMINADA ==="
