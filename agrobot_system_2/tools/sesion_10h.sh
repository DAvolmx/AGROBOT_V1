#!/bin/bash
# SESIÓN DE 10 HORAS (2026-10-21 02:15 -> 12:10): manejo + percepción + informes. Un entrenamiento a la vez.
#   0  espera a que termine la sesión de manejo v5 (sesion_manejo_optimo.sh: fase 1 + fase 2 de fallas)
#   1  informe 1: escenarios (recto a tope, sala cuadrada, hileras, zigzag + los 4 clásicos) y TODAS las fallas
#   2  manejo ronda 2: búsqueda más grande (continuo y pulsos), más agresiva
#   3  percepción (interior primero): segmentación senderos/interiores/exteriores, macetas (obj), plagas, coles
#   4  informe final con el modelo de manejo óptimo
# NO despliega nada. Parar: touch ~/.agrobot/aprendizaje/sesion_10h.STOP   Seguir: tail -f ~/.agrobot/aprendizaje/sesion_10h.log
set -u
RAIZ="$(cd "$(dirname "$0")/.." && pwd)"
SIM="$RAIZ/simulador_pasivo"; P="$SIM/percepcion_sintetica"
OUT="$HOME/.agrobot/aprendizaje"; STOP="$OUT/sesion_10h.STOP"; ESTADO="$OUT/sesion_10h.estado"
FIN_EPOCH=$(date -d "2026-10-21 12:10" +%s)
TS="$(date +%Y%m%d_%H%M)"
rm -f "$STOP"
echo 1000 > /proc/self/oom_score_adj 2>/dev/null
log() { echo "[$(date '+%F %T')] $*"; }
fase() { echo "$1" > "$ESTADO"; log "=== FASE $1 ==="; }
quedan_min() { echo $(( (FIN_EPOCH - $(date +%s)) / 60 )); }
paso() {   # paso <min_estimados> <nombre> <cmd...>
  local est="$1" nombre="$2"; shift 2
  if [ -e "$STOP" ]; then log "STOP pedido: se salta '$nombre'"; return; fi
  if [ "$(quedan_min)" -lt $((est + 45)) ]; then log "sin tiempo para '$nombre' (necesita ~${est} min, quedan $(quedan_min)): se salta"; return; fi
  log "---- INICIO: $nombre (~${est} min) ----"; local t0=$(date +%s)
  ( echo 1000 > /proc/self/oom_score_adj 2>/dev/null; "$@" ); local rc=$?
  log "---- FIN: $nombre · código $rc · $(( ($(date +%s) - t0) / 60 )) min ----"; free -h | sed -n 2p
}
log "=== SESIÓN 10 H ($TS) · fin previsto $(date -d @$FIN_EPOCH '+%F %T') ==="
fase "0 espera de la sesión de manejo v5"
while pgrep -f "[s]esion_manejo_optimo.sh" >/dev/null; do sleep 60; [ -e "$STOP" ] && break; done
fase "1 informe de escenarios y fallas"
paso 45 "informe 1 (escenarios + fallas)" bash -c "cd '$SIM' && python3 -u pruebas_geometria.py --config optimo --procs 6 --semillas 4"
fase "2 manejo ronda 2 (búsqueda agresiva)"
paso 100 "manejo ronda 2 (continuo+pulsos, n=30)" bash -c "cd '$SIM' && python3 -u entrenar_robusto.py --control ambos --procs 6 --n 30 --horas 1.6 --huecos 0.50,0.75 --sem-filtro 2 --sem-val 3 --sem-test 5 --top 6 --finalistas 3"
fase "3 percepción (interior primero)"
RESP="$OUT/respaldo_percepcion_$TS"; mkdir -p "$RESP"; cp -a "$P/modelos" "$RESP/modelos" 2>/dev/null
for d in yolo_col yolo_obj yolo_plagas yolo_whitefly; do [ -d "$P/$d" ] && cp -a "$P/$d" "$RESP/$d" 2>/dev/null; done
paso 90  "3.1 senderos / interiores / exteriores (segmentación 30 épocas)" bash -c "cd '$P' && python3 -u entrenar_seg.py --epocas 30 --batch 16 --nombre seg_10h_$TS"
paso 55  "3.2 macetas y objetos de interior (YOLO, 60 épocas)"       bash -c "cd '$P' && python3 -u entrenar_yolo.py --tarea obj --epocas 60"
paso 55  "3.3 plagas (YOLO, 45 épocas)"                              bash -c "cd '$P' && python3 -u entrenar_yolo.py --tarea plagas --epocas 45"
paso 55  "3.4 coles YOLO11s (40 épocas, paciencia 15)"               bash -c "cd '$P' && python3 -u entrenar_col.py --base yolo11s.pt --nombre col_10h --epocas 40 --patience 15 --batch 8"
fase "4 informe final"
paso 30 "informe final (manejo óptimo de la ronda 2)" bash -c "cd '$SIM' && python3 -u pruebas_geometria.py --config optimo --procs 6 --semillas 5"
fase "terminada"
log "=== SESIÓN 10 H TERMINADA ==="
