#!/bin/bash
cd /home/jetson/Desktop/AGROBOT/agrobot_system_2/simulador_pasivo/percepcion_sintetica
L=~/.agrobot/aprendizaje/pipeline_cam3_20261027.log
echo "== $(date +%T) preparar_real (con las sesiones de hoy)" >> $L
python3 preparar_real.py --cada 6 >> $L 2>&1
echo "== $(date +%T) entrenamiento" >> $L
python3 entrenar_trav.py --real --datos datos_cam_real --epocas 8 --nombre trav_cam3_20261027 >> $L 2>&1
echo "== $(date +%T) evaluación por columna" >> $L
python3 eval_real_columnas.py modelos/seg_10h_20261021_0213.mejor.pt modelos/trav_cam2_20261027.mejor.pt modelos/trav_cam3_20261027.mejor.pt >> $L 2>&1
echo "== $(date +%T) FIN" >> $L
