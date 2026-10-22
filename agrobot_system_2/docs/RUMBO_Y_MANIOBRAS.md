# Rumbo fusionado y maniobras del explorador (2026-10-22)

**Qué es.** El robot real siempre cae hacia un lado. `jetson/backend/rumbo_fusion.py` estima el error de rumbo (giroscopio + ángulo de paredes del LIDAR en el simulador; pose del SLAM en el robot) y `CorrectorRumbo` da un pulso de corrección cuando el error pasa la zona muerta, anticipando el sesgo por pulso que aprende.

**En el robot (`explorador.py`)**: APAGADO por defecto. Interruptor: `autonomia.rumbo_fusion_activo`. Con él encendido se activan juntos:
corrección de rumbo antes de cada pulso recto, histéresis de lado (`man_histeresis_cm`) y pausa tras girar (`man_pausa_s`, 0 = sin pausa, como en el óptimo validado). El retroceso previo al giro ya existía (`retroceso_previo_giro_eje_pasivo`).
Encender / apagar sin reiniciar:

    curl -X POST localhost:8080/api/params/autonomia -H 'Content-Type: application/json' -d '{"rumbo_fusion_activo": true}'

**Validar primero** con `python3 tools/probar_recta.py --si-mover --pulsos 12 --rondas 3 --atras`. Si `con_rumbo` no reduce el desvío lateral y el giro frente a `sin_corregir`, no lo enciendas.

**Solo en el simulador** (no portado): desaceleración al acercarse, pausa/escape por atasco, corrección doble, giros de varios pulsos.

**Aviso de validez.** Antes del 2026-10-22 el simulador no aplicaba `control.invertir_giro` y entrenó con giros invertidos; esos resultados no valen. Las cifras de este documento son de simulación con física hipótesis.
