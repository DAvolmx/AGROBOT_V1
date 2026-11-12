# Reajuste de PLAGASX con fotos del robot

1. `elegir_lote.py` ya creó `~/.agrobot/datos_usuario/lote_p6_20261109/` (150 fotos, sin usar modelos).
2. En `etiquetar.html`, fuente «usuario», dibujar cajas solo sobre daño o plaga visible. Marcar «sin plaga» únicamente si la planta se ve bien y está limpia. Si no se distingue, no guardar.
3. Cuando termine v9, descargar `coles_det_best.pt` y ejecutar:
   `python3 preparar_ajuste.py --v9 <ruta>/coles_det_best.pt --salida <carpeta_nueva>/agrobot-ajuste-operador`
4. Publicar esa carpeta como dataset **privado** `davidgomez32/agrobot-ajuste-operador`.
5. Ejecutar `python3 crear_notebook.py` y lanzar `publicar/` con `kernels push --accelerator NvidiaTeslaT4` (máximo 3 h de entrenamiento).
6. Leer `comparacion_robot.json`. Desplegar solo con confirmación del operador.

Reparto: el lote se divide por tramos de 5 fotos seguidas (50 % entrenamiento, 10 % validación, 40 % evaluación). Las fotos subidas a mano solo entrenan. Si `preparar_ajuste.py` encuentra una etiqueta nueva, se detiene: una clase nueva necesita un entrenamiento completo.

Limitación: el lote viene de fotos de 640×480 tomadas casi siempre de lejos. Sirve para medir falsas alarmas y daños grandes en hojas, no para pulgones o trips. Para esos hacen falta fotos de cerca.
