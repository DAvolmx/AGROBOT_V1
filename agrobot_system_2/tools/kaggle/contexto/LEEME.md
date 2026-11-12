# Modelo de contexto

1. En `etiquetar.html`, añadir cajas `planta` o `maceta` en fotos del lote y de la misión. Hacen falta al menos 15 fotos en evaluación.
2. Ejecutar `python3 preparar_contexto.py --salida <carpeta_nueva>/agrobot-contexto-robot`.
3. Publicar esa carpeta como dataset **privado** `davidgomez32/agrobot-contexto-robot`.
4. Cuando no haya otro entrenamiento en curso: `python3 crear_notebook.py` y lanzar `publicar/` con `kernels push --accelerator NvidiaTeslaT4` (máximo 4 h).
5. Leer `comparacion_contexto.json`. Desplegar solo con confirmación del operador.

Clases: planta, col, cogollo, maceta, obstaculo y borde_bajo (solo agronomía y lo que ayuda a manejar; desde el 2026-11-10 no hay persona, animal ni vehículo: el robot no hace nada con ellos y el LIDAR ya los frena como obstáculo). Para añadir herramienta, manguera, hueco, piedra, puerta, pared o cerca hace falta etiquetarlas, añadirlas a `CLASES` en ambos scripts y volver a entrenar.

## Etiquetas hechas por Claude (2026-11-10)

`~/.agrobot/etiquetas/contexto_claude/`: 160 cuadros variados de `~/.agrobot/campo` y `~/.agrobot/datos`, todos a más de 60 s de cualquier foto de evaluación. El modelo de contexto propuso cajas, Claude las revisó foto a foto (aceptar, corregir, añadir lo que faltaba) y descartó las dudosas. Resultado: 127 fotos, 243 cajas (132 planta, 111 maceta), 47 fotos sin plantas, 33 descartadas. Convención del operador: `planta` = planta de cultivo en tierra (no arbustos ni maleza); `maceta` = planta con su maceta. Van a `claude_train/` (solo entrenamiento, 5 repeticiones frente a 10 de las del operador); `decisiones.jsonl` guarda cada decisión.

Dataset público añadido: `cubeai/crop-weed-detection-for-yolov8` (CC0): crop → planta, la maleza no se marca. Vista casi cenital, aporta variedad, no la vista del robot.

Surcos: no se añaden como cajas. Un surco es una franja larga, no un objeto; lo cubre la segmentación del sendero (UNet v2, sin desplegar).
