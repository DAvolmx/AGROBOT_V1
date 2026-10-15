# Análisis del modelo Kaggle para AgroBot

Fecha: 12 de octubre de 2026. Notebook: `davidgomez32/notebook972b3bbf33`.

## Artefactos descargados

La Jetson conserva el notebook y sus salidas en `/home/jetson/.agrobot/kaggle_notebook972b3bbf33/`. Se descargaron los checkpoints base y v3, métricas, curvas, matrices de confusión, ejemplos visuales y el log del entrenamiento. Las credenciales se instalaron en `/home/jetson/.kaggle/kaggle.json` con permisos `600`; el CLI Kaggle vive en un entorno aislado bajo `~/.agrobot/kaggle-cli/`.

El archivo del dataset original no se pudo recuperar: tanto la descarga de Kaggle CLI como la URL de descarga autenticada devolvieron HTTP 404. El log del notebook confirma 239 pares de imagen/máscara, 209 reales y 30 sintéticos. En esta validación quedaron disponibles las gráficas y ejemplos exportados, pero no las 239 imágenes originales.

## Arquitectura e interfaz

La mejor candidata es `modelo_agrobot_unet.pth`: U-Net de `segmentation_models_pytorch`, encoder `timm-mobilenetv3_large_100`, entrada de 512 × 384 píxeles, seis canales y cinco clases: fondo, sendero, cultivo, bancal y obstáculo.

Los seis canales deben construirse igual que durante el entrenamiento: RGB en [0,1], canal `a*` de CIELAB dividido por 255, ExGR reescalado a [0,1] y suma de top-hat + black-hat morfológicos dividida por 255. El validador usa esta receta exacta y cargó los pesos con coincidencia estricta de claves.

La variante `modelo_agrobot_unet_v3.pth` agrega Laplaciano, Sobel y LBP para nueve canales, pero reduce las clases a cuatro y quita obstáculo. Su calidad medida es claramente inferior, por lo que queda como artefacto de comparación.

## Métricas reportadas por Kaggle

| Variante | mIoU validación | mIoU fotos reales | mIoU sintético | Resultado por clase |
|---|---:|---:|---:|---|
| Base, 6 canales / 5 clases | 0.710 | 0.739 | 0.565 | sendero 0.752; cultivo 0.807; bancal 0.673; obstáculo sin métrica |
| v3, 9 canales / 4 clases | 0.388 | 0.396 | 0.311 | sendero 0.482; cultivo 0.444; bancal 0.408 |

El notebook reporta 204 muestras de entrenamiento y 35 de validación. La clase obstáculo no tuvo píxeles etiquetados útiles; el checkpoint base no está validado como detector de obstáculos bajos. Las métricas se toman del artefacto exportado, no son una evaluación independiente del dataset.

## Prueba sobre el simulador

Se añadió [validar_kaggle_sim.py](simulador_pasivo/validacion_percepcion/validar_kaggle_sim.py) en la Jetson. Renderiza seis vistas del mundo `huerta_fotos_50cm`, aplica el modelo base y guarda superposiciones y métricas cualitativas en `simulador_pasivo/validacion_percepcion/salidas_kaggle_base/`. La prueba terminó correctamente en CPU; media medida 1,159 ms por imagen. Ese tiempo no es una medición de GPU ni de TensorRT.

Las superposiciones muestran detección útil del sendero en varios ángulos, con confusiones de suelo/mulch como cultivo o bancal en otras vistas. La proporción media predicha fue 16.27% sendero, 31.26% cultivo, 13.09% bancal y 0% obstáculo. Son porcentajes de salida, no métricas de acierto: el render actual no proporciona máscaras semánticas de verdad para calcular IoU.

![Seis vistas del simulador: imagen original a la izquierda y clases predichas a la derecha](simulador_pasivo/validacion_percepcion/salidas_kaggle_base/montaje.png)

## Uso recomendado en simulación y robot

Usar el modelo base primero como percepción en sombra: segmentar sendero y zonas de cultivo/bancal, medir centro y rumbo del sendero en varias franjas de la imagen, suavizar temporalmente y rechazar lecturas con baja confianza. Mantener LIDAR y las reglas de parada como veto independiente. No usar la clase obstáculo hasta incorporar y medir ejemplos anotados de piedras, mangueras, plantas caídas y otros objetos por debajo del plano del LIDAR.

El principal trabajo para cerrar la brecha es visual: añadir al simulador escenas basadas en fotos reales de día y noche, variando iluminación, exposición, desenfoque, barro, paja y densidad de cultivo. Después, el simulador debe exportar una máscara semántica sincronizada con cada RGB para evaluar IoU por clase y por escenario. Las predicciones actuales sirven para revisar dominio visual, no para autoetiquetar datos ni accionar motores.

Comando de repetición en Jetson, forzando CPU mientras la corrida de política usa CUDA:

```bash
cd ~/Desktop/AGROBOT/agrobot_system_2
AGROBOT_FORCE_CPU=1 python3 -u simulador_pasivo/validacion_percepcion/validar_kaggle_sim.py
```

## Estado del entrenamiento de navegación

La corrida separada de la política sigue activa en la Jetson. En la tanda 3, el evaluador reportó desvío medio de 89.6 a 81.6 cm y cero pisadas, con 21.12 choques por 10 m. La mejora de centrado aún no basta para considerar robusta la conducción. En la última lectura había 1.3 GiB disponibles y 615 MiB de swap ocupados; conviene vigilar la memoria durante las tandas siguientes.
