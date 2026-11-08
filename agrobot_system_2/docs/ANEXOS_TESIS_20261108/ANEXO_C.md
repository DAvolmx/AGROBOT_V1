# Anexo C. Conjunto de datos de plagas y enfermedades

Este anexo documenta el conjunto usado para entrenar PLAGASX v8, el modelo desplegado en el robot, y los cambios de v9. Se reconstruyó a partir del dataset generado (`dataset_combinado`) y del cuaderno de Kaggle `davidgomez32/plagasx`.

## C.1 Clases

**Tabla C-1.** Clases y número de cajas en PLAGASX v8. Una imagen puede tener varias cajas.

| Id | Clase | Entrenamiento | Validación | Prueba |
|---|---|---|---|---|
| 0 | `col_sana` | 144 | 41 | 15 |
| 1 | `col_amarillamiento` | 16 | 4 | 5 |
| 2 | `col_manchas_cafe` | 142 | 41 | 30 |
| 3 | `col_dano_insecto` | 6 555 | 884 | 718 |
| 4 | `col_mildiu_velloso` | 199 | 64 | 27 |
| 5 | `col_malformacion` | 143 | 39 | 18 |
| 6 | `col_pudricion` | 0 | 0 | 0 |
| 7 | `col_mosca_blanca` | 1 169 | 241 | 230 |
| 8 | `col_pulgon` | 1 176 | 78 | 67 |
| 9 | `col_oruga` | 742 | 76 | 72 |
| 10 | `col_pulguilla` | 357 | 12 | 15 |
| 11 | `col_trips` (solo v9) | — | — | — |

La clase `col_pudricion` existe pero no tuvo ninguna imagen: el modelo v8 no puede detectarla. El conjunto está muy desbalanceado: `col_dano_insecto` concentra la mayoría de las cajas, y `col_amarillamiento` solo tiene 25.

## C.2 Partición de entrenamiento, validación y prueba

**Tabla C-2.** Imágenes por origen y partición en v8.

| Origen | Contenido | Entrenamiento | Validación | Prueba | Regla de partición |
|---|---|---|---|---|---|
| `mio` | Fotos propias de coles | 120 | 33 | 17 | Conserva la partición de origen del dataset propio |
| `caj` | Cajas reales de daño y mosca blanca | 1 654 | 271 | 268 | Conserva la partición de origen |
| `ip` | IP102, plagas seleccionadas | 1 819 | 127 | 124 | train → entrenamiento; valid → alterna validación y prueba |
| `ext` | Datasets externos de coliflor y col | 700 | 200 | 100 | Aleatoria 70/20/10 con semilla 42 |
| `fondo` | Senderos propios sin enfermedad | 96 | 24 | 0 | Una de cada 5 a validación |
| `negint` | Negativos de interior y exterior | 258 | 87 | 86 | Cíclica: validación, prueba y tres a entrenamiento |
| **Total** | | **4 647** | **742** | **595** | |

Las imágenes sin cajas enseñan al modelo qué no es plaga: 460 en entrenamiento, 125 en validación y 107 en prueba.

**Riesgos de esta partición.**

- **Fuga en `ext`.** La partición es aleatoria por imagen y parte de estos datasets contiene variantes aumentadas de una misma foto. Las métricas de ese origen pueden ser optimistas.
- **Fotos cercanas en `mio`.** No se separó por planta ni por sesión.
- **Resultados de referencia.** mAP50 de 0,843 y top-1 de 0,932 corresponden a esta prueba, no a fotos nuevas del robot.

## C.3 Tipos de anotación

| Origen | Tipo de caja | Fiabilidad |
|---|---|---|
| `mio` | Automática, por color | **Baja:** muchas cajas abarcan la imagen o zonas sin daño |
| `ext` | Imagen completa | Sirve para clasificar, no para localizar |
| `caj` | Dibujada por los autores de cada dataset | Alta en localización. Las de mosca blanca son mayormente de otros cultivos |
| `ip` | Dibujada (IP102) | Alta en localización; varias especies no son de coles |
| Anotación manual propia | `etiquetar.html` | A la fecha hay 61 fotos revisadas y solo 1 caja |

## C.4 Ejemplos de anotaciones

![Figura C-1. Cajas automáticas de las fotos propias. Se muestran tal como entraron al entrenamiento: varias no coinciden con un daño real.](figuras/anot_propias_automaticas.jpg)

![Figura C-2. Cajas reales de daño de insecto en col y de mosca blanca en hojas de otro cultivo.](figuras/anot_cajas_reales.jpg)

## C.5 PLAGASX v9

La versión 9 se lanzó el 8 de noviembre de 2026 para coles y coliflores y estaba en entrenamiento al compilar este anexo. Sus resultados no se incluyen.

| Aspecto | v8 | v9 |
|---|---|---|
| Clases | 11 | 12 (añade `col_trips`) |
| Detector | yolov8s, 800 px | yolo11m, 960 px, 2 GPU T4 |
| Clasificador | yolov8s-cls, 224 px | yolo11m-cls, 320 px |
| Máximo por clase externa | 200 | 500 |

Las imágenes de `col_trips` provienen de IP102 y son de cereales y mango, **ninguna de coles**. Un buen resultado en esa clase no demuestra que detecte trips en coles o coliflores.

## C.6 Trabajo pendiente

La prueba P6 exige etiquetar 150 fotos del robot sin ver la salida del modelo. Es el dato que falta para afirmar la exhaustividad en plantas reales. Desde el 8 de noviembre, `etiquetar.html` admite etiquetas ilimitadas y muestra las fotos de la misión «Recoger dataset de plantas».
