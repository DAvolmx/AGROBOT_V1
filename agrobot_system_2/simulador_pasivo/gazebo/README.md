# Sendero del huerto — 12 oct 2026

97 fotos originales, resolución 1599×1200, tomadas durante una secuencia corta alrededor de las 15:40. Se conserva cada archivo sin modificación; `manifest.json` incluye SHA-256 para comprobar integridad y `contact_sheet.jpg` sirve para revisar cobertura.

## Qué aporta

Las imágenes muestran un sendero de tierra/barro con terrones y bordes elevados, cultivos de hoja ancha próximos al paso, maleza, piedras pequeñas y cambios de encuadre. Esto sirve para ampliar la apariencia del simulador y como conjunto de escenas para calibrar la percepción monocular del corredor.

## Estado de etiquetas

Las fotos están **sin etiquetas semánticas**. No deben usarse como máscaras de verdad ni como validación cuantitativa de un modelo hasta marcar, al menos, `transitable`, `planta/no pisar`, `obstáculo bajo` y `fondo`. No se deduce profundidad ni distancia métrica de estas fotos RGB. Conviene separar entrenamiento y evaluación por tramo/recorrido, no por fotograma consecutivo, para evitar fuga de imágenes casi idénticas.

## Uso en Gazebo

El ZIP se toma como referencia visual y de geometría cualitativa. El ancho nominal del corredor se fija en 0,50 m por el dato ya proporcionado; no se estima escala adicional a partir de la perspectiva de estas fotos. La cámara del simulador y el ángulo de servo aún requieren calibración física.
