# Prueba de exterior 2 (2026-10-28) — análisis y mejoras

Huerta con coles sembradas (sin macetas), sendero corto. Sesión de datos `20261028_141932` (el reloj saltó +5,4 h por NTP a los 100 s: 14:19 → 19:45 UTC). Grabaciones continuas `20261028_194540`, `195430`, `200037`, `200833` (exterior 2) y `202654` (manejo manual por la casa, etiquetado aparte). Datos en `~/.agrobot/pruebas/EXT2_20261028/` y `MANEJO_MANUAL`: `MANUAL_CASA_20261028/` (solo `fotos_seguras/` es compartible).

## Cifras
| | |
|---|---|
| Sesión total | 54 min (3 242 s) |
| **Autonomía activa** | **972 s = 16,2 min (30 %)**, 45 tramos; el resto, mando manual/pausas/escaneo |
| En autonomía | adelante 161 s / 24,8 m · **atrás 53 s / 8,8 m** · girando en el sitio 129 s |
| Video continuo | 43,4 min (exterior 2: 37,4 min; casa: 6,0 min) + 40 videos de planta (9,6 min) |
| Entorno | el clasificador sugirió «interior» el 99,7 % del tiempo; el operador tuvo que fijar «exterior» a mano |
| Vibración | σ(acc_z) 0,47 en marcha frente a 0,23 parado; 386 saltos del LIDAR frontal > 0,3 m en marcha |

## Falsos positivos
- 360 detecciones de «maceta» (no había macetas) y 261 avisos de plaga (amarillamiento 108, daño de insecto 60, mildiú 52, malformación 40).
- De 42 capturas de aviso, las revisadas son flores, suculentas, llantas, suelo, alfombra, paredes y una pierna: el detector PLAGASX solo conoce coles y a cualquier planta le pone una enfermedad.
- Con la cámara a 98° (15° bajo el horizonte) el suelo cercano ocupa más de la mitad de la imagen y las coles casi no entran: el detector de coles las vio en 1 de 30 cuadros del primer tramo.

## Cambios (probados con tests o simulación; sin probar en el robot)
1. **Prioridad adelante** (`autonomia.prioridad_adelante`): sin retroceso de esquina, sin retroceso por obstáculo lateral, sin giro compensado ni «sin salida: retrocediendo»; se gira en el sitio/arco. Monte Carlo del sendero corto (4 anchos × 12 semillas): contactos con plantas 1,48 → 0,88 por episodio (−40 %), éxito igual (52 %). Con 0,70 m fallan ambas variantes (17 %): el sendero estrecho es el límite; con ≥ 0,85 m sube.
2. **Cámara más alta en exterior** (`autonomia.servo_exterior_deg` = 112°, ~3° bajo el horizonte) para ver el camino libre y las plantas; solo en autonomía y modo exterior.
3. **Avisos de plaga**: en campo sin macetas se exige una col detectada (nuevo detector `agrobot_col_yolo11n.pt` como tercer modelo) confirmada por CLIP (p_col ≥ 0,55); sin eso, se descartan (`plaga_sin_maceta = "suprimir"`).
4. **Clasificador de entorno** reetiquetado (233 muestras de exterior 2 + interior) y desplegado: 97,8 % frente a 91,6 % de las reglas en validación por bloques; error «interior tomado por exterior» 2,4 % (el peligroso). Para volver a las reglas: borrar `~/.agrobot/entorno/modelo.json`.
5. **Datasets**: +107 negativos duros del exterior 2 en `agrobot-negativos-fondo-interior` v3 (sin personas ni pieles; se excluyeron las confirmadas como col) y reentrenamiento de PLAGASX en Kaggle.

## Vibración y LIDAR (mecánico, no resuelto por software)
El LIDAR se baja de su posición: apretar la base con tuerca autoblocante y añadir amortiguadores de goma entre el soporte y el chasis; revisar los tornillos tras cada recorrido. Mientras tanto, en terreno rugoso conviene bajar `escala_velocidad` exterior.
