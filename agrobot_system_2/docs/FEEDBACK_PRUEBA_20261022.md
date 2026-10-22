# Retroalimentación de la prueba real del 2026-10-22 (interiores)

Datos: sesión `~/.agrobot/datos/20261022_144243` (25 min, telemetría + 12 768 eventos) y grabación de campo `~/.agrobot/campo/20261022_145638_234236` (16 min de video a 10 fps, 4 730 muestras, 1 575 fotos).

## Qué falló y por qué (con números)
| Hallazgo | Evidencia | Corrección |
|---|---|---|
| **Falsos positivos del detector de plagas** (`col_dano_insecto` 77–95 %) sobre pufs, puerta, parlante y zapatos | 82/82 capturas finales sin planta; ~20 alarmas/min con el robot parado | Puertas en `camera.py`: ≥8 % de vegetación dentro de la caja y persistencia 3 de 5 fotogramas. 272 negativos en `~/.agrobot/dataset_planta/negativos_fondo_20261022(.zip)` para reentrenar (pendiente: el dataset de positivos está en Kaggle) |
| **Imagen oscura y borrosa** | brillo medio 65/255, 89 % de las fotos con nitidez (var. Laplaciano) < 60 | `hardware.camara_ajustes` (exposición, ganancia, brillo, enfoque) + `tools/ajustar_camara.py` para elegir valores con la cámara conectada y el backend parado |
| **Suelo por cámara da hueco 0** | `suelo_hueco`=0 en el 51 % de las muestras (rachas: mediana 0,4 s, p90 14,5 s) | `suelo_libre.py`: hace falta que la franja «rara» persista 3 fotogramas antes de declarar hueco 0. El freno por cámara sigue apagado en interiores hasta validar con baldosa oscura |
| **Sendero confundido en interiores** | el beige del suelo/paredes cae en H 10–45 («paja») | Perfil interior: `sendero_vision=false`, `sendero_paja=false` (solo LIDAR) |
| **Retroceso fuerte y largo** | un episodio de 6,5 s / 52 cm con Nav2 (`min_vel_x`=−0,2) | DWB sin marcha atrás (en vivo), suavizador −0,06 m/s y aceleración 0,5 (en el yaml, al reiniciar ROS), anticolisión 8 cm a 0,08 m/s con 30 cm libres atrás |
| **Objetos bajos invisibles** (pufs ≈15 cm) | LIDAR a 37,5 cm; el freno por cámara no es fiable | Necesita sensor bajo o cámara mejor orientada (hardware) |
| **Robot parado el 90 %** | 89 % de filas con v≈0; últimos 4 min estático | Operativo: el explorador no estuvo activo; ver `docs/RUMBO_Y_MANIOBRAS.md` |
| **Cámara sin moverse** | servo en 88° (61 %) o 98°; `barrido_tilt` apagado | Pendiente: medir altura del lente y posición respecto a la cara frontal |

## Retroalimentación del simulador y el entrenador
- **Bug grave:** el simulador no aplicaba `control.invertir_giro`; todo entrenamiento de pulsos previo al 2026-10-22 es inválido (éxito base 6 % → 56 % al corregirlo).
- Recalibrado con 385 giros y 371 rectos reales (29 % de giros < 2°); el entrenamiento calibrado se relanzó el 2026-10-22 15:29 UTC (`~/.agrobot/aprendizaje/reentreno_calibrado_20261022b.log`).
- Rumbo fusionado (giroscopio + pose del SLAM en el robot) y maniobras nuevas: ver `docs/RUMBO_Y_MANIOBRAS.md`. El giroscopio marca −2,5 °/s de media en marcha recta (11 °/s de desviación): hay inclinación, pero el signo por pulso no es consistente (n=16), por eso el corrector aprende el lado.

## Qué no se pudo verificar
Nada de esto se probó aún en el robot con la cámara y los sensores conectados. La búsqueda web falló (error de modelo), así que no hay investigación externa.
