# Análisis de las pruebas del 2026-10-26 (21:08-21:45 UTC) — qué falla

Datos: 3 grabaciones de campo (`~/.agrobot/campo/20261026_*`), telemetría `~/.agrobot/datos/20261026_140930`, 10 videos de planta (`jetson/backend/videos/agrobot_planta_20261026_*`), mapas `maps/mision_20261026_*`. Scripts: `tools/analizar_slam_dia.py`, `probar_mapa_scan.py`, `probar_pose_fusion.py`, `probar_desfase_pose.py`.

## 1. Error del SLAM (medido contra el ICP scan-a-scan; sin verdad terreno)
| | cm por paso (0,2 s) | rotación por paso |
|---|---|---|
| parado | 0,5-0,6 | 0,2° |
| avanzando recto (~7 cm/paso) | **3,0-3,4** (p95 7-8) | 0,6-0,7° |
| girando | 2,6-2,8 | **1,0-1,3°** (p95 3-4) |
- La pose del SLAM avanza **6-19 % más** por paso que el scan-matching (17-27 % en tramos sin orden de avance: el chasis desliza).
- Giroscopio vs pose: razón 0,99-1,06; diferencia media 5-6° en 5 s (p95 13-18°).
- Consistencia al revisitar: 56-66 % de puntos a < 4 cm (desfase mediano 2,4-3,3 cm).
- `MapaScan` (mapa del dashboard) mejora la nitidez 4-7 %, pero su corrección **choca con el tope de 30 cm / 10°**: la pose se separa del mapa más que eso en cada sesión.
- Descartado: **no hay desfase temporal** pose↔scan (mejor δ = 0 en las 3 sesiones); encadenar ICP o fusionarlo con el SLAM no es consistentemente mejor.
- Pendiente: probar `minimum_travel_distance/heading` más bajos en slam_toolbox y un recorrido de 5 m medido con cinta (calibra la escala).

## 2. Otras fallas
1. **Bug mío en E11 (corregido; backend reiniciado):** el barrido contaba paredes pegadas al costado como «frente» → frente NEGATIVO (−0,18 m) y «esquina: retrocedo» sin nada delante.
2. Predictor E11: 183 giros rechazados (35 cambiados), 0 avances/retrocesos; minutos «detenido» cerca de paredes (< ~29 cm).
3. 40 % de la exploración activa estuvo «en pausa (mando manual)».
4. ~40 avisos de Anticolisión «a 6-20 cm… sin espacio detrás».
5. 13 paradas de emergencia por el dashboard en 40 min.
6. ~250 alarmas «col_dano_insecto» (una cada 3 s con el robot PARADO mirando la misma col): falta deduplicar.
7. Imagen: nitidez mediana 6-20 (61-100 % de fotos < 60), brillo 61-97/255; 3 de 10 videos de planta casi sin enfoque.
8. Planta «escaneada» en (0,74, −7,49) con el robot dentro de y ≥ −4,7: mal ubicada.
9. CO2/temperatura/humedad siguen simulados; C3 sin enlace un momento; `vx_ekf` llegó a 3,08 m/s.

## 3. Retroalimentación
- Mundos reales para el editor del simulador: `simulador_pasivo/mundos/real_mision_20261026_{2116,2132,2144}.json`.
- El simulador de E11 conserva la versión anterior del barrido: reentrenar con la corrección queda pendiente.
