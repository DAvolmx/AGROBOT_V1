# E11 montada en el robot real (2026-10-26)

**Qué es:** la estrategia entrenada en simulación (`docs/estrategias/e11_entrenado/informe.json`: 28 % de misión completa, 62 % vuelve al inicio, 2,5 contactos por episodio; sin entrenar, 6 % y 12,2). **No está medida en el robot real.**

## Qué quedó activo (autonomia.e11_* en `agrobot_config.json`; por defecto también en `config_store.py`)
| Pieza | Qué hace | Dónde |
|---|---|---|
| Predictor de colisión por pulso | antes de CADA pulso autónomo (adelante, atrás, giro) barre la huella del robot contra el scan; si chocaría, no lo manda (o cambia el giro por el contrario / atrás). Si 8 rechazos en 20 s, relaja los márgenes 25 s para despegarse | `explorador._conducir` |
| Barrido de corredor | el «frente» también se mide en un corredor del ancho del robot (±25,6 cm) | `explorador._frente_barrido` |
| Regreso al inicio | botón **↩ Regresar al inicio** (o `POST /api/exploracion/regresar`): ruta A* sobre el mapa que acumuló el scan, persecución pura; llega a < 0,5 m y se detiene | `explorador._paso_regreso`, `planificador.py` |
| Escape de nicho | `mx_anti_dither`: atrapado 25 s → atrás ×3, giro largo (8 pulsos) al lado contrario, adelante ×3 | `explorador._escape_nicho` |
| Guardián del sendero | `guardian_sendero_activo` (tolerancia 4,6 cm / 6°, sin memoria) | `guardian_sendero.py` |
| Maniobras entrenadas | `man_retroceso_cm` 29,8, `man_atras_pulsos` 2, `man_histeresis_cm` 17,9, `man_pausa_s` 1,3 | `explorador` |
| Inspección de plantas a ~80 cm | la misión de plantas se detiene a 0,78 m de la maceta (antes 0,42) | `jetson_server.py` (`distancia_inspeccion_m`) |

## Qué NO se tocó (acoplado al frenado del robot)
`escala_velocidad`, `avance_libre_cm`, el cono del LIDAR y los límites del firmware de la C3. El entrenamiento usó otros valores (velocidad interior 0,10 / exterior 0,31); si quieres probarlos: `tools/aplicar_resultado_entrenamiento.py` (muestra el cambio antes de aplicar).

## Cómo se usa
1. Pon el robot en el suelo con espacio libre, **en el punto de salida**.
2. `▶ Explorar y mapear` (o la misión de plantas). El punto de salida queda registrado al arrancar.
3. Cuando quieras que vuelva: `↩ Regresar al inicio`. (Opcional: `e11_ida_max_m` / `e11_ida_max_s` lo disparan solos; 0 = solo manual.)
4. `■ Detener` siempre manda.

## Cómo apagarlo
`POST /api/params/autonomia {"e11_activo": false}` (o ponerlo en `false` en `agrobot_config.json` y reiniciar): el explorador vuelve a ser el de antes. `e11_predictor=false` apaga solo el predictor. `guardian_sendero_activo=false` apaga el guardián.

## Lo que NO está hecho / límites
- El mapa del planificador usa la pose del SLAM con el LIDAR tomado en el centro de `base_link` (unos cm de error).
- El regreso necesita pose del SLAM: sin ella no arranca. Si el planificador no encuentra ruta, el explorador sigue en modo reactivo 6 s.
- «Visitar plantas y repetir el escaneo» de E11 solo está en el simulador: en el robot la misión de plantas ya va maceta por maceta, pero sin reintento.
- Sin rumbo a la meta (en el robot no hay meta fija) ni «seguir el hueco» (entrenado casi apagado).
- Primera prueba recomendada: un pasillo recto de 5 m con cinta de salida, con la mano en `■ Detener`.
