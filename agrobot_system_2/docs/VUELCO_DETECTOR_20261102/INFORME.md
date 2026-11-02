# Detector de vuelco — simulación y entrenamiento (2026-11-02)

Datos completos: `resultado.json` (métricas por tipo y detalle de cada falsa alarma) y `entrenamiento.log`.
Código: `jetson/backend/detector_vuelco.py`, `simulador_pasivo/vuelco_datos.py` y `simulador_pasivo/entrenar_vuelco.py`.

## Por qué la regla actual no sirve
La regla de `modo_seguro.py` dice: vuelco si la inclinación instantánea del acelerómetro es ≥ 38°, o ≥ 22° con balanceo ≥ 90 °/s.

En **183 h de telemetría real** (214 sesiones, sin un solo vuelco) el acelerómetro con el robot en marcha marca inclinaciones de 50–120° en el p99, y el giroscopio picos de 190 °/s.
- Son **golpes de 0,1–0,3 s al arrancar cada pulso**, con los motores al 78–100 %. A los 0,5 s el robot vuelve a ~4°.
- Con esa regla salen **93 falsas alarmas por hora de marcha**. Es lo que enclavó el modo seguro el 19-oct («VUELCO 77°») y la razón por la que está apagado.

Otra trampa encontrada en los datos: el **nodo del IMU se congela 2–5 s** y repite el último paquete, a veces el de un golpe. Parece una inclinación sostenida y no lo es. El detector ignora los paquetes idénticos.

## Datos

| Fuente | Contenido | Uso |
|--------|-----------|-----|
| Real | 214 sesiones, niveladas por sesión con la mediana de la gravedad en reposo (como `imu_nivel`). 169 sesiones anteriores al 27-oct (2,2 h en marcha) | Entrenamiento |
| Real | 45 sesiones del 27-oct en adelante (0,7 h en marcha, exteriores 1 y 2, prueba en L) | Prueba |
| Simulada | Péndulo sobre la arista de `fallos_3d.py` + **ruido real** (493 trozos de 6 s de golpes en marcha) a ~10 Hz con jitter y paquetes repetidos. Dos lotes de 360 episodios con semillas distintas (entrenamiento / prueba) | Ambos |

Episodios simulados:
- **Vuelcos:** bulto de 3–10 cm bajo una rueda a 0,15–0,9 m/s; giro con aceleración lateral por encima del límite; caída de morro hasta 50–90°.
- **Negativos duros:**
  - casi-vuelcos que se recuperan;
  - taludes sostenidos de 5–24°;
  - baches de 8–28° durante 0,1–0,5 s;
  - llano con golpes.

## Resultados en la prueba (datos que el entrenamiento no vio)

| | Vuelcos detectados | Latencia media / p90 | Falsas en negativos simulados | Falsas reales en marcha |
|---|---|---|---|---|
| Regla actual (modo_seguro) | 51/128 (40 %) | −0,1 / 0,1 s | 216/232 (93 %) | 63 → **92,7/h** |
| Detector con valores a mano | 121/128 (95 %) | 1,5 / 2,2 s | 5/232 | 5 → 7,4/h |
| **Detector entrenado (desplegado como defecto)** | **110/128 (86 %)** | 2,0 / 2,6 s | **0/232** | **0 → 0/h** |

Por tipo, entrenado: obstáculo 48/57, giro 29/35, morro 33/36. Sin falsas en baches, casi-vuelcos, taludes ni llano.
Precauciones (talud ≥ 18° durante 2 s) en marcha real: 3 en 0,7 h de prueba.

Los «detectados» de la regla actual son poco más que suerte: dispara antes del vuelco en el 60 % de los positivos y en el 93 % de los negativos.

Parámetros entrenados:

| Parámetro | Valor |
|-----------|-------|
| `g_tol` | 0,24 g |
| `w_gate_dps` | 23 °/s |
| `ventana_s` | 0,9 s |
| `n_min` | 4 |
| `lim_lateral_deg` | 41° |
| `lim_frontal_deg` | 39° |
| `persistencia_s` | 1,07 s |

El entrenamiento prefirió no tener ni una falsa alarma real a detectar 11 vuelcos más (coste: 25 por falsa/h frente a 100 por cada 100 % de fallos).

## Qué significa y qué NO
- **Es un detector de «ya volcó»**, no un predictor. Corta motores y avisa ~2 s después de pasar el punto sin retorno. Evitar el vuelco sigue dependiendo de la velocidad y de los límites de giro (envolvente de `fallos_3d` / `montecarlo_firmware`).
- Los vuelcos son **simulados**. Las inercias del péndulo son hipótesis de `fallos_3d.py`, y el vuelco real del 2026-10-09 no se conservó en `~/.agrobot/datos`.
- Las falsas alarmas sí se midieron con datos reales, pero la marcha real total es poca (2,9 h): 0 falsas en 0,7 h de prueba no garantiza 0 en 10 h.
- El 14 % «no detectado» es en realidad **detección tardía**: medido después (sección v2), 125 de 128 vuelcos se detectan, 124 antes de 6 s. Solo 3 no se detectan nunca.

## Reentrenamiento v2 (mismo día, `v2/`)
Hipótesis: los vuelcos perdidos eran caídas que quedan apoyadas por debajo de 41°. Se probó un **segundo nivel** (inclinación total más baja sostenida más tiempo) y se reentrenó con CEM, 14 × 18 (`v2/resultado.json`, `v2/entrenamiento.log`).

- **No mejora:** con o sin segundo nivel, 109/128 en la ventana de 3 s, 0 falsas simuladas y 0 reales. El CEM se quedó con los parámetros de v1.
- La hipótesis era falsa. Los «perdidos» **se detectan, pero tarde** (3–7 s), porque los golpes de las ruedas, que siguen girando tras volcar, tapan la gravedad:

  | Detección tras el vuelco | Vuelcos (de 128) |
  |---|---|
  | ≤ 3 s | 109 |
  | ≤ 6 s | 124 |
  | Alguna vez | 125 |
  | Nunca | 3 |

- El segundo nivel se **quitó del código**: no aporta y sería un parámetro más sin datos que lo respalden. Los parámetros desplegados siguen siendo los de v1.
- Lo que sí acortaría la latencia: cortar los motores en cuanto hay PRECAUCIÓN y golpes seguidos, para que la gravedad se vuelva a ver. No está hecho; habría que simularlo con la dinámica de motores, que este modelo de péndulo no tiene.

## Cómo está montado
`jetson_server._bucle_vuelco` pasa cada paquete NUEVO del IMU al detector. `GET /api/vuelco` da el estado.
- Con `safety.vuelco_detector = true`:
  - en VUELCO para la autonomía y los motores, avisa por el dashboard y por la Pi, y se libera solo tras 3 s derecho;
  - el modo seguro deja de juzgar la inclinación con la lectura instantánea: solo recibe un vuelco confirmado.
- **Está APAGADO**: encenderlo es decisión del operador. Recomendación: encenderlo; con la regla vieja el modo seguro estaba inutilizable.

## Repetir
```bash
cd agrobot_system_2/simulador_pasivo
python3 vuelco_datos.py --extraer          # relee ~/.agrobot/datos (≈3 min)
nice python3 entrenar_vuelco.py            # ≈15 min con 3 procesos
```
