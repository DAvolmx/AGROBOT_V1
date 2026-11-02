# Autonomía robusta — 2026-11-02

Cuatro mejoras de software para que el robot siga solo más tiempo sin que el operador tenga que intervenir. Las cuatro tienen pruebas sin robot.
**Ninguna se ha probado todavía en el robot real.** Para cargarlas hay que reiniciar el backend (`sudo systemctl restart agrobot-backend`).

| # | Qué | Módulo | Interruptor (`autonomia.*` / `safety.*`) | Config viva |
|---|-----|--------|------------------------------------------|-------------|
| 1 | Vigía de la pose | `jetson/backend/vigia_pose.py` | `vigia_pose_activo` | encendido |
| 2 | Giro hasta un rumbo + fuerza aprendida | `giro_objetivo.py`, `pulse_drive.ConductorPulsos.w_giro` | `giro_objetivo_activo`, `giro_fuerza_aprendida` | encendidos |
| 3 | Atasco por pulso y salida escalonada | `atasco.py` | `atasco_escalado` | encendido |
| 4 | Detector de vuelco entrenado | `detector_vuelco.py` | `safety.vuelco_detector` | **apagado** (pendiente de tu OK) |

Por defecto (`config_store.DEFAULTS`) los cuatro están apagados, igual que el resto de cambios sin probar en el robot. Para volver al comportamiento de antes basta con poner el interruptor en `false`.

## 1. Vigía de la pose (`vigia_pose.py`)
Problema real: el 27-oct un salto del reloj (+711 s) hizo saltar la pose 30 y 24 m, y el EKF dio un pico de vx = −29,7 m/s. La misión siguió decidiendo con esa pose.

- La pose del mapa es `map->odom ∘ odom`. Mientras el «ancla» del SLAM (`map->odom`) cambie poco, se sigue al SLAM (estado **ok**). Un cierre de bucle de 30 cm se acepta.
- Si el ancla salta más de 0,5 m o 25° (o más de 0,15 m durante 20 s tras un salto de reloj), se **conserva el ancla anterior** y se navega con odometría (estado **anclada**).
- Vuelve a **ok** cuando el SLAM regresa a menos de 25 cm de la pose conservada. También vuelve si el SLAM se queda quieto 10 s en una posición nueva a menos de 1 m («se reubicó de verdad»).
- Un salto de la **odometría** (más de 1,2 m/s o 400 °/s, imposibles para este chasis) se absorbe: la pose servida no se mueve.
- Pasa a **perdida** si lleva 6 m o 180 s anclada, si el EKF está sin medidas (varianza de vx > 1 durante 2 s) o si no hay odometría. En ese estado:
  - la exploración y la misión esperan quietas (`explorador.bloqueos["localizacion"]`);
  - se pide ZUPT con el robot quieto;
  - en cuanto el SLAM vuelve, sigue sola, sin reiniciar ROS.
- Lo usan el explorador, la misión de plantas, la visión activa y las misiones (`get_pose=_pose_robusta`). La corrección manual de pose (`/api/training/correct_pose`) también reubica al vigía.
- Endpoint: `GET /api/localizacion` (estado, contadores, últimos eventos) y `POST /api/localizacion {"x","y","theta_deg"}`.
- Nav2 (patrulla) sigue usando los marcos de ROS: el vigía no la protege.

## 2. Giro hasta un rumbo (`giro_objetivo.py`)
Problema real: el explorador pedía «N pulsos de 12°» y daba por hecho que giraban. En el L (28-oct) los pulsos de 6° daban ≈1°; en exterior, 1,5° de mediana.

- En una esquina el explorador apunta al **centro del hueco libre** del scan, o a 30° hacia el lado elegido si no hay hueco claro, y gira hasta mirarlo. Tolerancia de 6°, tramos de lazo cerrado de hasta 30°, y tras cada tramo se mide el rumbo real con la pose.
- Un tramo que gira menos de 2° es un fallo. Dos fallos seguidos terminan el giro como «sin avance», que cuenta como atasco (punto 3).
- Sin IMU fresco, sin pose o con el predictor E11 en contra, se usan los pulsos de siempre.
- **Fuerza aprendida:** cada giro arranca con la fuerza (rad/s) que funcionó la última vez en ese sentido (`ConductorPulsos.w_giro`, entre 0,45 y 1,2). Antes cada giro empezaba en 0,6 y perdía medio segundo hasta que el lazo subía la fuerza.

## 3. Atasco por pulso (`atasco.py`)
Antes: atasco = la pose no se mueve 20 cm en 25 s; la salida siempre era la misma (atrás + 3 giros).

- Cada pulso se mide al mandar el siguiente. Hay atasco cuando:
  - 3 pulsos rectos seguidos avanzan menos de 3 cm (≈ 4–6 s);
  - 4 giros seguidos giran menos de 1,5°;
  - un giro por rumbo termina «sin avance».
- Salida escalonada. El nivel sube si se vuelve a atascar sin haberse alejado 50 cm:

  | Nivel | Maniobra |
  |-------|----------|
  | 1 | atrás + 30° al lado libre |
  | 2 | atrás ×2 + 60° al lado contrario |
  | 3 | balanceo adelante/atrás + 90° |
  | 4 | **se rinde**: la autonomía queda en espera y avisa |

- Los pulsos atrás o adelante solo se dan si hay hueco. Cuando el operador toma el mando y lo suelta (`continuar()`), se reintenta desde el nivel 1.
- El criterio viejo de 25 s sigue activo como red de seguridad.
- **A/B en el simulador del explorador** (`correr.py`, 150 s; el giro por rumbo y el vigía no existen ahí, solo actúa el atasco):
  - callejón y trampa en bucle: resultados idénticos (no llegó a atascarse);
  - invernadero, donde se atasca contra el follaje: contactos 100 → 71 y tiempo en contacto 59 → 28 s, pero **no logra salir** (2,1 m recorridos en ambos casos).

  La salida escalonada reduce el castigo; no resuelve ese escenario.

## 4. Detector de vuelco (`detector_vuelco.py`)
Gravedad = mediana de las muestras sin golpe en 0,9 s; vuelco = más de 41° de balanceo o 39° de cabeceo sostenidos 1,07 s; ignora los paquetes congelados; se libera al enderezarlo.
En la prueba con datos no vistos:

| | Vuelcos detectados | Falsas reales en marcha |
|---|---|---|
| Regla actual | 51/128 | 92,7/h |
| Entrenado | 110/128 | 0/h |

Detalle: `docs/VUELCO_DETECTOR_20261102/INFORME.md`. Endpoint `GET /api/vuelco`; interruptor `safety.vuelco_detector` (apagado).

## Pruebas
- `tests/test_vigia_pose.py` (15), `tests/test_autonomia_robusta.py` (20), `tests/test_detector_vuelco.py` (9).
- Batería completa del backend: 433 pruebas pasan. Se excluyeron `test_actuator_e2e.py` y `test_sensores_y_serie.py` (necesitan hardware o son scripts) y `test_scripts_como_pytest.py`.

## Qué mirar en la primera prueba real
- `GET /api/localizacion`: ¿pasa a «anclada» sin motivo en interior? Si lo hace, subir `VigiaPose(salto_m=...)`.
- Eventos `exploracion_event` con `giro_rumbo`: comparar `girado_deg` con `pedido`. También `w_giro` en el resumen del giro.
- Eventos `desatasco` con `nivel`: un nivel 1 frecuente con el robot avanzando bien indica que `min_recto_m` (3 cm) es alto para los pulsos cortos.

## Ronda 2 (mismo día): simulación fiel, fallas corregidas y A/B

### Diferencias del simulador con el robot (corregidas en `simulador_pasivo/correr.py`)
Todas las simulaciones del explorador anteriores a hoy tenían alguna de estas:

| Falla | Efecto |
|---|---|
| El `conducir` del simulador no aceptaba `esc_dur` | El explorador **reventaba** (TypeError) en el primer pulso de rumbo y la corrida seguía con el robot parado |
| Pulso de giro a tiempo fijo | El robot gira por giroscopio (`giro_lazo_cerrado`) |
| El explorador no recibía `scan_cb` | Hueco del giro, predictor E11 y sonar **apagados en la simulación** |
| La cámara se fusionaba siempre | Ignoraba `autonomia.camara_obstaculos` |

Nuevo: giro a rumbo conectado; `--aprendizaje RUTA` para que una corrida no contamine a otra (antes todas escribían en el mismo `maniobras.json`).

### Correcciones al software del robot
- **Bucle de giro compartido:** `giro_objetivo.girar_a_rumbo()` lo usan el backend y el simulador. Para si la pose y el giroscopio discrepan en más de 25° en un tramo; en la simulación, un cierre de bucle del SLAM le hizo girar 390° persiguiendo el rumbo.
- **El atasco se rinde solo si es físico:** pulsos que no mueven o un giro «sin avance». El criterio de 25 s («la pose no cambió») también salta cuando el robot no avanza porque *decide* no avanzar. En el primer A/B, esa rendición dejó al robot parado en 4 de 5 escenarios. Ahora vuelve al nivel 1.
- **Reintento:** tras rendirse, vuelve a intentarlo solo a los `autonomia.atasco_reintento_s` (90 s).

### A/B, 300 s por escenario, una semilla
Variantes:
- **antes:** los interruptores de hoy apagados;
- **nuevo:** encendidos;
- **sin cámara:** nuevo, más `camara_obstaculos` y `camara_como_lidar` apagados.

| Escenario | Recorrido (m) antes / nuevo / sin cámara | Pulsos adelante | Plantas pisadas |
|---|---|---|---|
| callejón | 3,6 / 2,9 / 5,1 | 4 / 3 / 42 | 0 / 0 / 0 |
| trampa en bucle | 4,8 / 4,2 / 2,3 | 9 / 10 / 12 | 0 / 0 / 0 |
| invernadero | 4,3 / 4,4 / **19,9** | 7 / 15 / 182 | 0 / 0 / 4 |
| serpentina | 6,5 / 1,5 / 3,7 | 18 / 2 / 36 | 5 / 2 / 2 |
| hileras | 3,6 / 2,9 / 9,7 | 5 / 2 / 96 | 0 / 0 / 0 |

Lectura honesta:
- **Lo que más frena la autonomía en la simulación es la cámara.** La paja del borde del sendero sale como obstáculo a ~25 cm y el explorador casi no da pulsos adelante. Esto pasa con el código de antes y con el nuevo.
- **Sin cámara**, el robot recorre mucho más, pero pisa plantas (4 en el invernadero). No cambié la config del robot: falta ver si la cámara real también confunde la paja.
- **Nuevo contra antes:** no hay una mejora clara en recorrido. Menos en serpentina, más celdas en el invernadero, más tiempo en el sendero en callejón y trampa. Con una semilla y la cámara frenando, la diferencia está dentro del ruido.
- El vigía de la pose no se mide en esta simulación, porque el SLAM simulado no tiene saltos de reloj.

### Vuelco v2
Ver `VUELCO_DETECTOR_20261102/INFORME.md`, sección v2. El segundo nivel no mejoró y se quitó. Los vuelcos «perdidos» se detectan tarde: 124 de 128 antes de 6 s.
