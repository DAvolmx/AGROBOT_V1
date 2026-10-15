# Auditoría de percepción y corredor seguro — 2026-10-14

Alcance: **la versión activa en la Jetson**, no los `.md` sueltos del repo ni el
workspace de Windows (`C:/Users/ddome/...`, no alcanzable desde la Jetson).

Rama: `mejoras-auditoria-20261007` · último commit `c14dd1c`

---

## 1. Qué está realmente implementado, probado y pendiente

### Lo que corre ahora

| Proceso | Estado |
|---|---|
| `jetson_server.py` | vivo |
| `lora_sensor_ble.py` | vivo |
| `agrobot_mapping.launch.py` | vivo: slam_toolbox async + rf2o + EKF + Nav2 (controller/planner/bt/behavior) |

**Incoherencia detectada:** `navigation.mode = "localization"` en la config,
pero el launch activo es el de **mapping**. Hay que decidir cuál manda.

### Qué modelo usa realmente el backend (objetivo 1)

Auditado el camino de inferencia real, no la documentación:

- **Hay una U-Net en la Jetson, pero NO en el camino de inferencia del
  backend.** Está en
  `simulador_pasivo/validacion_percepcion/modelo_agrobot_unet.pth` (27 MB) y
  solo la cargan los scripts de validación en simulador (`validar_sim.py`,
  `validar_kaggle_sim.py`). `jetson_server.py` y `camera.py` no la importan.
  **Las métricas de IoU 0,529 → 0,772 no describen el comportamiento del
  robot**, porque el robot no usa ese modelo.

  Auditoría del checkpoint (leído, no documentado de oídas):

  | | |
  |---|---|
  | arquitectura | `smp.Unet`, encoder `timm-mobilenetv3_large_100` |
  | canales de entrada | **6** (`encoder.model.conv_stem.weight` = `(16, 6, 3, 3)`) |
  | clases | **5** (`segmentation_head.0.bias` = `(5,)`): fondo, sendero_visible, zona_cultivo_no_pisar, bancal_borde_elevado, obstaculo_visible |
  | checkpoint | `OrderedDict` plano de 370 tensores |
  | normalización / resolución / nombres de clase | **NO están en el checkpoint** |

  **Hallazgo crítico para el objetivo 1: el preprocesado exacto no existe en el
  repo.** El checkpoint es un `state_dict` desnudo, sin `mean`/`std`, sin
  resolución, sin metadatos. Los 6 canales (RGB + a\* de LAB + ExGR + relieve)
  están **reconstruidos de memoria** en `validar_sim.py`, que lo declara
  explícitamente en su cabecera. Mientras falte el preprocesado original de
  Codex, **este modelo no es reproducible y no debe desplegarse**: un orden de
  canales o una normalización distintos degradan la segmentación sin avisar.
  Es lo primero que hay que pedir antes de tocar el objetivo 2.
- La **segmentación de suelo transitable activa es clásica**, en
  [`sendero.py`](../jetson/backend/sendero.py): Excess Green + Otsu con piso
  `EXG_MIN=0.06`, máscara de paja adaptativa, y un modelo de color del propio
  sendero tomado de la franja `REF_FILAS=(0.80, 0.94)` delante del chasis.
  Sin pesos, sin checkpoint, sin normalización de red.
- Lo único ML desplegado es **detección de plagas**: `ultralytics.YOLO` con
  `ip102_pest_yolo11s.pt`. Preproceso real: `modelo(frame, imgsz=480,
  verbose=False)` (letterbox interno de ultralytics, BGR→RGB, /255), umbral
  `confidence_threshold=0.6`. **Segmentación y plagas siguen separadas**, como
  se pidió.
- `politica.pt` (red de decisión del simulador) existe como opción y
  `politica_activa=True` en config, pero es un clasificador de 3 acciones, no
  percepción.

### Calibración: sin verificar

`calibration.servo_offset_deg = 0` y los tres `gyro_bias_*_dps = 0`.
`calibracion_camara.VERIFICADO = False`: los 15,5° de HOME y la ganancia
0,8°/° siguen siendo **estimaciones documentadas, no medidas**.

---

## 2. Fallos encontrados y corregidos

### A. El corredor aceptaba huecos por los que el robot no cabe

`sendero.py` validaba el ancho contra `ANCHO_MIN_M = 0.20` y
`slam.robot_ancho_cm` valía `35`. **Las dos por debajo de los 37 cm reales del
eje trasero.** El morro mide 19 cm: el detector se engancha a un hueco de
25–36 cm, el delantero pasa, y **el eje de atrás pisa la hilera**. Es
exactamente el modo de fallo de "seguir el centro de la mancha".

Corregido:

- Nuevo [`geometria_robot.py`](../jetson/backend/geometria_robot.py): fuente
  única de la envolvente (`ENTRE_EJES_M=0.37`, `ANCHO_DELANTERO_M=0.19`,
  `ANCHO_TRASERO_M=0.37`), con `cabe()` y `margen()`. Mismo patrón que
  `calibracion_camara.py` para que simulador y backend no vuelvan a divergir.
- `ANCHO_MIN_M` se queda en 0,20 **a propósito**: separa *plausibilidad de la
  medida* de *transitabilidad*. Un sendero de 25 cm es una medida real y hay
  que reportarla, no descartarla y seguir con un prior viejo.
- `VisionSendero` expone `cabe` y `margen_m` sobre el ancho **suavizado** (un
  fotograma suelto no declara intransitable un sendero).
- `SeguidorSendero.estrecho(cfg)` y parada en `explorador.py` con evento
  `sendero_estrecho`, contador `paradas_estrechas`.

### B. El tilt del servo no se compensaba en la proyección al suelo

`_geometria_camara()` sumaba el tilt del servo **solo si `barrido_tilt` estaba
activo**, y ese flag **no existe en la config activa**. Con el servo parado
fuera de HOME la proyección seguía asumiendo los 15,5° de HOME.

A servo 76° (el ángulo que se usa para mirar el sendero) la cámara mira 33,1°
abajo. Error de la distancia al suelo estimada, medido con el modelo documentado:

| fila de la imagen | asumiendo 15,5° | real a 76° (33,1°) | error |
|---|---|---|---|
| 0,60 | 1,43 m | 0,91 m | **+0,52 m** |
| 0,70 | 1,19 m | 0,82 m | +0,37 m |
| 0,80 | 1,03 m | 0,75 m | +0,28 m |
| 0,90 | 0,92 m | 0,69 m | +0,23 m |

El error lateral se mide `MIRADA_M = 0.6` adelante, así que ese factor de
escala falso contamina directamente la corrección de rumbo.

Corregido: `_tilt_servo_extra()` aplica **siempre** el delta del servo,
prefiriendo el ángulo confirmado por el actuador y cayendo al último ordenado
si la telemetría pasa de 1 s. **Ojo: la ganancia sigue sin verificar**; esto
sustituye un error grande y sistemático por uno pequeño y acotado, no lo elimina.

### C. "Sin referencia" se confundía con "centrado"

`correccion()` devolvía `None` tanto para *centrado* como para *no tengo
ninguna medida*, y el explorador caía al avance normal: **seguía recto a
ciegas**. Añadido `ciclos_sin_referencia` + `incierto(cfg)` y parada con evento
`sendero_incierto` tras `sendero_parada_incierta_ciclos` ciclos (exige una
racha, no un fotograma con un reflejo).

### D. Envolvente subestimada en todas las comprobaciones de holgura

`robot_ancho_cm = 35` se usa en 8 sitios como media carrocería, incluido el
hueco visual de suelo libre (`suelo_libre.hueco_delante`) y el costmap local.
Puesto a **37** (medido) en defaults y en la config activa.
`nav2_params.yaml`: footprint `±0.175` → `±0.185` en los dos costmaps.

---

## 3. Configuración (reproducible)

Claves nuevas en `autonomia`:

| clave | valor | qué hace |
|---|---|---|
| `sendero_parada_incierta_ciclos` | 5 | ciclos sin ninguna referencia antes de parar; 0 desactiva |
| `sendero_parar_si_no_cabe` | true | parar si el sendero medido no admite la envolvente |
| `sendero_holgura_lateral_cm` | 6 | holgura por lado; menos que `sendero_tolerancia_cm`/2 es pedirle al control precisión que no tiene |

Cambiada en `slam`: `robot_ancho_cm` 35 → **37**.

Copias previas: `agrobot_config.json.bak_envolvente_20261014` y, en el
scratchpad de la sesión, `jetson_server.py.pre_tilt`, `sendero.py.pre`,
`explorador.py.pre`, `nav2_params.yaml.pre`.

### Reversión

```bash
cd ~/Desktop/AGROBOT/agrobot_system_2/jetson/backend
cp agrobot_config.json.bak_envolvente_20261014 agrobot_config.json
# y en config: sendero_parar_si_no_cabe=false, sendero_parada_incierta_ciclos=0
```

---

## 4. Antes / después

Lo verificado son los **umbrales y las protecciones**, con 11 tests nuevos en
`tests/test_corredor_seguro.py` (84 pasan en total; el único fallo,
`test_enlace_c3::test_ignora_tramas_de_fuera_del_ap`, es previo y de otro módulo).

| | antes | después |
|---|---|---|
| ancho mínimo de corredor aceptado | 0,20 m | 0,49 m (envolvente 0,37 + 2×0,06) |
| envolvente usada en holguras | 0,35 m | 0,37 m (medida) |
| footprint Nav2 | ±0,175 m | ±0,185 m |
| tilt del servo en la proyección | ignorado salvo con `barrido_tilt` | aplicado siempre |
| error de distancia al suelo a servo 76° | +0,23…+0,52 m | el del modelo (sin verificar) |
| sin ninguna referencia de sendero | avanza recto | para tras 5 ciclos + evento |
| sendero medido más estrecho que el robot | sigue su centro | para + evento |

**No medido y por tanto no afirmado:** éxito de recorrido, error lateral real,
contactos, invasión de plantas, tiempo detenido y latencia. Esa tabla exige
recorridos instrumentados en el robot y **no se ha corrido ninguno**. Nada aquí
demuestra autonomía robusta.

---

## 5. Pendiente (objetivos 2, 3-parcial, 4, 7, 8)

1. **Calibración física del servo**: medir el cabeceo real a servo 25/98/110 y
   confirmar signo, ganancia 0,8°/° y los 15,5° de HOME. Hasta entonces B
   mejora pero no resuelve el objetivo 4.
2. **Biases del giróscopo y `servo_offset_deg`**: los cuatro a cero.
3. **Barrido de la huella en giro** con el pivote pasivo al tope: la
   envolvente de `geometria_robot` es la quieta. Al girar barre más.
   No lo inventé; falta medirlo.
4. **Dataset** (objetivo 2): etiquetar sendero / plantas protegidas /
   bancales / obstáculos, con separación de train y eval **por recorrido o
   sesión**. Requiere las fotos de `D:/ESCRITO TESIS/DATASET CASA/`, que no
   están en la Jetson.
5. **Recuperar de Codex el preprocesado exacto de los 6 canales** y la
   normalización. Sin eso, el objetivo 2 construye sobre arena.
6. **Ejemplos visuales de segmentación y corredor**: hay endpoint
   (`GET /api/sendero/vista.jpg`) y `sendero.dibujar()`, pero generarlos sobre
   las fotos reales necesita el dataset aquí.
7. **Reducir velocidad con curvas / poca confianza / datos viejos**
   (objetivo 6, parte): hoy el corte es binario. `slam.limitar_velocidad`
   documenta por qué (el firmware mapea cualquier velocidad a PWM 200–255, el
   78 % mínimo). Reducir de verdad exige tocar el firmware, y eso no se hace
   sin identificar la versión cargada y medirla.
8. **Maniobra de recuperación limitada y verificable** tras pérdida: ahora
   para y espera; no busca el sendero.
9. **Limpiar los 9.684 registros** (LiDAR hasta 11,93 s de antigüedad,
   velocidades imposibles) antes de alimentar cualquier aprendizaje.
10. **Nav2 sigue con el footprint viejo en memoria**: `install/` es symlink a
   `src/`, el fichero en disco ya está corregido, pero los costmaps cargaron
   los parámetros al arrancar. Hace falta reiniciar el stack — **no lo hice**.
