# Plan de mejora del AGROBOT — exploración autónoma y rendimiento

Fecha: 2026-10-07 · Autor del análisis: revisión de código + arquitectura
Objetivo del usuario: exigirle al máximo al robot con el software actual, mejorar la
exploración autónoma usando cámara + LIDAR + IMU y demás sensores.

> **Este es un plan, no cambios.** Nada se toca hasta que elijas el rumbo.

---

## 0. Veredicto corto

Tu sistema **real** (el que arranca `jetson_server.py`) está muy bien pensado y es
consciente del hardware. El problema para "exigirle al máximo" **no es que le falte
potencia: es que la exploración es un paseo reactivo sin cobertura garantizada y la
pose se va en deriva**. Y hay una **regresión de seguridad** en el árbol de trabajo
que hay que resolver antes de pedirle más.

"Máximo rendimiento" en este chasis **no es más velocidad** (el firmware ya manda
PWM 200–255 y las ruedas patinan 80 cm al parar): es **cubrir el campo entero, sin
perderse y sin repetir**, yendo lo más rápido que la seguridad permita. Ahí están las
ganancias reales.

---

## 1. ⚠️ ALERTA — resolver antes que nada

### 1a. Frenos de seguridad DESACTIVADOS en el árbol de trabajo (sin commitear)
En `jetson_server.py` (cambios del 2026-10-07, aún sin commit) se comentaron **tres**
capas de seguridad en `ActuatorController.send_command`:

1. **Freno por LIDAR** (`lidar_reactive.limitar_velocidad`) — el que corta el avance
   cuando hay algo más cerca que `margen_parada + deslizamiento`.
2. **Detector de patinaje** (`monitor_desliz.factor_velocidad`).
3. **Modo seguro** (`modo_seg.factor`).

Esto afecta a **todo** lo que mueve el robot: pulsos manuales, autonomía **y** Nav2.
Con 80 cm de deslizamiento al parar y sin freno por LIDAR, cualquier exploración
agresiva termina en golpe. Tu propia memoria del proyecto lo marca como línea roja
(rampa safety, deslizamiento, TF del LIDAR).

**Recomendación:** reactivar las tres, pero hacerlas **dependientes del modo**:
freno pleno en autonomía/Nav2; en manual, un "modo experto" opt-in que el operador
activa a sabiendas (no desactivado por defecto en el código). Así tenés control total
cuando lo pedís, sin dejar el robot sin frenos en autonomía.

### 1b. Código v3 muerto y contradictorio con el hardware
Estos archivos nuevos **no los importa nadie** (`grep` en `jetson_server.py` = 0):
`vision_system_v3.py`, `control_directo.py`, `slam_v2.py`, `path_detection.py`,
`camera_zoom.py`. Problemas:

- **`control_directo.py`** ("SIN BLOQUEOS") y **`PathDecision`** emiten `v`/`omega`
  **continuos**. Tu chasis **no sabe** ejecutar velocidades finas (PWM 200–255). Esto
  contradice toda la arquitectura de pulsos (`pulse_drive.py`) que diseñaste con
  mediciones reales. Si se conectara, el robot iría peor, no mejor.
- **`slam_v2.py`** promete "g2o + pose graph + triangulación" pero **nada de eso está
  implementado**: `pts_3d` nunca se puebla (el PnP jamás corre), la reproyección
  `camera_matrix @ [x,y,z,1]` es dimensionalmente incorrecta (3×3 × vector de 4), y el
  "loop closure" compara la **media** de descriptores ORB por coseno (no discrimina
  lugares). Produciría una pose basura. **No lo conectes.**
- Los `.md` tipo `BLOQUEOS_DESHABILITADOS.md`, `ELIMINAR_BLOQUEOS.md`,
  `SISTEMA_SIN_BLOQUEOS.md` documentan esa campaña de "quitar bloqueos". Son la raíz
  del punto 1a.

**Recomendación:** mover v3 a una carpeta `experimental/` (o borrar) para que no se
cuelen por error, y quedarnos con el stack real, que es bueno. Rescatar **solo** las
dos ideas buenas de v3: *zoom adaptativo de cámara* y *detección de sendero por HSV
con suavizado temporal* — pero adaptadas a pulsos, no a control continuo.

---

## 2. Puntos fuertes (lo que NO hay que tocar)

- **Arquitectura consciente del hardware.** `explorador.py` y `slam.py` razonan el
  porqué de cada decisión con experimentos fechados del simulador. Es ingeniería seria.
- **Seguridad en capas** (cuando está activa): freno LIDAR + cámara (`suelo_libre`),
  patinaje, modo seguro, rampas. La política propone, la seguridad dispone.
- **Control por pulsos** correcto para skid-steer sin gama lenta.
- **Aprendizaje del operador** (`aprendizaje.py`) + **política RL** (`politica.py`,
  14 features idénticas en sim y real). Diseño muy bueno: la misma entrada en
  simulación y campo, red diminuta (<1 ms en CPU), no toca la GPU de YOLO.
- **Detección reactiva por sectores LIDAR** robusta (historia de bugs ya resueltos:
  huella del robot, pasillos estrechos).
- **Puente ROS2 opcional** (`ros_bridge.py`) con `robot_localization` y Nav2 ya
  cableado — es la puerta a todo lo que sigue.
- **Simulador calibrado** con mundos reales (huerta, exterior) y Monte Carlo.

---

## 3. Falencias actuales (por subsistema)

| # | Subsistema | Falencia | Impacto |
|---|-----------|----------|---------|
| F1 | **Exploración** | Es un **paseo reactivo**: evita obstáculos y detecta bucles, pero **no garantiza cubrir todo el campo** ni evita repetir. No hay plan global de cobertura. | El robot "explora" pero puede dejar hileras sin recorrer y repetir otras. |
| F2 | **Pose / localización** | Dead-reckoning (v comandada) + gyro con filtro complementario. **Deriva** sin corrección externa. Sin GPS/RTK ni odometría LIDAR fusionada por defecto. | No se puede saber qué ya cubrió → imposible cobertura sistemática fiable. |
| F3 | **SLAM** | El 2D casero no cierra loops; el visual v2 está roto. | Mapa deriva en recorridos largos. |
| F4 | **Visión de sendero** | HSV + regresión de línea. Frágil con sombras/luz variable y sin geometría de hilera (punto de fuga). | Se descentra, pisa plantas en luz difícil. |
| F5 | **Fin de hilera / giro de cabecera** | No hay detección explícita de "se acabó la hilera" ni maniobra de *headland turn* para pasar a la siguiente. | En campo real no encadena hileras solo. |
| F6 | **Fusión de sensores** | IMU, LIDAR, visión y odometría de cámara se usan en silos; no hay un EKF que los una en una sola pose. | Cada fuente aporta pero ninguna corrige a la otra. |
| F7 | **Seguridad** | Desactivada en el árbol (punto 1a). | Riesgo físico. |

---

## 4. Plan de mejoras — por fases, priorizado

Orden pensado para que cada fase **habilite** la siguiente. Cobertura real (F1)
depende de pose fiable (F2/F6), así que la pose va primero.

### Fase 0 — Higiene y seguridad (rápida, imprescindible)
- Reactivar los 3 frenos como **dependientes de modo** (autonomía = freno pleno;
  manual experto = opt-in).
- Mover `*_v3.py` + `control_directo.py` a `experimental/`. Dejar nota en `MANIFEST`.
- Alinear el HSV del detector: `path_detection.py` default (H 30–90) **no coincide**
  con tu config (`sendero_verde_h 10–45`). Unificar con `CALIBRACION_HSV_TU_CULTIVO.md`.

### Fase 1 — Pose fiable y fusionada (la base de todo)  → ataca F2, F3, F6
- Levantar bajo ROS2 el trío estándar y fusionarlos con **`robot_localization` (EKF)**,
  que tu `ros_bridge.py` ya espera (`/odometry/filtered`):
  - **`rf2o_laser_odometry`** — odometría a partir del scan 2D (ya la usás según
    memoria; con `vx` invertido corregido).
  - **`imu_filter_madgwick`** — orientación estable del MPU6050.
  - **`slam_toolbox`** (2D LIDAR, modo mapping→localization) — cierre de loops de
    verdad, probado en Jetson. **Reemplaza a `slam_v2.py`**.
- Resultado: una pose global consistente que `explorador.py` puede usar para saber
  qué celdas ya cubrió (ya tiene el esqueleto: `_registrar_celda`, `_memoria`).
- *(Opcional hardware)* un **GPS** (aunque no sea RTK) da corrección de deriva y
  geocerca. Casi todos los robots de campo dependen de RTK; es el mayor salto posible.

### Fase 2 — Exploración = COBERTURA sistemática  → ataca F1, F5
Para un campo de cultivo, "explorar" bien no es vagar: es **cubrir en boustrophedon**
(ida y vuelta por hileras) o por frontera donde el terreno es desconocido.
- **Planificador de cobertura con `Fields2Cover`** (librería open-source hecha para
  agricultura): dado el polígono del campo y el ancho del robot, genera la ruta que
  recorre todas las hileras con los giros de cabecera. Integrable con Nav2 vía
  `opennav_coverage`.
- Adaptar la ruta a **waypoints gruesos** que tu `explorador`/`patrol` sigue con
  pulsos (no con velocidad continua): seguir la hilera con `sendero.py`, y al llegar a
  fin de hilera, ejecutar un *headland turn* como secuencia de pulsos aprendida.
- **Detección de fin de hilera**: combinar (a) el perfil lateral del `costmap_local`
  (las paredes/plantas laterales desaparecen) + (b) caída de confianza del sendero.
- Para zonas **sin estructura** (explorar un galpón, un terreno libre): añadir
  **frontier exploration** (`m-explore-ros2` / el frontier de Nav2) como modo aparte.
- Mantener bucle/atasco actuales como **capa reactiva** por debajo del plan.

### Fase 3 — Visión más robusta  → ataca F4
- **Sendero por geometría de hilera**, no solo color: detección de **punto de fuga**
  (líneas de las hileras convergen) → rumbo mucho más estable con sombra/sol. Es la
  técnica de los robots under-canopy (ver §5, CropFollow).
- **Segmentación de suelo transitable** ligera (clásica o un modelo chico) para
  alimentar la capa semántica del costmap — rescatando la idea de `SemanticCostmap`
  de v3 pero bien hecha.
- **Zoom adaptativo** (idea rescatable de `camera_zoom.py`): acercar para la detección
  YOLO de plaga, gran angular para navegar. Ya tenés el servo de tilt.
- Mantener el **freno por cámara** (`suelo_libre`) para lo bajo que el LIDAR no ve.

### Fase 4 — Autonomía que mejora sola  → potencia lo que ya tenés
- Cerrar el lazo **datos → entrenamiento → política**: ya recolectás datos
  automáticamente (memoria `agrobot-mejoras-navegacion`) y tenés supervisor OOM. Añadir
  features de la pose fusionada y de la geometría de hilera a las 14 entradas (subir
  `VERSION_ENTRADAS`), reentrenar, y desplegar con confirmación (ya lo pide).
- Métricas de cobertura por sesión (% de hileras recorridas, solape, m sin cubrir) en
  el dashboard, para medir "rendimiento" objetivamente y optimizar contra ellas.

---

## 5. Investigación: qué hacen otros robots (y de dónde copiar)

> Nota: la búsqueda web falló en esta sesión (error del buscador); esto viene de
> conocimiento del dominio hasta ene-2026. Vale la pena verificar versiones/enlaces.

### Robots de los que conviene aprender
| Robot | Tipo | Qué robar |
|-------|------|-----------|
| **EarthSense TerraSentia** | Investigación, *under-canopy*, pequeño | **Lo más parecido a vos.** Navega entre hileras con LIDAR 2D + cámara monocular y aprendizaje. Su stack **CropFollow / CropFollow++** es la referencia directa para F4/F3. |
| **Naïo (Oz, Dino, Ted)** | Industrial, desmalezado | Cobertura por hileras + RTK-GPS + LIDAR de seguridad. Modelo de *headland turns*. |
| **John Deere See & Spray (Blue River)** | Industrial | Visión por planta (YOLO-like) para tratar solo donde hace falta — tu YOLO de plaga va por ahí. |
| **Carbon Robotics LaserWeeder** | Industrial | Identificación de maleza por visión de alta precisión. |
| **Agrointelli Robotti / FarmWise / ecoRobotix / Bosch BoniRob** | Industrial | Arquitecturas de robot de campo: RTK + implementos + seguridad certificada. |
| **Twisted Fields "Acorn"** | **Casero / open-source** | Rover solar agrícola abierto; buenas ideas de mecánica y de "simple y robusto". |
| **FarmBot** | Casero (pórtico) | No móvil, pero su ecosistema de software abierto y cultivo por coordenadas es inspirador. |
| **OpenMower** | Casero | Cortacésped autónomo RTK open-source: cobertura + RTK barato (muy aplicable). |

**Patrón común que te falta y ellos tienen:** *pose absoluta* (RTK-GPS) + *cobertura
sistemática* en vez de reactiva. Ese es el verdadero salto.

### Librerías / repos concretos
| Herramienta | Para qué | Nota |
|-------------|----------|------|
| **Fields2Cover** | Planificación de **cobertura** agrícola (boustrophedon, giros de cabecera) | Núcleo de la Fase 2. C++ con bindings Python. |
| **Nav2 + `opennav_coverage`** | Servidor de cobertura sobre Nav2 (usa Fields2Cover) | Se integra con tu `ros_bridge`. |
| **`slam_toolbox`** | SLAM 2D LIDAR con loop closure | Reemplaza `slam_v2.py`. Probado en Jetson. |
| **`robot_localization`** | EKF/UKF que fusiona IMU + odom LIDAR + odom cámara | Tu `ros_bridge` ya lo espera. |
| **`rf2o_laser_odometry`** | Odometría del scan 2D | Ya en uso según memoria. |
| **`imu_filter_madgwick`** | Orientación estable del MPU6050 | Mejora la pose. |
| **`m-explore-ros2`** | Frontier exploration | Modo "explorar lo desconocido". |
| **RTAB-Map** | SLAM visual/visual-inercial en Jetson | Alternativa si querés mapa 3D/visual bien hecho (en vez del v2 roto). |
| **EarthSense CropFollow / CropFollow++** | Navegación entre hileras aprendida (monocular) | Referencia directa para F3/F4. |
| **Ultralytics YOLO** | Detección de plaga/maleza | **Ya lo usás** (ip102). Mantener. |
| **PhenoBench / CropAndWeed / Sugar Beets** | Datasets para entrenar visión de cultivo | Para mejorar el modelo. |

---

## 6. Qué necesito que decidas (antes de implementar)

1. **Seguridad (Fase 0):** ¿reactivo los 3 frenos como *dependientes de modo*
   (recomendado), o preferís otra política?
2. **Código v3:** ¿lo muevo a `experimental/` o lo borro? ¿rescato zoom + HSV?
3. **ROS2 (Fase 1):** ¿`slam_toolbox` + `robot_localization` ya están instalados en la
   Jetson, o hay que instalarlos? ¿querés que arranque por aquí?
4. **GPS:** ¿el robot puede llevar un GPS/RTK, o trabajamos solo con LIDAR+IMU+cámara?
   (Cambia bastante el techo de la cobertura.)
5. **Alcance de esta tanda:** ¿empezamos por Fase 0 (seguridad + limpieza, bajo riesgo,
   mejora ya) o querés ir directo a la exploración por cobertura (Fase 2, más grande)?

Decime 1–5 (o "arrancá por lo que recomiendes") y preparo la implementación de esa
fase, siempre con el simulador primero y sin reiniciar ROS sin avisar.
