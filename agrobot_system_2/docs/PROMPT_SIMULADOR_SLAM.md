# Prompt y datos de referencia: simulador de SLAM en Python (MRPT, Windows)

Documento de referencia para construir, **aparte y en Windows**, un simulador de SLAM en Python con MRPT cuyo
algoritmo se pueda **combinar con el de AGROBOT o subir al robot**. Todo lo que está aquí sale del repositorio
(archivos citados) o de mediciones del operador; lo que es **hipótesis** o **dato de memoria** está marcado.
Fecha: 2026-10-21. Rama de referencia: `mejoras-auditoria-20261007`.

> Nota sobre MRPT en Windows: las *bindings* de Python de MRPT (`pymrpt`) y sus *wheels* para Windows deben
> verificarse antes de empezar (versión de Python y de MRPT). Si no hay *wheel* utilizable, el simulador debe poder
> correr con NumPy puro y usar MRPT solo como referencia/comparación. El **algoritmo que sube al robot no puede
> depender de MRPT** si no compila en ARM64 (Jetson, Ubuntu 24.04): ver «Reglas de compatibilidad».

---

## 1. PROMPT (copiar y pegar)

```text
Eres un ingeniero de robótica. Construye un SIMULADOR DE SLAM 2D en Python (Windows, MRPT para Python cuando esté
disponible, NumPy/SciPy en otro caso) para un robot agrícola de tracción diferencial tipo skid-steer SIN encoders, con
LIDAR 2D RPLIDAR C1 e IMU MPU6050. El objetivo es desarrollar y validar un ALGORITMO DE SLAM (odometría por scans +
cierre de lazo + mapa de ocupación) que luego se integre al robot real (ROS 2 Jazzy, Jetson Orin Nano, ARM64) o se
combine con el slam_toolbox que ya corre allí.

Usa EXACTAMENTE los datos del documento adjunto (sección 2 en adelante): geometría, posición del LIDAR, modelo de
ruido, dinámica del robot, fallas, entornos y métricas. No inventes parámetros: si falta un dato, déjalo como parámetro
configurable con el valor y la incertidumbre indicados, y márcalo en el informe como HIPÓTESIS.

Entregables:
 1. Paquete Python `agrobot_slam_sim/` con: mundo (mapas con paredes y macetas), robot (cinemática/dinámica skid-steer con
    deslizamiento), sensores (LIDAR C1 con su ruido y fallas, IMU MPU6050 con sesgo), módulo de SLAM bajo prueba con una
    interfaz fija (ver «Interfaz») y un generador de escenarios reproducible por semilla.
 2. Implementación de referencia de odometría por scans (ICP / casamiento de scans) y de SLAM de grafo de poses con cierre
    de lazo; comparar contra MRPT (ICP, RBPF o graph-slam, lo que esté disponible) y contra una réplica de la configuración
    de slam_toolbox del documento.
 3. Batería de pruebas automática: los escenarios y las fallas del documento, N semillas, métricas ATE/RPE, IoU del mapa,
    tiempo por scan, y un informe (tablas CSV + imágenes PNG del mapa y la trayectoria estimada contra la real).
 4. Un módulo `algoritmo/` SIN dependencias de MRPT ni de ROS (solo NumPy/SciPy), con la interfaz fija, listo para
    envolverlo en un nodo rclpy (entrada /scan, /imu/data_raw, /cmd_vel; salida /map, TF map->odom, /odom).
 5. Pruebas unitarias (pytest) y criterios de aceptación del documento, reportados con números, sin maquillar fallos.

Reglas: determinismo por semilla; unidades SI; marco ROS (x adelante, y izquierda, ángulo + antihorario); todo en
español; si algo no se puede demostrar con el simulador, dilo.
```

---

## 2. Robot: dimensiones y geometría (croquis del operador, 2026-10-20)

Fuente: `jetson/backend/geometria_robot.py`, `simulador_pasivo/sim_logica_actual.py` (bloque GEOMETRIA).

| Dato | Valor | Estado |
|---|---|---|
| Cuerpo | **26 cm (ancho) × 32 cm (largo)** | croquis, aproximado |
| Pivote (módulo pasivo) | **TRASERO**, ruedas más separadas: **38 cm** de ancho exterior, libre ±**20°**, sin servo | croquis |
| Eje fijo | **DELANTERO**, estrecho: **28 cm** exterior, motores **más fuertes** | confirmado por el operador |
| Distancia entre ejes (wheelbase) | **0,37 m** | croquis |
| Llanta | ancho de rueda ~5–6,5 cm | estimado |
| Masa | ~**12 kg**, centro de gravedad ~0,20 m de altura | estimado |
| Tracción | 4 ruedas, skid-steer; ruedas [R1,L1,L2,R2], R1/L1 = delanteras (eje fijo) | firmware |
| Largo total (caras de ruedas) | **0,49 m**: 0,185 m por delante del LIDAR y 0,305 m por detrás | derivado |
| Footprint Nav2 | `[[0.185,0.19],[0.185,-0.19],[-0.305,-0.19],[-0.305,0.19]]` (x adelante, respecto al LIDAR) | `nav2_params.yaml` |

El **origen de `base_link` en el simulador es la posición del LIDAR**.

### Sensores y su montaje
| Sensor | Posición | Datos |
|---|---|---|
| **LIDAR** | **24 cm por delante del eje trasero** = 13 cm detrás del eje delantero = 5,5 cm por delante del centro del cuerpo; **37,5 cm sobre el suelo** (medida del operador: 35–40 cm) | ver sección 3 |
| **IMU MPU6050** | 5 cm sobre `base_link` (`IMU_XYZ = 0,0,0.05`) | solo se usa velocidad angular (yaw rate); sin magnetómetro |
| **Cámara** | webcam NexiGo con servo de inclinación (tilt) hasta 160° | no interviene en el SLAM actual |
| **Encoders** | **NO HAY** | la odometría sale del LIDAR |

> **Discrepancia a resolver:** `LIDAR_XYZ = ("0.0","0.0","0.18")` en `ros2_ws/src/agrobot_navigation/launch/agrobot_mapping.launch.py`
> está obsoleto (18 cm de altura, sobre el centro). Con el croquis serían ≈ 5,5 cm hacia adelante del centro y 0,375 m de
> altura. Si `base_link` se mantiene en el centro del cuerpo, esa TF estática hay que actualizarla; el simulador debe usar
> los valores del croquis y declararlo.

---

## 3. LIDAR: RPLIDAR C1 y cómo se usa

Configuración real (`agrobot_mapping.launch.py`, driver `sllidar_ros2`): `scan_mode: Standard`, `serial_baudrate: 460800`,
`angle_compensate: True`, `frame_id: laser`, **`yaw = π`** en la TF (el 0° del driver apunta hacia atrás del robot según la flecha
de la carcasa; medido el 2026-09-19).

Características del C1 (**de hoja de datos, de memoria: verificar antes de fijarlas**): láser DTOF 360°, alcance típico ~12 m
(blanco) y ~6 m (negro), ~5 000 muestras/s, ~10 Hz de giro (~500 puntos/scan crudos), exactitud del orden de ±30 mm, zona ciega
cercana de pocos cm. En el robot el driver entrega scans con compensación angular y el backend los trata como **360 rayos (1°)**.

Preproceso real (`laser_filters`, ya incluido en el launch):
- `/scan_odom` (para rf2o): rango **0,18–6,0 m** (lo demás → `inf`).
- `/scan_slam` (para slam_toolbox): rango **0,18–8,0 m** + **mediana espacial de ventana 3**.

### Modelo de ruido y fallas que debe reproducir el simulador
| Efecto | Valor / origen |
|---|---|
| Ruido de rango | gaussiano σ = **1 cm** (modelo del simulador actual `lidar_scan`); el real puede ser mayor: parametrizar 1–3 cm |
| Rayos más allá del alcance | `inf` |
| **Obstáculos bajos** | el plano del LIDAR está a 37,5 cm: **no ve objetos de menos de ~34,5 cm** (macetas de 15 cm son invisibles; sí ve paredes, columnas y macetas altas) |
| **Inclinación / cabeceo** | con suspensión de recorrido largo, ~2° de cabeceo hacen que el haz toque el suelo a ~`0,375/tan 2° ≈ 10,7 m` y rf2o lo lee como movimiento; por eso se recorta el rango |
| **Cortes del LIDAR** | 2–4 s sin scans, dos veces por episodio (falla `lidar_cortes`) |
| **Fantasmas** | retornos falsos cercanos delante (sol, reflejos): falla `lidar_fantasmas` |
| **Vidrio** | un panel que el LIDAR atraviesa (`pared_vidrio`) |
| **Reflejos / patas de mesa** | no modelados hoy: añadir como opcionales |
| **Caídas del USB** | el C1 va por adaptador CP2102N; el driver puede quedar colgado y el puerto reenumerarse: simular «sin scans hasta el reinicio» |
| **Retardo** | scan a 10 Hz, mando a 20 Hz |

---

## 4. Dinámica del robot y el origen de los errores de odometría

**No hay encoders.** La traslación real la aporta solo rf2o (casamiento de scans); el giro, el giroscopio.
Fuente: `ros2_ws/src/agrobot_navigation/config/ekf.yaml`.

- **Comando:** velocidades `v` máx. **0,55 m/s** y `ω` máx. **1,5 rad/s** (`V_SCALE`, `W_SCALE`). Límites de Nav2: **0,4 m/s y 1,0 rad/s**
  (`nav2_params.yaml`); límites del backend por config: v 0,25 m/s, ω 0,6 rad/s. El robot **no mide** lo que hizo: la odometría de ruedas
  `/wheel_odom` es **la velocidad COMANDADA**, no medida.
- **Skid-steer con deslizamiento:** el giro se hace invirtiendo el sentido de un lado; hay arrastre lateral grande. Zona muerta de motores
  (duty por debajo del cual no se mueven): **0, 25 o 40 %** (hipótesis, se sortea). Fricción μ **0,65–0,95**. Rodadura viscosa 0,13–0,22.
- **Medido por el operador (2026-10-21, robot real, firmware actual):**
  - Un toque de **giro** gira **~10°** y avanza **~10 cm**; un toque de retroceso va **~10 cm** hacia atrás.
  - En un pasillo gira **«como estacionar un auto»**: adelante–atrás–giro, maniobrando. **No gira en el sitio.**
  - Medida antigua (firmware viejo): 26,2 ± 2,2 cm por pulso recto de 1 s a escala 0,4.
- **Pulsos:** el manejo manual/automático es por **pulsos** de 0,25–1,0 s (modo `preciso` 0,25 s giro/0,4 s recto; `interior` 0,4/0,8; `exterior` 0,35/1,0).
  Hay un conductor continuo (`control_pasillo.py`) para los pasillos.
- **Imperfecciones de chasis** (sorteadas por semilla, `robot_rustico.py`): chasis soldado y algo desalineado; *toe* por rueda ~N(0, 1,5°);
  fuerza por rueda ±8 %; pivote tosco con rozamiento; sin amortiguación ideal.
- **IMU MPU6050:** 20 Hz hacia el EKF, **sin magnetómetro** (la orientación absoluta se ignora), sesgo/deriva del giroscopio **no medidos**
  (parametrizar sesgo 0,2–1 °/s y ruido blanco; hipótesis). Recibe vibración del chasis.
- **Calibración:** `calibrated = false` en toda la física. Todo el simulador de movimiento actual es hipótesis ajustada a un solo dato real.

---

## 5. Pila de SLAM actual (lo que hay que igualar o mejorar)

Fuente: `ros2_ws/src/agrobot_navigation/{config,launch}`.

| Pieza | Detalle |
|---|---|
| Odometría por scans | **rf2o_laser_odometry**, 10 Hz, sobre `/scan_odom`; publica `/odom_rf2o`; solo se fusiona `vx` |
| Fusión | **robot_localization (EKF)**: `/odom_rf2o` (vx), `/imu/data_raw` (yaw rate), `/wheel_odom` (giro comandado, degradado); salida `/odometry/filtered` |
| SLAM | **slam_toolbox**, `async_slam_toolbox_node`, solver **Ceres** (SPARSE_NORMAL_CHOLESKY, LM) |
| Parámetros clave | resolución **0,05 m**, `max_laser_range` 8,0, `minimum_time_interval` 0,1 s, `map_update_interval` 0,5 s, `minimum_travel_distance` 0,12 m, `minimum_travel_heading` 0,15 rad, scan matching activo, búsqueda de correlación 0,6 m / res. 0,01 / smear 0,06, `link_match_minimum_response_fine` 0,08, `link_scan_maximum_distance` 2,0, **cierre de lazo activo** (`loop_search_maximum_distance` 4,0; espacio 8,0 m, res. 0,05, smear 0,03) |
| Marcos | `map` → `odom` (slam_toolbox), `odom` → `base_link` (EKF), `base_link` → `laser` / `imu_link` (estáticos) |
| Navegación | Nav2 (planner, controller DWB, bt_navigator, behavior) sobre `/map`; modo **localización** con mapa guardado `maps/agrobot_map` |

Debilidades conocidas (el simulador debe poder reproducirlas):
1. rf2o es la **única** medida de avance: en campo abierto o con polvo se queda sin estructura y el avance se degrada.
2. Cabeceo → suelo leído como movimiento (ver sección 3).
3. Caídas del LIDAR/USB, **saltos del reloj** del sistema (sin RTC) y EKF sin medidas: causan la mayoría de los «SLAM caído».
4. Pasillos largos y uniformes (45–90 cm de ancho): el casamiento de scans es degenerado a lo largo del eje (deslizamiento en la dirección del pasillo).
5. Giros pequeños e inconsistentes: cada toque de giro mueve el robot ~10 cm, lo que rompe la hipótesis de movimiento suave del scan matching.

---

## 6. Entornos y escenarios (usar los mismos del simulador de manejo)

Fuente: `simulador_pasivo/sim_logica_actual.py` (`escenarios_interior`) y `simulador_pasivo/pruebas_geometria.py` (`escenarios_extra`).

- **Macetas:** círculos de radio **0,12 m**; alturas bajas (15 cm, invisibles) y altas (visibles). Hileras con separación de 45 cm entre macetas.
- **Pasillos entre hileras:** paso libre **50, 60 y 75 cm** (el robot mide 38 cm de ancho en las ruedas traseras).
- **Escenarios:** `sala_vacia` (2,6 m, sala 3 m de ancho), `pasillo_macetas`, `eslalon_macetas` (macetas alternas), `curva_pasillo` (pasillo de 90 cm, curva de 60° a la izquierda, R 1,6 m),
  `recto_tope` (6 m en pasillo de 1,4 m, a toda velocidad), `sala_cuadrada` (3×3 m con maceta al centro, de una esquina a la opuesta),
  `hileras_cuadricula` (tres hileras, cuadrícula de 45 cm), `zigzag_hileras` (tres hileras con un hueco de 50–60 cm que alterna de lado).
- **Entornos reales a cubrir:** interior (habitaciones, pasillos, mesas con patas finas, vidrio) y exterior/huerta (hileras de plantas, sin paredes cercanas, terreno irregular).
  El foco es **interior**.

## 7. Fallas del sistema a simular

Catálogo de `simulador_pasivo/episodio_fallas.py` (cada una con sus escenarios): `ninguno`, `maceta_baja`, `maceta_baja_grupo`, `pared_vidrio`,
`barro`, `rueda_muerta`, `motor_debil` (40 %), `bateria_cae` (100 → 45 %), `zona_muerta_alta` (55 %), `suelo_resbaloso` (μ 0,35), `lidar_cortes`,
`lidar_fantasmas`, `enlace_cortes` (1–3 s, el firmware frena a los 0,5 s), `persona_cruza`, `paso_estrecho`, `esquina_inicio`.
Además: **paradas para escanear** (8 s cada 45 s y ante cada planta), choques con respuesta física, atascos (menos de 12 cm de desplazamiento en 10 s) y
**secuencias de recuperación** (maniobras aprendidas por firma de LIDAR). Para SLAM, las más importantes: `lidar_cortes`, `lidar_fantasmas`,
`pared_vidrio`, `suelo_resbaloso`, `rueda_muerta`, `barro`, y los choques/atascos (el scan matching ve el robot «quieto» mientras las ruedas giran).

---

## 8. Interfaz fija del algoritmo (para que sea portable)

```python
class SlamAlgoritmo:
    def __init__(self, config: dict): ...
    def reset(self, pose0=(0.0, 0.0, 0.0)): ...
    def scan(self, t: float, rangos: "np.ndarray[360]", angulo_min: float, angulo_inc: float,
             rango_min: float, rango_max: float) -> None: ...
    def imu(self, t: float, gyro_z: float, accel_xy=(0.0, 0.0)) -> None: ...
    def comando(self, t: float, v: float, w: float) -> None: ...           # velocidad COMANDADA, no medida
    def pose(self) -> tuple[float, float, float]: ...                      # pose en `map` (x, y, yaw)
    def odom(self) -> tuple[float, float, float]: ...                      # pose en `odom`
    def mapa(self) -> tuple["np.ndarray", float, tuple[float, float]]: ... # grilla int8 (-1,0..100), resolución, origen
```

Convenciones: marco ROS (x adelante, y izquierda, yaw antihorario), unidades SI, tiempo en segundos, scan con 0° al frente del robot **ya corregido**
por el `yaw = π` del montaje del LIDAR.

## 9. Reglas de compatibilidad con el robot

- **Jetson Orin Nano, ARM64, Ubuntu 24.04, ROS 2 Jazzy, Python 3.12**; 6 núcleos a 1,5 GHz y **7,5 GB de RAM compartidos con la GPU**; el backend, Nav2 y el
  entrenamiento comparten esa máquina. El SLAM debe procesar un scan en **< 50 ms de media** con un solo núcleo.
- El módulo `algoritmo/` usa solo **NumPy/SciPy**; nada de MRPT, Windows ni rutas absolutas.
- Entradas/salidas ROS 2 equivalentes a las de slam_toolbox: `/scan_slam` (o `/scan`), `/imu/data_raw`, `/cmd_vel`; publica `/map` (OccupancyGrid 0,05 m), TF `map→odom` y `/odom`.
- Debe poder **ejecutarse junto a slam_toolbox** (comparar poses) y **reemplazarlo** cambiando el nodo en `agrobot_mapping.launch.py`.
- Nada se despliega al robot sin decisión del operador.

## 10. Métricas y criterios de aceptación

Por escenario y por falla, N ≥ 10 semillas, física **sorteada** como en el simulador de manejo (μ, zona muerta, deriva, ruido):

| Métrica | Criterio propuesto |
|---|---|
| ATE (RMSE de posición, tras alinear) | ≤ 10 cm en 20 m de recorrido interior |
| RPE a 1 m | ≤ 3 cm y ≤ 2° |
| Error de cierre de lazo | ≤ 15 cm al volver al punto de inicio |
| IoU del mapa de ocupación vs. el mapa real | ≥ 0,85 (paredes) |
| Tiempo por scan (1 núcleo ARM) | < 50 ms de media, < 120 ms en el peor caso |
| Recuperación tras corte del LIDAR (2–4 s) | no diverge: error < 20 cm tras reanudar |
| Fantasmas / vidrio | no deja obstáculos fantasma permanentes en el mapa |
| Reporte honesto | cada criterio con su número y sus fallos; lo no demostrable se declara |

## 11. Archivos de referencia en este repositorio

- Config del SLAM: `ros2_ws/src/agrobot_navigation/config/{slam_toolbox_mapping,slam_toolbox_localization,ekf,nav2_params}.yaml`
- Lanzadores: `ros2_ws/src/agrobot_navigation/launch/agrobot_mapping.launch.py`, `agrobot_navigation.launch.py`
- Geometría: `jetson/backend/geometria_robot.py`, `simulador_pasivo/sim_logica_actual.py` (LIDAR, escenarios, firmware, `Hip`)
- Fallas y escenarios extra: `simulador_pasivo/episodio_fallas.py`, `simulador_pasivo/pruebas_geometria.py`
- Chasis rústico y dinámica: `simulador_pasivo/robot_rustico.py`, `simulador_pasivo/dinamica_pasiva.py`
- SLAM propio del backend (referencia de la versión sin ROS): `jetson/backend/slam.py`, `jetson/backend/scan_mapa.py`
- Notas de fallos recurrentes: `docs/AUDITORIA_AUTONOMIA_2026-10-20.md`
