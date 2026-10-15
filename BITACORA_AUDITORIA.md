# Bitácora de auditoría y mejoras — AGROBOT

Rol: ingeniería sénior de robótica móvil (ROS 2 Jazzy, Nav2, ESP32, validación de tesis).
Inicio: 2026-10-07. Rama de trabajo: `mejoras-auditoria-20261007` (creada desde `main`,
conserva los cambios locales sin commitear).

> Registro vivo para poder continuar si se corta la sesión. Cada cambio queda con
> evidencia, respaldo y cómo revertir. **No se accionó hardware. No se reinició ningún
> servicio. No se flasheó nada.**

---

## 0. Andamiaje de seguridad (hecho)
- **Rama**: `git switch -c mejoras-auditoria-20261007` (los cambios previos sin commit
  se conservan en el árbol de trabajo).
- **Respaldos verificables** en `respaldos/auditoria_20261007/`:
  - `dataset.csv.bak` — sha256 idéntico al original verificado.
  - `entrenar.py.bak`.
- **Revertir**:
  - Dataset: `cp respaldos/auditoria_20261007/dataset.csv.bak ~/.agrobot/aprendizaje/dataset.csv`
  - entrenar.py: `git checkout agrobot_system_2/simulador/entrenar.py` (o desde el .bak)
  - Volver a main sin perder nada: `git switch main` (la rama queda con los cambios).

---

## 1. Trabajo COMPLETADO y verificado en esta sesión (reversible)

### 1.1 Fusión de entrenamiento simulación + real  ✅ aplicado
- **Qué**: las demostraciones reales del robot (`~/.agrobot/datos/<fecha>/telemetria.csv`,
  de `grabador.py`) no entraban al entrenamiento, que solo usaba el piloto ideal del
  simulador. Nuevo conversor `agrobot_system_2/simulador/fusion_real.py` las traduce a
  las 14 features de `politica.ENTRADAS` (las de sendero van None: no se graban) con
  origen `real_manual`/`real_auto`.
- **Evidencia**:
  - Dataset: 293.873 → 313.473 filas (+19.600 reales). Integridad: 100% de filas con
    17 columnas (verificado con awk). Orígenes: ideal*, +19.600 `real_manual`.
  - Brecha sim→real medida (dry-run): el piloto ideal va `adelante` 74.8% y **nunca
    retrocede**; el operador real: adelante 39.7%, mucho más giro, 9% `atras`.
  - `entrenar.py`: `ORIGENES` ahora incluye `real_manual`/`real_auto` (líneas 60 y 360).
  - Entrenamiento de validación (12 épocas, GPU): 498.856 ejemplos, **`atras=3546`
    (antes 0)** → confirma que lo real entra. Acierto 81.7% (equilibrado 82.3%).
    Escribió `politica_candidata.pt` — **NO** `politica.pt`: el robot no cambió.
- **Archivos**: `fusion_real.py` (nuevo), `entrenar.py` (ORIGENES), `dataset.csv` (append).
- **Pendiente**: desplegar el candidato es decisión manual del dashboard ("Desplegar el
  mejor"); conviene antes `--evaluar` con/sin política. Nota honesta: la FSM/explorador
  solo aplica la política para adelante/izq/der (ignora `atras`), así que el aporte real
  mejora sobre todo la diversidad de giros, no la marcha atrás autónoma.

### 1.2 Máquina de estados de exploración  ✅ construida y probada en sim
- `agrobot_system_2/jetson/backend/maquina_estados.py` (`MaquinaEstados`): FSM explícita,
  drop-in de `ExploradorReactivo` (misma firma `__init__`, verificado). Estados con
  nombre + log de transiciones (telemetría para el entrenamiento), ganchos de escaneo de
  planta al vuelo, sesgo de cobertura conservador.
- **Cableada conmutable** en el simulador: `AGROBOT_FSM=1` (`correr.py`).
- **A/B en sim** (semilla 1): tras acotar el sesgo de cobertura (`COBERTURA_EMPATE_M`)
  para que solo desempate con ambos lados despejados y parejos → **sin regresión** vs el
  explorador viejo (mismos choques/pisadas en hileras, trampa_bucle, campo_abierto).
- **Pendiente**: cablearla en `jetson_server.py` (robot real). La ganancia grande de
  cobertura pide planificador sistemático (Fields2Cover), no el sesgo reactivo.

---

## 2. Diagnóstico priorizado (Fase 1) — con evidencia

Categorías: **[CONFIRMADO]** defecto reproducible · **[CONDICIONADO]** riesgo según
estado · **[FÍSICO]** requiere medición en el robot.

### Arquitectura real observada (2026-10-07)
- **ROS 2 Jazzy** corriendo con el stack COMPLETO y montado (no era "por construir"):
  `sllidar_ros2` + `rf2o_laser_odometry` + `robot_localization/ekf_filter_node` +
  `slam_toolbox` + Nav2 (`bt_navigator`, `controller_server`, `planner_server`,
  `behavior_server`, costmaps). 69 topics; `/odometry/filtered`, `/imu/data_raw`,
  `/map`, `/cmd_vel`, `/odom_rf2o`, `/zupt`, `/wheel_odom` presentes.
  Fuente: `agrobot_system_2/ros2_ws/src/agrobot_navigation/` (launch + config).
- **EKF** (`config/ekf.yaml`): fusión madura de 5 fuentes (rf2o vx, IMU yaw-rate,
  wheel_odom giro comandado, ZUPT, hueco para odom visual). Aceleración correctamente
  deshabilitada (documenta el bug de deriva de 5,8 km). **No necesita
  `imu_filter_madgwick`** (no usa orientación absoluta; no hay magnetómetro). Corrige mi
  recomendación previa.
- **GPS**: no hay (Jetson Orin Nano, sin GNSS; sin módulo USB).

### Hallazgos (verificados contra las pistas de la auditoría previa)
1. **[CONFIRMADO] Frenos de seguridad comunes desactivados** en `jetson_server.py`
   (`ActuatorController.send_command`, ~línea 548): comentados el freno por LIDAR
   (`lidar_reactive.limitar_velocidad`), el factor de patinaje (`monitor_desliz`) y el
   modo seguro (`modo_seg.factor`), más `monitor_desliz.on_orden`. Afecta a la salida
   COMPARTIDA (manual + Nav2 + patrulla), no solo al manual. Evidencia: `git diff` en el
   árbol de trabajo. **Acción BLOQUEADA por decisión del usuario**: no reactivar hasta
   validar navegación autónoma y manual. Pendiente de Fase 2 (supervisor con freno final
   común dependiente de modo).
2. **[CONDICIONADO] `control_directo.py`, `slam_v2.py`, `vision_system_v3.py`,
   `path_detection.py`, `camera_zoom.py` = código v3 muerto**: no los importa
   `jetson_server.py` (grep = 0). `slam_v2` no implementa lo que su docstring promete
   (PnP sin puntos 3D, reproyección 3×3·vec4, loop closure por media de descriptores).
   `control_directo`/`PathDecision` emiten velocidad continua que el chasis no ejecuta.
   Riesgo: que alguien los conecte. Recomendación: mover a `experimental/` (reversible).
3. **[CONFIRMADO] Tres límites de velocidad distintos** sin unificar (pista #4):
   backend `max_v_mps=1.5`, `max_omega_rps=4.0` (agrobot_config.json); Nav2
   `max_vel_x=0.4`, `max_vel_theta=1.0` (nav2_params.yaml:67,71); firmware `V_MAX=0.55`,
   `OMEGA_MAX=1.5`. El firmware recorta físicamente a 0.55/1.5, así que el 1.5/4.0 del
   backend es ficticio. Pendiente de Fase 3 (unificar + marcar provisional).
4. **[CONFIRMADO] Emergencia no enclavada** (pista #2): `emergency_stop()`
   (`jetson_server.py:591-606`) resetea rampa y objetivo a 0 y manda el paquete de
   parada, pero NO deja ningún latch. `_ros_on_cmd_vel` (`:633-637`) llama
   `send_command` sin comprobar nada, y `send_command` (`:505-514`) tampoco → el
   siguiente `/cmd_vel` de Nav2 (o cualquier productor) vuelve a fijar objetivo y
   `tick()` acelera de nuevo. Mitigado en diseño por el nuevo supervisor (ver §1.3).
5. **[CONFIRMADO] `patrol._reinit_pose_slam`** (pista #5): `patrol.py:468-475` —
   docstring dice "reinicia la pose en AMCL" pero el cuerpo llama
   `self._send_nav_goal(x, y, theta)`: es una META de navegación, no un reinicio de
   localización (que sería publicar en `/initialpose`). Además el stack usa
   `slam_toolbox`, no AMCL. Se dispara al iniciar patrulla (`:202`) y puede competir
   con las metas del propio `_bucle`. Corrección pendiente (quitar la meta o publicar
   `/initialpose`).
6. **[CONFIRMADO] `current_pose()` sin chequeo de antigüedad** (pista #3):
   `jetson_server.py:738-742` devuelve `ros_fused_pose` si existe, SIN mirar timestamp;
   `_ros_on_fused_pose` (`:692-700`) guarda la última sin edad. Si el EKF deja de
   publicar, `current_pose()` retorna una pose CONGELADA indefinidamente. (El EKF tiene
   ZUPT interno, pero el consumidor del backend no se protege.) Corrección pendiente
   (sellar con monotonic y marcar la pose como caduca pasado un umbral).
7. **[FÍSICO] Velocidad real vs consigna, distancia de frenado, giros skid-steer**: no
   medidos. Requiere protocolo físico autorizado (Fase 7).
8. **[CONDICIONADO] Salud sin telemetría de ESP32-C3 / ESP32 LoRa** (pista #9): verificar
   si `/imu/data_raw` y el estado del C3 llegan ahora (el IMU publica, lo que sugiere que
   el C3 o su puente sí reporta). Pendiente de confirmar enlaces.

### 1.3 Código v3 archivado + supervisor de movimiento (Fase 2)  ✅ hecho y probado
- **v3 muerto movido** a `agrobot_system_2/jetson/backend/experimental/` (5 módulos:
  control_directo, slam_v2, vision_system_v3, path_detection, camera_zoom), con
  `experimental/README.md` y restauración documentada (`mv experimental/*.py .`).
  Verificado que nadie los importa en el repo. `jetson_server.py` intacto.
- **Supervisor de movimiento** nuevo: `supervisor_movimiento.py` — núcleo de decisión
  PURO (sin HW/ROS/asyncio). Un único propietario del mando; estados explícitos
  (DETENIDO/MANUAL/NAVEGACION/EXPLORACION/INSPECCION/RECUPERACION/EMERGENCIA);
  emergencia ENCLAVADA con rearme por salud; rechazo de órdenes caducas y de productores
  sin mando; manual arrebata autonomía y al soltar NO se reanuda sola; límites físicos
  unificados (0.55/1.5). **NO** frena por obstáculos (capa aparte, sigue bajo control del
  usuario) → no viola el bloqueo de frenos.
- **Tests**: `tests/test_supervisor_movimiento.py` — 10/10 PASAN (orden tras emergencia,
  rearme con salud, comando tardío/sin mando, manual arrebata, soltar no reanuda, orden
  caduca, límites, parar siempre pasa). Puros, reloj controlado, no abren hardware.
- **Pendiente (gated)**: CABLEARLO en `jetson_server.py` como puerta final común
  (`_aplicar`, `_ros_on_cmd_vel`, pulse_drive) — toca la ruta de movimiento real y
  conviene hacerlo junto con la reactivación de frenos, ambos bajo tu validación.

### 1.4 Correcciones aplicadas (2026-10-07, reversibles, sin HW)  ✅
Respaldos añadidos: `respaldos/auditoria_20261007/jetson_server.py.bak`, `patrol.py.bak`
(sha256 verificado). Revertir: `git checkout <archivo>` o copiar del `.bak`.

- **`current_pose()` con sello de antigüedad** (hallazgo #6): `_ros_on_fused_pose` ahora
  guarda `ros_fused_pose_t` (monotónico). Nuevos `pose_edad_s()` y `pose_confiable()`
  (umbral `POSE_VIEJA_S=1.5`). El snapshot del dashboard expone `pose_edad_s` y
  `pose_confiable`. `current_pose()` sigue devolviendo la última pose (continuidad de
  display), pero ahora se puede saber si está CONGELADA. *Pendiente* (siguiente paso):
  que los modos dependientes de localización consulten `pose_confiable()` y paren si es
  False. Cambio ADITIVO: no rompe a los 6 consumidores de `current_pose()`.
- **`patrol._reinit_pose_slam` corregido** (hallazgo #5): se eliminó la meta de
  navegación espuria al iniciar patrulla (`patrol.py:~198`) y el método quedó como
  no-op documentado. Ya no hay carrera de goals ni "reinicio de AMCL" inexistente
  (corre slam_toolbox). Un reinicio real de pose sería `/initialpose`, no un goal.
- **Supervisor cableado como protección final común, OFF por defecto** (Fase 2):
  `jetson_server.py` instancia `supervisor` (límites físicos 0.55/1.5) y en `_aplicar`
  —único punto por el que pasan manual, Nav2, patrulla y exploración— pasa la orden por
  `supervisor.proteccion_final(v, omega)` **solo si `safety.supervisor_activo` es true**.
  `emergency_stop()` enclava el supervisor (también solo con el flag ON). Nuevo método
  `proteccion_final` (emergencia + límites, sin depender del plumbing de propiedad) con
  2 tests más → **12/12 pasan**. Sintaxis de los 4 archivos verificada con py_compile.
  - **Activar para validar**: añadir `"supervisor_activo": true` en la sección `safety`
    de `agrobot_config.json`. Hoy NO reduce velocidades de Nav2 (0.4<0.55); solo recorta
    al tope físico y enclava la emergencia. **Pendiente para que el enclavamiento sea
    útil con el flag ON**: un control de REARME (`supervisor.rearmar(salud_ok)`) en el
    backend/dashboard, y plumbing de propiedad por productor si se quiere `filtrar`
    completo (rechazo por mando/caducidad). Esto es el trabajo de validación, con tu OK.
  - **NO** reactiva el freno por LIDAR (sigue comentado, bajo tu decisión).

### 1.5 Preparación para prueba de manejo en campo (2026-10-07)  ✅
Respaldo añadido: `respaldos/auditoria_20261007/agrobot_config.json.bak`,
`jetson_server.py.bak`, `patrol.py.bak`.

- **Inversión del giro por software** (pedido: "digo izquierda y va a la derecha"):
  nuevo `ActuatorController._giro_signo()` que niega omega SOLO al enviar a la C3
  (`self.link.send_command`, en `_aplicar` y `send_actuator_update`), gobernado por
  `control.invertir_giro` (puesto en **true** en `agrobot_config.json`). NO se toca el
  omega lógico (odometría/EKF/convención ROS omega>0=izquierda): es una corrección de
  cableado en el borde del actuador. Beneficio extra: antes `wheel_odom` (lógico) y el
  giroscopio tenían signos opuestos; ahora coinciden. Revertir: poner `invertir_giro`
  en false (sin reiniciar código).
- **Recopilación de datos**: ya existía y queda confirmada. El `Grabador`
  (`grabador.py`) arranca con el backend (`start_background`) y graba TODO a 10 Hz en
  `~/.agrobot/datos/<fecha>/telemetria.csv` (incluye `pulso_accion`, `modo_manejo`,
  lidar por sector, pose EKF, IMU, v/ω pedida y aplicada) en CUALQUIER modo, haya o no
  dashboard. Esos datos se fusionan al entrenamiento con `simulador/fusion_real.py`
  (ver §1.1). No hizo falta montar nada nuevo.
- **Botón "Grabar video"** en el dashboard (pestaña principal, junto a "Escanear
  planta"): toggle que llama `POST /api/video/record {action:start|stop|status}`.
  Se CORRIGIÓ el handler del backend, que estaba roto (creaba el VideoWriter pero nadie
  le pasaba frames → .mp4 vacío; además fijaba 1920x1080 a ciegas). Ahora un task de
  fondo se suscribe a la cámara, decodifica los MISMOS JPEG del stream (con overlay
  YOLO) y los escribe con el tamaño real del frame. Se guarda en `backend/videos/`.
  El botón sincroniza su estado al cargar (`action:status`).
- **Verificación**: `py_compile` de `jetson_server.py` OK; `agrobot_config.json` válido;
  12/12 tests del supervisor. **Pendiente de prueba en vivo** (requiere arrancar el
  backend / mover el robot → tu autorización): confirmar que izquierda/derecha quedan
  bien y que el .mp4 se graba. Si el giro quedara al revés, poné `invertir_giro:false`.

### 1.6 FSM cableada al backend (flag OFF) + métrica de exploración en sim (2026-10-08)  ✅
- **`jetson_server.py`**: la instanciación del explorador ahora elige
  `maquina_estados.MaquinaEstados` si `autonomia.maquina_estados: true` (OFF por
  defecto), misma interfaz (drop-in). Respaldo `jetson_server.py.bak2`. Con el flag
  OFF el comportamiento no cambia. py_compile OK.
- **Simulador (`correr.py`)**: resumen ahora incluye `celdas_por_min` (celdas nuevas
  de 0,5 m por minuto = eficiencia de exploración, comparable y reproducible sin
  rasterizar el mundo) y `escaneos_planta`. Verificado con un episodio (FSM on):
  `celdas_por_min` y `escaneos_planta` aparecen, sin regresión. *Nota*: en el sim los
  ganchos de escaneo de planta no están cableados aún, así que `escaneos_planta`=0 con
  la FSM; la métrica de eficiencia sí es plenamente útil.

### `sendero.py` — NO es un defecto
Ya hace proyección cenital + búsqueda de rumbo de hilera en ±40° + fusión
paja/vegetación/color adaptativo + confianza. No reescribir; a lo sumo, afinar.

---

## 3. Pendiente / gated (requiere tu autorización o medición)
- **Reactivar frenos como supervisor de modo** (Fase 2): BLOQUEADO por vos hasta validar.
- **Instalar/compilar en ROS**: no hace falta para EKF; evaluar solo si falta algo.
- **Pruebas físicas** (Fase 7): protocolo a autorizar antes de mover el robot.
- **Reinicios de servicios ROS/backend**: no hechos; requieren tu OK (pueden accionar HW).

## 4. Próximos pasos reversibles sugeridos (sin tocar HW)
- Leer y confirmar hallazgos 4/5/6 (emergencia, patrol, current_pose) → diagnóstico firme.
- Diseñar el supervisor de movimiento de Fase 2 como módulo probado en aislado (sin
  activarlo) + tests de: orden tras emergencia, rearme, comando tardío, pose caduca.
- Mover v3 muerto a `experimental/`.
