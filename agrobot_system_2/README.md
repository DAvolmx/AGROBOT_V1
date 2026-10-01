# AgroBot — sistema completo v2 (Jetson + Raspberry Pi, dashboards interactivos)

Arquitectura acordada: Jetson Nano como AP (`192.168.12.1`) y cerebro único
de decisión en tiempo real; ESP32-C3 Mini como actuador puro (sin PID, sin
encoder); ESP32 LoRa V3 sin cambios (firmware ya verificado en campo);
Raspberry Pi corriendo ChirpStack como gateway/almacén histórico remoto.

## Decisión de diseño importante: sin CDN externos

Los mockups de referencia que compartiste cargan Tailwind CSS y Google
Fonts desde CDN. **Los quité del todo** — el dashboard de la Jetson corre
dentro de la red aislada del AP (`192.168.12.x`), que no tiene salida a
internet, así que cualquier `<script src="https://...">` ahí simplemente
no carga en campo y el dashboard queda roto. Reescribí ambos dashboards
con CSS/JS propio, cero dependencias externas, conservando la estética
(paleta oscura, JetBrains-Mono-like vía fuentes del sistema, glow effects,
toasts, tabs). El dashboard de la Raspberry Pi también quedó sin CDN por
consistencia y porque tampoco depende de tenerlo disponible.

También quité dos campos que los mockups mostraban con datos simulados
pero que tu hardware real no produce: **batería** y **GPS** — ningún
sensor del BOM los reporta. Los dejé visibles pero marcados como "no
disponible" en vez de inventar valores, para que la tesis no reporte
capacidades que el sistema no tiene.

## Estructura

```
firmware/
  esp32c3_actuator/     → actuador puro: PWM directo, watchdog, servo/LED
  esp32_lora_v3/         → firmware YA VERIFICADO, sin cambios de lógica
jetson/
  backend/
    jetson_server.py      → asyncio: UDP↔WebSocket↔video↔REST, todo push
    protocol.py           → protocolo binario compartido con el C3
    config_store.py       → "EEPROM" local (JSON) para calibración/hardware
    requirements.txt
  dashboard/
    index.html            → panel multi-tab: Dashboard, Galería de misión,
                             Navegación, SLAM&Telemetría, Modelos IA,
                             Config. Hardware, Parámetros, Sistema
rpi/
  server/
    gateway_server.py     → puente MQTT(ChirpStack)↔WebSocket, histórico
                             SQLite, downlinks de emergencia vía API REST
    decoder.py             → decodifica el payload LoRaWAN de 21 bytes
    telegram_notifier.py   → alarmas y reportes a Telegram (única pieza
                             del sistema con salida a internet)
    requirements.txt
  dashboard/
    index.html             → panel remoto: parcela, paquetes LoRa,
                              Telegram, emergencia, histórico + export CSV
ros2_ws/
  verificar_slam.sh        → diagnóstico del SLAM: distingue las cuatro
                             causas que dan el mismo síntoma
```

Los dos dashboards comparten paleta (tema claro "Agricultura 5.0"): fondo
claro a propósito, porque en campo se miran desde una tablet a pleno sol y
sobre fondo oscuro la pantalla actúa de espejo.

## Topología de red

```
Robot (red aislada 192.168.12.x, AP = Jetson)        Base (con internet)
┌──────────────────────────────────────┐             ┌───────────────────────┐
│ Jetson Nano (192.168.12.1)           │             │ Raspberry Pi          │
│   · dashboard        :8080           │   LoRaWAN   │   · ChirpStack        │
│   · UDP 5001 ← ESP32 LoRa V3         │────────────▶│   · Mosquitto MQTT    │
│   · UDP 5002 ↔ ESP32-C3 Mini         │             │   · dashboard  :8092  │
│   · YOLO: plaga → POST /api/alert ───┼─────LAN────▶│   · bot de Telegram   │
└──────────────────────────────────────┘             └───────────┬───────────┘
                                                                 │ alarmas
                                        sin salida a internet    │ + reportes
                                                                 ▼
                                                       📱 celular del operador
```

La única conexión directa entre los dos es `POST /api/alert`: la Jetson le
empuja al gateway lo que detecta (plagas) para que él lo publique en
Telegram, porque la Jetson no tiene salida a internet. La URL del gateway
se ajusta con la variable de entorno `AGROBOT_GATEWAY_URL` (por defecto
`http://192.168.0.10:8092` desde el 2026-10-01; antes `192.168.12.2`); si el
gateway está caído, la Jetson lo registra y sigue trabajando. Plan de
direcciones vigente: [`docs/RED_E_IPS.md`](docs/RED_E_IPS.md).

Los dos dashboards son procesos y páginas independientes — no comparten
WebSocket. El de la Jetson (control en tiempo real) solo es accesible
desde dentro de la red del AP; el de la Raspberry Pi (histórico/remoto)
puede exponerse fuera de esa red porque solo depende de ChirpStack.

## Cómo correrlo

**Atajo**: `./INICIAR_BOT.sh` en la Jetson y `./INICIAR_SERVER.sh` en la
Raspberry levantan cada lado entero. `INICIAR_BOT.sh` acepta `mapeo` (por
defecto), `nav` o `solo-backend`. Los pasos manuales de abajo siguen siendo
válidos y son lo que hacen los scripts por dentro.

**ESP32-C3 Mini** — ajusta pines de motor al inicio del `.ino`, sube con
Arduino IDE (librería `ESP32Servo`). Reserva su IP por DHCP en el AP de la
Jetson y ponla en `jetson/backend/agrobot_config.json` → `hardware.actuator_ip`
(o desde la pestaña **Config. Hardware** del dashboard una vez corriendo).

**ESP32 LoRa V3** — sin cambios, tal como lo verificaste.

**Jetson Nano**:
```bash
cd jetson/backend
pip install -r requirements.txt
python3 jetson_server.py
```
Deja modelos entrenados (`.engine`/`.pt`/`.onnx`) en `jetson/backend/models/`
— aparecen automáticamente en la pestaña **Gestor de Modelos IA** del
dashboard para cargarlos sin tocar código.
Abre `http://192.168.12.1:8080` desde cualquier dispositivo en la red del AP.

**Raspberry Pi** (además de tu instalación existente de ChirpStack +
Mosquitto):
```bash
cd rpi/server
pip install -r requirements.txt
export CHIRPSTACK_APP_ID=<uuid>            # application_id (UUID en v4)
export CHIRPSTACK_API_URL=http://localhost:8090   # REST API, NO el 8080 de la web
export CHIRPSTACK_API_TOKEN=<tu_api_key>    # necesario para downlinks
export AGROBOT_DEV_EUI=f59d08345985b27d      # del firmware LoRa
python3 gateway_server.py
```
Abre `http://<ip-de-la-pi>:8092`.

## Por qué esta arquitectura minimiza latencia (sin cambios respecto a v1)

Comandos de movimiento en binario (no JSON), todo push por WebSocket
(nunca polling), buffer de cámara fijado a 1 frame, watchdog local en el
C3 independiente de la red. Ver el detalle completo en la v1 de este
README (conservado abajo) — nada de esto cambió, solo se le agregaron
más superficies de control (modelos, hardware, calibración, misión).

## Navegación con ROS 2 (Nav2 + slam_toolbox + robot_localization)

Decisión tomada con el usuario: el enlace Jetson↔ESP32-C3 sigue siendo el
UDP binario propio (no se migra a serial/ros2_control), el LIDAR se lee
con el driver oficial `sllidar_ros2`, se migra a Nav2 completo (mapear,
guardar mapa, navegar a metas — no solo evasión reactiva), y todo corre
en la Jetson.

```
ros2_ws/src/agrobot_navigation/
  package.xml, CMakeLists.txt
  launch/
    agrobot_mapping.launch.py      → sllidar_ros2 + robot_localization + slam_toolbox (modo mapeo)
    agrobot_navigation.launch.py   → lo mismo + slam_toolbox en modo localización + Nav2 completo
  config/
    ekf.yaml                       → robot_localization: fusiona IMU real + "wheel_odom" (velocidad
                                       comandada, no un encoder real — covarianza alta a propósito)
    slam_toolbox_mapping.yaml
    slam_toolbox_localization.yaml
    nav2_params.yaml                → costmaps, DWB, planner, behaviors — ajustado a un robot
                                       lento (~0.4 m/s tope) sin footprint medido (círculo 25cm)
  maps/                             → aquí caen agrobot_map.yaml / .pgm al guardar

jetson/backend/ros_bridge.py         → el puente: un nodo rclpy embebido en el mismo proceso de
                                        jetson_server.py (hilo de fondo). Si rclpy no está instalado,
                                        el módulo se vuelve no-op y el sistema sigue funcionando con
                                        el PoseEstimator/LidarReactive caseros de slam.py como antes.
```

**Qué hace `ros_bridge.py` exactamente:**
- Suscribe `/cmd_vel` (lo que publica Nav2) → llama a la MISMA función
  `send_command()` que ya usa el control manual, así la evasión reactiva
  de `slam.py` se sigue aplicando incluso con el costmap de Nav2 activo
  (doble capa de seguridad).
- Suscribe `/scan` (de `sllidar_ros2`) → alimenta `lidar_reactive` y el
  radar del dashboard — reemplaza al `POST /api/lidar/scan` manual, que
  se deja solo como fallback de pruebas.
- Suscribe `/odometry/filtered` (de `robot_localization`) → esta es la
  pose real que ve el dashboard ahora (`pose_source: "ros_ekf"`); si ROS
  no está corriendo, cae automáticamente al filtro complementario casero
  (`pose_source: "dead_reckoning"`).
- Suscribe `/map` (de `slam_toolbox`) → se dibuja en la nueva pestaña
  **Navegación** del dashboard.
- Publica `/imu/data_raw` (acelerómetro+giroscopio reales del MPU6050,
  vía la telemetría UDP de la ESP32 LoRa V3) y `/wheel_odom` (velocidad
  comandada, marcada con covarianza alta porque NO es un encoder real).
- Cliente de la acción `navigate_to_pose` de Nav2 — se dispara desde
  `POST /api/nav/set_goal`, que en el dashboard es simplemente un clic
  sobre el mapa en la pestaña **Navegación**.

**Cómo correrlo** (además de `python3 jetson_server.py`, que debe correr en un
entorno con ROS activado — `source /opt/ros/<distro>/setup.bash` antes de
`pip install -r requirements.txt`, para que `rclpy` sea importable):

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash

# comprobación de que la cadena entera funciona (LIDAR → TF → slam_toolbox → /map)
./verificar_slam.sh

# 1) primera vez en un terreno nuevo: mapear
ros2 launch agrobot_navigation agrobot_mapping.launch.py
# mueve el robot manualmente desde el dashboard mientras el mapa se arma,
# luego en el dashboard: pestaña Navegación → "Guardar mapa"

# 2) ya con el mapa guardado: navegación real
ros2 launch agrobot_navigation agrobot_navigation.launch.py
# ahora un clic en el mapa del dashboard manda al robot a ese punto
```

### Por qué el SLAM no publicaba mapa (corregido)

Verificado sobre el hardware real (RPLIDAR C1 en `/dev/ttyUSB0`, ROS 2
Jazzy). Había **dos** fallos, y los dos daban exactamente el mismo síntoma
—"el mapa nunca aparece"— sin imprimir un solo error:

1. **Faltaban los TF estáticos.** Nadie publicaba `base_link → laser`. El
   driver marca sus scans con `frame_id: laser` y el EKF publica
   `odom → base_link`, así que el árbol quedaba partido en dos y
   slam_toolbox descartaba **todos** los scans en silencio. Los launch
   ahora publican `base_link → laser` y `base_link → imu_link` (medidas en
   `_tf_estaticos()`, ajustar si se mueve el LIDAR).
2. **slam_toolbox es un lifecycle node en Jazzy** y se quedaba en
   `unconfigured` para siempre: aparecía vivo en `ros2 node list`, pero ni
   siquiera se suscribía a `/scan`. Los launch ahora incluyen un
   `nav2_lifecycle_manager` que lo configura y activa al arrancar.

Hay una tercera causa que **no** es un fallo sino una dependencia real: el
EKF no publica `odom → base_link` hasta recibir su primera medición, y esa
medición la manda `jetson_server.py` (`/imu/data_raw`, desde el MPU6050
que llega por LoRa). Con el backend apagado no hay mapa. Para probar el
SLAM en el banco, sin robot ni telemetría:

```bash
ros2 launch agrobot_navigation agrobot_mapping.launch.py odom_estatico:=true
```

Eso publica un `odom → base_link` fijo. **No usarlo en campo**: la pose
queda clavada en el origen y el mapa saldría corrupto en cuanto el robot
se mueva.

`./verificar_slam.sh` distingue las cuatro causas una por una (puerto
serie, nodos vivos, estado del lifecycle, cada eslabón del árbol TF, y si
`/scan` y `/map` publican de verdad).

Requiere tener instalados `sllidar_ros2`, `robot_localization`,
`slam_toolbox` y `nav2_bringup` en el entorno ROS 2 de la Jetson (no
vienen incluidos en este repo — son paquetes ROS estándar, se instalan
con `apt`/`rosdep`, no con `pip`).

## Alarmas y reportes por Telegram

La Jetson y el robot viven en la red aislada del AP y **no tienen salida a
internet**. El gateway de la Raspberry Pi sí, y es la única pieza que
habla con el exterior: todo lo que hay que hacerle llegar al operador sale
por ahí (`rpi/server/telegram_notifier.py`, pestaña **Telegram** del panel
del gateway).

**Puesta en marcha** (3 pasos, desde la pestaña Telegram):

1. En Telegram, hablar con `@BotFather` → `/newbot` → copiar el token.
2. Pegarlo en el panel y guardar. El token queda en
   `rpi/server/telegram_config.json` con permisos `600` y **nunca** se
   devuelve por la API — el dashboard solo sabe si hay uno cargado. Si se
   prefiere no escribirlo en disco, la variable de entorno
   `TELEGRAM_BOT_TOKEN` tiene prioridad y el campo del panel se bloquea.
3. Abrir el chat del bot y mandarle `/start`: el gateway aprende ese chat
   y queda vinculado. Solo ese chat recibe alarmas y puede dar órdenes.

**Qué avisa** (cada tipo con su interruptor y su propio antirrebote — sin
él, un sensor I2C mudo manda un mensaje por uplink y el chat se vuelve
inusable):

| Alarma | De dónde sale |
|---|---|
| 🐛 Plaga detectada | YOLO en la Jetson → `POST /api/alert` al gateway |
| 📉 Lectura fuera de rango | `schema.validate()` marcó un valor imposible |
| 🔌 Sensor sin inicializar | el nodo mandó `0.0` porque el sensor no arrancó |
| 📡 Enlace LoRa caído | sin uplinks por más de `link_timeout_s` (avisa una vez, y otra al volver) |
| 🛑 Downlink de emergencia | alguien mandó STOP/PARK desde el panel |

**Reporte de los últimos datos**: promedios, mínimos/máximos, calidad del
enlace y cuántos paquetes venían marcados como sospechosos. Se pide desde
el panel (con previsualización, para no gastar un mensaje), desde el chat
con `/reporte`, o automático cada N minutos. Los valores que el esquema ya
marcó como basura quedan **fuera de los promedios**: un SCD40 que no
arranca manda `0.0` y arrastraba la temperatura media a cero.

**Comandos desde el chat**: `/estado`, `/ultimos`, `/reporte`, `/silencio`,
`/ayuda`. Se leen por long-polling (`getUpdates`), no por webhook: un
webhook exigiría IP pública y TLS en la Raspberry Pi, y la mitad de los
despliegues de campo están detrás de un router 4G sin puerto abierto.

Si el gateway está apagado o sin internet, la Jetson sigue detectando,
guardando la foto en la galería y funcionando igual — el aviso remoto es
un extra, nunca un requisito.

## Pendiente de integrar (fuera del alcance de esta entrega)

- Footprint real del robot medido (hoy `nav2_params.yaml` usa un círculo
  de 25cm de radio como aproximación honesta, no una medida real).
- Puerto serie real del RPLIDAR C1 confirmado en la Jetson (los launch
  files asumen `/dev/ttyUSB0`, puede necesitar una regla udev si hay más
  de un dispositivo serie).
- Footprint real medido para el costmap (ver arriba).
- Detección automática de la red de base para subir el reporte de sesión.
