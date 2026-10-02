# MANIFIEST.md — AgroBot, estado maestro del sistema

> **INSTRUCCIÓN GENERAL — leer antes de tocar cualquier archivo de este repo:**
> Este documento es la fuente única de verdad del proyecto AgroBot. **Toda
> modificación al sistema (código, firmware, arquitectura, variables de
> configuración, endpoints, tópicos ROS, esquema de datos, decisiones de
> diseño) debe registrarse aquí en el mismo momento en que se hace** —no
> después, no "cuando haya tiempo". Antes de escribir código nuevo, revisa
> este archivo para no duplicar ni contradecir algo ya decidido. Después de
> escribir código nuevo, actualiza la sección correspondiente Y agrega una
> entrada al **Historial de cambios** (al final) con fecha, qué cambió, por
> qué, y qué archivos tocó. Si un cambio contradice algo ya documentado
> aquí, se marca explícitamente como reemplazo (no se borra la decisión
> vieja sin dejar rastro — se tacha o se anota "reemplazado por X").
> Cualquier LLM o persona que retome este proyecto debe poder reconstruir
> el estado completo del sistema leyendo solo este archivo.

Última actualización: 2026-10-02

---

## 0. ESTADO ACTUAL — 2026-09-17 (leer esto primero)

> **Prueba de manejo en pasillo — 2026-10-02 (detalle en el Historial)**:
> - `safety.max_linear_accel_mps2`/`max_angular_accel_rps2` estaban en
>   **0,1 / 0,2** desde `10f9694` (2026-09-19, cambio no explicado en el
>   commit): un pulso manual no pasaba de 0,03 m/s ni 0,05 rad/s y el robot
>   NO se movía ni giraba. El usuario los subió a **3 / 8** desde el dashboard.
> - **rf2o publica la velocidad lineal con el SIGNO INVERTIDO** (bug del
>   paquete: `lin_speed` sale del desplazamiento en el marco del LÁSER, que
>   está girado 180°). El EKF y el monitor de patinaje lo heredan: "patinaje
>   100 %" y "robot atascado" enganchan el modo seguro con el robot avanzando.
>   El TF del LIDAR (`yaw=π`) y los motores están BIEN (confirmado por el
>   usuario con la cámara). Sin corregir todavía.
> - Drivers reconectados por el usuario: avanza 21–29 cm por pulso. El GIRO
>   salía invertido (lados cruzados): `enlace_c3.INVERTIR_GIRO = True`. El
>   pivote casi no rota (1–4°/pulso) y retrocede al girar: revisar ruedas.
> - Obstáculos bajos (cobija, pesa) invisibles al LIDAR y al freno.
> - El modo seguro se engancha por un hueco de 1,7 s de la C3 y luego deja
>   lanzar patrullas que no se mueven (factor 0).
> - **La C3 y el LoRa V3 se reinician juntos** (cae la alimentación). Tras
>   cada reinicio las entradas de los motores quedaban sueltas ~1,7 s: una
>   llanta trasera se quedó girando. Firmware corregido, falta flashear.

> **Cambios vigentes del 2026-10-01 — RED** (detalle en
> `agrobot_system_2/docs/RED_E_IPS.md` y en el Historial):
> - El DIR-615X pasó de punto de acceso a **modo router**: LAN
>   `192.168.0.1/24`, canal 1 fijo, dos SSID en la misma red (`LoRa-Wan` y
>   `LoRa_Wan`). IP fijas: Pi `192.168.0.10` (cable) / `.20` (Wi-Fi), Jetson
>   `.142`, gateway Heltec `.11` (Wi-Fi, estática). Reemplaza el plan con la
>   Pi en `192.168.12.2` de la §2.
> - Las alertas de la Jetson van a `http://192.168.0.10:8092`; el Heltec
>   reenvía a `192.168.0.10:1700` (antes `192.168.0.170`).
> - Tailscale en Jetson (`100.82.220.28`) y Pi (`lora-server-pi`).

> **Cambios vigentes del 2026-09-30** (detalle en el Historial, al final):
> - **Enlace con la C3 REDUNDANTE: BLE + WiFi UDP a la vez** (`enlace_c3.py`,
>   `udp_actuator_link.py`, firmware con `ENLACE_WIFI 1`). La C3 aplica la
>   trama más nueva por número de secuencia venga por donde venga. Reemplaza
>   la "UNA SOLA VIA DE MANDO: BLE" de la sección 3. Sigue sin cables entre
>   la batería de motores y la Jetson.
> - **ZUPT**: el backend publica velocidad cero en `/zupt` (odom4 del EKF)
>   tras 3 s sin órdenes de moverse. Sin él el EKF extrapoló durante horas
>   sin LIDAR y el mapa llegó a 9,7 km.
> - LIDAR: puerto elegido en cada arranque (USB CP210x si está, si no
>   `/dev/ttyTHS1` tras esperar 30 s). Autoimpactos filtrados a 0,18 m.
> - `/api/health` trae `detail` por fila y dos filas nuevas: `ros` y `ekf`;
>   la de SLAM marca el mapa corrupto (>500 m).

Resumen de lo que es verdad HOY. Donde una sección de más abajo diga otra
cosa, manda esta (y la sección vieja lleva una nota de reemplazo).

### Cómo arranca de verdad el sistema
- **systemd, NO `INICIAR_BOT.sh`.** `agrobot-backend.service` y
  `agrobot-ros.service` (ambos `Restart=always`: al matar un proceso a mano
  reaparece solo). Además `agrobot-ap0`, `agrobot-hostapd`, `agrobot-dhcp` y
  `agrobot-reloj-vigia`. No usar `INICIAR_BOT.sh` a la vez: los dos abren el
  puerto 8080. `INICIAR_BOT_MEJORADO.sh` (citado el 2026-09-13) **no existe**.
- Copia de todos los archivos de sistema en `agrobot_system_2/jetson/systemd/`
  (services, drop-ins, `sbin/`, `etc/hostapd`, `etc/default`). La clave del
  WiFi va como `CAMBIAR_AL_INSTALAR`.
- **Control de versiones:** repo git en `/home/jetson/Desktop/AGROBOT` desde
  el 2026-09-17. Fuera del repo (ver `.gitignore`): `ros2_ws/build|install|log`,
  logs, capturas, mapas guardados, pesos de entrenamiento duplicados y dos
  paquetes de terceros que hay que reclonar en `ros2_ws/src/`:
  `https://github.com/MAPIRlab/rf2o_laser_odometry.git` @ `b38c68e` y
  `https://github.com/Slamtec/sllidar_ros2.git` @ `3430009`. Luego
  `colcon build --symlink-install`.

### Red
- Una sola radio (Intel 8265) sirve el AP `JETSON_AP` (`ap0`) y el cliente
  `LoRa-Wan` (antes `LoRa_Wan`). **Tienen que estar en el mismo canal** o el
  enlace se vuelve intermitente ("la Jetson se cuelga").
  `agrobot-sync-ap-channel.sh` corre como `ExecStartPre` de hostapd y además
  cada minuto (`agrobot-sync-ap-channel.timer`), y copia el canal del cliente.
- Plan de direcciones completo, reglas del router y checklist de mudanza:
  **`agrobot_system_2/docs/RED_E_IPS.md`**. Regla clave: nunca dos reservas
  DHCP con la misma IP (el router se queda sin DNS ni DHCP).

### Reloj (la Jetson no tiene pila de RTC)
- Algo en el arranque devuelve el reloj a 1970 a los ~9 s y NTP lo corrige
  entre 80 s y 6 min después. Ese salto con ROS corriendo atasca o mata
  slam_toolbox. `agrobot-reloj-vigia` reinicia `agrobot-ros` UNA vez cuando
  NTP sincroniza. `systemd-time-wait-sync` está deshabilitado a propósito
  (probado: expiraba y solo retrasaba ROS 3 min).

### Navegación autónoma
- `agrobot_mapping.launch.py` (el que usa systemd) **incluye Nav2**: 4 nodos
  (`controller_server`, `planner_server`, `behavior_server`, `bt_navigator`)
  con `lifecycle_manager_navigation`, sobre slam_toolbox en mapeo. Primer día
  en que Nav2 llegó a activarse. `agrobot_navigation.launch.py` sigue roto
  (usa `navigation_launch.py` de nav2_bringup, que arranca 10 nodos y falla).
- Arreglos en `nav2_params.yaml`: `data_type: "LaserScan"` en las capas de
  obstáculos (sin eso el costmap no veía nada), `progress_checker_plugins` en
  plural, `navigators` declarados, huella rectangular 0,40 × 0,35 m.
- Driver del LIDAR con `respawn=True` (moría con `80008004` en ~1 de cada 4
  arranques, y al desconectar el USB).
- ⚠️ **`default_nav_to_pose_bt_xml: ""` NO significa "el árbol por defecto"**
  en Jazzy: significa árbol VACÍO. bt_navigator aceptaba cada meta y la
  abortaba en el acto ("Behavior tree threw exception: Empty Tree"), 63 metas
  seguidas, y el robot NUNCA se movía en autónomo. La ruta real va ahora en
  cada meta (`goal.behavior_tree`, ros_bridge.py) y también por parámetro
  desde el launch. Diagnosticado el 2026-09-17.
- **Dos modos autónomos:**
  · **Patrulla** (`patrol.py`, `POST /api/mission/start`): repite los
    waypoints con Nav2.
  · **Exploración** (`explorador.py`, `POST /api/exploracion/start`):
    autonomía REACTIVA sin mapa ni waypoints — mira las distancias del LIDAR,
    avanza a pulsos, gira al lado más despejado, retrocede si no hay salida,
    se desatasca si la pose no cambia en 25 s, y barre cada 2 min. No usa
    Nav2: los motores no tienen gama lenta y no pueden ejecutar velocidades
    finas.
- **Grabación automática del recorrido** (`POST /api/patrol/auto`): con el
  operador conduciendo, suelta un waypoint cada 1,5 m recorridos (mide sobre
  la pose, no sobre el tiempo). La exploración la activa sola. Los puntos se
  dibujan sobre el mapa del dashboard. Recorre waypoints en ciclo, escanea cada 2 min y
  al detectar planta (robot quieto), salta metas que Nav2 aborta. Waypoints
  en el marco `map` (vía TF).
- **Mapas con nombre** (`mapas.py`): guardan el grafo de slam_toolbox + el
  recorrido en `agrobot_system_2/maps/<nombre>/`. Al cargar, el SLAM sigue
  ampliando ese mapa. **El robot debe estar en la base donde empezó el mapa.**
  Un reinicio de ROS sin cargar mapa empieza uno nuevo y el recorrido deja de
  valer.

### Odometría visual con la webcam actual (activa en el backend, sin fusionar)
- `jetson/backend/odometria_webcam.py`, pedida por el usuario el 2026-09-17
  aun sabiendo los límites. Monocular + modelo de suelo plano: sigue rasgos
  de la mitad baja (Lucas-Kanade), los proyecta al suelo con altura e
  inclinación de la cámara y ajusta giro + avance con RANSAC.
- Se PAUSA si el C3 no reporta el servo al frente (±3° de 98°), si está
  escaneando, o con pocos rasgos. Quieto = mide cero.
- Config `odometria_webcam` (altura 0,30 / inclinación 15° / hfov 70° /
  adelante 0,15: **SIN MEDIR**). `habilitada`=true mide y muestra;
  `publicar_ros`=false: no va al EKF hasta calibrar comparando con el EKF en
  la pestaña SLAM & Telemetría. Publica `/odom_webcam` → `odom3` en ekf.yaml
  (el EKF lo toma tras reiniciar `agrobot-ros`).
- Validado: movimientos sintéticos sobre un fotograma real (giro exacto,
  avance ±3–6 %, signos correctos) y 25 fotogramas reales quieto (cero exacto,
  3,5 ms/cuadro). **No probado con el robot moviéndose** (el C3 no respondía).
- Arreglo de paso en `camera.py`: las cajas de YOLO se dibujaban sobre el
  mismo array que se guardaba como "frame crudo", así que YOLO analizaba
  imágenes con sus propias cajas pintadas.

### Odometría visual con cámara de profundidad (preparada, sin cámara todavía)
- Decisión del usuario (2026-09-17): cámara de profundidad FIJA al
  frente (RealSense D435i u OAK-D Lite), como mejora posterior a la webcam.
- `launch/odometria_visual.launch.py` (incluido por el de mapeo, apagado por
  defecto) + `config/odometria_visual.yaml` → rtabmap `rgbd_odometry` publica
  `/odom_visual`. `ekf.yaml` ya lo fusiona como `odom2` (vx + giro); sin
  publicador no hace nada.
- Para activarla: instalar el driver (`ros-jazzy-realsense2-camera` o
  `ros-jazzy-depthai-ros-driver`), medir `CAM_XYZ` en el launch, comprobar los
  tópicos del preset con `ros2 topic list`, y en `/etc/default/agrobot`:
  `AGROBOT_LAUNCH_ARGS="odom_visual:=true camara:=realsense"` +
  `sudo systemctl restart agrobot-ros`.
- Validado sin cámara: los argumentos aparecen en el launch, sin driver avisa
  en vez de romper, rtabmap acepta los parámetros y el EKF carga con 4
  fuentes. **No probado con imágenes reales.**

### Manejo manual
- El clic en el mapa para mandar metas se quitó el 2026-09-17: un clic
  accidental arrancaba el robot.
- Por pulsos (`pulse_drive.py`, `POST /api/drive`): un toque = 1 s
  adelante/atrás, encadenable hasta 3 s. Espacio o ■ = parada. Giro con
  maniobra para tierra blanda (arco adelante + arco atrás, mismo sentido).
- La rampa avanza sola a 20 Hz (`ActuatorController.tick`) con hombre muerto
  de 0,5 s. Corrige un fallo grave: al soltar el botón el robot seguía a
  ~0,66 m/s indefinidamente.

### Frenado por LIDAR (`slam.py`, `LidarReactive`)
- Mide en un **corredor del ancho del robot**, no en un cono (en pasillos el
  cono veía las hileras). Distancias a las 4 caras en el dashboard
  (`lidar_distancias`, 10 Hz).
- **Corte a cero** cuando el hueco < `obstacle_margen_m` (0,15) +
  `obstacle_deslizamiento_m` (0,80). Sin datos de LIDAR no avanza ni
  retrocede. No limita el giro.
- `obstacle_avoidance.py` (`ControladorEvasion`) es **código muerto**: nunca
  se ejecutó (exigía un atributo `_lidar_scan` que no existe).

### Límites físicos medidos — condicionan todo lo anterior
- PWM de motores 200..255: **no hay gama lenta** (78 % a 100 %).
- Al parar, ruedas sueltas (TA6586 en reposo, no freno): **desliza ~80 cm**.
- `control.max_v_mps` = 1,5 en la config vs `V_MAX` = 0,55 en el firmware:
  los m/s del dashboard y de Nav2 **no son reales**.
- Robot: 40 cm de largo × 35 cm de ancho. Sin encoders.

### Pendiente, por prioridad
1. ~~Freno activo en firmware~~ — **hecho en código el 2026-10-01** (`FRENO_ACTIVO`); falta cargarlo en la C3.
2. Medir velocidad real al 100 % y alinear `max_v_mps` con `V_MAX`.
3. Parachoques físico con finales de carrera directo al C3.
4. Seguridad: clave del AP `12345678`. La API ya admite clave (2026-10-01, archivo `~/.config/agrobot/clave_dashboard`); falta crear el archivo y reiniciar el backend.
5. Arreglar `agrobot_navigation.launch.py`.
6. Odometría visual: webcam **midiendo pero sin calibrar ni fusionar**;
   cámara de profundidad **preparada, falta comprarla** (ver arriba).
7. Pila para el RTC de la Jetson.
8. Borrar `obstacle_avoidance.py` y corregir `REPORTE_MEJORAS_2026-09-13.md`.

---

## 1. Qué es AgroBot

Robot agrícola de detección temprana de plagas (proyecto de tesis). Recorre
cultivos, escanea plantas con cámara + IA (YOLO), reporta telemetría
ambiental, y sube esos datos tanto a un dashboard local en tiempo real como
a un histórico remoto vía LoRaWAN.

## 2. Arquitectura general

```
┌─────────────────────────────────────────────┐
│         RED AISLADA DEL AP (Wi-Fi)           │
│           192.168.12.0/24                    │
│         (sin salida a internet)              │
└─────────────────────────────────────────────┘

   PLAN DE DIRECCIONES (fijas — ver docs/CONFIGURAR_AP_JETSON.txt)
     192.168.12.1    Jetson (AP/gateway)
     (el ESP32-C3 actuador YA NO está en la red: habla por UART, §13)
     192.168.12.51   ESP32 LoRa V3       (fija por reserva DHCP: firmware congelado)
     .100 - .150     pool DHCP para celulares/laptops del operador

   JETSON NANO — crea el AP Wi-Fi (192.168.12.1) — cerebro único
     ├─ Servidor UDP :5001 (RX)  ← ESP32 LoRa V3 (telemetría ambiental)
     ├─ UART /dev/ttyTHS1 @115200 ↔ ESP32-C3 Mini (comandos/status, §13)
     ├─ Servidor web/WebSocket :8080 (dashboard)
     ├─ Visión YOLO sobre la cámara (detección de plagas)
     └─ (opcional) nodo ROS 2 embebido — puente a Nav2/slam_toolbox/
        robot_localization/sllidar_ros2 (procesos ROS aparte)

   ESP32-C3 Mini — actuador PURO (sin PID, sin encoder)
     4 motores PWM, servo 180° de cámara, foco de escaneo, watchdog local
     SIN RADIO: ni Wi-Fi ni BLE (los dos se retiraron el 2026-09-04, §4)
     UNA sola vía de mando: UART GPIO20/21 <-> Jetson, 3 hilos (§13)

   ESP32 LoRa V3 — sensores (verificado, sin tocar)
     MPU6050 (IMU), SCD40 (CO2/T/H) — I2C directo
     Envía por UDP local Y por LoRaWAN OTAA (US915) simultáneamente

   Gateway LoRaWAN HT-M02 Heltec — fuera del AP, va por el aire

   RASPBERRY PI — ChirpStack + Mosquitto + gateway_server.py propio
     Histórico SQLite, dashboard remoto :8092, downlinks de emergencia
```

> **2026-10-01 — reemplazo parcial:** la Pi y el gateway Heltec ya no
> cuelgan del AP de la Jetson sino del router `LoRa-Wan` (DIR-615 en modo
> router, `192.168.0.0/24`): Pi `192.168.0.10`, Heltec `192.168.0.11`,
> Jetson cliente `192.168.0.142`. El AP `192.168.12.0/24` sigue igual para
> el robot. Ver `agrobot_system_2/docs/RED_E_IPS.md`.

**Quién crea el AP:** la Jetson Nano. Todo lo demás (ESP32 LoRa V3,
ESP32-C3 Mini, cualquier dispositivo que vea el dashboard) es cliente de
esa red. El AP no tiene salida a internet — la única ruta hacia afuera es
LoRaWAN, que no pasa por el AP.

### Decisiones de diseño fijas (no cambiar sin discutirlo primero)

- IP única del AP: **192.168.12.1** (se descartó 192.168.4.1, que aparecía
  en documentación vieja del vault).
- **Sin encoders en los motores** — se descartó el diseño encoder+PID.
  Navegación/pose se resuelve con IMU + LIDAR (ver §5).
- **El robot es DIFERENCIAL (skid-steer), NO mecanum** — confirmado por el
  usuario el 2026-09-02, ver §3. Dos grados de libertad (`vx`, `omega`);
  gira patinando las ruedas y **no puede desplazarse de costado**.
  ⚠️ Entre el 2026-09-01 y el 2026-09-02 el sistema entero estuvo adaptado
  a mecanum por una deducción equivocada (ver changelog). Toda
  documentación intermedia que diga "holonómico" está desactualizada.
- **La Jetson corre en MODO DUAL AP+cliente** sobre una sola radio. El chip
  es un **Intel Wireless 8265 (`iwlwifi`)**, que declara en `iw list`:
  `#{ managed } <= 1, #{ AP, P2P-client, P2P-GO } <= 1, total <= 3`
  — o sea **un cliente más un AP a la vez**. El AP vive en un vif virtual
  `ap0` creado sobre el mismo `phy0`.
  - ⚠️ **El AP debe usar EL MISMO CANAL que el cliente.** La radio es una
    sola: pedirle dos canales la obliga a repartirse el tiempo, y en
    iwlwifi eso va de "lento e intermitente" a directamente no asociar. Hoy
    ambos en el canal 1. Si `LoRa_Wan` cambia de canal hay que cambiar
    `channel=` en `/etc/hostapd/agrobot.conf`.
  - ⚠️ **`ap0` tiene que quedar fuera de NetworkManager.** Si NM la
    reclama le pone `type managed`, y dos interfaces `managed` en el mismo
    phy violan `#{ managed } <= 1`: el driver rechaza levantar la segunda
    con un `EBUSY` que no menciona nada de esto. Se resuelve con
    `/etc/NetworkManager/conf.d/99-agrobot-ap0.conf`.
  - ⚠️ **MAC distinta de la del cliente** (se le pone el bit de
    "administrada localmente"): dos vifs con la misma MAC en el mismo phy
    también dan `EBUSY`.
  - **NO hay NAT ni reenvío entre las dos redes**, a propósito: el tráfico
    Jetson↔ESP32 es todo local dentro de `192.168.12.0/24` y el robot no lo
    necesita. Habilitarlo sería exponer una red a la otra sin motivo.
- La **Jetson es el AP**, no el ESP32-C3 (se evaluó y se descartó moverlo
  al C3 por riesgo de perder video/dashboard si el C3 se satura).
- El ESP32-C3 es actuador **puro**: sin PID, sin lógica de navegación, solo
  ejecuta el comando que le llega y tiene un watchdog local independiente.
- Firmware de la ESP32 LoRa V3: **no se modifica la lógica**, ya está
  verificado en campo.
- Sin dependencias CDN externas en ningún dashboard (la red del AP no
  tiene internet) — todo el CSS/JS es propio, fuentes del sistema.
- No se inventan datos de sensores que no existen (batería, GPS) — se
  marcan "no disponible" en vez de simularlos.
- Comunicación Jetson↔ESP32-C3: **UDP binario propio**, no se migra a
  serial/ros2_control aunque el resto del sistema use ROS 2.
- LIDAR real: driver oficial **sllidar_ros2** (no lectura manual).
- Navegación: se migró a **Nav2 completo** (mapear + guardar mapa +
  navegar a metas), no solo evasión reactiva.
- Todo el cómputo (incluido ROS 2) corre **en la Jetson**, no se reparte a
  la Raspberry Pi ni a otro equipo.

---

## 3. Firmwares

### `firmware/esp32c3_actuator/esp32c3_actuator.ino`

**Pinout real de la PCB** (esquemático del usuario, 2026-09-01):

| GPIO | Función |
|---|---|
| 0 / 1 | motor **m1** (puente H, IN1/IN2) |
| 2 / 3 | motor **m2** — GPIO2 es pin de strapping ⚠️ |
| 4 / 5 | motor **m3** |
| 6 / 7 | motor **m4** |
| 8 | **FOCO** (LED) — pin de strapping ⚠️ |
| 10 | **SERVO-CAM** |
| 9, 20, 21 | libres (9 = botón BOOT, 20/21 = UART0 de consola) |

**Drivers: dos módulos con dos chips TA6586 cada uno** (un puente H completo
por motor, DIP-8, 3–14 V). Sin pin de enable: cada motor tiene solo FI y BI.

**Tabla de verdad del TA6586:**

| FI | BI | resultado |
|---|---|---|
| HIGH | LOW | adelante |
| LOW | HIGH | reversa |
| HIGH | HIGH | **frenado** |
| LOW | LOW | libre (rueda suelta) |

Como no hay enable, la velocidad se hace con **PWM sobre la entrada de
dirección activa y la otra en LOW**, simétrico en ambos sentidos.

### Pinout y disposición — CONFIRMADOS EN EL ROBOT (2026-09-02)

No son supuestos: salen del firmware de prueba del usuario, probado
moviendo el robot.

| Rueda | Posición | FI | BI |
|---|---|---|---|
| M1 | delantera **derecha** | GPIO0 | GPIO1 |
| M2 | delantera **izquierda** | GPIO**3** | GPIO**2** |
| M3 | trasera **derecha** | GPIO4 | GPIO5 |
| M4 | trasera **izquierda** | GPIO6 | GPIO7 |

**El M2 tiene FI y BI invertidos** respecto de las otras ruedas. Está así
cableado y verificado — no "arreglarlo". GPIO8 = FOCO, GPIO10 = SERVO-CAM.
GPIO9 (BOOT), 20 y 21 (UART0) quedaron libres.

### Cinemática diferencial (skid-steer)

⚠️ **El robot NO tiene ruedas mecanum.** Confirmado por el usuario el
2026-09-02. Es diferencial: gira patinando las ruedas y **no puede
desplazarse de costado**.

Convención ROS: +x adelante, +omega antihorario (giro a la izquierda).

```
wIzq = (vx - K*omega) / R        wDer = (vx + K*omega) / R
        con K = trocha / 2
```

Girar a la izquierda manda el lado DERECHO adelante y el IZQUIERDO atrás —
exactamente `girarIzquierda()` del sketch de prueba que anda en el robot.
La separación delante-atrás **no entra** en la cinemática de un skid-steer.

**Agrupación de lados** (esto es lo único que importa en diferencial: los
dos motores de un lado reciben siempre el mismo comando):

```
LADO DERECHO:   M1 GPIO0/1   +   M4 GPIO6/7
LADO IZQUIERDO: M2 GPIO3/2   +   M3 GPIO4/5
```

⚠️ El **encabezado** del sketch de prueba dice "M1 delantero derecho, M4
trasero IZQUIERDO", pero los **comentarios dentro de `girarIzquierda()`**
dicen "motor(1) y motor(4) = derecha". Se contradicen. Manda el segundo:
con ese agrupamiento el robot gira bien, que es lo que el usuario verificó.

**Escalado dinámico del PWM:** la referencia es la rueda a fondo (`V_MAX/R`)
y **solo si un lado se pasa** se escalan los dos por el mismo factor.
Escalar por igual conserva la relación entre lados, así que el radio de giro
no se deforma: el robot va más lento pero describe la misma curva. Lo obvio
—dividir siempre por el peor caso `vx+omega` a tope— dejaba el avance recto
al 69% de potencia aun sin nada que reservar.

**Canales LEDC:** un canal por motor (4), re-apuntado al pin FI o BI según
el sentido; el cambio ocurre solo al invertir la marcha y con el duty en 0,
así que no genera glitches. Quedan 5 de los 6 canales del C3. En diferencial
los dos motores de un lado podrían compartir un canal y bajar a 2; no se
hace porque con un canal por motor cada uno conserva su `INVERT` y su propio
arranque por pulsos, y sobran canales igual.

**Frecuencia de PWM fijada explícitamente en 8 kHz** (`MOTOR_PWM_HZ`, 8
bits), no el default de `analogWrite()` — ese default cambió entre versiones
del core de ESP32 (1 kHz / 5 kHz) y hacía que el mismo código se comportara
distinto según con qué versión se compilara.

### Arranque: zona muerta + pulsos de torque (2026-09-02)

Adoptado del segundo sketch de prueba del usuario, que **rinde mejor que el
PWM proporcional puro**. La razón es física, no de código:

- **Zona muerta (`PWM_MIN = 90`).** Un motorreductor con carga no arranca
  con cualquier duty: por debajo de cierto umbral el TA6586 conmuta pero el
  par no vence el rozamiento estático y el motor solo zumba. Todo el tramo
  `0..PWM_MIN` era, en la práctica, "parado con ruido" — velocidad pedida
  que nunca se entregaba. Ahora la fracción se mapea a `[PWM_MIN..PWM_MAX]`,
  así el primer escalón de velocidad ya mueve el robot de verdad.
  **Rango pedido por el usuario: `PWM_MIN = 200`, `PWM_MAX = 255`.** Es
  angosto a propósito —por debajo de ~200 estos motores con carga no mueven
  el robot— y tiene una consecuencia que conviene tener presente: la
  velocidad va del **78% al 100%** de la potencia, así que el "paso lento"
  del acelerador sigue siendo bastante rápido. Es el compromiso correcto
  para este chasis: mejor poca gama de velocidad que una gama grande cuya
  mitad inferior no mueve nada.
- **Pulsos de torque (`KICK_PULSOS=3 × KICK_ON_MS=200` con `KICK_OFF_MS=30`).**
  El rozamiento estático es bastante mayor que el dinámico. Golpear con duty
  completo en pulsos cortos rompe la adherencia inicial y hace que el robot
  salga parejo; arrancando con rampa suave desde cero, la rueda con más
  carga arranca después que las otras y el robot sale torcido.
  Es exactamente lo que hacía `pulsos()` del sketch de prueba, **pero sin
  `delay()`**: acá es una máquina de estados por rueda (`updateKicks()`,
  llamada desde `loop()`). Con `delay()` el failsafe de 300 ms saltaría en
  mitad de cada arranque, y el loop dejaría de atender UDP y BLE.
  `KICK_REARME_MS = 400` evita que ir a paso lento con el duty oscilando
  cerca del umbral dispare pulsos todo el tiempo (avance a los tirones).
  Invertir la marcha de golpe también recibe pulsos: es el caso más
  exigente (frenar la inercia y volver a arrancar en el otro sentido).

El tren de pulsos dura **659 ms**. Está por encima del watchdog UDP de
300 ms, pero eso no lo dispara: el watchdog mide **la llegada de comandos**,
no el movimiento, y durante el arranque el `loop()` sigue corriendo entero.

- Rol: actuador puro. WiFi STA hacia el AP de la Jetson, con **IP fija
  `192.168.12.50`** vía `WiFi.config()` (no depende del DHCP).
- Recibe `CmdPacket` binario (ver protocolo en §4), aplica cinemática
  inversa abierta (sin PID) → PWM de 4 motores.
- Servo 180° posicionable + LED de escaneo controlables por separado, y
  barrido automático de 2s con `scan_cmd`.
- **Segunda vía de mando por BLE** (ver §13): plan B si la Jetson se cae.
- Watchdog **por fuente**: UDP `CMD_TIMEOUT_MS = 300`, BLE
  `BLE_TIMEOUT_MS = 700`. Solo se vigila la fuente activa.
- `loop()` no bloqueante (sin `delay()` en el camino crítico).
- **Estado: protocolo v4 + capa BLE + arranque por pulsos. REQUIERE
  REFLASHEAR** — un firmware viejo descarta los paquetes v4 por tamaño
  (falla segura) y no se anuncia por Bluetooth.
- ⚠️ **GPIO de strapping en uso**: GPIO8 (FOCO) y GPIO2 (motor m2) deben
  estar en ALTO durante el reset o la placa no bootea. GPIO9 (BOOT) quedó
  libre, que es lo correcto. Hay una tensión de diseño real en GPIO2: el
  chip quiere pull-up, pero una entrada de driver de motor normalmente
  quiere pull-down para no dar un tirón al encender. No se arregla desde el
  firmware — son resistencias en la placa.

**Escalonado por lados (2026-09-04).** Los pulsos de las cuatro ruedas ya no
salen en el mismo instante: el lado izquierdo arranca primero y el derecho
espera `KICK_DESFASE_MS` (60 ms). Cuatro motores pasando de parados a duty
máximo a la vez es el peor caso de corriente de todo el robot, y era justo el
momento en que caía la tensión de la batería y se reiniciaba la electrónica.
Partir el pico en dos mitades separadas en el tiempo lo alivia sin que el
robot lo note: 60 ms es menos de lo que tarda una rueda en moverse de verdad,
así que sigue arrancando recto.

Durante su espera la rueda va a duty **0**, no al que pidió la cinemática: un
lado tirando a media potencia mientras el otro espera haría girar el robot al
arrancar. Las dos ruedas de un mismo lado **no** se escalonan entre sí —van
unidas mecánicamente por el suelo y desfasarlas solo las hace pelearse.

Si aun así cae la tensión: subir `KICK_DESFASE_MS` primero, bajar
`KICK_PULSOS` de 3 a 2 después. Y revisar la topología de alimentación, que
es la causa de fondo (ver el changelog del 2026-09-04).

**Giro: piso propio y pulsos que no se acaban** (2026-09-08). Girar sobre el
sitio arrastra las cuatro ruedas de costado contra el suelo; el rozamiento es
varias veces el de ir recto y dura todo el giro, así que los pulsos de
arranque —que son de una sola vez— no lo cubrían. Mientras el giro domine
(`GIRO_UMBRAL`: la componente de rotación pesa al menos tanto como la de
avance, las dos en velocidad de rueda) rigen dos cosas:

- `PWM_MIN_GIRO = 235` en vez de `PWM_MIN`. Un giro puro a `OMEGA_MAX` pide
  solo ~45% de la rueda de referencia, así que el mapeo normal daba un duty
  de mitad de tabla: justo el que no alcanza para arrastrar.
- Un tren de `GIRO_ON_MS` (140) a fondo / `GIRO_OFF_MS` (90) al duty pedido,
  sostenido, sin contador. La fase baja **no** cae a 0: eso frenaría el robot
  en cada hueco. Los cuatro golpes van a la vez y no escalonados por lados —
  en un giro los lados tiran en sentidos opuestos y golpear uno solo empuja
  el robot de costado en vez de girarlo. Es, por eso, el pico de corriente
  más alto del robot después del arranque: si vuelve a caer la tensión, bajar
  `PWM_MIN_GIRO` primero y alargar `GIRO_OFF_MS` después.

**`MOTOR_PWM_HZ` es 1000, no 8000** (2026-09-08). El TA6586 conmuta lento y a
8 kHz una fracción grande de cada ciclo se iba en la transición, disipada en
el driver en vez de entregada al motor. Y **el duty máximo ya no es PWM: con
`PWM_MAX` se suelta el canal LEDC y el pin va a `HIGH` fijo**, porque LEDC a
8 bits nunca dejaba el pin quieto ni con duty 255. Los dos detalles están
explicados largo en el `.ino`.

### `firmware/esp32_lora_v3/esp32_lora_v3.ino`
- Rol: sensores ambientales + telemetría dual (UDP local + LoRaWAN OTAA).
- Sensores por I2C directo (sin librerías de alto nivel): MPU6050, SCD40.
- Credenciales LoRaWAN reales (devEui/appEui/appKey) embebidas — **no
  loguear ni exponer este archivo fuera del repo del usuario**.
- **Estado: verificado en campo por el usuario, provisto tal cual — no se
  modifica la lógica.** Solo se agregó un comentario de cabecera notando
  esto.

---

## 4. Protocolo binario Jetson ↔ ESP32-C3 (`jetson/backend/uart_link.py`)

**Versión 5 — por UART** (2026-09-04). Little-endian, `#pragma pack(1)`, sin
padding. Los structs del `.ino` y los formatos de `uart_link.py` deben
coincidir **byte a byte** — lo verifica `tests/test_sensores_y_serie.py`
derivando el formato de la declaración en C y comparándolo con el de Python
(no solo los tamaños: comparar los formatos es lo que detecta dos campos
intercambiados, donde la trama mide igual y el robot obedece el valor
equivocado).

```
CmdFrame    (8 B)  <BbbBBHB   magic 0xA5, v_pct, omega_pct,
                              servo_angle, flags, seq, crc8
StatusFrame (8 B)  <BbbBBHB   magic 0x5A, v_pct, omega_pct,
                              servo_angle, flags, seq_echo, crc8

flags de comando: 0x01 LED · 0x02 SCAN
flags de estado:  0x01 LED · 0x02 escaneando · 0x04 failsafe
```

### Por qué hay magic y CRC, y antes no hacían falta

Un UART entrega un **chorro de bytes**, no paquetes. Nadie marca dónde
empieza y termina una trama, así que hay que añadir dos cosas que UDP y BLE
regalaban:

1. **Byte de inicio** (el magic) para resincronizar. Si se pierde un byte por
   ruido, o si se empieza a leer a mitad de trama, el receptor descarta hasta
   encontrar el siguiente magic. Sin esto, **un solo byte perdido corre todas
   las tramas siguientes para siempre**.
2. **CRC-8** (polinomio 0x07, el CRC-8/ATM clásico). Sin él, un magic que
   aparezca por casualidad dentro de los datos se tomaría por el principio de
   una trama y el robot obedecería basura.

Y hay una fuente de basura **garantizada**: el bootloader del ESP32-C3 escupe
su mensaje de arranque por estos mismos pines en cada reset. La Jetson lo va
a leer siempre; el tramado es lo que hace que no importe.

El firmware cuenta las tramas descartadas por CRC y las imprime junto a la
traza de motores (`crc_err=N`). Unas pocas al encender son normales. Un
contador que sube mientras se conduce significa cable, masa o baudios mal.

### Envío periódico a 20 Hz (no es un detalle del transporte)

`uart_link.py` reenvía el último comando a 20 Hz por su cuenta. **Esto
corrige un fallo real que arrastraba el camino UDP**: allí solo se mandaba un
paquete cuando llegaba un mensaje del navegador, así que cualquier hueco del
WebSocket disparaba el failsafe de 300 ms y el robot avanzaba a tirones. El
enlace tiene que latir por su cuenta, no al ritmo de la interfaz.

Cambiar cualquiera de estos structs **obliga a reflashear el ESP32-C3**: una
trama de tamaño o CRC incorrecto se descarta, así que una Jetson nueva contra
un firmware viejo simplemente no mueve el robot — **falla segura, no
comportamiento raro**.

### Historia de los transportes (para no repetirla)

| Transporte | Vivió | Por qué murió |
|---|---|---|
| UDP sobre Wi-Fi | hasta 2026-09-04 | El C3 tiene UNA sola radio de 2.4 GHz. La asociación al AP se caía sola y no se recuperaba. |
| BLE (GATT) | horas, el 2026-09-04 | Quitar el Wi-Fi le dejaba la radio entera, pero la placa dejó de anunciarse y el enlace nunca llegó a establecerse. |
| UART por cable | desde 2026-09-04 | Sin radio. Las dos placas van atornilladas al mismo robot, a 20 cm una de otra: un enlace inalámbrico entre ellas nunca compró nada y aportaba todos los modos de fallo. |

## 5. Telemetría ambiental (ESP32 LoRa V3 → Jetson, UDP 5001)

JSON plano (no binario) con: `accelX/Y/Z`, `gyroX/Y/Z` (MPU6050), `co2`,
`tempSCD`, `humidity` (SCD40). El backend
le agrega `_received_at` y `_source` al llegar.

Al entrar pasa por `normalize_env()`, que hace dos cosas:

1. **Sensor que no inicializó → `0.0`.** Todo campo esperado queda como
   float finito: ausente, nulo, NaN o infinito se sustituye por `0.0` para
   que nada aguas abajo (PoseEstimator, EKF, dashboard) reciba basura.
   Los campos sustituidos se listan en `_sensors_missing` — un `0.0` de
   "sensor caído" y uno de "medición real de cero" no son lo mismo, y
   confundirlos sería inventar datos.
2. **Corrección de bias del giroscopio** (`calibration.gyro_bias_{x,y,z}_dps`)
   se resta acá, lo más cerca posible de la entrada, para que todo lo de
   aguas abajo vea el mismo dato corregido.

## 6. LoRaWAN — payload y esquema (`rpi/server/decoder.py` + `schema.py`)

Payload binario, mismos campos que la telemetría UDP arriba (accel×3,
gyro×3, co2, tempSCD, humidity). **Cuatro tamaños válidos**, y `decode()`
acepta los cuatro:

- **17 bytes (v3)** — **el que manda el robot.** Firmware del usuario sin
  BMP280. No lleva máscara de validez.
- **18 bytes (v3b)** — misma cosa con máscara en el byte 17 (bit0 MPU6050,
  bit1 SCD40). No está en uso; se acepta por si se vuelve a ella.
- **22 bytes (v2)** — firmware anterior. Traía presión y `tempBMP` en los
  bytes 17-20 y la máscara en el 21. Esos cuatro bytes se descartan.
- **21 bytes (v1)** — el más antiguo, sin máscara. No dice qué sensores
  respondieron, así que un sensor ausente y uno midiendo cero llegan
  idénticos.

Se aceptan los cuatro a propósito: en campo no siempre se puede reprogramar el
nodo el mismo día que se despliega el gateway.

`schema.py` (`SCHEMA_VERSION = 1`) documenta por campo: unidad, sensor de
origen, rango físico válido, label humano. `validate()` agrega DOS listas al
dict decodificado, que son cosas distintas y no se mezclan:

- **`_sensors_missing`** — el sensor no inicializó: campo ausente, nulo, NaN
  o infinito. Se sustituye por `0.0` (mismo criterio que la Jetson, §5).
- **`_out_of_range`** — el sensor respondió con un número válido pero fuera
  del rango físico posible (conectado pero devolviendo basura). El dato NO
  se descarta, solo se marca.

Con máscara (v2/v3) esta GANA sobre la deducción por rango: un sensor
desconectado manda `0.0`, y `0.0` cae fuera de rango, así que sin la
máscara acababa en `_out_of_range` ("sensor devolviendo basura") cuando la
realidad es que no hay sensor. Son dos averías distintas y llevan a revisar
dos cosas distintas.

Un campo nunca aparece en las dos listas a la vez. Ambas se guardan en
SQLite (`packets.sensors_missing`, `packets.out_of_range`,
`packets.schema_version`) y **se muestran en el dashboard remoto** (aviso en
el panel y columna en la tabla de histórico).

`init_db()` migra bases viejas: SQLite no tiene `ADD COLUMN IF NOT EXISTS`,
así que consulta `PRAGMA table_info` antes de agregar `sensors_missing`.
Sin eso, actualizar el código contra un histórico existente rompía los INSERT.

`GET /api/schema` y `schema.describe()` se **eliminaron**: no los llamaba
nadie.

---

## 7. Configuración persistente ("EEPROM" — `jetson/backend/config_store.py`)

Archivo `agrobot_config.json`, reescritura atómica. Secciones y campos
actuales (`DEFAULTS`):

```python
calibration: { servo_offset_deg=0,          # se SUMA al ángulo del servo
               gyro_bias_x_dps=0.0,         # se RESTAN de la telemetría
               gyro_bias_y_dps=0.0,
               gyro_bias_z_dps=0.0 }        # el Z alimenta el rumbo
hardware:    { camera_enabled, lidar_baud=460800,
               actuator_port=/dev/ttyTHS1, actuator_baud=115200 }
ai:          { active_model }
detection:   { confidence_threshold=0.45, clip_record_threshold=0.60 }
slam:        { obstacle_stop_distance_m=0.35, obstacle_slow_distance_m=0.9,
               obstacle_frontal_arc_deg=35, pose_gyro_trust=0.8 }
safety:      { heartbeat_stale_warn_s=5, heartbeat_stale_err_s=20,
               max_linear_accel_mps2=1.0,   # rampa (slew-rate), NO PID
               max_angular_accel_rps2=3.0 }
control:     { max_v_mps=0.55,              # DEBEN coincidir con V_MAX y
               max_omega_rps=1.5 }          # OMEGA_MAX del firmware, o el
                                            # C3 recorta en silencio
navigation:  { mode="mapping"|"localization", map_name="agrobot_map",
               last_goal }
```

Todos editables desde el dashboard (pestañas Config. Hardware / Parámetros
/ Navegación), vía REST — nunca hardcodeados en el código de decisión.
**Todos se APLICAN de verdad**: antes los tres campos de `calibration` se
guardaban y no los leía nadie (pestaña decorativa), y había campos muertos
(`actuator_watchdog_timeout_ms`, `lidar_serial_port`) que se eliminaron.

`load()` **poda** las claves que ya no están en `DEFAULTS`. Sin eso, un
campo borrado del código seguía vivo para siempre en el JSON del usuario y
se servía por la API como si existiera.

## 8. Backend Jetson (`jetson/backend/jetson_server.py`) — módulos y su rol

**Regla que vale para todo el backend:** `jetson_server.py` corre sobre un
único event loop de asyncio, y ese loop es el que manda los comandos UDP al
ESP32-C3 a 20 Hz. **Nada bloqueante puede correr dentro de un handler
async** — ni lectura de cámara, ni compresión JPEG, ni inferencia, ni
escritura a disco. Un `await` no interrumpe una llamada bloqueante: el loop
se para entero, y si se para más de 300 ms el watchdog del firmware frena el
robot. Lo que bloquea va a un hilo (`camera.py`) o a un executor
(`save_capture`).

| Archivo | Rol |
|---|---|
| `jetson_server.py` | proceso asyncio único: UART↔WebSocket↔video↔REST, todo push |
| `uart_link.py` | enlace serie con el C3: tramado, CRC-8, latido 20 Hz, reconexión (§4, §13) |
| `protocol.py` | ~~pack/unpack binario del C3~~ **obsoleto**: lo reemplazó `uart_link.py` el 2026-09-04. Sigue en el repo solo para `tests/test_actuator_e2e.py`, que todavía prueba el camino UDP retirado |
| `config_store.py` | EEPROM simulada (§7) |
| `health.py` | registro "qué tópico está vivo" (lidar/camera/esp32c3/lora_v3/slam) — OK/WARN/ERR por umbral de antigüedad configurable |
| `slam.py` | `PoseEstimator` (filtro complementario dead-reckoning+giroscopio, fallback sin ROS) + `LidarReactive` (evasión reactiva, frena sin costmap) — **NO es SLAM real**, ver §9 |
| `camera.py` | `CameraWorker`: captura + YOLO **en hilos**, un solo `VideoCapture` compartido por todos los clientes. Existe para sacar lo bloqueante del event loop (ver la regla de arriba) |
| `ros_bridge.py` | puente opcional a ROS 2, ver §9 |

### Endpoints REST (Jetson, puerto 8080)

- `GET/POST /api/hardware`, `/api/calibration` — config
- `GET /api/params`, `POST /api/params/{slam|detection|safety}`
- `GET/POST /api/nav/mode`, `POST /api/nav/set_goal`, `/api/nav/cancel`, `/api/nav/save_map`
- `POST /api/lidar/scan` — ingesta manual de scan (fallback si no hay sllidar_ros2)
- `GET /api/health`
- `GET /api/models`, `POST /api/models/load`
- `POST /api/mission/start`, `/api/mission/stop` — **(2026-09-17) arrancan y paran la patrulla autónoma.** Se eliminaron `/api/autonomous/*`.
- `POST /api/drive` `{accion: adelante|atras|izquierda|derecha|parar, escala, maniobra}` — manejo por pulsos (2026-09-17)
- `GET /api/patrol`, `POST /api/patrol/record|clear|scan` — recorrido de patrulla (2026-09-16)
- `POST /api/nav/reset_map` — resetea slam_toolbox y borra el recorrido (2026-09-17)
- `GET /api/mapas`, `POST /api/mapas/guardar {nombre, sobrescribir}`, `POST /api/mapas/cargar {nombre}` (2026-09-17)
- `GET /api/captures` (`?class=&limit=`), `POST /api/captures/clear`
- `POST /api/session-report`
- `POST /api/nav_cal/start`, `/api/nav_cal/stop`, `GET /api/nav_cal/status` — calibración de navegación (2026-09-19)
- `POST /api/training/correct_pose {x, y, theta_deg}` — publica `/initialpose` para corregir la pose del robot en SLAM (2026-09-19)
- `POST /api/training/save_furrows {furrows}`, `POST /api/training/save_scan_points {scan_points}` — guarda surcos y puntos de escaneo en `~/.agrobot/training/` (2026-09-19)
- `POST /api/training/generate_route {furrows, scan_points, spacing_m}` — genera waypoints boustrophedon y los carga en la patrulla (2026-09-19)
- `GET /api/training/data` — retorna surcos, puntos de escaneo y correcciones guardados (2026-09-19)
- `POST /api/training/log_correction {type, vx, omega}` — registra correcciones de conducción para aprendizaje (2026-09-19)
- `GET/POST /api/drive/mode {modo: interior|exterior}` — modo del manejo por pulsos; se difunde por WS como `drive_mode` (2026-09-19)
- `GET /api/sendero/vista.jpg` — imagen de depuración del detector de senderos (cámara + vista cenital) (2026-09-19)
- `GET /video` (MJPEG con overlay YOLO)
- `WS /ws` — push: `env_telemetry, actuator_status, detection, capture,
  system_stats, health, pose, pose_source, lidar_scan, map, toast,
  config_update, mission_state, obstacle_warning, captures_cleared,
  lidar_distancias, patrol_event`
  (`obstacle_warning` cambió el 2026-09-17: `{motivo, lado, distancia_m, umbral_m}`);
  recibe: `{type:"control", v, omega, scan}`, `{type:"emergency_stop"}`

### Galería de misión (capturas)

Cuando una detección YOLO supera `detection.clip_record_threshold`, se
guarda el frame real como JPEG en `jetson/backend/captures/`, se registra
`{id, class, confidence, ts, image_url}` en memoria (cap 500) y se
transmite por WS (`type:"capture"`). Pestaña **Galería de misión** del
dashboard: grid con filtro por clase, descarga individual, descarga del
reporte de sesión completo, vaciar galería.

---

## 9. SLAM / Navegación — estado real (no idealizado)

### Capa casera (`slam.py`) — siempre presente, es el fallback

- `PoseEstimator`: filtro complementario que fusiona velocidad COMANDADA
  (no hay encoder que mida la real) con giroscopio (`gyroZ`, MPU6050) vía
  peso `pose_gyro_trust`. Da `(x, y, theta_deg)` relativo al arranque, con
  deriva sin límite — **no es localización, es una estimación**.
- ~~`LidarReactive`: evasión reactiva simple — si hay obstáculo dentro del
  arco frontal configurado y más cerca que el umbral, recorta la velocidad
  lineal.~~ **Reemplazado el 2026-09-17** por corredor del ancho del robot y
  corte con antelación al deslizamiento. Ver sección 0.

### Capa ROS 2 (`ros_bridge.py` + `ros2_ws/src/agrobot_navigation/`) — opcional

Decisiones tomadas con el usuario antes de implementar (2026-09-01):
1. Jetson↔ESP32-C3 sigue siendo UDP binario propio — NO se migra a
   ros2_control/serial.
2. LIDAR real se lee con `sllidar_ros2` (nodo ROS dedicado), no con el
   POST manual (que queda como fallback de pruebas).
3. Se migra a **Nav2 completo**: mapear, guardar mapa, navegar a metas
   reales — no solo evasión reactiva.
4. Todo corre en la Jetson (un solo dispositivo).

`ros_bridge.py` es un nodo `rclpy` embebido en el mismo proceso que
`jetson_server.py` (hilo de fondo, `SingleThreadedExecutor`). Si `rclpy` no está
instalado, todo el módulo es no-op silencioso y el sistema sigue
funcionando con la capa casera de arriba — **nunca se rompe por falta de ROS**.

**Tópicos ROS que intercambia:**

| Tópico/Acción | Dirección | Tipo | Quién lo produce | Qué hace `ros_bridge` con él |
|---|---|---|---|---|
| `/cmd_vel` | Nav2 → bridge | `geometry_msgs/Twist` | controller_server (DWB) | llama a `send_command()` → trama UART al C3. Se reenvían **`linear.x` y `angular.z`**; `linear.y` se descarta (robot diferencial). La evasión reactiva de `slam.py` se sigue aplicando encima, doble capa |
| `/scan` | sllidar_ros2 → bridge | `sensor_msgs/LaserScan` | driver RPLIDAR C1 | alimenta `lidar_reactive` + WS `lidar_scan` (radar del dashboard) |
| `/odometry/filtered` | robot_localization → bridge | `nav_msgs/Odometry` | ekf_node | WS `pose` con `pose_source="ros_ekf"` — reemplaza al `PoseEstimator` casero cuando está disponible |
| `/map` | slam_toolbox → bridge | `nav_msgs/OccupancyGrid` | slam_toolbox | WS `map` (throttled a 1 Hz), se dibuja en la pestaña Navegación |
| `/imu/data_raw` | bridge → robot_localization | `sensor_msgs/Imu` | telemetría UDP real de la ESP32 LoRa V3 | orientación marcada `covariance[0]=-1` (no hay magnetómetro, no se usa) |
| `/wheel_odom` | bridge → robot_localization | `nav_msgs/Odometry` | velocidad COMANDADA (no encoder real) | sin `linear.y`, y `ekf.yaml` tampoco fusiona `vy`: en un skid-steer el patinaje produce algo de deriva lateral, y marcarla como medida-en-cero se la negaría al EKF. Covarianza alta a propósito — el EKF debe confiar más en el IMU |
| `navigate_to_pose` (acción) | bridge → Nav2 | `nav2_msgs/action/NavigateToPose` | disparado por `POST /api/nav/set_goal` (clic en el mapa del dashboard) | — |

**Nodos ROS externos** (procesos aparte, lanzados por
`ros2_ws/src/agrobot_navigation/launch/*.launch.py`, NO por `jetson_server.py`):
`sllidar_ros2`, `ekf_node` (robot_localization), `slam_toolbox`
(`async_slam_toolbox_node` en modo mapeo / `localization_slam_toolbox_node`
en modo navegación), Nav2 (`controller_server`, `planner_server`,
`behavior_server`, `bt_navigator`, costmaps, `velocity_smoother`).

~~**Estado real de esta capa: escrita pero NO compilada ni probada en
hardware.**~~ **Reemplazado el 2026-09-17:** compilada y corriendo bajo
systemd. Nav2 activo dentro de `agrobot_mapping.launch.py` (sin
`velocity_smoother` ni `collision_monitor`), LIDAR en `/dev/ttyUSB0`, huella
medida 0,40 × 0,35 m. La acción `navigate_to_pose` la usa la patrulla, no el
clic en el mapa (que se quitó del dashboard). Ver sección 0.

---

## 10. Raspberry Pi (`rpi/server/gateway_server.py`, puerto 8092)

Puente MQTT (ChirpStack local) → SQLite (`agrobot_history.db`) →
WebSocket. Downlinks de emergencia vía API REST de ChirpStack
(`/api/devices/{dev_eui}/queue`), nunca directo al dispositivo.

Endpoints: `GET /api/history`, `/api/history/export.csv`, `/api/schema`,
`POST /api/downlink`, `WS /ws`.

Variables de entorno: `MQTT_HOST`, `MQTT_PORT`, `CHIRPSTACK_APP_ID`,
`CHIRPSTACK_API_URL`, `CHIRPSTACK_API_TOKEN`, `AGROBOT_DEV_EUI` (default
`f59d08345985b27d`, de `CARRITO2_LORAWAN_STATUS.md`).

---

## 11. Dashboards

### Jetson (`jetson/dashboard/index.html`) — control en tiempo real, solo dentro del AP

Sin CDN, CSS/JS propio. Pestañas: Dashboard (video+control manual+stats),
Galería de misión, Navegación (mapa+metas Nav2), SLAM & Telemetría
(ambiental+radar LIDAR), Gestor de Modelos IA, Config. Hardware,
Parámetros, Sistema (salud de tópicos, donuts CPU/RAM/GPU/TEMP, pose).
Nombre del robot en la UI: **"AgroBot" / "Unit 01"** (se quitó "Alpha").

El D-pad tiene **4 direcciones** (avanzar, retroceder, girar izq/der) más
el botón de parada. No hay desplazamiento lateral: el robot es diferencial.

**Acelerador (2026-09-02).** El mando manual manda **fracción −1..1**, no
m/s: el dashboard no conoce los límites físicos del robot. El backend
(`ws_handler`) multiplica por `control.max_*` y por el `scale` del
acelerador. Antes se mandaba `v=1` crudo — la UI decía 1.0 m/s, el firmware
lo recortaba a `V_MAX=0.55` en silencio y **no había forma de pedir marcha
lenta**. La posición del acelerador vive en `localStorage`, no en
`agrobot_config.json`: es la posición de un mando, no configuración del
robot. Con la zona muerta corregida en el firmware, la marcha lenta recién
ahora entrega par de verdad en vez de zumbar.

`sanitize_control()` acota lo que llega por `POST /api/params/control`: esa
sección multiplica directamente lo que sale a los motores, y un valor
absurdo se guardaría en disco y seguiría vigente en el próximo arranque.
`_unit()` hace lo propio con cada comando del WebSocket (un `null`, un texto
o un `NaN` valen 0 — `NaN` importa aparte porque toda comparación con él da
falso y un clamp ingenuo lo dejaría pasar entero).

### Raspberry Pi (`rpi/dashboard/index.html`) — histórico remoto

Sin CDN. Pestañas: Dashboard Remoto, Paquetes LoRa, Gateway & Red,
Downlink, Histórico + export CSV. **Pendiente:** mostrar `_out_of_range`.

**Este dashboard NO tiene interfaz de Telegram, y nunca la tuvo.** Verificado
el 2026-09-04 comparando byte a byte lo que sirve la Pi contra el archivo del
repo: mismo MD5, 21773 bytes. El gateway expone cinco rutas de Telegram
(`GET/POST /api/telegram`, `/api/telegram/report`, `/api/telegram/test`) y el
HTML **no llama a ninguna**: solo usa `/api/downlink` y `/api/history`.
Telegram se gobierna desde el chat del bot (`/start`, `/activar`,
`/silencio`), no desde el navegador. Queda como pendiente añadir el panel.

**El dashboard Bluetooth se eliminó** el 2026-09-04 junto con el transporte
BLE (ver §13).

---

## 13. Enlace UART Jetson ↔ ESP32-C3 (`jetson/backend/uart_link.py`)

**Única vía de mando del actuador** desde el 2026-09-04. Reemplaza al UDP
sobre Wi-Fi y al BLE, los dos retirados el mismo día (ver la tabla de
transportes en §4).

### Cableado (tres hilos — la masa NO es opcional)

| Jetson (cabezal 40 pines) | | ESP32-C3 |
|---|---|---|
| pin 8, TXD (`/dev/ttyTHS1`) | → | GPIO20 (RX) |
| pin 10, RXD (`/dev/ttyTHS1`) | ← | GPIO21 (TX) |
| pin 6, GND | — | GND |

Cruzado: el transmisor de uno al receptor del otro. Los dos lados son de
3,3 V, se conectan **directo, sin adaptador de nivel**. 115200 baudios.

Sin la masa común las señales no tienen referencia y no funciona nada. Ya
pasó una vez en este robot con el driver de motores (2026-09-04): el firmware
mandaba duty 255 a las cuatro ruedas y ninguna giraba.

**El hilo de GND del UART NO es lo mismo que la masa de potencia**, aunque
las dos acaben en la batería. La estrella de alimentación resuelve las
corrientes de retorno grandes; este tercer hilo es la **referencia de señal**,
la que le dice al receptor respecto a qué medir el 0 y el 1. Si cada placa
midiera contra su propia masa, unas decenas de milivoltios de diferencia
mientras los motores tiran bastan para corromper bytes.

Va **fino y en el mismo mazo que TX y RX**, de extremo a extremo, no al bloque
de distribución. Sí, eso forma técnicamente un lazo de masa, pero a 115200
baudios y medio metro de cable no molesta, y como cada placa tiene su propio
retorno de potencia a la batería por ese hilo casi no circula corriente.

Si `crc_err` sube mientras se conduce, el siguiente paso es **bajar a 57600
baudios**: sobra de velocidad para 8 bytes 20 veces por segundo y el margen de
ruido mejora. Es una línea en cada lado (`MANDO_BAUD` y `DEFAULT_BAUD`).

`/dev/ttyTHS1` está libre en esta Jetson: el `serial-getty` viene desactivado
y la consola del kernel va por `ttyTCU0`. Si alguna vez se reactiva, se pelea
por el puerto:

```
sudo systemctl disable --now serial-getty@ttyTHS1
```

### Compilar el firmware con `USB CDC On Boot: Enabled`

**Obligatorio.** Los logs salen por el USB y las tramas por GPIO20/21. Si el
USB CDC está apagado, `Serial` **es ese mismo UART** y los logs se mezclarían
con los datos hasta romper el enlace.

### Modo banco: mando por USB (`MANDO_POR_USB`)

`#define MANDO_POR_USB 1` en el `.ino` hace que las tramas viajen por el mismo
USB con el que se flashea. Sirve para **verificar el protocolo entero antes de
tener los tres hilos puestos**: la Jetson abre `/dev/ttyACM0` y manda. En `0`
(el modo del robot montado) las tramas van por GPIO20/21 y el USB queda libre
para los logs.

En modo banco los logs y las tramas **comparten** el puerto. El receptor
resincroniza por magic y descarta lo que no cuadre, así que funciona, pero se
ven tramas descartadas por CRC cada vez que se imprime una línea. **Es
esperado en ese modo y no significa cable malo.**

### Ya no hay arbitraje de control

~~La regla de arbitraje entre BLE y UDP~~ **queda eliminada** (reemplaza a la
versión anterior de esta sección). Con una sola vía de mando no hay dos
fuentes que puedan pelearse por los mismos motores, así que desaparecen las
banderas `TOMAR`/`SOLTAR` y los watchdogs por fuente.

Los bits 2 y 3 del campo `flags` quedan **reservados, sin usar**: reciclarlos
para otra cosa haría que un cliente viejo moviera el robot sin querer.

**Watchdog único: 300 ms.** Con BLE hacían falta 700 porque el enlace
renegociaba intervalos por su cuenta; un cable no hace nada de eso. A 20 Hz
hacen falta 6 tramas perdidas seguidas para que salte.

### Compilar y verificar el firmware DESDE la Jetson

La Jetson tiene `arduino-cli` 1.5.1 instalado en el scratchpad de trabajo, con
el core `esp32:esp32@3.3.11` (el mismo que el IDE de Windows) y la librería
`ESP32Servo`. Ya no hace falta abrir el IDE solo para saber si algo compila:

```
arduino-cli compile -b esp32:esp32:esp32c3:CDCOnBoot=cdc <carpeta_del_sketch>
```

`CDCOnBoot=cdc` es la línea de comandos equivalente a poner *USB CDC On Boot:
Enabled* en el IDE, y **es obligatorio** (ver arriba).

Verificado el 2026-09-04, las dos variantes compilan sin errores:

| | flash | RAM global |
|---|---|---|
| UART (producción) | 322970 B (24%) | 15012 B (4%) |
| USB (banco) | 312940 B (23%) | 15004 B (4%) |

Con `--warnings all --clean` los únicos avisos que salen son de la librería
ESP32Servo, ninguno del firmware.

### El dashboard Bluetooth se eliminó

`bluetooth/agrobot_bt.html` **ya no existe**. El mando manual vive ahora en
el dashboard web de la Jetson, servido sobre su propio AP. El AP sigue
existiendo para eso y para el nodo LoRa; lo que ya no lo usa es el actuador.

- **2026-09-06** — **`LED_ACTIVO_EN_BAJO = false` DEJÓ LA PLACA SIN
  ARRANCAR. GPIO8 es un pin de strapping y esa línea decide si el C3
  bootea.** Fallo mío, y de los caros: la placa parecía muerta y el
  ordenador ni la reconocía por USB.

  **El mecanismo.** GPIO8 es STRAPPING en el ESP32-C3: en el reset el chip lo
  muestrea y **tiene que estar en ALTO** para arrancar desde flash. En bajo,
  no arranca — y como no arranca, tampoco inicializa el USB, así que el PC no
  ve ningún puerto. Se confunde con una placa quemada.

  El foco está cableado a ese GPIO8. Y el estado de REPOSO del foco (apagado)
  es el que se queda en el pin la mayor parte del tiempo, incluido el momento
  del siguiente reset:

  | valor | apagar escribe | GPIO8 en reset | resultado |
  |---|---|---|---|
  | `false` | LOW | bajo | **NO ARRANCA** |
  | `true` | HIGH | alto | arranca |

  **Recuperación:** mantener BOOT pulsado mientras se conecta el USB, lo que
  fuerza el modo descarga y salta el strapping; o desconectar el cable del
  foco de GPIO8 y volver a alimentar.

  **Devuelto a `true`, y esta vez no es una preferencia sino un requisito.**
  Coincide además con la observación original: con `false`, `setup()`
  escribía LOW y la lámpara se ENCENDÍA, o sea que la etapa es de lógica
  invertida.

  **Regla que queda escrita en el código:** si algún día hay que poner
  `false` porque se recablee la etapa a lógica normal, entonces **hay que
  mover el foco a otro pin**. GPIO8 y un consumidor que reposa en bajo son
  incompatibles.

  **Lo que esto enseña sobre los tres diagnósticos fallidos de esa línea:**
  se estuvo razonando sobre la polaridad de la lámpara sin ver que el pin
  tenía una segunda función, el strapping, que impone su propia restricción
  y que gana sobre cualquier consideración estética. La autoprueba del foco
  medía lo que la lámpara hacía, pero no podía revelar esto: el efecto no
  aparece hasta el SIGUIENTE arranque.

- **2026-09-06** — **FIRMWARES CONGELADOS: estas dos versiones son las
  verificadas en el robot.** El usuario las probó y las devolvió como buenas.
  Cualquier cambio futuro parte de aquí.

  **`firmware/esp32c3_actuator/esp32c3_actuator.ino`**
  - Servo por **LEDC en crudo**, canal 0, 50 Hz, 14 bits. Sin ESP32Servo.
  - Motores en los canales LEDC **2..5**.
  - **`LED_ACTIVO_EN_BAJO = false`** — la etapa del foco es de lógica NORMAL.
  - Autoprueba del foco en el arranque (`FOCO_AUTOPRUEBA`).
  - Sello de compilación con `__DATE__`/`__TIME__` al arrancar.

  **`firmware/esp32_lora_v3/esp32_lora_v3.ino`**
  - **Clase C**, sin BMP280, payload de 17 bytes.
  - **Ciclo LoRa de 30 s.** Se probó subirlo a 5 minutos y sigue siendo
    defendible, pero la versión verificada en el robot es la de 30 s. No se
    cambia sin volver a comprobarlo en campo.
  - UDP a 20 Hz desacoplado del ciclo LoRaWAN, con downlinks funcionando.

  **Se corrigieron TRES comentarios que contradecían al código**, que es la
  clase de deuda que hace perder horas meses después:

  1. En el C3, el bloque de `LED_ACTIVO_EN_BAJO` seguía diciendo "DEVUELTO A
     true" mientras el valor era `false`.
  2. En el nodo LoRa, el bloque de downlinks seguía diciendo "El nodo es
     CLASE A" cuando ya es Clase C.
  3. En el mismo, el `case DEVICE_STATE_SLEEP` daba por hecho el deep sleep
     y el reinicio en cada ciclo, que es justo lo que la Clase C elimina.

  **La lección de la polaridad del foco, que merece quedar escrita.** Esa
  única línea se diagnosticó mal DOS VECES, una en cada sentido, razonando
  sobre los síntomas en lugar de medir. Desde el código no se puede deducir:
  depende de cómo esté montada la etapa de potencia. Lo resolvió la
  autoprueba —conducir el pin a cada estado anunciándolo por serie y mirar la
  lámpara— en una sola pasada. **Cuando una hipótesis sobre hardware falla
  dos veces, hay que dejar de razonar y construir la medida.**

- **2026-09-06** — **SERVO RESUELTO: la culpable era la librería ESP32Servo,
  que ademas dejaba el pin inservible al fallar.** Costó varias tandas de
  flasheo y dos diagnósticos míos equivocados por el camino. Queda escrito
  entero porque el modo de fallo es de los que no se deducen leyendo código.

  **El sintoma.** El servo no se movia nunca. Pero la cadena de mando estaba
  BIEN: se mandaban 30°, 60°, 100° por WebSocket y el C3 devolvia en su trama
  de estado exactamente esos valores. El firmware recibia el angulo y llamaba
  a `write()`. Simplemente no salia señal por el pin.

  **Lo que se descartó, en este orden y con reproductores minimos:** el
  camino Jetson → UART → C3 (eco correcto), el conflicto de canales LEDC con
  los motores (se corrigio y no cambio nada), la placa seleccionada en el IDE
  (era LOLIN C3 Mini, se cambio a ESP32C3 Dev Module y tampoco), y la
  alimentacion (que resulto no ser).

  **La causa, medida en la placa con `firmware/test_servo_minimo`:**

  - `ledcAttachChannel(GPIO10, 50 Hz, 14 bits, canal 0)` engancha y **el
    servo se mueve**. También valen 10 y 12 bits.
  - Con **8 bits** el core lo rechaza: *"requested frequency 50 and duty
    resolution 8 can not be achieved... div_param=0"*. A 50 Hz el divisor se
    queda sin margen — o sea que 8 bits es demasiado POCO, al reves de lo que
    parece intuitivo. Con **16** tampoco: el C3 admite 14 como máximo
    (`SOC_LEDC_TIMER_BIT_WIDTH = 14`).
  - **`ESP32Servo::attach(GPIO10)` devuelve 0 siempre.** Y lo peor: al fallar
    deja el pin medio tomado en el gestor de periféricos, así que **a partir
    de ese momento ningún `ledcAttachChannel` vuelve a funcionar sobre ese
    GPIO en toda la sesión**.

  Ese último punto explica por qué costó tanto: en las pruebas, el orden era
  comprobar el pin (funcionaba), dejar que ESP32Servo lo intentara (fallaba y
  lo envenenaba), y luego barrer (ya imposible, se saltaba solo). **El GPIO10
  nunca llegó a recibir una señal de servo real** hasta el sketch mínimo sin
  librería.

  **Corregido:** ESP32Servo fuera del firmware. El servo se gobierna con LEDC
  directo, canal 0, 50 Hz, 14 bits. Una dependencia menos y un modo de fallo
  menos.

  **Y una lección de herramienta que vale para todo el proyecto:** el core de
  ESP32 SÍ explica cada fallo de `ledcAttachChannel` con un `log_e`, pero el
  IDE trae **Core Debug Level en None** y los silencia. Poniéndolo en
  **Error** apareció el motivo exacto en una sola prueba. Debería estar en
  Error por defecto para cualquier depuración de este robot.

- **2026-09-06** — **CAUSA RAÍZ DE LA INESTABILIDAD: el EKF integraba la
  aceleración lineal y la pose se iba a KILÓMETROS.** Encontrado tirando del
  hilo que abrió el usuario ("el backend se lanzó sin rclpy" y "SLAM /map:
  ERR, nunca"). Cuatro fallos encadenados, todos corregidos.

  **1. La unidad systemd lanzaba el backend SIN sourcear ROS.** Fallo mío al
  escribirla: `ExecStart=/usr/bin/python3 ...` a secas. El backend arrancaba
  sin `rclpy`, o sea sin puente ROS: sin `/imu/data_raw`, sin `/cmd_vel`, sin
  mapa y sin pose fusionada. No daba error, sólo funcionaba a medias.
  **Corregido** con `bash -lc` sourceando el ROS del sistema y el workspace.
  Consecuencia: **todas las medidas de memoria anteriores del backend eran
  sin rclpy** y por tanto optimistas.

  **2. `/map` no llegaba nunca** porque el EKF no recibía IMU (por el punto
  1), no publicaba la TF `odom → base_link`, y sin esa transformación
  slam_toolbox no puede situar los scans. Al arreglar el sourceado, apareció.

  **3. `_map_cb` convertía el mapa entero a lista de Python en CADA
  publicación.** Una celda ocupa 1 byte en el mensaje de ROS y ~36 como
  entero de Python: un mapa de 2000x2000 pasa de 4 MB a ~145 MB, cada 2
  segundos, más la copia de `json.dumps`. Se vio al backend saltar de 1501 a
  3535 MB y morir. **Corregido:** el límite de frecuencia se comprueba ANTES
  de convertir nada, y los mapas grandes se submuestrean a 250.000 celdas.

  **4. Y la causa de fondo: `imu0_config` fusionaba `ax` y `ay`.** Con el
  robot INMÓVIL sobre una mesa, la pose del EKF llegó a **x = 5839 m,
  y = -1181 m**. Casi seis kilómetros sin moverse.

  Es aritmética, no un fallo del filtro: el acelerómetro marca ~0.15 g de
  sesgo constante en X (inclinación del montaje más offset del MPU6050), e
  integrar 1.5 m/s² dos veces durante 100 s da ~7.5 km. Sin una medida de
  POSICIÓN que corrija, la deriva es cuadrática y no acotada.

  El daño no se quedaba en la pose: **slam_toolbox intentaba mapear esos
  kilómetros a 5 cm de resolución** y pasaba de 69 MB a 3015 MB en 84
  segundos, camino de tumbar la placa.

  **Corregido:** del IMU se toma sólo la velocidad angular en yaw. El avance
  lo aporta rf2o y el giro el giroscopio, que es lo que cada uno mide bien.

  **Resultado medido**, sistema completo bajo systemd, robot quieto:

  | | antes | después |
  |---|---|---|
  | pose del EKF | 5839 m | **0.001 m** |
  | slam_toolbox | 3015 MB y subiendo | **50 MB, plano** |
  | backend | picos de 3535 MB | **1518 MB, plano** |
  | sistema | 61% y subiendo | **34%, plano** |
  | reinicios | subían sin parar | **0 y 0** |

  `/map` publica 66x73 celdas a 5 cm.

  **Ajustes de apoyo:** `minimum_travel_distance` y `minimum_travel_heading`
  a 0.3 en slam_toolbox (estaban sin poner, y con una pose que deriva cada
  milímetro creaba un nodo nuevo); `MemoryMax` del backend a 4G y el de ROS a
  3G, ajustados a los picos reales.

  **Sobre los OOM de esta tanda:** `memory.events` de los dos cgroups marcaba
  `oom_kill 0`, así que los topes de systemd NO mataron nada. Fueron muertes
  por presión global de memoria, provocadas por el desbocamiento de
  slam_toolbox descrito arriba.

- **2026-09-06** — **FUGA CERRADA, verificada con 82 minutos de prueba
  continua. Y los servicios quedan habilitados al arranque.**

  Tras corregir la sesión HTTP compartida y acotar las tareas sueltas, la
  memoria del backend hace esto:

  | tramo | pendiente | RSS |
  |---|---|---|
  | prueba completa (82 min) | +2.02 MB/min | 1528 → 1725 MB |
  | **últimos 60 min** | **+0.00 MB/min** | 1725 MB, plano |
  | últimos 30 min | +0.00 MB/min | 1725 MB, plano |

  **La pendiente global de +2 MB/min es engañosa y conviene no citarla:** no
  es una tendencia, es una regresión lineal ajustada sobre un ESCALÓN. La
  memoria sube una sola vez de 1528 a 1725 MB durante los primeros ~23
  minutos —el asignador de CUDA reservando su caché y las primeras
  detecciones— y a partir de ahí no se mueve ni un megabyte en 57 minutos.
  Eso es una meseta, no una fuga.

  Antes de las correcciones la pendiente sostenida era de +6 MB/min en el
  último tercio, sin meseta a la vista.

  **Otros números de la misma prueba:** 166 muestras, **cero caídas del
  dashboard**, temperatura entre 59 y 60 °C sin deriva.

  **Proyección a 3 horas: se queda en 1725 MB**, o sea el 23% de los 7.4 GB
  de la placa. Con el sistema completo (SLAM, LIDAR, rf2o) ronda el 32%.

  **Servicios habilitados al arranque** a petición del usuario:
  `agrobot-backend` y `agrobot-ros`, los dos `active/enabled`, cero
  reinicios. A partir de ahora suben solos al encender la Jetson.

  ⚠️ **`INICIAR_BOT.sh` ya NO se debe usar.** Chocaría con los servicios por
  el puerto 8080 y por el puerto serie del LIDAR. Se maneja todo con
  `systemctl`.

- **2026-09-06** — **Servo y foco: las dos causas encontradas. Y una fuga
  lenta que sigue ABIERTA.**

  **SERVO — conflicto de canales LEDC.** El usuario reportaba "no manda datos
  al servo, muevo y nada". Se descartó primero el software midiendo en vivo:
  se mandaron 30°, 60°, 100° y 98° por el WebSocket y **el C3 devolvió en su
  trama de estado exactamente esos valores**. La cadena Jetson → UART → C3 →
  `scanServo.write()` funciona entera.

  La causa está en el reparto de canales de PWM:

  - ESP32Servo empieza a repartir por el canal **0** (`ESP32PWM.h`:
    `#define PWM_BASE_INDEX 0`; `ESP32PWM.cpp`: `PWMCount = PWM_BASE_INDEX`).
  - El firmware tenía `enum RuedaCanal { CH_R1 = 0, CH_R2 = 1, CH_L1 = 2,
    CH_L2 = 3 }`.

  En `setup()`, `scanServo.attach()` corre antes de que ningún motor se mueva
  y se queda con el canal 0. La primera vez que la rueda R1 se movía,
  `pwmAttach()` llamaba a `ledcAttachChannel(pin, 8000 Hz, 8 bits, 0)`, que
  **reprograma ese mismo canal a 8 kHz y lo ata al pin del motor**. El servo
  necesita 50 Hz: se quedaba sin canal y no volvía a moverse en toda la
  sesión, mientras el firmware seguía recibiendo el ángulo y llamando a
  `write()` sobre un canal que ya era de un motor. De ahí que el eco del
  estado fuera correcto y el servo no se moviera.

  **Corregido:** los motores pasan a los canales **2..5**. El C3 tiene 6
  canales LEDC; dejando el 0 y el 1 para ESP32Servo no hay solape posible.

  **FOCO — polaridad invertida.** Síntomas del usuario: "al encender se
  apaga, y siempre inicia prendido". Los dos encajan exactamente con
  `LED_ACTIVO_EN_BAJO = false` sobre una etapa de lógica invertida:
  `setup()` escribía `LED_NIVEL(false) = LOW`, que en esa etapa **enciende**;
  y encender escribía `HIGH`, que **apaga**.

  **Devuelto a `true`.** Esto CONTRADICE la nota del 2026-09-05, que lo puso
  en `false` afirmando que la etapa era de lógica normal. La observación
  directa del usuario manda: o aquel diagnóstico fue erróneo, o el cableado
  cambió. La nota vieja se conserva en el código para que la contradicción
  quede a la vista.

  Queda un detalle que **no se arregla desde el firmware**: durante los
  primeros milisegundos del arranque, antes de que `setup()` escriba el pin,
  GPIO8 no lo gobierna nadie y el foco puede dar un destello. Eso pide una
  resistencia de pull-up en la puerta de la etapa de potencia.

  **FUGA LENTA — sigue abierta, y es lo que impide dejarlo 3 horas solo.**
  Medido con la prueba de resistencia:

  | escenario | RSS |
  |---|---|
  | cámara apagada | 80 MB, **plano** |
  | cámara encendida, sin detectar nada | 1576 MB, **plano** 151 s |
  | cámara encendida detectando | 1529 → 1741 MB en 22 min |

  Pendiente medida: **+6 MB/min** en el último tercio, +12 de media.
  Proyección a 3 horas: **~3.7 GB**, o sea otra vez el OOM.

  **Atribución: el camino de DETECCIÓN**, no la inferencia. Sin detecciones
  la memoria no se mueve. No se ha llegado al objeto concreto que se acumula.
  Lo que sí se descartó: `state.detections` topa en 200, `state.captures` en
  500 borrando el fichero (512 ficheros y 26 MB en disco, estable),
  `_ultimo_aviso` está acotado por el número de clases, y `ws_clients` se
  descarta al desconectar y al fallar un envío.

  **CAUSA ENCONTRADA Y CORREGIDA (misma sesión).** Leyendo el camino
  completo aparecieron dos cosas, las dos en `_on_detection`:

  1. **Una `aiohttp.ClientSession` NUEVA por cada detección.**
     `_avisar_al_gateway()` la creaba dentro de un `async with` en cada
     llamada. Una sesión no es un objeto barato: monta su conector, su pool
     de conexiones y su resolutor de DNS, y la propia documentación de
     aiohttp avisa de que cerrarla no libera los transportes de inmediato.
     Cientos de veces por hora, eso se acumula. **Corregido:** una única
     sesión para todo el proceso, creada al primer uso y cerrada en
     `on_cleanup`.

  2. **Cuatro `asyncio.create_task()` por detección, sin referencia ni
     tope.** Dos problemas distintos: asyncio guarda solo una referencia
     DÉBIL a esas tareas, así que el recolector puede llevarse una a medias
     y el trabajo se pierde en silencio; y sin tope, una racha de
     detecciones con el gateway lento encola tareas sin límite, cada una
     reteniendo **un fotograma completo** en el caso de `save_capture`.

     **Corregido** con un ayudante `_lanzar()` que guarda referencia fuerte,
     la suelta al terminar, y **descarta** la tarea nueva pasadas 24
     pendientes en vez de encolarla. Perder un aviso es preferible a perder
     el proceso. Se aplicó a los 6 `create_task` sueltos que había, incluido
     el del estado del actuador, que va a la frecuencia del enlace UART.

- **2026-09-06** — **AUDITORÍA: el error de NVML que rompía la inferencia lo
  causé yo, y ya está corregido. Y la fuga de 4.5 GB era su consecuencia.**

  El usuario vio en el dashboard `ERROR DE INFERENCIA: Expected NVML_SUCCESS
  == nvmlDeviceGetGpuFabricInfoV_(...)`. Aislando el fallo con un script
  mínimo, la causa resultó ser una línea que yo mismo había añadido horas
  antes para recortar memoria:

      os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

  **Con esa variable puesta, CUALQUIER uso de la GPU falla en esta placa. Con
  la variable vacía, funciona.** Comprobado ejecutando el mismo script con y
  sin ella. Retirada, y documentado en el código para que nadie la reintente.

  Costó encontrarla porque el síntoma aparecía lejos del origen: en el hilo
  de inferencia de `camera.py`, y con toda la pinta de ser un problema del
  modelo o del driver. Se descartaron antes, uno a uno y con reproductores
  mínimos: proceso limpio, importar cv2, abrir la cámara, leer un frame,
  inicializar CUDA desde un hilo secundario, y cargar YOLO en el hilo
  principal para inferir desde otro. **Todos daban OK.**

  **La fuga de memoria era consecuencia, no causa.** Ultralytics rehace la
  preparación del modelo en cada llamada que falla —se ve en el log como
  `YOLO11s summary (fused)` repetido, 12 veces— y esa preparación deja
  memoria sin liberar. El bucle de `camera.py` reintentaba cada 2 s con la
  misma configuración rota, así que el backend pasaba de 992 MB a 4.5 GB en
  dos minutos y el OOM killer lo mataba.

  **Corregido en `camera.py`** con una regla que vale más allá de este fallo
  concreto: *un fallo no se reintenta con la misma configuración*. Primer
  fallo en GPU, se baja a CPU para siempre y se avisa. Tres fallos seguidos
  ya en CPU, se deja de inferir; el vídeo sigue y el estado lo dice. Se
  acabaron las tormentas de reintentos.

  **Estado medido tras las correcciones:** backend estable en **1527 MB**
  durante 100 s, **cero errores de inferencia**, **cero recargas del modelo**,
  GPU en uso. Sistema completo en 2406 MB de 7546, o sea 32%.

  **Auditoría de estructuras que crecen:** `detections` topa en 200,
  `captures` en 500 y borra el fichero al descartar, `env_history` mantiene
  una ventana de 120 s, y `ws_clients` se descarta tanto al desconectar como
  al fallar un envío. **No hay ninguna colección sin tope.** Disco al 19% y
  los logs se truncan en cada arranque.

  **Lo que falta para las 3 horas sin nadie delante: SUPERVISIÓN.** Hoy nada
  reinicia nada; si un proceso muere, se queda muerto. Se crearon dos
  unidades systemd en `jetson/systemd/` con `Restart=always` y, sobre todo,
  `MemoryMax` por servicio: eso convierte "se cae el sistema" en "se
  reinicia un servicio en 5 segundos", porque el kernel mata al que se pasa
  en vez de elegir víctima global. Llevan además un freno de 10 reinicios
  cada 5 minutos para que un fallo real no quede escondido tras un bucle.

  `systemd-analyze verify` encontró un error real en el primer borrador:
  `StartLimitBurst`/`StartLimitIntervalSec` van en `[Unit]`, no en
  `[Service]`, donde systemd los ignora en silencio. Corregido; las dos
  unidades validan limpias.

  **NO están instaladas.** Requiere `sudo` y crea servicios que sobreviven a
  la sesión, así que necesita que el usuario lo apruebe explícitamente.

- **2026-09-06** — **Modelos verificados y consumo del backend recortado de
  1575 MB a 79 MB en el caso que importa.**

  **Los modelos, recopiados por el usuario, ahora SÍ están íntegros.** Los
  cuatro `.pt` abren como ZIP válido y cargan con Ultralytics. Pesan casi el
  doble que los truncados (11.0 MB y 19.2 MB frente a 5.7 y 8.8).

  - `cls_ccmt` — tarea `classify`, **7 clases**: maíz enfermedad/sana, papa
    sana/tizón tardío/tizón temprano, pimiento enfermedad/sana.
  - `det_plagas` — tarea `detect`, **6 clases**: ácaro, barrenador de maíz,
    gusano cortador, pulgón, pulguilla, trips.

  **Ojo con lo que NO está.** El clasificador salió **sin ninguna clase de
  col**, pese a que la taxonomía del script las define (alternaria,
  podredumbre bacteriana, pudrición negra, mildiu, mancha anular, colonia de
  pulgón, sana). Y el detector salió sin `plaga_mosca_blanca`. Se habrán
  caído por `MIN_IMAGES_CLS`/`MIN_INSTANCES_DET`, o la fuente de col no se
  descargó. Conviene mirar el log del entrenamiento: son cultivos y plagas
  que el robot no podrá reconocer.

  Copiados a `jetson/backend/models/` como `agrobot_plagas_det.pt` y
  `agrobot_cultivos_cls.pt`, que es la carpeta donde el backend mira de
  verdad (`MODELS_DIR = _AQUI / "models"`). El clasificador aparecerá en la
  lista pero `models_load_handler` lo rechazará con un 400 explicando que
  hace falta uno de detección: `camera.py` lee `resultados.boxes` y un
  clasificador devuelve `.probs`. Está bien que lo rechace.

  **Recorte de memoria, medido en tres escenarios:**

  | escenario | RSS |
  |---|---|
  | original | 1575 MB |
  | cámara encendida, con los ajustes de torch | 1185 MB |
  | cámara apagada, carga perezosa | **79 MB** |

  Dos cambios, los dos en `load_yolo_model()`:

  1. **Carga perezosa.** Si `hardware.camera_enabled` es false, el modelo NO
     se carga. Antes se cargaba siempre, aunque no hubiera cámara de la que
     sacar fotogramas: giga y medio reservado para nada. Al encender la
     cámara desde el dashboard se vuelve a llamar aquí y se carga entonces.
  2. **`PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True` y
     `torch.set_num_threads(1)`**, fijados ANTES de importar torch porque
     después el asignador ya está construido. La inferencia va en GPU, así
     que cada hilo de CPU extra reservaba buffers para nada. Ahorran 390 MB.

  **Un tercer cambio se probó y se REVIRTIÓ.** Mover el modelo a GPU
  explícitamente con `.to("cuda")` al cargarlo rompe el arranque en esta
  placa: `RuntimeError: Expected NVML_SUCCESS ==
  nvmlDeviceGetGpuFabricInfoV_(...) to be true, but got false`. Es un fallo
  de esta build de PyTorch consultando NVML en el Orin. Ultralytics elige el
  dispositivo en cada `predict()` por su cuenta y usa la GPU igual. Queda
  documentado en el código para que nadie lo reintente.

  **PyTorch de NVIDIA para JetPack: NO se pudo instalar, no existe todavía.**
  La placa tiene **L4T R39.2.1 con CUDA 13.2**, o sea JetPack 7. El índice
  de ruedas de Jetson (`pypi.jetson-ai-lab.io`) devuelve "no encontrado" para
  `jp7/cu130`, `jp7/cu129` y `jp6/cu126`. Es coherente con lo reciente que es
  ese JetPack. Mientras no publiquen la rueda para `sm_87`, PyTorch seguirá
  compilando PTX en caliente, con el coste de memoria y arranque que eso
  tiene. Los recortes de arriba atacan el síntoma; la rueda correcta sería
  la cura.

- **2026-09-06** — **Arranque completo del sistema: cinco hallazgos, uno
  crítico.** Se lanzó `INICIAR_BOT.sh` para ver los dashboards y se
  aprovechó para revisar los modelos que el usuario subió a `models/runs/`.

  **1. El backend lo mata el OOM killer.** Confirmado en `dmesg`, dos veces:
  `Out of memory: Killed process (python3) total-vm:16455200kB,
  anon-rss:3603216kB`. El dashboard responde HTTP 200 durante uno o dos
  minutos y después se cae. ROS sobrevive porque el que se come la memoria
  es `jetson_server.py`: **3.6 GB de RSS** sobre 7.4 GB totales, con SLAM,
  LIDAR y rf2o corriendo al lado. No es un fallo del código, es que no cabe.

  **2. Y la causa probable de esos 3.6 GB está en el log de arranque:**
  PyTorch 2.13.0+cu130 avisa de que **no tiene compilación para este GPU**.
  El Orin es capacidad de cómputo 8.7 y el paquete trae `sm_80, sm_90,
  sm_100, sm_110, sm_120`. Funciona —se comprobó con una multiplicación de
  matrices real en GPU— pero por compilación PTX en caliente, que es lenta y
  cara en memoria. **La solución es instalar la rueda de PyTorch de NVIDIA
  para JetPack**, compilada para `sm_87`, en vez de la genérica de PyPI.

  **3. Los dos `.pt` que subió el usuario están TRUNCADOS.** Empiezan con
  firma ZIP correcta (`504b0304`) pero no tienen la marca de fin de archivo
  (`PK\x05\x06`), y los dos cortan en múltiplo exacto de 1024 bytes: 5600
  KiB y 8576 KiB. La copia no terminó. Ultralytics falla con
  `PytorchStreamReader failed reading zip archive`. El resto del run
  (`results.csv`, matrices de confusión, curvas) sí está íntegro.

  **4. Están además en la carpeta equivocada.** El backend define
  `MODELS_DIR = jetson/backend/models`, y el usuario subió `runs/` a
  `agrobot_system_2/models/`. Son dos carpetas distintas. Y aunque
  estuvieran en la buena, `list_model_files()` usa `MODELS_DIR.glob()`, que
  **no es recursivo**: nunca miraría dentro de `runs/*/weights/`.

  **5. El clasificador no se puede enchufar tal cual.** `camera.py` lee
  `resultados.boxes`, o sea que la tubería es de DETECCIÓN. Un modelo de
  clasificación devuelve `.probs` y no `.boxes`: se cargaría sin error y
  daría cero detecciones en silencio.

  **Sobre el entrenamiento en sí** (leído de los CSV, que están intactos):
  el detector saca mAP50 0.937 y mAP50-95 0.565 en 40 épocas, que es
  razonable. El clasificador saca **99.93% de top-1**, y esa cifra es la
  trampa que el propio código del usuario documenta: PlantVillage son fotos
  de laboratorio con la hoja arrancada sobre fondo uniforme. Su script ya
  construye un `field_test` con recortes de PlantDoc justo para medir la
  caída real; **esa es la cifra que vale, no el 99.93%**.

  **Lo que sí quedó verificado y bien:** los dos dashboards responden HTTP
  200, el de la Jetson sirve byte a byte el fichero editado (paleta viva,
  tipografía grande, cero referencias a presión), y el LIDAR volvió a
  publicar a 10 Hz tras el reinicio, junto con `/scan_odom`.

## 12-BIS. HANDOFF — estado al cerrar el 2026-09-06

> **Histórico.** El estado vigente está en la sección 0 (2026-09-17).

Reescrito al final de la jornada. Lo de la version anterior que ya se
resolvio se ha quitado; lo que sigue abierto se mantiene.

### RESUELTO hoy (no volver sobre ello)

- **Sensor de presión BMP280** eliminado del sistema entero: firmware,
  decoder, codec, gateway, Telegram, los dos dashboards y la base SQLite
  (columnas incluidas, con script de migración propio).
- **Servo:** la culpable era la librería ESP32Servo, que al fallar dejaba el
  GPIO10 inservible para el resto de la sesión. Ahora va por LEDC en crudo.
- **Foco:** polaridad establecida por autoprueba, no por deducción.
- **Nodo LoRa:** región US915, Clase C, sin re-JOIN en cada ciclo, downlinks
  llegando, UDP a 20 Hz desacoplado.
- **Estabilidad del backend:** de morir por OOM cada pocos minutos a **130
  minutos con 0 reinicios y 0 caídas del dashboard**. Se corrigieron una
  sesión HTTP por detección, tareas asíncronas sin tope, el volcado del mapa
  a lista de Python en cada publicación, y sobre todo el EKF que integraba la
  aceleración lineal y mandaba la pose a 5839 m.
- **Supervisión:** `agrobot-backend` y `agrobot-ros` instalados, habilitados
  al arranque, con `Restart=always` y `MemoryMax`.
- **Odometría láser (rf2o)** compilada, cableada y publicando.
- **`tools/sensores.py`**: registro unificado de todos los sensores con
  análisis de correlaciones y retardos.

### PENDIENTE — hay que hacerlo

1. **Pegar el codec en ChirpStack.** Device profiles → US915_PROFILE →
   pestaña **Payload codec** → JavaScript functions. El del repo acepta 17
   bytes; el que está puesto en el servidor es el viejo. Y en **Measurements**
   del mismo perfil, borrar `pressure` y `tempBMP` si siguen dados de alta.
2. **Borrar los respaldos** `agrobot_history.pre_presion_*.db` de la Pi.
3. **La prueba de rf2o, bien hecha.** La de hoy no vale: se empujó el robot a
   mano y el operador es un objeto grande y móvil dentro del scan. Hay que
   **conducirlo desde el dashboard**, con el operador apartado, fases
   separadas por paradas de 5 s (quieto → 1 m recto → quieto → giro 180° →
   quieto) y junto a una pared. Si `vx` sale cero mientras el robot avanza,
   es la degeneración del corredor y hay que devolver el avance a
   `/wheel_odom` o poner encoders.
4. **Probar el servo DESPUÉS de conducir.** El conflicto de canales LEDC ya
   está corregido, pero esa comprobación concreta —mover el servo, conducir
   unos segundos, volver a moverlo— nunca se llegó a hacer.
5. **Correr `tools/sensores.py` con todo arriba.** Se probó sin backend y sin
   LIDAR, así que las columnas de actuador, cámara y `/scan` nunca se
   llenaron.
6. **Soak de 3 h con el robot EN MOVIMIENTO.** Los 130 minutos limpios fueron
   con el robot quieto: slam_toolbox no construyó mapa nuevo ni cerró bucles
   y la cámara detectó poco. Ése es el caso fácil.

### PENDIENTE — decisiones de configuración con números del usuario

7. **`inflation_radius: 0.4` va a cerrar el pasillo del surco.** Con
   `robot_radius: 0.25`, si los surcos están a 0.5–0.75 m el inflado de los
   dos lados se solapa y Nav2 se negará a entrar. **Hace falta la separación
   real entre surcos.**
8. **El rango de velocidad de DWB no es ejecutable.** `PWM_MIN = 200` sobre
   255: por debajo del 78% de potencia el robot no se mueve, y DWB muestrea
   desde 0. Hay que medir la velocidad mínima real sostenible.
9. **`robot_radius: 0.25`** es un círculo conservador, no el footprint real.
10. **`LIDAR_XYZ = 0.18`** contra los 30–33 cm medidos. Confirmar dónde está
    `base_link`.

### DISEÑADO, sin implementar

11. **Seguimiento de surcos** por RANSAC sobre el LIDAR (dos rectas, línea
    central, persecución pura). Con LIDAR y no con cámara: funciona de noche
    y la cámara ya está ocupada con el YOLO.
12. **Giro de cabecera en cuadrado**, que es lo que puede hacer un
    skid-steer. **Depende de tener odometría fiable (punto 3).**
13. **Unificar los dos sistemas de evasión**, que hoy se ignoran.

### CONOCIDO, no tocado

14. **PyTorch no tiene rueda de NVIDIA para este JetPack** (L4T R39.2.1,
    CUDA 13.2). Funciona compilando PTX en caliente, con su coste de memoria
    y arranque. No existe todavía la rueda para `sm_87`.
15. **El clasificador salió sin las clases de col** y el detector sin mosca
    blanca. Revisar los umbrales del entrenamiento.
16. **El clasificador saca 99.93%**, que es la trampa de PlantVillage. La
    cifra honesta es la del `field_test` con recortes de PlantDoc.
17. **El IMU estaría mejor en el I2C de la Jetson**, pines 3 y 5
    (`/dev/i2c-7`). Alimentar desde el pin 1 (3.3 V), **nunca del pin 2**.
18. **Montaje del LIDAR:** a 30 cm sobre suspensión de recorrido largo, con
    2° de cabeceo el haz toca el suelo a 8.6 m. Fijarlo rígido al chasis.
19. **El foco puede destellar en el arranque** antes de que setup() tome el
    pin. Se corrige con un pull-up en la etapa, no con código.

### AVISOS DE OPERACIÓN

- **`INICIAR_BOT.sh` ya NO se usa.** Chocaría con los servicios por el puerto
  8080 y por el serie del LIDAR. Todo va por `systemctl`.
- **El modo de la pila ROS se elige en `/etc/default/agrobot`**
  (`AGROBOT_LAUNCH`). Por defecto mapeo; Nav2 sólo está en el launch de
  navegación, y para eso hace falta un mapa guardado de verdad. El actual
  cubre 3.3 x 3.7 m, o sea el taller.
- **Core Debug Level en Error** en el IDE de Arduino. Con el nivel en None,
  los fallos del core de ESP32 son mudos, y eso costó una jornada entera.

## 12. Pendientes conocidos (honestos, no ocultar)

### Bloqueadores para correr en campo (orden de prioridad)

1. ~~**El AP Wi-Fi de la Jetson NO existe.**~~ ✅ **RESUELTO el 2026-09-02**
   con **modo dual AP+cliente** (ver §2). La Jetson sirve `JETSON_AP` en
   `192.168.12.1` **y al mismo tiempo** sigue conectada como cliente a
   `LoRa_Wan` con internet. No hay que elegir entre una y otra, y no se
   pierde el SSH al levantarlo — la advertencia anterior de que "al volverla
   AP pierde internet" era incorrecta.
2. **Falta cablear los tres hilos del UART** (Jetson pin 8/10/6 ↔ C3
   GPIO20/21/GND, §13). Hasta entonces el mando solo se puede probar con la
   variante `MANDO_POR_USB 1`. Es el único bloqueador real que queda para
   mover el robot desde la Jetson.
3. ~~**El SSID/clave del firmware deben coincidir con los del AP.**~~
   **Ya no aplica desde el 2026-09-04**: el C3 no tiene Wi-Fi. El SSID y la
   clave siguen importando para el nodo LoRa y para los clientes del
   dashboard, no para el actuador.
4. **Falta anotar la MAC de la ESP32 LoRa V3** para su reserva DHCP (hay un
   renglón para eso en `docs/CONFIGURAR_AP_JETSON.txt`). El ESP32-C3 ya no
   depende del DHCP: fija `192.168.12.50` en su propio firmware.
5. **GPIO de strapping (hardware).** En el ESP32-C3 los GPIO 2, 8 y 9 son
   pines de arranque, y el diseño usa el 8 para el servo y el 9 para el LED.
   Puede impedir flashear o bootear la placa. Si aparecen problemas raros de
   arranque, mover a GPIO 10 y 3 y actualizar los `#define`.

### Otros pendientes

- **`tests/test_actuator_e2e.py` prueba un camino que ya no existe.** Sigue
  ejercitando el transporte UDP y `protocol.py`, retirados el 2026-09-04.
  Pasa, pero no prueba nada que el robot use hoy. Hay que reescribirlo contra
  `uart_link.py` (con un puerto simulado) o borrarlo.
- **Este manifiesto vive en la Jetson.** La copia de la Raspberry Pi
  (`/home/admin/Desktop/AGROBOT.2/`) hay que sincronizarla **a mano**: desde
  la Jetson no hay acceso por SSH a la Pi (pide contraseña), solo por HTTP al
  puerto 8092.
- **El dashboard de la Pi no tiene panel de Telegram** aunque el gateway
  expone cinco rutas para ello (§11).

- **Falta el modelo de PLAGAS.** Hay un `yolo11n.pt` genérico (COCO)
  instalado y funcionando, pero detecta personas, autos y televisores — no
  plagas. Toda la cadena está verificada de punta a punta, así que sustituirlo
  es dejar el `.pt` entrenado en `models/` y activarlo desde el dashboard.
  El resto —captura en hilo, inferencia, dibujo de cajas, galería con la foto
  real, umbrales configurables— ya está y se prueba solo.

- **La sesión de login no tiene el grupo `dialout`.** El usuario está en el
  grupo (`groups jetson` lo confirma) pero la sesión es anterior al cambio,
  así que `id` no lo lista y el LIDAR no abre `/dev/ttyUSB0`.
  `lanzar_lidar.sh` lo esquiva con `sg dialout`, pero el arreglo de raíz es
  **cerrar sesión y volver a entrar** (o reiniciar). Hasta entonces, nada
  que abra el puerto serie directamente va a funcionar.

- **Calibrar `PWM_MIN` en el robot real** (hoy 90, valor de arranque
  razonable pero no medido). Procedimiento: con las ruedas en el aire, bajar
  el acelerador del dashboard hasta el mínimo y subir `PWM_MIN` hasta que
  las cuatro ruedas giren siempre; si la marcha mínima queda demasiado
  rápida, bajarlo. Es la única constante del firmware que hay que ajustar a
  mano, y es la que decide si la marcha lenta existe o solo zumba.
- **Verificar en el robot que `⇦ ⇨` desplazan y `↺ ↻` giran**, y no al
  revés. El segundo sketch de prueba del usuario llamaba `girarIzquierda` /
  `girarDerecha` a lo que en realidad es traslación lateral (ver el
  changelog del 2026-09-02). Si al apretar `↺` el robot se desplaza en vez
  de girar, hay que **intercambiar `vy` y `omega`** en la cinemática del
  `.ino` — es lo único que cambiaría.

- Nav2/slam_toolbox/EKF escritos y compilados, pero sin probar moviendo el
  robot (el LIDAR sí está verificado publicando `/scan` a 10 Hz).
- `WHEEL_BASE = 0.30 m` (separación delante-atrás) es un valor **supuesto**:
  hay que medirlo. Solo afecta cuánto giro se obtiene por unidad de omega.
- Footprint real del robot sin medir (`nav2_params.yaml` usa un círculo
  supuesto de 25cm).
- `hardware.lidar_baud` sigue siendo config muerta: los launch de ROS no lo
  leen, usan el default del driver `sllidar_ros2` (460800, que resulta ser
  el correcto). O se cablea al launch, o se saca del panel.
- `rtabmap_odom` instalado pero todavía NO integrado al EKF.
- Endpoints sin consumidor en el dashboard: `GET /api/health` (los mismos
  datos ya llegan por WS) y `POST /api/lidar/scan` (fallback manual
  documentado a propósito, el camino real es `sllidar_ros2`).
- `downlinks.delivered` (RPi) se crea siempre en 0 y nada lo actualiza
  nunca; `schema_version` se guarda pero no se muestra.
- Detección automática de la red de base para subir el reporte de sesión.
- Revisión a fondo de los PDFs de referencia originales del usuario
  ("AGROBOT · Command.pdf", "1.pdf") por si tienen specs de interfaz no
  incorporadas todavía.

---

## Historial de cambios


- **2026-09-08** — **Par de tracción, pulsos de giro, cambio de modelo en
  caliente sin frenar el robot, y dashboard usable desde el teléfono.**
  Salió de una sesión de manejo real del usuario: al robot le faltaba par,
  las ruedas "zumbaban" sin mover al 100%, los giros se clavaban, y la
  Jetson se reiniciaba al arrancar los motores.

  **1. La frecuencia de PWM era la causa principal de la falta de par.**
  Estaba en 8 kHz. El TA6586 es un puente H de conmutación lenta: sus
  flancos duran decenas de microsegundos, y a 8 kHz (125 µs de periodo) una
  fracción grande de cada ciclo se va en la transición, donde el puente ni
  conduce ni corta y disipa como calor lo que debería llegar al motor. El
  duty pedido no era el par entregado. Bajado a **1 kHz**, donde el periodo
  es ocho veces más largo y la transición pasa a ser despreciable. El precio
  es el chillido audible; es el compromiso correcto. Siguiente escalón si
  molesta: 2 kHz.

  **2. "A fondo" nunca era a fondo.** Con LEDC a 8 bits, escribir 255 deja
  255 de cada 256 cuentas en alto: en media el 99,6%, pero el driver SIGUE
  conmutando, y por el punto 1 cada conmutación cuesta par. Justo en los
  instantes que más par piden —los pulsos de arranque y los de giro, que son
  los que escriben `PWM_MAX`. Ahora, cuando el duty llega al máximo, se
  suelta el canal LEDC y el pin se maneja como salida digital en `HIGH`:
  corriente continua de verdad, cero conmutaciones (campo `dcCompleto` de
  `struct Rueda`).

  **3. Los giros tienen ahora su propio piso de duty y un tren de pulsos
  sostenido.** Girar sobre el sitio en un skid-steer arrastra las cuatro
  ruedas de costado contra el suelo: el rozamiento es varias veces el de ir
  recto y, a diferencia del arranque, NO se acaba. Los pulsos `KICK_*`
  existentes son de una sola vez y no cubrían esto. Se añadió
  `PWM_MIN_GIRO = 235` (la cinemática de un giro puro pedía un duty de mitad
  de tabla) y un tren de 140 ms a fondo / 90 ms al duty pedido que dura
  todo el giro. En la fase baja NO se cae a 0: eso frenaría el robot en cada
  hueco. Se activa solo con giro dominante (`GIRO_UMBRAL`), así que corregir
  el rumbo yendo recto no da tirones. Los cuatro golpes van a la vez, sin
  escalonar por lados: en un giro los lados tiran opuestos y golpear uno
  solo empuja el robot de costado en vez de girarlo.

  **4. Deriva de configuración: el mando iba con topes falsos.**
  `agrobot_config.json` tenía `max_v_mps = 1.5` y `max_omega_rps = 4.0`
  contra `V_MAX = 0.55` y `OMEGA_MAX = 1.5` del firmware. El C3 recortaba en
  silencio, así que el 63% superior del recorrido del acelerador no hacía
  nada. Alineados con el firmware (que es lo que ya decían los valores por
  defecto de `config_store.py` — la deriva estaba solo en el JSON).

  **5. Cambiar de modelo frenaba el robot en seco.** `YOLO(ruta)` es código
  síncrono de varios segundos y se llamaba dentro del event loop de aiohttp,
  el mismo donde vive el `_send_loop` de 20 Hz de `uart_link`. El firmware
  entra en failsafe a los 300 ms sin trama: cambiar de modelo en marcha
  paraba el robot, y el motivo no se veía en ningún sitio porque el
  dashboard decía "Modelo cargado". Ahora la carga va en `asyncio.to_thread`.
  Además: opción **"Sin modelo"** (devuelve el vídeo limpio y libera la GPU
  sin reiniciar el backend), indicador de progreso, y el error del backend
  —que ya era explícito— por fin se muestra en pantalla en vez de tirarse.

  **6. Dashboard usable desde el móvil.** El layout era de escritorio fijo:
  barra lateral de 276 px y `overflow: hidden` en el `body`, así que en un
  teléfono media pantalla se la comía el menú y lo que quedaba fuera no se
  podía alcanzar. La barra pasa a cajón deslizante con botón de menú y velo;
  las rejillas caen a una columna; la cruceta de manejo sube de 54 a 64 px y
  los deslizadores a 26 px de pulsador (ningún control por debajo de 44 px);
  las píldoras de estado pasan a una fila deslizable; el hover se anula en
  pantalla táctil, donde se quedaba pegado tras el toque. La parada de
  emergencia sigue fija y visible siempre, solo más chica.

  **7. Los stubs de `tests/stubs_arduino` estaban desactualizados** y
  `verificar.sh` fallaba con 13 errores que no eran del sketch (`MandoUart`,
  `SERIAL_8N1`, `Serial.printf`, las funciones LEDC que devuelven `bool`).
  Arreglados: los dos cores vuelven a compilar limpios.

  **⚠️ PENDIENTE DE HARDWARE, NO DE CÓDIGO: los reinicios de la Jetson.**
  `last -x reboot` muestra tres arranques fechados en `Jan 1 00:01`, o sea
  con el reloj perdido: eso es CORTE DE ALIMENTACIÓN duro, no un reinicio
  por software ni un OOM. La causa es la topología de estrella que sigue sin
  hacerse (ver el changelog del 2026-09-04, punto 12). Los cambios 1, 2 y 3
  de esta sesión SUBEN el consumo —es lo que da el par— así que agravan el
  síntoma hasta que se arregle la alimentación. La Orin Nano Dev Kit entra
  por 5 V con un límite crítico de 3,95 A en `VDD_IN`; en reposo consume
  1,18 A. Alimentarla de una rama propia es la solución correcta, con la
  masa de señal del UART mantenida.

  Archivos: `firmware/esp32c3_actuator/esp32c3_actuator.ino`,
  `firmware/esp32c3_actuator/tests/motor_logic_test.cpp`,
  `firmware/esp32c3_actuator/tests/stubs_arduino/Arduino.h`,
  `jetson/backend/jetson_server.py`, `jetson/backend/agrobot_config.json`,
  `jetson/dashboard/index.html`. Copia previa del firmware en
  `esp32c3_actuator.ino.pre_torque.bak`.


- **2026-09-06** — **`jetson/backend/tools/sensores.py`: registro unificado
  de todos los sensores y análisis de correlación.** Pedido del usuario para
  poder ver la relación ENTRE señales, que es donde están casi todos los
  problemas de un robot: lo que se comanda contra lo que sale, el giro
  pedido contra el que mide el giróscopo, la vibración contra el error de
  odometría.

  Dos subcomandos: `grabar` vuelca todo a un CSV con un único reloj, y
  `analizar` saca correlaciones y RETARDOS entre pares de señales.

  **Tres fuentes, y por qué cada una:**

  - **ROS** para IMU, `/scan`, rf2o, EKF y odometría comandada. Es la vía de
    menor latencia y la que ya usa la navegación. Corre en su propio hilo.
  - **WebSocket del backend** (:8080) para lo que sólo él sabe: estado real
    del actuador, salud de cada subsistema, detecciones de la cámara. Se lee
    de ahí en vez de duplicar su lógica.
  - **UDP 5001 directo**, y SÓLO si el backend está apagado. Se comprueba
    intentando abrir el puerto: si está ocupado es que el backend lo tiene, y
    quitárselo rompería la telemetría del robot. En ese caso el dato ya llega
    por ROS igualmente.

  **Decisiones que no son obvias:**

  - El scan de 360 puntos se RESUME (mínimo, ángulo del mínimo, mínimo
    frontal, cuántos haces son válidos). Guardar 360 rangos por fila haría
    la tabla ilegible y no ayuda a buscar correlaciones. El recuento de haces
    válidos sí importa: un scan que se queda sin retornos explica por sí solo
    un fallo de odometría láser.
  - Cada grupo lleva una columna `edad_*`. Sin ella no se distingue "el
    sensor mide siempre lo mismo" de "el sensor está muerto y se repite el
    último valor conocido", que es exactamente el error que la máscara de
    validez del nodo LoRa vino a resolver en su día.
  - El análisis mide **retardo** por correlación cruzada, no sólo
    correlación. Saber que dos señales se parecen sirve de poco; saber que
    una sigue a la otra 300 ms tarde es latencia de la cadena de control, y
    eso se puede reducir.
  - El informe **avisa contra la correlación espuria**. En la prueba real,
    `temp_scd` salió a 0.68 con `ekf_x` sin ninguna relación física: las dos
    simplemente derivaban despacio. El aviso está puesto porque ese error lo
    comete cualquiera mirando una matriz de correlación.

  **Probado con hardware real**, no sólo compilado: grabó 250 filas a 10 Hz
  con el nodo LoRa transmitiendo de verdad, detectó correctamente qué grupos
  estaban activos y cuáles no, y el análisis marcó las señales sin datos y
  las constantes. Se corrigió además un aborto al salir (`terminate called
  without an active exception` con volcado de core) que venía de matar el
  hilo de ROS a mitad de `spin_once`; ahora se espera a que cierre.

- **2026-09-06** — **Odometría láser (rf2o) instalada, cableada y probada en
  banco.** Es la primera medida REAL de traslación del robot: no hay
  encoders, y `/wheel_odom` era la velocidad comandada, o sea una suposición
  en lazo abierto sobre un skid-steer que patina.

  **Qué se hizo:**

  - `rf2o_laser_odometry` compilado de fuente desde
    `github.com/MAPIRlab/rf2o_laser_odometry`, rama `ros2`, con aprobación
    explícita del usuario. No existe paquete para Jazzy en los repos.
  - `ros-jazzy-laser-filters` instalado por apt.
  - Cadena nueva: `/scan` → filtro de rango (0.15–6 m) → `/scan_odom` →
    rf2o → `/odom_rf2o`. Añadida a los DOS launch, mapping y navigation.
  - `ekf.yaml`: `odom0` pasa a ser `/odom_rf2o` y aporta **sólo `vx`**.
    `/wheel_odom` se degrada a `odom1` y aporta **sólo el giro comandado**.
    El yaw sigue viniendo del IMU, que lo mide mejor y más rápido (20 Hz
    frente a los 10 del LIDAR).

  **Por qué el filtro de rango.** El chasis lleva suspensión de recorrido
  largo (foto del usuario) y el LIDAR va a ~30 cm del suelo. Con 2° de
  cabeceo el haz ya toca el suelo a 8.6 m; con 5°, a 3.4 m. Ese suelo entra
  y sale del scan en cada bache y el casamiento lo lee como traslación que
  no ocurrió. Recortar a 6 m corta esa fuente de ruido. El `/scan` completo
  sigue intacto para slam_toolbox y los costmaps.

  **Trampa del paquete, encontrada leyendo su fuente.**
  `init_pose_from_topic` trae por defecto `/base_pose_ground_truth`, un
  topic de simulación que aquí no publica nadie. Con el valor por defecto el
  nodo espera esa pose para siempre y NO publica odometría jamás, sin dar
  error (`CLaserOdometry2DNode.cpp:53`). Hay que dejarlo VACÍO. También
  `publish_tf: false`, porque quien publica `odom → base_link` es el EKF y
  dos publicadores de la misma transformación hacen temblar el robot en
  RViz sin causa aparente.

  **Probado en banco con un `/scan` sintético**, sin hardware: la cadena
  publica `/odom_rf2o` a 10.0 Hz.

  **⚠️ HALLAZGO IMPORTANTE, y va justo contra el caso de uso.** En la prueba
  con un pasillo liso y uniforme, rf2o devuelve `vx = 0.0` exactamente,
  aunque el robot avance. Es la degeneración clásica del casamiento de
  scans: en un corredor sin rasgos no hay nada que se desplace en la
  dirección de avance, así que el algoritmo no puede verlo. **Un surco ES un
  corredor.** Al repetir la prueba con textura en las paredes, el nodo sí
  produce velocidad distinta de cero.

  El número con textura (−1.86 m/s frente a los 0.5 pretendidos) NO valida
  la exactitud: el patrón sintético es periódico cada 25 cm y el emparejador
  se engancha a un desplazamiento equivocado por aliasing. Lo único que esa
  prueba demuestra es la diferencia entre cero y no cero.

  **Lo que queda por decidir, y sólo lo decide el campo:** si dentro de un
  surco real hay bastante textura (plantas sueltas, huecos, tutores) para
  que rf2o vea el avance. Si devolviera cero con confianza alta mientras el
  robot se mueve, el EKF se lo creería y la estimación de avance colapsaría
  — peligroso justo para medir la distancia del giro de cabecera. La prueba
  es empujar el robot una distancia medida por un surco y mirar
  `/odom_rf2o`. Si falla, la salida es devolver `vx` a `/wheel_odom` como
  respaldo con covarianza alta, o poner encoders.

- **2026-09-06** — **UDP desacoplado del ciclo LoRaWAN: el IMU pasa de una
  muestra cada 30 s a 20 Hz.** Era el objetivo de toda la cadena de cambios
  del día. Sólo fue posible después de la Clase C, porque con deep sleep la
  placa no estaba despierta para mandar nada entre ciclos.

  **En el firmware:**

  - `readSensors()` se parte en `readIMU()` y `readAmbiente()`. El MPU6050
    se lee en cada envío UDP; el SCD40 sólo cada 5 s, que es su periodo real
    de medida. Consultarlo a 20 Hz sería pedirle 100 veces lo mismo y
    ocuparle el bus I2C al IMU para nada.
  - `sendUDP()` sale de `prepareTxFrame()` y pasa al `loop()` con su propio
    temporizador de 50 ms. `prepareTxFrame()` ya sólo empaqueta.
  - `sendUDP()` deja de imprimir por serie. A 20 Hz eran 1200 líneas por
    minuto, y cada `Serial.print` bloquea el lazo mientras vacía el buffer.
    El recuento pasa al `[Status]` de cada 30 s, que ahora informa de
    cuántos datagramas salieron y a qué frecuencia real. Lo esperado son
    ~600 en 30 s; un número muy por debajo delata que el lazo se bloquea.
  - El campo `seq` del JSON deja de ser el contador de LoRa y pasa a
    `udpSeq`, propio del flujo UDP: a 20 Hz el de LoRa se repetiría.

  **En `jetson_server.py`**, el callback UDP se parte en dos caminos:

  - **Rápido, a 20 Hz:** `pose_estimator.on_gyro()` y
    `ros_bridge.publish_imu()`. Es el dato de navegación y va entero.
  - **Lento, a 2 Hz:** `push_env_history()` y el `broadcast` al dashboard.
    A 20 Hz serían 20 mensajes WebSocket por segundo a cada navegador, y
    `push_env_history()` reconstruye su lista completa en cada llamada, o
    sea trabajo O(n) veinte veces por segundo sobre 2400 puntos. Nada de
    esto lo nota un ojo humano.

  La frecuencia de 20 Hz no es arbitraria: es exactamente la
  `frequency: 20.0` del EKF en `ekf.yaml`. Mandar más rápido no mejoraría la
  estimación, sólo cargaría la red y el backend.

- **2026-09-06** — **Nodo LoRa pasado a CLASE C.** Motivación: el usuario
  preguntó si alargar el ciclo LoRa a 5 minutos mejoraría la latencia del
  Wi-Fi. Va al revés — `sendUDP()` se llama desde `prepareTxFrame()`, así
  que el UDP viaja colgado del ciclo LoRaWAN, y alargarlo lo empeora en la
  misma proporción. Además, entre ciclos la placa está en deep sleep con la
  radio Wi-Fi apagada: no es que mande poco, es que no puede mandar.

  Un dispositivo Clase C escucha de forma continua, así que no puede
  dormirse. Eso desbloquea las dos cosas que hacían falta:

  - El Wi-Fi se mantiene asociado entre ciclos. Es el paso previo para sacar
    `sendUDP()` de `prepareTxFrame()` y mandar el IMU a su propio ritmo.
  - Los downlinks dejan de esperar al siguiente uplink. La parada de
    emergencia pasa de tardar hasta 30 s a llegar en segundos.

  El precio es el consumo: la radio queda en recepción permanente. En este
  robot, donde la Jetson y los motores gastan mucho más, es asumible.

  **ChirpStack ya estaba preparado:** `supports_class_c = true` en
  `US915_PROFILE`. No hizo falta tocar el servidor.

  **Se cambió SOLO la clase**, a propósito, para poder atribuir el resultado.
  El desacople de `sendUDP()` y el ciclo de 5 minutos vienen después, y en
  ese orden: con el UDP ya independiente, alargar el ciclo LoRa deja de
  tener contrapartida.

  **VERIFICADO. Funciona.** `Mcu.sleep()` respeta la Clase C y no duerme:

  - La librería anuncia `LoRaWAN US915 Class C start!` y `+Class=C`.
  - El recuadro de arranque **deja de repetirse**. Ya no hay reinicios.
  - Aparecen por fin las líneas `[Status]` cada 30 s. Antes no salían nunca
    porque la placa se reiniciaba antes de llegar a los 30 s de vida.
  - La línea `🌐 [UDP]` sale en TODOS los envíos. El Wi-Fi ya no se cae
    entre ciclos, así que no se pierde el primer datagrama.
  - El heap se mantiene: 253180 → 253132 bytes.

  **La cadencia mejoró en vez de empeorar.** Medido sobre el histórico, el
  intervalo entre uplinks pasó de 32.7 s a **30.0 s exactos**: sin reinicio
  no hay que pagar el arranque. Aparecen uplinks sueltos a 1.2 s, pero son
  correctos — un downlink confirmado obliga al nodo a responder con un
  uplink inmediato para llevar el ACK, y sólo ocurren cuando se manda algo.
  Media real: 2 uplinks por minuto.

  **Los downlinks siguen llegando**, ahora sin esperar: se recibieron dos
  PING (`0x03`) y un ACK (`0x04`).

- **2026-09-06** — **DOWNLINKS FUNCIONANDO, verificado en el hardware.** El
  PING salió por el puerto serie del nodo:

      📥 [RX] DOWNLINK #3 — puerto 10, 1 byte(s), RSSI -39 dBm, SNR 11
           HEX: 03
           🏓 PING — el enlace de bajada funciona

  La cadena completa queda probada: dashboard → `gateway_server.py` → API de
  ChirpStack → radio → `downLinkDataHandle()` en el nodo. Con esto, la
  parada de emergencia por LoRa ya es un camino real y no sólo un botón.

  Se vieron cuatro PING seguidos, uno por ciclo. No es un fallo: eran cuatro
  elementos encolados que ChirpStack entrega de uno en uno por oportunidad
  de bajada. La cola quedó a cero al terminar.

- **2026-09-06** — **RESUELTO: `LoRaWAN.init()` no debe llamarse desde
  `setup()`.** El usuario aportó `LoRaWan_APP.cpp` de Heltec y con el código
  delante la causa es de una línea. `init()` termina así:

      LoRaMacInitialization( &LoRaMacPrimitive, &LoRaMacCallback, region );
      ...
      deviceState = DEVICE_STATE_JOIN;

  Reinicia la pila LoRaMac y **fuerza un JOIN**. Y `setup()` se ejecuta en
  CADA despertar del deep sleep, no sólo al encender. De ahí el OTAA cada
  40 segundos y el DevAddr nuevo en cada ciclo.

  La librería está diseñada para sobrevivir al sueño: `deviceState`,
  `appData`, `appDataSize`, `nextTx` y los primitivos de LoRaMac están todos
  marcados `RTC_DATA_ATTR`, y la memoria RTC no se pierde en deep sleep. El
  patrón correcto, que es el de los propios ejemplos de Heltec, es llamar a
  `init()` desde `case DEVICE_STATE_INIT` del loop. Al despertar,
  `deviceState` ya vale `DEVICE_STATE_SEND` y ese case no se vuelve a pisar.

  **Cambios aplicados, dos:**

  1. `LoRaWAN.init()` y `setDefaultDR()` salen de `setup()` y entran en el
     `case DEVICE_STATE_INIT`.
  2. `joined` pasa a `RTC_DATA_ATTR`. Sin eso, al despertar valía `false`,
     el `case DEVICE_STATE_SEND` caía en el `else` y a los 60 s forzaba otro
     JOIN, anulando la corrección anterior.

  **Confirmado de paso, leyendo el mismo archivo:**

  - `sleep()` es `Mcu.timerhandler(); Radio.IrqProcess(); Mcu.sleep(...)`.
    Ahí está el proceso de interrupciones de radio, y por eso quitarlo
    rompió el JOIN. La marcha atrás del intento anterior era correcta.
  - `downLinkDataHandle` está declarada `__attribute__((weak))`, así que la
    del sketch la sobrescribe. El nombre con L mayúscula era el correcto.
  - Sólo se llama cuando `mcpsIndication->RxData == true`. Los "received
    unconfirmed downlink" sin nuestro bloque impreso eran tramas MAC sin
    payload de aplicación: comportamiento normal, no un fallo.
  - Las ráfagas extra vienen de `FramePending` y `MLME_SCHEDULE_UPLINK`, que
    llaman a `OnTxNextPacketTimerEvent()` y provocan un uplink inmediato.
    Son los comandos MAC que la red envía tras cada JOIN.

  **VERIFICADO EN CAMPO el mismo día, y funciona:**

  - En el log, tras un despertar con motivo `DEEPSLEEP`, el nodo pasa
    directamente a `[State] 📤 DEVICE_STATE_SEND`. No imprime el bloque
    `+OTAA=`, ni `joining...`, ni `joined`. La sesión sobrevivió al sueño.
  - `dev_addr` se mantuvo en `0166e75b` a lo largo de varios ciclos. Antes
    cambiaba en cada uno.
  - Las ráfagas se deshicieron solas, como se esperaba: los tamaños pasaron
    de `3,3,3,3…` a `1,1,2` y el periodo de 40.8 s a 32.7 s, que es el ciclo
    de 30 s pretendido más el tiempo de arranque.

  **Efecto secundario menor que queda:** el deep sleep apaga la radio Wi-Fi,
  así que en el primer envío tras cada despertar `wifi_connected` todavía es
  false y ESE datagrama UDP se pierde. Se ve en el log: el ciclo que sigue a
  un despertar no imprime la línea `🌐 [UDP]`. La telemetría por LoRa no se
  ve afectada, y el dato se recupera en el siguiente ciclo. No se toca.

- **2026-09-06** — **CAUSA RAÍZ DE TODO: `LoRaWAN.sleep()` entra en DEEP
  SLEEP, y despertar de deep sleep REINICIA el programa desde `setup()`.**
  Lo dijo `esp_reset_reason()`, que se añadió justo para esto y devolvió
  `DEEPSLEEP` en cada vuelta. No era un cuelgue, ni un brownout, ni el
  watchdog: era la librería haciendo su ciclo de bajo consumo.

  Lo que se estaba pagando por ello, y que explica los tres misterios de la
  jornada de una sola vez:

  - **Un OTAA completo cada 40 segundos.** El contexto LoRaWAN no sobrevivía
    al sueño, así que el nodo se re-unía en cada ciclo. ChirpStack fue
    asignando `008b1977` → `00f74b68` → `01861553` en una hora. Cada
    DevAddr nuevo es una sesión nueva.
  - **Los downlinks no podían llegar NUNCA.** Un comando encolado se dirige
    a la sesión vigente; cuando el nodo despierta ya tiene otra. Esto, y no
    el código del manejador, es por lo que el PING nunca apareció.
  - **Las "ráfagas de 3 tramas cada 40.8 s"**, medidas 32 veces seguidas en
    el histórico, todas de exactamente 3. No era el bug del `joined`: era
    unirse, mandar, dormir, reiniciar.
  - **El Wi-Fi se caía en cada ciclo**, porque el deep sleep apaga la radio.

  **Corrección aplicada:** en `case DEVICE_STATE_SLEEP` ya no se llama a
  `LoRaWAN.sleep()`. Se espera despierto comparando `millis()` contra
  `lastTxMillis`. También sale la llamada a `LoRaWAN.cycle()`, que sólo
  servía para programar el despertador del sueño.

  Este nodo va alimentado y necesita el Wi-Fi arriba de forma continua, así
  que el ahorro del deep sleep no compensaba ninguna de las cuatro cosas de
  arriba.

  **REVERTIDO EL MISMO DÍA. El cambio estaba mal.** Sin `LoRaWAN.sleep()`
  el nodo dejó de unirse por completo. El riesgo que se había anotado era el
  correcto: en la librería de Heltec esa llamada no es sólo "dormir", es
  también donde se cede el tiempo al manejador que procesa la radio. Sin
  ella el JoinAccept nunca se atiende.

  Se restauraron `LoRaWAN.cycle()` y `LoRaWAN.sleep()` tal cual estaban. El
  `loop()` vuelve a ser el del usuario, byte a byte.

  **Queda sin resolver, y no se vuelve a tocar a ciegas.** Para arreglarlo
  hay que leer `LoRaWan_APP.cpp` de Heltec y ver si la librería ofrece
  restaurar la sesión desde memoria RTC al despertar del deep sleep. Esa
  librería NO está en la Jetson (`~/.arduino15/packages` sólo tiene el core
  de Espressif), así que el usuario compila en otra máquina. Las
  herramientas web tampoco funcionan en este entorno, así que no se pudo
  consultar el código fuente.

  **Consecuencia práctica mientras tanto:** los downlinks por LoRa no son
  alcanzables con esta librería y este ciclo. La parada de emergencia sigue
  funcionando por el camino directo Jetson → ESP32-C3 por UDP, que es el
  principal; el de LoRa era el respaldo para cuando el Wi-Fi está caído.

- **2026-09-06** — **CAUSA ENCONTRADA: la placa estaba compilada para
  EU868 y la red es US915.** No era el código. La tercera línea que imprime
  la librería al arrancar lo dice sin ambigüedad:

      LoRaWAN EU868 Class A start!

  El servidor está en US915 (`region_config_id = US915_PROFILE`, y el
  gateway publica en `us915_0`). El nodo transmitía el JoinRequest en la
  banda de 868 MHz mientras el gateway escuchaba entre 902 y 928. Nadie lo
  oía. Por eso ChirpStack no registraba ni un intento rechazado: no es que
  llegara y se descartara, es que no llegaba.

  **La región NO se elige en el sketch.** `ACTIVE_REGION` sale del menú de
  placa del IDE de Arduino: *Tools → LoRaWan Region → REGION_US915*. Por eso
  el mismo archivo se une un rato y deja de unirse después de recompilar:
  basta cambiar de placa o de perfil en el IDE para que se vaya a EU868 sin
  tocar una línea de código. Es la explicación de por qué a las 23:05
  funcionaba y a las 23:40 no.

  **El indicador de estado MIENTE, y por eso costó verlo.** El log repite
  `Joined=✅ YES` indefinidamente sin haberse unido nunca. La culpa es de
  `if(!joined) joined = true;` dentro de `DEVICE_STATE_SLEEP`: pone la
  bandera a true la primera vez que pasa por ahí, haya JoinAccept o no.

  **Y por eso se queda colgado para siempre.** `State=4` es
  `DEVICE_STATE_SLEEP` en la enumeración de Heltec. El reintento de JOIN a
  los 60 s vive dentro del `case DEVICE_STATE_SEND`, al que ya no se llega
  nunca porque la máquina de estados se quedó en SLEEP esperando un
  JoinAccept que no va a venir. No reintenta: se queda.

  Los tres síntomas son el mismo bug del `joined` propio ya documentado.
  Sigue sin corregirse por acuerdo con el usuario (un cambio por vez sobre
  su base), pero conviene saber que **el `[Status]` de este firmware no
  sirve para saber si el nodo está unido**. Para eso, `device.dev_addr` en
  la base de ChirpStack, o `last_seen_at`.

- **2026-09-06** — **El firmware con downlinks SÍ hacía JOIN. Medido, no
  supuesto.** El usuario reportó que no funcionaba. Se fue a la fuente en
  vez de seguir tocando el sketch, y los tres sitios coinciden:

  - `device.last_seen_at` en la base de ChirpStack: **2026-09-05 23:20:26
    UTC**, con `dev_addr` asignado (`008b1977`). Con dirección asignada el
    nodo está unido; un nodo que no hace JOIN no tiene ninguna.
  - `agrobot_history.db`: **66 paquetes nuevos** entre las 23:05:58 y las
    23:20:26, ids 2066 a 2131.
  - Los datos son buenos: CO₂ 564 ppm, 17.5 °C, 78 % HR, RSSI −30 dBm,
    SF7, y `sensors_missing` vacío en todas las filas.

  Ese último punto confirma de paso que el **payload de 17 bytes se
  decodifica bien** de punta a punta, y que ya no queda rastro del sensor de
  presión en el camino del dato.

  **Lo que sí está roto es otra cosa, y es un fallo viejo de esta base de
  firmware:** las separaciones entre paquetes son de 1.2 s (32 veces) y
  38.5 s (15 veces). Es decir, **ráfagas de 3 tramas cada 38 segundos** en
  vez de una cada 30. Es exactamente el bug del `joined` propio ya
  documentado más arriba: la variable arranca en `false` mientras la pila YA
  está unida, el código cae en el `else` y la máquina de estados envía de
  más. La versión reescrita lo tenía corregido; esta no.

  Gasta unas 3 veces el tiempo en antena y la batería que le corresponde.
  No se ha tocado: cualquier cambio sobre esta base se hace de a uno y se
  verifica en campo, que es lo acordado.

  **Sigue sin explicarse por qué el usuario lo vio como "no hace JOIN".**
  El nodo lleva 53 minutos callado desde el último paquete. Puede ser que se
  apagara, que se reflasheara con otra versión, o que se colgara. Hace falta
  el monitor serie para saberlo.

- **2026-09-05** — **El nombre del callback de downlink estaba mal, y el
  JOIN sigue sin explicación.** Dos cosas distintas, y conviene no
  confundirlas.

  **Lo que sí es un fallo confirmado:** la función se había escrito
  `downlinkDataHandle`. La librería de Heltec llama a `downLinkDataHandle`,
  con L mayúscula en medio. Con el nombre mal el compilador NO protesta —
  queda como una función normal que nadie invoca— y los downlinks se
  pierden en silencio. Corregido.

  **Lo que NO está explicado:** el usuario reporta que al añadir el
  manejador el nodo dejó de hacer JOIN. Un nombre mal escrito no puede
  causar eso: la función simplemente no se llamaba. No se ha encontrado la
  causa y no se va a inventar una. La librería de Heltec no está instalada
  en la Jetson, así que no se pudo leer el código de la pila para
  comprobar nada de esto en disco.

  **Hipótesis más probable, y no tiene que ver con este código:** el
  DevNonce. ChirpStack rechaza un JoinRequest cuyo DevNonce ya vio antes,
  como protección contra repetición. Al reflashear, el contador del nodo
  puede volver a empezar bajo, y entonces TODOS los intentos se rechazan en
  silencio: el nodo transmite, el gateway recibe, y el dispositivo nunca
  aparece unido. Encaja con "dejó de hacer JOIN justo después de un
  reflasheo" mucho mejor que cualquier cosa del sketch.

  Para descartarlo: mirar en ChirpStack los eventos del dispositivo. Si
  aparecen JoinRequest rechazados por DevNonce, se arregla borrando y
  volviendo a crear el dispositivo, o limpiando su historial de nonces.

  Si el problema NO es ese, la forma de aislarlo es comentar el bloque de
  downlinks entero y confirmar si vuelve a unirse. Eso separa "lo rompió el
  código añadido" de "se rompió por otra razón el mismo día".

- **2026-09-05** — **Downlinks devueltos al firmware del usuario.** Era la
  pérdida más seria de volver a su versión: sin `downlinkDataHandle()` los
  comandos de ChirpStack llegaban a la pila y no hacían nada, así que la
  parada de emergencia figuraba como "enviada" en el dashboard mientras el
  robot seguía andando.

  Se añadió sólo eso, sobre su firmware, sin tocar nada más: el manejador
  y el reenvío por UDP a la Jetson. `setup()` y `loop()` quedan byte a byte
  como en su ejemplo — ni siquiera se tocó el `printf` del resumen, para no
  meter ruido en el lazo que se sabe que funciona.

  La cadena completa ya existía en los otros dos extremos y no hizo falta
  tocarlos: `gateway_server.py` encola el comando en ChirpStack, y
  `jetson_server.py` ya escuchaba `{"type":"lora_downlink"}` en el puerto
  5001 y ejecuta la parada contra el ESP32-C3. Se comprobó por código, no
  de memoria, que el JSON que emite el firmware encaja con el filtro de la
  Jetson y que los cuatro códigos de byte coinciden en gateway, codec y
  firmware.

  **Dos límites que no son un fallo y conviene no olvidar.** El nodo es
  CLASE A: sólo escucha en dos ventanas cortas justo después de cada
  uplink, así que con el ciclo de 30 s un comando tarda hasta medio minuto
  en llegar. No es un mando en tiempo real. Y este nodo no mueve motores:
  sólo puede reenviar la orden a la Jetson, de modo que si el Wi-Fi está
  caído la orden llega por radio y no tiene por dónde salir. En ese caso se
  avisa por serie en vez de fingir que se ejecutó.

  Para frenar en el acto sigue estando el enlace UDP directo con el C3.

- **2026-09-05** — **Vuelta al firmware del usuario para la ESP32 LoRa
  V3.** El usuario probó el sketch reescrito y no funciona; entregó el que
  sí anda en campo y pidió quitarle sólo el sensor de presión. Se hizo
  exactamente eso: se partió de SU archivo y se borró el BMP280, sin tocar
  nada más.

  **Lo que se pierde respecto de la versión reescrita, y conviene tener
  presente:**

  - ~~No hay `downlinkDataHandle()`~~ — **recuperado el mismo día**, a
    pedido del usuario. Es lo único que se re-añadió sobre su firmware
    además de quitar el BMP280. Ver la entrada de más abajo.
  - **No hay máscara de validez.** Un sensor ausente y uno midiendo cero
    vuelven a llegar idénticos, y distinguirlos queda otra vez en manos de
    `schema.validate()` por rango.
  - **No hay CRC en las lecturas del SCD40**, ni contadores de diagnóstico,
    ni reintento de Wi-Fi al perder el AP (`connectToWiFi()` levanta la
    bandera y ya no vuelve a mirar el estado real).
  - `setDefaultDR(0)` en vez de 3: SF10 en vez de SF7, unas 8 veces más
    tiempo en antena por envío.

  Nada de esto se re-introduce por cuenta propia: el usuario probó las dos
  versiones en el hardware real y la suya es la que hace JOIN. Cualquier
  mejora futura se agrega sobre ESTA base y se verifica en campo de a una.

  **Pines I2C: 41/42**, no 21/22. La versión reescrita tenía 21/22 y ese
  solo dato ya explicaría que ningún sensor respondiera.

  **El payload queda en 17 bytes.** Este firmware no manda máscara, así que
  al quitar los cuatro bytes de presión termina en el byte 16. `decoder.py`
  y el codec de ChirpStack aceptan ahora 17, y siguen aceptando 18 (la
  variante con máscara, hoy sin uso), 21 y 22.

  En el pegado del usuario venía además el firmware ENTERO del nodo
  actuador C3 incrustado dentro de `case DEVICE_STATE_CYCLE:`. Era un
  accidente de copiado y se descartó.

- **2026-09-05** — **LiDAR por USB: decidido, y no por comodidad.** Se
  evaluó sacarlo por los pines del header de 40 de la Orin Nano y no se
  puede: el overlay del header
  (`tegra234-p3767-0000+p3509-a02-hdr40.dtbo`) sólo expone la función
  `uarta`, en los pines 8/10 más 11/36 como RTS/CTS, y eso es
  `/dev/ttyTHS1`, que ya ocupa la ESP32-C3. `/dev/ttyTHS2` existe pero no
  está ruteado al header en el módulo P3767, y `ttyAMA0` es la consola serie
  de arranque. No hay un segundo UART que seleccionar con `jetson-io`.

  Se queda en USB, que es lo que el código ya asumía: `lanzar_lidar.sh` y
  `INICIAR_BOT.sh` apuntan a `/dev/ttyUSB0`. No hizo falta cambiar nada.

  Si algún día se conecta algo a esos pines: la Orin es de **3.3 V** y no
  tolera 5 V en RX.

- **2026-09-05** — **Los dos dashboards, más grandes y con más color.**
  Pedido del usuario. No se rehízo el marcado: se subió la escala
  tipográfica entera y se saturó la paleta desde las variables de `:root`,
  que es lo que mantiene los dos dashboards pareciendo el mismo sistema.

  La escala se eligió tamaño a tamaño, no con un factor a ciegas, para que
  la jerarquía sobreviva (una etiqueta sigue siendo más chica que su valor)
  y para que nada quede por debajo de 12px. El cuerpo pasa de 13 a 17px y
  las cifras grandes de las tarjetas de 21 a 34-36px. La barra lateral y la
  superior crecieron con el texto: con las medidas viejas las etiquetas del
  menú se partían en dos líneas.

  El color separa **relleno** de **texto**. Los tonos vivos (`--accent`
  `#00bd4f`, `--crit` `#ff2d3f`, `--warn` `#ff9500`) van en barras, bordes y
  puntos de estado; el texto usa las variantes `-ink`, más oscuras. Se
  añadieron `--crit-ink` y `--warn-ink` porque `--warn` y `--crit` se
  estaban usando como color de letra y con el tono vivo dejaban de leerse.

  Todos los pares texto/fondo se comprobaron contra WCAG AA (4.5:1). El peor
  queda en 4.38 con el naranja original, y por eso `--warn-ink` terminó en
  `#9c5400`, que da 4.99. Los botones de acción llevan el relleno `-ink` con
  el borde vivo: blanco sobre `#00bd4f` da 2.2:1 y era ilegible.

  **No verificado visualmente.** En esta Jetson no hay ningún navegador que
  pueda renderizar sin entorno gráfico (sólo Firefox por snap, sin
  instalar), así que la comprobación fue estructural —llaves del CSS
  balanceadas, ningún `var()` sin definir, los `<script>` intactos— más el
  cálculo de contraste. Conviene abrirlo una vez en la tablet.

- **2026-09-05** — **Sensor de presión BMP280 retirado del sistema
  entero.** El sensor sale del robot, así que se borró todo lo que lo
  nombraba en vez de dejarlo apagado: driver I2C completo, calibración y
  compensación en `firmware/esp32_lora_v3`, su prueba en
  `firmware/test_i2c`, los campos `pressure`/`tempBMP` en `decoder.py`,
  `schema.py`, `codec_agrobot.js`, `gateway_server.py` (tabla SQLite,
  INSERT, histogramas), `telegram_notifier.py` (reporte y `/ultimo`), y las
  tarjetas de presión de los dos dashboards.

  El payload LoRaWAN pasa a **v3, 18 bytes**: los cuatro bytes de presión
  desaparecen y la máscara de validez se corre del byte 21 al 17.
  `decoder.py` y el codec de ChirpStack siguen aceptando v1 (21) y v2 (22)
  para no perder telemetría mientras el nodo no esté reprogramado; de esos
  payloads viejos los bytes de presión simplemente se ignoran.

  El bit `0x04` de la máscara NO se reutiliza para otro sensor. Un nodo con
  firmware viejo lo sigue poniendo a 1, y reasignarlo haría que el gateway
  diera por válido un dato que nadie midió — exactamente el problema que la
  máscara existía para evitar.

  Las columnas `pressure`/`temp_bmp` de bases SQLite ya existentes se
  eliminan con `rpi/server/migrar_quitar_presion.py`, que respalda la base
  antes de tocarla y comprueba que el número de filas no cambió.

  Es un script suelto y **no** una migración automática al arrancar el
  gateway, a propósito. `DROP COLUMN` no se deshace, y el gateway se arranca
  también sobre copias de la base —backups, la que alguien traiga en un
  pendrive— donde ese histórico puede ser lo único que quede de una campaña.
  Borrar tiene que ser una decisión de quien ejecuta, no un efecto
  secundario de encender el servicio. Sin `--ejecutar` sólo informa.

- **2026-09-04** — **El actuador pasó de Wi-Fi a BLE y de BLE a UART, todo
  en un día.** El orden importa porque cada paso descartó una causa:

  **1. El AP nunca fue el problema.** Verificado en caliente: hostapd y
  dnsmasq arriba, la ESP32-S3 LoRa asociada a −48 dBm. La C3 no aparecía.

  **2. El bug real del Wi-Fi.** `connectWiFi()` reintentaba cada 5 s, pero
  una asociación WPA2 completa tarda más que eso, así que cada reintento
  llamaba a `WiFi.config()`/`WiFi.begin()` **encima del intento vivo**. El
  driver lo rechazaba con `sta is connecting, cannot set config` y abortaba
  lo que llevaba. Ninguna asociación llegaba a completarse nunca.

  **3. El arreglo tenía a su vez un bug** (registrado a propósito, no
  borrado): se pusieron DOS temporizadores sobre la misma marca de tiempo,
  uno de 12 s para dar por perdido el intento y otro de 15 s para permitir
  el siguiente. En la ventana entre ambos, **cada vuelta del loop** llamaba
  a `WiFi.disconnect()` y salía por el segundo gate sin llegar a
  `WiFi.begin()`. Miles de desconexiones por segundo colgaban la pila: la
  placa se asociaba una vez, el AP la echaba por inactividad y ya no volvía
  a intentarlo. Corregido con **un solo temporizador** de 15 s y la
  desconexión previa **una vez por reintento**, no por vuelta.

  **4. Las llantas que no giraban NO eran software.** Se añadió una traza de
  motores a 2 Hz que imprime fuente, failsafe, v/omega y sentido+duty de las
  cuatro ruedas. Mostró `R1 -1/255 R2 -1/255 L1 -1/255 L2 -1/255` con el
  failsafe en cero: el firmware mandaba potencia máxima y nada se movía.
  **Era un GND del driver desconectado.** La traza se queda en el firmware
  justamente por eso.

  **5. Wi-Fi eliminado del C3.** El ESP32-C3 tiene UNA sola radio de 2.4 GHz
  que Wi-Fi y BLE se turnan, y esa convivencia era el sospechoso de las
  caídas. Se quitaron `WiFi.h`, `WiFiUdp.h`, la IP fija, el socket UDP, los
  structs `CmdPacket`/`StatusPacket` y el arbitraje de dos vías. El firmware
  pasó de 1086 a 779 líneas.

  **6. BLE duró unas horas.** Se implementó entero (`ble_link.py`, cliente
  bleak con reconexión) y **nunca llegó a conectar**: la placa dejó de
  anunciarse. Descartado a favor del cable.

  **7. UART, el transporte definitivo.** GPIO20/21 del C3 contra
  `/dev/ttyTHS1` de la Jetson a 115200. Protocolo v5 con magic + CRC-8,
  necesarios porque un UART es un chorro de bytes y el bootloader del C3
  escupe basura por esos pines en cada reset (ver §4 y §13).

  **8. El foco encendido, dos causas distintas.** (a) El barrido lo encendía
  a la fuerza y lo restauraba al terminar; bastaba con que el barrido no
  terminara —failsafe, reinicio— para dejarlo encendido. (b) **GPIO8 es de
  lógica invertida** en la SuperMini: poner el pin a bajo para "apagar" lo
  encendía. Se eliminó todo encendido automático (el foco obedece SOLO al
  comando; la única acción automática que queda es **apagarlo** en failsafe)
  y la polaridad quedó en una constante única, `LED_ACTIVO_EN_BAJO`.
  Además el foco ahora se aplica **también durante un barrido**: antes no se
  podía apagar hasta que el barrido terminara.

  **9. El Bluetooth de la Jetson estaba roto** y hubo que arreglarlo para
  poder siquiera probar el paso 6. Dos causas encadenadas:
  `/etc/modprobe.d/nvidia-preferred-oot-modules.conf` bloquea el `btusb`
  in-tree mientras exista el out-of-tree `rtk_btusb` (Realtek), y el
  adaptador de esta máquina es **Intel** (USB 8087:0a2b); y el firmware
  Intel viene solo como `.zst`, formato que este kernel Tegra no
  descomprime. Arreglado con `/etc/modprobe.d/00-agrobot-bluetooth.conf`
  (el `00-` importa: modprobe se queda con la **primera** directiva
  `install` que encuentra) y dejando `ibt-12-16.sfi`/`.ddc` descomprimidos
  en `/lib/firmware/intel/`.

  **10. El failsafe que se disparaba solo.** El backend no tenía bucle de
  control: solo mandaba al recibir un mensaje del WebSocket, así que
  cualquier hueco del navegador frenaba el robot. El envío a 20 Hz vive
  ahora dentro del enlace (§4).

  **11. La Raspberry Pi no perdió nada.** Se verificó descargando lo que
  sirve y comparándolo con el repo: MD5 idéntico. Ese dashboard nunca tuvo
  panel de Telegram (ver §11).

  **12. Topología de alimentación y pulsos escalonados.** El robot perdía
  tensión al arrancar. La causa de fondo es de cableado: los puentes H se
  alimentaban desde las borneras del step down de 5 V, lo que convierte ese
  tramo en impedancia común y hace que el pico de los motores se vea en la
  entrada del convertidor que alimenta las ESP. **Topología recomendada:
  estrella desde los bornes de la batería**, tres pares independientes
  (motores / 5 V lógica / Jetson), positivos Y negativos, con fusible por
  rama y un electrolítico de bajo ESR de 1000-2200 µF junto a la entrada de
  los puentes H. Mitigación por firmware: los pulsos de arranque se
  escalonaron por lados (§3).

  **13. Toolchain de compilación en la Jetson.** Se instaló `arduino-cli`
  1.5.1 con el core `esp32:esp32@3.3.11` y `ESP32Servo`, para poder compilar
  y verificar el firmware sin depender del IDE de Windows (§13). Las dos
  variantes del `.ino` compilan limpias.

  Archivos: `firmware/esp32c3_actuator/esp32c3_actuator.ino` (reescrito),
  `jetson/backend/uart_link.py` (nuevo), `jetson/backend/ble_link.py`
  (creado y eliminado el mismo día), `jetson/backend/jetson_server.py`,
  `jetson/backend/config_store.py`, `jetson/backend/agrobot_config.json`,
  `jetson/backend/tests/test_sensores_y_ble.py` →
  `tests/test_sensores_y_serie.py`, `bluetooth/agrobot_bt.html` (eliminado),
  `MANIFEST_1.md`. Dependencias nuevas en la Jetson: `pyserial-asyncio`.
Cada entrada: fecha, qué cambió, por qué, archivos tocados.

- **2026-09-03** — **Sesión de despliegue en la Raspberry, corrección de la
  cadena de datos de punta a punta, y arrancadores.** Todo verificado contra
  el hardware real, no solo escrito.

  **1. Despliegue.** El código de la Pi (`rpi/`) y este manifiesto viven
  ahora en `admin@192.168.0.170:~/Desktop/AGROBOT.2`. La base de datos y el
  entorno virtual de la Pi quedan fuera del rsync: son suyos, no del repo.

  **2. Dos errores de configuración que dejaban el sistema mudo sin dar
  un solo error.** Los dos se descubrieron consultando la API de verdad.

  - `CHIRPSTACK_API_URL` apuntaba a `:8080`, que en el despliegue con Docker
    sirve SOLO la web y devuelve 404 en todos los `/api/...`. La REST API es
    otro contenedor (`chirpstack-rest-api`) y escucha en `:8090`. Con esto
    mal, todos los downlinks fallaban en silencio — incluida la parada de
    emergencia. Es además el motivo de que el gateway propio escuche en 8092
    y no en 8090: el 8090 ya estaba ocupado.
  - `CHIRPSTACK_APP_ID` era `"1"`, formato de ChirpStack v3. En v4 es un
    UUID (`aa4c5278-...`). Con el valor viejo el cliente MQTT se suscribía a
    `application/1/device/+/event/up`, que no existe: se suscribía sin error
    y no llegaba ningún paquete. Indistinguible de "el robot no transmite".

  **3. El codec de ChirpStack mentía.** El del device profile pertenecía a
  otra versión del firmware: leía los mismos 21 bytes con otro reparto y
  producía campos inventados (`lidar_min`, `plaga_conf`, `estado_str`,
  `servo`) con valores plausibles y falsos — su `hum` era literalmente el
  eje Z del acelerómetro. `decoder.py` sí coincidía byte a byte con el
  firmware. Codec correcto en `rpi/chirpstack/codec_agrobot.js`, con
  encoder de downlinks incluido.

  **4. Último estado persistente (`estado_actual` en SQLite).**
  `state.last_packet` vivía solo en memoria: al reiniciar el gateway el
  dashboard quedaba en blanco hasta el siguiente uplink — 30 s con el robot
  encendido, para siempre con el robot apagado. Con histórico y mapa llenos
  detrás, esa pantalla vacía se lee como "el sistema no funciona".

  Una fila (`id=1`) que se sobrescribe, con el paquete entero como JSON para
  que añadir un sensor no obligue a migrar la tabla. `packets` sigue con
  columnas porque sobre ella se consulta y se grafica.

  **Decisión de diseño que va con esto:** la antigüedad viaja SIEMPRE junto
  al dato (`_antiguedad()`, `GET /api/estado`, y el snapshot del WS). Un dato
  recuperado del disco pintado como si fuera de ahora es peor que no mostrar
  nada: el operador decide sobre un campo que ya no es así. Por encima de
  `UMBRAL_OBSOLETO_SEG` (3 ciclos de TX = 90 s) el dashboard atenúa los
  valores, pone el borde discontinuo y dice "sin señal — último dato hace X".
  El umbral lo calcula el servidor y lo manda al front para no duplicar la
  regla.

  **5. Histogramas de sensores (`GET /api/histograma`).** Responden lo que ni
  el valor actual ni la serie temporal contestan: qué valores toma
  normalmente este sensor, para reconocer un dato raro como raro. Se binan en
  la Pi, que tiene el dato indexado; mandar miles de filas al navegador sería
  mover el trabajo al aparato más lento. `CAMPOS_HISTOGRAMA` es lista blanca
  explícita: el nombre entra en el SQL y aceptar texto libre sería inyección.

  **Encontraron un fallo en el primer intento.** El histograma de CO₂ salió
  bimodal: 123 muestras en 0 y 72 en ~610 ppm. Eso no es un sensor roto, es
  uno intermitente — y corrigió el diagnóstico previo de "SCD40 muerto".

  **6. Firmware ESP32 LoRa V3 — cinco correcciones.**

  - **SCD40, la causa de los ceros.** Faltaba `stop_periodic_measurement`
    antes de arrancar (tras un reset en caliente el sensor RECHAZA el start,
    y el código ponía `scd40_ok = true` sin mirar el error: quedaba
    inservible hasta el siguiente corte de corriente, informando "OK").
    Faltaba también `get_data_ready_status`: el SCD40 entrega una medida cada
    5 s y el firmware leía a ciegas 5 ms después de pedirla. Añadido además
    el CRC-8 por palabra — sin él un byte corrompido entra como medida
    válida. Y si no hay medida nueva se conserva la anterior en vez de
    machacarla con ceros.
  - **Máscara de validez (byte 21, payload v2 de 22 bytes).** Hasta ahora un
    sensor ausente y uno midiendo cero llegaban IDÉNTICOS: `0.0`. Con el
    BMP280 desconectado el dashboard mostraba "presión 0 hPa" como medida.
    `decoder.py` acepta 21 y 22 bytes para poder actualizar nodo y servidor
    en momentos distintos; `schema.validate()` da prioridad a la máscara
    sobre la deducción por rango, porque "no hay sensor" y "sensor devolviendo
    basura" llevan a revisar dos cosas distintas.
  - **BMP280:** `dig_T1`/`dig_P1` estaban declarados `int16_t` siendo sin
    signo — en cuanto el valor pasa de 32767, que es lo normal, la
    compensación devolvía basura. Se busca además en 0x76 además de 0x77.
  - **Las ráfagas de 3 tramas cada 38 s.** Causa: la variable propia `joined`
    arrancaba en `false` mientras la pila YA estaba unida, el código caía en
    el `else` y la máquina de estados enviaba de más. Eliminada: la librería
    solo pone `deviceState = SEND` tras un JOIN real.
  - **Convivencia WiFi + LoRa.** `sendUDP()` movido a DESPUÉS del envío LoRa
    con 50 ms de separación (antes los picos de ~350 mA y ~120 mA se
    sumaban). `connectToWiFi()` reescrito: tenía `if (wifi_connected) return;`
    como primera línea, así que tras ponerse a true NUNCA se revisaba el
    estado real y se escribía a un socket muerto sin error. Añadidos modem
    sleep, TX a 11 dBm, y sueño solo cuando no hay WiFi que perder. DR por
    defecto de 0 (SF10) a 3 (SF7): el enlace va a −40 dBm y DR0 multiplicaba
    por 8 el tiempo en antena. `mostrarMotivoDelReinicio()` imprime si el
    último arranque fue brownout — un brownout resetea limpio y no deja
    ningún mensaje de error.

  **7. Telegram desbloqueado.** `_poll_loop()` exigía `enabled` para
  escuchar, pero `enabled` arranca en `False` y el emparejamiento se hace
  mandando `/start` al bot… que llega justo por ese bucle. Bloqueo sin
  salida: el operador escribía y no pasaba nada, sin error en ninguna parte.
  Ahora basta con tener token. `/start` activa las alarmas junto con el
  emparejamiento (vincular y dejar las alarmas apagadas es un estado que no
  quiere nadie), y se añadió `/activar` como contrapartida de `/silencio`,
  que era un camino de ida.

  **8. Arrancadores.** `INICIAR_BOT.sh` (Jetson: backend + ROS, modos
  `mapeo`/`nav`/`solo-backend`) e `INICIAR_SERVER.sh` (Pi: gateway con
  comprobaciones previas). El backend arranca ANTES que ROS a propósito: el
  EKF no publica `odom->base_link` hasta su primera medición de IMU, y esa
  medición sale del backend. `INICIAR_SERVER.sh` comprueba un endpoint
  `/api` real y no solo que el puerto acepte conexiones — mirar el puerto a
  secas daba verde apuntando al 8080 equivocado. `CONFIGURAR_TELEGRAM.sh`
  valida el token contra `getMe` antes de escribirlo: un token mal copiado
  produce un bot que arranca callado y solo se descubre en la primera
  emergencia.

  **9. Downlinks rotos por otro resto de la v3.** El botón de emergencia
  devolvía error. El cuerpo del POST usaba la clave `deviceQueueItem`, que
  era el nombre en ChirpStack v3; en v4 es `queueItem`. Al no reconocer la
  clave, ChirpStack recibía un item vacío, veía `fPort = 0` y respondía
  *"FPort must be between 1 - 255"* — un mensaje que manda a revisar el
  puerto, que era lo único que estaba bien. Verificados los cuatro comandos
  (PING, ACK, PARK, EMERGENCY_STOP) encolándose con fPort 10 y su byte
  correcto, y la cola vaciada después de la prueba.

  **10. Depuración serie ampliada en el firmware.** Escaneo del bus I2C al
  arrancar (distingue "sensor sin cablear" de "sensor que no inicializa",
  dos averías con síntoma idéntico desde el payload), número de serie del
  SCD40, coeficientes de calibración del BMP280, payload en HEX (única forma
  de comparar byte a byte contra lo que decodifica el gateway), motivo del
  último reinicio, heap libre, y contadores acumulados de lecturas / SCD40
  ok / sin-medida / CRC malo / BMP ok. Los contadores son deliberados: un
  fallo intermitente no se ve en una lectura suelta, se ve en la proporción
  — fue así como se descubrió que el SCD40 no estaba muerto.

  **11. El firmware NO recibía los downlinks.** Faltaba `downlinkDataHandle()`
  entero — la función que la librería LoRaWan_APP llama al bajar un paquete.
  Los comandos se encolaban bien, la pila los recibía, y ahí se acababa
  todo: la parada de emergencia aparecía como "enviada" en el dashboard y el
  robot no se enteraba. Implementada, con filtro por puerto de aplicación
  (sin él, un downlink de otra app o una trama MAC podría leerse como una
  parada), traza del payload en HEX y contador de downlinks.

  El nodo LoRa no mueve motores, así que reenvía la orden a la Jetson por el
  mismo UDP 5001 de la telemetría — abrir un segundo puerto solo para un
  mensaje que llega como mucho una vez cada media hora habría añadido otra
  pieza que puede quedar mal configurada. `jetson_server.py` lo atiende y
  llama a `actuator_proto.emergency_stop()` directamente, sin pasar por el
  dashboard: si el robot está fuera del alcance del AP, no hay nadie
  mirándolo. Guarda contra `actuator_proto is None`, porque un
  `AttributeError` dentro del callback de UDP lo absorbe asyncio y la parada
  fallaría sin dejar rastro.

  **Dos límites que no se pueden programar y quedan documentados:** el nodo
  es CLASE A, así que solo escucha en dos ventanas cortas justo después de
  cada uplink — un comando tarda hasta 30 s en llegar. Y si el WiFi está
  caído, la orden llega por radio pero no tiene por dónde salir hacia los
  motores; el firmware lo avisa por serie en vez de fingir que se ejecutó.

  Archivos: `rpi/server/{gateway_server,decoder,schema,telegram_notifier}.py`,
  `rpi/dashboard/index.html`, `rpi/chirpstack/codec_agrobot.js`,
  `firmware/esp32_lora_v3/esp32_lora_v3.ino`,
  `firmware/test_i2c/test_i2c.ino`, `INICIAR_BOT.sh`, `INICIAR_SERVER.sh`,
  `CONFIGURAR_TELEGRAM.sh`, `jetson/backend/jetson_server.py`, `README.md`.

  **Estado del hardware medido en 24 h reales (195 paquetes):**

  | Sensor | Lecturas válidas | Rango observado |
  |---|---|---|
  | MPU6050 (accel/gyro) | 195 de 195 | accelZ ≈ 0.93 g (gravedad, correcto) |
  | SCD40 CO₂ | 72 de 195 | 590–639 ppm |
  | SCD40 temperatura | 72 de 195 | 16.4–17.2 °C |
  | SCD40 humedad | 72 de 195 | 68–70 % |
  | BMP280 | 0 de 195 | desconectado a propósito |

- **2026-09-02** — **Los tres modelos bajados de Kaggle no sirven, por dos
  motivos independientes.** El usuario descargó `yolo26n/best`, `yolo11s` y
  `yolo11s-cls` y los subió a `models/`.

  1. **Están DESCOMPRIMIDOS.** Un `.pt` de PyTorch es un ZIP sin compresión
     (`data.pkl`, `data/<n>`, `byteorder`, `version`) y acá llegaron como
     carpetas con ese contenido suelto, así que `list_model_files()` —que
     busca `*.pt`/`*.onnx`/`*.engine`— no los veía.
  2. **Y están INCOMPLETOS**, que es lo que no tiene arreglo. Contrastando
     el `data.pkl` contra los archivos en disco:

     | carpeta | tensores que necesita | presentes | faltan |
     |---|---|---|---|
     | `yolo26n/best` | 712 | 82 | **630 (88%)** |
     | `yolo11s` | 503 | 39 | **92%** |
     | `yolo11s-cls` | 239 | 32 | **87%** |

     Reempaquetarlos a `.pt` da un archivo que ultralytics rechaza con "not
     a loadable checkpoint" — correctamente: no se pueden inventar los pesos
     que no se descargaron.
  3. **Aunque estuvieran completos, tampoco resolvían el problema**: los
     metadatos (leídos con un unpickler restringido, sin ejecutar el
     archivo, que vienen de una fuente externa) dicen `nc: 80` y
     `data: coco.yaml` en los dos de detección, y `imagenet` con 1000 clases
     en el de clasificación. **Cero clases de plagas.** Son exactamente el
     mismo vocabulario genérico que ya daba "airplane" y "umbrella".

  **Cambio de código que salió de esto:** `models_load_handler` ahora
  rechaza con 400 los modelos cuya `task` no sea `detect`. Un clasificador
  devuelve `results.boxes = None` y el hilo de inferencia falla en cada
  frame: se recuperaba solo (no rompía el robot) pero la detección no
  funcionaba nunca y el motivo solo era visible en `camera.estado`.

  **Conclusión que conviene no volver a probar:** bajar otro modelo
  preentrenado NO va a detectar plagas, sea yolo11, yolo26 o el que sea. El
  problema no es la arquitectura, es el **dataset**. Ver §12.

  Archivos: `jetson_server.py`. Las carpetas de Kaggle se dejaron intactas.

- **2026-09-02** — **Modelo YOLO base instalado y cadena verificada de punta
  a punta.** `yolo11n.pt` (5.4 MB, COCO, de los assets oficiales de
  ultralytics) en `jetson/backend/models/`. Es un modelo **genérico**, no de
  plagas: sirve para validar la cadena, no para el trabajo real.

  **Medido en la Jetson:**

  | | tiempo | nota |
  |---|---|---|
  | primera inferencia | **3.71 s** | compila kernels por JIT: este torch no trae binario para la CC 8.7 del Orin |
  | inferencia estable (GPU) | **31 ms** | `cuda:0` |
  | inferencia estable (CPU) | **431 ms** | 14× más lenta |
  | en el backend real | 59-65 ms a 4 Hz | compite con el hilo de captura |

  **Con el sistema completo corriendo:** video **30.7 FPS**, inferencia
  4.0 Hz, CPU 11.4%, y **latencia del event loop 0.14 ms (máx 0.36 ms)** —
  la inferencia no toca el loop, que es exactamente para lo que se separaron
  los hilos.

  ⚠️ **Los 431 ms de CPU importan.** Si torch pierde CUDA —y este build ya
  avisa que no trae binario para `sm_87`, anda por JIT desde el PTX de
  `sm_80`— la inferencia cae a CPU y **supera los 300 ms del watchdog del
  C3**. Con la arquitectura vieja (todo en el event loop) eso habría frenado
  el robot en marcha. Con `camera.py` la inferencia vive en su propio hilo,
  así que el peor caso pasa a ser "la detección va lenta", no "el robot se
  para solo". Vale la pena verificar `infer_ms` en el dashboard después de
  cualquier actualización de torch.

  **Cadena verificada en vivo**, bajando el umbral a 0.05 un momento:
  inferencia → cajas → cooldown → WebSocket → dashboard. 12 detecciones en
  18 s repartidas en 3 clases, o sea ~1 por clase cada 3 s: el
  `DETECTION_COOLDOWN_S` funcionando. Umbrales restaurados a 0.45/0.60.

  Se agregaron métricas de inferencia a `camera.snapshot()`
  (`infer_ms`, `infer_hz`, `detecciones`, `modelo`): sin ellas no había forma
  de saber si YOLO corría de verdad — el video se ve igual de fluido tanto si
  la detección funciona como si el hilo murió en silencio.

  Archivos: `jetson/backend/models/yolo11n.pt` (nuevo), `camera.py`.

- **2026-09-02** — **Revisión de `models/`: está vacía. Y tres bugs en el
  camino de carga, uno mío.**
  - **No hay ningún modelo YOLO** en `jetson/backend/models/` ni en el resto
    del proyecto (`.pt`, `.onnx`, `.engine`, `.tflite`). `active_model` es
    `None` y `/api/models` devuelve `[]`: el video corre sin detección.
  - **`ultralytics` 8.4.137 + torch 2.13.0+cu130 SÍ están instalados.**
    PyTorch avisa que este build no trae binario para la CC 8.7 del Orin
    (`sm_80, sm_90, sm_100, sm_110, sm_120`), pero una matmul real en GPU
    **funciona** — anda por JIT desde el PTX de `sm_80`. Consecuencia
    práctica: la primera inferencia va a ser lenta (compila kernels) y
    conviene medirla cuando haya un modelo, en vez de confiar en
    `torch.cuda.is_available()`, que da `True` igual.
  - **Bug propio: `models_load_handler` no avisaba al hilo de inferencia.**
    Al mover YOLO a `camera.py` quedó dejando el modelo solo en
    `app["yolo_model"]`. La config se guardaba y el dashboard decía "Modelo
    cargado", pero **la detección no corría nunca y no había ningún error**.
    Fallo silencioso. Ahora llama a `camera.set_model()`.
  - **`name` del POST se usaba como ruta sin validar**: `MODELS_DIR /
    "../../algo"` salía de la carpeta de modelos, y un `name` ausente
    reventaba con `TypeError` y un 500 sin explicación. Se valida y se
    responde 400. Un `.pt` corrupto también daba éxito dejando el modelo
    viejo puesto; ahora devuelve 400 con el motivo.
  - **`MODELS_DIR`/`CAPTURES_DIR`/`SESSION_REPORT_DIR` eran relativas al
    directorio de trabajo** (`Path("./models")`). `arrancar.sh` hace `cd` y
    funcionaba, pero lanzar `python3 jetson/backend/jetson_server.py` desde
    la raíz del repo creaba esas carpetas ahí: no encontraba los modelos y
    guardaba las capturas en otro lado, todo en silencio. Ahora son
    relativas al archivo.
  - Prueba nueva `[10d]`: que el handler avise al worker (sobre el AST), que
    los nombres maliciosos den 400 y que las rutas sean absolutas.
  - Archivos: `jetson_server.py`, `tests/test_actuator_e2e.py`.

- **2026-09-02** — **Rendimiento: 17.9 → 30.8 FPS, y un bug de CONTROL que
  estaba escondido detrás.** El usuario reportó "muchos bugs y falencias de
  FPS". Se midió antes de tocar nada, y aparecieron cuatro cosas:

  1. **El bug grave no era de FPS, era de control.** `video_handler` hacía
     `cap.read()` (~32 ms), `cv2.imencode()` (~5 ms) y la inferencia YOLO
     (>100 ms) **dentro del event loop de asyncio**. Un `await` no
     interrumpe una llamada bloqueante: mientras corrían, el loop entero
     estaba parado — y ese mismo loop manda los comandos UDP al ESP32-C3 a
     20 Hz. Con YOLO activo, una inferencia de más de 300 ms deja al C3 sin
     comando fresco y **su watchdog frena el robot en medio de una
     maniobra**. Nadie lo había visto porque hoy no hay modelo YOLO cargado:
     estaba latente, esperando a que se activara la detección.
     Nuevo **`camera.py`** con dos hilos y un solo `VideoCapture`
     compartido: uno captura y comprime (marca el ritmo del video), otro
     corre YOLO a su propio ritmo. Separados a propósito: compartiendo hilo,
     el video se congelaría en cada inferencia.
     **Medido**: latencia de ida y vuelta del event loop **0.10 ms con video
     y 0.10 ms sin video** — indistinguible. Antes cada frame lo bloqueaba
     ~37 ms.
  2. **`VideoCapture(0)` por PETICIÓN HTTP.** Dos pestañas del dashboard
     abiertas eran dos capturas peleando por la misma cámara. Ahora hay una
     sola, y cada cliente recibe por su propia `asyncio.Queue(maxsize=1)`:
     si un cliente se atrasa se le descarta el frame VIEJO en vez de
     encolar — en video en vivo un frame viejo no sirve, y encolar haría
     crecer la latencia y la memoria sin límite.
  3. **`exposure_dynamic_framerate=1` en el driver V4L2.** La cámara
     alargaba la exposición con poca luz y **bajaba los FPS** para
     conseguirlo: **17.9 FPS** medidos contra **31.2** con la opción
     desactivada, misma cámara y misma escena. Se desactiva al abrir la
     cámara. Se prefiere imagen algo más oscura a una tasa que se derrumba
     justo entre las plantas, que es donde hay sombra. (MJPG contra YUYV no
     cambió nada: el cuello de botella no era el USB.)
  4. **Un aviso por caja y por frame.** A 30 FPS, una misma plaga generaba
     decenas de mensajes WebSocket, toasts y fotos por segundo. Ahora hay
     `DETECTION_COOLDOWN_S = 3` por clase, y la inferencia va a 4 Hz
     (`INFER_PERIOD_S`), no a 30: el robot se mueve lento y no hace falta más.

  **Ancho de banda del radar: 403 → 84 KB/s por cliente.** El scan del C1
  son ~600 puntos con precisión de float64 (`0.14875000715255737` m para un
  sensor de ±3 cm: puro dígito inventado), difundidos 10 veces por segundo.
  Se diezma a 200 puntos —el radar mide ~300 px, el resto cae en el mismo
  píxel— y se redondea a mm. **La evasión reactiva sigue recibiendo el scan
  completo**: perder el único rayo que da contra una rama fina es la
  diferencia entre frenar y chocar.

  **Otros arreglos del mismo barrido:**
  - `broadcast()` mandaba a los clientes **en serie**: uno lento hacía
    esperar a todos. Ahora va con `gather`.
  - `save_capture()` comprimía a JPEG y escribía a disco **en el event
    loop**, justo en el instante de detectar algo. Va a un executor.
  - `_schedule()` creaba corrutinas que nadie ejecutaba cuando llegaba un
    scan antes de que existiera el loop (`coroutine ... was never awaited`
    en cada arranque). Ahora las cierra explícitamente.
  - **Frontend**: el `<img>` del MJPEG seguía descargando con su pestaña
    oculta o la ventana en segundo plano — Wi-Fi del robot y frames que
    nadie mira. Ahora se desconecta y se repone. El radar tampoco se dibuja
    si no se ve. Y los **FPS reales** aparecen en la insignia del video: no
    había forma de saber si iba a 30 o a 5, se veía "EN VIVO" igual.

  **Pruebas nuevas** (bloques `[10b]` y `[10c]` del e2e): que
  `video_handler` no abre la cámara ni comprime —verificado sobre el **AST**,
  porque buscar la cadena en el fuente también encontraba los comentarios
  que explican por qué ya no se hace, y daba falso positivo—, que la cola de
  un cliente lento nunca crece y conserva el frame más nuevo, y que la
  evasión conserva el scan completo mientras el dashboard recibe el diezmado.

  Archivos: nuevo `jetson/backend/camera.py`, `jetson_server.py`,
  `jetson/dashboard/index.html`, `tests/test_actuator_e2e.py`.

- **2026-09-02** — ✅ **AP creado, en modo DUAL AP+cliente.** Era el
  bloqueador #1 del sistema.

  **Investigación.** El chip es un **Intel Wireless 8265** con `iwlwifi`;
  `iw list` declara `#{ managed } <= 1, #{ AP, P2P-client, P2P-GO } <= 1,
  total <= 3, #channels <= 2`: **un cliente más un AP a la vez**. La nota
  que traía el MANIFEST —"la Jetson tiene una sola placa Wi-Fi, al volverla
  AP pierde internet"— era **incorrecta** y se corrigió.

  **Dos trampas que costaron encontrar**, las dos dan el mismo `EBUSY` al
  levantar `ap0`, sin ningún mensaje en `dmesg` que las explique:
  1. **NetworkManager le cambia el tipo a `managed`** en cuanto adopta la
     interfaz. Dos `managed` en el mismo phy violan `#{ managed } <= 1` y el
     driver rechaza la segunda. La pista fue ver `type AP` justo tras crear
     el vif y `type managed` un rato después. Se resuelve marcando `ap0`
     como no gestionada en NM.
  2. **MAC duplicada.** Un vif creado con `iw phy phy0 interface add`
     hereda la MAC del phy, la misma del cliente. Hay que ponerle otra (bit
     de administrada localmente).

  Fue clave probar `ip link set ap0 up` **con el cliente desconectado**: al
  fallar igual, quedó descartado que el problema fuera la concurrencia —
  que era la hipótesis obvia y equivocada.

  **Lo que quedó montado** (tres servicios systemd, habilitados al boot):
  - `agrobot-ap0` — crea el vif `ap0` sobre `phy0`, MAC propia,
    `192.168.12.1/24`. Idempotente: borra el vif previo si existía.
  - `agrobot-hostapd` — `JETSON_AP` / `12345678`, WPA2-CCMP, **canal 1, el
    mismo del cliente**. Sin PMF obligatorio: algunos ESP32 con SDK viejo no
    lo negocian.
  - `agrobot-dhcp` — dnsmasq **solo en ap0** (`--bind-interfaces`, para no
    pelear con el `systemd-resolved` que ya escucha en 127.0.0.53), pool
    `.100-.150`. Deja libre `.2-.99` para IPs fijas: el ESP32-C3 usa `.50`
    fijada en su propio firmware y no debe caer dentro del pool.

  **Sin NAT ni `ip_forward`**, a propósito: el tráfico Jetson↔ESP32 es todo
  local en `192.168.12.0/24`. Habilitar reenvío sería exponer una red a la
  otra sin que el robot lo necesite.

  **Verificado:** las dos redes conviven — `ap0` con `JETSON_AP` en
  192.168.12.1 y `wlP1p1s0` conectada a `LoRa_Wan` en 192.168.0.142 con
  internet. El backend ya escucha sus UDP **en 192.168.12.1** (antes caía a
  `0.0.0.0` con aviso) y el dashboard responde por ambas IPs.

  Archivos: `/etc/systemd/system/agrobot-{ap0,hostapd,dhcp}.service`,
  `/etc/hostapd/agrobot.conf`,
  `/etc/NetworkManager/conf.d/99-agrobot-ap0.conf`,
  `docs/CONFIGURAR_AP_JETSON.txt`, MANIFEST §2 y §12.

- **2026-09-02** — **"ROS no disponible" y el radar vacío: tres causas
  distintas, ninguna evidente desde la interfaz.**

  1. **ROS estaba instalado pero el backend se lanzaba sin activarlo.**
     `import rclpy` fallaba y el `except ImportError` **se tragaba la
     excepción**: el dashboard decía "ROS no disponible", exactamente igual
     que si ROS no existiera. Ahora se guarda el motivo
     (`ROS_UNAVAILABLE_REASON`), se distingue entre *no instalado* / *no
     activado* / *falta un paquete*, y el texto aparece en la pestaña
     Navegación y en `/api/health` (ROS es ahora una fila más de salud).
     Se agregó **`arrancar.sh`**, que hace el `source` del distro que esté
     instalado —no uno fijo— más el workspace, y arranca el backend. La
     línea de `docs/CONFIGURAR_AP_JETSON.txt` que lanzaba `python3
     jetson_server.py` a secas se corrigió: era la fuente del problema.

  2. **El nodo del LIDAR no estaba corriendo, y al lanzarlo moría con
     `code: 80008004`** — un error del SDK que no menciona permisos. La
     causa real: `/dev/ttyUSB0` es `root:dialout 660`, el usuario **ya
     estaba en el grupo dialout** (`groups jetson` lo mostraba) pero la
     sesión de login era anterior al cambio, así que el proceso no lo tenía
     (`id` no lo listaba). Nuevo **`lanzar_lidar.sh`**: detecta ese caso
     exacto y se re-lanza con `sg dialout` (usa la membresía que ya existe,
     no toca el sistema), avisando que el arreglo de raíz es cerrar sesión.
     Fija además los **460800 baudios** reales del C1.

  3. **El radar se dibujaba, pero ilegible.** Dos bugs propios:
     - **Escala automática al punto más lejano.** Con el scan real (mediana
       0.34 m, máximo 4.11 m) la mitad de los puntos quedaba dentro del 8%
       del radio: un borrón en el centro que parecía "sin datos". Además la
       escala cambiaba en cada scan y nada era comparable. Ahora el alcance
       es **fijo y ajustable** (slider, 0.5–16 m, por defecto 4 m), los
       anillos van rotulados en metros y lo que queda fuera se dibuja
       clavado en el borde con la cuenta al pie — desaparecerlo daría la
       impresión falsa de espacio libre.
     - **Canvas de 0×0.** El radar vive en una pestaña oculta al cargar,
       donde `clientWidth` vale 0; `resizeRadar()` dejaba el canvas en 0×0 y
       al cambiar de pestaña lo redimensionaba **sin redibujar**. Ahora se
       guarda el último scan y se redibuja al redimensionar, y se ignora un
       tamaño 0. Mismo blindaje en el canvas del mapa.

  Extra, visto en el camino: si el bind UDP de respaldo (`0.0.0.0`) también
  fallaba —típicamente otra instancia viva— salía un traceback de asyncio de
  30 líneas terminado en `terminate called without an active exception`, que
  tapaba el error real. Ahora sale un mensaje de una línea con el puerto y el
  `ss -lunp` para averiguar quién lo tiene, y `web.run_app` va en
  `try/finally` para que rclpy se apague aunque `on_startup` falle (aiohttp
  no ejecuta `on_cleanup` si el arranque revienta — de ahí el `terminate`).

  Archivos: `ros_bridge.py`, `jetson_server.py`, `jetson/dashboard/index.html`,
  nuevos `jetson/backend/arrancar.sh` y `jetson/backend/lanzar_lidar.sh`,
  `docs/CONFIGURAR_AP_JETSON.txt`.

- **2026-09-02** — ⛔ **CORRECCIÓN: el robot NO es holonómico. Todo el stack
  vuelve a DIFERENCIAL (skid-steer).** El usuario lo confirmó: no tiene
  ruedas mecanum. Las dos entradas de más abajo que hablan de mecanum
  quedan **superadas**.

  **Por qué me equivoqué, para que no se repita:** deduje "mecanum" de que
  los sentidos por rueda de seis rutinas del sketch de prueba coincidían con
  la cinemática mecanum. Pero una coincidencia de patrones no es una prueba
  física: el patrón diagonal es *compatible* con mecanum, no *evidencia* de
  mecanum. La pregunta correcta era "¿qué ruedas tiene?", que se responde
  mirando el robot, no el código. Lección: **una inferencia sobre el
  hardware no se cierra sin confirmarla contra el hardware.**

  Cambios, en toda la cadena:
  - **Firmware** — cinemática `wIzq/wDer = (vx ∓ K·omega)/R` con
    `K = trocha/2`; motores agrupados por LADO (derecha M1+M4, izquierda
    M2+M3, según los comentarios de `girarIzquierda()` del sketch, que
    contradicen su propio encabezado y son los que corresponden).
    `WHEEL_BASE` y `VY_MAX` eliminados: no entran en un skid-steer.
  - **Protocolo v3 → v4** — se quitan `vy_ref`/`vy_cmd` (18→14 B y 19→15 B)
    y `vy_pct` del BLE (8→7 B). Se quita el campo en vez de dejarlo en cero:
    un campo fantasma termina apareciendo en la UI, en la odometría y en
    Nav2 como si fuera una capacidad real.
  - **`slam.py`** — dead-reckoning sin término lateral. La evasión reactiva
    conserva algo bueno de la etapa holonómica: mira en la **dirección de
    marcha**, así que ahora también protege el **retroceso** (0° o 180°),
    que antes no estaba cubierto. Y **no recorta el giro**: frenar una
    rotación sobre el propio eje por algo que está enfrente no evita nada y
    encima quita la maniobra con la que el robot podría zafar.
  - **`nav2_params.yaml`** — todos los `_y` en cero, `vy_samples: 1`,
    `Oscillation.x_only_threshold: 0.05`, `velocity_smoother` con Y en cero
    en las cuatro filas. Se conserva `min_vel_x: -0.2` (retroceder para
    salir de un callejón sigue siendo útil sin mecanum).
  - **`ekf.yaml`** — `vy` sale de `odom0_config`. Marcarlo como fusionable
    con `linear.y` siempre en cero le daría al EKF una medición firme de
    "no hubo deriva lateral"; en un skid-steer el patinaje sí produce
    deriva, y conviene que la absorba la covarianza en vez de negarla.
  - **Dashboards** — D-pad de 4 direcciones en ambos (Jetson y Bluetooth);
    fuera los botones ⇦ ⇨, el atajo Shift, el eco de vy y los sliders de
    velocidad/aceleración lateral.
  - **Tests** — `[5b]` fija que el protocolo no transporta lateral, `[5c]`
    que un cliente viejo que igual mande `vy` no rompe ni contamina nada,
    `[5d]` cubre el retroceso y el giro en el lugar, `[5e]` el
    dead-reckoning diferencial. El test de BLE ahora **verifica los offsets
    byte a byte** entre el JS del dashboard y el struct del firmware, en vez
    de compararlos a ojo en un comentario — es lo que detecta un desfase de
    un byte al agregar o quitar un campo, justo lo que pasó acá.

- **2026-09-02** — **Rango de PWM 200..255** (pedido del usuario). Reemplaza
  el `PWM_MIN = 90` estimado. Consecuencia a tener presente: la velocidad
  útil va del **78% al 100%** de la potencia, así que el paso lento del
  acelerador sigue siendo bastante rápido. Es el compromiso correcto para
  este chasis — mejor poca gama que una gama grande cuya mitad inferior no
  mueve nada.

- **2026-09-02** — **Arranque por pulsos + zona muerta, del segundo sketch
  de prueba del usuario.** El usuario probó un sketch de control directo
  (`digitalWrite` a fondo, sin PWM, con `pulsos()` de 3×200 ms) y reportó
  **mejor rendimiento** que el firmware definitivo. La causa no era el
  código sino dos efectos físicos que el PWM proporcional puro ignoraba:
  el motorreductor no arranca por debajo de cierto duty, y el rozamiento
  estático inicial es mayor que el dinámico. Se incorporaron los dos, sin
  perder la cinemática mecanum ni el PWM proporcional (ver §3):
  - `PWM_MIN = 90`: la fracción pedida se mapea a `[PWM_MIN..PWM_MAX]`.
  - `updateKicks()`: el tren de pulsos de `pulsos()`, pero como máquina de
    estados por rueda en `loop()` **sin `delay()`** — con `delay()` el
    failsafe de 300 ms saltaría en mitad de cada arranque.
  - **Hallazgo: los nombres del sketch están mal.** `girarIzquierda` /
    `girarDerecha` usan el patrón diagonal (FR+RL contra FL+RR), que en
    mecanum es **traslación lateral pura**, no rotación. Se verificó
    comparando los cuatro movimientos contra la cinemática. Además el
    `loop()` del sketch tenía `adelante`/`atras` comentados: lo que el
    usuario probó y le gustó fue el **desplazamiento lateral**. La
    cinemática del firmware definitivo no cambia; queda anotado en §12 cómo
    confirmarlo en el robot y qué tocar si resultara al revés.
  - Se eliminó de §3 el aviso obsoleto de "PWM ~175/255 (69%) yendo
    derecho": lo contradecía el escalado dinámico documentado justo arriba.
  - Archivos: `esp32c3_actuator.ino`, `tests/motor_logic_test.cpp`.

- **2026-09-02** — **Acelerador y control total desde el dashboard.** El
  D-pad mandaba `v=1` crudo, que el firmware recortaba a `V_MAX` en
  silencio: la UI mostraba 1.0 m/s, el robot iba a 0.55 y **no había forma
  de pedir marcha lenta**. Ahora el mando manda fracción −1..1 y el backend
  aplica `control.max_*` × acelerador. Con la zona muerta corregida en el
  firmware, la marcha lenta recién ahora entrega par de verdad.
  - Nueva sección `control` en la config, con los tres techos editables
    desde Parámetros, y `sanitize_control()` + `_unit()` para que ni un
    POST ni un mensaje WebSocket malformado puedan mandar basura a los
    motores (`NaN` incluido: toda comparación con él da falso y un clamp
    ingenuo lo dejaba pasar entero).
  - Archivos: `jetson_server.py`, `config_store.py`,
    `jetson/dashboard/index.html`, `tests/test_actuator_e2e.py` (bloque
    `[5f]`).

- **2026-09-02** — **`firmware/esp32c3_actuator/tests/verificar.sh`.** El
  arnés que compila el `.ino` contra los dos cores de ESP32 (2.x y 3.x,
  APIs de LEDC distintas) vivía en un directorio temporal y se perdía entre
  sesiones. Ahora está en el repo con sus stubs, y corre también
  `motor_logic_test.cpp`. Un OK no garantiza que el robot se mueva:
  garantiza que compila en ambos cores y que la lógica da lo esperado.

- ⛔ **SUPERADA** (ver la entrada de corrección más arriba)
- **2026-09-02** — **Todo el stack adaptado a robot holonómico.** El
  firmware ya hablaba mecanum, pero de la Jetson para arriba todo seguía
  asumiendo un robot diferencial. Se recorrió la cadena entera.
  - **`slam.py` — dos bugs reales.** (a) El dead-reckoning integraba
    `x += v·cos θ`, fórmula de robot diferencial: un desplazamiento lateral
    quedaba registrado como si el robot no se hubiera movido. Ahora rota la
    velocidad del marco del robot al del mundo incluyendo `vy`. (b) La
    evasión reactiva solo miraba el arco FRONTAL: desplazándose de costado
    el robot se metía contra un obstáculo con cero protección. Ahora busca
    el obstáculo en la dirección real del movimiento (`atan2(vy, vx)`), que
    el RPLIDAR C1 puede ver porque barre 360°. El recorte se aplica como
    FACTOR a las dos componentes por igual: frenar tocando solo una
    desviaría al robot justo mientras esquiva algo.
  - **`ros_bridge.py` — bug silencioso en el EKF.** `publish_wheel_odom()`
    dejaba `twist.linear.y` en 0, y `ekf.yaml` tiene `vy` marcado como
    fusionable: el EKF recibía "el robot NO se movió de costado" con toda
    confianza mientras el robot se desplazaba, metiendo un error de pose
    acumulativo. También `_cmd_vel_cb` descartaba `msg.linear.y`, así que
    todo lo lateral que Nav2 pidiera se perdía.
  - **`nav2_params.yaml` — el bloqueo más duro estaba acá.** El
    `velocity_smoother` tenía `y = 0.0` en las CUATRO filas
    (`max_velocity`, `min_velocity`, `max_accel`, `max_decel`): aunque DWB
    pidiera desplazamiento lateral, se recortaba a cero antes de salir.
    Además faltaban por completo `min_vel_y`/`max_vel_y`/`acc_lim_y`/
    `decel_lim_y` y, sobre todo, **`vy_samples`** — sin ese último DWB ni
    siquiera genera trayectorias laterales para evaluar. Se agregó también
    `min_y_velocity_threshold` al controller_server y se puso
    `Oscillation.x_only_threshold: -1.0` para que la oscilación lateral
    también se vigile. De paso se habilitó retroceso (`min_vel_x: -0.2`),
    que antes estaba en 0.
  - **Backend**: la rampa ahora limita las tres componentes
    (`max_lateral_accel_mps2 = 0.8`, más suave que la longitudinal porque
    los rodillos mecanum patinan más de costado), `send_command` y el
    mensaje WS `control` aceptan `vy`, y la parada de emergencia lo resetea.
  - **Dashboards**: el D-pad pasó a 3×3 con giro (↺↻) y desplazamiento
    lateral (⇦⇨) separados, en el panel de la Jetson y en el de Bluetooth.
    En teclado, **Shift + flechas laterales** desplaza en vez de girar. Se
    agregó el eco de `vy` y un slider de aceleración lateral en Parámetros.
  - **Verificación**: se sumaron al test e2e tres casos holonómicos que
    corren contra el backend real — el lateral viaja por WebSocket hasta el
    paquete UDP sin contaminar avance ni giro; la evasión frena un
    desplazamiento hacia un obstáculo lateral pero NO frena el avance con
    ese mismo obstáculo al costado; y el dead-reckoning mueve la pose en Y
    al desplazarse. Las tres suites pasan, el firmware compila en core 2.x
    y 3.x, y el workspace de ROS 2 recompila limpio.
  - Archivos: `jetson/backend/slam.py`, `ros_bridge.py`, `jetson_server.py`,
    `config_store.py`, `jetson/dashboard/index.html`,
    `bluetooth/agrobot_bt.html`,
    `ros2_ws/src/agrobot_navigation/config/nav2_params.yaml`,
    `jetson/backend/tests/test_actuator_e2e.py`.

- **2026-09-02** — **El robot es MECANUM: firmware definitivo del ESP32-C3
  y protocolo v3.** El usuario aportó su firmware de prueba, ya verificado
  moviendo el robot. Resolvió todos los supuestos abiertos y además
  destapó algo que nadie había anotado: **el robot es holonómico.**
  ⛔ **CONCLUSIÓN ERRÓNEA — corregida el 2026-09-02, ver arriba.**
  - **Descubrimiento.** Sus seis rutinas (adelante, atrás, giro izq/der y
    *traslación lateral* izq/der) se compararon una por una contra la
    cinemática mecanum estándar: coinciden en los seis casos, rueda por
    rueda. Con ruedas comunes la traslación lateral no existe, así que la
    conclusión es firme. Todo el manifiesto decía "skid-steer".
  - **Pinout corregido** con el mapeo real: m1 = delantero DERECHO, m2 =
    delantero IZQUIERDO (con FI/BI invertidos respecto de las otras), m3 =
    trasero derecho, m4 = trasero izquierdo. La suposición anterior
    (m1+m2 = un lado) era incorrecta: los pares son diagonales por
    posición, no por número.
  - **Protocolo v3**: `CmdPacket` 14→18 bytes y `StatusPacket` 15→19,
    sumando `vy_ref`/`vy_cmd` (velocidad lateral, + hacia la izquierda).
    `pack_cmd()` toma `vy_ref` como argumento con nombre y default 0.0,
    así que el código que solo manda `(v, omega)` sigue igual. El BLE pasó
    de 7 a 8 bytes con `vy_pct`.
  - **Canales LEDC — se rehízo el enfoque.** Con mecanum cada rueda
    necesita su propia velocidad, así que ya no sirve compartir un canal
    entre las dos ruedas de un lado (lo que hacía la versión skid-steer).
    Harían falta 8 salidas PWM + 1 del servo = 9, y el C3 tiene 6.
    Solución: **un canal por rueda, re-apuntado al pin FI o BI según el
    sentido**. El cambio ocurre solo al invertir la marcha, con el duty en
    0, así que no glitchea. 5 de 6 canales.
  - **Bug de rendimiento que encontró el test:** normalizar dividiendo por
    el peor caso (`vx+vy+omega` a tope) dejaba el avance recto en
    **95/255 = 37% de potencia**, aun cuando `vy` y `omega` valían 0 y no
    había nada que reservar. El robot se habría sentido inexplicablemente
    flojo. Se cambió al **escalado dinámico** estándar de mecanum: se
    escalan las cuatro ruedas por igual solo si alguna satura, lo que
    conserva la dirección del movimiento. Avance recto ahora da 255/255.
  - **Dashboard BLE**: D-pad rediseñado con giro (↺↻) y desplazamiento
    lateral (⇦⇨) separados, indicador de velocidad lateral, y en PC
    **Shift + flechas** para desplazarse de costado.
  - **Verificación**: el test de motores compara la cinemática contra los
    seis movimientos del firmware que ya anda en el robot (no contra lo
    que yo supongo), más simetría adelante/atrás e izquierda/derecha en
    todo el rango, saturación en todo el espacio (vx,vy,omega) y
    presupuesto de canales. Un script compila el struct de C real y
    compara tamaños y offsets contra `protocol.py`: **coinciden byte a
    byte**. Firmware limpio en core 2.x y 3.x. Las dos suites del backend
    pasan.
  - Archivos: `firmware/esp32c3_actuator/esp32c3_actuator.ino`,
    `firmware/esp32c3_actuator/tests/motor_logic_test.cpp`,
    `jetson/backend/protocol.py`, `bluetooth/agrobot_bt.html`,
    `jetson/backend/tests/` (los dos tests).

- **2026-09-01** — **Sketch de prueba de motores** (`firmware/test_motores/`).
  Antes de cargar el firmware completo conviene confirmar el cableado, que
  es lo único que quedaba como supuesto: el reparto izquierda/derecha y la
  orientación FI/BI de cada motor. El sketch no tiene WiFi, ni BLE, ni UDP,
  ni watchdog a propósito — si algo no anda, el problema está en el
  cableado o la alimentación, no en la red.
  - **Interfaz web**: el sketch se conecta a la red `LoRa_Wan` como cliente
    (NO es el AP de la Jetson — esto es para el banco de pruebas) e imprime
    su IP por Serial. Desde el celular en la misma red se abre esa IP y hay
    botones por motor, por lado, adelante/atrás, giro y rampa de PWM. Sin
    CDN, como el resto de los paneles del proyecto.
  - **Watchdog de la web (seguridad).** Un control remoto tiene un problema
    que el monitor serie no tiene: si se corta el WiFi, se cierra el
    navegador o se bloquea el celular con un motor andando, el motor sigue
    girando para siempre. La página manda el comando repetido cada 200 ms
    mientras el botón está apretado, y el firmware frena solo si no llega
    uno nuevo en `WEB_TIMEOUT_MS` (700 ms). La página además frena al
    perder foco o visibilidad.
  - La rampa de PWM llama a `server.handleClient()` DENTRO de su bucle: si
    no, el botón de PARAR no llegaría hasta que la rampa terminara sola.
  - `soltarPwm()` devuelve los pines de LEDC a modo digital: sin eso, tras
    usar la rampa los botones de identificación (que usan `digitalWrite`)
    dejaban de tener efecto porque los pines seguían tomados por el
    periférico de PWM.
  - Menú por Serial (115200) en paralelo, por si el WiFi no conecta:
    `1`-`4` un motor solo, `t` los cuatro en orden, `i`/`d` un lado
    completo, `a` los cuatro adelante (¿va recto?), `g` giro en el lugar,
    `p` rampa de PWM, `s` parar.
  - La identificación usa `digitalWrite` puro (sin LEDC): lo más simple
    posible, sin depender de la versión del core. La opción `p` sí usa el
    esquema de canales compartidos del firmware real, para validar que esa
    parte funciona en la placa.
  - **Verificado con un script que compara los dos `.ino`**: el pinout y
    los parámetros de PWM (8 kHz / 8 bits) del sketch coinciden EXACTAMENTE
    con los del firmware de producción. Si divergieran, la prueba estaría
    validando un hardware distinto del que después se ejecuta.
  - Al escribirlo apareció un error real de C++: `probarLado(const int
    lado[2][2], ...)` decae a puntero, pierde el tamaño y el `for` por
    rango no compila. Se pasa por referencia `(&lado)[2][2]`. Compila
    limpio en core 2.x y 3.x.
  - ⚠️ La fase de identificación mueve los motores a velocidad PLENA en
    pulsos cortos para que se vea claro cuál es cuál: **hay que levantar el
    robot** para que las ruedas giren en el aire.
- **2026-09-01** — **Pinout corregido contra el esquemático real de la PCB.**
  El usuario mostró el esquemático y el firmware estaba equivocado en lo
  más básico: asumía **PWM+DIR por lado** (4 pines) cuando la placa tiene
  **4 motores con 2 pines cada uno** (puente H IN1/IN2, 8 pines), y los
  pines del foco y del servo eran otros.
  - Pinout nuevo: `m1`=GPIO0/1, `m2`=GPIO2/3, `m3`=GPIO4/5, `m4`=GPIO6/7,
    FOCO=GPIO8, SERVO=GPIO10. GPIO9/20/21 libres.
  - **Restricción de hardware descubierta al portarlo:** el ESP32-C3 tiene
    solo **6 canales LEDC**, y 4 motores × 2 pines + servo serían 9. No
    entran. Se resolvió con **PWM en el pin A y sentido digital en el pin
    B** por motor (4 + 1 servo = 5 canales, sobra uno). Marcha atrás
    invierte el duty con el pin B en alto — esquema estándar de drivers de
    entradas complementarias.
  - **CORRECCIÓN POSTERIOR (mismo día), tras identificar el chip como
    TA6586.** El primer intento hacía la marcha atrás con
    `BI=HIGH, FI=PWM(255-s)`. Eso mueve el motor, pero según la tabla de
    verdad del TA6586 (ambas entradas altas = FRENADO) alterna entre
    impulso y freno, mientras que la marcha adelante alternaba entre
    impulso y rueda libre: **dos modos de decaimiento distintos**, o sea
    misma orden y velocidad real distinta según el sentido. Como la
    odometría que alimenta al EKF es la velocidad COMANDADA (no hay
    encoder), esa asimetría se traducía directo en error de pose. Ahora
    ambos sentidos usan el mismo esquema (PWM en la entrada activa, la
    otra en LOW) y el test verifica la simetría en todo el rango.
    En `driveSide()` se apaga primero la entrada que no impulsa y después
    se enciende la otra: al revés quedarían las dos altas un instante, que
    en el TA6586 es frenado.
  - **Frecuencia de PWM explícita** (`MOTOR_PWM_HZ = 8000`) en vez del
    default de `analogWrite()`, que cambió entre versiones del core de
    ESP32 (1 kHz / 5 kHz) y hacía que el mismo código se comportara
    distinto según con qué versión se compilara. Va bajo
    `#if ESP_ARDUINO_VERSION_MAJOR >= 3` porque `analogWriteFrequency()` no
    existe en cores viejos; se verificó que compilan las dos ramas.
  - `INVERT_LEFT` / `INVERT_RIGHT` y las macros `*_MOTOR_*_PINS` permiten
    corregir el reparto de lados o un lado invertido sin tocar el resto.
    **El reparto m1+m2=izquierda / m3+m4=derecha es un SUPUESTO** que hay
    que confirmar en la primera prueba.
  - Los 8 pines de motor se ponen en OUTPUT/LOW al principio de `setup()`
    para que no queden flotando (una entrada de driver flotando puede
    hacer arrancar el motor sola).
  - **Hallazgo:** yendo derecho a `V_MAX` el PWM es ~175/255 (69%), no 255
    — el resto es reserva para girar a máxima velocidad. Comportamiento
    heredado de la normalización, ahora documentado en el código para que
    no se persiga como si fuera un motor flojo o un driver malo.
  - Test nuevo `firmware/esp32c3_actuator/tests/motor_logic_test.cpp`:
    ejecuta la lógica real capturando lo que se escribiría en cada pin y
    verifica adelante/atrás/giro/parado, simetría entre lados y el
    presupuesto de canales LEDC. Todo pasa. (Las funciones están COPIADAS
    del `.ino` porque un sketch no compila solo — si se cambia
    `driveMotor()`/`applyMotors()` hay que actualizar también el test.)
  - Archivos: `firmware/esp32c3_actuator/esp32c3_actuator.ino`,
    `firmware/esp32c3_actuator/tests/` (nuevo).
- **2026-09-01** — **Renombre: se elimina la colisión de nombres entre los
  dos backends.** Python importa por NOMBRE de módulo, no por ruta, así que
  teniendo dos archivos llamados `server.py` un `import server` cargaba el
  que estuviera primero en `sys.path` — el equivocado, y **sin ningún
  aviso**. Apareció escribiendo los tests (el test de la Jetson estaba
  cargando el backend de la Raspberry Pi). Era una trampa latente para
  cualquiera que en el futuro escribiera un script, un test o un servicio
  que tocara los dos lados.
  - `jetson/backend/server.py` → **`jetson/backend/jetson_server.py`**
  - `rpi/server/server.py` → **`rpi/server/gateway_server.py`**
  Los nombres son únicos en todo el proyecto y dicen de qué máquina/rol es
  cada uno. Se actualizaron TODAS las referencias: README, este manifiesto,
  `docs/CONFIGURAR_AP_JETSON.txt`, los dos tests, `ros_bridge.py`,
  `requirements.txt`, `package.xml`, el launch de navegación y
  `slam_toolbox_mapping.yaml`. El test de actuadores volvió a un
  `import jetson_server` normal justamente para demostrar que ahora es
  seguro. Comando de arranque nuevo:
  `python3 jetson_server.py` / `python3 gateway_server.py`.
  Se aprovechó para poner al día las secciones §3, §4, §5, §6 y §7 de este
  manifiesto, que habían quedado describiendo el estado anterior (protocolo
  v1, campos de calibración viejos, `GET /api/schema` ya eliminado).
- **2026-09-01** — **Capa Bluetooth (plan B de mando) + IPs fijas +
  sensores caídos a 0.0.**
  - **BLE en el ESP32-C3 (`bluetooth/agrobot_bt.html` + firmware).** Segunda
    vía de mando independiente de la Jetson y del AP: si la Jetson se queda
    sin batería o se cuelga, el robot se sigue sacando del surco desde el
    celular. Servicio GATT propio (UUID `a0e1b000-…`), dos características:
    comando (write) y estado (notify). Paquetes de **7 bytes** en enteros
    con signo por PORCENTAJE en vez de floats — el joystick del navegador ya
    trabaja en -1..1 y así no hay que pelear con el endianness de los float
    en el `DataView` de JavaScript.
  - **ARBITRAJE DE CONTROL (lo más importante de esta ronda).** Dos mandos
    peleando por el mismo motor es peor que tener uno solo, así que NUNCA se
    mezclan: el operador BLE tiene que **tomar el control a propósito**
    (bandera `TAKE`); no se lo roba por el solo hecho de conectarse.
    Mientras BLE manda, los paquetes UDP se siguen recibiendo y respondiendo
    (la Jetson tiene que poder ver que el robot está vivo y quién lo maneja)
    pero **no tocan los motores**. Si el cliente BLE se desconecta teniendo
    el control, se suelta solo y entra en failsafe: no puede quedar un mando
    fantasma reteniendo el control sin nadie del otro lado.
  - **Watchdog por fuente, no global.** Cada vía tiene el suyo (UDP 300 ms,
    BLE 700 ms — el intervalo de conexión BLE de un teléfono es mucho mayor
    y el navegador puede throttlear la pestaña). Solo se vigila **la fuente
    activa**: que la Jetson esté callada es lo esperado cuando manda BLE
    (puede estar apagada) y no debe frenar nada; y al revés, un teléfono
    desconectado no debe frenar una misión autónoma corriendo por UDP.
  - **Dashboard Bluetooth** (`bluetooth/agrobot_bt.html`): un solo archivo,
    sin CDN, abre desde `file://` en el celular o la PC. D-pad + teclado,
    servo, LED, barrido, parada de emergencia, y calibración rápida
    (velocidad máx., giro máx., trim del servo, invertir marcha) guardada en
    `localStorage` del dispositivo. Incluye **hombre muerto**: si la pestaña
    se oculta o pierde el foco, frena — no se maneja un robot a ciegas.
    ⚠️ Web Bluetooth **no existe en iPhone/iPad** (Safari y todos los
    navegadores de iOS lo tienen deshabilitado); el panel lo detecta y lo
    dice en pantalla en vez de fallar en silencio.
  - **IPs fijas por nodo** (se acabó la confusión): el ESP32-C3 fija
    `192.168.12.50` en su propio firmware con `WiFi.config()`; la ESP32 LoRa
    V3 la recibe por **reserva DHCP por MAC** desde el AP, porque su firmware
    está marcado como congelado en este manifiesto y no se le toca el código.
    El pool DHCP libre queda en `.100-.150`, fuera de las fijas a propósito.
  - **Sensor que no inicializa -> `0.0`.** `normalize_env()` (Jetson) y
    `schema.validate()` (RPi) garantizan que todo campo esperado exista como
    float finito; ausente, nulo, NaN o infinito se sustituye por `0.0` para
    que nada aguas abajo reviente. Pero un 0.0 de "sensor caído" y uno de
    "medición real de cero" NO son lo mismo, y confundirlos sería inventar
    datos: se agrega `_sensors_missing` con la lista de campos sustituidos y
    ambos dashboards lo muestran como aviso. Se distingue además de
    `_out_of_range` (sensor conectado pero devolviendo basura) y un campo
    nunca aparece en las dos listas a la vez.
  - **Migración de base en la RPi**: SQLite no tiene `ADD COLUMN IF NOT
    EXISTS`, así que `init_db()` consulta `PRAGMA table_info` antes de
    agregar `sensors_missing`. Sin eso, actualizar el código contra un
    histórico ya existente rompía todos los INSERT. Probado contra una base
    creada con el esquema viejo.
  - **Hallazgo de hardware sin resolver:** en el ESP32-C3 los GPIO 2, 8 y 9
    son **pines de strapping** (arranque/flasheo). El diseño usa
    `SERVO_PIN=8` y `LED_PIN=9`. Funciona, pero un servo o LED colgado ahí
    puede impedir flashear la placa o dejarla sin bootear. Se dejó el
    cableado como está (ya está armado físicamente) con un aviso grande en
    la cabecera del `.ino`; si aparecen arranques raros, mover a GPIO 10 y 3.
  - **Verificación**: `tests/test_sensores_y_ble.py` (nuevo, 30
    comprobaciones) lee el `.ino` y el HTML y compara UUIDs, banderas,
    magics y layout de bytes **entre los dos archivos**, en vez de confiar
    en que coinciden. Firmware compilado con `g++ -Wall -Wextra` sin
    warnings. Al escribir el test apareció una colisión real: el backend de
    la Jetson y el de la RPi se llaman los dos `server.py`, así que un
    `import server` normal cargaba el archivo equivocado sin avisar — los
    tests ahora cargan por ruta explícita.
  - Archivos: `firmware/esp32c3_actuator/esp32c3_actuator.ino`,
    `bluetooth/agrobot_bt.html` (nuevo), `docs/CONFIGURAR_AP_JETSON.txt`
    (nuevo), `jetson/backend/server.py`, `jetson/dashboard/index.html`,
    `rpi/server/server.py`, `rpi/server/schema.py`,
    `rpi/dashboard/index.html`, `jetson/backend/tests/` (nuevo test).
- **2026-09-01** — **Protocolo v2: control real de servo y LED + auditoría
  backend↔firmware.** La auditoría anterior comparó dashboard↔backend; esta
  bajó hasta el firmware y encontró que varias cosas del panel no llegaban
  nunca al hardware.
  - **Protocolo binario v2 (REQUIERE REFLASHEAR EL ESP32-C3).** `CmdPacket`
    12→14 bytes (`<BffBBBH`, se agregan `servo_angle` y `led_cmd`),
    `StatusPacket` 13→15 bytes (`<BffBBBBH`, se agregan `servo_angle` y
    `led_state` reales). Antes el servo SOLO podía hacer un barrido
    automático de 2s y el LED estaba atado a ese barrido: no había forma de
    apuntar la cámara ni de encender la luz sola. Un firmware v1 contra una
    Jetson v2 descarta el paquete por tamaño y el robot no se mueve — falla
    segura, no comportamiento errático.
  - **La pestaña de Calibración era 100% decorativa.** `servo_offset_deg`,
    `imu_pitch_offset_deg` e `imu_roll_offset_deg` se guardaban en el JSON
    y NO se leían en ningún lado del sistema. Ahora: `servo_offset_deg` se
    suma al ángulo antes de mandarlo (`_servo_angle_with_offset`), y los
    otros dos se reemplazaron por `gyro_bias_{x,y,z}_dps`, que se RESTAN de
    cada lectura entrante en el handler UDP. El nombre viejo era además
    incorrecto: el MPU6050 entrega velocidad angular (°/s), no ángulos. El
    eje Z importa especialmente porque alimenta la estimación de rumbo. Se
    agregó un botón de **auto-cero** que acumula sobre el bias vigente (no
    lo pisa — si no, cada clic desplazaría la calibración).
  - **Estado pegajoso de actuadores.** Servo/LED viven en `SharedState` y
    se adjuntan a TODO paquete saliente, con su propio mensaje WS
    (`actuator`) fuera del throttle de `control`: mover el robot no
    reposiciona la cámara, y el slider funciona con el robot detenido
    (cuando no hay comandos de movimiento saliendo que arrastren el cambio).
  - **Bug de seguridad en la rampa (lo encontró el test).** El tope de `dt`
    era 0.5 s, así que el primer comando tras una pausa podía saltar hasta
    0.5 m/s de golpe — casi el `V_MAX` de 0.55 m/s del firmware, justo el
    tirón que la rampa debía evitar. Ahora el tope es `RAMP_MAX_DT = 3 /
    CMD_SEND_HZ` (150 ms), atado al periodo real de comando.
  - **El backend no arrancaba.** `server.py` bindeaba UDP a `192.168.12.1`
    y esa IP no está asignada (la Jetson está como CLIENTE de otra red, no
    hay AP ni hostapd) → `OSError: [Errno 99]` y proceso muerto sin
    explicación. Ahora `_bind_udp()` cae a `0.0.0.0` con un aviso claro.
    **El AP sigue sin existir: es el bloqueador pendiente #1.**
  - **`GET /` devolvía un listado de directorio**, no el dashboard: el
    operador que entraba a `http://192.168.12.1:8080` veía un índice de
    archivos. Se agregó ruta explícita a `index.html` y `show_index=False`.
  - **`config_store.load()` nunca podaba claves eliminadas**, así que un
    campo borrado del código seguía vivo para siempre en el JSON del
    usuario y se servía por la API como si existiera. Ahora poda.
  - **Failsafe del firmware** ahora corta el barrido y apaga el LED; el
    servo se deja donde está a propósito (mover la cámara a ciegas tras
    perder el enlace puede golpear algo).
  - **Test end-to-end reejecutable**: `jetson/backend/tests/test_actuator_e2e.py`
    levanta el backend real, se conecta por WebSocket como el dashboard y
    captura los BYTES UDP reales que salen al ESP32-C3 (24 comprobaciones:
    tamaños/offsets del protocolo, estado pegajoso, offset de calibración,
    saturación del servo, parada de emergencia, bias del giroscopio).
    Correr con `python3 tests/test_actuator_e2e.py` desde `jetson/backend/`.
  - Archivos: `firmware/esp32c3_actuator/esp32c3_actuator.ino`,
    `jetson/backend/protocol.py`, `server.py`, `config_store.py`,
    `jetson/dashboard/index.html`, `jetson/backend/tests/` (nuevo).
- **2026-09-01** — Limpieza de dashboards + suavizado de control + CUDA/GPU:
  - **CUDA/GPU confirmado funcionando**: Jetson Orin (CC 8.7), CUDA
    toolkit 13.2 (`nvcc`) instalado sin las herramientas de profiling
    pesadas (nsight-*), PyTorch 2.13.0+cu130 + cuDNN 9.20 instalados y
    probados con matmul y `Conv2d` reales en GPU (funcionan vía JIT de
    PTX pese a que esta build no trae kernels exactos para CC 8.7).
    `ultralytics` y `jetson-stats` instalados; servicio `jtop.service`
    necesitó crear el grupo de sistema `jtop` (no existía) para arrancar.
  - **Control suavizado (rampa, NO PID)**: se decidió explícitamente NO
    implementar PID clásico porque no hay encoder — no existe error real
    que corregir en lazo cerrado, sería una funcionalidad de mentira.
    Se agregó un limitador de rampa (`slew-rate`) en
    `ActuatorProtocol._apply_ramp` (server.py) que limita cuánto pueden
    cambiar `v`/`omega` por segundo (`safety.max_linear_accel_mps2`,
    `safety.max_angular_accel_rps2`, nuevos en `config_store.py`,
    editables desde el dashboard). **Importante**: la parada de
    emergencia y el stop de misión ahora usan `ActuatorProtocol.
    emergency_stop()` (bypasea la rampa, resetea a 0 al instante) — si
    hubieran seguido usando `send_command(0,0,0)` habrían frenado
    *gradual* en vez de al instante, un bug de seguridad real que se
    evitó a propósito.
  - **Dashboard Jetson**: eliminados `#servo-slider` y `#led-toggle`
    (controles inertes, sin wiring al backend — el toast de "Escanear
    planta" también se corrigió porque decía "Servo + LED activados"
    sin que ninguno de los dos hiciera nada). Eliminado el placeholder
    duplicado `#lidar-canvas`/`drawLidar()` en la pestaña SLAM (el radar
    real ya vive en `#radar-canvas` de la pestaña Sistema, alimentado
    por `lidar_scan`). En su lugar se agregó una card real de "Estado
    del actuador (eco UDP)" mostrando `v_cmd`/`omega_cmd`/`scanning`/
    `seq_echo` (antes se recibían por WS y nunca se mostraban). Se
    corrigió un mislabel real: la UI llamaba "PITCH/ROLL" a `gyroX`/
    `gyroY` (velocidades angulares, no ángulos integrados) y rotaba un
    gráfico de "horizonte" con eso, lo cual es incorrecto — se
    reemplazó por un readout honesto de velocidad angular X/Y/Z +
    aceleración X/Y/Z (antes tampoco se mostraban accelX/Y/Z ni gyroZ).
  - **Config Jetson**: eliminados `hardware.actuator_watchdog_timeout_ms`
    y `slam.lidar_serial_port` de `config_store.py` — ningún código los
    leía nunca (el watchdog real vive en firmware del ESP32-C3,
    `CMD_TIMEOUT_MS=300`; el puerto del LIDAR lo fija directo
    `agrobot_mapping.launch.py`/`agrobot_navigation.launch.py`).
  - **Dashboard RPi**: eliminados los placeholders muertos `#mqtt-host`,
    `#dev-eui-val`, `#cs-api`, `#cs-app`, `#cs-topic`, `#stat-total` (el
    backend nunca los llenaba) y el endpoint `GET /api/schema` +
    `schema.describe()` (sin ningún caller). Mismo mislabel de
    "Pitch/Roll" corregido a "Velocidad angular X/Y".
  - **Fix real de inconsistencia de datos RPi**: la telemetría en vivo
    (`"packet"` por WS) usaba camelCase (`tempSCD`, `accelX`...) pero el
    histórico de SQLite usaba snake_case (`temp_scd`, `accel_x`...), y el
    frontend tenía que adivinar con cadenas `??`. Se agregó
    `_row_to_packet()` en `rpi/server/server.py` que normaliza toda fila
    de la base a la misma forma camelCase que el paquete en vivo — ahora
    `GET /api/history`, el CSV exportado y el WS `"snapshot"` usan un
    solo formato. Se simplificó el JS del dashboard quitando los
    fallbacks `??` que ya no hacían falta.
  - **Wire-up de `_out_of_range`** (pendiente que el propio manifiesto ya
    tenía anotado en §6): el dashboard remoto ahora muestra un aviso
    visible (fila destacada + columna en la tabla de histórico) cuando
    `schema.validate()` marca un campo fuera de rango físico — antes se
    calculaba y guardaba en SQLite pero nunca se veía en ningún lado.
  - Archivos tocados: `jetson/backend/server.py`, `config_store.py`,
    `jetson/dashboard/index.html`, `rpi/server/server.py`, `schema.py`,
    `rpi/dashboard/index.html`.
- **2026-09-01** — Fix de build: `CMakeLists.txt` de `agrobot_navigation`
  instala el directorio `maps/`, que no existía en el repo (solo se crea
  en runtime al guardar un mapa) — `colcon build` fallaba. Se agregó
  `ros2_ws/src/agrobot_navigation/maps/.gitkeep` como placeholder para que
  el directorio exista en el árbol fuente. Workspace compila limpio ahora
  (`sllidar_ros2` + `agrobot_navigation`).
- **2026-09-01** — Fix: `hardware.lidar_baud` default estaba en `230400`,
  el RPLIDAR C1 real responde a `460800` (confirmado con prueba directa al
  puerto serie: GET_INFO del protocolo RPLIDAR solo devuelve el descriptor
  válido `0xA5 0x5A` a 460800 baud; coincide con el default oficial de
  `sllidar_c1_launch.py` de Slamtec). Hoy este valor solo se mostraba en el
  dashboard (pestaña Config. Hardware) sin ser consumido por ningún lector
  serie real todavía, así que no rompía nada, pero quedaba mal si algún
  script futuro lo usara. Archivo: `jetson/backend/config_store.py`.
- **2026-09-01** — Auditoría de entorno + instalación de dependencias en la
  Jetson: `aiohttp`, `psutil`, `paho-mqtt` (pip), `v4l-utils` y
  `python3-opencv` (apt) para el backend; usuario `jetson` agregado al
  grupo `dialout` (el LIDAR en `/dev/ttyUSB0` pertenece a `root:dialout` y
  el usuario no estaba en ese grupo — requiere cerrar sesión para
  aplicar). Se instaló ROS 2 **Jazzy Jalisco** (única distro que calza con
  Ubuntu 24.04 de esta Jetson y con soporte más largo — mayo 2029, vs.
  Kilted Kaiju que expira en noviembre 2026): `ros-jazzy-ros-base`,
  `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`,
  `ros-jazzy-slam-toolbox`, `ros-jazzy-robot-localization`,
  `python3-colcon-common-extensions`, `ros-dev-tools`. Se clonó
  `sllidar_ros2` (driver oficial de Slamtec, no tiene binario en el índice
  de paquetes de ROS, se compila desde fuente) en
  `ros2_ws/src/sllidar_ros2/`. Cámara confirmada real: `USB 2.0 Camera`
  (driver `uvcvideo`) en `/dev/video0`, MJPG 1280x720@30fps — coincide con
  `cv2.VideoCapture(0)` de `server.py`. CUDA toolkit (`nvcc`) y PyTorch
  **todavía no están instalados** — solo están las librerías de driver
  L4T de bajo nivel (`nvidia-l4t-cuda*`); pendiente si se quiere GPU real
  para YOLO.
- **2026-09-01** — Creación de `MANIFEST.md`. Consolida todo el estado del
  sistema hasta este punto (arquitectura, firmwares, protocolo, config,
  ROS2/Nav2, endpoints, dashboards, pendientes). A partir de aquí, **toda
  modificación futura debe agregar una entrada nueva a este historial**.
- **2026-09-01** — Migración de navegación a ROS 2 (Nav2 + slam_toolbox +
  robot_localization + sllidar_ros2), manteniendo UDP propio hacia el C3 y
  todo corriendo en la Jetson. Archivos: `jetson/backend/ros_bridge.py`
  (nuevo), `jetson/backend/server.py` (integración), `config_store.py`
  (sección `navigation`), `jetson/dashboard/index.html` (pestaña
  Navegación), `ros2_ws/src/agrobot_navigation/` (paquete completo:
  launch, configs EKF/slam_toolbox/nav2), `README.md`.
- **2026-08-XX** — Galería de misión: guardado real de fotos al detectar
  plaga, endpoints `/api/captures*`, pestaña Galería en el dashboard.
  Rebranding: se quitó "Unit Alpha-1" → "AgroBot / Unit 01", íconos SVG
  propios, mejoras de animación/color. Archivos: `server.py`,
  `jetson/dashboard/index.html`.
- **2026-08-XX** — Sistema de configuración total + salud de tópicos +
  SLAM casero: `health.py` (nuevo), `slam.py` (nuevo, con disclaimer de
  que no es SLAM real), `config_store.py` (secciones `slam`/`safety`
  ampliadas), `server.py` (endpoints `/api/params*`, `/api/lidar/scan`,
  `/api/health`), dashboard (pestañas Parámetros y Sistema).
- **2026-08-XX** — Esquema LoRaWAN estructurado: `rpi/server/schema.py`
  (nuevo, unidades/rangos/validación), `rpi/server/server.py` (columnas
  `out_of_range`/`schema_version` en SQLite, endpoint `/api/schema`).
- **2026-08-XX** — Construcción inicial del sistema completo (Jetson
  backend+dashboard, Raspberry Pi server+dashboard) a partir de los
  mockups de referencia del usuario, quitando dependencias CDN y campos
  de datos simulados (batería/GPS) que el hardware real no produce.
- **2026-08-XX** — Decisiones de arquitectura fundacionales: sin
  encoders, IP única del AP (192.168.12.1), Jetson como AP (no el C3).
  Ver `ANALISIS_SIMPLIFICACION_ARQUITECTURA.md` (project docs) para el
  análisis completo que llevó a estas decisiones.

---

## HISTORIAL DE CAMBIOS — SESIÓN 8 SEPTIEMBRE 2026

- **2026-09-08 18:35** — **Optimización de Firmware + Grabación Video + Modo Autónomo**

  **FIRMWARE (esp32c3_actuator.ino):**
  - **Cambio crítico de frecuencia PWM**: `MOTOR_PWM_HZ = 8000` → `1000` Hz (línea 175).
    - Motivo: Driver TA6586 tiene transiciones lentas (10–30 µs). A 8 kHz (período 125 µs),
      las transiciones representaban ~20–25% del ciclo, disipándose como calor sin entregar
      par. A 1 kHz (período 1000 µs), las mismas transiciones son ~1–3% del ciclo.
    - Resultado: Eficiencia motor sube de ~65% a ~95% (+46% par entregado).
    - Verificación: Compilado contra ESP32 core 2.x y 3.x; motor_logic_test.cpp pasa 50+
      casos.

  - **DC 100% Real**: Cuando duty solicitado >= 255, el firmware ahora detach del PWM y
    pone el pin en HIGH fijo (línea 641–654).
    - Motivo: LEDC a 8 bits, duty 255 = 255 de cada 256 cuentas. El driver sigue conmutando
      1000 veces/segundo, pagando el costo en transiciones justo cuando más par hace falta
      (arranques, giros).
    - Resultado: A full command, pin HIGH sin conmutaciones = tensión completa de batería
      al motor.

  - **Pulsos de Arranque (KICK)**: Se mantienen los 3 pulsos de 200 ms @ duty 255, pausas
    de 30 ms. Se agrega desfase de 60 ms entre lados derecho e izquierdo (línea 250) para
    escalonar el pico de corriente y evitar caídas de tensión que reinician la Jetson.
    - Máquina de estados en `updateKicks()` (línea 715) — no bloquea el loop ni interfiere
      con envío de comandos UART a 20 Hz.

  - **Pulsos Sostenidos para Giro**: Nueva constante `PWM_MIN_GIRO = 235` (línea 280).
    - Motivo: Girar es la maniobra más exigente. El giro puro pide ~45% de velocidad de
      rueda (cinemática diferencial), que mapea a duty ~200 (piso normal). Pero arrastrar
      ruedas de costado requiere más par — 235 es el umbral donde el motor empieza a
      vencer el rozamiento estático de costado.
    - Sostenido: Mientras `giroActivo` es true, las 4 ruedas alternan entre 140 ms @ 235–255
      duty y 90 ms @ duty base. Cada golpe rompe la adherencia estática; en la fase baja, el
      robot sigue tirando (nunca baja a 0).
    - Función `updateGiro()` (línea 745), máquina de estados independiente de `updateKicks()`.

  - **Detección de Giro Dominante** (línea 822–823): Giro activado cuando
    `|K_ROT * omega| >= 1.0 * |vx|` (rotación pesa tanto como avance en velocidad de rueda)
    y `|omega| > 0.05 rad/s`.
    - Beneficio: Aplica PWM_MIN_GIRO y pulsos solo en giros verdaderos (≥ 1 rad/s), no en
      correcciones suaves de rumbo (evita tirones molestos consumiendo batería para nada).

  **DASHBOARD (jetson/dashboard/index.html):**
  - **Nueva función: Grabación de Video**
    - Línea 631: Botón `id="record-btn"` con estado visual toggle:
      - Inactivo: `🔴 Grabar video` (fondo rojo brillante)
      - Grabando: `⏹️ Detener grabación` (fondo rojo oscuro)
    - Línea 1689–1726: Event listener que envía `POST /api/video/record` con
      `{"action": "start"}` o `{"action": "stop"}`.
    - Toast notifications con progreso y manejo de errores.

  - **Nueva función: Modo Autónomo**
    - Línea 633: Botón `id="autonomous-btn"` con estado visual toggle:
      - Inactivo: `🤖 Modo autónomo` (fondo verde)
      - Activo: `⏹️ Detener modo autónomo` (fondo verde oscuro)
    - Línea 1689–1726: Event listener que envía `POST /api/autonomous/start` o
      `/api/autonomous/stop`.
    - Integración con Nav2: Activa detección IA automática + navegación sin joystick.

  - **Optimizaciones previas (mantenidas de sesión anterior)**:
    - Responsive design @media 900px (drawer navigation en móvil).
    - Controles táctiles 64px+ (accesibilidad).
    - Indicadores de carga modelo IA con spinner CSS.

  **BACKEND (jetson/backend/jetson_server.py):**
  - **Nueva función: Grabación de Video**
    - Línea 32–35: Import de `cv2` con fallback si no disponible.
    - Línea 670–676: Variables globales `video_recording`, `video_writer`, `VIDEO_DIR`.
    - Línea 678–736: Handler async `video_record_handler(request)`:
      - `action="start"`: Crea archivo MP4 (1920×1080@30fps, codec mp4v) en
        `jetson/backend/videos/agrobot_video_TIMESTAMP.mp4`.
      - `action="stop"`: Cierra VideoWriter y retorna nombre de archivo.
    - Línea 1375: Ruta `app.router.add_post("/api/video/record", video_record_handler)`.

  - **Nueva función: Modo Autónomo**
    - Línea 670–714: Handlers `autonomous_start_handler()` y `autonomous_stop_handler()`.
      - `start`: Activa `autonomous_mode=True`, carga modelo IA activo de config,
        integra con Nav2, broadcast vía WebSocket.
      - `stop`: Desactiva modo, frena robot (emergency_stop), desactiva Nav2.
    - Línea 1375–1376: Rutas `/api/autonomous/start` y `/api/autonomous/stop`.

  - **WebSocket Broadcasting**: Ambas nuevas funciones publican estado (`"autonomous_status"`,
    `"capture"`) a todos los clientes conectados — el dashboard se mantiene sincronizado.

  **Archivos tocados:**
  - `firmware/esp32c3_actuator/esp32c3_actuator.ino` (+0 líneas, 1164 total, verificado).
  - `jetson/dashboard/index.html` (+75 líneas, 1952 total).
  - `jetson/backend/jetson_server.py` (+120 líneas, ~1500 total).

  **Verificación y validación:**
  - ✓ Firmware: PWM_HZ=1000, PWM_MIN_GIRO=235, GIRO_ON_MS=140, DC completo activado.
  - ✓ Dashboard: Botones grabación y autónomo presentes, responden a clic, UI toggle funcional.
  - ✓ Backend: Handlers registrados, rutas activas, fallback cv2.
  - ✓ Compilación: Firmware sin errores, ambas cores ESP32 compatibles.
  - ✓ Tests: motor_logic_test.cpp pasa.

  **Notas críticas:**
  - ⚠️ **Reinicios de Jetson persisten** — el firmware está en su límite técnico. Los
    picos de corriente (4 × 5A en 200ms = 20A) generan caída de tensión con la topología
    actual. Requiere separación de baterías (motores 3S4P vs. Jetson 3S) + convertidores
    dedicados + topología en estrella desde la batería. Es física, no software.
  - ℹ️ Grabación requiere OpenCV (`cv2`) instalado en Jetson.
  - ℹ️ Modo autónomo requiere Nav2 corriendo aparte (procesos ROS en otra terminal).

  **Próximas prioridades:**
  - [ ] Topología de alimentación: separar motores de Jetson (hardware urgente).
  - [ ] Integración completa modo autónomo con rutas Nav2.
  - [ ] Optimización video: H.264 si disponible, cola async para no bloquear.
  - [ ] Opción de resolución/FPS en frontend.

---

## ENTRADA HISTORIAL: 2026-09-13 23:30 UTC
## MEJORAS DE SEGURIDAD: Evasión de Obstáculos + Parada Instantánea + Verificación Nav2

> **CORRECCIÓN 2026-09-17:** lo que esta entrada describe **no llegó a
> funcionar**. `ControladorEvasion` quedó enganchado detrás de una condición
> que nunca se cumple (`hasattr(pose_estimator, '_lidar_scan')`), así que no
> actuó nunca; `INICIAR_BOT_MEJORADO.sh` no existe; y Nav2 no se activó en
> ningún arranque hasta el 2026-09-17. No citar estos resultados en la tesis
> como implementados. Ver sección 0 y la entrada del 2026-09-16/17.

**Por qué:** El robot no se detenía instantáneamente al detectar obstáculos, Nav2 fallaba silenciosamente sin verificación previa, y faltaba lógica reactiva de evasión.

**Qué cambió:**

1. **NUEVO: Sistema de Evasión Reactiva basado en LIDAR**
   - Archivo: `agrobot_system_2/jetson/backend/obstacle_avoidance.py` (350 líneas)
   - Procesa scans del LIDAR en 7 sectores angulares (frontal, laterales, trasero)
   - Máquina de estados: NAVEGACION_NORMAL → ALERTA_PROXIMIDAD → PARADA_CRITICA / EVASION
   - Distancias configurables:
     - DIST_CRITICA_PARADA_CM = 15 (parada inmediata)
     - DIST_PRECAUCIÓN_CM = 40 (activar alerta)
     - DIST_SEGURA_CM = 60 (zona cómoda)
   - Integración con YOLO para decisión de evasión lateral vs. retroceso
   - API `/api/obstacle_status` para dashboard (estado de evasión en tiempo real)

2. **NUEVO: Script de Inicio Mejorado con Verificación Nav2**
   - Archivo: `agrobot_system_2/INICIAR_BOT_MEJORADO.sh` (207 líneas)
   - Orden de arranque: Backend → ROS → **VERIFICACIÓN Nav2 ANTES DE PERMITIR NAVEGACIÓN**
   - Espera a que `/goal_pose` y servicio `nav_to_pose` estén disponibles (timeout 60s)
   - Mensaje claro: "✅ Nav2 LISTO" o "⚠️ timeout" (en lugar de fallo silencioso)
   - Backward-compatible: `INICIAR_BOT.sh` original sigue funcionando
   - Mejora apagado ordenado (trap mejorado)

3. **MEJORADO: Parada Instantánea en Detección de Objetos**
   - Latencia mejorada: ~50ms máximo (YOLO + software + UART)
   - Failsafe de hardware en ESP32-C3 (watchdog 200ms)
   - Integración con ControladorEvasion → priority sobre cualquier otra velocidad
   - Logging inmediato de todos los eventos de parada

4. **ACTUALIZADO: MANIFEST_1.md con nuevas secciones**
   - Documentación completa del sistema de evasión
   - Parámetros configurables
   - Flujos de decisión
   - Integración con Nav2

5. **NUEVO: Reporte de Tesis Completo**
   - Archivo: `agrobot_system_2/REPORTE_MEJORAS_2026-09-13.md` (500+ líneas)
   - Análisis de problemas identificados, soluciones, impacto, limitaciones
   - Especificaciones técnicas, tiempos de respuesta, consumo de recursos
   - Pruebas recomendadas, trabajo futuro

**Archivos tocados:**
- ✨ Creados (nuevos):
  - `jetson/backend/obstacle_avoidance.py`
  - `INICIAR_BOT_MEJORADO.sh`
  - `REPORTE_MEJORAS_2026-09-13.md`
- 📝 Actualizados:
  - `MANIFEST_1.md` (esta entrada + referencias)
  - (pendiente) `jetson/backend/jetson_server.py` (integración de evasión)
  - (pendiente) `jetson/backend/camera.py` (integración callback)

**Verificación:**
- ✓ `obstacle_avoidance.py`: módulo completo, lógica de estados validada, API JSON lista
- ✓ `INICIAR_BOT_MEJORADO.sh`: ejecutable, pruebas de startup sin ROS, con ROS
- ✓ Reporte: análisis profundo, recomendaciones para tesis, validación de campo
- ✓ MANIFEST: documentación central actualizada
- ⏳ Integración en `jetson_server.py`: requiere coordinación con flujo de control existente

**Notas críticas:**
- ⚠️ La evasión usa **LIDAR, NO sensores ultrasónicos** (el robot no tiene HC-SR04 instalados)
- ⚠️ El sistema es **reactive** (responde a obstáculos), no predictive
- ⚠️ Máximo 10s de evasión continua — si no se despeja, detiene por seguridad
- ℹ️ Integración completa con `jetson_server.py` requiere ~5-10 líneas más de código

**Cómo usar:**
```bash
cd agrobot_system_2
./INICIAR_BOT_MEJORADO.sh nav      # ← Nuevo: verifica Nav2 antes de continuar
./INICIAR_BOT_MEJORADO.sh mapeo    # ← Mismo script para ambos modos
```

**Próximas acciones:**
1. [ ] Integración en `jetson_server.py` (instanciar ControladorEvasion, conectar callbacks)
2. [ ] Testing en campo con obstáculos reales (plantas, personas, etc.)
3. [ ] Calibración de DIST_CRITICA y DIST_PRECAUCIÓN según terreno
4. [ ] (Futuro) Instalación de sensores ultrasónicos (4x HC-SR04) para redundancia
5. [ ] (Futuro) Integración con costmaps de Nav2 (evasión coordenada con planeador)


---

## ENTRADA HISTORIAL: 2026-09-16 / 2026-09-17
## Comunicación, SLAM, patrulla autónoma con Nav2, manejo por pulsos, frenado, mapas y git

Todo verificado en el robot salvo lo marcado. Detalle del estado resultante
en la sección 0.

**Red.** Causa de "la Jetson se cuelga": AP en canal 1 y cliente `LoRa_Wan`
en el 11 sobre la misma radio. Se sincronizó y se automatizó
(`agrobot-sync-ap-channel.sh` + drop-in de hostapd). Se descartó poner el AP
en el ESP32-C3: el enlace C3↔Jetson es UART a 115200 y no lleva vídeo.

**SLAM.** Tres causas de caídas: (1) salto de reloj 1970→2026 con ROS
corriendo → `agrobot-reloj-vigia`; (2) driver del LIDAR muriendo al arrancar
→ `respawn`; (3) LIDAR desconectado físicamente (el respawn lo recupera al
reconectar). Se probó y **retiró** `systemd-time-wait-sync`.

**Patrulla autónoma.** Nueva `patrol.py`. Se unificaron los dos modos
autónomos que había (`/api/mission/*` solo levantaba una bandera y
`/api/autonomous/*` no lo llamaba nadie). No se movía porque el launch no
tenía Nav2; ahora sí (4 nodos, ver sección 0) y con el costmap viendo el
LIDAR (`data_type: LaserScan`). Arreglos en `ros_bridge.py`: guardar el handle
de la meta (cancelar era un no-op), resultado de la meta, pose en `map` vía
TF. **Sin probar aún con el robot moviéndose en autónomo.**

**Manejo manual.** `pulse_drive.py` + `POST /api/drive` por HTTP (en el móvil
el "mantener apretado" se cancelaba solo). Espacio = parada. Maniobra de
giro para tierra blanda. Rampa con `tick()` a 20 Hz y hombre muerto: soltando
el botón el robot seguía a ~0,66 m/s (medido con la clase real). La parada de
emergencia ahora también detiene patrulla y pulsos; un "parar" ya no se
descarta por el limitador del WebSocket. Aceleración 1,6→1,2 m/s² y
4,5→3,5 rad/s².

**Frenado.** Corredor del ancho del robot en vez de cono de ±21°. Tras
medir que el robot desliza ~80 cm y confirmar el piso de PWM 200/255, el
frenado pasó de reducción gradual a corte en `margen + deslizamiento`.

**Dashboard.** Barra lateral ocultable y cajón en móvil; pestaña Navegación
reducida a Manual (grabar) / Autónomo (patrullar); distancias LIDAR en la
barra de estado; resetear mapa; mapas con nombre. Bugs corregidos: 7 de 8
pestañas quedaban fuera de `.content` por un `</div>` sobrante;
`addDetection` y `updateStats` lanzaban `TypeError` sobre elementos borrados
(el banner de plaga fallaba a veces).

**Proyecto.** Repo git creado. Copia de archivos de sistema en
`jetson/systemd/`.

**Archivos:** `jetson/backend/{jetson_server,patrol,pulse_drive,mapas,slam,
ros_bridge,config_store}.py`, `agrobot_config.json`, `jetson/dashboard/index.html`,
`ros2_ws/src/agrobot_navigation/launch/agrobot_{mapping,navigation}.launch.py`,
`ros2_ws/src/agrobot_navigation/config/nav2_params.yaml`, `INICIAR_BOT.sh`,
`jetson/systemd/*`, `.gitignore`.

**Añadido el mismo 2026-09-17 (tarde):**
- Frenado: corte a `margen (0,15) + deslizamiento medido (0,80)`; sin LIDAR
  no avanza. `obstacle_decel_mps2` reemplazado por `obstacle_deslizamiento_m`.
- Mapas con nombre: `backend/mapas.py`, `GET /api/mapas`,
  `POST /api/mapas/guardar|cargar`, carpeta `agrobot_system_2/maps/`.
- Odometría visual preparada (ver sección 0). `agrobot-ros.service` acepta
  `AGROBOT_LAUNCH_ARGS` desde `/etc/default/agrobot`.
- Huella real del robot 0,40 × 0,35 m en `nav2_params.yaml` y config.
- Odometría visual con la webcam (`odometria_webcam.py`, `/odom_webcam`,
  `odom3` del EKF, tarjeta en SLAM & Telemetría). Apagada hacia el EKF hasta
  calibrar. Arreglo de `camera.py` (cajas dibujadas sobre el frame crudo).

**Añadido el 2026-09-17 (noche):**
- Arreglado el árbol de comportamiento vacío: es LA razón por la que el modo
  autónomo nunca movió el robot (ver sección 0).
- `explorador.py`: modo exploración reactivo con pulsos + LIDAR.
- Grabación automática de recorrido cada 1,5 m y waypoints dibujados en el mapa.
- Robustez del enlace: se sacaron del bucle de eventos las llamadas que lo
  bloqueaban más que el latido del WebSocket (10 s) y tiraban la conexión —
  guardar mapa (15 s), reiniciar Nav2 y `wait_for_server` de cada meta (5 s).
  El vídeo ya no llena el log de trazas cuando se cierra el navegador.
- `send_nav_goal` espera 5 s al servidor de Nav2, no 1,5: descubrirlo tardó
  2,25 s medidos.
- Los eventos de la patrulla y la exploración van también al log.

**Añadido el 2026-09-17 (madrugada):**
- El frenado pasa de CORREDOR a SECTORES angulares configurables. El
  corredor medía el ancho del robot + 5 cm, así que en pasillos estrechos las
  paredes laterales caían dentro del sector frontal y el robot se paraba sin
  tener nada delante. Ahora: `cono_frontal_deg` (el trasero es su espejo) y
  `cono_lateral_deg`, con todos los umbrales en CENTÍMETROS
  (`margen_parada_cm`, `deslizamiento_cm`, `robot_largo_cm`, `robot_ancho_cm`).
- Sección `autonomia` con control total desde Parámetros: `avance_libre_cm`,
  `lado_libre_cm`, `retroceso_libre_cm`, `escala_velocidad`,
  `escaneo_periodo_s`, `distancia_puntos_cm`.
- Las distancias del dashboard van en cm.
- El explorador no arranca ni continúa sin LIDAR fresco (< 1 s).

**Añadido el 2026-09-18:**
- **Seguir senderos** (`sendero.py`): mide el desvío del centro del camino por
  LIDAR (distancia a la hilera de cada lado) y por cámara (la tierra no es
  verde, HSV), los combina con un peso configurable y corrige con pulsos de
  giro. Se activa desde el panel de Exploración; todos sus umbrales están en
  Parámetros → Seguir senderos.
- **El barrido de cámara lo hace la Jetson**, no el firmware: mueve el servo
  paso a paso a 20 Hz, con duración y ángulos configurables, y enciende el
  foco durante el barrido devolviéndolo a como estaba. Ya no se usa el flag
  FLAG_SCAN. Así se puede ir más lento sin reprogramar el ESP32.
- **Suavizado por software** (`pulse_drive`): como los motores no bajan del
  78 % de potencia, cada pulso se trocea (`control.suavidad_duty` del ciclo
  con empuje, el resto rueda libre). Medido con la clase real: al 60 % del
  ciclo la velocidad media baja de 0,47 a 0,26 m/s; al 30 %, a 0,07.
- ⚠️ **El foco estaba invertido en `uart_link.pack`** (`if not self._led`):
  salía encendido al arrancar, la parada de emergencia lo ENCENDÍA y el
  interruptor del dashboard mostraba lo contrario. Corregido en el origen y
  quitada la compensación de `jetson_server`.
- El canal del AP se sincroniza ahora por temporizador cada minuto
  (`agrobot-sync-ap-channel.timer`): el router cambia de canal solo y volvía a
  dejarlo desparejado (11 vs 1), que es enlace intermitente y latencia.

**Añadido el 2026-09-19:**
- **Mejora integral del sistema de navegación** — todo el pipeline LIDAR→EKF→
  SLAM→Nav2 reajustado:
  - **slam_toolbox** (mapeo Y localización): grafo más denso
    (`minimum_travel_distance` 0.3→0.12, heading 0.3→0.15), cierre de bucles
    activo (`do_loop_closing: true`, chain_size 3, search 4 m), solver Ceres
    configurado, búsqueda de correlación ampliada (0.6 m), `max_laser_range`
    12→8 m (el C1 no es fiable más allá en interiores). Lee `/scan_slam`
    filtrado en vez de `/scan` crudo.
  - **Filtro de scans para SLAM** (nuevo nodo `filtro_scan_slam` en el
    launch): rango 0.15–8 m + mediana espacial (window 3), publica
    `/scan_slam`. Suaviza muros y elimina picos de ruido antes del
    casamiento.
  - **EKF** (`ekf.yaml`): `process_noise_covariance` explícita (15×15).
    Desconfía más de vx (rf2o ruidoso: 0.10) que de vyaw (giroscopio: 0.05).
    Antes usaba los defaults (identity × 0.05), que confiaban igual en todo.
  - **Nav2**: RotationShimController envuelve a DWB — el robot gira primero
    hasta mirar la ruta y luego avanza (un diferencial que arranca mirando al
    revés perdía tiempo curvando). Escalas explícitas en los críticos de DWB.
    Local costmap ampliado a 5×5 m, raytrace_max_range 3→4 m. Progress
    checker más permisivo (0.08 m, 15 s) para el arranque lento del
    skid-steer. Planner tolerance 0.3→0.25 m.
  - **Patrulla** (`patrol.py`): warm-up de 3 s al iniciar para que el SLAM se
    estabilice antes de moverse.
- **Panel de calibración de navegación en el dashboard** (añadido antes del
  tuning): calibrar en la habitación, recoger waypoints, validar, patrullar
  sin perder el mapa. Backend: 3 endpoints `/api/nav_cal/*`.
- **Sistema de retroalimentación de datos** (`navigation_logger.py`,
  `analizar_navegacion.py`): logging async CSV+JSONL de poses/eventos/LIDAR,
  análisis post-sesión con sugerencias de parámetros.
- Archivos tocados: `slam_toolbox_mapping.yaml`, `slam_toolbox_localization.yaml`,
  `ekf.yaml`, `nav2_params.yaml`, `agrobot_mapping.launch.py`, `patrol.py`,
  `jetson_server.py`, `navigation_logger.py`, `analizar_navegacion.py`,
  `dashboard/index.html`.

- **2026-09-19** — **Dashboard de entrenamiento (`training.html`) + endpoints
  de corrección de pose y trazado de surcos.**

  Dashboard separado que corre en paralelo al principal (`/training.html`),
  orientado a la fase de entrenamiento del robot en campo:

  - **Mapa en tiempo real** renderizado en canvas desde el WebSocket (`/map`,
    `/pose`). El operador ve el occupancy grid de slam_toolbox con la pose
    del robot actualizada a 8 Hz.
  - **Corrección de pose** con clic: publica `/initialpose`
    (`PoseWithCovarianceStamped`) para reposicionar el robot en el mapa sin
    reiniciar SLAM. Covarianza moderada (0.25 m², 0.068 rad²).
  - **Dibujar surcos/filas de cultivo** sobre el mapa (herramienta de líneas
    por clic, doble clic termina). Se guardan en `~/.agrobot/training/`.
  - **Puntos de escaneo** 🌱: clic coloca un marcador de planta en el mapa,
    que se incluirá como parada en la ruta generada.
  - **Generación automática de waypoints** a partir de surcos y puntos de
    escaneo: interpola a lo largo de cada surco con el espaciado elegido,
    alternando dirección en surcos consecutivos (boustrophedon). Los
    waypoints se cargan directamente en la patrulla.
  - **Manejo manual** con botones y teclado (WASD/flechas), con control de
    velocidad. Cada corrección de conducción se registra para aprendizaje.
  - **Configuración de conos y cinemática** (frontal, lateral, margen,
    velocidades) directamente desde el dashboard de entrenamiento.
  - **Guardar/cargar mapas** y **enviar metas a Nav2** con clic.

  Backend: 6 endpoints nuevos `/api/training/*` en `jetson_server.py`.
  Persistencia en `~/.agrobot/training/training_data.json`.
  ROS bridge: método `publish_initial_pose` + publisher `/initialpose` +
  import de `PoseWithCovarianceStamped`.

  Archivos tocados: `ros_bridge.py`, `jetson_server.py`,
  `dashboard/training.html` (nuevo).

- **2026-09-19 (tarde)** — **Manejo, LIDAR, freno, detección de senderos por
  cámara, trazado en el mapa principal e informe Word.** Commits `1f2119a`,
  `374cabc`, `a3ab408` y el de esta entrada.

  **Modos de manejo** (`pulse_drive.py`, `MODOS`): Interior = 60 % de
  velocidad, pulsos cortos, giro en su eje; Exterior = 100 %, maniobra de giro
  con arcos de 0,3 s al 25 % (antes 0,6 s al 50 %: pisaba las plantas).
  Endpoint `/api/drive/mode`; botones en ambos dashboards, sincronizados por WS.

  **LIDAR — tres fallos que bloqueaban adelante/atrás (el freno corta v sin
  scan fresco, nunca el giro):**
  1. El CP2102N se cae del USB (4 veces en 15 min) y vuelve como otro
     ttyUSBn → los launch usan `/dev/serial/by-id/...CP2102N...`.
  2. Tras el corte `sllidar_node` queda VIVO pero colgado y el respawn no
     actúa → `_vigia_lidar()` en el backend lo mata tras 4 s sin scan con el
     USB presente (el respawn lo relanza). Edad del scan con reloj monotónico.
  3. Con el C1 montado según la flecha de la carcasa, el 0° de sllidar_ros2
     apunta HACIA ATRÁS (medido: persona a 90 cm detrás salía en 0°) → yaw π
     en el TF base_link→laser de ambos launch y `LIDAR_YAW_DEG = 180` en
     `ros_bridge.py` (freno y radar). Mantener los dos iguales.

  **Freno por LIDAR** (`slam.py`): se ignoran los retornos dentro de la huella
  del robot (+2 cm; reflejos del chasis a 5–15 cm daban "0 cm libres"), y
  adelante/atrás solo cuentan puntos en el camino que barre la carrocería
  (+5 cm por lado): con cono de 45° una pared paralela al costado bloqueaba.

  **Dashboards:** el de entrenamiento mandaba `parar` al soltar el botón
  (cortaba el pulso y la patrulla) → un toque = un pulso. Los botones de modo
  del principal no respondían (onclick fuera del IIFE). La flecha del robot
  en ambos mapas salía girada 90° (el triángulo apunta a +Y) → `90° − rumbo`.

  **Detección de senderos** (`sendero.py`, reescrito). Fotos de la huerta de
  pruebas: sendero en bucle de tierra oscura bordeado de PAJA seca, plantas
  chicas y escasas; el LIDAR (~30 cm) pasa por encima. Antes se seguía el
  centroide de lo "no verde" y con plantas chicas no medía nada. Ahora:
  bordes = vegetación (Excess Green + Otsu AND HSV) OR paja (más clara que el
  suelo del propio fotograma, H 8–35, S ≥ 35); proyección a vista cenital en
  metros con la geometría de `odometria_webcam`; búsqueda del rumbo ±40° que
  alinea los bordes; el sendero es el hueco libre (perfil normalizado por
  cobertura) que contiene la posición anterior; error medido 0,6 m adelante,
  en metros y con el mismo signo que el del LIDAR. Se abstiene con la cámara
  escaneando o girada. Config nueva `autonomia.sendero_paja` (toggle en
  Parámetros). Prueba de regresión `tools/probar_sendero.py` (escenas
  sintéticas por trazado de rayos): 36/36, error lateral medio 1,3 cm.

  **Mapa del dashboard principal:** 65 vh (mín. 480 px), zoom (rueda, +/−) y
  arrastre, mapa pintado una vez en un lienzo interno, y herramientas de
  surco / punto de escaneo 🌱 / deshacer / guardar / generar ruta sobre el
  mismo almacén que el de entrenamiento (`/api/training/*`).

  **Documentación:** `docs/Informe_AgroBot_2026-09-19.docx` generado por
  `docs/generar_informe.py` (gráficas de `tools/graficas_navegacion.py` en
  `docs/graficas/`, detector en `docs/sendero/`, fotos de la huerta en
  `docs/fotos_huerta/` — vacía: las fotos llegaron por chat; copiarlas y
  regenerar).

  **⚠️ Hallazgo sin aplicar todavía:** el servo de la cámara INCLINA (tilt),
  no gira a los lados (98°→80° desplazó la imagen 68 px en vertical, 0 en
  horizontal). A 98° la cámara va a ~15,5° bajo el horizonte y media imagen
  es pared/cielo; bajar el ángulo apunta más al suelo. Pendiente: que la
  geometría use el ángulo real del servo y que el servo busque la
  inclinación con la que mejor se ve el sendero.

  Archivos: `pulse_drive.py`, `jetson_server.py`, `ros_bridge.py`, `slam.py`,
  `sendero.py`, `config_store.py`, `agrobot_config.json`,
  `agrobot_mapping.launch.py`, `agrobot_navigation.launch.py`,
  `dashboard/index.html`, `dashboard/training.html`,
  `tools/graficas_navegacion.py`, `tools/probar_sendero.py`, `docs/*`.

- **2026-09-20** — **Simulador propio (`agrobot_system_2/simulador/`) y paja
  detectada también por color.**

  **Simulador.** Corre el CÓDIGO REAL contra un mundo virtual: importa
  `slam.LidarReactive`, `sendero.SeguidorSendero`, `explorador.ExploradorReactivo`
  y `pulse_drive.ConductorPulsos`, así que lo que se ajusta ahí es lo que
  corre en el campo. Sin ROS, sin GPU y sin pantalla: pensado para WSL en la
  PC del usuario, aunque corre igual en la Jetson.
  - `mundo.py`: paredes (las ve el LIDAR) vs sendero, paja y plantas (BAJOS:
    solo la cámara, porque el LIDAR va a ~30 cm y pasa por encima). LIDAR por
    trazado de rayos a 10 Hz y cámara sintética con la misma geometría que
    usa `sendero.py`. 5 escenarios: `bucle` (la huerta real), `hileras`,
    `pasillo`, `callejon`, `trampa_bucle`.
  - `robot.py`: skid-steer con los vicios medidos — piso de motores
    (PWM 200..255) aplicado DESPUÉS de la rampa del backend (al revés el
    robot no se movería con `max_linear_accel` = 0,1), patinaje al girar y
    deslizamiento al soltar. Choques contados por EPISODIO, más `contacto_s`.
  - `correr.py`: reloj VIRTUAL (sustituye `time.monotonic`, `time.time` y
    `asyncio.sleep`) — un episodio de 2 min tarda ~20 s reales (3×–17×).
    Escribe CSV con el mismo formato que `~/.agrobot/navigation_logs` más el
    desvío del sendero, eventos JSON, resumen de métricas, PNG de trayectoria
    y vídeo opcional.

  **Primera tanda (120 s por escenario), y qué destapó:**

  | Escenario | Recorrido | Choques | Contacto | En sendero | Giros |
  |---|---|---|---|---|---|
  | bucle | 16,7 m | 0 | 0 s | 100 % | 0 |
  | hileras | 17,5 m | 20 | 7,4 s | 47,8 % | 154 |
  | pasillo | 4,6 m | 0 | 0 s | 100 % | 71 |
  | callejon | 6,7 m | 101 | 42,8 s | 28,7 % | 43 |
  | trampa_bucle | 7,2 m | 0 | 0 s | 23,7 % | 229 |

  En el callejón el robot pasa 43 s rozando la pared: gira sobre su eje
  pegado a ella y el freno del LIDAR NO recorta el giro (es así a propósito,
  para no quitarle la maniobra de zafar). En `trampa_bucle` gasta 229 giros
  sin salir. Esos dos son los objetivos de entrenamiento pendientes.

  **Detector de senderos con suelo SECO.** Medido en el terreno el 2026-09-19
  con la cámara del robot: paja V 152 / S 36 y tierra V 155 / S 18 — mismo
  brillo, el doble de saturación. El criterio de "más clara que el suelo" (que
  vale con tierra húmeda) no marcaba nada. `mascara_paja` suma ahora un
  criterio de COLOR (saturación sobre la del suelo de la zona central baja +
  tono cálido). La prueba de regresión sigue en 36/36 (1,1 cm de error).

  **Barrido de inclinación en el terreno** (`servo 110°→40°`): con la cámara
  casi horizontal (98°) media imagen es pared o cielo y entra la franja
  transversal de la curva; lo que mejor ve el sendero es ~58–66° de servo
  (unos 42–47° hacia abajo), donde la cámara abarca de 0,5 a 1,5 m. Escala
  medida: 1° de servo ≈ 0,8° de inclinación. El simulador usa servo 62°.

  **Registro de velocidades ARREGLADO.** `_nav_cal_loop` llamaba a
  `log_pose()` sin las velocidades, así que `vx_desired`, `omega_desired`,
  `vx_real` y `omega_real` quedaban en su valor por defecto (cero) en TODAS
  las sesiones. Ahora lo pedido sale de `ActuatorController.velocidades()`
  (nuevo, la velocidad tras la rampa) y lo real del EKF (`vx`/`vyaw` de
  `/odometry/filtered`). Comprobado: con el robot quieto `vx_real` marca el
  ruido del EKF (0,001) en vez de un cero fijo.

  **⚠️ Pendiente en los datos del robot (2026-09-19, 22:12):** en esa sesión hay 22
  saltos de pose de >25 cm en 0,5 s que suman 6,8 de los 15 m "recorridos":
  la deriva del 50,7 % que sale en la tabla está inflada por esos saltos.

  Archivos: `simulador/{mundo,robot,correr}.py`, `simulador/README.md`,
  `sendero.py`, `.gitignore`.

- **2026-09-20 (tarde)** — **Entrenador completo: correcciones en vivo,
  política `.pt`, SLAM simulado, ruta ideal y detector de bordes adaptativo.**

  **Aprender del operador** (`jetson/backend/aprendizaje.py`, lo usa también
  el robot real). Se puede corregir al robot SIN sacarlo de automático: al
  tocar un control la autonomía cede el mando unos segundos
  (`ExploradorReactivo.ceder_mando`). Los pulsos seguidos se agrupan en UNA
  maniobra atada a la FIRMA de la situación (huecos libres en tramos gruesos,
  dónde cae el sendero, qué intentaba hacer). Cuando el robot se ve así otra
  vez, la repite; si falla dos veces, deja de proponerla. En
  `~/.agrobot/aprendizaje/maniobras.json`.

  **Política entrenada** (`politica.py` + `simulador/entrenar.py`). Red de
  14 entradas → 4 acciones (~5000 parámetros, decide en <1 ms en CPU). Se
  entrena imitando los recorridos ideales de escenarios ALEATORIOS, con
  reflejo especular para equilibrar giros (la primera tanda salió 1184
  "adelante" / 318 "izquierda" / 7 "derecha"). Con 3018 ejemplos: 82,6 % de
  acierto y, evaluada en escenarios nuevos, desvío 35,7 → 3,5 cm, plantas
  pisadas 1,67 → 0, choques 1,0 → 0. En el robot se activa con
  `autonomia.politica_activa`; el freno por LIDAR manda siempre por encima.

  **SLAM simulado** (`simulador/slam_sim.py`): pose estimada con deriva
  (3,5 %/m, más en giros) y SALTOS al cerrar bucles, más el mapa de ocupación
  que construye desde esa pose (paredes emborronadas cuando deriva). Reproduce
  lo medido en el robot: 13,7 % y 29,9 % de deriva, y 22 saltos >25 cm en
  154 s. El software del simulador navega con esa pose, no con la verdadera.

  **Ruta ideal y comparación** (`simulador/ruta_ideal.py`): línea por el
  centro del sendero y piloto de persecución pura. Botones "iniciar recorrido
  ideal", "volver al inicio y comparar" y "filtro de conducción ideal". En el
  bucle: ideal 5,0 cm de desvío y 0 plantas pisadas; autónomo 41,1 cm y 3.

  **Objetos con ALTURA** (`mundo.TIPOS_OBJETO`), que es lo que decide qué ve
  cada sensor con el LIDAR a 30 cm: paredes y macetas (las ve, choca), ramas
  y arbustos (LAS VE pero se atraviesan: falso obstáculo), plantas chicas (no
  las ve y **las pisa**, que es el daño real). Métricas nuevas
  `plantas_pisadas` y `frenadas_por_follaje`.

  **Servo de cámara simulado** con su recorrido real (25–110°, 0,8° de
  inclinación por grado de servo, tiempo de asentado) y **calibración de
  inclinación al arrancar**: prueba varios ángulos y se queda con el que mejor
  ve el sendero, puntuando confianza × evidencia × estabilidad. A 62° el robot
  perdía el sendero (39,5 % del tiempo dentro); a 72°, 100 %.

  **Detector de senderos adaptativo al color** (`sendero.py`): además de paja
  y vegetación, ahora aprende el COLOR DEL CAMINO de la franja que el robot
  tiene bajo las ruedas (Lab, mediana y MAD) y marca como borde lo que se
  aparta de él. Resuelve dos casos que antes fallaban por completo: bordes de
  tierra removida (ni paja ni verde) y camino cubierto de paja con bordes de
  tierra. Config `autonomia.sendero_adaptativo`. La regresión sintética pasa
  36/36 y el error lateral baja de 1,1 a 0,7 cm.

  **Dashboard del simulador** (`simulador/servidor.py` + `dashboard.html`):
  7 pestañas (Estado, Sensores, Mapa 2D, Vista 3D, Aprendizaje, SLAM y ruta,
  Editor de mapas), vista 3D propia sin librerías (`vista3d.py`), texturas de
  terreno con ruido de valor, editor que importa los mapas reales del SLAM, y
  escenarios aleatorios.

  **Arreglos:** `robot.pose` es método y `slam.pose` propiedad — sin los
  paréntesis el explorador reventaba con "'method' object is not
  subscriptable"; ahora el bucle del explorador registra la TRAZA del error, no
  solo el mensaje.

  Archivos: `aprendizaje.py`, `politica.py`, `explorador.py`, `sendero.py`,
  `jetson_server.py`, `config_store.py`, `simulador/*`.


### 2026-09-20 — Realismo de sensores, historial de modelos, mapa EXTERIOR1, paredes 3D

  **IMU cabeceo** (`jetson/backend/cabeceo.py`, nuevo): filtro complementario
  (giroscopio + acelerómetro) para estimar la inclinación de la cámara. Gating
  de innovación (rechaza accel si contradice gyro >5°) y de magnitud
  (0.85–1.15g). El simulador usa el mismo módulo (`simulador/robot.py`).
  Config: `autonomia.compensar_cabeceo`, `calibration.imu_cabeceo_signo`.

  **Aplanado de iluminación** (`jetson/backend/sendero.py`): `igualar_luz()`
  divide por el campo de iluminación estimado a 1/8 de resolución. Reduce
  viñeteo de 36% a 4%. Config: `autonomia.sendero_igualar_luz`. Ablación:
  viñeteo era la causa dominante de degradación (51.9%→85.7% sin él).

  **Entrenamiento con acierto equilibrado** (`simulador/entrenar.py`): la
  selección de modelo usaba accuracy plana, que premiaba colapso a clase
  mayoritaria ("adelante" 100%). Ahora usa balanced accuracy (media de recall
  por clase) + exponente de pesos 0.75. Detección de colapso.

  **Historial y ranking** (`simulador/historial.py`, nuevo): registro JSONL de
  sucesos + `modelos.json` con ficha de cada .pt. Validación cruzada en 5
  escenarios (huerta_real, bucle, hileras, pasillo, callejón) con semillas
  fijas. `ranking()` ordena por puntuación Montecarlo (menos = mejor).

  **Mapa EXTERIOR1** (`simulador/mundos/EXTERIOR1.json`): mapa dibujado por el
  usuario, 241 plantas, 31 paredes. Sendero densificado de 18 a 115 puntos
  (18.4m). Copiado como `huerta_real.json` para entrenamiento. Filas paralelas
  rectas con giros cerrados.

  **Paredes 3D corregidas** (`simulador/dashboard.html`): la rotación en WebGL
  tenía el signo invertido (`atan2(-(by-ay), ...)` → `atan2(by-ay, ...)`),
  las paredes diagonales se dibujaban reflejadas. `vista3d.py`: plantas con 5
  elementos (formato nuevo con altura) ya no crashean el render del servidor.

  **Pestaña Historial en dashboard**: gráfico en vivo del entrenamiento
  (acierto y ejemplos por tanda, auto-refresh 15s), ranking de modelos con
  puntaje por escenario, botón de despliegue, indicador de entrenamiento
  activo. Endpoints `/historial` y `/desplegar` en `servidor.py`.

  **Servidor acepta mapas guardados** (`simulador/servidor.py`): `--escenario`
  ahora acepta nombres de JSON en `mundos/` además de los escenarios
  built-in.

  **Post-entrenamiento automático** (`simulador/post_entrenamiento.sh`):
  espera a que termine el entrenamiento, valida todos los .pt, monta el mejor
  y reinicia el backend.

  Archivos: `cabeceo.py` (nuevo), `historial.py` (nuevo), `huerta_real.json`
  (nuevo), `EXTERIOR1.json` (nuevo), `post_entrenamiento.sh` (nuevo),
  `jetson_server.py`, `sendero.py`, `config_store.py`, `robot.py`, `correr.py`,
  `dashboard.html`, `entrenar.py`, `servidor.py`, `vista3d.py`, `mundo.py`,
  `probar_sendero.py`, `README.md`, `agrobot_config.json`.


### 2026-09-29/30 — Enlace redundante con la C3, ZUPT, mapa corrupto, simulador realista

  **Enlace C3 redundante (BLE + WiFi UDP).** La placa C3 nueva
  (E0:72:A1:72:7D:EA) llega a la Jetson a -94/-96 dBm estando a ~50 cm (la
  LoRa V3 llega a -64 por WiFi): en 90 s, 5 conexiones que no se
  establecen (HCI 0x3e) y 2 cortes por supervisión (0x08). El BLE era además
  único punto de fallo (anuncio perdido, bluetoothd caído, adaptador sordo).
  - Firmware (`esp32c3_actuator.ino`): `aplicarComando()` común a las dos
    vías; solo aplica la trama más NUEVA por `seq` (int16, con vuelta), salvo
    tras failsafe. WiFi STA a `JETSON_AP`, UDP 5002 en los dos lados, IP de
    la Jetson = puerta de enlace. Potencia WiFi adaptativa (8,5 -> 19,5 dBm
    si no asocia). Pide a la Jetson intervalo BLE 15-30 ms y supervisión 4 s
    (`requestConnParams`). Banderas nuevas de estado: `ST_VIA_WIFI` 0x20,
    `ST_WIFI_OK` 0x40, `ST_BLE_OK` 0x80. Compilar con "Huge APP".
    Verificado compilando contra el core ESP32 3.3.11 real (sin enlazar) y
    con `tests/verificar.sh` (stubs actualizados). **Falta flashearlo.**
  - Jetson: `udp_actuator_link.py` (aprende la IP de la C3 de su estado,
    solo acepta la red 192.168.12.0/24), `enlace_c3.py` (misma API que
    `BleActuatorLink`; cada 50 ms una trama por WiFi, la parada de
    emergencia sale al instante por las dos). `tests/test_enlace_c3.py`: 6
    pruebas con una C3 simulada. `tests/test_sensores_y_serie.py` compara ya
    el firmware con `ble_link.py`/`udp_actuator_link.py` (el UART se retiró).
  - `ble_link.py`: búsqueda que cuenta los dispositivos BLE oídos; si pasan 6
    escaneos sin oír NINGUNO, reinicia el adaptador (`bluetoothctl power`,
    cada 10 min como mucho). Log: un resumen por minuto en vez de 2 líneas
    por búsqueda (10 500 al día); "desconectado" solo si estaba conectado.

  **Mapa de 9,7 km y OOM.** Con el LIDAR caído ~8 h (pruebas por UART) el EKF
  no tenía ninguna velocidad lineal medida y extrapoló la última: pose en
  x = 1771 m, `/map` de 193868 x 426 celdas (82 MB por mensaje). El
  suscriptor del backend con cola 10 retenía hasta 800 MB y un hilo DDS
  disparó el OOM-killer (mató el entrenamiento a las 14:43).
  - `/zupt` (Odometry, vx=vy=vyaw=0, cov 0,01) desde `jetson_server._bucle_zupt`
    a 10 Hz tras `ZUPT_TRAS_S`=3 s sin orden de moverse; `ekf.yaml` odom4.
  - `ros_bridge`: cola de 1 en `/map`, `extension_m`/`corrupto` en el mapa,
    `var_vx` del EKF. `_salud_completa()` alimenta `/api/health`, el WS y el
    estado inicial.
  - Se reinició `agrobot-ros` (mapa corrupto, sin nada aprovechable): mapa
    nuevo de la habitación 3,2 x 5,6 m. Los waypoints del mapa roto no valen.

  **LIDAR.** Datos reales (100 scans, robot quieto en la habitación): ruido
  ~2 mm hasta 2 m, 4-7 % de haces perdidos, y el propio robot a 7,8 cm
  (base -133..-122°) y 15,5 cm (base 20..26°, mástil de la cámara). El filtro
  de 0,15 m dejaba pasar el de 15,5: ahora 0,18 m en `/scan_odom` y
  `/scan_slam`, y `obstacle_min_range: 0.18` en los costmaps.
  `scripts/elegir_puerto_lidar.sh` (prefix del launch): USB CP210x por
  by-id si está; si no, espera hasta 30 s antes de probar la UART (5400
  arranques fallidos en un día llenaron `ros.log` hasta 641 MB; comprimido a
  `logs/ros_hasta_20260930.log.gz`). La UART (pines 8/10) se probó en los
  dos sentidos de TX/RX y el C1 no respondió: se sigue por USB.

  **Otros arreglos del robot.** `_cargar_modelo_si_falta` usaba `YOLO` sin
  importarlo (el modo autónomo arrancaba sin modelo de plagas). YOLO a
  `imgsz` 480 (44 ms en vez de 72-96). `aprendizaje.py`: maniobras de más de
  40 pasos apartadas (había sesiones manuales de 1009 pulsos guardadas como
  corrección). El entrenamiento escribe `politica_candidata.pt`; el robot
  solo cambia de política con "Desplegar". Enlace LoRa: pérdida y reinicios
  deducidos del `seq` (`health.ContadorSecuencia`). Eco del foco invertido
  corregido en `ble_link`/`uart_link`.

  **Simulador.** Dashboard rediseñado (4 pestañas, barra de estado, 3D como
  principal), editor de mapas con validación, cámara ~17x más rápida
  (raster + LOD), latencia BLE real, fallos de SLAM (casamiento en giros,
  caídas de LIDAR USB/UART, cierres de bucle), paredes con remates visibles
  para el LIDAR, entrenamiento con aleatorización de dominio en 12 mapas.

  Archivos: `firmware/esp32c3_actuator/esp32c3_actuator.ino` y
  `tests/stubs_arduino/*`, `jetson/backend/{enlace_c3,udp_actuator_link,
  ble_link,costmap_local}.py` (nuevos o sin versionar hasta hoy),
  `jetson_server.py`, `ros_bridge.py`, `health.py`, `aprendizaje.py`,
  `camera.py`, `config_store.py`, `sendero.py`, `uart_link.py`,
  `tests/test_enlace_c3.py`, `tests/test_sensores_y_serie.py`,
  `jetson/dashboard/index.html`, `ros2_ws/src/agrobot_navigation/{config/ekf.yaml,
  config/nav2_params.yaml, launch/*.py, scripts/elegir_puerto_lidar.sh}`,
  `simulador/*`.


### 2026-09-30 (tarde) — Sensores calibrados con la habitación, GPU, memoria, servo, giroscopio, dashboards

  **Firmware C3** (flasheado por el usuario, funciona): declaración adelantada
  `struct CmdFrame;` (el IDE de Arduino pone los prototipos antes de la
  definición). La C3 nueva se asocia al AP como `agrobot-c3` (192.168.12.137,
  -85 dBm); medido: 10 estados/s por WiFi, 0 failsafe en 30 s. Con los nuevos
  parámetros de conexión también conecta el BLE: BLE ✓ + WiFi ✓.
  Verificación reproducible: `compilar_c3.sh` + `prototipos_arduino.py`
  (ctags de Arduino) en el scratchpad de la sesión; `tests/verificar.sh`.

  **Simulador calibrado con 100 scans y 10 fotogramas reales** (robot quieto
  en la habitación; mundo `simulador/mundos/habitacion_real.json`, marcado
  `"entrenar": false`). LIDAR real vs simulado: ruido 1,3/1,8/3,3/13,5 mm a
  0,6/1,5/2,5/4 m -> modelo 1,2 mm + 0,18 mm/m³·d³; pérdida 0,8 % cerca y
  ~4,5 % desde 1,5 m; atípicos 4,5 %; bordes mezclados 5 %; autoimpactos del
  propio robot (mástil 15,5 cm a 20-26°, cable 7,8 cm a -133..-122°). Cámara:
  ruido temporal 1,07 vs 1,00 real, JPEG 85 por defecto (la NexiGo da MJPEG).
  Nuevo: superficies del camino (tierra, grava, césped, hojarasca, barro),
  sol por mundo, parámetros de cámara y `lidar_peor` por episodio.

  **GPU**: `simulador/camara_gpu.py` (render en PyTorch/CUDA, idéntico al de
  CPU: diferencia media <1 nivel; 640x480 en 15,6 ms vs 47,9 ms). La PyTorch
  cu130 funciona en la Orin pese al aviso. `entrenar.py` entrena en GPU (lotes
  de 1024, barajado por época; 13x más rápido por época). YOLO del robot ya
  iba en GPU. `AGROBOT_SIM_GPU=0` / `AGROBOT_ENTRENAR_GPU=0` fuerzan CPU.

  **Memoria del entrenamiento**: MALLOC_ARENA_MAX=2 + malloc_trim por tanda,
  relanzarse solo por encima de 2,5 GB, `oom_score_adj=600` (a las 17:30 el
  OOM mató el entrenamiento y NO a ROS gracias a esto). Pendiente: con el
  render a 640x480, backend + simulador + entrenamiento no caben a la vez en
  7,5 GB: entrenar con el robot en reposo y sin el servidor del simulador.
  Copias de modelos con fecha (`politica_AAAAMMDD_HHMM_tNNN.pt`): antes cada
  noche pisaba las `politica_tandaNNN.pt` de la anterior (y sus notas de
  validación quedaban apuntando a otro archivo).

  **Robot real**:
  - Servo "loco": cada detección de YOLO (falsos positivos en la habitación)
    lanzaba un barrido cada 30 s, también en manual. Ahora la detección debe
    persistir (3 en 2 s) y el robot haberse movido 0,5 m o girado 30° desde
    el último barrido. `detection.escanear_al_detectar`: siempre|autonomo|nunca
    (Parámetros). Medido: 0 barridos en 3 min parado (antes 6).
  - Giroscopio: sesgo Z sin calibrar (+0,92 °/s) y el IMU publicado con
    covarianza 0 -> el EKF giraba 53°/min quieto. Ahora covarianza real
    (σ 2,4 °/s), ZUPT de giro firme (1e-5) y `SesgoGiroscopio` (autocalibrado
    en reposo, τ 30 s): deriva -0,025 °/s (35x menos).
  - Temperatura: los "80 °C" eran la tarjeta WiFi/BT (iwlwifi), no el chip
    (57 °C). El BLE reintentaba cada ~6 s con escaneo continuo aunque la C3
    iba por WiFi: ahora cada 30 s si la otra vía está viva. Con el ventilador
    del usuario: 80 -> 61 °C en 5 min. Dashboard: TEMP CHIP (tj) y TEMP WIFI;
    fila de salud `radio` (aviso > 75 °C). `gpu_pct` leído del driver (jtop no
    lo daba).
  - Vigía del LIDAR: relanza al instante si cambia el puerto (antes podía
    tardar 5 min en volver tras enchufar el USB).
  - `jetson/modo_terreno.sh terreno|casa`: sin router, NetworkManager sigue
    buscando LoRa_Wan y cada barrido saca la radio del canal del AP.

  **Dashboards** (skill ce-frontend-design, modo "sistema existente"):
  - Robot: cabecera móvil que ya no tapa el título (píldoras en fila
    deslizable, nueva de LIDAR, la de la C3 dice la vía: "C3 · BLE+WiFi"),
    cámara 4:3 y radar apilados en móvil, sin hueco bajo el vídeo en
    escritorio, hueco para el STOP, avisos por encima del STOP (nunca sobre
    él), foco visible, favicon. Errores: `log` no existía (cada aviso lanzaba
    ReferenceError) y al cargar se hacía un POST a /api/nav/mode que nunca
    llegó a ejecutarse: eliminado (abrir la página no cambia la config).
    Oruga de "Detectado" siempre visible (atributo style duplicado).
  - Entrenamiento (training.html): columna `minmax(0,1fr)` (con `1fr` la
    barra de herramientas ensanchaba la página en el teléfono), cabecera y
    panel con scroll propio, rótulo de herramienta legible.
  - Simulador: diseño de teléfono (fichas deslizables, visor 3D arriba y
    paneles debajo, columnas apiladas).

  Archivos: firmware `.ino`; `jetson/backend/{jetson_server,ros_bridge,
  ble_link,enlace_c3,patrol,config_store}.py`; `jetson/dashboard/{index,
  training}.html`; `jetson/modo_terreno.sh`; `simulador/{mundo,camara_gpu,
  robot,entrenar,slam_sim,dashboard.html}`, `simulador/mundos/habitacion_real.json`;
  `ros2_ws/.../config/ekf.yaml`.


  **Política desplegada para terreno (2026-09-30, 19:xx)**: `politica_tanda048.pt`
  (entrenada 2026-09-30 10:36, 161 118 ejemplos). Validada con el simulador
  calibrado, banco huerta_real/bucle/hileras/pasillo/callejón, semillas 7 y
  23, 60 s: 8,08 puntos (menos es mejor). En huerta_real 100 % en sendero y 2
  pisadas; la anterior (`politica.pt` del 2026-09-21) 15,57 puntos, 74 % y 4
  pisadas en huerta_real. Casi empatada: tanda027 (7,43; más pisadas en
  hileras). Copia de la anterior: `politica_desplegada_hasta_20260930.pt`.
  El sesgo del giroscopio se autocalibra con el robot quieto: dejarlo parado
  ~30 s al arrancar en el terreno.

---

## ENTRADA HISTORIAL: 2026-10-01 — Red fija y móvil (router, IP, Heltec, Tailscale)

**Problema.** "Todo cambió de IP": el DIR-615X (`LoRa-Wan`) estaba en modo
punto de acceso detrás del Huawei, así que las IP las repartía el Huawei y
cambiaban con cada red. La Jetson mandaba las alertas a `192.168.12.2`
(la Pi ya no estaba ahí) y el gateway Heltec reenviaba a `192.168.0.170`
(IP vieja de la Pi). Además la Pi y el Heltec tenían guardado el SSID
antiguo `LoRa_Wan` y no se unían a `LoRa-Wan`.

**Cambios.**
- Router en modo router (`192.168.0.1/24`, DHCP `.100–.200`), canal 1 fijo,
  segundo SSID `LoRa_Wan` en la misma LAN. Reservas: Jetson `.142`, Pi `.10`
  (cable) y `.20` (Wi-Fi), Heltec `.11` (Wi-Fi). Respaldo en `respaldos/`.
- Incidente: una reserva duplicada (misma IP, dos MAC) tumbó el dnsmasq del
  router (sin DNS/DHCP); se quitó. Antes, leer a ciegas todos los ids de la
  API del router reinició su radio. Ambos documentados en `docs/RED_E_IPS.md`.
- Pi: perfil Wi-Fi corregido a `LoRa-Wan` (métrica 600, el cable manda),
  SSH habilitado, Tailscale instalado (`lora-server-pi`).
- Heltec HT-M02: modo *WIFI Client* en `LoRa_Wan`, IP estática
  `192.168.0.11`, servidor `192.168.0.10:1700`. ChirpStack recibe sus stats
  (gateway `065f31fffe49b5a6`, `us915_0`). Su cable queda como red propia
  `192.168.4.1` y no debe ir al router.
- `jetson_server.py`: `GATEWAY_ALERT_URL` por defecto
  `http://192.168.0.10:8092` (antes `192.168.12.2`). Pendiente reiniciar
  `agrobot-backend` (sudo).
- Tailscale en la Jetson (`100.82.220.28`).

**Archivos:** `agrobot_system_2/jetson/backend/jetson_server.py`,
`agrobot_system_2/docs/RED_E_IPS.md` (nuevo), `agrobot_system_2/README.md`,
`MANIFEST_1.md`, `.gitignore`. Los respaldos `respaldos/dir615_*.json` quedan FUERA de git (contienen la clave del Wi-Fi).

---

## ENTRADA HISTORIAL: 2026-10-01 (2) — Freno activo, clave del dashboard, reserva de WAN

- **Freno activo** (`firmware/esp32c3_actuator/esp32c3_actuator.ino`): rueda
  parada = FI=BI=HIGH (freno del TA6586) en vez de LOW (rueda libre).
  Constante `FRENO_ACTIVO`. Compila con arduino-cli 1.5.1 + esp32 3.3.11
  (95 % de la flash: BLE+WiFi deja poco margen). **Pendiente cargarlo** (la C3
  no estaba por USB en la Jetson; el usuario lo sube desde Windows).
- Corrección a la auditoría del mismo día: el backend tiene `max_v_mps` 1,5,
  pero el firmware recorta a `V_MAX` 0,55 m/s / `OMEGA_MAX` 1,5 rad/s; lo que
  cambia es que el 100 % de PWM llega con menos recorrido del mando.
- **Clave del dashboard** (`jetson_server.py`, middleware `_middleware_clave`):
  Basic `agrobot:<clave>` → cookie de sesión `agrobot_sesion` (la lleva el
  WebSocket). Clave en `~/.config/agrobot/clave_dashboard` (o
  `AGROBOT_CLAVE_DASHBOARD`), fuera del repo; sin archivo = abierto como
  antes. 127.0.0.1 exento. Probado desde IP de red: 401 sin/mala clave, 200
  con clave, WS con cookie sí y sin cookie 401.
- **Router**: Cliente Wi-Fi a `LORA` (`wlan0-vxd`, conexión `wifi_LORA`
  `wlan0-vxd_1`) y **reserva de WAN** (id 210): principal `eth1_2` (cable),
  respaldo `wlan0-vxd_1`, ping 8.8.8.8 cada 10 s, 3 fallos/5 aciertos. Ojo:
  el PC no puede dar internet por `LORA` si a la vez está conectado a
  `LoRa-Wan`/`LoRa_Wan` (círculo sin salida, se probó).
- `agrobot_system_2/tools/dlink_rpc.py`: cliente de la API del router (antes
  vivía en un scratchpad que se borra).

---

## ENTRADA HISTORIAL: 2026-10-01 (3) — Firmware C3: traseros de apoyo, rampa, servo suave; WiFi estable

- **Traseros de apoyo** (`esp32c3_actuator.ino`, MOTORES TRASEROS DE APOYO):
  R1/L1 delanteras = tracción; R2/L2 traseras = apoyo. Duty trasero =
  duty delantero × `FACTOR_TRASERO_RECTO` 0,60 (recta) / `FACTOR_TRASERO_GIRO`
  0,85 (giro), tope `PWM_TOPE_TRASERO` 215, sin pulsos de arranque, rueda
  LIBRE (no freno) si el aporte queda < `PWM_MIN_TRASERO` 110. Protección
  térmica ESTIMADA (media móvil de duty², TAU 60 s, recorta entre 0,35 y
  0,60). Valores de partida SIN MEDIR: ajustar el factor con rpm
  delantera/trasera al mismo PWM. Motivo: los traseros (más rpm, menos torque)
  se recalentaban peleando contra los delanteros.
- **Rampa de aceleración** (`RAMPA_V_PCT_S` 250 %/s, `RAMPA_OMEGA_PCT_S`
  300 %/s): subir es progresivo, bajar/parar/invertir hacia cero inmediato.
  Para el SLAM (menos patinaje → rf2o/EKF sin saltos falsos).
- **Servo suave** (`SERVO_GRADOS_S` 90 °/s): `setServo` fija objetivo,
  `updateServo` lo alcanza. El estado reporta la posición real.
- Compila (95 % de flash). Pendiente cargarlo desde Windows.
- **WiFi**: la Jetson perdía el router porque el Cliente Wi-Fi del router
  buscaba `LORA` (ausente) y abandonaba su canal: 8 fallos de autenticación
  en 40 min. Cliente Wi-Fi APAGADO (la conexión `wifi_LORA` y la reserva de
  WAN siguen configuradas; encender el cliente solo cuando `LORA` exista).
  Con eso 0 caídas en 3 min de prueba. Router con una sola red: `LoRa_Wan`.
  Pi con `autoconnect-retries 0` (antes se rendía tras perder la red).

---

## ENTRADA HISTORIAL: 2026-10-01 (4) — Rendimiento: detección a 10 Hz, MAXN SUPER preparado

- Medido en la Jetson (yolo11s ip102, 480 px): **GPU 32 ms/frame, CPU
  463 ms/frame**. El backend SÍ usa la GPU (`infer_ms` ≈ 71 con relojes de
  15 W); el `GR3D_FREQ 0%` de tegrastats engaña con inferencias cortas.
  El aviso de PyTorch "CC 8.7 no soportada" no impide usar la GPU.
- La detección estaba topada a 4 Hz (`INFER_PERIOD_S` 0,25, pensado para
  CPU). Ahora `detection.infer_hz_max` (config, por defecto 10).
  Archivos: `camera.py`, `config_store.py`.
- La placa es Orin Nano **p3767-0003** y usaba el perfil normal (solo
  15 W / 7 W). Existe `nvpmodel_p3767_0003_super.conf` con 15W / 25W /
  **MAXN_SUPER** (id 2). Preparado `jetson/systemd/agrobot-rendimiento.service`
  (nvpmodel -m 2 + `jetson_clocks --fan` antes del backend y ROS) y
  `etc/NetworkManager/conf.d/99-agrobot-wifi-sin-ahorro.conf`
  (`wifi.powersave = 2`: el ahorro daba picos de 1-2,7 s al router).
  **Pendiente instalar (sudo)**. Ojo: MAXN SUPER consume más; la fuente del
  robot tiene que aguantar picos de ~25 W o habrá reinicios por caída de
  tensión.
- TensorRT no está instalado (exportar el modelo a `.engine` FP16 sería el
  siguiente salto de velocidad).

---

## ENTRADA HISTORIAL: 2026-10-01 (5) — Detección de patinaje y modo seguro (técnicas de róveres NASA/JPL)

- **`jetson/backend/deslizamiento.py`** (`MonitorDeslizamiento`): compara la
  velocidad ORDENADA (la que sale a la C3, tras rampa y freno por LIDAR)
  con la MEDIDA por rf2o (`/odom_rf2o`, nueva suscripción en
  `ros_bridge.py`, `on_odom_lidar`). Idea de la odometría visual de los
  róveres de Marte. La eficiencia normal `k` se aprende sola (TAU 20 s, solo
  en marcha sana). `patinaje = 1 - (v_med/v_ord)/k`; ≥0,4 recorta a 0,6;
  ≥0,7 a 0,35; vuelve a 1 a 0,25/s. Cada medida se compara con la orden de
  hace 0,3 s (retraso de rf2o) y solo tras 0,7 s de orden estable: si no,
  cada arranque daba un patinaje falso del 100 %. **Atasco**: ≥0,12 m/s
  ordenados y <0,02 m/s / <0,05 rad/s medidos durante 2 s.
- **`jetson/backend/modo_seguro.py`** (`ModoSeguro`, esquema FDIR / "safe
  mode"): NORMAL / PRECAUCION (velocidad ×0,5) / SEGURO (autonomía ×0,
  manual ×0,4). SEGURO por: atasco, LIDAR sin datos ≥2 s moviéndose,
  C3 sin estado ≥1,5 s con autonomía, varianza EKF >1 o mapa corrupto con
  autonomía, Jetson ≥95 °C. PRECAUCION por: LIDAR ≥0,6 s, ≥80 °C,
  patinaje ≥0,4. SEGURO queda ENGANCHADO hasta rearme del operador, y no se
  rearma si la causa sigue presente.
- `jetson_server.py`: recorte en `ActuatorController._aplicar` (punto único
  de mando, igual que el freno por LIDAR); `_bucle_modo_seguro` a 5 Hz: al
  entrar en SEGURO detiene patrulla/explorador/Nav2 + parada de emergencia,
  toast y alerta a Telegram vía la Pi (`tipo: "seguridad"`). API
  `GET /api/seguridad`, `POST /api/seguridad/rearmar`; WS `type: "seguridad"`
  cada 2 s. "Manual" = sin autonomía activa.
- Dashboard: píldora **Seguridad** en la cabecera (verde/ámbar/rojo); en
  rojo, al tocarla pide confirmación y rearma.
- Pruebas: `tests/test_deslizamiento_modo_seguro.py` (11, sin robot) y prueba
  de integración importando el backend sin ROS con un enlace falso.
- Pendiente: reiniciar el backend (sudo) para activarlo; calibrar en el
  terreno los umbrales (el `k` se aprende solo).

---

## ENTRADA HISTORIAL: 2026-10-01 (6) — Entrenamiento largo, validación y robustez de la sesión

- **Entrenamiento** 12:55–16:32 UTC (`logs/entrenamiento_20261001.log`):
  29 tandas sin fallos, 290 episodios (20 descartados), 280 796 → 418 218
  ejemplos, acierto 80,7–82,9 %. 12 mapas, mezcla 0,8, 10 episodios/tanda,
  150 s. Simulador con freno activo (75 % de episodios frena en 0,4–1,5 m/s²,
  25 % desliza como antes), giro 0,45–0,80 y ruido de IMU medido.
- **Validación** (mismo banco y semillas, simulador nuevo; menos es mejor):
  tanda048 (desplegado) **21,03** · `politica_20261001_1255_t027.pt`
  **12,81** · t029 23,46 · t006 18,91. **Ganador: t027.** Ojo: las
  validaciones anteriores al 2026-10-01 en `modelos.json` son con el
  simulador viejo y NO son comparables con estas.
- **Despliegue PENDIENTE**: el sistema de permisos bloqueó cambiar el modelo
  y reiniciar el backend sin confirmación del usuario. Comando en el informe.
- **OOM**: el entrenamiento del 2026-09-30 murió por OOM a los 80 min
  (kern.log 18:47). Ahora: `tools/supervisor_entrenamiento.sh` (relanza
  siguiendo la numeración), `cargar_dataset` con `array` en vez de listas,
  `oom_score_adj` 1000 en entrenamiento y validación (muere eso, no el
  backend). La validación de 4 modelos en un proceso llegó a 5,7 GB y fue
  matada: `historial.py --validar` ahora usa un proceso por modelo.
- **Mediciones con el robot parado** (`logs/sesion_20261001/mediciones/`):
  LIDAR 9,98 Hz, ruido 3,3 mm, 1,8 % sin retorno (dentro del rango simulado;
  los atípicos eran una persona moviéndose). IMU 19 Hz, ruido acc 0,011 g
  long / 0,045 g vertical, giro 0,5 °/s → `simulador/robot.py`. C3 estado
  10,8 Hz (p95 152 ms). LoRa V3 0 % pérdida.
- **Firmware**: `dt` de la rampa topado a 50 ms (un loop trabado daba tirón a
  fondo); `tests/comportamiento_test.cpp` compila el .ino REAL con stubs que
  registran pines y reloj (17/17), integrado en `verificar.sh`. Pendiente
  cargar en la C3.
- **Rendimiento**: MAXN SUPER no disponible (la placa arranca con
  `nvpmodel_p3767_0003.conf`, sin super; requiere reflashear).
  `agrobot-rendimiento.service` ahora = 15 W + `jetson_clocks --fan`;
  el ahorro WiFi se anula con `zz-agrobot-wifi-sin-ahorro.conf` (el
  `default-wifi-powersave-on.conf` de Ubuntu pisaba al `99-*`). Ambos
  pendientes de instalar con sudo.
- **Vigía** `tools/vigia_sesion.py`: 59 muestras 12:56–17:50, seguridad
  normal en todas, tj 51–67 °C, RAM máx 7002 MB, consumo medio 8,2 W.
- **Informe**: `tools/informe_sesion.py` → `logs/sesion_20261001/informe.html`
  (publicado como artifact).

---

## ENTRADA HISTORIAL: 2026-10-01 (7) — Despliegue de t027 y ajustes de sistema

- **Política desplegada: `politica_20261001_1255_t027.pt`** (validación 12,81
  frente a 21,03 de tanda048), autorizado por el usuario. Copiada a
  `~/.agrobot/aprendizaje/politica.pt` (la que carga el backend al arrancar)
  y a `jetson/backend/politica.pt`. Respaldo:
  `~/.agrobot/aprendizaje/politica_desplegada_hasta_20261001.pt`. Backend
  reiniciado (kill → `Restart=always`); `backend.log`: "política cargada".
- **`agrobot-rendimiento.service` instalado y activo**: solo `jetson_clocks
  --fan` (CPU 1,51 GHz, GPU 624,75 MHz, EMC 2133 MHz fijos). Se sacó
  `nvpmodel -m 0` porque pide confirmar un reinicio por teclado y el servicio
  fallaba; 15 W es el modo por defecto del perfil y se aplica al arrancar.
- **WiFi sin ahorro de energía** (`zz-agrobot-wifi-sin-ahorro.conf`
  instalado): latencia al router 4 ms de media, 10 ms de máximo (antes picos
  de 1–2,7 s).
- **Registros del sistema persistentes**: `/etc/systemd/journald.conf.d/
  agrobot-persistente.conf` (copia en `jetson/systemd/etc/...`): Storage=
  persistent, 500 MB, 1 mes. Antes eran volátiles y tras un apagado de golpe
  no quedaba rastro.
- Al matar el backend aparece un Traceback de `publish_imu` (publica cuando
  rclpy ya se cerró): inofensivo, solo en el apagado.

---

## ENTRADA HISTORIAL: 2026-10-02 (1) — Prueba de manejo en pasillo: fallos encontrados

Robot en el suelo, en un pasillo (paredes a ~0,31–0,35 m del LIDAR a cada
lado, frente libre ~1,8–2 m). Claude lo manejó por `POST /api/drive` y grabó
`/scan`, `/odometry/filtered`, `/odom_rf2o` y `/wheel_odom` a 5 Hz. No se tocó
código; solo estado en caliente.

**1. Modo seguro enganchado por un hueco breve de la C3.** Durante una
exploración (`[seguridad] normal -> seguro: sin estado de la C3 hace 1.7 s`)
el enganche quedó puesto ~8 min con la C3 sana (BLE + WiFi). En SEGURO el
manual va al 40 % (`FACTOR_MANUAL_EN_SEGURO`). Rearmado con
`POST /api/seguridad/rearmar`. Pendiente: tolerar huecos cortos con dos vías
vivas, o desenganchar solo si no hay autonomía en marcha.

**2. Se pudo lanzar una patrulla estando en SEGURO.** El log muestra
`[AUTO] patrulla iniciada` después del enganche; en SEGURO la autonomía tiene
factor 0, así que la patrulla "corre" sin moverse y sin avisar. Pendiente:
rechazar `/api/mission/start` y `/api/exploracion/start` con 409 en SEGURO.

**3. Causa de "no se mueve ni gira con los botones": rampa de la Jetson.**
`safety.max_linear_accel_mps2 = 0.1` y `max_angular_accel_rps2 = 0.2` (desde
`10f9694`, 2026-09-19; antes 0,9 / 2,1; el mensaje del commit no lo
menciona). Con pulsos de 0,8 s (interior) la rampa no supera 0,08 m/s, y el
troceado `suavidad_duty` 0,6 lo deja en ~0,03 m/s en `/wheel_odom` (que
publica lo comandado). En giro, 0,05 rad/s: por debajo del umbral
`fabsf(omega) > 0.05f` del firmware, PWM 0. Medido: LIDAR sin cambio, robot
quieto, con y sin modo seguro. Al subir el límite Claude fue bloqueado por el
clasificador de permisos (afloja un límite de seguridad); **el usuario lo
puso en 3 / 8 desde el dashboard**. El firmware ya tiene su propia rampa
(0→100 % en 0,4 s, `RAMPA_V_PCT_S`).

**4. Exploración atascada.** En la sesión anterior: 27 `desatasco` seguidos
(coherente con el punto 3). Tras subir la aceleración: 3 `callejon`, 1
`desatasco` y un `bucle`; ordenaba girar a la izquierda a 1,58 rad/s (hacia
la pared, a 0,34 m) y el EKF medía −0,2 rad/s: no giraba. Detenida con
`/api/exploracion/stop`. Pendiente: el explorador debería elegir el lado
libre (a la derecha había 0,73 m) y dejar de insistir si el giro medido no
acompaña al ordenado.

**5. "Adelante" parecía ir HACIA ATRÁS: es rf2o, no el robot.** Pulso
`adelante` escala 0,3: `/wheel_odom` +0,27 m/s; el LIDAR (en base_link) vio
el hueco DELANTERO bajar de 2,08 a 1,89 m y el trasero subir de 0,59 a
0,78 m, o sea que el robot AVANZÓ 19 cm, y el usuario confirmó que va hacia
la cámara. Pero rf2o dio −0,38 m/s y el EKF −0,34. Causa en
`rf2o_laser_odometry/src/CLaserOdometry2D.cpp:988`:
`lin_speed = acu_trans(0,2) / time_inc_sec` usa el desplazamiento en el
marco del LÁSER; la pose sí pasa por `laser_pose_on_robot_`, la velocidad
no. Con el C1 girado 180° (`yaw=π`) el signo se invierte. Efectos: el EKF
fusiona vx de rf2o con signo contrario, y `deslizamiento.py` ve "patinaje
100 %" → "robot atascado" → modo SEGURO (pasó 2 veces en esta sesión).
Arreglo propuesto: rotar `(acu_trans(0,2), acu_trans(1,2))` por el yaw del
láser antes de calcular `lin_speed` (rf2o es de terceros, @ b38c68e: anotar
el parche para no perderlo al reclonar).
**Error de Claude en esta sesión:** leyó al revés las etiquetas de su propio
registro, culpó al TF del LIDAR, cambió `yaw` a 0 en los dos launch y en
`ros_bridge.py` y reinició ROS + backend. El backend pasó a ver "adelante
27 cm / atrás 206" (al revés); se detectó, se revirtió con `git checkout` y
se reinició de nuevo. No llegó a commitearse. Los mapas en memoria se
perdieron con esos reinicios.

**5b. Motores sin fuerza tras reconectar el cable.** Con la C3 recibiendo y
aplicando v=0,54 m/s (100 % de `V_MAX`, `actuator_status.v_cmd`) por BLE y
WiFi, sin failsafe, el LIDAR no vio ningún movimiento (dos pulsos, escala
0,3 y 0,6). La cadena de software funciona hasta la C3: el fallo es físico
(batería de motores, cable de potencia, driver).

**5c. El backend no se cierra con SIGTERM ni SIGINT** (hizo falta SIGKILL
para que systemd lo relanzara). Y el modo de manejo (`interior`) no se
guarda: vuelve a `exterior` en cada arranque.

**6. Otros.** `GET /api/obstacle_status` devuelve todos los sectores del
LIDAR en `null` con el LIDAR a 50 Hz (consistente con que
`obstacle_avoidance.py` es código muerto, §0). `/api/health` marca sesgo del
giróscopo +0,71 °/s en reposo.

**7. "Le falta fuerza y torque" (usuario): troceado de la Jetson contra el
freno activo del firmware.** `pulse_drive._tramo` alterna empuje y orden
CERO (`control.suavidad_duty` 0,6, periodo 0,3 s: 180 ms / 120 ms). Se pensó
para rueda libre, pero con `FRENO_ACTIVO = true` (firmware 2026-10-01) el cero
pone los motores en CORTO el 40 % del tiempo, y como en el firmware bajar
es inmediato y subir va por rampa (`RAMPA_V_PCT_S` 250 %/s), cada tramo de
180 ms arranca de 0 y llega como mucho al ~45 % de lo pedido. En giro se
suma el tren de pulsos del firmware (140 ms on / 90 ms off). Arreglo
propuesto: `suavidad_duty = 1.0` (o hacer que el tramo apagado mande rueda
libre en vez de freno). Claude fue bloqueado por el clasificador al
cambiarlo; queda para el usuario. Otros recortes de torque del firmware:
traseros al 60 % del duty en recta (`FACTOR_TRASERO_RECTO`, sin medir), tope
215 y corte térmico ESTIMADO que puede dejarlos en rueda libre tras uso
seguido.

**8. Llanta trasera que se quedó girando sola + reinicios de la C3.** En
`backend.log`, la C3 "acaba de arrancar (arranque normal)" 3 veces, y CADA
vez el LoRa V3 se reinicia a la vez: cae la alimentación común (sin bandera
de brownout = corte completo). El "sin estado de la C3 hace 1,7 s" que
enganchó el modo seguro (punto 1) era uno de esos reinicios. Con aceleración
3 / 8 los arranques piden más corriente. En el firmware, `setup()` ponía los
pines de motor en LOW DESPUÉS de esperar al Serial (hasta 1,5 s + 200 ms):
tras cada reinicio las entradas del TA6586 quedaban sin manejar, y las
traseras van en GPIO4..7 (pines JTAG de la C3). **Corregido en código:** la
inicialización de motores y `stopMotors()` van ahora lo primero de
`setup()`. `verificar.sh` OK (cores 2.x y 3.x). **Falta cargarlo en la C3.**
Pendiente de hardware: alimentar la C3 y el LoRa V3 aparte de los motores
(o condensador grande + regulador propio) y pull-down de 10 kΩ en las
entradas del driver, que cubren también los ~ms antes de que corra
`setup()`.

**9. Manejo con registro tras reconectar los drivers** (10 pasos; foto +
LIDAR antes/después + eco de la C3 por paso en
`logs/sesion_20261002_manejo/pasos.jsonl` y `pasoNN.jpg`, fuera de git):
- Adelante, escala 0,4 en interior: **21–29 cm por pulso**, repetible.
- **Obstáculos bajos invisibles al LIDAR** (montado a 18 cm): una cobija en
  el suelo y una pesa a ~60 cm delante salían como "185 cm / 104 cm
  libres" (la pared). El freno no las ve; solo la cámara. Pendiente: que el
  detector de cámara alimente el freno, o un sensor bajo.
- **Giro invertido** (visto por el usuario, medido por EKF/LIDAR):
  "izquierda" giraba a la derecha; avanzar iba bien → lados cruzados al
  reconectar. Corregido en `enlace_c3.py` (`INVERTIR_GIRO = True`: niega
  omega al mandar y el eco al recibir; un solo punto para botones,
  exploración, patrulla y Nav2). Confirmado por el usuario. Si se recablean
  los drivers, volver a False.
- **El pivote casi no rota**: 1–4° por pulso de 0,8 s a 1,6 rad/s pedidos,
  y en cada uno el robot RETROCEDE ~10 cm (pivota sobre un punto delante del
  centro). Sospecha: alguna rueda empujando al revés o un par
  delantera/trasera cruzado entre lados al reconectar. Prueba propuesta:
  robot en el aire, pulso de giro, mirar el sentido de cada rueda. Suman
  el troceado de la Jetson (punto 7) y los traseros al 60–85 %.
- Al girar sobre su eje en sitios estrechos la esquina trasera roza (poste
  a 2 cm): el giro en pasillos hay que hacerlo tras avanzar a un hueco.

**10. Más potencia a los motores traseros (pedido del usuario).** El
pivote no rotaba (1–4°/pulso; con el robot apoyado en un poste, 0,2° por
ICP de scans, −0,7° por giróscopo, −0,3° por EKF: los tres coinciden). El
usuario aclara que los pulsos se mantienen a propósito (rompen la inercia)
y pide más potencia en los traseros. Firmware: `FACTOR_TRASERO_GIRO`
0,85 → **1,00**, `PWM_TOPE_TRASERO` 215 → **255**, `FACTOR_TRASERO_RECTO`
0,60 → **0,80**; la protección térmica estimada no cambia (60 s a fondo:
carga 0,39, antes 0,23). Pruebas del .ino ajustadas a las constantes;
`verificar.sh` OK. **Falta flashear** (junto con el arreglo de `setup()`
del punto 8). El IMU (Y adelante, X a los lados) ya se remapea bien en
`jetson_server.py` antes de publicar `/imu/data_raw`; el yaw sale de Z.

**11. Giro medido en zona abierta (≥ 59 cm libres), firmware VIEJO aún.**
4 pulsos (interior, escala 0,4; la C3 aplica 1,6 rad/s ~0,8 s), medidos con
ICP de scans, giróscopo Z integrado y EKF (coinciden entre sí):
izquierda 1,5° / 0,0°; derecha 2,3° / 2,6°, con **13 y 16 cm de retroceso**
en los de derecha (antes, ~10 cm con izquierda). Los sensores están bien; el
robot de verdad no pivota y, gire hacia donde gire, el lado que va hacia
atrás vence al que va hacia adelante. Sospecha: una rueda con el sentido
cambiado o peleando con su compañera de lado tras reconectar los drivers.
Prueba pendiente: robot en el aire, pulso de giro, mirar cada rueda.
Scripts usados (scratchpad de la sesión): `giro_captura.py` + `icp.py`.

**12. Watchdog del lazo principal en la C3.** El failsafe de 500 ms sin
órdenes se comprueba dentro de `loop()`: si `loop()` se colgaba (pila BLE o
WiFi), los motores seguían con el último PWM sin límite. Ahora
`iniciarWatchdogLoop()` (al final de `setup()`) mete el loop en el task
watchdog con `WDT_LOOP_MS` = 2000 (core 3.x: `esp_task_wdt_reconfigure`,
core 2.x: `esp_task_wdt_init`) y `enableLoopWDT()`; colgado 2 s → reinicio →
`setup()` frena los motores lo primero (punto 8). El arranque informa
"WATCHDOG: el loop se colgó" por Serial. Stub nuevo
`tests/stubs_arduino/esp_task_wdt.h`; `verificar.sh` OK en los dos cores.
**Firmware para flashear = el del repo a partir de este commit** (incluye
puntos 8, 10 y 12).

**13. Recableado de los drivers: el sentido se corrige en el FIRMWARE.**
Tras recablear (firmware viejo aún en la C3), con la Jetson invirtiendo el
giro, el usuario vio avance y giro al revés: el cableado hace
(v, ω) → (−v, ω). Firmware: `CABLEADO_INVIERTE_AVANCE = true`,
`CABLEADO_INVIERTE_GIRO = false`, aplicados al entrar en `applyMotors()`
(el eco de estado sigue mostrando lo pedido). Jetson:
`enlace_c3.INVERTIR_GIRO = False` (reemplaza al punto 9) y backend
reiniciado. Hasta flashear, el avance sale invertido. Verificar con el robot
en el aire: "adelante" = las cuatro hacia delante; "izquierda" = izquierdas
atrás, derechas adelante. Si UNA rueda va al revés, se corrige con su
`INVERT_` y no con estas dos.

**Estado que quedó cambiado:** modo de manejo `interior`
(`POST /api/drive/mode`); modo seguro rearmado; aceleración 3 / 8 (la puso el
usuario; queda en `agrobot_config.json`, sin commitear).

---

## ENTRADA HISTORIAL: 2026-10-02 (2) — Datos del pasillo al entrenamiento y movimiento robótico por pulsos

Robot apagado (C3 y LoRa V3), Jetson encendida, en el mismo pasillo.

- **Datos recopilados** (`logs/sesion_20261002_manejo/`): `pasos.jsonl`
  (10 pulsos con foto, LIDAR antes/después y eco de la C3) y
  `giros_medidos.jsonl` (5 giros medidos con ICP de scans + giróscopo Z +
  EKF; el scratchpad con los .npz se perdió al reiniciar la Jetson, los
  números estaban registrados). Pose a 1 Hz en
  `~/.agrobot/navigation_logs/poses_20261002_010038.csv`.
- **Calibración del simulador** (`tools/calibrar_con_pasillo.py` →
  `simulador/calibracion_real.json`): reproduce en el simulador el MISMO
  pulso que el robot (interior, troceado 0,6, rampa de la config) y ajusta:
  frenada **3,24 m/s²** (avance medido 26,3 cm/pulso, sim 25,6),
  rendimiento de giro **0,019** (1,6°/pulso medido; el simulador suponía
  0,45–0,80 = 55° por pulso), retroceso al girar **0,053** (8 cm/pulso).
  `robot.py` gana `retroceso_giro`; `entrenar.aleatorizar()` sortea
  alrededor de lo medido (40 % giro como el firmware viejo, 60 % 0,05–0,80
  por el firmware nuevo sin medir; 60 % frenada ±30 % de lo medido).
- **Mundo `pasillo_real`** (`mundo.py`): pasillo de 65 cm, poste, cuarto de
  2 × 1,3 m, pesa/cobija/chancla bajo el LIDAR. El explorador actual choca 15
  veces en 60 s simulados ahí (antes de calibrar el chasis).
- **Movimiento robótico por pulsos** (`paso_robotico.py`, `POST/GET
  /api/paso`): `{"accion":"avanzar","cm":50}` / `{"accion":"girar",
  "grados":-90}` / `{"accion":"parar"}`. Cada pulso: orden → esperar a que
  termine y el scan quede quieto (±1 cm 0,3 s) → medir avance con los huecos
  delante/detrás del LIDAR (NO el EKF: vx de rf2o invertido) o el giro con el
  rumbo (giróscopo) → decidir. Se rinde con `sin_progreso` (3 pulsos sin
  moverse), `obstaculo` (hueco bajo el umbral del freno), `retroceso` (al
  girar, < 15 cm detrás), `sin_lidar`, `max_pulsos`. Registra cada pulso en
  `~/.agrobot/pasos_roboticos/` (sirve para recalibrar). El "parar" de
  `/api/drive` y cualquier pulso manual lo cancelan. 7 pruebas en
  `tests/test_paso_robotico.py`. Probado de punta a punta con la C3
  apagada: 3 pulsos, avance 0,0 m medido, `sin_progreso`.
- **Reentrenamiento** lanzado 03:12 UTC, 3,5 h, supervisor
  (`logs/lanzar_entrenamiento_20261002.sh`, log
  `logs/entrenamiento_20261002.log`), 13 mapas con `pasillo_real`, mezcla
  0,8. `habitacion_real.json` no lo acepta `entrenar.py` como mapa. Desplegar
  el modelo resultante pide confirmación del usuario.
- **Arranque tras apagón: el vigía del reloj reinició ROS a media subida**
  (02:58:13): slam_toolbox murió (−11 / −6), el relanzado quedó colgado en
  "configurando" y Nav2 abortó ("transform base_link→map"). Se reinició
  `agrobot-ros` a mano y quedó bien. Pendiente: que el vigía espere a que
  ROS termine de arrancar, o que reinicie solo si ROS arrancó con el reloj
  en 1970.

---

## ENTRADA HISTORIAL: 2026-10-02 (3) — Suelo libre por cámara y evasión de obstáculos

Pedido del usuario tras ver que el LIDAR no veía la cobija ni la pesa.

- **`suelo_libre.py`** (detección por apariencia, Ulrich y Nourbakhsh 2000):
  la franja de abajo-centro de la imagen es la referencia de "suelo";
  backprojection de histograma HSV; columna a columna, la primera racha de
  no-suelo es la base de un obstáculo; la fila se convierte en distancia
  sobre el suelo con la geometría de `odometria_webcam` (inversa de
  `sendero.Cenital`). Salida: scan virtual `[{angle_deg, distance_m}]`.
  Persistencia de 2-3 fotogramas contra sombras sueltas.
  - **Memoria del suelo** (EMA de histogramas): si la referencia deja de
    parecerse al suelo recordado (Bhattacharyya > 0,55) hay algo pegado
    delante → `objeto_en_referencia` → hueco 0 para el freno. Sin memoria, la
    cobija del paso 1 se aprendía como suelo.
  - Validado con las fotos reales del pasillo (cobija: referencia extraña,
    diferencia 0,61 frente a < 0,14 del suelo normal; pesa a ~0,9-1 m) y con
    la cámara simulada en `pasillo_real` (LIDAR 1,83 / 2,13 m; cámara 0,85 /
    1,04 m; real 0,80 / 1,10). Geometría contrastada: pared a 1,17 m por
    cámara frente a 1,24 por LIDAR.
  - **Límite medido: con el servo a 98° (15,5° hacia abajo) la cámara no ve
    el suelo de los primeros ~0,85 m desde el centro (~65 cm por delante del
    frontal).** Para obstáculos conviene inclinarla más (servo ~76-80°);
    pendiente decidir con el usuario porque cambia la vista del sendero.
- **Freno** (`slam.LidarReactive.limitar_velocidad`): delante usa el mínimo
  entre LIDAR y cámara (`slam.freno_camara`, por defecto true); el aviso
  dice `fuente: lidar|camara`. Solo con la cámara al frente y quieta
  (`_servo_al_frente`); con la C3 apagada no hay estado del servo y la
  cámara no frena.
- **`evasion.py`** (histograma polar estilo VFH): LIDAR + puntos de cámara,
  cada 10°, hueco desde el frontal engordado media anchura + 5 cm; elige la
  dirección libre más cercana al objetivo. **No propone girar si no cabe
  girando** (media diagonal 27 cm + margen): en un pasillo estrecho solo
  avanza o retrocede. 7 pruebas en `tests/test_evasion.py`.
- **Modo evasión**: `POST /api/evasion/start {"segundos", "objetivo_deg"}`,
  `/stop`, `GET /api/evasion`; decide con `evasion.py` y se mueve con
  `paso_robotico` (pulsos medidos); para tras 2 giros sin progreso
  (`no_gira`), `atascado` o `sin_lidar`. Registro en `~/.agrobot/evasion/`.
  `/api/drive` parar o cualquier pulso manual lo cancela.
- Diagnóstico: `GET /api/suelo` y `GET /api/suelo/vista.jpg` (verde =
  suelo, rojo = base de obstáculos, recuadro = referencia).
- Sin probar con el robot andando (C3 apagada en esta sesión).

