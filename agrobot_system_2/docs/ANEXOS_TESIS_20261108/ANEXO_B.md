# Anexo B. Firmware, pines, configuración y puesta en marcha

Los valores proceden del código del repositorio a fecha del 8 de noviembre de 2026. Las contraseñas, claves de API y claves de dispositivo LoRaWAN se excluyen a propósito: deben guardarse fuera del documento y del repositorio.

## B.1 Versiones de firmware

| Placa | Firmware | Código fuente | Commit | Estado |
|---|---|---|---|---|
| ESP32-C3 (motores y servo) | FUSIÓN, `FW_VERSION` = 5, con freno activo | `firmware/agrobot_c3_fusion_unico/` | `ebde6ea` (23-oct-2026) | Último flasheo registrado. **Confirmar la versión con la placa encendida**: responde `FW_VERSION` por el canal de configuración |
| ESP32-C3, alternativa | `agrobot_c3_skidsteer_maxfrente` | `firmware/agrobot_c3_skidsteer_maxfrente/` | `2d590a7` (21-oct-2026) | Versión estable anterior; base recomendada para volver atrás |
| Heltec WiFi LoRa 32 V3 | Telemetría AU915 + OLED | `firmware/esp32_lora_v3/esp32_lora_v3.ino` | `4f08332` (3-nov-2026) | Flasheado y unido a la red. MPU6050 y SCD40 no responden |

**Compilación del nodo.** `arduino-cli` con core esp32 3.3.11, librería «Heltec ESP32 Dev-Boards» 2.1.6 y FQBN `esp32:esp32:heltec_wifi_lora_32_V3:LORAWAN_REGION=4,LORAWAN_DEVEUI=0`. El sketch no compila en otra región.

## B.2 Asignación de pines

**Tabla B-1.** ESP32-C3. Los pines de motor son valores por defecto: se pueden cambiar desde el panel y se guardan en NVS.

| Función | GPIO | Función | GPIO |
|---|---|---|---|
| Rueda R1 (delantera der.), FI | 0 | Rueda L2 (trasera izq.), FI | 4 |
| Rueda R1, BI | 1 | Rueda L2, BI | 5 |
| Rueda L1 (delantera izq.), FI | 3 | Rueda R2 (trasera der.), FI | 6 |
| Rueda L1, BI | 2 | Rueda R2, BI | 7 |
| Foco LED (activo en bajo en firmware) | 8 | Servo de inclinación de cámara | 10 |

**Parámetros de seguridad de la C3.** Sin órdenes, para los motores a los 1 200 ms por Wi-Fi o a los 400 ms por USB. Servo limitado a 25–160°, reposo en 98°.

**Tabla B-2.** Heltec WiFi LoRa 32 V3.

| Función | Pines |
|---|---|
| Sensores (`Wire1`): MPU6050 y SCD40 | SDA 41, SCL 42, 3,3 V |
| OLED integrada (`Wire`, dirección 0x3C) | SDA 17, SCL 18, RST 21 |
| Alimentación Vext de la OLED | GPIO 36, activa en bajo |
| Radio | SX1262 integrada |

## B.3 Configuración de ROS 2

Plataforma: ROS 2 Jazzy en Jetson Orin Nano. Paquete: `ros2_ws/src/agrobot_navigation`. El servicio `agrobot-ros` lanza `agrobot_mapping.launch.py` por defecto; `/etc/default/agrobot` permite elegir `agrobot_navigation.launch.py` (Nav2 sobre un mapa guardado).

**Tabla B-3.** Nodos de la pila de mapeo.

| Nodo | Paquete | Función |
|---|---|---|
| `sllidar_node` | sllidar_ros2 | LIDAR C1, 460 800 baudios, puerto por número de serie |
| `scan_to_scan_filter_chain` | laser_filters | Filtro de caja del chasis |
| `rf2o_laser_odometry_node` | rf2o_laser_odometry | Odometría por escaneo |
| `ekf_node` | robot_localization | Fusión de estado |
| `async_slam_toolbox_node` | slam_toolbox | Construcción del mapa |

**Tabla B-4.** Parámetros principales.

| Archivo | Parámetro | Valor |
|---|---|---|
| `slam_toolbox_mapping.yaml` | `resolution` | 0,05 m |
| | `max_laser_range` | 8,0 m |
| | `minimum_travel_distance` | 0,06 m |
| | `minimum_travel_heading` | 0,08 rad |
| | `transform_publish_period` | 0,02 s |
| | `map_frame`, `base_frame` | `map`, `base_link` |
| `ekf.yaml` | `frequency` | 20 Hz |
| | `two_d_mode` | verdadero |

Los parámetros completos están en `config/*.yaml`, incluido `nav2_params.yaml`.

## B.4 Configuración de ChirpStack y LoRaWAN

| Elemento | Configuración |
|---|---|
| Servidor | ChirpStack v4 en Docker sobre Raspberry Pi (`/root/chirpstack-docker`) |
| Región | `au915_0`; `enabled_regions` en `configuration/chirpstack/chirpstack.toml` |
| Puente MQTT | Plantillas `au915_0/gateway/...` en `docker-compose.yml` |
| Gateway | Heltec HT-M02, UDP Packet Forwarder, puerto 1700; IP 192.168.0.11 |
| Radios del gateway | 915,5 MHz y 916,2 MHz |
| Canales | AU915, 0–7 (915,2–916,6 MHz) y 64 |
| Perfil de dispositivo | `AU915_PROFILE`, OTAA, clase C |
| Decodificador | `rpi/chirpstack/codec_agrobot.js` |

**Tabla B-5.** Carga útil del nodo: 17 bytes, big-endian.

| Bytes | Campo | Tipo y escala |
|---|---|---|
| 0–5 | accelX, accelY, accelZ | int16 / 100 (g) |
| 6–11 | gyroX, gyroY, gyroZ | int16 / 10 (°/s) |
| 12–13 | CO₂ | uint16 (ppm) |
| 14–15 | Temperatura | int16 / 100 (°C) |
| 16 | Humedad | uint8 (%) |

## B.5 Puesta en marcha del robot

1. **Revisión física.** Baterías cargadas, seguro de motores liberado y LIDAR firme en su soporte; la vibración lo afloja.
2. **Encendido.** Encender la Jetson. No dejar conectado un adaptador Wi-Fi USB defectuoso.
3. **Servicios.** `agrobot-ros` y `agrobot-backend` arrancan solos. Comprobar con `systemctl status`.
4. **Red.** El AP `JETSON_AP` debe estar activo en 192.168.12.1 y canal 1. El panel está en el puerto 8080.
5. **Placas.** Encender la C3 y el nodo de sensores, y comprobar en `/api/health` que la C3 tiene enlace.
6. **Reloj.** Esperar a que la hora esté sincronizada: la Jetson no tiene RTC.
7. **Modo.** En exterior, fijar modo `exterior` y cámara a 112°. En interior, modo `interior`.
8. **Verificación previa.** Con el robot en el suelo, ejecutar el comando de verificación:

`python3 tools/preflight_autonomia.py --ros --suelo-ok`

9. **Reposo inicial.** Dejar el robot quieto unos 10 s en terreno plano para que IMU y SLAM tomen referencia.
10. **Parada.** Tener a mano el botón físico de corte de motores. La C3 para sola si pierde el enlace.

Antes de actualizar el firmware de la C3 debe detenerse `agrobot-backend`, que ocupa su puerto USB.
