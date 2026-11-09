# Anexo B. Firmware, pines y puesta en marcha

Valores tomados del código al 8 de noviembre de 2026. Las contraseñas y claves no se incluyen por seguridad.

## B.1 Firmware

| Placa | Función | Firmware | Commit |
|---|---|---|---|
| ESP32-C3 | Motores y servo | FUSIÓN, versión 5 | `ebde6ea` |
| Heltec LoRa 32 V3 | Telemetría AU915 y OLED | `esp32_lora_v3` | `4f08332` |

La versión de la C3 debe confirmarse con la placa encendida. Los sensores MPU6050 y SCD40 del nodo no responden.

## B.2 Pines

**Tabla B-1.** ESP32-C3. Los pines de motor se pueden cambiar desde el panel.

| Función | GPIO | Función | GPIO |
|---|---|---|---|
| Delantera derecha, FI / BI | 0 / 1 | Trasera izquierda, FI / BI | 4 / 5 |
| Delantera izquierda, FI / BI | 3 / 2 | Trasera derecha, FI / BI | 6 / 7 |
| Foco LED | 8 | Servo de cámara | 10 |

**Seguridad.** Si deja de recibir órdenes, la C3 para los motores a los 1 200 ms por Wi-Fi o a los 400 ms por USB.

**Tabla B-2.** Heltec V3.

| Función | Pines |
|---|---|
| Sensores (MPU6050 y SCD40) | SDA 41, SCL 42 |
| Pantalla OLED | SDA 17, SCL 18, RST 21, Vext 36 |

## B.3 ROS 2

El robot usa ROS 2 Jazzy en la Jetson Orin Nano, con el paquete `agrobot_navigation`.

| Nodo | Función |
|---|---|
| `sllidar_node` | Lee el LIDAR C1 |
| `laser_filters` | Quita el chasis del escaneo |
| `rf2o_laser_odometry` | Calcula la odometría |
| `robot_localization` | Fusiona el estado a 20 Hz |
| `slam_toolbox` | Construye el mapa (celda de 5 cm, alcance de 8 m) |

Todos los parámetros están en `ros2_ws/src/agrobot_navigation/config/`.

## B.4 LoRaWAN

| Elemento | Configuración |
|---|---|
| Servidor | ChirpStack v4 en Docker, sobre Raspberry Pi |
| Región y canales | AU915, canales 0–7 y 64 |
| Gateway | Heltec HT-M02, puerto UDP 1700 |
| Dispositivo | OTAA, clase C |
| Mensaje | 17 bytes: acelerómetro, giroscopio, CO₂, temperatura y humedad |

## B.5 Puesta en marcha

1. Revisar baterías, seguro de motores y fijación del LIDAR.
2. Encender la Jetson. Los servicios `agrobot-ros` y `agrobot-backend` arrancan solos.
3. Conectarse a la red `JETSON_AP` y abrir el panel en `192.168.12.1:8080`.
4. Encender la C3 y el nodo de sensores. Comprobar el enlace en `/api/health`.
5. Esperar a que la hora se sincronice, porque la Jetson no tiene reloj propio.
6. Elegir el modo. En exterior: modo `exterior` y cámara a 112°.
7. Con el robot en el suelo, ejecutar `python3 tools/preflight_autonomia.py --ros --suelo-ok`.
8. Dejar el robot quieto unos 10 s en terreno plano.
9. Tener a mano el corte físico de motores.

Antes de flashear la C3 hay que detener `agrobot-backend`.
