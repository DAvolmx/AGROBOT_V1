"""
AgroBot — backend en la Jetson (`jetson_server.py`).
====================================================
El nombre del archivo importa: antes este módulo y el de la Raspberry Pi se
llamaban los DOS `server.py`. Como Python importa por NOMBRE de módulo (no
por ruta), un `import server` cargaba el que estuviera primero en sys.path
— el archivo equivocado, sin ningún aviso. Ahora cada uno tiene un nombre
único: `jetson_server.py` acá y `gateway_server.py` en la Raspberry Pi.

Une en un solo proceso asyncio:

  - Servidor UDP puerto 5001  <- ESP32 LoRa V3 (telemetría ambiental JSON)
  - Servidor UDP puerto 5002 <-> ESP32-C3 Mini (comandos binarios / status)
  - WebSocket /ws              -> dashboard (push inmediato, sin polling)
  - MJPEG      /video          -> cámara + overlay de detección YOLO
  - REST       /api/*          -> modelos, hardware, calibración, misión
  - Estáticos  /                -> dashboard/index.html

Diseño para latencia mínima: todo es push (ver README). Nada aquí depende
de internet — este proceso corre dentro de la red aislada del AP.
"""
import asyncio
import json
import math
import os
import subprocess
import time
from pathlib import Path

import aiohttp
from aiohttp import web, WSMsgType
try:
    import cv2
except ImportError:
    cv2 = None

from protocol import SERVO_HOME_DEG, clamp_servo
from uart_link import UartActuatorLink
import config_store
import health
from slam import PoseEstimator, LidarReactive
from ros_bridge import AgroBotROSBridge, ROS_UNAVAILABLE_REASON
from camera import CameraWorker
from obstacle_avoidance import ControladorEvasion, LecturasSensoresLIDAR
from patrol import PatrolController
from pulse_drive import ConductorPulsos, ACCIONES
from mapas import GestorMapas, ErrorMapa
from odometria_webcam import OdometriaWebcam, SERVO_TOLERANCIA_DEG

# =============================================
# ===== CONFIG =====
# =============================================
JETSON_AP_IP = "192.168.12.1"
LORA_UDP_PORT = 5001
HTTP_PORT = 8080
CMD_SEND_HZ = 20
# Gateway de la Raspberry Pi. La Jetson NO tiene salida a internet (vive en
# la red aislada del AP), así que las alarmas que hay que hacer llegar al
# celular del operador se empujan al gateway por LAN y él las publica en
# Telegram. Si el gateway está apagado, esto falla en silencio: avisar es
# deseable, pero nunca a costa de frenar la detección o el control.
GATEWAY_ALERT_URL = os.environ.get(
    "AGROBOT_GATEWAY_URL", "http://192.168.12.2:8092") + "/api/alert"
GATEWAY_ALERT_TIMEOUT_S = 4

# UNA sola sesion HTTP para todo el proceso, no una por aviso.
#
# `aiohttp.ClientSession()` no es un objeto barato: monta su propio conector,
# su pool de conexiones y su resolutor de DNS. Crear una por deteccion —que es
# lo que hacia `_avisar_al_gateway`— significa montarlos y tirarlos cientos de
# veces por hora, y aiohttp avisa en su documentacion de que cerrar una sesion
# no libera los transportes de inmediato.
#
# Encaja con lo medido el 2026-09-06: la memoria crecia ~6 MB/min SOLO cuando
# habia detecciones, y quedaba plana con la camara viendo pero sin detectar.
_sesion_http: "aiohttp.ClientSession | None" = None

# Tareas sueltas lanzadas desde el camino de deteccion, con referencia y tope.
#
# Dos motivos, los dos reales:
#   · asyncio guarda solo una referencia DEBIL a las tareas creadas con
#     create_task(). Sin guardarlas aqui, el recolector puede llevarse una a
#     medias y el trabajo se pierde en silencio (esta documentado en asyncio).
#   · sin tope, una racha de detecciones con el gateway lento encola tareas
#     sin limite, y la de guardar la captura retiene un FOTOGRAMA COMPLETO
#     cada una. Es la via mas directa a quedarse sin memoria.
_tareas_sueltas: set = set()
MAX_TAREAS_SUELTAS = 24


def _lanzar(coro, etiqueta: str = ""):
    """Lanza una corrutina de fondo con referencia fuerte y tope."""
    if len(_tareas_sueltas) >= MAX_TAREAS_SUELTAS:
        # Se DESCARTA en vez de encolar. Si hay 24 avisos pendientes, el
        # problema no se arregla apilando el numero 25: se arregla dejando
        # que se vacien. Perder un aviso es preferible a perder el proceso.
        coro.close()
        print(f"[tareas] descartada '{etiqueta}': ya hay {len(_tareas_sueltas)} "
              f"pendientes (tope {MAX_TAREAS_SUELTAS})")
        return None
    t = asyncio.create_task(coro)
    _tareas_sueltas.add(t)
    t.add_done_callback(_tareas_sueltas.discard)
    return t
# Tope del paso de tiempo de la rampa de velocidad: 3 periodos de comando.
# Absorbe jitter normal del stream sin permitir que una pausa larga se
# traduzca en un salto de velocidad grande (ver _apply_ramp).
RAMP_MAX_DT = 3.0 / CMD_SEND_HZ
# Hombre muerto: si nadie pide velocidad en este tiempo, el objetivo pasa a
# cero. Nav2 publica /cmd_vel a 10 Hz y el conductor por pulsos a 20 Hz, así
# que 0,5 s son 5 mensajes perdidos seguidos: solo salta si el que mandaba
# desapareció (navegador cerrado, WiFi caído, Nav2 muerto).
DEADMAN_S = 0.5
# Relativas a ESTE archivo, no al directorio de trabajo. Con `Path("./models")`
# el backend usaba carpetas distintas según desde dónde se lo lanzara:
# `arrancar.sh` hace `cd` y funcionaba, pero un `python3 jetson/backend/
# jetson_server.py` desde la raíz del repo creaba `./models` y `./captures`
# ahí, no encontraba los modelos y guardaba las fotos en otro lado — todo en
# silencio, sin un solo error.
_AQUI = Path(__file__).resolve().parent
SESSION_REPORT_DIR = _AQUI / "session_reports"
MODELS_DIR = _AQUI / "models"
CAPTURES_DIR = _AQUI / "captures"
MAPS_DIR = Path(__file__).parent.parent.parent / "ros2_ws" / "src" / "agrobot_navigation" / "maps"

SESSION_REPORT_DIR.mkdir(exist_ok=True)
MODELS_DIR.mkdir(exist_ok=True)
CAPTURES_DIR.mkdir(exist_ok=True)

# Fallback casero (slam.py) — se sigue usando si ROS/rclpy no está
# disponible en este entorno, o como respaldo del filtro complementario
# mientras el EKF de robot_localization no haya publicado pose todavía.
pose_estimator = PoseEstimator()
lidar_reactive = LidarReactive()

# Última pose/mapa que llegaron de ROS (robot_localization / slam_toolbox),
# si el puente está disponible. None hasta el primer mensaje.
ros_fused_pose: dict | None = None
ros_last_map: dict | None = None

# Sistema de evasión de obstáculos (NUEVO 2026-09-13)
obstacle_controller: "ControladorEvasion | None" = None


# =============================================
# ===== ESTADO COMPARTIDO =====
# =============================================
class SharedState:
    def __init__(self):
        self.env_telemetry = {}
        self.env_history = []          # para histogramas del dashboard
        self.actuator_status = {}
        self.detections = []
        self.captures = []             # fotos guardadas al detectar plaga (galería de misión)
        self.system_stats = {}
        self.mission_active = False
        self.config = config_store.load()
        self.ws_clients: set[web.WebSocketResponse] = set()
        self._seq = 0
        # Estado deseado de los actuadores auxiliares. Es PEGAJOSO a
        # propósito: los comandos de movimiento salen a 20 Hz y también los
        # manda Nav2 por /cmd_vel, así que el servo y el LED no pueden
        # viajar en cada mensaje de control — si no, mover el robot con las
        # flechas reposicionaría la cámara y apagaría la luz. Se guardan
        # acá y se adjuntan a TODO paquete saliente.
        # Arranca mirando al frente, no en 0: 0° está fuera del recorrido útil
        # del servo montado y el primer comando lo mandaba contra el tope.
        self.servo_angle = SERVO_HOME_DEG   # ángulo del operador (sin offset)
        self.led_on = False

    def next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        return self._seq

    def push_env_history(self, d: dict):
        self.env_history.append({
            "t": time.time(),
            "co2": d.get("co2", 0),
            "temp": d.get("tempSCD", 0),
            "hum": d.get("humidity", 0),
        })
        cutoff = time.time() - 120
        self.env_history = [p for p in self.env_history if p["t"] >= cutoff]

    async def broadcast(self, message: dict):
        """Manda el mensaje a todos los clientes EN PARALELO.

        Antes era un `await ws.send_str()` por cliente en serie: un cliente
        lento (Wi-Fi malo, pestaña en segundo plano) hacía esperar a todos
        los demás, y a 10 Hz de scan eso se acumula. Con `gather` el lento
        solo se retrasa a sí mismo."""
        clientes = list(self.ws_clients)
        if not clientes:
            return
        payload = json.dumps(message)
        resultados = await asyncio.gather(
            *(ws.send_str(payload) for ws in clientes),
            return_exceptions=True)
        for ws, res in zip(clientes, resultados):
            if isinstance(res, Exception):
                self.ws_clients.discard(ws)


state = SharedState()


# =============================================
# ===== UDP: TELEMETRÍA AMBIENTAL (ESP32 LoRa V3, puerto 5001) =====
# =============================================
# Campos que la ESP32 LoRa V3 manda en cada trama (ver §5 del MANIFEST).
ENV_FIELDS = ("accelX", "accelY", "accelZ", "gyroX", "gyroY", "gyroZ",
              "co2", "tempSCD", "humidity")

# Cada cuánto se refresca lo que mira una persona. El nodo manda a 20 Hz para
# el EKF; el dashboard no necesita ni la décima parte.
DASHBOARD_PERIODO_S = 0.5
_ultimo_envio_dashboard = 0.0


def _unit(value, lo: float = -1.0, hi: float = 1.0) -> float:
    """Fracción del mando manual, saneada. Un cliente WebSocket es entrada
    no confiable: un `null`, un texto o un 99 no pueden convertirse en un
    comando de motores, así que lo que no sea un número usable vale 0."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 0.0
    if v != v:          # NaN: cualquier comparación con NaN da False, así
        return 0.0      # que el clamp de abajo lo dejaría pasar entero
    return max(lo, min(hi, v))


def normalize_env(payload: dict) -> dict:
    """
    Garantiza que TODO campo esperado exista como float finito. Si un sensor
    no llegó a inicializarse (I2C sin responder, cable flojo) el campo puede
    venir ausente, en null, o como NaN/inf; en cualquiera de esos casos se
    entrega `0.0`, que es lo pedido para que nada aguas abajo se rompa —
    el filtro de pose, el EKF y el dashboard siempre reciben un número.

    Pero 0.0 por "sensor caído" y 0.0 por "medición real de cero" NO son lo
    mismo, y confundirlos sería inventar datos. Por eso se agrega
    `_sensors_missing`: la lista de campos que fueron sustituidos. La UI
    muestra el 0.0 marcado como no disponible en vez de graficarlo como si
    fuera una lectura buena.
    """
    missing = []
    for field in ENV_FIELDS:
        value = payload.get(field)
        try:
            number = float(value)
        except (TypeError, ValueError):
            number = None
        if number is None or not math.isfinite(number):
            payload[field] = 0.0
            missing.append(field)
        else:
            payload[field] = number
    payload["_sensors_missing"] = missing
    return payload


class EnvTelemetryProtocol(asyncio.DatagramProtocol):
    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        try:
            payload = json.loads(data.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return
        if not isinstance(payload, dict):
            return

        # Comando bajado por LoRaWAN y reenviado por el nodo LoRa V3. Llega
        # por el MISMO puerto que la telemetría porque el nodo ya tenía ese
        # socket abierto: abrir un segundo puerto solo para esto habría
        # añadido una pieza más que puede quedar mal configurada, para un
        # mensaje que llega como mucho una vez cada media hora.
        #
        # Es la vía de último recurso: cuando el robot está fuera del alcance
        # del AP, el enlace LoRa es lo único que queda. Por eso la parada de
        # emergencia se ejecuta aquí directamente en vez de pasar por el
        # dashboard, que en ese escenario no tiene a nadie mirándolo.
        if payload.get("type") == "lora_downlink":
            comando = payload.get("cmd")
            print(f"[lora] comando por downlink desde {addr[0]}: {comando}")
            if comando in ("emergency_stop", "park"):
                # `actuator_proto` es None hasta que el socket del C3 se abre
                # en el arranque. Sin esta guarda, un downlink que llegara
                # antes (o con el socket caído) lanzaría AttributeError
                # DENTRO del callback de UDP, que asyncio se traga: la
                # parada fallaría sin dejar rastro. Justo el caso en el que
                # menos se puede fallar en silencio.
                if actuator_proto is None:
                    print("[lora] ✗ PARADA NO EJECUTADA: el enlace con el "
                          "ESP32-C3 todavía no está abierto")
                    return
                actuator_proto.emergency_stop()
                _lanzar(state.broadcast({
                    "type": "emergency_stop",
                    "data": {"origen": "lora_downlink", "comando": comando},
                }))
                # Se avisa al gateway para que lo publique en Telegram: una
                # parada disparada por radio es justo el evento que el
                # operador NO está viendo en pantalla.
                _lanzar(_avisar_al_gateway({
                    "titulo": "Parada de emergencia por LoRa",
                    "detalle": f"El robot ejecutó '{comando}' recibido por "
                               f"downlink LoRaWAN.",
                    "tipo": "sistema",
                }))
            return

        payload = normalize_env(payload)
        # Corrección de bias del giroscopio (calibración). Se aplica ACÁ, lo
        # más cerca posible de la entrada, para que todo lo de aguas abajo
        # (pose casera, EKF de ROS, dashboard) vea el mismo dato corregido.
        # Antes estos offsets se guardaban en la config y no se aplicaban en
        # ningún lado: calibrar desde el dashboard no hacía absolutamente nada.
        cal = state.config["calibration"]
        for axis, bias_key in (("gyroX", "gyro_bias_x_dps"),
                               ("gyroY", "gyro_bias_y_dps"),
                               ("gyroZ", "gyro_bias_z_dps")):
            if axis in payload:
                payload[axis] = payload[axis] - cal[bias_key]

        payload["_received_at"] = time.time()
        payload["_source"] = addr[0]
        state.env_telemetry = payload
        health.beat("lora_v3")

        # --- CAMINO RÁPIDO: todo lo que consume el estimador de pose ---
        # Va a la frecuencia completa del nodo (20 Hz desde 2026-09-06), que
        # es justo la `frequency: 20.0` del EKF en ekf.yaml.
        pose_estimator.on_gyro(payload.get("gyroZ", 0.0))
        ros_bridge.publish_imu(
            {"x": payload.get("accelX", 0.0), "y": payload.get("accelY", 0.0), "z": payload.get("accelZ", 0.0)},
            {"x": payload.get("gyroX", 0.0), "y": payload.get("gyroY", 0.0), "z": payload.get("gyroZ", 0.0)},
        )

        # --- CAMINO LENTO: lo que solo mira una persona ---
        # El dashboard y el histórico se limitan a 2 Hz a propósito. A 20 Hz
        # serían 20 mensajes WebSocket por segundo a cada navegador —más
        # rápido de lo que un ojo distingue y suficiente para que el gráfico
        # se atragante— y `push_env_history()` reconstruye su lista entera en
        # cada llamada, así que a 20 Hz pasa a ser trabajo O(n) 20 veces por
        # segundo sobre 2400 puntos.
        #
        # El dato de navegación NO se toca: ese ya salió arriba, entero.
        global _ultimo_envio_dashboard
        ahora = payload["_received_at"]
        if ahora - _ultimo_envio_dashboard >= DASHBOARD_PERIODO_S:
            _ultimo_envio_dashboard = ahora
            state.push_env_history(payload)
            _lanzar(state.broadcast({"type": "env_telemetry", "data": payload}), "env_telemetry")


# =============================================
# ===== ACTUADOR (ESP32-C3 Mini, por UART) =====
# =============================================
# Esta clase conserva TODA la logica de seguridad que tenia el camino UDP
# —rampa, tope por LIDAR, odometria, calibracion del servo— y solo cambia el
# transporte por debajo: en vez de un datagrama al 192.168.12.50 escribe en
# un puerto serie. Ver uart_link.py para por que se dejaron el WiFi y el BLE.
class ActuatorController:
    def __init__(self, link: UartActuatorLink):
        self.link = link
        self._ramp_v = 0.0
        self._ramp_omega = 0.0
        self._ramp_last_t = time.monotonic()
        # Velocidad PEDIDA, distinta de la aplicada (`_ramp_*`). La rampa
        # avanza hacia este objetivo en cada `tick()`, llegue o no un mensaje.
        self._obj_v = 0.0
        self._obj_omega = 0.0
        self._obj_t = 0.0
        self._ultimo_aviso_t = 0.0

    def _apply_ramp(self, v_target: float, omega_target: float,
                    safety_cfg: dict) -> tuple[float, float]:
        """
        Limitador de rampa (slew-rate), NO es PID: no hay encoder que mida
        la velocidad real del robot, así que no hay error que corregir en
        lazo cerrado. Esto solo evita saltos bruscos entre el comando
        anterior y el nuevo, limitando cuánto puede cambiar v/omega por
        segundo (`safety.max_linear_accel_mps2` / `max_angular_accel_rps2`).

        El tope de `dt` va atado al periodo nominal de comando (CMD_SEND_HZ)
        para que un hueco largo no se traduzca en un salto grande. La rampa ya
        no depende de que lleguen mensajes: la hace avanzar `tick()` a 20 Hz.
        """
        now = time.monotonic()
        dt = max(0.0, min(now - self._ramp_last_t, RAMP_MAX_DT))
        self._ramp_last_t = now

        max_dv = safety_cfg["max_linear_accel_mps2"] * dt
        max_domega = safety_cfg["max_angular_accel_rps2"] * dt

        dv = max(-max_dv, min(max_dv, v_target - self._ramp_v))
        domega = max(-max_domega, min(max_domega, omega_target - self._ramp_omega))

        self._ramp_v += dv
        self._ramp_omega += domega
        return self._ramp_v, self._ramp_omega

    def on_status(self, status: dict):
        """Llega desde una trama del C3 por el serie, no desde un socket."""
        state.actuator_status = status
        health.beat("esp32c3")
        _lanzar(state.broadcast({"type": "actuator_status", "data": status}), "actuator_status")

    def send_command(self, v_ref: float, omega_ref: float, scan_cmd: int):
        """Fija la velocidad pedida y aplica un paso de rampa.

        Quien llama (Nav2, el conductor por pulsos, la patrulla) solo dice
        a dónde quiere ir; llegar ahí lo resuelve `tick()`, que sigue
        empujando la rampa aunque no vuelva a llegar ningún mensaje.
        """
        self._obj_v, self._obj_omega = v_ref, omega_ref
        self._obj_t = time.monotonic()
        self._aplicar(scan_cmd)

    def tick(self):
        """Avanza la rampa hacia el objetivo. Se llama a 20 Hz.

        Arregla un fallo real, medido el 2026-09-16: la rampa solo avanzaba
        al llegar un mensaje. Al soltar un botón llegaba UN único (0, 0),
        que la bajaba 0,24 m/s, y `uart_link` reenviaba al C3 el valor
        residual a 20 Hz para siempre: soltando a 0,90 m/s el robot seguía
        a 0,66 m/s sin nadie al mando. El watchdog de 300 ms del firmware no
        lo salvaba porque la trama nunca dejaba de llegar.
        """
        if time.monotonic() - self._obj_t > DEADMAN_S:
            self._obj_v = self._obj_omega = 0.0
        en_reposo = (self._ramp_v == 0.0 and self._ramp_omega == 0.0
                     and self._obj_v == 0.0 and self._obj_omega == 0.0)
        if not en_reposo:
            self._aplicar(0)

    def en_reposo(self) -> bool:
        return self._ramp_v == 0.0 and self._ramp_omega == 0.0

    def _aplicar(self, scan_cmd: int):
        v_ref, omega_ref = self._apply_ramp(
            self._obj_v, self._obj_omega, state.config["safety"])

        # Frenado por LIDAR. Vale para todo lo que mueve el robot: pulsos
        # manuales, Nav2 y la patrulla. (Aquí había una rama previa para
        # ControladorEvasion que nunca se ejecutaba: exigía un atributo
        # `_lidar_scan` que no existe en ningún archivo.)
        v_seg, aviso = lidar_reactive.limitar_velocidad(v_ref, state.config["slam"])
        if v_seg != v_ref:
            # La rampa sigue desde lo realmente aplicado. Si no, al
            # despejarse el obstáculo el robot saltaba de 0 a la velocidad
            # pedida sin rampa: un scan con un hueco bastaba para un tirón.
            self._ramp_v = v_seg
            ahora = time.monotonic()
            if aviso is not None and ahora - self._ultimo_aviso_t > 0.25:
                self._ultimo_aviso_t = ahora
                _lanzar(state.broadcast({"type": "obstacle_warning", "data": aviso}), "obstacle_warning")
            v_ref = v_seg

        pose_estimator.on_command(v_ref, omega_ref)
        ros_bridge.publish_wheel_odom(v_ref, omega_ref)
        self.link.set_actuators(self._servo_angle_with_offset(), state.led_on)
        self.link.send_command(v_ref, omega_ref, scan_cmd)

    def send_actuator_update(self):
        """Manda servo/LED sin tocar el movimiento: reusa la velocidad que
        la rampa ya tiene aplicada, en vez de forzar 0 (si el robot va en
        marcha y el operador toca el LED, no debe dar un tirón)."""
        self.link.set_actuators(self._servo_angle_with_offset(), state.led_on)
        self.link.send_command(self._ramp_v, self._ramp_omega, 0)

    def _servo_angle_with_offset(self) -> int:
        """Aplica el offset de calibración al ángulo pedido. Antes este
        offset se guardaba en la config y NO se usaba en ningún lado — la
        pestaña de calibración del servo era puramente decorativa."""
        return state.servo_angle + state.config["calibration"]["servo_offset_deg"]

    def emergency_stop(self):
        """Parada real e inmediata: NO pasa por la rampa de suavizado (que
        frenaría gradual) — resetea la rampa a 0 y manda el paquete de una.
        También apaga el LED: el estado deseado de los actuadores auxiliares
        se resetea para que al soltar la emergencia nada se reactive solo."""
        self._ramp_v = 0.0
        self._ramp_omega = 0.0
        # Sin esto `tick()` volvería a acelerar hacia la velocidad de antes.
        self._obj_v = 0.0
        self._obj_omega = 0.0
        self._ramp_last_t = time.monotonic()
        state.led_on = False
        pose_estimator.on_command(0.0, 0.0)
        ros_bridge.publish_wheel_odom(0.0, 0.0)
        self.link.emergency_stop()
        self.link.set_actuators(self._servo_angle_with_offset(), False)


# =============================================
# ===== PUENTE ROS 2 (opcional — ver ros_bridge.py) =====
# =============================================
# Estos callbacks corren en el hilo del executor de rclpy (NO en el loop
# de asyncio de aiohttp), así que cualquier cosa que toque `state` o mande
# WebSocket tiene que cruzar al loop principal con run_coroutine_threadsafe.
_main_loop: asyncio.AbstractEventLoop | None = None


def _schedule(coro):
    """Cruza del hilo del executor de rclpy al loop de asyncio.

    El `close()` importa: el puente ROS arranca al importar el módulo, ANTES
    de que `start_background` fije `_main_loop`. Los scans que llegan en esa
    ventana creaban una corrutina que nadie ejecutaba nunca, y Python la
    denunciaba con "coroutine ... was never awaited" en cada arranque.
    Descartarla explícitamente deja claro que es intencional y libera lo que
    tenga agarrado."""
    if _main_loop is None:
        coro.close()
        return
    asyncio.run_coroutine_threadsafe(coro, _main_loop)


def _ros_on_cmd_vel(v_ref: float, omega_ref: float):
    """Nav2 publicó /cmd_vel — se manda por el mismo UDP binario de siempre.
    Solo (v, omega): el robot es diferencial, no hay componente lateral."""
    if actuator_proto is not None:
        actuator_proto.send_command(v_ref, omega_ref, 0)


# El radar del dashboard mide ~300 px de lado: más de ~200 puntos caen en el
# mismo píxel y no aportan nada visible. Un scan completo del C1 son ~600
# puntos = 30 KB de JSON, 10 veces por segundo y POR CLIENTE (300 KB/s).
# Se diezma SOLO lo que se difunde; la evasión reactiva sigue recibiendo el
# scan completo, que es donde la resolución sí importa: perder el único rayo
# que da contra una rama fina es perder el frenado.
RADAR_MAX_PUNTOS = 200


def _decimar(points: list[dict], maximo: int) -> list[dict]:
    """Diezma y REDONDEA para la difusión.

    El scan entra con precisión de float64 —`0.14875000715255737` m,
    `-180.00000500895632`°— que en JSON son ~40 caracteres por punto de puro
    ruido: el C1 tiene un error de ±3 cm, así que todo lo que hay más allá
    del milímetro es dígito inventado. Redondear a mm y a centésimas de
    grado no pierde nada real y achica el mensaje a la mitad."""
    if len(points) > maximo:
        paso = len(points) / maximo
        points = [points[int(i * paso)] for i in range(maximo)]
    return [{"angle_deg": round(p["angle_deg"], 2),
             "distance_m": round(p["distance_m"], 3)} for p in points]


def _ros_on_scan_points(points: list[dict]):
    """sllidar_ros2 publicó /scan — alimenta la evasión reactiva + el radar."""
    lidar_reactive.ingest_scan(points)
    health.beat("lidar")
    _schedule(state.broadcast({"type": "lidar_scan",
                               "data": _decimar(points, RADAR_MAX_PUNTOS)}))
    cfg = state.config["slam"]
    _schedule(state.broadcast({"type": "lidar_distancias", "data": {
        **lidar_reactive.distancias(cfg),
        "margen_m": cfg["obstacle_margen_m"],
    }}))


def _ros_on_fused_pose(pose: dict):
    """robot_localization publicó /odometry/filtered — reemplaza al PoseEstimator casero."""
    global ros_fused_pose
    ros_fused_pose = pose
    _schedule(state.broadcast({"type": "pose", "data": pose, "source": "ros_ekf"}))


_last_map_broadcast = 0.0


def _ros_on_map(map_dict: dict):
    """slam_toolbox publicó /map — se manda al dashboard, limitado a 1 Hz (es pesado)."""
    global ros_last_map, _last_map_broadcast
    health.beat("slam")
    ros_last_map = map_dict
    now = time.time()
    if now - _last_map_broadcast < 1.0:
        return
    _last_map_broadcast = now
    _schedule(state.broadcast({"type": "map", "data": map_dict}))


ros_bridge = AgroBotROSBridge(_ros_on_cmd_vel, _ros_on_scan_points, _ros_on_fused_pose, _ros_on_map)

# Se crea en start_background, cuando ya existe el event loop al que el hilo
# de captura tiene que despachar los frames.
camera: CameraWorker | None = None


def current_pose() -> dict:
    """Pose fusionada real (EKF de robot_localization) si ROS está corriendo
    y ya publicó al menos una vez; si no, cae al filtro complementario
    casero de slam.py para que el dashboard nunca se quede sin lectura."""
    return ros_fused_pose if ros_fused_pose is not None else pose_estimator.snapshot()


# =============================================
# ===== WEBSOCKET =====
# =============================================
async def ws_handler(request: web.Request):
    ws = web.WebSocketResponse(heartbeat=10)
    await ws.prepare(request)
    state.ws_clients.add(ws)

    await ws.send_str(json.dumps({
        "type": "snapshot",
        "data": {
            "env_telemetry": state.env_telemetry,
            "env_history": state.env_history,
            "actuator_status": state.actuator_status,
            "detections": state.detections[-20:],
            "captures": list(reversed(state.captures[-40:])),
            "system_stats": state.system_stats,
            "mission_active": state.mission_active,
            "config": state.config,
            "models": list_model_files(),
            "health": health.snapshot(state.config["safety"]["heartbeat_stale_warn_s"],
                                       state.config["safety"]["heartbeat_stale_err_s"]),
            "pose": current_pose(),
            "pose_source": "ros_ekf" if ros_fused_pose is not None else "dead_reckoning",
            "lidar_scan": _decimar(lidar_reactive.last_scan or [], RADAR_MAX_PUNTOS),
            "map": ros_last_map,
            "ros_available": ros_bridge.available,
            "ros_reason": ROS_UNAVAILABLE_REASON,
            "navigation": state.config["navigation"],
            "actuator_setpoint": {"servo_angle": state.servo_angle, "led": state.led_on},
        },
    }))

    last_cmd_sent = 0.0
    min_interval = 1.0 / CMD_SEND_HZ

    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                cmd = json.loads(msg.data)
            except json.JSONDecodeError:
                continue

            if cmd.get("type") == "control":
                now = time.monotonic()
                # Un "parar" nunca se descarta por el limitador: antes, si
                # llegaba a menos de 50 ms del último comando (un toque corto
                # en el móvil), se tiraba y el robot seguía con el anterior.
                es_parada = not cmd.get("v") and not cmd.get("omega")
                if now - last_cmd_sent < min_interval and not es_parada:
                    continue
                last_cmd_sent = now
                # El mando manual manda FRACCIÓN (-1..1), no m/s: el
                # dashboard no tiene por qué conocer los límites físicos del
                # robot, y así el acelerador (`scale`) es una sola
                # multiplicación acá en vez de tres cuentas en el navegador.
                # Antes se mandaba v=1 crudo y el firmware lo recortaba a
                # V_MAX en silencio: la UI decía 1.0 m/s y el robot iba a
                # 0.55, y no había forma de pedir marcha lenta.
                ctrl = state.config["control"]
                scale = _unit(cmd.get("scale", 1.0), lo=0.0)
                v = _unit(cmd.get("v", 0.0)) * ctrl["max_v_mps"] * scale
                omega = _unit(cmd.get("omega", 0.0)) * ctrl["max_omega_rps"] * scale
                scan = int(bool(cmd.get("scan", False)))
                actuator_proto.send_command(v, omega, scan)

            elif cmd.get("type") == "actuator":
                # Servo / LED. Va por su propio mensaje y NO pasa por el
                # throttle de `control`: el operador puede mover el slider
                # con el robot detenido, y en ese caso no hay comandos de
                # movimiento saliendo que arrastren el cambio.
                if "servo_angle" in cmd:
                    state.servo_angle = clamp_servo(float(cmd["servo_angle"]))
                if "led" in cmd:
                    state.led_on = not bool(cmd["led"])  # INVERTIDA: 1→0, 0→1
                actuator_proto.send_actuator_update()
                await state.broadcast({"type": "actuator_setpoint", "data": {
                    "servo_angle": state.servo_angle,
                    "led": state.led_on,
                }})

            elif cmd.get("type") == "emergency_stop":
                state.mission_active = False
                # La patrulla y los pulsos primero: si no, vuelven a mandar
                # velocidad (o una meta a Nav2) justo después de la parada.
                if patrol is not None:
                    patrol.detener()
                if conductor is not None:
                    conductor.parar()
                actuator_proto.emergency_stop()
                await state.broadcast({"type": "toast", "data": {
                    "kind": "error", "title": "PARADA DE EMERGENCIA",
                    "message": "Motores detenidos por el operador.",
                }})
                await state.broadcast({"type": "mission_state", "data": {"active": False}})

    finally:
        state.ws_clients.discard(ws)

    return ws


# =============================================
# ===== VIDEO EN VIVO + DETECCIÓN YOLO =====
# =============================================
async def video_handler(request: web.Request):
    """Sirve el MJPEG. No toca la cámara: solo consume lo que el hilo de
    captura ya dejó listo (ver camera.py). Antes este handler abría su
    propio `VideoCapture(0)` por petición —dos pestañas del dashboard eran
    dos capturas peleando por la misma cámara— y hacía la lectura y la
    compresión bloqueando el event loop."""
    response = web.StreamResponse(
        status=200,
        headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            # Sin esto algunos navegadores y proxies cachean el stream y el
            # video queda congelado en el primer frame.
            "Cache-Control": "no-store, no-cache, must-revalidate",
        },
    )
    await response.prepare(request)

    if camera is None:
        return response

    cola = camera.subscribe()
    try:
        while True:
            jpeg = await cola.get()
            await response.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                                 + jpeg + b"\r\n")
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    finally:
        camera.unsubscribe(cola)

    return response


# =============================================
# ===== MODO AUTÓNOMO =====
# =============================================
patrol: PatrolController | None = None
conductor: ConductorPulsos | None = None

WAYPOINTS_PATH = _AQUI / "patrol_waypoints.json"


odom_webcam: OdometriaWebcam | None = None


def _servo_al_frente() -> tuple[bool, str]:
    """¿La cámara mira al frente y quieta? Si no, la odometría visual leería
    el giro del servo como giro del robot. Se mira lo que REPORTA el C3, no lo
    que se le pidió: durante un barrido el firmware ignora el ángulo pedido."""
    st = state.actuator_status or {}
    if not st or time.time() - st.get("_received_at", 0) > 1.0:
        return False, "sin estado del servo (¿C3 desconectado?)"
    if st.get("scanning"):
        return False, "cámara escaneando"
    angulo = st.get("servo_angle")
    frentes = {SERVO_HOME_DEG, SERVO_HOME_DEG + state.config["calibration"]["servo_offset_deg"]}
    if angulo is None or min(abs(angulo - f) for f in frentes) > SERVO_TOLERANCIA_DEG:
        return False, f"cámara girada ({angulo}°)"
    return True, ""


async def _bucle_rampa():
    """Hace avanzar la rampa del actuador a 20 Hz, llegue o no un comando."""
    while True:
        try:
            if actuator_proto is not None:
                actuator_proto.tick()
        except Exception as exc:
            print(f"[rampa] tick falló: {type(exc).__name__}: {exc}")
        await asyncio.sleep(1.0 / CMD_SEND_HZ)


# =============================================
# ===== MANEJO MANUAL POR PULSOS =====
# =============================================
async def drive_handler(request: web.Request):
    """POST /api/drive  {"accion": "adelante|atras|izquierda|derecha|parar",
                         "escala": 0.1..1, "maniobra": true}

    HTTP y no WebSocket a propósito: en el móvil el WebSocket se cae al
    bloquear la pantalla y `sendControl` descartaba el mensaje en silencio
    si no estaba abierto. Una petición HTTP o llega o devuelve error."""
    try:
        body = await request.json()
    except Exception:
        body = {}
    accion = body.get("accion", "")

    if accion == "parar":
        # Parar lo para TODO, patrulla incluida: es la tecla de pánico.
        if patrol is not None and patrol.activo:
            patrol.detener()
            state.mission_active = False
            await state.broadcast({"type": "mission_state", "data": {"active": False}})
        conductor.parar()
        return web.json_response({"ok": True, "accion": "parar"})

    if accion not in ACCIONES:
        return web.json_response({"error": f"acción desconocida: {accion!r}"}, status=400)
    if patrol is not None and patrol.activo:
        return web.json_response(
            {"error": "la patrulla autónoma está en marcha — detenela para manejar a mano"},
            status=409)

    ctrl = state.config["control"]
    escala = _unit(body.get("escala", 1.0), lo=0.0)
    conductor.ordenar(accion,
                      v_max=ctrl["max_v_mps"] * escala,
                      omega_max=ctrl["max_omega_rps"] * escala,
                      maniobra=bool(body.get("maniobra", True)))
    return web.json_response({"ok": True, "accion": accion})


def _patrol_scan_now():
    """Robot quieto y flag de barrido arriba.

    El firmware solo arranca el barrido en el flanco (`if (FLAG_SCAN &&
    !scanning)`), así que repetir la petición durante un barrido en curso no
    lo reinicia ni lo corta."""
    if actuator_proto is not None:
        actuator_proto.send_command(0.0, 0.0, 1)


def _patrol_is_scanning() -> bool:
    return bool((state.actuator_status or {}).get("scanning", False))


def _patrol_event(data: dict):
    _lanzar(state.broadcast({"type": "patrol_event", "data": data}), "patrulla")


async def _cargar_modelo_si_falta(app: web.Application):
    """Deja el YOLO activo cargado antes de soltar al robot solo."""
    if camera is None or app.get("yolo_model") is not None:
        return
    try:
        active_model = state.config.get("ai", {}).get("active_model", "")
        if not active_model:
            return
        model_path = MODELS_DIR / active_model
        if model_path.exists():
            modelo = await asyncio.to_thread(YOLO, str(model_path))
            app["yolo_model"] = modelo
            camera.set_model(modelo)
    except Exception as e:
        print(f"[AUTO] Error cargando modelo: {e}")


# =============================================
# ===== PATRULLA (recorrido por waypoints) =====
# =============================================
async def patrol_status_handler(request: web.Request):
    if patrol is None:
        return web.json_response({"error": "patrulla no inicializada"}, status=503)
    return web.json_response(patrol.snapshot())


async def patrol_record_handler(request: web.Request):
    """Graba el punto donde está el robot ahora mismo.

    Es la vía prevista para armar el recorrido: se conduce el robot a mano
    hasta cada planta o cabecera de surco y se pulsa grabar."""
    if patrol is None:
        return web.json_response({"error": "patrulla no inicializada"}, status=503)

    wp = patrol.grabar_pose_actual()
    if wp is None:
        return web.json_response(
            {"error": "sin pose todavía — ¿está corriendo el SLAM?"}, status=409)

    await state.broadcast({"type": "patrol_event",
                           "data": {"evento": "waypoint_grabado", "waypoint": wp}})
    return web.json_response({"ok": True, "waypoint": wp,
                              "total": len(patrol.waypoints)})


async def patrol_clear_handler(request: web.Request):
    if patrol is None:
        return web.json_response({"error": "patrulla no inicializada"}, status=503)
    if patrol.activo:
        return web.json_response(
            {"error": "no se borra el recorrido con la patrulla corriendo"}, status=409)
    patrol.borrar_waypoints()
    return web.json_response({"ok": True})


async def patrol_scan_handler(request: web.Request):
    """Barrido manual bajo demanda, sin esperar a los 2 minutos."""
    if patrol is None:
        return web.json_response({"error": "patrulla no inicializada"}, status=503)
    asyncio.create_task(patrol.escanear("manual"))
    return web.json_response({"ok": True})

# =============================================
# ===== GRABACIÓN DE VIDEO =====
# =============================================
video_recording = False
video_writer = None
video_frames_queue = None
VIDEO_DIR = _AQUI / "videos"
VIDEO_DIR.mkdir(exist_ok=True)

async def video_record_handler(request: web.Request):
    """Inicia o detiene la grabación de video desde la cámara."""
    global video_recording, video_writer, video_frames_queue

    body = await request.json()
    action = body.get("action", "start")

    if action == "start":
        if video_recording:
            return web.json_response({"error": "ya hay una grabación en curso"}, status=400)

        # crear archivo de salida
        timestamp = int(time.time())
        filename = f"agrobot_video_{timestamp}.mp4"
        video_path = VIDEO_DIR / filename

        # inicializar video writer con los parámetros de la cámara
        if camera is not None:
            # usar dimensiones estándar (1920x1080 o lo que tenga la cámara)
            w, h, fps = 1920, 1080, 30
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(str(video_path), fourcc, fps, (w, h))
            video_recording = True
            video_frames_queue = asyncio.Queue()

            print(f"[VIDEO] Iniciando grabación: {filename}")
            return web.json_response({"ok": True, "file": filename})
        else:
            return web.json_response({"error": "cámara no disponible"}, status=500)

    elif action == "stop":
        if not video_recording:
            return web.json_response({"error": "no hay grabación en curso"}, status=400)

        video_recording = False
        if video_writer is not None:
            video_writer.release()
            video_writer = None

        # obtener el nombre del archivo desde el path
        timestamp = int(time.time())
        filename = f"agrobot_video_{timestamp - 60}.mp4"  # aproximado

        print(f"[VIDEO] Grabación detenida: {filename}")
        return web.json_response({"ok": True, "file": filename})

    else:
        return web.json_response({"error": "acción inválida"}, status=400)


async def _avisar_al_gateway(payload: dict):
    """Empuja una alarma al gateway de la RPi para que la publique en Telegram.

    Cualquier fallo (gateway apagado, LAN caída, timeout) se traga a
    propósito: el aviso remoto es un extra, y un `raise` acá cancelaría la
    tarea que además guarda la captura en disco.
    """
    global _sesion_http
    try:
        if _sesion_http is None or _sesion_http.closed:
            _sesion_http = aiohttp.ClientSession(
                timeout=aiohttp.ClientTimeout(total=GATEWAY_ALERT_TIMEOUT_S))
        async with _sesion_http.post(GATEWAY_ALERT_URL, json=payload) as resp:
            await resp.read()
    except Exception as exc:
        print(f"[alerta] no se pudo avisar al gateway ({type(exc).__name__}) — "
              f"la detección igual quedó guardada en la galería")


def _on_detection(detection: dict, frame=None):
    """Llega desde el hilo de inferencia, ya en el event loop."""
    state.detections.append(detection)
    if len(state.detections) > 200:
        state.detections.pop(0)
    _lanzar(state.broadcast({"type": "detection", "data": detection}), "deteccion")

    # Detectar una planta dispara el barrido de cámara, con patrulla o sin
    # ella. El propio controlador aplica la ventana anti-repetición: YOLO
    # detecta la misma planta varias veces por segundo y sin ese freno el
    # robot se quedaría barriendo delante del primer arbusto que viera.
    if patrol is not None:
        patrol.solicitar_escaneo_por_deteccion()

    if frame is not None:
        _lanzar(state.broadcast({"type": "toast", "data": {
            "kind": "warning", "title": "Plaga detectada",
            "message": f"{detection['class']} · {detection['confidence']*100:.0f}% de confianza",
        }}), "aviso")
        _lanzar(save_capture(frame, detection["class"],
                             detection["confidence"]), "captura")
        # Solo se avisa al operador de lo que superó el umbral de "grabar
        # clip" (el mismo que dispara la captura), no de cada caja que pinta
        # el modelo: con el umbral bajo de visualización el chat sería ruido.
        _lanzar(_avisar_al_gateway({
            "tipo": "plaga",
            "clase": detection["class"],
            "confianza": detection["confidence"],
            "origen": "jetson-yolo",
        }), "gateway")


# =============================================
# ===== GALERÍA DE MISIÓN (capturas de plagas) =====
# =============================================
# Cuando una detección supera el umbral de "grabar clip", la foto real de
# la cámara (no un placeholder) se guarda en disco y queda disponible para
# que el dashboard la descargue — esta es la parte de "el robot llega y
# descarga lo que encontró" que antes faltaba: antes solo había un log de
# texto, ahora cada hallazgo trae su evidencia fotográfica.
async def save_capture(frame, cls_name: str, conf: float):
    import cv2

    ts = time.time()
    cap_id = f"{int(ts * 1000)}"
    filename = f"{cap_id}_{cls_name}.jpg"

    def _comprimir_y_guardar():
        ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            return False
        (CAPTURES_DIR / filename).write_bytes(jpeg.tobytes())
        return True

    # Comprimir a JPEG de calidad 90 y escribir a disco son dos operaciones
    # bloqueantes; en el event loop frenarían los comandos al robot justo en
    # el momento en que se detecta algo, que es cuando menos conviene.
    if not await asyncio.get_running_loop().run_in_executor(None, _comprimir_y_guardar):
        return

    capture = {
        "id": cap_id,
        "class": cls_name,
        "confidence": conf,
        "ts": ts,
        "image_url": f"/captures/{filename}",
    }
    state.captures.append(capture)
    if len(state.captures) > 500:
        oldest = state.captures.pop(0)
        old_path = CAPTURES_DIR / oldest["image_url"].split("/")[-1]
        old_path.unlink(missing_ok=True)

    await state.broadcast({"type": "capture", "data": capture})


async def captures_list_handler(request: web.Request):
    limit = int(request.query.get("limit", "60"))
    cls_filter = request.query.get("class")
    items = state.captures
    if cls_filter:
        items = [c for c in items if c["class"] == cls_filter]
    return web.json_response(list(reversed(items))[:limit])


async def captures_clear_handler(request: web.Request):
    for c in state.captures:
        (CAPTURES_DIR / c["image_url"].split("/")[-1]).unlink(missing_ok=True)
    state.captures = []
    await state.broadcast({"type": "captures_cleared", "data": {}})
    return web.json_response({"ok": True})


# =============================================
# ===== GESTOR DE MODELOS =====
# =============================================
def list_model_files() -> list[dict]:
    out = []
    for ext in ("*.engine", "*.pt", "*.onnx"):
        for p in MODELS_DIR.glob(ext):
            out.append({
                "name": p.name,
                "size_mb": round(p.stat().st_size / (1024 * 1024), 1),
                "active": p.name == state.config["ai"]["active_model"],
            })
    return out


async def models_list_handler(request: web.Request):
    return web.json_response(list_model_files())


async def models_load_handler(request: web.Request):
    body = await request.json()
    name = body.get("name")

    # DESCARGAR EL MODELO: `name` vacío o nulo deja el video sin detección.
    if name in (None, "", "none"):
        request.app["yolo_model"] = None
        if camera is not None:
            camera.set_model(None)
        config_store.update_section("ai", {"active_model": ""})
        state.config = config_store.load()
        await state.broadcast({"type": "toast", "data": {
            "kind": "info", "title": "Modelo descargado", "message": "Video sin detección",
        }})
        return web.json_response({"ok": True, "active_model": ""})

    if not isinstance(name, str) or not name or "/" in name or "\\" in name or name.startswith("."):
        return web.json_response({"error": "Nombre de modelo inválido"}, status=400)

    model_path = MODELS_DIR / name
    if not model_path.exists():
        print(f"[YOLO] Modelo no encontrado: {model_path} (MODELS_DIR={MODELS_DIR})")
        disponibles = list_model_files()
        return web.json_response({"error": f"Modelo '{name}' no encontrado. Disponibles: {[m['name'] for m in disponibles]}"}, status=404)

    try:
        from ultralytics import YOLO
        print(f"[YOLO] Cargando {name}...")
        modelo = await asyncio.to_thread(YOLO, str(model_path))
        print(f"[YOLO] {name} cargado OK")
    except ImportError:
        return web.json_response({"error": "Librería ultralytics no instalada. Instala: pip install ultralytics"}, status=500)
    except Exception as exc:
        err = str(exc)
        print(f"[YOLO] Error cargando {name}: {err}")
        return web.json_response({"error": f"Error al cargar modelo: {err}"}, status=400)

    # Un modelo de CLASIFICACIÓN no produce cajas: `results.boxes` viene en
    # None y el hilo de inferencia revienta en cada frame. Degradaba
    # "bien" (se recupera y reintenta) pero la detección no funcionaba nunca
    # y el motivo solo se veía en `estado`. Mejor rechazarlo acá con el
    # motivo claro. Pasó de verdad: un yolo11s-cls bajado de Kaggle.
    tarea = getattr(modelo, "task", None)
    if tarea != "detect":
        return web.json_response(
            {"error": f"el modelo es de tipo '{tarea}', y el robot necesita "
                      f"uno de detección ('detect'): un clasificador no "
                      f"produce cajas y no se puede dibujar ni ubicar la plaga"},
            status=400)

    request.app["yolo_model"] = modelo
    # Sin esto el modelo quedaba guardado en la config y el dashboard decía
    # "Modelo cargado", pero el HILO DE INFERENCIA nunca se enteraba: la
    # detección no corría nunca y no había ningún error que lo delatara.
    if camera is not None:
        camera.set_model(modelo)

    config_store.update_section("ai", {"active_model": name})
    state.config = config_store.load()
    await state.broadcast({"type": "toast", "data": {
        "kind": "success", "title": "Modelo cargado", "message": name,
    }})
    return web.json_response({"ok": True, "active_model": name})


# =============================================
# ===== HARDWARE / CALIBRACIÓN =====
# =============================================
async def hardware_get_handler(request: web.Request):
    return web.json_response(state.config["hardware"])


async def hardware_update_handler(request: web.Request):
    body = await request.json()
    data = config_store.update_section("hardware", body)
    state.config = data
    await state.broadcast({"type": "config_update", "data": state.config})
    return web.json_response(data["hardware"])


async def calibration_get_handler(request: web.Request):
    return web.json_response(state.config["calibration"])


async def calibration_update_handler(request: web.Request):
    body = await request.json()
    data = config_store.update_section("calibration", body)
    state.config = data
    await state.broadcast({"type": "toast", "data": {
        "kind": "success", "title": "Calibración guardada", "message": "Offsets escritos en EEPROM local.",
    }})
    await state.broadcast({"type": "config_update", "data": state.config})
    return web.json_response(data["calibration"])


# =============================================
# ===== PARÁMETROS (SLAM / DETECCIÓN / SEGURIDAD) =====
# =============================================
PARAM_SECTIONS = ("slam", "detection", "safety", "control", "odometria_webcam")

# Topes que un cliente NO puede sobrepasar por más que mande cualquier cosa
# al POST. La sección `control` multiplica directamente lo que sale hacia
# los motores, así que un valor absurdo (o un texto) no puede quedar
# guardado: se guardaría en disco y seguiría vigente en el próximo arranque.
CONTROL_LIMITS = {
    "max_v_mps": (0.05, 1.5),
    "max_omega_rps": (0.1, 4.0),
}


def sanitize_control(body: dict) -> dict:
    limpio = {}
    for clave, (lo, hi) in CONTROL_LIMITS.items():
        if clave not in body:
            continue
        try:
            valor = float(body[clave])
        except (TypeError, ValueError):
            continue
        if valor != valor:      # NaN
            continue
        limpio[clave] = max(lo, min(hi, valor))
    return limpio


async def params_get_handler(request: web.Request):
    return web.json_response({k: state.config[k] for k in PARAM_SECTIONS})


async def params_update_handler(request: web.Request):
    section = request.match_info["section"]
    if section not in PARAM_SECTIONS:
        return web.json_response({"error": "sección desconocida"}, status=404)
    body = await request.json()
    if section == "control":
        body = sanitize_control(body)
    data = config_store.update_section(section, body)
    state.config = data
    if section == "slam" and "pose_gyro_trust" in body:
        pose_estimator.gyro_weight = float(body["pose_gyro_trust"])
    await state.broadcast({"type": "toast", "data": {
        "kind": "success", "title": "Parámetros actualizados", "message": section,
    }})
    await state.broadcast({"type": "config_update", "data": state.config})
    return web.json_response(data[section])


# =============================================
# ===== LIDAR (ingesta de scan 2D) =====
# =============================================
async def lidar_scan_handler(request: web.Request):
    """
    Punto de enganche para quien lea el RPLIDAR C1 (por ejemplo un script
    con la librería `rplidar` corriendo aparte, o el nodo sllidar_ros2):
    POST /api/lidar/scan  con {"points": [{"angle_deg": .., "distance_m": ..}, ...]}
    """
    body = await request.json()
    points = body.get("points", [])
    lidar_reactive.ingest_scan(points)
    health.beat("lidar")

    # Procesar LIDAR para evasión de obstáculos (NUEVO 2026-09-13)
    global obstacle_controller
    if obstacle_controller and points:
        try:
            ranges = [p.get("distance_m", 10.0) for p in points]
            angles = [math.radians(p.get("angle_deg", 0)) for p in points]
            lecturas = LecturasSensoresLIDAR.desde_scan_lidar(ranges, angles, time.time())
            obstacle_controller.procesar_lidar(lecturas)
        except Exception as e:
            print(f"[EVASION] Error procesando LIDAR: {e}")

    await state.broadcast({"type": "lidar_scan", "data": points})
    return web.json_response({"ok": True, "points_received": len(points)})


async def obstacle_status_handler(request: web.Request):
    """Retorna el estado actual del sistema de evasión de obstáculos."""
    global obstacle_controller
    if obstacle_controller:
        return web.json_response(obstacle_controller.get_estado_json())
    return web.json_response({"error": "Sistema de evasión no disponible", "estado": "deshabilitado"})


# =============================================
# ===== SALUD DE TÓPICOS =====
# =============================================
async def health_handler(request: web.Request):
    cfg = state.config["safety"]
    filas = health.snapshot(cfg["heartbeat_stale_warn_s"], cfg["heartbeat_stale_err_s"])
    # ROS no tiene heartbeat (no es un nodo que mande datos periódicos), pero
    # sí es un servicio del que depende media pestaña de Navegación — así que
    # aparece en la misma lista, con el MOTIVO cuando no está.
    filas.append({
        "key": "ros",
        "label": "ROS 2 / Nav2",
        "icon": "🧭",
        "status": "ok" if ros_bridge.available else "warn",
        "age_s": None,
        "detail": "" if ros_bridge.available else ROS_UNAVAILABLE_REASON,
    })
    return web.json_response(filas)


# =============================================
# ===== NAVEGACIÓN (Nav2 vía ros_bridge.py) =====
# =============================================
async def nav_set_goal_handler(request: web.Request):
    """
    POST /api/nav/set_goal  {"x": .., "y": .., "theta_deg": ..}
    El click en el mini-mapa del dashboard llega aquí ya convertido a
    coordenadas del mundo (metros, frame "map") usando la resolución/
    origen que trae el propio mensaje /map.
    """
    if not ros_bridge.available:
        return web.json_response({"error": "ROS no disponible en este backend — instala rclpy/nav2 y corre agrobot_navigation.launch.py"}, status=503)
    body = await request.json()
    x, y = float(body.get("x", 0.0)), float(body.get("y", 0.0))
    theta = float(body.get("theta_deg", 0.0))
    ok = ros_bridge.send_nav_goal(x, y, theta)
    if ok:
        config_store.update_section("navigation", {"last_goal": {"x": x, "y": y, "theta_deg": theta}})
        state.config = config_store.load()
        await state.broadcast({"type": "toast", "data": {
            "kind": "success", "title": "Meta enviada a Nav2", "message": f"x={x:.2f} y={y:.2f}",
        }})
    else:
        await state.broadcast({"type": "toast", "data": {
            "kind": "error", "title": "Nav2 no respondió", "message": "¿Está corriendo agrobot_navigation.launch.py?",
        }})
    return web.json_response({"ok": ok})


async def nav_restart_handler(request: web.Request):
    """Reinicia el servicio Nav2 (killea y relanza)"""
    try:
        # Matar nodos Nav2
        subprocess.run(["ros2", "node", "kill", "/nav2_bringup/lifecycle_manager_navigation"],
                      capture_output=True, timeout=5)
        await asyncio.sleep(2)
        await state.broadcast({"type": "toast", "data": {
            "kind": "ok", "title": "Nav2 reiniciándose", "message": "El servicio se está reiniciando. Espera 5-10 segundos.",
        }})
        return web.json_response({"ok": True})
    except Exception as e:
        print(f"[NAV2] Error reiniciando: {e}")
        await state.broadcast({"type": "toast", "data": {
            "kind": "error", "title": "Error", "message": f"No se pudo reiniciar Nav2: {str(e)}",
        }})
        return web.json_response({"error": str(e)}, status=500)


async def nav_cancel_handler(request: web.Request):
    ros_bridge.cancel_nav_goal()
    await state.broadcast({"type": "toast", "data": {
        "kind": "warning", "title": "Misión cancelada", "message": "Meta de navegación cancelada por el operador.",
    }})
    return web.json_response({"ok": True})


async def nav_save_map_handler(request: web.Request):
    """
    POST /api/nav/save_map  — cierra la sesión de mapeo: le pide a
    nav2_map_server que escriba maps/agrobot_map.{yaml,pgm} a partir del
    mapa que slam_toolbox tiene armado en memoria en ese momento. Requiere
    correr esto MIENTRAS agrobot_mapping.launch.py sigue vivo.
    """
    name = state.config["navigation"]["map_name"]
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    out_path = str(MAPS_DIR / name)
    try:
        result = subprocess.run(
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", out_path],
            capture_output=True, text=True, timeout=15,
        )
        ok = result.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        ok = False
        result = None
        err_msg = str(e)
    await state.broadcast({"type": "toast", "data": {
        "kind": "success" if ok else "error",
        "title": "Mapa guardado" if ok else "No se pudo guardar el mapa",
        "message": f"{name}.yaml" if ok else (result.stderr[-200:] if result else err_msg),
    }})
    return web.json_response({"ok": ok, "path": out_path if ok else None})


async def nav_reset_map_handler(request: web.Request):
    """POST /api/nav/reset_map — borra el mapa de slam_toolbox y empieza de cero.

    También borra el recorrido grabado: los waypoints son coordenadas del
    mapa viejo, y tras el reset el origen pasa a ser donde está el robot
    ahora. Conservarlos mandaría la patrulla a sitios equivocados."""
    if patrol is not None and patrol.activo:
        return web.json_response(
            {"ok": False, "error": "detené la patrulla antes de resetear el mapa"}, status=409)

    def _llamar():
        return subprocess.run(
            ["ros2", "service", "call", "/slam_toolbox/reset",
             "slam_toolbox/srv/Reset", "{pause_new_measurements: false}"],
            capture_output=True, text=True, timeout=20)

    try:
        res = await asyncio.to_thread(_llamar)
        ok = res.returncode == 0 and "result=0" in res.stdout
        detalle = "" if ok else (res.stderr or res.stdout)[-200:]
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        ok, detalle = False, str(exc)

    borrados = 0
    if ok and patrol is not None:
        borrados = len(patrol.waypoints)
        patrol.borrar_waypoints()

    await state.broadcast({"type": "toast", "data": {
        "kind": "success" if ok else "error",
        "title": "Mapa reseteado" if ok else "No se pudo resetear el mapa",
        "message": (f"Mapa en blanco. Se borraron {borrados} punto(s) del recorrido."
                    if ok else detalle or "¿Está corriendo slam_toolbox?"),
    }})
    return web.json_response({"ok": ok, "waypoints_borrados": borrados},
                             status=200 if ok else 502)


# =============================================
# ===== MAPAS CON NOMBRE =====
# =============================================
# Carpeta propia, distinta de MAPS_DIR (esa es la del map_saver de Nav2).
mapas = GestorMapas(_AQUI.parent.parent / "maps")


async def mapas_listar_handler(request: web.Request):
    return web.json_response({"mapas": mapas.listar()})


async def mapas_guardar_handler(request: web.Request):
    """POST /api/mapas/guardar  {"nombre": "invernadero", "sobrescribir": false}"""
    body = await request.json()
    try:
        info = await mapas.guardar(body.get("nombre", ""), patrol.waypoints if patrol else [],
                                   bool(body.get("sobrescribir", False)))
    except ErrorMapa as exc:
        existe = "ya existe" in str(exc)
        return web.json_response({"ok": False, "error": str(exc), "existe": existe},
                                 status=409 if existe else 400)
    await state.broadcast({"type": "toast", "data": {
        "kind": "success", "title": "Mapa guardado",
        "message": f"«{info['nombre']}» con {info['puntos']} punto(s) de recorrido.",
    }})
    return web.json_response({"ok": True, **info})


async def mapas_cargar_handler(request: web.Request):
    """POST /api/mapas/cargar  {"nombre": "invernadero"}

    Reemplaza el mapa en memoria de slam_toolbox y el recorrido de la
    patrulla. El robot tiene que estar en la base donde empezó ese mapa."""
    if patrol is not None and patrol.activo:
        return web.json_response(
            {"ok": False, "error": "detené la patrulla antes de cargar otro mapa"}, status=409)
    body = await request.json()
    try:
        waypoints = await mapas.cargar(body.get("nombre", ""))
    except ErrorMapa as exc:
        return web.json_response({"ok": False, "error": str(exc)}, status=400)
    if patrol is not None:
        patrol.reemplazar_waypoints(waypoints)
    await state.broadcast({"type": "toast", "data": {
        "kind": "success", "title": "Mapa cargado",
        "message": f"«{body.get('nombre')}» con {len(waypoints)} punto(s). El SLAM sigue ampliándolo.",
    }})
    return web.json_response({"ok": True, "puntos": len(waypoints)})


async def nav_mode_handler(request: web.Request):
    """GET: modo actual. POST {"mode": "mapping"|"localization"}: solo lo anota
    para la UI — relanzar los nodos ROS correctos se hace a mano (ver README),
    cambiar este valor no reinicia ningún proceso."""
    if request.method == "GET":
        return web.json_response(state.config["navigation"])
    body = await request.json()
    mode = body.get("mode")
    if mode not in ("mapping", "localization"):
        return web.json_response({"error": "mode debe ser 'mapping' o 'localization'"}, status=400)
    data = config_store.update_section("navigation", {"mode": mode})
    state.config = data
    await state.broadcast({"type": "config_update", "data": state.config})
    return web.json_response(data["navigation"])


# =============================================
# ===== MISIÓN =====
# =============================================
async def mission_start_handler(request: web.Request):
    """
    MODO AUTÓNOMO. Único punto de entrada: arranca la patrulla por los
    waypoints grabados y carga el YOLO.

    Antes esto solo levantaba una bandera y sacaba un aviso diciéndole al
    operador que fuera a mandar metas a mano — o sea que "modo autónomo"
    dejaba el robot parado. Había además un segundo par de endpoints
    (/api/autonomous/*) que hacía casi lo mismo y al que no llamaba nadie;
    se eliminaron para que exista UN solo camino.
    """
    await _cargar_modelo_si_falta(request.app)

    # Un pulso manual a medio ejecutar pelearía con Nav2 por el actuador.
    if conductor is not None and conductor.activo:
        conductor.parar()

    ok, msg = (False, "patrulla no inicializada")
    if patrol is not None:
        ok, msg = patrol.iniciar()

    state.mission_active = ok
    await state.broadcast({"type": "mission_state", "data": {"active": ok}})
    await state.broadcast({"type": "toast", "data": {
        "kind": "success" if ok else "warning",
        "title": "Modo autónomo" if ok else "No se pudo arrancar",
        "message": msg,
    }})
    print(f"[AUTO] {msg}")
    return web.json_response({"active": ok, "message": msg}, status=200 if ok else 409)


async def mission_stop_handler(request: web.Request):
    # La patrulla primero: parar el actuador antes dejaría el bucle vivo y
    # volvería a mandarle una meta a Nav2 acto seguido.
    if patrol is not None:
        patrol.detener()
    state.mission_active = False
    actuator_proto.emergency_stop()
    await state.broadcast({"type": "mission_state", "data": {"active": False}})
    return web.json_response({"active": False})


# =============================================
# ===== REPORTE DE SESIÓN =====
# =============================================
async def session_report_handler(request: web.Request):
    report = {
        "generated_at": time.time(),
        "detections_summary": _summarize_detections(state.detections),
        "captures_count": len(state.captures),
        "captures": state.captures,
        "env_last": state.env_telemetry,
    }
    out_path = SESSION_REPORT_DIR / f"session_{int(time.time())}.json"
    out_path.write_text(json.dumps(report, indent=2))
    return web.json_response(report)


def _summarize_detections(detections: list[dict]) -> dict:
    summary: dict[str, int] = {}
    for d in detections:
        summary[d["class"]] = summary.get(d["class"], 0) + 1
    return summary


# =============================================
# ===== ESTADÍSTICAS DEL SISTEMA (CPU/RAM) =====
# =============================================
async def stats_loop():
    try:
        import psutil
    except ImportError:
        psutil = None

    # jtop (jetson-stats) da GPU/temperatura reales en Jetson; si no está
    # instalado (`pip install jetson-stats`, requiere reinicio del daemon
    # jtop una vez), esos campos quedan honestamente en None.
    jtop_reader = None
    try:
        from jtop import jtop
        jtop_reader = jtop()
        jtop_reader.start()
    except Exception:
        jtop_reader = None

    while True:
        stats = {
            "cpu_pct": psutil.cpu_percent(interval=None) if psutil else None,
            "ram_pct": psutil.virtual_memory().percent if psutil else None,
            "gpu_pct": None,
            "temp_c": None,
            # FPS REALES de captura, medidos en el hilo. Sin esto no había
            # forma de saber desde la interfaz si el video iba a 30 o a 5.
            "camera": camera.snapshot() if camera is not None else None,
        }
        if jtop_reader is not None:
            try:
                stats["gpu_pct"] = jtop_reader.stats.get("GPU")
                stats["temp_c"] = jtop_reader.stats.get("Temp CPU") or jtop_reader.stats.get("Temp thermal")
            except Exception:
                pass

        state.system_stats = stats
        cfg = state.config["safety"]
        health_data = health.snapshot(cfg["heartbeat_stale_warn_s"], cfg["heartbeat_stale_err_s"])

        await state.broadcast({"type": "system_stats", "data": stats})
        if odom_webcam is not None:
            await state.broadcast({"type": "odom_webcam", "data": odom_webcam.snapshot()})
        await state.broadcast({"type": "health", "data": health_data})
        # solo emite el fallback casero si ROS todavía no ha mandado una
        # pose fusionada real — evita pisar la buena con la de dead-reckoning
        if ros_fused_pose is None:
            # La procedencia viaja CON la pose. Antes los dos caminos —el EKF
            # real y este fallback de dead-reckoning— mandaban el mismo
            # mensaje pelado y el dashboard asumía que toda pose venía del
            # EKF: con ROS caído seguía anunciando "EKF (robot_localization)"
            # sobre una pose integrada a mano, que es justo el dato que el
            # operador necesita para NO confiar en ella.
            await state.broadcast({"type": "pose",
                                   "data": pose_estimator.snapshot(),
                                   "source": "dead_reckoning"})
        await asyncio.sleep(2)


# =============================================
# ===== APP =====
# =============================================
actuator_proto: ActuatorController | None = None
uart_link: UartActuatorLink | None = None


async def _bind_udp(loop, protocol_factory, port: int, label: str):
    """
    Intenta escuchar en la IP del AP de la Jetson (192.168.12.1). Si esa IP
    no está asignada —el AP no está levantado todavía, o la Jetson está
    conectada a otra red como cliente— cae a 0.0.0.0 con un aviso claro en
    vez de reventar con un `OSError: [Errno 99]` sin contexto, que era lo
    que pasaba antes y dejaba el backend sin arrancar sin explicar por qué.
    """
    try:
        transport, proto = await loop.create_datagram_endpoint(
            protocol_factory, local_addr=(JETSON_AP_IP, port)
        )
        print(f"[UDP] {label} ↔ {JETSON_AP_IP}:{port}")
        return transport, proto
    except OSError:
        try:
            transport, proto = await loop.create_datagram_endpoint(
                protocol_factory, local_addr=("0.0.0.0", port)
            )
        except OSError as exc:
            # Si TAMBIÉN falla 0.0.0.0 el motivo ya no es "el AP no existe":
            # lo normal es que otra instancia del backend siga viva. Sin este
            # mensaje salía un traceback de asyncio de 30 líneas terminado en
            # "Address already in use" sin decir de qué puerto ni de quién.
            raise SystemExit(
                f"\n[UDP] no se pudo escuchar {label} en el puerto {port}: {exc}\n"
                f"      Suele ser otra instancia del backend todavía viva. Verificá con:\n"
                f"        ss -lunp | grep {port}\n"
                f"      y cerrala antes de volver a arrancar.\n"
            ) from exc
        print(f"[UDP] {label} ↔ 0.0.0.0:{port}")
        print(f"      ⚠️  {JETSON_AP_IP} no está asignada en esta máquina — el AP "
              f"de la Jetson no está levantado. Se escucha en todas las interfaces "
              f"para poder trabajar igual, pero los ESP32 configurados contra el AP "
              f"no van a encontrar al robot hasta que el AP exista (ver README).")
        return transport, proto


async def start_background(app: web.Application):
    global camera
    loop = asyncio.get_running_loop()
    global _main_loop
    _main_loop = loop

    await _bind_udp(loop, EnvTelemetryProtocol, LORA_UDP_PORT, "telemetría LoRa")

    global actuator_proto, uart_link
    ctrl = state.config["control"]
    hw = state.config["hardware"]
    uart_link = UartActuatorLink(
        v_max=ctrl["max_v_mps"], omega_max=ctrl["max_omega_rps"],
        port=hw["actuator_port"], baud=hw["actuator_baud"])
    actuator_proto = ActuatorController(uart_link)
    uart_link._on_status = actuator_proto.on_status
    uart_link.start()

    # Inicializar sistema de evasión de obstáculos (NUEVO 2026-09-13)
    global obstacle_controller
    obstacle_controller = ControladorEvasion(
        callback_comando=actuator_proto.send_command,
        callback_parada=actuator_proto.emergency_stop,
        callback_debug=lambda msg: print(f"[EVASION] {msg}")
    )

    global conductor
    conductor = ConductorPulsos(send_command=actuator_proto.send_command,
                                emergency_stop=actuator_proto.emergency_stop)
    app["rampa_task"] = asyncio.create_task(_bucle_rampa())

    # Patrulla autónoma. Se crea acá y no arriba porque sus callbacks
    # necesitan el `actuator_proto` que se acaba de construir.
    global patrol
    patrol = PatrolController(
        send_nav_goal=ros_bridge.send_nav_goal,
        cancel_nav_goal=ros_bridge.cancel_nav_goal,
        estado_meta=ros_bridge.estado_meta,
        # En `map`, el mismo marco en que Nav2 recibe las metas.
        get_pose=ros_bridge.pose_en_mapa,
        trigger_scan=_patrol_scan_now,
        is_scanning=_patrol_is_scanning,
        on_event=_patrol_event,
        ruta_waypoints=WAYPOINTS_PATH,
    )
    print(f"[patrulla] lista — {len(patrol.waypoints)} waypoints grabados")

    pose_estimator.gyro_weight = state.config["slam"]["pose_gyro_trust"]

    camera = CameraWorker(loop, _on_detection, lambda: state.config, health.beat)
    camera.set_model(app.get("yolo_model"))
    camera.start()

    global odom_webcam
    odom_webcam = OdometriaWebcam(
        obtener_frame=camera.ultimo_frame,
        get_config=lambda: state.config["odometria_webcam"],
        servo_al_frente=_servo_al_frente,
        publicar=ros_bridge.publish_odom_webcam,
    )
    odom_webcam.start()
    app["camera"] = camera

    app["stats_task"] = asyncio.create_task(stats_loop())


def load_yolo_model(app: web.Application):
    """
    Carga el modelo YOLO. Medido el 2026-09-06: importar torch cuesta 509 MB,
    cargarlo 630 MB, y la primera inferencia en GPU lo deja en 1525 MB — porque
    esta version de PyTorch no trae kernels para sm_87 y compila PTX en
    caliente. Es 1.5 GB de los 7.4 GB de la placa.

    Por eso NO se carga si la camara esta apagada: sin camara no hay fotograma
    que inferir y ese giga y medio se estaria reservando para nada. Con SLAM,
    LIDAR y rf2o al lado, esa reserva inutil es la que hacia que el OOM killer
    matara el backend (dos veces el 2026-09-06, ver dmesg).

    Al encender la camara desde el dashboard, `models_load_handler` vuelve a
    llamar aqui y el modelo se carga entonces.
    """
    active = state.config["ai"]["active_model"]
    if not active:
        app["yolo_model"] = None
        return
    if not state.config["hardware"].get("camera_enabled", True):
        app["yolo_model"] = None
        print(f"[YOLO] '{active}' NO se carga: la camara esta apagada "
              f"(ahorra ~1.5 GB). Se cargara al encenderla.")
        return
    try:
        # ⚠️ NO PONER AQUI `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True`.
        # Se probo el 2026-09-06 y ROMPE CUDA en esta placa: cualquier uso de
        # la GPU falla con
        #     RuntimeError: Expected NVML_SUCCESS ==
        #     DriverAPI::get()->nvmlDeviceGetGpuFabricInfoV_(...) to be true
        # Verificado aislandolo: el mismo script con la variable vacia usa la
        # GPU sin problema y con la variable puesta falla siempre. Costo una
        # tarde de diagnostico porque el sintoma aparece lejos, en el hilo de
        # inferencia de camera.py, y parece un fallo del modelo.
        #
        # Un solo hilo de CPU si se conserva: la inferencia va en GPU y cada
        # hilo extra de torch reserva sus propios buffers de trabajo.
        import torch
        torch.set_num_threads(1)
        from ultralytics import YOLO
        model_path = MODELS_DIR / active
        if model_path.exists():
            app["yolo_model"] = YOLO(str(model_path))
            # NO se hace `.to("cuda")` aqui. Se probo el 2026-09-06 y ROMPE el
            # arranque en esta placa:
            #   RuntimeError: Expected NVML_SUCCESS == DriverAPI::get()->
            #   nvmlDeviceGetGpuFabricInfoV_(...) to be true, but got false
            # Es un fallo de esta build de PyTorch consultando NVML en el Orin,
            # no del modelo. Ultralytics elige el dispositivo en cada predict()
            # por su cuenta y usa la GPU igual.
            print(f"[YOLO] modelo activo: {active}")
            if camera is not None:
                camera.set_model(app["yolo_model"])
        else:
            app["yolo_model"] = None
            if camera is not None:
                camera.set_model(None)
    except ImportError:
        app["yolo_model"] = None
        print("[YOLO] ultralytics no instalado, video sin detección")


def create_app() -> web.Application:
    app = web.Application()
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/video", video_handler)
    app.router.add_post("/api/video/record", video_record_handler)
    app.router.add_post("/api/drive", drive_handler)
    app.router.add_post("/api/nav/reset_map", nav_reset_map_handler)
    app.router.add_get("/api/mapas", mapas_listar_handler)
    app.router.add_post("/api/mapas/guardar", mapas_guardar_handler)
    app.router.add_post("/api/mapas/cargar", mapas_cargar_handler)
    app.router.add_get("/api/patrol", patrol_status_handler)
    app.router.add_post("/api/patrol/record", patrol_record_handler)
    app.router.add_post("/api/patrol/clear", patrol_clear_handler)
    app.router.add_post("/api/patrol/scan", patrol_scan_handler)
    app.router.add_post("/api/session-report", session_report_handler)
    app.router.add_get("/api/models", models_list_handler)
    app.router.add_post("/api/models/load", models_load_handler)
    app.router.add_get("/api/hardware", hardware_get_handler)
    app.router.add_post("/api/hardware", hardware_update_handler)
    app.router.add_get("/api/calibration", calibration_get_handler)
    app.router.add_post("/api/calibration", calibration_update_handler)
    app.router.add_post("/api/mission/start", mission_start_handler)
    app.router.add_post("/api/mission/stop", mission_stop_handler)
    app.router.add_get("/api/params", params_get_handler)
    app.router.add_post("/api/params/{section}", params_update_handler)
    app.router.add_post("/api/lidar/scan", lidar_scan_handler)
    app.router.add_get("/api/obstacle_status", obstacle_status_handler)
    app.router.add_get("/api/health", health_handler)
    app.router.add_get("/api/captures", captures_list_handler)
    app.router.add_post("/api/captures/clear", captures_clear_handler)
    app.router.add_post("/api/nav/set_goal", nav_set_goal_handler)
    app.router.add_post("/api/nav/cancel", nav_cancel_handler)
    app.router.add_post("/api/nav/restart", nav_restart_handler)
    app.router.add_post("/api/nav/save_map", nav_save_map_handler)
    app.router.add_get("/api/nav/mode", nav_mode_handler)
    app.router.add_post("/api/nav/mode", nav_mode_handler)
    app.router.add_static("/captures", CAPTURES_DIR, show_index=False)

    # La raíz sirve el dashboard directamente. Antes `add_static("/")` con
    # show_index=True devolvía un LISTADO DE DIRECTORIO en `/`: el operador
    # que entraba a http://192.168.12.1:8080 veía un índice de archivos en
    # vez del panel, y tenía que saber que debía pedir /index.html.
    DASHBOARD_DIR = Path(__file__).parent.parent / "dashboard"

    async def index_handler(request: web.Request):
        return web.FileResponse(DASHBOARD_DIR / "index.html")

    app.router.add_get("/", index_handler)
    app.router.add_static("/", DASHBOARD_DIR, show_index=False)

    async def on_cleanup(app):
        if camera is not None:
            camera.stop()
        if uart_link is not None:
            # Cerrar a proposito: si se deja el puerto abierto el C3 no ve
            # nada raro y solo para cuando salta su watchdog 300 ms despues.
            await uart_link.stop()
        ros_bridge.shutdown()
        global _sesion_http
        if _sesion_http is not None and not _sesion_http.closed:
            await _sesion_http.close()
            _sesion_http = None

    app.on_startup.append(start_background)
    app.on_cleanup.append(on_cleanup)
    load_yolo_model(app)
    return app


if __name__ == "__main__":
    # El finally importa: si `on_startup` falla (por ejemplo un puerto UDP
    # ocupado), aiohttp NO ejecuta `on_cleanup`, y el hilo del executor de
    # rclpy quedaba girando mientras el proceso moría — de ahí el
    # "terminate called without an active exception" al final del traceback,
    # que no aportaba nada y tapaba el error real.
    try:
        web.run_app(create_app(), host="0.0.0.0", port=HTTP_PORT)
    finally:
        ros_bridge.shutdown()
