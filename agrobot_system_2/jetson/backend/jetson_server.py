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
import glob
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
from ble_link import BleActuatorLink
import imu_nivel
import anticolision
import vision_activa as vision_activa_mod
import entorno_auto
from enlace_c3 import EnlaceC3
import udp_actuator_link
import config_store
import perfiles_entorno
import health
from slam import PoseEstimator, LidarReactive, MapManager
from costmap_local import CostmapLocal
from ros_bridge import AgroBotROSBridge, ROS_UNAVAILABLE_REASON
from camera import CameraWorker
from obstacle_avoidance import ControladorEvasion, LecturasSensoresLIDAR
from patrol import PatrolController
from pulse_drive import ConductorPulsos, ACCIONES, MODOS
from mapas import GestorMapas, ErrorMapa, validar_nombre as validar_nombre_mapa
from scan_mapa import scan_a_mapa
from mapa_scan import MapaScan
import simulador_visual
from asistido import Asistido
from servo_bajo import ServoBajo
from zoom_auto import ZoomAuto, DEFECTOS as ZOOM_DEFECTOS
from odometria_webcam import OdometriaWebcam, SERVO_TOLERANCIA_DEG
from explorador import ExploradorReactivo
from sendero import SeguidorSendero
from navigation_logger import NavigationLogger, analizar_sesion
from aprendizaje import Aprendizaje
from paso_robotico import PasoRobotico
from suelo_libre import SueloLibre
from camara_lidar import CamaraComoLidar
from maniobra_giro import ManiobraGiro
import misiones as misiones_mod
from reloj import DetectorSalto
import suelo_libre as suelo_mod
from barrido_tilt import (BarridoTilt, FrenoProactivo, EstimadorAltura,
                          inclinacion_por_servo, SERVO_LEJOS_DEG, SERVO_CERCA_DEG)
from grabador import Grabador, TELEMETRIA_HZ
import evasion as evasion_mod
from cabeceo import CabeceoIMU
from politica import Politica
from deslizamiento import MonitorDeslizamiento
from modo_seguro import ModoSeguro, NORMAL, SEGURO, PRECAUCION
from supervisor_movimiento import SupervisorMovimiento
from manual_demo import command as demo_command, reference as demo_reference
from c3_setup import C3Setup
c3_setup = C3Setup()
from c3_pines import C3Pines
c3_pines = C3Pines(lambda: enlace_c3)
from auto_comunicacion import inicializar_auto_comunicacion

# =============================================
# ===== CONFIG =====
# =============================================
JETSON_AP_IP = "192.168.12.1"
LORA_UDP_PORT = 5001
HTTP_PORT = 8080
CMD_SEND_HZ = 20
# Gateway de la Raspberry Pi. Las alarmas que hay que hacer llegar al
# celular del operador se empujan al gateway por LAN y él las publica en
# Telegram. La Pi va por cable al router "LoRa-Wan" (DIR-615 en modo router),
# que le reserva 192.168.0.10 por MAC; la Jetson llega a ella como cliente
# WiFi de esa misma red. Si el gateway está apagado, esto falla en silencio:
# avisar es deseable, pero nunca a costa de frenar la detección o el control.
GATEWAY_ALERT_URL = os.environ.get(
    "AGROBOT_GATEWAY_URL", "http://192.168.0.10:8092") + "/api/alert"
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
# Cabeceo del chasis a partir del MPU6050. Se crea aquí porque la telemetría
# empieza a llegar antes de que arranque la cámara; el signo se ajusta al
# cargar la configuración.
cabeceo_imu = CabeceoIMU()
lidar_reactive = LidarReactive()
# Suelo libre por cámara: lo bajo que el LIDAR no ve (cobija, pesa: 2026-10-02).
suelo_libre = SueloLibre(cfg={"metodo": "apariencia"})   # 2026-10-22: sin los «0 m» falsos del histograma en baldosa
camara_lidar = CamaraComoLidar()


def _cargar_modelo_transitabilidad():
    """autonomia.suelo_modelo = ruta de un .pt de entrenar_trav.py: suelo_libre usa la UNet (libre/obstáculo) en vez de la apariencia. Si falla, sigue con la apariencia."""
    ruta = state.config.get("autonomia", {}).get("suelo_modelo", "")
    if not ruta:
        return
    try:
        from transitabilidad import Transitabilidad
        suelo_libre.segmentador = Transitabilidad(ruta)
        suelo_libre.metodo = "modelo"
        print(f"[suelo] modelo de transitabilidad cargado: {ruta}", flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f"[suelo] no se pudo cargar el modelo ({type(exc).__name__}: {exc}); sigo con la apariencia", flush=True)
lidar_reactive.hueco_camara = lambda: suelo_libre.hueco_delante(
    state.config["slam"]["robot_ancho_cm"] / 100.0)
# Barrido de tilt + freno proactivo por envolvente de vuelco (ver barrido_tilt.py).
# Se activa con autonomia.barrido_tilt (OFF por defecto): mueve el servo para ver
# lo bajo que el LIDAR no ve y frena antes de volcar/atrancarse.
barrido_tilt_ctrl = BarridoTilt()
freno_proactivo = FrenoProactivo()
# Estima la altura del montículo por paralaje al acercarse (ver barrido_tilt.py).
# h_cam ~ altura de la cámara sobre el suelo.
estimador_altura = EstimadorAltura(h_cam=0.4)
costmap = CostmapLocal()
map_manager = MapManager()

ros_fused_pose: dict | None = None
# Monotónico de cuándo llegó la última pose del EKF. Sin esto, `current_pose()`
# devolvía la última pose para siempre aunque el EKF dejara de publicar: una
# pose CONGELADA que el dashboard y la lógica tomaban como viva (auditoría
# 2026-10-07). Con el sello se puede saber su antigüedad y no confiar en ella.
ros_fused_pose_t: float | None = None
# Pasado este umbral sin pose nueva del EKF, la localización deja de ser fiable.
POSE_VIEJA_S = 1.5
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
        # El grabador ve TODO lo anunciado, haya o no un dashboard abierto.
        if grabador is not None:
            grabador.evento(message)
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
    # 2026-10-23: el firmware del nodo (esp32_lora_v3) marca el SCD40 como listo sin comprobar que responde y, si nunca hay medida,
    # manda co2=0, tempSCD=0, humidity=0. Los tres a cero a la vez es imposible en la realidad: es «sensor ambiental no disponible».
    if all(payload.get(k) == 0.0 for k in ("co2", "tempSCD", "humidity")):
        missing.extend(k for k in ("co2", "tempSCD", "humidity") if k not in missing)
    payload["_sensors_missing"] = missing
    return payload


# Pérdidas y reinicios del nodo LoRa V3, deducidos de su `seq`.
enlace_lora = health.ContadorSecuencia()


class SesgoGiroscopio:
    """Sesgo del giroscopio estimado solo, con el robot quieto.

    En la configuración estaba en 0 (nunca se calibró) y el MPU6050 marca
    +0,92 °/s en Z estando quieto: la pose giraba 53° por minuto sin moverse
    (2026-09-30). Cada vez que el robot lleva ZUPT_TRAS_S sin orden de
    moverse, las lecturas son puro sesgo + ruido: se promedian (media simple
    al principio, luego media móvil de TAU_S) y se restan. Así se corrige
    también en marcha, y se sigue la deriva térmica del sensor."""

    TAU_S = 30.0
    MIN_MUESTRAS = 100          # ~5 s a 20 Hz antes de aplicarlo
    SALTO_DPS = 20.0            # una lectura así en reposo es un golpe, no sesgo

    def __init__(self):
        self.sesgo = {"gyroX": 0.0, "gyroY": 0.0, "gyroZ": 0.0}
        self.muestras = 0
        self._t = None

    @property
    def listo(self) -> bool:
        return self.muestras >= self.MIN_MUESTRAS

    def actualizar(self, lectura: dict, quieto: bool, ahora: float):
        if not quieto:
            self._t = None
            return
        if any(abs(lectura.get(e, 0.0) - b) > self.SALTO_DPS for e, b in self.sesgo.items()) and self.listo:
            return
        dt = 0.05 if self._t is None else min(max(ahora - self._t, 0.0), 0.5)
        self._t = ahora
        self.muestras += 1
        a = 1.0 / self.muestras if not self.listo else dt / self.TAU_S
        for eje in self.sesgo:
            self.sesgo[eje] += a * (lectura.get(eje, 0.0) - self.sesgo[eje])


sesgo_giro = SesgoGiroscopio()


from lora_sensor_codec import SampleGate
from ambiente_simulado import AmbienteSimulado
ambiente_simulado = AmbienteSimulado()
lora_sample_gate=SampleGate()

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
                _parada_total(f"LoRa {comando}")
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

        if payload.get("sensor_v") == 3 and not lora_sample_gate.accept(payload):
            return

        seq = payload.get("seq")
        if isinstance(seq, (int, float)) and enlace_lora.registrar(int(seq)):
            print(f"[lora] el nodo LoRa V3 se reinició (#{enlace_lora.reinicios} desde que "
                  f"arrancó el backend): si se repite, revisa su alimentación", flush=True)
        payload = normalize_env(payload)
        if state.config["calibration"].get("ambiente_simulado"):
            payload = ambiente_simulado.aplicar(payload)
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
        # Nivel del montaje (imu_nivel.py): con la gravedad medida en nivel se gira el acelerómetro y el
        # giroscopio para que «arriba» sea arriba. Se conserva el acelerómetro CRUDO para recalibrar.
        if all(k in payload for k in ("accelX", "accelY", "accelZ")):
            payload["_acc_raw"] = (payload["accelX"], payload["accelY"], payload["accelZ"])
            R = _matriz_nivel_cache(cal.get("imu_gravedad_nivel"))
            if R is not None:
                payload["accelX"], payload["accelY"], payload["accelZ"] = imu_nivel.aplicar(R, payload["_acc_raw"])
                if all(k in payload for k in ("gyroX", "gyroY", "gyroZ")):
                    payload["gyroX"], payload["gyroY"], payload["gyroZ"] = imu_nivel.aplicar(
                        R, (payload["gyroX"], payload["gyroY"], payload["gyroZ"]))
        # Autocalibración en reposo (ver SesgoGiroscopio). Sin enlace con la
        # C3 el robot no puede moverse: también cuenta como quieto.
        quieto = actuator_proto is None or actuator_proto.segundos_quieto() > ZUPT_TRAS_S
        sesgo_giro.actualizar(payload, quieto, time.monotonic())
        if sesgo_giro.listo:
            for axis, b in sesgo_giro.sesgo.items():
                if axis in payload:
                    payload[axis] = payload[axis] - b

        payload["_received_at"] = time.time()
        payload["_source"] = addr[0]
        state.env_telemetry = payload
        health.beat("lora_v3")

        # --- CAMINO RÁPIDO: todo lo que consume el estimador de pose ---
        # Va a la frecuencia completa del nodo (20 Hz desde 2026-09-06), que
        # es justo la `frequency: 20.0` del EKF en ekf.yaml.
        pose_estimator.on_gyro(payload.get("gyroZ", 0.0))
        # El mismo paquete sirve para saber cuánto cabecea el chasis, que es
        # lo que descuadra la vista del sendero (ver cabeceo.py).
        cabeceo_imu.actualizar(payload.get("accelX", 0.0), payload.get("accelY", 0.0),
                               payload.get("accelZ", 0.0), payload.get("gyroX", 0.0),
                               time.monotonic())
        # IMU montado con Y adelante, X a los lados. ROS quiere X adelante,
        # Y izquierda, Z arriba. Remap: ROS_x=sensor_y, ROS_y=-sensor_x.
        ros_bridge.publish_imu(
            {"x": payload.get("accelY", 0.0), "y": -payload.get("accelX", 0.0), "z": payload.get("accelZ", 0.0)},
            {"x": payload.get("gyroY", 0.0), "y": -payload.get("gyroX", 0.0), "z": payload.get("gyroZ", 0.0)},
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


_nivel_cache = {"g": None, "R": None}


def _matriz_nivel_cache(g):
    """Matriz de nivel del IMU, recalculada solo cuando cambia la gravedad guardada."""
    if not g:
        return None
    clave = tuple(g)
    if _nivel_cache["g"] != clave:
        _nivel_cache["g"], _nivel_cache["R"] = clave, imu_nivel.matriz_nivel(g)
    return _nivel_cache["R"]


IMU_VIEJO_S = 3.0    # telemetría del IMU más vieja que esto = el nodo está apagado: no se usa


def _imu_fresco() -> bool:
    ra = (state.env_telemetry or {}).get("_received_at")
    return ra is not None and time.time() - ra <= IMU_VIEJO_S


async def calibrar_imu_nivel_handler(request: web.Request):
    """POST /api/calibracion/imu_nivel  {"reset": true} para borrarla.
    Con el robot QUIETO y NIVELADO: promedia ~5 s de acelerómetro crudo y guarda la gravedad medida; desde
    entonces el IMU se usa corregido (inclinación y cabeceo reales). Requiere el nodo de sensores encendido."""
    try:
        body = await request.json()
    except Exception:
        body = {}
    if body.get("reset"):
        state.config = config_store.update_section("calibration", {"imu_gravedad_nivel": None})
        return web.json_response({"ok": True, "imu_gravedad_nivel": None})
    if actuator_proto is not None and actuator_proto.segundos_quieto() < 2.0:
        return web.json_response({"error": "el robot se está moviendo"}, status=409)
    muestras, vistos, t0 = [], set(), time.monotonic()
    while time.monotonic() - t0 < 8.0 and len(muestras) < 60:
        env = state.env_telemetry or {}
        ra = env.get("_received_at")
        if ra is not None and time.time() - ra <= 1.0 and ra not in vistos and env.get("_acc_raw"):
            vistos.add(ra)
            muestras.append(env["_acc_raw"])
        await asyncio.sleep(0.05)
    g, err = imu_nivel.promediar(muestras)
    if err:
        return web.json_response({"error": err, "muestras": len(muestras)}, status=409)
    state.config = config_store.update_section("calibration", {"imu_gravedad_nivel": g})
    roll, pitch = imu_nivel.inclinacion_deg(g)
    print(f"[imu] nivel calibrado: g={g} (balanceo {roll:.1f}°, cabeceo {pitch:.1f}°)", flush=True)
    return web.json_response({"ok": True, "imu_gravedad_nivel": g, "muestras": len(muestras),
                              "montaje_balanceo_deg": round(roll, 1), "montaje_cabeceo_deg": round(pitch, 1)})


# =============================================
# ===== ACTUADOR (ESP32-C3 Mini, por UART) =====
# =============================================
# Esta clase conserva TODA la logica de seguridad que tenia el camino UDP
# —rampa, tope por LIDAR, odometria, calibracion del servo— y solo cambia el
# transporte por debajo: en vez de un datagrama al 192.168.12.50 escribe en
# un puerto serie. Ver uart_link.py para por que se dejaron el WiFi y el BLE.
anticolision_global = anticolision.Anticolision()
vision_activa = None          # vision_activa.VisionActiva (se crea al arrancar; APAGADA hasta que el operador la pide)


class ActuatorController:
    def __init__(self, link):
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
        # Último instante en que se mandó moverse (tras rampa y freno).
        self._t_movimiento = 0.0

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
        _schedule(state.broadcast({"type": "actuator_status", "data": status}))

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

    def segundos_quieto(self) -> float:
        """Tiempo desde la última orden de moverse que salió hacia la C3."""
        return time.monotonic() - self._t_movimiento

    def velocidades(self) -> dict:
        """Lo que pidió el software y lo que salió tras la rampa. Sin esto el
        registro de calibración guardaba velocidades en cero (2026-09-19) y
        no se podía comparar lo comandado con lo real."""
        return {"v_pedida": self._obj_v, "omega_pedida": self._obj_omega,
                "v_aplicada": self._ramp_v, "omega_aplicada": self._ramp_omega}

    def _aplicar(self, scan_cmd: int):
        v_ref, omega_ref = self._apply_ramp(
            self._obj_v, self._obj_omega, state.config["safety"])

        try:
            v_ref, omega_ref = demo_reference(v_ref, omega_ref,
                pivote=bool(state.config.get("control", {}).get("pivote_en_sitio", False)))
        except (ValueError, TypeError):
            v_ref = omega_ref = 0.0
        if not self.link.connected:
            v_ref = omega_ref = 0.0
            self._obj_v = self._obj_omega = self._ramp_v = self._ramp_omega = 0.0

        # Frenado por LIDAR DESHABILITADO: permite manejo manual sin restricciones en pasillos
        # (comentado 2026-10-07 — usuario pide control total sin bloqueos automáticos)
        # v_seg, aviso = lidar_reactive.limitar_velocidad(v_ref, state.config["slam"])
        # if v_seg != v_ref:
        #     self._ramp_v = v_seg
        #     ahora = time.monotonic()
        #     if aviso is not None and ahora - self._ultimo_aviso_t > 0.25:
        #         self._ultimo_aviso_t = ahora
        #         _schedule(state.broadcast({"type": "obstacle_warning", "data": aviso}))
        #     v_ref = v_seg

        # Patinaje y modo seguro DESHABILITADOS: control manual sin restricciones
        # (comentado 2026-10-07 — usuario pide control total sin limitaciones automáticas)
        # factor = min(monitor_desliz.factor_velocidad(),
        #              modo_seg.factor(manual=not _autonomia_activa()))
        # if factor < 1.0:
        #     v_ref *= factor
        #     omega_ref *= factor
        # monitor_desliz.on_orden(v_ref, omega_ref)  # DESHABILITADO: sin monitoreo restrictivo

        # FRENO POR LIDAR/CÁMARA SOLO EN AUTÓNOMO (2026-10-16, exposición): cuando el robot va solo, no
        # puede avanzar contra algo ni sin ver. En manual sigue sin restricciones (pedido del 2026-10-07).
        if (abs(v_ref) > 1e-6 and _autonomia_activa()
                and state.config.get("safety", {}).get("freno_lidar_autonomo", True)):
            v_seg, aviso = lidar_reactive.limitar_velocidad(v_ref, state.config["slam"])
            if v_seg != v_ref:
                self._ramp_v = v_seg
                ahora = time.monotonic()
                if aviso is not None and ahora - self._ultimo_aviso_t > 0.25:
                    self._ultimo_aviso_t = ahora
                    _schedule(state.broadcast({"type": "obstacle_warning", "data": aviso}))
                v_ref = v_seg

        # ANTICOLISIÓN CON RETROCESO (2026-10-21): si va a tocar algo delante, cancela el avance y retrocede un poco.
        # Vale en manual y en autónomo (es el mismo punto de salida). Apagable: safety.anticolision_activa.
        try:
            v_ac, w_ac, ev_ac, reinicia = anticolision_global.filtrar(
                v_ref, omega_ref, lidar_reactive.distancias(state.config["slam"]), lidar_reactive.edad_scan_s(), state.config.get("safety", {}))
            if reinicia:
                self._ramp_v, self._ramp_omega = v_ac, 0.0
            v_ref, omega_ref = v_ac, w_ac
            if ev_ac is not None:
                print(f"[anticolision] {ev_ac}", flush=True)
                if ev_ac["tipo"] in ("retroceso", "alto_sin_espacio"):
                    txt = (f"Obstáculo a {ev_ac['delante_m'] * 100:.0f} cm: retrocedo" if ev_ac["tipo"] == "retroceso"
                           else f"Obstáculo a {ev_ac['delante_m'] * 100:.0f} cm y sin espacio detrás: me detengo")
                    _schedule(state.broadcast({"type": "toast", "data": {"kind": "warn", "title": "Anticolisión", "message": txt}}))
        except Exception as exc:                       # nunca dejar sin órdenes al robot por un fallo de esta capa
            print(f"[anticolision] fallo: {type(exc).__name__}: {exc}", flush=True)

        # PROTECCIÓN FINAL COMÚN (supervisor): único punto por el que pasa TODO
        # lo que mueve el robot (manual, Nav2, patrulla, exploración), así que
        # es el lugar correcto para enclavar la emergencia y unificar límites.
        # OFF por defecto (`safety.supervisor_activo`): activar tras validar.
        # NO reemplaza el freno por LIDAR (capa aparte, sigue bajo tu control).
        if state.config.get("safety", {}).get("supervisor_activo", False):
            v_ref, omega_ref, _mot_sup = supervisor.proteccion_final(v_ref, omega_ref)

        if abs(v_ref) > 1e-3 or abs(omega_ref) > 1e-3:
            self._t_movimiento = time.monotonic()
        pose_estimator.on_command(v_ref, omega_ref)
        ros_bridge.publish_wheel_odom(v_ref, omega_ref)
        self.link.set_actuators(self._servo_angle_with_offset(), state.led_on)
        self.link.send_command(v_ref, self._giro_signo() * omega_ref, scan_cmd)

    def send_actuator_update(self):
        """Manda servo/LED sin tocar el movimiento: reusa la velocidad que
        la rampa ya tiene aplicada, en vez de forzar 0 (si el robot va en
        marcha y el operador toca el LED, no debe dar un tirón)."""
        self.link.set_actuators(self._servo_angle_with_offset(), state.led_on)
        self.link.send_command(self._ramp_v, self._giro_signo() * self._ramp_omega, 0)

    def _giro_signo(self) -> int:
        """-1 si el giro está invertido por hardware (config control.invertir_giro).

        Corrige por SOFTWARE el caso "pido izquierda y va a la derecha" sin
        reflashear: se niega omega JUSTO al enviar a la C3. No se toca el omega
        LÓGICO (el que va a la odometría/EKF y respeta la convención ROS
        omega>0=izquierda): es solo una corrección de cableado en el borde del
        actuador, así el giroscopio y lo comandado quedan con el mismo signo."""
        return -1 if state.config.get("control", {}).get("invertir_giro", False) else 1

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
        # Enclava la emergencia en el supervisor SOLO si está activo: así, con
        # el flag OFF el supervisor queda dormido y no acumula estado (a
        # `emergency_stop` también lo llaman `park`/desconexión/watchdog, que no
        # son emergencias reales). Con el flag ON, la protección final rechaza
        # todo movimiento hasta un rearme explícito (supervisor.rearmar, salud OK).
        if state.config.get("safety", {}).get("supervisor_activo", False):
            supervisor.emergencia()
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
    """Nav2 publicó /cmd_vel. Hasta el 2026-10-20 esta función hacía `return` al empezar («manual demo»): el backend
    DESCARTABA todo lo que Nav2 calculaba y la patrulla / «ir a meta» no movían el robot. Ahora Nav2 manda motores
    SOLO si hay una meta suya en curso, y nunca contra el operador ni en el perfil de demostración. Se recorta a
    control.nav2_v_max_mps / nav2_w_max_rps. Solo (v, omega): el robot es diferencial.

    Pasa por `actuator_proto.send_command`, así que también le valen la rampa, el freno por LIDAR (si está activo),
    `invertir_giro` y la parada total (que cancela la meta de Nav2)."""
    if actuator_proto is None:
        return
    ctl = state.config.get("control", {})
    if ctl.get("solo_manual", False):
        return
    if ros_bridge.estado_meta() not in ("enviada", "en_curso"):
        return                                   # sin meta de Nav2 en curso, /cmd_vel no manda nada
    if conductor is not None and conductor.activo:
        return                                   # el operador tiene un pulso en marcha: manda él
    vmax = float(ctl.get("nav2_v_max_mps", 0.25))
    wmax = float(ctl.get("nav2_w_max_rps", 0.6))
    v = max(-vmax, min(vmax, v_ref))
    w = max(-wmax, min(wmax, omega_ref))
    actuator_proto.send_command(v, w, 0)


# El radar del dashboard mide ~300 px de lado: más de ~200 puntos caen en el
# mismo píxel y no aportan nada visible. Un scan completo del C1 son ~600
# puntos = 30 KB de JSON, 10 veces por segundo y POR CLIENTE (300 KB/s).
# Se diezma SOLO lo que se difunde; la evasión reactiva sigue recibiendo el
# scan completo, que es donde la resolución sí importa: perder el único rayo
# que da contra una rama fina es perder el frenado.
RADAR_MAX_PUNTOS = 200
_ultimo_envio_lidar = 0.0
_ultimo_envio_pose = 0.0
_ultimo_envio_scan_mapa = 0.0
SCAN_MAPA_PERIODO_S = 0.1     # el scan sobre el mapa va a 10 Hz: es "en vivo"


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


mapa_scan = MapaScan()
_ultimo_mapa_scan_t = 0.0
_ultimo_envio_mapa_scan = 0.0
MAPA_SCAN_PERIODO_S = 0.2          # el scan se acumula a 5 Hz
MAPA_SCAN_ENVIO_S = 1.0            # y el mapa acumulado se difunde a 1 Hz


def _mapa_desde_scan() -> bool:
    return (state.config.get("slam") or {}).get("mapa_fuente", "scan") == "scan"


_mapa_scan_ocupado = False


def _mapa_limpio() -> bool:
    """slam.mapa_limpio (por defecto True): el dashboard recibe el mapa con paredes de una celda, sin franjas dobles ni motas (mapa_scan._clasificar)."""
    return bool(state.config.get("slam", {}).get("mapa_limpio", True))


async def _agregar_y_difundir(ahora: float, pose: dict, points: list[dict], difundir: bool):
    """Registro scan-a-mapa (~60 ms) y limpieza del mapa FUERA del bucle de eventos: si no, el dashboard y el robot se atascaban cada 0,2 s."""
    global _mapa_scan_ocupado
    try:
        await asyncio.to_thread(mapa_scan.agregar, ahora, pose, [p["angle_deg"] for p in points], [p["distance_m"] for p in points])
        if difundir:
            m = await asyncio.to_thread(mapa_scan.como_mapa, _mapa_limpio())
            if m is not None:
                m["received_at"] = time.time()
                await state.broadcast({"type": "map", "data": m})
    except Exception as exc:  # noqa: BLE001
        print(f"[mapa_scan] {type(exc).__name__}: {exc}", flush=True)
    finally:
        _mapa_scan_ocupado = False


def _acumular_scan(points: list[dict], pose: dict):
    """El mapa del dashboard se dibuja con los puntos del scan acumulados (mapa_scan.py), registrados contra lo ya dibujado."""
    global _ultimo_mapa_scan_t, _ultimo_envio_mapa_scan, _mapa_scan_ocupado
    ahora = time.monotonic()
    if _mapa_scan_ocupado or ahora - _ultimo_mapa_scan_t < MAPA_SCAN_PERIODO_S:
        return
    _ultimo_mapa_scan_t = ahora
    difundir = ahora - _ultimo_envio_mapa_scan >= MAPA_SCAN_ENVIO_S
    if difundir:
        _ultimo_envio_mapa_scan = ahora
    _mapa_scan_ocupado = True
    _schedule(_agregar_y_difundir(ahora, dict(pose), points, difundir))


def _difundir_scan_en_mapa(points: list[dict]):
    """El scan en coordenadas del MAPA (pose = TF map->base_link de
    slam_toolbox), para pintar los puntos en vivo sobre el mapa. Sin TF
    (SLAM aún sin arrancar) no se manda nada: pintarlo con la pose del EKF
    lo dibujaría desplazado respecto al mapa."""
    global _ultimo_envio_scan_mapa
    ahora = time.monotonic()
    if ahora - _ultimo_envio_scan_mapa < SCAN_MAPA_PERIODO_S:
        return
    pose = ros_bridge.pose_en_mapa()
    if pose is None:
        return
    _ultimo_envio_scan_mapa = ahora
    if _mapa_desde_scan():
        _acumular_scan(points, pose)
    _schedule(state.broadcast({"type": "scan_mapa", "data": {
        "pose": {k: round(pose[k], 3) for k in ("x", "y", "theta_deg")},
        "pts": scan_a_mapa(points, pose)}}))


def _ros_on_scan_points(points: list[dict]):
    """sllidar_ros2 publicó /scan — alimenta la evasión reactiva + el radar."""
    lidar_reactive.ingest_scan(points)
    cfg_slam = state.config["slam"]
    costmap.update(points, cfg_slam["robot_largo_cm"], cfg_slam["robot_ancho_cm"])
    health.beat("lidar")
    _difundir_scan_en_mapa(points)
    # Al dashboard se manda a 4 Hz, no a 10: el scan decimado son ~10 kB por
    # mensaje y por WiFi compartido eso era ~100 kB/s solo de radar, que es
    # parte de la latencia que se notaba. La evasión sigue usando los 10 Hz.
    global _ultimo_envio_lidar
    ahora = time.monotonic()
    if ahora - _ultimo_envio_lidar < 0.25:
        return
    _ultimo_envio_lidar = ahora
    cfg = state.config["slam"]
    _schedule(state.broadcast({"type": "lidar_scan",
                               "data": _decimar(points, RADAR_MAX_PUNTOS)}))
    # Al dashboard en CENTÍMETROS: a estas distancias es lo que se entiende
    # de un vistazo, y es como se configuran los umbrales.
    d = lidar_reactive.distancias(cfg)
    _schedule(state.broadcast({"type": "lidar_distancias", "data": {
        **{lado: (None if v is None else round(v * 100)) for lado, v in d.items()},
        "umbral_cm": cfg["margen_parada_cm"] + cfg["deslizamiento_cm"],
    }}))


def _ros_on_fused_pose(pose: dict):
    """robot_localization publicó /odometry/filtered — reemplaza al PoseEstimator casero."""
    global ros_fused_pose, ros_fused_pose_t, _ultimo_envio_pose
    ros_fused_pose = pose
    ros_fused_pose_t = time.monotonic()
    ahora = time.monotonic()
    if ahora - _ultimo_envio_pose < 0.12:      # ~8 Hz al dashboard
        return
    _ultimo_envio_pose = ahora
    _schedule(state.broadcast({"type": "pose", "data": pose, "source": "ros_ekf"}))


_last_map_broadcast = 0.0


def _ros_on_map(map_dict: dict):
    """slam_toolbox publicó /map — se manda al dashboard, limitado a 1 Hz (es pesado)."""
    global ros_last_map, _last_map_broadcast
    health.beat("slam")
    ros_last_map = map_dict
    now = time.time()
    if _mapa_desde_scan() and mapa_scan.version:     # el dashboard dibuja el mapa de puntos del scan; la rejilla SLAM queda de respaldo
        return
    if now - _last_map_broadcast < 0.5:
        return
    _last_map_broadcast = now
    _schedule(state.broadcast({"type": "map", "data": map_dict}))


# Patinaje / atasco (ver deslizamiento.py) y modo seguro (ver modo_seguro.py):
# lo ordenado se compara con lo que mide el LIDAR, y los fallos serios
# llevan al robot a un estado seguro en vez de dejarlo seguir a ciegas.
monitor_desliz = MonitorDeslizamiento()
modo_seg = ModoSeguro()

# Supervisor central de movimiento (ver supervisor_movimiento.py). Por ahora
# solo se usa su PROTECCIÓN FINAL COMÚN (emergencia enclavada + límites), y SOLO
# si `safety.supervisor_activo` está en true (OFF por defecto): así se puede
# validar sin cambiar el comportamiento actual. Los límites salen de los topes
# FÍSICOS reales del firmware (V_MAX/OMEGA_MAX), no de los ficticios del backend.
supervisor = SupervisorMovimiento(v_max=0.55, omega_max=1.5)


def _ros_on_odom_lidar(v: float, w: float):
    """Velocidad medida por rf2o (hilo de ROS): solo actualiza números."""
    monitor_desliz.on_medida(v, w)


ros_bridge = AgroBotROSBridge(_ros_on_cmd_vel, _ros_on_scan_points, _ros_on_fused_pose, _ros_on_map,
                              on_odom_lidar=_ros_on_odom_lidar)

# Se crea en start_background, cuando ya existe el event loop al que el hilo
# de captura tiene que despachar los frames.
camera: CameraWorker | None = None


def current_pose() -> dict:
    """Pose fusionada real (EKF de robot_localization) si ROS está corriendo
    y ya publicó al menos una vez; si no, cae al filtro complementario
    casero de slam.py para que el dashboard nunca se quede sin lectura."""
    return ros_fused_pose if ros_fused_pose is not None else pose_estimator.snapshot()


def pose_edad_s() -> float | None:
    """Segundos desde la última pose del EKF, o None si nunca llegó una.

    Reloj monótono (no time.time): inmune a saltos del reloj del sistema, que
    en esta Jetson sin RTC pueden ser de años al arrancar (ver memoria)."""
    if ros_fused_pose_t is None:
        return None
    return time.monotonic() - ros_fused_pose_t


def pose_confiable() -> bool:
    """¿La localización es de fiar AHORA? False si el EKF no corre o si su
    última pose es más vieja que POSE_VIEJA_S (pose congelada). Los modos que
    dependen de localización deben consultarlo antes de navegar con la pose."""
    edad = pose_edad_s()
    return edad is not None and edad <= POSE_VIEJA_S


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
            "health": _salud_completa(),
            "pose": current_pose(),
            "pose_source": "ros_ekf" if ros_fused_pose is not None else "dead_reckoning",
            "pose_edad_s": (round(e, 2) if (e := pose_edad_s()) is not None else None),
            "pose_confiable": pose_confiable(),
            "lidar_scan": _decimar(lidar_reactive.last_scan or [], RADAR_MAX_PUNTOS),
            "map": (mapa_scan.como_mapa(_mapa_limpio()) if _mapa_desde_scan() and mapa_scan.version else None) or ros_last_map,
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
                if vision_activa is not None and (cmd.get("v") or cmd.get("omega")):
                    vision_activa.cancelar("orden manual")
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
                try:
                    v, omega = demo_command(cmd.get("v",0),cmd.get("omega",0),
                                            float(cmd.get("scale",1))*100)
                except (ValueError, TypeError):
                    actuator_proto.emergency_stop()
                    continue
                if (v or omega):
                    _tomar_mando("ws")
                    _registrar_orden_manual(v, omega, "ws")
                if conductor is not None:
                    conductor.parar()
                if enlace_c3 is not None and enlace_c3.connected:
                    if (v or omega) and not await _c3_ready_for_drive():
                        actuator_proto.emergency_stop()
                        await ws.send_json({"type":"error","error":"Configura la C3 1.1 y entrega control a Jetson"})
                        continue
                    actuator_proto.send_command(v, omega, 0)

            elif cmd.get("type") == "actuator":
                # Servo / LED. Va por su propio mensaje y NO pasa por el
                # throttle de `control`: el operador puede mover el slider
                # con el robot detenido, y en ese caso no hay comandos de
                # movimiento saliendo que arrastren el cambio.
                if "servo_angle" in cmd:
                    state.servo_angle = clamp_servo(float(cmd["servo_angle"]))
                if "led" in cmd:
                    state.led_on = bool(cmd["led"])
                actuator_proto.send_actuator_update()
                await state.broadcast({"type": "actuator_setpoint", "data": {
                    "servo_angle": state.servo_angle,
                    "led": state.led_on,
                }})

            elif cmd.get("type") == "emergency_stop":
                _manejo = cmd.get("tipo") == "manejo"
                _parada_total("manejo WebSocket" if _manejo else "dashboard WebSocket", emergencia=not _manejo)

    finally:
        state.ws_clients.discard(ws)
        # Un celular que se duerme o pierde el WiFi NO debe cortar la autonomía (presentación 2026-10-23): solo se frena si
        # nadie la está llevando, o si el operador tenía el mando manual.
        if _modo_auto_activo() is None or asistido.activa:
            if conductor is not None:
                conductor.parar()
            if actuator_proto is not None:
                actuator_proto.emergency_stop()

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
    except (ConnectionError, asyncio.CancelledError):
        # ConnectionError cubre BrokenPipe y ConnectionReset: es lo normal
        # cuando el navegador cierra la pestaña, no un fallo que reportar.
        pass
    finally:
        camera.unsubscribe(cola)

    return response


# =============================================
# ===== MODO AUTÓNOMO =====
# =============================================
patrol: PatrolController | None = None
conductor: ConductorPulsos | None = None
paso_robotico: PasoRobotico | None = None
grabador: Grabador | None = None
explorador: ExploradorReactivo | None = None
nav_logger: NavigationLogger | None = None
_nav_cal_task: asyncio.Task | None = None
_nav_cal_stats: dict = {}
seguidor_sendero: SeguidorSendero | None = None

WAYPOINTS_PATH = _AQUI / "patrol_waypoints.json"


odom_webcam: OdometriaWebcam | None = None


def _distancias_explorador() -> dict:
    """Huecos del LIDAR + lo que la CÁMARA ve en el suelo delante (2026-10-22).

    El LIDAR está a 37,5 cm: cojines, cajas, macetas bajas no existen para él y el explorador se les echaba encima. suelo_libre (método por
    apariencia) da la distancia al primer obstáculo del suelo en el camino del robot; si es menor que la del LIDAR, manda la cámara.
    Se apaga con autonomia.camara_obstaculos = false."""
    d = lidar_reactive.distancias(state.config["slam"])
    if not state.config["autonomia"].get("camara_obstaculos", True):
        return d
    cam = suelo_libre.hueco_delante(state.config["slam"]["robot_ancho_cm"] / 100.0)
    if cam is None:
        return d
    cam = max(0.0, cam - state.config["slam"]["robot_largo_cm"] / 200.0)       # desde la cara delantera, como el LIDAR
    if d.get("adelante") is None or cam < d["adelante"]:
        d = dict(d)
        d["adelante"] = cam
        d["adelante_fuente"] = "camara"
    return d


def _scan_explorador() -> list[dict]:
    """Scan que ve el explorador: el del LIDAR y, con autonomia.camara_como_lidar, también el frente de la CÁMARA filtrado (camara_lidar.py: obstáculos bajos que el
    plano del LIDAR no ve). autonomia.camara_lidar_solo = true reemplaza el frente del LIDAR por el de la cámara (prueba)."""
    scan = lidar_reactive.last_scan
    cfg = state.config["autonomia"]
    if not cfg.get("camara_como_lidar", False):
        return scan
    try:
        return camara_lidar.fusionar(scan, suelo_libre.ultimo, state.config["slam"]["robot_largo_cm"] / 100.0, bool(cfg.get("camara_lidar_solo", False)))
    except Exception as exc:  # noqa: BLE001
        print(f"[camara-lidar] filtro falló: {type(exc).__name__}: {exc}", flush=True)
        return scan


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


# Con el robot quieto se le dice al EKF "velocidad cero" (ZUPT). Sin LIDAR el
# EKF no recibía ninguna velocidad lineal y extrapolaba la última durante
# horas: el 2026-09-30 la pose llegó a x = 1771 m y el mapa a 9,7 km. Sin
# scan fresco el freno por LIDAR ya impide avanzar, así que en ese caso el
# robot siempre está quieto. Se espera ZUPT_TRAS_S tras la última orden
# porque el chasis desliza ~80 cm al soltar.
ZUPT_TRAS_S = 3.0
ZUPT_HZ = 10


async def _bucle_zupt():
    while True:
        try:
            if actuator_proto is not None and actuator_proto.segundos_quieto() > ZUPT_TRAS_S:
                ros_bridge.publish_zupt()
        except Exception as exc:
            print(f"[zupt] falló: {type(exc).__name__}: {exc}")
        await asyncio.sleep(1.0 / ZUPT_HZ)


def _autonomia_sin_mision() -> bool:
    """Patrulla o exploración en marcha, SIN contar `mission_active`. La visión activa la usa como candado: la misión de plantas
    detiene el explorador y entonces arranca la visión; con `mission_active` dentro del candado la visión NUNCA arrancaba en
    misión («exploración/patrulla activa: páralas primero») y el robot se quedaba en un bucle de escaneos (2026-10-23)."""
    return bool((patrol is not None and patrol.activo) or (explorador is not None and explorador.activo))


def _autonomia_activa() -> bool:
    """Algo que no es el operador está moviendo (o puede mover) el robot."""
    return bool(state.mission_active
                or (patrol is not None and patrol.activo)
                or (explorador is not None and explorador.activo))


def _barrido_activo() -> bool:
    """¿Activo el barrido de tilt + freno proactivo? (flag de config, OFF def.)"""
    return bool(state.config["autonomia"].get("barrido_tilt"))


def _barrido_estable() -> bool:
    """Con el barrido activo, ¿el servo está quieto en un ángulo conocido del
    barrido? Entonces la geometría (corregida por tilt) vale para suelo_libre."""
    if not _barrido_activo():
        return False
    st = state.actuator_status or {}
    if not st or st.get("scanning") or time.time() - st.get("_received_at", 0) > 1.0:
        return False
    ang = st.get("servo_angle")
    if ang is None:
        return False
    off = state.config["calibration"]["servo_offset_deg"]
    return min(abs(ang - (SERVO_LEJOS_DEG + off)),
               abs(ang - (SERVO_CERCA_DEG + off))) <= SERVO_TOLERANCIA_DEG


def _detener_autonomia():
    """Lo mismo que el botón de parar misión: la patrulla y el explorador
    primero (si no, vuelven a mandar velocidad o una meta a Nav2)."""
    if patrol is not None:
        patrol.detener()
    if explorador is not None:
        explorador.detener()
    ros_bridge.cancel_nav_goal()
    state.mission_active = False
    if actuator_proto is not None:
        actuator_proto.emergency_stop()


MODO_SEGURO_HZ = 5


def _imu_vuelco() -> dict:
    """Acelerómetro (g) y velocidad de balanceo (gyroY, eje de avance) del
    último paquete del IMU, para que modo_seguro detecte inclinación/vuelco.
    Vacío si aún no hay IMU: así no dispara nada."""
    env = state.env_telemetry or {}
    if "accelZ" not in env or not _imu_fresco():
        return {}
    return {
        "acc": (env.get("accelX", 0.0), env.get("accelY", 0.0), env.get("accelZ", 0.0)),
        "giro_roll_dps": env.get("gyroY", 0.0),
    }


def _entradas_modo_seguro() -> dict:
    ahora = time.time()
    vistos = health._last_seen
    edad = lambda k: (ahora - vistos[k]) if k in vistos else None
    return {
        "moviendose": actuator_proto is not None and not actuator_proto.en_reposo(),
        "autonomia": _autonomia_activa(),
        "edad_lidar_s": edad("lidar"),
        "edad_c3_s": edad("esp32c3"),
        "atascado": monitor_desliz.atascado,
        "patinaje": monitor_desliz.patinaje,
        "temp_c": (state.system_stats or {}).get("temp_c"),
        "ekf_var_vx": (ros_fused_pose or {}).get("var_vx") if ros_bridge.available else None,
        "mapa_corrupto": bool((ros_last_map or {}).get("corrupto")),
        **_imu_vuelco(),
    }


def estado_seguridad() -> dict:
    return {**modo_seg.estado(), "deslizamiento": monitor_desliz.estado()}


async def _bucle_modo_seguro():
    """Monitores de fallos a 5 Hz (ver modo_seguro.py). Al ENTRAR en SEGURO
    detiene la autonomía, avisa al dashboard y a Telegram (vía la Pi)."""
    nivel_previo = modo_seg.nivel
    ultimo_envio = 0.0
    while True:
        try:
            if not modo_seguro_habilitado:
                # Modo seguro desactivado: ignorar evaluación
                nivel = NORMAL
                est = {"nivel": NORMAL, "motivos": [], "factor": 1.0}
            else:
                est = modo_seg.evaluar(_entradas_modo_seguro())
                nivel = est["nivel"]
            if nivel != nivel_previo:
                motivo = "; ".join(est["motivos"]) or "todo en orden"
                print(f"[seguridad] {nivel_previo} -> {nivel}: {motivo}")
                if nivel == SEGURO:
                    _detener_autonomia()
                    await state.broadcast({"type": "toast", "data": {
                        "kind": "error", "title": "MODO SEGURO",
                        "message": f"Robot detenido: {motivo}. Revisar y rearmar."}})
                    _schedule(_avisar_al_gateway({
                        "tipo": "seguridad", "titulo": "AgroBot en MODO SEGURO",
                        "mensaje": motivo}))
                elif nivel == PRECAUCION:
                    await state.broadcast({"type": "toast", "data": {
                        "kind": "warning", "title": "Precaución",
                        "message": f"Velocidad reducida: {motivo}"}})
                nivel_previo = nivel
                ultimo_envio = 0.0
            if time.monotonic() - ultimo_envio > 2.0:
                ultimo_envio = time.monotonic()
                await state.broadcast({"type": "seguridad", "data": estado_seguridad()})
        except Exception as exc:
            print(f"[seguridad] bucle falló: {type(exc).__name__}: {exc}")
        await asyncio.sleep(1.0 / MODO_SEGURO_HZ)


async def seguridad_get_handler(request: web.Request):
    return web.json_response(estado_seguridad())


async def seguridad_rearmar_handler(request: web.Request):
    ok, msg = modo_seg.rearmar()
    await state.broadcast({"type": "toast", "data": {
        "kind": "success" if ok else "warning", "title": "Modo seguro", "message": msg}})
    return web.json_response({"ok": ok, "message": msg, **estado_seguridad()},
                             status=200 if ok else 409)


async def modo_seguro_toggle_handler(request: web.Request):
    """Activa/desactiva el modo seguro completamente."""
    global modo_seguro_habilitado
    try:
        data = await request.json()
        nuevo_estado = data.get("habilitado")
        if not isinstance(nuevo_estado, bool):
            return web.json_response({"error": "habilitado debe ser true/false"}, status=400)

        modo_seguro_habilitado = nuevo_estado
        state.config = config_store.update_section("safety", {"modo_seguro_activo": nuevo_estado})
        estado = "DESACTIVADO" if not nuevo_estado else "ACTIVADO"
        print(f"[seguridad] modo seguro {estado}", flush=True)

        await state.broadcast({"type": "toast", "data": {
            "kind": "warning",
            "title": "Modo Seguro",
            "message": f"Modo seguro {estado}"}})

        return web.json_response({
            "ok": True,
            "modo_seguro_habilitado": modo_seguro_habilitado,
            "estado": estado
        })
    except Exception as e:
        return web.json_response({"error": str(e)}, status=400)


async def _bucle_rampa():
    """Hace avanzar la rampa del actuador a 20 Hz, llegue o no un comando."""
    while True:
        try:
            if actuator_proto is not None:
                actuator_proto.tick()
        except Exception as exc:
            print(f"[rampa] tick falló: {type(exc).__name__}: {exc}")
        await asyncio.sleep(1.0 / CMD_SEND_HZ)


# Con el adaptador USB CP2102N el LIDAR se caía del bus (4 veces en 15 min el
# 2026-09-19) y sllidar_node quedaba bloqueado sobre el puerto viejo: el
# `respawn` nunca actuaba y /scan callaba hasta reiniciar ROS. Sin scans el
# freno por LIDAR bloquea adelante/atrás. Matar el driver colgado deja que
# el respawn lo relance.
# El puerto se elige como en scripts/elegir_puerto_lidar.sh: el USB si está
# enchufado, si no la UART del conector (pines 8/10). La UART existe siempre,
# así que "puerto presente" no dice que el LIDAR esté conectado: por eso los
# reinicios se espacian si no sirven.
LIDAR_USB = Path("/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_"
                 "Bridge_Controller_5e723775cd73ef11b311cf8c8fcc3fa0-if00-port0")
LIDAR_UART = Path("/dev/ttyTHS1")


def _puerto_lidar() -> Path:
    """El mismo criterio que scripts/elegir_puerto_lidar.sh: el adaptador
    conocido, si no cualquier CP210x por by-id, si no la UART."""
    if LIDAR_USB.exists():
        return LIDAR_USB
    for otro in sorted(Path("/dev/serial/by-id").glob("usb-Silicon_Labs_CP210*")):
        return otro
    return LIDAR_UART


LIDAR_MUDO_S = 4.0          # sin scan este tiempo con el puerto presente = colgado
LIDAR_ENTRE_REINICIOS_S = 20.0   # arranque del driver + respawn_delay, con holgura
LIDAR_ESPERA_MAX_S = 300.0       # tope de la espera cuando reiniciar no lo trae
_SLLIDAR_PATRON = "sllidar_ros2/lib/sllidar_ros2/sllidar_node"


def _matar_sllidar() -> bool:
    """SIGTERM y, si sigue vivo (bloqueado en el puerto), SIGKILL."""
    r = subprocess.run(["pkill", "-TERM", "-f", _SLLIDAR_PATRON], capture_output=True)
    if r.returncode != 0:
        return False
    time.sleep(1.5)
    subprocess.run(["pkill", "-KILL", "-f", _SLLIDAR_PATRON], capture_output=True)
    return True


_detector_reloj = DetectorSalto()


def _ocupado() -> dict:
    """¿Está el robot haciendo algo que no debe interrumpirse (autonomía, misión, pulso, movimiento reciente)? Lo consulta agrobot-reloj-vigia antes de reiniciar ROS tras un salto de reloj:
    reiniciar la pila en plena misión borra el mapa y salta la pose (27-oct: saltos de 30 y 24 m)."""
    motivos = []
    try:
        modo = _modo_auto_activo()
        if modo:
            motivos.append(f"autonomía: {modo}")
        if conductor is not None and conductor.activo:
            motivos.append("pulso en curso")
        if _maniobra_giro_tarea is not None and not _maniobra_giro_tarea.done():
            motivos.append("maniobra de giro")
        if gestor_misiones is not None and gestor_misiones.estado in ("en curso", "terminando"):
            motivos.append(f"misión: {gestor_misiones.mision}")
        if actuator_proto is not None and actuator_proto.segundos_quieto() < 10.0:
            motivos.append("movimiento reciente")
    except Exception as exc:
        motivos.append(f"no se pudo comprobar ({type(exc).__name__})")
    return {"ocupado": bool(motivos), "motivos": motivos}


async def ocupado_handler(request: web.Request):
    """GET /api/ocupado — {"ocupado": bool, "motivos": [...]}."""
    return web.json_response(_ocupado())


async def _vigia_reloj():
    """Detecta saltos del reloj de pared (NTP sin RTC). Los anota (~/.agrobot/reloj_saltos.jsonl) y avisa al operador; el reinicio de ROS lo decide agrobot-reloj-vigia, que espera a que
    el robot esté en reposo (/api/ocupado). Los análisis posteriores deben usar el tiempo monótono, no t_unix."""
    while True:
        await asyncio.sleep(1.0)
        try:
            delta = _detector_reloj.paso(time.time(), time.monotonic())
            if delta is None:
                continue
            msg = f"El reloj de la Jetson saltó {delta:+.1f} s (sincronización NTP). La pose del SLAM puede haber saltado."
            print(f"[reloj] {msg}", flush=True)
            try:
                with open(Path.home() / ".agrobot" / "reloj_saltos.jsonl", "a", encoding="utf-8") as fh:
                    fh.write(json.dumps({**_detector_reloj.saltos[-1], "iso": time.strftime("%Y-%m-%d %H:%M:%S")}, ensure_ascii=False) + "\n")
            except OSError:
                pass
            _evento("reloj", {"delta_s": round(delta, 1)})
            await state.broadcast({"type": "toast", "data": {"kind": "warning", "title": "Salto de reloj", "message": msg}})
        except Exception as exc:
            print(f"[reloj] vigía: {type(exc).__name__}: {exc}", flush=True)


async def _vigia_lidar():
    ultimo_reinicio = time.monotonic()
    fallidos = 0                  # reinicios seguidos que no trajeron scans
    puerto_antes = _puerto_lidar()
    while True:
        await asyncio.sleep(1.0)
        try:
            if not ros_bridge.available:
                continue
            ahora = time.monotonic()
            if lidar_reactive.edad_scan_s() < LIDAR_MUDO_S:
                if fallidos:
                    print(f"[lidar] vuelven los scans tras {fallidos} reinicios", flush=True)
                fallidos = 0
                continue
            espera = min(LIDAR_ESPERA_MAX_S, LIDAR_ENTRE_REINICIOS_S * (2 ** min(fallidos, 4)))
            puerto = _puerto_lidar()
            # Si cambió el puerto (se volvió a enchufar el USB tras caer a la
            # UART) se relanza YA: con la espera acumulada, el 2026-09-30 el
            # LIDAR podía tardar hasta 5 min en volver después de enchufarlo.
            if puerto != puerto_antes:
                print(f"[lidar] el puerto cambió: {puerto_antes} -> {puerto}; relanzo el driver",
                      flush=True)
                puerto_antes = puerto
                fallidos = 0
                ultimo_reinicio = 0.0
                espera = 0.0
            if not puerto.exists() or ahora - ultimo_reinicio < espera:
                continue
            ultimo_reinicio = ahora
            if await asyncio.to_thread(_matar_sllidar):
                fallidos += 1
                print(f"[lidar] sin scans con {puerto} presente — driver relanzado "
                      f"(intento {fallidos}, siguiente en {espera:.0f} s)", flush=True)
                if fallidos <= 2:
                    via = "USB" if puerto == LIDAR_USB else "UART (pines 8/10)"
                    await state.broadcast({"type": "toast", "data": {
                        "kind": "warning", "title": "LIDAR sin datos",
                        "message": (f"Se relanzó el driver del LIDAR ({via}). Si sigue sin "
                                    f"datos, revisá cable y alimentación."),
                    }})
        except Exception as exc:
            print(f"[lidar] vigía falló: {type(exc).__name__}: {exc}", flush=True)


# =============================================
# ===== MANEJO MANUAL POR PULSOS =====
# =============================================
def _suelo_para_grabar() -> dict | None:
    r = suelo_libre.ultimo
    if r is None:
        return None
    return {"hueco": suelo_libre.hueco_delante(state.config["slam"]["robot_ancho_cm"] / 100.0),
            "suelo_pct": r["suelo_pct"], "objeto_en_referencia": r["objeto_en_referencia"]}


async def _bucle_grabador():
    """Graba la sesión a 10 Hz en un hilo aparte (escribir disco, comprimir
    scans y codificar JPEG no debe frenar el bucle del control)."""
    while True:
        await asyncio.sleep(1.0 / TELEMETRIA_HZ)
        try:
            await asyncio.to_thread(grabador.tick)
        except Exception as exc:
            print(f"[grabador] tick falló: {type(exc).__name__}: {exc}", flush=True)
            await asyncio.sleep(2.0)


SUELO_HZ = 8.0


async def _bucle_suelo_libre():
    """Analiza el suelo delante con cada fotograma (~8 Hz). Solo con la
    cámara al frente y quieta: con el servo girado la geometría no vale y un
    obstáculo saldría a una distancia falsa."""
    ultimo_n = None
    recorrido = 0.0        # distancia avanzada, para el paralaje de altura
    while True:
        await asyncio.sleep(1.0 / SUELO_HZ)
        try:
            if cv2 is None or camera is None:
                continue
            dato = camera.ultimo_frame()
            if dato is None or dato[2] == ultimo_n:
                continue
            ultimo_n = dato[2]
            al_frente, _ = _servo_al_frente()
            # Con el barrido activo la cámara mira a propósito al suelo cercano o
            # al sendero: si está quieta en un ángulo conocido, la geometría
            # corregida por tilt (ver _geometria_camara) sí vale.
            st_srv = state.actuator_status or {}
            # 2026-10-22: con el servo QUIETO en cualquier ángulo conocido la geometría vale (_geometria_camara suma el tilt del servo
            # siempre); antes solo se aceptaba HOME y con la cámara a 64° (mirando el suelo cercano) no había ningún dato.
            servo_quieto = bool(st_srv) and not st_srv.get("scanning") and time.time() - st_srv.get("_received_at", 0) < 1.0 \
                and st_srv.get("servo_angle") is not None
            if not al_frente and not _barrido_estable() and not servo_quieto:
                suelo_libre.ultimo = None          # nada de datos con la cámara moviéndose
                continue
            await asyncio.to_thread(suelo_libre.analizar, cv2, dato[0], _geometria_camara())
            # Paralaje: con el barrido activo, acumula avance y alimenta el
            # estimador de altura con la distancia al obstáculo que ve la cámara.
            if _barrido_activo():
                recorrido += abs(getattr(actuator_proto, "_ramp_v", 0.0)) / SUELO_HZ
                hueco = suelo_libre.hueco_delante(
                    state.config["slam"]["robot_ancho_cm"] / 100.0)
                estimador_altura.alimentar(recorrido, hueco, time.monotonic())
        except Exception as exc:
            print(f"[suelo] análisis falló: {type(exc).__name__}: {exc}", flush=True)
            await asyncio.sleep(1.0)


async def _bucle_barrido_tilt():
    """Mueve el servo de tilt en barrido (lejos=sendero / cerca=suelo) cuando el
    barrido está activo y hay autonomía. Con el flag OFF no toca el servo: lo
    maneja el operador. A ~5 Hz, suficiente para el dwell del barrido."""
    t0 = time.monotonic()
    anterior = None
    while True:
        await asyncio.sleep(0.2)
        try:
            if not _barrido_activo() or not _autonomia_activa() or actuator_proto is None:
                anterior = None
                continue
            v = abs(getattr(actuator_proto, "_ramp_v", 0.0))
            ang = clamp_servo(barrido_tilt_ctrl.angulo(time.monotonic() - t0, v))
            if ang != anterior:
                state.servo_angle = ang
                actuator_proto.send_actuator_update()
                anterior = ang
        except Exception as exc:
            print(f"[barrido] falló: {type(exc).__name__}: {exc}", flush=True)
            await asyncio.sleep(1.0)


async def suelo_vista_handler(request: web.Request):
    """GET /api/suelo/vista.jpg — suelo libre por cámara: verde = suelo,
    rojo = base de obstáculos, recuadro = franja de referencia."""
    if cv2 is None or camera is None:
        return web.json_response({"error": "cámara no disponible"}, status=503)
    dato = camera.ultimo_frame()
    if dato is None:
        return web.json_response({"error": "sin fotograma"}, status=503)
    img = cv2.resize(suelo_libre.dibujar(cv2, dato[0]), (480, 360), interpolation=cv2.INTER_NEAREST)
    ok, jpg = cv2.imencode(".jpg", img)
    return web.Response(body=jpg.tobytes(), content_type="image/jpeg",
                        headers={"Cache-Control": "no-store"})


async def suelo_estado_handler(request: web.Request):
    """GET /api/suelo — resumen del suelo libre (sin la máscara)."""
    r = suelo_libre.ultimo
    if r is None:
        return web.json_response({"activo": False})
    return web.json_response({
        "activo": True, "edad_s": round(time.monotonic() - r["t"], 2),
        "hueco_delante_m": suelo_libre.hueco_delante(state.config["slam"]["robot_ancho_cm"] / 100.0),
        "obstaculos": len(r["puntos"]), "objeto_en_referencia": r["objeto_en_referencia"],
        "referencia_dudosa": r["referencia_dudosa"], "suelo_pct": r["suelo_pct"],
        "distancia_min_visible_m": r["distancia_min_visible_m"]})


# ----- Evasión autónoma: decide con evasion.py, se mueve con paso_robotico -----
evasion_estado = {"activo": False, "decisiones": 0, "ultima": None, "resultado": None}
_evasion_tarea: asyncio.Task | None = None
RUTA_EVASION = Path.home() / ".agrobot" / "evasion"


def _puntos_camara() -> list[dict]:
    r = suelo_libre.ultimo
    if r is None or time.monotonic() - r["t"] > 0.6:
        return []
    puntos = list(r["puntos"])
    if r.get("objeto_en_referencia"):
        # Algo pegado delante que la cámara no alcanza a medir: se pone justo
        # en el frontal para que ninguna decisión avance hacia ahí.
        ml = state.config["slam"]["robot_largo_cm"] / 200.0
        puntos += [{"angle_deg": a, "distance_m": ml + 0.05} for a in (-10.0, 0.0, 10.0)]
    return puntos


async def _bucle_evasion(segundos: float, objetivo_deg: float):
    cfg_slam = state.config["slam"]
    umbral = (cfg_slam["margen_parada_cm"] + cfg_slam["deslizamiento_cm"]) / 100.0
    fin = time.monotonic() + segundos
    giros_fallidos = 0
    RUTA_EVASION.mkdir(parents=True, exist_ok=True)
    archivo = RUTA_EVASION / time.strftime("evasion_%Y%m%d_%H%M%S.jsonl")
    evasion_estado.update(activo=True, decisiones=0, ultima=None, resultado=None)
    resultado = "tiempo"
    try:
        while time.monotonic() < fin:
            if lidar_reactive.edad_scan_s() > 1.0:
                resultado = "sin_lidar"
                break
            d = evasion_mod.decidir(lidar_reactive.last_scan, _puntos_camara(),
                                    objetivo_deg=objetivo_deg, umbral_m=umbral,
                                    robot_largo_m=cfg_slam["robot_largo_cm"] / 100.0,
                                    robot_ancho_m=cfg_slam["robot_ancho_cm"] / 100.0)
            resumen = {k: v for k, v in d.items() if k != "libre"}
            evasion_estado["decisiones"] += 1
            evasion_estado["ultima"] = resumen
            await state.broadcast({"type": "evasion", "data": evasion_estado})
            if d["accion"] == "atascado":
                resultado = "atascado"
                break
            if d["accion"] == "avanzar":
                res = await paso_robotico.avanzar(d["cm"] / 100.0)
            elif d["accion"] == "retroceder":
                res = await paso_robotico.avanzar(-d["cm"] / 100.0)
            else:
                res = await paso_robotico.girar(d["grados"])
                # El giro en sitio rinde muy poco en este chasis (2026-10-02):
                # si dos giros seguidos no progresan, se para y se avisa.
                giros_fallidos = giros_fallidos + 1 if res["resultado"] == "sin_progreso" else 0
            with open(archivo, "a") as f:
                f.write(json.dumps({"t": time.strftime("%H:%M:%S"), "decision": resumen,
                                    "paso": res, "camara_pts": len(_puntos_camara())},
                                   ensure_ascii=False) + "\n")
            if giros_fallidos >= 2:
                resultado = "no_gira"
                break
            if res["resultado"] in ("sin_lidar",):
                resultado = res["resultado"]
                break
    except asyncio.CancelledError:
        resultado = "detenido"
        raise
    finally:
        conductor.parar()
        evasion_estado.update(activo=False, resultado=resultado)
        _schedule(state.broadcast({"type": "evasion", "data": evasion_estado}))
        print(f"[evasion] fin: {resultado} tras {evasion_estado['decisiones']} decisiones", flush=True)


def _detener_evasion():
    global _evasion_tarea
    if _evasion_tarea is not None and not _evasion_tarea.done():
        _evasion_tarea.cancel()
    _evasion_tarea = None


async def evasion_start_handler(request: web.Request):
    """POST /api/evasion/start {"segundos": 120, "objetivo_deg": 0}"""
    global _evasion_tarea
    if paso_robotico is None:
        return web.json_response({"error": "no disponible"}, status=503)
    if (_evasion_tarea is not None and not _evasion_tarea.done()) \
            or (patrol is not None and patrol.activo) or (explorador is not None and explorador.activo):
        return web.json_response({"error": "ya hay autonomía en marcha"}, status=409)
    try:
        body = await request.json()
    except Exception:
        body = {}
    segundos = max(5.0, min(1800.0, float(body.get("segundos", 120))))
    objetivo = float(body.get("objetivo_deg", 0.0))
    _evasion_tarea = asyncio.create_task(_bucle_evasion(segundos, objetivo))
    return web.json_response({"ok": True, "segundos": segundos, "objetivo_deg": objetivo})


async def evasion_stop_handler(request: web.Request):
    _detener_evasion()
    if paso_robotico is not None:
        paso_robotico.detener()
    return web.json_response({"ok": True, **evasion_estado})


async def evasion_get_handler(request: web.Request):
    return web.json_response(evasion_estado)


def _parada_total(origen: str, remote: bool = True, emergencia: bool = True, aviso: bool = True) -> list[str]:
    """PARADA DE EMERGENCIA (o, con emergencia=False, el STOP de la sección de manejo: misma parada,
    pero se avisa como «Parada» informativa y no cuenta como emergencia): detiene TODO lo que puede mover el robot.

    Un solo sitio para todos los caminos (botón del dashboard por HTTP o
    WebSocket, /api/drive "parar", downlink LoRa). Antes cada camino paraba
    una lista distinta y el botón no conocía la evasión ni el paso robótico:
    el lazo de pasos veía cortado su pulso, esperaba quieto, medía y mandaba
    el SIGUIENTE (2026-10-02, "no vale el stop de emergencia").

    Orden: primero lo que DECIDE (para que no vuelva a mandar nada), luego lo
    que ejecuta, y al final la trama de parada al C3. Síncrona y sin await:
    sirve también desde el callback UDP del LoRa."""
    hecho = []
    pasos = [
        ("misiones", lambda: gestor_misiones is not None and gestor_misiones.detener("parada")),
        ("mision_plantas", lambda: mision_plantas is not None and mision_plantas.detener("parada")),
        ("vision_activa", lambda: vision_activa is not None and vision_activa.cancelar("parada")),
        ("evasion", _detener_evasion),
        ("paso_robotico", lambda: paso_robotico is not None and paso_robotico.detener()),
        ("exploracion", lambda: explorador is not None and explorador.detener()),
        ("patrulla", lambda: patrol is not None and patrol.detener()),
        ("nav2", ros_bridge.cancel_nav_goal),
        ("saludo", lambda: saludo_tarea is not None and not saludo_tarea.done() and saludo_tarea.cancel()),
        ("maniobra_giro", lambda: _maniobra_giro_tarea is not None and not _maniobra_giro_tarea.done() and _maniobra_giro_tarea.cancel()),
        ("pulsos", lambda: conductor is not None and conductor.parar()),
        ("motores", lambda: actuator_proto is not None and actuator_proto.emergency_stop()),
    ]
    for nombre, fn in pasos:
        try:
            fn()
            hecho.append(nombre)
        except Exception as exc:          # una pieza rota no impide parar el resto
            print(f"[emergencia] {nombre} falló al parar: {type(exc).__name__}: {exc}", flush=True)
    state.mission_active = False
    print(f"[{'emergencia' if emergencia else 'parada'}] PARADA TOTAL ({origen}): {', '.join(hecho)}", flush=True)
    _schedule(state.broadcast({"type": "mission_state", "data": {"active": False}}))
    if aviso:
        _schedule(state.broadcast({"type": "toast", "data": {
            "kind": "error" if emergencia else "info",
            "title": "PARADA DE EMERGENCIA" if emergencia else "Parada de manejo",
            "message": f"Todo detenido ({origen})."}}))
    if remote:
        _schedule(c3_setup.stop())
    return hecho


# =============================================
# ===== MANEJO ASISTIDO (mando manual sobre la autonomía) =====
# =============================================
asistido = Asistido(Path.home() / ".agrobot" / "aprendizaje" / "intervenciones")


def _modo_auto_activo() -> str | None:
    """Qué autonomía está moviendo el robot ahora (None = ninguna). Orden: la más «alta» primero."""
    if mision_plantas is not None and mision_plantas.estado in ("explorando", "inspeccionando"):
        return "mision"
    if patrol is not None and patrol.activo:
        return "patrulla"
    if explorador is not None and explorador.activo:
        return "exploracion"
    if ros_bridge.available and ros_bridge.estado_meta() in ("enviada", "en_curso"):
        return "nav2"
    if vision_activa is not None and getattr(vision_activa, "estado", "inactiva") not in ("inactiva", "pausa"):
        return "vision_activa"
    if paso_robotico is not None and paso_robotico.estado.get("activo"):
        return "paso_robotico"
    return None


def _distancias_lidar() -> dict:
    try:
        return lidar_reactive.distancias(state.config["slam"])
    except Exception:
        return {}


def _pausar_autonomia():
    """PAUSA (no apaga) la autonomía: misión, exploración y patrulla siguen «activas» —la interfaz no pasa a manual— pero
    sueltan el mando; se cancelan la visión, la evasión, el paso robótico, la meta de Nav2 y los pulsos en curso."""
    pasos = [
        ("mision_plantas", lambda: mision_plantas is not None and mision_plantas.pausar()),
        ("exploracion", lambda: explorador is not None and explorador.pausar()),
        ("patrulla", lambda: patrol is not None and patrol.pausar()),
        ("vision_activa", lambda: vision_activa is not None and vision_activa.estado != "inactiva" and setattr(vision_activa, "pausa_externa", True)),
        ("evasion", _detener_evasion),
        ("paso_robotico", lambda: paso_robotico is not None and paso_robotico.detener()),
        ("nav2", ros_bridge.cancel_nav_goal),
        ("pulsos", lambda: conductor is not None and conductor.parar()),
        ("motores", lambda: actuator_proto is not None and actuator_proto.emergency_stop()),
    ]
    for nombre, fn in pasos:
        try:
            fn()
        except Exception as exc:
            print(f"[asistido] {nombre} falló al pausar: {type(exc).__name__}: {exc}", flush=True)


def _tomar_mando(origen: str) -> str | None:
    """El operador da una orden manual: si hay autonomía en marcha se PAUSA (queda registrado qué hacía y qué ordenaba) y el
    mando pasa a él un momento; vuelve sola (`_vigia_asistido`) o con el botón. Devuelve el modo interrumpido, o None."""
    if asistido.activa:
        return asistido.modo_previo                     # ya está en manos del operador
    modo = _modo_auto_activo()
    if modo is None:
        return None
    st = state.actuator_status or {}
    meta = (state.config.get("navigation") or {}).get("last_goal") if modo == "nav2" else None
    asistido.tomar(modo, (st.get("v_cmd") or 0.0, st.get("omega_cmd") or 0.0), meta)
    _pausar_autonomia()
    _schedule(state.broadcast({"type": "asistido", "data": asistido.estado()}))
    return modo


def _registrar_orden_manual(v: float, w: float, origen: str):
    if asistido.activa:
        asistido.muestra(v, w, current_pose(), _distancias_lidar(), origen)


async def asistido_estado_handler(request: web.Request):
    """GET /api/asistido — estado del mando, resumen de correcciones y ajustes sugeridos."""
    ev = asistido.leer()
    espera = _asistido_reanuda_s()
    reanuda_en = (max(0.0, round(espera - asistido.segundos_sin_orden(), 1)) if asistido.activa and espera > 0 else None)
    return web.json_response({**asistido.estado(), "autonomia_activa": _modo_auto_activo(),
                              "reanuda_s": espera, "reanuda_en_s": reanuda_en,
                              "resumen": asistido.resumen(ev), "sugerencias": asistido.sugerencias(state.config, ev)})


async def _reanudar_autonomia() -> tuple[bool, str, str | None]:
    """Devuelve el mando a la autonomía interrumpida (sin barrido inmediato y, en la misión, conservando las plantas)."""
    modo = asistido.modo_previo
    if not asistido.activa or modo is None:
        return False, "no hay ninguna autonomía interrumpida", None
    meta = asistido.meta_previa
    en_pausa = any(getattr(o, "pausado", False) for o in (mision_plantas, explorador, patrol) if o is not None)
    if _modo_auto_activo() is not None and not en_pausa:
        return False, "ya hay una autonomía en marcha", modo
    conductor.parar()
    ok, msg = False, "este modo no se puede reanudar solo"
    objetos = [o for o in (mision_plantas, explorador, patrol) if o is not None and getattr(o, "pausado", False)]
    if vision_activa is not None and getattr(vision_activa, "pausa_externa", False):
        vision_activa.pausa_externa = False                # la inspección sigue donde iba
        if modo == "vision_activa" and not objetos:
            ok, msg = True, "la visión activa continúa"
    if modo != "nav2" and objetos:
        for o in objetos:                                # en pausa: se CONTINÚA donde se quedó (plantas, ruta, contadores)
            o.continuar()
        ok, msg = True, "continúa donde se quedó"
    elif modo == "exploracion" and explorador is not None:
        explorador.escaneo_al_iniciar = False
        try:
            ok, msg = explorador.iniciar()
        finally:
            explorador.escaneo_al_iniciar = True
    elif modo == "patrulla" and patrol is not None:
        ok, msg = patrol.iniciar()
    elif modo == "mision" and mision_plantas is not None:
        ok, msg = mision_plantas.reanudar()
        if ok:
            mision_plantas.arrancar_tarea()
    elif modo == "nav2" and meta:
        ok = ros_bridge.send_nav_goal(float(meta["x"]), float(meta["y"]), float(meta.get("theta_deg", 0.0)))
        msg = "meta reenviada a Nav2" if ok else "Nav2 no respondió"
    if ok:
        asistido.soltar()
        state.mission_active = modo in ("exploracion", "mision", "patrulla")
    await state.broadcast({"type": "asistido", "data": asistido.estado()})
    return bool(ok), msg, modo


async def asistido_reanudar_handler(request: web.Request):
    """POST /api/asistido/reanudar — devuelve el mando a la autonomía que se interrumpió."""
    ok, msg, modo = await _reanudar_autonomia()
    return web.json_response({"ok": ok, "modo": modo, "msg": msg}, status=200 if ok else 409)


def _asistido_reanuda_s() -> float:
    try:
        return float((state.config.get("autonomia") or {}).get("asistido_reanuda_s", 2.5))
    except (TypeError, ValueError):
        return 2.5


async def _aprender_de_correcciones():
    """Las correcciones con los botones enseñan al robot: si se repiten, aplica solo el ajuste que lo hace MÁS cauto
    (parar antes, ir más despacio, mirar más a los lados). Los que lo harían menos cauto quedan como sugerencia manual."""
    if not (state.config.get("autonomia") or {}).get("asistido_aprende", True):
        return
    for sug in asistido.auto_aplicables(state.config):
        state.config = config_store.update_section(sug["seccion"], {sug["clave"]: sug["propuesto"]})
        asistido.marcar_aprendido(sug)
        print(f"[asistido] aprendido: {sug['seccion']}.{sug['clave']} {sug['actual']} -> {sug['propuesto']} ({sug['texto']})", flush=True)
        await state.broadcast({"type": "config_update", "data": state.config})
        await state.broadcast({"type": "toast", "data": {"kind": "info", "title": "Aprendí de tus correcciones",
                                                          "message": f"{sug['clave']}: {sug['actual']} → {sug['propuesto']}. {sug['texto']}"}})


async def _vigia_asistido():
    """Un toque de mando para corregir NO saca al robot del modo autónomo: pasados `autonomia.asistido_reanuda_s` segundos
    sin órdenes manuales (0 = nunca, hay que pulsar «Reanudar») la autonomía interrumpida vuelve sola."""
    while True:
        try:
            await asyncio.sleep(0.4)
            await _aprender_de_correcciones()
            espera = _asistido_reanuda_s()
            if asistido.activa and espera > 0 and asistido.segundos_sin_orden() >= espera and not conductor.activo:
                ok, msg, modo = await _reanudar_autonomia()
                if ok:
                    print(f"[asistido] reanudo «{modo}» tras {espera:g} s sin órdenes manuales", flush=True)
                    await state.broadcast({"type": "toast", "data": {"kind": "info", "title": "Autonomía reanudada",
                                                                      "message": f"«{modo}» sigue donde se quedó."}})
                elif modo is None or "no se puede reanudar" in msg:
                    asistido.soltar()               # nada que reanudar: se acaba la intervención en vez de reintentar siempre
                    await state.broadcast({"type": "asistido", "data": asistido.estado()})
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            print(f"[asistido] vigía falló: {type(exc).__name__}: {exc}", flush=True)
            await asyncio.sleep(2.0)


async def asistido_soltar_handler(request: web.Request):
    """POST /api/asistido/soltar — se queda en manual: la autonomía interrumpida NO se reanuda."""
    hecho = []
    if asistido.activa:
        hecho = _parada_total("mando manual: se queda en manual", remote=False, emergencia=False, aviso=False)
    modo = asistido.soltar()
    await state.broadcast({"type": "asistido", "data": asistido.estado()})
    return web.json_response({"ok": True, "modo_descartado": modo, "detenido": hecho})


async def asistido_aplicar_handler(request: web.Request):
    """POST /api/asistido/aplicar {"id": ...} — aplica UNA sugerencia (nunca se aplican solas)."""
    body = await request.json()
    sug = {s["id"]: s for s in asistido.sugerencias(state.config)}.get(body.get("id"))
    if sug is None:
        return web.json_response({"error": "esa sugerencia ya no está vigente"}, status=404)
    data = config_store.update_section(sug["seccion"], {sug["clave"]: sug["propuesto"]})
    state.config = data
    await state.broadcast({"type": "config_update", "data": state.config})
    await state.broadcast({"type": "toast", "data": {
        "kind": "success", "title": "Ajuste aplicado",
        "message": f"{sug['seccion']}.{sug['clave']}: {sug['actual']} → {sug['propuesto']}"}})
    return web.json_response({"ok": True, "aplicado": sug})


async def emergencia_handler(request: web.Request):
    """POST /api/emergencia — parada total por HTTP (el WebSocket se cae al
    bloquear la pantalla del móvil; una petición HTTP llega o da error)."""
    try:
        tipo = (await request.json()).get("tipo")
    except Exception:
        tipo = None
    if tipo == "soltar":
        # 2026-10-22: el dashboard perdió el foco (otra pestaña, el video, una captura, pantalla bloqueada). Antes eso era PARADA DE
        # EMERGENCIA y apagaba el modo autónomo a cada rato. Ahora solo se suelta el manejo MANUAL; la autonomía sigue.
        if _autonomia_activa() or (vision_activa is not None and vision_activa.estado != "inactiva") \
                or (mision_plantas is not None and mision_plantas.estado in ("explorando", "inspeccionando")):
            return web.json_response({"ok": True, "detenido": [], "nota": "autonomía activa: sigue"})
        hecho = []
        for nombre, fn in (("pulsos", lambda: conductor is not None and conductor.parar()),
                           ("motores", lambda: actuator_proto is not None and actuator_proto.emergency_stop())):
            try:
                fn()
                hecho.append(nombre)
            except Exception as exc:
                print(f"[soltar] {nombre}: {exc}", flush=True)
        return web.json_response({"ok": True, "detenido": hecho})
    manejo = tipo == "manejo"
    hecho = _parada_total("manejo HTTP" if manejo else "dashboard HTTP", emergencia=not manejo)
    return web.json_response({"ok": True, "detenido": hecho})


def _pulso_robotico(accion: str, escala: float, maniobra: bool):
    """Un pulso para paso_robotico, con el acelerador de la config."""
    ctrl = state.config["control"]
    conductor.ordenar(accion, v_max=ctrl["max_v_mps"] * escala,
                      omega_max=ctrl["max_omega_rps"] * escala, maniobra=maniobra)


_plagasx_cls_modelo = None


def _plagasx_clasificador():
    global _plagasx_cls_modelo
    if _plagasx_cls_modelo is None:
        from ultralytics import YOLO
        _plagasx_cls_modelo = YOLO(str(Path(__file__).resolve().parent / "models" / "agrobot_plagasx_cls.pt"))
    return _plagasx_cls_modelo


def _gyro_dps():
    """Velocidad angular Z del IMU en °/s (+ izquierda, ya con sesgo y nivel corregidos) o None si el dato no es fresco."""
    env = state.env_telemetry or {}
    ra = env.get("_received_at")
    if ra is None or time.time() - ra > 0.5 or env.get("gyroZ") is None:
        return None
    return float(env["gyroZ"])


_mision_vision_override: dict = {}
mision_plantas = None


def _dist_caja_m(xyxy, ancho, alto):
    """Distancia (m) desde la cara delantera hasta la BASE de la caja, por la geometría de la cámara (suelo plano). None si la base
    queda por encima del horizonte. Sirve para macetas bajas que el LIDAR (a 37,5 cm) no ve."""
    try:
        x1, y1, x2, y2 = xyxy
        # Con zoom digital la imagen es el recorte CENTRAL ampliado z veces: se devuelve el píxel al cuadro sin zoom
        # (antes la distancia por cámara salía mal en cuanto se hacía zoom, y la visión activa ahora lo usa al acercarse).
        z = max(1.0, float(getattr(camera, "zoom", 1.0) or 1.0)) if camera is not None else 1.0
        xc = (x1 + x2) / 2.0
        yb = min(y2, alto - 1)
        xc = (xc - ancho / 2.0) / z + ancho / 2.0
        yb = (yb - alto / 2.0) / z + alto / 2.0
        u = xc * suelo_mod.ANCHO / float(ancho)
        v = yb * suelo_mod.ALTO / float(alto)
        x, y = suelo_libre.a_suelo(u, v, _geometria_camara())
        d = float(math.hypot(float(x), float(y))) - state.config["slam"]["robot_largo_cm"] / 200.0
        return round(d, 2) if math.isfinite(d) and d > 0 else None
    except Exception:
        return None


def _poner_led(on: bool):
    state.led_on = bool(on)
    if actuator_proto is not None:
        actuator_proto.send_actuator_update()
    _schedule(state.broadcast({"type": "actuator_setpoint", "data": {"servo_angle": state.servo_angle, "led": state.led_on}}))


def _grabar_video(on: bool):
    """Inicia/detiene la grabación (la misma del botón del dashboard) desde código síncrono."""
    global video_recording, _video_task, _video_file
    if on and not video_recording and camera is not None:
        _video_file = f"agrobot_planta_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
        video_recording = True
        _video_task = asyncio.get_event_loop().create_task(_bucle_grabar_video(_video_file))
        print(f"[VIDEO] misión: grabando {_video_file}", flush=True)
    elif not on and video_recording:
        video_recording = False          # el bucle cierra el archivo solo


_servo_bajo = ServoBajo()
_zoom_auto = ZoomAuto()
_zoom_puesto_por_auto = False


async def _bucle_zoom_auto():
    """Zoom digital automático al acercarse a una planta (zoom_auto.py): sube hasta que la planta ocupa ~55 % de la imagen mientras la
    visión activa centra, se acerca, encuadra y analiza; vuelve a 1× al terminar. No toca el zoom si el operador tiene el mando o lo
    puso él (solo suelta el zoom que puso este lazo)."""
    global _zoom_puesto_por_auto
    while True:
        try:
            await asyncio.sleep(0.25)
            if camera is None or vision_activa is None:
                continue
            cfg = {**ZOOM_DEFECTOS, **(state.config.get("vision_activa") or {})}
            _zoom_auto.cfg = cfg
            estado = vision_activa.estado
            camera.guardar_nitidos = estado in ("analizando", "escaneando")
            if not cfg["zoom_auto"] or asistido.activa:
                continue
            if estado == "inactiva" and not _zoom_puesto_por_auto:
                continue
            f = camera.ultimo_frame()
            if f is None:
                continue
            h, w = f[0].shape[:2]
            with camera._lock:
                lista, ts = list(camera._cajas), camera._cajas_ts
            caja = None
            if ts is not None and time.time() - ts < 0.8:
                cand = [b for b in lista if float(b.get("confidence", 0)) >= 0.35
                        and any(k in str(b.get("class", "")).lower() for k in ("planta", "col", "maceta"))]
                if cand:
                    caja = max(cand, key=lambda b: (b["xyxy"][2] - b["xyxy"][0]) * (b["xyxy"][3] - b["xyxy"][1]))["xyxy"]
            nuevo = _zoom_auto.paso(estado, caja, w, h, float(camera.zoom), time.monotonic())
            if nuevo is None:
                if estado == "inactiva" and camera.zoom <= 1.02:
                    _zoom_puesto_por_auto = False
                continue
            camera.set_zoom(nuevo)
            _zoom_puesto_por_auto = nuevo > 1.02
            if hasattr(vision_activa, "_esperar"):
                vision_activa._esperar(0.7)                  # el zoom reabre/reencuadra: que no decida con cajas de antes
            await state.broadcast({"type": "zoom_auto", "data": {"zoom": nuevo, "estado": estado}})
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            print(f"[zoom_auto] falló: {type(exc).__name__}: {exc}", flush=True)
            await asyncio.sleep(2.0)


async def _bucle_servo_macetas():
    """En autonomía, si una maceta está baja en la imagen (o cortada por el borde inferior) la cámara baja para no perderla antes
    de tocarla (servo_bajo.py). No actúa con el operador al mando, durante barridos de escaneo ni con la visión activa trabajando."""
    while True:
        try:
            await asyncio.sleep(0.5)
            if not state.config.get("autonomia", {}).get("servo_baja_macetas", True):
                continue
            if asistido.activa or camera is None or not _autonomia_activa():
                continue
            if vision_activa is not None and vision_activa.estado != "inactiva":
                continue
            if patrol is not None and getattr(patrol, "_escaneando", False):
                continue
            f = camera.ultimo_frame()
            if f is None:
                continue
            with camera._lock:
                lista, ts = list(camera._cajas), camera._cajas_ts
            if ts is None or time.time() - ts > 1.0:
                continue
            h = f[0].shape[0]
            cajas = [b["xyxy"] for b in lista if b.get("class") == "maceta" and float(b.get("confidence", 0)) >= 0.35]
            nuevo = _servo_bajo.paso(cajas, h, float(state.servo_angle), time.monotonic())
            if nuevo is not None:
                state.servo_angle = clamp_servo(float(nuevo))
                if actuator_proto is not None:
                    actuator_proto.send_actuator_update()
                await state.broadcast({"type": "actuator_setpoint",
                                       "data": {"servo_angle": state.servo_angle, "led": state.led_on}})
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            print(f"[servo_bajo] falló: {type(exc).__name__}: {exc}", flush=True)
            await asyncio.sleep(2.0)


def _macetas_ahora():
    if camera is None:
        return []
    f = camera.ultimo_frame()
    if f is None:
        return []
    with camera._lock:
        lista, ts = list(camera._cajas), camera._cajas_ts
    if ts is None or time.time() - ts > 1.0:
        return []
    h, w = f[0].shape[:2]
    hf = vision_activa_mod.hfov_con_zoom(state.config["odometria_webcam"]["camara_hfov_deg"], getattr(camera, "zoom", 1.0))
    out = []
    for b in lista:
        if b.get("class") != "maceta":
            continue
        yaw, _ = vision_activa_mod.angulos_de_caja(b["xyxy"], w, h, hf, state.config.get("vision_activa", {}).get("camara_yaw_offset_deg", 0.0))
        out.append({"conf": float(b["confidence"]), "yaw_deg": yaw, "dist_m": _dist_caja_m(b["xyxy"], w, h)})
    return out


def _crear_mision_plantas():
    global mision_plantas
    import mision_plantas as mp

    async def guardar_mapa_mision(estado):
        """Al terminar la misión se guarda solo el mapa del SLAM con las plantas encontradas (para mostrarlo y recargarlo después).
        `estado` es una COPIA tomada al terminar: si el operador relanza la misión enseguida (28-oct, tres veces), mision_plantas.plantas ya está vacío cuando esto escribe."""
        nombre = time.strftime("mision_%Y%m%d_%H%M")
        try:
            info = await mapas.guardar(nombre, patrol.waypoints if patrol else [], True)
            (mapas.carpeta / nombre / "plantas.json").write_text(json.dumps(estado, indent=1, ensure_ascii=False))
            await state.broadcast({"type": "toast", "data": {"kind": "success", "title": "Mapa de la misión guardado",
                                                             "message": f"«{info['nombre']}» con {len(estado.get('plantas', []))} planta(s)."}})
        except Exception as exc:
            print(f"[mision_plantas] no se pudo guardar el mapa: {exc}", flush=True)

    def evento(e):
        print(f"[mision_plantas] {e}", flush=True)
        if e.get("tipo") == "mision_fin":
            _schedule(guardar_mapa_mision(json.loads(json.dumps(mision_plantas.estado_dict(), default=str))))
            if gestor_misiones is not None:
                gestor_misiones.al_fin_mision_plantas(e.get("motivo"))
        elif e.get("tipo") == "planta_inspeccionada" and gestor_misiones is not None:
            try:
                gestor_misiones.al_planta(mision_plantas.plantas[-1] if mision_plantas.plantas else e, getattr(vision_activa, "ultimo_resultado", None))
            except Exception as exc:
                print(f"[misiones] no se pudo registrar la planta: {exc}", flush=True)
        _schedule(state.broadcast({"type": "mision_plantas", "data": e}))
        titulos = {"planta_detectada": "Planta detectada", "planta_inspeccionada": "Planta escaneada", "mision_fin": "Misión terminada"}
        if e.get("tipo") in titulos:
            _schedule(state.broadcast({"type": "toast", "data": {"kind": "success", "title": titulos[e["tipo"]],
                                                                  "message": ", ".join(f"{k}: {v}" for k, v in e.items() if k != "tipo")}}))

    mision_plantas = mp.MisionPlantas(explorador=explorador, vision=vision_activa, get_macetas=_macetas_ahora,
                                      get_pose=ros_bridge.pose_en_mapa, led=_poner_led, grabar=_grabar_video,
                                      get_config=lambda: {**state.config.get("mision_plantas", {}), **_mision_plantas_extra}, on_event=evento)
    _crear_gestor_misiones()


_mision_plantas_extra: dict = {}
gestor_misiones = None


def _crear_gestor_misiones():
    """Misiones específicas (misiones.py): explorar y mapear / recoger dataset de plantas / inspección de plagas. Usa el explorador y MisionPlantas."""
    global gestor_misiones

    def aplicar(mp_cfg, vis_cfg):
        _mision_plantas_extra.clear()
        _mision_plantas_extra.update(mp_cfg)
        _mision_vision_override.update({**_ajustes_vision_mision(), **vis_cfg})

    def quitar():
        _mision_plantas_extra.clear()
        _mision_vision_override.clear()

    async def guardar(nombre):
        info = await mapas.guardar(nombre, patrol.waypoints if patrol else [], True)
        await state.broadcast({"type": "toast", "data": {"kind": "success", "title": "Mapa guardado", "message": f"«{info['nombre']}»"}})
        return info

    def evento(e):
        print(f"[misiones] {e}", flush=True)
        _schedule(state.broadcast({"type": "misiones", "data": e}))
        if e.get("tipo") in ("mision_especifica_fin", "inspeccion_planta"):
            _schedule(state.broadcast({"type": "toast", "data": {"kind": "success", "title": "Misión: " + e.get("tipo", ""),
                                                                  "message": ", ".join(f"{k}: {v}" for k, v in e.items() if k != "tipo")}}))

    gestor_misiones = misiones_mod.GestorMisiones(
        carpeta=Path.home() / ".agrobot", mision_plantas=mision_plantas, explorador=explorador, vision=vision_activa,
        aplicar_ajustes=aplicar, quitar_ajustes=quitar, guardar_mapa=guardar, get_pose=ros_bridge.pose_en_mapa, on_event=evento)


async def misiones_handler(request: web.Request):
    """GET /api/misiones — catálogo y estado. POST {"mision": "mapear"|"dataset"|"inspeccion", "accion": "iniciar"|"detener", "opciones": {...}}."""
    if gestor_misiones is None:
        return web.json_response({"error": "misiones no disponibles"}, status=503)
    if request.method == "GET":
        return web.json_response(gestor_misiones.estado_dict())
    try:
        body = await request.json()
    except ValueError:
        body = {}
    if body.get("accion", "iniciar") == "detener":
        gestor_misiones.detener("operador")
        return web.json_response({"ok": True, **gestor_misiones.estado_dict()})
    if _modo_auto_activo() is not None and gestor_misiones.estado not in ("en curso", "terminando"):
        return web.json_response({"ok": False, "msg": f"hay otra autonomía activa ({_modo_auto_activo()}): páralas antes"}, status=409)
    ok, msg = gestor_misiones.iniciar(str(body.get("mision", "")), body.get("opciones") or {})
    return web.json_response({"ok": ok, "msg": msg, **gestor_misiones.estado_dict()}, status=200 if ok else 409)


async def simulador_handler(request: web.Request):
    """GET /api/simulador — estado del simulador visual. POST {"accion": "iniciar"|"detener", "escenario"?}."""
    if request.method == "GET":
        return web.json_response(await asyncio.to_thread(simulador_visual.estado))
    try:
        body = await request.json()
    except ValueError:
        body = {}
    accion = body.get("accion")
    if accion == "iniciar":
        return web.json_response(await asyncio.to_thread(simulador_visual.iniciar, body.get("escenario")))
    if accion == "detener":
        return web.json_response(await asyncio.to_thread(simulador_visual.detener))
    return web.json_response({"error": "accion debe ser iniciar o detener"}, status=400)


async def camara_cajas_handler(request: web.Request):
    """GET /api/camara/cajas — lo que el robot detecta AHORA (clase, confianza, caja y distancia por cámara a la cara delantera)."""
    if camera is None or camera.ultimo_frame() is None:
        return web.json_response({"error": "cámara sin imagen"}, status=503)
    f = camera.ultimo_frame()
    with camera._lock:
        lista, ts = list(camera._cajas), camera._cajas_ts
    h, w = f[0].shape[:2]
    especies = {}
    if request.query.get("especie"):
        import especie_clip
        for k, b in enumerate(lista):
            if b.get("class") == "maceta":
                try:
                    especies[k] = await asyncio.to_thread(especie_clip.clasificar, f[0], b["xyxy"])
                except Exception as exc:
                    especies[k] = {"error": str(exc)}
    return web.json_response({"edad_s": None if ts is None else round(time.time() - ts, 2), "servo": state.servo_angle,
                              "lidar": lidar_reactive.distancias(state.config["slam"]),
                              "cajas": [{"clase": b["class"], "conf": round(float(b["confidence"]), 3), "xyxy": list(b["xyxy"]),
                                         "dist_cara_m": _dist_caja_m(b["xyxy"], w, h), **({"especie": especies[k]} if k in especies else {})}
                                        for k, b in enumerate(lista)]})


async def camara_cruda_handler(request: web.Request):
    """GET /api/camara/cruda.jpg — el último fotograma SIN cajas dibujadas."""
    f = camera.ultimo_frame() if camera is not None else None
    if f is None or cv2 is None:
        return web.json_response({"error": "cámara sin imagen"}, status=503)
    ok, buf = cv2.imencode(".jpg", f[0], [cv2.IMWRITE_JPEG_QUALITY, 92])
    return web.Response(body=buf.tobytes(), content_type="image/jpeg")


def _ajustes_vision_mision() -> dict:
    """Visión activa durante la misión de plantas. Distancia 0,42 m (2026-10-23, el operador veía paradas a ~75 cm con 0,65+0,08; el zoom ya encuadra), antes 0,65 m, 3 retrocesos y 3 giros: lo mejor del barrido en la sala real en L
    (simular_demo.py layout 3, 2026-10-23: 17/18 plantas y 1 roce en 6 simulaciones; con 0,45 m y 2 retrocesos rozaba esquinas y macetas)."""
    return {"clases_objetivo": ["maceta"], "max_objetivos": 1, "conf_min": 0.30,
            "angulos_barrido_deg": [state.servo_angle, 82, 98, state.servo_angle],
            "distancia_inspeccion_m": 0.78, "tol_distancia_m": 0.04, "retroceso_tras_analisis": 3, "giros_tras_analisis": 3}


async def mision_plantas_handler(request: web.Request):
    """GET/POST /api/mision_plantas {"activa": true|false} — la misión de la demo: explorar, mapear y escanear plantas en maceta."""
    if request.method == "GET":
        return web.json_response(mision_plantas.estado_dict() if mision_plantas else {"estado": "no disponible"})
    body = await request.json()
    if body.get("activa"):
        _mision_vision_override.update(_ajustes_vision_mision())
        ok, msg = mision_plantas.iniciar()
        if ok:
            mision_plantas.arrancar_tarea()
        return web.json_response({"ok": ok, "msg": msg, **mision_plantas.estado_dict()}, status=200 if ok else 409)
    mision_plantas.detener("operador")
    _mision_vision_override.clear()
    return web.json_response({"ok": True, **mision_plantas.estado_dict()})


def _crear_vision_activa():
    """Visión activa (vision_activa.py): mueve el servo y pulsa el conductor para centrar, acercarse y analizar un objetivo."""
    global vision_activa

    def cajas():
        if camera is None:
            return None
        f = camera.ultimo_frame()
        if f is None:
            return None
        with camera._lock:
            lista, ts = list(camera._cajas), camera._cajas_ts
        h, w = f[0].shape[:2]
        hf = vision_activa_mod.hfov_con_zoom(state.config["odometria_webcam"]["camara_hfov_deg"], getattr(camera, "zoom", 1.0))
        lista = [{**b, "dist_m": _dist_caja_m(b["xyxy"], w, h)} for b in lista]
        return lista, ts, w, h, hf

    def poner_servo(a):
        state.servo_angle = clamp_servo(float(a))
        if actuator_proto is not None:
            actuator_proto.send_actuator_update()
        _schedule(state.broadcast({"type": "actuator_setpoint", "data": {"servo_angle": state.servo_angle, "led": state.led_on}}))

    def conducir(accion):
        ctrl = state.config["control"]
        esc = state.config["autonomia"]["escala_velocidad"]
        conductor.ordenar(accion, v_max=ctrl["max_v_mps"] * esc, omega_max=ctrl["max_omega_rps"], maniobra=False)

    def capturar(info):
        f = (camera.mejor_nitido() if camera is not None else None)       # el cuadro más nítido de los últimos ~1 s
        if f is None:
            return None
        import cv2
        carpeta = vision_activa_mod.CARPETA / "fotos"
        carpeta.mkdir(parents=True, exist_ok=True)
        ruta = carpeta / f"{time.strftime('%Y%m%d_%H%M%S')}_{int(time.time() * 1000) % 1000:03d}_{info['clase']}_{info.get('vista', 'frente')}.jpg"
        cv2.imwrite(str(ruta), f[0])
        return str(ruta)

    def clasificar(ruta):
        """Segunda opinión sobre la foto: especie con CLIP sobre el recorte de la maceta (¿es una col?) y PLAGASX (enfermedad de col)."""
        out = {}
        img, caja = None, None
        try:
            import especie_clip
            img = cv2.imread(ruta)
            with camera._lock:
                macetas = [b for b in camera._cajas if b.get("class") == "maceta"]
            caja = None
            if macetas and img is not None:                      # la maceta más centrada en la imagen
                w = img.shape[1]
                caja = min(macetas, key=lambda b: abs((b["xyxy"][0] + b["xyxy"][2]) / 2 - w / 2))["xyxy"]
            out["especie"] = especie_clip.clasificar(img, caja)
        except Exception as exc:
            out["especie_error"] = f"{type(exc).__name__}: {exc}"
        mod = _plagasx_clasificador()
        entrada = ruta
        try:
            # Se clasifica el RECORTE de la planta (caja de la maceta +10 %) llevado a 640x480: validado con hojas reales de col a varias
            # distancias (validar_zoom_plagas.py): con zoom 0,8 + recorte la exactitud es la mayor a todas las escalas (30-36 %, sin zoom 1-34 %).
            if img is not None and caja is not None:
                x1, y1, x2, y2 = (float(v) for v in caja)
                mx, my = 0.10 * (x2 - x1), 0.10 * (y2 - y1)
                a0, b0, a1, b1 = int(max(0, x1 - mx)), int(max(0, y1 - my)), int(min(img.shape[1], x2 + mx)), int(min(img.shape[0], y2 + my))
                if a1 - a0 > 40 and b1 - b0 > 40:
                    entrada = cv2.resize(img[b0:b1, a0:a1], (640, 480), interpolation=cv2.INTER_CUBIC)
                    out["recorte"] = True
        except Exception:
            entrada = ruta
        r = mod.predict(entrada, verbose=False, device="cpu")[0]
        p = r.probs.data.cpu().numpy()
        orden = p.argsort()[::-1][:3]
        out.update({"modelo": "plagasx_cls", "top": [{"clase": mod.names[int(i)], "prob": round(float(p[i]), 3)} for i in orden]})
        return out

    def evento(e):
        print(f"[vision_activa] {e}", flush=True)
        _schedule(state.broadcast({"type": "vision_activa", "data": e}))
        if e.get("tipo") in ("inspeccion", "fin", "pausa"):
            txt = {"inspeccion": f"Inspección: {e.get('clase')} ({e.get('conf_media')})",
                   "fin": f"Visión activa terminada: {e.get('motivo')}", "pausa": f"Visión activa en pausa: {e.get('motivo')}"}[e["tipo"]]
            _schedule(state.broadcast({"type": "toast", "data": {"kind": "info", "title": "Visión activa", "message": txt}}))

    vision_activa = vision_activa_mod.VisionActiva(
        get_cajas=cajas, get_servo=lambda: state.servo_angle, set_servo=poner_servo, conducir=conducir,
        ocupado=lambda: conductor.activo, parar=lambda: conductor.parar(),
        distancias=lambda: lidar_reactive.distancias(state.config["slam"]), edad_scan=lidar_reactive.edad_scan_s,
        get_config=lambda: {**state.config.get("vision_activa", {}), **_mision_vision_override}, on_event=evento, capturar=capturar, clasificar=clasificar, girar=lambda g: conductor.girar_grados(g, _gyro_dps),
        get_pose=ros_bridge.pose_en_mapa, enlace_ok=lambda: enlace_c3 is not None and enlace_c3.connected,
        autonomia_activa=_autonomia_sin_mision, reloj=time.time)


async def giro_cerrado_handler(request: web.Request):
    """GET /api/giro_cerrado — último giro de lazo cerrado. POST {"grados": 15, "w": 0.6} — gira esos grados (+ izquierda) midiendo con el
    giroscopio y para al alcanzarlos. Se cancela con la parada total. Rechaza si ya hay un movimiento en curso o no hay C3/IMU."""
    if request.method == "GET":
        return web.json_response({"activo": conductor.activo, "ultimo": conductor.ultimo_giro, "anticipo_s": round(conductor.anticipo_giro_s, 3)})
    try:
        body = await request.json()
        grados = float(body["grados"])
        w = float(body.get("w", 0.6))
    except Exception:
        return web.json_response({"error": "se espera {'grados': número}"}, status=400)
    if not (-180.0 <= grados <= 180.0 and 0.1 <= w <= 1.2):
        return web.json_response({"error": "grados en [-180, 180] y w en [0.1, 1.2]"}, status=400)
    if enlace_c3 is None or not enlace_c3.connected:
        return web.json_response({"error": "C3 sin enlace"}, status=409)
    if _gyro_dps() is None:
        return web.json_response({"error": "IMU sin datos frescos"}, status=409)
    if not conductor.girar_grados(grados, _gyro_dps, w_rps=w):
        return web.json_response({"error": "ya hay un movimiento en curso"}, status=409)
    return web.json_response({"ok": True, "grados": grados})


_maniobra_giro_tarea = None
_maniobra_giro_estado: dict = {"activa": False}


async def _maniobra_giro_bucle(grados: float, tramo_deg: float):
    """Media vuelta / vuelta entera en tramos de lazo cerrado con compensación de la deriva (maniobra_giro.py). Cancelable con la parada total."""
    global _maniobra_giro_estado
    m = ManiobraGiro(grados, tramo_deg=tramo_deg)
    _maniobra_giro_estado = {"activa": True, "pedido_deg": grados, "estado": "arrancando"}
    ctrl = state.config["control"]
    try:
        while True:
            pose = ros_bridge.pose_en_mapa()
            if not pose:
                m._fin("sin pose del SLAM")
                break
            d = _distancias_explorador()
            orden = m.siguiente(pose["theta_deg"], (pose["x"], pose["y"]), d.get("adelante"), d.get("atras"))
            _maniobra_giro_estado = {"activa": True, **m.resumen(), "estado": orden["tipo"]}
            if orden["tipo"] == "fin":
                break
            if orden["tipo"] == "girar":
                ok = conductor.girar_grados(orden["grados"], _gyro_dps)
            else:
                ok = conductor.ordenar(orden["tipo"], v_max=ctrl["max_v_mps"], omega_max=ctrl["max_omega_rps"], maniobra=False)
            if not ok:
                m._fin("movimiento en curso rechazado")
                break
            await asyncio.sleep(0.2)
            while conductor.activo:
                await asyncio.sleep(0.1)
            await asyncio.sleep(0.6)                      # el chasis y el SLAM se asientan antes de medir
    except asyncio.CancelledError:
        m._fin("cancelada")
        raise
    finally:
        if conductor.activo:
            conductor.parar()
        _maniobra_giro_estado = {"activa": False, **m.resumen(), "estado": "fin"}
        print(f"[maniobra_giro] {_maniobra_giro_estado}", flush=True)


async def maniobra_giro_handler(request: web.Request):
    """GET /api/maniobra_giro — estado. POST {"grados": 180|-180|360|-360, "tramo": 30} — gira ese ángulo en el sitio (+ izquierda) en tramos de lazo
    cerrado, corrigiendo la deriva con marcha atrás y parando si no hay espacio o el robot no gira. Rechaza si hay autonomía activa."""
    global _maniobra_giro_tarea
    if request.method == "GET":
        return web.json_response(_maniobra_giro_estado)
    try:
        body = await request.json()
        grados = float(body["grados"])
        tramo = float(body.get("tramo", 30.0))
        ManiobraGiro(grados, tramo_deg=tramo)
    except Exception as exc:
        return web.json_response({"error": f"se espera {{'grados': 180|360|-180|-360, 'tramo': 10..45}}: {exc}"}, status=400)
    if not 10.0 <= tramo <= 45.0:
        return web.json_response({"error": "tramo en [10, 45] grados"}, status=400)
    if _modo_auto_activo() is not None or conductor.activo or (_maniobra_giro_tarea is not None and not _maniobra_giro_tarea.done()):
        return web.json_response({"error": "ya hay un movimiento o una autonomía en curso"}, status=409)
    if enlace_c3 is None or not enlace_c3.connected:
        return web.json_response({"error": "C3 sin enlace"}, status=409)
    if _gyro_dps() is None or not ros_bridge.pose_en_mapa():
        return web.json_response({"error": "IMU o pose del SLAM sin datos frescos"}, status=409)
    _maniobra_giro_tarea = asyncio.create_task(_maniobra_giro_bucle(grados, tramo))
    return web.json_response({"ok": True, "grados": grados, "tramo": tramo})


async def vision_activa_get_handler(request: web.Request):
    """GET /api/vision_activa — estado de la visión activa y su configuración."""
    if vision_activa is None:
        return web.json_response({"error": "no disponible"}, status=503)
    return web.json_response({**vision_activa.estado_dict(), "config": state.config.get("vision_activa", {})})


async def vision_activa_post_handler(request: web.Request):
    """POST /api/vision_activa {"activa": true|false} — la ENCIENDE (mueve el servo y el robot en pulsos cortos) o la para."""
    if vision_activa is None:
        return web.json_response({"error": "no disponible"}, status=503)
    try:
        body = await request.json()
    except Exception:
        body = {}
    if body.get("activa"):
        try:
            await asyncio.to_thread(_plagasx_clasificador)          # se carga ahora, no con el robot ya en pleno análisis
        except Exception as exc:
            print(f"[vision_activa] sin clasificador PLAGASX: {exc}", flush=True)
        vision_activa.aviso = None
        ok, msg = vision_activa.iniciar()
        if not ok:
            return web.json_response({"error": msg}, status=409)
        vision_activa.arrancar_tarea()
    else:
        vision_activa.cancelar("detenida por el operador")
    return web.json_response(vision_activa.estado_dict())


async def paso_get_handler(request: web.Request):
    """GET /api/paso — en qué va el movimiento robótico por pulsos."""
    if paso_robotico is None:
        return web.json_response({"error": "no disponible"}, status=503)
    return web.json_response(paso_robotico.estado)


async def paso_post_handler(request: web.Request):
    """POST /api/paso  {"accion": "avanzar", "cm": 50} | {"accion": "girar",
    "grados": -90} | {"accion": "parar"}; opcional "escala" (0,1-1).

    Movimiento cerrado por sensores: pulso, esperar quieto, medir, decidir
    (ver paso_robotico.py). Responde enseguida; el progreso en GET."""
    if paso_robotico is None:
        return web.json_response({"error": "no disponible"}, status=503)
    try:
        body = await request.json()
    except Exception:
        body = {}
    accion = body.get("accion")
    if accion == "parar":
        paso_robotico.detener()
        return web.json_response({"ok": True, **paso_robotico.estado})
    if accion not in ("avanzar", "girar"):
        return web.json_response({"error": "accion = avanzar | girar | parar"}, status=400)
    if (patrol is not None and patrol.activo) or (explorador is not None and explorador.activo):
        return web.json_response({"error": "hay autonomía en marcha — detenela primero"}, status=409)
    try:
        cantidad = float(body["cm"] if accion == "avanzar" else body["grados"])
    except (KeyError, TypeError, ValueError):
        return web.json_response({"error": "falta cm (avanzar) o grados (girar)"}, status=400)
    if "escala" in body:
        paso_robotico.escala = _unit(body.get("escala"), lo=0.1)
    if not paso_robotico.iniciar(accion, cantidad):
        return web.json_response({"error": "ya hay un movimiento en curso"}, status=409)
    return web.json_response({"ok": True, "accion": accion, "cantidad": cantidad,
                              "escala": paso_robotico.escala})


async def _c3_ready_for_drive():
    """La C3 manda con solo estar enlazada (firmware skid-steer: sin panel web,
    sin mapeo que confirmar). Antes exigía el firmware 1.2 por HTTP."""
    return bool(enlace_c3 is not None and enlace_c3.connected)


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
    if accion and accion != "parar" and vision_activa is not None:
        vision_activa.cancelar("orden manual")             # el operador manda: la visión activa no pelea con él

    if accion == "parar":
        # Parar lo para TODO (misma parada que el botón de emergencia).
        hecho = _parada_total("/api/drive parar", emergencia=False)
        return web.json_response({"ok": True, "accion": "parar", "detenido": hecho})

    if accion not in ACCIONES:
        return web.json_response({"error": f"acción desconocida: {accion!r}"}, status=400)
    # El operador toma el mando: si hay autonomía (patrulla, exploración, misión, Nav2...) se PAUSA y queda registrada
    # la corrección (antes la patrulla devolvía 409 y había que detenerla a mano).
    _tomar_mando("botones")

    # La evasión y el paso robótico se cancelan.
    _detener_evasion()
    if paso_robotico is not None and paso_robotico.estado.get("activo"):
        paso_robotico.detener()
    if enlace_c3 is None or not enlace_c3.connected:
        return web.json_response({"error":"C3 apagada o sin enlace"},status=503)
    try:
        v, _ = demo_command(1,0,float(body.get("escala",.25))*100)
    except (ValueError, TypeError):
        return web.json_response({"error":"potencia invalida"},status=400)
    if not await _c3_ready_for_drive():
        return web.json_response({"error":"C3 sin enlace"},status=409)
    accepted = conductor.ordenar(accion,v_max=v,omega_max=1.5,maniobra=False)
    if accepted:
        _registrar_orden_manual(v if accion == "adelante" else -v if accion == "atras" else 0.0,
                                1.5 if accion == "izquierda" else -1.5 if accion == "derecha" else 0.0, "botones")
    return web.json_response({"ok":accepted,"accion":accion,"duracion_s":.3,
                              "error":None if accepted else "pulso en curso; no se acumula"},
                              status=200 if accepted else 409)



def _tilt_servo_extra() -> float:
    """Inclinación extra de la cámara por el ÁNGULO DEL SERVO, respecto a HOME.

    Corregido el 2026-10-14. Antes esto solo se sumaba con `barrido_tilt`
    activo (flag OFF por defecto), así que con el servo parado fuera de HOME la
    proyección al suelo seguía suponiendo la inclinación de HOME. Con el servo
    a 76° —el ángulo que se usa para mirar el sendero— la cámara mira ~33°
    abajo y la geometría asumía 15,5°: el suelo estimado se va al doble de
    distancia y el sendero aparece corrido. El tilt del servo se conoce
    siempre (es una orden propia), así que se aplica siempre.

    Se prefiere el ángulo CONFIRMADO por el actuador y, si su telemetría está
    vieja, el último ordenado: es la mejor estimación disponible y suponer HOME
    es peor que suponer la orden. El modelo (ganancia 0,8°/°) sigue sin
    verificar físicamente -> calibracion_camara.VERIFICADO.
    """
    st = state.actuator_status or {}
    ang = st.get("servo_angle")
    if ang is None or time.time() - st.get("_received_at", 0) > 1.0:
        ang = state.servo_angle            # último ordenado (ya sin offset)
    else:
        ang -= state.config["calibration"]["servo_offset_deg"]
    return inclinacion_por_servo(ang)


def _geometria_camara() -> dict:
    """La geometría de la cámara, corregida con el tilt del servo y con lo que
    cabecea el chasis.

    El detector de senderos proyecta la imagen al suelo suponiendo una
    inclinación fija de la cámara. En la huerta el robot cabecea en cada surco
    y 2° de error desplazan el suelo estimado 5 cm a un metro: el sendero
    aparece corrido y el robot corrige hacia donde no debe. Medido en el
    simulador, suponer cero da 2,6° de error medio; con el IMU, 0,2°.

    Solo se corrige aquí, para la cámara. La odometría visual sigue con la
    geometría cruda a propósito: alimenta el EKF y no se toca sin medirlo.
    """
    g = state.config["odometria_webcam"]
    extra = _tilt_servo_extra()
    if state.config["autonomia"].get("compensar_cabeceo", True):
        cabeceo = cabeceo_imu.grados if _imu_fresco() else None
        if cabeceo is not None:            # None = sin telemetría (o nodo apagado: dato viejo)
            extra += cabeceo
    if not extra:
        return g
    return {**g, "camara_inclinacion_deg": g["camara_inclinacion_deg"] + extra}


async def sendero_vista_handler(request: web.Request):
    """GET /api/sendero/vista.jpg — lo que ve el detector de hileras ahora:
    cámara con las plantas marcadas + vista cenital con hileras y sendero.
    Para ajustar en campo el rango de verde y la geometría de la cámara."""
    if cv2 is None or camera is None or seguidor_sendero is None:
        return web.json_response({"error": "cámara no disponible"}, status=503)
    dato = camera.ultimo_frame()
    if dato is None:
        return web.json_response({"error": "sin fotograma de la cámara"}, status=503)

    def _render():
        import sendero as sd
        res = seguidor_sendero.vision.detectar(cv2, dato[0], state.config["autonomia"],
                                               _geometria_camara())
        ok, jpg = cv2.imencode(".jpg", sd.dibujar(cv2, dato[0], res),
                               [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpg.tobytes() if ok else None

    datos = await asyncio.to_thread(_render)
    if datos is None:
        return web.json_response({"error": "no se pudo codificar la imagen"}, status=500)
    return web.Response(body=datos, content_type="image/jpeg",
                        headers={"Cache-Control": "no-store"})


def _aplicar_modo(modo: str, perfil: bool) -> dict:
    """Cambia el modo de manejo (pulsos) y, con `perfil`, los umbrales de ese entorno. Lo usan el botón del
    dashboard y el entorno automático. Devuelve las diferencias; puede lanzar PerfilIncoherente."""
    if conductor:
        conductor.modo = modo
    # Se guarda: antes cada reinicio del backend volvía a "exterior" (pulsos de 1 s a toda velocidad)
    # sin avisar (2026-10-02).
    state.config = config_store.update_section("control", {"modo_manejo": modo})
    difs = {}
    if perfil and modo in perfiles_entorno.PERFILES:
        difs = perfiles_entorno.aplicar(config_store, state.config, modo)
        state.config = config_store.load()
    return difs


async def drive_mode_handler(request: web.Request):
    """GET: modo actual. POST {"modo": ..., "perfil": bool}: cambia el modo.

    `modo` cambia la duración de los pulsos (pulse_drive.MODOS). Con
    `"perfil": true` aplica ADEMÁS los umbrales espaciales de ese entorno
    (perfiles_entorno): hasta el 2026-10-14 el modo solo tocaba los pulsos, y
    el robot seguía exigiendo 1 m libre por delante para moverse —el perfil de
    huerta— lo que en una sala con macetas lo deja clavado.
    """
    if request.method == "GET":
        actual = conductor.modo if conductor else "exterior"
        return web.json_response({
            "modo": actual, "modos": list(MODOS.keys()),
            "perfiles": list(perfiles_entorno.PERFILES.keys()),
            # Qué cambiaría al aplicar cada perfil, sin aplicarlo.
            "perfil_pendiente": {
                n: {k: {"antes": a, "despues": d}
                    for k, (a, d) in perfiles_entorno.diferencias(state.config, n).items()}
                for n in perfiles_entorno.PERFILES}})
    body = await request.json()
    modo = body.get("modo", "exterior")
    if modo not in MODOS:
        return web.json_response({"error": f"modo desconocido: {modo!r}"}, status=400)
    try:
        difs = _aplicar_modo(modo, bool(body.get("perfil")))
    except perfiles_entorno.PerfilIncoherente as exc:
        # No se aplica a medias: el margen de parada y la velocidad van juntos o no van.
        return web.json_response({"error": f"perfil incoherente: {exc}"}, status=400)
    if entorno is not None:
        entorno.fijar_operador(modo)      # lo que fija el operador a mano manda y queda como ETIQUETA

    await state.broadcast({"type": "drive_mode", "data": {"modo": modo,
                                                          "perfil": difs}})
    detalle = (f"Cambiado a modo {modo}" if not difs else
               f"Modo {modo} + perfil de entorno ({len(difs)} umbrales)")
    await state.broadcast({"type": "toast", "data": {
        "kind": "success", "title": "Modo de manejo", "message": detalle,
    }})
    return web.json_response({"ok": True, "modo": modo, "perfil": difs})


def _foco(encendido: bool):
    """Enciende o apaga el foco de escaneo."""
    state.led_on = encendido


async def _barrido_camara():
    """Barrido de la cámara HECHO AQUÍ, moviendo el servo paso a paso.

    Antes se levantaba el flag FLAG_SCAN y el barrido lo hacía el firmware,
    con duración y recorrido fijos (10 s, 25→110°). Haciéndolo por software
    se ajustan desde el dashboard sin reprogramar el ESP32, y se puede ir
    más lento para que las fotos salgan nítidas.

    El robot se deja parado durante el barrido: un servo barriendo mientras
    el robot avanza da fotos movidas."""
    cfg = state.config["autonomia"]
    dur = max(0.5, float(cfg["escaneo_duracion_s"]))
    a0, a1 = int(cfg["escaneo_angulo_min"]), int(cfg["escaneo_angulo_max"])
    paso_s = 0.05                      # 20 pasos por segundo
    n = max(1, int(dur / paso_s))
    volver = state.servo_angle
    foco_previo = state.led_on
    try:
        if actuator_proto is not None:
            actuator_proto.send_command(0.0, 0.0, 0)
        if cfg["escaneo_enciende_foco"]:
            _foco(True)
        for i in range(n + 1):
            state.servo_angle = clamp_servo(a0 + (a1 - a0) * i / n)
            if actuator_proto is not None:
                actuator_proto.send_actuator_update()
            await asyncio.sleep(paso_s)
    finally:
        # Vuelve a donde estaba el slider: si no, la odometría visual se
        # queda pausada para siempre por "cámara girada".
        state.servo_angle = volver
        state.led_on = foco_previo          # el foco vuelve como estaba
        if actuator_proto is not None:
            actuator_proto.send_actuator_update()
        await state.broadcast({"type": "actuator_setpoint", "data": {
            "servo_angle": state.servo_angle, "led": state.led_on}})


aprendizaje = None
politica = None


def _percepcion_robot() -> dict:
    """Lo que percibe el robot, con los MISMOS nombres que en el simulador.

    De esto dependen las maniobras aprendidas y la política entrenada: si
    aquí se llamara distinto, lo aprendido en simulación no valdría."""
    d = lidar_reactive.distancias(state.config["slam"])
    v = seguidor_sendero.vision if seguidor_sendero is not None else None
    pose = current_pose()
    cm_centro, cm_ancho = costmap.hueco_libre()
    return {
        "lidar_adelante_m": d.get("adelante"), "lidar_atras_m": d.get("atras"),
        "lidar_izquierda_m": d.get("izquierda"), "lidar_derecha_m": d.get("derecha"),
        "costmap_hueco_m": cm_centro, "costmap_ancho_m": cm_ancho,
        "sendero_lateral_m": getattr(v, "lateral_m", None),
        "sendero_rumbo_deg": getattr(v, "angulo_deg", None),
        "sendero_ancho_m": getattr(v, "ancho_m", None),
        "sendero_confianza": getattr(v, "confianza", 0.0),
        "v_actual": round(float(pose.get("vx", 0.0)), 3),
        "omega_actual": round(float(pose.get("vyaw", 0.0)), 3),
    }


def _sensores_guardian() -> dict:
    """_percepcion_robot() + IMU (giro Z en rad/s, balanceo y cabeceo en °) para que el guardián correlacione distancia, IMU y cámara."""
    s = _percepcion_robot()
    env = state.env_telemetry or {}
    ra = env.get("_received_at")
    if ra is not None and time.time() - ra < 1.0:
        if env.get("gyroZ") is not None:
            s["imu_gz"] = math.radians(float(env["gyroZ"]))
        if all(env.get(k) is not None for k in ("accelX", "accelY", "accelZ")):
            inc = imu_nivel.inclinacion_deg((env["accelX"], env["accelY"], env["accelZ"]))
            if inc:
                s["imu_roll_deg"], s["imu_pitch_deg"] = inc
    return s


def _conducir_pulso(accion: str, maniobra: bool = True, esc_dur: float = 1.0):
    """Un pulso del conductor manual, con el acelerador de la config.

    `maniobra` la decide quien llama (el explorador): con una pared cerca la
    maniobra de tierra blanda haría girar al robot rozándola."""
    ctrl = state.config["control"]
    escala = state.config["autonomia"]["escala_velocidad"]
    v_max = ctrl["max_v_mps"] * escala
    # Freno proactivo: si la cámara ve un montículo bajo delante (lo que el
    # LIDAR alto no ve), baja a la velocidad segura de la envolvente de vuelco
    # ANTES de llegar. Solo con el barrido activo y para ir hacia delante.
    if _barrido_activo() and accion == "adelante":
        hueco = suelo_libre.hueco_delante(state.config["slam"]["robot_ancho_cm"] / 100.0)
        # Altura estimada por paralaje; si aún no hay estimación, tope_velocidad
        # usa el peor caso conservador.
        altura = estimador_altura.altura()
        v_max = min(v_max, freno_proactivo.tope_velocidad(hueco, v_max, altura_m=altura))
    # Giro de lazo cerrado (2026-10-27): un pulso de giro a tiempo fijo no gira (29 % de los pulsos <2°; el 27-oct 18 pulsos seguidos no movieron el rumbo).
    # Con autonomia.giro_lazo_cerrado el pulso de giro del explorador es «gira mx_grados_pulso» medido con el giroscopio, subiendo la fuerza si el pulso sale muerto.
    # Sin IMU fresco cae al pulso de siempre.
    if accion in ("izquierda", "derecha") and state.config["autonomia"].get("giro_lazo_cerrado", False) and _gyro_dps() is not None:
        grados = float(state.config["autonomia"].get("mx_grados_pulso", 6.0))
        if conductor.girar_grados(grados if accion == "izquierda" else -grados, _gyro_dps, w_rps=0.6, t_max_s=2.0):
            return
    conductor.ordenar(accion, v_max=v_max,
                      omega_max=ctrl["max_omega_rps"] * escala, maniobra=maniobra, esc_dur=esc_dur)


def _evento(tipo: str, data: dict):
    print(f"[{tipo}] {data}", flush=True)
    _lanzar(state.broadcast({"type": f"{tipo}_event", "data": data}), tipo)


def _patrol_event(data: dict):
    # Al log ADEMÁS de al dashboard: sin esto no había forma de saber qué hizo
    # la patrulla si el navegador no estaba abierto (pasó el 2026-09-17).
    print(f"[patrulla] {data}", flush=True)
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
            # Import local, como en el cargador del dashboard: faltaba y cada
            # arranque autónomo fallaba con NameError sin cargar el modelo.
            from ultralytics import YOLO
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


async def exploracion_start_handler(request: web.Request):
    """POST /api/exploracion/start — autonomía reactiva: recorre y mapea."""
    if explorador is None:
        return web.json_response({"error": "explorador no inicializado"}, status=503)
    if patrol is not None and patrol.activo:
        return web.json_response(
            {"ok": False, "error": "detené la patrulla antes de explorar"}, status=409)
    await _cargar_modelo_si_falta(request.app)
    if camera is not None:
        camera.set_zoom(1.0)        # explorando se necesita el campo completo
    ok, msg = explorador.iniciar()
    state.mission_active = ok
    await state.broadcast({"type": "mission_state", "data": {"active": ok}})
    await state.broadcast({"type": "toast", "data": {
        "kind": "success" if ok else "warning", "title": "Exploración", "message": msg}})
    return web.json_response({"ok": ok, "message": msg}, status=200 if ok else 409)


async def exploracion_stop_handler(request: web.Request):
    if explorador is not None:
        explorador.detener()
    state.mission_active = False
    actuator_proto.emergency_stop()
    await state.broadcast({"type": "mission_state", "data": {"active": False}})
    return web.json_response({"ok": True})


async def patrol_auto_handler(request: web.Request):
    """POST /api/patrol/auto {"activa": true} — graba un punto cada 1,5 m
    mientras el operador conduce, para no tener que ir pulsando Grabar."""
    if patrol is None:
        return web.json_response({"error": "patrulla no inicializada"}, status=503)
    body = await request.json()
    ok, msg = patrol.grabacion_auto(bool(body.get("activa", False)))
    if not ok:
        return web.json_response({"ok": False, "error": msg}, status=409)
    await state.broadcast({"type": "toast", "data": {
        "kind": "success", "title": "Grabación de recorrido", "message": msg}})
    return web.json_response({"ok": True, "activa": patrol.auto_activa, "message": msg})


async def patrol_clear_handler(request: web.Request):
    if patrol is None:
        return web.json_response({"error": "patrulla no inicializada"}, status=503)
    if patrol.activo:
        return web.json_response(
            {"error": "no se borra el recorrido con la patrulla corriendo"}, status=409)
    patrol.borrar_waypoints()
    return web.json_response({"ok": True})


# ----- Saludo (botón del dashboard) -----
saludo_tarea: "asyncio.Task | None" = None


async def _saludo_girar(accion: str):
    """Un giro como el de las flechas del dashboard (pulso), esperando a que acabe."""
    escala = state.config["autonomia"]["escala_velocidad"]
    v, _ = demo_command(1, 0, float(escala) * 100)
    conductor.ordenar(accion, v_max=v, omega_max=1.5, maniobra=False)
    t0 = time.monotonic()
    while conductor.activo and time.monotonic() - t0 < 5.0:
        await asyncio.sleep(0.05)
    await asyncio.sleep(0.4)                  # que se asiente antes del siguiente movimiento


async def _saludo_servo(angulo: int):
    state.servo_angle = clamp_servo(float(angulo))
    actuator_proto.send_actuator_update()
    await state.broadcast({"type": "actuator_setpoint", "data": {
        "servo_angle": state.servo_angle, "led": state.led_on}})


async def _saludo_foco(encendido: bool):
    state.led_on = bool(encendido)
    actuator_proto.send_actuator_update()
    await state.broadcast({"type": "actuator_setpoint", "data": {
        "servo_angle": state.servo_angle, "led": state.led_on}})


async def _saludar():
    from saludo import Saludo
    await state.broadcast({"type": "toast", "data": {"kind": "info", "title": "👋 Saludo", "message": "Saludando…"}})
    try:
        await Saludo(_saludo_girar, _saludo_servo, _saludo_foco).ejecutar()
        await state.broadcast({"type": "toast", "data": {"kind": "success", "title": "👋 Saludo", "message": "Listo."}})
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        print(f"[saludo] falló: {type(exc).__name__}: {exc}", flush=True)
        await state.broadcast({"type": "toast", "data": {"kind": "error", "title": "Saludo", "message": "No se pudo completar."}})


async def saludo_handler(request: web.Request):
    """POST /api/saludo — izquierda, derecha, mira arriba, foco x3 y barrido de cámara."""
    global saludo_tarea
    if enlace_c3 is None or not enlace_c3.connected:
        return web.json_response({"error": "C3 apagada o sin enlace"}, status=503)
    if saludo_tarea is not None and not saludo_tarea.done():
        return web.json_response({"error": "ya está saludando"}, status=409)
    if patrol is not None and patrol.activo:
        return web.json_response({"error": "la patrulla autónoma está en marcha: detenla primero"}, status=409)
    if conductor is not None and conductor.activo:
        return web.json_response({"error": "hay un movimiento en curso"}, status=409)
    saludo_tarea = asyncio.create_task(_saludar())
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
_video_task: "asyncio.Task | None" = None
_video_file: str | None = None
VIDEO_DIR = _AQUI / "videos"
VIDEO_DIR.mkdir(exist_ok=True)
VIDEO_FPS = 15          # metadato de reproducción; aproxima el ritmo del stream


async def _bucle_grabar_video(filename: str):
    """Graba a disco los MISMOS frames que ve el stream (con overlay YOLO).

    El handler anterior creaba el VideoWriter pero NADIE le pasaba frames: el
    .mp4 salía vacío. Acá nos suscribimos a la cámara (igual que el MJPEG),
    decodificamos cada JPEG a BGR y lo escribimos. El writer se crea con el
    TAMAÑO REAL del primer frame (antes se fijaba 1920x1080 a ciegas y, si no
    coincidía, write() fallaba en silencio)."""
    global video_recording, _video_file
    import numpy as np          # np no está en el scope global de este módulo
    if camera is None or cv2 is None:
        video_recording = False
        return
    cola = camera.subscribe()
    writer = None
    n = 0
    try:
        while video_recording:
            try:
                jpeg = await asyncio.wait_for(cola.get(), timeout=2.0)
            except asyncio.TimeoutError:
                continue
            frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            if writer is None:
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(
                    str(VIDEO_DIR / filename),
                    cv2.VideoWriter_fourcc(*"mp4v"), VIDEO_FPS, (w, h))
                if not writer.isOpened():
                    print(f"[VIDEO] no se pudo abrir el writer para {filename}", flush=True)
                    video_recording = False
                    break
            writer.write(frame)
            n += 1
    except asyncio.CancelledError:
        pass
    finally:
        camera.unsubscribe(cola)
        if writer is not None:
            writer.release()
        print(f"[VIDEO] grabación cerrada: {filename} ({n} frames)", flush=True)


async def video_record_handler(request: web.Request):
    """Inicia o detiene la grabación de video desde la cámara.
    POST {"action": "start"|"stop"|"status"}."""
    global video_recording, _video_task, _video_file

    body = await request.json()
    action = body.get("action", "start")

    if action == "status":
        return web.json_response({"grabando": video_recording, "file": _video_file})

    if action == "start":
        if video_recording:
            return web.json_response({"error": "ya hay una grabación en curso",
                                      "file": _video_file}, status=400)
        if camera is None:
            return web.json_response({"error": "cámara no disponible"}, status=503)
        _video_file = f"agrobot_video_{int(time.time())}.mp4"
        video_recording = True
        _video_task = asyncio.create_task(_bucle_grabar_video(_video_file))
        print(f"[VIDEO] iniciando grabación: {_video_file}", flush=True)
        return web.json_response({"ok": True, "file": _video_file})

    if action == "stop":
        if not video_recording:
            return web.json_response({"error": "no hay grabación en curso"}, status=400)
        video_recording = False
        if _video_task is not None:
            try:
                await asyncio.wait_for(_video_task, timeout=3.0)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                _video_task.cancel()
            _video_task = None
        archivo = _video_file
        _video_file = None
        print(f"[VIDEO] grabación detenida: {archivo}", flush=True)
        return web.json_response({"ok": True, "file": archivo})

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


_ultima_plaga: dict = {}      # clase -> (instante monotónico, x, y, repeticiones suprimidas)


def _plaga_repetida(clase: str) -> int | None:
    """None = aviso nuevo. Si la MISMA clase se avisó hace < `detection.plaga_dedup_s` (30 s) y el robot casi no se movió (< 0,5 m), es la misma
    planta vista otra vez: devuelve cuántas llevan suprimidas. El 2026-10-26 salieron ~250 avisos en 37 min, uno cada 3 s con el robot parado."""
    ventana = float(state.config.get("detection", {}).get("plaga_dedup_s", 30.0))
    if ventana <= 0:
        return None
    ahora = time.monotonic()
    pose = current_pose() or {}
    x, y = float(pose.get("x", 0.0)), float(pose.get("y", 0.0))
    prev = _ultima_plaga.get(clase)
    if prev and ahora - prev[0] < ventana and math.hypot(x - prev[1], y - prev[2]) < 0.5:
        _ultima_plaga[clase] = (prev[0], prev[1], prev[2], prev[3] + 1)
        return prev[3] + 1
    _ultima_plaga[clase] = (ahora, x, y, 0)
    return None


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
        patrol.solicitar_escaneo_por_deteccion(
            state.config["detection"].get("escanear_al_detectar", "siempre"))

    # Una planta no es una plaga: dispara el escaneo (arriba) pero ni toast, ni captura, ni alerta.
    if detection.get("class") in state.config["detection"].get("clases_planta", []):
        return

    if frame is not None and _plaga_repetida(detection["class"]) is not None:
        return                                  # la misma plaga en el mismo sitio: ya se avisó (y se guardó la captura)
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


async def models_presets_handler(request: web.Request):
    import modelos_preset
    return web.json_response(modelos_preset.listar(
        MODELS_DIR, state.config["ai"]["active_model"]))


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
PARAM_SECTIONS = ("slam", "detection", "safety", "control", "odometria_webcam", "autonomia", "vision_activa")

# Topes que un cliente NO puede sobrepasar por más que mande cualquier cosa
# al POST. La sección `control` multiplica directamente lo que sale hacia
# los motores, así que un valor absurdo (o un texto) no puede quedar
# guardado: se guardaría en disco y seguiría vigente en el próximo arranque.
CONTROL_LIMITS = {
    "max_v_mps": (0.05, 1.5),
    "max_omega_rps": (0.1, 4.0),
    "suavidad_duty": (0.15, 1.0),        # los pulsos de la exploración se trocean con este ciclo (ver Conductor._tramo)
    "suavidad_periodo_s": (0.1, 1.0),
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
    cfg_slam = state.config["slam"]
    costmap.update(points, cfg_slam["robot_largo_cm"], cfg_slam["robot_ancho_cm"])
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
# ===== COSTMAP LOCAL =====
# =============================================

async def costmap_status_handler(request: web.Request):
    """GET /api/costmap — resumen del costmap local."""
    res = costmap.resumen()
    centro, ancho = costmap.hueco_libre()
    res["hueco_centro_cm"] = None if centro is None else round(centro * 100)
    res["hueco_ancho_cm"] = None if ancho is None else round(ancho * 100)
    return web.json_response(res)


async def costmap_grid_handler(request: web.Request):
    """GET /api/costmap/grid — rejilla como lista de listas (para debug/visor)."""
    grid = costmap.as_occupancy_grid()
    return web.json_response({"grid": grid.tolist(),
                              "cells": costmap.cells, "res_m": costmap.res})


async def costmap_reset_handler(request: web.Request):
    """POST /api/costmap/reset"""
    import numpy as np
    costmap.grid = np.full((costmap.cells, costmap.cells), 0.5, dtype=np.float32)
    costmap._scan_count = 0
    return web.json_response({"ok": True})


async def local_map_save_handler(request: web.Request):
    """POST /api/costmap/save — guarda el costmap local con nombre."""
    body = await request.json()
    name = body.get("name", "costmap_auto")
    pose = current_pose()
    path = map_manager.save(name, costmap, pose)
    return web.json_response({"ok": True, "path": str(path)})


async def local_map_load_handler(request: web.Request):
    """POST /api/costmap/load — restaura un costmap guardado."""
    body = await request.json()
    name = body.get("name", "costmap_auto")
    meta = map_manager.load(name, costmap)
    if meta is None:
        return web.json_response({"error": f"mapa '{name}' no existe"}, status=404)
    return web.json_response({"ok": True, "meta": meta})


async def local_map_list_handler(request: web.Request):
    """GET /api/costmap/maps — lista de mapas locales guardados."""
    return web.json_response(map_manager.list_maps())


async def local_map_delete_handler(request: web.Request):
    """POST /api/costmap/delete — borra un mapa guardado."""
    body = await request.json()
    name = body.get("name", "")
    ok = map_manager.delete(name)
    if not ok:
        return web.json_response({"error": f"mapa '{name}' no existe"}, status=404)
    return web.json_response({"ok": True})


# =============================================
# ===== SALUD DE TÓPICOS =====
# =============================================
# Varianza de la velocidad de avance del EKF por encima de la cual ya no le
# llega ninguna medida (rf2o ni /zupt): crece ~0,1 (m/s)² por segundo.
EKF_VARIANZA_DISPARADA = 1.0
# La Intel 8265 se protege sola hacia 100 °C (apaga la radio: sin AP, sin
# dashboard y sin C3 por WiFi). Aviso con margen.
TEMP_RADIO_AVISO_C = 75.0


def _salud_completa() -> list[dict]:
    """Latidos de cada fuente + el MOTIVO cuando algo falla. El dashboard, el
    WebSocket y /api/health usan esta misma lista."""
    cfg = state.config["safety"]
    filas = health.snapshot(cfg["heartbeat_stale_warn_s"], cfg["heartbeat_stale_err_s"])
    for f in filas:
        f["detail"] = ""
        try:
            if f["key"] == "lidar":
                puerto = _puerto_lidar()
                f["detail"] = ("USB (CP2102N)" if puerto != LIDAR_UART
                               else "sin adaptador USB: ¿cable suelto? (probando la UART)")
            elif f["key"] == "esp32c3" and enlace_c3 is not None:
                r = enlace_c3.resumen()
                wifi = r["wifi"] or {}
                partes = ["BLE ✓" if r["ble"] else (
                    f"BLE buscando hace {r['buscando_s'] // 60} min" if r["buscando_s"] is not None
                    else "BLE ✗")]
                partes.append(f"WiFi ✓ {wifi['ip']}" if wifi.get("conectado") else "WiFi ✗")
                partes.append(f"{r['reconexiones']} conexiones BLE")
                if r["reinicios_c3"]:
                    partes.append(f"{r['reinicios_c3']} reinicios")
                f["detail"] = " · ".join(partes)
            elif f["key"] == "lora_v3":
                r = enlace_lora.resumen()
                if r["perdida_pct"] is not None:
                    f["detail"] = f"UDP · pérdida {r['perdida_pct']}% · {r['reinicios']} reinicios"
            elif f["key"] == "slam" and ros_last_map:
                ext = ros_last_map.get("extension_m")
                if ext is not None:
                    f["detail"] = f"mapa de {ext:.0f} m"
                if ros_last_map.get("corrupto"):
                    f["status"] = "err"
                    f["detail"] = (f"mapa corrupto ({ext / 1000:.1f} km): la pose saltó. "
                                   "Reinicia el mapa")
        except Exception as exc:
            f["detail"] = f"({type(exc).__name__})"
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
    # AP JETSON_AP en la antena USB: lo vigila agrobot-ap-watchdog (servicio aparte) y deja su estado en /run.
    try:
        ap = json.loads(Path("/run/agrobot_ap_status.json").read_text())
        edad = time.time() - float(ap["t"])
        if edad > 40:
            st, det = "err", f"el vigilante del AP no responde desde hace {edad:.0f} s (¿servicio agrobot-ap-watchdog?)"
        elif not ap["ok"]:
            st, det = "err", f"AP caído: {ap['detalle']} (se está recreando)"
        elif not ap.get("en_usb") and ap.get("usb_presente"):
            st, det = "warn", "el AP no está en la antena USB"
        else:
            st = "ok" if ap.get("modo") == "usb" else "warn"
            det = f"{'antena USB' if ap.get('modo') == 'usb' else 'RESPALDO radio interna'} · {ap.get('estaciones', 0)} estaciones · {ap.get('recreaciones', 0)} recreaciones"
        filas.append({"key": "ap", "label": "AP JETSON_AP (antena)", "icon": "📶", "status": st, "age_s": None, "detail": det})
    except (OSError, ValueError, KeyError):
        filas.append({"key": "ap", "label": "AP JETSON_AP (antena)", "icon": "📶", "status": "warn", "age_s": None,
                      "detail": "sin estado del vigilante (servicio agrobot-ap-watchdog parado)"})
    t_radio = _temperatura_zona("iwlwifi_1")
    if t_radio is not None:
        filas.append({
            "key": "radio",
            "label": "Tarjeta WiFi/Bluetooth",
            "icon": "🌡️",
            "status": "ok" if t_radio < TEMP_RADIO_AVISO_C else ("warn" if t_radio < 90 else "err"),
            "age_s": None,
            "detail": f"{t_radio:.0f} °C" + ("" if t_radio < TEMP_RADIO_AVISO_C
                                            else " — caliente: ventilación o menos carga de radio"),
        })
    if ros_fused_pose is not None:
        var = ros_fused_pose.get("var_vx") or 0.0
        disparada = var > EKF_VARIANZA_DISPARADA
        filas.append({
            "key": "ekf",
            "label": "Odometría fusionada (EKF)",
            "icon": "🧮",
            "status": "err" if disparada else "ok",
            "age_s": None,
            "detail": ("sin medida de velocidad: extrapola (¿LIDAR y ZUPT caídos?)" if disparada
                       else f"x={ros_fused_pose['x']:.1f} y={ros_fused_pose['y']:.1f} m · "
                            f"sesgo del giro {sesgo_giro.sesgo['gyroZ']:+.2f} °/s"
                            + ("" if sesgo_giro.listo else " (calibrando)")),
        })
    return filas


async def health_handler(request: web.Request):
    return web.json_response(_salud_completa())


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
        await asyncio.to_thread(
            subprocess.run,
            ["ros2", "node", "kill", "/nav2_bringup/lifecycle_manager_navigation"],
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
        # En un hilo: bloquear el bucle de eventos 15 s tira el WebSocket
        # (latido de 10 s) y corta el vídeo. Era una de las causas de "se
        # pierde el enlace Jetson".
        result = await asyncio.to_thread(
            subprocess.run,
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

    mapa_scan.reiniciar()
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


async def mapas_digitalizar_handler(request: web.Request):
    """POST /api/mapas/digitalizar {"nombre": "invernadero"} -> plano vectorial (SVG, DXF, JSON y PNG) en maps/<nombre>/digital/.
    Hace falta el mapa.pgm/mapa.yaml de ese mapa (los guarda «Guardar mapa»)."""
    body = await request.json()
    try:
        nombre = validar_nombre_mapa(body.get("nombre", ""))
    except ErrorMapa as exc:
        return web.json_response({"ok": False, "error": str(exc)}, status=400)
    yaml_path = mapas.carpeta / nombre / "mapa.yaml"
    if not yaml_path.exists():
        return web.json_response({"ok": False, "error": f"«{nombre}» no tiene mapa.yaml/mapa.pgm"}, status=404)
    import digitalizar_mapa
    loop = asyncio.get_running_loop()
    try:
        res = await loop.run_in_executor(None, digitalizar_mapa.digitalizar, yaml_path, mapas.carpeta / nombre / "digital")
    except Exception as exc:  # noqa: BLE001  (cualquier fallo de lectura/OpenCV se le muestra al operador)
        return web.json_response({"ok": False, "error": f"no se pudo digitalizar: {exc}"}, status=500)
    base = f"/api/mapas/digital/{nombre}/"
    return web.json_response({"ok": True, **{k: v for k, v in res.items() if k not in ("muros", "esquinas")},
                              "archivos": {f: base + f for f in ("plano.svg", "plano.dxf", "plano.json", "plano.png")}})


async def mapas_digital_archivo_handler(request: web.Request):
    """GET /api/mapas/digital/<nombre>/<archivo>: descarga lo generado por /api/mapas/digitalizar."""
    try:
        nombre = validar_nombre_mapa(request.match_info["nombre"])
    except ErrorMapa as exc:
        return web.json_response({"ok": False, "error": str(exc)}, status=400)
    archivo = request.match_info["archivo"]
    if archivo not in ("plano.svg", "plano.dxf", "plano.json", "plano.png", "limpio.png"):
        return web.json_response({"ok": False, "error": "archivo desconocido"}, status=404)
    ruta = mapas.carpeta / nombre / "digital" / archivo
    if not ruta.exists():
        return web.json_response({"ok": False, "error": "primero hay que digitalizar el mapa"}, status=404)
    return web.FileResponse(ruta)


async def simulador_subir_datos_handler(request: web.Request):
    """POST /api/simulador/subir_datos {"nombre"?: "...", "mapa"?: "<mapa ya guardado>"}

    Sube lo que el robot recolectó (mapa del SLAM, recorrido, plantas escaneadas, punto de inicio) al simulador: guarda un paquete en
    ~/.agrobot/datos_simulador/<nombre>/ y escribe el mundo `real_<nombre>` en simulador_pasivo/mundos/, que el editor de terrenos del simulador
    abre y deja corregir. Si el SLAM no está corriendo, usa `mapa` o el último mapa guardado."""
    try:
        body = await request.json()
    except ValueError:
        body = {}
    try:
        nombre = validar_nombre_mapa(body.get("nombre") or time.strftime("salida_%Y%m%d_%H%M"))
    except ErrorMapa as exc:
        return web.json_response({"ok": False, "error": str(exc)}, status=400)
    waypoints = list(patrol.waypoints) if patrol else []
    plantas = [dict(p) for p in (mision_plantas.plantas if mision_plantas is not None else [])]
    aviso = None
    carpeta = None
    try:
        await mapas.guardar(nombre, waypoints, True)             # congela el mapa actual del SLAM con su recorrido
        carpeta = mapas.carpeta / nombre
        (carpeta / "plantas.json").write_text(json.dumps(mision_plantas.estado_dict() if mision_plantas is not None else {"plantas": plantas}, indent=1, ensure_ascii=False))
    except Exception as exc:  # noqa: BLE001  (SLAM caído u otro fallo: se usa un mapa que ya esté guardado)
        previo = body.get("mapa") or (mapas.listar()[0]["nombre"] if mapas.listar() else None)
        if previo is None or not (mapas.carpeta / validar_nombre_mapa(previo) / "mapa.yaml").exists():
            return web.json_response({"ok": False, "error": f"no hay mapa para subir ({exc}). Guarda un mapa primero."}, status=409)
        carpeta = mapas.carpeta / validar_nombre_mapa(previo)
        aviso = f"el SLAM no respondió ({exc}); se usó el mapa guardado «{previo}»"
        if not waypoints:
            try:
                waypoints = json.loads((carpeta / "waypoints.json").read_text())
            except (OSError, ValueError):
                waypoints = []
        if not plantas:
            try:
                plantas = json.loads((carpeta / "plantas.json").read_text()).get("plantas", [])
            except (OSError, ValueError):
                plantas = []
    import subir_datos_simulador
    mundos_dir = _AQUI.parent.parent / "simulador_pasivo" / "mundos"
    try:
        res = await asyncio.to_thread(subir_datos_simulador.exportar, nombre, carpeta, plantas, waypoints, mundos_dir)
    except Exception as exc:  # noqa: BLE001
        return web.json_response({"ok": False, "error": f"no se pudo exportar al simulador: {exc}"}, status=500)
    await state.broadcast({"type": "toast", "data": {"kind": "success", "title": "Datos subidos al simulador",
                                                     "message": f"Mundo «{res['mundo']}»: {res['muros']} muros, {res['plantas']} planta(s). Ábrelo en el editor de terrenos del simulador."}})
    return web.json_response({"ok": True, "aviso": aviso, **res})


async def exploracion_regresar_handler(request: web.Request):
    """POST /api/exploracion/regresar — E11: el robot da la vuelta y vuelve al punto de salida por una ruta A* sobre el mapa que acumuló."""
    if explorador is None or not hasattr(explorador, "iniciar_regreso"):
        return web.json_response({"ok": False, "error": "el explorador no soporta regreso (¿FSM activa?)"}, status=409)
    ok, msg = explorador.iniciar_regreso()
    return web.json_response({"ok": ok, "mensaje": msg, "e11": explorador.snapshot().get("e11")}, status=200 if ok else 409)


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
    if camera is not None:
        camera.set_zoom(1.0)        # misión autónoma = campo completo

    # Un pulso manual a medio ejecutar pelearía con Nav2 por el actuador.
    if conductor is not None and conductor.activo:
        conductor.parar()
    if explorador is not None and explorador.activo:
        explorador.detener()

    ok, msg = (False, "patrulla no inicializada")
    if mision_plantas is not None and state.config["autonomia"].get("auto_mision_plantas", True):
        # Demo (2026-10-23): AUTO = misión de plantas. Antes AUTO exploraba y, al segundo intento, «patrullaba» un único punto.
        _mision_vision_override.update(_ajustes_vision_mision())
        ok, msg = mision_plantas.iniciar()
        if ok:
            mision_plantas.arrancar_tarea()
            msg = "misión de plantas: explora, mapea y escanea las macetas"
    elif patrol is not None:
        ok, msg = patrol.iniciar()
        # Sin recorrido grabado no hay nada que patrullar: en vez de quedarse parado, el robot
        # explora solo (esquiva y mapea) y deja el recorrido grabado para la siguiente patrulla.
        if not ok and "waypoints" in msg and explorador is not None:
            ok, msg = explorador.iniciar()
            if ok:
                msg = "sin recorrido grabado: explora, esquiva y mapea (deja el recorrido para patrullar)"

    state.mission_active = ok
    await state.broadcast({"type": "mission_state", "data": {"active": ok}})
    await state.broadcast({"type": "toast", "data": {
        "kind": "success" if ok else "warning",
        "title": "Modo autónomo" if ok else "No se pudo arrancar",
        "message": msg,
    }})
    print(f"[AUTO] {msg}")
    return web.json_response({"active": ok, "message": msg}, status=200 if ok else 409)


async def camara_zoom_handler(request: web.Request):
    """GET/POST /api/camara/zoom {"zoom": 1..4} — zoom de la cámara para inspeccionar plantas/plagas.
    Con zoom > 1 se captura a 1280x720 y se recorta el centro (detalle real). Solo con el robot quieto:
    con exploración/patrulla en marcha debe ser 1x (SLAM y sendero esperan el campo completo)."""
    if camera is None:
        return web.json_response({"error": "cámara no disponible"}, status=503)
    if request.method == "POST":
        try:
            z = float((await request.json()).get("zoom", 1.0))
        except Exception:
            return web.json_response({"error": "falta zoom (1 a 4)"}, status=400)
        if z > 1.0 and _autonomia_activa():
            return web.json_response({"error": "hay autonomía en marcha: el zoom debe ser 1x"}, status=409)
        camera.set_zoom(z)
    return web.json_response({"zoom": camera.zoom, "max": 4.0})


# ===== ENTORNO AUTOMÁTICO (interior / exterior) =====
entorno: "entorno_auto.EntornoAuto | None" = None
_entorno_ultimo_log = 0.0


def _rasgos_entorno() -> dict:
    """Rasgos de LIDAR + cámara del instante (corre en un hilo: el RANSAC de rectas no debe frenar el loop)."""
    r = dict(entorno_auto.rasgos_lidar(list(lidar_reactive.last_scan)) or {})
    frame = camera.ultimo_frame() if camera is not None else None
    # Con zoom la imagen es un recorte: sin cielo ni encuadre normal, así que no opina.
    if frame is not None and getattr(camera, "zoom", 1.0) <= 1.0:
        r.update(entorno_auto.rasgos_camara(frame[0]) or {})
    return r


async def _bucle_entorno():
    """1 Hz: puntúa el entorno, sugiere y (si está activo) cambia de modo con evidencia sostenida."""
    global _entorno_ultimo_log
    ciclos = 0
    while True:
        await asyncio.sleep(1.0)
        ciclos += 1
        try:
            if entorno is None or lidar_reactive.edad_scan_s() > 2.0:
                continue
            rasgos = await asyncio.to_thread(_rasgos_entorno)
            modo = conductor.modo if conductor else "exterior"
            quieto = actuator_proto is None or actuator_proto.segundos_quieto() > 1.0
            frente = lidar_reactive.distancias(state.config["slam"]).get("adelante")
            res = entorno.paso(rasgos, modo, puede_cambiar=quieto,
                               hueco_frente_m=None if frente is None else float(frente))
            if res["cambiar_a"]:
                nuevo = res["cambiar_a"]
                try:
                    difs = _aplicar_modo(nuevo, True)
                except perfiles_entorno.PerfilIncoherente as exc:
                    print(f"[entorno] perfil incoherente, no se cambia: {exc}", flush=True)
                    continue
                print(f"[entorno] {modo} -> {nuevo} (ema {entorno.ema:+.2f}, {res['motivo']})", flush=True)
                await state.broadcast({"type": "drive_mode", "data": {"modo": nuevo, "perfil": difs}})
                await state.broadcast({"type": "toast", "data": {
                    "kind": "info", "title": "Entorno automático",
                    "message": f"Pasó a {nuevo} ({len(difs)} umbrales): evidencia sostenida del LIDAR/cámara."}})
                entorno_auto.registrar(entorno_auto.CARPETA, {"t": time.time(), "evento": "cambio", "de": modo,
                                                              "a": nuevo, "ema": round(entorno.ema, 3)})
            ahora = time.time()
            if ahora - _entorno_ultimo_log >= 5.0 and entorno.puntuacion is not None:
                _entorno_ultimo_log = ahora
                entorno_auto.registrar(entorno_auto.CARPETA, {
                    "t": ahora, "rasgos": {k: v for k, v in entorno.rasgos.items()},
                    "puntuacion": round(entorno.puntuacion, 3), "ema": round(entorno.ema, 3),
                    "modo": modo, "sugerido": entorno.sugerido, "auto": entorno.activo,
                    "etiqueta": entorno.etiqueta_vigente(), "zoom": getattr(camera, "zoom", 1.0) if camera else 1.0})
            if ciclos % 2 == 0:
                await state.broadcast({"type": "entorno", "data": entorno.estado(modo)})
        except Exception as exc:
            print(f"[entorno] bucle falló: {type(exc).__name__}: {exc}", flush=True)


async def entorno_handler(request: web.Request):
    """GET /api/entorno — qué cree el robot que es el entorno. POST {"activo": true|false} lo activa/apaga
    (persistente). Apagado solo SUGIERE: no cambia nada."""
    if entorno is None:
        return web.json_response({"error": "entorno automático no inicializado"}, status=503)
    if request.method == "POST":
        try:
            activo = (await request.json()).get("activo")
        except Exception:
            activo = None
        if not isinstance(activo, bool):
            return web.json_response({"error": "activo debe ser true/false"}, status=400)
        entorno.activo = activo
        state.config = config_store.update_section("control", {"entorno_auto": activo})
        await state.broadcast({"type": "toast", "data": {
            "kind": "info", "title": "Entorno automático",
            "message": "ACTIVADO: elegirá interior/exterior solo." if activo else "Apagado: solo sugiere."}})
    return web.json_response(entorno.estado(conductor.modo if conductor else None))


async def asistente_estado_handler(request: web.Request):
    """GET /api/asistente/estado — foto compacta de TODO lo que hace falta para manejar el robot
    de forma asistida (Claude u otro operador remoto): hueco libre del LIDAR, pose en el mapa,
    autonomía en marcha, enlace con la C3, seguridad e inclinación. Solo lectura."""
    d = lidar_reactive.distancias(state.config["slam"])
    pose = ros_bridge.pose_en_mapa() if ros_bridge.available else None
    st = state.actuator_status or {}
    env = state.env_telemetry or {}
    return web.json_response({
        "t": time.time(),
        "lidar": {"hueco_m": {k: (None if v is None else round(float(v), 2)) for k, v in d.items()},
                  "edad_scan_s": round(min(lidar_reactive.edad_scan_s(), 999.0), 2)},
        "pose_mapa": pose,
        "ros": bool(ros_bridge.available),
        "c3": {"conectada": bool(enlace_c3 is not None and enlace_c3.connected),
               "failsafe": st.get("failsafe"), "via": st.get("via")},
        "autonomia": {"activa": _autonomia_activa(), "mision": bool(state.mission_active),
                      "explorador": explorador.snapshot() if explorador is not None else None,
                      "patrulla_activa": bool(patrol is not None and patrol.activo),
                      "waypoints": len(patrol.waypoints) if patrol is not None else 0,
                      "paso": paso_robotico.estado if paso_robotico is not None else None},
        "seguridad": {"modo_seguro_habilitado": modo_seguro_habilitado, **estado_seguridad()},
        "imu": {**{k: env.get(k) for k in ("accelX", "accelY", "accelZ", "gyroX", "gyroY", "gyroZ")},
                "fresco": _imu_fresco(),
                "nivel_calibrado": bool(state.config["calibration"].get("imu_gravedad_nivel"))},
        "modo_manejo": state.config["control"].get("modo_manejo"),
    })


async def mission_stop_handler(request: web.Request):
    # La patrulla primero: parar el actuador antes dejaría el bucle vivo y
    # volvería a mandarle una meta a Nav2 acto seguido.
    if mision_plantas is not None and mision_plantas.estado not in ("inactiva", "fin"):
        mision_plantas.detener("operador")
        _mision_vision_override.clear()
    if patrol is not None:
        patrol.detener()
        patrol.grabacion_auto(False)
    if explorador is not None:
        explorador.detener()
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
        # Para comprobar el signo del IMU: levantar el morro a mano y ver si
        # `cabeceo_deg` se vuelve negativo (ver calibration.imu_cabeceo_signo).
        "cabeceo_imu": cabeceo_imu.resumen(),
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
_GPU_CARGA = Path("/sys/devices/platform/bus@0/17000000.gpu/load")
_muestras_gpu: list[float] = []


def _carga_gpu_pct() -> float | None:
    """Media de las últimas lecturas: el valor instantáneo salta de 0 a 90
    entre dos inferencias de YOLO."""
    try:
        _muestras_gpu.append(int(_GPU_CARGA.read_text()) / 10.0)
    except (OSError, ValueError):
        return None
    del _muestras_gpu[:-5]
    return round(sum(_muestras_gpu) / len(_muestras_gpu), 1)


VDD_IN_AVISO_V = 4.80


def _vdd_in_v():
    """Tensión VDD_IN de la Jetson en voltios (INA3221 de la placa), o None si no se puede leer."""
    for f in glob.glob("/sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*/in*_label"):
        try:
            if open(f).read().strip() == "VDD_IN":
                return round(int(open(f[:-len("_label")] + "_input").read()) / 1000.0, 3)
        except (OSError, ValueError):
            continue
    return None


def _temperatura_zona(tipo: str) -> float | None:
    """Temperatura de la zona térmica del kernel de ese tipo (p. ej.
    "tj-thermal", la unión del chip, o "iwlwifi_1", la tarjeta WiFi/BT)."""
    for zona in Path("/sys/class/thermal").glob("thermal_zone*"):
        try:
            if (zona / "type").read_text().strip() == tipo:
                t = int((zona / "temp").read_text()) / 1000.0
                return round(t, 1) if 0 < t < 150 else None
        except (OSError, ValueError):
            continue
    return None


def _temperatura_c() -> float | None:
    """La del chip (unión, "tj"). NO la zona más caliente: esa era la tarjeta
    WiFi a 79 °C y el dashboard la mostraba como la del sistema."""
    return _temperatura_zona("tj-thermal") or _temperatura_zona("cpu-thermal")


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
                # jtop nombra las zonas en minúscula ("Temp tj", "Temp cpu"):
                # con "Temp CPU" esto salía siempre vacío.
                stats["temp_c"] = (jtop_reader.stats.get("Temp tj")
                                   or jtop_reader.stats.get("Temp cpu"))
            except Exception:
                pass
        # jtop no da la carga de la GPU en esta Orin (salía siempre vacía):
        # se lee del propio driver, en tantos por mil.
        if stats["gpu_pct"] is None:
            stats["gpu_pct"] = _carga_gpu_pct()
        if stats["temp_c"] is None:
            stats["temp_c"] = _temperatura_c()
        # La tarjeta WiFi/BT va aparte: es la que se calienta (AP + cliente +
        # Bluetooth en una sola radio) y la que se apaga sola si pasa de ~100.
        stats["temp_wifi_c"] = _temperatura_zona("iwlwifi_1")
        # Tensión de entrada de la Jetson (INA3221). Va regulada (~5,05 V), pero con la batería floja cae bajo carga: el 2026-10-23 la
        # Jetson se apagó sin aviso en plena prueba. Aviso una vez por caída por debajo de VDD_IN_AVISO_V.
        stats["vdd_in_v"] = _vdd_in_v()
        if stats["vdd_in_v"] is not None:
            bajo = stats["vdd_in_v"] < VDD_IN_AVISO_V
            if bajo and not getattr(state, "_aviso_tension", False):
                print(f"[energia] VDD_IN {stats['vdd_in_v']:.2f} V: batería baja, cargar antes de seguir", flush=True)
                await state.broadcast({"type": "toast", "data": {"kind": "warning", "title": "Batería baja",
                                                                 "message": f"La Jetson recibe {stats['vdd_in_v']:.2f} V: carga la batería antes de seguir."}})
            state._aviso_tension = bajo

        state.system_stats = stats
        health_data = _salud_completa()

        # --- ACTUALIZAR COMUNICACIÓN AUTOMÁTICA ---
        if auto_comunicacion and enlace_c3:
            auto_comunicacion.actualizar_estado_vias(
                ble_conectado=enlace_c3.ble.connected,
                wifi_conectado=(enlace_c3.wifi.conectado
                               if enlace_c3.wifi else False)
            )
        # --- FIN ACTUALIZACIÓN ---

        await state.broadcast({"type": "system_stats", "data": stats})
        if odom_webcam is not None:
            await state.broadcast({"type": "odom_webcam", "data": odom_webcam.snapshot()})
        if explorador is not None:
            await state.broadcast({"type": "exploracion", "data": explorador.snapshot()})
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
ble_link: BleActuatorLink | None = None
enlace_c3: EnlaceC3 | None = None
auto_comunicacion = None  # Sistema automático de comunicación
# Botón para desactivar/activar modo seguro. APAGADO por defecto y persistente (safety.modo_seguro_activo).
modo_seguro_habilitado = bool(state.config.get("safety", {}).get("modo_seguro_activo", False))


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

    global actuator_proto, ble_link, enlace_c3
    ctrl = state.config["control"]
    ble_link = BleActuatorLink(
        v_max=0.55, omega_max=1.5)
    # Vía de respaldo por WiFi (ver enlace_c3.py). Si el puerto no se puede
    # abrir, queda solo el BLE, como antes.
    wifi_c3 = await udp_actuator_link.abrir(lambda _trama: None)
    if wifi_c3 is not None:
        wifi_c3.on_cfg = c3_pines.on_cfg
    uart_c3 = None
    if ctrl.get("c3_usb", True):
        # Cable USB a la C3 (CDC del chip): vía fiable, ~1 ms de ida y vuelta. Se busca por by-id (el ttyACMn cambia).
        puerto_c3 = next(iter(sorted(Path("/dev/serial/by-id").glob("usb-Espressif_USB_JTAG_serial*"))), None)
        import uart_link
        uart_c3 = uart_link.UartActuatorLink(v_max=0.55, omega_max=1.5, port=str(puerto_c3 or "/dev/ttyACM0"), esclavo=True)
    enlace_c3 = EnlaceC3(ble_link, wifi_c3, wifi_only=True, uart=uart_c3)
    actuator_proto = ActuatorController(enlace_c3)
    enlace_c3._on_status = actuator_proto.on_status
    enlace_c3.start()

    # --- SISTEMA AUTOMÁTICO DE COMUNICACIÓN ---
    global auto_comunicacion
    try:
        if os.environ.get("AGROBOT_AUTO_COMUNICACION", "0") != "1":
            raise RuntimeError("desactivado: requiere corregir interfaz WiFi y contrato de configuración")
        auto_comunicacion = await inicializar_auto_comunicacion(
            config_store=state.config,
            modo_wifi="patrullaje"
        )
        print("[servidor] ✓ Sistema automático de comunicación iniciado", flush=True)
    except Exception as e:
        auto_comunicacion = None
        print(f"[servidor] ✗ Error en auto-comunicación: {e}", flush=True)
    # --- FIN SISTEMA AUTOMÁTICO ---

    # Inicializar sistema de evasión de obstáculos (NUEVO 2026-09-13)
    global obstacle_controller
    obstacle_controller = ControladorEvasion(
        callback_comando=actuator_proto.send_command,
        callback_parada=actuator_proto.emergency_stop,
        callback_debug=lambda msg: print(f"[EVASION] {msg}")
    )

    global conductor
    conductor = ConductorPulsos(send_command=actuator_proto.send_command,
                                emergency_stop=actuator_proto.emergency_stop,
                                get_config=lambda: state.config["control"])
    modo_guardado = state.config["control"].get("modo_manejo", "exterior")
    conductor.modo = modo_guardado if modo_guardado in MODOS else "exterior"
    global paso_robotico
    paso_robotico = PasoRobotico(
        ordenar=_pulso_robotico,
        parar=conductor.parar,
        conductor_activo=lambda: conductor.activo,
        distancias=lambda: lidar_reactive.distancias(state.config["slam"]),
        rumbo_deg=lambda: current_pose().get("theta_deg"),
        edad_scan_s=lidar_reactive.edad_scan_s,
        umbral_freno_m=lambda: (state.config["slam"]["margen_parada_cm"]
                                + state.config["slam"]["deslizamiento_cm"]) / 100.0)
    global grabador
    try:
        grabador = Grabador(
            leer={
                "velocidades": actuator_proto.velocidades,
                "c3": lambda: state.actuator_status,
                "pose": current_pose,
                "distancias": lambda: lidar_reactive.distancias(state.config["slam"]),
                "edad_scan": lidar_reactive.edad_scan_s,
                "t_scan": lambda: lidar_reactive.last_scan_at,
                "scan": lambda: lidar_reactive.last_scan,
                "suelo": _suelo_para_grabar,
                "imu": lambda: state.env_telemetry,
                "seguridad": estado_seguridad,
                "pulso_accion": lambda: conductor._accion_actual,
                "modo": lambda: conductor.modo,
            },
            # La cámara se crea MÁS ABAJO en este mismo arranque: se busca al
            # pedir cada foto, no ahora (si no, nunca grababa fotos).
            obtener_frame=lambda: camera.ultimo_frame() if camera is not None else None,
            cv2=cv2, config=state.config)
        app["grabador_task"] = asyncio.create_task(_bucle_grabador())
        print(f"[grabador] sesión en {grabador.dir}", flush=True)
    except Exception as exc:
        grabador = None
        print(f"[grabador] no arrancó: {type(exc).__name__}: {exc}", flush=True)
    app["rampa_task"] = asyncio.create_task(_bucle_rampa())
    app["vigia_lidar_task"] = asyncio.create_task(_vigia_lidar())
    app["vigia_reloj_task"] = asyncio.create_task(_vigia_reloj())
    app["vigia_asistido_task"] = asyncio.create_task(_vigia_asistido())
    app["servo_macetas_task"] = asyncio.create_task(_bucle_servo_macetas())
    app["zoom_auto_task"] = asyncio.create_task(_bucle_zoom_auto())
    app["zupt_task"] = asyncio.create_task(_bucle_zupt())
    app["modo_seguro_task"] = asyncio.create_task(_bucle_modo_seguro())
    global entorno
    entorno = entorno_auto.EntornoAuto(modelo=entorno_auto.cargar_modelo())
    entorno.activo = bool(state.config.get("control", {}).get("entorno_auto", False))
    app["entorno_task"] = asyncio.create_task(_bucle_entorno())
    _cargar_modelo_transitabilidad()
    app["suelo_libre_task"] = asyncio.create_task(_bucle_suelo_libre())
    app["barrido_tilt_task"] = asyncio.create_task(_bucle_barrido_tilt())

    # Patrulla autónoma. Se crea acá y no arriba porque sus callbacks
    # necesitan el `actuator_proto` que se acaba de construir.
    global patrol
    patrol = PatrolController(
        send_nav_goal=ros_bridge.send_nav_goal,
        cancel_nav_goal=ros_bridge.cancel_nav_goal,
        estado_meta=ros_bridge.estado_meta,
        get_pose=ros_bridge.pose_en_mapa,
        barrido=_barrido_camara,
        on_event=_patrol_event,
        ruta_waypoints=WAYPOINTS_PATH,
        distancia_puntos=lambda: state.config["autonomia"]["distancia_puntos_cm"],
        conducir=lambda accion, maniobra=True: conductor.ordenar(
            accion,
            v_max=state.config["control"]["max_v_mps"] * state.config["autonomia"]["escala_velocidad"],
            omega_max=state.config["control"]["max_omega_rps"],
            maniobra=maniobra),
        parar=lambda: conductor.parar(),
        distancias=lambda: lidar_reactive.distancias(state.config["slam"]),
        edad_scan=lidar_reactive.edad_scan_s,
        get_costmap=lambda: costmap,
    )
    print(f"[patrulla] lista — {len(patrol.waypoints)} waypoints grabados")

    # Lo aprendido en el simulador y de las correcciones del operador. Ambos
    # son opcionales: si no hay ficheros, el robot se comporta como siempre.
    global aprendizaje, politica
    aprendizaje = Aprendizaje()
    politica = Politica()
    print(f"[aprendizaje] {aprendizaje.resumen()['maniobras']} maniobras · "
          f"política {'cargada' if politica.disponible else 'no disponible'}")

    cabeceo_imu.signo = 1.0 if state.config["calibration"].get("imu_cabeceo_signo", 1) >= 0 else -1.0

    global seguidor_sendero, explorador
    seguidor_sendero = SeguidorSendero(
        get_config=lambda: state.config["autonomia"],
        obtener_frame=lambda: camera.ultimo_frame() if camera is not None else None,
        camara_al_frente=_servo_al_frente,
        get_geometria=_geometria_camara,
        get_costmap=lambda: costmap)
    # FSM conmutable detrás de un flag OFF por defecto: `maquina_estados.
    # MaquinaEstados` tiene la MISMA interfaz que ExploradorReactivo (drop-in),
    # añade estados explícitos + telemetría de transiciones y ganchos de escaneo
    # de planta. Validada en el simulador (AGROBOT_FSM, A/B 2026-10-07). Se
    # activa con `autonomia.maquina_estados: true`.
    _ClaseExpl = ExploradorReactivo
    if state.config["autonomia"].get("maquina_estados", False):
        try:
            from maquina_estados import MaquinaEstados as _ClaseExpl
            print("[exploración] usando MaquinaEstados (FSM)", flush=True)
        except Exception as exc:
            print(f"[exploración] FSM no disponible ({exc}); uso ExploradorReactivo", flush=True)
    explorador = _ClaseExpl(
        distancias=_distancias_explorador,
        get_pose=ros_bridge.pose_en_mapa,
        conducir=_conducir_pulso,
        parar=lambda: conductor.parar(),
        escanear=patrol.escanear,
        on_event=lambda d: _evento("exploracion", d),
        grabar_recorrido=lambda activa: patrol.grabacion_auto(activa),
        edad_scan=lidar_reactive.edad_scan_s,
        get_config=lambda: state.config["autonomia"],
        seguidor=seguidor_sendero,
        aprendizaje=aprendizaje,
        sensores=_percepcion_robot,
        politica=politica if state.config["autonomia"].get("politica_activa") else None,
        get_costmap=lambda: costmap,
    )

    # Guardián del sendero: correlaciona LIDAR + cámara + IMU y apunta cada corrección (autonomia.guardian_sendero_activo, apagado por defecto)
    try:
        import guardian_sendero
        explorador.guardian = guardian_sendero.GuardianSendero(guardian_sendero.MemoriaCorrecciones())
        explorador.sensores_guardian = _sensores_guardian
        explorador.scan_cb = _scan_explorador
        explorador.ocupado_cb = lambda: conductor.activo
        explorador.geom_cb = lambda: state.config["slam"]
    except Exception as exc:  # noqa: BLE001
        print(f"[guardián] no disponible: {exc}", flush=True)

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
    _crear_vision_activa()
    _crear_mision_plantas()
    try:
        import especie_clip
        especie_clip.precargar()          # CLIP tarda en cargar: que esté listo antes de la primera planta
    except Exception as exc:
        print(f"[especie] {exc}", flush=True)

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


# =============================================
# ===== CALIBRACIÓN DE NAVEGACIÓN =====
# =============================================

async def _nav_cal_loop():
    global _nav_cal_stats
    prev_pose = None
    dist_total = 0.0
    poses_count = 0
    try:
        while nav_logger and nav_logger._activo:
            await asyncio.sleep(0.5)
            pose = current_pose()
            if not pose:
                continue
            cfg = state.config["slam"]
            d = lidar_reactive.distancias(cfg)
            frontal_cm = round(d["adelante"] * 100) if d.get("adelante") is not None else None
            izq_cm = round(d["izquierda"] * 100) if d.get("izquierda") is not None else None
            der_cm = round(d["derecha"] * 100) if d.get("derecha") is not None else None
            # Velocidades: lo PEDIDO sale del actuador (tras la rampa) y lo
            # REAL del EKF, que es la única medida de verdad (rf2o + giroscopio).
            vel = actuator_proto.velocidades() if actuator_proto is not None else {}
            nav_logger.log_pose(
                pose["x"], pose["y"], pose.get("theta_deg", 0),
                vx_desired=round(vel.get("v_aplicada", 0.0), 3),
                omega_desired=round(vel.get("omega_aplicada", 0.0), 3),
                vx_real=round(pose.get("vx", 0.0), 3),
                omega_real=round(pose.get("vyaw", 0.0), 3),
                lidar_frontal=frontal_cm, lidar_izq=izq_cm, lidar_der=der_cm,
            )
            poses_count += 1
            if prev_pose:
                dist_total += math.hypot(pose["x"] - prev_pose["x"],
                                         pose["y"] - prev_pose["y"])
            prev_pose = pose
            _nav_cal_stats = {
                "activo": True,
                "session_id": nav_logger.session_id,
                "poses": poses_count,
                "distancia_m": round(dist_total, 3),
                "frontal_cm": frontal_cm, "izq_cm": izq_cm, "der_cm": der_cm,
                "pose_x": round(pose["x"], 3),
                "pose_y": round(pose["y"], 3),
                "pose_theta": round(pose.get("theta_deg", 0), 1),
                "waypoints": len(patrol.waypoints) if patrol else 0,
            }
    except asyncio.CancelledError:
        raise


async def nav_cal_start_handler(request: web.Request):
    global nav_logger, _nav_cal_task, _nav_cal_stats
    if nav_logger and nav_logger._activo:
        return web.json_response({"error": "calibración ya en curso"}, status=409)
    nav_logger = NavigationLogger()
    await nav_logger.iniciar()
    _nav_cal_stats = {"activo": True, "session_id": nav_logger.session_id,
                      "poses": 0, "distancia_m": 0}
    _nav_cal_task = asyncio.create_task(_nav_cal_loop())
    await state.broadcast({"type": "toast", "data": {
        "kind": "success", "title": "Calibración",
        "message": f"Sesión {nav_logger.session_id} iniciada — conducí el robot por la habitación.",
    }})
    return web.json_response({"ok": True, "session_id": nav_logger.session_id})


async def nav_cal_stop_handler(request: web.Request):
    global nav_logger, _nav_cal_task, _nav_cal_stats
    if not nav_logger or not nav_logger._activo:
        return web.json_response({"error": "no hay calibración en curso"}, status=409)
    session_id = nav_logger.session_id
    nav_logger.detener()
    if _nav_cal_task:
        _nav_cal_task.cancel()
        _nav_cal_task = None
    await asyncio.sleep(0.3)
    stats = analizar_sesion(session_id)
    try:
        from analizar_navegacion import AnalizadorNavegacion
        analizador = AnalizadorNavegacion(session_id)
        mejoras = analizador.sugerir_mejoras()
        lidar_a = analizador.analizar_distancias_lidar()
        vel_a = analizador.analizar_velocidades()
    except Exception as exc:
        print(f"[nav_cal] error en análisis: {exc}")
        mejoras, lidar_a, vel_a = [], {}, {}
    result = {**stats, "lidar_analysis": lidar_a,
              "velocity_analysis": vel_a, "mejoras": mejoras}
    _nav_cal_stats = {"activo": False, **result}
    nav_logger = None
    await state.broadcast({"type": "toast", "data": {
        "kind": "success", "title": "Calibración terminada",
        "message": f"Sesión {session_id}: {stats['poses_count']} poses, "
                   f"{stats['distancia_recorrida_m']:.2f} m recorridos.",
    }})
    return web.json_response(result)


async def nav_cal_status_handler(request: web.Request):
    if not _nav_cal_stats:
        return web.json_response({"activo": False})
    return web.json_response(_nav_cal_stats)


# =============================================
# ===== ENTRENAMIENTO / TRAINING DASHBOARD =====
# =============================================
TRAINING_DIR = Path.home() / ".agrobot" / "training"

_training_data = {
    "furrows": [],
    "scan_points": [],
    "corrections": [],
}


def _load_training_data():
    f = TRAINING_DIR / "training_data.json"
    if f.exists():
        try:
            _training_data.update(json.loads(f.read_text()))
        except Exception:
            pass


def _save_training_data():
    TRAINING_DIR.mkdir(parents=True, exist_ok=True)
    (TRAINING_DIR / "training_data.json").write_text(
        json.dumps(_training_data, indent=2))


_load_training_data()


async def training_correct_pose_handler(request: web.Request):
    if not ros_bridge.available:
        return web.json_response(
            {"error": "ROS no disponible"}, status=503)
    body = await request.json()
    x, y = float(body["x"]), float(body["y"])
    theta = float(body.get("theta_deg", 0.0))
    asistido.evento_pose(x, y, theta, current_pose())
    ros_bridge.publish_initial_pose(x, y, theta)
    _training_data["corrections"].append({
        "ts": time.time(), "x": x, "y": y, "theta_deg": theta})
    _save_training_data()
    await state.broadcast({"type": "toast", "data": {
        "kind": "success", "title": "Pose corregida",
        "message": f"x={x:.2f} y={y:.2f} θ={theta:.0f}°",
    }})
    return web.json_response({"ok": True})


async def training_save_furrows_handler(request: web.Request):
    body = await request.json()
    _training_data["furrows"] = body.get("furrows", [])
    _save_training_data()
    return web.json_response({"ok": True, "count": len(_training_data["furrows"])})


async def training_save_scan_points_handler(request: web.Request):
    body = await request.json()
    _training_data["scan_points"] = body.get("scan_points", [])
    _save_training_data()
    return web.json_response({"ok": True, "count": len(_training_data["scan_points"])})


async def training_generate_route_handler(request: web.Request):
    body = await request.json()
    furrows = body.get("furrows", _training_data.get("furrows", []))
    spacing_m = float(body.get("spacing_m", 0.5))
    waypoints = []
    for fi, furrow in enumerate(furrows):
        pts = furrow.get("points", [])
        if len(pts) < 2:
            continue
        total_len = 0
        segs = []
        for i in range(1, len(pts)):
            dx = pts[i]["x"] - pts[i-1]["x"]
            dy = pts[i]["y"] - pts[i-1]["y"]
            seg_len = math.sqrt(dx*dx + dy*dy)
            segs.append(seg_len)
            total_len += seg_len
        if total_len < 0.01:
            continue
        n_pts = max(2, int(total_len / spacing_m) + 1)
        for j in range(n_pts):
            frac = j / (n_pts - 1)
            target_dist = frac * total_len
            accum = 0
            for i, sl in enumerate(segs):
                if accum + sl >= target_dist:
                    t = (target_dist - accum) / sl if sl > 0 else 0
                    wx = pts[i]["x"] + t * (pts[i+1]["x"] - pts[i]["x"])
                    wy = pts[i]["y"] + t * (pts[i+1]["y"] - pts[i]["y"])
                    dx = pts[i+1]["x"] - pts[i]["x"]
                    dy = pts[i+1]["y"] - pts[i]["y"]
                    theta = math.degrees(math.atan2(dy, dx))
                    waypoints.append({"x": round(wx, 3), "y": round(wy, 3),
                                      "theta_deg": round(theta, 1)})
                    break
                accum += sl
        if fi % 2 == 1:
            waypoints[-n_pts:] = reversed(waypoints[-n_pts:])
    scan_pts = body.get("scan_points", _training_data.get("scan_points", []))
    for sp in scan_pts:
        waypoints.append({"x": sp["x"], "y": sp["y"], "theta_deg": 0,
                          "scan": True})
    if patrol is not None and waypoints:
        patrol.reemplazar_waypoints(
            [{"x": w["x"], "y": w["y"], "theta_deg": w.get("theta_deg", 0)}
             for w in waypoints if not w.get("scan")])
    return web.json_response({"ok": True, "waypoints": waypoints,
                              "count": len(waypoints)})


async def training_data_handler(request: web.Request):
    return web.json_response(_training_data)


async def training_log_correction_handler(request: web.Request):
    body = await request.json()
    entry = {
        "ts": time.time(),
        "type": body.get("type", "drive"),
        "vx": body.get("vx", 0),
        "omega": body.get("omega", 0),
        "pose": current_pose(),
    }
    _training_data["corrections"].append(entry)
    if len(_training_data["corrections"]) % 20 == 0:
        _save_training_data()
    return web.json_response({"ok": True})


# =============================================
# ===== CLAVE DEL DASHBOARD =====
# =============================================
# Sin esto cualquiera en la red (o por Tailscale) podía mover el robot. La
# clave vive FUERA del repo; si el archivo no existe el dashboard queda
# abierto como antes, para no dejar a nadie fuera por un despliegue a medias.
# El navegador la pide una vez (Basic) y desde ahí vale una cookie de sesión,
# que es lo que lleva el WebSocket: no todos los navegadores reenvían el
# Basic en el handshake de /ws. Lo local (127.0.0.1: tools/sensores.py, tests)
# no la necesita.
CLAVE_DASHBOARD_PATH = Path(os.environ.get(
    "AGROBOT_CLAVE_DASHBOARD", Path.home() / ".config/agrobot/clave_dashboard"))
USUARIO_DASHBOARD = "agrobot"
COOKIE_SESION = "agrobot_sesion"


def _leer_clave_dashboard() -> str:
    try:
        return CLAVE_DASHBOARD_PATH.read_text().strip()
    except OSError:
        return ""


def _middleware_clave(clave: str):
    import base64, hashlib, hmac, secrets
    esperado = "Basic " + base64.b64encode(f"{USUARIO_DASHBOARD}:{clave}".encode()).decode()
    # Token derivado de la clave + sal de este arranque: cambiar la clave o
    # reiniciar el backend invalida las sesiones viejas.
    token = hashlib.sha256((secrets.token_hex(16) + clave).encode()).hexdigest()

    @web.middleware
    async def exigir_clave(request: web.Request, handler):
        if request.remote in ("127.0.0.1", "::1"):
            return await handler(request)
        if hmac.compare_digest(request.cookies.get(COOKIE_SESION, ""), token):
            return await handler(request)
        if hmac.compare_digest(request.headers.get("Authorization", ""), esperado):
            resp = await handler(request)
            if not resp.prepared:
                resp.set_cookie(COOKIE_SESION, token, httponly=True, samesite="Strict")
            return resp
        return web.Response(status=401, text="Clave requerida",
                            headers={"WWW-Authenticate": 'Basic realm="AgroBot"'})
    return exigir_clave


# --- HANDLERS DE COMUNICACIÓN AUTOMÁTICA ---
async def comunicacion_estado_handler(request: web.Request):
    """Retorna estado de todos los sistemas de comunicación."""
    if auto_comunicacion is None:
        return web.json_response({"error": "no disponible"}, status=503)
    return web.json_response(auto_comunicacion.resumen_completo())


async def wifi_modo_handler(request: web.Request):
    """Cambia entre 'estacion' (Lora_Wan) y 'patrullaje' (AP propio)."""
    if auto_comunicacion is None:
        return web.json_response({"error": "no disponible"}, status=503)

    try:
        data = await request.json()
        modo = data.get("modo")
        if modo not in ["estacion", "patrullaje"]:
            return web.json_response(
                {"error": "modo inválido (debe ser 'estacion' o 'patrullaje')"},
                status=400
            )

        await auto_comunicacion.cambiar_modo_wifi(modo)
        return web.json_response({"ok": True, "modo": modo})
    except Exception as e:
        return web.json_response({"error": str(e)}, status=400)


async def diagnostico_wifi_handler(request: web.Request):
    """Retorna análisis detallado del WiFi (caídas, salud, etc)."""
    if auto_comunicacion is None or auto_comunicacion.diagnostico is None:
        return web.json_response({"error": "no disponible"}, status=503)

    import asyncio
    analisis = await auto_comunicacion.diagnostico.analizar_caidas()
    return web.json_response(analisis)


# Manual demo: a single command path and truthful status.
async def motor_comando_handler(request):
    try:
        data = await request.json()
        v, w = demo_command(data.get("v",0), data.get("omega",0), data.get("duty",100))
    except (ValueError, TypeError, AttributeError):
        return web.json_response({"error":"orden invalida"}, status=400)
    if v == 0 and w == 0:
        _parada_total("motor HTTP cero")
        return web.json_response({"status":"parada solicitada"})
    if enlace_c3 is None or not enlace_c3.connected:
        return web.json_response({"error":"C3 apagada o sin enlace"}, status=503)
    if conductor is not None:
        conductor.parar()
    if not await _c3_ready_for_drive():
        return web.json_response({"error":"Firmware o configuración pendiente; abre Motores y servo"},status=409)
    actuator_proto.send_command(v,w,0)
    return web.json_response({"status":"orden enviada", "referencia_v":v, "referencia_w":w,
                              "velocidad_medida":False, "caduca_s":DEADMAN_S})

async def motor_parar_handler(request):
    hecho = _parada_total("motor HTTP parada")
    return web.json_response({"status":"parada solicitada", "detenido":hecho,
                              "c3_conectada":bool(enlace_c3 and enlace_c3.connected)})

async def motor_toggle_safeguard_handler(request):
    return web.json_response({"error":"El timeout no se desactiva", "safeguard_enabled":True}, status=409)

async def estado_completo_handler(request):
    link = enlace_c3.resumen() if enlace_c3 is not None else {}
    wifi = link.get("wifi") or {}
    env = state.env_telemetry or {}
    received = env.get("_received_at")
    age = max(0,time.time()-received) if received else None
    return web.json_response({"timestamp":time.time(), "perfil":"demo-manual-trasero-v1",
        "c3":{"conectado":bool(link.get("conectado")), "firmware_verificado":False},
        "wifi":wifi, "ble":{"conectado":bool(link.get("ble"))},
        "motor":{"status":"enlace disponible" if link.get("conectado") else "apagado o desconectado",
                 "referencias":actuator_proto.velocidades() if actuator_proto else None,
                 "velocidad_medida":False},
        "imu":{"edad_s":age,"vigente":age is not None and age<1.0},
        "safeguard_enabled":True,"deadman_s":DEADMAN_S})

# --- FIN HANDLERS ---


def create_app() -> web.Application:
    clave = _leer_clave_dashboard()
    if clave:
        app = web.Application(middlewares=[_middleware_clave(clave)])
    else:
        print(f"[seguridad] SIN clave de dashboard ({CLAVE_DASHBOARD_PATH} no existe): "
              f"cualquiera en la red puede mover el robot")
        app = web.Application()
    c3_setup.on_claim = lambda: _parada_total("configuracion C3", remote=False)
    app.router.add_get("/api/c3/pines", c3_pines.get_handler)
    app.router.add_post("/api/c3/pines", c3_pines.set_handler)
    app.router.add_post("/api/c3/pines/reset", c3_pines.reset_handler)
    app.router.add_post("/api/c3/pines/intercambiar", c3_pines.swap_handler)
    app.router.add_post("/api/c3/prueba", c3_pines.test_handler)
    app.router.add_get("/api/c3/limites", c3_pines.lim_get_handler)
    app.router.add_post("/api/c3/limites", c3_pines.lim_set_handler)
    app.router.add_post("/api/c3/limites/reset", c3_pines.lim_reset_handler)
    app.router.add_get("/api/c3/setup", c3_setup.get_handler)
    app.router.add_post("/api/c3/setup/{action}", c3_setup.post_handler)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/video", video_handler)
    app.router.add_post("/api/video/record", video_record_handler)
    app.router.add_post("/api/drive", drive_handler)
    app.router.add_post("/api/emergencia", emergencia_handler)
    app.router.add_get("/api/paso", paso_get_handler)
    app.router.add_post("/api/paso", paso_post_handler)
    app.router.add_get("/api/drive/mode", drive_mode_handler)
    app.router.add_post("/api/drive/mode", drive_mode_handler)
    app.router.add_get("/api/sendero/vista.jpg", sendero_vista_handler)
    app.router.add_get("/api/suelo/vista.jpg", suelo_vista_handler)
    app.router.add_get("/api/suelo", suelo_estado_handler)
    app.router.add_post("/api/evasion/start", evasion_start_handler)
    app.router.add_post("/api/evasion/stop", evasion_stop_handler)
    app.router.add_get("/api/evasion", evasion_get_handler)
    app.router.add_post("/api/nav/reset_map", nav_reset_map_handler)
    app.router.add_get("/api/mapas", mapas_listar_handler)
    app.router.add_post("/api/mapas/guardar", mapas_guardar_handler)
    app.router.add_post("/api/mapas/cargar", mapas_cargar_handler)
    app.router.add_post("/api/mapas/digitalizar", mapas_digitalizar_handler)
    app.router.add_post("/api/exploracion/regresar", exploracion_regresar_handler)
    app.router.add_post("/api/simulador/subir_datos", simulador_subir_datos_handler)
    app.router.add_get("/api/mapas/digital/{nombre}/{archivo}", mapas_digital_archivo_handler)
    app.router.add_get("/api/giro_cerrado", giro_cerrado_handler)
    app.router.add_post("/api/giro_cerrado", giro_cerrado_handler)
    app.router.add_get("/api/maniobra_giro", maniobra_giro_handler)
    app.router.add_post("/api/maniobra_giro", maniobra_giro_handler)
    app.router.add_get("/api/simulador", simulador_handler)
    app.router.add_post("/api/simulador", simulador_handler)
    app.router.add_get("/api/camara/cajas", camara_cajas_handler)
    app.router.add_get("/api/camara/cruda.jpg", camara_cruda_handler)
    app.router.add_get("/api/misiones", misiones_handler)
    app.router.add_post("/api/misiones", misiones_handler)
    app.router.add_get("/api/mision_plantas", mision_plantas_handler)
    app.router.add_post("/api/mision_plantas", mision_plantas_handler)
    app.router.add_get("/api/vision_activa", vision_activa_get_handler)
    app.router.add_post("/api/vision_activa", vision_activa_post_handler)
    app.router.add_get("/api/patrol", patrol_status_handler)
    app.router.add_post("/api/patrol/record", patrol_record_handler)
    app.router.add_post("/api/exploracion/start", exploracion_start_handler)
    app.router.add_post("/api/exploracion/stop", exploracion_stop_handler)
    app.router.add_post("/api/patrol/auto", patrol_auto_handler)
    app.router.add_post("/api/patrol/clear", patrol_clear_handler)
    app.router.add_post("/api/patrol/scan", patrol_scan_handler)
    app.router.add_post("/api/saludo", saludo_handler)
    app.router.add_post("/api/session-report", session_report_handler)
    app.router.add_get("/api/models", models_list_handler)
    app.router.add_post("/api/models/load", models_load_handler)
    app.router.add_get("/api/models/presets", models_presets_handler)
    app.router.add_get("/api/hardware", hardware_get_handler)
    app.router.add_post("/api/hardware", hardware_update_handler)
    app.router.add_get("/api/calibration", calibration_get_handler)
    app.router.add_post("/api/calibration", calibration_update_handler)
    app.router.add_post("/api/mission/start", mission_start_handler)
    app.router.add_post("/api/mission/stop", mission_stop_handler)
    app.router.add_get("/api/asistente/estado", asistente_estado_handler)
    app.router.add_get("/api/entorno", entorno_handler)
    app.router.add_post("/api/entorno", entorno_handler)
    app.router.add_post("/api/calibracion/imu_nivel", calibrar_imu_nivel_handler)
    app.router.add_get("/api/camara/zoom", camara_zoom_handler)
    app.router.add_post("/api/camara/zoom", camara_zoom_handler)
    app.router.add_get("/api/params", params_get_handler)
    app.router.add_post("/api/params/{section}", params_update_handler)
    app.router.add_post("/api/lidar/scan", lidar_scan_handler)
    app.router.add_get("/api/obstacle_status", obstacle_status_handler)
    app.router.add_get("/api/costmap", costmap_status_handler)
    app.router.add_get("/api/costmap/grid", costmap_grid_handler)
    app.router.add_post("/api/costmap/reset", costmap_reset_handler)
    app.router.add_post("/api/costmap/save", local_map_save_handler)
    app.router.add_post("/api/costmap/load", local_map_load_handler)
    app.router.add_get("/api/costmap/maps", local_map_list_handler)
    app.router.add_post("/api/costmap/delete", local_map_delete_handler)
    app.router.add_get("/api/ocupado", ocupado_handler)
    app.router.add_get("/api/health", health_handler)
    app.router.add_get("/api/seguridad", seguridad_get_handler)
    app.router.add_post("/api/seguridad/rearmar", seguridad_rearmar_handler)
    app.router.add_post("/api/seguridad/modo-seguro", modo_seguro_toggle_handler)
    app.router.add_get("/api/captures", captures_list_handler)
    app.router.add_post("/api/captures/clear", captures_clear_handler)
    app.router.add_post("/api/nav/set_goal", nav_set_goal_handler)
    app.router.add_post("/api/nav/cancel", nav_cancel_handler)
    app.router.add_post("/api/nav/restart", nav_restart_handler)
    app.router.add_post("/api/nav/save_map", nav_save_map_handler)
    app.router.add_get("/api/nav/mode", nav_mode_handler)
    app.router.add_post("/api/nav/mode", nav_mode_handler)
    app.router.add_post("/api/nav_cal/start", nav_cal_start_handler)
    app.router.add_post("/api/nav_cal/stop", nav_cal_stop_handler)
    app.router.add_get("/api/nav_cal/status", nav_cal_status_handler)
    from recoleccion import install as install_collection
    install_collection(app, lambda: {
        "env": state.env_telemetry, "actuator_status": state.actuator_status,
        "setpoint": {"servo_deg": state.servo_angle, "led": state.led_on},
        "velocities": actuator_proto.velocidades(), "pose": current_pose(),
        "pose_age_s": pose_edad_s(), "pose_reliable": pose_confiable(),
        "lidar_scan": lidar_reactive.last_scan, "lidar_timestamp": lidar_reactive.last_scan_at,
        "lidar_age_s": lidar_reactive.edad_scan_s(), "security": estado_seguridad(),
        "system_stats": state.system_stats, "camera": camera.snapshot() if camera else None,
        "detections": state.detections[-20:], "configuration": state.config,
        "mission_active": state.mission_active,
    }, lambda: camera.ultimo_frame() if camera else None)
    from etiquetado import install as install_etiquetado      # cajas de plaga a mano sobre fotos propias (/etiquetar.html)
    install_etiquetado(app, CAPTURES_DIR)
    app.router.add_static("/captures", CAPTURES_DIR, show_index=False)

    # --- Training dashboard endpoints ---
    app.router.add_post("/api/training/correct_pose", training_correct_pose_handler)
    app.router.add_post("/api/training/save_furrows", training_save_furrows_handler)
    app.router.add_post("/api/training/save_scan_points", training_save_scan_points_handler)
    app.router.add_post("/api/training/generate_route", training_generate_route_handler)
    app.router.add_get("/api/training/data", training_data_handler)
    app.router.add_post("/api/training/log_correction", training_log_correction_handler)
    app.router.add_get("/api/asistido", asistido_estado_handler)
    app.router.add_post("/api/asistido/reanudar", asistido_reanudar_handler)
    app.router.add_post("/api/asistido/soltar", asistido_soltar_handler)
    app.router.add_post("/api/asistido/aplicar", asistido_aplicar_handler)

    async def _training_redirige(request):          # el dashboard de entrenamiento ya no existe: vive en Manejo asistido
        raise web.HTTPFound("/#asistido")
    app.router.add_get("/training.html", _training_redirige)

    # --- RUTAS DE COMUNICACIÓN AUTOMÁTICA ---
    app.router.add_get("/api/comunicacion/estado", comunicacion_estado_handler)
    app.router.add_post("/api/wifi/modo", wifi_modo_handler)
    app.router.add_get("/api/diagnostico/wifi", diagnostico_wifi_handler)
    # --- RUTAS DE MOTOR v2.0 ---
    app.router.add_post("/api/motor/comando", motor_comando_handler)
    app.router.add_post("/api/motor/parar", motor_parar_handler)
    app.router.add_post("/api/motor/toggle-safeguard", motor_toggle_safeguard_handler)  # ⭐ NUEVO
    app.router.add_get("/api/estado/completo", estado_completo_handler)
    # --- FIN RUTAS ---

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
        if grabador is not None:
            grabador.cerrar()
        if camera is not None:
            camera.stop()
        if ble_link is not None:
            await (enlace_c3 or ble_link).stop()
        # --- LIMPIAR COMUNICACIÓN AUTOMÁTICA ---
        global auto_comunicacion
        if auto_comunicacion is not None:
            await auto_comunicacion.detener()
        # --- FIN LIMPIEZA ---
        ros_bridge.shutdown()
        global _sesion_http
        if _sesion_http is not None and not _sesion_http.closed:
            await _sesion_http.close()
            _sesion_http = None

    app.on_startup.append(start_background)
    app.on_cleanup.append(on_cleanup)
    load_yolo_model(app)
    @web.middleware
    async def demo_guard(request, handler):
        blocked = {"/api/mission/start", "/api/exploracion/start", "/api/evasion/start",
                   "/api/patrol/auto", "/api/nav/set_goal", "/api/paso", "/api/nav_cal/start"}
        if (request.method == "POST" and request.path in blocked
                and state.config.get("control", {}).get("solo_manual", False)):
            return web.json_response({"error":"Perfil de demostracion: solo manejo manual"},status=409)
        return await handler(request)
    app.middlewares.append(demo_guard)
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
