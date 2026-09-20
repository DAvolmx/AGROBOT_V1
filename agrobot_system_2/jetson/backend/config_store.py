"""
Almacén de configuración persistente ("EEPROM" simulada).

Guarda en un JSON local: offsets de calibración (servo, pitch, roll de la
IMU), configuración de hardware (cámara habilitada, baudrate del LIDAR) y
qué modelo YOLO está activo. Se lee al arrancar y se reescribe atómicamente
en cada guardado — no se pierde si el proceso se cae.
"""
import json
import tempfile
from pathlib import Path
from threading import Lock

CONFIG_PATH = Path(__file__).parent / "agrobot_config.json"

DEFAULTS = {
    "calibration": {
        # se SUMA al ángulo pedido antes de mandarlo al servo, para corregir
        # el desalineo mecánico del montaje de la cámara (el firmware es
        # actuador puro: recibe el ángulo ya corregido y lo aplica tal cual)
        "servo_offset_deg": 0,
        # bias de reposo del giroscopio (MPU6050): lo que marca cada eje con
        # el robot QUIETO. Se RESTA de cada lectura entrante. Corregir el eje
        # Z es lo que más importa: alimenta la estimación de rumbo
        # (PoseEstimator / EKF) y un bias ahí es deriva de heading constante.
        # Antes estos campos se llamaban imu_pitch/roll_offset_deg, nombre
        # incorrecto: el MPU6050 entrega velocidad angular (°/s), no ángulos.
        "gyro_bias_x_dps": 0.0,
        "gyro_bias_y_dps": 0.0,
        "gyro_bias_z_dps": 0.0,
        # Sentido del eje de cabeceo del MPU6050 tal como está montado.
        # Comprobación: con el robot quieto, levantar el morro a mano; en
        # /api/diagnostics el `cabeceo_deg` debe volverse NEGATIVO (positivo =
        # morro abajo). Si sube, poner -1 aquí.
        "imu_cabeceo_signo": 1,
    },
    "hardware": {
        "camera_enabled": True,
        "lidar_baud": 460800,
        # El C3 se comunica por UART, no por red: `actuator_ip` se elimino
        # el 2026-09-04. ttyTHS1 es el UART del cabezal de 40 pines de la
        # Orin. Ver jetson/backend/uart_link.py.
        "actuator_port": "/dev/ttyTHS1",
        "actuator_baud": 115200,
    },
    "ai": {
        "active_model": None,
    },
    "detection": {
        "confidence_threshold": 0.45,
        "clip_record_threshold": 0.60,
    },
    "slam": {
        # TODO EN CENTÍMETROS Y GRADOS, que es como se mide en el robot.
        #
        # Carrocería (medida por el usuario el 2026-09-17). Sirve para restar
        # media carrocería a las distancias: así "adelante: 30 cm" significa
        # 30 cm hasta tocar, no hasta el centro del robot. El `footprint` de
        # nav2_params.yaml es aparte y va en metros: si cambia esto, cambiarlo.
        "robot_largo_cm": 40,
        "robot_ancho_cm": 35,
        # Sectores del LIDAR. El trasero es el espejo del frontal.
        # Frontal estrecho = pasa por pasillos sin frenar por las paredes;
        # ancho = ve antes las esquinas, pero se frena más a menudo.
        "cono_frontal_deg": 40,
        "cono_lateral_deg": 60,
        # Hueco que se deja sin invadir al parar.
        "margen_parada_cm": 26,
        # Lo que sigue avanzando tras cortar motores (las ruedas quedan
        # sueltas). Se corta con esta antelación.
        "deslizamiento_cm": 40,
        # filtro complementario de pose: 0 = confía solo en la velocidad
        # comandada (dead-reckoning puro), 1 = confía solo en el giroscopio
        "pose_gyro_trust": 0.8,
    },
    "autonomia": {
        # Acelerador que usan los pulsos del modo exploración, como fracción
        # del máximo. Los motores no bajan de ~78 % de potencia, así que esto
        # solo limita el tope; para ir "despacio" lo que vale son los pulsos.
        "escala_velocidad": 0.5,
        # Un punto del recorrido cada tantos cm recorridos (grabación
        # automática y exploración).
        "distancia_puntos_cm": 50,
        # Decisiones del modo exploración, en cm de hueco LIBRE hasta tocar:
        "avance_libre_cm": 100,      # para lanzar un pulso de avance
        "lado_libre_cm": 60,         # para considerar un lado como salida
        "retroceso_libre_cm": 50,    # para poder retroceder
        "escaneo_periodo_s": 120,    # cada cuánto se para a barrer con la cámara
        # El barrido lo hace el BACKEND moviendo el servo paso a paso (como
        # el slider), no el flag de escaneo del firmware: así la duración y el
        # recorrido se cambian desde el dashboard sin reprogramar el ESP32.
        # Los ángulos se recortan igual a 25..110° en protocol.py.
        "escaneo_duracion_s": 10,
        "escaneo_angulo_min": 25,
        "escaneo_angulo_max": 110,
        # Encender el foco mientras barre (lo apaga al terminar, dejándolo
        # como estaba). El firmware ya no lo enciende solo: un actuador de
        # potencia no debe tener vías de encendido automático fuera de control
        # de la Jetson, porque si el barrido se corta se queda encendido.
        "escaneo_enciende_foco": True,
        # --- Seguir senderos (sendero.py) ---
        # Con esto activo, en vez de vagar el robot intenta ir centrado por el
        # camino: entre hileras con el LIDAR y/o por la cámara (bordes de paja
        # y plantas proyectados al suelo).
        "sendero_activo": False,
        "sendero_tolerancia_cm": 10,     # desvío que se tolera sin corregir
        "sendero_pared_max_cm": 150,     # más allá, ese lado no cuenta como hilera
        "sendero_vision": True,          # usar también la cámara
        # 0 = solo LIDAR, 1 = solo cámara. 0.5 = ambas por igual.
        "sendero_vision_peso": 0.5,
        # Qué se considera vegetación en HSV de OpenCV (matiz 0..179).
        "sendero_verde_h_min": 30,
        "sendero_verde_h_max": 90,
        "sendero_verde_s_min": 60,
        # La paja seca (mulch) de los costados también marca el borde del
        # sendero. Apagar si el camino mismo está cubierto de paja.
        "sendero_paja": True,
        # Borde por contraste con el color del propio sendero: se adapta a
        # bordes de paja, de maleza verde o de tierra distinta.
        "sendero_adaptativo": True,
        # Aplanar el gradiente de luz de la imagen antes de buscar los bordes
        # del sendero (viñeteo del objetivo, media imagen al sol). Ver
        # sendero.igualar_luz: sin esto el seguimiento cae a la mitad.
        "sendero_igualar_luz": True,
        # Corregir la inclinación de la cámara con el cabeceo que mide el IMU.
        # Sin esto, en terreno con baches el detector cree mirar a un sitio y
        # mira a otro (2° de cabeceo = 5 cm de error a 1 m).
        "compensar_cabeceo": True,
        # Política entrenada en el simulador (politica.pt). Apagada por
        # defecto: se enciende cuando se ha comprobado que mejora.
        "politica_activa": False,
    },
    "odometria_webcam": {
        # Odometría visual con la webcam (odometria_webcam.py). `habilitada`
        # mide y lo muestra en el dashboard; `publicar_ros` además lo manda al
        # EKF (/odom_webcam). Se deja apagado hasta comparar con el EKF: una
        # fuente mal calibrada ya descontroló la pose una vez (el acelerómetro,
        # 2026-09-06).
        "habilitada": True,
        "publicar_ros": False,
        # SIN MEDIR. Altura del centro del lente sobre el suelo y cuánto mira
        # hacia abajo. La escala del avance es proporcional a la altura.
        "camara_altura_m": 0.30,
        "camara_inclinacion_deg": 15.0,
        # Campo de visión HORIZONTAL. Medirlo: poner la cámara a una distancia
        # D de una pared y medir el ancho visible A: hfov = 2·atan(A / 2D).
        "camara_hfov_deg": 70.0,
        # Distancia del lente por delante del centro del robot.
        "camara_adelante_m": 0.15,
    },
    "control": {
        # Techos físicos del robot. Tienen que coincidir con V_MAX / VY_MAX /
        # OMEGA_MAX del firmware: si acá se pide más, el ESP32-C3 lo recorta
        # en silencio y el dashboard muestra una velocidad que el robot nunca
        # alcanza. El mando manual manda -1..1 (fracción), no m/s, y es ACÁ
        # donde esa fracción se convierte a unidades reales.
        "max_v_mps": 0.55,
        "max_omega_rps": 1.5,
        # Suavizado por software. Los motores no bajan del 78 % de potencia
        # (PWM 200..255 en el firmware), así que para ir más lento se trocea
        # el pulso: manda velocidad una fracción `suavidad_duty` de cada
        # ciclo y cero el resto. duty 1.0 = como antes, sin trocear.
        "suavidad_duty": 0.6,
        "suavidad_periodo_s": 0.30,
    },
    "safety": {
        "heartbeat_stale_warn_s": 5,
        "heartbeat_stale_err_s": 20,
        # limitador de rampa (slew-rate) sobre los comandos v/omega antes de
        # mandarlos al ESP32-C3 — no es PID (no hay encoder, no hay error
        # real que corregir), solo evita saltos bruscos de velocidad/giro
        # entre un comando y el siguiente. Máximo cambio permitido por segundo.
        "max_linear_accel_mps2": 1.0,
        "max_angular_accel_rps2": 3.0,
    },
    "navigation": {
        # "mapping"      -> corriendo ros2_ws/.../agrobot_mapping.launch.py
        # "localization" -> corriendo ros2_ws/.../agrobot_navigation.launch.py
        # Cambio 2026-09-10: "localization" ahora es el modo por defecto.
        # El robot inicia listo para navegar, no en mapeo.
        "mode": "localization",
        "map_name": "agrobot_map",
        "last_goal": None,
    },
}

_lock = Lock()


def load() -> dict:
    if not CONFIG_PATH.exists():
        save(DEFAULTS)
        return json.loads(json.dumps(DEFAULTS))
    with open(CONFIG_PATH) as f:
        data = json.load(f)
    # Rellena claves nuevas que no existan en un config viejo, Y PODA las
    # que ya no están en DEFAULTS. Sin la poda, un campo que se elimina del
    # código sigue vivo para siempre en el JSON del usuario y se sigue
    # sirviendo por la API como si existiera (pasó con imu_pitch_offset_deg
    # / imu_roll_offset_deg, que quedaron fantasma tras renombrarlos).
    merged = json.loads(json.dumps(DEFAULTS))
    for section, values in data.items():
        if section not in merged:
            continue
        for key, value in values.items():
            if key in merged[section]:
                merged[section][key] = value
    return merged


def save(data: dict) -> None:
    with _lock:
        fd, tmp_path = tempfile.mkstemp(dir=CONFIG_PATH.parent)
        with open(fd, "w") as f:
            json.dump(data, f, indent=2)
        Path(tmp_path).replace(CONFIG_PATH)


def update_section(section: str, values: dict) -> dict:
    data = load()
    data.setdefault(section, {}).update(values)
    save(data)
    return data
