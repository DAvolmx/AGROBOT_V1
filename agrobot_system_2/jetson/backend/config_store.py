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
        # Caja de la carrocería, centrada en el LIDAR. Medida del usuario
        # (2026-09-17): 40 cm de largo x 35 de ancho. Tiene que coincidir con
        # el `footprint` de nav2_params.yaml. El frenado mide dentro de un
        # corredor de este ancho.
        "robot_largo_m": 0.40,
        "robot_ancho_m": 0.35,
        # hueco que se deja libre al parar, medido desde la carrocería (m)
        "obstacle_margen_m": 0.15,
        # cuánto sigue avanzando el robot tras cortar los motores (m). Medido
        # por el usuario el 2026-09-17: ~80 cm. El firmware deja las ruedas
        # sueltas al parar; con freno activo este número bajaría mucho.
        "obstacle_deslizamiento_m": 0.80,
        # filtro complementario de pose: 0 = confía solo en la velocidad
        # comandada (dead-reckoning puro), 1 = confía solo en el giroscopio
        "pose_gyro_trust": 0.8,
    },
    "autonomia": {
        # Acelerador que usan los pulsos del modo exploración, como fracción
        # del máximo. Los motores no bajan de ~78 % de potencia, así que esto
        # solo limita el tope; para ir "despacio" lo que vale son los pulsos.
        "escala_velocidad": 0.5,
        # Un punto del recorrido cada tantos metros recorridos (grabación
        # automática y exploración). 0,50 m lo pidió el usuario el 2026-09-17.
        "distancia_puntos_m": 0.50,
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
