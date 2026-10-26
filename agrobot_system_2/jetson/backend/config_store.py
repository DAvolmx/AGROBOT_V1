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
        # Gravedad (ax, ay, az en g) medida con el robot quieto y NIVELADO (POST /api/calibracion/imu_nivel).
        # None = sin calibrar. Ver imu_nivel.py.
        "imu_gravedad_nivel": None,
        # SCD40 quemado (2026-10-23): rellena CO₂/temperatura/humedad con valores simulados, marcados en
        # `_sensors_simulated` (ver ambiente_simulado.py). Apagar en cuanto haya un sensor nuevo.
        "ambiente_simulado": False,
    },
    "hardware": {
        "camera_enabled": True,
        # Exposición/enfoque de la cámara USB; vacío = valores del driver. Claves: auto_exposure, exposure, gain, brightness, contrast,
        # autofocus, focus, backlight. Elegirlos con tools/ajustar_camara.py.
        "camara_ajustes": {},
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
        # Puertas contra falsos positivos (2026-10-22): vegetación mínima dentro de la caja y persistencia en N de M fotogramas.
        "plaga_requiere_vegetacion": True,
        "plaga_vegetacion_min": 0.05,
        "plaga_bordes_max": 0.30,          # textura de pasto/paja (≈0,35) frente a hoja de col (≤0,26)
        "plaga_persistencia_ventana": 5,
        "plaga_persistencia_min": 3,
        "confidence_threshold": 0.45,
        "clip_record_threshold": 0.60,
        # Lado de la imagen que entra a YOLO. Medido en la Jetson el
        # 2026-09-29 (yolo11s, GPU): 640 px = 72-96 ms, 480 px = 44 ms. El
        # robot va lento y en los barridos la plaga ocupa mucha imagen.
        "imgsz": 480,
        # Tope de inferencias por segundo. Era 4 (0,25 s) cuando YOLO corría en
        # CPU (~460 ms/frame); en GPU tarda 32-71 ms, así que 10 Hz cabe y la
        # detección acompaña al robot en tiempo real. Bajarlo libera GPU/CPU.
        "infer_hz_max": 10,
        # Cuándo una detección dispara el barrido de cámara: "siempre",
        # "autonomo" (solo con la patrulla en marcha) o "nunca". Con
        # cualquiera, la detección tiene que persistir y el robot haberse
        # movido desde el último escaneo (patrol.py).
        # Clases que son la PLANTA (no una plaga): al detectarlas el robot escanea, pero no se emite el
        # aviso «Plaga detectada» ni se manda alerta al gateway. Las demás clases se tratan como plaga.
        "clases_planta": ["col_planta", "col_cogollo", "maceta_planta", "maceta"],
        "modelo_maceta": "agrobot_maceta_det.pt",   # segundo modelo en la cámara (detector de macetas, Kaggle 2026-10-22)
        "maceta_conf": 0.30,
        "escanear_al_detectar": "siempre",
    },
    "slam": {
        # TODO EN CENTÍMETROS Y GRADOS, que es como se mide en el robot.
        #
        # Carrocería (medida por el usuario el 2026-09-17). Sirve para restar
        # media carrocería a las distancias: así "adelante: 30 cm" significa
        # 30 cm hasta tocar, no hasta el centro del robot. El `footprint` de
        # nav2_params.yaml es aparte y va en metros: si cambia esto, cambiarlo.
        "robot_largo_cm": 49,
        # Corrección ML de la odometría por scans (slam_correccion.py, MLP entrenado en Kaggle 2026-10-21). APAGADA: en escenarios
        # simulados NO vistos mejoró el giro pero empeoró la posición (ATE 15,0 -> 17,1 cm). Solo para pruebas en sombra.
        "correccion_ml_activa": False,
        "correccion_ml_pesos": "models/slam_correccion/pesos_mlp.npz",
        # El LIDAR NO está en el centro (croquis 2026-10-20): a 24 cm del eje trasero. Distancia del LIDAR a la cara
        # delantera y a la trasera, para restar la carrocería correcta a cada lado (ver slam.distancias).
        "lidar_adelante_cm": 18.5,
        "lidar_atras_cm": 30.5,
        # 37 cm = ancho exterior del eje TRASERO (medido). Era 35 hasta el
        # 2026-10-14: subestimaba la envolvente 1 cm por lado en los ocho
        # sitios que comprueban holgura, incluido el hueco visual de suelo
        # libre. El delantero mide 19, pero la holgura la manda lo más ancho.
        # Fuente única: geometria_robot.ANCHO_ENVOLVENTE_M.
        "robot_ancho_cm": 38,
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
        # El freno también usa el suelo libre por CÁMARA (suelo_libre.py):
        # ve lo bajo que el LIDAR no. Falso para volver a solo LIDAR.
        "freno_camara": True,
        # filtro complementario de pose: 0 = confía solo en la velocidad
        # comandada (dead-reckoning puro), 1 = confía solo en el giroscopio
        "pose_gyro_trust": 0.8,
    },
    "autonomia": {
        # Rumbo fusionado y maniobras (2026-10-22, rumbo_fusion.py + tanda de entrenamiento). APAGADO por defecto: se prueba primero con
        # tools/probar_recta.py y se enciende con POST /api/params/autonomia {"rumbo_fusion_activo": true}.
        "rumbo_fusion_activo": False,
        "rumbo_zona_muerta_deg": 6.0,
        "rumbo_aprende": 0.15,
        "rumbo_pulso_escala": 0.3,       # duración del pulso de corrección de rumbo respecto al giro normal
        "man_histeresis_cm": 12.0,
        "man_pausa_s": 0.0,
        # Maniobra de esquina (2026-10-24): 1 / 0 / 1 = comportamiento de siempre. Valores del simulador: ver docs/REPORTE_MEJORAS_20261024.
        "man_giro_pulsos": 1,
        "man_retroceso_cm": 0.0,
        "man_atras_pulsos": 1,
        # Maniobras específicas (2026-10-24; simuladas por misiones). TODAS APAGADAS: ir al hueco del scan, rumbo a la meta y escape de nicho.
        "mx_hueco": False,
        "mx_hueco_m": 1.0,
        "mx_grados_pulso": 6.0,
        "mx_meta": False,
        "mx_rumbo_tol_deg": 25.0,
        "mx_bordear_s": 12.0,
        "mx_anti_dither": False,
        "mx_ventana_s": 25.0,
        "mx_giro_largo": 5,
        # ===== E11 (2026-10-26): estrategia entrenada en simulación (docs/estrategias/e11_entrenado). Interruptor general: e11_activo. =====
        # Predictor de colisión por pulso + corredor de barrido + regreso al punto de inicio por ruta A* (botón «Regresar al inicio» / POST /api/exploracion/regresar).
        # NO medido en el robot real. Para apagarlo todo: e11_activo=false (el explorador vuelve a ser el de antes).
        "e11_activo": True,
        "e11_predictor": True,
        "e11_pred_avance_m": 0.289, "e11_pred_atras_m": 0.166, "e11_pred_margen_m": 0.067, "e11_pred_giro_extra_m": 0.033, "e11_pred_relajar_s": 25.0,
        "e11_barrido": True, "e11_barrido_half_m": 0.256,
        "e11_plan_inflar_m": 0.23, "e11_plan_mirada_m": 0.75, "e11_plan_cada_s": 2.6, "e11_plan_giro_deg": 19.6, "e11_regreso_radio_m": 0.5,
        "e11_ida_max_m": 0.0, "e11_ida_max_s": 0.0,
        # Sonar sobre el mapa acumulado (sonar_mapa.py): elige el hueco al bloquearse en 360°, no solo con los conos.
        "e11_sonar": False, "e11_sonar_prof_m": 1.0, "e11_sonar_w_meta": 1.0, "e11_sonar_w_visitas": 0.5,          # disparadores automáticos del regreso (0 = solo manual)
        # Guardián del sendero (2026-10-24, guardian_sendero.py). APAGADO: se prueba primero en un pasillo con el robot en el suelo.
        "guardian_sendero_activo": False,
        "guardian_tol_cm": 6.0,
        "guardian_tol_deg": 4.0,
        "guardian_usar_memoria": True,
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
        "servo_baja_macetas": True,   # en autonomía, la cámara baja si la maceta está baja en la imagen (servo_bajo.py)
        "asistido_reanuda_s": 2.5,   # segundos sin órdenes manuales tras los que la autonomía interrumpida se reanuda sola (0 = nunca)
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
        # Parada por incertidumbre (2026-10-14): ciclos consecutivos SIN
        # ninguna referencia del sendero (ni hileras, ni hueco del costmap, ni
        # cámara con confianza) tras los que el explorador para en vez de
        # seguir recto a ciegas. 0 = desactivado. 5 ciclos ~= una racha real,
        # no un fotograma con un reflejo.
        "sendero_parada_incierta_ciclos": 5,
        # Transitabilidad del corredor (2026-10-14). El robot mide 19 cm de
        # ancho delante y 37 DETRÁS: por el hueco que pasa el morro no pasa el
        # eje trasero. Con esto activo, un sendero medido más estrecho que la
        # envolvente + holgura para la autonomía en vez de seguir su centro.
        # La holgura por lado la define geometria_robot; aquí se puede ajustar.
        # "aviso" = cuenta cuántas veces el sendero medido no admitiría la
        # envolvente y sigue; "frenar" = para; cualquier otro valor lo apaga.
        # Arranca en "aviso" (decisión del operador 2026-10-14): la holgura de
        # 19 cm sale del simulador y sus senderos miden 60-80 cm, así que
        # primero se recogen datos y después se decide si frenar.
        "sendero_si_no_cabe": "aviso",
        # 14 cm = donde desaparecen los choques en el simulador con el control
        # ajustado (con el viejo hacían falta 19). Era 6, a ojo: con 6 choca en
        # el 62-88 % de los episodios. Ver geometria_robot.HOLGURA_PROCEDENCIA.
        "sendero_holgura_lateral_cm": 14,
        # Política entrenada en el simulador (politica.pt). Apagada por
        # defecto: se enciende cuando se ha comprobado que mejora.
        "politica_activa": False,
        # Botón AUTO = misión de plantas en maceta (explorar, mapear, escanear macetas). False = patrulla/exploración de antes.
        "auto_mision_plantas": True,
        "camara_obstaculos": True,        # el explorador usa también el suelo libre de la cámara (obstáculos bajos que el LIDAR no ve)
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
        "camara_altura_m": 0.40,
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
        # Modo de manejo por pulsos (pulse_drive.MODOS). Tiene que estar aquí:
        # load() descarta las claves que no están en DEFAULTS, y por eso el
        # modo elegido se perdía en cada reinicio (2026-10-02). "preciso" =
        # pulsos cortos, lo que pidió el usuario.
        "modo_manejo": "preciso",
        # Elegir solo interior/exterior según LIDAR+cámara (entorno_auto.py). False = solo SUGIERE.
        "entorno_auto": False,
        # Giro INVERTIDO por cableado: «pido izquierda y va a la derecha». Se niega omega JUSTO al enviar a la C3
        # (ActuatorController._giro_signo), así que vale para manual, exploración, patrulla y Nav2, y la odometría
        # y el giroscopio siguen coincidiendo con lo comandado. True desde 2026-10-20 (medido por el operador).
        "invertir_giro": True,
        # Tope de lo que Nav2 puede mandar a los motores (la patrulla y «ir a meta» pasan por aquí). Se recorta en el
        # backend: se cambia sin reiniciar ROS. Subirlos solo tras ver que navega bien a esta velocidad.
        "nav2_v_max_mps": 0.25,
        "nav2_w_max_rps": 0.6,
        # Perfil de demostración: True bloquea TODO lo autónomo (exploración, misión, patrulla, Nav2...).
        # Hasta el 2026-10-16 estaba fijo en el código; ahora se elige aquí.
        "solo_manual": False,
        # Giro EN EL SITIO (v=0, omega!=0): solo con el firmware «suave» (las traseras pivotan).
        "pivote_en_sitio": False,
    },
    "safety": {
        # Frenado por LIDAR/cámara SOLO cuando el robot va solo (exploración, patrulla, misión, Nav2):
        # no avanza si hay algo a menos de margen+deslizamiento (slam.*) ni sin datos frescos del LIDAR.
        # El manejo manual no se toca. Apagarlo solo para pruebas.
        "freno_lidar_autonomo": True,
        # ANTICOLISIÓN CON RETROCESO (anticolision.py, 2026-10-21): si el robot VA A TOCAR algo delante (hueco < disparo_m) cancela
        # el avance y retrocede retroceso_m si detrás hay sitio. Independiente del freno de arriba. Solo ve lo que ve el LIDAR
        # (plano a 37,5 cm): no detecta cajas, bolsas, mantas ni macetas bajas.
        "anticolision_activa": True,
        "anticolision_disparo_m": 0.12,
        "anticolision_retroceso_m": 0.15,
        "anticolision_v_retroceso": 0.15,
        "anticolision_margen_atras_m": 0.20,
        "anticolision_parar_atras_m": 0.08,
        "anticolision_soltar_s": 0.3,
        "anticolision_scan_max_s": 1.0,
        # MODO SEGURO (FDIR, modo_seguro.py): al entrar detiene la autonomía y queda enclavado hasta
        # rearmar. APAGADO por defecto desde 2026-10-19 (pedido del operador): la inclinación del IMU
        # (28-77°) lo disparaba durante las pruebas de exploración/SLAM. Se activa desde el dashboard.
        "modo_seguro_activo": False,
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

from vision_activa import DEFECTOS as _VISION_ACTIVA_DEFECTOS   # noqa: E402  (calibracion_camara es lo único que importa)
DEFAULTS["vision_activa"] = dict(_VISION_ACTIVA_DEFECTOS)         # APAGADA por defecto (2026-10-22)
DEFAULTS["control"].setdefault("c3_wifi_only", False)
DEFAULTS["control"].setdefault("c3_usb", True)       # cable USB a la C3 como vía principal (2026-10-21)

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
