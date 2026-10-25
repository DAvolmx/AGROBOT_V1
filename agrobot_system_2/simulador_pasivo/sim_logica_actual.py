"""Simulación de la LÓGICA ACTUAL de movimiento del robot, en interiores.

Creado el 2026-10-15 para la presentación. Hasta hoy el banco usaba un
controlador continuo (v, omega) y el pivote DELANTE. Nada de eso es lo que corre
en el robot. Aquí se reproduce, eslabón por eslabón, lo que SÍ corre:

  explorador (decide cada 0,25 s)            -> jetson/backend/explorador.py
    -> ConductorPulsos.segmentos()           -> pulse_drive.py  (se IMPORTA el real)
    -> troceado por `suavidad_duty`          -> pulse_drive._tramo (copiado)
    -> rampa de aceleración de la Jetson     -> jetson_server._apply_ramp (copiado)
    -> demo_reference() y _pct()             -> manual_demo.py / ble_link.py (reales)
    -> trama de 8 bytes a 20 Hz
    -> mezcla de motores + rampa por rueda   -> firmware 1.2 (copiado, ver abajo)
    -> dinámica con el pivote TRASERO        -> dinamica_pasiva.py (girada 180°)
  LIDAR simulado -> slam.LidarReactive.distancias()  (se IMPORTA el real)

EL PIVOTE TRASERO. `dinamica_pasiva` modela un eje ANCHO FIJO (37 cm) y un módulo
ESTRECHO (19 cm) con pivote pasivo ±20° unido al chasis a una batalla de distancia.
Es exactamente el chasis ACTUAL si se gira 180°: el eje ancho fijo del modelo es el
delantero REAL y el módulo del pivote es el trasero REAL. Avanzar en la realidad
es retroceder en el modelo. Por eso aquí no se toca la física: se rota el marco.

    mundo real (x adelante, y izquierda)  ->  modelo: x' = -x, y' = -y, psi' = psi + pi
    ruedas del firmware [R1, L1, L2, R2]  ->  modelo [trasL, trasR, delL, delR]
                                              = [R1,  L1,  R2,  L2]

LÍMITES (importan para lo que se puede concluir):
  · La física es la del simulador: masa, inercias, fricción y respuesta del motor
    son HIPÓTESIS (`contrato_robot.json` calibrated=false). La velocidad de cada
    rueda se toma LINEAL con el duty (duty/100 * 0,55 m/s, la escala de transporte
    del firmware, NO medida). Se añade una zona muerta `dead` (duty mínimo que
    vence la fricción estática) porque es lo que más probablemente falle en la
    realidad y no se ha medido: se aleatoriza entre episodios.
  · El explorador real hace más cosas (callejones, bucles, retroceso, escaneo).
    Aquí solo está lo que decide un avance recto por un pasillo: medir el LIDAR,
    avanzar, corregir el centrado, o girar hacia el lado libre si hay algo delante.
    Los resultados de `esquina` son por tanto una aproximación.
  · Un LIDAR ideal con ruido gaussiano de 1 cm: sin reflejos, sin patas de mesa.
"""
from __future__ import annotations

import copy
import math
import os
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
BACKEND = AQUI.parent / "jetson" / "backend"
for p in (str(BACKEND), str(AQUI)):          # AQUI primero: gana en nombres repetidos
    if p in sys.path:
        sys.path.remove(p)
sys.path.insert(0, str(BACKEND))
sys.path.insert(0, str(AQUI))

import pulse_drive                                    # noqa: E402  (REAL)
from calibracion_real import CAL                       # noqa: E402  (datos reales del robot parado y de los pulsos)
from manual_demo import reference as demo_reference   # noqa: E402  (REAL)
from ble_link import _pct                             # noqa: E402  (REAL)
from slam import LidarReactive                        # noqa: E402  (REAL)
import rumbo_fusion as RF                             # noqa: E402  (REAL: giroscopio + paredes del LIDAR; ver jetson/backend/rumbo_fusion.py)
import huella_pasiva as H                             # noqa: E402
from dinamica_pasiva import DinamicaPasiva            # noqa: E402
from robot_rustico import parametros_rusticos         # noqa: E402

# ---------------------------------------------------------------- constantes
# Paso de integración de la dinámica. NO subir de 0,005: a 0,01 la dinámica de rueda es
# INESTABLE (inercia 0,002 con rigidez de neumático 45 y ganancia de motor .18): a los 3 s de
# marcha daba 23 cm/s con las ruedas de signos mezclados, cuando la solución convergida (0,005,
# 0,0025 y 0,001 coinciden) es 49 cm/s con las cuatro a -5,8 rad/s. Se descubrió el 2026-10-15
# al comparar rueda libre y frenada. Ojo: `evaluacion.py` del banco anterior usa 0,01.
SUBSTEP = 0.005
V_SCALE, W_SCALE = 0.55, 1.5            # = control_model.h = BleActuatorLink(v_max, omega_max)
MID = 0.185                             # del eje ANCHO FIJO al centro del robot
# Dónde está el módulo estrecho con pivote: "trasero" (chasis actual; marco girado 180°) o
# "delantero" (el chasis hasta el 2026-10-14; sin girar). Solo para COMPARAR orientaciones:
# AGROBOT_PIVOTE=delantero python3 entrenar_interior_estricto.py ...
PIVOTE = os.environ.get("AGROBOT_PIVOTE", "trasero")

# GEOMETRÍA del robot según el CROQUIS del operador (2026-10-20) y su CORRECCIÓN el mismo día:
#   · el módulo del PIVOTE va ATRÁS y es el de las ruedas MÁS SEPARADAS: 38 cm exteriores (en el modelo es el «front»).
#   · el eje FIJO es el DELANTERO y es el ESTRECHO: 28 cm (CONFIRMADO por el operador; la medida anterior de 19 cm
#     era de otra configuración) y lleva los motores MÁS FUERTES (en el modelo es el «rear», origen de coordenadas).
#   · cuerpo 26 x 32 cm; batalla 37 cm; LIDAR a 24 cm del eje TRASERO (el del pivote) -> a 13 cm del eje fijo, o sea
#     5,5 cm DELANTE del centro; y a 35-40 cm del suelo.
# AGROBOT_GEOMETRIA=anterior reproduce los resultados previos (pivote estrecho de 19 cm, LIDAR en el centro y ve todo).
GEOMETRIA = os.environ.get("AGROBOT_GEOMETRIA", "croquis")
if GEOMETRIA == "anterior":
    FIJO_EXT_M, PIVOTE_EXT_M, CUERPO_L, CUERPO_A = 0.37, 0.19, 0.35, 0.25
    LIDAR_DESDE_EJE_M, ALTURA_LIDAR_M = MID, 0.0
    FIJO_OPCIONES = (0.37,)
    BIAS_OPCIONES = (1.0, 2.0)
else:
    FIJO_EXT_M, PIVOTE_EXT_M, CUERPO_L, CUERPO_A = 0.28, 0.38, 0.32, 0.26
    LIDAR_DESDE_EJE_M, ALTURA_LIDAR_M = 2 * MID - 0.24, 0.375
    FIJO_OPCIONES = (0.28,)            # CONFIRMADO por el operador el 2026-10-20: el eje fijo delantero mide 28 cm
    # `front_drive_bias` multiplica la fuerza del módulo del PIVOTE (atrás). Los motores fuertes están en el eje fijo
    # delantero: el pivote tiene entre un 35 y un 70 % de esa fuerza.
    BIAS_OPCIONES = (0.35, 0.5, 0.7)
# El LIDAR es un PLANO horizontal a ALTURA_LIDAR_M: solo ve lo que sobresale de él. Altura de las macetas con
# planta (hipótesis): 0,45 m las hace visibles; con 0,20 m el LIDAR pasa por encima y el robot choca a ciegas.
ALTURA_MACETA_M = float(os.environ.get("AGROBOT_ALTURA_MACETA", "0.45"))
GROSOR_HAZ_M = 0.03
TICK_CONDUCTOR = 0.05                   # pulse_drive.TICK_S
TICK_JETSON = 0.05                      # CMD_SEND_HZ = 20
TICK_FW = 0.01                          # motorTask: 10 ms
TICK_EXPLORADOR = 0.25                  # explorador.TICK_S
DEADMAN_JETSON_S = 0.5                  # jetson_server.DEADMAN_S
DEADMAN_FW_S = 0.5                      # firmware DEADMAN_MS
RAMP_MAX_DT = 3.0 / 20                  # jetson_server.RAMP_MAX_DT
LATENCIA_ENLACE_S = 0.03                # Jetson -> C3 (UDP/WiFi), hipótesis


# ======================================================================
# Configuración que se está evaluando
# ======================================================================
@dataclass
class Config:
    """Todo lo que el operador puede cambiar mañana. Defecto = config VIVA."""
    nombre: str = "actual"
    # --- Jetson (agrobot_config.json) ---
    modo: str = "exterior"              # control.modo_manejo -> duración de pulsos
    escala_velocidad: float = 0.5       # autonomia.escala_velocidad
    max_v_mps: float = 1.5              # control.max_v_mps
    max_omega_rps: float = 4.0          # control.max_omega_rps
    suavidad_duty: float = 1.0          # control.suavidad_duty
    suavidad_periodo_s: float = 0.30
    accel_lin: float = 3.0              # safety.max_linear_accel_mps2
    accel_ang: float = 8.0              # safety.max_angular_accel_rps2
    avance_libre_cm: float = 100        # autonomia.avance_libre_cm
    sendero_tol_cm: float = 10          # autonomia.sendero_tolerancia_cm
    sendero_pared_max_cm: float = 150
    cono_frontal_deg: float = 10        # slam.cono_frontal_deg
    cono_lateral_deg: float = 60
    robot_largo_cm: float = 49          # slam.robot_largo_cm (croquis 2026-10-20: 18,5 cm delante + 30,5 cm detrás del LIDAR)
    robot_ancho_cm: float = 38          # slam.robot_ancho_cm = módulo del pivote, el más ancho
    lidar_adelante_cm: float = 18.5     # slam.lidar_adelante_cm: LIDAR -> cara delantera
    lidar_atras_cm: float = 30.5        # slam.lidar_atras_cm: LIDAR -> cara trasera
    # Tope de avance que la Jetson deja pasar, en % de duty. 60 = el real: pulse_drive recorta
    # a 0,60*V_SCALE y manual_demo.command a .60. Cambiarlo es un "¿y si?": exigiría tocar esos
    # DOS sitios (son límites de seguridad que puso Codex), NO es un parámetro de config.
    tope_duty: float = 60.0
    # control.invertir_giro del robot REAL (true): el cableado cruza los lados y jetson_server niega omega al enviarlo a la C3. El simulador
    # lo omitía y por eso su «izquierda» giraba a la derecha (descubierto el 2026-10-22 al medir el rumbo tras cada pulso).
    invertir_giro: bool = True
    # --- firmware 1.2 (/config) ---
    fw_fwd: float = 60.0
    fw_rev: float = 35.0
    fw_minp: float = 0.0
    fw_inner: float = 65.0
    fw_slew: float = 60.0
    fw_gap_ms: float = 25.0
    # Firmware flasheado: "skidsteer_v3" (original BLE+WiFi, PWM por software de 200 ms, sin
    # rampa, gira sobre su eje) o "1.2" (rampa, sin giro en el sitio, /config). Desde el
    # 2026-10-16 el robot lleva el skidsteer_v3; los fw_* solo afectan al "1.2".
    firmware: str = "maxfrente"
    # --- firmware "suave" v4 (limites configurables desde el dashboard, = suave_model.h) ---
    su_adel: float = 80.0
    su_atras: float = 60.0
    su_pivote: float = 100.0
    su_minp: float = 50.0
    su_acel: float = 120.0
    su_frena: float = 300.0
    su_tras: float = 130.0
    # --- firmware FUSION (v3 + suave): ver firmware/agrobot_c3_fusion/fusion_model.h ---
    fu_adel: float = 80.0
    fu_atras: float = 60.0
    fu_pivote: float = 100.0
    fu_minp: float = 30.0
    fu_acel: float = 200.0
    fu_frena: float = 400.0
    fu_tras: float = 130.0
    fu_dif_del: float = 0.0
    fu_periodo: float = 100.0
    fu_refuerzo: float = 100.0
    fu_comp_neg: float = 0.0
    fu_comp_pos: float = 0.0
    fu_freno_ms: float = 80.0           # freno activo (firmware 2026-10-23): 0 = rueda libre como antes
    # --- firmware «MAX FRENTE» (firmware/agrobot_c3_skidsteer_maxfrente): las ruedas delanteras (R1, L1) al 100 % de PWM en
    # el sentido que pida su lado; las traseras (pivote) a tras_factor x el duty proporcional (0 = sin ayuda trasera) ---
    tras_factor: float = 1.0
    # --- conducción: "pulsos" (explorador actual) o "continuo" (control_pasillo.PasilloController) ---
    control: str = "pulsos"
    pivote_en_sitio: bool = True       # manual_demo.reference(pivote=...): giro sin avanzar. True = como el agrobot_config.json real
    cc_kh: float = 1.2
    cc_kl: float = 2.0
    cc_v: float = 0.30
    cc_kp: float = 1.6
    cc_kd: float = 0.6
    cc_wmax: float = 0.7
    cc_nominal: float = 0.35
    cc_dstop: float = 0.30
    cc_dslow: float = 0.80
    # --- MANIOBRAS del explorador de pulsos (2026-10-22). Todas en 0/False = el explorador de siempre. Solo existen en el simulador:
    # llevarlas al robot exige portarlas a autonomia/pulse_drive y es decisión del operador. ---
    man_retroceso_cm: float = 0.0       # bloqueado a menos de esto: marcha atrás de 1 pulso antes de girar (0 = no)
    man_atras_pulsos: int = 1           # cuántos pulsos hacia atrás
    man_giro_pulsos: int = 1            # pulsos de giro seguidos al bloquearse (giro más decidido)
    man_desacel_cm: float = 0.0         # frente a menos de esto: avanza a man_desacel_factor de la velocidad (0 = no)
    man_desacel_factor: float = 0.6
    man_histeresis_cm: float = 0.0      # lados casi iguales: repite el último giro (evita titubear)
    man_pausa_s: float = 0.0            # espera tras girar para que el LIDAR se asiente antes de decidir
    man_escape_s: float = 0.0           # sin avanzar 2 cm en tanto tiempo: secuencia de escape (atrás + giro al lado libre)
    man_corr_doble: bool = False        # corrección de centrado en 2 pulsos si el error es el doble de la tolerancia
    # --- mantener el RUMBO (el robot real siempre cae hacia un lado): giroscopio + paredes del LIDAR fusionados (rumbo_fusion) ---
    rumbo_zm_deg: float = 0.0           # zona muerta del error de rumbo; 0 = apagado
    rumbo_paredes: bool = True          # corrige el giroscopio con el ángulo de las paredes cuando se ven
    rumbo_pulso_escala: float = 1.0     # duración del pulso de corrección de rumbo respecto al giro normal (<1 = giros más finos)
    rumbo_mirada_m: float = 0.0         # >0: apunta a un punto a esa distancia sobre el centro del pasillo (ángulo + desvío lateral, arXiv 2106.15029)
    rumbo_modo: str = "pulso"           # "pulso" = pulso de giro aparte; "arco" = el propio pulso recto lleva un omega de corrección (frena un lado, arXiv 2609.38715)
    rumbo_arco_k: float = 1.5           # omega de corrección por radián de error (modo arco)
    rumbo_arco_max: float = 0.4         # tope de ese omega (rad/s)
    rumbo_aprende: float = 0.3          # rapidez con que aprende hacia qué lado cae el robot en cada pulso recto
    # Paredes SÓLIDAS (2026-10-24): al chocar, el robot vuelve a su última pose sin contacto y pierde la velocidad (choque inelástico). Antes
    # atravesaba las paredes y solo contaba un contacto, así que en esquinas "ganaba" saliéndose del mapa. False = comportamiento anterior.
    paredes_solidas: bool = True
    # --- Guardián del sendero (2026-10-24, jetson/backend/guardian_sendero.py): mantiene el sendero con pulsos correctivos que aprende de
    # la correlación LIDAR + IMU + cámara + error de pose. `guardian_obj` = instancia compartida entre episodios (su memoria aprende) o None.
    # --- MANIOBRAS ESPECÍFICAS (2026-10-24, para las fallas de las misiones): rumbo a la meta, bordear con compromiso y escape de nicho
    mx_meta: bool = False               # al bloquearse elige el lado que acerca a la meta, lo mantiene `mx_bordear_s` s y, con el frente libre, vuelve al rumbo de la meta
    mx_bordear_s: float = 12.0
    mx_rumbo_tol_deg: float = 25.0
    mx_anti_dither: bool = False        # sin avanzar 0,45 m en `mx_ventana_s` s y con >= 8 bloqueos: atrás x3 + giro largo al lado CONTRARIO + adelante x3 y compromiso de lado
    mx_ventana_s: float = 25.0
    mx_giro_largo: int = 5
    mx_hueco: bool = False              # al bloquearse: busca en el scan el hueco libre (>= mx_hueco_m de profundidad y ancho para el robot) más cercano a la meta y gira hacia él
    mx_hueco_m: float = 1.0
    mx_grados_pulso: float = 6.0        # cuánto gira un pulso de giro (para convertir el ángulo del hueco en pulsos); el guardián lo aprende en el robot
    # --- misión (mision_sim.py): escanear y contar plantas mientras se conduce
    mision: bool = False
    mis_inspeccionar: bool = True
    mis_escaneo_s: float = 8.0
    mis_dist_insp_m: float = 0.5
    mis_fov_deg: float = 70.0
    mis_rango_m: float = 2.0
    mis_dist_visita_m: float = 0.9      # una planta pendiente que se está visitando se escanea desde esta distancia (0,9 = a la distancia de seguridad)
    mis_reintentos: int = 0             # repetir el escaneo si no detectó nada
    mx_barrido: bool = False            # el frente se mide en un CORREDOR del ancho del robot (no en un cono): ve lo que está a los costados de la trayectoria
    mx_ancho_half_m: float = 0.24       # media anchura del corredor de barrido (robot 38 cm = 0,19 + margen)
    # --- PREDICTOR DE COLISIÓN POR PULSO (2026-10-24): antes de cada pulso se barre la huella del robot con lo que se ve en el scan.
    # Diagnóstico previo: la mayoría de los contactos eran en marcha ATRÁS y en giros con el obstáculo detrás/al costado, no al avanzar.
    mx_prediccion: bool = False
    mx_pred_avance_m: float = 0.30      # cuánto puede avanzar un pulso hacia delante (barrido de la huella)
    mx_pred_atras_m: float = 0.22       # idem hacia atrás
    mx_pred_margen_m: float = 0.05      # holgura lateral/frontal añadida a la huella
    mx_pred_giro_m: float = 0.31        # radio que barre el chasis al girar (esquina más lejana al centro)
    # --- IDA Y VUELTA: al llegar a la meta da la vuelta y regresa al punto de inicio; el éxito exige volver
    mx_regreso: bool = False
    mx_visitar: bool = False            # ir a las plantas vistas y aún sin escanear (el objetivo pasa a ser la planta, no la meta)
    mx_visitar_m: float = 3.5
    mx_memoria_w: float = 0.0           # peso de la memoria de zonas visitadas al elegir hueco: prefiere lo menos pisado
    guardian: bool = False
    guardian_obj: object = None
    guardian_explorar: float = 0.0
    guardian_usar_memoria: bool = True
    guardian_tol_cm: float = 6.0
    guardian_tol_deg: float = 4.0

    def slam_cfg(self):
        return {"robot_largo_cm": self.robot_largo_cm, "robot_ancho_cm": self.robot_ancho_cm,
                "lidar_adelante_cm": self.lidar_adelante_cm, "lidar_atras_cm": self.lidar_atras_cm,
                "cono_frontal_deg": self.cono_frontal_deg, "cono_lateral_deg": self.cono_lateral_deg}


def config_actual() -> Config:
    """La que está en agrobot_config.json hoy (modo exterior, escala .5, duty 1.0)."""
    return Config()


def config_interior_perfil() -> Config:
    """El perfil de interior de perfiles_entorno.PERFILES['interior']."""
    return Config(nombre="perfil_interior", modo="interior", escala_velocidad=0.13,
                  avance_libre_cm=40, sendero_pared_max_cm=120, cono_frontal_deg=30)


# ======================================================================
# Firmware 1.2: mezcla y rampa por rueda (port de control_model.h / .ino)
# ======================================================================
def fw_mix(v: float, w: float, cfg: Config, limit: float = V_SCALE):
    """Port de agro::mix. Devuelve duties [R1, L1, L2, R2] en %."""
    if not (math.isfinite(v) and math.isfinite(w)) or limit <= 0:
        return [0.0] * 4
    lim = min(limit, V_SCALE)
    v = max(-lim, min(lim, v))
    if abs(v) < 0.002:
        return [0.0] * 4
    base = min(100 * abs(v) / V_SCALE, cfg.fw_rev if v < 0 else cfg.fw_fwd)
    steer = 0.0 if v < 0 else max(-1.0, min(1.0, w / W_SCALE))
    inner = cfg.fw_inner * 0.01
    left = base * (1 - inner * max(steer, 0.0))
    right = base * (1 - inner * max(-steer, 0.0))
    if cfg.fw_minp > 0:
        if 0 < left < cfg.fw_minp:
            left = cfg.fw_minp
        if 0 < right < cfg.fw_minp:
            right = cfg.fw_minp
    s = -1.0 if v < 0 else 1.0
    return [s * right, s * left, s * left, s * right]


class MotoresFirmware:
    """motorTask(): 4 canales, rampa, pausa al invertir, deadman, rueda libre."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.target = [0.0] * 4
        self.duty = [0.0] * 4
        self.dir = [0] * 4
        self.switch_at = [0.0] * 4
        self.refreshed = -1e9

    def set_targets(self, vals, t):
        self.target = list(vals)
        self.refreshed = t

    def tick(self, t):
        expire = (t - self.refreshed) > DEADMAN_FW_S
        out = []
        for i in range(4):
            value = 0.0 if expire else self.target[i]
            d = 1 if value > 0 else -1 if value < 0 else 0
            if d == 0:
                self.duty[i] = 0.0              # detachMotor(): rueda libre al instante
                self.dir[i] = 0
            else:
                if self.dir[i] != d:
                    self.duty[i] = 0.0
                    self.dir[i] = d
                    self.switch_at[i] = t
                if (t - self.switch_at[i]) * 1000.0 >= self.cfg.fw_gap_ms:
                    goal = abs(value)
                    # El .ino suma clip(meta-duty, ±slew*.01) por tick de 10 ms con duty en
                    # puntos de 0..100: slew=60 -> 0,6 puntos/tick = 60 %/s.
                    step = self.cfg.fw_slew * 0.01
                    self.duty[i] += max(-step, min(step, goal - self.duty[i]))
            out.append(self.dir[i] * self.duty[i])
        return out                                 # duty firmado real de cada canal, en %


# ======================================================================
# Firmware skid-steer v3 ORIGINAL (esp32c3_actuador_v3_skidsteer.ino / agrobot_c3_skidsteer)
# ======================================================================
SK_V_MAX, SK_OMEGA_MAX, SK_TRACK, SK_RADIO = 0.55, 1.5, 0.33, 0.06
SK_PERIODO_S = 0.200            # MOTOR_PERIODO_MS: PWM por SOFTWARE, 5 Hz
SK_DEAD_S = 0.020               # DEAD_TIME_MS al invertir
SK_BOOST, SK_UMBRAL, SK_HIST, SK_MON_S = 1.15, 0.85, 10, 0.100   # anti-hundimiento


def sk_mix(v: float, w: float, boost: float = 1.0):
    """Port de applyMotors(): devuelve (duty_izq, sent_izq, duty_der, sent_der) en %.
    Suposición: el lado "IZQ" del firmware es el izquierdo físico (lo que el operador deja
    configurado en el dashboard); omega>0 gira a la izquierda."""
    vx = max(-SK_V_MAX, min(SK_V_MAX, v))
    om = max(-SK_OMEGA_MAX, min(SK_OMEGA_MAX, w))
    k = om * (SK_TRACK / 2.0) / SK_RADIO
    vi, vd = vx - k, vx + k
    m = max(abs(vi), abs(vd))
    if m > SK_V_MAX:
        vi, vd = vi / m * SK_V_MAX, vd / m * SK_V_MAX

    def duty(x):
        f = min(1.0, abs(x) / SK_V_MAX)
        return min(100, int(round(f * 100 * boost))) if f > 0.02 else 0
    di, dd = duty(vi), duty(vd)
    return di, (1 if vi >= 0 else -1), dd, (1 if vd >= 0 else -1)


class MotoresSkidSteer:
    """Firmware original: duty por rueda -> ENCENDIDO/APAGADO por software (ciclo de 200 ms).
    Encendido = tension completa (+-100 %); apagado = rueda libre (ambos pines a LOW).
    Interfaz igual que MotoresFirmware: recibir(pv, pw, t) y tick(t) -> [R1, L1, L2, R2] en %."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.duty = [0] * 4            # dutyPedido por rueda
        self.sent = [0] * 4
        self.libre_hasta = [0.0] * 4
        self.completo = [False] * 4    # dcCompleto
        self.t_ciclo = 0.0             # cicloMotoraAt (global)
        self.refreshed = -1e9
        self.hist = [0] * SK_HIST
        self.hi = 0
        self.t_mon = 0.0
        self.boost = 1.0
        self.omega_gain = 1.0          # autoridad de giro (Hip.giro_gain); 1 = la del firmware
        # Variabilidad POR PULSO medida en el robot real (calibracion_real): giros «muertos», dispersión del giro y deriva en recta.
        self.rng = np.random.default_rng(0)
        self._en_pulso = False
        self._k_giro = 1.0
        self._off_pct = 0.0

    def _perturbar(self, pv, pw):
        """Devuelve el `pw` efectivo: al EMPEZAR cada pulso se sortea cuánto responde el giro y cuánta deriva lateral aparece."""
        mueve = abs(pv) > 0.5 or abs(pw) > 0.5
        if mueve and not self._en_pulso:
            r = self.rng
            self._k_giro = 0.0 if r.random() < CAL["giro_p_muerto"] else min(2.2, float(np.exp(r.normal(CAL.get("giro_mu", 0.0), CAL["giro_sigma"]))))
            sd = CAL["deriva_sd_grande"] if r.random() < CAL["deriva_p_grande"] else CAL["deriva_sd_chica"]
            self._off_pct = float(r.normal(0.0, sd))
        self._en_pulso = mueve
        if not mueve:
            return pw
        return pw + (self._off_pct if abs(pw) < 0.5 else 0.0)

    def _escala_giro(self, di, si, dd, sd):
        """Escala el DIFERENCIAL entre lados por k (giros muertos / flojos): se hace tras el mezclador porque la mezcla satura y escalar omega
        antes casi no cambia el giro resultante."""
        k = self._k_giro if self._en_pulso else 1.0
        if k == 1.0:
            return di, si, dd, sd
        Di, Dd = si * di, sd * dd
        m, d = (Di + Dd) / 2.0, (Dd - Di) / 2.0
        Di2, Dd2 = m - k * d, m + k * d
        return (int(round(abs(Di2))), 1 if Di2 >= 0 else -1, int(round(abs(Dd2))), 1 if Dd2 >= 0 else -1)

    def _poner(self, i, duty, sent, t):
        if duty == 0:
            sent = 0
        if sent != self.sent[i]:
            self.libre_hasta[i] = t + SK_DEAD_S
            self.sent[i] = sent
        self.duty[i] = duty

    def recibir(self, pv, pw, t):
        self.refreshed = t
        pw = self._perturbar(pv, pw)
        di, si, dd, sd = self._escala_giro(*sk_mix(pv * 0.01 * SK_V_MAX, pw * self.omega_gain * 0.01 * SK_OMEGA_MAX, self.boost))
        # orden del firmware [R1, L1, L2, R2]: R* = lado izquierdo del mix ("IZQ"), L* = derecho
        for i, (du, se) in enumerate(((di, si), (dd, sd), (dd, sd), (di, si))):
            self._poner(i, du, se, t)

    def tick(self, t):
        if t - self.refreshed > DEADMAN_FW_S:             # failsafe: sin ordenes -> todo apagado
            for i in range(4):
                self._poner(i, 0, 0, t)
        if t - self.t_mon >= SK_MON_S:                    # updateAntiHundimiento()
            self.t_mon = t
            self.hist[self.hi] = sum(self.duty) // 4
            self.hi = (self.hi + 1) % SK_HIST
            self.boost = SK_BOOST if sum(self.hist) / SK_HIST > SK_UMBRAL * 100 else 1.0
        out = []
        for i in range(4):
            if t < self.libre_hasta[i] or self.sent[i] == 0 or self.duty[i] <= 0:
                self.completo[i] = False
                out.append(0.0)
                continue
            if not self.completo[i]:
                self.completo[i] = True
                self.t_ciclo = t                          # arranca ciclo nuevo (como el .ino)
            if self.duty[i] >= 100:
                on = True
            else:
                on = ((t - self.t_ciclo) % SK_PERIODO_S) < SK_PERIODO_S * self.duty[i] / 100.0
            out.append(100.0 * self.sent[i] if on else 0.0)
        return out


class MotoresMaxFrente(MotoresSkidSteer):
    """firmware/agrobot_c3_skidsteer_maxfrente: como el skid-steer v3, pero las ruedas DELANTERAS (R1 y L1: el eje fijo
    estrecho, con los motores fuertes) van al 100 % de PWM, continuo, mientras su lado tenga mando. Las traseras
    (módulo del pivote) llevan el duty proporcional x `tras_factor`. El giro sale de invertir el sentido de un lado."""

    def recibir(self, pv, pw, t):
        self.refreshed = t
        pw = self._perturbar(pv, pw)
        di, si, dd, sd = self._escala_giro(*sk_mix(pv * 0.01 * SK_V_MAX, pw * self.omega_gain * 0.01 * SK_OMEGA_MAX, self.boost))
        # El lado más rápido va al 100 % y el otro CONSERVA la proporción (antes ambos iban al 100 % y toda curva era un
        # giro en el sitio: el robot no avanzaba mientras corregía).
        m = max(di, dd)
        fi, fd = (int(round(100 * di / m)), int(round(100 * dd / m))) if m > 0 else (0, 0)
        k = max(0.0, min(1.0, self.cfg.tras_factor))
        ti, td = int(round(di * k)), int(round(dd * k))
        # orden del firmware [R1, L1, L2, R2]: R1/L1 = delanteras; L2/R2 = traseras
        for i, (du, se) in enumerate(((fi, si), (fd, sd), (td, sd), (ti, si))):
            self._poner(i, du, se, t)


# ======================================================================
# Firmware "SUAVE" v4 (firmware/agrobot_c3_suave): port EXACTO de suave_model.h
# ======================================================================
SU_V_MAX, SU_OMEGA_MAX = 0.55, 1.5
SU_GAP_S = 0.025


def su_mezclar(v, w, adel, atras, pivote, minp, tras=130.0):
    """-> [d0..d3] duty firmado: 0=R1 del, 1=R2 tras, 2=L1 del, 3=L2 tras (antes de invertir)."""
    if not (math.isfinite(v) and math.isfinite(w)):
        return [0.0] * 4
    fv = max(-1.0, min(1.0, v / SU_V_MAX))
    wn = max(-1.0, min(1.0, w / SU_OMEGA_MAX))
    if abs(fv) < 0.02:
        fv = 0.0
    if abs(wn) < 0.02:
        wn = 0.0
    if fv == 0 and wn == 0:
        return [0.0] * 4
    base = min(fv * 100, adel) if fv > 0 else -min(-fv * 100, atras)
    base_r = max(-100.0, min(100.0, base * tras / 100.0))
    dif = wn * pivote
    izq, der = base_r - dif, base_r + dif
    m = max(abs(izq), abs(der))
    if m > 100:
        izq *= 100 / m
        der *= 100 / m
    d = [base, izq, base, der]
    for i in range(4):
        a = abs(d[i])
        if 0 < a < minp:
            d[i] = (1 if d[i] > 0 else -1) * minp
    return d


def su_rampa(cur, t, dt, minp, acel, frena):
    if t == 0 or (cur != 0 and ((t > 0) != (cur > 0))):
        st = frena * dt
        return 0.0 if abs(cur) <= st else cur - (st if cur > 0 else -st)
    if cur == 0:
        cur = (1 if t > 0 else -1) * min(abs(t), minp)
    if abs(t) > abs(cur):
        return cur + max(-acel * dt, min(acel * dt, t - cur))
    return cur + max(-frena * dt, min(frena * dt, t - cur))


class MotoresSuave:
    """Firmware suave: delanteras solo longitudinales, traseras con el giro, rampas.
    Interfaz igual que los otros: recibir(pv, pw, t) y tick(t) -> [R1, L1, L2, R2] en % (orden del sim)."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.t = [0.0] * 4              # objetivos en indices suave 0..3
        self.cur = [0.0] * 4
        self.zero_at = [-1e9] * 4
        self.refreshed = -1e9

    def recibir(self, pv, pw, t):
        self.refreshed = t
        c = self.cfg
        self.t = su_mezclar(pv * 0.01 * SU_V_MAX, pw * 0.01 * SU_OMEGA_MAX, c.su_adel, c.su_atras, c.su_pivote, c.su_minp, c.su_tras)

    def tick(self, t):
        c = self.cfg
        if t - self.refreshed > DEADMAN_FW_S:            # deadman: ruedas libres al instante
            self.cur = [0.0] * 4
            self.t = [0.0] * 4
        else:
            for i in range(4):
                tgt = self.t[i]
                if self.cur[i] == 0 and tgt != 0 and (t - self.zero_at[i]) < SU_GAP_S:
                    tgt = 0.0
                prev = self.cur[i]
                self.cur[i] = su_rampa(prev, tgt, TICK_FW, c.su_minp, c.su_acel, c.su_frena)
                if self.cur[i] == 0 and prev != 0:
                    self.zero_at[i] = t
        # El firmware satura el duty a 100 % al escribir el PWM (fminf(|cur|,100)): con limites
        # fuera de rango (pivote/minp/tras de fabrica > 100) la mezcla puede pedir 150.
        d = [max(-100.0, min(100.0, x)) for x in self.cur]
        return [d[0], d[2], d[3], d[1]]                    # -> [R1, L1, L2, R2]


# ======================================================================
# Firmware FUSION: port EXACTO de fusion_model.h (tests/test_fusion_firmware.py lo comprueba)
# ======================================================================
def fu_mezclar(v, w, adel, atras, pivote, minp, tras, dif_del, boost=1.0):
    if not (math.isfinite(v) and math.isfinite(w)):
        return [0.0] * 4
    fv = max(-1.0, min(1.0, v / SU_V_MAX))
    wn = max(-1.0, min(1.0, w / SU_OMEGA_MAX))
    if abs(fv) < 0.02:
        fv = 0.0
    if abs(wn) < 0.02:
        wn = 0.0
    if fv == 0 and wn == 0:
        return [0.0] * 4
    base = min(fv * 100, adel) if fv > 0 else -min(-fv * 100, atras)
    base_r = max(-100.0, min(100.0, base * tras / 100.0))
    dif = wn * pivote
    kd = dif_del / 100.0
    izq, der = base_r - dif, base_r + dif
    fi, fd = base - kd * dif, base + kd * dif
    m = max(abs(izq), abs(der), abs(fi), abs(fd))
    if m > 100:
        k = 100 / m
        izq, der, fi, fd = izq * k, der * k, fi * k, fd * k
    d = [fi, izq, fd, der]
    for i in range(4):
        d[i] = max(-100.0, min(100.0, d[i] * boost))
        if abs(d[i]) < 0.5:
            d[i] = 0.0
        a = abs(d[i])
        if 0 < a < minp:
            d[i] = (1 if d[i] > 0 else -1) * minp
    return d


def fu_escala_giro(w, d, comp_neg, comp_pos):
    """Port de fusion::escala_giro: fracción de ciclo ENCENDIDA de cada rueda (1 = sin recorte) en un giro."""
    wn = max(-1.0, min(1.0, w / SU_OMEGA_MAX))
    c = 0.0 if abs(wn) < 0.02 else (comp_neg if wn < 0 else comp_pos)
    return [1.0 - c / 100.0 if (c > 0 and x < 0) else 1.0 for x in d]


def fu_rebanada(salida, t_ms, esc):
    if esc >= 1.0 or salida == 0:
        return salida
    return salida if (t_ms % 100) < 100.0 * esc else 0.0


def fu_rafaga(env, t_ms, periodo):
    if periodo <= 0:
        return env
    a = abs(env)
    if a <= 0:
        return 0.0
    if a >= 98:
        return 100.0 if env > 0 else -100.0
    fase = t_ms % int(periodo)
    return (100.0 if env > 0 else -100.0) if fase < periodo * a / 100.0 else 0.0


class MotoresFusion:
    """Fusion v3 + suave. Interfaz: recibir(pv, pw, t) y tick(t) -> [R1, L1, L2, R2] en %."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.t = [0.0] * 4
        self.cur = [0.0] * 4
        self.zero_at = [-1e9] * 4
        self.refreshed = -1e9
        self.hist = [0.0] * 10
        self.hi = 0
        self.boost = 1.0
        self.t_mon = 0.0
        self.esc = [1.0] * 4
        self.cero_desde = [None] * 4
        self.frenado = [False] * 4
        self.freno_hasta = [-1.0] * 4
        self.frenando = [False] * 4        # en el orden de salida [R1, L1, L2, R2]

    def recibir(self, pv, pw, t):
        self.refreshed = t
        c = self.cfg
        w = pw * 0.01 * SU_OMEGA_MAX
        self.t = fu_mezclar(pv * 0.01 * SU_V_MAX, w, c.fu_adel, c.fu_atras,
                            c.fu_pivote, c.fu_minp, c.fu_tras, c.fu_dif_del, self.boost)
        self.esc = fu_escala_giro(w, self.t, c.fu_comp_neg, c.fu_comp_pos)

    def _freno(self, i, t, inmediato, c):
        """Port del freno activo de motorTask(): frena tras 200 ms seguidos con orden cero (no en cada hueco del troceo de la Jetson)
        o al instante en failsafe; dura fu_freno_ms. Devuelve True mientras frena la rueda i."""
        if t < self.freno_hasta[i]:
            return self.t[i] == 0.0 or inmediato
        if self.freno_hasta[i] > 0:
            self.freno_hasta[i] = -1.0
        if c.fu_freno_ms <= 0 or self.frenado[i]:
            return False
        if inmediato and self.cur_previo[i] != 0.0:
            self.freno_hasta[i], self.frenado[i] = t + c.fu_freno_ms / 1000.0, True
            return True
        if self.t[i] == 0.0 and self.cero_desde[i] is not None and t - self.cero_desde[i] >= 0.2:
            self.freno_hasta[i], self.frenado[i] = t + c.fu_freno_ms / 1000.0, True
            return True
        return False

    def tick(self, t):
        c = self.cfg
        self.cur_previo = list(self.cur)
        inmediato = t - self.refreshed > DEADMAN_FW_S
        if inmediato:
            self.cur = [0.0] * 4
            self.t = [0.0] * 4
        else:
            if c.fu_refuerzo > 100 and t - self.t_mon >= 0.1:        # anti-hundimiento (cada 100 ms)
                self.t_mon = t
                self.hist[self.hi] = sum(abs(x) for x in self.t) / 4.0
                self.hi = (self.hi + 1) % 10
                self.boost = c.fu_refuerzo / 100.0 if sum(self.hist) / 10.0 > 85.0 else 1.0
            for i in range(4):
                tgt = self.t[i]
                if self.cur[i] == 0 and tgt != 0 and (t - self.zero_at[i]) < SU_GAP_S:
                    tgt = 0.0
                prev = self.cur[i]
                self.cur[i] = su_rampa(prev, tgt, TICK_FW, c.fu_minp, c.fu_acel, c.fu_frena)
                if self.cur[i] == 0 and prev != 0:
                    self.zero_at[i] = t
        for i in range(4):                                     # seguimiento del «orden cero sostenido» para el freno
            if self.t[i] != 0.0:
                self.cero_desde[i], self.frenado[i] = None, False
            elif self.cero_desde[i] is None:
                self.cero_desde[i] = t
        fr = [self._freno(i, t, inmediato, c) for i in range(4)]
        t_ms = int(round(t * 1000))
        d = [0.0 if f else fu_rebanada(fu_rafaga(x, t_ms, c.fu_periodo), t_ms, e) for x, e, f in zip(self.cur, self.esc, fr)]
        self.frenando = [fr[0], fr[2], fr[3], fr[1]]
        return [d[0], d[2], d[3], d[1]]


def crear_firmware(cfg: Config):
    if cfg.firmware == "fusion":
        return MotoresFusion(cfg)
    if cfg.firmware == "maxfrente":
        return MotoresMaxFrente(cfg)
    if cfg.firmware == "suave_v4":
        return MotoresSuave(cfg)
    return MotoresSkidSteer(cfg) if cfg.firmware == "skidsteer_v3" else _Firmware12(cfg)


class _Firmware12(MotoresFirmware):
    def recibir(self, pv, pw, t):
        self.set_targets(fw_mix(pv * 0.01 * V_SCALE, pw * 0.01 * W_SCALE, self.cfg), t)


# ======================================================================
# Jetson: rampa + conductor de pulsos (port)
# ======================================================================
def demo_ref_tope(v, w, tope_duty):
    """manual_demo.reference() con el tope de avance parametrizable (60 = idéntico)."""
    vn = max(-1.0, min(1.0, v / V_SCALE))
    wn = max(-1.0, min(1.0, w / W_SCALE))
    if abs(vn) < 0.004:
        return 0.0, 0.0
    vv = max(-0.35, min(tope_duty / 100.0, vn))
    return vv * V_SCALE, (0.0 if vv < 0 else wn * W_SCALE)


class ActuadorJetson:
    """ActuatorProtocol: send_command / tick / emergency_stop con la rampa real."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.obj_v = self.obj_w = 0.0
        self.ramp_v = self.ramp_w = 0.0
        self.last_t = 0.0
        self.obj_t = -1e9
        self.out_v = self.out_w = 0.0            # lo que sale hacia el enlace

    def _aplicar(self, t):
        dt = max(0.0, min(t - self.last_t, RAMP_MAX_DT))
        self.last_t = t
        dv = max(-self.cfg.accel_lin * dt, min(self.cfg.accel_lin * dt, self.obj_v - self.ramp_v))
        dw = max(-self.cfg.accel_ang * dt, min(self.cfg.accel_ang * dt, self.obj_w - self.ramp_w))
        self.ramp_v += dv
        self.ramp_w += dw
        try:
            if self.cfg.tope_duty == 60.0:
                v, w = demo_reference(self.ramp_v, self.ramp_w, pivote=self.cfg.pivote_en_sitio)   # el REAL
            else:
                v, w = demo_ref_tope(self.ramp_v, self.ramp_w, self.cfg.tope_duty)
        except (ValueError, TypeError):
            v = w = 0.0
        self.out_v, self.out_w = v, (-w if self.cfg.invertir_giro else w)      # borde del actuador: jetson_server._giro_signo()

    def send_command(self, v, w, t):
        self.obj_v, self.obj_w, self.obj_t = v, w, t
        self._aplicar(t)

    def tick(self, t):
        if t - self.obj_t > DEADMAN_JETSON_S:
            self.obj_v = self.obj_w = 0.0
        en_reposo = (self.ramp_v == 0 and self.ramp_w == 0 and self.obj_v == 0 and self.obj_w == 0)
        if not en_reposo:
            self._aplicar(t)

    def emergency_stop(self, t):
        self.ramp_v = self.ramp_w = self.obj_v = self.obj_w = 0.0
        self.last_t = t
        self.out_v = self.out_w = 0.0


class Conductor:
    """pulse_drive.ConductorPulsos en tiempo simulado, usando su `segmentos()` REAL."""

    def __init__(self, cfg: Config, act: ActuadorJetson):
        self.cfg = cfg
        self.act = act
        self._real = pulse_drive.ConductorPulsos(send_command=lambda *a: None,
                                                 emergency_stop=lambda: None)
        self._real.modo = cfg.modo
        self._gen = None
        self.clock = 0.0
        self.aceptadas = 0
        self.descartadas = 0
        self.ultima_accion = None

    @property
    def activo(self) -> bool:
        return self._gen is not None

    def ordenar(self, accion, t, maniobra=True, esc_v=1.0, esc_dur=1.0, w_extra=0.0) -> bool:
        v_max = self.cfg.max_v_mps * self.cfg.escala_velocidad * esc_v
        w_max = self.cfg.max_omega_rps * self.cfg.escala_velocidad
        segs = self._real.segmentos(accion, v_max, w_max, maniobra)
        if esc_dur != 1.0:
            segs = [(v, w, d * esc_dur) for (v, w, d) in segs]
        if w_extra and accion == "adelante":
            segs = [(v, w + w_extra, d) for (v, w, d) in segs]
        if self.cfg.tope_duty != 60.0:            # ¿y si? subir el tope de avance
            vf = pulse_drive.MODOS[self.cfg.modo]["vel_factor"]
            tope_v = self.cfg.tope_duty / 100.0 * V_SCALE
            segs = [((min(v_max * vf, tope_v) if v > 0 else v), w, d) for (v, w, d) in segs]
        if self.activo:                           # Codex: `if self.activo: return False`
            self.descartadas += 1
            return False
        self._gen = self._ejecutar(segs)
        self.ultima_accion = accion
        self.aceptadas += 1
        self.paso(t)                              # asyncio.create_task: arranca ya
        return True

    def _tramo(self, v, w, dur):
        duty = min(1.0, max(0.15, self.cfg.suavidad_duty))
        periodo = max(0.1, self.cfg.suavidad_periodo_s)
        fin = self.clock + dur
        while self.clock < fin:
            if duty >= 0.99:
                yield (v, w)
                continue
            fin_on = min(fin, self.clock + periodo * duty)
            while self.clock < fin_on:
                yield (v, w)
            fin_off = min(fin, self.clock + periodo * (1.0 - duty))
            while self.clock < fin_off:
                yield (0.0, 0.0)

    def _ejecutar(self, segs):
        for v, w, dur in segs:
            yield from self._tramo(v, w, dur)
        yield None                                # fin: emergency_stop

    def paso(self, t):
        """Una vuelta del bucle de 0,05 s del conductor."""
        if self._gen is None:
            return
        self.clock = t
        try:
            cmd = next(self._gen)
        except StopIteration:
            cmd = None
        if cmd is None:
            self._gen = None
            self.act.emergency_stop(t)
        else:
            self.act.send_command(cmd[0], cmd[1], t)


# ======================================================================
# Mundo, LIDAR y dinámica con el pivote trasero
# ======================================================================
@dataclass
class Escenario:
    nombre: str
    paredes: list              # [(x0,y0,x1,y1)] en el mundo REAL
    macetas: list              # [(x,y,r)] en el mundo REAL
    goal_x: float
    centro_y: float = 0.0
    inicio: tuple = (0.0, 0.0, 0.0)   # x, y, rumbo real
    t_max: float = 100.0
    descripcion: str = ""
    meta_xy: tuple | None = None       # si no es solo avanzar en x
    linea_central: list | None = None  # polilinea [(x,y)...]; None = la recta y=centro_y
    plantas: list | None = None        # [(x, y)] plantas a inspeccionar y contar en una MISIÓN (mision_sim.py); None = sin misión

    def desvio(self, x, y):
        """Distancia con signo a la línea central (m)."""
        if not self.linea_central:
            return y - self.centro_y
        best = math.inf
        for (ax, ay), (bx, by) in zip(self.linea_central, self.linea_central[1:]):
            ex, ey = bx - ax, by - ay
            L2 = ex * ex + ey * ey
            u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * ex + (y - ay) * ey) / L2))
            best = min(best, math.hypot(x - (ax + u * ex), y - (ay + u * ey)))
        return best


class MundoSim:
    """Interfaz que espera huella_pasiva.choca (paredes + objetos sólidos)."""

    def __init__(self, esc: Escenario | None = None):
        self.paredes = list(esc.paredes) if esc else []
        # (x, y, r, altura, solido): solo los sólidos cuentan
        self.objetos = [(x, y, r, 0.25, 1.0) for (x, y, r) in esc.macetas] if esc else []

    @staticmethod
    def cercano(mundo: "MundoSim", cx, cy, radio) -> "MundoSim":
        """Subconjunto de paredes/objetos a menos de `radio` del punto."""
        m = MundoSim()
        for (x0, y0, x1, y1) in mundo.paredes:
            ex, ey = x1 - x0, y1 - y0
            L2 = ex * ex + ey * ey
            u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((cx - x0) * ex + (cy - y0) * ey) / L2))
            if math.hypot(cx - (x0 + u * ex), cy - (y0 + u * ey)) < radio:
                m.paredes.append((x0, y0, x1, y1))
        m.objetos = [o for o in mundo.objetos if math.hypot(cx - o[0], cy - o[1]) < radio + o[2]]
        return m


def lidar_scan(esc: Escenario, cx, cy, psi_real, rng, ruido=None, rango=8.0):
    """360 rayos desde (cx, cy) = posición del LIDAR. 0° = adelante, + = izquierda (como ROS).
    Las paredes se ven siempre; las macetas solo si sobresalen del plano del LIDAR (ALTURA_LIDAR_M)."""
    ang = np.deg2rad(np.arange(360)) + psi_real
    dx, dy = np.cos(ang), np.sin(ang)
    dist = np.full(360, np.inf)
    macetas_visibles = esc.macetas if ALTURA_MACETA_M >= ALTURA_LIDAR_M - GROSOR_HAZ_M else []
    if macetas_visibles:                          # círculos, todos a la vez: (360 rayos x M círculos)
        M_ = np.asarray(macetas_visibles, dtype=float)
        fx, fy = cx - M_[:, 0][None, :], cy - M_[:, 1][None, :]
        b = fx * dx[:, None] + fy * dy[:, None]
        c = fx * fx + fy * fy - (M_[:, 2] ** 2)[None, :]
        disc = b * b - c
        ok = disc >= 0
        t = np.where(ok, -b - np.sqrt(np.where(ok, disc, 0.0)), np.inf)
        t = np.where(t > 0.02, t, np.inf)
        dist = np.minimum(dist, t.min(axis=1))
    if len(esc.paredes):                          # segmentos, todos a la vez: (360 rayos x W paredes)
        W_ = np.asarray(esc.paredes, dtype=float)
        x0_, y0_, ex_, ey_ = W_[:, 0][None, :], W_[:, 1][None, :], (W_[:, 2] - W_[:, 0])[None, :], (W_[:, 3] - W_[:, 1])[None, :]
        den = dx[:, None] * ey_ - dy[:, None] * ex_
        with np.errstate(divide="ignore", invalid="ignore"):
            tt = ((x0_ - cx) * ey_ - (y0_ - cy) * ex_) / den
            uu = ((x0_ - cx) * dy[:, None] - (y0_ - cy) * dx[:, None]) / den
        ok = (np.abs(den) > 1e-9) & (tt > 0.02) & (uu >= 0) & (uu <= 1)
        dist = np.minimum(dist, np.where(ok, tt, np.inf).min(axis=1))
    if ruido is None:                      # σ(r) = a + b·r medida con el robot PARADO, por el factor de marcha (hipótesis), y rayos perdidos
        sigma = (CAL["lidar_sigma_a"] + CAL["lidar_sigma_b"] * np.where(np.isfinite(dist), dist, 0.0)) * CAL["lidar_factor_marcha"]
        ruido = sigma
        if CAL["lidar_dropout"] > 0:
            dist = np.where(rng.random(360) < CAL["lidar_dropout"], np.inf, dist)
    for sec in CAL.get("lidar_chasis", []):       # el LIDAR ve su propio chasis (medido en el robot real): puntos fijos que viajan con él
        for a in range(int(sec["desde"]), int(sec["hasta"])):
            i = a % 360
            if rng.random() < sec["prob"]:
                dist[i] = min(dist[i], sec["dist_m"])
    dist = np.where(dist <= rango, dist + rng.normal(0, 1.0, 360) * ruido, np.inf)
    return [{"angle_deg": float(a), "distance_m": float(d)}
            for a, d in zip(range(360), dist) if np.isfinite(d) and d > 0.05]


class RobotModelo:
    """Envuelve DinamicaPasiva con el marco girado 180° (pivote trasero)."""

    def __init__(self, params, pose_real):
        x, y, psi = pose_real                       # CENTRO y rumbo reales
        self.p = params
        self.giro = math.pi if PIVOTE == "trasero" else 0.0
        psi_m = psi + self.giro
        # origen del modelo = eje ancho fijo: centro + MID hacia delante real (pivote atrás)
        # o centro - MID (pivote delante: el eje ancho queda atrás)
        k = 1.0 if PIVOTE == "trasero" else -1.0
        ox, oy = x + k * MID * math.cos(psi), y + k * MID * math.sin(psi)
        self.d = DinamicaPasiva(pose=(ox, oy, psi_m), parametros=params)
        self.geo = replace(H.geo_de_parametros(params), cuerpo_largo=CUERPO_L, cuerpo_ancho=CUERPO_A)
        self.efic = None                            # [(fwd, rev)] por rueda en el orden [R1, L1, L2, R2]; None = ideal

    def centro(self):
        ox, oy, psi_m, delta = self.d.q
        return ox + MID * math.cos(psi_m), oy + MID * math.sin(psi_m)

    def lidar_pos(self):
        """Posición del LIDAR: LIDAR_DESDE_EJE_M del eje ancho hacia el módulo del pivote (no en el centro)."""
        ox, oy, psi_m, _ = self.d.q
        return ox + LIDAR_DESDE_EJE_M * math.cos(psi_m), oy + LIDAR_DESDE_EJE_M * math.sin(psi_m)

    def psi_real(self):
        return math.atan2(math.sin(self.d.q[2] + self.giro), math.cos(self.d.q[2] + self.giro))

    def delta(self):
        return float(self.d.q[3])

    def step(self, dt, duties, dead, frenos=None):
        """duties firmados [R1,L1,L2,R2] en %  ->  velocidades de rueda del modelo. `frenos`: [bool] por rueda con el freno activo
        del firmware (el puente en corto): la rueda NO queda libre, su objetivo de velocidad es 0 y el motor la retiene."""
        if self.efic:
            duties = [d * (e[0] if d >= 0 else e[1]) for d, e in zip(duties, self.efic)]
        vel, libre = [], []
        for dpc in duties:
            # Duty 0 (o por debajo de la zona muerta) = rueda LIBRE, no frenada: el firmware
            # hace detachMotor() y el puente queda en alta impedancia.
            es_libre = abs(dpc) < max(dead, 1e-6)
            libre.append(es_libre)
            sg = -1.0 if PIVOTE == "trasero" else 1.0   # pivote atrás: avanzar real = retroceder en el modelo
            vel.append(0.0 if es_libre else sg * (dpc / 100.0 * V_SCALE))
        if frenos:
            libre = [l and not f for l, f in zip(libre, frenos)]      # frenada = NO libre con objetivo 0
        R1, L1, L2, R2 = vel
        fR1, fL1, fL2, fR2 = libre
        if PIVOTE == "trasero":
            self.d.step(dt, 0.0, 0.0, wheel_targets=[R1, L1, R2, L2],   # [trasL,trasR,delL,delR]
                        libres=[fR1, fL1, fR2, fL2])
        else:   # ruedas delanteras reales (R1,L1) = módulo del pivote = delL,delR del modelo
            self.d.step(dt, 0.0, 0.0, wheel_targets=[L2, R2, L1, R1],
                        libres=[fL2, fR2, fL1, fR1])


# ======================================================================
# Episodio
# ======================================================================
@dataclass
class Hip:
    """Hipótesis NO medidas que se aleatorizan entre episodios."""
    mu: float = 0.8
    dead: float = 0.0          # duty (%) por debajo del cual el motor no vence la fricción
    bias: float = 1.0          # fuerza relativa del módulo trasero (motores iguales = 1)
    semilla_chasis: int = 0
    # Rozamiento viscoso de rodadura por rueda (N·m por rad/s). Calibrado el 2026-10-16 contra
    # el ÚNICO dato real: 26,2 ± 2,2 cm por pulso (n=4, firmware antiguo, escala .4, ~87 % de duty,
    # 1 s). Con ese pulso equivalente el ajuste da ~0,17 (0,15 -> 28 cm, 0,20 -> 24 cm). El
    # valor anterior (0,02) sale de calibrar con el modelo de firmware 1.2 y rodaba 82 cm por
    # pulso con el skid-steer v3. Incertidumbre: duty y duración exactos del pulso viejo.
    drag: float = 0.17
    # Robot PESADO / delanteras flojas (montecarlo_firmware_pesado.py). None = el modelo por defecto.
    masa: float | None = None          # chassis_mass en kg (la medida: ~12 kg en total)
    # Asimetrías MEDIDAS el 2026-10-23 (calibracion_pulsos_20261023.json): hacia atrás el robot rueda más que hacia delante y un lado
    # empuja más que el otro. Multiplican el duty de cada rueda: avance (+), retroceso (-) y lado (der = R1,R2; izq = L1,L2).
    efic_fwd: float = 1.0
    efic_rev: float = 1.0
    lateral: float | None = None       # rigidez lateral del neumático (menor = el robot gira más fácil, patinando)
    pivot_fric: float | None = None    # rozamiento del pivote delantero pasivo
    lado_der: float = 1.0
    lado_izq: float = 1.0
    eje_fijo: float | None = None      # ancho exterior del eje FIJO delantero (estrecho); None = FIJO_EXT_M
    # Rigidez lateral de los neumáticos (N por m/s): lo que se resiste al giro de un skid-steer. El 2026-10-21 el
    # operador dijo que el robot REAL gira bien; con 65 el simulador gira 13 °/s en el sitio (muy poco), con 15 gira ~40.
    # Sin medida real: se sortea (verificar_giro.py da el pico de gyroZ para ajustarlo).
    lateral: float = 65.0
    # Autoridad de giro del robot real: el operador midió el 2026-10-21 ~10° y ~10 cm por toque de giro («como estacionar
    # un auto»: adelante-atrás-giro). El simulador con autoridad plena giraba en el sitio (0,3 cm, 2–9°). Se multiplica el
    # omega que llega al mezclador del firmware; el controlador aprende a pedir más.
    giro_gain: float = 1.0


LATERAL_OPCIONES = (9.0, 7.0, 5.5)     # 2026-10-21: giro ~15° por toque a cada lado (antes ~10°)
GIRO_GAIN_OPCIONES = (0.60, 0.45, 0.30)


def hip_por_semilla(semilla: int) -> Hip:
    rng = np.random.default_rng(777 + semilla)
    return Hip(mu=float(rng.choice([0.65, 0.8, 0.95])),
               dead=float(rng.choice([0.0, 25.0, 40.0])),
               bias=float(rng.choice(BIAS_OPCIONES)), semilla_chasis=int(semilla),
               drag=float(rng.choice([0.13, 0.17, 0.22])),
               # AL FINAL: no altera los demás sorteos de cada semilla. Con el croquis el ancho del eje fijo es
               # dudoso (croquis 28 cm, medida anterior 19 cm): se entrena con los tres.
               eje_fijo=float(rng.choice(FIJO_OPCIONES)),
               lateral=float(rng.choice(LATERAL_OPCIONES)),
               giro_gain=float(rng.choice(GIRO_GAIN_OPCIONES)))


def episodio(esc: Escenario, cfg: Config, semilla: int, hip: Hip | None = None,
             traza: bool = False) -> dict:
    hip = hip or hip_por_semilla(semilla)
    rng = np.random.default_rng(10_000 + semilla)
    params = replace(parametros_rusticos(hip.semilla_chasis), mu=hip.mu,
                     front_drive_bias=hip.bias, rolling_drag=hip.drag, substep=SUBSTEP, lateral=hip.lateral,
                     front_outer=PIVOTE_EXT_M, rear_outer=hip.eje_fijo or FIJO_EXT_M)
    if hip.masa is not None:
        params = replace(params, chassis_mass=hip.masa)
    if hip.lateral is not None:
        params = replace(params, lateral=hip.lateral)
    if hip.pivot_fric is not None:
        params = replace(params, pivot_friction=hip.pivot_fric)
    # perturbación inicial: el robot nunca arranca perfectamente centrado
    x0, y0, psi0 = esc.inicio
    y0 += rng.uniform(-0.04, 0.04)
    psi0 += math.radians(rng.uniform(-3.0, 3.0))
    robot = RobotModelo(params, (x0, y0, psi0))
    if (hip.efic_fwd, hip.efic_rev, hip.lado_der, hip.lado_izq) != (1.0, 1.0, 1.0, 1.0):
        lado = (hip.lado_der, hip.lado_izq, hip.lado_izq, hip.lado_der)           # [R1, L1, L2, R2]
        robot.efic = [(hip.efic_fwd * k, hip.efic_rev * k) for k in lado]
    mundo = MundoSim(esc)

    act = ActuadorJetson(cfg)
    cond = Conductor(cfg, act)
    fw = crear_firmware(cfg)
    fw.omega_gain = hip.giro_gain
    if hasattr(fw, "rng"):
        fw.rng = np.random.default_rng(20_000 + semilla)           # variabilidad por pulso reproducible por semilla
    lidar = LidarReactive()
    cola_enlace = []                               # (t_llegada, pct_v, pct_w)
    ctrl = None
    prox_cc = prox_scan_cc = 0.0
    if cfg.control == "continuo":
        from control_pasillo import PasilloController
        ctrl = PasilloController(v_cruise=cfg.cc_v, kp=cfg.cc_kp, kd=cfg.cc_kd, w_max=cfg.cc_wmax,
                                 nominal=cfg.cc_nominal, d_stop=cfg.cc_dstop, d_slow=cfg.cc_dslow,
                                 kh=cfg.cc_kh, kl=cfg.cc_kl)
    d_cc = {}

    t = 0.0
    dt = TICK_FW
    prox_expl, prox_cond, prox_jet = TICK_EXPLORADOR, 0.0, 0.0
    sin_corregir = 0
    cola_man, pausa_hasta, ultimo_giro, escapes = [], 0.0, None, 0
    rf = RF.RumboFusion() if cfg.rumbo_zm_deg > 0 else None
    corr_rumbo = RF.CorrectorRumbo(cfg.rumbo_zm_deg, aprende=cfg.rumbo_aprende) if rf else None
    sesgo_gyro = float(rng.normal(0.0, math.radians(CAL["imu_gyro_sesgo_dps"]) / 2)) if rf else 0.0
    sigma_gyro = math.radians(CAL["imu_gyro_sigma_dps"])
    pulso_recto_pend, rumbo_corr = False, 0
    desvio_ult = None
    prog_ref, prog_t = (x0, y0), 0.0
    correcciones = corr_ejec = bloqueos = avances = 0
    contactos, prev_choque = 0, False
    ultima_ok, t_choque = None, -9.0
    contactos_por_accion = {}
    contacto_info = []
    guardian, g_libre_desde, g_reg = None, None, []
    mx_hist, mx_commit, mx_commit_hasta, mx_rectos, mx_escapes = [], None, 0.0, 0, 0
    visitas = {}
    mx_meta_xy = esc.meta_xy if esc.meta_xy is not None else (esc.goal_x, esc.centro_y)
    mision = None
    if cfg.mision and esc.plantas:
        import mision_sim
        mision = mision_sim.MisionSim(esc, cfg, np.random.default_rng(777 + semilla))
    if cfg.guardian:
        import guardian_sendero as GS
        guardian = cfg.guardian_obj or GS.GuardianSendero(GS.MemoriaCorrecciones(None), log_dir=None, tol_cm=cfg.guardian_tol_cm,
                                                           tol_deg=cfg.guardian_tol_deg, usar_memoria=cfg.guardian_usar_memoria)
        guardian.explorar = cfg.guardian_explorar
        guardian.usar_memoria = cfg.guardian_usar_memoria        # manda la config del episodio, no lo que traiga el objeto
        guardian.pend = None
        if esc.linea_central:
            guardian.fijar_polilinea(esc.linea_central)
        else:
            guardian.fijar_polilinea([(x0 - 1.0, esc.centro_y), (x0 + 400.0, esc.centro_y)])

    scan_xy = (np.zeros(0), np.zeros(0))
    PRED = {"adelante": 0, "atras": 0, "giro": 0, "cambiadas": 0}
    FRENTE_LIDAR, ATRAS_LIDAR, MEDIO_ANCHO, CENTRO_X = 0.185, 0.305, 0.19, -0.06   # huella en ejes del LIDAR (x adelante); centro del cuerpo a -6 cm

    def _despejado(accion, scan_):
        """¿Se puede ejecutar este pulso sin tocar nada de lo que ve el scan? Barre la huella (ejes del LIDAR). Vectorizado con numpy."""
        m = cfg.mx_pred_margen_m
        px_, py_ = scan_xy
        if px_.size == 0:
            return True
        chasis = (px_ >= -ATRAS_LIDAR - 0.01) & (px_ <= FRENTE_LIDAR + 0.01) & (np.abs(py_) <= MEDIO_ANCHO + 0.01)       # el propio chasis
        if accion == "adelante":
            hit = (px_ > FRENTE_LIDAR) & (px_ < FRENTE_LIDAR + cfg.mx_pred_avance_m + m) & (np.abs(py_) < MEDIO_ANCHO + m)
        elif accion == "atras":
            hit = (px_ > -ATRAS_LIDAR - cfg.mx_pred_atras_m - m) & (px_ < -ATRAS_LIDAR) & (np.abs(py_) < MEDIO_ANCHO + m)
        else:                                                         # giro: la esquina más lejana barre un círculo alrededor del centro
            hit = np.hypot(px_ - CENTRO_X, py_) < cfg.mx_pred_giro_m + m
        return not bool(np.any(hit & ~chasis))

    def _ord(accion, tt, **kw):
        """cond.ordenar con el predictor: si el pulso chocaría, no se ejecuta (o se cambia por el giro contrario / retroceso despejado)."""
        if not cfg.mx_prediccion or accion not in ("adelante", "atras", "izquierda", "derecha"):
            return cond.ordenar(accion, tt, **kw)
        clave = "adelante" if accion == "adelante" else "atras" if accion == "atras" else "giro"
        if _despejado(clave, scan):
            return cond.ordenar(accion, tt, **kw)
        PRED[clave] += 1
        if clave == "giro":
            for alt in (("derecha" if accion == "izquierda" else "izquierda"), "atras"):
                if _despejado("giro" if alt != "atras" else "atras", scan):
                    PRED["cambiadas"] += 1
                    return cond.ordenar(alt, tt, **kw)
        return False

    def _clamp_(v, lo, hi):
        return max(lo, min(hi, v))

    def _mejor_hueco(scan_, necesita, peso_meta, full=False):
        """Ángulo (rad, + = izquierda) hacia el centro del hueco libre más conveniente, o None. Un rumbo es libre si el rayo llega a `necesita` m;
        se erosiona ±12° (media anchura del robot a 1 m) para que el hueco quepa. Puntúa: cercanía a la meta (peso `peso_meta`) y anchura."""
        ab = 180 if full else 90                                    # ±90° (hacia delante) o ±180° (para dar la vuelta al escapar de un nicho)
        n_ = 2 * ab + 1
        r = np.full(n_, np.inf)
        for q in scan_:
            a = int(round(q["angle_deg"]))
            a = a - 360 if a > 180 else a
            if -ab <= a <= ab:
                r[a + ab] = min(r[a + ab], q["distance_m"])
        libre = (r >= necesita).astype(np.uint8)
        k = 12
        er = np.array([libre[max(0, i - k):i + k + 1].min() if i - k >= 0 and i + k <= n_ - 1 else 0 for i in range(n_)])
        mejor, mejor_p = None, -1e9
        i = 0
        meta = _rumbo_meta()
        mx0, my0 = robot.centro()
        psi0 = robot.psi_real()
        while i <= n_ - 1:
            if er[i]:
                j = i
                while j + 1 <= n_ - 1 and er[j + 1]:
                    j += 1
                c = math.radians((i + j) / 2 - ab)
                pt = -peso_meta * abs(c - meta) + 0.01 * (j - i)
                if cfg.mx_memoria_w > 0:                           # prefiere lo menos pisado: visitas cerca de 1,2 m en esa dirección
                    vx, vy = mx0 + 1.2 * math.cos(psi0 + c), my0 + 1.2 * math.sin(psi0 + c)
                    pt -= cfg.mx_memoria_w * visitas.get((round(vx / 0.5), round(vy / 0.5)), 0) / 20.0
                if pt > mejor_p:
                    mejor, mejor_p = c, pt
                i = j + 1
            else:
                i += 1
        return mejor

    def _rumbo_meta():
        """Error de rumbo hacia la meta (rad, + = la meta queda a la izquierda), con la pose tipo SLAM (1° de ruido)."""
        gx, gy = robot.centro()
        tgt = mx_meta_xy
        if cfg.mx_visitar and mision is not None:
            tgt = mision.objetivo(t, gx, gy, cfg.mx_visitar_m) or mx_meta_xy        # primero las plantas vistas y pendientes, luego la meta
        b = math.atan2(tgt[1] - gy, tgt[0] - gx) - (robot.psi_real() + math.radians(float(rng.normal(0, 1.0))))
        return math.atan2(math.sin(b), math.cos(b))

    def _atrapado(tt, gx, gy, nb):
        """Dando vueltas en un nicho: en `mx_ventana_s` s no se alejó 0,45 m de donde estaba y se bloqueó >= 8 veces."""
        mx_hist.append((tt, gx, gy, nb))
        while mx_hist and tt - mx_hist[0][0] > cfg.mx_ventana_s:
            mx_hist.pop(0)
        if len(mx_hist) < 2 or tt - mx_hist[0][0] < 0.9 * cfg.mx_ventana_s:
            return False
        return max(math.hypot(gx - h[1], gy - h[2]) for h in mx_hist) < 0.45 and nb - mx_hist[0][3] >= 8

    def _pose_g():
        """Pose estimada tipo SLAM: la verdadera con ruido (2 cm, 1°)."""
        gx, gy = robot.centro()
        return {"x": gx + float(rng.normal(0, 0.02)), "y": gy + float(rng.normal(0, 0.02)),
                "theta_deg": math.degrees(robot.psi_real()) + float(rng.normal(0, 1.0))}

    def _sens_g(dd):
        """Lo que correlaciona el guardián: LIDAR (real del simulador), IMU (giro verdadero + ruido) y cámara SINTÉTICA (el simulador no
        renderiza el sendero: se deriva del error verdadero con ruido de 3 cm / 4° y confianza 0,7)."""
        e_t = guardian.errores({"x": robot.centro()[0], "y": robot.centro()[1], "theta_deg": math.degrees(robot.psi_real())}) or (0.0, 0.0)
        return {"lidar_adelante_m": dd.get("adelante"), "lidar_atras_m": dd.get("atras"), "lidar_izquierda_m": dd.get("izquierda"),
                "lidar_derecha_m": dd.get("derecha"), "cerca": dd.get("cerca"),
                "imu_gz": float(robot.d.dq[2]) + float(rng.normal(0, 0.01)), "imu_roll_deg": float(rng.normal(0, 0.4)),
                "imu_pitch_deg": float(rng.normal(0, 0.4)),
                "sendero_lateral_m": -e_t[0] + float(rng.normal(0, 0.03)), "sendero_rumbo_deg": -math.degrees(e_t[1]) + float(rng.normal(0, 4.0)),
                "sendero_confianza": 0.7}
    err, quieto, dist_rec = [], 0, 0.0
    pico_delta = 0.0
    ultimo = robot.centro()
    s_max = x0
    traj = []
    atasco_ref, atasco_t = (ultimo[0], ultimo[1]), 0.0
    atascado = False
    cerca_cada = 0
    mundo_cerca = mundo
    duty_max_visto = 0.0
    ok = False

    t_lim = esc.t_max * (2.2 if cfg.mx_regreso else 1.0)
    fase, ida_ok, inicio_xy = "ida", False, (x0, y0)
    while t < t_lim:
        # ---- conducción CONTINUA (control_pasillo): LIDAR a 10 Hz, mando a 20 Hz ----
        if ctrl is not None:
            if t + 1e-9 >= prox_scan_cc:
                prox_scan_cc += 0.1
                lx, ly = robot.lidar_pos()
                lidar.ingest_scan(lidar_scan(esc, lx, ly, robot.psi_real(), rng))
                d_cc = lidar.distancias(cfg.slam_cfg())
            if t + 1e-9 >= prox_cc:
                prox_cc += TICK_JETSON
                gyro = float(robot.d.dq[2]) + float(rng.normal(0.0, 0.01))
                v_c, w_c = ctrl.step(d_cc, gyro, TICK_JETSON)
                act.send_command(v_c, w_c, t)
        # ---- explorador (cada 0,25 s) ----
        if ctrl is None and t + 1e-9 >= prox_expl:
            prox_expl += TICK_EXPLORADOR
            lx, ly = robot.lidar_pos()
            scan = lidar_scan(esc, lx, ly, robot.psi_real(), rng)
            if scan:
                _a = np.radians([q_["angle_deg"] for q_ in scan])
                _d = np.array([q_["distance_m"] for q_ in scan])
                scan_xy = (_d * np.cos(_a), _d * np.sin(_a))
            else:
                scan_xy = (np.zeros(0), np.zeros(0))
            lidar.ingest_scan(scan)
            d = lidar.distancias(cfg.slam_cfg())
            if rf is not None and cfg.rumbo_paredes and not cond.activo and not cola_man:
                pi = [(r["distance_m"] * math.cos(math.radians(r["angle_deg"])), r["distance_m"] * math.sin(math.radians(r["angle_deg"])))
                      for r in scan if 40 <= r["angle_deg"] <= 140 and r["distance_m"] < 1.6]
                pd = [(r["distance_m"] * math.cos(math.radians(r["angle_deg"])), r["distance_m"] * math.sin(math.radians(r["angle_deg"])))
                      for r in scan if 220 <= r["angle_deg"] <= 320 and r["distance_m"] < 1.6]
                ref = RF.angulo_paredes(pi, pd, y_max=1.6)
                if ref is not None:
                    rf.actualizar(*ref)
                desvio_ult = RF.desvio_lateral(pi, pd, y_max=1.6) if cfg.rumbo_mirada_m > 0 else None
            if cfg.mx_memoria_w > 0:
                vx_, vy_ = robot.centro()
                kk = (round(vx_ / 0.5), round(vy_ / 0.5))
                visitas[kk] = visitas.get(kk, 0) + 1
            if mision is not None:
                cmx, cmy = robot.centro()
                pedir = mision.tick(t, cmx, cmy, robot.psi_real(), TICK_EXPLORADOR)
                if pedir > 0:
                    pausa_hasta = max(pausa_hasta, t + pedir)       # el robot se queda quieto escaneando
                    cola_man = []
            frente = d["adelante"] if d["adelante"] is not None else math.inf
            if cfg.mx_barrido:
                sel = (scan_xy[0] > 0) & (np.abs(scan_xy[1]) < cfg.mx_ancho_half_m)
                if sel.any():
                    frente = min(frente, float(scan_xy[0][sel].min()) - cfg.lidar_adelante_cm / 100.0)
            izq = d["izquierda"]
            der = d["derecha"]
            li = izq if izq is not None else math.inf
            ld = der if der is not None else math.inf
            # progreso para el escape: referencia que se mueve cuando el robot avanza 2 cm
            cx_, cy_ = robot.centro()
            if math.hypot(cx_ - prog_ref[0], cy_ - prog_ref[1]) > 0.02:
                prog_ref, prog_t = (cx_, cy_), t
            if t < pausa_hasta and not cola_man:
                pass
            elif cola_man:                                      # maniobra en curso: se ejecuta el siguiente pulso cuando el conductor queda libre
                if not cond.activo:
                    _ord(cola_man.pop(0), t, maniobra=False)
            elif cfg.mx_anti_dither and _atrapado(t, cx_, cy_, bloqueos):
                lado_op = ("derecha" if ultimo_giro == "izquierda" else "izquierda") if ultimo_giro else ("izquierda" if li >= ld else "derecha")
                atras_ok = d.get("atras") is None or d["atras"] >= 0.30
                n_largo = max(1, int(cfg.mx_giro_largo))
                if cfg.mx_memoria_w > 0:                           # escape inteligente: el hueco menos pisado en TODAS las direcciones (incluida dar la vuelta)
                    g360 = _mejor_hueco(scan, 0.8, 0.0, full=True)
                    if g360 is not None:
                        lado_op = "izquierda" if g360 > 0 else "derecha"
                        n_largo = int(_clamp_(round(abs(math.degrees(g360)) / cfg.mx_grados_pulso), 2, 30))
                cola_man = (["atras"] * 3 if atras_ok else []) + [lado_op] * n_largo + ["adelante"] * 3
                mx_commit, mx_commit_hasta = lado_op, t + 2 * cfg.mx_bordear_s
                ultimo_giro = lado_op
                mx_escapes += 1
                mx_hist.clear()
                prog_ref, prog_t = (cx_, cy_), t
            elif cfg.man_escape_s > 0 and t - prog_t > cfg.man_escape_s:
                lado = "izquierda" if li >= ld else "derecha"   # secuencia de escape: atrás x2 + giro x3 hacia el lado libre
                cola_man = ["atras", "atras", lado, lado, lado]
                escapes += 1
                prog_ref, prog_t = (cx_, cy_), t
            elif frente < cfg.avance_libre_cm / 100.0:
                bloqueos += 1
                gap = _mejor_hueco(scan, cfg.mx_hueco_m, 1.0 if esc.linea_central is None else 0.2) if cfg.mx_hueco else None
                if gap is None and cfg.mx_hueco:
                    gap = _mejor_hueco(scan, 0.6, 1.0 if esc.linea_central is None else 0.2)
                if gap is not None and abs(math.degrees(gap)) >= 4.0:
                    lado = "izquierda" if gap > 0 else "derecha"
                elif gap is not None:
                    lado = "izquierda" if li >= ld else "derecha"
                elif cfg.mx_meta and esc.linea_central is None and mx_commit and t < mx_commit_hasta and (li if mx_commit == "izquierda" else ld) >= 0.30:
                    lado = mx_commit                            # bordear: sigue por el mismo lado hasta rodear el obstáculo
                elif cfg.mx_meta and esc.linea_central is None:
                    eb = _rumbo_meta()
                    if abs(eb) < math.radians(10):
                        lado = "izquierda" if li >= ld else "derecha"
                    else:
                        pref = "izquierda" if eb > 0 else "derecha"
                        lado = pref if (li if pref == "izquierda" else ld) >= 0.35 else ("derecha" if pref == "izquierda" else "izquierda")
                    mx_commit, mx_commit_hasta = lado, t + cfg.mx_bordear_s
                elif cfg.man_histeresis_cm > 0 and ultimo_giro and abs(li - ld) < cfg.man_histeresis_cm / 100.0:
                    lado = ultimo_giro
                else:
                    lado = "izquierda" if li >= ld else "derecha"
                ultimo_giro = lado
                if rf is not None:
                    rf.e = 0.0                                  # el rumbo nuevo, tras esquivar, pasa a ser el deseado
                    rf.P[0][0] = math.radians(8.0) ** 2
                seq = []
                if cfg.man_retroceso_cm > 0 and frente < cfg.man_retroceso_cm / 100.0:
                    seq += ["atras"] * max(1, int(cfg.man_atras_pulsos))
                n_giro = max(1, int(cfg.man_giro_pulsos))
                if cfg.mx_hueco and gap is not None:
                    n_giro = int(_clamp_(round(abs(math.degrees(gap)) / cfg.mx_grados_pulso), 1, 9))
                seq += [lado] * n_giro
                if len(seq) == 1 and cfg.man_pausa_s <= 0:
                    _ord(seq[0], t, maniobra=False)
                else:
                    cola_man = seq[1:]
                    _ord(seq[0], t, maniobra=False)
                if cfg.man_pausa_s > 0:
                    pausa_hasta = t + cfg.man_pausa_s + 0.4 * len(seq)
                sin_corregir = 0
            else:
                corr = None
                n_corr = 1
                if pulso_recto_pend and not cond.activo and rf is not None:
                    corr_rumbo.fin_pulso_recto(rf.e)
                    pulso_recto_pend = False
                if (cfg.mx_meta and esc.linea_central is None and corr is None and not cond.activo and (guardian is None or guardian.pend is None)
                        and frente > 1.0 and li > 0.6 and ld > 0.6 and mx_rectos >= 3 and abs(_rumbo_meta()) > math.radians(cfg.mx_rumbo_tol_deg)):
                    _ord("izquierda" if _rumbo_meta() > 0 else "derecha", t, maniobra=False)
                    mx_rectos = 0
                    corr = "hecho"
                    correcciones += 1
                if corr is None and guardian is not None:
                    if guardian.pend is not None:                    # esperando a que asiente la corrección anterior (sin pulsos rectos mezclados)
                        corr = "hecho"
                        if not cond.activo:
                            g_libre_desde = t if g_libre_desde is None else g_libre_desde
                            if t - g_libre_desde >= 0.5:
                                g_reg.append(guardian.terminar(_pose_g(), _sens_g(d), colision=prev_choque))
                                g_libre_desde = None
                    elif not cond.activo:
                        sg = _sens_g(d)
                        pg = _pose_g()
                        dec = guardian.decidir(pg, sg)
                        if dec:
                            guardian.iniciar(dec, pg, sg)
                            _ord(dec["accion"], t, maniobra=False, esc_dur=dec["escala"])
                            correcciones += 1
                            corr = "hecho"
                            sin_corregir = 0
                if corr is None and rf is not None and not cond.activo:
                    e_eff = RF.error_con_mirada(rf.e, desvio_ult, cfg.rumbo_mirada_m)
                    corr = corr_rumbo.decidir(e_eff) if cfg.rumbo_modo == "pulso" else None   # primero el rumbo; el centrado lateral después
                    if corr:
                        rumbo_corr += 1
                        correcciones += 1
                        if _ord(corr, t, maniobra=False, esc_dur=cfg.rumbo_pulso_escala):
                            corr_ejec += 1
                        sin_corregir = 0
                        corr = "hecho"
                if corr is None and guardian is None and sin_corregir >= 1 and izq is not None and der is not None:
                    lim = cfg.sendero_pared_max_cm / 100.0
                    if izq <= lim and der <= lim:
                        e = izq - der
                        if abs(e) / max(cfg.sendero_tol_cm / 100.0, 1e-3) >= 1.0:
                            corr = "izquierda" if e > 0 else "derecha"
                            if cfg.man_corr_doble and abs(e) / max(cfg.sendero_tol_cm / 100.0, 1e-3) >= 2.0:
                                n_corr = 2
                if corr == "hecho":
                    pass
                elif corr:
                    correcciones += 1
                    if n_corr == 2:
                        cola_man = [corr]
                    if _ord(corr, t, maniobra=False):
                        corr_ejec += 1
                    sin_corregir = 0
                else:
                    esc_v = cfg.man_desacel_factor if (cfg.man_desacel_cm > 0 and frente < cfg.man_desacel_cm / 100.0) else 1.0
                    w_arco = 0.0
                    if rf is not None and cfg.rumbo_modo == "arco":
                        e_eff = RF.error_con_mirada(rf.e, desvio_ult, cfg.rumbo_mirada_m)
                        w_arco = max(-cfg.rumbo_arco_max, min(cfg.rumbo_arco_max, -cfg.rumbo_arco_k * e_eff))
                    if _ord("adelante", t, esc_v=esc_v, w_extra=w_arco):
                        mx_rectos += 1
                        if rf is not None:
                            corr_rumbo.inicio_pulso_recto(rf.e)
                            pulso_recto_pend = True
                    avances += 1
                    sin_corregir += 1
        # ---- conductor (cada 0,05 s) ----
        if ctrl is None and t + 1e-9 >= prox_cond:
            prox_cond += TICK_CONDUCTOR
            cond.paso(t)
        # ---- Jetson tick + trama al firmware (20 Hz) ----
        if t + 1e-9 >= prox_jet:
            prox_jet += TICK_JETSON
            act.tick(t)
            cola_enlace.append((t + LATENCIA_ENLACE_S, _pct(act.out_v, V_SCALE), _pct(act.out_w, W_SCALE)))
        while cola_enlace and cola_enlace[0][0] <= t + 1e-9:
            _, pv, pw = cola_enlace.pop(0)
            fw.recibir(pv, pw, t)
        # ---- firmware + dinámica (10 ms) ----
        duties = fw.tick(t)
        duty_max_visto = max(duty_max_visto, max(abs(x) for x in duties))
        robot.step(dt, duties, hip.dead, getattr(fw, "frenando", None))
        t += dt
        if rf is not None:
            rf.predecir(float(robot.d.dq[2]) + sesgo_gyro + float(rng.normal(0.0, sigma_gyro)), dt)

        cx, cy = robot.centro()
        dist_rec += math.hypot(cx - ultimo[0], cy - ultimo[1])
        ultimo = (cx, cy)
        s_max = max(s_max, cx)
        err.append(esc.desvio(cx, cy))
        quieto += (max(abs(x) for x in duties) < 1.0)
        pico_delta = max(pico_delta, abs(robot.delta()))
        q = robot.d.q
        # Colisión cada 50 ms y solo contra lo cercano (a <0,9 m): con ~60 paredes
        # comprobarlas todas en cada paso de 10 ms multiplicaba x8 el coste. A 0,3 m/s
        # el robot avanza 1,5 cm en 50 ms, así que no se pierde ningún contacto.
        if cerca_cada == 0:
            cerca_cada = 5
            mundo_cerca = MundoSim.cercano(mundo, cx, cy, 0.9)
        cerca_cada -= 1
        choque = H.choca(mundo_cerca, cx, cy, q[2], q[3], geo=robot.geo) if cerca_cada == 4 else prev_choque
        if cfg.paredes_solidas and cerca_cada == 4:
            if choque and ultima_ok is not None:                # choque inelástico: vuelve a donde no tocaba y se para
                d_ = robot.d
                d_.q[:] = ultima_ok[0]
                d_.dq[:] = 0.0
                d_.wheels[:] = 0.0
                d_.stop_impacts, d_.stop_energy_loss, d_.slip, d_.max_force_ratio = ultima_ok[1:]
            elif not choque:
                d_ = robot.d
                ultima_ok = (d_.q.copy(), d_.stop_impacts, d_.stop_energy_loss, d_.slip, d_.max_force_ratio)
        if choque and not prev_choque and (t - t_choque > 1.0 or not cfg.paredes_solidas):
            contactos += 1
            if len(contacto_info) < 12:                 # diagnóstico: dónde está lo que tocó, en ejes del robot (x adelante, y izquierda), y qué veía de frente
                psi_c = robot.psi_real()
                mejor_o = None
                for (ox, oy, orr) in esc.macetas:
                    dd_ = math.hypot(ox - cx, oy - cy) - orr
                    if mejor_o is None or dd_ < mejor_o[0]:
                        mejor_o = (dd_, ox - cx, oy - cy, "obj")
                for (x0w, y0w, x1w, y1w) in esc.paredes:
                    ex_, ey_ = x1w - x0w, y1w - y0w
                    L2_ = max(ex_ * ex_ + ey_ * ey_, 1e-9)
                    u_ = max(0.0, min(1.0, ((cx - x0w) * ex_ + (cy - y0w) * ey_) / L2_))
                    qx, qy = x0w + u_ * ex_, y0w + u_ * ey_
                    dd_ = math.hypot(qx - cx, qy - cy)
                    if mejor_o is None or dd_ < mejor_o[0]:
                        mejor_o = (dd_, qx - cx, qy - cy, "pared")
                rx = math.cos(psi_c) * mejor_o[1] + math.sin(psi_c) * mejor_o[2]
                ry = -math.sin(psi_c) * mejor_o[1] + math.cos(psi_c) * mejor_o[2]
                contacto_info.append({"t": round(t), "accion": cond.ultima_accion, "activo": bool(cond.activo), "tipo": mejor_o[3],
                                      "rel_x": round(rx, 2), "rel_y": round(ry, 2), "d": round(mejor_o[0], 2), "frente_leido": round(float(frente), 2) if math.isfinite(frente) else None})
            k_ac = (cond.ultima_accion or "?") + ("" if cond.activo else "·parado")
            contactos_por_accion[k_ac] = contactos_por_accion.get(k_ac, 0) + 1
        if choque:
            t_choque = t
        prev_choque = choque
        # Atasco: sin moverse 5 cm en 40 s simulados no va a salir solo; se corta para no
        # gastar minutos de CPU. Cuenta como fallo (progreso lo que lleve).
        if math.hypot(cx - atasco_ref[0], cy - atasco_ref[1]) > 0.05:
            atasco_ref, atasco_t = (cx, cy), t
        elif t - atasco_t > 40.0:
            atascado = True
            break
        if traza and int(round(t / dt)) % 10 == 0:
            traj.append((t, cx, cy, robot.psi_real(), robot.delta(), choque,
                         max(abs(x) for x in duties), int(cond.activo)))
        if cfg.mx_regreso and fase == "regreso":
            if math.hypot(cx - inicio_xy[0], cy - inicio_xy[1]) < 0.5:
                ok = True
                break
            continue
        llego_ida = (math.hypot(cx - esc.meta_xy[0], cy - esc.meta_xy[1]) < 0.25) if esc.meta_xy is not None else (cx >= esc.goal_x - 0.05)
        if llego_ida:
            if cfg.mx_regreso:                                   # da la vuelta: el punto de inicio pasa a ser la meta
                fase, ida_ok = "regreso", True
                mx_meta_xy = inicio_xy
                mx_commit, mx_hist = None, []
                if guardian is not None:
                    guardian.pend = None
                    guardian.fijar_polilinea(list(reversed(esc.linea_central)) if esc.linea_central else [(cx, cy), inicio_xy])
                continue
            ok = True
            break

    recorrido = esc.goal_x - x0 if esc.meta_xy is None else math.hypot(esc.meta_xy[0] - x0, esc.meta_xy[1] - y0)
    avance = (s_max - x0) if esc.meta_xy is None else max(0.0, 1 - math.hypot(
        ultimo[0] - esc.meta_xy[0], ultimo[1] - esc.meta_xy[1]) / max(recorrido, 1e-6)) * recorrido
    progreso = max(0.0, min(1.0, avance / max(recorrido, 1e-6)))
    if cfg.mx_regreso:                                           # ida y vuelta: la mitad del progreso es llegar a la meta y la otra mitad volver
        if ok:
            progreso = 1.0
        elif fase == "regreso":
            progreso = 0.5 + 0.5 * max(0.0, 1.0 - math.hypot(ultimo[0] - inicio_xy[0], ultimo[1] - inicio_xy[1]) / max(recorrido, 1e-6))
        else:
            progreso = 0.5 * progreso
    e = np.asarray(err) if err else np.zeros(1)
    res = {
        "escenario": esc.nombre, "config": cfg.nombre, "semilla": semilla,
        "exito": bool(ok and contactos == 0), "llego": bool(ok),
        "contactos": contactos, "progreso": round(progreso, 3),
        "t_s": round(t, 1), "dist_m": round(dist_rec, 2),
        "rmse_cm": round(float(np.sqrt(np.mean(e ** 2))) * 100, 1),
        "pico_cm": round(float(np.max(np.abs(e))) * 100, 1),
        "quieto_pct": round(100.0 * quieto / max(1, len(err)), 1),
        "pulsos": cond.aceptadas, "descartados": cond.descartadas,
        "correcciones": correcciones, "corr_ejecutadas": corr_ejec, "bloqueos": bloqueos, "escapes": escapes, "correcciones_rumbo": rumbo_corr,
        "atascado": atascado,
        "pivote_pico_deg": round(math.degrees(pico_delta), 1),
        "duty_max": round(duty_max_visto, 1),
        "mu": hip.mu, "dead": hip.dead, "bias": hip.bias, "drag": hip.drag,
    }
    res["mx_escapes"] = mx_escapes
    res["pred"] = dict(PRED)
    res["fase"] = fase
    res["ida_ok"] = bool(ida_ok)
    res["contactos_por_accion"] = contactos_por_accion
    res["contacto_info"] = contacto_info
    if guardian is not None:
        res["guardian"] = guardian.resumen()
    if mision is not None:
        res["mision"] = mision.resumen()
    if traza:
        res["traza"] = traj
    return res


def puntuar(r: dict) -> float:
    """Menos = mejor. Tocar una maceta pesa más que ir despacio o torcido."""
    return (8.0 * r["contactos"] + 6.0 * (1.0 - r["progreso"])
            + 0.02 * r["t_s"] + 3.0 * r["pico_cm"] / 100.0)


def puntuar_v2(r: dict) -> float:
    """Objetivo de la RONDA 2. Menos = mejor.

    La ronda 1 usaba `puntuar` y eligió una config que casi no se movía (éxito 0 %, atascada
    el 75 %): 6 puntos por progreso incompleto cuestan parecido a unos pocos contactos, así
    que el objetivo PREMIABA la inmovilidad. Aquí no llegar sin tocar nada vale 10 (más que
    cualquier combinación de contactos y lentitud), y el resto desempata.
    """
    return (10.0 * (0.0 if r["exito"] else 1.0) + 4.0 * r["contactos"]
            + 3.0 * (1.0 - r["progreso"]) + 0.01 * r["t_s"])


# ======================================================================
# Escenarios de interior
# ======================================================================
R_MACETA = 0.12


def _filas_pasillo(hueco, largo, paso=0.45, x_ini=0.7):
    borde = hueco / 2 + R_MACETA
    out, x = [], x_ini
    while x <= largo:
        out += [(x, borde, R_MACETA), (x, -borde, R_MACETA)]
        x += paso
    return out


def _eslalon(hueco, largo, paso=0.7, x_ini=0.8):
    borde = hueco / 2 + R_MACETA
    out, lado, x = [], 1, x_ini
    while x <= largo:
        out.append((x, lado * borde, R_MACETA))
        lado *= -1
        x += paso
    return out


def _sala(largo, ancho):
    a = ancho / 2
    return [(-0.6, a, largo, a), (-0.6, -a, largo, -a),
            (-0.6, -a, -0.6, a), (largo, -a, largo, a)]


def _curva_pasillo(ancho=0.9, R=1.6, grados=60, recto_ini=0.7, recto_fin=0.9):
    """Pasillo con una curva suave a la izquierda. Devuelve paredes, línea central, meta."""
    th = math.radians(grados)
    n = 18
    centro = [(-recto_ini, 0.0), (0.0, 0.0)]
    centro += [(R * math.sin(a), R - R * math.cos(a)) for a in np.linspace(0, th, n)[1:]]
    ex, ey = centro[-1]
    centro.append((ex + recto_fin * math.cos(th), ey + recto_fin * math.sin(th)))

    def lado(signo):
        pts = []
        for (ax, ay), (bx, by) in zip(centro, centro[1:]):
            tx, ty = bx - ax, by - ay
            L = math.hypot(tx, ty)
            nx, ny = -ty / L, tx / L                    # normal izquierda
            pts += [(ax + signo * ancho / 2 * nx, ay + signo * ancho / 2 * ny),
                    (bx + signo * ancho / 2 * nx, by + signo * ancho / 2 * ny)]
        return pts
    paredes = []
    for signo in (+1, -1):
        pts = lado(signo)
        # une los extremos de cada tramo para no dejar rendijas
        seg = []
        for k in range(0, len(pts), 2):
            seg.append((pts[k], pts[k + 1]))
        for k, (a, b) in enumerate(seg):
            paredes.append((a[0], a[1], b[0], b[1]))
            if k + 1 < len(seg):
                c = seg[k + 1][0]
                paredes.append((b[0], b[1], c[0], c[1]))
    fin = centro[-1]
    # tapa del fondo, lejos de la meta para no disparar la regla de "frente bloqueado"
    tx, ty = math.cos(th), math.sin(th)
    nx, ny = -ty, tx
    f2 = (fin[0] + 1.6 * tx, fin[1] + 1.6 * ty)
    for signo in (+1, -1):
        paredes.append((fin[0] + signo * ancho / 2 * nx, fin[1] + signo * ancho / 2 * ny,
                        f2[0] + signo * ancho / 2 * nx, f2[1] + signo * ancho / 2 * ny))
    paredes.append((f2[0] + ancho / 2 * nx, f2[1] + ancho / 2 * ny,
                    f2[0] - ancho / 2 * nx, f2[1] - ancho / 2 * ny))
    # tapa de atrás
    a0 = centro[0]
    paredes.append((a0[0], a0[1] + ancho / 2, a0[0], a0[1] - ancho / 2))
    return paredes, centro, fin


def escenarios_interior(hueco=0.60) -> dict:
    """Interiores. El fondo está SIEMPRE a >1,5 m de la meta: con `avance_libre_cm`=100
    el robot real se niega a avanzar a menos de 1 m de una pared, y eso no es lo que se
    mide aquí."""
    L = 2.6
    fondo = L + 1.7
    sala = _sala(fondo, 3.0)
    esc = {
        "sala_vacia": Escenario("sala_vacia", sala, [], goal_x=L, t_max=130.0,
                                descripcion="Referencia: 2,6 m en una sala sin obstáculos."),
        "pasillo_macetas": Escenario("pasillo_macetas", sala, _filas_pasillo(hueco, L), goal_x=L, t_max=130.0,
                                     descripcion=f"Dos filas de macetas, paso libre {hueco*100:.0f} cm."),
        "eslalon_macetas": Escenario("eslalon_macetas", sala, _eslalon(hueco, L), goal_x=L, t_max=130.0,
                                     descripcion=f"Macetas alternas a cada lado, paso libre {hueco*100:.0f} cm."),
    }
    paredes, centro, fin = _curva_pasillo()
    esc["curva_pasillo"] = Escenario(
        "curva_pasillo", paredes, [], goal_x=fin[0], t_max=170.0, meta_xy=fin,
        linea_central=centro,
        descripcion="Pasillo de 90 cm con una curva suave de 60° a la izquierda (radio 1,6 m).")
    return esc


if __name__ == "__main__":
    import json
    import time
    cfg = config_actual()
    for nombre, e in escenarios_interior().items():
        t0 = time.perf_counter()
        r = episodio(e, cfg, 0)
        print(f"{nombre:16s} {time.perf_counter()-t0:5.1f}s reales -> "
              + json.dumps({k: r[k] for k in ('exito', 'llego', 'contactos', 'progreso', 't_s',
                                              'pico_cm', 'pulsos', 'descartados', 'correcciones',
                                              'bloqueos', 'duty_max')}, ensure_ascii=False))
