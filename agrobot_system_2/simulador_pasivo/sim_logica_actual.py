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

import math
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
from manual_demo import reference as demo_reference   # noqa: E402  (REAL)
from ble_link import _pct                             # noqa: E402  (REAL)
from slam import LidarReactive                        # noqa: E402  (REAL)
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
MID = 0.185                             # del eje delantero REAL al centro del robot
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
    robot_largo_cm: float = 40
    robot_ancho_cm: float = 37
    # Tope de avance que la Jetson deja pasar, en % de duty. 60 = el real: pulse_drive recorta
    # a 0,60*V_SCALE y manual_demo.command a .60. Cambiarlo es un "¿y si?": exigiría tocar esos
    # DOS sitios (son límites de seguridad que puso Codex), NO es un parámetro de config.
    tope_duty: float = 60.0
    # --- firmware 1.2 (/config) ---
    fw_fwd: float = 60.0
    fw_rev: float = 35.0
    fw_minp: float = 0.0
    fw_inner: float = 65.0
    fw_slew: float = 60.0
    fw_gap_ms: float = 25.0

    def slam_cfg(self):
        return {"robot_largo_cm": self.robot_largo_cm, "robot_ancho_cm": self.robot_ancho_cm,
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
                v, w = demo_reference(self.ramp_v, self.ramp_w)          # el REAL
            else:
                v, w = demo_ref_tope(self.ramp_v, self.ramp_w, self.cfg.tope_duty)
        except (ValueError, TypeError):
            v = w = 0.0
        self.out_v, self.out_w = v, w

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

    @property
    def activo(self) -> bool:
        return self._gen is not None

    def ordenar(self, accion, t, maniobra=True) -> bool:
        v_max = self.cfg.max_v_mps * self.cfg.escala_velocidad
        w_max = self.cfg.max_omega_rps * self.cfg.escala_velocidad
        segs = self._real.segmentos(accion, v_max, w_max, maniobra)
        if self.cfg.tope_duty != 60.0:            # ¿y si? subir el tope de avance
            vf = pulse_drive.MODOS[self.cfg.modo]["vel_factor"]
            tope_v = self.cfg.tope_duty / 100.0 * V_SCALE
            segs = [((min(v_max * vf, tope_v) if v > 0 else v), w, d) for (v, w, d) in segs]
        if self.activo:                           # Codex: `if self.activo: return False`
            self.descartadas += 1
            return False
        self._gen = self._ejecutar(segs)
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


def lidar_scan(esc: Escenario, cx, cy, psi_real, rng, ruido=0.01, rango=8.0):
    """360 rayos desde el centro. 0° = adelante, + = izquierda (como ROS)."""
    ang = np.deg2rad(np.arange(360)) + psi_real
    dx, dy = np.cos(ang), np.sin(ang)
    dist = np.full(360, np.inf)
    for (x, y, r) in esc.macetas:                 # círculos
        fx, fy = cx - x, cy - y
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - r * r
        disc = b * b - c
        ok = disc >= 0
        t = np.where(ok, -b - np.sqrt(np.where(ok, disc, 0.0)), np.inf)
        t = np.where(t > 0.02, t, np.inf)
        dist = np.minimum(dist, t)
    for (x0, y0, x1, y1) in esc.paredes:          # segmentos
        ex, ey = x1 - x0, y1 - y0
        den = dx * ey - dy * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            tt = ((x0 - cx) * ey - (y0 - cy) * ex) / den
            uu = ((x0 - cx) * dy - (y0 - cy) * dx) / den
        ok = (np.abs(den) > 1e-9) & (tt > 0.02) & (uu >= 0) & (uu <= 1)
        dist = np.minimum(dist, np.where(ok, tt, np.inf))
    dist = np.where(dist <= rango, dist + rng.normal(0, ruido, 360), np.inf)
    return [{"angle_deg": float(a), "distance_m": float(d)}
            for a, d in zip(range(360), dist) if np.isfinite(d) and d > 0.05]


class RobotModelo:
    """Envuelve DinamicaPasiva con el marco girado 180° (pivote trasero)."""

    def __init__(self, params, pose_real):
        x, y, psi = pose_real                       # CENTRO y rumbo reales
        self.p = params
        psi_m = psi + math.pi
        # origen del modelo = eje delantero real = centro + MID hacia delante real
        ox, oy = x + MID * math.cos(psi), y + MID * math.sin(psi)
        self.d = DinamicaPasiva(pose=(ox, oy, psi_m), parametros=params)
        self.geo = H.geo_de_parametros(params)

    def centro(self):
        ox, oy, psi_m, delta = self.d.q
        return ox + MID * math.cos(psi_m), oy + MID * math.sin(psi_m)

    def psi_real(self):
        return math.atan2(math.sin(self.d.q[2] + math.pi), math.cos(self.d.q[2] + math.pi))

    def delta(self):
        return float(self.d.q[3])

    def step(self, dt, duties, dead):
        """duties firmados [R1,L1,L2,R2] en %  ->  velocidades de rueda del modelo."""
        vel, libre = [], []
        for dpc in duties:
            # Duty 0 (o por debajo de la zona muerta) = rueda LIBRE, no frenada: el firmware
            # hace detachMotor() y el puente queda en alta impedancia.
            es_libre = abs(dpc) < max(dead, 1e-6)
            libre.append(es_libre)
            vel.append(0.0 if es_libre else -(dpc / 100.0 * V_SCALE))   # avanzar real = retroceder en el modelo
        R1, L1, L2, R2 = vel
        fR1, fL1, fL2, fR2 = libre
        self.d.step(dt, 0.0, 0.0, wheel_targets=[R1, L1, R2, L2],       # [trasL,trasR,delL,delR]
                    libres=[fR1, fL1, fR2, fL2])


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
    # Rozamiento viscoso de rodadura por rueda (N·m por rad/s). El valor por defecto del
    # simulador (0,001) hacía rodar al robot 63 cm por pulso; lo MEDIDO el 2026-10-02 fueron
    # 26,2 ± 2,2 cm (n=4, firmware antiguo, ~87 % de duty). Eso corresponde a ~0,02. Es UN dato
    # y el duty exacto es incierto (78-100 % => 0,015-0,03), por eso se aleatoriza.
    drag: float = 0.02


def hip_por_semilla(semilla: int) -> Hip:
    rng = np.random.default_rng(777 + semilla)
    return Hip(mu=float(rng.choice([0.65, 0.8, 0.95])),
               dead=float(rng.choice([0.0, 25.0, 40.0])),
               bias=float(rng.choice([1.0, 2.0])), semilla_chasis=int(semilla),
               drag=float(rng.choice([0.012, 0.02, 0.03])))


def episodio(esc: Escenario, cfg: Config, semilla: int, hip: Hip | None = None,
             traza: bool = False) -> dict:
    hip = hip or hip_por_semilla(semilla)
    rng = np.random.default_rng(10_000 + semilla)
    params = replace(parametros_rusticos(hip.semilla_chasis), mu=hip.mu,
                     front_drive_bias=hip.bias, rolling_drag=hip.drag, substep=SUBSTEP)
    # perturbación inicial: el robot nunca arranca perfectamente centrado
    x0, y0, psi0 = esc.inicio
    y0 += rng.uniform(-0.04, 0.04)
    psi0 += math.radians(rng.uniform(-3.0, 3.0))
    robot = RobotModelo(params, (x0, y0, psi0))
    mundo = MundoSim(esc)

    act = ActuadorJetson(cfg)
    cond = Conductor(cfg, act)
    fw = MotoresFirmware(cfg)
    lidar = LidarReactive()
    cola_enlace = []                               # (t_llegada, pct_v, pct_w)

    t = 0.0
    dt = TICK_FW
    prox_expl, prox_cond, prox_jet = TICK_EXPLORADOR, 0.0, 0.0
    sin_corregir = 0
    correcciones = corr_ejec = bloqueos = avances = 0
    contactos, prev_choque = 0, False
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

    while t < esc.t_max:
        # ---- explorador (cada 0,25 s) ----
        if t + 1e-9 >= prox_expl:
            prox_expl += TICK_EXPLORADOR
            cx, cy = robot.centro()
            lidar.ingest_scan(lidar_scan(esc, cx, cy, robot.psi_real(), rng))
            d = lidar.distancias(cfg.slam_cfg())
            frente = d["adelante"] if d["adelante"] is not None else math.inf
            izq = d["izquierda"]
            der = d["derecha"]
            if frente < cfg.avance_libre_cm / 100.0:
                bloqueos += 1
                li = izq if izq is not None else math.inf
                ld = der if der is not None else math.inf
                cond.ordenar("izquierda" if li >= ld else "derecha", t, maniobra=False)
                sin_corregir = 0
            else:
                corr = None
                if sin_corregir >= 1 and izq is not None and der is not None:
                    lim = cfg.sendero_pared_max_cm / 100.0
                    if izq <= lim and der <= lim:
                        e = izq - der
                        if abs(e) / max(cfg.sendero_tol_cm / 100.0, 1e-3) >= 1.0:
                            corr = "izquierda" if e > 0 else "derecha"
                if corr:
                    correcciones += 1
                    if cond.ordenar(corr, t, maniobra=False):
                        corr_ejec += 1
                    sin_corregir = 0
                else:
                    cond.ordenar("adelante", t)
                    avances += 1
                    sin_corregir += 1
        # ---- conductor (cada 0,05 s) ----
        if t + 1e-9 >= prox_cond:
            prox_cond += TICK_CONDUCTOR
            cond.paso(t)
        # ---- Jetson tick + trama al firmware (20 Hz) ----
        if t + 1e-9 >= prox_jet:
            prox_jet += TICK_JETSON
            act.tick(t)
            cola_enlace.append((t + LATENCIA_ENLACE_S, _pct(act.out_v, V_SCALE), _pct(act.out_w, W_SCALE)))
        while cola_enlace and cola_enlace[0][0] <= t + 1e-9:
            _, pv, pw = cola_enlace.pop(0)
            fw.set_targets(fw_mix(pv * 0.01 * V_SCALE, pw * 0.01 * W_SCALE, cfg), t)
        # ---- firmware + dinámica (10 ms) ----
        duties = fw.tick(t)
        duty_max_visto = max(duty_max_visto, max(abs(x) for x in duties))
        robot.step(dt, duties, hip.dead)
        t += dt

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
        if choque and not prev_choque:
            contactos += 1
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
        if esc.meta_xy is not None:
            if math.hypot(cx - esc.meta_xy[0], cy - esc.meta_xy[1]) < 0.25:
                ok = True
                break
        elif cx >= esc.goal_x - 0.05:
            ok = True
            break

    recorrido = esc.goal_x - x0 if esc.meta_xy is None else math.hypot(esc.meta_xy[0] - x0, esc.meta_xy[1] - y0)
    avance = (s_max - x0) if esc.meta_xy is None else max(0.0, 1 - math.hypot(
        ultimo[0] - esc.meta_xy[0], ultimo[1] - esc.meta_xy[1]) / max(recorrido, 1e-6)) * recorrido
    progreso = max(0.0, min(1.0, avance / max(recorrido, 1e-6)))
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
        "correcciones": correcciones, "corr_ejecutadas": corr_ejec, "bloqueos": bloqueos,
        "atascado": atascado,
        "pivote_pico_deg": round(math.degrees(pico_delta), 1),
        "duty_max": round(duty_max_visto, 1),
        "mu": hip.mu, "dead": hip.dead, "bias": hip.bias, "drag": hip.drag,
    }
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
