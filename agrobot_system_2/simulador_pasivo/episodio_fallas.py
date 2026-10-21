"""Episodios con FALLAS inyectadas, choques con respuesta física, atascos, paradas para escanear y
recuperación con LECCIONES + MANEJO ASISTIDO (2026-10-20).

Qué añade a `sim_logica_actual.episodio` (que se deja intacto: sus resultados previos siguen valiendo):
  · CHOQUES con respuesta física: al tocar algo el robot ya no lo atraviesa; se le quita la velocidad hacia el
    obstáculo y se le saca del solape. Las ruedas siguen empujando: así se TRABA de verdad.
  · ATASCO como el real (`explorador._atascado`): sin moverse 12 cm en 10 s (el real son 25 s; se acorta por la
    duración de los episodios). Entonces recupera: primero busca una LECCIÓN (`aprendizaje.Aprendizaje`, el mismo módulo
    que usa el robot real) y si no hay, hace el desatasco por defecto (atrás, atrás, girar al lado contrario).
  · PARADAS PARA ESCANEAR: periódicas (cada `escaneo_cada_s`) y al encontrar una planta delante (como `escanear_al_detectar`):
    el robot se detiene `escaneo_dur_s` y reanuda desde parado (con zona muerta y rozamiento estático).
  · CATÁLOGO DE FALLAS: maceta baja y vidrio (el LIDAR no los ve), barro, rueda muerta, motor débil, batería que cae,
    zona muerta alta, suelo resbaloso, cortes del LIDAR, fantasmas del LIDAR, cortes del enlace con la C3, una persona
    que cruza, un paso estrecho y arranque encajonado en una esquina.
  · MANEJO ASISTIDO: un «operador» con información PRIVILEGIADA (el mundo verdadero, incluido lo que el LIDAR no ve)
    ordena maniobras candidatas según el espacio libre real; la que saca al robot del apuro se guarda como lección
    (firma de lo que percibe el LIDAR -> secuencia de pasos), igual que si la hubiera enseñado una persona.

El robot nunca usa la información privilegiada: solo las lecciones, que se indexan por lo que ve el LIDAR.
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

import sim_logica_actual as S

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "jetson" / "backend"))
import aprendizaje as AP  # noqa: E402

ATASCO_S = 10.0              # sin moverse ATASCO_M en esto = atasco (real: 25 s)
ATASCO_M = 0.12
PAUSA_PASO_S = 1.0           # explorador.PAUSA_GIRO_S: pausa entre pasos de una maniobra
ESCANEO_DUR_S = 8.0          # autonomia.escaneo_duracion_s
ESCANEO_CADA_S = 45.0        # real 180 s; se acorta para que cada episodio incluya paradas
ESCANEO_PLANTA_M = 0.9       # distancia a la que una planta delante dispara el escaneo
LIDAR_VIEJO_S = 1.0          # explorador.SCAN_VIEJO_S
MAX_RECUPERACIONES = 8


# Grabador opcional (generar_dataset_slam.py): .scan(t, pts|None, (lx, ly), psi) y .paso(t, v_cmd, w_cmd, (lx, ly), psi).
GRABADOR = None


# ---------------------------------------------------------------------------------------------
# Catálogo
# ---------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Fallo:
    nombre: str
    descripcion: str
    escenarios: tuple = ("sala_vacia", "pasillo_macetas", "eslalon_macetas", "curva_pasillo")
    contactos_permitidos: int = 0     # choques inevitables con lo que el LIDAR no ve


TODOS = S.escenarios_interior().keys()
FALLOS = {f.nombre: f for f in (
    Fallo("ninguno", "Sin fallas: solo paradas para escanear y respuesta física a los choques."),
    Fallo("maceta_baja", "Una maceta BAJA (15 cm) en el camino: el LIDAR a 37,5 cm pasa por encima.",
          ("sala_vacia", "pasillo_macetas", "curva_pasillo"), 1),
    Fallo("maceta_baja_grupo", "Tres macetas bajas seguidas, invisibles para el LIDAR.", ("sala_vacia",), 3),
    Fallo("pared_vidrio", "Un panel de vidrio que el LIDAR atraviesa.", ("sala_vacia",), 1),
    Fallo("barro", "Un tramo de barro: poco agarre y mucha resistencia a la rodadura.", tuple(TODOS)),
    Fallo("rueda_muerta", "Una rueda sin fuerza: el robot deriva hacia ese lado.", tuple(TODOS)),
    Fallo("motor_debil", "Motores al 40 % (batería floja o tensión baja).", tuple(TODOS)),
    Fallo("bateria_cae", "La fuerza de los motores cae del 100 % al 45 % durante el episodio.", tuple(TODOS)),
    Fallo("zona_muerta_alta", "Los motores no se mueven por debajo del 55 % de duty.", tuple(TODOS)),
    Fallo("suelo_resbaloso", "Agarre bajo en todo el recorrido (mu 0,35).", tuple(TODOS)),
    Fallo("lidar_cortes", "El LIDAR deja de dar scans 2-4 s, dos veces.", tuple(TODOS)),
    Fallo("lidar_fantasmas", "El LIDAR inventa retornos cercanos delante (sol, reflejos).", tuple(TODOS)),
    Fallo("enlace_cortes", "Se corta el enlace con la C3 1-3 s, dos veces (el firmware frena a los 0,5 s).", tuple(TODOS)),
    Fallo("persona_cruza", "Una persona cruza delante a 0,4 m/s.", ("sala_vacia", "curva_pasillo")),
    Fallo("paso_estrecho", "Dos columnas dejan un paso de 52 cm (el robot mide ~38).", ("sala_vacia",)),
    Fallo("esquina_inicio", "Arranca encajonado en una esquina, mirando a la pared.", ("sala_vacia",)),
)}


def combinaciones(fallos=None):
    """[(fallo, escenario)] válidos."""
    return [(f, e) for f, d in FALLOS.items() if fallos is None or f in fallos for e in d.escenarios]


# ---------------------------------------------------------------------------------------------
# Geometría auxiliar
# ---------------------------------------------------------------------------------------------
def punto_en_ruta(esc, f):
    """(x, y, rumbo) a la fracción f del recorrido."""
    if esc.linea_central:
        pts = esc.linea_central
        L = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
        objetivo = f * sum(L)
        for (a, b), l in zip(zip(pts, pts[1:]), L):
            if objetivo <= l or (a, b) == (pts[-2], pts[-1]):
                u = 0.0 if l == 0 else min(1.0, objetivo / l)
                return a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]), math.atan2(b[1] - a[1], b[0] - a[0])
            objetivo -= l
    x0 = esc.inicio[0]
    return x0 + f * (esc.goal_x - x0), esc.centro_y, 0.0


def _dist_seg(px, py, x0, y0, x1, y1):
    ex, ey = x1 - x0, y1 - y0
    L2 = ex * ex + ey * ey
    u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x0) * ex + (py - y0) * ey) / L2))
    qx, qy = x0 + u * ex, y0 + u * ey
    return math.hypot(px - qx, py - qy), qx, qy


def holgura_real(cx, cy, ang_rad, mundo, hasta=1.5, paso=0.05, radio=0.24):
    """Distancia libre en una dirección contra el mundo VERDADERO (paredes y objetos, visibles o no). Información privilegiada."""
    d = paso
    while d <= hasta:
        px, py = cx + d * math.cos(ang_rad), cy + d * math.sin(ang_rad)
        for (x0, y0, x1, y1) in mundo.paredes:
            if _dist_seg(px, py, x0, y0, x1, y1)[0] < radio:
                return d
        for o in mundo.objetos:
            if math.hypot(px - o[0], py - o[1]) < radio + o[2]:
                return d
        d += paso
    return hasta


def normal_de_contacto(cx, cy, mundo):
    """Vector unitario del obstáculo más cercano hacia el robot, y su holgura (negativa = solape)."""
    mejor, nv = None, (1.0, 0.0)
    for (x0, y0, x1, y1) in mundo.paredes:
        d, qx, qy = _dist_seg(cx, cy, x0, y0, x1, y1)
        if mejor is None or d < mejor:
            mejor, nv = d, ((cx - qx) / max(d, 1e-9), (cy - qy) / max(d, 1e-9))
    for o in mundo.objetos:
        d = math.hypot(cx - o[0], cy - o[1]) - o[2]
        if mejor is None or d < mejor:
            dd = max(math.hypot(cx - o[0], cy - o[1]), 1e-9)
            mejor, nv = d, ((cx - o[0]) / dd, (cy - o[1]) / dd)
    return nv, mejor


# ---------------------------------------------------------------------------------------------
# Preparar la falla de un episodio
# ---------------------------------------------------------------------------------------------
def preparar(fallo: str, esc, hip, params, rng):
    """Devuelve (params, hip, ctx). ctx: lo que el bucle debe aplicar durante el episodio."""
    ctx = {"bajos": [], "paredes_ocultas": [], "barro": [], "cortes_lidar": [], "cortes_enlace": [],
           "fantasmas": 0.0, "persona": None, "bateria": False, "inicio": None, "columnas": []}
    if fallo == "ninguno":
        return params, hip, ctx
    if fallo in ("maceta_baja", "maceta_baja_grupo"):
        n = 1 if fallo == "maceta_baja" else 3
        for k in range(n):
            f = 0.40 + 0.08 * k if n > 1 else rng.uniform(0.35, 0.6)
            x, y, th = punto_en_ruta(esc, f)
            lado = float(rng.choice([-1.0, 1.0]))
            if esc.nombre == "sala_vacia":
                desv, r = 0.12 * lado, 0.10
            else:                                   # pasillo: pegada a una pared para dejar un carril de ~46 cm
                ancho = 0.60 if esc.nombre == "pasillo_macetas" else 0.90
                desv, r = (ancho / 2 - 0.07) * lado, 0.07
            ctx["bajos"].append((x - math.sin(th) * desv, y + math.cos(th) * desv, r))
    elif fallo == "pared_vidrio":
        x, y, _ = punto_en_ruta(esc, rng.uniform(0.45, 0.6))
        ctx["paredes_ocultas"].append((x, y - 0.55, x, y + 0.55))
    elif fallo == "barro":
        a = rng.uniform(0.30, 0.5)
        ctx["barro"].append((a, a + 0.22))                    # fracciones del recorrido
    elif fallo == "rueda_muerta":
        g = list(params.gain_ruedas)
        g[int(rng.integers(0, 4))] = 0.05
        params = replace(params, gain_ruedas=tuple(g))
    elif fallo == "motor_debil":
        params = replace(params, motor_gain=params.motor_gain * 0.4, max_torque=params.max_torque * 0.5)
    elif fallo == "bateria_cae":
        ctx["bateria"] = True
    elif fallo == "zona_muerta_alta":
        hip = replace(hip, dead=55.0)
    elif fallo == "suelo_resbaloso":
        hip = replace(hip, mu=0.35)
    elif fallo == "lidar_cortes":
        for k in range(2):
            t0 = rng.uniform(8 + 40 * k, 30 + 40 * k)
            ctx["cortes_lidar"].append((t0, t0 + rng.uniform(2.0, 4.0)))
    elif fallo == "lidar_fantasmas":
        ctx["fantasmas"] = 0.12
    elif fallo == "enlace_cortes":
        for k in range(2):
            t0 = rng.uniform(8 + 40 * k, 30 + 40 * k)
            ctx["cortes_enlace"].append((t0, t0 + rng.uniform(1.0, 3.0)))
    elif fallo == "persona_cruza":
        x, y, th = punto_en_ruta(esc, rng.uniform(0.35, 0.55))
        lado = rng.choice([-1.0, 1.0])
        ctx["persona"] = {"t0": rng.uniform(6.0, 14.0), "x": x, "y0": y + lado * 1.0, "vy": -lado * 0.4, "r": 0.2}
    elif fallo == "paso_estrecho":
        x, y, _ = punto_en_ruta(esc, 0.5)
        ctx["columnas"] = [(x, y + 0.26 + 0.12, 0.12), (x, y - 0.26 - 0.12, 0.12)]   # hueco 52 cm
    elif fallo == "esquina_inicio":
        ctx["inicio"] = (0.2, esc.centro_y + 1.18, math.radians(70))
    return params, hip, ctx


# ---------------------------------------------------------------------------------------------
# Manejo asistido: maniobras candidatas ordenadas con el mundo verdadero
# ---------------------------------------------------------------------------------------------
CANDIDATAS = {
    "A": ["atras", "atras", "izquierda", "izquierda", "adelante", "adelante"],
    "B": ["atras", "atras", "derecha", "derecha", "adelante", "adelante"],
    "C": ["atras", "atras", "atras", "izquierda", "adelante", "adelante", "derecha"],
    "D": ["atras", "atras", "atras", "derecha", "adelante", "adelante", "izquierda"],
    "E": ["izquierda", "izquierda", "izquierda", "adelante", "adelante"],
    "F": ["derecha", "derecha", "derecha", "adelante", "adelante"],
    "G": ["atras", "izquierda", "atras", "izquierda", "adelante"],
    "H": ["atras", "derecha", "atras", "derecha", "adelante"],
}


def accion_asistente(cx, cy, psi, mundo, acciones):
    """Un operador que ve el mundo VERDADERO maneja el robot paso a paso hasta sacarlo del apuro: adelante si hay sitio,
    atrás si está pegado a algo y hay hueco detrás, y si no, gira hacia el lado con más espacio real.
    Devuelve (accion, maniobra). Lo que haga se guarda como lección, igual que si lo hiciera una persona."""
    f = holgura_real(cx, cy, psi, mundo, hasta=1.0)
    b = holgura_real(cx, cy, psi + math.pi, mundo, hasta=0.8)
    l = holgura_real(cx, cy, psi + math.pi / 2, mundo, hasta=0.8)
    r = holgura_real(cx, cy, psi - math.pi / 2, mundo, hasta=0.8)
    if f >= 0.6:
        return "adelante", True
    if f < 0.30 and b >= 0.45:
        return "atras", True
    ultimos = [a["accion"] for a in acciones[-2:]]
    lado = "izquierda" if l >= r else "derecha"
    if ultimos and ultimos[-1] in ("izquierda", "derecha") and abs(l - r) < 0.15:
        lado = ultimos[-1]                                   # sin oscilar entre un lado y otro
    return lado, bool(f >= 0.35 and b >= 0.35)               # arco si cabe; si no, pivote


def ordenar_candidatas(cx, cy, psi, mundo, excluidas=()):
    """El asistente ve el mundo VERDADERO: prefiere retroceder si hay sitio detrás y girar hacia el lado con más espacio."""
    atras = holgura_real(cx, cy, psi + math.pi, mundo, hasta=0.8)
    izq = holgura_real(cx, cy, psi + math.pi / 2, mundo, hasta=0.8)
    der = holgura_real(cx, cy, psi - math.pi / 2, mundo, hasta=0.8)
    lado_izq = izq >= der

    def puntaje(nombre):
        pasos = CANDIDATAS[nombre]
        p = 0.0
        if pasos[0] == "atras":
            p += 1.0 if atras >= 0.45 else -2.0
        else:
            p += 0.3 if atras < 0.45 else -0.2
        giros = [s for s in pasos if s in ("izquierda", "derecha")]
        if giros:
            p += 1.0 if (giros[0] == "izquierda") == lado_izq else -0.6
            p += 0.3 * (max(izq, der) / 0.8)
        return p - 0.02 * len(pasos)
    return [n for n in sorted(CANDIDATAS, key=lambda n: -puntaje(n)) if n not in excluidas]


# ---------------------------------------------------------------------------------------------
# Episodio
# ---------------------------------------------------------------------------------------------
def episodio_fallas(esc, cfg, semilla: int, fallo: str = "ninguno", lecciones: AP.Aprendizaje | None = None,
                    asistir: bool = False, hip=None, escaneos: bool = True, traza: bool = False) -> dict:
    """Un episodio con la falla `fallo`. `lecciones`: almacén que el robot CONSULTA; con `asistir`, además se le
    añaden las maniobras que el asistente logra que funcionen. Devuelve el resultado y `lecciones_nuevas`/`juicios`."""
    hip = hip or S.hip_por_semilla(semilla)
    rng = np.random.default_rng(10_000 + semilla)
    rng_f = np.random.default_rng(55_000 + semilla)
    params = replace(S.parametros_rusticos(hip.semilla_chasis), mu=hip.mu, front_drive_bias=hip.bias,
                     rolling_drag=hip.drag, substep=S.SUBSTEP,
                     front_outer=S.PIVOTE_EXT_M, rear_outer=hip.eje_fijo or S.FIJO_EXT_M)
    if hip.masa is not None:
        params = replace(params, chassis_mass=hip.masa)
    params, hip, ctx = preparar(fallo, esc, hip, params, rng_f)
    mu_base, drag_base, motor_base = params.mu, params.rolling_drag, (params.motor_gain, params.max_torque)

    x0, y0, psi0 = ctx["inicio"] or esc.inicio
    if not ctx["inicio"]:
        y0 += rng.uniform(-0.04, 0.04)
        psi0 += math.radians(rng.uniform(-3.0, 3.0))
    robot = S.RobotModelo(params, (x0, y0, psi0))
    mundo = S.MundoSim(esc)
    mundo.objetos += [(x, y, r, 0.15, 1.0) for (x, y, r) in ctx["bajos"]]          # solo choque, el LIDAR no las ve
    mundo.objetos += [(x, y, r, 0.6, 1.0) for (x, y, r) in ctx["columnas"]]
    mundo.paredes += ctx["paredes_ocultas"]
    esc_lidar = esc
    if ctx["columnas"]:
        esc_lidar = replace(esc, macetas=list(esc.macetas) + ctx["columnas"])        # las columnas sí las ve

    act = S.ActuadorJetson(cfg)
    cond = S.Conductor(cfg, act)
    fw = S.crear_firmware(cfg)
    fw.omega_gain = hip.giro_gain
    lidar = S.LidarReactive()
    cola_enlace = []
    ctrl = None
    prox_cc = prox_scan_cc = 0.0
    if cfg.control == "continuo":
        from control_pasillo import PasilloController
        ctrl = PasilloController(v_cruise=cfg.cc_v, kp=cfg.cc_kp, kd=cfg.cc_kd, w_max=cfg.cc_wmax,
                                 nominal=cfg.cc_nominal, d_stop=cfg.cc_dstop, d_slow=cfg.cc_dslow,
                                 kh=cfg.cc_kh, kl=cfg.cc_kl)
    d_cc = {}
    t, dt = 0.0, S.TICK_FW
    prox_expl, prox_cond, prox_jet = S.TICK_EXPLORADOR, 0.0, 0.0
    sin_corregir = 0
    contactos, prev_choque, cerca_cada, mundo_cerca = 0, False, 0, mundo
    traj = []
    n_contacto, t_ult_choque = None, -1e9
    t_contactos = []
    ultimo = robot.centro()
    s_max, dist_rec, err = x0, 0.0, []
    recorrido0 = (esc.goal_x - x0) if esc.meta_xy is None else math.hypot(esc.meta_xy[0] - x0, esc.meta_xy[1] - y0)
    mejor_av = 0.0                                   # mayor avance a lo largo de la ruta visto hasta ahora

    def avance_actual(cx, cy):
        nonlocal mejor_av
        a = (cx - x0) if esc.meta_xy is None else (recorrido0 - math.hypot(esc.meta_xy[0] - cx, esc.meta_xy[1] - cy))
        mejor_av = max(mejor_av, a)
        return mejor_av
    atasco_ref, atasco_t = (ultimo[0], ultimo[1]), 0.0
    eje_ref = (float(robot.d.q[0]), float(robot.d.q[1]))
    estado, hasta_estado = "normal", 0.0           # normal | escaneando | recuperando
    cola, t_paso = [], 0.0
    ultimo_giro, pendiente = "izquierda", None
    t_ult_scan, prox_escaneo = 0.0, ESCANEO_CADA_S
    plantas_escaneadas = set()
    n_escaneos = n_recup = n_atascos = n_asist = n_leccion = 0
    rec_info = None                                  # recuperación en curso: dict
    ultima_recup_t, ultima_recup_pos = -1e9, (0.0, 0.0)
    lecciones_nuevas, juicios, eventos = [], [], []
    descartar_scan = 0
    t_extra = 0.0
    ok = False
    quieto_ticks = 0

    def sens(d):
        return {"lidar_adelante_m": d["adelante"], "lidar_atras_m": d["atras"], "lidar_izquierda_m": d["izquierda"],
                "lidar_derecha_m": d["derecha"], "sendero_lateral_m": None}

    def tomar_scan():
        """Un scan del LIDAR con las fallas aplicadas. None = el LIDAR no entregó nada."""
        nonlocal descartar_scan
        if any(a <= t < b for (a, b) in ctx["cortes_lidar"]):
            if GRABADOR is not None:
                GRABADOR.scan(t, None, robot.lidar_pos(), robot.psi_real())
            return None
        persona = ctx["persona"]
        e = esc_lidar
        if persona and t >= persona["t0"]:
            py = persona["y0"] + persona["vy"] * (t - persona["t0"])
            e = replace(esc_lidar, macetas=list(esc_lidar.macetas) + [(persona["x"], py, persona["r"])])
        lx, ly = robot.lidar_pos()
        pts = S.lidar_scan(e, lx, ly, robot.psi_real(), rng)
        if ctx["fantasmas"] and rng_f.random() < ctx["fantasmas"] * 0.25 or descartar_scan > 0:
            if descartar_scan == 0:
                descartar_scan = int(rng_f.integers(1, 4))
            descartar_scan -= 1
            ang = float(rng_f.uniform(-12, 12))
            pts = [p for p in pts if abs(((p["angle_deg"] + 180) % 360) - 180 - ang) > 4] + \
                  [{"angle_deg": ang % 360, "distance_m": float(rng_f.uniform(0.15, 0.30))}]
        if GRABADOR is not None:
            GRABADOR.scan(t, pts, (lx, ly), robot.psi_real())
        return pts

    def lanzar_recuperacion(motivo):
        """Atasco detectado: lección -> por defecto -> (asistente)."""
        nonlocal estado, cola, t_paso, pendiente, n_recup, n_atascos, n_leccion, rec_info, n_asist
        nonlocal ultima_recup_t, ultima_recup_pos, atasco_ref, atasco_t, t_extra, eje_ref
        d = lidar.distancias(cfg.slam_cfg())
        s = sens(d)
        n_atascos += 1
        t_extra += 15.0                                   # recuperarse también lleva tiempo
        cx, cy = robot.centro()
        repite = (t - ultima_recup_t < 30.0 and math.hypot(cx - ultima_recup_pos[0], cy - ultima_recup_pos[1]) < 0.25)
        ultima_recup_t, ultima_recup_pos = t, (cx, cy)
        info = {"sens": s, "firma": AP.firma(s, "atascado"), "pos": (cx, cy), "t": t, "origen": None, "prob": [],
                "prog0": avance_actual(cx, cy)}
        pasos = None
        if repite and asistir:
            # El robot ya probó lo suyo y sigue trabado: entra el asistente (manejo asistido).
            previas = rec_info["asistente_probadas"] if rec_info and rec_info.get("asistente_probadas") else []
            pasos = []                                              # los irá eligiendo el asistente, paso a paso
            info.update(origen="asistente", candidata="lazo", acciones=[], asistente_probadas=previas + ["lazo"])
            n_asist += 1
        if pasos is None and lecciones is not None:
            m = lecciones.consultar(s, "atascado")
            if m is not None:
                pasos = list(m.pasos)
                lecciones.aplicada(m, s, ahora=t)
                pendiente = {"m": m, "t": t, "pos": (cx, cy), "firma": m.firma}
                info.update(origen="leccion")
                n_leccion += 1
        if pasos is None:
            lado = "derecha" if ultimo_giro == "izquierda" else "izquierda"
            pasos = [{"accion": a, "maniobra": True} for a in ("atras", "atras", lado, lado, lado)]
            info["origen"] = "defecto"
        t_contactos.clear()                                        # los golpes de antes ya se atendieron
        if rec_info and rec_info.get("eval_en"):
            rec_info["eval_en"] = None                             # otra recuperación antes de tiempo: la anterior no sirvió
        cola, t_paso, estado = list(pasos), t, "recuperando"
        rec_info = info
        eventos.append((round(t, 1), motivo, info["origen"], info.get("candidata"), info["firma"], [round(c, 2) for c in (cx, cy)],
                        round(avance_actual(cx, cy), 2)))
        n_recup += 1
        cond._gen = None
        act.emergency_stop(t)
        atasco_ref, atasco_t = (cx, cy), t + 60.0       # no vuelve a disparar mientras recupera
        eje_ref = (float(robot.d.q[0]), float(robot.d.q[1]))

    while t < esc.t_max + t_extra:
        d_cc_ok = True
        # ---------------- sensores: LIDAR (con fallas) ----------------
        hubo_scan = False
        if ctrl is not None:
            if t + 1e-9 >= prox_scan_cc:
                prox_scan_cc += 0.1
                pts = tomar_scan()
                if pts is not None:
                    lidar.ingest_scan(pts)
                    t_ult_scan = t
                    d_cc = lidar.distancias(cfg.slam_cfg())
        elif t + 1e-9 >= prox_expl:
            pts = tomar_scan()
            if pts is not None:
                lidar.ingest_scan(pts)
                t_ult_scan = t
                hubo_scan = True
        lidar_ok = (t - t_ult_scan) <= LIDAR_VIEJO_S or t < LIDAR_VIEJO_S

        # ---------------- máquina de estados de la autonomía ----------------
        if estado == "normal" and escaneos and t >= prox_escaneo:
            estado, hasta_estado = "escaneando", t + ESCANEO_DUR_S
            prox_escaneo = hasta_estado + ESCANEO_CADA_S
            n_escaneos += 1
            t_extra += ESCANEO_DUR_S
            cond._gen = None
            act.emergency_stop(t)
        if estado == "escaneando":
            if t >= hasta_estado:
                estado = "normal"
                atasco_ref, atasco_t = robot.centro(), t
        elif estado == "recuperando":
            cx, cy = robot.centro()
            if not cond.activo and t >= t_paso:
                if not cola and rec_info and rec_info.get("origen") == "asistente" and len(rec_info["acciones"]) < 10:
                    d_p = lidar.distancias(cfg.slam_cfg())
                    libre_p = d_p["adelante"] is None or d_p["adelante"] >= 0.5
                    if not (math.hypot(cx - rec_info["pos"][0], cy - rec_info["pos"][1]) >= 0.40 and libre_p):
                        ac, man = accion_asistente(cx, cy, robot.psi_real(), mundo, rec_info["acciones"])
                        cola.append({"accion": ac, "maniobra": man})
                        rec_info["acciones"].append({"accion": ac, "maniobra": man})
                if cola:
                    p = cola.pop(0)
                    cond.ordenar(p["accion"], t, maniobra=p.get("maniobra", True))
                    if p["accion"] in ("izquierda", "derecha"):
                        ultimo_giro = p["accion"]
                    t_paso = t + PAUSA_PASO_S + 1.2
                else:
                    estado = "normal"
                    if rec_info and rec_info.get("origen") == "asistente":
                        rec_info["eval_en"] = t + 5.0              # 5 s después de la maniobra se mira si quedó libre
                    atasco_ref, atasco_t = (cx, cy), t
        elif not lidar_ok:
            # Sin scan fresco no se avanza (explorador: «pausada: sin datos del LIDAR»).
            if cond.activo or act.out_v or act.out_w:
                cond._gen = None
                act.emergency_stop(t)
            atasco_ref, atasco_t = robot.centro(), t
        else:
            # ---- decisión normal (como sim_logica_actual.episodio) ----
            if ctrl is not None:
                if t + 1e-9 >= prox_cc:
                    prox_cc += S.TICK_JETSON
                    gyro = float(robot.d.dq[2]) + float(rng.normal(0.0, 0.01))
                    v_c, w_c = ctrl.step(d_cc, gyro, S.TICK_JETSON)
                    act.send_command(v_c, w_c, t)
            elif hubo_scan:
                d = lidar.distancias(cfg.slam_cfg())
                frente = d["adelante"] if d["adelante"] is not None else math.inf
                izq, der = d["izquierda"], d["derecha"]
                # parar a escanear una planta delante (autonomia.escanear_al_detectar)
                if escaneos and frente < ESCANEO_PLANTA_M and esc.macetas and frente > 0.35:
                    cand = [k for k, (mx, my, mr) in enumerate(esc.macetas)
                            if k not in plantas_escaneadas and math.hypot(mx - robot.lidar_pos()[0], my - robot.lidar_pos()[1]) < ESCANEO_PLANTA_M + 0.2]
                    if cand:
                        plantas_escaneadas.add(cand[0])
                        estado, hasta_estado = "escaneando", t + ESCANEO_DUR_S
                        prox_escaneo = max(prox_escaneo, hasta_estado + ESCANEO_CADA_S)
                        n_escaneos += 1
                        t_extra += ESCANEO_DUR_S
                        cond._gen = None
                        act.emergency_stop(t)
                if estado == "normal":
                    if frente < cfg.avance_libre_cm / 100.0:
                        li = izq if izq is not None else math.inf
                        ld = der if der is not None else math.inf
                        lado = "izquierda" if li >= ld else "derecha"
                        if cond.ordenar(lado, t, maniobra=False):
                            ultimo_giro = lado
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
                            if cond.ordenar(corr, t, maniobra=False):
                                ultimo_giro = corr
                            sin_corregir = 0
                        else:
                            cond.ordenar("adelante", t)
                            sin_corregir += 1
        if t + 1e-9 >= prox_expl:
            prox_expl += S.TICK_EXPLORADOR

        # ---- juicio de la lección aplicada y de la maniobra del asistente ----
        cx, cy = robot.centro()
        if pendiente and t - pendiente["t"] >= AP.JUICIO_S:
            d = lidar.distancias(cfg.slam_cfg())
            res_j = lecciones.juzgar(sens(d), math.hypot(cx - pendiente["pos"][0], cy - pendiente["pos"][1]), ahora=t)
            if res_j is not None:
                juicios.append({"firma": pendiente["firma"], "pasos": [p["accion"] for p in pendiente["m"].pasos], "ok": bool(res_j)})
            pendiente = None
        if rec_info and rec_info.get("eval_en") and estado == "normal" and t >= rec_info["eval_en"]:
            # Como aprendizaje.juzgar: se movió y quedó despejado (no basta con haberse movido).
            d_ev = lidar.distancias(cfg.slam_cfg())
            libre = d_ev["adelante"] is None or d_ev["adelante"] >= 0.45
            movido = math.hypot(cx - rec_info["pos"][0], cy - rec_info["pos"][1])
            if movido >= 0.30 and libre and lecciones is not None:
                pasos = [dict(a) for a in rec_info["acciones"]]
                lecciones_nuevas.append({"firma": rec_info["firma"], "pasos": pasos, "origen": "asistente_sim",
                                         "candidata": "lazo"})
                rec_info["resuelta"] = True
            rec_info["eval_en"] = None

        # ---------------- enlace con la C3 ----------------
        if t + 1e-9 >= prox_cond:
            prox_cond += S.TICK_CONDUCTOR
            if ctrl is None or estado == "recuperando":
                cond.paso(t)
        if t + 1e-9 >= prox_jet:
            prox_jet += S.TICK_JETSON
            act.tick(t)
            if not any(a <= t < b for (a, b) in ctx["cortes_enlace"]):
                cola_enlace.append((t + S.LATENCIA_ENLACE_S, S._pct(act.out_v, S.V_SCALE), S._pct(act.out_w, S.W_SCALE)))
        while cola_enlace and cola_enlace[0][0] <= t + 1e-9:
            _, pv, pw = cola_enlace.pop(0)
            fw.recibir(pv, pw, t)

        # ---------------- terreno y batería ----------------
        if ctx["barro"]:
            f = min(1.0, max(0.0, (cx - x0) / max(esc.goal_x - x0, 1e-6)))
            en_barro = any(a <= f <= b for (a, b) in ctx["barro"])
            robot.d.p.mu = 0.25 if en_barro else mu_base
            robot.d.p.rolling_drag = drag_base * 3.0 if en_barro else drag_base
        if ctx["bateria"]:
            k = max(0.45, 1.0 - 0.55 * t / max(esc.t_max, 1.0))
            robot.d.p.motor_gain, robot.d.p.max_torque = motor_base[0] * k, motor_base[1] * k

        # ---------------- dinámica ----------------
        duties = fw.tick(t)
        robot.step(dt, duties, hip.dead)
        t += dt
        if GRABADOR is not None:
            GRABADOR.paso(t, act.out_v, act.out_w, robot.lidar_pos(), robot.psi_real())

        # ---------------- métricas, colisión y respuesta física ----------------
        cx, cy = robot.centro()
        dist_rec += math.hypot(cx - ultimo[0], cy - ultimo[1])
        ultimo = (cx, cy)
        s_max = max(s_max, cx)
        err.append(esc.desvio(cx, cy))
        quieto_ticks += (max(abs(x) for x in duties) < 1.0)
        q = robot.d.q
        if ctx["persona"] and t >= ctx["persona"]["t0"] and cerca_cada == 0:
            p_ = ctx["persona"]
            mundo.objetos = [o for o in mundo.objetos if o[3] != 1.7] + [(p_["x"], p_["y0"] + p_["vy"] * (t - p_["t0"]), p_["r"], 1.7, 1.0)]
        if cerca_cada == 0:
            cerca_cada = 5
            mundo_cerca = S.MundoSim.cercano(mundo, cx, cy, 0.9)
        cerca_cada -= 1
        if cerca_cada == 4:
            choque = S.H.choca(mundo_cerca, cx, cy, q[2], q[3], geo=robot.geo)
            if choque:
                # Un contacto NUEVO = ha pasado >0,8 s sin tocar nada (el rebote y el roce no se cuentan de nuevo).
                if t - t_ult_choque > 0.8 and t > 0.5:
                    contactos += 1
                    t_contactos.append(t)
                t_ult_choque = t
            if choque:
                n_contacto, _ = normal_de_contacto(cx, cy, mundo_cerca)
            else:
                n_contacto = None
            prev_choque = choque
        if n_contacto is not None:
            # Respuesta física: sin velocidad hacia el obstáculo y un empujoncito hacia fuera.
            vx, vy = float(robot.d.dq[0]), float(robot.d.dq[1])
            vn = vx * n_contacto[0] + vy * n_contacto[1]
            if vn < 0:
                robot.d.dq[0] -= vn * n_contacto[0]
                robot.d.dq[1] -= vn * n_contacto[1]
            robot.d.q[0] += 0.0004 * n_contacto[0]
            robot.d.q[1] += 0.0004 * n_contacto[1]

        # ---------------- atasco ----------------
        if estado == "normal":
            mov_centro = math.hypot(cx - atasco_ref[0], cy - atasco_ref[1])
            mov_eje = math.hypot(float(robot.d.q[0]) - eje_ref[0], float(robot.d.q[1]) - eje_ref[1])
            if min(mov_centro, mov_eje) > ATASCO_M:           # girar alrededor de un punto no es avanzar
                atasco_ref, atasco_t, eje_ref = (cx, cy), t, (float(robot.d.q[0]), float(robot.d.q[1]))
            elif t - atasco_t > ATASCO_S and lidar_ok:
                if n_recup >= MAX_RECUPERACIONES:
                    break
                lanzar_recuperacion("atasco")
            elif lidar_ok and sum(1 for x in t_contactos if t - x < 20.0) >= 3:
                # Chocar una y otra vez contra algo que el LIDAR no ve: el IMU notaría los golpes. Es un atasco.
                if n_recup >= MAX_RECUPERACIONES:
                    break
                t_contactos.clear()
                lanzar_recuperacion("choques")
        if traza and int(round(t / dt)) % 10 == 0:
            traj.append((t, cx, cy, robot.psi_real(), robot.delta(), bool(prev_choque), max(abs(x) for x in duties), 0))

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
    permitidos = FALLOS[fallo].contactos_permitidos
    return {
        **({"traza": traj} if traza else {}),
        "escenario": esc.nombre, "fallo": fallo, "config": cfg.nombre, "semilla": semilla,
        "llego": bool(ok), "contactos": contactos, "contactos_permitidos": permitidos,
        "pasa": bool(ok and contactos <= permitidos), "progreso": round(progreso, 3), "t_s": round(t, 1),
        "atascos": n_atascos, "recuperaciones": n_recup, "con_leccion": n_leccion, "con_asistente": n_asist,
        "escaneos": n_escaneos, "dist_m": round(dist_rec, 2),
        "firma_final": AP.firma(sens(lidar.distancias(cfg.slam_cfg())), "atascado") if lidar.last_scan else None,
        "lecciones_nuevas": lecciones_nuevas, "juicios": juicios, "eventos": eventos,
    }
