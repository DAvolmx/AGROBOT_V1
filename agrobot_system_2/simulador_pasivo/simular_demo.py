#!/usr/bin/env python3
"""DEMO «plantas en maceta» en el SIMULADOR CENTRAL (2026-10-22), con el MISMO código que corre en el robot:
explorador.ExploradorReactivo + vision_activa.VisionActiva + mision_plantas.MisionPlantas + pulse_drive.ConductorPulsos + slam.LidarReactive,
sobre la física calibrada (sim_logica_actual: RobotModelo, firmware maxfrente, rampa del actuador, latencia del enlace).

Laboratorio: sala con paredes, mesa (4 patas), sillas (patas finas), 3 macetas con coliflor marchita. Las macetas (22 cm) quedan POR DEBAJO
del plano del LIDAR (37,5 cm): solo las hojas lo cortan a veces; el robot tiene que verlas con la CÁMARA (detector de macetas sintético con
la geometría real de la cámara y del servo) y con el suelo libre por cámara. Variantes de suelo: cambian la tasa de detección y los
falsos positivos de la cámara.

Corre en TIEMPO REAL (los módulos del robot usan asyncio y time.monotonic). Varias variantes en paralelo, un proceso cada una.
  python3 simular_demo.py [--variantes baldosa_clara,baldosa_oscura,vinilo_brillante] [--layouts 0,1] [--tiempo 420] [--procs 6]
Salida: ~/.agrobot/informes/demo_<fecha>/ (resumen.json, informe.md, trayectorias PNG).
"""
import argparse
import asyncio
import json
import math
import sys
import time
from dataclasses import replace
from multiprocessing import Pool
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import sim_logica_actual as S                         # noqa: E402  (añade jetson/backend al path)
from calibracion_real import CAL                      # noqa: E402
from robot_rustico import parametros_rusticos         # noqa: E402
import pulse_drive                                    # noqa: E402
import explorador as EXP                              # noqa: E402
import vision_activa as VA                            # noqa: E402
import mision_plantas as MP                           # noqa: E402
from slam import LidarReactive                        # noqa: E402
from anticolision import Anticolision                 # noqa: E402
import sendero as SEN                                 # noqa: E402

CFG_ROBOT = json.loads((AQUI.parent / "jetson" / "backend" / "agrobot_config.json").read_text())
ALTURA_LIDAR = 0.375
CAM_ALTURA, CAM_HFOV, CAM_TILT_HOME, SERVO_HOME = 0.40, 71.0, 15.5, 98.0
ANCHO, ALTO = 640, 480
VFOV = math.degrees(2 * math.atan(math.tan(math.radians(CAM_HFOV) / 2) * ALTO / ANCHO))
# Suelos: (prob. de detectar una maceta visible, falsos positivos por segundo, factor de ruido del LIDAR)
SUELOS = {"baldosa_clara": (0.92, 0.02, 1.0), "baldosa_oscura": (0.85, 0.05, 1.0), "vinilo_brillante": (0.80, 0.08, 1.3),
          "alfombra_estampada": (0.75, 0.12, 1.0)}


def laboratorio(layout: int):
    """Paredes (segmentos), patas (círculos visibles al LIDAR) y macetas (círculo de maceta + copa de hojas)."""
    W, H = 6.0, 5.0
    paredes = [(0, 0, W, 0), (W, 0, W, H), (W, H, 0, H), (0, H, 0, 0)]
    patas = []

    def mesa(cx, cy, lx=1.2, ly=0.7):
        for sx in (-1, 1):
            for sy in (-1, 1):
                patas.append((cx + sx * lx / 2, cy + sy * ly / 2, 0.025))

    def silla(cx, cy, l=0.42):
        for sx in (-1, 1):
            for sy in (-1, 1):
                patas.append((cx + sx * l / 2, cy + sy * l / 2, 0.015))
    if layout == 0:
        mesa(3.0, 2.6); silla(3.0, 1.75); silla(1.9, 2.6); silla(4.6, 3.6)
        macetas = [(1.2, 4.1), (4.9, 1.2), (5.0, 4.2)]
        inicio = (0.7, 0.8, 0.6)
    else:
        mesa(2.2, 3.6); mesa(4.3, 1.6, 1.0, 0.6); silla(2.2, 2.75); silla(3.4, 3.6); silla(4.3, 2.4)
        macetas = [(0.9, 2.2), (3.3, 0.8), (5.1, 3.9)]
        inicio = (0.6, 0.6, 0.5)
    return {"W": W, "H": H, "paredes": paredes, "patas": patas, "macetas": macetas, "inicio": inicio}


def rayos(mundo, lx, ly, psi, rng, ruido_k):
    """Scan de 360 rayos: paredes y patas siempre; copa de cada planta (r 0,20) solo a veces (hojas marchitas sueltas en el plano)."""
    ang = np.deg2rad(np.arange(360)) + psi
    dx, dy = np.cos(ang), np.sin(ang)
    dist = np.full(360, np.inf)
    circ = [(x, y, r) for (x, y, r) in mundo["patas"]] + [(x, y, 0.20) for (x, y) in mundo["macetas"]]
    hojas = np.zeros(360, bool)
    for k, (x, y, r) in enumerate(circ):
        fx, fy = lx - x, ly - y
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - r * r
        disc = b * b - c
        ok = disc >= 0
        t = np.where(ok, -b - np.sqrt(np.where(ok, disc, 0.0)), np.inf)
        t = np.where(t > 0.02, t, np.inf)
        if k >= len(mundo["patas"]):                      # hojas: el haz las corta en ~35 % de los rayos
            t = np.where(rng.random(360) < 0.35, t, np.inf)
        dist = np.minimum(dist, t)
    for (x0, y0, x1, y1) in mundo["paredes"]:
        ex, ey = x1 - x0, y1 - y0
        den = dx * ey - dy * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            tt = ((x0 - lx) * ey - (y0 - ly) * ex) / den
            uu = ((x0 - lx) * dy - (y0 - ly) * dx) / den
        okp = (np.abs(den) > 1e-9) & (tt > 0.02) & (uu >= 0) & (uu <= 1)
        dist = np.minimum(dist, np.where(okp, tt, np.inf))
    sigma = (CAL["lidar_sigma_a"] + CAL["lidar_sigma_b"] * np.where(np.isfinite(dist), dist, 0)) * CAL["lidar_factor_marcha"] * ruido_k
    dist = np.where(rng.random(360) < CAL["lidar_dropout"], np.inf, dist)
    dist = np.where(dist <= 8.0, dist + rng.normal(0, 1, 360) * sigma, np.inf)
    return [{"angle_deg": float(a if a <= 180 else a - 360), "distance_m": float(d)} for a, d in zip(range(360), dist) if np.isfinite(d)]


def choca(mundo, cx, cy, psi):
    """Huella 49 x 38 cm (alrededor del centro) contra patas, macetas (r 0,13) y paredes."""
    c, s = math.cos(psi), math.sin(psi)
    hl, hw = 0.245, 0.19
    for (x, y, r) in mundo["patas"] + [(mx, my, 0.13) for (mx, my) in mundo["macetas"]]:
        u, v = c * (x - cx) + s * (y - cy), -s * (x - cx) + c * (y - cy)
        if abs(u) <= hl + r and abs(v) <= hw + r and (abs(u) <= hl or abs(v) <= hw or math.hypot(abs(u) - hl, abs(v) - hw) <= r):
            return True
    for sx in (-1, 1):
        for sy in (-1, 1):
            px, py = cx + c * sx * hl - s * sy * hw, cy + s * sx * hl + c * sy * hw
            if not (0 < px < mundo["W"] and 0 < py < mundo["H"]):
                return True
    return False


class Demo:
    def __init__(self, layout, suelo, semilla, tiempo):
        self.mundo = laboratorio(layout)
        self.suelo = suelo
        self.p_det, self.fp_s, self.ruido_k = SUELOS[suelo]
        self.rng = np.random.default_rng(semilla)
        self.tiempo = tiempo
        hip = S.hip_por_semilla(semilla)
        params = replace(parametros_rusticos(hip.semilla_chasis), mu=hip.mu, front_drive_bias=hip.bias, rolling_drag=hip.drag,
                         substep=S.SUBSTEP, lateral=hip.lateral, front_outer=S.PIVOTE_EXT_M, rear_outer=hip.eje_fijo or S.FIJO_EXT_M)
        self.hip = hip
        self.robot = S.RobotModelo(params, self.mundo["inicio"])
        ctl, saf, aut = CFG_ROBOT["control"], CFG_ROBOT["safety"], CFG_ROBOT["autonomia"]
        self.scfg = S.Config(modo=ctl.get("modo_manejo", "preciso"), escala_velocidad=aut["escala_velocidad"],
                             max_v_mps=ctl["max_v_mps"], max_omega_rps=ctl["max_omega_rps"],
                             accel_lin=saf.get("max_linear_accel_mps2", 3), accel_ang=saf.get("max_angular_accel_rps2", 8))
        self.act = S.ActuadorJetson(self.scfg)
        self.fw = S.crear_firmware(self.scfg)
        self.fw.omega_gain = hip.giro_gain
        if hasattr(self.fw, "rng"):
            self.fw.rng = np.random.default_rng(20_000 + semilla)
        self.cola = []
        self.lidar = LidarReactive()
        self.servo = SERVO_HOME
        self.led = False
        self.grabando = False
        self.t0 = time.monotonic()
        self.tray, self.eventos = [], []
        self.contactos, self._choque_prev = 0, False
        self.led_s = 0.0
        self.cajas, self.cajas_ts = [], 0.0
        self.t_sim = 0.0
        cfg_ctl = lambda: CFG_ROBOT["control"]
        self.anticol = Anticolision(reloj=lambda: self.t_sim)
        self.conductor = pulse_drive.ConductorPulsos(send_command=self._enviar,
                                                     emergency_stop=lambda: self.act.emergency_stop(self.t_sim), get_config=cfg_ctl)
        self.conductor.modo = ctl.get("modo_manejo", "preciso")

    def _enviar(self, v, w, sc=0):
        """Como ActuatorController._aplicar del robot: la anticolisión con retroceso filtra cada orden antes del actuador."""
        v2, w2, ev, _ = self.anticol.filtrar(v, w, self.distancias(), self.lidar.edad_scan_s(), CFG_ROBOT["safety"])
        if ev:
            self.eventos.append({"t": round(self.t_sim, 1), "tipo": "anticolision", **{k: ev[k] for k in list(ev)[:3]}})
        self.act.send_command(v2, w2, self.t_sim)

    # ------------------- sensores sintéticos
    def pose(self):
        cx, cy = self.robot.centro()
        return cx, cy, self.robot.psi_real()

    def distancias(self):
        return self.lidar.distancias(CFG_ROBOT["slam"])

    def distancias_explorador(self):
        """Como _distancias_explorador del backend: LIDAR + lo que la cámara ve en el suelo (macetas bajas en el camino)."""
        d = dict(self.distancias())
        cx, cy, psi = self.pose()
        tilt = CAM_TILT_HOME + VA.GRADOS_POR_GRADO * (SERVO_HOME - self.servo)
        d_min_vista = CAM_ALTURA / math.tan(math.radians(tilt + VFOV / 2)) + 0.18
        for (mx, my) in self.mundo["macetas"]:
            u = math.cos(psi) * (mx - cx) + math.sin(psi) * (my - cy)
            v = -math.sin(psi) * (mx - cx) + math.cos(psi) * (my - cy)
            if u > 0 and abs(v) <= 0.19 + 0.13 + 0.05 and u - 0.13 >= d_min_vista - 0.245:
                cam = u - 0.13 - 0.245 + self.rng.normal(0, 0.04)
                if d.get("adelante") is None or cam < d["adelante"]:
                    d["adelante"] = max(0.0, cam)
        return d

    def actualizar_camara(self):
        cx, cy, psi = self.pose()
        tilt = CAM_TILT_HOME + VA.GRADOS_POR_GRADO * (SERVO_HOME - self.servo)
        f = (ANCHO / 2) / math.tan(math.radians(CAM_HFOV / 2))
        cajas = []
        for (mx, my) in self.mundo["macetas"]:
            dxr, dyr = mx - cx, my - cy
            u = math.cos(psi) * dxr + math.sin(psi) * dyr
            v = -math.sin(psi) * dxr + math.cos(psi) * dyr
            d = math.hypot(u, v)
            if u <= 0.05 or d < 0.30 or d > 3.2:
                continue
            yaw = -math.degrees(math.atan2(v, u))                   # + = a la derecha
            if abs(yaw) > CAM_HFOV / 2 - 3:
                continue
            dep_centro = math.degrees(math.atan2(CAM_ALTURA - 0.30, d))
            dep_base = math.degrees(math.atan2(CAM_ALTURA, d - 0.13))
            if abs(dep_centro - tilt) > VFOV / 2:
                continue
            p = self.p_det * (1.0 if d < 2.0 else max(0.4, 1.0 - 0.4 * (d - 2.0)))
            if self.rng.random() > p:
                continue
            xc = ANCHO / 2 + f * math.tan(math.radians(yaw))
            half = f * math.tan(math.atan2(0.20, d))
            ytop = ALTO / 2 + f * math.tan(math.radians(math.degrees(math.atan2(CAM_ALTURA - 0.55, d)) - tilt))
            ybot = ALTO / 2 + f * math.tan(math.radians(min(dep_base - tilt, VFOV / 2 - 0.5)))
            dist_cara = max(0.0, (d - 0.13 - 0.245) * (1 + self.rng.normal(0, 0.06)))
            cajas.append({"class": "maceta", "confidence": float(np.clip(0.55 + 0.4 * self.rng.random(), 0, 0.97)),
                          "xyxy": (int(xc - half), int(max(0, ytop)), int(xc + half), int(min(ALTO - 1, ybot))), "dist_m": round(dist_cara, 2)})
        if self.rng.random() < self.fp_s * 0.1:                      # falso positivo ocasional (a 0,1 s por actualización)
            x = self.rng.integers(50, ANCHO - 100)
            cajas.append({"class": "maceta", "confidence": 0.46, "xyxy": (int(x), 300, int(x) + 60, 380), "dist_m": 1.5})
        self.cajas, self.cajas_ts = cajas, time.time()

    def get_cajas(self):
        return self.cajas, self.cajas_ts, ANCHO, ALTO, CAM_HFOV

    def macetas(self):
        out = []
        for b in self.cajas:
            yaw, _ = VA.angulos_de_caja(b["xyxy"], ANCHO, ALTO, CAM_HFOV)
            out.append({"conf": b["confidence"], "yaw_deg": yaw, "dist_m": b.get("dist_m")})
        return out

    # ------------------- física en tiempo real
    async def fisica(self):
        dt = S.TICK_FW
        prox_jet = prox_scan = prox_cam = 0.0
        while self.t_sim < self.tiempo + 5:
            objetivo = time.monotonic() - self.t0
            while self.t_sim < objetivo:
                if self.t_sim >= prox_jet:
                    prox_jet += S.TICK_JETSON
                    self.act.tick(self.t_sim)
                    self.cola.append((self.t_sim + S.LATENCIA_ENLACE_S, S._pct(self.act.out_v, S.V_SCALE), S._pct(self.act.out_w, S.W_SCALE)))
                while self.cola and self.cola[0][0] <= self.t_sim:
                    _, pv, pw = self.cola.pop(0)
                    self.fw.recibir(pv, pw, self.t_sim)
                self.robot.step(dt, self.fw.tick(self.t_sim), self.hip.dead)
                self.t_sim += dt
                if self.t_sim >= prox_scan:
                    prox_scan += 0.1
                    lx, ly = self.robot.lidar_pos()
                    self.lidar.ingest_scan(rayos(self.mundo, lx, ly, self.robot.psi_real(), self.rng, self.ruido_k))
                if self.t_sim >= prox_cam:
                    prox_cam += 0.1
                    self.actualizar_camara()
                    cx, cy, psi = self.pose()
                    self.tray.append((round(self.t_sim, 2), round(cx, 3), round(cy, 3), round(psi, 3), int(self.led)))
                    ch = choca(self.mundo, cx, cy, psi)
                    if ch and not self._choque_prev:
                        self.contactos += 1
                        self.eventos.append({"t": round(self.t_sim, 1), "tipo": "contacto", "x": round(cx, 2), "y": round(cy, 2)})
                    self._choque_prev = ch
                    if self.led:
                        self.led_s += 0.1
            await asyncio.sleep(0.005)

    async def correr(self):
        cfg_aut = dict(CFG_ROBOT["autonomia"])
        cfg_aut["escaneo_periodo_s"] = 9999          # el barrido de cámara del explorador no aporta en la simulación
        ev = lambda e: self.eventos.append({"t": round(self.t_sim, 1), **e})

        async def escanear(motivo):
            await asyncio.sleep(1.0)
        self.expl = EXP.ExploradorReactivo(
            distancias=self.distancias_explorador, get_pose=lambda: {"x": self.pose()[0], "y": self.pose()[1], "theta_deg": math.degrees(self.pose()[2]) % 360},
            conducir=lambda a, m=True, esc_dur=1.0: self.conductor.ordenar(a, v_max=CFG_ROBOT["control"]["max_v_mps"] * cfg_aut["escala_velocidad"],
                                                                          omega_max=CFG_ROBOT["control"]["max_omega_rps"] * cfg_aut["escala_velocidad"],
                                                                          maniobra=m, esc_dur=esc_dur),
            parar=self.conductor.parar, escanear=escanear, on_event=lambda e: None, grabar_recorrido=lambda a: (True, ""),
            edad_scan=self.lidar.edad_scan_s, get_config=lambda: cfg_aut,
            seguidor=SEN.SeguidorSendero(lambda: cfg_aut, obtener_frame=lambda: None))   # como en el robot: en interior corrige con el LIDAR
        vcfg = {**CFG_ROBOT.get("vision_activa", {}), "clases_objetivo": ["maceta"], "max_objetivos": 1, "conf_min": 0.40}

        def poner_servo(a):
            self.servo = max(60.0, min(125.0, float(a)))
        self.vision = VA.VisionActiva(
            get_cajas=self.get_cajas, get_servo=lambda: self.servo, set_servo=poner_servo,
            conducir=lambda a: self.conductor.ordenar(a, v_max=CFG_ROBOT["control"]["max_v_mps"] * cfg_aut["escala_velocidad"],
                                                      omega_max=CFG_ROBOT["control"]["max_omega_rps"], maniobra=False),
            ocupado=lambda: self.conductor.activo, parar=self.conductor.parar, distancias=self.distancias, edad_scan=self.lidar.edad_scan_s,
            get_config=lambda: vcfg, on_event=lambda e: ev({"origen": "vision", **{k: v for k, v in e.items() if k in ("tipo", "estado", "motivo", "clase")}}),
            capturar=lambda info: f"foto_{info.get('vista', 'frente')}", autonomia_activa=lambda: self.expl.activo, carpeta=Path("/tmp"),
            get_pose=lambda: {"x": self.pose()[0], "y": self.pose()[1], "theta_deg": math.degrees(self.pose()[2])})

        def led(on):
            self.led = on
            ev({"tipo": "led", "on": on})

        def grabar(on):
            self.grabando = on
            ev({"tipo": "video", "on": on})
        self.mision = MP.MisionPlantas(explorador=self.expl, vision=self.vision, get_macetas=self.macetas,
                                       get_pose=lambda: {"x": self.pose()[0], "y": self.pose()[1], "theta_deg": math.degrees(self.pose()[2])},
                                       led=led, grabar=grabar, get_config=lambda: {"tiempo_max_s": self.tiempo, **CFG_ROBOT.get("mision_plantas", {})},
                                       on_event=ev)
        fis = asyncio.create_task(self.fisica())
        await asyncio.sleep(1.0)                       # primer scan
        ok, msg = self.mision.iniciar()
        if not ok:
            ev({"tipo": "error", "msg": msg})
        else:
            self.mision.arrancar_tarea()
        while self.mision.estado not in ("fin", "inactiva") and self.t_sim < self.tiempo:
            await asyncio.sleep(0.5)
        self.mision.detener("fin de la simulación") if self.mision.estado not in ("fin", "inactiva") else None
        self.conductor.parar()
        await asyncio.sleep(0.5)
        fis.cancel()
        return self.resumen()

    def resumen(self):
        insp = [e for e in self.eventos if e.get("tipo") == "planta_inspeccionada"]
        reales = self.mundo["macetas"]
        acertadas = set()
        for e in insp:
            if e.get("x") is not None:
                d = [math.hypot(e["x"] - mx, e["y"] - my) for mx, my in reales]
                if min(d) < 0.8:
                    acertadas.add(int(np.argmin(d)))
        tr = np.array([(x, y) for _, x, y, _, _ in self.tray]) if self.tray else np.zeros((0, 2))
        celdas = {(int(x / 0.25), int(y / 0.25)) for x, y in tr}
        return {"suelo": self.suelo, "plantas_inspeccionadas": len(insp), "plantas_distintas_reales": len(acertadas),
                "vistas_por_planta": [e.get("vistas") for e in insp], "contactos": self.contactos,
                "videos": sum(1 for e in self.eventos if e.get("tipo") == "video" and e.get("on")),
                "led_encendido_s": round(self.led_s, 1), "tiempo_s": round(self.t_sim, 1), "cobertura_m2": round(len(celdas) * 0.0625, 1),
                "motivo_fin": self.mision.motivo_fin, "eventos": self.eventos[-60:], "tray": self.tray[::5]}


AJUSTES = {}


def una(args):
    layout, suelo, semilla, tiempo, ajustes = args
    for sec, kv in ajustes.items():
        CFG_ROBOT.setdefault(sec, {}).update(kv)
    d = Demo(layout, suelo, semilla, tiempo)
    r = asyncio.run(d.correr())
    r.update(layout=layout, semilla=semilla, ajustes=ajustes)
    return r


def dibujar(r, mundo, ruta):
    import cv2
    k = 100
    img = np.full((int(mundo["H"] * k) + 20, int(mundo["W"] * k) + 20, 3), 255, np.uint8)
    P = lambda x, y: (int(x * k) + 10, int((mundo["H"] - y) * k) + 10)
    cv2.rectangle(img, P(0, mundo["H"]), P(mundo["W"], 0), (0, 0, 0), 2)
    for (x, y, rr) in mundo["patas"]:
        cv2.circle(img, P(x, y), max(2, int(rr * k)), (60, 60, 60), -1)
    for (x, y) in mundo["macetas"]:
        cv2.circle(img, P(x, y), int(0.13 * k), (30, 70, 120), -1)
        cv2.circle(img, P(x, y), int(0.20 * k), (40, 160, 40), 1)
    for (_, x, y, _, led) in r["tray"]:
        cv2.circle(img, P(x, y), 2, (0, 200, 255) if led else (200, 80, 0), -1)
    for e in r["eventos"]:
        if e.get("tipo") == "contacto":
            cv2.drawMarker(img, P(e["x"], e["y"]), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
    cv2.putText(img, f"{r['suelo']} L{r['layout']}: {r['plantas_distintas_reales']}/3 plantas, {r['contactos']} contactos", (12, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.imwrite(str(ruta), img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variantes", default="baldosa_clara,baldosa_oscura,vinilo_brillante")
    ap.add_argument("--layouts", default="0,1")
    ap.add_argument("--semillas", default="1")
    ap.add_argument("--tiempo", type=float, default=420)
    ap.add_argument("--procs", type=int, default=6)
    ap.add_argument("--ajustes", default="", help='JSON con variantes de config, p. ej. {"duty60":{"control":{"suavidad_duty":0.6}}}')
    a = ap.parse_args()
    carpeta = Path.home() / ".agrobot" / "informes" / f"demo_{time.strftime('%Y%m%d_%H%M')}"
    carpeta.mkdir(parents=True, exist_ok=True)
    variantes_cfg = json.loads(a.ajustes) if a.ajustes else {"robot": {}}
    trabajos = [(int(l), v, int(s), a.tiempo, aj) for l in a.layouts.split(",") for v in a.variantes.split(",") for s in a.semillas.split(",")
                for aj in variantes_cfg.values()]
    nombres = {json.dumps(aj, sort_keys=True): n for n, aj in variantes_cfg.items()}
    print(f"[demo] {len(trabajos)} simulaciones en tiempo real (~{a.tiempo / 60:.0f} min) -> {carpeta}", flush=True)
    with Pool(min(a.procs, len(trabajos))) as p:
        res = p.map(una, trabajos)
    filas = ["| variante | layout | suelo | plantas escaneadas (reales) | vistas | contactos | videos | LED s | cobertura m² | tiempo s | fin |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in res:
        nv = nombres.get(json.dumps(r["ajustes"], sort_keys=True), "?")
        dibujar(r, laboratorio(r["layout"]), carpeta / f"tray_{nv}_L{r['layout']}_{r['suelo']}_s{r['semilla']}.png")
        filas.append(f"| {nv} | {r['layout']} | {r['suelo']} | {r['plantas_inspeccionadas']} ({r['plantas_distintas_reales']}) | {r['vistas_por_planta']} | "
                     f"{r['contactos']} | {r['videos']} | {r['led_encendido_s']} | {r['cobertura_m2']} | {r['tiempo_s']} | {r['motivo_fin']} |")
    (carpeta / "resumen.json").write_text(json.dumps(res, indent=1, ensure_ascii=False, default=str))
    (carpeta / "informe.md").write_text("# Demo plantas en maceta (simulador, código del robot)\n\n" + "\n".join(filas) + "\n")
    print("\n".join(filas), flush=True)


if __name__ == "__main__":
    main()
