#!/usr/bin/env python3
"""
Simulador del AgroBot: corre el CÓDIGO REAL del robot contra un mundo virtual.

    python3 simulador/correr.py --escenario bucle --segundos 120
    python3 simulador/correr.py --escenario trampa_bucle --video salida.mp4
    python3 simulador/correr.py --todos --segundos 90 --salida resultados/

No se simula el software: se ejecutan los módulos de verdad
(`slam.LidarReactive`, `sendero.SeguidorSendero`, `explorador.ExploradorReactivo`,
`pulse_drive.ConductorPulsos`), así que lo que aquí se arregla o se ajusta es
lo mismo que corre en la Jetson. El mundo entrega scans del LIDAR e imágenes
de cámara sintéticas (`mundo.py`) y el chasis responde con sus vicios reales
(`robot.py`).

El reloj es VIRTUAL: el episodio avanza tan rápido como dé la CPU, sin
esperar en tiempo real. Todo `time.monotonic()` y `asyncio.sleep()` del
código bajo prueba pasa por él.
"""
import argparse
import asyncio
import csv
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
sys.path.insert(0, str(AQUI.parent / "jetson" / "backend"))

import mundo as mundo_mod                                   # noqa: E402
from robot import RobotSim                                  # noqa: E402

DT = 0.02                     # paso de física (s de simulación)
LIDAR_PERIODO_S = 0.1         # 10 Hz, como el RPLIDAR C1
SERVO_SENDERO_DEG = 62        # inclinación del servo con la que mejor se ve el sendero
SERVO_GRADOS_POR_GRADO = 0.8  # 1° de servo ≈ 0,8° de inclinación (medido 2026-09-19)
REVISITA_S = 15.0             # volver a una celda después de esto cuenta como revisita
REVISITAS_BUCLE = 3           # tantas revisitas de la misma celda = está dando vueltas


class RelojVirtual:
    """Tiempo simulado. Sustituye a time.monotonic/time.time y asyncio.sleep
    para que un episodio de minutos corra en segundos."""

    def __init__(self):
        self.t = 0.0
        self._base = time.time()
        self._sleep = asyncio.sleep
        self._monotonic, self._time = time.monotonic, time.time

    def instalar(self):
        time.monotonic = lambda: self.t
        time.time = lambda: self._base + self.t
        asyncio.sleep = self._dormir

    def quitar(self):
        time.monotonic, time.time = self._monotonic, self._time
        asyncio.sleep = self._sleep

    async def _dormir(self, segundos, result=None):
        if segundos <= 0:
            return await self._sleep(0, result)
        objetivo = self.t + segundos
        while self.t < objetivo:
            await self._sleep(0)
        return result


class Episodio:
    def __init__(self, escenario, cfg, segundos, semilla=0, video=None):
        crear = mundo_mod.ESCENARIOS[escenario]
        self.mundo, pose = crear()
        self.cfg = cfg
        self.segundos = segundos
        self.rng = np.random.default_rng(semilla)
        self.robot = RobotSim(self.mundo, pose, cfg, self.rng)
        self.reloj = RelojVirtual()
        self.video = video
        self.eventos = []
        self.filas = []
        self.escaneos = 0
        self.plagas_vistas = set()
        self.t_en_sendero = 0.0
        self.t_total = 0.0
        self.visitas = {}
        self.bucles = 0
        self._ultimo_lidar = -1.0
        self._ultimo_log = -1.0
        self._seq = 0
        self._escribe = None

    # ------------------------------------------------------------- geometría
    def geometria(self):
        g = dict(self.cfg["odometria_webcam"])
        g["camara_inclinacion_deg"] = (g["camara_inclinacion_deg"]
                                       + SERVO_GRADOS_POR_GRADO * (98 - SERVO_SENDERO_DEG))
        return g

    # --------------------------------------------------------------- montaje
    def montar(self):
        import slam
        import sendero
        from pulse_drive import ConductorPulsos
        from explorador import ExploradorReactivo

        self.lidar = slam.LidarReactive()
        self.seguidor = sendero.SeguidorSendero(
            get_config=lambda: self.cfg["autonomia"],
            obtener_frame=self._frame,
            camara_al_frente=lambda: (True, ""),
            get_geometria=self.geometria)

        def mandar(v, omega, scan=0):
            # Mismo orden que el backend real: el freno por LIDAR se aplica
            # DESPUÉS de la orden y antes de mover el chasis.
            v_seg, aviso = self.lidar.limitar_velocidad(v, self.cfg["slam"])
            if aviso is not None and v_seg != v:
                self.eventos.append({"t": round(self.reloj.t, 1), "evento": "freno_lidar", **aviso})
            self.robot.ordenar(v_seg, omega)

        self.conductor = ConductorPulsos(
            send_command=mandar,
            emergency_stop=lambda: self.robot.ordenar(0.0, 0.0),
            get_config=lambda: self.cfg["control"])

        ctrl = self.cfg["control"]
        escala = self.cfg["autonomia"]["escala_velocidad"]
        self.explorador = ExploradorReactivo(
            distancias=lambda: self.lidar.distancias(self.cfg["slam"]),
            get_pose=self.robot.pose,
            conducir=lambda accion: self.conductor.ordenar(
                accion, v_max=ctrl["max_v_mps"] * escala,
                omega_max=ctrl["max_omega_rps"] * escala, maniobra=True),
            parar=self.conductor.parar,
            escanear=self._escanear,
            on_event=lambda d: self.eventos.append({"t": round(self.reloj.t, 1), **d}),
            grabar_recorrido=lambda activa: (True, ""),
            edad_scan=self.lidar.edad_scan_s,
            get_config=lambda: self.cfg["autonomia"],
            seguidor=self.seguidor)

    def _frame(self):
        self._seq += 1
        return self.robot.frame(self.geometria()), self.reloj.t, self._seq

    async def _escanear(self, motivo):
        """Barrido de cámara: el robot se para y mira las plantas cercanas."""
        self.robot.ordenar(0.0, 0.0)
        await asyncio.sleep(self.cfg["autonomia"]["escaneo_duracion_s"])
        self.escaneos += 1
        for i, (px, py, _, plaga) in enumerate(self.mundo.plantas):
            if math.dist((px, py), (self.robot.x, self.robot.y)) < 1.2 and plaga:
                self.plagas_vistas.add(i)
        self.eventos.append({"t": round(self.reloj.t, 1), "evento": "escaneo", "motivo": motivo})

    # ----------------------------------------------------------------- física
    async def _fisica(self):
        marco = 0
        while self.reloj.t < self.segundos:
            self.robot.paso(DT)
            self.reloj.t += DT
            self.t_total += DT
            if self.robot.desvio_del_sendero() <= self.mundo.ancho_sendero / 2:
                self.t_en_sendero += DT

            if self.reloj.t - self._ultimo_lidar >= LIDAR_PERIODO_S:
                self._ultimo_lidar = self.reloj.t
                self.lidar.ingest_scan(self.robot.scan())
                self._contar_visita()

            if self.reloj.t - self._ultimo_log >= 0.5:
                self._ultimo_log = self.reloj.t
                self._registrar()
                if self.video is not None:
                    marco += 1
                    if marco % 2 == 0:
                        self._grabar_video()
            await self.reloj._sleep(0)

    def _contar_visita(self):
        """Bucle = VOLVER a una celda de 50 cm de la que ya se había ido hace
        rato. Contar el tiempo dentro de la celda no vale: avanzando despacio
        el robot está segundos en la misma sin estar dando vueltas."""
        celda = (round(self.robot.x * 2), round(self.robot.y * 2))
        veces, ultimo = self.visitas.get(celda, (0, -1e9))
        if self.reloj.t - ultimo > REVISITA_S:
            veces += 1
            if veces >= REVISITAS_BUCLE:
                self.bucles += 1
                self.eventos.append({"t": round(self.reloj.t, 1), "evento": "bucle_detectado",
                                     "celda": celda, "visitas": veces})
        self.visitas[celda] = (veces, self.reloj.t)

    def _registrar(self):
        d = self.lidar.distancias(self.cfg["slam"])
        cm = lambda v: "" if v is None else round(v * 100)
        self.filas.append({
            "timestamp": round(self.reloj.t, 2), "x": round(self.robot.x, 4),
            "y": round(self.robot.y, 4), "theta_deg": round(math.degrees(self.robot.theta) % 360, 2),
            "vx_desired": round(self.robot.v_obj, 3), "omega_desired": round(self.robot.omega_obj, 3),
            "vx_real": round(self.robot.v, 3), "omega_real": round(self.robot.omega, 3),
            "lidar_frontal_cm": cm(d["adelante"]), "lidar_lateral_izq_cm": cm(d["izquierda"]),
            "lidar_lateral_der_cm": cm(d["derecha"]),
            "desvio_sendero_cm": round(self.robot.desvio_del_sendero() * 100, 1),
            "estado": self.explorador.estado,
        })

    # ------------------------------------------------------------------ vídeo
    def _grabar_video(self):
        arriba = self.dibujo_trayectoria(traza=False)
        cam = cv2.resize(self.robot.frame(self.geometria()), (320, 240))
        alto = max(arriba.shape[0], 240)
        izq = cv2.copyMakeBorder(arriba, 0, alto - arriba.shape[0], 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))
        der = cv2.copyMakeBorder(cam, 0, alto - 240, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))
        marco = np.hstack([izq, der])
        if self._escribe is None:
            self._escribe = cv2.VideoWriter(str(self.video), cv2.VideoWriter_fourcc(*"mp4v"),
                                            10, (marco.shape[1], marco.shape[0]))
        self._escribe.write(marco)

    def dibujo_trayectoria(self, traza=True):
        img = self.mundo.dibujo().copy()
        for x1, y1, x2, y2 in self.mundo.paredes:
            cv2.line(img, self.mundo.a_pixel(x1, y1), self.mundo.a_pixel(x2, y2), (60, 60, 60), 3)
        if traza and len(self.filas) > 1:
            pts = np.array([self.mundo.a_pixel(f["x"], f["y"]) for f in self.filas], np.int32)
            cv2.polylines(img, [pts], False, (200, 60, 200), 2)
        px, py = self.mundo.a_pixel(self.robot.x, self.robot.y)
        punta = self.mundo.a_pixel(self.robot.x + 0.35 * math.cos(self.robot.theta),
                                   self.robot.y + 0.35 * math.sin(self.robot.theta))
        cv2.circle(img, (px, py), 9, (40, 190, 60), -1)
        cv2.line(img, (px, py), punta, (255, 255, 255), 3)
        return img

    # ------------------------------------------------------------------ correr
    async def _correr(self):
        self.montar()
        self.reloj.instalar()
        try:
            # El explorador exige un scan fresco para arrancar (sin LIDAR el
            # robot iría a ciegas), así que se le da el primero a mano.
            self.lidar.ingest_scan(self.robot.scan())
            ok, msg = self.explorador.iniciar()
            if not ok:
                raise RuntimeError(f"no arrancó la exploración: {msg}")
            self.eventos.append({"t": 0.0, "evento": "inicio", "detalle": msg})
            await self._fisica()
            self.explorador.detener()
            await self.reloj._sleep(0)
        finally:
            self.reloj.quitar()
            if self._escribe is not None:
                self._escribe.release()

    def correr(self) -> dict:
        t0 = time.perf_counter()
        asyncio.run(self._correr())
        real = time.perf_counter() - t0
        plagas = int((self.mundo.plantas[:, 3] > 0).sum()) if len(self.mundo.plantas) else 0
        return {
            "escenario": self.mundo.nombre,
            "segundos_simulados": round(self.t_total, 1),
            "segundos_reales": round(real, 1),
            "aceleracion": round(self.t_total / max(real, 1e-6), 1),
            "recorrido_m": round(self.robot.recorrido, 2),
            "colisiones": self.robot.colisiones,
            "contacto_s": round(self.robot.contacto_s, 1),
            "en_sendero_pct": round(100 * self.t_en_sendero / max(self.t_total, 1e-6), 1),
            "desvio_medio_cm": round(float(np.mean([f["desvio_sendero_cm"] for f in self.filas])), 1) if self.filas else None,
            "pulsos": self.explorador.pulsos, "giros": self.explorador.giros,
            "correcciones_sendero": self.explorador.correcciones,
            "desatascos": self.explorador.desatascos,
            "escaneos": self.escaneos,
            "plagas_detectadas": f"{len(self.plagas_vistas)}/{plagas}",
            "bucles_detectados": self.bucles,
            "celdas_visitadas": len(self.visitas),
            "revisitas_max": max((v for v, _ in self.visitas.values()), default=0),
            "estado_final": self.explorador.estado,
        }


def guardar(ep: Episodio, res: dict, salida: Path, nombre: str):
    salida.mkdir(parents=True, exist_ok=True)
    with (salida / f"{nombre}.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(ep.filas[0].keys()))
        w.writeheader(); w.writerows(ep.filas)
    (salida / f"{nombre}_eventos.json").write_text(json.dumps(ep.eventos, indent=1, ensure_ascii=False))
    (salida / f"{nombre}_resumen.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
    cv2.imwrite(str(salida / f"{nombre}.png"), ep.dibujo_trayectoria())


def main():
    p = argparse.ArgumentParser(description="Simulador del AgroBot")
    p.add_argument("--escenario", default="bucle", choices=list(mundo_mod.ESCENARIOS))
    p.add_argument("--todos", action="store_true", help="corre todos los escenarios")
    p.add_argument("--segundos", type=float, default=120)
    p.add_argument("--semilla", type=int, default=0)
    p.add_argument("--config", default=str(AQUI.parent / "jetson" / "backend" / "agrobot_config.json"))
    p.add_argument("--salida", default=str(AQUI / "resultados"))
    p.add_argument("--video", default=None, help="archivo .mp4 (solo con un escenario)")
    a = p.parse_args()

    cfg = json.loads(Path(a.config).read_text())
    cfg["autonomia"].setdefault("sendero_paja", True)
    cfg["autonomia"]["sendero_activo"] = True
    salida = Path(a.salida)
    escenarios = list(mundo_mod.ESCENARIOS) if a.todos else [a.escenario]

    resumenes = []
    for nombre in escenarios:
        ep = Episodio(nombre, cfg, a.segundos, a.semilla, a.video if not a.todos else None)
        res = ep.correr()
        resumenes.append(res)
        guardar(ep, res, salida, nombre)
        print(json.dumps(res, ensure_ascii=False))
    (salida / "resumen.json").write_text(json.dumps(resumenes, indent=1, ensure_ascii=False))
    print(f"-> {salida}")


if __name__ == "__main__":
    main()
