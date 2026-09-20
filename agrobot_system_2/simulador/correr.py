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
from aprendizaje import Aprendizaje                         # noqa: E402
import ruta_ideal                                           # noqa: E402
from slam_sim import SlamSimulado                           # noqa: E402

DT = 0.02                     # paso de física (s de simulación)
# Tras una corrección del operador, la autonomía espera en silencio: si no,
# le quitaría el mando a mitad de la maniobra que le están enseñando.
SILENCIO_TRAS_CORRECCION_S = 2.5
# Escaneo: hasta dónde se reconoce una planta y con cuántos píxeles mínimos.
DISTANCIA_ESCANEO_MAX_M = 2.5
PIXELES_MINIMOS_PLANTA = 18
ANCHO_CAMARA = 320
# A qué distancia de un punto marcado el robot se para a escanearlo.
RADIO_PUNTO_ESCANEO_M = 0.55
LIDAR_PERIODO_S = 0.1         # 10 Hz, como el RPLIDAR C1
# Inclinación de partida. 72° de servo ≈ 36° bajo el horizonte: medido como
# el mejor compromiso en el simulador (100 % en sendero frente al 39 % a 62°).
SERVO_SENDERO_DEG = 72
# Ángulos que prueba el buscador cuando pierde el sendero. Barren de mirar
# casi al horizonte (98°) a mirar el suelo cerca (54°).
SERVO_CANDIDATOS = (72, 62, 54, 84, 98)
BUSQUEDA_SERVO_S = 1.2        # cuánto mira en cada ángulo antes de juzgarlo
SIN_SENDERO_PARA_BUSCAR_S = 6.0
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
    """Un episodio de simulación.

    `tiempo_real=False` (por defecto) usa el reloj virtual y corre tan rápido
    como dé la CPU. `tiempo_real=True` lo hace al ritmo del reloj de pared
    (por `factor`), que es lo que necesita el dashboard para verse y para
    dejar manejar a una persona."""

    def __init__(self, escenario, cfg, segundos, semilla=0, video=None,
                 tiempo_real=False, factor=1.0, ruta_aprendizaje=None,
                 ruido_slam=1.0, usar_slam=True):
        # `escenario` es el nombre de uno de los de siempre, o ya un
        # (mundo, pose) — así el dashboard puede correr mapas dibujados a
        # mano o importados del robot real.
        if isinstance(escenario, tuple):
            self.mundo, pose = escenario
        elif escenario == "aleatorio":
            self.mundo, pose = mundo_mod.aleatorio(semilla)
        else:
            self.mundo, pose = mundo_mod.ESCENARIOS[escenario]()
        self.cfg = cfg
        self.segundos = segundos
        self.rng = np.random.default_rng(semilla)
        self.robot = RobotSim(self.mundo, pose, cfg, self.rng)
        self.parar_pedido = False
        self.reloj = RelojVirtual()
        self.video = video
        self.eventos = []
        self.filas = []
        self.escaneos = 0
        self.plagas_vistas = set()
        self.escaneadas = set()            # plantas efectivamente miradas de cerca
        self.puntos_escaneados = set()     # puntos marcados ya cubiertos
        self.t_escaneando = 0.0
        self.foco = False
        self._escaneo_en_curso = None
        self.t_en_sendero = 0.0
        self.t_total = 0.0
        self.visitas = {}
        self.bucles = 0
        self._ultimo_lidar = -1.0
        self._ultimo_log = -1.0
        self._seq = 0
        self._escribe = None
        self.tiempo_real = tiempo_real
        self.factor = factor
        self.modo = "auto"
        self.traza = []
        self.muestras = []          # (sensores, acción) del operador, para aprender
        self._ultima_traza = -1.0
        self._ultimo_piloto = -1.0
        # Aprendizaje por correcciones. El mismo fichero lo usa el robot real.
        self.aprendizaje = Aprendizaje(ruta_aprendizaje) if ruta_aprendizaje else Aprendizaje()
        self.usar_aprendido = True
        self.intervenciones = 0
        self._pose_al_aplicar = None
        self.busquedas_servo = 0
        self.calibrar_servo = True
        # --- SLAM simulado: la pose que el robot CREE tener, y su mapa ---
        self.slam = SlamSimulado((self.robot.x, self.robot.y, self.robot.theta),
                                 self.mundo.limites, semilla=semilla, ruido=ruido_slam)
        self.usar_slam = usar_slam        # si el software navega con la pose estimada
        self.traza_slam = []
        # --- Ruta ideal por el centro del sendero ---
        self.ruta = ruta_ideal.trazar(self.mundo.sendero)
        self.piloto = None                # activo durante el "recorrido ideal"
        self.filtro_ideal = False         # corrige la conducción hacia la ruta
        self.modo_recorrido = None        # "ideal" mientras lo hace
        self.pose_inicial = (self.robot.x, self.robot.y, self.robot.theta)
        self.registro_ideal = []
        self.comparacion = None
        self.desvio_ruta_suma = 0.0
        self.desvio_ruta_n = 0
        self.correcciones_filtro = 0
        self.resultado_ideal = None
        self.politica = None            # se le asigna una Politica para usarla
        self._sendero_visto_en = 0.0
        self._buscando_servo = None

    # ------------------------------------------------------------- geometría
    def geometria(self):
        """La geometría de la cámara sale del ángulo REAL del servo.

        Si no, el detector creería mirar a un sitio y estaría mirando a otro:
        el mismo error que habría tenido el robot real al mover el servo."""
        g = dict(self.cfg["odometria_webcam"])
        incl = self.robot.inclinacion_camara
        # El chasis cabecea en los baches y la cámara mira a otro sitio. Con
        # el IMU se sabe cuánto y se corrige; sin él, el detector cree estar
        # mirando a 1 m cuando mira a 1,2 y el sendero le sale desplazado.
        if self.cfg["autonomia"].get("compensar_cabeceo", True):
            incl += self.robot.cabeceo_medido()
        return {**g, "camara_inclinacion_deg": incl}

    async def _buscar_inclinacion(self):
        """Mover el servo hasta encontrar desde dónde SE VE el sendero.

        En el terreno real (2026-09-19) con la cámara casi horizontal media
        imagen es pared o cielo y el sendero no se distingue; mirando más
        abajo se ve de 0,5 a 1,5 m por delante y aparece. Esto lo busca solo,
        probando ángulos y quedándose con el de más confianza."""
        mejor = (-1.0, self.robot.servo)
        for ang in SERVO_CANDIDATOS:
            if self.parar_pedido:
                return
            self.robot.mover_servo(ang)
            t0 = self.reloj.t
            while self.reloj.t - t0 < BUSQUEDA_SERVO_S and not self.parar_pedido:
                await self.reloj._sleep(0.01)
            if not self.robot.servo_asentado:
                continue
            # Se puntúa con TRES cosas, no solo la confianza: esa satura
            # (0,96 desde cualquier ángulo) y elegiría casi al azar.
            #   · ver el sendero                    (si no, 0)
            #   · ver los DOS bordes, no uno        (el ancho sale medido)
            #   · tener evidencia suficiente        (celdas de borde en la vista)
            # Varias tomas: un ángulo desde el que la medida BAILA es malo,
            # aunque cada toma suelta parezca buena.
            medidas, confs, celdas = [], [], 0
            for k in range(3):
                frame = self.robot.frame(self.geometria())
                res = self.seguidor.vision.detectar(cv2, frame, self.cfg["autonomia"],
                                                    self.geometria())
                celdas = int((res["cenital"] > 0).sum())
                if res["error_m"] is not None:
                    medidas.append(res["lateral_m"])
                    confs.append(res["confianza"] * (1.0 if res["ancho_m"] is not None else 0.45))
                if k < 2:
                    tk = self.reloj.t
                    while self.reloj.t - tk < 0.3 and not self.parar_pedido:
                        await self.reloj._sleep(0.005)
            conf = 0.0
            if len(medidas) == 3:
                inestable = float(np.std(medidas))
                conf = (float(np.mean(confs)) * min(1.0, celdas / 400.0)
                        * max(0.0, 1.0 - inestable / 0.15))
            self.eventos.append({"t": round(self.reloj.t, 1), "evento": "servo_probado",
                                 "servo": ang, "inclinacion": round(self.robot.inclinacion_camara, 1),
                                 "nota": round(conf, 2), "celdas": celdas,
                                 "estado": res["estado"][:60]})
            if conf > mejor[0]:
                mejor = (conf, ang)
        self.robot.mover_servo(mejor[1])
        self.busquedas_servo += 1
        self.eventos.append({"t": round(self.reloj.t, 1), "evento": "servo_elegido",
                             "servo": mejor[1], "confianza": round(mejor[0], 2)})

    # --------------------------------------------------------------- montaje
    def montar(self):
        self.robot.mover_servo(SERVO_SENDERO_DEG)
        import slam
        import sendero
        from pulse_drive import ConductorPulsos
        from explorador import ExploradorReactivo

        self.lidar = slam.LidarReactive()
        self.seguidor = sendero.SeguidorSendero(
            get_config=lambda: self.cfg["autonomia"],
            obtener_frame=self._frame,
            camara_al_frente=lambda: ((True, "") if self.robot.servo_asentado
                                      else (False, "servo moviéndose")),
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
            get_pose=self.pose_para_el_software,
            conducir=lambda accion, maniobra=True: self._conducir_filtrado(accion, maniobra),
            parar=self.conductor.parar,
            escanear=self._escanear,
            on_event=lambda d: self.eventos.append({"t": round(self.reloj.t, 1), **d}),
            grabar_recorrido=lambda activa: (True, ""),
            edad_scan=self.lidar.edad_scan_s,
            get_config=lambda: self.cfg["autonomia"],
            seguidor=self.seguidor,
            aprendizaje=self.aprendizaje if self.usar_aprendido else None,
            sensores=self.sensores,
            politica=self.politica)

    def _conducir_filtrado(self, accion, maniobra=True):
        ctrl, escala = self.cfg["control"], self.cfg["autonomia"]["escala_velocidad"]
        accion, maniobra = self.filtrar(accion, maniobra)
        self.conductor.ordenar(accion, v_max=ctrl["max_v_mps"] * escala,
                               omega_max=ctrl["max_omega_rps"] * escala, maniobra=maniobra)

    def pose_para_el_software(self) -> dict:
        """Lo que el robot cree. Con `usar_slam=False` se le da la verdad,
        que sirve para separar "falla la navegación" de "falla el SLAM"."""
        # `slam.pose` es propiedad y `robot.pose` es método: sin los
        # paréntesis se devolvía la función y el explorador reventaba.
        return self.slam.pose if self.usar_slam else self.robot.pose()

    def _frame(self):
        self._seq += 1
        return self.robot.frame(self.geometria()), self.reloj.t, self._seq

    async def _escanear(self, motivo):
        """Barrido de escaneo, como el del robot: parar, foco y barrer el servo.

        El robot no "ve" una planta por estar cerca: la ve si cae dentro del
        campo de la cámara CON la inclinación que tiene el servo en ese
        instante, y si está lo bastante cerca para distinguirla. Por eso el
        barrido recorre de `escaneo_angulo_min` a `escaneo_angulo_max`: una
        sola inclinación deja fuera lo que tiene justo delante o lo que está
        a dos metros."""
        cfg = self.cfg["autonomia"]
        self.robot.ordenar(0.0, 0.0)
        self.foco = True
        a0, a1 = int(cfg["escaneo_angulo_min"]), int(cfg["escaneo_angulo_max"])
        dur = max(0.5, float(cfg["escaneo_duracion_s"]))
        pasos = max(3, int(dur / 0.6))
        servo_previo = self.robot.servo_objetivo
        vistas, plagas = set(), set()
        t_ini = self.reloj.t
        for k in range(pasos + 1):
            if self.parar_pedido:
                break
            self.robot.mover_servo(a0 + (a1 - a0) * k / pasos)
            await asyncio.sleep(dur / (pasos + 1))
            v, p = self._plantas_a_la_vista()
            vistas |= v
            plagas |= p
        self.robot.mover_servo(servo_previo)
        self.foco = False
        self.escaneos += 1
        self.escaneadas |= vistas
        self.plagas_vistas |= plagas
        self.t_escaneando += self.reloj.t - t_ini
        self.eventos.append({"t": round(self.reloj.t, 1), "evento": "escaneo", "motivo": motivo,
                             "plantas_vistas": len(vistas), "plagas": len(plagas),
                             "segundos": round(self.reloj.t - t_ini, 1)})

    def _plantas_a_la_vista(self):
        """Qué plantas entran en el campo de la cámara con la inclinación actual.

        Se comprueba de verdad la geometría: la franja de suelo que abarca la
        cámara depende de su altura, su inclinación y su campo; y una planta
        muy lejos ocupa tan pocos píxeles que no se reconoce."""
        geo = self.geometria()
        phi = math.radians(geo["camara_inclinacion_deg"])
        medio_v = math.radians(geo["camara_hfov_deg"] * 0.75) / 2      # vertical ≈ 3/4 del horizontal
        medio_h = math.radians(geo["camara_hfov_deg"]) / 2
        h = geo["camara_altura_m"]
        # Distancias al suelo que cubre el borde inferior y el superior.
        cerca = h / math.tan(min(math.pi / 2 - 1e-3, phi + medio_v))
        lejos = (h / math.tan(max(1e-3, phi - medio_v))) if phi > medio_v else DISTANCIA_ESCANEO_MAX_M
        cerca, lejos = max(0.05, cerca), min(DISTANCIA_ESCANEO_MAX_M, lejos)
        vistas, plagas = set(), set()
        for i, (px, py, radio, plaga, altura) in enumerate(self.mundo.plantas):
            dx, dy = px - self.robot.x, py - self.robot.y
            # A ejes del robot (la cámara va montada mirando al frente).
            adelante = dx * math.cos(self.robot.theta) + dy * math.sin(self.robot.theta)
            lado = -dx * math.sin(self.robot.theta) + dy * math.cos(self.robot.theta)
            adelante -= geo["camara_adelante_m"]
            if not (cerca <= adelante <= lejos):
                continue
            if abs(math.atan2(lado, max(adelante, 1e-3))) > medio_h:
                continue
            # Tamaño aparente: por debajo de unos pocos píxeles no se
            # reconoce nada, ni con el mejor modelo.
            dist = math.hypot(adelante, lado)
            px_planta = 2 * radio / dist * (ANCHO_CAMARA / 2) / math.tan(medio_h)
            if px_planta < PIXELES_MINIMOS_PLANTA:
                continue
            vistas.add(i)
            if plaga:
                plagas.add(i)
        return vistas, plagas

    async def _escanear_punto(self, indice, pos):
        """Escaneo de un punto marcado en el mapa (los 🌱 del dashboard)."""
        self.explorador.ceder_mando(self.cfg["autonomia"]["escaneo_duracion_s"] + 2)
        await self._escanear(f"punto de escaneo {indice}")
        self.puntos_escaneados.add(indice)

    # ----------------------------------------------------------------- física
    async def _fisica(self):
        marco = 0
        anterior = time.perf_counter()
        while self.reloj.t < self.segundos and not self.parar_pedido:
            if self.tiempo_real:
                ahora = time.perf_counter()
                # Tope de 0,1 s: si el proceso se atasca, el robot no debe
                # dar un salto enorme en un único paso.
                dt = min(0.1, (ahora - anterior) * self.factor)
                anterior = ahora
                if dt <= 0:
                    await self.reloj._sleep(0.005)
                    continue
            else:
                dt = DT
            self.robot.paso(dt)
            self.slam.paso(dt, self.robot.v, self.robot.omega,
                           (self.robot.x, self.robot.y, self.robot.theta))
            self.reloj.t += dt
            self.t_total += dt
            if len(self.ruta):
                self.desvio_ruta_suma += ruta_ideal.desvio(self.ruta, (self.robot.x, self.robot.y))
                self.desvio_ruta_n += 1
            if self.robot.desvio_del_sendero() <= self.mundo.ancho_sendero / 2:
                self.t_en_sendero += DT

            # Cerrar la lección si el operador dejó de tocar, y juzgar si lo
            # que se aplicó de lo aprendido funcionó.
            m = self.aprendizaje.cerrar(self.reloj.t) if (
                self.aprendizaje._abierta
                and self.reloj.t - self.aprendizaje._ultimo_toque > 3.0) else None
            if m is not None:
                self.eventos.append({"t": round(self.reloj.t, 1), "evento": "aprendido",
                                     "firma": m.firma,
                                     "pasos": [p["accion"] for p in m.pasos]})
            if self.aprendizaje._pendiente is not None:
                p0 = self._pose_al_aplicar or (self.robot.x, self.robot.y)
                movido = math.dist((self.robot.x, self.robot.y), p0)
                ok = self.aprendizaje.juzgar(self.sensores(), movido, self.reloj.t)
                if ok is not None:
                    self.eventos.append({"t": round(self.reloj.t, 1),
                                         "evento": "maniobra_juzgada", "funciono": ok})

            # ¿Hace rato que no se ve el sendero? Probar otra inclinación.
            # Solo mientras la autonomía USE la cámara: durante el recorrido
            # ideal el piloto va por la ruta y nadie mira, así que el vigía se
            # disparaba cada 6 s y el robot se pasaba media sesión buscando.
            usa_camara = self.explorador.activo or self.piloto is not None
            if self.seguidor.vision.error_m is not None or not usa_camara:
                self._sendero_visto_en = self.reloj.t
            elif (self.reloj.t - self._sendero_visto_en > SIN_SENDERO_PARA_BUSCAR_S
                  and (self._buscando_servo is None or self._buscando_servo.done())
                  and self.cfg["autonomia"].get("sendero_activo")):
                self._sendero_visto_en = self.reloj.t
                self._buscando_servo = asyncio.create_task(self._buscar_inclinacion())

            # ¿Hay un punto de escaneo al lado y sin cubrir? Pararse y mirarlo.
            if (len(self.mundo.escaneos) and self.piloto is None
                    and (self._escaneo_en_curso is None or self._escaneo_en_curso.done())):
                for i, (ex, ey) in enumerate(self.mundo.escaneos):
                    if i in self.puntos_escaneados:
                        continue
                    if math.dist((ex, ey), (self.robot.x, self.robot.y)) < RADIO_PUNTO_ESCANEO_M:
                        self._escaneo_en_curso = asyncio.create_task(self._escanear_punto(i, (ex, ey)))
                        break

            if self.reloj.t - self._ultima_traza >= 0.25:
                self._ultima_traza = self.reloj.t
                self.traza.append((round(self.robot.x, 3), round(self.robot.y, 3)))

            if self.reloj.t - self._ultimo_lidar >= LIDAR_PERIODO_S:
                self._ultimo_lidar = self.reloj.t
                scan = self.robot.scan()
                self.lidar.ingest_scan(scan)
                self.slam.ingerir_scan(scan)
                self.traza_slam.append((round(self.slam.x, 3), round(self.slam.y, 3)))
                self._contar_visita()

            # Recorrido ideal: lo pilota la ruta, no la autonomía.
            if self.piloto is not None and self.reloj.t - self._ultimo_piloto >= 0.5:
                self._ultimo_piloto = self.reloj.t
                self._pilotar()

            if self.reloj.t - self._ultimo_log >= 0.5:
                self._ultimo_log = self.reloj.t
                self._registrar()
                if self.video is not None:
                    marco += 1
                    if marco % 2 == 0:
                        self._grabar_video()
            await self.reloj._sleep(0.004 if self.tiempo_real else 0)

    # ------------------------------------------------- recorrido ideal
    def _pilotar(self):
        """Un pulso del piloto que sigue la ruta ideal."""
        # La cámara TIENE que mirar también aquí: cada pulso del recorrido
        # ideal es un ejemplo de entrenamiento, y sin el detector de senderos
        # los ejemplos salen sin la mitad de la información (el modelo
        # entrenado así solo sabía decir "adelante").
        if self.cfg["autonomia"].get("sendero_activo") and self.robot.servo_asentado:
            dato = self._frame()
            try:
                self.seguidor.vision.analizar(cv2, dato[0], self.cfg["autonomia"],
                                              self.geometria())
            except Exception as exc:
                self.eventos.append({"t": round(self.reloj.t, 1), "evento": "error_vision",
                                     "detalle": f"{type(exc).__name__}: {exc}"})
        pose = (self.robot.x, self.robot.y, self.robot.theta)
        accion, detalle = self.piloto.orden(pose)
        ctrl, escala = self.cfg["control"], self.cfg["autonomia"]["escala_velocidad"]
        if accion == "parar":
            self.conductor.parar()
            self.terminar_recorrido()
            return
        self.conductor.ordenar(accion, v_max=ctrl["max_v_mps"] * escala,
                               omega_max=ctrl["max_omega_rps"] * escala, maniobra=False)
        # Cada pulso del recorrido ideal es un EJEMPLO de conducción buena:
        # situación percibida -> lo que había que hacer.
        self.registro_ideal.append({"t": round(self.reloj.t, 2), **self.sensores(),
                                    **detalle, "accion": accion,
                                    "x": round(self.robot.x, 3), "y": round(self.robot.y, 3),
                                    "theta_deg": round(math.degrees(self.robot.theta) % 360, 1),
                                    "error_slam_m": round(self.slam.error(pose), 3)})

    def iniciar_recorrido_ideal(self, vueltas=1) -> dict:
        """El robot recorre la ruta trazada por el centro del sendero.

        Es la referencia: "así había que hacerlo". Se guardan los datos para
        comparar después con el modo autónomo y para entrenar."""
        if not len(self.ruta):
            return {"error": "este mapa no tiene sendero trazado"}
        self.explorador.detener()
        self.modo = "ideal"
        self.modo_recorrido = "ideal"
        cerrado = bool(math.dist(self.ruta[0], self.ruta[-1]) < 0.5)
        self.piloto = ruta_ideal.Piloto(self.ruta, cerrado=cerrado)
        self.registro_ideal = []
        self._marca_ideal = {"t": self.reloj.t, "recorrido": self.robot.recorrido,
                             "pisadas": self.robot.plantas_pisadas,
                             "choques": self.robot.colisiones,
                             "desvio_suma": self.desvio_ruta_suma,
                             "desvio_n": self.desvio_ruta_n,
                             "slam_saltos": self.slam.saltos}
        self.eventos.append({"t": round(self.reloj.t, 1), "evento": "recorrido_ideal_inicio",
                             "puntos": len(self.ruta), "cerrado": cerrado})
        return {"ok": True, "puntos": len(self.ruta), "cerrado": cerrado}

    def terminar_recorrido(self) -> dict:
        """Cierra el recorrido ideal y guarda sus métricas."""
        if self.piloto is None:
            return {"error": "no hay recorrido en curso"}
        m = self._resumen_tramo(self._marca_ideal)
        m["muestras_ideales"] = len(self.registro_ideal)
        self.resultado_ideal = m
        self.piloto = None
        self.modo_recorrido = None
        self.modo = "auto"
        self.conductor.parar()
        self.eventos.append({"t": round(self.reloj.t, 1), "evento": "recorrido_ideal_fin", **m})
        return m

    def _resumen_tramo(self, marca) -> dict:
        """Métricas del tramo recorrido desde `marca`."""
        n = max(self.desvio_ruta_n - marca["desvio_n"], 1)
        return {
            "segundos": round(self.reloj.t - marca["t"], 1),
            "recorrido_m": round(self.robot.recorrido - marca["recorrido"], 2),
            "desvio_medio_cm": round(100 * (self.desvio_ruta_suma - marca["desvio_suma"]) / n, 1),
            "plantas_pisadas": self.robot.plantas_pisadas - marca["pisadas"],
            "choques": self.robot.colisiones - marca["choques"],
            "saltos_slam": self.slam.saltos - marca["slam_saltos"],
            "error_slam_final_cm": round(100 * self.slam.error(
                (self.robot.x, self.robot.y, self.robot.theta)), 1),
        }

    def volver_al_inicio(self, reiniciar_slam=True):
        """Devuelve el robot al punto de partida para repetir en otro modo."""
        self.robot.x, self.robot.y, self.robot.theta = self.pose_inicial
        self.robot.v = self.robot.omega = 0.0
        self.robot.v_rampa = self.robot.omega_rampa = 0.0
        self.mundo.pisadas.clear()
        if reiniciar_slam:
            self.slam = SlamSimulado(self.pose_inicial, self.mundo.limites,
                                     semilla=int(self.reloj.t * 1000) % 9999,
                                     ruido=self.slam.ruido)
            self.traza_slam = []
        self.traza = []
        self.eventos.append({"t": round(self.reloj.t, 1), "evento": "vuelta_al_inicio"})

    def iniciar_comparacion(self) -> dict:
        """Marca el arranque del tramo AUTÓNOMO que se compara con el ideal."""
        self._marca_auto = {"t": self.reloj.t, "recorrido": self.robot.recorrido,
                            "pisadas": self.robot.plantas_pisadas,
                            "choques": self.robot.colisiones,
                            "desvio_suma": self.desvio_ruta_suma,
                            "desvio_n": self.desvio_ruta_n,
                            "slam_saltos": self.slam.saltos}
        self.eventos.append({"t": round(self.reloj.t, 1), "evento": "comparacion_inicio"})
        return {"ok": True}

    def cerrar_comparacion(self) -> dict:
        """Ideal contra autónomo, lado a lado."""
        if not hasattr(self, "_marca_auto"):
            return {"error": "no se marcó el arranque del tramo autónomo"}
        auto = self._resumen_tramo(self._marca_auto)
        ideal = getattr(self, "resultado_ideal", None)
        self.comparacion = {"ideal": ideal, "autonomo": auto}
        if ideal:
            self.comparacion["diferencia"] = {
                k: round(auto[k] - ideal[k], 2)
                for k in ("recorrido_m", "desvio_medio_cm", "plantas_pisadas", "choques")
                if isinstance(auto.get(k), (int, float)) and isinstance(ideal.get(k), (int, float))}
        self.eventos.append({"t": round(self.reloj.t, 1), "evento": "comparacion",
                             **self.comparacion.get("diferencia", {})})
        return self.comparacion

    # --------------------------------------------------------- filtro ideal
    def filtrar(self, accion: str, maniobra: bool) -> tuple[str, bool]:
        """Filtro de conducción ideal: corrige la orden para no salirse.

        No sustituye a la autonomía: la deja conducir y solo interviene
        cuando el robot se aleja de la ruta más de media anchura de sendero,
        cambiando el pulso por el giro que lo devuelve a la línea."""
        if not self.filtro_ideal or not len(self.ruta) or accion == "parar":
            return accion, maniobra
        pose = (self.robot.x, self.robot.y, self.robot.theta)
        fuera = ruta_ideal.desvio(self.ruta, (pose[0], pose[1]))
        if fuera < max(0.15, self.mundo.ancho_sendero / 2):
            return accion, maniobra
        piloto = ruta_ideal.Piloto(self.ruta, cerrado=False)
        correcta, detalle = piloto.orden(pose)
        if correcta != accion and correcta != "parar":
            self.correcciones_filtro += 1
            self.eventos.append({"t": round(self.reloj.t, 1), "evento": "filtro_ideal",
                                 "pedida": accion, "corregida": correcta,
                                 "desvio_cm": round(fuera * 100)})
            return correcta, False
        return accion, maniobra

    # ------------------------------------------------------- mando manual
    def cambiar_modo(self, modo: str):
        """"auto" = decide el explorador; "manual" = decide la persona."""
        if modo == self.modo:
            return self.modo
        if modo == "manual":
            self.explorador.detener()
        else:
            self.lidar.ingest_scan(self.robot.scan())
            self.explorador.iniciar()
        self.modo = modo
        self.eventos.append({"t": round(self.reloj.t, 1), "evento": "modo", "modo": modo})
        return self.modo

    def sensores(self) -> dict:
        """Lo que el robot sabe de su entorno, que es la ENTRADA de cualquier
        modelo que se entrene con las decisiones del operador."""
        d = self.lidar.distancias(self.cfg["slam"])
        v = self.seguidor.vision
        return {
            "lidar_adelante_m": d["adelante"], "lidar_atras_m": d["atras"],
            "lidar_izquierda_m": d["izquierda"], "lidar_derecha_m": d["derecha"],
            "sendero_lateral_m": v.lateral_m, "sendero_rumbo_deg": v.angulo_deg,
            "sendero_ancho_m": v.ancho_m, "sendero_confianza": v.confianza,
            "v_actual": round(self.robot.v, 3), "omega_actual": round(self.robot.omega, 3),
        }

    def conducir_manual(self, accion: str, maniobra: bool = True):
        """Un pulso mandado por la persona.

        Funciona TAMBIÉN en automático: el operador tiene prioridad y su
        corrección se aprende. Es la diferencia entre "conducir el robot" y
        "enseñarle": mientras él intenta algo y se equivoca, tú le enseñás la
        combinación que saca de ahí, y queda guardada para esa situación."""
        ctrl = self.cfg["control"]
        escala = self.cfg["autonomia"]["escala_velocidad"]
        sensores = self.sensores()
        estado = self.explorador.estado

        if self.modo == "auto":
            # Tomar el mando: se corta lo que estuviera haciendo la autonomía.
            self.explorador.ceder_mando(SILENCIO_TRAS_CORRECCION_S)
            self.intervenciones += 1

        if accion == "parar":
            self.conductor.parar()
        else:
            self.conductor.ordenar(accion, v_max=ctrl["max_v_mps"] * escala,
                                   omega_max=ctrl["max_omega_rps"] * escala,
                                   maniobra=maniobra)
        # El reloj del aprendizaje es el del EPISODIO (virtual o real, según el
        # modo): si se mezclan, las lecciones se cierran cuando no deben.
        firma = self.aprendizaje.corregir(sensores, estado, accion, maniobra,
                                          ahora=self.reloj.t)
        self.muestras.append({"t": round(self.reloj.t, 2), "escenario": self.mundo.nombre,
                              **sensores, "estado_autonomia": estado, "firma": firma,
                              "modo": self.modo, "accion": accion, "maniobra": maniobra})
        self.eventos.append({"t": round(self.reloj.t, 1), "evento": "correccion",
                             "accion": accion, "firma": firma, "estado": estado[:40]})
        return len(self.muestras)

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
        if not self.tiempo_real:
            self.reloj.instalar()
        try:
            # El explorador exige un scan fresco para arrancar (sin LIDAR el
            # robot iría a ciegas), así que se le da el primero a mano.
            self.lidar.ingest_scan(self.robot.scan())
            # La física arranca YA: la calibración necesita que el tiempo
            # corra, y en modo rápido el reloj solo avanza dentro de ese bucle.
            fisica = asyncio.create_task(self._fisica())
            # Calibración previa: antes de moverse, buscar desde qué
            # inclinación se ve el sendero en ESTE terreno. El ángulo bueno
            # depende del suelo y de la luz, no es una constante.
            if self.cfg["autonomia"].get("sendero_activo") and self.calibrar_servo:
                await self._buscar_inclinacion()
            ok, msg = self.explorador.iniciar()
            if not ok:
                raise RuntimeError(f"no arrancó la exploración: {msg}")
            self.eventos.append({"t": round(self.reloj.t, 1), "evento": "inicio", "detalle": msg})
            await fisica
            self.explorador.detener()
            await self.reloj._sleep(0)
        finally:
            if not self.tiempo_real:
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
            "plantas_pisadas": self.robot.plantas_pisadas,
            "frenadas_por_follaje": self.robot.roces_rama,
            "en_sendero_pct": round(100 * self.t_en_sendero / max(self.t_total, 1e-6), 1),
            "desvio_medio_cm": round(float(np.mean([f["desvio_sendero_cm"] for f in self.filas])), 1) if self.filas else None,
            "pulsos": self.explorador.pulsos, "giros": self.explorador.giros,
            "correcciones_sendero": self.explorador.correcciones,
            "desatascos": self.explorador.desatascos,
            "callejones": self.explorador.callejones,
            "bucles_explorador": self.explorador.bucles,
            "intervenciones": self.intervenciones,
            "busquedas_servo": self.busquedas_servo,
            "desvio_ruta_medio_cm": round(100 * self.desvio_ruta_suma / max(self.desvio_ruta_n, 1), 1),
            "correcciones_filtro": self.correcciones_filtro,
            "pulsos_politica": self.explorador.pulsos_politica,
            "slam": self.slam.resumen((self.robot.x, self.robot.y, self.robot.theta)),
            "mapa": self.slam.calidad_mapa(self.mundo),
            "servo_final": round(self.robot.servo),
            "aprendidas": self.explorador.aprendidas,
            "aprendizaje": self.aprendizaje.resumen(),
            "escaneos": self.escaneos,
            "puntos_escaneo_cubiertos": f"{len(self.puntos_escaneados)}/{len(self.mundo.escaneos)}",
            "plantas_escaneadas": len(self.escaneadas),
            "segundos_escaneando": round(self.t_escaneando, 1),
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
