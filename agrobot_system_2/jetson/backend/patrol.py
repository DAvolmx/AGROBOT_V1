"""
Patrullaje autónomo por waypoints.
=================================

Dos modos de navegación, con caída automática:

  1. Nav2 (por defecto): planificación global + costmap + recovery behaviors.
  2. Reactiva (fallback): si Nav2 no está o aborta un punto, el explorador
     reactivo toma el mando con un rumbo objetivo hacia el waypoint.

El recorrido son waypoints en el marco `map`, grabados conduciendo a mano.
Se repiten en ciclo cerrado. El escaneo va por reloj propio (SCAN_PERIODO_S),
no por waypoint.
"""
import asyncio
import json
import math
import time
from pathlib import Path

SCAN_PERIODO_S = 120.0        # un barrido cada 2 minutos de patrulla
WAYPOINT_TOL_M = 0.35         # a esta distancia se da por llegado
GOAL_TIMEOUT_S = 90.0         # si no llega, se pasa al siguiente y se sigue
TICK_S = 0.5                  # cadencia del bucle de vigilancia

# Una plaga detectada dispara un barrido, pero YOLO detecta varias veces por
# segundo sobre la misma planta. Sin esta ventana el robot se quedaría
# barriendo en bucle delante del primer arbusto enfermo que encontrara.
COOLDOWN_DETECCION_S = 30.0
# Una detección cuenta para escanear solo si PERSISTE: al menos
# DETECCIONES_MIN en VENTANA_DETECCION_S. Un destello suelto de YOLO no.
DETECCIONES_MIN = 3
VENTANA_DETECCION_S = 2.0
# Tras un escaneo por detección, el siguiente pide que el robot se haya
# movido: escanear la misma escena desde el mismo sitio no aporta nada. El
# 2026-09-30, aparcado ante una estantería que YOLO tomaba por planta, el
# servo barría cada 30 s ("el servo está loco").
MOVIDO_MIN_M = 0.5
GIRADO_MIN_DEG = 30.0

# Grabación automática del recorrido: un punto cada tanto avance. La
# distancia la fija la config (`autonomia.distancia_puntos_m`, 0,50 m); este
# valor solo se usa si esa clave no estuviera.
DISTANCIA_AUTO_CM = 50
TICK_AUTO_S = 0.2


REACTIVE_TICK_S = 0.25
REACTIVE_ARRIVE_M = 0.50
REACTIVE_TIMEOUT_S = 120.0


class PatrolController:
    """
    Bucle de patrulla. Navega con Nav2 cuando está disponible; si no, cae
    a navegación reactiva (explorador + rumbo objetivo hacia el waypoint).
    """

    def __init__(self, *, send_nav_goal, cancel_nav_goal, estado_meta, get_pose,
                 barrido, on_event, ruta_waypoints: Path,
                 distancia_puntos=lambda: DISTANCIA_AUTO_CM,
                 conducir=None, parar=None, distancias=None, edad_scan=None,
                 get_costmap=None):
        self._distancia_puntos = distancia_puntos
        self._send_nav_goal = send_nav_goal
        self._cancel_nav_goal = cancel_nav_goal
        self._estado_meta = estado_meta
        self._get_pose = get_pose
        self._barrido = barrido
        self._on_event = on_event
        self._ruta = ruta_waypoints
        self._conducir = conducir
        self._parar_robot = parar
        self._distancias = distancias
        self._edad_scan = edad_scan
        self._get_costmap = get_costmap

        self.waypoints: list[dict] = []
        self.activo = False
        self.indice = 0
        self.vueltas = 0
        self.ultimo_escaneo = 0.0
        self.escaneos = 0
        self.estado = "detenido"
        self.modo_nav = "nav2"
        self._tarea: asyncio.Task | None = None
        self._ultima_deteccion_escaneada = 0.0
        self._escaneando = False
        self._meta_cancelada = False
        self.auto_activa = False
        self._auto_ultima = None
        self._tarea_auto: asyncio.Task | None = None
        self._intentos_reactivos = 0

        self._cargar()

    # ---------------------------------------------------------------
    # Waypoints
    # ---------------------------------------------------------------
    def _cargar(self):
        try:
            self.waypoints = json.loads(self._ruta.read_text())
        except FileNotFoundError:
            self.waypoints = []
        except (json.JSONDecodeError, OSError) as exc:
            print(f"[patrulla] no se pudo leer {self._ruta.name} ({exc}) — se empieza vacío")
            self.waypoints = []

    def _guardar(self):
        self._ruta.write_text(json.dumps(self.waypoints, indent=2))

    def agregar_waypoint(self, x: float, y: float, theta_deg: float = 0.0) -> dict:
        wp = {"x": float(x), "y": float(y), "theta_deg": float(theta_deg)}
        self.waypoints.append(wp)
        self._guardar()
        return wp

    def grabar_pose_actual(self) -> dict | None:
        """Graba donde está el robot ahora. Es la vía prevista para armar el
        recorrido: se conduce a mano hasta cada punto y se pulsa grabar."""
        pose = self._get_pose()
        if not pose:
            return None
        return self.agregar_waypoint(pose.get("x", 0.0), pose.get("y", 0.0),
                                     pose.get("theta_deg", 0.0))

    def reemplazar_waypoints(self, waypoints: list[dict]):
        """Pone el recorrido de un mapa recién cargado."""
        self.waypoints = [{"x": float(w["x"]), "y": float(w["y"]),
                           "theta_deg": float(w.get("theta_deg", 0.0))} for w in waypoints]
        self.indice = 0
        self._guardar()

    # ---------------------------------------------------------------
    # Grabación automática mientras el operador conduce
    # ---------------------------------------------------------------
    def grabacion_auto(self, activa: bool) -> tuple[bool, str]:
        self.auto_activa = bool(activa)
        if not self.auto_activa:
            if self._tarea_auto is not None:
                self._tarea_auto.cancel()
                self._tarea_auto = None
            return True, "grabación automática detenida"

        pose = self._get_pose()
        if pose is None:
            self.auto_activa = False
            return False, "sin pose todavía — ¿está corriendo el SLAM?"
        # El primer punto es donde está el robot ahora: así el recorrido
        # empieza en el sitio desde el que se sale.
        self._auto_ultima = (pose["x"], pose["y"])
        self.agregar_waypoint(pose["x"], pose["y"], pose.get("theta_deg", 0.0))
        self._emitir({"evento": "punto_auto", "total": len(self.waypoints)})
        if self._tarea_auto is None or self._tarea_auto.done():
            self._tarea_auto = asyncio.create_task(self._bucle_auto())
        return True, f"grabando un punto cada {self._distancia_puntos():g} cm"

    async def _bucle_auto(self):
        """Suelta un punto cada `autonomia.distancia_puntos_cm`. Mide sobre la
        pose del mapa, no sobre el tiempo: parado no acumula puntos."""
        try:
            while self.auto_activa:
                await asyncio.sleep(TICK_AUTO_S)
                pose = self._get_pose()
                if pose is None or self._auto_ultima is None:
                    continue
                if math.hypot(pose["x"] - self._auto_ultima[0],
                              pose["y"] - self._auto_ultima[1]) >= self._distancia_puntos() / 100.0:
                    self._auto_ultima = (pose["x"], pose["y"])
                    self.agregar_waypoint(pose["x"], pose["y"], pose.get("theta_deg", 0.0))
                    self._emitir({"evento": "punto_auto", "total": len(self.waypoints)})
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.auto_activa = False
            self._emitir({"evento": "error_auto", "detalle": str(exc)})

    def borrar_waypoints(self):
        self.waypoints = []
        self.indice = 0
        self._auto_ultima = None
        self._guardar()

    # ---------------------------------------------------------------
    # Ciclo de vida
    # ---------------------------------------------------------------
    def iniciar(self) -> tuple[bool, str]:
        if self.activo:
            return False, "la patrulla ya está corriendo"
        if len(self.waypoints) < 2:
            # Con un solo punto (el que deja la exploración al arrancar) la «patrulla» era ir y volver al mismo sitio en bucle
            # (2026-10-23: 98 vueltas sin moverse del sitio). Hacen falta al menos dos puntos de recorrido.
            return False, "no hay waypoints grabados suficientes (mínimo 2) — conducí el robot y grabá el recorrido primero"

        self.activo = True
        self.vueltas = 0
        self.escaneos = 0
        self.ultimo_escaneo = 0.0

        # (2026-10-07) Antes aquí se llamaba a _reinit_pose_slam, que decía
        # "reiniciar la pose en AMCL" pero en realidad mandaba una META de
        # navegación a la pose actual. Era doblemente incorrecto: (1) el stack
        # usa slam_toolbox en localización continua, no AMCL, así que no hay
        # nada que "resincronizar"; (2) esa meta competía con las metas del
        # propio _bucle (carrera de goals). Se eliminó: iniciar la patrulla no
        # debe lanzar una meta espuria. Si en el futuro hiciera falta FIJAR una
        # pose inicial, es un /initialpose explícito, no un goal de navegación.

        self._tarea = asyncio.create_task(self._bucle())
        return True, f"patrulla iniciada sobre {len(self.waypoints)} waypoints"

    def detener(self):
        self.activo = False
        if self._tarea is not None:
            self._tarea.cancel()
            self._tarea = None
        self._cancel_nav_goal()
        self.estado = "detenido"

    # ---------------------------------------------------------------
    # Escaneo
    # ---------------------------------------------------------------
    async def escanear(self, motivo: str):
        """Para el robot, barre 10 s y devuelve el control.

        Se cancela la meta ANTES de mandar el cero: si no, Nav2 sigue
        publicando /cmd_vel y el robot se mueve durante el barrido. Quien
        esté navegando (`_ir_a`) ve `_meta_cancelada` y la reenvía al acabar,
        venga el escaneo del reloj o de una detección de planta.
        """
        if self._escaneando:
            return
        self._escaneando = True
        try:
            self.estado = f"escaneando ({motivo})"
            self._cancel_nav_goal()
            self._meta_cancelada = True
            await asyncio.sleep(0.3)      # que se vacíe el /cmd_vel en vuelo

            self.escaneos += 1
            self.ultimo_escaneo = time.monotonic()
            self._emitir({"evento": "escaneo", "motivo": motivo, "total": self.escaneos})
            await self._barrido()
        finally:
            self._escaneando = False
            self.estado = "navegando" if self.activo else "detenido"

    def solicitar_escaneo_por_deteccion(self, modo: str = "siempre") -> bool:
        """Dispara un barrido al detectar una planta.

        Funciona con la patrulla apagada también: el usuario pidió que el
        escaneo se active SIEMPRE que se detecte una planta, no solo en
        modo autónomo. `modo` (config detection.escanear_al_detectar):
        "siempre" | "autonomo" (solo con la patrulla en marcha) | "nunca".

        Filtros (2026-09-30): la detección tiene que persistir, y tras un
        escaneo el robot tiene que haberse movido antes de repetir.
        """
        ahora = time.monotonic()
        if modo == "nunca" or (modo == "autonomo" and not self.activo):
            return False
        self._detecciones_recientes = [t for t in getattr(self, "_detecciones_recientes", [])
                                       if ahora - t < VENTANA_DETECCION_S] + [ahora]
        if len(self._detecciones_recientes) < DETECCIONES_MIN:
            return False
        if ahora - self._ultima_deteccion_escaneada < COOLDOWN_DETECCION_S:
            return False
        if self._escaneando:
            return False
        pose = self._get_pose() or {}
        previa = getattr(self, "_pose_ultimo_escaneo", None)
        if previa is not None and pose:
            movido = math.hypot(pose.get("x", 0) - previa.get("x", 0),
                                pose.get("y", 0) - previa.get("y", 0))
            giro = abs((pose.get("theta_deg", 0) - previa.get("theta_deg", 0) + 180) % 360 - 180)
            if movido < MOVIDO_MIN_M and giro < GIRADO_MIN_DEG:
                return False
        self._pose_ultimo_escaneo = dict(pose) if pose else None
        self._ultima_deteccion_escaneada = ahora
        self._detecciones_recientes = []
        asyncio.create_task(self.escanear("planta detectada"))
        return True

    # ---------------------------------------------------------------
    # Bucle principal
    # ---------------------------------------------------------------
    async def _bucle(self):
        try:
            self.estado = "estabilizando SLAM"
            self._emitir({"evento": "warmup_inicio"})
            await asyncio.sleep(3.0)
            self._emitir({"evento": "warmup_fin"})

            while self.activo:
                wp = self.waypoints[self.indice % len(self.waypoints)]
                resultado = await self._ir_a(wp)

                if resultado == "sin_nav2":
                    # Se reintenta el MISMO punto: saltar al siguiente con
                    # Nav2 caído solo recorre la lista sin moverse.
                    await asyncio.sleep(3.0)
                    continue

                self.indice += 1
                if self.indice % len(self.waypoints) == 0:
                    self.vueltas += 1
                    self._emitir({"evento": "vuelta_completa", "vueltas": self.vueltas})
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.activo = False
            self.estado = f"error: {exc}"
            self._emitir({"evento": "error", "detalle": str(exc)})
            print(f"[patrulla] bucle abortado: {type(exc).__name__}: {exc}")

    async def _enviar(self, wp: dict) -> bool:
        """En un hilo: `send_nav_goal` espera al servidor de Nav2 hasta 5 s y
        eso, en el bucle de eventos, congela el WebSocket y el vídeo."""
        self._meta_cancelada = False
        return await asyncio.to_thread(
            self._send_nav_goal, wp["x"], wp["y"], wp["theta_deg"])

    async def _ir_a(self, wp: dict) -> str:
        """Navega a un waypoint, intercalando los barridos que toquen.

        Devuelve "llegado" | "abortada" | "timeout" | "sin_nav2" | "detenido".
        Si Nav2 falla, cae a navegación reactiva automáticamente.
        """
        i = self.indice % len(self.waypoints)
        if not await self._enviar(wp):
            self.modo_nav = "reactiva"
            self._emitir({"evento": "nav2_caido", "fallback": "reactiva"})
            return await self._ir_reactivo(wp)

        self.modo_nav = "nav2"
        self.estado = "navegando"
        self._emitir({"evento": "rumbo_a", "indice": i, "waypoint": wp})

        inicio = time.monotonic()
        while self.activo:
            await asyncio.sleep(TICK_S)

            if self._escaneando:
                inicio = time.monotonic()
                continue

            if time.monotonic() - self.ultimo_escaneo >= SCAN_PERIODO_S:
                await self.escanear("periódico")
                continue

            if self._meta_cancelada:
                if not await self._enviar(wp):
                    self.modo_nav = "reactiva"
                    return await self._ir_reactivo(wp)
                inicio = time.monotonic()
                continue

            estado = self._estado_meta()
            if estado == "exito" or self._distancia_a(wp) <= WAYPOINT_TOL_M:
                self._emitir({"evento": "llegado", "indice": i})
                return "llegado"

            if estado in ("abortada", "rechazada", "cancelada"):
                self._emitir({"evento": "meta_abortada", "indice": i, "fallback": "reactiva"})
                self.modo_nav = "reactiva"
                return await self._ir_reactivo(wp)

            if time.monotonic() - inicio > GOAL_TIMEOUT_S:
                self._emitir({"evento": "timeout", "indice": i})
                self._cancel_nav_goal()
                return "timeout"
        return "detenido"

    def _rumbo_a_wp(self, wp: dict):
        """Ángulo relativo y distancia al waypoint, desde la pose actual."""
        pose = self._get_pose()
        if pose is None:
            return None
        dx = wp["x"] - pose.get("x", 0.0)
        dy = wp["y"] - pose.get("y", 0.0)
        dist = math.hypot(dx, dy)
        if dist < 0.01:
            return (0.0, dist)
        ang_mundo = math.degrees(math.atan2(dy, dx))
        ang_robot = pose.get("theta_deg", 0.0)
        rel = (ang_mundo - ang_robot + 180) % 360 - 180
        return (rel, dist)

    async def _ir_reactivo(self, wp: dict) -> str:
        """Navega hacia un waypoint con pulsos reactivos + costmap.

        Es más lento y menos preciso que Nav2, pero funciona sin ROS.
        El robot avanza si el waypoint está adelante, o gira hasta que lo esté.
        """
        if self._conducir is None or self._distancias is None:
            self.estado = "sin Nav2 ni control reactivo"
            return "sin_nav2"

        i = self.indice % len(self.waypoints)
        self.estado = f"reactiva → wp {i}"
        self._emitir({"evento": "reactiva_inicio", "indice": i, "waypoint": wp})
        self._intentos_reactivos += 1

        inicio = time.monotonic()
        while self.activo:
            await asyncio.sleep(REACTIVE_TICK_S)

            if self._escaneando:
                inicio = time.monotonic()
                continue

            if time.monotonic() - self.ultimo_escaneo >= SCAN_PERIODO_S:
                await self.escanear("periódico reactivo")
                continue

            if self._distancia_a(wp) <= REACTIVE_ARRIVE_M:
                self._emitir({"evento": "llegado", "indice": i, "modo": "reactiva"})
                return "llegado"

            if time.monotonic() - inicio > REACTIVE_TIMEOUT_S:
                self._emitir({"evento": "timeout_reactivo", "indice": i})
                if self._parar_robot:
                    self._parar_robot()
                return "timeout"

            if self._edad_scan and self._edad_scan() > 1.0:
                self.estado = "reactiva: sin LIDAR"
                if self._parar_robot:
                    self._parar_robot()
                continue

            d = self._distancias()
            rumbo = self._rumbo_a_wp(wp)
            if rumbo is None:
                continue
            ang, dist = rumbo
            avance_libre = 0.6

            frente = d.get("adelante")
            frente = math.inf if frente is None else frente

            if abs(ang) < 30 and frente >= avance_libre:
                self.estado = f"reactiva: avanzando ({dist:.1f}m)"
                self._conducir("adelante")
            elif abs(ang) >= 30:
                lado = "izquierda" if ang > 0 else "derecha"
                self.estado = f"reactiva: girando {lado} ({ang:+.0f}°)"
                self._conducir(lado, False)
            elif frente < avance_libre:
                costmap = self._get_costmap() if self._get_costmap else None
                if costmap and costmap._scan_count >= 3:
                    centro, _ = costmap.hueco_libre()
                    if centro is not None:
                        lado = "izquierda" if centro > 0 else "derecha"
                    else:
                        lado = "izquierda" if (d.get("izquierda") or 0) >= (d.get("derecha") or 0) else "derecha"
                else:
                    lado = "izquierda" if (d.get("izquierda") or 0) >= (d.get("derecha") or 0) else "derecha"
                self.estado = f"reactiva: esquivando {lado}"
                self._conducir(lado, False)

        if self._parar_robot:
            self._parar_robot()
        return "detenido"

    def _distancia_a(self, wp: dict) -> float:
        pose = self._get_pose()
        if not pose:
            return math.inf
        return math.hypot(pose.get("x", 0.0) - wp["x"], pose.get("y", 0.0) - wp["y"])

    # ---------------------------------------------------------------
    async def _reinit_pose_slam(self, pose: dict):
        """DEPRECADO (2026-10-07) y sin uso. Antes mandaba una meta de
        navegación a la pose actual creyendo que "reiniciaba AMCL"; era una
        meta espuria que competía con las del _bucle, y el stack usa
        slam_toolbox (sin AMCL). No hace nada. Reiniciar una pose de verdad es
        publicar en /initialpose, no enviar un goal."""
        self._emitir({"evento": "reinit_pose_slam_noop",
                      "detalle": "deprecado: no se envía meta al iniciar patrulla"})

    def _emitir(self, data: dict):
        try:
            self._on_event(data)
        except Exception:
            pass

    def snapshot(self) -> dict:
        restante = max(0.0, SCAN_PERIODO_S - (time.monotonic() - self.ultimo_escaneo))
        return {
            "activo": self.activo,
            "estado": self.estado,
            "modo_nav": self.modo_nav,
            "waypoints": self.waypoints,
            "indice": self.indice % len(self.waypoints) if self.waypoints else 0,
            "vueltas": self.vueltas,
            "escaneos": self.escaneos,
            "intentos_reactivos": self._intentos_reactivos,
            "proximo_escaneo_s": round(restante if self.activo else SCAN_PERIODO_S, 1),
            "auto_activa": self.auto_activa,
            "distancia_auto_m": self._distancia_puntos(),
        }
