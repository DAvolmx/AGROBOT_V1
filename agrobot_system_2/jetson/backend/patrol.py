"""
Patrullaje autónomo por waypoints.
=================================

El modo autónomo anterior solo levantaba una bandera y cargaba el modelo
YOLO: el robot se quedaba quieto esperando que alguien lo condujera. Este
módulo es la pieza que faltaba — el bucle que recorre el cultivo solo.

Cómo funciona, y por qué así:

  · El recorrido son waypoints en el marco `map` de Nav2, grabados una vez
    conduciendo el robot a mano. Se repiten en ciclo cerrado: al llegar al
    último se vuelve al primero. No se calcula una ruta de cobertura porque
    en un cultivo el recorrido útil lo decide el agricultor (por los surcos,
    no por el área libre), y grabarlo es más fiable que inferirlo.

  · La llegada se da por buena con lo primero que ocurra: Nav2 informa
    éxito, o la pose en `map` queda a menos de WAYPOINT_TOL_M. La distancia
    cubre el caso de que el resultado de la acción se pierda (Nav2
    reiniciado a mitad de misión). Si Nav2 aborta —no encontró camino ni
    tras sus maniobras de recuperación— se pasa al punto siguiente.

  · El escaneo va por reloj propio (cada SCAN_PERIODO_S), no por waypoint.
    Si fuera por waypoint, dos puntos juntos darían dos barridos seguidos y
    un tramo largo se quedaría sin ninguno.

  · Para escanear se CANCELA la meta primero. El barrido tiene que hacerse
    con el robot quieto —un servo barriendo mientras el robot avanza da
    fotos movidas— y mientras haya una meta viva Nav2 sigue publicando
    /cmd_vel y pisaría el cero que mandamos.
"""
import asyncio
import json
import math
import time
from pathlib import Path

# El barrido dura lo que dure en el firmware (SCAN_SWEEP_MS = 10000 en
# esp32c3_actuator.ino). Si allá se cambia, acá también: este valor solo
# dice cuánto hay que esperar antes de volver a mover el robot.
SCAN_SWEEP_S = 10.0

SCAN_PERIODO_S = 120.0        # un barrido cada 2 minutos de patrulla
WAYPOINT_TOL_M = 0.35         # a esta distancia se da por llegado
GOAL_TIMEOUT_S = 90.0         # si no llega, se pasa al siguiente y se sigue
TICK_S = 0.5                  # cadencia del bucle de vigilancia

# Una plaga detectada dispara un barrido, pero YOLO detecta varias veces por
# segundo sobre la misma planta. Sin esta ventana el robot se quedaría
# barriendo en bucle delante del primer arbusto enfermo que encontrara.
COOLDOWN_DETECCION_S = 30.0

# Grabación automática del recorrido: un punto cada tanto avance. La
# distancia la fija la config (`autonomia.distancia_puntos_m`, 0,50 m); este
# valor solo se usa si esa clave no estuviera.
DISTANCIA_AUTO_CM = 50
TICK_AUTO_S = 0.2


class PatrolController:
    """
    Bucle de patrulla. Vive en el event loop del backend y habla con Nav2
    por los callbacks que le pasa jetson_server, para no acoplarse ni al
    puente de ROS ni al actuador.
    """

    def __init__(self, *, send_nav_goal, cancel_nav_goal, estado_meta, get_pose,
                 trigger_scan, is_scanning, on_event, ruta_waypoints: Path,
                 distancia_puntos=lambda: DISTANCIA_AUTO_CM):
        self._distancia_puntos = distancia_puntos
        self._send_nav_goal = send_nav_goal      # (x, y, theta_deg) -> bool
        self._cancel_nav_goal = cancel_nav_goal  # ()
        self._estado_meta = estado_meta          # () -> "en_curso" | "exito" | "abortada" | ...
        self._get_pose = get_pose                # () -> {x, y, theta_deg}
        self._trigger_scan = trigger_scan        # () — manda v=0, omega=0, scan=1
        self._is_scanning = is_scanning          # () -> bool (bit1 del status del C3)
        self._on_event = on_event                # (dict) — al dashboard
        self._ruta = ruta_waypoints

        self.waypoints: list[dict] = []
        self.activo = False
        self.indice = 0
        self.vueltas = 0
        self.ultimo_escaneo = 0.0
        self.escaneos = 0
        self.estado = "detenido"
        self._tarea: asyncio.Task | None = None
        self._ultima_deteccion_escaneada = 0.0
        # Un escaneo cancela la meta de Nav2 para parar el robot. Puede
        # dispararlo el reloj o una detección de planta, que corre en otra
        # tarea: `_ir_a` usa esto para esperar y reenviar la meta después.
        self._escaneando = False
        self._meta_cancelada = False
        self.auto_activa = False
        self._auto_ultima = None
        self._tarea_auto: asyncio.Task | None = None

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
        if not self.waypoints:
            return False, "no hay waypoints grabados — conducí el robot y grabá el recorrido primero"

        self.activo = True
        self.vueltas = 0
        self.escaneos = 0
        # Se arranca con el reloj a cero para que el primer barrido salga al
        # empezar, no a los dos minutos: el operador necesita ver que el
        # escaneo funciona sin esperar de pie en el cultivo.
        self.ultimo_escaneo = 0.0
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

            self._trigger_scan()
            self.escaneos += 1
            self.ultimo_escaneo = time.monotonic()
            self._emitir({"evento": "escaneo", "motivo": motivo, "total": self.escaneos})

            await asyncio.sleep(SCAN_SWEEP_S + 0.5)
        finally:
            self._escaneando = False
            self.estado = "navegando" if self.activo else "detenido"

    def solicitar_escaneo_por_deteccion(self) -> bool:
        """Dispara un barrido al detectar una planta.

        Funciona con la patrulla apagada también: el usuario pidió que el
        escaneo se active SIEMPRE que se detecte una planta, no solo en
        modo autónomo.
        """
        ahora = time.monotonic()
        if ahora - self._ultima_deteccion_escaneada < COOLDOWN_DETECCION_S:
            return False
        if self._is_scanning():
            return False
        self._ultima_deteccion_escaneada = ahora
        asyncio.create_task(self.escanear("planta detectada"))
        return True

    # ---------------------------------------------------------------
    # Bucle principal
    # ---------------------------------------------------------------
    async def _bucle(self):
        try:
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
        """
        i = self.indice % len(self.waypoints)
        if not await self._enviar(wp):
            self.estado = "Nav2 no disponible"
            self._emitir({"evento": "nav2_caido"})
            return "sin_nav2"

        self.estado = "navegando"
        self._emitir({"evento": "rumbo_a", "indice": i, "waypoint": wp})

        inicio = time.monotonic()
        while self.activo:
            await asyncio.sleep(TICK_S)

            if self._escaneando:
                inicio = time.monotonic()     # el tiempo barriendo no cuenta
                continue

            if time.monotonic() - self.ultimo_escaneo >= SCAN_PERIODO_S:
                await self.escanear("periódico")
                continue

            if self._meta_cancelada:
                # La canceló un escaneo (del reloj o de una detección).
                if not await self._enviar(wp):
                    return "sin_nav2"
                inicio = time.monotonic()
                continue

            estado = self._estado_meta()
            if estado == "exito" or self._distancia_a(wp) <= WAYPOINT_TOL_M:
                self._emitir({"evento": "llegado", "indice": i})
                return "llegado"

            if estado in ("abortada", "rechazada", "cancelada"):
                # Nav2 ya probó sus maniobras de recuperación (girar,
                # retroceder, esperar) y no encontró camino. Antes la
                # patrulla no se enteraba y el robot quedaba quieto hasta
                # el timeout de 90 s.
                self._emitir({"evento": "meta_abortada", "indice": i})
                await asyncio.sleep(1.0)
                return "abortada"

            if time.monotonic() - inicio > GOAL_TIMEOUT_S:
                # No se aborta la misión: en un cultivo que un punto quede
                # inalcanzable (barro, una rama caída) es normal, y plantarse
                # ahí para siempre es peor que saltárselo.
                self._emitir({"evento": "timeout", "indice": i})
                self._cancel_nav_goal()
                return "timeout"
        return "detenido"

    def _distancia_a(self, wp: dict) -> float:
        pose = self._get_pose()
        if not pose:
            return math.inf
        return math.hypot(pose.get("x", 0.0) - wp["x"], pose.get("y", 0.0) - wp["y"])

    # ---------------------------------------------------------------
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
            "waypoints": self.waypoints,
            "indice": self.indice % len(self.waypoints) if self.waypoints else 0,
            "vueltas": self.vueltas,
            "escaneos": self.escaneos,
            "proximo_escaneo_s": round(restante if self.activo else SCAN_PERIODO_S, 1),
            "auto_activa": self.auto_activa,
            "distancia_auto_m": self._distancia_puntos(),
        }
