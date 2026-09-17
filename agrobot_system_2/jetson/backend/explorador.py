"""
Modo exploración: el robot recorre solo, esquiva y deja que el SLAM mapee.
=========================================================================

Es autonomía REACTIVA: no planifica rutas ni necesita un mapa previo ni
waypoints. Mira las distancias del LIDAR, decide, y ejecuta un pulso. Se
eligió así, y no con Nav2, por cómo es este robot:

  · Los motores no tienen gama lenta (PWM 200..255 en el firmware): cualquier
    orden sale al 78-100 % de potencia. Nav2 calcula velocidades finas que
    este chasis no sabe ejecutar.
  · Al parar, las ruedas quedan sueltas y el robot desliza. Avanzar a
    pulsos cortos con pausas para mirar es lo que el hardware permite.

Ciclo, cuatro veces por segundo:
  1. Mirar el hueco libre adelante, a los lados y atrás (corredor del ancho
     del robot, ver slam.py).
  2. Si hay sitio delante -> pulso de avance.
     Si no, girar hacia el lado más despejado con la maniobra de tierra
     blanda (arco adelante + arco atrás).
     Si no hay sitio en ninguno de los tres -> retroceder y volver a mirar.
  3. Cada SCAN_PERIODO_S se para y barre con la cámara.
  4. Si la pose no cambia en ATASCO_S, se da por atascado y ejecuta un
     desatasco (retroceder + girar al lado contrario del último intento).

Mientras explora deja puntos del recorrido cada 1,5 m (la misma grabación
automática que en modo manual), así el mapa nuevo queda con su ruta.
"""
import asyncio
import math
import time

TICK_S = 0.25

# Umbrales de decisión, en metros de hueco LIBRE desde la carrocería.
AVANCE_LIBRE_M = 1.00     # sitio suficiente para lanzar un pulso de avance
LADO_LIBRE_M = 0.60       # un lado con al menos esto se considera salida
RETROCESO_LIBRE_M = 0.50  # hace falta esto detrás para poder retroceder

SCAN_PERIODO_S = 120.0
SCAN_SWEEP_S = 10.0

SCAN_VIEJO_S = 1.0        # sin scan más reciente que esto, no se explora
ATASCO_S = 25.0           # sin moverse tanto tiempo = atascado
ATASCO_MOVIMIENTO_M = 0.2
DESATASCO_GIROS = 3       # pulsos de giro seguidos al desatascarse


class ExploradorReactivo:
    def __init__(self, *, distancias, get_pose, conducir, parar,
                 escanear, on_event, grabar_recorrido, edad_scan):
        self._distancias = distancias      # () -> {adelante, atras, izquierda, derecha}
        self._edad_scan = edad_scan        # () -> segundos desde el último /scan
        self._get_pose = get_pose          # () -> {x, y, theta_deg} | None
        self._conducir = conducir          # (accion) -> None  (pulso)
        self._parar = parar                # () -> None
        self._escanear = escanear          # corutina(motivo)
        self._on_event = on_event          # (dict)
        self._grabar_recorrido = grabar_recorrido   # (bool) -> (ok, msg)

        self.activo = False
        self.estado = "detenido"
        self.pulsos = 0
        self.giros = 0
        self.desatascos = 0
        self.escaneos = 0
        self._tarea: asyncio.Task | None = None
        self._ultimo_escaneo = 0.0
        self._ultimo_lado = "izquierda"
        self._ref_pose = None
        self._ref_t = 0.0

    # ------------------------------------------------------------------
    def iniciar(self) -> tuple[bool, str]:
        if self.activo:
            return False, "la exploración ya está corriendo"
        # Sin LIDAR fresco el robot estaría avanzando a ciegas, y con 80 cm
        # de deslizamiento eso termina en golpe.
        if self._edad_scan() > SCAN_VIEJO_S:
            return False, "sin datos del LIDAR — revisá el cable y que el SLAM esté corriendo"
        self.activo = True
        self.pulsos = self.giros = self.desatascos = self.escaneos = 0
        self._ultimo_escaneo = 0.0        # primer barrido nada más arrancar
        self._ref_pose, self._ref_t = None, time.monotonic()
        self._grabar_recorrido(True)      # deja el rastro del recorrido
        self._tarea = asyncio.create_task(self._bucle())
        return True, "explorando: avanza, esquiva y mapea"

    def detener(self):
        self.activo = False
        if self._tarea is not None:
            self._tarea.cancel()
            self._tarea = None
        self._grabar_recorrido(False)
        self._parar()
        self.estado = "detenido"

    def snapshot(self) -> dict:
        return {"activo": self.activo, "estado": self.estado, "pulsos": self.pulsos,
                "giros": self.giros, "desatascos": self.desatascos,
                "escaneos": self.escaneos}

    # ------------------------------------------------------------------
    def _emitir(self, **data):
        try:
            self._on_event(data)
        except Exception:
            pass

    def _hueco(self, d, lado) -> float:
        """None en las distancias significa "nada a la vista" = libre."""
        v = d.get(lado)
        return math.inf if v is None else v

    def _atascado(self) -> bool:
        pose = self._get_pose()
        ahora = time.monotonic()
        if pose is None:
            return False
        if self._ref_pose is None:
            self._ref_pose, self._ref_t = (pose["x"], pose["y"]), ahora
            return False
        if math.hypot(pose["x"] - self._ref_pose[0],
                      pose["y"] - self._ref_pose[1]) > ATASCO_MOVIMIENTO_M:
            self._ref_pose, self._ref_t = (pose["x"], pose["y"]), ahora
            return False
        return ahora - self._ref_t > ATASCO_S

    async def _bucle(self):
        try:
            while self.activo:
                await asyncio.sleep(TICK_S)

                if time.monotonic() - self._ultimo_escaneo >= SCAN_PERIODO_S:
                    self.estado = "escaneando"
                    self._parar()
                    await self._escanear("exploración")
                    self._ultimo_escaneo = time.monotonic()
                    self.escaneos += 1
                    # el tiempo parado barriendo no cuenta como atasco
                    self._ref_t = time.monotonic()
                    continue

                # El LIDAR puede caerse en marcha (el driver murió 34 veces
                # en un arranque): si eso pasa, parar, no seguir a ciegas.
                if self._edad_scan() > SCAN_VIEJO_S:
                    self.estado = "pausada: sin datos del LIDAR"
                    self._parar()
                    continue

                if self._atascado():
                    await self._desatascar()
                    continue

                d = self._distancias()
                frente = self._hueco(d, "adelante")
                izq, der = self._hueco(d, "izquierda"), self._hueco(d, "derecha")

                if frente >= AVANCE_LIBRE_M:
                    self.estado = f"avanzando ({frente:.2f} m libres)"
                    self._conducir("adelante")
                    self.pulsos += 1
                    continue

                # Sin sitio delante: girar hacia donde más hueco haya. Se
                # recuerda el lado para no quedar oscilando izquierda-derecha.
                lado = "izquierda" if izq >= der else "derecha"
                if self._hueco(d, lado) >= LADO_LIBRE_M:
                    self.estado = f"girando a la {lado} (frente {frente:.2f} m)"
                    self._ultimo_lado = lado
                    self._conducir(lado)
                    self.giros += 1
                    continue

                if self._hueco(d, "atras") >= RETROCESO_LIBRE_M:
                    self.estado = "sin salida: retrocediendo"
                    self._conducir("atras")
                    continue

                self.estado = "encerrado: girando en el sitio"
                self._conducir(self._ultimo_lado)
                self.giros += 1
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.activo = False
            self.estado = f"error: {exc}"
            self._parar()
            self._emitir(evento="error", detalle=str(exc))
            print(f"[exploración] bucle abortado: {type(exc).__name__}: {exc}", flush=True)

    async def _desatascar(self):
        """El robot cree que se mueve pero la pose no cambia: rueda patinando,
        enganchado en una planta o contra algo que el LIDAR no ve (hueco bajo,
        rama fina). Se retrocede y se gira al lado contrario del último."""
        self.desatascos += 1
        self.estado = "desatascando"
        self._emitir(evento="desatasco", total=self.desatascos)
        d = self._distancias()
        if self._hueco(d, "atras") >= RETROCESO_LIBRE_M:
            self._conducir("atras")
            await asyncio.sleep(1.5)
        contrario = "derecha" if self._ultimo_lado == "izquierda" else "izquierda"
        self._ultimo_lado = contrario
        for _ in range(DESATASCO_GIROS):
            if not self.activo:
                return
            self._conducir(contrario)
            self.giros += 1
            await asyncio.sleep(1.3)
        self._ref_pose, self._ref_t = None, time.monotonic()
