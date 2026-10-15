"""
Máquina de estados de exploración — versión explícita y mejorada.
=================================================================

Reemplaza el `if/elif` reactivo de `explorador.ExploradorReactivo` por una
máquina de estados FINITA y EXPLÍCITA. Misma interfaz (drop-in): se construye
con los mismos argumentos y expone `iniciar/detener/snapshot/ceder_mando` y los
atributos `activo/estado/aprendidas`, así el simulador (`correr.py`) y el
servidor (`jetson_server.py`) pueden cambiar a ella sin tocar nada más:

    from maquina_estados import MaquinaEstados as ExploradorReactivo

Por qué una FSM y no el cascada de `if`:
  · Cada decisión queda con NOMBRE y con una transición explícita → se puede
    depurar, dibujar y, sobre todo, correlacionar con el entrenamiento: cada
    tick se emite (estado, motivo_transición, percepción), que es justo lo que
    el dataset necesita para aprender "en esta situación, este estado".
  · Se añaden capacidades nuevas sin ensuciar el lazo: ESCANEO de una planta
    cuando la visión detecta algo (gancho `detecciones`) y seguimiento de
    COBERTURA (qué celdas ya se pisaron) para no vagar ni repetir.

Las CONSTANTES y el orden de prioridad de la seguridad se conservan de
`explorador.py` (están validadas con experimentos del simulador, fechados): sin
scan fresco no se conduce; el operador manda sobre la regla; atasco antes que
avanzar. Lo aprendido por el operador y la política siguen por encima de la
regla, y la capa de freno (cuando el usuario la reactive) sigue por debajo:
esta FSM PROPONE pulsos, no evita los choques por su cuenta.
"""
import asyncio
import math
import time
import traceback
from collections import deque
from enum import Enum

TICK_S = 0.25

SCAN_VIEJO_S = 1.0
ATASCO_S = 25.0
ATASCO_MOVIMIENTO_M = 0.2
DESATASCO_GIROS = 3
MARGEN_MANIOBRA_M = 0.35
GIROS_CALLEJON = 8
PAUSA_GIRO_S = 1.0
CELDA_M = 0.5
REVISITA_S = 15.0
REVISITAS_BUCLE = 3
COMPROMISO_PULSOS = 5
# El sesgo de cobertura (ir a lo menos pisado) SOLO desempata cuando los dos
# lados están parejos: si uno está más de esto más abierto que el otro, manda
# el hueco, no la cobertura. Sin esta guarda, en una trampa de bucle el robot
# se metía al lado estrecho por cubrir y chocaba (A/B sim 2026-10-07: 7→15
# choques). La cobertura no vale un choque.
COBERTURA_EMPATE_M = 0.30
MANIOBRA_EN_CORRECCION = False
PULSOS_ENTRE_CORRECCIONES = 1
FRENTE_CALLEJON_M = 0.45

# Escaneo de planta al detectar: no interrumpir el barrido periódico ni
# re-escanear la misma planta sin haberse movido.
PLANTA_CONF_MIN = 0.60
PLANTA_REFRACTARIO_S = 12.0
PLANTA_MOV_MIN_M = 0.4


class Estado(str, Enum):
    DETENIDO = "detenido"
    EXPLORANDO = "explorando"
    GIRANDO = "girando"
    CALLEJON = "saliendo del callejón"
    RETROCEDER = "retrocediendo"
    DESATASCAR = "desatascando"
    ESCANEAR = "escaneando (periódico)"
    ESCANEAR_PLANTA = "escaneando planta detectada"
    CORRIGIENDO_SENDERO = "corrigiendo sendero"
    APRENDIDO = "ejecutando lo aprendido"
    CEDIDO = "el operador tiene el mando"
    SIN_LIDAR = "pausada: sin datos del LIDAR"
    ERROR = "error"


class MaquinaEstados:
    def __init__(self, *, distancias, get_pose, conducir, parar,
                 escanear, on_event, grabar_recorrido, edad_scan, get_config,
                 seguidor=None, aprendizaje=None, sensores=None, politica=None,
                 get_costmap=None, rumbo_objetivo=None,
                 detecciones=None, escanear_planta=None, marcar_cobertura=None):
        # --- mismas dependencias que ExploradorReactivo ---
        self._seguidor = seguidor
        self._aprendizaje = aprendizaje
        self._sensores = sensores
        self._politica = politica
        self._get_config = get_config
        self._distancias = distancias
        self._edad_scan = edad_scan
        self._get_pose = get_pose
        self._conducir_cb = conducir
        self._parar = parar
        self._escanear = escanear
        self._on_event = on_event
        self._grabar_recorrido = grabar_recorrido
        self._get_costmap = get_costmap or (lambda: None)
        self._rumbo_objetivo = rumbo_objetivo
        # --- nuevas, opcionales (None = comportamiento de antes) ---
        # detecciones() -> dict|None con {"clase","confianza",...} de lo mejor
        # que ve la visión ahora mismo (plaga/planta). Si None, no se escanea.
        self._detecciones = detecciones
        # escanear_planta(det) corutina: barrido/zoom/clip sobre la planta.
        # Si None, se usa el barrido periódico normal (`escanear`).
        self._escanear_planta_cb = escanear_planta or escanear
        # marcar_cobertura(pose) -> None: registra la celda cubierta fuera.
        self._marcar_cobertura = marcar_cobertura

        # --- estado público (compatible con el anterior) ---
        self.activo = False
        self.estado = Estado.DETENIDO.value
        self.pulsos = self.giros = self.desatascos = self.escaneos = 0
        self.correcciones = self.bucles = self.callejones = self.aprendidas = 0
        self.pulsos_politica = 0
        self.escaneos_planta = 0

        # --- internos ---
        self._fsm = Estado.DETENIDO
        self._tarea: asyncio.Task | None = None
        self._ultimo_escaneo = 0.0
        self._ultimo_lado = "izquierda"
        self._memoria: dict[tuple[int, int], tuple[int, float]] = {}
        self._cobertura: set[tuple[int, int]] = set()
        self._forzar_lado = None
        self._compromiso = 0
        self._sin_corregir = 0
        self._mando_cedido_hasta = 0.0
        self._ref_pose = None
        self._ref_t = 0.0
        self._ultima_planta_t = 0.0
        self._ultima_planta_pose = None
        # Telemetría para correlacionar con el entrenamiento: últimas
        # transiciones (estado_desde, estado_hacia, motivo, firma_percepción).
        self._transiciones: deque = deque(maxlen=60)
        self._t_estado = 0.0

    # ------------------------------------------------------------------ API
    def iniciar(self) -> tuple[bool, str]:
        if self.activo:
            return False, "la exploración ya está corriendo"
        if self._edad_scan() > SCAN_VIEJO_S:
            return False, "sin datos del LIDAR — revisá el cable y que el SLAM esté corriendo"
        self.activo = True
        self.pulsos = self.giros = self.desatascos = self.escaneos = 0
        self.correcciones = self.bucles = self.callejones = self.aprendidas = 0
        self.pulsos_politica = self.escaneos_planta = 0
        self._memoria.clear()
        self._cobertura.clear()
        self._forzar_lado, self._compromiso, self._sin_corregir = None, 0, 0
        self._ultimo_escaneo = 0.0
        self._ref_pose, self._ref_t = None, time.monotonic()
        self._transiciones.clear()
        self._grabar_recorrido(True)
        self._ir_a(Estado.EXPLORANDO, "inicio")
        self._tarea = asyncio.create_task(self._bucle())
        return True, "explorando con máquina de estados: avanza, cubre y mapea"

    def detener(self):
        self.activo = False
        if self._tarea is not None:
            self._tarea.cancel()
            self._tarea = None
        self._grabar_recorrido(False)
        self._parar()
        self._ir_a(Estado.DETENIDO, "detener")

    def ceder_mando(self, segundos: float):
        self._mando_cedido_hasta = time.monotonic() + segundos
        self._ir_a(Estado.CEDIDO, "operador")

    def snapshot(self) -> dict:
        d = {"activo": self.activo, "estado": self.estado, "fsm": self._fsm.name,
             "pulsos": self.pulsos, "giros": self.giros,
             "desatascos": self.desatascos, "escaneos": self.escaneos,
             "escaneos_planta": self.escaneos_planta,
             "correcciones": self.correcciones, "bucles": self.bucles,
             "callejones": self.callejones, "aprendidas": self.aprendidas,
             "pulsos_politica": self.pulsos_politica,
             "cobertura_celdas": len(self._cobertura),
             "transiciones": list(self._transiciones)[-12:]}
        if self._seguidor is not None:
            d["sendero"] = self._seguidor.snapshot()
        return d

    # ----------------------------------------------------------- utilidades
    def _emitir(self, **data):
        try:
            self._on_event(data)
        except Exception:
            pass

    def _ir_a(self, nuevo: Estado, motivo: str):
        """Transición explícita con telemetría (lo que el dataset necesita)."""
        if nuevo is not self._fsm:
            self._transiciones.append({
                "t": round(time.monotonic(), 2),
                "de": self._fsm.name, "a": nuevo.name, "motivo": motivo})
            self._emitir(evento="transicion", de=self._fsm.name,
                         a=nuevo.name, motivo=motivo)
            self._fsm = nuevo
            self._t_estado = time.monotonic()
        self.estado = nuevo.value

    def _hueco(self, d, lado) -> float:
        v = d.get(lado)
        return math.inf if v is None else v

    def _conducir(self, accion, maniobra=True):
        self._conducir_cb(accion, maniobra)

    def _percepcion(self) -> dict | None:
        if self._sensores is not None:
            return self._sensores()
        d = self._distancias()
        v = getattr(self._seguidor, "vision", None)
        return {"lidar_adelante_m": d.get("adelante"), "lidar_atras_m": d.get("atras"),
                "lidar_izquierda_m": d.get("izquierda"), "lidar_derecha_m": d.get("derecha"),
                "sendero_lateral_m": getattr(v, "lateral_m", None)}

    def _hay_sitio_para_maniobra(self, d) -> bool:
        return (self._hueco(d, "adelante") >= MARGEN_MANIOBRA_M
                and self._hueco(d, "atras") >= MARGEN_MANIOBRA_M)

    def _lado_costmap(self) -> str | None:
        costmap = self._get_costmap()
        if costmap is None or costmap._scan_count < 3:
            return None
        centro, ancho = costmap.hueco_libre()
        if centro is None or ancho is None or abs(centro) < 0.05:
            return None
        return "izquierda" if centro > 0 else "derecha"

    def _lado_objetivo(self) -> str | None:
        if self._rumbo_objetivo is None:
            return None
        info = self._rumbo_objetivo()
        if info is None:
            return None
        ang, dist = info
        if dist < 0.3 or abs(ang) < 15:
            return None
        return "izquierda" if ang > 0 else "derecha"

    def _mejor_lado(self, d) -> str:
        """objetivo > costmap > cobertura (menos pisado) > más hueco."""
        if self._forzar_lado:
            return self._forzar_lado
        lado_libre = self._get_config()["lado_libre_cm"] / 100.0
        lado = self._lado_objetivo()
        if lado and self._hueco(d, lado) >= lado_libre:
            return lado
        lado = self._lado_costmap()
        if lado and self._hueco(d, lado) >= lado_libre:
            return lado
        izq, der = self._hueco(d, "izquierda"), self._hueco(d, "derecha")
        # Sesgo de COBERTURA, pero solo como DESEMPATE seguro: los dos lados
        # despejados y con hueco parejo (ver COBERTURA_EMPATE_M). Si uno está
        # claramente más abierto, manda el hueco (no meterse a lo estrecho por
        # cubrir). Con un lado "infinito" (nada a la vista) no hay empate.
        pose = self._get_pose()
        if (pose is not None and izq >= lado_libre and der >= lado_libre
                and math.isfinite(izq) and math.isfinite(der)
                and abs(izq - der) <= COBERTURA_EMPATE_M):
            lado_cob = self._lado_menos_cubierto(pose)
            if lado_cob:
                return lado_cob
        return "izquierda" if izq >= der else "derecha"

    def _celda(self, pose) -> tuple[int, int]:
        return (round(pose["x"] / CELDA_M), round(pose["y"] / CELDA_M))

    def _lado_menos_cubierto(self, pose) -> str | None:
        rumbo = math.radians(pose.get("theta_deg", 0.0))
        c = self._celda(pose)
        pesos = {}
        for lado, signo in (("izquierda", 1), ("derecha", -1)):
            ang = rumbo + signo * math.pi / 2
            cx = c[0] + round(2 * math.cos(ang))
            cy = c[1] + round(2 * math.sin(ang))
            pesos[lado] = 1 if (cx, cy) in self._cobertura else 0
        if pesos["izquierda"] == pesos["derecha"]:
            return None
        return min(pesos, key=pesos.get)

    def _registrar_celda(self):
        pose = self._get_pose()
        if pose is None:
            return
        celda = self._celda(pose)
        self._cobertura.add(celda)
        if self._marcar_cobertura is not None:
            try:
                self._marcar_cobertura(pose)
            except Exception:
                pass
        ahora = time.monotonic()
        veces, ultimo = self._memoria.get(celda, (0, -1e9))
        if ahora - ultimo > REVISITA_S:
            veces += 1
            if veces >= REVISITAS_BUCLE:
                self.bucles += 1
                self._forzar_lado = self._lado_menos_visitado(pose, celda)
                self._compromiso = COMPROMISO_PULSOS
                self._emitir(evento="bucle", celda=list(celda), visitas=veces,
                             lado=self._forzar_lado)
        self._memoria[celda] = (veces, ahora)

    def _lado_menos_visitado(self, pose, celda) -> str:
        rumbo = math.radians(pose.get("theta_deg", 0.0))
        pesos = {}
        for lado, signo in (("izquierda", 1), ("derecha", -1)):
            ang = rumbo + signo * math.pi / 2
            cx = celda[0] + round(2 * math.cos(ang))
            cy = celda[1] + round(2 * math.sin(ang))
            pesos[lado] = self._memoria.get((cx, cy), (0, 0))[0]
        return min(pesos, key=pesos.get)

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

    def _planta_a_la_vista(self) -> dict | None:
        """La visión ve una planta/plaga con confianza y hace rato que no
        escaneamos una, y nos movimos desde la última: vale escanearla."""
        if self._detecciones is None:
            return None
        ahora = time.monotonic()
        if ahora - self._ultima_planta_t < PLANTA_REFRACTARIO_S:
            return None
        pose = self._get_pose()
        if (self._ultima_planta_pose is not None and pose is not None
                and math.hypot(pose["x"] - self._ultima_planta_pose[0],
                               pose["y"] - self._ultima_planta_pose[1]) < PLANTA_MOV_MIN_M):
            return None
        try:
            det = self._detecciones()
        except Exception:
            return None
        if det and det.get("confianza", 0.0) >= PLANTA_CONF_MIN:
            return det
        return None

    async def _probar_aprendido(self, apurado=True) -> bool:
        if self._aprendizaje is None:
            return False
        sensores = self._percepcion()
        m = self._aprendizaje.consultar(sensores, self.estado)
        if m is None or (not apurado and m.exitos < 1):
            return False
        self.aprendidas += 1
        self._ir_a(Estado.APRENDIDO, m.firma)
        self.estado = f"aprendido: {' → '.join(p['accion'] for p in m.pasos)}"
        self._emitir(evento="usa_aprendido", firma=m.firma,
                     pasos=[p["accion"] for p in m.pasos], confianza=round(m.confianza, 2))
        self._aprendizaje.aplicada(m, sensores, ahora=time.monotonic())
        for paso in m.pasos:
            if not self.activo:
                return True
            self._conducir(paso["accion"], maniobra=paso.get("maniobra", True))
            await asyncio.sleep(PAUSA_GIRO_S)
        return True

    def _correccion_sendero(self):
        if self._seguidor is None:
            return None
        try:
            import cv2
            return self._seguidor.correccion(cv2, self._distancias())
        except Exception as exc:
            self._emitir(evento="error_sendero", detalle=str(exc))
            return None

    # --------------------------------------------------------------- estados
    async def _salir_callejon(self, d):
        self.callejones += 1
        lado = self._mejor_lado(d)
        self._ultimo_lado = lado
        avance_libre = self._get_config()["avance_libre_cm"] / 100.0
        self._ir_a(Estado.CALLEJON, f"girar {lado}")
        self.estado = f"callejón: girando a la {lado} hasta encontrar salida"
        self._emitir(evento="callejon", lado=lado, total=self.callejones)
        for _ in range(GIROS_CALLEJON):
            if not self.activo:
                return
            if self._hueco(self._distancias(), "adelante") >= avance_libre:
                return
            self._conducir(lado, maniobra=False)
            self.giros += 1
            await asyncio.sleep(PAUSA_GIRO_S)

    async def _desatascar(self):
        self.desatascos += 1
        self._ir_a(Estado.DESATASCAR, "pose quieta")
        self._emitir(evento="desatasco", total=self.desatascos)
        d = self._distancias()
        if self._hueco(d, "atras") >= self._get_config()["retroceso_libre_cm"] / 100.0:
            self._conducir("atras")
            await asyncio.sleep(1.5)
        lado_cm = self._lado_costmap()
        contrario = lado_cm if lado_cm is not None else (
            "derecha" if self._ultimo_lado == "izquierda" else "izquierda")
        self._ultimo_lado = contrario
        for _ in range(DESATASCO_GIROS):
            if not self.activo:
                return
            self._conducir(contrario, maniobra=self._hay_sitio_para_maniobra(self._distancias()))
            self.giros += 1
            await asyncio.sleep(1.3)
        self._ref_pose, self._ref_t = None, time.monotonic()

    async def _escanear_periodico(self):
        self._ir_a(Estado.ESCANEAR, "periódico")
        self._parar()
        await self._escanear("exploración")
        self._ultimo_escaneo = time.monotonic()
        self.escaneos += 1
        self._ref_t = time.monotonic()

    async def _hacer_escaneo_planta(self, det):
        """Para, escanea la planta detectada y vuelve. El callback puede tener
        firma `fn(det)` o `fn(motivo)`: se tolera cualquiera de las dos."""
        self._ir_a(Estado.ESCANEAR_PLANTA, det.get("clase", "planta"))
        self.estado = f"escaneando: {det.get('clase', 'planta')} ({det.get('confianza', 0):.2f})"
        self._emitir(evento="escaneo_planta", clase=det.get("clase"),
                     confianza=round(det.get("confianza", 0.0), 2))
        self._parar()
        try:
            await self._escanear_planta_cb(det)
        except TypeError:
            await self._escanear_planta_cb(f"planta:{det.get('clase', '?')}")
        self.escaneos_planta += 1
        self._ultima_planta_t = time.monotonic()
        pose = self._get_pose()
        if pose is not None:
            self._ultima_planta_pose = (pose["x"], pose["y"])

    # --------------------------------------------------------------- bucle
    async def _bucle(self):
        try:
            while self.activo:
                await asyncio.sleep(TICK_S)

                # 1) Barrido periódico (prioridad alta, como antes).
                if time.monotonic() - self._ultimo_escaneo >= self._get_config()["escaneo_periodo_s"]:
                    await self._escanear_periodico()
                    continue

                # 2) Sin LIDAR fresco: no se conduce a ciegas.
                if self._edad_scan() > SCAN_VIEJO_S:
                    self._ir_a(Estado.SIN_LIDAR, "scan viejo")
                    self._parar()
                    continue

                # 3) El operador tiene prioridad mientras corrige.
                if time.monotonic() < self._mando_cedido_hasta:
                    self._ir_a(Estado.CEDIDO, "operador")
                    continue

                # 4) Planta a la vista → escanearla (si hay gancho de visión).
                det = self._planta_a_la_vista()
                if det is not None:
                    await self._hacer_escaneo_planta(det)
                    continue

                # 5) Atascado → aprendido o desatasco.
                if self._atascado():
                    if not await self._probar_aprendido():
                        await self._desatascar()
                    continue

                self._registrar_celda()
                await self._decidir_movimiento()
        except asyncio.CancelledError:  # noqa: PERF203
            raise
        except Exception as exc:
            self.activo = False
            self._ir_a(Estado.ERROR, str(exc))
            self.estado = f"error: {exc}"
            self._parar()
            self._emitir(evento="error", detalle=str(exc))
            print(f"[fsm] bucle abortado: {type(exc).__name__}: {exc}\n"
                  + traceback.format_exc(), flush=True)

    async def _decidir_movimiento(self):
        cfg = self._get_config()
        avance_libre = cfg["avance_libre_cm"] / 100.0
        lado_libre = cfg["lado_libre_cm"] / 100.0
        retro_libre = cfg["retroceso_libre_cm"] / 100.0

        d = self._distancias()
        frente = self._hueco(d, "adelante")
        izq, der = self._hueco(d, "izquierda"), self._hueco(d, "derecha")

        if frente >= avance_libre:
            if await self._probar_aprendido(apurado=False):
                return
            correccion = (self._correccion_sendero()
                          if self._sin_corregir >= PULSOS_ENTRE_CORRECCIONES else None)
            if correccion is not None:
                self._ir_a(Estado.CORRIGIENDO_SENDERO, self._seguidor.estado)
                self.estado = f"sendero: {self._seguidor.estado}"
                self._conducir(correccion, maniobra=MANIOBRA_EN_CORRECCION)
                self.correcciones += 1
                self._sin_corregir = 0
                return
            if self._politica is not None and self._politica.disponible:
                accion_politica, seguridad = self._politica.decidir(self._percepcion())
                if accion_politica in ("adelante", "izquierda", "derecha"):
                    self._ir_a(Estado.EXPLORANDO, f"política {seguridad:.2f}")
                    self.estado = f"política: {accion_politica} ({seguridad:.2f})"
                    self._conducir(accion_politica, maniobra=False)
                    self.pulsos_politica += 1
                    self.pulsos += 1
                    self._sin_corregir += 1
                    return
            self._ir_a(Estado.EXPLORANDO, "frente libre")
            self.estado = f"avanzando ({frente:.2f} m libres)"
            self._conducir("adelante")
            self.pulsos += 1
            self._sin_corregir += 1
            if self._compromiso:
                self._compromiso -= 1
                if not self._compromiso:
                    self._forzar_lado = None
            return

        if frente < FRENTE_CALLEJON_M and izq < lado_libre and der < lado_libre:
            if not await self._probar_aprendido():
                await self._salir_callejon(d)
            return

        lado = self._mejor_lado(d)
        if self._hueco(d, lado) < lado_libre:
            lado = "izquierda" if izq >= der else "derecha"
        if self._hueco(d, lado) >= lado_libre:
            maniobra = self._hay_sitio_para_maniobra(d)
            self._ir_a(Estado.GIRANDO, f"{lado}")
            self.estado = (f"girando a la {lado} (frente {frente:.2f} m"
                           + ("" if maniobra else ", pivote") + ")")
            self._ultimo_lado = lado
            self._conducir(lado, maniobra=maniobra)
            self.giros += 1
            return

        if self._hueco(d, "atras") >= retro_libre:
            self._ir_a(Estado.RETROCEDER, "sin salida")
            self.estado = "sin salida: retrocediendo"
            self._conducir("atras")
            return

        if await self._probar_aprendido():
            return
        self._ir_a(Estado.GIRANDO, "encerrado")
        self.estado = "encerrado: girando en el sitio"
        self._conducir(self._ultimo_lado, maniobra=False)
        self.giros += 1
