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

Mientras explora deja puntos del recorrido cada tantos centímetros (la misma
grabación automática que en modo manual), así el mapa nuevo queda con su ruta.
"""
import asyncio
import math
import time
import traceback

try:                                    # E11 (2026-10-26): planificador A* para el regreso al inicio
    from planificador import Planificador as _Planificador
except Exception:                       # noqa: BLE001
    _Planificador = None

try:
    import rumbo_fusion as _RF
except ImportError:                    # sin el módulo el explorador sigue como siempre
    _RF = None

TICK_S = 0.25

# Los umbrales de decisión (cuánto hueco hace falta para avanzar, girar o
# retroceder, y cada cuánto barrer) los fija el operador desde el dashboard:
# sección `autonomia` de la config, en centímetros.

SCAN_VIEJO_S = 1.0        # sin scan más reciente que esto, no se explora
ATASCO_S = 25.0           # sin moverse tanto tiempo = atascado
ATASCO_MOVIMIENTO_M = 0.2
DESATASCO_GIROS = 3       # pulsos de giro seguidos al desatascarse

# La maniobra de giro (arco adelante + arco atrás, para tierra blanda) SOLO
# vale con sitio por delante y por detrás: con una pared cerca el robot gira
# rozándola. Medido en el simulador el 2026-09-20: 43 s de roce en el
# escenario `callejon`. Sin sitio se gira sobre el eje, que no traslada.
MARGEN_MANIOBRA_M = 0.35
# Salir de un callejón: pivotar hasta que haya salida, sin volver a decidir
# en cada tick (antes oscilaba entre girar y retroceder).
GIROS_CALLEJON = 8
PAUSA_GIRO_S = 1.0

# Detección de "estoy dando vueltas": volver a una celda de este tamaño
# después de haber estado fuera un rato.
CELDA_M = 0.5
REVISITA_S = 15.0
REVISITAS_BUCLE = 3
# Al detectar el bucle, se insiste en el lado menos visitado durante varios
# pulsos en vez de volver a elegir con el mismo criterio de siempre.
COMPROMISO_PULSOS = 5

# Recentrarse en el sendero es corregir el RUMBO: con pivote el robot no
# traslada (y en el campo no pisa las plantas). Ver la comparación del
# 2026-09-20 en el simulador antes de cambiarlo.
MANIOBRA_EN_CORRECCION = True
# No corregir dos veces seguidas: entre corrección y corrección va al menos un
# pulso de avance. Si no, el robot se pasa el rato pivotando sin recorrer
# (102 correcciones y la mitad de distancia en el simulador, 2026-09-20).
PULSOS_ENTRE_CORRECCIONES = 1
# Parada por incertidumbre: cuánto se espera antes de volver a medir el
# sendero. Lo bastante para que la cámara dé fotogramas nuevos y el LIDAR un
# barrido, sin dejar el robot clavado si la referencia vuelve enseguida.
ESPERA_INCERTIDUMBRE_S = 1.0
# Un callejón es que NO se pueda seguir, no que haya poco sitio: en un pasillo
# estrecho los dos lados siempre están cerca y se confundía con un callejón.
FRENTE_CALLEJON_M = 0.45


class ExploradorReactivo:
    escaneo_al_iniciar = True       # False: iniciar() NO lanza el barrido inmediato (reinicios de la misión / reanudar)

    def __init__(self, *, distancias, get_pose, conducir, parar,
                 escanear, on_event, grabar_recorrido, edad_scan, get_config,
                 seguidor=None, aprendizaje=None, sensores=None, politica=None,
                 get_costmap=None, rumbo_objetivo=None):
        self._seguidor = seguidor          # SeguidorSendero | None
        self._aprendizaje = aprendizaje
        self._sensores = sensores          # () -> dict con lo que percibe el robot
        self._politica = politica
        self.pulsos_politica = 0
        self._get_config = get_config      # () -> sección `autonomia` de la config
        self._distancias = distancias      # () -> {adelante, atras, izquierda, derecha}
        self._edad_scan = edad_scan        # () -> segundos desde el último /scan
        self._get_pose = get_pose          # () -> {x, y, theta_deg} | None
        self._conducir_cb = conducir       # (accion, maniobra) -> None  (pulso)
        self._parar = parar                # () -> None
        self._escanear = escanear          # corutina(motivo)
        self._on_event = on_event          # (dict)
        self._grabar_recorrido = grabar_recorrido   # (bool) -> (ok, msg)
        self._get_costmap = get_costmap or (lambda: None)
        # () -> (angulo_deg, dist_m) | None — si se quiere navegar HACIA un
        # punto concreto (modo patrulla reactiva).
        self._rumbo_objetivo = rumbo_objetivo

        self.activo = False
        self.pausado = False                # mando manual encima: sigue «activo» pero no decide ni mueve
        self.estado = "detenido"
        self.pulsos = 0
        self.giros = 0
        self.desatascos = 0
        self.escaneos = 0
        self.correcciones = 0
        self.paradas_inciertas = 0
        self.paradas_estrechas = 0
        self.avisos_estrechos = 0
        self._tarea: asyncio.Task | None = None
        self._ultimo_escaneo = 0.0
        self._ultimo_lado = "izquierda"
        self._memoria: dict[tuple[int, int], tuple[int, float]] = {}
        self._forzar_lado = None
        self._compromiso = 0
        self._sin_corregir = 0
        self._mando_cedido_hasta = 0.0
        self.aprendidas = 0
        self.bucles = 0
        self.callejones = 0
        self._ref_pose = None
        self._ref_t = 0.0
        self._rumbo_ref = None             # rumbo (rad) que se quiere mantener en el tramo recto actual
        self._rumbo_corr = None            # rumbo_fusion.CorrectorRumbo
        self._rumbo_pend = False           # hay un pulso recto cuya caída de rumbo aún no se midió
        self._corr_propia = False          # el pulso en curso es una corrección de rumbo (no reinicia la referencia)
        self.correcciones_rumbo = 0
        # Guardián del sendero (guardian_sendero.py): se asigna desde fuera; sin él (o con autonomia.guardian_sendero_activo=false) no hace nada.
        # Maniobras específicas (2026-10-24, validadas en el simulador por misiones; todas con interruptor en autonomia.mx_*): ir al hueco,
        # rumbo a la meta y escape de nicho.
        self.scan_cb = None                 # () -> [{angle_deg, distance_m}] el último scan del LIDAR (0° = adelante, + = izquierda)
        self._rectos = 0
        self._bloq = 0
        self._hist: list = []
        self._commit = None
        self._commit_hasta = 0.0
        self.mx_escapes = 0
        # --- E11 (2026-10-26): predictor de colisión por pulso, barrido de corredor y regreso planificado al punto de inicio. TODO bajo autonomia.e11_*.
        self.geom_cb = None                 # () -> sección slam de la config (lidar_adelante_cm, lidar_atras_cm, robot_ancho_cm)
        self.plan = None                    # planificador.Planificador (se crea al iniciar con la pose de salida)
        self._inicio_xy = None
        self._t_inicio = 0.0
        self._regresando = False
        self._ruta = None
        self._ruta_t = 0.0
        self._pl_pausa_hasta = 0.0
        self._pl_fallos = 0
        self._rechazos = []                 # instantes (monotonic) de pulsos que el predictor no dejó pasar
        self._relajar_hasta = 0.0
        self.pred_rechazados = {"adelante": 0, "atras": 0, "giro": 0, "cambiados": 0}
        self.regreso_estado = "no"          # no | en curso | completo | sin ruta
        self.guardian = None
        self.sensores_guardian = None       # () -> dict: LIDAR + cámara + IMU (ver jetson_server._sensores_guardian)

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
        self.correcciones = self.bucles = self.callejones = self.aprendidas = 0
        self.paradas_inciertas = 0
        self.paradas_estrechas = 0
        self.avisos_estrechos = 0
        self.pulsos_politica = 0
        self._memoria.clear()
        self._forzar_lado, self._compromiso = None, 0
        # primer barrido nada más arrancar... salvo que quien reinicia (misión tras inspeccionar, reanudar tras un mando manual)
        # diga lo contrario: si no, cada reinicio lanzaba un barrido de 8 s y la misión entraba en un BUCLE DE ESCANEOS (2026-10-23)
        self._ultimo_escaneo = 0.0 if self.escaneo_al_iniciar else time.monotonic()
        self._ref_pose, self._ref_t = None, time.monotonic()
        self._regresando, self._ruta, self.regreso_estado, self._rechazos = False, None, "no", []
        self._t_inicio = time.monotonic()
        pose0 = self._get_pose()
        self._inicio_xy = (pose0["x"], pose0["y"]) if pose0 else None
        if _Planificador is not None and self._get_config().get("e11_activo", False):
            c0 = self._get_config()
            self.plan = _Planificador(inflar_m=float(c0.get("e11_plan_inflar_m", 0.23)), centro=self._inicio_xy or (0.0, 0.0))
        if self.guardian is not None:
            self.guardian.ref, self.guardian.pend = [], None
        self._grabar_recorrido(True)      # deja el rastro del recorrido
        self._tarea = asyncio.create_task(self._bucle())
        return True, "explorando: avanza, esquiva y mapea"

    def pausar(self):
        """El operador toma el mando un momento: el modo autónomo SIGUE activo (la interfaz no cambia a manual)
        pero el explorador no decide ni manda nada hasta `continuar()`."""
        if self.activo and not self.pausado:
            self.pausado = True
            self._t_pausa = time.monotonic()
            self._parar()

    def continuar(self):
        if self.pausado:
            self.pausado = False
            dt = time.monotonic() - getattr(self, "_t_pausa", time.monotonic())
            self._ref_t += dt                # el rato en pausa no cuenta como atasco
            self._ref_pose = None
            self._ultimo_escaneo = max(self._ultimo_escaneo, time.monotonic() - 1.0)   # sin barrido inmediato al volver

    def detener(self):
        self.activo = False
        self.pausado = False
        if self._tarea is not None:
            self._tarea.cancel()
            self._tarea = None
        self._grabar_recorrido(False)
        self._parar()
        self.estado = "detenido"

    def snapshot(self) -> dict:
        d = {"e11": {"regreso": self.regreso_estado, "predictor_rechazados": dict(self.pred_rechazados), "plan": self.plan is not None},
             "activo": self.activo, "estado": self.estado, "pulsos": self.pulsos,
             "giros": self.giros, "desatascos": self.desatascos,
             "escaneos": self.escaneos, "correcciones": self.correcciones,
             "paradas_inciertas": self.paradas_inciertas,
             "paradas_estrechas": self.paradas_estrechas,
             "avisos_estrechos": self.avisos_estrechos,
             "bucles": self.bucles, "callejones": self.callejones,
             "aprendidas": self.aprendidas, "pulsos_politica": self.pulsos_politica}
        if self._seguidor is not None:
            d["sendero"] = self._seguidor.snapshot()
        return d

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

    # ------------------------------------------------------------------ E11: predictor de colisión por pulso
    def _geom(self):
        g = (self.geom_cb() if self.geom_cb else None) or {}
        return (float(g.get("lidar_adelante_cm", 18.5)) / 100.0, float(g.get("lidar_atras_cm", 30.5)) / 100.0, float(g.get("robot_ancho_cm", 38.0)) / 200.0)

    def _despejado(self, clave: str, cfg) -> bool:
        """¿Cabe este pulso ('adelante' | 'atras' | 'giro') sin tocar nada de lo que ve el último scan? Barre la huella del robot (ejes del LIDAR)."""
        if self.scan_cb is None:
            return True                                    # E11 sin LIDAR conectado al explorador (pruebas): no se puede juzgar, no se frena
        scan = self.scan_cb()
        if not scan:
            return False                                   # con LIDAR conectado pero sin scan no se mueve: a ciegas no
        f, a, h = self._geom()
        relajado = time.monotonic() < self._relajar_hasta
        m = float(cfg.get("e11_pred_margen_m", 0.067)) * (0.3 if relajado else 1.0)
        fr = 0.6 if relajado else 1.0
        for q in scan:
            ang = math.radians(q["angle_deg"])
            px, py = q["distance_m"] * math.cos(ang), q["distance_m"] * math.sin(ang)
            if -a - 0.01 <= px <= f + 0.01 and abs(py) <= h + 0.01:
                continue                                    # el propio chasis
            if clave == "adelante":
                if f < px < f + float(cfg.get("e11_pred_avance_m", 0.29)) * fr + m and abs(py) < h + m:
                    return False
            elif clave == "atras":
                if -a - float(cfg.get("e11_pred_atras_m", 0.17)) * fr - m < px < -a and abs(py) < h + m:
                    return False
            else:
                e = m + float(cfg.get("e11_pred_giro_extra_m", 0.033)) * (0.4 if relajado else 1.0)
                if -a - e < px < f + e and abs(py) < h + e:
                    return False
        return True

    def _frente_barrido(self, frente: float, cfg) -> float:
        """E11: el frente se mide también en un CORREDOR del ancho del robot (lo que está a los costados de la trayectoria)."""
        scan = self.scan_cb() if self.scan_cb else None
        if not scan or not cfg.get("e11_barrido", False):
            return frente
        f, _a, _h = self._geom()
        medio = float(cfg.get("e11_barrido_half_m", 0.256))
        # Solo cuenta lo que está DELANTE de la cara frontal (x > f). Con x > 0 una pared a menos de 25 cm del costado (x entre 0 y f, junto al cuerpo)
        # daba un «frente» NEGATIVO y disparaba «esquina: retrocedo» sin tener nada delante (grabaciones del 2026-10-26: frente de −0,18 m).
        xs = [x for x, y in ((q["distance_m"] * math.cos(math.radians(q["angle_deg"])), q["distance_m"] * math.sin(math.radians(q["angle_deg"]))) for q in scan)
              if x > f and abs(y) < medio]
        return min(frente, min(xs) - f) if xs else frente

    def _conducir(self, accion, maniobra=True, esc_dur=1.0):
        """Envía un pulso. Con E11 activo, antes lo pasa por el predictor de colisión; devuelve True si se envió."""
        cfg = self._get_config() or {}
        if cfg.get("e11_activo", False) and cfg.get("e11_predictor", True) and accion in ("adelante", "atras", "izquierda", "derecha"):
            clave = "giro" if accion in ("izquierda", "derecha") else accion
            if not self._despejado(clave, cfg):
                self.pred_rechazados[clave] += 1
                ahora = time.monotonic()
                self._rechazos = [t for t in self._rechazos if ahora - t < 20.0] + [ahora]
                if len(self._rechazos) >= 8 and ahora >= self._relajar_hasta:          # todo bloqueado: márgenes al 30 % un rato para despegarse
                    self._relajar_hasta = ahora + float(cfg.get("e11_pred_relajar_s", 25.0))
                    self._rechazos = []
                    self._emitir(evento="predictor_relaja", segundos=round(float(cfg.get("e11_pred_relajar_s", 25.0))))
                if clave == "giro":
                    for alt, k in (("derecha" if accion == "izquierda" else "izquierda", "giro"), ("atras", "atras")):
                        if self._despejado(k, cfg):
                            self.pred_rechazados["cambiados"] += 1
                            accion = alt
                            break
                    else:
                        return False
                else:
                    return False
        self._conducir_interno(accion, maniobra, esc_dur)
        return True

    def _conducir_interno(self, accion, maniobra=True, esc_dur=1.0):
        # Girar en el sitio barre un círculo de ~31 cm con las esquinas: una pata de silla o mesa en diagonal (fuera de los sectores
        # frontal y laterales) quedaba dentro y se golpeaba (simulación del laboratorio, 2026-10-22). Si algo está a menos de
        # `margen_giro_m` del cuerpo y hay sitio detrás, primero se retrocede un pulso; el giro llega en el ciclo siguiente.
        if accion in ("izquierda", "derecha"):
            cfg = self._get_config() or {}
            d = self._distancias() or {}
            margen = float(cfg.get("margen_giro_m", 0.18))   # al girar, la esquina del chasis barre ~12 cm más allá del costado; 0,18 = barrido sala en L 2026-10-23
            atras = self._hueco(d, "atras")
            ok_atras = not (cfg.get("e11_activo", False) and cfg.get("e11_predictor", True)) or self._despejado("atras", cfg)
            if ok_atras and d.get("cerca") is not None and d["cerca"] < margen and atras > max(0.30, float(cfg.get("retroceso_libre_cm", 30)) / 100.0):
                self.estado = f"algo a {d['cerca'] * 100:.0f} cm del costado: retrocedo antes de girar"
                self._rumbo_ref = None
                self._conducir_cb("atras", False)
                return
        if accion in ("izquierda", "derecha", "atras") and not self._corr_propia:
            self._rumbo_ref = None          # giro o marcha atrás a propósito: el rumbo deseado será el nuevo
        if esc_dur != 1.0:
            self._conducir_cb(accion, maniobra, esc_dur)
        else:
            self._conducir_cb(accion, maniobra)

    def _rumbo_correccion(self, cfg):
        """Pulso de corrección ('izquierda'/'derecha') para volver al rumbo del tramo recto, o None.

        Fusión del 2026-10-22 (rumbo_fusion): la pose del SLAM mide el error de rumbo, la zona muerta evita corregir ruido y el sesgo por
        pulso aprendido anticipa hacia qué lado cae este robot. Apagado por defecto (autonomia.rumbo_fusion_activo): se valida primero con
        tools/probar_recta.py."""
        if _RF is None or not cfg.get("rumbo_fusion_activo", False):
            return None
        pose = self._get_pose()
        if not pose:
            return None
        th = math.radians(pose["theta_deg"])
        if self._rumbo_corr is None:
            self._rumbo_corr = _RF.CorrectorRumbo(float(cfg.get("rumbo_zona_muerta_deg", 6.0)), aprende=float(cfg.get("rumbo_aprende", 0.15)))
        if self._rumbo_ref is None:
            self._rumbo_ref = th
            self._rumbo_pend = False
            return None
        e = _RF._ang(th - self._rumbo_ref)
        if self._rumbo_pend:
            self._rumbo_corr.fin_pulso_recto(e)
            self._rumbo_pend = False
        return self._rumbo_corr.decidir(e)

    def ceder_mando(self, segundos: float):
        """El operador acaba de corregir: la autonomía calla un rato.

        Sin esto le quitaría el mando a mitad de la maniobra que le están
        enseñando, y ni la lección saldría entera ni el robot obedecería."""
        self._mando_cedido_hasta = time.monotonic() + segundos
        self.estado = "corrigiendo el operador"

    def _percepcion(self) -> dict | None:
        """Lo que se le pasa al aprendizaje para reconocer la situación."""
        if self._sensores is not None:
            return self._sensores()
        d = self._distancias()
        v = getattr(self._seguidor, "vision", None)
        return {"lidar_adelante_m": d.get("adelante"), "lidar_atras_m": d.get("atras"),
                "lidar_izquierda_m": d.get("izquierda"), "lidar_derecha_m": d.get("derecha"),
                "sendero_lateral_m": getattr(v, "lateral_m", None)}

    async def _probar_aprendido(self, apurado=True) -> bool:
        """¿Me enseñaron algo para esta situación? Entonces eso, y no la regla.

        Se ejecuta la SECUENCIA entera: salir de un atasco suele necesitar
        varios movimientos encadenados, no uno.

        `apurado=False` es marcha normal, y ahí el listón sube: hace falta que
        la lección se haya dado (o confirmado) más de una vez. Si no, una
        corrección suelta en campo abierto secuestraría la conducción."""
        if self._aprendizaje is None:
            return False
        sensores = self._percepcion()
        m = self._aprendizaje.consultar(sensores, self.estado)
        if m is None or (not apurado and m.exitos < 1):
            return False
        self.aprendidas += 1
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

    def _hay_sitio_para_maniobra(self, d) -> bool:
        """La maniobra traslada el robot hacia delante y hacia atrás."""
        return (self._hueco(d, "adelante") >= MARGEN_MANIOBRA_M
                and self._hueco(d, "atras") >= MARGEN_MANIOBRA_M)

    def _giro_con_arco_seguro(self, d) -> bool:
        """En chasis de eje pasivo, evita el giro ficticio con v=0.

        El pulso exterior intenta avance+retroceso; el freno conserva cada
        tramo seguro. Si no hay margen delantero, no se autoriza un pivote.
        Esta política se habilita solo desde la config del simulador.
        """
        if self._hay_sitio_para_maniobra(d):
            return True
        cfg = self._get_config()
        if not cfg.get("arco_frontal_si_giro_pasivo_bloqueado", False):
            return False
        return (self._hueco(d, "adelante")
                >= float(cfg.get("margen_arco_frontal_eje_pasivo_m", 0.56)))

    def _lado_costmap(self) -> str | None:
        """Hacia dónde girar según el costmap: el centro del hueco libre."""
        costmap = self._get_costmap()
        if costmap is None or costmap._scan_count < 3:
            return None
        centro, ancho = costmap.hueco_libre()
        if centro is None or ancho is None:
            return None
        if abs(centro) < 0.05:
            return None
        return "izquierda" if centro > 0 else "derecha"

    def _lado_objetivo(self) -> str | None:
        """Hacia dónde girar para acercarse al waypoint objetivo."""
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
        """Elige el lado de giro combinando: objetivo > costmap > distancias."""
        lado = self._forzar_lado
        if lado:
            return lado
        lado = self._lado_objetivo()
        if lado and self._hueco(d, lado) >= self._get_config()["lado_libre_cm"] / 100.0:
            return lado
        lado = self._lado_costmap()
        if lado and self._hueco(d, lado) >= self._get_config()["lado_libre_cm"] / 100.0:
            return lado
        izq, der = self._hueco(d, "izquierda"), self._hueco(d, "derecha")
        return "izquierda" if izq >= der else "derecha"

    def _registrar_celda(self):
        """Marca dónde está y avisa si YA había estado ahí hace rato.

        Contar el tiempo dentro de la celda no sirve: avanzando despacio el
        robot pasa segundos en la misma sin estar dando vueltas. Lo que
        delata el bucle es VOLVER tras haberse ido."""
        pose = self._get_pose()
        if pose is None:
            return
        celda = (round(pose["x"] / CELDA_M), round(pose["y"] / CELDA_M))
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
        """Hacia qué lado hay menos terreno ya pisado (en ejes del robot)."""
        rumbo = math.radians(pose.get("theta_deg", 0.0))
        pesos = {}
        for lado, signo in (("izquierda", 1), ("derecha", -1)):
            ang = rumbo + signo * math.pi / 2
            cx = celda[0] + round(2 * math.cos(ang))
            cy = celda[1] + round(2 * math.sin(ang))
            pesos[lado] = self._memoria.get((cx, cy), (0, 0))[0]
        return min(pesos, key=pesos.get)

    async def _salir_callejon(self, d):
        """Sin salida: pivotar hasta encontrarla, SIN volver a decidir en cada
        tick. Antes oscilaba entre girar y retroceder y se quedaba rozando."""
        self.callejones += 1
        lado = self._mejor_lado(d)
        self._ultimo_lado = lado
        avance_libre = self._get_config()["avance_libre_cm"] / 100.0
        self.estado = f"callejón: girando a la {lado} hasta encontrar salida"
        self._emitir(evento="callejon", lado=lado, total=self.callejones)
        for _ in range(GIROS_CALLEJON):
            if not self.activo:
                return
            d_actual = self._distancias()
            frente = self._hueco(d_actual, "adelante")
            if frente >= avance_libre:
                return
            cfg = self._get_config()
            if cfg.get("arco_frontal_si_giro_pasivo_bloqueado", False):
                if self._hueco(d_actual, "atras") >= max(
                        self._get_config()["retroceso_libre_cm"] / 100.0,
                        float(cfg.get("espacio_giro_eje_pasivo_m", 0.65))):
                    self.estado = "callejón: retrocediendo con LIDAR trasero libre"
                    self._conducir("atras", maniobra=False)
                elif self._giro_con_arco_seguro(d_actual):
                    self.estado = f"callejón: arco hacia {lado}"
                    self._conducir(lado, maniobra=True)
                else:
                    self.estado = "detenido: sin espacio seguro para girar el eje pasivo"
                    self._parar()
                    await asyncio.sleep(0.4)
                    continue
            else:
                self._conducir(lado, maniobra=False)
            self.giros += 1
            await asyncio.sleep(PAUSA_GIRO_S)

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

                if self.pausado:
                    self.estado = "en pausa (mando manual)"
                    continue

                if time.monotonic() - self._ultimo_escaneo >= self._get_config()["escaneo_periodo_s"]:
                    self.estado = "escaneando"   # SCAN_SWEEP del firmware: 10 s
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

                # El operador tiene prioridad: mientras corrige, la autonomía
                # mira y aprende, no manda.
                if time.monotonic() < self._mando_cedido_hasta:
                    continue

                if self._atascado():
                    if not await self._probar_aprendido():
                        await self._desatascar()
                    continue

                self._registrar_celda()

                cfg = self._get_config()
                avance_libre = cfg["avance_libre_cm"] / 100.0
                lado_libre = cfg["lado_libre_cm"] / 100.0
                retro_libre = cfg["retroceso_libre_cm"] / 100.0

                d = self._distancias()
                frente = self._hueco(d, "adelante")
                if cfg.get("e11_activo", False):
                    frente = self._frente_barrido(frente, cfg)
                    self._e11_mapa()
                    if not self._regresando and self._debe_regresar(cfg):
                        self.iniciar_regreso()
                    if self._regresando:
                        if await self._paso_regreso(cfg, d):
                            break
                        if time.monotonic() >= self._pl_pausa_hasta:
                            continue
                atras = self._hueco(d, "atras")
                izq, der = self._hueco(d, "izquierda"), self._hueco(d, "derecha")
                if cfg.get("mx_anti_dither", False) and self._atrapado(cfg):
                    await self._escape_nicho(cfg, d)
                    continue
                # En el eje pasivo no existe giro sobre el sitio: retrocede en
                # un pulso protegido por el LIDAR hasta dejar hueco para un arco.
                margen_giro = float(cfg.get("espacio_giro_eje_pasivo_m", 0.65))
                if (cfg.get("retroceso_previo_giro_eje_pasivo", False)
                        and frente < margen_giro and atras >= max(retro_libre, margen_giro)):
                    self.estado = f"retrocediendo para liberar giro ({frente:.2f} m delante)"
                    self._conducir("atras", maniobra=False)
                    await asyncio.sleep(1.05)  # un pulso exterior; luego vuelve a medir
                    continue

                # En evaluación aislada, dejar que la política aprendida decida
                # primero en espacio libre. El LIDAR/freno sigue teniendo prioridad
                # en obstáculo; el seguidor clásico queda como fallback si el modelo
                # duda. Esta opción solo se activa desde entrenar.py --solo-politica.
                if (cfg.get("politica_prioritaria_segura", False)
                        and frente >= avance_libre
                        and self._politica is not None and self._politica.disponible):
                    accion, seguridad = self._politica.decidir(self._percepcion())
                    if accion in ("adelante", "izquierda", "derecha"):
                        self.estado = f"política: {accion} ({seguridad:.2f})"
                        self._conducir(accion, maniobra=False)
                        self.pulsos_politica += 1
                        self.pulsos += 1
                        self._sin_corregir += 1
                        continue

                # Seguimiento preventivo en el margen seguro: recentra antes de
                # alcanzar el umbral de avance libre, sin anular el freno cercano.
                umbral_visual = float(cfg.get("espacio_giro_eje_pasivo_m", 0.65))
                if (cfg.get("correccion_sendero_preventiva", False)
                        and frente >= umbral_visual
                        and self._sin_corregir >= PULSOS_ENTRE_CORRECCIONES):
                    correccion = self._correccion_sendero()
                    if correccion is not None:
                        self.estado = f"sendero: {self._seguidor.estado}"
                        self._conducir(correccion, maniobra=bool(cfg.get(
                            "correccion_eje_pasivo_arcos", MANIOBRA_EN_CORRECCION)))
                        self.correcciones += 1
                        self._sin_corregir = 0
                        continue

                # Rumbo a la meta con el frente libre y espacio a los lados (campo abierto): un giro hacia ella cada 3 pulsos rectos como mucho.
                if (cfg.get("mx_meta", False) and frente > 1.0 and izq > 0.6 and der > 0.6 and self._rectos >= 3 and not self._corr_propia):
                    eb = self._rumbo_meta_rad()
                    if eb is not None and abs(eb) > math.radians(float(cfg.get("mx_rumbo_tol_deg", 25.0))):
                        self.estado = f"rumbo a la meta ({math.degrees(eb):+.0f}°)"
                        self._conducir("izquierda" if eb > 0 else "derecha", maniobra=False)
                        self._rectos = 0
                        self.correcciones += 1
                        continue
                # Guardián del sendero: mide el error contra la referencia, correlaciona LIDAR + IMU + cámara con lo que funcionó antes y
                # corrige con un pulso; APUNTA qué hizo y qué resultó (JSONL + memoria). Apagado por defecto.
                if (frente >= avance_libre and self.guardian is not None and cfg.get("guardian_sendero_activo", False)
                        and not self._corr_propia):
                    if await self._guardian_paso(cfg):
                        continue

                if frente >= avance_libre:
                    # Lo enseñado por el operador manda sobre la regla, también
                    # en marcha normal (con el listón más alto: ver _probar_aprendido).
                    if await self._probar_aprendido(apurado=False):
                        continue
                    # Con sitio delante, antes de avanzar se mira si hay que
                    # recentrarse en el sendero. La corrección es un pulso de
                    # giro corto: este chasis no sabe girar "un poco".
                    correccion = (self._correccion_sendero()
                                  if self._sin_corregir >= PULSOS_ENTRE_CORRECCIONES else None)
                    if correccion is not None:
                        self.estado = f"sendero: {self._seguidor.estado}"
                        # Recentrarse es corregir el RUMBO: pivote, que no
                        # traslada. Con la maniobra el robot se salía del
                        # sendero (y en el campo pisa las plantas).
                        maniobra_correccion = bool(cfg.get(
                            "correccion_eje_pasivo_arcos", MANIOBRA_EN_CORRECCION))
                        self._conducir(correccion, maniobra=maniobra_correccion)
                        self.correcciones += 1
                        self._sin_corregir = 0
                        continue

                    # Parada por incertidumbre (2026-10-14). Con el sendero
                    # como referencia de rumbo, una racha de ciclos sin
                    # NINGUNA medida —ni hileras, ni costmap, ni cámara con
                    # confianza— no es "centrado": es no saber. Antes se caía
                    # aquí y se avanzaba recto, que en la huerta es pisar la
                    # hilera. Se para y se avisa; no se escanea ni se gira a
                    # ciegas, porque no hay nada que diga hacia dónde.
                    # El sendero se ve, se mide, y NO cabe el robot. Seguir
                    # el centro de la mancha metería el eje trasero (37 cm)
                    # en un hueco de 30. En modo "aviso" se cuenta y se sigue;
                    # en "frenar" se para (abrirse paso no es una decisión que
                    # pueda tomar la autonomía).
                    estrechez = self._seguidor.estrecho(cfg)
                    if estrechez:
                        snap = self._seguidor.snapshot()
                        self._emitir(evento="sendero_estrecho", modo=estrechez,
                                     ancho_cm=snap.get("sendero_ancho_cm"),
                                     margen_cm=snap.get("sendero_margen_cm"))
                        if estrechez == "frenar":
                            self.estado = ("parado: sendero de "
                                           f"{snap.get('sendero_ancho_cm')} cm, "
                                           "no cabe el eje trasero")
                            self._parar()
                            self.paradas_estrechas += 1
                            await asyncio.sleep(ESPERA_INCERTIDUMBRE_S)
                            continue
                        # Modo aviso: se registra cuántas veces se habría
                        # frenado y se sigue adelante.
                        self.avisos_estrechos += 1

                    if self._seguidor.incierto(cfg):
                        self.estado = f"parado: {self._seguidor.estado}"
                        self._parar()
                        self.paradas_inciertas += 1
                        self._emitir(evento="sendero_incierto",
                                     detalle=self._seguidor.estado,
                                     ciclos=self._seguidor.ciclos_sin_referencia)
                        await asyncio.sleep(ESPERA_INCERTIDUMBRE_S)
                        continue

                    # El rumbo fusionado va ANTES que la política: si no, una política que siempre dice «adelante» lo anulaba (2026-10-22).
                    a_rumbo_previo = self._rumbo_correccion(cfg)
                    if a_rumbo_previo:
                        self.estado = f"corrigiendo rumbo ({a_rumbo_previo})"
                        self._corr_propia = True
                        try:
                            self._conducir(a_rumbo_previo, maniobra=False, esc_dur=float(cfg.get("rumbo_pulso_escala", 1.0)))
                        finally:
                            self._corr_propia = False
                        self.correcciones_rumbo += 1
                        self._sin_corregir = 0
                        continue
                    accion_politica = None
                    if self._politica is not None and self._politica.disponible and cfg.get("politica_activa", False):
                        accion_politica, seguridad = self._politica.decidir(self._percepcion())
                    if accion_politica in ("adelante", "izquierda", "derecha"):
                        self.estado = f"política: {accion_politica} ({seguridad:.2f})"
                        self._conducir(accion_politica, maniobra=False)
                        self.pulsos_politica += 1
                        self.pulsos += 1
                        self._sin_corregir += 1
                        continue
                    a_rumbo = self._rumbo_correccion(cfg)
                    if a_rumbo:
                        self.estado = f"corrigiendo rumbo ({a_rumbo})"
                        self._corr_propia = True
                        try:
                            self._conducir(a_rumbo, maniobra=False, esc_dur=float(cfg.get("rumbo_pulso_escala", 1.0)))
                        finally:
                            self._corr_propia = False
                        self.correcciones_rumbo += 1
                        self._sin_corregir = 0
                        continue
                    self.estado = f"avanzando ({frente:.2f} m libres)"
                    self._conducir("adelante")
                    self._rectos += 1
                    if self._rumbo_corr is not None and self._rumbo_ref is not None:
                        pose_r = self._get_pose()
                        if pose_r:
                            self._rumbo_corr.inicio_pulso_recto(_RF._ang(math.radians(pose_r["theta_deg"]) - self._rumbo_ref))
                            self._rumbo_pend = True
                    self.pulsos += 1
                    self._sin_corregir += 1
                    if self._compromiso:
                        self._compromiso -= 1
                        if not self._compromiso:
                            self._forzar_lado = None
                    continue

                # Sin sitio delante ni a los lados: es un callejón, y se sale
                # con una maniobra comprometida en vez de re-decidir cada tick.
                if frente < FRENTE_CALLEJON_M and izq < lado_libre and der < lado_libre:
                    if not await self._probar_aprendido():
                        await self._salir_callejon(d)
                    continue

                lado = self._mejor_lado(d)
                if self._hueco(d, lado) < lado_libre:
                    lado = "izquierda" if izq >= der else "derecha"
                h_cm = float(cfg.get("man_histeresis_cm", 0.0)) if cfg.get("rumbo_fusion_activo", False) else 0.0
                if h_cm > 0 and math.isfinite(izq) and math.isfinite(der) and abs(izq - der) < h_cm / 100.0 \
                        and self._hueco(d, self._ultimo_lado) >= lado_libre:
                    lado = self._ultimo_lado          # lados casi iguales: repite el último giro en vez de titubear
                if self._hueco(d, lado) >= lado_libre:
                    self._bloq += 1
                    gap = None
                    if cfg.get("mx_hueco", False):
                        con_meta = self._rumbo_meta_rad() is not None
                        gap = self._mejor_hueco(float(cfg.get("mx_hueco_m", 1.0)), 1.0 if con_meta else 0.2) or \
                            self._mejor_hueco(0.6, 1.0 if con_meta else 0.2)
                        if gap is not None and abs(math.degrees(gap)) >= 4.0 and self._hueco(d, "izquierda" if gap > 0 else "derecha") >= lado_libre:
                            lado = "izquierda" if gap > 0 else "derecha"
                    elif cfg.get("mx_meta", False) and self._commit and time.monotonic() < self._commit_hasta and self._hueco(d, self._commit) >= 0.30:
                        lado = self._commit
                    maniobra = self._giro_con_arco_seguro(d)
                    if (not maniobra and cfg.get(
                            "arco_frontal_si_giro_pasivo_bloqueado", False)):
                        self.estado = "detenido: sin espacio seguro para girar el eje pasivo"
                        self._parar()
                        await asyncio.sleep(0.4)
                        continue
                    self.estado = (f"girando a la {lado} (frente {frente:.2f} m"
                                   + (" · arco" if maniobra else ", pivote") + ")")
                    self._ultimo_lado = lado
                    if self.guardian is not None:
                        self.guardian.ref = []             # tras girar, el sendero es otro: la referencia nueva se fija en el siguiente tramo recto
                    # Maniobra de esquina (2026-10-24, simulada en sim_logica_actual: man_*): si el frente está muy cerca, primero se retrocede
                    # `man_atras_pulsos` pulsos y luego se gira `man_giro_pulsos` seguidos (se corta si el frente ya quedó libre).
                    # Con los valores por defecto (0 / 1 / 1) es el comportamiento de siempre.
                    n_atras = max(1, int(cfg.get("man_atras_pulsos", 1)))
                    n_giro = max(1, int(cfg.get("man_giro_pulsos", 1)))
                    if gap is not None:
                        n_giro = int(max(1, min(9, round(abs(math.degrees(gap)) / float(cfg.get("mx_grados_pulso", 6.0))))))
                    ret_cm = float(cfg.get("man_retroceso_cm", 0.0))
                    if ret_cm > 0 and frente < ret_cm / 100.0 and atras >= max(retro_libre, 0.30):
                        self.estado = f"esquina: retrocedo {n_atras} pulso(s) (frente {frente:.2f} m)"
                        for _ in range(n_atras):
                            self._conducir("atras", maniobra=False)
                            await asyncio.sleep(1.05)
                            if self._hueco(self._distancias(), "atras") < 0.30:
                                break
                    self._conducir(lado, maniobra=maniobra)
                    self.giros += 1
                    for _ in range(n_giro - 1):
                        await asyncio.sleep(1.05)
                        if not self.activo or self.pausado or time.monotonic() < self._mando_cedido_hasta:
                            break
                        if self._hueco(self._distancias(), "adelante") >= avance_libre + 0.30:
                            break                   # el frente ya está despejado: no hace falta seguir girando
                        self._conducir(lado, maniobra=maniobra)
                        self.giros += 1
                    if cfg.get("rumbo_fusion_activo", False) and float(cfg.get("man_pausa_s", 0.0)) > 0:
                        await asyncio.sleep(float(cfg["man_pausa_s"]))     # deja que el LIDAR se asiente antes de decidir
                    continue

                if self._hueco(d, "atras") >= retro_libre:
                    self.estado = "sin salida: retrocediendo"
                    self._conducir("atras")
                    continue

                if await self._probar_aprendido():
                    continue
                if cfg.get("arco_frontal_si_giro_pasivo_bloqueado", False):
                    self.estado = "detenido: sin ruta libre; requiere intervención"
                    self._parar()
                    await asyncio.sleep(0.4)
                    continue
                self.estado = "encerrado: girando en el sitio"
                self._conducir(self._ultimo_lado, maniobra=False)
                self.giros += 1
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.activo = False
            self.estado = f"error: {exc}"
            self._parar()
            self._emitir(evento="error", detalle=str(exc))
            # Con la traza: "TypeError" a secas no dice en qué línea fue, y
            # este bucle es el que conduce el robot.
            print(f"[exploración] bucle abortado: {type(exc).__name__}: {exc}\n"
                  + traceback.format_exc(), flush=True)

    def _rumbo_meta_rad(self):
        """Error de rumbo hacia la meta en rad (+ = la meta queda a la izquierda) o None si no hay meta."""
        if self._rumbo_objetivo is None:
            return None
        info = self._rumbo_objetivo()
        if info is None or info[1] < 0.3:
            return None
        return math.radians(info[0])

    def _mejor_hueco(self, necesita: float, peso_meta: float):
        """Ángulo (rad, + = izquierda) hacia el centro del hueco libre más conveniente en el scan, o None.
        Un rumbo es libre si el rayo llega a `necesita` m; se erosiona ±12° para que el robot quepa; se prefiere el más cercano a la meta."""
        scan = self.scan_cb() if self.scan_cb else None
        if not scan:
            return None
        r = [math.inf] * 181                                        # -90..90° de 1° en 1°
        for q in scan:
            a = int(round(q["angle_deg"]))
            a = a - 360 if a > 180 else a
            if -90 <= a <= 90:
                r[a + 90] = min(r[a + 90], q["distance_m"])
        libre = [1 if v >= necesita else 0 for v in r]
        k = 12
        er = [1 if (i - k >= 0 and i + k <= 180 and all(libre[i - k:i + k + 1])) else 0 for i in range(181)]
        meta = self._rumbo_meta_rad() or 0.0
        mejor, mejor_p, i = None, -1e9, 0
        while i <= 180:
            if er[i]:
                j = i
                while j + 1 <= 180 and er[j + 1]:
                    j += 1
                c = math.radians((i + j) / 2 - 90)
                pt = -peso_meta * abs(c - meta) + 0.01 * (j - i)
                if pt > mejor_p:
                    mejor, mejor_p = c, pt
                i = j + 1
            else:
                i += 1
        return mejor

    def _atrapado(self, cfg) -> bool:
        """Dando vueltas en un nicho: en `mx_ventana_s` s no se alejó 0,45 m y se bloqueó >= 8 veces."""
        pose = self._get_pose()
        if not pose:
            return False
        t = time.monotonic()
        self._hist.append((t, pose["x"], pose["y"], self._bloq))
        ventana = float(cfg.get("mx_ventana_s", 25.0))
        while self._hist and t - self._hist[0][0] > ventana:
            self._hist.pop(0)
        if len(self._hist) < 2 or t - self._hist[0][0] < 0.9 * ventana:
            return False
        return (max(math.hypot(pose["x"] - h[1], pose["y"] - h[2]) for h in self._hist) < 0.45
                and self._bloq - self._hist[0][3] >= 8)

    async def _escape_nicho(self, cfg, d):
        """Atrás x3 + giro largo al lado CONTRARIO del último + adelante x3, y compromiso de lado un rato."""
        izq, der = self._hueco(d, "izquierda"), self._hueco(d, "derecha")
        lado = ("derecha" if self._ultimo_lado == "izquierda" else "izquierda")
        self.mx_escapes += 1
        self._hist.clear()
        self._commit, self._commit_hasta = lado, time.monotonic() + 2 * float(cfg.get("mx_bordear_s", 12.0))
        self._ultimo_lado = lado
        self.estado = f"atrapado en un nicho: escapo girando a la {lado}"
        self._emitir(evento="escape_nicho", lado=lado, total=self.mx_escapes)
        if self._hueco(d, "atras") >= 0.30:
            for _ in range(3):
                self._conducir("atras", maniobra=False)
                await asyncio.sleep(1.05)
        for _ in range(max(1, int(cfg.get("mx_giro_largo", 5)))):
            if not self.activo or self.pausado:
                return
            self._conducir(lado, maniobra=False)
            await asyncio.sleep(1.05)
        for _ in range(3):
            if not self.activo or self.pausado:
                return
            if self._hueco(self._distancias(), "adelante") < cfg["avance_libre_cm"] / 100.0:
                return
            self._conducir("adelante")
            await asyncio.sleep(1.05)

    # ------------------------------------------------------------------ E11: mapa, regreso al inicio planificado
    def _e11_mapa(self):
        """Acumula el scan en el mapa del planificador (pose del SLAM; el LIDAR se toma en el centro de base_link)."""
        pose = self._get_pose()
        scan = self.scan_cb() if self.scan_cb else None
        if self.plan is not None and pose and scan:
            self.plan.actualizar(pose["x"], pose["y"], math.radians(pose["theta_deg"]), scan)

    def _debe_regresar(self, cfg) -> bool:
        """Disparadores automáticos de la ida (apagados por defecto: 0): distancia al inicio o tiempo de ida."""
        pose = self._get_pose()
        if not pose or self._inicio_xy is None:
            return False
        d_max, t_max = float(cfg.get("e11_ida_max_m", 0.0)), float(cfg.get("e11_ida_max_s", 0.0))
        if d_max > 0 and math.hypot(pose["x"] - self._inicio_xy[0], pose["y"] - self._inicio_xy[1]) >= d_max:
            return True
        return t_max > 0 and time.monotonic() - self._t_inicio >= t_max

    def iniciar_regreso(self) -> tuple[bool, str]:
        """Da la vuelta: a partir de ahora el robot vuelve al punto de salida siguiendo una ruta A* sobre el mapa que acumuló."""
        if not self.activo:
            return False, "la exploración no está corriendo"
        if self._inicio_xy is None:
            pose0 = self._get_pose()
            if not pose0:
                return False, "no hay pose del SLAM: no sé dónde está el inicio"
            self._inicio_xy = (pose0["x"], pose0["y"])
        if self.plan is None:
            return False, "E11 no está activo (autonomia.e11_activo) o falta planificador.py"
        self._regresando, self._ruta, self._pl_fallos, self._pl_pausa_hasta = True, None, 0, 0.0
        self.regreso_estado = "en curso"
        self.estado = "regresando al punto de inicio"
        self._emitir(evento="regreso_inicia", inicio=[round(self._inicio_xy[0], 2), round(self._inicio_xy[1], 2)])
        return True, "regresando al punto de inicio"

    async def _paso_regreso(self, cfg, d) -> bool:
        """Un ciclo del regreso. True = terminó (llegó al inicio): el bucle sale."""
        pose = self._get_pose()
        if not pose or self._inicio_xy is None:
            self._parar()
            return False
        x, y = pose["x"], pose["y"]
        if math.hypot(x - self._inicio_xy[0], y - self._inicio_xy[1]) < float(cfg.get("e11_regreso_radio_m", 0.5)):
            self._regresando = False
            self.regreso_estado = "completo"
            self.activo = False
            self._grabar_recorrido(False)
            self._parar()
            self.estado = "regreso completo: en el punto de inicio"
            self._emitir(evento="regreso_completo")
            return True
        ahora = time.monotonic()
        if ahora < self._pl_pausa_hasta:
            return False                                   # el planificador cedió: ciclo normal del explorador (reactivo)
        if self._ruta is None or ahora - self._ruta_t >= float(cfg.get("e11_plan_cada_s", 2.6)):
            self._ruta = self.plan.ruta((x, y), self._inicio_xy, tol_m=0.3)
            self._ruta_t = ahora
            if self._ruta is None:
                self.regreso_estado = "sin ruta"
                self._pl_pausa_hasta = ahora + 6.0
                self.estado = "regreso: sin ruta conocida, modo reactivo 6 s"
                return False
            self.regreso_estado = "en curso"
        wp = _Planificador.siguiente(self._ruta, (x, y), float(cfg.get("e11_plan_mirada_m", 0.75)))
        eb = math.atan2(wp[1] - y, wp[0] - x) - math.radians(pose["theta_deg"])
        eb = math.atan2(math.sin(eb), math.cos(eb))
        accion = ("izquierda" if eb > 0 else "derecha") if abs(math.degrees(eb)) > float(cfg.get("e11_plan_giro_deg", 19.6)) else "adelante"
        self.estado = f"regresando: {accion} (a {math.hypot(x - self._inicio_xy[0], y - self._inicio_xy[1]):.1f} m del inicio)"
        if self._conducir(accion, maniobra=False):
            self._pl_fallos = 0
            self.pulsos += 1
        else:
            self._pl_fallos += 1
            self._ruta = None
            if self._pl_fallos >= 4:
                self._pl_pausa_hasta, self._pl_fallos = ahora + 10.0, 0
        return False

    async def _guardian_paso(self, cfg) -> bool:
        """Un ciclo del guardián. True si ejecutó una corrección (y esperó a que asentara)."""
        g = self.guardian
        pose = self._get_pose()
        if not pose:
            return False
        g.tol_y, g.tol_h = float(cfg.get("guardian_tol_cm", 6.0)) / 100.0, math.radians(float(cfg.get("guardian_tol_deg", 4.0)))
        g.usar_memoria = bool(cfg.get("guardian_usar_memoria", True))
        if len(g.ref) < 2:
            g.fijar_recta(pose)                 # la referencia es el rumbo y la posición del primer tramo recto tras un giro o el arranque
            return False
        sens = self.sensores_guardian() if self.sensores_guardian else self._percepcion()
        dec = g.decidir(pose, sens)
        if dec is None:
            return False
        self.estado = f"guardián: {dec['explicacion']}"
        g.iniciar(dec, pose, sens)
        self._conducir(dec["accion"], maniobra=bool(cfg.get("correccion_eje_pasivo_arcos", MANIOBRA_EN_CORRECCION)), esc_dur=dec["escala"])
        await asyncio.sleep(1.05 * max(dec["escala"], 0.5) + 0.5)          # el pulso + el asentamiento antes de medir qué cambió
        g.terminar(self._get_pose() or pose, self.sensores_guardian() if self.sensores_guardian else self._percepcion())
        self.correcciones += 1
        self._sin_corregir = 0
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

    async def _desatascar(self):
        """El robot cree que se mueve pero la pose no cambia: rueda patinando,
        enganchado en una planta o contra algo que el LIDAR no ve (hueco bajo,
        rama fina). Se retrocede y se gira al lado contrario del último.

        Si el costmap tiene un hueco libre claro, se gira hacia él en vez de
        alternar a ciegas."""
        self.desatascos += 1
        self.estado = "desatascando"
        self._emitir(evento="desatasco", total=self.desatascos)
        d = self._distancias()
        if self._hueco(d, "atras") >= self._get_config()["retroceso_libre_cm"] / 100.0:
            self._conducir("atras")
            await asyncio.sleep(1.5)
        lado_cm = self._lado_costmap()
        if lado_cm is not None:
            contrario = lado_cm
        else:
            contrario = "derecha" if self._ultimo_lado == "izquierda" else "izquierda"
        self._ultimo_lado = contrario
        for _ in range(DESATASCO_GIROS):
            if not self.activo:
                return
            self._conducir(contrario, maniobra=self._hay_sitio_para_maniobra(self._distancias()))
            self.giros += 1
            await asyncio.sleep(1.3)
        self._ref_pose, self._ref_t = None, time.monotonic()
