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
        d = {"activo": self.activo, "estado": self.estado, "pulsos": self.pulsos,
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

    def _conducir(self, accion, maniobra=True, esc_dur=1.0):
        # Girar en el sitio barre un círculo de ~31 cm con las esquinas: una pata de silla o mesa en diagonal (fuera de los sectores
        # frontal y laterales) quedaba dentro y se golpeaba (simulación del laboratorio, 2026-10-22). Si algo está a menos de
        # `margen_giro_m` del cuerpo y hay sitio detrás, primero se retrocede un pulso; el giro llega en el ciclo siguiente.
        if accion in ("izquierda", "derecha"):
            cfg = self._get_config() or {}
            d = self._distancias() or {}
            margen = float(cfg.get("margen_giro_m", 0.18))   # al girar, la esquina del chasis barre ~12 cm más allá del costado; 0,18 = barrido sala en L 2026-10-23
            atras = self._hueco(d, "atras")
            if d.get("cerca") is not None and d["cerca"] < margen and atras > max(0.30, float(cfg.get("retroceso_libre_cm", 30)) / 100.0):
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
                atras = self._hueco(d, "atras")
                izq, der = self._hueco(d, "izquierda"), self._hueco(d, "derecha")
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
