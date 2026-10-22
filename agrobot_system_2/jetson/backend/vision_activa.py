"""VISIÓN ACTIVA (2026-10-22): mueve la cámara y el robot para mirar el entorno, centrar un objetivo, acercarse y analizarlo.

Qué hace (ciclo por objetivo):
  BUSCANDO    barre el servo de tilt (arriba/abajo) y, si no ve nada, gira un pulso y vuelve a barrer.
  CENTRANDO   cuando ve un objetivo (planta/col/maceta...) lo centra: el SERVO de tilt corrige la altura y pulsos de giro corrigen el
              rumbo (la cámara solo tiene tilt: el «paneo» lo hace el robot girando).
  ACERCANDO   avanza en pulsos hasta `distancia_inspeccion_m` (medida con el LIDAR, hueco hasta la carrocería).
  ENCUADRANDO ajusta el tilt para que el objetivo quede en el centro de la imagen.
  ANALIZANDO  durante `analisis_s` reúne las detecciones del modelo activo, guarda una foto y un registro.
  ALEJANDO    gira unos pulsos para mirar a otro lado y vuelve a BUSCANDO (hasta `max_objetivos`).

SEGURIDAD (todo esto se comprueba en cada paso):
  · APAGADA por defecto: solo se enciende con POST /api/vision_activa {"activa": true}.
  · Sin scan fresco del LIDAR, sin C3 conectada o con la autonomía (exploración/patrulla) activa, se detiene y espera.
  · Nunca avanza con menos de `no_avanzar_bajo_m` de hueco delante, y la anticolisión con retroceso sigue actuando por debajo.
  · Una parada (emergencia o STOP de manejo) o cualquier orden manual la cancela.
  · Límites: pulsos por objetivo, giros de búsqueda, tiempo total. Solo usa pulsos cortos del conductor (nada de velocidad continua).

Es lógica con dependencias inyectadas (cámara, servo, conductor, LIDAR): `paso()` es síncrono y determinista (reloj inyectable),
así que se prueba sin hardware.
"""
from __future__ import annotations

import asyncio
import json
import math
import time
from collections import Counter
from pathlib import Path

from calibracion_camara import GRADOS_POR_GRADO

DEFECTOS = {
    "activa": False,
    "clases_objetivo": ["planta", "col", "maceta"],     # subcadenas del nombre de clase (minúsculas)
    "conf_min": 0.35,
    "camara_yaw_offset_deg": 0.0,                       # si la cámara no mira recto respecto al eje del robot (+ = mira a la derecha)
    "tol_rumbo_deg": 6.0,
    "tol_tilt_deg": 4.0,
    "ganancia_tilt": 0.7,
    "paso_servo_max_deg": 4.0,
    "servo_min_deg": 60,
    "servo_max_deg": 125,
    "distancia_inspeccion_m": 0.45,                     # hueco LIDAR (hasta la carrocería) al que se inspecciona
    "tol_distancia_m": 0.08,
    "no_avanzar_bajo_m": 0.20,
    "asentamiento_s": 1.0,                              # espera tras cada pulso/servo para que la imagen no salga movida
    "analisis_s": 2.0,
    "dwell_barrido_s": 1.4,
    "angulos_barrido_deg": [98, 82, 112, 98],
    "max_giros_busqueda": 26,
    "giros_tras_analisis": 4,
    "max_pulsos_objetivo": 16,
    "max_objetivos": 3,
    "perder_objetivo_s": 4.0,
    "tiempo_max_s": 600.0,
    "scan_max_s": 1.0,
    "max_area_frac": 0.85,                              # cajas que ocupan casi toda la imagen no sirven para centrar (modelo sin localización)
    "sin_mover": False,                                 # True = ensayo: mueve el servo y DECIDE, pero los pulsos solo se registran (el robot no se mueve)
}

INACTIVA, BUSCANDO, CENTRANDO, ACERCANDO, ENCUADRANDO, ANALIZANDO, ALEJANDO, PAUSA = (
    "inactiva", "buscando", "centrando", "acercando", "encuadrando", "analizando", "alejando", "pausa")

CARPETA = Path.home() / ".agrobot" / "vision_activa"


def angulos_de_caja(caja_xyxy, ancho, alto, hfov_deg, yaw_offset_deg=0.0):
    """(error de rumbo en °, + = objetivo a la DERECHA; error vertical en °, + = objetivo ABAJO) del centro de una caja."""
    x1, y1, x2, y2 = caja_xyxy
    f = (ancho / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    yaw = math.degrees(math.atan(((x1 + x2) / 2.0 - ancho / 2.0) / f)) + yaw_offset_deg
    pitch = math.degrees(math.atan(((y1 + y2) / 2.0 - alto / 2.0) / f))
    return yaw, pitch


def hfov_con_zoom(hfov_deg, zoom):
    z = max(1.0, float(zoom or 1.0))
    return math.degrees(2.0 * math.atan(math.tan(math.radians(hfov_deg) / 2.0) / z))


class VisionActiva:
    def __init__(self, *, get_cajas, get_servo, set_servo, conducir, ocupado, parar, distancias, edad_scan, get_config,
                 on_event=None, capturar=None, clasificar=None, get_pose=None, enlace_ok=None, autonomia_activa=None, reloj=time.monotonic,
                 carpeta: Path | None = None):
        self._get_cajas = get_cajas        # () -> (cajas[{class, confidence, xyxy}], instante_unix, ancho_px, alto_px, hfov_efectivo_deg) | None
        self._get_servo = get_servo        # () -> ángulo actual del servo
        self._set_servo = set_servo        # (ángulo)
        self._conducir = conducir          # (accion: adelante|atras|izquierda|derecha)
        self._ocupado = ocupado            # () -> bool: un pulso en curso
        self._parar = parar                # ()
        self._distancias = distancias      # () -> {adelante, atras, izquierda, derecha}
        self._edad_scan = edad_scan        # () -> s
        self._get_config = get_config      # () -> dict de la sección vision_activa
        self._on_event = on_event or (lambda e: None)
        self._capturar = capturar          # (info) -> ruta | None
        self._clasificar = clasificar      # (ruta_foto) -> dict | None: segunda opinión con un clasificador (p. ej. PLAGASX)
        self.aviso: str | None = None
        self._get_pose = get_pose or (lambda: None)
        self._enlace_ok = enlace_ok or (lambda: True)
        self._autonomia = autonomia_activa or (lambda: False)
        self.reloj = reloj
        self.carpeta = carpeta or CARPETA
        self._tarea: asyncio.Task | None = None
        self.estado = INACTIVA
        self._reset()

    # ------------------------------------------------------------------ utilidades
    def cfg(self) -> dict:
        c = dict(DEFECTOS)
        c.update(self._get_config() or {})
        return c

    def _reset(self):
        self._previo = BUSCANDO
        self._t_inicio = 0.0
        self._t_estado = 0.0
        self._t_espera = 0.0
        self._t_valida = 0.0              # solo cuentan cajas calculadas DESPUÉS de este instante (imagen asentada)
        self._t_visto = 0.0
        self._t_analisis = 0.0
        self._i_barrido = 0
        self._t_barrido = 0.0
        self.giros_busqueda = 0
        self.pulsos = 0
        self.objetivos_hechos = 0
        self.giros_alejando = 0
        self._dir_busqueda = "izquierda"
        self._muestras: list[tuple[str, float]] = []
        self.ultimo_objetivo: dict | None = None
        self.ultimo_resultado: dict | None = None
        self.motivo_fin: str | None = None

    def _cambiar(self, nuevo: str):
        if nuevo != self.estado:
            self.estado = nuevo
            self._t_estado = self.reloj()
            self._on_event({"tipo": "estado", "estado": nuevo})

    def _esperar(self, s: float):
        self._t_espera = self.reloj() + s
        self._t_valida = self._t_espera       # las cajas anteriores a la espera ya no valen (imagen movida)

    def estado_dict(self) -> dict:
        return {"estado": self.estado, "activa": self.estado != INACTIVA, "pulsos": self.pulsos,
                "giros_busqueda": self.giros_busqueda, "objetivos_hechos": self.objetivos_hechos,
                "objetivo": self.ultimo_objetivo, "resultado": self.ultimo_resultado, "motivo_fin": self.motivo_fin,
                "aviso": self.aviso,
                "servo": self._get_servo()}

    # ------------------------------------------------------------------ ciclo de vida
    def iniciar(self) -> tuple[bool, str]:
        if self.estado != INACTIVA:
            return False, "ya está activa"
        if self._autonomia():
            return False, "la exploración/patrulla está activa: páralas primero"
        self._reset()
        self._t_inicio = self.reloj()
        self.estado = BUSCANDO
        self._t_estado = self._t_inicio
        self._set_servo(DEFECTOS["angulos_barrido_deg"][0])
        self._esperar(self.cfg()["asentamiento_s"])
        self._on_event({"tipo": "inicio"})
        return True, "ok"

    def cancelar(self, motivo: str = "cancelada"):
        """Parada inmediata (también desde la parada total del servidor). Idempotente."""
        if self.estado == INACTIVA:
            return
        try:
            self._parar()
        finally:
            self.motivo_fin = motivo
            self.estado = INACTIVA
            self._on_event({"tipo": "fin", "motivo": motivo})

    def _terminar(self, motivo: str):
        self.cancelar(motivo)

    async def _bucle(self):
        while self.estado != INACTIVA:
            try:
                self.paso()
            except Exception as exc:                      # un fallo aquí no puede dejar al robot moviéndose
                self.cancelar(f"error: {type(exc).__name__}: {exc}")
                break
            await asyncio.sleep(0.2)

    def arrancar_tarea(self):
        self._tarea = asyncio.create_task(self._bucle())

    # ------------------------------------------------------------------ decisión
    def _seguro(self, c) -> tuple[bool, str]:
        if self._autonomia():
            return False, "autonomía activa"
        if not self._enlace_ok():
            return False, "C3 sin enlace"
        if self._edad_scan() > c["scan_max_s"]:
            return False, "LIDAR sin scans frescos"
        return True, ""

    def _objetivo(self, c) -> dict | None:
        r = self._get_cajas()
        if not r:
            return None
        cajas, ts, ancho, alto, hfov = r
        if ts is None or ts < self._t_valida + 0.0:
            return None
        mejor = None
        for b in cajas:
            nombre = str(b.get("class", "")).lower()
            if b.get("confidence", 0.0) < c["conf_min"] or not any(k in nombre for k in c["clases_objetivo"]):
                continue
            x1, y1, x2, y2 = b["xyxy"]
            area = max(0, x2 - x1) * max(0, y2 - y1)
            if area / float(ancho * alto) > c["max_area_frac"]:
                self.aviso = ("el modelo activo no localiza (sus cajas ocupan toda la imagen): para centrar y acercarse usa "
                              "'Interior (macetas)' o 'Col (planta y cogollo)'")
                continue
            if mejor is None or area > mejor[0]:
                mejor = (area, b)
        if mejor is None:
            return None
        b = mejor[1]
        yaw, pitch = angulos_de_caja(b["xyxy"], ancho, alto, hfov, c["camara_yaw_offset_deg"])
        return {"clase": b["class"], "conf": round(float(b["confidence"]), 3), "yaw_deg": round(yaw, 1),
                "pitch_deg": round(pitch, 1), "area_frac": round(mejor[0] / float(ancho * alto), 3)}

    def _ajustar_tilt(self, obj, c) -> bool:
        """Un paso del servo hacia centrar verticalmente. True si movió el servo."""
        if abs(obj["pitch_deg"]) <= c["tol_tilt_deg"]:
            return False
        delta = -c["ganancia_tilt"] * obj["pitch_deg"] / GRADOS_POR_GRADO        # objetivo abajo -> ángulo de servo MENOR
        delta = max(-c["paso_servo_max_deg"], min(c["paso_servo_max_deg"], delta))
        actual = float(self._get_servo())
        nuevo = max(c["servo_min_deg"], min(c["servo_max_deg"], actual + delta))
        if abs(nuevo - actual) < 0.5:
            return False
        self._set_servo(nuevo)
        self._esperar(0.5)
        return True

    def _mover(self, accion: str, c):
        if c.get("sin_mover"):
            self._on_event({"tipo": "pulso_simulado", "accion": accion})
        else:
            self._conducir(accion)

    def _pulso(self, accion: str, c):
        self._mover(accion, c)
        self.pulsos += 1
        self._esperar(c["asentamiento_s"])

    def paso(self):
        if self.estado == INACTIVA:
            return
        c = self.cfg()
        ahora = self.reloj()
        if ahora - self._t_inicio > c["tiempo_max_s"]:
            self._terminar("tiempo máximo")
            return
        ok, motivo = self._seguro(c)
        if not ok:
            if self.estado != PAUSA:
                self._previo = self.estado
                self._parar()
                self._cambiar(PAUSA)
                self._on_event({"tipo": "pausa", "motivo": motivo})
            return
        if self.estado == PAUSA:
            self._cambiar(self._previo)
            self._esperar(c["asentamiento_s"])
            return
        if self._ocupado() or ahora < self._t_espera:
            return
        obj = self._objetivo(c)
        if obj is not None:
            self._t_visto = ahora
            self.ultimo_objetivo = obj
        getattr(self, "_paso_" + self.estado)(c, ahora, obj)

    # ---- estados
    def _paso_buscando(self, c, ahora, obj):
        if obj is not None:
            self.pulsos = 0
            self._cambiar(CENTRANDO)
            return
        angulos = c["angulos_barrido_deg"]
        if ahora - self._t_barrido >= c["dwell_barrido_s"]:
            self._t_barrido = ahora
            if self._i_barrido >= len(angulos):                       # barrido completo sin ver nada: gira y repite
                self._i_barrido = 0
                if self.giros_busqueda >= c["max_giros_busqueda"]:
                    self._terminar("sin objetivo tras girar toda la vuelta")
                    return
                self.giros_busqueda += 1
                self._mover(self._dir_busqueda, c)
                self._esperar(c["asentamiento_s"])
                return
            self._set_servo(angulos[self._i_barrido])
            self._i_barrido += 1
            self._esperar(0.6)

    def _perdido(self, c, ahora) -> bool:
        if ahora - self._t_visto > c["perder_objetivo_s"]:
            self._cambiar(BUSCANDO)
            return True
        return False

    def _paso_centrando(self, c, ahora, obj):
        if obj is None:
            self._perdido(c, ahora)
            return
        if self._ajustar_tilt(obj, c):
            return
        if abs(obj["yaw_deg"]) > c["tol_rumbo_deg"]:
            if self.pulsos >= c["max_pulsos_objetivo"]:
                self._terminar("no logré centrar el objetivo")
                return
            self._pulso("derecha" if obj["yaw_deg"] > 0 else "izquierda", c)
            return
        self._cambiar(ACERCANDO)

    def _paso_acercando(self, c, ahora, obj):
        if obj is None:
            self._perdido(c, ahora)
            return
        if abs(obj["yaw_deg"]) > 2 * c["tol_rumbo_deg"]:
            self._cambiar(CENTRANDO)
            return
        self._ajustar_tilt(obj, c)
        gap = self._distancias().get("adelante")
        if gap is not None and gap <= c["distancia_inspeccion_m"] + c["tol_distancia_m"]:
            self._cambiar(ENCUADRANDO)
            return
        if gap is not None and gap < c["no_avanzar_bajo_m"]:
            self._cambiar(ENCUADRANDO)                             # demasiado cerca para seguir: se inspecciona desde aquí
            return
        if self.pulsos >= c["max_pulsos_objetivo"]:
            self._cambiar(ENCUADRANDO)                             # no llegó a la distancia: se inspecciona desde donde está
            return
        self._pulso("adelante", c)

    def _paso_encuadrando(self, c, ahora, obj):
        if obj is None:
            self._perdido(c, ahora)
            return
        if not self._ajustar_tilt(obj, c) or ahora - self._t_estado > 5.0:
            self._muestras = []
            self._t_analisis = ahora
            self._cambiar(ANALIZANDO)

    def _paso_analizando(self, c, ahora, obj):
        if obj is not None:
            self._muestras.append((obj["clase"], obj["conf"]))
        if ahora - self._t_analisis < c["analisis_s"]:
            return
        if not self._muestras:
            self._cambiar(BUSCANDO)
            return
        cuenta = Counter()
        for clase, conf in self._muestras:
            cuenta[clase] += conf
        clase = cuenta.most_common(1)[0][0]
        confs = [cf for cl, cf in self._muestras if cl == clase]
        res = {"t": time.time(), "clase": clase, "conf_media": round(sum(confs) / len(confs), 3), "muestras": len(self._muestras),
               "yaw_deg": (self.ultimo_objetivo or {}).get("yaw_deg"), "hueco_m": self._distancias().get("adelante"),
               "servo": self._get_servo(), "pulsos": self.pulsos}
        if self._capturar is not None:
            try:
                res["foto"] = self._capturar(res)
            except Exception as exc:
                res["foto_error"] = str(exc)
        if self._clasificar is not None and res.get("foto"):
            try:
                res["diagnostico"] = self._clasificar(res["foto"])
            except Exception as exc:
                res["diagnostico_error"] = f"{type(exc).__name__}: {exc}"
        self.ultimo_resultado = res
        self._registrar(res)
        self._on_event({"tipo": "inspeccion", **res})
        self.objetivos_hechos += 1
        if self.objetivos_hechos >= c["max_objetivos"]:
            self._terminar(f"{self.objetivos_hechos} objetivos inspeccionados")
            return
        self.giros_alejando = 0
        self._dir_busqueda = "derecha" if self._dir_busqueda == "izquierda" else "izquierda"
        self._cambiar(ALEJANDO)

    def _paso_alejando(self, c, ahora, obj):
        if self.giros_alejando >= c["giros_tras_analisis"]:
            self.pulsos = 0
            self._i_barrido = 0
            self._cambiar(BUSCANDO)
            return
        self.giros_alejando += 1
        self._mover(self._dir_busqueda, c)
        self._esperar(c["asentamiento_s"])

    def _paso_pausa(self, c, ahora, obj):                       # se resuelve en paso()
        return

    def _registrar(self, res: dict):
        try:
            self.carpeta.mkdir(parents=True, exist_ok=True)
            with open(self.carpeta / (time.strftime("%Y%m%d") + ".jsonl"), "a") as f:
                f.write(json.dumps(res, ensure_ascii=False) + "\n")
        except OSError:
            pass
