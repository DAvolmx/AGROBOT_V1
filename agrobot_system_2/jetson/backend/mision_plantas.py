"""MISIÓN «PLANTAS EN MACETA» para la demo (2026-10-22): explora y mapea; al ver una maceta enciende el LED, se acerca sin chocar,
la escanea desde varias vistas mientras GRABA video, guarda todo y sigue explorando. No repite plantas ya vistas (memoria en el mapa).

  EXPLORANDO       el explorador reactivo (LIDAR + cámara para obstáculos bajos + rumbo fusionado) recorre la sala; el SLAM mapea.
  INSPECCIONANDO   se para el explorador y la visión activa (clase «maceta») centra, se acerca a ~45 cm, analiza y escanea las vistas.
  FIN              tras `max_plantas` o `tiempo_max_s`.
LED: encendido mientras se ve una maceta (o durante la inspección). Video: de principio a fin de cada inspección.
Lógica pura con dependencias inyectadas (paso() síncrono, reloj inyectable): la usa el robot y el simulador (simular_demo.py) igual.
"""
from __future__ import annotations

import asyncio
import math
import time

DEFECTOS = {
    "max_plantas": 12,                 # una por una: ve una, la escanea, la marca y sigue con la siguiente hasta agotar el recorrido o el tiempo
    "tiempo_max_s": 900.0,
    "conf_min": 0.35,           # la col marchita sale con 0,42-0,48 en la cámara del robot (2026-10-23); la persistencia frena falsos
    "persistencia": 3,                 # cuadros seguidos viendo la maceta antes de ir a por ella
    "distancia_max_m": 2.5,            # solo se va a por macetas a menos de esto (por cámara)
    "radio_ya_vista_m": 0.28,          # una maceta a menos de esto de una ya inspeccionada es la misma (0,9 tapaba macetas en fila a 30 cm, 2026-10-23)
    "pausa_tras_planta_s": 6.0,        # tras inspeccionar, se ignoran detecciones un rato (la que acaba de ver sigue en imagen)
    "led_mantener_s": 1.5,
}
EXPLORANDO, INSPECCIONANDO, FIN, INACTIVA = "explorando", "inspeccionando", "fin", "inactiva"


class MisionPlantas:
    def __init__(self, *, explorador, vision, get_macetas, get_pose, led, grabar, get_config=lambda: {}, on_event=None,
                 reloj=time.monotonic):
        self.expl = explorador             # iniciar() -> (ok, msg), detener(), .activo
        self.vision = vision               # iniciar() -> (ok, msg), cancelar(m), arrancar_tarea(), .estado, .ultimo_resultado
        self._macetas = get_macetas        # () -> [{conf, yaw_deg (+ = derecha), dist_m|None}] del último cuadro, o []
        self._pose = get_pose              # () -> {x, y, theta_deg} | None
        self._led = led                    # (bool)
        self._grabar = grabar              # (bool) -> inicia / detiene el video
        self._cfg = get_config
        self._ev = on_event or (lambda e: None)
        self.reloj = reloj
        self.estado = INACTIVA
        self.plantas: list[dict] = []
        self._racha = 0
        self._led_on = None
        self._t_visto = -1e9
        self._t_ignorar = -1e9
        self._t0 = 0.0
        self._tarea = None
        self.motivo_fin = None
        self.pausado = False
        self._t_pausa = 0.0

    def cfg(self):
        c = dict(DEFECTOS)
        c.update(self._cfg() or {})
        return c

    # ---------------- ciclo de vida
    def iniciar(self):
        if self.estado not in (INACTIVA, FIN):
            return False, "ya está en marcha"
        self.plantas, self._racha, self.motivo_fin = [], 0, None
        self._t0 = self.reloj()
        ok, msg = self.expl.iniciar()
        if not ok and not getattr(self.expl, "activo", False):
            return False, f"no arrancó la exploración: {msg}"
        self.estado = EXPLORANDO
        self._ev({"tipo": "mision_inicio"})
        return True, "ok"

    def pausar(self):
        """Mando manual encima: la misión sigue EN MARCHA (la interfaz no pasa a manual); el explorador y la visión se paran."""
        if self.estado in (INACTIVA, FIN) or self.pausado:
            return
        self.pausado, self._t_pausa = True, self.reloj()
        try:
            if self.vision.estado != "inactiva":
                if hasattr(self.vision, "pausa_externa"):
                    self.vision.pausa_externa = True       # la inspección se PAUSA y sigue donde iba (antes se cancelaba y se perdía)
                else:
                    self.vision.cancelar("mando manual")
        finally:
            if hasattr(self.expl, "pausar"):
                self.expl.pausar()
            else:
                self.expl.detener()

    def continuar(self):
        if not self.pausado:
            return
        self.pausado = False
        if hasattr(self.vision, "pausa_externa"):
            self.vision.pausa_externa = False
        self._t0 += self.reloj() - self._t_pausa        # el rato en pausa no gasta el tiempo máximo
        self._racha = 0
        self._t_ignorar = self.reloj() + 1.0
        if hasattr(self.expl, "continuar"):
            self.expl.continuar()
        else:
            self._reiniciar_expl()

    def _reiniciar_expl(self):
        """Reinicia el explorador SIN el barrido inmediato: tras inspeccionar una maceta (o fallar) volver a escanear en seguida
        provocaba un bucle de escaneos de 8 s que el siguiente reconocimiento interrumpía (2026-10-23)."""
        previo = getattr(self.expl, "escaneo_al_iniciar", True)
        self.expl.escaneo_al_iniciar = False
        try:
            return self.expl.iniciar()
        finally:
            self.expl.escaneo_al_iniciar = previo

    def reanudar(self):
        """Vuelve a explorar tras un mando manual conservando las plantas ya inspeccionadas."""
        if self.estado != FIN or self.motivo_fin != "mando manual":
            return False, "no hay una misión interrumpida por el mando manual"
        ok, msg = self._reiniciar_expl()
        if not ok and not getattr(self.expl, "activo", False):
            return False, f"no arrancó la exploración: {msg}"
        self._racha, self.motivo_fin = 0, None
        self._t_ignorar = self.reloj() + 2.0
        self.estado = EXPLORANDO
        self._ev({"tipo": "mision_reanudada", "plantas": len(self.plantas)})
        return True, "ok"

    def detener(self, motivo="detenida"):
        if self.estado in (INACTIVA, FIN):
            return
        try:
            if self.vision.estado != "inactiva":
                self.vision.cancelar("misión detenida")
            if hasattr(self.vision, "filtro_objetivo"):
                self.vision.filtro_objetivo = None
            self.expl.detener()
            self._grabar(False)
        finally:
            self._poner_led(False)
            self.estado = FIN
            self.pausado = False
            self.motivo_fin = motivo
            self._ev({"tipo": "mision_fin", "motivo": motivo, "plantas": len(self.plantas)})

    def _poner_led(self, on: bool):
        if on != self._led_on:
            self._led_on = on
            self._led(on)

    # ---------------- decisión
    def _posicion_estimada(self, m):
        p = self._pose()
        if not p or m.get("dist_m") is None:
            return None
        rumbo = math.radians(p["theta_deg"] - m["yaw_deg"])          # yaw + = a la derecha
        d = float(m["dist_m"]) + 0.15                               # hasta el centro de la maceta
        return (p["x"] + d * math.cos(rumbo), p["y"] + d * math.sin(rumbo))

    def _ya_vista(self, pos, c):
        return pos is not None and any(math.hypot(pos[0] - q["x"], pos[1] - q["y"]) < c["radio_ya_vista_m"]
                                       for q in self.plantas if q.get("x") is not None)

    def paso(self):
        if self.estado in (INACTIVA, FIN) or self.pausado:
            return
        c = self.cfg()
        ahora = self.reloj()
        if ahora - self._t0 > c["tiempo_max_s"]:
            self.detener("tiempo máximo")
            return
        macetas = [m for m in (self._macetas() or []) if m.get("conf", 0) >= c["conf_min"]]
        if macetas:
            self._t_visto = ahora
        self._poner_led(self.estado == INSPECCIONANDO or ahora - self._t_visto < c["led_mantener_s"])

        if self.estado == EXPLORANDO:
            if not getattr(self.expl, "activo", True):                  # el explorador se paró solo (p. ej. sin LIDAR): reintentar
                self._reiniciar_expl()
            candidatas = [m for m in macetas if (m.get("dist_m") is None or m["dist_m"] <= c["distancia_max_m"])]
            nuevas = [m for m in candidatas if not self._ya_vista(self._posicion_estimada(m), c)]
            self._racha = self._racha + 1 if (nuevas and ahora >= self._t_ignorar) else 0
            if self._racha >= c["persistencia"]:
                m = max(nuevas, key=lambda q: q["conf"])
                self.expl.detener()
                if hasattr(self.vision, "filtro_objetivo"):
                    self.vision.filtro_objetivo = lambda q, c=c: not self._ya_vista(self._posicion_estimada(q), c)
                ok, msg = self.vision.iniciar()
                if not ok:
                    self._ev({"tipo": "aviso", "msg": f"visión activa no arrancó: {msg}"})
                    self._reiniciar_expl()
                    self._racha = 0
                    return
                if hasattr(self.vision, "arrancar_tarea"):
                    self.vision.arrancar_tarea()
                self._grabar(True)
                self._objetivo = {"estimada": self._posicion_estimada(m), "t": ahora, "conf": m["conf"]}
                self.estado = INSPECCIONANDO
                self._ev({"tipo": "planta_detectada", "conf": m["conf"], "dist_m": m.get("dist_m"), "yaw_deg": m["yaw_deg"],
                          "n": len(self.plantas) + 1})
            return

        if self.estado == INSPECCIONANDO and self.vision.estado == "inactiva":
            self._grabar(False)
            if hasattr(self.vision, "filtro_objetivo"):
                self.vision.filtro_objetivo = None
            if getattr(self.vision, "objetivos_hechos", 1) == 0:
                # la visión terminó SIN analizar (perdió la maceta): no es una planta escaneada, y ultimo_resultado sería el de la anterior
                self._ev({"tipo": "inspeccion_fallida", "motivo": getattr(self.vision, "motivo_fin", None)})
                self._t_ignorar = ahora + c["pausa_tras_planta_s"]
                self._racha = 0
                self._reiniciar_expl()
                self.estado = EXPLORANDO
                return
            res = getattr(self.vision, "ultimo_resultado", None) or {}
            p = self._pose()
            pos = self._objetivo.get("estimada")
            ult = getattr(self.vision, "ultimo_objetivo", None) or {}
            if res.get("pose") and res.get("dist_cam_m") is not None:     # pose y distancia del momento del análisis (antes de alejarse)
                pa = res["pose"]
                r = math.radians(pa["theta_deg"] - (res.get("yaw_deg") or 0.0))
                d = float(res["dist_cam_m"]) + 0.15
                pos = (pa["x"] + d * math.cos(r), pa["y"] + d * math.sin(r))
            elif p and ult.get("dist_m") is not None:
                # La posición sale de la CAJA de la cámara (rumbo + distancia), no del hueco del LIDAR: si al acercarse se interpuso una pata
                # de silla, el LIDAR mide la pata y la planta quedaba mal ubicada (simulación 2026-10-22: la misma planta escaneada dos veces).
                pos = self._posicion_estimada({"yaw_deg": ult.get("yaw_deg", 0.0), "dist_m": ult["dist_m"]})
            if self._ya_vista(pos, c):
                # Ya estaba en la lista: la visión acabó en una planta escaneada antes. No cuenta como nueva.
                self._ev({"tipo": "planta_repetida", "x": round(pos[0], 2), "y": round(pos[1], 2)})
                self._t_ignorar = ahora + c["pausa_tras_planta_s"]
                self._racha = 0
                self._reiniciar_expl()
                self.estado = EXPLORANDO
                return
            esp = (res.get("diagnostico") or {}).get("especie") or {}
            self.plantas.append({"n": len(self.plantas) + 1, "x": None if pos is None else round(pos[0], 2),
                                 "y": None if pos is None else round(pos[1], 2), "clase": res.get("clase"),
                                 "especie": esp.get("especie"), "p_col": esp.get("p_col"),
                                 "vistas": len(res.get("vistas", [])), "motivo_fin_vision": getattr(self.vision, "motivo_fin", None)})
            self._ev({"tipo": "planta_inspeccionada", **self.plantas[-1]})
            if len(self.plantas) >= c["max_plantas"]:
                self.detener(f"{len(self.plantas)} plantas inspeccionadas")
                return
            self._t_ignorar = ahora + c["pausa_tras_planta_s"]
            self._racha = 0
            self._reiniciar_expl()
            self.estado = EXPLORANDO

    async def _bucle(self):
        while self.estado not in (INACTIVA, FIN):
            try:
                self.paso()
            except Exception as exc:
                self.detener(f"error: {type(exc).__name__}: {exc}")
                break
            await asyncio.sleep(0.25)

    def arrancar_tarea(self):
        self._tarea = asyncio.create_task(self._bucle())

    def estado_dict(self):
        return {"estado": self.estado, "plantas": self.plantas, "motivo_fin": self.motivo_fin,
                "vision": getattr(self.vision, "estado", None)}
