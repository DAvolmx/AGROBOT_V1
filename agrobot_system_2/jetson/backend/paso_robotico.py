"""
Movimiento ROBÓTICO por pulsos: pulso -> quieto -> medir -> decidir.
===================================================================

Los pulsos de `pulse_drive.py` son a tiempo: "adelante" = 0,8 s de empuje,
y lo que avance depende del suelo, la batería y el freno (en el pasillo del
2026-10-02 fueron 21-29 cm, y un pulso de giro rotaba 0-3° y hacía retroceder
3-16 cm). Para moverse como un robot hace falta cerrar el lazo:

    avanzar(50 cm):  pulso, esperar a que el robot esté QUIETO (scan
                     estable), medir cuánto avanzó con el LIDAR, decidir
                     otro pulso o terminar.
    girar(90°):      igual, midiendo el rumbo (giróscopo del EKF).

Cada pulso termina parado y medido antes del siguiente: nada se encadena a
ciegas, y el freno por LIDAR sigue actuando sobre cada pulso como siempre.

Qué se mide y por qué:
  · Avance: hueco libre delante y detrás (`LidarReactive.distancias`). Se
    promedia lo que se acercó lo de delante y lo que se alejó lo de atrás;
    si un lado no tiene nada a la vista se usa el otro. NO la pose del EKF:
    rf2o publica la velocidad lineal con el signo invertido (MANIFEST,
    2026-10-02) y el EKF la hereda.
  · Giro: rumbo de la pose fusionada, que sale del giróscopo Z (medido
    contra ICP de scans el 2026-10-02: coinciden en ±1°).

Fallos que detecta en vez de insistir:
  · "sin_progreso": N pulsos seguidos sin moverse (atascado, rueda suelta,
    batería) -> para y lo dice.
  · "obstaculo": el hueco en la dirección de avance ya está bajo el umbral
    del freno por LIDAR (el pulso no saldría): no es un atasco, no cabe.
  · "retroceso": al girar el robot retrocede (medido) y se queda sin hueco
    detrás -> para antes de tocar.
  · "sin_lidar": scan viejo, no se puede medir.

Cada pulso queda registrado (antes/después/orden) en JSONL con la misma forma
que `logs/sesion_20261002_manejo/pasos.jsonl`, para recalibrar el simulador
con `tools/calibrar_con_pasillo.py`.
"""
from __future__ import annotations

import asyncio
import json
import math
import time
from pathlib import Path

RUTA_REGISTRO = Path.home() / ".agrobot" / "pasos_roboticos"

# Quieto = el hueco delantero y trasero no cambia más que esto en QUIETO_S.
QUIETO_TOL_M = 0.01
QUIETO_S = 0.3
QUIETO_MAX_S = 2.0        # tope de espera a que pare (desliza al frenar)
SCAN_VIEJO_S = 1.0
# Un pulso que mueve menos que esto no cuenta como progreso.
PROGRESO_MIN_M = 0.03
PROGRESO_MIN_DEG = 1.0
MAX_SIN_PROGRESO = 3      # pulsos seguidos sin moverse -> se rinde
MAX_PULSOS = 40           # tope absoluto por orden
TOL_AVANCE_M = 0.08       # llegar a ±8 cm (un pulso son ~25)
TOL_GIRO_DEG = 5.0
# Hueco mínimo detrás para seguir girando (el giro retrocede, medido).
HUECO_TRASERO_GIRO_M = 0.15


def _envolver(grados: float) -> float:
    return (grados + 180.0) % 360.0 - 180.0


class PasoRobotico:
    def __init__(self, *, ordenar, parar, conductor_activo, distancias, rumbo_deg,
                 edad_scan_s, umbral_freno_m=lambda: 0.57, escala=0.4, ruta_registro: Path | None = RUTA_REGISTRO,
                 reloj=time.monotonic, dormir=asyncio.sleep, periodo_s=0.05):
        """
        ordenar(accion, escala, maniobra)  manda UN pulso al conductor.
        parar()                            parada inmediata.
        conductor_activo() -> bool         el pulso sigue en marcha.
        distancias() -> {adelante, atras, ...} en metros (None = nada a la vista).
        rumbo_deg() -> float | None        rumbo del robot (EKF/giróscopo).
        edad_scan_s() -> float             antigüedad del último scan.
        umbral_freno_m() -> float          hueco al que corta el freno por LIDAR.
        """
        self._ordenar = ordenar
        self._parar = parar
        self._activo = conductor_activo
        self._dist = distancias
        self._rumbo = rumbo_deg
        self._edad_scan = edad_scan_s
        self._umbral = umbral_freno_m
        self.escala = escala
        self._reloj = reloj
        self._dormir = dormir
        self._periodo = periodo_s
        self._ruta = ruta_registro
        self._archivo = None
        self.tarea: asyncio.Task | None = None
        self.estado = {"activo": False, "orden": None, "resultado": None}

    # ------------------------------------------------------------ registro
    def _registrar(self, reg: dict):
        if self._ruta is None:
            return
        try:
            if self._archivo is None:
                self._ruta.mkdir(parents=True, exist_ok=True)
                self._archivo = self._ruta / time.strftime("pasos_%Y%m%d_%H%M%S.jsonl")
            with open(self._archivo, "a") as f:
                f.write(json.dumps(reg, ensure_ascii=False) + "\n")
        except OSError:
            pass                      # registrar nunca debe parar al robot

    # ------------------------------------------------------------ medición
    async def _esperar_quieto(self) -> dict:
        """Espera a que acabe el pulso y el robot deje de deslizar."""
        while self._activo():
            await self._dormir(self._periodo)
        t0 = self._reloj()
        ref = self._dist()
        ref_t = t0
        while self._reloj() - t0 < QUIETO_MAX_S:
            await self._dormir(self._periodo)
            d = self._dist()
            if any(_cambio(ref.get(k), d.get(k)) > QUIETO_TOL_M for k in ("adelante", "atras")):
                ref, ref_t = d, self._reloj()
            elif self._reloj() - ref_t >= QUIETO_S:
                return d
        return self._dist()

    @staticmethod
    def _avance(antes: dict, despues: dict) -> float | None:
        """Lo que avanzó (m, + = hacia delante) según los huecos del LIDAR."""
        medidas = []
        if antes.get("adelante") is not None and despues.get("adelante") is not None:
            medidas.append(antes["adelante"] - despues["adelante"])
        if antes.get("atras") is not None and despues.get("atras") is not None:
            medidas.append(despues["atras"] - antes["atras"])
        if not medidas:
            return None
        return sum(medidas) / len(medidas)

    # ------------------------------------------------------------ órdenes
    def iniciar(self, accion: str, cantidad: float) -> bool:
        """accion = "avanzar" (cm, negativo = atrás) o "girar" (grados, + = izquierda)."""
        if self.tarea is not None and not self.tarea.done():
            return False
        coro = self.avanzar(cantidad / 100.0) if accion == "avanzar" else self.girar(cantidad)
        self.tarea = asyncio.create_task(coro)
        return True

    def detener(self):
        if self.tarea is not None and not self.tarea.done():
            self.tarea.cancel()
        self._parar()
        self.estado.update(activo=False, resultado="detenido")

    async def avanzar(self, metros: float) -> dict:
        accion = "adelante" if metros >= 0 else "atras"
        objetivo = abs(metros)
        return await self._bucle("avanzar", round(metros * 100, 1), accion, objetivo, es_giro=False)

    async def girar(self, grados: float) -> dict:
        accion = "izquierda" if grados >= 0 else "derecha"
        return await self._bucle("girar", grados, accion, abs(grados), es_giro=True)

    async def _bucle(self, nombre, pedido, accion, objetivo, es_giro) -> dict:
        unidad = "grados" if es_giro else "cm"
        self.estado = {"activo": True, "orden": {"accion": nombre, unidad: pedido},
                       "hecho": 0.0, "pulsos": 0, "resultado": None}
        hecho, sin_progreso, pulsos = 0.0, 0, 0
        tol = TOL_GIRO_DEG if es_giro else TOL_AVANCE_M
        resultado = "completado"
        try:
            while objetivo - hecho > tol:
                if pulsos >= MAX_PULSOS:
                    resultado = "max_pulsos"
                    break
                if self._edad_scan() > SCAN_VIEJO_S:
                    resultado = "sin_lidar"
                    break
                antes = self._dist()
                r0 = self._rumbo()
                if es_giro and antes.get("atras") is not None and antes["atras"] < HUECO_TRASERO_GIRO_M:
                    resultado = "retroceso"
                    break
                if not es_giro:
                    hueco = antes.get(accion)
                    if hueco is not None and hueco <= self._umbral():
                        # El freno cortaría el pulso: no es un atasco, es que no cabe.
                        resultado = "obstaculo"
                        break
                t0 = self._reloj()
                self._ordenar(accion, self.escala, False)
                pulsos += 1
                despues = await self._esperar_quieto()
                r1 = self._rumbo()

                avance = self._avance(antes, despues)
                giro = (None if r0 is None or r1 is None
                        else _envolver(r1 - r0) * (1 if accion == "izquierda" else -1))
                progreso = giro if es_giro else (None if avance is None
                                                 else avance * (1 if accion == "adelante" else -1))
                self._registrar({
                    "t": time.strftime("%H:%M:%S"), "orden": nombre, "accion": accion,
                    "escala": self.escala, "pulso": pulsos,
                    "lidar_antes_m": antes, "lidar_despues_m": despues,
                    "avance_m": None if avance is None else round(avance, 3),
                    "giro_deg": None if giro is None else round(giro, 2),
                    "rumbo_antes_deg": r0, "rumbo_despues_deg": r1,
                    "duracion_s": round(self._reloj() - t0, 2),
                })
                if progreso is None:
                    resultado = "sin_medida"
                    break
                minimo = PROGRESO_MIN_DEG if es_giro else PROGRESO_MIN_M
                if progreso < minimo:
                    sin_progreso += 1
                    if sin_progreso >= MAX_SIN_PROGRESO:
                        resultado = "sin_progreso"
                        break
                else:
                    sin_progreso = 0
                hecho += max(0.0, progreso)
                self.estado.update(hecho=round(hecho, 3), pulsos=pulsos)
        except asyncio.CancelledError:
            resultado = "detenido"
            raise
        finally:
            self._parar()
            self.estado.update(activo=False, resultado=resultado, hecho=round(hecho, 3),
                               pulsos=pulsos)
        return dict(self.estado)


def _cambio(a, b) -> float:
    if a is None and b is None:
        return 0.0
    if a is None or b is None:
        return math.inf
    return abs(a - b)
