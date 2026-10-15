"""
Modo seguro con protección de fallos (FDIR: detección, aislamiento y
recuperación de fallos) — el esquema que usan las naves y róveres de la
NASA/JPL: monitores independientes vigilan cada subsistema y, ante un fallo,
el vehículo NO sigue a ciegas: pasa solo a un estado seguro conocido, avisa
y espera a que un operador lo revise ("safe mode").

Tres niveles:
  NORMAL      todo bien, sin recortes.
  PRECAUCION  algo degradado pero manejable: la velocidad se recorta.
  SEGURO      fallo serio: se detiene la autonomía y los motores; el mando
              manual sigue permitido, lento, para sacar al robot del apuro.
              Queda ENGANCHADO hasta que el operador lo rearma y la causa
              ya no está (un LIDAR que vuelve a medio camino no basta para
              que el robot reanude solo una patrulla).

Este módulo solo decide: recibe un dict con el estado del robot y devuelve
nivel, motivos y factor de velocidad. Quien llama ejecuta las acciones. Así
se prueba sin el robot.
"""
from __future__ import annotations

import math
import time

NORMAL, PRECAUCION, SEGURO = "normal", "precaucion", "seguro"

# Umbrales (los de LIDAR y EKF salen de fallos reales documentados en el
# MANIFEST: LIDAR USB que se cae, EKF que extrapola y estira el mapa).
LIDAR_PRECAUCION_S = 0.6     # scan algo viejo: ir con cuidado
LIDAR_SEGURO_S = 2.0         # sin scan moviéndose: no se ve el camino
C3_SEGURO_S = 1.5            # sin estado de la C3 con autonomía en marcha
EKF_VARIANZA_SEGURO = 1.0    # = EKF_VARIANZA_DISPARADA del backend
TEMP_PRECAUCION_C = 80.0     # tj de la Orin
TEMP_SEGURO_C = 95.0

FACTOR_PRECAUCION = 0.5
FACTOR_MANUAL_EN_SEGURO = 0.4

# Vuelco / inclinación. Añadido el 2026-10-09 tras volcar con el firmware NO
# adaptado al chasis nuevo: el eje delantero estrecho (19 cm) y el CG alto
# (~20 cm) dan un ángulo de vuelco lateral de solo ~29.5° (ver fallos_3d.py del
# simulador), y la seguridad era CIEGA a la inclinación (todo salía "normal").
# Un talud de huerta inclina de forma SOSTENIDA con balanceo ~0; un vuelco
# inclina rápido (alta velocidad de balanceo). Por eso:
#   >= PRECAUCION          -> recorta velocidad (bueno también en pendientes).
#   >= SEGURO              -> ya pasó el vuelco (>29.5°): para y engancha.
#   >= PRECAUCION + balanceo rápido -> vuelco en curso: para aunque no llegue
#                            al ángulo SEGURO, para cortar el par cuanto antes.
INCLINACION_PRECAUCION_DEG = 22.0
INCLINACION_SEGURO_DEG = 38.0
ROLL_RATE_VUELCO_DPS = 90.0


def inclinacion_desde_acc(acc) -> float | None:
    """Inclinación respecto a la vertical (grados) desde el acelerómetro en g.

    Solo es fiable cuando el módulo del vector está cerca de 1 g (en reposo o a
    velocidad casi constante): con aceleración fuerte el vector ya no es la
    gravedad pura, así que se descarta la muestra (devuelve None). Se usa el
    signo de az: az>0 arriba = derecho; az<0 = boca abajo (~180°). Esto asume
    el montaje del AGROBOT, donde en reposo accelZ ≈ +1 g (medido 2026-10-09)."""
    try:
        ax, ay, az = (float(v) for v in acc)
    except (TypeError, ValueError):
        return None
    mag = math.sqrt(ax * ax + ay * ay + az * az)
    if not (0.6 <= mag <= 1.4):
        return None
    return math.degrees(math.acos(max(-1.0, min(1.0, az / mag))))


class ModoSeguro:
    def __init__(self, reloj=time.monotonic):
        self._reloj = reloj
        self.nivel = NORMAL
        self.motivos: list[str] = []
        self.enganchado = False          # SEGURO pendiente de rearme
        self.motivos_enganche: list[str] = []
        self.desde = reloj()

    def evaluar(self, e: dict) -> dict:
        """`e`: moviendose, autonomia, edad_lidar_s, edad_c3_s, atascado,
        patinaje, temp_c, ekf_var_vx, mapa_corrupto, inclinacion_deg (o `acc`
        = (ax,ay,az) en g), giro_roll_dps. Las claves que falten se toman como
        "sin dato" y no disparan nada."""
        graves, leves = [], []
        mov = bool(e.get("moviendose"))
        auto = bool(e.get("autonomia"))

        lidar = e.get("edad_lidar_s")
        if lidar is not None and (mov or auto):
            if lidar >= LIDAR_SEGURO_S:
                graves.append(f"LIDAR sin datos hace {lidar:.1f} s")
            elif lidar >= LIDAR_PRECAUCION_S:
                leves.append(f"LIDAR con retraso ({lidar:.1f} s)")

        c3 = e.get("edad_c3_s")
        if auto and c3 is not None and c3 >= C3_SEGURO_S:
            graves.append(f"sin estado de la C3 hace {c3:.1f} s")

        if e.get("atascado"):
            graves.append("robot atascado: se ordena avanzar y el LIDAR no ve movimiento")

        var = e.get("ekf_var_vx")
        if auto and var is not None and var > EKF_VARIANZA_SEGURO:
            graves.append(f"EKF sin medidas (varianza {var:.2f})")
        if auto and e.get("mapa_corrupto"):
            graves.append("mapa del SLAM corrupto")

        temp = e.get("temp_c")
        if temp is not None:
            if temp >= TEMP_SEGURO_C:
                graves.append(f"Jetson a {temp:.0f} °C")
            elif temp >= TEMP_PRECAUCION_C:
                leves.append(f"Jetson caliente ({temp:.0f} °C)")

        pat = e.get("patinaje")
        if pat is not None and pat >= 0.4:
            leves.append(f"patinaje {pat * 100:.0f} %")

        inc = e.get("inclinacion_deg")
        if inc is None and e.get("acc") is not None:
            inc = inclinacion_desde_acc(e.get("acc"))
        rate = e.get("giro_roll_dps")
        if inc is not None:
            vuelco_rapido = (inc >= INCLINACION_PRECAUCION_DEG and rate is not None
                             and abs(rate) >= ROLL_RATE_VUELCO_DPS)
            if inc >= INCLINACION_SEGURO_DEG or vuelco_rapido:
                graves.append(f"VUELCO: inclinación {inc:.0f}°"
                              + (f" a {abs(rate):.0f}°/s" if rate else ""))
            elif inc >= INCLINACION_PRECAUCION_DEG:
                leves.append(f"inclinación alta ({inc:.0f}°)")

        if graves and not self.enganchado:
            self.enganchado = True
            self.motivos_enganche = list(graves)

        if self.enganchado:
            nivel = SEGURO
            motivos = list(dict.fromkeys(self.motivos_enganche + graves + leves))
        elif leves:
            nivel, motivos = PRECAUCION, leves
        else:
            nivel, motivos = NORMAL, []

        if nivel != self.nivel:
            self.desde = self._reloj()
        self.nivel, self.motivos = nivel, motivos
        self._graves_actuales = graves
        return self.estado()

    def rearmar(self) -> tuple[bool, str]:
        """El operador pide salir de SEGURO. Solo si la causa ya no está."""
        if not self.enganchado:
            return True, "no estaba en modo seguro"
        if getattr(self, "_graves_actuales", []):
            return False, "la causa sigue presente: " + "; ".join(self._graves_actuales)
        self.enganchado = False
        self.motivos_enganche = []
        self.nivel, self.motivos = NORMAL, []
        self.desde = self._reloj()
        return True, "modo seguro rearmado"

    def factor(self, manual: bool) -> float:
        if self.nivel == SEGURO:
            return FACTOR_MANUAL_EN_SEGURO if manual else 0.0
        if self.nivel == PRECAUCION:
            return FACTOR_PRECAUCION
        return 1.0

    def estado(self) -> dict:
        return {"nivel": self.nivel, "motivos": self.motivos,
                "enganchado": self.enganchado,
                "desde_s": round(self._reloj() - self.desde, 1)}
