"""ATASCO POR PULSO Y SALIDA ESCALONADA (2026-11-02).

Antes: el explorador declaraba atasco si la pose no se movía 20 cm en 25 s, y salía SIEMPRE igual (un pulso atrás + 3 giros al lado contrario). 25 s es casi
toda la autonomía perdida en cada atasco, y repetir la misma maniobra no saca al robot de lo que ya no lo sacó.

Ahora cada pulso se MIDE: se apunta la pose al mandarlo y se compara con la pose al mandar el siguiente (para entonces el pulso y su pausa ya acabaron).
  · Un pulso recto (adelante/atrás) que avanza menos de `min_recto_m` es un fallo; `fallos_recto` seguidos = atascado (≈ 4–6 s en vez de 25).
  · Un giro que rota menos de `min_giro_deg` es un fallo de giro; `fallos_giro` seguidos = atascado (en exterior los giros ya rinden poco: umbral bajo).
  · Un giro de lazo cerrado que termina «sin avance» (giro_objetivo.py) cuenta como atasco directo.
Al declararlo, `plan()` da una maniobra que SUBE de nivel si el robot vuelve a atascarse sin haber progresado `progreso_m` desde el sitio del atasco:
  nivel 1  atrás + giro de 30° al lado libre
  nivel 2  atrás ×2 + giro de 60° al lado CONTRARIO
  nivel 3  balanceo adelante/atrás (rompe el agarre del barro o una rama) + giro de 90°
  nivel 4  rendirse: parar la autonomía y avisar al operador (seguir insistiendo sólo cava más hondo o pisa plantas)
Los pulsos atrás se quitan si no hay hueco detrás, y los de adelante si no lo hay delante. Lógica pura, sin hardware ni reloj."""
from __future__ import annotations

import math

RECTOS = ("adelante", "atras")
GIROS = ("izquierda", "derecha")


class DetectorAtasco:
    def __init__(self, *, min_recto_m: float = 0.03, min_giro_deg: float = 1.5, fallos_recto: int = 3, fallos_giro: int = 4,
                 progreso_m: float = 0.5, nivel_max: int = 3):
        self.min_recto_m, self.min_giro_deg = float(min_recto_m), float(min_giro_deg)
        self.fallos_recto_max, self.fallos_giro_max = int(fallos_recto), int(fallos_giro)
        self.progreso_m = float(progreso_m)
        self.nivel_max = int(nivel_max)
        self._pend: tuple[str, dict] | None = None
        self.fallos_recto = 0
        self.fallos_giro = 0
        self.atascado = False
        self.motivo: str | None = None
        self.nivel = 0                    # última maniobra usada (0 = ninguna desde el último progreso)
        self._sitio: tuple[float, float] | None = None
        self.declarados = 0
        self.fisico = False               # algún atasco de esta racha fue FÍSICO (pulsos mandados que no movieron): solo así se rinde

    # ------------------------------------------------------------------ medida de pulsos
    def pulso(self, accion: str, pose: dict | None):
        """Se manda un pulso: cierra el anterior con esta pose y apunta el nuevo."""
        self.cerrar(pose)
        if pose is not None and (accion in RECTOS or accion in GIROS):
            self._pend = (accion, pose)

    def cerrar(self, pose: dict | None) -> dict | None:
        """Mide el pulso pendiente con la pose actual. Devuelve {accion, m, deg, fallo} o None."""
        if pose is None:
            self._pend = None                 # sin pose no se puede medir: el pulso no cuenta
            return None
        if self._pend is None:
            return None
        accion, p0 = self._pend
        self._pend = None
        m = math.hypot(pose["x"] - p0["x"], pose["y"] - p0["y"])
        deg = abs((pose["theta_deg"] - p0["theta_deg"] + 180.0) % 360.0 - 180.0)
        self._progreso(pose)
        if accion in RECTOS:
            fallo = m < self.min_recto_m
            self.fallos_recto = self.fallos_recto + 1 if fallo else 0
            if self.fallos_recto >= self.fallos_recto_max:
                self._declarar(pose, f"{self.fallos_recto} pulsos rectos sin moverse (<{self.min_recto_m * 100:.0f} cm)", fisico=True)
        else:
            fallo = deg < self.min_giro_deg
            self.fallos_giro = self.fallos_giro + 1 if fallo else 0
            if self.fallos_giro >= self.fallos_giro_max:
                self._declarar(pose, f"{self.fallos_giro} giros sin girar (<{self.min_giro_deg:.1f}°)", fisico=True)
        return {"accion": accion, "m": round(m, 3), "deg": round(deg, 1), "fallo": fallo}

    def giro_sin_avance(self, pose: dict | None):
        """Un giro hasta un rumbo terminó «sin avance»: atasco directo."""
        self._declarar(pose, "el giro de lazo cerrado no avanza", fisico=True)

    def _declarar(self, pose, motivo, fisico: bool = False):
        """`fisico` = medido en los pulsos (el robot empuja y no se mueve). El criterio viejo del explorador («la pose no cambió 20 cm en 25 s») NO lo
        es: también salta cuando el robot no avanza porque DECIDE no avanzar (en el simulador, la cámara viendo obstáculos falsos en la paja del borde)."""
        if not self.atascado:
            self.declarados += 1
        self.atascado, self.motivo = True, motivo
        self.fisico = self.fisico or fisico
        if pose is not None and self._sitio is None:
            self._sitio = (pose["x"], pose["y"])

    def _progreso(self, pose):
        if self._sitio is not None and math.hypot(pose["x"] - self._sitio[0], pose["y"] - self._sitio[1]) >= self.progreso_m:
            self.nivel, self._sitio, self.fisico = 0, None, False     # salió de verdad: el próximo atasco empieza otra vez por el nivel 1

    # ------------------------------------------------------------------ salida
    def plan(self, *, lado_libre: str, hueco_atras: bool, hueco_delante: bool) -> list[tuple]:
        """La maniobra del siguiente nivel. Elementos: ("adelante",) | ("atras",) | ("girar", grados±) | ("rendirse", motivo)."""
        self.nivel += 1
        self.atascado, self.fallos_recto, self.fallos_giro, self._pend = False, 0, 0, None
        signo = 1.0 if lado_libre == "izquierda" else -1.0
        if self.nivel > self.nivel_max:
            if self.fisico:
                return [("rendirse", f"{self.nivel - 1} maniobras sin salir del sitio ({self.motivo})")]
            self.nivel = 1                    # sin atasco físico medido no se rinde (A/B 2026-11-02: rendirse por el criterio de 25 s dejó al robot
                                              # parado en 4 de 5 escenarios): vuelve a empezar por el nivel 1
        if self.nivel == 1:
            pasos = [("atras",), ("girar", 30.0 * signo)]
        elif self.nivel == 2:
            pasos = [("atras",), ("atras",), ("girar", -60.0 * signo)]
        else:
            pasos = [("adelante",), ("atras",), ("adelante",), ("atras",), ("girar", 90.0 * signo)]
        return [p for p in pasos if not ((p[0] == "atras" and not hueco_atras) or (p[0] == "adelante" and not hueco_delante))]

    def reiniciar(self):
        self.__init__(min_recto_m=self.min_recto_m, min_giro_deg=self.min_giro_deg, fallos_recto=self.fallos_recto_max,
                      fallos_giro=self.fallos_giro_max, progreso_m=self.progreso_m, nivel_max=self.nivel_max)

    def snapshot(self) -> dict:
        return {"atascado": self.atascado, "motivo": self.motivo, "nivel": self.nivel, "declarados": self.declarados,
                "fallos_recto": self.fallos_recto, "fallos_giro": self.fallos_giro}
