"""
Supervisor central de movimiento — propiedad del mando, emergencia y límites.
=============================================================================

NÚCLEO DE DECISIÓN PURO: no toca hardware, ni ROS, ni asyncio. Recibe órdenes
de velocidad de distintos PRODUCTORES (manual, navegación Nav2, exploración,
inspección, recuperación) y decide cuál pasa, enclava la emergencia, rechaza
órdenes caducas o de quien no tiene el mando, y recorta a los límites físicos.
Así se puede probar en aislado (ver tests/test_supervisor_movimiento.py) antes
de cablearlo, y se convierte en la PROTECCIÓN FINAL COMÚN por la que, una vez
integrado, pasará TODO lo que mueve el robot (manual, Nav2, BLE, UDP).

Lo que este supervisor SÍ hace (Fase 2 de la auditoría):
  · Un único propietario del mando en cada momento. Las órdenes de cualquier
    otro productor se rechazan (así una tarea autónoma que quedó colgando no
    puede mover el robot cuando ya no manda: cancelación efectiva en la puerta).
  · Estados explícitos: DETENIDO, MANUAL, NAVEGACION, EXPLORACION, INSPECCION,
    RECUPERACION, EMERGENCIA, con transiciones documentadas.
  · Tomar el mando manual SUSPENDE la autonomía de forma determinista. Al soltar
    el manual se vuelve a DETENIDO y la autonomía NO se reanuda sola.
  · Emergencia ENCLAVADA: mientras está activa, toda orden de movimiento se
    rechaza (devuelve 0,0). Solo se sale con rearme explícito y con salud OK. El
    rearme NO restaura consignas ni propietario previos: arranca en DETENIDO.
  · Caducidad: una orden más vieja que `deadman_s` se rechaza (evita que el
    último valor residual siga empujando si el productor se calló).
  · Límites unificados: recorta v y omega a los máximos físicos reales del
    chasis (firmware V_MAX/OMEGA_MAX). Es el único lugar donde se fija el tope.

Lo que este supervisor NO hace (a propósito, y para no pisar otras capas):
  · NO frena por obstáculos (eso es `slam.LidarReactive` / costmap, capa aparte
    que el usuario mantiene bajo su control). Integrar el freno por LIDAR es una
    decisión separada y pendiente de validación.
  · NO habla con el C3 ni con ROS: devuelve la orden permitida; quien la envía
    es el backend al integrarlo.
"""
import time
from enum import Enum


class Modo(str, Enum):
    DETENIDO = "detenido"
    MANUAL = "manual"
    NAVEGACION = "navegacion"
    EXPLORACION = "exploracion"
    INSPECCION = "inspeccion"
    RECUPERACION = "recuperacion"
    EMERGENCIA = "emergencia"


# Modos en los que manda una tarea AUTÓNOMA (no el operador).
AUTONOMOS = frozenset({Modo.NAVEGACION, Modo.EXPLORACION,
                       Modo.INSPECCION, Modo.RECUPERACION})

# Qué propietario (token de productor) corresponde a cada modo. El manual tiene
# prioridad sobre todos: puede arrebatar el mando a cualquier autónomo.
PROPIETARIO_DE_MODO = {
    Modo.MANUAL: "manual",
    Modo.NAVEGACION: "nav2",
    Modo.EXPLORACION: "exploracion",
    Modo.INSPECCION: "inspeccion",
    Modo.RECUPERACION: "recuperacion",
}

# Motivos de rechazo (strings estables para logs y tests).
OK = "ok"
R_EMERGENCIA = "emergencia_enclavada"
R_SIN_MANDO = "sin_mando"
R_CADUCA = "orden_caduca"
R_DETENIDO = "detenido"


class SupervisorMovimiento:
    """Puerta única por la que pasa toda orden de movimiento."""

    def __init__(self, *, v_max: float = 0.55, omega_max: float = 1.5,
                 deadman_s: float = 0.5, reloj=time.monotonic):
        # Límites FÍSICOS reales del chasis (firmware). Unificar aquí evita los
        # tres topes distintos que había (backend 1.5/4.0, Nav2 0.4/1.0,
        # firmware 0.55/1.5): el firmware recorta a 0.55/1.5, así que ese es el
        # techo real y el que vale.
        self.v_max = float(v_max)
        self.omega_max = float(omega_max)
        self.deadman_s = float(deadman_s)
        self._reloj = reloj

        self.modo = Modo.DETENIDO
        self._emergencia = False
        self._propietario = None            # quién tiene el mando ahora
        self._ultimo_motivo = R_DETENIDO
        self._ultima_orden_t = None
        # Contadores para telemetría/depuración.
        self.rechazos = {R_EMERGENCIA: 0, R_SIN_MANDO: 0, R_CADUCA: 0, R_DETENIDO: 0}

    # ------------------------------------------------------------ propiedad
    @property
    def propietario(self) -> str | None:
        return self._propietario

    @property
    def emergencia_activa(self) -> bool:
        return self._emergencia

    def autonomia_permitida(self) -> bool:
        """Tras un manual o un rearme, la autonomía NO arranca sola: hay que
        pedir el modo explícitamente. Esto lo refleja el backend para no
        reanudar una patrulla/exploración por su cuenta."""
        return not self._emergencia and self.modo == Modo.DETENIDO

    # ------------------------------------------------------------ transiciones
    def tomar_manual(self, propietario: str = "manual") -> tuple[bool, str]:
        """El operador toma el mando. Arrebata la autonomía de forma
        determinista (el autónomo pierde el mando y sus órdenes se rechazan).
        No se permite durante emergencia (primero rearmar)."""
        if self._emergencia:
            return False, R_EMERGENCIA
        self.modo = Modo.MANUAL
        self._propietario = propietario
        return True, OK

    def soltar_manual(self) -> tuple[bool, str]:
        """Suelta el mando manual -> DETENIDO. La autonomía NO se reanuda sola."""
        if self.modo == Modo.MANUAL:
            self.modo = Modo.DETENIDO
            self._propietario = None
        return True, OK

    def pedir_modo(self, modo: Modo, propietario: str | None = None) -> tuple[bool, str]:
        """Pide pasar a un modo autónomo (o DETENIDO). Rechazado en emergencia.
        El manual NO se pide por aquí (usar tomar_manual), y un autónomo no
        puede arrebatarle el mando al manual."""
        if self._emergencia:
            return False, R_EMERGENCIA
        if modo == Modo.EMERGENCIA:
            self.emergencia()
            return True, OK
        if modo == Modo.DETENIDO:
            self.modo = Modo.DETENIDO
            self._propietario = None
            return True, OK
        if self.modo == Modo.MANUAL:
            # El operador tiene prioridad: un autónomo no lo desplaza.
            return False, R_SIN_MANDO
        if modo in AUTONOMOS:
            self.modo = modo
            self._propietario = propietario or PROPIETARIO_DE_MODO[modo]
            return True, OK
        return False, "modo_desconocido"

    def emergencia(self):
        """Enclava la emergencia. Idempotente. Resetea propietario y consigna:
        al rearmar no se restaura nada."""
        self._emergencia = True
        self.modo = Modo.EMERGENCIA
        self._propietario = None
        self._ultima_orden_t = None

    def rearmar(self, salud_ok: bool) -> tuple[bool, str]:
        """Sale de emergencia SOLO con salud adecuada. Arranca en DETENIDO,
        sin propietario ni consignas previas. Es el único camino de salida."""
        if not self._emergencia:
            return True, OK
        if not salud_ok:
            return False, "salud_insuficiente"
        self._emergencia = False
        self.modo = Modo.DETENIDO
        self._propietario = None
        self._ultima_orden_t = None
        return True, OK

    # ------------------------------------------------------------ puerta
    def _clamp(self, v: float, omega: float) -> tuple[float, float]:
        v = max(-self.v_max, min(self.v_max, float(v)))
        omega = max(-self.omega_max, min(self.omega_max, float(omega)))
        return v, omega

    def filtrar(self, propietario: str, v: float, omega: float,
                t: float | None = None) -> tuple[float, float, str]:
        """La protección final común: toda orden de movimiento pasa por aquí.

        Devuelve (v_permitida, omega_permitida, motivo). Cualquier rechazo da
        (0.0, 0.0, motivo). Orden:
          1. Emergencia enclavada -> 0.
          2. Quien manda no es este propietario -> 0 (cancela tareas colgadas).
          3. Orden caduca (más vieja que deadman) -> 0.
          4. Si no, recorta a límites y pasa.
        `t` es un reloj monótono de cuándo se GENERÓ la orden; si es None se usa
        el reloj actual (orden fresca)."""
        ahora = self._reloj()
        t = ahora if t is None else t

        if self._emergencia:
            self._rechazar(R_EMERGENCIA)
            return 0.0, 0.0, R_EMERGENCIA
        # Una orden de parar (0,0) siempre se acepta: nunca se rechaza frenar.
        parar = abs(v) < 1e-6 and abs(omega) < 1e-6
        if not parar and propietario != self._propietario:
            self._rechazar(R_SIN_MANDO)
            return 0.0, 0.0, R_SIN_MANDO
        if not parar and (ahora - t) > self.deadman_s:
            self._rechazar(R_CADUCA)
            return 0.0, 0.0, R_CADUCA
        self._ultima_orden_t = ahora
        self._ultimo_motivo = OK
        vc, wc = self._clamp(v, omega)
        return vc, wc, OK

    def proteccion_final(self, v: float, omega: float) -> tuple[float, float, str]:
        """PROTECCIÓN FINAL COMÚN, independiente de la propiedad del mando.

        Pensada para cablearse en el último punto por el que pasa TODA orden de
        movimiento (backend `_aplicar`), sin necesitar aún el plumbing de qué
        productor manda. Solo dos cosas, las que no dependen del dueño:
          · Emergencia enclavada -> (0, 0).
          · Recorte a los límites físicos unificados.
        La lógica de propiedad/caducidad/modos vive en `filtrar` y se cablea
        cuando cada productor registre su mando (paso de validación aparte)."""
        if self._emergencia:
            self._rechazar(R_EMERGENCIA)
            return 0.0, 0.0, R_EMERGENCIA
        vc, wc = self._clamp(v, omega)
        return vc, wc, OK

    def _rechazar(self, motivo: str):
        self._ultimo_motivo = motivo
        self.rechazos[motivo] = self.rechazos.get(motivo, 0) + 1

    # ------------------------------------------------------------ estado
    def estado(self) -> dict:
        return {
            "modo": self.modo.value,
            "propietario": self._propietario,
            "emergencia": self._emergencia,
            "autonomia_permitida": self.autonomia_permitida(),
            "ultimo_motivo": self._ultimo_motivo,
            "limites": {"v_max": self.v_max, "omega_max": self.omega_max,
                        "deadman_s": self.deadman_s},
            "rechazos": dict(self.rechazos),
        }
