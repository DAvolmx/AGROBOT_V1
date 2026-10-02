"""
Manejo manual por pulsos.
========================

Cada pulsación del operador es un movimiento de duración fija que ejecuta la
Jetson, no un "mantener apretado". Tres motivos:

  · En el móvil, mantener apretado no funcionaba: el navegador cancela el
    toque (`pointercancel`) en cuanto el dedo se mueve un píxel o reconoce
    una pulsación larga, y el robot recibía un arranque seguido de una
    parada a los pocos milisegundos. Un toque, en cambio, llega siempre.
  · Si el WiFi pierde el "suelto el botón", con mantener apretado el robot
    sigue andando. Con pulsos el final lo decide la Jetson, que no depende
    de la red para parar.
  · Precisión: un pulso es una distancia repetible. "Adelante, adelante,
    adelante" son tres pasos iguales.

Pulsar la misma dirección con un pulso en curso lo ENCADENA (se suma otro
pulso, con tope); pulsar otra dirección lo REEMPLAZA en el acto.

Giros en tierra blanda: un skid-steer que gira sobre sí mismo en suelo
blando entierra las ruedas y no rota. La maniobra alterna un arco hacia
delante y otro hacia atrás con el MISMO sentido de giro: las traslaciones
se cancelan, la rotación se suma, y el movimiento lineal rompe el agarre del
barro. Es la versión ordenada de lo que el operador hacía a mano
(izquierda, derecha, adelante, atrás) hasta que el robot giraba.
"""
import asyncio
import time
from collections import deque

TICK_S = 0.05             # 20 Hz, el mismo ritmo al que late el enlace UART
MAX_ENCOLADO_S = 3.0      # tope de pulsos encadenados

ACCIONES = ("adelante", "atras", "izquierda", "derecha")

MODOS = {
    "interior": {
        "pulso_recto_s": 0.8,
        "pulso_giro_s": 0.4,
        "maniobra_v": 0.0,
        "maniobra_omega": 1.0,
        "maniobra_medio_s": 0.4,
        "vel_factor": 0.6,
    },
    # Pulsos CORTOS (2026-10-02, pedido del usuario): la mitad de largos
    # que en interior. Con el firmware nuevo cada pulso recto de 0,8 s
    # desviaba el rumbo 8-16° a la derecha; con la mitad de pulso se mueve
    # ~12 cm y se puede corregir antes de quedar cruzado en un pasillo.
    "preciso": {
        "pulso_recto_s": 0.4,
        "pulso_giro_s": 0.25,
        "maniobra_v": 0.0,
        "maniobra_omega": 1.0,
        "maniobra_medio_s": 0.25,
        "vel_factor": 0.6,
    },
    "exterior": {
        "pulso_recto_s": 1.0,
        "pulso_giro_s": 0.35,
        "maniobra_v": 0.25,
        "maniobra_omega": 1.0,
        "maniobra_medio_s": 0.3,
        "vel_factor": 1.0,
    },
}


class ConductorPulsos:
    def __init__(self, *, send_command, emergency_stop, get_config=None):
        self._send_command = send_command    # (v, omega, scan)
        self._emergency_stop = emergency_stop
        self._get_config = get_config or (lambda: {"suavidad_duty": 1.0,
                                                   "suavidad_periodo_s": 0.3})
        self._cola: deque[tuple[float, float, float]] = deque()
        self._accion_actual: str | None = None
        self._tarea: asyncio.Task | None = None
        self.modo = "exterior"

    @property
    def activo(self) -> bool:
        return self._tarea is not None and not self._tarea.done()

    def segmentos(self, accion: str, v_max: float, omega_max: float,
                  maniobra: bool) -> list[tuple[float, float, float]]:
        """(v, omega, duración) de UNA pulsación. omega > 0 = izquierda."""
        m = MODOS.get(self.modo, MODOS["exterior"])
        vf = m["vel_factor"]
        if accion == "adelante":
            return [(v_max * vf, 0.0, m["pulso_recto_s"])]
        if accion == "atras":
            return [(-v_max * vf, 0.0, m["pulso_recto_s"])]
        sentido = 1.0 if accion == "izquierda" else -1.0
        if not maniobra:
            return [(0.0, sentido * omega_max, m["pulso_giro_s"])]
        w = sentido * m["maniobra_omega"] * omega_max
        v = m["maniobra_v"] * v_max * vf
        return [(v, w, m["maniobra_medio_s"]), (-v, w, m["maniobra_medio_s"])]

    def ordenar(self, accion: str, v_max: float, omega_max: float,
                maniobra: bool = True):
        nuevos = self.segmentos(accion, v_max, omega_max, maniobra)

        if accion != self._accion_actual or not self.activo:
            # Otra dirección: se descarta lo pendiente y se arranca ya. La
            # rampa del actuador suaviza el cambio de sentido.
            self._cancelar_tarea()
            self._cola.clear()
            self._accion_actual = accion
            self._cola.extend(nuevos)
            self._tarea = asyncio.create_task(self._ejecutar())
            return

        # Misma dirección con un pulso en curso: se encadena, con tope, para
        # que aporrear el botón no deje al robot andando medio minuto.
        pendiente = sum(d for _, _, d in self._cola)
        if pendiente + sum(d for _, _, d in nuevos) <= MAX_ENCOLADO_S:
            self._cola.extend(nuevos)

    def parar(self):
        self._cancelar_tarea()
        self._cola.clear()
        self._accion_actual = None
        self._emergency_stop()

    async def _tramo(self, v, w, dur):
        """Ejecuta un tramo TROCEADO, para poder ir más lento que el mínimo
        del hardware.

        Los motores no bajan del 78 % de potencia: pedir media velocidad no
        frena casi nada. Lo que sí funciona es alternar empuje y rueda libre
        varias veces por segundo (`suavidad_duty` del ciclo con velocidad, el
        resto a cero). El robot avanza a tirones cortos pero mucho más lento
        y más controlable, sin tocar el firmware.
        """
        cfg = self._get_config()
        duty = min(1.0, max(0.15, float(cfg.get("suavidad_duty", 1.0))))
        periodo = max(0.1, float(cfg.get("suavidad_periodo_s", 0.3)))
        fin = time.monotonic() + dur
        while time.monotonic() < fin:
            if duty >= 0.99:
                self._send_command(v, w, 0)
                await asyncio.sleep(TICK_S)
                continue
            fin_on = min(fin, time.monotonic() + periodo * duty)
            while time.monotonic() < fin_on:
                self._send_command(v, w, 0)
                await asyncio.sleep(TICK_S)
            fin_off = min(fin, time.monotonic() + periodo * (1.0 - duty))
            while time.monotonic() < fin_off:
                self._send_command(0.0, 0.0, 0)
                await asyncio.sleep(TICK_S)

    def _cancelar_tarea(self):
        if self._tarea is not None and not self._tarea.done():
            self._tarea.cancel()
        self._tarea = None

    async def _ejecutar(self):
        try:
            while self._cola:
                v, w, dur = self._cola.popleft()
                await self._tramo(v, w, dur)
            # Objetivo a cero: el `tick()` del actuador baja la rampa hasta
            # parar del todo, sin tirón.
            self._send_command(0.0, 0.0, 0)
        finally:
            # Una tarea cancelada por un reemplazo corre este `finally` DESPUÉS
            # de que `ordenar` fijó la acción nueva: no debe borrarla.
            if asyncio.current_task() is self._tarea:
                self._accion_actual = None
