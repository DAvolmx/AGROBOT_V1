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

PULSO_RECTO_S = 1.0       # adelante / atrás
PULSO_GIRO_S = 0.5        # giro directo: 1 s girando son demasiados grados
MAX_ENCOLADO_S = 3.0      # tope de pulsos encadenados

# Maniobra de giro para tierra blanda: arco adelante + arco atrás.
MANIOBRA_V = 0.5          # fracción de la velocidad lineal máxima
MANIOBRA_OMEGA = 1.0      # fracción de la angular máxima
MANIOBRA_MEDIO_S = 0.6    # duración de cada arco

ACCIONES = ("adelante", "atras", "izquierda", "derecha")


class ConductorPulsos:
    def __init__(self, *, send_command, emergency_stop):
        self._send_command = send_command    # (v, omega, scan)
        self._emergency_stop = emergency_stop
        self._cola: deque[tuple[float, float, float]] = deque()
        self._accion_actual: str | None = None
        self._tarea: asyncio.Task | None = None

    @property
    def activo(self) -> bool:
        return self._tarea is not None and not self._tarea.done()

    def segmentos(self, accion: str, v_max: float, omega_max: float,
                  maniobra: bool) -> list[tuple[float, float, float]]:
        """(v, omega, duración) de UNA pulsación. omega > 0 = izquierda."""
        if accion == "adelante":
            return [(v_max, 0.0, PULSO_RECTO_S)]
        if accion == "atras":
            return [(-v_max, 0.0, PULSO_RECTO_S)]
        sentido = 1.0 if accion == "izquierda" else -1.0
        if not maniobra:
            return [(0.0, sentido * omega_max, PULSO_GIRO_S)]
        w = sentido * MANIOBRA_OMEGA * omega_max
        v = MANIOBRA_V * v_max
        return [(v, w, MANIOBRA_MEDIO_S), (-v, w, MANIOBRA_MEDIO_S)]

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

    def _cancelar_tarea(self):
        if self._tarea is not None and not self._tarea.done():
            self._tarea.cancel()
        self._tarea = None

    async def _ejecutar(self):
        try:
            while self._cola:
                v, w, dur = self._cola.popleft()
                fin = time.monotonic() + dur
                while time.monotonic() < fin:
                    self._send_command(v, w, 0)
                    await asyncio.sleep(TICK_S)
            # Objetivo a cero: el `tick()` del actuador baja la rampa hasta
            # parar del todo, sin tirón.
            self._send_command(0.0, 0.0, 0)
        finally:
            # Una tarea cancelada por un reemplazo corre este `finally` DESPUÉS
            # de que `ordenar` fijó la acción nueva: no debe borrarla.
            if asyncio.current_task() is self._tarea:
                self._accion_actual = None
