"""Pulsos manuales finitos para el pivote pasivo trasero.

Base: la reescritura de Codex del 2026-10-15 (sin cola ni giro sobre el eje,
clamps de `manual_demo`, sin girar marcha atrás). Se le devolvió el 2026-10-15
lo que esa reescritura había perdido:

  1. EL TROCEADO POR DUTY (`_tramo`). Es el ÚNICO modo de ir más despacio que
     el suelo de PWM del hardware: los motores no bajan del 78 % de potencia,
     así que pedir media velocidad casi no frena. Lo que sí funciona es
     alternar empuje y rueda libre varias veces por segundo (`suavidad_duty`
     del ciclo con velocidad, el resto a cero): el robot avanza a tirones
     cortos, mucho más lento y controlable, sin tocar el firmware.

     Por qué importa que esté: el perfil de interior (`perfiles_entorno`) baja
     `deslizamiento_cm` a 20 PORQUE la velocidad queda en 0,195 m/s. Medido en
     simulador, al soltar el mando el robot sigue rodando 14 cm a 0,2 m/s pero
     86 cm a 0,4. Sin el troceado la velocidad real no baja, el margen de
     parada de 32 cm deja de cubrir el deslizamiento, y el perfil de interior
     es inseguro. `torque_optimizer` sigue ajustando `suavidad_duty` en
     caliente, y sin este bucle quedaba escribiendo un campo que nadie leía.

  2. LAS DURACIONES POR MODO. La reescritura dejó los tres modos idénticos
     (0,3 s), con lo que `modo_manejo` no hacía nada.

SIGUE PERDIDO respecto a la versión anterior al 2026-10-15 (no se restauró, no
se pidió): la `maniobra` de giro en tierra blanda (arco alterno adelante/atrás
que rompe el agarre del barro; `segmentos` ignora el parámetro `maniobra`) y el
encadenado de pulsos al repetir dirección.
"""
import asyncio
import time

from manual_demo import V_SCALE, W_SCALE

TICK_S = .05
ACCIONES = ("adelante", "atras", "izquierda", "derecha")

# Duraciones por modo. `vel_factor` escala la velocidad pedida ANTES de los
# clamps de manual_demo.
MODOS = {
    "interior": {"pulso_recto_s": .8, "pulso_giro_s": .4, "vel_factor": 1.0},
    # Pulsos CORTOS (2026-10-02, pedido del operador): la mitad que interior.
    # Con el firmware nuevo un pulso recto de 0,8 s desviaba el rumbo 8-16° a
    # la derecha; con la mitad se mueve ~12 cm y se corrige antes de quedar
    # cruzado en un pasillo.
    "preciso": {"pulso_recto_s": .4, "pulso_giro_s": .25, "vel_factor": 1.0},
    "exterior": {"pulso_recto_s": 1.0, "pulso_giro_s": .35, "vel_factor": 1.0},
}


class ConductorPulsos:
    def __init__(self, *, send_command, emergency_stop, get_config=None):
        self._send_command = send_command
        self._emergency_stop = emergency_stop
        # Codex aceptaba get_config y lo descartaba; sin él no hay troceado.
        self._get_config = get_config or (lambda: {"suavidad_duty": 1.0,
                                                   "suavidad_periodo_s": 0.3})
        self._tarea = None
        self.ultimo_giro = None
        self.anticipo_giro_s = 0.30          # inercia del giro: se APRENDE de cada giro medido (EMA)
        self.modo = "preciso"

    @property
    def activo(self):
        return self._tarea is not None and not self._tarea.done()

    def girar_grados(self, grados: float, get_gyro_dps, *, w_rps: float = 0.6, anticipo_s: float | None = None, t_max_s: float = 2.0) -> bool:
        """Gira `grados` (+ = izquierda) en LAZO CERRADO con el giroscopio (giro_cerrado.py). No bloquea: crea la tarea como `ordenar`.
        `get_gyro_dps()` -> °/s o None si no hay IMU fresco (entonces no se mueve). False si ya hay un movimiento en curso."""
        if self.activo:
            return False
        self.ultimo_giro = None
        self._tarea = asyncio.create_task(self._girar(float(grados), get_gyro_dps, w_rps, anticipo_s, t_max_s))
        return True

    async def _girar(self, grados, get_gyro_dps, w_rps, anticipo_s, t_max_s):
        from giro_cerrado import GiroCerrado
        g = GiroCerrado(grados, w_rps=w_rps, anticipo_s=self.anticipo_giro_s if anticipo_s is None else anticipo_s, t_max_s=t_max_s)
        previo = time.monotonic()
        try:
            while True:
                await asyncio.sleep(TICK_S)
                ahora = time.monotonic()
                dt, previo = ahora - previo, ahora
                gz = get_gyro_dps()
                if gz is None:
                    g._fin("sin IMU fresco")
                    break
                w = g.paso(gz, dt)
                if g.terminado:
                    break
                self._send_command(0.0, w, 0)
            # motor parado: se sigue midiendo 0,5 s para conocer el ángulo FINAL y aprender la inercia
            self._emergency_stop()
            fin_obs = time.monotonic() + 0.5
            previo = time.monotonic()
            while time.monotonic() < fin_obs:
                await asyncio.sleep(TICK_S)
                ahora = time.monotonic()
                gz = get_gyro_dps()
                if gz is not None:
                    g.observar_tras_parar(gz, ahora - previo)
                previo = ahora
            medido = g.anticipo_medido()
            if medido is not None:
                self.anticipo_giro_s = min(0.6, max(0.05, 0.7 * self.anticipo_giro_s + 0.3 * medido))
        finally:
            self.ultimo_giro = g.resumen()
            if asyncio.current_task() is self._tarea:
                self._emergency_stop()
                self._tarea = None

    def _modo(self):
        return MODOS.get(self.modo, MODOS["preciso"])

    def segmentos(self, accion, v_max, omega_max, maniobra=True):
        """(v, omega, duración) de UNA pulsación. omega > 0 = izquierda.

        Marcha atrás sin giro a propósito (decisión de Codex, se conserva):
        con el pivote pasivo, retroceder girando puede cruzar el robot.
        """
        if accion not in ACCIONES:
            raise ValueError('accion desconocida')
        m = self._modo()
        vf = m["vel_factor"]
        v = max(0, min(v_max * vf, .60 * V_SCALE))
        w = max(0, min(omega_max, W_SCALE))
        if accion == 'atras':
            return [(-min(v, .35 * V_SCALE), 0., m["pulso_recto_s"])]
        if accion == 'adelante':
            return [(v, 0., m["pulso_recto_s"])]
        sentido = 1. if accion == 'izquierda' else -1.
        return [(v, sentido * w, m["pulso_giro_s"])]

    def ordenar(self, accion, v_max, omega_max, maniobra=True, esc_dur=1.0):
        segmentos = self.segmentos(accion, v_max, omega_max, maniobra)
        if esc_dur != 1.0:                    # pulsos de corrección de rumbo más finos (rumbo_pulso_escala)
            segmentos = [(v, w, d * max(0.1, min(1.0, esc_dur))) for (v, w, d) in segmentos]
        if self.activo:
            return False
        self._tarea = asyncio.create_task(self._ejecutar(segmentos))
        return True

    def parar(self):
        if self._tarea is not None:
            self._tarea.cancel()
        self._tarea = None
        self._emergency_stop()

    async def _tramo(self, v, w, dur):
        """Un tramo TROCEADO, para poder ir más lento que el mínimo del
        hardware. Con duty >= 0,99 manda continuo (comportamiento de Codex)."""
        cfg = self._get_config() or {}
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

    async def _ejecutar(self, segmentos):
        try:
            for v, w, dur in segmentos:
                await self._tramo(v, w, dur)
        finally:
            # Se conserva el cierre de Codex: al acabar o al cancelar, parada.
            if asyncio.current_task() is self._tarea:
                self._emergency_stop()
                self._tarea = None
