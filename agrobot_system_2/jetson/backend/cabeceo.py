"""
Cabeceo del chasis a partir del acelerómetro (MPU6050).
=======================================================

Para qué: el detector de senderos proyecta la imagen al suelo suponiendo que
la cámara mira siempre con la misma inclinación. En la huerta eso es falso —
el chasis cabecea en cada surco y en cada frenada—, y un error de 2° en la
inclinación desplaza el suelo estimado unos 5 cm a un metro. El sendero
"aparece" corrido y el robot corrige hacia donde no debe.

El MPU6050 ya está publicando a 20 Hz, así que el ángulo sale gratis: en
reposo el acelerómetro mide la gravedad, y la dirección de la gravedad ES la
inclinación. Lo que hay que cuidar es cuándo creerle.

Decisiones, y por qué:

  · **Giróscopo para lo rápido, acelerómetro para lo lento** (filtro
    complementario). Tentaba usar solo el acelerómetro y descartar las
    muestras cuya magnitud no fuera ~1 g, pero ese filtro no sirve: un frenazo
    de 0,4 g deja la magnitud en 1,08 g —dentro de cualquier ventana
    razonable— y sin embargo falsea el ángulo 21°. La aceleración propia del
    robot es casi toda longitudinal, justo el eje del que se saca el cabeceo.
    El giróscopo (gyroY) mide la velocidad de cabeceo directamente y no le
    afecta acelerar; a cambio, integrado, deriva. Se integra el giro y se
    corrige lentamente (2 % por muestra) con el ángulo del acelerómetro: la
    frenada pasa sin ensuciar el ángulo y la deriva no se acumula.
  · **El acelerómetro solo cuando mide ~1 g.** Fuera de esa ventana ni siquiera
    entra como corrección lenta, y el estimador queda un rato en giro puro.
  · **Y solo si no se contradice con el giróscopo** (gating por innovación).
    La ventana de magnitud no basta: ese frenazo de 0,4 g pasa el filtro y, a
    un 2 % por muestra durante un segundo, arrastra el ángulo 7°. Pero si el
    chasis de verdad hubiera cabeceado 20°, el giróscopo lo habría visto. Un
    salto grande del acelerómetro que el giro NO corrobora es aceleración
    propia, y se descarta. Lo que queda es la deriva lenta, que es justo para
    lo que está el acelerómetro.
  · **Cero por referencia, no por confianza.** El sensor no está perfectamente
    a nivel en el chasis. Se guarda el ángulo en reposo sobre suelo plano
    (`poner_cero`) y se informa la DIFERENCIA, así el montaje no importa.
  · **Tope de ±12°.** Más que eso en este robot es que el dato está mal (o que
    volcó). Un valor extremo colándose en la geometría de la cámara hace más
    daño que no corregir nada.

Signo: positivo = morro abajo, que es como lo usa la cámara (mira más cerca).
Si al levantar el morro el número sube en vez de bajar, el sensor está montado
al revés: se corrige con `calibration.imu_cabeceo_signo` = -1.
"""
import math

# Ventana de magnitud (en g) dentro de la cual el acelerómetro mide gravedad y
# no el zarandeo del chasis.
VENTANA_G = (0.85, 1.15)
# Peso del acelerómetro en cada muestra. Bajo a propósito: solo tiene que
# quitar la deriva del giróscopo (constante de tiempo ~2,5 s a 20 Hz), no
# seguir el cabeceo. Subirlo devuelve el problema de la frenada.
ALFA = 0.02
TOPE_DEG = 12.0
# Si entre muestras pasa más que esto, el enlace se cortó: integrar ese hueco
# con el último giro daría un salto enorme. Se reinicia con el acelerómetro.
HUECO_MAX_S = 0.5
# Diferencia máxima (grados) entre el ángulo del acelerómetro y el estimado
# para creerle. Por encima, el chasis tendría que haber girado y el giróscopo
# no lo vio: es aceleración longitudinal disfrazada de inclinación.
INNOVACION_MAX_DEG = 5.0


class CabeceoIMU:
    """Cabeceo filtrado, en grados, o `None` mientras no haya dato fiable."""

    def __init__(self, alfa=ALFA, tope_deg=TOPE_DEG, signo=1.0):
        self.alfa = float(alfa)
        self.tope = float(tope_deg)
        self.signo = 1.0 if float(signo) >= 0 else -1.0
        self.cero_deg = 0.0
        self._filtrado = None
        self._ultimo_crudo = None
        self._t_anterior = None
        self.descartadas = 0
        self.aceptadas = 0
        self.contradictorias = 0

    @property
    def grados(self):
        """Cabeceo respecto al cero, ya filtrado y acotado."""
        if self._filtrado is None:
            return None
        return max(-self.tope, min(self.tope, self._filtrado - self.cero_deg))

    def actualizar(self, ax, ay, az, giro_pitch_dps=0.0, t=None):
        """Una muestra del IMU: acelerómetro en g y cabeceo en °/s.

        `giro_pitch_dps` es la velocidad angular alrededor del eje lateral
        (pitch). Con el IMU montado con Y adelante y X a los lados, es gyroX.

        `t` es el instante (monotónico). Si no se da, se asume el periodo
        nominal de 20 Hz del nodo."""
        try:
            ax, ay, az = float(ax), float(ay), float(az)
            giro = self.signo * float(giro_pitch_dps or 0.0)
        except (TypeError, ValueError):
            return self.grados

        dt = 0.05
        if t is not None:
            if self._t_anterior is not None:
                dt = t - self._t_anterior
            self._t_anterior = t
        corte = not (0.0 < dt <= HUECO_MAX_S)

        magnitud = math.sqrt(ax * ax + ay * ay + az * az)
        fiable = VENTANA_G[0] <= magnitud <= VENTANA_G[1]
        crudo = None
        if fiable:
            # IMU montado con Y adelante: morro abajo inclina ay hacia la gravedad.
            crudo = self.signo * math.degrees(math.atan2(ay, az if az else 1e-6))
            self._ultimo_crudo = crudo
            self.aceptadas += 1
        else:
            self.descartadas += 1

        if self._filtrado is None or corte:
            # Sin estado previo (o tras un corte del enlace) solo hay el
            # acelerómetro; si tampoco es fiable, se espera.
            self._filtrado = crudo if crudo is not None else self._filtrado
            return self.grados

        # Predicción con el giróscopo...
        self._filtrado += giro * dt
        # ...y corrección lenta con la gravedad, si se la puede creer.
        if crudo is not None:
            innovacion = crudo - self._filtrado
            if abs(innovacion) <= INNOVACION_MAX_DEG:
                self._filtrado += self.alfa * innovacion
            else:
                self.contradictorias += 1
        return self.grados

    def poner_cero(self):
        """Fija el cero con el robot quieto y a nivel. Devuelve si pudo."""
        if self._filtrado is None:
            return False
        self.cero_deg = self._filtrado
        return True

    def resumen(self) -> dict:
        return {"cabeceo_deg": self.grados, "cero_deg": round(self.cero_deg, 2),
                "crudo_deg": None if self._ultimo_crudo is None else round(self._ultimo_crudo, 2),
                "aceptadas": self.aceptadas, "descartadas": self.descartadas,
                "contradictorias": self.contradictorias}
