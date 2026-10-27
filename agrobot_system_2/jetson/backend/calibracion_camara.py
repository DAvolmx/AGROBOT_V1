"""Calibración ÚNICA y COMPARTIDA del servo de tilt de la cámara.

Resuelve la contradicción detectada el 2026-10-10: el simulador (robot.py) y el
backend (barrido_tilt.py) usaban el SIGNO OPUESTO del servo. Esto es la fuente
de verdad; ambos la importan para que no vuelvan a divergir.

CONVENCIÓN CANÓNICA (la del modelo establecido robot.py): un ángulo de servo
MENOR inclina la cámara MÁS hacia abajo.

    inclinacion_abajo(servo) = INCLINACION_HOME_DEG + GRADOS_POR_GRADO*(HOME - servo)

En HOME (98°) la cámara mira ~15,5° bajo el horizonte; a 76° mira ~33° abajo
(suelo cercano); a 110° mira ~5,9° abajo (más al frente).

IMPORTANTE — estado de la calibración:
Todos estos números son ESTIMACIONES documentadas en el chasis/MANIFEST, NO
mediciones. `VERIFICADO = False`: queda PENDIENTE la comprobación física —
medir el cabeceo real de la cámara con un nivel o el IMU a servo 25/98/110 y
confirmar (a) el SIGNO, (b) la ganancia 0,8°/° y (c) el 15,5° de HOME. Hasta
entonces, trátese como hipótesis configurable, no como dato medido.
"""

SERVO_MIN_DEG = 25.0
SERVO_MAX_DEG = 110.0
SERVO_HOME_DEG = 98.0

INCLINACION_HOME_DEG = 15.5      # estimación (no medido). MEDIDO el 2026-10-27 con cajas de altura/distancia conocidas: en HOME (98°) la cámara mira ~1,9° bajo el
                                # horizonte (agrobot_config.json: odometria_webcam.camara_inclinacion_deg), no 15,5°. Esta constante la siguen usando los simuladores viejos.
GRADOS_POR_GRADO = 0.8          # medido 2026-10-27 (barrido 98→62°, caja a 1,30 m y maceta a 0,95 m): 0,78°/° y SIGNO correcto (servo menor = más abajo); desvío ≤ ±1,1°
VERIFICADO = False              # <-- pendiente de comprobación física


def inclinacion_abajo(servo):
    """Grados que la cámara mira hacia ABAJO para un ángulo de servo dado."""
    return INCLINACION_HOME_DEG + GRADOS_POR_GRADO * (SERVO_HOME_DEG - servo)


def delta_inclinacion(servo):
    """Cambio de inclinación respecto a HOME (para corregir la geometría de la
    cámara cuando el servo no está en HOME). + = más abajo que en HOME."""
    return GRADOS_POR_GRADO * (SERVO_HOME_DEG - servo)
