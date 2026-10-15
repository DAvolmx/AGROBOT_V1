"""Geometría ÚNICA y COMPARTIDA de la envolvente del robot.

Creado el 2026-10-14 al auditar el corredor visual. `sendero.py` aceptaba
senderos de 20 cm de ancho (`ANCHO_MIN_M = 0.20`) y `slam.robot_ancho_cm`
valía 35: las dos por debajo de los 37 cm REALES del eje trasero. El efecto
es el que se ve en campo: el detector se engancha a un hueco por el que pasa
el morro (19 cm) y no el trasero, el robot entra y el eje de atrás pisa la
hilera.

MEDIDAS CONFIRMADAS por el usuario (son medidas, no estimaciones). Son anchos
EXTERIORES del chasis, no separación entre centros de rueda:

    distancia entre ejes      0,37 m
    ancho exterior delantero  0,37 m   <-- fijo; la envolvente la manda ESTE
    ancho exterior trasero    0,19 m   (módulo con el pivote pasivo)

Las TRASERAS forman un módulo con pivote pasivo limitado a ~±20°
mecánicos. Al girar, ese pivote hace que la huella barra MÁS que la
envolvente quieta; eso NO está modelado aquí todavía (ver PENDIENTE).

PENDIENTE de medir, no lo inventes aquí:
  · barrido real de la huella en giro con el pivote al tope.
  · ángulo del pivote (no hay sensor).
  · masa, inercias, fricción, respuesta PWM  ->  contrato_robot.json.
"""

ENTRE_EJES_M = 0.37
# GEOMETRÍA INVERTIDA el 2026-10-15 (el operador movió el pivote atrás):
# ahora el eje ANCHO (37 cm) es el DELANTERO y fijo, y el módulo ESTRECHO
# (19 cm) con el pivote pasivo (±20°) va DETRÁS. Hasta el 2026-10-14 era al
# revés. La envolvente no cambia (manda el más ancho, 37 cm), pero ahora es
# el eje DELANTERO el que entra primero en un hueco, y lo que barre al girar
# es el módulo trasero.
ANCHO_DELANTERO_M = 0.37
ANCHO_TRASERO_M = 0.19

#: Lo más ancho del robot: lo que tiene que caber en el corredor.
ANCHO_ENVOLVENTE_M = max(ANCHO_DELANTERO_M, ANCHO_TRASERO_M)

# --- Holgura lateral: MEDIDA, no supuesta -------------------------------
# Es holgura de SEGUIMIENTO (el robot no va perfectamente centrado ni recto),
# no de fabricación.
#
# El 2026-10-14 este valor era 0,06 m puesto a ojo, y el simulador lo
# desmintió. `simulador_pasivo/calibrar_holgura.py` barre el ancho del pasillo
# con el chasis rústico, 8 semillas, y busca dónde dejan de haber choques.
#
# ESTO NO ES UNA PROPIEDAD DEL CHASIS: es el pico de desvío del CONTROLADOR.
# Con las ganancias viejas (kp3,5 ki2,0 v0,5) y con las que el ajuste aceptó
# ese mismo día (kp4,0 ki2,5 v0,4):
#
#     pasillo  holgura   % episodios con choque
#                        viejo        ajustado
#       45 cm    4,0 cm    88 %          88 %
#       49 cm    6,0 cm    88 %          62 %   <-- el umbral viejo
#       53 cm    8,0 cm     --           38 %
#       57 cm   10,0 cm    50 %          12 %
#       65 cm   14,0 cm    25 %           0 %   <-- ajustado, limpio
#       75 cm   19,0 cm     0 %           0 %   <-- viejo, limpio
#
#     RMSE lateral:  6,0 cm (viejo)  ->  3,6 cm (ajustado)
#
# Por qué el RMSE no basta para fijarla: era 6,0 cm y CONSTANTE a todos los
# anchos, pero el PICO de desvío es 15,8 cm (pico de rumbo 7,5°, que sobre un
# cuerpo de 35 cm añade ~2,3 cm). 15,8 + 2,3 ≈ 18 cm explica que con el
# control viejo los choques solo desaparecieran a 19 cm. La huella NO se
# ensancha al girar —el semiancho es 18,5 cm con el pivote a 0° y a 20°, manda
# la vía trasera— así que el barrido en giro queda descartado como causa.
#
# OJO con el alcance: sale del SIMULADOR, con `contrato_robot.json`
# `calibrated: false` (masa, fricción e inercias son estimaciones). Es una cota
# de por dónde NO ir, no una medida del robot. Al mejorar el control, se vuelve
# a medir y ESTO baja: es lo que pasó de 19 a 14 cm.
HOLGURA_LATERAL_M = 0.14

#: De dónde sale la holgura: el pico de desvío lateral, no el RMSE.
PICO_DESVIO_LATERAL_M = 0.158     # con el control viejo
RMSE_DESVIO_LATERAL_M = 0.036     # con el control ajustado
PICO_RUMBO_DEG = 7.5
HOLGURA_PROCEDENCIA = ("simulador_pasivo/calibrar_holgura.py, 8 semillas, "
                       "kp4.0/ki2.5/v0.4, 2026-10-14; simulador sin calibrar")

#: Mínimo que hay que medir para declarar un sendero transitable.
ANCHO_CORREDOR_MIN_M = ANCHO_ENVOLVENTE_M + 2 * HOLGURA_LATERAL_M


def cabe(ancho_m: float, holgura_m: float = HOLGURA_LATERAL_M) -> bool:
    """¿Cabe la envolvente completa del robot en un corredor de ese ancho?

    Se compara contra el eje TRASERO (el ancho), no contra el delantero: por
    un hueco de 25 cm pasa el morro y se queda clavado el trasero.
    """
    return ancho_m >= ANCHO_ENVOLVENTE_M + 2 * max(0.0, holgura_m)


def margen(ancho_m: float) -> float:
    """Holgura que sobra a cada lado en un corredor de ese ancho (m).

    Negativa = no cabe. Sirve para bajar la velocidad cuando el sendero se
    estrecha, en vez de decidir solo sí/no.
    """
    return (ancho_m - ANCHO_ENVOLVENTE_M) / 2.0
