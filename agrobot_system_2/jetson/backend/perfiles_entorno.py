"""Perfiles de entorno: los umbrales ESPACIALES de interior y de huerta.

Creado el 2026-10-14 para la defensa de tesis en interiores. `modo_manejo`
(pulse_drive.MODOS) ya cambiaba la duración de los pulsos, pero NO los umbrales
de espacio, y esos son los que deciden si el robot se mueve:

    avance_libre_cm = 100   espacio libre que exige DELANTE para lanzar un pulso
    margen_parada_cm + deslizamiento_cm = 12 + 40 = 52 cm   donde corta el avance

En una sala de 4 x 3 m con macetas eso deja el robot clavado: exige un metro
libre por delante para moverse. No es un fallo del control, es un perfil de
huerta aplicado a un sitio chico.

EL ACOPLAMIENTO QUE NO SE PUEDE ROMPER
--------------------------------------
El margen de parada existe porque al soltar el mando las ruedas quedan libres y
el robot SIGUE RODANDO (los frenos están desactivados, ver memoria del
proyecto). Ese recorrido depende muchísimo de la velocidad. Medido en el
simulador el 2026-10-14 (`dinamica_pasiva`, chasis rústico, mu 0,8 = suelo de
interior pulido):

    v pedida   recorrido tras soltar
    0,15 m/s        12,8 cm
    0,20 m/s        14,0 cm
    0,30 m/s        27,5 cm
    0,40 m/s        86,6 cm
    0,50 m/s       142,0 cm

Cuadra con lo medido en el robot real: el operador midió ~80 cm, que en esta
curva cae en ~0,4 m/s. Los 40 cm de la config corresponden a ~0,3 m/s.

Por eso bajar `deslizamiento_cm` SIN bajar la velocidad es quitarle al robot la
distancia de frenado: el perfil de interior trae las dos cosas juntas y
`validar()` se niega a aplicar una sin la otra. No separar estos dos campos.

ALCANCE: los recorridos salen del SIMULADOR, con `contrato_robot.json`
`calibrated: false`. Antes de confiar en el perfil de interior hay que repetir
la medida en el robot: mandar 0,2 m/s en el suelo de verdad, soltar y medir con
cinta. El procedimiento está en `LEEME_CAMPO.md`.
"""

#: Velocidad por encima de la cual el recorrido tras soltar se dispara (m/s).
#: De la curva medida: entre 0,2 y 0,3 el recorrido se duplica.
V_SEGURA_INTERIOR_MPS = 0.2

PERFILES = {
    "interior": {
        # --- acoplados: el deslizamiento vale SOLO a esta velocidad ---
        # 0,13 x control.max_v_mps (1,5) = 0,195 m/s, dentro del tramo donde
        # el recorrido tras soltar son ~14 cm. A 0,3 m/s ya son 27,5.
        "escala_velocidad": 0.13,
        "deslizamiento_cm": 20,       # medido 14 cm a 0,2 m/s, con margen
        "margen_parada_cm": 12,       # corte total: 32 cm
        # --- espacio que exige para maniobrar en un sitio chico ---
        "avance_libre_cm": 40,        # > el corte de 32, si no se corta a medio pulso
        "lado_libre_cm": 30,
        "retroceso_libre_cm": 30,
        "distancia_puntos_cm": 120,
        # Paredes cerca: en una sala los dos lados cuentan siempre.
        "sendero_pared_max_cm": 120,
        # 2026-10-22: 20° (antes 30) — el óptimo del entrenamiento calibrado (pe1_11) usa 20 y con él baja de 79 a 54 contactos.
        "cono_frontal_deg": 20,
        # 2026-10-22 (prueba real): en interiores el color del suelo y las paredes (beige/madera, H 10-45) cae en el rango de «paja»
        # y la cámara confundía el sendero. Dentro, el sendero sale solo del LIDAR.
        "sendero_vision": False,
        "sendero_paja": False,
        "sendero_metodo": "apariencia",        # si se enciende la visión dentro: el suelo por apariencia separa bien la baldosa
        "sendero_apariencia_umbral": 2.6,
        "sendero_apariencia_textura": 1.2,
    },
    "exterior": {
        "escala_velocidad": 0.5,
        "deslizamiento_cm": 40,
        "margen_parada_cm": 12,
        "avance_libre_cm": 100,
        "lado_libre_cm": 60,
        "retroceso_libre_cm": 30,
        "distancia_puntos_cm": 290,
        "sendero_pared_max_cm": 150,
        "cono_frontal_deg": 10,
        "sendero_vision": True,
        "sendero_paja": True,
        # 2026-10-22, videos del operador: el detector por contraste veía el sendero en el 38–46 % de los cuadros; por apariencia
        # (color + textura del suelo de delante) en el 82–85 %, y recorta bien un callejón de tierra con obstáculos.
        "sendero_metodo": "apariencia",
        "sendero_apariencia_umbral": 1.4,
        "sendero_apariencia_textura": 2.5,
    },
}

#: A qué sección de la config va cada clave.
SECCION = {
    "escala_velocidad": "autonomia", "avance_libre_cm": "autonomia",
    "lado_libre_cm": "autonomia", "retroceso_libre_cm": "autonomia",
    "distancia_puntos_cm": "autonomia", "sendero_pared_max_cm": "autonomia",
    "deslizamiento_cm": "slam", "margen_parada_cm": "slam",
    "cono_frontal_deg": "slam",
    "sendero_vision": "autonomia", "sendero_paja": "autonomia", "sendero_metodo": "autonomia",
    "sendero_apariencia_umbral": "autonomia", "sendero_apariencia_textura": "autonomia",
}


class PerfilIncoherente(ValueError):
    """El perfil pide una distancia de frenado que su velocidad no respalda."""


def validar(perfil: dict) -> None:
    """Comprueba el acoplamiento velocidad <-> deslizamiento <-> avance libre.

    Lanza `PerfilIncoherente` si:
      · se recorta el deslizamiento sin recortar la velocidad, o
      · se exige menos espacio libre delante que la propia distancia de corte
        (el robot arrancaría un pulso que el freno corta de inmediato).
    """
    desl = perfil["deslizamiento_cm"]
    corte = perfil["margen_parada_cm"] + desl
    if perfil["avance_libre_cm"] < corte:
        raise PerfilIncoherente(
            f"avance_libre_cm={perfil['avance_libre_cm']} es menor que el corte "
            f"de {corte} cm: el robot arrancaría y se frenaría solo")
    # El recorrido medido a 0,2 m/s es 14 cm; a 0,3 ya son 27,5, casi el doble.
    # Declarar menos de 25 cm solo es defendible dentro del tramo medido, así
    # que la velocidad del perfil no puede pasar de V_SEGURA_INTERIOR_MPS.
    # Sin holgura a propósito: ese 1,6x de antes dejaba colar 0,3 m/s.
    if desl < 25:
        v = perfil["escala_velocidad"] * 1.5      # control.max_v_mps
        if v > V_SEGURA_INTERIOR_MPS:
            raise PerfilIncoherente(
                f"deslizamiento_cm={desl} se midió a "
                f"{V_SEGURA_INTERIOR_MPS} m/s, pero el perfil permite "
                f"{v:.3f} m/s: a esa velocidad el recorrido tras soltar es "
                f"mayor y el margen de parada deja de cubrirlo")


def diferencias(config: dict, nombre: str) -> dict:
    """Qué cambiaría al aplicar el perfil, sin aplicarlo. {clave: (antes, después)}"""
    perfil = PERFILES[nombre]
    out = {}
    for clave, valor in perfil.items():
        actual = config.get(SECCION[clave], {}).get(clave)
        if actual != valor:
            out[clave] = (actual, valor)
    return out


def aplicar(config_store, config: dict, nombre: str) -> dict:
    """Aplica el perfil y devuelve las diferencias que se escribieron.

    Valida ANTES de escribir: un perfil incoherente no se aplica a medias.
    """
    if nombre not in PERFILES:
        raise KeyError(f"perfil desconocido: {nombre!r}")
    validar(PERFILES[nombre])
    difs = diferencias(config, nombre)
    por_seccion: dict[str, dict] = {}
    for clave, valor in PERFILES[nombre].items():
        por_seccion.setdefault(SECCION[clave], {})[clave] = valor
    for seccion, valores in por_seccion.items():
        config = config_store.update_section(seccion, valores)
    return difs
