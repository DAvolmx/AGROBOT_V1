"""Troceado por duty y duraciones por modo en los pulsos manuales.

Estos tests NO existían, y por eso la reescritura de `pulse_drive.py` del
2026-10-15 pudo borrar el troceado sin que ninguna de las 98 pruebas fallara.

Por qué el troceado es crítico: los motores no bajan del 78 % de potencia, así
que pedir media velocidad casi no frena. Lo único que frena de verdad es
alternar empuje y rueda libre varias veces por segundo. El perfil de interior
(`perfiles_entorno`) baja `deslizamiento_cm` a 20 PORQUE la velocidad real
queda en 0,195 m/s; sin troceado la velocidad real no baja y el margen de
parada deja de cubrir el deslizamiento.
"""
import asyncio
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pulse_drive


def _conductor(duty=1.0, periodo=0.3):
    enviados, paradas = [], []
    c = pulse_drive.ConductorPulsos(
        send_command=lambda v, w, s: enviados.append((v, w, s)),
        emergency_stop=lambda: paradas.append(True),
        get_config=lambda: {"suavidad_duty": duty, "suavidad_periodo_s": periodo})
    return c, enviados, paradas


# --- Troceado ------------------------------------------------------------

def test_con_duty_bajo_se_alternan_empuje_y_rueda_libre():
    c, enviados, _ = _conductor(duty=0.5, periodo=0.2)
    asyncio.run(c._tramo(0.2, 0.0, 0.6))
    assert enviados, "no mandó nada"
    con_empuje = [e for e in enviados if e[0] != 0.0]
    en_libre = [e for e in enviados if e[0] == 0.0]
    # La clave: TIENE que haber ticks a cero intercalados.
    assert con_empuje, "nunca empujó"
    assert en_libre, "nunca soltó: el troceado no está funcionando"


def test_con_duty_alto_manda_continuo():
    c, enviados, _ = _conductor(duty=1.0)
    asyncio.run(c._tramo(0.2, 0.0, 0.3))
    assert enviados
    assert all(e[0] == 0.2 for e in enviados), "con duty 1 no debe trocear"


def test_duty_mas_bajo_deja_mas_tiempo_en_rueda_libre():
    fracciones = {}
    for duty in (0.3, 0.8):
        c, enviados, _ = _conductor(duty=duty, periodo=0.2)
        asyncio.run(c._tramo(0.2, 0.0, 0.8))
        libres = sum(1 for e in enviados if e[0] == 0.0)
        fracciones[duty] = libres / len(enviados)
    assert fracciones[0.3] > fracciones[0.8]


def test_el_duty_se_recorta_a_un_minimo_razonable():
    """Un duty de 0 dejaría el robot sin empujar nunca."""
    c, enviados, _ = _conductor(duty=0.0, periodo=0.2)
    asyncio.run(c._tramo(0.2, 0.0, 0.6))
    assert any(e[0] != 0.0 for e in enviados), "con duty 0 debe seguir empujando algo"


def test_sin_get_config_no_revienta():
    enviados = []
    c = pulse_drive.ConductorPulsos(send_command=lambda v, w, s: enviados.append(v),
                                    emergency_stop=lambda: None)
    asyncio.run(c._tramo(0.2, 0.0, 0.15))
    assert enviados


def test_el_conductor_usa_el_troceado_al_ordenar():
    """Regresión directa: `_ejecutar` tiene que pasar por `_tramo`."""
    c, enviados, _ = _conductor(duty=0.4, periodo=0.2)

    async def correr():
        c.modo = "interior"
        c.ordenar("adelante", 0.195, 1.5)
        await asyncio.sleep(0.9)

    asyncio.run(correr())
    assert any(e[0] == 0.0 for e in enviados), \
        "ordenar() no trocea: la velocidad real no bajará del suelo de PWM"


# --- Duraciones por modo -------------------------------------------------

def test_los_modos_tienen_duraciones_DISTINTAS():
    """Si los tres son iguales, `modo_manejo` no hace nada."""
    rectos = {n: m["pulso_recto_s"] for n, m in pulse_drive.MODOS.items()}
    assert len(set(rectos.values())) == len(rectos), rectos


def test_preciso_pulsa_mas_corto_que_interior_y_exterior():
    m = pulse_drive.MODOS
    assert m["preciso"]["pulso_recto_s"] < m["interior"]["pulso_recto_s"]
    assert m["preciso"]["pulso_recto_s"] < m["exterior"]["pulso_recto_s"]


def test_el_modo_cambia_la_duracion_del_pulso():
    c, _, _ = _conductor()
    duraciones = {}
    for modo in pulse_drive.MODOS:
        c.modo = modo
        duraciones[modo] = c.segmentos("adelante", 0.195, 1.5)[0][2]
    assert duraciones["preciso"] < duraciones["exterior"]


def test_un_modo_desconocido_cae_en_preciso_no_revienta():
    c, _, _ = _conductor()
    c.modo = "submarino"
    assert c.segmentos("adelante", 0.195, 1.5)[0][2] == \
        pulse_drive.MODOS["preciso"]["pulso_recto_s"]


# --- Lo que Codex añadió y se conserva ----------------------------------

def test_marcha_atras_sin_giro():
    """Con el pivote pasivo, retroceder girando puede cruzar el robot."""
    c, _, _ = _conductor()
    v, w, _dur = c.segmentos("atras", 0.5, 1.5)[0]
    assert v < 0
    assert w == 0.0


def test_una_accion_desconocida_se_rechaza():
    c, _, _ = _conductor()
    with pytest.raises(ValueError):
        c.segmentos("volar", 0.5, 1.5)


def test_la_velocidad_queda_acotada_por_los_clamps():
    c, _, _ = _conductor()
    v, _w, _d = c.segmentos("adelante", 99.0, 1.5)[0]
    assert v <= 0.60 * pulse_drive.V_SCALE


def test_parar_corta_y_manda_la_parada():
    c, _, paradas = _conductor()
    c.parar()
    assert paradas, "parar() debe llamar al corte de emergencia"
