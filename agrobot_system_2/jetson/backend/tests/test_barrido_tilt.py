"""Pruebas del barrido de tilt y el freno proactivo (sin robot, sin cámara).

    python3 -m pytest tests/test_barrido_tilt.py -q
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from barrido_tilt import (BarridoTilt, FrenoProactivo, velocidad_segura,  # noqa: E402
                          inclinacion_por_servo, SERVO_LEJOS_DEG, SERVO_CERCA_DEG,
                          SERVO_HOME_DEG, ALTURA_ASUMIDA_M,
                          altura_por_paralaje, EstimadorAltura)


def _aproximar(estimador, altura_real, h_cam=0.4, x_obs=1.5, pasos=8, dx=0.07):
    """Alimenta el estimador simulando acercarse a un punto de altura dada."""
    factor = 1.0 - altura_real / h_cam           # d_app = x_real / factor
    for i in range(pasos):
        recorrido = i * dx
        x_real = x_obs - recorrido
        estimador.alimentar(recorrido, x_real / factor, t=i * 0.3)


def test_envolvente_decrece_con_la_altura():
    assert velocidad_segura(0.02) > velocidad_segura(0.05) > velocidad_segura(0.08)
    assert velocidad_segura(0.09) == 0.0          # vuelca hasta parado
    assert velocidad_segura(0.0) >= 1.5


def test_inclinacion_por_servo():
    assert inclinacion_por_servo(SERVO_HOME_DEG) == 0.0
    assert inclinacion_por_servo(SERVO_CERCA_DEG) > 0     # más abajo
    assert inclinacion_por_servo(SERVO_LEJOS_DEG) < 0     # más arriba/lejos


def test_barrido_alterna_lejos_y_cerca():
    b = BarridoTilt()
    angulos = {b.angulo(t * 0.1, v=0.6) for t in range(60)}
    assert SERVO_LEJOS_DEG in angulos and SERVO_CERCA_DEG in angulos


def test_barrido_mira_mas_lejos_a_mas_velocidad():
    b = BarridoTilt()
    dl_lento, _ = b._dwells(0.2)
    dl_rapido, _ = b._dwells(1.2)
    assert dl_rapido > dl_lento                   # más v -> más tiempo lejos


def test_freno_sin_obstaculo_no_limita():
    f = FrenoProactivo()
    assert f.tope_velocidad(None, 0.75) == 0.75


def test_freno_limita_a_la_velocidad_segura():
    f = FrenoProactivo()
    # Montículo de 5 cm visto a 1 m: seguro ~0.41 m/s, y hay sitio para frenar.
    tope = f.tope_velocidad(1.0, 0.75, altura_m=0.05)
    assert abs(tope - velocidad_segura(0.05)) < 1e-6


def test_freno_para_ante_monticulo_que_vuelca_siempre():
    f = FrenoProactivo()
    assert f.tope_velocidad(1.0, 0.75, altura_m=0.10) == 0.0   # vuelca hasta parado


def test_freno_para_si_esta_muy_cerca():
    f = FrenoProactivo()
    assert f.tope_velocidad(0.10, 0.75, altura_m=0.05) == 0.0


def test_sin_altura_asume_peor_caso_conservador():
    f = FrenoProactivo()
    tope = f.tope_velocidad(1.0, 0.75)            # sin altura -> ALTURA_ASUMIDA
    assert abs(tope - velocidad_segura(ALTURA_ASUMIDA_M)) < 1e-6


def test_paralaje_dos_puntos():
    # Suelo: la distancia aparente cae justo el baseline -> altura 0.
    assert abs(altura_por_paralaje(1.50, 1.40, 0.10, 0.4)) < 1e-6
    # Montículo: la distancia aparente cae MÁS que el baseline -> altura > 0.
    h = altura_por_paralaje(1.50, 1.35, 0.10, 0.4)
    assert 0.0 < h < 0.4
    # Hoyo: cae MENOS que el baseline -> altura < 0.
    assert altura_por_paralaje(1.50, 1.45, 0.10, 0.4) < 0.0
    # No se acerca (misma distancia) -> sin información.
    assert altura_por_paralaje(1.5, 1.5, 0.1, 0.4) is None


def test_estimador_suelo_da_cero():
    e = EstimadorAltura(h_cam=0.4)
    _aproximar(e, altura_real=0.0)
    assert abs(e.altura()) < 0.01


def test_estimador_monticulo_recupera_la_altura():
    e = EstimadorAltura(h_cam=0.4)
    _aproximar(e, altura_real=0.05)
    assert abs(e.altura() - 0.05) < 0.015


def test_estimador_detecta_hoyo():
    e = EstimadorAltura(h_cam=0.4)
    _aproximar(e, altura_real=-0.04)
    assert e.altura() < 0.0


def test_estimador_sin_muestras_suficientes():
    e = EstimadorAltura(h_cam=0.4)
    e.alimentar(0.0, 1.5, 0.0)
    assert e.altura() is None


def test_estimador_resetea_si_el_obstaculo_salta():
    e = EstimadorAltura(h_cam=0.4)
    _aproximar(e, altura_real=0.05, pasos=5)
    e.alimentar(0.5, 3.0, t=2.0)        # aparece otro obstáculo lejano
    assert len(e.muestras) == 1


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
