import asyncio
import math

import numpy as np
import pytest

from giro_cerrado import GiroCerrado


def _planta(w_dps_por_rps=60.0, tau=0.15, zona_muerta=0.0, retraso=0):
    """Planta mínima: la velocidad angular sigue a w con una constante de tiempo (inercia) y una zona muerta."""
    estado = {"rate": 0.0}

    def paso(w, dt):
        objetivo = 0.0 if abs(w) <= zona_muerta else w * w_dps_por_rps
        estado["rate"] += (objetivo - estado["rate"]) * (dt / tau)
        return estado["rate"]
    return estado, paso


def _correr(objetivo, anticipo=0.30, tau=0.15):
    g = GiroCerrado(objetivo, anticipo_s=anticipo)
    _, planta = _planta(tau=tau)
    yaw, w = 0.0, 0.0
    for _ in range(400):
        rate = planta(w, 0.05)
        yaw += rate * 0.05
        w = g.paso(rate, 0.05)
        if g.terminado:
            break
    for _ in range(40):                                  # inercia residual con el motor ya parado (y se observa)
        rate = planta(0.0, 0.05)
        yaw += rate * 0.05
        g.observar_tras_parar(rate, 0.05)
    return yaw, g


@pytest.mark.parametrize("objetivo", [15.0, 30.0, -20.0, -45.0])
def test_alcanza_el_angulo_pedido_con_la_inercia_bien_estimada(objetivo):
    yaw, g = _correr(objetivo, anticipo=0.15, tau=0.15)
    assert g.motivo == "alcanzado"
    assert abs(yaw - objetivo) <= 4.0


def test_aprende_la_inercia_real_aunque_parta_de_una_mala_estimacion():
    """Con anticipo 0,5 s y una planta de 0,15 s el primer giro se queda corto; tras unos giros la estimación converge y el error baja."""
    anticipo, errores = 0.5, []
    for _ in range(6):
        yaw, g = _correr(25.0, anticipo=anticipo, tau=0.15)
        errores.append(abs(yaw - 25.0))
        medido = g.anticipo_medido()
        if medido is not None:
            anticipo = 0.7 * anticipo + 0.3 * medido
    assert errores[0] > 4.0 and errores[-1] < 3.0 and abs(anticipo - 0.15) < 0.12


def test_sentido_del_giro():
    assert GiroCerrado(10.0).paso(0.0, 0.05) > 0
    assert GiroCerrado(-10.0).paso(0.0, 0.05) < 0


def test_pulso_muerto_sube_la_fuerza_y_respeta_el_tope():
    g = GiroCerrado(20.0, w_rps=0.4, w_max_rps=0.8, t_max_s=10.0)
    for _ in range(40):                                  # el robot no se mueve nada
        w = g.paso(0.0, 0.05)
    assert 0.4 < w <= 0.8


def test_tope_de_tiempo():
    g = GiroCerrado(90.0, t_max_s=0.5)
    for _ in range(20):
        g.paso(0.0, 0.05)
    assert g.terminado and g.motivo == "tiempo" and g.paso(0.0, 0.05) == 0.0


def test_objetivo_nulo_no_gira():
    g = GiroCerrado(0.2)
    assert g.paso(0.0, 0.05) == 0.0 and g.motivo == "nada que girar"


def test_el_conductor_gira_en_lazo_cerrado_y_para():
    from pulse_drive import ConductorPulsos

    async def prueba():
        enviados, paradas = [], []
        estado = {"rate": 0.0, "yaw": 0.0}

        def enviar(v, w, scan):
            enviados.append((v, w))
            estado["rate"] = w * 60.0

        conductor = ConductorPulsos(send_command=enviar, emergency_stop=lambda: (paradas.append(1), estado.update(rate=0.0)))
        conductor.anticipo_giro_s = 0.05                  # esta planta de prueba no tiene inercia

        def gyro():
            estado["yaw"] += estado["rate"] * 0.05
            return estado["rate"]
        assert conductor.girar_grados(20.0, gyro)
        assert conductor.girar_grados(5.0, gyro) is False        # ya hay uno en curso
        while conductor.activo:
            await asyncio.sleep(0.05)
        return enviados, paradas, conductor.ultimo_giro

    enviados, paradas, res = asyncio.run(prueba())
    assert res["motivo"] == "alcanzado" and 15.0 <= res["girado_deg"] <= 24.0
    assert all(v == 0.0 and w > 0 for v, w in enviados) and paradas


def test_sin_imu_no_se_mueve():
    from pulse_drive import ConductorPulsos

    async def prueba():
        enviados = []
        c = ConductorPulsos(send_command=lambda *a: enviados.append(a), emergency_stop=lambda: None)
        c.girar_grados(20.0, lambda: None)
        while c.activo:
            await asyncio.sleep(0.05)
        return enviados, c.ultimo_giro
    enviados, res = asyncio.run(prueba())
    assert enviados == [] and res["motivo"] == "sin IMU fresco"
