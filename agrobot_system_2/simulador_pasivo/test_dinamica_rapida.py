"""El núcleo compilado (numba) debe dar lo MISMO que la versión de Python de dinamica_pasiva (tolerancia numérica)."""
import copy
import math

import numpy as np
import pytest

import dinamica_pasiva as D
import dinamica_rapida as R

pytestmark = pytest.mark.skipif(not R.DISPONIBLE, reason="numba no está instalado")


def _robot(seed):
    rng = np.random.default_rng(seed)
    p = D.Parametros(mu=float(rng.choice([.4, .65, .95])), lateral=float(rng.choice([8., 30., 65.])), rolling_drag=float(rng.choice([.03, .17])),
                     front_drive_bias=float(rng.choice([.35, .7, 2.0])), drive=str(rng.choice(["all", "front", "rear"])),
                     toe_ruedas=tuple(rng.normal(0, .02, 4)), gain_ruedas=tuple(1 + rng.normal(0, .08, 4)), trail=float(rng.choice([0., .05])))
    return D.DinamicaPasiva(pose=(0., 0., float(rng.uniform(-3, 3))), parametros=p)


@pytest.mark.parametrize("seed", range(8))
def test_trayectoria_igual_a_la_de_python(seed):
    a, b = _robot(seed), _robot(seed)
    b.forzar_python = True
    rng = np.random.default_rng(100 + seed)
    for k in range(400):                                    # mandos aleatorios, con ruedas libres y objetivos por rueda, golpes al límite del pivote
        wt = rng.uniform(-.55, .55, 4) if k % 3 else None
        lb = rng.random(4) < .3 if k % 5 == 0 else None
        lin, ang = float(rng.uniform(-.5, .5)), float(rng.uniform(-1.5, 1.5))
        a.step(0.01, lin, ang, wheel_targets=wt, libres=lb)
        b.step(0.01, lin, ang, wheel_targets=wt, libres=lb)
    assert np.allclose(a.q, b.q, atol=1e-7, rtol=1e-7)
    assert np.allclose(a.dq, b.dq, atol=1e-6, rtol=1e-6)
    assert np.allclose(a.wheels, b.wheels, atol=1e-6, rtol=1e-6)
    assert a.stop_impacts == b.stop_impacts
    assert math.isclose(a.slip, b.slip, rel_tol=1e-6, abs_tol=1e-8)
