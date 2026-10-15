"""Pruebas de la detección de vuelco/inclinación del modo seguro (sin robot).

    python3 -m pytest tests/test_vuelco_modo_seguro.py -q

Contexto: el 2026-10-09 el robot volcó con el firmware no adaptado al chasis
nuevo y la seguridad no lo vio (todo "normal"). Estas pruebas fijan que ahora
sí lo detecta y que no salta por una pendiente normal.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from modo_seguro import (ModoSeguro, NORMAL, PRECAUCION, SEGURO,   # noqa: E402
                         inclinacion_desde_acc,
                         INCLINACION_PRECAUCION_DEG, INCLINACION_SEGURO_DEG,
                         ROLL_RATE_VUELCO_DPS)


def _acc(inclinacion_deg):
    """Acelerómetro en g (reposo) para una inclinación dada desde la vertical."""
    r = math.radians(inclinacion_deg)
    return (math.sin(r), 0.0, math.cos(r))


class Reloj:
    def __init__(self): self.t = 0.0
    def __call__(self): return self.t


def test_inclinacion_desde_acc_derecho_y_tumbado():
    assert inclinacion_desde_acc((0.0, 0.0, 1.0)) < 1          # derecho
    assert abs(inclinacion_desde_acc((1.0, 0.0, 0.0)) - 90) < 1  # de lado
    assert inclinacion_desde_acc((0.0, 0.0, -1.0)) > 179       # boca abajo


def test_descarta_muestra_con_aceleracion_fuerte():
    # Vector lejos de 1 g: no es gravedad pura, no se puede leer inclinación.
    assert inclinacion_desde_acc((0.0, 0.0, 2.5)) is None
    assert inclinacion_desde_acc((0.0, 0.0, 0.2)) is None


def test_robot_derecho_sigue_normal():
    m = ModoSeguro(reloj=Reloj())
    est = m.evaluar({"moviendose": True, "acc": _acc(2.0)})
    assert est["nivel"] == NORMAL


def test_pendiente_moderada_solo_precaucion_no_engancha():
    # Un talud inclina de forma sostenida pero sin balanceo: recorta, no para.
    m = ModoSeguro(reloj=Reloj())
    est = m.evaluar({"moviendose": True,
                     "acc": _acc(INCLINACION_PRECAUCION_DEG + 3),
                     "giro_roll_dps": 2.0})
    assert est["nivel"] == PRECAUCION
    assert not est["enganchado"]
    assert m.factor(manual=False) == 0.5


def test_inclinacion_extrema_para_y_engancha():
    m = ModoSeguro(reloj=Reloj())
    est = m.evaluar({"moviendose": True, "acc": _acc(INCLINACION_SEGURO_DEG + 5)})
    assert est["nivel"] == SEGURO
    assert est["enganchado"]
    assert any("VUELCO" in x for x in est["motivos"])
    assert m.factor(manual=False) == 0.0


def test_vuelco_rapido_para_antes_del_angulo_seguro():
    # Inclinación aún en zona de precaución PERO balanceándose rápido: es un
    # vuelco en curso, hay que cortar el par ya, no esperar a 38°.
    m = ModoSeguro(reloj=Reloj())
    est = m.evaluar({"moviendose": True,
                     "acc": _acc(INCLINACION_PRECAUCION_DEG + 4),
                     "giro_roll_dps": ROLL_RATE_VUELCO_DPS + 20})
    assert est["nivel"] == SEGURO
    assert est["enganchado"]


def test_rearme_solo_cuando_se_endereza():
    m = ModoSeguro(reloj=Reloj())
    m.evaluar({"moviendose": True, "acc": _acc(INCLINACION_SEGURO_DEG + 5)})
    ok, _ = m.rearmar()
    assert not ok                                   # sigue volcado
    m.evaluar({"moviendose": True, "acc": _acc(2.0)})  # ya derecho
    ok, _ = m.rearmar()
    assert ok
    assert m.nivel == NORMAL


def test_sin_imu_no_dispara():
    m = ModoSeguro(reloj=Reloj())
    est = m.evaluar({"moviendose": True})           # sin acc ni inclinacion
    assert est["nivel"] == NORMAL


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
