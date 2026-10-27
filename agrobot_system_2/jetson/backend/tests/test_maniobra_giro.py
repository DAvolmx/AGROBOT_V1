import math
import random

import pytest

from maniobra_giro import ManiobraGiro


class Robot:
    """Robot de juguete: cada tramo gira lo pedido ±, y en arco avanza 5 cm por cada 6° (chasis de eje pasivo). Un pulso atrás/adelante mueve 12 cm."""

    def __init__(self, seed=0, factor=1.0, atascado=False, pared_frente=None, pared_atras=None):
        self.r = random.Random(seed)
        self.x = self.y = 0.0
        self.th = 0.0
        self.factor, self.atascado = factor, atascado
        self.pf, self.pa = pared_frente, pared_atras

    def huecos(self):
        return self.pf, self.pa

    def aplicar(self, o):
        if o["tipo"] == "girar":
            g = 0.0 if self.atascado else o["grados"] * self.factor * self.r.uniform(0.75, 1.1)
            self.th += g
            d = abs(g) / 6.0 * 0.05
            self.x += d * math.cos(math.radians(self.th))
            self.y += d * math.sin(math.radians(self.th))
        elif o["tipo"] in ("atras", "adelante"):
            s = -1.0 if o["tipo"] == "atras" else 1.0
            self.x += s * 0.12 * math.cos(math.radians(self.th))
            self.y += s * 0.12 * math.sin(math.radians(self.th))


def correr(g, robot, max_pasos=200):
    for _ in range(max_pasos):
        f, a = robot.huecos()
        o = g.siguiente(robot.th % 360.0, (robot.x, robot.y), f, a)
        if o["tipo"] == "fin":
            return o
        robot.aplicar(o)
    raise AssertionError("no termina")


@pytest.mark.parametrize("grados", [180, -180, 360, -360])
def test_completa_y_termina_cerca_del_inicio(grados):
    for seed in range(10):
        g, r = ManiobraGiro(grados), Robot(seed)
        fin = correr(g, r)
        assert fin["motivo"] == "completa"
        assert abs(abs(grados) - g.girado) <= g.tol + 1
        assert math.hypot(r.x, r.y) < 0.45           # sin compensar, 180° se iría ~0,75 m en este modelo


def test_compensar_reduce_la_deriva():
    sin = Robot(1)
    for _ in range(6):
        sin.aplicar({"tipo": "girar", "grados": 30.0})
    g, r = ManiobraGiro(180), Robot(1)
    correr(g, r)
    assert math.hypot(r.x, r.y) < math.hypot(sin.x, sin.y)
    assert g.atras >= 1


def test_atascado_se_detiene():
    g, r = ManiobraGiro(180), Robot(0, atascado=True)
    fin = correr(g, r)
    assert "atascado" in fin["motivo"]
    assert g.tramos == g.sin_avance_max


def test_sin_espacio_delante_ni_detras():
    g, r = ManiobraGiro(180), Robot(0, pared_frente=0.10, pared_atras=0.10)
    assert correr(g, r)["motivo"] == "sin espacio"
    assert g.tramos == 0


def test_pared_delante_retrocede_antes_de_girar():
    g = ManiobraGiro(180)
    assert g.siguiente(0.0, (0, 0), 0.10, None)["tipo"] == "atras"


def test_pared_detras_no_retrocede():
    g = ManiobraGiro(180)
    o = g.siguiente(0.0, (0, 0), None, 0.10)
    assert o["tipo"] == "adelante"


def test_vuelta_de_atras_en_el_rumbo_se_normaliza():
    g = ManiobraGiro(180)
    g.siguiente(350.0, (0, 0), None, None)
    g.siguiente(15.0, (0, 0), None, None)                # 350 -> 15 son +25°, no -335°
    assert 20 < g.girado < 30


def test_grados_invalidos():
    with pytest.raises(ValueError):
        ManiobraGiro(0)
    with pytest.raises(ValueError):
        ManiobraGiro(1000)
