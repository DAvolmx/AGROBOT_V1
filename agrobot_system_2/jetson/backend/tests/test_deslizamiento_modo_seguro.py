"""Pruebas del detector de patinaje y del modo seguro (sin robot, sin ROS).

    python3 -m pytest tests/test_deslizamiento_modo_seguro.py -q
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from deslizamiento import MonitorDeslizamiento, K_INICIAL       # noqa: E402
from modo_seguro import ModoSeguro, NORMAL, PRECAUCION, SEGURO   # noqa: E402


class Reloj:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def _andar(m, reloj, v_orden, v_medida, segundos, w_medida=0.0, paso=0.1):
    m.on_orden(v_orden, 0.0)
    for _ in range(int(segundos / paso)):
        reloj.t += paso
        m.on_medida(v_medida, w_medida)


def test_aprende_eficiencia_y_no_ve_patinaje_en_marcha_sana():
    r = Reloj()
    m = MonitorDeslizamiento(reloj=r)
    _andar(m, r, 0.4, 0.4 * 0.7, 40)          # el robot rinde el 70 %
    assert abs(m.k - 0.7) < 0.05
    assert m.patinaje < 0.1
    assert m.factor_velocidad() == 1.0


def test_patinaje_fuerte_recorta_velocidad_y_luego_se_recupera():
    r = Reloj()
    m = MonitorDeslizamiento(reloj=r)
    _andar(m, r, 0.4, 0.4 * 0.8, 30)          # sano, k ≈ 0.8
    _andar(m, r, 0.4, 0.4 * 0.8 * 0.2, 3)     # rinde el 20 % de lo normal
    assert m.patinaje > 0.7
    assert m.factor_velocidad() < 0.5
    k_antes = m.k
    _andar(m, r, 0.4, 0.4 * 0.8, 6)           # vuelve a agarrar
    assert abs(m.k - k_antes) < 0.05          # no aprendió del patinaje
    for _ in range(40):
        r.t += 0.1
        f = m.factor_velocidad()
    assert f == 1.0


def test_arranque_con_retraso_del_lidar_no_es_patinaje():
    r = Reloj()
    m = MonitorDeslizamiento(reloj=r)
    # rampa de 0 a 0.4 en 0.4 s; el LIDAR mide con 0.3 s de retraso
    ordenes = [min(0.4, 0.05 * i) for i in range(30)]
    for i, v in enumerate(ordenes):
        m.on_orden(v, 0.0)
        r.t += 0.1
        medida = ordenes[i - 3] * 0.8 if i >= 3 else 0.0
        m.on_medida(medida, 0.0)
        assert m.factor_velocidad() == 1.0, f"recortó en el paso {i}"


def test_aceleracion_lenta_con_retraso_no_es_patinaje():
    r = Reloj()
    m = MonitorDeslizamiento(reloj=r)
    # rampa suave, 2,5 s acelerando, LIDAR 0,3 s atrás
    ordenes = [min(0.5, 0.01 * i) for i in range(80)]
    for i, v in enumerate(ordenes):
        m.on_orden(v, 0.0)
        r.t += 0.05
        medida = ordenes[i - 6] * 0.8 if i >= 6 else 0.0
        m.on_medida(medida, 0.0)
        assert m.factor_velocidad() == 1.0, f"recortó en el paso {i} (patinaje {m.patinaje:.2f})"


def test_atasco_requiere_tiempo_y_se_suelta_al_moverse():
    r = Reloj()
    m = MonitorDeslizamiento(reloj=r)
    _andar(m, r, 0.3, 0.0, 1.0)
    assert not m.atascado                      # 1 s no basta
    _andar(m, r, 0.3, 0.0, 1.5)
    assert m.atascado
    _andar(m, r, 0.3, 0.25, 0.3)
    assert not m.atascado


def test_girar_en_el_lugar_no_es_atasco():
    r = Reloj()
    m = MonitorDeslizamiento(reloj=r)
    _andar(m, r, 0.3, 0.0, 3, w_medida=0.8)
    assert not m.atascado


def test_sin_orden_no_mide():
    r = Reloj()
    m = MonitorDeslizamiento(reloj=r)
    _andar(m, r, 0.0, 0.0, 5)
    assert m.k == K_INICIAL and m.patinaje == 0.0 and not m.atascado


def test_modo_seguro_niveles_y_enganche():
    r = Reloj()
    s = ModoSeguro(reloj=r)
    assert s.evaluar({"moviendose": True, "edad_lidar_s": 0.1})["nivel"] == NORMAL
    assert s.evaluar({"moviendose": True, "edad_lidar_s": 1.0})["nivel"] == PRECAUCION
    assert s.factor(manual=False) == 0.5
    est = s.evaluar({"moviendose": True, "edad_lidar_s": 3.0})
    assert est["nivel"] == SEGURO and est["enganchado"]
    assert s.factor(manual=False) == 0.0 and 0 < s.factor(manual=True) < 1
    # el LIDAR vuelve, pero sigue enganchado hasta rearmar
    assert s.evaluar({"moviendose": False, "edad_lidar_s": 0.1})["nivel"] == SEGURO
    ok, _ = s.rearmar()
    assert ok and s.nivel == NORMAL


def test_no_se_rearma_con_la_causa_presente():
    s = ModoSeguro(reloj=Reloj())
    s.evaluar({"autonomia": True, "atascado": True})
    ok, msg = s.rearmar()
    assert not ok and "atascado" in msg
    s.evaluar({"autonomia": True, "atascado": False})
    assert s.rearmar()[0]


def test_lidar_viejo_con_robot_quieto_y_sin_autonomia_no_molesta():
    s = ModoSeguro(reloj=Reloj())
    assert s.evaluar({"moviendose": False, "autonomia": False, "edad_lidar_s": 60})["nivel"] == NORMAL


def test_c3_y_ekf_solo_cuentan_en_autonomia():
    s = ModoSeguro(reloj=Reloj())
    assert s.evaluar({"autonomia": False, "edad_c3_s": 9, "ekf_var_vx": 5})["nivel"] == NORMAL
    assert s.evaluar({"autonomia": True, "edad_c3_s": 9})["nivel"] == SEGURO
