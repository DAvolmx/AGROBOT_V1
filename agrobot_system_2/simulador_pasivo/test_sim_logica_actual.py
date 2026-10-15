"""Validación del simulador de la lógica actual (sim_logica_actual.py).

No prueban el robot: prueban que el SIMULADOR hace lo que dice que hace. Cada uno cubre un
error real que se cometió o estuvo a punto de cometerse el 2026-10-15.
"""
import math
import shutil
import subprocess
from dataclasses import replace

import pytest

import sim_logica_actual as S


def _planta(mu=0.8, drag=0.02, seed=0):
    return replace(S.parametros_rusticos(seed), mu=mu, rolling_drag=drag, front_drive_bias=1.0, substep=0.01)


# --- mezcla del firmware ---------------------------------------------------
def test_mezcla_recta_y_giro():
    cfg = S.Config()
    assert S.fw_mix(0.55, 0, cfg) == [60.0] * 4
    r1, l1, l2, r2 = S.fw_mix(0.33, 1.5, cfg)                 # izquierda: lado izq mas lento
    assert l1 < r1 and l2 < r2 and l1 == l2 and r1 == r2


def test_giro_sin_avance_no_mueve_ruedas():
    assert S.fw_mix(0.0, 1.5, S.Config()) == [0.0] * 4


def test_nunca_contrarrotacion():
    for v in (-0.5, -0.1, 0.1, 0.5):
        for w in (-1.5, 0, 1.5):
            m = S.fw_mix(v, w, S.Config(fw_minp=30))
            assert all(a * b >= 0 for a in m for b in m)


@pytest.mark.skipif(not shutil.which("g++"), reason="sin g++")
def test_mezcla_python_igual_a_la_del_firmware_cpp(tmp_path):
    fuente = tmp_path / "m.cpp"
    fuente.write_text('#include <cstdio>\n#include "control_model.h"\n'
                      'int main(){double v,w,f,r,i,m;while(scanf("%lf %lf %lf %lf %lf %lf",&v,&w,&f,&r,&i,&m)==6){'
                      'agro::Limits L;L.fwd=f;L.rev=r;L.inner=i*.01f;L.minp=m;agro::Mix x=agro::mix((float)v,(float)w,.55f,L);'
                      'printf("%.4f %.4f %.4f %.4f\\n",x.duty[0],x.duty[1],x.duty[2],x.duty[3]);}}')
    fw = S.AQUI.parent / "firmware" / "agrobot_c3_demo_trasero_1_2"
    if not (fw / "control_model.h").exists():
        pytest.skip("sin firmware 1.2")
    subprocess.run(["g++", "-std=c++17", f"-I{fw}", str(fuente), "-o", str(tmp_path / "m")], check=True)
    casos = [(0.4, 1.2, 60, 35, 65, 0), (-0.3, 0.5, 60, 35, 65, 0), (0.1, 0, 60, 35, 65, 40),
             (0.55, -1.5, 80, 35, 90, 30), (0.2, 1.5, 100, 50, 80, 45)]
    ent = "\n".join(" ".join(map(str, c)) for c in casos)
    sal = subprocess.run([str(tmp_path / "m")], input=ent, capture_output=True, text=True, check=True).stdout.split("\n")
    for c, l in zip(casos, sal):
        py = S.fw_mix(c[0], c[1], S.Config(fw_fwd=c[2], fw_rev=c[3], fw_inner=c[4], fw_minp=c[5]))
        assert max(abs(a - float(b)) for a, b in zip(py, l.split())) < 0.01


# --- rampa por rueda del firmware -----------------------------------------
def test_la_rampa_limita_lo_que_empuja_un_pulso_corto():
    """Con 60 %/s un pulso de 0,4 s no puede pasar de 24 % de duty. Es aritmetica pura."""
    fw = S.MotoresFirmware(S.Config(fw_slew=60))
    fw.set_targets([60.0] * 4, 0.0)
    t, m = 0.0, 0.0
    while t < 0.4:
        m = max(m, max(abs(x) for x in fw.tick(t)))
        fw.set_targets([60.0] * 4, t)
        t += 0.01
    assert m <= 24.01


def test_con_rampa_alta_llega_al_duty_pedido():
    fw = S.MotoresFirmware(S.Config(fw_slew=300))
    t, m = 0.0, 0.0
    while t < 0.4:
        fw.set_targets([60.0] * 4, t)
        m = max(m, max(abs(x) for x in fw.tick(t)))
        t += 0.01
    assert m == pytest.approx(60.0, abs=0.01)


def test_duty_cero_es_rueda_libre_instantanea():
    fw = S.MotoresFirmware(S.Config())
    for i in range(300):
        fw.set_targets([60.0] * 4, i * 0.01)
        fw.tick(i * 0.01)
    fw.set_targets([0.0] * 4, 3.0)
    assert fw.tick(3.0) == [0.0] * 4


def test_el_deadman_del_firmware_corta():
    fw = S.MotoresFirmware(S.Config(fw_slew=300))
    for i in range(100):
        fw.set_targets([60.0] * 4, i * 0.01)
        fw.tick(i * 0.01)
    assert max(abs(x) for x in fw.tick(1.0 + 0.6)) == 0.0      # sin ordenes 0,6 s


# --- rueda libre en la dinamica --------------------------------------------
def test_rueda_libre_rueda_mas_que_rueda_frenada():
    """El error del 2026-10-15: duty 0 se simulaba como objetivo de velocidad 0, que FRENA."""
    def recorrido(libres):
        r = S.DinamicaPasiva(pose=(0, 0, 0), parametros=_planta(drag=0.003))
        for _ in range(300):
            r.step(0.01, 0, 0, wheel_targets=[-0.3] * 4)
        x0 = r.q[0]
        for _ in range(300):
            r.step(0.01, 0, 0, wheel_targets=[0.0] * 4, libres=libres)
        return abs(r.q[0] - x0)
    assert recorrido([True] * 4) > 3 * recorrido(None)


def test_sin_libres_el_comportamiento_anterior_no_cambia():
    a = S.DinamicaPasiva(pose=(0, 0, 0), parametros=_planta())
    b = S.DinamicaPasiva(pose=(0, 0, 0), parametros=_planta())
    for _ in range(50):
        a.step(0.01, 0, 0, wheel_targets=[0.2] * 4)
        b.step(0.01, 0, 0, wheel_targets=[0.2] * 4, libres=None)
    assert list(a.q) == list(b.q)


def test_el_modelo_girado_avanza_hacia_delante_real():
    r = S.RobotModelo(_planta(), (0, 0, 0))
    for _ in range(200):
        r.step(0.01, [100] * 4, 0.0)
    assert r.centro()[0] > 0.02           # duty positivo = adelante real
    assert abs(math.degrees(r.psi_real())) < 5


def test_el_centro_del_robot_coincide_con_la_pose_dada():
    r = S.RobotModelo(_planta(), (1.0, 0.5, math.radians(30)))
    cx, cy = r.centro()
    assert (cx, cy) == pytest.approx((1.0, 0.5), abs=1e-9)
    assert math.degrees(r.psi_real()) == pytest.approx(30, abs=1e-6)


# --- conductor y rampa de la Jetson ---------------------------------------
def test_un_pulso_en_curso_descarta_la_orden_nueva():
    cfg = S.Config()
    act = S.ActuadorJetson(cfg)
    c = S.Conductor(cfg, act)
    assert c.ordenar("adelante", 0.0) is True
    assert c.ordenar("izquierda", 0.1) is False
    assert c.descartadas == 1


def test_el_troceado_intercala_ceros():
    cfg = S.Config(suavidad_duty=0.6)
    c = S.Conductor(cfg, S.ActuadorJetson(cfg))
    cmds = []
    t = 0.0
    c.ordenar("adelante", t)
    while c.activo and t < 2:
        t += 0.05
        antes = (c.act.obj_v, c.act.obj_w)
        c.paso(t)
        cmds.append(c.act.obj_v)
    assert any(v == 0 for v in cmds) and any(v > 0 for v in cmds)


def test_el_primer_comando_tras_reposo_salta_a_plena_velocidad():
    """_apply_ramp limita dt a 0,15 s: tras un reposo la rampa no suaviza el arranque."""
    cfg = S.Config()
    a = S.ActuadorJetson(cfg)
    a.emergency_stop(0.0)
    a.send_command(0.33, 0.0, 5.0)
    assert a.ramp_v == pytest.approx(0.33)


def test_modos_de_pulso_vienen_del_pulse_drive_real():
    for modo, m in S.pulse_drive.MODOS.items():
        cfg = S.Config(modo=modo)
        c = S.Conductor(cfg, S.ActuadorJetson(cfg))
        seg = c._real.segmentos("adelante", 0.75, 2.0, False)[0]
        assert seg[2] == m["pulso_recto_s"]


# --- objetivo ---------------------------------------------------------------
def test_el_objetivo_v2_no_premia_la_inmovilidad():
    parado = {"exito": False, "contactos": 0, "progreso": 0.0, "t_s": 100}
    roza = {"exito": False, "contactos": 2, "progreso": 1.0, "t_s": 40}
    llega = {"exito": True, "contactos": 0, "progreso": 1.0, "t_s": 60}
    assert S.puntuar_v2(llega) < S.puntuar_v2(roza) and S.puntuar_v2(llega) < S.puntuar_v2(parado)


def test_tope_60_es_identico_al_manual_demo_real():
    from manual_demo import reference
    for v, w in ((0.1, 0.5), (0.33, 1.5), (0.6, 0.0), (-0.2, 1.0), (0.0, 1.0)):
        assert S.demo_ref_tope(v, w, 60.0) == pytest.approx(reference(v, w))


# --- escenarios -------------------------------------------------------------
def test_el_pasillo_deja_el_hueco_pedido():
    esc = S.escenarios_interior(0.60)["pasillo_macetas"]
    izq = min(y for (_, y, _) in esc.macetas if y > 0)
    der = max(y for (_, y, _) in esc.macetas if y < 0)
    assert (izq - S.R_MACETA) - (der + S.R_MACETA) == pytest.approx(0.60)


def test_el_fondo_esta_lejos_de_la_meta():
    for nombre in ("sala_vacia", "pasillo_macetas", "eslalon_macetas"):
        esc = S.escenarios_interior()[nombre]
        fondo = max(x for w in esc.paredes for x in (w[0], w[2]))
        assert fondo - esc.goal_x > 1.5
