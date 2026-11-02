import math

from vigia_pose import ANCLADA, OK, PERDIDA, VigiaPose


def _p(x, y, th=0.0):
    return {"x": x, "y": y, "theta_deg": th}


def _recorrido(v, n, *, ancla=(0.0, 0.0), dt=0.1, vel=0.2, t0=0.0, x0=0.0, salto_mapa=None, salto_odom=None, var=None):
    """El robot avanza en línea recta por x a `vel`. El mapa = odom + ancla; salto_* = {tick: (dx, dy)} se suma desde ese tick."""
    out = []
    dm = [0.0, 0.0]
    do = [0.0, 0.0]
    for k in range(n):
        if salto_mapa and k in salto_mapa:
            dm[0] += salto_mapa[k][0]
            dm[1] += salto_mapa[k][1]
        if salto_odom and k in salto_odom:
            do[0] += salto_odom[k][0]
            do[1] += salto_odom[k][1]
        ox = x0 + vel * k * dt + do[0]
        oy = do[1]
        mapa = _p(ox + ancla[0] + dm[0], oy + ancla[1] + dm[1])
        out.append(v.paso(t0 + k * dt, mapa, _p(ox, oy), var))
    return out


def test_sigue_al_slam_sin_saltos():
    v = VigiaPose()
    r = _recorrido(v, 50, ancla=(1.0, 2.0))
    assert r[-1]["estado"] == OK
    assert abs(r[-1]["pose"]["x"] - (1.0 + 0.2 * 4.9)) < 1e-6
    assert abs(r[-1]["pose"]["y"] - 2.0) < 1e-6


def test_correccion_normal_del_slam_se_acepta():
    """Un cierre de bucle de 30 cm (medido en interior) no es un salto."""
    v = VigiaPose()
    r = _recorrido(v, 40, salto_mapa={20: (0.30, 0.0)})
    assert r[-1]["estado"] == OK
    assert abs(r[-1]["pose"]["x"] - (0.2 * 3.9 + 0.30)) < 1e-6


def test_salto_grande_del_slam_conserva_la_pose():
    """27-oct: la pose saltó 30 m. La pose servida sigue continua (odometría) y el estado es «anclada»."""
    v = VigiaPose()
    r = _recorrido(v, 40, salto_mapa={20: (30.0, 0.0)})
    assert r[-1]["estado"] == ANCLADA
    xs = [e["pose"]["x"] for e in r]
    assert max(b - a for a, b in zip(xs, xs[1:])) < 0.05
    assert abs(xs[-1] - 0.2 * 3.9) < 1e-6
    assert v.cuenta["saltos_slam"] == 1


def test_vuelve_a_ok_si_el_slam_regresa():
    v = VigiaPose()
    r = _recorrido(v, 60, salto_mapa={20: (30.0, 0.0), 30: (-30.0, 0.0)})
    assert r[-1]["estado"] == OK
    assert v.cuenta["resync"] == 1


def test_acepta_una_reubicacion_pequena_y_estable():
    """Un salto de 0,7 m que el SLAM MANTIENE (se reubicó de verdad) se acepta tras `aceptar_s` quieto."""
    v = VigiaPose(aceptar_s=3.0)
    r = _recorrido(v, 80, salto_mapa={20: (0.7, 0.0)})
    assert r[25]["estado"] == ANCLADA
    assert r[-1]["estado"] == OK
    assert v.cuenta["reubicadas"] == 1
    assert abs(r[-1]["pose"]["x"] - (0.2 * 7.9 + 0.7)) < 1e-6


def test_no_acepta_un_salto_enorme_aunque_sea_estable():
    v = VigiaPose(aceptar_s=1.0)
    r = _recorrido(v, 80, salto_mapa={20: (30.0, 0.0)})
    assert r[-1]["estado"] == ANCLADA


def test_salto_de_odometria_se_absorbe():
    """Pico del EKF (vx −29,7 m/s): la odometría salta 3 m en 0,1 s y el mapa con ella. La pose servida no se mueve."""
    v = VigiaPose()
    r = _recorrido(v, 40, salto_odom={20: (-3.0, 0.0)}, salto_mapa={20: (0.0, 0.0)})
    xs = [e["pose"]["x"] for e in r]
    assert max(abs(b - a) for a, b in zip(xs, xs[1:])) < 0.05
    assert v.cuenta["saltos_odom"] == 1
    assert r[21]["estado"] == ANCLADA


def test_anclado_demasiados_metros_es_perdida():
    v = VigiaPose(max_anclada_m=1.0)
    r = _recorrido(v, 100, salto_mapa={10: (30.0, 0.0)})
    assert r[-1]["estado"] == PERDIDA
    assert not v.autonomia_permitida()
    assert v.cuenta["perdidas"] == 1


def test_perdida_se_recupera_sola_si_vuelve_el_slam():
    v = VigiaPose(max_anclada_m=1.0)
    r = _recorrido(v, 120, salto_mapa={10: (30.0, 0.0), 100: (-30.0, 0.0)})
    assert r[90]["estado"] == PERDIDA
    assert r[-1]["estado"] == OK


def test_ekf_sin_medidas_es_perdida():
    v = VigiaPose(ekf_var_s=0.5)
    r = _recorrido(v, 20, var=5.0)
    assert r[-1]["estado"] == PERDIDA
    assert "EKF" in r[-1]["motivo"]


def test_tras_salto_de_reloj_baja_el_umbral():
    v = VigiaPose()
    _recorrido(v, 10)
    v.sospechar(0.9, 15.0)
    r = _recorrido(v, 20, t0=1.0, x0=0.2 * 10, salto_mapa={5: (0.30, 0.0)})
    assert r[-1]["estado"] == ANCLADA


def test_sin_mapa_sirve_odometria():
    v = VigiaPose()
    _recorrido(v, 10, ancla=(1.0, 0.0))
    r = v.paso(1.0, None, _p(0.2, 0.0))
    assert abs(r["pose"]["x"] - 1.2) < 1e-6


def test_sin_odometria_es_perdida():
    v = VigiaPose()
    _recorrido(v, 10)
    r = v.paso(5.0, _p(1.0, 0.0), None)
    assert r["estado"] == PERDIDA


def test_reubicar_por_el_operador():
    v = VigiaPose(max_anclada_m=0.5)
    _recorrido(v, 60, salto_mapa={10: (30.0, 0.0)})
    assert v.estado == PERDIDA
    assert v.reubicar(6.0, _p(5.0, 5.0, 90.0))
    assert v.estado == OK
    r = v.paso(6.1, None, _p(0.02 * 59 + 0.02, 0.0))
    assert abs(r["pose"]["y"] - 5.02) < 1e-6         # +2 cm de odometría en x -> +y en el mapa (girado 90°)


def test_giro_con_ancla_rotada():
    """Ancla con rotación: la composición respeta el ángulo."""
    v = VigiaPose()
    for k in range(10):
        o = _p(0.1 * k, 0.0, 0.0)
        m = _p(1.0, 1.0 + 0.1 * k, 90.0)
        r = v.paso(0.1 * k, m, o)
    assert r["estado"] == OK
    assert abs(r["pose"]["theta_deg"] - 90.0) < 1e-6
    assert math.isclose(r["pose"]["y"], 1.9, abs_tol=1e-6)
