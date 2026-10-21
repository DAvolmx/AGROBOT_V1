"""El port de Python (simulador) coincide con suave_model.h, el C++ que corre en la C3."""
import math, random, subprocess, shutil, pathlib, pytest
import sim_logica_actual as S

FW = pathlib.Path(__file__).resolve().parent.parent / "firmware" / "agrobot_c3_suave"


@pytest.mark.skipif(shutil.which("g++") is None, reason="sin g++")
def test_mezcla_y_rampa_coinciden_con_el_cpp(tmp_path):
    exe = tmp_path / "t"
    subprocess.run(["g++", "-O2", "-std=c++17", "-I", str(FW), str(FW / "tests" / "test_suave.cpp"), "-o", str(exe)], check=True)
    rng = random.Random(7)
    casos, esperado = [], []
    for _ in range(3000):
        v = rng.choice([0.0, rng.uniform(-0.6, 0.6)])
        w = rng.choice([0.0, rng.uniform(-1.6, 1.6)])
        lim = (rng.choice([40, 60, 80, 100]), rng.choice([30, 50, 100]), rng.choice([40, 90, 100]), rng.choice([0, 30, 55, 70]), rng.choice([100, 130, 200]))
        casos.append("M %.6f %.6f %d %d %d %d %d" % ((v, w) + lim))
        esperado.append(S.su_mezclar(float(f"{v:.6f}") if False else v, w, *lim))
    for _ in range(3000):
        cur = rng.choice([0.0, rng.uniform(-100, 100)]); t = rng.choice([0.0, rng.uniform(-100, 100)])
        minp, acel, fre = rng.choice([0, 55, 70]), rng.choice([60, 120, 400]), rng.choice([100, 300, 800])
        casos.append("R %.6f %.6f %d %d %d" % (cur, t, minp, acel, fre))
        esperado.append(S.su_rampa(cur, t, 0.01, minp, acel, fre))
    out = subprocess.run([str(exe)], input="\n".join(casos), capture_output=True, text=True, check=True).stdout.split("\n")
    assert out[0] == "1", "los limites de fabrica del firmware son invalidos"
    out = out[1:]
    assert len([o for o in out if o]) == 6000
    malos = 0
    for linea, esp, caso in zip(out, esperado, casos):
        got = [float(x) for x in linea.split()]
        exp = esp if isinstance(esp, list) else [esp]
        # el C++ trabaja en float32 y recibe v,w con 6 decimales: tolerancia pequeña
        if not all(abs(g - e) < 2e-3 for g, e in zip(got, exp)):
            malos += 1
    assert malos == 0, f"{malos} discrepancias"


def test_propiedades_del_diseno():
    for v in (-0.5, -0.2, 0.0, 0.2, 0.5):
        for w in (-1.5, -0.5, 0.0, 0.5, 1.5):
            d = S.su_mezclar(v, w, 80, 50, 90, 55)
            assert d[0] == d[2], "las delanteras deben ir IGUAL (sin diferencial)"
            if v > 0 and w == 0:
                assert (d[1] + d[3]) / 2 >= d[0], "las traseras llevan MAS fuerza que las delanteras"
            if v == 0:
                assert d[0] == 0 and d[2] == 0, "girando en el sitio las delanteras no empujan"
            if v > 0:
                assert d[0] >= 0 and d[2] >= 0, "delanteras nunca en sentido contrario al avance"
            if v < 0:
                assert d[0] <= 0 and d[2] <= 0
            assert all(abs(x) <= 100 for x in d)
    d = S.su_mezclar(0.0, 1.0, 80, 50, 90, 55)          # pivote en el sitio
    assert d[1] == -d[3] and abs(d[1]) >= 55


def test_defectos_python_y_backend_coinciden_con_el_firmware():
    import re
    h = (FW / "suave_model.h").read_text()
    defs = {n: float(re.search(rf"float {n} = (\d+);", h).group(1)) for n in ("adel", "atras", "pivote", "minp", "acel", "frena", "tras")}
    c = S.Config()
    assert (c.su_adel, c.su_atras, c.su_pivote, c.su_minp, c.su_acel, c.su_frena, c.su_tras) == tuple(defs[n] for n in ("adel", "atras", "pivote", "minp", "acel", "frena", "tras"))
    import sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "jetson" / "backend"))
    import c3_pines
    assert {k: v[2] for k, v in c3_pines.LIMITES.items()} == {k: int(v) for k, v in defs.items()}
