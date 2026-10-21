"""El port de Python (simulador) coincide con fusion_model.h, el C++ que corre en la C3."""
import random, subprocess, shutil, pathlib, pytest
import sim_logica_actual as S

FW = pathlib.Path(__file__).resolve().parent.parent / "firmware" / "agrobot_c3_fusion"


@pytest.mark.skipif(shutil.which("g++") is None, reason="sin g++")
def test_mezcla_rampa_y_rafaga_coinciden_con_el_cpp(tmp_path):
    exe = tmp_path / "t"
    subprocess.run(["g++", "-O2", "-std=c++17", "-I", str(FW), str(FW / "tests" / "test_fusion.cpp"), "-o", str(exe)], check=True)
    rng = random.Random(11)
    casos, esperado = [], []
    for _ in range(3000):
        v = rng.choice([0.0, rng.uniform(-0.6, 0.6)]); w = rng.choice([0.0, rng.uniform(-1.6, 1.6)])
        lim = (rng.choice([40, 80, 100]), rng.choice([30, 60, 100]), rng.choice([40, 90, 100]), rng.choice([0, 30, 70]),
               rng.choice([100, 130, 200]), rng.choice([0, 30, 100]), rng.choice([1.0, 1.15, 1.3]))
        casos.append("M %.6f %.6f %d %d %d %d %d %d %.2f" % ((v, w) + lim))
        esperado.append(S.fu_mezclar(v, w, *lim))
    for _ in range(2000):
        cur = rng.choice([0.0, rng.uniform(-100, 100)]); t = rng.choice([0.0, rng.uniform(-100, 100)])
        minp, acel, fre = rng.choice([0, 30, 70]), rng.choice([60, 200, 400]), rng.choice([100, 400, 800])
        casos.append("R %.6f %.6f %d %d %d" % (cur, t, minp, acel, fre))
        esperado.append(S.su_rampa(cur, t, 0.01, minp, acel, fre))
    for _ in range(2000):
        env = rng.choice([0.0, 99.0, -99.0, rng.uniform(-100, 100)]); tms = rng.randrange(0, 100000); per = rng.choice([0, 40, 100, 250, 400])
        casos.append("B %.6f %d %d" % (env, tms, per))
        esperado.append(S.fu_rafaga(env, tms, per))
    out = subprocess.run([str(exe)], input="\n".join(casos), capture_output=True, text=True, check=True).stdout.split("\n")
    assert out[0] == "1", "los limites de fabrica del firmware son invalidos"
    out = [o for o in out[1:] if o]
    assert len(out) == len(casos)
    malos = 0
    for linea, esp in zip(out, esperado):
        got = [float(x) for x in linea.split()]
        esp = esp if isinstance(esp, list) else [esp]
        if any(abs(g - e) > 1e-3 for g, e in zip(got, esp)):
            malos += 1; print("MAL", linea, esp)
    assert malos == 0, f"{malos} casos distintos"
